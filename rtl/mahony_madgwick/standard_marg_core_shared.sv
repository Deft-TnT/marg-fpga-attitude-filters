// Created by Wang Jialin.
// See README.md for interfaces and usage.
// Shared conventional MARG core used for the Mahony and Madgwick comparison baselines.
// FILTER_MODE=0 selects Mahony PI feedback; FILTER_MODE=1 selects Madgwick gradient feedback.
// This path uses standard square-root/division normalization and is independent of SAAM/RGRSF logic.

`timescale 1ns/1ps
module standard_marg_core_shared #(
    parameter bit FILTER_MODE = 1'b0,
    parameter logic signed [31:0] MAHONY_KP_Q30 = 32'sd268_435_456,
    parameter logic signed [31:0] MAHONY_KI_Q30 = 32'sd10_737_418,
    parameter logic signed [31:0] MADGWICK_BETA_Q30 = 32'sd16_106_127
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        in_valid,
    output logic        in_ready,
    input  logic signed [31:0] ax_in, ay_in, az_in,
    input  logic signed [31:0] mx_in, my_in, mz_in,
    input  logic signed [31:0] wx_in, wy_in, wz_in,
    input  logic [31:0] dt_in,
    input  logic [31:0] config_word_in,
    output logic        out_valid,
    input  logic        out_ready,
    output logic signed [31:0] qw_out, qx_out, qy_out, qz_out,
    output logic [7:0]  status_out,
    output logic        busy
);
    import ahrs_fixed_pkg::*;

    // A sequential FSM reuses multiplication, division, square-root, and normalization operators.
    typedef enum logic [5:0] {
        IDLE,
        ACC_N_START, ACC_N_WAIT, MAG_N_START, MAG_N_WAIT,
        QP_START, QP_WAIT, ROT_CALC,
        H_START, H_WAIT, H_CALC, HSQ_START, HSQ_WAIT, HSQRT_START, HSQRT_WAIT, FIELD_CALC,
        OBS_START, OBS_WAIT, OBS_CALC, ERR_START, ERR_WAIT, ERR_CALC,
        MKP_START, MKP_WAIT, MKP_CALC, MKI_START, MKI_WAIT, MKI_CALC,
        MKIDT_START, MKIDT_WAIT, OMEGA_CALC,
        MBQ_START, MBQ_WAIT, MBQ_CALC, MGRAD_START, MGRAD_WAIT, MGRAD_CALC,
        GRAD_N_START, GRAD_N_WAIT,
        QDOT_START, QDOT_WAIT, QDOT_CALC, BETA_START, BETA_WAIT, BETA_CALC,
        DT_START, DT_WAIT, DT_CALC, Q_N_START, Q_N_WAIT, OUTPUT
    } state_t;
    state_t state;

    logic signed [31:0] ax_reg, ay_reg, az_reg, mx_reg, my_reg, mz_reg;
    logic signed [31:0] wx_reg, wy_reg, wz_reg;
    logic [31:0] dt_reg;
    logic [7:0] status_work;
    logic acc_valid, mag_valid;

    logic signed [31:0] ax_n, ay_n, az_n, mx_n, my_n, mz_n;
    logic signed [31:0] q0, q1, q2, q3;
    logic signed [31:0] ix, iy, iz;
    logic signed [31:0] ix_next, iy_next, iz_next;
    logic signed [31:0] q0_hold, q1_hold, q2_hold, q3_hold;
    logic signed [31:0] bx, bz;
    logic signed [31:0] r00, r01, r02, r10, r11, r12, r20, r21, r22;
    logic signed [31:0] hx, hy, hz;
    logic signed [31:0] mx_est, my_est, mz_est;
    logic signed [31:0] ex, ey, ez;
    logic signed [31:0] kpex, kpey, kpez;
    logic signed [31:0] omega_x, omega_y, omega_z;
    logic signed [31:0] qdot0, qdot1, qdot2, qdot3;
    logic signed [31:0] grad0, grad1, grad2, grad3;
    logic signed [31:0] q0_candidate, q1_candidate, q2_candidate, q3_candidate;
    logic signed [31:0] beta_g0, beta_g1, beta_g2, beta_g3;
    logic signed [31:0] bzq0, bzq1, bzq2, bzq3, bxq0, bxq1, bxq2, bxq3;
    logic signed [31:0] rgx, rgy, rgz, rmx, rmy, rmz;
    logic signed [31:0] jm00, jm01, jm02, jm03, jm10, jm11, jm12, jm13, jm20, jm21, jm22, jm23;

    logic [5:0] op_index;
    logic signed [31:0] product_q30 [0:31];
    logic signed [31:0] mul_a, mul_b;
    logic [5:0] mul_shift;
    logic mul_start, mul_busy, mul_done;
    logic signed [63:0] mul_product;

    logic norm_start, norm_busy, norm_done, norm_zero;
    logic [2:0] norm_dim;
    logic signed [31:0] norm_v0, norm_v1, norm_v2, norm_v3;
    logic signed [31:0] norm_o0, norm_o1, norm_o2, norm_o3;
    logic sqrt_start, sqrt_busy, sqrt_done;
    logic [31:0] sqrt_root;

    function automatic logic signed [31:0] add3(
        input logic signed [31:0] a, b, c
    );
        begin add3 = q30_add(q30_add(a,b),c); end
    endfunction
    function automatic logic signed [31:0] add6(
        input logic signed [31:0] a,b,c,d,e,f
    );
        begin add6 = q30_add(q30_add(q30_add(a,b),q30_add(c,d)),q30_add(e,f)); end
    endfunction
    function automatic logic signed [31:0] q30_to_q24(input logic signed [31:0] value);
        logic signed [63:0] wide;
        begin
            wide = $signed({{32{value[31]}}, value});
            q30_to_q24 = rshift_s64_rne(wide, 6);
        end
    endfunction

    function automatic logic signed [31:0] q30_to_q27(input logic signed [31:0] value);
        logic signed [63:0] wide;
        begin
            wide = $signed({{32{value[31]}}, value});
            q30_to_q27 = rshift_s64_rne(wide, 3);
        end
    endfunction

    function automatic logic signed [31:0] add_q24_sat(
        input logic signed [31:0] a,
        input logic signed [31:0] b
    );
        logic signed [63:0] wide;
        begin
            wide = $signed({{32{a[31]}}, a}) + $signed({{32{b[31]}}, b});
            add_q24_sat = sat_s64(wide);
        end
    endfunction

    always_comb begin
        norm_start = 1'b0; norm_dim = 3'd3;
        norm_v0 = '0; norm_v1 = '0; norm_v2 = '0; norm_v3 = '0;
        case (state)
            ACC_N_START: begin norm_start=1'b1; norm_v0=ax_reg; norm_v1=ay_reg; norm_v2=az_reg; end
            MAG_N_START: begin norm_start=1'b1; norm_v0=mx_reg; norm_v1=my_reg; norm_v2=mz_reg; end
            GRAD_N_START: begin norm_start=1'b1; norm_dim=3'd4; norm_v0=grad0; norm_v1=grad1; norm_v2=grad2; norm_v3=grad3; end
            Q_N_START: begin norm_start=1'b1; norm_dim=3'd4; norm_v0=q0_candidate; norm_v1=q1_candidate; norm_v2=q2_candidate; norm_v3=q3_candidate; end
            default: begin end
        endcase
    end

    standard_normalizer_q30_shared u_normalizer (
        .clk(clk), .rst_n(rst_n), .start(norm_start), .dim(norm_dim),
        .v0_in(norm_v0), .v1_in(norm_v1), .v2_in(norm_v2), .v3_in(norm_v3),
        .busy(norm_busy), .done(norm_done), .zero_input(norm_zero),
        .v0_out(norm_o0), .v1_out(norm_o1), .v2_out(norm_o2), .v3_out(norm_o3)
    );
    standard_sqrt_q30_seq u_field_sqrt (
        .clk(clk), .rst_n(rst_n), .start(sqrt_start),
        .u_q30(q30_add(product_q30[0], product_q30[1])),
        .busy(sqrt_busy), .done(sqrt_done), .root_q30(sqrt_root)
    );
    standard_mul_signed32_seq u_core_multiplier (
        .clk(clk), .rst_n(rst_n), .start(mul_start), .a(mul_a), .b(mul_b),
        .busy(mul_busy), .done(mul_done), .product(mul_product)
    );

    always_comb begin
        mul_start = 1'b0; mul_a = '0; mul_b = '0; mul_shift = 6'd30;
        case (state)
            QP_START: begin
                mul_start = 1'b1;
                case (op_index)
                    0: begin mul_a=q0; mul_b=q0; end 1: begin mul_a=q1; mul_b=q1; end
                    2: begin mul_a=q2; mul_b=q2; end 3: begin mul_a=q3; mul_b=q3; end
                    4: begin mul_a=q0; mul_b=q1; end 5: begin mul_a=q0; mul_b=q2; end
                    6: begin mul_a=q0; mul_b=q3; end 7: begin mul_a=q1; mul_b=q2; end
                    8: begin mul_a=q1; mul_b=q3; end default: begin mul_a=q2; mul_b=q3; end
                endcase
            end
            H_START: begin
                mul_start=1'b1;
                case (op_index)
                    0: begin mul_a=r00; mul_b=mx_n; end 1: begin mul_a=r01; mul_b=my_n; end 2: begin mul_a=r02; mul_b=mz_n; end
                    3: begin mul_a=r10; mul_b=mx_n; end 4: begin mul_a=r11; mul_b=my_n; end 5: begin mul_a=r12; mul_b=mz_n; end
                    6: begin mul_a=r20; mul_b=mx_n; end 7: begin mul_a=r21; mul_b=my_n; end default: begin mul_a=r22; mul_b=mz_n; end
                endcase
            end
            HSQ_START: begin mul_start=1'b1; if (op_index==0) begin mul_a=hx; mul_b=hx; end else begin mul_a=hy; mul_b=hy; end end
            OBS_START: begin
                mul_start=1'b1;
                case (op_index)
                    0: begin mul_a=bx; mul_b=r00; end 1: begin mul_a=bz; mul_b=r20; end
                    2: begin mul_a=bx; mul_b=r01; end 3: begin mul_a=bz; mul_b=r21; end
                    4: begin mul_a=bx; mul_b=r02; end default: begin mul_a=bz; mul_b=r22; end
                endcase
            end
            ERR_START: begin
                mul_start=1'b1;
                case (op_index)
                    0: begin mul_a=ay_n; mul_b=r22; end 1: begin mul_a=az_n; mul_b=r21; end
                    2: begin mul_a=az_n; mul_b=r20; end 3: begin mul_a=ax_n; mul_b=r22; end
                    4: begin mul_a=ax_n; mul_b=r21; end 5: begin mul_a=ay_n; mul_b=r20; end
                    6: begin mul_a=my_n; mul_b=mz_est; end 7: begin mul_a=mz_n; mul_b=my_est; end
                    8: begin mul_a=mz_n; mul_b=mx_est; end 9: begin mul_a=mx_n; mul_b=mz_est; end
                    10: begin mul_a=mx_n; mul_b=my_est; end default: begin mul_a=my_n; mul_b=mx_est; end
                endcase
            end
            MKP_START: begin mul_start=1'b1; mul_a=MAHONY_KP_Q30; if(op_index==0) mul_b=ex; else if(op_index==1) mul_b=ey; else mul_b=ez; end
            MKI_START: begin mul_start=1'b1; mul_a=MAHONY_KI_Q30; if(op_index==0) mul_b=ex; else if(op_index==1) mul_b=ey; else mul_b=ez; end
            MKIDT_START: begin mul_start=1'b1; mul_a=$signed(dt_reg); mul_b=product_q30[op_index]; end
            MBQ_START: begin
                mul_start=1'b1;
                case (op_index)
                    0: begin mul_a=bz; mul_b=q0; end 1: begin mul_a=bz; mul_b=q1; end 2: begin mul_a=bz; mul_b=q2; end 3: begin mul_a=bz; mul_b=q3; end
                    4: begin mul_a=bx; mul_b=q0; end 5: begin mul_a=bx; mul_b=q1; end 6: begin mul_a=bx; mul_b=q2; end default: begin mul_a=bx; mul_b=q3; end
                endcase
            end
            MGRAD_START: begin
                mul_start=1'b1;
                case (op_index)
                    0: begin mul_a=q30_neg(q30_double(q2)); mul_b=rgx; end 1: begin mul_a=q30_double(q3); mul_b=rgx; end
                    2: begin mul_a=q30_neg(q30_double(q0)); mul_b=rgx; end 3: begin mul_a=q30_double(q1); mul_b=rgx; end
                    4: begin mul_a=q30_double(q1); mul_b=rgy; end 5: begin mul_a=q30_double(q0); mul_b=rgy; end
                    6: begin mul_a=q30_double(q3); mul_b=rgy; end 7: begin mul_a=q30_double(q2); mul_b=rgy; end
                    8: begin mul_a=q30_double(q0); mul_b=rgz; end 9: begin mul_a=q30_neg(q30_double(q1)); mul_b=rgz; end
                    10: begin mul_a=q30_neg(q30_double(q2)); mul_b=rgz; end 11: begin mul_a=q30_double(q3); mul_b=rgz; end
                    12: begin mul_a=jm00; mul_b=rmx; end 13: begin mul_a=jm10; mul_b=rmy; end 14: begin mul_a=jm20; mul_b=rmz; end
                    15: begin mul_a=jm01; mul_b=rmx; end 16: begin mul_a=jm11; mul_b=rmy; end 17: begin mul_a=jm21; mul_b=rmz; end
                    18: begin mul_a=jm02; mul_b=rmx; end 19: begin mul_a=jm12; mul_b=rmy; end 20: begin mul_a=jm22; mul_b=rmz; end
                    21: begin mul_a=jm03; mul_b=rmx; end 22: begin mul_a=jm13; mul_b=rmy; end default: begin mul_a=jm23; mul_b=rmz; end
                endcase
            end
            QDOT_START: begin
                mul_start=1'b1; mul_shift=6'd27;
                if (FILTER_MODE) begin
                    case (op_index)
                        0: begin mul_a=q1; mul_b=wx_reg; end 1: begin mul_a=q2; mul_b=wy_reg; end 2: begin mul_a=q3; mul_b=wz_reg; end
                        3: begin mul_a=q0; mul_b=wx_reg; end 4: begin mul_a=q2; mul_b=wz_reg; end 5: begin mul_a=q3; mul_b=wy_reg; end
                        6: begin mul_a=q0; mul_b=wy_reg; end 7: begin mul_a=q1; mul_b=wz_reg; end 8: begin mul_a=q3; mul_b=wx_reg; end
                        9: begin mul_a=q0; mul_b=wz_reg; end 10: begin mul_a=q1; mul_b=wy_reg; end default: begin mul_a=q2; mul_b=wx_reg; end
                    endcase
                end else begin
                    case (op_index)
                        0: begin mul_a=q1; mul_b=omega_x; end 1: begin mul_a=q2; mul_b=omega_y; end 2: begin mul_a=q3; mul_b=omega_z; end
                        3: begin mul_a=q0; mul_b=omega_x; end 4: begin mul_a=q2; mul_b=omega_z; end 5: begin mul_a=q3; mul_b=omega_y; end
                        6: begin mul_a=q0; mul_b=omega_y; end 7: begin mul_a=q1; mul_b=omega_z; end 8: begin mul_a=q3; mul_b=omega_x; end
                        9: begin mul_a=q0; mul_b=omega_z; end 10: begin mul_a=q1; mul_b=omega_y; end default: begin mul_a=q2; mul_b=omega_x; end
                    endcase
                end
            end
            QDOT_WAIT: begin mul_shift=6'd27; end
            BETA_START: begin mul_start=1'b1; mul_a=MADGWICK_BETA_Q30; case(op_index) 0:mul_b=grad0; 1:mul_b=grad1; 2:mul_b=grad2; default:mul_b=grad3; endcase end
            DT_START: begin mul_start=1'b1; mul_shift=6'd27; mul_a=$signed(dt_reg); case(op_index) 0:mul_b=qdot0; 1:mul_b=qdot1; 2:mul_b=qdot2; default:mul_b=qdot3; endcase end
            DT_WAIT: begin mul_shift=6'd27; end
            default: begin end
        endcase
    end

    assign sqrt_start = (state == HSQRT_START);
    assign in_ready = (state == IDLE);
    assign out_valid = (state == OUTPUT);
    assign qw_out=q0_hold; assign qx_out=q1_hold; assign qy_out=q2_hold; assign qz_out=q3_hold;
    assign status_out=status_work;
    assign busy=(state != IDLE);

    always_ff @(posedge clk) begin
        integer n;
        if (!rst_n) begin
            state <= IDLE; op_index <= '0;
            ax_reg<='0; ay_reg<='0; az_reg<='0; mx_reg<='0; my_reg<='0; mz_reg<='0;
            wx_reg<='0; wy_reg<='0; wz_reg<='0; dt_reg<='0;
            status_work<='0; acc_valid<=1'b0; mag_valid<=1'b0;
            ax_n<='0; ay_n<='0; az_n<='0; mx_n<='0; my_n<='0; mz_n<='0;
            q0<=Q30_ONE; q1<='0; q2<='0; q3<='0; ix<='0; iy<='0; iz<='0;
            ix_next<='0; iy_next<='0; iz_next<='0; q0_hold<=Q30_ONE; q1_hold<='0; q2_hold<='0; q3_hold<='0;
            bx<='0; bz<='0; r00<='0; r01<='0; r02<='0; r10<='0; r11<='0; r12<='0; r20<='0; r21<='0; r22<='0;
            hx<='0; hy<='0; hz<='0; mx_est<='0; my_est<='0; mz_est<='0; ex<='0; ey<='0; ez<='0;
            kpex<='0; kpey<='0; kpez<='0; omega_x<='0; omega_y<='0; omega_z<='0;
            qdot0<='0; qdot1<='0; qdot2<='0; qdot3<='0; grad0<='0; grad1<='0; grad2<='0; grad3<='0;
            q0_candidate<='0; q1_candidate<='0; q2_candidate<='0; q3_candidate<='0;
            beta_g0<='0; beta_g1<='0; beta_g2<='0; beta_g3<='0;
            bzq0<='0; bzq1<='0; bzq2<='0; bzq3<='0; bxq0<='0; bxq1<='0; bxq2<='0; bxq3<='0;
            rgx<='0; rgy<='0; rgz<='0; rmx<='0; rmy<='0; rmz<='0;
            jm00<='0; jm01<='0; jm02<='0; jm03<='0; jm10<='0; jm11<='0; jm12<='0; jm13<='0; jm20<='0; jm21<='0; jm22<='0; jm23<='0;
            for (n=0; n<32; n=n+1) product_q30[n] <= '0;
        end else begin
            case (state)
                IDLE: if (in_valid) begin
                    ax_reg<=ax_in; ay_reg<=ay_in; az_reg<=az_in; mx_reg<=mx_in; my_reg<=my_in; mz_reg<=mz_in;
                    wx_reg<=wx_in; wy_reg<=wy_in; wz_reg<=wz_in; dt_reg<=dt_in;
                    status_work <= (dt_in == 0) ? 8'h04 : 8'h00;
                    acc_valid<=1'b0; mag_valid<=1'b0; state<=ACC_N_START;
                end
                ACC_N_START: state<=ACC_N_WAIT;
                ACC_N_WAIT: if(norm_done) begin
                    ax_n<=norm_o0; ay_n<=norm_o1; az_n<=norm_o2; acc_valid<=!norm_zero;
                    if(norm_zero) status_work[0]<=1'b1;
                    state<=MAG_N_START;
                end
                MAG_N_START: state<=MAG_N_WAIT;
                MAG_N_WAIT: if(norm_done) begin
                    mx_n<=norm_o0; my_n<=norm_o1; mz_n<=norm_o2; mag_valid<=!norm_zero;
                    if(norm_zero) status_work[1]<=1'b1;
                    op_index<=0; state<=QP_START;
                end
                QP_START: state<=QP_WAIT;
                QP_WAIT: if(mul_done) begin
                    product_q30[op_index] <= rshift_s64_rne(mul_product, mul_shift);
                    if(op_index==9) state<=ROT_CALC; else begin op_index<=op_index+1'b1; state<=QP_START; end
                end
                ROT_CALC: begin
                    r00<=q30_sub(Q30_ONE,q30_double(q30_add(product_q30[2],product_q30[3])));
                    r01<=q30_double(q30_sub(product_q30[7],product_q30[6]));
                    r02<=q30_double(q30_add(product_q30[8],product_q30[5]));
                    r10<=q30_double(q30_add(product_q30[7],product_q30[6]));
                    r11<=q30_sub(Q30_ONE,q30_double(q30_add(product_q30[1],product_q30[3])));
                    r12<=q30_double(q30_sub(product_q30[9],product_q30[4]));
                    r20<=q30_double(q30_sub(product_q30[8],product_q30[5]));
                    r21<=q30_double(q30_add(product_q30[9],product_q30[4]));
                    r22<=q30_sub(Q30_ONE,q30_double(q30_add(product_q30[1],product_q30[2])));
                    op_index<=0; state<=H_START;
                end
                H_START: state<=H_WAIT;
                H_WAIT: if(mul_done) begin
                    product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);
                    if(op_index==8) state<=H_CALC; else begin op_index<=op_index+1'b1; state<=H_START; end
                end
                H_CALC: begin
                    hx<=add3(product_q30[0],product_q30[1],product_q30[2]);
                    hy<=add3(product_q30[3],product_q30[4],product_q30[5]);
                    hz<=add3(product_q30[6],product_q30[7],product_q30[8]);
                    op_index<=0; state<=HSQ_START;
                end
                HSQ_START: state<=HSQ_WAIT;
                HSQ_WAIT: if(mul_done) begin
                    product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);
                    if(op_index==1) state<=HSQRT_START; else begin op_index<=1; state<=HSQ_START; end
                end
                HSQRT_START: state<=HSQRT_WAIT;
                HSQRT_WAIT: if(sqrt_done) begin bx<=$signed(sqrt_root); bz<=hz; op_index<=0; state<=OBS_START; end
                OBS_START: state<=OBS_WAIT;
                OBS_WAIT: if(mul_done) begin
                    product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);
                    if(op_index==5) state<=OBS_CALC; else begin op_index<=op_index+1'b1; state<=OBS_START; end
                end
                OBS_CALC: begin
                    mx_est<=q30_add(product_q30[0],product_q30[1]);
                    my_est<=q30_add(product_q30[2],product_q30[3]);
                    mz_est<=q30_add(product_q30[4],product_q30[5]);
                    op_index<=0; state<=ERR_START;
                end
                ERR_START: state<=ERR_WAIT;
                ERR_WAIT: if(mul_done) begin
                    product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);
                    if(op_index==11) state<=ERR_CALC; else begin op_index<=op_index+1'b1; state<=ERR_START; end
                end
                ERR_CALC: begin
                    ex<=q30_add(q30_sub(product_q30[0],product_q30[1]), mag_valid ? q30_sub(product_q30[6],product_q30[7]) : '0);
                    ey<=q30_add(q30_sub(product_q30[2],product_q30[3]), mag_valid ? q30_sub(product_q30[8],product_q30[9]) : '0);
                    ez<=q30_add(q30_sub(product_q30[4],product_q30[5]), mag_valid ? q30_sub(product_q30[10],product_q30[11]) : '0);
                    if (FILTER_MODE && acc_valid) begin
                        rgx<=q30_sub(r20,ax_n); rgy<=q30_sub(r21,ay_n); rgz<=q30_sub(r22,az_n);
                        rmx<=q30_sub(mx_est,mx_n); rmy<=q30_sub(my_est,my_n); rmz<=q30_sub(mz_est,mz_n);
                        op_index<=0; state<=MBQ_START;
                    end else if (!FILTER_MODE && acc_valid) begin
                        op_index<=0; state<=MKP_START;
                    end else if (!FILTER_MODE) begin
                        kpex<='0; kpey<='0; kpez<='0; ix_next<=ix; iy_next<=iy; iz_next<=iz;
                        omega_x<=wx_reg; omega_y<=wy_reg; omega_z<=wz_reg;
                        op_index<=0; state<=QDOT_START;
                    end else begin
                        grad0<='0; grad1<='0; grad2<='0; grad3<='0; op_index<=0; state<=QDOT_START;
                    end
                end
                MKP_START: state<=MKP_WAIT;
                MKP_WAIT: if(mul_done) begin
                    product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);
                    if(op_index==2) state<=MKP_CALC; else begin op_index<=op_index+1'b1; state<=MKP_START; end
                end
                MKP_CALC: begin
                    kpex<=product_q30[0]; kpey<=product_q30[1]; kpez<=product_q30[2];
                    if(MAHONY_KI_Q30 != 0) begin op_index<=0; state<=MKI_START; end
                    else begin ix_next<=ix; iy_next<=iy; iz_next<=iz; state<=OMEGA_CALC; end
                end
                MKI_START: state<=MKI_WAIT;
                MKI_WAIT: if(mul_done) begin
                    product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);
                    if(op_index==2) state<=MKI_CALC; else begin op_index<=op_index+1'b1; state<=MKI_START; end
                end
                MKI_CALC: begin op_index<=0; state<=MKIDT_START; end
                MKIDT_START: state<=MKIDT_WAIT;
                MKIDT_WAIT: if(mul_done) begin
                    product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);
                    if(op_index==2) begin
                        ix_next<=q30_add(ix,product_q30[0]);
                        iy_next<=q30_add(iy,product_q30[1]);
                        iz_next<=q30_add(iz,rshift_s64_rne(mul_product,mul_shift));
                        state<=OMEGA_CALC;
                    end else begin op_index<=op_index+1'b1; state<=MKIDT_START; end
                end
                OMEGA_CALC: begin
                    omega_x<=add_q24_sat(wx_reg,q30_to_q24(q30_add(kpex,ix_next)));
                    omega_y<=add_q24_sat(wy_reg,q30_to_q24(q30_add(kpey,iy_next)));
                    omega_z<=add_q24_sat(wz_reg,q30_to_q24(q30_add(kpez,iz_next)));
                    op_index<=0; state<=QDOT_START;
                end
                MBQ_START: state<=MBQ_WAIT;
                MBQ_WAIT: if(mul_done) begin
                    product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);
                    if(op_index==7) state<=MBQ_CALC; else begin op_index<=op_index+1'b1; state<=MBQ_START; end
                end
                MBQ_CALC: begin
                    bzq0<=product_q30[0]; bzq1<=product_q30[1]; bzq2<=product_q30[2]; bzq3<=product_q30[3];
                    bxq0<=product_q30[4]; bxq1<=product_q30[5]; bxq2<=product_q30[6]; bxq3<=product_q30[7];
                    jm00<=q30_neg(q30_double(product_q30[2])); jm01<=q30_double(product_q30[3]);
                    jm02<=q30_sub(q30_neg(q30_double(q30_double(product_q30[6]))),q30_double(product_q30[0]));
                    jm03<=q30_add(q30_neg(q30_double(q30_double(product_q30[7]))),q30_double(product_q30[1]));
                    jm10<=q30_add(q30_neg(q30_double(product_q30[7])),q30_double(product_q30[1]));
                    jm11<=q30_add(q30_double(product_q30[6]),q30_double(product_q30[0]));
                    jm12<=q30_add(q30_double(product_q30[5]),q30_double(product_q30[3]));
                    jm13<=q30_add(q30_neg(q30_double(product_q30[4])),q30_double(product_q30[2]));
                    jm20<=q30_double(product_q30[6]);
                    jm21<=q30_sub(q30_double(product_q30[7]),q30_double(q30_double(product_q30[1])));
                    jm22<=q30_sub(q30_double(product_q30[4]),q30_double(q30_double(product_q30[2])));
                    jm23<=q30_double(product_q30[5]);
                    op_index<=0; state<=MGRAD_START;
                end
                MGRAD_START: state<=MGRAD_WAIT;
                MGRAD_WAIT: if(mul_done) begin
                    product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);
                    if(op_index==23) state<=MGRAD_CALC; else begin op_index<=op_index+1'b1; state<=MGRAD_START; end
                end
                MGRAD_CALC: begin
                    grad0<=add6(add3(product_q30[0],product_q30[4],product_q30[8]), product_q30[12],product_q30[13],product_q30[14], '0,'0);
                    grad1<=add6(add3(product_q30[1],product_q30[5],product_q30[9]), product_q30[15],product_q30[16],product_q30[17], '0,'0);
                    grad2<=add6(add3(product_q30[2],product_q30[6],product_q30[10]), product_q30[18],product_q30[19],product_q30[20], '0,'0);
                    grad3<=add6(add3(product_q30[3],product_q30[7],product_q30[11]), product_q30[21],product_q30[22],product_q30[23], '0,'0);
                    state<=GRAD_N_START;
                end
                GRAD_N_START: state<=GRAD_N_WAIT;
                GRAD_N_WAIT: if(norm_done) begin
                    if(norm_zero) begin grad0<='0;grad1<='0;grad2<='0;grad3<='0;status_work[5]<=1'b1; end
                    else begin grad0<=norm_o0;grad1<=norm_o1;grad2<=norm_o2;grad3<=norm_o3; end
                    op_index<=0; state<=QDOT_START;
                end
                QDOT_START: state<=QDOT_WAIT;
                QDOT_WAIT: if(mul_done) begin
                    product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);
                    if(op_index==11) state<=QDOT_CALC; else begin op_index<=op_index+1'b1; state<=QDOT_START; end
                end
                QDOT_CALC: begin
                    qdot0<=q30_neg(q30_half(add3(product_q30[0],product_q30[1],product_q30[2])));
                    qdot1<=q30_half(q30_add(q30_sub(product_q30[3],product_q30[5]),product_q30[4]));
                    qdot2<=q30_half(q30_add(q30_sub(product_q30[6],product_q30[7]),product_q30[8]));
                    qdot3<=q30_half(q30_add(q30_sub(product_q30[9],product_q30[11]),product_q30[10]));
                    if(FILTER_MODE) begin op_index<=0; state<=BETA_START; end else begin op_index<=0; state<=DT_START; end
                end
                BETA_START: state<=BETA_WAIT;
                BETA_WAIT: if(mul_done) begin
                    product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);
                    if(op_index==3) state<=BETA_CALC; else begin op_index<=op_index+1'b1; state<=BETA_START; end
                end
                BETA_CALC: begin
                    beta_g0<=product_q30[0]; beta_g1<=product_q30[1]; beta_g2<=product_q30[2]; beta_g3<=product_q30[3];
                    qdot0<=q30_sub(qdot0,q30_to_q27(product_q30[0]));
                    qdot1<=q30_sub(qdot1,q30_to_q27(product_q30[1]));
                    qdot2<=q30_sub(qdot2,q30_to_q27(product_q30[2]));
                    qdot3<=q30_sub(qdot3,q30_to_q27(product_q30[3]));
                    op_index<=0; state<=DT_START;
                end
                DT_START: state<=DT_WAIT;
                DT_WAIT: if(mul_done) begin
                    product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);
                    if(op_index==3) state<=DT_CALC; else begin op_index<=op_index+1'b1; state<=DT_START; end
                end
                DT_CALC: begin
                    q0_candidate<=q30_add(q0,product_q30[0]); q1_candidate<=q30_add(q1,product_q30[1]); q2_candidate<=q30_add(q2,product_q30[2]); q3_candidate<=q30_add(q3,product_q30[3]);
                    state<=Q_N_START;
                end
                Q_N_START: state<=Q_N_WAIT;
                Q_N_WAIT: if(norm_done) begin
                    if(norm_zero) begin q0_hold<=q0;q1_hold<=q1;q2_hold<=q2;q3_hold<=q3;status_work[3]<=1'b1; end
                    else begin q0_hold<=norm_o0;q1_hold<=norm_o1;q2_hold<=norm_o2;q3_hold<=norm_o3; end
                    state<=OUTPUT;
                end
                OUTPUT: if(out_ready) begin
                    q0<=q0_hold;q1<=q1_hold;q2<=q2_hold;q3<=q3_hold;
                    if(!FILTER_MODE) begin ix<=ix_next;iy<=iy_next;iz<=iz_next; end
                    state<=IDLE;
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule
