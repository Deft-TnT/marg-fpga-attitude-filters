// Created by Wang Jialin.
// See README.md for interfaces and usage.

`timescale 1ns/1ps
module mekf_diag_core #(
    parameter logic signed [31:0] R_MEAS_Q30      = 32'sd322_122_547, // 0.30
    parameter logic signed [31:0] PTHETA_INIT_Q30 = 32'sd21_474_836,  // 0.02
    parameter logic signed [31:0] PBIAS_INIT_Q30  = 32'sd2_147_484,   // 0.002
    parameter logic signed [31:0] QTHETA_DT_Q30   = 32'sd107,         // 1e-7/frame
    parameter logic signed [31:0] QBIAS_DT_Q30    = 32'sd1,           // floor for 1e-10/frame
    parameter logic signed [31:0] BWX_Q30         = 32'sd858_993_459, // +0.8
    parameter logic signed [31:0] BWZ_Q30         = -32'sd644_245_094 // -0.6
) (
    input  logic clk, rst_n,
    input  logic in_valid,
    output logic in_ready,
    input  logic signed [31:0] ax_in, ay_in, az_in,
    input  logic signed [31:0] mx_in, my_in, mz_in,
    input  logic signed [31:0] wx_in, wy_in, wz_in,
    input  logic [31:0] dt_in,
    input  logic [31:0] config_word_in,
    output logic out_valid,
    input  logic out_ready,
    output logic signed [31:0] qw_out, qx_out, qy_out, qz_out,
    output logic [7:0] status_out,
    output logic busy
);
    import ahrs_fixed_pkg::*;

    typedef enum logic [5:0] {
        IDLE, AN_START, AN_WAIT, MN_START, MN_WAIT,
        QDOT_START, QDOT_WAIT, QDOT_CALC, DT_START, DT_WAIT, PRED_CALC, QP_START, QP_WAIT,
        ROT_START, ROT_WAIT, ROT_CALC, MAG_START, MAG_WAIT, ERR_START, ERR_WAIT, ERR_CALC,
        KT_START, KT_WAIT, KB_START, KB_WAIT, DELTA_START, DELTA_WAIT, DELTA_CALC,
        CORR_START, CORR_WAIT, CORR_CALC, QN_START, QN_WAIT, COV_START, COV_WAIT, COV_CALC, OUTPUT
    } state_t;
    state_t state;

    logic signed [31:0] ax_reg, ay_reg, az_reg, mx_reg, my_reg, mz_reg;
    logic signed [31:0] wx_reg, wy_reg, wz_reg;
    logic [31:0] dt_reg;
    logic [7:0] status_work;
    logic acc_valid, mag_valid;
    logic signed [31:0] ax_n, ay_n, az_n, mx_n, my_n, mz_n;

    logic signed [31:0] q0, q1, q2, q3;
    logic signed [31:0] q0_pred, q1_pred, q2_pred, q3_pred;
    logic signed [31:0] q0_hold, q1_hold, q2_hold, q3_hold;
    logic signed [31:0] b0, b1, b2, b0_next, b1_next, b2_next;
    logic signed [31:0] ptheta, pbias, ptheta_pred, pbias_pred, ktheta, kbias;
    logic signed [31:0] qdot0, qdot1, qdot2, qdot3;
    logic signed [31:0] r00, r01, r02, r10, r11, r12, r20, r21, r22;
    logic signed [31:0] mx_est, my_est, mz_est, ex, ey, ez;
    logic signed [31:0] dtheta0, dtheta1, dtheta2;
    logic signed [31:0] q0_corr, q1_corr, q2_corr, q3_corr;
    logic [5:0] op_index;
    logic signed [31:0] product_q30 [0:15];

    logic mul_start, mul_busy, mul_done;
    logic signed [31:0] mul_a, mul_b;
    logic signed [63:0] mul_product;
    logic [5:0] mul_shift;
    logic norm_start, norm_busy, norm_done, norm_zero;
    logic [2:0] norm_dim;
    logic signed [31:0] norm_v0, norm_v1, norm_v2, norm_v3;
    logic signed [31:0] norm_o0, norm_o1, norm_o2, norm_o3;
    logic div_start, div_busy, div_done, div_zero;
    logic [63:0] div_dividend, div_quotient;
    logic [31:0] div_divisor;

    function automatic logic signed [31:0] add3(input logic signed [31:0] a,b,c);
        begin add3=q30_add(q30_add(a,b),c); end
    endfunction
    function automatic logic signed [31:0] gyro_q30(input logic signed [31:0] x);
        logic signed [63:0] wide;
        begin wide=$signed({{32{x[31]}},x}) <<< 6; gyro_q30=sat_s64(wide); end
    endfunction
    function automatic logic signed [31:0] omega_x(); begin omega_x=q30_sub(gyro_q30(wx_reg),b0); end endfunction
    function automatic logic signed [31:0] omega_y(); begin omega_y=q30_sub(gyro_q30(wy_reg),b1); end endfunction
    function automatic logic signed [31:0] omega_z(); begin omega_z=q30_sub(gyro_q30(wz_reg),b2); end endfunction

    always_comb begin
        norm_start=1'b0; norm_dim=3'd3; norm_v0='0; norm_v1='0; norm_v2='0; norm_v3='0;
        case(state)
            AN_START: begin norm_start=1'b1; norm_v0=ax_reg; norm_v1=ay_reg; norm_v2=az_reg; end
            MN_START: begin norm_start=1'b1; norm_v0=mx_reg; norm_v1=my_reg; norm_v2=mz_reg; end
            QP_START: begin norm_start=1'b1; norm_dim=3'd4; norm_v0=q0_corr; norm_v1=q1_corr; norm_v2=q2_corr; norm_v3=q3_corr; end
            QN_START: begin norm_start=1'b1; norm_dim=3'd4; norm_v0=q0_corr; norm_v1=q1_corr; norm_v2=q2_corr; norm_v3=q3_corr; end
            default: begin end
        endcase
        div_start=(state==KT_START)||(state==KB_START);
        div_divisor=q30_add(ptheta_pred,R_MEAS_Q30);
        div_dividend=(state==KT_START) ? {2'b00,ptheta_pred,30'b0} : {2'b00,pbias_pred,30'b0};
    end

    standard_normalizer_q30_shared u_norm (
        .clk(clk),.rst_n(rst_n),.start(norm_start),.dim(norm_dim),.v0_in(norm_v0),.v1_in(norm_v1),.v2_in(norm_v2),.v3_in(norm_v3),
        .busy(norm_busy),.done(norm_done),.zero_input(norm_zero),.v0_out(norm_o0),.v1_out(norm_o1),.v2_out(norm_o2),.v3_out(norm_o3)
    );
    standard_mul_signed32_seq u_multiplier(.clk(clk),.rst_n(rst_n),.start(mul_start),.a(mul_a),.b(mul_b),.busy(mul_busy),.done(mul_done),.product(mul_product));
    standard_div_u64_u32_seq u_div(.clk(clk),.rst_n(rst_n),.start(div_start),.dividend(div_dividend),.divisor(div_divisor),.busy(div_busy),.done(div_done),.div_by_zero(div_zero),.quotient(div_quotient));

    always_comb begin
        mul_start=1'b0; mul_a='0; mul_b='0; mul_shift=6'd30;
        case(state)
            QDOT_START: begin
                mul_start=1'b1;
                case(op_index)
                    0:begin mul_a=q1;mul_b=omega_x();end 1:begin mul_a=q2;mul_b=omega_y();end 2:begin mul_a=q3;mul_b=omega_z();end
                    3:begin mul_a=q0;mul_b=omega_x();end 4:begin mul_a=q2;mul_b=omega_z();end 5:begin mul_a=q3;mul_b=omega_y();end
                    6:begin mul_a=q0;mul_b=omega_y();end 7:begin mul_a=q1;mul_b=omega_z();end 8:begin mul_a=q3;mul_b=omega_x();end
                    9:begin mul_a=q0;mul_b=omega_z();end 10:begin mul_a=q1;mul_b=omega_y();end default:begin mul_a=q2;mul_b=omega_x();end
                endcase
            end
            DT_START: begin mul_start=1'b1; mul_a=$signed(dt_reg); case(op_index) 0:mul_b=qdot0;1:mul_b=qdot1;2:mul_b=qdot2;default:mul_b=qdot3;endcase end
            ROT_START: begin
                mul_start=1'b1;
                case(op_index)
                    0:begin mul_a=q0_pred;mul_b=q0_pred;end 1:begin mul_a=q1_pred;mul_b=q1_pred;end 2:begin mul_a=q2_pred;mul_b=q2_pred;end 3:begin mul_a=q3_pred;mul_b=q3_pred;end
                    4:begin mul_a=q0_pred;mul_b=q1_pred;end 5:begin mul_a=q0_pred;mul_b=q2_pred;end 6:begin mul_a=q0_pred;mul_b=q3_pred;end 7:begin mul_a=q1_pred;mul_b=q2_pred;end 8:begin mul_a=q1_pred;mul_b=q3_pred;end default:begin mul_a=q2_pred;mul_b=q3_pred;end
                endcase
            end
            MAG_START: begin
                mul_start=1'b1;
                case(op_index)
                    0:begin mul_a=BWX_Q30;mul_b=r00;end 1:begin mul_a=BWZ_Q30;mul_b=r20;end
                    2:begin mul_a=BWX_Q30;mul_b=r01;end 3:begin mul_a=BWZ_Q30;mul_b=r21;end
                    4:begin mul_a=BWX_Q30;mul_b=r02;end default:begin mul_a=BWZ_Q30;mul_b=r22;end
                endcase
            end
            ERR_START: begin
                mul_start=1'b1;
                case(op_index)
                    0:begin mul_a=ay_n;mul_b=r22;end 1:begin mul_a=az_n;mul_b=r21;end 2:begin mul_a=az_n;mul_b=r20;end
                    3:begin mul_a=ax_n;mul_b=r22;end 4:begin mul_a=ax_n;mul_b=r21;end 5:begin mul_a=ay_n;mul_b=r20;end
                    6:begin mul_a=my_n;mul_b=mz_est;end 7:begin mul_a=mz_n;mul_b=my_est;end 8:begin mul_a=mz_n;mul_b=mx_est;end
                    9:begin mul_a=mx_n;mul_b=mz_est;end 10:begin mul_a=mx_n;mul_b=my_est;end default:begin mul_a=my_n;mul_b=mx_est;end
                endcase
            end
            DELTA_START: begin
                mul_start=1'b1;
                case(op_index)
                    0:begin mul_a=ktheta;mul_b=ex;end 1:begin mul_a=ktheta;mul_b=ey;end 2:begin mul_a=ktheta;mul_b=ez;end
                    3:begin mul_a=kbias;mul_b=ex;end 4:begin mul_a=kbias;mul_b=ey;end default:begin mul_a=kbias;mul_b=ez;end
                endcase
            end
            CORR_START: begin
                mul_start=1'b1;
                case(op_index)
                    0:begin mul_a=q1_pred;mul_b=q30_half(dtheta0);end 1:begin mul_a=q2_pred;mul_b=q30_half(dtheta1);end 2:begin mul_a=q3_pred;mul_b=q30_half(dtheta2);end
                    3:begin mul_a=q0_pred;mul_b=q30_half(dtheta0);end 4:begin mul_a=q2_pred;mul_b=q30_half(dtheta2);end 5:begin mul_a=q3_pred;mul_b=q30_half(dtheta1);end
                    6:begin mul_a=q0_pred;mul_b=q30_half(dtheta1);end 7:begin mul_a=q1_pred;mul_b=q30_half(dtheta2);end 8:begin mul_a=q3_pred;mul_b=q30_half(dtheta0);end
                    9:begin mul_a=q0_pred;mul_b=q30_half(dtheta2);end 10:begin mul_a=q1_pred;mul_b=q30_half(dtheta1);end default:begin mul_a=q2_pred;mul_b=q30_half(dtheta0);end
                endcase
            end
            COV_START: begin mul_start=1'b1; if(op_index==0) begin mul_a=q30_sub(Q30_ONE,ktheta);mul_b=ptheta_pred;end else begin mul_a=q30_sub(Q30_ONE,kbias);mul_b=pbias_pred;end end
            default: begin end
        endcase
    end

    assign in_ready=(state==IDLE); assign out_valid=(state==OUTPUT); assign busy=(state!=IDLE);
    assign qw_out=q0_hold;assign qx_out=q1_hold;assign qy_out=q2_hold;assign qz_out=q3_hold;assign status_out=status_work;

    always_ff @(posedge clk) begin
        integer i;
        if(!rst_n) begin
            state<=IDLE;op_index<='0;ax_reg<='0;ay_reg<='0;az_reg<='0;mx_reg<='0;my_reg<='0;mz_reg<='0;wx_reg<='0;wy_reg<='0;wz_reg<='0;dt_reg<='0;status_work<='0;acc_valid<=0;mag_valid<=0;
            ax_n<='0;ay_n<='0;az_n<='0;mx_n<='0;my_n<='0;mz_n<='0;q0<=Q30_ONE;q1<='0;q2<='0;q3<='0;q0_pred<=Q30_ONE;q1_pred<='0;q2_pred<='0;q3_pred<='0;q0_hold<=Q30_ONE;q1_hold<='0;q2_hold<='0;q3_hold<='0;
            b0<='0;b1<='0;b2<='0;b0_next<='0;b1_next<='0;b2_next<='0;ptheta<=PTHETA_INIT_Q30;pbias<=PBIAS_INIT_Q30;ptheta_pred<=PTHETA_INIT_Q30;pbias_pred<=PBIAS_INIT_Q30;ktheta<='0;kbias<='0;
            qdot0<='0;qdot1<='0;qdot2<='0;qdot3<='0;r00<='0;r01<='0;r02<='0;r10<='0;r11<='0;r12<='0;r20<='0;r21<='0;r22<='0;mx_est<='0;my_est<='0;mz_est<='0;ex<='0;ey<='0;ez<='0;dtheta0<='0;dtheta1<='0;dtheta2<='0;q0_corr<='0;q1_corr<='0;q2_corr<='0;q3_corr<='0;
            for(i=0;i<16;i=i+1) product_q30[i]<='0;
        end else case(state)
            IDLE: if(in_valid) begin ax_reg<=ax_in;ay_reg<=ay_in;az_reg<=az_in;mx_reg<=mx_in;my_reg<=my_in;mz_reg<=mz_in;wx_reg<=wx_in;wy_reg<=wy_in;wz_reg<=wz_in;dt_reg<=dt_in;status_work<=(dt_in==0)?8'h04:8'h00;acc_valid<=0;mag_valid<=0;state<=AN_START;end
            AN_START:state<=AN_WAIT;
            AN_WAIT:if(norm_done)begin ax_n<=norm_o0;ay_n<=norm_o1;az_n<=norm_o2;acc_valid<=!norm_zero;if(norm_zero)status_work[0]<=1;state<=MN_START;end
            MN_START:state<=MN_WAIT;
            MN_WAIT:if(norm_done)begin mx_n<=norm_o0;my_n<=norm_o1;mz_n<=norm_o2;mag_valid<=!norm_zero;if(norm_zero)status_work[1]<=1;op_index<=0;state<=QDOT_START;end
            QDOT_START:state<=QDOT_WAIT;
            QDOT_WAIT:if(mul_done)begin product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);if(op_index==11)state<=QDOT_CALC;else begin op_index<=op_index+1;state<=QDOT_START;end end
            QDOT_CALC:begin qdot0<=q30_neg(q30_half(add3(product_q30[0],product_q30[1],product_q30[2])));qdot1<=q30_half(q30_add(q30_sub(product_q30[3],product_q30[5]),product_q30[4]));qdot2<=q30_half(q30_add(q30_sub(product_q30[6],product_q30[7]),product_q30[8]));qdot3<=q30_half(q30_add(q30_sub(product_q30[9],product_q30[11]),product_q30[10]));op_index<=0;state<=DT_START;end
            DT_START:state<=DT_WAIT;
            DT_WAIT:if(mul_done)begin product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);if(op_index==3)state<=PRED_CALC;else begin op_index<=op_index+1;state<=DT_START;end end
            PRED_CALC:begin q0_corr<=q30_add(q0,product_q30[0]);q1_corr<=q30_add(q1,product_q30[1]);q2_corr<=q30_add(q2,product_q30[2]);q3_corr<=q30_add(q3,product_q30[3]);state<=QP_START;end
            QP_START:state<=QP_WAIT;
            QP_WAIT:if(norm_done)begin if(norm_zero)begin q0_pred<=q0;q1_pred<=q1;q2_pred<=q2;q3_pred<=q3;status_work[3]<=1;end else begin q0_pred<=norm_o0;q1_pred<=norm_o1;q2_pred<=norm_o2;q3_pred<=norm_o3;end op_index<=0;state<=ROT_START;end
            ROT_START:state<=ROT_WAIT;
            ROT_WAIT:if(mul_done)begin product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);if(op_index==9)state<=ROT_CALC;else begin op_index<=op_index+1;state<=ROT_START;end end
            ROT_CALC:begin r00<=q30_sub(Q30_ONE,q30_double(q30_add(product_q30[2],product_q30[3])));r01<=q30_double(q30_sub(product_q30[7],product_q30[6]));r02<=q30_double(q30_add(product_q30[8],product_q30[5]));r10<=q30_double(q30_add(product_q30[7],product_q30[6]));r11<=q30_sub(Q30_ONE,q30_double(q30_add(product_q30[1],product_q30[3])));r12<=q30_double(q30_sub(product_q30[9],product_q30[4]));r20<=q30_double(q30_sub(product_q30[8],product_q30[5]));r21<=q30_double(q30_add(product_q30[9],product_q30[4]));r22<=q30_sub(Q30_ONE,q30_double(q30_add(product_q30[1],product_q30[2])));op_index<=0;state<=MAG_START;end
            MAG_START:state<=MAG_WAIT;
            MAG_WAIT:if(mul_done)begin product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);if(op_index==5)begin mx_est<=q30_add(product_q30[0],product_q30[1]);my_est<=q30_add(product_q30[2],product_q30[3]);mz_est<=q30_add(product_q30[4],rshift_s64_rne(mul_product,mul_shift));op_index<=0;state<=ERR_START;end else begin op_index<=op_index+1;state<=MAG_START;end end
            ERR_START:state<=ERR_WAIT;
            ERR_WAIT:if(mul_done)begin product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);if(op_index==11)state<=ERR_CALC;else begin op_index<=op_index+1;state<=ERR_START;end end
            ERR_CALC:begin ex<=q30_add(q30_sub(product_q30[0],product_q30[1]),mag_valid?q30_sub(product_q30[6],product_q30[7]):'0);ey<=q30_add(q30_sub(product_q30[2],product_q30[3]),mag_valid?q30_sub(product_q30[8],product_q30[9]):'0);ez<=q30_add(q30_sub(product_q30[4],product_q30[5]),mag_valid?q30_sub(product_q30[10],product_q30[11]):'0);ptheta_pred<=q30_add(ptheta,QTHETA_DT_Q30);pbias_pred<=q30_add(pbias,QBIAS_DT_Q30);state<=KT_START;end
            KT_START:state<=KT_WAIT;
            KT_WAIT:if(div_done)begin if(div_zero)begin ktheta<='0;status_work[4]<=1;end else ktheta<=signed_quotient_q30(div_quotient,1'b0);state<=KB_START;end
            KB_START:state<=KB_WAIT;
            KB_WAIT:if(div_done)begin if(div_zero)begin kbias<='0;status_work[4]<=1;end else kbias<=signed_quotient_q30(div_quotient,1'b0);op_index<=0;state<=DELTA_START;end
            DELTA_START:state<=DELTA_WAIT;
            DELTA_WAIT:if(mul_done)begin product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);if(op_index==5)state<=DELTA_CALC;else begin op_index<=op_index+1;state<=DELTA_START;end end
            DELTA_CALC:begin dtheta0<=product_q30[0];dtheta1<=product_q30[1];dtheta2<=product_q30[2];b0_next<=q30_add(b0,product_q30[3]);b1_next<=q30_add(b1,product_q30[4]);b2_next<=q30_add(b2,product_q30[5]);op_index<=0;state<=CORR_START;end
            CORR_START:state<=CORR_WAIT;
            CORR_WAIT:if(mul_done)begin product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);if(op_index==11)state<=CORR_CALC;else begin op_index<=op_index+1;state<=CORR_START;end end
            CORR_CALC:begin q0_corr<=q30_sub(q0_pred,add3(product_q30[0],product_q30[1],product_q30[2]));q1_corr<=q30_add(q1_pred,q30_add(q30_sub(product_q30[3],product_q30[5]),product_q30[4]));q2_corr<=q30_add(q2_pred,q30_add(q30_sub(product_q30[6],product_q30[7]),product_q30[8]));q3_corr<=q30_add(q3_pred,q30_add(q30_sub(product_q30[9],product_q30[11]),product_q30[10]));state<=QN_START;end
            QN_START:state<=QN_WAIT;
            QN_WAIT:if(norm_done)begin if(norm_zero)begin q0_hold<=q0_pred;q1_hold<=q1_pred;q2_hold<=q2_pred;q3_hold<=q3_pred;status_work[3]<=1;end else begin q0_hold<=norm_o0;q1_hold<=norm_o1;q2_hold<=norm_o2;q3_hold<=norm_o3;end op_index<=0;state<=COV_START;end
            COV_START:state<=COV_WAIT;
            COV_WAIT:if(mul_done)begin product_q30[op_index]<=rshift_s64_rne(mul_product,mul_shift);if(op_index==1)state<=COV_CALC;else begin op_index<=1;state<=COV_START;end end
            COV_CALC:begin ptheta<=product_q30[0];pbias<=product_q30[1];state<=OUTPUT;end
            OUTPUT:if(out_ready)begin q0<=q0_hold;q1<=q1_hold;q2<=q2_hold;q3<=q3_hold;b0<=b0_next;b1<=b1_next;b2<=b2_next;state<=IDLE;end
            default:state<=IDLE;
        endcase
    end
endmodule
