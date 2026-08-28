// Created by Wang Jialin.
// See README.md for interfaces and usage.

`timescale 1ns/1ps
module standard_marg_core #(
    parameter bit FILTER_MODE = 1'b0, // 0: Mahony, 1: Madgwick
    parameter logic signed [31:0] MAHONY_KP_Q30 = 32'sd268_435_456, // 0.25
    parameter logic signed [31:0] MAHONY_KI_Q30 = 32'sd10_737_418,  // 0.01
    parameter logic signed [31:0] MADGWICK_BETA_Q30 = 32'sd16_106_127 // 0.015
) (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        in_valid,
    output logic        in_ready,
    input  logic signed [31:0] ax_in,
    input  logic signed [31:0] ay_in,
    input  logic signed [31:0] az_in,
    input  logic signed [31:0] mx_in,
    input  logic signed [31:0] my_in,
    input  logic signed [31:0] mz_in,
    input  logic signed [31:0] wx_in,
    input  logic signed [31:0] wy_in,
    input  logic signed [31:0] wz_in,
    input  logic        [31:0] dt_in,
    input  logic        [31:0] config_word_in,
    output logic        out_valid,
    input  logic        out_ready,
    output logic signed [31:0] qw_out,
    output logic signed [31:0] qx_out,
    output logic signed [31:0] qy_out,
    output logic signed [31:0] qz_out,
    output logic [7:0]  status_out,
    output logic        busy
);
    import ahrs_fixed_pkg::*;

    typedef enum logic [3:0] {
        IDLE, ACC_START, ACC_WAIT, MAG_START, MAG_WAIT,
        FIELD_START, FIELD_WAIT, CALC_PRE, GRAD_START, GRAD_WAIT,
        CALC_Q, Q_START, Q_WAIT, OUTPUT
    } state_t;
    state_t state;

    logic signed [31:0] ax_reg, ay_reg, az_reg, mx_reg, my_reg, mz_reg;
    logic signed [31:0] wx_reg, wy_reg, wz_reg;
    logic [31:0] dt_reg, config_word_reg;
    logic acc_valid, mag_valid;
    logic [7:0] status_work;

    logic signed [31:0] ax_norm, ay_norm, az_norm, mx_norm, my_norm, mz_norm;
    logic signed [31:0] q0_state, q1_state, q2_state, q3_state;
    logic signed [31:0] int_x_state, int_y_state, int_z_state;
    logic signed [31:0] field_bx, field_bz;
    logic signed [31:0] grad0_reg, grad1_reg, grad2_reg, grad3_reg;
    logic signed [31:0] q0_candidate, q1_candidate, q2_candidate, q3_candidate;
    logic signed [31:0] int_x_candidate, int_y_candidate, int_z_candidate;
    logic signed [31:0] q0_hold, q1_hold, q2_hold, q3_hold;

    logic norm_start, norm_busy, norm_done, norm_zero;
    logic [2:0] norm_dim;
    logic signed [31:0] norm_v0, norm_v1, norm_v2, norm_v3;
    logic signed [31:0] norm_o0, norm_o1, norm_o2, norm_o3;
    logic field_sqrt_start, field_sqrt_busy, field_sqrt_done;
    logic [31:0] field_sqrt_input, field_sqrt_root;

    logic signed [31:0] q0q0, q1q1, q2q2, q3q3, q0q1, q0q2, q0q3, q1q2, q1q3, q2q3;
    logic signed [31:0] r00, r01, r02, r10, r11, r12, r20, r21, r22;
    logic signed [31:0] hx_comb, hy_comb, hz_comb, hxy_norm2;
    logic signed [31:0] mx_est, my_est, mz_est;
    logic signed [31:0] err_x, err_y, err_z;
    logic signed [31:0] int_x_next, int_y_next, int_z_next;
    logic signed [31:0] gyro_dot0, gyro_dot1, gyro_dot2, gyro_dot3;
    logic signed [31:0] corr_dot0, corr_dot1, corr_dot2, corr_dot3;
    logic signed [31:0] mahony_q0_next, mahony_q1_next, mahony_q2_next, mahony_q3_next;
    logic signed [31:0] grad0_raw, grad1_raw, grad2_raw, grad3_raw;
    logic signed [31:0] madgwick_q0_next, madgwick_q1_next, madgwick_q2_next, madgwick_q3_next;
    logic signed [31:0] rgx, rgy, rgz, rmx, rmy, rmz;
    logic signed [31:0] jg00, jg01, jg02, jg03, jg10, jg11, jg12, jg13, jg20, jg21, jg22, jg23;
    logic signed [31:0] jm00, jm01, jm02, jm03, jm10, jm11, jm12, jm13, jm20, jm21, jm22, jm23;

    function automatic logic signed [31:0] sum3_q30(
        input logic signed [31:0] a,
        input logic signed [31:0] b,
        input logic signed [31:0] c
    );
        begin
            sum3_q30 = q30_add(q30_add(a, b), c);
        end
    endfunction

    function automatic logic signed [31:0] sum6_q30(
        input logic signed [31:0] a,
        input logic signed [31:0] b,
        input logic signed [31:0] c,
        input logic signed [31:0] d,
        input logic signed [31:0] e,
        input logic signed [31:0] f
    );
        begin
            sum6_q30 = q30_add(q30_add(q30_add(a, b), q30_add(c, d)), q30_add(e, f));
        end
    endfunction

    always_comb begin
        norm_start = 1'b0;
        norm_dim = 3'd3;
        norm_v0 = '0; norm_v1 = '0; norm_v2 = '0; norm_v3 = '0;
        case (state)
            ACC_START: begin
                norm_start = 1'b1;
                norm_v0 = ax_reg; norm_v1 = ay_reg; norm_v2 = az_reg;
            end
            MAG_START: begin
                norm_start = 1'b1;
                norm_v0 = mx_reg; norm_v1 = my_reg; norm_v2 = mz_reg;
            end
            GRAD_START: begin
                norm_start = 1'b1;
                norm_dim = 3'd4;
                norm_v0 = grad0_raw; norm_v1 = grad1_raw; norm_v2 = grad2_raw; norm_v3 = grad3_raw;
            end
            Q_START: begin
                norm_start = 1'b1;
                norm_dim = 3'd4;
                norm_v0 = q0_candidate; norm_v1 = q1_candidate;
                norm_v2 = q2_candidate; norm_v3 = q3_candidate;
            end
            default: begin end
        endcase
        field_sqrt_start = (state == FIELD_START) && mag_valid;
        field_sqrt_input = hxy_norm2[31] ? 32'd0 : hxy_norm2;
    end

    standard_normalizer_q30 u_standard_normalizer (
        .clk(clk), .rst_n(rst_n), .start(norm_start), .dim(norm_dim),
        .v0_in(norm_v0), .v1_in(norm_v1), .v2_in(norm_v2), .v3_in(norm_v3),
        .busy(norm_busy), .done(norm_done), .zero_input(norm_zero),
        .v0_out(norm_o0), .v1_out(norm_o1), .v2_out(norm_o2), .v3_out(norm_o3)
    );
    standard_sqrt_q30_seq u_field_sqrt (
        .clk(clk), .rst_n(rst_n), .start(field_sqrt_start), .u_q30(field_sqrt_input),
        .busy(field_sqrt_busy), .done(field_sqrt_done), .root_q30(field_sqrt_root)
    );

    always_comb begin
        q0q0 = q30_mul(q0_state, q0_state); q1q1 = q30_mul(q1_state, q1_state);
        q2q2 = q30_mul(q2_state, q2_state); q3q3 = q30_mul(q3_state, q3_state);
        q0q1 = q30_mul(q0_state, q1_state); q0q2 = q30_mul(q0_state, q2_state);
        q0q3 = q30_mul(q0_state, q3_state); q1q2 = q30_mul(q1_state, q2_state);
        q1q3 = q30_mul(q1_state, q3_state); q2q3 = q30_mul(q2_state, q3_state);

        r00 = q30_sub(Q30_ONE, q30_double(q30_add(q2q2, q3q3)));
        r01 = q30_double(q30_sub(q1q2, q0q3));
        r02 = q30_double(q30_add(q1q3, q0q2));
        r10 = q30_double(q30_add(q1q2, q0q3));
        r11 = q30_sub(Q30_ONE, q30_double(q30_add(q1q1, q3q3)));
        r12 = q30_double(q30_sub(q2q3, q0q1));
        r20 = q30_double(q30_sub(q1q3, q0q2));
        r21 = q30_double(q30_add(q2q3, q0q1));
        r22 = q30_sub(Q30_ONE, q30_double(q30_add(q1q1, q2q2)));

        hx_comb = sum3_q30(q30_mul(r00, mx_norm), q30_mul(r01, my_norm), q30_mul(r02, mz_norm));
        hy_comb = sum3_q30(q30_mul(r10, mx_norm), q30_mul(r11, my_norm), q30_mul(r12, mz_norm));
        hz_comb = sum3_q30(q30_mul(r20, mx_norm), q30_mul(r21, my_norm), q30_mul(r22, mz_norm));
        hxy_norm2 = q30_add(q30_mul(hx_comb, hx_comb), q30_mul(hy_comb, hy_comb));

        mx_est = q30_add(q30_mul(field_bx, r00), q30_mul(field_bz, r20));
        my_est = q30_add(q30_mul(field_bx, r01), q30_mul(field_bz, r21));
        mz_est = q30_add(q30_mul(field_bx, r02), q30_mul(field_bz, r22));

        err_x = q30_sub(q30_mul(ay_norm, r22), q30_mul(az_norm, r21));
        err_y = q30_sub(q30_mul(az_norm, r20), q30_mul(ax_norm, r22));
        err_z = q30_sub(q30_mul(ax_norm, r21), q30_mul(ay_norm, r20));
        if (mag_valid) begin
            err_x = q30_add(err_x, q30_sub(q30_mul(my_norm, mz_est), q30_mul(mz_norm, my_est)));
            err_y = q30_add(err_y, q30_sub(q30_mul(mz_norm, mx_est), q30_mul(mx_norm, mz_est)));
            err_z = q30_add(err_z, q30_sub(q30_mul(mx_norm, my_est), q30_mul(my_norm, mx_est)));
        end

        if (MAHONY_KI_Q30 == 0) begin
            int_x_next = int_x_state; int_y_next = int_y_state; int_z_next = int_z_state;
        end else begin
            int_x_next = q30_add(int_x_state, q30_mul($signed(dt_reg), q30_mul(MAHONY_KI_Q30, err_x)));
            int_y_next = q30_add(int_y_state, q30_mul($signed(dt_reg), q30_mul(MAHONY_KI_Q30, err_y)));
            int_z_next = q30_add(int_z_state, q30_mul($signed(dt_reg), q30_mul(MAHONY_KI_Q30, err_z)));
        end

        gyro_dot0 = q30_neg(q30_half(sum3_q30(q30_mul_q24(q1_state, wx_reg), q30_mul_q24(q2_state, wy_reg), q30_mul_q24(q3_state, wz_reg))));
        gyro_dot1 = q30_half(q30_add(q30_sub(q30_mul_q24(q0_state, wx_reg), q30_mul_q24(q3_state, wy_reg)), q30_mul_q24(q2_state, wz_reg)));
        gyro_dot2 = q30_half(q30_add(q30_sub(q30_mul_q24(q0_state, wy_reg), q30_mul_q24(q1_state, wz_reg)), q30_mul_q24(q3_state, wx_reg)));
        gyro_dot3 = q30_half(q30_add(q30_sub(q30_mul_q24(q0_state, wz_reg), q30_mul_q24(q2_state, wx_reg)), q30_mul_q24(q1_state, wy_reg)));

        corr_dot0 = q30_neg(q30_half(sum3_q30(
            q30_mul(q1_state, q30_add(q30_mul(MAHONY_KP_Q30, err_x), int_x_next)),
            q30_mul(q2_state, q30_add(q30_mul(MAHONY_KP_Q30, err_y), int_y_next)),
            q30_mul(q3_state, q30_add(q30_mul(MAHONY_KP_Q30, err_z), int_z_next)) )));
        corr_dot1 = q30_half(q30_add(q30_sub(
            q30_mul(q0_state, q30_add(q30_mul(MAHONY_KP_Q30, err_x), int_x_next)),
            q30_mul(q3_state, q30_add(q30_mul(MAHONY_KP_Q30, err_y), int_y_next))),
            q30_mul(q2_state, q30_add(q30_mul(MAHONY_KP_Q30, err_z), int_z_next))));
        corr_dot2 = q30_half(q30_add(q30_sub(
            q30_mul(q0_state, q30_add(q30_mul(MAHONY_KP_Q30, err_y), int_y_next)),
            q30_mul(q1_state, q30_add(q30_mul(MAHONY_KP_Q30, err_z), int_z_next))),
            q30_mul(q3_state, q30_add(q30_mul(MAHONY_KP_Q30, err_x), int_x_next))));
        corr_dot3 = q30_half(q30_add(q30_sub(
            q30_mul(q0_state, q30_add(q30_mul(MAHONY_KP_Q30, err_z), int_z_next)),
            q30_mul(q2_state, q30_add(q30_mul(MAHONY_KP_Q30, err_x), int_x_next))),
            q30_mul(q1_state, q30_add(q30_mul(MAHONY_KP_Q30, err_y), int_y_next))));

        mahony_q0_next = q30_add(q0_state, q30_mul($signed(dt_reg), q30_add(gyro_dot0, corr_dot0)));
        mahony_q1_next = q30_add(q1_state, q30_mul($signed(dt_reg), q30_add(gyro_dot1, corr_dot1)));
        mahony_q2_next = q30_add(q2_state, q30_mul($signed(dt_reg), q30_add(gyro_dot2, corr_dot2)));
        mahony_q3_next = q30_add(q3_state, q30_mul($signed(dt_reg), q30_add(gyro_dot3, corr_dot3)));

        rgx = q30_sub(r20, ax_norm); rgy = q30_sub(r21, ay_norm); rgz = q30_sub(r22, az_norm);
        rmx = q30_sub(mx_est, mx_norm); rmy = q30_sub(my_est, my_norm); rmz = q30_sub(mz_est, mz_norm);
        jg00 = q30_neg(q30_double(q2_state)); jg01 = q30_double(q3_state);
        jg02 = q30_neg(q30_double(q0_state)); jg03 = q30_double(q1_state);
        jg10 = q30_double(q1_state); jg11 = q30_double(q0_state);
        jg12 = q30_double(q3_state); jg13 = q30_double(q2_state);
        jg20 = q30_double(q0_state); jg21 = q30_neg(q30_double(q1_state));
        jg22 = q30_neg(q30_double(q2_state)); jg23 = q30_double(q3_state);
        jm00 = q30_neg(q30_mul(q30_double(field_bz), q2_state));
        jm01 = q30_mul(q30_double(field_bz), q3_state);
        jm02 = q30_sub(q30_neg(q30_mul(q30_double(q30_double(field_bx)), q2_state)), q30_mul(q30_double(field_bz), q0_state));
        jm03 = q30_add(q30_neg(q30_mul(q30_double(q30_double(field_bx)), q3_state)), q30_mul(q30_double(field_bz), q1_state));
        jm10 = q30_add(q30_neg(q30_mul(q30_double(field_bx), q3_state)), q30_mul(q30_double(field_bz), q1_state));
        jm11 = q30_add(q30_mul(q30_double(field_bx), q2_state), q30_mul(q30_double(field_bz), q0_state));
        jm12 = q30_add(q30_mul(q30_double(field_bx), q1_state), q30_mul(q30_double(field_bz), q3_state));
        jm13 = q30_add(q30_neg(q30_mul(q30_double(field_bx), q0_state)), q30_mul(q30_double(field_bz), q2_state));
        jm20 = q30_mul(q30_double(field_bx), q2_state);
        jm21 = q30_sub(q30_mul(q30_double(field_bx), q3_state), q30_mul(q30_double(q30_double(field_bz)), q1_state));
        jm22 = q30_sub(q30_mul(q30_double(field_bx), q0_state), q30_mul(q30_double(q30_double(field_bz)), q2_state));
        jm23 = q30_mul(q30_double(field_bx), q1_state);
        grad0_raw = sum3_q30(q30_mul(jg00, rgx), q30_mul(jg10, rgy), q30_mul(jg20, rgz));
        grad1_raw = sum3_q30(q30_mul(jg01, rgx), q30_mul(jg11, rgy), q30_mul(jg21, rgz));
        grad2_raw = sum3_q30(q30_mul(jg02, rgx), q30_mul(jg12, rgy), q30_mul(jg22, rgz));
        grad3_raw = sum3_q30(q30_mul(jg03, rgx), q30_mul(jg13, rgy), q30_mul(jg23, rgz));
        if (mag_valid) begin
            grad0_raw = sum6_q30(grad0_raw, q30_mul(jm00, rmx), q30_mul(jm10, rmy), q30_mul(jm20, rmz), '0, '0);
            grad1_raw = sum6_q30(grad1_raw, q30_mul(jm01, rmx), q30_mul(jm11, rmy), q30_mul(jm21, rmz), '0, '0);
            grad2_raw = sum6_q30(grad2_raw, q30_mul(jm02, rmx), q30_mul(jm12, rmy), q30_mul(jm22, rmz), '0, '0);
            grad3_raw = sum6_q30(grad3_raw, q30_mul(jm03, rmx), q30_mul(jm13, rmy), q30_mul(jm23, rmz), '0, '0);
        end
        madgwick_q0_next = q30_add(q0_state, q30_mul($signed(dt_reg), q30_sub(gyro_dot0, q30_mul(MADGWICK_BETA_Q30, grad0_reg))));
        madgwick_q1_next = q30_add(q1_state, q30_mul($signed(dt_reg), q30_sub(gyro_dot1, q30_mul(MADGWICK_BETA_Q30, grad1_reg))));
        madgwick_q2_next = q30_add(q2_state, q30_mul($signed(dt_reg), q30_sub(gyro_dot2, q30_mul(MADGWICK_BETA_Q30, grad2_reg))));
        madgwick_q3_next = q30_add(q3_state, q30_mul($signed(dt_reg), q30_sub(gyro_dot3, q30_mul(MADGWICK_BETA_Q30, grad3_reg))));
    end

    assign in_ready = (state == IDLE);
    assign out_valid = (state == OUTPUT);
    assign qw_out = q0_hold; assign qx_out = q1_hold; assign qy_out = q2_hold; assign qz_out = q3_hold;
    assign status_out = status_work;
    assign busy = (state != IDLE);

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            ax_reg <= '0; ay_reg <= '0; az_reg <= '0; mx_reg <= '0; my_reg <= '0; mz_reg <= '0;
            wx_reg <= '0; wy_reg <= '0; wz_reg <= '0; dt_reg <= '0; config_word_reg <= '0;
            acc_valid <= 1'b0; mag_valid <= 1'b0; status_work <= '0;
            ax_norm <= '0; ay_norm <= '0; az_norm <= '0; mx_norm <= '0; my_norm <= '0; mz_norm <= '0;
            q0_state <= Q30_ONE; q1_state <= '0; q2_state <= '0; q3_state <= '0;
            int_x_state <= '0; int_y_state <= '0; int_z_state <= '0;
            field_bx <= '0; field_bz <= '0;
            grad0_reg <= '0; grad1_reg <= '0; grad2_reg <= '0; grad3_reg <= '0;
            q0_candidate <= '0; q1_candidate <= '0; q2_candidate <= '0; q3_candidate <= '0;
            int_x_candidate <= '0; int_y_candidate <= '0; int_z_candidate <= '0;
            q0_hold <= Q30_ONE; q1_hold <= '0; q2_hold <= '0; q3_hold <= '0;
        end else begin
            case (state)
                IDLE: if (in_valid) begin
                    ax_reg <= ax_in; ay_reg <= ay_in; az_reg <= az_in;
                    mx_reg <= mx_in; my_reg <= my_in; mz_reg <= mz_in;
                    wx_reg <= wx_in; wy_reg <= wy_in; wz_reg <= wz_in;
                    dt_reg <= dt_in; config_word_reg <= config_word_in;
                    status_work <= (dt_in == 0) ? 8'h04 : 8'h00;
                    acc_valid <= 1'b0; mag_valid <= 1'b0;
                    state <= ACC_START;
                end
                ACC_START: state <= ACC_WAIT;
                ACC_WAIT: if (norm_done) begin
                    acc_valid <= !norm_zero;
                    ax_norm <= norm_o0; ay_norm <= norm_o1; az_norm <= norm_o2;
                    if (norm_zero) status_work[0] <= 1'b1;
                    state <= MAG_START;
                end
                MAG_START: state <= MAG_WAIT;
                MAG_WAIT: if (norm_done) begin
                    mag_valid <= !norm_zero;
                    mx_norm <= norm_o0; my_norm <= norm_o1; mz_norm <= norm_o2;
                    if (norm_zero) status_work[1] <= 1'b1;
                    state <= FIELD_START;
                end
                FIELD_START: if (mag_valid) state <= FIELD_WAIT; else begin
                    field_bx <= '0; field_bz <= '0; state <= CALC_PRE;
                end
                FIELD_WAIT: if (field_sqrt_done) begin
                    field_bx <= $signed(field_sqrt_root);
                    field_bz <= hz_comb;
                    state <= CALC_PRE;
                end
                CALC_PRE: begin
                    if (FILTER_MODE && acc_valid)
                        state <= GRAD_START;
                    else begin
                        grad0_reg <= '0; grad1_reg <= '0; grad2_reg <= '0; grad3_reg <= '0;
                        state <= CALC_Q;
                    end
                end
                GRAD_START: state <= GRAD_WAIT;
                GRAD_WAIT: if (norm_done) begin
                    if (norm_zero) begin
                        grad0_reg <= '0; grad1_reg <= '0; grad2_reg <= '0; grad3_reg <= '0;
                        status_work[5] <= 1'b1;
                    end else begin
                        grad0_reg <= norm_o0; grad1_reg <= norm_o1; grad2_reg <= norm_o2; grad3_reg <= norm_o3;
                    end
                    state <= CALC_Q;
                end
                CALC_Q: begin
                    if (FILTER_MODE) begin
                        q0_candidate <= madgwick_q0_next; q1_candidate <= madgwick_q1_next;
                        q2_candidate <= madgwick_q2_next; q3_candidate <= madgwick_q3_next;
                        int_x_candidate <= int_x_state; int_y_candidate <= int_y_state; int_z_candidate <= int_z_state;
                    end else begin
                        q0_candidate <= mahony_q0_next; q1_candidate <= mahony_q1_next;
                        q2_candidate <= mahony_q2_next; q3_candidate <= mahony_q3_next;
                        int_x_candidate <= int_x_next; int_y_candidate <= int_y_next; int_z_candidate <= int_z_next;
                    end
                    state <= Q_START;
                end
                Q_START: state <= Q_WAIT;
                Q_WAIT: if (norm_done) begin
                    if (norm_zero) begin
                        q0_hold <= q0_state; q1_hold <= q1_state; q2_hold <= q2_state; q3_hold <= q3_state;
                        status_work[3] <= 1'b1;
                    end else begin
                        q0_hold <= norm_o0; q1_hold <= norm_o1; q2_hold <= norm_o2; q3_hold <= norm_o3;
                    end
                    state <= OUTPUT;
                end
                OUTPUT: if (out_ready) begin
                    q0_state <= q0_hold; q1_state <= q1_hold; q2_state <= q2_hold; q3_state <= q3_hold;
                    int_x_state <= int_x_candidate; int_y_state <= int_y_candidate; int_z_state <= int_z_candidate;
                    state <= IDLE;
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
