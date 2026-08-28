// Created by Wang Jialin.
// See README.md for interfaces and usage.

`timescale 1ns/1ps
module marg_estimator_dynamic_beta_core #(
    parameter logic [31:0] MAX_DT_Q30 = 32'd21_474_836,

    parameter logic [33:0] ACC_NORMAL_LOW_Q30  = 34'd966_367_642,   // 0.90
    parameter logic [33:0] ACC_NORMAL_HIGH_Q30 = 34'd1_181_115_006, // 1.10
    parameter logic [33:0] ACC_HARD_LOW_Q30    = 34'd805_306_368,   // 0.75
    parameter logic [33:0] ACC_HARD_HIGH_Q30   = 34'd1_342_177_280, // 1.25
    parameter logic [33:0] MAG_NORMAL_LOW_Q30  = 34'd858_993_459,   // frozen 0.80
    parameter logic [33:0] MAG_NORMAL_HIGH_Q30 = 34'd1_288_490_189, // frozen 1.20
    parameter logic [33:0] MAG_HARD_LOW_Q30    = 34'd590_558_003,   // frozen 0.55
    parameter logic [33:0] MAG_HARD_HIGH_Q30   = 34'd1_556_925_645, // frozen 1.45
    parameter logic [31:0] MN2_NORMAL_Q30      = 32'd214_748_365,   // 0.20
    parameter logic [31:0] MN2_REJECT_Q30      = 32'd53_687_091,    // 0.05
    parameter logic [31:0] QDOT_NORMAL_Q30     = 32'd1_068_373_115, // 0.995
    parameter logic [31:0] QDOT_REJECT_Q30     = 32'd1_052_266_988, // 0.980
    parameter integer BAD_FRAME_COUNT           = 1,
    parameter integer GOOD_FRAME_COUNT          = 5
) (
    input  logic               clk,
    input  logic               rst_n,

    input  logic               in_valid,
    output logic               in_ready,
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
    input  logic        [31:0] beta_in,

    output logic               out_valid,
    input  logic               out_ready,
    output logic signed [31:0] qw_out,
    output logic signed [31:0] qx_out,
    output logic signed [31:0] qy_out,
    output logic signed [31:0] qz_out,
    output logic        [7:0]  status_out,
    output logic        [31:0] beta_used_out,
    output logic        [1:0]  beta_level_out,
    output logic        [1:0]  raw_level_out,
    output logic        [7:0]  dynamic_flags_out,
    output logic        [33:0] accel_norm2_out,
    output logic        [33:0] mag_norm2_out,
    output logic        [31:0] mn2_out,
    output logic        [31:0] innovation_abs_dot_out,
    output logic               busy
);
    import marg_fixed_pkg::*;

    localparam logic [7:0] ST_ACCEL_ZERO      = 8'h01;
    localparam logic [7:0] ST_MAG_ZERO        = 8'h02;
    localparam logic [7:0] ST_DT_INVALID      = 8'h04;
    localparam logic [7:0] ST_SAAM_DEGENERATE = 8'h08;
    localparam logic [7:0] ST_FINAL_ZERO      = 8'h10;
    localparam logic [7:0] ST_BETA_CLAMPED    = 8'h20;
    localparam logic [7:0] ST_NO_STATE         = 8'h40;
    localparam logic [7:0] ST_BETA_DYNAMIC     = 8'h80;

    localparam logic [7:0] DYN_FLAG_ACCEL      = 8'h01;
    localparam logic [7:0] DYN_FLAG_MAG        = 8'h02;
    localparam logic [7:0] DYN_FLAG_MN2        = 8'h04;
    localparam logic [7:0] DYN_FLAG_INNOVATION = 8'h08;
    localparam logic [7:0] DYN_FLAG_REDUCED    = 8'h10;
    localparam logic [7:0] DYN_FLAG_RECOVERY   = 8'h20;

    typedef enum logic [5:0] {
        C_IDLE,
        C_CONF_NORM_INIT, C_CONF_NORM_MUL_START, C_CONF_NORM_MUL_WAIT, C_CONF_NORM_FINAL,
        C_NORM_A_START, C_NORM_A_WAIT,
        C_NORM_M_START, C_NORM_M_WAIT,
        C_MD_INIT, C_MD_MUL_START, C_MD_MUL_WAIT, C_MD_FINAL,
        C_MN_SQUARE_START, C_MN_SQUARE_WAIT, C_MN_SQUARE_FINAL,
        C_SQRT_START, C_SQRT_WAIT,
        C_SAAM_MUL_INIT, C_SAAM_MUL_START, C_SAAM_MUL_WAIT, C_SAAM_FINAL,
        C_SAAM_NORM_START, C_SAAM_NORM_WAIT,
        C_GYRO_H_INIT, C_GYRO_H_MUL_START, C_GYRO_H_MUL_WAIT,
        C_GYRO_PROD_INIT, C_GYRO_PROD_START, C_GYRO_PROD_WAIT,
        C_GYRO_FINAL, C_GYRO_NORM_START, C_GYRO_NORM_WAIT, C_GYRO_ONLY_PREP,
        C_ALIGN_INIT, C_ALIGN_MUL_START, C_ALIGN_MUL_WAIT, C_ALIGN_DECIDE,
        C_FUSE_INIT, C_FUSE_MUL_START, C_FUSE_MUL_WAIT, C_FUSE_FINAL,
        C_FINAL_NORM_START, C_FINAL_NORM_WAIT,
        C_ERROR_LATCH, C_OUTPUT
    } core_state_t;

    core_state_t state;

    logic signed [31:0] ax_l, ay_l, az_l, mx_l, my_l, mz_l;
    logic signed [31:0] wx_l, wy_l, wz_l;
    logic        [31:0] dt_l, beta_l;
    logic        [31:0] beta_active_l, one_minus_beta_active_l;

    logic signed [31:0] ax_u, ay_u, az_u, mx_u, my_u, mz_u;
    logic signed [31:0] qsaam_w, qsaam_x, qsaam_y, qsaam_z;
    logic signed [31:0] qgyro_w, qgyro_x, qgyro_y, qgyro_z;
    logic signed [31:0] qalign_w, qalign_x, qalign_y, qalign_z;
    logic signed [31:0] qmix_w, qmix_x, qmix_y, qmix_z;
    logic signed [31:0] qprev_w, qprev_x, qprev_y, qprev_z;
    logic               have_state;
    logic               saam_available;
    logic               commit_pending;
    logic [7:0]         status_work;

    logic [1:0]          beta_level_l, beta_level_pending, raw_level_l;
    logic [3:0]          bad_frame_count_l, good_frame_count_l;
    logic [3:0]          bad_frame_count_pending, good_frame_count_pending;
    logic [33:0]         accel_norm2_q30, mag_norm2_q30;
    logic [31:0]         qdot_abs_q30;
    logic [7:0]          candidate_dynamic_flags;
    logic [1:0]          candidate_raw_level;
    logic [2:0]          soft_metric_count;
    logic [63:0]         dot_abs_q60;

    logic signed [31:0] md_q30, md_sq_q30, mn_q30;
    logic        [31:0] mn_sq_q30;
    logic signed [35:0] qtilde0, qtilde1, qtilde2, qtilde3;
    logic signed [63:0] dot_sum;
    logic signed [63:0] saam_product [0:7];
    logic signed [63:0] gyro_product [0:11];
    logic signed [63:0] fuse_product [0:7];
    logic signed [31:0] hx_q30, hy_q30, hz_q30;
    logic [3:0]         op_index;

    logic               mul_start, mul_busy, mul_done;
    logic signed [31:0] mul_a, mul_b;
    logic signed [63:0] mul_product;

    logic        sqrt_start, sqrt_busy, sqrt_done;
    logic [31:0] sqrt_u_q30, sqrt_result_q30;

    logic               norm_start, norm_busy, norm_done, norm_zero, norm_dim3;
    logic signed [35:0] norm_in0, norm_in1, norm_in2, norm_in3;
    logic signed [31:0] norm_out0, norm_out1, norm_out2, norm_out3;

    logic signed [31:0] saam_az_minus_one;
    logic signed [31:0] saam_mn_plus_mx;
    logic signed [31:0] saam_md_minus_mz;

    function automatic logic signed [31:0] vec_qgyro(input logic [1:0] idx);
        begin
            case (idx)
                2'd0: vec_qgyro = qgyro_w;
                2'd1: vec_qgyro = qgyro_x;
                2'd2: vec_qgyro = qgyro_y;
                default: vec_qgyro = qgyro_z;
            endcase
        end
    endfunction

    function automatic logic signed [31:0] vec_qsaam(input logic [1:0] idx);
        begin
            case (idx)
                2'd0: vec_qsaam = qsaam_w;
                2'd1: vec_qsaam = qsaam_x;
                2'd2: vec_qsaam = qsaam_y;
                default: vec_qsaam = qsaam_z;
            endcase
        end
    endfunction

    function automatic logic signed [31:0] vec_qalign(input logic [1:0] idx);
        begin
            case (idx)
                2'd0: vec_qalign = qalign_w;
                2'd1: vec_qalign = qalign_x;
                2'd2: vec_qalign = qalign_y;
                default: vec_qalign = qalign_z;
            endcase
        end
    endfunction

    function automatic logic [31:0] beta_for_level(
        input logic [31:0] nominal_beta,
        input logic [1:0] level
    );
        begin
            case (level)
                2'd3: beta_for_level = nominal_beta;
                2'd2: beta_for_level = nominal_beta >> 1;
                2'd1: beta_for_level = nominal_beta >> 2;
                default: beta_for_level = 32'd0;
            endcase
        end
    endfunction

    always_comb begin
        saam_az_minus_one = sat_s33_to_s32(
            $signed({az_u[31], az_u}) - $signed({ONE_Q30[31], ONE_Q30})
        );
        saam_mn_plus_mx = sat_s33_to_s32(
            $signed({mn_q30[31], mn_q30}) + $signed({mx_u[31], mx_u})
        );
        saam_md_minus_mz = sat_s33_to_s32(
            $signed({md_q30[31], md_q30}) - $signed({mz_u[31], mz_u})
        );

        dot_abs_q60 = dot_sum[63] ? $unsigned(-dot_sum) : $unsigned(dot_sum);
        qdot_abs_q30 = rshift_u64_rne(dot_abs_q60, 30);

        candidate_dynamic_flags = 8'h00;
        soft_metric_count = 3'd0;
        candidate_raw_level = 2'd3;

        if ((accel_norm2_q30 < ACC_NORMAL_LOW_Q30) ||
            (accel_norm2_q30 > ACC_NORMAL_HIGH_Q30)) begin
            candidate_dynamic_flags = candidate_dynamic_flags | DYN_FLAG_ACCEL;
            if ((accel_norm2_q30 < ACC_HARD_LOW_Q30) ||
                (accel_norm2_q30 > ACC_HARD_HIGH_Q30))
                candidate_raw_level = 2'd0;
            else
                soft_metric_count = soft_metric_count + 3'd1;
        end

        if ((mag_norm2_q30 < MAG_NORMAL_LOW_Q30) ||
            (mag_norm2_q30 > MAG_NORMAL_HIGH_Q30)) begin
            candidate_dynamic_flags = candidate_dynamic_flags | DYN_FLAG_MAG;
            if ((mag_norm2_q30 < MAG_HARD_LOW_Q30) ||
                (mag_norm2_q30 > MAG_HARD_HIGH_Q30))
                candidate_raw_level = 2'd0;
            else
                soft_metric_count = soft_metric_count + 3'd1;
        end

        if (mn_sq_q30 < MN2_NORMAL_Q30) begin
            candidate_dynamic_flags = candidate_dynamic_flags | DYN_FLAG_MN2;
            if (mn_sq_q30 <= MN2_REJECT_Q30)
                candidate_raw_level = 2'd0;
            else
                soft_metric_count = soft_metric_count + 3'd1;
        end

        if (qdot_abs_q30 < QDOT_NORMAL_Q30) begin
            candidate_dynamic_flags = candidate_dynamic_flags | DYN_FLAG_INNOVATION;
            if (qdot_abs_q30 < QDOT_REJECT_Q30)
                candidate_raw_level = 2'd0;
            else
                soft_metric_count = soft_metric_count + 3'd1;
        end

        if (candidate_raw_level != 2'd0) begin
            if (soft_metric_count == 0)
                candidate_raw_level = 2'd3;
            else if (soft_metric_count == 3'd1)
                candidate_raw_level = 2'd2;
            else
                candidate_raw_level = 2'd1;
        end
    end

    assign in_ready = (state == C_IDLE) && !out_valid;
    assign busy     = (state != C_IDLE) || out_valid;

    mul_signed32_seq u_core_mul (
        .clk(clk), .rst_n(rst_n), .start(mul_start), .a(mul_a), .b(mul_b),
        .busy(mul_busy), .done(mul_done), .product(mul_product)
    );

    sqrt_q30_seq u_mn_sqrt (
        .clk(clk), .rst_n(rst_n), .start(sqrt_start), .u_q30(sqrt_u_q30),
        .busy(sqrt_busy), .done(sqrt_done), .root_q30(sqrt_result_q30)
    );

    rational_normalizer #(.ITERATIONS(4)) u_norm (
        .clk(clk), .rst_n(rst_n), .start(norm_start), .dim3(norm_dim3),
        .in0(norm_in0), .in1(norm_in1), .in2(norm_in2), .in3(norm_in3),
        .busy(norm_busy), .done(norm_done), .zero_input(norm_zero),
        .out0(norm_out0), .out1(norm_out1), .out2(norm_out2), .out3(norm_out3)
    );

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state               <= C_IDLE;
            ax_l                <= '0; ay_l <= '0; az_l <= '0;
            mx_l                <= '0; my_l <= '0; mz_l <= '0;
            wx_l                <= '0; wy_l <= '0; wz_l <= '0;
            dt_l                <= '0; beta_l <= '0;
            beta_active_l       <= '0; one_minus_beta_active_l <= '0;
            ax_u                <= '0; ay_u <= '0; az_u <= '0;
            mx_u                <= '0; my_u <= '0; mz_u <= '0;
            qsaam_w             <= '0; qsaam_x <= '0; qsaam_y <= '0; qsaam_z <= '0;
            qgyro_w             <= '0; qgyro_x <= '0; qgyro_y <= '0; qgyro_z <= '0;
            qalign_w            <= '0; qalign_x <= '0; qalign_y <= '0; qalign_z <= '0;
            qmix_w              <= '0; qmix_x <= '0; qmix_y <= '0; qmix_z <= '0;
            qprev_w             <= ONE_Q30; qprev_x <= '0; qprev_y <= '0; qprev_z <= '0;
            qw_out              <= ONE_Q30; qx_out <= '0; qy_out <= '0; qz_out <= '0;
            have_state          <= 1'b0;
            saam_available      <= 1'b0;
            commit_pending      <= 1'b0;
            status_work         <= '0;
            status_out          <= '0;
            beta_used_out       <= '0;
            beta_level_out      <= 2'd3;
            raw_level_out       <= 2'd3;
            dynamic_flags_out   <= '0;
            accel_norm2_out     <= '0;
            mag_norm2_out       <= '0;
            mn2_out             <= '0;
            innovation_abs_dot_out <= '0;
            beta_level_l        <= 2'd3;
            beta_level_pending  <= 2'd3;
            raw_level_l         <= 2'd3;
            bad_frame_count_l   <= '0;
            good_frame_count_l  <= '0;
            bad_frame_count_pending <= '0;
            good_frame_count_pending <= '0;
            accel_norm2_q30     <= '0;
            mag_norm2_q30       <= '0;
            out_valid           <= 1'b0;
            md_q30              <= '0; md_sq_q30 <= '0; mn_q30 <= '0; mn_sq_q30 <= '0;
            qtilde0             <= '0; qtilde1 <= '0; qtilde2 <= '0; qtilde3 <= '0;
            dot_sum             <= '0;
            hx_q30              <= '0; hy_q30 <= '0; hz_q30 <= '0;
            op_index            <= '0;
            mul_start           <= 1'b0; mul_a <= '0; mul_b <= '0;
            sqrt_start          <= 1'b0; sqrt_u_q30 <= '0;
            norm_start          <= 1'b0; norm_dim3 <= 1'b0;
            norm_in0            <= '0; norm_in1 <= '0; norm_in2 <= '0; norm_in3 <= '0;
        end else begin
            mul_start  <= 1'b0;
            sqrt_start <= 1'b0;
            norm_start <= 1'b0;

            case (state)
                C_IDLE: begin
                    if (in_valid && in_ready) begin
                        ax_l <= ax_in; ay_l <= ay_in; az_l <= az_in;
                        mx_l <= mx_in; my_l <= my_in; mz_l <= mz_in;
                        wx_l <= wx_in; wy_l <= wy_in; wz_l <= wz_in;
                        dt_l <= dt_in;
                        beta_l <= (beta_in > $unsigned(ONE_Q30)) ? $unsigned(ONE_Q30) : beta_in;
                        beta_active_l <= (beta_in > $unsigned(ONE_Q30)) ? $unsigned(ONE_Q30) : beta_in;
                        one_minus_beta_active_l <= $unsigned(ONE_Q30) -
                            ((beta_in > $unsigned(ONE_Q30)) ? $unsigned(ONE_Q30) : beta_in);
                        beta_level_pending <= beta_level_l;
                        bad_frame_count_pending <= bad_frame_count_l;
                        good_frame_count_pending <= good_frame_count_l;
                        status_work <= ((beta_in > $unsigned(ONE_Q30)) ? ST_BETA_CLAMPED : 8'h00) |
                                       (((dt_in == 0) || (dt_in > MAX_DT_Q30)) ? ST_DT_INVALID : 8'h00);
                        beta_used_out <= '0;
                        beta_level_out <= beta_level_l;
                        raw_level_out <= 2'd3;
                        dynamic_flags_out <= '0;
                        accel_norm2_out <= '0;
                        mag_norm2_out <= '0;
                        mn2_out <= '0;
                        innovation_abs_dot_out <= '0;
                        saam_available <= 1'b1;
                        commit_pending <= 1'b0;
                        if ((dt_in == 0) || (dt_in > MAX_DT_Q30))
                            state <= C_ERROR_LATCH;
                        else
                            state <= C_CONF_NORM_INIT;
                    end
                end

                C_CONF_NORM_INIT: begin
                    accel_norm2_q30 <= '0;
                    mag_norm2_q30   <= '0;
                    op_index        <= '0;
                    state           <= C_CONF_NORM_MUL_START;
                end

                C_CONF_NORM_MUL_START: begin
                    if (!mul_busy) begin
                        case (op_index)
                            4'd0: mul_a <= ax_l;
                            4'd1: mul_a <= ay_l;
                            4'd2: mul_a <= az_l;
                            4'd3: mul_a <= mx_l;
                            4'd4: mul_a <= my_l;
                            default: mul_a <= mz_l;
                        endcase
                        case (op_index)
                            4'd0: mul_b <= ax_l;
                            4'd1: mul_b <= ay_l;
                            4'd2: mul_b <= az_l;
                            4'd3: mul_b <= mx_l;
                            4'd4: mul_b <= my_l;
                            default: mul_b <= mz_l;
                        endcase
                        mul_start <= 1'b1;
                        state <= C_CONF_NORM_MUL_WAIT;
                    end
                end

                C_CONF_NORM_MUL_WAIT: begin
                    if (mul_done) begin
                        if (op_index < 4'd3)
                            accel_norm2_q30 <= accel_norm2_q30 +
                                {2'b00, $unsigned(sat_rshift_s64_rne(mul_product, 30))};
                        else
                            mag_norm2_q30 <= mag_norm2_q30 +
                                {2'b00, $unsigned(sat_rshift_s64_rne(mul_product, 30))};

                        if (op_index == 4'd5)
                            state <= C_CONF_NORM_FINAL;
                        else begin
                            op_index <= op_index + 4'd1;
                            state <= C_CONF_NORM_MUL_START;
                        end
                    end
                end

                C_CONF_NORM_FINAL: begin
                    accel_norm2_out <= accel_norm2_q30;
                    mag_norm2_out   <= mag_norm2_q30;
                    state <= C_NORM_A_START;
                end

                C_NORM_A_START: begin
                    if (!norm_busy) begin
                        norm_dim3 <= 1'b1;
                        norm_in0  <= {{4{ax_l[31]}}, ax_l};
                        norm_in1  <= {{4{ay_l[31]}}, ay_l};
                        norm_in2  <= {{4{az_l[31]}}, az_l};
                        norm_in3  <= '0;
                        norm_start <= 1'b1;
                        state <= C_NORM_A_WAIT;
                    end
                end

                C_NORM_A_WAIT: begin
                    if (norm_done) begin
                        if (norm_zero) begin
                            status_work    <= status_work | ST_ACCEL_ZERO;
                            saam_available <= 1'b0;
                            state <= have_state ? C_GYRO_H_INIT : C_ERROR_LATCH;
                        end else begin
                            ax_u <= norm_out0; ay_u <= norm_out1; az_u <= norm_out2;
                            state <= C_NORM_M_START;
                        end
                    end
                end

                C_NORM_M_START: begin
                    if (!norm_busy) begin
                        norm_dim3 <= 1'b1;
                        norm_in0  <= {{4{mx_l[31]}}, mx_l};
                        norm_in1  <= {{4{my_l[31]}}, my_l};
                        norm_in2  <= {{4{mz_l[31]}}, mz_l};
                        norm_in3  <= '0;
                        norm_start <= 1'b1;
                        state <= C_NORM_M_WAIT;
                    end
                end

                C_NORM_M_WAIT: begin
                    if (norm_done) begin
                        if (norm_zero) begin
                            status_work    <= status_work | ST_MAG_ZERO;
                            saam_available <= 1'b0;
                            state <= have_state ? C_GYRO_H_INIT : C_ERROR_LATCH;
                        end else begin
                            mx_u <= norm_out0; my_u <= norm_out1; mz_u <= norm_out2;
                            state <= C_MD_INIT;
                        end
                    end
                end

                C_MD_INIT: begin
                    dot_sum  <= '0;
                    op_index <= '0;
                    state    <= C_MD_MUL_START;
                end

                C_MD_MUL_START: begin
                    if (!mul_busy) begin
                        case (op_index)
                            4'd0: begin mul_a <= ax_u; mul_b <= mx_u; end
                            4'd1: begin mul_a <= ay_u; mul_b <= my_u; end
                            default: begin mul_a <= az_u; mul_b <= mz_u; end
                        endcase
                        mul_start <= 1'b1;
                        state <= C_MD_MUL_WAIT;
                    end
                end

                C_MD_MUL_WAIT: begin
                    if (mul_done) begin
                        dot_sum <= dot_sum + mul_product;
                        if (op_index == 4'd2)
                            state <= C_MD_FINAL;
                        else begin
                            op_index <= op_index + 4'd1;
                            state <= C_MD_MUL_START;
                        end
                    end
                end

                C_MD_FINAL: begin
                    md_q30 <= clamp_unit_q30(sat_rshift_s64_rne(dot_sum, 30));
                    state  <= C_MN_SQUARE_START;
                end

                C_MN_SQUARE_START: begin
                    if (!mul_busy) begin
                        mul_a <= md_q30;
                        mul_b <= md_q30;
                        mul_start <= 1'b1;
                        state <= C_MN_SQUARE_WAIT;
                    end
                end

                C_MN_SQUARE_WAIT: begin
                    if (mul_done) begin
                        md_sq_q30 <= sat_rshift_s64_rne(mul_product, 30);
                        state <= C_MN_SQUARE_FINAL;
                    end
                end

                C_MN_SQUARE_FINAL: begin
                    if ($unsigned(md_sq_q30) >= (32'h4000_0000 - MN2_MIN_Q30)) begin
                        mn2_out        <= '0;
                        status_work    <= status_work | ST_SAAM_DEGENERATE;
                        saam_available <= 1'b0;
                        state <= have_state ? C_GYRO_H_INIT : C_ERROR_LATCH;
                    end else begin
                        mn_sq_q30 <= 32'h4000_0000 - $unsigned(md_sq_q30);
                        mn2_out    <= 32'h4000_0000 - $unsigned(md_sq_q30);
                        state <= C_SQRT_START;
                    end
                end

                C_SQRT_START: begin
                    if (!sqrt_busy) begin
                        sqrt_u_q30 <= mn_sq_q30;
                        sqrt_start <= 1'b1;
                        state <= C_SQRT_WAIT;
                    end
                end

                C_SQRT_WAIT: begin
                    if (sqrt_done) begin
                        mn_q30 <= $signed(sqrt_result_q30);
                        state <= C_SAAM_MUL_INIT;
                    end
                end

                C_SAAM_MUL_INIT: begin
                    op_index <= '0;
                    state <= C_SAAM_MUL_START;
                end

                C_SAAM_MUL_START: begin
                    if (!mul_busy) begin
                        case (op_index)
                            4'd0: begin mul_a <= saam_az_minus_one; mul_b <= saam_mn_plus_mx; end
                            4'd1: begin mul_a <= ax_u;               mul_b <= saam_md_minus_mz; end
                            4'd2: begin mul_a <= saam_az_minus_one; mul_b <= my_u; end
                            4'd3: begin mul_a <= ay_u;               mul_b <= saam_md_minus_mz; end
                            4'd4: begin mul_a <= az_u;               mul_b <= md_q30; end
                            4'd5: begin mul_a <= ax_u;               mul_b <= mn_q30; end
                            4'd6: begin mul_a <= ax_u;               mul_b <= my_u; end
                            default: begin mul_a <= ay_u;             mul_b <= saam_mn_plus_mx; end
                        endcase
                        mul_start <= 1'b1;
                        state <= C_SAAM_MUL_WAIT;
                    end
                end

                C_SAAM_MUL_WAIT: begin
                    if (mul_done) begin
                        saam_product[op_index] <= mul_product;
                        if (op_index == 4'd7)
                            state <= C_SAAM_FINAL;
                        else begin
                            op_index <= op_index + 4'd1;
                            state <= C_SAAM_MUL_START;
                        end
                    end
                end

                C_SAAM_FINAL: begin
                    qtilde0 <= rshift_s64_to_s36_rne(saam_product[0] + saam_product[1], 30);
                    qtilde1 <= rshift_s64_to_s36_rne(saam_product[2] + saam_product[3], 30);
                    qtilde2 <= rshift_s64_to_s36_rne(
                        saam_product[4] - saam_product[5] - q30_to_q60(mz_u), 30
                    );
                    qtilde3 <= rshift_s64_to_s36_rne(saam_product[6] - saam_product[7], 30);
                    state <= C_SAAM_NORM_START;
                end

                C_SAAM_NORM_START: begin
                    if (!norm_busy) begin
                        norm_dim3 <= 1'b0;
                        norm_in0 <= qtilde3;
                        norm_in1 <= qtilde0;
                        norm_in2 <= qtilde1;
                        norm_in3 <= qtilde2;
                        norm_start <= 1'b1;
                        state <= C_SAAM_NORM_WAIT;
                    end
                end

                C_SAAM_NORM_WAIT: begin
                    if (norm_done) begin
                        if (norm_zero) begin
                            status_work    <= status_work | ST_SAAM_DEGENERATE;
                            saam_available <= 1'b0;
                            state <= have_state ? C_GYRO_H_INIT : C_ERROR_LATCH;
                        end else begin
                            qsaam_w <= norm_out0; qsaam_x <= norm_out1;
                            qsaam_y <= norm_out2; qsaam_z <= norm_out3;
                            if (have_state)
                                state <= C_GYRO_H_INIT;
                            else begin
                                qgyro_w <= norm_out0; qgyro_x <= norm_out1;
                                qgyro_y <= norm_out2; qgyro_z <= norm_out3;
                                state <= C_ALIGN_INIT;
                            end
                        end
                    end
                end

                C_GYRO_H_INIT: begin
                    op_index <= '0;
                    state <= C_GYRO_H_MUL_START;
                end

                C_GYRO_H_MUL_START: begin
                    if (!mul_busy) begin
                        case (op_index)
                            4'd0: mul_a <= wx_l;
                            4'd1: mul_a <= wy_l;
                            default: mul_a <= wz_l;
                        endcase
                        mul_b <= $signed(dt_l);
                        mul_start <= 1'b1;
                        state <= C_GYRO_H_MUL_WAIT;
                    end
                end

                C_GYRO_H_MUL_WAIT: begin
                    if (mul_done) begin
                        case (op_index)
                            4'd0: hx_q30 <= sat_rshift_s64_rne(mul_product, 25);
                            4'd1: hy_q30 <= sat_rshift_s64_rne(mul_product, 25);
                            default: hz_q30 <= sat_rshift_s64_rne(mul_product, 25);
                        endcase
                        if (op_index == 4'd2)
                            state <= C_GYRO_PROD_INIT;
                        else begin
                            op_index <= op_index + 4'd1;
                            state <= C_GYRO_H_MUL_START;
                        end
                    end
                end

                C_GYRO_PROD_INIT: begin
                    op_index <= '0;
                    state <= C_GYRO_PROD_START;
                end

                C_GYRO_PROD_START: begin
                    if (!mul_busy) begin
                        case (op_index)
                            4'd0:  begin mul_a <= hx_q30; mul_b <= qprev_x; end
                            4'd1:  begin mul_a <= hy_q30; mul_b <= qprev_y; end
                            4'd2:  begin mul_a <= hz_q30; mul_b <= qprev_z; end
                            4'd3:  begin mul_a <= hx_q30; mul_b <= qprev_w; end
                            4'd4:  begin mul_a <= hz_q30; mul_b <= qprev_y; end
                            4'd5:  begin mul_a <= hy_q30; mul_b <= qprev_z; end
                            4'd6:  begin mul_a <= hy_q30; mul_b <= qprev_w; end
                            4'd7:  begin mul_a <= hz_q30; mul_b <= qprev_x; end
                            4'd8:  begin mul_a <= hx_q30; mul_b <= qprev_z; end
                            4'd9:  begin mul_a <= hz_q30; mul_b <= qprev_w; end
                            4'd10: begin mul_a <= hy_q30; mul_b <= qprev_x; end
                            default: begin mul_a <= hx_q30; mul_b <= qprev_y; end
                        endcase
                        mul_start <= 1'b1;
                        state <= C_GYRO_PROD_WAIT;
                    end
                end

                C_GYRO_PROD_WAIT: begin
                    if (mul_done) begin
                        gyro_product[op_index] <= mul_product;
                        if (op_index == 4'd11)
                            state <= C_GYRO_FINAL;
                        else begin
                            op_index <= op_index + 4'd1;
                            state <= C_GYRO_PROD_START;
                        end
                    end
                end

                C_GYRO_FINAL: begin
                    qgyro_w <= sat_rshift_s64_rne(
                        q30_to_q60(qprev_w) - gyro_product[0] - gyro_product[1] - gyro_product[2], 30
                    );
                    qgyro_x <= sat_rshift_s64_rne(
                        q30_to_q60(qprev_x) + gyro_product[3] + gyro_product[4] - gyro_product[5], 30
                    );
                    qgyro_y <= sat_rshift_s64_rne(
                        q30_to_q60(qprev_y) + gyro_product[6] - gyro_product[7] + gyro_product[8], 30
                    );
                    qgyro_z <= sat_rshift_s64_rne(
                        q30_to_q60(qprev_z) + gyro_product[9] + gyro_product[10] - gyro_product[11], 30
                    );
                    state <= C_GYRO_NORM_START;
                end

                C_GYRO_NORM_START: begin
                    if (!norm_busy) begin
                        norm_dim3 <= 1'b0;
                        norm_in0 <= {{4{qgyro_w[31]}}, qgyro_w};
                        norm_in1 <= {{4{qgyro_x[31]}}, qgyro_x};
                        norm_in2 <= {{4{qgyro_y[31]}}, qgyro_y};
                        norm_in3 <= {{4{qgyro_z[31]}}, qgyro_z};
                        norm_start <= 1'b1;
                        state <= C_GYRO_NORM_WAIT;
                    end
                end

                C_GYRO_NORM_WAIT: begin
                    if (norm_done) begin
                        if (norm_zero) begin
                            status_work <= status_work | ST_FINAL_ZERO;
                            commit_pending <= 1'b0;
                            state <= C_ERROR_LATCH;
                        end else begin
                            qgyro_w <= norm_out0; qgyro_x <= norm_out1;
                            qgyro_y <= norm_out2; qgyro_z <= norm_out3;
                            state <= saam_available ? C_ALIGN_INIT : C_GYRO_ONLY_PREP;
                        end
                    end
                end

                C_GYRO_ONLY_PREP: begin
                    beta_level_pending <= 2'd0;
                    bad_frame_count_pending <= BAD_FRAME_COUNT;
                    good_frame_count_pending <= '0;
                    raw_level_l <= 2'd0;
                    raw_level_out <= 2'd0;
                    beta_active_l <= '0;
                    one_minus_beta_active_l <= $unsigned(ONE_Q30);
                    beta_used_out <= '0;
                    beta_level_out <= 2'd0;
                    dynamic_flags_out <= DYN_FLAG_REDUCED;
                    status_work <= status_work | ST_BETA_DYNAMIC;
                    qmix_w <= qgyro_w; qmix_x <= qgyro_x;
                    qmix_y <= qgyro_y; qmix_z <= qgyro_z;
                    state <= C_FINAL_NORM_START;
                end

                C_ALIGN_INIT: begin
                    dot_sum  <= '0;
                    op_index <= '0;
                    state    <= C_ALIGN_MUL_START;
                end

                C_ALIGN_MUL_START: begin
                    if (!mul_busy) begin
                        mul_a <= vec_qgyro(op_index[1:0]);
                        mul_b <= vec_qsaam(op_index[1:0]);
                        mul_start <= 1'b1;
                        state <= C_ALIGN_MUL_WAIT;
                    end
                end

                C_ALIGN_MUL_WAIT: begin
                    if (mul_done) begin
                        dot_sum <= dot_sum + mul_product;
                        if (op_index == 4'd3)
                            state <= C_ALIGN_DECIDE;
                        else begin
                            op_index <= op_index + 4'd1;
                            state <= C_ALIGN_MUL_START;
                        end
                    end
                end

                C_ALIGN_DECIDE: begin
                    if (dot_sum[63]) begin
                        qalign_w <= -qsaam_w; qalign_x <= -qsaam_x;
                        qalign_y <= -qsaam_y; qalign_z <= -qsaam_z;
                    end else begin
                        qalign_w <= qsaam_w; qalign_x <= qsaam_x;
                        qalign_y <= qsaam_y; qalign_z <= qsaam_z;
                    end

                    innovation_abs_dot_out <= qdot_abs_q30;
                    raw_level_l <= candidate_raw_level;
                    raw_level_out <= candidate_raw_level;

                    if (candidate_raw_level < 2'd3) begin
                        good_frame_count_pending <= '0;
                        if (bad_frame_count_l < 4'hf)
                            bad_frame_count_pending <= bad_frame_count_l + 4'd1;
                        else
                            bad_frame_count_pending <= bad_frame_count_l;

                        if (bad_frame_count_l >= (BAD_FRAME_COUNT - 1)) begin
                            if (candidate_raw_level < beta_level_l)
                                beta_level_pending <= candidate_raw_level;
                            else
                                beta_level_pending <= beta_level_l;

                            if (candidate_raw_level < beta_level_l) begin
                                beta_active_l <= beta_for_level(beta_l, candidate_raw_level);
                                one_minus_beta_active_l <= $unsigned(ONE_Q30) -
                                    beta_for_level(beta_l, candidate_raw_level);
                                beta_used_out <= beta_for_level(beta_l, candidate_raw_level);
                                beta_level_out <= candidate_raw_level;
                            end else begin
                                beta_active_l <= beta_for_level(beta_l, beta_level_l);
                                one_minus_beta_active_l <= $unsigned(ONE_Q30) -
                                    beta_for_level(beta_l, beta_level_l);
                                beta_used_out <= beta_for_level(beta_l, beta_level_l);
                                beta_level_out <= beta_level_l;
                            end
                            dynamic_flags_out <= candidate_dynamic_flags | DYN_FLAG_REDUCED;
                            status_work <= status_work | ST_BETA_DYNAMIC;
                        end else begin
                            beta_active_l <= beta_for_level(beta_l, beta_level_l);
                            one_minus_beta_active_l <= $unsigned(ONE_Q30) -
                                beta_for_level(beta_l, beta_level_l);
                            beta_used_out <= beta_for_level(beta_l, beta_level_l);
                            beta_level_out <= beta_level_l;
                            if (beta_level_l < 2'd3) begin
                                dynamic_flags_out <= candidate_dynamic_flags | DYN_FLAG_REDUCED;
                                status_work <= status_work | ST_BETA_DYNAMIC;
                            end else
                                dynamic_flags_out <= candidate_dynamic_flags;
                        end
                    end else begin
                        bad_frame_count_pending <= '0;
                        if (beta_level_l < 2'd3) begin
                            if (good_frame_count_l >= (GOOD_FRAME_COUNT - 1)) begin
                                beta_level_pending <= beta_level_l + 2'd1;
                                good_frame_count_pending <= '0;
                                beta_active_l <= beta_for_level(beta_l, beta_level_l + 2'd1);
                                one_minus_beta_active_l <= $unsigned(ONE_Q30) -
                                    beta_for_level(beta_l, beta_level_l + 2'd1);
                                beta_used_out <= beta_for_level(beta_l, beta_level_l + 2'd1);
                                beta_level_out <= beta_level_l + 2'd1;
                                if ((beta_level_l + 2'd1) < 2'd3) begin
                                    dynamic_flags_out <= DYN_FLAG_REDUCED | DYN_FLAG_RECOVERY;
                                    status_work <= status_work | ST_BETA_DYNAMIC;
                                end else
                                    dynamic_flags_out <= DYN_FLAG_RECOVERY;
                            end else begin
                                good_frame_count_pending <= good_frame_count_l + 4'd1;
                                beta_active_l <= beta_for_level(beta_l, beta_level_l);
                                one_minus_beta_active_l <= $unsigned(ONE_Q30) -
                                    beta_for_level(beta_l, beta_level_l);
                                beta_used_out <= beta_for_level(beta_l, beta_level_l);
                                beta_level_out <= beta_level_l;
                                dynamic_flags_out <= DYN_FLAG_REDUCED;
                                status_work <= status_work | ST_BETA_DYNAMIC;
                            end
                        end else begin
                            good_frame_count_pending <= '0;
                            beta_active_l <= beta_l;
                            one_minus_beta_active_l <= $unsigned(ONE_Q30) - beta_l;
                            beta_used_out <= beta_l;
                            beta_level_out <= 2'd3;
                            dynamic_flags_out <= '0;
                        end
                    end
                    state <= C_FUSE_INIT;
                end

                C_FUSE_INIT: begin
                    op_index <= '0;
                    state <= C_FUSE_MUL_START;
                end

                C_FUSE_MUL_START: begin
                    if (!mul_busy) begin
                        if (op_index[0] == 1'b0) begin
                            mul_a <= $signed(beta_active_l);
                            mul_b <= vec_qalign(op_index[2:1]);
                        end else begin
                            mul_a <= $signed(one_minus_beta_active_l);
                            mul_b <= vec_qgyro(op_index[2:1]);
                        end
                        mul_start <= 1'b1;
                        state <= C_FUSE_MUL_WAIT;
                    end
                end

                C_FUSE_MUL_WAIT: begin
                    if (mul_done) begin
                        fuse_product[op_index] <= mul_product;
                        if (op_index == 4'd7)
                            state <= C_FUSE_FINAL;
                        else begin
                            op_index <= op_index + 4'd1;
                            state <= C_FUSE_MUL_START;
                        end
                    end
                end

                C_FUSE_FINAL: begin
                    qmix_w <= sat_rshift_s64_rne(fuse_product[0] + fuse_product[1], 30);
                    qmix_x <= sat_rshift_s64_rne(fuse_product[2] + fuse_product[3], 30);
                    qmix_y <= sat_rshift_s64_rne(fuse_product[4] + fuse_product[5], 30);
                    qmix_z <= sat_rshift_s64_rne(fuse_product[6] + fuse_product[7], 30);
                    state <= C_FINAL_NORM_START;
                end

                C_FINAL_NORM_START: begin
                    if (!norm_busy) begin
                        norm_dim3 <= 1'b0;
                        norm_in0 <= {{4{qmix_w[31]}}, qmix_w};
                        norm_in1 <= {{4{qmix_x[31]}}, qmix_x};
                        norm_in2 <= {{4{qmix_y[31]}}, qmix_y};
                        norm_in3 <= {{4{qmix_z[31]}}, qmix_z};
                        norm_start <= 1'b1;
                        state <= C_FINAL_NORM_WAIT;
                    end
                end

                C_FINAL_NORM_WAIT: begin
                    if (norm_done) begin
                        if (norm_zero) begin
                            status_work <= status_work | ST_FINAL_ZERO;
                            commit_pending <= 1'b0;
                            state <= C_ERROR_LATCH;
                        end else begin
                            qw_out <= norm_out0; qx_out <= norm_out1;
                            qy_out <= norm_out2; qz_out <= norm_out3;
                            status_out <= status_work;
                            commit_pending <= 1'b1;
                            out_valid <= 1'b1;
                            state <= C_OUTPUT;
                        end
                    end
                end

                C_ERROR_LATCH: begin
                    qw_out <= qprev_w; qx_out <= qprev_x;
                    qy_out <= qprev_y; qz_out <= qprev_z;
                    status_out <= status_work | ((!have_state) ? ST_NO_STATE : 8'h00);
                    commit_pending <= 1'b0;
                    out_valid <= 1'b1;
                    state <= C_OUTPUT;
                end

                C_OUTPUT: begin
                    if (out_valid && out_ready) begin
                        if (commit_pending) begin
                            qprev_w <= qw_out; qprev_x <= qx_out;
                            qprev_y <= qy_out; qprev_z <= qz_out;
                            have_state <= 1'b1;
                            beta_level_l <= beta_level_pending;
                            bad_frame_count_l <= bad_frame_count_pending;
                            good_frame_count_l <= good_frame_count_pending;
                        end
                        out_valid <= 1'b0;
                        state <= C_IDLE;
                    end
                end

                default: state <= C_IDLE;
            endcase
        end
    end
endmodule
