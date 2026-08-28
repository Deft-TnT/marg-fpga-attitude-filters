// Created by Wang Jialin.
// See README.md for interfaces and usage.
// RGRSF (Reliability-Gated Reference-Selective Fusion) MARG attitude-estimation core.
// It selects one reference policy per frame: full MARG correction, yaw-preserving IMU tilt correction,
// or gyro-only propagation when the gravity reference is unreliable.
// Reliability is evaluated from sensor magnitudes, magnetic observability, quaternion innovation,
// predicted-gravity alignment, and horizontal magnetic-direction alignment.

`timescale 1ns/1ps
module marg_estimator_three_mode_core #(
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
    parameter logic signed [31:0] ACC_DIR_NORMAL_Q30 = 32'sd1_063_001_210, // 0.990
    parameter logic signed [31:0] ACC_DIR_REJECT_Q30 = 32'sd1_009_964_356, // 0.940
    parameter logic [31:0] MAG_DIR_NORMAL_COS2_Q30 = 32'd1_041_364_546, // frozen cos^2(10 deg)
    parameter logic [31:0] MAG_DIR_REJECT_COS2_Q30 = 32'd881_964_882,   // frozen cos^2(25 deg)
    parameter integer IMU_BETA_SHIFT = 2,
    parameter logic [31:0] IMU_GEOMETRY_MIN2_Q30 = 32'd1_073_742, // 0.001
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
    output logic        [1:0]  mode_out,
    output logic        [1:0]  raw_mode_out,
    output logic        [7:0]  mode_flags_out,
    output logic signed [31:0] accel_dir_dot_out,
    output logic signed [31:0] mag_hdir_dot_out,
    output logic signed [31:0] qgyro_w_out,
    output logic signed [31:0] qgyro_x_out,
    output logic signed [31:0] qgyro_y_out,
    output logic signed [31:0] qgyro_z_out,
    output logic signed [31:0] qacc_w_out,
    output logic signed [31:0] qacc_x_out,
    output logic signed [31:0] qacc_y_out,
    output logic signed [31:0] qacc_z_out,
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

    typedef enum logic [6:0] {
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
        C_GYRO_FINAL, C_GYRO_NORM_START, C_GYRO_NORM_WAIT,
        C_DIR_QTERM_INIT, C_DIR_QTERM_START, C_DIR_QTERM_WAIT, C_DIR_QTERM_FINAL,
        C_DIR_ACCEL_INIT, C_DIR_ACCEL_START, C_DIR_ACCEL_WAIT, C_DIR_ACCEL_FINAL,
        C_DIR_SAAM_YAW_INIT, C_DIR_SAAM_YAW_START, C_DIR_SAAM_YAW_WAIT,
        C_DIR_SAAM_YAW_FINAL, C_DIR_HMAG_INIT, C_DIR_HMAG_START,
        C_DIR_HMAG_WAIT, C_DIR_HMAG_FINAL,
        C_ALIGN_INIT, C_ALIGN_MUL_START, C_ALIGN_MUL_WAIT, C_ALIGN_DECIDE,
        C_MODE_DECIDE, C_GYRO_ONLY_PREP,
        C_IMU_CT_INIT, C_IMU_CT_START, C_IMU_CT_WAIT, C_IMU_CT_FINAL,
        C_IMU_CT_SQRT_START, C_IMU_CT_SQRT_WAIT,
        C_IMU_RHO_INIT, C_IMU_RHO_START, C_IMU_RHO_WAIT, C_IMU_RHO_FINAL,
        C_IMU_RHO_SQRT_START, C_IMU_RHO_SQRT_WAIT,
        C_IMU_QMUL1_INIT, C_IMU_QMUL1_START, C_IMU_QMUL1_WAIT, C_IMU_QMUL1_FINAL,
        C_IMU_QMUL2_INIT, C_IMU_QMUL2_START, C_IMU_QMUL2_WAIT, C_IMU_QMUL2_FINAL,
        C_IMU_QACC_NORM_START, C_IMU_QACC_NORM_WAIT,
        C_IMU_ALIGN_INIT, C_IMU_ALIGN_MUL_START, C_IMU_ALIGN_MUL_WAIT,
        C_IMU_ALIGN_DECIDE,
        C_FUSE_INIT, C_FUSE_MUL_START, C_FUSE_MUL_WAIT, C_FUSE_FINAL,
        C_FINAL_NORM_START, C_FINAL_NORM_WAIT,
        C_ERROR_LATCH, C_OUTPUT
    } core_state_t;

    core_state_t state;

    logic signed [31:0] ax_l, ay_l, az_l, mx_l, my_l, mz_l;
    logic signed [31:0] wx_l, wy_l, wz_l;
    logic        [31:0] dt_l, beta_l;
    logic        [31:0] beta_active_l, one_minus_beta_active_l;

    // MARG uses SAAM; IMU_TILT rejects magnetic heading but retains gravity tilt; GYRO_ONLY rejects both references.
    localparam logic [1:0] MODE_MARG      = 2'd0;
    localparam logic [1:0] MODE_IMU_TILT  = 2'd1;
    localparam logic [1:0] MODE_GYRO_ONLY = 2'd2;

    logic signed [31:0] ax_u, ay_u, az_u, mx_u, my_u, mz_u;
    logic signed [31:0] qsaam_w, qsaam_x, qsaam_y, qsaam_z;
    logic signed [31:0] qgyro_w, qgyro_x, qgyro_y, qgyro_z;
    logic signed [31:0] qacc_w, qacc_x, qacc_y, qacc_z;
    logic signed [31:0] qalign_w, qalign_x, qalign_y, qalign_z;
    logic signed [31:0] qmix_w, qmix_x, qmix_y, qmix_z;
    logic signed [31:0] qprev_w, qprev_x, qprev_y, qprev_z;
    logic               have_state;
    logic               saam_available;
    logic               accel_unit_valid, mag_unit_valid;
    logic               qacc_available;
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

    logic [1:0]          mode_l, mode_pending, raw_mode_l, selected_mode_l;
    logic [1:0]          candidate_raw_mode;
    logic [1:0]          mode_decided_comb;
    logic [3:0]          mode_bad_count_l, mode_good_count_l;
    logic [3:0]          mode_bad_count_pending, mode_good_count_pending;
    logic [7:0]          mode_flags_work;
    logic                accel_dir_soft, accel_dir_hard;
    logic                mag_dir_soft, mag_dir_hard;
    logic                accel_raw_soft, accel_raw_hard, mag_raw_soft, mag_raw_hard;
    logic                accel_direction_actionable, mag_direction_actionable;
    logic                mn2_soft, mn2_hard, innovation_soft, innovation_hard;

    logic signed [31:0] qterm_x2, qterm_y2, qterm_z2, qterm_xy, qterm_xz,
                        qterm_yz, qterm_wx, qterm_wy, qterm_wz;
    logic signed [31:0] gpred_x, gpred_y, gpred_z;
    logic signed [31:0] accel_dir_dot_q30, mag_hdir_dot_q30;
    logic signed [31:0] yaw_g_x, yaw_g_y, yaw_s_x, yaw_s_y;
    logic signed [31:0] saam_y2_q30, saam_z2_q30, saam_wz_q30, saam_xy_q30;
    logic signed [31:0] yaw_g_norm2, yaw_s_norm2, yaw_cross_q30;
    logic signed [31:0] yaw_cross_sq_q30, yaw_norm_product_q30,
                        yaw_normal_rhs_q30, yaw_reject_rhs_q30;
    logic signed [63:0] dir_sum;
    logic signed [63:0] dir_product [0:15];

    logic signed [31:0] ct_q30, rho_q30;
    logic signed [31:0] yaw_half_w, yaw_half_z;
    logic signed [31:0] pitch_half_w, pitch_half_y;
    logic signed [31:0] roll_half_w, roll_half_x;
    logic signed [31:0] qmul1_w, qmul1_x, qmul1_y, qmul1_z;
    logic signed [35:0] qacc_pre_w, qacc_pre_x, qacc_pre_y, qacc_pre_z;
    logic signed [63:0] qmul_product [0:15];

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

    function automatic logic signed [31:0] vec_qacc(input logic [1:0] idx);
        begin
            case (idx)
                2'd0: vec_qacc = qacc_w;
                2'd1: vec_qacc = qacc_x;
                2'd2: vec_qacc = qacc_y;
                default: vec_qacc = qacc_z;
            endcase
        end
    endfunction

    function automatic logic signed [31:0] sat_double_q30(
        input logic signed [32:0] value
    );
        begin
            sat_double_q30 = sat_s33_to_s32(value <<< 1);
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

    // Convert per-frame quality indicators into a raw candidate mode and a dynamic β level.
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
        mode_flags_work = 8'h00;

        accel_raw_hard = 1'b0;
        accel_raw_soft = 1'b0;
        mag_raw_hard = 1'b0;
        mag_raw_soft = 1'b0;
        mn2_soft = 1'b0;
        mn2_hard = 1'b0;
        innovation_soft = 1'b0;
        innovation_hard = 1'b0;
        accel_dir_soft = 1'b0;
        accel_dir_hard = !accel_unit_valid;
        mag_dir_soft = 1'b0;
        mag_dir_hard = (!mag_unit_valid) || (!saam_available) ||
                       (yaw_cross_q30 <= 0) ||
                       ($unsigned(yaw_cross_sq_q30) < $unsigned(yaw_reject_rhs_q30));
        accel_direction_actionable = 1'b0;
        mag_direction_actionable = 1'b0;

        if ((accel_norm2_q30 < ACC_NORMAL_LOW_Q30) ||
            (accel_norm2_q30 > ACC_NORMAL_HIGH_Q30)) begin
            candidate_dynamic_flags = candidate_dynamic_flags | DYN_FLAG_ACCEL;
            mode_flags_work = mode_flags_work | 8'h01;
            if ((accel_norm2_q30 < ACC_HARD_LOW_Q30) ||
                (accel_norm2_q30 > ACC_HARD_HIGH_Q30)) begin
                accel_raw_hard = 1'b1;
                candidate_raw_level = 2'd0;
            end
            else begin
                accel_raw_soft = 1'b1;
                soft_metric_count = soft_metric_count + 3'd1;
            end
        end

        if ((mag_norm2_q30 < MAG_NORMAL_LOW_Q30) ||
            (mag_norm2_q30 > MAG_NORMAL_HIGH_Q30)) begin
            candidate_dynamic_flags = candidate_dynamic_flags | DYN_FLAG_MAG;
            mode_flags_work = mode_flags_work | 8'h04;
            if ((mag_norm2_q30 < MAG_HARD_LOW_Q30) ||
                (mag_norm2_q30 > MAG_HARD_HIGH_Q30)) begin
                mag_raw_hard = 1'b1;
                candidate_raw_level = 2'd0;
            end
            else begin
                mag_raw_soft = 1'b1;
                soft_metric_count = soft_metric_count + 3'd1;
            end
        end

        if (mn_sq_q30 < MN2_NORMAL_Q30) begin
            candidate_dynamic_flags = candidate_dynamic_flags | DYN_FLAG_MN2;
            mode_flags_work = mode_flags_work | 8'h08;
            if (mn_sq_q30 <= MN2_REJECT_Q30) begin
                mn2_hard = 1'b1;
                candidate_raw_level = 2'd0;
            end else begin
                mn2_soft = 1'b1;
                soft_metric_count = soft_metric_count + 3'd1;
            end
        end

        if (saam_available && (qdot_abs_q30 < QDOT_NORMAL_Q30)) begin
            candidate_dynamic_flags = candidate_dynamic_flags | DYN_FLAG_INNOVATION;
            mode_flags_work = mode_flags_work | 8'h20;
            if (qdot_abs_q30 < QDOT_REJECT_Q30) begin
                innovation_hard = 1'b1;
                candidate_raw_level = 2'd0;
            end else begin
                innovation_soft = 1'b1;
                soft_metric_count = soft_metric_count + 3'd1;
            end
        end

        if (accel_unit_valid && (accel_dir_dot_q30 < ACC_DIR_NORMAL_Q30)) begin
            candidate_dynamic_flags = candidate_dynamic_flags | 8'h40;
            mode_flags_work = mode_flags_work | 8'h02;
            if (accel_dir_dot_q30 < ACC_DIR_REJECT_Q30) begin
                accel_dir_hard = 1'b1;
                accel_direction_actionable = 1'b1;
                candidate_raw_level = 2'd0;
            end else begin
                accel_dir_soft = 1'b1;
                if (accel_raw_soft)
                    soft_metric_count = soft_metric_count + 3'd1;
            end
        end

        if (mag_unit_valid && saam_available &&
            (($unsigned(yaw_cross_sq_q30) < $unsigned(yaw_normal_rhs_q30)) ||
             (yaw_cross_q30 <= 0))) begin
            candidate_dynamic_flags = candidate_dynamic_flags | 8'h80;
            mode_flags_work = mode_flags_work | 8'h10;
            if (mag_dir_hard) begin
                mag_direction_actionable = 1'b1;
                candidate_raw_level = 2'd0;
            end
            else begin
                mag_dir_soft = 1'b1;
                if (mag_raw_soft || mn2_soft || innovation_soft || innovation_hard ||
                    (mode_l == MODE_IMU_TILT)) begin
                    soft_metric_count = soft_metric_count + 3'd1;
                    mag_direction_actionable = 1'b1;
                    candidate_raw_level = 2'd0;
                end
            end
        end

        if (candidate_raw_level != 2'd0) begin
            if (soft_metric_count == 0)
                candidate_raw_level = 2'd3;
            else if (soft_metric_count == 3'd1)
                candidate_raw_level = 2'd2;
            else
                candidate_raw_level = 2'd1;
        end

        if ((!accel_unit_valid) || accel_raw_hard || accel_direction_actionable) begin
            candidate_raw_mode = MODE_GYRO_ONLY;
        end else if ((!mag_unit_valid) || (!saam_available) || mag_raw_hard ||
                     mn2_hard || mag_direction_actionable) begin
            candidate_raw_mode = MODE_IMU_TILT;
        end else begin
            candidate_raw_mode = MODE_MARG;
        end
    end

    // Degradation is immediate after the configured bad-frame count; recovery advances one mode at a time.
    always_comb begin
        mode_decided_comb = mode_l;
        if (candidate_raw_mode > mode_l) begin
            if (mode_bad_count_l >= (BAD_FRAME_COUNT - 1))
                mode_decided_comb = candidate_raw_mode;
        end else if (candidate_raw_mode < mode_l) begin
            if (mode_good_count_l >= (GOOD_FRAME_COUNT - 1))
                mode_decided_comb = mode_l - 2'd1;
        end

        if (!accel_unit_valid || accel_raw_hard || accel_direction_actionable)
            mode_decided_comb = MODE_GYRO_ONLY;
        else if ((!mag_unit_valid) || (!saam_available) || mag_raw_hard ||
                 mn2_hard || mag_direction_actionable)
            mode_decided_comb = MODE_IMU_TILT;
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
            qacc_w              <= '0; qacc_x <= '0; qacc_y <= '0; qacc_z <= '0;
            qalign_w            <= '0; qalign_x <= '0; qalign_y <= '0; qalign_z <= '0;
            qmix_w              <= '0; qmix_x <= '0; qmix_y <= '0; qmix_z <= '0;
            qprev_w             <= ONE_Q30; qprev_x <= '0; qprev_y <= '0; qprev_z <= '0;
            qw_out              <= ONE_Q30; qx_out <= '0; qy_out <= '0; qz_out <= '0;
            have_state          <= 1'b0;
            saam_available      <= 1'b0;
            accel_unit_valid    <= 1'b0;
            mag_unit_valid      <= 1'b0;
            qacc_available      <= 1'b0;
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
            mode_out            <= MODE_MARG;
            raw_mode_out        <= MODE_MARG;
            mode_flags_out      <= '0;
            accel_dir_dot_out   <= '0;
            mag_hdir_dot_out    <= '0;
            qgyro_w_out         <= '0; qgyro_x_out <= '0;
            qgyro_y_out         <= '0; qgyro_z_out <= '0;
            qacc_w_out          <= '0; qacc_x_out <= '0;
            qacc_y_out          <= '0; qacc_z_out <= '0;
            beta_level_l        <= 2'd3;
            beta_level_pending  <= 2'd3;
            raw_level_l         <= 2'd3;
            bad_frame_count_l   <= '0;
            good_frame_count_l  <= '0;
            bad_frame_count_pending <= '0;
            good_frame_count_pending <= '0;
            accel_norm2_q30     <= '0;
            mag_norm2_q30       <= '0;
            mode_l              <= MODE_MARG;
            mode_pending        <= MODE_MARG;
            raw_mode_l          <= MODE_MARG;
            selected_mode_l     <= MODE_MARG;
            mode_bad_count_l    <= '0;
            mode_good_count_l   <= '0;
            mode_bad_count_pending <= '0;
            mode_good_count_pending <= '0;
            qterm_x2 <= '0; qterm_y2 <= '0; qterm_z2 <= '0;
            qterm_xy <= '0; qterm_xz <= '0; qterm_yz <= '0;
            qterm_wx <= '0; qterm_wy <= '0; qterm_wz <= '0;
            gpred_x <= '0; gpred_y <= '0; gpred_z <= '0;
            accel_dir_dot_q30 <= '0; mag_hdir_dot_q30 <= '0;
            yaw_g_x <= '0; yaw_g_y <= '0; yaw_s_x <= '0; yaw_s_y <= '0;
            saam_y2_q30 <= '0; saam_z2_q30 <= '0;
            saam_wz_q30 <= '0; saam_xy_q30 <= '0;
            yaw_g_norm2 <= '0; yaw_s_norm2 <= '0; yaw_cross_q30 <= '0;
            yaw_cross_sq_q30 <= '0; yaw_norm_product_q30 <= '0;
            yaw_normal_rhs_q30 <= '0; yaw_reject_rhs_q30 <= '0;
            dir_sum <= '0;
            ct_q30 <= '0; rho_q30 <= '0;
            yaw_half_w <= '0; yaw_half_z <= '0;
            pitch_half_w <= '0; pitch_half_y <= '0;
            roll_half_w <= '0; roll_half_x <= '0;
            qmul1_w <= '0; qmul1_x <= '0; qmul1_y <= '0; qmul1_z <= '0;
            qacc_pre_w <= '0; qacc_pre_x <= '0; qacc_pre_y <= '0; qacc_pre_z <= '0;
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
                        mode_pending <= mode_l;
                        mode_bad_count_pending <= mode_bad_count_l;
                        mode_good_count_pending <= mode_good_count_l;
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
                        mode_out <= mode_l;
                        raw_mode_out <= MODE_MARG;
                        mode_flags_out <= '0;
                        accel_dir_dot_out <= '0;
                        mag_hdir_dot_out <= '0;
                        qgyro_w_out <= '0; qgyro_x_out <= '0;
                        qgyro_y_out <= '0; qgyro_z_out <= '0;
                        qacc_w_out <= '0; qacc_x_out <= '0;
                        qacc_y_out <= '0; qacc_z_out <= '0;
                        saam_available <= 1'b1;
                        accel_unit_valid <= 1'b1;
                        mag_unit_valid <= 1'b1;
                        qacc_available <= 1'b0;
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
                            accel_unit_valid <= 1'b0;
                            state <= have_state ? C_GYRO_H_INIT : C_ERROR_LATCH;
                        end else begin
                            ax_u <= norm_out0; ay_u <= norm_out1; az_u <= norm_out2;
                            accel_unit_valid <= 1'b1;
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
                            mag_unit_valid <= 1'b0;
                            state <= have_state ? C_GYRO_H_INIT : C_ERROR_LATCH;
                        end else begin
                            mx_u <= norm_out0; my_u <= norm_out1; mz_u <= norm_out2;
                            mag_unit_valid <= 1'b1;
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
                        mn_sq_q30      <= '0;
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
                                state <= C_DIR_QTERM_INIT;
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
                            state <= C_DIR_QTERM_INIT;
                        end
                    end
                end

                C_DIR_QTERM_INIT: begin
                    op_index <= '0;
                    state <= C_DIR_QTERM_START;
                end

                C_DIR_QTERM_START: begin
                    if (!mul_busy) begin
                        case (op_index)
                            4'd0: begin mul_a <= qgyro_x; mul_b <= qgyro_x; end
                            4'd1: begin mul_a <= qgyro_y; mul_b <= qgyro_y; end
                            4'd2: begin mul_a <= qgyro_z; mul_b <= qgyro_z; end
                            4'd3: begin mul_a <= qgyro_x; mul_b <= qgyro_y; end
                            4'd4: begin mul_a <= qgyro_x; mul_b <= qgyro_z; end
                            4'd5: begin mul_a <= qgyro_y; mul_b <= qgyro_z; end
                            4'd6: begin mul_a <= qgyro_w; mul_b <= qgyro_x; end
                            4'd7: begin mul_a <= qgyro_w; mul_b <= qgyro_y; end
                            default: begin mul_a <= qgyro_w; mul_b <= qgyro_z; end
                        endcase
                        mul_start <= 1'b1;
                        state <= C_DIR_QTERM_WAIT;
                    end
                end

                C_DIR_QTERM_WAIT: begin
                    if (mul_done) begin
                        case (op_index)
                            4'd0: qterm_x2 <= sat_rshift_s64_rne(mul_product, 30);
                            4'd1: qterm_y2 <= sat_rshift_s64_rne(mul_product, 30);
                            4'd2: qterm_z2 <= sat_rshift_s64_rne(mul_product, 30);
                            4'd3: qterm_xy <= sat_rshift_s64_rne(mul_product, 30);
                            4'd4: qterm_xz <= sat_rshift_s64_rne(mul_product, 30);
                            4'd5: qterm_yz <= sat_rshift_s64_rne(mul_product, 30);
                            4'd6: qterm_wx <= sat_rshift_s64_rne(mul_product, 30);
                            4'd7: qterm_wy <= sat_rshift_s64_rne(mul_product, 30);
                            default: qterm_wz <= sat_rshift_s64_rne(mul_product, 30);
                        endcase
                        if (op_index == 4'd8)
                            state <= C_DIR_QTERM_FINAL;
                        else begin
                            op_index <= op_index + 4'd1;
                            state <= C_DIR_QTERM_START;
                        end
                    end
                end

                C_DIR_QTERM_FINAL: begin
                    gpred_x <= sat_double_q30(
                        $signed({qterm_xz[31], qterm_xz}) - $signed({qterm_wy[31], qterm_wy})
                    );
                    gpred_y <= sat_double_q30(
                        $signed({qterm_yz[31], qterm_yz}) + $signed({qterm_wx[31], qterm_wx})
                    );
                    gpred_z <= sat_s33_to_s32(
                        $signed({ONE_Q30[31], ONE_Q30}) -
                        (($signed({qterm_x2[31], qterm_x2}) +
                          $signed({qterm_y2[31], qterm_y2})) <<< 1)
                    );
                    yaw_g_x <= sat_s33_to_s32(
                        $signed({ONE_Q30[31], ONE_Q30}) -
                        (($signed({qterm_y2[31], qterm_y2}) +
                          $signed({qterm_z2[31], qterm_z2})) <<< 1)
                    );
                    yaw_g_y <= sat_double_q30(
                        $signed({qterm_wz[31], qterm_wz}) + $signed({qterm_xy[31], qterm_xy})
                    );
                    qgyro_w_out <= qgyro_w; qgyro_x_out <= qgyro_x;
                    qgyro_y_out <= qgyro_y; qgyro_z_out <= qgyro_z;
                    state <= C_DIR_ACCEL_INIT;
                end

                C_DIR_ACCEL_INIT: begin
                    if (!accel_unit_valid) begin
                        accel_dir_dot_q30 <= -$signed(ONE_Q30);
                        accel_dir_dot_out <= -$signed(ONE_Q30);
                        state <= saam_available ? C_DIR_SAAM_YAW_INIT : C_MODE_DECIDE;
                    end else begin
                        dir_sum <= '0;
                        op_index <= '0;
                        state <= C_DIR_ACCEL_START;
                    end
                end

                C_DIR_ACCEL_START: begin
                    if (!mul_busy) begin
                        case (op_index)
                            4'd0: begin mul_a <= ax_u; mul_b <= gpred_x; end
                            4'd1: begin mul_a <= ay_u; mul_b <= gpred_y; end
                            default: begin mul_a <= az_u; mul_b <= gpred_z; end
                        endcase
                        mul_start <= 1'b1;
                        state <= C_DIR_ACCEL_WAIT;
                    end
                end

                C_DIR_ACCEL_WAIT: begin
                    if (mul_done) begin
                        dir_sum <= dir_sum + mul_product;
                        if (op_index == 4'd2)
                            state <= C_DIR_ACCEL_FINAL;
                        else begin
                            op_index <= op_index + 4'd1;
                            state <= C_DIR_ACCEL_START;
                        end
                    end
                end

                C_DIR_ACCEL_FINAL: begin
                    accel_dir_dot_q30 <= clamp_unit_q30(sat_rshift_s64_rne(dir_sum, 30));
                    accel_dir_dot_out <= clamp_unit_q30(sat_rshift_s64_rne(dir_sum, 30));
                    state <= saam_available ? C_DIR_SAAM_YAW_INIT : C_MODE_DECIDE;
                end

                C_DIR_SAAM_YAW_INIT: begin
                    op_index <= '0;
                    state <= C_DIR_SAAM_YAW_START;
                end

                C_DIR_SAAM_YAW_START: begin
                    if (!mul_busy) begin
                        case (op_index)
                            4'd0: begin mul_a <= qsaam_y; mul_b <= qsaam_y; end
                            4'd1: begin mul_a <= qsaam_z; mul_b <= qsaam_z; end
                            4'd2: begin mul_a <= qsaam_w; mul_b <= qsaam_z; end
                            default: begin mul_a <= qsaam_x; mul_b <= qsaam_y; end
                        endcase
                        mul_start <= 1'b1;
                        state <= C_DIR_SAAM_YAW_WAIT;
                    end
                end

                C_DIR_SAAM_YAW_WAIT: begin
                    if (mul_done) begin
                        dir_product[op_index] <= mul_product;
                        case (op_index)
                            4'd0: saam_y2_q30 <= sat_rshift_s64_rne(mul_product, 30);
                            4'd1: saam_z2_q30 <= sat_rshift_s64_rne(mul_product, 30);
                            4'd2: saam_wz_q30 <= sat_rshift_s64_rne(mul_product, 30);
                            default: saam_xy_q30 <= sat_rshift_s64_rne(mul_product, 30);
                        endcase
                        if (op_index == 4'd3)
                            state <= C_DIR_SAAM_YAW_FINAL;
                        else begin
                            op_index <= op_index + 4'd1;
                            state <= C_DIR_SAAM_YAW_START;
                        end
                    end
                end

                C_DIR_SAAM_YAW_FINAL: begin
                    yaw_s_x <= sat_s33_to_s32(
                        $signed({ONE_Q30[31], ONE_Q30}) -
                        (($signed({saam_y2_q30[31], saam_y2_q30}) +
                          $signed({saam_z2_q30[31], saam_z2_q30})) <<< 1)
                    );
                    yaw_s_y <= sat_double_q30(
                        $signed({saam_wz_q30[31], saam_wz_q30}) +
                        $signed({saam_xy_q30[31], saam_xy_q30})
                    );
                    state <= C_DIR_HMAG_INIT;
                end

                C_DIR_HMAG_INIT: begin
                    op_index <= '0;
                    state <= C_DIR_HMAG_START;
                end

                C_DIR_HMAG_START: begin
                    if (!mul_busy) begin
                        case (op_index)
                            4'd0: begin mul_a <= yaw_g_x; mul_b <= yaw_s_x; end
                            4'd1: begin mul_a <= yaw_g_y; mul_b <= yaw_s_y; end
                            4'd2: begin mul_a <= yaw_g_x; mul_b <= yaw_g_x; end
                            4'd3: begin mul_a <= yaw_g_y; mul_b <= yaw_g_y; end
                            4'd4: begin mul_a <= yaw_s_x; mul_b <= yaw_s_x; end
                            4'd5: begin mul_a <= yaw_s_y; mul_b <= yaw_s_y; end
                            4'd6: begin mul_a <= yaw_cross_q30; mul_b <= yaw_cross_q30; end
                            4'd7: begin mul_a <= yaw_g_norm2; mul_b <= yaw_s_norm2; end
                            4'd8: begin mul_a <= $signed(MAG_DIR_NORMAL_COS2_Q30); mul_b <= yaw_norm_product_q30; end
                            default: begin mul_a <= $signed(MAG_DIR_REJECT_COS2_Q30); mul_b <= yaw_norm_product_q30; end
                        endcase
                        mul_start <= 1'b1;
                        state <= C_DIR_HMAG_WAIT;
                    end
                end

                C_DIR_HMAG_WAIT: begin
                    if (mul_done) begin
                        case (op_index)
                            4'd0, 4'd1, 4'd2, 4'd3, 4'd4, 4'd5: begin
                                dir_product[op_index] <= mul_product;
                                if (op_index == 4'd5)
                                    state <= C_DIR_HMAG_FINAL;
                                else begin
                                    op_index <= op_index + 4'd1;
                                    state <= C_DIR_HMAG_START;
                                end
                            end
                            4'd6: begin
                                yaw_cross_sq_q30 <= $signed(rshift_u64_rne(
                                    mul_product[63] ? $unsigned(-mul_product) : $unsigned(mul_product), 30));
                                op_index <= 4'd7;
                                state <= C_DIR_HMAG_START;
                            end
                            4'd7: begin
                                yaw_norm_product_q30 <= sat_rshift_s64_rne(mul_product, 30);
                                op_index <= 4'd8;
                                state <= C_DIR_HMAG_START;
                            end
                            4'd8: begin
                                yaw_normal_rhs_q30 <= sat_rshift_s64_rne(mul_product, 30);
                                op_index <= 4'd9;
                                state <= C_DIR_HMAG_START;
                            end
                            default: begin
                                yaw_reject_rhs_q30 <= sat_rshift_s64_rne(mul_product, 30);
                                state <= C_DIR_HMAG_FINAL;
                            end
                        endcase
                    end
                end

                C_DIR_HMAG_FINAL: begin
                    if (op_index == 4'd5) begin
                        yaw_cross_q30 <= sat_rshift_s64_rne(dir_product[0] + dir_product[1], 30);
                        yaw_g_norm2 <= sat_rshift_s64_rne(dir_product[2] + dir_product[3], 30);
                        yaw_s_norm2 <= sat_rshift_s64_rne(dir_product[4] + dir_product[5], 30);
                        op_index <= 4'd6;
                        state <= C_DIR_HMAG_START;
                    end else begin
                        mag_hdir_dot_q30 <= yaw_cross_q30;
                        mag_hdir_dot_out <= yaw_cross_q30;
                        state <= C_ALIGN_INIT;
                    end
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
                    state <= C_MODE_DECIDE;
                end

                // Latch diagnostics and select the MARG, IMU-tilt, or gyro-only reference path.
                C_MODE_DECIDE: begin
                    raw_level_l <= candidate_raw_level;
                    raw_level_out <= candidate_raw_level;
                    raw_mode_l <= candidate_raw_mode;
                    raw_mode_out <= candidate_raw_mode;
                    mode_out <= mode_decided_comb;
                    mode_flags_out <= mode_flags_work |
                        ((mode_decided_comb == MODE_IMU_TILT) ? 8'h40 : 8'h00) |
                        ((mode_decided_comb == MODE_GYRO_ONLY) ? 8'h80 : 8'h00);

                    if (candidate_raw_mode > mode_l) begin
                        mode_good_count_pending <= '0;
                        if (mode_bad_count_l < 4'hf)
                            mode_bad_count_pending <= mode_bad_count_l + 4'd1;
                        else
                            mode_bad_count_pending <= mode_bad_count_l;
                        if (mode_bad_count_l >= (BAD_FRAME_COUNT - 1))
                            mode_pending <= candidate_raw_mode;
                        else
                            mode_pending <= mode_l;
                    end else if (candidate_raw_mode < mode_l) begin
                        mode_bad_count_pending <= '0;
                        if (mode_good_count_l < 4'hf)
                            mode_good_count_pending <= mode_good_count_l + 4'd1;
                        else
                            mode_good_count_pending <= mode_good_count_l;
                        if (mode_good_count_l >= (GOOD_FRAME_COUNT - 1)) begin
                            mode_pending <= mode_l - 2'd1;
                            mode_good_count_pending <= '0;
                        end else
                            mode_pending <= mode_l;
                    end else begin
                        mode_bad_count_pending <= '0;
                        mode_good_count_pending <= '0;
                        mode_pending <= mode_l;
                    end

                    if (mode_decided_comb == MODE_MARG) begin
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
                                beta_active_l <= beta_for_level(beta_l,
                                    (candidate_raw_level < beta_level_l) ? candidate_raw_level : beta_level_l);
                                one_minus_beta_active_l <= $unsigned(ONE_Q30) - beta_for_level(beta_l,
                                    (candidate_raw_level < beta_level_l) ? candidate_raw_level : beta_level_l);
                                beta_used_out <= beta_for_level(beta_l,
                                    (candidate_raw_level < beta_level_l) ? candidate_raw_level : beta_level_l);
                                beta_level_out <= (candidate_raw_level < beta_level_l) ? candidate_raw_level : beta_level_l;
                                dynamic_flags_out <= candidate_dynamic_flags | DYN_FLAG_REDUCED;
                                status_work <= status_work | ST_BETA_DYNAMIC;
                            end else begin
                                beta_active_l <= beta_for_level(beta_l, beta_level_l);
                                one_minus_beta_active_l <= $unsigned(ONE_Q30) - beta_for_level(beta_l, beta_level_l);
                                beta_used_out <= beta_for_level(beta_l, beta_level_l);
                                beta_level_out <= beta_level_l;
                                dynamic_flags_out <= candidate_dynamic_flags |
                                    ((beta_level_l < 2'd3) ? DYN_FLAG_REDUCED : 8'h00);
                            end
                        end else begin
                            bad_frame_count_pending <= '0;
                            if (beta_level_l < 2'd3) begin
                                if (good_frame_count_l >= (GOOD_FRAME_COUNT - 1)) begin
                                    beta_level_pending <= beta_level_l + 2'd1;
                                    good_frame_count_pending <= '0;
                                    beta_active_l <= beta_for_level(beta_l, beta_level_l + 2'd1);
                                    one_minus_beta_active_l <= $unsigned(ONE_Q30) - beta_for_level(beta_l, beta_level_l + 2'd1);
                                    beta_used_out <= beta_for_level(beta_l, beta_level_l + 2'd1);
                                    beta_level_out <= beta_level_l + 2'd1;
                                    dynamic_flags_out <= ((beta_level_l + 2'd1) < 2'd3) ?
                                        (DYN_FLAG_REDUCED | DYN_FLAG_RECOVERY) : DYN_FLAG_RECOVERY;
                                end else begin
                                    good_frame_count_pending <= good_frame_count_l + 4'd1;
                                    beta_active_l <= beta_for_level(beta_l, beta_level_l);
                                    one_minus_beta_active_l <= $unsigned(ONE_Q30) - beta_for_level(beta_l, beta_level_l);
                                    beta_used_out <= beta_for_level(beta_l, beta_level_l);
                                    beta_level_out <= beta_level_l;
                                    dynamic_flags_out <= DYN_FLAG_REDUCED;
                                end
                            end else begin
                                good_frame_count_pending <= '0;
                                beta_active_l <= beta_l;
                                one_minus_beta_active_l <= $unsigned(ONE_Q30) - beta_l;
                                beta_used_out <= beta_l;
                                beta_level_out <= 2'd3;
                                dynamic_flags_out <= candidate_dynamic_flags;
                            end
                        end
                        selected_mode_l <= MODE_MARG;
                        state <= C_FUSE_INIT;
                    end else if (mode_decided_comb == MODE_IMU_TILT) begin
                        beta_level_pending <= 2'd0;
                        bad_frame_count_pending <= '0;
                        good_frame_count_pending <= '0;
                        beta_active_l <= beta_l >> IMU_BETA_SHIFT;
                        one_minus_beta_active_l <= $unsigned(ONE_Q30) - (beta_l >> IMU_BETA_SHIFT);
                        beta_used_out <= beta_l >> IMU_BETA_SHIFT;
                        beta_level_out <= 2'd1;
                        dynamic_flags_out <= candidate_dynamic_flags | DYN_FLAG_REDUCED;
                        selected_mode_l <= MODE_IMU_TILT;
                        state <= C_IMU_CT_INIT;
                    end else begin
                        beta_level_pending <= 2'd0;
                        bad_frame_count_pending <= '0;
                        good_frame_count_pending <= '0;
                        beta_active_l <= '0;
                        one_minus_beta_active_l <= $unsigned(ONE_Q30);
                        beta_used_out <= '0;
                        beta_level_out <= 2'd0;
                        dynamic_flags_out <= candidate_dynamic_flags | DYN_FLAG_REDUCED;
                        selected_mode_l <= MODE_GYRO_ONLY;
                        state <= C_GYRO_ONLY_PREP;
                    end
                end

                C_GYRO_ONLY_PREP: begin
                    qmix_w <= qgyro_w; qmix_x <= qgyro_x;
                    qmix_y <= qgyro_y; qmix_z <= qgyro_z;
                    state <= C_FINAL_NORM_START;
                end

                C_IMU_CT_INIT: begin
                    dir_sum <= '0;
                    op_index <= '0;
                    state <= C_IMU_CT_START;
                end

                C_IMU_CT_START: begin
                    if (!mul_busy) begin
                        if (op_index == 4'd0) begin mul_a <= ay_u; mul_b <= ay_u; end
                        else begin mul_a <= az_u; mul_b <= az_u; end
                        mul_start <= 1'b1;
                        state <= C_IMU_CT_WAIT;
                    end
                end

                C_IMU_CT_WAIT: begin
                    if (mul_done) begin
                        dir_sum <= dir_sum + mul_product;
                        if (op_index == 4'd1)
                            state <= C_IMU_CT_FINAL;
                        else begin
                            op_index <= 4'd1;
                            state <= C_IMU_CT_START;
                        end
                    end
                end

                C_IMU_CT_FINAL: begin
                    if ($unsigned(sat_rshift_s64_rne(dir_sum, 30)) <= $unsigned(IMU_GEOMETRY_MIN2_Q30)) begin
                        qacc_available <= 1'b0;
                        selected_mode_l <= MODE_GYRO_ONLY;
                        mode_pending <= MODE_GYRO_ONLY;
                        mode_out <= MODE_GYRO_ONLY;
                        mode_flags_out <= mode_flags_out | 8'h80;
                        state <= C_GYRO_ONLY_PREP;
                    end else if (!sqrt_busy) begin
                        sqrt_u_q30 <= $unsigned(sat_rshift_s64_rne(dir_sum, 30));
                        sqrt_start <= 1'b1;
                        state <= C_IMU_CT_SQRT_WAIT;
                    end
                end

                C_IMU_CT_SQRT_WAIT: begin
                    if (sqrt_done) begin
                        ct_q30 <= $signed(sqrt_result_q30);
                        state <= C_IMU_RHO_INIT;
                    end
                end

                C_IMU_RHO_INIT: begin
                    dir_sum <= '0;
                    op_index <= '0;
                    state <= C_IMU_RHO_START;
                end

                C_IMU_RHO_START: begin
                    if (!mul_busy) begin
                        if (op_index == 4'd0) begin mul_a <= yaw_g_x; mul_b <= yaw_g_x; end
                        else begin mul_a <= yaw_g_y; mul_b <= yaw_g_y; end
                        mul_start <= 1'b1;
                        state <= C_IMU_RHO_WAIT;
                    end
                end

                C_IMU_RHO_WAIT: begin
                    if (mul_done) begin
                        dir_sum <= dir_sum + mul_product;
                        if (op_index == 4'd1)
                            state <= C_IMU_RHO_FINAL;
                        else begin
                            op_index <= 4'd1;
                            state <= C_IMU_RHO_START;
                        end
                    end
                end

                C_IMU_RHO_FINAL: begin
                    if ($unsigned(sat_rshift_s64_rne(dir_sum, 30)) <= $unsigned(IMU_GEOMETRY_MIN2_Q30)) begin
                        qacc_available <= 1'b0;
                        selected_mode_l <= MODE_GYRO_ONLY;
                        mode_pending <= MODE_GYRO_ONLY;
                        mode_out <= MODE_GYRO_ONLY;
                        mode_flags_out <= mode_flags_out | 8'h80;
                        state <= C_GYRO_ONLY_PREP;
                    end else if (!sqrt_busy) begin
                        sqrt_u_q30 <= $unsigned(sat_rshift_s64_rne(dir_sum, 30));
                        sqrt_start <= 1'b1;
                        state <= C_IMU_RHO_SQRT_WAIT;
                    end
                end

                C_IMU_RHO_SQRT_WAIT: begin
                    if (sqrt_done) begin
                        rho_q30 <= $signed(sqrt_result_q30);
                        if (($signed({sqrt_result_q30[31], sqrt_result_q30}) +
                             $signed({yaw_g_x[31], yaw_g_x}) <= 0) && (yaw_g_y == 0)) begin
                            yaw_half_w <= '0;
                            yaw_half_z <= $signed(ONE_Q30);
                        end else begin
                            yaw_half_w <= sat_s33_to_s32(
                                $signed({sqrt_result_q30[31], sqrt_result_q30}) +
                                $signed({yaw_g_x[31], yaw_g_x})
                            );
                            yaw_half_z <= yaw_g_y;
                        end
                        pitch_half_w <= sat_s33_to_s32(
                            $signed({ONE_Q30[31], ONE_Q30}) + $signed({ct_q30[31], ct_q30})
                        );
                        pitch_half_y <= -ax_u;
                        if (($signed({ct_q30[31], ct_q30}) + $signed({az_u[31], az_u}) <= 0) &&
                            (ay_u == 0)) begin
                            roll_half_w <= '0;
                            roll_half_x <= $signed(ONE_Q30);
                        end else begin
                            roll_half_w <= sat_s33_to_s32(
                                $signed({ct_q30[31], ct_q30}) + $signed({az_u[31], az_u})
                            );
                            roll_half_x <= ay_u;
                        end
                        state <= C_IMU_QMUL1_INIT;
                    end
                end

                C_IMU_QMUL1_INIT: begin
                    op_index <= '0;
                    state <= C_IMU_QMUL1_START;
                end

                C_IMU_QMUL1_START: begin
                    if (!mul_busy) begin
                        case (op_index)
                            4'd0: begin mul_a <= yaw_half_w; mul_b <= pitch_half_w; end
                            4'd1: begin mul_a <= yaw_half_z; mul_b <= pitch_half_y; end
                            4'd2: begin mul_a <= yaw_half_w; mul_b <= pitch_half_y; end
                            default: begin mul_a <= yaw_half_z; mul_b <= pitch_half_w; end
                        endcase
                        mul_start <= 1'b1;
                        state <= C_IMU_QMUL1_WAIT;
                    end
                end

                C_IMU_QMUL1_WAIT: begin
                    if (mul_done) begin
                        qmul_product[op_index] <= mul_product;
                        if (op_index == 4'd3)
                            state <= C_IMU_QMUL1_FINAL;
                        else begin
                            op_index <= op_index + 4'd1;
                            state <= C_IMU_QMUL1_START;
                        end
                    end
                end

                C_IMU_QMUL1_FINAL: begin
                    qmul1_w <= sat_rshift_s64_rne(qmul_product[0], 31);
                    qmul1_x <= sat_rshift_s64_rne(-qmul_product[1], 31);
                    qmul1_y <= sat_rshift_s64_rne(qmul_product[2], 31);
                    qmul1_z <= sat_rshift_s64_rne(qmul_product[3], 31);
                    state <= C_IMU_QMUL2_INIT;
                end

                C_IMU_QMUL2_INIT: begin
                    op_index <= '0;
                    state <= C_IMU_QMUL2_START;
                end

                C_IMU_QMUL2_START: begin
                    if (!mul_busy) begin
                        case (op_index)
                            4'd0: begin mul_a <= qmul1_w; mul_b <= roll_half_w; end
                            4'd1: begin mul_a <= qmul1_x; mul_b <= roll_half_x; end
                            4'd2: begin mul_a <= qmul1_w; mul_b <= roll_half_x; end
                            4'd3: begin mul_a <= qmul1_x; mul_b <= roll_half_w; end
                            4'd4: begin mul_a <= qmul1_y; mul_b <= roll_half_w; end
                            4'd5: begin mul_a <= qmul1_z; mul_b <= roll_half_x; end
                            4'd6: begin mul_a <= qmul1_y; mul_b <= roll_half_x; end
                            default: begin mul_a <= qmul1_z; mul_b <= roll_half_w; end
                        endcase
                        mul_start <= 1'b1;
                        state <= C_IMU_QMUL2_WAIT;
                    end
                end

                C_IMU_QMUL2_WAIT: begin
                    if (mul_done) begin
                        qmul_product[op_index] <= mul_product;
                        if (op_index == 4'd7)
                            state <= C_IMU_QMUL2_FINAL;
                        else begin
                            op_index <= op_index + 4'd1;
                            state <= C_IMU_QMUL2_START;
                        end
                    end
                end

                C_IMU_QMUL2_FINAL: begin
                    qacc_pre_w <= rshift_s64_to_s36_rne(qmul_product[0] - qmul_product[1], 30);
                    qacc_pre_x <= rshift_s64_to_s36_rne(qmul_product[2] + qmul_product[3], 30);
                    qacc_pre_y <= rshift_s64_to_s36_rne(qmul_product[4] + qmul_product[5], 30);
                    qacc_pre_z <= rshift_s64_to_s36_rne(-qmul_product[6] + qmul_product[7], 30);
                    state <= C_IMU_QACC_NORM_START;
                end

                C_IMU_QACC_NORM_START: begin
                    if (!norm_busy) begin
                        norm_dim3 <= 1'b0;
                        norm_in0 <= qacc_pre_w; norm_in1 <= qacc_pre_x;
                        norm_in2 <= qacc_pre_y; norm_in3 <= qacc_pre_z;
                        norm_start <= 1'b1;
                        state <= C_IMU_QACC_NORM_WAIT;
                    end
                end

                C_IMU_QACC_NORM_WAIT: begin
                    if (norm_done) begin
                        if (norm_zero) begin
                            qacc_available <= 1'b0;
                            selected_mode_l <= MODE_GYRO_ONLY;
                            mode_pending <= MODE_GYRO_ONLY;
                            mode_out <= MODE_GYRO_ONLY;
                            mode_flags_out <= mode_flags_out | 8'h80;
                            state <= C_GYRO_ONLY_PREP;
                        end else begin
                            qacc_w <= norm_out0; qacc_x <= norm_out1;
                            qacc_y <= norm_out2; qacc_z <= norm_out3;
                            qacc_w_out <= norm_out0; qacc_x_out <= norm_out1;
                            qacc_y_out <= norm_out2; qacc_z_out <= norm_out3;
                            qacc_available <= 1'b1;
                            state <= C_IMU_ALIGN_INIT;
                        end
                    end
                end

                C_IMU_ALIGN_INIT: begin
                    dot_sum <= '0;
                    op_index <= '0;
                    state <= C_IMU_ALIGN_MUL_START;
                end

                C_IMU_ALIGN_MUL_START: begin
                    if (!mul_busy) begin
                        mul_a <= vec_qgyro(op_index[1:0]);
                        mul_b <= vec_qacc(op_index[1:0]);
                        mul_start <= 1'b1;
                        state <= C_IMU_ALIGN_MUL_WAIT;
                    end
                end

                C_IMU_ALIGN_MUL_WAIT: begin
                    if (mul_done) begin
                        dot_sum <= dot_sum + mul_product;
                        if (op_index == 4'd3)
                            state <= C_IMU_ALIGN_DECIDE;
                        else begin
                            op_index <= op_index + 4'd1;
                            state <= C_IMU_ALIGN_MUL_START;
                        end
                    end
                end

                C_IMU_ALIGN_DECIDE: begin
                    if (dot_sum[63]) begin
                        qalign_w <= -qacc_w; qalign_x <= -qacc_x;
                        qalign_y <= -qacc_y; qalign_z <= -qacc_z;
                    end else begin
                        qalign_w <= qacc_w; qalign_x <= qacc_x;
                        qalign_y <= qacc_y; qalign_z <= qacc_z;
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

                // State is committed only when the consumer accepts the output frame.
                C_OUTPUT: begin
                    if (out_valid && out_ready) begin
                        if (commit_pending) begin
                            qprev_w <= qw_out; qprev_x <= qx_out;
                            qprev_y <= qy_out; qprev_z <= qz_out;
                            have_state <= 1'b1;
                            beta_level_l <= beta_level_pending;
                            bad_frame_count_l <= bad_frame_count_pending;
                            good_frame_count_l <= good_frame_count_pending;
                            mode_l <= mode_pending;
                            mode_bad_count_l <= mode_bad_count_pending;
                            mode_good_count_l <= mode_good_count_pending;
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
