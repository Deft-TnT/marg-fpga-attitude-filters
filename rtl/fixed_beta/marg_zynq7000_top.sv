`timescale 1ns/1ps
// Board-neutral, I/O-feasible top for xc7z020clg400-2.
//
// The estimator core itself has one clear parallel input frame.  This wrapper
// serializes that frame over a 32-bit ready/valid stream, avoiding an
// impossible 400+ pin top-level interface on the CLG400 package.  A Zynq PS,
// AXI DMA, UART or SPI front-end can drive this stream.
//
// Input stream, one 32-bit word per handshake (s_axis_tlast on word 10):
//   0 ax, 1 ay, 2 az, 3 mx, 4 my, 5 mz, 6 wx, 7 wy, 8 wz, 9 dt, 10 beta
// Output stream, one 32-bit word per handshake (m_axis_tlast on word 4):
//   0 qw, 1 qx, 2 qy, 3 qz, 4 {24'd0,status[7:0]}
module marg_zynq7000_top #(
    // Default 20 ms (UQ30 seconds); tune for a known sensor/update rate.
    parameter logic [31:0] MAX_DT_Q30 = 32'd21_474_836
) (
    input  logic        clk,
    input  logic        rst_n,

    input  logic [31:0] s_axis_tdata,
    input  logic        s_axis_tvalid,
    output logic        s_axis_tready,
    input  logic        s_axis_tlast,

    output logic [31:0] m_axis_tdata,
    output logic        m_axis_tvalid,
    input  logic        m_axis_tready,
    output logic        m_axis_tlast,
    output logic        busy
);
    logic signed [31:0] ax_buf, ay_buf, az_buf, mx_buf, my_buf, mz_buf;
    logic signed [31:0] wx_buf, wy_buf, wz_buf;
    logic        [31:0] dt_buf, beta_buf;
    logic [3:0]         input_word_index;
    logic               frame_pending;

    logic               core_in_valid, core_in_ready;
    logic               core_out_valid, core_out_ready, core_busy;
    logic signed [31:0] core_qw, core_qx, core_qy, core_qz;
    logic [7:0]         core_status;

    logic               output_active;
    logic [2:0]         output_word_index;
    logic signed [31:0] qw_hold, qx_hold, qy_hold, qz_hold;
    logic [7:0]         status_hold;

    assign s_axis_tready = !frame_pending;
    assign core_in_valid = frame_pending;
    assign core_out_ready = !output_active;
    assign m_axis_tvalid = output_active;
    assign m_axis_tlast  = output_active && (output_word_index == 3'd4);
    assign busy = core_busy || frame_pending || output_active || (input_word_index != 0);

    always_comb begin
        case (output_word_index)
            3'd0: m_axis_tdata = qw_hold;
            3'd1: m_axis_tdata = qx_hold;
            3'd2: m_axis_tdata = qy_hold;
            3'd3: m_axis_tdata = qz_hold;
            default: m_axis_tdata = {24'd0, status_hold};
        endcase
    end

    marg_estimator_core #(
        .MAX_DT_Q30(MAX_DT_Q30)
    ) u_core (
        .clk(clk), .rst_n(rst_n), .in_valid(core_in_valid), .in_ready(core_in_ready),
        .ax_in(ax_buf), .ay_in(ay_buf), .az_in(az_buf),
        .mx_in(mx_buf), .my_in(my_buf), .mz_in(mz_buf),
        .wx_in(wx_buf), .wy_in(wy_buf), .wz_in(wz_buf),
        .dt_in(dt_buf), .beta_in(beta_buf),
        .out_valid(core_out_valid), .out_ready(core_out_ready),
        .qw_out(core_qw), .qx_out(core_qx), .qy_out(core_qy), .qz_out(core_qz),
        .status_out(core_status), .busy(core_busy)
    );

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            ax_buf            <= '0; ay_buf <= '0; az_buf <= '0;
            mx_buf            <= '0; my_buf <= '0; mz_buf <= '0;
            wx_buf            <= '0; wy_buf <= '0; wz_buf <= '0;
            dt_buf            <= '0; beta_buf <= '0;
            input_word_index  <= '0;
            frame_pending     <= 1'b0;
            output_active     <= 1'b0;
            output_word_index <= '0;
            qw_hold           <= '0; qx_hold <= '0; qy_hold <= '0; qz_hold <= '0;
            status_hold       <= '0;
        end else begin
            // The core consumes a complete buffered frame exactly once.
            if (core_in_valid && core_in_ready)
                frame_pending <= 1'b0;

            // Input parser.  An early or missing tlast discards the partial
            // frame and returns to word zero; no malformed frame reaches core.
            if (s_axis_tvalid && s_axis_tready) begin
                case (input_word_index)
                    4'd0:  ax_buf   <= $signed(s_axis_tdata);
                    4'd1:  ay_buf   <= $signed(s_axis_tdata);
                    4'd2:  az_buf   <= $signed(s_axis_tdata);
                    4'd3:  mx_buf   <= $signed(s_axis_tdata);
                    4'd4:  my_buf   <= $signed(s_axis_tdata);
                    4'd5:  mz_buf   <= $signed(s_axis_tdata);
                    4'd6:  wx_buf   <= $signed(s_axis_tdata);
                    4'd7:  wy_buf   <= $signed(s_axis_tdata);
                    4'd8:  wz_buf   <= $signed(s_axis_tdata);
                    4'd9:  dt_buf   <= s_axis_tdata;
                    default: beta_buf <= s_axis_tdata;
                endcase

                if (s_axis_tlast) begin
                    if (input_word_index == 4'd10)
                        frame_pending <= 1'b1;
                    input_word_index <= '0;
                end else if (input_word_index == 4'd10) begin
                    input_word_index <= '0;
                end else begin
                    input_word_index <= input_word_index + 4'd1;
                end
            end

            // Core result serializer.  The core holds its result until this
            // wrapper latches it, then the five-word output stream may stall.
            if (core_out_valid && core_out_ready) begin
                qw_hold           <= core_qw;
                qx_hold           <= core_qx;
                qy_hold           <= core_qy;
                qz_hold           <= core_qz;
                status_hold       <= core_status;
                output_word_index <= '0;
                output_active     <= 1'b1;
            end else if (output_active && m_axis_tready) begin
                if (output_word_index == 3'd4) begin
                    output_active     <= 1'b0;
                    output_word_index <= '0;
                end else begin
                    output_word_index <= output_word_index + 3'd1;
                end
            end
        end
    end
endmodule
