`timescale 1ns/1ps
// SPDX-License-Identifier: MIT
// Fixed-point helpers for the MARG estimator.
// Convention: S(W,F) means W-bit two's-complement, F fractional bits.
package marg_fixed_pkg;
    localparam logic signed [31:0] ONE_Q30     = 32'sh4000_0000;
    localparam logic signed [31:0] NEG_ONE_Q30 = -32'sh4000_0000;
    localparam logic [31:0]        MN2_MIN_Q30 = 32'd1_024;     // ~9.54e-7

    function automatic logic signed [31:0] clamp_unit_q30(
        input logic signed [31:0] value
    );
        begin
            if (value > ONE_Q30)
                clamp_unit_q30 = ONE_Q30;
            else if (value < NEG_ONE_Q30)
                clamp_unit_q30 = NEG_ONE_Q30;
            else
                clamp_unit_q30 = value;
        end
    endfunction

    function automatic logic signed [31:0] sat_s33_to_s32(
        input logic signed [32:0] value
    );
        begin
            if (value > 33'sh0_7fff_ffff)
                sat_s33_to_s32 = 32'sh7fff_ffff;
            else if (value < -33'sh0_8000_0000)
                sat_s33_to_s32 = 32'sh8000_0000;
            else
                sat_s33_to_s32 = value[31:0];
        end
    endfunction

    // Signed, symmetric round-to-nearest-even right shift with S32 saturation.
    function automatic logic signed [31:0] sat_rshift_s64_rne(
        input logic signed [63:0] value,
        input integer shift
    );
        logic [63:0] magnitude;
        logic [63:0] rounded;
        logic [63:0] remainder;
        logic [63:0] half;
        begin
            magnitude = value[63] ? (~value + 64'd1) : value;
            rounded   = magnitude >> shift;
            remainder = magnitude & ((64'd1 << shift) - 64'd1);
            half      = (64'd1 << (shift - 1));
            if ((remainder > half) || ((remainder == half) && rounded[0]))
                rounded = rounded + 64'd1;

            if (value[63]) begin
                if (rounded >= 64'd2_147_483_648)
                    sat_rshift_s64_rne = 32'sh8000_0000;
                else
                    sat_rshift_s64_rne = -$signed({1'b0, rounded[30:0]});
            end else begin
                if (rounded > 64'd2_147_483_647)
                    sat_rshift_s64_rne = 32'sh7fff_ffff;
                else
                    sat_rshift_s64_rne = $signed(rounded[31:0]);
            end
        end
    endfunction

    // Same rounding rule, retaining a protected S(36,30) result.
    function automatic logic signed [35:0] rshift_s64_to_s36_rne(
        input logic signed [63:0] value,
        input integer shift
    );
        logic [63:0] magnitude;
        logic [63:0] rounded;
        logic [63:0] remainder;
        logic [63:0] half;
        begin
            magnitude = value[63] ? (~value + 64'd1) : value;
            rounded   = magnitude >> shift;
            remainder = magnitude & ((64'd1 << shift) - 64'd1);
            half      = (64'd1 << (shift - 1));
            if ((remainder > half) || ((remainder == half) && rounded[0]))
                rounded = rounded + 64'd1;

            if (value[63]) begin
                if (rounded >= 64'd34_359_738_368)
                    rshift_s64_to_s36_rne = 36'sh8_0000_0000;
                else
                    rshift_s64_to_s36_rne = -$signed({1'b0, rounded[34:0]});
            end else begin
                if (rounded > 64'd34_359_738_367)
                    rshift_s64_to_s36_rne = 36'sh7_ffff_ffff;
                else
                    rshift_s64_to_s36_rne = $signed(rounded[35:0]);
            end
        end
    endfunction

    function automatic logic [31:0] rshift_u64_rne(
        input logic [63:0] value,
        input integer shift
    );
        logic [63:0] rounded;
        logic [63:0] remainder;
        logic [63:0] half;
        begin
            rounded   = value >> shift;
            remainder = value & ((64'd1 << shift) - 64'd1);
            half      = (64'd1 << (shift - 1));
            if ((remainder > half) || ((remainder == half) && rounded[0]))
                rounded = rounded + 64'd1;
            rshift_u64_rne = (|rounded[63:32]) ? 32'hffff_ffff : rounded[31:0];
        end
    endfunction

    function automatic logic signed [63:0] q30_to_q60(
        input logic signed [31:0] value
    );
        begin
            q30_to_q60 = $signed({{32{value[31]}}, value}) <<< 30;
        end
    endfunction

endpackage
