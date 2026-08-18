`timescale 1ns/1ps
// Independent fixed-point helpers for the conventional Mahony/Madgwick
// baselines.  No SAAM or rational-normalization primitive is used here.
package ahrs_fixed_pkg;
    localparam logic signed [31:0] Q30_ONE      = 32'sh4000_0000;
    localparam logic signed [31:0] Q30_HALF     = 32'sh2000_0000;
    localparam logic signed [31:0] Q30_NEG_ONE  = -32'sh4000_0000;

    function automatic logic signed [31:0] sat_s64(input logic signed [63:0] value);
        begin
            if (value > 64'sd2147483647)
                sat_s64 = 32'sh7fff_ffff;
            else if (value < -64'sd2147483648)
                sat_s64 = 32'sh8000_0000;
            else
                sat_s64 = value[31:0];
        end
    endfunction

    function automatic logic signed [31:0] rshift_s64_rne(
        input logic signed [63:0] value,
        input integer shift
    );
        logic [63:0] magnitude;
        logic [63:0] rounded;
        logic [63:0] remainder;
        logic [63:0] half;
        begin
            magnitude = value[63] ? (~value + 64'd1) : value;
            rounded = magnitude >> shift;
            remainder = magnitude & ((64'd1 << shift) - 64'd1);
            half = (64'd1 << (shift - 1));
            if ((remainder > half) || ((remainder == half) && rounded[0]))
                rounded = rounded + 64'd1;
            if (value[63]) begin
                if (rounded >= 64'd2147483648)
                    rshift_s64_rne = 32'sh8000_0000;
                else
                    rshift_s64_rne = -$signed({1'b0, rounded[30:0]});
            end else if (rounded > 64'd2147483647) begin
                rshift_s64_rne = 32'sh7fff_ffff;
            end else begin
                rshift_s64_rne = $signed(rounded[31:0]);
            end
        end
    endfunction

    function automatic logic signed [31:0] q30_mul(
        input logic signed [31:0] left,
        input logic signed [31:0] right
    );
        logic signed [63:0] product;
        begin
            product = left * right;
            q30_mul = rshift_s64_rne(product, 30);
        end
    endfunction

    // left is S(32,30); right is angular rate S(32,24).  The result is Q30.
    function automatic logic signed [31:0] q30_mul_q24(
        input logic signed [31:0] left,
        input logic signed [31:0] right
    );
        logic signed [63:0] product;
        begin
            product = left * right;
            q30_mul_q24 = rshift_s64_rne(product, 24);
        end
    endfunction

    function automatic logic signed [31:0] q30_add(
        input logic signed [31:0] left,
        input logic signed [31:0] right
    );
        begin
            q30_add = sat_s64($signed({{32{left[31]}}, left}) + $signed({{32{right[31]}}, right}));
        end
    endfunction

    function automatic logic signed [31:0] q30_sub(
        input logic signed [31:0] left,
        input logic signed [31:0] right
    );
        begin
            q30_sub = sat_s64($signed({{32{left[31]}}, left}) - $signed({{32{right[31]}}, right}));
        end
    endfunction

    function automatic logic signed [31:0] q30_neg(input logic signed [31:0] value);
        begin
            if (value == 32'sh8000_0000)
                q30_neg = 32'sh7fff_ffff;
            else
                q30_neg = -value;
        end
    endfunction

    function automatic logic signed [31:0] q30_half(input logic signed [31:0] value);
        begin
            q30_half = value >>> 1;
        end
    endfunction

    function automatic logic signed [31:0] q30_double(input logic signed [31:0] value);
        begin
            q30_double = sat_s64($signed({{32{value[31]}}, value}) <<< 1);
        end
    endfunction

    function automatic logic [31:0] abs_s32(input logic signed [31:0] value);
        begin
            if (value == 32'sh8000_0000)
                abs_s32 = 32'h7fff_ffff;
            else if (value[31])
                abs_s32 = -value;
            else
                abs_s32 = value;
        end
    endfunction

    function automatic logic signed [31:0] signed_quotient_q30(
        input logic [63:0] quotient,
        input logic negative
    );
        begin
            if (negative) begin
                if (|quotient[63:31])
                    signed_quotient_q30 = 32'sh8000_0000;
                else
                    signed_quotient_q30 = -$signed({1'b0, quotient[30:0]});
            end else if (|quotient[63:31]) begin
                signed_quotient_q30 = 32'sh7fff_ffff;
            end else begin
                signed_quotient_q30 = $signed(quotient[31:0]);
            end
        end
    endfunction
endpackage
