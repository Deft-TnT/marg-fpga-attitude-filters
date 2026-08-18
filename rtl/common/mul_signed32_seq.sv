`timescale 1ns/1ps
// One registered signed multiplier.  The estimator deliberately shares this
// unit across arithmetic phases to trade latency for DSP48E1 usage.
module mul_signed32_seq (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,
    input  logic signed [31:0] a,
    input  logic signed [31:0] b,
    output logic               busy,
    output logic               done,
    output logic signed [63:0] product
);
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            busy    <= 1'b0;
            done    <= 1'b0;
            product <= '0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                if (start) begin
                    product <= a * b;
                    busy    <= 1'b1;
                end
            end else begin
                busy <= 1'b0;
                done <= 1'b1;
            end
        end
    end
endmodule
