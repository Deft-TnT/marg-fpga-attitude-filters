`timescale 1ns/1ps
// Unsigned restoring divider: quotient = dividend / divisor.
// It uses 64 deterministic cycles and does not infer a wide combinational divider.
module udiv64_by35_seq (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [63:0] dividend,
    input  logic [34:0] divisor,
    output logic        busy,
    output logic        done,
    output logic        div_by_zero,
    output logic [63:0] quotient
);
    logic [63:0] dividend_shift;
    logic [63:0] quotient_work;
    logic [35:0] remainder_work;
    logic [34:0] divisor_reg;
    logic [6:0]  count;
    logic [35:0] remainder_shift;
    logic        quotient_bit;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            busy            <= 1'b0;
            done            <= 1'b0;
            div_by_zero     <= 1'b0;
            quotient         <= '0;
            dividend_shift   <= '0;
            quotient_work    <= '0;
            remainder_work   <= '0;
            divisor_reg      <= '0;
            count            <= '0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                if (start) begin
                    div_by_zero <= (divisor == '0);
                    if (divisor == '0) begin
                        quotient <= '0;
                        done     <= 1'b1;
                    end else begin
                        dividend_shift <= dividend;
                        quotient_work  <= '0;
                        remainder_work <= '0;
                        divisor_reg    <= divisor;
                        count          <= '0;
                        busy           <= 1'b1;
                    end
                end
            end else begin
                remainder_shift = {remainder_work[34:0], dividend_shift[63]};
                quotient_bit    = (remainder_shift >= {1'b0, divisor_reg});
                dividend_shift  <= {dividend_shift[62:0], 1'b0};
                quotient_work   <= {quotient_work[62:0], quotient_bit};
                if (quotient_bit)
                    remainder_work <= remainder_shift - {1'b0, divisor_reg};
                else
                    remainder_work <= remainder_shift;

                if (count == 7'd63) begin
                    // One dividend bit and one quotient bit are processed each cycle.
                    quotient <= {quotient_work[62:0], quotient_bit};
                    busy     <= 1'b0;
                    done     <= 1'b1;
                end
                count <= count + 7'd1;
            end
        end
    end
endmodule
