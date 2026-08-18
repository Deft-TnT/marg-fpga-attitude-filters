`timescale 1ns/1ps
// Q30 square root.  Input is u in U(32,30); output is sqrt(u) in U(32,30).
// The implementation forms sqrt(u_int << 30) with a radix-4 restoring core.
module sqrt_q30_seq (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [31:0] u_q30,
    output logic        busy,
    output logic        done,
    output logic [31:0] root_q30
);
    logic [63:0] radicand_shift;
    logic [33:0] remainder_work;
    logic [31:0] root_work;
    logic [5:0]  count;
    logic [33:0] remainder_shift;
    logic [33:0] trial;
    logic [31:0] root_next;
    logic [33:0] remainder_next;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            busy            <= 1'b0;
            done            <= 1'b0;
            root_q30        <= '0;
            radicand_shift  <= '0;
            remainder_work  <= '0;
            root_work       <= '0;
            count           <= '0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                if (start) begin
                    radicand_shift <= {2'b00, u_q30, 30'b0};
                    remainder_work <= '0;
                    root_work      <= '0;
                    count          <= '0;
                    busy           <= 1'b1;
                end
            end else begin
                remainder_shift = {remainder_work[31:0], radicand_shift[63:62]};
                trial           = {root_work, 2'b01};
                if (remainder_shift >= trial) begin
                    remainder_next = remainder_shift - trial;
                    root_next      = {root_work[30:0], 1'b1};
                end else begin
                    remainder_next = remainder_shift;
                    root_next      = {root_work[30:0], 1'b0};
                end
                radicand_shift <= {radicand_shift[61:0], 2'b00};
                remainder_work <= remainder_next;
                root_work      <= root_next;

                if (count == 6'd31) begin
                    // Nearest-integer rounding: the halfway threshold is root.
                    root_q30 <= root_next + ((remainder_next > {2'b00, root_next}) ? 32'd1 : 32'd0);
                    busy     <= 1'b0;
                    done     <= 1'b1;
                end
                count <= count + 6'd1;
            end
        end
    end
endmodule
