// Created by Wang Jialin.
// See README.md for interfaces and usage.

`timescale 1ns/1ps
module standard_sqrt_q30_seq (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,
    input  logic [31:0] u_q30,
    output logic        busy,
    output logic        done,
    output logic [31:0] root_q30
);
    logic [63:0] radicand;
    logic [33:0] remainder_work;
    logic [31:0] root_work;
    logic [5:0] count;
    logic [33:0] remainder_shift;
    logic [33:0] trial;
    logic [33:0] remainder_next;
    logic [31:0] root_next;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            busy <= 1'b0;
            done <= 1'b0;
            root_q30 <= '0;
            radicand <= '0;
            remainder_work <= '0;
            root_work <= '0;
            count <= '0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                if (start) begin
                    radicand <= {2'b00, u_q30, 30'b0};
                    remainder_work <= '0;
                    root_work <= '0;
                    count <= '0;
                    busy <= 1'b1;
                end
            end else begin
                remainder_shift = {remainder_work[31:0], radicand[63:62]};
                trial = {root_work, 2'b01};
                if (remainder_shift >= trial) begin
                    remainder_next = remainder_shift - trial;
                    root_next = {root_work[30:0], 1'b1};
                end else begin
                    remainder_next = remainder_shift;
                    root_next = {root_work[30:0], 1'b0};
                end
                radicand <= {radicand[61:0], 2'b00};
                remainder_work <= remainder_next;
                root_work <= root_next;
                if (count == 6'd31) begin
                    root_q30 <= root_next + ((remainder_next > {2'b00, root_next}) ? 32'd1 : 32'd0);
                    busy <= 1'b0;
                    done <= 1'b1;
                end
                count <= count + 6'd1;
            end
        end
    end
endmodule
