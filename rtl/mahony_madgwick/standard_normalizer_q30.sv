// Created by Wang Jialin.
// See README.md for interfaces and usage.

`timescale 1ns/1ps
module standard_normalizer_q30 (
    input  logic              clk,
    input  logic              rst_n,
    input  logic              start,
    input  logic [2:0]        dim,
    input  logic signed [31:0] v0_in,
    input  logic signed [31:0] v1_in,
    input  logic signed [31:0] v2_in,
    input  logic signed [31:0] v3_in,
    output logic              busy,
    output logic              done,
    output logic              zero_input,
    output logic signed [31:0] v0_out,
    output logic signed [31:0] v1_out,
    output logic signed [31:0] v2_out,
    output logic signed [31:0] v3_out
);
    import ahrs_fixed_pkg::*;

    typedef enum logic [2:0] {IDLE, SQRT_START, SQRT_WAIT, DIV_START, DIV_WAIT, FINISH} state_t;
    state_t state;
    logic [2:0] dim_reg;
    logic signed [31:0] v0_reg, v1_reg, v2_reg, v3_reg;
    logic [2:0] component;
    logic [63:0] norm2_q60;
    logic [31:0] norm2_q30;
    logic [31:0] norm_q30;
    logic signed [63:0] product0, product1, product2, product3;

    logic sqrt_start, sqrt_busy, sqrt_done;
    logic [31:0] sqrt_root;
    logic div_start, div_busy, div_done, div_by_zero;
    logic [63:0] div_dividend, div_quotient;
    logic [31:0] div_divisor;
    logic signed [31:0] component_value;

    always_comb begin
        product0 = v0_reg * v0_reg;
        product1 = v1_reg * v1_reg;
        product2 = v2_reg * v2_reg;
        product3 = v3_reg * v3_reg;
        norm2_q60 = $unsigned(product0);
        if (dim_reg >= 3'd2)
            norm2_q60 = norm2_q60 + $unsigned(product1);
        if (dim_reg >= 3'd3)
            norm2_q60 = norm2_q60 + $unsigned(product2);
        if (dim_reg >= 3'd4)
            norm2_q60 = norm2_q60 + $unsigned(product3);
        norm2_q30 = (|norm2_q60[63:62]) ? 32'hffff_ffff : norm2_q60[61:30];
        case (component)
            3'd0: component_value = v0_reg;
            3'd1: component_value = v1_reg;
            3'd2: component_value = v2_reg;
            default: component_value = v3_reg;
        endcase
        sqrt_start = (state == SQRT_START);
        div_start = (state == DIV_START);
        div_dividend = {2'b00, abs_s32(component_value), 30'b0};
        div_divisor = norm_q30;
    end

    standard_sqrt_q30_seq u_sqrt (
        .clk(clk), .rst_n(rst_n), .start(sqrt_start), .u_q30(norm2_q30),
        .busy(sqrt_busy), .done(sqrt_done), .root_q30(sqrt_root)
    );
    standard_div_u64_u32_seq u_div (
        .clk(clk), .rst_n(rst_n), .start(div_start),
        .dividend(div_dividend), .divisor(div_divisor), .busy(div_busy),
        .done(div_done), .div_by_zero(div_by_zero), .quotient(div_quotient)
    );

    assign busy = (state != IDLE);
    assign done = (state == FINISH);

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            dim_reg <= '0;
            v0_reg <= '0; v1_reg <= '0; v2_reg <= '0; v3_reg <= '0;
            component <= '0;
            norm_q30 <= '0;
            zero_input <= 1'b0;
            v0_out <= '0; v1_out <= '0; v2_out <= '0; v3_out <= '0;
        end else begin
            case (state)
                IDLE: if (start) begin
                    dim_reg <= (dim == 3'd4) ? 3'd4 : 3'd3;
                    v0_reg <= v0_in; v1_reg <= v1_in; v2_reg <= v2_in; v3_reg <= v3_in;
                    zero_input <= 1'b0;
                    state <= SQRT_START;
                end
                SQRT_START: state <= SQRT_WAIT;
                SQRT_WAIT: if (sqrt_done) begin
                    norm_q30 <= sqrt_root;
                    if (sqrt_root == '0) begin
                        zero_input <= 1'b1;
                        v0_out <= '0; v1_out <= '0; v2_out <= '0; v3_out <= '0;
                        state <= FINISH;
                    end else begin
                        component <= '0;
                        state <= DIV_START;
                    end
                end
                DIV_START: state <= DIV_WAIT;
                DIV_WAIT: if (div_done) begin
                    case (component)
                        3'd0: v0_out <= signed_quotient_q30(div_quotient, component_value[31]);
                        3'd1: v1_out <= signed_quotient_q30(div_quotient, component_value[31]);
                        3'd2: v2_out <= signed_quotient_q30(div_quotient, component_value[31]);
                        default: v3_out <= signed_quotient_q30(div_quotient, component_value[31]);
                    endcase
                    if (component + 3'd1 >= dim_reg)
                        state <= FINISH;
                    else begin
                        component <= component + 3'd1;
                        state <= DIV_START;
                    end
                end
                FINISH: state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end
endmodule
