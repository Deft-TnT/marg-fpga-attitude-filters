`timescale 1ns/1ps
// Four-iteration rational normalizer from paper Eq. (12).
// One sequential multiplier and one iterative divider are shared by every
// iteration.  Inputs are S(36,30), outputs are S(32,30).
module rational_normalizer #(
    parameter int ITERATIONS = 4
) (
    input  logic               clk,
    input  logic               rst_n,
    input  logic               start,
    input  logic               dim3,
    input  logic signed [35:0] in0,
    input  logic signed [35:0] in1,
    input  logic signed [35:0] in2,
    input  logic signed [35:0] in3,
    output logic               busy,
    output logic               done,
    output logic               zero_input,
    output logic signed [31:0] out0,
    output logic signed [31:0] out1,
    output logic signed [31:0] out2,
    output logic signed [31:0] out3
);
    import marg_fixed_pkg::*;

    typedef enum logic [3:0] {
        N_IDLE,
        N_SQ_INIT,
        N_SQ_MUL_START,
        N_SQ_MUL_WAIT,
        N_GAIN_PREP,
        N_DIV_START,
        N_DIV_WAIT,
        N_SCALE_MUL_START,
        N_SCALE_MUL_WAIT,
        N_SCALE_COMMIT,
        N_DONE
    } norm_state_t;

    norm_state_t state;

    logic signed [31:0] vec0, vec1, vec2, vec3;
    logic signed [31:0] next0, next1, next2, next3;
    logic [63:0]        sum_sq;
    logic [31:0]        s_q30;
    logic [33:0]        num_q30;
    logic [34:0]        den_q30;
    logic [31:0]        gain_q30;
    logic [2:0]         element_index;
    logic [2:0]         iteration_index;

    logic               mul_start;
    logic               mul_busy;
    logic               mul_done;
    logic signed [31:0] mul_a, mul_b;
    logic signed [63:0] mul_product;

    logic               div_start;
    logic               div_busy;
    logic               div_done;
    logic               div_by_zero;
    logic [63:0]        div_dividend;
    logic [34:0]        div_divisor;
    logic [63:0]        div_quotient;

    logic [35:0] abs0, abs1, abs2, abs3, max_abs;
    logic [35:0] scaled0_36, scaled1_36, scaled2_36, scaled3_36;
    integer max_bit;
    integer shift_amount;
    integer i;

    function automatic logic [35:0] abs36(input logic signed [35:0] value);
        begin
            abs36 = value[35] ? (~value + 36'd1) : value;
        end
    endfunction

    function automatic logic signed [31:0] select_vec(input logic [2:0] index);
        begin
            case (index)
                3'd0: select_vec = vec0;
                3'd1: select_vec = vec1;
                3'd2: select_vec = vec2;
                default: select_vec = vec3;
            endcase
        end
    endfunction

    always_comb begin
        abs0 = abs36(in0);
        abs1 = abs36(in1);
        abs2 = abs36(in2);
        abs3 = dim3 ? 36'd0 : abs36(in3);
        max_abs = abs0;
        if (abs1 > max_abs) max_abs = abs1;
        if (abs2 > max_abs) max_abs = abs2;
        if (abs3 > max_abs) max_abs = abs3;

        max_bit = 0;
        for (i = 0; i < 36; i = i + 1)
            if (max_abs[i]) max_bit = i;

        scaled0_36 = in0;
        scaled1_36 = in1;
        scaled2_36 = in2;
        scaled3_36 = dim3 ? 36'sd0 : in3;
        if (max_abs != 0) begin
            if (max_bit > 29) begin
                shift_amount = max_bit - 29;
                scaled0_36 = in0 >>> shift_amount;
                scaled1_36 = in1 >>> shift_amount;
                scaled2_36 = in2 >>> shift_amount;
                scaled3_36 = dim3 ? 36'sd0 : (in3 >>> shift_amount);
            end else if (max_bit < 29) begin
                shift_amount = 29 - max_bit;
                scaled0_36 = in0 <<< shift_amount;
                scaled1_36 = in1 <<< shift_amount;
                scaled2_36 = in2 <<< shift_amount;
                scaled3_36 = dim3 ? 36'sd0 : (in3 <<< shift_amount);
            end
        end
    end

    mul_signed32_seq u_mul (
        .clk(clk), .rst_n(rst_n), .start(mul_start), .a(mul_a), .b(mul_b),
        .busy(mul_busy), .done(mul_done), .product(mul_product)
    );

    udiv64_by35_seq u_div (
        .clk(clk), .rst_n(rst_n), .start(div_start),
        .dividend(div_dividend), .divisor(div_divisor), .busy(div_busy),
        .done(div_done), .div_by_zero(div_by_zero), .quotient(div_quotient)
    );

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state           <= N_IDLE;
            busy            <= 1'b0;
            done            <= 1'b0;
            zero_input      <= 1'b0;
            out0            <= '0;
            out1            <= '0;
            out2            <= '0;
            out3            <= '0;
            vec0            <= '0;
            vec1            <= '0;
            vec2            <= '0;
            vec3            <= '0;
            next0           <= '0;
            next1           <= '0;
            next2           <= '0;
            next3           <= '0;
            sum_sq          <= '0;
            s_q30           <= '0;
            num_q30         <= '0;
            den_q30         <= '0;
            gain_q30        <= '0;
            element_index   <= '0;
            iteration_index <= '0;
            mul_start       <= 1'b0;
            mul_a           <= '0;
            mul_b           <= '0;
            div_start       <= 1'b0;
            div_dividend    <= '0;
            div_divisor     <= '0;
        end else begin
            done      <= 1'b0;
            mul_start <= 1'b0;
            div_start <= 1'b0;
            case (state)
                N_IDLE: begin
                    if (start) begin
                        busy       <= 1'b1;
                        zero_input <= (max_abs == 0);
                        if (max_abs == 0) begin
                            out0  <= '0;
                            out1  <= '0;
                            out2  <= '0;
                            out3  <= '0;
                            state <= N_DONE;
                        end else begin
                            vec0            <= scaled0_36[31:0];
                            vec1            <= scaled1_36[31:0];
                            vec2            <= scaled2_36[31:0];
                            vec3            <= scaled3_36[31:0];
                            iteration_index <= '0;
                            state           <= N_SQ_INIT;
                        end
                    end
                end

                N_SQ_INIT: begin
                    sum_sq        <= '0;
                    element_index <= '0;
                    state         <= N_SQ_MUL_START;
                end

                N_SQ_MUL_START: begin
                    if (!mul_busy) begin
                        mul_a     <= select_vec(element_index);
                        mul_b     <= select_vec(element_index);
                        mul_start <= 1'b1;
                        state     <= N_SQ_MUL_WAIT;
                    end
                end

                N_SQ_MUL_WAIT: begin
                    if (mul_done) begin
                        if (element_index == (dim3 ? 3'd2 : 3'd3)) begin
                            s_q30 <= rshift_u64_rne(sum_sq + $unsigned(mul_product), 30);
                            state <= N_GAIN_PREP;
                        end else begin
                            sum_sq        <= sum_sq + $unsigned(mul_product);
                            element_index <= element_index + 3'd1;
                            state         <= N_SQ_MUL_START;
                        end
                    end
                end

                N_GAIN_PREP: begin
                    if (s_q30 == 0) begin
                        zero_input <= 1'b1;
                        out0       <= '0;
                        out1       <= '0;
                        out2       <= '0;
                        out3       <= '0;
                        state      <= N_DONE;
                    end else begin
                        num_q30 <= (34'd5 << 30) + {2'b00, s_q30};
                        den_q30 <= (35'd2 << 30) + ({3'b000, s_q30} << 2);
                        state    <= N_DIV_START;
                    end
                end

                N_DIV_START: begin
                    if (!div_busy) begin
                        div_dividend <= {num_q30, 30'b0};
                        div_divisor  <= den_q30;
                        div_start    <= 1'b1;
                        state        <= N_DIV_WAIT;
                    end
                end

                N_DIV_WAIT: begin
                    if (div_done) begin
                        gain_q30 <= (|div_quotient[63:32]) ? 32'hffff_ffff : div_quotient[31:0];
                        state    <= N_SCALE_MUL_START;
                        element_index <= '0;
                    end
                end

                N_SCALE_MUL_START: begin
                    if (!mul_busy) begin
                        mul_a     <= select_vec(element_index);
                        mul_b     <= $signed(gain_q30);
                        mul_start <= 1'b1;
                        state     <= N_SCALE_MUL_WAIT;
                    end
                end

                N_SCALE_MUL_WAIT: begin
                    if (mul_done) begin
                        case (element_index)
                            3'd0: next0 <= sat_rshift_s64_rne(mul_product, 30);
                            3'd1: next1 <= sat_rshift_s64_rne(mul_product, 30);
                            3'd2: next2 <= sat_rshift_s64_rne(mul_product, 30);
                            default: next3 <= sat_rshift_s64_rne(mul_product, 30);
                        endcase
                        if (element_index == (dim3 ? 3'd2 : 3'd3))
                            state <= N_SCALE_COMMIT;
                        else begin
                            element_index <= element_index + 3'd1;
                            state         <= N_SCALE_MUL_START;
                        end
                    end
                end

                N_SCALE_COMMIT: begin
                    if (iteration_index == ITERATIONS-1) begin
                        out0  <= next0;
                        out1  <= next1;
                        out2  <= next2;
                        out3  <= dim3 ? 32'sd0 : next3;
                        state <= N_DONE;
                    end else begin
                        vec0            <= next0;
                        vec1            <= next1;
                        vec2            <= next2;
                        vec3            <= dim3 ? 32'sd0 : next3;
                        iteration_index <= iteration_index + 3'd1;
                        state           <= N_SQ_INIT;
                    end
                end

                N_DONE: begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= N_IDLE;
                end

                default: state <= N_IDLE;
            endcase
        end
    end
endmodule
