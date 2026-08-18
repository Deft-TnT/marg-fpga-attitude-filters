`timescale 1ns/1ps
// Same 11-word input / five-word output stream envelope used by all current
// MARG comparison cores.  Word 10 remains reserved for future configuration.
module mekf_diag_top (
    input logic clk, rst_n,
    input logic [31:0] s_axis_tdata,
    input logic s_axis_tvalid,
    output logic s_axis_tready,
    input logic s_axis_tlast,
    output logic [31:0] m_axis_tdata,
    output logic m_axis_tvalid,
    input logic m_axis_tready,
    output logic m_axis_tlast,
    output logic busy
);
    logic signed [31:0] ax_buf,ay_buf,az_buf,mx_buf,my_buf,mz_buf,wx_buf,wy_buf,wz_buf;
    logic [31:0] dt_buf,config_buf;
    logic [3:0] input_index;
    logic frame_pending,core_in_ready,core_out_valid,core_out_ready,core_busy,output_active;
    logic signed [31:0] core_qw,core_qx,core_qy,core_qz,qw_hold,qx_hold,qy_hold,qz_hold;
    logic [7:0] core_status,status_hold;
    logic [2:0] output_index;

    assign s_axis_tready=!frame_pending;
    assign core_out_ready=!output_active;
    assign m_axis_tvalid=output_active;
    assign m_axis_tlast=output_active&&(output_index==3'd4);
    assign busy=core_busy||frame_pending||output_active||(input_index!=0);
    always_comb case(output_index)
        0:m_axis_tdata=qw_hold;1:m_axis_tdata=qx_hold;2:m_axis_tdata=qy_hold;3:m_axis_tdata=qz_hold;default:m_axis_tdata={24'd0,status_hold};
    endcase

    mekf_diag_core u_core(
        .clk(clk),.rst_n(rst_n),.in_valid(frame_pending),.in_ready(core_in_ready),
        .ax_in(ax_buf),.ay_in(ay_buf),.az_in(az_buf),.mx_in(mx_buf),.my_in(my_buf),.mz_in(mz_buf),.wx_in(wx_buf),.wy_in(wy_buf),.wz_in(wz_buf),
        .dt_in(dt_buf),.config_word_in(config_buf),.out_valid(core_out_valid),.out_ready(core_out_ready),.qw_out(core_qw),.qx_out(core_qx),.qy_out(core_qy),.qz_out(core_qz),.status_out(core_status),.busy(core_busy)
    );
    always_ff @(posedge clk) begin
        if(!rst_n) begin
            ax_buf<='0;ay_buf<='0;az_buf<='0;mx_buf<='0;my_buf<='0;mz_buf<='0;wx_buf<='0;wy_buf<='0;wz_buf<='0;dt_buf<='0;config_buf<='0;input_index<='0;frame_pending<=0;output_active<=0;output_index<='0;qw_hold<='0;qx_hold<='0;qy_hold<='0;qz_hold<='0;status_hold<='0;
        end else begin
            if(frame_pending&&core_in_ready)frame_pending<=0;
            if(s_axis_tvalid&&s_axis_tready)begin
                case(input_index)
                    0:ax_buf<=$signed(s_axis_tdata);1:ay_buf<=$signed(s_axis_tdata);2:az_buf<=$signed(s_axis_tdata);3:mx_buf<=$signed(s_axis_tdata);4:my_buf<=$signed(s_axis_tdata);5:mz_buf<=$signed(s_axis_tdata);6:wx_buf<=$signed(s_axis_tdata);7:wy_buf<=$signed(s_axis_tdata);8:wz_buf<=$signed(s_axis_tdata);9:dt_buf<=s_axis_tdata;default:config_buf<=s_axis_tdata;
                endcase
                if(s_axis_tlast)begin if(input_index==10)frame_pending<=1;input_index<=0;end
                else if(input_index==10)input_index<=0;else input_index<=input_index+1;
            end
            if(core_out_valid&&core_out_ready)begin qw_hold<=core_qw;qx_hold<=core_qx;qy_hold<=core_qy;qz_hold<=core_qz;status_hold<=core_status;output_index<=0;output_active<=1;end
            else if(output_active&&m_axis_tready)begin if(output_index==4)begin output_active<=0;output_index<=0;end else output_index<=output_index+1;end
        end
    end
endmodule
