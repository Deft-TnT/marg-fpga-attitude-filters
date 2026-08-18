`timescale 1ns/1ps
// Interface-identical wrapper for the frozen dynamic-beta-v1 core.
module dynamic_beta_stream_top(input logic clk,rst_n,input logic[31:0]s_axis_tdata,input logic s_axis_tvalid,output logic s_axis_tready,input logic s_axis_tlast,output logic[31:0]m_axis_tdata,output logic m_axis_tvalid,input logic m_axis_tready,output logic m_axis_tlast,output logic busy);
logic signed[31:0] b[0:8],qw,qx,qy,qz,qwh,qxh,qyh,qzh;logic[31:0]dt,beta;logic[3:0]wi;logic[2:0]wo;logic pending,cin_ready,cout_valid,cout_ready,cbusy,active;logic[7:0]status,sh;
assign s_axis_tready=!pending;assign cout_ready=!active;assign m_axis_tvalid=active;assign m_axis_tlast=active&&(wo==4);assign busy=cbusy||pending||active||(wi!=0);
always_comb case(wo)0:m_axis_tdata=qwh;1:m_axis_tdata=qxh;2:m_axis_tdata=qyh;3:m_axis_tdata=qzh;default:m_axis_tdata={24'd0,sh};endcase
marg_estimator_dynamic_beta_core u(.clk(clk),.rst_n(rst_n),.in_valid(pending),.in_ready(cin_ready),.ax_in(b[0]),.ay_in(b[1]),.az_in(b[2]),.mx_in(b[3]),.my_in(b[4]),.mz_in(b[5]),.wx_in(b[6]),.wy_in(b[7]),.wz_in(b[8]),.dt_in(dt),.beta_in(beta),.out_valid(cout_valid),.out_ready(cout_ready),.qw_out(qw),.qx_out(qx),.qy_out(qy),.qz_out(qz),.status_out(status),.busy(cbusy));
always_ff@(posedge clk)if(!rst_n)begin for(integer k=0;k<9;k++)b[k]<=0;dt<=0;beta<=0;wi<=0;wo<=0;pending<=0;active<=0;qwh<=0;qxh<=0;qyh<=0;qzh<=0;sh<=0;end else begin
if(pending&&cin_ready)pending<=0;if(s_axis_tvalid&&s_axis_tready)begin if(wi<9)b[wi]<=$signed(s_axis_tdata);else if(wi==9)dt<=s_axis_tdata;else beta<=s_axis_tdata;if(s_axis_tlast)begin if(wi==10)pending<=1;wi<=0;end else if(wi==10)wi<=0;else wi<=wi+1;end
if(cout_valid&&cout_ready)begin qwh<=qw;qxh<=qx;qyh<=qy;qzh<=qz;sh<=status;wo<=0;active<=1;end else if(active&&m_axis_tready)begin if(wo==4)begin active<=0;wo<=0;end else wo<=wo+1;end end
endmodule
