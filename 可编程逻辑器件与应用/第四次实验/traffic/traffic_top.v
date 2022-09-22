//凌智城 201806061211 通信工程1803
module traffic_top(clk,rst_n,prim_red_time,prim_green_time,prim_yellow_time,prim_wait_time,seco_wait_time,prim_ryg_light,seco_ryg_light,emergency,test);

input clk,rst_n;					//1Hz时钟信号,低电平复位信号
input[7:0]prim_red_time,prim_green_time,prim_yellow_time;	//主干道红绿黄时间（秒）
input emergency,test;										//紧急状态控制信号，信号灯测试控制信号
output[7:0]prim_wait_time,seco_wait_time;					//主干道次干道倒计时时间
output[2:0]prim_ryg_light,seco_ryg_light;					//主干道次干道红黄绿信号灯

wire[7:0]seco_red_time,seco_green_time,seco_yellow_time;

assign seco_red_time=prim_green_time+prim_yellow_time;//(11=9+2)
assign seco_green_time=prim_red_time-prim_yellow_time;//(4=6-2)
assign seco_yellow_time=prim_yellow_time;//(2=2)

traffic_con primary_light(clk,rst_n,1'b1,prim_red_time,prim_green_time,prim_yellow_time,prim_wait_time,prim_ryg_light,emergency,test);
traffic_con secondary_light(clk,rst_n,1'b0,seco_red_time,seco_green_time,seco_yellow_time,seco_wait_time,seco_ryg_light,emergency,test);

endmodule