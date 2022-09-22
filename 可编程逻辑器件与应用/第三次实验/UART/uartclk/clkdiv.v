//整个UART的同步时钟频率为16*f1，用系统时钟获取时，f1=9600时，设计分频系数N=f0/(16*f1)=325.52，取整为326
//凌智城 201806061211 通信工程1803
module clkdiv(clk, rst,clkout);
input clk;          //系统时钟
input rst;          //复位信号
output clkout;      //采样时钟输出
reg clkout;
reg [15:0] cnt;
parameter N=326;

always @(posedge clk)   //分频进程
	if(!rst)
		begin
			cnt<=1'b0;
			clkout<=1'b0;
		end
	else if(N%2==0)
		begin
			if(cnt<N/2-1)
				begin
					cnt<=cnt+1'b1;
				end
			else
				begin
					cnt<=1'b0;
					clkout<=~ clkout;
				end
		end

endmodule
