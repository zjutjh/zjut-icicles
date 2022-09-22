//通信工程1803 凌智城201806061211
module absolute_relative_code(clk,start,x,y);
	input clk;		//系统时钟信号
	input start;	//开始调制信号
	input x;		//绝对码输入信号
	output y;		//相对码输出信号
	
	reg y;
	reg w;			//寄存器
	reg[1:0] q;		//计数器
	always@(posedge clk)//分频和计数
		begin
			if(!start)
				begin 
					q<=0;
					w<=0;
				end
			else if(q==0)
				begin
					q<=1;
					w<=w^x;
					y<=w^x;
				end
			else if(q==3)
				begin
					q<=0;
				end
			else
				begin
					q<=q+1'b1;
				end
		end
	
endmodule