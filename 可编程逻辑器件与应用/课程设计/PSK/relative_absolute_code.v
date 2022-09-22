//通信工程1803 凌智城201806061211
module relative_absolute_code(clk,start,x,y);
	input clk;		//系统时钟信号
	input start;	//开始调制信号
	input x;
	output y;
	
	reg y;
	reg[1:0] q;		//计数器
	reg w;
	always@(posedge clk)//分频和计数
		begin
			if(!start)
				begin 
					q<=0;
				end
			else if(q==0)
				begin
					q<=1;
				end
			else if(q==3)
				begin
					q<=0;
					y<=w^x;
					w<=x;
				end
			else
				begin
					q<=q+1'b1;
				end
		end
	
endmodule