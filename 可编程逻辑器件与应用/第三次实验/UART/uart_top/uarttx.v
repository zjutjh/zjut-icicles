//凌智城 201806061211 通信工程1803
`timescale 1ns / 1ps
module uarttx(clk, rst,txd_data, txd_en, idle, txd);
input clk;                //UART时钟
input [7:0] txd_data;     //需要发送的数据
input txd_en;             //发送命令，上升沿有效
input rst;                //系统复位

output idle;              //线路状态指示，高1为线路忙，低0为线路空闲
output txd;               //发送数据信号

reg idle, txd;
reg send;
reg txd_en_buf, txd_en_rise;
reg presult;			//奇偶校验位

reg[7:0] cnt;             //计数器
parameter paritymode = 1'b1;//奇偶校验模式'高电平为奇校验，对每一位数据位进行异或，8位数据若有奇数个1，则presult为1

//检测发送命令是否有效，判断txd_en的上升沿
always @(posedge clk)
begin
   txd_en_buf <= txd_en;
   txd_en_rise <= (~txd_en_buf) & txd_en;
end

//当发送命令有效且线路为空闲时，启动新的数据发送进程
always @(posedge clk)
begin
  if (txd_en_rise &&  (~idle)) 
	begin
		send <= 1'b1;         //send=1允许发送新的数据
	end
  else if(cnt == 8'd168)      //计数器的值为168时一帧资料发送结束
	begin
	send <= 1'b0;             //新的数据发送完毕
	end
end


//使用168个时钟发送一个数据（起始位、8位数据、奇偶校验位、停止位），每位占用16个时钟
//11*16-8=168 
always @(posedge clk or negedge rst)    //异步复位
begin
  if (!rst) 					//如果有复位信号输入
	begin						//数据清零、计数器清零、预置结果清零、线路转为空闲状态
		txd <= 1'b0;
		cnt <= 8'd0;
		presult <= 1'b0;
		idle <= 1'b0;
	end		
  else if(send == 1'b1)			//如果允许发送
	begin
	case(cnt)               	//txd变低电平产生起始位，0~15个时钟为发送起始位
		8'd0: 
			begin
				txd <= 1'b0;			//串行发送
				idle <= 1'b1;			//占用忙状态
				cnt <= cnt + 8'd1;		//计数器+1
			end
		8'd16: 
			begin
				txd <= txd_data[0];    				//发送数据位的低位bit0,占用第16~31个时钟
				presult <= txd_data[0]^paritymode;	//从0开始对每一个数据位作异或操作
				idle <= 1'b1;						//占用忙状态
				cnt <= cnt + 8'd1;					//计数器+1
			end
		8'd32: 
			begin
				txd <= txd_data[1];    //发送数据位的第2位bit1,占用第47~32个时钟
				presult <= txd_data[1]^presult;
				idle <= 1'b1;
				cnt <= cnt + 8'd1;
			end
		8'd48: 
			begin
				txd <= txd_data[2];    //发送数据位的第3位bit2,占用第63~48个时钟
				presult <= txd_data[2]^presult;
				idle <= 1'b1;
				cnt <= cnt + 8'd1;
			end
		8'd64: 
			begin
				txd <= txd_data[3];    //发送数据位的第4位bit3,占用第79~64个时钟
				presult <= txd_data[3]^presult;
				idle <= 1'b1;
				cnt <= cnt + 8'd1;
			end
		8'd80: 
			begin 
				txd <= txd_data[4];   //发送数据位的第5位bit4,占用第95~80个时钟
				presult <= txd_data[4]^presult;
				idle <= 1'b1;
				cnt <= cnt + 8'd1;
			end
		8'd96: 
			begin
				txd <= txd_data[5];    //发送数据位的第6位bit5,占用第111~96个时钟
				presult <= txd_data[5]^presult;
				idle <= 1'b1;
				cnt <= cnt + 8'd1;
			end
		8'd112: 
			begin
				txd <= txd_data[6];    //发送数据位的第7位bit6,占用第127~112个时钟
				presult <= txd_data[6]^presult;
				idle <= 1'b1;
				cnt <= cnt + 8'd1;
			end
		8'd128: 
			begin 
				txd <= txd_data[7];    //发送数据位的第8位bit7,占用第143~128个时钟
				presult <= txd_data[7]^presult;
				idle <= 1'b1;
				cnt <= cnt + 8'd1;
			end
		8'd144: 
			begin
				txd <= presult;      //发送奇偶校验位，占用第159~144个时钟
				presult <= txd_data[0]^paritymode;
				idle <= 1'b1;
				cnt <= cnt + 8'd1;
			end
		8'd160: 
			begin
				txd <= 1'b1;         //发送停止位，占用第160~167个时钟            
				idle <= 1'b1;
				cnt <= cnt + 8'd1;
			end
		8'd168: 
			begin
				txd <= 1'b1;             
				idle <= 1'b0;       //一帧资料发送结束
				cnt <= cnt + 8'd1;
			end
		default: begin cnt <= cnt + 8'd1; end  //其他计数状态，未到数据位的中间，则计数器+1
	endcase
	end
  else  
	begin
		txd <= 1'b1;
		cnt <= 8'd0;
		idle <= 1'b0;
	end
end
endmodule