//凌智城 201806061211 通信工程1803
`timescale 1ns / 1ps
module uartrx(clk, rst,rxd, rxd_data, rxd_en, dataerror, frameerror);
input clk;             //采样时钟
input rst;             //复位信号
input rxd;             //UART数据输入

output rxd_data;        //接收数据输出
output rxd_en;          //接收数据有效，高说明接收到一个字节
output dataerror;      //数据出错指示
output frameerror;     //帧出错指示

reg[7:0] rxd_data;
reg rxd_en, dataerror;
reg frameerror;
reg [7:0] cnt;
reg rxbuf, rxfall, receive;
parameter paritymode = 1'b1;
reg presult, idle;		//presult奇偶校验位;idle线路状态指示，高1为线路忙，低0为线路空闲

//当线路产生下降沿时，即认为线路有数据传输
always @(posedge clk)   
begin
  rxbuf <= rxd;
  rxfall <= rxbuf & (~rxd);
end

//检测到线路的下降沿并且原先线路为空闲，启动接收数据进程
always @(posedge clk)
begin
  if (rxfall && (~idle))  
	begin
		receive <= 1'b1;      //开始接收数据
	end
  else if(cnt == 8'd168)  //接收数据完成
	begin
		receive <= 1'b0;
	end
end

//使用176个时钟接收一个数据（起始位、8位数据、奇偶校验位、停止位），每位占用16个时钟//
always @(posedge clk or negedge rst)
begin
  if (!rst) 
	begin
		idle<=1'b0;
		cnt<=8'd0;
		rxd_en <= 1'b0;
		frameerror <= 1'b0;
		dataerror <= 1'b0;
		presult<=1'b0;
	end
  else if(receive == 1'b1)
	begin
	case (cnt)
	8'd0:                   //0~15个时钟为接收第一个比特，起始位
		begin
			idle <= 1'b1;
			cnt <= cnt + 8'd1;
			rxd_en <= 1'b0;
		end
	8'd24:                  //16~31个时钟为第1个bit数据，取中间第24个时钟的采样值
		begin
			idle <= 1'b1;
			rxd_data[0] <= rxd;
			presult <= paritymode^rxd;
			cnt <= cnt + 8'd1;
			rxd_en <= 1'b0;
		end
	8'd40:                 //47~32个时钟为第2个bit数据，取中间第40个时钟的采样值 
		begin
			idle <= 1'b1;
			rxd_data[1] <= rxd;
			presult <= presult^rxd;
			cnt <= cnt + 8'd1;
			rxd_en <= 1'b0;
		end
	8'd56:                 //63~48个时钟为第3个bit数据，取中间第56个时钟的采样值   
		begin
			idle <= 1'b1;
			rxd_data[2] <= rxd;
			presult <= presult^rxd;
			cnt <= cnt + 8'd1;
			rxd_en <= 1'b0;
		end
	8'd72:                //79~64个时钟为第4个bit数据，取中间第72个时钟的采样值   
		begin
			idle <= 1'b1;
			rxd_data[3] <= rxd;
			presult <= presult^rxd;
			cnt <= cnt + 8'd1;
			rxd_en <= 1'b0;
		end
	8'd88:               //95~80个时钟为第5个bit数据，取中间第88个时钟的采样值    
		begin
			idle <= 1'b1;
			rxd_data[4] <= rxd;
			presult <= presult^rxd;
			cnt <= cnt + 8'd1;
			rxd_en <= 1'b0;
		end
	8'd104:             //111~96个时钟为第6个bit数据，取中间第104个时钟的采样值    
		begin
			idle <= 1'b1;
			rxd_data[5] <= rxd;
			presult <= presult^rxd;
			cnt <= cnt + 8'd1;
			rxd_en <= 1'b0;
		end
	8'd120:             //127~112个时钟为第7个bit数据，取中间第120个时钟的采样值     
		begin
			idle <= 1'b1;
			rxd_data[6] <= rxd;
			presult <= presult^rxd;
			cnt <= cnt + 8'd1;
			rxd_en <= 1'b0;
		end
	8'd136:            //143~128个时钟为第8个bit数据，取中间第136个时钟的采样值   
		begin
			idle <= 1'b1;
			rxd_data[7] <= rxd;
			presult <= presult^rxd;
			cnt <= cnt + 8'd1;
			rxd_en <= 1'b1;      //接收数据有效
		end
	8'd152:            //159~144个时钟为接收奇偶校验位，取中间第152个时钟的采样值     
		begin
			idle <= 1'b1;
				if(presult == rxd)
					dataerror <= 1'b0;
				else
					dataerror <= 1'b1;       //如果奇偶校验位不对，表示数据出错
			cnt <= cnt + 8'd1;
			rxd_en <= 1'b1;             
		end
	8'd168:            //160~175个时钟为接收停止位，取中间第168个时钟的采样值  
		begin
		idle <= 1'b1;
		if(1'b1 == rxd)
			frameerror <= 1'b0;
		else
			frameerror <= 1'b1;      //如果没有接收到停止位，表示帧出错
		cnt <= cnt + 8'd1;
		rxd_en <= 1'b1;
		end
	default:
		begin
			cnt <= cnt + 8'd1;
		end
	endcase
	end
  else
	begin
		cnt <= 8'd0;
		idle <= 1'b0;
		rxd_en <= 1'b0;
	end
 end
endmodule