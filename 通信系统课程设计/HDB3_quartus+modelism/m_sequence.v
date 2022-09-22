module m_sequence(clk,rst,ena,data_out,load);

input clk;                        //时钟信号
input rst;                        //复位信号，低电平有效
input ena;                        //控制信号，高电平时序列发生器开始工作
output data_out;
output load;                      //控制信号，为高电平时表示伪随机序列开始
reg data_out,load;
reg [6:0]temp;                    //7级移位寄存器产生周期为127的m序列

always @(posedge clk or negedge rst) begin
    if(!rst) begin
            data_out <= 0;
            load <= 1'b0;            //控制信号设为无效
            temp <= 7'b1111_111;     //移位寄存器初始状态设为全1
    end
    else begin                       //开始产生序列信号
        if(ena) begin                //判断序列发生器的控制信号是否有效
                //若控制信号有效
                load <= 1'b1;        //将控制伪随机序列产生的信号设为有效        
                temp <= {temp[5:0],temp[2]^temp[6]};        //对应f(x)=x^7+x^3+1
                //当控制伪随机数列产生的信号使能时将移位寄存器的最高位做为m序列进行输出
            if(load)    data_out <= temp[6];
        end
        else load <= 1'b0;    //若控制信号无效，则不开始产生伪随机序列
    end
end

endmodule