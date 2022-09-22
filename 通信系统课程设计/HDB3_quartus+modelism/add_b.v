module add_b(rst,data_in,data_out,clk);

input clk,rst;
input [1:0]data_in;
output [1:0]data_out;        //用00、01、10、11分别表示输入信号为“0、1、B、V”
reg counter1;                    //设置“01”的计数器
reg counterv;                    //设置“11”的计数器
reg [1:0]buffer[3:0];        //取代节选择

always@(posedge clk or negedge rst) begin
//设置四位移位寄存器方便插“B”的实现
    if(!rst) begin
        buffer[3]<=0;
        buffer[2]<=0;
        buffer[1]<=0;
        buffer[0]<=0;
    end
    else begin
        buffer[3]<=buffer[2];
        buffer[2]<=buffer[1];
        buffer[1]<=buffer[0];
        buffer[0]<=data_in;
    end
end

always@(posedge clk or negedge rst) begin
//对输入进行判断
    if(!rst) begin
        counter1 = 0;
        counterv = 0;
    end
else if(data_in == 2'b11) //如果输入为“11”，则counterv加1
        counterv <= counterv + 1'b1;
    else if(data_in == 2'b01) begin //如果输入为“01”，则counter1加1
        counter1 <= counter1 + 1'b1 + counterv;
        counterv <= 1'b0;
    end
    else begin
        counter1 <= counter1 + counterv;
        counterv <= 1'b0;
    end
end

//若输入为“11”,如果此前1的个数为奇数，则输出不变，若为偶数，则输出“10”
//若输入不为“11”,则输出不变
assign data_out = (counter1%2 == 0) && (counterv == 1)? 2'b10 : buffer[3];

endmodule
