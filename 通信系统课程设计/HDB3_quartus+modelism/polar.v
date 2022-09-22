module polar(rst,data_in,data_outP,data_outN,clk);

input [1:0]data_in;
input rst,clk;
output data_outP,data_outN;    //用P与N表示正负
reg [1:0] polar_out;                 //“00”表示0，“10”表示+1，“01”表示-1
reg data_outN,data_outP;
reg even;                             //设置极性判断标志,1表示+1和-V，0表示—1和+V

always@(posedge clk or negedge rst) begin
    if(!rst) even <= 1;
    else if(data_in == 2'b11) begin    //若输入为“11（V）”
        if(even ==     1)                          //判断极性标志，若even为1
            polar_out <= 2'b01;             //输出为“01（-1）”
        else                                     //若even为0
            polar_out <= 2'b10;             //输出为“10（+1）”
    end
    //若输入为“01（1）”或者“10（B）”
    else if(data_in == 2'b01 || data_in == 2'b10) begin
        if(even == 1) begin              //判断极性标志，若even为1
            even <= 0;                     //将even翻转
            polar_out <= 2'b10;         //输出为“10（+1）”
        end
        else begin                          //若even为0
            even <= 1;                      //将even翻转
            polar_out <= 2'b01;         //输出为“01（-1）”
        end
    end
    else if(data_in == 2'b00)        //若输入为“00（0）”
        polar_out <= 2'b00;            //输出为“00（0）”
end

always@(polar_out or rst) begin    
//将输出寄存器的两位数分别赋值给输出端口data_outP和data_outN
    if(!rst) begin
        data_outP <= 0;
        data_outN <= 0;
    end
    else if(polar_out == 2'b01) begin
        data_outP <= 0;                //PN=01表示-1
        data_outN <= 1;
    end
    else if(polar_out == 2'b10) begin
        data_outP <= 1;                //PN=10表示+1
        data_outN <= 0;
    end
    else begin
        data_outP <= 0;                //PN=00表示0
        data_outN <= 0;            
    end
end
endmodule
