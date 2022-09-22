module add_v(rst,data_in,data_out,clk);

input data_in,rst,clk;
output [1:0]data_out;
reg [1:0]data_out;            //用00、01、11分别表示输入信号为“0、1、V”
reg [1:0]counter;               //设置连0计数器

always@(posedge clk or negedge rst) begin
    if(!rst) begin
        counter <= 0;
        data_out <= 0;
    end
    else if(data_in == 1'b1) begin  //判断输入信号是否为1
        counter <= 0;                //若为1则计数器复位，输出“01”
        data_out <= 2'b01;
    end
    else if(data_in == 1'b0) begin                        //若输入信号为0，计数器+1
        counter <= counter + 1; 
        if(counter == 2'b11) begin    //判断连0个数是否达到4个，因为非阻塞赋值，此时计数器值应为3时，表示出现4个连0
            data_out <= 2'b11;        //将0的输出变为V
            counter <= 0;            //计数器复位
        end
        else data_out <= 2'b00;       //若连0数不为4，输出“00”
    end
end

endmodule
