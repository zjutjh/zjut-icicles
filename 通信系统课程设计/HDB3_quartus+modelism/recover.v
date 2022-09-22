module recover(clk_in, rst_n, indata_8, clk_out);

input clk_in, rst_n;
input [7:0]indata_8;
output reg clk_out;
reg [3:0]counter;
reg [7:0]buffer;
reg flag;

always @(posedge clk_in or negedge rst_n) begin
    // 如果复位则buffer出清零，否则就将输入的八位信号给buffer处理
    if(!rst_n) buffer<=8'b0;
    else buffer<=indata_8;
end


always @(posedge clk_in or negedge rst_n) begin
    if(!rst_n) begin
    // 复位初始化
        clk_out<=0;
        flag <=0;
        counter <=1;
    end
    else begin
        /*if ((buffer[7] == 1'b0&&buffer[6] == 1'b1&&(indata_8 ==1'b1 || indata_8 ==1'b0)) || 
            (buffer[7] == 1'b1 &&indata_8[7]!=1'b1)||
            (buffer[7] == 1'b0&&buffer[6]==1'b0&&(indata_8[7]==1'b1||indata_8[6]==1'b1))) flag<= 1;    */
            // 根据实际AD输出调整，由于AD输出不是刚好的8'h80,8'hff和8'h00三种情况，而是三个不确定的区间
            // 用这三个实际的变化区间来进行码元的边缘检测
        if ((buffer >= 8'h58 && indata_8 <=8'h3f) || (buffer <=8'h3f && buffer >= 8'h30 && (indata_8 >= 8'h58 || indata_8 <= 8'h0f)) || 
                (buffer <= 8'h0f && indata_8 >= 8'h30)) flag<= 1;
        if(flag == 1) begin
            if (counter==4'h8) begin
            // 如果计数到8则翻转时钟
                clk_out=~clk_out;
                counter<=1;
            end    
            else begin
            // 计数没到8则继续
                counter <= counter+1;
            end
        end
    end
end

endmodule