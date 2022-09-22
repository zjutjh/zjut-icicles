module even16_div(clk,rst,clk_out);
    
input clk,rst;
output clk_out;
reg clk_out;
reg[3:0] count;
// 偶数分频
parameter N=16;

always @(posedge clk or negedge rst)
    if(!rst) begin
        // 复位初始化
        count<=1'b0;
        clk_out<=1'b0;
    end
    else if(N%2==0)begin
        if(count<N/2-1) count <=count+1'b1;
        else begin
            // 计数到一半，时钟翻转实现分频
            count<=1'b0;
            clk_out<=~clk_out;
        end
    end

endmodule
