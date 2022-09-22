module trans8to1(rst_n, indata_8, outdata_P, outdata_N,clk);

input rst_n,clk;
input [7:0] indata_8;
output outdata_P, outdata_N;
reg outdata_P, outdata_N;

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        // 复位初始化
        outdata_P <= 0;
        outdata_N <= 0;
    end
    else begin
    // 三个区间分别对应三种双极性P和N的值来表示+1、0和-1
    if(indata_8 >= 8'h58) begin
            outdata_P<=1;
            outdata_N<=0;
        end
        else if(indata_8 <= 8'h3f && indata_8 >= 8'h30) begin
            outdata_P<=0;
            outdata_N<=0;
        end
        else if(indata_8 <= 8'h0f) begin
            outdata_P<=0;
            outdata_N<=1;
        end
    end
end

endmodule