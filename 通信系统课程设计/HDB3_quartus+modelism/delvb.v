module delvb(rst_n, indata, outdata,clk);

input rst_n,clk;
input [1:0]indata;
output outdata;
reg [3:0]buffer;
reg bufferdata;
reg counterv;

always@(posedge clk or negedge rst_n)
begin// 设置四位buffer暂存前四位数据
if (!rst_n) begin
    buffer[3]<=0;
    buffer[2]<=0;
    buffer[1]<=0;
    buffer[0]<=0;
end
else begin
    buffer[3]<=buffer[2];
    buffer[2]<=buffer[1];
    buffer[1]<=buffer[0];
    buffer[0]<=bufferdata;
end
end


always @(posedge clk or negedge rst_n) begin
    // 复位初始化
    if(!rst_n) bufferdata <= 0;
    else begin
        if(indata==2'b01) begin
        // 如果是正常的01，则数据还是1
            counterv <= 0;
            bufferdata<=1;
        end
        else if(indata==2'b00) begin
        // 如果是正常的00，则数据还是0
            counterv <= 0;
            bufferdata<=0;
        end
        else if(indata==2'b11) begin
        // 如果是的11，则表示该位为V码，将counterv标记1
            counterv <= 1;
            bufferdata<=0;
        end
    end
end

assign outdata = (counterv == 1)  ? 0 : buffer[2];

endmodule