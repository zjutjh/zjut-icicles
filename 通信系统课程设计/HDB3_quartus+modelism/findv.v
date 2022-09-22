module findv(rst_n, indata_P, indata_N, outdata,clk);

input rst_n,clk;
input indata_P, indata_N;
output reg [1:0] outdata;
reg [1:0]flag1;
reg [1:0]flag2;

initial begin
    flag1=2'b00;
    flag2=2'b00;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        // 复位初始化
        flag1=2'b00;
        flag2=2'b00;
        outdata <= 2'b00;
    end
    else begin
        if(indata_P==1'b1 && indata_N==1'b0) begin
        // 如果PN是10，则flag2置0，flag1自加1，如果flag1自加+后为2则表示有两个连续的+1
            flag2<=2'b00;
            flag1=flag1+2'b01;
            if(flag1==2'b10) begin
                outdata<=2'b11;
                flag1<=2'b00;
            end
            else outdata<=2'b01;
        end
        else if(indata_P==1'b0 && indata_N==1'b1) begin
        // 如果PN是01，则flag1置0，flag2自加1，如果flag2自加+后为2则表示有两个连续的-1
            flag1<=2'b00;
            flag2=flag2+2'b01;
            if(flag2==2'b10) begin
                outdata<=2'b11;
                flag2<=2'b00;
            end
            else outdata<=2'b01;
        end
        // 其他情况为00则正常置00
        else if(indata_P==1'b0 && indata_N==1'b0) outdata<=2'b00;
        //else outdata<=2'b01;
    end
end

endmodule