module change(rst,data_inP,data_inN,data_out,clk);

//二输入八输出模块，将PN信号转换为八位输出信号作为DAC的输入
input rst,clk,data_inP,data_inN;
output [7:0]data_out;
reg [7:0]data_out;

always@(posedge clk or negedge rst)    begin
    if(!rst)
        data_out <= 8'b10000000;
    else if(data_inP == 1 && data_inN == 0) begin        //PN=10对应输出HFF
        data_out <= 8'b11111111;
    end
    else if(data_inP == 0 && data_inN ==1) begin        //PN=01对应输出H00
        data_out <= 8'b00000000;
    end
    else if(data_inP == 0 && data_inN ==0) begin        //PN=00对应输出H80
        data_out <= 8'b10000000;
    end
end

endmodule
