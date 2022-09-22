module fp(clk,rst,clk_even,clk_odd);
    input clk;
    input rst;
    output clk_odd;
    output clk_even;
    division_6 fp1(clk,rst,clk_odd);
    division_7 fp2(clk,rst,clk_even);
endmodule
