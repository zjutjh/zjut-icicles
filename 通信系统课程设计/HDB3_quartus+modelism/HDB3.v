module HDB3(rst,
                clk,clk_256,clk_16,clk_recover,clk_256_ad,clk_256_da,
                ena,
                data_out,data_out_ad,outm,outv,outb,outP,outN,data_out_check,
                finallyout,outdata_P,outdata_N,outdata_v
                );


input  rst,clk,ena;
input [7:0]data_out_ad;
output clk_256,clk_16,clk_recover,clk_256_ad,clk_256_da;
output [7:0]data_out_check;
output [7:0]data_out;
output [1:0]outv;
output [1:0]outb;
output outm,outP,outN;
output outdata_P,outdata_N;
output [1:0]outdata_v;
output finallyout;
//分频
even256_div div256(clk,rst,clk_256);
even16_div div16(clk_256,rst,clk_16);
//m序列
m_sequence m(clk_16,rst,ena,outm,load);
//编码
add_v u1(rst,outm,outv,clk_16);
add_b u2(rst,outv,outb,clk_16);
polar u3(rst,outb,outP,outN,clk_16);
change u4(rst,outP,outN,data_out,clk_16);
//码元定时恢复
recover u8(clk_256, rst, data_out_ad, clk_recover);
//译码
trans8to1 u5(rst, data_out_ad, outdata_P, outdata_N,clk_recover);
findv u6(rst, outdata_P, outdata_N, outdata_v,clk_recover);
delvb u7(rst, outdata_v, finallyout,clk_recover);

assign clk_256_ad = clk_256;
assign clk_256_da = clk_256;
assign data_out_check = data_out_ad;

endmodule
