//凌智城 201806061211 通信工程1803
module uart_top(
	clk,
	rst,
	rxd,
	clkout,
	en,
	dataerror,
	framerror,
	idle,
	txd,
	data
);


input wire	clk;
input wire	rst;
input wire	rxd;
output wire	clkout;
output wire	en;
output wire	dataerror;
output wire	framerror;
output wire	idle;
output wire	txd;
output wire	[7:0] data;

wire	SYNTHESIZED_WIRE_4;
wire	SYNTHESIZED_WIRE_2;
wire	[7:0] SYNTHESIZED_WIRE_3;

assign	clkout = SYNTHESIZED_WIRE_4;
assign	en = SYNTHESIZED_WIRE_2;
assign	data = SYNTHESIZED_WIRE_3;




clkdiv	b2v_inst(
	.clk(clk),
	.rst(rst),
	.clkout(SYNTHESIZED_WIRE_4));
	defparam	b2v_inst.N = 326;


uartrx	b2v_inst1(
	.clk(SYNTHESIZED_WIRE_4),
	.rst(rst),
	.rxd(rxd),
	.rxd_en(SYNTHESIZED_WIRE_2),
	.dataerror(dataerror),
	.frameerror(framerror),
	.rxd_data(SYNTHESIZED_WIRE_3));
	defparam	b2v_inst1.paritymode = 1'b1;


uarttx	b2v_inst2(
	.clk(SYNTHESIZED_WIRE_4),
	.rst(rst),
	.txd_en(SYNTHESIZED_WIRE_2),
	.txd_data(SYNTHESIZED_WIRE_3),
	.idle(idle),
	.txd(txd));
	defparam	b2v_inst2.paritymode = 1'b1;


endmodule
