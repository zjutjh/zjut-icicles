//通信工程1803 凌智城201806061211
module PSK_top(
	start,
	clk,
	absolute_in,
	absolute_out
);


input wire	start;
input wire	clk;
input wire	absolute_in;
output wire	absolute_out;

wire	SYNTHESIZED_WIRE_0;

Demodulator_CPSK	b2v_inst(
	.clk(clk),
	.start(absolute_in),
	.x(SYNTHESIZED_WIRE_0),
	.y(absolute_out));


Modulator_CPSK	b2v_inst1(
	.clk(clk),
	.start(start),
	.x(absolute_in),
	.y(SYNTHESIZED_WIRE_0));


endmodule
