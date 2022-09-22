//通信工程1803 凌智城201806061211
module DPSK_top(
	clk,
	start,
	absolute_in,
	absolute_out,
	relative_in,
	relative_out
);
input wire	clk;
input wire	start;
input wire	absolute_in;
output wire	absolute_out;
output wire	relative_in;
output wire	relative_out;

wire	SYNTHESIZED_WIRE_0;
wire	SYNTHESIZED_WIRE_1;
wire	SYNTHESIZED_WIRE_2;
assign	relative_in = SYNTHESIZED_WIRE_1;
assign	relative_out = SYNTHESIZED_WIRE_2;
Demodulator_CPSK	b2v_inst(
	.clk(clk),
	.start(start),
	.x(SYNTHESIZED_WIRE_0),
	.y(SYNTHESIZED_WIRE_2));
Modulator_CPSK	b2v_inst1(
	.clk(clk),
	.start(start),
	.x(SYNTHESIZED_WIRE_1),
	.y(SYNTHESIZED_WIRE_0));
absolute_relative_code	b2v_inst2(
	.clk(clk),
	.start(start),
	.x(absolute_in),
	.y(SYNTHESIZED_WIRE_1));
relative_absolute_code	b2v_inst3(
	.clk(clk),
	.start(start),
	.x(SYNTHESIZED_WIRE_2),
	.y(absolute_out));
endmodule
