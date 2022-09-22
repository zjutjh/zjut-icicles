module add_1(
	a,
	b,
	cin,
	cout,
	sum
);

input wire	a;
input wire	b;
input wire	cin;
output wire	cout;
output wire	sum;

wire	SYNTHESIZED_WIRE_0;
wire	SYNTHESIZED_WIRE_1;
wire	SYNTHESIZED_WIRE_2;

h_add	b2v_inst1(
	.a(a),
	.b(b),
	.cout(SYNTHESIZED_WIRE_2),
	.sum(SYNTHESIZED_WIRE_0));


h_add	b2v_inst2(
	.a(SYNTHESIZED_WIRE_0),
	.b(cin),
	.cout(SYNTHESIZED_WIRE_1),
	.sum(sum));
assign	cout = SYNTHESIZED_WIRE_1 | SYNTHESIZED_WIRE_2;

endmodule
