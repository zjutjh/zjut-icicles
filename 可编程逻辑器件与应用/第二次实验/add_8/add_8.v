module add_8(
	cin,
	A,
	B,
	COUNT,
	SUM
);


input wire	cin;
input wire	[7:0] A;
input wire	[7:0] B;
output wire	COUNT;
output wire	[7:0] SUM;

wire	[7:0] SUM_ALTERA_SYNTHESIZED;
wire	SYNTHESIZED_WIRE_0;
wire	SYNTHESIZED_WIRE_1;
wire	SYNTHESIZED_WIRE_2;
wire	SYNTHESIZED_WIRE_3;
wire	SYNTHESIZED_WIRE_4;
wire	SYNTHESIZED_WIRE_5;
wire	SYNTHESIZED_WIRE_6;





add_1	b2v_inst(
	.a(A[0]),
	.b(B[0]),
	.cin(cin),
	.cout(SYNTHESIZED_WIRE_0),
	.sum(SUM_ALTERA_SYNTHESIZED[0]));


add_1	b2v_inst2(
	.a(A[1]),
	.b(B[1]),
	.cin(SYNTHESIZED_WIRE_0),
	.cout(SYNTHESIZED_WIRE_1),
	.sum(SUM_ALTERA_SYNTHESIZED[1]));


add_1	b2v_inst3(
	.a(A[2]),
	.b(B[2]),
	.cin(SYNTHESIZED_WIRE_1),
	.cout(SYNTHESIZED_WIRE_2),
	.sum(SUM_ALTERA_SYNTHESIZED[2]));


add_1	b2v_inst4(
	.a(A[3]),
	.b(B[3]),
	.cin(SYNTHESIZED_WIRE_2),
	.cout(SYNTHESIZED_WIRE_3),
	.sum(SUM_ALTERA_SYNTHESIZED[3]));


add_1	b2v_inst5(
	.a(A[4]),
	.b(B[4]),
	.cin(SYNTHESIZED_WIRE_3),
	.cout(SYNTHESIZED_WIRE_4),
	.sum(SUM_ALTERA_SYNTHESIZED[4]));


add_1	b2v_inst6(
	.a(A[5]),
	.b(B[5]),
	.cin(SYNTHESIZED_WIRE_4),
	.cout(SYNTHESIZED_WIRE_5),
	.sum(SUM_ALTERA_SYNTHESIZED[5]));


add_1	b2v_inst7(
	.a(A[6]),
	.b(B[6]),
	.cin(SYNTHESIZED_WIRE_5),
	.cout(SYNTHESIZED_WIRE_6),
	.sum(SUM_ALTERA_SYNTHESIZED[6]));


add_1	b2v_inst8(
	.a(A[7]),
	.b(B[7]),
	.cin(SYNTHESIZED_WIRE_6),
	.cout(COUNT),
	.sum(SUM_ALTERA_SYNTHESIZED[7]));

assign	SUM = SUM_ALTERA_SYNTHESIZED;

endmodule
