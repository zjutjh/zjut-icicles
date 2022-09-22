module count6(out,data,load,clk,rst);
	output[5:0]out;
	input[5:0]data;
	input load,clk,rst;
	reg[5:0]out;
	always@(posedge clk )
		if(!rst)
			out=6'b0000;	
		else if(load)
			out=data;
		else
			out=out+1;
endmodule