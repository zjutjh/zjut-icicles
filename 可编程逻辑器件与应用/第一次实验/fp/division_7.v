module division_7(clk,rst,clk_even);
    input clk,rst;
    output clk_even;
    reg[3:0] count1,count2;
    reg clkA,clkB;
    parameter N=7;

    assign clk_re=~clk;
    assign clk_even=clkA|clkB;

    always @(posedge clk)
       if(!rst)
           begin
               count1<=1'b0;
               clkA<=1'b0;
           end      
	else if(N%2==1)
	    begin
		if(count1<(N-1))
                    begin
                       count1<=count1+1'b1;
                            if(count1==(N-1)/2)
                                begin
                                    clkA=~clkA;
                                end
	     	    end
                 else
                       begin
                           clkA=~clkA;
                           count1=1'b0;
                       end
            end
	else
    	    clkA=1'b0;

    always @(posedge clk_re)
       if(!rst)
           begin
               count2<=1'b0;
               clkB<=1'b0;
          end
       else if(N%2==1)
             begin
		if(count2<(N-1))
                    begin
                       count2<=count2+1'b1;
                            if(count2==(N-1)/2)
                                begin
                                    clkB=~clkB;
                                end
	             end
	       else
         	   begin
             		clkB=~clkB;
           		  count2=1'b0;
         	   end
  	    end
	else
      clkB=1'b0;
endmodule
