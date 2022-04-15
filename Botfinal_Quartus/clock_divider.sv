module clock_divider(
	input 		clock_in, 
	output	logic	clock_out 
); 

logic[31:0] counter ; 
parameter divisor = 32'd500000 ; 
always @(posedge clock_in) 
	begin
		counter <= counter + 32'b1 ; 
		if (counter >= divisor -1) begin
			counter <= 32'b0 ; 
			clock_out <= ~clock_out ; 
		end
	end
	
endmodule 
