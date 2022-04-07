module speed_tick_computer(
	input clock_slow, 
	input logic signed [31:0] count, 
	output logic signed [31:0] speed_tick 
);

logic signed [31:0] count1, count2; 
logic signed [31:0] diff ;  
logic sign ; 

always_ff @(posedge clock_slow) begin 
	count1 <= count2 ; 
	count2 <= count ; 
end 

always_comb begin
	diff = count2-count1 ; 
	sign = 0 ;
	if (diff[31]) begin
		sign = 1 ; 
		diff = count1-count2 ; 
	end 
end
	 
assign speed_tick = {7'b000,sign, diff[23:0]} ; 
endmodule 