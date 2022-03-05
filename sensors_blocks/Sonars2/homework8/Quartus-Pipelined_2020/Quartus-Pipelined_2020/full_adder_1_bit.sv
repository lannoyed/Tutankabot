module full_adder_1_bit ( 
	input  logic a,
	input  logic b,
	input  logic c_in,
	output logic sum,
	output logic c_out);
								  								  
assign sum = a ^ b ^ c_in;
assign c_out = (a & b) | ((a ^ b) & c_in);

endmodule