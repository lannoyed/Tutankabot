module full_adder20 ( input logic [19:0] a, b, output logic [19:0] sum, input logic c_in, output logic c_out );
	
	// simple full adder 14 bits /!\ requiere full_adder8 et full_adder_1bit et full_adder5 
	wire c1, c2, c3;

	full_adder9 FA90 		 (.a(a[8:0]) , .b(b[8:0]),  .c_in(c_in),.c_out(c1),     .sum(sum[8:0]));
	full_adder9 FA91 		 (.a(a[17:9]), .b(b[17:9]), .c_in(c1), .c_out(c2),      .sum(sum[17:9]));
	full_adder_1_bit FA19 (.a(a[18])  , .b(b[18]),   .c_in(c2), .sum(sum[18]),   .c_out(c3));
	full_adder_1_bit FA20 (.a(a[19])  , .b(b[19]),   .c_in(c3), .sum(sum[19]),   .c_out(c_out));
	
endmodule
