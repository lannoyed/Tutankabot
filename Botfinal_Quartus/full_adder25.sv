
module full_adder25 ( input logic [24:0] a, b, output logic [24:0] sum, input logic c_in, output logic c_out );
	
	// simple full adder 14 bits /!\ requiere full_adder8 et full_adder_1bit et full_adder5 
	wire c1, c2, c3, c4, c5, c6, c7, c8;

	full_adder9 FA90 		 (.a(a[8:0]) , .b(b[8:0]),  .c_in(c_in),	.sum(sum[8:0]), 	.c_out(c1));
	full_adder9 FA91 		 (.a(a[17:9]), .b(b[17:9]), .c_in(c1), 	.sum(sum[17:9]), 	.c_out(c2));
	full_adder_1_bit FA19 (.a(a[18])  , .b(b[18]),   .c_in(c2), 	.sum(sum[18]),   	.c_out(c3));
	full_adder_1_bit FA20 (.a(a[19])  , .b(b[19]),   .c_in(c3), 	.sum(sum[19]),  	.c_out(c4));
	full_adder_1_bit FA21 (.a(a[20])  , .b(b[20]),   .c_in(c4), 	.sum(sum[20]),   	.c_out(c5));
	full_adder_1_bit FA22 (.a(a[21])  , .b(b[21]),   .c_in(c5), 	.sum(sum[21]),   	.c_out(c6));
	full_adder_1_bit FA23 (.a(a[22])  , .b(b[22]),   .c_in(c6), 	.sum(sum[22]),   	.c_out(c7));
	full_adder_1_bit FA24 (.a(a[23])  , .b(b[23]),   .c_in(c7), 	.sum(sum[23]),   	.c_out(c8));
	full_adder_1_bit FA25 (.a(a[24])  , .b(b[24]),   .c_in(c8), 	.sum(sum[24]),   	.c_out(c_out));

endmodule
