module full_adder5 (input logic [4:0] a, input logic [4:0] b, input logic c_in, output logic [4:0] sum, output logic c_out);

wire c1, c2, c3, c4, c5, c6, c7;

full_adder_1_bit FA0 ( 
	.a(a[0]),
	.b(b[0]),
	.c_in(c_in),
	.sum(sum[0]),
	.c_out(c1));
							  
full_adder_1_bit FA1 ( 
	.a(a[1]),
	.b(b[1]),
	.c_in(c1),
	.sum(sum[1]),
	.c_out(c2));
							  
full_adder_1_bit FA2 ( 
	.a(a[2]),
	.b(b[2]),
	.c_in(c2),
	.sum(sum[2]),
	.c_out(c3));
							  
full_adder_1_bit FA3 ( 
	.a(a[3]),
	.b(b[3]),
	.c_in(c3),
	.sum(sum[3]),
	.c_out(c_out));

full_adder_1_bit FA4 ( 
	.a(a[4]),
	.b(b[4]),
	.c_in(c4),
	.sum(sum[4]),
	.c_out(c_out));
							  


endmodule
