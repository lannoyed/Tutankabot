module full_adder14 ( input logic [13:0] a, b, output logic [13:0] sum, input logic c_in, output logic c_out );
// simple full adder 14 bits /!\ requiere full_adder8 et full_adder_1bit et full_adder5 
wire c1, c2, c3, c4, c5;

full_adder9 FA90 		 (.a(a[8:0]) , .b(b[8:0]),  .c_in(c_in), .c_out(c1),  .sum(sum[8:0]));
full_adder_1_bit FA10 (.a(a[9])   , .b(b[9]),    .c_in(c1), .sum(sum[9]),    .c_out(c2));
full_adder_1_bit FA11 (.a(a[10])  , .b(b[10]),   .c_in(c2), .sum(sum[10]),   .c_out(c3));
full_adder_1_bit FA12 (.a(a[11])  , .b(b[11]),   .c_in(c3), .sum(sum[11]),   .c_out(c4));
full_adder_1_bit FA13 (.a(a[12])  , .b(b[12]),   .c_in(c4), .sum(sum[12]),   .c_out(c5));
full_adder_1_bit FA14 (.a(a[13])  , .b(b[13]),   .c_in(c5), .sum(sum[13]),   .c_out(c_out));

endmodule
