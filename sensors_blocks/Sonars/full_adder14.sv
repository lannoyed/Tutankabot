module full_adder14 ( input logic [13:0] a, b, output logic [13:0] sum, input logic c_in, output logic c_out );
// simple full adder 14 bits /!\ requiere full_adder8 et full_adder_1bit et full_adder5 
wire c1, c2;

full_adder8 FA80 		(.a(a[7:0]) , .b(b[7:0]),  .c_in(c_in),  .c_out(c1), .sum(sum[7:0]));
full_adder5 FA50 		(.a(a[12:8]), .b(b[12:8]), .c_in(c1),    .c_out(c2), .sum(sum[12:8]));
full_adder_1_bit FA0 (.a(a[13])  , .b(b[13]),   .c_in(c2), .sum(sum[13]),   .c_out(c_out));

endmodule
