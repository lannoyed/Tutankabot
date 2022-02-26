module mux4 (input logic [15:0] opA, opB, input logic [1:0] control, output logic [15:0] result);
// control = 00 -> result = opA+ opB
// control = 01 -> result = opA - opB
// control = 10 -> result = opA + 1
// control = 11 -> result = opA - 1
// on taff en 16 bist 

logic [15:0] vari_abs;
logic [15:0] vari;
assign vari_abs = control[1] ? 16'd1 : opB;
assign vari = control[0] ? (~vari_abs + 'b1) : vari_abs;

full_adder16 FA16( .a(opA), .b(vari), .sum(result));


endmodule 

