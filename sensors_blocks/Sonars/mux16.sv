module mux16 (input [2:0] control, input [15:0] opA, input [15:0] opB, output [15:0] result);
// control = 000 -> result = opA . opB            AND = &
// control = 001 -> result = ~(opA . opB)
// control = 010 -> result = opA + opB            OR  = |
// control = 100 -> result = ~(opA + opB)
// control = 011 -> result = opA Xor opB          XOR = ^
// control = 101 -> result = ~(opA Xor opB)
// control = 110 -> result = ~opA                 NOT = ~
// control = 111 -> result = ~opB 
// on taff en 16 bist 

 assign result = control[2] ? (control[1] ? (control[0] ?  ~opB  : ~opA) : (control[0] ?  opA ~^ opB  : opA ^opB)) 
	    	       : (control[1] ? (control[0] ?  ~(opA | opB)  : opA | opB) : (control[0] ?  ~(opA & opB)  : opA & opB)); 

endmodule 
			 