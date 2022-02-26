module Homework_4(input  logic [31:0] a, b, 
           input  logic [1:0]  ALUControl, 
           output logic [31:0] Result, 
           output logic [3:0]  ALUFlags); 

// ALUControl : 00 Addition 01 Substraction 10 AND 11 OR
//ALUFlag : 3 result negative 2 result = 0 1 adder carry out 0 adder overflow


// Adder
logic [31:0] sum;
logic [31:0] in_b;
logic c_in;
logic c_out;
// flags 
logic carry;
logic sign_different_a_b;
logic sign_different_a_sum;
logic overflow;
logic negative;
logic zero;


// ADD / SUB in the full adder
assign in_b = ALUControl[0] ? ~b : b;
assign c_in = ALUControl[0] ? 1'b1 : 1'b0; 

// Result Attribution 
assign Result = ALUControl[1] ? (ALUControl[0] ? a | b : a & b ) : sum;

//Carry 
assign carry = c_out & ~ALUControl[1];
//Overflow 
assign sign_different_a_b = ~(a[31] ^ b[31] ^ ALUControl[0]); 
assign sign_different_a_sum = sum[31] ^a[31];
assign overflow = sign_different_a_b & sign_different_a_sum & ~ALUControl[1];
//Negative 
assign negative = Result[31];
assign zero = ~(|Result); 
assign ALUFlags = {negative, zero, carry, overflow};

full_adder32 FA32_0(.a(a), .b(in_b), .sum(sum) , .c_out(c_out), .c_in(c_in));

endmodule
