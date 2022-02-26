// This test bench is HIGHLLY INSPIRED of the Exemple 4.39 of the reference book
// More Preciselly This code is a addapation of the Example 4.39

module Homework_4_tb();
logic clk, reset;

logic [31:0] a, b;
logic [1:0] trash;
logic [1:0]  ALUControl;
logic [31:0] Result, Result_Expected; 
logic [3:0]  ALUFlags, ALUFlags_Expected;
logic [31:0] vectornum, errors;
logic [103:0] testvectors[10000:0];
// instantiate device under test

Homework_4 dut(.a(a), .b(b), .ALUControl(ALUControl), .Result(Result) , .ALUFlags(ALUFlags));
// generate clock

always begin
		clk =1; #5; clk =0; #5;
end

// at start of test, load vectors
// and pulse reset

initial begin
$readmemh("Homework_4.tv", testvectors);
vectornum =0; errors =0;
reset =1; #27; reset =0;
end

// apply test vectors on rising edge of clk
always @(posedge clk)
begin
#1; {trash, ALUControl, a, b, Result_Expected, ALUFlags_Expected} =testvectors[vectornum];
end
// check results on falling edge of clk
always @(negedge clk)
if (~reset) begin // skip during reset
if (Result !== Result_Expected) begin // check result
$display("Error: inputs =%h", {a, b, ALUControl});
$display(" outputs =%h (%h expected)", Result, Result_Expected);
errors =errors +1;
end

if (ALUFlags !== ALUFlags_Expected) begin 
$display("Error: inputs =%h", {a, b, ALUControl});
$display(" outputs =%h (%h expected)", ALUFlags,ALUFlags_Expected );
errors =errors +1;
end

vectornum =vectornum +1;
if (testvectors[vectornum] ===4'bx) begin
$display("%d tests completed with %d errors",
vectornum, errors);
$finish;
end

end
endmodule
