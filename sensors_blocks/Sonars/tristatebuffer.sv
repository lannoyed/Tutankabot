module tristatebuffer ( input logic [3:0] a, output logic [3:0] y, input logic enable);
	assign y = enable ? a : 4'bz;
endmodule  
