module spi_slave(
	input 	logic sck, 			// Clock given by the master 
	input 	logic mosi, 		// Master out slave in 
	output 	logic miso, 		// Master in slave out 
	input 	logic reset, 		// System reset 
	input	logic[39:0]	d, 		// Data to send 
	output 	logic[39:0] q		// Data received 
);

	logic [6:0] cnt ; 
	logic 		qdelayed ; 

	// 5-bit counter tracks when full byte is transmitted 
	always_ff @(negedge sck, posedge reset ) begin 
		if (reset) cnt = 0 ; 
		else cnt = cnt + 6'b1 ; 
	end 

	// Loadable shift register 
	// Loads d at the start, shifts mosi into bottom on each step 

	always_ff @(posedge sck) begin
		q <= (cnt == 0) ? {d[38:0], mosi} : {q[38:0], mosi} ; 
	end 

	always_ff @(negedge sck) begin 
		qdelayed = q[39] ; 
	end

	assign miso = (cnt==0) ? d[39] : qdelayed ; 
endmodule 