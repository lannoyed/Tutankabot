module spi_slave(
	input  logic 		sck,		// From master 
	input  logic 		mosi,		// From master 
	output logic 		miso,		// To master 
	input  logic 		reset,	// System reset 
	input  logic[31:0] 	d,		// Data to send
	output logic[31:0] 	q		// Data received
);

	logic [4:0] cnt; 
	logic qdelayed;

	// 5-bit counter tracks when full 32-bit word is transmitted
	always_ff @(negedge sck, posedge reset)
		if (reset) 	cnt <= 0;
		else     	cnt <= cnt + 5'b1;

	// Loadable shift register
	// Loads d at the start, shifts mosi into bottom on each step 
	always_ff @(posedge sck)
		q <= (cnt == 0) ? {d[30:0], mosi} : {q[30:0], mosi};
	
	// Align miso to falling edge of sck 
	// Load d at the start
	always_ff @(negedge sck)
		qdelayed <= q[31];
	
	assign miso = (cnt == 0) ? d[31] : qdelayed;

endmodule
