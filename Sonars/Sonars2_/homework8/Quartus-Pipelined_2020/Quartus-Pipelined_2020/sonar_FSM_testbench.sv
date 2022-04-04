`timescale 1ns/1ps

module sonar_FSM_testbench ();

logic [19:0] 	distance;
logic 			clock, reset; 
logic [1:0] 	Flags;
logic 			Trigger, Echo;
logic 			TIC;

sonar_FSM sonar_FSM_01(clock, reset, Trigger, Echo, TIC, distance[19:0],Flags[1:0]);

always #10 clock = ~clock;

initial
	begin
		clock = 1'b0;
		reset = 1'b1;
		TIC = 1'b0;
		Echo = 1'b0;
		
		#80
		
		reset = 1'b0;
		
		#80
		
		TIC = 1'b1;
		
		#80 
		
		Echo = 1'b1;
		TIC = 1'b0;
		
		#289920		
		
		#59710000
		
		TIC = 1'b1;
		
		#80
		
		TIC = 1'b0;
		
		#28920
		
		Echo = 1'b0;
	end
endmodule  
