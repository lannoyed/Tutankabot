//module test_sonar ();

`timescale 1ns/1ps


module De0_testbench ();

wire 	[33:0]	GPIO_0_PI;
wire 	[1:0]		GPIO_0_PI_IN;
wire 	[33:0]	GPIO_1;
wire  [1:0]		GPIO_1_IN;
logic  			clk;
logic [7:0] 	LED;
logic [1:0] 	KEY; // KEY[1] = reset. KEY[0] : change le mode de display (ordre d'accès des sonars) 

logic Echo1, Echo2 ,Echo3 ,Echo4 ,Echo5;
logic Trigger1 ,Trigger2 ,Trigger3 ,Trigger4 ,Trigger5;


 MyDE0_Nano dut(	
	.CLOCK_50(clk),
	.LED(LED),
	.KEY(KEY),
	.GPIO_0_PI(GPIO_0_PI),
	.GPIO_0_PI_IN(GPIO_0_PI_IN),
	.GPIO_1(GPIO_1),
	.GPIO_1_IN(GPIO_1_IN)
);			 


// 5 sonars. IN : Echo (que dans un sens) et Trigger (envoi du son pour capter obstacle) est in/out.
// Dans minibot : 19 (GPIO_014) : Echo et 20 (GPIO_015) : Trigger (un seul sonar). C'est dans GPIO_0_PI a priori.
assign Echo1 = GPIO_1[2];
assign GPIO_1[3] = Trigger1;

assign Echo2 = GPIO_1[4];
assign GPIO_1[5] = Trigger2;


/*assign GPIO_1[6] = Echo3;
assign Trigger3 = GPIO_1[8];

assign GPIO_1[10] = Echo4;
assign Trigger4 = GPIO_1[12];

assign GPIO_1[14] = Echo5;
assign Trigger5 = GPIO_1[16];*/



always #10 clk = ~clk;


initial 
	begin
		KEY[1] = 1'b0; // Reset
		KEY[0] = 1'b0; 
		
		clk = 1'b0; // Lancement de clock
		
		// Tout à 0 pour echo : reset d'Echo.
		Echo1 = 1'b0;
		Echo2 = 1'b0;
		/*Echo3 = 1'b0;
		Echo4 = 1'b0;
		Echo5 = 1'b0;*/
		
		#70
		
		KEY[1] = 1'b1;	// On sort du reset (rappel : KEY en active low)
		KEY[0] = 1'b1; 
		
		#20 
		
		KEY[0] = 1'b0; // Display mode
		
		
		#290000
		
		Echo1 = 1'b1;
		Echo2 = 1'b1;
		/*Echo3 = 1'b1;
		Echo4 = 1'b1;
		Echo5 = 1'b1;*/
		
		#290000
		
		Echo1 = 1'b0;
		
		#290000
		
		Echo2 = 1'b0;
		
		#290000
		
		//Echo3 = 1'b0;
		
		#290000
		
		//Echo4 = 1'b0;
		
		#290000
		
		//Echo5 = 1'b0;
		
		#58260000
		
		KEY[0] = 1'b1;
		#1
		KEY[0] = 1'b0;
	end
endmodule  
