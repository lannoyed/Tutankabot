
//=======================================================
//=======  GENERALISED CODE FOR THE DE0-NANO PART =======
//=======================================================

module DE0_NANO_ENC(

	//////////// CLOCK //////////
	CLOCK_50,

	//////////// LED //////////
	LED,

	//////////// KEY //////////
	KEY,

	//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	GPIO_0,
	GPIO_0_IN,
	GPIO_1,
	GPIO_1_IN 
	
);


//=======================================================
//  PORT declarations
//=======================================================

//////////// CLOCK //////////
input 		          		CLOCK_50;

//////////// LED //////////
output		     [7:0]		LED;

//////////// KEY //////////
input 		     [1:0]		KEY;

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout 		    [33:0]		GPIO_0;
input 		     [1:0]		GPIO_0_IN;

inout 		    [33:0]		GPIO_1;
input 		     [1:0]		GPIO_1_IN;


//=======================================================
//============== ODOMETERS & ENCODERS PART ==============
//=======================================================

	logic clk;
	logic rst;

	assign clk = CLOCK_50;
	
	// For SPI (later stages)
	logic [39:0] d ; 
	logic [39:0] q ; 

   logic enc1a, enc1b, enc2a, enc2b, odo1a, odo1b, odo2a, odo2b; 
	
	logic [31:0] countOdo1;
	logic [31:0] countOdo2;
	logic [31:0] countEnc1;
	logic [31:0] countEnc2;
	
	logic [31:0] spi_mem[15:0]; // Stores the datas sent to the de from the Pi 

	logic clock_slow ; 
	logic [31:0]speedOdo1, speedOdo2, speedEnc1, speedEnc2;

	assign enc1a = GPIO_1[33];
	assign enc1b = GPIO_1[31];
	
	assign enc2a = GPIO_1[29];
	assign enc2b = GPIO_1[27];
	
	assign odo1a = GPIO_1[14];
	assign odo1b = GPIO_1[16];
	
	assign odo2a = GPIO_1[10];
	assign odo2b = GPIO_1[12];
	
	assign rst = ~KEY[0];

	
	clock_divider clock_divider1(
		.clock_in(clk), 
		.clock_out(clock_slow)
	);
	
	quad_dec decoder1(
		.clk  (clk), 
		.reset  (rst),
		.A  (odo1a), 
		.B  (odo1b),
		.count  (countOdo1))
	;

	quad_dec decoder2(
		.clk  (clk), 
		.reset  (rst),
		.A  (odo2a), 
		.B  (odo2b),
		.count  (countOdo2))
	;
	
	quad_dec decoder3(
		.clk  (clk), 
		.reset  (rst),
		.A  (enc1a), 
		.B  (enc1b),
		.count  (countEnc1))
	;

	quad_dec decoder4(
		.clk  (clk), 
		.reset  (rst),
		.A  (enc2a), 
		.B  (enc2b),
		.count  (countEnc2))
	;

	speed_tick_computer stcOdo1(
		.clock_slow(clock_slow), 
		.count(countOdo1), 
		.speed_tick(speedOdo1)
	);
	
	speed_tick_computer stcOdo2(
		.clock_slow(clock_slow), 
		.count(countOdo2), 
		.speed_tick(speedOdo2)
	);

	speed_tick_computer stcEnc1(
		.clock_slow(clock_slow), 
		.count(countEnc1), 
		.speed_tick(speedEnc1)
	);

	speed_tick_computer stcEnc2(
		.clock_slow(clock_slow), 
		.count(countEnc2), 
		.speed_tick(speedEnc2)
	);

	
	
//=======================================================
//=============== SONARS' PART : 5 SONARS ===============
//=======================================================

logic [19:0] 	distance1, distance2, distance3, distance4, distance5;
logic 			reset;
logic 			Echo1, Echo2 ,Echo3 ,Echo4 ,Echo5;
logic 			Trigger1 ,Trigger2 ,Trigger3 ,Trigger4 ,Trigger5;
logic [1:0] 	Flags1, Flags2, Flags3, Flags4, Flags5; 

logic [2:0] 	led_afficheur;

logic [24:0] 	a;
logic [24:0] 	sum;
logic 			c_out;

logic triggerLowStart1, triggerLowStart2, triggerLowStart3, triggerLowStart4, triggerLowStart5;
logic reset1, reset2, reset3, reset4, reset5;



// Mise en place des FMS pour chaque sonars
sonar_FSM sonar_FSM_01( CLOCK_50, reset1, Trigger1, Echo1, triggerLowStart1, distance1[19:0], Flags1 [1:0]);
sonar_FSM sonar_FSM_02( CLOCK_50, reset2, Trigger2, Echo2, triggerLowStart2, distance2[19:0], Flags2 [1:0]);
sonar_FSM sonar_FSM_03( CLOCK_50, reset3, Trigger3, Echo3, triggerLowStart3, distance3[19:0], Flags3 [1:0]);
sonar_FSM sonar_FSM_04( CLOCK_50, reset4, Trigger4, Echo4, triggerLowStart4, distance4[19:0], Flags4 [1:0]);
sonar_FSM sonar_FSM_05( CLOCK_50, reset5, Trigger5, Echo5, triggerLowStart5, distance5[19:0], Flags5 [1:0]);

// "sum" représente le compteur qui va nous permettre de calculer les 61 [ms] de fonctionnement.
full_adder25 FA25_0(a, 25'd1, sum, 1'b0, c_out); 

always_ff @(posedge CLOCK_50)

	begin
		if (KEY[1] == 1'b0) reset <= 1'b1; // reset manuel (uniquement utile pour les tests)
		else reset <= 1'b0; //reset1 <= 1'b0; reset2 <= 1'b0; end
	
	 
		if (reset) begin  
			a <= 25'd0; 
			
			triggerLowStart1 = 1'b1; 
			triggerLowStart2 = 1'b1; 
			triggerLowStart3 = 1'b1; 
			triggerLowStart4 = 1'b1;
			triggerLowStart5 = 1'b1;
			
			reset1 = 1'b1; 
			reset2 = 1'b1; 
			reset3 = 1'b1;
			reset4 = 1'b1;
			reset5 = 1'b1;

		end
		else if (sum == 25'd1) begin 
			triggerLowStart1 = 1'b1; 
			triggerLowStart2 = 1'b1; 
			triggerLowStart3 = 1'b1; 
			triggerLowStart4 = 1'b1;
			triggerLowStart5 = 1'b1;
			
			a <= sum; 
			
			reset1 = 1'b0; 
			reset2 = 1'b0; 
			reset3 = 1'b0;
			reset4 = 1'b0;
			reset5 = 1'b0;
		end
		else if (sum == 25'd3_050_000) begin 
			triggerLowStart1 = 1'b0; 
			triggerLowStart2 = 1'b0; 
			triggerLowStart3 = 1'b0; 
			triggerLowStart4 = 1'b0;
			triggerLowStart5 = 1'b0; 
			
			a <= 25'd0; 
			
			reset1 = 1'b1; 
			reset2 = 1'b1; 
			reset3 = 1'b1;
			reset4 = 1'b1;
			reset5 = 1'b1; 
		end 
		else begin 
			a <= sum; 
			
			triggerLowStart1 = 1'b0; 
			triggerLowStart2 = 1'b0; 
			triggerLowStart3 = 1'b0; 
			triggerLowStart4 = 1'b0;
			triggerLowStart5 = 1'b0;  
			
			reset1 = 1'b0; 
			reset2 = 1'b0; 
			reset3 = 1'b0;
			reset4 = 1'b0;
			reset5 = 1'b0;
		end
	end

// Attribution des pins pour tous les sonars : cfr. l'image dans le Teams -> Paques -> Branchement_Modifié_1.png. ATTENTION : l'ordre est inversé
// car pour les branchements, c'était impossible de faire autrement. Donc, le sonar 1 est "de l'autre côté" de ce qu'il devrait être.


// SONAR 1
assign GPIO_1[17] = Trigger1;
assign Echo1 = GPIO_1[15];

// SONAR 2
assign GPIO_1[13] = Trigger2;
assign Echo2 = GPIO_1[11];

// SONAR 3
assign Echo3 = GPIO_1[9];

// SONAR 5
assign GPIO_1[7] = Trigger5;
assign Echo5 = GPIO_1[5];

// SONAR 4
assign GPIO_1[3] = Trigger4;
assign Echo4 = GPIO_1[1];

// FIN SONAR 3
assign GPIO_1[0] = Trigger3;

	
//=======================================================
//===== SPI -> SEND INFORMATION TO THE RASPBERRY PI =====
//=======================================================

	logic 			spi_clk, spi_cs, spi_mosi, spi_miso;

	logic [7:0] 	AddressToSend ; 
	logic [7:0] 	AddressReceived ; 
	logic [31:0] 	DataToSend;
	logic [31:0] 	DataReceived;
	
	spi_slave spi_slave_instance(
		.sck    	(spi_clk),			 					//input clock given by the master
		.reset  	(spi_cs),			 					//input reset signal 
		.mosi   	(spi_mosi),			 					//input mosi
		.miso   	(spi_miso),			 					//output miso
		.d  		(d),										//input data to send	: DataToSend et AddressToSend passent par-là.
		.q 		(q) 										//output data received 
	);

	assign spi_clk    = 	GPIO_0[11];						// SCLK = pin 13 = miroir pin 23 = GPIO_18
	assign spi_cs     = 	GPIO_0[9];						// CE1  = pin 15 = miroir pin 25 = GPIO_20
	assign spi_mosi   = 	GPIO_0[15];						// MOSI = pin 14 = miroir pin 19 = GPIO_24
	assign GPIO_0[13] = 	spi_cs ? 1'bz : spi_miso; // MISO = pin 16 = miroir pin 26 = GPIO_21


	
//========================================================
//===== Memory Management : what to send to the PI ? =====
//========================================================	
	
	always_comb begin
		case(AddressReceived)  
			// Odometers & Encoders
			8'd0 	: DataToSend = countOdo1;
			8'd1 	: DataToSend = countOdo2;
			
			8'd2 	: DataToSend = countEnc1;
			8'd3 	: DataToSend = countEnc2;
			
			8'd4 	: DataToSend = speedOdo1; 
			8'd5 	: DataToSend = speedOdo2; 
			
			8'd6 	: DataToSend = speedEnc1; 
			8'd7 	: DataToSend = speedEnc2; 
			
			
			// Sonars
			8'd8  : DataToSend = {12'd0, distance1};
			8'd9	: DataToSend = {12'd0, distance2};
			8'd10	: DataToSend = {12'd0, distance3};
			8'd11 : DataToSend = {12'd0, distance4};
			8'd12 : DataToSend = {12'd0, distance5};
			
			
			default: DataToSend = 32'b1111_1111_1111_1111_1111_1111_1111_1111;
		endcase
	end
	
	assign d[39:32] = AddressToSend; 
	assign d[31:0] = DataToSend; 
	assign AddressReceived = q[39:32]; 
	assign DataReceived = q[31:0];
	
	
//======================================================
//============  LEDS FOR DEBUGGING VISUALLY ============
//======================================================


assign LED = distance5[19:12];
	

endmodule
