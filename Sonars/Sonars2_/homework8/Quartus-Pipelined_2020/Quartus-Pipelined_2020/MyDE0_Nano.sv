
//=======================================================
//  MyARM
//=======================================================

module MyDE0_Nano(

//////////// CLOCK //////////
input logic 		          		CLOCK_50,

//////////// LED //////////
output logic		     [7:0]		LED,

//////////// KEY //////////
input logic 		     [1:0]		KEY,

//////////// SW //////////
input logic 		     [3:0]		SW, 

//////////// SDRAM //////////
output logic		    [12:0]		DRAM_ADDR,
output logic		     [1:0]		DRAM_BA,
output logic		          		DRAM_CAS_N,
output logic		          		DRAM_CKE,
output logic		          		DRAM_CLK,
output logic		          		DRAM_CS_N,
inout logic 		    [15:0]		DRAM_DQ,
output logic		     [1:0]		DRAM_DQM,
output logic		          		DRAM_RAS_N,
output logic		          		DRAM_WE_N,

//////////// EPCS //////////
output logic		          		EPCS_ASDO,
input logic 		          		EPCS_DATA0,
output logic		          		EPCS_DCLK,
output logic		          		EPCS_NCSO,

//////////// Accelerometer and EEPROM //////////
output logic		          		G_SENSOR_CS_N,
input logic 		          		G_SENSOR_INT,
output logic		          		I2C_SCLK,
inout logic 		          		I2C_SDAT,

//////////// ADC //////////
output logic		          		ADC_CS_N,
output logic		          		ADC_SADDR,
output logic		          		ADC_SCLK,
input logic 		          		ADC_SDAT,

//////////// 2x13 GPIO Header //////////
inout logic 		    [12:0]		GPIO_2,
input logic 		     [2:0]		GPIO_2_IN,

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout logic 		    [33:0]		GPIO_0_PI,
input logic 		     [1:0]		GPIO_0_PI_IN,

//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
inout logic 		    [33:0]		GPIO_1,
input logic 		     [1:0]		GPIO_1_IN
);			

//=======================================================
//  MyARM
//=======================================================

 
// variables 
logic [19:0] 	distance1, distance2, distance3, distance4, distance5;
logic 			reset;
logic 			Echo1, Echo2 ,Echo3 ,Echo4 ,Echo5;
logic 			Trigger1 ,Trigger2 ,Trigger3 ,Trigger4 ,Trigger5;
logic [1:0] 	Flags1, Flags2, Flags3, Flags4, Flags5; 

logic [2:0] 	led_afficheur;

logic [24:0] 	a;
logic [24:0] 	sum; //25
logic 			c_out;

logic triggerLowStart1, triggerLowStart2;


//logic state_SB, state_TL, state_IT, state_TM, state_IM, state_CO, state_OV;

	
logic triggerLowStart; // Envoyé automatiquement toutes les 60 [ms]. 

	
//=======================================================
//  SPI
//=======================================================


/*	
logic spi_clk, spi_cs, spi_mosi, spi_miso;
logic [31:0] DataToPI, DataFromPI;

	spi_slave spi_slave_instance(
		.sck(spi_clk),
		.mosi(spi_mosi),
		.miso(spi_miso),
		.reset(),
		.d(DataToPI),
		.q(DataFromPI)
	);

	assign spi_clk  		= GPIO_0_PI[11];	// SCLK = pin 16 = GPIO_11
	assign spi_cs   		= GPIO_0_PI[9];	// CE0  = pin 14 = GPIO_9
	assign spi_mosi     	= GPIO_0_PI[15];	// MOSI = pin 20 = GPIO_15
	
	assign GPIO_0_PI[13] = spi_cs ? 1'bz : spi_miso;  // MISO = pin 18 = GPIO_13		


always_comb
	begin
		DataToPI [31:16] = 16'b0;
		if (reset) DataToPI[15:0] = 16'b0;  // À CHANGER, DISTANCE FAIT 23 BITS MAINTENANT
		else begin
			casex (DataFromPI[31:0])
			32'd1 : begin DataToPI[13:0] = distance1; DataToPI[15:14] = Flags1[1:0]; end
			32'd2 : begin DataToPI[13:0] = distance2; DataToPI[15:14] = Flags2[1:0]; end
			32'd3 : begin DataToPI[13:0] = distance3; DataToPI[15:14] = Flags3[1:0]; end
			32'd4 : begin DataToPI[13:0] = distance4; DataToPI[15:14] = Flags4[1:0]; end
			32'd5 : begin DataToPI[13:0] = distance5; DataToPI[15:14] = Flags5[1:0]; end
			default begin DataToPI[15:0] = 16'bx; end
		endcase 
	end
end*/
		
//=======================================================
//  Sonars
//=======================================================

// Commentaires
// il y a 2 possibilitÃ©s de fonctionnement 
// calculer tous les sonars Ã  la fois -> high performance mode 
// effectuer le calcul un par un      -> lowperformance mode
// ICI seul High Performance mode est implÃ©mentÃ© 

	
// Choses Ã  faire : 
// 	- envoie de signal et rÃ©ception de signal par le De0 nano
// 	- Communication SPI
//		- test de calibrage
// demander quid des GPIO INS

 
// mise en place des FMS pour chaque sonars
sonar_FSM sonar_FSM_01( CLOCK_50, reset, Trigger1, Echo1, triggerLowStart1, distance1[19:0] , Flags1 [1:0]);

sonar_FSM sonar_FSM_02( CLOCK_50, reset, Trigger2, Echo2, triggerLowStart2, distance2[19:0] , Flags2 [1:0] );
/*sonar_FSM sonar_FSM_03( CLOCK_50, reset, Trigger3, Echo3, triggerLowStart, distance3[19:0] , Flags3 [1:0] );
sonar_FSM sonar_FSM_04( CLOCK_50, reset, Trigger4, Echo4, triggerLowStart, distance4[19:0] , Flags4 [1:0] );
sonar_FSM sonar_FSM_05( CLOCK_50, reset, Trigger5, Echo5, triggerLowStart, distance5[19:0] , Flags5 [1:0] );*/

full_adder25 FA25_0( a,25'd1,sum,1'b0,c_out); // sum représente le compteur

always_ff @(posedge CLOCK_50)
	begin 
		if (reset) begin  a <= 25'd0; triggerLowStart = 1'b1; end                       // triggerLowStart est mis à l'initialisation 
		else if (sum == 25'd1) begin triggerLowStart = 1'b1; a <= sum; end
		else if (sum == 25'd3_050_000) begin triggerLowStart = 1'b0; a <= 25'd0; end 
		else begin a <= sum; triggerLowStart = 1'b0; end
	end



// attribution des pins : sur minibot

//assign Echo1 = GPIO_0_PI[16];
//assign GPIO_0_PI[17] = Trigger1;


// attribution des pins : sur notre robot


assign Echo1 = GPIO_1[2];
assign GPIO_1[3] = Trigger1;

assign Echo2 = GPIO_1[4];
assign GPIO_1[5] = Trigger2;

/*assign Echo3 = GPIO_1[6];
assign GPIO_1[8] = Trigger3;

assign Echo4 = GPIO_1[10];
assign GPIO_1[12] = Trigger4;

assign Echo5 = GPIO_1[14];
assign GPIO_1[16] = Trigger5;*/


assign GPIO_1[33] = 1'b0;
assign GPIO_1[32] = 1'b0;
assign GPIO_1[0] = 1'b0;
assign GPIO_1[1] = CLOCK_50;

assign triggerLowStart1 = triggerLowStart;
assign triggerLowStart2 = triggerLowStart;

always_comb

	// On regarde sur les LED 1 sonar à la fois. Les 2 premières LEDs indiquent des flags !
	casex (led_afficheur) // On ne devrait rentrer que dans le premier cas sur le minibot !
		3'b000 : LED[7:0] = {distance1[19:14], Flags1[1:0]}; // On montre les flags pour montrer l'overflow (Flags[0]) ou si on est à l'arrêt (Flags[1])
		//3'b001 : LED[7:0] = {distance1[19:14], Flags1[1:0]};
		
		3'b001 : LED[7:0] = {distance2[19:14], Flags2[1:0]};
		/*3'b010 : LED[7:0] = {distance3[19:14], Flags3[1:0]};
		3'b011 : LED[7:0] = {distance4[19:14], Flags4[1:0]};
		3'b100 : LED[7:0] = {distance5[19:14], Flags5[1:0]};*/
		default  LED[7:0] = sum[21:14];
	endcase

always_ff @ (posedge KEY[0])
		if (reset) led_afficheur <= 3'b111; 
		else if (led_afficheur == 3'b000 ) led_afficheur <= 3'b001;
		else if (led_afficheur == 3'b001 ) led_afficheur <= 3'b111;
		
		/*else if (led_afficheur == 3'b010 ) led_afficheur <= 3'b011;
		/*else if (led_afficheur == 3'b011 ) led_afficheur <= 3'b100;
		else if (led_afficheur == 3'b100 ) led_afficheur <= 3'b111;*/
		else led_afficheur <= 3'b000;
	
always_ff @ (posedge CLOCK_50)
	if (KEY[1] == 1'b0) reset <= 1'b1;
	else reset <= 1'b0;
	
endmodule


