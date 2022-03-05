
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

	
//=======================================================
//  SPI
//=======================================================

	
	
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
		casex (DataFromPI[31:0])
		32'd1 : begin DataToPI[13:0] = distance1; DataToPI[15:14] = Flags1[1:0]; end
		32'd2 : begin DataToPI[13:0] = distance2; DataToPI[15:14] = Flags2[1:0]; end
		32'd3 : begin DataToPI[13:0] = distance3; DataToPI[15:14] = Flags3[1:0]; end
		32'd4 : begin DataToPI[13:0] = distance4; DataToPI[15:14] = Flags4[1:0]; end
		32'd5 : begin DataToPI[13:0] = distance5; DataToPI[15:14] = Flags5[1:0]; end
		default begin DataToPI[15:0] = 16'bx; end
		endcase 
		
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
 

logic [13:0] distance1, distance2, distance3, distance4, distance5;
logic reset;
logic Echo1, Echo2 ,Echo3 ,Echo4 ,Echo5;
logic Trigger1 ,Trigger2 ,Trigger3 ,Trigger4 ,Trigger5;
logic [1:0] Flags1, Flags2, Flags3, Flags4, Flags5; 

logic [2:0] led_afficheur;

initial
	begin
	reset = 1'b1;
	DataToPI = 32'd0;

	end

// mise en place des FMS pour chaque sonars
sonar_FSM sonar_FSM_01( CLOCK_50, reset, Trigger1, Echo1, distance1[13:0] , Flags1 [1:0] );
sonar_FSM sonar_FSM_02( CLOCK_50, reset, Trigger2, Echo2, distance2[13:0] , Flags2 [1:0] );
sonar_FSM sonar_FSM_03( CLOCK_50, reset, Trigger3, Echo3, distance3[13:0] , Flags3 [1:0] );
sonar_FSM sonar_FSM_04( CLOCK_50, reset, Trigger4, Echo4, distance4[13:0] , Flags4 [1:0] );
sonar_FSM sonar_FSM_05( CLOCK_50, reset, Trigger5, Echo5, distance5[13:0] , Flags5 [1:0] );

// attribution des pins

assign Echo1 = GPIO_1_IN[0];
assign Echo2 = GPIO_1_IN[1];

assign Trigger1 = GPIO_1[2];
assign Trigger2 = GPIO_1[4];

assign Echo3 = GPIO_1[6];
assign Trigger3 = GPIO_1[8];

assign Echo4 = GPIO_1[10];
assign Trigger4 = GPIO_1[12];

assign Echo5 = GPIO_1[14];
assign Trigger5 = GPIO_1[16];




always_comb
	casex (led_afficheur)
	3'b000 : LED[7:0] = distance1[13:6];
	3'b001 : LED[7:0] = distance2[13:6];
	3'b010 : LED[7:0] = distance3[13:6];
	3'b011 : LED[7:0] = distance4[13:6];
	3'b100 : LED[7:0] = distance5[13:6];
	default  LED[7:0] = 8'b0;
	endcase

always_ff @ (posedge KEY[0])
		if (led_afficheur == 3'b000 ) led_afficheur = 3'b001;
		else if (led_afficheur == 3'b001 ) led_afficheur <= 3'b010;
		else if (led_afficheur == 3'b010 ) led_afficheur <= 3'b011;
		else if (led_afficheur == 3'b011 ) led_afficheur <= 3'b100;
		else if (led_afficheur == 3'b100 ) led_afficheur <= 3'b000;
		else led_afficheur <= 3'b000;
	

	
endmodule


