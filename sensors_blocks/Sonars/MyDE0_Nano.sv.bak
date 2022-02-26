
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

//=======================================================
//  MyARM
//=======================================================

	logic clk, reset, MemWrite;
	logic [31:0] PC, Instr, DataAdr, WriteData, ReadData, ReadData_dmem;
	logic  cs_dmem, cs_led, cs_spi_wr, cs_spi_rd;
	logic [7:0] led_reg;
	
	assign clk = CLOCK_50;
	assign reset = GPIO_0_PI[1];
  
	// Instantiate processor and memories
	arm arm(clk, reset, PC, Instr, MemWrite, DataAdr, WriteData, ReadData);
	imem imem(PC, Instr);
	dmem dmem(clk, MemWrite, cs_dmem, DataAdr, WriteData, ReadData_dmem);
	
	// Chip Select logic
	assign cs_dmem   = ~DataAdr[11] & ~DataAdr[10] & ~DataAdr[9];	// 000x - xxxx xxxx
	assign cs_spi_wr = ~DataAdr[11] & ~DataAdr[10] &  DataAdr[9];	// 001x - xxxx xxxx = 0x200
	assign cs_spi_rd = ~DataAdr[11] &  DataAdr[10] & ~DataAdr[9];	// 010x - xxxx xxxx = 0x400
	assign cs_led    =  DataAdr[11] & ~DataAdr[10] & ~DataAdr[9];	// 100x - xxxx xxxx = 0x800
	
	// Read LED or SPI
	always_comb
    	case(DataAdr[11:9]) 
  	    	3'b010:   ReadData = DataFromPI; 				// 0x400
  	    	3'b100:   ReadData = {24'h000000, led_reg};	// 0x800
  	    	default: ReadData = ReadData_dmem;
      endcase
	
	// SPI output Register
	 always_ff @(posedge clk)
		if (MemWrite & cs_spi_wr) DataToPI <= WriteData;
	
	// LED logic	
	assign LED = led_reg;	
	always_ff @(posedge clk)
    	if (MemWrite & cs_led) led_reg <= WriteData[7:0];

	// Testbench
	assign GPIO_1[33]    = MemWrite;
	assign GPIO_1[15:0]  = WriteData[15:0];
	assign GPIO_1[31:16] = PC[15:0];
	assign GPIO_2[12:0]  = DataAdr[12:0];
	
	
	
endmodule
