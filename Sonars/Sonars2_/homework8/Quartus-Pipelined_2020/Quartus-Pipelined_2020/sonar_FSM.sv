//==============================
// Diego Lannoye Sonar FSM
//===============================

// Pré :
// Trigger : signal issu de la commande du Trigger envoyé au sonar
// Echo    : signal issu de la commande du Echo envoyé par le sonar 
// clk     : CLOCK_50 

// Post :
// distance : mesure de la distance par le sonar
// Flags   : Flags[0] Overflow Flags[1] Interrupt 

//

// Commentaire :
// Ce module est la pour permettre le controle du module sonar.sv
// Overflow pour ine distance suppérieure à 1.638 m

// Dépendances :
// sonar.sv
// full_adder14.sv 
// full_adder5.sv
// full_adder8.sv
// full_adder_1_bit.sv
//

module sonar_FSM(input logic clk, reset,
					  output logic Trigger,
					  input logic Echo, 
					  input logic triggerLowStart,
					  output logic [19:0] distance, // Permet d'aller au-delà de 1.7 [m] d'encodage.
					  output logic [1:0] Flags);
					  

					  
	logic [8:0] PrimaryCounterSum;
	logic [8:0] PrimaryCounterRegister;
	logic c9_out; 
	logic PrimaryCounterReset;

	full_adder9 FA90 (.a(PrimaryCounterRegister[8:0]) , .b(9'b1),  .c_in(1'b0),  .c_out(c9_out), .sum(PrimaryCounterSum[8:0]));

	logic [13:0] SecondaryCounterSum;
	logic [13:0] SecondaryCounterRegister;
	logic c14_out; 
	
	logic pulse;
	
	logic SecondaryCounterReset;
	logic PrimaryCounterPulse;

	logic overflow; 

	full_adder14 FA140 (.a(SecondaryCounterRegister[13:0]) , .b(14'b1),  .c_in(1'b0),  .c_out(c14_out), .sum(SecondaryCounterSum[13:0]));

	// latch pour conserver l'ouput jusqu'à la prochaine mesure
	myLatch23 MyLatch23 (.in_value( {SecondaryCounterSum[10:0],PrimaryCounterSum[8:0]}), .pulse(SecondaryCounterReset), .out_value(distance[19:0]) );
	myLatch_1bit ML1 (.in_value( SecondaryCounterSum[11]), .pulse(pulse), .out_value(overflow) );

	always_ff @ (posedge clk )
		begin
		if (PrimaryCounterReset) begin PrimaryCounterRegister <= 9'b0; end
		else PrimaryCounterRegister <= PrimaryCounterSum;
		
		if (SecondaryCounterReset) SecondaryCounterRegister <= 14'b0;
		if (PrimaryCounterPulse) SecondaryCounterRegister <= SecondaryCounterSum;
		end	
		
	 
	 logic triggerHighEnd;
	 logic triggerLowEnd;
	 
	always_comb 
		begin
			if (c9_out)  PrimaryCounterPulse = 1'b1;
			else PrimaryCounterPulse = 1'b0; 
			
			if (PrimaryCounterRegister == 9'd500)  triggerHighEnd = 1'b1;
			else triggerHighEnd = 1'b0; 
		
			if (PrimaryCounterRegister == 9'd100)  triggerLowEnd = 1'b1;
			else triggerLowEnd = 1'b0;
		end


	/*logic [3:0] state, nextstate;

	parameter S0 = 3'b000; // stand by mode
	parameter TRIG_MODE = 3'b001; // Triggeur mode
	parameter INIT_MODE = 3'b010; // Initialisation Mode
	parameter COUNTING = 3'b011; // Counting mode
	parameter OVERFLOW = 3'b100; // overflow mode*/

	//{S0, S1, S2, S3, S4, S5} ! TRIG_MODE est nouveau. Ancien TRIG_MODE = INIT_MODE, etc.
	typedef enum logic [2:0] {STAND_BY,TRIG_LOW,INTER_TRIG,TRIG_MODE,INIT_MODE,COUNTING,OVERFLOW} statetype; //On définit les états du système
		statetype state, nextstate;


		
	always_ff @ (posedge clk)
		if (reset) state <= STAND_BY;
		else state <= nextstate;
		
	always_comb 
		case (state) 
			STAND_BY : if (triggerLowStart)   nextstate = TRIG_LOW;
				  else nextstate = STAND_BY;
			
			TRIG_LOW : if(triggerLowEnd)	nextstate = INTER_TRIG;
					else nextstate = TRIG_LOW;
					
			// Sert juste à reset un compteur
			INTER_TRIG : nextstate = TRIG_MODE;
				  
			TRIG_MODE : if (triggerHighEnd)  nextstate = INIT_MODE; 
				  else nextstate = TRIG_MODE;
				  
			INIT_MODE : 
				begin
					if(triggerLowStart)		nextstate = TRIG_LOW;
					else if(Echo) nextstate = COUNTING;
					else nextstate = INIT_MODE;
				end
			
			COUNTING : if (~Echo) nextstate = STAND_BY;
				  else if (SecondaryCounterSum[11]) nextstate = OVERFLOW;
				  else nextstate = COUNTING; 
				  
			OVERFLOW : if (triggerLowStart)  nextstate = TRIG_LOW;
				  else nextstate = OVERFLOW;
		
			default : nextstate = STAND_BY;
		endcase

	
	assign pulse = (SecondaryCounterSum[11]) | (state == INIT_MODE);
	
	assign PrimaryCounterReset = (state == STAND_BY) | (state == INTER_TRIG) | (state == INIT_MODE) | (state == OVERFLOW) | ((state == COUNTING) & PrimaryCounterPulse) | reset; 
	assign SecondaryCounterReset = ~(state == COUNTING) | reset;

	assign Flags[0] = (overflow == 1'b1);
	assign Flags[1] = (state == INIT_MODE); // interrupt
	assign Trigger = (state == TRIG_MODE);

endmodule


// Stockage de la valeur de counting lorsqu'on va repartir pour un autre trigger.
module myLatch23 ( input logic [19:0] in_value, input logic pulse, output logic [19:0] out_value );
	always_ff @ (posedge pulse) 
		out_value <= in_value; 
endmodule 

module myLatch_1bit ( input logic in_value, input logic pulse, output logic out_value );
	always_ff @ (posedge pulse) 
		out_value <= in_value; 
endmodule   
