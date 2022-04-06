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

module sonar_FSM(input logic 		clk, reset,
					  output logic 	Trigger,
					  input logic		Echo, 
					  input logic 		triggerLowStart,
					  output logic 	[19:0] distance, // Permet d'aller au-delà de 1.7 [m] d'encodage.
					  output logic 	[1:0] Flags);
					  //output logic 	state_SB, state_TL, state_IT, state_TM, state_IM, state_CO, state_OV);
					  

						  
	/*logic [8:0] PrimaryCounterSum;
	logic [8:0] PrimaryCounterRegister;
	logic c9_out; 
	logic PrimaryCounterReset;

	full_adder9 FA90 (.a(PrimaryCounterRegister[8:0]) , .b(9'd1),  .c_in(1'b0),  .c_out(c9_out), .sum(PrimaryCounterSum[8:0]));

	logic [13:0] SecondaryCounterSum;
	logic [13:0] SecondaryCounterRegister;
	logic c14_out; */
	
	logic 			updateFlagOverflow;
	
	logic 			counterReset;
	logic [19:0] 	counterRegister;
	logic [19:0] 	counterSum;
	logic 			c_out_adder;
	
	logic 			updateDistance;
	
	logic overflow; 
	logic blockedInInitMode;
	logic updateBlockedInInitMode;
	logic stateIsInitMode;

	
	//logic SecondaryCounterReset;
	//logic PrimaryCounterPulse;

	//full_adder14 FA140 (.a(SecondaryCounterRegister[13:0]) , .b(14'd1),  .c_in(c9_out),  .c_out(c14_out), .sum(SecondaryCounterSum[13:0]));

	// latch pour conserver l'ouput jusqu'à la prochaine mesure
	//myLatch20 MyLatch20 (.in_value( {SecondaryCounterSum[10:0],PrimaryCounterSum[8:0]}), .pulse(SecondaryCounterReset), .out_value(distance[19:0]) );
	//myLatch_1bit ML1 (.in_value( SecondaryCounterSum[11]), .pulse(updateFlagOverflow), .out_value(overflow) );
	
	
	full_adder20 	FA20(.a(counterRegister[19:0]), .b(20'd1), .c_in(1'b0), .c_out(c_out_adder), .sum(counterSum[19:0]));
	myLatch20 		latch_20_bits(.in_value(counterSum[19:0]), .pulse(updateDistance), .out_value(distance[19:0]));
	myLatch_1bit 	latch_carry_out(.in_value(c_out_adder), .pulse(updateFlagOverflow), .reset(reset), .out_value(overflow));
	
	myLatch_1bit	forInitMode(.in_value(stateIsInitMode), .pulse(updateBlockedInInitMode), .reset(reset), .out_value(blockedInInitMode));
	
	always_ff @ (posedge clk)
		begin
			/*if (PrimaryCounterReset) begin PrimaryCounterRegister <= 9'b0; end
			else PrimaryCounterRegister <= PrimaryCounterSum;
			
			if (SecondaryCounterReset) SecondaryCounterRegister <= 14'b0;
			//if (PrimaryCounterPulse) SecondaryCounterRegister <= SecondaryCounterSum;
			SecondaryCounterRegister <= SecondaryCounterSum;*/
			
			if(counterReset) counterRegister <= 20'b0;
			else counterRegister <= counterSum;
		end	
		
	 
	 logic triggerHighEnd;
	 logic triggerLowEnd;
	 
	always_comb 
		begin
			if (counterRegister == 20'd500)  triggerHighEnd = 1'b1;
			else triggerHighEnd = 1'b0; 
		
			if (counterRegister == 20'd100)  triggerLowEnd = 1'b1;
			else triggerLowEnd = 1'b0;
		end


	typedef enum logic [2:0] {STAND_BY,TRIG_LOW,INTER_TRIG,TRIG_MODE,INIT_MODE,COUNTING,OVERFLOW} statetype; //On définit les états du système
		statetype state, nextstate;


	always_ff @ (posedge clk)
		if (reset) state <= STAND_BY;
		else state <= nextstate;
		
	always_comb 
		case (state) 
			STAND_BY : if (triggerLowStart) begin   nextstate = TRIG_LOW; updateDistance = 1'b0; end
				  else begin nextstate = STAND_BY; updateDistance = 1'b0; end
				  
			
			TRIG_LOW : if(triggerLowEnd) begin	nextstate = INTER_TRIG; updateDistance = 1'b0; end
					else begin nextstate = TRIG_LOW; updateDistance = 1'b0; end
					
			// Sert juste à reset un compteur
			INTER_TRIG : begin nextstate = TRIG_MODE; updateDistance = 1'b0; end
				  
			TRIG_MODE : if (triggerHighEnd) begin  nextstate = INIT_MODE; updateDistance = 1'b0; end
				  else begin nextstate = TRIG_MODE; updateDistance = 1'b0; end
				  
			INIT_MODE : 
				begin
					updateDistance = 1'b0;
					if(triggerLowStart)		nextstate = TRIG_LOW;
					else if(Echo) nextstate = COUNTING;
					else nextstate = INIT_MODE;
				end
			
			COUNTING : if (~Echo) begin	nextstate = STAND_BY; updateDistance = 1'b1; end
				  else if (c_out_adder) begin nextstate = OVERFLOW; updateDistance = 1'b1; end
				  else begin nextstate = COUNTING; updateDistance = 1'b0; end
				  
			OVERFLOW : if (triggerLowStart) begin  nextstate = TRIG_LOW; updateDistance = 1'b0; end
				  else begin nextstate = OVERFLOW; updateDistance = 1'b0; end
		
			default : begin nextstate = STAND_BY; updateDistance = 1'b0; end
		endcase

	assign updateFlagOverflow = (c_out_adder) | (state == INIT_MODE);
	
	assign counterReset = (state == STAND_BY) | (state == INTER_TRIG) | (state == INIT_MODE) | (state == OVERFLOW) | reset; 
	//assign SecondaryCounterReset = ~(state == COUNTING) | reset;

	assign Flags[0] = (overflow == 1'b1);
	assign Flags[1] = (blockedInInitMode == 1'b1); // is blocked in init_mode.
	assign Trigger = (state == TRIG_MODE);
	
	assign updateBlockedInInitMode = ((state == INIT_MODE) | (state == COUNTING)) & clk;
	assign stateIsInitMode = (state == INIT_MODE);
	
	/*assign state_SB = (state == STAND_BY);
	assign state_TL = (state == TRIG_LOW);
	assign state_IT = (state == INTER_TRIG);
	assign state_TM = (state == TRIG_MODE);
	assign state_IM = (state == INIT_MODE);
	assign state_CO = (state == COUNTING);
	assign state_OV = (state == OVERFLOW);*/

endmodule


// Stockage de la valeur de counting lorsqu'on va repartir pour un autre trigger.
module myLatch20 ( input logic [19:0] in_value, input logic pulse, output logic [19:0] out_value);
	always_ff @ (posedge pulse) 
		out_value <= in_value; 
endmodule 

module myLatch_1bit ( input logic in_value, input logic pulse, input logic reset, output logic out_value);		
	always_ff @ (posedge reset, posedge pulse) 
		if(reset) out_value <= 1'b0;
		else out_value <= in_value; 
endmodule   
