//==============================
// Diego Lannoye & Nicolas Isenguerre Sonar FSM
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
// sonar_FSM.sv
// full_adder14.sv 
// full_adder5.sv
// full_adder8.sv
// full_adder_1_bit.sv
//

module sonar_FSM(input logic 		clk, reset,
					  output logic 	Trigger,
					  input logic		Echo, 
					  input logic 		triggerLowStart,
					  output logic 	[19:0] distance, // 20 bits permet d'aller au-delà de 1.7 [m] d'encodage.
					  output logic 	[1:0] Flags);

					  
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

	
	
	// LE COUNTER PRINCIPAL
	full_adder20 	FA20(.a(counterRegister[19:0]), .b(20'd1), .c_in(1'b0), .sum(counterSum[19:0]), .c_out(c_out_adder));
	
	// LE LATCH POUR CONSERVER CETTE VALEUR
	myLatch20 		latch_20_bits(.in_value(counterSum[19:0]), .pulse(updateDistance), .out_value(distance[19:0]));
	
	// LATCH POUR LE CARRY_OUT
	myLatch_1bit 	latch_carry_out(.in_value(c_out_adder), .pulse(updateFlagOverflow), .reset(reset), .out_value(overflow));
	
	// LATCH POUR LE STATE INIT MODE
	myLatch_1bit	forInitMode(.in_value(stateIsInitMode), .pulse(updateBlockedInInitMode), .reset(reset), .out_value(blockedInInitMode));
	
	always_ff @ (posedge clk)
		begin
			if(counterReset) counterRegister <= 20'b0;
			else counterRegister <= counterSum;
		end	
		
	 
	 logic triggerHighEnd;
	 logic triggerLowEnd;
	 
	always_comb 
		begin
			if (counterRegister == 20'd600)  triggerHighEnd = 1'b1;
			else triggerHighEnd = 1'b0; 
		
			if (counterRegister == 20'd150)  triggerLowEnd = 1'b1;
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

	// state == INIT_MODE est là pour dire qu'on le remet à 0.
	assign updateFlagOverflow = (c_out_adder) | (state == INIT_MODE);
	
	assign counterReset = (state == STAND_BY) | (state == INTER_TRIG) | (state == INIT_MODE) | (state == OVERFLOW) | reset | triggerLowStart; 


	assign Flags[0] = (overflow == 1'b1);
	assign Flags[1] = (blockedInInitMode == 1'b1); // is blocked in init_mode.
	assign Trigger = (state == TRIG_MODE);
	
	assign updateBlockedInInitMode = ((state == INIT_MODE) | (state == COUNTING)) & clk;
	assign stateIsInitMode = (state == INIT_MODE);

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
