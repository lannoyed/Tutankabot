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
					  input logic Trigger, Echo, 
					  output logic [13:0] distance, 
					  output logic [1:0] Flags);

logic [1:0] state, nextstate;

parameter S0 = 2'b00; // stand by mode
parameter S1 = 2'b01; // computation mode
parameter S2 = 2'b10; // overflow mode
parameter S3 = 2'b11; // standby during Trigger

logic reset1;
logic overflow;

sonar Sonar01 ( clk , reset1, distance, overflow);

always_ff @ (posedge clk)
	if (reset) state <= S0;
	else state <= nextstate;
	
always_comb 
	case (state) 
		S0 : if (Trigger)   nextstate = S3;
			  else nextstate = S0;
			  
		S1 : if (overflow)  nextstate = S2;
			  else if (Echo) nextstate = S0; 
			  else nextstate = S1;
			  
		S2 : if (Trigger) nextstate = S3;
			  else nextstate = S2;
		
		S3 : if (~Trigger) nextstate = S1;
			  else nextstate = S3; 
	
		default : nextstate = S0;
	endcase


	
assign Flags[0] =  (state == S2);					 // overflow
assign reset1   = ~(state == S1);					 // reset du compteur
assign Flags[1] = (state == S2) | (state == S0); // interupt

endmodule
