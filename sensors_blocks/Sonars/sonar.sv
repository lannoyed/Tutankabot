//==============================
// Diego Lannoye Sonar
//===============================


// Ce module permet via le reset de calculer la distance en 1/10 mm observée par le sonnar
// Cela se fait par une division paf 29 du nombre de clock cycle
//
// Pré :
// Clock 50 MHz
// reset -> mis à 0 quand le signal est envoyé
//       -> mis à 1 quand le signal est reçus 
// 
// Post :
// la distance calculée 
//
// Dépendances :
//
// full_adder14.sv 
// full_adder5.sv
// full_adder8.sv
// full_adder_1_bit.sv
//
   

module sonar (
input  logic clk,
input  logic reset,
output logic [13:0] distance,
output  logic overflow);


logic [4:0]  DividerCompteur;
logic [13:0] DistanceCompteur;
logic  		 DividerCompteurPulse;

logic [4:0]  DividerCompteurRegister;
logic [13:0] DistanceCompteurRegister;

logic c5_out;


logic DividerCompteurReset;

//compteur 5 bits
full_adder5 FA50 (.a(DividerCompteurRegister[4:0]) , .b(5'b1),  .c_in(1'b0),  .c_out(c5_out), .sum(DividerCompteur[4:0]));

//compteur 14 bits
full_adder14 FA140 (.a(DistanceCompteurRegister[13:0]) , .b(14'b1),  .c_in(1'b0),  .c_out(overflow), .sum( DistanceCompteur[13:0]));

// latch pour conserver l'ouput jusqu'à la prochaine mesure
myLatch ML10 (.in_value( DistanceCompteur[13:0]), .tic(reset), .out_value(distance[13:0]) );


always_ff @ (posedge clk )
	begin
	//if (DividerCompteurPulse) begin DividerCompteurPulse <= 1'b0; end
	if (DividerCompteurReset) begin DividerCompteurRegister <= 5'b0; end
	else DividerCompteurRegister <= DividerCompteur;
	
	if (reset) DistanceCompteurRegister <= 14'b0;
	if (DividerCompteurPulse) DistanceCompteurRegister <= DistanceCompteur;
	end	
	
 
always_comb 
	begin
	if (DividerCompteurRegister == 5'd28)  DividerCompteurPulse = 1'b1;
	else DividerCompteurPulse = 1'b0; 
	DividerCompteurReset = reset || DividerCompteurPulse;
	end

endmodule 


module myLatch ( input logic [13:0] in_value, input logic tic, output logic [13:0] out_value );
always_ff @ (posedge tic) 
	out_value <= in_value; 
endmodule  