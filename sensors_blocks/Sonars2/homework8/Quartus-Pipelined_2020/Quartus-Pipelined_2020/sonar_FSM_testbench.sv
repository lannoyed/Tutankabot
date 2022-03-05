
module sonar_FSM_testbench ();

logic [13:0] distance;
logic clock, reset; 
logic [1:0] Flags;
logic Trigger, Echo;

sonar_FSM sonar_FSM_01(clock, reset,Trigger, Echo, distance[13:0],Flags[1:0]);

always #1 clock = ~clock;

initial
begin
clock = 1'b0;
reset = 1'b1;
Trigger = 1'b0;
Echo = 1'b0;
#8
reset = 1'b0;
#2900

Trigger=1'b1;
#29
Trigger=1'b0;

#29000

Echo = 1'b1;
#29
Echo = 1'b0;

#2900

Trigger = 1'b1;
#29
Trigger = 1'b0;

#59000
Echo = 1'b1;
#29
Echo = 1'b0;
reset = 1'b1;
end
endmodule  
