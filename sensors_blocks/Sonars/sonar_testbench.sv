//module test_sonar ();
module sonar_testbench ();

logic [13:0] distance;
logic clock, reset; 

sonar Sonar01 ( clock, reset, distance);

always #2 clock = ~clock;

initial 
begin
clock = 1'b0;
reset = 1'b1;
#8
reset = 1'b0;
#2900
reset = 1'b1;
#8
reset = 1'b0;
#5900
reset = 1'b1;

end
endmodule  
