module quad_dec(
input clk,
input reset,
input A,
input B,
output reg signed [31:0] count
);



reg[1:0] sync, AB; // synchronization registers
typedef enum logic [1:0] {S00, S01, S10, S11} statetype;
statetype state;



always @ (posedge clk) // two-stage input synchronizer
begin
  sync <= {A,B};
  AB <= sync;
end



always @(posedge clk, posedge reset) // always block to compute output
begin
if(reset) begin
state <= S00;
count <= 0;
end else
case(state)
S00: if(AB == 2'b01) begin
count <= count-1;
state <= S01;
end else if(AB == 2'b10) begin
count <= count+1;
state <= S10;
end
S01: if(AB == 2'b00) begin
count <= count+1;
state <= S00;
end else if(AB == 2'b11) begin
count <= count-1;
state <= S11;
end
S10: if(AB == 2'b00) begin
count <= count-1;
state <= S00;
end else if(AB == 2'b11) begin
count <= count+1;
state <= S11;
end
S11: if(AB == 2'b01) begin
count <= count+1;
state <= S01;
end else if(AB == 2'b10) begin
count <= count-1;
state <= S10;
end
endcase
end
endmodule