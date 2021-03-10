module local_mult_64_64_128(
dataa,
datab,
clock,
clken,
aclr,
result
);

input [64-1:0] dataa;
input [64-1:0] datab;
input clock;
input clken;
input aclr;
output reg [128-1:0] result;

wire [64-1:0] unsignedinputA;
wire [64-1:0] unsignedinputB;
wire [128-1:0] unsignedoutputP;

wire gated_clock;

assign unsignedinputA = dataa;
assign unsignedinputB = datab;

assign unsignedoutputP = unsignedinputA * unsignedinputB;

assign gated_clock = clock & clken;

always @(posedge gated_clock)begin
    if(aclr)begin
       result <= 0;
    end
    else
       result <= unsignedoutputP; 
end
endmodule