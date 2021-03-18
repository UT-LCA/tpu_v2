module local_mult_33_33_66(
dataa,
datab,
clock,
clken,
aclr,
result
);

input [33-1:0] dataa;
input [33-1:0] datab;
input clock;
input clken;
input aclr;
output reg [66-1:0] result;

wire [33-1:0] unsignedinputA;
wire [33-1:0] unsignedinputB;
wire [66-1:0] unsignedoutputP;

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