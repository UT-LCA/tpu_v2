module local_mult_64_64_128(
dataa,
datab,
clock,
clken,
aclr,
result
);

parameter LPM_WIDTHA = 64;
parameter LPM_WIDTHB = 64;
parameter LPM_WIDTHP = 128;

input [LPM_WIDTHA-1:0] dataa;
input [LPM_WIDTHB-1:0] datab;
input clock;
input clken;
input aclr;
output reg [LPM_WIDTHP-1:0] result;

wire [LPM_WIDTHA-1:0] unsignedinputA;
wire [LPM_WIDTHB-1:0] unsignedinputB;
wire [LPM_WIDTHP-1:0] unsignedoutputP;

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