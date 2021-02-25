module local_mult(
dataa,
datab,
clock,
clken,
aclr,
result
);

parameter LPM_WIDTHA = 32;
parameter LPM_WIDTHB = 32;
parameter LPM_WIDTHP = 64;
parameter LPM_REPRESENTATION = "SIGNED";

input [LPM_WIDTHA-1:0] dataa;
input [LPM_WIDTHB-1:0] datab;
input clock;
input clken;
input aclr;
output reg [LPM_WIDTHP-1:0] result;

wire signed [LPM_WIDTHA-1:0] signedinputA;
wire signed [LPM_WIDTHB-1:0] signedinputB;
wire signed [LPM_WIDTHP-1:0] signedoutputP;

wire [LPM_WIDTHA-1:0] unsignedinputA;
wire [LPM_WIDTHB-1:0] unsignedinputB;
wire [LPM_WIDTHP-1:0] unsignedinputP;

wire gated_clock;

assign signedinputA = dataa;
assign signedinputB = datab;
assign unsignedinputA = dataa;
assign unsignedinputB = datab;

assign signedoutputP = signedinputA * signedinputB;
assign unsignedoutputP = unsignedinputA * unsignedinputB;

assign gated_clock = clock & clken;

always @(posedge gated_clock)begin
    if(aclr)begin
       result <= 0;
    end
    else if(LPM_REPRESENTATION == "SIGNED")
       result <= signedoutputP;
    else
       result <= unsignedoutputP; 
end

endmodule
