module local_mult(
dataa,
datab,
clock,
clken,
aclr,
result
);

parameter LPA_WIDTHA = 32;
parameter LPA_WIDTHB = 32;
parameter LPA_WIDTHP = 64;
parameter LPA_REPRESENTATION = "SIGNED";

input [LPA_WIDTHA-1:0] dataa;
input [LPA_WIDTHB-1:0] datab;
input clock;
input clken;
input aclr;
output reg [LPA_WIDTHP-1:0] result;

wire signed [LPA_WIDTHA-1:0] signedinputA;
wire signed [LPA_WIDTHB-1:0] signedinputB;
wire signed [LPA_WIDTHP-1:0] signedoutputP;

wire unsigned [LPA_WIDTHA-1:0] unsignedinputA;
wire unsigned [LPA_WIDTHB-1:0] unsignedinputB;
wire unsigned [LPA_WIDTHP-1:0] unsignedinputP;

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
    else if(LPA_REPRESENTATION == "SIGNED")
       result <= signedoutputP;
    else
       result <= unsignedoutputP; 
end

endmodule
