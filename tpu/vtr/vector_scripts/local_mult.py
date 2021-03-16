import parser

class local_mult():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, lpm_widtha, lpm_widthb, lpm_widthp):
        str = '''\
module local_mult_{LPM_WIDTHA}_{LPM_WIDTHB}_{LPM_WIDTHP}(
dataa,
datab,
clock,
clken,
aclr,
result
);

input [{LPM_WIDTHA}-1:0] dataa;
input [{LPM_WIDTHB}-1:0] datab;
input clock;
input clken;
input aclr;
output reg [{LPM_WIDTHP}-1:0] result;

wire [{LPM_WIDTHA}-1:0] unsignedinputA;
wire [{LPM_WIDTHB}-1:0] unsignedinputB;
wire [{LPM_WIDTHP}-1:0] unsignedoutputP;

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
endmodule'''

        return str.format(LPM_WIDTHA=lpm_widtha, LPM_WIDTHB=lpm_widthb, LPM_WIDTHP=lpm_widthp)

    def write(self, lpm_widtha, lpm_widthb, lpm_widthp):
        self.fp.write(self.make_str(lpm_widtha, lpm_widthb, lpm_widthp))


if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    mult1 = local_mult(fp)
    mult1.write(64, 64, 128)
    fp.close()