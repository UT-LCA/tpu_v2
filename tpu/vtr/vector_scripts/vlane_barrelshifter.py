from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class vlane_barrelshifter():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width, log2width):
        widthp1 = width + 1
        string = '''\
/****************************************************************************
          Shifter unit

Opcode Table:

sign_ext dir 
 0        0    |  ShiftLeft
 0        1    |  ShiftRightLogic
 1        1    |  ShiftRightArith
          
****************************************************************************/
module vlane_barrelshifter(clk, resetn,
            opB, sa, 
            op, 
            result);
parameter {WIDTH}=32;
parameter {LOG2WIDTH}=5;

//Shifts the first 2 bits in one cycle, the rest in the next cycle
parameter ({LOG2WIDTH}-2)={LOG2WIDTH}-2;

input clk;
input resetn;

input [{WIDTH}-1:0] opB;
input [{LOG2WIDTH}-1:0] sa;                             // Shift Amount
input [2-1:0] op;

output [{WIDTH}-1:0] result;


wire sign_ext;
wire shift_direction;
assign sign_ext=op[1];
assign shift_direction=op[0];

wire dum,dum_,dum2;
wire [{WIDTH}-1:0] partial_result_,partial_result;

`ifdef USE_INHOUSE_LOGIC
 local_shifter_WIDTHP1_2_ARITHMATIC local_shifter_inst1(
  .data({CBS}sign_ext&opB[{WIDTH}-1],opB{CBE}),
  .distance(sa&(32'hffffffff<<((({LOG2WIDTH}-2)>0) ? ({LOG2WIDTH}-2) : 0))),
  .direction(shift_direction),
  .result(dum,partial_result)
 );
 defparam
    local_shifter_inst1.LPM_WIDTH = {WIDTH}+1,
    local_shifter_inst1.LPM_WIDTHDIST = {LOG2WIDTH},
    local_shifter_inst1.LPM_SHIFTTYPE="ARITHMETIC";
`else
lpm_clshift shifter_inst1(
    .data({CBS}sign_ext&opB[{WIDTH}-1],opB{CBE}),
    .distance(sa&(32'hffffffff<<((({LOG2WIDTH}-2)>0) ? ({LOG2WIDTH}-2) : 0))),
    .direction(shift_direction),
    .result(dum,partial_result));
 defparam
    shifter_inst1.lpm_width = {WIDTH}+1,
    shifter_inst1.lpm_widthdist = {LOG2WIDTH},
    shifter_inst1.lpm_shifttype="ARITHMETIC";
`endif

register partial_reg
  ({CBS}dum,partial_result{CBE},clk,resetn,1'b1,{CBS}dum_,partial_result_{CBE});
    defparam partial_reg.{WIDTH}={WIDTH}+1;

wire [5-1:0] sa_2;
wire shift_direction_2;

register secondstage (sa, clk,resetn,1'b1,sa_2); 
  defparam secondstage.{WIDTH}=5;

register secondstagedir (shift_direction, clk,resetn,1'b1,shift_direction_2); 
  defparam secondstagedir.{WIDTH}=1;

`ifdef USE_INHOUSE_LOGIC
 local_shifter_WIDTHP1_2_ARITHMATIC local_shifter_inst2(
  .data({CBS}dum_,partial_result_{CBE}),
  .distance(sa_2[((({LOG2WIDTH}-2)>0) ? ({LOG2WIDTH}-2)-1 : 0):0]),
  .direction(shift_direction_2),
  .result({CBS}dum2,result{CBE})
 );
 defparam
    local_shifter_inst2.LPM_WIDTH = {WIDTH}+1,
    local_shifter_inst2.LPM_WIDTHDIST = (({LOG2WIDTH}-2)>0) ? ({LOG2WIDTH}-2) : 1,
    local_shifter_inst2.LPM_SHIFTTYPE ="ARITHMETIC";
`else
lpm_clshift_WIDTHP1_2_ARITHMATIC shifter_inst2(
    .data({CBS}dum_,partial_result_{CBE}),
    .distance(sa_2[((({LOG2WIDTH}-2)>0) ? ({LOG2WIDTH}-2)-1 : 0):0]),
    .direction(shift_direction_2),
    .result({CBS}dum2,resulti{CBE}));
 defparam 
    shifter_inst2.lpm_width = {WIDTH}+1,
    shifter_inst2.lpm_widthdist = (({LOG2WIDTH}-2)>0) ? ({LOG2WIDTH}-2) : 1,
    shifter_inst2.lpm_shifttype="ARITHMETIC";
`endif


endmodule

        '''
        return string.format(WIDTH=width, LOG2WIDTH=log2width, WIDTHP1 = widthp1, CBS="{", CBE="}") 

    def write (self, width,log2width):
        self.fp.write(self.make_str(width,log2width))

if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = vlane_barrelshifter(fp)
    uut1.write(32,5)
    fp.close()
