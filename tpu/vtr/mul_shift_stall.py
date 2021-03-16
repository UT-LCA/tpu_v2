from local_mult import local_mult
from components import onecyclestall
import parser

class mul():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''\
/****************************************************************************
          MUL/DIV unit

Operation table

   op sign dir
4  1   0    x    |  MULTU
6  1   1    x    |  MULT
0  0   0    0    |  ShiftLeft
1  0   0    1    |  ShiftRightLogic
3  0   1    1    |  ShiftRightArith
****************************************************************************/
//`include "options.v"

module mul_{WIDTH} (clk, resetn, start, stalled, dst,
            opA, opB, sa,
            op,
            shift_result,
            hi, lo);

input clk;
input resetn;

input start;
output stalled;

input [4:0] dst;

input [{WIDTH}-1:0] opA;
input [{WIDTH}-1:0] opB;
input [5-1:0] sa;
input [2:0] op;

output [{WIDTH}-1:0] shift_result;
output [{WIDTH}-1:0] hi;
output [{WIDTH}-1:0] lo;

/********* Control Signals *********/
wire is_signed, dir, is_mul;
assign is_mul=op[2];      // selects between opB and the computed shift amount
assign is_signed=op[1];
assign dir=op[0];         // selects between 2^sa and 2^(32-sa) for right shift

/********* Circuit Body *********/
wire dum,dum2,dum3;
wire [{WIDTH}:0] opB_mux_out;
wire [5-1:0] left_sa;     // Amount of left shift required for both left/right
reg [{WIDTH}:0] decoded_sa;

assign opB_mux_out= (is_mul) ? {{is_signed&opB[{WIDTH}-1],opB}} : decoded_sa;

`ifndef USE_INHOUSE_LOGIC
    `define USE_INHOUSE_LOGIC
`endif

`ifdef USE_INHOUSE_LOGIC
wire [{MULT_WIDTHA}-1:0] mult_dataa;
wire mult_aclr;

assign mult_dataa = {{is_signed&opA[{WIDTH}-1],opA}};
assign mult_aclr = ~resetn;

local_mult_{MULT_WIDTHA}_{MULT_WIDTHB}_{MULT_WIDTHP} local_mult_component (
.dataa(mult_dataa),
.datab(opB_mux_out),
.clock(clk),
.clken(1'b1),
.aclr(mult_aclr),
.result({{dum2,dum,hi,lo}})
);

`else
 
lpm_mult  lpm_mult_component (
  .dataa ({{is_signed&opA[{WIDTH}-1],opA}}),
  .datab (opB_mux_out),
  .sum(),
  .clock(clk),
  .clken(),
  .aclr(~resetn),
  .result ({{dum2,dum,hi,lo}}));
defparam
  lpm_mult_component.lpm_widtha = {WIDTH}+1,
  lpm_mult_component.lpm_widthb = {WIDTH}+1,
  lpm_mult_component.lpm_widthp = 2*{WIDTH}+2,
  lpm_mult_component.lpm_widths = 1,
  lpm_mult_component.lpm_pipeline = 1,
  lpm_mult_component.lpm_type = "LPM_MULT",
  lpm_mult_component.lpm_representation = "SIGNED",
  lpm_mult_component.lpm_hint = "MAXIMIZE_SPEED=6";
`endif

assign shift_result= (dir && |sa) ? hi : lo;

assign {{dum3, left_sa}} = (dir) ? 32-sa : {{1'b0,sa}};

always@(left_sa or dir)
begin
  decoded_sa = 1 << left_sa;
end

// 1 cycle stall state machine
wire staller_request;
assign staller_request = (start&is_mul)|(start&(|dst)&~is_mul);
onecyclestall staller(staller_request,clk,resetn,stalled);

endmodule'''       

        return string.format(WIDTH=width, MULT_WIDTHA=width+1, MULT_WIDTHB=width+1, MULT_WIDTHP=2*(width+1))

    def write (self, width):
        self.fp.write(self.make_str(width))

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = mul(fp)
    uut.write(32)
    fp.close()
    fp = open(parser.parse(), "a")
    fp.write("\r\r")
    mult = local_mult(fp)
    mult.write(32+1, 32+1, 2*(32+1))
    fp.write("\r\r")
    stall = onecyclestall(fp)
    stall.write()
    fp.close()