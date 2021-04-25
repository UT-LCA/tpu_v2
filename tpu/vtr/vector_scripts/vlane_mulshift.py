from local_mult import local_mult
import math

from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class vlane_mulshift():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width, log2width):
        widtha = width + 1
        widthb = width + 1
        widthp = 2*(width + 1) 
        string = '''\
/****************************************************************************
          MUL unit

Operation table

     half sat  op unsign dir
2      0   0   0   0    0    |  Zero (This value is used to reset multiplier)
14     1   0   1   1    0    |  LMULU Lower MULtiply Unsigned
15     1   0   1   1    1    |  UMULU Upper MULtiply Unsigned
16     1   0   1   0    0    |  LMUL Lower MULtiply signed
17     1   0   1   0    1    |  UMUL Upper MULtiply signed
4      0   0   1   1    0    |  MULLOU
4      0   0   1   1    1    |  MULHIU
6      0   0   1   0    0    |  MULLO
6      0   0   1   0    1    |  MULHI
0      0   0   0   1    0    |  ShiftLeft
8      0   1   0   1    0    |  ShiftLeftSatU
10     0   1   0   0    0    |  ShiftLeftSat
1      0   0   0   1    1    |  ShiftRightLogic
3      0   0   0   0    1    |  ShiftRightArith
****************************************************************************/
module vlane_mulshift_{WIDTH}_{LOG2WIDTH}(clk, resetn,
            opA, opB, sa,
            op,
            en,
            result
            );
parameter WIDTH={WIDTH};
parameter LOG2WIDTH={LOG2WIDTH};

input clk;
input resetn;

input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
input [LOG2WIDTH-1:0] sa;
input [4:0] op;
input [3:1] en;  //Enable for each pipestage

output [WIDTH-1:0] result;

wire [3:0] temp;
/********* Control Signals *********/
wire is_signed, dir, is_mul, saturate, half;
assign is_mul=op[2];      // selects between opB and the computed shift amount
assign is_signed=~op[1];
assign dir=op[0];         // selects between 2^sa and 2^(32-sa) for right shift
assign saturate=op[3];
assign half=op[4];
assign temp = en;

/********* Circuit Body *********/
wire dum,dum2,dum3;
wire [WIDTH:0] opB_mux_out;
wire [WIDTH-1:0] opA_mux_out;
wire [WIDTH-1:0] opB_mul;
wire [5-1:0] left_sa;     // Amount of left shift required for both left/right
reg [WIDTH:0] decoded_sa;
wire [WIDTH-1:0] hi;
wire [WIDTH-1:0] lo;

assign opA_mux_out = (~half) ? ( opA ) : (WIDTH<2) ? 0 : (dir) ?
              {{{{WIDTH/2{{is_signed&is_mul&opA[WIDTH-1]}}}},opA[WIDTH-1:WIDTH/2]}} :
              {{{{WIDTH/2{{is_signed&is_mul&opA[WIDTH/2-1]}}}},opA[WIDTH/2-1:0]}};

assign opB_mul = (~half) ? opB : (WIDTH<2) ? 0 : (dir) ? 
              {{{{WIDTH/2{{is_signed&is_mul&opB[WIDTH-1]}}}},opB[WIDTH-1:WIDTH/2]}} :
              {{{{WIDTH/2{{is_signed&is_mul&opB[WIDTH/2-1]}}}},opB[WIDTH/2-1:0]}};

assign opB_mux_out=(is_mul) ? {{is_signed&opB_mul[WIDTH-1],opB_mul}} : decoded_sa;

reg zeroout;

`ifndef USE_INHOUSE_LOGIC
  `define USE_INHOUSE_LOGIC
`endif

always@(posedge clk)
  if (en[1])
    zeroout<=(op[3:0]==0);

`ifdef USE_INHOUSE_LOGIC
wire [(WIDTH+1)-1:0] dataa;
wire aclr;
wire [{WIDTHP}-1:0] local_mult_component_result;

assign dataa = {{is_signed&opA_mux_out[WIDTH-1],opA_mux_out}};
assign aclr = ~resetn;
assign {{dum2,dum,hi,lo}} = local_mult_component_result;

local_mult_{WIDTHA}_{WIDTHB}_{WIDTHP} local_mult_component (
.dataa(dataa),
.datab(opB_mux_out),
.clock(clk),
.clken(en[1]),
.aclr(aclr),
.result(local_mult_component_result)
);
`else 
lpm_mult  lpm_mult_component (
  .dataa ({{is_signed&opA_mux_out[WIDTH-1],opA_mux_out}}),
  .datab (opB_mux_out),
  .sum(),
  .clock(clk),
  .clken(en[1]),
  .aclr(~resetn),
  .result ({{dum2,dum,hi,lo}}));
defparam
  lpm_mult_component.lpm_widtha = WIDTH+1,
  lpm_mult_component.lpm_widthb = WIDTH+1,
  lpm_mult_component.lpm_widthp = 2*WIDTH+2,
  lpm_mult_component.lpm_widths = 1,
  lpm_mult_component.lpm_pipeline = 1,
  lpm_mult_component.lpm_type = "LPM_MULT",
  lpm_mult_component.lpm_representation = "SIGNED",
  lpm_mult_component.lpm_hint = "MAXIMIZE_SPEED=6";
`endif
// if A is positive/negative make it maximum/minimum positive/negative
wire [WIDTH-1:0] signedsaturate=
                    (opA_mux_out[WIDTH-1]) ? {{1'b1,{{WIDTH-1{{1'b0}}}}}} : 
                                             {{1'b0,{{WIDTH-1{{1'b1}}}}}};

reg [WIDTH-1:0] saturatedval_s2;
reg saturate_s2;

//Capture saturated value and saturate signal for next stage
always@(posedge clk)
  if (!resetn)
  begin
    saturatedval_s2<=0;
    saturate_s2<=0;
  end
  else if (en[1])
  begin
    saturatedval_s2<=((~is_signed) ? {{WIDTH{{1'b1}}}} : signedsaturate);
    saturate_s2<=saturate;
  end

reg sel_hi;

always@(posedge clk)
  if (!resetn)
    sel_hi<=0;
  else if (en[1])
    sel_hi<=(is_mul && dir || ~is_mul && dir && |sa);

assign result =(zeroout) ? 0 : 
                (saturate_s2 && |hi ) ? saturatedval_s2 : 
                (sel_hi) ? hi : lo;


assign {{dum3, left_sa}}= (dir) ? WIDTH-sa : {{1'b0,sa}};

//Decoder - computes 2^left_sa
always@*
begin
    decoded_sa=1<<left_sa;
end
endmodule'''
        
        fp = open("verilog/local_mult.v",'a')
        uut = local_mult(fp)
        uut.write(widtha,widthb,widthp)

        return string.format(WIDTH=width, LOG2WIDTH=log2width, WIDTHA=widtha, WIDTHB=widthb, WIDTHP=widthp)

    def write (self, width, log2width):
        self.fp.write(self.make_str(width, log2width))


if __name__ == '__main__':
    fp = open(args[0], "w")
    uut = vlane_mulshift(fp)
    uut.write(32, 5)
    fp.close()
    # fp = open(parser.parse(), "a")
    # mult = local_mult(fp)
    # append_str = "\r\r" + mult.make_str(32+1, 32+1, 2*(32+1))
    # fp.write(append_str)
    # fp.close()
