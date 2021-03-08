from local_add_sub import local_add_sub
import parser

class addersub_slt():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''\
/****************************************************************************
          AddSub unit
- Should perform ADD, ADDU, SUBU, SUB, SLT, SLTU

  is_slt signext addsub
    op[2] op[1] op[0]  |  Operation
0     0     0     0         SUBU
2     0     1     0         SUB
1     0     0     1         ADDU
3     0     1     1         ADD
4     1     0     0         SLTU
6     1     1     0         SLT

****************************************************************************/
//`include "options.v"

module addersub_{WIDTH} (
            opA, opB,
            op, 
            result,
            result_slt );

parameter WIDTH={WIDTH};


input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
//input carry_in;
input [3-1:0] op;

output [WIDTH-1:0] result;
output result_slt;

wire carry_out;
wire [WIDTH:0] sum;

// Mux between sum, and slt
wire is_slt;
wire signext;
wire addsub;

assign is_slt=op[2];
assign signext=op[1];
assign addsub=op[0];

assign result=sum[WIDTH-1:0];
//assign result_slt[WIDTH-1:1]={{31{{1'b0}}}};
//assign result_slt[0]=sum[WIDTH];
assign result_slt=sum[WIDTH];

`ifndef USE_INHOUSE_LOGIC
    `define USE_INHOUSE_LOGIC
`endif

`ifdef USE_INHOUSE_LOGIC
wire [(WIDTH+1)-1:0] dataa;
wire [(WIDTH+1)-1:0] datab;
wire cin;

assign dataa = {{signext&opA[WIDTH-1],opA}};
assign datab = {{signext&opB[WIDTH-1],opB}};
assign cin = ~addsub;

  local_add_sub_{ADD_SUB_WIDTH}_{PIPELINE}_{REPRESENTATION} local_adder_inst(
      .dataa(dataa),
      .datab(datab),
      .cin(cin),
      .add_sub(addsub),
      .result(sum)
  );
//  defparam
//      local_adder_inst.WIDTH = WIDTH+1,
//      local_adder_inst.PIPELINE = 0,
//      local_adder_inst.REPRESENTATION = "SIGNED";
`else
lpm_add_sub adder_inst(
    .dataa({{signext&opA[WIDTH-1],opA}}),
    .datab({{signext&opB[WIDTH-1],opB}}),
    .cin(~addsub),
    .add_sub(addsub),
    .result(sum)
        // synopsys translate_off
        ,
        .cout (),
        .clken (),
        .clock (),
        .overflow (),
        .aclr ()
        // synopsys translate_on
    );
defparam 
    adder_inst.lpm_width=WIDTH+1,
    adder_inst.lpm_representation="SIGNED";
`endif

assign carry_out=sum[WIDTH];
endmodule'''       

        return string.format(WIDTH=width, ADD_SUB_WIDTH=width+1, PIPELINE=0, REPRESENTATION="SIGNED")

    def write (self, width):
        self.fp.write(self.make_str(width))

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = addersub_slt(fp)
    uut.write(32)
    fp.close()
    fp = open(parser.parse(), "a")
    fp.write("\r\r")
    add_sub = local_add_sub(fp)
    add_sub.write(32+1, 0, "SIGNED")
    fp.close()