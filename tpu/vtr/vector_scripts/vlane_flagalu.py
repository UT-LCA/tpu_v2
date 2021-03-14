import parser

class vlane_flagalu():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self):
        string = '''\
/******************************************************************************

0  VFAND
1  VFOR
2  VFXOR
3  VFNOR
4  VFCLR
5  VFSET

Not handled here:
  VIOTA
  VCIOTA
  VFPOP
  VFFF1
  VFFL1
  VFSETBF
  VFSETIF
  VFSETOF

******************************************************************************/

module vlane_flagalu (
    clk,
    resetn,

    src1,
    src2,

    op,

    result

    );

input clk;
input resetn;

input src1;
input src2;
input [2:0] op;

output result;

reg result;

  always@*
    case(op)
      0: result=src1&src2;
      1: result=src1|src2;
      2: result=src1^src2;
      3: result=~(src1|src2);
      4: result=1'b0;
      5: result=1'b1;
      default: result=1'b0;
    endcase

endmodule'''

        return string

    def write (self):
        self.fp.write(self.make_str())

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = vlane_flagalu(fp)
    uut.write()
    fp.close()
