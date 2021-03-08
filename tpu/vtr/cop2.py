import parser

class cop0():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self):
        string = '''\
/*******
 * SPREE limitation - by not specifying stall signal name and assuming
 * "stalled" requires you to have only one opcode port which stalls
 *
 * We get around this since both INPUT&OUTPUT are in the same stage so we 
 * can use the same stall signal.
 *******/

module cop2(
    clk,
    resetn,
    stalled,

    fromcpu,
    fromcpu_en,
    tocpu,
    tocpu_en,

    //Global I/O
    tocop2,
    tocop2_en,
    tocop2_wait,
    fromcop2,
    fromcop2_en,
    fromcop2_wait
    );

input clk;
input resetn;
output stalled;

input   [31:0] fromcpu;
input          fromcpu_en;
output  [31:0] tocpu;
input          tocpu_en;

output  [31:0] tocop2;
output         tocop2_en;
input          tocop2_wait;
input   [31:0] fromcop2;
input          fromcop2_en;
output         fromcop2_wait;


  assign tocop2=fromcpu;
  assign tocop2_en=fromcpu_en;

  assign tocpu=fromcop2;
  assign fromcop2_wait=fromcop2_en&~tocpu_en;   //assign 1 if pipe is stalled 

  assign stalled= (fromcpu_en & tocop2_wait) || (tocpu_en & ~fromcop2_en);

endmodule'''       

        return string

    def write (self):
        self.fp.write(self.make_str())

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = cop0(fp)
    uut.write()
    fp.close()