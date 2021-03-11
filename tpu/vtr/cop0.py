from components import multicyclestall
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

module cop0(
    clk,
    resetn,
    stalled,

    instr,

    exception,

    read_addr,
    dest_addr,
    fromcpu,
    fromcpu_en,
    tocpu,
    tocpu_en,

    epc_in,
    ext_cause_in,
    int_cause_in_stage1,  //very weak - implement OR in SPREE instead
    int_cause_in_stage2,
    status,

    badvaddr_in,
    badvaddr_we
    );

//parameter NUMSTAGESTIMES32=64;
//parameter NUMSTAGES=NUMSTAGESTIMES32/32;

input clk;
input resetn;
output stalled;

input   [31:0] instr;

output exception;

input   [4:0]  read_addr;
input   [4:0]  dest_addr;
input   [31:0] fromcpu;
input          fromcpu_en;
output  [31:0] tocpu;
input          tocpu_en;

input  [31:0] epc_in;

input  [31:0] ext_cause_in;
input  [31:0] int_cause_in_stage1;
input  [31:0] int_cause_in_stage2;

output [31:0] status;
input  [31:0] badvaddr_in;
input         badvaddr_we;

// not connected ports
wire [31:0] instr_nc;
assign instr_nc = instr;

wire [31:0] cause_in;

reg [31:0] epc_out;
reg [31:0] cause_out;
reg [31:0] status;
reg [31:0] badvaddr_out;

reg  [31:0] tocpu;

  assign cause_in=ext_cause_in | int_cause_in_stage1 | 
                                 int_cause_in_stage2;

  always@(posedge clk)
    if (!resetn)
      epc_out<=0;
    else if (fromcpu_en && dest_addr==14)
      epc_out<=fromcpu;
    else if (exception)
      epc_out<=epc_in;

  always@(posedge clk)
    if (!resetn)
      cause_out<=0;
    else if (fromcpu_en && dest_addr==13)
      cause_out<=fromcpu;
    else
      cause_out<=cause_in;

  always@(posedge clk)
    if (!resetn)
      status<=0;
    else if (fromcpu_en && dest_addr==12)
      status<=fromcpu;
    else if (exception)
      status[5:0]<={status[3:0],2'b0};

  always@(posedge clk)
    if (!resetn)
      badvaddr_out<=0;
    else if (fromcpu_en && dest_addr==8)
      badvaddr_out<=fromcpu;
    else if (badvaddr_we)
      badvaddr_out<=badvaddr_in;

  always@(posedge clk)
      tocpu <= (read_addr==14) ? epc_out : 
               (read_addr==13) ? cause_out : 
               (read_addr==8)  ? badvaddr_out : status;

  // 1 cycle stall
  multicyclestall mc(tocpu_en,0,clk,resetn,stalled);
  //assign stalled= 0;

  assign exception = ((|(cause_in[15:8] & status[15:8])) && status[0]);

endmodule'''       

        return string

    def write (self):
        self.fp.write(self.make_str())

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = cop0(fp)
    uut.write()
    fp.close()
    fp = open(parser.parse(), "a")
    fp.write("\r\r")
    stall = multicyclestall(fp)
    stall.write()
    fp.close()