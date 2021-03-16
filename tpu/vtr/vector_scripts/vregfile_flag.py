from ram_wrapper import ram_wrapper
from math import log
from optparse import OptionParser

parser = OptionParser()
(_,args) = parser.parse_args()

class vregfile_flag():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self,numbanks, log2numbanks, width, numregs, log2numregs):
        numregsperbank = numregs/numbanks
        log2numregsperbank = int(log(numregsperbank,2))
        string = '''\
/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_flag_{NUMBANKS}_{LOG2NUMBANKS}_{WIDTH}_{NUMREGS}_{LOG2NUMREGS}
  (clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_we);

parameter {NUMBANKS}=1;
parameter {LOG2NUMBANKS}=0;
parameter {WIDTH}=32;
parameter {NUMREGS}=32;
parameter {LOG2NUMREGS}=5;

parameter {NUMREGSPERBANK}={NUMREGS}/{NUMBANKS};
parameter {LOG2NUMREGSPERBANK}={LOG2NUMREGS}-{LOG2NUMBANKS};

input clk;
input resetn;

input [{NUMBANKS}-1:0] a_en;
input [{NUMBANKS}-1:0] b_en;
input [{NUMBANKS}-1:0] c_we;

input [{NUMBANKS}*{LOG2NUMREGSPERBANK}-1:0] a_reg,b_reg,c_reg;
output [{NUMBANKS}*{WIDTH}-1:0] a_readdataout, b_readdataout;
input [{NUMBANKS}*{WIDTH}-1:0] c_writedatain;

genvar k;

generate
for (k=0; k<{NUMBANKS}; k=k+1)
begin : bank_gen
      ram_wrapper_{LOG2NUMREGSPERBANK}_{NUMREGSPERBANK}_{WIDTH} reg_file1(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(a_en[k]),
          .address_a(c_reg[k* {LOG2NUMREGSPERBANK} +: {LOG2NUMREGSPERBANK}]),
          .address_b(a_reg[k* {LOG2NUMREGSPERBANK} +: {LOG2NUMREGSPERBANK}]),
          .wren_a(c_we[k]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ k*{WIDTH} +: {WIDTH}]),
          .data_b(0),
          .out_a(),
          .out_b(a_readdataout[k*{WIDTH} +: {WIDTH}])
      );

      ram_wrapper_{LOG2NUMREGSPERBANK}_{NUMREGSPERBANK}_{WIDTH} reg_file2(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(b_en[k]),
          .address_a(c_reg[k* {LOG2NUMREGSPERBANK} +: {LOG2NUMREGSPERBANK}]),
          .address_b(b_reg[k* {LOG2NUMREGSPERBANK} +: {LOG2NUMREGSPERBANK}]),
          .wren_a(c_we[k]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ k*{WIDTH} +: {WIDTH}]),
          .data_b(0),
          .out_a(),
          .out_b(b_readdataout[ k* {WIDTH} +: {WIDTH}])
      );
end
endgenerate

endmodule
'''
        fp = open("ram_wrapper.v", 'a')
        uut = ram_wrapper(fp)
        uut.write(log2numregsperbank, numregsperbank, width)
        fp.close()
        return string.format(NUMBANKS=numbanks, LOG2NUMBANKS=log2numbanks, WIDTH=width, NUMREGS=numregs, LOG2NUMREGS = log2numregs, NUMREGSPERBANK=numregsperbank, LOG2NUMREGSPERBANK= log2numregsperbank) 

    def write(self,numbanks, log2numbanks, width, numregs, log2numregs):
        self.fp.write(self.make_str(numbanks, log2numbanks, width, numregs, log2numregs))

if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = vregfile_flag(fp)
    uut1.write(2,1,32,16,4)
    fp.close();
