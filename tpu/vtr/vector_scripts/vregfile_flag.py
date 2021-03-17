from ram_wrapper import ram_wrapper
from math import log
from optparse import OptionParser
import os

parser = OptionParser()
(_,args) = parser.parse_args()

class vregfile_flag():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self,numbanks, log2numbanks, width, numregs, log2numregs):
        numregsperbank = int(numregs/numbanks)
        log2numregsperbank = int(log(numregsperbank,2))
        string1 = '''\
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

input clk;
input resetn;

input [{NUMBANKS}-1:0] a_en;
input [{NUMBANKS}-1:0] b_en;
input [{NUMBANKS}-1:0] c_we;

input [{NUMBANKS}*{LOG2NUMREGSPERBANK}-1:0] a_reg,b_reg,c_reg;
output [{NUMBANKS}*{WIDTH}-1:0] a_readdataout, b_readdataout;
wire [{NUMBANKS}*{WIDTH}-1:0] a_temp, b_temp;
input [{NUMBANKS}*{WIDTH}-1:0] c_writedatain;

'''
        string2_basic ='''
      ram_wrapper_{LOG2NUMREGSPERBANK}_{NUMREGSPERBANK}_{WIDTH} reg_file1__k_(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(a_en[_k_]),
          .address_a(c_reg[_k_* {LOG2NUMREGSPERBANK} +: {LOG2NUMREGSPERBANK}]),
          .address_b(a_reg[_k_* {LOG2NUMREGSPERBANK} +: {LOG2NUMREGSPERBANK}]),
          .wren_a(c_we[_k_]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ _k_*{WIDTH} +: {WIDTH}]),
          .data_b(0),
          .out_a(a_temp[ _k_*{WIDTH} +: {WIDTH}]),
          .out_b(a_readdataout[_k_*{WIDTH} +: {WIDTH}])
      );

      ram_wrapper_{LOG2NUMREGSPERBANK}_{NUMREGSPERBANK}_{WIDTH} reg_file2__k_(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(b_en[_k_]),
          .address_a(c_reg[_k_* {LOG2NUMREGSPERBANK} +: {LOG2NUMREGSPERBANK}]),
          .address_b(b_reg[_k_* {LOG2NUMREGSPERBANK} +: {LOG2NUMREGSPERBANK}]),
          .wren_a(c_we[_k_]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ _k_*{WIDTH} +: {WIDTH}]),
          .data_b(0),
          .out_a(b_temp[ _k_*{WIDTH} +: {WIDTH}]),
          .out_b(b_readdataout[ _k_* {WIDTH} +: {WIDTH}])
      );
      '''
        string2 = ""
        for k in range(0,numbanks):
            string2_basic = string2_basic.replace("_k_*{LOG2NUMREGSPERBANK} +: {LOG2NUMREGSPERBANK}",str((((k+1)*log2numregsperbank)-1))+":"+str(k* log2numregsperbank))
            string2_basic = string2_basic.replace("_k_*{WIDTH}+:{WIDTH}",str(((k+1)*width)-1)+":"+str(k*width))
            string2 += string2_basic.replace("_k_",str(k))

        string3='''      
endmodule
'''
        string = string1 + string2 + string3 
        filename = "verilog/ram_wrapper_"+str(log2numregs)+"_"+str(numregs)+"_"+str(width)+".v"
        if(os.path.exists(filename) == False):
            fp = open(filename, 'a')
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
