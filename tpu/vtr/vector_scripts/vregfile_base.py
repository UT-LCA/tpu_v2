from ram_wrapper import ram_wrapper
from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class vregfile_base():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width, numregs, log2numregs):
        string = '''\
/****************************************************************************
          Base Register File

   - Has one read port (a) and one write port (c)
****************************************************************************/
module vregfile_base_{WIDTH}_{NUMREGS}_{LOG2NUMREGS} (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

input clk;
input resetn;

input [{LOG2NUMREGS}-1:0] a_reg,c_reg;
output [{WIDTH}-1:0] a_readdataout;
input [{WIDTH}-1:0] c_writedatain;
input a_en, c_we;

        ram_wrapper_{LOG2NUMREGS}_{NUMREGS}_{WIDTH} reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[{LOG2NUMREGS}-1:0]),
	    .address_b(a_reg[{LOG2NUMREGS}-1:0]),
	    .wren_a(c_we),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout)
        );

endmodule

        '''
        fp = open("ram_wrapper.v", 'a')
        uut = ram_wrapper(fp)
        uut.write(log2numregs, numregs, width)
        fp.close()
        return string.format(WIDTH=width, NUMREGS=numregs, LOG2NUMREGS = log2numregs) 

    def write (self, width, numregs, log2numregs):
        self.fp.write(self.make_str(width,numregs,log2numregs))

if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = vregfile_base(fp)
    uut1.write(32,16,4)
    fp.close()
