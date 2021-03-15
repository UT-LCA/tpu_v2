from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class vregfile_control():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width, numregs, log2numregs,matmulsize):
        string = '''\
/****************************************************************************
          Control Register File

   - Has one read port (a) and one write port (c)
   - vl promoted as first-class entities
****************************************************************************/
module vregfile_control_{WIDTH}_{NUMREGS}_{LOG2NUMREGS} (
    clk,
    resetn, 

    a_reg, 
    a_en,
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we,

    vl,
    matmul_masks
    );

input clk;
input resetn;

input a_en;
input [{LOG2NUMREGS}-1:0] a_reg,c_reg;
output [{WIDTH}-1:0] a_readdataout;
input [{WIDTH}-1:0] c_writedatain;
input c_we;

output [{WIDTH}-1:0] vl;
output [3*{MAT_MUL_SIZE}-1:0] matmul_masks;

reg [{WIDTH}-1:0] vl;
reg [{WIDTH}-1:0] matmul_masks;

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

`ifdef TEST_BENCH
   initial begin
       $readmemh("vregfile_control.dat",reg_file1.dpram1.ram,'h0);
   end
`endif 

always@(posedge clk) begin
  if (!resetn) begin
    vl<=0;
    matmul_masks<=32'hffffffff;
  end
  else begin
    if (c_we) begin
      if (c_reg==0) begin
        vl<=c_writedatain;
      end 
      else if (c_reg==31) begin //a_rows
        matmul_masks[1*{MAT_MUL_SIZE}-1:0*{MAT_MUL_SIZE}] <= c_writedatain[{MAT_MUL_SIZE}-1:0];
      end
      else if (c_reg==30) begin //a_cols, b_rows
        matmul_masks[2*{MAT_MUL_SIZE}-1:1*{MAT_MUL_SIZE}] <= c_writedatain[{MAT_MUL_SIZE}-1:0];
      end
      else if (c_reg==29) begin //b_cols
        matmul_masks[3*{MAT_MUL_SIZE}-1:2*{MAT_MUL_SIZE}] <= c_writedatain[{MAT_MUL_SIZE}-1:0];
      end
    end  
  end
end

endmodule
'''
        return string.format(WIDTH=width, NUMREGS=numregs, LOG2NUMREGS = log2numregs, MAT_MUL_SIZE=matmulsize) 

    def write(self, width, numregs, log2numregs, matmulsize):
        self.fp.write(self.make_str(width,numregs,log2numregs,matmulsize))

if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = vregfile_control(fp)
    uut1.write(32,16,4,8)
    fp.close()
