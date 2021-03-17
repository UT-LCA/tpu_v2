from reduction import reduction_layer
from transpose import transpose
from math import log
from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class trp_unit():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        logwidth = int(log(width,2))
        string = '''\

module trp_unit_{WIDTH} (
 input clk,
 input resetn,
 input en,
 input [{WIDTH}-1:0] a,
 input [1:0] mode,
 input read,
 output busy,
 output reg valid,
 output reg[{WIDTH}-1:0] out
);

reg en_reduction;
reg en_transpose;
reg read_transpose;
reg read_reduction;
wire transpose_busy;
wire reduction_busy;
wire reduction_done;
wire [{WIDTH}-1:0] reduction_out;
wire [{WIDTH}-1:0] transpose_out;
 
 
assign busy = transpose_busy || reduction_busy;

always@(*)begin
  if(mode == 2'b11)begin
    en_transpose = en;
    en_reduction = 1'b0;
    read_transpose = read;
    read_reduction = 1'b0;
  end
  else begin
    en_transpose = 1'b0;
    en_reduction = en;
    read_transpose = 1'b0;
    read_reduction = read;
  end
end

always@(*)begin
  if(transpose_busy)begin
    out = transpose_out;
    valid = 1'b1;
  end
  else if(reduction_done)begin
    out = reduction_out;
    valid = 1'b1;
  end
  else begin
    out = 'h0;
    valid = 1'b0;
  end
end

reduction_layer_{WIDTH}_{LOGWIDTH}_32_5_10 u_reduction_layer(
  .clk(clk),
  .resetn(resetn),
  .en(en_reduction),
  .read(read_reduction),
  .reduction_type(mode),
  .a(a),
  .reduced_out(reduction_out),
  .done(reduction_done),
  .busy(reduction_busy)
);

transpose_{WIDTH}_2_1 u_transpose(
  .clk(clk),
  .resetn(resetn),
  .read(),
  .en(en_transpose), 
  .a(a),
  .out(transpose_out),
  .busy(transpose_busy)
);

endmodule

'''
        fp1 = open("verilog/reduction_layer.v", "a")
        uut1 = reduction_layer(fp1)
        uut1.write(width,logwidth,32,5,10)
        fp1.close()
        fp2 = open("verilog/transpose.v",'a')
        uut2 = transpose(fp2)
        uut2.write(width,2,1)
        fp2.close()

        return string.format(WIDTH=width,LOGWIDTH=logwidth) 

    def write (self, width):
        self.fp.write(self.make_str(width))

if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = trp_unit(fp)
    uut1.write(32)
    fp.close()
