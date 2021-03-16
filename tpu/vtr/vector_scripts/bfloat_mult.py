from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class bfloat_mult():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''\

module bfloat_mult_{WIDTH} (
 input clk,
 input resetn,
 input en,
 input stall,
 input [{WIDTH}-1:0] a,
 input [{WIDTH}-1:0] b,
 output reg[{WIDTH}-1:0] out
);

always@(posedge clk)begin
  if(!resetn)
    out <= 'h0;
  else
    if(en)
      out <= a * b;
end

endmodule
        '''
        return string.format(WIDTH=width) 

    def write (self, width):
        self.fp.write(self.make_str(width))

if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = bfloat_mult(fp)
    uut1.write(32)
    fp.close()
