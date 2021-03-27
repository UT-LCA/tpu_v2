from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class activation():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''\
module activation_{WIDTH}(
 input clk,
 input resetn,
 input en,
 input stall,
 input [{WIDTH}-1:0] a,
 output reg[{WIDTH}-1:0] out
);

always@(posedge clk)begin
  if(!resetn)
    out <= 'h0;
  else
    if(en)
      if(a>0)
        out <= a;
      else
        out <= 0;
end

endmodule

        '''
        return string.format(WIDTH=width) 

    def write (self, width):
        self.fp.write(self.make_str(width))

if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = activation(fp)
    uut1.write(10)
    fp.close()
