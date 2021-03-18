from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class test():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''\

module test_{WIDTH} (
 input clk,
 input resetn,
 input en,
 input stall,
 input [{WIDTH}-1:0] a,
 input [{WIDTH}-1:0] b,
 output reg[{WIDTH}-1:0] out
);

function [{WIDTH}-1:0] sum;
    input [31:0] x, b;
    begin
        sum = x+b;
    end
endfunction

always@*
    out = sum (a,b);

endmodule
        '''
        return string.format(WIDTH=width) 

    def write (self, width):
        self.fp.write(self.make_str(width))

if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = test(fp)
    uut1.write(32)
    fp.close()
