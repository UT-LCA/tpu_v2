import parser

class pcadder():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, pc_width):
        string = '''\
module pcadder_{PC_WIDTH}(pc, offset, result);

input [{PC_WIDTH}-1:0] pc;
input [{PC_WIDTH}-1:0] offset;
output [{PC_WIDTH}-1:0] result;

// not connect ports
wire [{PC_WIDTH}-1:0] offset_nc;
assign offset_nc = offset;

wire dum;

assign {{dum,result}} = pc + {{offset[{PC_WIDTH}-3:0],2'b0}};

endmodule'''       

        return string.format(PC_WIDTH=pc_width)

    def write (self, pc_width):
        self.fp.write(self.make_str(pc_width))

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = pcadder(fp)
    uut.write(32)
    fp.close()