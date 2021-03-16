import parser

class branchresolve():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''

module branchresolve_{WIDTH} ( en, rs, rt, eq, ne, ltz, lez, gtz, gez, eqz);
parameter WIDTH={WIDTH};
input en;
input [WIDTH-1:0] rs;
input [WIDTH-1:0] rt;
output eq;
output ne;
output ltz;
output lez;
output gtz;
output gez;
output eqz;

assign eq=(en)&(rs==rt);
assign ne=(en)&~eq;
assign eqz=(en)&~(|rs);
assign ltz=(en)&rs[WIDTH-1];
assign lez=(en)&rs[WIDTH-1] | eqz;
assign gtz=(en)&(~rs[WIDTH-1]) & ~eqz;
assign gez=(en)&(~rs[WIDTH-1]);

endmodule
'''       

        return string.format(WIDTH=width)

    def write (self, width):
        self.fp.write(self.make_str(width))

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = branchresolve(fp)
    uut.write(32)
    fp.close()