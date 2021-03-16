import parser

class merge26lo():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self):
        string = '''
module merge26lo(in1, in2, out);
input [31:0] in1;
input [25:0] in2;
output [31:0] out;

// not connected port
wire [31:0] in1_nc;
assign in1_nc = in1;

assign out[31:0]={in1[31:28],in2[25:0],2'b0};
endmodule
'''       

        return string

    def write (self):
        self.fp.write(self.make_str())

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = merge26lo(fp)
    uut.write()
    fp.close()