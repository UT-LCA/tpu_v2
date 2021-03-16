import parser

class signext16():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self):
        string = '''
module signext16 ( in, out);

input [15:0] in;
output [31:0] out;

assign out={{{{16{{in[15]}}}},in[15:0]}};

endmodule
'''       
        return string

    def write (self):
        self.fp.write(self.make_str())

if __name__ == '__main__':
	fp = open(parser.parse(), "w")
	uut = signext16(fp)
	uut.write()
	fp.close()