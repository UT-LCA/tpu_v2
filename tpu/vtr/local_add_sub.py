import parser

class local_add_sub():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width, pipeline, representation):
        string = '''
module local_add_sub_{WIDTH}_{PIPELINE}_{REPRESENTATION}(
dataa,
datab,
cin,
add_sub,
result
);

input[{WIDTH}-1:0] dataa;
input[{WIDTH}-1:0] datab;
input cin;
input add_sub;
output reg [{WIDTH}-1:0] result;

always @(*)begin
    if(add_sub == 1'b1)
         result = dataa+datab+cin;
    else
         result = dataa - datab;
end

endmodule
'''

        return string.format(WIDTH=width, PIPELINE=pipeline, REPRESENTATION=representation)

    def write(self, width, pipeline, representation):
        self.fp.write(self.make_str(width, pipeline, representation))


if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = local_add_sub(fp)
    uut.write(32, 0, "SIGNED")
    fp.close()