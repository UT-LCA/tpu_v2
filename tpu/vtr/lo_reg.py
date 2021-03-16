import parser

class lo_reg():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''
/****************************************************************************
          Generic Register
****************************************************************************/
module lo_reg_{WIDTH} (d,clk,resetn,squashn,en,q);

input clk;
input resetn;
input squashn;
input en;
input [{WIDTH}-1:0] d;
output [{WIDTH}-1:0] q;
reg [{WIDTH}-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1 && squashn)
		q<=d;
end

endmodule
'''       

        return string.format(WIDTH=width)

    def write (self, width):
        self.fp.write(self.make_str(width))

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = lo_reg(fp)
    uut.write(32)
    fp.close()