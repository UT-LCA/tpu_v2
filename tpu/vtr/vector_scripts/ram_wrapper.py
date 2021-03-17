from rams import dpram
from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()
import os
class ram_wrapper():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, awidth, num_words, dwidth):
        string = '''\
module ram_wrapper_{AWIDTH}_{NUM_WORDS}_{DWIDTH} (
	clk,
        resetn,
	address_a,
	address_b,
        rden_a,
        rden_b,
	wren_a,
	wren_b,
	data_a,
	data_b,
	out_a,
	out_b
);

input clk;
input resetn;
input [({AWIDTH}-1):0] address_a;
input [({AWIDTH}-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [({DWIDTH}-1):0] data_a;
input [({DWIDTH}-1):0] data_b;
output [({DWIDTH}-1):0] out_a;
output [({DWIDTH}-1):0] out_b;

reg [({AWIDTH}-1):0] q_address_a;
reg [({AWIDTH}-1):0] q_address_b;
reg [({AWIDTH}-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_{AWIDTH}_{NUM_WORDS}_{DWIDTH} dpram1(
    .clk(clk),
    .address_a(address_a),
    .address_b(mux_address_b),
    .wren_a(wren_a),
    .wren_b(wren_b),
    .data_a(data_a),
    .data_b(data_b),
    .out_a(out_a),
    .out_b(out_b)
);

always@(posedge clk)begin
   if(!resetn)begin
     q_address_a <= 'h0;
     q_address_b <= 'h0;
   end
   else begin
     if(rden_b)
       q_address_b <= address_b;
   end
end

always@(*)begin
  if(rden_b)   
    mux_address_b = address_b;
  else
    mux_address_b = q_address_b; 
end

endmodule
'''       
        filename = "verilog/dpram_"+str(awidth)+"_"+str(num_words)+"_"+dwidth+".v"
        if(os.path.exists(filename) == False):
            fp =open(filename,'w')
            uut = dpram(fp)
            uut.write(awidth,num_words,dwidth)
            fp.close()

        return string.format(AWIDTH=awidth, NUM_WORDS=num_words, DWIDTH=dwidth)

    def write (self, awidth, num_words, dwidth):
        self.fp.write(self.make_str(awidth, num_words, dwidth))

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = ram_wrapper(fp)
    uut.write(10, 1024, 32)
    fp.close()
    fp = open(parser.parse(), "a")
    fp.write("\r\r")
    ram = dpram(fp)
    ram.write(10, 1024, 32)
    fp.close()
