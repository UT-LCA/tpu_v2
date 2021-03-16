import parser

class dpram():
    def __init__(self, fp=None):
        self.fp = fp

    def make_str(self, awidth, num_words, dwidth):
        string = '''
module dpram_{AWIDTH}_{NUM_WORDS}_{DWIDTH} (
	clk,
	address_a,
	address_b,
	wren_a,
	wren_b,
	data_a,
	data_b,
	out_a,
	out_b
);

input clk;
input [({AWIDTH}-1):0] address_a;
input [({AWIDTH}-1):0] address_b;
input  wren_a;
input  wren_b;
input [({DWIDTH}-1):0] data_a;
input [({DWIDTH}-1):0] data_b;
output reg [({DWIDTH}-1):0] out_a;
output reg [({DWIDTH}-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [{DWIDTH}-1:0] ram[{NUM_WORDS}-1:0];

always @ (posedge clk) begin 
  if (wren_a) begin
      ram[address_a] <= data_a;
  end
  else begin
      out_a <= ram[address_a];
  end
end
  
always @ (posedge clk) begin 
  if (wren_b) begin
      ram[address_b] <= data_b;
  end 
  else begin
      out_b <= ram[address_b];
  end
end

`else

dual_port_ram u_dual_port_ram(
.addr1(address_a),
.we1(wren_a),
.data1(data_a),
.out1(out_a),
.addr2(address_b),
.we2(wren_b),
.data2(data_b),
.out2(out_b),
.clk(clk)
);

`endif

endmodule
'''

        return string.format(AWIDTH=awidth, NUM_WORDS=num_words, DWIDTH=dwidth)

    def write (self, awidth, num_words, dwidth):
        self.fp.write(self.make_str(awidth, num_words, dwidth))

class spram():
    def __init__(self, fp=None):
        self.fp = fp

    def make_str(self, awidth, num_words, dwidth):
        string = '''
module spram_{AWIDTH}_{NUM_WORDS}_{DWIDTH} (
    clk,
	address,
	wren,
	data,
	out
);

input clk;
input [({AWIDTH}-1):0] address;
input  wren;
input [({DWIDTH}-1):0] data;
output reg [({DWIDTH}-1):0] out;

`ifdef SIMULATION_MEMORY

reg [{DWIDTH}-1:0] ram[{NUM_WORDS}-1:0];

always @ (posedge clk) begin 
  if (wren) begin
      ram[address] <= data;
  end
  else begin
      out <= ram[address];
  end
end
  
`else

single_port_ram u_single_port_ram(
.addr(address),
.we(wren),
.data(data),
.out(out),
.clk(clk)
);

`endif

endmodule
'''

        return string.format(AWIDTH=awidth, NUM_WORDS=num_words, DWIDTH=dwidth)

    def write (self, awidth, num_words, dwidth):
        self.fp.write(self.make_str(awidth, num_words, dwidth))
        self.fp.close()


if __name__ == '__main__':

    fp = open(parser.parse(), "w")
    ram = spram(fp)
    ram.write(10, 1024, 32)
    fp.close()