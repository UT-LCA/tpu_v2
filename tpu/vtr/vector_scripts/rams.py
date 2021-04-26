from optparse import OptionParser

parser = OptionParser()
(_,args) = parser.parse_args()

class dpram():
    def __init__(self, fp=None):
        self.fp = fp

    def make_str(self, awidth, num_words, dwidth):
        string = '''\
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
input [{AWIDTH}-1:0] address_a;
input [{AWIDTH}-1:0] address_b;
input  wren_a;
input  wren_b;
input [{DWIDTH}-1:0] data_a;
input [{DWIDTH}-1:0] data_b;
output [{DWIDTH}-1:0] out_a;
output[{DWIDTH}-1:0] out_b;

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

endmodule
'''

        return string.format(AWIDTH=awidth, NUM_WORDS=num_words, DWIDTH=dwidth)

    def write (self, awidth, num_words, dwidth):
        self.fp.write(self.make_str(awidth, num_words, dwidth))

class spram():
    def __init__(self, fp=None):
        self.fp = fp

    def make_str(self, awidth, num_words, dwidth):
        string = '''\
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

endmodule'''

        return string.format(AWIDTH=awidth, NUM_WORDS=num_words, DWIDTH=dwidth)

    def write (self, awidth, num_words, dwidth):
        self.fp.write(self.make_str(awidth, num_words, dwidth))
        self.fp.close()

class dpram1():
    def __init__(self, fp=None):
        self.fp = fp

    def make_str(self, awidth, num_words, dwidth):
        string = '''\
module dpram1_{AWIDTH}_{NUM_WORDS}_{DWIDTH} (
    clk,
    address_a,
    address_b,
    wren_a, 
    wren_b, 
    data_a,
    data_b,
    byteen_a,
    byteen_b,
    out_a,
    out_b
);

// parameter AWIDTH=10;
// parameter NUM_WORDS=1024;
// parameter DWIDTH=32;
// parameter LOG2DWIDTH = $clog2(DWIDTH);

input clk;
input [({AWIDTH}-1):0] address_a;
input [({AWIDTH}-1):0] address_b;
input  wren_a;
input  wren_b;
input [({DWIDTH}/8)-1:0] byteen_a;
input [({DWIDTH}/8)-1:0] byteen_b;
input [({DWIDTH}-1):0] data_a;
input [({DWIDTH}-1):0] data_b;
output reg [({DWIDTH}-1):0] out_a;
output reg [({DWIDTH}-1):0] out_b;

`ifdef SIMULATION_MEMORY

integer i;
integer k;

//reg [32-1:0] ram [67108864-1:0];
reg [32-1:0] ram[4096-1:0];
reg [25:0] addr_a;
reg [25:0] addr_b;
 
initial
 begin
   //This is TOO big for 256 MB RAM!  We right shift data by 1
   //$readmemh("instr.dat",ram,'h100_0000);
   $readmemh("instr.dat",ram,'h100);
   //$readmemh("data.dat",ram,'h400_0000>>1);
   $readmemh("data.dat",ram,'h400>>1);
 end

always@(*) begin
    addr_a = address_a << 26-{AWIDTH};
    addr_b = address_b << 26-{AWIDTH};
end

always@(posedge clk) begin 
  if (wren_a) begin
      for(k=0; k < {DWIDTH}/32;k=k+1)begin
          for(i=0; i < 4 ;i=i+1)begin
              if(byteen_a[(({DWIDTH}/8-1)-((4*k)+i))])
                  ram[addr_a+k][(i*8)+:8] <= data_a[((32*k)+(i*8))+:8];
          end
      end
  end
  else begin
      for(i=0; i < {DWIDTH}/32; i=i+1)begin
          out_a[(32*i)+:32] <= ram[addr_a+i];
      end
  end
  if (wren_b) begin
      for(k=0; k < {DWIDTH}/32;k=k+1)begin
          for(i=0; i < 4 ;i=i+1)begin
              if(byteen_b[(({DWIDTH}/8-1)-((4*k)+i))])
                  ram[addr_b+k][(i*8)+:8] <= data_b[((32*k)+(i*8))+:8];
          end
      end
  end
  else begin
      for(i=0; i < {DWIDTH}/32; i=i+1)begin
          out_b[32*i+:32] <= ram[addr_b+i];
      end
  end
end
`else

// Not connected wires
wire [({DWIDTH}/8)-1:0] byteen_a_nc;
wire [({DWIDTH}/8)-1:0] byteen_b_nc;

assign byteen_a_nc = byteen_a;
assign byteen_b_nc = byteen_b;

dual_port_ram u_dual_port_ram(
.addr1(address_a[11:0]),
.we1(wren_a),
.data1(data_a),
.out1(out_a),
.addr2(address_b[11:0]),
.we2(wren_b),
.data2(data_b),
.out2(out_b),
.clk(clk)
);

`endif

endmodule'''

        return string.format(AWIDTH=awidth, NUM_WORDS=num_words, DWIDTH=dwidth)

    def write (self, awidth, num_words, dwidth):
        self.fp.write(self.make_str(awidth, num_words, dwidth))
        self.fp.close()

if __name__ == '__main__':

    fp = open(args[0], "w")
    ram = dpram1(fp)
    ram.write(22, 4096, 32)
    fp.close()
