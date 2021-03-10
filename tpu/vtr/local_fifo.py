from rams import dpram
import math
import parser

class local_fifo():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, fifo_width, fifo_depth):
        awidth = int(math.log(fifo_depth, 2))
        string = '''\
module fifo_{FIFO_WIDTH}_{FIFO_DEPTH}(
input                      clk,
input                      reset,
input   [{FIFO_WIDTH}-1:0]   wrdata,
input                      write,

input                       read,
output reg [{FIFO_WIDTH}-1:0] rddata,
output reg                  full,
output reg                  empty 
);

localparam PTR_WIDTH = $clog2({FIFO_DEPTH});

wire [{FIFO_WIDTH}-1:0] data_a_nc;
wire [{FIFO_WIDTH}-1:0] out_b_nc;

reg  [PTR_WIDTH:0]    wrptr,rdptr;
wire [PTR_WIDTH:0]    wrptr_nxt,rdptr_nxt;

assign  wrptr_nxt = write? wrptr + 1'b1: wrptr;
assign  rdptr_nxt = read? rdptr + 1'b1:rdptr ;

always@(posedge clk) begin
  if(!reset)begin
    full <= 1'b0;
  end
  else begin
    if(wrptr_nxt[PTR_WIDTH]^ rdptr_nxt[PTR_WIDTH])
  	full <= ~(|(wrptr_nxt[PTR_WIDTH-1:0]^rdptr_nxt[PTR_WIDTH-1:0]));
    else
	full <= 1'b0;
  end 
end

always@(posedge clk) begin
  if(!reset)begin
    empty <= 1'b1;
  end
  else begin
    empty <= ~(|(wrptr_nxt^rdptr_nxt));
  end 
end

always @(posedge clk) begin
 if(!reset)begin
   wrptr <= 'h0;
 end
 else begin
     wrptr <= wrptr_nxt;
 end
end

always@(posedge clk) begin
 if(!reset)begin
   rdptr <= 'h0;
 end
 else begin
     rdptr <= rdptr_nxt;
 end
end

dpram_{AWIDTH}_{FIFO_DEPTH}_{FIFO_WIDTH} ram(
	.clk(clk),
    .address_a(rdptr),
    .address_b(wrptr),
    .wren_a(0),
    .wren_b(write),
    .data_a(data_a_nc),
    .data_b(wrdata),
    .out_a(rddata),
    .out_b(out_b_nc)
);
endmodule'''
        return string.format(FIFO_WIDTH=fifo_width, FIFO_DEPTH=fifo_depth, AWIDTH=awidth)

    def write (self, fifo_width, fifo_depth):
        self.fp.write(self.make_str(fifo_width, fifo_depth))

if __name__ == '__main__':
  fp = open(parser.parse(), "w") 
  uut = local_fifo(fp)
  uut.write(8, 4)
  fp.close()
  ram = dpram()
  fp = open(parser.parse(), "a")
  append_str = "\r\r" + ram.make_str(int(math.log(4, 2)), 4, 8)
  fp.write(append_str)
  fp.close()
  