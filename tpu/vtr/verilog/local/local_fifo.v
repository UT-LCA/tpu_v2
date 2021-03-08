module fifo_8_4(
input                      clk,
input                      reset,
input   [8-1:0]   wrdata,
input                      write,

input                       read,
output reg [8-1:0] rddata,
output reg                  full,
output reg                  empty 
);

localparam PTR_WIDTH = $clog2(4);

wire [8-1:0] data_a_nc;
wire [8-1:0] out_b_nc;

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

dpram_2_4_8 ram(
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
endmodulemodule dpram_2_4_8 (
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

parameter AWIDTH = 2;
parameter NUM_WORDS = 4;
parameter DWIDTH = 8;

input clk;
input [(AWIDTH-1):0] address_a;
input [(AWIDTH-1):0] address_b;
input  wren_a;
input  wren_b;
input [(DWIDTH-1):0] data_a;
input [(DWIDTH-1):0] data_b;
output reg [(DWIDTH-1):0] out_a;
output reg [(DWIDTH-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [DWIDTH-1:0] ram[NUM_WORDS-1:0];

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