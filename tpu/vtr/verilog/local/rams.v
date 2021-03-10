module spram_10_1024_32 (
    clk,
	address,
	wren,
	data,
	out
);

input clk;
input [(10-1):0] address;
input  wren;
input [(32-1):0] data;
output reg [(32-1):0] out;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[1024-1:0];

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