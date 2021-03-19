module dpram_9_512_128 (
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
input [9-1:0] address_a;
input [9-1:0] address_b;
input  wren_a;
input  wren_b;
input [128-1:0] data_a;
input [128-1:0] data_b;
output reg [128-1:0] out_a;
output reg [128-1:0] out_b;

reg [128-1:0] ram[512-1:0];

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

endmodule
