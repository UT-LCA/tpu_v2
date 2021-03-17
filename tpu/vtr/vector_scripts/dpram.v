module dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output reg [(128-1):0] out_a;
output reg [(128-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [128-1:0] ram[256-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output reg [(128-1):0] out_a;
output reg [(128-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [128-1:0] ram[256-1:0];

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

endmodulemodule dpram_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output reg [(8-1):0] out_a;
output reg [(8-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [8-1:0] ram[128-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output reg [(128-1):0] out_a;
output reg [(128-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [128-1:0] ram[256-1:0];

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

endmodulemodule dpram_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output reg [(8-1):0] out_a;
output reg [(8-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [8-1:0] ram[128-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output reg [(128-1):0] out_a;
output reg [(128-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [128-1:0] ram[256-1:0];

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

endmodulemodule dpram_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output reg [(8-1):0] out_a;
output reg [(8-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [8-1:0] ram[128-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output reg [(128-1):0] out_a;
output reg [(128-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [128-1:0] ram[256-1:0];

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

endmodulemodule dpram_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output reg [(8-1):0] out_a;
output reg [(8-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [8-1:0] ram[128-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output reg [(128-1):0] out_a;
output reg [(128-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [128-1:0] ram[256-1:0];

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

endmodulemodule dpram_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output reg [(8-1):0] out_a;
output reg [(8-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [8-1:0] ram[128-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output reg [(128-1):0] out_a;
output reg [(128-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [128-1:0] ram[256-1:0];

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

endmodulemodule dpram_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output reg [(8-1):0] out_a;
output reg [(8-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [8-1:0] ram[128-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output reg [(128-1):0] out_a;
output reg [(128-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [128-1:0] ram[256-1:0];

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

endmodulemodule dpram_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output reg [(8-1):0] out_a;
output reg [(8-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [8-1:0] ram[128-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output reg [(128-1):0] out_a;
output reg [(128-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [128-1:0] ram[256-1:0];

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

endmodulemodule dpram_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output reg [(8-1):0] out_a;
output reg [(8-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [8-1:0] ram[128-1:0];

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

endmodulemodule dpram_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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

endmodulemodule dpram_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

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

endmodulemodule dpram_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output reg [(128-1):0] out_a;
output reg [(128-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [128-1:0] ram[256-1:0];

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

endmodulemodule dpram_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output reg [(8-1):0] out_a;
output reg [(8-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [8-1:0] ram[128-1:0];

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

endmodulemodule dpram_4_16_32 (
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
input [(4-1):0] address_a;
input [(4-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_4_16_32 (
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
input [(4-1):0] address_a;
input [(4-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_4_16_32 (
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
input [(4-1):0] address_a;
input [(4-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_4_16_32 (
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
input [(4-1):0] address_a;
input [(4-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_4_16_32 (
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
input [(4-1):0] address_a;
input [(4-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[16-1:0];

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

endmodulemodule dpram_3_8_32 (
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
input [(3-1):0] address_a;
input [(3-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[8-1:0];

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