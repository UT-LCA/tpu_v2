module ram_wrapper_10_1024_32 (
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
input [(10-1):0] address_a;
input [(10-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(10-1):0] q_address_a;
reg [(10-1):0] q_address_b;
reg [(10-1):0] mux_address_b;

dpram_10_1024_32 dpram1(
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

endmodulemodule dpram_10_1024_32 (
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
input [(10-1):0] address_a;
input [(10-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[1024-1:0];

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