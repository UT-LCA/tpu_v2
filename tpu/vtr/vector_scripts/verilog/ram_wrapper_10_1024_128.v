module ram_wrapper_9_512_128 (
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
input [9-1:0] address_a;
input [9-1:0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [128-1:0] data_a;
input [128-1:0] data_b;
output [128-1:0] out_a;
output [128-1:0] out_b;

reg [9-1:0] q_address_a;
reg [9-1:0] q_address_b;
reg [9-1:0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_9_512_128 dpram1(
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
