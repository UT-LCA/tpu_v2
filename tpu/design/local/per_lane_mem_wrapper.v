module per_lane_mem_wrapper (
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

parameter AWIDTH=10;
parameter NUM_WORDS=1024;
parameter DWIDTH=32;

input clk;
input resetn;
input [(AWIDTH-1):0] address_a;
input [(AWIDTH-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(DWIDTH-1):0] data_a;
input [(DWIDTH-1):0] data_b;
output [(DWIDTH-1):0] out_a;
output [(DWIDTH-1):0] out_b;

reg [(AWIDTH-1):0] q_address_a;
reg [(AWIDTH-1):0] q_address_b;
reg [(AWIDTH-1):0] mux_address_b;
reg [(AWIDTH-1):0] mux_address_a;

dpram inst_dpram(
    .clk(clk),
    .address_a(mux_address_a),
    .address_b(mux_address_b),
    .wren_a(wren_a),
    .wren_b(wren_b),
    .data_a(data_a),
    .data_b(data_b),
    .out_a(out_a),
    .out_b(out_b)
);
defparam
    inst_dpram.AWIDTH=AWIDTH,
    inst_dpram.NUM_WORDS=NUM_WORDS,
    inst_dpram.DWIDTH=DWIDTH;

always@(posedge clk)begin
   if(!resetn)begin
     q_address_a <= 'h0;
     q_address_b <= 'h0;
   end
   else begin
     if(rden_a | wren_a)
       q_address_a <= address_a;
     if(rden_b | wren_b)
       q_address_b <= address_b;
   end
end

always@(*)begin

  if(rden_a | wren_a)   
    mux_address_a = address_a;
  else
    mux_address_a = q_address_a; 

  if(rden_b | wren_b)   
    mux_address_b = address_b;
  else
    mux_address_b = q_address_b;
 
end

endmodule
