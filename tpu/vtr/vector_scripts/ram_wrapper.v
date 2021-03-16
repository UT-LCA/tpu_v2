module ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5.0-1):0] q_address_a;
reg [(5.0-1):0] q_address_b;
reg [(5.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5.0_32_32 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5.0-1):0] q_address_a;
reg [(5.0-1):0] q_address_b;
reg [(5.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5.0_32_32 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5.0-1):0] q_address_a;
reg [(5.0-1):0] q_address_b;
reg [(5.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5.0_32_32 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5.0-1):0] q_address_a;
reg [(5.0-1):0] q_address_b;
reg [(5.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5.0_32_32 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5.0-1):0] q_address_a;
reg [(5.0-1):0] q_address_b;
reg [(5.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5.0_32_32 dpram1(
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

endmodulemodule ram_wrapper_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output [(128-1):0] out_a;
output [(128-1):0] out_b;

reg [(8-1):0] q_address_a;
reg [(8-1):0] q_address_b;
reg [(8-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_8_256_128 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5.0-1):0] q_address_a;
reg [(5.0-1):0] q_address_b;
reg [(5.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5.0_32_32 dpram1(
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

endmodulemodule ram_wrapper_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output [(128-1):0] out_a;
output [(128-1):0] out_b;

reg [(8-1):0] q_address_a;
reg [(8-1):0] q_address_b;
reg [(8-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_8_256_128 dpram1(
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

endmodulemodule ram_wrapper_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output [(8-1):0] out_a;
output [(8-1):0] out_b;

reg [(7-1):0] q_address_a;
reg [(7-1):0] q_address_b;
reg [(7-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_7_128_8 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5.0-1):0] q_address_a;
reg [(5.0-1):0] q_address_b;
reg [(5.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5.0_32_32 dpram1(
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

endmodulemodule ram_wrapper_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output [(128-1):0] out_a;
output [(128-1):0] out_b;

reg [(8-1):0] q_address_a;
reg [(8-1):0] q_address_b;
reg [(8-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_8_256_128 dpram1(
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

endmodulemodule ram_wrapper_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output [(8-1):0] out_a;
output [(8-1):0] out_b;

reg [(7-1):0] q_address_a;
reg [(7-1):0] q_address_b;
reg [(7-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_7_128_8 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5.0-1):0] q_address_a;
reg [(5.0-1):0] q_address_b;
reg [(5.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5.0_32_32 dpram1(
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

endmodulemodule ram_wrapper_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output [(128-1):0] out_a;
output [(128-1):0] out_b;

reg [(8-1):0] q_address_a;
reg [(8-1):0] q_address_b;
reg [(8-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_8_256_128 dpram1(
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

endmodulemodule ram_wrapper_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output [(8-1):0] out_a;
output [(8-1):0] out_b;

reg [(7-1):0] q_address_a;
reg [(7-1):0] q_address_b;
reg [(7-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_7_128_8 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5.0-1):0] q_address_a;
reg [(5.0-1):0] q_address_b;
reg [(5.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5.0_32_32 dpram1(
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

endmodulemodule ram_wrapper_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output [(128-1):0] out_a;
output [(128-1):0] out_b;

reg [(8-1):0] q_address_a;
reg [(8-1):0] q_address_b;
reg [(8-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_8_256_128 dpram1(
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

endmodulemodule ram_wrapper_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output [(8-1):0] out_a;
output [(8-1):0] out_b;

reg [(7-1):0] q_address_a;
reg [(7-1):0] q_address_b;
reg [(7-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_7_128_8 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5.0-1):0] q_address_a;
reg [(5.0-1):0] q_address_b;
reg [(5.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5.0_32_32 dpram1(
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

endmodulemodule ram_wrapper_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output [(128-1):0] out_a;
output [(128-1):0] out_b;

reg [(8-1):0] q_address_a;
reg [(8-1):0] q_address_b;
reg [(8-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_8_256_128 dpram1(
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

endmodulemodule ram_wrapper_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output [(8-1):0] out_a;
output [(8-1):0] out_b;

reg [(7-1):0] q_address_a;
reg [(7-1):0] q_address_b;
reg [(7-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_7_128_8 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5.0-1):0] q_address_a;
reg [(5.0-1):0] q_address_b;
reg [(5.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5.0_32_32 dpram1(
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

endmodulemodule ram_wrapper_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output [(128-1):0] out_a;
output [(128-1):0] out_b;

reg [(8-1):0] q_address_a;
reg [(8-1):0] q_address_b;
reg [(8-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_8_256_128 dpram1(
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

endmodulemodule ram_wrapper_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output [(8-1):0] out_a;
output [(8-1):0] out_b;

reg [(7-1):0] q_address_a;
reg [(7-1):0] q_address_b;
reg [(7-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_7_128_8 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5.0-1):0] q_address_a;
reg [(5.0-1):0] q_address_b;
reg [(5.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5.0_32_32 dpram1(
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

endmodulemodule ram_wrapper_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output [(128-1):0] out_a;
output [(128-1):0] out_b;

reg [(8-1):0] q_address_a;
reg [(8-1):0] q_address_b;
reg [(8-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_8_256_128 dpram1(
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

endmodulemodule ram_wrapper_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output [(8-1):0] out_a;
output [(8-1):0] out_b;

reg [(7-1):0] q_address_a;
reg [(7-1):0] q_address_b;
reg [(7-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_7_128_8 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5.0-1):0] q_address_a;
reg [(5.0-1):0] q_address_b;
reg [(5.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5.0_32_32 dpram1(
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

endmodulemodule ram_wrapper_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output [(128-1):0] out_a;
output [(128-1):0] out_b;

reg [(8-1):0] q_address_a;
reg [(8-1):0] q_address_b;
reg [(8-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_8_256_128 dpram1(
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

endmodulemodule ram_wrapper_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output [(8-1):0] out_a;
output [(8-1):0] out_b;

reg [(7-1):0] q_address_a;
reg [(7-1):0] q_address_b;
reg [(7-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_7_128_8 dpram1(
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

endmodulemodule ram_wrapper_5_32_32 (
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
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5_32_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_3.0_8_32 (
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
input [(3.0-1):0] address_a;
input [(3.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(3.0-1):0] q_address_a;
reg [(3.0-1):0] q_address_b;
reg [(3.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3.0_8_32 dpram1(
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

endmodulemodule ram_wrapper_4.0_16_32 (
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
input [(4.0-1):0] address_a;
input [(4.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(4.0-1):0] q_address_a;
reg [(4.0-1):0] q_address_b;
reg [(4.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4.0_16_32 dpram1(
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

endmodulemodule ram_wrapper_5.0_32_32 (
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
input [(5.0-1):0] address_a;
input [(5.0-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5.0-1):0] q_address_a;
reg [(5.0-1):0] q_address_b;
reg [(5.0-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_5.0_32_32 dpram1(
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

endmodulemodule ram_wrapper_8_256_128 (
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
input [(8-1):0] address_a;
input [(8-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(128-1):0] data_a;
input [(128-1):0] data_b;
output [(128-1):0] out_a;
output [(128-1):0] out_b;

reg [(8-1):0] q_address_a;
reg [(8-1):0] q_address_b;
reg [(8-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_8_256_128 dpram1(
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

endmodulemodule ram_wrapper_7_128_8 (
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
input [(7-1):0] address_a;
input [(7-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(8-1):0] data_a;
input [(8-1):0] data_b;
output [(8-1):0] out_a;
output [(8-1):0] out_b;

reg [(7-1):0] q_address_a;
reg [(7-1):0] q_address_b;
reg [(7-1):0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_7_128_8 dpram1(
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