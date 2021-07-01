module vmem_local (
clk,
resetn,
en,

op,
address_a,
stride_val_a,
offset_a,
data_a,
out_a,
last_subvector,

address_b,
rden_b,
wren_b,
data_b,
out_b
);


parameter NUMLANES = 8;
parameter DATAWORDSIZE = 16;
parameter VCWIDTH = 32;  
parameter MEMDEPTH = 2048; 
parameter LOGMEMDEPTH = $clog2(MEMDEPTH);

parameter MEM_IDLE = 1'b0;
parameter MEM_STRIDE = 1'b1;

input clk;
input resetn;
input [LOGMEMDEPTH-1:0] address_a;
input [NUMLANES*LOGMEMDEPTH-1:0] address_b;
input en;
input [6:0] op;
input rden_b;
input wren_b;
input last_subvector;
input [NUMLANES*DATAWORDSIZE-1:0] data_a;
input [NUMLANES*DATAWORDSIZE-1:0] data_b;
output [NUMLANES*DATAWORDSIZE-1:0] out_a;
output [NUMLANES*DATAWORDSIZE-1:0] out_b;
input [VCWIDTH-1:0] stride_val_a;
input [NUMLANES*16-1:0] offset_a;

wire stride_req_a;
wire [VCWIDTH-1:0] stride;
wire index_req_a;
reg wren_a,rden_a; 
reg req_we_a;
reg req_we_b;
reg [NUMLANES*LOGMEMDEPTH-1:0] reg_address_a;
reg [NUMLANES*LOGMEMDEPTH-1:0] reg_address_b;

wire  [1:0]  op_pattern;       // 0-Unit, 1-Strided, 2-Indexed
wire  [1:0]  op_size;          // 0-byte, 1-16bits, 2-32bits, 3-64bits
wire         op_signed;        // 0-unsigned, 1-signed
wire         op_we;            // 0-load, 1-store
wire         op_memop;         // 1-memory op, 0-vector shift op
genvar  g_mem;
reg [31:0] i;

reg [NUMLANES-1:0] g_en;

assign {op_memop,op_pattern,op_size,op_signed,op_we}=op;
//assign rden_a = (op_memop & (~op_we))? 1'b1:1'b0;
//assign wren_a = (op_memop & op_we)? 1'b1:1'b0;
assign stride = (op_pattern == 2'b01)? stride_val_a: 1;
assign stride_req_a = (~op_pattern[1] & op_memop)? 1'b1:1'b0;
assign index_req_a =  (op_pattern[1] & op_memop)? 1'b1: 1'b0;

//Lane 0 Memory address space : 0,8,16,24,...,(MEMDEPTH*8-1)
//Lane 1 Memory address space : 0+1,8+1,16+1,24+1,...,(MEMDEPTH*8-1+1)

reg last_subvector_q;

always@(posedge clk)begin 
  if(!resetn)
    last_subvector_q <= 1'b0;
  else
    last_subvector_q <= last_subvector;
end

wire mem_op_valid;
assign mem_op_valid = ~(last_subvector & last_subvector_q); 

always@(*)begin
  if(op_memop & (~op_we))
    rden_a = 1'b1;
  else
    rden_a = 1'b0;

  if(op_memop & op_we & mem_op_valid)
    wren_a = 1'b1;
  else
    wren_a = 1'b0;

  for(i=0; i< NUMLANES; i=i+1)begin
    if(i[2:0] == address_a[2:0])
      g_en[i] = 1'b1;
    else
      g_en[i] = 1'b0;
    
    reg_address_a[i*LOGMEMDEPTH +: LOGMEMDEPTH] = address_a + i * stride;
    reg_address_b[i*LOGMEMDEPTH +: LOGMEMDEPTH] = address_b[i*LOGMEMDEPTH +: LOGMEMDEPTH];
    if(index_req_a)begin
        reg_address_a[i*LOGMEMDEPTH +: LOGMEMDEPTH] = address_a + offset_a[i*16+:16];
    end
  end
end

generate 
    for(g_mem =0; g_mem < NUMLANES ; g_mem = g_mem+1 )begin:gen_memperlane
        per_lane_mem_wrapper #(.AWIDTH(LOGMEMDEPTH),.NUM_WORDS(MEMDEPTH), .DWIDTH(DATAWORDSIZE)) inst_mem(
	    .clk(clk),
            .resetn(resetn),
	    .address_a(reg_address_a[g_mem*LOGMEMDEPTH +: LOGMEMDEPTH]),
	    .address_b(reg_address_b[g_mem*LOGMEMDEPTH +: LOGMEMDEPTH]),
            .rden_a(rden_a & en),
            .rden_b(rden_b),
	    .wren_a(wren_a & en),
	    .wren_b(wren_b),
	    .data_a(data_a[g_mem*DATAWORDSIZE +: DATAWORDSIZE]),
	    .data_b(data_b[g_mem*DATAWORDSIZE +: DATAWORDSIZE]),
	    .out_a(out_a[g_mem*DATAWORDSIZE +: DATAWORDSIZE]),
	    .out_b(out_b[g_mem*DATAWORDSIZE +: DATAWORDSIZE])
        );
    end
endgenerate

endmodule
