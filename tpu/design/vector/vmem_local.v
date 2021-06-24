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

address_b,
rden_b,
wren_b,
data_b,
out_b
);


parameter NUMLANES = 8;
parameter DATAWORDSIZE = 32;
parameter VCWIDTH = 32;  
parameter MEMDEPTH = 2048; 
parameter LOGMEMDEPTH = $clog2(MEMDEPTH);

parameter MEM_IDLE = 1'b0;
parameter MEM_STRIDE = 1'b1;

input clk;
input resetn;
input [LOGMEMDEPTH-1:0] address_a;
input [LOGMEMDEPTH-1:0] address_b;
input [NUMLANES-1:0] en;
input [6:0] op;
input rden_b;
input wren_b;
input [NUMLANES*DATAWORDSIZE-1:0] data_a;
input [NUMLANES*DATAWORDSIZE-1:0] data_b;
output [NUMLANES*DATAWORDSIZE-1:0] out_a;
output [NUMLANES*DATAWORDSIZE-1:0] out_b;
input [VCWIDTH-1:0] stride_val_a;
input [NUMLANES*8-1:0] offset_a;

wire stride_req_a;
wire index_req_a;
wire rden_a,wren_a;
 
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


assign rden_a = (op_memop & (~op_we))? 1'b1:1'b0;
assign wren_a = (op_memop & op_we)? 1'b1:1'b0;
assign {op_memop,op_pattern,op_size,op_signed,op_we}=op;

assign stride_val_a = (op_pattern == 2'b01)? stride_val_a: 1;
assign stride_req_a = (~op_pattern[1] & op_memop)? 1'b1:1'b0;
assign index_req_a =  (op_pattern[1] & op_memop)? 1'b1: 1'b0;

//Lane 0 Memory address space : 0,8,16,24,...,(MEMDEPTH*8-1)
//Lane 1 Memory address space : 0+1,8+1,16+1,24+1,...,(MEMDEPTH*8-1+1)
 
always@(*)begin
  for(i=0; i< NUMLANES; i=i+1)begin
    reg_address_a[i*LOGMEMDEPTH +: LOGMEMDEPTH] = address_a;
    reg_address_b[i*LOGMEMDEPTH +: LOGMEMDEPTH] = address_b;
    if(stride_req_a)begin
        reg_address_a[i*LOGMEMDEPTH +: LOGMEMDEPTH] = address_a + i * stride_val_a;
    end
    else if(index_req_a)begin
        reg_address_a[i*LOGMEMDEPTH +: LOGMEMDEPTH] = address_a + offset_a[i*8+:8];
    end
  end
end

generate 
    for(g_mem =0; g_mem < NUMLANES ; g_mem = g_mem+1 )begin:gen_memperlane
        ram_wrapper #(.AWIDTH(LOGMEMDEPTH),.NUM_WORDS(MEMDEPTH), .DWIDTH(DATAWORDSIZE)) inst_mem(
	    .clk(clk),
	    .address_a(reg_address_a[g_mem*LOGMEMDEPTH +: LOGMEMDEPTH]),
	    .address_b(reg_address_b[g_mem*LOGMEMDEPTH +: LOGMEMDEPTH]),
            .rden_a(rden_a),
            .rden_b(rden_b),
	    .wren_a(wren_a & en[g_mem]),
	    .wren_b(wren_b & en[g_mem]),
	    .data_a(data_a[g_mem*DATAWORDSIZE +: DATAWORDSIZE]),
	    .data_b(data_b[g_mem*DATAWORDSIZE +: DATAWORDSIZE]),
	    .out_a(out_a[g_mem*DATAWORDSIZE +: DATAWORDSIZE]),
	    .out_b(out_b[g_mem*DATAWORDSIZE +: DATAWORDSIZE])
        );
    end
endgenerate

endmodule
