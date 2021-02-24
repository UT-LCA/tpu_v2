
`define DWIDTH 32
`define AWIDTH 10
`define MEM_SIZE 1024

///////////////////////////////////////////////////////////
//MAT_MUL_SIZE refers to the dimension of the matrix
//multiplication performed by the matmul unit. The value 
//8 means it will multiply an 8x8 matrix with an 8x8 matrix.
///////////////////////////////////////////////////////////
//IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP
///////////////////////////////////////////////////////////
//MAT_MUL_SIZE should be equal to number of lanes in the vector processor
///////////////////////////////////////////////////////////
//IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP
///////////////////////////////////////////////////////////

`define MASK_WIDTH 8
`define LOG2_MAT_MUL_SIZE 3

`define BB_MAT_MUL_SIZE `MAT_MUL_SIZE
`define NUM_CYCLES_IN_MAC 3
`define MEM_ACCESS_LATENCY 1
`define REG_DATAWIDTH 32
`define REG_ADDRWIDTH 8
`define ADDR_STRIDE_WIDTH 16
`define MAX_BITS_POOL 3

`define PIPE_STAGES_MATMUL 29

`include "matmul_8x8.v"

module matmul_unit(
 clk,
 resetn,
 activate,
 en,
 squash,
 stall,
 a_data,
 b_data,
 validity_mask_a_rows,
 validity_mask_a_cols,
 validity_mask_b_rows,
 validity_mask_b_cols,
 c_data, 
 vmask,
 in_dst,
 in_dst_we,
 out_dst,
 out_dst_we,
 out_dst_mask
);

parameter REGIDWIDTH=8;
parameter PIPE_STAGES_MATMUL=29;
parameter NUMLANES=8;

 input clk;
 input resetn;
 input activate;
 input [PIPE_STAGES_MATMUL:1] en;  //Enable for each pipestage
 input [PIPE_STAGES_MATMUL:1] squash;  //Squash for each pipestage
 output stall;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data;
 input [`MASK_WIDTH-1:0] validity_mask_a_rows;
 input [`MASK_WIDTH-1:0] validity_mask_a_cols;
 input [`MASK_WIDTH-1:0] validity_mask_b_rows;
 input [`MASK_WIDTH-1:0] validity_mask_b_cols;

 output [`MAT_MUL_SIZE*`DWIDTH-1:0] c_data;
 input [NUMLANES-1:0] vmask;

 input    [REGIDWIDTH-1:0] in_dst;
 input                     in_dst_we;
 output [PIPE_STAGES_MATMUL*REGIDWIDTH-1:0] out_dst;
 output            [PIPE_STAGES_MATMUL-1:0] out_dst_we;
 output   [PIPE_STAGES_MATMUL*NUMLANES-1:0] out_dst_mask;

wire [PIPE_STAGES_MATMUL:1] ctrl_activate;
wire [PIPE_STAGES_MATMUL:1] squash_activatepipe_NC;
pipe #(1,PIPE_STAGES_MATMUL-1) activatepipe (
    .d(activate),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(squash_activatepipe_NC),
    .q(ctrl_activate));

wire in_progress;
assign stall=in_progress;

//The actual matrix multiplier block. This is an abridged
//version of the matmul block used in the TPU v1 design.
matmul_8x8 mat(
    .clk(clk),
    .reset(~resetn),
    .pe_reset(~resetn),
    .start(activate),
    .in_progress(in_progress),
    .done(done_NC),
    .a_data(a_data),
    .b_data(b_data),
    .c_data_out(c_data), 
    .c_data_available(c_data_available),
    .validity_mask_a_rows(validity_mask_a_rows),
    .validity_mask_a_cols(validity_mask_a_cols),
    .validity_mask_b_rows(validity_mask_b_rows),
    .validity_mask_b_cols(validity_mask_b_cols)
);

wire [PIPE_STAGES_MATMUL:1] squash_dstpipe_NC;
pipe #(REGIDWIDTH,PIPE_STAGES_MATMUL-1) dstpipe (
    .d(in_dst ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[PIPE_STAGES_MATMUL-1:1] & {1'b1,{(PIPE_STAGES_MATMUL-2){~stall}}} ),
    .squash(squash_dstpipe_NC),
    .q(out_dst));

pipe #(1,PIPE_STAGES_MATMUL-1) dstwepipe (
    .d(in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[PIPE_STAGES_MATMUL-1:1] & {1'b1,{(PIPE_STAGES_MATMUL-2){~stall}}} ),
    .squash(squash[PIPE_STAGES_MATMUL-1:1]),
    .q(out_dst_we));

wire [PIPE_STAGES_MATMUL:1] squash_dstmaskpipe_NC;
pipe #(NUMLANES,PIPE_STAGES_MATMUL-1) dstmaskpipe (
    .d(vmask ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[PIPE_STAGES_MATMUL-1:1] & {1'b1,{(PIPE_STAGES_MATMUL-2){~stall}}} ),
    .squash(squash_dstmaskpipe_NC),
    .q(out_dst_mask));

endmodule