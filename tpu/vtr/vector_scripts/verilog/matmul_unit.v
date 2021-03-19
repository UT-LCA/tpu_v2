
module matmul_unit_10_33_4(
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

 input clk;
 input resetn;
 input activate;
 input [33:1] en;  //Enable for each pipestage
 input [33:1] squash;  //Squash for each pipestage
 output stall;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data;
 input [`MASK_WIDTH-1:0] validity_mask_a_rows;
 input [`MASK_WIDTH-1:0] validity_mask_a_cols;
 input [`MASK_WIDTH-1:0] validity_mask_b_rows;
 input [`MASK_WIDTH-1:0] validity_mask_b_cols;

 output [`MAT_MUL_SIZE*`DWIDTH-1:0] c_data;
 input [4-1:0] vmask;

 input    [10-1:0] in_dst;
 input                     in_dst_we;
 output [33*10-1:0] out_dst;
 output            [33-1:0] out_dst_we;
 output   [33*4-1:0] out_dst_mask;

wire [33:1] ctrl_activate;
wire [33:1] squash_activatepipe_NC;
pipe_1_32 activatepipe (
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

wire [33:1] squash_dstpipe_NC;
pipe_10_32 dstpipe (
    .d(in_dst ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[33-1:1] & {1'b1,{(33-2){~stall}}} ),
    .squash(squash_dstpipe_NC),
    .q(out_dst));

pipe_1_32 dstwepipe (
    .d(in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[33-1:1] & {1'b1,{(33-2){~stall}}} ),
    .squash(squash[33-1:1]),
    .q(out_dst_we));

wire [33:1] squash_dstmaskpipe_NC;
pipe_4_32 dstmaskpipe (
    .d(vmask ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[33-1:1] & {1'b1,{(33-2){~stall}}} ),
    .squash(squash_dstmaskpipe_NC),
    .q(out_dst_mask));
endmodule