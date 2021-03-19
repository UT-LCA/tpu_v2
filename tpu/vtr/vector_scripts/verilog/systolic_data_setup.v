//////////////////////////////////////////////////////////////////////////
// Systolic data setup
//////////////////////////////////////////////////////////////////////////
module systolic_data_setup(
clk,
reset,
matmul_op_in_progress,
a_data,
b_data,
clk_cnt,
a0_data,
b0_data,
a1_data_delayed_1,
b1_data_delayed_1,
a2_data_delayed_2,
b2_data_delayed_2,
a3_data_delayed_3,
b3_data_delayed_3,
a4_data_delayed_4,
b4_data_delayed_4,
a5_data_delayed_5,
b5_data_delayed_5,
a6_data_delayed_6,
b6_data_delayed_6,
a7_data_delayed_7,
b7_data_delayed_7,

validity_mask_a_rows,
validity_mask_a_cols,
validity_mask_b_rows,
validity_mask_b_cols
);

input clk;
input reset;
input matmul_op_in_progress;
input [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data;
input [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data;
input [7:0] clk_cnt;
output [`DWIDTH-1:0] a0_data;
output [`DWIDTH-1:0] b0_data;
output [`DWIDTH-1:0] a1_data_delayed_1;
output [`DWIDTH-1:0] b1_data_delayed_1;
output [`DWIDTH-1:0] a2_data_delayed_2;
output [`DWIDTH-1:0] b2_data_delayed_2;
output [`DWIDTH-1:0] a3_data_delayed_3;
output [`DWIDTH-1:0] b3_data_delayed_3;
output [`DWIDTH-1:0] a4_data_delayed_4;
output [`DWIDTH-1:0] b4_data_delayed_4;
output [`DWIDTH-1:0] a5_data_delayed_5;
output [`DWIDTH-1:0] b5_data_delayed_5;
output [`DWIDTH-1:0] a6_data_delayed_6;
output [`DWIDTH-1:0] b6_data_delayed_6;
output [`DWIDTH-1:0] a7_data_delayed_7;
output [`DWIDTH-1:0] b7_data_delayed_7;

input [`MASK_WIDTH-1:0] validity_mask_a_rows;
input [`MASK_WIDTH-1:0] validity_mask_a_cols;
input [`MASK_WIDTH-1:0] validity_mask_b_rows;
input [`MASK_WIDTH-1:0] validity_mask_b_cols;

wire [`DWIDTH-1:0] a0_data;
wire [`DWIDTH-1:0] a1_data;
wire [`DWIDTH-1:0] a2_data;
wire [`DWIDTH-1:0] a3_data;
wire [`DWIDTH-1:0] a4_data;
wire [`DWIDTH-1:0] a5_data;
wire [`DWIDTH-1:0] a6_data;
wire [`DWIDTH-1:0] a7_data;
wire [`DWIDTH-1:0] b0_data;
wire [`DWIDTH-1:0] b1_data;
wire [`DWIDTH-1:0] b2_data;
wire [`DWIDTH-1:0] b3_data;
wire [`DWIDTH-1:0] b4_data;
wire [`DWIDTH-1:0] b5_data;
wire [`DWIDTH-1:0] b6_data;
wire [`DWIDTH-1:0] b7_data;

wire [7:0] a_mem_access_counter;
assign a_mem_access_counter = ((clk_cnt>=8) ? 0 : (matmul_op_in_progress ? (clk_cnt+1) : 0));

wire a_data_valid; //flag that tells whether the data from memory is valid
assign a_data_valid = 
     ((validity_mask_a_cols[0]==1'b0 && a_mem_access_counter==1) ||
      (validity_mask_a_cols[1]==1'b0 && a_mem_access_counter==2) ||
      (validity_mask_a_cols[2]==1'b0 && a_mem_access_counter==3) ||
      (validity_mask_a_cols[3]==1'b0 && a_mem_access_counter==4) ||
      (validity_mask_a_cols[4]==1'b0 && a_mem_access_counter==5) ||
      (validity_mask_a_cols[5]==1'b0 && a_mem_access_counter==6) ||
      (validity_mask_a_cols[6]==1'b0 && a_mem_access_counter==7) ||
      (validity_mask_a_cols[7]==1'b0 && a_mem_access_counter==8)) ?
    
    1'b0 : (a_mem_access_counter >= `MEM_ACCESS_LATENCY);

//////////////////////////////////////////////////////////////////////////
// Logic to delay certain parts of the data received from BRAM A (systolic data setup)
//////////////////////////////////////////////////////////////////////////
assign a0_data = a_data[1*`DWIDTH-1:0*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[0]}};
assign a1_data = a_data[2*`DWIDTH-1:1*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[1]}};
assign a2_data = a_data[3*`DWIDTH-1:2*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[2]}};
assign a3_data = a_data[4*`DWIDTH-1:3*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[3]}};
assign a4_data = a_data[5*`DWIDTH-1:4*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[4]}};
assign a5_data = a_data[6*`DWIDTH-1:5*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[5]}};
assign a6_data = a_data[7*`DWIDTH-1:6*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[6]}};
assign a7_data = a_data[8*`DWIDTH-1:7*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[7]}};

reg [`DWIDTH-1:0] a1_data_delayed_1;
reg [`DWIDTH-1:0] a2_data_delayed_1;
reg [`DWIDTH-1:0] a2_data_delayed_2;
reg [`DWIDTH-1:0] a3_data_delayed_1;
reg [`DWIDTH-1:0] a3_data_delayed_2;
reg [`DWIDTH-1:0] a3_data_delayed_3;
reg [`DWIDTH-1:0] a4_data_delayed_1;
reg [`DWIDTH-1:0] a4_data_delayed_2;
reg [`DWIDTH-1:0] a4_data_delayed_3;
reg [`DWIDTH-1:0] a4_data_delayed_4;
reg [`DWIDTH-1:0] a5_data_delayed_1;
reg [`DWIDTH-1:0] a5_data_delayed_2;
reg [`DWIDTH-1:0] a5_data_delayed_3;
reg [`DWIDTH-1:0] a5_data_delayed_4;
reg [`DWIDTH-1:0] a5_data_delayed_5;
reg [`DWIDTH-1:0] a6_data_delayed_1;
reg [`DWIDTH-1:0] a6_data_delayed_2;
reg [`DWIDTH-1:0] a6_data_delayed_3;
reg [`DWIDTH-1:0] a6_data_delayed_4;
reg [`DWIDTH-1:0] a6_data_delayed_5;
reg [`DWIDTH-1:0] a6_data_delayed_6;
reg [`DWIDTH-1:0] a7_data_delayed_1;
reg [`DWIDTH-1:0] a7_data_delayed_2;
reg [`DWIDTH-1:0] a7_data_delayed_3;
reg [`DWIDTH-1:0] a7_data_delayed_4;
reg [`DWIDTH-1:0] a7_data_delayed_5;
reg [`DWIDTH-1:0] a7_data_delayed_6;
reg [`DWIDTH-1:0] a7_data_delayed_7;


always @(posedge clk) begin
  if (reset || ~matmul_op_in_progress) begin
    a1_data_delayed_1 <= 0;
    a2_data_delayed_1 <= 0;
    a2_data_delayed_2 <= 0;
    a3_data_delayed_1 <= 0;
    a3_data_delayed_2 <= 0;
    a3_data_delayed_3 <= 0;
    a4_data_delayed_1 <= 0;
    a4_data_delayed_2 <= 0;
    a4_data_delayed_3 <= 0;
    a4_data_delayed_4 <= 0;
    a5_data_delayed_1 <= 0;
    a5_data_delayed_2 <= 0;
    a5_data_delayed_3 <= 0;
    a5_data_delayed_4 <= 0;
    a5_data_delayed_5 <= 0;
    a6_data_delayed_1 <= 0;
    a6_data_delayed_2 <= 0;
    a6_data_delayed_3 <= 0;
    a6_data_delayed_4 <= 0;
    a6_data_delayed_5 <= 0;
    a6_data_delayed_6 <= 0;
    a7_data_delayed_1 <= 0;
    a7_data_delayed_2 <= 0;
    a7_data_delayed_3 <= 0;
    a7_data_delayed_4 <= 0;
    a7_data_delayed_5 <= 0;
    a7_data_delayed_6 <= 0;
    a7_data_delayed_7 <= 0;

  end
  else begin
  a1_data_delayed_1 <= a1_data;
  a2_data_delayed_1 <= a2_data;
  a3_data_delayed_1 <= a3_data;
  a4_data_delayed_1 <= a4_data;
  a5_data_delayed_1 <= a5_data;
  a6_data_delayed_1 <= a6_data;
  a7_data_delayed_1 <= a7_data;
  a2_data_delayed_2 <= a2_data_delayed_1;
  a3_data_delayed_2 <= a3_data_delayed_1;
  a3_data_delayed_3 <= a3_data_delayed_2;
  a4_data_delayed_2 <= a4_data_delayed_1;
  a4_data_delayed_3 <= a4_data_delayed_2;
  a4_data_delayed_4 <= a4_data_delayed_3;
  a5_data_delayed_2 <= a5_data_delayed_1;
  a5_data_delayed_3 <= a5_data_delayed_2;
  a5_data_delayed_4 <= a5_data_delayed_3;
  a5_data_delayed_5 <= a5_data_delayed_4;
  a6_data_delayed_2 <= a6_data_delayed_1;
  a6_data_delayed_3 <= a6_data_delayed_2;
  a6_data_delayed_4 <= a6_data_delayed_3;
  a6_data_delayed_5 <= a6_data_delayed_4;
  a6_data_delayed_6 <= a6_data_delayed_5;
  a7_data_delayed_2 <= a7_data_delayed_1;
  a7_data_delayed_3 <= a7_data_delayed_2;
  a7_data_delayed_4 <= a7_data_delayed_3;
  a7_data_delayed_5 <= a7_data_delayed_4;
  a7_data_delayed_6 <= a7_data_delayed_5;
  a7_data_delayed_7 <= a7_data_delayed_6;
 
  end
end

wire [7:0] b_mem_access_counter;
assign b_mem_access_counter = ((clk_cnt>=8) ? 0 : (matmul_op_in_progress ? (clk_cnt+1) : 0));

wire b_data_valid; //flag that tells whether the data from memory is valid
assign b_data_valid = 
     ((validity_mask_b_rows[0]==1'b0 && b_mem_access_counter==1) ||
      (validity_mask_b_rows[1]==1'b0 && b_mem_access_counter==2) ||
      (validity_mask_b_rows[2]==1'b0 && b_mem_access_counter==3) ||
      (validity_mask_b_rows[3]==1'b0 && b_mem_access_counter==4) ||
      (validity_mask_b_rows[4]==1'b0 && b_mem_access_counter==5) ||
      (validity_mask_b_rows[5]==1'b0 && b_mem_access_counter==6) ||
      (validity_mask_b_rows[6]==1'b0 && b_mem_access_counter==7) ||
      (validity_mask_b_rows[7]==1'b0 && b_mem_access_counter==8)) ?
    
        1'b0 : (b_mem_access_counter >= `MEM_ACCESS_LATENCY);

//////////////////////////////////////////////////////////////////////////
// Logic to delay certain parts of the data received from BRAM B (systolic data setup)
//////////////////////////////////////////////////////////////////////////
assign b0_data = b_data[1*`DWIDTH-1:0*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[0]}};
assign b1_data = b_data[2*`DWIDTH-1:1*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[1]}};
assign b2_data = b_data[3*`DWIDTH-1:2*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[2]}};
assign b3_data = b_data[4*`DWIDTH-1:3*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[3]}};
assign b4_data = b_data[5*`DWIDTH-1:4*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[4]}};
assign b5_data = b_data[6*`DWIDTH-1:5*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[5]}};
assign b6_data = b_data[7*`DWIDTH-1:6*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[6]}};
assign b7_data = b_data[8*`DWIDTH-1:7*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[7]}};

reg [`DWIDTH-1:0] b1_data_delayed_1;
reg [`DWIDTH-1:0] b2_data_delayed_1;
reg [`DWIDTH-1:0] b2_data_delayed_2;
reg [`DWIDTH-1:0] b3_data_delayed_1;
reg [`DWIDTH-1:0] b3_data_delayed_2;
reg [`DWIDTH-1:0] b3_data_delayed_3;
reg [`DWIDTH-1:0] b4_data_delayed_1;
reg [`DWIDTH-1:0] b4_data_delayed_2;
reg [`DWIDTH-1:0] b4_data_delayed_3;
reg [`DWIDTH-1:0] b4_data_delayed_4;
reg [`DWIDTH-1:0] b5_data_delayed_1;
reg [`DWIDTH-1:0] b5_data_delayed_2;
reg [`DWIDTH-1:0] b5_data_delayed_3;
reg [`DWIDTH-1:0] b5_data_delayed_4;
reg [`DWIDTH-1:0] b5_data_delayed_5;
reg [`DWIDTH-1:0] b6_data_delayed_1;
reg [`DWIDTH-1:0] b6_data_delayed_2;
reg [`DWIDTH-1:0] b6_data_delayed_3;
reg [`DWIDTH-1:0] b6_data_delayed_4;
reg [`DWIDTH-1:0] b6_data_delayed_5;
reg [`DWIDTH-1:0] b6_data_delayed_6;
reg [`DWIDTH-1:0] b7_data_delayed_1;
reg [`DWIDTH-1:0] b7_data_delayed_2;
reg [`DWIDTH-1:0] b7_data_delayed_3;
reg [`DWIDTH-1:0] b7_data_delayed_4;
reg [`DWIDTH-1:0] b7_data_delayed_5;
reg [`DWIDTH-1:0] b7_data_delayed_6;
reg [`DWIDTH-1:0] b7_data_delayed_7;


always @(posedge clk) begin
  if (reset || ~matmul_op_in_progress) begin
    b1_data_delayed_1 <= 0;
    b2_data_delayed_1 <= 0;
    b2_data_delayed_2 <= 0;
    b3_data_delayed_1 <= 0;
    b3_data_delayed_2 <= 0;
    b3_data_delayed_3 <= 0;
    b4_data_delayed_1 <= 0;
    b4_data_delayed_2 <= 0;
    b4_data_delayed_3 <= 0;
    b4_data_delayed_4 <= 0;
    b5_data_delayed_1 <= 0;
    b5_data_delayed_2 <= 0;
    b5_data_delayed_3 <= 0;
    b5_data_delayed_4 <= 0;
    b5_data_delayed_5 <= 0;
    b6_data_delayed_1 <= 0;
    b6_data_delayed_2 <= 0;
    b6_data_delayed_3 <= 0;
    b6_data_delayed_4 <= 0;
    b6_data_delayed_5 <= 0;
    b6_data_delayed_6 <= 0;
    b7_data_delayed_1 <= 0;
    b7_data_delayed_2 <= 0;
    b7_data_delayed_3 <= 0;
    b7_data_delayed_4 <= 0;
    b7_data_delayed_5 <= 0;
    b7_data_delayed_6 <= 0;
    b7_data_delayed_7 <= 0;

  end
  else begin
  b1_data_delayed_1 <= b1_data;
  b2_data_delayed_1 <= b2_data;
  b3_data_delayed_1 <= b3_data;
  b4_data_delayed_1 <= b4_data;
  b5_data_delayed_1 <= b5_data;
  b6_data_delayed_1 <= b6_data;
  b7_data_delayed_1 <= b7_data;
  b2_data_delayed_2 <= b2_data_delayed_1;
  b3_data_delayed_2 <= b3_data_delayed_1;
  b3_data_delayed_3 <= b3_data_delayed_2;
  b4_data_delayed_2 <= b4_data_delayed_1;
  b4_data_delayed_3 <= b4_data_delayed_2;
  b4_data_delayed_4 <= b4_data_delayed_3;
  b5_data_delayed_2 <= b5_data_delayed_1;
  b5_data_delayed_3 <= b5_data_delayed_2;
  b5_data_delayed_4 <= b5_data_delayed_3;
  b5_data_delayed_5 <= b5_data_delayed_4;
  b6_data_delayed_2 <= b6_data_delayed_1;
  b6_data_delayed_3 <= b6_data_delayed_2;
  b6_data_delayed_4 <= b6_data_delayed_3;
  b6_data_delayed_5 <= b6_data_delayed_4;
  b6_data_delayed_6 <= b6_data_delayed_5;
  b7_data_delayed_2 <= b7_data_delayed_1;
  b7_data_delayed_3 <= b7_data_delayed_2;
  b7_data_delayed_4 <= b7_data_delayed_3;
  b7_data_delayed_5 <= b7_data_delayed_4;
  b7_data_delayed_6 <= b7_data_delayed_5;
  b7_data_delayed_7 <= b7_data_delayed_6;
 
  end
end
endmodule