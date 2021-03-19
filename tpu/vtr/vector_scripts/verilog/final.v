

`ifndef _OPTIONS_V_
`define _OPTIONS_V_ 1

`define NO_PLI 1
//`define TEST_BENCH 1
`define USE_INHOUSE_LOGIC
`define SIMULATION_MEMORY
// Replaces altera blocks with local logic files

/************************** ABBREVIEATED NAMES *****************************/
// Note: LG = Log base 2
//
// Default configuration:
//    8KB Icache (LGID=13) with 16 byte cache line size (LGIW=4)
//   32KB Dcache (LGDD=15) with 64 byte cache line size (LGDD=6)
//    Data prefetching off (DP=0, DPV=0)
//    16 Vector Lanes (LGL=4)
//    64 Maximum Vector Length (LGMVL=6)
//    32-bit (4-byte) Vector lane width (LGVPW=2)
//    32-bit Vector lane datapath width (LGLW=5)
//    16 Memory Crossbar lanes (LGM=LGL)
//    16 Multiplier lanes (LGX=LGL)
//    2 Register Banks (LGB=1)
//    Disable ALU per Bank (APB=0)


// INSTR CACHE
`define LGID 13
`define LGIW 4

// DATA CACHE
`define LGDD 15
`define LGDW 6

// DATA CACHE PREFETCHER - dcache needs to have 64 byte lines
`define LGDWB 7
`define DP 0
// VECTOR DATA CACHE PREFETCHER 0:off, 65535-N:N*veclength, N:pfch N cache lines
`define DPV 0

// VECTOR CORE
//Changing to 3. That is, we now have 8 lanes.
`define LGL 3
`define LGB 1
`define APB 0
`define LGM `LGL
`define LGX `LGL

// VECTOR ISA
`define LGMVL 6
`define LGVPW 1 //chaging the word size of vector processor to 16: support for bfloat16
`define LGLW 5

/****************************** FULL NAMES *********************************/

// INSTR CACHE
`define LOG2ICACHEDEPTHBYTES `LGID
`define LOG2ICACHEWIDTHBITS (`LGIW+3)

// DATA CACHE
`define LOG2DCACHEDEPTHBYTES `LGDD
`define LOG2DCACHEWIDTHBITS (`LGDW+3)

// DATA CACHE PREFETCHER - dcache needs to have 64 byte lines
`define LOG2DATAWBBUFFERSIZE `LGDWB
`define DEFAULTDCACHEPREFETCHES `DP
// VECTOR DATA CACHE PREFETCHER 0:off, 65535:vectorlength, N:pfch N cache lines
`define VECTORPREFETCHES `DPV

// VECTOR CORE
`define LOG2NUMLANES `LGL
`define LOG2MVL `LGMVL
`define LOG2VPW `LGVPW
`define LOG2LANEWIDTHBITS `LGLW
`define LOG2NUMMEMLANES `LGM
`define LOG2NUMMULLANES `LGX
`define LOG2NUMBANKS `LGB
`define ALUPERBANK `APB

/****************************** OTHER PARAMS *********************************/

// DRAM
`define LOG2DRAMWIDTHBITS 7

/****************** NUM PIPELINE STAGES in VECTOR PROCESSOR ***************/
//mult consumes 3 cycles
//`define MAX_PIPE_STAGES 7
//matmul consumes 29 cycles
`define MAX_PIPE_STAGES 33
`define MATMUL_STAGES 29

/****************** SIZE OF THE MATMUL UNIT ***************/
`define MAT_MUL_SIZE 8

`endif


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

/****************************************************************************

  ALU Circuit:

         |    _     flags
 src1 ---0---| \    |
             >+|----+----|Sat|------|M src1 --|M\--|-/         |sum|      |U|---
        |U|                       +-|X/
 src2 --|X/                       |    1
                                  |
      src2 ------------|M\        |
 src1 ----------|&|    |U|--|Sat|-+
                |||----|X/  |siz|
 src2 ---0------|^|      0
         |


              ADD SUB   CMP    CMP.U & | ^ ~| MIN MAX ABS MRG
              S U S U = ! < <= < <=           S U S U

adderopA_sel  0 0 0 0 0 0 0 0  0 0   x x x x  0 0 0 0  1   1
adderopB_sel  0 0 0 0 0 0 0 0  0 0   x x x x  0 0 0 0  1   0
logicopB_sel  x x x x x x x x  x x   0 0 0 0  1 1 1 1  1   1
signed        1 0 1 0 1 1 1 1  0 0   x x x x  1 0 1 0  1   x
addsub        0 0 1 1 1 1 1 1  1 1   x x x x  1 1 1 1  1   0
max           x x x x x x x x  x x   0 0 0 0  0 0 1 1  0   0
min           x x x x x x x x  x x   0 0 0 0  1 1 0 0  0   0
logic[1:0]    x x x x x x x x  x x   1 0 2 3  0 0 0 0  0   0
mux1_sel[1:0] 0 0 0 0 x x x x  x x   1 1 1 1  1 1 1 1  2   3 -- Combine
flag_sel[1:0] x x x x 0 1 2 3  2 3   x x x x  x x x x  x   x /

ALUOP_ZERO    =11'bzz1zzzz0101
ALUOP_ADD     =11'b00z10zzzz00
ALUOP_ADDU    =11'b00z00zzzz00
ALUOP_SUB     =11'b00z11zzzz00
ALUOP_SUBU    =11'b00z01zzzz00
ALUOP_CMP_EQ  =11'b00z11zzzz00
ALUOP_CMP_NEQ =11'b00z11zzzz01
ALUOP_CMP_LT  =11'b00z11zzzz10
ALUOP_CMP_LE  =11'b00z11zzzz11
ALUOP_CMP_LT_U=11'b00z01zzzz10
ALUOP_CMP_LE_U=11'b00z01zzzz11
ALUOP_AND     =11'bzz0zz000101
ALUOP_OR      =11'bzz0zz000001
ALUOP_XOR     =11'bzz0zz001001
ALUOP_NOR     =11'bzz0zz001101
ALUOP_MIN     =11'b00111010001
ALUOP_MIN_U   =11'b00101010001
ALUOP_MAX     =11'b00111100001
ALUOP_MAX_U   =11'b00101100001
ALUOP_ABS     =11'b11111000010
ALUOP_MERGE   =11'b101z0000011

****************************************************************************/

//`include "vlane_saturate.v"

module vlane_alu_32(
    clk,
    resetn,

    pipe_en,
    pipe_squashn,

    src1,
    src2,
    mask,

    op,
    satsum_op,
    satsize_op,

    cmp_result,
    result

    );

parameter WIDTH=32;

input clk;
input resetn;

input pipe_en;
input pipe_squashn;

input  [WIDTH-1:0] src1;
input  [WIDTH-1:0] src2;

input  mask;

input  [10:0] op;
input  [1:0] satsum_op;
input  [3:0] satsize_op;

output cmp_result;
output [WIDTH-1:0] result;

wire [WIDTH-1:0] adder_opA;
wire [WIDTH-1:0] adder_opB;
wire [WIDTH+2-1:0] adder_result;

wire [WIDTH-1:0] logic_opA;
wire [WIDTH-1:0] logic_opB;
reg  [WIDTH-1:0] logic_result;

wire [WIDTH-1:0] mux0_result;
wire [WIDTH-1:0] mux1_result;

wire [WIDTH-1:0] satsum_result;
wire [WIDTH-1:0] satsize_result;

wire lt;
wire neq;
wire eq;
wire le;

wire ctrl_adderopA_sel;
wire ctrl_adderopB_sel;
wire ctrl_logicopB_sel;
wire ctrl_signed;
wire ctrl_addsub;
wire ctrl_max;
wire ctrl_min;
wire [1:0] ctrl_logic;
wire [1:0] ctrl_mux1_sel;
wire [1:0] ctrl_flag_sel;

  assign ctrl_adderopA_sel=op[10];
  assign ctrl_adderopB_sel=op[9];
  assign ctrl_logicopB_sel=op[8];
  assign ctrl_signed=op[7];
  assign ctrl_addsub=~op[6];  //Make add 0, sub 1
  assign ctrl_max=op[5];
  assign ctrl_min=op[4];
  assign ctrl_logic=op[3:2];
  assign ctrl_mux1_sel=op[1:0];
  assign ctrl_flag_sel=op[1:0];

  assign adder_opA=(ctrl_adderopA_sel) ? 0 : src1;
  assign adder_opB=(ctrl_adderopB_sel) ? src1 : src2;

`ifndef USE_INHOUSE_LOGIC
    `define USE_INHOUSE_LOGIC
`endif

  `ifdef USE_INHOUSE_LOGIC
  wire [(WIDTH+2)-1:0] dataa;
  wire [(WIDTH+2)-1:0] datab;
  wire cin;

  assign dataa = {{2{ctrl_signed&adder_opA[WIDTH-1]}},adder_opA};
  assign datab = {{2{ctrl_signed&adder_opB[WIDTH-1]}},adder_opB};
  assign cin = ~ctrl_addsub;

  local_add_sub_34_0_SIGNED local_adder_inst(
      .dataa(dataa),
      .datab(datab),
      .cin(cin),
      .add_sub(ctrl_addsub),
      .result(adder_result)
  );
  `else
  lpm_add_sub adder_inst(
      .dataa({{2{ctrl_signed&adder_opA[WIDTH-1]}},adder_opA}),
      .datab({{2{ctrl_signed&adder_opB[WIDTH-1]}},adder_opB}),
      .cin(~ctrl_addsub),
      .add_sub(ctrl_addsub),
      .result(adder_result)
          // synopsys translate_off
          , .cout (), .overflow (), .clken (), .clock (), .aclr ()
          // synopsys translate_on
      );
  defparam
      adder_inst.lpm_width=WIDTH+2,
      adder_inst.lpm_pipeline=0,
      adder_inst.lpm_representation="SIGNED";
  `endif

  assign lt=adder_result[WIDTH];
  assign neq=|adder_result;
  assign eq=~neq;
  assign le=lt||eq;

  assign cmp_result= (ctrl_flag_sel==0) ? eq :
                     (ctrl_flag_sel==1) ? neq :
                     (ctrl_flag_sel==2) ? lt : le;

  assign logic_opA=src1;
  assign logic_opB=(ctrl_logicopB_sel) ? 0 : src2;

  always@*
    case(ctrl_logic)
        2'b00:
            logic_result=logic_opA|logic_opB;
        2'b01:
            logic_result=logic_opA&logic_opB;
        2'b10:
            logic_result=logic_opA^logic_opB;
        2'b11:
            logic_result=~(logic_opA|logic_opB);
    endcase

  assign mux0_result=((lt && ctrl_max) || (~lt && ctrl_min)) ?  src2 : 
                                                                logic_result;

  /************* PIPELINE at this point ***************/
  reg [WIDTH-1:0] mux0_result_s2;
  reg [WIDTH+2-1:0] adder_result_s2;
  reg  [3:0] satsize_op_s2;
  reg  [3:0] satsum_op_s2;
  reg ctrl_mux1_sel_s2;

  always@(posedge clk)
  begin
    if (!resetn || !pipe_squashn)
    begin
      mux0_result_s2<=0;
      adder_result_s2<=0;
      satsize_op_s2<=0;
      satsum_op_s2<=0;
      ctrl_mux1_sel_s2<=0;
    end
    else if (pipe_en)
    begin
      mux0_result_s2<=mux0_result;
      adder_result_s2<=adder_result;
      satsize_op_s2<=satsize_op;
      satsum_op_s2<=satsum_op;
      ctrl_mux1_sel_s2<=((ctrl_mux1_sel==0) ||
                         (ctrl_mux1_sel==2 && src1[WIDTH-1]) ||
                         (ctrl_mux1_sel==3 && mask) );
    end
  end

  vlane_saturatesize satsize(   //Unit only works for 32-bit inputs
      .in(mux0_result_s2),
      .op(satsize_op_s2),
      .out(satsize_result)
      );

  vlane_saturatesum_32 satsum(
      .in(adder_result_s2),
      .op(satsum_op_s2),
      .out(satsum_result)
      );

  assign mux1_result= (ctrl_mux1_sel_s2) ? 
                              satsum_result : satsize_result;

  assign result=mux1_result;

endmodule
module seq_mac(a, b, out, reset, clk);
input [`DWIDTH-1:0] a;
input [`DWIDTH-1:0] b;
input reset;
input clk;
output [`DWIDTH-1:0] out;

reg [2*`DWIDTH-1:0] out_temp;
wire [`DWIDTH-1:0] mul_out;
wire [2*`DWIDTH-1:0] add_out;

reg [`DWIDTH-1:0] a_flopped;
reg [`DWIDTH-1:0] b_flopped;

wire [2*`DWIDTH-1:0] mul_out_temp;
reg [2*`DWIDTH-1:0] mul_out_temp_reg;

always @(posedge clk) begin
  if (reset) begin
    a_flopped <= 0;
    b_flopped <= 0;
  end else begin
    a_flopped <= a;
    b_flopped <= b;
  end
end

//assign mul_out = a * b;
qmult mult_u1(.i_multiplicand(a_flopped), .i_multiplier(b_flopped), .o_result(mul_out_temp));

always @(posedge clk) begin
  if (reset) begin
    mul_out_temp_reg <= 0;
  end else begin
    mul_out_temp_reg <= mul_out_temp;
  end
end

//we just truncate the higher bits of the product
//assign add_out = mul_out + out;
qadd add_u1(.a(out_temp), .b(mul_out_temp_reg), .c(add_out));

always @(posedge clk) begin
  if (reset) begin
    out_temp <= 0;
  end else begin
    out_temp <= add_out;
  end
end

//down cast the result
assign out = 
    (out_temp[2*`DWIDTH-1] == 0) ?  //positive number
        (
           (|(out_temp[2*`DWIDTH-2 : `DWIDTH-1])) ?  //is any bit from 14:7 is 1, that means overlfow
             {out_temp[2*`DWIDTH-1] , {(`DWIDTH-1){1'b1}}} : //sign bit and then all 1s
             {out_temp[2*`DWIDTH-1] , out_temp[`DWIDTH-2:0]} 
        )
        : //negative number
        (
           (|(out_temp[2*`DWIDTH-2 : `DWIDTH-1])) ?  //is any bit from 14:7 is 0, that means overlfow
             {out_temp[2*`DWIDTH-1] , out_temp[`DWIDTH-2:0]} :
             {out_temp[2*`DWIDTH-1] , {(`DWIDTH-1){1'b0}}} //sign bit and then all 0s
        );

endmodule
module dpram_9_512_128 (
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
input [9-1:0] address_a;
input [9-1:0] address_b;
input  wren_a;
input  wren_b;
input [128-1:0] data_a;
input [128-1:0] data_b;
output reg [128-1:0] out_a;
output reg [128-1:0] out_b;

reg [128-1:0] ram[512-1:0];

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

endmodule

module local_add_sub_34_0_SIGNED(
dataa,
datab,
cin,
add_sub,
result
);

input[34-1:0] dataa;
input[34-1:0] datab;
input cin;
input add_sub;
output reg [34-1:0] result;

always @(*)begin
    if(add_sub == 1'b1)
         result = dataa+datab+cin;
    else
         result = dataa - datab;
end

endmodule
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

///////////////////////////////////////////////////////////////////
//Module to reduce multiple values (add, max, min) into one final result.
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// Numerics. We use fixed point format:
//  Most significant 8 bits represent integer part and Least significant 8 bits
//  represent fraction part
//  i.e. IIIIIIIIFFFFFFFF = IIIIIIII.FFFFFFFF
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// There are 32 inputs to the reduction unit. We use a tree structure to reduce the 32 values.
// It is assumed that the number of addressses supplied (end_addr - start_addr + 1) is a multiple
// of 32. If the real application needs to reduce a number of values that are not a multiple of
// 32, then the application must pad the values in the input BRAM appropriately
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// A user is expected to use the resetn signal everytime before starting a new reduction operation.
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// Accumulation is done in 20 bits (16 + log(16))
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// Each entry of the RAM contains `NUM_INPUTS (which is 32) values. So,
// 1 read of the RAM provides all the inputs required for going through
// the reduction unit once. 
//////////////////////////////////////////////////////////////////


////////////////////////////////////////////
// Top module
////////////////////////////////////////////
module reduction_layer_10_3_32_5_10 
(
  input clk,
  input resetn, //resets the processing elements
  input en, //indicates valid reduction operation
  input  [1:0] reduction_type, //can have 3 values: 0 (Add), 1 (Max), 2 (Min)
  input read,
  input  [8 * 10 -1:0] a, // input data to reduction logic
  output [8 * 10 -1:0] reduced_out, //output
  output reg done, //output is valid when done is 1
  output reg busy
);

wire [8 * 10 -1:0] reduced_out_1; //output
wire [8 * 10 -1:0] reduced_out_add; //output
wire [10 + 3-1:0] reduced_out_unrounded;

reduction_unit_10_3 ucu(
  .clk(clk),
  .reset(reset_reduction_unit),
  .inp0(a[1*10-1:0*10]), 
  .inp1(a[2*10-1:1*10]), 
  .inp2(a[3*10-1:2*10]), 
  .inp3(a[4*10-1:3*10]), 
  .inp4(a[5*10-1:4*10]), 
  .inp5(a[6*10-1:5*10]), 
  .inp6(a[7*10-1:6*10]), 
  .inp7(a[8*10-1:7*10]), 
  .mode(reduction_type),
  .outp(reduced_out_unrounded)
);

////////////////////////////////////////////////////////////////
// Rounding of the output of reduction unit (from 20 bits to 16 bits).
// This is required only when reduction type is "sum"
////////////////////////////////////////////////////////////////
rounding_13_10 u_round(.i_data(reduced_out_unrounded), .o_data(reduced_out_add));

assign reduced_out_1 = (reduction_type==2'b0) ? reduced_out_add : reduced_out_unrounded[10-1:0];
assign reduced_out = {8{reduced_out_1}};

reg[2:0] count;

always@(posedge clk)begin
  if(!resetn)begin
    count <= 3'b0;
  end
  else begin
    if(en)
        count <= count + 1;
    if(read)
        count <= count - 1;
  end
end

always@(*)begin
  if(count == 8)begin
    busy = 1'b1;
  end
  else begin
    busy = 1'b0;
  end
end

always@(posedge clk)begin
  if(!resetn)begin
    done <= 1'b0;
  end
  else begin
    if(count == 8)
      done <= 1'b1;
    else if(count ==0)
      done <= 1'b0;
  end
end

endmodule

/****************************************************************************
          Shifter unit

Opcode Table:

sign_ext dir 
 0        0    |  ShiftLeft
 0        1    |  ShiftRightLogic
 1        1    |  ShiftRightArith
          
****************************************************************************/
module vlane_barrelshifter_32_5(clk, resetn,
            opB, sa, 
            op, 
            result);
//parameter 32=32;
//parameter 5=5;

//Shifts the first 2 bits in one cycle, the rest in the next cycle
//parameter (5-2)=5-2;

input clk;
input resetn;

input [32-1:0] opB;
input [5-1:0] sa;                             // Shift Amount
input [2-1:0] op;

output [32-1:0] result;


wire sign_ext;
wire shift_direction;
assign sign_ext=op[1];
assign shift_direction=op[0];

wire dum,dum_,dum2;
wire [32-1:0] partial_result_,partial_result;
`ifndef USE_INHOUSE_LOGIC
    `define USE_INHOUSE_LOGIC
`endif

`ifdef USE_INHOUSE_LOGIC
wire [33-1:0] local_shifter_inst1_result;
assign {dum,partial_result} = local_shifter_inst1_result;

wire [2-1:0] local_shifter_inst1_distance;
assign local_shifter_inst1_distance = sa&(32'hffffffff<<(((5-2)>0) ? (5-2) : 0));

wire [33-1:0] local_shifter_inst1_data;
assign local_shifter_inst1_data = {sign_ext&opB[32-1],opB};

local_shifter_33_2_ARITHMATIC local_shifter_inst1(
  .data(local_shifter_inst1_data),
  .distance(local_shifter_inst1_distance),
  .direction(shift_direction),
  .result(local_shifter_inst1_result)
);
 //defparam
 //   local_shifter_inst1.LPM_WIDTH = 32+1,
 //   local_shifter_inst1.LPM_WIDTHDIST = 5,
 //   local_shifter_inst1.LPM_SHIFTTYPE="ARITHMETIC";
`else
lpm_clshift shifter_inst1(
    .data({sign_ext&opB[32-1],opB}),
    .distance(sa&(32'hffffffff<<(((5-2)>0) ? (5-2) : 0))),
    .direction(shift_direction),
    .result(dum,partial_result));
 defparam
    shifter_inst1.lpm_width = 32+1,
    shifter_inst1.lpm_widthdist = 5,
    shifter_inst1.lpm_shifttype="ARITHMETIC";
`endif

wire [33-1:0] partial_reg_q;
assign partial_reg_q = {dum_,partial_result_};
register_33 partial_reg
  ({dum,partial_result},clk,resetn,1'b1,partial_reg_q);

wire [5-1:0] sa_2;
wire shift_direction_2;

register_5 secondstage (sa, clk,resetn,1'b1,sa_2); 

register_1 secondstagedir (shift_direction, clk,resetn,1'b1,shift_direction_2); 

`ifdef USE_INHOUSE_LOGIC
wire [33-1:0] local_shifter_inst2_result;
assign {dum2,result} = local_shifter_inst2_result;

wire [2-1:0] local_shifter_inst2_distance;
assign local_shifter_inst2_distance = sa_2[(5-2)-1:0];

wire [33-1:0] local_shifter_inst2_data;
assign local_shifter_inst2_data = {dum_,partial_result_};

local_shifter_33_2_ARITHMATIC local_shifter_inst2(
  .data(local_shifter_inst2_data),
  .distance(local_shifter_inst2_distance),
  .direction(shift_direction_2),
  .result(local_shifter_inst2_result)
);
// defparam
//    local_shifter_inst2.LPM_WIDTH = 32+1,
//   local_shifter_inst2.LPM_WIDTHDIST = ((5-2)>0) ? (5-2) : 1,
//    local_shifter_inst2.LPM_SHIFTTYPE ="ARITHMETIC";
`else
lpm_clshift_33_2_ARITHMATIC shifter_inst2(
    .data({dum_,partial_result_}),
    .distance(sa_2[(((5-2)>0) ? (5-2)-1 : 0):0]),
    .direction(shift_direction_2),
    .result({dum2,resulti}));
 defparam 
    shifter_inst2.lpm_width = 32+1,
    shifter_inst2.lpm_widthdist = ((5-2)>0) ? (5-2) : 1,
    shifter_inst2.lpm_shifttype="ARITHMETIC";
`endif


endmodule

        
module qadd(a,b,c);
    input [2*`DWIDTH-1:0] a;
    input [2*`DWIDTH-1:0] b;
    output [2*`DWIDTH-1:0] c;

    assign c = a + b;
    //DW01_add #(`DWIDTH) u_add(.A(a), .B(b), .CI(1'b0), .SUM(c), .CO());
endmodule
/****************************************************************************
          Saturate unit

  Takes a signed input and saturates it

  sat sign
   0    x   NOP (pass through)
   1    1   VS[ADD/SUB]
   1    0   VS[ADD/SUB]_U

parameter
  SATSUMOP_NOP=2'b00,
  SATSUMOP_VS =2'b11,
  SATSUMOP_VSU=2'b10;

****************************************************************************/


module vlane_saturatesum_32(
    in,
    op,
    out
    );

parameter WIDTH=32;

input [WIDTH+2-1:0] in;
input [1:0] op;
output [WIDTH-1:0] out;

reg [WIDTH-1:0] out;

wire op_saturate;
wire op_signed;

assign op_saturate=op[1];
assign op_signed=op[0];

wire [WIDTH-1:0] maxunsigned;
wire [WIDTH-1:0] minunsigned;
wire [WIDTH-1:0] maxsigned;
wire [WIDTH-1:0] minsigned;

assign maxunsigned = {WIDTH{1'b1}};
assign minunsigned = 0;
assign maxsigned = {1'b0,{WIDTH-1{1'b1}}};
assign minsigned = {1'b1,{WIDTH-1{1'b0}}};

wire [WIDTH-1:0] through;

assign through=in[WIDTH-1:0];

wire [2:0] top3bits=(op_saturate) ? in[WIDTH+2-1:WIDTH-1] : 3'b0 ;

  always@*
    case(top3bits)
      3'b010: out=maxunsigned;
      3'b011: out=maxunsigned;
      3'b001: out=(op_signed) ? maxsigned : through;
      3'b111: out=(op_signed) ? through : minunsigned;
      3'b110: out=(op_signed) ? minsigned : minunsigned;
      default: out=through;
    endcase

endmodule
module ram_wrapper_9_512_4 (
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
input [4-1:0] data_a;
input [4-1:0] data_b;
output [4-1:0] out_a;
output [4-1:0] out_b;

reg [9-1:0] q_address_a;
reg [9-1:0] q_address_b;
reg [9-1:0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_9_512_4 dpram1(
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


module trp_unit_10 (
 input clk,
 input resetn,
 input en,
 input [10-1:0] a,
 input [1:0] mode,
 input read,
 output busy,
 output reg valid,
 output reg[10-1:0] out
);

reg en_reduction;
reg en_transpose;
reg read_transpose;
reg read_reduction;
wire transpose_busy;
wire reduction_busy;
wire reduction_done;
wire [10-1:0] reduction_out;
wire [10-1:0] transpose_out;
 
 
assign busy = transpose_busy || reduction_busy;

always@(*)begin
  if(mode == 2'b11)begin
    en_transpose = en;
    en_reduction = 1'b0;
    read_transpose = read;
    read_reduction = 1'b0;
  end
  else begin
    en_transpose = 1'b0;
    en_reduction = en;
    read_transpose = 1'b0;
    read_reduction = read;
  end
end

always@(*)begin
  if(transpose_busy)begin
    out = transpose_out;
    valid = 1'b1;
  end
  else if(reduction_done)begin
    out = reduction_out;
    valid = 1'b1;
  end
  else begin
    out = 'h0;
    valid = 1'b0;
  end
end

reduction_layer_10_3_32_5_10 u_reduction_layer(
  .clk(clk),
  .resetn(resetn),
  .en(en_reduction),
  .read(read_reduction),
  .reduction_type(mode),
  .a(a),
  .reduced_out(reduction_out),
  .done(reduction_done),
  .busy(reduction_busy)
);

transpose_10_2_1 u_transpose(
  .clk(clk),
  .resetn(resetn),
  .read(),
  .en(en_transpose), 
  .a(a),
  .out(transpose_out),
  .busy(transpose_busy)
);

endmodule


 

/****************************************************************************
 *           Load data translator
 *- moves read data to appropriate byte/halfword and zero/sign extends
 *****************************************************************************/
module vload_data_translator(
    d_readdatain,
    d_address,
    load_size,
    load_sign_ext,
    d_loadresult);
parameter WIDTH=32;

input [WIDTH-1:0] d_readdatain;
input [1:0] d_address;
input [1:0] load_size;
input load_sign_ext;
output [WIDTH-1:0] d_loadresult;

reg [WIDTH-1:0] d_loadresult;

always @(d_readdatain or d_address or load_size or load_sign_ext)
begin
    case (load_size)
        2'b00:
        begin
            case (d_address[1:0])
                2'd0: d_loadresult[7:0]=d_readdatain[31:24];
                2'd1: d_loadresult[7:0]=d_readdatain[23:16];
                2'd2: d_loadresult[7:0]=d_readdatain[15:8];
                default: d_loadresult[7:0]=d_readdatain[7:0];
            endcase
            d_loadresult[31:8]={24{load_sign_ext&d_loadresult[7]}};
        end
        2'b01:
        begin
            case (d_address[1])
                2'd0: d_loadresult[15:0]=d_readdatain[31:16];
                default: d_loadresult[15:0]=d_readdatain[15:0];
            endcase
            d_loadresult[31:16]={16{load_sign_ext&d_loadresult[15]}};
        end
        default:
            d_loadresult=d_readdatain;
    endcase
end
endmodule


//////////////////////////////////////////////////////////////////////////
// Systolically connected PEs
//////////////////////////////////////////////////////////////////////////
module systolic_pe_matrix(
clk,
reset,
pe_reset,
a0,
a1,
a2,
a3,
a4,
a5,
a6,
a7,
b0,
b1,
b2,
b3,
b4,
b5,
b6,
b7,
matrixC0_0,
matrixC0_1,
matrixC0_2,
matrixC0_3,
matrixC0_4,
matrixC0_5,
matrixC0_6,
matrixC0_7,
matrixC1_0,
matrixC1_1,
matrixC1_2,
matrixC1_3,
matrixC1_4,
matrixC1_5,
matrixC1_6,
matrixC1_7,
matrixC2_0,
matrixC2_1,
matrixC2_2,
matrixC2_3,
matrixC2_4,
matrixC2_5,
matrixC2_6,
matrixC2_7,
matrixC3_0,
matrixC3_1,
matrixC3_2,
matrixC3_3,
matrixC3_4,
matrixC3_5,
matrixC3_6,
matrixC3_7,
matrixC4_0,
matrixC4_1,
matrixC4_2,
matrixC4_3,
matrixC4_4,
matrixC4_5,
matrixC4_6,
matrixC4_7,
matrixC5_0,
matrixC5_1,
matrixC5_2,
matrixC5_3,
matrixC5_4,
matrixC5_5,
matrixC5_6,
matrixC5_7,
matrixC6_0,
matrixC6_1,
matrixC6_2,
matrixC6_3,
matrixC6_4,
matrixC6_5,
matrixC6_6,
matrixC6_7,
matrixC7_0,
matrixC7_1,
matrixC7_2,
matrixC7_3,
matrixC7_4,
matrixC7_5,
matrixC7_6,
matrixC7_7,

a_data_out,
b_data_out
);

input clk;
input reset;
input pe_reset;
input [`DWIDTH-1:0] a0;
input [`DWIDTH-1:0] a1;
input [`DWIDTH-1:0] a2;
input [`DWIDTH-1:0] a3;
input [`DWIDTH-1:0] a4;
input [`DWIDTH-1:0] a5;
input [`DWIDTH-1:0] a6;
input [`DWIDTH-1:0] a7;
input [`DWIDTH-1:0] b0;
input [`DWIDTH-1:0] b1;
input [`DWIDTH-1:0] b2;
input [`DWIDTH-1:0] b3;
input [`DWIDTH-1:0] b4;
input [`DWIDTH-1:0] b5;
input [`DWIDTH-1:0] b6;
input [`DWIDTH-1:0] b7;
output [`DWIDTH-1:0] matrixC0_0;
output [`DWIDTH-1:0] matrixC0_1;
output [`DWIDTH-1:0] matrixC0_2;
output [`DWIDTH-1:0] matrixC0_3;
output [`DWIDTH-1:0] matrixC0_4;
output [`DWIDTH-1:0] matrixC0_5;
output [`DWIDTH-1:0] matrixC0_6;
output [`DWIDTH-1:0] matrixC0_7;
output [`DWIDTH-1:0] matrixC1_0;
output [`DWIDTH-1:0] matrixC1_1;
output [`DWIDTH-1:0] matrixC1_2;
output [`DWIDTH-1:0] matrixC1_3;
output [`DWIDTH-1:0] matrixC1_4;
output [`DWIDTH-1:0] matrixC1_5;
output [`DWIDTH-1:0] matrixC1_6;
output [`DWIDTH-1:0] matrixC1_7;
output [`DWIDTH-1:0] matrixC2_0;
output [`DWIDTH-1:0] matrixC2_1;
output [`DWIDTH-1:0] matrixC2_2;
output [`DWIDTH-1:0] matrixC2_3;
output [`DWIDTH-1:0] matrixC2_4;
output [`DWIDTH-1:0] matrixC2_5;
output [`DWIDTH-1:0] matrixC2_6;
output [`DWIDTH-1:0] matrixC2_7;
output [`DWIDTH-1:0] matrixC3_0;
output [`DWIDTH-1:0] matrixC3_1;
output [`DWIDTH-1:0] matrixC3_2;
output [`DWIDTH-1:0] matrixC3_3;
output [`DWIDTH-1:0] matrixC3_4;
output [`DWIDTH-1:0] matrixC3_5;
output [`DWIDTH-1:0] matrixC3_6;
output [`DWIDTH-1:0] matrixC3_7;
output [`DWIDTH-1:0] matrixC4_0;
output [`DWIDTH-1:0] matrixC4_1;
output [`DWIDTH-1:0] matrixC4_2;
output [`DWIDTH-1:0] matrixC4_3;
output [`DWIDTH-1:0] matrixC4_4;
output [`DWIDTH-1:0] matrixC4_5;
output [`DWIDTH-1:0] matrixC4_6;
output [`DWIDTH-1:0] matrixC4_7;
output [`DWIDTH-1:0] matrixC5_0;
output [`DWIDTH-1:0] matrixC5_1;
output [`DWIDTH-1:0] matrixC5_2;
output [`DWIDTH-1:0] matrixC5_3;
output [`DWIDTH-1:0] matrixC5_4;
output [`DWIDTH-1:0] matrixC5_5;
output [`DWIDTH-1:0] matrixC5_6;
output [`DWIDTH-1:0] matrixC5_7;
output [`DWIDTH-1:0] matrixC6_0;
output [`DWIDTH-1:0] matrixC6_1;
output [`DWIDTH-1:0] matrixC6_2;
output [`DWIDTH-1:0] matrixC6_3;
output [`DWIDTH-1:0] matrixC6_4;
output [`DWIDTH-1:0] matrixC6_5;
output [`DWIDTH-1:0] matrixC6_6;
output [`DWIDTH-1:0] matrixC6_7;
output [`DWIDTH-1:0] matrixC7_0;
output [`DWIDTH-1:0] matrixC7_1;
output [`DWIDTH-1:0] matrixC7_2;
output [`DWIDTH-1:0] matrixC7_3;
output [`DWIDTH-1:0] matrixC7_4;
output [`DWIDTH-1:0] matrixC7_5;
output [`DWIDTH-1:0] matrixC7_6;
output [`DWIDTH-1:0] matrixC7_7;

output [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data_out;
output [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data_out;

wire [`DWIDTH-1:0] a0_0to0_1, a0_1to0_2, a0_2to0_3, a0_3to0_4, a0_4to0_5, a0_5to0_6, a0_6to0_7, a0_7to0_8;
wire [`DWIDTH-1:0] a1_0to1_1, a1_1to1_2, a1_2to1_3, a1_3to1_4, a1_4to1_5, a1_5to1_6, a1_6to1_7, a1_7to1_8;
wire [`DWIDTH-1:0] a2_0to2_1, a2_1to2_2, a2_2to2_3, a2_3to2_4, a2_4to2_5, a2_5to2_6, a2_6to2_7, a2_7to2_8;
wire [`DWIDTH-1:0] a3_0to3_1, a3_1to3_2, a3_2to3_3, a3_3to3_4, a3_4to3_5, a3_5to3_6, a3_6to3_7, a3_7to3_8;
wire [`DWIDTH-1:0] a4_0to4_1, a4_1to4_2, a4_2to4_3, a4_3to4_4, a4_4to4_5, a4_5to4_6, a4_6to4_7, a4_7to4_8;
wire [`DWIDTH-1:0] a5_0to5_1, a5_1to5_2, a5_2to5_3, a5_3to5_4, a5_4to5_5, a5_5to5_6, a5_6to5_7, a5_7to5_8;
wire [`DWIDTH-1:0] a6_0to6_1, a6_1to6_2, a6_2to6_3, a6_3to6_4, a6_4to6_5, a6_5to6_6, a6_6to6_7, a6_7to6_8;
wire [`DWIDTH-1:0] a7_0to7_1, a7_1to7_2, a7_2to7_3, a7_3to7_4, a7_4to7_5, a7_5to7_6, a7_6to7_7, a7_7to7_8;

wire [`DWIDTH-1:0] b0_0to1_0, b1_0to2_0, b2_0to3_0, b3_0to4_0, b4_0to5_0, b5_0to6_0, b6_0to7_0, b7_0to8_0;
wire [`DWIDTH-1:0] b0_1to1_1, b1_1to2_1, b2_1to3_1, b3_1to4_1, b4_1to5_1, b5_1to6_1, b6_1to7_1, b7_1to8_1;
wire [`DWIDTH-1:0] b0_2to1_2, b1_2to2_2, b2_2to3_2, b3_2to4_2, b4_2to5_2, b5_2to6_2, b6_2to7_2, b7_2to8_2;
wire [`DWIDTH-1:0] b0_3to1_3, b1_3to2_3, b2_3to3_3, b3_3to4_3, b4_3to5_3, b5_3to6_3, b6_3to7_3, b7_3to8_3;
wire [`DWIDTH-1:0] b0_4to1_4, b1_4to2_4, b2_4to3_4, b3_4to4_4, b4_4to5_4, b5_4to6_4, b6_4to7_4, b7_4to8_4;
wire [`DWIDTH-1:0] b0_5to1_5, b1_5to2_5, b2_5to3_5, b3_5to4_5, b4_5to5_5, b5_5to6_5, b6_5to7_5, b7_5to8_5;
wire [`DWIDTH-1:0] b0_6to1_6, b1_6to2_6, b2_6to3_6, b3_6to4_6, b4_6to5_6, b5_6to6_6, b6_6to7_6, b7_6to8_6;
wire [`DWIDTH-1:0] b0_7to1_7, b1_7to2_7, b2_7to3_7, b3_7to4_7, b4_7to5_7, b5_7to6_7, b6_7to7_7, b7_7to8_7;

//////////////////////////////////////////////////////////////////////////
// Instantiations of the actual PEs
//////////////////////////////////////////////////////////////////////////
//For larger matmul, more PEs will be needed
wire effective_rst;
assign effective_rst = reset | pe_reset;

processing_element pe0_0(.reset(effective_rst), .clk(clk),  .in_a(a0),      .in_b(b0),  .out_a(a0_0to0_1), .out_b(b0_0to1_0), .out_c(matrixC0_0));
processing_element pe0_1(.reset(effective_rst), .clk(clk),  .in_a(a0_0to0_1), .in_b(b1),  .out_a(a0_1to0_2), .out_b(b0_1to1_1), .out_c(matrixC0_1));
processing_element pe0_2(.reset(effective_rst), .clk(clk),  .in_a(a0_1to0_2), .in_b(b2),  .out_a(a0_2to0_3), .out_b(b0_2to1_2), .out_c(matrixC0_2));
processing_element pe0_3(.reset(effective_rst), .clk(clk),  .in_a(a0_2to0_3), .in_b(b3),  .out_a(a0_3to0_4), .out_b(b0_3to1_3), .out_c(matrixC0_3));
processing_element pe0_4(.reset(effective_rst), .clk(clk),  .in_a(a0_3to0_4), .in_b(b4),  .out_a(a0_4to0_5), .out_b(b0_4to1_4), .out_c(matrixC0_4));
processing_element pe0_5(.reset(effective_rst), .clk(clk),  .in_a(a0_4to0_5), .in_b(b5),  .out_a(a0_5to0_6), .out_b(b0_5to1_5), .out_c(matrixC0_5));
processing_element pe0_6(.reset(effective_rst), .clk(clk),  .in_a(a0_5to0_6), .in_b(b6),  .out_a(a0_6to0_7), .out_b(b0_6to1_6), .out_c(matrixC0_6));
processing_element pe0_7(.reset(effective_rst), .clk(clk),  .in_a(a0_6to0_7), .in_b(b7),  .out_a(a0_7to0_8), .out_b(b0_7to1_7), .out_c(matrixC0_7));

processing_element pe1_0(.reset(effective_rst), .clk(clk),  .in_a(a1), .in_b(b0_0to1_0),  .out_a(a1_0to1_1), .out_b(b1_0to2_0), .out_c(matrixC1_0));
processing_element pe2_0(.reset(effective_rst), .clk(clk),  .in_a(a2), .in_b(b1_0to2_0),  .out_a(a2_0to2_1), .out_b(b2_0to3_0), .out_c(matrixC2_0));
processing_element pe3_0(.reset(effective_rst), .clk(clk),  .in_a(a3), .in_b(b2_0to3_0),  .out_a(a3_0to3_1), .out_b(b3_0to4_0), .out_c(matrixC3_0));
processing_element pe4_0(.reset(effective_rst), .clk(clk),  .in_a(a4), .in_b(b3_0to4_0),  .out_a(a4_0to4_1), .out_b(b4_0to5_0), .out_c(matrixC4_0));
processing_element pe5_0(.reset(effective_rst), .clk(clk),  .in_a(a5), .in_b(b4_0to5_0),  .out_a(a5_0to5_1), .out_b(b5_0to6_0), .out_c(matrixC5_0));
processing_element pe6_0(.reset(effective_rst), .clk(clk),  .in_a(a6), .in_b(b5_0to6_0),  .out_a(a6_0to6_1), .out_b(b6_0to7_0), .out_c(matrixC6_0));
processing_element pe7_0(.reset(effective_rst), .clk(clk),  .in_a(a7), .in_b(b6_0to7_0),  .out_a(a7_0to7_1), .out_b(b7_0to8_0), .out_c(matrixC7_0));

processing_element pe1_1(.reset(effective_rst), .clk(clk),  .in_a(a1_0to1_1), .in_b(b0_1to1_1),  .out_a(a1_1to1_2), .out_b(b1_1to2_1), .out_c(matrixC1_1));
processing_element pe1_2(.reset(effective_rst), .clk(clk),  .in_a(a1_1to1_2), .in_b(b0_2to1_2),  .out_a(a1_2to1_3), .out_b(b1_2to2_2), .out_c(matrixC1_2));
processing_element pe1_3(.reset(effective_rst), .clk(clk),  .in_a(a1_2to1_3), .in_b(b0_3to1_3),  .out_a(a1_3to1_4), .out_b(b1_3to2_3), .out_c(matrixC1_3));
processing_element pe1_4(.reset(effective_rst), .clk(clk),  .in_a(a1_3to1_4), .in_b(b0_4to1_4),  .out_a(a1_4to1_5), .out_b(b1_4to2_4), .out_c(matrixC1_4));
processing_element pe1_5(.reset(effective_rst), .clk(clk),  .in_a(a1_4to1_5), .in_b(b0_5to1_5),  .out_a(a1_5to1_6), .out_b(b1_5to2_5), .out_c(matrixC1_5));
processing_element pe1_6(.reset(effective_rst), .clk(clk),  .in_a(a1_5to1_6), .in_b(b0_6to1_6),  .out_a(a1_6to1_7), .out_b(b1_6to2_6), .out_c(matrixC1_6));
processing_element pe1_7(.reset(effective_rst), .clk(clk),  .in_a(a1_6to1_7), .in_b(b0_7to1_7),  .out_a(a1_7to1_8), .out_b(b1_7to2_7), .out_c(matrixC1_7));
processing_element pe2_1(.reset(effective_rst), .clk(clk),  .in_a(a2_0to2_1), .in_b(b1_1to2_1),  .out_a(a2_1to2_2), .out_b(b2_1to3_1), .out_c(matrixC2_1));
processing_element pe2_2(.reset(effective_rst), .clk(clk),  .in_a(a2_1to2_2), .in_b(b1_2to2_2),  .out_a(a2_2to2_3), .out_b(b2_2to3_2), .out_c(matrixC2_2));
processing_element pe2_3(.reset(effective_rst), .clk(clk),  .in_a(a2_2to2_3), .in_b(b1_3to2_3),  .out_a(a2_3to2_4), .out_b(b2_3to3_3), .out_c(matrixC2_3));
processing_element pe2_4(.reset(effective_rst), .clk(clk),  .in_a(a2_3to2_4), .in_b(b1_4to2_4),  .out_a(a2_4to2_5), .out_b(b2_4to3_4), .out_c(matrixC2_4));
processing_element pe2_5(.reset(effective_rst), .clk(clk),  .in_a(a2_4to2_5), .in_b(b1_5to2_5),  .out_a(a2_5to2_6), .out_b(b2_5to3_5), .out_c(matrixC2_5));
processing_element pe2_6(.reset(effective_rst), .clk(clk),  .in_a(a2_5to2_6), .in_b(b1_6to2_6),  .out_a(a2_6to2_7), .out_b(b2_6to3_6), .out_c(matrixC2_6));
processing_element pe2_7(.reset(effective_rst), .clk(clk),  .in_a(a2_6to2_7), .in_b(b1_7to2_7),  .out_a(a2_7to2_8), .out_b(b2_7to3_7), .out_c(matrixC2_7));
processing_element pe3_1(.reset(effective_rst), .clk(clk),  .in_a(a3_0to3_1), .in_b(b2_1to3_1),  .out_a(a3_1to3_2), .out_b(b3_1to4_1), .out_c(matrixC3_1));
processing_element pe3_2(.reset(effective_rst), .clk(clk),  .in_a(a3_1to3_2), .in_b(b2_2to3_2),  .out_a(a3_2to3_3), .out_b(b3_2to4_2), .out_c(matrixC3_2));
processing_element pe3_3(.reset(effective_rst), .clk(clk),  .in_a(a3_2to3_3), .in_b(b2_3to3_3),  .out_a(a3_3to3_4), .out_b(b3_3to4_3), .out_c(matrixC3_3));
processing_element pe3_4(.reset(effective_rst), .clk(clk),  .in_a(a3_3to3_4), .in_b(b2_4to3_4),  .out_a(a3_4to3_5), .out_b(b3_4to4_4), .out_c(matrixC3_4));
processing_element pe3_5(.reset(effective_rst), .clk(clk),  .in_a(a3_4to3_5), .in_b(b2_5to3_5),  .out_a(a3_5to3_6), .out_b(b3_5to4_5), .out_c(matrixC3_5));
processing_element pe3_6(.reset(effective_rst), .clk(clk),  .in_a(a3_5to3_6), .in_b(b2_6to3_6),  .out_a(a3_6to3_7), .out_b(b3_6to4_6), .out_c(matrixC3_6));
processing_element pe3_7(.reset(effective_rst), .clk(clk),  .in_a(a3_6to3_7), .in_b(b2_7to3_7),  .out_a(a3_7to3_8), .out_b(b3_7to4_7), .out_c(matrixC3_7));
processing_element pe4_1(.reset(effective_rst), .clk(clk),  .in_a(a4_0to4_1), .in_b(b3_1to4_1),  .out_a(a4_1to4_2), .out_b(b4_1to5_1), .out_c(matrixC4_1));
processing_element pe4_2(.reset(effective_rst), .clk(clk),  .in_a(a4_1to4_2), .in_b(b3_2to4_2),  .out_a(a4_2to4_3), .out_b(b4_2to5_2), .out_c(matrixC4_2));
processing_element pe4_3(.reset(effective_rst), .clk(clk),  .in_a(a4_2to4_3), .in_b(b3_3to4_3),  .out_a(a4_3to4_4), .out_b(b4_3to5_3), .out_c(matrixC4_3));
processing_element pe4_4(.reset(effective_rst), .clk(clk),  .in_a(a4_3to4_4), .in_b(b3_4to4_4),  .out_a(a4_4to4_5), .out_b(b4_4to5_4), .out_c(matrixC4_4));
processing_element pe4_5(.reset(effective_rst), .clk(clk),  .in_a(a4_4to4_5), .in_b(b3_5to4_5),  .out_a(a4_5to4_6), .out_b(b4_5to5_5), .out_c(matrixC4_5));
processing_element pe4_6(.reset(effective_rst), .clk(clk),  .in_a(a4_5to4_6), .in_b(b3_6to4_6),  .out_a(a4_6to4_7), .out_b(b4_6to5_6), .out_c(matrixC4_6));
processing_element pe4_7(.reset(effective_rst), .clk(clk),  .in_a(a4_6to4_7), .in_b(b3_7to4_7),  .out_a(a4_7to4_8), .out_b(b4_7to5_7), .out_c(matrixC4_7));
processing_element pe5_1(.reset(effective_rst), .clk(clk),  .in_a(a5_0to5_1), .in_b(b4_1to5_1),  .out_a(a5_1to5_2), .out_b(b5_1to6_1), .out_c(matrixC5_1));
processing_element pe5_2(.reset(effective_rst), .clk(clk),  .in_a(a5_1to5_2), .in_b(b4_2to5_2),  .out_a(a5_2to5_3), .out_b(b5_2to6_2), .out_c(matrixC5_2));
processing_element pe5_3(.reset(effective_rst), .clk(clk),  .in_a(a5_2to5_3), .in_b(b4_3to5_3),  .out_a(a5_3to5_4), .out_b(b5_3to6_3), .out_c(matrixC5_3));
processing_element pe5_4(.reset(effective_rst), .clk(clk),  .in_a(a5_3to5_4), .in_b(b4_4to5_4),  .out_a(a5_4to5_5), .out_b(b5_4to6_4), .out_c(matrixC5_4));
processing_element pe5_5(.reset(effective_rst), .clk(clk),  .in_a(a5_4to5_5), .in_b(b4_5to5_5),  .out_a(a5_5to5_6), .out_b(b5_5to6_5), .out_c(matrixC5_5));
processing_element pe5_6(.reset(effective_rst), .clk(clk),  .in_a(a5_5to5_6), .in_b(b4_6to5_6),  .out_a(a5_6to5_7), .out_b(b5_6to6_6), .out_c(matrixC5_6));
processing_element pe5_7(.reset(effective_rst), .clk(clk),  .in_a(a5_6to5_7), .in_b(b4_7to5_7),  .out_a(a5_7to5_8), .out_b(b5_7to6_7), .out_c(matrixC5_7));
processing_element pe6_1(.reset(effective_rst), .clk(clk),  .in_a(a6_0to6_1), .in_b(b5_1to6_1),  .out_a(a6_1to6_2), .out_b(b6_1to7_1), .out_c(matrixC6_1));
processing_element pe6_2(.reset(effective_rst), .clk(clk),  .in_a(a6_1to6_2), .in_b(b5_2to6_2),  .out_a(a6_2to6_3), .out_b(b6_2to7_2), .out_c(matrixC6_2));
processing_element pe6_3(.reset(effective_rst), .clk(clk),  .in_a(a6_2to6_3), .in_b(b5_3to6_3),  .out_a(a6_3to6_4), .out_b(b6_3to7_3), .out_c(matrixC6_3));
processing_element pe6_4(.reset(effective_rst), .clk(clk),  .in_a(a6_3to6_4), .in_b(b5_4to6_4),  .out_a(a6_4to6_5), .out_b(b6_4to7_4), .out_c(matrixC6_4));
processing_element pe6_5(.reset(effective_rst), .clk(clk),  .in_a(a6_4to6_5), .in_b(b5_5to6_5),  .out_a(a6_5to6_6), .out_b(b6_5to7_5), .out_c(matrixC6_5));
processing_element pe6_6(.reset(effective_rst), .clk(clk),  .in_a(a6_5to6_6), .in_b(b5_6to6_6),  .out_a(a6_6to6_7), .out_b(b6_6to7_6), .out_c(matrixC6_6));
processing_element pe6_7(.reset(effective_rst), .clk(clk),  .in_a(a6_6to6_7), .in_b(b5_7to6_7),  .out_a(a6_7to6_8), .out_b(b6_7to7_7), .out_c(matrixC6_7));
processing_element pe7_1(.reset(effective_rst), .clk(clk),  .in_a(a7_0to7_1), .in_b(b6_1to7_1),  .out_a(a7_1to7_2), .out_b(b7_1to8_1), .out_c(matrixC7_1));
processing_element pe7_2(.reset(effective_rst), .clk(clk),  .in_a(a7_1to7_2), .in_b(b6_2to7_2),  .out_a(a7_2to7_3), .out_b(b7_2to8_2), .out_c(matrixC7_2));
processing_element pe7_3(.reset(effective_rst), .clk(clk),  .in_a(a7_2to7_3), .in_b(b6_3to7_3),  .out_a(a7_3to7_4), .out_b(b7_3to8_3), .out_c(matrixC7_3));
processing_element pe7_4(.reset(effective_rst), .clk(clk),  .in_a(a7_3to7_4), .in_b(b6_4to7_4),  .out_a(a7_4to7_5), .out_b(b7_4to8_4), .out_c(matrixC7_4));
processing_element pe7_5(.reset(effective_rst), .clk(clk),  .in_a(a7_4to7_5), .in_b(b6_5to7_5),  .out_a(a7_5to7_6), .out_b(b7_5to8_5), .out_c(matrixC7_5));
processing_element pe7_6(.reset(effective_rst), .clk(clk),  .in_a(a7_5to7_6), .in_b(b6_6to7_6),  .out_a(a7_6to7_7), .out_b(b7_6to8_6), .out_c(matrixC7_6));
processing_element pe7_7(.reset(effective_rst), .clk(clk),  .in_a(a7_6to7_7), .in_b(b6_7to7_7),  .out_a(a7_7to7_8), .out_b(b7_7to8_7), .out_c(matrixC7_7));
assign a_data_out = {a7_7to7_8,a6_7to6_8,a5_7to5_8,a4_7to4_8,a3_7to3_8,a2_7to2_8,a1_7to1_8,a0_7to0_8};
assign b_data_out = {b7_7to8_7,b7_6to8_6,b7_5to8_5,b7_4to8_4,b7_3to8_3,b7_2to8_2,b7_1to8_1,b7_0to8_0};

endmodule
/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_vector_2_1_128_1024_10
  (clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_byteen, c_we);

input clk;
input resetn;

input [2-1:0] a_en;
input [2-1:0] b_en;
input [2*9-1:0] a_reg,b_reg,c_reg;
output [2*128-1:0] a_readdataout, b_readdataout;
wire [2*128-1:0] a_temp, b_temp;

input [2*128-1:0] c_writedatain;
input [2*128/8-1:0] c_byteen;
input [2-1:0] c_we;


          ram_wrapper_9_512_128 reg_file1_0(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
  	    .address_a(c_reg[8:0]),
  	    .address_b(a_reg[8:0]),
  	    .wren_a(c_we[0] &  c_byteen[0]),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[0*128 +: 128]),
  	    .data_b(0),
  	    .out_a(a_temp[0*128 +: 128]),
  	    .out_b(a_readdataout[0*128 +: 128])
          );
  
          ram_wrapper_9_512_128 reg_file2_0(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(b_en),
  	    .address_a(c_reg[8:0]),
  	    .address_b(b_reg[8:0]),
  	    .wren_a(c_we[0] &  c_byteen[0]),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[127:0]),
  	    .data_b(0),
  	    .out_a(b_temp[0*128 +: 128]),
  	    .out_b(b_readdataout[127:0])
          );

          ram_wrapper_9_512_128 reg_file1_1(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
  	    .address_a(c_reg[8:0]),
  	    .address_b(a_reg[8:0]),
  	    .wren_a(c_we[1] &  c_byteen[1]),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[1*128 +: 128]),
  	    .data_b(0),
  	    .out_a(a_temp[1*128 +: 128]),
  	    .out_b(a_readdataout[1*128 +: 128])
          );
  
          ram_wrapper_9_512_128 reg_file2_1(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(b_en),
  	    .address_a(c_reg[8:0]),
  	    .address_b(b_reg[8:0]),
  	    .wren_a(c_we[1] &  c_byteen[1]),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[127:0]),
  	    .data_b(0),
  	    .out_a(b_temp[1*128 +: 128]),
  	    .out_b(b_readdataout[127:0])
          );

endmodule


module local_shifter_33_2_ARITHMATIC(
  data,
  distance,
  direction,
  result
);

parameter LPM_SHIFTTYPE = "ARITHMATIC";

input [33-1:0] data;
input [2-1:0] distance;
input direction;

output reg [33-1:0] result;
reg [33-1:0] arith_reg; 
always @* begin
  arith_reg = {33{1'b1}};
    if (LPM_SHIFTTYPE == "ARITHMETIC") begin
    if(direction)begin
      result = data << distance;
    end
    else begin
      if(data[33-1] == 1'b1)
          result =  ((arith_reg <<(33 - distance))|| (data >> distance));
      else
          result = data >> distance;
    end
  end
  else begin
    if(direction == 1'b0)begin
        result = data << distance;
    end
    else begin
        result = data >> distance;
    end
  end 
end
endmodule
///////////////////////////////////////////////////////
// Processing element. Finds sum, min or max depending on mode
///////////////////////////////////////////////////////
module reduction_processing_element_10_13 (
  A, B, OUT, MODE
);

input [10-1:0] A;
input [10-1:0] B;
output [13-1:0] OUT;
input [1:0] MODE;

wire [13-1:0] greater;
wire [13-1:0] smaller;
wire [13-1:0] sum;

assign greater = (A>B) ? A : B;
assign smaller = (A<B) ? A : B;
assign sum = A + B;

assign OUT = (MODE==0) ? sum : 
             (MODE==1) ? greater :
             smaller;

endmodule
///////////////////////////////////////////////////////
// Processing element. Finds sum, min or max depending on mode
///////////////////////////////////////////////////////
module reduction_processing_element_13_13 (
  A, B, OUT, MODE
);

input [13-1:0] A;
input [13-1:0] B;
output [13-1:0] OUT;
input [1:0] MODE;

wire [13-1:0] greater;
wire [13-1:0] smaller;
wire [13-1:0] sum;

assign greater = (A>B) ? A : B;
assign smaller = (A<B) ? A : B;
assign sum = A + B;

assign OUT = (MODE==0) ? sum : 
             (MODE==1) ? greater :
             smaller;

endmodule

`define DISPATCHWIDTH 164
 

/******************************************************************************
 * Vector Pipeline
 *
 *                vl (arrives from vector control pipeline in Stage2)
 *                vs
 *                vc
 *|   Stage1  |  Stage2   |  Stage3  |  Stage4  |  Stage5  | Stage6  |  Stage7 |
 *| instr     |           |          | dispatch |          |         |
 *| decode    | control   | bank     | regfile  | execute  | WB
 *
 *****************************************************************************/

//`include "vdispatcher.v"
//`include "vlane_alu.v"
//`include "vmul_unit.v"
//`include "vmem_unit.v"
//`include "vlane_flagalu.v"
//`include "matmul_unit.v"

`define LO(x,b) ((x)&~({1024{1'b1}}<<b))

module vlanes_4_2_2_1_4_128_7_4_2_32_5_2_1_1_128_7_128_7_32_32_32_5 (
    clk,
    resetn,

    // Instruction interface
    instr,
    instr_en,     // tells when instr is valid and available
    instr_wait,   // if high says vpu is not ready to receive

    stall_in,
    is_stalled,
    has_memop,

    // Control register values 
    vc_in,
    vl_in,
    vbase_in,
    vinc_in,
    vstride_in,
    vs_in,
    matmul_masks_in,

    // vs Writeback
    vs_writedata,
    vs_we,       // Actually issues write command for vs (after done stalling)
    vs_wetrack,  // says there exist a scalar write operation in pipe
    vs_dst,

    // Data memory interface
    dbus_address,
    dbus_en,
    dbus_we,
    dbus_byteen,
    dbus_writedata,
    dbus_readdata,
    dbus_cachematch,
    dbus_cachemiss,
    dbus_prefetch,    //Prefetch hint
    dbus_wait

    );

parameter NUMMEMPARALLELLANES=2;
parameter LOG2NUMMEMPARALLELLANES=1;
parameter LOG2NUMMULLANES=4;

parameter VRIDWIDTH=5;
parameter VELMIDWIDTH=7-2;
parameter REGIDWIDTH=VRIDWIDTH+7-2;
parameter BANKREGIDWIDTH=VRIDWIDTH+7-2-1;

// Register identifier = { vr[0-31], element }

`define VRID_RANGE (REGIDWIDTH-1:REGIDWIDTH-VRIDWIDTH)
`define VRELM_RANGE (REGIDWIDTH-VRIDWIDTH-1:0)

// NUMFUS = ALU*2 + MUL + MEM + FALU*MEMBANKS + MATMUL(1) + BFLOAT
// UNITS(2) + ACTIVATION + TRP
parameter NUMFUS=4+2*(2-1)*1+1+4; //Adding 1 for matmul FU
parameter FU_ALU=0;
parameter FU_MUL=FU_ALU+(2-1)*1+1;
parameter FU_MEM=FU_MUL+1;
parameter FU_FALU=FU_MEM+1;
parameter FU_MATMUL=FU_FALU+1;
parameter FU_BFADDER = FU_MATMUL + 1;
parameter FU_BFMULT = FU_BFADDER + 1;
parameter FU_ACT = FU_BFMULT + 1;
parameter FU_TRP = FU_ACT + 1;

input clk;
input resetn;

//Instruction Format:
//9 - op[21] mask 1/0
//8 - op[23] scalar/vec src2
//7 - op[22] scalar/vec src1
//6:0 - {op[24],op[5:0]} op

input [31:0] instr;
input instr_en;     // tells when instr is valid and available
output instr_wait;   // if high says vpu is not ready to receive

input [3:0] stall_in;  
output [`MAX_PIPE_STAGES-1:0] is_stalled;
output has_memop;

// Control register values
input  [ 32-1 : 0 ]   vc_in;
input  [ 32-1 : 0 ]   vl_in;
input  [ 32-1 : 0 ]   vbase_in;
input  [ 32-1 : 0 ]   vinc_in;
input  [ 32-1 : 0 ]   vstride_in;
input  [ 32-1 : 0 ]   vs_in;
input  [3*`MAT_MUL_SIZE-1 : 0]  matmul_masks_in;

// vs Writeback
output       [ 32-1 : 0 ]   vs_writedata;
output                           vs_we;
output               [ 5 : 2 ]   vs_wetrack;
output [ 5-1 : 0 ]   vs_dst;

// Data memory interface
output  [ 31 : 0 ]  dbus_address;
output              dbus_en;
output              dbus_we;
output  [ (128/8)-1 : 0 ]   dbus_byteen;
output  [ 128-1 : 0 ]  dbus_writedata;
input   [ 128-1 : 0 ]  dbus_readdata;
input               dbus_cachematch;
input               dbus_cachemiss;
input               dbus_wait;
output  [ 31 : 0 ]  dbus_prefetch;


//`include "visa.v"
parameter COP2_VADD           = 'b1000000000;
parameter COP2_VADD_U         = 'b1000000001;
parameter COP2_VSUB           = 'b1000000010;
parameter COP2_VSUB_U         = 'b1000000011;
parameter COP2_VMULHI         = 'b1000000100;
parameter COP2_VMULHI_U       = 'b1000000101;
parameter COP2_VDIV           = 'b1000000110; //Using as matmul
parameter COP2_VBFADD         = 'b0100000001; //Using BF16 add
parameter COP2_VBFMULT        = 'b0100000010; //Using BF16 MULT
parameter COP2_VACT           = 'b0100000011; //Using ACT
parameter COP2_VTRP           = 'b0100000100; //Using ACT
parameter COP2_VDIV_U         = 'b1000000111;
parameter COP2_VMOD           = 'b1000001000;
parameter COP2_VMOD_U         = 'b1000001001;
parameter COP2_VCMP_EQ        = 'b1000001010;
parameter COP2_VCMP_NE        = 'b1000001100;
parameter COP2_VCMP_LT        = 'b1000001110;
parameter COP2_VCMP_U_LT      = 'b1000001111;
parameter COP2_VCMP_LE        = 'b1000010000;
parameter COP2_VCMP_U_LE      = 'b1000010001;
parameter COP2_VMIN           = 'b1000010010;
parameter COP2_VMIN_U         = 'b1000010011;
parameter COP2_VMAX           = 'b1000010100;
parameter COP2_VMAX_U         = 'b1000010101;
parameter COP2_VMULLO         = 'b1000010110;
parameter COP2_VABS           = 'b1000010111;
parameter COP2_VAND           = 'b1000011000;
parameter COP2_VOR            = 'b1000011001;
parameter COP2_VXOR           = 'b1000011010;
parameter COP2_VNOR           = 'b1000011011;
parameter COP2_VSLL           = 'b1000011100;
parameter COP2_VSRL           = 'b1000011101;
parameter COP2_VSRA           = 'b1000011110;
parameter COP2_VSAT_B         = 'b1000011111;
parameter COP2_VSAT_H         = 'b1001011111;
parameter COP2_VSAT_W         = 'b1010011111;
parameter COP2_VSAT_SU_B      = 'b1000100000;
parameter COP2_VSAT_SU_H      = 'b1001100000;
parameter COP2_VSAT_SU_W      = 'b1010100000;
parameter COP2_VSAT_SU_L      = 'b1011100000;
parameter COP2_VSAT_U_B       = 'b1000100001;
parameter COP2_VSAT_U_H       = 'b1001100001;
parameter COP2_VSAT_U_W       = 'b1010100001;
parameter COP2_VSADD          = 'b1000100010;
parameter COP2_VSADD_U        = 'b1000100011;
parameter COP2_VSSUB          = 'b1000100100;
parameter COP2_VSSUB_U        = 'b1000100101;
parameter COP2_VSRR           = 'b1000100110;
parameter COP2_VSRR_U         = 'b1000100111;
parameter COP2_VSLS           = 'b1000101000;
parameter COP2_VSLS_U         = 'b1000101001;
parameter COP2_VXUMUL         = 'b1000101010;
parameter COP2_VXUMUL_U       = 'b1000101011;
parameter COP2_VXLMUL         = 'b1000101100;
parameter COP2_VXLMUL_U       = 'b1000101101;
parameter COP2_VXUMADD        = 'b1000101110;
parameter COP2_VXUMADD_U      = 'b1000101111;
parameter COP2_VXUMSUB        = 'b1000110000;
parameter COP2_VXUMSUB_U      = 'b1000110001;
parameter COP2_VXLMADD        = 'b1000110010;
parameter COP2_VXLMADD_U      = 'b1000110011;
parameter COP2_VXLMSUB        = 'b1000110100;
parameter COP2_VXLMSUB_U      = 'b1000110101;
parameter COP2_VINS_VV        = 'b1100000000;
parameter COP2_VINS_SV        = 'b1110000001;
parameter COP2_VEXT_VV        = 'b1100000010;
parameter COP2_VEXT_SV        = 'b1100000011;
parameter COP2_VEXT_U_SV      = 'b1100000100;
parameter COP2_VCOMPRESS      = 'b1100000101;
parameter COP2_VEXPAND        = 'b1100000110;
parameter COP2_VMERGE         = 'b1100000111;
parameter COP2_VFINS          = 'b1110001000;
parameter COP2_VEXTHALF       = 'b1100001001;
parameter COP2_VHALF          = 'b1100001010;
parameter COP2_VHALFUP        = 'b1100001011;
parameter COP2_VHALFDN        = 'b1100001100;
parameter COP2_VSATVL         = 'b1100001101;
parameter COP2_VFAND          = 'b1100001110;
parameter COP2_VFOR           = 'b1100001111;
parameter COP2_VFXOR          = 'b1100010000;
parameter COP2_VFNOR          = 'b1100010001;
parameter COP2_VFCLR          = 'b1100010010;
parameter COP2_VFSET          = 'b1100010011;
parameter COP2_VIOTA          = 'b1100010100;
parameter COP2_VCIOTA         = 'b1100010101;
parameter COP2_VFPOP          = 'b1100010110;
parameter COP2_VFFF1          = 'b1100010111;
parameter COP2_VFFL1          = 'b1100011000;
parameter COP2_VFSETBF        = 'b1100011001;
parameter COP2_VFSETIF        = 'b1100011010;
parameter COP2_VFSETOF        = 'b1100011011;
parameter COP2_VFMT8          = 'b1100011100;
parameter COP2_VFMF8          = 'b1100011101;
parameter COP2_VFCLR8         = 'b1100011110;
parameter COP2_VFOR8          = 'b1100011111;
parameter COP2_VFLD           = 'b1100100000;
parameter COP2_VLD_B          = 'b1100100001;
parameter COP2_VLD_H          = 'b1101100001;
parameter COP2_VLD_W          = 'b1110100001;
parameter COP2_VLD_L          = 'b1111100001;
parameter COP2_VLD_U_B        = 'b1100100010;
parameter COP2_VLD_U_H        = 'b1101100010;
parameter COP2_VLD_U_W        = 'b1110100010;
parameter COP2_VLDS_B         = 'b1100100011;
parameter COP2_VLDS_H         = 'b1101100011;
parameter COP2_VLDS_W         = 'b1110100011;
parameter COP2_VLDS_L         = 'b1111100011;
parameter COP2_VLDS_U_B       = 'b1100100100;
parameter COP2_VLDS_U_H       = 'b1101100100;
parameter COP2_VLDS_U_W       = 'b1110100100;
parameter COP2_VLDX_B         = 'b1100100101;
parameter COP2_VLDX_H         = 'b1101100101;
parameter COP2_VLDX_W         = 'b1110100101;
parameter COP2_VLDX_L         = 'b1111100101;
parameter COP2_VLDX_U_B       = 'b1100100110;
parameter COP2_VLDX_U_H       = 'b1101100110;
parameter COP2_VLDX_U_W       = 'b1110100110;
parameter COP2_VFST           = 'b1100101000;
parameter COP2_VST_B          = 'b1100101001;
parameter COP2_VST_H          = 'b1101101001;
parameter COP2_VST_W          = 'b1110101001;
parameter COP2_VST_L          = 'b1111101001;
parameter COP2_VSTS_B         = 'b1100101010;
parameter COP2_VSTS_H         = 'b1101101010;
parameter COP2_VSTS_W         = 'b1110101010;
parameter COP2_VSTS_L         = 'b1111101010;
parameter COP2_VSTX_B         = 'b1100101011;
parameter COP2_VSTX_H         = 'b1101101011;
parameter COP2_VSTX_W         = 'b1110101011;
parameter COP2_VSTX_L         = 'b1111101011;
parameter COP2_VSTXO_B        = 'b1100101100;
parameter COP2_VSTXO_H        = 'b1101101100;
parameter COP2_VSTXO_W        = 'b1110101100;
parameter COP2_VSTXO_L        = 'b1111101100;
parameter COP2_VMCTS          = 'b1101110000;
parameter COP2_VMSTC          = 'b1101110001;
parameter COP2_CFC2           = 'b0000111000;
parameter COP2_CTC2           = 'b0000111010;
parameter COP2_MTC2           = 'b0000111011;


parameter BIT_VSSRC2=6;
parameter BIT_VSSRC1=7;

parameter ALUOP_ZERO    =11'b00100000101,
          ALUOP_ADD     =11'b00010000000,
          ALUOP_ADDU    =11'b00000000000,
          ALUOP_SUB     =11'b00011000000,
          ALUOP_SUBU    =11'b00001000000,
          ALUOP_CMP_EQ  =11'b00011000000,
          ALUOP_CMP_NEQ =11'b00011000001,
          ALUOP_CMP_LT  =11'b00011000010,
          ALUOP_CMP_LE  =11'b00011000011,
          ALUOP_CMP_LT_U=11'b00001000010,
          ALUOP_CMP_LE_U=11'b00001000011,
          ALUOP_AND     =11'b00000000101,
          ALUOP_OR      =11'b00000000001,
          ALUOP_XOR     =11'b00000001001,
          ALUOP_NOR     =11'b00000001101,
          ALUOP_MIN     =11'b00111010001,
          ALUOP_MIN_U   =11'b00101010001,
          ALUOP_MAX     =11'b00111100001,
          ALUOP_MAX_U   =11'b00101100001,
          ALUOP_ABS     =11'b11111000010,
          ALUOP_MERGE   =11'b10100000011;

parameter SATSUMOP_NOP=2'b00,
          SATSUMOP_VS =2'b11,
          SATSUMOP_VSU=2'b10;

parameter SATSIZEOP_VSATUW=4'b0000,
          SATSIZEOP_VSATUB=4'b0001,
          SATSIZEOP_VSATUH=4'b0010,
          SATSIZEOP_VSATW  =4'b1100,
          SATSIZEOP_VSATB  =4'b1101,
          SATSIZEOP_VSATH  =4'b1110,
          SATSIZEOP_VSATSUW=4'b0100,
          SATSIZEOP_VSATSUB=4'b0101,
          SATSIZEOP_VSATSUH=4'b0110;

parameter MULOP_ZERO  =5'b00000,
          MULOP_LMULU =5'b10110,
          MULOP_UMULU =5'b10111,
          MULOP_LMUL  =5'b10100,
          MULOP_UMUL  =5'b10101,
          MULOP_MULUHI=5'b00111,
          MULOP_MULULO=5'b00110,
          MULOP_MULHI =5'b00101,
          MULOP_MULLO =5'b00100,
          MULOP_SLL   =5'b00010,
          MULOP_SLS   =5'b01010,
          MULOP_SLSU  =5'b01000,
          MULOP_SRL   =5'b00011,
          MULOP_SRA   =5'b00001;

parameter MEMOP_SHIFT=7'b0000000, 
          MEMOP_LDUB=7'b1000000,
          MEMOP_LDUH=7'b1000100,
          MEMOP_LDUW=7'b1001000,
          MEMOP_LDB=7'b1000010,
          MEMOP_LDH=7'b1000110,
          MEMOP_LDW=7'b1001010,
          MEMOP_LDSUB=7'b1010000,
          MEMOP_LDSUH=7'b1010100,
          MEMOP_LDSUW=7'b1011000,
          MEMOP_LDSB=7'b1010010,
          MEMOP_LDSH=7'b1010110,
          MEMOP_LDSW=7'b1011010,
          MEMOP_LDXUB=7'b1100000,
          MEMOP_LDXUH=7'b1100100,
          MEMOP_LDXUW=7'b1101000,
          MEMOP_LDXB=7'b1100010,
          MEMOP_LDXH=7'b1100110,
          MEMOP_LDXW=7'b1101010,
          MEMOP_STB=7'b1000001,
          MEMOP_STH=7'b1000101,
          MEMOP_STW=7'b1001001,
          MEMOP_STSB=7'b1010001,
          MEMOP_STSH=7'b1010101,
          MEMOP_STSW=7'b1011001,
          MEMOP_STXB=7'b1100001,
          MEMOP_STXH=7'b1100101,
          MEMOP_STXW=7'b1101001;

parameter FLAGOP_AND=3'b000,
          FLAGOP_OR= 3'b001,
          FLAGOP_XOR=3'b010,
          FLAGOP_NOR=3'b011,
          FLAGOP_CLR=3'b100,
          FLAGOP_SET=3'b101;

wire                        [ 9 : 0 ]   ir_op;
wire                        [ 4 : 0 ]   ir_src2;
wire                        [ 4 : 0 ]   ir_src1;
wire                        [ 4 : 0 ]   ir_dst;
wire                                    ir_mask;

// Control register saved values
reg                       pipe_advance_s2_r;
reg   [ 32-1 : 0 ]   vc_in_saved;
reg   [ 32-1 : 0 ]   vl_in_saved;
reg   [ 32-1 : 0 ]   vbase_in_saved;
reg   [ 32-1 : 0 ]   vinc_in_saved;
reg   [ 32-1 : 0 ]   vstride_in_saved;
reg   [ 32-1 : 0 ]   vs_in_saved;

// Control register values
//TODO: Rethink whether the following vars (vc, vl, vbase, vinc, vstride)
//need to be change to use `MAX_PIPE_STAGES. I think these variables are 
//only used for memory operations and for that, we only need 6 stages.
//Not a big deal because most of these are not used much. Synthesis tool
//will optimize out the unnecessary bits anyway.
wire  [ 32-1 : 0 ]   vc[`MAX_PIPE_STAGES-1:2];
wire  [ 32-1 : 0 ]   vl[`MAX_PIPE_STAGES-1:2];
wire  [ 32-1 : 0 ]   vbase[`MAX_PIPE_STAGES-1:2];
reg   [ 32-1 : 0 ]   vbase_s4;
wire  [ 32-1 : 0 ]   vinc[`MAX_PIPE_STAGES-1:2];
wire  [ 32-1 : 0 ]   vstride[`MAX_PIPE_STAGES-1:2];
wire  [ 32-1 : 0 ]   vs[`MAX_PIPE_STAGES-1:2];
reg   [ 32-1 : 0 ]   vs_s3[2-1:0];
reg   [ 32-1 : 0 ]   vs_s4[NUMFUS-1:0];
reg   [ 32-1 : 0 ]   vc_s3[2-1:0];
reg   [ 32-1 : 0 ]   vc_s4[NUMFUS-1:0];

// Vector register file signals
reg   [ 2*BANKREGIDWIDTH-1 : 0 ]   vr_a_reg;
wire     [ 2*8*4*4-1 : 0 ]   _vr_a_readdataout;
reg                         [2-1:0]   vr_a_en;
reg   [ 2*BANKREGIDWIDTH-1 : 0 ]   vr_b_reg;
wire     [ 2*8*4*4-1 : 0 ]   _vr_b_readdataout;
reg                         [2-1:0]   vr_b_en;
reg   [ 2*BANKREGIDWIDTH-1 : 0 ]   _vr_c_reg;
reg      [ 2*8*4*4-1 : 0 ]   _vr_c_writedatain;
reg        [ 2*4*4-1 : 0 ]   vr_c_byteen;
reg                         [2-1:0]   vr_c_we;

reg          [ 32*4-1 : 0 ]   vr_a_readdataout[2-1:0];
reg          [ 32*4-1 : 0 ]   vr_b_readdataout[2-1:0];
reg          [ 32*4-1 : 0 ]   vr_c_writedatain[2-1:0];
reg           [ REGIDWIDTH-1 : 0 ]   vr_c_reg[2-1:0]; //for testbench and debugging

// Flag register file signals
reg       [ 2*BANKREGIDWIDTH-1 : 0 ]   vf_a_reg;
wire               [ 2*4-1 : 0 ]   vf_a_readdataout;
reg                            [ 2-1:0]   vf_a_en;
reg       [ 2*BANKREGIDWIDTH-1 : 0 ]   vf_b_reg;
wire               [ 2*4-1 : 0 ]   vf_b_readdataout;
reg                            [ 2-1:0]   vf_b_en;
reg       [ 2*BANKREGIDWIDTH-1 : 0 ]   vf_c_reg;
reg                [ 2*4-1 : 0 ]   vf_c_writedatain;
reg                            [ 2-1:0]   vf_c_we;

wire             [ REGIDWIDTH-1 : 0 ]   wb_dst[NUMFUS-1:0];
wire                     [NUMFUS-1:0]   wb_dst_we;
wire               [ 4-1 : 0 ]   wb_dst_mask[NUMFUS-1:0];

wire                        [ `MAX_PIPE_STAGES-1 : 4 ]   dst_we[NUMFUS-1:0];
wire             [ REGIDWIDTH-1 : 0 ]   dst[NUMFUS-1:0][`MAX_PIPE_STAGES-1:4];
wire               [ 4-1 : 0 ]   dst_mask[NUMFUS-1:0][`MAX_PIPE_STAGES-1:4];
wire             [ REGIDWIDTH-1 : 0 ]   dst_s2;
reg              [ REGIDWIDTH-1 : 0 ]   _dst_s3[2-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   dst_s3[2-1:0];
reg     [ 2*REGIDWIDTH-1 : 0 ]   t_dst_s3;
reg              [ REGIDWIDTH-1 : 0 ]   dst_s4[NUMFUS-1:0];
reg                  [ NUMFUS-1 : 0 ]   dst_we_s4;
reg  [(1+(2-1)*1)*REGIDWIDTH-1:0] alu_dst[4:4];
reg  [(1+(2-1)*1)-1:0]            alu_dst_we[4:4];
reg  [(1+(2-1)*1)*REGIDWIDTH-1:0] falu_dst[4:4];
reg  [(1+(2-1)*1)-1:0]            falu_dst_we[4:4];

wire                                    imask_s2;
reg                [ 2-1 : 0 ]   imask_s3;

wire             [ REGIDWIDTH-1 : 0 ]   src1_s2;
wire             [ REGIDWIDTH-1 : 0 ]   src2_s2;
reg              [ REGIDWIDTH-1 : 0 ]   src1_s3[2-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   src2_s3[2-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   _src1_s3[2-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   _src2_s3[2-1:0];
wire                                    src1scalar_s2;
wire                                    src2scalar_s2;
reg                    [2-1:0]   src1scalar_s3;
reg                    [2-1:0]   src2scalar_s3;
reg                      [NUMFUS-1:0]   src1scalar_s4;
reg                      [NUMFUS-1:0]   src2scalar_s4;

reg                    [4-1:0]   lane_en[2-1:0];
reg                    [4-1:0]   vlane_en[NUMFUS-1:0];
reg                                     mem_last_subvector_s4;

reg                     [7-1:0]   src_start_delayed;
wire                    [7-1:0]   src_elm;
wire                    [7-1:0]   src_limit;
reg                     [7-1:0]   src_limit_s3[2-1:0];
wire                    [7-1:0]   src_start;

wire                    [32-1:0]   total_shamt;
wire                    [7-1:0]   dst_start;

reg          [ 32*4-1 : 0 ]   vr_src1[NUMFUS-1:0];
reg          [ 32*4-1 : 0 ]   vr_src2[NUMFUS-1:0];
wire         [ 32*4-1 : 0 ]   matmul_out;
wire         [ 32*4-1 : 0 ]   bfadder_result_s5;
wire         [ 32*4-1 : 0 ]   bfmult_result_s5;
wire         [ 32*4-1 : 0 ]   act_result_s5;
wire         [ 32*4-1 : 0 ]   trp_out;
reg                [ 4-1 : 0 ]   vf_src1[NUMFUS-1:0];
reg                [ 4-1 : 0 ]   vf_src2[NUMFUS-1:0];
reg                [ 4-1 : 0 ]   vmask[NUMFUS-1:0];
reg                [ 4-1 : 0 ]   vmask_final[2-1:0];

reg        [ 32*4-1 : 0 ]   vstrideoffset_s4;
wire     [ 32*4-1 : 0 ]   load_result_s5;
wire               [ 4-1 : 0 ]   load_result_mask_s5;

wire [ 32*4-1 : 0 ] alu_result_s5[(2-1)*1:0];
wire           [ 4-1 : 0 ] alu_cmpresult_s4[(2-1)*1:0];
wire           [ 4-1 : 0 ] flagalu_result_s4[(2-1)*1:0];
wire           [ 4-1 : 0 ] flagalu_result_s5[(2-1)*1:0];
wire [ 32*4-1 : 0 ] mulshift_result_s5;

//Support 1 Lane processor
// wire [((2>0) ? 2 : 1)-1:0] elmshamt[`MAX_PIPE_STAGES-1:2];
wire [2-1:0] elmshamt[`MAX_PIPE_STAGES-1:2];

reg ctrl1_vr_a_en; // SRC1
reg ctrl1_vr_b_en; // SRC2
reg ctrl1_vr_c_en; // SRC3
reg ctrl1_vr_d_we; // DEST
reg ctrl1_vf_a_en;
reg ctrl1_vf_b_en;
reg ctrl1_vf_c_we;
reg ctrl1_vs_we;
reg ctrl1_usesvssel;    // 1-if instruction can have .sv/.vs variants
reg [1:0] ctrl1_elmshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
reg [1:0] ctrl1_srcshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
reg ctrl1_srclimit_sel;  //0-vl, 1 vl+vindex
reg [1:0] ctrl1_dstshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
reg ctrl1_setvlto1;
reg ctrl1_mem_dir_left;  //1-left, 0-right
reg ctrl1_rshiftnonzero;
reg [10:0] ctrl1_alu_op;
reg [1:0] ctrl1_satsum_op;
reg [3:0] ctrl1_satsize_op;
reg [4:0] ctrl1_mulshift_op;
reg ctrl1_matmul_en;
reg ctrl1_bfadder_en;
reg ctrl1_bfmult_en;
reg ctrl1_act_en;
reg ctrl1_trp_en;
reg ctrl1_memunit_en;
reg ctrl1_mem_en;
reg [6:0] ctrl1_memunit_op;
reg ctrl1_ismasked;
reg [2:0] ctrl1_flagalu_op;
reg ctrl1_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg ctrl1_volatiledest;
reg ctrl1_vrdest_sel;  //0-dest, 1-src2
reg ctrl1_vf_a_sel; //0-0/1 from instr, 1-src1

wire [5:4] ctrl_vs_we;
wire [6:0] ctrl_memunit_op[`MAX_PIPE_STAGES-1:1] ;

wire ctrl2_vr_a_en; // SRC1
wire ctrl2_vr_b_en; // SRC2
wire ctrl2_vr_c_we; // DEST
wire ctrl2_vf_a_en;
wire ctrl2_vf_b_en;
wire ctrl2_vf_c_we;
wire ctrl2_vs_we;
wire ctrl2_useslanes;
wire [1:0] ctrl2_elmshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
wire [1:0] ctrl2_srcshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
wire ctrl2_srclimit_sel;  //0-vl, 1 vl+vindex
wire [1:0] ctrl2_dstshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
wire ctrl2_setvlto1;
wire ctrl2_mem_dir_left;  //1-left, 0-right
wire ctrl2_rshiftnonzero;
wire [10:0] ctrl2_alu_op;
wire [1:0] ctrl2_satsum_op;
wire [3:0] ctrl2_satsize_op;
wire [4:0] ctrl2_mulshift_op;
wire ctrl2_memunit_en;
wire ctrl2_mem_en;
wire [6:0] ctrl2_memunit_op;
wire ctrl2_ismasked;
wire [2:0] ctrl2_flagalu_op;
wire ctrl2_vf_wbsel;   //0-flag ALU, 1-normal ALU
wire ctrl2_volatiledest;
wire ctrl2_vf_a_sel; //0-0/1 from instr, 1-src1
wire ctrl2_mulshift_en;
wire ctrl2_matmul_en;
wire ctrl2_bfadder_en;
wire ctrl2_bfmult_en;
wire ctrl2_act_en;
wire ctrl2_trp_en;
wire ctrl2_alufalu_en;

reg [2-1:0] ctrl3_vr_a_en; // SRC1
reg [2-1:0] ctrl3_vr_b_en; // SRC2
reg [2-1:0] ctrl3_vr_c_we; // DEST
reg [2-1:0] ctrl3_vf_a_en;
reg [2-1:0] ctrl3_vf_b_en;
reg [2-1:0] ctrl3_vf_c_we;
reg [2-1:0] ctrl3_vs_we;
reg [2-1:0] ctrl3_useslanes;
reg [2-1:0] ctrl3_mem_dir_left;
reg [2-1:0] ctrl3_rshiftnonzero;
reg [10:0] ctrl3_alu_op[2-1:0];
reg [1:0] ctrl3_satsum_op[2-1:0];
reg [3:0] ctrl3_satsize_op[2-1:0];
reg [4:0] ctrl3_mulshift_op[2-1:0];
reg [2-1:0] ctrl3_memunit_en;
reg [2-1:0] ctrl3_mem_en;
reg [6:0] ctrl3_memunit_op[2-1:0];
reg [2-1:0] ctrl3_ismasked;
reg [2:0] ctrl3_flagalu_op[2-1:0];
reg [2-1:0] ctrl3_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg [2-1:0] ctrl3_volatiledest;
reg [2-1:0] ctrl3_vf_a_sel; //0-0/1 from instr, 1-src1
reg [2-1:0] ctrl3_mulshift_en;
reg [2-1:0] ctrl3_matmul_en;
reg [2-1:0] ctrl3_trp_en;
reg [2-1:0] ctrl3_bfadder_en;
reg [2-1:0] ctrl3_bfmult_en;
reg [2-1:0] ctrl3_act_en;
reg [2-1:0] ctrl3_alufalu_en;

reg ctrl4_mem_dir_left;
reg ctrl4_rshiftnonzero;
reg [10:0] ctrl4_alu_op[(2-1)*1:0];
reg [1:0] ctrl4_satsum_op[(2-1)*1:0];
reg [3:0] ctrl4_satsize_op[(2-1)*1:0];
reg [4:0] ctrl4_mulshift_op;
reg ctrl4_memunit_en;
reg ctrl4_mem_en;
reg [6:0] ctrl4_memunit_op;
reg [NUMFUS-1:0] ctrl4_ismasked;
reg [2:0] ctrl4_flagalu_op[(2-1)*1:0];
//reg ctrl4_vf_wbsel[(2-1)*1:0];   //0-flag ALU, 1-normal ALU
reg [(2-1)*1:0] ctrl4_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg ctrl4_volatiledest;
//reg ctrl4_vf_a_sel[(2-1)*1:0]; //0-0/1 from instr, 1-src1
reg [(2-1)*1:0] ctrl4_vf_a_sel; //0-0/1 from instr, 1-src1
reg ctrl4_mulshift_en;
reg ctrl4_matmul_en;
reg ctrl4_trp_en;
reg ctrl4_bfadder_en;
reg ctrl4_bfmult_en;
reg ctrl4_act_en;

wire ctrl5_mem_en;

wire [`VRELM_RANGE] regid_pad;

reg[31:0] bd;
//genvar  ba;
reg[31:0] bi;
reg[31:0] i;
reg[31:0] j;
//genvar  bk;
//genvar  k;
reg[31:0] m;
reg[31:0] n;
reg[31:0] b;
reg[31:0] b3;
reg[31:0] f3;
reg[31:0] bn;
reg[31:0] fn;
reg[31:0] fn2;
reg[31:0] bw;

wire [2*7-1:0] rdelm;
wire [2*7-1:0] wrelm;
// MSb of count entries indicates if instruction is dead or not.
wire [2*(7-2+1)-1:0] count;
reg  [2-1:0]   last_subvector;
wire [2-1:0]   first_subvector;
reg  [2-1:0] wrongbank_s3;
reg  [2-1:0] alive_s3;
reg  [2-1:0] banksel_s4[NUMFUS-1:0];

wire dispatcher_shift;
wire dispatcher_rotate;

wire [`MAX_PIPE_STAGES-1:0] internal_pipe_advance;
wire [`MAX_PIPE_STAGES-1:0] pipe_advance;
wire [`MAX_PIPE_STAGES-1:0] pipe_squash;
wire stall_srcstart;
wire stall_dispatcher;
wire stall_hazsrc1;
wire stall_hazsrc2;
wire stall_hazfsrc1;
wire stall_hazfsrc2;
wire stall_memunit;
wire _stall_memunit;
wire stall_mulcooldown;
wire stall_mulunit;
wire stall_matmul;

// DEBUG signals for Modelsim
wire  [7:0] D_instr[2:1];
reg   [7:0] D_instr_s3[2-1:0];
reg   [7:0] D_instr_s4[NUMFUS-1:0];
wire  [7:0] D_instr_s5[NUMFUS-1:0];
wire  [7:0] D_instr_s6[NUMFUS-1:0];
reg   [NUMFUS-1:0] D_last_subvector_s4;
wire  [NUMFUS-1:0] D_last_subvector_s5;
wire  [NUMFUS-1:0] D_last_subvector_s6;
wire  [NUMFUS-1:0] D_last_subvector_s31;
wire  [NUMFUS-1:0] D_wb_last_subvector;
reg   [2-1:0] D_last_subvector_done;
reg   [2-1:0] D_wb_instrdone;

// Module instance
  wire [7:0] debuginstrpipe_q;
  assign {D_instr[2],D_instr[1]} = debuginstrpipe_q;

  pipe_8_1  debuginstrpipe (
      .d( {instr[25:24],instr[5:0]} ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[2:1] ),
      .squash( pipe_squash[2:1] ),
      .q(debuginstrpipe_q));



// Module instance
    wire debugintrfupipereg1_0_squashn;
    assign debugintrfupipereg1_0_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg1_0 (
      .d( D_instr_s4[0] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_0_squashn),
      .q(D_instr_s5[0]));

// Module instance
    wire debugintrfupipereg2_0_squashn;
    assign debugintrfupipereg2_0_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_0 (
      .d( D_instr_s5[0] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_0_squashn),
      .q(D_instr_s6[0]));

// Module instance
    wire debuglastpipereg1_0_squashn;
    assign debuglastpipereg1_0_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg1_0 (
      .d( D_last_subvector_s4[0] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg1_0_squashn),
      .q(D_last_subvector_s5[0]));

// Module instance
    wire debuglastpipereg2_0_squashn;
    assign debuglastpipereg2_0_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg2_0 (
      .d( D_last_subvector_s5[0] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg2_0_squashn),
      .q(D_last_subvector_s6[0]));



// Module instance
    wire debugintrfupipereg1_1_squashn;
    assign debugintrfupipereg1_1_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg1_1 (
      .d( D_instr_s4[1] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_1_squashn),
      .q(D_instr_s5[1]));

// Module instance
    wire debugintrfupipereg2_1_squashn;
    assign debugintrfupipereg2_1_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_1 (
      .d( D_instr_s5[1] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_1_squashn),
      .q(D_instr_s6[1]));

// Module instance
    wire debuglastpipereg1_1_squashn;
    assign debuglastpipereg1_1_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg1_1 (
      .d( D_last_subvector_s4[1] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg1_1_squashn),
      .q(D_last_subvector_s5[1]));

// Module instance
    wire debuglastpipereg2_1_squashn;
    assign debuglastpipereg2_1_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg2_1 (
      .d( D_last_subvector_s5[1] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg2_1_squashn),
      .q(D_last_subvector_s6[1]));



// Module instance
    wire debugintrfupipereg1_2_squashn;
    assign debugintrfupipereg1_2_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg1_2 (
      .d( D_instr_s4[2] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_2_squashn),
      .q(D_instr_s5[2]));

// Module instance
    wire debugintrfupipereg2_2_squashn;
    assign debugintrfupipereg2_2_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_2 (
      .d( D_instr_s5[2] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_2_squashn),
      .q(D_instr_s6[2]));

// Module instance
    wire debuglastpipereg1_2_squashn;
    assign debuglastpipereg1_2_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg1_2 (
      .d( D_last_subvector_s4[2] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg1_2_squashn),
      .q(D_last_subvector_s5[2]));

// Module instance
    wire debuglastpipereg2_2_squashn;
    assign debuglastpipereg2_2_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg2_2 (
      .d( D_last_subvector_s5[2] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg2_2_squashn),
      .q(D_last_subvector_s6[2]));



// Module instance
    wire debugintrfupipereg1_3_squashn;
    assign debugintrfupipereg1_3_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg1_3 (
      .d( D_instr_s4[3] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_3_squashn),
      .q(D_instr_s5[3]));

// Module instance
    wire debugintrfupipereg2_3_squashn;
    assign debugintrfupipereg2_3_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_3 (
      .d( D_instr_s5[3] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_3_squashn),
      .q(D_instr_s6[3]));

// Module instance
    wire debuglastpipereg1_3_squashn;
    assign debuglastpipereg1_3_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg1_3 (
      .d( D_last_subvector_s4[3] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg1_3_squashn),
      .q(D_last_subvector_s5[3]));

// Module instance
    wire debuglastpipereg2_3_squashn;
    assign debuglastpipereg2_3_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg2_3 (
      .d( D_last_subvector_s5[3] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg2_3_squashn),
      .q(D_last_subvector_s6[3]));



// Module instance
    wire debugintrfupipereg1_4_squashn;
    assign debugintrfupipereg1_4_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg1_4 (
      .d( D_instr_s4[4] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_4_squashn),
      .q(D_instr_s5[4]));

// Module instance
    wire debugintrfupipereg2_4_squashn;
    assign debugintrfupipereg2_4_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_4 (
      .d( D_instr_s5[4] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_4_squashn),
      .q(D_instr_s6[4]));

// Module instance
    wire debuglastpipereg1_4_squashn;
    assign debuglastpipereg1_4_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg1_4 (
      .d( D_last_subvector_s4[4] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg1_4_squashn),
      .q(D_last_subvector_s5[4]));

// Module instance
    wire debuglastpipereg2_4_squashn;
    assign debuglastpipereg2_4_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg2_4 (
      .d( D_last_subvector_s5[4] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg2_4_squashn),
      .q(D_last_subvector_s6[4]));



// Module instance
    wire debugintrfupipereg1_5_squashn;
    assign debugintrfupipereg1_5_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg1_5 (
      .d( D_instr_s4[5] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_5_squashn),
      .q(D_instr_s5[5]));

// Module instance
    wire debugintrfupipereg2_5_squashn;
    assign debugintrfupipereg2_5_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_5 (
      .d( D_instr_s5[5] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_5_squashn),
      .q(D_instr_s6[5]));

// Module instance
    wire debuglastpipereg1_5_squashn;
    assign debuglastpipereg1_5_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg1_5 (
      .d( D_last_subvector_s4[5] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg1_5_squashn),
      .q(D_last_subvector_s5[5]));

// Module instance
    wire debuglastpipereg2_5_squashn;
    assign debuglastpipereg2_5_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg2_5 (
      .d( D_last_subvector_s5[5] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg2_5_squashn),
      .q(D_last_subvector_s6[5]));



// Module instance
    wire debugintrfupipereg1_6_squashn;
    assign debugintrfupipereg1_6_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg1_6 (
      .d( D_instr_s4[6] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_6_squashn),
      .q(D_instr_s5[6]));

// Module instance
    wire debugintrfupipereg2_6_squashn;
    assign debugintrfupipereg2_6_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_6 (
      .d( D_instr_s5[6] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_6_squashn),
      .q(D_instr_s6[6]));

// Module instance
    wire debuglastpipereg1_6_squashn;
    assign debuglastpipereg1_6_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg1_6 (
      .d( D_last_subvector_s4[6] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg1_6_squashn),
      .q(D_last_subvector_s5[6]));

// Module instance
    wire debuglastpipereg2_6_squashn;
    assign debuglastpipereg2_6_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg2_6 (
      .d( D_last_subvector_s5[6] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg2_6_squashn),
      .q(D_last_subvector_s6[6]));



// Module instance
    wire debugintrfupipereg1_7_squashn;
    assign debugintrfupipereg1_7_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg1_7 (
      .d( D_instr_s4[7] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_7_squashn),
      .q(D_instr_s5[7]));

// Module instance
    wire debugintrfupipereg2_7_squashn;
    assign debugintrfupipereg2_7_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_7 (
      .d( D_instr_s5[7] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_7_squashn),
      .q(D_instr_s6[7]));

// Module instance
    wire debuglastpipereg1_7_squashn;
    assign debuglastpipereg1_7_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg1_7 (
      .d( D_last_subvector_s4[7] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg1_7_squashn),
      .q(D_last_subvector_s5[7]));

// Module instance
    wire debuglastpipereg2_7_squashn;
    assign debuglastpipereg2_7_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg2_7 (
      .d( D_last_subvector_s5[7] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg2_7_squashn),
      .q(D_last_subvector_s6[7]));



// Module instance
    wire debugintrfupipereg1_8_squashn;
    assign debugintrfupipereg1_8_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg1_8 (
      .d( D_instr_s4[8] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_8_squashn),
      .q(D_instr_s5[8]));

// Module instance
    wire debugintrfupipereg2_8_squashn;
    assign debugintrfupipereg2_8_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_8 (
      .d( D_instr_s5[8] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_8_squashn),
      .q(D_instr_s6[8]));

// Module instance
    wire debuglastpipereg1_8_squashn;
    assign debuglastpipereg1_8_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg1_8 (
      .d( D_last_subvector_s4[8] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg1_8_squashn),
      .q(D_last_subvector_s5[8]));

// Module instance
    wire debuglastpipereg2_8_squashn;
    assign debuglastpipereg2_8_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg2_8 (
      .d( D_last_subvector_s5[8] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg2_8_squashn),
      .q(D_last_subvector_s6[8]));



// Module instance
    wire debugintrfupipereg1_9_squashn;
    assign debugintrfupipereg1_9_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg1_9 (
      .d( D_instr_s4[9] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_9_squashn),
      .q(D_instr_s5[9]));

// Module instance
    wire debugintrfupipereg2_9_squashn;
    assign debugintrfupipereg2_9_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_9 (
      .d( D_instr_s5[9] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_9_squashn),
      .q(D_instr_s6[9]));

// Module instance
    wire debuglastpipereg1_9_squashn;
    assign debuglastpipereg1_9_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg1_9 (
      .d( D_last_subvector_s4[9] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg1_9_squashn),
      .q(D_last_subvector_s5[9]));

// Module instance
    wire debuglastpipereg2_9_squashn;
    assign debuglastpipereg2_9_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg2_9 (
      .d( D_last_subvector_s5[9] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg2_9_squashn),
      .q(D_last_subvector_s6[9]));



// Module instance
    wire debugintrfupipereg1_10_squashn;
    assign debugintrfupipereg1_10_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg1_10 (
      .d( D_instr_s4[10] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_10_squashn),
      .q(D_instr_s5[10]));

// Module instance
    wire debugintrfupipereg2_10_squashn;
    assign debugintrfupipereg2_10_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_10 (
      .d( D_instr_s5[10] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_10_squashn),
      .q(D_instr_s6[10]));

// Module instance
    wire debuglastpipereg1_10_squashn;
    assign debuglastpipereg1_10_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg1_10 (
      .d( D_last_subvector_s4[10] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg1_10_squashn),
      .q(D_last_subvector_s5[10]));

// Module instance
    wire debuglastpipereg2_10_squashn;
    assign debuglastpipereg2_10_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg2_10 (
      .d( D_last_subvector_s5[10] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg2_10_squashn),
      .q(D_last_subvector_s6[10]));



//tell vpu to hold vc,vbase,etc values
assign is_stalled=~internal_pipe_advance | {stall_srcstart,2'b0};

assign has_memop=ctrl1_mem_en|ctrl2_mem_en|(|ctrl3_mem_en)|ctrl4_mem_en|ctrl5_mem_en;

/******************************************************************************/
/************************** Inter-Pipe Signals ********************************/
/******************************************************************************/

/* Memunit is in stage 5 but we stall in stage 4 so we don't squash the
* destination register contents*/

assign pipe_advance=internal_pipe_advance & ~{3'b0,stall_in};

assign internal_pipe_advance[0]=internal_pipe_advance[1];
assign internal_pipe_advance[1]=internal_pipe_advance[2];
assign internal_pipe_advance[2]=internal_pipe_advance[3] && 
                          ~stall_srcstart &&
                          ~stall_dispatcher &&
                          ~stall_hazsrc1 && ~stall_hazsrc2 && 
                          ~stall_hazfsrc1 && ~stall_hazfsrc2 && 
                          ~stall_mulcooldown;
assign internal_pipe_advance[3]=internal_pipe_advance[4];
assign internal_pipe_advance[4]=internal_pipe_advance[5];
assign internal_pipe_advance[5]=internal_pipe_advance[6] && ~stall_mulunit && ~stall_memunit;
//assign internal_pipe_advance[6]=1'b1;

assign pipe_squash[0]=pipe_advance[1]&~pipe_advance[0];
assign pipe_squash[1]=pipe_advance[2]&~pipe_advance[1];
assign pipe_squash[2]=pipe_advance[3]&~pipe_advance[2];
assign pipe_squash[3]=pipe_advance[4]&~pipe_advance[3];
assign pipe_squash[4]=pipe_advance[5]&~pipe_advance[4];
assign pipe_squash[5]=pipe_advance[6]&~pipe_advance[5];
//assign pipe_squash[6]=1'b0;

//The code below basically replicates the statements above for
//pipe stages from 6 to MAX_PIPE_STAGES. Using generate statement
//to reduce typing.



  assign pipe_squash[0] = pipe_advance[0+1] & ~pipe_advance[0];
  if (0==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[0] = internal_pipe_advance[0+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[0] = internal_pipe_advance[0+1];
  end



  assign pipe_squash[1] = pipe_advance[1+1] & ~pipe_advance[1];
  if (1==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[1] = internal_pipe_advance[1+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[1] = internal_pipe_advance[1+1];
  end



  assign pipe_squash[2] = pipe_advance[2+1] & ~pipe_advance[2];
  if (2==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[2] = internal_pipe_advance[2+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[2] = internal_pipe_advance[2+1];
  end



  assign pipe_squash[3] = pipe_advance[3+1] & ~pipe_advance[3];
  if (3==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[3] = internal_pipe_advance[3+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[3] = internal_pipe_advance[3+1];
  end



  assign pipe_squash[4] = pipe_advance[4+1] & ~pipe_advance[4];
  if (4==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[4] = internal_pipe_advance[4+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[4] = internal_pipe_advance[4+1];
  end



  assign pipe_squash[5] = pipe_advance[5+1] & ~pipe_advance[5];
  if (5==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[5] = internal_pipe_advance[5+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[5] = internal_pipe_advance[5+1];
  end



  assign pipe_squash[6] = pipe_advance[6+1] & ~pipe_advance[6];
  if (6==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[6] = internal_pipe_advance[6+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[6] = internal_pipe_advance[6+1];
  end



  assign pipe_squash[7] = pipe_advance[7+1] & ~pipe_advance[7];
  if (7==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[7] = internal_pipe_advance[7+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[7] = internal_pipe_advance[7+1];
  end



  assign pipe_squash[8] = pipe_advance[8+1] & ~pipe_advance[8];
  if (8==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[8] = internal_pipe_advance[8+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[8] = internal_pipe_advance[8+1];
  end



  assign pipe_squash[9] = pipe_advance[9+1] & ~pipe_advance[9];
  if (9==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[9] = internal_pipe_advance[9+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[9] = internal_pipe_advance[9+1];
  end



  assign pipe_squash[10] = pipe_advance[10+1] & ~pipe_advance[10];
  if (10==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[10] = internal_pipe_advance[10+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[10] = internal_pipe_advance[10+1];
  end



  assign pipe_squash[11] = pipe_advance[11+1] & ~pipe_advance[11];
  if (11==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[11] = internal_pipe_advance[11+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[11] = internal_pipe_advance[11+1];
  end



  assign pipe_squash[12] = pipe_advance[12+1] & ~pipe_advance[12];
  if (12==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[12] = internal_pipe_advance[12+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[12] = internal_pipe_advance[12+1];
  end



  assign pipe_squash[13] = pipe_advance[13+1] & ~pipe_advance[13];
  if (13==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[13] = internal_pipe_advance[13+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[13] = internal_pipe_advance[13+1];
  end



  assign pipe_squash[14] = pipe_advance[14+1] & ~pipe_advance[14];
  if (14==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[14] = internal_pipe_advance[14+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[14] = internal_pipe_advance[14+1];
  end



  assign pipe_squash[15] = pipe_advance[15+1] & ~pipe_advance[15];
  if (15==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[15] = internal_pipe_advance[15+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[15] = internal_pipe_advance[15+1];
  end



  assign pipe_squash[16] = pipe_advance[16+1] & ~pipe_advance[16];
  if (16==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[16] = internal_pipe_advance[16+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[16] = internal_pipe_advance[16+1];
  end



  assign pipe_squash[17] = pipe_advance[17+1] & ~pipe_advance[17];
  if (17==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[17] = internal_pipe_advance[17+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[17] = internal_pipe_advance[17+1];
  end



  assign pipe_squash[18] = pipe_advance[18+1] & ~pipe_advance[18];
  if (18==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[18] = internal_pipe_advance[18+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[18] = internal_pipe_advance[18+1];
  end



  assign pipe_squash[19] = pipe_advance[19+1] & ~pipe_advance[19];
  if (19==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[19] = internal_pipe_advance[19+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[19] = internal_pipe_advance[19+1];
  end



  assign pipe_squash[20] = pipe_advance[20+1] & ~pipe_advance[20];
  if (20==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[20] = internal_pipe_advance[20+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[20] = internal_pipe_advance[20+1];
  end



  assign pipe_squash[21] = pipe_advance[21+1] & ~pipe_advance[21];
  if (21==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[21] = internal_pipe_advance[21+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[21] = internal_pipe_advance[21+1];
  end



  assign pipe_squash[22] = pipe_advance[22+1] & ~pipe_advance[22];
  if (22==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[22] = internal_pipe_advance[22+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[22] = internal_pipe_advance[22+1];
  end



  assign pipe_squash[23] = pipe_advance[23+1] & ~pipe_advance[23];
  if (23==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[23] = internal_pipe_advance[23+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[23] = internal_pipe_advance[23+1];
  end



  assign pipe_squash[24] = pipe_advance[24+1] & ~pipe_advance[24];
  if (24==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[24] = internal_pipe_advance[24+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[24] = internal_pipe_advance[24+1];
  end



  assign pipe_squash[25] = pipe_advance[25+1] & ~pipe_advance[25];
  if (25==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[25] = internal_pipe_advance[25+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[25] = internal_pipe_advance[25+1];
  end



  assign pipe_squash[26] = pipe_advance[26+1] & ~pipe_advance[26];
  if (26==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[26] = internal_pipe_advance[26+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[26] = internal_pipe_advance[26+1];
  end



  assign pipe_squash[27] = pipe_advance[27+1] & ~pipe_advance[27];
  if (27==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[27] = internal_pipe_advance[27+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[27] = internal_pipe_advance[27+1];
  end



  assign pipe_squash[28] = pipe_advance[28+1] & ~pipe_advance[28];
  if (28==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[28] = internal_pipe_advance[28+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[28] = internal_pipe_advance[28+1];
  end



  assign pipe_squash[29] = pipe_advance[29+1] & ~pipe_advance[29];
  if (29==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[29] = internal_pipe_advance[29+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[29] = internal_pipe_advance[29+1];
  end



  assign pipe_squash[30] = pipe_advance[30+1] & ~pipe_advance[30];
  if (30==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[30] = internal_pipe_advance[30+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[30] = internal_pipe_advance[30+1];
  end



  assign pipe_squash[31] = pipe_advance[31+1] & ~pipe_advance[31];
  if (31==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[31] = internal_pipe_advance[31+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[31] = internal_pipe_advance[31+1];
  end



  assign pipe_squash[32] = pipe_advance[32+1] & ~pipe_advance[32];
  if (32==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[32] = internal_pipe_advance[32+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[32] = internal_pipe_advance[32+1];
  end



assign pipe_squash[`MAX_PIPE_STAGES-1]=1'b0;
assign internal_pipe_advance[`MAX_PIPE_STAGES-1]=1'b1;

/******************************************************************************/
/************************** 1st Pipeline Stage ********************************/
/******************************************************************************/

  assign ir_op={instr[25:22],instr[5:0]}; //10 bits
  assign ir_dst=instr[10:6];
  assign ir_src1=instr[15:11];
  assign ir_src2=instr[20:16];
  assign ir_mask=instr[21];

  // Determine which instruction read from which ports
  always@*
  begin
//    ctrl1_vr_a_en=0;
//    ctrl1_vr_b_en=0;
//    ctrl1_vr_c_en=0;
//    ctrl1_vf_a_en=0;
//    ctrl1_vf_b_en=0;
//    ctrl1_vf_a_sel=0;
//    ctrl1_usesvssel=0;
    case(ir_op)
    COP2_VADD:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VADD_U:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VSUB:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VSUB_U:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VMULHI:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VMULHI_U:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VMOD:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VMOD_U:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VCMP_EQ:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VCMP_NE:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VCMP_LT:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VCMP_U_LT:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VCMP_LE:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VCMP_U_LE:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VDIV:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VDIV_U:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VMIN:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VMIN_U:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VMAX:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VMAX_U:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VMULLO:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VABS:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=1;        end
    COP2_VAND:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VOR:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXOR:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VNOR:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_usesvssel=1;        end
    COP2_VSLL:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VSRL:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VSRA:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_usesvssel=1;        end
    COP2_VSAT_B:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_H:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_W:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_SU_B:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_SU_H:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_SU_W:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_SU_L:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_U_B:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_U_H:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSAT_U_W:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSADD:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VSADD_U:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VSSUB:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VSSUB_U:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VSRR:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSRR_U:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSLS:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSLS_U:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VXUMUL:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXUMUL_U:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXLMUL:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXLMUL_U:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXUMADD:

        begin          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXUMADD_U:

        begin          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXUMSUB:

        begin          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXUMSUB_U:

        begin          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXLMADD:

        begin          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXLMADD_U:

        begin          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXLMSUB:

        begin          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VXLMSUB_U:

        begin          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=1;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;          ctrl1_usesvssel=1;        end
    COP2_VINS_VV:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vr_b_en=1;        end
      //COP2_VINS_SV: doesn't read any vectors or flags
    COP2_VEXT_VV:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;        end
    COP2_VEXT_SV:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;        end
    COP2_VEXT_U_SV:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;        end
    COP2_VCOMPRESS:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VEXPAND:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VMERGE:

        begin          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];          ctrl1_vf_a_en=1;          ctrl1_usesvssel=1;        end
      //COP2_VFINS:
    COP2_VEXTHALF:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;        end
    COP2_VHALF:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;        end
    COP2_VHALFUP:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;        end
    COP2_VHALFDN:

        begin          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_a_en=1;        end
      //COP2_VSATVL:
    COP2_VFAND:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vf_b_en=1;          ctrl1_vf_a_sel=1;          ctrl1_usesvssel=1;        end
    COP2_VFOR:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vf_b_en=1;          ctrl1_vf_a_sel=1;          ctrl1_usesvssel=1;        end
    COP2_VFXOR:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vf_b_en=1;          ctrl1_vf_a_sel=1;          ctrl1_usesvssel=1;        end
    COP2_VFNOR:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];          ctrl1_vf_b_en=1;          ctrl1_vf_a_sel=1;          ctrl1_usesvssel=1;        end
      //COP2_VFCLR:
      //COP2_VFSET:
    COP2_VIOTA:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
    COP2_VCIOTA:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
    COP2_VFPOP:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
    COP2_VFFF1:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
    COP2_VFFL1:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
    COP2_VFSETBF:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
    COP2_VFSETIF:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
    COP2_VFSETOF:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vf_a_sel=1;        end
      //COP2_VFMT8:
      //COP2_VFMF8:
      //COP2_VFCLR8:
      //COP2_VFOR8:
      //COP2_VFLD:
    COP2_VLD_B:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;        end
    COP2_VLD_H:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;        end
    COP2_VLD_W:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;        end
    COP2_VLD_L:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;        end
    COP2_VLD_U_B:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;        end
    COP2_VLD_U_H:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;        end
    COP2_VLD_U_W:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;        end
    COP2_VLDS_B:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;        end
    COP2_VLDS_H:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;        end
    COP2_VLDS_W:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;        end
    COP2_VLDS_L:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;        end
    COP2_VLDS_U_B:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;        end
    COP2_VLDS_U_H:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;        end
    COP2_VLDS_U_W:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;        end
    COP2_VLDX_B:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;        end
    COP2_VLDX_H:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;        end
    COP2_VLDX_W:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;        end
    COP2_VLDX_L:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;        end
    COP2_VLDX_U_B:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;        end
    COP2_VLDX_U_H:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;        end
    COP2_VLDX_U_W:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_b_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_c_en=1;        end
      //COP2_VFST:
    COP2_VST_B:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VST_H:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VST_W:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VST_L:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VSTS_B:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VSTS_H:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VSTS_W:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VSTS_L:

        begin          ctrl1_vr_a_en=0;          ctrl1_vr_c_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vf_a_en=1;          ctrl1_vr_b_en=1;        end
    COP2_VSTX_B:

        begin          ctrl1_vr_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSTX_H:

        begin          ctrl1_vr_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSTX_W:

        begin          ctrl1_vr_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSTX_L:

        begin          ctrl1_vr_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSTXO_B:

        begin          ctrl1_vr_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSTXO_H:

        begin          ctrl1_vr_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSTXO_W:

        begin          ctrl1_vr_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
    COP2_VSTXO_L:

        begin          ctrl1_vr_a_en=0;          ctrl1_vf_b_en=0;          ctrl1_vf_a_sel=0;          ctrl1_usesvssel=0;          ctrl1_vr_b_en=1;          ctrl1_vr_c_en=1;          ctrl1_vf_a_en=1;        end
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  // Decode enables 
  always@*
  begin
//    ctrl1_vr_d_we=0;
//    ctrl1_vf_c_we=0;
//    ctrl1_vs_we=0;
//    ctrl1_vrdest_sel=0;
//    ctrl1_elmshamt_sel=0;
//    ctrl1_srcshamt_sel=0;
//    ctrl1_srclimit_sel=0;
//    ctrl1_dstshamt_sel=0;
//    ctrl1_mem_dir_left=0;
//    ctrl1_rshiftnonzero=0;
//    ctrl1_memunit_en=0;
//    ctrl1_mem_en=0;
//    ctrl1_ismasked=0;
//    ctrl1_setvlto1=0;
//    ctrl1_vf_wbsel=0;
//    ctrl1_volatiledest=0;
    case(ir_op)
    COP2_VADD:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VADD_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSUB:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSUB_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMULHI:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMULHI_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VDIV:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VDIV_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMOD:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMOD_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VCMP_EQ:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;          ctrl1_vf_wbsel=1;          ctrl1_ismasked=1;        end
    COP2_VCMP_NE:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;          ctrl1_vf_wbsel=1;          ctrl1_ismasked=1;        end
    COP2_VCMP_LT:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;          ctrl1_vf_wbsel=1;          ctrl1_ismasked=1;        end
    COP2_VCMP_U_LT:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;          ctrl1_vf_wbsel=1;          ctrl1_ismasked=1;        end
    COP2_VCMP_LE:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;          ctrl1_vf_wbsel=1;          ctrl1_ismasked=1;        end
    COP2_VCMP_U_LE:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;          ctrl1_vf_wbsel=1;          ctrl1_ismasked=1;        end
    COP2_VMIN:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMIN_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMAX:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMAX_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VMULLO:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VABS:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VAND:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VOR:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VXOR:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VNOR:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSLL:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSRL:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSRA:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_B:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_H:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_W:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_SU_B:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_SU_H:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_SU_W:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_SU_L:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_U_B:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_U_H:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSAT_U_W:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSADD:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSADD_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSSUB:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSSUB_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSRR:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSRR_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSLS:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VSLS_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_ismasked=1;        end
    COP2_VXUMUL:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXUMUL_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXLMUL:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXLMUL_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXUMADD:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXUMADD_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXUMSUB:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXUMSUB_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXLMADD:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXLMADD_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXLMSUB:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VXLMSUB_U:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_rshiftnonzero=1;          ctrl1_ismasked=1;        end
    COP2_VINS_VV:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_rshiftnonzero=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_memunit_en=1;          ctrl1_vr_d_we=1;          ctrl1_elmshamt_sel=3;          ctrl1_srcshamt_sel=0;          ctrl1_dstshamt_sel=3;          ctrl1_mem_dir_left=1;          ctrl1_volatiledest=1;        end
    COP2_VINS_SV:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_rshiftnonzero=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_vr_d_we=1;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_dstshamt_sel=3;          ctrl1_mem_dir_left=1;          ctrl1_setvlto1=1;        end
    COP2_VEXT_VV:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_rshiftnonzero=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_memunit_en=1;          ctrl1_vr_d_we=1;          ctrl1_elmshamt_sel=3;          ctrl1_srcshamt_sel=3;          ctrl1_srclimit_sel=1;          ctrl1_mem_dir_left=0;          ctrl1_volatiledest=1;        end
    COP2_VEXT_SV:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vrdest_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_rshiftnonzero=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vs_we=1;          ctrl1_memunit_en=1;          ctrl1_elmshamt_sel=3;          ctrl1_srcshamt_sel=3;          ctrl1_srclimit_sel=1;          ctrl1_mem_dir_left=0;          ctrl1_setvlto1=1;        end
    COP2_VEXT_U_SV:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vrdest_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_rshiftnonzero=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vs_we=1;          ctrl1_memunit_en=1;          ctrl1_elmshamt_sel=3;          ctrl1_srcshamt_sel=3;          ctrl1_srclimit_sel=1;          ctrl1_mem_dir_left=0;          ctrl1_setvlto1=1;        end
    COP2_VCOMPRESS:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_rshiftnonzero=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_memunit_en=1;          ctrl1_vr_d_we=1;          ctrl1_mem_dir_left=0;          ctrl1_ismasked=1;          ctrl1_volatiledest=1;        end
    COP2_VEXPAND:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_rshiftnonzero=0;          ctrl1_mem_en=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_memunit_en=1;          ctrl1_vr_d_we=1;          ctrl1_mem_dir_left=1;          ctrl1_ismasked=1;          ctrl1_volatiledest=1;        end
    COP2_VMERGE:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;        end
    COP2_VFINS:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_elmshamt_sel=3;          ctrl1_srcshamt_sel=0;          ctrl1_mem_dir_left=1;        end
    COP2_VEXTHALF:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_rshiftnonzero=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_memunit_en=1;          ctrl1_elmshamt_sel=1;          ctrl1_srcshamt_sel=1;          ctrl1_mem_dir_left=0;          ctrl1_vr_d_we=1;          ctrl1_volatiledest=1;        end
    COP2_VHALF:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_rshiftnonzero=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_memunit_en=1;          ctrl1_elmshamt_sel=1;          ctrl1_srcshamt_sel=1;          ctrl1_mem_dir_left=0;          ctrl1_vr_d_we=1;          ctrl1_volatiledest=1;        end
    COP2_VHALFUP:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_rshiftnonzero=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_memunit_en=1;          ctrl1_elmshamt_sel=2;          ctrl1_srcshamt_sel=2;          ctrl1_mem_dir_left=0;          ctrl1_vr_d_we=1;          ctrl1_volatiledest=1;        end
    COP2_VHALFDN:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_rshiftnonzero=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_memunit_en=1;          ctrl1_elmshamt_sel=2;          ctrl1_srcshamt_sel=0;          ctrl1_dstshamt_sel=2;          ctrl1_mem_dir_left=1;          ctrl1_vr_d_we=1;          ctrl1_volatiledest=1;        end
      //COP2_VSATVL:
    COP2_VFAND:

        begin          ctrl1_vr_d_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;        end
    COP2_VFOR:

        begin          ctrl1_vr_d_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;        end
    COP2_VFXOR:

        begin          ctrl1_vr_d_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;        end
    COP2_VFNOR:

        begin          ctrl1_vr_d_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;        end
    COP2_VFCLR:

        begin          ctrl1_vr_d_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;        end
    COP2_VFSET:

        begin          ctrl1_vr_d_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;        end
    COP2_VIOTA:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;        end
    COP2_VCIOTA:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;        end
    COP2_VFPOP:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vs_we=1;        end
    COP2_VFFF1:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vs_we=1;        end
    COP2_VFFL1:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vs_we=1;        end
    COP2_VFSETBF:

        begin          ctrl1_vr_d_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;        end
    COP2_VFSETIF:

        begin          ctrl1_vr_d_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;        end
    COP2_VFSETOF:

        begin          ctrl1_vr_d_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;        end
    COP2_VFMT8:

        begin          ctrl1_vr_d_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;        end
    COP2_VFMF8:

        begin          ctrl1_vr_d_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;        end
    COP2_VFCLR8:

        begin          ctrl1_vr_d_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;        end
    COP2_VFOR8:

        begin          ctrl1_vr_d_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_memunit_en=0;          ctrl1_mem_en=0;          ctrl1_ismasked=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vf_c_we=1;        end
      //COP2_VFLD,
    COP2_VLD_B:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLD_H:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLD_W:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLD_L:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLD_U_B:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLD_U_H:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLD_U_W:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDS_B:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDS_H:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDS_W:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDS_L:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDS_U_B:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDS_U_H:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDS_U_W:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDX_B:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDX_H:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDX_W:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDX_L:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDX_U_B:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDX_U_H:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VLDX_U_W:

        begin          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_vr_d_we=1;          ctrl1_vrdest_sel=1;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
      //COP2_VFST:
    COP2_VST_B:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VST_H:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VST_W:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VST_L:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTS_B:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTS_H:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTS_W:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTS_L:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTX_B:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTX_H:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTX_W:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTX_L:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTXO_B:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTXO_H:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTXO_W:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
    COP2_VSTXO_L:

        begin          ctrl1_vr_d_we=0;          ctrl1_vf_c_we=0;          ctrl1_vs_we=0;          ctrl1_vrdest_sel=0;          ctrl1_elmshamt_sel=0;          ctrl1_srcshamt_sel=0;          ctrl1_srclimit_sel=0;          ctrl1_dstshamt_sel=0;          ctrl1_mem_dir_left=0;          ctrl1_rshiftnonzero=0;          ctrl1_setvlto1=0;          ctrl1_vf_wbsel=0;          ctrl1_volatiledest=0;          ctrl1_memunit_en=1;          ctrl1_mem_en=1;          ctrl1_ismasked=1;        end
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  // Decode instructions FU op codes
  initial
  begin
    ctrl1_alu_op=ALUOP_ZERO^ALUOP_ZERO;  //Aid subsetting by setting to zero
    ctrl1_satsum_op=SATSUMOP_NOP;
    ctrl1_satsize_op=SATSIZEOP_VSATUW;
    ctrl1_mulshift_op=MULOP_ZERO;
    ctrl1_matmul_en=0;
    ctrl1_trp_en = 1'b0;
    ctrl1_bfadder_en = 1'b0;
    ctrl1_bfmult_en = 1'b0;
    ctrl1_act_en = 1'b0;
    ctrl1_memunit_op=0;
    ctrl1_flagalu_op=FLAGOP_CLR;
  end
  always@*
  begin
    ctrl1_alu_op=ALUOP_ZERO^ALUOP_ZERO;
    ctrl1_satsum_op=SATSUMOP_NOP;
    ctrl1_satsize_op=SATSIZEOP_VSATUW;
    ctrl1_mulshift_op=MULOP_ZERO;
    ctrl1_memunit_op=0;
    ctrl1_matmul_en=0;
    ctrl1_bfadder_en = 1'b0;
    ctrl1_bfmult_en = 1'b0;
    ctrl1_act_en = 1'b0;
    ctrl1_trp_en = 1'b0;
    ctrl1_flagalu_op=FLAGOP_CLR;
    case(ir_op)
      COP2_VADD:      ctrl1_alu_op=ALUOP_ADD^ALUOP_ZERO;
      COP2_VADD_U:    ctrl1_alu_op=ALUOP_ADDU^ALUOP_ZERO;
      COP2_VSUB:      ctrl1_alu_op=ALUOP_SUB^ALUOP_ZERO;
      COP2_VSUB_U:    ctrl1_alu_op=ALUOP_SUBU^ALUOP_ZERO;
      COP2_VMULHI:  ctrl1_mulshift_op=MULOP_MULHI;
      COP2_VMULHI_U:ctrl1_mulshift_op=MULOP_MULUHI;
      COP2_VDIV:    ctrl1_matmul_en=1'b1;  //Note: This is a hack. We're using VDIV opcode for matmul operation
      COP2_VBFADD:    ctrl1_bfadder_en = 1'b1;
      COP2_VBFMULT:   ctrl1_bfmult_en = 1'b1;
      COP2_VACT:      ctrl1_act_en = 1'b1;
      COP2_VTRP:      ctrl1_trp_en = 1'b1;
      //COP2_VDIV_U,
      //COP2_VMOD,
      //COP2_VMOD_U,
      COP2_VCMP_EQ:   ctrl1_alu_op=ALUOP_CMP_EQ^ALUOP_ZERO;
      COP2_VCMP_NE:   ctrl1_alu_op=ALUOP_CMP_NEQ^ALUOP_ZERO;
      COP2_VCMP_LT:   ctrl1_alu_op=ALUOP_CMP_LT^ALUOP_ZERO;
      COP2_VCMP_U_LT: ctrl1_alu_op=ALUOP_CMP_LT_U^ALUOP_ZERO;
      COP2_VCMP_LE:   ctrl1_alu_op=ALUOP_CMP_LE^ALUOP_ZERO;
      COP2_VCMP_U_LE: ctrl1_alu_op=ALUOP_CMP_LE_U^ALUOP_ZERO;
      COP2_VMIN:      ctrl1_alu_op=ALUOP_MIN^ALUOP_ZERO;
      COP2_VMIN_U:    ctrl1_alu_op=ALUOP_MIN_U^ALUOP_ZERO;
      COP2_VMAX:      ctrl1_alu_op=ALUOP_MAX^ALUOP_ZERO;
      COP2_VMAX_U:    ctrl1_alu_op=ALUOP_MAX_U^ALUOP_ZERO;
      COP2_VMULLO:    ctrl1_mulshift_op=MULOP_MULLO;
      COP2_VABS:      ctrl1_alu_op=ALUOP_ABS^ALUOP_ZERO;
      COP2_VAND:      ctrl1_alu_op=ALUOP_AND^ALUOP_ZERO;
      COP2_VOR:       ctrl1_alu_op=ALUOP_OR^ALUOP_ZERO;
      COP2_VXOR:      ctrl1_alu_op=ALUOP_XOR^ALUOP_ZERO;
      COP2_VNOR:      ctrl1_alu_op=ALUOP_NOR^ALUOP_ZERO;
      COP2_VSLL: ctrl1_mulshift_op=MULOP_SLL;
      COP2_VSRL: ctrl1_mulshift_op=MULOP_SRL;
      COP2_VSRA: ctrl1_mulshift_op=MULOP_SRA;
      COP2_VSAT_B: ctrl1_satsize_op=SATSIZEOP_VSATB;
      COP2_VSAT_H: ctrl1_satsize_op=SATSIZEOP_VSATH;
      COP2_VSAT_W: ctrl1_satsize_op=SATSIZEOP_VSATW;
      COP2_VSAT_SU_B: ctrl1_satsize_op=SATSIZEOP_VSATSUB;
      COP2_VSAT_SU_H: ctrl1_satsize_op=SATSIZEOP_VSATSUH;
      COP2_VSAT_SU_W: ctrl1_satsize_op=SATSIZEOP_VSATSUW;
      //COP2_VSAT_SU_L:
      COP2_VSAT_U_B: ctrl1_satsize_op=SATSIZEOP_VSATUB;
      COP2_VSAT_U_H: ctrl1_satsize_op=SATSIZEOP_VSATUH;
      COP2_VSAT_U_W: ctrl1_satsize_op=SATSIZEOP_VSATUW;
      COP2_VSADD: begin ctrl1_alu_op=ALUOP_ADD^ALUOP_ZERO; ctrl1_satsum_op=SATSUMOP_VS; end
      COP2_VSADD_U: begin 
          ctrl1_alu_op=ALUOP_ADDU^ALUOP_ZERO; 
          ctrl1_satsum_op=SATSUMOP_VSU; 
        end
      COP2_VSSUB: begin ctrl1_alu_op=ALUOP_SUB^ALUOP_ZERO; ctrl1_satsum_op=SATSUMOP_VS; end
      COP2_VSSUB_U: begin 
          ctrl1_alu_op=ALUOP_SUBU^ALUOP_ZERO;
          ctrl1_satsum_op=SATSUMOP_VSU; 
        end
      COP2_VSRR: ctrl1_mulshift_op=MULOP_SRA;
      COP2_VSRR_U: ctrl1_mulshift_op=MULOP_SRL;
      COP2_VSLS: ctrl1_mulshift_op=MULOP_SLS;
      COP2_VSLS_U: ctrl1_mulshift_op=MULOP_SLSU;
      COP2_VXUMUL: ctrl1_mulshift_op=MULOP_UMUL;
      COP2_VXUMUL_U: ctrl1_mulshift_op=MULOP_UMULU;
      COP2_VXLMUL: ctrl1_mulshift_op=MULOP_LMUL;
      COP2_VXLMUL_U: ctrl1_mulshift_op=MULOP_LMULU;
      COP2_VXUMADD: ctrl1_mulshift_op=MULOP_UMUL;
      COP2_VXUMADD_U: ctrl1_mulshift_op=MULOP_UMULU;
      COP2_VXUMSUB: ctrl1_mulshift_op=MULOP_UMUL;
      COP2_VXUMSUB_U: ctrl1_mulshift_op=MULOP_UMULU;
      COP2_VXLMADD: ctrl1_mulshift_op=MULOP_LMUL;
      COP2_VXLMADD_U: ctrl1_mulshift_op=MULOP_LMULU;
      COP2_VXLMSUB: ctrl1_mulshift_op=MULOP_LMUL;
      COP2_VXLMSUB_U: ctrl1_mulshift_op=MULOP_LMULU;
      COP2_VINS_VV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VINS_SV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VEXT_VV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VEXT_SV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VEXT_U_SV: ctrl1_memunit_op=MEMOP_SHIFT;
      //COP2_VCOMPRESS:
      //COP2_VEXPAND:
      COP2_VMERGE:      ctrl1_alu_op=ALUOP_MERGE^ALUOP_ZERO;
      //COP2_VFINS:
      COP2_VEXTHALF: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VHALF: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VHALFUP: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VHALFDN: ctrl1_memunit_op=MEMOP_SHIFT;
      //COP2_VSATVL:
      COP2_VFAND: ctrl1_flagalu_op=FLAGOP_AND;
      COP2_VFOR: ctrl1_flagalu_op=FLAGOP_OR;
      COP2_VFXOR: ctrl1_flagalu_op=FLAGOP_XOR;
      COP2_VFNOR: ctrl1_flagalu_op=FLAGOP_NOR;
      COP2_VFCLR: ctrl1_flagalu_op=FLAGOP_CLR;
      COP2_VFSET: ctrl1_flagalu_op=FLAGOP_SET;
      //COP2_VIOTA,
      //COP2_VCIOTA:
      //COP2_VFPOP,
      //COP2_VFFF1,
      //COP2_VFFL1:
      //COP2_VFSETBF,
      //COP2_VFSETIF,
      //COP2_VFSETOF,
      //COP2_VFMT8,
      //COP2_VFMF8,
      //COP2_VFCLR8,
      //COP2_VFOR8:
      //COP2_VFLD:
      COP2_VLD_B: ctrl1_memunit_op=MEMOP_LDB;
      COP2_VLD_H: ctrl1_memunit_op=MEMOP_LDH;
      COP2_VLD_W: ctrl1_memunit_op=MEMOP_LDW;
      //COP2_VLD_L,
      COP2_VLD_U_B: ctrl1_memunit_op=MEMOP_LDUB;
      COP2_VLD_U_H: ctrl1_memunit_op=MEMOP_LDUH;
      COP2_VLD_U_W: ctrl1_memunit_op=MEMOP_LDUW;
      COP2_VLDS_B: ctrl1_memunit_op=MEMOP_LDSB;
      COP2_VLDS_H: ctrl1_memunit_op=MEMOP_LDSH;
      COP2_VLDS_W: ctrl1_memunit_op=MEMOP_LDSW;
      //COP2_VLDS_L:
      COP2_VLDS_U_B: ctrl1_memunit_op=MEMOP_LDSUB;
      COP2_VLDS_U_H: ctrl1_memunit_op=MEMOP_LDSUH;
      COP2_VLDS_U_W: ctrl1_memunit_op=MEMOP_LDSUW;
      COP2_VLDX_B: ctrl1_memunit_op=MEMOP_LDXB;
      COP2_VLDX_H: ctrl1_memunit_op=MEMOP_LDXH;
      COP2_VLDX_W: ctrl1_memunit_op=MEMOP_LDXW;
      //COP2_VLDX_L:
      COP2_VLDX_U_B: ctrl1_memunit_op=MEMOP_LDXUB;
      COP2_VLDX_U_H: ctrl1_memunit_op=MEMOP_LDXUH;
      COP2_VLDX_U_W: ctrl1_memunit_op=MEMOP_LDXUW;
      //COP2_VFST:
      COP2_VST_B: ctrl1_memunit_op=MEMOP_STB;
      COP2_VST_H: ctrl1_memunit_op=MEMOP_STH;
      COP2_VST_W: ctrl1_memunit_op=MEMOP_STW;
      //COP2_VST_L:
      COP2_VSTS_B: ctrl1_memunit_op=MEMOP_STSB;
      COP2_VSTS_H: ctrl1_memunit_op=MEMOP_STSH;
      COP2_VSTS_W: ctrl1_memunit_op=MEMOP_STSW;
      //COP2_VSTS_L:
      COP2_VSTX_B: ctrl1_memunit_op=MEMOP_STXB;
      COP2_VSTX_H: ctrl1_memunit_op=MEMOP_STXH;
      COP2_VSTX_W: ctrl1_memunit_op=MEMOP_STXW;
      //COP2_VSTX_L:
      COP2_VSTXO_B: ctrl1_memunit_op=MEMOP_STXB;
      COP2_VSTXO_H: ctrl1_memunit_op=MEMOP_STXH;
      COP2_VSTXO_W: ctrl1_memunit_op=MEMOP_STXW;
      //COP2_VSTXO_L:
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  assign regid_pad=0;

// Module instance
  wire [7:0] pipe1reg_secondstageonly_q;
  assign {
        ctrl2_elmshamt_sel,
        ctrl2_srcshamt_sel,
        ctrl2_srclimit_sel, //used to get squashed, pretend doesn't need to
        ctrl2_dstshamt_sel,
        ctrl2_setvlto1
      } = pipe1reg_secondstageonly_q;
  pipereg_8  pipe1reg_secondstageonly (
      .d( {
        ctrl1_elmshamt_sel,
        ctrl1_srcshamt_sel,
        ctrl1_srclimit_sel,
        ctrl1_dstshamt_sel,
        ctrl1_setvlto1
      }),
      .clk(clk),
      .resetn(resetn),
      .en(pipe_advance[1]),
      .squashn(1'b1),
      .q(pipe1reg_secondstageonly_q));

  // *********** Pipeline signals that need to be squashed *********

//module instance
  wire [16:0] pipe1regwsquash_q;
  wire pipe1regwsquash_squashn;
  wire pipe1regwsquash_d1;
  wire pipe1regwsquash_d2;
  wire pipe1regwsquash_d3;
  wire pipe1regwsquash_d4;

  assign {
        ctrl2_vr_c_we,
        ctrl2_vf_c_we,
        ctrl2_vs_we,
        ctrl2_vr_a_en,
        ctrl2_vr_b_en,
        ctrl2_vf_a_en,
        ctrl2_vf_b_en,
        ctrl2_useslanes,
        ctrl2_memunit_en,
        ctrl2_mem_en,
        ctrl2_mulshift_en,
        ctrl2_matmul_en,
        ctrl2_bfadder_en,
        ctrl2_bfmult_en,
        ctrl2_act_en,
        ctrl2_trp_en,
        ctrl2_alufalu_en
        } = pipe1regwsquash_q;

  assign pipe1regwsquash_squashn = ~pipe_squash[1];

  assign pipe1regwsquash_d1 = ctrl1_vr_a_en|ctrl1_vr_c_en;
  assign pipe1regwsquash_d2 = ctrl1_vr_a_en|ctrl1_vr_b_en|ctrl1_vr_c_en| ctrl1_vr_d_we|ctrl1_vf_a_en|ctrl1_vf_b_en|ctrl1_vf_c_we;
  assign pipe1regwsquash_d3 = |ctrl1_mulshift_op;
  assign pipe1regwsquash_d4 = (ctrl1_alu_op!=(ALUOP_ZERO^ALUOP_ZERO)) || ctrl1_vf_c_we;

  pipereg_17  pipe1regwsquash (
      .d( {
        ctrl1_vr_d_we,
        ctrl1_vf_c_we,
        ctrl1_vs_we,
        pipe1regwsquash_d1,
        ctrl1_vr_b_en,
        ctrl1_vf_a_en,
        ctrl1_vf_b_en,
        pipe1regwsquash_d2,
        ctrl1_memunit_en,
        ctrl1_mem_en,
        pipe1regwsquash_d3,
        ctrl1_matmul_en,
        ctrl1_bfadder_en,
        ctrl1_bfmult_en,
        ctrl1_act_en,
        ctrl1_trp_en,
        pipe1regwsquash_d4
      }),
      .clk(clk),
      .resetn(resetn),
      .en(pipe_advance[1]),
      .squashn(pipe1regwsquash_squashn),
      .q(pipe1regwsquash_q)
      );

  // *********** Pipeline signals that don't need to be squashed *********
  wire [72-1:0] pipe1reg_q;
//  wire [`VRELM_RANGE+5:0] pipe1reg_d1;
//  wire [`VRELM_RANGE+5:0] pipe1reg_d2;
//  wire [`VRELM_RANGE+5:0] pipe1reg_d3;
//  wire pipe1reg_d4;
//  wire pipe1reg_d5;

  assign {
        dst_s2,
        src1_s2,
        src2_s2,
        src1scalar_s2,
        src2scalar_s2,
        imask_s2,
        ctrl2_vf_a_sel,
        ctrl2_rshiftnonzero,
        ctrl2_mem_dir_left,
        ctrl2_alu_op,
        ctrl2_satsum_op,
        ctrl2_satsize_op,
        ctrl2_mulshift_op,
        ctrl2_memunit_op,
        ctrl2_ismasked,
        ctrl2_flagalu_op,
        ctrl2_vf_wbsel,
        ctrl2_volatiledest
      } = pipe1reg_q;

//  assign pipe1reg_d1 = {(ctrl1_vrdest_sel) ? ir_src2 : ir_dst, regid_pad};
//  assign pipe1reg_d2 = {(ctrl1_vr_c_en ) ? ir_dst : ir_src1, regid_pad};
//  assign pipe1reg_d3 = {ir_src2,regid_pad};
//  assign pipe1reg_d4 = ir_op[7] & ctrl1_usesvssel;
//  assign pipe1reg_d5 = ir_op[6] & ctrl1_usesvssel;

  pipereg_72  pipe1reg (
      .d( {
        {(ctrl1_vrdest_sel) ? ir_src2 : ir_dst, regid_pad},
        {(ctrl1_vr_c_en ) ? ir_dst : ir_src1, regid_pad},
        {ir_src2,regid_pad},
        ir_op[7] && ctrl1_usesvssel,
        ir_op[6] && ctrl1_usesvssel,
        ir_mask,
        ctrl1_vf_a_sel,
        ctrl1_rshiftnonzero,
        ctrl1_mem_dir_left,
        ctrl1_alu_op,
        ctrl1_satsum_op,
        ctrl1_satsize_op,
        ctrl1_mulshift_op,
        ctrl1_memunit_op,
        ctrl1_ismasked,
        ctrl1_flagalu_op,
        ctrl1_vf_wbsel,
        ctrl1_volatiledest
      }),
      .clk(clk),
      .resetn(resetn),
      .en(pipe_advance[1]),
      .squashn( 1'b1 ),
      .q(pipe1reg_q));
  
  wire [6:1] squash_ctrlmemoppipe_NC;
  wire [5-1:0] ctrlmemoppipe_en;
  wire [7*(5+1)-1:0] ctrlmemoppipe_q;

  assign ctrlmemoppipe_en = pipe_advance[6:1] & {4'b1,ctrl2_memunit_en,1'b1};
  assign {ctrl_memunit_op[6],ctrl_memunit_op[5],ctrl_memunit_op[4],ctrl_memunit_op[3],ctrl_memunit_op[2],ctrl_memunit_op[1]} = ctrlmemoppipe_q;

//module instance
  pipe_7_5  ctrlmemoppipe (
      .d(ctrl1_memunit_op),
      .clk(clk),
      .resetn(resetn),
      .en(ctrlmemoppipe_en),
      .squash(squash_ctrlmemoppipe_NC),
      //.squash( pipe_squash[6:1] ),
      .q(ctrlmemoppipe_q));

/******************************************************************************/
/******************************* 2nd Pipeline Stage ***************************/
/******************************************************************************/

  // if src_start!=0 stall pipeline to calculate it and then do haz check
  wire shamtstall1;
  assign shamtstall1 = ctrl2_srcshamt_sel!=0;
  onecyclestall shamtstall(shamtstall1,clk,resetn,stall_srcstart);

  always@(posedge clk)
    if (!resetn || pipe_advance[1] )
      src_start_delayed<=0;
    else 
      src_start_delayed<=src_start;

  assign src_start= ( ctrl2_srcshamt_sel==3 ) ? vc[2] :           // vcindex
               ( ctrl2_srcshamt_sel==1 ) ? vl[2][32-1:1] :   // vl>>2
               ( ctrl2_srcshamt_sel==2 ) ? 1 << vc[2][7-1:0]://2^vcindex
               0;

  assign src_elm=src_start_delayed & (-1<<2);

  assign src_limit= ((ctrl2_setvlto1) ? 0 : vl[2][7-1:0] - 1'b1) +
                    ((ctrl2_srclimit_sel) ? vc[2][7-1:0] : 0);


  /******************* Adjust dest to account for shift  ****************/

  // Compute real destination register - accounting for shifting instructions

  assign dst_start= ( ctrl2_dstshamt_sel==3 ) ? vc[2] :           // vcindex
               ( ctrl2_dstshamt_sel==2 ) ? 1 << vc[2][7-1:0]://2^vcindex
               0;

  //assign dst_elm = {dst_start[7-1:0]>>2,{2{1'b0}}};

  assign total_shamt= ( ctrl2_elmshamt_sel==3 ) ? vc[2] :          // vcindex
               ( ctrl2_elmshamt_sel==1 ) ? vl[2][32-1:1] :    // vl>>2
               ( ctrl2_elmshamt_sel==2 ) ? 1 << vc[2][7-1:0]://2^vcindex
               0;

  /************ Save vc_in values to not stall control pipeline ************/
  always@(posedge clk)
    if (!resetn)
      pipe_advance_s2_r<=0;
    else
      pipe_advance_s2_r<=pipe_advance[1];

  always@(posedge clk)
    if (!resetn)
    begin
      vc_in_saved<=0;
      vl_in_saved<=0;
      vbase_in_saved<=0;
      vinc_in_saved<=0;
      vstride_in_saved<=0;
      vs_in_saved<=0;
    end
    else if (pipe_advance_s2_r)
    begin
      vc_in_saved<=vc_in;
      vl_in_saved<=vl_in;
      vbase_in_saved<=vbase_in;
      vinc_in_saved<=vinc_in;
      vstride_in_saved<=vstride_in;
      vs_in_saved<=vs_in;
    end

  wire [5:2] squash_vcpipe_NC;
  wire [32-1:0] vcpipe_d;
  wire [32*(4+1)-1:0] vcpipe_q;
  wire [4-1:0] vcpipe_en;

  assign vcpipe_d = (!pipe_advance_s2_r) ? vc_in_saved : vc_in;
  assign {vc[6],vc[5],vc[4],vc[3],vc[2]} = vcpipe_q;
  assign vcpipe_en = pipe_advance[5:2] & {3'b1,ctrl2_memunit_en};

//module instance 
 pipe_32_4 vcpipe (
      .d(vcpipe_d),
      .clk(clk),
      .resetn(resetn),
      .en(vcpipe_en),
      .squash(squash_vcpipe_NC),
      .q(vcpipe_q));

  wire [5:2] squash_vlpipe_NC;

//module instance 
  pipe_32_4 vlpipe (
      .d( (!pipe_advance_s2_r) ? vl_in_saved : vl_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[5:2] & {3'b1,ctrl2_memunit_en} ),
      .squash(squash_vlpipe_NC),
      .q( {vl[6],vl[5],vl[4],vl[3],vl[2]} ));

  wire [5:2] squash_vbasepipe_NC;

//module instance 
  pipe_32_4 vbasepipe (
      .d( (!pipe_advance_s2_r) ? vbase_in_saved : vbase_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[5:2] & {3'b1,ctrl2_memunit_en} ),
      .squash(squash_vbasepipe_NC),
      .q( {vbase[6],vbase[5],vbase[4],vbase[3],vbase[2]} ));

  wire [5:2] squash_vincpipe_NC;

//module instance 
  pipe_32_4 vincpipe (
      .d( (!pipe_advance_s2_r) ? vinc_in_saved : vinc_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[5:2] & {3'b1,ctrl2_memunit_en} ),
      .squash(squash_vincpipe_NC),
      .q( {vinc[6],vinc[5],vinc[4],vinc[3],vinc[2]} ));

  //stride register also used for elmt shamt for vext/vhalf/etc in memunit
  wire [5:2] squash_vstridepipe_NC;

//module instance 
  pipe_32_4 vstridepipe (
      .d( (ctrl2_memunit_en&~ctrl2_mem_en) ? 
            ((2>0) ?
                  total_shamt[((2>0) ? 2 : 1)-1:0] : 0) :
          (!pipe_advance_s2_r) ? vstride_in_saved : vstride_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[5:2] & {3'b1,ctrl2_memunit_en} ),
      .squash(squash_vstridepipe_NC),
      .q( {vstride[6],vstride[5],vstride[4],vstride[3],vstride[2]} ));


  /*****************************    ISSUER    *****************************/

  assign stall_dispatcher=
    //Structural hazard
    (|(ctrl3_memunit_en&~last_subvector) && ctrl2_memunit_en) ||
    (|(ctrl3_mulshift_en&~last_subvector) && ctrl2_mulshift_en) ||
    (|(ctrl3_matmul_en&~last_subvector) && ctrl2_matmul_en) ||
    (|(ctrl3_bfadder_en&~last_subvector) && ctrl2_bfadder_en) ||
    (|(ctrl3_bfmult_en&~last_subvector) && ctrl2_bfmult_en) ||
    (|(ctrl3_act_en&~last_subvector) && ctrl2_act_en) ||
    (|(ctrl3_trp_en&~last_subvector) && ctrl2_trp_en) ||
    (|(ctrl3_alufalu_en&~last_subvector) && ctrl2_alufalu_en && 1==0)||
    //Entry 0 is taken
    (dispatcher_rotate && ctrl2_useslanes);

  //assign stall_mulcooldown=|(ctrl3_mulshift_en&last_subvector);
  assign stall_mulcooldown=1'b0;

  assign dispatcher_shift=pipe_advance[3];
  // Rotate if last entry is alive (count not -1) and greater than zero
  assign dispatcher_rotate=(~count[2*(7-2+1)-1]) && 
                          ((count>>((2-1)*(7-2+1)))!=0);

wire [2*(`DISPATCHWIDTH)-1:0] dispatcher_instr;

//module instance
  vdispatcher_2_157_7_7__3 vdispatcher(
      .clk(clk),
      .resetn(resetn),
      .shift(dispatcher_shift),
      .rotate(dispatcher_rotate), //rotate and shift must be high to rotate
      .inshift_instr( {
          src_limit,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vr_c_we,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vf_c_we,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vs_we,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vr_a_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vr_b_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vf_a_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vf_b_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_useslanes,
          (pipe_squash[2]) ? 1'b0 : ctrl2_memunit_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_mem_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_mulshift_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_matmul_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_bfadder_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_bfmult_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_act_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_trp_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_alufalu_en,
          dst_s2,
          src1_s2,
          src2_s2,
          src1scalar_s2,
          src2scalar_s2,
          imask_s2,
          ctrl2_vf_a_sel,
          ctrl2_rshiftnonzero,
          ctrl2_mem_dir_left,
          ctrl2_alu_op,
          ctrl2_satsum_op,
          ctrl2_satsize_op,
          ctrl2_mulshift_op,
          ctrl2_memunit_op,
          ctrl2_ismasked,
          ctrl2_flagalu_op,
          ctrl2_vf_wbsel,
          ctrl2_volatiledest,
          (!pipe_advance_s2_r) ? vc_in_saved : vc_in,
          (!pipe_advance_s2_r) ? vs_in_saved : vs_in,
          (pipe_squash[2]) ? 8'b0 : D_instr[2]
          }),
      .inshift_first(ctrl2_useslanes),
      .inshift_rdelm(src_start),
      .inshift_wrelm(dst_start),
      .inshift_count((pipe_squash[2] || !ctrl2_useslanes) ? -1 : (src_limit[7-1:0]>>2) - (src_elm[7-1:0]>>2) ),
      .increment((~wrongbank_s3)&{2{pipe_advance[3]}}),
      .rdelm_add_sub(0),
      .wrelm_add_sub(0),
      .count_add_sub(1),
      .rdelm_valuetoadd(4),
      .wrelm_valuetoadd(4),
      .count_valuetoadd(1),
      .instr(dispatcher_instr),
      .first(first_subvector),
      .rdelm(rdelm),
      .wrelm(wrelm),
      .count(count)
    );

  

  /******************* Stall on RAW HAZARDS ************************/

  always@*
    for (bd=0; bd<1+(2-1)*1; bd=bd+1)
    begin
      alu_dst[4][bd*REGIDWIDTH +: REGIDWIDTH]=dst[FU_ALU+bd][4];
      alu_dst_we[4][bd]=dst_we[FU_ALU+bd][4];
      falu_dst[4][bd*REGIDWIDTH +: REGIDWIDTH]=dst[FU_FALU+bd][4];
      falu_dst_we[4][bd]=dst_we[FU_FALU+bd][4];
    end

//module instance 
  hazardchecker_10_5_5_2 src1hazchecker(
      .src( src1_s2 |(src_start_delayed[7-1:0]>>2) ),
      .src_valid(ctrl2_vr_a_en),
      .dst({
        alu_dst[4],
        dst[FU_MUL][4],
        dst[FU_MEM][4],
        dst[FU_MATMUL][4]
        }),
      .dst_valid({
        alu_dst_we[4],
        dst_we[FU_MUL][4],
        dst_we[FU_MEM][4],
        dst_we[FU_MATMUL][4]
        }),
      .dst_mode({{3+(2-1)*1{1'b0}},ctrl4_volatiledest}),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vr_c_we),
      .lt_mode(ctrl3_volatiledest),
      .haz(stall_hazsrc1));
//module instance 

  hazardchecker_10_5_5_2 src2hazchecker(
      .src( src2_s2 |(src_start_delayed[7-1:0]>>2) ),
      .src_valid(ctrl2_vr_b_en),
      .dst({
        alu_dst[4],
        dst[FU_MUL][4],
        dst[FU_MEM][4],
        dst[FU_MATMUL][4]
        }),
      .dst_valid({
        alu_dst_we[4],
        dst_we[FU_MUL][4],
        dst_we[FU_MEM][4],
        dst_we[FU_MATMUL][4]
        }),
      .dst_mode({{3+(2-1)*1{1'b0}},ctrl4_volatiledest}),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vr_c_we),
      .lt_mode(ctrl3_volatiledest),
      .haz(stall_hazsrc2));

  //Check flag hazards - flags always start at 0th element
//module instance 
  hazardchecker_10_5_2_2 fsrc1hazchecker(
      // Check for mask flag and src1 flag depending on instruction
      .src( (ctrl2_vf_a_sel) ? src1_s2 : (imask_s2<<VELMIDWIDTH)),
      .src_valid(ctrl2_vf_a_en),
      .dst(falu_dst[4]),
      .dst_valid(falu_dst_we[4]),
      .dst_mode(0),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vf_c_we),
      .lt_mode(0),
      .haz(stall_hazfsrc1));

//module instance 
  hazardchecker_10_5_2_2 fsrc2hazchecker(
      .src( src2_s2 ),
      .src_valid(ctrl2_vf_b_en),
      .dst(falu_dst[4]),
      .dst_valid(falu_dst_we[4]),
      .dst_mode(0),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vf_c_we),
      .lt_mode(0),
      .haz(stall_hazfsrc2));


/******************************************************************************/
/************************** 3rd Pipeline Stage ********************************/
/******************************************************************************/

  always@*
    for (bi=0; bi<2; bi=bi+1)
    begin
      {
        src_limit_s3[bi],
        ctrl3_vr_c_we[bi],
        ctrl3_vf_c_we[bi],
        ctrl3_vs_we[bi],
        ctrl3_vr_a_en[bi],
        ctrl3_vr_b_en[bi],
        ctrl3_vf_a_en[bi],
        ctrl3_vf_b_en[bi],
        ctrl3_useslanes[bi],
        ctrl3_memunit_en[bi],
        ctrl3_mem_en[bi],
        ctrl3_mulshift_en[bi],
        ctrl3_matmul_en[bi],
        ctrl3_bfadder_en[bi],
        ctrl3_bfmult_en[bi],
        ctrl3_act_en[bi],
        ctrl3_trp_en[bi],
        ctrl3_alufalu_en[bi],
        _dst_s3[bi],
        _src1_s3[bi],
        _src2_s3[bi],
        src1scalar_s3[bi],
        src2scalar_s3[bi],
        imask_s3[bi],
        ctrl3_vf_a_sel[bi],
        ctrl3_rshiftnonzero[bi],
        ctrl3_mem_dir_left[bi],
        ctrl3_alu_op[bi],
        ctrl3_satsum_op[bi],
        ctrl3_satsize_op[bi],
        ctrl3_mulshift_op[bi],
        ctrl3_memunit_op[bi],
        ctrl3_ismasked[bi],
        ctrl3_flagalu_op[bi],
        ctrl3_vf_wbsel[bi],
        ctrl3_volatiledest[bi],
        vc_s3[bi],
        vs_s3[bi],
        D_instr_s3[bi]
      } = dispatcher_instr>>(bi*(`DISPATCHWIDTH));

      last_subvector[bi]=~|count[bi*(7-2+1)+:7-2];
      alive_s3[bi]=~count[(bi+1)*(7-2+1)-1];

      for (i=0; i<4; i=i+1)
        lane_en[bi][i]= (2==0);
        //amana: Removed support for 1 lane. This is not a usecase for us. 
        //The code was just causing errors with the parser to convert code 
        //to VTR compatible code.
        // || //Support 1 lane
        //  ~((first_subvector[bi]) && 
        //      i<`LO(rdelm[bi*7 +: 7],2) ||
        //    (last_subvector[bi]) && 
        //      i>src_limit_s3[bi][((2>0) ? 2 : 1)-1:0] );
    end


  // ************* Map from issuer to register file banks *************
  always@*
    for (b=0; b<2; b=b+1)
    begin 
      dst_s3[b]=_dst_s3[b] | (wrelm[b*7+:7]>>2);
      t_dst_s3[b*REGIDWIDTH +: REGIDWIDTH]=dst_s3[b];
      src1_s3[b]=_src1_s3[b] | (rdelm[b*7+:7]>>2);
      src2_s3[b]=_src2_s3[b] | (rdelm[b*7+:7]>>2);

      wrongbank_s3[b]=(`LO(rdelm[b*7+:7]>>2,1)!=b);

      vr_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src1_s3[b]>>1;
      vr_b_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src2_s3[b]>>1;
      vr_a_en[b]=ctrl3_vr_a_en[b] && pipe_advance[3];
      vr_b_en[b]=ctrl3_vr_b_en[b] && pipe_advance[3];

      vf_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]= 
        {(ctrl3_vf_a_sel[b]) ? _src1_s3[b][`VRID_RANGE] : imask_s3[b],
        src1_s3[b][`VRELM_RANGE]}>>1;
      vf_b_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src2_s3[b]>>1;
      vf_a_en[b]=ctrl3_vf_a_en[b] && pipe_advance[3];
      vf_b_en[b]=ctrl3_vf_b_en[b] && pipe_advance[3];
    end

  // ************* Map from issuer/banks to Functional Units *************
  always@(posedge clk)
    if (!resetn)
    begin
      dst_we_s4=0;
      ctrl4_mulshift_en=0;
      ctrl4_matmul_en=0;
      ctrl4_bfadder_en=0;
      ctrl4_bfmult_en=0;
      ctrl4_act_en=0;
      ctrl4_trp_en=0;
      ctrl4_memunit_en=0;
      ctrl4_mem_en=0;
      ctrl4_volatiledest=0;
    end
    else if (pipe_advance[3])
    begin
      dst_we_s4=0;
      ctrl4_mulshift_en=0;
      ctrl4_matmul_en=0;
      ctrl4_bfadder_en=0;
      ctrl4_bfmult_en=0;
      ctrl4_act_en=0;
      ctrl4_trp_en=0;
      ctrl4_memunit_en=0;
      ctrl4_mem_en=|ctrl3_mem_en;
      ctrl4_volatiledest=0;

      for (f3=0; f3<NUMFUS; f3=f3+1)
      begin
        D_instr_s4[f3]=0;
        D_last_subvector_s4[f3]=0;
        dst_s4[f3]=0;
      end

      for (b3=0; b3<2; b3=b3+1)
      begin
        //if instruction is alive && is in correct bank
        if ( alive_s3[b3]  &&     //alive
            ~wrongbank_s3[b3] &&  //correct bank
            ~pipe_squash[3])
          if (ctrl3_mulshift_en[b3])    //is multiply
          begin
            D_instr_s4[FU_MUL]=D_instr_s3[b3];
            D_last_subvector_s4[FU_MUL]=last_subvector[b3];
            ctrl4_mulshift_en=ctrl3_mulshift_en[b3];
            banksel_s4[FU_MUL]=b3;
            vs_s4[FU_MUL]=vs_s3[b3];
            vc_s4[FU_MUL]=vc_s3[b3];
            dst_s4[FU_MUL]=dst_s3[b3];
            dst_we_s4[FU_MUL]=ctrl3_vr_c_we[b3];
            vlane_en[FU_MUL]=lane_en[b3];
            src1scalar_s4[FU_MUL]=src1scalar_s3[b3];
            src2scalar_s4[FU_MUL]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MUL]=ctrl3_ismasked[b3];
            ctrl4_mulshift_op=ctrl3_mulshift_op[b3];
            ctrl4_rshiftnonzero=ctrl3_rshiftnonzero[b3];
          end
          else if (ctrl3_matmul_en[b3])    //is matmul
          begin
            D_instr_s4[FU_MATMUL]=D_instr_s3[b3];
            D_last_subvector_s4[FU_MATMUL]=last_subvector[b3];
            ctrl4_matmul_en=ctrl3_matmul_en[b3];
            banksel_s4[FU_MATMUL]=b3;
            vs_s4[FU_MATMUL]=vs_s3[b3];
            vc_s4[FU_MATMUL]=vc_s3[b3];
            dst_s4[FU_MATMUL]=dst_s3[b3];
            dst_we_s4[FU_MATMUL]=ctrl3_vr_c_we[b3];
            vlane_en[FU_MATMUL]=lane_en[b3];
            src1scalar_s4[FU_MATMUL]=src1scalar_s3[b3];
            src2scalar_s4[FU_MATMUL]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MATMUL]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_bfadder_en[b3])    //is bfloat addition
          begin
            D_instr_s4[FU_BFADDER]=D_instr_s3[b3];
            D_last_subvector_s4[FU_BFADDER]=last_subvector[b3];
            ctrl4_bfadder_en=ctrl3_bfadder_en[b3];
            banksel_s4[FU_BFADDER]=b3;
            vs_s4[FU_BFADDER]=vs_s3[b3];
            vc_s4[FU_BFADDER]=vc_s3[b3];
            dst_s4[FU_BFADDER]=dst_s3[b3];
            dst_we_s4[FU_BFADDER]=ctrl3_vr_c_we[b3];
            vlane_en[FU_BFADDER]=lane_en[b3];
            src1scalar_s4[FU_BFADDER]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFADDER]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFADDER]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_bfmult_en[b3])    //is bfloat addition
          begin
            D_instr_s4[FU_BFMULT]=D_instr_s3[b3];
            D_last_subvector_s4[FU_BFMULT]=last_subvector[b3];
            ctrl4_bfmult_en = ctrl3_bfmult_en[b3];
            banksel_s4[FU_BFMULT]=b3;
            vs_s4[FU_BFMULT]=vs_s3[b3];
            vc_s4[FU_BFMULT]=vc_s3[b3];
            dst_s4[FU_BFMULT]=dst_s3[b3];
            dst_we_s4[FU_BFMULT]=ctrl3_vr_c_we[b3];
            vlane_en[FU_BFMULT]=lane_en[b3];
            src1scalar_s4[FU_BFMULT]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFMULT]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFMULT]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_trp_en[b3])    //is bfloat addition
          begin
            D_instr_s4[FU_BFMULT]=D_instr_s3[b3];
            D_last_subvector_s4[FU_BFMULT]=last_subvector[b3];
            ctrl4_act_en = ctrl3_act_en[b3];
            banksel_s4[FU_BFMULT]=b3;
            vs_s4[FU_BFMULT]=vs_s3[b3];
            vc_s4[FU_BFMULT]=vc_s3[b3];
            dst_s4[FU_BFMULT]=dst_s3[b3];
            dst_we_s4[FU_BFMULT]=ctrl3_vr_c_we[b3];
            vlane_en[FU_BFMULT]=lane_en[b3];
            src1scalar_s4[FU_BFMULT]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFMULT]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFMULT]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_act_en[b3])    //is bfloat addition
          begin
            D_instr_s4[FU_TRP]=D_instr_s3[b3];
            D_last_subvector_s4[FU_TRP]=last_subvector[b3];
            ctrl4_act_en = ctrl3_act_en[b3];
            banksel_s4[FU_TRP]=b3;
            vs_s4[FU_TRP]=vs_s3[b3];
            vc_s4[FU_TRP]=vc_s3[b3];
            dst_s4[FU_TRP]=dst_s3[b3];
            dst_we_s4[FU_TRP]=ctrl3_vr_c_we[b3];
            vlane_en[FU_TRP]=lane_en[b3];
            src1scalar_s4[FU_TRP]=src1scalar_s3[b3];
            src2scalar_s4[FU_TRP]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_TRP]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_memunit_en[b3] && !ctrl3_mem_en[b3]) //is memunit shift
          begin
            D_instr_s4[FU_MEM]=D_instr_s3[b3];
            D_last_subvector_s4[FU_MEM]=last_subvector[b3];
            ctrl4_memunit_en=ctrl3_memunit_en[b3];
            banksel_s4[FU_MEM]=b3;
            vs_s4[FU_MEM]=vs_s3[b3];
            dst_s4[FU_MEM]=dst_s3[b3];
            dst_we_s4[FU_MEM]=ctrl3_vr_c_we[b3];
            vlane_en[FU_MEM]=lane_en[b3];
            src1scalar_s4[FU_MEM]=src1scalar_s3[b3];
            src2scalar_s4[FU_MEM]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MEM]=ctrl3_ismasked[b3];
            ctrl4_volatiledest=ctrl3_volatiledest[b3];
            ctrl4_mem_dir_left=ctrl3_mem_dir_left[b3];
            ctrl4_memunit_op=ctrl3_memunit_op[b3];
            mem_last_subvector_s4=last_subvector[b3];
          end
          else if (ctrl3_memunit_en[b3] &&  ctrl3_mem_en[b3]) //is mem operation
          begin
            D_instr_s4[FU_MEM]=D_instr_s3[b3];
            D_last_subvector_s4[FU_MEM]=last_subvector[b3];
            ctrl4_memunit_en=ctrl3_memunit_en[b3];
            banksel_s4[FU_MEM]=b3;
            // Use vs to store current length for prefetching
            vs_s4[FU_MEM]=count[b3*(7-2+1)+:(7-2)]<<2;
            dst_s4[FU_MEM]=dst_s3[b3];
            dst_we_s4[FU_MEM]=ctrl3_vr_c_we[b3];
            vlane_en[FU_MEM]=lane_en[b3];
            src1scalar_s4[FU_MEM]=src1scalar_s3[b3];
            src2scalar_s4[FU_MEM]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MEM]=ctrl3_ismasked[b3];
            ctrl4_volatiledest=ctrl3_volatiledest[b3];
            ctrl4_mem_dir_left=ctrl3_mem_dir_left[b3];
            ctrl4_memunit_op=ctrl3_memunit_op[b3];
            //Load base on first subvector or if INDEXED memory operation
            vbase_s4=(|(first_subvector&ctrl3_mem_en) || ctrl_memunit_op[3][5])?
              vbase[3] : vbase_s4 + ((((ctrl_memunit_op[3][4])? vstride[3]: 1)
                      <<ctrl_memunit_op[3][3:2])<<2);
            // Partial Address Gen for each lane - just do multiplication part
            for (m=0; m<4; m=m+1)
              vstrideoffset_s4[m*32 +: 32] = 
                                ((ctrl_memunit_op[3][4]) ? vstride[3] : 1)*m;
            mem_last_subvector_s4=last_subvector[b3];
          end
          else if (ctrl3_alu_op[b3]!=(ALUOP_ZERO^ALUOP_ZERO)) //is ALU
          begin
            D_instr_s4[FU_ALU+b3*1]=D_instr_s3[b3];
            D_last_subvector_s4[FU_ALU+b3*1]=last_subvector[b3];
            banksel_s4[FU_ALU+b3*1]=b3;
            vs_s4[FU_ALU+b3*1]=vs_s3[b3];
            dst_we_s4[FU_ALU+b3*1]=ctrl3_vr_c_we[b3];
            dst_s4[FU_ALU+b3*1]=dst_s3[b3];
            dst_we_s4[FU_FALU+b3*1]=ctrl3_vf_c_we[b3];
            dst_s4[FU_FALU+b3*1]=dst_s3[b3];
            vlane_en[FU_ALU+b3*1]=lane_en[b3];
            src1scalar_s4[FU_ALU+b3*1]=src1scalar_s3[b3];
            src2scalar_s4[FU_ALU+b3*1]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_ALU+b3*1]=ctrl3_ismasked[b3];
            ctrl4_alu_op[b3*1]=ctrl3_alu_op[b3];
            ctrl4_satsum_op[b3*1]=ctrl3_satsum_op[b3];
            ctrl4_satsize_op[b3*1]=ctrl3_satsize_op[b3];
            ctrl4_vf_wbsel[b3*1]=ctrl3_vf_wbsel[b3];
          end
          else if (ctrl3_vf_c_we[b3])
          begin                                    //is FALU
            D_instr_s4[FU_FALU+b3*1]=D_instr_s3[b3];
            D_last_subvector_s4[FU_FALU+b3*1]=last_subvector[b3];
            banksel_s4[FU_FALU+b3*1]=b3;
            vs_s4[FU_FALU+b3*1]=|vs_s3[b3];
            dst_we_s4[FU_FALU+b3*1]=ctrl3_vf_c_we[b3];
            dst_s4[FU_FALU+b3*1]=dst_s3[b3];
            vlane_en[FU_FALU+b3*1]=lane_en[b3];
            src1scalar_s4[FU_FALU+b3*1]=src1scalar_s3[b3];
            src2scalar_s4[FU_FALU+b3*1]=src2scalar_s3[b3];
            ctrl4_flagalu_op[b3*1]=ctrl3_flagalu_op[b3];
            ctrl4_vf_wbsel[b3*1]=ctrl3_vf_wbsel[b3];
            ctrl4_vf_a_sel[b3*1]=ctrl3_vf_a_sel[b3];
          end
        end
      end

// module instance
  vregfile_vector_2_1_128_1024_10 vregfile_vector (
      .clk(clk),
      .resetn(resetn),
      .a_reg(vr_a_reg),
      .a_readdataout(_vr_a_readdataout),
      .a_en(vr_a_en),
      .b_reg(vr_b_reg),
      .b_readdataout(_vr_b_readdataout),
      .b_en(vr_b_en),
      .c_reg(_vr_c_reg),
      .c_writedatain(_vr_c_writedatain),
      .c_byteen(vr_c_byteen),
      .c_we(vr_c_we));

// module instance
  vregfile_flag_2_1_4_1024_10 
    vregfile_flag (
      .clk(clk),
      .resetn(resetn),
      .a_reg(vf_a_reg),
      .a_readdataout(vf_a_readdataout),
      .a_en(vf_a_en),
      .b_reg(vf_b_reg),
      .b_readdataout(vf_b_readdataout),
      .b_en(vf_b_en),
      .c_reg(vf_c_reg),
      .c_writedatain(vf_c_writedatain),
      .c_we(vf_c_we));


// module instance
  pipereg_1 sdstwepipereg (
      .d( |ctrl3_vs_we ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[3] ),
      .squashn( ~pipe_squash[3] ),
      .q(ctrl_vs_we[4]));

/******************************************************************************/
/************************** 4th Pipeline Stage ********************************/
/******************************************************************************/

  //Convert register file width (8*4) to datpath width (32)
  always@*
    for (bn=0; bn<2; bn=bn+1)
      for (n=0; n<4; n=n+1)
      begin
        vr_a_readdataout[bn][32*n +: 32] = 
          _vr_a_readdataout[bn*8*4*4 + 8*4*n +: 8*4];
        vr_b_readdataout[bn][32*n +: 32] = 
          _vr_b_readdataout[bn*8*4*4 + 8*4*n +: 8*4];
        _vr_c_writedatain[bn*8*4*4 + 8*4*n +: 8*4] = 
          vr_c_writedatain[bn][32*n +: 32];
      end

  //Bank Multiplex for each functional unit
  always@*
  begin
    for (fn=0; fn<=FU_MUL; fn=fn+1)
    begin
      vr_src1[fn] =(src1scalar_s4[fn]) ? {4{vs_s4[fn][32-1:0]}} : 
                                    vr_a_readdataout[banksel_s4[fn]];
      vr_src2[fn] =(src2scalar_s4[fn]) ? {4{vs_s4[fn][32-1:0]}} : 
                                    vr_b_readdataout[banksel_s4[fn]];
      vf_src1[fn] = vf_a_readdataout[banksel_s4[fn]*4 +: 4];

      vmask[fn] =vlane_en[fn] &
            ((ctrl4_ismasked[fn]) ?
              vf_a_readdataout[banksel_s4[fn]*4 +: 4] :
              {4{1'b1}}) ;
    end

    vr_src1[FU_MEM] = vr_a_readdataout[banksel_s4[FU_MEM]];
    vr_src2[FU_MEM] = vr_b_readdataout[banksel_s4[FU_MEM]];
    vmask[FU_MEM] =  vlane_en[FU_MEM] &
           ((ctrl4_ismasked[FU_MEM]) ?  
             vf_a_readdataout[banksel_s4[FU_MEM]*4 +: 4] :
             {4{1'b1}}) ;

    vr_src1[FU_MATMUL] =(src1scalar_s4[FU_MATMUL]) ? {4{vs_s4[FU_MATMUL][32-1:0]}} : 
                                    vr_a_readdataout[banksel_s4[FU_MATMUL]];
    vr_src2[FU_MATMUL] =(src2scalar_s4[FU_MATMUL]) ? {4{vs_s4[FU_MATMUL][32-1:0]}} : 
                                    vr_b_readdataout[banksel_s4[FU_MATMUL]];
    vmask[FU_MATMUL] =  vlane_en[FU_MATMUL] &
           ((ctrl4_ismasked[FU_MATMUL]) ?  
             vf_a_readdataout[banksel_s4[FU_MATMUL]*4 +: 4] :
             {4{1'b1}}) ;

    for (fn2=FU_FALU; fn2<=FU_FALU+(2-1)*1; fn2=fn2+1)
    begin
      vf_src1[fn2] =(src1scalar_s4[fn2]&ctrl4_vf_a_sel[fn2-FU_FALU]) ? {4{vs_s4[fn2][0]}} :
        vf_a_readdataout[banksel_s4[fn2]*4 +: 4];

      //Only FALU uses this
      vf_src2[fn2] = (src2scalar_s4[fn2]) ? {4{vs_s4[fn2][0]}} : 
        vf_b_readdataout[banksel_s4[fn2]*4 +: 4];
    end

  end

/******************************************************************************/
/************************** 5th Pipeline Stage ********************************/
/******************************************************************************/

  // *********************** FUNCTIONAL UNITS *********************
  //
  // Note that functional units (recently) carry the register they're
  // writing to and a write signal so that once the result is computed
  // it can issue the full register write request.  As a result, it 
  // needs to know the status of the pipeline (pipe_advance and pipe_squash)

  // If mem_unit is stalled, pipeline can still go on if both
  //    i) it's a store hence requiring no writeback bank, AND
  //    ii) another memory instruction isn't waiting on the memunit
  assign stall_memunit=_stall_memunit && (dst_we[FU_MEM][5] ||
    (ctrl4_memunit_en && ~stall_mulunit && ~stall_matmul));

//module instance 

  pipereg_1 memen5pipereg (
      .d( ctrl4_mem_en ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] && ~_stall_memunit ),
      .squashn( ~pipe_squash[4] | _stall_memunit ),
      .q(ctrl5_mem_en ));

 //============== Memory Unit =============

//module instance 

  vmem_unit_32_4_2_2_1_32_128_7_128_7_5_10 vmem_unit(
    .clk(clk),
    .resetn(resetn),
    .enable(ctrl4_memunit_en && ~stall_mulunit && ~stall_matmul),  //unit on and pipe active
    .en(pipe_advance[6:4]),
    .squash(pipe_squash[6:4]),
    .op(ctrl4_memunit_op),
    .stall(_stall_memunit),
    .last_subvector(mem_last_subvector_s4),
    // Parameters ports
    .cbase(vbase_s4),
    .cstride(vstride[4]),
    .cprefetch(vs_s4[FU_MEM]),
    // Vector ports
    .vmask(vmask[FU_MEM]),
    .vstrideoffset(vstrideoffset_s4),
    .vindex(vr_src1[FU_MEM]),
    .vwritedata(vr_src2[FU_MEM]),
    .voutput(load_result_s5),
    .voutput_we(load_result_mask_s5),
    // Writeback ports
    .in_dst(dst_s4[FU_MEM]),
    .in_dst_we(dst_we_s4[FU_MEM]),
    .out_dst(dst[FU_MEM][5]),
    .out_dst_we(dst_we[FU_MEM][5]),
    .in_vs_dst_we(ctrl_vs_we[4]),
    .out_vs_dst_we(ctrl_vs_we[5]),
    // Vector operations ports
    .sa(vstride[4]),
    .dir_left(ctrl4_mem_dir_left),
    // Data memory interface
    .dmem_en(dbus_en),
    .dmem_address(dbus_address),
    .dmem_we(dbus_we),
    .dmem_byteen(dbus_byteen),
    .dmem_writedata(dbus_writedata),
    .dmem_readdata(dbus_readdata),
    .dmem_cachematch(dbus_cachematch),
    .dmem_cachemiss(dbus_cachemiss),
    .dmem_prefetch(dbus_prefetch),
    .dmem_wait(dbus_wait)
    );


  assign dst[FU_MEM][4]=dst_s4[FU_MEM];
  assign dst_we[FU_MEM][4]=dst_we_s4[FU_MEM];

  //============== Multiplier Unit (spans stages 4-6) =============

//module instance 

  vmul_unit_5_4_2_10 vmul_unit(
    .clk(clk),
    .resetn(resetn),
    .op(ctrl4_mulshift_op),
    .activate(ctrl4_mulshift_en),
    .en(pipe_advance[6:4]),
    .squash(pipe_squash[6:4]),
    .stall(stall_mulunit),
    .opA(vr_src1[FU_MUL]),
    .opB(vr_src2[FU_MUL]),
    .vshamt( (ctrl4_rshiftnonzero) ? vc_s4[FU_MUL] : 0 ),
    .vmask(vmask[FU_MUL]),
    .in_dst(dst_s4[FU_MUL]),
    .in_dst_we(dst_we_s4[FU_MUL]),
    .out_dst({dst[FU_MUL][6],dst[FU_MUL][5],dst[FU_MUL][4]}),
    .out_dst_we(dst_we[FU_MUL][6:4]),
    .out_dst_mask({dst_mask[FU_MUL][6],dst_mask[FU_MUL][5],dst_mask[FU_MUL][4]}),
    .result(mulshift_result_s5)
  );

  //============== ALU Unit =============

  //If APB value is true, create one ALU per bank (per lane)

  wire squash_aludstpipe_NC;
wire squash_faludstpipe_NC;
wire squash_aludstmaskpipe_NC;


//module instance 

    vlane_alu_32 valu_0_0(
        .clk(clk), 
        .resetn(resetn),
        .pipe_en(pipe_advance[4]),
        .pipe_squashn(~pipe_squash[4]),
        .src1(vr_src1[FU_ALU+0][32*0 +: 32]),
        .src2(vr_src2[FU_ALU+0][32*0 +: 32]),
        .mask(vf_src1[FU_ALU+0][0]),  //Note: MERGE is not masked
        .op(ctrl4_alu_op[0]^ALUOP_ZERO),
        .satsum_op(ctrl4_satsum_op[0]),
        .satsize_op(ctrl4_satsize_op[0]),
        .cmp_result(alu_cmpresult_s4[0][0]),
        .result(alu_result_s5[0][32*0 +: 32])
        );
//module instance 

      vlane_flagalu vflagalu_0_0(
        .clk(clk), 
        .resetn(resetn),
        .src1(vf_src1[FU_FALU+0][0]),
        .src2(vf_src2[FU_FALU+0][0]),
        .op(ctrl4_flagalu_op[0]),
        .result(flagalu_result_s4[0][0])
        );

//module instance 
      pipereg_1 flagaluresultreg_0_0 (
        .d( (ctrl4_vf_wbsel[0]) ? alu_cmpresult_s4[0][0] : 
                                   flagalu_result_s4[0][0]),
        .clk(clk),
        .resetn(resetn),
        .en( pipe_advance[4] ), //Advance unless next stage stalled
        .squashn( 1'b1 ),
        .q(flagalu_result_s5[0][0])
        );


//module instance 

    vlane_alu_32 valu_1_0(
        .clk(clk), 
        .resetn(resetn),
        .pipe_en(pipe_advance[4]),
        .pipe_squashn(~pipe_squash[4]),
        .src1(vr_src1[FU_ALU+0][32*1 +: 32]),
        .src2(vr_src2[FU_ALU+0][32*1 +: 32]),
        .mask(vf_src1[FU_ALU+0][1]),  //Note: MERGE is not masked
        .op(ctrl4_alu_op[0]^ALUOP_ZERO),
        .satsum_op(ctrl4_satsum_op[0]),
        .satsize_op(ctrl4_satsize_op[0]),
        .cmp_result(alu_cmpresult_s4[0][1]),
        .result(alu_result_s5[0][32*1 +: 32])
        );
//module instance 

      vlane_flagalu vflagalu_1_0(
        .clk(clk), 
        .resetn(resetn),
        .src1(vf_src1[FU_FALU+0][1]),
        .src2(vf_src2[FU_FALU+0][1]),
        .op(ctrl4_flagalu_op[0]),
        .result(flagalu_result_s4[0][1])
        );

//module instance 
      pipereg_1 flagaluresultreg_1_0 (
        .d( (ctrl4_vf_wbsel[0]) ? alu_cmpresult_s4[0][1] : 
                                   flagalu_result_s4[0][1]),
        .clk(clk),
        .resetn(resetn),
        .en( pipe_advance[4] ), //Advance unless next stage stalled
        .squashn( 1'b1 ),
        .q(flagalu_result_s5[0][1])
        );


//module instance 

    vlane_alu_32 valu_2_0(
        .clk(clk), 
        .resetn(resetn),
        .pipe_en(pipe_advance[4]),
        .pipe_squashn(~pipe_squash[4]),
        .src1(vr_src1[FU_ALU+0][32*2 +: 32]),
        .src2(vr_src2[FU_ALU+0][32*2 +: 32]),
        .mask(vf_src1[FU_ALU+0][2]),  //Note: MERGE is not masked
        .op(ctrl4_alu_op[0]^ALUOP_ZERO),
        .satsum_op(ctrl4_satsum_op[0]),
        .satsize_op(ctrl4_satsize_op[0]),
        .cmp_result(alu_cmpresult_s4[0][2]),
        .result(alu_result_s5[0][32*2 +: 32])
        );
//module instance 

      vlane_flagalu vflagalu_2_0(
        .clk(clk), 
        .resetn(resetn),
        .src1(vf_src1[FU_FALU+0][2]),
        .src2(vf_src2[FU_FALU+0][2]),
        .op(ctrl4_flagalu_op[0]),
        .result(flagalu_result_s4[0][2])
        );

//module instance 
      pipereg_1 flagaluresultreg_2_0 (
        .d( (ctrl4_vf_wbsel[0]) ? alu_cmpresult_s4[0][2] : 
                                   flagalu_result_s4[0][2]),
        .clk(clk),
        .resetn(resetn),
        .en( pipe_advance[4] ), //Advance unless next stage stalled
        .squashn( 1'b1 ),
        .q(flagalu_result_s5[0][2])
        );


//module instance 

    vlane_alu_32 valu_3_0(
        .clk(clk), 
        .resetn(resetn),
        .pipe_en(pipe_advance[4]),
        .pipe_squashn(~pipe_squash[4]),
        .src1(vr_src1[FU_ALU+0][32*3 +: 32]),
        .src2(vr_src2[FU_ALU+0][32*3 +: 32]),
        .mask(vf_src1[FU_ALU+0][3]),  //Note: MERGE is not masked
        .op(ctrl4_alu_op[0]^ALUOP_ZERO),
        .satsum_op(ctrl4_satsum_op[0]),
        .satsize_op(ctrl4_satsize_op[0]),
        .cmp_result(alu_cmpresult_s4[0][3]),
        .result(alu_result_s5[0][32*3 +: 32])
        );
//module instance 

      vlane_flagalu vflagalu_3_0(
        .clk(clk), 
        .resetn(resetn),
        .src1(vf_src1[FU_FALU+0][3]),
        .src2(vf_src2[FU_FALU+0][3]),
        .op(ctrl4_flagalu_op[0]),
        .result(flagalu_result_s4[0][3])
        );

//module instance 
      pipereg_1 flagaluresultreg_3_0 (
        .d( (ctrl4_vf_wbsel[0]) ? alu_cmpresult_s4[0][3] : 
                                   flagalu_result_s4[0][3]),
        .clk(clk),
        .resetn(resetn),
        .en( pipe_advance[4] ), //Advance unless next stage stalled
        .squashn( 1'b1 ),
        .q(flagalu_result_s5[0][3])
        );


//module instance 

    pipe_10_1 aludstpipe_0 (
      .d( dst_s4[FU_ALU+0] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_aludstpipe_NC),
      .q({dst[FU_ALU+0][5],dst[FU_ALU+0][4]}));

//module instance 

   pipe_1_3 aludstwepipe_0 (
      .d( dst_we_s4[FU_ALU+0] & ~ctrl4_vf_wbsel[0] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:4] ),
      .squash( pipe_squash[6:4] ),
      .q({dst_we[FU_ALU+0][5],dst_we[FU_ALU+0][4]}));


//module instance 
    pipe_4_1 aludstmaskpipe_0 (
      .d( vmask[FU_ALU+0] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_aludstmaskpipe_NC),
      .q({dst_mask[FU_ALU+0][5],dst_mask[FU_ALU+0][4]}));

//module instance 
    pipe_10_1 faludstpipe_0 (
      .d( dst_s4[FU_FALU+0] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_faludstpipe_NC),
      .q({dst[FU_FALU+0][5],dst[FU_FALU+0][4]}));

//module instance 
    pipe_1_3 faludstwepipe_0 (
      .d( dst_we_s4[FU_FALU+0] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:4] ),
      .squash( pipe_squash[6:4] ),
      .q({dst_we[FU_FALU+0][5],dst_we[FU_FALU+0][4]}));



//module instance 

    vlane_alu_32 valu_0_1(
        .clk(clk), 
        .resetn(resetn),
        .pipe_en(pipe_advance[4]),
        .pipe_squashn(~pipe_squash[4]),
        .src1(vr_src1[FU_ALU+1][32*0 +: 32]),
        .src2(vr_src2[FU_ALU+1][32*0 +: 32]),
        .mask(vf_src1[FU_ALU+1][0]),  //Note: MERGE is not masked
        .op(ctrl4_alu_op[1]^ALUOP_ZERO),
        .satsum_op(ctrl4_satsum_op[1]),
        .satsize_op(ctrl4_satsize_op[1]),
        .cmp_result(alu_cmpresult_s4[1][0]),
        .result(alu_result_s5[1][32*0 +: 32])
        );
//module instance 

      vlane_flagalu vflagalu_0_1(
        .clk(clk), 
        .resetn(resetn),
        .src1(vf_src1[FU_FALU+1][0]),
        .src2(vf_src2[FU_FALU+1][0]),
        .op(ctrl4_flagalu_op[1]),
        .result(flagalu_result_s4[1][0])
        );

//module instance 
      pipereg_1 flagaluresultreg_0_1 (
        .d( (ctrl4_vf_wbsel[1]) ? alu_cmpresult_s4[1][0] : 
                                   flagalu_result_s4[1][0]),
        .clk(clk),
        .resetn(resetn),
        .en( pipe_advance[4] ), //Advance unless next stage stalled
        .squashn( 1'b1 ),
        .q(flagalu_result_s5[1][0])
        );


//module instance 

    vlane_alu_32 valu_1_1(
        .clk(clk), 
        .resetn(resetn),
        .pipe_en(pipe_advance[4]),
        .pipe_squashn(~pipe_squash[4]),
        .src1(vr_src1[FU_ALU+1][32*1 +: 32]),
        .src2(vr_src2[FU_ALU+1][32*1 +: 32]),
        .mask(vf_src1[FU_ALU+1][1]),  //Note: MERGE is not masked
        .op(ctrl4_alu_op[1]^ALUOP_ZERO),
        .satsum_op(ctrl4_satsum_op[1]),
        .satsize_op(ctrl4_satsize_op[1]),
        .cmp_result(alu_cmpresult_s4[1][1]),
        .result(alu_result_s5[1][32*1 +: 32])
        );
//module instance 

      vlane_flagalu vflagalu_1_1(
        .clk(clk), 
        .resetn(resetn),
        .src1(vf_src1[FU_FALU+1][1]),
        .src2(vf_src2[FU_FALU+1][1]),
        .op(ctrl4_flagalu_op[1]),
        .result(flagalu_result_s4[1][1])
        );

//module instance 
      pipereg_1 flagaluresultreg_1_1 (
        .d( (ctrl4_vf_wbsel[1]) ? alu_cmpresult_s4[1][1] : 
                                   flagalu_result_s4[1][1]),
        .clk(clk),
        .resetn(resetn),
        .en( pipe_advance[4] ), //Advance unless next stage stalled
        .squashn( 1'b1 ),
        .q(flagalu_result_s5[1][1])
        );


//module instance 

    vlane_alu_32 valu_2_1(
        .clk(clk), 
        .resetn(resetn),
        .pipe_en(pipe_advance[4]),
        .pipe_squashn(~pipe_squash[4]),
        .src1(vr_src1[FU_ALU+1][32*2 +: 32]),
        .src2(vr_src2[FU_ALU+1][32*2 +: 32]),
        .mask(vf_src1[FU_ALU+1][2]),  //Note: MERGE is not masked
        .op(ctrl4_alu_op[1]^ALUOP_ZERO),
        .satsum_op(ctrl4_satsum_op[1]),
        .satsize_op(ctrl4_satsize_op[1]),
        .cmp_result(alu_cmpresult_s4[1][2]),
        .result(alu_result_s5[1][32*2 +: 32])
        );
//module instance 

      vlane_flagalu vflagalu_2_1(
        .clk(clk), 
        .resetn(resetn),
        .src1(vf_src1[FU_FALU+1][2]),
        .src2(vf_src2[FU_FALU+1][2]),
        .op(ctrl4_flagalu_op[1]),
        .result(flagalu_result_s4[1][2])
        );

//module instance 
      pipereg_1 flagaluresultreg_2_1 (
        .d( (ctrl4_vf_wbsel[1]) ? alu_cmpresult_s4[1][2] : 
                                   flagalu_result_s4[1][2]),
        .clk(clk),
        .resetn(resetn),
        .en( pipe_advance[4] ), //Advance unless next stage stalled
        .squashn( 1'b1 ),
        .q(flagalu_result_s5[1][2])
        );


//module instance 

    vlane_alu_32 valu_3_1(
        .clk(clk), 
        .resetn(resetn),
        .pipe_en(pipe_advance[4]),
        .pipe_squashn(~pipe_squash[4]),
        .src1(vr_src1[FU_ALU+1][32*3 +: 32]),
        .src2(vr_src2[FU_ALU+1][32*3 +: 32]),
        .mask(vf_src1[FU_ALU+1][3]),  //Note: MERGE is not masked
        .op(ctrl4_alu_op[1]^ALUOP_ZERO),
        .satsum_op(ctrl4_satsum_op[1]),
        .satsize_op(ctrl4_satsize_op[1]),
        .cmp_result(alu_cmpresult_s4[1][3]),
        .result(alu_result_s5[1][32*3 +: 32])
        );
//module instance 

      vlane_flagalu vflagalu_3_1(
        .clk(clk), 
        .resetn(resetn),
        .src1(vf_src1[FU_FALU+1][3]),
        .src2(vf_src2[FU_FALU+1][3]),
        .op(ctrl4_flagalu_op[1]),
        .result(flagalu_result_s4[1][3])
        );

//module instance 
      pipereg_1 flagaluresultreg_3_1 (
        .d( (ctrl4_vf_wbsel[1]) ? alu_cmpresult_s4[1][3] : 
                                   flagalu_result_s4[1][3]),
        .clk(clk),
        .resetn(resetn),
        .en( pipe_advance[4] ), //Advance unless next stage stalled
        .squashn( 1'b1 ),
        .q(flagalu_result_s5[1][3])
        );


//module instance 

    pipe_10_1 aludstpipe_1 (
      .d( dst_s4[FU_ALU+1] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_aludstpipe_NC),
      .q({dst[FU_ALU+1][5],dst[FU_ALU+1][4]}));

//module instance 

   pipe_1_3 aludstwepipe_1 (
      .d( dst_we_s4[FU_ALU+1] & ~ctrl4_vf_wbsel[1] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:4] ),
      .squash( pipe_squash[6:4] ),
      .q({dst_we[FU_ALU+1][5],dst_we[FU_ALU+1][4]}));


//module instance 
    pipe_4_1 aludstmaskpipe_1 (
      .d( vmask[FU_ALU+1] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_aludstmaskpipe_NC),
      .q({dst_mask[FU_ALU+1][5],dst_mask[FU_ALU+1][4]}));

//module instance 
    pipe_10_1 faludstpipe_1 (
      .d( dst_s4[FU_FALU+1] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_faludstpipe_NC),
      .q({dst[FU_FALU+1][5],dst[FU_FALU+1][4]}));

//module instance 
    pipe_1_3 faludstwepipe_1 (
      .d( dst_we_s4[FU_FALU+1] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:4] ),
      .squash( pipe_squash[6:4] ),
      .q({dst_we[FU_FALU+1][5],dst_we[FU_FALU+1][4]}));



 //This code is just for assigning from signals connected
 //to the matmul, which are linear multi bit signals, to the
 //multi-dimensional signals.
 wire [`MATMUL_STAGES*REGIDWIDTH-1:0] dst_matmul;
 wire [`MATMUL_STAGES*4-1:0] dst_mask_matmul;



      assign dst[FU_MATMUL][0+4] = dst_matmul[REGIDWIDTH*(0+1)-1:REGIDWIDTH*0];
      assign dst_mask[FU_MATMUL][0+4] = dst_mask_matmul[4*(0+1)-1:4*0];



      assign dst[FU_MATMUL][1+4] = dst_matmul[REGIDWIDTH*(1+1)-1:REGIDWIDTH*1];
      assign dst_mask[FU_MATMUL][1+4] = dst_mask_matmul[4*(1+1)-1:4*1];



      assign dst[FU_MATMUL][2+4] = dst_matmul[REGIDWIDTH*(2+1)-1:REGIDWIDTH*2];
      assign dst_mask[FU_MATMUL][2+4] = dst_mask_matmul[4*(2+1)-1:4*2];



      assign dst[FU_MATMUL][3+4] = dst_matmul[REGIDWIDTH*(3+1)-1:REGIDWIDTH*3];
      assign dst_mask[FU_MATMUL][3+4] = dst_mask_matmul[4*(3+1)-1:4*3];



      assign dst[FU_MATMUL][4+4] = dst_matmul[REGIDWIDTH*(4+1)-1:REGIDWIDTH*4];
      assign dst_mask[FU_MATMUL][4+4] = dst_mask_matmul[4*(4+1)-1:4*4];



      assign dst[FU_MATMUL][5+4] = dst_matmul[REGIDWIDTH*(5+1)-1:REGIDWIDTH*5];
      assign dst_mask[FU_MATMUL][5+4] = dst_mask_matmul[4*(5+1)-1:4*5];



      assign dst[FU_MATMUL][6+4] = dst_matmul[REGIDWIDTH*(6+1)-1:REGIDWIDTH*6];
      assign dst_mask[FU_MATMUL][6+4] = dst_mask_matmul[4*(6+1)-1:4*6];



      assign dst[FU_MATMUL][7+4] = dst_matmul[REGIDWIDTH*(7+1)-1:REGIDWIDTH*7];
      assign dst_mask[FU_MATMUL][7+4] = dst_mask_matmul[4*(7+1)-1:4*7];



      assign dst[FU_MATMUL][8+4] = dst_matmul[REGIDWIDTH*(8+1)-1:REGIDWIDTH*8];
      assign dst_mask[FU_MATMUL][8+4] = dst_mask_matmul[4*(8+1)-1:4*8];



      assign dst[FU_MATMUL][9+4] = dst_matmul[REGIDWIDTH*(9+1)-1:REGIDWIDTH*9];
      assign dst_mask[FU_MATMUL][9+4] = dst_mask_matmul[4*(9+1)-1:4*9];



      assign dst[FU_MATMUL][10+4] = dst_matmul[REGIDWIDTH*(10+1)-1:REGIDWIDTH*10];
      assign dst_mask[FU_MATMUL][10+4] = dst_mask_matmul[4*(10+1)-1:4*10];



      assign dst[FU_MATMUL][11+4] = dst_matmul[REGIDWIDTH*(11+1)-1:REGIDWIDTH*11];
      assign dst_mask[FU_MATMUL][11+4] = dst_mask_matmul[4*(11+1)-1:4*11];



      assign dst[FU_MATMUL][12+4] = dst_matmul[REGIDWIDTH*(12+1)-1:REGIDWIDTH*12];
      assign dst_mask[FU_MATMUL][12+4] = dst_mask_matmul[4*(12+1)-1:4*12];



      assign dst[FU_MATMUL][13+4] = dst_matmul[REGIDWIDTH*(13+1)-1:REGIDWIDTH*13];
      assign dst_mask[FU_MATMUL][13+4] = dst_mask_matmul[4*(13+1)-1:4*13];



      assign dst[FU_MATMUL][14+4] = dst_matmul[REGIDWIDTH*(14+1)-1:REGIDWIDTH*14];
      assign dst_mask[FU_MATMUL][14+4] = dst_mask_matmul[4*(14+1)-1:4*14];



      assign dst[FU_MATMUL][15+4] = dst_matmul[REGIDWIDTH*(15+1)-1:REGIDWIDTH*15];
      assign dst_mask[FU_MATMUL][15+4] = dst_mask_matmul[4*(15+1)-1:4*15];



      assign dst[FU_MATMUL][16+4] = dst_matmul[REGIDWIDTH*(16+1)-1:REGIDWIDTH*16];
      assign dst_mask[FU_MATMUL][16+4] = dst_mask_matmul[4*(16+1)-1:4*16];



      assign dst[FU_MATMUL][17+4] = dst_matmul[REGIDWIDTH*(17+1)-1:REGIDWIDTH*17];
      assign dst_mask[FU_MATMUL][17+4] = dst_mask_matmul[4*(17+1)-1:4*17];



      assign dst[FU_MATMUL][18+4] = dst_matmul[REGIDWIDTH*(18+1)-1:REGIDWIDTH*18];
      assign dst_mask[FU_MATMUL][18+4] = dst_mask_matmul[4*(18+1)-1:4*18];



      assign dst[FU_MATMUL][19+4] = dst_matmul[REGIDWIDTH*(19+1)-1:REGIDWIDTH*19];
      assign dst_mask[FU_MATMUL][19+4] = dst_mask_matmul[4*(19+1)-1:4*19];



      assign dst[FU_MATMUL][20+4] = dst_matmul[REGIDWIDTH*(20+1)-1:REGIDWIDTH*20];
      assign dst_mask[FU_MATMUL][20+4] = dst_mask_matmul[4*(20+1)-1:4*20];



      assign dst[FU_MATMUL][21+4] = dst_matmul[REGIDWIDTH*(21+1)-1:REGIDWIDTH*21];
      assign dst_mask[FU_MATMUL][21+4] = dst_mask_matmul[4*(21+1)-1:4*21];



      assign dst[FU_MATMUL][22+4] = dst_matmul[REGIDWIDTH*(22+1)-1:REGIDWIDTH*22];
      assign dst_mask[FU_MATMUL][22+4] = dst_mask_matmul[4*(22+1)-1:4*22];



      assign dst[FU_MATMUL][23+4] = dst_matmul[REGIDWIDTH*(23+1)-1:REGIDWIDTH*23];
      assign dst_mask[FU_MATMUL][23+4] = dst_mask_matmul[4*(23+1)-1:4*23];



      assign dst[FU_MATMUL][24+4] = dst_matmul[REGIDWIDTH*(24+1)-1:REGIDWIDTH*24];
      assign dst_mask[FU_MATMUL][24+4] = dst_mask_matmul[4*(24+1)-1:4*24];



      assign dst[FU_MATMUL][25+4] = dst_matmul[REGIDWIDTH*(25+1)-1:REGIDWIDTH*25];
      assign dst_mask[FU_MATMUL][25+4] = dst_mask_matmul[4*(25+1)-1:4*25];



      assign dst[FU_MATMUL][26+4] = dst_matmul[REGIDWIDTH*(26+1)-1:REGIDWIDTH*26];
      assign dst_mask[FU_MATMUL][26+4] = dst_mask_matmul[4*(26+1)-1:4*26];



      assign dst[FU_MATMUL][27+4] = dst_matmul[REGIDWIDTH*(27+1)-1:REGIDWIDTH*27];
      assign dst_mask[FU_MATMUL][27+4] = dst_mask_matmul[4*(27+1)-1:4*27];



      assign dst[FU_MATMUL][28+4] = dst_matmul[REGIDWIDTH*(28+1)-1:REGIDWIDTH*28];
      assign dst_mask[FU_MATMUL][28+4] = dst_mask_matmul[4*(28+1)-1:4*28];



///////////////////////////
// Matmul unit
///////////////////////////

//module instance 

matmul_unit_10_33_4 u_matmul(
.clk(clk),
.resetn(resetn),
.activate(ctrl4_matmul_en),
.en(pipe_advance[`MAX_PIPE_STAGES-1:4]),
.squash(pipe_squash[`MAX_PIPE_STAGES-1:4]),
.stall(stall_matmul),
.a_data(vr_src1[FU_MATMUL][4*32-1:0]),
.b_data(vr_src2[FU_MATMUL][4*32-1:0]),
.validity_mask_a_rows(matmul_masks_in[1*`MAT_MUL_SIZE-1:0*`MAT_MUL_SIZE]),
.validity_mask_a_cols(matmul_masks_in[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE]),
.validity_mask_b_rows(matmul_masks_in[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE]),
.validity_mask_b_cols(matmul_masks_in[3*`MAT_MUL_SIZE-1:2*`MAT_MUL_SIZE]),
.c_data(matmul_out), 
.vmask(vmask[FU_MATMUL]),
.in_dst(dst_s4[FU_MATMUL]),
.in_dst_we(dst_we_s4[FU_MATMUL]),
.out_dst(dst_matmul),
.out_dst_we(dst_we[FU_MATMUL][`MAX_PIPE_STAGES-1:4]),
.out_dst_mask(dst_mask_matmul)
);

//module instance 

trp_unit_10 u_trp (
.clk(clk),
.resetn(resetn),
.en(ctrl4_trp_en),
.a(vr_src1[FU_TRP][4*32-1:0]),
.mode(),
.read(),
.busy(),
.valid(),
.out(trp_out)
);


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_10 bf_add_0(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFADDER][0 * 32 +: 32]),
    .b(vr_src2[FU_BFADDER][0 * 32 +: 32]),
    .out(bfadder_result_s5[0*32 +: 32])
    );
    
//module instance 
    bfloat_mult_10 bf_mult_0(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFMULT][0 * 32 +: 32]),
    .b(vr_src2[FU_BFMULT][0 * 32 +: 32]),
    .out(bfmult_result_s5[0*32 +: 32])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_10 inst_activation_0(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_ACT][0 * 32 +: 32]),
    .out(act_result_s5[0*32 +: 32])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_10 bf_add_1(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFADDER][1 * 32 +: 32]),
    .b(vr_src2[FU_BFADDER][1 * 32 +: 32]),
    .out(bfadder_result_s5[1*32 +: 32])
    );
    
//module instance 
    bfloat_mult_10 bf_mult_1(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFMULT][1 * 32 +: 32]),
    .b(vr_src2[FU_BFMULT][1 * 32 +: 32]),
    .out(bfmult_result_s5[1*32 +: 32])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_10 inst_activation_1(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_ACT][1 * 32 +: 32]),
    .out(act_result_s5[1*32 +: 32])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_10 bf_add_2(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFADDER][2 * 32 +: 32]),
    .b(vr_src2[FU_BFADDER][2 * 32 +: 32]),
    .out(bfadder_result_s5[2*32 +: 32])
    );
    
//module instance 
    bfloat_mult_10 bf_mult_2(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFMULT][2 * 32 +: 32]),
    .b(vr_src2[FU_BFMULT][2 * 32 +: 32]),
    .out(bfmult_result_s5[2*32 +: 32])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_10 inst_activation_2(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_ACT][2 * 32 +: 32]),
    .out(act_result_s5[2*32 +: 32])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_10 bf_add_3(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFADDER][3 * 32 +: 32]),
    .b(vr_src2[FU_BFADDER][3 * 32 +: 32]),
    .out(bfadder_result_s5[3*32 +: 32])
    );
    
//module instance 
    bfloat_mult_10 bf_mult_3(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFMULT][3 * 32 +: 32]),
    .b(vr_src2[FU_BFMULT][3 * 32 +: 32]),
    .out(bfmult_result_s5[3*32 +: 32])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_10 inst_activation_3(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_ACT][3 * 32 +: 32]),
    .out(act_result_s5[3*32 +: 32])
    );



/******************************************************************************/
/************************** WB Pipeline Stage ********************************/
/******************************************************************************/

    assign wb_dst[FU_ALU+1]=dst[FU_ALU+1][5];
    assign wb_dst_we[FU_ALU+1]=dst_we[FU_ALU+1][5] && ~pipe_squash[5];
    assign wb_dst_mask[FU_ALU+1]=dst_mask[FU_ALU+1][5];
    assign D_wb_last_subvector[FU_ALU+1]=D_last_subvector_s5[FU_ALU+1];

    assign D_wb_last_subvector[FU_FALU+1]=D_last_subvector_s5[FU_FALU+1];



  assign wb_dst[FU_MEM]=dst[FU_MEM][5];
  assign wb_dst_we[FU_MEM]=dst_we[FU_MEM][5] && (~pipe_advance[5]|~pipe_squash[5]);
  assign wb_dst_mask[FU_MEM]=load_result_mask_s5;
  assign D_wb_last_subvector[FU_MEM]=D_last_subvector_s5[FU_MEM];

  assign wb_dst[FU_MUL]=dst[FU_MUL][5];
  assign wb_dst_we[FU_MUL]=dst_we[FU_MUL][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_MUL]=dst_mask[FU_MUL][5];
  assign D_wb_last_subvector[FU_MUL]=D_last_subvector_s5[FU_MUL];


  assign wb_dst[FU_BFADDER] = dst[FU_BFADDER][5];
  assign wb_dst_we[FU_BFADDER] = dst_we[FU_BFADDER][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_BFADDER] = dst_mask[FU_BFADDER][5];
  assign D_wb_last_subvector[FU_BFADDER] = D_last_subvector_s5[FU_BFADDER];

  assign wb_dst[FU_BFMULT] = dst[FU_BFMULT][5];
  assign wb_dst_we[FU_BFMULT] = dst_we[FU_BFMULT][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_BFMULT] = dst_mask[FU_BFMULT][5];
  assign D_wb_last_subvector[FU_BFMULT] = D_last_subvector_s5[FU_BFMULT];

  assign wb_dst[FU_ACT] = dst[FU_ACT][5];
  assign wb_dst_we[FU_ACT] = dst_we[FU_ACT][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_ACT] = dst_mask[FU_ACT][5];
  assign D_wb_last_subvector[FU_ACT] = D_last_subvector_s5[FU_ACT];

  assign wb_dst[FU_MATMUL]=dst[FU_MATMUL][5];
  assign wb_dst_we[FU_MATMUL]=dst_we[FU_MATMUL][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_MATMUL]=dst_mask[FU_MATMUL][5];
  //TODO: There is no code that assigns to the s31 var used below. Need to add that code
  //This is only a debug var, so it doesn't affect functionality
  assign D_wb_last_subvector[FU_MATMUL]=D_last_subvector_s31[FU_MATMUL];

  // ******************  Map functional units to banks ******************
  always@*
    for (bw=0; bw<2; bw=bw+1)
    begin
      vr_c_we[bw]=(wb_dst_we[FU_MUL] && `LO(wb_dst[FU_MUL],1)==bw) ||
                  (wb_dst_we[FU_MATMUL] && `LO(wb_dst[FU_MATMUL],1)==bw) ||
                  (wb_dst_we[FU_BFADDER] && `LO(wb_dst[FU_BFADDER],1)==bw) ||
                  (wb_dst_we[FU_BFMULT] && `LO(wb_dst[FU_BFMULT],1)==bw) ||
                  (wb_dst_we[FU_ACT] && `LO(wb_dst[FU_ACT],1)==bw) ||
                  (wb_dst_we[FU_MEM] && `LO(wb_dst[FU_MEM],1)==bw) ||
                  (wb_dst_we[FU_ALU] && `LO(wb_dst[FU_ALU],1)==bw && 1==0) ||
                  (wb_dst_we[FU_ALU+bw] && 1!=0);

      //TODO: Update this code for matmul. This is for debug only, so skipping it for now.
      //Tells test_bench when to record register file contents.
      //Record if instruction writes to VRF, on last subvector, and not stalled
      D_wb_instrdone[bw] = pipe_advance[5] && (
        ((1==0) ?
          (dst_we[FU_ALU][5] && D_wb_last_subvector[FU_ALU] && `LO(wb_dst[FU_ALU],1)==bw) :
          (dst_we[FU_ALU+bw][5] && D_wb_last_subvector[FU_ALU+bw])) || 
        (dst_we[FU_MUL][5] && D_wb_last_subvector[FU_MUL] && `LO(wb_dst[FU_MUL],1)==bw) || 
        (dst_we[FU_MEM][5] && D_wb_last_subvector[FU_MEM] && `LO(wb_dst[FU_MEM],1)==bw));

      //Take matmul output
      if (wb_dst_we[FU_MATMUL] && `LO(wb_dst[FU_MATMUL],1)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_MATMUL];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_MATMUL]>>1;
        vr_c_reg[bw]= wb_dst[FU_MATMUL];
        vr_c_writedatain[bw]= matmul_out;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MATMUL];
      end      
      else if (wb_dst_we[FU_TRP] && `LO(wb_dst[FU_TRP],1)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_TRP];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_TRP]>>1;
        vr_c_reg[bw]= wb_dst[FU_TRP];
        vr_c_writedatain[bw]= bfadder_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_TRP];
      end
      //Take bfadder output
      else if (wb_dst_we[FU_BFADDER] && `LO(wb_dst[FU_BFADDER],1)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_BFADDER];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_BFADDER]>>1;
        vr_c_reg[bw]= wb_dst[FU_BFADDER];
        vr_c_writedatain[bw]= bfadder_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_BFADDER];
      end
      else if (wb_dst_we[FU_BFMULT] && `LO(wb_dst[FU_BFMULT],1)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_BFMULT];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_BFMULT]>>1;
        vr_c_reg[bw]= wb_dst[FU_BFMULT];
        vr_c_writedatain[bw]= bfmult_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_BFMULT];
      end
      else if (wb_dst_we[FU_ACT] && `LO(wb_dst[FU_ACT],1)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_ACT];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_ACT]>>1;
        vr_c_reg[bw]= wb_dst[FU_ACT];
        vr_c_writedatain[bw]= act_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_ACT];
      end
      //Take multiplier output
      else if (wb_dst_we[FU_MUL] && `LO(wb_dst[FU_MUL],1)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_MUL];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_MUL]>>1;
        vr_c_reg[bw]= wb_dst[FU_MUL];
        vr_c_writedatain[bw]= mulshift_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MUL];
      end
      //Take Memory unit output
      else if (wb_dst_we[FU_MEM] && `LO(wb_dst[FU_MEM],1)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_MEM];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_MEM]>>1;
        vr_c_reg[bw]= wb_dst[FU_MEM];
        vr_c_writedatain[bw]= load_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MEM];
      end
      else
      //Take ALU output
      begin
        vmask_final[bw]=wb_dst_mask[FU_ALU+bw*1];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_ALU+bw*1]>>1;
        vr_c_reg[bw]= wb_dst[FU_ALU+bw*1];
        vr_c_writedatain[bw]= alu_result_s5[bw*1];
        //Do ALU and FALU for last subvector
        D_last_subvector_done[bw]=
          (D_wb_last_subvector[FU_ALU+bw*1] && `LO(wb_dst[FU_ALU+bw*1],1)==bw) |
          (D_wb_last_subvector[FU_FALU+bw*1] && `LO(dst[FU_FALU+bw*1][5],1)==bw);
      end


      //Generate byte enable from mask
      for (j=0; j<4; j=j+1)
        vr_c_byteen[bw*4*4 + j*4 +: 4 ]={4{vmask_final[bw][j]}};

      //*********** Flag writeback ***********
      vf_c_reg[bw*BANKREGIDWIDTH+:BANKREGIDWIDTH]=dst[FU_FALU+bw*1][5]>>1;
      vf_c_writedatain[bw*4+:4]=flagalu_result_s5[bw*1];
      vf_c_we[bw]=dst_we[FU_FALU+bw*1][5] && `LO(dst[FU_FALU+bw*1][5],1)==bw;
    end

  //********** Scalar writeback ***********
  assign vs_wetrack={ctrl_vs_we[5:4],|ctrl3_vs_we,ctrl2_vs_we};
  assign vs_we=ctrl_vs_we[5] & load_result_mask_s5[0];
  assign vs_dst=wb_dst[FU_MEM][`VRID_RANGE];
  assign vs_writedata=load_result_s5[32-1:0];

endmodule


 
// THIS UNIT SHOULD BE REDESIGNED!!!!!
// It started off simple with low performance and after adding a bunch of hacks
// to make it perform better it's gotten messy.  A new memory unit should
// special case strided loads and service them early and pipelined, and it
// should have a store buffer with variable width for writes.

/************************
 *
 * VPUWIDTH must equal CONTROLWIDTH must equal 32 
 * NUMPARALLELLANES must be a divisor of NUMLANES
 *
 * op[6] - memop 1-memory operation, 0-shift operation
 *
 * op[5] op[4]
 *   0     0    UNIT
 *   0     1    STRIDE
 *   1     0    INDEX
 *
 * size1 size0 signd ld/st
 * op[3] op[2] op[1] op[0]
 *   0     0     0     0    VLD.u.b
 *   0     1     0     0    VLD.u.h
 *   1     0     0     0    VLD.u.w
 *   0     0     1     0    VLD.b
 *   0     1     1     0    VLD.h
 *   1     0     1     0    VLD.w
 *   0     0     X     1    VST.b
 *   0     1     X     1    VST.h
 *   1     0     X     1    VST.w
 *
 *
 * Ported to handle element shifting as well:
 *
 * Data shifts
 * ===========
 * vins -   L shift vindex
 * vext -     R shift vindex
 * vcompress -  R
 * vexpand -  L 
 * vfins -  L vindex
 * vhalf -        R shift vl/2
 * vexthalf -     R shift vl/2
 * vhalfup -  R shift 2^vindex
 * vhalfdn -  L shift 2^vindex
 *
 * Mask shifting - much harder
 * =============
 *
 * Quartus 5.0: 919 LEs, 137 MHz (CritPath: vip_r to vreaddata)
 *************************/

module vmem_unit_32_4_2_2_1_32_128_7_128_7_5_10 (
    clk,
    resetn,

    enable,       //Is this unit on? (is it less than VL, does the instr use it)
    en, //Is pipeline advancing
    squash, //Is pipeline squashing us
    op,
    stall,
    last_subvector,

    // Control ports
    cbase,
    cstride,
    cprefetch,

    // Vector ports
    vmask,
    vstrideoffset,  //pre-computed stride offsets for each lane (stride*laneid)
    vindex,
    vwritedata,
    voutput,
    voutput_we,

    // Writeback ports
    in_dst,
    in_dst_we,
    out_dst,
    out_dst_we,
    in_vs_dst_we,
    out_vs_dst_we,

    // Vector operations ports
    sa,
    dir_left,

    // Data memory interface
    dmem_en,
    dmem_address,
    dmem_we,
    dmem_byteen,
    dmem_writedata,
    dmem_readdata,
    dmem_cachematch,
    dmem_cachemiss,
    dmem_prefetch,
    dmem_wait
    );

parameter       MUNIT_IDLE=0,
                MUNIT_ISSUE=1,
                MUNIT_DONE=2,
                MUNIT_SHIFT=3, 
                MUNIT_XTRAWRITE=4; 
  
parameter ISSUE_IDLE=3'b000,
          ISSUE_INITIAL_INCREMENT=3'b001,
          ISSUE_INCREMENTED=3'b010,
          ISSUE_WAITONE=3'b011,
          ISSUE_WAITING=3'b100;

input clk;
input resetn;

input  enable;
input  [2:0] en; //Is pipeline one
input  [2:0] squash;
input [6:0] op;
output stall;
input last_subvector;

// Control ports
input [ 32-1 : 0 ] cbase;
input [ 32-1 : 0 ] cstride;
input [ 32-1 : 0 ] cprefetch;

// Vector ports
input  [          4-1 : 0 ]  vmask;
input  [ 4*32-1 : 0 ]  vstrideoffset;
input  [ 4*32-1 : 0 ]  vindex;
input  [ 4*32-1 : 0 ]  vwritedata;
output [ 4*32-1 : 0 ]  voutput;
output [          4-1 : 0 ]  voutput_we;

input   [10-1:0]  in_dst;
input                     in_dst_we;
output   [10-1:0] out_dst;
output                    out_dst_we;
input                     in_vs_dst_we;
output                    out_vs_dst_we;

input  [      2-1 : 0 ]  sa;
input                               dir_left;

// Data memory interface
output dmem_en;
output dmem_we;
output  [ 31 : 0 ]                  dmem_address;
output  [ 128/8-1 : 0 ] dmem_byteen;
output  [ 128-1 : 0 ]   dmem_writedata;
input   [ 128-1 : 0 ]    dmem_readdata;
input                               dmem_cachematch;
input                               dmem_cachemiss;
output  [ 31 : 0 ]                  dmem_prefetch;
input                               dmem_wait;

reg     [ 31 : 0 ]                  dmem_address;
reg     [ 128/8-1 : 0 ] dmem_byteen;
reg     [ 128-1 : 0 ]   dmem_writedata;
reg     [ 31 : 0 ]                  dmem_prefetch;

wire  [1:0]  op_pattern;      // 0-Unit, 1-Strided, 2-Indexed
wire  [1:0]  op_size;         // 0-byte, 1-16bits, 2-32bits, 3-64bits
wire         op_signed;       // 0-unsigned, 1-signed
wire         op_we;         // 0-load, 1-store
wire         op_memop;         // 1-memory op, 0-vector shift op

wire  [ 2*32-1 : 0 ]  __vreaddata;
reg           [ 4*32-1 : 0 ]  address;
reg              [ 4*32-1:0]  vreaddata;
reg                    [ 4-1 : 0 ]  vreaddata_we;
reg   [ 2*(7-5)-1 : 0 ] crossbar_sel;
wire  [ 2*32-1 : 0 ] crossbar;
wire  [ 2*32-1 : 0 ]  _vwritedata;
wire   [ 2*4-1 : 0 ]  _vbyteen;

reg  [ 32-1 : 0 ] stride;
wire [ 32-1 : 0 ] stride_tmp;
reg  [ 32-1 : 0 ] prefetch;
wire [ 32-1 : 0 ] t_cprefetch;

wire           [         4-1 : 0 ]  vshifted_mask;
wire           [         4-1 : 0 ]  vshifted_masksave;
wire          [ 4*32-1 : 0 ]  vshifted_writedata;
wire          [ 4*32-1 : 0 ]  vshifted_address;


reg                           dmem_valid;
reg   [ 31 : 0 ]              dmem_readdata_address;
reg                           cachedata_stillvalid;

reg [2:0] munit_state;
reg [2:0] issue_state;

reg  [ 2-1 : 0 ] vpid;
reg  [ 4-1 : 0 ] done;
wire doneall;

wire  [         4-1 : 0 ]  _vreaddata_we;
reg   [ 2-1 : 0 ]  _parhits;
wire  [ 2-1 : 0 ]  parhits;
reg   [ 2-1 : 0 ]  parhits_done;
wire  parhits_doneall;
wire  parhits_all;
wire  parhits_none;
wire  parhits_some;

wire  shifter_dirleft;
wire  shifter_load;
wire  shifter_shift;
wire  shifter_jump;
wire  shifter_jump_s1;

wire addr_sneakyload;
wire addr_advance;
wire addr_rewind;

wire do_quick_load;
reg quick_loaded;

reg  [ 2-1 : 0 ] sa_count;
wire [ 2-1 : 0 ] next_sa_count;
wire                        next_sa_count_zero;
wire                        sa_zero;
reg  [ 4*32-1 : 0 ]  vshiftresult;

genvar i;
reg [31:0] j;
genvar k;
reg [31:0] l;
reg [31:0] m;
reg [31:0] n;
reg [31:0] p;

  assign {op_memop,op_pattern,op_size,op_signed,op_we}=op;

  assign stall= (munit_state!=MUNIT_IDLE && munit_state!=MUNIT_DONE);

/************************** Pipeline load stage *******************************/
  reg enable_s2;
  reg last_subvector_s2;
  reg [ 10-1 : 0 ] in_dst_s2;
  reg [      2-1 : 0 ] sa_s2;
  reg dir_left_s2;
  reg  [1:0]  op_pattern_s2;      // 0-Unit, 1-Strided, 2-Indexed
  reg  [1:0]  op_size_s2;         // 0-byte, 1-16bits, 2-32bits, 3-64bits
  reg         op_signed_s2;       // 0-unsigned, 1-signed
  reg         op_we_s2;         // 0-load, 1-store
  reg         op_memop_s2;         // 1-memory op, 0-vector shift op

  always@(posedge clk)
    if (!resetn)
    begin
      enable_s2<=0;
      last_subvector_s2<=0;
      in_dst_s2<=0;
      sa_s2<=0;
      dir_left_s2<=0;
      op_memop_s2<=0;
      op_pattern_s2<=0;
      op_size_s2<=0;
      op_signed_s2<=0;
      op_we_s2<=0;
    end
    else if (!stall)
    begin
      enable_s2<=enable;
      last_subvector_s2<=last_subvector;
      if (en[0])
        in_dst_s2<=in_dst;
      sa_s2<=sa;
      dir_left_s2<=dir_left;
      //{op_memop_s2,op_pattern_s2,op_size_s2,op_signed_s2,op_we_s2}<=op;
      op_memop_s2<=op[6];
      op_pattern_s2<=op[5:4];
      op_size_s2<=op[3:2];
      op_signed_s2<=op[1];
      op_we_s2<=op[0];
    end

/*************************** Vector op logic *********************************/

  assign next_sa_count=sa_count-1;
  assign next_sa_count_zero=(next_sa_count==0);
  assign sa_zero=(sa==0);

  always@(posedge clk)
  begin
    if (!resetn)
      sa_count<=0;
    else if (shifter_load)
      sa_count<=sa;
    else if (shifter_shift)
      sa_count<=next_sa_count;
  end

  wire doublewrite;
  // Detect double write one clock cycle early - also support 1 lane
  assign doublewrite=(~op_memop_s2) && (4>1) && ( (dir_left_s2) ? 
              (|vshifted_mask[(4)-2:0]) && //support L=1
                (vshifted_mask[4-1]&(|vshifted_masksave)) :
              (|vshifted_mask[4-1:1]) && //support L=1
                (vshifted_mask[0]&(|vshifted_masksave)));

  assign out_dst = {in_dst_s2[10-1:5],
                    (munit_state!=MUNIT_XTRAWRITE) ? in_dst_s2[5-1:0] :
                     (dir_left_s2) ? in_dst_s2[5-1:0]+1'b1 : 
                       in_dst_s2[5-1:0]-1'b1};

  //Truncate shifted result to VPW's size
  always@*
    for (p=0; p<4; p=p+1)
      vshiftresult[p*32 +: 32]=
              vshifted_address[p*32 +: 32];

  assign voutput= (~enable_s2) ? 0 :
                  (op_memop_s2) ? vreaddata : 
                               vshiftresult;
 

  assign voutput_we=(op_memop_s2) ? vreaddata_we : 
                      (munit_state==MUNIT_XTRAWRITE) ? vshifted_masksave : 
                        (munit_state==MUNIT_DONE) ? vshifted_mask : 0;

/*************************** ISSUING LOGIC *********************************/

  assign shifter_load=enable && ~quick_loaded &&
                        (munit_state==MUNIT_IDLE || 
                         munit_state==MUNIT_DONE ||
                         do_quick_load);

  assign shifter_shift= (munit_state==MUNIT_SHIFT) || 
               ((munit_state==MUNIT_ISSUE) && parhits_doneall);

  assign shifter_dirleft=(op_memop_s2) ? 1'b0 : dir_left_s2;
  assign shifter_jump=op_memop_s2 && ~op_pattern_s2[1];
  assign shifter_jump_s1=op_memop && ~op_pattern[1];

  // Address Generator for each lane, or store vector to be shifted
  always@*
    for (m=0; m<4; m=m+1)
      address[m*32 +: 32] = ((op_memop) ? cbase : 0) + 
            ( (op_pattern[1] || ~op_memop)  ? vindex[m*32 +: 32] : 
               (vstrideoffset[m*32 +: 32]<<op_size));

  wire [4-1:0] vwritedata_shifter_squash_NC;
  velmshifter_jump_32_4_2 vwritedatashifter(
      .clk(clk),
      .resetn(resetn),    //Don't use if not a store
      .load(shifter_load && op_we),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .jump(shifter_jump),
      .squash(vwritedata_shifter_squash_NC),
      .shiftin_left(0),
      .shiftin_right(0),
      .inpipe(vwritedata),
      .outpipe(vshifted_writedata));

  wire [4-1:0] vaddress_shifter_squash_NC;
  velmshifter_jump_32_4_2 vaddressshifter(
      .clk(clk),
      .resetn(resetn), //consider forcing to 0 if not used
      .load(shifter_load),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .jump(shifter_jump),
      .squash(vaddress_shifter_squash_NC),
      .shiftin_left(0),
      .shiftin_right(0),
      .inpipe( address),
      .outpipe(vshifted_address));

  velmshifter_jump_1_4_2 vmaskshifter(
      .clk(clk),
      .resetn(resetn),
      .load(shifter_load),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .jump(shifter_jump),
      .squash(0),
      .shiftin_left( 1'b0),
      .shiftin_right( 1'b0 ),
      .inpipe(vmask),
      .outpipe(vshifted_mask));

  //Save spilled over masks for shifting instructions (can only shift by 1)
  wire    [4-1 : 0]  vmasks_save_inpipe_NC;
  velmshifter_4_1  vmaskssave(
      .clk(clk),
      .resetn(resetn && ~shifter_load),
      .load(1'b0),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .squash(0),
      .shiftin_left( vshifted_mask[0] ),
      .shiftin_right( vshifted_mask[4-1] ),
      .inpipe(vmasks_save_inpipe_NC),
      .outpipe(vshifted_masksave));

  //Stride is maximum of stride specified and cache line size 
  assign stride_tmp = (((op_pattern[0]) ? cstride : 1) << op_size);
  always@(posedge clk)
    if (!resetn || last_subvector_s2&~stall&~shifter_load&~quick_loaded )
      stride<=0;
    else if (shifter_load)
      stride<= (stride_tmp>128/8) ? 
                      stride_tmp : 128/8;

  wire [15:0] constantprefetch;
  assign constantprefetch=`VECTORPREFETCHES;
  assign t_cprefetch=cprefetch[15:0]<<(65535-`VECTORPREFETCHES);

  //***********************  Prefetch Logic **************************
  //
  //Send stride and vector length to bus.  To do constant prefetching set
  //stride to cache line size and send constant as length
  //******************************************************************
  wire [15:0] temp_prefetch;
  assign temp_prefetch = 16'd0+2**(7-3);
  always@(posedge clk)
    if (!resetn || last_subvector_s2&~stall&~shifter_load&~quick_loaded )
      prefetch<=0;
    else if (shifter_load)
      if (`VECTORPREFETCHES >= 65530)
        prefetch<= { stride_tmp[15:0], t_cprefetch[15:0] };
      else if (`VECTORPREFETCHES == 0)
        prefetch<= 0;
      else
        prefetch<= { temp_prefetch,constantprefetch };

  // State machine for issuing addresses - this is really sneaky!!!!!
  // The cache takes 1 cycle to respond to a cache request and is generally
  // not pipelined - but here we treat it as if it is pipeline by changing
  // the address during a single cache access.  
  //
  // We speculatively increment to the next cache line before we know if the
  // previous cache request hit or missed.  If it hits then we're fine and
  // we get pipelined outputs from the cache, but if not, we have to rewind 
  // our increment and wait for the data to come in.
  //
  // NOTE: We only do this for loads - writes write on the second clock
  always@(posedge clk)
  begin
    if (!resetn)
      issue_state<=ISSUE_IDLE;
    else
      case(issue_state)
        ISSUE_IDLE:
          issue_state<= (addr_sneakyload) ? 
                          ISSUE_INITIAL_INCREMENT: MUNIT_IDLE;
        ISSUE_INITIAL_INCREMENT:
          issue_state<= (doneall && ~addr_sneakyload) ? ISSUE_IDLE :
                      (doneall && addr_sneakyload) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_doneall && !doneall) ? ISSUE_INITIAL_INCREMENT :
                                            ISSUE_INCREMENTED;
        ISSUE_INCREMENTED:
          issue_state<= (doneall && ~addr_sneakyload) ? ISSUE_IDLE :
                      (doneall && addr_sneakyload) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_doneall) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_none) ? ISSUE_WAITONE : ISSUE_INCREMENTED;
        // This stage ignores the speculated result following a miss
        ISSUE_WAITONE:      
          issue_state<= ISSUE_WAITING;
        ISSUE_WAITING:
          issue_state<= (doneall && ~addr_sneakyload) ? ISSUE_IDLE :
                      (doneall && addr_sneakyload) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_doneall) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_some) ? ISSUE_INITIAL_INCREMENT : ISSUE_WAITING;
      endcase
  end

  assign addr_sneakyload=(shifter_load && op_memop && ~op_we && ~op_pattern[1] && enable);
                  //(shifter_shift && ~shifter_load && ~op_we_s2 && enable_s2));

  assign addr_advance=(issue_state==ISSUE_INITIAL_INCREMENT) ||
                      (issue_state==ISSUE_INCREMENTED && parhits_some) ||
                      (issue_state==ISSUE_WAITING && parhits_some) ||
                      (op_we_s2 && parhits_some && ~cachedata_stillvalid);

  assign addr_rewind=(issue_state==ISSUE_INCREMENTED && parhits_none);

  always@(posedge clk)
    if (!resetn || shifter_load || last_subvector_s2&~stall&~shifter_load )
      vpid<=0;
    else if (shifter_shift)
      vpid<=vpid + ((shifter_jump) ? 2 : 1'b1);

  // Loads in a new request if it is a memory request. quick_loaded is high 
  // the cycle after the new request has been loaded and instructs the
  // logic below to continue requesting from memory (even though munit_state
  // will be in the DONE state).
  // Note we don't do quickload on last subvector - a speculative miss can 
  // cause the quickloaded memory instruction to re-load its cache line from
  // memory - but if that line was dirty we would have missed the changes.
    // I don't think this is true anymore PY 07/2008
  assign do_quick_load=doneall&enable_s2&op_memop_s2&~op_we_s2&
                       ~last_subvector_s2&enable&op_memop&~op_we&~op_pattern[1];
  always@(posedge clk)
    quick_loaded<=shifter_load&do_quick_load;

  assign dmem_en = ((shifter_jump) ? (|vshifted_mask[2-1:0]) : 
                                      vshifted_mask[0]) && 
                    (munit_state==MUNIT_ISSUE || quick_loaded);
  assign dmem_we=op_we_s2;

  /*********************
  * We don't want cache miss signal to be an add/sub select signal or else it
  * will propagate through the adder.  We perform the addition/subtraction
  * in parallel with the miss calculation and use the miss as a mux select
  *********************/
  wire [ 32-1 : 0 ] dmem_address_next /* synthesis keep */;
  wire [ 32-1 : 0 ] dmem_address_prev /* synthesis keep */;
  assign dmem_address_next=dmem_address + stride;
  assign dmem_address_prev=dmem_address - stride;

  always@(posedge clk)
    if (!resetn)
      dmem_address<=0;
    else if (shifter_load)
      dmem_address<=address[32-1:0];
    else if (shifter_shift)

      dmem_address<= (!shifter_jump) ? 
              vshifted_address[
                2*32-1 : 32] :
              vshifted_address[
                 (2)*32+32-1 : (2)*32];
                   
    //Fetch next cache line if partial match on initial cache access
    //else if (parhits_some && ~cachedata_stillvalid)
    else if (addr_advance)
      dmem_address<=dmem_address_next;
    else if (addr_rewind)
      dmem_address<=dmem_address_prev;

  always@(posedge clk)
    if (!resetn)
      dmem_prefetch<=0;
    //else if (shifter_load)
    else
      dmem_prefetch<=prefetch[32-1:0];

/*************************** Mem Write LOGIC ********************************/

// Generate byte/halfword alignment circuitry for each word                  

       vstore_data_translator vstore_data_translator_0(
         //Pad vshifted_writedata with zeros incase signal is less than 32-bits
      .write_data({32'b0,vshifted_writedata[0*32+32-1 : 0*32]}),
      .d_address(vshifted_address[0*32+2-1 : 0*32]),
      .store_size(op_size_s2), 
      .d_byteena(_vbyteen[4*0+4-1 : 4*0]),  
      .d_writedataout(_vwritedata[0*32+32-1 : 0*32]));

        


       vstore_data_translator vstore_data_translator_1(
         //Pad vshifted_writedata with zeros incase signal is less than 32-bits
      .write_data({32'b0,vshifted_writedata[1*32+32-1 : 1*32]}),
      .d_address(vshifted_address[1*32+2-1 : 1*32]),
      .store_size(op_size_s2), 
      .d_byteena(_vbyteen[4*1+4-1 : 4*1]),  
      .d_writedataout(_vwritedata[1*32+32-1 : 1*32]));

        


  always@*
  begin
    dmem_writedata=0;
    dmem_byteen=0;
    for (l=0; l<2; l=l+1)
      if (dmem_address[31:7-3] == 
          vshifted_address[32*l+7-3 +: 
                            32-7+3])
      begin
        dmem_writedata=dmem_writedata| (_vwritedata[l*32 +: 32] << 
            {vshifted_address[32*l+2 +: 7-5], {5{1'b0}}});
        if (vshifted_mask[l] && (shifter_jump || (l==0)))
          dmem_byteen=dmem_byteen | (_vbyteen[4*l+:4]<<
            {vshifted_address[32*l+2 +: 7-5], {2{1'b0}}});
      end
  end

/*************************** Mem Receive LOGIC ********************************/
  reg wastedcache;  //Identifies when a cache request is made but then aborted because the data was found in the previous cache request which is "stillvalid"

  // This is specific to cache timing - 1 cycle
  always@(posedge clk)
    if (!resetn) 
    begin
      dmem_valid<=0;
      dmem_readdata_address<=0;
      wastedcache<=0;
      cachedata_stillvalid<=0;
    end
    else
    begin
      dmem_valid<=dmem_en;
      if (!wastedcache)
        dmem_readdata_address<=dmem_address;
      //WE know for sure data is still valid after the cache hit since it takes
      //two clock cyles to get new cacheline
      //wastedcache<=shifter_shift&cachedata_stillvalid;
      //cachedata_stillvalid<=dmem_valid&~dmem_wait;
      //Disable this since now we speculatively advance the address
      wastedcache<=0;
      cachedata_stillvalid<=0;
    end

  // Find out which parallel lanes hit in the cache
  always@*
    for (j=0; j<2; j=j+1)
      //This signal tells us a cache request succeeded (for either load/store)
      //Each bit corresponds to a parallel lane
      _parhits[j]= vshifted_mask[j] &&
          ((dmem_valid&(dmem_cachematch&~op_we_s2 || op_we_s2&~dmem_wait)) || 
              (cachedata_stillvalid&~op_we_s2)) &&
            (vshifted_address[j*32+7-3 +: 
                             32-7+3] ==
               dmem_readdata_address[31:7-3]);

  //For operations that don't jump, just look at first bit of _parhits
  assign parhits=(shifter_jump) ? _parhits : {2{_parhits[0]}};

  // Detect all parallel lanes hitting
  assign parhits_all=&(parhits|~vshifted_mask[2-1:0]);
  // Detect cache misses
  assign parhits_none=~|(parhits);
  // Detect some misses - fetch next cache line
  assign parhits_some=~parhits_none && ~parhits_all;

  //If 4<=2 then we will never do a jump, so we make
  //compiler ignore this statement if that's the case
  always@(posedge clk)
    if (!resetn || shifter_load)
      parhits_done<= (shifter_jump_s1) ?  ~vmask : {2{~vmask[0]}};
    else if ( parhits_doneall )
    
      parhits_done<= (shifter_jump && (4>2)) ? 
                    ~vshifted_mask[ 
                        2*(2)-1 : 2] :
                    {2{(4>1) ? ~vshifted_mask[1] : 1'b0}};
                    
    else           
      parhits_done<= parhits_done|parhits;

  assign parhits_doneall=&(parhits|parhits_done);

  assign _vreaddata_we= ((shifter_jump) ? _parhits : _parhits[0]) 
                          << vpid[2-1:0];

  always@(posedge clk)
    if (!resetn || ~enable_s2)  // Force to zero if unit not used
      vreaddata_we<= 0 ;
    else           // write to regfile if a) is a load, b) we haven't already
      vreaddata_we<= {4{~op_we_s2}} & _vreaddata_we & ~done;

  //Select signal for crossbar either uses bits from corresponding parallel
  //lane address or all selects are set to the same if we're not jumping
  always@*
    for (n=0; n<2; n=n+1)
      crossbar_sel[(7-5)*n +: (7-5)]=
            (!shifter_jump) ? 
              {2{vshifted_address[2 +: 7-5]}} :
              vshifted_address[n*32+2 +: (7-5)];

  vmem_crossbar_128_7_2_32_5 vmem_crossbar(
      .clk(clk), .resetn(resetn),
      .sel(crossbar_sel),
      .in(dmem_readdata),
      .out(crossbar));

                // Generate byte/halfword alignment circuitry for each word                   

      vload_data_translator load_data_translator0(
      .d_readdatain(crossbar[32*(0+1)-1:32*0]),
      .d_address( (shifter_jump) ? vshifted_address[32*0+2-1 : 32*0] :
                                   vshifted_address[1:0]),
      .load_size(op_size_s2[1:0]),
      .load_sign_ext(op_signed_s2),
      .d_loadresult(__vreaddata[32*(0+1)-1:32*0])
      );

        


      vload_data_translator load_data_translator1(
      .d_readdatain(crossbar[32*(1+1)-1:32*1]),
      .d_address( (shifter_jump) ? vshifted_address[32*1+2-1 : 32*1] :
                                   vshifted_address[1:0]),
      .load_size(op_size_s2[1:0]),
      .load_sign_ext(op_signed_s2),
      .d_loadresult(__vreaddata[32*(1+1)-1:32*1])
      );

        


always@(posedge clk)
    //zero if unit not used
    if (!resetn || ~enable_s2 || (~stall&~enable) || op_we_s2)
      vreaddata<= 0 ;
    else                // Don't write to regfile unless this is a load!
      vreaddata<= {4/2{__vreaddata}};

/*************************** DONE LOGIC *********************************/

  // Done register has 1 bit for each lane, we are finished when done is all 1's
  always@(posedge clk)
    if (!resetn)
      done<=0;
    //Don't check for MUNIT_DONE, because of quickload we are reading then
    //else if (shifter_load || (munit_state==MUNIT_DONE) )
    else if (shifter_load || doneall || (munit_state==MUNIT_DONE && (op_we_s2 || op_pattern_s2[1])))
      done<=~vmask;
    else 
      done<=done|_vreaddata_we;

  assign doneall=(&(done|_vreaddata_we));

  always@(posedge clk)
  begin
    if (!resetn)
      munit_state<=MUNIT_IDLE;
    else
      case(munit_state)
        MUNIT_IDLE:
          munit_state<= (enable) ? (op_memop) ? MUNIT_ISSUE : 
                                      (sa_zero) ? MUNIT_DONE : MUNIT_SHIFT : 
                                   MUNIT_IDLE;
        MUNIT_ISSUE:
          munit_state<= doneall ? MUNIT_DONE : MUNIT_ISSUE;
        MUNIT_SHIFT:
          munit_state<= (next_sa_count_zero) ? 
                            (doublewrite) ? MUNIT_XTRAWRITE : MUNIT_DONE : 
                            MUNIT_SHIFT;
        MUNIT_XTRAWRITE:  //Spilled over subvector bounds when shifting 
          munit_state<= MUNIT_DONE;
        MUNIT_DONE:
          munit_state<= (enable) ? (op_memop) ? MUNIT_ISSUE : 
                                      (sa_zero) ? MUNIT_DONE : MUNIT_SHIFT : 
                                   MUNIT_IDLE;
        default:
          munit_state<=MUNIT_IDLE;
      endcase
  end

  pipereg_1  dstwepipe (
    .d( in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .squashn(1'b1),
    .en( en[0] && ~stall ),
    .q(out_dst_we));

  pipereg_1 vsdstwepipe (
    .d( in_vs_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .squashn(1'b1),
    .en( en[0] && ~stall ),
    .q(out_vs_dst_we));

endmodule




/****************************************************************************
 *           Load data translator
 *- moves read data to appropriate byte/halfword and zero/sign extends
 *****************************************************************************/
module vstore_data_translator(
    write_data,             // data in least significant position
    d_address,
    store_size,             // 0-byte, 1-16bits, 2-32bits, 3-64bits
    d_byteena,
    d_writedataout);        // shifted data to coincide with address
parameter WIDTH=32;

input [WIDTH-1:0] write_data;
input [1:0] d_address;
input [1:0] store_size;
output [3:0] d_byteena;
output [WIDTH-1:0] d_writedataout;

reg [3:0] d_byteena;
reg [WIDTH-1:0] d_writedataout;

always @*
begin
    case (store_size)
        2'b00:
        begin
            case(d_address[1:0])
                2'b00: begin d_byteena=4'b1000; 
                        d_writedataout={write_data[7:0],24'b0}; end
                2'b01: begin d_byteena=4'b0100;
                        d_writedataout={8'b0,write_data[7:0],16'b0}; end
                2'b10: begin d_byteena=4'b0010;
                        d_writedataout={16'b0,write_data[7:0],8'b0}; end
                default: begin d_byteena=4'b0001;
                        d_writedataout={24'b0,write_data[7:0]}; end
            endcase
        end
        2'b01:
        begin
            d_writedataout=(d_address[1]) ? {16'b0,write_data[15:0]} : 
                                            {write_data[15:0],16'b0};
            d_byteena=(d_address[1]) ? 4'b0011 : 4'b1100 ;
        end
        default:
        begin
            d_byteena=4'b1111;
            d_writedataout=write_data;
        end
    endcase
end

endmodule

                  

module vdispatcher_shift_2_1 (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    squash,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ 2-1:0 ]  squash;

input [ 1-1:0 ] inshift_data;

input [ 2*1-1:0 ]  inparallel_data;
output [ 2*1-1:0 ] outparallel_data;

wire [ 1-1:0 ]  shiftin_left;
wire [ 1-1:0 ]  shiftin_right;


  assign shiftin_right = (!rotate) ? inshift_data : 
                      outparallel_data[2*1-1:(2-1)*1];

  assign shiftin_left = (!rotate) ? 0 : outparallel_data[1-1:0];

  velmshifter_2_1 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load),
      .shift(shift),
      .dir_left(1),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe(inparallel_data),
      .outpipe(outparallel_data));


endmodule



module bfloat_adder_10 (
 input clk,
 input resetn,
 input en,
 input stall,
 input [10-1:0] a,
 input [10-1:0] b,
 output reg[10-1:0] out
);

always@(posedge clk)begin
  if(!resetn)
    out <= 'h0;
  else
    if(en)
      out <= a + b;
end

endmodule
        


module vdispatcher_add_2_7 (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    increment,
    squash,

    add_sub,
    valuetoadd,

    inshift_data,

    inparallel_data,
    outparallel_data

    );


input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ 2-1:0 ]  increment;
input [ 2-1:0 ]  squash;

input               add_sub;
input [ 7-1:0 ] valuetoadd;

input [ 7-1:0 ] inshift_data;

input [ 2*7-1:0 ]  inparallel_data;
output [ 2*7-1:0 ] outparallel_data;

wire [ 7-1:0 ]  shiftin_right;

reg [ 2*7-1:0 ] outparallel_data;
reg [ 2*7-1:0 ] outparallel_added;


  assign shiftin_right = (!rotate) ? inshift_data : 
                    outparallel_added[2*7-1:(2-1)*7];

  reg [ 7-1:0 ] v;
  reg[31:0] i;
  always@*
    for (i=0; i<2; i=i+1)
    begin
      //Saturate at 0xffff... in either direction
      v=(increment[i] && outparallel_data[i*7 +: 7] != {7{1'b1}}) ?
        valuetoadd : 0;
      if (add_sub==0)
        outparallel_added[i*7 +: 7]=outparallel_data[i*7+:7] + v;
      else
        outparallel_added[i*7 +: 7]=outparallel_data[i*7+:7] - v;
    end

  always@(posedge clk)
    if (!resetn)
      outparallel_data=0;
    else if (load)
      outparallel_data=inparallel_data;
    else if (shift)
      outparallel_data=(outparallel_added<<7) | shiftin_right;
      


endmodule


module vmem_busmux_128_7_32_5 (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter SELWIDTH=7-5;   // LOG2(INWIDTH/OUTWIDTH) = 4

input clk;
input resetn;
input  [SELWIDTH-1 : 0] sel;
input  [128-1 : 0]  in;
output [32-1 : 0] out;

reg    [32-1 : 0] out;

integer k;

  always@*
  begin
    out=0;
    for (k=0; k<128/32; k=k+1)
      if (k==sel)
        out=in[ k*32 +: 32 ];
  end
endmodule
        /************************
 *
 *************************/

module vmem_crossbar_128_7_2_32_5 (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter SELWIDTH=7-5;   // LOG2(INWIDTH/OUTWIDTH) = 4

input                             clk;
input                             resetn;
input  [(SELWIDTH*2)-1 : 0] sel;
input  [128-1 : 0]            in;
output [(32*2)-1 : 0] out;


     vmem_busmux_128_7_32_5 bmux0(clk,resetn,
        sel[(0+1)*SELWIDTH - 1 : 0*SELWIDTH],
        in,
        out[(0+1)*32 - 1 : 0*32]);

     vmem_busmux_128_7_32_5 bmux1(clk,resetn,
        sel[(1+1)*SELWIDTH - 1 : 1*SELWIDTH],
        in,
        out[(1+1)*32 - 1 : 1*32]);

 endmodule 


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
/****************************************************************************
          Generic Register
****************************************************************************/
module register_33(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [33-1:0] d;
output [33-1:0] q;
reg [33-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule
/****************************************************************************
          Generic Register
****************************************************************************/
module register_5(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [5-1:0] d;
output [5-1:0] q;
reg [5-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule
/****************************************************************************
          Generic Register
****************************************************************************/
module register_1(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [1-1:0] d;
output [1-1:0] q;
reg [1-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule



module vdispatcher_add_2_3 (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    increment,
    squash,

    add_sub,
    valuetoadd,

    inshift_data,

    inparallel_data,
    outparallel_data

    );


input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ 2-1:0 ]  increment;
input [ 2-1:0 ]  squash;

input               add_sub;
input [ 3-1:0 ] valuetoadd;

input [ 3-1:0 ] inshift_data;

input [ 2*3-1:0 ]  inparallel_data;
output [ 2*3-1:0 ] outparallel_data;

wire [ 3-1:0 ]  shiftin_right;

reg [ 2*3-1:0 ] outparallel_data;
reg [ 2*3-1:0 ] outparallel_added;


  assign shiftin_right = (!rotate) ? inshift_data : 
                    outparallel_added[2*3-1:(2-1)*3];

  reg [ 3-1:0 ] v;
  reg[31:0] i;
  always@*
    for (i=0; i<2; i=i+1)
    begin
      //Saturate at 0xffff... in either direction
      v=(increment[i] && outparallel_data[i*3 +: 3] != {3{1'b1}}) ?
        valuetoadd : 0;
      if (add_sub==0)
        outparallel_added[i*3 +: 3]=outparallel_data[i*3+:3] + v;
      else
        outparallel_added[i*3 +: 3]=outparallel_data[i*3+:3] - v;
    end

  always@(posedge clk)
    if (!resetn)
      outparallel_data=0;
    else if (load)
      outparallel_data=inparallel_data;
    else if (shift)
      outparallel_data=(outparallel_added<<3) | shiftin_right;
      


endmodule



module vdispatcher_shift_2_157 (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    squash,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ 2-1:0 ]  squash;

input [ 157-1:0 ] inshift_data;

input [ 2*157-1:0 ]  inparallel_data;
output [ 2*157-1:0 ] outparallel_data;

wire [ 157-1:0 ]  shiftin_left;
wire [ 157-1:0 ]  shiftin_right;


  assign shiftin_right = (!rotate) ? inshift_data : 
                      outparallel_data[2*157-1:(2-1)*157];

  assign shiftin_left = (!rotate) ? 0 : outparallel_data[157-1:0];

  velmshifter_2_157 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load),
      .shift(shift),
      .dir_left(1),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe(inparallel_data),
      .outpipe(outparallel_data));


endmodule


//////////////////////////////////////////////////////////////////////////
// Output logic
//////////////////////////////////////////////////////////////////////////
module output_logic(
matmul_op_in_progress,
done_mat_mul,
c_data_out,
c_data_available,
clk_cnt,
row_latch_en,
matrixC0_0,
matrixC0_1,
matrixC0_2,
matrixC0_3,
matrixC0_4,
matrixC0_5,
matrixC0_6,
matrixC0_7,
matrixC1_0,
matrixC1_1,
matrixC1_2,
matrixC1_3,
matrixC1_4,
matrixC1_5,
matrixC1_6,
matrixC1_7,
matrixC2_0,
matrixC2_1,
matrixC2_2,
matrixC2_3,
matrixC2_4,
matrixC2_5,
matrixC2_6,
matrixC2_7,
matrixC3_0,
matrixC3_1,
matrixC3_2,
matrixC3_3,
matrixC3_4,
matrixC3_5,
matrixC3_6,
matrixC3_7,
matrixC4_0,
matrixC4_1,
matrixC4_2,
matrixC4_3,
matrixC4_4,
matrixC4_5,
matrixC4_6,
matrixC4_7,
matrixC5_0,
matrixC5_1,
matrixC5_2,
matrixC5_3,
matrixC5_4,
matrixC5_5,
matrixC5_6,
matrixC5_7,
matrixC6_0,
matrixC6_1,
matrixC6_2,
matrixC6_3,
matrixC6_4,
matrixC6_5,
matrixC6_6,
matrixC6_7,
matrixC7_0,
matrixC7_1,
matrixC7_2,
matrixC7_3,
matrixC7_4,
matrixC7_5,
matrixC7_6,
matrixC7_7,

clk,
reset
);

input clk;
input reset;
input matmul_op_in_progress;
input done_mat_mul;
output [`MAT_MUL_SIZE*`DWIDTH-1:0] c_data_out;
output c_data_available;
input [7:0] clk_cnt;
output row_latch_en;
input [`DWIDTH-1:0] matrixC0_0;
input [`DWIDTH-1:0] matrixC0_1;
input [`DWIDTH-1:0] matrixC0_2;
input [`DWIDTH-1:0] matrixC0_3;
input [`DWIDTH-1:0] matrixC0_4;
input [`DWIDTH-1:0] matrixC0_5;
input [`DWIDTH-1:0] matrixC0_6;
input [`DWIDTH-1:0] matrixC0_7;
input [`DWIDTH-1:0] matrixC1_0;
input [`DWIDTH-1:0] matrixC1_1;
input [`DWIDTH-1:0] matrixC1_2;
input [`DWIDTH-1:0] matrixC1_3;
input [`DWIDTH-1:0] matrixC1_4;
input [`DWIDTH-1:0] matrixC1_5;
input [`DWIDTH-1:0] matrixC1_6;
input [`DWIDTH-1:0] matrixC1_7;
input [`DWIDTH-1:0] matrixC2_0;
input [`DWIDTH-1:0] matrixC2_1;
input [`DWIDTH-1:0] matrixC2_2;
input [`DWIDTH-1:0] matrixC2_3;
input [`DWIDTH-1:0] matrixC2_4;
input [`DWIDTH-1:0] matrixC2_5;
input [`DWIDTH-1:0] matrixC2_6;
input [`DWIDTH-1:0] matrixC2_7;
input [`DWIDTH-1:0] matrixC3_0;
input [`DWIDTH-1:0] matrixC3_1;
input [`DWIDTH-1:0] matrixC3_2;
input [`DWIDTH-1:0] matrixC3_3;
input [`DWIDTH-1:0] matrixC3_4;
input [`DWIDTH-1:0] matrixC3_5;
input [`DWIDTH-1:0] matrixC3_6;
input [`DWIDTH-1:0] matrixC3_7;
input [`DWIDTH-1:0] matrixC4_0;
input [`DWIDTH-1:0] matrixC4_1;
input [`DWIDTH-1:0] matrixC4_2;
input [`DWIDTH-1:0] matrixC4_3;
input [`DWIDTH-1:0] matrixC4_4;
input [`DWIDTH-1:0] matrixC4_5;
input [`DWIDTH-1:0] matrixC4_6;
input [`DWIDTH-1:0] matrixC4_7;
input [`DWIDTH-1:0] matrixC5_0;
input [`DWIDTH-1:0] matrixC5_1;
input [`DWIDTH-1:0] matrixC5_2;
input [`DWIDTH-1:0] matrixC5_3;
input [`DWIDTH-1:0] matrixC5_4;
input [`DWIDTH-1:0] matrixC5_5;
input [`DWIDTH-1:0] matrixC5_6;
input [`DWIDTH-1:0] matrixC5_7;
input [`DWIDTH-1:0] matrixC6_0;
input [`DWIDTH-1:0] matrixC6_1;
input [`DWIDTH-1:0] matrixC6_2;
input [`DWIDTH-1:0] matrixC6_3;
input [`DWIDTH-1:0] matrixC6_4;
input [`DWIDTH-1:0] matrixC6_5;
input [`DWIDTH-1:0] matrixC6_6;
input [`DWIDTH-1:0] matrixC6_7;
input [`DWIDTH-1:0] matrixC7_0;
input [`DWIDTH-1:0] matrixC7_1;
input [`DWIDTH-1:0] matrixC7_2;
input [`DWIDTH-1:0] matrixC7_3;
input [`DWIDTH-1:0] matrixC7_4;
input [`DWIDTH-1:0] matrixC7_5;
input [`DWIDTH-1:0] matrixC7_6;
input [`DWIDTH-1:0] matrixC7_7;
wire row_latch_en;


//////////////////////////////////////////////////////////////////////////
// Logic to capture matrix C data from the PEs and shift it out
//////////////////////////////////////////////////////////////////////////
//assign row_latch_en = (clk_cnt==(`MAT_MUL_SIZE + (a_loc+b_loc) * `BB_MAT_MUL_SIZE + 10 +  `NUM_CYCLES_IN_MAC - 1));
//Writing the line above to avoid multiplication:
//assign row_latch_en = (clk_cnt==(`MAT_MUL_SIZE + ((a_loc+b_loc) << `LOG2_MAT_MUL_SIZE) + 10 +  `NUM_CYCLES_IN_MAC - 1));

assign row_latch_en =  
                       ((clk_cnt == 27 ));
    
reg c_data_available;
reg start_capturing_c_data;
reg[31:0] counter;
reg [8*`DWIDTH-1:0] c_data_out;
reg [8*`DWIDTH-1:0] c_data_out_1;
reg [8*`DWIDTH-1:0] c_data_out_2;
reg [8*`DWIDTH-1:0] c_data_out_3;
reg [8*`DWIDTH-1:0] c_data_out_4;
reg [8*`DWIDTH-1:0] c_data_out_5;
reg [8*`DWIDTH-1:0] c_data_out_6;
reg [8*`DWIDTH-1:0] c_data_out_7;
wire condition_to_start_shifting_output;
assign condition_to_start_shifting_output = 
                          row_latch_en ;  

  
//For larger matmuls, this logic will have more entries in the case statement
always @(posedge clk) begin
  if (reset | ~matmul_op_in_progress) begin
    start_capturing_c_data <= 1'b0;
    c_data_available <= 1'b0;
    c_data_out <= 0;
    counter <= 0;

    c_data_out_1 <= 0;
    c_data_out_2 <= 0;
    c_data_out_3 <= 0;
    c_data_out_4 <= 0;
    c_data_out_5 <= 0;
    c_data_out_6 <= 0;
    c_data_out_7 <= 0;
  end else if (condition_to_start_shifting_output) begin
    start_capturing_c_data <= 1'b1;
    c_data_available <= 1'b1;
    c_data_out <= {matrixC7_7, matrixC6_7, matrixC5_7, matrixC4_7, matrixC3_7, matrixC2_7, matrixC1_7, matrixC0_7};
    c_data_out_1 <= {matrixC7_6, matrixC6_6, matrixC5_6, matrixC4_6, matrixC3_6, matrixC2_6, matrixC1_6, matrixC0_6};
    c_data_out_2 <= {matrixC7_5, matrixC6_5, matrixC5_5, matrixC4_5, matrixC3_5, matrixC2_5, matrixC1_5, matrixC0_5};
    c_data_out_3 <= {matrixC7_4, matrixC6_4, matrixC5_4, matrixC4_4, matrixC3_4, matrixC2_4, matrixC1_4, matrixC0_4};
    c_data_out_4 <= {matrixC7_3, matrixC6_3, matrixC5_3, matrixC4_3, matrixC3_3, matrixC2_3, matrixC1_3, matrixC0_3};
    c_data_out_5 <= {matrixC7_2, matrixC6_2, matrixC5_2, matrixC4_2, matrixC3_2, matrixC2_2, matrixC1_2, matrixC0_2};
    c_data_out_6 <= {matrixC7_1, matrixC6_1, matrixC5_1, matrixC4_1, matrixC3_1, matrixC2_1, matrixC1_1, matrixC0_1};
    c_data_out_7 <= {matrixC7_0, matrixC6_0, matrixC5_0, matrixC4_0, matrixC3_0, matrixC2_0, matrixC1_0, matrixC0_0};

    counter <= counter + 1;
  end else if (done_mat_mul) begin
    start_capturing_c_data <= 1'b0;
    c_data_available <= 1'b0;
    c_data_out <= 0;

    c_data_out_1 <= 0;
    c_data_out_2 <= 0;
    c_data_out_3 <= 0;
    c_data_out_4 <= 0;
    c_data_out_5 <= 0;
    c_data_out_6 <= 0;
    c_data_out_7 <= 0;
  end 
  else if (counter >= `MAT_MUL_SIZE) begin
    c_data_out <= c_data_out_1;

    c_data_out_1 <= c_data_out_2;
    c_data_out_2 <= c_data_out_3;
    c_data_out_3 <= c_data_out_4;
    c_data_out_4 <= c_data_out_5;
    c_data_out_5 <= c_data_out_6;
    c_data_out_6 <= c_data_out_7;
    c_data_out_7 <= 0;
  end
  else if (start_capturing_c_data) begin
    c_data_available <= 1'b1;
    counter <= counter + 1;
    c_data_out <= c_data_out_1;

    c_data_out_1 <= c_data_out_2;
    c_data_out_2 <= c_data_out_3;
    c_data_out_3 <= c_data_out_4;
    c_data_out_4 <= c_data_out_5;
    c_data_out_5 <= c_data_out_6;
    c_data_out_6 <= c_data_out_7;
    c_data_out_7 <= 0;
  end
end

endmodule
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


/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_2_157 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 2-1:0 ]  squash;

input [ 2*157-1:0 ]  inpipe;
output [ 2*157-1:0 ] outpipe;

input [ 157-1:0 ]  shiftin_left;
input [ 157-1:0 ]  shiftin_right;

wire [ (2+1)*157-1:0 ] _outpipe;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_157 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[156:0],
      _outpipe[313:157], //Support 1 lane
      shiftin_right,
      _outpipe[156:0]);
 // defparam velmshifter_laneunit0.157=157;

  //Generate everything in between 


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_157 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[1],
      inpipe[313:157],
      shiftin_left,
      _outpipe[156:0], //L=1
      _outpipe[313:157]); //L=1
   // defparam velmshifter_laneunitlast.157=157;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
module velmshifter_laneunit_157 (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );


input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 157-1:0 ]  inpipe;
input [ 157-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 157-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 157-1:0 ] outpipe;

reg [ 157-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        
module dpram_9_512_4 (
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
input [9-1:0] address_a;
input [9-1:0] address_b;
input  wren_a;
input  wren_b;
input [4-1:0] data_a;
input [4-1:0] data_b;
output reg [4-1:0] out_a;
output reg [4-1:0] out_b;

reg [4-1:0] ram[512-1:0];

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

endmodule


///////////////////////////////////////////////////////
//Rounding logic based on convergent rounding described 
//here: https://zipcpu.com/dsp/2017/07/22/rounding.html
///////////////////////////////////////////////////////
module rounding_13_10( i_data, o_data );
input  [13-1:0] i_data;
output [10-1:0] o_data;

wire [13-1:0] w_convergent;

assign	w_convergent = i_data[(13-1):0]
			+ { {(10){1'b0}},
				i_data[(13-10)],
                                {(13-10-1){!i_data[(13-10)]}}};

assign o_data = w_convergent[(13-1):(13-10)];

endmodule
        
module velmshifter_laneunit_32 (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );


input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 32-1:0 ]  inpipe;
input [ 32-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 32-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 32-1:0 ] outpipe;

reg [ 32-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        
module matmul_8x8(
 clk,
 //Reset for the whole matmul
 reset,
 //Reset for the PEs. Typically connected to 'reset'
 pe_reset,
 //When this is asserted, the matmul operation starts. This can remain
 //asserted during the execution, but is not required to.
 start, 
 //This is asserted a cycle after the matmul start operation. It stays
 //asserted until the matmul starts shifting out the output data.
 in_progress,
 //This is not used any more. But it is a pulse, and is asserted when
 //the execution (including output shift out) is finished.
 done,
 //Input data matrix A. For MAT_MUL_SIZE=8, 64 values come in over 8 cycles.
 //8 values in one cycle.
 a_data,
 //Input data matrix B. For MAT_MUL_SIZE=8, 64 values come in over 8 cycles.
 //8 values in one cycle.
 b_data,
 //Output data matrix C. For MAT_MUL_SIZE=8, 64 values come out over 8 cycles.
 //8 values in one cycle.
 c_data_out, 
 //This isn't used anymore. It stays asserted when the output data matrix C
 //is being shifted out.
 c_data_available,
 //Masks for input matrices A and B. These are used when we want to use this 
 //unit to multiply matrices that are less than 8x8 (eg: 6x4 with a 4x5 matrix).
 validity_mask_a_rows,
 validity_mask_a_cols,
 validity_mask_b_rows,
 validity_mask_b_cols
);

 input clk;
 input reset;
 input pe_reset;
 input start;
 output in_progress;
 output done;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data;
 output [`MAT_MUL_SIZE*`DWIDTH-1:0] c_data_out;
 output c_data_available;

 input [`MASK_WIDTH-1:0] validity_mask_a_rows;
 input [`MASK_WIDTH-1:0] validity_mask_a_cols;
 input [`MASK_WIDTH-1:0] validity_mask_b_rows;
 input [`MASK_WIDTH-1:0] validity_mask_b_cols;

//////////////////////////////////////////////////////////////////////////
// Logic for clock counting and when to assert done
//////////////////////////////////////////////////////////////////////////
reg in_progress;
reg done_mat_mul;
wire matmul_op_in_progress;
reg shift_out_data;
reg shift_in_data;
wire start_pulse;
wire [7:0] clk_cnt_for_done;
wire [7:0] clk_cnt_for_latching_c_data;
wire [7:0] clk_cnt_for_shifting_inputs;

assign done = done_mat_mul;

reg start_delayed;
always @(posedge clk) begin
  if (reset) begin
    start_delayed <= 0;
  end 
  else begin
    start_delayed <= start;
  end
end

assign start_pulse = start & (~start_delayed);

//This signal is used in other modules instantiated in this design.
//It stays high during the entire time matmul is working.
//                            _
//start_pulse           _____| |___________________________________________
//                              ___
//shift_in_data         _______|   |_______________________________________
//                                   ________________________
//in_progress           ____________|                        |______________
//                                                            _______
//shift_out_data        _____________________________________|       |______
//                            _______________________________________
//matmul_op_in_progress _____|                                       |______
assign matmul_op_in_progress = start_pulse|shift_in_data|in_progress|shift_out_data;

//This is 7 bits because the expectation is that clock count will be pretty
//small. For large matmuls, this will need to increased to have more bits.
//In general, a systolic multiplier takes 4*N-2+P cycles, where N is the size 
//of the matmul and P is the number of pipleine stages in the MAC block.
reg [7:0] clk_cnt;

//Number of cycles to assert matmul done. This includes the cycles to shift out the results. 
//This is hardcoded here, because this was generated using a script.
assign clk_cnt_for_done = 
                          (34);  

//Number of cycles at which we latch the output data and start shifting it out.
assign clk_cnt_for_latching_c_data =                        
                          (27);  

//Number of cycles at which we finish shifting inputs into the matmul.
//Note that while this shifting is happening, the matmul is calculating
//outputs in its PEs.
assign clk_cnt_for_shifting_inputs =                        
                          (1);  //Ideally this should have been 7, but if we keep this as
                                //7, then stall signal is asserted a bit later than required

always @(posedge clk) begin
  if (reset) begin
    clk_cnt <= 0;
    done_mat_mul <= 0;
    in_progress <= 0;
    shift_out_data <= 0;
    shift_in_data <= 0;
  end
  else if (start_pulse == 1'b1) begin
    clk_cnt <= clk_cnt + 1;
    done_mat_mul <= 0;
    in_progress <= 0;
    shift_out_data <= 0;
    shift_in_data <= 1;
  end
  else if (clk_cnt == clk_cnt_for_shifting_inputs) begin 
    clk_cnt <= clk_cnt + 1;
    done_mat_mul <= 0;
    in_progress <= 1;
    shift_out_data <= 0;
    shift_in_data <= 0;
  end
  else if (clk_cnt == clk_cnt_for_latching_c_data) begin 
    clk_cnt <= clk_cnt + 1;
    done_mat_mul <= 0;
    in_progress <= 0;
    shift_out_data <= 1;
    shift_in_data <= 0;
  end
  else if (clk_cnt == clk_cnt_for_done) begin
    done_mat_mul <= 1;
    clk_cnt <= clk_cnt + 1;
    in_progress <= 0;
    shift_out_data <= 0;
    shift_in_data <= 0;
  end
  else if ((in_progress == 1) || (shift_out_data == 1) || (shift_in_data == 1)) begin
    clk_cnt <= clk_cnt + 1;
  end    
  else begin
    clk_cnt <= 0;
    done_mat_mul <= 0;
    in_progress <= 0;
  end
end

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
wire [`DWIDTH-1:0] a1_data_delayed_1;
wire [`DWIDTH-1:0] a2_data_delayed_1;
wire [`DWIDTH-1:0] a2_data_delayed_2;
wire [`DWIDTH-1:0] a3_data_delayed_1;
wire [`DWIDTH-1:0] a3_data_delayed_2;
wire [`DWIDTH-1:0] a3_data_delayed_3;
wire [`DWIDTH-1:0] a4_data_delayed_1;
wire [`DWIDTH-1:0] a4_data_delayed_2;
wire [`DWIDTH-1:0] a4_data_delayed_3;
wire [`DWIDTH-1:0] a4_data_delayed_4;
wire [`DWIDTH-1:0] a5_data_delayed_1;
wire [`DWIDTH-1:0] a5_data_delayed_2;
wire [`DWIDTH-1:0] a5_data_delayed_3;
wire [`DWIDTH-1:0] a5_data_delayed_4;
wire [`DWIDTH-1:0] a5_data_delayed_5;
wire [`DWIDTH-1:0] a6_data_delayed_1;
wire [`DWIDTH-1:0] a6_data_delayed_2;
wire [`DWIDTH-1:0] a6_data_delayed_3;
wire [`DWIDTH-1:0] a6_data_delayed_4;
wire [`DWIDTH-1:0] a6_data_delayed_5;
wire [`DWIDTH-1:0] a6_data_delayed_6;
wire [`DWIDTH-1:0] a7_data_delayed_1;
wire [`DWIDTH-1:0] a7_data_delayed_2;
wire [`DWIDTH-1:0] a7_data_delayed_3;
wire [`DWIDTH-1:0] a7_data_delayed_4;
wire [`DWIDTH-1:0] a7_data_delayed_5;
wire [`DWIDTH-1:0] a7_data_delayed_6;
wire [`DWIDTH-1:0] a7_data_delayed_7;
wire [`DWIDTH-1:0] b1_data_delayed_1;
wire [`DWIDTH-1:0] b2_data_delayed_1;
wire [`DWIDTH-1:0] b2_data_delayed_2;
wire [`DWIDTH-1:0] b3_data_delayed_1;
wire [`DWIDTH-1:0] b3_data_delayed_2;
wire [`DWIDTH-1:0] b3_data_delayed_3;
wire [`DWIDTH-1:0] b4_data_delayed_1;
wire [`DWIDTH-1:0] b4_data_delayed_2;
wire [`DWIDTH-1:0] b4_data_delayed_3;
wire [`DWIDTH-1:0] b4_data_delayed_4;
wire [`DWIDTH-1:0] b5_data_delayed_1;
wire [`DWIDTH-1:0] b5_data_delayed_2;
wire [`DWIDTH-1:0] b5_data_delayed_3;
wire [`DWIDTH-1:0] b5_data_delayed_4;
wire [`DWIDTH-1:0] b5_data_delayed_5;
wire [`DWIDTH-1:0] b6_data_delayed_1;
wire [`DWIDTH-1:0] b6_data_delayed_2;
wire [`DWIDTH-1:0] b6_data_delayed_3;
wire [`DWIDTH-1:0] b6_data_delayed_4;
wire [`DWIDTH-1:0] b6_data_delayed_5;
wire [`DWIDTH-1:0] b6_data_delayed_6;
wire [`DWIDTH-1:0] b7_data_delayed_1;
wire [`DWIDTH-1:0] b7_data_delayed_2;
wire [`DWIDTH-1:0] b7_data_delayed_3;
wire [`DWIDTH-1:0] b7_data_delayed_4;
wire [`DWIDTH-1:0] b7_data_delayed_5;
wire [`DWIDTH-1:0] b7_data_delayed_6;
wire [`DWIDTH-1:0] b7_data_delayed_7;


//////////////////////////////////////////////////////////////////////////
// Instantiation of systolic data setup
//////////////////////////////////////////////////////////////////////////
systolic_data_setup u_systolic_data_setup(
.clk(clk),
.reset(reset),
.matmul_op_in_progress(matmul_op_in_progress),
.a_data(a_data),
.b_data(b_data),
.clk_cnt(clk_cnt),
.a0_data(a0_data),
.b0_data(b0_data),
.a1_data_delayed_1(a1_data_delayed_1),
.b1_data_delayed_1(b1_data_delayed_1),
.a2_data_delayed_2(a2_data_delayed_2),
.b2_data_delayed_2(b2_data_delayed_2),
.a3_data_delayed_3(a3_data_delayed_3),
.b3_data_delayed_3(b3_data_delayed_3),
.a4_data_delayed_4(a4_data_delayed_4),
.b4_data_delayed_4(b4_data_delayed_4),
.a5_data_delayed_5(a5_data_delayed_5),
.b5_data_delayed_5(b5_data_delayed_5),
.a6_data_delayed_6(a6_data_delayed_6),
.b6_data_delayed_6(b6_data_delayed_6),
.a7_data_delayed_7(a7_data_delayed_7),
.b7_data_delayed_7(b7_data_delayed_7),

.validity_mask_a_rows(validity_mask_a_rows),
.validity_mask_a_cols(validity_mask_a_cols),
.validity_mask_b_rows(validity_mask_b_rows),
.validity_mask_b_cols(validity_mask_b_cols)

);

wire [`DWIDTH-1:0] a0;
wire [`DWIDTH-1:0] a1;
wire [`DWIDTH-1:0] a2;
wire [`DWIDTH-1:0] a3;
wire [`DWIDTH-1:0] a4;
wire [`DWIDTH-1:0] a5;
wire [`DWIDTH-1:0] a6;
wire [`DWIDTH-1:0] a7;
wire [`DWIDTH-1:0] b0;
wire [`DWIDTH-1:0] b1;
wire [`DWIDTH-1:0] b2;
wire [`DWIDTH-1:0] b3;
wire [`DWIDTH-1:0] b4;
wire [`DWIDTH-1:0] b5;
wire [`DWIDTH-1:0] b6;
wire [`DWIDTH-1:0] b7;

assign a0 = a0_data          ;
assign a1 = a1_data_delayed_1;
assign a2 = a2_data_delayed_2;
assign a3 = a3_data_delayed_3;
assign a4 = a4_data_delayed_4;
assign a5 = a5_data_delayed_5;
assign a6 = a6_data_delayed_6;
assign a7 = a7_data_delayed_7;

assign b0 = b0_data          ;
assign b1 = b1_data_delayed_1;
assign b2 = b2_data_delayed_2;
assign b3 = b3_data_delayed_3;
assign b4 = b4_data_delayed_4;
assign b5 = b5_data_delayed_5;
assign b6 = b6_data_delayed_6;
assign b7 = b7_data_delayed_7;

wire [`DWIDTH-1:0] matrixC0_0;
wire [`DWIDTH-1:0] matrixC0_1;
wire [`DWIDTH-1:0] matrixC0_2;
wire [`DWIDTH-1:0] matrixC0_3;
wire [`DWIDTH-1:0] matrixC0_4;
wire [`DWIDTH-1:0] matrixC0_5;
wire [`DWIDTH-1:0] matrixC0_6;
wire [`DWIDTH-1:0] matrixC0_7;
wire [`DWIDTH-1:0] matrixC1_0;
wire [`DWIDTH-1:0] matrixC1_1;
wire [`DWIDTH-1:0] matrixC1_2;
wire [`DWIDTH-1:0] matrixC1_3;
wire [`DWIDTH-1:0] matrixC1_4;
wire [`DWIDTH-1:0] matrixC1_5;
wire [`DWIDTH-1:0] matrixC1_6;
wire [`DWIDTH-1:0] matrixC1_7;
wire [`DWIDTH-1:0] matrixC2_0;
wire [`DWIDTH-1:0] matrixC2_1;
wire [`DWIDTH-1:0] matrixC2_2;
wire [`DWIDTH-1:0] matrixC2_3;
wire [`DWIDTH-1:0] matrixC2_4;
wire [`DWIDTH-1:0] matrixC2_5;
wire [`DWIDTH-1:0] matrixC2_6;
wire [`DWIDTH-1:0] matrixC2_7;
wire [`DWIDTH-1:0] matrixC3_0;
wire [`DWIDTH-1:0] matrixC3_1;
wire [`DWIDTH-1:0] matrixC3_2;
wire [`DWIDTH-1:0] matrixC3_3;
wire [`DWIDTH-1:0] matrixC3_4;
wire [`DWIDTH-1:0] matrixC3_5;
wire [`DWIDTH-1:0] matrixC3_6;
wire [`DWIDTH-1:0] matrixC3_7;
wire [`DWIDTH-1:0] matrixC4_0;
wire [`DWIDTH-1:0] matrixC4_1;
wire [`DWIDTH-1:0] matrixC4_2;
wire [`DWIDTH-1:0] matrixC4_3;
wire [`DWIDTH-1:0] matrixC4_4;
wire [`DWIDTH-1:0] matrixC4_5;
wire [`DWIDTH-1:0] matrixC4_6;
wire [`DWIDTH-1:0] matrixC4_7;
wire [`DWIDTH-1:0] matrixC5_0;
wire [`DWIDTH-1:0] matrixC5_1;
wire [`DWIDTH-1:0] matrixC5_2;
wire [`DWIDTH-1:0] matrixC5_3;
wire [`DWIDTH-1:0] matrixC5_4;
wire [`DWIDTH-1:0] matrixC5_5;
wire [`DWIDTH-1:0] matrixC5_6;
wire [`DWIDTH-1:0] matrixC5_7;
wire [`DWIDTH-1:0] matrixC6_0;
wire [`DWIDTH-1:0] matrixC6_1;
wire [`DWIDTH-1:0] matrixC6_2;
wire [`DWIDTH-1:0] matrixC6_3;
wire [`DWIDTH-1:0] matrixC6_4;
wire [`DWIDTH-1:0] matrixC6_5;
wire [`DWIDTH-1:0] matrixC6_6;
wire [`DWIDTH-1:0] matrixC6_7;
wire [`DWIDTH-1:0] matrixC7_0;
wire [`DWIDTH-1:0] matrixC7_1;
wire [`DWIDTH-1:0] matrixC7_2;
wire [`DWIDTH-1:0] matrixC7_3;
wire [`DWIDTH-1:0] matrixC7_4;
wire [`DWIDTH-1:0] matrixC7_5;
wire [`DWIDTH-1:0] matrixC7_6;
wire [`DWIDTH-1:0] matrixC7_7;

wire [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data_out;
wire [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data_out;

wire row_latch_en;
//////////////////////////////////////////////////////////////////////////
// Instantiation of the output logic
//////////////////////////////////////////////////////////////////////////
output_logic u_output_logic(
.matmul_op_in_progress(matmul_op_in_progress),
.done_mat_mul(done_mat_mul),
.c_data_out(c_data_out),
.c_data_available(c_data_available),
.clk_cnt(clk_cnt),
.row_latch_en(row_latch_en),
.matrixC0_0(matrixC0_0),
.matrixC0_1(matrixC0_1),
.matrixC0_2(matrixC0_2),
.matrixC0_3(matrixC0_3),
.matrixC0_4(matrixC0_4),
.matrixC0_5(matrixC0_5),
.matrixC0_6(matrixC0_6),
.matrixC0_7(matrixC0_7),
.matrixC1_0(matrixC1_0),
.matrixC1_1(matrixC1_1),
.matrixC1_2(matrixC1_2),
.matrixC1_3(matrixC1_3),
.matrixC1_4(matrixC1_4),
.matrixC1_5(matrixC1_5),
.matrixC1_6(matrixC1_6),
.matrixC1_7(matrixC1_7),
.matrixC2_0(matrixC2_0),
.matrixC2_1(matrixC2_1),
.matrixC2_2(matrixC2_2),
.matrixC2_3(matrixC2_3),
.matrixC2_4(matrixC2_4),
.matrixC2_5(matrixC2_5),
.matrixC2_6(matrixC2_6),
.matrixC2_7(matrixC2_7),
.matrixC3_0(matrixC3_0),
.matrixC3_1(matrixC3_1),
.matrixC3_2(matrixC3_2),
.matrixC3_3(matrixC3_3),
.matrixC3_4(matrixC3_4),
.matrixC3_5(matrixC3_5),
.matrixC3_6(matrixC3_6),
.matrixC3_7(matrixC3_7),
.matrixC4_0(matrixC4_0),
.matrixC4_1(matrixC4_1),
.matrixC4_2(matrixC4_2),
.matrixC4_3(matrixC4_3),
.matrixC4_4(matrixC4_4),
.matrixC4_5(matrixC4_5),
.matrixC4_6(matrixC4_6),
.matrixC4_7(matrixC4_7),
.matrixC5_0(matrixC5_0),
.matrixC5_1(matrixC5_1),
.matrixC5_2(matrixC5_2),
.matrixC5_3(matrixC5_3),
.matrixC5_4(matrixC5_4),
.matrixC5_5(matrixC5_5),
.matrixC5_6(matrixC5_6),
.matrixC5_7(matrixC5_7),
.matrixC6_0(matrixC6_0),
.matrixC6_1(matrixC6_1),
.matrixC6_2(matrixC6_2),
.matrixC6_3(matrixC6_3),
.matrixC6_4(matrixC6_4),
.matrixC6_5(matrixC6_5),
.matrixC6_6(matrixC6_6),
.matrixC6_7(matrixC6_7),
.matrixC7_0(matrixC7_0),
.matrixC7_1(matrixC7_1),
.matrixC7_2(matrixC7_2),
.matrixC7_3(matrixC7_3),
.matrixC7_4(matrixC7_4),
.matrixC7_5(matrixC7_5),
.matrixC7_6(matrixC7_6),
.matrixC7_7(matrixC7_7),

.clk(clk),
.reset(reset)
);

//////////////////////////////////////////////////////////////////////////
// Instantiations of the actual PEs
//////////////////////////////////////////////////////////////////////////
systolic_pe_matrix u_systolic_pe_matrix(
.clk(clk),
.reset(reset),
.pe_reset(pe_reset),
.a0(a0),
.a1(a1),
.a2(a2),
.a3(a3),
.a4(a4),
.a5(a5),
.a6(a6),
.a7(a7),
.b0(b0),
.b1(b1),
.b2(b2),
.b3(b3),
.b4(b4),
.b5(b5),
.b6(b6),
.b7(b7),
.matrixC0_0(matrixC0_0),
.matrixC0_1(matrixC0_1),
.matrixC0_2(matrixC0_2),
.matrixC0_3(matrixC0_3),
.matrixC0_4(matrixC0_4),
.matrixC0_5(matrixC0_5),
.matrixC0_6(matrixC0_6),
.matrixC0_7(matrixC0_7),
.matrixC1_0(matrixC1_0),
.matrixC1_1(matrixC1_1),
.matrixC1_2(matrixC1_2),
.matrixC1_3(matrixC1_3),
.matrixC1_4(matrixC1_4),
.matrixC1_5(matrixC1_5),
.matrixC1_6(matrixC1_6),
.matrixC1_7(matrixC1_7),
.matrixC2_0(matrixC2_0),
.matrixC2_1(matrixC2_1),
.matrixC2_2(matrixC2_2),
.matrixC2_3(matrixC2_3),
.matrixC2_4(matrixC2_4),
.matrixC2_5(matrixC2_5),
.matrixC2_6(matrixC2_6),
.matrixC2_7(matrixC2_7),
.matrixC3_0(matrixC3_0),
.matrixC3_1(matrixC3_1),
.matrixC3_2(matrixC3_2),
.matrixC3_3(matrixC3_3),
.matrixC3_4(matrixC3_4),
.matrixC3_5(matrixC3_5),
.matrixC3_6(matrixC3_6),
.matrixC3_7(matrixC3_7),
.matrixC4_0(matrixC4_0),
.matrixC4_1(matrixC4_1),
.matrixC4_2(matrixC4_2),
.matrixC4_3(matrixC4_3),
.matrixC4_4(matrixC4_4),
.matrixC4_5(matrixC4_5),
.matrixC4_6(matrixC4_6),
.matrixC4_7(matrixC4_7),
.matrixC5_0(matrixC5_0),
.matrixC5_1(matrixC5_1),
.matrixC5_2(matrixC5_2),
.matrixC5_3(matrixC5_3),
.matrixC5_4(matrixC5_4),
.matrixC5_5(matrixC5_5),
.matrixC5_6(matrixC5_6),
.matrixC5_7(matrixC5_7),
.matrixC6_0(matrixC6_0),
.matrixC6_1(matrixC6_1),
.matrixC6_2(matrixC6_2),
.matrixC6_3(matrixC6_3),
.matrixC6_4(matrixC6_4),
.matrixC6_5(matrixC6_5),
.matrixC6_6(matrixC6_6),
.matrixC6_7(matrixC6_7),
.matrixC7_0(matrixC7_0),
.matrixC7_1(matrixC7_1),
.matrixC7_2(matrixC7_2),
.matrixC7_3(matrixC7_3),
.matrixC7_4(matrixC7_4),
.matrixC7_5(matrixC7_5),
.matrixC7_6(matrixC7_6),
.matrixC7_7(matrixC7_7),

.a_data_out(a_data_out),
.b_data_out(b_data_out)
);

endmodule
module velmshifter_laneunit_128 (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );


input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 128-1:0 ]  inpipe;
input [ 128-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 128-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 128-1:0 ] outpipe;

reg [ 128-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        
module velmshifter_laneunit_4 (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );


input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 4-1:0 ]  inpipe;
input [ 4-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 4-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 4-1:0 ] outpipe;

reg [ 4-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        

/******************************************************************************/
/**************************** Shifter w Jump **********************************/
/******************************************************************************/

module velmshifter_jump_32_4_2 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    jump,
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe
    );


input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  jump;              //0-shift by 1, 1 shift by 2
input [ 4-1:0 ]  squash;

input [ 4*32-1:0 ]  inpipe;
output [ 4*32-1:0 ] outpipe;

input [ 32-1:0 ]  shiftin_left;
input [ 32-1:0 ]  shiftin_right;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter_4_32 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<(32*2) :
                      outpipe >>(32*2)),
      .outpipe(outpipe));

endmodule
        
/******************************************************************************/
/**************************** Shifter w Jump **********************************/
/******************************************************************************/

module velmshifter_jump_1_4_2 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    jump,
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe
    );


input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  jump;              //0-shift by 1, 1 shift by 2
input [ 4-1:0 ]  squash;

input [ 4*1-1:0 ]  inpipe;
output [ 4*1-1:0 ] outpipe;

input [ 1-1:0 ]  shiftin_left;
input [ 1-1:0 ]  shiftin_right;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter_4_1 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<(1*2) :
                      outpipe >>(1*2)),
      .outpipe(outpipe));

endmodule
        
module qmult(i_multiplicand,i_multiplier,o_result);
input [`DWIDTH-1:0] i_multiplicand;
input [`DWIDTH-1:0] i_multiplier;
output [2*`DWIDTH-1:0] o_result;

assign o_result = i_multiplicand * i_multiplier;
//DW02_mult #(`DWIDTH,`DWIDTH) u_mult(.A(i_multiplicand), .B(i_multiplier), .TC(1'b1), .PRODUCT(o_result));

endmodule
module activation_32(
 input clk,
 input resetn,
 input en,
 input stall,
 input [32-1:0] a,
 output reg[32-1:0] out
);

always@(posedge clk)begin
  if(!resetn)
    out <= 'h0;
  else
    if(en)
      if(a>0)
        out <= a;
      else
        out <= 0;
end

endmodule

        
module velmshifter_laneunit_1 (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );


input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 1-1:0 ]  inpipe;
input [ 1-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 1-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 1-1:0 ] outpipe;

reg [ 1-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        

         
//`include "vlane_mulshift.v"
//`include "vlane_barrelshifter.v"

/****************************************************************************
          MUL unit

opA/B ---------------------------|         |                       | |----| Multiplier |--+------------------
         ------|shiftregister|---|/                     |--|shiftregister|-

 |  Pipe stage 1  |      Pipe stage 2    |    Pipe stage 3
                         done/stall
   operands----Multiply------------barrelshifter ----------

Notes:

  Stalls no matter the vector length - if the mask bits are all off we could
  theoretically skip this computation, but then we would need multiplexing
  logic to order the different results correctly so they line up with their
  lane.  Since we use a shift register to do this, all NUMLANES multiplies are
  performed.

****************************************************************************/

module vmul_unit_5_4_2_10 (clk, resetn,
            op,
            activate,
            en,
            squash,
            stall,
            opA, opB,
            vshamt,
            vmask,
            in_dst,
            in_dst_we,
            out_dst,
            out_dst_we,
            out_dst_mask,
            result
            );

parameter NUMLANES=2**2;
parameter WIDTH=2**5;

input clk;
input resetn;

input [NUMLANES*WIDTH-1:0] opA;
input [NUMLANES*WIDTH-1:0] opB;
// input [((5==0) ? 1 : 5)-1:0] vshamt;  // Fixed point rounding
// The original version is the above declaration. This is not supported by VTR. So I am using the below version ~ Aatman
input [5-1:0] vshamt;
input [NUMLANES-1:0] vmask;
input [4:0] op;
input       activate;
input [3:1] en;  //Enable for each pipestage
input [3:1] squash;  //Squash for each pipestage

input    [10-1:0] in_dst;
input                     in_dst_we;
output [3*10-1:0] out_dst;
output              [2:0] out_dst_we;
output   [3*NUMLANES-1:0] out_dst_mask;

output stall;
output [NUMLANES*WIDTH-1:0] result;

  /********* Circuit Body *********/
  wire [4*WIDTH-1:0] mul_opA;
  wire [4*WIDTH-1:0] mul_opB;
  wire [4*WIDTH-1:0] mul_result;
  wire [4*WIDTH-1:0] rshift_result;
  wire [NUMLANES*WIDTH-1:0] result_tmp;

  wire [NUMLANES*WIDTH-1:0] opA_buffered;
  wire [NUMLANES*WIDTH-1:0] opB_buffered;
  wire [NUMLANES-1:0]       mask_buffered;
  wire [NUMLANES*WIDTH-1:0] result_buffered;
  reg  done;
  wire [4:0] ctrl_op[3:1];                  //3 pipe stages
  wire [3:1] ctrl_activate;                 //3 pipe stages
  //amana: Making modification for VTR
  //wire [((5==0) ? 1 : 5)-1:0] ctrl_vshamt[3:1]; //3 pipe stages
  // wire [((5==0) ? 1*3 : 5*3)-1:0] ctrl_vshamt; //3 pipe stages
  wire [5*3-1:0] ctrl_vshamt; //3 pipe stages

  //Shift Register for all multiplier operands for lanes without multipliers
  wire [WIDTH*4-1:0] opA_elmshifter_shiftin_right_NC;
 
 //TODO: update the parameters
  velmshifter_1_128  opA_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*4{1'b0}}),
    .shiftin_right(opA_elmshifter_shiftin_right_NC), 
    .inpipe(opA),
    .outpipe(opA_buffered)
  );

  wire [WIDTH*4-1:0] opB_elmshifter_shiftin_right_NC;

  //TODO: update the parameters
  velmshifter_1_128 opB_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*4{1'b0}}),
    .shiftin_right(opB_elmshifter_shiftin_right_NC), 
    .inpipe(opB),
    .outpipe(opB_buffered)
  );

  wire [4-1:0] mask_elmshifter_shiftin_right_NC;
  wire mask_elmshifter_load;

  assign mask_elmshifter_load = done & ctrl_activate[1];

  //TODO: Update the parameters 
  velmshifter_1_4  mask_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(mask_elmshifter_load),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({4{1'b0}}),
    .shiftin_right(mask_elmshifter_shiftin_right_NC), 
    //.inpipe(vmask), //DISABLE - always do all multiplications
    .inpipe({NUMLANES{1'b1}}),
    .outpipe(mask_buffered)
  );

  //Shift Register for all multiplier operands w/ parallel load
  always@(posedge clk)
  begin
    if (!resetn || 4==NUMLANES)
      done<=1;
    else if (done && ctrl_activate[1] && en[1])
      //done<=~(|(vmask>>4)); // multiply only if mask - DISABLED
      done<=~(|(vmask));
    else
      done<=~( |(mask_buffered >> (2*4) ));
  end

  assign mul_opA=(done) ? opA : (opA_buffered >> 4*WIDTH);

  assign mul_opB=(done) ? opB : (opB_buffered >> 4*WIDTH);

  assign stall=~done && (ctrl_activate[2]);

  wire [3:1] oppipe_squash_NC; 
  wire [5*(2+1)-1:0] oppipe_q;

  assign {ctrl_op[3],ctrl_op[2],ctrl_op[1]} = oppipe_q;

  pipe_5_2 oppipe (
    .d(op),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(oppipe_squash_NC),
    .q(oppipe_q)
  );

  wire [3:1] activatepipe_squash_NC;
  wire [(2+1)-1:0]  activatepipe_q;

  assign {ctrl_activate[3],ctrl_activate[2],ctrl_activate[1]} = activatepipe_q;
  pipe_1_2  activatepipe (
    .d(activate),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(activatepipe_squash_NC),
    .q(activatepipe_q)
  );

  wire [3:1] vshamtpipe_squash_NC; 

//TODO: remove parameters

  pipe_5_2 vshamtpipe (
    .d(vshamt),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(vshamtpipe_squash_NC),
    //.q({ctrl_vshamt[3],ctrl_vshamt[2],ctrl_vshamt[1]}));
    .q(ctrl_vshamt));


//============== Instantiate across lanes =============                  
    wire vmul0_en;
    wire [4:0] vmul0_op;

    assign vmul0_en = en[1] | ~done;
    assign vmul0_op = (done&en[1]) ? ctrl_op[1] : ctrl_op[2];

    vlane_mulshift_32_5  vmul0(
      .clk(clk),
      .resetn(resetn),
      .en(vmul0_en),
      .opA(mul_opA[WIDTH*0 +: WIDTH]),
      .opB(mul_opB[WIDTH*0 +: WIDTH]),
      .sa( mul_opB[WIDTH*0+5-1 : WIDTH*0] ),
      .op(vmul0_op),
      .result(mul_result[WIDTH*0 +: WIDTH])
      );

    wire [5-1:0] vshift0_sa;
    wire [2-1:0] vshift0_op;

    assign vshift0_sa = ctrl_vshamt[3*5-1:2*5];
    assign vshift0_op = {~ctrl_op[2][1] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift0(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(0+1)-1:WIDTH*0]),
      .sa(vshift0_sa),
      .op(vshift0_op),
      .result(rshift_result[WIDTH*(0+1)-1:WIDTH*0])
      );

        

    wire vmul1_en;
    wire [4:0] vmul1_op;

    assign vmul1_en = en[1] | ~done;
    assign vmul1_op = (done&en[1]) ? ctrl_op[1] : ctrl_op[2];

    vlane_mulshift_32_5  vmul1(
      .clk(clk),
      .resetn(resetn),
      .en(vmul1_en),
      .opA(mul_opA[WIDTH*1 +: WIDTH]),
      .opB(mul_opB[WIDTH*1 +: WIDTH]),
      .sa( mul_opB[WIDTH*1+5-1 : WIDTH*1] ),
      .op(vmul1_op),
      .result(mul_result[WIDTH*1 +: WIDTH])
      );

    wire [5-1:0] vshift1_sa;
    wire [2-1:0] vshift1_op;

    assign vshift1_sa = ctrl_vshamt[3*5-1:2*5];
    assign vshift1_op = {~ctrl_op[2][1] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift1(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(1+1)-1:WIDTH*1]),
      .sa(vshift1_sa),
      .op(vshift1_op),
      .result(rshift_result[WIDTH*(1+1)-1:WIDTH*1])
      );

        

    wire vmul2_en;
    wire [4:0] vmul2_op;

    assign vmul2_en = en[1] | ~done;
    assign vmul2_op = (done&en[1]) ? ctrl_op[1] : ctrl_op[2];

    vlane_mulshift_32_5  vmul2(
      .clk(clk),
      .resetn(resetn),
      .en(vmul2_en),
      .opA(mul_opA[WIDTH*2 +: WIDTH]),
      .opB(mul_opB[WIDTH*2 +: WIDTH]),
      .sa( mul_opB[WIDTH*2+5-1 : WIDTH*2] ),
      .op(vmul2_op),
      .result(mul_result[WIDTH*2 +: WIDTH])
      );

    wire [5-1:0] vshift2_sa;
    wire [2-1:0] vshift2_op;

    assign vshift2_sa = ctrl_vshamt[3*5-1:2*5];
    assign vshift2_op = {~ctrl_op[2][1] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift2(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(2+1)-1:WIDTH*2]),
      .sa(vshift2_sa),
      .op(vshift2_op),
      .result(rshift_result[WIDTH*(2+1)-1:WIDTH*2])
      );

        

    wire vmul3_en;
    wire [4:0] vmul3_op;

    assign vmul3_en = en[1] | ~done;
    assign vmul3_op = (done&en[1]) ? ctrl_op[1] : ctrl_op[2];

    vlane_mulshift_32_5  vmul3(
      .clk(clk),
      .resetn(resetn),
      .en(vmul3_en),
      .opA(mul_opA[WIDTH*3 +: WIDTH]),
      .opB(mul_opB[WIDTH*3 +: WIDTH]),
      .sa( mul_opB[WIDTH*3+5-1 : WIDTH*3] ),
      .op(vmul3_op),
      .result(mul_result[WIDTH*3 +: WIDTH])
      );

    wire [5-1:0] vshift3_sa;
    wire [2-1:0] vshift3_op;

    assign vshift3_sa = ctrl_vshamt[3*5-1:2*5];
    assign vshift3_op = {~ctrl_op[2][1] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift3(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(3+1)-1:WIDTH*3]),
      .sa(vshift3_sa),
      .op(vshift3_op),
      .result(rshift_result[WIDTH*(3+1)-1:WIDTH*3])
      );

        

//TO DO: parameters

  //Shift Register for all multiplier results
  wire [4*WIDTH-1:0] shiftin_right_result_elmshifter_NC;
  wire [NUMLANES*WIDTH-1:0] inpipe_result_elmshifter_NC;
  wire shift_result_elmshifter;
  assign shift_result_elmshifter = ~done;

  velmshifter_1_128  result_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(1'b0),
    .shift(shift_result_elmshifter),
    .dir_left(1'b0),
    .squash(1'b0),
    //Enable/Disable rshifter for fixed-point multiply support
    //.shiftin_left(rshift_result),
    .shiftin_left(mul_result),
    .shiftin_right(shiftin_right_result_elmshifter_NC),
    .inpipe(inpipe_result_elmshifter_NC),
    .outpipe(result_buffered)
  );

  //Enable/Disable rshifter for fixed-point multiply support
  //assign result_tmp=rshift_result;
  assign result_tmp=mul_result;

  assign result=(result_tmp<<((NUMLANES/4-1)*4*WIDTH)) |
                (result_buffered>>4*WIDTH);

  wire [2:1] dstpipe_squash_NC;
  wire [2-1:0] dstpipe_en;
  assign dstpipe_en = en[2:1] & {1'b1,~stall};

  pipe_10_2 dstpipe (
    .d(in_dst),  
    .clk(clk),
    .resetn(resetn),
    .en(dstpipe_en),
    .squash(dstpipe_squash_NC),
    .q(out_dst));

  wire [2-1:0] dstwepipe_en;
  assign dstwepipe_en = en[2:1] & {1'b1,~stall};

  pipe_1_2 dstwepipe (
    .d(in_dst_we),  
    .clk(clk),
    .resetn(resetn),
    .en(dstwepipe_en),
    .squash(squash[2:1]),
    .q(out_dst_we));

  wire [2-1:0] dstmaskpipe_en;
  assign dstmaskpipe_en = en[2:1] & {1'b1,~stall};

  wire [2:1] dstmaskpipe_squash_NC;
  pipe_4_2 dstmaskpipe (
    .d(vmask),  
    .clk(clk),
    .resetn(resetn),
    .en(dstmaskpipe_en),
    .squash(dstmaskpipe_squash_NC),
    .q(out_dst_mask));


endmodule

/****************************************************************************
          MUL unit

Operation table

     half sat  op unsign dir
2      0   0   0   0    0    |  Zero (This value is used to reset multiplier)
14     1   0   1   1    0    |  LMULU Lower MULtiply Unsigned
15     1   0   1   1    1    |  UMULU Upper MULtiply Unsigned
16     1   0   1   0    0    |  LMUL Lower MULtiply signed
17     1   0   1   0    1    |  UMUL Upper MULtiply signed
4      0   0   1   1    0    |  MULLOU
4      0   0   1   1    1    |  MULHIU
6      0   0   1   0    0    |  MULLO
6      0   0   1   0    1    |  MULHI
0      0   0   0   1    0    |  ShiftLeft
8      0   1   0   1    0    |  ShiftLeftSatU
10     0   1   0   0    0    |  ShiftLeftSat
1      0   0   0   1    1    |  ShiftRightLogic
3      0   0   0   0    1    |  ShiftRightArith
****************************************************************************/
module vlane_mulshift_32_5(clk, resetn,
            opA, opB, sa,
            op,
            en,
            result
            );
parameter WIDTH=32;
parameter LOG2WIDTH=5;

input clk;
input resetn;

input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
input [LOG2WIDTH-1:0] sa;
input [4:0] op;
input [3:1] en;  //Enable for each pipestage

output [WIDTH-1:0] result;

/********* Control Signals *********/
wire is_signed, dir, is_mul, saturate, half;
assign is_mul=op[2];      // selects between opB and the computed shift amount
assign is_signed=~op[1];
assign dir=op[0];         // selects between 2^sa and 2^(32-sa) for right shift
assign saturate=op[3];
assign half=op[4];

/********* Circuit Body *********/
wire dum,dum2,dum3;
wire [WIDTH:0] opB_mux_out;
wire [WIDTH-1:0] opA_mux_out;
wire [WIDTH-1:0] opB_mul;
wire [5-1:0] left_sa;     // Amount of left shift required for both left/right
reg [WIDTH:0] decoded_sa;
wire [WIDTH-1:0] hi;
wire [WIDTH-1:0] lo;

assign opA_mux_out = (~half) ? ( opA ) : (WIDTH<2) ? 0 : (dir) ?
              {{WIDTH/2{is_signed&is_mul&opA[WIDTH-1]}},opA[WIDTH-1:WIDTH/2]} :
              {{WIDTH/2{is_signed&is_mul&opA[WIDTH/2-1]}},opA[WIDTH/2-1:0]};

assign opB_mul = (~half) ? opB : (WIDTH<2) ? 0 : (dir) ? 
              {{WIDTH/2{is_signed&is_mul&opB[WIDTH-1]}},opB[WIDTH-1:WIDTH/2]} :
              {{WIDTH/2{is_signed&is_mul&opB[WIDTH/2-1]}},opB[WIDTH/2-1:0]};

assign opB_mux_out=(is_mul) ? {is_signed&opB_mul[WIDTH-1],opB_mul} : decoded_sa;

reg zeroout;

`ifndef USE_INHOUSE_LOGIC
  `define USE_INHOUSE_LOGIC
`endif

always@(posedge clk)
  if (en[1])
    zeroout<=(op[3:0]==0);

`ifdef USE_INHOUSE_LOGIC
wire [(WIDTH+1)-1:0] dataa;
wire aclr;
wire [66-1:0] local_mult_component_result;

assign dataa = {is_signed&opA_mux_out[WIDTH-1],opA_mux_out};
assign aclr = ~resetn;
assign {dum2,dum,hi,lo} = local_mult_component_result;

local_mult_33_33_66 local_mult_component (
.dataa(dataa),
.datab(opB_mux_out),
.clock(clk),
.clken(en[1]),
.aclr(aclr),
.result(local_mult_component_result)
);
`else 
lpm_mult  lpm_mult_component (
  .dataa ({is_signed&opA_mux_out[WIDTH-1],opA_mux_out}),
  .datab (opB_mux_out),
  .sum(),
  .clock(clk),
  .clken(en[1]),
  .aclr(~resetn),
  .result ({dum2,dum,hi,lo}));
defparam
  lpm_mult_component.lpm_widtha = WIDTH+1,
  lpm_mult_component.lpm_widthb = WIDTH+1,
  lpm_mult_component.lpm_widthp = 2*WIDTH+2,
  lpm_mult_component.lpm_widths = 1,
  lpm_mult_component.lpm_pipeline = 1,
  lpm_mult_component.lpm_type = "LPM_MULT",
  lpm_mult_component.lpm_representation = "SIGNED",
  lpm_mult_component.lpm_hint = "MAXIMIZE_SPEED=6";
`endif
// if A is positive/negative make it maximum/minimum positive/negative
wire [WIDTH-1:0] signedsaturate=
                    (opA_mux_out[WIDTH-1]) ? {1'b1,{WIDTH-1{1'b0}}} : 
                                             {1'b0,{WIDTH-1{1'b1}}};

reg [WIDTH-1:0] saturatedval_s2;
reg saturate_s2;

//Capture saturated value and saturate signal for next stage
always@(posedge clk)
  if (!resetn)
  begin
    saturatedval_s2<=0;
    saturate_s2<=0;
  end
  else if (en[1])
  begin
    saturatedval_s2<=((~is_signed) ? {WIDTH{1'b1}} : signedsaturate);
    saturate_s2<=saturate;
  end

reg sel_hi;

always@(posedge clk)
  if (!resetn)
    sel_hi<=0;
  else if (en[1])
    sel_hi<=(is_mul && dir || ~is_mul && dir && |sa);

assign result =(zeroout) ? 0 : 
                (saturate_s2 && |hi ) ? saturatedval_s2 : 
                (sel_hi) ? hi : lo;


assign {dum3, left_sa}= (dir) ? WIDTH-sa : {1'b0,sa};

//Decoder - computes 2^left_sa
always@*
begin
    decoded_sa=1<<left_sa;
end
endmodule
/******************************************************************************

0  VFAND
1  VFOR
2  VFXOR
3  VFNOR
4  VFCLR
5  VFSET

Not handled here:
  VIOTA
  VCIOTA
  VFPOP
  VFFF1
  VFFL1
  VFSETBF
  VFSETIF
  VFSETOF

******************************************************************************/

module vlane_flagalu (
    clk,
    resetn,

    src1,
    src2,

    op,

    result

    );

input clk;
input resetn;

input src1;
input src2;
input [2:0] op;

output result;

reg result;

  always@*
    case(op)
      0: result=src1&src2;
      1: result=src1|src2;
      2: result=src1^src2;
      3: result=~(src1|src2);
      4: result=1'b0;
      5: result=1'b1;
      default: result=1'b0;
    endcase

endmodule
module processing_element(
 reset, 
 clk, 
 in_a,
 in_b, 
 out_a, 
 out_b, 
 out_c
 );

 input reset;
 input clk;
 input  [`DWIDTH-1:0] in_a;
 input  [`DWIDTH-1:0] in_b;
 output [`DWIDTH-1:0] out_a;
 output [`DWIDTH-1:0] out_b;
 output [`DWIDTH-1:0] out_c;  //reduced precision

 reg [`DWIDTH-1:0] out_a;
 reg [`DWIDTH-1:0] out_b;
 wire [`DWIDTH-1:0] out_c;

 wire [`DWIDTH-1:0] out_mac;

 assign out_c = out_mac;

 seq_mac u_mac(.a(in_a), .b(in_b), .out(out_mac), .reset(reset), .clk(clk));

 always @(posedge clk)begin
    if(reset) begin
      out_a<=0;
      out_b<=0;
    end
    else begin  
      out_a<=in_a;
      out_b<=in_b;
    end
 end
 
endmodule

module bfloat_mult_10 (
 input clk,
 input resetn,
 input en,
 input stall,
 input [10-1:0] a,
 input [10-1:0] b,
 output reg[10-1:0] out
);

always@(posedge clk)begin
  if(!resetn)
    out <= 'h0;
  else
    if(en)
      out <= a * b;
end

endmodule
        
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_1_2(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input  d;
input              clk;
input              resetn;
input  [2-1:0] en;
input  [2-1:0] squash;
output [(2+1)-1:0] q;

reg [2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq <= 0;
    else if (en[0])
      tq <=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i]<= 0;
      else if (en[i])
        tq[i]<=tq[(i-1)];
  end

  assign q[0] =d;
  assign q[(2+1)-1:1]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_5_2(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [5-1:0]  d;
input              clk;
input              resetn;
input  [2-1:0] en;
input  [2-1:0] squash;
output [5*(2+1)-1:0] q;

reg [5*2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 5-1:0 ]<= 0;
    else if (en[0])
      tq[ 5-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i*5 +: 5 ]<= 0;
      else if (en[i])
        tq[i*5 +: 5 ]<=tq[(i-1)*5 +: 5 ];
  end

  assign q[5-1:0]=d;
  assign q[5*(2+1)-1:5]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_10_2(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [10-1:0]  d;
input              clk;
input              resetn;
input  [2-1:0] en;
input  [2-1:0] squash;
output [10*(2+1)-1:0] q;

reg [10*2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 10-1:0 ]<= 0;
    else if (en[0])
      tq[ 10-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i*10 +: 10 ]<= 0;
      else if (en[i])
        tq[i*10 +: 10 ]<=tq[(i-1)*10 +: 10 ];
  end

  assign q[10-1:0]=d;
  assign q[10*(2+1)-1:10]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_4_2(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [4-1:0]  d;
input              clk;
input              resetn;
input  [2-1:0] en;
input  [2-1:0] squash;
output [4*(2+1)-1:0] q;

reg [4*2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 4-1:0 ]<= 0;
    else if (en[0])
      tq[ 4-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i*4 +: 4 ]<= 0;
      else if (en[i])
        tq[i*4 +: 4 ]<=tq[(i-1)*4 +: 4 ];
  end

  assign q[4-1:0]=d;
  assign q[4*(2+1)-1:4]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_7_5(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [7-1:0]  d;
input              clk;
input              resetn;
input  [5-1:0] en;
input  [5-1:0] squash;
output [7*(5+1)-1:0] q;

reg [7*5-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 7-1:0 ]<= 0;
    else if (en[0])
      tq[ 7-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<5; i=i+1)
      if (!resetn || squash[i] )
        tq[i*7 +: 7 ]<= 0;
      else if (en[i])
        tq[i*7 +: 7 ]<=tq[(i-1)*7 +: 7 ];
  end

  assign q[7-1:0]=d;
  assign q[7*(5+1)-1:7]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_32_4(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [32-1:0]  d;
input              clk;
input              resetn;
input  [4-1:0] en;
input  [4-1:0] squash;
output [32*(4+1)-1:0] q;

reg [32*4-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 32-1:0 ]<= 0;
    else if (en[0])
      tq[ 32-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<4; i=i+1)
      if (!resetn || squash[i] )
        tq[i*32 +: 32 ]<= 0;
      else if (en[i])
        tq[i*32 +: 32 ]<=tq[(i-1)*32 +: 32 ];
  end

  assign q[32-1:0]=d;
  assign q[32*(4+1)-1:32]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_10_1(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [10-1:0]  d;
input              clk;
input              resetn;
input en;
input squash;
output [10*(1+1)-1:0] q;

reg [10*1-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 10-1:0 ]<= 0;
    else if (en[0])
      tq[ 10-1:0 ]<=d;
  end

  assign q[10-1:0]=d;
  assign q[10*(1+1)-1:10]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_1_3(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input d;
input              clk;
input              resetn;
input en;
input squash;
output [1:0] q;

reg  tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq <= 0;
    else if (en[0])
      tq <=d;
  end

  assign q[0] = d;
  assign q[1]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_8_1(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [8-1:0]  d;
input              clk;
input              resetn;
input en;
input squash;
output [8*(1+1)-1:0] q;

reg [8*1-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 8-1:0 ]<= 0;
    else if (en[0])
      tq[ 8-1:0 ]<=d;
  end

  assign q[8-1:0]=d;
  assign q[8*(1+1)-1:8]=tq;
endmodule
        
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_1_32(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input  d;
input              clk;
input              resetn;
input  [32-1:0] en;
input  [32-1:0] squash;
output [(32+1)-1:0] q;

reg [32-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq <= 0;
    else if (en[0])
      tq <=d;

    // All the rest registers
    for (i=1; i<32; i=i+1)
      if (!resetn || squash[i] )
        tq[i]<= 0;
      else if (en[i])
        tq[i]<=tq[(i-1)];
  end

  assign q[0] =d;
  assign q[(32+1)-1:1]=tq;
endmodule
        

/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_10_32(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [10-1:0]  d;
input              clk;
input              resetn;
input  [32-1:0] en;
input  [32-1:0] squash;
output [10*(32+1)-1:0] q;

reg [10*32-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 10-1:0 ]<= 0;
    else if (en[0])
      tq[ 10-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<32; i=i+1)
      if (!resetn || squash[i] )
        tq[i*10 +: 10 ]<= 0;
      else if (en[i])
        tq[i*10 +: 10 ]<=tq[(i-1)*10 +: 10 ];
  end

  assign q[10-1:0]=d;
  assign q[10*(32+1)-1:10]=tq;
endmodule
        

/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_4_32(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [4-1:0]  d;
input              clk;
input              resetn;
input  [32-1:0] en;
input  [32-1:0] squash;
output [4*(32+1)-1:0] q;

reg [4*32-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 4-1:0 ]<= 0;
    else if (en[0])
      tq[ 4-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<32; i=i+1)
      if (!resetn || squash[i] )
        tq[i*4 +: 4 ]<= 0;
      else if (en[i])
        tq[i*4 +: 4 ]<=tq[(i-1)*4 +: 4 ];
  end

  assign q[4-1:0]=d;
  assign q[4*(32+1)-1:4]=tq;
endmodule
        
/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_1(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [1-1:0] d;
output [1-1:0] q;
reg [1-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_17(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [17-1:0] d;
output [17-1:0] q;
reg [17-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_1(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [1-1:0] d;
output [1-1:0] q;
reg [1-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_8(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [8-1:0] d;
output [8-1:0] q;
reg [8-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_72(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [72-1:0] d;
output [72-1:0] q;
reg [72-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule
module local_mult_33_33_66(
dataa,
datab,
clock,
clken,
aclr,
result
);

input [33-1:0] dataa;
input [33-1:0] datab;
input clock;
input clken;
input aclr;
output reg [66-1:0] result;

wire [33-1:0] unsignedinputA;
wire [33-1:0] unsignedinputB;
wire [66-1:0] unsignedoutputP;

wire gated_clock;

assign unsignedinputA = dataa;
assign unsignedinputB = datab;

assign unsignedoutputP = unsignedinputA * unsignedinputB;

assign gated_clock = clock & clken;

always @(posedge gated_clock)begin
    if(aclr)begin
       result <= 0;
    end
    else
       result <= unsignedoutputP; 
end
endmodule
///////////////////////////////////////////////////////
// Reduction unit. It's a tree of processing elements.
// There are 32 inputs and one output and 6 stages. 
//
// The output is
// wider (more bits) than the inputs. It has logN more
// bits (if N is the number of bits in the inputs). This
// is based on https://zipcpu.com/dsp/2017/07/22/rounding.html.
// 
// The last stage is special. It adds the previous 
// result. This is useful when we have more than 32 inputs
// to reduce. We send the next set of 32 inputs in the next
// clock after the first set. 
// 
// Each stage of the tree is pipelined.
///////////////////////////////////////////////////////
module reduction_unit_10_3(
  clk,
  reset,

  inp0, 
  inp1, 
  inp2, 
  inp3, 
  inp4, 
  inp5, 
  inp6, 
  inp7, 

  mode,
  outp
);

  input clk;
  input reset;
  input  [10-1 : 0] inp0; 
  input  [10-1 : 0] inp1; 
  input  [10-1 : 0] inp2; 
  input  [10-1 : 0] inp3; 
  input  [10-1 : 0] inp4; 
  input  [10-1 : 0] inp5; 
  input  [10-1 : 0] inp6; 
  input  [10-1 : 0] inp7; 

  input [1:0] mode;
  output [10+3-1 : 0] outp;

  wire   [10+3-1 : 0] compute0_out_stage4;
  reg    [10+3-1 : 0] compute0_out_stage4_reg;
  wire   [10+3-1 : 0] compute1_out_stage4;
  reg    [10+3-1 : 0] compute1_out_stage4_reg;
  wire   [10+3-1 : 0] compute2_out_stage4;
  reg    [10+3-1 : 0] compute2_out_stage4_reg;
  wire   [10+3-1 : 0] compute3_out_stage4;
  wire   [10+3-1 : 0] compute3_out_stage4_reg;
  wire   [10+3-1 : 0] compute4_out_stage4;
  reg    [10+3-1 : 0] compute4_out_stage4_reg;
  wire   [10+3-1 : 0] compute5_out_stage4;
  reg    [10+3-1 : 0] compute5_out_stage4_reg;
  wire   [10+3-1 : 0] compute6_out_stage4;
  reg    [10+3-1 : 0] compute6_out_stage4_reg;
  wire   [10+3-1 : 0] compute7_out_stage4;
  wire   [10+3-1 : 0] compute7_out_stage4_reg;

  wire   [10+3-1 : 0] compute0_out_stage3;
  reg    [10+3-1 : 0] compute0_out_stage3_reg;
  wire   [10+3-1 : 0] compute1_out_stage3;
  reg    [10+3-1 : 0] compute1_out_stage3_reg;
  wire   [10+3-1 : 0] compute2_out_stage3;
  reg    [10+3-1 : 0] compute2_out_stage3_reg;
  wire   [10+3-1 : 0] compute3_out_stage3;
  wire   [10+3-1 : 0] compute3_out_stage3_reg;

  wire   [10+3-1 : 0] compute0_out_stage2;
  reg    [10+3-1 : 0] compute0_out_stage2_reg;
  wire   [10+3-1 : 0] compute1_out_stage2;
  reg    [10+3-1 : 0] compute1_out_stage2_reg;

  wire   [10+3-1 : 0] compute0_out_stage1;
  reg    [10+3-1 : 0] compute0_out_stage1_reg;

  wire   [10+3-1 : 0] compute0_out_stage0;
  reg    [10+3-1 : 0] outp;

  always @(posedge clk) begin
    if (reset) begin
      outp <= 0;
      compute0_out_stage3_reg <= 0;
      compute1_out_stage3_reg <= 0;
      compute2_out_stage3_reg <= 0;
      compute3_out_stage3_reg <= 0;
      compute0_out_stage2_reg <= 0;
      compute1_out_stage2_reg <= 0;
      compute0_out_stage1_reg <= 0;
    end

    else begin
      compute0_out_stage3_reg <= compute0_out_stage3;
      compute1_out_stage3_reg <= compute1_out_stage3;
      compute2_out_stage3_reg <= compute2_out_stage3;
      compute3_out_stage3_reg <= compute3_out_stage3;

      compute0_out_stage2_reg <= compute0_out_stage2;
      compute1_out_stage2_reg <= compute1_out_stage2;

      compute0_out_stage1_reg <= compute0_out_stage1;

      outp <= compute0_out_stage0;
    end
  end
  reduction_processing_element_10_13 compute0_stage3(.A(compute0_out_stage4_reg),       .B(compute1_out_stage4_reg),    .OUT(compute0_out_stage3), .MODE(mode));
  reduction_processing_element_10_13 compute1_stage3(.A(compute2_out_stage4_reg),       .B(compute3_out_stage4_reg),    .OUT(compute1_out_stage3), .MODE(mode));
  reduction_processing_element_10_13 compute2_stage3(.A(compute4_out_stage4_reg),       .B(compute5_out_stage4_reg),    .OUT(compute2_out_stage3), .MODE(mode));
  reduction_processing_element_10_13 compute3_stage3(.A(compute6_out_stage4_reg),       .B(compute7_out_stage4_reg),    .OUT(compute3_out_stage3), .MODE(mode));

  reduction_processing_element_13_13 compute0_stage2(.A(compute0_out_stage3_reg),       .B(compute1_out_stage3_reg),    .OUT(compute0_out_stage2), .MODE(mode));
  reduction_processing_element_13_13 compute1_stage2(.A(compute2_out_stage3_reg),       .B(compute3_out_stage3_reg),    .OUT(compute1_out_stage2), .MODE(mode));

  reduction_processing_element_13_13 compute0_stage1(.A(compute0_out_stage2_reg),       .B(compute1_out_stage2_reg),    .OUT(compute0_out_stage1), .MODE(mode));

  reduction_processing_element_13_13 compute0_stage0(.A(outp),       .B(compute0_out_stage1_reg),     .OUT(compute0_out_stage0), .MODE(mode));

endmodule


/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_4_32 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 4-1:0 ]  squash;

input [ 4*32-1:0 ]  inpipe;
output [ 4*32-1:0 ] outpipe;

input [ 32-1:0 ]  shiftin_left;
input [ 32-1:0 ]  shiftin_right;

wire [ (4+1)*32-1:0 ] _outpipe;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_32 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[31:0],
      _outpipe[63:32], //Support 1 lane
      shiftin_right,
      _outpipe[31:0]);
 // defparam velmshifter_laneunit0.32=32;

  //Generate everything in between 


      velmshifter_laneunit_32 velmshifter_laneunit_1(clk,resetn,load,shift,dir_left,
          squash[1],
          inpipe[63:32],
          _outpipe[95:64],
          _outpipe[31:0],
          _outpipe[63:32]);
     // defparam velmshifter_laneunit.32=32;


      velmshifter_laneunit_32 velmshifter_laneunit_2(clk,resetn,load,shift,dir_left,
          squash[2],
          inpipe[63:32],
          _outpipe[95:64],
          _outpipe[31:0],
          _outpipe[63:32]);
     // defparam velmshifter_laneunit.32=32;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_32 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[3],
      inpipe[127:96],
      shiftin_left,
      _outpipe[95:64], //L=1
      _outpipe[127:96]); //L=1
   // defparam velmshifter_laneunitlast.32=32;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_4_1 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 4-1:0 ]  squash;

input [ 4*1-1:0 ]  inpipe;
output [ 4*1-1:0 ] outpipe;

input [ 1-1:0 ]  shiftin_left;
input [ 1-1:0 ]  shiftin_right;

wire [ (4+1)*1-1:0 ] _outpipe;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_1 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[0],
      _outpipe[1], //Support 1 lane
      shiftin_right,
      _outpipe[0]);
 // defparam velmshifter_laneunit0.1=1;

  //Generate everything in between 


      velmshifter_laneunit_1 velmshifter_laneunit_1(clk,resetn,load,shift,dir_left,
          squash[1],
          inpipe[1],
          _outpipe[2],
          _outpipe[0],
          _outpipe[1]);
     // defparam velmshifter_laneunit.1=1;


      velmshifter_laneunit_1 velmshifter_laneunit_2(clk,resetn,load,shift,dir_left,
          squash[2],
          inpipe[1],
          _outpipe[2],
          _outpipe[0],
          _outpipe[1]);
     // defparam velmshifter_laneunit.1=1;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_1 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[3],
      inpipe[3],
      shiftin_left,
      _outpipe[2], //L=1
      _outpipe[3]); //L=1
   // defparam velmshifter_laneunitlast.1=1;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_1_128 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 1-1:0 ]  squash;

input [ 1*128-1:0 ]  inpipe;
output [ 1*128-1:0 ] outpipe;

input [ 128-1:0 ]  shiftin_left;
input [ 128-1:0 ]  shiftin_right;

wire [ (1+1)*128-1:0 ] _outpipe;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_128 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[127:0],
      _outpipe[255:128], //Support 1 lane
      shiftin_right,
      _outpipe[127:0]);
 // defparam velmshifter_laneunit0.128=128;

  //Generate everything in between 


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_128 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[127:0],
      shiftin_left,
      _outpipe[-1:-128], //L=1
      _outpipe[127:0]); //L=1
   // defparam velmshifter_laneunitlast.128=128;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_1_4 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 1-1:0 ]  squash;

input [ 1*4-1:0 ]  inpipe;
output [ 1*4-1:0 ] outpipe;

input [ 4-1:0 ]  shiftin_left;
input [ 4-1:0 ]  shiftin_right;

wire [ (1+1)*4-1:0 ] _outpipe;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_4 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[3:0],
      _outpipe[7:4], //Support 1 lane
      shiftin_right,
      _outpipe[3:0]);
 // defparam velmshifter_laneunit0.4=4;

  //Generate everything in between 


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_4 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[3:0],
      shiftin_left,
      _outpipe[-1:-4], //L=1
      _outpipe[3:0]); //L=1
   // defparam velmshifter_laneunitlast.4=4;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        

module vdispatcher_2_157_7_7_3(
    clk,
    resetn,

    shift,
    rotate, //shift must also be high to rotate

    inshift_instr,
    inshift_first,
    inshift_rdelm,
    inshift_wrelm,
    inshift_count,

    increment,
    rdelm_add_sub,
    wrelm_add_sub,
    count_add_sub,
    rdelm_valuetoadd,
    wrelm_valuetoadd,
    count_valuetoadd,

    instr,
    first,
    rdelm,
    wrelm,
    count

    );

input clk;
input resetn;

input  shift;
input  rotate;

input [ 157-1:0 ] inshift_instr;
input                    inshift_first;
input [ 7-1:0 ] inshift_rdelm;
input [ 7-1:0 ] inshift_wrelm;
input [ 3-1:0 ] inshift_count;

input [ 2-1:0 ] increment;

input rdelm_add_sub;
input wrelm_add_sub;
input count_add_sub;
input [ 7-1:0 ] rdelm_valuetoadd;
input [ 7-1:0 ] wrelm_valuetoadd;
input [ 3-1:0 ] count_valuetoadd;

output [ 2*157-1:0 ] instr;
output            [ 2-1:0 ] first;
output [ 2*7-1:0 ] rdelm;
output [ 2*7-1:0 ] wrelm;
output [ 2*3-1:0 ] count;

wire [2*157-1:0 ] inparallel_data_inst_NC;
vdispatcher_shift_2_157 vdispatcher_instr (
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .squash({2{1'b0}}),
      .inshift_data(inshift_instr),
      .inparallel_data(inparallel_data_inst_NC),
      .outparallel_data(instr));

wire [ 2-1:0 ] inparallel_data_first_NC;
vdispatcher_shift_2_1 vdispatcher_first (
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .squash(increment&~{32'b0,shift&~rotate}),
      .inshift_data(inshift_first),
      .inparallel_data(inparallel_data_first_NC),
      .outparallel_data(first));

wire [ 2*7-1:0 ] inparallel_data_rdelm_NC;
wire [ 2-1:0 ] squash_rdelm_NC;
vdispatcher_add_2_7 vdispatcher_rdelm(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_rdelm_NC),
      .add_sub(rdelm_add_sub),
      .valuetoadd(rdelm_valuetoadd),
      .inshift_data(inshift_rdelm),
      .inparallel_data(inparallel_data_rdelm_NC),
      .outparallel_data(rdelm));

wire [ 2*7-1:0 ] inparallel_data_wrelm_NC;
wire [ 2-1:0 ] squash_wrelm_NC;
vdispatcher_add_2_7 vdispatcher_wrelm(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_wrelm_NC),
      .add_sub(wrelm_add_sub),
      .valuetoadd(wrelm_valuetoadd),
      .inshift_data(inshift_wrelm),
      .inparallel_data(inparallel_data_wrelm_NC),
      .outparallel_data(wrelm));

wire [ 2*3-1:0 ] inparallel_data_count_NC;
wire [ 2-1:0 ] squash_count_NC;
vdispatcher_add_2_3 vdispatcher_count(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_count_NC),
      .add_sub(count_add_sub),
      .valuetoadd(count_valuetoadd),
      .inshift_data(inshift_count),
      .inparallel_data(inparallel_data_count_NC),
      .outparallel_data(count));


endmodule


/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_2_1 (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 2-1:0 ]  squash;

input [ 2*1-1:0 ]  inpipe;
output [ 2*1-1:0 ] outpipe;

input [ 1-1:0 ]  shiftin_left;
input [ 1-1:0 ]  shiftin_right;

wire [ (2+1)*1-1:0 ] _outpipe;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_1 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[0],
      _outpipe[1], //Support 1 lane
      shiftin_right,
      _outpipe[0]);
 // defparam velmshifter_laneunit0.1=1;

  //Generate everything in between 


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_1 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[1],
      inpipe[1],
      shiftin_left,
      _outpipe[0], //L=1
      _outpipe[1]); //L=1
   // defparam velmshifter_laneunitlast.1=1;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_flag_2_1_4_1024_10
  (clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_we);

input clk;
input resetn;

input [2-1:0] a_en;
input [2-1:0] b_en;
input [2-1:0] c_we;

input [2*9-1:0] a_reg,b_reg,c_reg;
output [2*4-1:0] a_readdataout, b_readdataout;
wire [2*4-1:0] a_temp, b_temp;
input [2*4-1:0] c_writedatain;


      ram_wrapper_9_512_4 reg_file1_0(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(a_en[0]),
          .address_a(c_reg[0* 9 +: 9]),
          .address_b(a_reg[0* 9 +: 9]),
          .wren_a(c_we[0]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ 0*4 +: 4]),
          .data_b(0),
          .out_a(a_temp[ 0*4 +: 4]),
          .out_b(a_readdataout[0*4 +: 4])
      );

      ram_wrapper_9_512_4 reg_file2_0(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(b_en[0]),
          .address_a(c_reg[0* 9 +: 9]),
          .address_b(b_reg[0* 9 +: 9]),
          .wren_a(c_we[0]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ 0*4 +: 4]),
          .data_b(0),
          .out_a(b_temp[ 0*4 +: 4]),
          .out_b(b_readdataout[ 0* 4 +: 4])
      );
      
      ram_wrapper_9_512_4 reg_file1_1(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(a_en[1]),
          .address_a(c_reg[1* 9 +: 9]),
          .address_b(a_reg[1* 9 +: 9]),
          .wren_a(c_we[1]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ 1*4 +: 4]),
          .data_b(0),
          .out_a(a_temp[ 1*4 +: 4]),
          .out_b(a_readdataout[1*4 +: 4])
      );

      ram_wrapper_9_512_4 reg_file2_1(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(b_en[1]),
          .address_a(c_reg[1* 9 +: 9]),
          .address_b(b_reg[1* 9 +: 9]),
          .wren_a(c_we[1]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ 1*4 +: 4]),
          .data_b(0),
          .out_a(b_temp[ 1*4 +: 4]),
          .out_b(b_readdataout[ 1* 4 +: 4])
      );
            
endmodule

/****************************************************************************
          Saturate unit for different widths given 32-bit input

  Only works for 32-bit inputs

  Interprets input as signed/unsigned, saturates to signed/unsigned 
  byte/half/word

  NOP (pass through) when size=word and signed==outsigned

   out
   sign sign size
    0    0    00    VSAT_U_W - NOP (pass through)
    0    0    01    VSAT_U_B
    0    0    10    VSAT_U_H
    1    1    00    VSAT_W - NOP (pass through)
    1    1    01    VSAT_B
    1    1    10    VSAT_H
    0    1    00    VSAT_SU_W
    0    1    01    VSAT_SU_B
    0    1    10    VSAT_SU_H

parameter
  SATSIZEOP_VSATUW=4'b0000,
  SATSIZEOP_VSATUB=4'b0001,
  SATSIZEOP_VSATUH=4'b0010,
  SATSIZEOP_VSATW  =4'b1100,
  SATSIZEOP_VSATB  =4'b1101,
  SATSIZEOP_VSATH  =4'b1110,
  SATSIZEOP_VSATSUW=4'b0100,
  SATSIZEOP_VSATSUB=4'b0101,
  SATSIZEOP_VSATSUH=4'b0110;

****************************************************************************/

module vlane_saturatesize(
    in,
    op,
    out
    );

parameter WIDTH=32;

input [WIDTH-1:0] in;
input [3:0] op;
output [WIDTH-1:0] out;

wire op_outsigned;
wire op_signed;
wire op_size;

reg [WIDTH-1:0] out;

  assign op_outsigned=op[3];
  assign op_signed=op[2];
  assign op_size=op[1:0]; //0 - word, 1 - byte, 2 - half

always@*
case(op_size)
    2'b01:  //byte
    case({op_signed,op_outsigned})
        2'b11:    // signed
        out = ((in[WIDTH-1])&(!(&in[WIDTH-2:7]))) ? {{WIDTH-8{1'b1}},8'h80} :
                ((~in[WIDTH-1]) && (|in[WIDTH-2:7])) ? 127 : in;
        2'b10:    // signed-unsigned
        out = (in[WIDTH-1]) ? 0 :
                (~in[WIDTH-1]&&(|in[WIDTH-2:8])) ? 255 : in;
        default:  //2'b00: unsigned
        out=(|in[WIDTH-1:8]) ? 255 : in;
        endcase
    2'b10:  //half-word 16-bits
    case({op_signed,op_outsigned})
        2'b11:    // signed
        out=((in[WIDTH-1])&(!(&in[WIDTH-2:15])))? {{WIDTH-16{1'b1}},16'h8000}:
                ((~in[WIDTH-1]) && (|in[WIDTH-2:15])) ? 32767 : in;
        2'b10:    // signed-unsigned
        out = (in[WIDTH-1]) ? 0 :
                (~in[WIDTH-1]&&(|in[WIDTH-2:16])) ? 65535 : in;
        default:  //2'b00: unsigned
        out=(|in[WIDTH-1:16]) ? 65535 : in;
        endcase
    default: 
    case({op_signed,op_outsigned})
        2'b10:    // signed-unsigned
        out = (in[WIDTH-1]) ? 0 : in;
        default:  //2'b00: unsigned
        out=in;
        endcase
endcase
  
endmodule
module transpose_10_2_1 (
  input clk,
  input resetn,
  input read,
  input en, 
  input [2 * 10 -1:0] a,
  output reg [2 * 10 -1:0] out,
  output reg busy
);

reg [2 * 10 - 1:0] data0;
reg [2 * 10 - 1:0] data1;
reg [2 * 10 - 1:0] data2;
reg [2 * 10 - 1:0] data3;
reg [2 * 10 - 1:0] data4;
reg [2 * 10 - 1:0] data5;
reg [2 * 10 - 1:0] data6;
reg [2 * 10 - 1:0] data7;
reg [1:0] count;
reg[31:0] i;
reg[31:0] j;
always@(posedge clk)begin
  if(!resetn)
    count <= 'h0;
  else begin
    if(en)
      count <= count + 1;
    else if(read)
      count <= count - 1 ;
  end
end

always@(posedge clk)begin
  if(!resetn)
     busy <= 1'b0;
  else
    if((count == 2-1) && en)
       busy <= 1'b1;
    else if((count == 'h1) && read)
       busy <= 1'b0; 
end

always @(posedge clk)begin
  if(!resetn)begin
    data0 <= 'h0;
    data1 <= 'h0;
    data2 <= 'h0;
    data3 <= 'h0;
    data4 <= 'h0;
    data5 <= 'h0;
    data6 <= 'h0;
    data7 <= 'h0;
    out <= 'h0;
  end
  else begin
    out <= data0;
    if(en)begin
          if(count==0)begin
              data0[0*10 +: 10] <= a[0*10 +: 10];
              data1[0*10 +: 10] <= a[1*10 +: 10];
              data2[0*10 +: 10] <= a[2*10 +: 10];
              data3[0*10 +: 10] <= a[3*10 +: 10];
              data4[0*10 +: 10] <= a[4*10 +: 10];
              data5[0*10 +: 10] <= a[5*10 +: 10];
              data6[0*10 +: 10] <= a[6*10 +: 10];
              data7[0*10 +: 10] <= a[7*10 +: 10];
          end   
          else if(count==1)begin
              data0[1*10 +: 10] <= a[0*10 +: 10];
              data1[1*10 +: 10] <= a[1*10 +: 10];
              data2[1*10 +: 10] <= a[2*10 +: 10];
              data3[1*10 +: 10] <= a[3*10 +: 10];
              data4[1*10 +: 10] <= a[4*10 +: 10];
              data5[1*10 +: 10] <= a[5*10 +: 10];
              data6[1*10 +: 10] <= a[6*10 +: 10];
              data7[1*10 +: 10] <= a[7*10 +: 10];
          end   
          else if(count==2)begin
              data0[2*10 +: 10] <= a[0*10 +: 10];
              data1[2*10 +: 10] <= a[1*10 +: 10];
              data2[2*10 +: 10] <= a[2*10 +: 10];
              data3[2*10 +: 10] <= a[3*10 +: 10];
              data4[2*10 +: 10] <= a[4*10 +: 10];
              data5[2*10 +: 10] <= a[5*10 +: 10];
              data6[2*10 +: 10] <= a[6*10 +: 10];
              data7[2*10 +: 10] <= a[7*10 +: 10];
          end   
          else if(count==3)begin
              data0[3*10 +: 10] <= a[0*10 +: 10];
              data1[3*10 +: 10] <= a[1*10 +: 10];
              data2[3*10 +: 10] <= a[2*10 +: 10];
              data3[3*10 +: 10] <= a[3*10 +: 10];
              data4[3*10 +: 10] <= a[4*10 +: 10];
              data5[3*10 +: 10] <= a[5*10 +: 10];
              data6[3*10 +: 10] <= a[6*10 +: 10];
              data7[3*10 +: 10] <= a[7*10 +: 10];
          end   
          else if(count==4)begin
              data0[4*10 +: 10] <= a[0*10 +: 10];
              data1[4*10 +: 10] <= a[1*10 +: 10];
              data2[4*10 +: 10] <= a[2*10 +: 10];
              data3[4*10 +: 10] <= a[3*10 +: 10];
              data4[4*10 +: 10] <= a[4*10 +: 10];
              data5[4*10 +: 10] <= a[5*10 +: 10];
              data6[4*10 +: 10] <= a[6*10 +: 10];
              data7[4*10 +: 10] <= a[7*10 +: 10];
          end   
          else if(count==5)begin
              data0[5*10 +: 10] <= a[0*10 +: 10];
              data1[5*10 +: 10] <= a[1*10 +: 10];
              data2[5*10 +: 10] <= a[2*10 +: 10];
              data3[5*10 +: 10] <= a[3*10 +: 10];
              data4[5*10 +: 10] <= a[4*10 +: 10];
              data5[5*10 +: 10] <= a[5*10 +: 10];
              data6[5*10 +: 10] <= a[6*10 +: 10];
              data7[5*10 +: 10] <= a[7*10 +: 10];
          end   
          else if(count==6)begin
              data0[6*10 +: 10] <= a[0*10 +: 10];
              data1[6*10 +: 10] <= a[1*10 +: 10];
              data2[6*10 +: 10] <= a[2*10 +: 10];
              data3[6*10 +: 10] <= a[3*10 +: 10];
              data4[6*10 +: 10] <= a[4*10 +: 10];
              data5[6*10 +: 10] <= a[5*10 +: 10];
              data6[6*10 +: 10] <= a[6*10 +: 10];
              data7[6*10 +: 10] <= a[7*10 +: 10];
          end   
          else if(count==7)begin
              data0[7*10 +: 10] <= a[0*10 +: 10];
              data1[7*10 +: 10] <= a[1*10 +: 10];
              data2[7*10 +: 10] <= a[2*10 +: 10];
              data3[7*10 +: 10] <= a[3*10 +: 10];
              data4[7*10 +: 10] <= a[4*10 +: 10];
              data5[7*10 +: 10] <= a[5*10 +: 10];
              data6[7*10 +: 10] <= a[6*10 +: 10];
              data7[7*10 +: 10] <= a[7*10 +: 10];
          end   
    end
    else if(busy & read)begin
        data0 <= data1;
        data1 <= data2;
        data2 <= data3;
        data3 <= data4;
        data4 <= data5;
        data5 <= data6;
        data6 <= data7;
        data7 <= 0;
    end
    else begin
        data0 <= data0;
        data1 <= data1;
        data2 <= data2;
        data3 <= data3;
        data4 <= data4;
        data5 <= data5;
        data6 <= data6;
        data7 <= data7;
    end
  end  
end

endmodule

