
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

`include "vdispatcher.v"
`include "vlane_alu.v"
`include "vmul_unit.v"
`include "vmem_unit.v"
`include "vlane_flagalu.v"
`include "matmul_unit.v"

`define LO(x,b) ((x)&~({1024{1'b1}}<<b))

module vlanes (
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
    dma_en,
    mem_addr,
    lane_addr,
    dma_we,
    num_bytes,
    dma_busy,
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
    dbus_wait,

    dma_dbus_address,   
    dma_dbus_readdata,  
    dma_dbus_writedata, 
    dma_dbus_byteen,
    dma_dbus_en,        
    dma_dbus_wren,      
    dma_dbus_prefetch,  
    dma_dbus_wait,      
    dma_dbus_data_valid   
    );

parameter NUMLANES=4;
parameter LOG2NUMLANES=2;
parameter MVL=128;
parameter LOG2MVL=7;
parameter VPW=4;                  // VP width in number of bytes
parameter LOG2VPW=2;
parameter LANEWIDTH=8*VPW;
parameter LOG2LANEWIDTH=3+LOG2VPW;
parameter NUMBANKS=2;
parameter LOG2NUMBANKS=1;
parameter ALUPERBANK=1;
parameter NUMMEMPARALLELLANES=2;
parameter LOG2NUMMEMPARALLELLANES=1;
parameter NUMMULLANES=NUMLANES;
parameter DMEM_WRITEWIDTH=128;     // Width of write bus to memory
parameter LOG2DMEM_WRITEWIDTH=7;  // Log2 of Width of write bus to memory
parameter DMEM_READWIDTH=128;
parameter LOG2DMEM_READWIDTH=7;
parameter VCWIDTH=32;
parameter VSWIDTH=32;
parameter NUMVSREGS=32;
parameter LOG2NUMVSREGS=5;

parameter VRIDWIDTH=5;
parameter VELMIDWIDTH=LOG2MVL-LOG2NUMLANES;
parameter REGIDWIDTH=VRIDWIDTH+LOG2MVL-LOG2NUMLANES;
parameter BANKREGIDWIDTH=VRIDWIDTH+LOG2MVL-LOG2NUMLANES-LOG2NUMBANKS;

// Register identifier = { vr[0-31], element }

`define VRID_RANGE REGIDWIDTH-1:REGIDWIDTH-VRIDWIDTH
`define VRELM_RANGE REGIDWIDTH-VRIDWIDTH-1:0

// NUMFUS = ALU*NUMBANKS + MUL + MEM + FALU*MEMBANKS + MATMUL(1) + BFLOAT
// UNITS(2) + ACTIVATION + TRP
parameter NUMFUS=4+2*(NUMBANKS-1)*ALUPERBANK+1+4; //Adding 1 for matmul FU
parameter FU_ALU=0;
parameter FU_MUL=FU_ALU+(NUMBANKS-1)*ALUPERBANK+1;
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
output dma_busy;

// Control register values
input  [ VCWIDTH-1 : 0 ]   vc_in;
input  [ VCWIDTH-1 : 0 ]   vl_in;
input  [ VCWIDTH-1 : 0 ]   vbase_in;
input  [ VCWIDTH-1 : 0 ]   vinc_in;
input  [ VCWIDTH-1 : 0 ]   vstride_in;
input  [ VSWIDTH-1 : 0 ]   vs_in;
input  [3*`MAT_MUL_SIZE-1 : 0]  matmul_masks_in;
input  dma_en;
input  [VCWIDTH-1:0] mem_addr;
input  [VCWIDTH-1:0] lane_addr;
input  dma_we;
input  [VCWIDTH-1:0] num_bytes;

// vs Writeback
output       [ VSWIDTH-1 : 0 ]   vs_writedata;
output                           vs_we;
output               [ 5 : 2 ]   vs_wetrack;
output [ LOG2NUMVSREGS-1 : 0 ]   vs_dst;

// Data memory interface
output  [ 31 : 0 ]  dbus_address;
output              dbus_en;
output              dbus_we;
output  [ (DMEM_WRITEWIDTH/8)-1 : 0 ]   dbus_byteen;
output  [ DMEM_WRITEWIDTH-1 : 0 ]  dbus_writedata;
input   [ DMEM_READWIDTH-1 : 0 ]  dbus_readdata;
input               dbus_cachematch;
input               dbus_cachemiss;
input               dbus_wait;
output  [ 31 : 0 ]  dbus_prefetch;

//DMA signals
output [31:0]                 dma_dbus_address;   
input [DMEM_READWIDTH-1:0]    dma_dbus_readdata;  
output [DMEM_WRITEWIDTH-1:0]  dma_dbus_writedata; 
output [DMEM_WRITEWIDTH/8-1:0]dma_dbus_byteen;
output                        dma_dbus_en;        
output                        dma_dbus_wren;      
output                        dma_dbus_prefetch;  
input                         dma_dbus_wait;      
input                         dma_dbus_data_valid;


`include "visa.v"

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
reg   [ VCWIDTH-1 : 0 ]   vc_in_saved;
reg   [ VCWIDTH-1 : 0 ]   vl_in_saved;
reg   [ VCWIDTH-1 : 0 ]   vbase_in_saved;
reg   [ VCWIDTH-1 : 0 ]   vinc_in_saved;
reg   [ VCWIDTH-1 : 0 ]   vstride_in_saved;
reg   [ VSWIDTH-1 : 0 ]   vs_in_saved;

// Control register values
//TODO: Rethink whether the following vars (vc, vl, vbase, vinc, vstride)
//need to be change to use `MAX_PIPE_STAGES. I think these variables are 
//only used for memory operations and for that, we only need 6 stages.
//Not a big deal because most of these are not used much. Synthesis tool
//will optimize out the unnecessary bits anyway.
wire  [ VCWIDTH-1 : 0 ]   vc[`MAX_PIPE_STAGES-1:2];
wire  [ VCWIDTH-1 : 0 ]   vl[`MAX_PIPE_STAGES-1:2];
wire  [ VCWIDTH-1 : 0 ]   vbase[`MAX_PIPE_STAGES-1:2];
reg   [ VCWIDTH-1 : 0 ]   vbase_s4;
wire  [ VCWIDTH-1 : 0 ]   vinc[`MAX_PIPE_STAGES-1:2];
wire  [ VCWIDTH-1 : 0 ]   vstride[`MAX_PIPE_STAGES-1:2];
wire  [ VSWIDTH-1 : 0 ]   vs[`MAX_PIPE_STAGES-1:2];
reg   [ VSWIDTH-1 : 0 ]   vs_s3[NUMBANKS-1:0];
reg   [ VSWIDTH-1 : 0 ]   vs_s4[NUMFUS-1:0];
reg   [ VCWIDTH-1 : 0 ]   vc_s3[NUMBANKS-1:0];
reg   [ VCWIDTH-1 : 0 ]   vc_s4[NUMFUS-1:0];

// Vector register file signals
reg   [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vr_a_reg;
wire     [ NUMBANKS*8*VPW*NUMLANES-1 : 0 ]   _vr_a_readdataout;
reg                         [NUMBANKS-1:0]   vr_a_en;
reg   [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vr_b_reg;
wire     [ NUMBANKS*8*VPW*NUMLANES-1 : 0 ]   _vr_b_readdataout;
reg                         [NUMBANKS-1:0]   vr_b_en;
reg   [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   _vr_c_reg;
reg      [ NUMBANKS*8*VPW*NUMLANES-1 : 0 ]   _vr_c_writedatain;
reg        [ NUMBANKS*VPW*NUMLANES-1 : 0 ]   vr_c_byteen;
reg                         [NUMBANKS-1:0]   vr_c_we;

reg          [ LANEWIDTH*NUMLANES-1 : 0 ]   vr_a_readdataout[NUMBANKS-1:0];
reg          [ LANEWIDTH*NUMLANES-1 : 0 ]   vr_b_readdataout[NUMBANKS-1:0];
reg          [ LANEWIDTH*NUMLANES-1 : 0 ]   vr_c_writedatain[NUMBANKS-1:0];
reg           [ REGIDWIDTH-1 : 0 ]   vr_c_reg[NUMBANKS-1:0]; //for testbench and debugging

// Flag register file signals
reg       [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vf_a_reg;
wire               [ NUMBANKS*NUMLANES-1 : 0 ]   vf_a_readdataout;
reg                            [ NUMBANKS-1:0]   vf_a_en;
reg       [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vf_b_reg;
wire               [ NUMBANKS*NUMLANES-1 : 0 ]   vf_b_readdataout;
reg                            [ NUMBANKS-1:0]   vf_b_en;
reg       [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vf_c_reg;
reg                [ NUMBANKS*NUMLANES-1 : 0 ]   vf_c_writedatain;
reg                            [ NUMBANKS-1:0]   vf_c_we;

wire             [ REGIDWIDTH-1 : 0 ]   wb_dst[NUMFUS-1:0];
wire                     [NUMFUS-1:0]   wb_dst_we;
wire               [ NUMLANES-1 : 0 ]   wb_dst_mask[NUMFUS-1:0];

wire                        [ `MAX_PIPE_STAGES-1 : 4 ]   dst_we[NUMFUS-1:0];
wire             [ REGIDWIDTH-1 : 0 ]   dst[NUMFUS-1:0][`MAX_PIPE_STAGES-1:4];
wire               [ NUMLANES-1 : 0 ]   dst_mask[NUMFUS-1:0][`MAX_PIPE_STAGES-1:4];
wire             [ REGIDWIDTH-1 : 0 ]   dst_s2;
reg              [ REGIDWIDTH-1 : 0 ]   _dst_s3[NUMBANKS-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   dst_s3[NUMBANKS-1:0];
reg     [ NUMBANKS*REGIDWIDTH-1 : 0 ]   t_dst_s3;
reg              [ REGIDWIDTH-1 : 0 ]   dst_s4[NUMFUS-1:0];
reg                  [ NUMFUS-1 : 0 ]   dst_we_s4;
reg  [(1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1:0] alu_dst[4:4];
reg  [(1+(NUMBANKS-1)*ALUPERBANK)-1:0]            alu_dst_we[4:4];
reg  [(1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1:0] falu_dst[4:4];
reg  [(1+(NUMBANKS-1)*ALUPERBANK)-1:0]            falu_dst_we[4:4];

wire                                    imask_s2;
reg                [ NUMBANKS-1 : 0 ]   imask_s3;

wire             [ REGIDWIDTH-1 : 0 ]   src1_s2;
wire             [ REGIDWIDTH-1 : 0 ]   src2_s2;
reg              [ REGIDWIDTH-1 : 0 ]   src1_s3[NUMBANKS-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   src2_s3[NUMBANKS-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   _src1_s3[NUMBANKS-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   _src2_s3[NUMBANKS-1:0];
wire                                    src1scalar_s2;
wire                                    src2scalar_s2;
reg                    [NUMBANKS-1:0]   src1scalar_s3;
reg                    [NUMBANKS-1:0]   src2scalar_s3;
reg                      [NUMFUS-1:0]   src1scalar_s4;
reg                      [NUMFUS-1:0]   src2scalar_s4;

reg                    [NUMLANES-1:0]   lane_en[NUMBANKS-1:0];
reg                    [NUMLANES-1:0]   vlane_en[NUMFUS-1:0];
reg                                     mem_last_subvector_s4;

reg                     [LOG2MVL-1:0]   src_start_delayed;
wire                    [LOG2MVL-1:0]   src_elm;
wire                    [LOG2MVL-1:0]   src_limit;
reg                     [LOG2MVL-1:0]   src_limit_s3[NUMBANKS-1:0];
wire                    [LOG2MVL-1:0]   src_start;

wire                    [VCWIDTH-1:0]   total_shamt;
wire                    [LOG2MVL-1:0]   dst_start;

reg          [ LANEWIDTH*NUMLANES-1 : 0 ]   vr_src1[NUMFUS-1:0];
reg          [ LANEWIDTH*NUMLANES-1 : 0 ]   vr_src2[NUMFUS-1:0];
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   matmul_out;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   bfadder_result_s5;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   bfmult_result_s5;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   act_result_s5;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   trp_out;
reg                [ NUMLANES-1 : 0 ]   vf_src1[NUMFUS-1:0];
reg                [ NUMLANES-1 : 0 ]   vf_src2[NUMFUS-1:0];
reg                [ NUMLANES-1 : 0 ]   vmask[NUMFUS-1:0];
reg                [ NUMLANES-1 : 0 ]   vmask_final[NUMBANKS-1:0];

reg        [ VCWIDTH*NUMLANES-1 : 0 ]   vstrideoffset_s4;
wire     [ LANEWIDTH*NUMLANES-1 : 0 ]   load_result_s5;
wire               [ NUMLANES-1 : 0 ]   load_result_mask_s5;

wire [ LANEWIDTH*NUMLANES-1 : 0 ] alu_result_s5[(NUMBANKS-1)*ALUPERBANK:0];
wire           [ NUMLANES-1 : 0 ] alu_cmpresult_s4[(NUMBANKS-1)*ALUPERBANK:0];
wire           [ NUMLANES-1 : 0 ] flagalu_result_s4[(NUMBANKS-1)*ALUPERBANK:0];
wire           [ NUMLANES-1 : 0 ] flagalu_result_s5[(NUMBANKS-1)*ALUPERBANK:0];
wire [ LANEWIDTH*NUMLANES-1 : 0 ] mulshift_result_s5;

//Support 1 Lane processor
wire [((LOG2NUMLANES>0) ? LOG2NUMLANES : 1)-1:0] elmshamt[`MAX_PIPE_STAGES-1:2];

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
reg bf_op;

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

reg [NUMBANKS-1:0] ctrl3_vr_a_en; // SRC1
reg [NUMBANKS-1:0] ctrl3_vr_b_en; // SRC2
reg [NUMBANKS-1:0] ctrl3_vr_c_we; // DEST
reg [NUMBANKS-1:0] ctrl3_vf_a_en;
reg [NUMBANKS-1:0] ctrl3_vf_b_en;
reg [NUMBANKS-1:0] ctrl3_vf_c_we;
reg [NUMBANKS-1:0] ctrl3_vs_we;
reg [NUMBANKS-1:0] ctrl3_useslanes;
reg [NUMBANKS-1:0] ctrl3_mem_dir_left;
reg [NUMBANKS-1:0] ctrl3_rshiftnonzero;
reg [10:0] ctrl3_alu_op[NUMBANKS-1:0];
reg [1:0] ctrl3_satsum_op[NUMBANKS-1:0];
reg [3:0] ctrl3_satsize_op[NUMBANKS-1:0];
reg [4:0] ctrl3_mulshift_op[NUMBANKS-1:0];
reg [NUMBANKS-1:0] ctrl3_memunit_en;
reg [NUMBANKS-1:0] ctrl3_mem_en;
reg [6:0] ctrl3_memunit_op[NUMBANKS-1:0];
reg [NUMBANKS-1:0] ctrl3_ismasked;
reg [2:0] ctrl3_flagalu_op[NUMBANKS-1:0];
reg [NUMBANKS-1:0] ctrl3_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg [NUMBANKS-1:0] ctrl3_volatiledest;
reg [NUMBANKS-1:0] ctrl3_vf_a_sel; //0-0/1 from instr, 1-src1
reg [NUMBANKS-1:0] ctrl3_mulshift_en;
reg [NUMBANKS-1:0] ctrl3_matmul_en;
reg [NUMBANKS-1:0] ctrl3_trp_en;
reg [NUMBANKS-1:0] ctrl3_bfadder_en;
reg [NUMBANKS-1:0] ctrl3_bfmult_en;
reg [NUMBANKS-1:0] ctrl3_act_en;
reg [NUMBANKS-1:0] ctrl3_alufalu_en;

reg ctrl4_mem_dir_left;
reg ctrl4_rshiftnonzero;
reg [10:0] ctrl4_alu_op[(NUMBANKS-1)*ALUPERBANK:0];
reg [1:0] ctrl4_satsum_op[(NUMBANKS-1)*ALUPERBANK:0];
reg [3:0] ctrl4_satsize_op[(NUMBANKS-1)*ALUPERBANK:0];
reg [4:0] ctrl4_mulshift_op;
reg ctrl4_memunit_en;
reg ctrl4_mem_en;
reg [6:0] ctrl4_memunit_op;
reg [NUMFUS-1:0] ctrl4_ismasked;
reg [2:0] ctrl4_flagalu_op[(NUMBANKS-1)*ALUPERBANK:0];
//reg ctrl4_vf_wbsel[(NUMBANKS-1)*ALUPERBANK:0];   //0-flag ALU, 1-normal ALU
reg [(NUMBANKS-1)*ALUPERBANK:0] ctrl4_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg ctrl4_volatiledest;
//reg ctrl4_vf_a_sel[(NUMBANKS-1)*ALUPERBANK:0]; //0-0/1 from instr, 1-src1
reg [(NUMBANKS-1)*ALUPERBANK:0] ctrl4_vf_a_sel; //0-0/1 from instr, 1-src1
reg ctrl4_mulshift_en;
reg ctrl4_matmul_en;
reg ctrl4_trp_en;
reg ctrl4_bfadder_en;
reg ctrl4_bfmult_en;
reg ctrl4_act_en;

wire ctrl5_mem_en;

wire [`VRELM_RANGE] regid_pad;

integer bd;
genvar  ba;
integer bi;
integer i;
integer j;
genvar  bk;
genvar  k;
integer m;
integer n;
integer b;
integer b3;
integer f3;
integer bn;
integer fn;
integer fn2;
integer bw;

wire [NUMBANKS*LOG2MVL-1:0] rdelm;
wire [NUMBANKS*LOG2MVL-1:0] wrelm;
// MSb of count entries indicates if instruction is dead or not.
wire [NUMBANKS*(LOG2MVL-LOG2NUMLANES+1)-1:0] count;
reg  [NUMBANKS-1:0]   last_subvector;
wire [NUMBANKS-1:0]   first_subvector;
reg  [NUMBANKS-1:0] wrongbank_s3;
reg  [NUMBANKS-1:0] alive_s3;
reg  [NUMBANKS-1:0] banksel_s4[NUMFUS-1:0];

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

wire temp_stall_matmul;
// DEBUG signals for Modelsim
wire  [7:0] D_instr[2:1];
reg   [7:0] D_instr_s3[NUMBANKS-1:0];
reg   [7:0] D_instr_s4[NUMFUS-1:0];
wire  [7:0] D_instr_s5[NUMFUS-1:0];
wire  [7:0] D_instr_s6[NUMFUS-1:0];
reg   [NUMFUS-1:0] D_last_subvector_s4;
wire  [NUMFUS-1:0] D_last_subvector_s5;
wire  [NUMFUS-1:0] D_last_subvector_s6;
wire  [NUMFUS-1:0] D_last_subvector_s31;
wire  [NUMFUS-1:0] D_wb_last_subvector;
reg   [NUMBANKS-1:0] D_last_subvector_done;
reg   [NUMBANKS-1:0] D_wb_instrdone;

  pipe #(8,1) debuginstrpipe (
      .d( {instr[25:24],instr[5:0]} ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[2:1] ),
      .squash( pipe_squash[2:1] ),
      .q( {D_instr[2],D_instr[1]}));

  genvar Df;
  generate
  for (Df=0; Df<NUMFUS; Df=Df+1)
  begin : Debug_gen

    pipereg #(8) debugintrfupipereg1 (
      .d( D_instr_s4[Df] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s5[Df]));

    pipereg #(8) debugintrfupipereg2 (
      .d( D_instr_s5[Df] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s6[Df]));

    pipereg #(1) debuglastpipereg1 (
      .d( D_last_subvector_s4[Df] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s5[Df]));

    pipereg #(1) debuglastpipereg2 (
      .d( D_last_subvector_s5[Df] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s6[Df]));

  end
  endgenerate


//tell vpu to hold vc,vbase,etc values
assign is_stalled=~internal_pipe_advance | {stall_srcstart,2'b0};

assign has_memop=ctrl1_mem_en|ctrl2_mem_en|(|ctrl3_mem_en)|ctrl4_mem_en|ctrl5_mem_en;

/******************************************************************************/
/************************** Inter-Pipe Signals ********************************/
/******************************************************************************/

/* Memunit is in stage 5 but we stall in stage 4 so we don't squash the
* destination register contents*/

assign pipe_advance=internal_pipe_advance & ~{5'b0,stall_in};

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
assign internal_pipe_advance[5]=internal_pipe_advance[6] && ~stall_mulunit && ~stall_memunit && ~temp_stall_matmul;
//assign internal_pipe_advance[6]=1'b1;

assign pipe_squash[0]=pipe_advance[1]&~pipe_advance[0];
assign pipe_squash[1]=pipe_advance[2]&~pipe_advance[1];
assign pipe_squash[2]=pipe_advance[3]&~pipe_advance[2];
assign pipe_squash[3]=pipe_advance[4]&~pipe_advance[3];
assign pipe_squash[4]=pipe_advance[5]&~pipe_advance[4];
assign pipe_squash[5]=pipe_advance[6]&~pipe_advance[5];
assign pipe_squash[6]=1'b0;
assign pipe_squash[7]=1'b0;
assign internal_pipe_advance[7:6] = 2'b11;
//assign pipe_squash[6]=1'b0;

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
    ctrl1_vr_a_en=0;
    ctrl1_vr_b_en=0;
    ctrl1_vr_c_en=0;
    ctrl1_vf_a_en=0;
    ctrl1_vf_b_en=0;
    ctrl1_vf_a_sel=0;
    ctrl1_usesvssel=0;
    casez(ir_op)
      COP2_VADD,
      COP2_VADD_U:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VSUB,
      COP2_VSUB_U:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
      COP2_VMULHI,
      COP2_VMULHI_U:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    //  COP2_VMOD,
      COP2_VMOD_U,
      COP2_VCMP_EQ,
      COP2_VCMP_NE,
      COP2_VCMP_LT,
      COP2_VCMP_U_LT,
      COP2_VCMP_LE,
      COP2_VCMP_U_LE:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
      COP2_VDIV,
      COP2_VDIV_U,
      COP2_VBFADD,
      COP2_VMIN,
      COP2_VMIN_U,
      COP2_VMAX,
      COP2_VMAX_U,
      COP2_VMULLO:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VABS:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=1;
        end
      COP2_VAND,
      COP2_VOR,
      COP2_VXOR,
      COP2_VNOR:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VSLL,
      COP2_VSRL,
      COP2_VSRA:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
      COP2_VSAT_B,
      COP2_VSAT_H,
      COP2_VSAT_W,
      COP2_VSAT_SU_B,
      COP2_VSAT_SU_H,
      COP2_VSAT_SU_W,
      COP2_VSAT_SU_L,
      COP2_VSAT_U_B,
      COP2_VSAT_U_H,
      COP2_VSAT_U_W:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
      COP2_VSADD,
      COP2_VSADD_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VSSUB,
      COP2_VSSUB_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VSRR,
      COP2_VSRR_U,
      COP2_VSLS,
      COP2_VSLS_U:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
      COP2_VXUMUL,
      COP2_VXUMUL_U,
      COP2_VXLMUL,
      COP2_VXLMUL_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VXUMADD,
      COP2_VXUMADD_U,
      COP2_VXUMSUB,
      COP2_VXUMSUB_U,
      COP2_VXLMADD,
      COP2_VXLMADD_U,
      COP2_VXLMSUB,
      COP2_VXLMSUB_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VINS_VV:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vr_b_en=1;
        end
      //COP2_VINS_SV: doesn't read any vectors or flags
      COP2_VEXT_VV,
      COP2_VEXT_SV,
      COP2_VEXT_U_SV:
        begin
          ctrl1_vr_a_en=1;
        end
      COP2_VCOMPRESS,
      COP2_VEXPAND:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
      COP2_VMERGE:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
      //COP2_VFINS:
      COP2_VEXTHALF,
      COP2_VHALF,
      COP2_VHALFUP,
      COP2_VHALFDN:
        begin
          ctrl1_vr_a_en=1;
        end
      //COP2_VSATVL:
      COP2_VFAND,
      COP2_VFOR,
      COP2_VFXOR,
      COP2_VFNOR:
        begin
          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vf_b_en=1;
          ctrl1_vf_a_sel=1;
          ctrl1_usesvssel=1;
        end
      //COP2_VFCLR:
      //COP2_VFSET:
      COP2_VIOTA,
      COP2_VCIOTA,
      COP2_VFPOP,
      COP2_VFFF1,
      COP2_VFFL1,
      COP2_VFSETBF,
      COP2_VFSETIF,
      COP2_VFSETOF:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
      //COP2_VFMT8:
      //COP2_VFMF8:
      //COP2_VFCLR8:
      //COP2_VFOR8:
      //COP2_VFLD:
      COP2_VLD_B,
      COP2_VLD_H,
      //COP2_VBFADD,
      COP2_VLD_L,
      COP2_VLD_U_B,
      COP2_VLD_U_H,
      COP2_VLD_U_W,
      COP2_VLDS_B,
      COP2_VLDS_H,
      COP2_VBFSUB,
      COP2_VLDS_L,
      COP2_VLDS_U_B,
      COP2_VLDS_U_H,
      COP2_VBFMULT:
        begin
          ctrl1_vf_a_en=1;
        end
      COP2_VLDX_B,
      COP2_VLDX_H,
      COP2_VTRP,
      COP2_VLDX_L,
      COP2_VLDX_U_B,
      COP2_VLDX_U_H,
      COP2_VACT:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
        end
      //COP2_VFST:
      COP2_VST_B,
      COP2_VST_H,
      COP2_VRED,
      COP2_VST_L,
      COP2_VSTS_B,
      COP2_VSTS_H,
      COP2_VPER,
      COP2_VSTS_L:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_b_en=1;
        end
      COP2_VSTX_B,
      COP2_VSTX_H,
      COP2_VSTX_W,
      COP2_VSTX_L,
      COP2_VSTXO_B,
      COP2_VSTXO_H,
      COP2_VSTXO_W,
      COP2_VSTXO_L:
        begin
          ctrl1_vr_b_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_vf_a_en=1;
        end
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  // Decode enables 
  always@*
  begin
    ctrl1_vr_d_we=0;
    ctrl1_vf_c_we=0;
    ctrl1_vs_we=0;
    ctrl1_vrdest_sel=0;
    ctrl1_elmshamt_sel=0;
    ctrl1_srcshamt_sel=0;
    ctrl1_srclimit_sel=0;
    ctrl1_dstshamt_sel=0;
    ctrl1_mem_dir_left=0;
    ctrl1_rshiftnonzero=0;
    ctrl1_memunit_en=0;
    ctrl1_mem_en=0;
    ctrl1_ismasked=0;
    ctrl1_setvlto1=0;
    ctrl1_vf_wbsel=0;
    ctrl1_volatiledest=0;
    casez(ir_op)
      COP2_VADD,
      COP2_VADD_U,
      COP2_VSUB,
      COP2_VSUB_U,
      COP2_VMULHI,
      COP2_VMULHI_U,
      COP2_VDIV,
      COP2_VDIV_U,
      COP2_VBFADD,
    //  COP2_VMOD,
      COP2_VMOD_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
      COP2_VCMP_EQ,
      COP2_VCMP_NE,
      COP2_VCMP_LT,
      COP2_VCMP_U_LT,
      COP2_VCMP_LE,
      COP2_VCMP_U_LE:
        begin
          ctrl1_vf_c_we=1;
          ctrl1_vf_wbsel=1;
          ctrl1_ismasked=1;
        end
      COP2_VMIN,
      COP2_VMIN_U,
      COP2_VMAX,
      COP2_VMAX_U,
      COP2_VMULLO,
      COP2_VABS,
      COP2_VAND,
      COP2_VOR,
      COP2_VXOR,
      COP2_VNOR,
      COP2_VSLL,
      COP2_VSRL,
      COP2_VSRA,
      COP2_VSAT_B,
      COP2_VSAT_H,
      COP2_VSAT_W,
      COP2_VSAT_SU_B,
      COP2_VSAT_SU_H,
      COP2_VSAT_SU_W,
      COP2_VSAT_SU_L,
      COP2_VSAT_U_B,
      COP2_VSAT_U_H,
      COP2_VSAT_U_W,
      COP2_VSADD,
      COP2_VSADD_U,
      COP2_VSSUB,
      COP2_VSSUB_U,
      COP2_VSRR,
      COP2_VSRR_U,
      COP2_VSLS,
      COP2_VSLS_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
      COP2_VXUMUL,
      COP2_VXUMUL_U,
      COP2_VXLMUL,
      COP2_VXLMUL_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
      COP2_VXUMADD,
      COP2_VXUMADD_U,
      COP2_VXUMSUB,
      COP2_VXUMSUB_U,
      COP2_VXLMADD,
      COP2_VXLMADD_U,
      COP2_VXLMSUB,
      COP2_VXLMSUB_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
      COP2_VINS_VV:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=0;
          ctrl1_dstshamt_sel=3;
          ctrl1_mem_dir_left=1;
          ctrl1_volatiledest=1;
        end
      COP2_VINS_SV:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_elmshamt_sel=0;
          ctrl1_srcshamt_sel=0;
          ctrl1_dstshamt_sel=3;
          ctrl1_mem_dir_left=1;
          ctrl1_setvlto1=1;
        end
      COP2_VEXT_VV:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=3;
          ctrl1_srclimit_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_volatiledest=1;
        end
      COP2_VEXT_SV,
      COP2_VEXT_U_SV:
        begin
          ctrl1_vs_we=1;
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=3;
          ctrl1_srclimit_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_setvlto1=1;
        end
      COP2_VCOMPRESS:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_mem_dir_left=0;
          ctrl1_ismasked=1;
          ctrl1_volatiledest=1;
        end
      COP2_VEXPAND:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_mem_dir_left=1;
          ctrl1_ismasked=1;
          ctrl1_volatiledest=1;
        end
      COP2_VMERGE:
        begin
          ctrl1_vr_d_we=1;
        end
      COP2_VFINS:
        begin
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=0;
          ctrl1_mem_dir_left=1;
        end
      COP2_VEXTHALF,
      COP2_VHALF:
        begin
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=1;
          ctrl1_srcshamt_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_vr_d_we=1;
          ctrl1_volatiledest=1;
        end
      COP2_VHALFUP:
        begin
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=2;
          ctrl1_srcshamt_sel=2;
          ctrl1_mem_dir_left=0;
          ctrl1_vr_d_we=1;
          ctrl1_volatiledest=1;
        end
      COP2_VHALFDN:
        begin
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=2;
          ctrl1_srcshamt_sel=0;
          ctrl1_dstshamt_sel=2;
          ctrl1_mem_dir_left=1;
          ctrl1_vr_d_we=1;
          ctrl1_volatiledest=1;
        end
      //COP2_VSATVL:
      COP2_VFAND,
      COP2_VFOR,
      COP2_VFXOR,
      COP2_VFNOR,
      COP2_VFCLR,
      COP2_VFSET:
        begin
          ctrl1_vf_c_we=1;
        end
      COP2_VIOTA,
      COP2_VCIOTA:
        begin
          ctrl1_vr_d_we=1;
        end
      COP2_VFPOP,
      COP2_VFFF1,
      COP2_VFFL1:
        begin
          ctrl1_vs_we=1;
        end
      COP2_VFSETBF,
      COP2_VFSETIF,
      COP2_VFSETOF,
      COP2_VFMT8,
      COP2_VFMF8,
      COP2_VFCLR8,
      COP2_VFOR8:
        begin
          ctrl1_vf_c_we=1;
        end
      //COP2_VFLD,
      COP2_VLD_B,
      COP2_VLD_H,
      COP2_VBFADD,
      COP2_VLD_L,
      COP2_VLD_U_B,
      COP2_VLD_U_H,
      COP2_VLD_U_W,
      COP2_VLDS_B,
      COP2_VLDS_H,
      COP2_VBFSUB,
      COP2_VLDS_L,
      COP2_VLDS_U_B,
      COP2_VLDS_U_H,
      COP2_VBFMULT:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
      COP2_VLDX_B,
      COP2_VLDX_H,
      COP2_VTRP,
      COP2_VLDX_L,
      COP2_VLDX_U_B,
      COP2_VLDX_U_H,
      COP2_VACT:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
      //COP2_VFST:
      COP2_VST_B,
      COP2_VST_H,
      COP2_VRED,
      COP2_VST_L,
      COP2_VSTS_B,
      COP2_VSTS_H,
      COP2_VPER,
      COP2_VSTS_L,
      COP2_VSTX_B,
      COP2_VSTX_H,
      COP2_VSTX_W,
      COP2_VSTX_L,
      COP2_VSTXO_B,
      COP2_VSTXO_H,
      COP2_VSTXO_W,
      COP2_VSTXO_L:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
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
    casez(ir_op)
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
      //COP2_VBFADD: ctrl1_memunit_op=MEMOP_LDW;
      //COP2_VLD_L,
      COP2_VLD_U_B: ctrl1_memunit_op=MEMOP_LDUB;
      COP2_VLD_U_H: ctrl1_memunit_op=MEMOP_LDUH;
      COP2_VLD_U_W: ctrl1_memunit_op=MEMOP_LDUW;
      COP2_VLDS_B: ctrl1_memunit_op=MEMOP_LDSB;
      COP2_VLDS_H: ctrl1_memunit_op=MEMOP_LDSH;
      COP2_VBFSUB: ctrl1_memunit_op=MEMOP_LDSW;
      //COP2_VLDS_L:
      COP2_VLDS_U_B: ctrl1_memunit_op=MEMOP_LDSUB;
      COP2_VLDS_U_H: ctrl1_memunit_op=MEMOP_LDSUH;
      COP2_VBFMULT: ctrl1_memunit_op=MEMOP_LDSUW;
      COP2_VLDX_B: ctrl1_memunit_op=MEMOP_LDXB;
      COP2_VLDX_H: ctrl1_memunit_op=MEMOP_LDXH;
      COP2_VTRP: ctrl1_memunit_op=MEMOP_LDXW;
      //COP2_VLDX_L:
      COP2_VLDX_U_B: ctrl1_memunit_op=MEMOP_LDXUB;
      COP2_VLDX_U_H: ctrl1_memunit_op=MEMOP_LDXUH;
      COP2_VACT: ctrl1_memunit_op=MEMOP_LDXUW;
      //COP2_VFST:
      COP2_VST_B: ctrl1_memunit_op=MEMOP_STB;
      COP2_VST_H: ctrl1_memunit_op=MEMOP_STH;
      COP2_VRED: ctrl1_memunit_op=MEMOP_STW;
      //COP2_VST_L:
      COP2_VSTS_B: ctrl1_memunit_op=MEMOP_STSB;
      COP2_VSTS_H: ctrl1_memunit_op=MEMOP_STSH;
      COP2_VPER: ctrl1_memunit_op=MEMOP_STSW;
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

  pipereg 
    #(
      2+
      2+
      1+
      2+
      1
    ) pipe1reg_secondstageonly (
      .d( {
        ctrl1_elmshamt_sel,
        ctrl1_srcshamt_sel,
        ctrl1_srclimit_sel,
        ctrl1_dstshamt_sel,
        ctrl1_setvlto1
      }),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[1] ),
      .squashn( 1'b1 ),
      .q({
        ctrl2_elmshamt_sel,
        ctrl2_srcshamt_sel,
        ctrl2_srclimit_sel, //used to get squashed, pretend doesn't need to
        ctrl2_dstshamt_sel,
        ctrl2_setvlto1
      }));

  // *********** Pipeline signals that need to be squashed *********
  pipereg 
    #(
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      3+
      1+
      1
    ) pipe1regwsquash (
      .d( {
        ctrl1_vr_d_we,
        ctrl1_vf_c_we,
        ctrl1_vs_we,
        ctrl1_vr_a_en|ctrl1_vr_c_en,
        ctrl1_vr_b_en,
        ctrl1_vf_a_en,
        ctrl1_vf_b_en,
        ctrl1_vr_a_en|ctrl1_vr_b_en|ctrl1_vr_c_en|ctrl1_vr_d_we|ctrl1_vf_a_en|ctrl1_vf_b_en|ctrl1_vf_c_we,
        ctrl1_memunit_en,
        ctrl1_mem_en,
        |ctrl1_mulshift_op,
        ctrl1_matmul_en,
        ctrl1_bfadder_en,
        ctrl1_bfmult_en,
        ctrl1_act_en,
        ctrl1_trp_en,
        (ctrl1_alu_op!=(ALUOP_ZERO^ALUOP_ZERO)) || ctrl1_vf_c_we
      }),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[1] ),
      .squashn( ~pipe_squash[1] ),
      .q({
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
        })
      );

  // *********** Pipeline signals that don't need to be squashed *********
  pipereg 
    #(
      REGIDWIDTH+
      REGIDWIDTH+
      REGIDWIDTH+
      1+
      1+
      1+
      1+
      1+
      1+
      11+
      2+
      4+
      5+
      7+
      1+
      3+
      1+
      1+
      1
    ) pipe1reg (
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
      .en( pipe_advance[1] ),
      .squashn( 1'b1 ),
      .q({
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
      }));
  
  wire [6:1] squash_ctrlmemoppipe_NC;
  pipe #(7,5) ctrlmemoppipe (
      .d( ctrl1_memunit_op ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:1] & {4'b1,ctrl2_memunit_en,1'b1} ),
      .squash(squash_ctrlmemoppipe_NC),
      //.squash( pipe_squash[6:1] ),
      .q( {ctrl_memunit_op[6],ctrl_memunit_op[5],ctrl_memunit_op[4],ctrl_memunit_op[3],ctrl_memunit_op[2],ctrl_memunit_op[1]} ));

/******************************************************************************/
/******************************* 2nd Pipeline Stage ***************************/
/******************************************************************************/

  // if src_start!=0 stall pipeline to calculate it and then do haz check

  onecyclestall shamtstall(ctrl2_srcshamt_sel!=0,clk,resetn,stall_srcstart);

  always@(posedge clk)
    if (!resetn || pipe_advance[1] )
      src_start_delayed<=0;
    else 
      src_start_delayed<=src_start;

  assign src_start= ( ctrl2_srcshamt_sel==3 ) ? vc[2] :           // vcindex
               ( ctrl2_srcshamt_sel==1 ) ? vl[2][VCWIDTH-1:1] :   // vl>>2
               ( ctrl2_srcshamt_sel==2 ) ? 1 << vc[2][LOG2MVL-1:0]://2^vcindex
               0;

  assign src_elm=src_start_delayed & (-1<<LOG2NUMLANES);

  assign src_limit= ((ctrl2_setvlto1) ? 0 : vl[2][LOG2MVL-1:0] - 1'b1) +
                    ((ctrl2_srclimit_sel) ? vc[2][LOG2MVL-1:0] : 0);


  /******************* Adjust dest to account for shift  ****************/

  // Compute real destination register - accounting for shifting instructions

  assign dst_start= ( ctrl2_dstshamt_sel==3 ) ? vc[2] :           // vcindex
               ( ctrl2_dstshamt_sel==2 ) ? 1 << vc[2][LOG2MVL-1:0]://2^vcindex
               0;

  //assign dst_elm = {dst_start[LOG2MVL-1:0]>>LOG2NUMLANES,{LOG2NUMLANES{1'b0}}};

  assign total_shamt= ( ctrl2_elmshamt_sel==3 ) ? vc[2] :          // vcindex
               ( ctrl2_elmshamt_sel==1 ) ? vl[2][VCWIDTH-1:1] :    // vl>>2
               ( ctrl2_elmshamt_sel==2 ) ? 1 << vc[2][LOG2MVL-1:0]://2^vcindex
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

  wire [6:2] squash_vcpipe_NC;
  pipe #(VCWIDTH,4) vcpipe (
      .d( (!pipe_advance_s2_r) ? vc_in_saved : vc_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vcpipe_NC),
      .q( {vc[6],vc[5],vc[4],vc[3],vc[2]} ));

  wire [6:2] squash_vlpipe_NC;
  pipe #(VCWIDTH,4) vlpipe (
      .d( (!pipe_advance_s2_r) ? vl_in_saved : vl_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vlpipe_NC),
      .q( {vl[6],vl[5],vl[4],vl[3],vl[2]} ));

  wire [6:2] squash_vbasepipe_NC;
  pipe #(VCWIDTH,4) vbasepipe (
      .d( (!pipe_advance_s2_r) ? vbase_in_saved : vbase_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vbasepipe_NC),
      .q( {vbase[6],vbase[5],vbase[4],vbase[3],vbase[2]} ));

  wire [6:2] squash_vincpipe_NC;
  pipe #(VCWIDTH,4) vincpipe (
      .d( (!pipe_advance_s2_r) ? vinc_in_saved : vinc_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vincpipe_NC),
      .q( {vinc[6],vinc[5],vinc[4],vinc[3],vinc[2]} ));

  //stride register also used for elmt shamt for vext/vhalf/etc in memunit
  wire [6:2] squash_vstridepipe_NC;
  pipe #(VCWIDTH,4) vstridepipe (
      .d( (ctrl2_memunit_en&~ctrl2_mem_en) ? 
            ((LOG2NUMLANES>0) ?
                  total_shamt[((LOG2NUMLANES>0) ? LOG2NUMLANES : 1)-1:0] : 0) :
          (!pipe_advance_s2_r) ? vstride_in_saved : vstride_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
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
    (|(ctrl3_alufalu_en&~last_subvector) && ctrl2_alufalu_en && ALUPERBANK==0)||
    //Entry 0 is taken
    (dispatcher_rotate && ctrl2_useslanes);

  //assign stall_mulcooldown=|(ctrl3_mulshift_en&last_subvector);
  assign stall_mulcooldown=1'b0;

  assign dispatcher_shift=pipe_advance[3];
  // Rotate if last entry is alive (count not -1) and greater than zero
  assign dispatcher_rotate=(~count[NUMBANKS*(LOG2MVL-LOG2NUMLANES+1)-1]) && 
                          ((count>>((NUMBANKS-1)*(LOG2MVL-LOG2NUMLANES+1)))!=0);

`define DISPATCHWIDTH LOG2MVL+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    REGIDWIDTH+ \
    REGIDWIDTH+ \
    REGIDWIDTH+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    11+ \
    2+ \
    4+ \
    5+ \
    7+ \
    1+ \
    3+ \
    1+ \
    1+ \
    1+ \
    1+ \
    VCWIDTH+ \
    VSWIDTH+ \
    8
wire [NUMBANKS*(`DISPATCHWIDTH)-1:0] dispatcher_instr;
  vdispatcher #(NUMBANKS, `DISPATCHWIDTH,LOG2MVL,LOG2MVL,LOG2MVL-LOG2NUMLANES+1)
    vdispatcher(
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
      .inshift_count((pipe_squash[2] || !ctrl2_useslanes) ? -1 : (src_limit[LOG2MVL-1:0]>>LOG2NUMLANES) - (src_elm[LOG2MVL-1:0]>>LOG2NUMLANES) ),
      .increment((~wrongbank_s3)&{NUMBANKS{pipe_advance[3]}}),
      .rdelm_add_sub(0),
      .wrelm_add_sub(0),
      .count_add_sub(1),
      .rdelm_valuetoadd(NUMLANES),
      .wrelm_valuetoadd(NUMLANES),
      .count_valuetoadd(1),
      .instr(dispatcher_instr),
      .first(first_subvector),
      .rdelm(rdelm),
      .wrelm(wrelm),
      .count(count)
    );

  

  /******************* Stall on RAW HAZARDS ************************/

  always@*
    for (bd=0; bd<1+(NUMBANKS-1)*ALUPERBANK; bd=bd+1)
    begin
      alu_dst[4][bd*REGIDWIDTH +: REGIDWIDTH]=dst[FU_ALU+bd][4];
      alu_dst_we[4][bd]=dst_we[FU_ALU+bd][4];
      falu_dst[4][bd*REGIDWIDTH +: REGIDWIDTH]=dst[FU_FALU+bd][4];
      falu_dst_we[4][bd]=dst_we[FU_FALU+bd][4];
    end


  hazardchecker #(REGIDWIDTH,VELMIDWIDTH,4+(NUMBANKS-1)*ALUPERBANK,NUMBANKS) 
    src1hazchecker(
      .src( src1_s2 |(src_start_delayed[LOG2MVL-1:0]>>LOG2NUMLANES) ),
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
      .dst_mode({{3+(NUMBANKS-1)*ALUPERBANK{1'b0}},ctrl4_volatiledest}),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vr_c_we),
      .lt_mode(ctrl3_volatiledest),
      .haz(stall_hazsrc1));

  hazardchecker #(REGIDWIDTH,VELMIDWIDTH,4+(NUMBANKS-1)*ALUPERBANK,NUMBANKS) 
    src2hazchecker(
      .src( src2_s2 |(src_start_delayed[LOG2MVL-1:0]>>LOG2NUMLANES) ),
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
      .dst_mode({{3+(NUMBANKS-1)*ALUPERBANK{1'b0}},ctrl4_volatiledest}),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vr_c_we),
      .lt_mode(ctrl3_volatiledest),
      .haz(stall_hazsrc2));

  //Check flag hazards - flags always start at 0th element
  hazardchecker #(REGIDWIDTH,VELMIDWIDTH,1+(NUMBANKS-1)*ALUPERBANK,NUMBANKS) 
    fsrc1hazchecker(
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

  hazardchecker #(REGIDWIDTH,VELMIDWIDTH,1+(NUMBANKS-1)*ALUPERBANK,NUMBANKS) 
    fsrc2hazchecker(
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
    for (bi=0; bi<NUMBANKS; bi=bi+1)
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

      last_subvector[bi]=~|count[bi*(LOG2MVL-LOG2NUMLANES+1)+:LOG2MVL-LOG2NUMLANES];
      alive_s3[bi]=~count[(bi+1)*(LOG2MVL-LOG2NUMLANES+1)-1];

      for (i=0; i<NUMLANES; i=i+1)
        lane_en[bi][i]= (LOG2NUMLANES==0) || //Support 1 lane
          ~((first_subvector[bi]) && 
              i<`LO(rdelm[bi*LOG2MVL +: LOG2MVL],LOG2NUMLANES) ||
            (last_subvector[bi]) && 
              i>src_limit_s3[bi][((LOG2NUMLANES>0) ? LOG2NUMLANES : 1)-1:0] );
    end


  // ************* Map from issuer to register file banks *************
  always@*
    for (b=0; b<NUMBANKS; b=b+1)
    begin 
      dst_s3[b]=_dst_s3[b] | (wrelm[b*LOG2MVL+:LOG2MVL]>>LOG2NUMLANES);
      t_dst_s3[b*REGIDWIDTH +: REGIDWIDTH]=dst_s3[b];
      src1_s3[b]=_src1_s3[b] | (rdelm[b*LOG2MVL+:LOG2MVL]>>LOG2NUMLANES);
      src2_s3[b]=_src2_s3[b] | (rdelm[b*LOG2MVL+:LOG2MVL]>>LOG2NUMLANES);

      wrongbank_s3[b]=(`LO(rdelm[b*LOG2MVL+:LOG2MVL]>>LOG2NUMLANES,LOG2NUMBANKS)!=b);

      vr_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src1_s3[b]>>LOG2NUMBANKS;
      vr_b_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src2_s3[b]>>LOG2NUMBANKS;
      vr_a_en[b]=ctrl3_vr_a_en[b] && pipe_advance[3];
      vr_b_en[b]=ctrl3_vr_b_en[b] && pipe_advance[3];

      vf_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]= 
        {(ctrl3_vf_a_sel[b]) ? _src1_s3[b][`VRID_RANGE] : imask_s3[b],
        src1_s3[b][`VRELM_RANGE]}>>LOG2NUMBANKS;
      vf_b_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src2_s3[b]>>LOG2NUMBANKS;
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

      for (b3=0; b3<NUMBANKS; b3=b3+1)
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
          else if (ctrl3_act_en[b3])    //is bfloat addition
          begin
            D_instr_s4[FU_ACT]=D_instr_s3[b3];
            D_last_subvector_s4[FU_ACT]=last_subvector[b3];
            ctrl4_act_en = ctrl3_act_en[b3];
            banksel_s4[FU_ACT]=b3;
            vs_s4[FU_ACT]=vs_s3[b3];
            vc_s4[FU_ACT]=vc_s3[b3];
            dst_s4[FU_ACT]=dst_s3[b3];
            dst_we_s4[FU_ACT]=ctrl3_vr_c_we[b3];
            vlane_en[FU_ACT]=lane_en[b3];
            src1scalar_s4[FU_ACT]=src1scalar_s3[b3];
            src2scalar_s4[FU_ACT]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_ACT]=ctrl3_ismasked[b3];
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
            vs_s4[FU_MEM]=count[b3*(LOG2MVL-LOG2NUMLANES+1)+:(LOG2MVL-LOG2NUMLANES)]<<LOG2NUMLANES;
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
              vbase[3] : vbase_s4 + ((((ctrl_memunit_op[3][4])? vstride[3]: 2)
                      <<ctrl_memunit_op[3][3:2])<<LOG2NUMLANES);
            // Partial Address Gen for each lane - just do multiplication part
            for (m=0; m<NUMLANES; m=m+1)
              vstrideoffset_s4[m*VCWIDTH +: VCWIDTH] = 
                                ((ctrl_memunit_op[3][4]) ? vstride[3] : 1)*m;
            mem_last_subvector_s4=last_subvector[b3];
          end
          else if (ctrl3_alu_op[b3]!=(ALUOP_ZERO^ALUOP_ZERO)) //is ALU
          begin
            D_instr_s4[FU_ALU+b3*ALUPERBANK]=D_instr_s3[b3];
            D_last_subvector_s4[FU_ALU+b3*ALUPERBANK]=last_subvector[b3];
            banksel_s4[FU_ALU+b3*ALUPERBANK]=b3;
            vs_s4[FU_ALU+b3*ALUPERBANK]=vs_s3[b3];
            dst_we_s4[FU_ALU+b3*ALUPERBANK]=ctrl3_vr_c_we[b3];
            dst_s4[FU_ALU+b3*ALUPERBANK]=dst_s3[b3];
            dst_we_s4[FU_FALU+b3*ALUPERBANK]=ctrl3_vf_c_we[b3];
            dst_s4[FU_FALU+b3*ALUPERBANK]=dst_s3[b3];
            vlane_en[FU_ALU+b3*ALUPERBANK]=lane_en[b3];
            src1scalar_s4[FU_ALU+b3*ALUPERBANK]=src1scalar_s3[b3];
            src2scalar_s4[FU_ALU+b3*ALUPERBANK]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_ALU+b3*ALUPERBANK]=ctrl3_ismasked[b3];
            ctrl4_alu_op[b3*ALUPERBANK]=ctrl3_alu_op[b3];
            ctrl4_satsum_op[b3*ALUPERBANK]=ctrl3_satsum_op[b3];
            ctrl4_satsize_op[b3*ALUPERBANK]=ctrl3_satsize_op[b3];
            ctrl4_vf_wbsel[b3*ALUPERBANK]=ctrl3_vf_wbsel[b3];
          end
          else if (ctrl3_vf_c_we[b3])
          begin                                    //is FALU
            D_instr_s4[FU_FALU+b3*ALUPERBANK]=D_instr_s3[b3];
            D_last_subvector_s4[FU_FALU+b3*ALUPERBANK]=last_subvector[b3];
            banksel_s4[FU_FALU+b3*ALUPERBANK]=b3;
            vs_s4[FU_FALU+b3*ALUPERBANK]=|vs_s3[b3];
            dst_we_s4[FU_FALU+b3*ALUPERBANK]=ctrl3_vf_c_we[b3];
            dst_s4[FU_FALU+b3*ALUPERBANK]=dst_s3[b3];
            vlane_en[FU_FALU+b3*ALUPERBANK]=lane_en[b3];
            src1scalar_s4[FU_FALU+b3*ALUPERBANK]=src1scalar_s3[b3];
            src2scalar_s4[FU_FALU+b3*ALUPERBANK]=src2scalar_s3[b3];
            ctrl4_flagalu_op[b3*ALUPERBANK]=ctrl3_flagalu_op[b3];
            ctrl4_vf_wbsel[b3*ALUPERBANK]=ctrl3_vf_wbsel[b3];
            ctrl4_vf_a_sel[b3*ALUPERBANK]=ctrl3_vf_a_sel[b3];
          end
        end
      end


  vregfile_vector 
    #(NUMBANKS,LOG2NUMBANKS,8*VPW*NUMLANES,32*MVL/NUMLANES,REGIDWIDTH) 
    vregfile_vector (
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

  vregfile_flag 
    #(NUMBANKS,LOG2NUMBANKS,NUMLANES,32*MVL/NUMLANES,REGIDWIDTH) 
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


  pipereg #(1) sdstwepipereg (
      .d( |ctrl3_vs_we ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[3] ),
      .squashn( ~pipe_squash[3] ),
      .q(ctrl_vs_we[4]));

/******************************************************************************/
/************************** 4th Pipeline Stage ********************************/
/******************************************************************************/

  //Convert register file width (8*VPW) to datpath width (LANEWIDTH)
  always@*
    for (bn=0; bn<NUMBANKS; bn=bn+1)
      for (n=0; n<NUMLANES; n=n+1)
      begin
        vr_a_readdataout[bn][LANEWIDTH*n +: LANEWIDTH] = 
          _vr_a_readdataout[bn*8*VPW*NUMLANES + 8*VPW*n +: 8*VPW];
        vr_b_readdataout[bn][LANEWIDTH*n +: LANEWIDTH] = 
          _vr_b_readdataout[bn*8*VPW*NUMLANES + 8*VPW*n +: 8*VPW];
        _vr_c_writedatain[bn*8*VPW*NUMLANES + 8*VPW*n +: 8*VPW] = 
          vr_c_writedatain[bn][LANEWIDTH*n +: LANEWIDTH];
      end

  //Bank Multiplex for each functional unit
  always@*
  begin
    for (fn=0; fn<=FU_MUL; fn=fn+1)
    begin
      vr_src1[fn] =(src1scalar_s4[fn]) ? {NUMLANES{vs_s4[fn][LANEWIDTH-1:0]}} : 
                                    vr_a_readdataout[banksel_s4[fn]];
      vr_src2[fn] =(src2scalar_s4[fn]) ? {NUMLANES{vs_s4[fn][LANEWIDTH-1:0]}} : 
                                    vr_b_readdataout[banksel_s4[fn]];
      vf_src1[fn] = vf_a_readdataout[banksel_s4[fn]*NUMLANES +: NUMLANES];

      vmask[fn] =vlane_en[fn] &
            ((ctrl4_ismasked[fn]) ?
              vf_a_readdataout[banksel_s4[fn]*NUMLANES +: NUMLANES] :
              {NUMLANES{1'b1}}) ;
    end

    vr_src1[FU_MEM] = vr_a_readdataout[banksel_s4[FU_MEM]];
    vr_src2[FU_MEM] = vr_b_readdataout[banksel_s4[FU_MEM]];
    vmask[FU_MEM] =  vlane_en[FU_MEM] &
           ((ctrl4_ismasked[FU_MEM]) ?  
             vf_a_readdataout[banksel_s4[FU_MEM]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vr_src1[FU_MATMUL] =(src1scalar_s4[FU_MATMUL]) ? {NUMLANES{vs_s4[FU_MATMUL][LANEWIDTH-1:0]}} : 
                                    vr_a_readdataout[banksel_s4[FU_MATMUL]];
    vr_src2[FU_MATMUL] =(src2scalar_s4[FU_MATMUL]) ? {NUMLANES{vs_s4[FU_MATMUL][LANEWIDTH-1:0]}} : 
                                    vr_b_readdataout[banksel_s4[FU_MATMUL]];
    vr_src1[FU_BFADDER] =(src1scalar_s4[FU_BFADDER]) ? {NUMLANES{vs_s4[FU_BFADDER][LANEWIDTH-1:0]}} : 
                                    vr_a_readdataout[banksel_s4[FU_BFADDER]];
    vr_src2[FU_BFADDER] =(src2scalar_s4[FU_BFADDER]) ? {NUMLANES{vs_s4[FU_BFADDER][LANEWIDTH-1:0]}} : 
                                    vr_b_readdataout[banksel_s4[FU_BFADDER]];
    vr_src1[FU_BFMULT] =(src1scalar_s4[FU_BFMULT]) ? {NUMLANES{vs_s4[FU_BFMULT][LANEWIDTH-1:0]}} : 
                                    vr_a_readdataout[banksel_s4[FU_BFMULT]];
    vr_src2[FU_BFMULT] =(src2scalar_s4[FU_BFMULT]) ? {NUMLANES{vs_s4[FU_BFMULT][LANEWIDTH-1:0]}} : 
                                    vr_b_readdataout[banksel_s4[FU_BFMULT]];
    vmask[FU_MATMUL] =  vlane_en[FU_MATMUL] &
           ((ctrl4_ismasked[FU_MATMUL]) ?  
             vf_a_readdataout[banksel_s4[FU_MATMUL]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vmask[FU_BFADDER] =  vlane_en[FU_BFADDER] &
           ((ctrl4_ismasked[FU_BFADDER]) ?  
             vf_a_readdataout[banksel_s4[FU_BFADDER]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vmask[FU_BFMULT] =  vlane_en[FU_BFMULT] &
           ((ctrl4_ismasked[FU_BFMULT]) ?  
             vf_a_readdataout[banksel_s4[FU_BFMULT]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    for (fn2=FU_FALU; fn2<=FU_FALU+(NUMBANKS-1)*ALUPERBANK; fn2=fn2+1)
    begin
      vf_src1[fn2] =(src1scalar_s4[fn2]&ctrl4_vf_a_sel[fn2-FU_FALU]) ? {NUMLANES{vs_s4[fn2][0]}} :
        vf_a_readdataout[banksel_s4[fn2]*NUMLANES +: NUMLANES];

      //Only FALU uses this
      vf_src2[fn2] = (src2scalar_s4[fn2]) ? {NUMLANES{vs_s4[fn2][0]}} : 
        vf_b_readdataout[banksel_s4[fn2]*NUMLANES +: NUMLANES];
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
  //assign stall_memunit=_stall_memunit && (dst_we[FU_MEM][5] || (ctrl4_memunit_en && ~stall_mulunit && ~stall_matmul));
  assign stall_memunit= 1'b0;

//  pipereg #(1) memen5pipereg (
//      .d( ctrl4_mem_en ),
//      .clk(clk),
//      .resetn(resetn),
//      .en( pipe_advance[4] && ~_stall_memunit ),
//      .squashn( ~pipe_squash[4] | _stall_memunit ),
//      .q(ctrl5_mem_en ));

  pipereg #(1) memen5pipereg (
      .d( ctrl4_mem_en ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4]),
      .squashn( ~pipe_squash[4]),
      .q( ctrl5_mem_en ));

  //============== Memory Unit =============
 
 wire [127:0] dma_lane_wrdata,dma_lane_rddata;
// wire [NUMLANES-1:0] vmem_local_en;
 wire [11*NUMLANES-1:0] dma_lane_addr;

// assign vmem_local_en = {NUMLANES{pipe_advance[4]}};

 wire squash_per_lane_mem_dstpipe_NC;
 wire squash_per_lane_mem_dstmask_NC;

pipe #(REGIDWIDTH,3) lane_mem_dstpipe (
  .d( dst_s4[FU_MEM] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash(squash_per_lane_mem_dstpipe_NC),
  .q({dst[FU_MEM][7],dst[FU_MEM][6],dst[FU_MEM][5],dst[FU_MEM][4]}));

pipe #(1,3) lane_mem_dstwepipe (
  .d( dst_we_s4[FU_MEM]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash( pipe_squash[6:4] ),
  .q({dst_we[FU_MEM][7],dst_we[FU_MEM][6],dst_we[FU_MEM][5],dst_we[FU_MEM][4]}));

pipe #(NUMLANES,3) lane_mem_dstmaskpipe (
  .d( vmask[FU_MEM]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[4] ),
  .squash(squash_per_lane_mem_dstmask_NC),
  .q({dst_mask[FU_MEM][7],dst_mask[FU_MEM][6],dst_mask[FU_MEM][5],dst_mask[FU_MEM][4]}));

wire     [ LANEWIDTH*NUMLANES-1 : 0 ]   load_mem_result_s5;

 vmem_local inst_vmem_local(
   .clk(clk),
   .resetn(resetn),
   .en(pipe_advance[4]),
   .op(ctrl4_memunit_op),
   .address_a(vbase_s4[10:0]),
   .stride_val_a(vstrideoffset_s4[VCWIDTH-1:0]),
   .offset_a(vr_src1[FU_MEM]),
   .data_a(vr_src2[FU_MEM]),
   .out_a(load_mem_result_s5),
   .address_b(dma_lane_addr),
   .rden_b(dma_lane_rden),
   .wren_b(dma_lane_wren),
   .data_b(dma_lane_wrdata),
   .out_b(dma_lane_rddata)
  );

  dma #(
    .NUMLANES(NUMLANES),
    .WIDTH(LANEWIDTH),
    .ADDRWIDTH(11),
    .DMEM_ADDRWIDTH(32)
  )inst_dma(
   .clk(clk),
   .resetn(resetn),
   .mem_addr(mem_addr),
   .num_bytes(num_bytes),
   .dma_en(dma_en),
   .lane_addr(lane_addr),
   .we(dma_we),
  
   .local_addr(dma_lane_addr),
   .local_wren(dma_lane_wren),
   .local_rden(dma_lane_rden),
   .local_wrdata(dma_lane_wrdata),
   .local_rddata(dma_lane_rddata),
  
   .dma_busy(dma_busy),
   .dbus_address   (dma_dbus_address   ),
   .dbus_readdata  (dma_dbus_readdata  ), 
   .dbus_writedata (dma_dbus_writedata ),
   .dbus_byteen    (dma_dbus_byteen    ),
   .dbus_en        (dma_dbus_en        ),
   .dbus_wren      (dma_dbus_wren      ),
   .dbus_prefetch  (dma_dbus_prefetch  ),
   .dbus_wait      (dma_dbus_wait      ),
   .dbus_data_valid(dma_dbus_data_valid)
  );

  vmem_unit vmem_unit(
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
    //.out_dst(dst[FU_MEM][5]),
    .out_dst(),
    //.out_dst_we(dst_we[FU_MEM][5]),
    .out_dst_we(),
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
  defparam 
    vmem_unit.VPUWIDTH=LANEWIDTH,         // Width of the Vector processing unit
    vmem_unit.NUMLANES=NUMLANES,      // Number of vector lanes
    vmem_unit.LOG2NUMLANES=LOG2NUMLANES, 
    vmem_unit.NUMPARALLELLANES=NUMMEMPARALLELLANES,   // Crossbar size
    vmem_unit.LOG2NUMPARALLELLANES=LOG2NUMMEMPARALLELLANES, 
    vmem_unit.CONTROLWIDTH=VCWIDTH,   // Width of the Control Registers
    vmem_unit.DMEM_WRITEWIDTH=DMEM_WRITEWIDTH, // Width of write bus to memory
    vmem_unit.LOG2DMEM_WRITEWIDTH=LOG2DMEM_WRITEWIDTH,
    vmem_unit.DMEM_READWIDTH=DMEM_READWIDTH,   // Width of read bus from memory
    vmem_unit.LOG2DMEM_READWIDTH=LOG2DMEM_READWIDTH,
    vmem_unit.ELMIDWIDTH=REGIDWIDTH-VRIDWIDTH,
    vmem_unit.REGIDWIDTH=REGIDWIDTH;

  assign dst[FU_MEM][4]=dst_s4[FU_MEM];
  assign dst_we[FU_MEM][4]=dst_we_s4[FU_MEM];

  //============== Multiplier Unit (spans stages 4-6) =============

  vmul_unit vmul_unit(
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
  defparam 
    vmul_unit.LOG2WIDTH=LOG2LANEWIDTH,   // Width of the Vector processing unit
    vmul_unit.NUMMULLANES=NUMMULLANES,   // Number of vector lanes
    vmul_unit.LOG2NUMLANES=LOG2NUMLANES, // Number of vector lanes
    vmul_unit.REGIDWIDTH=REGIDWIDTH;

  //============== ALU Unit =============

  //If APB value is true, create one ALU per bank (per lane)

  generate
  for (bk=0; bk<1+(NUMBANKS-1)*ALUPERBANK; bk=bk+1)
  begin : aluperbank_gen
    for (k=0; k<NUMLANES; k=k+1)
    begin : lanes4_gen

      vlane_alu #(LANEWIDTH) valu(
        .clk(clk), 
        .resetn(resetn),
        .pipe_en(pipe_advance[4]),
        .pipe_squashn(~pipe_squash[4]),
        .src1(vr_src1[FU_ALU+bk][LANEWIDTH*k +: LANEWIDTH]),
        .src2(vr_src2[FU_ALU+bk][LANEWIDTH*k +: LANEWIDTH]),
        .mask(vf_src1[FU_ALU+bk][k]),  //Note: MERGE is not masked
        .op(ctrl4_alu_op[bk]^ALUOP_ZERO),
        .satsum_op(ctrl4_satsum_op[bk]),
        .satsize_op(ctrl4_satsize_op[bk]),
        .cmp_result(alu_cmpresult_s4[bk][k]),
        .result(alu_result_s5[bk][LANEWIDTH*k +: LANEWIDTH])
        );

      vlane_flagalu vflagalu(
        .clk(clk), 
        .resetn(resetn),
        .src1(vf_src1[FU_FALU+bk][k]),
        .src2(vf_src2[FU_FALU+bk][k]),
        .op(ctrl4_flagalu_op[bk]),
        .result(flagalu_result_s4[bk][k])
        );

      pipereg #(1) flagaluresultreg (
        .d( (ctrl4_vf_wbsel[bk]) ? alu_cmpresult_s4[bk][k] : 
                                   flagalu_result_s4[bk][k]),
        .clk(clk),
        .resetn(resetn),
        .en( pipe_advance[4] ), //Advance unless next stage stalled
        .squashn( 1'b1 ),
        .q(flagalu_result_s5[bk][k])
        );

    end

    wire squash_aludstpipe_NC;
    pipe #(REGIDWIDTH,1) aludstpipe (
      .d( dst_s4[FU_ALU+bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_aludstpipe_NC),
      .q({dst[FU_ALU+bk][5],dst[FU_ALU+bk][4]}));

    pipe #(1,3) aludstwepipe (
      .d( dst_we_s4[FU_ALU+bk] & ~ctrl4_vf_wbsel[bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:4] ),
      .squash( pipe_squash[6:4] ),
      .q({dst_we[FU_ALU+bk][5],dst_we[FU_ALU+bk][4]}));

    wire squash_aludstmaskpipe_NC;
    pipe #(NUMLANES,1) aludstmaskpipe (
      .d( vmask[FU_ALU+bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_aludstmaskpipe_NC),
      .q({dst_mask[FU_ALU+bk][5],dst_mask[FU_ALU+bk][4]}));

    wire squash_faludstpipe_NC;
    pipe #(REGIDWIDTH,1) faludstpipe (
      .d( dst_s4[FU_FALU+bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_faludstpipe_NC),
      .q({dst[FU_FALU+bk][5],dst[FU_FALU+bk][4]}));

    pipe #(1,3) faludstwepipe (
      .d( dst_we_s4[FU_FALU+bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:4] ),
      .squash( pipe_squash[6:4] ),
      .q({dst_we[FU_FALU+bk][5],dst_we[FU_FALU+bk][4]}));
  end
  endgenerate

 //This code is just for assigning from signals connected
 //to the matmul, which are linear multi bit signals, to the
 //multi-dimensional signals.
 wire [NUMLANES*REGIDWIDTH-1:0] dst_matmul;
 wire [NUMLANES-1:0] dst_we_matmul;
 wire [NUMLANES*NUMLANES-1:0] dst_mask_matmul;

 assign dst[FU_MATMUL][4] = dst_matmul[(REGIDWIDTH*NUMLANES)-1:REGIDWIDTH*(NUMLANES-1)];
 assign dst_mask[FU_MATMUL][4] = dst_mask_matmul[(NUMLANES*NUMLANES)-1:NUMLANES*(NUMLANES-1)];
 assign dst_we[FU_MATMUL][4] = dst_we_matmul[NUMLANES-1];


///////////////////////////
// Matmul unit
///////////////////////////
//

wire out_data_avail;

matmul_unit #(REGIDWIDTH,`MATMUL_STAGES,NUMLANES) u_matmul(
.clk(clk),
.resetn(resetn),
.activate(ctrl4_matmul_en),
.en({NUMLANES{pipe_advance[6]}}),
.squash({NUMLANES{pipe_squash[6]}}),
.stall(stall_matmul),
.stall_matmul(temp_stall_matmul),
.a_data(vr_src1[FU_MATMUL][NUMLANES*LANEWIDTH-1:0]),
.b_data(vr_src2[FU_MATMUL][NUMLANES*LANEWIDTH-1:0]),
.validity_mask_a_rows(matmul_masks_in[1*`MAT_MUL_SIZE-1:0*`MAT_MUL_SIZE]),
.validity_mask_a_cols(matmul_masks_in[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE]),
.validity_mask_b_rows(matmul_masks_in[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE]),
.validity_mask_b_cols(matmul_masks_in[3*`MAT_MUL_SIZE-1:2*`MAT_MUL_SIZE]),
.c_data(matmul_out), 
.vmask(vmask[FU_MATMUL]),
.in_dst(dst_s4[FU_MATMUL]),
.in_dst_we(dst_we_s4[FU_MATMUL]),
.out_dst(dst_matmul),
.out_dst_we(dst_we_matmul),
.out_data_avail(out_data_avail),
.out_dst_mask(dst_mask_matmul)
);

trp_unit #(REGIDWIDTH) u_trp (
.clk(clk),
.resetn(resetn),
.en(ctrl4_trp_en),
.a(vr_src1[FU_TRP][NUMLANES*LANEWIDTH-1:0]),
.mode(),
.read(),
.busy(),
.valid(),
.out(trp_out));

///////////////////////////
// Bfloat unit
///////////////////////////
genvar g_func;
wire[5*NUMLANES-1:0] bfadd_excp;
wire[5*NUMLANES-1:0] bfmult_excp;
wire bfadder_output_valid;
wire bfmult_output_valid;

wire squash_bfadder_dstmask_NC;
wire squash_bfadder_dstpipe_NC;
wire squash_bfmult_dstmask_NC;
wire squash_bfmult_dstpipe_NC;

pipe #(REGIDWIDTH,3) bfadddstpipe (
  .d( dst_s4[FU_BFADDER] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash(squash_bfadder_dstpipe_NC),
  .q({dst[FU_BFADDER][7],dst[FU_BFADDER][6],dst[FU_BFADDER][5],dst[FU_BFADDER][4]}));

pipe #(1,3) bfadddstwepipe (
  .d( dst_we_s4[FU_BFADDER]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash( pipe_squash[6:4] ),
  .q({dst_we[FU_BFADDER][7],dst_we[FU_BFADDER][6],dst_we[FU_BFADDER][5],dst_we[FU_BFADDER][4]}));

pipe #(NUMLANES,3) bfadddstmaskpipe (
  .d( vmask[FU_BFADDER] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash(squash_bfadder_dstmask_NC),
  .q({dst[FU_BFMULT][7],dst[FU_BFMULT][6],dst_mask[FU_BFADDER][5],dst_mask[FU_BFADDER][4]}));

  generate
  for(g_func =0; g_func <NUMLANES; g_func = g_func+1) begin

FPAddSub bfloat_add(
	.clk(clk),
	.rst(~resetn),
        .en(ctrl4_bfadder_en),
	.a(vr_src1[FU_BFADDER][g_func * LANEWIDTH +: LANEWIDTH]),
	.b(vr_src2[FU_BFADDER][g_func * LANEWIDTH +: LANEWIDTH]),
	.operation(1'b0),
	.result(bfadder_result_s5[g_func*LANEWIDTH +: LANEWIDTH]),
        .valid(bfadder_output_valid),
	.flags(bfadd_excp[5*g_func +: 5]));

FPMult_16 bfloat_mult(
	.clk(clk),
	.rst(~resetn),
        .en(ctrl4_bfmult_en),
	.a(vr_src1[FU_BFMULT][g_func * LANEWIDTH +: LANEWIDTH]),
	.b(vr_src1[FU_BFMULT][g_func * LANEWIDTH +: LANEWIDTH]),
	.result(bfmult_result_s5[g_func*LANEWIDTH +: LANEWIDTH]),
        .valid(bfmult_output_valid),
	.flags(bfmult_excp[5*g_func +: 5]));


pipe #(REGIDWIDTH,3) bfmultdstpipe (
  .d( dst_s4[FU_BFMULT+g_func] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash(squash_per_lane_mem_NC),
  .q({dst[FU_BFMULT+g_func][7],dst[FU_BFMULT+g_func][6],dst[FU_BFMULT+g_func][5],dst[FU_BFMULT+g_func][4]}));

pipe #(1,3) bfmultdstwepipe (
  .d( dst_we_s4[FU_BFMULT+g_func] & ~ctrl4_vf_wbsel[g_func] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash( pipe_squash[6:4] ),
  .q({dst_we[FU_BFMULT+g_func][7],dst_we[FU_BFMULT+g_func][6],dst_we[FU_BFMULT+g_func][5],dst_we[FU_BFMULT+g_func][4]}));

pipe #(NUMLANES,3) bfmultdstmaskpipe (
  .d( vmask[FU_BFMULT+g_func] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[4] ),
  .squash(squash_bfmult_dstmask_NC),
  .q({dst_mask[FU_BFMULT+g_func][7],dst_mask[FU_BFMULT+g_func][6],dst_mask[FU_BFMULT+g_func][5],dst_mask[FU_BFMULT+g_func][4]}));

//    bfloat_adder #(REGIDWIDTH) bf_add(
//    .clk(clk),
//    .resetn(resetn),
//    .en(ctrl4_bfadder_en),
//    .stall(stall_bf_adder),
//    .a(vr_src1[FU_BFADDER][g_func * LANEWIDTHE +: LANEWIDTH]),
//    .b(vr_src2[FU_BFADDER][g_func * LANEWIDTHE +: LANEWIDTH]),
//    .out(bfadder_result_s5[g_func*LANEWIDTH +: LANEWIDTH])
//    );
//    
//    bfloat_mult #(REGIDWIDTH) bf_mult(
//    .clk(clk),
//    .resetn(resetn),
//    .en(ctrl4_bfmult_en),
//    .stall(stall_bf_adder),
//    .a(vr_src1[FU_BFMULT][g_func * LANEWIDTHE +: LANEWIDTH]),
//    .b(vr_src2[FU_BFMULT][g_func * LANEWIDTHE +: LANEWIDTH]),
//    .out(bfmult_result_s5[g_func*LANEWIDTH +: LANEWIDTH])
//    );
 
///////////////////////////
// activation unit
///////////////////////////

    activation #(REGIDWIDTH) inst_activation(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_ACT][g_func * LANEWIDTHE +: LANEWIDTH]),
    .out(act_result_s5[g_func*LANEWIDTH +: LANEWIDTH])
    );
  end
 endgenerate


/******************************************************************************/
/************************** WB Pipeline Stage ********************************/
/******************************************************************************/

  generate
  for (ba=0; ba<1+(NUMBANKS-1)*ALUPERBANK; ba=ba+1)
  begin : wbaluperbank_gen
    assign wb_dst[FU_ALU+ba]=dst[FU_ALU+ba][5];
    assign wb_dst_we[FU_ALU+ba]=dst_we[FU_ALU+ba][5] && ~pipe_squash[5];
    assign wb_dst_mask[FU_ALU+ba]=dst_mask[FU_ALU+ba][5];
    assign D_wb_last_subvector[FU_ALU+ba]=D_last_subvector_s5[FU_ALU+ba];

    assign D_wb_last_subvector[FU_FALU+ba]=D_last_subvector_s5[FU_FALU+ba];
  end
  endgenerate

  assign wb_dst[FU_MEM]=dst[FU_MEM][5];
  assign wb_dst_we[FU_MEM]=dst_we[FU_MEM][5] && (~pipe_advance[5]|~pipe_squash[5]);
  assign wb_dst_mask[FU_MEM]=dst_mask[FU_MEM][5];
  assign D_wb_last_subvector[FU_MEM]=D_last_subvector_s5[FU_MEM];

  assign wb_dst[FU_MUL]=dst[FU_MUL][5];
  assign wb_dst_we[FU_MUL]=dst_we[FU_MUL][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_MUL]=dst_mask[FU_MUL][5];
  assign D_wb_last_subvector[FU_MUL]=D_last_subvector_s5[FU_MUL];


  assign wb_dst[FU_BFADDER] = dst[FU_BFADDER][7];
  assign wb_dst_we[FU_BFADDER] = dst_we[FU_BFADDER][7] && ~pipe_squash[7];
  assign wb_dst_mask[FU_BFADDER] = dst_mask[FU_BFADDER][7];
  assign D_wb_last_subvector[FU_BFADDER] = D_last_subvector_s5[FU_BFADDER];

  assign wb_dst[FU_BFMULT] = dst[FU_BFMULT][7];
  assign wb_dst_we[FU_BFMULT] = dst_we[FU_BFMULT][7] && ~pipe_squash[7];
  assign wb_dst_mask[FU_BFMULT] = dst_mask[FU_BFMULT][7];
  assign D_wb_last_subvector[FU_BFMULT] = D_last_subvector_s5[FU_BFMULT];

  assign wb_dst[FU_ACT] = dst[FU_ACT][5];
  assign wb_dst_we[FU_ACT] = dst_we[FU_ACT][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_ACT] = dst_mask[FU_ACT][5];
  assign D_wb_last_subvector[FU_ACT] = D_last_subvector_s5[FU_ACT];

  assign wb_dst[FU_MATMUL]=dst[FU_MATMUL][4];
  assign wb_dst_we[FU_MATMUL]=dst_we[FU_MATMUL][4] && out_data_avail;
  assign wb_dst_mask[FU_MATMUL]=dst_mask[FU_MATMUL][4];
  //TODO: There is no code that assigns to the s31 var used below. Need to add that code
  //This is only a debug var, so it doesn't affect functionality
  assign D_wb_last_subvector[FU_MATMUL]=D_last_subvector_s31[FU_MATMUL];

  // ******************  Map functional units to banks ******************
  always@*
    for (bw=0; bw<NUMBANKS; bw=bw+1)
    begin
      vr_c_we[bw]=(wb_dst_we[FU_MUL] && `LO(wb_dst[FU_MUL],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_MATMUL] && `LO(wb_dst[FU_MATMUL],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_BFADDER] && `LO(wb_dst[FU_BFADDER],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_BFMULT] && `LO(wb_dst[FU_BFMULT],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_ACT] && `LO(wb_dst[FU_ACT],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_MEM] && `LO(wb_dst[FU_MEM],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_ALU] && `LO(wb_dst[FU_ALU],LOG2NUMBANKS)==bw && ALUPERBANK==0) ||
                  (wb_dst_we[FU_ALU+bw] && ALUPERBANK!=0);

      //TODO: Update this code for matmul. This is for debug only, so skipping it for now.
      //Tells test_bench when to record register file contents.
      //Record if instruction writes to VRF, on last subvector, and not stalled
      D_wb_instrdone[bw] = pipe_advance[5] && (
        ((ALUPERBANK==0) ?
          (dst_we[FU_ALU][5] && D_wb_last_subvector[FU_ALU] && `LO(wb_dst[FU_ALU],LOG2NUMBANKS)==bw) :
          (dst_we[FU_ALU+bw][5] && D_wb_last_subvector[FU_ALU+bw])) || 
        (dst_we[FU_MUL][5] && D_wb_last_subvector[FU_MUL] && `LO(wb_dst[FU_MUL],LOG2NUMBANKS)==bw) || 
        (dst_we[FU_MEM][5] && D_wb_last_subvector[FU_MEM] && `LO(wb_dst[FU_MEM],LOG2NUMBANKS)==bw));

      //Take matmul output
      if (wb_dst_we[FU_MATMUL] && `LO(wb_dst[FU_MATMUL],LOG2NUMBANKS)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_MATMUL];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_MATMUL]>>LOG2NUMBANKS;
        vr_c_reg[bw]= wb_dst[FU_MATMUL];
        vr_c_writedatain[bw]= matmul_out;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MATMUL];
      end      
      else if (wb_dst_we[FU_TRP] && `LO(wb_dst[FU_TRP],LOG2NUMBANKS)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_TRP];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_TRP]>>LOG2NUMBANKS;
        vr_c_reg[bw]= wb_dst[FU_TRP];
        vr_c_writedatain[bw]= bfadder_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_TRP];
      end
      //Take bfadder output
      else if (wb_dst_we[FU_BFADDER] && `LO(wb_dst[FU_BFADDER],LOG2NUMBANKS)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_BFADDER];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_BFADDER]>>LOG2NUMBANKS;
        vr_c_reg[bw]= wb_dst[FU_BFADDER];
        vr_c_writedatain[bw]= bfadder_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_BFADDER];
      end
      else if (wb_dst_we[FU_BFMULT] && `LO(wb_dst[FU_BFMULT],LOG2NUMBANKS)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_BFMULT];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_BFMULT]>>LOG2NUMBANKS;
        vr_c_reg[bw]= wb_dst[FU_BFMULT];
        vr_c_writedatain[bw]= bfmult_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_BFMULT];
      end
      else if (wb_dst_we[FU_ACT] && `LO(wb_dst[FU_ACT],LOG2NUMBANKS)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_ACT];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_ACT]>>LOG2NUMBANKS;
        vr_c_reg[bw]= wb_dst[FU_ACT];
        vr_c_writedatain[bw]= act_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_ACT];
      end
      //Take multiplier output
      else if (wb_dst_we[FU_MUL] && `LO(wb_dst[FU_MUL],LOG2NUMBANKS)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_MUL];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_MUL]>>LOG2NUMBANKS;
        vr_c_reg[bw]= wb_dst[FU_MUL];
        vr_c_writedatain[bw]= mulshift_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MUL];
      end
      //Take Memory unit output
      else if (wb_dst_we[FU_MEM] && `LO(wb_dst[FU_MEM],LOG2NUMBANKS)==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_MEM];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_MEM]>>LOG2NUMBANKS;
        vr_c_reg[bw]= wb_dst[FU_MEM];
        vr_c_writedatain[bw]= load_mem_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MEM];
      end
      else
      //Take ALU output
      begin
        vmask_final[bw]=wb_dst_mask[FU_ALU+bw*ALUPERBANK];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_ALU+bw*ALUPERBANK]>>LOG2NUMBANKS;
        vr_c_reg[bw]= wb_dst[FU_ALU+bw*ALUPERBANK];
        vr_c_writedatain[bw]= alu_result_s5[bw*ALUPERBANK];
        //Do ALU and FALU for last subvector
        D_last_subvector_done[bw]=
          (D_wb_last_subvector[FU_ALU+bw*ALUPERBANK] && `LO(wb_dst[FU_ALU+bw*ALUPERBANK],LOG2NUMBANKS)==bw) |
          (D_wb_last_subvector[FU_FALU+bw*ALUPERBANK] && `LO(dst[FU_FALU+bw*ALUPERBANK][5],LOG2NUMBANKS)==bw);
      end
      for (j=0; j<NUMLANES; j=j+1)
        vr_c_byteen[bw*VPW*NUMLANES + j*VPW +: VPW ]={VPW{vmask_final[bw][j]}};

      //*********** Flag writeback ***********
      vf_c_reg[bw*BANKREGIDWIDTH+:BANKREGIDWIDTH]=dst[FU_FALU+bw*ALUPERBANK][5]>>LOG2NUMBANKS;
      vf_c_writedatain[bw*NUMLANES+:NUMLANES]=flagalu_result_s5[bw*ALUPERBANK];
      vf_c_we[bw]=dst_we[FU_FALU+bw*ALUPERBANK][5] && `LO(dst[FU_FALU+bw*ALUPERBANK][5],LOG2NUMBANKS)==bw;
    end

  //********** Scalar writeback ***********
  assign vs_wetrack={ctrl_vs_we[5:4],|ctrl3_vs_we,ctrl2_vs_we};
  assign vs_we=ctrl_vs_we[5] & load_result_mask_s5[0];
  assign vs_dst=wb_dst[FU_MEM][`VRID_RANGE];
  assign vs_writedata=load_result_s5[LANEWIDTH-1:0];

endmodule
