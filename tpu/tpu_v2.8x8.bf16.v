

`define NO_PLI 1
//`define TEST_BENCH 1
`define USE_INHOUSE_LOGIC
//`define SIMULATION_MEMORY
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



`define DWIDTH 16
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

 
/******
 * 1-cycle cache (not including initial registering)
 *    0th cycle: register addresses into block RAMs
 *    1st cycle: Read out tags and status and do comparison
 */

module mem_icache_12_7 (

    resetn,

    // CPU data bus
    bus_clk,
    bus_address,
    bus_readdata,
    bus_writedata,
    bus_byteen,
    bus_en,
    bus_wren,
    bus_wait,

    // Mem hierarchy
    mem_clk,
    mem_filladdr,
    mem_filldata,
    mem_fillwe,

    bus_flush,    // runs on mem_clk

    // Cache signals
    cache_hit,
    cache_miss,

    mem_icache_address,
    mem_icache_data,
    mem_icache_out,
    mem_icache_byteen,
    mem_icache_wren
    );

                // In bits, subtract 3 for bytes
parameter CACHELINESIZE=2**12; // 128

parameter CACHEDEPTH=2**7;  // 64

parameter TAGSIZE=32-7-12+3;

`define TAGRANGE 31:32-TAGSIZE
`define OFFSETRANGE 7-1+12-3:12-3
// selects 32-bit word
`define OFFSETWORDRANGE 7-1+12-3:2


input           resetn;

// CPU data bus
input           bus_clk;
input   [31:0]  bus_address;
output  [31:0]  bus_readdata;
input   [31:0]  bus_writedata;
input   [3:0]   bus_byteen;
input           bus_en;
input           bus_wren;
output          bus_wait;
input           bus_flush;

// Mem hierarchy
input                       mem_clk;
input [31:0]                mem_filladdr;
input [CACHELINESIZE-1:0]   mem_filldata;
input                       mem_fillwe;

// Cache signals
output cache_hit;
output cache_miss;

//wire [TAGSIZE-1:0]       cache_tagout;
//wire                     cache_validout;
wire [32-1:0]            cache_dataout;
//wire [CACHELINESIZE-1:0] cache_lineout;

input  [31:0]                mem_icache_address;
input  [CACHELINESIZE-1:0]   mem_icache_data;
output [31:0]                mem_icache_out;
input  [CACHELINESIZE/8-1:0] mem_icache_byteen;
input                        mem_icache_wren;

//wire                     tagmatch;
reg  [32-1:0] bus_address_saved;

parameter [1:0] S_IDLE=2'b00, S_TAGLOOKUP=2'b01, S_RESULT=2'b10;
//reg [1:0] cache_state;

wire  cache_hit;

//reg  [7-1:0] count;

//  always@(posedge mem_clk)
//    if (bus_flush)
//      count <= count + 1'b1;

 dpram1_26_67108864_32 data1 (
    .clk(bus_clk),
    .address_a(bus_address[27:2]),
    .address_b(mem_icache_address[25:0]),
    .wren_a(1'b0),
    .wren_b(mem_icache_wren),
    .data_a(0),
    .data_b(mem_icache_data),
    .byteen_a(-1),
    .byteen_b(mem_icache_byteen),
    .out_a(cache_dataout),
    .out_b(mem_icache_out)
 );


  assign bus_readdata=cache_dataout;

    // Save offset to make sure still asking for same data
  always@(posedge bus_clk or negedge resetn)
    if (!resetn)
      bus_address_saved<=0;
    else 
      bus_address_saved<=bus_address;

  assign cache_hit = 1'b1;
  assign cache_miss=bus_en && ~cache_hit;
  assign bus_wait=bus_en & ~cache_hit;
endmodule
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
`ifndef MODULE_PIPE_1_28
`define MODULE_PIPE_1_28
module pipe_1_28(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [1-1:0]  d;
input              clk;
input              resetn;
input  [28-1:0] en;
input  [28-1:0] squash;
output [1*(28+1)-1:0] q;

reg [1*28-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 1-1:0 ]<= 0;
    else if (en[0])
      tq[ 1-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<28; i=i+1)
      if (!resetn || squash[i] )
        tq[i*1 +: 1 ]<= 0;
      else if (en[i])
        tq[i*1 +: 1 ]<=tq[(i-1)*1 +: 1 ];
  end

  assign q[1-1:0]=d;
  assign q[1*(28+1)-1:1]=tq;
endmodule
        `endif


/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
`ifndef MODULE_PIPE_8_28
`define MODULE_PIPE_8_28
module pipe_8_28(
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
input  [28-1:0] en;
input  [28-1:0] squash;
output [8*(28+1)-1:0] q;

reg [8*28-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 8-1:0 ]<= 0;
    else if (en[0])
      tq[ 8-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<28; i=i+1)
      if (!resetn || squash[i] )
        tq[i*8 +: 8 ]<= 0;
      else if (en[i])
        tq[i*8 +: 8 ]<=tq[(i-1)*8 +: 8 ];
  end

  assign q[8-1:0]=d;
  assign q[8*(28+1)-1:8]=tq;
endmodule
        `endif
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
`ifndef MODULE_PIPE_8_28
`define MODULE_PIPE_8_28
module pipe_8_28(
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
input  [28-1:0] en;
input  [28-1:0] squash;
output [8*(28+1)-1:0] q;

reg [8*28-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 8-1:0 ]<= 0;
    else if (en[0])
      tq[ 8-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<28; i=i+1)
      if (!resetn || squash[i] )
        tq[i*8 +: 8 ]<= 0;
      else if (en[i])
        tq[i*8 +: 8 ]<=tq[(i-1)*8 +: 8 ];
  end

  assign q[8-1:0]=d;
  assign q[8*(28+1)-1:8]=tq;
endmodule
        `endif


module bfloat_adder_16 (
 input clk,
 input resetn,
 input en,
 input stall,
 input [16-1:0] a,
 input [16-1:0] b,
 output reg[16-1:0] out
);

always@(posedge clk)begin
  if(!resetn)
    out <= 'h0;
  else
    if(en)
      out <= a + b;
end

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
        
`ifndef MODULE_RAM_WRAPPER_5_32_32 
`define MODULE_RAM_WRAPPER_5_32_32 
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
input [5-1:0] address_a;
input [5-1:0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [32-1:0] data_a;
input [32-1:0] data_b;
output [32-1:0] out_a;
output [32-1:0] out_b;

reg [5-1:0] q_address_a;
reg [5-1:0] q_address_b;
reg [5-1:0] mux_address_b;

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

endmodule
`endif



module vdispatcher_add_2_4 (
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
input [ 4-1:0 ] valuetoadd;

input [ 4-1:0 ] inshift_data;

input [ 2*4-1:0 ]  inparallel_data;
output [ 2*4-1:0 ] outparallel_data;

wire [ 4-1:0 ]  shiftin_right;

reg [ 2*4-1:0 ] outparallel_data;
reg [ 2*4-1:0 ] outparallel_added;

wire [ 2-1:0 ] squash_nc;
assign squash_nc = squash;

  assign shiftin_right = (!rotate) ? inshift_data : 
                    outparallel_added[2*4-1:(2-1)*4];

  reg [ 4-1:0 ] v;
  reg[31:0] i;
  always@*
    for (i=0; i<2; i=i+1)
    begin
      //Saturate at 0xffff... in either direction
      v=(increment[i] && outparallel_data[i*4 +: 4] != {4{1'b1}}) ?
        valuetoadd : 0;
      if (add_sub==0)
        outparallel_added[i*4 +: 4]=outparallel_data[i*4+:4] + v;
      else
        outparallel_added[i*4 +: 4]=outparallel_data[i*4+:4] - v;
    end

  always@(posedge clk)
    if (!resetn)
      outparallel_data=0;
    else if (load)
      outparallel_data=inparallel_data;
    else if (shift)
      outparallel_data=(outparallel_added<<4) | shiftin_right;
      


endmodule


module dpram_7_128_8 (
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
input [7-1:0] address_a;
input [7-1:0] address_b;
input  wren_a;
input  wren_b;
input [8-1:0] data_a;
input [8-1:0] data_b;
output [8-1:0] out_a;
output[8-1:0] out_b;

dual_port_ram u_dual_port_ram(
.addr1(address_a),
.we1(wren_a),
.data1(data_a),
.out1(out_a),
.addr2(address_b),
.we2(wren_b),
.data2(data_b),
.out2(out_b),
.clk(clk)
);

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

wire [1-1:0] temp_left;  // Sangram: added to remove warning
wire [1-1:0] temp_right;  // Sangram: added to remove warning
assign temp_left = shiftin_left; 
assign temp_right = shiftin_right; 

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
          inc Register File

   - Has one read port (a) and one write port (c)
   - inc0 fixed to 0 and unwriteable
****************************************************************************/
module vregfile_inc_32_8_3 (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

input clk;
input resetn;

input a_en;
input [3-1:0] a_reg,c_reg;
output [32-1:0] a_readdataout;
input [32-1:0] c_writedatain;
input c_we;

     ram_wrapper_3_8_32 reg_file1(
         .clk(clk),
         .resetn(resetn),
         .rden_a(1'b0),
         .rden_b(a_en),
         .address_a(c_reg[3-1:0]),
         .address_b(a_reg[3-1:0]),
         .wren_a(c_we&(|c_reg)),
         .wren_b(1'b0),
         .data_a(c_writedatain),
         .data_b(0),
         .out_a(),
         .out_b(a_readdataout)
     );

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
module local_mult_17_17_34(
dataa,
datab,
clock,
clken,
aclr,
result
);

input [17-1:0] dataa;
input [17-1:0] datab;
input clock;
input clken;
input aclr;
output reg [34-1:0] result;

wire [17-1:0] unsignedinputA;
wire [17-1:0] unsignedinputB;
wire [34-1:0] unsignedoutputP;

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
`ifndef MODULE_RAM_WRAPPER_7_128_128 
`define MODULE_RAM_WRAPPER_7_128_128 
module ram_wrapper_7_128_128 (
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
input [7-1:0] address_a;
input [7-1:0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [128-1:0] data_a;
input [128-1:0] data_b;
output [128-1:0] out_a;
output [128-1:0] out_b;

reg [7-1:0] q_address_a;
reg [7-1:0] q_address_b;
reg [7-1:0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_7_128_128 dpram1(
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
`endif

 

/******
 *                         +--fill_we
 *     +------------------|&|--------------------+
 *     +-----| D$ |                              |
 * fill ---- |    |---|cacheaddr==filladdr|------+
 *                    |\      &&          |   
 *                    |/   cache dirty    |
 *
 * Must ensure that prefetched cache lines don't overwrite dirty lines (we
 * evict the dirty lines and write them out to memory, BUT if we're prefetching
 * the same memory location that is dirty in the cache we have to cancel that
 * prefetch
 */

// `include "mem_wbbuffer.v"

module mem_dcache_wb_7_12_5 (

    resetn,

    // CPU data bus
    bus_clk,
    bus_address,
    bus_readdata,
    bus_readdata_line,
    bus_writedata,
    bus_byteen,
    bus_en,
    bus_wren,
    bus_dirty,
    bus_dirtywe,
    bus_wait,

    // Mem hierarchy
    mem_clk,

    mem_filladdr,   //For writing into cache
    mem_filldata,
    mem_fillwe,
    mem_fillrddirty,

    mem_wbaddr,   //For writing into memory from cache
    mem_wbdata,
    mem_wbwe,
    mem_wback,

    bus_flush,    // runs on mem_clk

    // Cache signals
    cache_hit,
    cache_miss,
 
    mem_dcache_address,
    mem_dcache_data,
    mem_dcache_out,
    mem_dcache_byteen,
    mem_dcache_wren
    );

// Only need to set THESE TWO PARAMETERS
// parameter 7=7;            // In bits, subtract 3 for bytes
// parameter 12=6;
// parameter 5=7;            // In bits, subtract 3 for bytes

parameter CACHELINESIZE=2**7; 
parameter CACHEDEPTH=2**12;  
parameter DRAMWIDTHBITS=2**5;

// parameter TAGSIZE=32-12-7+3;

// `define TAGRANGE 31:32-TAGSIZE
// `define OFFSETRANGE 12-1+7-3:7-3
// selects 32-bit word
// `define OFFSETWORDRANGE 12-1+7-3:2


input           resetn;

// CPU data bus
input           bus_clk;
input   [31:0]  bus_address;
output  [31:0]  bus_readdata;
output  [CACHELINESIZE-1:0] bus_readdata_line;
input   [CACHELINESIZE-1:0]  bus_writedata;
input   [CACHELINESIZE/8-1:0]   bus_byteen;
input           bus_en;
input           bus_wren;
input           bus_dirty;
input           bus_dirtywe;
output          bus_wait;
input           bus_flush;

// Mem hierarchy
input                       mem_clk;
input [31:0]                mem_filladdr;
input [CACHELINESIZE-1:0]   mem_filldata;
input                       mem_fillwe;
input                       mem_fillrddirty;

output [31:0]               mem_wbaddr; 
output [DRAMWIDTHBITS-1:0]  mem_wbdata;
output                      mem_wbwe;
input                       mem_wback;

// Cache signals
output cache_hit;
output cache_miss;

input  [31:0]                mem_dcache_address;
input  [CACHELINESIZE-1:0]   mem_dcache_data;
output [31:0]                mem_dcache_out;
input  [CACHELINESIZE/8-1:0] mem_dcache_byteen;
input                        mem_dcache_wren;


//wire [31:0]              mem_dirtyaddr; 
//wire [CACHELINESIZE-1:0] mem_dirtydata;
//wire                     mem_dirtywe;

//wire                     mem_validout;
//wire                     mem_dirtyout;
// wire [TAGSIZE-1:0]       mem_tagout;
//wire [CACHELINESIZE-1:0] mem_lineout;

// wire [TAGSIZE-1:0]       cache_tagout;
//wire                     cache_validout;
wire [CACHELINESIZE-1:0] cache_dataout;

//wire                     tagmatch;
reg  [32-1:0] bus_address_saved;
//wire  [12-1:0] mem_ndx_saved;

wire  cache_hit;

//reg  [12-1:0] count;

reg   [CACHELINESIZE/8-1:0]   bus_byteen_t;
wire [7-5-1:0] wordsel_saved;
reg  [31:0]  bus_readdata;
wire [CACHELINESIZE-1:0] bus_writedata_t;

//reg [31:0]                  mem_wbaddr_r;  // captures address for writeback
//reg [CACHELINESIZE-1:0]     mem_wbdata_r;
//wire                        mem_wbwe_pulse;
//wire                        mem_wbload;
//reg [31:0]                  mem_filladdr_r; // don't prefetch over dirty lines
//reg [31:0]                  mem_filladdr_r2;
//wire                        t_mem_fillwe;

//  always@(posedge mem_clk)
//    if (bus_flush)
//      count <= count + 1'b1;

 dpram1_22_67108864_512 data0 (
    .clk(bus_clk),
    .address_a(bus_address[28:7]),
    .address_b(mem_dcache_address[21:0]),
    .wren_a(bus_wren),
    .wren_b(mem_dcache_wren),
    .data_a(bus_writedata_t),
    .data_b(mem_dcache_data),
    .byteen_a(bus_byteen_t),
    .byteen_b(mem_dcache_byteen),
    .out_a(cache_dataout),
    .out_b(mem_dcache_out)
 );

    // Save offset to make sure still asking for same data
  always@(posedge bus_clk or negedge resetn)
    if (!resetn)
      bus_address_saved<=0;
    else 
      bus_address_saved<=bus_address;

  assign cache_hit = 1'b1;

  reg bus_en_r;
  always@(posedge bus_clk)
    bus_en_r<=bus_en;

  assign cache_miss= 1'b0;

  assign bus_wait= 1'b0;


  //Word muxing - assign byteena, datain, dataout
  assign wordsel_saved=bus_address_saved[7-3:3];


  always @*
      bus_byteen_t=bus_byteen;

  assign bus_writedata_t = bus_writedata;

//   reg [31:0] i;
  wire [31:0] bus_readdata_wire;
  assign bus_readdata_wire = cache_dataout >> (wordsel_saved*32);

  always @*
  begin
    // bus_readdata<=cache_dataout[31:0];
    // for (i=0; i<CACHELINESIZE/32; i=i+1)
    //   if (wordsel_saved==i)
        bus_readdata<=bus_readdata_wire;
  end


  assign bus_readdata_line=cache_dataout;
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

wire [ 2-1:0 ] squash_nc;
assign squash_nc = squash;

velmshifter_2_1 velmshift (
    .clk(clk),
    .resetn(resetn),      
    .load(load),
    .shift(shift),
    .dir_left(1),
    .squash(squash),
    .shiftin_left((!rotate) ? 0 : outparallel_data[1-1:0]),
    .shiftin_right((!rotate) ? inshift_data : outparallel_data[2*1-1:(2-1)*1]),
    .inpipe(inparallel_data),
    .outpipe(outparallel_data));


endmodule



/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_2_151 (
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

input [ 2*151-1:0 ]  inpipe;
output [ 2*151-1:0 ] outpipe;

input [ 151-1:0 ]  shiftin_left;
input [ 151-1:0 ]  shiftin_right;

wire [ (2+1)*151-1:0 ] _outpipe;

wire [151-1:0] temp_left;  // Sangram: added to remove warning
wire [151-1:0] temp_right;  // Sangram: added to remove warning
assign temp_left = shiftin_left; 
assign temp_right = shiftin_right; 

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_151 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[150:0],
      _outpipe[301:151], //Support 1 lane
      shiftin_right,
      _outpipe[150:0]);
 // defparam velmshifter_laneunit0.151=151;

  //Generate everything in between 


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_151 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[1],
      inpipe[301:151],
      shiftin_left,
      _outpipe[150:0], //L=1
      _outpipe[301:151]); //L=1
   // defparam velmshifter_laneunitlast.151=151;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
module dpram1_22_67108864_512 (
    clk,
    address_a,
    address_b,
    wren_a, 
    wren_b, 
    data_a,
    data_b,
    byteen_a,
    byteen_b,
    out_a,
    out_b
);

// parameter AWIDTH=10;
// parameter NUM_WORDS=1024;
// parameter DWIDTH=32;
// parameter LOG2DWIDTH = $clog2(DWIDTH);

input clk;
input [(22-1):0] address_a;
input [(22-1):0] address_b;
input  wren_a;
input  wren_b;
input [(512/8)-1:0] byteen_a;
input [(512/8)-1:0] byteen_b;
input [(512-1):0] data_a;
input [(512-1):0] data_b;
output reg [(512-1):0] out_a;
output reg [(512-1):0] out_b;

`ifdef SIMULATION_MEMORY

integer i;
integer k;

//reg [32-1:0] ram [67108864-1:0];
reg [(((1096-1)-(0)+1)*((32-1)-(0)+1))-1 : 0] ram;
reg [25:0] addr_a;
reg [25:0] addr_b;
 
initial
 begin
   //This is TOO big for 256 MB RAM!  We right shift data by 1
   //$readmemh("instr.dat",ram,'h100_0000);
   $readmemh("instr.dat",ram,'h100);
   //$readmemh("data.dat",ram,'h400_0000>>1);
   $readmemh("data.dat",ram,'h400>>1);
 end

always@(*) begin
    addr_a = address_a << 26-22;
    addr_b = address_b << 26-22;
end

always@(posedge clk) begin 
  if (wren_a) begin
      for(k=0; k < 512/32;k=k+1)begin
          for(i=0; i < 4 ;i=i+1)begin
              if(byteen_a[((512/8-1)-((4*k)+i))])
                  ram[addr_a+k][(i*8)+:8] <= data_a[((32*k)+(i*8))+:8];
          end
      end
  end
  else begin
      for(i=0; i < 512/32; i=i+1)begin
          out_a[(32*i)+:32] <= ram[addr_a+i];
      end
  end
  if (wren_b) begin
      for(k=0; k < 512/32;k=k+1)begin
          for(i=0; i < 4 ;i=i+1)begin
              if(byteen_b[((512/8-1)-((4*k)+i))])
                  ram[addr_b+k][(i*8)+:8] <= data_b[((32*k)+(i*8))+:8];
          end
      end
  end
  else begin
      for(i=0; i < 512/32; i=i+1)begin
          out_b[32*i+:32] <= ram[addr_b+i];
      end
  end
end
`else

// Not connected wires
wire [(512/8)-1:0] byteen_a_nc;
wire [(512/8)-1:0] byteen_b_nc;

assign byteen_a_nc = byteen_a;
assign byteen_b_nc = byteen_b;

dual_port_ram u_dual_port_ram(
.addr1(address_a[11:0]),
.we1(wren_a),
.data1(data_a),
.out1(out_a),
.addr2(address_b[11:0]),
.we2(wren_b),
.data2(data_b),
.out2(out_b),
.clk(clk)
);

`endif

endmodule
/****************************************************************************
          Generic Register
****************************************************************************/
module register_17(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [17-1:0] d;
output [17-1:0] q;
reg [17-1:0] q;

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

 

//`include "core.v"
//`include "mem_icache.v"
//`include "mem_dcache_wb.v"

module processor (
    clk,
    mem_clk,
    resetn,
    rt_dataout, // Dummy output - use this to prevent design from being
                // synthesized away if using on-chip memory

    ibus_en,
    ibus_address,
    ibus_wait,

    // Instruction cache fill - on DRAM clock 133MHz
    imem_filladdr,
    imem_filldata,
    imem_fillwe,

    dbus_address,
    dbus_readdata,
    dbus_writedata,
    dbus_byteen,
    dbus_en,
    dbus_wren,
    dbus_prefetch,
    dbus_wait,    //Goes high 1-cycle after dbus_en

    // PETES CHANGE for tracing
    trc_addr,
    trc_data,
    trc_we,
    trc_stall,
    trc_pipestall,

    // Data fill/WB 133MHz
    dmem_filladdr,
    dmem_filldata,
    dmem_fillwe,
    dmem_fillrddirty,
    dmem_wbaddr,
    dmem_wbdata,
    dmem_wbwe,
    dmem_wback,

    mem_dcache_address,
    mem_dcache_data,
    mem_dcache_out,
    mem_dcache_byteen,
    mem_dcache_wren,

    mem_icache_address,
    mem_icache_data,
    mem_icache_out,
    mem_icache_byteen,
    mem_icache_wren
    );

parameter DCACHEWIDTHBITS=2**7;

parameter ICACHEWIDTHBITS=2**7;

parameter DRAMWIDTHBITS=2**5;

input              clk;
input              mem_clk;
input              resetn;
output [31:0]      rt_dataout;

output        ibus_en;             // Instruction bus signals
output [31:0] ibus_address;
input         ibus_wait;

input [31:0]    imem_filladdr;
input [ICACHEWIDTHBITS-1:0]   imem_filldata;
input           imem_fillwe;

output [31:0] dbus_address;    // Data bus signals
input  [31:0] dbus_readdata;
output [31:0] dbus_writedata;
output [3:0]  dbus_byteen;
output        dbus_en;
output        dbus_wren;
output [31:0] dbus_prefetch;
input         dbus_wait;

    // Mem hierarchy
input [31:0]    dmem_filladdr;
input [DCACHEWIDTHBITS-1:0]   dmem_filldata;
input           dmem_fillwe;
input           dmem_fillrddirty;

output [31:0]   dmem_wbaddr;
output [DRAMWIDTHBITS-1:0]  dmem_wbdata;
output          dmem_wbwe;
input           dmem_wback;

// PETES CHANGE for tracing
output  [ 4 : 0 ]   trc_addr;
output  [ 31 : 0 ]  trc_data;
output              trc_we;
input               trc_stall;
output              trc_pipestall;

input  [31:0]                  mem_dcache_address;
input  [DCACHEWIDTHBITS-1:0]   mem_dcache_data;
output [DCACHEWIDTHBITS-1:0]   mem_dcache_out;
input  [DCACHEWIDTHBITS/8-1:0] mem_dcache_byteen;
input                          mem_dcache_wren;

input  [31:0]                  mem_icache_address;
input  [31-1:0]   mem_icache_data;
output [31-1:0]   mem_icache_out;
input  [32/8-1:0] mem_icache_byteen;
input                          mem_icache_wren;

  wire [31:0] dcpu_address;    // Processor's data bus signals
  wire [31:0] dcpu_writedata;
  wire [3:0]  dcpu_byteen;
  wire        dcpu_en;
  wire        dcpu_wren;
  wire [31:0] dcpu_prefetch;    // Processor's data bus signals
  wire [31:0] dcpu_readdata;
  wire [DCACHEWIDTHBITS-1:0] dcpu_readdata_line;
  wire [DCACHEWIDTHBITS-1:0] dcpu_writedata_line;
  wire [DCACHEWIDTHBITS/8-1:0]  dcpu_byteen_line;
  wire         dcpu_wait;


  wire        icache_en;             // Instruction bus signals
  wire [31:0] icache_address;
  wire [31:0] icache_readdata;
  wire        icache_wait;
  wire icache_hit;
  wire icache_miss;


  wire [31:0] dcache_address;    // Processor's data bus signals
  reg [DCACHEWIDTHBITS-1:0] dcache_writedata;
  reg [DCACHEWIDTHBITS/8-1:0]  dcache_byteen;
  wire        dcache_en;
  wire        dcache_wren;
  wire  [31:0] dcache_readdata;
  wire  [DCACHEWIDTHBITS-1:0] dcache_readdata_line;
  wire         dcache_wait;

  wire        dcache_hit;
  wire        dcache_miss;

  reg [3:0] dcache_state,icache_state;
  parameter [2:0] CACHE_IDLE=3'b000, 
          CACHE_TAGLOOKUP=3'b001,
          CACHE_ISSUE=3'b010,
          CACHE_STOREMISS=3'b011,
          CACHE_LOADMISS=3'b100;

  reg [4-1:0]  icache_timeout;
  reg [4-1:0]  dcache_timeout;

/*************************  Processor Core **************************/

  core c (
    .clk(clk),
    .resetn(resetn),
    .rt_dataout(rt_dataout),
    .ibus_en(icache_en),
    .ibus_address(icache_address),
    .ibus_readdata((icache_hit) ? icache_readdata : 0),
    .ibus_wait(icache_wait),

    // PETES CHANGE for tracing
    .trc_addr(trc_addr),
    .trc_data(trc_data),
    .trc_we(trc_we),
    .trc_stall(trc_stall),
    .trc_pipestall(trc_pipestall),

    .dbus_address(dcpu_address),
    .dbus_readdata(dcpu_readdata),
    .dbus_writedata(dcpu_writedata),
    .dbus_byteen(dcpu_byteen),
    .dbus_readdata_line(dcpu_readdata_line),
    .dbus_writedata_line(dcpu_writedata_line),
    .dbus_byteen_line(dcpu_byteen_line),
    .dbus_en(dcpu_en),
    .dbus_wren(dcpu_wren),
    .dbus_cachematch(dcache_hit),
    .dbus_cachemiss(dcache_miss),
    .dbus_prefetch(dcpu_prefetch),
    .dbus_wait(dcpu_wait) 
    );


/*************************  Instr Cache **************************/

  reg cache_flush;
  reg [31:0] next_address;

  always@(posedge mem_clk)begin
    cache_flush<=~resetn;
  end

  always@(posedge clk)begin
    if(resetn)
      next_address <= 0;
    else
      next_address<= icache_address;
  end

  mem_icache_12_7 icache1 (
    .resetn(resetn),

    // CPU data bus
    .bus_clk(clk),
    .bus_address(next_address),
    .bus_readdata(icache_readdata),
    .bus_writedata(0),
    .bus_byteen(0),
    .bus_en(icache_en),
    .bus_wren(0),
    .bus_wait(icache_wait),

    // Mem hierarchy
    .mem_clk(mem_clk),
    .mem_filladdr(imem_filladdr),
    .mem_filldata(imem_filldata),
    .mem_fillwe(imem_fillwe),
    .bus_flush(cache_flush), // Runs at 133 MHz

    // Cache signals
    .cache_hit(icache_hit),
    .cache_miss(icache_miss),

    .mem_icache_address  (mem_icache_address),
    .mem_icache_data     (mem_icache_data   ),
    .mem_icache_out      (mem_icache_out    ),
    .mem_icache_byteen   (mem_icache_byteen ),
    .mem_icache_wren     (mem_icache_wren   )
    );

  always@(posedge clk)
    if (!resetn)
      icache_state<=CACHE_IDLE;
    else
      case(icache_state)
        CACHE_IDLE:
          icache_state<= (icache_en && ~icache_address[31]) ? CACHE_TAGLOOKUP : 
                                                          CACHE_IDLE;
        CACHE_TAGLOOKUP:
          icache_state<= (icache_hit || ~icache_en) ? CACHE_IDLE : CACHE_ISSUE;
        CACHE_ISSUE:
          icache_state<= (ibus_wait) ? CACHE_ISSUE : CACHE_LOADMISS;
        CACHE_LOADMISS:
          icache_state<= (icache_hit || (|icache_timeout)==0) ?  CACHE_IDLE : 
                              CACHE_LOADMISS;
        default:
          icache_state<=CACHE_IDLE;
      endcase

      // Timeout if ibus_wait has been low for 4 cycles
  always@(posedge clk)
    if (icache_state==CACHE_ISSUE)
      icache_timeout<={4{1'b1}};
    else
      icache_timeout<=(ibus_wait<<3) | (icache_timeout>>1);

  // Send single pulse to bus, unless already a request stalling
  assign ibus_en= (icache_state==CACHE_ISSUE);

  always@(posedge clk)
    ibus_address <= icache_address;


/*************************  Data Cache **************************/


  assign dcache_address=dcpu_address;
  //Since we know we need 1st cycle just to check tags, we register the writes
  always@(posedge clk)
  begin
    dcache_writedata<=dcpu_writedata_line;
    dcache_byteen<=dcpu_byteen_line;
  end
  assign dcache_wren = dcpu_wren&dcache_hit&
        (dcache_state==CACHE_TAGLOOKUP || dcache_state==CACHE_STOREMISS);
  assign dcache_en = dcpu_en;

  mem_dcache_wb_7_12_5 dcache1 (
    .resetn(resetn),

    // CPU data bus    
    .bus_clk(clk),
    .bus_address(dcache_address),
    .bus_readdata(dcache_readdata),
    .bus_readdata_line(dcache_readdata_line),
    .bus_writedata(dcache_writedata),
    .bus_byteen(dcache_byteen),
    .bus_wren(dcache_wren),
    .bus_en(dcache_en),
    .bus_dirty(1'b1),   //Every store is dirty (even on store miss)
    .bus_dirtywe(dcache_wren),
    .bus_wait(dbus_wait),

    // Mem hierarchy
    .mem_clk(mem_clk),
    .mem_filladdr(dmem_filladdr),
    .mem_filldata(dmem_filldata),
    .mem_fillwe(dmem_fillwe),
    .mem_fillrddirty(dmem_fillrddirty),

    // Write back outputs for writing into DRAM, wbwe accounts for dirty bit
    .mem_wbaddr(dmem_wbaddr),
    .mem_wbdata(dmem_wbdata),
    .mem_wbwe(dmem_wbwe),
    .mem_wback(dmem_wback),

    .bus_flush(cache_flush), // Runs at 133 MHz
    // Cache signals
    .cache_hit(dcache_hit), //Checks only match - doesn't consider if en was on
    .cache_miss(dcache_miss), //Checks if not in the cache AND if enabled

    .mem_dcache_address(mem_dcache_address),
    .mem_dcache_data(mem_dcache_data),
    .mem_dcache_out(mem_dcache_out),
    .mem_dcache_byteen(mem_dcache_byteen),
    .mem_dcache_wren(mem_dcache_wren)
    );

  reg [32-1:0] dcpu_address_r;
  always@(posedge clk)
    dcpu_address_r<=dcpu_address;

  assign dcpu_readdata= (dcpu_address_r[31]) ? dbus_readdata : dcache_readdata;
  assign dcpu_readdata_line = dcache_readdata_line;

  //If this is an uncached access ([31]) wait on bus, otherwise wait on cache
  //multicyclestall staller(dcpu_en,
  //    dcache_wait|(dcpu_address_r[31]&dbus_wait),
  //    clk,resetn,dcpu_wait);

  // CPU should already stall 1st cycle, so register everything
  reg dcpu_en_r;
  always@(posedge clk)
    dcpu_en_r<=dcpu_en;

  assign dcache_wait= (~dcpu_address_r[31] && 
                             dcache_state==CACHE_IDLE && dcpu_en_r) ||
                   (dcache_state==CACHE_TAGLOOKUP && dcache_miss) ||
                   (dcache_state==CACHE_ISSUE && dcache_miss) ||
                   (dcache_state==CACHE_STOREMISS && dcache_miss) ||
                   (dcache_state==CACHE_LOADMISS && dcache_miss);

  always@(posedge clk)
    if (!resetn)
      dcache_state<=CACHE_IDLE;
    else
      case(dcache_state)
        CACHE_IDLE:
          dcache_state<= (dcpu_en && ~dcpu_address[31]) ? CACHE_TAGLOOKUP : 
                                                          CACHE_IDLE;
        CACHE_TAGLOOKUP:
          dcache_state<= (dcache_hit || ~dcpu_en) ? CACHE_IDLE : CACHE_ISSUE;
        CACHE_ISSUE:
          dcache_state<= (dbus_wait) ? CACHE_ISSUE :
            (dbus_wren) ?  CACHE_STOREMISS : CACHE_LOADMISS;
        CACHE_STOREMISS:
          dcache_state<= (dcache_hit || (|dcache_timeout)==0) ?  CACHE_IDLE : 
                        CACHE_STOREMISS;
        CACHE_LOADMISS:
          dcache_state<= (dcache_hit || (|dcache_timeout)==0) ?  CACHE_IDLE : 
                        CACHE_LOADMISS;
        default:
          dcache_state<=CACHE_IDLE;
      endcase

      // Timeout if dbus_wait has been low for 4 cycles
  always@(posedge clk)
    if (dcache_state==CACHE_ISSUE)
      dcache_timeout<={4{1'b1}};
    else
      dcache_timeout<=(dbus_wait<<3) | (dcache_timeout>>1);

      // Go to mem if miss happened or if non-mem region addr>0x80000000
      // For memory requests (0-0x7fffffff) send single pulse 
      // For peripherals (0xffffffff-0x80000000) hold dbus_en high
  assign dbus_en= (dcpu_address[31] && dcpu_en) ||
                  //Don't issue yet - wait one more cycle for vpu to rewind
                  (dcache_state==CACHE_ISSUE);

  //assign dbus_address=dcpu_address;  //not dcpu_address_r since bus must react to 0x80000000
  assign dbus_address=dcpu_address;
  assign dbus_writedata=dcpu_writedata;
  assign dbus_byteen=dcpu_byteen;
  assign dbus_wren=dcpu_wren;
  assign dbus_prefetch=dcpu_prefetch;


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


module vlane_saturatesum_16(
    in,
    op,
    out
    );

parameter WIDTH=16;

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
        
 

/******************************************************************************
  Vector Control Pipeline

          Stage 1       Stage 2         Stage 3
  ----|--------------|-----------------|-----------------|
      | Decode       | RF/EX           |WB & Send to cpu

 * Note, vs register file is written back to in Stage 3 also, but we check 
 * the vector lanes first to see if they're writing to the vs RF.  If so then
 * we stall.

******************************************************************************/

//`include "options.v"

//`include "vregfile_base.v"
//`include "vregfile_control.v"
//`include "vregfile_flag.v"
//`include "vregfile_inc.v"
//`include "vregfile_scalar.v"
//`include "vregfile_stride.v"
//`include "vregfile_vector.v"
//`include "vcomponents.v"
//`include "vlanes.v"

module vpu_7_7 (
    clk,
    resetn,

    // Instruction interface
    instr,
    instr_en,     // tells when instr is valid and available
    instr_wait,   // if high says vpu is not ready to receive
    has_memop,    // indicates vector pipeline has a memory operation
    mulshift_result,
    // For mtc2/ctc2 instructions
    scalar_in,    
    scalar_in_en,
    scalar_in_wait,

    // For cfc2 instructions
    scalar_out,
    scalar_out_en,
    scalar_out_wait,

    // Data memory interface
    dbus_address,
    dbus_en,
    dbus_we,
    dbus_byteen,
    dbus_writedata,
    dbus_readdata,
    dbus_cachematch,
    dbus_cachemiss,
    dbus_prefetch,
    dbus_wait

    );

parameter LOG2NUMLANES=`LOG2NUMLANES;
parameter LOG2MVL=`LOG2MVL;
parameter LOG2VPW=`LOG2VPW;
parameter LOG2LANEWIDTH=`LOG2LANEWIDTHBITS;
parameter LOG2NUMMEMLANES=`LOG2NUMMEMLANES;
parameter NUMMULLANES=2**`LOG2NUMMULLANES;
parameter LOG2NUMBANKS=`LOG2NUMBANKS;
parameter ALUPERBANK=`ALUPERBANK;

parameter NUMLANES=2**LOG2NUMLANES;
parameter MVL=2**LOG2MVL;
parameter VPW=2**LOG2VPW;
parameter LANEWIDTH=2**LOG2LANEWIDTH;
parameter DMEM_WRITEWIDTH=2**7; //Width of write bus to memory
parameter DMEM_READWIDTH=2**7; // Width of read bus from memory
parameter NUMMEMLANES=2**LOG2NUMMEMLANES;
parameter NUMBANKS=2**LOG2NUMBANKS;

parameter VCWIDTH=32;
parameter NUMVCREGS=64;
parameter LOG2NUMVCREGS=6;
parameter NUMNONMEMVCREGS=32;
parameter LOG2NUMNONMEMVCREGS=5;
parameter NUMVBASEREGS=16;
parameter LOG2NUMVBASEREGS=4;
parameter NUMVINCREGS=8;
parameter LOG2NUMVINCREGS=3;
parameter NUMVSTRIDEREGS=8;
parameter LOG2NUMVSTRIDEREGS=3;

parameter VSWIDTH=32;
parameter NUMVSREGS=32;
parameter LOG2NUMVSREGS=5;

parameter BIT_VSSRC2=6;
parameter BIT_VSSRC1=7;

// `include "visa.v"
parameter COP2_VADD           = 'b10z0000000;

parameter COP2_VADD_U         = 'b10z0000001;

parameter COP2_VSUB           = 'b10zz000010;

parameter COP2_VSUB_U         = 'b10zz000011;

parameter COP2_VMULHI         = 'b10z0000100;

parameter COP2_VMULHI_U       = 'b10z0000101;

parameter COP2_VDIV           = 'b10zz000110; //Using as matmul

parameter COP2_VBFADD         = 'b0100000001; //Using BF16 add

parameter COP2_VBFMULT        = 'b0100000010; //Using BF16 MULT

parameter COP2_VACT           = 'b0100000011; //Using ACT

parameter COP2_VTRP           = 'b0100000100; //Using ACT

parameter COP2_VDIV_U         = 'b10zz000111;

parameter COP2_VMOD           = 'b10zz001000;

parameter COP2_VMOD_U         = 'b10zz001001;

parameter COP2_VCMP_EQ        = 'b10zz001010;

parameter COP2_VCMP_NE        = 'b10zz001100;

parameter COP2_VCMP_LT        = 'b10zz001110;

parameter COP2_VCMP_U_LT      = 'b10zz001111;

parameter COP2_VCMP_LE        = 'b10zz010000;

parameter COP2_VCMP_U_LE      = 'b10zz010001;

parameter COP2_VMIN           = 'b10z0010010;

parameter COP2_VMIN_U         = 'b10z0010011;

parameter COP2_VMAX           = 'b10z0010100;

parameter COP2_VMAX_U         = 'b10z0010101;

parameter COP2_VMULLO         = 'b10z0010110;

parameter COP2_VABS           = 'b1000010111;

parameter COP2_VAND           = 'b10z0011000;

parameter COP2_VOR            = 'b10z0011001;

parameter COP2_VXOR           = 'b10z0011010;

parameter COP2_VNOR           = 'b10z0011011;

parameter COP2_VSLL           = 'b10zz011100;

parameter COP2_VSRL           = 'b10zz011101;

parameter COP2_VSRA           = 'b10zz011110;

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

parameter COP2_VSADD          = 'b10z0100010;

parameter COP2_VSADD_U        = 'b10z0100011;

parameter COP2_VSSUB          = 'b10zz100100;

parameter COP2_VSSUB_U        = 'b10zz100101;

parameter COP2_VSRR           = 'b1000100110;

parameter COP2_VSRR_U         = 'b1000100111;

parameter COP2_VSLS           = 'b1000101000;

parameter COP2_VSLS_U         = 'b1000101001;

parameter COP2_VXUMUL         = 'b10z0101010;

parameter COP2_VXUMUL_U       = 'b10z0101011;

parameter COP2_VXLMUL         = 'b10z0101100;

parameter COP2_VXLMUL_U       = 'b10z0101101;

parameter COP2_VXUMADD        = 'b10z0101110;

parameter COP2_VXUMADD_U      = 'b10z0101111;

parameter COP2_VXUMSUB        = 'b10z0110000;

parameter COP2_VXUMSUB_U      = 'b10z0110001;

parameter COP2_VXLMADD        = 'b10z0110010;

parameter COP2_VXLMADD_U      = 'b10z0110011;

parameter COP2_VXLMSUB        = 'b10z0110100;

parameter COP2_VXLMSUB_U      = 'b10z0110101;

parameter COP2_VINS_VV        = 'b1100000000;

parameter COP2_VINS_SV        = 'b1110000001;

parameter COP2_VEXT_VV        = 'b1100000010;

parameter COP2_VEXT_SV        = 'b1100000011;

parameter COP2_VEXT_U_SV      = 'b1100000100;

parameter COP2_VCOMPRESS      = 'b1100000101;

parameter COP2_VEXPAND        = 'b1100000110;

parameter COP2_VMERGE         = 'b11zz000111;

parameter COP2_VFINS          = 'b1110001000;

parameter COP2_VEXTHALF       = 'b1100001001;

parameter COP2_VHALF          = 'b1100001010;

parameter COP2_VHALFUP        = 'b1100001011;

parameter COP2_VHALFDN        = 'b1100001100;

parameter COP2_VSATVL         = 'b1100001101;

parameter COP2_VFAND          = 'b11z0001110;

parameter COP2_VFOR           = 'b11z0001111;

parameter COP2_VFXOR          = 'b11z0010000;

parameter COP2_VFNOR          = 'b11z0010001;

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

 
input clk;
input resetn;

input [31:0] instr;
input instr_en;     // tells when instr is valid and available
output instr_wait;   // if high says vpu is not ready to receive

output has_memop;

// For mtc2/ctc2 instructions
input [31:0] scalar_in;    
input scalar_in_en;
output scalar_in_wait;

// For cfc2 instructions
output [31:0] scalar_out;
output scalar_out_en;
input scalar_out_wait;

// Data memory interface
output  [ 31 : 0 ]  dbus_address;
output              dbus_en;
output              dbus_we;
output  [ (DMEM_WRITEWIDTH/8)-1 : 0 ]   dbus_byteen;
output  [ DMEM_WRITEWIDTH-1 : 0 ]       dbus_writedata;
input   [ DMEM_READWIDTH-1 : 0 ]        dbus_readdata;
input         dbus_cachematch;
input         dbus_cachemiss;
output  [ 31 : 0 ]  dbus_prefetch;
input               dbus_wait;
output [16*8-1:0] mulshift_result;

reg                        [ 31 : 0 ]   ir;
wire                       [ 31 : 0 ]   ir2;
wire                       [ 31 : 0 ]   ir3;

wire                                    stall1;
wire                                    stall2;
wire                                    stall3;
wire                        [ 3 : 1 ]   internal_stall;
wire                                    squash1;
wire                                    squash2;
wire        [`MAX_PIPE_STAGES-1 : 0 ]   vlanes_stalled;

wire                        [ 5 : 0 ]   ir_cop2;
wire                        [ 9 : 0 ]   ir_op;
wire                        [ 5 : 0 ]   ir_vcr;
wire                        [ 4 : 0 ]   ir_vsr;
wire                        [ 4 : 0 ]   ir_src2;
wire                        [ 4 : 0 ]   ir_src1;
wire                        [ 5 : 0 ]   ir_vcr_r;

wire    [ LOG2NUMNONMEMVCREGS-1 : 0 ]   vc_a_reg;
wire                [ VCWIDTH-1 : 0 ]   vc_a_readdataout;
wire                                    vc_a_en;
wire    [ LOG2NUMNONMEMVCREGS-1 : 0 ]   vc_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vc_c_writedatain;
wire                                    vc_c_we;
wire                [ VCWIDTH-1 : 0 ]   vl;
//Masks for the matmul (3 masks. each is 8-bit.)
//1-bit for each row/column element in the matmul. we have an 8x8 matmul)
wire         [3*`MAT_MUL_SIZE-1 : 0 ]   matmul_masks;

wire       [ LOG2NUMVBASEREGS-1 : 0 ]   vbase_a_reg;
wire                [ VCWIDTH-1 : 0 ]   vbase_a_readdataout;
wire                                    vbase_a_en;
wire       [ LOG2NUMVBASEREGS-1 : 0 ]   vbase_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vbase_c_writedatain;
wire                                    vbase_c_we;

wire        [ LOG2NUMVINCREGS-1 : 0 ]   vinc_a_reg;
wire                                    vinc_a_en;
wire                [ VCWIDTH-1 : 0 ]   vinc_a_readdataout;
wire        [ LOG2NUMVINCREGS-1 : 0 ]   vinc_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vinc_c_writedatain;
wire                                    vinc_c_we;

wire     [ LOG2NUMVSTRIDEREGS-1 : 0 ]   vstride_a_reg;
wire                                    vstride_a_en;
wire                [ VCWIDTH-1 : 0 ]   vstride_a_readdataout;
wire     [ LOG2NUMVSTRIDEREGS-1 : 0 ]   vstride_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vstride_c_writedatain;
wire                                    vstride_c_we;

wire          [ LOG2NUMVSREGS-1 : 0 ]   vs_a_reg;
wire                [ VCWIDTH-1 : 0 ]   vs_a_readdataout;
wire                                    vs_a_en;
wire          [ LOG2NUMVSREGS-1 : 0 ]   vs_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vs_c_writedatain;
wire                                    vs_c_we;

wire                [ VCWIDTH-1 : 0 ]   vc_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vl_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vbase_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vinc_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vstride_readdataout;
wire                [ VSWIDTH-1 : 0 ]   vs_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vc_combined_out;
wire                [ VCWIDTH-1 : 0 ]   vc_combined_out_s3;
wire                [ VCWIDTH-1 : 0 ]   _vc_c_writedatain;
wire                [ VCWIDTH-1 : 0 ]   fwddata;
wire                                    fwd_vc;
wire                                    fwd_vl;
wire                                    fwd_vbase;
wire                                    fwd_vinc;
wire                                    fwd_vstride;

reg                                     ctrl_vc_a_en;
reg                                     ctrl_vl_a_en;
reg                                     ctrl_vbase_a_en;
reg                                     ctrl_vinc_a_en;
reg                                     ctrl_vstride_a_en;
reg                                     ctrl_vs_a_en;

wire          [ LOG2NUMVCREGS-1 : 0 ]   vc_rd_reg;
wire          [ LOG2NUMVCREGS-1 : 0 ]   vbase_rd_reg;
wire          [ LOG2NUMVCREGS-1 : 0 ]   vinc_rd_reg;
wire          [ LOG2NUMVCREGS-1 : 0 ]   vstride_rd_reg;
wire          [ LOG2NUMVSREGS-1 : 0 ]   vs_rd_reg;

wire          [ LOG2NUMVSREGS-1 : 0 ]   vlanes_vs_dst;
wire                                    vlanes_vs_we;
wire                        [ 5 : 2 ]   vlanes_vs_wetrack;
wire                [ VSWIDTH-1 : 0 ]   vlanes_vs_writedata;

wire                [ VCWIDTH-1 : 0 ]   vbase_plus_vinc;

wire [LOG2NUMVCREGS-1:0] ir_base;
wire [LOG2NUMVCREGS-1:0] ir_inc;
wire [LOG2NUMVCREGS-1:0] ir_stride;
wire [LOG2NUMVCREGS-1:0] vcdest_s1;
wire [LOG2NUMVCREGS-1:0] vcdest_s2;
wire [LOG2NUMVCREGS-1:0] vsdest_s1;
wire [LOG2NUMVCREGS-1:0] vsdest_s2;
wire       wevalid;
wire       wevalid_s2;
wire       haz_vc;
wire       haz_vl;
wire       haz_vbase;
wire       haz_vinc;
wire       haz_vstride;
wire       haz_vs_RAW;
wire       haz_vs_WAW;

reg cfc_satisfied;

reg ctrl__vc_writedatain_sel;          //1 - vmstc, 0 - mtc;
reg [1:0] ctrl_vc_writedatain_sel;  //0 - vmstc/mtc, 1-vhalf, 2/3-vsatvl
reg ctrl_vc_we;
reg ctrl_vbase_writedatain_sel;    //1 - ctc/mstc, 0 - vld/vst
reg ctrl_scalarin_en;   //1 when ctc2/mtc2
reg ctrl_scalar_out_en;  //1 when cfc2
reg [1:0] ctrl_vcdest_sel;  //0 - ctc/mstc, 1- ld/st, 2 - satvl/half
reg ctrl_vsdest_sel;  //1 - mtc2, 0 - vmcts
reg ctrl_vs_we;
reg ctrl_rdctl_sel;        // 1 - cfc2/vmcts  0 - ld/st,
reg [1:0] ctrl_rdvc_sel;   // 2-vshamt, 3-vindex, other-ir_vcr


wire ctrl_scalarin_en_s2;
wire ctrl_scalar_out_en_s2;
wire      scalar_out_en_s3;
wire ctrl_vc_we_s2;
wire ctrl_vbase_writedatain_sel_s2;
wire ctrl__vc_writedatain_sel_s2;
wire [1:0] ctrl_vc_writedatain_sel_s2;
wire ctrl_vsdest_sel_s2;
wire ctrl_vs_we_s2;

/************************ Instruction Register ******************************/
wire is_cop2;
reg is_cop2_s1;

  assign is_cop2=instr[31:26]==6'b010010;

  always @(posedge clk)
    if (!resetn || (~is_cop2&~stall1) || (~instr_en&~stall1))
      ir<=32'h0;    // NOP  (Used to use VMSTC $vc48,$vs0)
    else if (instr_en&~stall1)
      ir<=instr;

  always @(posedge clk)
    if (!resetn)
      is_cop2_s1<=1'b0;
    else if (instr_en)
      is_cop2_s1<=is_cop2;

  assign instr_wait = stall1 & is_cop2;

/******************************************************************************/
/************************** 1st Pipeline Stage ********************************/
/******************************************************************************/

//Flag instructions which don't use lanes so they don't stall when lanes stalled
  reg ctrl_doesnt_use_lanes;
  always@*
  begin
    ctrl_doesnt_use_lanes=0;
    case(ir_op)
      0:
        ctrl_doesnt_use_lanes=1;
      COP2_VSATVL:
        ctrl_doesnt_use_lanes=1;
      //COP2_VMCTS:  //Omit since vlanes modifies scalar
      COP2_VMSTC:
        ctrl_doesnt_use_lanes=1;
      COP2_CFC2:
        ctrl_doesnt_use_lanes=1;
      COP2_CTC2:
        ctrl_doesnt_use_lanes=1;
    endcase
  end

  assign internal_stall[1]=internal_stall[2] | haz_vs_RAW;
  assign stall1=internal_stall[1] | (vlanes_stalled[1]&&~ctrl_doesnt_use_lanes);
  assign squash1 = (stall1&~stall2)|~resetn;

  pipereg_32 IR_reg2(ir,clk,resetn,~stall1,~squash1,ir2);

  assign ir_cop2=ir[31:26];
  assign ir_op={ir[25:22],ir[5:0]}; //10 bits
  assign ir_vcr=ir[15:10];
  assign ir_vsr=ir[15:11];
  assign ir_src1=ir[15:11];
  assign ir_src2=ir[20:16];
  assign ir_base={2'b10,ir[15:12]};
  assign ir_inc={3'b110,ir[11:9]};
  assign ir_stride={3'b111,ir[8:6]};

  assign wevalid = (ir_cop2==6'b010010) & ctrl_vc_we & (vcdest_s1!=48);

  pipereg_1 wevalid_reg1(wevalid,clk,~squash1, ~stall1,1'b1, wevalid_s2);

  assign vcdest_s1 = (ctrl_vcdest_sel==0) ? ir_vcr :
                    (ctrl_vcdest_sel==1) ? ir_base : 0;

  pipereg_6 vcdest_reg1(vcdest_s1,clk,~squash1, ~stall1,1'b1,
                      vcdest_s2);

  assign vsdest_s1 = (ctrl_vsdest_sel) ? ir_vsr : ir_src2;

  pipereg_5 vsdest_reg1(vsdest_s1,clk,~squash1, ~stall1,1'b1,
                      vsdest_s2);

  pipereg_5 vsdestsel_reg1
        (ctrl_vsdest_sel,clk,~squash1, ~stall1,1'b1, ctrl_vsdest_sel_s2);

  // Before reading all source operands, we need to:
  //  1. Determine what all the sources are
  always@*
  begin
  case(ir_op)
      COP2_VADD:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VADD_U:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VSUB:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSUB_U:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VMULHI:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VMULHI_U:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VDIV:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VDIV_U:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VMOD:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VMOD_U:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_EQ:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_NE:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_LT:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_U_LT:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_LE:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_U_LE:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VMIN:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VMIN_U:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VMAX:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VMAX_U:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VMULLO:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VABS:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VAND:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VOR:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VXOR:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VNOR:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VSLL:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSRL:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSRA:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSAT_B:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSAT_H:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSAT_W:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSAT_SU_B:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSAT_SU_H:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSAT_SU_W:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSAT_SU_L:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSAT_U_B:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSAT_U_H:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSAT_U_W:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSADD:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VSADD_U:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VSSUB:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSSUB_U:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSRR:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VSRR_U:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VSLS:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VSLS_U:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VXUMUL:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VXUMUL_U:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VXLMUL:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VXLMUL_U:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VXUMADD:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VXUMADD_U:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VXUMSUB:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VXUMSUB_U:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VXLMADD:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VXLMADD_U:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VXLMSUB:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VXLMSUB_U:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VINS_VV:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VINS_SV:
      begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VEXT_VV:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VEXT_SV:
begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VEXT_U_SV:
begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VCOMPRESS:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VEXPAND:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VMERGE:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VFINS:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 1;
        end
      COP2_VEXTHALF:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VHALF:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VHALFUP:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VHALFDN:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VSATVL:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VFAND:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VFOR:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VFXOR:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VFNOR:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = ir_op[BIT_VSSRC1];
        end
      COP2_VFCLR:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VFSET:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VIOTA:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VCIOTA:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VFPOP:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VFFF1:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VFFL1:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VFSETBF:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VFSETIF:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VFSETOF:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VFMT8:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VFMF8:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VFCLR8:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VFOR8:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 1;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VFLD:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VLD_B:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VLD_H:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VLD_W:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VLD_L:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VLD_U_B:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VLD_U_H:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VLD_U_W:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VLDS_B:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 1;
ctrl_vs_a_en = 0;
        end
      COP2_VLDS_H:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 1;
ctrl_vs_a_en = 0;
        end
      COP2_VLDS_W:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 1;
ctrl_vs_a_en = 0;
        end
      COP2_VLDS_L:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 1;
ctrl_vs_a_en = 0;
        end
      COP2_VLDS_U_B:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 1;
ctrl_vs_a_en = 0;
        end
      COP2_VLDS_U_H:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 1;
ctrl_vs_a_en = 0;
        end
      COP2_VLDS_U_W:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 1;
ctrl_vs_a_en = 0;
        end
      COP2_VLDX_B:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VLDX_H:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VLDX_W:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VLDX_L:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VLDX_U_B:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VLDX_U_H:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VLDX_U_W:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VFST:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VST_B:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VST_H:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VST_W:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VST_L:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
        end
      COP2_VSTS_B:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 1;
ctrl_vs_a_en = 0;
        end
      COP2_VSTS_H:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 1;
ctrl_vs_a_en = 0;
        end
      COP2_VSTS_W:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 1;
ctrl_vs_a_en = 0;
        end
      COP2_VSTS_L:
        begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 1;
ctrl_vs_a_en = 0;
        end
      COP2_VSTX_B:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSTX_H:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSTX_W:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSTX_L:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSTXO_B:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSTXO_H:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSTXO_W:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VSTXO_L:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
      COP2_VMCTS:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 1;
ctrl_vs_a_en = 0;
        end
      COP2_VMSTC:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 1;
end
      COP2_CFC2:
        begin
ctrl_vc_a_en = 1;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 1;
ctrl_vinc_a_en = 1;
ctrl_vstride_a_en = 1;
ctrl_vs_a_en = 0;
        end
default:
begin
ctrl_vc_a_en = 0;
ctrl_vl_a_en = 0;
ctrl_vbase_a_en = 0;
ctrl_vinc_a_en = 0;
ctrl_vstride_a_en = 0;
ctrl_vs_a_en = 0;
end
endcase

  end

  //  2. Determine if any of the sources have a RAW hazard
  assign haz_vc=(ctrl_vc_a_en&wevalid_s2)&&(vc_rd_reg==vcdest_s2);
  assign haz_vl=(ctrl_vl_a_en&wevalid_s2)&&(vcdest_s2==0);
  assign haz_vbase=(ctrl_vbase_a_en&wevalid_s2)&&(vbase_rd_reg==vcdest_s2);
  assign haz_vinc=(ctrl_vinc_a_en&wevalid_s2)&&(vinc_rd_reg==vcdest_s2)&&(vcdest_s2!=48);
  assign haz_vstride=(ctrl_vstride_a_en&wevalid_s2)&(vstride_rd_reg==vcdest_s2);

  // Coarse-grained checks for VS - don't check registers
  assign haz_vs_RAW=ctrl_vs_a_en && (ctrl_vs_we_s2 || (|vlanes_vs_wetrack));
  assign haz_vs_WAW=ctrl_vs_we_s2 && (|vlanes_vs_wetrack[5:3]);

  pipereg_32 fwddata_reg (
       (ctrl_vbase_writedatain_sel_s2) ?  vbase_plus_vinc : vc_c_writedatain,
       clk,~squash1, ~stall1,1'b1,fwddata);

  pipereg_1  fwdvc_reg ( haz_vc,clk,~squash1, ~stall1,1'b1,fwd_vc);
  pipereg_1  fwdvl_reg ( haz_vl,clk,~squash1, ~stall1,1'b1,fwd_vl);
  pipereg_1  fwdvbase_reg ( haz_vbase,clk,~squash1, ~stall1,1'b1,fwd_vbase);
  pipereg_1  fwdvinc_reg ( haz_vinc,clk,~squash1, ~stall1,1'b1,fwd_vinc);
  pipereg_1  fwdvstride_reg(haz_vstride,clk,~squash1,~stall1,1,fwd_vstride);

  /************************ REGISTER FILES ******************************/

  assign vc_rd_reg= (ctrl_rdvc_sel==2) ? 2 :  //vshamt
                    (ctrl_rdvc_sel==3) ? 3 :  //vindex
                    ir_vcr;
  assign vbase_rd_reg= (ctrl_rdctl_sel) ?  ir_vcr : ir_base;
  assign vinc_rd_reg= (ctrl_rdctl_sel) ?  ir_vcr : ir_inc;
  assign vstride_rd_reg= (ctrl_rdctl_sel) ?  ir_vcr : ir_stride;
  assign vs_rd_reg= (ctrl_vs_a_en & ir_op[BIT_VSSRC1]) ? 
                        ir_src1[LOG2NUMVSREGS-1:0] : 
                        ir_src2[LOG2NUMVSREGS-1:0];

  assign vc_a_reg= vc_rd_reg[LOG2NUMNONMEMVCREGS-1:0];
  assign vbase_a_reg= vbase_rd_reg[LOG2NUMVBASEREGS-1:0];
  assign vinc_a_reg=vinc_rd_reg[LOG2NUMVINCREGS-1:0];
  assign vstride_a_reg=vstride_rd_reg[LOG2NUMVSTRIDEREGS-1:0];

  assign vs_a_reg=vs_rd_reg[LOG2NUMVSREGS-1:0];

  assign vc_a_en=ctrl_vc_a_en&~stall1;
  assign vbase_a_en=ctrl_vbase_a_en&~stall1;
  assign vinc_a_en=ctrl_vinc_a_en&~stall1;
  assign vstride_a_en=ctrl_vstride_a_en&~stall1;
  assign vs_a_en=ctrl_vs_a_en&~stall1;

  vregfile_control_32_32_5 vregfile_control (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vc_a_reg), 
      .a_en(vc_a_en),
      .a_readdataout(vc_a_readdataout),
      .c_reg(vc_c_reg), 
      .c_writedatain(vc_c_writedatain), 
      .c_we(vc_c_we),
      .vl(vl),
      //The reserved registers vc31, vc30, vc29 are used
      //for the matmul's masks.
      .matmul_masks(matmul_masks)
      );

  vregfile_base_32_16_4 vregfile_base (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vbase_a_reg), 
      .a_en(vbase_a_en), 
      .a_readdataout(vbase_a_readdataout),
      .c_reg(vbase_c_reg), 
      .c_writedatain(vbase_c_writedatain), 
      .c_we(vbase_c_we));

  vregfile_inc_32_8_3 vregfile_inc (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vinc_a_reg), 
      .a_en(vinc_a_en), 
      .a_readdataout(vinc_a_readdataout),
      .c_reg(vinc_c_reg), 
      .c_writedatain(vinc_c_writedatain), 
      .c_we(vinc_c_we));

  vregfile_stride_32_8_3 vregfile_stride (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vstride_a_reg), 
      .a_en(vstride_a_en), 
      .a_readdataout(vstride_a_readdataout),
      .c_reg(vstride_c_reg), 
      .c_writedatain(vstride_c_writedatain), 
      .c_we(vstride_c_we));

  vregfile_scalar_32_32_5 vregfile_scalar (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vs_a_reg), 
      .a_en(vs_a_en), 
      .a_readdataout(vs_a_readdataout),
      .c_reg(vs_c_reg), 
      .c_writedatain(vs_c_writedatain), 
      .c_we(vs_c_we));

////////////////////////////////////////////////////////////      
//    parameters sequence for regfiles
//
//    defparam WIDTH=VSWIDTH;
//    defparam NUMREGS=NUMVSREGS;
//    defparam LOG2NUMREGS=LOG2NUMVSREGS;
////////////////////////////////////////////////////////////
  pipereg_6 vc_reg(ir_vcr,clk,resetn,vc_a_en,1'b1,ir_vcr_r);

  pipereg_1 r1(ctrl_scalarin_en,clk,~squash1,~stall1,1'b1,ctrl_scalarin_en_s2);
  pipereg_1 r2(ctrl_scalar_out_en,clk,~squash1,~stall1,1'b1,ctrl_scalar_out_en_s2);
  pipereg_1 r4(ctrl_vc_we,clk,~squash1,~stall1,1'b1,ctrl_vc_we_s2);
  pipereg_1 r5(ctrl_vbase_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl_vbase_writedatain_sel_s2);
  pipereg_1 r6(ctrl__vc_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl__vc_writedatain_sel_s2);
  pipereg_2 r7(ctrl_vc_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl_vc_writedatain_sel_s2);
  pipereg_1 r8(ctrl_vs_we,clk,~squash1,~stall1,1'b1,ctrl_vs_we_s2);


  /*********************** Control control signals ****************************/
  always@*
  begin
  case(ir_op)
      COP2_VSRR:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VSRR_U:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VSLS:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VSLS_U:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VXUMUL:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VXUMUL_U:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VXLMUL:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VXLMUL_U:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VXUMADD:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VXUMADD_U:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VXUMSUB:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VXUMSUB_U:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VXLMADD:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VXLMADD_U:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VXLMSUB:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VXLMSUB_U:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VINS_VV:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 2;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VINS_VV:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 3;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VINS_SV:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 3;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VEXT_VV:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 3;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VEXT_SV:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 3;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VEXT_U_SV:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 3;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VFINS:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 3;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VHALFUP:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 3;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VHALFDN:  
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 3;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VHALF:
      begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 1;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 2;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
      end
      COP2_VSATVL:
      begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 2;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 2;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
      end
      COP2_VSTS_L:
      begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
      end
COP2_VFLD:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VLD_B:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VLD_H:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VLD_W:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VLD_L:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VLD_U_B:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VLD_U_H:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VLD_U_W:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VLDS_B:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VLDS_H:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VLDS_W:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VLDS_L:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VLDS_U_B:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VLDS_U_H:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VLDS_U_W:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VFST:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VST_B:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VST_H:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VST_W:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VST_L:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VSTS_B:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VSTS_H:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
COP2_VSTS_W:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 1;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 1;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
      COP2_VMCTS:
      begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 1;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 1;
      end
      COP2_VMSTC:
      begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 1;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
      end
      COP2_CFC2:
      begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 1;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 1;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
      end
      COP2_CTC2:
      begin
ctrl_scalarin_en = 1;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 1;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
      end
      COP2_MTC2:
      begin
ctrl_scalarin_en = 1;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 1;
ctrl_vs_we = 1;
      end
default:
begin
ctrl_scalarin_en = 0;
ctrl_scalar_out_en = 0;
ctrl__vc_writedatain_sel = 0;
ctrl_vc_writedatain_sel = 0;
ctrl_vbase_writedatain_sel = 0;
ctrl_vc_we = 0;
ctrl_rdctl_sel = 0;
ctrl_rdvc_sel = 0;
ctrl_vcdest_sel = 0;
ctrl_vsdest_sel = 0;
ctrl_vs_we = 0;
end
endcase
    
  end

/******************************************************************************/
/************************** 2nd Pipeline Stage ********************************/
/******************************************************************************/

  assign internal_stall[2]=internal_stall[3] |
                              (ctrl_scalarin_en_s2&~scalar_in_en) | 
                              haz_vs_WAW;
  assign stall2=internal_stall[2]; // | vlanes_stalled[2];
  assign squash2 = ((stall2&~stall3))|~resetn;

  // Stall scalar if a) we are stalled, are expecting a scalar, and the it is
  // available OR b) if we haven't gotten there yet.
  assign scalar_in_wait=(scalar_in_en&~ctrl_scalarin_en_s2) || 
                        (ctrl_scalarin_en_s2&scalar_in_en&stall2);

  assign vc_readdataout= (fwd_vc) ? fwddata : vc_a_readdataout;
  assign vl_readdataout= (fwd_vl) ? fwddata : vl;
  assign vbase_readdataout= (fwd_vbase) ? fwddata : vbase_a_readdataout;
  assign vinc_readdataout= (fwd_vinc) ? fwddata : vinc_a_readdataout;
  assign vstride_readdataout= (fwd_vstride) ? fwddata : vstride_a_readdataout;
  assign vs_readdataout= vs_a_readdataout;

  assign vc_combined_out=(!ir_vcr_r[5]) ? vc_readdataout :
                         (!ir_vcr_r[4]) ? vbase_readdataout :
                         (!ir_vcr_r[3]) ? vinc_readdataout : 
                         vstride_readdataout;

  pipereg_32 scalar_out_reg(
      vc_combined_out,clk,resetn,ctrl_scalar_out_en_s2,1'b1,vc_combined_out_s3);

  pipereg_1 scalar_out_en_reg(
      ctrl_scalar_out_en_s2,clk,~squash2,~stall2,1'b1,scalar_out_en_s3);


  /************** Datapath - Control Regs *******************/

  assign vc_c_we = ctrl_vc_we_s2&~vcdest_s2[5] & ~stall2;
  assign vc_c_reg = vcdest_s2[LOG2NUMNONMEMVCREGS-1:0]; 
  //temporary to feed to base,inc, and stride regfiles (without vl logic)
  assign _vc_c_writedatain= (ctrl__vc_writedatain_sel_s2) ? vs_a_readdataout :
                                                        scalar_in;
  assign vc_c_writedatain= (ctrl_vc_writedatain_sel_s2==0) ? _vc_c_writedatain :
                            //vhalf instruction
                            (ctrl_vc_writedatain_sel_s2==1) ? vl_readdataout>>1:
                            //vsatvl instruction
                            //(vl_readdataout>vc_readdataout) ? vc_readdataout :
                            (vl_readdataout>MVL) ? MVL : 
                            vl_readdataout;

  assign vbase_c_we = ctrl_vc_we_s2 & (vcdest_s2[5:4]==2) & ~stall2;
  assign vbase_c_reg = vcdest_s2[LOG2NUMVBASEREGS-1:0];
  assign vbase_c_writedatain = (ctrl_vbase_writedatain_sel_s2) ?
                                vbase_plus_vinc :
                                _vc_c_writedatain;

  assign vbase_plus_vinc=vbase_readdataout+vinc_readdataout;

  assign vinc_c_we = ctrl_vc_we_s2 & (vcdest_s2[5:3]==6) & ~stall2;
  assign vinc_c_reg = vcdest_s2[LOG2NUMVINCREGS-1:0];
  assign vinc_c_writedatain = _vc_c_writedatain;

  assign vstride_c_we = ctrl_vc_we_s2 & (vcdest_s2[5:3]==7) & ~stall2;
  assign vstride_c_reg = vcdest_s2[LOG2NUMVSTRIDEREGS-1:0];
  assign vstride_c_writedatain = _vc_c_writedatain;

  //FIXME - OR vector writes in, but check for RAW & WAW hazards
  assign vs_c_we = vlanes_vs_we || (ctrl_vs_we_s2 & ~stall2);
  assign vs_c_reg = (vlanes_vs_we) ? vlanes_vs_dst : vsdest_s2;
  assign vs_c_writedatain = (vlanes_vs_we) ? vlanes_vs_writedata : 
                    (ctrl_vsdest_sel_s2) ? scalar_in : vc_combined_out;


/******************************************************************************/
/************************** 3rd Pipeline Stage ********************************/
/******************************************************************************/

  assign internal_stall[3]=scalar_out_en_s3&scalar_out_wait;
  assign stall3=internal_stall[3]; // | vlanes_stalled[3];

  assign scalar_out=vc_combined_out_s3;
  assign scalar_out_en=scalar_out_en_s3&~cfc_satisfied;

  always@(posedge clk)
    cfc_satisfied<=scalar_out_en_s3 & ~scalar_out_wait;

/******************************************************************************/
/************************ Instantiate Vector Lanes ****************************/
/******************************************************************************/

  vlanes_8_3_8_3_8_64_6_2_1_16_4_2_1_0_128_7_128_7_32_32_32_5 vlanes(
    .clk(clk),
    .resetn(resetn),

    // Instruction interface
    .instr(ir),
    .instr_en(is_cop2_s1),    // tells when instr is valid and available
    .instr_wait(),            // if high says vpu is not ready to receive

    .stall_in({internal_stall,1'b0}),
    .is_stalled(vlanes_stalled),
    .has_memop(has_memop),

    // Control register values - 2nd stage
    .vc_in(vc_readdataout),
    .vl_in(vl_readdataout),
    .vbase_in(vbase_readdataout),
    .vinc_in(vinc_readdataout),
    .vstride_in(vstride_readdataout),
    .vs_in(vs_readdataout),
    .matmul_masks_in(matmul_masks),
    .mulshift_result_s5(mulshift_result),
    // vs Writeback
    .vs_dst(vlanes_vs_dst),
    .vs_wetrack(vlanes_vs_wetrack),  //1-bit for each pipe-stage
    .vs_we(vlanes_vs_we),
    .vs_writedata(vlanes_vs_writedata),

    // Data memory interface
    .dbus_address(dbus_address),
    .dbus_en(dbus_en),
    .dbus_we(dbus_we),
    .dbus_byteen(dbus_byteen),
    .dbus_writedata(dbus_writedata),
    .dbus_readdata(dbus_readdata),
    .dbus_cachematch(dbus_cachematch),
    .dbus_cachemiss(dbus_cachemiss),
    .dbus_prefetch(dbus_prefetch),
    .dbus_wait(dbus_wait)
    );
  
///////////////////////////////////////////////////
//    vlanes parameter sequence for reference
//
//    vlanes.NUMLANES=NUMLANES,
//    vlanes.LOG2NUMLANES=LOG2NUMLANES,
//    vlanes.NUMMEMPARALLELLANES=NUMMEMLANES,
//    vlanes.LOG2NUMMEMPARALLELLANES=LOG2NUMMEMLANES,
//    vlanes.NUMMULLANES=NUMMULLANES,
//    vlanes.MVL=MVL,
//    vlanes.LOG2MVL=LOG2MVL,
//    vlanes.VPW=VPW,
//    vlanes.LOG2VPW=LOG2VPW,
//    vlanes.LANEWIDTH=LANEWIDTH,
//    vlanes.LOG2LANEWIDTH=LOG2LANEWIDTH,
//    vlanes.NUMBANKS=NUMBANKS,
//    vlanes.LOG2NUMBANKS=LOG2NUMBANKS,
//    vlanes.ALUPERBANK=ALUPERBANK,
//    vlanes.DMEM_WRITEWIDTH=DMEM_WRITEWIDTH, 
//    vlanes.LOG2DMEM_WRITEWIDTH=7,
//    vlanes.DMEM_READWIDTH=DMEM_READWIDTH,
//    vlanes.LOG2DMEM_READWIDTH=7,
//    vlanes.VCWIDTH=VCWIDTH,
//    vlanes.VSWIDTH=VSWIDTH,
//    vlanes.NUMVSREGS=NUMVSREGS,
//    vlanes.LOG2NUMVSREGS=LOG2NUMVSREGS;
//////////////////////////////////////////////////////////
endmodule

                 
module dpram_5_32_32 (
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
input [5-1:0] address_a;
input [5-1:0] address_b;
input  wren_a;
input  wren_b;
input [32-1:0] data_a;
input [32-1:0] data_b;
output [32-1:0] out_a;
output[32-1:0] out_b;

dual_port_ram u_dual_port_ram(
.addr1(address_a),
.we1(wren_a),
.data1(data_a),
.out1(out_a),
.addr2(address_b),
.we2(wren_b),
.data2(data_b),
.out2(out_b),
.clk(clk)
);

endmodule

module local_shifter_17_2_ARITHMATIC(
  data,
  distance,
  direction,
  result
);

parameter LPM_SHIFTTYPE = "ARITHMATIC";

input [17-1:0] data;
input [2-1:0] distance;
input direction;

output reg [17-1:0] result;
reg [17-1:0] arith_reg; 
always @* begin
  arith_reg = {17{1'b1}};
    if (LPM_SHIFTTYPE == "ARITHMETIC") begin
    if(direction)begin
      result = data << distance;
    end
    else begin
      if(data[17-1] == 1'b1)
          result =  ((arith_reg <<(17 - distance))|| (data >> distance));
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
module velmshifter_laneunit_8 (
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

input [ 8-1:0 ]  inpipe;
input [ 8-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 8-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 8-1:0 ] outpipe;

reg [ 8-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

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
module vlane_mulshift_16_4(clk, resetn,
            opA, opB, sa,
            op,
            en,
            result
            );
parameter WIDTH=16;
parameter LOG2WIDTH=4;

input clk;
input resetn;

input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
input [LOG2WIDTH-1:0] sa;
input [4:0] op;
input [3:1] en;  //Enable for each pipestage

output [WIDTH-1:0] result;

wire [3:0] temp;
/********* Control Signals *********/
wire is_signed, dir, is_mul, saturate, half;
assign is_mul=op[2];      // selects between opB and the computed shift amount
assign is_signed=~op[1];
assign dir=op[0];         // selects between 2^sa and 2^(32-sa) for right shift
assign saturate=op[3];
assign half=op[4];
assign temp = en;

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
wire [34-1:0] local_mult_component_result;

assign dataa = {is_signed&opA_mux_out[WIDTH-1],opA_mux_out};
assign aclr = ~resetn;
assign {dum2,dum,hi,lo} = local_mult_component_result;

local_mult_17_17_34 local_mult_component (
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
/****************************************************************************
          Scalar Register File

   - Has one read port (a) and one write port (c)
   - vs0 fixed to 0 and unwriteable
****************************************************************************/
module vregfile_scalar_32_32_5 (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

input clk;
input resetn;

input a_en;
input [5-1:0] a_reg,c_reg;
output [32-1:0] a_readdataout;
input [32-1:0] c_writedatain;
input c_we;

    ram_wrapper_5_32_32 reg_file1(
        .clk(clk),
        .resetn(resetn),
        .rden_a(1'b0),
        .rden_b(a_en),
        .address_a(c_reg[5-1:0]),
        .address_b(a_reg[5-1:0]),
        .wren_a(c_we & (|c_reg)),
        .wren_b(1'b0),
        .data_a(c_writedatain),
        .data_b(0),
        .out_a(),
        .out_b(a_readdataout)
    );

endmodule
        
module dpram_3_8_32 (
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
input [3-1:0] address_a;
input [3-1:0] address_b;
input  wren_a;
input  wren_b;
input [32-1:0] data_a;
input [32-1:0] data_b;
output [32-1:0] out_a;
output[32-1:0] out_b;

dual_port_ram u_dual_port_ram(
.addr1(address_a),
.we1(wren_a),
.data1(data_a),
.out1(out_a),
.addr2(address_b),
.we2(wren_b),
.data2(data_b),
.out2(out_b),
.clk(clk)
);

endmodule

/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_flag_2_1_8_256_8
  (clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_we);

input clk;
input resetn;

input [2-1:0] a_en;
input [2-1:0] b_en;
input [2-1:0] c_we;

input [2*7-1:0] a_reg,b_reg,c_reg;
output [2*8-1:0] a_readdataout, b_readdataout;
wire [2*8-1:0] a_temp, b_temp;
input [2*8-1:0] c_writedatain;


      ram_wrapper_7_128_8 reg_file1_0(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(a_en[0]),
          .address_a(c_reg[0* 7 +: 7]),
          .address_b(a_reg[0* 7 +: 7]),
          .wren_a(c_we[0]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ 0*8 +: 8]),
          .data_b(0),
          .out_a(a_temp[ 0*8 +: 8]),
          .out_b(a_readdataout[0*8 +: 8])
      );

      ram_wrapper_7_128_8 reg_file2_0(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(b_en[0]),
          .address_a(c_reg[0* 7 +: 7]),
          .address_b(b_reg[0* 7 +: 7]),
          .wren_a(c_we[0]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ 0*8 +: 8]),
          .data_b(0),
          .out_a(b_temp[ 0*8 +: 8]),
          .out_b(b_readdataout[ 0* 8 +: 8])
      );
      
      ram_wrapper_7_128_8 reg_file1_1(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(a_en[1]),
          .address_a(c_reg[1* 7 +: 7]),
          .address_b(a_reg[1* 7 +: 7]),
          .wren_a(c_we[1]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ 1*8 +: 8]),
          .data_b(0),
          .out_a(a_temp[ 1*8 +: 8]),
          .out_b(a_readdataout[1*8 +: 8])
      );

      ram_wrapper_7_128_8 reg_file2_1(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(b_en[1]),
          .address_a(c_reg[1* 7 +: 7]),
          .address_b(b_reg[1* 7 +: 7]),
          .wren_a(c_we[1]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ 1*8 +: 8]),
          .data_b(0),
          .out_a(b_temp[ 1*8 +: 8]),
          .out_b(b_readdataout[ 1* 8 +: 8])
      );
            
endmodule

module local_add_sub_18_0_SIGNED(
dataa,
datab,
cin,
add_sub,
result
);

input[18-1:0] dataa;
input[18-1:0] datab;
input cin;
input add_sub;
output reg [18-1:0] result;

always @(*)begin
    if(add_sub == 1'b1)
         result = dataa+datab+cin;
    else
         result = dataa - datab;
end

endmodule
/****************************************************************************
          One cycle Stall circuit
****************************************************************************/
module onecyclestall(request,clk,resetn,stalled);
input request;
input clk;
input resetn;
output stalled;

  reg T,Tnext;

  // State machine for Stalling 1 cycle
  always@(request or T)
  begin
    case(T) 
      1'b0: Tnext=request;
      1'b1: Tnext=0;
    endcase 
  end       
  always@(posedge clk)
    if (~resetn)
      T<=0; 
    else    
      T<=Tnext;
  assign stalled=(request&~T);
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
///////////////////////////////////////////////////////
// Processing element. Finds sum, min or max depending on mode
///////////////////////////////////////////////////////
module reduction_processing_element_16_20 (
  A, B, OUT, MODE
);

input [16-1:0] A;
input [16-1:0] B;
output [20-1:0] OUT;
input [1:0] MODE;

wire [20-1:0] greater;
wire [20-1:0] smaller;
wire [20-1:0] sum;

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
module reduction_processing_element_20_20 (
  A, B, OUT, MODE
);

input [20-1:0] A;
input [20-1:0] B;
output [20-1:0] OUT;
input [1:0] MODE;

wire [20-1:0] greater;
wire [20-1:0] smaller;
wire [20-1:0] sum;

assign greater = (A>B) ? A : B;
assign smaller = (A<B) ? A : B;
assign sum = A + B;

assign OUT = (MODE==0) ? sum : 
             (MODE==1) ? greater :
             smaller;

endmodule

`ifndef MODULE_RAM_WRAPPER_3_8_32 
`define MODULE_RAM_WRAPPER_3_8_32 
module ram_wrapper_3_8_32 (
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
input [3-1:0] address_a;
input [3-1:0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [32-1:0] data_a;
input [32-1:0] data_b;
output [32-1:0] out_a;
output [32-1:0] out_b;

reg [3-1:0] q_address_a;
reg [3-1:0] q_address_b;
reg [3-1:0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_3_8_32 dpram1(
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
`endif

/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
`ifndef MODULE_PIPEREG_1
`define MODULE_PIPEREG_1
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

endmodule
`endif
/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
`ifndef MODULE_PIPEREG_17
`define MODULE_PIPEREG_17
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

endmodule
`endif
/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
`ifndef MODULE_PIPEREG_1
`define MODULE_PIPEREG_1
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

endmodule
`endif
/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
`ifndef MODULE_PIPEREG_8
`define MODULE_PIPEREG_8
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

endmodule
`endif
/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
`ifndef MODULE_PIPEREG_65
`define MODULE_PIPEREG_65
module pipereg_65(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [65-1:0] d;
output [65-1:0] q;
reg [65-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule
`endif
/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
`ifndef MODULE_PIPEREG_32
`define MODULE_PIPEREG_32
module pipereg_32(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [32-1:0] d;
output [32-1:0] q;
reg [32-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule
`endif
/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
`ifndef MODULE_PIPEREG_5
`define MODULE_PIPEREG_5
module pipereg_5(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [5-1:0] d;
output [5-1:0] q;
reg [5-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule
`endif
/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
`ifndef MODULE_PIPEREG_5
`define MODULE_PIPEREG_5
module pipereg_5(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [5-1:0] d;
output [5-1:0] q;
reg [5-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule
`endif
/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
`ifndef MODULE_PIPEREG_32
`define MODULE_PIPEREG_32
module pipereg_32(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [32-1:0] d;
output [32-1:0] q;
reg [32-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule
`endif
/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
`ifndef MODULE_PIPEREG_6
`define MODULE_PIPEREG_6
module pipereg_6(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [6-1:0] d;
output [6-1:0] q;
reg [6-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule
`endif
/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
`ifndef MODULE_PIPEREG_2
`define MODULE_PIPEREG_2
module pipereg_2(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [2-1:0] d;
output [2-1:0] q;
reg [2-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule
`endif
/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
`ifndef MODULE_PIPEREG_1
`define MODULE_PIPEREG_1
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

endmodule
`endif


module vdispatcher_2_151_6_6_4(
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

input [ 151-1:0 ] inshift_instr;
input                    inshift_first;
input [ 6-1:0 ] inshift_rdelm;
input [ 6-1:0 ] inshift_wrelm;
input [ 4-1:0 ] inshift_count;

input [ 2-1:0 ] increment;

input rdelm_add_sub;
input wrelm_add_sub;
input count_add_sub;
input [ 6-1:0 ] rdelm_valuetoadd;
input [ 6-1:0 ] wrelm_valuetoadd;
input [ 4-1:0 ] count_valuetoadd;

output [ 2*151-1:0 ] instr;
output            [ 2-1:0 ] first;
output [ 2*6-1:0 ] rdelm;
output [ 2*6-1:0 ] wrelm;
output [ 2*4-1:0 ] count;

wire [2*151-1:0 ] inparallel_data_inst_NC;
vdispatcher_shift_2_151 vdispatcher_instr (
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

wire [ 2*6-1:0 ] inparallel_data_rdelm_NC;
wire [ 2-1:0 ] squash_rdelm_NC;
vdispatcher_add_2_6 vdispatcher_rdelm(
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

wire [ 2*6-1:0 ] inparallel_data_wrelm_NC;
wire [ 2-1:0 ] squash_wrelm_NC;
vdispatcher_add_2_6 vdispatcher_wrelm(
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

wire [ 2*4-1:0 ] inparallel_data_count_NC;
wire [ 2-1:0 ] squash_count_NC;
vdispatcher_add_2_4 vdispatcher_count(
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

wire [128-1:0] temp_left;  // Sangram: added to remove warning
wire [128-1:0] temp_right;  // Sangram: added to remove warning
assign temp_left = shiftin_left; 
assign temp_right = shiftin_right; 

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

module velmshifter_1_8 (
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

input [ 1*8-1:0 ]  inpipe;
output [ 1*8-1:0 ] outpipe;

input [ 8-1:0 ]  shiftin_left;
input [ 8-1:0 ]  shiftin_right;

wire [ (1+1)*8-1:0 ] _outpipe;

wire [8-1:0] temp_left;  // Sangram: added to remove warning
wire [8-1:0] temp_right;  // Sangram: added to remove warning
assign temp_left = shiftin_left; 
assign temp_right = shiftin_right; 

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_8 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[7:0],
      _outpipe[15:8], //Support 1 lane
      shiftin_right,
      _outpipe[7:0]);
 // defparam velmshifter_laneunit0.8=8;

  //Generate everything in between 


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_8 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[7:0],
      shiftin_left,
      _outpipe[-1:-8], //L=1
      _outpipe[7:0]); //L=1
   // defparam velmshifter_laneunitlast.8=8;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

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
/****************************************************************************
          Stride Register File

   - Has one read port (a) and one write port (c)
****************************************************************************/
module vregfile_stride_32_8_3 (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

input clk;
input resetn;

input a_en;
input [3-1:0] a_reg,c_reg;
output [32-1:0] a_readdataout;
input [32-1:0] c_writedatain;
input c_we;

        ram_wrapper_3_8_32 reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[3-1:0]),
	    .address_b(a_reg[3-1:0]),
	    .wren_a(c_we),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout)
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

module trp_unit_16 (
 input clk,
 input resetn,
 input en,
 input [8*16-1:0] a,
 input [1:0] mode,
 input read,
 output busy,
 output reg valid,
 output reg[8*16-1:0] out
);

reg en_reduction;
reg en_transpose;
reg read_transpose;
reg read_reduction;
//wire transpose_busy;
wire reduction_busy;
wire reduction_done;
wire [8*16-1:0] reduction_out;
wire [8*16-1:0] transpose_out;
 
 
//assign busy = transpose_busy || reduction_busy;
assign busy = reduction_busy;

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
//  if(transpose_busy)begin
//    out = transpose_out;
//    valid = 1'b1;
//  end
//  else if(reduction_done)begin
//    out = reduction_out;
//    valid = 1'b1;
//  end

  if(reduction_done)begin
    out = reduction_out;
    valid = 1'b1;
  end
  else begin
    out = 'h0;
    valid = 1'b0;
  end
end

reduction_layer_16_4_32_5_10 u_reduction_layer(
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

// transpose_16_2_1 u_transpose(
//   .clk(clk),
//   .resetn(resetn),
//   .read(),
//   .en(en_transpose), 
//   .a(a),
//   .out(transpose_out),
//   .busy(transpose_busy)
// );

endmodule


/****************************************************************************
          Hazard checker
 if mode==0 compare entire width of src & dests,
 if mode==1 compare upper WIDTH-SUBWIDTH bits of src & dests
****************************************************************************/
module hazardchecker_8_3_1_2 (
    src,
    src_valid,
    dst,
    dst_valid,
    dst_mode,
    lt_dst,
    lt_dst_valid,
    lt_mode,
    haz
    );

input [8-1:0]  src;
input              src_valid;
input [8*1-1:0]  dst;
input [1-1:0]    dst_valid;
input [1-1:0]  dst_mode;
input [8*2-1:0]  lt_dst;
input [2-1:0]    lt_dst_valid;
input [2-1:0]  lt_mode;
output             haz;

reg             t_haz;
reg             t_haz_lt;

reg[31:0] i,j;

  always@*
  begin
    t_haz=1'b0;
    t_haz_lt=1'b0;

    for (i=0; i<1; i=i+1)
      if (dst_mode)
        t_haz=t_haz | ((src[8-1:3]==dst[i*8+3 +: 8-3 ])&dst_valid[i]&src_valid);
      else
        t_haz=t_haz | ((src==dst[i*8 +: 8 ])&dst_valid[i]&src_valid);

    //Check if src is less than dest - used for hazards in issuer
    for (j=0; j<2; j=j+1)
      if (lt_mode)
        t_haz_lt=t_haz_lt | ((src[8-1:3]==lt_dst[j*8+3 +: 8-3 ])&lt_dst_valid[j]&src_valid);
      else
        t_haz_lt=t_haz_lt | (((src[8-1:3]==lt_dst[j*8+3 +: 8-3 ])&lt_dst_valid[j]&src_valid) && (src[3-1:0]>=lt_dst[j*8 +: 3]));
  end

  assign haz=(t_haz|t_haz_lt)&src_valid;

endmodule
        /****************************************************************************
          Hazard checker
 if mode==0 compare entire width of src & dests,
 if mode==1 compare upper WIDTH-SUBWIDTH bits of src & dests
****************************************************************************/
module hazardchecker_8_3_4_2 (
    src,
    src_valid,
    dst,
    dst_valid,
    dst_mode,
    lt_dst,
    lt_dst_valid,
    lt_mode,
    haz
    );

input [8-1:0]  src;
input              src_valid;
input [8*4-1:0]  dst;
input [4-1:0]    dst_valid;
input [4-1:0]  dst_mode;
input [8*2-1:0]  lt_dst;
input [2-1:0]    lt_dst_valid;
input [2-1:0]  lt_mode;
output             haz;

reg             t_haz;
reg             t_haz_lt;

reg[31:0] i,j;

  always@*
  begin
    t_haz=1'b0;
    t_haz_lt=1'b0;

    for (i=0; i<4; i=i+1)
      if (dst_mode)
        t_haz=t_haz | ((src[8-1:3]==dst[i*8+3 +: 8-3 ])&dst_valid[i]&src_valid);
      else
        t_haz=t_haz | ((src==dst[i*8 +: 8 ])&dst_valid[i]&src_valid);

    //Check if src is less than dest - used for hazards in issuer
    for (j=0; j<2; j=j+1)
      if (lt_mode)
        t_haz_lt=t_haz_lt | ((src[8-1:3]==lt_dst[j*8+3 +: 8-3 ])&lt_dst_valid[j]&src_valid);
      else
        t_haz_lt=t_haz_lt | (((src[8-1:3]==lt_dst[j*8+3 +: 8-3 ])&lt_dst_valid[j]&src_valid) && (src[3-1:0]>=lt_dst[j*8 +: 3]));
  end

  assign haz=(t_haz|t_haz_lt)&src_valid;

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

module vmul_unit_4_8_3_8 (clk, resetn,
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

parameter NUMLANES=2**3;
parameter WIDTH=2**4;

input clk;
input resetn;

input [NUMLANES*WIDTH-1:0] opA;
input [NUMLANES*WIDTH-1:0] opB;
// input [((4==0) ? 1 : 4)-1:0] vshamt;  // Fixed point rounding
// The original version is the above declaration. This is not supported by VTR. So I am using the below version ~ Aatman
input [4-1:0] vshamt;
input [NUMLANES-1:0] vmask;
input [4:0] op;
input       activate;
input [3:1] en;  //Enable for each pipestage
input [3:1] squash;  //Squash for each pipestage

input    [8-1:0] in_dst;
input                     in_dst_we;
output [3*8-1:0] out_dst;
output              [2:0] out_dst_we;
output   [3*NUMLANES-1:0] out_dst_mask;

output stall;
output [NUMLANES*WIDTH-1:0] result;

  /********* Circuit Body *********/
  wire [8*WIDTH-1:0] mul_opA;
  wire [8*WIDTH-1:0] mul_opB;
  wire [8*WIDTH-1:0] mul_result;
  wire [8*WIDTH-1:0] rshift_result;
  wire [NUMLANES*WIDTH-1:0] result_tmp;

  wire [NUMLANES*WIDTH-1:0] opA_buffered;
  wire [NUMLANES*WIDTH-1:0] opB_buffered;
  wire [NUMLANES-1:0]       mask_buffered;
  wire [NUMLANES*WIDTH-1:0] result_buffered;
  reg  done;
wire [(((3)-(1)+1)*((4)-(0)+1))-1 : 0] ctrl_op;
  wire [3:1] ctrl_activate;                 //3 pipe stages
  //amana: Making modification for VTR
  //wire [((4==0) ? 1 : 4)-1:0] ctrl_vshamt[3:1]; //3 pipe stages
  // wire [((4==0) ? 1*3 : 4*3)-1:0] ctrl_vshamt; //3 pipe stages
  wire [4*3-1:0] ctrl_vshamt; //3 pipe stages

  //Shift Register for all multiplier operands for lanes without multipliers
  wire [WIDTH*8-1:0] opA_elmshifter_shiftin_right_NC;
 
 //TODO: update the parameters
  velmshifter_1_128  opA_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*8{1'b0}}),
    .shiftin_right(opA_elmshifter_shiftin_right_NC), 
    .inpipe(opA),
    .outpipe(opA_buffered)
  );

  wire [WIDTH*8-1:0] opB_elmshifter_shiftin_right_NC;

  //TODO: update the parameters
  velmshifter_1_128 opB_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*8{1'b0}}),
    .shiftin_right(opB_elmshifter_shiftin_right_NC), 
    .inpipe(opB),
    .outpipe(opB_buffered)
  );

  wire [8-1:0] mask_elmshifter_shiftin_right_NC;
  wire mask_elmshifter_load;

  assign mask_elmshifter_load = done & ctrl_activate[1];

  //TODO: Update the parameters 
  velmshifter_1_8  mask_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(mask_elmshifter_load),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({8{1'b0}}),
    .shiftin_right(mask_elmshifter_shiftin_right_NC), 
    //.inpipe(vmask), //DISABLE - always do all multiplications
    .inpipe({NUMLANES{1'b1}}),
    .outpipe(mask_buffered)
  );

  //Shift Register for all multiplier operands w/ parallel load
  always@(posedge clk)
  begin
    if (!resetn || 8==NUMLANES)
      done<=1;
    else if (done && ctrl_activate[1] && en[1])
      //done<=~(|(vmask>>8)); // multiply only if mask - DISABLED
      done<=~(|(vmask));
    else
      done<=~( |(mask_buffered >> (2*8) ));
  end

  assign mul_opA=(done) ? opA : (opA_buffered >> 8*WIDTH);

  assign mul_opB=(done) ? opB : (opB_buffered >> 8*WIDTH);

  assign stall=~done && (ctrl_activate[2]);

  wire [3:1] oppipe_squash_NC; 
  wire [5*(2+1)-1:0] oppipe_q;

  assign {ctrl_op[((3-1)*((4)-(0)+1)+4) : ((3-1)*((4)-(0)+1)+0)],ctrl_op[((2-1)*((4)-(0)+1)+4) : ((2-1)*((4)-(0)+1)+0)],ctrl_op[((1-1)*((4)-(0)+1)+4) : ((1-1)*((4)-(0)+1)+0)]} = oppipe_q;

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

  pipe_4_2 vshamtpipe (
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
    assign vmul0_op = (done&en[1]) ? ctrl_op[((1-1)*((4)-(0)+1)+4) : ((1-1)*((4)-(0)+1)+0)] : ctrl_op[((2-1)*((4)-(0)+1)+4) : ((2-1)*((4)-(0)+1)+0)];

    vlane_mulshift_16_4  vmul0(
      .clk(clk),
      .resetn(resetn),
      .en(vmul0_en),
      .opA(mul_opA[WIDTH*0 +: WIDTH]),
      .opB(mul_opB[WIDTH*0 +: WIDTH]),
      .sa( mul_opB[WIDTH*0+4-1 : WIDTH*0] ),
      .op(vmul0_op),
      .result(mul_result[WIDTH*0 +: WIDTH])
      );

    wire [4-1:0] vshift0_sa;
    wire [2-1:0] vshift0_op;

    assign vshift0_sa = ctrl_vshamt[3*4-1:2*4];
    assign vshift0_op = {~ctrl_op[((2-1)*((4)-(0)+1)+1) : ((2-1)*((4)-(0)+1)+1)] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_16_4 vshift0(
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
    assign vmul1_op = (done&en[1]) ? ctrl_op[((1-1)*((4)-(0)+1)+4) : ((1-1)*((4)-(0)+1)+0)] : ctrl_op[((2-1)*((4)-(0)+1)+4) : ((2-1)*((4)-(0)+1)+0)];

    vlane_mulshift_16_4  vmul1(
      .clk(clk),
      .resetn(resetn),
      .en(vmul1_en),
      .opA(mul_opA[WIDTH*1 +: WIDTH]),
      .opB(mul_opB[WIDTH*1 +: WIDTH]),
      .sa( mul_opB[WIDTH*1+4-1 : WIDTH*1] ),
      .op(vmul1_op),
      .result(mul_result[WIDTH*1 +: WIDTH])
      );

    wire [4-1:0] vshift1_sa;
    wire [2-1:0] vshift1_op;

    assign vshift1_sa = ctrl_vshamt[3*4-1:2*4];
    assign vshift1_op = {~ctrl_op[((2-1)*((4)-(0)+1)+1) : ((2-1)*((4)-(0)+1)+1)] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_16_4 vshift1(
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
    assign vmul2_op = (done&en[1]) ? ctrl_op[((1-1)*((4)-(0)+1)+4) : ((1-1)*((4)-(0)+1)+0)] : ctrl_op[((2-1)*((4)-(0)+1)+4) : ((2-1)*((4)-(0)+1)+0)];

    vlane_mulshift_16_4  vmul2(
      .clk(clk),
      .resetn(resetn),
      .en(vmul2_en),
      .opA(mul_opA[WIDTH*2 +: WIDTH]),
      .opB(mul_opB[WIDTH*2 +: WIDTH]),
      .sa( mul_opB[WIDTH*2+4-1 : WIDTH*2] ),
      .op(vmul2_op),
      .result(mul_result[WIDTH*2 +: WIDTH])
      );

    wire [4-1:0] vshift2_sa;
    wire [2-1:0] vshift2_op;

    assign vshift2_sa = ctrl_vshamt[3*4-1:2*4];
    assign vshift2_op = {~ctrl_op[((2-1)*((4)-(0)+1)+1) : ((2-1)*((4)-(0)+1)+1)] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_16_4 vshift2(
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
    assign vmul3_op = (done&en[1]) ? ctrl_op[((1-1)*((4)-(0)+1)+4) : ((1-1)*((4)-(0)+1)+0)] : ctrl_op[((2-1)*((4)-(0)+1)+4) : ((2-1)*((4)-(0)+1)+0)];

    vlane_mulshift_16_4  vmul3(
      .clk(clk),
      .resetn(resetn),
      .en(vmul3_en),
      .opA(mul_opA[WIDTH*3 +: WIDTH]),
      .opB(mul_opB[WIDTH*3 +: WIDTH]),
      .sa( mul_opB[WIDTH*3+4-1 : WIDTH*3] ),
      .op(vmul3_op),
      .result(mul_result[WIDTH*3 +: WIDTH])
      );

    wire [4-1:0] vshift3_sa;
    wire [2-1:0] vshift3_op;

    assign vshift3_sa = ctrl_vshamt[3*4-1:2*4];
    assign vshift3_op = {~ctrl_op[((2-1)*((4)-(0)+1)+1) : ((2-1)*((4)-(0)+1)+1)] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_16_4 vshift3(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(3+1)-1:WIDTH*3]),
      .sa(vshift3_sa),
      .op(vshift3_op),
      .result(rshift_result[WIDTH*(3+1)-1:WIDTH*3])
      );

        

    wire vmul4_en;
    wire [4:0] vmul4_op;

    assign vmul4_en = en[1] | ~done;
    assign vmul4_op = (done&en[1]) ? ctrl_op[((1-1)*((4)-(0)+1)+4) : ((1-1)*((4)-(0)+1)+0)] : ctrl_op[((2-1)*((4)-(0)+1)+4) : ((2-1)*((4)-(0)+1)+0)];

    vlane_mulshift_16_4  vmul4(
      .clk(clk),
      .resetn(resetn),
      .en(vmul4_en),
      .opA(mul_opA[WIDTH*4 +: WIDTH]),
      .opB(mul_opB[WIDTH*4 +: WIDTH]),
      .sa( mul_opB[WIDTH*4+4-1 : WIDTH*4] ),
      .op(vmul4_op),
      .result(mul_result[WIDTH*4 +: WIDTH])
      );

    wire [4-1:0] vshift4_sa;
    wire [2-1:0] vshift4_op;

    assign vshift4_sa = ctrl_vshamt[3*4-1:2*4];
    assign vshift4_op = {~ctrl_op[((2-1)*((4)-(0)+1)+1) : ((2-1)*((4)-(0)+1)+1)] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_16_4 vshift4(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(4+1)-1:WIDTH*4]),
      .sa(vshift4_sa),
      .op(vshift4_op),
      .result(rshift_result[WIDTH*(4+1)-1:WIDTH*4])
      );

        

    wire vmul5_en;
    wire [4:0] vmul5_op;

    assign vmul5_en = en[1] | ~done;
    assign vmul5_op = (done&en[1]) ? ctrl_op[((1-1)*((4)-(0)+1)+4) : ((1-1)*((4)-(0)+1)+0)] : ctrl_op[((2-1)*((4)-(0)+1)+4) : ((2-1)*((4)-(0)+1)+0)];

    vlane_mulshift_16_4  vmul5(
      .clk(clk),
      .resetn(resetn),
      .en(vmul5_en),
      .opA(mul_opA[WIDTH*5 +: WIDTH]),
      .opB(mul_opB[WIDTH*5 +: WIDTH]),
      .sa( mul_opB[WIDTH*5+4-1 : WIDTH*5] ),
      .op(vmul5_op),
      .result(mul_result[WIDTH*5 +: WIDTH])
      );

    wire [4-1:0] vshift5_sa;
    wire [2-1:0] vshift5_op;

    assign vshift5_sa = ctrl_vshamt[3*4-1:2*4];
    assign vshift5_op = {~ctrl_op[((2-1)*((4)-(0)+1)+1) : ((2-1)*((4)-(0)+1)+1)] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_16_4 vshift5(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(5+1)-1:WIDTH*5]),
      .sa(vshift5_sa),
      .op(vshift5_op),
      .result(rshift_result[WIDTH*(5+1)-1:WIDTH*5])
      );

        

    wire vmul6_en;
    wire [4:0] vmul6_op;

    assign vmul6_en = en[1] | ~done;
    assign vmul6_op = (done&en[1]) ? ctrl_op[((1-1)*((4)-(0)+1)+4) : ((1-1)*((4)-(0)+1)+0)] : ctrl_op[((2-1)*((4)-(0)+1)+4) : ((2-1)*((4)-(0)+1)+0)];

    vlane_mulshift_16_4  vmul6(
      .clk(clk),
      .resetn(resetn),
      .en(vmul6_en),
      .opA(mul_opA[WIDTH*6 +: WIDTH]),
      .opB(mul_opB[WIDTH*6 +: WIDTH]),
      .sa( mul_opB[WIDTH*6+4-1 : WIDTH*6] ),
      .op(vmul6_op),
      .result(mul_result[WIDTH*6 +: WIDTH])
      );

    wire [4-1:0] vshift6_sa;
    wire [2-1:0] vshift6_op;

    assign vshift6_sa = ctrl_vshamt[3*4-1:2*4];
    assign vshift6_op = {~ctrl_op[((2-1)*((4)-(0)+1)+1) : ((2-1)*((4)-(0)+1)+1)] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_16_4 vshift6(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(6+1)-1:WIDTH*6]),
      .sa(vshift6_sa),
      .op(vshift6_op),
      .result(rshift_result[WIDTH*(6+1)-1:WIDTH*6])
      );

        

    wire vmul7_en;
    wire [4:0] vmul7_op;

    assign vmul7_en = en[1] | ~done;
    assign vmul7_op = (done&en[1]) ? ctrl_op[((1-1)*((4)-(0)+1)+4) : ((1-1)*((4)-(0)+1)+0)] : ctrl_op[((2-1)*((4)-(0)+1)+4) : ((2-1)*((4)-(0)+1)+0)];

    vlane_mulshift_16_4  vmul7(
      .clk(clk),
      .resetn(resetn),
      .en(vmul7_en),
      .opA(mul_opA[WIDTH*7 +: WIDTH]),
      .opB(mul_opB[WIDTH*7 +: WIDTH]),
      .sa( mul_opB[WIDTH*7+4-1 : WIDTH*7] ),
      .op(vmul7_op),
      .result(mul_result[WIDTH*7 +: WIDTH])
      );

    wire [4-1:0] vshift7_sa;
    wire [2-1:0] vshift7_op;

    assign vshift7_sa = ctrl_vshamt[3*4-1:2*4];
    assign vshift7_op = {~ctrl_op[((2-1)*((4)-(0)+1)+1) : ((2-1)*((4)-(0)+1)+1)] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_16_4 vshift7(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(7+1)-1:WIDTH*7]),
      .sa(vshift7_sa),
      .op(vshift7_op),
      .result(rshift_result[WIDTH*(7+1)-1:WIDTH*7])
      );

        

//TO DO: parameters

  //Shift Register for all multiplier results
  wire [8*WIDTH-1:0] shiftin_right_result_elmshifter_NC;
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

  assign result=(result_tmp<<((NUMLANES/8-1)*8*WIDTH)) |
                (result_buffered>>8*WIDTH);

  wire [2:1] dstpipe_squash_NC;
  wire [2-1:0] dstpipe_en;
  assign dstpipe_en = en[2:1] & {1'b1,~stall};

  pipe_8_2 dstpipe (
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
  pipe_8_2 dstmaskpipe (
    .d(vmask),  
    .clk(clk),
    .resetn(resetn),
    .en(dstmaskpipe_en),
    .squash(dstmaskpipe_squash_NC),
    .q(out_dst_mask));


endmodule

module dpram_7_128_128 (
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
input [7-1:0] address_a;
input [7-1:0] address_b;
input  wren_a;
input  wren_b;
input [128-1:0] data_a;
input [128-1:0] data_b;
output [128-1:0] out_a;
output[128-1:0] out_b;

dual_port_ram u_dual_port_ram(
.addr1(address_a),
.we1(wren_a),
.data1(data_a),
.out1(out_a),
.addr2(address_b),
.we2(wren_b),
.data2(data_b),
.out2(out_b),
.clk(clk)
);

endmodule



module vdispatcher_add_2_6 (
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
input [ 6-1:0 ] valuetoadd;

input [ 6-1:0 ] inshift_data;

input [ 2*6-1:0 ]  inparallel_data;
output [ 2*6-1:0 ] outparallel_data;

wire [ 6-1:0 ]  shiftin_right;

reg [ 2*6-1:0 ] outparallel_data;
reg [ 2*6-1:0 ] outparallel_added;

wire [ 2-1:0 ] squash_nc;
assign squash_nc = squash;

  assign shiftin_right = (!rotate) ? inshift_data : 
                    outparallel_added[2*6-1:(2-1)*6];

  reg [ 6-1:0 ] v;
  reg[31:0] i;
  always@*
    for (i=0; i<2; i=i+1)
    begin
      //Saturate at 0xffff... in either direction
      v=(increment[i] && outparallel_data[i*6 +: 6] != {6{1'b1}}) ?
        valuetoadd : 0;
      if (add_sub==0)
        outparallel_added[i*6 +: 6]=outparallel_data[i*6+:6] + v;
      else
        outparallel_added[i*6 +: 6]=outparallel_data[i*6+:6] - v;
    end

  always@(posedge clk)
    if (!resetn)
      outparallel_data=0;
    else if (load)
      outparallel_data=inparallel_data;
    else if (shift)
      outparallel_data=(outparallel_added<<6) | shiftin_right;
      


endmodule



module vdispatcher_shift_2_151 (
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

input [ 151-1:0 ] inshift_data;

input [ 2*151-1:0 ]  inparallel_data;
output [ 2*151-1:0 ] outparallel_data;

wire [ 2-1:0 ] squash_nc;
assign squash_nc = squash;

velmshifter_2_151 velmshift (
    .clk(clk),
    .resetn(resetn),      
    .load(load),
    .shift(shift),
    .dir_left(1),
    .squash(squash),
    .shiftin_left((!rotate) ? 0 : outparallel_data[151-1:0]),
    .shiftin_right((!rotate) ? inshift_data : outparallel_data[2*151-1:(2-1)*151]),
    .inpipe(inparallel_data),
    .outpipe(outparallel_data));


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
`ifndef MODULE_RAM_WRAPPER_4_16_32 
`define MODULE_RAM_WRAPPER_4_16_32 
module ram_wrapper_4_16_32 (
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
input [4-1:0] address_a;
input [4-1:0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [32-1:0] data_a;
input [32-1:0] data_b;
output [32-1:0] out_a;
output [32-1:0] out_b;

reg [4-1:0] q_address_a;
reg [4-1:0] q_address_b;
reg [4-1:0] mux_address_b;

// not connect ports
wire rden_a_nc;
assign rden_a_nc = rden_a;

dpram_4_16_32 dpram1(
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
`endif

/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
`ifndef MODULE_PIPE_1_2
`define MODULE_PIPE_1_2
module pipe_1_2(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [1-1:0]  d;
input              clk;
input              resetn;
input  [2-1:0] en;
input  [2-1:0] squash;
output [1*(2+1)-1:0] q;

reg [1*2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 1-1:0 ]<= 0;
    else if (en[0])
      tq[ 1-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i*1 +: 1 ]<= 0;
      else if (en[i])
        tq[i*1 +: 1 ]<=tq[(i-1)*1 +: 1 ];
  end

  assign q[1-1:0]=d;
  assign q[1*(2+1)-1:1]=tq;
endmodule
        `endif
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
`ifndef MODULE_PIPE_5_2
`define MODULE_PIPE_5_2
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
        `endif
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
`ifndef MODULE_PIPE_4_2
`define MODULE_PIPE_4_2
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
        `endif
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
`ifndef MODULE_PIPE_8_2
`define MODULE_PIPE_8_2
module pipe_8_2(
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
input  [2-1:0] en;
input  [2-1:0] squash;
output [8*(2+1)-1:0] q;

reg [8*2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 8-1:0 ]<= 0;
    else if (en[0])
      tq[ 8-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i*8 +: 8 ]<= 0;
      else if (en[i])
        tq[i*8 +: 8 ]<=tq[(i-1)*8 +: 8 ];
  end

  assign q[8-1:0]=d;
  assign q[8*(2+1)-1:8]=tq;
endmodule
        `endif
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
`ifndef MODULE_PIPE_7_5
`define MODULE_PIPE_7_5
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
        `endif
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
`ifndef MODULE_PIPE_1_3
`define MODULE_PIPE_1_3
module pipe_1_3(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [1-1:0]  d;
input              clk;
input              resetn;
input  [3-1:0] en;
input  [3-1:0] squash;
output [1*(3+1)-1:0] q;

reg [1*3-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 1-1:0 ]<= 0;
    else if (en[0])
      tq[ 1-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<3; i=i+1)
      if (!resetn || squash[i] )
        tq[i*1 +: 1 ]<= 0;
      else if (en[i])
        tq[i*1 +: 1 ]<=tq[(i-1)*1 +: 1 ];
  end

  assign q[1-1:0]=d;
  assign q[1*(3+1)-1:1]=tq;
endmodule
        `endif
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
`ifndef MODULE_PIPE_8_1
`define MODULE_PIPE_8_1
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
    if (!resetn || squash )
      tq[ 8-1:0 ]<= 0;
    else if (en)
      tq[ 8-1:0 ]<=d;
  end

  assign q[8-1:0]=d;
  assign q[8*(1+1)-1:8]=tq;
endmodule
        `endif
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
`ifndef MODULE_PIPE_32_4
`define MODULE_PIPE_32_4
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
        `endif
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
`ifndef MODULE_PIPE_8_1
`define MODULE_PIPE_8_1
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
    if (!resetn || squash )
      tq[ 8-1:0 ]<= 0;
    else if (en)
      tq[ 8-1:0 ]<=d;
  end

  assign q[8-1:0]=d;
  assign q[8*(1+1)-1:8]=tq;
endmodule
        `endif
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
`ifndef MODULE_PIPE_8_1
`define MODULE_PIPE_8_1
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
    if (!resetn || squash )
      tq[ 8-1:0 ]<= 0;
    else if (en)
      tq[ 8-1:0 ]<=d;
  end

  assign q[8-1:0]=d;
  assign q[8*(1+1)-1:8]=tq;
endmodule
        `endif


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
module reduction_layer_16_4_32_5_10 
(
  input clk,
  input resetn, //resets the processing elements
  input en, //indicates valid reduction operation
  input  [1:0] reduction_type, //can have 3 values: 0 (Add), 1 (Max), 2 (Min)
  input read,
  input  [8 * 16 -1:0] a, // input data to reduction logic
  output [8 * 16 -1:0] reduced_out, //output
  output reg done, //output is valid when done is 1
  output reg busy
);

wire [8 * 16 -1:0] reduced_out_1; //output
wire [8 * 16 -1:0] reduced_out_add; //output
wire [16 + 4-1:0] reduced_out_unrounded;

wire reset_reduction_unit;
assign  reset_reduction_unit = ~resetn;

reduction_unit_16_4 ucu(
  .clk(clk),
  .reset(reset_reduction_unit),
  .inp0(a[1*16-1:0*16]), 
  .inp1(a[2*16-1:1*16]), 
  .inp2(a[3*16-1:2*16]), 
  .inp3(a[4*16-1:3*16]), 
  .inp4(a[5*16-1:4*16]), 
  .inp5(a[6*16-1:5*16]), 
  .inp6(a[7*16-1:6*16]), 
  .inp7(a[8*16-1:7*16]), 
  .mode(reduction_type),
  .outp(reduced_out_unrounded)
);

////////////////////////////////////////////////////////////////
// Rounding of the output of reduction unit (from 20 bits to 16 bits).
// This is required only when reduction type is "sum"
////////////////////////////////////////////////////////////////
rounding_20_16 u_round(.i_data(reduced_out_unrounded), .o_data(reduced_out_add));

assign reduced_out_1 = (reduction_type==2'b0) ? reduced_out_add : reduced_out_unrounded[16-1:0];
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

module dpram_4_16_32 (
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
input [4-1:0] address_a;
input [4-1:0] address_b;
input  wren_a;
input  wren_b;
input [32-1:0] data_a;
input [32-1:0] data_b;
output [32-1:0] out_a;
output[32-1:0] out_b;

dual_port_ram u_dual_port_ram(
.addr1(address_a),
.we1(wren_a),
.data1(data_a),
.out1(out_a),
.addr2(address_b),
.we2(wren_b),
.data2(data_b),
.out2(out_b),
.clk(clk)
);

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

module vmem_unit_16_8_3_8_3_32_128_7_128_7_3_8 (
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
input  [          8-1 : 0 ]  vmask;
input  [ 8*32-1 : 0 ]  vstrideoffset;
input  [ 8*16-1 : 0 ]  vindex;
input  [ 8*16-1 : 0 ]  vwritedata;
output [ 8*16-1 : 0 ]  voutput;
output [          8-1 : 0 ]  voutput_we;

input   [8-1:0]  in_dst;
input                     in_dst_we;
output   [8-1:0] out_dst;
output                    out_dst_we;
input                     in_vs_dst_we;
output                    out_vs_dst_we;

input  [      3-1 : 0 ]  sa;
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

wire  [ 8*16-1 : 0 ]  __vreaddata;
reg           [ 8*32-1 : 0 ]  address;
reg              [ 8*16-1:0]  vreaddata;
reg                    [ 8-1 : 0 ]  vreaddata_we;
reg   [ 8*(7-5)-1 : 0 ] crossbar_sel;
wire  [ 8*32-1 : 0 ] crossbar;
wire  [ 8*32-1 : 0 ]  _vwritedata;
wire   [ 8*4-1 : 0 ]  _vbyteen;

reg  [ 32-1 : 0 ] stride;
wire [ 32-1 : 0 ] stride_tmp;
reg  [ 32-1 : 0 ] prefetch;
wire [ 32-1 : 0 ] t_cprefetch;

wire           [         8-1 : 0 ]  vshifted_mask;
wire           [         8-1 : 0 ]  vshifted_masksave;
wire          [ 8*16-1 : 0 ]  vshifted_writedata;
wire          [ 8*32-1 : 0 ]  vshifted_address;


reg                           dmem_valid;
reg   [ 31 : 0 ]              dmem_readdata_address;
reg                           cachedata_stillvalid;

reg [2:0] munit_state;
reg [2:0] issue_state;

reg  [ 3-1 : 0 ] vpid;
reg  [ 8-1 : 0 ] done;
wire doneall;

wire  [         8-1 : 0 ]  _vreaddata_we;
reg   [ 8-1 : 0 ]  _parhits;
wire  [ 8-1 : 0 ]  parhits;
reg   [ 8-1 : 0 ]  parhits_done;
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

reg  [ 3-1 : 0 ] sa_count;
wire [ 3-1 : 0 ] next_sa_count;
wire                        next_sa_count_zero;
wire                        sa_zero;
reg  [ 8*16-1 : 0 ]  vshiftresult;

reg [31:0] j;
reg [31:0] l;
reg [31:0] m;
reg [31:0] n;
reg [31:0] p;

  assign {op_memop,op_pattern,op_size,op_signed,op_we}=op;

  assign stall= (munit_state!=MUNIT_IDLE && munit_state!=MUNIT_DONE);

/************************** Pipeline load stage *******************************/
  reg enable_s2;
  reg last_subvector_s2;
  reg [ 8-1 : 0 ] in_dst_s2;
  reg [      3-1 : 0 ] sa_s2;
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
  assign doublewrite=(~op_memop_s2) && (8>1) && ( (dir_left_s2) ? 
              (|vshifted_mask[(8)-2:0]) && //support L=1
                (vshifted_mask[8-1]&(|vshifted_masksave)) :
              (|vshifted_mask[8-1:1]) && //support L=1
                (vshifted_mask[0]&(|vshifted_masksave)));

  assign out_dst = {in_dst_s2[8-1:3],
                    (munit_state!=MUNIT_XTRAWRITE) ? in_dst_s2[3-1:0] :
                     (dir_left_s2) ? in_dst_s2[3-1:0]+1'b1 : 
                       in_dst_s2[3-1:0]-1'b1};

  //Truncate shifted result to VPW's size
  always@*
    for (p=0; p<8; p=p+1)
      vshiftresult[p*16 +: 16]=
              vshifted_address[p*32 +: 16];

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
    for (m=0; m<8; m=m+1)
      address[m*32 +: 32] = ((op_memop) ? cbase : 0) + 
            ( (op_pattern[1] || ~op_memop)  ? vindex[m*16 +: 16] : 
               (vstrideoffset[m*32 +: 32]<<op_size));

  wire [8-1:0] vwritedata_shifter_squash_NC;
  velmshifter_jump_16_8_8 vwritedatashifter(
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

  wire [8-1:0] vaddress_shifter_squash_NC;
  velmshifter_jump_32_8_8 vaddressshifter(
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

  velmshifter_jump_1_8_8 vmaskshifter(
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
  wire    [8-1 : 0]  vmasks_save_inpipe_NC;
  velmshifter_8_1  vmaskssave(
      .clk(clk),
      .resetn(resetn && ~shifter_load),
      .load(1'b0),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .squash(0),
      .shiftin_left( vshifted_mask[0] ),
      .shiftin_right( vshifted_mask[8-1] ),
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
      vpid<=vpid + ((shifter_jump) ? 8 : 1'b1);

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

  assign dmem_en = ((shifter_jump) ? (|vshifted_mask[8-1:0]) : 
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
                2*32-1 : 32 ] :
              vshifted_address[
                  32-1 : 0 ];
                   
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
    .write_data({32'b0,vshifted_writedata[0*16+16-1 : 0*16]}),
    .d_address(vshifted_address[0*32+2-1 : 0*32]),
    .store_size(op_size_s2), 
    .d_byteena(_vbyteen[4*0+4-1 : 4*0]),  
    .d_writedataout(_vwritedata[0*32+32-1 : 0*32]));

        


     vstore_data_translator vstore_data_translator_1(
       //Pad vshifted_writedata with zeros incase signal is less than 32-bits
    .write_data({32'b0,vshifted_writedata[1*16+16-1 : 1*16]}),
    .d_address(vshifted_address[1*32+2-1 : 1*32]),
    .store_size(op_size_s2), 
    .d_byteena(_vbyteen[4*1+4-1 : 4*1]),  
    .d_writedataout(_vwritedata[1*32+32-1 : 1*32]));

        


     vstore_data_translator vstore_data_translator_2(
       //Pad vshifted_writedata with zeros incase signal is less than 32-bits
    .write_data({32'b0,vshifted_writedata[2*16+16-1 : 2*16]}),
    .d_address(vshifted_address[2*32+2-1 : 2*32]),
    .store_size(op_size_s2), 
    .d_byteena(_vbyteen[4*2+4-1 : 4*2]),  
    .d_writedataout(_vwritedata[2*32+32-1 : 2*32]));

        


     vstore_data_translator vstore_data_translator_3(
       //Pad vshifted_writedata with zeros incase signal is less than 32-bits
    .write_data({32'b0,vshifted_writedata[3*16+16-1 : 3*16]}),
    .d_address(vshifted_address[3*32+2-1 : 3*32]),
    .store_size(op_size_s2), 
    .d_byteena(_vbyteen[4*3+4-1 : 4*3]),  
    .d_writedataout(_vwritedata[3*32+32-1 : 3*32]));

        


     vstore_data_translator vstore_data_translator_4(
       //Pad vshifted_writedata with zeros incase signal is less than 32-bits
    .write_data({32'b0,vshifted_writedata[4*16+16-1 : 4*16]}),
    .d_address(vshifted_address[4*32+2-1 : 4*32]),
    .store_size(op_size_s2), 
    .d_byteena(_vbyteen[4*4+4-1 : 4*4]),  
    .d_writedataout(_vwritedata[4*32+32-1 : 4*32]));

        


     vstore_data_translator vstore_data_translator_5(
       //Pad vshifted_writedata with zeros incase signal is less than 32-bits
    .write_data({32'b0,vshifted_writedata[5*16+16-1 : 5*16]}),
    .d_address(vshifted_address[5*32+2-1 : 5*32]),
    .store_size(op_size_s2), 
    .d_byteena(_vbyteen[4*5+4-1 : 4*5]),  
    .d_writedataout(_vwritedata[5*32+32-1 : 5*32]));

        


     vstore_data_translator vstore_data_translator_6(
       //Pad vshifted_writedata with zeros incase signal is less than 32-bits
    .write_data({32'b0,vshifted_writedata[6*16+16-1 : 6*16]}),
    .d_address(vshifted_address[6*32+2-1 : 6*32]),
    .store_size(op_size_s2), 
    .d_byteena(_vbyteen[4*6+4-1 : 4*6]),  
    .d_writedataout(_vwritedata[6*32+32-1 : 6*32]));

        


     vstore_data_translator vstore_data_translator_7(
       //Pad vshifted_writedata with zeros incase signal is less than 32-bits
    .write_data({32'b0,vshifted_writedata[7*16+16-1 : 7*16]}),
    .d_address(vshifted_address[7*32+2-1 : 7*32]),
    .store_size(op_size_s2), 
    .d_byteena(_vbyteen[4*7+4-1 : 4*7]),  
    .d_writedataout(_vwritedata[7*32+32-1 : 7*32]));

        


  always@*
  begin
    dmem_writedata=0;
    dmem_byteen=0;
    for (l=0; l<8; l=l+1)
      if (dmem_address[31:7-3] == 
          vshifted_address[32*l+7-3 +: 
                            32-7+3])
      begin
        dmem_writedata=dmem_writedata| (_vwritedata[l*16 +: 16] << 
            {vshifted_address[32*l+2 +: 7-5], {5{1'b0}}});
        if (vshifted_mask[l] && (shifter_jump || (l==0)))
          dmem_byteen=dmem_byteen | (_vbyteen[2*l+:2]<<
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
    for (j=0; j<8; j=j+1)
      //This signal tells us a cache request succeeded (for either load/store)
      //Each bit corresponds to a parallel lane
      _parhits[j]= vshifted_mask[j] &&
          ((dmem_valid&(dmem_cachematch&~op_we_s2 || op_we_s2&~dmem_wait)) || 
              (cachedata_stillvalid&~op_we_s2)) &&
            (vshifted_address[j*32+7-3 +: 
                             32-7+3] ==
               dmem_readdata_address[31:7-3]);

  //For operations that don't jump, just look at first bit of _parhits
  assign parhits=(shifter_jump) ? _parhits : {8{_parhits[0]}};

  // Detect all parallel lanes hitting
  assign parhits_all=&(parhits|~vshifted_mask[8-1:0]);
  // Detect cache misses
  assign parhits_none=~|(parhits);
  // Detect some misses - fetch next cache line
  assign parhits_some=~parhits_none && ~parhits_all;

  //If 8<=8 then we will never do a jump, so we make
  //compiler ignore this statement if that's the case
  always@(posedge clk)
    if (!resetn || shifter_load)
      parhits_done<= (shifter_jump_s1) ?  ~vmask : {8{~vmask[0]}};
    else if ( parhits_doneall )
    
      parhits_done<= (shifter_jump && (8>8)) ? 
                    ~vshifted_mask[ 
                      8-1:0] :
                    {8{(8>1) ? ~vshifted_mask[1] : 1'b0}};
                    
    else           
      parhits_done<= parhits_done|parhits;

  assign parhits_doneall=&(parhits|parhits_done);

  assign _vreaddata_we= ((shifter_jump) ? _parhits : _parhits[0]) 
                          << vpid[3-1:0];

  always@(posedge clk)
    if (!resetn || ~enable_s2)  // Force to zero if unit not used
      vreaddata_we<= 0 ;
    else           // write to regfile if a) is a load, b) we haven't already
      vreaddata_we<= {8{~op_we_s2}} & _vreaddata_we & ~done;

  //Select signal for crossbar either uses bits from corresponding parallel
  //lane address or all selects are set to the same if we're not jumping
  always@*
    for (n=0; n<8; n=n+1)
      crossbar_sel[(7-5)*n +: (7-5)]=
            (!shifter_jump) ? 
              {8{vshifted_address[2 +: 7-5]}} :
              vshifted_address[n*32+2 +: (7-5)];

  vmem_crossbar_128_7_8_32_5 vmem_crossbar(
      .clk(clk), .resetn(resetn),
      .sel(crossbar_sel),
      .in(dmem_readdata),
      .out(crossbar));

                // Generate byte/halfword alignment circuitry for each word                   

    //  vload_data_translator load_data_translator0(
    //  .d_readdatain(crossbar[32*(0+1)-1:32*0]),
    //  .d_address( (shifter_jump) ? vshifted_address[32*0+2-1 : 32*0] :
    //                               vshifted_address[1:0]),
    //  .load_size(op_size_s2[1:0]),
    //  .load_sign_ext(op_signed_s2),
    //  .d_loadresult(__vreaddata[16*(0+1)-1:16*0])
    //  );
   assign  __vreaddata[16*(0+1)-1:16*0] = crossbar[16*(0+1):16*0];
        


    //  vload_data_translator load_data_translator1(
    //  .d_readdatain(crossbar[32*(1+1)-1:32*1]),
    //  .d_address( (shifter_jump) ? vshifted_address[32*1+2-1 : 32*1] :
    //                               vshifted_address[1:0]),
    //  .load_size(op_size_s2[1:0]),
    //  .load_sign_ext(op_signed_s2),
    //  .d_loadresult(__vreaddata[16*(1+1)-1:16*1])
    //  );
   assign  __vreaddata[16*(1+1)-1:16*1] = crossbar[16*(1+1):16*1];
        


    //  vload_data_translator load_data_translator2(
    //  .d_readdatain(crossbar[32*(2+1)-1:32*2]),
    //  .d_address( (shifter_jump) ? vshifted_address[32*2+2-1 : 32*2] :
    //                               vshifted_address[1:0]),
    //  .load_size(op_size_s2[1:0]),
    //  .load_sign_ext(op_signed_s2),
    //  .d_loadresult(__vreaddata[16*(2+1)-1:16*2])
    //  );
   assign  __vreaddata[16*(2+1)-1:16*2] = crossbar[16*(2+1):16*2];
        


    //  vload_data_translator load_data_translator3(
    //  .d_readdatain(crossbar[32*(3+1)-1:32*3]),
    //  .d_address( (shifter_jump) ? vshifted_address[32*3+2-1 : 32*3] :
    //                               vshifted_address[1:0]),
    //  .load_size(op_size_s2[1:0]),
    //  .load_sign_ext(op_signed_s2),
    //  .d_loadresult(__vreaddata[16*(3+1)-1:16*3])
    //  );
   assign  __vreaddata[16*(3+1)-1:16*3] = crossbar[16*(3+1):16*3];
        


    //  vload_data_translator load_data_translator4(
    //  .d_readdatain(crossbar[32*(4+1)-1:32*4]),
    //  .d_address( (shifter_jump) ? vshifted_address[32*4+2-1 : 32*4] :
    //                               vshifted_address[1:0]),
    //  .load_size(op_size_s2[1:0]),
    //  .load_sign_ext(op_signed_s2),
    //  .d_loadresult(__vreaddata[16*(4+1)-1:16*4])
    //  );
   assign  __vreaddata[16*(4+1)-1:16*4] = crossbar[16*(4+1):16*4];
        


    //  vload_data_translator load_data_translator5(
    //  .d_readdatain(crossbar[32*(5+1)-1:32*5]),
    //  .d_address( (shifter_jump) ? vshifted_address[32*5+2-1 : 32*5] :
    //                               vshifted_address[1:0]),
    //  .load_size(op_size_s2[1:0]),
    //  .load_sign_ext(op_signed_s2),
    //  .d_loadresult(__vreaddata[16*(5+1)-1:16*5])
    //  );
   assign  __vreaddata[16*(5+1)-1:16*5] = crossbar[16*(5+1):16*5];
        


    //  vload_data_translator load_data_translator6(
    //  .d_readdatain(crossbar[32*(6+1)-1:32*6]),
    //  .d_address( (shifter_jump) ? vshifted_address[32*6+2-1 : 32*6] :
    //                               vshifted_address[1:0]),
    //  .load_size(op_size_s2[1:0]),
    //  .load_sign_ext(op_signed_s2),
    //  .d_loadresult(__vreaddata[16*(6+1)-1:16*6])
    //  );
   assign  __vreaddata[16*(6+1)-1:16*6] = crossbar[16*(6+1):16*6];
        


    //  vload_data_translator load_data_translator7(
    //  .d_readdatain(crossbar[32*(7+1)-1:32*7]),
    //  .d_address( (shifter_jump) ? vshifted_address[32*7+2-1 : 32*7] :
    //                               vshifted_address[1:0]),
    //  .load_size(op_size_s2[1:0]),
    //  .load_sign_ext(op_signed_s2),
    //  .d_loadresult(__vreaddata[16*(7+1)-1:16*7])
    //  );
   assign  __vreaddata[16*(7+1)-1:16*7] = crossbar[16*(7+1):16*7];
        


always@(posedge clk)
    //zero if unit not used
    if (!resetn || ~enable_s2 || (~stall&~enable) || op_we_s2)
      vreaddata<= 0 ;
    else                // Don't write to regfile unless this is a load!
      vreaddata<= {8/8{__vreaddata}};

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


module velmshifter_laneunit_151 (
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

input [ 151-1:0 ]  inpipe;
input [ 151-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 151-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 151-1:0 ] outpipe;

reg [ 151-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        
module qadd(a,b,c);
    input [2*`DWIDTH-1:0] a;
    input [2*`DWIDTH-1:0] b;
    output [2*`DWIDTH-1:0] c;

    assign c = a + b;
    //DW01_add #(`DWIDTH) u_add(.A(a), .B(b), .CI(1'b0), .SUM(c), .CO());
endmodule
`define DISPATCHWIDTH 157
 

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

// `define LO(x,b) ((x)&~({1024{1'b1}}<<b))

module vlanes_8_3_8_3_8_64_6_2_1_16_4_2_1_0_128_7_128_7_32_32_32_5 (
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
    mulshift_result_s5,
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
parameter LOG2NUMMULLANES=8;

parameter VRIDWIDTH=5;
parameter VELMIDWIDTH=6-3;
parameter REGIDWIDTH=VRIDWIDTH+6-3;
parameter BANKREGIDWIDTH=VRIDWIDTH+6-3-1;

// Register identifier = { vr[0-31], element }

// `define VRID_RANGE (REGIDWIDTH-1:REGIDWIDTH-VRIDWIDTH)
// `define VRELM_RANGE (REGIDWIDTH-VRIDWIDTH-1:0)

// NUMFUS = ALU*2 + MUL + MEM + FALU*MEMBANKS + MATMUL(1) + BFLOAT
// UNITS(2) + ACTIVATION + TRP
parameter NUMFUS=4+2*(2-1)*0+1+5; //Adding 1 for matmul FU
parameter FU_ALU=0;
parameter FU_MUL=FU_ALU+(2-1)*0+1;
parameter FU_MEM=FU_MUL+1;
parameter FU_FALU=FU_MEM+2*(2-1)*0+1;
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

output [ 16*8-1 : 0 ] mulshift_result_s5;

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
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((32-1)-(0)+1))-1 : 0] vc;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((32-1)-(0)+1))-1 : 0] vl;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((32-1)-(0)+1))-1 : 0] vbase;
reg   [ 32-1 : 0 ]   vbase_s4;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((32-1)-(0)+1))-1 : 0] vinc;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((32-1)-(0)+1))-1 : 0] vstride;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((32-1)-(0)+1))-1 : 0] vs;
reg [(((2-1)-(0)+1)*((32-1)-(0)+1))-1 : 0] vs_s3;
reg [(((NUMFUS-1)-(0)+1)*((32-1)-(0)+1))-1 : 0] vs_s4;
reg [(((2-1)-(0)+1)*((32-1)-(0)+1))-1 : 0] vc_s3;
reg [(((NUMFUS-1)-(0)+1)*((32-1)-(0)+1))-1 : 0] vc_s4;

// Vector register file signals
reg   [ 2*BANKREGIDWIDTH-1 : 0 ]   vr_a_reg;
wire     [ 2*8*2*8-1 : 0 ]   _vr_a_readdataout;
reg                         [2-1:0]   vr_a_en;
reg   [ 2*BANKREGIDWIDTH-1 : 0 ]   vr_b_reg;
wire     [ 2*8*2*8-1 : 0 ]   _vr_b_readdataout;
reg                         [2-1:0]   vr_b_en;
reg   [ 2*BANKREGIDWIDTH-1 : 0 ]   _vr_c_reg;
reg      [ 2*8*2*8-1 : 0 ]   _vr_c_writedatain;
reg        [ 2*2*8-1 : 0 ]   vr_c_byteen;
reg                         [2-1:0]   vr_c_we;

reg [(((2-1)-(0)+1)*((16*8-1)-(0)+1))-1 : 0] vr_a_readdataout;
reg [(((2-1)-(0)+1)*((16*8-1)-(0)+1))-1 : 0] vr_b_readdataout;
reg [(((2-1)-(0)+1)*((16*8-1)-(0)+1))-1 : 0] vr_c_writedatain;
reg [(((2-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] vr_c_reg;

// Flag register file signals
reg       [ 2*BANKREGIDWIDTH-1 : 0 ]   vf_a_reg;
wire               [ 2*8-1 : 0 ]   vf_a_readdataout;
reg                            [ 2-1:0]   vf_a_en;
reg       [ 2*BANKREGIDWIDTH-1 : 0 ]   vf_b_reg;
wire               [ 2*8-1 : 0 ]   vf_b_readdataout;
reg                            [ 2-1:0]   vf_b_en;
reg       [ 2*BANKREGIDWIDTH-1 : 0 ]   vf_c_reg;
reg                [ 2*8-1 : 0 ]   vf_c_writedatain;
reg                            [ 2-1:0]   vf_c_we;

wire [(((NUMFUS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] wb_dst;
wire                     [NUMFUS-1:0]   wb_dst_we;
wire [(((NUMFUS-1)-(0)+1)*((8-1)-(0)+1))-1 : 0] wb_dst_mask;

wire [(((NUMFUS-1)-(0)+1)*((`MAX_PIPE_STAGES-1)-(4)+1))-1 : 0] dst_we;
wire             [(`MAX_PIPE_STAGES-4) * NUMFUS * REGIDWIDTH-1 : 0 ] dst;
wire               [ (`MAX_PIPE_STAGES-4) * NUMFUS * 8-1 : 0 ]   dst_mask;
wire             [ REGIDWIDTH-1 : 0 ]   dst_s2;
reg [(((2-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] _dst_s3;
reg [(((2-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] dst_s3;
reg     [ 2*REGIDWIDTH-1 : 0 ]   t_dst_s3;
reg [(((NUMFUS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] dst_s4;
reg                  [ NUMFUS-1 : 0 ]   dst_we_s4;
reg [(((4)-(4)+1)*(((1+(2-1)*0)*REGIDWIDTH-1)-(0)+1))-1 : 0] alu_dst;
reg [(((4)-(4)+1)*(((1+(2-1)*0)-1)-(0)+1))-1 : 0] alu_dst_we;
reg [(((4)-(4)+1)*(((1+(2-1)*0)*REGIDWIDTH-1)-(0)+1))-1 : 0] falu_dst;
reg [(((4)-(4)+1)*(((1+(2-1)*0)-1)-(0)+1))-1 : 0] falu_dst_we;

wire                                    imask_s2;
reg                [ 2-1 : 0 ]   imask_s3;

wire             [ REGIDWIDTH-1 : 0 ]   src1_s2;
wire             [ REGIDWIDTH-1 : 0 ]   src2_s2;
reg [(((2-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] src1_s3;
reg [(((2-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] src2_s3;
reg [(((2-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] _src1_s3;
reg [(((2-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] _src2_s3;
wire                                    src1scalar_s2;
wire                                    src2scalar_s2;
reg                    [2-1:0]   src1scalar_s3;
reg                    [2-1:0]   src2scalar_s3;
reg                      [NUMFUS-1:0]   src1scalar_s4;
reg                      [NUMFUS-1:0]   src2scalar_s4;

reg [(((2-1)-(0)+1)*((8-1)-(0)+1))-1 : 0] lane_en;
reg [(((NUMFUS-1)-(0)+1)*((8-1)-(0)+1))-1 : 0] vlane_en;
reg                                     mem_last_subvector_s4;

reg                     [6-1:0]   src_start_delayed;
wire                    [6-1:0]   src_elm;
wire                    [6-1:0]   src_limit;
reg [(((2-1)-(0)+1)*((6-1)-(0)+1))-1 : 0] src_limit_s3;
wire                    [6-1:0]   src_start;

wire                    [32-1:0]   total_shamt;
wire                    [6-1:0]   dst_start;

reg [(((NUMFUS-1)-(0)+1)*((16*8-1)-(0)+1))-1 : 0] vr_src1;
reg [(((NUMFUS-1)-(0)+1)*((16*8-1)-(0)+1))-1 : 0] vr_src2;
wire         [ 16*8-1 : 0 ]   matmul_out;
wire         [ 16*8-1 : 0 ]   bfadder_result_s5;
wire         [ 16*8-1 : 0 ]   bfmult_result_s5;
wire         [ 16*8-1 : 0 ]   act_result_s5;
wire         [ 16*8-1 : 0 ]   trp_out;
reg [(((NUMFUS-1)-(0)+1)*((8-1)-(0)+1))-1 : 0] vf_src1;
reg [(((NUMFUS-1)-(0)+1)*((8-1)-(0)+1))-1 : 0] vf_src2;
reg [(((NUMFUS-1)-(0)+1)*((8-1)-(0)+1))-1 : 0] vmask;
reg [(((2-1)-(0)+1)*((8-1)-(0)+1))-1 : 0] vmask_final;

reg        [ 32*8-1 : 0 ]   vstrideoffset_s4;
wire     [ 16*8-1 : 0 ]   load_result_s5;
wire               [ 8-1 : 0 ]   load_result_mask_s5;

wire [((((2-1)*0)-(0)+1)*((16*8-1)-(0)+1))-1 : 0] alu_result_s5;
wire [((((2-1)*0)-(0)+1)*((8-1)-(0)+1))-1 : 0] alu_cmpresult_s4;
wire [((((2-1)*0)-(0)+1)*((8-1)-(0)+1))-1 : 0] flagalu_result_s4;
wire [((((2-1)*0)-(0)+1)*((8-1)-(0)+1))-1 : 0] flagalu_result_s5;

//Support 1 Lane processor
// wire [((3>0) ? 3 : 1)-1:0] elmshamt[`MAX_PIPE_STAGES-1:2];
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((3-1)-(0)+1))-1 : 0] elmshamt;

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
wire [(((33-1)-(1)+1)*((6)-(0)+1))-1 : 0] ctrl_memunit_op;

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
reg [(((2-1)-(0)+1)*((10)-(0)+1))-1 : 0] ctrl3_alu_op;
reg [(((2-1)-(0)+1)*((1)-(0)+1))-1 : 0] ctrl3_satsum_op;
reg [(((2-1)-(0)+1)*((3)-(0)+1))-1 : 0] ctrl3_satsize_op;
reg [(((2-1)-(0)+1)*((4)-(0)+1))-1 : 0] ctrl3_mulshift_op;
reg [2-1:0] ctrl3_memunit_en;
reg [2-1:0] ctrl3_mem_en;
reg [(((2-1)-(0)+1)*((6)-(0)+1))-1 : 0] ctrl3_memunit_op;
reg [2-1:0] ctrl3_ismasked;
reg [(((2-1)-(0)+1)*((2)-(0)+1))-1 : 0] ctrl3_flagalu_op;
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
reg [((((2-1)*0)-(0)+1)*((10)-(0)+1))-1 : 0] ctrl4_alu_op;
reg [((((2-1)*0)-(0)+1)*((1)-(0)+1))-1 : 0] ctrl4_satsum_op;
reg [((((2-1)*0)-(0)+1)*((3)-(0)+1))-1 : 0] ctrl4_satsize_op;
reg [4:0] ctrl4_mulshift_op;
reg ctrl4_memunit_en;
reg ctrl4_mem_en;
reg [6:0] ctrl4_memunit_op;
reg [NUMFUS-1:0] ctrl4_ismasked;
reg [((((2-1)*0)-(0)+1)*((2)-(0)+1))-1 : 0] ctrl4_flagalu_op;
//reg ctrl4_vf_wbsel[(2-1)*0:0];   //0-flag ALU, 1-normal ALU
reg [(2-1)*0:0] ctrl4_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg ctrl4_volatiledest;
//reg ctrl4_vf_a_sel[(2-1)*0:0]; //0-0/1 from instr, 1-src1
reg [(2-1)*0:0] ctrl4_vf_a_sel; //0-0/1 from instr, 1-src1
reg ctrl4_mulshift_en;
reg ctrl4_matmul_en;
reg ctrl4_trp_en;
reg ctrl4_bfadder_en;
reg ctrl4_bfmult_en;
reg ctrl4_act_en;

wire ctrl5_mem_en;

wire [REGIDWIDTH-VRIDWIDTH-1:0] regid_pad;

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

wire [2*6-1:0] rdelm;
wire [2*6-1:0] wrelm;
// MSb of count entries indicates if instruction is dead or not.
wire [2*(6-3+1)-1:0] count;
reg  [2-1:0]   last_subvector;
wire [2-1:0]   first_subvector;
reg  [2-1:0] wrongbank_s3;
reg  [2-1:0] alive_s3;
reg [(((NUMFUS-1)-(0)+1)*((2-1)-(0)+1))-1 : 0] banksel_s4;

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
wire [(((2)-(1)+1)*((7)-(0)+1))-1 : 0] d_instr;
reg [(((2-1)-(0)+1)*((7)-(0)+1))-1 : 0] d_instr_s3;
reg [(((NUMFUS-1)-(0)+1)*((7)-(0)+1))-1 : 0] d_instr_s4;
wire [(((NUMFUS-1)-(0)+1)*((7)-(0)+1))-1 : 0] d_instr_s5;
wire [(((NUMFUS-1)-(0)+1)*((7)-(0)+1))-1 : 0] d_instr_s6;
reg   [NUMFUS-1:0] D_last_subvector_s4;
wire  [NUMFUS-1:0] D_last_subvector_s5;
wire  [NUMFUS-1:0] D_last_subvector_s6;
wire  [NUMFUS-1:0] D_last_subvector_s31;
wire  [NUMFUS-1:0] D_wb_last_subvector;
reg   [2-1:0] D_last_subvector_done;
reg   [2-1:0] D_wb_instrdone;

// Module instance
  wire [15:0] debuginstrpipe_q;
  //assign {d_instr[2],d_instr[1]} = debuginstrpipe_q;
  assign d_instr[((2-1)*((7)-(0)+1)+7) : ((2-1)*((7)-(0)+1)+0)] = debuginstrpipe_q[15:8];
  assign d_instr[((1-1)*((7)-(0)+1)+7) : ((1-1)*((7)-(0)+1)+0)] = debuginstrpipe_q[7:0];

  pipe_8_1  debuginstrpipe (
      .d( {instr[25:24],instr[5:0]} ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[1] ),
      .squash( pipe_squash[1] ),
      .q(debuginstrpipe_q));



// Module instance
    wire debugintrfupipereg1_0_squashn;
    assign debugintrfupipereg1_0_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg1_0 (
      .d( d_instr_s4[((0-0)*((7)-(0)+1)+7) : ((0-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_0_squashn),
      .q(d_instr_s5[((0-0)*((7)-(0)+1)+7) : ((0-0)*((7)-(0)+1)+0)]));

// Module instance
    wire debugintrfupipereg2_0_squashn;
    assign debugintrfupipereg2_0_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_0 (
      .d( d_instr_s5[((0-0)*((7)-(0)+1)+7) : ((0-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_0_squashn),
      .q(d_instr_s6[((0-0)*((7)-(0)+1)+7) : ((0-0)*((7)-(0)+1)+0)]));

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
      .d( d_instr_s4[((1-0)*((7)-(0)+1)+7) : ((1-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_1_squashn),
      .q(d_instr_s5[((1-0)*((7)-(0)+1)+7) : ((1-0)*((7)-(0)+1)+0)]));

// Module instance
    wire debugintrfupipereg2_1_squashn;
    assign debugintrfupipereg2_1_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_1 (
      .d( d_instr_s5[((1-0)*((7)-(0)+1)+7) : ((1-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_1_squashn),
      .q(d_instr_s6[((1-0)*((7)-(0)+1)+7) : ((1-0)*((7)-(0)+1)+0)]));

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
      .d( d_instr_s4[((2-0)*((7)-(0)+1)+7) : ((2-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_2_squashn),
      .q(d_instr_s5[((2-0)*((7)-(0)+1)+7) : ((2-0)*((7)-(0)+1)+0)]));

// Module instance
    wire debugintrfupipereg2_2_squashn;
    assign debugintrfupipereg2_2_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_2 (
      .d( d_instr_s5[((2-0)*((7)-(0)+1)+7) : ((2-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_2_squashn),
      .q(d_instr_s6[((2-0)*((7)-(0)+1)+7) : ((2-0)*((7)-(0)+1)+0)]));

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
      .d( d_instr_s4[((3-0)*((7)-(0)+1)+7) : ((3-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_3_squashn),
      .q(d_instr_s5[((3-0)*((7)-(0)+1)+7) : ((3-0)*((7)-(0)+1)+0)]));

// Module instance
    wire debugintrfupipereg2_3_squashn;
    assign debugintrfupipereg2_3_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_3 (
      .d( d_instr_s5[((3-0)*((7)-(0)+1)+7) : ((3-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_3_squashn),
      .q(d_instr_s6[((3-0)*((7)-(0)+1)+7) : ((3-0)*((7)-(0)+1)+0)]));

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
      .d( d_instr_s4[((4-0)*((7)-(0)+1)+7) : ((4-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_4_squashn),
      .q(d_instr_s5[((4-0)*((7)-(0)+1)+7) : ((4-0)*((7)-(0)+1)+0)]));

// Module instance
    wire debugintrfupipereg2_4_squashn;
    assign debugintrfupipereg2_4_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_4 (
      .d( d_instr_s5[((4-0)*((7)-(0)+1)+7) : ((4-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_4_squashn),
      .q(d_instr_s6[((4-0)*((7)-(0)+1)+7) : ((4-0)*((7)-(0)+1)+0)]));

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
      .d( d_instr_s4[((5-0)*((7)-(0)+1)+7) : ((5-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_5_squashn),
      .q(d_instr_s5[((5-0)*((7)-(0)+1)+7) : ((5-0)*((7)-(0)+1)+0)]));

// Module instance
    wire debugintrfupipereg2_5_squashn;
    assign debugintrfupipereg2_5_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_5 (
      .d( d_instr_s5[((5-0)*((7)-(0)+1)+7) : ((5-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_5_squashn),
      .q(d_instr_s6[((5-0)*((7)-(0)+1)+7) : ((5-0)*((7)-(0)+1)+0)]));

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
      .d( d_instr_s4[((6-0)*((7)-(0)+1)+7) : ((6-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_6_squashn),
      .q(d_instr_s5[((6-0)*((7)-(0)+1)+7) : ((6-0)*((7)-(0)+1)+0)]));

// Module instance
    wire debugintrfupipereg2_6_squashn;
    assign debugintrfupipereg2_6_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_6 (
      .d( d_instr_s5[((6-0)*((7)-(0)+1)+7) : ((6-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_6_squashn),
      .q(d_instr_s6[((6-0)*((7)-(0)+1)+7) : ((6-0)*((7)-(0)+1)+0)]));

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
      .d( d_instr_s4[((7-0)*((7)-(0)+1)+7) : ((7-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_7_squashn),
      .q(d_instr_s5[((7-0)*((7)-(0)+1)+7) : ((7-0)*((7)-(0)+1)+0)]));

// Module instance
    wire debugintrfupipereg2_7_squashn;
    assign debugintrfupipereg2_7_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_7 (
      .d( d_instr_s5[((7-0)*((7)-(0)+1)+7) : ((7-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_7_squashn),
      .q(d_instr_s6[((7-0)*((7)-(0)+1)+7) : ((7-0)*((7)-(0)+1)+0)]));

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
      .d( d_instr_s4[((8-0)*((7)-(0)+1)+7) : ((8-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_8_squashn),
      .q(d_instr_s5[((8-0)*((7)-(0)+1)+7) : ((8-0)*((7)-(0)+1)+0)]));

// Module instance
    wire debugintrfupipereg2_8_squashn;
    assign debugintrfupipereg2_8_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_8 (
      .d( d_instr_s5[((8-0)*((7)-(0)+1)+7) : ((8-0)*((7)-(0)+1)+0)] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_8_squashn),
      .q(d_instr_s6[((8-0)*((7)-(0)+1)+7) : ((8-0)*((7)-(0)+1)+0)]));

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
  wire [65:0] pipe1reg_q;

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

  pipereg_65  pipe1reg (
      .d( {
        (ctrl1_vrdest_sel) ? {{REGIDWIDTH-VRIDWIDTH{1'b0}},ir_src2} : {ir_dst, regid_pad},
        (ctrl1_vr_c_en ) ? {{REGIDWIDTH-VRIDWIDTH{1'b0}},ir_dst} : {ir_src1, regid_pad},
        {ir_src2,regid_pad},
        ir_op[7] & ctrl1_usesvssel,
        ir_op[6] & ctrl1_usesvssel,
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
  assign {ctrl_memunit_op[((6-1)*((6)-(0)+1)+6) : ((6-1)*((6)-(0)+1)+0)],ctrl_memunit_op[((5-1)*((6)-(0)+1)+6) : ((5-1)*((6)-(0)+1)+0)],ctrl_memunit_op[((4-1)*((6)-(0)+1)+6) : ((4-1)*((6)-(0)+1)+0)],ctrl_memunit_op[((3-1)*((6)-(0)+1)+6) : ((3-1)*((6)-(0)+1)+0)],ctrl_memunit_op[((2-1)*((6)-(0)+1)+6) : ((2-1)*((6)-(0)+1)+0)],ctrl_memunit_op[((1-1)*((6)-(0)+1)+6) : ((1-1)*((6)-(0)+1)+0)]} = ctrlmemoppipe_q;

// module instance
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

  assign src_start= ( ctrl2_srcshamt_sel==3 ) ? vc[((2-2)*((32-1)-(0)+1)+32-1) : ((2-2)*((32-1)-(0)+1)+0)] :           // vcindex
               ( ctrl2_srcshamt_sel==1 ) ? vl[((2-2)*((32-1)-(0)+1)+32-1) : ((2-2)*((32-1)-(0)+1)+1)] :   // vl>>2
               ( ctrl2_srcshamt_sel==2 ) ? 1 << vc[((2-2)*((32-1)-(0)+1)+6-1) : ((2-2)*((32-1)-(0)+1)+0)]://2^vcindex
               0;

  assign src_elm=src_start_delayed & (-1<<3);

  assign src_limit= ((ctrl2_setvlto1) ? 0 : vl[((2-2)*((32-1)-(0)+1)+6-1) : ((2-2)*((32-1)-(0)+1)+0)] - 1'b1) +
                    ((ctrl2_srclimit_sel) ? vc[((2-2)*((32-1)-(0)+1)+6-1) : ((2-2)*((32-1)-(0)+1)+0)] : 0);


  /******************* Adjust dest to account for shift  ****************/

  // Compute real destination register - accounting for shifting instructions

  assign dst_start= ( ctrl2_dstshamt_sel==3 ) ? vc[((2-2)*((32-1)-(0)+1)+32-1) : ((2-2)*((32-1)-(0)+1)+0)] :           // vcindex
               ( ctrl2_dstshamt_sel==2 ) ? 1 << vc[((2-2)*((32-1)-(0)+1)+6-1) : ((2-2)*((32-1)-(0)+1)+0)]://2^vcindex
               0;

  //assign dst_elm = {dst_start[6-1:0]>>3,{3{1'b0}}};

  assign total_shamt= ( ctrl2_elmshamt_sel==3 ) ? vc[((2-2)*((32-1)-(0)+1)+32-1) : ((2-2)*((32-1)-(0)+1)+0)] :          // vcindex
               ( ctrl2_elmshamt_sel==1 ) ? vl[((2-2)*((32-1)-(0)+1)+32-1) : ((2-2)*((32-1)-(0)+1)+1)] :    // vl>>2
               ( ctrl2_elmshamt_sel==2 ) ? 1 << vc[((2-2)*((32-1)-(0)+1)+6-1) : ((2-2)*((32-1)-(0)+1)+0)]://2^vcindex
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
  assign {vc[((6-2)*((32-1)-(0)+1)+32-1) : ((6-2)*((32-1)-(0)+1)+0)],vc[((5-2)*((32-1)-(0)+1)+32-1) : ((5-2)*((32-1)-(0)+1)+0)],vc[((4-2)*((32-1)-(0)+1)+32-1) : ((4-2)*((32-1)-(0)+1)+0)],vc[((3-2)*((32-1)-(0)+1)+32-1) : ((3-2)*((32-1)-(0)+1)+0)],vc[((2-2)*((32-1)-(0)+1)+32-1) : ((2-2)*((32-1)-(0)+1)+0)]} = vcpipe_q;
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
  
  wire [5*32-1:0] vlpipe_q;
  assign {vl[((6-2)*((32-1)-(0)+1)+32-1) : ((6-2)*((32-1)-(0)+1)+0)],vl[((5-2)*((32-1)-(0)+1)+32-1) : ((5-2)*((32-1)-(0)+1)+0)],vl[((4-2)*((32-1)-(0)+1)+32-1) : ((4-2)*((32-1)-(0)+1)+0)],vl[((3-2)*((32-1)-(0)+1)+32-1) : ((3-2)*((32-1)-(0)+1)+0)],vl[((2-2)*((32-1)-(0)+1)+32-1) : ((2-2)*((32-1)-(0)+1)+0)]} = vlpipe_q;
//module instance 
  pipe_32_4 vlpipe (
      .d( (!pipe_advance_s2_r) ? vl_in_saved : vl_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[5:2] & {3'b1,ctrl2_memunit_en} ),
      .squash(squash_vlpipe_NC),
      .q(vlpipe_q));

  wire [5:2] squash_vbasepipe_NC;

//module instance 
  pipe_32_4 vbasepipe (
      .d( (!pipe_advance_s2_r) ? vbase_in_saved : vbase_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[5:2] & {3'b1,ctrl2_memunit_en} ),
      .squash(squash_vbasepipe_NC),
      .q( {vbase[((6-2)*((32-1)-(0)+1)+32-1) : ((6-2)*((32-1)-(0)+1)+0)],vbase[((5-2)*((32-1)-(0)+1)+32-1) : ((5-2)*((32-1)-(0)+1)+0)],vbase[((4-2)*((32-1)-(0)+1)+32-1) : ((4-2)*((32-1)-(0)+1)+0)],vbase[((3-2)*((32-1)-(0)+1)+32-1) : ((3-2)*((32-1)-(0)+1)+0)],vbase[((2-2)*((32-1)-(0)+1)+32-1) : ((2-2)*((32-1)-(0)+1)+0)]} ));

  wire [5:2] squash_vincpipe_NC;

//module instance 
  pipe_32_4 vincpipe (
      .d( (!pipe_advance_s2_r) ? vinc_in_saved : vinc_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[5:2] & {3'b1,ctrl2_memunit_en} ),
      .squash(squash_vincpipe_NC),
      .q( {vinc[((6-2)*((32-1)-(0)+1)+32-1) : ((6-2)*((32-1)-(0)+1)+0)],vinc[((5-2)*((32-1)-(0)+1)+32-1) : ((5-2)*((32-1)-(0)+1)+0)],vinc[((4-2)*((32-1)-(0)+1)+32-1) : ((4-2)*((32-1)-(0)+1)+0)],vinc[((3-2)*((32-1)-(0)+1)+32-1) : ((3-2)*((32-1)-(0)+1)+0)],vinc[((2-2)*((32-1)-(0)+1)+32-1) : ((2-2)*((32-1)-(0)+1)+0)]} ));

  //stride register also used for elmt shamt for vext/vhalf/etc in memunit
  wire [5:2] squash_vstridepipe_NC;

//module instance 
  pipe_32_4 vstridepipe (
      .d( (ctrl2_memunit_en&~ctrl2_mem_en) ? 
            ( 
                  total_shamt[3-1:0] ) :
          (!pipe_advance_s2_r) ? vstride_in_saved : vstride_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[5:2] & {3'b1,ctrl2_memunit_en} ),
      .squash(squash_vstridepipe_NC),
      .q( {vstride[((6-2)*((32-1)-(0)+1)+32-1) : ((6-2)*((32-1)-(0)+1)+0)],vstride[((5-2)*((32-1)-(0)+1)+32-1) : ((5-2)*((32-1)-(0)+1)+0)],vstride[((4-2)*((32-1)-(0)+1)+32-1) : ((4-2)*((32-1)-(0)+1)+0)],vstride[((3-2)*((32-1)-(0)+1)+32-1) : ((3-2)*((32-1)-(0)+1)+0)],vstride[((2-2)*((32-1)-(0)+1)+32-1) : ((2-2)*((32-1)-(0)+1)+0)]} ));


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
    (|(ctrl3_alufalu_en&~last_subvector) && ctrl2_alufalu_en && 0==0)||
    //Entry 0 is taken
    (dispatcher_rotate && ctrl2_useslanes);

  //assign stall_mulcooldown=|(ctrl3_mulshift_en&last_subvector);
  assign stall_mulcooldown=1'b0;

  assign dispatcher_shift=pipe_advance[3];
  // Rotate if last entry is alive (count not -1) and greater than zero
  assign dispatcher_rotate=(~count[2*(6-3+1)-1]) && 
                          ((count>>((2-1)*(6-3+1)))!=0);

wire [2*(`DISPATCHWIDTH)-1:0] dispatcher_instr;

// module instance
  vdispatcher_2_151_6_6_4 vdispatcher(
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
          (pipe_squash[2]) ? 8'b0 : d_instr[((2-1)*((7)-(0)+1)+7) : ((2-1)*((7)-(0)+1)+0)]
          }),
      .inshift_first(ctrl2_useslanes),
      .inshift_rdelm(src_start),
      .inshift_wrelm(dst_start),
      .inshift_count((pipe_squash[2] || !ctrl2_useslanes) ? -1 : (src_limit[6-1:0]>>3) - (src_elm[6-1:0]>>3) ),
      .increment((~wrongbank_s3)&{2{pipe_advance[3]}}),
      .rdelm_add_sub(0),
      .wrelm_add_sub(0),
      .count_add_sub(1),
      .rdelm_valuetoadd(8),
      .wrelm_valuetoadd(8),
      .count_valuetoadd(1),
      .instr(dispatcher_instr),
      .first(first_subvector),
      .rdelm(rdelm),
      .wrelm(wrelm),
      .count(count)
    );

  

  /******************* Stall on RAW HAZARDS ************************/

  always@*
    for (bd=0; bd<1+(2-1)*0; bd=bd+1)
    begin
      alu_dst[((4-4)*(((1+(2-1)*0)*REGIDWIDTH-1)-(0)+1)+bd*REGIDWIDTH ) +: ( REGIDWIDTH)] = dst[ 4*(NUMFUS*REGIDWIDTH) + (FU_ALU+bd) * REGIDWIDTH +: REGIDWIDTH];

      alu_dst_we[((4-4)*(((1+(2-1)*0)-1)-(0)+1)+bd) : ((4-4)*(((1+(2-1)*0)-1)-(0)+1)+bd)] = dst_we[((FU_ALU+bd-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4) : ((FU_ALU+bd-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4)];

      falu_dst[((4-4)*(((1+(2-1)*0)*REGIDWIDTH-1)-(0)+1)+bd*REGIDWIDTH ) +: ( REGIDWIDTH)] = dst[4*(NUMFUS*REGIDWIDTH) + (FU_ALU+bd) * REGIDWIDTH +: REGIDWIDTH];

      falu_dst_we[((4-4)*(((1+(2-1)*0)-1)-(0)+1)+bd) : ((4-4)*(((1+(2-1)*0)-1)-(0)+1)+bd)] = dst_we[((FU_FALU+bd-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4) : ((FU_FALU+bd-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4)];
    end

//module instance 
  hazardchecker_8_3_4_2 src1hazchecker(
      .src( src1_s2 |(src_start_delayed[6-1:0]>>3) ),
      .src_valid(ctrl2_vr_a_en),
      .dst({
        alu_dst,
        dst[4*(NUMFUS*REGIDWIDTH)+(FU_MUL) * REGIDWIDTH +: REGIDWIDTH],
        dst[4*(NUMFUS*REGIDWIDTH)+(FU_MEM) * REGIDWIDTH +: REGIDWIDTH],
        dst[4*(NUMFUS*REGIDWIDTH)+(FU_MATMUL) * REGIDWIDTH +: REGIDWIDTH]
        }),
    //  .dst(),
      .dst_valid({
        alu_dst_we[((4-4)*(((1+(2-1)*0)-1)-(0)+1)+(1+(2-1)*0)-1) : ((4-4)*(((1+(2-1)*0)-1)-(0)+1)+0)],
        dst_we[((FU_MUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4) : ((FU_MUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4)],
        dst_we[((FU_MEM-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4) : ((FU_MEM-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4)],
        dst_we[((FU_MATMUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4) : ((FU_MATMUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4)]
        }),
      .dst_mode({{3+(2-1)*0{1'b0}},ctrl4_volatiledest}),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vr_c_we),
      .lt_mode(ctrl3_volatiledest),
      .haz(stall_hazsrc1));
//module instance 

  hazardchecker_8_3_4_2 src2hazchecker(
      .src( src2_s2 |(src_start_delayed[6-1:0]>>3) ),
      .src_valid(ctrl2_vr_b_en),
      .dst({
        alu_dst,
        dst[4*(NUMFUS*REGIDWIDTH)+(FU_MUL) * REGIDWIDTH +: REGIDWIDTH],
        dst[4*(NUMFUS*REGIDWIDTH)+(FU_MEM) * REGIDWIDTH +: REGIDWIDTH],
        dst[4*(NUMFUS*REGIDWIDTH)+(FU_MATMUL) * REGIDWIDTH +: REGIDWIDTH]
        }),
     // .dst(),
      .dst_valid({
        alu_dst_we[((4-4)*(((1+(2-1)*0)-1)-(0)+1)+(1+(2-1)*0)-1) : ((4-4)*(((1+(2-1)*0)-1)-(0)+1)+0)],
        dst_we[((FU_MUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4) : ((FU_MUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4)],
        dst_we[((FU_MEM-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4) : ((FU_MEM-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4)],
        dst_we[((FU_MATMUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4) : ((FU_MATMUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4)]
        }),
      .dst_mode({{3+(2-1)*0{1'b0}},ctrl4_volatiledest}),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vr_c_we),
      .lt_mode(ctrl3_volatiledest),
      .haz(stall_hazsrc2));

  //Check flag hazards - flags always start at 0th element
//module instance 
  hazardchecker_8_3_1_2 fsrc1hazchecker(
      // Check for mask flag and src1 flag depending on instruction
      .src( (ctrl2_vf_a_sel) ? src1_s2 : (imask_s2<<VELMIDWIDTH)),
      .src_valid(ctrl2_vf_a_en),
      .dst(falu_dst),
      .dst_valid(falu_dst_we),
      .dst_mode(0),
      //.lt_dst(t_dst_s3),
      .lt_dst(0),
      //.lt_dst_valid(ctrl3_vf_c_we),
      .lt_dst_valid(0),
      .lt_mode(0),
      .haz(stall_hazfsrc1));

//module instance 
  hazardchecker_8_3_1_2 fsrc2hazchecker(
      .src( src2_s2 ),
      .src_valid(ctrl2_vf_b_en),
      .dst(falu_dst),
      .dst_valid(falu_dst_we),
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
        src_limit_s3[((bi-0)*((6-1)-(0)+1)+6-1) : ((bi-0)*((6-1)-(0)+1)+0)],
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
        _dst_s3[((bi-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((bi-0)*((REGIDWIDTH-1)-(0)+1)+0)],
        _src1_s3[((bi-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((bi-0)*((REGIDWIDTH-1)-(0)+1)+0)],
        _src2_s3[((bi-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((bi-0)*((REGIDWIDTH-1)-(0)+1)+0)],
        src1scalar_s3[bi],
        src2scalar_s3[bi],
        imask_s3[bi],
        ctrl3_vf_a_sel[bi],
        ctrl3_rshiftnonzero[bi],
        ctrl3_mem_dir_left[bi],
        ctrl3_alu_op[((bi-0)*((10)-(0)+1)+10) : ((bi-0)*((10)-(0)+1)+0)],
        ctrl3_satsum_op[((bi-0)*((1)-(0)+1)+1) : ((bi-0)*((1)-(0)+1)+0)],
        ctrl3_satsize_op[((bi-0)*((3)-(0)+1)+3) : ((bi-0)*((3)-(0)+1)+0)],
        ctrl3_mulshift_op[((bi-0)*((4)-(0)+1)+4) : ((bi-0)*((4)-(0)+1)+0)],
        ctrl3_memunit_op[((bi-0)*((6)-(0)+1)+6) : ((bi-0)*((6)-(0)+1)+0)],
        ctrl3_ismasked[bi],
        ctrl3_flagalu_op[((bi-0)*((2)-(0)+1)+2) : ((bi-0)*((2)-(0)+1)+0)],
        ctrl3_vf_wbsel[bi],
        ctrl3_volatiledest[bi],
        vc_s3[((bi-0)*((32-1)-(0)+1)+32-1) : ((bi-0)*((32-1)-(0)+1)+0)],
        vs_s3[((bi-0)*((32-1)-(0)+1)+32-1) : ((bi-0)*((32-1)-(0)+1)+0)],
        d_instr_s3[((bi-0)*((7)-(0)+1)+7) : ((bi-0)*((7)-(0)+1)+0)]
      } = dispatcher_instr>>(bi*(`DISPATCHWIDTH));

      last_subvector[bi]=~|count[bi*(6-3+1)+:6-3];
      alive_s3[bi]=~count[(bi+1)*(6-3+1)-1];

//      for (i=0; i<8; i=i+1)
//        lane_en[bi][i]= (3==0) || //Support 1 lane
//          ~((first_subvector[bi]) && 
//              i<((rdelm[bi*6 +: 6])&~({64{1'b1}}<<3)) ||
//            (last_subvector[bi]) && 
//              i>src_limit_s3[bi][((3>0) ? 3 : 1)-1:0] );
    end


  /************* Map from issuer to register file banks *************/
  always@*
    for (b=0; b<2; b=b+1)
    begin 
      dst_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b-0)*((REGIDWIDTH-1)-(0)+1)+0)]=_dst_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b-0)*((REGIDWIDTH-1)-(0)+1)+0)] | (wrelm[b*6+:6]>>3);
      t_dst_s3[b*REGIDWIDTH +: REGIDWIDTH]=dst_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b-0)*((REGIDWIDTH-1)-(0)+1)+0)];
      src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b-0)*((REGIDWIDTH-1)-(0)+1)+0)]=_src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b-0)*((REGIDWIDTH-1)-(0)+1)+0)] | (rdelm[b*6+:6]>>3);
      src2_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b-0)*((REGIDWIDTH-1)-(0)+1)+0)]=_src2_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b-0)*((REGIDWIDTH-1)-(0)+1)+0)] | (rdelm[b*6+:6]>>3);

      // wrongbank_s3[b]=(((rdelm[b*6+:6]>>3)&~({64{1'b1}}<<1))!=b);
       wrongbank_s3[b]=(((rdelm[b*6+:6]>>3)&({{(8-1){1'b0}},{1{1'b1}}})) != b );

      vr_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b-0)*((REGIDWIDTH-1)-(0)+1)+0)]>>1;
      vr_b_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src2_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b-0)*((REGIDWIDTH-1)-(0)+1)+0)]>>1;
      vr_a_en[b]=ctrl3_vr_a_en[b] && pipe_advance[3];
      vr_b_en[b]=ctrl3_vr_b_en[b] && pipe_advance[3];
     
    if ((ctrl3_vf_a_sel[b]))
    begin
      vf_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH] = _src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-VRIDWIDTH)] >> 1;
    end
    else
    begin
      vf_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH] = {imask_s3[b], src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-VRIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1)+0)]} >> 1;
    end
      vf_a_en[b]=ctrl3_vf_a_en[b] & pipe_advance[3];
      vf_b_en[b]=ctrl3_vf_b_en[b] & pipe_advance[3];
    end

  /************* Map from issuer/banks to Functional Units *************/
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
        d_instr_s4[((f3-0)*((7)-(0)+1)+7) : ((f3-0)*((7)-(0)+1)+0)]=0;
        D_last_subvector_s4[f3]=0;
        dst_s4[((f3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((f3-0)*((REGIDWIDTH-1)-(0)+1)+0)]=0;
      end
      for (b3=0; b3<2; b3=b3+1)
      begin
        //if instruction is alive && is in correct bank
        if ( alive_s3[b3]  &&     //alive
            ~wrongbank_s3[b3] &&  //correct bank
            ~pipe_squash[3])
          if (ctrl3_mulshift_en[b3])    //is multiply
          begin
            d_instr_s4[((FU_MUL-0)*((7)-(0)+1)+7) : ((FU_MUL-0)*((7)-(0)+1)+0)]=d_instr_s3[((b3-0)*((7)-(0)+1)+7) : ((b3-0)*((7)-(0)+1)+0)];
            D_last_subvector_s4[FU_MUL]=last_subvector[b3];
            ctrl4_mulshift_en=ctrl3_mulshift_en[b3];
            banksel_s4[((FU_MUL-0)*((2-1)-(0)+1)+2-1) : ((FU_MUL-0)*((2-1)-(0)+1)+0)]=b3;
            vs_s4[((FU_MUL-0)*((32-1)-(0)+1)+32-1) : ((FU_MUL-0)*((32-1)-(0)+1)+0)]=vs_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            vc_s4[((FU_MUL-0)*((32-1)-(0)+1)+32-1) : ((FU_MUL-0)*((32-1)-(0)+1)+0)]=vc_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            dst_s4[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+0)]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b3-0)*((REGIDWIDTH-1)-(0)+1)+0)];
            dst_we_s4[FU_MUL]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_MUL-0)*((8-1)-(0)+1)+8-1) : ((FU_MUL-0)*((8-1)-(0)+1)+0)]=lane_en[((b3-0)*((8-1)-(0)+1)+8-1) : ((b3-0)*((8-1)-(0)+1)+0)];
            src1scalar_s4[FU_MUL]=src1scalar_s3[b3];
            src2scalar_s4[FU_MUL]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MUL]=ctrl3_ismasked[b3];
            ctrl4_mulshift_op=ctrl3_mulshift_op[((b3-0)*((4)-(0)+1)+4) : ((b3-0)*((4)-(0)+1)+0)];
            ctrl4_rshiftnonzero=ctrl3_rshiftnonzero[b3];
          end
          else if (ctrl3_matmul_en[b3])    //is matmul
          begin
            d_instr_s4[((FU_MATMUL-0)*((7)-(0)+1)+7) : ((FU_MATMUL-0)*((7)-(0)+1)+0)]=d_instr_s3[((b3-0)*((7)-(0)+1)+7) : ((b3-0)*((7)-(0)+1)+0)];
            D_last_subvector_s4[FU_MATMUL]=last_subvector[b3];
            ctrl4_matmul_en=ctrl3_matmul_en[b3];
            banksel_s4[((FU_MATMUL-0)*((2-1)-(0)+1)+2-1) : ((FU_MATMUL-0)*((2-1)-(0)+1)+0)]=b3;
            vs_s4[((FU_MATMUL-0)*((32-1)-(0)+1)+32-1) : ((FU_MATMUL-0)*((32-1)-(0)+1)+0)]=vs_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            vc_s4[((FU_MATMUL-0)*((32-1)-(0)+1)+32-1) : ((FU_MATMUL-0)*((32-1)-(0)+1)+0)]=vc_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            dst_s4[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+0)]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b3-0)*((REGIDWIDTH-1)-(0)+1)+0)];
            dst_we_s4[FU_MATMUL]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_MATMUL-0)*((8-1)-(0)+1)+8-1) : ((FU_MATMUL-0)*((8-1)-(0)+1)+0)]=lane_en[((b3-0)*((8-1)-(0)+1)+8-1) : ((b3-0)*((8-1)-(0)+1)+0)];
            src1scalar_s4[FU_MATMUL]=src1scalar_s3[b3];
            src2scalar_s4[FU_MATMUL]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MATMUL]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_bfadder_en[b3])    //is bfloat addition
          begin
            d_instr_s4[((FU_BFADDER-0)*((7)-(0)+1)+7) : ((FU_BFADDER-0)*((7)-(0)+1)+0)]=d_instr_s3[((b3-0)*((7)-(0)+1)+7) : ((b3-0)*((7)-(0)+1)+0)];
            D_last_subvector_s4[FU_BFADDER]=last_subvector[b3];
            ctrl4_bfadder_en=ctrl3_bfadder_en[b3];
            banksel_s4[((FU_BFADDER-0)*((2-1)-(0)+1)+2-1) : ((FU_BFADDER-0)*((2-1)-(0)+1)+0)]=b3;
            vs_s4[((FU_BFADDER-0)*((32-1)-(0)+1)+32-1) : ((FU_BFADDER-0)*((32-1)-(0)+1)+0)]=vs_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            vc_s4[((FU_BFADDER-0)*((32-1)-(0)+1)+32-1) : ((FU_BFADDER-0)*((32-1)-(0)+1)+0)]=vc_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            dst_s4[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+0)]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b3-0)*((REGIDWIDTH-1)-(0)+1)+0)];
            dst_we_s4[FU_BFADDER]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_BFADDER-0)*((8-1)-(0)+1)+8-1) : ((FU_BFADDER-0)*((8-1)-(0)+1)+0)]=lane_en[((b3-0)*((8-1)-(0)+1)+8-1) : ((b3-0)*((8-1)-(0)+1)+0)];
            src1scalar_s4[FU_BFADDER]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFADDER]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFADDER]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_bfmult_en[b3])    //is bfloat addition
          begin
            d_instr_s4[((FU_BFMULT-0)*((7)-(0)+1)+7) : ((FU_BFMULT-0)*((7)-(0)+1)+0)]=d_instr_s3[((b3-0)*((7)-(0)+1)+7) : ((b3-0)*((7)-(0)+1)+0)];
            D_last_subvector_s4[FU_BFMULT]=last_subvector[b3];
            ctrl4_bfmult_en = ctrl3_bfmult_en[b3];
            banksel_s4[((FU_BFMULT-0)*((2-1)-(0)+1)+2-1) : ((FU_BFMULT-0)*((2-1)-(0)+1)+0)]=b3;
            vs_s4[((FU_BFMULT-0)*((32-1)-(0)+1)+32-1) : ((FU_BFMULT-0)*((32-1)-(0)+1)+0)]=vs_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            vc_s4[((FU_BFMULT-0)*((32-1)-(0)+1)+32-1) : ((FU_BFMULT-0)*((32-1)-(0)+1)+0)]=vc_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            dst_s4[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+0)]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b3-0)*((REGIDWIDTH-1)-(0)+1)+0)];
            dst_we_s4[FU_BFMULT]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_BFMULT-0)*((8-1)-(0)+1)+8-1) : ((FU_BFMULT-0)*((8-1)-(0)+1)+0)]=lane_en[((b3-0)*((8-1)-(0)+1)+8-1) : ((b3-0)*((8-1)-(0)+1)+0)];
            src1scalar_s4[FU_BFMULT]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFMULT]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFMULT]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_trp_en[b3])    //is bfloat addition
          begin
            d_instr_s4[((FU_BFMULT-0)*((7)-(0)+1)+7) : ((FU_BFMULT-0)*((7)-(0)+1)+0)]=d_instr_s3[((b3-0)*((7)-(0)+1)+7) : ((b3-0)*((7)-(0)+1)+0)];
            D_last_subvector_s4[FU_BFMULT]=last_subvector[b3];
            ctrl4_act_en = ctrl3_act_en[b3];
            banksel_s4[((FU_BFMULT-0)*((2-1)-(0)+1)+2-1) : ((FU_BFMULT-0)*((2-1)-(0)+1)+0)]=b3;
            vs_s4[((FU_BFMULT-0)*((32-1)-(0)+1)+32-1) : ((FU_BFMULT-0)*((32-1)-(0)+1)+0)]=vs_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            vc_s4[((FU_BFMULT-0)*((32-1)-(0)+1)+32-1) : ((FU_BFMULT-0)*((32-1)-(0)+1)+0)]=vc_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            dst_s4[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+0)]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b3-0)*((REGIDWIDTH-1)-(0)+1)+0)];
            dst_we_s4[FU_BFMULT]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_BFMULT-0)*((8-1)-(0)+1)+8-1) : ((FU_BFMULT-0)*((8-1)-(0)+1)+0)]=lane_en[((b3-0)*((8-1)-(0)+1)+8-1) : ((b3-0)*((8-1)-(0)+1)+0)];
            src1scalar_s4[FU_BFMULT]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFMULT]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFMULT]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_act_en[b3])    //is bfloat addition
          begin
            d_instr_s4[((FU_TRP-0)*((7)-(0)+1)+7) : ((FU_TRP-0)*((7)-(0)+1)+0)]=d_instr_s3[((b3-0)*((7)-(0)+1)+7) : ((b3-0)*((7)-(0)+1)+0)];
            D_last_subvector_s4[FU_TRP]=last_subvector[b3];
            ctrl4_act_en = ctrl3_act_en[b3];
            banksel_s4[((FU_TRP-0)*((2-1)-(0)+1)+2-1) : ((FU_TRP-0)*((2-1)-(0)+1)+0)]=b3;
            vs_s4[((FU_TRP-0)*((32-1)-(0)+1)+32-1) : ((FU_TRP-0)*((32-1)-(0)+1)+0)]=vs_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            vc_s4[((FU_TRP-0)*((32-1)-(0)+1)+32-1) : ((FU_TRP-0)*((32-1)-(0)+1)+0)]=vc_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            dst_s4[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+0)]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b3-0)*((REGIDWIDTH-1)-(0)+1)+0)];
            dst_we_s4[FU_TRP]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_TRP-0)*((8-1)-(0)+1)+8-1) : ((FU_TRP-0)*((8-1)-(0)+1)+0)]=lane_en[((b3-0)*((8-1)-(0)+1)+8-1) : ((b3-0)*((8-1)-(0)+1)+0)];
            src1scalar_s4[FU_TRP]=src1scalar_s3[b3];
            src2scalar_s4[FU_TRP]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_TRP]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_memunit_en[b3] && !ctrl3_mem_en[b3]) //is memunit shift
          begin
            d_instr_s4[((FU_MEM-0)*((7)-(0)+1)+7) : ((FU_MEM-0)*((7)-(0)+1)+0)]=d_instr_s3[((b3-0)*((7)-(0)+1)+7) : ((b3-0)*((7)-(0)+1)+0)];
            D_last_subvector_s4[FU_MEM]=last_subvector[b3];
            ctrl4_memunit_en=ctrl3_memunit_en[b3];
            banksel_s4[((FU_MEM-0)*((2-1)-(0)+1)+2-1) : ((FU_MEM-0)*((2-1)-(0)+1)+0)]=b3;
            vs_s4[((FU_MEM-0)*((32-1)-(0)+1)+32-1) : ((FU_MEM-0)*((32-1)-(0)+1)+0)]=vs_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            dst_s4[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+0)]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b3-0)*((REGIDWIDTH-1)-(0)+1)+0)];
            dst_we_s4[FU_MEM]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_MEM-0)*((8-1)-(0)+1)+8-1) : ((FU_MEM-0)*((8-1)-(0)+1)+0)]=lane_en[((b3-0)*((8-1)-(0)+1)+8-1) : ((b3-0)*((8-1)-(0)+1)+0)];
            src1scalar_s4[FU_MEM]=src1scalar_s3[b3];
            src2scalar_s4[FU_MEM]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MEM]=ctrl3_ismasked[b3];
            ctrl4_volatiledest=ctrl3_volatiledest[b3];
            ctrl4_mem_dir_left=ctrl3_mem_dir_left[b3];
            ctrl4_memunit_op=ctrl3_memunit_op[((b3-0)*((6)-(0)+1)+6) : ((b3-0)*((6)-(0)+1)+0)];
            mem_last_subvector_s4=last_subvector[b3];
          end
          else if (ctrl3_memunit_en[b3] &&  ctrl3_mem_en[b3]) //is mem operation
          begin
            d_instr_s4[((FU_MEM-0)*((7)-(0)+1)+7) : ((FU_MEM-0)*((7)-(0)+1)+0)]=d_instr_s3[((b3-0)*((7)-(0)+1)+7) : ((b3-0)*((7)-(0)+1)+0)];
            D_last_subvector_s4[FU_MEM]=last_subvector[b3];
            ctrl4_memunit_en=ctrl3_memunit_en[b3];
            banksel_s4[((FU_MEM-0)*((2-1)-(0)+1)+2-1) : ((FU_MEM-0)*((2-1)-(0)+1)+0)]=b3;
            // Use vs to store current length for prefetching
            vs_s4[((FU_MEM-0)*((32-1)-(0)+1)+32-1) : ((FU_MEM-0)*((32-1)-(0)+1)+0)]=count[b3*(6-3+1)+:(6-3)]<<3;
            dst_s4[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+0)]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b3-0)*((REGIDWIDTH-1)-(0)+1)+0)];
            dst_we_s4[FU_MEM]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_MEM-0)*((8-1)-(0)+1)+8-1) : ((FU_MEM-0)*((8-1)-(0)+1)+0)]=lane_en[((b3-0)*((8-1)-(0)+1)+8-1) : ((b3-0)*((8-1)-(0)+1)+0)];
            src1scalar_s4[FU_MEM]=src1scalar_s3[b3];
            src2scalar_s4[FU_MEM]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MEM]=ctrl3_ismasked[b3];
            ctrl4_volatiledest=ctrl3_volatiledest[b3];
            ctrl4_mem_dir_left=ctrl3_mem_dir_left[b3];
            ctrl4_memunit_op=ctrl3_memunit_op[((b3-0)*((6)-(0)+1)+6) : ((b3-0)*((6)-(0)+1)+0)];
            //Load base on first subvector or if INDEXED memory operation
//            vbase_s4=(|(first_subvector&ctrl3_mem_en) || ctrl_memunit_op[3][5])? vbase[3] : vbase_s4 + ((((ctrl_memunit_op[3][4])? vstride[3]: 1)<<ctrl_memunit_op[3][3:2])<<3);
//            // Partial Address Gen for each lane - just do multiplication part
//            for (m=0; m<8; m=m+1)
//              vstrideoffset_s4[m*32 +: 32] = ((ctrl_memunit_op[3][4]) ? vstride[3] : 1)*m;
            mem_last_subvector_s4=last_subvector[b3];
          end
          else if (ctrl3_alu_op[((b3-0)*((10)-(0)+1)+10) : ((b3-0)*((10)-(0)+1)+0)]!=(ALUOP_ZERO^ALUOP_ZERO)) //is ALU
          begin
            d_instr_s4[FU_ALU+b3*0]=d_instr_s3[((b3-0)*((7)-(0)+1)+7) : ((b3-0)*((7)-(0)+1)+0)];
            D_last_subvector_s4[FU_ALU+b3*0]=last_subvector[b3];
            banksel_s4[FU_ALU+b3*0]=b3;
            vs_s4[FU_ALU+b3*0]=vs_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            dst_we_s4[FU_ALU+b3*0]=ctrl3_vr_c_we[b3];
            dst_s4[FU_ALU+b3*0]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b3-0)*((REGIDWIDTH-1)-(0)+1)+0)];
            dst_we_s4[FU_FALU+b3*0]=ctrl3_vf_c_we[b3];
            dst_s4[FU_FALU+b3*0]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b3-0)*((REGIDWIDTH-1)-(0)+1)+0)];
            vlane_en[FU_ALU+b3*0]=lane_en[((b3-0)*((8-1)-(0)+1)+8-1) : ((b3-0)*((8-1)-(0)+1)+0)];
            src1scalar_s4[FU_ALU+b3*0]=src1scalar_s3[b3];
            src2scalar_s4[FU_ALU+b3*0]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_ALU+b3*0]=ctrl3_ismasked[b3];
            ctrl4_alu_op[((b3*0-0)*((10)-(0)+1)+10) : ((b3*0-0)*((10)-(0)+1)+0)]=ctrl3_alu_op[((b3-0)*((10)-(0)+1)+10) : ((b3-0)*((10)-(0)+1)+0)];
            ctrl4_satsum_op[((b3*0-0)*((1)-(0)+1)+1) : ((b3*0-0)*((1)-(0)+1)+0)]=ctrl3_satsum_op[((b3-0)*((1)-(0)+1)+1) : ((b3-0)*((1)-(0)+1)+0)];
            ctrl4_satsize_op[((b3*0-0)*((3)-(0)+1)+3) : ((b3*0-0)*((3)-(0)+1)+0)]=ctrl3_satsize_op[((b3-0)*((3)-(0)+1)+3) : ((b3-0)*((3)-(0)+1)+0)];
            ctrl4_vf_wbsel[b3*0]=ctrl3_vf_wbsel[b3];
          end
          else if (ctrl3_vf_c_we[b3])
          begin                                    //is FALU
            d_instr_s4[FU_FALU+b3*0]=d_instr_s3[((b3-0)*((7)-(0)+1)+7) : ((b3-0)*((7)-(0)+1)+0)];
            D_last_subvector_s4[FU_FALU+b3*0]=last_subvector[b3];
            banksel_s4[FU_FALU+b3*0]=b3;
            vs_s4[FU_FALU+b3*0]=|vs_s3[((b3-0)*((32-1)-(0)+1)+32-1) : ((b3-0)*((32-1)-(0)+1)+0)];
            dst_we_s4[FU_FALU+b3*0]=ctrl3_vf_c_we[b3];
            dst_s4[FU_FALU+b3*0]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b3-0)*((REGIDWIDTH-1)-(0)+1)+0)];
            vlane_en[FU_FALU+b3*0]=lane_en[((b3-0)*((8-1)-(0)+1)+8-1) : ((b3-0)*((8-1)-(0)+1)+0)];
            src1scalar_s4[FU_FALU+b3*0]=src1scalar_s3[b3];
            src2scalar_s4[FU_FALU+b3*0]=src2scalar_s3[b3];
            ctrl4_flagalu_op[((b3*0-0)*((2)-(0)+1)+2) : ((b3*0-0)*((2)-(0)+1)+0)]=ctrl3_flagalu_op[((b3-0)*((2)-(0)+1)+2) : ((b3-0)*((2)-(0)+1)+0)];
            ctrl4_vf_wbsel[b3*0]=ctrl3_vf_wbsel[b3];
            ctrl4_vf_a_sel[b3*0]=ctrl3_vf_a_sel[b3];
          end
        end
      end

// module instance
  vregfile_vector_2_1_128_256_8 vregfile_vector (
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
  vregfile_flag_2_1_8_256_8 vregfile_flag (
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

  //Convert register file width (8*2) to datpath width (16)
  always@*
    for (bn=0; bn<2; bn=bn+1)
      for (n=0; n<8; n=n+1)
      begin
        vr_a_readdataout[((bn-0)*((16*8-1)-(0)+1)+16*n ) +: ( 16)] =          _vr_a_readdataout[bn*8*2*8 + 8*2*n +: 8*2];
        vr_b_readdataout[((bn-0)*((16*8-1)-(0)+1)+16*n ) +: ( 16)] =           _vr_b_readdataout[bn*8*2*8 + 8*2*n +: 8*2];
        _vr_c_writedatain[bn*8*2*8 + 8*2*n +: 8*2] =      vr_c_writedatain[((bn-0)*((16*8-1)-(0)+1)+16*n ) +: ( 16)];
      end

  //Bank Multiplex for each functional unit
 always@*
 begin
   for (fn=0; fn<=FU_MUL; fn=fn+1)
   begin
     vr_src1[fn] =(src1scalar_s4[fn]) ? {8{vs_s4[((fn-0)*((32-1)-(0)+1)+16-1) : ((fn-0)*((32-1)-(0)+1)+0)]}} :(banksel_s4[fn]? vr_a_readdataout[1]: vr_a_readdataout[0]);

     vr_src2[fn] =(src2scalar_s4[fn]) ? {8{vs_s4[((fn-0)*((32-1)-(0)+1)+16-1) : ((fn-0)*((32-1)-(0)+1)+0)]}} :(banksel_s4[fn]? vr_b_readdataout[1]: vr_b_readdataout[0]);

     vf_src1[((fn-0)*((8-1)-(0)+1)+8-1) : ((fn-0)*((8-1)-(0)+1)+0)] = banksel_s4[((fn-0)*((2-1)-(0)+1)+2-1) : ((fn-0)*((2-1)-(0)+1)+0)] ? vf_a_readdataout[1*8 +: 8] : vf_a_readdataout[0*8 +: 8] ;

     vmask[((fn-0)*((8-1)-(0)+1)+8-1) : ((fn-0)*((8-1)-(0)+1)+0)] =vlane_en[((fn-0)*((8-1)-(0)+1)+8-1) : ((fn-0)*((8-1)-(0)+1)+0)] & ((ctrl4_ismasked[fn]) ? ( banksel_s4[((fn-0)*((2-1)-(0)+1)+2-1) : ((fn-0)*((2-1)-(0)+1)+0)]? vf_a_readdataout[1*8 +: 8]: vf_a_readdataout[0*8 +: 8] ) : {8{1'b1}});
   end

   vr_src1[((FU_MEM-0)*((16*8-1)-(0)+1)+16*8-1) : ((FU_MEM-0)*((16*8-1)-(0)+1)+0)] = banksel_s4[((FU_MEM-0)*((2-1)-(0)+1)+2-1) : ((FU_MEM-0)*((2-1)-(0)+1)+0)]? vr_a_readdataout[((1-0)*((16*8-1)-(0)+1)+16*8-1) : ((1-0)*((16*8-1)-(0)+1)+0)]: vr_a_readdataout[((0-0)*((16*8-1)-(0)+1)+16*8-1) : ((0-0)*((16*8-1)-(0)+1)+0)] ;

   vr_src2[((FU_MEM-0)*((16*8-1)-(0)+1)+16*8-1) : ((FU_MEM-0)*((16*8-1)-(0)+1)+0)] = banksel_s4[((FU_MEM-0)*((2-1)-(0)+1)+2-1) : ((FU_MEM-0)*((2-1)-(0)+1)+0)]? vr_b_readdataout[((1-0)*((16*8-1)-(0)+1)+16*8-1) : ((1-0)*((16*8-1)-(0)+1)+0)]: vr_b_readdataout[((0-0)*((16*8-1)-(0)+1)+16*8-1) : ((0-0)*((16*8-1)-(0)+1)+0)];

   vmask[((FU_MEM-0)*((8-1)-(0)+1)+8-1) : ((FU_MEM-0)*((8-1)-(0)+1)+0)] =  vlane_en[((FU_MEM-0)*((8-1)-(0)+1)+8-1) : ((FU_MEM-0)*((8-1)-(0)+1)+0)] & ((ctrl4_ismasked[FU_MEM]) ?(banksel_s4[((FU_MEM-0)*((2-1)-(0)+1)+2-1) : ((FU_MEM-0)*((2-1)-(0)+1)+0)] ?  vf_a_readdataout[1*8 +: 8] :  vf_a_readdataout[0*8 +: 8] ) : {8{1'b1}}) ;

   vr_src1[FU_MATMUL] =(src1scalar_s4[FU_MATMUL]) ? {8{vs_s4[((FU_MATMUL-0)*((32-1)-(0)+1)+16-1) : ((FU_MATMUL-0)*((32-1)-(0)+1)+0)]}} :(banksel_s4[FU_MATMUL]? vr_a_readdataout[1]: vr_a_readdataout[1]);

   vr_src2[FU_MATMUL] =(src2scalar_s4[FU_MATMUL]) ? {8{vs_s4[((FU_MATMUL-0)*((32-1)-(0)+1)+16-1) : ((FU_MATMUL-0)*((32-1)-(0)+1)+0)]}} :(banksel_s4[FU_MATMUL]? vr_b_readdataout[1]: vr_b_readdataout[1]);

   vmask[((FU_MATMUL-0)*((8-1)-(0)+1)+8-1) : ((FU_MATMUL-0)*((8-1)-(0)+1)+0)] =  vlane_en[((FU_MATMUL-0)*((8-1)-(0)+1)+8-1) : ((FU_MATMUL-0)*((8-1)-(0)+1)+0)] & ((ctrl4_ismasked[FU_MATMUL]) ? (banksel_s4[((FU_MATMUL-0)*((2-1)-(0)+1)+2-1) : ((FU_MATMUL-0)*((2-1)-(0)+1)+0)]? vf_a_readdataout[1*8 +: 8] : vf_a_readdataout[0*8 +: 8] ) : {8{1'b1}});

   for (fn2=FU_FALU; fn2<=FU_FALU+(2-1)*0; fn2=fn2+1)
   begin
     vf_src1[fn2] =(src1scalar_s4[fn2]&ctrl4_vf_a_sel[fn2-FU_FALU]) ? {8{vs_s4[((fn2-0)*((32-1)-(0)+1)+0) : ((fn2-0)*((32-1)-(0)+1)+0)]}} : (banksel_s4[fn2]? vf_a_readdataout[1*8 +: 8] : vf_a_readdataout[0*8 +: 8]);

   //Only FALU uses this
     vf_src2[fn2] = (src2scalar_s4[fn2]) ? {8{vs_s4[((fn2-0)*((32-1)-(0)+1)+0) : ((fn2-0)*((32-1)-(0)+1)+0)]}} : (banksel_s4[fn2]? vf_b_readdataout[1*8 +: 8]:vf_b_readdataout[0*8 +: 8]);
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
  assign stall_memunit=_stall_memunit && (dst_we[((FU_MEM-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_MEM-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)] ||(ctrl4_memunit_en && ~stall_mulunit && ~stall_matmul));

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

  wire vmem_unit_out_dst_we;
  assign dst_we[((FU_MEM-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_MEM-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)] = vmem_unit_out_dst_we;

  vmem_unit_16_8_3_8_3_32_128_7_128_7_3_8 vmem_unit(
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
    .cstride(vstride[((4-2)*((32-1)-(0)+1)+32-1) : ((4-2)*((32-1)-(0)+1)+0)]),
    .cprefetch(vs_s4[((FU_MEM-0)*((32-1)-(0)+1)+32-1) : ((FU_MEM-0)*((32-1)-(0)+1)+0)]),
    // Vector ports
    .vmask(vmask[((FU_MEM-0)*((8-1)-(0)+1)+8-1) : ((FU_MEM-0)*((8-1)-(0)+1)+0)]),
    .vstrideoffset(vstrideoffset_s4),
    .vindex(vr_src1[((FU_MEM-0)*((16*8-1)-(0)+1)+16*8-1) : ((FU_MEM-0)*((16*8-1)-(0)+1)+0)]),
    .vwritedata(vr_src2[((FU_MEM-0)*((16*8-1)-(0)+1)+16*8-1) : ((FU_MEM-0)*((16*8-1)-(0)+1)+0)]),
    .voutput(load_result_s5),
    .voutput_we(load_result_mask_s5),
    // Writeback ports
    .in_dst(dst_s4[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+0)]),
    .in_dst_we(dst_we_s4[FU_MEM]),
    .out_dst(dst[5*(NUMFUS*REGIDWIDTH)+ FU_MEM * REGIDWIDTH +: REGIDWIDTH]),
    .out_dst_we(vmem_unit_out_dst_we),
    .in_vs_dst_we(ctrl_vs_we[4]),
    .out_vs_dst_we(ctrl_vs_we[5]),
    // Vector operations ports
    .sa(vstride[((4-2)*((32-1)-(0)+1)+32-1) : ((4-2)*((32-1)-(0)+1)+0)]),
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


  assign dst[4*(NUMFUS*REGIDWIDTH)+FU_MEM * REGIDWIDTH +: REGIDWIDTH] =dst_s4[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+0)];
  assign dst_we[((FU_MEM-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4) : ((FU_MEM-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4)]=dst_we_s4[FU_MEM];

  //============== Multiplier Unit (spans stages 4-6) =============

//module instance 

  wire [2:0] vmul_unit_out_dst_we;
  assign dst_we[((FU_MUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+6) : ((FU_MUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4)] = vmul_unit_out_dst_we;

  wire [3*REGIDWIDTH-1:0] temp_vmul_dst;
  wire [3*8-1:0] temp_vmul_mask;

   vmul_unit_4_8_3_8 vmul_unit(
     .clk(clk),
     .resetn(resetn),
     .op(ctrl4_mulshift_op),
     .activate(ctrl4_mulshift_en),
     .en(pipe_advance[6:4]),
     .squash(pipe_squash[6:4]),
     .stall(stall_mulunit),
     .opA(vr_src1[((FU_MUL-0)*((16*8-1)-(0)+1)+16*8-1) : ((FU_MUL-0)*((16*8-1)-(0)+1)+0)]),
     .opB(vr_src2[((FU_MUL-0)*((16*8-1)-(0)+1)+16*8-1) : ((FU_MUL-0)*((16*8-1)-(0)+1)+0)]),
     .vshamt( (ctrl4_rshiftnonzero) ? vc_s4[((FU_MUL-0)*((32-1)-(0)+1)+32-1) : ((FU_MUL-0)*((32-1)-(0)+1)+0)] : 0 ),
     .vmask(vmask[((FU_MUL-0)*((8-1)-(0)+1)+8-1) : ((FU_MUL-0)*((8-1)-(0)+1)+0)]),
     .in_dst(dst_s4[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+0)]),
     .in_dst_we(dst_we_s4[FU_MUL]),
     .out_dst(temp_vmul_dst),
     .out_dst_we(vmul_unit_out_dst_we),
     .out_dst_mask(temp_vmul_mask),
     .result(mulshift_result_s5)
   );
    assign dst[6*(NUMFUS*REGIDWIDTH) + FU_MUL*REGIDWIDTH +: REGIDWIDTH] = temp_vmul_dst[3*REGIDWIDTH-1:2*REGIDWIDTH] ;
    assign dst[5*(NUMFUS*REGIDWIDTH) + FU_MUL*REGIDWIDTH +: REGIDWIDTH] = temp_vmul_dst[2*REGIDWIDTH-1:1*REGIDWIDTH] ;
    assign dst[4*(NUMFUS*REGIDWIDTH) + FU_MUL*REGIDWIDTH +: REGIDWIDTH] = temp_vmul_dst[1*REGIDWIDTH-1:0*REGIDWIDTH] ;
    assign dst_mask[6*(NUMFUS*8)+FU_MUL*8 +: 8] = temp_vmul_mask[3*8-1:2*8];
    assign dst_mask[5*(NUMFUS*8)+FU_MUL*8 +: 8] = temp_vmul_mask[2*8-1:1*8];
    assign dst_mask[4*(NUMFUS*8)+FU_MUL*8 +: 8] = temp_vmul_mask[1*8-1:0*8];
  //============== ALU Unit =============

  //If APB value is true, create one ALU per bank (per lane)

  wire squash_aludstpipe_NC;
wire squash_faludstpipe_NC;
wire squash_aludstmaskpipe_NC;

 
 //module instance 
     wire valu_0_0_mask;
     wire valu_0_0_cmp_result;
 
     assign valu_0_0_mask = vf_src1[((FU_ALU+0-0)*((8-1)-(0)+1)+0+1) : ((FU_ALU+0-0)*((8-1)-(0)+1)+0)];
     assign alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+0+1) : ((0-0)*((8-1)-(0)+1)+0)] = valu_0_0_cmp_result;
 
     vlane_alu_16 valu_0_0(
         .clk(clk), 
         .resetn(resetn),
         .pipe_en(pipe_advance[4]),
         .pipe_squashn(~pipe_squash[4]),
         .src1(vr_src1[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*0 ) +: ( 16)]),
         .src2(vr_src2[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*0 ) +: ( 16)]),
         .mask(valu_0_0_mask),  //Note: MERGE is not masked
         .op(ctrl4_alu_op[((0-0)*((10)-(0)+1)+10) : ((0-0)*((10)-(0)+1)+0)]^ALUOP_ZERO),
         .satsum_op(ctrl4_satsum_op[((0-0)*((1)-(0)+1)+1) : ((0-0)*((1)-(0)+1)+0)]),
         .satsize_op(ctrl4_satsize_op[((0-0)*((3)-(0)+1)+3) : ((0-0)*((3)-(0)+1)+0)]),
         .cmp_result(valu_0_0_cmp_result),
         .result(alu_result_s5[((0-0)*((16*8-1)-(0)+1)+16*0 ) +: ( 16)])
         );
 //module instance 
 
     wire vflagalu_0_0_src1;
     wire vflagalu_0_0_src2;
     wire vflagalu_0_0_result;
 
     assign vflagalu_0_0_src1 = vf_src1[((FU_FALU+0-0)*((8-1)-(0)+1)+0) : ((FU_FALU+0-0)*((8-1)-(0)+1)+0)];
     assign vflagalu_0_0_src2 = vf_src2[((FU_FALU+0-0)*((8-1)-(0)+1)+0) : ((FU_FALU+0-0)*((8-1)-(0)+1)+0)];
     assign flagalu_result_s4[((0-0)*((8-1)-(0)+1)+0) : ((0-0)*((8-1)-(0)+1)+0)] = vflagalu_0_0_result;
 
       vlane_flagalu vflagalu_0_0(
         .clk(clk), 
         .resetn(resetn),
         .src1(vflagalu_0_0_src1),
         .src2(vflagalu_0_0_src2),
         .op(ctrl4_flagalu_op[((0-0)*((2)-(0)+1)+2) : ((0-0)*((2)-(0)+1)+0)]),
         .result(vflagalu_0_0_result)
         );
 
   wire flagaluresultreg_0_0_q;
   assign flagalu_result_s5[((0-0)*((8-1)-(0)+1)+0) : ((0-0)*((8-1)-(0)+1)+0)] = flagaluresultreg_0_0_q;
 //module instance 
       pipereg_1 flagaluresultreg_0_0 (
         .d( (ctrl4_vf_wbsel[0]) ? alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+0) : ((0-0)*((8-1)-(0)+1)+0)] : flagalu_result_s4[((0-0)*((8-1)-(0)+1)+0) : ((0-0)*((8-1)-(0)+1)+0)]),
         .clk(clk),
         .resetn(resetn),
         .en( pipe_advance[4] ), //Advance unless next stage stalled
         .squashn( 1'b1 ),
         .q(flagaluresultreg_0_0_q)
         );

 
 //module instance 
     wire valu_1_0_mask;
     wire valu_1_0_cmp_result;
 
     assign valu_1_0_mask = vf_src1[((FU_ALU+0-0)*((8-1)-(0)+1)+1+1) : ((FU_ALU+0-0)*((8-1)-(0)+1)+1)];
     assign alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+1+1) : ((0-0)*((8-1)-(0)+1)+1)] = valu_1_0_cmp_result;
 
     vlane_alu_16 valu_1_0(
         .clk(clk), 
         .resetn(resetn),
         .pipe_en(pipe_advance[4]),
         .pipe_squashn(~pipe_squash[4]),
         .src1(vr_src1[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*1 ) +: ( 16)]),
         .src2(vr_src2[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*1 ) +: ( 16)]),
         .mask(valu_1_0_mask),  //Note: MERGE is not masked
         .op(ctrl4_alu_op[((0-0)*((10)-(0)+1)+10) : ((0-0)*((10)-(0)+1)+0)]^ALUOP_ZERO),
         .satsum_op(ctrl4_satsum_op[((0-0)*((1)-(0)+1)+1) : ((0-0)*((1)-(0)+1)+0)]),
         .satsize_op(ctrl4_satsize_op[((0-0)*((3)-(0)+1)+3) : ((0-0)*((3)-(0)+1)+0)]),
         .cmp_result(valu_1_0_cmp_result),
         .result(alu_result_s5[((0-0)*((16*8-1)-(0)+1)+16*1 ) +: ( 16)])
         );
 //module instance 
 
     wire vflagalu_1_0_src1;
     wire vflagalu_1_0_src2;
     wire vflagalu_1_0_result;
 
     assign vflagalu_1_0_src1 = vf_src1[((FU_FALU+0-0)*((8-1)-(0)+1)+1) : ((FU_FALU+0-0)*((8-1)-(0)+1)+1)];
     assign vflagalu_1_0_src2 = vf_src2[((FU_FALU+0-0)*((8-1)-(0)+1)+1) : ((FU_FALU+0-0)*((8-1)-(0)+1)+1)];
     assign flagalu_result_s4[((0-0)*((8-1)-(0)+1)+1) : ((0-0)*((8-1)-(0)+1)+1)] = vflagalu_1_0_result;
 
       vlane_flagalu vflagalu_1_0(
         .clk(clk), 
         .resetn(resetn),
         .src1(vflagalu_1_0_src1),
         .src2(vflagalu_1_0_src2),
         .op(ctrl4_flagalu_op[((0-0)*((2)-(0)+1)+2) : ((0-0)*((2)-(0)+1)+0)]),
         .result(vflagalu_1_0_result)
         );
 
   wire flagaluresultreg_1_0_q;
   assign flagalu_result_s5[((0-0)*((8-1)-(0)+1)+1) : ((0-0)*((8-1)-(0)+1)+1)] = flagaluresultreg_1_0_q;
 //module instance 
       pipereg_1 flagaluresultreg_1_0 (
         .d( (ctrl4_vf_wbsel[0]) ? alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+1) : ((0-0)*((8-1)-(0)+1)+1)] : flagalu_result_s4[((0-0)*((8-1)-(0)+1)+1) : ((0-0)*((8-1)-(0)+1)+1)]),
         .clk(clk),
         .resetn(resetn),
         .en( pipe_advance[4] ), //Advance unless next stage stalled
         .squashn( 1'b1 ),
         .q(flagaluresultreg_1_0_q)
         );

 
 //module instance 
     wire valu_2_0_mask;
     wire valu_2_0_cmp_result;
 
     assign valu_2_0_mask = vf_src1[((FU_ALU+0-0)*((8-1)-(0)+1)+2+1) : ((FU_ALU+0-0)*((8-1)-(0)+1)+2)];
     assign alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+2+1) : ((0-0)*((8-1)-(0)+1)+2)] = valu_2_0_cmp_result;
 
     vlane_alu_16 valu_2_0(
         .clk(clk), 
         .resetn(resetn),
         .pipe_en(pipe_advance[4]),
         .pipe_squashn(~pipe_squash[4]),
         .src1(vr_src1[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*2 ) +: ( 16)]),
         .src2(vr_src2[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*2 ) +: ( 16)]),
         .mask(valu_2_0_mask),  //Note: MERGE is not masked
         .op(ctrl4_alu_op[((0-0)*((10)-(0)+1)+10) : ((0-0)*((10)-(0)+1)+0)]^ALUOP_ZERO),
         .satsum_op(ctrl4_satsum_op[((0-0)*((1)-(0)+1)+1) : ((0-0)*((1)-(0)+1)+0)]),
         .satsize_op(ctrl4_satsize_op[((0-0)*((3)-(0)+1)+3) : ((0-0)*((3)-(0)+1)+0)]),
         .cmp_result(valu_2_0_cmp_result),
         .result(alu_result_s5[((0-0)*((16*8-1)-(0)+1)+16*2 ) +: ( 16)])
         );
 //module instance 
 
     wire vflagalu_2_0_src1;
     wire vflagalu_2_0_src2;
     wire vflagalu_2_0_result;
 
     assign vflagalu_2_0_src1 = vf_src1[((FU_FALU+0-0)*((8-1)-(0)+1)+2) : ((FU_FALU+0-0)*((8-1)-(0)+1)+2)];
     assign vflagalu_2_0_src2 = vf_src2[((FU_FALU+0-0)*((8-1)-(0)+1)+2) : ((FU_FALU+0-0)*((8-1)-(0)+1)+2)];
     assign flagalu_result_s4[((0-0)*((8-1)-(0)+1)+2) : ((0-0)*((8-1)-(0)+1)+2)] = vflagalu_2_0_result;
 
       vlane_flagalu vflagalu_2_0(
         .clk(clk), 
         .resetn(resetn),
         .src1(vflagalu_2_0_src1),
         .src2(vflagalu_2_0_src2),
         .op(ctrl4_flagalu_op[((0-0)*((2)-(0)+1)+2) : ((0-0)*((2)-(0)+1)+0)]),
         .result(vflagalu_2_0_result)
         );
 
   wire flagaluresultreg_2_0_q;
   assign flagalu_result_s5[((0-0)*((8-1)-(0)+1)+2) : ((0-0)*((8-1)-(0)+1)+2)] = flagaluresultreg_2_0_q;
 //module instance 
       pipereg_1 flagaluresultreg_2_0 (
         .d( (ctrl4_vf_wbsel[0]) ? alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+2) : ((0-0)*((8-1)-(0)+1)+2)] : flagalu_result_s4[((0-0)*((8-1)-(0)+1)+2) : ((0-0)*((8-1)-(0)+1)+2)]),
         .clk(clk),
         .resetn(resetn),
         .en( pipe_advance[4] ), //Advance unless next stage stalled
         .squashn( 1'b1 ),
         .q(flagaluresultreg_2_0_q)
         );

 
 //module instance 
     wire valu_3_0_mask;
     wire valu_3_0_cmp_result;
 
     assign valu_3_0_mask = vf_src1[((FU_ALU+0-0)*((8-1)-(0)+1)+3+1) : ((FU_ALU+0-0)*((8-1)-(0)+1)+3)];
     assign alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+3+1) : ((0-0)*((8-1)-(0)+1)+3)] = valu_3_0_cmp_result;
 
     vlane_alu_16 valu_3_0(
         .clk(clk), 
         .resetn(resetn),
         .pipe_en(pipe_advance[4]),
         .pipe_squashn(~pipe_squash[4]),
         .src1(vr_src1[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*3 ) +: ( 16)]),
         .src2(vr_src2[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*3 ) +: ( 16)]),
         .mask(valu_3_0_mask),  //Note: MERGE is not masked
         .op(ctrl4_alu_op[((0-0)*((10)-(0)+1)+10) : ((0-0)*((10)-(0)+1)+0)]^ALUOP_ZERO),
         .satsum_op(ctrl4_satsum_op[((0-0)*((1)-(0)+1)+1) : ((0-0)*((1)-(0)+1)+0)]),
         .satsize_op(ctrl4_satsize_op[((0-0)*((3)-(0)+1)+3) : ((0-0)*((3)-(0)+1)+0)]),
         .cmp_result(valu_3_0_cmp_result),
         .result(alu_result_s5[((0-0)*((16*8-1)-(0)+1)+16*3 ) +: ( 16)])
         );
 //module instance 
 
     wire vflagalu_3_0_src1;
     wire vflagalu_3_0_src2;
     wire vflagalu_3_0_result;
 
     assign vflagalu_3_0_src1 = vf_src1[((FU_FALU+0-0)*((8-1)-(0)+1)+3) : ((FU_FALU+0-0)*((8-1)-(0)+1)+3)];
     assign vflagalu_3_0_src2 = vf_src2[((FU_FALU+0-0)*((8-1)-(0)+1)+3) : ((FU_FALU+0-0)*((8-1)-(0)+1)+3)];
     assign flagalu_result_s4[((0-0)*((8-1)-(0)+1)+3) : ((0-0)*((8-1)-(0)+1)+3)] = vflagalu_3_0_result;
 
       vlane_flagalu vflagalu_3_0(
         .clk(clk), 
         .resetn(resetn),
         .src1(vflagalu_3_0_src1),
         .src2(vflagalu_3_0_src2),
         .op(ctrl4_flagalu_op[((0-0)*((2)-(0)+1)+2) : ((0-0)*((2)-(0)+1)+0)]),
         .result(vflagalu_3_0_result)
         );
 
   wire flagaluresultreg_3_0_q;
   assign flagalu_result_s5[((0-0)*((8-1)-(0)+1)+3) : ((0-0)*((8-1)-(0)+1)+3)] = flagaluresultreg_3_0_q;
 //module instance 
       pipereg_1 flagaluresultreg_3_0 (
         .d( (ctrl4_vf_wbsel[0]) ? alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+3) : ((0-0)*((8-1)-(0)+1)+3)] : flagalu_result_s4[((0-0)*((8-1)-(0)+1)+3) : ((0-0)*((8-1)-(0)+1)+3)]),
         .clk(clk),
         .resetn(resetn),
         .en( pipe_advance[4] ), //Advance unless next stage stalled
         .squashn( 1'b1 ),
         .q(flagaluresultreg_3_0_q)
         );

 
 //module instance 
     wire valu_4_0_mask;
     wire valu_4_0_cmp_result;
 
     assign valu_4_0_mask = vf_src1[((FU_ALU+0-0)*((8-1)-(0)+1)+4+1) : ((FU_ALU+0-0)*((8-1)-(0)+1)+4)];
     assign alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+4+1) : ((0-0)*((8-1)-(0)+1)+4)] = valu_4_0_cmp_result;
 
     vlane_alu_16 valu_4_0(
         .clk(clk), 
         .resetn(resetn),
         .pipe_en(pipe_advance[4]),
         .pipe_squashn(~pipe_squash[4]),
         .src1(vr_src1[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*4 ) +: ( 16)]),
         .src2(vr_src2[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*4 ) +: ( 16)]),
         .mask(valu_4_0_mask),  //Note: MERGE is not masked
         .op(ctrl4_alu_op[((0-0)*((10)-(0)+1)+10) : ((0-0)*((10)-(0)+1)+0)]^ALUOP_ZERO),
         .satsum_op(ctrl4_satsum_op[((0-0)*((1)-(0)+1)+1) : ((0-0)*((1)-(0)+1)+0)]),
         .satsize_op(ctrl4_satsize_op[((0-0)*((3)-(0)+1)+3) : ((0-0)*((3)-(0)+1)+0)]),
         .cmp_result(valu_4_0_cmp_result),
         .result(alu_result_s5[((0-0)*((16*8-1)-(0)+1)+16*4 ) +: ( 16)])
         );
 //module instance 
 
     wire vflagalu_4_0_src1;
     wire vflagalu_4_0_src2;
     wire vflagalu_4_0_result;
 
     assign vflagalu_4_0_src1 = vf_src1[((FU_FALU+0-0)*((8-1)-(0)+1)+4) : ((FU_FALU+0-0)*((8-1)-(0)+1)+4)];
     assign vflagalu_4_0_src2 = vf_src2[((FU_FALU+0-0)*((8-1)-(0)+1)+4) : ((FU_FALU+0-0)*((8-1)-(0)+1)+4)];
     assign flagalu_result_s4[((0-0)*((8-1)-(0)+1)+4) : ((0-0)*((8-1)-(0)+1)+4)] = vflagalu_4_0_result;
 
       vlane_flagalu vflagalu_4_0(
         .clk(clk), 
         .resetn(resetn),
         .src1(vflagalu_4_0_src1),
         .src2(vflagalu_4_0_src2),
         .op(ctrl4_flagalu_op[((0-0)*((2)-(0)+1)+2) : ((0-0)*((2)-(0)+1)+0)]),
         .result(vflagalu_4_0_result)
         );
 
   wire flagaluresultreg_4_0_q;
   assign flagalu_result_s5[((0-0)*((8-1)-(0)+1)+4) : ((0-0)*((8-1)-(0)+1)+4)] = flagaluresultreg_4_0_q;
 //module instance 
       pipereg_1 flagaluresultreg_4_0 (
         .d( (ctrl4_vf_wbsel[0]) ? alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+4) : ((0-0)*((8-1)-(0)+1)+4)] : flagalu_result_s4[((0-0)*((8-1)-(0)+1)+4) : ((0-0)*((8-1)-(0)+1)+4)]),
         .clk(clk),
         .resetn(resetn),
         .en( pipe_advance[4] ), //Advance unless next stage stalled
         .squashn( 1'b1 ),
         .q(flagaluresultreg_4_0_q)
         );

 
 //module instance 
     wire valu_5_0_mask;
     wire valu_5_0_cmp_result;
 
     assign valu_5_0_mask = vf_src1[((FU_ALU+0-0)*((8-1)-(0)+1)+5+1) : ((FU_ALU+0-0)*((8-1)-(0)+1)+5)];
     assign alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+5+1) : ((0-0)*((8-1)-(0)+1)+5)] = valu_5_0_cmp_result;
 
     vlane_alu_16 valu_5_0(
         .clk(clk), 
         .resetn(resetn),
         .pipe_en(pipe_advance[4]),
         .pipe_squashn(~pipe_squash[4]),
         .src1(vr_src1[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*5 ) +: ( 16)]),
         .src2(vr_src2[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*5 ) +: ( 16)]),
         .mask(valu_5_0_mask),  //Note: MERGE is not masked
         .op(ctrl4_alu_op[((0-0)*((10)-(0)+1)+10) : ((0-0)*((10)-(0)+1)+0)]^ALUOP_ZERO),
         .satsum_op(ctrl4_satsum_op[((0-0)*((1)-(0)+1)+1) : ((0-0)*((1)-(0)+1)+0)]),
         .satsize_op(ctrl4_satsize_op[((0-0)*((3)-(0)+1)+3) : ((0-0)*((3)-(0)+1)+0)]),
         .cmp_result(valu_5_0_cmp_result),
         .result(alu_result_s5[((0-0)*((16*8-1)-(0)+1)+16*5 ) +: ( 16)])
         );
 //module instance 
 
     wire vflagalu_5_0_src1;
     wire vflagalu_5_0_src2;
     wire vflagalu_5_0_result;
 
     assign vflagalu_5_0_src1 = vf_src1[((FU_FALU+0-0)*((8-1)-(0)+1)+5) : ((FU_FALU+0-0)*((8-1)-(0)+1)+5)];
     assign vflagalu_5_0_src2 = vf_src2[((FU_FALU+0-0)*((8-1)-(0)+1)+5) : ((FU_FALU+0-0)*((8-1)-(0)+1)+5)];
     assign flagalu_result_s4[((0-0)*((8-1)-(0)+1)+5) : ((0-0)*((8-1)-(0)+1)+5)] = vflagalu_5_0_result;
 
       vlane_flagalu vflagalu_5_0(
         .clk(clk), 
         .resetn(resetn),
         .src1(vflagalu_5_0_src1),
         .src2(vflagalu_5_0_src2),
         .op(ctrl4_flagalu_op[((0-0)*((2)-(0)+1)+2) : ((0-0)*((2)-(0)+1)+0)]),
         .result(vflagalu_5_0_result)
         );
 
   wire flagaluresultreg_5_0_q;
   assign flagalu_result_s5[((0-0)*((8-1)-(0)+1)+5) : ((0-0)*((8-1)-(0)+1)+5)] = flagaluresultreg_5_0_q;
 //module instance 
       pipereg_1 flagaluresultreg_5_0 (
         .d( (ctrl4_vf_wbsel[0]) ? alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+5) : ((0-0)*((8-1)-(0)+1)+5)] : flagalu_result_s4[((0-0)*((8-1)-(0)+1)+5) : ((0-0)*((8-1)-(0)+1)+5)]),
         .clk(clk),
         .resetn(resetn),
         .en( pipe_advance[4] ), //Advance unless next stage stalled
         .squashn( 1'b1 ),
         .q(flagaluresultreg_5_0_q)
         );

 
 //module instance 
     wire valu_6_0_mask;
     wire valu_6_0_cmp_result;
 
     assign valu_6_0_mask = vf_src1[((FU_ALU+0-0)*((8-1)-(0)+1)+6+1) : ((FU_ALU+0-0)*((8-1)-(0)+1)+6)];
     assign alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+6+1) : ((0-0)*((8-1)-(0)+1)+6)] = valu_6_0_cmp_result;
 
     vlane_alu_16 valu_6_0(
         .clk(clk), 
         .resetn(resetn),
         .pipe_en(pipe_advance[4]),
         .pipe_squashn(~pipe_squash[4]),
         .src1(vr_src1[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*6 ) +: ( 16)]),
         .src2(vr_src2[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*6 ) +: ( 16)]),
         .mask(valu_6_0_mask),  //Note: MERGE is not masked
         .op(ctrl4_alu_op[((0-0)*((10)-(0)+1)+10) : ((0-0)*((10)-(0)+1)+0)]^ALUOP_ZERO),
         .satsum_op(ctrl4_satsum_op[((0-0)*((1)-(0)+1)+1) : ((0-0)*((1)-(0)+1)+0)]),
         .satsize_op(ctrl4_satsize_op[((0-0)*((3)-(0)+1)+3) : ((0-0)*((3)-(0)+1)+0)]),
         .cmp_result(valu_6_0_cmp_result),
         .result(alu_result_s5[((0-0)*((16*8-1)-(0)+1)+16*6 ) +: ( 16)])
         );
 //module instance 
 
     wire vflagalu_6_0_src1;
     wire vflagalu_6_0_src2;
     wire vflagalu_6_0_result;
 
     assign vflagalu_6_0_src1 = vf_src1[((FU_FALU+0-0)*((8-1)-(0)+1)+6) : ((FU_FALU+0-0)*((8-1)-(0)+1)+6)];
     assign vflagalu_6_0_src2 = vf_src2[((FU_FALU+0-0)*((8-1)-(0)+1)+6) : ((FU_FALU+0-0)*((8-1)-(0)+1)+6)];
     assign flagalu_result_s4[((0-0)*((8-1)-(0)+1)+6) : ((0-0)*((8-1)-(0)+1)+6)] = vflagalu_6_0_result;
 
       vlane_flagalu vflagalu_6_0(
         .clk(clk), 
         .resetn(resetn),
         .src1(vflagalu_6_0_src1),
         .src2(vflagalu_6_0_src2),
         .op(ctrl4_flagalu_op[((0-0)*((2)-(0)+1)+2) : ((0-0)*((2)-(0)+1)+0)]),
         .result(vflagalu_6_0_result)
         );
 
   wire flagaluresultreg_6_0_q;
   assign flagalu_result_s5[((0-0)*((8-1)-(0)+1)+6) : ((0-0)*((8-1)-(0)+1)+6)] = flagaluresultreg_6_0_q;
 //module instance 
       pipereg_1 flagaluresultreg_6_0 (
         .d( (ctrl4_vf_wbsel[0]) ? alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+6) : ((0-0)*((8-1)-(0)+1)+6)] : flagalu_result_s4[((0-0)*((8-1)-(0)+1)+6) : ((0-0)*((8-1)-(0)+1)+6)]),
         .clk(clk),
         .resetn(resetn),
         .en( pipe_advance[4] ), //Advance unless next stage stalled
         .squashn( 1'b1 ),
         .q(flagaluresultreg_6_0_q)
         );

 
 //module instance 
     wire valu_7_0_mask;
     wire valu_7_0_cmp_result;
 
     assign valu_7_0_mask = vf_src1[((FU_ALU+0-0)*((8-1)-(0)+1)+7+1) : ((FU_ALU+0-0)*((8-1)-(0)+1)+7)];
     assign alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+7+1) : ((0-0)*((8-1)-(0)+1)+7)] = valu_7_0_cmp_result;
 
     vlane_alu_16 valu_7_0(
         .clk(clk), 
         .resetn(resetn),
         .pipe_en(pipe_advance[4]),
         .pipe_squashn(~pipe_squash[4]),
         .src1(vr_src1[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*7 ) +: ( 16)]),
         .src2(vr_src2[((FU_ALU+0-0)*((16*8-1)-(0)+1)+16*7 ) +: ( 16)]),
         .mask(valu_7_0_mask),  //Note: MERGE is not masked
         .op(ctrl4_alu_op[((0-0)*((10)-(0)+1)+10) : ((0-0)*((10)-(0)+1)+0)]^ALUOP_ZERO),
         .satsum_op(ctrl4_satsum_op[((0-0)*((1)-(0)+1)+1) : ((0-0)*((1)-(0)+1)+0)]),
         .satsize_op(ctrl4_satsize_op[((0-0)*((3)-(0)+1)+3) : ((0-0)*((3)-(0)+1)+0)]),
         .cmp_result(valu_7_0_cmp_result),
         .result(alu_result_s5[((0-0)*((16*8-1)-(0)+1)+16*7 ) +: ( 16)])
         );
 //module instance 
 
     wire vflagalu_7_0_src1;
     wire vflagalu_7_0_src2;
     wire vflagalu_7_0_result;
 
     assign vflagalu_7_0_src1 = vf_src1[((FU_FALU+0-0)*((8-1)-(0)+1)+7) : ((FU_FALU+0-0)*((8-1)-(0)+1)+7)];
     assign vflagalu_7_0_src2 = vf_src2[((FU_FALU+0-0)*((8-1)-(0)+1)+7) : ((FU_FALU+0-0)*((8-1)-(0)+1)+7)];
     assign flagalu_result_s4[((0-0)*((8-1)-(0)+1)+7) : ((0-0)*((8-1)-(0)+1)+7)] = vflagalu_7_0_result;
 
       vlane_flagalu vflagalu_7_0(
         .clk(clk), 
         .resetn(resetn),
         .src1(vflagalu_7_0_src1),
         .src2(vflagalu_7_0_src2),
         .op(ctrl4_flagalu_op[((0-0)*((2)-(0)+1)+2) : ((0-0)*((2)-(0)+1)+0)]),
         .result(vflagalu_7_0_result)
         );
 
   wire flagaluresultreg_7_0_q;
   assign flagalu_result_s5[((0-0)*((8-1)-(0)+1)+7) : ((0-0)*((8-1)-(0)+1)+7)] = flagaluresultreg_7_0_q;
 //module instance 
       pipereg_1 flagaluresultreg_7_0 (
         .d( (ctrl4_vf_wbsel[0]) ? alu_cmpresult_s4[((0-0)*((8-1)-(0)+1)+7) : ((0-0)*((8-1)-(0)+1)+7)] : flagalu_result_s4[((0-0)*((8-1)-(0)+1)+7) : ((0-0)*((8-1)-(0)+1)+7)]),
         .clk(clk),
         .resetn(resetn),
         .en( pipe_advance[4] ), //Advance unless next stage stalled
         .squashn( 1'b1 ),
         .q(flagaluresultreg_7_0_q)
         );


 //module instance 
     wire [8*(1+1)-1:0] aludstpipe_0_q;
     assign {dst[5*(NUMFUS*REGIDWIDTH)+(FU_ALU+0)*REGIDWIDTH +: REGIDWIDTH],dst[4*(NUMFUS*REGIDWIDTH)+(FU_ALU+0)*REGIDWIDTH +: REGIDWIDTH]} = aludstpipe_0_q;
     pipe_8_1 aludstpipe_0 (
       .d( dst_s4[FU_ALU+0] ),  
       .clk(clk),
       .resetn(resetn),
       .en( pipe_advance[4] ),
       .squash(squash_aludstpipe_NC),
       .q(aludstpipe_0_q));
 
 //module instance 
 
   wire [1:0] aludstwepipe_0_q;
   assign {dst_we[((FU_ALU+0-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_ALU+0-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)],dst_we[((FU_ALU+0-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4) : ((FU_ALU+0-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4)]} = aludstwepipe_0_q;
 
    pipe_1_3 aludstwepipe_0 (
       .d( dst_we_s4[FU_ALU+0] & ~ctrl4_vf_wbsel[0] ),  
       .clk(clk),
       .resetn(resetn),
       .en( pipe_advance[6:4] ),
       .squash( pipe_squash[6:4] ),
       .q(aludstwepipe_0_q));
 
 wire [8*(1+1)-1:0] aludstmaskpipe_0_q;
 assign {dst_mask[5*(NUMFUS*8)+(FU_ALU+0)*8 +: 8],dst_mask[4*(NUMFUS*8)+(FU_ALU+0)*8 +: 8]} = aludstmaskpipe_0_q;
 
 //module instance 
     pipe_8_1 aludstmaskpipe_0 (
       .d( vmask[FU_ALU+0] ),  
       .clk(clk),
       .resetn(resetn),
       .en( pipe_advance[4] ),
       .squash(squash_aludstmaskpipe_NC),
       .q(aludstmaskpipe_0_q));
 
     wire [8*(1+1)-1:0] faludstpipe_0_q;
     assign {dst[5*(NUMFUS*REGIDWIDTH)+(FU_FALU+0)*REGIDWIDTH +: REGIDWIDTH],dst[5*(NUMFUS*REGIDWIDTH)+(FU_FALU+0)*REGIDWIDTH +: REGIDWIDTH]} = faludstpipe_0_q;
 
 //module instance 
     pipe_8_1 faludstpipe_0 (
       .d( dst_s4[FU_FALU+0] ),  
       .clk(clk),
       .resetn(resetn),
       .en( pipe_advance[4] ),
       .squash(squash_faludstpipe_NC),
       .q(faludstpipe_0_q));
 
   wire [1:0] faludstwepipe_0_q;
   assign {dst_we[((FU_FALU+0-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_FALU+0-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)],dst_we[((FU_FALU+0-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4) : ((FU_FALU+0-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4)]} = faludstwepipe_0_q;
 //module instance 
     pipe_1_3 faludstwepipe_0 (
       .d( dst_we_s4[FU_FALU+0] ),  
       .clk(clk),
       .resetn(resetn),
       .en( pipe_advance[6:4] ),
       .squash( pipe_squash[6:4] ),
       .q(faludstwepipe_0_q));
 


  //This code is just for assigning from signals connected
  //to the matmul, which are linear multi bit signals, to the
  //multi-dimensional signals.
  wire [`MATMUL_STAGES*REGIDWIDTH-1:0] dst_matmul;
  wire [`MATMUL_STAGES*8-1:0] dst_mask_matmul;



      assign dst[(0+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(0+1)-1:REGIDWIDTH*0];
      assign dst_mask[(0+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(0+1)-1:8*0];



      assign dst[(1+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(1+1)-1:REGIDWIDTH*1];
      assign dst_mask[(1+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(1+1)-1:8*1];



      assign dst[(2+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(2+1)-1:REGIDWIDTH*2];
      assign dst_mask[(2+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(2+1)-1:8*2];



      assign dst[(3+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(3+1)-1:REGIDWIDTH*3];
      assign dst_mask[(3+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(3+1)-1:8*3];



      assign dst[(4+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(4+1)-1:REGIDWIDTH*4];
      assign dst_mask[(4+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(4+1)-1:8*4];



      assign dst[(5+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(5+1)-1:REGIDWIDTH*5];
      assign dst_mask[(5+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(5+1)-1:8*5];



      assign dst[(6+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(6+1)-1:REGIDWIDTH*6];
      assign dst_mask[(6+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(6+1)-1:8*6];



      assign dst[(7+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(7+1)-1:REGIDWIDTH*7];
      assign dst_mask[(7+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(7+1)-1:8*7];



      assign dst[(8+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(8+1)-1:REGIDWIDTH*8];
      assign dst_mask[(8+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(8+1)-1:8*8];



      assign dst[(9+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(9+1)-1:REGIDWIDTH*9];
      assign dst_mask[(9+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(9+1)-1:8*9];



      assign dst[(10+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(10+1)-1:REGIDWIDTH*10];
      assign dst_mask[(10+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(10+1)-1:8*10];



      assign dst[(11+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(11+1)-1:REGIDWIDTH*11];
      assign dst_mask[(11+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(11+1)-1:8*11];



      assign dst[(12+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(12+1)-1:REGIDWIDTH*12];
      assign dst_mask[(12+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(12+1)-1:8*12];



      assign dst[(13+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(13+1)-1:REGIDWIDTH*13];
      assign dst_mask[(13+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(13+1)-1:8*13];



      assign dst[(14+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(14+1)-1:REGIDWIDTH*14];
      assign dst_mask[(14+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(14+1)-1:8*14];



      assign dst[(15+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(15+1)-1:REGIDWIDTH*15];
      assign dst_mask[(15+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(15+1)-1:8*15];



      assign dst[(16+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(16+1)-1:REGIDWIDTH*16];
      assign dst_mask[(16+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(16+1)-1:8*16];



      assign dst[(17+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(17+1)-1:REGIDWIDTH*17];
      assign dst_mask[(17+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(17+1)-1:8*17];



      assign dst[(18+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(18+1)-1:REGIDWIDTH*18];
      assign dst_mask[(18+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(18+1)-1:8*18];



      assign dst[(19+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(19+1)-1:REGIDWIDTH*19];
      assign dst_mask[(19+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(19+1)-1:8*19];



      assign dst[(20+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(20+1)-1:REGIDWIDTH*20];
      assign dst_mask[(20+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(20+1)-1:8*20];



      assign dst[(21+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(21+1)-1:REGIDWIDTH*21];
      assign dst_mask[(21+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(21+1)-1:8*21];



      assign dst[(22+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(22+1)-1:REGIDWIDTH*22];
      assign dst_mask[(22+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(22+1)-1:8*22];



      assign dst[(23+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(23+1)-1:REGIDWIDTH*23];
      assign dst_mask[(23+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(23+1)-1:8*23];



      assign dst[(24+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(24+1)-1:REGIDWIDTH*24];
      assign dst_mask[(24+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(24+1)-1:8*24];



      assign dst[(25+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(25+1)-1:REGIDWIDTH*25];
      assign dst_mask[(25+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(25+1)-1:8*25];



      assign dst[(26+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(26+1)-1:REGIDWIDTH*26];
      assign dst_mask[(26+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(26+1)-1:8*26];



      assign dst[(27+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(27+1)-1:REGIDWIDTH*27];
      assign dst_mask[(27+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(27+1)-1:8*27];



      assign dst[(28+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(28+1)-1:REGIDWIDTH*28];
      assign dst_mask[(28+4)*(NUMFUS*8)+(FU_MATMUL)*8 +: 8] = dst_mask_matmul[8*(28+1)-1:8*28];



///////////////////////////
// Matmul unit
///////////////////////////

//module instance 

wire [`MAX_PIPE_STAGES-1:4] temp_matmul_we;
wire [`MAT_MUL_SIZE * `DWIDTH-1:0] matmul_in0,matmul_in1;

assign matmul_in0 = vr_src1[((FU_MATMUL-0)*((16*8-1)-(0)+1)+16*8-1) : ((FU_MATMUL-0)*((16*8-1)-(0)+1)+0)];
assign matmul_in1 = vr_src2[((FU_MATMUL-0)*((16*8-1)-(0)+1)+16*8-1) : ((FU_MATMUL-0)*((16*8-1)-(0)+1)+0)];

 matmul_unit_8_29_8 u_matmul(
 .clk(clk),
 .resetn(resetn),
 .activate(ctrl4_matmul_en),
 .en(pipe_advance[`MAX_PIPE_STAGES-1:4]),
 .squash(pipe_squash[`MAX_PIPE_STAGES-1:4]),
 .stall(stall_matmul),
 .a_data(matmul_in0),
 .b_data(matmul_in1),
 .validity_mask_a_rows(matmul_masks_in[1*`MAT_MUL_SIZE-1:0*`MAT_MUL_SIZE]),
 .validity_mask_a_cols(matmul_masks_in[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE]),
 .validity_mask_b_rows(matmul_masks_in[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE]),
 .validity_mask_b_cols(matmul_masks_in[3*`MAT_MUL_SIZE-1:2*`MAT_MUL_SIZE]),
 .c_data(matmul_out), 
 .vmask(vmask[((FU_MATMUL-0)*((8-1)-(0)+1)+8-1) : ((FU_MATMUL-0)*((8-1)-(0)+1)+0)]),
 .in_dst(dst_s4[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+0)]),
 .in_dst_we(dst_we_s4[FU_MATMUL]),
 .out_dst(dst_matmul),
 .out_dst_we(temp_matmul_we),
 .out_dst_mask(dst_mask_matmul)
 );

assign dst_we[((FU_MATMUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+`MAX_PIPE_STAGES-1) : ((FU_MATMUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+4)] = temp_matmul_we;

//module instance 

trp_unit_16 u_trp (
.clk(clk),
.resetn(resetn),
.en(ctrl4_trp_en),
.a(vr_src1[((FU_TRP-0)*((16*8-1)-(0)+1)+8*16-1) : ((FU_TRP-0)*((16*8-1)-(0)+1)+0)]),
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

    bfloat_adder_16 bf_add_0(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFADDER-0)*((16*8-1)-(0)+1)+0 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFADDER-0)*((16*8-1)-(0)+1)+0 * 16 ) +: ( 16)]),
    .out(bfadder_result_s5[0*16 +: 16])
    );
    
//module instance 
    bfloat_mult_16 bf_mult_0(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFMULT-0)*((16*8-1)-(0)+1)+0 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFMULT-0)*((16*8-1)-(0)+1)+0 * 16 ) +: ( 16)]),
    .out(bfmult_result_s5[0*16 +: 16])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_16 inst_activation_0(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_ACT-0)*((16*8-1)-(0)+1)+0 * 16 ) +: ( 16)]),
    .out(act_result_s5[0*16 +: 16])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_16 bf_add_1(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFADDER-0)*((16*8-1)-(0)+1)+1 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFADDER-0)*((16*8-1)-(0)+1)+1 * 16 ) +: ( 16)]),
    .out(bfadder_result_s5[1*16 +: 16])
    );
    
//module instance 
    bfloat_mult_16 bf_mult_1(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFMULT-0)*((16*8-1)-(0)+1)+1 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFMULT-0)*((16*8-1)-(0)+1)+1 * 16 ) +: ( 16)]),
    .out(bfmult_result_s5[1*16 +: 16])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_16 inst_activation_1(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_ACT-0)*((16*8-1)-(0)+1)+1 * 16 ) +: ( 16)]),
    .out(act_result_s5[1*16 +: 16])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_16 bf_add_2(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFADDER-0)*((16*8-1)-(0)+1)+2 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFADDER-0)*((16*8-1)-(0)+1)+2 * 16 ) +: ( 16)]),
    .out(bfadder_result_s5[2*16 +: 16])
    );
    
//module instance 
    bfloat_mult_16 bf_mult_2(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFMULT-0)*((16*8-1)-(0)+1)+2 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFMULT-0)*((16*8-1)-(0)+1)+2 * 16 ) +: ( 16)]),
    .out(bfmult_result_s5[2*16 +: 16])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_16 inst_activation_2(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_ACT-0)*((16*8-1)-(0)+1)+2 * 16 ) +: ( 16)]),
    .out(act_result_s5[2*16 +: 16])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_16 bf_add_3(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFADDER-0)*((16*8-1)-(0)+1)+3 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFADDER-0)*((16*8-1)-(0)+1)+3 * 16 ) +: ( 16)]),
    .out(bfadder_result_s5[3*16 +: 16])
    );
    
//module instance 
    bfloat_mult_16 bf_mult_3(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFMULT-0)*((16*8-1)-(0)+1)+3 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFMULT-0)*((16*8-1)-(0)+1)+3 * 16 ) +: ( 16)]),
    .out(bfmult_result_s5[3*16 +: 16])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_16 inst_activation_3(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_ACT-0)*((16*8-1)-(0)+1)+3 * 16 ) +: ( 16)]),
    .out(act_result_s5[3*16 +: 16])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_16 bf_add_4(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFADDER-0)*((16*8-1)-(0)+1)+4 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFADDER-0)*((16*8-1)-(0)+1)+4 * 16 ) +: ( 16)]),
    .out(bfadder_result_s5[4*16 +: 16])
    );
    
//module instance 
    bfloat_mult_16 bf_mult_4(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFMULT-0)*((16*8-1)-(0)+1)+4 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFMULT-0)*((16*8-1)-(0)+1)+4 * 16 ) +: ( 16)]),
    .out(bfmult_result_s5[4*16 +: 16])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_16 inst_activation_4(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_ACT-0)*((16*8-1)-(0)+1)+4 * 16 ) +: ( 16)]),
    .out(act_result_s5[4*16 +: 16])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_16 bf_add_5(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFADDER-0)*((16*8-1)-(0)+1)+5 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFADDER-0)*((16*8-1)-(0)+1)+5 * 16 ) +: ( 16)]),
    .out(bfadder_result_s5[5*16 +: 16])
    );
    
//module instance 
    bfloat_mult_16 bf_mult_5(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFMULT-0)*((16*8-1)-(0)+1)+5 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFMULT-0)*((16*8-1)-(0)+1)+5 * 16 ) +: ( 16)]),
    .out(bfmult_result_s5[5*16 +: 16])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_16 inst_activation_5(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_ACT-0)*((16*8-1)-(0)+1)+5 * 16 ) +: ( 16)]),
    .out(act_result_s5[5*16 +: 16])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_16 bf_add_6(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFADDER-0)*((16*8-1)-(0)+1)+6 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFADDER-0)*((16*8-1)-(0)+1)+6 * 16 ) +: ( 16)]),
    .out(bfadder_result_s5[6*16 +: 16])
    );
    
//module instance 
    bfloat_mult_16 bf_mult_6(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFMULT-0)*((16*8-1)-(0)+1)+6 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFMULT-0)*((16*8-1)-(0)+1)+6 * 16 ) +: ( 16)]),
    .out(bfmult_result_s5[6*16 +: 16])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_16 inst_activation_6(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_ACT-0)*((16*8-1)-(0)+1)+6 * 16 ) +: ( 16)]),
    .out(act_result_s5[6*16 +: 16])
    );


 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_16 bf_add_7(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFADDER-0)*((16*8-1)-(0)+1)+7 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFADDER-0)*((16*8-1)-(0)+1)+7 * 16 ) +: ( 16)]),
    .out(bfadder_result_s5[7*16 +: 16])
    );
    
//module instance 
    bfloat_mult_16 bf_mult_7(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_BFMULT-0)*((16*8-1)-(0)+1)+7 * 16 ) +: ( 16)]),
    .b(vr_src2[((FU_BFMULT-0)*((16*8-1)-(0)+1)+7 * 16 ) +: ( 16)]),
    .out(bfmult_result_s5[7*16 +: 16])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_16 inst_activation_7(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_ACT-0)*((16*8-1)-(0)+1)+7 * 16 ) +: ( 16)]),
    .out(act_result_s5[7*16 +: 16])
    );



/******************************************************************************/
/************************** WB Pipeline Stage ********************************/
/******************************************************************************/

    assign wb_dst[FU_ALU+0]=dst[5*(NUMFUS*REGIDWIDTH)+(FU_ALU+0)*REGIDWIDTH +: REGIDWIDTH];
    assign wb_dst_we[FU_ALU+0]=dst_we[((FU_ALU+0-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_ALU+0-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)] && ~pipe_squash[5];
    assign wb_dst_mask[FU_ALU+0]=dst_mask[5*(NUMFUS*8)+(FU_ALU+0)*8 +: 8];
    assign D_wb_last_subvector[FU_ALU+0]=D_last_subvector_s5[FU_ALU+0];

    assign D_wb_last_subvector[FU_FALU+0]=D_last_subvector_s5[FU_FALU+0];



  assign wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+0)]=dst[5*(NUMFUS*REGIDWIDTH)+(FU_MEM)*REGIDWIDTH +: REGIDWIDTH];
  assign wb_dst_we[FU_MEM]=dst_we[((FU_MEM-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_MEM-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)] && (~pipe_advance[5]|~pipe_squash[5]);
  assign wb_dst_mask[((FU_MEM-0)*((8-1)-(0)+1)+8-1) : ((FU_MEM-0)*((8-1)-(0)+1)+0)]=load_result_mask_s5;
  assign D_wb_last_subvector[FU_MEM]=D_last_subvector_s5[FU_MEM];

  assign wb_dst[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+0)]=dst[5*(NUMFUS*REGIDWIDTH)+(FU_MUL)*REGIDWIDTH +: REGIDWIDTH];
  assign wb_dst_we[FU_MUL]=dst_we[((FU_MUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_MUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)] && ~pipe_squash[5];
  assign wb_dst_mask[((FU_MUL-0)*((8-1)-(0)+1)+8-1) : ((FU_MUL-0)*((8-1)-(0)+1)+0)]=dst_mask[5*(NUMFUS*8)+(FU_MUL)*8 +: 8];
  assign D_wb_last_subvector[FU_MUL]=D_last_subvector_s5[FU_MUL];


  assign wb_dst[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+0)] = dst[5*(NUMFUS*REGIDWIDTH)+(FU_BFADDER)*REGIDWIDTH +: REGIDWIDTH];
  assign wb_dst_we[FU_BFADDER] = dst_we[((FU_BFADDER-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_BFADDER-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)] && ~pipe_squash[5];
  assign wb_dst_mask[((FU_BFADDER-0)*((8-1)-(0)+1)+8-1) : ((FU_BFADDER-0)*((8-1)-(0)+1)+0)] = dst_mask[5*(NUMFUS*8)+(FU_BFADDER)*8 +: 8];
  assign D_wb_last_subvector[FU_BFADDER] = D_last_subvector_s5[FU_BFADDER];

  assign wb_dst[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+0)] = dst[5*(NUMFUS*REGIDWIDTH)+(FU_BFMULT)*REGIDWIDTH +: REGIDWIDTH];
  assign wb_dst_we[FU_BFMULT] = dst_we[((FU_BFMULT-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_BFMULT-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)] && ~pipe_squash[5];
  assign wb_dst_mask[((FU_BFMULT-0)*((8-1)-(0)+1)+8-1) : ((FU_BFMULT-0)*((8-1)-(0)+1)+0)] = dst_mask[5*(NUMFUS*8)+(FU_BFMULT)*8 +: 8];
  assign D_wb_last_subvector[FU_BFMULT] = D_last_subvector_s5[FU_BFMULT];

  assign wb_dst[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+0)] = dst[5*(NUMFUS*REGIDWIDTH)+(FU_ACT)*REGIDWIDTH +: REGIDWIDTH];
  assign wb_dst_we[FU_ACT] = dst_we[((FU_ACT-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_ACT-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)] && ~pipe_squash[5];
  assign wb_dst_mask[((FU_ACT-0)*((8-1)-(0)+1)+8-1) : ((FU_ACT-0)*((8-1)-(0)+1)+0)] = dst_mask[5*(NUMFUS*8)+(FU_ACT)*8 +: 8];
  assign D_wb_last_subvector[FU_ACT] = D_last_subvector_s5[FU_ACT];

  assign wb_dst[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+0)]=dst[5*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH];
  assign wb_dst_we[FU_MATMUL]=dst_we[((FU_MATMUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_MATMUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)] && ~pipe_squash[5];
  assign wb_dst_mask[((FU_MATMUL-0)*((8-1)-(0)+1)+8-1) : ((FU_MATMUL-0)*((8-1)-(0)+1)+0)]=dst_mask[5*(NUMFUS*8)+(FU_MATMUL)*8 +: 8];
  //TODO: There is no code that assigns to the s31 var used below. Need to add that code
  //This is only a debug var, so it doesn't affect functionality
  assign D_wb_last_subvector[FU_MATMUL]=D_last_subvector_s31[FU_MATMUL];

  // ******************  Map functional units to banks ******************
  always@*
    for (bw=0; bw<2; bw=bw+1)
    begin
      vr_c_we[bw]=(wb_dst_we[FU_MUL]     && (wb_dst[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+0)]    &{{8-1{1'b0}},{1{1'b1}}})==bw) ||
                  (wb_dst_we[FU_MATMUL]  && (wb_dst[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+0)] &{{8-1{1'b0}},{1{1'b1}}})==bw) ||
                  (wb_dst_we[FU_BFADDER] && (wb_dst[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+0)]&{{8-1{1'b0}},{1{1'b1}}})==bw) ||
                  (wb_dst_we[FU_BFMULT]  && (wb_dst[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+0)] &{{8-1{1'b0}},{1{1'b1}}})==bw) ||
                  (wb_dst_we[FU_ACT]     && (wb_dst[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+0)]    &{{8-1{1'b0}},{1{1'b1}}})==bw) ||
                  (wb_dst_we[FU_MEM]     && (wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+0)]    &{{8-1{1'b0}},{1{1'b1}}})==bw) ||
                  (wb_dst_we[FU_ALU]     && (wb_dst[((FU_ALU-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_ALU-0)*((REGIDWIDTH-1)-(0)+1)+0)]    &{{8-1{1'b0}},{1{1'b1}}})==bw && 0==0) ||
                  (wb_dst_we[FU_ALU+bw]  && 0!=0);
      //TODO: Update this code for matmul. This is for debug only, so skipping //it for now.
      //Tells test_bench when to record register file contents.

      //Record if instruction writes to VRF, on last subvector, and not stalled
      D_wb_instrdone[bw] = pipe_advance[5] && (
        ((0==0) ?
          (dst_we[((FU_ALU-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_ALU-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)] && D_wb_last_subvector[FU_ALU] && ((wb_dst[FU_ALU] && {{8-1{1'b0}},{1{1'b1}}})==bw)) :
          (dst_we[((FU_ALU+bw-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_ALU+bw-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)] && D_wb_last_subvector[FU_ALU+bw]) || 
          (dst_we[((FU_MUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_MUL-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)] && D_wb_last_subvector[FU_MUL] && ((wb_dst[FU_MUL] && {{8-1{1'b0}},{1{1'b1}}})==bw))|| 
          (dst_we[((FU_MEM-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5) : ((FU_MEM-0)*((`MAX_PIPE_STAGES-1)-(4)+1)+5)] && D_wb_last_subvector[FU_MEM] && ((wb_dst[FU_MEM] && {{8-1{1'b0}},{1{1'b1}}})==bw))));
      //Take matmul output
      if (wb_dst_we[FU_MATMUL] && (wb_dst[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+0)] && ({{32-1{1'b0}},{1{1'b1}}}))==bw)
      begin
        vmask_final[((bw-0)*((8-1)-(0)+1)+8-1) : ((bw-0)*((8-1)-(0)+1)+0)]=wb_dst_mask[((FU_MATMUL-0)*((8-1)-(0)+1)+8-1) : ((FU_MATMUL-0)*((8-1)-(0)+1)+0)];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+0)]>>1;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((bw-0)*((REGIDWIDTH-1)-(0)+1)+0)]= wb_dst[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+0)];
        vr_c_writedatain[((bw-0)*((16*8-1)-(0)+1)+16*8-1) : ((bw-0)*((16*8-1)-(0)+1)+0)]= matmul_out;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MATMUL];
      end      
      else if(wb_dst_we[FU_TRP] && (wb_dst[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+0)] && ({{32-1{1'b0}},{1{1'b1}}}))==bw)
      begin
        vmask_final[((bw-0)*((8-1)-(0)+1)+8-1) : ((bw-0)*((8-1)-(0)+1)+0)]=wb_dst_mask[((FU_TRP-0)*((8-1)-(0)+1)+8-1) : ((FU_TRP-0)*((8-1)-(0)+1)+0)];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+0)]>>1;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((bw-0)*((REGIDWIDTH-1)-(0)+1)+0)]= wb_dst[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+0)];
//        vr_c_writedatain[bw]= trp_out;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_TRP];
      end
      //Take bfadder output
      else if (wb_dst_we[FU_BFADDER] && (wb_dst[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+0)] && ({{32-1{1'b0}},{1{1'b1}}})) ==bw)
      begin
        vmask_final[((bw-0)*((8-1)-(0)+1)+8-1) : ((bw-0)*((8-1)-(0)+1)+0)]=wb_dst_mask[((FU_BFADDER-0)*((8-1)-(0)+1)+8-1) : ((FU_BFADDER-0)*((8-1)-(0)+1)+0)];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+0)]>>1;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((bw-0)*((REGIDWIDTH-1)-(0)+1)+0)]= wb_dst[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+0)];
//        vr_c_writedatain[bw]= bfadder_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_BFADDER];
      end
      else if (wb_dst_we[FU_BFMULT] && (wb_dst[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+0)] && ({{32-1{1'b0}},{1{1'b1}}}))==bw)
      begin
        vmask_final[((bw-0)*((8-1)-(0)+1)+8-1) : ((bw-0)*((8-1)-(0)+1)+0)]=wb_dst_mask[((FU_BFMULT-0)*((8-1)-(0)+1)+8-1) : ((FU_BFMULT-0)*((8-1)-(0)+1)+0)];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+0)]>>1;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((bw-0)*((REGIDWIDTH-1)-(0)+1)+0)]= wb_dst[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+0)];
//        vr_c_writedatain[bw]= bfmult_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_BFMULT];
      end
      else if (wb_dst_we[FU_ACT] && (wb_dst[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+0)] &&({{32-1{1'b0}},{1{1'b1}}}))==bw)
      begin
        vmask_final[((bw-0)*((8-1)-(0)+1)+8-1) : ((bw-0)*((8-1)-(0)+1)+0)]=wb_dst_mask[((FU_ACT-0)*((8-1)-(0)+1)+8-1) : ((FU_ACT-0)*((8-1)-(0)+1)+0)];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+0)]>>1;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((bw-0)*((REGIDWIDTH-1)-(0)+1)+0)]= wb_dst[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+0)];
//        vr_c_writedatain[bw]= act_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_ACT];
      end
      //Take multiplier output
      else if (wb_dst_we[FU_MUL] && (wb_dst[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+0)] && ({{32-1{1'b0}},{1{1'b1}}}))==bw)
      begin
        vmask_final[((bw-0)*((8-1)-(0)+1)+8-1) : ((bw-0)*((8-1)-(0)+1)+0)]=wb_dst_mask[((FU_MUL-0)*((8-1)-(0)+1)+8-1) : ((FU_MUL-0)*((8-1)-(0)+1)+0)];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+0)]>>1;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((bw-0)*((REGIDWIDTH-1)-(0)+1)+0)]= wb_dst[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+0)];
        vr_c_writedatain[((bw-0)*((16*8-1)-(0)+1)+16*8-1) : ((bw-0)*((16*8-1)-(0)+1)+0)]= mulshift_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MUL];
      end
      //Take Memory unit output
      else if (wb_dst_we[FU_MEM] && (wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+0)]&&({{32-1{1'b0}},{1{1'b1}}}))==bw)
      begin
        vmask_final[((bw-0)*((8-1)-(0)+1)+8-1) : ((bw-0)*((8-1)-(0)+1)+0)]=wb_dst_mask[((FU_MEM-0)*((8-1)-(0)+1)+8-1) : ((FU_MEM-0)*((8-1)-(0)+1)+0)];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+0)]>>1;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((bw-0)*((REGIDWIDTH-1)-(0)+1)+0)]= wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+0)];
        vr_c_writedatain[((bw-0)*((16*8-1)-(0)+1)+16*8-1) : ((bw-0)*((16*8-1)-(0)+1)+0)]= load_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MEM];
      end
      else
      //Take ALU output
      begin
        vmask_final[((bw-0)*((8-1)-(0)+1)+8-1) : ((bw-0)*((8-1)-(0)+1)+0)]=wb_dst_mask[FU_ALU+bw*0];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_ALU+bw*0]>>1;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((bw-0)*((REGIDWIDTH-1)-(0)+1)+0)]= wb_dst[FU_ALU+bw*0];
        vr_c_writedatain[((bw-0)*((16*8-1)-(0)+1)+16*8-1) : ((bw-0)*((16*8-1)-(0)+1)+0)]= alu_result_s5[((bw*0-0)*((16*8-1)-(0)+1)+16*8-1) : ((bw*0-0)*((16*8-1)-(0)+1)+0)];
        //Do ALU and FALU for last subvector
        D_last_subvector_done[bw]=
          (D_wb_last_subvector[FU_ALU+bw*0] && (wb_dst[FU_ALU+bw*0] & ({{32-1{1'b0}},{1{1'b1}}})) ==bw) | 
          (D_wb_last_subvector[FU_FALU+bw*0] && (dst[5*(NUMFUS * REGIDWIDTH) + ((FU_FALU+bw)*0) * REGIDWIDTH +: REGIDWIDTH] &({{32-1{1'b0}},{1{1'b1}}}))==bw);
      end


      //Generate byte enable from mask
      for (j=0; j<8; j=j+1)
        vr_c_byteen[bw*2*8 + j*2 +: 2 ]={2{vmask_final[((bw-0)*((8-1)-(0)+1)+j) : ((bw-0)*((8-1)-(0)+1)+j)]}};

      //*********** Flag writeback ***********
    //  vf_c_reg[bw*BANKREGIDWIDTH+:BANKREGIDWIDTH]=dst[5*(NUMFUS* REGIDWIDTH) + (FU_FALU+bw*0)*REGIDWIDTH +: REGIDWIDTH]>>1;
      vf_c_writedatain[bw*8+:8]=flagalu_result_s5[((bw*0-0)*((8-1)-(0)+1)+8-1) : ((bw*0-0)*((8-1)-(0)+1)+0)];
    //  vf_c_we[bw]= (dst_we[FU_FALU+bw*0][5] && (dst[5*(NUMFUS*REGIDWIDTH)+ (FU_FALU+bw*0) * REGIDWIDTH +: REGIDWIDTH] & ({{32-1{1'b0}},{1{1'b1}}}))==bw);
    end

  //********** Scalar writeback ***********
  assign vs_wetrack={ctrl_vs_we[5:4],|ctrl3_vs_we,ctrl2_vs_we};
  assign vs_we=ctrl_vs_we[5] & load_result_mask_s5[0];
  assign vs_dst=wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-VRIDWIDTH)];
  assign vs_writedata=load_result_s5[16-1:0];

endmodule



/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_8_1 (
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
input [ 8-1:0 ]  squash;

input [ 8*1-1:0 ]  inpipe;
output [ 8*1-1:0 ] outpipe;

input [ 1-1:0 ]  shiftin_left;
input [ 1-1:0 ]  shiftin_right;

wire [ (8+1)*1-1:0 ] _outpipe;

wire [1-1:0] temp_left;  // Sangram: added to remove warning
wire [1-1:0] temp_right;  // Sangram: added to remove warning
assign temp_left = shiftin_left; 
assign temp_right = shiftin_right; 

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


        wire [ 1-1:0 ] velmshifter_laneunit1inpipe;
        wire [ 1-1:0 ] velmshifter_laneunit1outpipe1;
        wire [ 1-1:0 ] velmshifter_laneunit1outpipe2;
        wire [ 1-1:0 ] velmshifter_laneunit1outpipe3;

        assign velmshifter_laneunit1inpipe = inpipe[(1+1)*1-1:(1)*1];

        assign velmshifter_laneunit1outpipe1 = _outpipe[(1+2)*1-1:(1+1)*1];

        assign velmshifter_laneunit1outpipe2 = _outpipe[(1)*1-1:(1-1)*1];

        assign _outpipe[(1+1)*1-1:(1)*1] = velmshifter_laneunit1outpipe3;

      velmshifter_laneunit_1 velmshifter_laneunit1(clk,resetn,load,shift,dir_left,
          squash[(1)],
          velmshifter_laneunit1inpipe,
          velmshifter_laneunit1outpipe1,
          velmshifter_laneunit1outpipe2,
          velmshifter_laneunit1outpipe3);
     // defparam velmshifter_laneunit.1=1;


        wire [ 1-1:0 ] velmshifter_laneunit2inpipe;
        wire [ 1-1:0 ] velmshifter_laneunit2outpipe1;
        wire [ 1-1:0 ] velmshifter_laneunit2outpipe2;
        wire [ 1-1:0 ] velmshifter_laneunit2outpipe3;

        assign velmshifter_laneunit2inpipe = inpipe[(2+1)*1-1:(2)*1];

        assign velmshifter_laneunit2outpipe1 = _outpipe[(2+2)*1-1:(2+1)*1];

        assign velmshifter_laneunit2outpipe2 = _outpipe[(2)*1-1:(2-1)*1];

        assign _outpipe[(2+1)*1-1:(2)*1] = velmshifter_laneunit2outpipe3;

      velmshifter_laneunit_1 velmshifter_laneunit2(clk,resetn,load,shift,dir_left,
          squash[(2)],
          velmshifter_laneunit2inpipe,
          velmshifter_laneunit2outpipe1,
          velmshifter_laneunit2outpipe2,
          velmshifter_laneunit2outpipe3);
     // defparam velmshifter_laneunit.1=1;


        wire [ 1-1:0 ] velmshifter_laneunit3inpipe;
        wire [ 1-1:0 ] velmshifter_laneunit3outpipe1;
        wire [ 1-1:0 ] velmshifter_laneunit3outpipe2;
        wire [ 1-1:0 ] velmshifter_laneunit3outpipe3;

        assign velmshifter_laneunit3inpipe = inpipe[(3+1)*1-1:(3)*1];

        assign velmshifter_laneunit3outpipe1 = _outpipe[(3+2)*1-1:(3+1)*1];

        assign velmshifter_laneunit3outpipe2 = _outpipe[(3)*1-1:(3-1)*1];

        assign _outpipe[(3+1)*1-1:(3)*1] = velmshifter_laneunit3outpipe3;

      velmshifter_laneunit_1 velmshifter_laneunit3(clk,resetn,load,shift,dir_left,
          squash[(3)],
          velmshifter_laneunit3inpipe,
          velmshifter_laneunit3outpipe1,
          velmshifter_laneunit3outpipe2,
          velmshifter_laneunit3outpipe3);
     // defparam velmshifter_laneunit.1=1;


        wire [ 1-1:0 ] velmshifter_laneunit4inpipe;
        wire [ 1-1:0 ] velmshifter_laneunit4outpipe1;
        wire [ 1-1:0 ] velmshifter_laneunit4outpipe2;
        wire [ 1-1:0 ] velmshifter_laneunit4outpipe3;

        assign velmshifter_laneunit4inpipe = inpipe[(4+1)*1-1:(4)*1];

        assign velmshifter_laneunit4outpipe1 = _outpipe[(4+2)*1-1:(4+1)*1];

        assign velmshifter_laneunit4outpipe2 = _outpipe[(4)*1-1:(4-1)*1];

        assign _outpipe[(4+1)*1-1:(4)*1] = velmshifter_laneunit4outpipe3;

      velmshifter_laneunit_1 velmshifter_laneunit4(clk,resetn,load,shift,dir_left,
          squash[(4)],
          velmshifter_laneunit4inpipe,
          velmshifter_laneunit4outpipe1,
          velmshifter_laneunit4outpipe2,
          velmshifter_laneunit4outpipe3);
     // defparam velmshifter_laneunit.1=1;


        wire [ 1-1:0 ] velmshifter_laneunit5inpipe;
        wire [ 1-1:0 ] velmshifter_laneunit5outpipe1;
        wire [ 1-1:0 ] velmshifter_laneunit5outpipe2;
        wire [ 1-1:0 ] velmshifter_laneunit5outpipe3;

        assign velmshifter_laneunit5inpipe = inpipe[(5+1)*1-1:(5)*1];

        assign velmshifter_laneunit5outpipe1 = _outpipe[(5+2)*1-1:(5+1)*1];

        assign velmshifter_laneunit5outpipe2 = _outpipe[(5)*1-1:(5-1)*1];

        assign _outpipe[(5+1)*1-1:(5)*1] = velmshifter_laneunit5outpipe3;

      velmshifter_laneunit_1 velmshifter_laneunit5(clk,resetn,load,shift,dir_left,
          squash[(5)],
          velmshifter_laneunit5inpipe,
          velmshifter_laneunit5outpipe1,
          velmshifter_laneunit5outpipe2,
          velmshifter_laneunit5outpipe3);
     // defparam velmshifter_laneunit.1=1;


        wire [ 1-1:0 ] velmshifter_laneunit6inpipe;
        wire [ 1-1:0 ] velmshifter_laneunit6outpipe1;
        wire [ 1-1:0 ] velmshifter_laneunit6outpipe2;
        wire [ 1-1:0 ] velmshifter_laneunit6outpipe3;

        assign velmshifter_laneunit6inpipe = inpipe[(6+1)*1-1:(6)*1];

        assign velmshifter_laneunit6outpipe1 = _outpipe[(6+2)*1-1:(6+1)*1];

        assign velmshifter_laneunit6outpipe2 = _outpipe[(6)*1-1:(6-1)*1];

        assign _outpipe[(6+1)*1-1:(6)*1] = velmshifter_laneunit6outpipe3;

      velmshifter_laneunit_1 velmshifter_laneunit6(clk,resetn,load,shift,dir_left,
          squash[(6)],
          velmshifter_laneunit6inpipe,
          velmshifter_laneunit6outpipe1,
          velmshifter_laneunit6outpipe2,
          velmshifter_laneunit6outpipe3);
     // defparam velmshifter_laneunit.1=1;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_1 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[7],
      inpipe[7],
      shiftin_left,
      _outpipe[6], //L=1
      _outpipe[7]); //L=1
   // defparam velmshifter_laneunitlast.1=1;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
`ifndef MODULE_RAM_WRAPPER_5_32_32 
`define MODULE_RAM_WRAPPER_5_32_32 
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
input [5-1:0] address_a;
input [5-1:0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [32-1:0] data_a;
input [32-1:0] data_b;
output [32-1:0] out_a;
output [32-1:0] out_b;

reg [5-1:0] q_address_a;
reg [5-1:0] q_address_b;
reg [5-1:0] mux_address_b;

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

endmodule
`endif

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

module vlane_alu_16(
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

parameter WIDTH=16;

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

  local_add_sub_18_0_SIGNED local_adder_inst(
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

  vlane_saturatesum_16 satsum(
      .in(adder_result_s2),
      .op(satsum_op_s2),
      .out(satsum_result)
      );

  assign mux1_result= (ctrl_mux1_sel_s2) ? 
                              satsum_result : satsize_result;

  assign result=mux1_result;

endmodule
module velmshifter_laneunit_16 (
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

input [ 16-1:0 ]  inpipe;
input [ 16-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 16-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 16-1:0 ] outpipe;

reg [ 16-1:0 ] outpipe;   // Cross-lane output lane i+1


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

module velmshifter_jump_16_8_8 (
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
input  jump;              //0-shift by 1, 1 shift by 8
input [ 8-1:0 ]  squash;

input [ 8*16-1:0 ]  inpipe;
output [ 8*16-1:0 ] outpipe;

input [ 16-1:0 ]  shiftin_left;
input [ 16-1:0 ]  shiftin_right;

wire [16-1:0] temp_left;  // Sangram: added to remove warning
wire [16-1:0] temp_right;  // Sangram: added to remove warning
assign temp_left = shiftin_left; 
assign temp_right = shiftin_right; 

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter_8_16 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<(16*8) :
                      outpipe >>(16*8)),
      .outpipe(outpipe));

endmodule
        
/******************************************************************************/
/**************************** Shifter w Jump **********************************/
/******************************************************************************/

module velmshifter_jump_32_8_8 (
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
input  jump;              //0-shift by 1, 1 shift by 8
input [ 8-1:0 ]  squash;

input [ 8*32-1:0 ]  inpipe;
output [ 8*32-1:0 ] outpipe;

input [ 32-1:0 ]  shiftin_left;
input [ 32-1:0 ]  shiftin_right;

wire [32-1:0] temp_left;  // Sangram: added to remove warning
wire [32-1:0] temp_right;  // Sangram: added to remove warning
assign temp_left = shiftin_left; 
assign temp_right = shiftin_right; 

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter_8_32 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<(32*8) :
                      outpipe >>(32*8)),
      .outpipe(outpipe));

endmodule
        
/******************************************************************************/
/**************************** Shifter w Jump **********************************/
/******************************************************************************/

module velmshifter_jump_1_8_8 (
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
input  jump;              //0-shift by 1, 1 shift by 8
input [ 8-1:0 ]  squash;

input [ 8*1-1:0 ]  inpipe;
output [ 8*1-1:0 ] outpipe;

input [ 1-1:0 ]  shiftin_left;
input [ 1-1:0 ]  shiftin_right;

wire [1-1:0] temp_left;  // Sangram: added to remove warning
wire [1-1:0] temp_right;  // Sangram: added to remove warning
assign temp_left = shiftin_left; 
assign temp_right = shiftin_right; 

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter_8_1 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<(1*8) :
                      outpipe >>(1*8)),
      .outpipe(outpipe));

endmodule
        

/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_8_16 (
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
input [ 8-1:0 ]  squash;

input [ 8*16-1:0 ]  inpipe;
output [ 8*16-1:0 ] outpipe;

input [ 16-1:0 ]  shiftin_left;
input [ 16-1:0 ]  shiftin_right;

wire [ (8+1)*16-1:0 ] _outpipe;

wire [16-1:0] temp_left;  // Sangram: added to remove warning
wire [16-1:0] temp_right;  // Sangram: added to remove warning
assign temp_left = shiftin_left; 
assign temp_right = shiftin_right; 

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_16 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[15:0],
      _outpipe[31:16], //Support 1 lane
      shiftin_right,
      _outpipe[15:0]);
 // defparam velmshifter_laneunit0.16=16;

  //Generate everything in between 


        wire [ 16-1:0 ] velmshifter_laneunit1inpipe;
        wire [ 16-1:0 ] velmshifter_laneunit1outpipe1;
        wire [ 16-1:0 ] velmshifter_laneunit1outpipe2;
        wire [ 16-1:0 ] velmshifter_laneunit1outpipe3;

        assign velmshifter_laneunit1inpipe = inpipe[(1+1)*16-1:(1)*16];

        assign velmshifter_laneunit1outpipe1 = _outpipe[(1+2)*16-1:(1+1)*16];

        assign velmshifter_laneunit1outpipe2 = _outpipe[(1)*16-1:(1-1)*16];

        assign _outpipe[(1+1)*16-1:(1)*16] = velmshifter_laneunit1outpipe3;

      velmshifter_laneunit_16 velmshifter_laneunit1(clk,resetn,load,shift,dir_left,
          squash[(1)],
          velmshifter_laneunit1inpipe,
          velmshifter_laneunit1outpipe1,
          velmshifter_laneunit1outpipe2,
          velmshifter_laneunit1outpipe3);
     // defparam velmshifter_laneunit.16=16;


        wire [ 16-1:0 ] velmshifter_laneunit2inpipe;
        wire [ 16-1:0 ] velmshifter_laneunit2outpipe1;
        wire [ 16-1:0 ] velmshifter_laneunit2outpipe2;
        wire [ 16-1:0 ] velmshifter_laneunit2outpipe3;

        assign velmshifter_laneunit2inpipe = inpipe[(2+1)*16-1:(2)*16];

        assign velmshifter_laneunit2outpipe1 = _outpipe[(2+2)*16-1:(2+1)*16];

        assign velmshifter_laneunit2outpipe2 = _outpipe[(2)*16-1:(2-1)*16];

        assign _outpipe[(2+1)*16-1:(2)*16] = velmshifter_laneunit2outpipe3;

      velmshifter_laneunit_16 velmshifter_laneunit2(clk,resetn,load,shift,dir_left,
          squash[(2)],
          velmshifter_laneunit2inpipe,
          velmshifter_laneunit2outpipe1,
          velmshifter_laneunit2outpipe2,
          velmshifter_laneunit2outpipe3);
     // defparam velmshifter_laneunit.16=16;


        wire [ 16-1:0 ] velmshifter_laneunit3inpipe;
        wire [ 16-1:0 ] velmshifter_laneunit3outpipe1;
        wire [ 16-1:0 ] velmshifter_laneunit3outpipe2;
        wire [ 16-1:0 ] velmshifter_laneunit3outpipe3;

        assign velmshifter_laneunit3inpipe = inpipe[(3+1)*16-1:(3)*16];

        assign velmshifter_laneunit3outpipe1 = _outpipe[(3+2)*16-1:(3+1)*16];

        assign velmshifter_laneunit3outpipe2 = _outpipe[(3)*16-1:(3-1)*16];

        assign _outpipe[(3+1)*16-1:(3)*16] = velmshifter_laneunit3outpipe3;

      velmshifter_laneunit_16 velmshifter_laneunit3(clk,resetn,load,shift,dir_left,
          squash[(3)],
          velmshifter_laneunit3inpipe,
          velmshifter_laneunit3outpipe1,
          velmshifter_laneunit3outpipe2,
          velmshifter_laneunit3outpipe3);
     // defparam velmshifter_laneunit.16=16;


        wire [ 16-1:0 ] velmshifter_laneunit4inpipe;
        wire [ 16-1:0 ] velmshifter_laneunit4outpipe1;
        wire [ 16-1:0 ] velmshifter_laneunit4outpipe2;
        wire [ 16-1:0 ] velmshifter_laneunit4outpipe3;

        assign velmshifter_laneunit4inpipe = inpipe[(4+1)*16-1:(4)*16];

        assign velmshifter_laneunit4outpipe1 = _outpipe[(4+2)*16-1:(4+1)*16];

        assign velmshifter_laneunit4outpipe2 = _outpipe[(4)*16-1:(4-1)*16];

        assign _outpipe[(4+1)*16-1:(4)*16] = velmshifter_laneunit4outpipe3;

      velmshifter_laneunit_16 velmshifter_laneunit4(clk,resetn,load,shift,dir_left,
          squash[(4)],
          velmshifter_laneunit4inpipe,
          velmshifter_laneunit4outpipe1,
          velmshifter_laneunit4outpipe2,
          velmshifter_laneunit4outpipe3);
     // defparam velmshifter_laneunit.16=16;


        wire [ 16-1:0 ] velmshifter_laneunit5inpipe;
        wire [ 16-1:0 ] velmshifter_laneunit5outpipe1;
        wire [ 16-1:0 ] velmshifter_laneunit5outpipe2;
        wire [ 16-1:0 ] velmshifter_laneunit5outpipe3;

        assign velmshifter_laneunit5inpipe = inpipe[(5+1)*16-1:(5)*16];

        assign velmshifter_laneunit5outpipe1 = _outpipe[(5+2)*16-1:(5+1)*16];

        assign velmshifter_laneunit5outpipe2 = _outpipe[(5)*16-1:(5-1)*16];

        assign _outpipe[(5+1)*16-1:(5)*16] = velmshifter_laneunit5outpipe3;

      velmshifter_laneunit_16 velmshifter_laneunit5(clk,resetn,load,shift,dir_left,
          squash[(5)],
          velmshifter_laneunit5inpipe,
          velmshifter_laneunit5outpipe1,
          velmshifter_laneunit5outpipe2,
          velmshifter_laneunit5outpipe3);
     // defparam velmshifter_laneunit.16=16;


        wire [ 16-1:0 ] velmshifter_laneunit6inpipe;
        wire [ 16-1:0 ] velmshifter_laneunit6outpipe1;
        wire [ 16-1:0 ] velmshifter_laneunit6outpipe2;
        wire [ 16-1:0 ] velmshifter_laneunit6outpipe3;

        assign velmshifter_laneunit6inpipe = inpipe[(6+1)*16-1:(6)*16];

        assign velmshifter_laneunit6outpipe1 = _outpipe[(6+2)*16-1:(6+1)*16];

        assign velmshifter_laneunit6outpipe2 = _outpipe[(6)*16-1:(6-1)*16];

        assign _outpipe[(6+1)*16-1:(6)*16] = velmshifter_laneunit6outpipe3;

      velmshifter_laneunit_16 velmshifter_laneunit6(clk,resetn,load,shift,dir_left,
          squash[(6)],
          velmshifter_laneunit6inpipe,
          velmshifter_laneunit6outpipe1,
          velmshifter_laneunit6outpipe2,
          velmshifter_laneunit6outpipe3);
     // defparam velmshifter_laneunit.16=16;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_16 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[7],
      inpipe[127:112],
      shiftin_left,
      _outpipe[111:96], //L=1
      _outpipe[127:112]); //L=1
   // defparam velmshifter_laneunitlast.16=16;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
`ifndef MODULE_RAM_WRAPPER_7_128_8 
`define MODULE_RAM_WRAPPER_7_128_8 
module ram_wrapper_7_128_8 (
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
input [7-1:0] address_a;
input [7-1:0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [8-1:0] data_a;
input [8-1:0] data_b;
output [8-1:0] out_a;
output [8-1:0] out_b;

reg [7-1:0] q_address_a;
reg [7-1:0] q_address_b;
reg [7-1:0] mux_address_b;

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
`endif


///////////////////////////////////////////////////////
//Rounding logic based on convergent rounding described 
//here: https://zipcpu.com/dsp/2017/07/22/rounding.html
///////////////////////////////////////////////////////
module rounding_20_16( i_data, o_data );
input  [20-1:0] i_data;
output [16-1:0] o_data;

wire [20-1:0] w_convergent;

assign	w_convergent = i_data[(20-1):0]
			+ { {(16){1'b0}},
				i_data[(20-16)],
                                {(20-16-1){!i_data[(20-16)]}}};

assign o_data = w_convergent[(20-1):(20-16)];

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
/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_vector_2_1_128_256_8
  (clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_byteen, c_we);

input clk;
input resetn;

input [2-1:0] a_en;
input [2-1:0] b_en;
input [2*7-1:0] a_reg,b_reg,c_reg;
output [2*128-1:0] a_readdataout, b_readdataout;
wire [2*128-1:0] a_temp, b_temp;

input [2*128-1:0] c_writedatain;
input [2*128/8-1:0] c_byteen;
input [2-1:0] c_we;


        ram_wrapper_7_128_128 reg_file1_0(
  	    .clk(clk),
        .resetn(resetn),
        .rden_a(1'b0),
        .rden_b(a_en[0]),
  	    .address_a(c_reg[0*7 +: 7]),
  	    .address_b(a_reg[0*7 +: 7]),
  	    .wren_a(c_we[0] &  c_byteen[0]),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[0*128 +: 128]),
  	    .data_b(0),
  	    .out_a(a_temp[0*128 +: 128]),
  	    .out_b(a_readdataout[0*128 +: 128])
          );
  
        ram_wrapper_7_128_128 reg_file2_0(
  	    .clk(clk),
        .resetn(resetn),
        .rden_a(1'b0),
        .rden_b(b_en[0]),
  	    .address_a(c_reg[0*7 +: 7]),
  	    .address_b(b_reg[0*7 +: 7]),
  	    .wren_a(c_we[0] &  c_byteen[0]),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[0*128+:128]),
  	    .data_b(0),
  	    .out_a(b_temp[0*128 +: 128]),
  	    .out_b(b_readdataout[0*128+:128])
          );

        ram_wrapper_7_128_128 reg_file1_1(
  	    .clk(clk),
        .resetn(resetn),
        .rden_a(1'b0),
        .rden_b(a_en[1]),
  	    .address_a(c_reg[1*7 +: 7]),
  	    .address_b(a_reg[1*7 +: 7]),
  	    .wren_a(c_we[1] &  c_byteen[1]),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[1*128 +: 128]),
  	    .data_b(0),
  	    .out_a(a_temp[1*128 +: 128]),
  	    .out_b(a_readdataout[1*128 +: 128])
          );
  
        ram_wrapper_7_128_128 reg_file2_1(
  	    .clk(clk),
        .resetn(resetn),
        .rden_a(1'b0),
        .rden_b(b_en[1]),
  	    .address_a(c_reg[1*7 +: 7]),
  	    .address_b(b_reg[1*7 +: 7]),
  	    .wren_a(c_we[1] &  c_byteen[1]),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[1*128+:128]),
  	    .data_b(0),
  	    .out_a(b_temp[1*128 +: 128]),
  	    .out_b(b_readdataout[1*128+:128])
          );

endmodule



/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_8_32 (
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
input [ 8-1:0 ]  squash;

input [ 8*32-1:0 ]  inpipe;
output [ 8*32-1:0 ] outpipe;

input [ 32-1:0 ]  shiftin_left;
input [ 32-1:0 ]  shiftin_right;

wire [ (8+1)*32-1:0 ] _outpipe;

wire [32-1:0] temp_left;  // Sangram: added to remove warning
wire [32-1:0] temp_right;  // Sangram: added to remove warning
assign temp_left = shiftin_left; 
assign temp_right = shiftin_right; 

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


        wire [ 32-1:0 ] velmshifter_laneunit1inpipe;
        wire [ 32-1:0 ] velmshifter_laneunit1outpipe1;
        wire [ 32-1:0 ] velmshifter_laneunit1outpipe2;
        wire [ 32-1:0 ] velmshifter_laneunit1outpipe3;

        assign velmshifter_laneunit1inpipe = inpipe[(1+1)*32-1:(1)*32];

        assign velmshifter_laneunit1outpipe1 = _outpipe[(1+2)*32-1:(1+1)*32];

        assign velmshifter_laneunit1outpipe2 = _outpipe[(1)*32-1:(1-1)*32];

        assign _outpipe[(1+1)*32-1:(1)*32] = velmshifter_laneunit1outpipe3;

      velmshifter_laneunit_32 velmshifter_laneunit1(clk,resetn,load,shift,dir_left,
          squash[(1)],
          velmshifter_laneunit1inpipe,
          velmshifter_laneunit1outpipe1,
          velmshifter_laneunit1outpipe2,
          velmshifter_laneunit1outpipe3);
     // defparam velmshifter_laneunit.32=32;


        wire [ 32-1:0 ] velmshifter_laneunit2inpipe;
        wire [ 32-1:0 ] velmshifter_laneunit2outpipe1;
        wire [ 32-1:0 ] velmshifter_laneunit2outpipe2;
        wire [ 32-1:0 ] velmshifter_laneunit2outpipe3;

        assign velmshifter_laneunit2inpipe = inpipe[(2+1)*32-1:(2)*32];

        assign velmshifter_laneunit2outpipe1 = _outpipe[(2+2)*32-1:(2+1)*32];

        assign velmshifter_laneunit2outpipe2 = _outpipe[(2)*32-1:(2-1)*32];

        assign _outpipe[(2+1)*32-1:(2)*32] = velmshifter_laneunit2outpipe3;

      velmshifter_laneunit_32 velmshifter_laneunit2(clk,resetn,load,shift,dir_left,
          squash[(2)],
          velmshifter_laneunit2inpipe,
          velmshifter_laneunit2outpipe1,
          velmshifter_laneunit2outpipe2,
          velmshifter_laneunit2outpipe3);
     // defparam velmshifter_laneunit.32=32;


        wire [ 32-1:0 ] velmshifter_laneunit3inpipe;
        wire [ 32-1:0 ] velmshifter_laneunit3outpipe1;
        wire [ 32-1:0 ] velmshifter_laneunit3outpipe2;
        wire [ 32-1:0 ] velmshifter_laneunit3outpipe3;

        assign velmshifter_laneunit3inpipe = inpipe[(3+1)*32-1:(3)*32];

        assign velmshifter_laneunit3outpipe1 = _outpipe[(3+2)*32-1:(3+1)*32];

        assign velmshifter_laneunit3outpipe2 = _outpipe[(3)*32-1:(3-1)*32];

        assign _outpipe[(3+1)*32-1:(3)*32] = velmshifter_laneunit3outpipe3;

      velmshifter_laneunit_32 velmshifter_laneunit3(clk,resetn,load,shift,dir_left,
          squash[(3)],
          velmshifter_laneunit3inpipe,
          velmshifter_laneunit3outpipe1,
          velmshifter_laneunit3outpipe2,
          velmshifter_laneunit3outpipe3);
     // defparam velmshifter_laneunit.32=32;


        wire [ 32-1:0 ] velmshifter_laneunit4inpipe;
        wire [ 32-1:0 ] velmshifter_laneunit4outpipe1;
        wire [ 32-1:0 ] velmshifter_laneunit4outpipe2;
        wire [ 32-1:0 ] velmshifter_laneunit4outpipe3;

        assign velmshifter_laneunit4inpipe = inpipe[(4+1)*32-1:(4)*32];

        assign velmshifter_laneunit4outpipe1 = _outpipe[(4+2)*32-1:(4+1)*32];

        assign velmshifter_laneunit4outpipe2 = _outpipe[(4)*32-1:(4-1)*32];

        assign _outpipe[(4+1)*32-1:(4)*32] = velmshifter_laneunit4outpipe3;

      velmshifter_laneunit_32 velmshifter_laneunit4(clk,resetn,load,shift,dir_left,
          squash[(4)],
          velmshifter_laneunit4inpipe,
          velmshifter_laneunit4outpipe1,
          velmshifter_laneunit4outpipe2,
          velmshifter_laneunit4outpipe3);
     // defparam velmshifter_laneunit.32=32;


        wire [ 32-1:0 ] velmshifter_laneunit5inpipe;
        wire [ 32-1:0 ] velmshifter_laneunit5outpipe1;
        wire [ 32-1:0 ] velmshifter_laneunit5outpipe2;
        wire [ 32-1:0 ] velmshifter_laneunit5outpipe3;

        assign velmshifter_laneunit5inpipe = inpipe[(5+1)*32-1:(5)*32];

        assign velmshifter_laneunit5outpipe1 = _outpipe[(5+2)*32-1:(5+1)*32];

        assign velmshifter_laneunit5outpipe2 = _outpipe[(5)*32-1:(5-1)*32];

        assign _outpipe[(5+1)*32-1:(5)*32] = velmshifter_laneunit5outpipe3;

      velmshifter_laneunit_32 velmshifter_laneunit5(clk,resetn,load,shift,dir_left,
          squash[(5)],
          velmshifter_laneunit5inpipe,
          velmshifter_laneunit5outpipe1,
          velmshifter_laneunit5outpipe2,
          velmshifter_laneunit5outpipe3);
     // defparam velmshifter_laneunit.32=32;


        wire [ 32-1:0 ] velmshifter_laneunit6inpipe;
        wire [ 32-1:0 ] velmshifter_laneunit6outpipe1;
        wire [ 32-1:0 ] velmshifter_laneunit6outpipe2;
        wire [ 32-1:0 ] velmshifter_laneunit6outpipe3;

        assign velmshifter_laneunit6inpipe = inpipe[(6+1)*32-1:(6)*32];

        assign velmshifter_laneunit6outpipe1 = _outpipe[(6+2)*32-1:(6+1)*32];

        assign velmshifter_laneunit6outpipe2 = _outpipe[(6)*32-1:(6-1)*32];

        assign _outpipe[(6+1)*32-1:(6)*32] = velmshifter_laneunit6outpipe3;

      velmshifter_laneunit_32 velmshifter_laneunit6(clk,resetn,load,shift,dir_left,
          squash[(6)],
          velmshifter_laneunit6inpipe,
          velmshifter_laneunit6outpipe1,
          velmshifter_laneunit6outpipe2,
          velmshifter_laneunit6outpipe3);
     // defparam velmshifter_laneunit.32=32;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_32 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[7],
      inpipe[255:224],
      shiftin_left,
      _outpipe[223:192], //L=1
      _outpipe[255:224]); //L=1
   // defparam velmshifter_laneunitlast.32=32;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

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
module reduction_unit_16_4(
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
  input  [16-1 : 0] inp0; 
  input  [16-1 : 0] inp1; 
  input  [16-1 : 0] inp2; 
  input  [16-1 : 0] inp3; 
  input  [16-1 : 0] inp4; 
  input  [16-1 : 0] inp5; 
  input  [16-1 : 0] inp6; 
  input  [16-1 : 0] inp7; 

  input [1:0] mode;
  output [16+4-1 : 0] outp;


  wire   [16+4-1 : 0] compute0_out_stage3;
  reg    [16+4-1 : 0] compute0_out_stage3_reg;
  wire   [16+4-1 : 0] compute1_out_stage3;
  reg    [16+4-1 : 0] compute1_out_stage3_reg;
  wire   [16+4-1 : 0] compute2_out_stage3;
  reg    [16+4-1 : 0] compute2_out_stage3_reg;
  wire   [16+4-1 : 0] compute3_out_stage3;
  wire   [16+4-1 : 0] compute3_out_stage3_reg;

  wire   [16+4-1 : 0] compute0_out_stage2;
  reg    [16+4-1 : 0] compute0_out_stage2_reg;
  wire   [16+4-1 : 0] compute1_out_stage2;
  reg    [16+4-1 : 0] compute1_out_stage2_reg;

  wire   [16+4-1 : 0] compute0_out_stage1;
  reg    [16+4-1 : 0] compute0_out_stage1_reg;

  wire   [16+4-1 : 0] compute0_out_stage0;
  reg    [16+4-1 : 0] outp;

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
  reduction_processing_element_16_20 compute0_stage3(.A(inp0),       .B(inp1),    .OUT(compute0_out_stage3), .MODE(mode));
  reduction_processing_element_16_20 compute1_stage3(.A(inp2),       .B(inp3),    .OUT(compute1_out_stage3), .MODE(mode));
  reduction_processing_element_16_20 compute2_stage3(.A(inp4),       .B(inp5),    .OUT(compute2_out_stage3), .MODE(mode));
  reduction_processing_element_16_20 compute3_stage3(.A(inp6),       .B(inp7),    .OUT(compute3_out_stage3), .MODE(mode));

  reduction_processing_element_20_20 compute0_stage2(.A(compute0_out_stage3_reg),       .B(compute1_out_stage3_reg),    .OUT(compute0_out_stage2), .MODE(mode));
  reduction_processing_element_20_20 compute1_stage2(.A(compute2_out_stage3_reg),       .B(compute3_out_stage3_reg),    .OUT(compute1_out_stage2), .MODE(mode));

  reduction_processing_element_20_20 compute0_stage1(.A(compute0_out_stage2_reg),       .B(compute1_out_stage2_reg),    .OUT(compute0_out_stage1), .MODE(mode));

  reduction_processing_element_20_20 compute0_stage0(.A(outp),       .B(compute0_out_stage1_reg),     .OUT(compute0_out_stage0), .MODE(mode));

endmodule

/****************************************************************************
          Control Register File

   - Has one read port (a) and one write port (c)
   - vl promoted as first-class entities
****************************************************************************/
module vregfile_control_32_32_5 (
    clk,
    resetn, 

    a_reg, 
    a_en,
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we,

    vl,
    matmul_masks
    );

input clk;
input resetn;

input a_en;
input [5-1:0] a_reg,c_reg;
output [32-1:0] a_readdataout;
input [32-1:0] c_writedatain;
input c_we;

output [32-1:0] vl;
output [3*8-1:0] matmul_masks;

reg [32-1:0] vl;
reg [32-1:0] matmul_masks;

ram_wrapper_5_32_32 reg_file1(
        .clk(clk),
        .resetn(resetn),
        .rden_a(1'b0),
        .rden_b(a_en),
        .address_a(c_reg[5-1:0]),
        .address_b(a_reg[5-1:0]),
        .wren_a(c_we),
        .wren_b(1'b0),
        .data_a(c_writedatain),
        .data_b(0),
        .out_a(),
        .out_b(a_readdataout)
  );

`ifdef TEST_BENCH
   initial begin
       $readmemh("vregfile_control.dat",reg_file1.dpram1.ram,'h0);
   end
`endif 

always@(posedge clk) begin
  if (!resetn) begin
    vl<=0;
    matmul_masks<=32'hffffffff;
  end
  else begin
    if (c_we) begin
      if (c_reg==0) begin
        vl<=c_writedatain;
      end 
      else if (c_reg==31) begin //a_rows
        matmul_masks[1*8-1:0*8] <= c_writedatain[8-1:0];
      end
      else if (c_reg==30) begin //a_cols, b_rows
        matmul_masks[2*8-1:1*8] <= c_writedatain[8-1:0];
      end
      else if (c_reg==29) begin //b_cols
        matmul_masks[3*8-1:2*8] <= c_writedatain[8-1:0];
      end
    end  
  end
end

endmodule


module matmul_unit_8_29_8(
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
 input [29:1] en;  //Enable for each pipestage
 input [29:1] squash;  //Squash for each pipestage
 output stall;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data;
 input [`MASK_WIDTH-1:0] validity_mask_a_rows;
 input [`MASK_WIDTH-1:0] validity_mask_a_cols;
 input [`MASK_WIDTH-1:0] validity_mask_b_rows;
 input [`MASK_WIDTH-1:0] validity_mask_b_cols;

 output [`MAT_MUL_SIZE*`DWIDTH-1:0] c_data;
 input [8-1:0] vmask;

 input    [8-1:0] in_dst;
 input                     in_dst_we;
 output [29*8-1:0] out_dst;
 output            [29-1:0] out_dst_we;
 output   [29*8-1:0] out_dst_mask;

wire [29:1] ctrl_activate;
wire [29:1] squash_activatepipe_NC;
pipe_1_28 activatepipe (
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

wire [29:1] squash_dstpipe_NC;
pipe_8_28 dstpipe (
    .d(in_dst ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[29-1:1] & {1'b1,{(29-2){~stall}}} ),
    .squash(squash_dstpipe_NC),
    .q(out_dst));

pipe_1_28 dstwepipe (
    .d(in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[29-1:1] & {1'b1,{(29-2){~stall}}} ),
    .squash(squash[29-1:1]),
    .q(out_dst_we));

wire [29:1] squash_dstmaskpipe_NC;
pipe_8_28 dstmaskpipe (
    .d(vmask ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[29-1:1] & {1'b1,{(29-2){~stall}}} ),
    .squash(squash_dstmaskpipe_NC),
    .q(out_dst_mask));
endmodule
 


//// Scalar core/////


module core (
    clk,
    resetn,
    rt_dataout, // Dummy output - use this to prevent design from being
                // synthesized away if using on-chip memory

    ibus_en,
    ibus_address,
    ibus_readdata,
    ibus_wait,

  // PETES CHANGE for tracing
  trc_addr,
  trc_data,
  trc_we,
  trc_stall,
  trc_pipestall,

    dbus_address,
    dbus_readdata,
    dbus_writedata,
    dbus_byteen,
    dbus_readdata_line,
    dbus_writedata_line,
    dbus_byteen_line,
    dbus_en,
    dbus_wren,
    dbus_cachematch,
    dbus_cachemiss,
    dbus_prefetch,
    dbus_wait

    );

/************************* IO Declarations *********************/
//`include "isa.v"

/****************************************************************************
          ISA definition file

  - The MIPS I ISA has a 6 bit opcode in the upper 6 bits.  
  - The opcode can also specify a "class".  There are two classes:
            1.  SPECIAL - look in lowest 6 bits to find operation
            2.  REGIMM - look in [20:16] to find type of branch

****************************************************************************/

/****** OPCODES - bits 31...26 *******/

parameter     OP_SPECIAL      = 6'b000000;
parameter     OP_REGIMM       = 6'b000001;
parameter     OP_J            = 6'b000010;
parameter     OP_JAL          = 6'b000011;
parameter     OP_BEQ          = 6'b000100;
parameter     OP_BNE          = 6'b000101;
parameter     OP_BLEZ         = 6'b000110;
parameter     OP_BGTZ         = 6'b000111;

parameter     OP_ADDI         = 6'b001000;
parameter     OP_ADDIU        = 6'b001001;
parameter     OP_SLTI         = 6'b001010;
parameter     OP_SLTIU        = 6'b001011;
parameter     OP_ANDI         = 6'b001100;
parameter     OP_ORI          = 6'b001101;
parameter     OP_XORI         = 6'b001110;
parameter     OP_LUI          = 6'b001111;

parameter     OP_LB           = 6'b100000;
parameter     OP_LH           = 6'b100001;
parameter     OP_LWL          = 6'b100010;
parameter     OP_LW           = 6'b100011;
parameter     OP_LBU          = 6'b100100;
parameter     OP_LHU          = 6'b100101;
parameter     OP_LWR          = 6'b100110;

// parameter     OP_SB           = 6'b101x00;
parameter     OP_SB_0           = 6'b101000;
parameter     OP_SB_1           = 6'b101100;
// parameter     OP_SH           = 6'b101x01;
parameter     OP_SH_0           = 6'b101001;
parameter     OP_SH_1           = 6'b101101;
parameter     OP_SWL          = 6'b101010;
// parameter     OP_SW           = 6'b101x11;
parameter     OP_SW_0           = 6'b101011;
parameter     OP_SW_1           = 6'b101111;
parameter     OP_SWR          = 6'b101110;

/****** FUNCTION CLASS - bits 5...0 *******/
parameter     FUNC_SLL        = 6'b000000;
parameter     FUNC_SRL        = 6'b000010;
parameter     FUNC_SRA        = 6'b000011;
parameter     FUNC_SLLV       = 6'b000100;
parameter     FUNC_SRLV       = 6'b000110;
parameter     FUNC_SRAV       = 6'b000111;

// parameter     FUNC_JR         = 6'b001xx0;
parameter     FUNC_JR_00         = 6'b001000;
parameter     FUNC_JR_01         = 6'b001010;
parameter     FUNC_JR_10         = 6'b001100;
parameter     FUNC_JR_11         = 6'b001110;
// parameter     FUNC_JALR       = 6'b001xx1;
parameter     FUNC_JALR_00       = 6'b001001;
parameter     FUNC_JALR_01       = 6'b001011;
parameter     FUNC_JALR_10       = 6'b001101;
parameter     FUNC_JALR_11       = 6'b001111;

// parameter     FUNC_MFHI       = 6'bx10x00;
parameter     FUNC_MFHI_00       = 6'b010000;
parameter     FUNC_MFHI_01       = 6'b010100;
parameter     FUNC_MFHI_10       = 6'b110000;
parameter     FUNC_MFHI_11       = 6'b110100;
// parameter     FUNC_MTHI       = 6'bx10x01;
parameter     FUNC_MTHI_00       = 6'b010001;
parameter     FUNC_MTHI_01       = 6'b010101;
parameter     FUNC_MTHI_10       = 6'b110001;
parameter     FUNC_MTHI_11       = 6'b110101;
// parameter     FUNC_MFLO       = 6'bx10x10;
parameter     FUNC_MFLO_00       = 6'b010010;
parameter     FUNC_MFLO_01       = 6'b010110;
parameter     FUNC_MFLO_10       = 6'b110010;
parameter     FUNC_MFLO_11       = 6'b110110;
// parameter     FUNC_MTLO       = 6'bx10x11;
parameter     FUNC_MTLO_00       = 6'b010011;
parameter     FUNC_MTLO_01       = 6'b010111;
parameter     FUNC_MTLO_10       = 6'b110011;
parameter     FUNC_MTLO_11       = 6'b110111;

// parameter     FUNC_MULT       = 6'bx11x00;
parameter     FUNC_MULT_00       = 6'b011000;
parameter     FUNC_MULT_01       = 6'b011100;
parameter     FUNC_MULT_10       = 6'b111000;
parameter     FUNC_MULT_11       = 6'b111100;
// parameter     FUNC_MULTU      = 6'bx11x01;
parameter     FUNC_MULTU_00      = 6'b011001;
parameter     FUNC_MULTU_01      = 6'b011101;
parameter     FUNC_MULTU_10      = 6'b111001;
parameter     FUNC_MULTU_11      = 6'b111101;
// parameter     FUNC_DIV        = 6'bx11x10;
parameter     FUNC_DIV_00        = 6'b011010;
parameter     FUNC_DIV_01        = 6'b011110;
parameter     FUNC_DIV_10        = 6'b111010;
parameter     FUNC_DIV_11        = 6'b111110;
// parameter     FUNC_DIVU       = 6'bx11x11;
parameter     FUNC_DIVU_00       = 6'b011011;
parameter     FUNC_DIVU_01       = 6'b011111;
parameter     FUNC_DIVU_10       = 6'b111011;
parameter     FUNC_DIVU_11       = 6'b111111;

parameter     FUNC_ADD        = 6'b100000;
parameter     FUNC_ADDU       = 6'b100001;
parameter     FUNC_SUB        = 6'b100010;
parameter     FUNC_SUBU       = 6'b100011;
parameter     FUNC_AND        = 6'b100100;
parameter     FUNC_OR         = 6'b100101;
parameter     FUNC_XOR        = 6'b100110;
parameter     FUNC_NOR        = 6'b100111;

parameter     FUNC_SLT        = 6'b101010;
parameter     FUNC_SLTU       = 6'b101011;

/****** REGIMM Class - bits 20...16 *******/
parameter     FUNC_BLTZ       = 1'b0;
parameter     FUNC_BGEZ       = 1'b1;

parameter     OP_COP2       = 6'b010010;
parameter     COP2_FUNC_CFC2     = 6'b111000;
parameter     COP2_FUNC_CTC2     = 6'b111010;
parameter     COP2_FUNC_MTC2     = 6'b111011;

parameter     OP_COP0       = 6'b010000;
parameter     COP0_MFC0     = 5'b00000;
parameter     COP0_MTC0     = 5'b00100;

//parameter     FUNC_BLTZAL     = 5'b10000;
//parameter     FUNC_BGEZAL     = 5'b10001;

/****** 
 * Original REGIMM class, compressed above to save decode logic
parameter     FUNC_BLTZ       = 5'b00000;
parameter     FUNC_BGEZ       = 5'b00001;
parameter     FUNC_BLTZAL     = 5'b10000;
parameter     FUNC_BGEZAL     = 5'b10001;
*/
parameter COP2_VADD           = 'b10z0000000;

parameter COP2_VADD_U         = 'b10z0000001;

parameter COP2_VSUB           = 'b10zz000010;

parameter COP2_VSUB_U         = 'b10zz000011;

parameter COP2_VMULHI         = 'b10z0000100;

parameter COP2_VMULHI_U       = 'b10z0000101;

parameter COP2_VDIV           = 'b10zz000110; //Using as matmul

parameter COP2_VBFADD         = 'b0100000001; //Using BF16 add

parameter COP2_VBFMULT        = 'b0100000010; //Using BF16 MULT

parameter COP2_VACT           = 'b0100000011; //Using ACT

parameter COP2_VTRP           = 'b0100000100; //Using ACT

parameter COP2_VDIV_U         = 'b10zz000111;

parameter COP2_VMOD           = 'b10zz001000;

parameter COP2_VMOD_U         = 'b10zz001001;

parameter COP2_VCMP_EQ        = 'b10zz001010;

parameter COP2_VCMP_NE        = 'b10zz001100;

parameter COP2_VCMP_LT        = 'b10zz001110;

parameter COP2_VCMP_U_LT      = 'b10zz001111;

parameter COP2_VCMP_LE        = 'b10zz010000;

parameter COP2_VCMP_U_LE      = 'b10zz010001;

parameter COP2_VMIN           = 'b10z0010010;

parameter COP2_VMIN_U         = 'b10z0010011;

parameter COP2_VMAX           = 'b10z0010100;

parameter COP2_VMAX_U         = 'b10z0010101;

parameter COP2_VMULLO         = 'b10z0010110;

parameter COP2_VABS           = 'b1000010111;

parameter COP2_VAND           = 'b10z0011000;

parameter COP2_VOR            = 'b10z0011001;

parameter COP2_VXOR           = 'b10z0011010;

parameter COP2_VNOR           = 'b10z0011011;

parameter COP2_VSLL           = 'b10zz011100;

parameter COP2_VSRL           = 'b10zz011101;

parameter COP2_VSRA           = 'b10zz011110;

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

parameter COP2_VSADD          = 'b10z0100010;

parameter COP2_VSADD_U        = 'b10z0100011;

parameter COP2_VSSUB          = 'b10zz100100;

parameter COP2_VSSUB_U        = 'b10zz100101;

parameter COP2_VSRR           = 'b1000100110;

parameter COP2_VSRR_U         = 'b1000100111;

parameter COP2_VSLS           = 'b1000101000;

parameter COP2_VSLS_U         = 'b1000101001;

parameter COP2_VXUMUL         = 'b10z0101010;

parameter COP2_VXUMUL_U       = 'b10z0101011;

parameter COP2_VXLMUL         = 'b10z0101100;

parameter COP2_VXLMUL_U       = 'b10z0101101;

parameter COP2_VXUMADD        = 'b10z0101110;

parameter COP2_VXUMADD_U      = 'b10z0101111;

parameter COP2_VXUMSUB        = 'b10z0110000;

parameter COP2_VXUMSUB_U      = 'b10z0110001;

parameter COP2_VXLMADD        = 'b10z0110010;

parameter COP2_VXLMADD_U      = 'b10z0110011;

parameter COP2_VXLMSUB        = 'b10z0110100;

parameter COP2_VXLMSUB_U      = 'b10z0110101;

parameter COP2_VINS_VV        = 'b1100000000;

parameter COP2_VINS_SV        = 'b1110000001;

parameter COP2_VEXT_VV        = 'b1100000010;

parameter COP2_VEXT_SV        = 'b1100000011;

parameter COP2_VEXT_U_SV      = 'b1100000100;

parameter COP2_VCOMPRESS      = 'b1100000101;

parameter COP2_VEXPAND        = 'b1100000110;

parameter COP2_VMERGE         = 'b11zz000111;

parameter COP2_VFINS          = 'b1110001000;

parameter COP2_VEXTHALF       = 'b1100001001;

parameter COP2_VHALF          = 'b1100001010;

parameter COP2_VHALFUP        = 'b1100001011;

parameter COP2_VHALFDN        = 'b1100001100;

parameter COP2_VSATVL         = 'b1100001101;

parameter COP2_VFAND          = 'b11z0001110;

parameter COP2_VFOR           = 'b11z0001111;

parameter COP2_VFXOR          = 'b11z0010000;

parameter COP2_VFNOR          = 'b11z0010001;

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

 

parameter LOG2DCACHEWIDTHBITS=`LOG2DCACHEWIDTHBITS;
parameter DCACHEWIDTHBITS=2**LOG2DCACHEWIDTHBITS;

input              clk;
input              resetn;
output [31:0]      rt_dataout;

output        ibus_en;             // Instruction bus signals
output [31:0] ibus_address;
input  [31:0] ibus_readdata;
input         ibus_wait;

output [31:0] dbus_address;    // Data bus signals
input  [31:0] dbus_readdata;
input  [DCACHEWIDTHBITS-1:0] dbus_readdata_line;
output [31:0] dbus_writedata;
output [3:0]  dbus_byteen;
output [DCACHEWIDTHBITS-1:0] dbus_writedata_line;
output [DCACHEWIDTHBITS/8-1:0]  dbus_byteen_line;
output        dbus_en;
output        dbus_wren;
input         dbus_cachematch;
input         dbus_cachemiss;
output [31:0] dbus_prefetch;
input         dbus_wait;

// PETES CHANGE for tracing
output  [ 4 : 0 ]   trc_addr;
output  [ 31 : 0 ]  trc_data;
output              trc_we;
input               trc_stall;
output              trc_pipestall;

//`include "isa.v"
//`include "visa.v"

    wire [31:0] p_dbus_address;    // Processor's data bus signals
    wire [31:0] p_dbus_writedata;
    wire [3:0]  p_dbus_byteen;
    wire [DCACHEWIDTHBITS/8-1:0]  p_dbus_byteen_line;
    wire        p_dbus_en;
    wire        p_dbus_wren;
    wire  [31:0] p_dbus_readdata;
    wire         p_dbus_wait;

    wire [31:0] v_dbus_address;    // VPU's data bus signals
    wire [DCACHEWIDTHBITS-1:0] v_dbus_writedata;
    wire [DCACHEWIDTHBITS/8-1:0]  v_dbus_byteen;
    wire        v_dbus_en;
    wire        v_dbus_wren;
    wire  [DCACHEWIDTHBITS-1:0] v_dbus_readdata;
    wire [31:0] v_dbus_prefetch;
    wire         v_dbus_wait;

    wire cop2_fromcop2_wait;
    wire cop2_fromcop2_en;
    wire [31:0] cop2_fromcop2;
    wire cop2_tocop2_wait;
    wire cop2_tocop2_en;
    wire [31:0] cop2_tocop2;

    wire vpu_stalled;
    wire vpu_has_memop;
    reg ibus_en_r;
    wire instr_en;
    wire ifetch_bus_wait;
    wire ifetch_squashn;

    reg is_cop2;              //Instr is a coprocessor 2 instr
    reg is_scalar_cop2;       //Instr executes both scalar and vpu
    reg is_vec_cop2;          //Instr executes only in vpu
    reg is_scalar_memop;      //Scalar memory operation 
    reg is_vec_memop;         //Vector memory operation 

    wire [31:0] ibus_ecause;
    wire [31:0] dbus_ecause;
    wire [31:0] device_ecause;
    wire [31:0] badvaddr;
    wire        badvaddr_we;

    // Do some instruction decoding to see when we're allowed to issue
    always@*
    begin
      is_cop2=0;
      is_scalar_cop2=0;
      is_vec_cop2=0;
      is_scalar_memop=0;
      is_vec_memop=0;

      case (ibus_readdata[31:26])
        OP_COP2:
        begin
          is_cop2=1;
          case(ibus_readdata[5:0])
            COP2_FUNC_CFC2: is_scalar_cop2=1;
            COP2_FUNC_CTC2: is_scalar_cop2=1;
            COP2_FUNC_MTC2: is_scalar_cop2=1;
            default: is_vec_cop2=1;
          endcase
          case ({ibus_readdata[25:22],ibus_readdata[5:0]})
            COP2_VFLD: is_vec_memop=1;
            COP2_VLD_B: is_vec_memop=1;
            COP2_VLD_H: is_vec_memop=1;
            COP2_VLD_L: is_vec_memop=1;
            COP2_VLD_W: is_vec_memop=1;
            COP2_VLD_U_B: is_vec_memop=1;
            COP2_VLD_U_H: is_vec_memop=1;
            COP2_VLD_U_W: is_vec_memop=1;
            COP2_VLDS_B: is_vec_memop=1;
            COP2_VLDS_H: is_vec_memop=1;
            COP2_VLDS_L: is_vec_memop=1;
            COP2_VLDS_U_B: is_vec_memop=1;
            COP2_VLDS_U_H: is_vec_memop=1;
            COP2_VLDS_U_W: is_vec_memop=1;
            COP2_VLDX_B: is_vec_memop=1;
            COP2_VLDX_H: is_vec_memop=1;
            COP2_VLDX_W: is_vec_memop=1;
            COP2_VLDX_L: is_vec_memop=1;
            COP2_VLDX_U_B: is_vec_memop=1;
            COP2_VLDX_U_H: is_vec_memop=1;
            COP2_VLDX_U_W: is_vec_memop=1;
            COP2_VFST: is_vec_memop=1;
            COP2_VST_B: is_vec_memop=1;
            COP2_VST_H: is_vec_memop=1;
            COP2_VST_W: is_vec_memop=1;
            COP2_VST_L: is_vec_memop=1;
            COP2_VSTS_B: is_vec_memop=1;
            COP2_VSTS_H: is_vec_memop=1;
            COP2_VSTS_W: is_vec_memop=1;
            COP2_VSTS_L: is_vec_memop=1;
            COP2_VSTX_B: is_vec_memop=1;
            COP2_VSTX_H: is_vec_memop=1;
            COP2_VSTX_W: is_vec_memop=1;
            COP2_VSTX_L: is_vec_memop=1;
            COP2_VSTXO_B: is_vec_memop=1;
            COP2_VSTXO_H: is_vec_memop=1;
            COP2_VSTXO_W: is_vec_memop=1;
            COP2_VSTXO_L: is_vec_memop=1;
            default: is_vec_memop=0;
          endcase
        end
        OP_LB: is_scalar_memop=1;
        OP_LBU: is_scalar_memop=1;
        OP_LH: is_scalar_memop=1;
        OP_LHU: is_scalar_memop=1;
        OP_LW: is_scalar_memop=1;
       // OP_SB: is_scalar_memop=1;
       // OP_SH: is_scalar_memop=1;
       // OP_SW: is_scalar_memop=1;
        6'b101000 : is_scalar_memop = 1;
        6'b101001 : is_scalar_memop = 1;
        6'b101011 : is_scalar_memop = 1;
        6'b101100 : is_scalar_memop = 1;
        6'b101101 : is_scalar_memop = 1;
        6'b101111 : is_scalar_memop = 1;
      endcase
    end

    //Stall scalar processor when:
    //  (a) instr not fetched (ibus_wait)
    //  (b) vpu is stalled (vpu_stalled) and next instr is_cop2
    //  (c) vpu has mem operation
    assign ifetch_bus_wait=ibus_wait||
                            (vpu_stalled && is_cop2) || 
                            (vpu_has_memop && is_scalar_memop);

    //Submit valid instr to VPU when
    //  (a) instr is fetched (ibus_wait)
    //  (b) scalar cpu is not staled (ibus_en)
    //  (c) scalar cpu isn't squashing the instruction (ifetch_squashn)
    //       - Do we need (c) now that vec insns not allowed in delay slot?
    //  (d) vpu has a mem op (vpu_has_memop) in which case scalar is stalling
    //  Note that a vec_memop will not be issued when a scalar mem_op is in
    //  flight because the pipe will stall right after issuing the scalar memop
    assign instr_en=ibus_en&ifetch_squashn&~ibus_wait&
                        ~(vpu_has_memop && is_scalar_memop);


    /*********************** SPREE scalar MIPS processor ********************
    * This processor was generated by SPREE which automatically produces the
    * system module that implements your described processor
    ************************************************************************/

    system p
      (
      .clk     (clk),
      .resetn (resetn),
      .boot_iaddr(0),
      .boot_idata(0),
      .boot_iwe(0),
      .boot_daddr(0),
      .boot_ddata(0),
      .boot_dwe(0),
      .ifetch_bus_ecause(ibus_ecause),
      .ifetch_bus_squashn(ifetch_squashn),
      .ifetch_bus_address(ibus_address),
      .ifetch_bus_en(ibus_en),
      .ifetch_bus_readdata(ibus_readdata),
      .ifetch_bus_wait(ifetch_bus_wait),
      .data_mem_bus_ecause(dbus_ecause),
      .data_mem_bus_address(p_dbus_address),
      .data_mem_bus_en(p_dbus_en),
      .data_mem_bus_we(p_dbus_wren),
      .data_mem_bus_byteen(p_dbus_byteen),
      .data_mem_bus_writedata(p_dbus_writedata),
      .data_mem_bus_readdata(p_dbus_readdata),
      .data_mem_bus_wait(p_dbus_wait),
      .cop2_fromcop2_wait(cop2_fromcop2_wait),
      .cop2_fromcop2_en(cop2_fromcop2_en),
      .cop2_fromcop2(cop2_fromcop2),
      .cop2_tocop2_wait(cop2_tocop2_wait),
      .cop2_tocop2_en(cop2_tocop2_en),
      .cop2_tocop2(cop2_tocop2),
      .cop0_badvaddr_we(badvaddr_we),
      .cop0_badvaddr_in(badvaddr),
      .cop0_ext_cause_in(device_ecause),
      // PETES CHANGE for tracing
      .trc_addr(trc_addr),
      .trc_data(trc_data),
      .trc_we(trc_we),
      .trc_stall(trc_stall),
      .trc_pipestall(trc_pipestall),

      . nop10_q (rt_dataout)
      );

    always@(posedge clk)
      if(!resetn)
        ibus_en_r<=0;
      else if(~ibus_en_r || ~vpu_stalled)
        ibus_en_r<=ibus_en;

    /********************** Exception processing *********************/
    assign ibus_ecause=0;     //This is for instruction fetching exceptions
    assign dbus_ecause=0;     //This is for data access exceptions
    assign device_ecause=0;   //This is for external device interrupts

    //Register exception to create one pulsed write to badvaddr
    reg ibus_exception_r;
    reg dbus_exception_r;
    always@(posedge clk)
    begin
      ibus_exception_r<=(ibus_ecause!=0);
      dbus_exception_r<=(dbus_ecause!=0);
    end

    assign badvaddr=(ibus_ecause!=0) ? ibus_address : dbus_address;
    assign badvaddr_we=(ibus_ecause!=0  && !ibus_exception_r) ||
      (dbus_ecause!=0  && !dbus_exception_r);
    /********************** /Exception processing *********************/

    vpu_7_7 v(
      .clk(clk),
      .resetn(resetn),

      // Instruction interface
      .instr(ibus_readdata),
      .instr_en(instr_en), // instr is valid and available
      .instr_wait(vpu_stalled),   // if high says vpu is not ready to receive

      .has_memop(vpu_has_memop),
      .mulshift_result(),

      // For mtc2/ctc2 instructions
      .scalar_in(cop2_tocop2),
      .scalar_in_en(cop2_tocop2_en),
      .scalar_in_wait(cop2_tocop2_wait),

      // For cfc2 instructions
      .scalar_out(cop2_fromcop2),
      .scalar_out_en(cop2_fromcop2_en),
      .scalar_out_wait(cop2_fromcop2_wait),

      // Data memory interface

      .dbus_address(v_dbus_address),
      .dbus_en(v_dbus_en),
      .dbus_we(v_dbus_wren),
      .dbus_byteen(v_dbus_byteen),
      .dbus_writedata(v_dbus_writedata),
      .dbus_readdata(v_dbus_readdata),
      .dbus_cachematch(dbus_cachematch),
      .dbus_cachemiss(dbus_cachemiss),
      .dbus_prefetch(v_dbus_prefetch),
      .dbus_wait(v_dbus_wait)
    );


  /********* Arbitrate between scalar SPREE and vector coprocessor *********/

  assign p_dbus_byteen_line=(p_dbus_byteen<<
    {p_dbus_address[LOG2DCACHEWIDTHBITS-3-1:2],2'b0});

  // Vector processor should take priority since it's request would have 
  // have been issued before the scalar's (since it has a deeper pipeline)

  assign dbus_address= (v_dbus_en) ? v_dbus_address : p_dbus_address;
  assign dbus_writedata= p_dbus_writedata;
  assign dbus_byteen=  p_dbus_byteen;
  assign dbus_writedata_line= (v_dbus_en) ? v_dbus_writedata : {DCACHEWIDTHBITS/32{p_dbus_writedata}};
  assign dbus_byteen_line= (v_dbus_en) ? v_dbus_byteen : p_dbus_byteen_line;
  assign dbus_wren= (v_dbus_en) ? v_dbus_wren : p_dbus_wren;
  assign dbus_en=p_dbus_en || v_dbus_en;
  assign dbus_prefetch= v_dbus_prefetch;

  assign p_dbus_readdata=dbus_readdata;
  assign v_dbus_readdata=dbus_readdata_line;
  //Loads/stores need to wait for vpu to finish with theirs - hence vpu_stalled
  assign p_dbus_wait=p_dbus_en&dbus_wait;
  assign v_dbus_wait=v_dbus_en&dbus_wait;

endmodule

//////  system ////////
module system ( 
	clk,
	resetn,
	boot_iaddr,
	boot_idata,
	boot_iwe,
	boot_daddr,
	boot_ddata,
	boot_dwe,
	ifetch_bus_ecause,
	ifetch_bus_squashn,
	ifetch_bus_address,
	ifetch_bus_en,
	ifetch_bus_readdata,
	ifetch_bus_wait,
	data_mem_bus_ecause,
	data_mem_bus_address,
	data_mem_bus_en,
	data_mem_bus_we,
	data_mem_bus_byteen,
	data_mem_bus_writedata,
	data_mem_bus_readdata,
	data_mem_bus_wait,
	cop2_fromcop2_wait,
	cop2_fromcop2_en,
	cop2_fromcop2,
	cop2_tocop2_wait,
	cop2_tocop2_en,
	cop2_tocop2,
	cop0_badvaddr_we,
	cop0_badvaddr_in,
	cop0_ext_cause_in,

  // PETES CHANGE for tracing
  trc_addr,
  trc_data,
  trc_we,
  trc_stall,
  trc_pipestall,

	nop10_q
	);

/************************* IO Declarations *********************/
//`include "isa.v"

/****************************************************************************
          ISA definition file

  - The MIPS I ISA has a 6 bit opcode in the upper 6 bits.  
  - The opcode can also specify a "class".  There are two classes:
            1.  SPECIAL - look in lowest 6 bits to find operation
            2.  REGIMM - look in [20:16] to find type of branch

****************************************************************************/

/****** OPCODES - bits 31...26 *******/

parameter     OP_SPECIAL      = 6'b000000;
parameter     OP_REGIMM       = 6'b000001;
parameter     OP_J            = 6'b000010;
parameter     OP_JAL          = 6'b000011;
parameter     OP_BEQ          = 6'b000100;
parameter     OP_BNE          = 6'b000101;
parameter     OP_BLEZ         = 6'b000110;
parameter     OP_BGTZ         = 6'b000111;

parameter     OP_ADDI         = 6'b001000;
parameter     OP_ADDIU        = 6'b001001;
parameter     OP_SLTI         = 6'b001010;
parameter     OP_SLTIU        = 6'b001011;
parameter     OP_ANDI         = 6'b001100;
parameter     OP_ORI          = 6'b001101;
parameter     OP_XORI         = 6'b001110;
parameter     OP_LUI          = 6'b001111;

parameter     OP_LB           = 6'b100000;
parameter     OP_LH           = 6'b100001;
parameter     OP_LWL          = 6'b100010;
parameter     OP_LW           = 6'b100011;
parameter     OP_LBU          = 6'b100100;
parameter     OP_LHU          = 6'b100101;
parameter     OP_LWR          = 6'b100110;

// parameter     OP_SB           = 6'b101x00;
parameter     OP_SB_0           = 6'b101000;
parameter     OP_SB_1           = 6'b101100;
// parameter     OP_SH           = 6'b101x01;
parameter     OP_SH_0           = 6'b101001;
parameter     OP_SH_1           = 6'b101101;
parameter     OP_SWL          = 6'b101010;
// parameter     OP_SW           = 6'b101x11;
parameter     OP_SW_0           = 6'b101011;
parameter     OP_SW_1           = 6'b101111;
parameter     OP_SWR          = 6'b101110;

/****** FUNCTION CLASS - bits 5...0 *******/
parameter     FUNC_SLL        = 6'b000000;
parameter     FUNC_SRL        = 6'b000010;
parameter     FUNC_SRA        = 6'b000011;
parameter     FUNC_SLLV       = 6'b000100;
parameter     FUNC_SRLV       = 6'b000110;
parameter     FUNC_SRAV       = 6'b000111;

// parameter     FUNC_JR         = 6'b001xx0;
parameter     FUNC_JR_00         = 6'b001000;
parameter     FUNC_JR_01         = 6'b001010;
parameter     FUNC_JR_10         = 6'b001100;
parameter     FUNC_JR_11         = 6'b001110;
// parameter     FUNC_JALR       = 6'b001xx1;
parameter     FUNC_JALR_00       = 6'b001001;
parameter     FUNC_JALR_01       = 6'b001011;
parameter     FUNC_JALR_10       = 6'b001101;
parameter     FUNC_JALR_11       = 6'b001111;

// parameter     FUNC_MFHI       = 6'bx10x00;
parameter     FUNC_MFHI_00       = 6'b010000;
parameter     FUNC_MFHI_01       = 6'b010100;
parameter     FUNC_MFHI_10       = 6'b110000;
parameter     FUNC_MFHI_11       = 6'b110100;
// parameter     FUNC_MTHI       = 6'bx10x01;
parameter     FUNC_MTHI_00       = 6'b010001;
parameter     FUNC_MTHI_01       = 6'b010101;
parameter     FUNC_MTHI_10       = 6'b110001;
parameter     FUNC_MTHI_11       = 6'b110101;
// parameter     FUNC_MFLO       = 6'bx10x10;
parameter     FUNC_MFLO_00       = 6'b010010;
parameter     FUNC_MFLO_01       = 6'b010110;
parameter     FUNC_MFLO_10       = 6'b110010;
parameter     FUNC_MFLO_11       = 6'b110110;
// parameter     FUNC_MTLO       = 6'bx10x11;
parameter     FUNC_MTLO_00       = 6'b010011;
parameter     FUNC_MTLO_01       = 6'b010111;
parameter     FUNC_MTLO_10       = 6'b110011;
parameter     FUNC_MTLO_11       = 6'b110111;

// parameter     FUNC_MULT       = 6'bx11x00;
parameter     FUNC_MULT_00       = 6'b011000;
parameter     FUNC_MULT_01       = 6'b011100;
parameter     FUNC_MULT_10       = 6'b111000;
parameter     FUNC_MULT_11       = 6'b111100;
// parameter     FUNC_MULTU      = 6'bx11x01;
parameter     FUNC_MULTU_00      = 6'b011001;
parameter     FUNC_MULTU_01      = 6'b011101;
parameter     FUNC_MULTU_10      = 6'b111001;
parameter     FUNC_MULTU_11      = 6'b111101;
// parameter     FUNC_DIV        = 6'bx11x10;
parameter     FUNC_DIV_00        = 6'b011010;
parameter     FUNC_DIV_01        = 6'b011110;
parameter     FUNC_DIV_10        = 6'b111010;
parameter     FUNC_DIV_11        = 6'b111110;
// parameter     FUNC_DIVU       = 6'bx11x11;
parameter     FUNC_DIVU_00       = 6'b011011;
parameter     FUNC_DIVU_01       = 6'b011111;
parameter     FUNC_DIVU_10       = 6'b111011;
parameter     FUNC_DIVU_11       = 6'b111111;

parameter     FUNC_ADD        = 6'b100000;
parameter     FUNC_ADDU       = 6'b100001;
parameter     FUNC_SUB        = 6'b100010;
parameter     FUNC_SUBU       = 6'b100011;
parameter     FUNC_AND        = 6'b100100;
parameter     FUNC_OR         = 6'b100101;
parameter     FUNC_XOR        = 6'b100110;
parameter     FUNC_NOR        = 6'b100111;

parameter     FUNC_SLT        = 6'b101010;
parameter     FUNC_SLTU       = 6'b101011;

/****** REGIMM Class - bits 20...16 *******/
parameter     FUNC_BLTZ       = 1'b0;
parameter     FUNC_BGEZ       = 1'b1;

parameter     OP_COP2       = 6'b010010;
parameter     COP2_FUNC_CFC2     = 6'b111000;
parameter     COP2_FUNC_CTC2     = 6'b111010;
parameter     COP2_FUNC_MTC2     = 6'b111011;

parameter     OP_COP0       = 6'b010000;
parameter     COP0_MFC0     = 5'b00000;
parameter     COP0_MTC0     = 5'b00100;

//parameter     FUNC_BLTZAL     = 5'b10000;
//parameter     FUNC_BGEZAL     = 5'b10001;

/****** 
 * Original REGIMM class, compressed above to save decode logic
parameter     FUNC_BLTZ       = 5'b00000;
parameter     FUNC_BGEZ       = 5'b00001;
parameter     FUNC_BLTZAL     = 5'b10000;
parameter     FUNC_BGEZAL     = 5'b10001;
*/


input clk;
input resetn;
input [31:0] boot_iaddr;
input [31:0] boot_idata;
input boot_iwe;
input [31:0] boot_daddr;
input [31:0] boot_ddata;
input boot_dwe;
input	[ 31 : 0 ]	ifetch_bus_ecause;
output	ifetch_bus_squashn;
output	[ 31 : 0 ]	ifetch_bus_address;
output	ifetch_bus_en;
input	[ 31 : 0 ]	ifetch_bus_readdata;
input	ifetch_bus_wait;
input	[ 31 : 0 ]	data_mem_bus_ecause;
output	[ 31 : 0 ]	data_mem_bus_address;
output	data_mem_bus_en;
output	data_mem_bus_we;
output	[ 3 : 0 ]	data_mem_bus_byteen;
output	[ 31 : 0 ]	data_mem_bus_writedata;
input	[ 31 : 0 ]	data_mem_bus_readdata;
input	data_mem_bus_wait;
output	cop2_fromcop2_wait;
input	cop2_fromcop2_en;
input	[ 31 : 0 ]	cop2_fromcop2;
input	cop2_tocop2_wait;
output	cop2_tocop2_en;
output	[ 31 : 0 ]	cop2_tocop2;
input	cop0_badvaddr_we;
input	[ 31 : 0 ]	cop0_badvaddr_in;
input	[ 31 : 0 ]	cop0_ext_cause_in;
output [31:0] nop10_q;

// PETES CHANGE for tracing
output  [ 4 : 0 ]   trc_addr;
output  [ 31 : 0 ]  trc_data;
output              trc_we;
input               trc_stall;
output              trc_pipestall;


/*********************** Signal Declarations *******************/
wire	branch_mispred;
wire	stall_2nd_delayslot;
wire	has_delayslot;
wire	haz_zeroer0_q_pipereg5_q;
wire	haz_zeroer_q_pipereg5_q;
		// Datapath signals declarations
wire	addersub_result_slt;
wire	[ 31 : 0 ]	addersub_result;
wire	[ 31 : 0 ]	logic_unit_result;
wire	[ 31 : 0 ]	ifetch_pc_out;
wire	[ 31 : 0 ]	ifetch_instr;
wire	[ 5 : 0 ]	ifetch_opcode;
wire	[ 5 : 0 ]	ifetch_func;
wire	[ 4 : 0 ]	ifetch_rs;
wire	[ 4 : 0 ]	ifetch_rt;
wire	[ 4 : 0 ]	ifetch_rd;
wire	[ 25 : 0 ]	ifetch_instr_index;
wire	[ 15 : 0 ]	ifetch_offset;
wire	[ 4 : 0 ]	ifetch_sa;
wire	[ 31 : 0 ]	ifetch_next_pc;
wire	ifetch_predict_result;
wire	[ 31 : 0 ]	ifetch_ecause;
wire	[ 31 : 0 ]	ifetch_epc;
wire	[ 31 : 0 ]	mul_shift_result;
wire	[ 31 : 0 ]	mul_lo;
wire	[ 31 : 0 ]	mul_hi;
wire	ctrl_mul_stalled;
wire	[ 31 : 0 ]	div_remainder;
wire	[ 31 : 0 ]	div_quotient;
wire	ctrl_div_stalled;
wire	[ 31 : 0 ]	data_mem_d_loadresult;
wire	[ 31 : 0 ]	data_mem_ecause;
wire	ctrl_data_mem_stalled;
wire	[ 31 : 0 ]	reg_file_b_readdataout;
wire	[ 31 : 0 ]	reg_file_a_readdataout;
wire	[ 31 : 0 ]	pcadder_result;
wire	[ 31 : 0 ]	signext16_out;
wire	[ 31 : 0 ]	merge26lo_out;
wire	branchresolve_eqz;
wire	branchresolve_gez;
wire	branchresolve_gtz;
wire	branchresolve_lez;
wire	branchresolve_ltz;
wire	branchresolve_ne;
wire	branchresolve_eq;
wire	[ 31 : 0 ]	lo_reg_q;
wire	[ 31 : 0 ]	hi_reg_q;
wire	[ 31 : 0 ]	const11_out;
wire	[ 31 : 0 ]	const12_out;
wire	[ 31 : 0 ]	const_out;
wire	[ 31 : 0 ]	pipereg_q;
wire	[ 4 : 0 ]	pipereg5_q;
wire	[ 4 : 0 ]	pipereg2_q;
wire	[ 31 : 0 ]	pipereg6_q;
wire	[ 31 : 0 ]	pipereg26_q;
wire	[ 31 : 0 ]	pipereg8_q;
wire	pipereg7_q;
wire	[ 31 : 0 ]	fakedelay_q;
wire	[ 31 : 0 ]	pipereg27_q;
wire	[ 31 : 0 ]	pipereg28_q;
wire	[ 31 : 0 ]	pipereg29_q;
wire	[ 31 : 0 ]	pipereg30_q;
wire	[ 31 : 0 ]	nop_q;
wire	[ 31 : 0 ]	nop10_q;
wire	[ 31 : 0 ]	nop13_q;
wire	[ 31 : 0 ]	nop9_q;
wire	[ 4 : 0 ]	zeroer_q;
wire	[ 4 : 0 ]	zeroer0_q;
wire	[ 4 : 0 ]	zeroer4_q;
wire	[ 31 : 0 ]	cop2_tocpu;
wire	ctrl_cop2_stalled;
wire	[ 31 : 0 ]	cop0_status;
wire	[ 31 : 0 ]	cop0_tocpu;
wire	cop0_exception;
wire	ctrl_cop0_stalled;
wire	[ 31 : 0 ]	mux2to1_mul_opA_out;
wire	[ 31 : 0 ]	mux2to1_addersub_opA_out;
wire	[ 4 : 0 ]	mux3to1_mul_sa_out;
wire	[ 31 : 0 ]	mux2to1_hi_reg_d_out;
wire	[ 31 : 0 ]	mux2to1_lo_reg_d_out;
wire	[ 31 : 0 ]	mux9to1_nop13_d_out;
wire	[ 31 : 0 ]	mux2to1_pipereg_d_out;
wire	[ 31 : 0 ]	mux2to1_pipereg6_d_out;
wire	mux6to1_pipereg7_d_out;
wire	[ 31 : 0 ]	mux3to1_nop9_d_out;
wire	[ 4 : 0 ]	mux3to1_zeroer4_d_out;
wire	[ 5 : 0 ]	pipereg15_q;
wire	[ 4 : 0 ]	pipereg16_q;
wire	[ 5 : 0 ]	pipereg14_q;
wire	branch_detector_is_branch;
wire	[ 4 : 0 ]	pipereg17_q;
wire	[ 5 : 0 ]	pipereg19_q;
wire	[ 5 : 0 ]	pipereg18_q;
wire	[ 4 : 0 ]	pipereg20_q;
wire	[ 4 : 0 ]	pipereg21_q;
wire	pipereg22_q;
wire	pipereg23_q;
wire	[ 31 : 0 ]	mux2to1_nop_d_out;
wire	pipereg31_q;
wire	[ 31 : 0 ]	mux2to1_nop10_d_out;
wire	pipereg32_q;
wire	pipereg25_q;
/***************** Control Signals ***************/
		//Decoded Opcode signal declarations
reg	[ 2 : 0 ]	ctrl_mux6to1_pipereg7_d_sel;
reg	[ 3 : 0 ]	ctrl_mux9to1_nop13_d_sel;
reg	[ 1 : 0 ]	ctrl_mux3to1_nop9_d_sel;
reg	ctrl_mux2to1_lo_reg_d_sel;
reg	ctrl_mux2to1_hi_reg_d_sel;
reg	ctrl_mux2to1_mul_opA_sel;
reg	[ 1 : 0 ]	ctrl_mux3to1_mul_sa_sel;
reg	ctrl_mux2to1_addersub_opA_sel;
reg	ctrl_mux2to1_pipereg6_d_sel;
reg	[ 1 : 0 ]	ctrl_mux3to1_zeroer4_d_sel;
reg	ctrl_mux2to1_pipereg_d_sel;
reg	ctrl_zeroer4_en;
reg	ctrl_zeroer0_en;
reg	ctrl_zeroer_en;
reg	ctrl_ifetch_pcwrop;
reg	ctrl_ifetch_op;
reg	[ 2 : 0 ]	ctrl_addersub_op;
reg	[ 3 : 0 ]	ctrl_data_mem_op;
reg	ctrl_div_sign;
reg	[ 2 : 0 ]	ctrl_mul_op;
reg	[ 1 : 0 ]	ctrl_logic_unit_op;
		//Enable signal declarations
reg	ctrl_cop0_fromcpu_en;
reg	ctrl_cop0_tocpu_en;
reg	ctrl_cop2_fromcpu_en;
reg	ctrl_cop2_tocpu_en;
reg	ctrl_lo_reg_en;
reg	ctrl_ifetch_we;
reg	ctrl_hi_reg_en;
reg	ctrl_branchresolve_en;
reg	ctrl_reg_file_c_we;
reg	ctrl_reg_file_b_en;
reg	ctrl_reg_file_a_en;
reg	ctrl_ifetch_en;
reg	ctrl_data_mem_en;
reg	ctrl_div_en;
reg	ctrl_mul_start;
		//Other Signals
wire	squash_stage3;
wire	stall_out_stage3;
wire	squash_stage2;
wire	stall_out_stage2;
wire	ctrl_pipereg25_squashn;
wire	ctrl_pipereg8_squashn;
wire	ctrl_pipereg7_squashn;
wire	ctrl_pipereg18_squashn;
wire	ctrl_pipereg19_squashn;
wire	ctrl_pipereg20_squashn;
wire	ctrl_pipereg21_squashn;
wire	ctrl_pipereg25_resetn;
wire	ctrl_pipereg8_resetn;
wire	ctrl_pipereg7_resetn;
wire	ctrl_pipereg18_resetn;
wire	ctrl_pipereg19_resetn;
wire	ctrl_pipereg20_resetn;
wire	ctrl_pipereg21_resetn;
wire	ctrl_pipereg25_en;
wire	ctrl_pipereg8_en;
wire	ctrl_pipereg7_en;
wire	ctrl_pipereg18_en;
wire	ctrl_pipereg19_en;
wire	ctrl_pipereg20_en;
wire	ctrl_pipereg21_en;
wire	squash_stage1;
wire	stall_out_stage1;
wire	ctrl_pipereg32_squashn;
wire	ctrl_pipereg31_squashn;
wire	ctrl_pipereg30_squashn;
wire	ctrl_pipereg29_squashn;
wire	ctrl_pipereg28_squashn;
wire	ctrl_pipereg27_squashn;
wire	ctrl_pipereg26_squashn;
wire	ctrl_pipereg23_squashn;
wire	ctrl_pipereg_squashn;
wire	ctrl_pipereg5_squashn;
wire	ctrl_pipereg2_squashn;
wire	ctrl_pipereg6_squashn;
wire	ctrl_pipereg14_squashn;
wire	ctrl_pipereg15_squashn;
wire	ctrl_pipereg16_squashn;
wire	ctrl_pipereg17_squashn;
wire	ctrl_pipereg32_resetn;
wire	ctrl_pipereg31_resetn;
wire	ctrl_pipereg30_resetn;
wire	ctrl_pipereg29_resetn;
wire	ctrl_pipereg28_resetn;
wire	ctrl_pipereg27_resetn;
wire	ctrl_pipereg26_resetn;
wire	ctrl_pipereg23_resetn;
wire	ctrl_pipereg_resetn;
wire	ctrl_pipereg5_resetn;
wire	ctrl_pipereg2_resetn;
wire	ctrl_pipereg6_resetn;
wire	ctrl_pipereg14_resetn;
wire	ctrl_pipereg15_resetn;
wire	ctrl_pipereg16_resetn;
wire	ctrl_pipereg17_resetn;
wire	ctrl_pipereg32_en;
wire	ctrl_pipereg31_en;
wire	ctrl_pipereg30_en;
wire	ctrl_pipereg29_en;
wire	ctrl_pipereg28_en;
wire	ctrl_pipereg27_en;
wire	ctrl_pipereg26_en;
wire	ctrl_pipereg23_en;
wire	ctrl_pipereg_en;
wire	ctrl_pipereg5_en;
wire	ctrl_pipereg2_en;
wire	ctrl_pipereg6_en;
wire	ctrl_pipereg14_en;
wire	ctrl_pipereg15_en;
wire	ctrl_pipereg16_en;
wire	ctrl_pipereg17_en;
wire	ctrl_ifetch_squashn;
wire	ctrl_lo_reg_squashn;
wire	ctrl_hi_reg_squashn;
wire	ctrl_reg_file_c_squashn;
reg	predictme;


/****************************** Control **************************/
		//Decode Logic for Opcode and Multiplex Select signals
always@(ifetch_opcode or ifetch_func or ifetch_rt or ifetch_rs)
begin
	// 	// Initialize control opcodes to zero
	// ctrl_mux2to1_pipereg6_d_sel = 0;
	// ctrl_mux3to1_zeroer4_d_sel = 0;
	// ctrl_mux2to1_pipereg_d_sel = 0;
	// ctrl_zeroer4_en = 0;
	// ctrl_zeroer0_en = 0;
	// ctrl_zeroer_en = 0;
	
	case (ifetch_opcode)
		OP_ADDI:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_ADDIU:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_ANDI:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_BEQ:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
            ctrl_mux3to1_zeroer4_d_sel = 0;
            ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_BGTZ:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
            ctrl_mux3to1_zeroer4_d_sel = 0;
            ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_BLEZ:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
            ctrl_mux3to1_zeroer4_d_sel = 0;
            ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_BNE:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
            ctrl_mux3to1_zeroer4_d_sel = 0;
            ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_COP0:
		case (ifetch_rs)
			COP0_MFC0:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 2;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
                ctrl_zeroer_en = 0;
			end
			COP0_MTC0:
            begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
                ctrl_zeroer_en = 0;
            end
		endcase
		OP_COP2:
		case (ifetch_func)
			COP2_FUNC_CFC2:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 2;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
                ctrl_zeroer_en = 0;
			end
			COP2_FUNC_CTC2:
            begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
                ctrl_zeroer_en = 0;
            end
			COP2_FUNC_MTC2:
            begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
                ctrl_zeroer_en = 0;
            end
		endcase
		OP_J: 
        begin
			ctrl_mux2to1_pipereg6_d_sel = 0;
            ctrl_mux3to1_zeroer4_d_sel = 0;
            ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
            ctrl_zeroer0_en = 0;
            ctrl_zeroer_en = 0;
        end
		OP_JAL:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 0;
            ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
            ctrl_zeroer_en = 0;
		end
		OP_LB:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_LBU:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_LH:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_LHU:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_LUI:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
            ctrl_zeroer_en = 0;
		end
		OP_LW:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_ORI:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_REGIMM:
		case (ifetch_rt[0])
			FUNC_BGEZ:
			begin
				ctrl_mux2to1_pipereg6_d_sel = 1;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
                ctrl_zeroer0_en = 0;
				ctrl_zeroer_en = 1;
			end
			FUNC_BLTZ:
			begin
				ctrl_mux2to1_pipereg6_d_sel = 1;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
                ctrl_zeroer0_en = 0;
				ctrl_zeroer_en = 1;
			end
		endcase
		OP_SB_0:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
            ctrl_mux3to1_zeroer4_d_sel = 0;
			ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SB_1:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
            ctrl_mux3to1_zeroer4_d_sel = 0;
			ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SH_0:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
            ctrl_mux3to1_zeroer4_d_sel = 0;
			ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SH_1:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
            ctrl_mux3to1_zeroer4_d_sel = 0;
			ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SLTI:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_SLTIU:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_SPECIAL:
		case (ifetch_func)
			FUNC_ADD:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_ADDU:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_AND:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_DIV_00:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_DIV_01:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_DIV_10:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_DIV_11:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_DIVU_00:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_DIVU_01:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_DIVU_10:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_DIVU_11:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_JALR_00:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
				ctrl_zeroer_en = 1;
			end
			FUNC_JALR_01:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
				ctrl_zeroer_en = 1;
			end
			FUNC_JALR_10:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
				ctrl_zeroer_en = 1;
			end
			FUNC_JALR_11:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
				ctrl_zeroer_en = 1;
			end
			FUNC_JR_00:
            begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
                ctrl_zeroer0_en = 0;
				ctrl_zeroer_en = 1;
            end
			FUNC_JR_01:
            begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
                ctrl_zeroer0_en = 0;
				ctrl_zeroer_en = 1;
            end
			FUNC_JR_10:
            begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
                ctrl_zeroer0_en = 0;
				ctrl_zeroer_en = 1;
            end
			FUNC_JR_11:
            begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
                ctrl_zeroer0_en = 0;
				ctrl_zeroer_en = 1;
            end
			FUNC_MFHI_00:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
                ctrl_zeroer_en = 0;
			end
			FUNC_MFHI_01:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
                ctrl_zeroer_en = 0;
			end
			FUNC_MFHI_10:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
                ctrl_zeroer_en = 0;
			end
			FUNC_MFHI_11:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
                ctrl_zeroer_en = 0;
			end
			FUNC_MFLO_00:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
                ctrl_zeroer_en = 0;
			end
			FUNC_MFLO_01:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
                ctrl_zeroer_en = 0;
			end
			FUNC_MFLO_10:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
                ctrl_zeroer_en = 0;
			end
			FUNC_MFLO_11:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
                ctrl_zeroer_en = 0;
			end
			FUNC_MULT_00:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_MULT_01:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_MULT_10:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_MULT_11:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_MULTU_00:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_MULTU_01:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_MULTU_10:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_MULTU_11:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_NOR:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_OR:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SLL:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
                ctrl_zeroer_en = 0;
			end
			FUNC_SLLV:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SLT:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SLTU:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SRA:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
                ctrl_zeroer_en = 0;
			end
			FUNC_SRAV:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SRL:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
                ctrl_zeroer_en = 0;
			end
			FUNC_SRLV:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SUB:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SUBU:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_XOR:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
		endcase
		OP_SW_0:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
            ctrl_mux3to1_zeroer4_d_sel = 0;
			ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SW_1:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
            ctrl_mux3to1_zeroer4_d_sel = 0;
			ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_XORI:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
	endcase
end
		//Logic for enable signals in Pipe Stage 1
always@(ifetch_opcode or ifetch_func or ifetch_rt[0] or ifetch_rs or stall_out_stage2)
begin
	ctrl_reg_file_b_en = 1 &~stall_out_stage2;
	ctrl_reg_file_a_en = 1 &~stall_out_stage2;
	ctrl_ifetch_en = 1 &~stall_out_stage2;
end
		//Decode Logic for Opcode and Multiplex Select signals
always@(pipereg14_q or pipereg15_q or pipereg16_q or pipereg17_q)
begin
		// Initialize control opcodes to zero
	// ctrl_mux6to1_pipereg7_d_sel = 0;
	// ctrl_mux9to1_nop13_d_sel = 0;
	// ctrl_mux3to1_nop9_d_sel = 0;
	// ctrl_mux2to1_lo_reg_d_sel = 0;
	// ctrl_mux2to1_hi_reg_d_sel = 0;
	// ctrl_mux2to1_mul_opA_sel = 0;
	// ctrl_mux3to1_mul_sa_sel = 0;
	// ctrl_mux2to1_addersub_opA_sel = 0;
	// ctrl_ifetch_pcwrop = 0;
	// ctrl_addersub_op = 0;
	// ctrl_data_mem_op = 0;
	// ctrl_div_sign = 0;
	// ctrl_mul_op = 0;
	// ctrl_logic_unit_op = 0;
	
	case (pipereg14_q)
		OP_ADDI:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 6;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_ADDIU:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 6;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 1;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_ANDI:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 4;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 0;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_BEQ:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 5;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 0;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 0;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_BGTZ:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 0;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 0;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_BLEZ:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 3;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 0;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 0;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_BNE:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 4;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 0;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 0;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_COP0:
		case (pipereg17_q)
			COP0_MFC0:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 7;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
		endcase
		OP_COP2:
		case (pipereg15_q)
			COP2_FUNC_CFC2:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 8;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;	
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;	
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;	
			end
		endcase
		OP_J:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 0;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 1;
			ctrl_addersub_op = 0;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_JAL:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 6;
			ctrl_mux3to1_nop9_d_sel = 0;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 1;
			ctrl_ifetch_pcwrop = 1;
			ctrl_addersub_op = 1;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_LB:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 7;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_LBU:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 3;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_LH:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 5;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_LHU:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 1;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_LUI:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 3;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 1;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 0;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_LW:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_ORI:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 4;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 0;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 1;
		end
		OP_REGIMM:
		case (pipereg16_q[0])
			FUNC_BGEZ:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 1;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_BLTZ:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 2;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
		endcase
		OP_SB_0:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 11;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_SB_1:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 11;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_SH_0:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 9;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_SH_1:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 9;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_SLTI:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 5;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 6;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_SLTIU:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 5;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 4;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_SPECIAL:
		case (pipereg15_q)
			FUNC_ADD:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 3;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_ADDU:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 1;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_AND:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_DIV_00:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 1;
				ctrl_mux2to1_hi_reg_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 1;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_DIV_01:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 1;
				ctrl_mux2to1_hi_reg_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 1;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_DIV_10:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 1;
				ctrl_mux2to1_hi_reg_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 1;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_DIV_11:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 1;
				ctrl_mux2to1_hi_reg_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 1;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_DIVU_00:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 1;
				ctrl_mux2to1_hi_reg_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_DIVU_01:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 1;
				ctrl_mux2to1_hi_reg_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_DIVU_10:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 1;
				ctrl_mux2to1_hi_reg_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_DIVU_11:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 1;
				ctrl_mux2to1_hi_reg_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_JALR_00:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 1;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 1;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_JALR_01:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 1;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 1;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_JALR_10:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 1;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 1;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_JALR_11:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 1;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 1;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MFHI_00:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 1;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MFHI_01:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 1;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MFHI_10:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 1;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MFHI_11:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 1;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MFLO_00:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MFLO_01:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MFLO_10:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MFLO_11:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MULT_00:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 1;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 6;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MULT_01:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 1;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 6;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MULT_10:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 1;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 6;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MULT_11:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 1;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 6;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MULTU_00:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 1;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 4;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MULTU_01:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 1;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 4;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MULTU_10:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 1;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 4;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MULTU_11:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 1;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 4;
				ctrl_logic_unit_op = 0;
			end
			FUNC_NOR:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 3;
			end
			FUNC_OR:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 1;
			end
			FUNC_SLL:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SLLV:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 2;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SLT:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 5;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 6;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SLTU:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 5;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 4;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SRA:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 3;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SRAV:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 2;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 3;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SRL:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 1;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SRLV:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 2;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 1;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SUB:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SUBU:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 2;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_XOR:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 2;
			end
		endcase
		OP_SW_0:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 8;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_SW_1:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 8;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_XORI:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 4;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 0;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 2;
		end
	endcase
end
		//Logic for enable signals in Pipe Stage 2
always@(pipereg14_q or pipereg15_q or pipereg16_q[0] or pipereg17_q or stall_out_stage3 or ctrl_mul_stalled or ctrl_data_mem_stalled or ctrl_div_stalled or ctrl_cop2_stalled or ctrl_cop0_stalled)
begin
//	ctrl_cop0_fromcpu_en = 0;
//	ctrl_cop0_tocpu_en = 0;
//	ctrl_cop2_fromcpu_en = 0;
//	ctrl_cop2_tocpu_en = 0;
//	ctrl_lo_reg_en = 0;
//	ctrl_hi_reg_en = 0;
//	ctrl_branchresolve_en = 0;
//	ctrl_reg_file_c_we = 0;
//	ctrl_data_mem_en = 0;
//	ctrl_div_en = 0;
//	ctrl_mul_start = 0;
	case (pipereg14_q)
		OP_ADDI:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_ADDIU:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_ANDI:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_BEQ:
        begin
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_BGTZ:
        begin
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_BLEZ:
        begin
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_BNE:
        begin
			ctrl_branchresolve_en = 1 &(~ctrl_cop0_stalled)&(~ctrl_cop2_stalled)&(~ctrl_div_stalled)&(~ctrl_data_mem_stalled)&(~ctrl_mul_stalled)&(~stall_out_stage3);
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_COP0:
		case (pipereg17_q)
			COP0_MFC0:
			begin
				ctrl_cop0_tocpu_en = 1 &~stall_out_stage3;
				ctrl_reg_file_c_we = 1 &(~ctrl_cop0_stalled)&(~ctrl_cop2_stalled)&(~ctrl_div_stalled)&(~ctrl_data_mem_stalled)&(~ctrl_mul_stalled)&(~stall_out_stage3);
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
			end
			COP0_MTC0:
			begin
				ctrl_cop0_fromcpu_en = 1 &(~ctrl_cop2_stalled)&(~ctrl_div_stalled)&(~ctrl_data_mem_stalled)&(~ctrl_mul_stalled)&(~stall_out_stage3);
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
			end
		endcase
		OP_COP2:
		case (pipereg15_q)
			COP2_FUNC_CFC2:
			begin
				ctrl_cop2_tocpu_en = 1 &~stall_out_stage3;
				ctrl_reg_file_c_we = 1 &(~ctrl_cop0_stalled)&(~ctrl_cop2_stalled)&(~ctrl_div_stalled)&(~ctrl_data_mem_stalled)&(~ctrl_mul_stalled)&(~stall_out_stage3);
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
			end
			COP2_FUNC_CTC2:
            begin
				ctrl_cop2_fromcpu_en = 1 &~ctrl_cop0_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			COP2_FUNC_MTC2:
            begin
				ctrl_cop2_fromcpu_en = 1 &~ctrl_cop0_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
		endcase
		OP_JAL:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_LB:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
		end
		OP_LBU:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
		end
		OP_LH:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
		end
		OP_LHU:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
		end
		OP_LUI:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_mul_start = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
		end
		OP_LW:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
		end
		OP_ORI:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_REGIMM:
		case (pipereg16_q[0])
			FUNC_BGEZ:
            begin
				ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_BLTZ:
            begin
				ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
		endcase
		OP_SB_0:
        begin
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_SB_1:
        begin
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_SH_0:
        begin
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_SH_1:
        begin
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_SLTI:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_SLTIU:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_SPECIAL:
		case (pipereg15_q)
			FUNC_ADD:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_ADDU:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_AND:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_DIV_00:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_div_en = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_mul_start = 0;
			end
			FUNC_DIV_01:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_div_en = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_mul_start = 0;
			end
			FUNC_DIV_10:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_div_en = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_mul_start = 0;
			end
			FUNC_DIV_11:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_div_en = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_mul_start = 0;
			end
			FUNC_DIVU_00:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_div_en = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_mul_start = 0;
			end
			FUNC_DIVU_01:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_div_en = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_mul_start = 0;
			end
			FUNC_DIVU_10:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_div_en = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_mul_start = 0;
			end
			FUNC_DIVU_11:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_div_en = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_mul_start = 0;
			end
			FUNC_JALR_00:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_JALR_01:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_JALR_10:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_JALR_11:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_MFHI_00:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_MFHI_01:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_MFHI_10:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_MFHI_11:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_MFLO_00:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_MFLO_01:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_MFLO_10:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_MFLO_11:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_MULT_00:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_MULT_01:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_MULT_10:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_MULT_11:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_MULTU_00:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_MULTU_01:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_MULTU_10:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_MULTU_11:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_NOR:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_OR:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_SLL:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_SLLV:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_SLT:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_SLTU:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_SRA:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_SRAV:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_SRL:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_SRLV:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_SUB:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_SUBU:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_XOR:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
		endcase
		OP_SW_0:
        begin
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_SW_1:
        begin
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_XORI:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
	endcase
end
		//Decode Logic for Opcode and Multiplex Select signals
always@(pipereg18_q or pipereg19_q or pipereg20_q or pipereg21_q)
begin
	
	case (pipereg18_q)
		OP_BEQ:
			ctrl_ifetch_op = 0;
		OP_BGTZ:
			ctrl_ifetch_op = 0;
		OP_BLEZ:
			ctrl_ifetch_op = 0;
		OP_BNE:
			ctrl_ifetch_op = 0;
		OP_REGIMM:
		case (pipereg20_q[0])
			FUNC_BGEZ:
				ctrl_ifetch_op = 0;
			FUNC_BLTZ:
				ctrl_ifetch_op = 0;
		endcase
		OP_SPECIAL:
		case (pipereg19_q)
			FUNC_JALR_00:
				ctrl_ifetch_op = 1;
			FUNC_JALR_01:
				ctrl_ifetch_op = 1;
			FUNC_JALR_10:
				ctrl_ifetch_op = 1;
			FUNC_JALR_11:
				ctrl_ifetch_op = 1;
			FUNC_JR_00:
				ctrl_ifetch_op = 1;
			FUNC_JR_01:
				ctrl_ifetch_op = 1;
			FUNC_JR_10:
				ctrl_ifetch_op = 1;
			FUNC_JR_11:
				ctrl_ifetch_op = 1;
		endcase
        default: 
            ctrl_ifetch_op = 0;
	endcase
end
		//Logic for enable signals in Pipe Stage 3
always@(pipereg18_q or pipereg19_q or pipereg20_q[0] or pipereg21_q)// or 1'b0)
begin
	
	case (pipereg18_q)
		OP_BEQ:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_BGTZ:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_BLEZ:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_BNE:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_REGIMM:
		case (pipereg20_q[0])
			FUNC_BGEZ:
				ctrl_ifetch_we = 1 &~1'b0;
			FUNC_BLTZ:
				ctrl_ifetch_we = 1 &~1'b0;
		endcase
		OP_SPECIAL:
		case (pipereg19_q)
			FUNC_JALR_00:
				ctrl_ifetch_we = 1 &~1'b0;
			FUNC_JALR_01:
				ctrl_ifetch_we = 1 &~1'b0;
			FUNC_JALR_10:
				ctrl_ifetch_we = 1 &~1'b0;
			FUNC_JALR_11:
				ctrl_ifetch_we = 1 &~1'b0;
			FUNC_JR_00:
				ctrl_ifetch_we = 1 &~1'b0;
			FUNC_JR_01:
				ctrl_ifetch_we = 1 &~1'b0;
			FUNC_JR_10:
				ctrl_ifetch_we = 1 &~1'b0;
			FUNC_JR_11:
				ctrl_ifetch_we = 1 &~1'b0;
		endcase
        default:
            ctrl_ifetch_we = 0;
	endcase
end

/********* Stall Network & PipeReg Control ********/
assign stall_out_stage1 = stall_out_stage2;
assign ctrl_pipereg17_en = ~stall_out_stage1;
assign ctrl_pipereg16_en = ~stall_out_stage1;
assign ctrl_pipereg15_en = ~stall_out_stage1;
assign ctrl_pipereg14_en = ~stall_out_stage1;
assign ctrl_pipereg6_en = ~stall_out_stage1;
assign ctrl_pipereg2_en = ~stall_out_stage1;
assign ctrl_pipereg5_en = ~stall_out_stage1;
assign ctrl_pipereg_en = ~stall_out_stage1;
assign ctrl_pipereg23_en = ~stall_out_stage1;
assign ctrl_pipereg26_en = ~stall_out_stage1;
assign ctrl_pipereg27_en = ~stall_out_stage1;
assign ctrl_pipereg28_en = ~stall_out_stage1;
assign ctrl_pipereg29_en = ~stall_out_stage1;
assign ctrl_pipereg30_en = ~stall_out_stage1;
assign ctrl_pipereg31_en = ~stall_out_stage1;
assign ctrl_pipereg32_en = ~stall_out_stage1;
assign stall_out_stage2 = stall_out_stage3|ctrl_cop0_stalled|ctrl_cop2_stalled|ctrl_div_stalled|ctrl_data_mem_stalled|ctrl_mul_stalled;
assign ctrl_pipereg21_en = ~stall_out_stage2;
assign ctrl_pipereg20_en = ~stall_out_stage2;
assign ctrl_pipereg19_en = ~stall_out_stage2;
assign ctrl_pipereg18_en = ~stall_out_stage2;
assign ctrl_pipereg7_en = ~stall_out_stage2;
assign ctrl_pipereg8_en = ~stall_out_stage2;
assign ctrl_pipereg25_en = ~stall_out_stage2;
assign stall_out_stage3 = 1'b0;
assign branch_mispred = (!ifetch_predict_result);
assign stall_2nd_delayslot = branch_detector_is_branch&has_delayslot;
assign has_delayslot = pipereg22_q;

		//Identify branches that will be predicted
always@(ifetch_opcode or ifetch_func or ifetch_rt[0] or ifetch_rs)
begin
	case (ifetch_opcode)
		OP_BEQ:
			predictme=1;
		OP_BGTZ:
			predictme=1;
		OP_BLEZ:
			predictme=1;
		OP_BNE:
			predictme=1;
		OP_J:
			predictme=1;
		OP_JAL:
			predictme=1;
		OP_REGIMM:
		case (ifetch_rt[0])
			FUNC_BGEZ:
				predictme=1;
			FUNC_BLTZ:
				predictme=1;
		endcase
        default:
            predictme=0;
	endcase
end

assign squash_stage1 = ((stall_out_stage1&~stall_out_stage2))|~resetn;
assign ctrl_pipereg17_resetn = ~squash_stage1;
assign ctrl_pipereg16_resetn = ~squash_stage1;
assign ctrl_pipereg15_resetn = ~squash_stage1;
assign ctrl_pipereg14_resetn = ~squash_stage1;
assign ctrl_pipereg6_resetn = ~squash_stage1;
assign ctrl_pipereg2_resetn = ~squash_stage1;
assign ctrl_pipereg5_resetn = ~squash_stage1;
assign ctrl_pipereg_resetn = ~squash_stage1;
assign ctrl_pipereg23_resetn = ~squash_stage1;
assign ctrl_pipereg26_resetn = ~squash_stage1;
assign ctrl_pipereg27_resetn = ~squash_stage1;
assign ctrl_pipereg28_resetn = ~squash_stage1;
assign ctrl_pipereg29_resetn = ~squash_stage1;
assign ctrl_pipereg30_resetn = ~squash_stage1;
assign ctrl_pipereg31_resetn = ~squash_stage1;
assign ctrl_pipereg32_resetn = ~squash_stage1;
assign ctrl_pipereg32_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg31_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg30_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg29_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg28_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg27_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg26_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg23_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg5_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg2_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg6_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg14_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg15_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg16_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg17_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_ifetch_squashn = ~((branch_mispred&~(pipereg22_q)) || (cop0_exception));
assign squash_stage2 = ((stall_out_stage2&~stall_out_stage3))|~resetn;
assign ctrl_pipereg21_resetn = ~squash_stage2;
assign ctrl_pipereg20_resetn = ~squash_stage2;
assign ctrl_pipereg19_resetn = ~squash_stage2;
assign ctrl_pipereg18_resetn = ~squash_stage2;
assign ctrl_pipereg7_resetn = ~squash_stage2;
assign ctrl_pipereg8_resetn = ~squash_stage2;
assign ctrl_pipereg25_resetn = ~squash_stage2;
assign ctrl_pipereg25_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg8_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg7_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg18_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg19_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg20_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg21_squashn = ~((0) || (cop0_exception));
assign ctrl_lo_reg_squashn = ~((0) || (cop0_exception));
assign ctrl_hi_reg_squashn = ~((0) || (cop0_exception));
assign ctrl_reg_file_c_squashn = ~((0) || (cop0_exception));
assign squash_stage3 = ((stall_out_stage3&~1'b0))|~resetn;

/****************************** Datapath **************************/
/******************** Hazard Detection Logic ***********************/
assign haz_zeroer0_q_pipereg5_q = (zeroer0_q==pipereg5_q) && (|zeroer0_q);
assign haz_zeroer_q_pipereg5_q = (zeroer_q==pipereg5_q) && (|zeroer_q);

/*************** DATAPATH COMPONENTS **************/
addersub_32 addersub (
	.opB(nop9_q),
	.opA(mux2to1_addersub_opA_out),
	.op(ctrl_addersub_op),
	.result_slt(addersub_result_slt),
	.result(addersub_result));
//	defparam
//		addersub.WIDTH=32;

logic_unit_32 logic_unit (
	.opB(nop9_q),
	.opA(nop_q),
	.op(ctrl_logic_unit_op),
	.result(logic_unit_result));
//	defparam
//		logic_unit.WIDTH=32;

ifetch_67108896_32_14_16384 ifetch (
	.clk(clk),
	.resetn(resetn),
	.en(ctrl_ifetch_en),
	.squashn(ctrl_ifetch_squashn),
	.we(ctrl_ifetch_we),
	.op(ctrl_ifetch_op),
	.load(pipereg7_q),
	.load_data(pipereg8_q),
	.pcwrop(ctrl_ifetch_pcwrop),
	.predict_tgt_pc(pipereg6_q),
	.predict_en(pipereg23_q),
	.predict_result_rdy(pipereg25_q),
	.predict_result(ifetch_predict_result),
	.interrupt(cop0_exception),
	.epc(ifetch_epc),
	.ecause(ifetch_ecause),
	.pc_out(ifetch_pc_out),
	.next_pc(ifetch_next_pc),
	.boot_iaddr(boot_iaddr),
	.boot_idata(boot_idata),
	.boot_iwe(boot_iwe),
	.bus_address(ifetch_bus_address),
	.bus_en(ifetch_bus_en),
	.bus_readdata(ifetch_bus_readdata),
	.bus_wait(ifetch_bus_wait),
	.bus_squashn(ifetch_bus_squashn),
	.bus_ecause(ifetch_bus_ecause),
	.opcode(ifetch_opcode),
	.rs(ifetch_rs),
	.rt(ifetch_rt),
	.rd(ifetch_rd),
	.sa(ifetch_sa),
	.offset(ifetch_offset),
	.instr_index(ifetch_instr_index),
	.func(ifetch_func),
	.instr(ifetch_instr)
	);

mul_32 mul (
	.clk(clk),
	.resetn(resetn),
	.start(ctrl_mul_start),
	.stalled(ctrl_mul_stalled),
	.dst(pipereg5_q),
	.opA(mux2to1_mul_opA_out),
	.opB(nop10_q),
	.sa(mux3to1_mul_sa_out),	
	.op(ctrl_mul_op),
	.shift_result(mul_shift_result),
	.hi(mul_hi),
	.lo(mul_lo)
	);
//	defparam
//		mul.WIDTH=32;

div_0_1_2 div (
	.en(ctrl_div_en),
	.resetn(resetn),
	.stalled(ctrl_div_stalled),
	.quotient(div_quotient),
	.remainder(div_remainder),
	.dividend(nop_q),
	.divider(nop10_q),
	.sign(ctrl_div_sign),
	.clk(clk)	
	);

wire bus_wait_temp;
assign bus_wait_temp = data_mem_bus_wait|trc_stall;
data_mem_32_32_4_16_16384 data_mem (
	.clk(clk),
	.resetn(resetn),
	.en(ctrl_data_mem_en),
	.stalled(ctrl_data_mem_stalled),
	.d_writedata(nop10_q),
	.d_address(addersub_result),
	.op(ctrl_data_mem_op),
	.d_loadresult(data_mem_d_loadresult),
	.ecause(data_mem_ecause),
	.boot_daddr(boot_daddr),
	.boot_ddata(boot_ddata),
	.boot_dwe(boot_dwe),
	.bus_address(data_mem_bus_address),
	.bus_byteen(data_mem_bus_byteen),
	.bus_we(data_mem_bus_we),
	.bus_en(data_mem_bus_en),
	.bus_writedata(data_mem_bus_writedata),
	.bus_readdata(data_mem_bus_readdata),
	//PETES CHANGE for tracing, was: .bus_wait(data_mem_bus_wait),
	.bus_wait(bus_wait_temp),
	.bus_ecause(data_mem_bus_ecause)
	);

reg_file_32_32_5 reg_file (
	.clk(clk),
	.resetn(resetn),
	.c_squashn(ctrl_reg_file_c_squashn),
	.a_reg(zeroer_q),
	.a_readdataout(reg_file_a_readdataout),
	.a_en(ctrl_reg_file_a_en),
	.b_reg(zeroer0_q),
	.b_readdataout(reg_file_b_readdataout),
	.b_en(ctrl_reg_file_b_en),
	.c_reg(pipereg5_q),
	.c_writedatain(nop13_q),
	.c_we(ctrl_reg_file_c_we)
	);

pcadder_32 pcadder (
	.offset(signext16_out),
	.pc(ifetch_pc_out),
	.result(pcadder_result));

signext16 signext16 (
	.in(ifetch_offset),
	.out(signext16_out));

merge26lo merge26lo (
	.in2(ifetch_instr_index),
	.in1(ifetch_pc_out),
	.out(merge26lo_out));

branchresolve_32 branchresolve (
	.rt(nop10_q),
	.rs(nop_q),
	.en(ctrl_branchresolve_en),
	.eqz(branchresolve_eqz),
	.gez(branchresolve_gez),
	.gtz(branchresolve_gtz),
	.lez(branchresolve_lez),
	.ltz(branchresolve_ltz),
	.ne(branchresolve_ne),
	.eq(branchresolve_eq));
//	defparam
//		branchresolve.WIDTH=32;

lo_reg_32 lo_reg (
	.clk(clk),
	.resetn(resetn),
	.d(mux2to1_lo_reg_d_out),
	.squashn(ctrl_lo_reg_squashn),
	.en(ctrl_lo_reg_en),
	.q(lo_reg_q));
//	defparam
//		lo_reg.WIDTH=32;

hi_reg_32 hi_reg (
	.clk(clk),
	.resetn(resetn),
	.d(mux2to1_hi_reg_d_out),
	.squashn(ctrl_hi_reg_squashn),
	.en(ctrl_hi_reg_en),
	.q(hi_reg_q));
//	defparam
//		hi_reg.WIDTH=32;

const_32_0 const11 (
	.out(const11_out));
//	defparam
//		const11.WIDTH=32,
//		const11.VAL=0;

const_32_16 const12 (
	.out(const12_out));
//	defparam
//		const12.WIDTH=32,
//		const12.VAL=16;

const_32_31 const (
	.out(const_out));
//	defparam
//		const.WIDTH=32,
//		const.VAL=31;

pipereg_32 pipereg (
	.clk(clk),
	.resetn(ctrl_pipereg_resetn),
	.d(mux2to1_pipereg_d_out),
	.squashn(ctrl_pipereg_squashn),
	.en(ctrl_pipereg_en),
	.q(pipereg_q));
//	defparam
//		pipereg.WIDTH=32;

pipereg_5 pipereg5 (
	.clk(clk),
	.resetn(ctrl_pipereg5_resetn),
	.d(zeroer4_q),
	.squashn(ctrl_pipereg5_squashn),
	.en(ctrl_pipereg5_en),
	.q(pipereg5_q));
//	defparam
//		pipereg5.WIDTH=5;

pipereg_5 pipereg2 (
	.clk(clk),
	.resetn(ctrl_pipereg2_resetn),
	.d(ifetch_sa),
	.squashn(ctrl_pipereg2_squashn),
	.en(ctrl_pipereg2_en),
	.q(pipereg2_q));
//	defparam
//		pipereg2.WIDTH=5;

pipereg_32 pipereg6 (
	.clk(clk),
	.resetn(ctrl_pipereg6_resetn),
	.d(mux2to1_pipereg6_d_out),
	.squashn(ctrl_pipereg6_squashn),
	.en(ctrl_pipereg6_en),
	.q(pipereg6_q));
//	defparam
//		pipereg6.WIDTH=32;

pipereg_32 pipereg26 (
	.clk(clk),
	.resetn(ctrl_pipereg26_resetn),
	.d(nop13_q),
	.squashn(ctrl_pipereg26_squashn),
	.en(ctrl_pipereg26_en),
	.q(pipereg26_q));
//	defparam
//		pipereg26.WIDTH=32;

pipereg_32 pipereg8 (
	.clk(clk),
	.resetn(ctrl_pipereg8_resetn),
	.d(nop_q),
	.squashn(ctrl_pipereg8_squashn),
	.en(ctrl_pipereg8_en),
	.q(pipereg8_q));
//	defparam
//		pipereg8.WIDTH=32;

pipereg_1 pipereg7 (
	.clk(clk),
	.resetn(ctrl_pipereg7_resetn),
	.d(mux6to1_pipereg7_d_out),
	.squashn(ctrl_pipereg7_squashn),
	.en(ctrl_pipereg7_en),
	.q(pipereg7_q));
//	defparam
//		pipereg7.WIDTH=1;

fakedelay_32 fakedelay (
	.clk(clk),
	.d(ifetch_pc_out),
	.q(fakedelay_q));
//	defparam
//		fakedelay.WIDTH=32;

pipereg_32 pipereg27 (
	.clk(clk),
	.resetn(ctrl_pipereg27_resetn),
	.d(ifetch_instr),
	.squashn(ctrl_pipereg27_squashn),
	.en(ctrl_pipereg27_en),
	.q(pipereg27_q));
//	defparam
//		pipereg27.WIDTH=32;

pipereg_32 pipereg28 (
	.clk(clk),
	.resetn(ctrl_pipereg28_resetn),
	.d(ifetch_epc),
	.squashn(ctrl_pipereg28_squashn),
	.en(ctrl_pipereg28_en),
	.q(pipereg28_q));
//	defparam
//		pipereg28.WIDTH=32;

pipereg_32 pipereg29 (
	.clk(clk),
	.resetn(ctrl_pipereg29_resetn),
	.d(ifetch_rd),
	.squashn(ctrl_pipereg29_squashn),
	.en(ctrl_pipereg29_en),
	.q(pipereg29_q));
//	defparam
//		pipereg29.WIDTH=32;

pipereg_32 pipereg30 (
	.clk(clk),
	.resetn(ctrl_pipereg30_resetn),
	.d(ifetch_ecause),
	.squashn(ctrl_pipereg30_squashn),
	.en(ctrl_pipereg30_en),
	.q(pipereg30_q));
//	defparam
//		pipereg30.WIDTH=32;

nop_32 nop (
	.d(mux2to1_nop_d_out),
	.q(nop_q));
//	defparam
//		nop.WIDTH=32;

nop_32 nop10 (
	.d(mux2to1_nop10_d_out),
	.q(nop10_q));
//	defparam
//		nop10.WIDTH=32;

nop_32 nop13 (
	.d(mux9to1_nop13_d_out),
	.q(nop13_q));
//	defparam
//		nop13.WIDTH=32;

nop_32 nop9 (
	.d(mux3to1_nop9_d_out),
	.q(nop9_q));
//	defparam
//		nop9.WIDTH=32;

zeroer_5 zeroer (
	.d(ifetch_rs),
	.en(ctrl_zeroer_en),
	.q(zeroer_q));
//	defparam
//		zeroer.WIDTH=5;

zeroer_5 zeroer0 (
	.d(ifetch_rt),
	.en(ctrl_zeroer0_en),
	.q(zeroer0_q));
//	defparam
//		zeroer0.WIDTH=5;

zeroer_5 zeroer4 (
	.d(mux3to1_zeroer4_d_out),
	.en(ctrl_zeroer4_en),
	.q(zeroer4_q));
//	defparam
//		zeroer4.WIDTH=5;

cop2 cop2 (
	.clk(clk),
	.resetn(resetn),
	.stalled(ctrl_cop2_stalled),
	.fromcpu(nop10_q),
	.fromcpu_en(ctrl_cop2_fromcpu_en),
	.tocpu(cop2_tocpu),
	.tocpu_en(ctrl_cop2_tocpu_en),
	.tocop2(cop2_tocop2),
	.tocop2_en(cop2_tocop2_en),
	.tocop2_wait(cop2_tocop2_wait),
	.fromcop2(cop2_fromcop2),
	.fromcop2_en(cop2_fromcop2_en),
	.fromcop2_wait(cop2_fromcop2_wait)	
	);

cop0 cop0 (
	.clk(clk),
	.resetn(resetn),
	.stalled(ctrl_cop0_stalled),
	.instr(pipereg27_q),
	.exception(cop0_exception),
	.read_addr(pipereg29_q),
	.dest_addr(pipereg29_q),
	.fromcpu(nop10_q),
	.fromcpu_en(ctrl_cop0_fromcpu_en),
	.tocpu(cop0_tocpu),
	.tocpu_en(ctrl_cop0_tocpu_en),
	.epc_in(pipereg28_q),
	.ext_cause_in(cop0_ext_cause_in),
	.int_cause_in_stage1(pipereg30_q),
	.int_cause_in_stage2(data_mem_ecause),
	.status(cop0_status),
	.badvaddr_in(cop0_badvaddr_in),
	.badvaddr_we(cop0_badvaddr_we)
	);

		// Multiplexor mux2to1_mul_opA instantiation
assign mux2to1_mul_opA_out = 
	(ctrl_mux2to1_mul_opA_sel==1) ? nop_q :
	nop9_q;

		// Multiplexor mux2to1_addersub_opA instantiation
assign mux2to1_addersub_opA_out = 
	(ctrl_mux2to1_addersub_opA_sel==1) ? fakedelay_q :
	nop_q;

		// Multiplexor mux3to1_mul_sa instantiation
assign mux3to1_mul_sa_out = 
	(ctrl_mux3to1_mul_sa_sel==2) ? nop_q :
	(ctrl_mux3to1_mul_sa_sel==1) ? const12_out :
	pipereg2_q;

		// Multiplexor mux2to1_hi_reg_d instantiation
assign mux2to1_hi_reg_d_out = 
	(ctrl_mux2to1_hi_reg_d_sel==1) ? div_remainder :
	mul_hi;

		// Multiplexor mux2to1_lo_reg_d instantiation
assign mux2to1_lo_reg_d_out = 
	(ctrl_mux2to1_lo_reg_d_sel==1) ? div_quotient :
	mul_lo;

		// Multiplexor mux9to1_nop13_d instantiation
assign mux9to1_nop13_d_out = 
	(ctrl_mux9to1_nop13_d_sel==8) ? cop2_tocpu :
	(ctrl_mux9to1_nop13_d_sel==7) ? cop0_tocpu :
	(ctrl_mux9to1_nop13_d_sel==6) ? addersub_result :
	(ctrl_mux9to1_nop13_d_sel==5) ? addersub_result_slt :
	(ctrl_mux9to1_nop13_d_sel==4) ? logic_unit_result :
	(ctrl_mux9to1_nop13_d_sel==3) ? mul_shift_result :
	(ctrl_mux9to1_nop13_d_sel==2) ? data_mem_d_loadresult :
	(ctrl_mux9to1_nop13_d_sel==1) ? hi_reg_q :
	lo_reg_q;

		// Multiplexor mux2to1_pipereg_d instantiation
assign mux2to1_pipereg_d_out = 
	(ctrl_mux2to1_pipereg_d_sel==1) ? ifetch_offset :
	signext16_out;

		// Multiplexor mux2to1_pipereg6_d instantiation
assign mux2to1_pipereg6_d_out = 
	(ctrl_mux2to1_pipereg6_d_sel==1) ? pcadder_result :
	merge26lo_out;

		// Multiplexor mux6to1_pipereg7_d instantiation
assign mux6to1_pipereg7_d_out = 
	(ctrl_mux6to1_pipereg7_d_sel==5) ? branchresolve_eq :
	(ctrl_mux6to1_pipereg7_d_sel==4) ? branchresolve_ne :
	(ctrl_mux6to1_pipereg7_d_sel==3) ? branchresolve_lez :
	(ctrl_mux6to1_pipereg7_d_sel==2) ? branchresolve_ltz :
	(ctrl_mux6to1_pipereg7_d_sel==1) ? branchresolve_gez :
	branchresolve_gtz;

		// Multiplexor mux3to1_nop9_d instantiation
assign mux3to1_nop9_d_out = 
	(ctrl_mux3to1_nop9_d_sel==2) ? pipereg_q :
	(ctrl_mux3to1_nop9_d_sel==1) ? nop10_q :
	const11_out;

		// Multiplexor mux3to1_zeroer4_d instantiation
assign mux3to1_zeroer4_d_out = 
	(ctrl_mux3to1_zeroer4_d_sel==2) ? ifetch_rt :
	(ctrl_mux3to1_zeroer4_d_sel==1) ? ifetch_rd :
	const_out;

pipereg_6 pipereg15 (
	.clk(clk),
	.resetn(ctrl_pipereg15_resetn),
	.d(ifetch_func),
	.squashn(ctrl_pipereg15_squashn),
	.en(ctrl_pipereg15_en),
	.q(pipereg15_q));
//	defparam
//		pipereg15.WIDTH=6;

pipereg_5 pipereg16 (
	.clk(clk),
	.resetn(ctrl_pipereg16_resetn),
	.d(ifetch_rt),
	.squashn(ctrl_pipereg16_squashn),
	.en(ctrl_pipereg16_en),
	.q(pipereg16_q));
//	defparam
//		pipereg16.WIDTH=5;

pipereg_6 pipereg14 (
	.clk(clk),
	.resetn(ctrl_pipereg14_resetn),
	.d(ifetch_opcode),
	.squashn(ctrl_pipereg14_squashn),
	.en(ctrl_pipereg14_en),
	.q(pipereg14_q));
//	defparam
//		pipereg14.WIDTH=6;

branch_detector branch_detector (
	.func(ifetch_func),
	.opcode(ifetch_opcode),
	.is_branch(branch_detector_is_branch));

pipereg_5 pipereg17 (
	.clk(clk),
	.resetn(ctrl_pipereg17_resetn),
	.d(ifetch_rs),
	.squashn(ctrl_pipereg17_squashn),
	.en(ctrl_pipereg17_en),
	.q(pipereg17_q));
//	defparam
//		pipereg17.WIDTH=5;

pipereg_6 pipereg19 (
	.clk(clk),
	.resetn(ctrl_pipereg19_resetn),
	.d(pipereg15_q),
	.squashn(ctrl_pipereg19_squashn),
	.en(ctrl_pipereg19_en),
	.q(pipereg19_q));
//	defparam
//		pipereg19.WIDTH=6;

pipereg_6 pipereg18 (
	.clk(clk),
	.resetn(ctrl_pipereg18_resetn),
	.d(pipereg14_q),
	.squashn(ctrl_pipereg18_squashn),
	.en(ctrl_pipereg18_en),
	.q(pipereg18_q));
//	defparam
//		pipereg18.WIDTH=6;

pipereg_5 pipereg20 (
	.clk(clk),
	.resetn(ctrl_pipereg20_resetn),
	.d(pipereg16_q),
	.squashn(ctrl_pipereg20_squashn),
	.en(ctrl_pipereg20_en),
	.q(pipereg20_q));
//	defparam
//		pipereg20.WIDTH=5;

pipereg_5 pipereg21 (
	.clk(clk),
	.resetn(ctrl_pipereg21_resetn),
	.d(pipereg17_q),
	.squashn(ctrl_pipereg21_squashn),
	.en(ctrl_pipereg21_en),
	.q(pipereg21_q));
//	defparam
//		pipereg21.WIDTH=5;

wire en_pipereg22;
assign en_pipereg22 = ~stall_out_stage1;
wire squashn_pipereg22;
assign squashn_pipereg22 = ~branch_mispred;
pipereg_1 pipereg22 (
	.clk(clk),
	.resetn(resetn),
	.d(branch_detector_is_branch),
	.squashn(squashn_pipereg22),
	.en(en_pipereg22),
	.q(pipereg22_q));
//	defparam
//		pipereg22.WIDTH=1;

pipereg_1 pipereg23 (
	.clk(clk),
	.resetn(ctrl_pipereg23_resetn),
	.d(predictme),
	.squashn(ctrl_pipereg23_squashn),
	.en(ctrl_pipereg23_en),
	.q(pipereg23_q));
//	defparam
//		pipereg23.WIDTH=1;

		// Multiplexor mux2to1_nop_d instantiation
assign mux2to1_nop_d_out = 
	(pipereg31_q==1) ? pipereg26_q :
	reg_file_a_readdataout;

pipereg_1 pipereg31 (
	.clk(clk),
	.resetn(ctrl_pipereg31_resetn),
	.d(haz_zeroer_q_pipereg5_q),
	.squashn(ctrl_pipereg31_squashn),
	.en(ctrl_pipereg31_en),
	.q(pipereg31_q));
//	defparam
//		pipereg31.WIDTH=1;

		// Multiplexor mux2to1_nop10_d instantiation
assign mux2to1_nop10_d_out = 
	(pipereg32_q==1) ? pipereg26_q :
	reg_file_b_readdataout;

pipereg_1 pipereg32 (
	.clk(clk),
	.resetn(ctrl_pipereg32_resetn),
	.d(haz_zeroer0_q_pipereg5_q),
	.squashn(ctrl_pipereg32_squashn),
	.en(ctrl_pipereg32_en),
	.q(pipereg32_q));
//	defparam
//		pipereg32.WIDTH=1;

pipereg_1 pipereg25 (
	.clk(clk),
	.resetn(ctrl_pipereg25_resetn),
	.d(pipereg23_q),
	.squashn(ctrl_pipereg25_squashn),
	.en(ctrl_pipereg25_en),
	.q(pipereg25_q));
//	defparam
//		pipereg25.WIDTH=1;

// PETES CHANGE add trace signals
assign trc_data=nop13_q;
assign trc_we=ctrl_reg_file_c_we;
assign trc_addr=pipereg5_q;
assign trc_pipestall=stall_out_stage2;

endmodule

/****************************************************************************
          AddSub unit
- Should perform ADD, ADDU, SUBU, SUB, SLT, SLTU

  is_slt signext addsub
    op[2] op[1] op[0]  |  Operation
0     0     0     0         SUBU
2     0     1     0         SUB
1     0     0     1         ADDU
3     0     1     1         ADD
4     1     0     0         SLTU
6     1     1     0         SLT

****************************************************************************/
//`include "options.v"

module addersub_32 (
            opA, opB,
            op, 
            result,
            result_slt );

input [32-1:0] opA;
input [32-1:0] opB;
//input carry_in;
input [3-1:0] op;

output [32-1:0] result;
output result_slt;

wire carry_out;
wire [32:0] sum;

// Mux between sum, and slt
wire is_slt;
wire signext;
wire addsub;

assign is_slt=op[2];
assign signext=op[1];
assign addsub=op[0];

assign result=sum[32-1:0];
//assign result_slt[32-1:1]={31{1'b0}};
//assign result_slt[0]=sum[32];
assign result_slt=sum[32];

`ifndef USE_INHOUSE_LOGIC
    `define USE_INHOUSE_LOGIC
`endif

`ifdef USE_INHOUSE_LOGIC
wire [(32+1)-1:0] dataa;
wire [(32+1)-1:0] datab;
wire cin;

assign dataa = {signext&opA[32-1],opA};
assign datab = {signext&opB[32-1],opB};
assign cin = ~addsub;

  local_add_sub_33_0_SIGNED local_adder_inst(
      .dataa(dataa),
      .datab(datab),
      .cin(cin),
      .add_sub(addsub),
      .result(sum)
  );

`else
lpm_add_sub adder_inst(
    .dataa({signext&opA[32-1],opA}),
    .datab({signext&opB[32-1],opB}),
    .cin(~addsub),
    .add_sub(addsub),
    .result(sum)
        // synopsys translate_off
        ,
        .cout (),
        .clken (),
        .clock (),
        .overflow (),
        .aclr ()
        // synopsys translate_on
    );
defparam 
    adder_inst.lpm_width=32+1,
    adder_inst.lpm_representation="SIGNED";
`endif

assign carry_out=sum[32];
endmodule

module local_add_sub_33_0_SIGNED(
dataa,
datab,
cin,
add_sub,
result
);

input[33-1:0] dataa;
input[33-1:0] datab;
input cin;
input add_sub;
output reg [33-1:0] result;

always @(*)begin
    if(add_sub == 1'b1)
         result = dataa+datab+cin;
    else
         result = dataa - datab;
end

endmodule

/****************************************************************************
          logic unit
- note ALU must be able to increment PC for JAL type instructions

Operation Table
  op
  0     AND
  1     OR
  2     XOR
  3     NOR
****************************************************************************/
module logic_unit_32 (
            opA, opB,
            op,
            result);

input [32-1:0] opA;
input [32-1:0] opB;
input [2-1:0] op;
output [32-1:0] result;

reg [32-1:0] logic_result;

always@(opA or opB or op )
    case(op)
        2'b00:
            logic_result=opA&opB;
        2'b01:
            logic_result=opA|opB;
        2'b10:
            logic_result=opA^opB;
        2'b11:
            logic_result=~(opA|opB);
    endcase

assign result=logic_result;


endmodule

/****************************************************************************
            Fetch Unit with branch prediction

  IMPORTANT: tgt_pc should arrive 1 cycle after instruction to account for delay slot.
  Also, we compress the prediction PC's by one bit since delay slots prevent consecutive branches.
            
op
  0  Conditional PC write
  1  UnConditional PC write

****************************************************************************/

module ifetch_67108896_32_14_16384(clk,resetn,
        en,         // enable increment (stage 1)
        squashn,
        we,         // enable pc update (later stage)
        op,
        load,
        load_data,

        pcwrop,     // differentiates between unconditionals: 1-unconditional
        predict_tgt_pc,
        predict_en, // enable pc update (early prediction stage)
        predict_result_rdy,
        predict_result,

        interrupt,
        epc,
        ecause,

        pc_out,
        next_pc,

  boot_iaddr, 
  boot_idata, 
  boot_iwe,

    bus_address,
    bus_en,
    bus_readdata,
    bus_wait,
    bus_squashn,
    bus_ecause,

        opcode,
        rs,
        rt,
        rd,
        sa,
        offset,
        instr_index,
        func,
        instr);

input [31:0] boot_iaddr;
input [31:0] boot_idata;
input boot_iwe;

output [32-1:0] bus_address;
output         bus_en;
input  [32-1:0] bus_readdata;
input           bus_wait;
output         bus_squashn;
input  [32-1:0] bus_ecause;

input clk;
input resetn;
input en;     // PC increment enable
input we;     // PC write enable
input squashn;// squash fetch
input op;     // determines if conditional or unconditional branch
input load;
input [32-1:0] load_data;

input pcwrop;
input [32-1:0] predict_tgt_pc;
input predict_en;
input predict_result_rdy;
output predict_result;

input  interrupt; 
output [32-1:0] epc; 
output [31:0] ecause; 

output [32-1:0] pc_out;   // output pc + 1 shifted left 2 bits
output [32-1:0] next_pc;
output [31:26] opcode;
output [25:21] rs;
output [20:16] rt;
output [15:11] rd;
output [10:6] sa;
output [15:0] offset;
output [25:0] instr_index;
output [5:0] func;
output [32-1:0] instr;


wire [32-1:0] pc_plus_1;
reg [32-1:0] pc;
wire ctrl_load;

// prediction stuff
wire prediction;
wire prediction_saved;
wire predict_en_saved;
wire [32-1:0] pc_rollbacknottaken;
wire [32-1:0] pc_rollback;

//not connect ports
wire [31:0] boot_iaddr_nc;
assign boot_iaddr_nc = boot_iaddr;
wire [31:0] boot_idata_nc;
assign boot_idata_nc = boot_idata;
wire  boot_iwe_nc;
assign boot_iwe_nc = boot_iwe;

reg [32-1:0] _next_pc;
reg pc_load_en;
reg predict_result;

// tolerating waits stuff
reg [32-1:0] pc_save_data;
reg pc_save;
wire pc_advance;

wire is_branch;
wire is_delayslot;  //Tells us if we're fetching a delay slot

assign pc_advance=(en&~bus_wait);

assign ctrl_load=(load&~op|op);
  
assign bus_address=next_pc;
assign bus_en=(en|~squashn)&resetn;
assign bus_squashn=squashn;

/******
* WARNING: pipeline-specific - because we know the stage after fetching never
* stalls, we don't need to freeze the result.  So 'en' is a don't care here
******/
assign instr=(bus_wait) ? 'h021 : bus_readdata;

/*  DEBUG: using onchip memory and synthetic stalls
assign instr=(imem_wait) ? 'h021 : imem_readdata;

reg [5:0] count;
  always@(posedge clk)
  if (!resetn)
    count<=0;
  else
    count<=count+1;
assign imem_wait=(count[2]);
altsyncram  imem (
    .clock0 (clk),
    .clocken0 (bus_en|~resetn),
    .clock1 (clk),                              // changed
    .clocken1 (boot_iwe),                       // changed
    `ifdef TEST_BENCH
    .aclr0(~resetn), 
    `endif
    .address_a (next_pc[32-1:2]),
    .wren_b (boot_iwe), .data_b (boot_idata), .address_b (boot_iaddr), //changed

    // synopsys translate_off
    .wren_a (), .rden_b (), .data_a (), 
    .aclr1 (), .byteena_a (), .byteena_b (),
    .addressstall_a (), .addressstall_b (), .q_b (),
    // synopsys translate_on
    
    .q_a (imem_readdata)
    );
    defparam
        imem.intended_device_family = "Stratix",
        imem.width_a = 32, 
        imem.widthad_a = 14,
        imem.numwords_a = 16384,
        imem.operation_mode = "BIDIR_DUAL_PORT",    // changed
        imem.width_b = 32,                 // new
        imem.widthad_b = 14,            // new
        imem.numwords_b = 16384,                   // new
        imem.outdata_reg_b = "UNREGISTERED",
        imem.outdata_reg_a = "UNREGISTERED",
        imem.address_reg_b = "CLOCK1",              // new
        imem.wrcontrol_wraddress_reg_b = "CLOCK1",  // new
        imem.width_byteena_a = 1,
        `ifdef TEST_BENCH
        imem.address_aclr_a = "CLEAR0",
        imem.outdata_aclr_a = "CLEAR0",
        imem.init_file = "instr.rif",
        `endif
        `ifdef QUARTUS_SIM
          imem.init_file = "instr.mif",
          imem.ram_block_type = "AUTO",
        `else
          imem.ram_block_type = "MEGARAM",
        `endif
        imem.lpm_type = "altsyncram";
*/

wire dummy;

assign {dummy,pc_plus_1} = pc + 4;
assign pc_out=pc_plus_1;

//For delay slot instruction - point to branch
assign epc=(is_delayslot) ? pc - 4 : pc;
//Insert your interrupt pending flags here 
assign ecause=bus_ecause;

assign opcode=instr[31:26];
assign rs=instr[25:21];
assign rt=instr[20:16];
assign rd=instr[15:11];
assign sa=instr[10:6];
assign offset=instr[15:0]; 
assign instr_index=instr[25:0];
assign func=instr[5:0];

//************** BRANCH PREDICTION stuff ************
// When predict_en is asserted we accept the prediction signal's value and
// adjust the PC accordingly.  In addition, we buffer everything needed to
// verify and rollback.  
// When predict_result_rdy is asserted we examine ctrl_load and compare the
// pipeline's intended change to the PC with the one we've done here.
// Note that prediction must happen on last delay slot instruction meaning that
// both the predict_en assertion and tgt_pc must come at that time


// Backup PC for both taken/not taken paths
wire [32-1:0] pcrollback_d;
wire pcrollback_en;

assign pcrollback_d = (prediction) ? pc_plus_1 : predict_tgt_pc;
assign pcrollback_en = en&predict_en;

register_32 pcrollback( pcrollback_d, 
    clk, resetn, pcrollback_en, pc_rollback);
  //defparam pcrollback.WIDTH=32;

wire pcrollbacknt_en;
assign pcrollbacknt_en = en&predict_en;

register_32 pcrollbacknt( pc, clk, resetn, pcrollbacknt_en, pc_rollbacknottaken);
 // defparam pcrollbacknt.WIDTH=32;

//register_32 pcrollbacktk(predict_tgt_pc, clk, resetn,predict_en, 
    //pc_rollback);
  //defparam pcrollbacktk.WIDTH=32;

wire [2-1: 0] buf_predict_d;
wire buf_predict_en;
wire [2-1:0] buf_predict_q;

assign buf_predict_d = {prediction,predict_en&(pcwrop!=1'b1)};
assign buf_predict_en = en&predict_en;
assign {prediction_saved,predict_en_saved} = buf_predict_q;

register_2 buf_predict(buf_predict_d,clk,resetn,buf_predict_en, 
    buf_predict_q);
  //defparam buf_predict.WIDTH=2;
  //predict_en_saved saves if it wa

/*** Saving Business
 * When a write to the PC happens deeper in the pipe while the ifetch is frozen
 * we originally stalled that branch also.  Now we save the new PC and load it
 * in once the ifetch becomes available.
 */
reg squash_save;
always@(posedge clk or negedge resetn) 
  if (!resetn)
    squash_save<=0;
  else if ( ~squashn || pc_advance)  // Capture squash requests when we're busy
    squash_save<=~squashn&~pc_advance;

always@(posedge clk or negedge resetn) 
  if (!resetn)
    pc_save<=0;
  else if ( pc_load_en || pc_advance)  // Capture we and advance to clear it
    pc_save<=pc_load_en&~pc_advance;  // zero the save if we're enabled

always@(posedge clk or negedge resetn)
  if (!resetn)
    pc_save_data<=0;
  else if (pc_load_en)  // Capture we, and advance to clear captured data
    pc_save_data<=_next_pc;


always@(posedge clk or negedge resetn)
  if (!resetn)
    pc<='h400_0000;                 // 0x400_0000/4
  else if (pc_advance)
    pc<=_next_pc;


reg [2:0] debug;

//always@(prediction_saved or predict_en_saved or prediction or en or predict_en or ctrl_load or predict_result_rdy or pc_plus_1 or load_data or we or predict_tgt_pc or pc_rollback or pc_rollbacknottaken or pc or pc_advance)
always@(*)
  begin
    if (interrupt)
    begin   // If interrupt occurs, jump to 67108896
      _next_pc=67108896;
      pc_load_en=1;
      debug=5;
    end
    else if (ctrl_load & !predict_result_rdy)
    begin   // No prediction, do indirect jump
      _next_pc=load_data;
      pc_load_en=1;
      debug=1;
    end
    else if (predict_en & prediction)
    begin   // Make a prediction to take
      _next_pc=predict_tgt_pc;
      pc_load_en=en;
      debug=2;
    end
    else if (predict_result_rdy & (ctrl_load!=prediction_saved) & predict_en_saved)
    begin   // Mispredict - restore branch
      _next_pc=pc_rollback;
      pc_load_en=1;
      debug=3;
    end
    else if (pc_save)
    begin   // If bus is stalled and a pc load happens, save it and restore it
            // once the bus unstalls, deal with the squash + protect delay slot
      _next_pc=pc_save_data;
      pc_load_en=pc_advance;
      debug=4;
    end
    else
    begin
        _next_pc=pc_plus_1;
        pc_load_en=0;
        debug=0;
    end
end

// Separated from above always block because not dependent on pc_advance
always@*
  begin
    if (ctrl_load & !predict_result_rdy)
      predict_result=~we;  // Only squash proc pipeline when not stalled
    else if (predict_result_rdy & (ctrl_load!=prediction_saved) & predict_en_saved)
      predict_result=~we;
    else if (pc_save)
      predict_result=~(squash_save&~is_delayslot);
    else
      predict_result=1;  // Used to flush pipe
end

assign next_pc=(pc_advance) ? _next_pc : pc;


/********************* Delay slot protection *******************************/
// We have to do the protection here since we've separated this ifetch and its
// stalls from the processor pipeline, we just emit nops.  SPREE automatically
// protects delay slots but since we can't tell it when the delay slot is
// stalled, we have to do it here.

branch_detector branch_detector (
  .func(func),
  .opcode(opcode),
  .is_branch(is_branch));

pipereg_1 pipereg (
  .clk(clk),
  .resetn(resetn),
  .d(is_branch),
  .squashn(1'b1),
  .en(pc_advance),
  .q(is_delayslot));
  //defparam
  //  pipereg.WIDTH=1;
/***************************************************************************/

wire prediction_tmp;
wire predict_result_rdy_tmp;

assign predict_result_rdy_tmp=predict_result_rdy&predict_en_saved;

branchpredict_32_4096_12_1 bpredictor ( 
    .clk(clk),
    .resetn(resetn),
    .predict(en),
    .prediction(prediction_tmp),
    .pc_predict({next_pc[32-1:3],3'b0}),
    .result_rdy(predict_result_rdy_tmp),
    .result(ctrl_load),
    .pc_result({pc_rollbacknottaken[32-1:3],3'b0}) );

assign prediction=(pcwrop!=1) ? prediction_tmp :1;

endmodule

/****************************************************************************
          Generic Register
****************************************************************************/
module register_32(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [32-1:0] d;
output [32-1:0] q;
reg [32-1:0] q;

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
module register_2(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [2-1:0] d;
output [2-1:0] q;
reg [2-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule

module branchpredict_32_4096_12_1 ( clk, resetn,
    predict,
    prediction,
    pc_predict,
    result_rdy,
    result,
    pc_result);

input clk;
input resetn;

// Prediction Port
input predict;                  // When high tells predictor to predict in next cycle
input [32-1:0] pc_predict; // The PC value for which to predict 
output reg prediction;              // The actual prediction 1-taken, 0-nottaken
wire prediction_temp;

// Prediction Result Port - tells us if the prediction made at pc_result was taken
input result_rdy;               // The branch has been resolved when result_rdy goes hi
input [32-1:0] pc_result;  // The PC value that this result is for
input result;                   // The actual result 1-taken, 0-nottaken

wire resetn_nc;
wire predict_nc;
wire [32-1:0] pc_predict_local;
wire [32-1:0] pc_result_local;

assign resetn_nc = resetn;
assign predict_nc = predict;
assign pc_predict_local = pc_predict;
assign pc_result_local = pc_result;

wire [12-1:0] address_b;

assign address_b=pc_predict_local[12+2-1:2];

`ifndef USE_INHOUSE_LOGIC
    `define USE_INHOUSE_LOGIC
`endif

`ifdef USE_INHOUSE_LOGIC
    wire [1-1:0] pred_table_out_a_nc;

    dpram_12_4096_1 pred_table(
	.clk(clk),
	.address_a(pc_result_local[12+2-1:2]),
	.address_b(address_b),
	.wren_a(result_rdy),
	.wren_b(0),
	.data_a(result),
	.data_b(0),
	.out_a(pred_table_out_a_nc),
	.out_b(prediction_temp)
    );
  // HACK...HACK....HACK
  // Somehow abc was thinking that output of dpram is a combinational port. Though the port is sequential as per the architecture file (agilex_arch.auto_layout.xml). The input address (address_b, pc_predict) comes from the parent module and the output data (out_b, prediction) goes to parent module without any logic in between. The output and input of a dpram are connected combinatoraly in the parent module. So abc thinks that here is a combinatoral loop.
    always @(posedge clk)
    begin
        prediction <= prediction_temp;
    end
`else

	altsyncram	pred_table(
				.clock0 (clk),
				.wren_a (result_rdy),
				.address_a (pc_result[LOG2TABLEDEPTH+2-1:2]),
				.data_a (result),
				.address_b (address_b),
        .clock1 (clk),
        .clocken1 (predict),
				.q_b (prediction)
        // synopsys translate_off
        ,
        .aclr0 (1'b0),
        .aclr1 (1'b0),
        .byteena_a (1'b1),
        .byteena_b (1'b1),
        .data_b (32'b11111111),
        .wren_b (1'b0),
        .rden_b(1'b1),
        .q_a (),
        .clocken0 (1'b1),
        .addressstall_a (1'b0),
        .addressstall_b (1'b0)
        // synopsys translate_on
    );
	defparam
		pred_table.operation_mode = "DUAL_PORT",
		pred_table.width_a = TABLEWIDTH,
		pred_table.widthad_a = LOG2TABLEDEPTH,
		pred_table.numwords_a = TABLEDEPTH,
		pred_table.width_b = TABLEWIDTH,
		pred_table.widthad_b = LOG2TABLEDEPTH,
		pred_table.numwords_b = TABLEDEPTH,
		pred_table.lpm_type = "altsyncram",
		pred_table.width_byteena_a = 1,
		pred_table.outdata_reg_b = "UNREGISTERED",
		pred_table.indata_aclr_a = "NONE",
		pred_table.wrcontrol_aclr_a = "NONE",
		pred_table.address_aclr_a = "NONE",
		pred_table.rdcontrol_reg_b = "CLOCK1",
		pred_table.address_reg_b = "CLOCK1",
		pred_table.address_aclr_b = "NONE",
		pred_table.outdata_aclr_b = "NONE",
		pred_table.read_during_write_mode_mixed_ports = "OLD_DATA",
		pred_table.ram_block_type = "AUTO",
		pred_table.intended_device_family = "Stratix";

`endif
endmodule

module dpram_12_4096_1 (
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
input [(12-1):0] address_a;
input [(12-1):0] address_b;
input  wren_a;
input  wren_b;
input [(1-1):0] data_a;
input [(1-1):0] data_b;
output reg [(1-1):0] out_a;
output reg [(1-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [(((4096-1)-(0)+1)*((1-1)-(0)+1))-1 : 0] ram;

always @ (posedge clk) begin 
  if (wren_a) begin
      ram[((address_a-0)*((1-1)-(0)+1)+1-1) : ((address_a-0)*((1-1)-(0)+1)+0)] <= data_a;
  end
  else begin
      out_a <= ram[((address_a-0)*((1-1)-(0)+1)+1-1) : ((address_a-0)*((1-1)-(0)+1)+0)];
  end
end
  
always @ (posedge clk) begin 
  if (wren_b) begin
      ram[((address_b-0)*((1-1)-(0)+1)+1-1) : ((address_b-0)*((1-1)-(0)+1)+0)] <= data_b;
  end 
  else begin
      out_b <= ram[((address_b-0)*((1-1)-(0)+1)+1-1) : ((address_b-0)*((1-1)-(0)+1)+0)];
  end
end

`else

dual_port_ram u_dual_port_ram(
.addr1(address_a),
.we1(wren_a),
.data1(data_a),
.out1(out_a),
.addr2(address_b),
.we2(wren_b),
.data2(data_b),
.out2(out_b),
.clk(clk)
);

`endif

endmodule

/****************************************************************************
          MUL/DIV unit

Operation table

   op sign dir
4  1   0    x    |  MULTU
6  1   1    x    |  MULT
0  0   0    0    |  ShiftLeft
1  0   0    1    |  ShiftRightLogic
3  0   1    1    |  ShiftRightArith
****************************************************************************/
//`include "options.v"

module mul_32 (clk, resetn, start, stalled, dst,
            opA, opB, sa,
            op,
            shift_result,
            hi, lo);

input clk;
input resetn;

input start;
output stalled;

input [4:0] dst;

input [32-1:0] opA;
input [32-1:0] opB;
input [5-1:0] sa;
input [2:0] op;

output [32-1:0] shift_result;
output [32-1:0] hi;
output [32-1:0] lo;

/********* Control Signals *********/
wire is_signed, dir, is_mul;
assign is_mul=op[2];      // selects between opB and the computed shift amount
assign is_signed=op[1];
assign dir=op[0];         // selects between 2^sa and 2^(32-sa) for right shift

/********* Circuit Body *********/
wire dum,dum2,dum3;
wire [32:0] opB_mux_out;
wire [5-1:0] left_sa;     // Amount of left shift required for both left/right
reg [32:0] decoded_sa;

assign opB_mux_out= (is_mul) ? {is_signed&opB[32-1],opB} : decoded_sa;

`ifndef USE_INHOUSE_LOGIC
    `define USE_INHOUSE_LOGIC
`endif

`ifdef USE_INHOUSE_LOGIC
wire [33-1:0] mult_dataa;
wire mult_aclr;
wire [66-1:0] mult_result;

assign mult_dataa = {is_signed&opA[32-1],opA};
assign mult_aclr = ~resetn;

assign {dum2,dum,hi,lo} = mult_result;

local_mult_33_33_66 local_mult_component (
.dataa(mult_dataa),
.datab(opB_mux_out),
.clock(clk),
.clken(1'b1),
.aclr(mult_aclr),
.result(mult_result)
);

`else
 
lpm_mult  lpm_mult_component (
  .dataa ({is_signed&opA[32-1],opA}),
  .datab (opB_mux_out),
  .sum(),
  .clock(clk),
  .clken(),
  .aclr(~resetn),
  .result ({dum2,dum,hi,lo}));
defparam
  lpm_mult_component.lpm_widtha = 32+1,
  lpm_mult_component.lpm_widthb = 32+1,
  lpm_mult_component.lpm_widthp = 2*32+2,
  lpm_mult_component.lpm_widths = 1,
  lpm_mult_component.lpm_pipeline = 1,
  lpm_mult_component.lpm_type = "LPM_MULT",
  lpm_mult_component.lpm_representation = "SIGNED",
  lpm_mult_component.lpm_hint = "MAXIMIZE_SPEED=6";
`endif

assign shift_result= (dir && |sa) ? hi : lo;

assign {dum3, left_sa} = (dir) ? 32-sa : {1'b0,sa};

always@(left_sa or dir)
begin
  decoded_sa = 1 << left_sa;
end

// 1 cycle stall state machine
wire staller_request;
assign staller_request = (start&is_mul)|(start&(|dst)&~is_mul);
onecyclestall staller(staller_request,clk,resetn,stalled);

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

/****************************************************************************
          One cycle Stall circuit
****************************************************************************/
//module onecyclestall(request,clk,resetn,stalled);
//input request;
//input clk;
//input resetn;
//output stalled;
//
//  reg T,Tnext;
//
//  // State machine for Stalling 1 cycle
//  always@(request or T)
//  begin
//    case(T) 
//      1'b0: Tnext=request;
//      1'b1: Tnext=0;
//    endcase 
//  end       
//  always@(posedge clk)
//    if (~resetn)
//      T<=0; 
//    else    
//      T<=Tnext;
//  assign stalled=(request&~T);
//endmodule

module div_0_1_2(en,resetn,stalled,quotient,remainder,dividend,divider,sign,clk);

   input         clk;
   input         resetn;
   input         sign;
   input         en;
   input [31:0]  dividend, divider;
   output [31:0] quotient, remainder;
   output        stalled;

   reg [31:0]    quotient, quotient_temp;
   reg [63:0]    dividend_copy, divider_copy, diff;
   reg           negative_output;
   
   wire [31:0]   remainder = (!negative_output) ? 
                             dividend_copy[31:0] : 
                             ~dividend_copy[31:0] + 1'b1;

   reg [5:0]     bits; 

   reg [1:0] state;

   always@(posedge clk)
     if (!resetn)
       state<=0;
     else
       case(state)
        0: state<=(en) ? 1 : 0;
        1: state<=(bits==5'd1) ? 2 : 1;
        2: state<= 0;
        default: state<=0;
       endcase

   assign stalled = (state==1) || (state==0 && en);
   //assign stalled = (bits==0 && en) || (|bits);

   always @( posedge clk ) 
     if (!resetn)
     begin

        bits = 0;
        quotient = 0;
        quotient_temp = 0;
        dividend_copy = 0;
        divider_copy = 0;
        negative_output =0;
        diff=0;

     end
     else if( en && state==0) begin

        bits = 6'd32;
        quotient = 0;
        quotient_temp = 0;
        dividend_copy = (!sign || !dividend[31]) ? 
                        {32'd0,dividend} : 
                        {32'd0,~dividend + 1'b1};
        divider_copy = (!sign || !divider[31]) ? 
                       {1'b0,divider,31'd0} : 
                       {1'b0,~divider + 1'b1,31'd0};

        negative_output = sign &&
                          ((divider[31] && !dividend[31]) 
                        ||(!divider[31] && dividend[31]));
        
     end 
     else if ( bits > 0 ) begin

        diff = dividend_copy - divider_copy;

        if( !diff[63] ) begin
           dividend_copy = diff;
           quotient_temp = (quotient_temp << 1) | 1'd1;
        end
        else begin
           quotient_temp = quotient_temp << 1;
        end

        quotient = (!negative_output) ? 
                   quotient_temp : 
                   ~quotient_temp + 1'b1;

        divider_copy = divider_copy >> 1;
        bits = bits - 1'b1;

     end
endmodule

/******************************************************************************
            Data memory and interface

Operation table:

  load/store sign size1 size0    |   Operation
7     0       1     1     1      |      LB
5     0       1     0     1      |      LH
0     0       X     0     0      |      LW
3     0       0     1     1      |      LBU
1     0       0     0     1      |      LHU
11    1       X     1     1      |      SB
9     1       X     0     1      |      SH
8     1       X     0     0      |      SW

******************************************************************************/

module data_mem_32_32_4_16_16384( clk, resetn, en, stalled,
    d_writedata,
    d_address,
    op,
    d_loadresult,

    ecause,

    boot_daddr, 
    boot_ddata, 
    boot_dwe, 

    bus_address,
    bus_byteen,
    bus_we,
    bus_en,
    bus_writedata,
    bus_readdata,
    bus_wait,
    bus_ecause
                );

input clk;
input resetn;
input en;
output stalled;

output [31:0] ecause; 

input [31:0] boot_daddr;
input [31:0] boot_ddata;
input boot_dwe;

input [32-1:0] d_address;
input [4-1:0] op;
input [32-1:0] d_writedata;
output [32-1:0] d_loadresult;

output [32-1:0] bus_address;
output [4-1:0] bus_byteen;
output         bus_we;
output         bus_en;
output [32-1:0] bus_writedata;
input  [32-1:0] bus_readdata;
input           bus_wait;
input  [32-1:0] bus_ecause;

wire [4-1:0] d_byteena;
wire [32-1:0] d_readdatain;
wire [32-1:0] d_writedatamem;
wire d_write;
wire [1:0] d_address_latched;

// not connected ports
wire resetn_nc;
assign resetn_nc = resetn;
wire [31:0] boot_daddr_nc;
assign boot_daddr_nc = boot_daddr;
wire [31:0] boot_ddata_nc;
assign boot_ddata_nc = boot_ddata;
wire boot_dwe_nc;
assign boot_dwe_nc = boot_dwe;

assign d_write=op[3];

assign ecause=bus_ecause;

register_2 d_address_reg(d_address[1:0],clk,1'b1,en,d_address_latched);
                
store_data_translator_32 sdtrans_inst(
    .write_data(d_writedata),
    .d_address(d_address[1:0]),
    .store_size(op[1:0]),
    .d_byteena(d_byteena),
    .d_writedataout(d_writedatamem));

load_data_translator_32 ldtrans_inst(
    .d_readdatain(d_readdatain),
    .d_address(d_address_latched[1:0]),
    .load_size(op[1:0]),
    .load_sign_ext(op[2]),
    .d_loadresult(d_loadresult));
  
assign bus_address=d_address;
assign bus_byteen=d_byteena;
assign bus_we=d_write;
assign bus_en=en;
assign bus_writedata=d_writedatamem;
assign d_readdatain=bus_readdata;
assign stalled=bus_wait;

/*
altsyncram  dmem (
            .wren_a (d_write&en&(~d_address[31])),
            .clock0 (clk),
            .clocken0 (),
            .clock1 (clk),
            .clocken1 (boot_dwe),
            `ifdef TEST_BENCH
            .aclr0(~resetn), 
            `endif
            .byteena_a (d_byteena),
            .address_a (d_address[DM_ADDRESSWIDTH+2-1:2]),
            .data_a (d_writedatamem),
            .wren_b (boot_dwe), .data_b (boot_ddata), .address_b (boot_daddr), 
            // synopsys translate_off
            .rden_b (), 
            .aclr1 (), .byteena_b (),
            .addressstall_a (), .addressstall_b (), .q_b (),
            // synopsys translate_on
            .q_a (d_readdatain)
    
);  
    defparam
        dmem.intended_device_family = "Stratix",
        dmem.width_a = DM_DATAWIDTH,
        dmem.widthad_a = DM_ADDRESSWIDTH-2,
        dmem.numwords_a = DM_SIZE,
        dmem.width_byteena_a = DM_BYTEENAWIDTH,
        dmem.operation_mode = "BIDIR_DUAL_PORT",
        dmem.width_b = DM_DATAWIDTH,
        dmem.widthad_b = DM_ADDRESSWIDTH-2,
        dmem.numwords_b = DM_SIZE,
        dmem.width_byteena_b = 1,
        dmem.outdata_reg_a = "UNREGISTERED",
        dmem.address_reg_b = "CLOCK1",
        dmem.wrcontrol_wraddress_reg_b = "CLOCK1",
        dmem.wrcontrol_aclr_a = "NONE",
        dmem.address_aclr_a = "NONE",
        dmem.outdata_aclr_a = "NONE",
        dmem.byteena_aclr_a = "NONE",
        dmem.byte_size = 8,
        `ifdef TEST_BENCH
          dmem.indata_aclr_a = "CLEAR0",
          dmem.init_file = "data.rif",
        `endif
        `ifdef QUARTUS_SIM
          dmem.init_file = "data.mif",
          dmem.ram_block_type = "M4K",
        `else
          dmem.ram_block_type = "MEGARAM",
        `endif
        dmem.lpm_type = "altsyncram";
*/
  
endmodule

/****************************************************************************
          Store data translator
          - moves store data to appropriate byte/halfword 
          - interfaces with altera blockrams
****************************************************************************/
module store_data_translator_32(
    write_data,             // data in least significant position
    d_address,
    store_size,
    d_byteena,
    d_writedataout);        // shifted data to coincide with address

input [32-1:0] write_data;
input [1:0] d_address;
input [1:0] store_size;
output [3:0] d_byteena;
output [32-1:0] d_writedataout;

reg [3:0] d_byteena;
reg [32-1:0] d_writedataout;

always @(write_data or d_address or store_size)
begin
    case (store_size)
        2'b11:
            case(d_address[1:0])
                0: 
                begin 
                    d_byteena=4'b1000; 
                    d_writedataout={write_data[7:0],24'b0}; 
                end
                1: 
                begin 
                    d_byteena=4'b0100; 
                    d_writedataout={8'b0,write_data[7:0],16'b0}; 
                end
                2: 
                begin 
                    d_byteena=4'b0010; 
                    d_writedataout={16'b0,write_data[7:0],8'b0}; 
                end
                default: 
                begin 
                    d_byteena=4'b0001; 
                    d_writedataout={24'b0,write_data[7:0]}; 
                end
            endcase
        2'b01:
            case(d_address[1])
                0: 
                begin 
                    d_byteena=4'b1100; 
                    d_writedataout={write_data[15:0],16'b0}; 
                end
                default: 
                begin 
                    d_byteena=4'b0011; 
                    d_writedataout={16'b0,write_data[15:0]}; 
                end
            endcase
        default:
        begin
            d_byteena=4'b1111;
            d_writedataout=write_data;
        end
    endcase
end
endmodule

/****************************************************************************
          Load data translator
          - moves read data to appropriate byte/halfword and zero/sign extends
****************************************************************************/
module load_data_translator_32(
    d_readdatain,
    d_address,
    load_size,
    load_sign_ext,
    d_loadresult);

input [32-1:0] d_readdatain;
input [1:0] d_address;
input [1:0] load_size;
input load_sign_ext;
output [32-1:0] d_loadresult;

reg [32-1:0] d_loadresult;

always @(d_readdatain or d_address or load_size or load_sign_ext)
begin
    case (load_size)
        2'b11:
        begin
            case (d_address[1:0])
                0: d_loadresult[7:0]=d_readdatain[31:24];
                1: d_loadresult[7:0]=d_readdatain[23:16];
                2: d_loadresult[7:0]=d_readdatain[15:8];
                default: d_loadresult[7:0]=d_readdatain[7:0];
            endcase
            d_loadresult[31:8]={24{load_sign_ext&d_loadresult[7]}};
        end
        2'b01:
        begin
            case (d_address[1])
                0: d_loadresult[15:0]=d_readdatain[31:16];
                default: d_loadresult[15:0]=d_readdatain[15:0];
            endcase
            d_loadresult[31:16]={16{load_sign_ext&d_loadresult[15]}};
        end
        default:
            d_loadresult=d_readdatain;
    endcase
end

endmodule

/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
//`include "options.v"
module reg_file_32_32_5(clk,resetn, c_squashn,
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_we);

input clk;
input resetn;

input a_en;
input b_en;

input [5-1:0] a_reg,b_reg,c_reg;
output [32-1:0] a_readdataout, b_readdataout;
input [32-1:0] c_writedatain;
input c_we;
input c_squashn;
reg [31:0] i;

`ifndef USE_INHOUSE_LOGIC1
	`define USE_INHOUSE_LOGIC1
`endif

`ifdef USE_INHOUSE_LOGIC1

wire [32-1:0] reg_file1_out_a_nc;
wire reg_file1_wren_a;

assign reg_file1_wren_a = c_we & (|c_reg) & c_squashn;

        ram_wrapper_5_32_32 reg_file1(
	    .clk(clk),
		.resetn(resetn),
		.rden_a(1'b0),
		.rden_b(a_en),
	    .address_a(c_reg[5-1:0]),
	    .address_b(a_reg[5-1:0]),
	    .wren_a(reg_file1_wren_a),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(reg_file1_out_a_nc),
	    .out_b(a_readdataout)
        );
 
// initial begin
//    for(i=0;i<32;i=i+1)
//        reg_file1.dpram1.ram[i]=0;
// end 
         
wire [32-1:0] reg_file2_out_a_nc;
wire reg_file2_wren_a;

assign reg_file2_wren_a = c_we & (|c_reg);
        ram_wrapper_5_32_32 reg_file2(
	    .clk(clk),
		.resetn(resetn),
		.rden_a(1'b0),
		.rden_b(b_en),
	    .address_a(c_reg[5-1:0]),
	    .address_b(b_reg[5-1:0]),
	    .wren_a(reg_file2_wren_a),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(reg_file2_out_a_nc),
	    .out_b(b_readdataout)
        );

// initial begin
//    for(i=0;i<32;i=i+1)
//        reg_file2.dpram1.ram[i]=0;
// end 
`else

	altsyncram	reg_file1(
				.wren_a (c_we & (|c_reg) & c_squashn),
				.clock0 (clk),
        .clock1 (clk),
        .clocken1 (a_en),
				.address_a (c_reg[5-1:0]),
				.address_b (a_reg[5-1:0]),
				.data_a (c_writedatain),
				.q_b (a_readdataout)
        // synopsys translate_off
        ,
        .aclr0 (1'b0),
        .aclr1 (1'b0),
        .byteena_a (1'b1),
        .byteena_b (1'b1),
        .data_b (32'b11111111),
        .wren_b (1'b0),
        .rden_b(1'b1),
        .q_a (),
        .clocken0 (1'b1),
        .addressstall_a (1'b0),
        .addressstall_b (1'b0)
        // synopsys translate_on
    );
	defparam
		reg_file1.operation_mode = "DUAL_PORT",
		reg_file1.width_a = 32,
		reg_file1.widthad_a = 5,
		reg_file1.numwords_a = 32,
		reg_file1.width_b = 32,
		reg_file1.widthad_b = 5,
		reg_file1.numwords_b = 32,
		reg_file1.lpm_type = "altsyncram",
		reg_file1.width_byteena_a = 1,
		reg_file1.outdata_reg_b = "UNREGISTERED",
		reg_file1.indata_aclr_a = "NONE",
		reg_file1.wrcontrol_aclr_a = "NONE",
		reg_file1.address_aclr_a = "NONE",
		reg_file1.rdcontrol_reg_b = "CLOCK1",
		reg_file1.address_reg_b = "CLOCK1",
		reg_file1.address_aclr_b = "NONE",
		reg_file1.outdata_aclr_b = "NONE",
		reg_file1.read_during_write_mode_mixed_ports = "OLD_DATA",
		reg_file1.ram_block_type = "AUTO",
		reg_file1.intended_device_family = "Stratix";

		//Reg file duplicated to avoid contention between 2 read
		//and 1 write
	altsyncram	reg_file2(
				.wren_a (c_we&(|c_reg)),
				.clock0 (clk),
        .clock1 (clk),
        .clocken1 (b_en),
				.address_a (c_reg[5-1:0]),
				.address_b (b_reg[5-1:0]),
				.data_a (c_writedatain),
				.q_b (b_readdataout)
        // synopsys translate_off
        ,
        .aclr0 (1'b0),
        .aclr1 (1'b0),
        .byteena_a (1'b1),
        .byteena_b (1'b1),
        .data_b (32'b11111111),
        .rden_b(1'b1),
        .wren_b (1'b0),
        .q_a (),
        .clocken0 (1'b1),
        .addressstall_a (1'b0),
        .addressstall_b (1'b0)
        // synopsys translate_on
    );
	defparam
		reg_file2.operation_mode = "DUAL_PORT",
		reg_file2.width_a = 32,
		reg_file2.widthad_a = 5,
		reg_file2.numwords_a = 32,
		reg_file2.width_b = 32,
		reg_file2.widthad_b = 5,
		reg_file2.numwords_b = 32,
		reg_file2.lpm_type = "altsyncram",
		reg_file2.width_byteena_a = 1,
		reg_file2.outdata_reg_b = "UNREGISTERED",
		reg_file2.indata_aclr_a = "NONE",
		reg_file2.wrcontrol_aclr_a = "NONE",
		reg_file2.address_aclr_a = "NONE",
		reg_file2.rdcontrol_reg_b = "CLOCK1",
		reg_file2.address_reg_b = "CLOCK1",
		reg_file2.address_aclr_b = "NONE",
		reg_file2.outdata_aclr_b = "NONE",
		reg_file2.read_during_write_mode_mixed_ports = "OLD_DATA",
		reg_file2.ram_block_type = "AUTO",
		reg_file2.intended_device_family = "Stratix";

`endif

endmodule

//module ram_wrapper_5_32_32 (
//	clk,
//        resetn,
//	address_a,
//	address_b,
//        rden_a,
//        rden_b,
//	wren_a,
//	wren_b,
//	data_a,
//	data_b,
//	out_a,
//	out_b
//);
//
//input clk;
//input resetn;
//input [(5-1):0] address_a;
//input [(5-1):0] address_b;
//input  wren_a;
//input  wren_b;
//input  rden_a;
//input  rden_b;
//input [(32-1):0] data_a;
//input [(32-1):0] data_b;
//output [(32-1):0] out_a;
//output [(32-1):0] out_b;
//
//reg [(5-1):0] q_address_a;
//reg [(5-1):0] q_address_b;
//reg [(5-1):0] mux_address_b;
//
//// not connect ports
//wire rden_a_nc;
//assign rden_a_nc = rden_a;
//
//dpram_5_32_32 dpram1(
//    .clk(clk),
//    .address_a(address_a),
//    .address_b(mux_address_b),
//    .wren_a(wren_a),
//    .wren_b(wren_b),
//    .data_a(data_a),
//    .data_b(data_b),
//    .out_a(out_a),
//    .out_b(out_b)
//);
//
//always@(posedge clk)begin
//   if(!resetn)begin
//     q_address_a <= 'h0;
//     q_address_b <= 'h0;
//   end
//   else begin
//     if(rden_b)
//       q_address_b <= address_b;
//   end
//end
//
//always@(*)begin
//  if(rden_b)   
//    mux_address_b = address_b;
//  else
//    mux_address_b = q_address_b; 
//end
//
//endmodule

//module dpram_5_32_32 (
//	clk,
//	address_a,
//	address_b,
//	wren_a,
//	wren_b,
//	data_a,
//	data_b,
//	out_a,
//	out_b
//);
//
//input clk;
//input [(5-1):0] address_a;
//input [(5-1):0] address_b;
//input  wren_a;
//input  wren_b;
//input [(32-1):0] data_a;
//input [(32-1):0] data_b;
//output reg [(32-1):0] out_a;
//output reg [(32-1):0] out_b;
//
//`ifdef SIMULATION_MEMORY
//
//reg [32-1:0] ram[32-1:0];
//
//always @ (posedge clk) begin 
//  if (wren_a) begin
//      ram[address_a] <= data_a;
//  end
//  else begin
//      out_a <= ram[address_a];
//  end
//end
//  
//always @ (posedge clk) begin 
//  if (wren_b) begin
//      ram[address_b] <= data_b;
//  end 
//  else begin
//      out_b <= ram[address_b];
//  end
//end
//
//`else
//
//dual_port_ram u_dual_port_ram(
//.addr1(address_a),
//.we1(wren_a),
//.data1(data_a),
//.out1(out_a),
//.addr2(address_b),
//.we2(wren_b),
//.data2(data_b),
//.out2(out_b),
//.clk(clk)
//);
//
//`endif
//
//endmodule

module pcadder_32(pc, offset, result);

input [32-1:0] pc;
input [32-1:0] offset;
output [32-1:0] result;

// not connect ports
wire [32-1:0] offset_nc;
assign offset_nc = offset;

wire dum;

assign {dum,result} = pc + {offset[32-3:0],2'b0};

endmodule

module signext16 ( in, out);

input [15:0] in;
output [31:0] out;

assign out={{{{16{{in[15]}}}},in[15:0]}};

endmodule

module merge26lo(in1, in2, out);
input [31:0] in1;
input [25:0] in2;
output [31:0] out;

// not connected port
wire [31:0] in1_nc;
assign in1_nc = in1;

assign out[31:0]={in1[31:28],in2[25:0],2'b0};
endmodule


module branchresolve_32 ( en, rs, rt, eq, ne, ltz, lez, gtz, gez, eqz);
parameter WIDTH=32;
input en;
input [WIDTH-1:0] rs;
input [WIDTH-1:0] rt;
output eq;
output ne;
output ltz;
output lez;
output gtz;
output gez;
output eqz;

assign eq=(en)&(rs==rt);
assign ne=(en)&~eq;
assign eqz=(en)&~(|rs);
assign ltz=(en)&rs[WIDTH-1];
assign lez=(en)&rs[WIDTH-1] | eqz;
assign gtz=(en)&(~rs[WIDTH-1]) & ~eqz;
assign gez=(en)&(~rs[WIDTH-1]);

endmodule

/****************************************************************************
          Generic Register
****************************************************************************/
module lo_reg_32 (d,clk,resetn,squashn,en,q);

input clk;
input resetn;
input squashn;
input en;
input [32-1:0] d;
output [32-1:0] q;
reg [32-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1 && squashn)
		q<=d;
end

endmodule

/****************************************************************************
          Generic Register
****************************************************************************/
module hi_reg_32(d,clk,resetn,squashn,en,q);

input clk;
input resetn;
input squashn;
input en;
input [32-1:0] d;
output [32-1:0] q;
reg [32-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1 && squashn)
		q<=d;
end

endmodule

/****************************************************************************
          Const
****************************************************************************/
module const_32_0 (out);

output [32-1:0] out;

assign out=0;

endmodule

/****************************************************************************
          Const
****************************************************************************/
module const_32_16 (out);

output [32-1:0] out;

assign out=16;

endmodule

/****************************************************************************
          Const
****************************************************************************/
module const_32_31 (out);

output [32-1:0] out;

assign out=31;

endmodule

/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
//module pipereg_32(d,clk,resetn,en,squashn,q);
//
//input clk;
//input resetn;
//input en;
//input squashn;
//input [32-1:0] d;
//output [32-1:0] q;
//reg [32-1:0] q;
//
//always @(posedge clk)   //synchronous reset
//begin
//  if (resetn==0 || squashn==0)
//    q<=0;
//  else if (en==1)
//    q<=d;
//end
//
//endmodule

/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
//module pipereg_5(d,clk,resetn,en,squashn,q);
//
//input clk;
//input resetn;
//input en;
//input squashn;
//input [5-1:0] d;
//output [5-1:0] q;
//reg [5-1:0] q;
//
//always @(posedge clk)   //synchronous reset
//begin
//  if (resetn==0 || squashn==0)
//    q<=0;
//  else if (en==1)
//    q<=d;
//end
//
//endmodule

/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
//module pipereg_1(d,clk,resetn,en,squashn,q);
//
//input clk;
//input resetn;
//input en;
//input squashn;
//input [1-1:0] d;
//output [1-1:0] q;
//reg [1-1:0] q;
//
//always @(posedge clk)   //synchronous reset
//begin
//  if (resetn==0 || squashn==0)
//    q<=0;
//  else if (en==1)
//    q<=d;
//end
//
//endmodule

/****************************************************************************
          Fake Delay
****************************************************************************/
module fakedelay_32(d,clk,q);

input [32-1:0] d;
input clk;
output [32-1:0] q;

// not connectr ports
wire clk_nc;
assign clk_nc = clk;

assign q=d;

endmodule

/****************************************************************************
          NOP - used to hack position of multiplexors
****************************************************************************/
module nop_32(d,q);

input [32-1:0] d;
output [32-1:0] q;

  assign q=d;

endmodule

/****************************************************************************
          Zeroer
****************************************************************************/
module zeroer_5(d,en,q);

input en;
input [5-1:0] d;
output [5-1:0] q;
assign q= (en) ? d : 0;

endmodule

/*******
 * SPREE limitation - by not specifying stall signal name and assuming
 * "stalled" requires you to have only one opcode port which stalls
 *
 * We get around this since both INPUT&OUTPUT are in the same stage so we 
 * can use the same stall signal.
 *******/

module cop2(
    clk,
    resetn,
    stalled,

    fromcpu,
    fromcpu_en,
    tocpu,
    tocpu_en,

    //Global I/O
    tocop2,
    tocop2_en,
    tocop2_wait,
    fromcop2,
    fromcop2_en,
    fromcop2_wait
    );

input clk;
input resetn;
output stalled;

input   [31:0] fromcpu;
input          fromcpu_en;
output  [31:0] tocpu;
input          tocpu_en;

output  [31:0] tocop2;
output         tocop2_en;
input          tocop2_wait;
input   [31:0] fromcop2;
input          fromcop2_en;
output         fromcop2_wait;

// not connected ports
wire clk_nc;
assign clk_nc = clk;
wire resetn_nc;
assign resetn_nc = resetn;

  assign tocop2=fromcpu;
  assign tocop2_en=fromcpu_en;

  assign tocpu=fromcop2;
  assign fromcop2_wait=fromcop2_en&~tocpu_en;   //assign 1 if pipe is stalled 

  assign stalled= (fromcpu_en & tocop2_wait) || (tocpu_en & ~fromcop2_en);

endmodule

/*******
 * SPREE limitation - by not specifying stall signal name and assuming
 * "stalled" requires you to have only one opcode port which stalls
 *
 * We get around this since both INPUT&OUTPUT are in the same stage so we 
 * can use the same stall signal.
 *******/

module cop0(
    clk,
    resetn,
    stalled,

    instr,

    exception,

    read_addr,
    dest_addr,
    fromcpu,
    fromcpu_en,
    tocpu,
    tocpu_en,

    epc_in,
    ext_cause_in,
    int_cause_in_stage1,  //very weak - implement OR in SPREE instead
    int_cause_in_stage2,
    status,

    badvaddr_in,
    badvaddr_we
    );

//parameter NUMSTAGESTIMES32=64;
//parameter NUMSTAGES=NUMSTAGESTIMES32/32;

input clk;
input resetn;
output stalled;

input   [31:0] instr;

output exception;

input   [4:0]  read_addr;
input   [4:0]  dest_addr;
input   [31:0] fromcpu;
input          fromcpu_en;
output  [31:0] tocpu;
input          tocpu_en;

input  [31:0] epc_in;

input  [31:0] ext_cause_in;
input  [31:0] int_cause_in_stage1;
input  [31:0] int_cause_in_stage2;

output [31:0] status;
input  [31:0] badvaddr_in;
input         badvaddr_we;

// not connected ports
wire [31:0] instr_nc;
assign instr_nc = instr;

wire [31:0] cause_in;

reg [31:0] epc_out;
reg [31:0] cause_out;
reg [31:0] status;
reg [31:0] badvaddr_out;

reg  [31:0] tocpu;

  assign cause_in=ext_cause_in | int_cause_in_stage1 | 
                                 int_cause_in_stage2;

  always@(posedge clk)
    if (!resetn)
      epc_out<=0;
    else if (fromcpu_en && dest_addr==14)
      epc_out<=fromcpu;
    else if (exception)
      epc_out<=epc_in;

  always@(posedge clk)
    if (!resetn)
      cause_out<=0;
    else if (fromcpu_en && dest_addr==13)
      cause_out<=fromcpu;
    else
      cause_out<=cause_in;

  always@(posedge clk)
    if (!resetn)
      status<=0;
    else if (fromcpu_en && dest_addr==12)
      status<=fromcpu;
    else if (exception)
      status[5:0]<={status[3:0],2'b0};

  always@(posedge clk)
    if (!resetn)
      badvaddr_out<=0;
    else if (fromcpu_en && dest_addr==8)
      badvaddr_out<=fromcpu;
    else if (badvaddr_we)
      badvaddr_out<=badvaddr_in;

  always@(posedge clk)
      tocpu <= (read_addr==14) ? epc_out : 
               (read_addr==13) ? cause_out : 
               (read_addr==8)  ? badvaddr_out : status;

  // 1 cycle stall
  multicyclestall mc(tocpu_en,0,clk,resetn,stalled);
  //assign stalled= 0;

  assign exception = ((|(cause_in[15:8] & status[15:8])) && status[0]);

endmodule

/****************************************************************************
          Multi cycle Stall circuit - with wait signal

          - One FF plus one 2:1 mux to stall 1st cycle on request, then wait
          - this makes wait don't care for the first cycle
****************************************************************************/
module multicyclestall(request, devwait,clk,resetn,stalled);
input request;
input devwait;
input clk;
input resetn;
output stalled;

  reg T;

  always@(posedge clk)
    if (~resetn)
      T<=0;
    else
      T<=stalled;

  assign stalled=(T) ? devwait : request;
endmodule

/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
//module pipereg_6(d,clk,resetn,en,squashn,q);
//
//input clk;
//input resetn;
//input en;
//input squashn;
//input [6-1:0] d;
//output [6-1:0] q;
//reg [6-1:0] q;
//
//always @(posedge clk)   //synchronous reset
//begin
//  if (resetn==0 || squashn==0)
//    q<=0;
//  else if (en==1)
//    q<=d;
//end
//
//endmodule

/****************************************************************************
          Branch detector
****************************************************************************/
module branch_detector(opcode, func, is_branch);
input [5:0] opcode;
input [5:0] func;
output is_branch;

wire is_special;
wire [5:0] func_local;

assign func_local = func & 6'b111000;

assign is_special=!(|opcode);
assign is_branch=((!(|opcode[5:3])) && !is_special) || 
                  ((is_special)&&(func_local==6'b001000));

endmodule
////  ///////// ///

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

wire clk_nc, resetn_nc;
assign clk_nc = clk;
assign resetn_nc = resetn;

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

module vmem_crossbar_128_7_8_32_5 (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter SELWIDTH=7-5;   // LOG2(INWIDTH/OUTWIDTH) = 4

input                             clk;
input                             resetn;
input  [(SELWIDTH*8)-1 : 0] sel;
input  [128-1 : 0]            in;
output [(32*8)-1 : 0] out;


     vmem_busmux_128_7_32_5 bmux0(clk,resetn,
        sel[(0+1)*SELWIDTH - 1 : 0*SELWIDTH],
        in,
        out[(0+1)*32 - 1 : 0*32]);

     vmem_busmux_128_7_32_5 bmux1(clk,resetn,
        sel[(1+1)*SELWIDTH - 1 : 1*SELWIDTH],
        in,
        out[(1+1)*32 - 1 : 1*32]);

     vmem_busmux_128_7_32_5 bmux2(clk,resetn,
        sel[(2+1)*SELWIDTH - 1 : 2*SELWIDTH],
        in,
        out[(2+1)*32 - 1 : 2*32]);

     vmem_busmux_128_7_32_5 bmux3(clk,resetn,
        sel[(3+1)*SELWIDTH - 1 : 3*SELWIDTH],
        in,
        out[(3+1)*32 - 1 : 3*32]);

     vmem_busmux_128_7_32_5 bmux4(clk,resetn,
        sel[(4+1)*SELWIDTH - 1 : 4*SELWIDTH],
        in,
        out[(4+1)*32 - 1 : 4*32]);

     vmem_busmux_128_7_32_5 bmux5(clk,resetn,
        sel[(5+1)*SELWIDTH - 1 : 5*SELWIDTH],
        in,
        out[(5+1)*32 - 1 : 5*32]);

     vmem_busmux_128_7_32_5 bmux6(clk,resetn,
        sel[(6+1)*SELWIDTH - 1 : 6*SELWIDTH],
        in,
        out[(6+1)*32 - 1 : 6*32]);

     vmem_busmux_128_7_32_5 bmux7(clk,resetn,
        sel[(7+1)*SELWIDTH - 1 : 7*SELWIDTH],
        in,
        out[(7+1)*32 - 1 : 7*32]);

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

                  
module dpram1_26_67108864_32 (
    clk,
    address_a,
    address_b,
    wren_a, 
    wren_b, 
    data_a,
    data_b,
    byteen_a,
    byteen_b,
    out_a,
    out_b
);

// parameter AWIDTH=10;
// parameter NUM_WORDS=1024;
// parameter DWIDTH=32;
// parameter LOG2DWIDTH = $clog2(DWIDTH);

input clk;
input [(26-1):0] address_a;
input [(26-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32/8)-1:0] byteen_a;
input [(32/8)-1:0] byteen_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

integer i;
integer k;

//reg [32-1:0] ram [67108864-1:0];
reg [(((1096-1)-(0)+1)*((32-1)-(0)+1))-1 : 0] ram;
reg [25:0] addr_a;
reg [25:0] addr_b;
 
initial
 begin
   //This is TOO big for 256 MB RAM!  We right shift data by 1
   //$readmemh("instr.dat",ram,'h100_0000);
   $readmemh("instr.dat",ram,'h100);
   //$readmemh("data.dat",ram,'h400_0000>>1);
   $readmemh("data.dat",ram,'h400>>1);
 end

always@(*) begin
    addr_a = address_a << 26-26;
    addr_b = address_b << 26-26;
end

always@(posedge clk) begin 
  if (wren_a) begin
      for(k=0; k < 32/32;k=k+1)begin
          for(i=0; i < 4 ;i=i+1)begin
              if(byteen_a[((32/8-1)-((4*k)+i))])
                  ram[addr_a+k][(i*8)+:8] <= data_a[((32*k)+(i*8))+:8];
          end
      end
  end
  else begin
      for(i=0; i < 32/32; i=i+1)begin
          out_a[(32*i)+:32] <= ram[addr_a+i];
      end
  end
  if (wren_b) begin
      for(k=0; k < 32/32;k=k+1)begin
          for(i=0; i < 4 ;i=i+1)begin
              if(byteen_b[((32/8-1)-((4*k)+i))])
                  ram[addr_b+k][(i*8)+:8] <= data_b[((32*k)+(i*8))+:8];
          end
      end
  end
  else begin
      for(i=0; i < 32/32; i=i+1)begin
          out_b[32*i+:32] <= ram[addr_b+i];
      end
  end
end
`else

// Not connected wires
wire [(32/8)-1:0] byteen_a_nc;
wire [(32/8)-1:0] byteen_b_nc;

assign byteen_a_nc = byteen_a;
assign byteen_b_nc = byteen_b;

dual_port_ram u_dual_port_ram(
.addr1(address_a[11:0]),
.we1(wren_a),
.data1(data_a),
.out1(out_a),
.addr2(address_b[11:0]),
.we2(wren_b),
.data2(data_b),
.out2(out_b),
.clk(clk)
);

`endif

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
          Shifter unit

Opcode Table:

sign_ext dir 
 0        0    |  ShiftLeft
 0        1    |  ShiftRightLogic
 1        1    |  ShiftRightArith
          
****************************************************************************/
module vlane_barrelshifter_16_4(clk, resetn,
            opB, sa, 
            op, 
            result);
//parameter 16=32;
//parameter 4=5;

//Shifts the first 2 bits in one cycle, the rest in the next cycle
//parameter (4-2)=4-2;

input clk;
input resetn;

input [16-1:0] opB;
input [4-1:0] sa;                             // Shift Amount
input [2-1:0] op;

output [16-1:0] result;


wire sign_ext;
wire shift_direction;
assign sign_ext=op[1];
assign shift_direction=op[0];

wire dum,dum_,dum2;
wire [16-1:0] partial_result_,partial_result;
`ifndef USE_INHOUSE_LOGIC
    `define USE_INHOUSE_LOGIC
`endif

`ifdef USE_INHOUSE_LOGIC
wire [17-1:0] local_shifter_inst1_result;
assign {dum,partial_result} = local_shifter_inst1_result;

wire [2-1:0] local_shifter_inst1_distance;
assign local_shifter_inst1_distance = sa&(32'hffffffff<<(((4-2)>0) ? (4-2) : 0));

wire [17-1:0] local_shifter_inst1_data;
assign local_shifter_inst1_data = {sign_ext&opB[16-1],opB};

local_shifter_17_2_ARITHMATIC local_shifter_inst1(
  .data(local_shifter_inst1_data),
  .distance(local_shifter_inst1_distance),
  .direction(shift_direction),
  .result(local_shifter_inst1_result)
);
 //defparam
 //   local_shifter_inst1.LPM_WIDTH = 16+1,
 //   local_shifter_inst1.LPM_WIDTHDIST = 4,
 //   local_shifter_inst1.LPM_SHIFTTYPE="ARITHMETIC";
`else
lpm_clshift shifter_inst1(
    .data({sign_ext&opB[16-1],opB}),
    .distance(sa&(32'hffffffff<<(((4-2)>0) ? (4-2) : 0))),
    .direction(shift_direction),
    .result(dum,partial_result));
 defparam
    shifter_inst1.lpm_width = 16+1,
    shifter_inst1.lpm_widthdist = 4,
    shifter_inst1.lpm_shifttype="ARITHMETIC";
`endif

wire [17-1:0] partial_reg_q;
assign partial_reg_q = {dum_,partial_result_};
register_17 partial_reg
  ({dum,partial_result},clk,resetn,1'b1,partial_reg_q);

wire [5-1:0] sa_2;
wire shift_direction_2;

register_5 secondstage (sa, clk,resetn,1'b1,sa_2); 

register_1 secondstagedir (shift_direction, clk,resetn,1'b1,shift_direction_2); 

`ifdef USE_INHOUSE_LOGIC
wire [17-1:0] local_shifter_inst2_result;
assign {dum2,result} = local_shifter_inst2_result;

wire [2-1:0] local_shifter_inst2_distance;
assign local_shifter_inst2_distance = sa_2[(4-2)-1:0];

wire [17-1:0] local_shifter_inst2_data;
assign local_shifter_inst2_data = {dum_,partial_result_};

local_shifter_17_2_ARITHMATIC local_shifter_inst2(
  .data(local_shifter_inst2_data),
  .distance(local_shifter_inst2_distance),
  .direction(shift_direction_2),
  .result(local_shifter_inst2_result)
);
// defparam
//    local_shifter_inst2.LPM_WIDTH = 16+1,
//   local_shifter_inst2.LPM_WIDTHDIST = ((4-2)>0) ? (4-2) : 1,
//    local_shifter_inst2.LPM_SHIFTTYPE ="ARITHMETIC";
`else
lpm_clshift_17_2_ARITHMATIC shifter_inst2(
    .data({dum_,partial_result_}),
    .distance(sa_2[(((4-2)>0) ? (4-2)-1 : 0):0]),
    .direction(shift_direction_2),
    .result({dum2,resulti}));
 defparam 
    shifter_inst2.lpm_width = 16+1,
    shifter_inst2.lpm_widthdist = ((4-2)>0) ? (4-2) : 1,
    shifter_inst2.lpm_shifttype="ARITHMETIC";
`endif


endmodule

        
/****************************************************************************
          Base Register File

   - Has one read port (a) and one write port (c)
****************************************************************************/
module vregfile_base_32_16_4 (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

input clk;
input resetn;

input [4-1:0] a_reg,c_reg;
output [32-1:0] a_readdataout;
input [32-1:0] c_writedatain;
input a_en, c_we;

        ram_wrapper_4_16_32 reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[4-1:0]),
	    .address_b(a_reg[4-1:0]),
	    .wren_a(c_we),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout)
        );

endmodule

        

module bfloat_mult_16 (
 input clk,
 input resetn,
 input en,
 input stall,
 input [16-1:0] a,
 input [16-1:0] b,
 output reg[16-1:0] out
);

always@(posedge clk)begin
  if(!resetn)
    out <= 'h0;
  else
    if(en)
      out <= a * b;
end

endmodule
        
module activation_16(
 input clk,
 input resetn,
 input en,
 input stall,
 input [16-1:0] a,
 output reg[16-1:0] out
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

        
module qmult(i_multiplicand,i_multiplier,o_result);
input [`DWIDTH-1:0] i_multiplicand;
input [`DWIDTH-1:0] i_multiplier;
output [2*`DWIDTH-1:0] o_result;

assign o_result = i_multiplicand * i_multiplier;
//DW02_mult #(`DWIDTH,`DWIDTH) u_mult(.A(i_multiplicand), .B(i_multiplier), .TC(1'b1), .PRODUCT(o_result));

endmodule
