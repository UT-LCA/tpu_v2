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

`include "mem_wbbuffer.v"

module mem_dcache_wb (

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
    cache_miss
    );

// Only need to set THESE TWO PARAMETERS
parameter LOG2CACHELINESIZE=7;            // In bits, subtract 3 for bytes
parameter LOG2CACHEDEPTH=6;
parameter LOG2DRAMWIDTHBITS=7;            // In bits, subtract 3 for bytes

parameter CACHELINESIZE=2**LOG2CACHELINESIZE; 
parameter CACHEDEPTH=2**LOG2CACHEDEPTH;  
parameter DRAMWIDTHBITS=2**LOG2DRAMWIDTHBITS;

parameter TAGSIZE=32-LOG2CACHEDEPTH-LOG2CACHELINESIZE+3;

`define TAGRANGE 31:32-TAGSIZE
`define OFFSETRANGE LOG2CACHEDEPTH-1+LOG2CACHELINESIZE-3:LOG2CACHELINESIZE-3
// selects 32-bit word
`define OFFSETWORDRANGE LOG2CACHEDEPTH-1+LOG2CACHELINESIZE-3:2


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

wire [31:0]              mem_dirtyaddr; 
wire [CACHELINESIZE-1:0] mem_dirtydata;
wire                     mem_dirtywe;

wire                     mem_validout;
wire                     mem_dirtyout;
wire [TAGSIZE-1:0]       mem_tagout;
wire [CACHELINESIZE-1:0] mem_lineout;

wire [TAGSIZE-1:0]       cache_tagout;
wire                     cache_validout;
wire [CACHELINESIZE-1:0] cache_dataout;

wire                     tagmatch;
reg  [32-1:0] bus_address_saved;
wire  [LOG2CACHEDEPTH-1:0] mem_ndx_saved;

wire  cache_hit;

reg  [LOG2CACHEDEPTH-1:0] count;

reg   [CACHELINESIZE/8-1:0]   bus_byteen_t;
wire [LOG2CACHELINESIZE-5-1:0] wordsel_saved;
reg  [31:0]  bus_readdata;
wire [CACHELINESIZE-1:0] bus_writedata_t;

reg [31:0]                  mem_wbaddr_r;  // captures address for writeback
reg [CACHELINESIZE-1:0]     mem_wbdata_r;
wire                        mem_wbwe_pulse;
wire                        mem_wbload;
reg [31:0]                  mem_filladdr_r; // don't prefetch over dirty lines
reg [31:0]                  mem_filladdr_r2;
wire                        t_mem_fillwe;

  always@(posedge mem_clk)
    if (bus_flush)
      count <= count + 1'b1;

  // Must flush for CACHEDEPTH cycles
//  altsyncram status
//    (
//     .clock0    (mem_clk),
//     .wren_a    ((bus_flush) ? 1'b1 : t_mem_fillwe),
//     .data_a    ((bus_flush) ? 1'b0 : 1'b1),
//     .address_a ((bus_flush) ? count : mem_filladdr[`OFFSETRANGE]),
//     .q_a       (mem_validout),
//     .clock1    (bus_clk),
//     .clocken1  (bus_en),
//     .address_b (bus_address[`OFFSETRANGE]),
//     .q_b       (cache_validout)
//    );
//  defparam
//   status.operation_mode = "BIDIR_DUAL_PORT",
//   status.width_a = 1, 
//   status.widthad_a = LOG2CACHEDEPTH,
//   status.width_b = 1,
//   status.widthad_b = LOG2CACHEDEPTH,
//   status.outdata_reg_a = "UNREGISTERED",
//   status.outdata_reg_b = "UNREGISTERED",
//   //status.rdcontrol_reg_a = "CLOCK0",
//   //status.address_reg_a = "CLOCK0",
//   status.rdcontrol_reg_b = "CLOCK1",
//   status.address_reg_b = "CLOCK1",
//   status.read_during_write_mode_mixed_ports = "OLD_DATA";
//
//  // Must flush for CACHEDEPTH cycles
//  altsyncram dirty
//    (
//     .clock0    (bus_clk),
//     .clocken0  (bus_en),
//     .address_a (bus_address[`OFFSETRANGE]),
//     .wren_a    (bus_dirtywe),
//     .data_a    (bus_dirty),
//
//     .clock1    (mem_clk),
//     .wren_b    (t_mem_fillwe),
//     .data_b    (1'b0),
//     .address_b (mem_filladdr[`OFFSETRANGE]),
//     .q_b       (mem_dirtyout)
//    );
//  defparam
//   dirty.operation_mode = "BIDIR_DUAL_PORT",
//   dirty.width_a = 1, 
//   dirty.widthad_a = LOG2CACHEDEPTH,
//   dirty.width_b = 1,
//   dirty.widthad_b = LOG2CACHEDEPTH,
//   dirty.outdata_reg_a = "UNREGISTERED",
//   dirty.outdata_reg_b = "UNREGISTERED",
//   //dirty.rdcontrol_reg_a = "CLOCK0",
//   //dirty.address_reg_a = "CLOCK0",
//   dirty.rdcontrol_reg_b = "CLOCK1",
//   dirty.address_reg_b = "CLOCK1",
//   dirty.read_during_write_mode_mixed_ports = "OLD_DATA";
//
//
//  altsyncram tags
//    (
//     .clock0    (mem_clk),
//     .wren_a    (t_mem_fillwe),
//     .data_a    (mem_filladdr[`TAGRANGE]),
//     .address_a (mem_filladdr[`OFFSETRANGE]),
//     .q_a       (mem_tagout),
//
//     .clock1    (bus_clk),
//     .clocken1  (bus_en),
//     .address_b (bus_address[`OFFSETRANGE]),
//     .q_b       (cache_tagout)
//    );
//  defparam
//   tags.operation_mode = "BIDIR_DUAL_PORT",
//   tags.width_a = TAGSIZE, 
//   tags.widthad_a = LOG2CACHEDEPTH,
//   tags.width_b = TAGSIZE,
//   tags.widthad_b = LOG2CACHEDEPTH,
//   tags.outdata_reg_a = "UNREGISTERED",
//   tags.outdata_reg_b = "UNREGISTERED",
//   //tags.rdcontrol_reg_a = "CLOCK0",
//   //tags.address_reg_a = "CLOCK0",
//   tags.rdcontrol_reg_b = "CLOCK1",
//   tags.address_reg_b = "CLOCK1",
//   tags.read_during_write_mode_mixed_ports = "OLD_DATA";

`ifdef USE_INHOUSE_LOGIC
 spram1 data0 (
    .clk(bus_clk),
    .address(bus_address[29:8]),
    .wren(bus_wren),
    .data(bus_writedata_t),
    .byteen(bus_byteen_t),
    .out(cache_dataout)
 );
 defparam 
    data0.AWIDTH=22,
    data0.NUM_WORDS = 67108864,
    data0.DWIDTH= 512;

`else
  altsyncram data
    (
     .clock0    (bus_clk),
     .clocken0  (bus_en),
     .wren_a    (bus_wren),
     .byteena_a (bus_byteen_t),
     .data_a    (bus_writedata_t),
     .address_a (bus_address[`OFFSETRANGE]),
     .q_a       (cache_dataout),
     .clock1    (mem_clk),
     .wren_b    (t_mem_fillwe),
     .data_b    (mem_filldata),
     .address_b (mem_filladdr[`OFFSETRANGE]),
     .q_b       (mem_lineout)
    );
  defparam
    data.operation_mode = "BIDIR_DUAL_PORT",
    data.width_a = CACHELINESIZE,                           //32-bit specific
    data.widthad_a = LOG2CACHEDEPTH,
    data.width_byteena_a = CACHELINESIZE/8,
    data.width_b = CACHELINESIZE,
    data.widthad_b = LOG2CACHEDEPTH,
    data.outdata_reg_a = "UNREGISTERED",
    data.outdata_reg_b = "UNREGISTERED",
    //data.rdcontrol_reg_a = "CLOCK0",
    //data.address_reg_a = "CLOCK0",
    data.rdcontrol_reg_b = "CLOCK1",
    data.address_reg_b = "CLOCK1";
`endif

    // Save offset to make sure still asking for same data
  always@(posedge bus_clk or negedge resetn)
    if (!resetn)
      bus_address_saved<=0;
    else 
      bus_address_saved<=bus_address;

  register memtag(mem_filladdr[`OFFSETRANGE],mem_clk,resetn,1'b1,mem_ndx_saved);
    defparam memtag.WIDTH=LOG2CACHEDEPTH;

//  assign tagmatch=(cache_tagout==bus_address_saved[`TAGRANGE]);

//  assign cache_hit = tagmatch && cache_validout;
  assign cache_hit = 1'b1;

  reg bus_en_r;
  always@(posedge bus_clk)
    bus_en_r<=bus_en;

//  assign cache_miss=bus_en_r && ~cache_hit;
  assign cache_miss= 1'b0;

//  assign bus_wait=bus_en_r & ~cache_hit;
  assign bus_wait= 1'b0;

  // DIRTY BIT processing
//  wire [LOG2CACHELINESIZE-3-1:0] zeros;
//  assign zeros=0;
//  assign mem_dirtydata=mem_lineout;
//  assign mem_dirtyaddr={mem_tagout,mem_ndx_saved,zeros}; 
//  assign mem_dirtywe=(mem_validout) & (mem_dirtyout);

  //Word muxing - assign byteena, datain, dataout
  assign wordsel_saved=bus_address_saved[LOG2CACHELINESIZE-3-1:2];


  always @*
      bus_byteen_t=bus_byteen;

  assign bus_writedata_t = bus_writedata;

  integer i;
  always @*
  begin
    bus_readdata<=cache_dataout[31:0];
    for (i=0; i<CACHELINESIZE/32; i=i+1)
      if (wordsel_saved==i)
        bus_readdata<=cache_dataout[32*i +: 32];
  end


  assign bus_readdata_line=cache_dataout;

  /********************* Write Back Buffer ***********************/

  /*
  * filladdr      ----Ox00----
  * fillrddirty   ____----____
  * wbwe_valid        ____----____
  * addrmatch     ------------0x00----
  */

  //reg wbwe_valid;
  //reg wbwe_valid_r;
  //always @(posedge mem_clk)
  //  wbwe_valid<=mem_fillrddirty;
  //always @(posedge mem_clk)
  //  wbwe_valid_r<=wbwe_valid;

  //always@(posedge mem_clk)
  //  mem_filladdr_r<={mem_filladdr[31:LOG2CACHELINESIZE-3],zeros};
  //always@(posedge mem_clk)
  //  mem_filladdr_r2<=mem_filladdr_r;

  ////Assume mem_dirtywe is ready
  //reg mem_dirtyaddrmatch;
  //reg mem_dirtywe_r;
  //reg [31:0] mem_dirtyaddr_r;
  //always@(posedge mem_clk)
  //  mem_dirtyaddr_r=mem_dirtyaddr;
  //always@*
  //  mem_dirtyaddrmatch=(mem_filladdr_r2[`TAGRANGE]==mem_dirtyaddr_r[`TAGRANGE]);
  //always@(posedge mem_clk)
  //  mem_dirtywe_r=mem_dirtywe;

  ////wbwe_valid - pulse from memory system saying new cache line is about to come
  ////mem_dirtywe - tells whether cache line currently targetted is dirty
  ////mem_dirtyaddrmatch - if prefetching data that already is cached and is
  ////dirty do not put in WB buffer (since we don't put it in cache)
  //assign mem_wbwe_pulse = wbwe_valid_r && mem_dirtywe_r && !mem_dirtyaddrmatch;

  ////Don't overwrite dirty line with prefetched line if it's the same address
  ////Cache line has to be >= 4 x Dram size to do this
  //reg mem_fillwe_block;
  //always@(posedge mem_clk)
  //  mem_fillwe_block = (mem_dirtywe_r && mem_dirtyaddrmatch);
  //assign t_mem_fillwe = mem_fillwe && !mem_fillwe_block;

  //assign mem_wbload=mem_wbwe_pulse; 
  //always @(posedge mem_clk)
  //  mem_wbaddr_r<=mem_dirtyaddr; 
  //always @(posedge mem_clk)
  //  mem_wbdata_r<=mem_dirtydata; 

  //reg resetn133;
  //always @(posedge mem_clk)
  //  resetn133<=resetn;

  //mem_wbbuffer wbbuffer(
  //  .clk(mem_clk),
  //  .resetn(resetn133),
  //  .wbaddr(mem_wbaddr_r),
  //  .wbdata(mem_wbdata_r),
  //  .wbwe(mem_wbload),
  //  .wbfull(),
  //  .mem_address(mem_wbaddr),
  //  .mem_writedata(mem_wbdata),
  //  .mem_wren(mem_wbwe),
  //  .mem_ack(mem_wback));
  //defparam 
  //  wbbuffer.LOG2WBDATAWIDTH=LOG2CACHELINESIZE,
  //  wbbuffer.LOG2MEMDATAWIDTH=LOG2DRAMWIDTHBITS;

endmodule

