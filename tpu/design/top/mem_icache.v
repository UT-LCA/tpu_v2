/******
 * 1-cycle cache (not including initial registering)
 *    0th cycle: register addresses into block RAMs
 *    1st cycle: Read out tags and status and do comparison
 */

module mem_icache (

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
parameter LOG2CACHELINESIZE=7;
parameter CACHELINESIZE=2**LOG2CACHELINESIZE; // 128

parameter LOG2CACHEDEPTH=6;
parameter CACHEDEPTH=2**LOG2CACHEDEPTH;  // 64

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

wire [TAGSIZE-1:0]       cache_tagout;
wire                     cache_validout;
wire [32-1:0]            cache_dataout;
wire [CACHELINESIZE-1:0] cache_lineout;

input  [31:0]                mem_icache_address;
input  [CACHELINESIZE-1:0]   mem_icache_data;
output [31:0]                mem_icache_out;
input  [CACHELINESIZE/8-1:0] mem_icache_byteen;
input                        mem_icache_wren;

wire                     tagmatch;
reg  [32-1:0] bus_address_saved;

parameter [1:0] S_IDLE=2'b00, S_TAGLOOKUP=2'b01, S_RESULT=2'b10;
reg [1:0] cache_state;

wire  cache_hit;

reg  [LOG2CACHEDEPTH-1:0] count;

  always@(posedge mem_clk)
    if (bus_flush)
      count <= count + 1'b1;

//  // Must flush for CACHEDEPTH cycles
//  altsyncram status
//    (
//     .clock0    (mem_clk),
//     .wren_a    ((bus_flush) ? 1'b1 : mem_fillwe),
//     .data_a    ((bus_flush) ? 0 : 1),
//     .address_a ((bus_flush) ? count : mem_filladdr[`OFFSETRANGE]),
//     .clock1    (bus_clk),
//     .clocken1  (bus_en),
//     .address_b (bus_address[`OFFSETRANGE]),
//     .q_b       (cache_validout)
//    );
//  defparam
//   status.operation_mode = "DUAL_PORT",
//   status.width_a = 1, 
//   status.widthad_a = LOG2CACHEDEPTH,
//   status.width_b = 1,
//   status.widthad_b = LOG2CACHEDEPTH,
//   status.outdata_reg_a = "UNREGISTERED",
//   status.outdata_reg_b = "UNREGISTERED",
//   //status.rdcontrol_reg_a = "CLOCK0",
//   status.rdcontrol_reg_b = "CLOCK1",
//   //status.address_reg_a = "CLOCK0",
//   status.address_reg_b = "CLOCK1",
//   status.read_during_write_mode_mixed_ports = "OLD_DATA";
//
//
//  altsyncram tags
//    (
//     .clock0    (mem_clk),
//     .wren_a    (mem_fillwe),
//     .data_a    (mem_filladdr[`TAGRANGE]),
//     .address_a (mem_filladdr[`OFFSETRANGE]),
//
//     .clock1    (bus_clk),
//     .clocken1  (bus_en),
//     .address_b (bus_address[`OFFSETRANGE]),
//     .q_b       (cache_tagout)
//    );
//  defparam
//   tags.operation_mode = "DUAL_PORT",
//   tags.width_a = TAGSIZE, 
//   tags.widthad_a = LOG2CACHEDEPTH,
//   tags.width_b = TAGSIZE,
//   tags.widthad_b = LOG2CACHEDEPTH,
//   tags.outdata_reg_a = "UNREGISTERED",
//   tags.outdata_reg_b = "UNREGISTERED",
//   //tags.rdcontrol_reg_a = "CLOCK0",
//   tags.rdcontrol_reg_b = "CLOCK1",
//   //tags.address_reg_a = "CLOCK0",
//   tags.address_reg_b = "CLOCK1",
//   tags.read_during_write_mode_mixed_ports = "OLD_DATA";

`ifdef USE_INHOUSE_LOGIC
 dpram1 data1 (
    .clk(bus_clk),
    .address_a(bus_address[27:2]),
    .address_b(mem_icache_address),
    .wren_a(1'b0),
    .wren_b(mem_icache_wren),
    .data_a(0),
    .data_b(mem_icache_data),
    .byteen_a(-1),
    .byteen_b(mem_icache_byteen),
    .out_a(cache_dataout),
    .out_b(meme_icache_out)
 );
 defparam 
    data1.AWIDTH=26,
    data1.NUM_WORDS = 67108864,
    data1.DWIDTH= 32;

`else

  altsyncram data
    (
     .clock0    (bus_clk),
     .clocken0  (bus_en),
     .wren_a    (bus_wren),
     //.byteena_a (bus_byteen),
     .data_a    (bus_writedata),
     .address_a (bus_address[`OFFSETWORDRANGE]),
     .q_a       (cache_dataout),
     .clock1    (mem_clk),
     .wren_b    (mem_fillwe),
     .data_b    (mem_filldata),
     .address_b (mem_filladdr[`OFFSETRANGE])
     //.q_b       (cache_lineout)
    );
  defparam
    data.operation_mode = "BIDIR_DUAL_PORT",
    data.width_a = 32,                                   //32-bit specific
    data.widthad_a = LOG2CACHEDEPTH+(LOG2CACHELINESIZE-5),  //32-bit specific
    //data.WIDTH_BYTEENA_A = 4,
    data.width_b = CACHELINESIZE,
    data.widthad_b = LOG2CACHEDEPTH,
    data.outdata_reg_a = "UNREGISTERED",
    data.outdata_reg_b = "UNREGISTERED",
    //data.rdcontrol_reg_a = "CLOCK0",
    data.rdcontrol_reg_b = "CLOCK1",
    //data.address_reg_a = "CLOCK0",
    data.address_reg_b = "CLOCK1";

`endif

  assign bus_readdata=cache_dataout;

    // Save offset to make sure still asking for same data
  always@(posedge bus_clk or negedge resetn)
    if (!resetn)
      bus_address_saved<=0;
    else 
      bus_address_saved<=bus_address;

//  assign tagmatch=(cache_tagout==bus_address_saved[`TAGRANGE]);

//  assign cache_hit = tagmatch && cache_validout;
  assign cache_hit = 1'b1;

  assign cache_miss=bus_en && ~cache_hit;

  assign bus_wait=bus_en & ~cache_hit;


endmodule

