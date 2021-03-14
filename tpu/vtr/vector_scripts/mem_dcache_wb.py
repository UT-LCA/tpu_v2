from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class mem_dcache_wb():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self,log2dramwidthbits, log2cachewidthbits, log2cachedepth):
        string = ''' 

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
    cache_miss,
 
    mem_dcache_address,
    mem_dcache_data,
    mem_dcache_out,
    mem_dcache_byteen,
    mem_dcache_wren
    );

// Only need to set THESE TWO PARAMETERS
parameter {LOG2CACHELINESIZE}=7;            // In bits, subtract 3 for bytes
parameter {LOG2CACHEDEPTH}=6;
parameter {LOG2DRAMWIDTHBITS}=7;            // In bits, subtract 3 for bytes

parameter CACHELINESIZE=2**{LOG2CACHELINESIZE}; 
parameter CACHEDEPTH=2**{LOG2CACHEDEPTH};  
parameter DRAMWIDTHBITS=2**{LOG2DRAMWIDTHBITS};

parameter TAGSIZE=32-{LOG2CACHEDEPTH}-{LOG2CACHELINESIZE}+3;

`define TAGRANGE 31:32-TAGSIZE
`define OFFSETRANGE {LOG2CACHEDEPTH}-1+{LOG2CACHELINESIZE}-3:{LOG2CACHELINESIZE}-3
// selects 32-bit word
`define OFFSETWORDRANGE {LOG2CACHEDEPTH}-1+{LOG2CACHELINESIZE}-3:2


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
wire  [{LOG2CACHEDEPTH}-1:0] mem_ndx_saved;

wire  cache_hit;

reg  [{LOG2CACHEDEPTH}-1:0] count;

reg   [CACHELINESIZE/8-1:0]   bus_byteen_t;
wire [{LOG2CACHELINESIZE}-5-1:0] wordsel_saved;
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

 dpram1_22_67108864_512 data0 (
    .clk(bus_clk),
    .address_a(bus_address[28:7]),
    .address_b(mem_dcache_address),
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
  assign wordsel_saved=bus_address_saved[{LOG2CACHELINESIZE}-3:3];


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
endmodule


                 '''
        return string.format(LOG2DRAMWIDTHBITS = log2dramwidthbits , \
                             LOG2CACHELINESIZE = log2cachewidthbits , \
                             LOG2CACHEDEPTH = log2cachedepth , \
                             CBS = "{" , \
                             CBE = "}" \
                             )
    def write (self,log2dramwidthbits, log2cachewidthbits, log2cachedepth):
        self.fp.write(self.make_str(log2dramwidthbits, log2cachewidthbits, log2cachedepth))


if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = mem_dcache_wb(fp)
    uut1.write(32,32,4096)
    fp.close()
