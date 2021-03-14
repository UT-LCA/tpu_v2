from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class mem_icache():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, log2cachewidthbits, log2cachedepth):
        string = ''' 
/******
 * 1-cycle cache (not including initial registering)
 *    0th cycle: register addresses into block RAMs
 *    1st cycle: Read out tags and status and do comparison
 */

module mem_icache_{LOG2CACHELINESIZE}_{LOG2CACHEDEPTH} (

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
parameter CACHELINESIZE=2**{LOG2CACHELINESIZE}; // 128

parameter CACHEDEPTH=2**{LOG2CACHEDEPTH};  // 64

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

reg  [{LOG2CACHEDEPTH}-1:0] count;

  always@(posedge mem_clk)
    if (bus_flush)
      count <= count + 1'b1;

 dpram1_26_67108864_32 data1 (
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

                 '''
        return string.format( \
                             LOG2CACHELINESIZE = log2cachewidthbits , \
                             LOG2CACHEDEPTH = log2cachedepth , \
                             CBS = "{" , \
                             CBE = "}" \
                             )
    def write (self, log2cachewidthbits, log2cachedepth):
        self.fp.write(self.make_str( log2cachewidthbits, log2cachedepth))


if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = mem_icache(fp)
    uut1.write(32,4096)
    fp.close()
