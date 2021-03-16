from mem_dcache_wb import mem_dcache_wb
from mem_icache import mem_icache
from vlanes import vlanes
from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class processor():
    def __init__(self, fp):
        self.fp = fp
        
    def make_str(self):
        log2dcachewidthbits = 32 
        log2dcachedepth = 4096
        log2icachewidthbits = 32
        log2icachedepth = 4096
        log2dramwidthbits = 128
        string = ''' 

`include "core.v"
`include "mem_icache.v"
`include "mem_dcache_wb.v"

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

parameter DCACHEWIDTHBITS=2**{LOG2DCACHEWIDTHBITS};

parameter ICACHEWIDTHBITS=2**{LOG2ICACHEWIDTHBITS};

parameter DRAMWIDTHBITS=2**{LOG2DRAMWIDTHBITS};

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
  always@(posedge mem_clk)
    cache_flush<=~resetn;

  mem_icache_{LOG2ICACHEDEPTH}_{LOG2ICACHEWIDTHBITS} icache1 (
    .resetn(resetn),

    // CPU data bus
    .bus_clk(clk),
    .bus_address(icache_address),
    .bus_readdata(icache_readdata),
    .bus_en(icache_en),
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
      icache_timeout<={CBS}4{CBS}1'b1{CBE}{CBE};
    else
      icache_timeout<=(ibus_wait<<3) | (icache_timeout>>1);

  // Send single pulse to bus, unless already a request stalling
  assign ibus_en= (icache_state==CACHE_ISSUE);

  assign ibus_address=icache_address;


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

  mem_dcache_wb_{LOG2DRAMWIDTHBITS}_{LOG2DCACHEWIDTHBITS}_{LOG2DCACHEDEPTH} dcache1 (
    .resetn(resetn),

    // CPU data bus    
    .bus_clk(clk),
    .bus_address(dcache_address),
    .bus_writedata(dcache_writedata),
    .bus_byteen(dcache_byteen),
    .bus_wren(dcache_wren),
    .bus_dirty(1'b1),   //Every store is dirty (even on store miss)
    .bus_dirtywe(dcache_wren),
    .bus_readdata(dcache_readdata),
    .bus_readdata_line(dcache_readdata_line),
    .bus_en(dcache_en),

    // Mem hierarchy
    .mem_clk(mem_clk),
    .mem_filladdr(dmem_filladdr),
    .mem_filldata(dmem_filldata),
    .mem_fillwe(dmem_fillwe),
    .mem_fillrddirty(dmem_fillrddirty),
    .bus_flush(cache_flush), // Runs at 133 MHz

    // Write back outputs for writing into DRAM, wbwe accounts for dirty bit
    .mem_wbaddr(dmem_wbaddr),
    .mem_wbdata(dmem_wbdata),
    .mem_wbwe(dmem_wbwe),
    .mem_wback(dmem_wback),

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
  multicyclestall staller(dcpu_en,
      dcache_wait|(dcpu_address_r[31]&dbus_wait),
      clk,resetn,dcpu_wait);

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
      dcache_timeout<={CBS}4{CBS}1'b1{CBE}{CBE};
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

                 '''
        fp1 = open("verilog/mem_dcache_wb.v", "w")
        uut2 = mem_dcache_wb(fp1)
        uut2.write(log2dramwidthbits,log2dcachewidthbits,log2dcachedepth)
        fp2 = open("verilog/mem_icache.v", "w")
        uut3 = mem_icache(fp2)
        uut3.write(log2icachedepth,log2icachewidthbits)
        return string.format(LOG2DCACHEWIDTHBITS = log2dcachewidthbits ,\
                             LOG2DCACHEDEPTH = log2dcachedepth , \
                             LOG2ICACHEWIDTHBITS = log2icachewidthbits , \
                             LOG2ICACHEDEPTH = log2icachedepth , \
                             LOG2DRAMWIDTHBITS = log2dramwidthbits , \
                             CBS = "{" , \
                             CBE = "}" \
                             )
    def write (self):
        self.fp.write(self.make_str())


if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = processor(fp)
    uut1.write()
    fp.close()
