
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
    
    scalar_dbus_address,
    scalar_dbus_en,
    scalar_dbus_writedata,
    
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
    mem_icache_wren,

    //DMA interface
    dma_dbus_address,   
    dma_dbus_readdata,  
    dma_dbus_writedata, 
    dma_dbus_byteen,
    dma_dbus_en,        
    dma_dbus_wren,      
    dma_dbus_prefetch,  
    dma_dbus_wait,      
    dma_dbus_data_valid,
    // AXI Interface
    M_AWID    ,
    M_AWADDR  ,
    M_AWLEN   ,
    M_AWSIZE  ,
    M_AWBURST ,
    M_AWLOCK  ,
    M_AWCACHE ,
    M_AWPROT  ,
    M_AWQOS   ,
    M_AWVALID ,
    M_AWREADY ,
    M_WDATA   ,
    M_WSTRB   ,
    M_WLAST   ,
    M_WVALID  ,
    M_WREADY  ,
    M_ARID    ,
    M_ARADDR  ,
    M_ARLEN   ,
    M_ARSIZE  ,
    M_ARBURST ,
    M_ARLOCK  ,
    M_ARCACHE ,
    M_ARPROT  ,
    M_ARQOS   ,
    M_ARVALID ,
    M_ARREADY ,
    M_RID     ,
    M_RDATA   ,
    M_RRESP   ,
    M_RLAST   ,
    M_RVALID  ,
    M_RREADY  ,
    M_BREADY  ,
    M_BVALID  ,
    M_BRESP   ,
    M_BID     ,

    S_AWID   , 
    S_AWADDR , 
    S_AWLEN  , 
    S_AWSIZE , 
    S_AWBURST, 
    S_AWLOCK , 
    S_AWCACHE, 
    S_AWPROT , 
    S_AWQOS  , 
    S_AWVALID, 
    S_AWREADY, 
    S_WDATA  , 
    S_WSTRB  , 
    S_WLAST  , 
    S_WVALID , 
    S_WREADY , 
    S_ARID   , 
    S_ARADDR , 
    S_ARLEN  , 
    S_ARSIZE , 
    S_ARBURST, 
    S_ARLOCK , 
    S_ARCACHE, 
    S_ARPROT , 
    S_ARQOS  , 
    S_ARVALID, 
    S_ARREADY, 
    S_RID    , 
    S_RDATA  , 
    S_RRESP  , 
    S_RLAST  , 
    S_RVALID , 
    S_RREADY , 
    S_BREADY , 
    S_BVALID , 
    S_BRESP  , 
    S_BID 
    );


parameter LOG2DCACHEWIDTHBITS=`LOG2DCACHEWIDTHBITS;
parameter DCACHEWIDTHBITS=2**LOG2DCACHEWIDTHBITS;
parameter LOG2DCACHEDEPTH=`LOG2DCACHEDEPTHBYTES -(`LOG2DCACHEWIDTHBITS-3);

parameter LOG2ICACHEWIDTHBITS=`LOG2ICACHEWIDTHBITS;
parameter ICACHEWIDTHBITS=2**LOG2ICACHEWIDTHBITS;
parameter LOG2ICACHEDEPTH=`LOG2ICACHEDEPTHBYTES -(`LOG2ICACHEWIDTHBITS-3);

parameter LOG2DRAMWIDTHBITS=`LOG2DRAMWIDTHBITS;
parameter DRAMWIDTHBITS=2**LOG2DRAMWIDTHBITS;

// AXI params


//
output [31:0] scalar_dbus_writedata;
output scalar_dbus_en;
output [31:0] scalar_dbus_address;

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

// DMA signals
output [31:0]                   dma_dbus_address;   
input  [DCACHEWIDTHBITS-1:0]    dma_dbus_readdata;  
output [DCACHEWIDTHBITS-1:0]    dma_dbus_writedata; 
output [DCACHEWIDTHBITS/8-1:0]  dma_dbus_byteen;
output                          dma_dbus_en;        
output                          dma_dbus_wren;      
output                          dma_dbus_prefetch;  
input                           dma_dbus_wait;      
input                           dma_dbus_data_valid;

// AXI interface
 output  [6 - 1:0]     M_AWID;                                                 
 output  [16- 1:0]     M_AWADDR;                                    
 output  [7:0]         M_AWLEN;                                                    
 output  [2:0]         M_AWSIZE;                                                   
 output  [1:0]         M_AWBURST;                                                  
 output                M_AWLOCK;                                                         
 output  [3:0]         M_AWCACHE;                                                  
 output  [2:0]         M_AWPROT;                                                   
 output  [3:0]         M_AWQOS;                                                    
 output                M_AWVALID;                                                        
 input                 M_AWREADY;                                                        
 output  [128-1 : 0]   M_WDATA;                                     
 output  [128/8-1 : 0] M_WSTRB;                                   
 output                M_WLAST;                                                          
 output                M_WVALID;                                                         
 input                 M_WREADY;                                                         

 output  [6-1 : 0]     M_ARID;                                                 
 output  [16-1 : 0]    M_ARADDR;                                    
 output  [7 : 0]       M_ARLEN;                                                  
 output  [2 : 0]       M_ARSIZE;                                                 
 output  [1 : 0]       M_ARBURST;                                                
 output                M_ARLOCK;                                                         
 output  [3 : 0]       M_ARCACHE;                                                
 output  [2 : 0]       M_ARPROT;                                                 
 output  [3 : 0]       M_ARQOS;                                                  
 output                M_ARVALID;                                                        
 input                 M_ARREADY;                                                        
 input   [6-1 : 0]     M_RID;                                         	    
 input   [128-1 : 0]   M_RDATA;                                     
 input   [1 : 0]       M_RRESP;                                                  
 input                 M_RLAST;                                                          
 input                 M_RVALID;                                                         
 output                M_RREADY;                                                         

 output                M_BREADY;                                                         
 input                 M_BVALID;                                                         
 input   [1 : 0]       M_BRESP;                                                  
 input   [6-1 : 0]     M_BID;                                                   

 input  [6 - 1:0]       S_AWID;                                                 
 input  [16- 1:0]       S_AWADDR;                                    
 input  [7:0]           S_AWLEN;                                                    
 input  [2:0]           S_AWSIZE;                                                   
 input  [1:0]           S_AWBURST;                                                  
 input                  S_AWLOCK;                                                         
 input  [3:0]           S_AWCACHE;                                                  
 input  [2:0]           S_AWPROT;                                                   
 input  [3:0]           S_AWQOS;                                                    
 input                  S_AWVALID;                                                        
 output                 S_AWREADY;                                                        
 input  [128-1 : 0]     S_WDATA;                                     
 input  [128/8-1 : 0]   S_WSTRB;                                   
 input                  S_WLAST;                                                          
 input                  S_WVALID;                                                         
 output                 S_WREADY;                                                         

 input  [6-1 : 0]       S_ARID;                                                 
 input  [16-1 : 0]      S_ARADDR;                                    
 input  [7 : 0]         S_ARLEN;                                                  
 input  [2 : 0]         S_ARSIZE;                                                 
 input  [1 : 0]         S_ARBURST;                                                
 input                  S_ARLOCK;                                                         
 input  [3 : 0]         S_ARCACHE;                                                
 input  [2 : 0]         S_ARPROT;                                                 
 input  [3 : 0]         S_ARQOS;                                                  
 input                  S_ARVALID;                                                        
 output                 S_ARREADY;                                                        
 output   [6-1 : 0]     S_RID;                                         	    
 output   [128-1 : 0]   S_RDATA;                                     
 output   [1 : 0]       S_RRESP;                                                  
 output                 S_RLAST;                                                          
 output                 S_RVALID;                                                         
 input                  S_RREADY;                                                         

 input                  S_BREADY;                                                         
 output                 S_BVALID;                                                         
 output   [1 : 0]       S_BRESP;                                                  
 output   [6-1 : 0]     S_BID;                                                  

// 
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

 

/*************************  AXI interface  **************************/


/*************************  Processor Core **************************/

  wire [31:0]    scalar_dbus_address; 
  wire [31:0]    scalar_dbus_readdata; 
  wire [31:0]    scalar_dbus_writedata;
  wire [3:0]    scalar_dbus_byteen; 
  wire     scalar_dbus_en;       
  wire     scalar_dbus_wren;    

 

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

    // AXI interface
    .M_AWID    (M_AWID    ),
    .M_AWADDR  (M_AWADDR  ),
    .M_AWLEN   (M_AWLEN   ),
    .M_AWSIZE  (M_AWSIZE  ),
    .M_AWBURST (M_AWBURST ),
    .M_AWLOCK  (M_AWLOCK  ),
    .M_AWCACHE (M_AWCACHE ),
    .M_AWPROT  (M_AWPROT  ),
    .M_AWQOS   (M_AWQOS   ),
    .M_AWVALID (M_AWVALID ),
    .M_AWREADY (M_AWREADY ),
    .M_WDATA   (M_WDATA   ),
    .M_WSTRB   (M_WSTRB   ),
    .M_WLAST   (M_WLAST   ),
    .M_WVALID  (M_WVALID  ),
    .M_WREADY  (M_WREADY  ),
    .M_ARID    (M_ARID    ),
    .M_ARADDR  (M_ARADDR  ),
    .M_ARLEN   (M_ARLEN   ),
    .M_ARSIZE  (M_ARSIZE  ),
    .M_ARBURST (M_ARBURST ),
    .M_ARLOCK  (M_ARLOCK  ),
    .M_ARCACHE (M_ARCACHE ),
    .M_ARPROT  (M_ARPROT  ),
    .M_ARQOS   (M_ARQOS   ),
    .M_ARVALID (M_ARVALID ),
    .M_ARREADY (M_ARREADY ),
    .M_RID     (M_RID     ),
    .M_RDATA   (M_RDATA   ),
    .M_RRESP   (M_RRESP   ),
    .M_RLAST   (M_RLAST   ),
    .M_RVALID  (M_RVALID  ),
    .M_RREADY  (M_RREADY  ),
    .M_BREADY  (M_BREADY  ),
    .M_BVALID  (M_BVALID  ),
    .M_BRESP   (M_BRESP   ),
    .M_BID     (M_BID     ),
    .S_AWID    (S_AWID   ), 
    .S_AWADDR  (S_AWADDR ), 
    .S_AWLEN   (S_AWLEN  ), 
    .S_AWSIZE  (S_AWSIZE ), 
    .S_AWBURST (S_AWBURST), 
    .S_AWLOCK  (S_AWLOCK ), 
    .S_AWCACHE (S_AWCACHE), 
    .S_AWPROT  (S_AWPROT ), 
    .S_AWQOS   (S_AWQOS  ), 
    .S_AWVALID (S_AWVALID), 
    .S_AWREADY (S_AWREADY), 
    .S_WDATA   (S_WDATA  ), 
    .S_WSTRB   (S_WSTRB  ), 
    .S_WLAST   (S_WLAST  ), 
    .S_WVALID  (S_WVALID ), 
    .S_WREADY  (S_WREADY ), 
    .S_ARID    (S_ARID   ), 
    .S_ARADDR  (S_ARADDR ), 
    .S_ARLEN   (S_ARLEN  ), 
    .S_ARSIZE  (S_ARSIZE ), 
    .S_ARBURST (S_ARBURST), 
    .S_ARLOCK  (S_ARLOCK ), 
    .S_ARCACHE (S_ARCACHE), 
    .S_ARPROT  (S_ARPROT ), 
    .S_ARQOS   (S_ARQOS  ), 
    .S_ARVALID (S_ARVALID), 
    .S_ARREADY (S_ARREADY), 
    .S_RID     (S_RID    ), 
    .S_RDATA   (S_RDATA  ), 
    .S_RRESP   (S_RRESP  ), 
    .S_RLAST   (S_RLAST  ), 
    .S_RVALID  (S_RVALID ), 
    .S_RREADY  (S_RREADY ), 
    .S_BREADY  (S_BREADY ), 
    .S_BVALID  (S_BVALID ), 
    .S_BRESP   (S_BRESP  ), 
    .S_BID     (S_BID    ),
    
    .scalar_dbus_address  (scalar_dbus_address  ),
    .scalar_dbus_readdata (scalar_dbus_readdata ),
    .scalar_dbus_writedata(scalar_dbus_writedata),
    .scalar_dbus_byteen   (scalar_dbus_byteen   ),
    .scalar_dbus_en       (scalar_dbus_en       ),
    .scalar_dbus_wren     (scalar_dbus_wren     ),

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
    .dbus_wait(dcpu_wait),
    
    .dma_dbus_address	(dma_dbus_address), 
    .dma_dbus_readdata	(dma_dbus_readdata), 
    .dma_dbus_writedata	(dma_dbus_writedata),
    .dma_dbus_byteen	(dma_dbus_byteen),
    .dma_dbus_en	(dma_dbus_en),       
    .dma_dbus_wren	(dma_dbus_wren),     
    .dma_dbus_prefetch	(dma_dbus_prefetch), 
    .dma_dbus_wait	(dma_dbus_wait),     
    .dma_dbus_data_valid (dma_dbus_data_valid)
    );


/*************************  Instr Cache **************************/

  reg cache_flush;
  always@(posedge mem_clk)
    cache_flush<=~resetn;

  mem_icache icache1 (
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

    .mem_icache_address  (scalar_dbus_address),
    .mem_icache_data     (scalar_dbus_writedata),
    .mem_icache_out      (scalar_dbus_readdata),
    .mem_icache_byteen   (scalar_dbus_byteen),
    .mem_icache_wren     (scalar_dbus_wren)
    );
    defparam icache1.LOG2CACHEDEPTH=LOG2ICACHEDEPTH,
      icache1.LOG2CACHELINESIZE=LOG2ICACHEWIDTHBITS;

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

  mem_dcache_wb dcache1 (
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
    defparam 
      dcache1.LOG2DRAMWIDTHBITS=LOG2DRAMWIDTHBITS,
      dcache1.LOG2CACHELINESIZE=LOG2DCACHEWIDTHBITS,
      dcache1.LOG2CACHEDEPTH=LOG2DCACHEDEPTH;

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
