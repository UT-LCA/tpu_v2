
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
   
    // AXI Interface
    // Read desciptor interface 
    s_axis_read_desc_addr,
    s_axis_read_desc_len,
    s_axis_read_desc_tag,
    s_axis_read_desc_id,
    s_axis_read_desc_dest,
    s_axis_read_desc_user,
    s_axis_read_desc_valid,
    s_axis_read_desc_ready,
  
    // AXI read descriptor status output
    m_axis_read_desc_status_tag,
    m_axis_read_desc_status_valid,

    // AXI stream read data output  
    m_axis_read_data_tdata,
    m_axis_read_data_tkeep,
    m_axis_read_data_tvalid,
    m_axis_read_data_tready,
    m_axis_read_data_tlast,
    m_axis_read_data_tid,
    m_axis_read_data_tdest,
    m_axis_read_data_tuser,

    // AXI write descriptor input
    s_axis_write_desc_addr,
    s_axis_write_desc_len,
    s_axis_write_desc_tag,
    s_axis_write_desc_valid,
    s_axis_write_desc_ready,
    
    // AXI write descriptor output
    m_axis_write_desc_status_len,
    m_axis_write_desc_status_tag,
    m_axis_write_desc_status_id,
    m_axis_write_desc_status_dest,
    m_axis_write_desc_status_user,
    m_axis_write_desc_status_valid,
    
    // AXI stream write data input
    s_axis_write_data_tdata,
    s_axis_write_data_tkeep,
    s_axis_write_data_tvalid,
    s_axis_write_data_tready,
    s_axis_write_data_tlast,
    s_axis_write_data_tid,
    s_axis_write_data_tdest,
    s_axis_write_data_tuser,

    //AXI master interface 
    m_axi_awid,
    m_axi_awaddr,
    m_axi_awlen,
    m_axi_awsize,
    m_axi_awburst,
    m_axi_awlock,
    m_axi_awcache,
    m_axi_awprot,
    m_axi_awvalid,
    m_axi_awready,
    m_axi_wdata,
    m_axi_wstrb,
    m_axi_wlast,
    m_axi_wvalid,
    m_axi_wready,
    m_axi_bid,
    m_axi_bresp,
    m_axi_bvalid,
    m_axi_bready,
    m_axi_arid,
    m_axi_araddr,
    m_axi_arlen,
    m_axi_arsize,
    m_axi_arburst,
    m_axi_arlock,
    m_axi_arcache,
    m_axi_arprot,
    m_axi_arvalid,
    m_axi_arready,
    m_axi_rid,
    m_axi_rdata,
    m_axi_rresp,
    m_axi_rlast,
    m_axi_rvalid,
    m_axi_rready,
   
    // AXI configuration 
    read_enable,
    write_enable,
    write_abort
    );


parameter LOG2DCACHEWIDTHBITS=`LOG2DCACHEWIDTHBITS;
parameter DCACHEWIDTHBITS=2**LOG2DCACHEWIDTHBITS;
parameter LOG2DCACHEDEPTH=`LOG2DCACHEDEPTHBYTES -(`LOG2DCACHEWIDTHBITS-3);

parameter LOG2ICACHEWIDTHBITS=`LOG2ICACHEWIDTHBITS;
parameter ICACHEWIDTHBITS=2**LOG2ICACHEWIDTHBITS;
parameter LOG2ICACHEDEPTH=`LOG2ICACHEDEPTHBYTES -(`LOG2ICACHEWIDTHBITS-3);

parameter LOG2DRAMWIDTHBITS=`LOG2DRAMWIDTHBITS;
parameter DRAMWIDTHBITS=2**LOG2DRAMWIDTHBITS;

// Width of AXI data bus in bits
parameter AXI_DATA_WIDTH = 32,
// Width of AXI address bus in bits
parameter AXI_ADDR_WIDTH = 16,
// Width of AXI wstrb (width of data bus in words)
parameter AXI_STRB_WIDTH = (AXI_DATA_WIDTH/8),
// Width of AXI ID signal
parameter AXI_ID_WIDTH = 8,
// Maximum AXI burst length to generate
parameter AXI_MAX_BURST_LEN = 16,
// Width of AXI stream interfaces in bits
parameter AXIS_DATA_WIDTH = AXI_DATA_WIDTH,
// Use AXI stream tkeep signal
parameter AXIS_KEEP_ENABLE = (AXIS_DATA_WIDTH>8),
// AXI stream tkeep signal width (words per cycle)
parameter AXIS_KEEP_WIDTH = (AXIS_DATA_WIDTH/8),
// Use AXI stream tlast signal
parameter AXIS_LAST_ENABLE = 1,
// Propagate AXI stream tid signal
parameter AXIS_ID_ENABLE = 0,
// AXI stream tid signal width
parameter AXIS_ID_WIDTH = 8,
// Propagate AXI stream tdest signal
parameter AXIS_DEST_ENABLE = 0,
// AXI stream tdest signal width
parameter AXIS_DEST_WIDTH = 8,
// Propagate AXI stream tuser signal
parameter AXIS_USER_ENABLE = 1,
// AXI stream tuser signal width
parameter AXIS_USER_WIDTH = 1,
// Width of length field
parameter LEN_WIDTH = 20,
// Width of tag field
parameter TAG_WIDTH = 8,
// Enable support for scatter/gather DMA
// (multiple descriptors per AXI stream frame)
parameter ENABLE_SG = 0,
// Enable support for unaligned transfers
parameter ENABLE_UNALIGNED = 0



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


// AXI interface

/*
 * AXI read descriptor input
 */
input  wire [AXI_ADDR_WIDTH-1:0]  s_axis_read_desc_addr,
input  wire [LEN_WIDTH-1:0]       s_axis_read_desc_len,
input  wire [TAG_WIDTH-1:0]       s_axis_read_desc_tag,
input  wire [AXIS_ID_WIDTH-1:0]   s_axis_read_desc_id,
input  wire [AXIS_DEST_WIDTH-1:0] s_axis_read_desc_dest,
input  wire [AXIS_USER_WIDTH-1:0] s_axis_read_desc_user,
input  wire                       s_axis_read_desc_valid,
output wire                       s_axis_read_desc_ready,

/*
 * AXI read descriptor status output
 */
output wire [TAG_WIDTH-1:0]       m_axis_read_desc_status_tag,
output wire                       m_axis_read_desc_status_valid,

/*
 * AXI stream read data output
 */
output wire [AXIS_DATA_WIDTH-1:0] m_axis_read_data_tdata,
output wire [AXIS_KEEP_WIDTH-1:0] m_axis_read_data_tkeep,
output wire                       m_axis_read_data_tvalid,
input  wire                       m_axis_read_data_tready,
output wire                       m_axis_read_data_tlast,
output wire [AXIS_ID_WIDTH-1:0]   m_axis_read_data_tid,
output wire [AXIS_DEST_WIDTH-1:0] m_axis_read_data_tdest,
output wire [AXIS_USER_WIDTH-1:0] m_axis_read_data_tuser,

/*
 * AXI write descriptor input
 */
input  wire [AXI_ADDR_WIDTH-1:0]  s_axis_write_desc_addr,
input  wire [LEN_WIDTH-1:0]       s_axis_write_desc_len,
input  wire [TAG_WIDTH-1:0]       s_axis_write_desc_tag,
input  wire                       s_axis_write_desc_valid,
output wire                       s_axis_write_desc_ready,

/*
 * AXI write descriptor status output
 */
output wire [LEN_WIDTH-1:0]       m_axis_write_desc_status_len,
output wire [TAG_WIDTH-1:0]       m_axis_write_desc_status_tag,
output wire [AXIS_ID_WIDTH-1:0]   m_axis_write_desc_status_id,
output wire [AXIS_DEST_WIDTH-1:0] m_axis_write_desc_status_dest,
output wire [AXIS_USER_WIDTH-1:0] m_axis_write_desc_status_user,
output wire                       m_axis_write_desc_status_valid,

/*
 * AXI stream write data input
 */
input  wire [AXIS_DATA_WIDTH-1:0] s_axis_write_data_tdata,
input  wire [AXIS_KEEP_WIDTH-1:0] s_axis_write_data_tkeep,
input  wire                       s_axis_write_data_tvalid,
output wire                       s_axis_write_data_tready,
input  wire                       s_axis_write_data_tlast,
input  wire [AXIS_ID_WIDTH-1:0]   s_axis_write_data_tid,
input  wire [AXIS_DEST_WIDTH-1:0] s_axis_write_data_tdest,
input  wire [AXIS_USER_WIDTH-1:0] s_axis_write_data_tuser,

/*
 * AXI master interface
 */
output wire [AXI_ID_WIDTH-1:0]    m_axi_awid,
output wire [AXI_ADDR_WIDTH-1:0]  m_axi_awaddr,
output wire [7:0]                 m_axi_awlen,
output wire [2:0]                 m_axi_awsize,
output wire [1:0]                 m_axi_awburst,
output wire                       m_axi_awlock,
output wire [3:0]                 m_axi_awcache,
output wire [2:0]                 m_axi_awprot,
output wire                       m_axi_awvalid,
input  wire                       m_axi_awready,
output wire [AXI_DATA_WIDTH-1:0]  m_axi_wdata,
output wire [AXI_STRB_WIDTH-1:0]  m_axi_wstrb,
output wire                       m_axi_wlast,
output wire                       m_axi_wvalid,
input  wire                       m_axi_wready,
input  wire [AXI_ID_WIDTH-1:0]    m_axi_bid,
input  wire [1:0]                 m_axi_bresp,
input  wire                       m_axi_bvalid,
output wire                       m_axi_bready,
output wire [AXI_ID_WIDTH-1:0]    m_axi_arid,
output wire [AXI_ADDR_WIDTH-1:0]  m_axi_araddr,
output wire [7:0]                 m_axi_arlen,
output wire [2:0]                 m_axi_arsize,
output wire [1:0]                 m_axi_arburst,
output wire                       m_axi_arlock,
output wire [3:0]                 m_axi_arcache,
output wire [2:0]                 m_axi_arprot,
output wire                       m_axi_arvalid,
input  wire                       m_axi_arready,
input  wire [AXI_ID_WIDTH-1:0]    m_axi_rid,
input  wire [AXI_DATA_WIDTH-1:0]  m_axi_rdata,
input  wire [1:0]                 m_axi_rresp,
input  wire                       m_axi_rlast,
input  wire                       m_axi_rvalid,
output wire                       m_axi_rready,

/*
 * Configuration
 */
input  wire                       read_enable,
input  wire                       write_enable,
input  wire                       write_abort

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

axi_dma #(
.AXI_DATA_WIDTH    (AXI_DATA_WIDTH    ), 
.AXI_ADDR_WIDTH    (AXI_ADDR_WIDTH    ), 
.AXI_STRB_WIDTH    (AXI_STRB_WIDTH    ), 
.AXI_ID_WIDTH      (AXI_ID_WIDTH      ), 
.AXI_MAX_BURST_LEN (AXI_MAX_BURST_LEN ), 
.AXIS_DATA_WIDTH   (AXIS_DATA_WIDTH   ), 
.AXIS_KEEP_ENABLE  (AXIS_KEEP_ENABLE  ), 
.AXIS_KEEP_WIDTH   (AXIS_KEEP_WIDTH   ), 
.AXIS_LAST_ENABLE  (AXIS_LAST_ENABLE  ),
.AXIS_ID_ENABLE    (AXIS_ID_ENABLE    ),
.AXIS_ID_WIDTH     (AXIS_ID_WIDTH     ),
.AXIS_DEST_ENABLE  (AXIS_DEST_ENABLE  ),
.AXIS_DEST_WIDTH   (AXIS_DEST_WIDTH   ),
.AXIS_USER_ENABLE  (AXIS_USER_ENABLE  ),
.AXIS_USER_WIDTH   (AXIS_USER_WIDTH   ),
.LEN_WIDTH         (LEN_WIDTH         ),
.TAG_WIDTH         (TAG_WIDTH         ),
.ENABLE_SG         (ENABLE_SG         ),
.ENABLE_UNALIGNED  (ENABLE_UNALIGNED  )
) inst_axi (
    .clk(clk),
    .rst(~resetn),
    // AXI Interface
    // Read desciptor interface 
    .s_axis_read_desc_addr		(s_axis_read_desc_addr ),
    .s_axis_read_desc_len		(s_axis_read_desc_len  ),
    .s_axis_read_desc_tag		(s_axis_read_desc_tag  ),
    .s_axis_read_desc_id		(s_axis_read_desc_id   ),
    .s_axis_read_desc_dest		(s_axis_read_desc_dest ),
    .s_axis_read_desc_user		(s_axis_read_desc_user ),
    .s_axis_read_desc_valid		(s_axis_read_desc_valid),
    .s_axis_read_desc_ready		(s_axis_read_desc_ready),
  
    // AXI read descriptor status output
    .m_axis_read_desc_status_tag	(m_axis_read_desc_status_tag  ),
    .m_axis_read_desc_status_valid	(m_axis_read_desc_status_valid),

    // AXI stream read data output  
    .m_axis_read_data_tdata		(m_axis_read_data_tdata	),
    .m_axis_read_data_tkeep		(m_axis_read_data_tkeep	),
    .m_axis_read_data_tvalid		(m_axis_read_data_tvalid	),
    .m_axis_read_data_tready		(m_axis_read_data_tready	),
    .m_axis_read_data_tlast		(m_axis_read_data_tlast	),
    .m_axis_read_data_tid		(m_axis_read_data_tid	),
    .m_axis_read_data_tdest		(m_axis_read_data_tdest	),
    .m_axis_read_data_tuser		(m_axis_read_data_tuser	),

    // AXI write descriptor input
    .s_axis_write_desc_addr		(s_axis_write_desc_addr ),
    .s_axis_write_desc_len		(s_axis_write_desc_len  ),
    .s_axis_write_desc_tag		(s_axis_write_desc_tag  ),
    .s_axis_write_desc_valid		(s_axis_write_desc_valid),
    .s_axis_write_desc_ready		(s_axis_write_desc_ready),
    
    // AXI write descriptor output
    .m_axis_write_desc_status_len	(m_axis_write_desc_status_len  ),
    .m_axis_write_desc_status_tag	(m_axis_write_desc_status_tag  ),
    .m_axis_write_desc_status_id	(m_axis_write_desc_status_id   ),
    .m_axis_write_desc_status_dest	(m_axis_write_desc_status_dest ),
    .m_axis_write_desc_status_user	(m_axis_write_desc_status_user ),
    .m_axis_write_desc_status_valid	(m_axis_write_desc_status_valid),
    
    // AXI stream write data input
    .s_axis_write_data_tdata		(s_axis_write_data_tdata	),
    .s_axis_write_data_tkeep		(s_axis_write_data_tkeep	),
    .s_axis_write_data_tvalid		(s_axis_write_data_tvalid	),
    .s_axis_write_data_tready		(s_axis_write_data_tready	),
    .s_axis_write_data_tlast		(s_axis_write_data_tlast	),
    .s_axis_write_data_tid		(s_axis_write_data_tid	),
    .s_axis_write_data_tdest		(s_axis_write_data_tdest	),
    .s_axis_write_data_tuser		(s_axis_write_data_tuser	),

    //AXI master interface 
    .m_axi_awid			(m_axi_awid	),
    .m_axi_awaddr		(m_axi_awaddr	),
    .m_axi_awlen		(m_axi_awlen	),
    .m_axi_awsize		(m_axi_awsize	),
    .m_axi_awburst		(m_axi_awburst	),
    .m_axi_awlock		(m_axi_awlock	),
    .m_axi_awcache		(m_axi_awcache	),
    .m_axi_awprot		(m_axi_awprot	),
    .m_axi_awvalid		(m_axi_awvalid	),
    .m_axi_awready		(m_axi_awready	),
    .m_axi_wdata		(m_axi_wdata	),
    .m_axi_wstrb		(m_axi_wstrb	),
    .m_axi_wlast		(m_axi_wlast	),
    .m_axi_wvalid		(m_axi_wvalid	),
    .m_axi_wready		(m_axi_wready	),
    .m_axi_bid			(m_axi_bid	),
    .m_axi_bresp		(m_axi_bresp	),
    .m_axi_bvalid		(m_axi_bvalid	),
    .m_axi_bready		(m_axi_bready	),
    .m_axi_arid			(m_axi_arid	),
    .m_axi_araddr		(m_axi_araddr	),
    .m_axi_arlen		(m_axi_arlen	),
    .m_axi_arsize		(m_axi_arsize	),
    .m_axi_arburst		(m_axi_arburst	),
    .m_axi_arlock		(m_axi_arlock	),
    .m_axi_arcache		(m_axi_arcache	),
    .m_axi_arprot		(m_axi_arprot	),
    .m_axi_arvalid		(m_axi_arvalid	),
    .m_axi_arready		(m_axi_arready	),
    .m_axi_rid			(m_axi_rid	),
    .m_axi_rdata		(m_axi_rdata	),
    .m_axi_rresp		(m_axi_rresp	),
    .m_axi_rlast		(m_axi_rlast	),
    .m_axi_rvalid		(m_axi_rvalid	),
    .m_axi_rready		(m_axi_rready	),
   
    // AXI configuration 
    .read_enable		(read_enable	),
    .write_enable		(write_enable	),
    .write_abort		(write_abort	)
)

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

    .mem_icache_address  (mem_icache_address),
    .mem_icache_data     (mem_icache_data   ),
    .mem_icache_out      (mem_icache_out    ),
    .mem_icache_byteen   (mem_icache_byteen ),
    .mem_icache_wren     (mem_icache_wren   )
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
