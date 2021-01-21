
`include "options.v"
/****
 * 
 *   1st stage: bus_
 *   2nd stage: mem_
 *   3rd stage: jddr_
 *
 */

/*
 * Reset_n resetn
 *   0       0     In reset and bootload reset
 *   0       1     In reset state but not bootload reset 
 *   1       0     INVALID
 *   1       1     Running
 */

module mem_hierarchy (

    procresetn,      // In reset state

    // From CPU data bus
    bus_clk,
    resetn,       // In reset state and in bootload reset state

    dbus_address,
    dbus_en,
    dbus_wait,
    dbus_prefetch, //{stride[15:0] in bytes, num_accesses[15:0] (vector length)}

    // Data fill/WB 133MHz
    clk133,
    dmem_filladdr,
    dmem_filldata,
    dmem_fillwe,
    dmem_fillrddirty,
    dmem_wbaddr,
    dmem_wbdata,
    dmem_wbwe,
    dmem_wback,
    

    // From CPU instr bus
    ibus_en,
    ibus_address,
    ibus_wait,

    imem_filladdr,
    imem_filldata,
    imem_fillwe,

    // From bootloader (bypassing cache)
    bootload_address,
    bootload_writedata,
    bootload_byteen,
    bootload_wren,
    bootload_en,
    bootload_wait,

    // From Memory

    // TM4 signals needed by jddr interface
    tm4_glbclk,
    tm4_resetn,

    // DDR Interface Signals 
    mem_odt,
    mem_cs_n,
    mem_cke,
    mem_addr,
    mem_ba,
    mem_ras_n,
    mem_cas_n,
    mem_we_n,
    mem_dm,
    mem_clk,
    mem_clk_n,
    mem_dq,
    mem_dqs,
    mem_dqsn
    );

parameter DEFAULTDCACHEPREFETCHES=`DEFAULTDCACHEPREFETCHES;
parameter LOG2DATAWBBUFFERSIZE=`LOG2DATAWBBUFFERSIZE;

parameter LOG2DCACHEWIDTHBITS=`LOG2DCACHEWIDTHBITS;
parameter DCACHEWIDTHBITS=2**LOG2DCACHEWIDTHBITS;
parameter LOG2DCACHEDEPTHBYTES=`LOG2DCACHEDEPTHBYTES;
parameter DCACHEDEPTHBYTES=2**LOG2DCACHEDEPTHBYTES;

parameter LOG2ICACHEWIDTHBITS=`LOG2ICACHEWIDTHBITS;
parameter ICACHEWIDTHBITS=2**`LOG2ICACHEWIDTHBITS;

parameter LOG2DRAMWIDTHBITS=`LOG2DRAMWIDTHBITS;
parameter DRAMWIDTHBITS=2**LOG2DRAMWIDTHBITS;

parameter CPU_DATAWIDTH=32;
parameter CPU_ADDRWIDTH=32;

parameter DRAMROWSIZE=16;   //29:14 since 1GB RAM
parameter DRAMCOLSIZE=14;   //13:4 column address, + interleave, + precharge

`define DRAMCOLRANGE DRAMCOLSIZE-1:LOG2DRAMWIDTHBITS-3
`define DRAMROWRANGE DRAMROWSIZE+DRAMCOLSIZE-1:DRAMCOLSIZE

input           procresetn;

// CPU data bus
input           bus_clk;
input           resetn;
input   [CPU_ADDRWIDTH-1:0]  dbus_address;
input           dbus_en;
input   [31:0]  dbus_prefetch;
output          dbus_wait;

    // Mem hierarchy
output clk133;
output [31:0]    dmem_filladdr;
output [DCACHEWIDTHBITS-1:0]   dmem_filldata;
output           dmem_fillwe;
output           dmem_fillrddirty;

input [31:0]   dmem_wbaddr;
input [DRAMWIDTHBITS-1:0]  dmem_wbdata;
input          dmem_wbwe;
output         dmem_wback;

// CPU instr bus
input           ibus_en;
input  [31:0]   ibus_address;
output           ibus_wait;

output [31:0]    imem_filladdr;
output [ICACHEWIDTHBITS-1:0]   imem_filldata;
output           imem_fillwe;

input [31:0] bootload_address;
input [31:0] bootload_writedata;
input [3:0] bootload_byteen;
input bootload_wren;
input bootload_en;
output bootload_wait;


// TM4 signals needed by jddr interface
input tm4_glbclk;
input tm4_resetn;

// DDR Interface Signals for DDR slot A
output  [0:0] mem_odt;
output  [0:0] mem_cs_n;
output  [0:0] mem_cke;
output  [12:0]  mem_addr;
output  [1:0] mem_ba;
output    mem_ras_n;
output    mem_cas_n;
output    mem_we_n;
output  [7:0] mem_dm;
inout [1:0] mem_clk;
inout [1:0] mem_clk_n;
inout [63:0]  mem_dq;
inout [7:0] mem_dqs;
inout [7:0] mem_dqsn;

reg tm4_resetn133;

wire dbus_wait_r;
wire ibus_wait_r;
wire dbus_idle;
wire ibus_idle;

wire [31:0]                        dmem_address;
wire [CPU_DATAWIDTH-1:0]           dmem_writedata;
wire [3:0]                         dmem_byteen;
wire [15:0]                        dmem_numblocks;
wire                               dmem_wren;
wire                               dmem_en;
wire                               dmem_ack;
reg                                dmem_wait; 

wire [31:0]                        imem_address;
wire                               imem_en;
wire                               imem_ack;
wire [15:0]                        imem_numblocks;
reg                                imem_wait; 

reg [31:0]                       mem_address;
reg [CPU_DATAWIDTH-1:0]          mem_writedata;
reg [3:0]                        mem_byteen;
reg [15:0]                       mem_numblocks;
reg                              mem_wren;
reg                              mem_en;
reg                              mem_ack;

reg                               mem_wait; // uses clk133

wire [LOG2DRAMWIDTHBITS-3-1-2:0] mem_wordsel;

wire local_ready;
wire local_init_done;

reg mem_dbus_sel;
reg dbus_sel;

reg [3:0] mem_state;
parameter [3:0] MEM_IDLE=4'b0000, 
                MEM_READ=4'b0001, 
                MEM_READWAIT1=4'b0010, 
                MEM_READWAIT2=4'b0011, 
                MEM_WRITE=4'b0100, 
                MEM_WRITEWAIT1=4'b0101,
                MEM_WRITEWAIT2=4'b0110,
                MEM_WB=4'b1001, 
                MEM_WBWAIT1=4'b1010,
                MEM_WBWAIT2=4'b1011,
                MEM_DONE=4'b1000;

reg [DRAMWIDTHBITS/8-1:0]        fillbuf_byteen;

wire readwriten;
reg transrequest;
wire jddr_data_available;
wire jddr_data_wanted;
wire [DRAMWIDTHBITS-1:0]             jddr_read_data;
wire [DRAMWIDTHBITS-1:0]             jddr_write_data;
reg  [DRAMWIDTHBITS/8-1:0]           jddr_byteena;

reg [DCACHEWIDTHBITS-1:0]                dmem_jddr_read_data_r;
reg [ICACHEWIDTHBITS-1:0]                imem_jddr_read_data_r;
reg [DCACHEWIDTHBITS/DRAMWIDTHBITS-1:0]  dmem_jddr_data_available_r;
reg [ICACHEWIDTHBITS/DRAMWIDTHBITS-1:0]  imem_jddr_data_available_r;
reg [DRAMWIDTHBITS-1:0]                  jddr_read_data_bug;


reg [31:0]                  mem_address_t;  // captures address for writeback
reg [31:0]                  mem_address_offset;
reg [15:0]                  mem_address_recvd;
reg [31:0]                  mem_numblocks_r; 
reg                         mem_numblocks_not0_r; 
reg                         mem_coalesced; 
wire                        mem_address_doneissue;
wire                        mem_address_donerecv;

wire [31:0]                 mem_filladdr;
reg  [31:0]                 imem_filladdr;
reg  [31:0]                 dmem_filladdr;

wire mem_wbsel;
reg  dmem_wback;


reg Resetn133;

  always@(posedge clk133)
    Resetn133<=procresetn;

/*************************  Instr Cache Stage **************************/

  always @(posedge clk133)
    imem_filladdr=mem_filladdr;
  assign imem_filldata=imem_jddr_read_data_r;
  assign imem_fillwe=(&imem_jddr_data_available_r)&~mem_dbus_sel;

  // Fill Data - Buffer all DRAM words needed for a single cache line
  //Shift in jddr_read_data
  always @(posedge clk133)
    if (!tm4_resetn133)
      imem_jddr_read_data_r<=0;
    else if (jddr_data_available)
      imem_jddr_read_data_r<=
        (jddr_read_data_bug<<(ICACHEWIDTHBITS-DRAMWIDTHBITS)) | 
        (imem_jddr_read_data_r>>DRAMWIDTHBITS);

  //Shift in jddr_data_available, but reset when all 1's
  always @(posedge clk133)
    if (!tm4_resetn133)
      imem_jddr_data_available_r<=0;
    else if (&imem_jddr_data_available_r)
      imem_jddr_data_available_r<=
          (jddr_data_available<<(ICACHEWIDTHBITS/DRAMWIDTHBITS-1));
    else if (jddr_data_available&~mem_dbus_sel)
      imem_jddr_data_available_r<=
        (jddr_data_available<<(ICACHEWIDTHBITS/DRAMWIDTHBITS-1)) | 
        (imem_jddr_data_available_r>>1);

/*************************  Data Cache Stage **************************/

  always @(posedge clk133)
    dmem_filladdr=mem_filladdr;
  assign dmem_filldata=dmem_jddr_read_data_r;
  assign dmem_fillwe=(&dmem_jddr_data_available_r)&mem_dbus_sel&Resetn133;

  //Read dirty bit when first DRAM word comes in - if cache line size is
  //equal to one DRAM word, rddirty before it comes in since we know we
  //can't prefetch in this case anyway.
  assign dmem_fillrddirty=
    ((DCACHEWIDTHBITS/DRAMWIDTHBITS == 1) ?
      jddr_data_available :
      (dmem_jddr_data_available_r==1<<(DCACHEWIDTHBITS/DRAMWIDTHBITS-1))) &
    mem_dbus_sel&Resetn133;

  // Fill Data - Buffer all DRAM words needed for a single cache line
    //Shift in jddr_read_data
  always @(posedge clk133)
    if (!tm4_resetn133)
      dmem_jddr_read_data_r<=0;
    else if (jddr_data_available)
      dmem_jddr_read_data_r<=
        (jddr_read_data_bug<<(DCACHEWIDTHBITS-DRAMWIDTHBITS)) | 
        (dmem_jddr_read_data_r>>DRAMWIDTHBITS);

  //Shift in jddr_data_available, but reset when all 1's
  //We want to fillwe only when the whole cache line is available
  always @(posedge clk133)
    if (!tm4_resetn133)
      dmem_jddr_data_available_r<=0;
    else if (&dmem_jddr_data_available_r)
      dmem_jddr_data_available_r<=
          (jddr_data_available<<(DCACHEWIDTHBITS/DRAMWIDTHBITS-1));
    else if (jddr_data_available&mem_dbus_sel&Resetn133)
      dmem_jddr_data_available_r<=
        (jddr_data_available<<(DCACHEWIDTHBITS/DRAMWIDTHBITS-1)) | 
        (dmem_jddr_data_available_r>>1);

  //WTF IS it with Modelsim not registering jddr_read_data?????
  //If we don't assign to another signal then jddr_read_data will warp
  //through any number of clock edges in Modelsim!!!!
  always @*
    jddr_read_data_bug=jddr_read_data;
    

  wire [31:0] t_address;
  wire [31:0] t_writedata;
  wire [3:0]  t_byteen;
  wire [15:0] t_numblocks;
  wire [31:0] t_prefetch;
  wire t_wren;
  wire t_en;


  // Multiplex between processor and bootloader
  assign t_address=(!procresetn) ? bootload_address : 
    {dbus_address[31:LOG2DCACHEWIDTHBITS-3], {LOG2DCACHEWIDTHBITS-3{1'b0}}};
  assign t_writedata=(!procresetn) ? bootload_writedata : 0 ;
  assign t_byteen=(!procresetn) ? bootload_byteen : 4'b0000;
  assign t_prefetch=(!procresetn) ? 0 : 0;
  assign t_wren=(!procresetn) ? bootload_wren : 1'b0;
  assign t_en=(!procresetn) ? bootload_en : dbus_en;
  assign bootload_wait=dbus_wait_r;
  assign dbus_wait = dbus_wait_r;

  //Use Dcache size to bound number of prefetches, cap at 16 bits
  parameter LOG2MAXDPREFETCHBYTES=(LOG2DCACHEDEPTHBYTES>15) ? LOG2DCACHEDEPTHBYTES : 16;

  //Calculate total contiguous bytes to prefetch, cap at WB BUFFERSIZE
  wire [15:0] dbus_bytestoaccess; 
  assign dbus_bytestoaccess=(2**(LOG2DRAMWIDTHBITS-3+LOG2DATAWBBUFFERSIZE)-1) &
    dbus_prefetch[LOG2MAXDPREFETCHBYTES-1:0] *
      dbus_prefetch[16 +: LOG2DCACHEWIDTHBITS-3+1];

  //********** Prefetch as follows:
  // if (stride==0 || stride > dcachelinesize)
  //   use default prefetch amount
  // else
  //   use stride*length/DRAMwordsize
  assign t_numblocks=(!procresetn) ? 0 : 
    (!(|dbus_prefetch[31:LOG2DCACHEWIDTHBITS-3+1+16]) && 
      |dbus_prefetch[16 +: LOG2DCACHEWIDTHBITS-3+1]) ? 
          //   use stride*length/DRAMwordsize, pad to end of cache line
          ((dbus_bytestoaccess>>(LOG2DRAMWIDTHBITS-3)))|
              ~(32'hffffffff<<(LOG2DCACHEWIDTHBITS-LOG2DRAMWIDTHBITS)) :
          //   use default prefetch amount
          (DCACHEWIDTHBITS/DRAMWIDTHBITS)*(1+DEFAULTDCACHEPREFETCHES)-1;

/*************************  Data BUS Stage **************************/

  //Warning: A bus request may get ignored if the wbbuffer has stores to do
  //We assume the bus request will continuously re-issue until satisfied
  mem_port dport (
          .clk(bus_clk),
          .resetn(resetn),
          .bus_address(t_address),
          .bus_writedata(t_writedata),
          .bus_byteen(t_byteen),
          .bus_numblocks(t_numblocks),
          //{stride[15:0] in bytes, length[15:0] in # mem accesses}
          .bus_prefetch(t_prefetch),
          .bus_en(t_en),
          .bus_wren(t_wren),
          .bus_wait_r(dbus_wait_r),  // goes high 1 cycle AFTER bus_en
          .bus_idle(dbus_idle),  
          .clk_mem(clk133),
          .resetn_mem(Resetn133 ),
          .mem_address(dmem_address),
          .mem_writedata(dmem_writedata),
          .mem_byteen(dmem_byteen),
          .mem_numblocks(dmem_numblocks),
          .mem_wren(dmem_wren),
          .mem_en(dmem_en),
          .mem_ack(dmem_ack),
          .mem_wait(dmem_wait));


/*************************  Instruction BUS Stage **************************/

  mem_port iport (
          .clk(bus_clk),
          .resetn(resetn),
          .bus_address( {ibus_address[31:LOG2ICACHEWIDTHBITS-3], 
                          {LOG2ICACHEWIDTHBITS-3{1'b0}}}),
          .bus_en(ibus_en),
          .bus_numblocks(ICACHEWIDTHBITS/DRAMWIDTHBITS-1),
          .bus_prefetch(0),
          .bus_wait_r(ibus_wait_r),  // goes high 1 cycle AFTER bus_en
          .bus_idle(ibus_idle),  
          .clk_mem(clk133),
          .resetn_mem(Resetn133 ),
          .mem_numblocks(imem_numblocks),
          .mem_address(imem_address),
          .mem_en(imem_en),
          .mem_ack(imem_ack),
          .mem_wait(imem_wait));

  assign ibus_wait=ibus_wait_r;

/*************************  MEM Stage **************************/

  always @( posedge clk133 )
    tm4_resetn133<=tm4_resetn;

  /********** Arbiter between instr and data busses ********/
  //WARNING: Shouldn't use logic when crossing clock domains, but hey
  always @( posedge bus_clk )
    if (!tm4_resetn)
      dbus_sel<=0;
    else if (dbus_wait_r && ibus_wait_r) // if busy hold arbiter
      dbus_sel<=dbus_sel;
    else if (dbus_wait_r || ibus_wait_r)  // if busy hold arbiter
      dbus_sel<=dbus_wait_r;
    //Doesn't work with mem_port prefetching!!! ibus_en signal can be low
    //while prefetching instrs, but a sudden dbus_en will switch the arbiter!
    else if (dbus_en && ibus_en)    // On contention, round robin
      dbus_sel<=~dbus_sel; 
    else if (dbus_en || ibus_en)    // Else assign to whomever is requesting
      dbus_sel<=dbus_en;

  always @(posedge clk133)
  begin
    mem_address[31:0] <= (dbus_sel) ? dmem_address : imem_address;
    mem_writedata <= dmem_writedata;
    mem_byteen <= dmem_byteen;
    mem_numblocks <= (dbus_sel) ? dmem_numblocks : imem_numblocks;
    mem_wren <= (dbus_sel) ? dmem_wren : 1'b0;
    mem_en <= (dbus_sel) ? dmem_en : imem_en;
    mem_ack <= (dbus_sel) ? dmem_ack : imem_ack;
    dmem_wait <= dbus_sel&mem_wait;
    imem_wait <= (~dbus_sel)&mem_wait;
    mem_dbus_sel <= dbus_sel;
  end


  /********************* Write Back Handling ***********************/

  /*******
  * Still need to handle: when wb buffer is full
  ********/

 //Select address, data, and byteenable from writeback instead of bus
 assign mem_wbsel=dmem_wbwe && ((mem_state==MEM_IDLE)||
                                (mem_state==MEM_WB)||
                                (mem_state==MEM_WBWAIT1)||
                                (mem_state==MEM_WBWAIT2));


  always @(posedge clk133)
    dmem_wback=jddr_data_wanted &&(mem_state==MEM_WB || mem_state==MEM_WBWAIT1);

  /**********************************************************/
  /********************* JDDR signals ***********************/
  /**********************************************************/

// - mem_ signals are used synchronously with jddr 
// mem_wait high starts command, then waits for mem_wait to go low
// mem_en must go low same cycle wait goes low

  // JDDR address - if store miss take mem_address, if wb take wbaddr
  always @*
    if (mem_wbsel) 
      mem_address_t<= {dmem_wbaddr[31:LOG2DCACHEWIDTHBITS-3],
        {LOG2DCACHEWIDTHBITS-3{1'b0}}};
    else
      mem_address_t<= mem_address;

  assign readwriten = !((mem_wren|mem_wbsel));
       //((mem_state==MEM_WRITE)       || (mem_state==MEM_WB)      || 
       //(mem_state==MEM_WRITEWAIT1)  || (mem_state==MEM_WBWAIT1) || 
       //(mem_state==MEM_WRITEWAIT2)  || (mem_state==MEM_WBWAIT2) ));

  //Don't stall in WB stages unless a request is waiting, otherwise a newly
  //arrived request will interpret the wait signal as its own and think it 
  //finished
  always @*
    mem_wait=
      (mem_state==MEM_WB || mem_state==MEM_WBWAIT1 || mem_state==MEM_WBWAIT2) ? 
          mem_en : ((mem_state!=MEM_IDLE) && (mem_state!=MEM_DONE));

  assign mem_wordsel=mem_address[LOG2DRAMWIDTHBITS-3-1:2]; //[3:2]

  always @*
  begin
    case(mem_wordsel)
      2'b00: fillbuf_byteen<={12'b0,mem_byteen};
      2'b01: fillbuf_byteen<={8'b0,mem_byteen,4'b0};
      2'b10: fillbuf_byteen<={4'b0,mem_byteen,8'b0};
      default: fillbuf_byteen<={mem_byteen,12'b0};
    endcase
  end

  always@(posedge clk133)
    jddr_byteena=(mem_wbsel|~mem_wren) ? {DRAMWIDTHBITS/8{1'b1}}:fillbuf_byteen;

  // JDDR write data - if store miss take mem_writedata, if wb take wbdata
  assign jddr_write_data=(~mem_wbsel) ? 
        {DRAMWIDTHBITS/32/2{mem_writedata[31:0],mem_writedata[31:0]}} :
        dmem_wbdata;


  always @*
  begin
    transrequest=0;
    case (mem_state)
      MEM_READ:       transrequest=1;
      MEM_READWAIT1:  transrequest=1;
      MEM_WRITE:      transrequest=1;
      MEM_WRITEWAIT1: transrequest=1;
      MEM_WB:         transrequest=1;
      MEM_WBWAIT1:    transrequest=1;
      default: transrequest=0;
    endcase
  end

  // Issues all read requests during READ stage, then waits to receive all
  // requests in READWAIT1.  Analgously for writes and writeback writes.
  always @(posedge clk133)
  begin
    if (!tm4_resetn133)
      mem_state<=MEM_IDLE;
    else
    case (mem_state)
      MEM_IDLE: 
        mem_state <= (~local_init_done & ~local_ready) ? MEM_IDLE :
                    (dmem_wbwe) ? MEM_WB :
                      (mem_en) ? 
                          (mem_wren) ? MEM_WRITE : MEM_READ : MEM_IDLE;
      MEM_READ: 
        mem_state <= (mem_address_doneissue) ? MEM_READWAIT1 : MEM_READ;
      MEM_READWAIT1: 
        mem_state <= (mem_address_donerecv) ? MEM_READWAIT2 : MEM_READWAIT1;
      MEM_READWAIT2: 
        mem_state <=  MEM_DONE;
      MEM_WRITE: 
        mem_state <= (mem_address_doneissue) ? MEM_WRITEWAIT1 : MEM_WRITE;
      MEM_WRITEWAIT1: 
        mem_state <= (mem_address_donerecv) ? MEM_WRITEWAIT2 : MEM_WRITEWAIT1;
      MEM_WRITEWAIT2: 
        mem_state <=  MEM_DONE;
      MEM_WB: 
        mem_state <= (mem_address_doneissue) ? MEM_WBWAIT1 : MEM_WB;
      MEM_WBWAIT1: 
        mem_state <= (mem_address_donerecv) ? MEM_WBWAIT2 : MEM_WBWAIT1;
      MEM_WBWAIT2: 
        mem_state <= // DON'T go to DONE/IDLE or else waiting memory request
                      // will think it was satisfied
                      //Keep Writing for as long as wbbuffer wants to
                      (dmem_wbwe) ? MEM_WB :
                        //If no more writebacks, service any new requests
                        (mem_en) ? 
                          (mem_wren) ? MEM_WRITE : MEM_READ : MEM_IDLE;
      MEM_DONE: 
        mem_state <= (mem_en) ? MEM_DONE : MEM_IDLE;
      default:
        mem_state <= MEM_IDLE;
    endcase
  end

/*************************  DDR Stage **************************/

  reg  [31:0] local_address_r;
  wire [31:0] local_address;
  wire  local_read_req;
  wire  local_write_req;
  wire [1:0] local_size;
  wire  fifo_wren;
  reg  _local_read_req;
  reg  _local_write_req;
  reg  local_read_req_r;

  always @(posedge clk133)
    if (!tm4_resetn133 || !transrequest )
      mem_address_offset<= 0;
    else if (local_ready) 
      mem_address_offset<=mem_address_offset +  (DRAMWIDTHBITS/8);

  always @(posedge clk133)
    if (!tm4_resetn133 || !transrequest )
      mem_address_recvd<= {32{1'b1}};
    else if (jddr_data_available || jddr_data_wanted) 
      mem_address_recvd<=mem_address_recvd+1;

  always @(posedge clk133)
    if (!tm4_resetn133)
      mem_numblocks_r<= 0;
    else if (mem_wbsel) 
      mem_numblocks_r<=DCACHEWIDTHBITS/DRAMWIDTHBITS-1;
    else 
      mem_numblocks_r<= mem_numblocks;

  //We assume if we're asking for more than 1 block (numblocks>0) that an
  //even number of blocks are being requested (ie. cache lines are powers of 2)
  //so we can set local_size to 2 for the DDR controller
  always @(posedge clk133)
    if (!tm4_resetn133)
      mem_numblocks_not0_r<= 0;
    else if (mem_wbsel) 
      mem_numblocks_not0_r<=(DCACHEWIDTHBITS>DRAMWIDTHBITS);
    else 
      mem_numblocks_not0_r<= (|mem_numblocks);

  //If numblocks is not 0 we assume the last request would have been accepted
  //with the previous (even) memory request, so local_ready doesn't matter
  assign mem_address_doneissue= (local_ready || mem_numblocks_not0_r) &&
      (mem_state==MEM_READ || mem_state==MEM_WRITE || mem_state==MEM_WB) &&
      (mem_numblocks_r==(mem_address_offset>>(LOG2DRAMWIDTHBITS-3)));

  assign mem_address_donerecv= (mem_numblocks_r==mem_address_recvd);

  always@(posedge clk133) 
    if (!tm4_resetn133)
      local_address_r<= 0;
    else if ( !transrequest )
      local_address_r<= mem_address_t;
    else if (local_ready && (mem_state==MEM_READ || mem_state==MEM_WRITE || mem_state==MEM_WB)) 
      local_address_r<=local_address_r + (DRAMWIDTHBITS/8);

  //Compress address space to 256MB by right shifting high part of address by 1
  assign local_address={local_address_r[31:28],local_address_r[26:0]};

  assign local_size=(mem_numblocks_not0_r) ? 2 : 1;

  //We assume consecutive accesses can be coalesced and issue 1 burst transfer
  //with local_size=2, rather than 2 separate 1 size transfers.  This signal is
  //used to zero out the 2nd would-be transfer
  always@(posedge clk133) 
    if (!tm4_resetn133 || !transrequest)
      mem_coalesced <= 0;
    else if (local_ready)
      mem_coalesced <= (local_read_req || local_write_req) && (local_size==2);

  always@(posedge clk133) 
    _local_read_req=mem_en && readwriten && local_init_done;
  assign local_read_req=_local_read_req && (mem_state==MEM_READ) 
                        && !mem_coalesced;

  //If we issued a read successfully put address into fifo
  always@(posedge clk133) 
    local_read_req_r=local_read_req && local_ready;
  assign fifo_wren=(mem_coalesced) ? local_read_req_r : local_read_req && local_ready;

  always@(posedge clk133) 
    _local_write_req=((mem_en && ~readwriten) || dmem_wbwe) && 
      local_init_done;
  assign local_write_req= _local_write_req && (mem_state==MEM_WRITE || mem_state==MEM_WB) && !mem_coalesced;

  altmemddr altmemddr
  (
    .global_reset_n    (tm4_resetn),
    .local_address     (local_address>>(LOG2DRAMWIDTHBITS-3)),
    .local_be          (jddr_byteena),
    .local_size        (local_size),
    .local_init_done   (local_init_done),
    .local_rdata       (jddr_read_data),
    .local_rdata_valid (jddr_data_available),
    .local_read_req    (local_read_req),
    .local_wdata       (jddr_write_data),
    .local_wdata_req   (jddr_data_wanted),
    .local_write_req   (local_write_req),
    .local_ready       (local_ready),
    .local_refresh_ack (local_refresh_ack_from_the_altmemddr),
    .mem_addr          (mem_addr),
    .mem_ba            (mem_ba),
    .mem_cas_n         (mem_cas_n),
    .mem_cke           (mem_cke),
    .mem_clk           (mem_clk),
    .mem_clk_n         (mem_clk_n),
    .mem_cs_n          (mem_cs_n),
    .mem_dm            (mem_dm),
    .mem_dq            (mem_dq),
    .mem_dqs           (mem_dqs),
    .mem_dqsn          (mem_dqsn),
    .mem_odt           (mem_odt),
    .mem_ras_n         (mem_ras_n),
    .mem_we_n          (mem_we_n),
    //.aux_full_rate_clk (full_rate_clk),
    //.aux_half_rate_clk (half_rate_clk),
    //.oct_ctl_rs_value  (0),
    //.oct_ctl_rt_value  (0),
    .phy_clk           (clk133),
    .pll_ref_clk       (tm4_glbclk),
    .reset_phy_clk_n   (reset_phy_clk_n_from_the_altmemddr),
    //.reset_request_n   (altmemddr_s1_resetrequest_n),
    .soft_reset_n      (1'b1)
  );

  // Fill address fifo, these are the addresses we expect to receive from
  // the DRAM (in this order)
  scfifo  addrfifo (
    .clock (clk133),
    .wrreq (fifo_wren),
    .data (local_address_r),
    .rdreq (jddr_data_available),
    .q (mem_filladdr),
    .sclr (!resetn)
    // synopsys translate_off
    , .almost_full (), .usedw (),.empty (), .full () , .aclr(), .almost_empty ()
      // synopsys translate_on
      );
      defparam
        addrfifo.add_ram_output_register = "OFF",
        addrfifo.intended_device_family = "Stratix",
        addrfifo.lpm_numwords = 128,
        addrfifo.lpm_showahead = "ON",
        addrfifo.lpm_type = "scfifo",
        addrfifo.lpm_width = 32,
        addrfifo.lpm_widthu = 7, //Log2(128)
        addrfifo.overflow_checking = "ON",
        addrfifo.underflow_checking = "ON",
        addrfifo.almost_full_value = 0,
        addrfifo.use_eab = "ON";

endmodule
