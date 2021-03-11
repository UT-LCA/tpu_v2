`include "processor.v"
`include "mem_hierarchy.v"
`include "mem_port.v"
`include "trace.v"
`include "clkx2_pll.v"

module de3 ( 
  
    OSC1_50, 

    `ifdef TEST_BENCH
      tm4_devbus,
    `endif

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

    `ifndef TEST_BENCH
      ,
      JVC_CLK,
      JVC_CS,
      JVC_DATAOUT,
      JVC_DATAIN
    `endif

    );


  parameter [1:0] NULL=2'b00, IDLE=2'b01, TRANSFER=2'b10, DONE=2'b11;
  parameter COUNTER_SIZE=32; //must be >=32, we read the top 32 bits

  parameter LOG2DCACHEWIDTHBITS=`LOG2DCACHEWIDTHBITS;
  parameter DCACHEWIDTHBITS=2**LOG2DCACHEWIDTHBITS;

  parameter ICACHEWIDTHBITS=2**`LOG2ICACHEWIDTHBITS;

  parameter LOG2DRAMWIDTHBITS=`LOG2DRAMWIDTHBITS;
  parameter DRAMWIDTHBITS=2**LOG2DRAMWIDTHBITS;

  input   OSC1_50;

  `ifdef TEST_BENCH
    inout [40:0]  tm4_devbus;
  `else
    wire [40:0]  tm4_devbus;
    assign tm4_devbus={41{1'b1}};
  `endif

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

  `ifndef TEST_BENCH
    output                        JVC_CLK;
    output                        JVC_CS;
    output                        JVC_DATAOUT;
    input                         JVC_DATAIN;
  `endif

  reg [COUNTER_SIZE-1:0] count_r;
  wire [32-1:0] count;
  wire count_want;
  reg count_ready;

  wire procresetn_in;
  reg procresetn;
  reg procreset_want;
  wire procreset_ready;

  wire bootloadresetn_in;
  reg bootloadresetn;
  reg bootloadreset_want;
  wire bootloadreset_ready;
  reg bootload_en;

  wire traceactivate_in;
  reg traceactivate;
  reg traceactivate_want;
  wire traceactivate_ready;

  reg datawrite_want;
  wire datawrite_ready;

  reg [31:0] boot_daddr;
  wire [31:0] data;
  wire [31:0] boot_data;
  wire boot_dwe;
  reg instrwrite_want;
  wire instrwrite_ready;

  wire [7:0] fs_readdata;    // Bus reads for filesystem
  reg fs_readdata_want;
  wire fs_readdata_ready;

  wire [8:0] fs_writedata;    // Bus writes for filesystem
  wire fs_writedata_want;
  reg fs_writedata_ready;
  wire fs_wait;  

  wire [31:0] dbus_address;    // Generic BUS signals
  wire [31:0] dbus_readdata;
  wire [31:0] dbus_writedata;
  wire [3:0] dbus_byteen;
  wire dbus_en;
  wire dbus_wren;
  wire [31:0] dbus_prefetch;
  wire dbus_wait;

  wire ddr_en;
  wire ddr_wait;

    // Mem hierarchy
  wire [31:0]    dmem_filladdr;
  wire [DCACHEWIDTHBITS-1:0]   dmem_filldata;
  wire           dmem_fillwe;
  wire           dmem_fillrddirty;

  wire [31:0]   dmem_wbaddr;
  wire [DRAMWIDTHBITS-1:0]  dmem_wbdata;
  wire          dmem_wbwe;
  wire          dmem_wback;

  wire [31:0] proc_writedata;

  wire dead_select;
  wire count_select;
  wire fs_select;
  wire ddr_select;

  wire ibus_en;             // Instruction bus signals
  wire [31:0] ibus_address;
  wire ibus_wait;

    // Mem hierarchy
  wire [31:0]    imem_filladdr;
  wire [ICACHEWIDTHBITS-1:0]   imem_filldata;
  wire           imem_fillwe;

  wire  reg_file_we;               // Trace circuit
  wire [4:0]  reg_file_addr;
  wire [31:0] reg_file_data;
  wire  trc_waiting;
  wire [4:0] trc_addr;
  wire  trc_addr_ready;
  wire  trc_addr_want;
  wire [31:0] trc_data;
  wire  trc_data_ready;
  wire  trc_data_want;
  wire  trc_pipestall;

  wire memclk;
  wire clk;

  wire ddr_select_r;
  wire count_select_r;
  wire fs_select_r;

  reg [1:0] procreset_state,procreset_next_state;
  reg [1:0] bootloadreset_state,bootloadreset_next_state;
  reg [1:0] traceactivate_state,traceactivate_next_state;
  wire bootload_wait;

  wire resetn;

  assign resetn=(tm4_devbus[40]!=1'b0);

  /*************** Reset signal ***************/
  initial
    procresetn<=0;

  wire dum, dum2;  //Modelsim happy with each on its own, but not anded together
  assign dum=(procreset_state==TRANSFER);
  assign dum2=(procreset_ready==1'b0);

  always@(posedge clk)
    if (!resetn)
      procresetn<=0;
    else if (dum & dum2)
      procresetn<=procresetn_in;

  /*************** Reset signal for boot loader ***************/
  always@(posedge clk)
    if (!resetn)
      bootloadresetn<=0;
    else if ((bootloadreset_state==TRANSFER)&~bootloadreset_ready)
      bootloadresetn<=bootloadresetn_in;

  /*************** Trace activator ***************/
  // When trace activate is requested we buffer it and wait till reset high
  always@(posedge clk)
    if ((traceactivate_state==TRANSFER)&~traceactivate_ready)
      traceactivate<=traceactivate_in;

  /*************** Detect end of benchmark ***************/
  reg dead;
  always@(posedge clk)
    if (!procresetn)
      dead<=1'b0;
    else if (!dead)
      dead<=dead_select && (dbus_writedata==32'hdeaddead);

  /*************** Performance counter ***************/
  always@(posedge clk)
    if (!procresetn)
      count_r<=0;
    else if (count_select && dbus_wren)
    begin
      count_r[COUNTER_SIZE-1:COUNTER_SIZE-32]<=dbus_writedata;
    end
    else if (!dead && !(&count_r))
      count_r<=count_r+1;
  assign count=count_r[COUNTER_SIZE-1:COUNTER_SIZE-32];


  /***************************** State Machines  ******************************/

    reg [1:0] count_state,count_next_state;
    always@(count_state or count_want)
      case(count_state)
        NULL:
          begin
            count_ready=1'b0;
            count_next_state=IDLE;
          end
        IDLE:
          begin
            count_ready=1'b1; 
            count_next_state= (count_want) ? TRANSFER : IDLE;
          end
        TRANSFER:
          begin
            count_ready=1'b0; 
            count_next_state= (count_want) ? TRANSFER : IDLE;
          end
      endcase
    initial
        count_state=0;
    always@(posedge clk)
      if (!resetn)
        count_state=0;
      else
        count_state=count_next_state;
    
    /***** Resetn to data in circuit ******/
    always@(procreset_state or procreset_ready)
      case(procreset_state)
        NULL:
          begin
            procreset_want=1'b0;
            procreset_next_state=IDLE;
          end
        IDLE:
          begin
            procreset_want=1'b1;
            procreset_next_state= (procreset_ready) ? TRANSFER : IDLE;
          end
        TRANSFER:
          begin
            procreset_want=1'b0;
            procreset_next_state= (procreset_ready) ? TRANSFER : IDLE;
          end
      endcase
    initial
      procreset_state=0;
    always@(posedge clk)
      if (!resetn)
        procreset_state=0;
      else
        procreset_state=procreset_next_state;
    
    /***** Resetn to bootloader in circuit ******/
    always@(bootloadreset_state or bootloadreset_ready)
      case(bootloadreset_state)
        NULL:
          begin
            bootloadreset_want=1'b0;
            bootloadreset_next_state=IDLE;
          end
        IDLE:
          begin
            bootloadreset_want=1'b1;
            bootloadreset_next_state= (bootloadreset_ready) ? TRANSFER : IDLE;
          end
        TRANSFER:
          begin
            bootloadreset_want=1'b0;
            bootloadreset_next_state= (bootloadreset_ready) ? TRANSFER : IDLE;
          end
      endcase
    initial
      bootloadreset_state=0;
    always@(posedge clk)
      if (!resetn)
        bootloadreset_state=0;
      else
        bootloadreset_state=bootloadreset_next_state;
    
    /***** Activate trace command in circuit ******/
    always@(traceactivate_state or traceactivate_ready)
      case(traceactivate_state)
        NULL:
          begin
            traceactivate_want=1'b0;
            traceactivate_next_state=IDLE;
          end
        IDLE:
          begin
            traceactivate_want=1'b1;
            traceactivate_next_state= (traceactivate_ready) ? TRANSFER : IDLE;
          end
        TRANSFER:
          begin
            traceactivate_want=1'b0;
            traceactivate_next_state= (traceactivate_ready) ? TRANSFER : IDLE;
          end
      endcase
    initial
      traceactivate_state=0;
    always@(posedge clk)
      if (!resetn)
        traceactivate_state=0;
      else
        traceactivate_state=traceactivate_next_state;

  /*********** processor instantiation ************/
  processor p (
    .clk(clk),
    .mem_clk(memclk),
    .resetn(procresetn),
    .rt_dataout(proc_writedata),

    .ibus_en(ibus_en),
    .ibus_address(ibus_address),
    .ibus_wait(ibus_wait),

    .imem_filladdr(imem_filladdr),
    .imem_filldata(imem_filldata),
    .imem_fillwe(imem_fillwe),

    .dbus_address(dbus_address),
    .dbus_readdata(dbus_readdata),
    .dbus_writedata(dbus_writedata),
    .dbus_byteen(dbus_byteen),
    .dbus_en(dbus_en),
    .dbus_wren(dbus_wren),
    .dbus_prefetch(dbus_prefetch),
    .dbus_wait(dbus_wait),

    // PETES CHANGE for tracing
    .trc_addr(reg_file_addr),
    .trc_data(reg_file_data),
    .trc_we(reg_file_we),
    .trc_stall(trc_waiting),
    .trc_pipestall(trc_pipestall),

    .dmem_filladdr(dmem_filladdr),
    .dmem_filldata(dmem_filldata),
    .dmem_fillwe(dmem_fillwe),
    .dmem_fillrddirty(dmem_fillrddirty),
    .dmem_wbaddr(dmem_wbaddr),
    .dmem_wbdata(dmem_wbdata),
    .dmem_wbwe(dmem_wbwe),
    .dmem_wback(dmem_wback),

    .mem_dcache_address(0),
    .mem_dcache_data(0),
    .mem_dcache_out(),
    .mem_dcache_byteen(0),
    .mem_dcache_wren(0),
    
    .mem_icache_address(0),
    .mem_icache_data(0),
    .mem_icache_out(),
    .mem_icache_byteen(0),
    .mem_icache_wren(0)
    );

  /*********** Trace circuit ************
  * This circuit buffers all writes to the scalar register file and 
  * dumps them to to the ports trc_addr and trc_data
  ****************************************/
  trace trace(
    .clk(clk),
    .resetn(procresetn),
    .activated(traceactivate),

    .reg_file_we(reg_file_we), // stop recording if dead
    .reg_file_addr(reg_file_addr),
    .reg_file_data(reg_file_data),

    .pipestalled(trc_pipestall),  // tells us if trace pipe stage is stalled
    .stallpipe(trc_waiting),      // trace issues a stall to the pipe

    .addr(trc_addr),
    .addr_ready(trc_addr_ready),
    .addr_want(trc_addr_want),

    .data(trc_data),
    .data_ready(trc_data_ready),
    .data_want(trc_data_want)
    );
    /**/

  /***************************** Port MUX  *************************
  * The port mux from the TM4 ports package allows a host computer
  * to access signals in your design.  There are three versions:
  *   1) The TM4 version (over PCI - tm4_portmux)
  *   2) The DE2/DE3 version (over JTAG over USB - tmj_portmux)
  *   3) There's also a new DE3 version (over USB 2.0 - tmu_portmux)
  ****************************************/

  `ifdef TEST_BENCH
  tm4_portmux pm(
      .tm4_glbclk0(clk),
      .tm4_devbus(tm4_devbus),
  `else
  tmj_portmux pm(
      .clk(clk),
  `endif

      .count_ready(count_ready),
      .count_want(count_want),
      .count(count),

      .procresetn_in(procresetn_in),
      .procreset_want(procreset_want),
      .procreset_ready(procreset_ready),

      .bootloadresetn_in(bootloadresetn_in),
      .bootloadreset_want(bootloadreset_want),
      .bootloadreset_ready(bootloadreset_ready),

      .traceactivate_in(traceactivate_in),
      .traceactivate_want(traceactivate_want),
      .traceactivate_ready(traceactivate_ready),

      .data(boot_data),
      .datawrite_want(datawrite_want),
      .datawrite_ready(datawrite_ready),

      .trc_addr(trc_addr),
      .trc_addr_want(trc_addr_want),
      .trc_addr_ready(trc_addr_ready),

      .trc_data(trc_data),
      .trc_data_want(trc_data_want),
      .trc_data_ready(trc_data_ready),

      .fs_writedata(fs_writedata),   // host software must read this
      .fs_writedata_want(fs_writedata_want),
      .fs_writedata_ready(fs_writedata_ready),

      .fs_readdata(fs_readdata),   // host software must write here
      .fs_readdata_want(fs_readdata_want),
      .fs_readdata_ready(fs_readdata_ready)
      );

  /*************************************************************
   * Bootloading Protocol:
   *************************************************************
   *  0. Put processor in reset
   *  1. Lower bootloadresetn 
   *  2. Raise bootloadresetn 
   *  3. Send starting address to write to over "data" input
   *  4. Send all data successively on "data"
   *  6. Go to 1. for any other data segments
   *  7. Bring proc out of reset
   ************************************************************/

  /***** Write to data in circuit - Bootloader communication FSM ******/
  reg [2:0] datawrite_state;
  parameter WAITCYCLE=3'b100;
  initial
    datawrite_state=0;
  always@(posedge clk)
    if (!resetn)
      datawrite_state<=0;
    else
      case(datawrite_state)
        NULL:
          begin
            datawrite_want<=1'b0;
            datawrite_state<=IDLE;
          end
        IDLE:
          begin
            datawrite_want<=1'b1;
            datawrite_state<= (datawrite_ready) ? WAITCYCLE : IDLE;
          end
        WAITCYCLE:  //WAIT 1 cycle since bootload_wait goes high 1 cycle late
          begin
            datawrite_want<=1'b1;
            datawrite_state<= TRANSFER;
          end
        TRANSFER:
          begin
            datawrite_want<=bootload_wait;
            datawrite_state<= (bootload_wait) ? TRANSFER : DONE;
          end
        DONE:
          begin
            datawrite_want<=1'b0;
            datawrite_state<= (datawrite_ready) ? DONE : IDLE;
          end
      endcase

  /*************** Data Boot loader address counter & control ***************/
  always@(posedge clk)
    if ((!bootloadresetn)||procresetn)
    begin
      boot_daddr<=32'hffffffff;
      bootload_en<=0;
    end
    else if (boot_dwe)
    begin
      if (boot_daddr==32'hffffffff)
      begin
        boot_daddr<=boot_data;
        bootload_en<=1;
      end
      else 
        boot_daddr<=boot_daddr+4;
    end
  assign boot_dwe=((datawrite_state==TRANSFER)&~bootload_wait);


  /************************************************************************/
  /***************************** File System ******************************/
  /************************************************************************/

  /***** SP Read from FS through TM4 ******/
  reg [1:0] fs_readdata_state;
  initial
    fs_readdata_state=0;
  always@(posedge clk)
    if (!resetn)
      fs_readdata_state<=0;
    else
      case(fs_readdata_state)
        NULL:
          begin
            fs_readdata_want<=1'b0;
            fs_readdata_state<=IDLE;
          end
        IDLE:
          begin
            fs_readdata_want<=(fs_select)&&(~dbus_wren);
            fs_readdata_state<= (fs_readdata_ready) ? TRANSFER : IDLE;
          end
        TRANSFER:
          begin
            fs_readdata_want<=1'b0;
            fs_readdata_state<= (fs_readdata_ready) ? TRANSFER : IDLE;
          end
      endcase

  /***** Write to FS ******/
  reg [1:0] fs_writedata_state;
  initial
    fs_writedata_state=0;
  always@(posedge clk)
    if (!resetn)
      fs_writedata_state<=0;
    else
      case(fs_writedata_state)
        NULL:
          begin
            fs_writedata_ready<=1'b0;
            fs_writedata_state<=IDLE;
          end
        IDLE:
          begin
            fs_writedata_ready<=1'b1;
            //fs_writedata_ready<=(fs_select)&&(dbus_wren);
            fs_writedata_state<= (fs_writedata_want) ? TRANSFER : IDLE;
          end
        TRANSFER:
          begin
            fs_writedata_ready<=1'b0; 
            fs_writedata_state<= (fs_writedata_want) ? TRANSFER : IDLE;
          end
      endcase

    // 9th bit indicates if data is valid or not
  assign fs_writedata=(fs_select&&dbus_wren) ?  
                        {(fs_select&dbus_wren),dbus_writedata[31:24]} : 0;
  assign fs_wait = (fs_select_r)&&
                    ( (!((fs_readdata_state==TRANSFER)&~fs_readdata_ready)) &&
                      !((fs_writedata_state==TRANSFER)&~fs_writedata_want));


    // Dead mapped to:               0x80000104-0x80000107
    // Counter mapped to             0x80000100-0x80000103
    // File system mapped to address 0x80000000-0x8000000f
    // DDR mapped to address         0x00000000-0x7fffffff

  assign dead_select =  dbus_en&({dbus_address[31:2],2'b0}==32'h80000104);
  assign count_select = dbus_en&({dbus_address[31:2],2'b0}==32'h80000100);
  assign fs_select =    dbus_en&({dbus_address[31:4],4'b0}==32'h80000000);
  assign ddr_select =   ({dbus_address[31:31],31'b0}==32'h00000000);

  /************************************************************************/
  /************************** Memory Subsystem ****************************/
  /************************************************************************/

  assign dbus_wait = (fs_select_r&fs_wait)|(ddr_select_r&ddr_wait);

  /********* Timing Fixes **********/
  // Guarantee at least one clock delay for readdata also
  //   i) DDR - cache, ii) count - we force, iii) fs - takes long

  register ddrselreg(ddr_select,clk,procresetn,1'b1,ddr_select_r);
    defparam ddrselreg.WIDTH=1;
  register countselreg(count_select,clk,procresetn,1'b1,count_select_r);
    defparam ddrselreg.WIDTH=1;
  register fsselreg(fs_select,clk,procresetn,1'b1,fs_select_r);
    defparam ddrselreg.WIDTH=1;

  assign dbus_readdata = (count_select_r) ? count :
                          {fs_readdata[7:0],24'b0};

  /********* /Timing Fixes **********/

  assign ddr_en = dbus_en & ddr_select;

  /*****
   * Expected CPU bus protocol:
   *  1. dbus_en goes high requesting a bus transaction 
   *  2. dbus_wait goes high in same cycle.
   *  3. addr, data, byteen, wren are then registered but should be kept stable
   *  4. When dbus_wait=0, CPU is assumed to have moved on in the next cycle
   *      ie. if dbus_en still on it is interpreted as a new request
   *****/

  mem_hierarchy mem_hierarchy (

    .procresetn(procresetn),

    .bus_clk(clk),
    .resetn(procresetn|bootloadresetn),

    // From CPU data bus
    .dbus_address(dbus_address),
    .dbus_en(ddr_en), 
    .dbus_wait(ddr_wait), 
    .dbus_prefetch(dbus_prefetch),

    .clk133(memclk),
    .dmem_filladdr(dmem_filladdr),
    .dmem_filldata(dmem_filldata),
    .dmem_fillwe(dmem_fillwe),
    .dmem_fillrddirty(dmem_fillrddirty),
    .dmem_wbaddr(dmem_wbaddr),
    .dmem_wbdata(dmem_wbdata),
    .dmem_wbwe(dmem_wbwe),
    .dmem_wback(dmem_wback),

    .ibus_en(ibus_en),
    .ibus_address(ibus_address),
    .ibus_wait(ibus_wait),

    .imem_filladdr(imem_filladdr),
    .imem_filldata(imem_filldata),
    .imem_fillwe(imem_fillwe),

    // From bootloader
    .bootload_address(boot_daddr),
    .bootload_writedata(boot_data),
    .bootload_byteen(4'b1111),
    .bootload_en(((datawrite_state==WAITCYCLE)||(datawrite_state==TRANSFER))&&bootload_en), 
    .bootload_wren(1'b1), 
    .bootload_wait(bootload_wait), 

    // TM4 signals needed by jddr interface
    .tm4_glbclk(clk),
    .tm4_resetn(resetn),

    // DDR Interface Signals for DDR slot A
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
    .mem_we_n          (mem_we_n)
    );
    defparam mem_hierarchy.LOG2DCACHEWIDTHBITS=LOG2DCACHEWIDTHBITS;



  //=======================================================
  //  REG/WIRE declarations
  //=======================================================

  //=======================================================
  //  IO Group Voltage Configuration (Do not modify it)
  //=======================================================
`ifndef TEST_BENCH
  IOV_A3V3_B1V8_C3V3_D3V3 IOV_Instance(
    .iCLK(OSC1_50),
    //.iRST_n(system_reset_n),
    .iRST_n(1'b1),
    .iENABLE(1'b0),
    .oREADY(),
    .oERR(),
    .oERRCODE(),
    .oJVC_CLK(JVC_CLK),
    .oJVC_CS(JVC_CS),
    .oJVC_DATAOUT(JVC_DATAOUT),
    .iJVC_DATAIN(JVC_DATAIN)
    );
`endif 


    clkx2_pll  clkx2_pll (
      .inclk0(OSC1_50),   //50 MHz Clock
      .areset (1'b0),
      .c0 (clk));         //100 MHz Clock



endmodule
