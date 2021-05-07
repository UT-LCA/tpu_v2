
//Declared here but applies to whole design (thanks to `includes)!
`include "options.v"


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
    dbus_wait,

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
    S_BID ,    
    dma_dbus_address,   
    dma_dbus_readdata,  
    dma_dbus_writedata, 
    dma_dbus_byteen,
    dma_dbus_en,        
    dma_dbus_wren,      
    dma_dbus_prefetch,  
    dma_dbus_wait,      
    dma_dbus_data_valid   
    );

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

//DMA changes
output [31:0]                 dma_dbus_address;   
input  [DCACHEWIDTHBITS-1:0]    dma_dbus_readdata;  
output [DCACHEWIDTHBITS-1:0]  dma_dbus_writedata; 
output [DCACHEWIDTHBITS/8-1:0]dma_dbus_byteen;
output                        dma_dbus_en;        
output                        dma_dbus_wren;      
output                        dma_dbus_prefetch;  
input                         dma_dbus_wait;      
input                         dma_dbus_data_valid;

// PETES CHANGE for tracing
output  [ 4 : 0 ]   trc_addr;
output  [ 31 : 0 ]  trc_data;
output              trc_we;
input               trc_stall;
output              trc_pipestall;

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


`include "isa.v"
`include "visa.v"

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

      casex (ibus_readdata[31:26])
        OP_COP2:
        begin
          is_cop2=1;
          casex (ibus_readdata[5:0])
            COP2_FUNC_CFC2,
            COP2_FUNC_CTC2,
            COP2_FUNC_MTC2: is_scalar_cop2=1;
            default: is_vec_cop2=1;
          endcase
          casez ({ibus_readdata[25:22],ibus_readdata[5:0]})
            COP2_VFLD,
            COP2_VLD_B,
            COP2_VLD_H,
           // COP2_VBFADD,
            COP2_VLD_L,
            COP2_VLD_U_B,
            COP2_VLD_U_H,
            COP2_VLD_U_W,
            COP2_VLDS_B,
            COP2_VLDS_H,
            COP2_VBFSUB,
            COP2_VLDS_L,
            COP2_VLDS_U_B,
            COP2_VLDS_U_H,
            COP2_VBFMULT,
            COP2_VLDX_B,
            COP2_VLDX_H,
            COP2_VTRP,
            COP2_VLDX_L,
            COP2_VLDX_U_B,
            COP2_VLDX_U_H,
            COP2_VACT,
            COP2_VFST,
            COP2_VST_B,
            COP2_VST_H,
            COP2_VRED,
            COP2_VST_L,
            COP2_VSTS_B,
            COP2_VSTS_H,
            COP2_VPER,
            COP2_VSTS_L,
            COP2_VSTX_B,
            COP2_VSTX_H,
            COP2_VSTX_W,
            COP2_VSTX_L,
            COP2_VSTXO_B,
            COP2_VSTXO_H,
            COP2_VSTXO_W,
            COP2_VSTXO_L: is_vec_memop=1;
          endcase
        end
        OP_LB,
        OP_LBU,
        OP_LH,
        OP_LHU,
        OP_LW,
        OP_SB,
        OP_SH,
        OP_SW: is_scalar_memop=1;
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

      .ifetch_bus_en(ibus_en),
      .ifetch_bus_address(ibus_address),
      .ifetch_bus_readdata(ibus_readdata),
      .ifetch_bus_wait(ifetch_bus_wait),
      .ifetch_bus_squashn(ifetch_squashn),
      .ifetch_bus_ecause(ibus_ecause),


      .data_mem_bus_address(p_dbus_address),
      .data_mem_bus_readdata(p_dbus_readdata),
      .data_mem_bus_writedata(p_dbus_writedata),
      .data_mem_bus_byteen(p_dbus_byteen),
      .data_mem_bus_en(p_dbus_en),
      .data_mem_bus_we(p_dbus_wren),
      .data_mem_bus_wait(p_dbus_wait),
      .data_mem_bus_ecause(dbus_ecause),

      .cop2_fromcop2_wait(cop2_fromcop2_wait),
      .cop2_fromcop2_en(cop2_fromcop2_en),
      .cop2_fromcop2(cop2_fromcop2),
      .cop2_tocop2_wait(cop2_tocop2_wait),
      .cop2_tocop2_en(cop2_tocop2_en),
      .cop2_tocop2(cop2_tocop2),

      .cop0_ext_cause_in(device_ecause),
      .cop0_badvaddr_in(badvaddr),
      .cop0_badvaddr_we(badvaddr_we),

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

    vpu v(
      .clk(clk),
      .resetn(resetn),

      // Instruction interface
      .instr(ibus_readdata),
      .instr_en(instr_en), // instr is valid and available
      .instr_wait(vpu_stalled),   // if high says vpu is not ready to receive

      .has_memop(vpu_has_memop),

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
      .dbus_readdata(v_dbus_readdata),
      .dbus_writedata(v_dbus_writedata),
      .dbus_byteen(v_dbus_byteen),
      .dbus_en(v_dbus_en),
      .dbus_we(v_dbus_wren),
      .dbus_cachematch(dbus_cachematch),
      .dbus_cachemiss(dbus_cachemiss),
      .dbus_prefetch(v_dbus_prefetch),
      .dbus_wait(v_dbus_wait),
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
      .dma_dbus_address	(dma_dbus_address), 
      .dma_dbus_readdata	(dma_dbus_readdata), 
      .dma_dbus_writedata	(dma_dbus_writedata),
      .dma_dbus_byteen	(dma_dbus_byteen),
      .dma_dbus_en	(dma_dbus_en),       
      .dma_dbus_wren	(dma_dbus_wren),     
      .dma_dbus_prefetch	(dma_dbus_prefetch), 
      .dma_dbus_wait	(dma_dbus_wait),     
      .dma_dbus_data_valid(dma_dbus_data_valid)
    );
    defparam v.LOG2DMEM_WRITEWIDTH=LOG2DCACHEWIDTHBITS,
             v.LOG2DMEM_READWIDTH=LOG2DCACHEWIDTHBITS;


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
