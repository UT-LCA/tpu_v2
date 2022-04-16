
//Declared here but applies to whole design (thanks to `includes)!
//`include "options.v"


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

  // AXI interface
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
  
  // Databus interface
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

    scalar_dbus_address,
    scalar_dbus_readdata,
    scalar_dbus_writedata,
    scalar_dbus_byteen,
    scalar_dbus_en,
    scalar_dbus_wren,

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

output [31:0] scalar_dbus_address;    // Data bus signals
input  [31:0] scalar_dbus_readdata;
output [31:0] scalar_dbus_writedata;
output [3:0]  scalar_dbus_byteen;
output        scalar_dbus_en;
output        scalar_dbus_wren;

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

//`include "isa.v"
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: isa.v
//////////////////////////////////////////////////////////////////////
parameter     OP_SPECIAL      = 6'b000000;
parameter     OP_REGIMM       = 6'b000001;
parameter     OP_J            = 6'b000010;
parameter     OP_JAL          = 6'b000011;
parameter     OP_BEQ          = 6'b000100;
parameter     OP_BNE          = 6'b000101;
parameter     OP_BLEZ         = 6'b000110;
parameter     OP_BGTZ         = 6'b000111;

parameter     OP_ADDI         = 6'b001000;
parameter     OP_ADDIU        = 6'b001001;
parameter     OP_SLTI         = 6'b001010;
parameter     OP_SLTIU        = 6'b001011;
parameter     OP_ANDI         = 6'b001100;
parameter     OP_ORI          = 6'b001101;
parameter     OP_XORI         = 6'b001110;
parameter     OP_LUI          = 6'b001111;

parameter     OP_LB           = 6'b100000;
parameter     OP_LH           = 6'b100001;
parameter     OP_LWL          = 6'b100010;
parameter     OP_LW           = 6'b100011;
parameter     OP_LBU          = 6'b100100;
parameter     OP_LHU          = 6'b100101;
parameter     OP_LWR          = 6'b100110;

parameter     OP_SB           = 6'b101x00;
parameter     OP_SH           = 6'b101x01;
parameter     OP_SWL          = 6'b101010;
parameter     OP_SW           = 6'b101x11;
parameter     OP_SWR          = 6'b101110;

/****** FUNCTION CLASS - bits 5...0 *******/
parameter     FUNC_SLL        = 6'b000000;
parameter     FUNC_SRL        = 6'b000010;
parameter     FUNC_SRA        = 6'b000011;
parameter     FUNC_SLLV       = 6'b000100;
parameter     FUNC_SRLV       = 6'b000110;
parameter     FUNC_SRAV       = 6'b000111;

parameter     FUNC_JR         = 6'b001xx0;
parameter     FUNC_JALR       = 6'b001xx1;

parameter     FUNC_MFHI       = 6'bx10x00;
parameter     FUNC_MTHI       = 6'bx10x01;
parameter     FUNC_MFLO       = 6'bx10x10;
parameter     FUNC_MTLO       = 6'bx10x11;

parameter     FUNC_MULT       = 6'bx11x00;
parameter     FUNC_MULTU      = 6'bx11x01;
parameter     FUNC_DIV        = 6'bx11x10;
parameter     FUNC_DIVU       = 6'bx11x11;

parameter     FUNC_ADD        = 6'b100000;
parameter     FUNC_ADDU       = 6'b100001;
parameter     FUNC_SUB        = 6'b100010;
parameter     FUNC_SUBU       = 6'b100011;
parameter     FUNC_AND        = 6'b100100;
parameter     FUNC_OR         = 6'b100101;
parameter     FUNC_XOR        = 6'b100110;
parameter     FUNC_NOR        = 6'b100111;

parameter     FUNC_SLT        = 6'b101010;
parameter     FUNC_SLTU       = 6'b101011;

/****** REGIMM Class - bits 20...16 *******/
parameter     FUNC_BLTZ       = 1'b0;
parameter     FUNC_BGEZ       = 1'b1;

parameter     OP_COP2       = 6'b010010;
parameter     COP2_FUNC_CFC2     = 6'b111000;
parameter     COP2_FUNC_CTC2     = 6'b111010;
parameter     COP2_FUNC_MTC2     = 6'b111011;

parameter     OP_COP0       = 6'b010000;
parameter     COP0_MFC0     = 5'b00000;
parameter     COP0_MTC0     = 5'b00100;

//parameter     FUNC_BLTZAL     = 5'b10000;
//parameter     FUNC_BGEZAL     = 5'b10001;

/****** 
 * Original REGIMM class, compressed above to save decode logic
parameter     FUNC_BLTZ       = 5'b00000;
parameter     FUNC_BGEZ       = 5'b00001;
parameter     FUNC_BLTZAL     = 5'b10000;
parameter     FUNC_BGEZAL     = 5'b10001;
*/
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: isa.v
//////////////////////////////////////////////////////////////////////

//`include "visa.v"
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: visa.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/visa.v
//////////////////////////////////////////////////////////////////////
parameter COP2_VADD           = 'b1000000000;
parameter COP2_VADD_U         = 'b1000000001;
parameter COP2_VSUB           = 'b1000000010;
parameter COP2_VSUB_U         = 'b1000000011;
parameter COP2_VMULHI         = 'b1000000100;
parameter COP2_VMULHI_U       = 'b1000000101;
parameter COP2_VDIV           = 'b1000000110; //Using as matmul
//parameter COP2_VBFADD         = 'b0100000001; //Using BF16 add
//parameter COP2_VBFMULT        = 'b0100000010; //Using BF16 MULT
//parameter COP2_VACT           = 'b0100000011; //Using ACT
//parameter COP2_VTRP           = 'b0100000100; //Using ACT
parameter COP2_VDIV_U         = 'b1000000111;

//parameter COP2_VMOD           = 'b1000001000;
parameter COP2_VBFADD           = 'b1000001000;  // USING bfloat add Instr: vmod.vv vrdest, vrsrc1,vrsrc2
parameter COP2_VBFMULT         = 'b1000001001;  // Using bfloat mult Instr: vmod.u.vv vrdest, vrsrc1,vrsrc2
parameter COP2_VMOD_U         = 'b1000001001;
parameter COP2_VCMP_EQ        = 'b1000001010;
parameter COP2_VCMP_NE        = 'b1000001100;
parameter COP2_VCMP_LT        = 'b1000001110;
parameter COP2_VCMP_U_LT      = 'b1000001111;
parameter COP2_VCMP_LE        = 'b1000010000;
parameter COP2_VCMP_U_LE      = 'b1000010001;
parameter COP2_VMIN           = 'b1000010010;
parameter COP2_VMIN_U         = 'b1000010011;
parameter COP2_VMAX           = 'b1000010100;
parameter COP2_VMAX_U         = 'b1000010101;
parameter COP2_VMULLO         = 'b1000010110;
parameter COP2_VABS           = 'b1000010111;
parameter COP2_VAND           = 'b1000011000;
parameter COP2_VOR            = 'b1000011001;
parameter COP2_VXOR           = 'b1000011010;
parameter COP2_VNOR           = 'b1000011011;
parameter COP2_VSLL           = 'b1000011100;
parameter COP2_VSRL           = 'b1000011101;
parameter COP2_VSRA           = 'b1000011110;
parameter COP2_VSAT_B         = 'b1000011111;
parameter COP2_VSAT_H         = 'b1001011111;
//parameter COP2_VSAT_W         = 'b1010011111;
parameter COP2_VACT         = 'b1010011111;  // Using activation instruction: vsat.w vrdest,vrsrc
parameter COP2_VSAT_SU_B      = 'b1000100000;
parameter COP2_VSAT_SU_H      = 'b1001100000;
//parameter COP2_VSAT_SU_W      = 'b1010100000;
parameter COP2_VRED     = 'b1010100000;   // Using reduction instruction: vsat.su.w vrdest,vrsrc
parameter COP2_VSAT_SU_L      = 'b1011100000;
parameter COP2_VSAT_U_B       = 'b1000100001;
parameter COP2_VSAT_U_H       = 'b1001100001;
//parameter COP2_VSAT_U_W       = 'b1010100001;
parameter COP2_VTRP       = 'b1010100001;   // Using transpose instruction: vsat.u.w vrdest, vrsrc 
parameter COP2_VSADD          = 'b1000100010;
parameter COP2_VSADD_U        = 'b1000100011;
parameter COP2_VSSUB          = 'b1000100100;
parameter COP2_VSSUB_U        = 'b1000100101;
parameter COP2_VSRR           = 'b1000100110;
parameter COP2_VSRR_U         = 'b1000100111;
parameter COP2_VSLS           = 'b1000101000;
parameter COP2_VSLS_U         = 'b1000101001;
parameter COP2_VXUMUL         = 'b1000101010;
parameter COP2_VXUMUL_U       = 'b1000101011;
parameter COP2_VXLMUL         = 'b1000101100;
parameter COP2_VXLMUL_U       = 'b1000101101;
parameter COP2_VXUMADD        = 'b1000101110;
parameter COP2_VXUMADD_U      = 'b1000101111;
parameter COP2_VXUMSUB        = 'b1000110000;
parameter COP2_VXUMSUB_U      = 'b1000110001;
parameter COP2_VXLMADD        = 'b1000110010;
parameter COP2_VXLMADD_U      = 'b1000110011;
parameter COP2_VXLMSUB        = 'b1000110100;
parameter COP2_VXLMSUB_U      = 'b1000110101;
parameter COP2_VINS_VV        = 'b1100000000;
parameter COP2_VINS_SV        = 'b1110000001;
parameter COP2_VEXT_VV        = 'b1100000010;
parameter COP2_VEXT_SV        = 'b1100000011;
parameter COP2_VEXT_U_SV      = 'b1100000100;
parameter COP2_VCOMPRESS      = 'b1100000101;
parameter COP2_VEXPAND        = 'b1100000110;
parameter COP2_VMERGE         = 'b1100000111;
parameter COP2_VFINS          = 'b1110001000;
parameter COP2_VEXTHALF       = 'b1100001001;
parameter COP2_VHALF          = 'b1100001010;
parameter COP2_VHALFUP        = 'b1100001011;
parameter COP2_VHALFDN        = 'b1100001100;
parameter COP2_VSATVL         = 'b1100001101;
parameter COP2_VFAND          = 'b1100001110;
parameter COP2_VFOR           = 'b1100001111;
parameter COP2_VFXOR          = 'b1100010000;
parameter COP2_VFNOR          = 'b1100010001;
parameter COP2_VFCLR          = 'b1100010010;
parameter COP2_VFSET          = 'b1100010011;
parameter COP2_VIOTA          = 'b1100010100;
parameter COP2_VCIOTA         = 'b1100010101;
parameter COP2_VFPOP          = 'b1100010110;
parameter COP2_VFFF1          = 'b1100010111;
parameter COP2_VFFL1          = 'b1100011000;
parameter COP2_VFSETBF        = 'b1100011001;
parameter COP2_VFSETIF        = 'b1100011010;
parameter COP2_VFSETOF        = 'b1100011011;
parameter COP2_VFMT8          = 'b1100011100;
parameter COP2_VFMF8          = 'b1100011101;
parameter COP2_VFCLR8         = 'b1100011110;
parameter COP2_VFOR8          = 'b1100011111;
parameter COP2_VFLD           = 'b1100100000;
parameter COP2_VLD_B          = 'b1100100001;
parameter COP2_VLD_H          = 'b1101100001;

//parameter COP2_VLD_W          = 'b1110100001;
//parameter COP2_VBFADD         = 'b1110100001;  // adding bfadder Instr: vld.u.w

parameter COP2_VLD_L          = 'b1111100001;
parameter COP2_VLD_U_B        = 'b1100100010;
parameter COP2_VLD_U_H        = 'b1101100010;

//parameter COP2_VLD_U_W        = 'b1110100010;
parameter COP2_VAXIRD	        = 'b1110100010;  // adding an instruction for AXI Load;

parameter COP2_VLDS_B         = 'b1100100011;
parameter COP2_VLDS_H         = 'b1101100011;

//parameter COP2_VLDS_W         = 'b1110100011;
parameter COP2_VBFSUB         = 'b1110100011;   // adding bfsub Instr: vlds.w

parameter COP2_VLDS_L         = 'b1111100011;
parameter COP2_VLDS_U_B       = 'b1100100100;
parameter COP2_VLDS_U_H       = 'b1101100100;

//parameter COP2_VLDS_U_W       = 'b1110100100;
//parameter COP2_VBFMULT         = 'b1110100100;   // adding bfmult Instr: vlds.u.w

parameter COP2_VLDX_B         = 'b1100100101;
parameter COP2_VLDX_H         = 'b1101100101;

//parameter COP2_VLDX_W         = 'b1110100101;
parameter COP2_VPER_STR         = 'b1110100101;     // adding transpose instruction: vldx.w

parameter COP2_VLDX_L         = 'b1111100101;
parameter COP2_VLDX_U_B       = 'b1100100110;
parameter COP2_VLDX_U_H       = 'b1101100110;

//parameter COP2_VLDX_U_W       = 'b1110100110;
//parameter COP2_VPER     = 'b1110100110;        //adding activation Instr: vldx.u.w 

parameter COP2_VFST           = 'b1100101000;
parameter COP2_VST_B          = 'b1100101001;
parameter COP2_VST_H          = 'b1101101001;

//parameter COP2_VST_W        = 'b1110101001;  // adding reduction Instr: vst.w
//parameter COP2_VRED           = 'b1110101001;  // adding reduction Instr: vst.w

parameter COP2_VST_L          = 'b1111101001;
parameter COP2_VSTS_B         = 'b1100101010;
parameter COP2_VSTS_H         = 'b1101101010;

//parameter COP2_VSTS_W         = 'b1110101010;
parameter COP2_VPER_LD           = 'b1110101010;  // adding permute Instr: vsts.w

parameter COP2_VSTS_L         = 'b1111101010;
parameter COP2_VSTX_B         = 'b1100101011;
parameter COP2_VSTX_H         = 'b1101101011;
//parameter COP2_VSTX_W         = 'b1110101011;
parameter COP2_VPER         = 'b1110101011;  // adding permute operation: vstx.w Vsrc,Vbase, 
parameter COP2_VSTX_L         = 'b1111101011;
parameter COP2_VSTXO_B        = 'b1100101100;
parameter COP2_VSTXO_H        = 'b1101101100;
//parameter COP2_VSTXO_W        = 'b1110101100;  
parameter COP2_VAXIWR        = 'b1110101100;  // adding an instruction for axi write
parameter COP2_VSTXO_L        = 'b1111101100;
parameter COP2_VMCTS          = 'b1101110000;
parameter COP2_VMSTC          = 'b1101110001;
parameter COP2_CFC2           = 'b0000111000;
parameter COP2_CTC2           = 'b0000111010;
parameter COP2_MTC2           = 'b0000111011;
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: visa.v
//////////////////////////////////////////////////////////////////////

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
            //COP2_VLD_U_W,
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
            //COP2_VSTX_W,
            COP2_VSTX_L,
            COP2_VSTXO_B,
            COP2_VSTXO_H,
            //COP2_VSTXO_W,
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
   reg scalar_dbus_wait;

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


      .data_mem_bus_address(scalar_dbus_address),
      .data_mem_bus_readdata(scalar_dbus_readdata),
      .data_mem_bus_writedata(scalar_dbus_writedata),
      .data_mem_bus_byteen(scalar_dbus_byteen),
      .data_mem_bus_en(scalar_dbus_en),
      .data_mem_bus_we(scalar_dbus_wren),
      .data_mem_bus_wait(scalar_dbus_wait),
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
    reg scalar_cache_req_state;
    

   
    always@(*)begin
      case(scalar_cache_req_state)
        1'b0:begin
               scalar_dbus_wait = scalar_dbus_en;
             end
        1'b1:begin
               scalar_dbus_wait = 1'b0;
             end
      endcase
    end

    always@(posedge clk)begin
        if(!resetn)
          scalar_cache_req_state = 1'b0;
        else begin
          if(scalar_dbus_en & (scalar_cache_req_state == 1'b0))
             scalar_cache_req_state <= 1'b1;
          else
             scalar_cache_req_state <= 1'b0;
        end
    end

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

 // assign dbus_address= (v_dbus_en) ? v_dbus_address : p_dbus_address;
 // assign dbus_writedata= p_dbus_writedata;
 // assign dbus_byteen=  p_dbus_byteen;
 // assign dbus_writedata_line= (v_dbus_en) ? v_dbus_writedata : {DCACHEWIDTHBITS/32{p_dbus_writedata}};
 // assign dbus_byteen_line= (v_dbus_en) ? v_dbus_byteen : p_dbus_byteen_line;
 // assign dbus_wren= (v_dbus_en) ? v_dbus_wren : p_dbus_wren;
 // assign dbus_en=p_dbus_en || v_dbus_en;
 // assign dbus_prefetch= v_dbus_prefetch;


  assign dbus_address= (v_dbus_en) ? v_dbus_address : 0;
  assign dbus_writedata= 0;
  assign dbus_byteen=  0;
  assign dbus_writedata_line= (v_dbus_en) ? v_dbus_writedata : {DCACHEWIDTHBITS/32{32'h0}};
  assign dbus_byteen_line= (v_dbus_en) ? v_dbus_byteen : 0;
  assign dbus_wren= (v_dbus_en) ? v_dbus_wren : 0;
  assign dbus_en= v_dbus_en;
  assign dbus_prefetch= v_dbus_prefetch;

  assign p_dbus_readdata=dbus_readdata;
  assign v_dbus_readdata=dbus_readdata_line;
  //Loads/stores need to wait for vpu to finish with theirs - hence vpu_stalled
  assign p_dbus_wait=p_dbus_en&dbus_wait;
  assign v_dbus_wait=v_dbus_en&dbus_wait;

endmodule
