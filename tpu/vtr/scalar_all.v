`define USE_INHOUSE_LOGIC
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/system.v', mode 'r' at 0x7f7855a0c6f0>
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: bpred_1bittable.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/bpred_1bittable.v', mode 'r' at 0x7f7855a0c780>
//////////////////////////////////////////////////////////////////////

module branchpredict ( clk, resetn,
    predict,
    prediction,
    pc_predict,
    result_rdy,
    result,
    pc_result);
parameter PCWIDTH=32;
parameter TABLEDEPTH=4096;
parameter LOG2TABLEDEPTH=12;
parameter TABLEWIDTH=1;

input clk;
input resetn;

// Prediction Port
input predict;                  // When high tells predictor to predict in next cycle
input [PCWIDTH-1:0] pc_predict; // The PC value for which to predict 
output prediction;              // The actual prediction 1-taken, 0-nottaken

// Prediction Result Port - tells us if the prediction made at pc_result was taken
input result_rdy;               // The branch has been resolved when result_rdy goes hi
input [PCWIDTH-1:0] pc_result;  // The PC value that this result is for
input result;                   // The actual result 1-taken, 0-nottaken

wire [LOG2TABLEDEPTH-1:0] address_b;

assign address_b=pc_predict[LOG2TABLEDEPTH+2-1:2];

`ifdef USE_INHOUSE_LOGIC
    dpram pred_table(
	.clk(clk),
	.address_a(pc_result[LOG2TABLEDEPTH+2-1:2]),
	.address_b(address_b),
	.wren_a(result_rdy),
	.wren_b(0),
	.data_a(result),
	.data_b(0),
	.out_a(),
	.out_b(prediction)
    );
    defparam
        reg_file1.AWIDTH=LOG2TABLEDEPTH,
        reg_file1.NUM_WORDS=TABLEDEPTH,
        reg_file1.DWIDTH=TABLEWIDTH;

`else

	altsyncram	pred_table(
				.clock0 (clk),
				.wren_a (result_rdy),
				.address_a (pc_result[LOG2TABLEDEPTH+2-1:2]),
				.data_a (result),
				.address_b (address_b),
        .clock1 (clk),
        .clocken1 (predict),
				.q_b (prediction)
        // synopsys translate_off
        ,
        .aclr0 (1'b0),
        .aclr1 (1'b0),
        .byteena_a (1'b1),
        .byteena_b (1'b1),
        .data_b (32'b11111111),
        .wren_b (1'b0),
        .rden_b(1'b1),
        .q_a (),
        .clocken0 (1'b1),
        .addressstall_a (1'b0),
        .addressstall_b (1'b0)
        // synopsys translate_on
    );
	defparam
		pred_table.operation_mode = "DUAL_PORT",
		pred_table.width_a = TABLEWIDTH,
		pred_table.widthad_a = LOG2TABLEDEPTH,
		pred_table.numwords_a = TABLEDEPTH,
		pred_table.width_b = TABLEWIDTH,
		pred_table.widthad_b = LOG2TABLEDEPTH,
		pred_table.numwords_b = TABLEDEPTH,
		pred_table.lpm_type = "altsyncram",
		pred_table.width_byteena_a = 1,
		pred_table.outdata_reg_b = "UNREGISTERED",
		pred_table.indata_aclr_a = "NONE",
		pred_table.wrcontrol_aclr_a = "NONE",
		pred_table.address_aclr_a = "NONE",
		pred_table.rdcontrol_reg_b = "CLOCK1",
		pred_table.address_reg_b = "CLOCK1",
		pred_table.address_aclr_b = "NONE",
		pred_table.outdata_aclr_b = "NONE",
		pred_table.read_during_write_mode_mixed_ports = "OLD_DATA",
		pred_table.ram_block_type = "AUTO",
		pred_table.intended_device_family = "Stratix";

`endif

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: bpred_1bittable.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: cop0.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/cop0.v', mode 'r' at 0x7f7855a0c810>
//////////////////////////////////////////////////////////////////////

/*******
 * SPREE limitation - by not specifying stall signal name and assuming
 * "stalled" requires you to have only one opcode port which stalls
 *
 * We get around this since both INPUT&OUTPUT are in the same stage so we 
 * can use the same stall signal.
 *******/

module cop0(
    clk,
    resetn,
    stalled,

    instr,

    exception,

    read_addr,
    dest_addr,
    fromcpu,
    fromcpu_en,
    tocpu,
    tocpu_en,

    epc_in,
    ext_cause_in,
    int_cause_in_stage1,  //very weak - implement OR in SPREE instead
    int_cause_in_stage2,
    status,

    badvaddr_in,
    badvaddr_we
    );

parameter NUMSTAGESTIMES32=64;
parameter NUMSTAGES=NUMSTAGESTIMES32/32;

input clk;
input resetn;
output stalled;

input   [31:0] instr;

output exception;

input   [4:0]  read_addr;
input   [4:0]  dest_addr;
input   [31:0] fromcpu;
input          fromcpu_en;
output  [31:0] tocpu;
input          tocpu_en;

input  [31:0] epc_in;

input  [31:0] ext_cause_in;
input  [31:0] int_cause_in_stage1;
input  [31:0] int_cause_in_stage2;

output [31:0] status;
input  [31:0] badvaddr_in;
input         badvaddr_we;

wire [31:0] cause_in;

reg [31:0] epc_out;
reg [31:0] cause_out;
reg [31:0] status;
reg [31:0] badvaddr_out;

reg  [31:0] tocpu;

  assign cause_in=ext_cause_in | int_cause_in_stage1 | 
                                 int_cause_in_stage2;

  always@(posedge clk)
    if (!resetn)
      epc_out<=0;
    else if (fromcpu_en && dest_addr==14)
      epc_out<=fromcpu;
    else if (exception)
      epc_out<=epc_in;

  always@(posedge clk)
    if (!resetn)
      cause_out<=0;
    else if (fromcpu_en && dest_addr==13)
      cause_out<=fromcpu;
    else
      cause_out<=cause_in;

  always@(posedge clk)
    if (!resetn)
      status<=0;
    else if (fromcpu_en && dest_addr==12)
      status<=fromcpu;
    else if (exception)
      status[5:0]<={status[3:0],2'b0};

  always@(posedge clk)
    if (!resetn)
      badvaddr_out<=0;
    else if (fromcpu_en && dest_addr==8)
      badvaddr_out<=fromcpu;
    else if (badvaddr_we)
      badvaddr_out<=badvaddr_in;

  always@(posedge clk)
      tocpu <= (read_addr==14) ? epc_out : 
               (read_addr==13) ? cause_out : 
               (read_addr==8)  ? badvaddr_out : status;

  // 1 cycle stall
  multicyclestall mc(tocpu_en,0,clk,resetn,stalled);
  //assign stalled= 0;

  assign exception = ((|(cause_in[15:8] & status[15:8])) && status[0]);

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: cop0.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: cop2.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/cop2.v', mode 'r' at 0x7f7855a0c780>
//////////////////////////////////////////////////////////////////////

/*******
 * SPREE limitation - by not specifying stall signal name and assuming
 * "stalled" requires you to have only one opcode port which stalls
 *
 * We get around this since both INPUT&OUTPUT are in the same stage so we 
 * can use the same stall signal.
 *******/

module cop2(
    clk,
    resetn,
    stalled,

    fromcpu,
    fromcpu_en,
    tocpu,
    tocpu_en,

    //Global I/O
    tocop2,
    tocop2_en,
    tocop2_wait,
    fromcop2,
    fromcop2_en,
    fromcop2_wait
    );

input clk;
input resetn;
output stalled;

input   [31:0] fromcpu;
input          fromcpu_en;
output  [31:0] tocpu;
input          tocpu_en;

output  [31:0] tocop2;
output         tocop2_en;
input          tocop2_wait;
input   [31:0] fromcop2;
input          fromcop2_en;
output         fromcop2_wait;


  assign tocop2=fromcpu;
  assign tocop2_en=fromcpu_en;

  assign tocpu=fromcop2;
  assign fromcop2_wait=fromcop2_en&~tocpu_en;   //assign 1 if pipe is stalled 

  assign stalled= (fromcpu_en & tocop2_wait) || (tocpu_en & ~fromcop2_en);


endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: cop2.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: lo_reg.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/lo_reg.v', mode 'r' at 0x7f7855a0c810>
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Generic Register
****************************************************************************/
module lo_reg(d,clk,resetn,squashn,en,q);
parameter WIDTH=32;

input clk;
input resetn;
input squashn;
input en;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
reg [WIDTH-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1 && squashn)
		q<=d;
end

endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: lo_reg.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: hi_reg.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/hi_reg.v', mode 'r' at 0x7f7855a0c780>
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Generic Register
****************************************************************************/
module hi_reg(d,clk,resetn,squashn,en,q);
parameter WIDTH=32;

input clk;
input resetn;
input squashn;
input en;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
reg [WIDTH-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1 && squashn)
		q<=d;
end

endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: hi_reg.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: data_mem_bus_int.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/data_mem_bus_int.v', mode 'r' at 0x7f7855a0c810>
//////////////////////////////////////////////////////////////////////
/******************************************************************************
            Data memory and interface

Operation table:

  load/store sign size1 size0    |   Operation
7     0       1     1     1      |      LB
5     0       1     0     1      |      LH
0     0       X     0     0      |      LW
3     0       0     1     1      |      LBU
1     0       0     0     1      |      LHU
11    1       X     1     1      |      SB
9     1       X     0     1      |      SH
8     1       X     0     0      |      SW

******************************************************************************/

module data_mem( clk, resetn, en, stalled,
    d_writedata,
    d_address,
    op,
    d_loadresult,

    ecause,

    boot_daddr, 
    boot_ddata, 
    boot_dwe, 

    bus_address,
    bus_byteen,
    bus_we,
    bus_en,
    bus_writedata,
    bus_readdata,
    bus_wait,
    bus_ecause
                );

parameter D_ADDRESSWIDTH=32;

parameter DM_DATAWIDTH=32;
parameter DM_BYTEENAWIDTH=4;             // usually should be DM_DATAWIDTH/8
parameter DM_ADDRESSWIDTH=16;
parameter DM_SIZE=16384;

input clk;
input resetn;
input en;
output stalled;

output [31:0] ecause; 

input [31:0] boot_daddr;
input [31:0] boot_ddata;
input boot_dwe;

input [D_ADDRESSWIDTH-1:0] d_address;
input [4-1:0] op;
input [DM_DATAWIDTH-1:0] d_writedata;
output [DM_DATAWIDTH-1:0] d_loadresult;

output [32-1:0] bus_address;
output [4-1:0] bus_byteen;
output         bus_we;
output         bus_en;
output [32-1:0] bus_writedata;
input  [32-1:0] bus_readdata;
input           bus_wait;
input  [32-1:0] bus_ecause;

wire [DM_BYTEENAWIDTH-1:0] d_byteena;
wire [DM_DATAWIDTH-1:0] d_readdatain;
wire [DM_DATAWIDTH-1:0] d_writedatamem;
wire d_write;
wire [1:0] d_address_latched;

assign d_write=op[3];

assign ecause=bus_ecause;

register d_address_reg(d_address[1:0],clk,1'b1,en,d_address_latched);
    defparam d_address_reg.WIDTH=2;
                
store_data_translator sdtrans_inst(
    .write_data(d_writedata),
    .d_address(d_address[1:0]),
    .store_size(op[1:0]),
    .d_byteena(d_byteena),
    .d_writedataout(d_writedatamem));

load_data_translator ldtrans_inst(
    .d_readdatain(d_readdatain),
    .d_address(d_address_latched[1:0]),
    .load_size(op[1:0]),
    .load_sign_ext(op[2]),
    .d_loadresult(d_loadresult));
  
assign bus_address=d_address;
assign bus_byteen=d_byteena;
assign bus_we=d_write;
assign bus_en=en;
assign bus_writedata=d_writedatamem;
assign d_readdatain=bus_readdata;
assign stalled=bus_wait;

/*
altsyncram  dmem (
            .wren_a (d_write&en&(~d_address[31])),
            .clock0 (clk),
            .clocken0 (),
            .clock1 (clk),
            .clocken1 (boot_dwe),
            `ifdef TEST_BENCH
            .aclr0(~resetn), 
            `endif
            .byteena_a (d_byteena),
            .address_a (d_address[DM_ADDRESSWIDTH+2-1:2]),
            .data_a (d_writedatamem),
            .wren_b (boot_dwe), .data_b (boot_ddata), .address_b (boot_daddr), 
            // synopsys translate_off
            .rden_b (), 
            .aclr1 (), .byteena_b (),
            .addressstall_a (), .addressstall_b (), .q_b (),
            // synopsys translate_on
            .q_a (d_readdatain)
    
);  
    defparam
        dmem.intended_device_family = "Stratix",
        dmem.width_a = DM_DATAWIDTH,
        dmem.widthad_a = DM_ADDRESSWIDTH-2,
        dmem.numwords_a = DM_SIZE,
        dmem.width_byteena_a = DM_BYTEENAWIDTH,
        dmem.operation_mode = "BIDIR_DUAL_PORT",
        dmem.width_b = DM_DATAWIDTH,
        dmem.widthad_b = DM_ADDRESSWIDTH-2,
        dmem.numwords_b = DM_SIZE,
        dmem.width_byteena_b = 1,
        dmem.outdata_reg_a = "UNREGISTERED",
        dmem.address_reg_b = "CLOCK1",
        dmem.wrcontrol_wraddress_reg_b = "CLOCK1",
        dmem.wrcontrol_aclr_a = "NONE",
        dmem.address_aclr_a = "NONE",
        dmem.outdata_aclr_a = "NONE",
        dmem.byteena_aclr_a = "NONE",
        dmem.byte_size = 8,
        `ifdef TEST_BENCH
          dmem.indata_aclr_a = "CLEAR0",
          dmem.init_file = "data.rif",
        `endif
        `ifdef QUARTUS_SIM
          dmem.init_file = "data.mif",
          dmem.ram_block_type = "M4K",
        `else
          dmem.ram_block_type = "MEGARAM",
        `endif
        dmem.lpm_type = "altsyncram";
*/
  
endmodule



/****************************************************************************
          Store data translator
          - moves store data to appropriate byte/halfword 
          - interfaces with altera blockrams
****************************************************************************/
module store_data_translator(
    write_data,             // data in least significant position
    d_address,
    store_size,
    d_byteena,
    d_writedataout);        // shifted data to coincide with address
parameter WIDTH=32;

input [WIDTH-1:0] write_data;
input [1:0] d_address;
input [1:0] store_size;
output [3:0] d_byteena;
output [WIDTH-1:0] d_writedataout;

reg [3:0] d_byteena;
reg [WIDTH-1:0] d_writedataout;

always @(write_data or d_address or store_size)
begin
    case (store_size)
        2'b11:
            case(d_address[1:0])
                0: 
                begin 
                    d_byteena=4'b1000; 
                    d_writedataout={write_data[7:0],24'b0}; 
                end
                1: 
                begin 
                    d_byteena=4'b0100; 
                    d_writedataout={8'b0,write_data[7:0],16'b0}; 
                end
                2: 
                begin 
                    d_byteena=4'b0010; 
                    d_writedataout={16'b0,write_data[7:0],8'b0}; 
                end
                default: 
                begin 
                    d_byteena=4'b0001; 
                    d_writedataout={24'b0,write_data[7:0]}; 
                end
            endcase
        2'b01:
            case(d_address[1])
                0: 
                begin 
                    d_byteena=4'b1100; 
                    d_writedataout={write_data[15:0],16'b0}; 
                end
                default: 
                begin 
                    d_byteena=4'b0011; 
                    d_writedataout={16'b0,write_data[15:0]}; 
                end
            endcase
        default:
        begin
            d_byteena=4'b1111;
            d_writedataout=write_data;
        end
    endcase
end

endmodule

/****************************************************************************
          Load data translator
          - moves read data to appropriate byte/halfword and zero/sign extends
****************************************************************************/
module load_data_translator(
    d_readdatain,
    d_address,
    load_size,
    load_sign_ext,
    d_loadresult);
parameter WIDTH=32;

input [WIDTH-1:0] d_readdatain;
input [1:0] d_address;
input [1:0] load_size;
input load_sign_ext;
output [WIDTH-1:0] d_loadresult;

reg [WIDTH-1:0] d_loadresult;

always @(d_readdatain or d_address or load_size or load_sign_ext)
begin
    case (load_size)
        2'b11:
        begin
            case (d_address[1:0])
                0: d_loadresult[7:0]=d_readdatain[31:24];
                1: d_loadresult[7:0]=d_readdatain[23:16];
                2: d_loadresult[7:0]=d_readdatain[15:8];
                default: d_loadresult[7:0]=d_readdatain[7:0];
            endcase
            d_loadresult[31:8]={24{load_sign_ext&d_loadresult[7]}};
        end
        2'b01:
        begin
            case (d_address[1])
                0: d_loadresult[15:0]=d_readdatain[31:16];
                default: d_loadresult[15:0]=d_readdatain[15:0];
            endcase
            d_loadresult[31:16]={16{load_sign_ext&d_loadresult[15]}};
        end
        default:
            d_loadresult=d_readdatain;
    endcase
end

endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: data_mem_bus_int.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: divider.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/divider.v', mode 'r' at 0x7f7855a0c780>
//////////////////////////////////////////////////////////////////////


module div(en,resetn,stalled,quotient,remainder,dividend,divider,sign,clk);

   input         clk;
   input         resetn;
   input         sign;
   input         en;
   input [31:0]  dividend, divider;
   output [31:0] quotient, remainder;
   output        stalled;

   reg [31:0]    quotient, quotient_temp;
   reg [63:0]    dividend_copy, divider_copy, diff;
   reg           negative_output;
   
   wire [31:0]   remainder = (!negative_output) ? 
                             dividend_copy[31:0] : 
                             ~dividend_copy[31:0] + 1'b1;

   reg [5:0]     bits; 

   parameter IDLE=2'b00,
             DIVIDING=2'b01,
             DONE=2'b10;

   reg [1:0] state;

   always@(posedge clk)
     if (!resetn)
       state<=0;
     else
       case(state)
        IDLE: state<=(en) ? DIVIDING : IDLE;
        DIVIDING: state<=(bits==5'd1) ? DONE : DIVIDING;
        DONE: state<= IDLE;
        default: state<=0;
       endcase

   assign stalled = (state==DIVIDING) || (state==IDLE && en);
   //assign stalled = (bits==0 && en) || (|bits);

   always @( posedge clk ) 
     if (!resetn)
     begin

        bits = 0;
        quotient = 0;
        quotient_temp = 0;
        dividend_copy = 0;
        divider_copy = 0;
        negative_output =0;
        diff=0;

     end
     else if( en && state==IDLE) begin

        bits = 6'd32;
        quotient = 0;
        quotient_temp = 0;
        dividend_copy = (!sign || !dividend[31]) ? 
                        {32'd0,dividend} : 
                        {32'd0,~dividend + 1'b1};
        divider_copy = (!sign || !divider[31]) ? 
                       {1'b0,divider,31'd0} : 
                       {1'b0,~divider + 1'b1,31'd0};

        negative_output = sign &&
                          ((divider[31] && !dividend[31]) 
                        ||(!divider[31] && dividend[31]));
        
     end 
     else if ( bits > 0 ) begin

        diff = dividend_copy - divider_copy;

        quotient_temp = quotient_temp << 1;

        if( !diff[63] ) begin

           dividend_copy = diff;
           quotient_temp[0] = 1'd1;

        end

        quotient = (!negative_output) ? 
                   quotient_temp : 
                   ~quotient_temp + 1'b1;

        divider_copy = divider_copy >> 1;
        bits = bits - 1'b1;

     end
endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: divider.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: mul_shift_stall.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/mul_shift_stall.v', mode 'r' at 0x7f7855a0c810>
//////////////////////////////////////////////////////////////////////

/****************************************************************************
          MUL/DIV unit

Operation table

   op sign dir
4  1   0    x    |  MULTU
6  1   1    x    |  MULT
0  0   0    0    |  ShiftLeft
1  0   0    1    |  ShiftRightLogic
3  0   1    1    |  ShiftRightArith
****************************************************************************/
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: options.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/top/options.v', mode 'r' at 0x7f7855a0c780>
//////////////////////////////////////////////////////////////////////
`ifndef _OPTIONS_V_
`define _OPTIONS_V_ 1

`define NO_PLI 1
//`define TEST_BENCH 1
`define USE_INHOUSE_LOGIC
`define SIMULATION_MEMORY
// Replaces altera blocks with local logic files

/************************** ABBREVIEATED NAMES *****************************/
// Note: LG = Log base 2
//
// Default configuration:
//    8KB Icache (LGID=13) with 16 byte cache line size (LGIW=4)
//   32KB Dcache (LGDD=15) with 64 byte cache line size (LGDD=6)
//    Data prefetching off (DP=0, DPV=0)
//    16 Vector Lanes (LGL=4)
//    64 Maximum Vector Length (LGMVL=6)
//    32-bit (4-byte) Vector lane width (LGVPW=2)
//    32-bit Vector lane datapath width (LGLW=5)
//    16 Memory Crossbar lanes (LGM=LGL)
//    16 Multiplier lanes (LGX=LGL)
//    2 Register Banks (LGB=1)
//    Disable ALU per Bank (APB=0)


// INSTR CACHE
`define LGID 13
`define LGIW 4

// DATA CACHE
`define LGDD 15
`define LGDW 6

// DATA CACHE PREFETCHER - dcache needs to have 64 byte lines
`define LGDWB 7
`define DP 0
// VECTOR DATA CACHE PREFETCHER 0:off, 65535-N:N*veclength, N:pfch N cache lines
`define DPV 0

// VECTOR CORE
//Changing to 3. That is, we now have 8 lanes.
`define LGL 3
`define LGB 1
`define APB 0
`define LGM `LGL
`define LGX `LGL

// VECTOR ISA
`define LGMVL 6
`define LGVPW 2 //chaging the word size of vector processor to 16: support for bfloat16
`define LGLW 5

/****************************** FULL NAMES *********************************/

// INSTR CACHE
`define LOG2ICACHEDEPTHBYTES `LGID
`define LOG2ICACHEWIDTHBITS (`LGIW+3)

// DATA CACHE
`define LOG2DCACHEDEPTHBYTES `LGDD
`define LOG2DCACHEWIDTHBITS (`LGDW+3)

// DATA CACHE PREFETCHER - dcache needs to have 64 byte lines
`define LOG2DATAWBBUFFERSIZE `LGDWB
`define DEFAULTDCACHEPREFETCHES `DP
// VECTOR DATA CACHE PREFETCHER 0:off, 65535:vectorlength, N:pfch N cache lines
`define VECTORPREFETCHES `DPV

// VECTOR CORE
`define LOG2NUMLANES `LGL
`define LOG2MVL `LGMVL
`define LOG2VPW `LGVPW
`define LOG2LANEWIDTHBITS `LGLW
`define LOG2NUMMEMLANES `LGM
`define LOG2NUMMULLANES `LGX
`define LOG2NUMBANKS `LGB
`define ALUPERBANK `APB

/****************************** OTHER PARAMS *********************************/

// DRAM
`define LOG2DRAMWIDTHBITS 7

/****************** NUM PIPELINE STAGES in VECTOR PROCESSOR ***************/
//mult consumes 3 cycles
//`define MAX_PIPE_STAGES 7
//matmul consumes 29 cycles
`define MAX_PIPE_STAGES 33
`define MATMUL_STAGES 29

`endif

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: options.v
//////////////////////////////////////////////////////////////////////

module mul(clk, resetn, start, stalled, dst,
            opA, opB, sa,
            op,
            shift_result,
            hi, lo);
parameter WIDTH=32;

input clk;
input resetn;

input start;
output stalled;

input [4:0] dst;

input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
input [5-1:0] sa;
input [2:0] op;

output [WIDTH-1:0] shift_result;
output [WIDTH-1:0] hi;
output [WIDTH-1:0] lo;

/********* Control Signals *********/
wire is_signed,dir, is_mul;
assign is_mul=op[2];      // selects between opB and the computed shift amount
assign is_signed=op[1];
assign dir=op[0];         // selects between 2^sa and 2^(32-sa) for right shift

/********* Circuit Body *********/
wire dum,dum2,dum3;
wire [WIDTH:0] opB_mux_out;
wire [5-1:0] left_sa;     // Amount of left shift required for both left/right
reg [WIDTH:0] decoded_sa;

assign opB_mux_out= (is_mul) ? {is_signed&opB[WIDTH-1],opB} : decoded_sa;

`ifdef USE_INHOUSE_LOGIC
local_mult local_mult_component (
.dataa({is_signed&opA[WIDTH-1],opA}),
.datab(opB_mux_out),
.clock(clk),
.clken(1'b1),
.aclr(~resetn),
.result({dum2,dum,hi,lo})
);
defparam
 local_mult_component.LPM_WIDTHA = WIDTH + 1,
 local_mult_component.LPM_WIDTHB = WIDTH + 1,
 local_mult_component.LPM_WIDTHP = 2*WIDTH + 2,
 local_mult_component.LPM_REPRESENTATION = "SIGNED";

`else
 
lpm_mult  lpm_mult_component (
  .dataa ({is_signed&opA[WIDTH-1],opA}),
  .datab (opB_mux_out),
  .sum(),
  .clock(clk),
  .clken(),
  .aclr(~resetn),
  .result ({dum2,dum,hi,lo}));
defparam
  lpm_mult_component.lpm_widtha = WIDTH+1,
  lpm_mult_component.lpm_widthb = WIDTH+1,
  lpm_mult_component.lpm_widthp = 2*WIDTH+2,
  lpm_mult_component.lpm_widths = 1,
  lpm_mult_component.lpm_pipeline = 1,
  lpm_mult_component.lpm_type = "LPM_MULT",
  lpm_mult_component.lpm_representation = "SIGNED",
  lpm_mult_component.lpm_hint = "MAXIMIZE_SPEED=6";
`endif

assign shift_result= (dir && |sa) ? hi : lo;


assign {dum3, left_sa}= (dir) ? 32-sa : {1'b0,sa};

always@(left_sa or dir)
begin
  decoded_sa=0;
  case(left_sa)
    0: decoded_sa[0]=1;
    1: decoded_sa[1]=1;
    2: decoded_sa[2]=1;
    3: decoded_sa[3]=1;
    4: decoded_sa[4]=1;
    5: decoded_sa[5]=1;
    6: decoded_sa[6]=1;
    7: decoded_sa[7]=1;
    8: decoded_sa[8]=1;
    9: decoded_sa[9]=1;
    10: decoded_sa[10]=1;
    11: decoded_sa[11]=1;
    12: decoded_sa[12]=1;
    13: decoded_sa[13]=1;
    14: decoded_sa[14]=1;
    15: decoded_sa[15]=1;
    16: decoded_sa[16]=1;
    17: decoded_sa[17]=1;
    18: decoded_sa[18]=1;
    19: decoded_sa[19]=1;
    20: decoded_sa[20]=1;
    21: decoded_sa[21]=1;
    22: decoded_sa[22]=1;
    23: decoded_sa[23]=1;
    24: decoded_sa[24]=1;
    25: decoded_sa[25]=1;
    26: decoded_sa[26]=1;
    27: decoded_sa[27]=1;
    28: decoded_sa[28]=1;
    29: decoded_sa[29]=1;
    30: decoded_sa[30]=1;
    31: decoded_sa[31]=1;
  endcase
end

// 1 cycle stall state machine
onecyclestall staller((start&is_mul)|(start&(|dst)&~is_mul),clk,resetn,stalled);


endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: mul_shift_stall.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: logic_unit.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/logic_unit.v', mode 'r' at 0x7f7855a0c780>
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          logic unit
- note ALU must be able to increment PC for JAL type instructions

Operation Table
  op
  0     AND
  1     OR
  2     XOR
  3     NOR
****************************************************************************/
module logic_unit (
            opA, opB,
            op,
            result);
parameter WIDTH=32;


input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
input [2-1:0] op;
output [WIDTH-1:0] result;

reg [WIDTH-1:0] logic_result;

always@(opA or opB or op )
    case(op)
        2'b00:
            logic_result=opA&opB;
        2'b01:
            logic_result=opA|opB;
        2'b10:
            logic_result=opA^opB;
        2'b11:
            logic_result=~(opA|opB);
    endcase

assign result=logic_result;


endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: logic_unit.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: addersub_slt.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/addersub_slt.v', mode 'r' at 0x7f7855a0c810>
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          AddSub unit
- Should perform ADD, ADDU, SUBU, SUB, SLT, SLTU

  is_slt signext addsub
    op[2] op[1] op[0]  |  Operation
0     0     0     0         SUBU
2     0     1     0         SUB
1     0     0     1         ADDU
3     0     1     1         ADD
4     1     0     0         SLTU
6     1     1     0         SLT

****************************************************************************/
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: options.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/top/options.v', mode 'r' at 0x7f7855a0c780>
//////////////////////////////////////////////////////////////////////
`ifndef _OPTIONS_V_
`define _OPTIONS_V_ 1

`define NO_PLI 1
//`define TEST_BENCH 1
`define USE_INHOUSE_LOGIC
`define SIMULATION_MEMORY
// Replaces altera blocks with local logic files

/************************** ABBREVIEATED NAMES *****************************/
// Note: LG = Log base 2
//
// Default configuration:
//    8KB Icache (LGID=13) with 16 byte cache line size (LGIW=4)
//   32KB Dcache (LGDD=15) with 64 byte cache line size (LGDD=6)
//    Data prefetching off (DP=0, DPV=0)
//    16 Vector Lanes (LGL=4)
//    64 Maximum Vector Length (LGMVL=6)
//    32-bit (4-byte) Vector lane width (LGVPW=2)
//    32-bit Vector lane datapath width (LGLW=5)
//    16 Memory Crossbar lanes (LGM=LGL)
//    16 Multiplier lanes (LGX=LGL)
//    2 Register Banks (LGB=1)
//    Disable ALU per Bank (APB=0)


// INSTR CACHE
`define LGID 13
`define LGIW 4

// DATA CACHE
`define LGDD 15
`define LGDW 6

// DATA CACHE PREFETCHER - dcache needs to have 64 byte lines
`define LGDWB 7
`define DP 0
// VECTOR DATA CACHE PREFETCHER 0:off, 65535-N:N*veclength, N:pfch N cache lines
`define DPV 0

// VECTOR CORE
//Changing to 3. That is, we now have 8 lanes.
`define LGL 3
`define LGB 1
`define APB 0
`define LGM `LGL
`define LGX `LGL

// VECTOR ISA
`define LGMVL 6
`define LGVPW 2 //chaging the word size of vector processor to 16: support for bfloat16
`define LGLW 5

/****************************** FULL NAMES *********************************/

// INSTR CACHE
`define LOG2ICACHEDEPTHBYTES `LGID
`define LOG2ICACHEWIDTHBITS (`LGIW+3)

// DATA CACHE
`define LOG2DCACHEDEPTHBYTES `LGDD
`define LOG2DCACHEWIDTHBITS (`LGDW+3)

// DATA CACHE PREFETCHER - dcache needs to have 64 byte lines
`define LOG2DATAWBBUFFERSIZE `LGDWB
`define DEFAULTDCACHEPREFETCHES `DP
// VECTOR DATA CACHE PREFETCHER 0:off, 65535:vectorlength, N:pfch N cache lines
`define VECTORPREFETCHES `DPV

// VECTOR CORE
`define LOG2NUMLANES `LGL
`define LOG2MVL `LGMVL
`define LOG2VPW `LGVPW
`define LOG2LANEWIDTHBITS `LGLW
`define LOG2NUMMEMLANES `LGM
`define LOG2NUMMULLANES `LGX
`define LOG2NUMBANKS `LGB
`define ALUPERBANK `APB

/****************************** OTHER PARAMS *********************************/

// DRAM
`define LOG2DRAMWIDTHBITS 7

/****************** NUM PIPELINE STAGES in VECTOR PROCESSOR ***************/
//mult consumes 3 cycles
//`define MAX_PIPE_STAGES 7
//matmul consumes 29 cycles
`define MAX_PIPE_STAGES 33
`define MATMUL_STAGES 29

`endif

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: options.v
//////////////////////////////////////////////////////////////////////

module addersub (
            opA, opB,
            op, 
            result,
            result_slt );

parameter WIDTH=32;


input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
//input carry_in;
input [3-1:0] op;

output [WIDTH-1:0] result;
output result_slt;

wire carry_out;
wire [WIDTH:0] sum;

// Mux between sum, and slt
wire is_slt;
wire signext;
wire addsub;

assign is_slt=op[2];
assign signext=op[1];
assign addsub=op[0];

assign result=sum[WIDTH-1:0];
//assign result_slt[WIDTH-1:1]={31{1'b0}};
//assign result_slt[0]=sum[WIDTH];
assign result_slt=sum[WIDTH];

`ifdef USE_INHOUSE_LOGIC
  local_add_sub local_adder_inst(
      .dataa({signext&opA[WIDTH-1],opA}),
      .datab({signext&opB[WIDTH-1],opB}),
      .cin(~addsub),
      .add_sub(addsub),
      .result(sum)
  );
  defparam
      local_adder_inst.WIDTH = WIDTH+1,
      local_adder_inst.PIPELINE = 0,
      local_adder_inst.REPRESENTATION = "SIGNED";
`else
lpm_add_sub adder_inst(
    .dataa({signext&opA[WIDTH-1],opA}),
    .datab({signext&opB[WIDTH-1],opB}),
    .cin(~addsub),
    .add_sub(addsub),
    .result(sum)
        // synopsys translate_off
        ,
        .cout (),
        .clken (),
        .clock (),
        .overflow (),
        .aclr ()
        // synopsys translate_on
    );
defparam 
    adder_inst.lpm_width=WIDTH+1,
    adder_inst.lpm_representation="SIGNED";
`endif

assign carry_out=sum[WIDTH];


endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: addersub_slt.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: merge26lo.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/merge26lo.v', mode 'r' at 0x7f7855a0c780>
//////////////////////////////////////////////////////////////////////

module merge26lo(in1, in2, out);
input [31:0] in1;
input [25:0] in2;
output [31:0] out;

assign out[31:0]={in1[31:28],in2[25:0],2'b0};
endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: merge26lo.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: branchresolve.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/branchresolve.v', mode 'r' at 0x7f7855a0c810>
//////////////////////////////////////////////////////////////////////

module branchresolve ( en, rs, rt, eq, ne, ltz, lez, gtz, gez, eqz);
parameter WIDTH=32;
input en;
input [WIDTH-1:0] rs;
input [WIDTH-1:0] rt;
output eq;
output ne;
output ltz;
output lez;
output gtz;
output gez;
output eqz;

assign eq=(en)&(rs==rt);
assign ne=(en)&~eq;
assign eqz=(en)&~(|rs);
assign ltz=(en)&rs[WIDTH-1];
assign lez=(en)&rs[WIDTH-1] | eqz;
assign gtz=(en)&(~rs[WIDTH-1]) & ~eqz;
assign gez=(en)&(~rs[WIDTH-1]);

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: branchresolve.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: pcadder.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/pcadder.v', mode 'r' at 0x7f7855a0c780>
//////////////////////////////////////////////////////////////////////

module pcadder(pc, offset, result);
parameter PC_WIDTH=32;

input [PC_WIDTH-1:0] pc;
input [PC_WIDTH-1:0] offset;
output [PC_WIDTH-1:0] result;

wire dum;

assign {dum,result} = pc + {offset[PC_WIDTH-3:0],2'b0};

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: pcadder.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: signext16.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/signext16.v', mode 'r' at 0x7f7855a0c810>
//////////////////////////////////////////////////////////////////////

module signext16 ( in, out);

input [15:0] in;
output [31:0] out;

assign out={{16{in[15]}},in[15:0]};

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: signext16.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: reg_file_pipe.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/reg_file_pipe.v', mode 'r' at 0x7f7855a0c780>
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: options.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/top/options.v', mode 'r' at 0x7f7855a0c810>
//////////////////////////////////////////////////////////////////////
`ifndef _OPTIONS_V_
`define _OPTIONS_V_ 1

`define NO_PLI 1
//`define TEST_BENCH 1
`define USE_INHOUSE_LOGIC
`define SIMULATION_MEMORY
// Replaces altera blocks with local logic files

/************************** ABBREVIEATED NAMES *****************************/
// Note: LG = Log base 2
//
// Default configuration:
//    8KB Icache (LGID=13) with 16 byte cache line size (LGIW=4)
//   32KB Dcache (LGDD=15) with 64 byte cache line size (LGDD=6)
//    Data prefetching off (DP=0, DPV=0)
//    16 Vector Lanes (LGL=4)
//    64 Maximum Vector Length (LGMVL=6)
//    32-bit (4-byte) Vector lane width (LGVPW=2)
//    32-bit Vector lane datapath width (LGLW=5)
//    16 Memory Crossbar lanes (LGM=LGL)
//    16 Multiplier lanes (LGX=LGL)
//    2 Register Banks (LGB=1)
//    Disable ALU per Bank (APB=0)


// INSTR CACHE
`define LGID 13
`define LGIW 4

// DATA CACHE
`define LGDD 15
`define LGDW 6

// DATA CACHE PREFETCHER - dcache needs to have 64 byte lines
`define LGDWB 7
`define DP 0
// VECTOR DATA CACHE PREFETCHER 0:off, 65535-N:N*veclength, N:pfch N cache lines
`define DPV 0

// VECTOR CORE
//Changing to 3. That is, we now have 8 lanes.
`define LGL 3
`define LGB 1
`define APB 0
`define LGM `LGL
`define LGX `LGL

// VECTOR ISA
`define LGMVL 6
`define LGVPW 2 //chaging the word size of vector processor to 16: support for bfloat16
`define LGLW 5

/****************************** FULL NAMES *********************************/

// INSTR CACHE
`define LOG2ICACHEDEPTHBYTES `LGID
`define LOG2ICACHEWIDTHBITS (`LGIW+3)

// DATA CACHE
`define LOG2DCACHEDEPTHBYTES `LGDD
`define LOG2DCACHEWIDTHBITS (`LGDW+3)

// DATA CACHE PREFETCHER - dcache needs to have 64 byte lines
`define LOG2DATAWBBUFFERSIZE `LGDWB
`define DEFAULTDCACHEPREFETCHES `DP
// VECTOR DATA CACHE PREFETCHER 0:off, 65535:vectorlength, N:pfch N cache lines
`define VECTORPREFETCHES `DPV

// VECTOR CORE
`define LOG2NUMLANES `LGL
`define LOG2MVL `LGMVL
`define LOG2VPW `LGVPW
`define LOG2LANEWIDTHBITS `LGLW
`define LOG2NUMMEMLANES `LGM
`define LOG2NUMMULLANES `LGX
`define LOG2NUMBANKS `LGB
`define ALUPERBANK `APB

/****************************** OTHER PARAMS *********************************/

// DRAM
`define LOG2DRAMWIDTHBITS 7

/****************** NUM PIPELINE STAGES in VECTOR PROCESSOR ***************/
//mult consumes 3 cycles
//`define MAX_PIPE_STAGES 7
//matmul consumes 29 cycles
`define MAX_PIPE_STAGES 33
`define MATMUL_STAGES 29

`endif

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: options.v
//////////////////////////////////////////////////////////////////////
module reg_file(clk,resetn, c_squashn,
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_we);

parameter WIDTH=32;
parameter NUMREGS=32;
parameter LOG2NUMREGS=5;

input clk;
input resetn;

input a_en;
input b_en;

input [LOG2NUMREGS-1:0] a_reg,b_reg,c_reg;
output [WIDTH-1:0] a_readdataout, b_readdataout;
input [WIDTH-1:0] c_writedatain;
input c_we;
input c_squashn;
integer i;


`ifdef USE_INHOUSE_LOGIC1
        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(a_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we & (|c_reg) & c_squashn),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout)
        );
        defparam
            reg_file1.AWIDTH=LOG2NUMREGS,
            reg_file1.NUM_WORDS=NUMREGS,
            reg_file1.DWIDTH=WIDTH;
 
 initial begin
    for(i=0;i<NUMREGS;i=i+1)
        reg_file1.dpram1.ram[i]=0;
 end 
         
        ram_wrapper reg_file2(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(b_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(b_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we & (|c_reg)),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(b_readdataout)
        );
        defparam
            reg_file2.AWIDTH=LOG2NUMREGS,
            reg_file2.NUM_WORDS=NUMREGS,
            reg_file2.DWIDTH=WIDTH;

 initial begin
    for(i=0;i<NUMREGS;i=i+1)
        reg_file2.dpram1.ram[i]=0;
 end 
`else

	altsyncram	reg_file1(
				.wren_a (c_we & (|c_reg) & c_squashn),
				.clock0 (clk),
        .clock1 (clk),
        .clocken1 (a_en),
				.address_a (c_reg[LOG2NUMREGS-1:0]),
				.address_b (a_reg[LOG2NUMREGS-1:0]),
				.data_a (c_writedatain),
				.q_b (a_readdataout)
        // synopsys translate_off
        ,
        .aclr0 (1'b0),
        .aclr1 (1'b0),
        .byteena_a (1'b1),
        .byteena_b (1'b1),
        .data_b (32'b11111111),
        .wren_b (1'b0),
        .rden_b(1'b1),
        .q_a (),
        .clocken0 (1'b1),
        .addressstall_a (1'b0),
        .addressstall_b (1'b0)
        // synopsys translate_on
    );
	defparam
		reg_file1.operation_mode = "DUAL_PORT",
		reg_file1.width_a = WIDTH,
		reg_file1.widthad_a = LOG2NUMREGS,
		reg_file1.numwords_a = NUMREGS,
		reg_file1.width_b = WIDTH,
		reg_file1.widthad_b = LOG2NUMREGS,
		reg_file1.numwords_b = NUMREGS,
		reg_file1.lpm_type = "altsyncram",
		reg_file1.width_byteena_a = 1,
		reg_file1.outdata_reg_b = "UNREGISTERED",
		reg_file1.indata_aclr_a = "NONE",
		reg_file1.wrcontrol_aclr_a = "NONE",
		reg_file1.address_aclr_a = "NONE",
		reg_file1.rdcontrol_reg_b = "CLOCK1",
		reg_file1.address_reg_b = "CLOCK1",
		reg_file1.address_aclr_b = "NONE",
		reg_file1.outdata_aclr_b = "NONE",
		reg_file1.read_during_write_mode_mixed_ports = "OLD_DATA",
		reg_file1.ram_block_type = "AUTO",
		reg_file1.intended_device_family = "Stratix";

		//Reg file duplicated to avoid contention between 2 read
		//and 1 write
	altsyncram	reg_file2(
				.wren_a (c_we&(|c_reg)),
				.clock0 (clk),
        .clock1 (clk),
        .clocken1 (b_en),
				.address_a (c_reg[LOG2NUMREGS-1:0]),
				.address_b (b_reg[LOG2NUMREGS-1:0]),
				.data_a (c_writedatain),
				.q_b (b_readdataout)
        // synopsys translate_off
        ,
        .aclr0 (1'b0),
        .aclr1 (1'b0),
        .byteena_a (1'b1),
        .byteena_b (1'b1),
        .data_b (32'b11111111),
        .rden_b(1'b1),
        .wren_b (1'b0),
        .q_a (),
        .clocken0 (1'b1),
        .addressstall_a (1'b0),
        .addressstall_b (1'b0)
        // synopsys translate_on
    );
	defparam
		reg_file2.operation_mode = "DUAL_PORT",
		reg_file2.width_a = WIDTH,
		reg_file2.widthad_a = LOG2NUMREGS,
		reg_file2.numwords_a = NUMREGS,
		reg_file2.width_b = WIDTH,
		reg_file2.widthad_b = LOG2NUMREGS,
		reg_file2.numwords_b = NUMREGS,
		reg_file2.lpm_type = "altsyncram",
		reg_file2.width_byteena_a = 1,
		reg_file2.outdata_reg_b = "UNREGISTERED",
		reg_file2.indata_aclr_a = "NONE",
		reg_file2.wrcontrol_aclr_a = "NONE",
		reg_file2.address_aclr_a = "NONE",
		reg_file2.rdcontrol_reg_b = "CLOCK1",
		reg_file2.address_reg_b = "CLOCK1",
		reg_file2.address_aclr_b = "NONE",
		reg_file2.outdata_aclr_b = "NONE",
		reg_file2.read_during_write_mode_mixed_ports = "OLD_DATA",
		reg_file2.ram_block_type = "AUTO",
		reg_file2.intended_device_family = "Stratix";

`endif

endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: reg_file_pipe.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: ifetch_pipe_bpred_bus_int.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/ifetch_pipe_bpred_bus_int.v', mode 'r' at 0x7f7855a0c810>
//////////////////////////////////////////////////////////////////////
/****************************************************************************
            Fetch Unit with branch prediction

  IMPORTANT: tgt_pc should arrive 1 cycle after instruction to account for delay slot.
  Also, we compress the prediction PC's by one bit since delay slots prevent consecutive branches.
            
op
  0  Conditional PC write
  1  UnConditional PC write

****************************************************************************/

module ifetch(clk,resetn,
        en,         // enable increment (stage 1)
        squashn,
        we,         // enable pc update (later stage)
        op,
        load,
        load_data,

        pcwrop,     // differentiates between unconditionals: 1-unconditional
        predict_tgt_pc,
        predict_en, // enable pc update (early prediction stage)
        predict_result_rdy,
        predict_result,

        interrupt,
        epc,
        ecause,

        pc_out,
        next_pc,

  boot_iaddr, 
  boot_idata, 
  boot_iwe,

    bus_address,
    bus_en,
    bus_readdata,
    bus_wait,
    bus_squashn,
    bus_ecause,

        opcode,
        rs,
        rt,
        rd,
        sa,
        offset,
        instr_index,
        func,
        instr);

parameter EXCEPTION_ADDRESS=32'h400_0020;
parameter I_DATAWIDTH=32;
parameter I_ADDRESSWIDTH=14;
parameter I_SIZE=16384;

input [31:0] boot_iaddr;
input [31:0] boot_idata;
input boot_iwe;

output [32-1:0] bus_address;
output         bus_en;
input  [32-1:0] bus_readdata;
input           bus_wait;
output         bus_squashn;
input  [32-1:0] bus_ecause;

input clk;
input resetn;
input en;     // PC increment enable
input we;     // PC write enable
input squashn;// squash fetch
input op;     // determines if conditional or unconditional branch
input load;
input [I_DATAWIDTH-1:0] load_data;

input pcwrop;
input [I_DATAWIDTH-1:0] predict_tgt_pc;
input predict_en;
input predict_result_rdy;
output predict_result;

input  interrupt; 
output [I_DATAWIDTH-1:0] epc; 
output [31:0] ecause; 

output [I_DATAWIDTH-1:0] pc_out;   // output pc + 1 shifted left 2 bits
output [I_DATAWIDTH-1:0] next_pc;
output [31:26] opcode;
output [25:21] rs;
output [20:16] rt;
output [15:11] rd;
output [10:6] sa;
output [15:0] offset;
output [25:0] instr_index;
output [5:0] func;
output [I_DATAWIDTH-1:0] instr;


wire [I_DATAWIDTH-1:0] pc_plus_1;
reg [I_DATAWIDTH-1:0] pc;
wire ctrl_load;

// prediction stuff
wire prediction;
wire prediction_saved;
wire predict_en_saved;
wire [I_DATAWIDTH-1:0] pc_rollbacknottaken;
wire [I_DATAWIDTH-1:0] pc_rollback;

reg [I_DATAWIDTH-1:0] _next_pc;
reg pc_load_en;
reg predict_result;

// tolerating waits stuff
reg [I_DATAWIDTH-1:0] pc_save_data;
reg pc_save;
wire pc_advance;

wire is_branch;
wire is_delayslot;  //Tells us if we're fetching a delay slot

assign pc_advance=(en&~bus_wait);

assign ctrl_load=(load&~op|op);
  
assign bus_address=next_pc;
assign bus_en=(en|~squashn)&resetn;
assign bus_squashn=squashn;

/******
* WARNING: pipeline-specific - because we know the stage after fetching never
* stalls, we don't need to freeze the result.  So 'en' is a don't care here
******/
assign instr=(bus_wait) ? 'h021 : bus_readdata;

/*  DEBUG: using onchip memory and synthetic stalls
assign instr=(imem_wait) ? 'h021 : imem_readdata;

reg [5:0] count;
  always@(posedge clk)
  if (!resetn)
    count<=0;
  else
    count<=count+1;
assign imem_wait=(count[2]);
altsyncram  imem (
    .clock0 (clk),
    .clocken0 (bus_en|~resetn),
    .clock1 (clk),                              // changed
    .clocken1 (boot_iwe),                       // changed
    `ifdef TEST_BENCH
    .aclr0(~resetn), 
    `endif
    .address_a (next_pc[I_DATAWIDTH-1:2]),
    .wren_b (boot_iwe), .data_b (boot_idata), .address_b (boot_iaddr), //changed

    // synopsys translate_off
    .wren_a (), .rden_b (), .data_a (), 
    .aclr1 (), .byteena_a (), .byteena_b (),
    .addressstall_a (), .addressstall_b (), .q_b (),
    // synopsys translate_on
    
    .q_a (imem_readdata)
    );
    defparam
        imem.intended_device_family = "Stratix",
        imem.width_a = I_DATAWIDTH, 
        imem.widthad_a = I_ADDRESSWIDTH,
        imem.numwords_a = I_SIZE,
        imem.operation_mode = "BIDIR_DUAL_PORT",    // changed
        imem.width_b = I_DATAWIDTH,                 // new
        imem.widthad_b = I_ADDRESSWIDTH,            // new
        imem.numwords_b = I_SIZE,                   // new
        imem.outdata_reg_b = "UNREGISTERED",
        imem.outdata_reg_a = "UNREGISTERED",
        imem.address_reg_b = "CLOCK1",              // new
        imem.wrcontrol_wraddress_reg_b = "CLOCK1",  // new
        imem.width_byteena_a = 1,
        `ifdef TEST_BENCH
        imem.address_aclr_a = "CLEAR0",
        imem.outdata_aclr_a = "CLEAR0",
        imem.init_file = "instr.rif",
        `endif
        `ifdef QUARTUS_SIM
          imem.init_file = "instr.mif",
          imem.ram_block_type = "AUTO",
        `else
          imem.ram_block_type = "MEGARAM",
        `endif
        imem.lpm_type = "altsyncram";
*/

wire dummy;

assign {dummy,pc_plus_1} = pc + 4;
assign pc_out=pc_plus_1;

//For delay slot instruction - point to branch
assign epc=(is_delayslot) ? pc - 4 : pc;
//Insert your interrupt pending flags here 
assign ecause=bus_ecause;

assign opcode=instr[31:26];
assign rs=instr[25:21];
assign rt=instr[20:16];
assign rd=instr[15:11];
assign sa=instr[10:6];
assign offset=instr[15:0]; 
assign instr_index=instr[25:0];
assign func=instr[5:0];

//************** BRANCH PREDICTION stuff ************
// When predict_en is asserted we accept the prediction signal's value and
// adjust the PC accordingly.  In addition, we buffer everything needed to
// verify and rollback.  
// When predict_result_rdy is asserted we examine ctrl_load and compare the
// pipeline's intended change to the PC with the one we've done here.
// Note that prediction must happen on last delay slot instruction meaning that
// both the predict_en assertion and tgt_pc must come at that time


// Backup PC for both taken/not taken paths
register pcrollback( (prediction) ? pc_plus_1 : predict_tgt_pc, 
    clk, resetn,en&predict_en, pc_rollback);
  defparam pcrollback.WIDTH=I_DATAWIDTH;

register pcrollbacknt( pc, clk, resetn,en&predict_en, pc_rollbacknottaken);
  defparam pcrollbacknt.WIDTH=I_DATAWIDTH;

//register pcrollbacktk(predict_tgt_pc, clk, resetn,predict_en, 
    //pc_rollback);
  //defparam pcrollbacktk.WIDTH=I_DATAWIDTH;

register buf_predict({prediction,predict_en&(pcwrop!=1)},clk,resetn,en&predict_en, 
    {prediction_saved,predict_en_saved});
  defparam buf_predict.WIDTH=2;
  //predict_en_saved saves if it wa

/*** Saving Business
 * When a write to the PC happens deeper in the pipe while the ifetch is frozen
 * we originally stalled that branch also.  Now we save the new PC and load it
 * in once the ifetch becomes available.
 */
reg squash_save;
always@(posedge clk or negedge resetn) 
  if (!resetn)
    squash_save<=0;
  else if ( ~squashn || pc_advance)  // Capture squash requests when we're busy
    squash_save<=~squashn&~pc_advance;

always@(posedge clk or negedge resetn) 
  if (!resetn)
    pc_save<=0;
  else if ( pc_load_en || pc_advance)  // Capture we and advance to clear it
    pc_save<=pc_load_en&~pc_advance;  // zero the save if we're enabled

always@(posedge clk or negedge resetn)
  if (!resetn)
    pc_save_data<=0;
  else if (pc_load_en)  // Capture we, and advance to clear captured data
    pc_save_data<=_next_pc;


always@(posedge clk or negedge resetn)
  if (!resetn)
    pc<='h400_0000;                 // 0x400_0000/4
  else if (pc_advance)
    pc<=_next_pc;


reg [2:0] debug;

//always@(prediction_saved or predict_en_saved or prediction or en or predict_en or ctrl_load or predict_result_rdy or pc_plus_1 or load_data or we or predict_tgt_pc or pc_rollback or pc_rollbacknottaken or pc or pc_advance)
always@*
  begin
    _next_pc=pc_plus_1;
    pc_load_en=0;
    debug=0;

    if (interrupt)
    begin   // If interrupt occurs, jump to EXCEPTION_ADDRESS
      _next_pc=EXCEPTION_ADDRESS;
      pc_load_en=1;
      debug=5;
    end
    else if (ctrl_load & !predict_result_rdy)
    begin   // No prediction, do indirect jump
      _next_pc=load_data;
      pc_load_en=1;
      debug=1;
    end
    else if (predict_en & prediction)
    begin   // Make a prediction to take
      _next_pc=predict_tgt_pc;
      pc_load_en=en;
      debug=2;
    end
    else if (predict_result_rdy & (ctrl_load!=prediction_saved) & predict_en_saved)
    begin   // Mispredict - restore branch
      _next_pc=pc_rollback;
      pc_load_en=1;
      debug=3;
    end
    else if (pc_save)
    begin   // If bus is stalled and a pc load happens, save it and restore it
            // once the bus unstalls, deal with the squash + protect delay slot
      _next_pc=pc_save_data;
      pc_load_en=pc_advance;
      debug=4;
    end
end

// Separated from above always block because not dependent on pc_advance
always@*
  begin
    predict_result=1;  // Used to flush pipe
    if (ctrl_load & !predict_result_rdy)
      predict_result=~we;  // Only squash proc pipeline when not stalled
    else if (predict_result_rdy & (ctrl_load!=prediction_saved) & predict_en_saved)
      predict_result=~we;
    else if (pc_save)
      predict_result=~(squash_save&~is_delayslot);
end

assign next_pc=(pc_advance) ? _next_pc : pc;


/********************* Delay slot protection *******************************/
// We have to do the protection here since we've separated this ifetch and its
// stalls from the processor pipeline, we just emit nops.  SPREE automatically
// protects delay slots but since we can't tell it when the delay slot is
// stalled, we have to do it here.

branch_detector branch_detector (
  .func(func),
  .opcode(opcode),
  .is_branch(is_branch));

pipereg pipereg (
  .clk(clk),
  .resetn(resetn),
  .d(is_branch),
  .squashn(1'b1),
  .en(pc_advance),
  .q(is_delayslot));
  defparam
    pipereg.WIDTH=1;
/***************************************************************************/

wire prediction_tmp;
wire predict_result_rdy_tmp;

assign predict_result_rdy_tmp=predict_result_rdy&predict_en_saved;

branchpredict bpredictor ( 
    .clk(clk),
    .resetn(resetn),
    .predict(en),
    .prediction(prediction_tmp),
    .pc_predict({next_pc[I_DATAWIDTH-1:3],2'b0}),
    .result_rdy(predict_result_rdy_tmp),
    .result(ctrl_load),
    .pc_result({pc_rollbacknottaken[I_DATAWIDTH-1:3],2'b0}) );

assign prediction=(pcwrop!=1) ? prediction_tmp :1;

endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: ifetch_pipe_bpred_bus_int.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: components.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/components.v', mode 'r' at 0x7f7855a0c780>
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Generic Register
****************************************************************************/
module register(d,clk,resetn,en,q);
parameter WIDTH=32;

input clk;
input resetn;
input en;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
reg [WIDTH-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule

/****************************************************************************
          Generic Register - synchronous reset
****************************************************************************/
module register_sync(d,clk,resetn,en,q);
parameter WIDTH=32;

input clk;
input resetn;
input en;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
reg [WIDTH-1:0] q;

always @(posedge clk)		//synchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule

/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg(d,clk,resetn,en,squashn,q);
parameter WIDTH=32;

input clk;
input resetn;
input en;
input squashn;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
reg [WIDTH-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule

/****************************************************************************
          Generic Pipelined Register 2 -OLD: If not enabled, queues squash

          - This piperegister stalls the reset signal as well
module pipereg_full(d,clk,resetn,squashn,en,q);
parameter WIDTH=32;

input clk;
input resetn;
input en;
input squashn;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
reg [WIDTH-1:0] q;
reg squash_save;

  always @(posedge clk)   //synchronous reset
  begin
    if (resetn==0 || (squashn==0 && en==1) || (squash_save&en))
      q<=0;
    else if (en==1)
      q<=d;
  end

  always @(posedge clk)
  begin
    if (resetn==1 && squashn==0 && en==0)
      squash_save<=1;
    else
      squash_save<=0;
  end
endmodule
****************************************************************************/

/****************************************************************************
          One cycle Stall circuit
****************************************************************************/
module onecyclestall(request,clk,resetn,stalled);
input request;
input clk;
input resetn;
output stalled;

  reg T,Tnext;

  // State machine for Stalling 1 cycle
  always@(request or T)
  begin
    case(T) 
      1'b0: Tnext=request;
      1'b1: Tnext=0;
    endcase 
  end       
  always@(posedge clk)
    if (~resetn)
      T<=0; 
    else    
      T<=Tnext;
  assign stalled=(request&~T);
endmodule

/****************************************************************************
          Multi cycle Stall circuit - with wait signal

          - One FF plus one 2:1 mux to stall 1st cycle on request, then wait
          - this makes wait don't care for the first cycle
****************************************************************************/
module multicyclestall(request, devwait,clk,resetn,stalled);
input request;
input devwait;
input clk;
input resetn;
output stalled;

  reg T;

  always@(posedge clk)
    if (~resetn)
      T<=0;
    else
      T<=stalled;

  assign stalled=(T) ? devwait : request;
endmodule
                
/****************************************************************************
          One cycle - Pipeline delay register
****************************************************************************/
module pipedelayreg(d,en,clk,resetn,squashn,dst,stalled,q);
parameter WIDTH=32;
input [WIDTH-1:0] d;
input [4:0] dst;
input en;
input clk;
input resetn;
input squashn;
output stalled;
output [WIDTH-1:0] q;

  reg [WIDTH-1:0] q;
  reg T,Tnext;

  // State machine for Stalling 1 cycle
  always@(en or T or dst)
  begin
    case(T) 
      0: Tnext=en&(|dst);
      1: Tnext=0;
    endcase 
  end       
  always@(posedge clk)
    if (~resetn)
      T<=0; 
    else    
      T<=Tnext;

  always @(posedge clk)   //synchronous reset
  begin
    if (resetn==0 || squashn==0)
      q<=0;
    else if (en==1)
      q<=d;
  end

  assign stalled=(en&~T&(|dst));
endmodule

/****************************************************************************
          Fake Delay
****************************************************************************/
module fakedelay(d,clk,q);
parameter WIDTH=32;
input [WIDTH-1:0] d;
input clk;
output [WIDTH-1:0] q;

assign q=d;

endmodule

/****************************************************************************
          Zeroer
****************************************************************************/
module zeroer(d,en,q);
parameter WIDTH=32;

input en;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
assign q= (en) ? d : 0;

endmodule

/****************************************************************************
          NOP - used to hack position of multiplexors
****************************************************************************/
module nop(d,q);
parameter WIDTH=32;

input [WIDTH-1:0] d;
output [WIDTH-1:0] q;

  assign q=d;

endmodule

/****************************************************************************
          Const
****************************************************************************/
module const (out);

parameter WIDTH=32;
parameter VAL=31;

output [WIDTH-1:0] out;

assign out=VAL;

endmodule

/****************************************************************************
          Branch detector
****************************************************************************/
module branch_detector(opcode, func, is_branch);
input [5:0] opcode;
input [5:0] func;
output is_branch;

wire is_special;

assign is_special=!(|opcode);
assign is_branch=((!(|opcode[5:3])) && !is_special) || 
                  ((is_special)&&(func[5:3]==3'b001));

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: components.v
//////////////////////////////////////////////////////////////////////


module system ( 
	clk,
	resetn,
	boot_iaddr,
	boot_idata,
	boot_iwe,
	boot_daddr,
	boot_ddata,
	boot_dwe,
	ifetch_bus_ecause,
	ifetch_bus_squashn,
	ifetch_bus_address,
	ifetch_bus_en,
	ifetch_bus_readdata,
	ifetch_bus_wait,
	data_mem_bus_ecause,
	data_mem_bus_address,
	data_mem_bus_en,
	data_mem_bus_we,
	data_mem_bus_byteen,
	data_mem_bus_writedata,
	data_mem_bus_readdata,
	data_mem_bus_wait,
	cop2_fromcop2_wait,
	cop2_fromcop2_en,
	cop2_fromcop2,
	cop2_tocop2_wait,
	cop2_tocop2_en,
	cop2_tocop2,
	cop0_badvaddr_we,
	cop0_badvaddr_in,
	cop0_ext_cause_in,

  // PETES CHANGE for tracing
  trc_addr,
  trc_data,
  trc_we,
  trc_stall,
  trc_pipestall,

	nop10_q
	);

/************************* IO Declarations *********************/
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: isa.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/isa.v', mode 'r' at 0x7f7855a0c810>
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          ISA definition file

  - The MIPS I ISA has a 6 bit opcode in the upper 6 bits.  
  - The opcode can also specify a "class".  There are two classes:
            1.  SPECIAL - look in lowest 6 bits to find operation
            2.  REGIMM - look in [20:16] to find type of branch

****************************************************************************/

/****** OPCODES - bits 31...26 *******/

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
input clk;
input resetn;
input [31:0] boot_iaddr;
input [31:0] boot_idata;
input boot_iwe;
input [31:0] boot_daddr;
input [31:0] boot_ddata;
input boot_dwe;
input	[ 31 : 0 ]	ifetch_bus_ecause;
output	ifetch_bus_squashn;
output	[ 31 : 0 ]	ifetch_bus_address;
output	ifetch_bus_en;
input	[ 31 : 0 ]	ifetch_bus_readdata;
input	ifetch_bus_wait;
input	[ 31 : 0 ]	data_mem_bus_ecause;
output	[ 31 : 0 ]	data_mem_bus_address;
output	data_mem_bus_en;
output	data_mem_bus_we;
output	[ 3 : 0 ]	data_mem_bus_byteen;
output	[ 31 : 0 ]	data_mem_bus_writedata;
input	[ 31 : 0 ]	data_mem_bus_readdata;
input	data_mem_bus_wait;
output	cop2_fromcop2_wait;
input	cop2_fromcop2_en;
input	[ 31 : 0 ]	cop2_fromcop2;
input	cop2_tocop2_wait;
output	cop2_tocop2_en;
output	[ 31 : 0 ]	cop2_tocop2;
input	cop0_badvaddr_we;
input	[ 31 : 0 ]	cop0_badvaddr_in;
input	[ 31 : 0 ]	cop0_ext_cause_in;
output [31:0] nop10_q;

// PETES CHANGE for tracing
output  [ 4 : 0 ]   trc_addr;
output  [ 31 : 0 ]  trc_data;
output              trc_we;
input               trc_stall;
output              trc_pipestall;


/*********************** Signal Declarations *******************/
wire	branch_mispred;
wire	stall_2nd_delayslot;
wire	has_delayslot;
wire	haz_zeroer0_q_pipereg5_q;
wire	haz_zeroer_q_pipereg5_q;
		// Datapath signals declarations
wire	addersub_result_slt;
wire	[ 31 : 0 ]	addersub_result;
wire	[ 31 : 0 ]	logic_unit_result;
wire	[ 31 : 0 ]	ifetch_pc_out;
wire	[ 31 : 0 ]	ifetch_instr;
wire	[ 5 : 0 ]	ifetch_opcode;
wire	[ 5 : 0 ]	ifetch_func;
wire	[ 4 : 0 ]	ifetch_rs;
wire	[ 4 : 0 ]	ifetch_rt;
wire	[ 4 : 0 ]	ifetch_rd;
wire	[ 25 : 0 ]	ifetch_instr_index;
wire	[ 15 : 0 ]	ifetch_offset;
wire	[ 4 : 0 ]	ifetch_sa;
wire	[ 31 : 0 ]	ifetch_next_pc;
wire	ifetch_predict_result;
wire	[ 31 : 0 ]	ifetch_ecause;
wire	[ 31 : 0 ]	ifetch_epc;
wire	[ 31 : 0 ]	mul_shift_result;
wire	[ 31 : 0 ]	mul_lo;
wire	[ 31 : 0 ]	mul_hi;
wire	ctrl_mul_stalled;
wire	[ 31 : 0 ]	div_remainder;
wire	[ 31 : 0 ]	div_quotient;
wire	ctrl_div_stalled;
wire	[ 31 : 0 ]	data_mem_d_loadresult;
wire	[ 31 : 0 ]	data_mem_ecause;
wire	ctrl_data_mem_stalled;
wire	[ 31 : 0 ]	reg_file_b_readdataout;
wire	[ 31 : 0 ]	reg_file_a_readdataout;
wire	[ 31 : 0 ]	pcadder_result;
wire	[ 31 : 0 ]	signext16_out;
wire	[ 31 : 0 ]	merge26lo_out;
wire	branchresolve_eqz;
wire	branchresolve_gez;
wire	branchresolve_gtz;
wire	branchresolve_lez;
wire	branchresolve_ltz;
wire	branchresolve_ne;
wire	branchresolve_eq;
wire	[ 31 : 0 ]	lo_reg_q;
wire	[ 31 : 0 ]	hi_reg_q;
wire	[ 31 : 0 ]	const11_out;
wire	[ 31 : 0 ]	const12_out;
wire	[ 31 : 0 ]	const_out;
wire	[ 31 : 0 ]	pipereg_q;
wire	[ 4 : 0 ]	pipereg5_q;
wire	[ 4 : 0 ]	pipereg2_q;
wire	[ 31 : 0 ]	pipereg6_q;
wire	[ 31 : 0 ]	pipereg26_q;
wire	[ 31 : 0 ]	pipereg8_q;
wire	pipereg7_q;
wire	[ 31 : 0 ]	fakedelay_q;
wire	[ 31 : 0 ]	pipereg27_q;
wire	[ 31 : 0 ]	pipereg28_q;
wire	[ 31 : 0 ]	pipereg29_q;
wire	[ 31 : 0 ]	pipereg30_q;
wire	[ 31 : 0 ]	nop_q;
wire	[ 31 : 0 ]	nop10_q;
wire	[ 31 : 0 ]	nop13_q;
wire	[ 31 : 0 ]	nop9_q;
wire	[ 4 : 0 ]	zeroer_q;
wire	[ 4 : 0 ]	zeroer0_q;
wire	[ 4 : 0 ]	zeroer4_q;
wire	[ 31 : 0 ]	cop2_tocpu;
wire	ctrl_cop2_stalled;
wire	[ 31 : 0 ]	cop0_status;
wire	[ 31 : 0 ]	cop0_tocpu;
wire	cop0_exception;
wire	ctrl_cop0_stalled;
wire	[ 31 : 0 ]	mux2to1_mul_opA_out;
wire	[ 31 : 0 ]	mux2to1_addersub_opA_out;
wire	[ 4 : 0 ]	mux3to1_mul_sa_out;
wire	[ 31 : 0 ]	mux2to1_hi_reg_d_out;
wire	[ 31 : 0 ]	mux2to1_lo_reg_d_out;
wire	[ 31 : 0 ]	mux9to1_nop13_d_out;
wire	[ 31 : 0 ]	mux2to1_pipereg_d_out;
wire	[ 31 : 0 ]	mux2to1_pipereg6_d_out;
wire	mux6to1_pipereg7_d_out;
wire	[ 31 : 0 ]	mux3to1_nop9_d_out;
wire	[ 4 : 0 ]	mux3to1_zeroer4_d_out;
wire	[ 5 : 0 ]	pipereg15_q;
wire	[ 4 : 0 ]	pipereg16_q;
wire	[ 5 : 0 ]	pipereg14_q;
wire	branch_detector_is_branch;
wire	[ 4 : 0 ]	pipereg17_q;
wire	[ 5 : 0 ]	pipereg19_q;
wire	[ 5 : 0 ]	pipereg18_q;
wire	[ 4 : 0 ]	pipereg20_q;
wire	[ 4 : 0 ]	pipereg21_q;
wire	pipereg22_q;
wire	pipereg23_q;
wire	[ 31 : 0 ]	mux2to1_nop_d_out;
wire	pipereg31_q;
wire	[ 31 : 0 ]	mux2to1_nop10_d_out;
wire	pipereg32_q;
wire	pipereg25_q;
/***************** Control Signals ***************/
		//Decoded Opcode signal declarations
reg	[ 2 : 0 ]	ctrl_mux6to1_pipereg7_d_sel;
reg	[ 3 : 0 ]	ctrl_mux9to1_nop13_d_sel;
reg	[ 1 : 0 ]	ctrl_mux3to1_nop9_d_sel;
reg	ctrl_mux2to1_lo_reg_d_sel;
reg	ctrl_mux2to1_hi_reg_d_sel;
reg	ctrl_mux2to1_mul_opA_sel;
reg	[ 1 : 0 ]	ctrl_mux3to1_mul_sa_sel;
reg	ctrl_mux2to1_addersub_opA_sel;
reg	ctrl_mux2to1_pipereg6_d_sel;
reg	[ 1 : 0 ]	ctrl_mux3to1_zeroer4_d_sel;
reg	ctrl_mux2to1_pipereg_d_sel;
reg	ctrl_zeroer4_en;
reg	ctrl_zeroer0_en;
reg	ctrl_zeroer_en;
reg	ctrl_ifetch_pcwrop;
reg	ctrl_ifetch_op;
reg	[ 2 : 0 ]	ctrl_addersub_op;
reg	[ 3 : 0 ]	ctrl_data_mem_op;
reg	ctrl_div_sign;
reg	[ 2 : 0 ]	ctrl_mul_op;
reg	[ 1 : 0 ]	ctrl_logic_unit_op;
		//Enable signal declarations
reg	ctrl_cop0_fromcpu_en;
reg	ctrl_cop0_tocpu_en;
reg	ctrl_cop2_fromcpu_en;
reg	ctrl_cop2_tocpu_en;
reg	ctrl_lo_reg_en;
reg	ctrl_ifetch_we;
reg	ctrl_hi_reg_en;
reg	ctrl_branchresolve_en;
reg	ctrl_reg_file_c_we;
reg	ctrl_reg_file_b_en;
reg	ctrl_reg_file_a_en;
reg	ctrl_ifetch_en;
reg	ctrl_data_mem_en;
reg	ctrl_div_en;
reg	ctrl_mul_start;
		//Other Signals
wire	squash_stage3;
wire	stall_out_stage3;
wire	squash_stage2;
wire	stall_out_stage2;
wire	ctrl_pipereg25_squashn;
wire	ctrl_pipereg8_squashn;
wire	ctrl_pipereg7_squashn;
wire	ctrl_pipereg18_squashn;
wire	ctrl_pipereg19_squashn;
wire	ctrl_pipereg20_squashn;
wire	ctrl_pipereg21_squashn;
wire	ctrl_pipereg25_resetn;
wire	ctrl_pipereg8_resetn;
wire	ctrl_pipereg7_resetn;
wire	ctrl_pipereg18_resetn;
wire	ctrl_pipereg19_resetn;
wire	ctrl_pipereg20_resetn;
wire	ctrl_pipereg21_resetn;
wire	ctrl_pipereg25_en;
wire	ctrl_pipereg8_en;
wire	ctrl_pipereg7_en;
wire	ctrl_pipereg18_en;
wire	ctrl_pipereg19_en;
wire	ctrl_pipereg20_en;
wire	ctrl_pipereg21_en;
wire	squash_stage1;
wire	stall_out_stage1;
wire	ctrl_pipereg32_squashn;
wire	ctrl_pipereg31_squashn;
wire	ctrl_pipereg30_squashn;
wire	ctrl_pipereg29_squashn;
wire	ctrl_pipereg28_squashn;
wire	ctrl_pipereg27_squashn;
wire	ctrl_pipereg26_squashn;
wire	ctrl_pipereg23_squashn;
wire	ctrl_pipereg_squashn;
wire	ctrl_pipereg5_squashn;
wire	ctrl_pipereg2_squashn;
wire	ctrl_pipereg6_squashn;
wire	ctrl_pipereg14_squashn;
wire	ctrl_pipereg15_squashn;
wire	ctrl_pipereg16_squashn;
wire	ctrl_pipereg17_squashn;
wire	ctrl_pipereg32_resetn;
wire	ctrl_pipereg31_resetn;
wire	ctrl_pipereg30_resetn;
wire	ctrl_pipereg29_resetn;
wire	ctrl_pipereg28_resetn;
wire	ctrl_pipereg27_resetn;
wire	ctrl_pipereg26_resetn;
wire	ctrl_pipereg23_resetn;
wire	ctrl_pipereg_resetn;
wire	ctrl_pipereg5_resetn;
wire	ctrl_pipereg2_resetn;
wire	ctrl_pipereg6_resetn;
wire	ctrl_pipereg14_resetn;
wire	ctrl_pipereg15_resetn;
wire	ctrl_pipereg16_resetn;
wire	ctrl_pipereg17_resetn;
wire	ctrl_pipereg32_en;
wire	ctrl_pipereg31_en;
wire	ctrl_pipereg30_en;
wire	ctrl_pipereg29_en;
wire	ctrl_pipereg28_en;
wire	ctrl_pipereg27_en;
wire	ctrl_pipereg26_en;
wire	ctrl_pipereg23_en;
wire	ctrl_pipereg_en;
wire	ctrl_pipereg5_en;
wire	ctrl_pipereg2_en;
wire	ctrl_pipereg6_en;
wire	ctrl_pipereg14_en;
wire	ctrl_pipereg15_en;
wire	ctrl_pipereg16_en;
wire	ctrl_pipereg17_en;
reg	predictme;


/****************************** Control **************************/
		//Decode Logic for Opcode and Multiplex Select signals
always@(ifetch_opcode or ifetch_func or ifetch_rt or ifetch_rs)
begin
		// Initialize control opcodes to zero
	ctrl_mux2to1_pipereg6_d_sel = 0;
	ctrl_mux3to1_zeroer4_d_sel = 0;
	ctrl_mux2to1_pipereg_d_sel = 0;
	ctrl_zeroer4_en = 0;
	ctrl_zeroer0_en = 0;
	ctrl_zeroer_en = 0;
	
	casex (ifetch_opcode)
		OP_ADDI:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_ADDIU:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_ANDI:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_BEQ:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_BGTZ:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
			ctrl_zeroer_en = 1;
		end
		OP_BLEZ:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
			ctrl_zeroer_en = 1;
		end
		OP_BNE:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_COP0:
		casex (ifetch_rs)
			COP0_MFC0:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 2;
				ctrl_zeroer4_en = 1;
			end
			COP0_MTC0:
				ctrl_zeroer0_en = 1;
		endcase
		OP_COP2:
		casex (ifetch_func)
			COP2_FUNC_CFC2:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 2;
				ctrl_zeroer4_en = 1;
			end
			COP2_FUNC_CTC2:
				ctrl_zeroer0_en = 1;
			COP2_FUNC_MTC2:
				ctrl_zeroer0_en = 1;
		endcase
		OP_J:
			ctrl_mux2to1_pipereg6_d_sel = 0;
		OP_JAL:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 0;
			ctrl_zeroer4_en = 1;
		end
		OP_LB:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_LBU:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_LH:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_LHU:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_LUI:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
		end
		OP_LW:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_ORI:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_REGIMM:
		casex (ifetch_rt[0])
			FUNC_BGEZ:
			begin
				ctrl_mux2to1_pipereg6_d_sel = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_BLTZ:
			begin
				ctrl_mux2to1_pipereg6_d_sel = 1;
				ctrl_zeroer_en = 1;
			end
		endcase
		OP_SB:
		begin
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SH:
		begin
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SLTI:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SLTIU:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SPECIAL:
		casex (ifetch_func)
			FUNC_ADD:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_ADDU:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_AND:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_DIV:
			begin
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_DIVU:
			begin
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_JALR:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_JR:
				ctrl_zeroer_en = 1;
			FUNC_MFHI:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
			end
			FUNC_MFLO:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
			end
			FUNC_MULT:
			begin
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_MULTU:
			begin
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_NOR:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_OR:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SLL:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
			end
			FUNC_SLLV:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SLT:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SLTU:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SRA:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
			end
			FUNC_SRAV:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SRL:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
			end
			FUNC_SRLV:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SUB:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SUBU:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_XOR:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
		endcase
		OP_SW:
		begin
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_XORI:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
	endcase
end
		//Logic for enable signals in Pipe Stage 1
always@(ifetch_opcode or ifetch_func or ifetch_rt[0] or ifetch_rs or stall_out_stage2)
begin
	ctrl_reg_file_b_en = 1 &~stall_out_stage2;
	ctrl_reg_file_a_en = 1 &~stall_out_stage2;
	ctrl_ifetch_en = 1 &~stall_out_stage2;
end
		//Decode Logic for Opcode and Multiplex Select signals
always@(pipereg14_q or pipereg15_q or pipereg16_q or pipereg17_q)
begin
		// Initialize control opcodes to zero
	ctrl_mux6to1_pipereg7_d_sel = 0;
	ctrl_mux9to1_nop13_d_sel = 0;
	ctrl_mux3to1_nop9_d_sel = 0;
	ctrl_mux2to1_lo_reg_d_sel = 0;
	ctrl_mux2to1_hi_reg_d_sel = 0;
	ctrl_mux2to1_mul_opA_sel = 0;
	ctrl_mux3to1_mul_sa_sel = 0;
	ctrl_mux2to1_addersub_opA_sel = 0;
	ctrl_ifetch_pcwrop = 0;
	ctrl_addersub_op = 0;
	ctrl_data_mem_op = 0;
	ctrl_div_sign = 0;
	ctrl_mul_op = 0;
	ctrl_logic_unit_op = 0;
	
	casex (pipereg14_q)
		OP_ADDI:
		begin
			ctrl_mux9to1_nop13_d_sel = 6;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
		end
		OP_ADDIU:
		begin
			ctrl_mux9to1_nop13_d_sel = 6;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 1;
		end
		OP_ANDI:
		begin
			ctrl_mux9to1_nop13_d_sel = 4;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_logic_unit_op = 0;
		end
		OP_BEQ:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 5;
			ctrl_ifetch_pcwrop = 0;
		end
		OP_BGTZ:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_ifetch_pcwrop = 0;
		end
		OP_BLEZ:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 3;
			ctrl_ifetch_pcwrop = 0;
		end
		OP_BNE:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 4;
			ctrl_ifetch_pcwrop = 0;
		end
		OP_COP0:
		casex (pipereg17_q)
			COP0_MFC0:
				ctrl_mux9to1_nop13_d_sel = 7;
		endcase
		OP_COP2:
		casex (pipereg15_q)
			COP2_FUNC_CFC2:
				ctrl_mux9to1_nop13_d_sel = 8;
		endcase
		OP_J:
			ctrl_ifetch_pcwrop = 1;
		OP_JAL:
		begin
			ctrl_mux9to1_nop13_d_sel = 6;
			ctrl_mux2to1_addersub_opA_sel = 1;
			ctrl_ifetch_pcwrop = 1;
			ctrl_addersub_op = 1;
		end
		OP_LB:
		begin
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 7;
		end
		OP_LBU:
		begin
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 3;
		end
		OP_LH:
		begin
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 5;
		end
		OP_LHU:
		begin
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 1;
		end
		OP_LUI:
		begin
			ctrl_mux9to1_nop13_d_sel = 3;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 1;
			ctrl_mul_op = 0;
		end
		OP_LW:
		begin
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 0;
		end
		OP_ORI:
		begin
			ctrl_mux9to1_nop13_d_sel = 4;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_logic_unit_op = 1;
		end
		OP_REGIMM:
		casex (pipereg16_q[0])
			FUNC_BGEZ:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 1;
				ctrl_ifetch_pcwrop = 0;
			end
			FUNC_BLTZ:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 2;
				ctrl_ifetch_pcwrop = 0;
			end
		endcase
		OP_SB:
		begin
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 11;
		end
		OP_SH:
		begin
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 9;
		end
		OP_SLTI:
		begin
			ctrl_mux9to1_nop13_d_sel = 5;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 6;
		end
		OP_SLTIU:
		begin
			ctrl_mux9to1_nop13_d_sel = 5;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 4;
		end
		OP_SPECIAL:
		casex (pipereg15_q)
			FUNC_ADD:
			begin
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_addersub_op = 3;
			end
			FUNC_ADDU:
			begin
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_addersub_op = 1;
			end
			FUNC_AND:
			begin
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_logic_unit_op = 0;
			end
			FUNC_DIV:
			begin
				ctrl_mux2to1_lo_reg_d_sel = 1;
				ctrl_mux2to1_hi_reg_d_sel = 1;
				ctrl_div_sign = 1;
			end
			FUNC_DIVU:
			begin
				ctrl_mux2to1_lo_reg_d_sel = 1;
				ctrl_mux2to1_hi_reg_d_sel = 1;
				ctrl_div_sign = 0;
			end
			FUNC_JALR:
			begin
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux2to1_addersub_opA_sel = 1;
				ctrl_addersub_op = 1;
			end
			FUNC_MFHI:
				ctrl_mux9to1_nop13_d_sel = 1;
			FUNC_MFLO:
				ctrl_mux9to1_nop13_d_sel = 0;
			FUNC_MULT:
			begin
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 1;
				ctrl_mul_op = 6;
			end
			FUNC_MULTU:
			begin
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 1;
				ctrl_mul_op = 4;
			end
			FUNC_NOR:
			begin
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_logic_unit_op = 3;
			end
			FUNC_OR:
			begin
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_logic_unit_op = 1;
			end
			FUNC_SLL:
			begin
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mul_op = 0;
			end
			FUNC_SLLV:
			begin
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 2;
				ctrl_mul_op = 0;
			end
			FUNC_SLT:
			begin
				ctrl_mux9to1_nop13_d_sel = 5;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_addersub_op = 6;
			end
			FUNC_SLTU:
			begin
				ctrl_mux9to1_nop13_d_sel = 5;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_addersub_op = 4;
			end
			FUNC_SRA:
			begin
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mul_op = 3;
			end
			FUNC_SRAV:
			begin
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 2;
				ctrl_mul_op = 3;
			end
			FUNC_SRL:
			begin
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mul_op = 1;
			end
			FUNC_SRLV:
			begin
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 2;
				ctrl_mul_op = 1;
			end
			FUNC_SUB:
			begin
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_addersub_op = 0;
			end
			FUNC_SUBU:
			begin
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_addersub_op = 2;
			end
			FUNC_XOR:
			begin
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_logic_unit_op = 2;
			end
		endcase
		OP_SW:
		begin
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 8;
		end
		OP_XORI:
		begin
			ctrl_mux9to1_nop13_d_sel = 4;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_logic_unit_op = 2;
		end
	endcase
end
		//Logic for enable signals in Pipe Stage 2
always@(pipereg14_q or pipereg15_q or pipereg16_q[0] or pipereg17_q or stall_out_stage3 or ctrl_mul_stalled or ctrl_data_mem_stalled or ctrl_div_stalled or ctrl_cop2_stalled or ctrl_cop0_stalled)
begin
	ctrl_cop0_fromcpu_en = 0;
	ctrl_cop0_tocpu_en = 0;
	ctrl_cop2_fromcpu_en = 0;
	ctrl_cop2_tocpu_en = 0;
	ctrl_lo_reg_en = 0;
	ctrl_hi_reg_en = 0;
	ctrl_branchresolve_en = 0;
	ctrl_reg_file_c_we = 0;
	ctrl_data_mem_en = 0;
	ctrl_div_en = 0;
	ctrl_mul_start = 0;
	casex (pipereg14_q)
		OP_ADDI:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_ADDIU:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_ANDI:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_BEQ:
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_BGTZ:
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_BLEZ:
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_BNE:
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_COP0:
		casex (pipereg17_q)
			COP0_MFC0:
			begin
				ctrl_cop0_tocpu_en = 1 &~stall_out_stage3;
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			end
			COP0_MTC0:
				ctrl_cop0_fromcpu_en = 1 &~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		endcase
		OP_COP2:
		casex (pipereg15_q)
			COP2_FUNC_CFC2:
			begin
				ctrl_cop2_tocpu_en = 1 &~stall_out_stage3;
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			end
			COP2_FUNC_CTC2:
				ctrl_cop2_fromcpu_en = 1 &~ctrl_cop0_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			COP2_FUNC_MTC2:
				ctrl_cop2_fromcpu_en = 1 &~ctrl_cop0_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		endcase
		OP_JAL:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_LB:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		end
		OP_LBU:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		end
		OP_LH:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		end
		OP_LHU:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		end
		OP_LUI:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_mul_start = 1 &~stall_out_stage3;
		end
		OP_LW:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		end
		OP_ORI:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_REGIMM:
		casex (pipereg16_q[0])
			FUNC_BGEZ:
				ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_BLTZ:
				ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		endcase
		OP_SB:
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		OP_SH:
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		OP_SLTI:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_SLTIU:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_SPECIAL:
		casex (pipereg15_q)
			FUNC_ADD:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_ADDU:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_AND:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_DIV:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_div_en = 1 &~stall_out_stage3;
			end
			FUNC_DIVU:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_div_en = 1 &~stall_out_stage3;
			end
			FUNC_JALR:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_MFHI:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_MFLO:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_MULT:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_MULTU:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_NOR:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_OR:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_SLL:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_SLLV:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_SLT:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_SLTU:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_SRA:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_SRAV:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_SRL:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_SRLV:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_SUB:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_SUBU:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_XOR:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		endcase
		OP_SW:
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		OP_XORI:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
	endcase
end
		//Decode Logic for Opcode and Multiplex Select signals
always@(pipereg18_q or pipereg19_q or pipereg20_q or pipereg21_q)
begin
		// Initialize control opcodes to zero
	ctrl_ifetch_op = 0;
	
	casex (pipereg18_q)
		OP_BEQ:
			ctrl_ifetch_op = 0;
		OP_BGTZ:
			ctrl_ifetch_op = 0;
		OP_BLEZ:
			ctrl_ifetch_op = 0;
		OP_BNE:
			ctrl_ifetch_op = 0;
		OP_REGIMM:
		casex (pipereg20_q[0])
			FUNC_BGEZ:
				ctrl_ifetch_op = 0;
			FUNC_BLTZ:
				ctrl_ifetch_op = 0;
		endcase
		OP_SPECIAL:
		casex (pipereg19_q)
			FUNC_JALR:
				ctrl_ifetch_op = 1;
			FUNC_JR:
				ctrl_ifetch_op = 1;
		endcase
	endcase
end
		//Logic for enable signals in Pipe Stage 3
always@(pipereg18_q or pipereg19_q or pipereg20_q[0] or pipereg21_q or 1'b0)
begin
	ctrl_ifetch_we = 0;
	casex (pipereg18_q)
		OP_BEQ:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_BGTZ:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_BLEZ:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_BNE:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_REGIMM:
		casex (pipereg20_q[0])
			FUNC_BGEZ:
				ctrl_ifetch_we = 1 &~1'b0;
			FUNC_BLTZ:
				ctrl_ifetch_we = 1 &~1'b0;
		endcase
		OP_SPECIAL:
		casex (pipereg19_q)
			FUNC_JALR:
				ctrl_ifetch_we = 1 &~1'b0;
			FUNC_JR:
				ctrl_ifetch_we = 1 &~1'b0;
		endcase
	endcase
end

/********* Stall Network & PipeReg Control ********/
assign stall_out_stage1 = stall_out_stage2;
assign ctrl_pipereg17_en = ~stall_out_stage1;
assign ctrl_pipereg16_en = ~stall_out_stage1;
assign ctrl_pipereg15_en = ~stall_out_stage1;
assign ctrl_pipereg14_en = ~stall_out_stage1;
assign ctrl_pipereg6_en = ~stall_out_stage1;
assign ctrl_pipereg2_en = ~stall_out_stage1;
assign ctrl_pipereg5_en = ~stall_out_stage1;
assign ctrl_pipereg_en = ~stall_out_stage1;
assign ctrl_pipereg23_en = ~stall_out_stage1;
assign ctrl_pipereg26_en = ~stall_out_stage1;
assign ctrl_pipereg27_en = ~stall_out_stage1;
assign ctrl_pipereg28_en = ~stall_out_stage1;
assign ctrl_pipereg29_en = ~stall_out_stage1;
assign ctrl_pipereg30_en = ~stall_out_stage1;
assign ctrl_pipereg31_en = ~stall_out_stage1;
assign ctrl_pipereg32_en = ~stall_out_stage1;
assign stall_out_stage2 = stall_out_stage3|ctrl_cop0_stalled|ctrl_cop2_stalled|ctrl_div_stalled|ctrl_data_mem_stalled|ctrl_mul_stalled;
assign ctrl_pipereg21_en = ~stall_out_stage2;
assign ctrl_pipereg20_en = ~stall_out_stage2;
assign ctrl_pipereg19_en = ~stall_out_stage2;
assign ctrl_pipereg18_en = ~stall_out_stage2;
assign ctrl_pipereg7_en = ~stall_out_stage2;
assign ctrl_pipereg8_en = ~stall_out_stage2;
assign ctrl_pipereg25_en = ~stall_out_stage2;
assign stall_out_stage3 = 1'b0;
assign branch_mispred = (!ifetch_predict_result);
assign stall_2nd_delayslot = branch_detector_is_branch&has_delayslot;
assign has_delayslot = pipereg22_q;

		//Identify branches that will be predicted
always@(ifetch_opcode or ifetch_func or ifetch_rt[0] or ifetch_rs)
begin
	predictme=0;
	casex (ifetch_opcode)
		OP_BEQ:
			predictme=1;
		OP_BGTZ:
			predictme=1;
		OP_BLEZ:
			predictme=1;
		OP_BNE:
			predictme=1;
		OP_J:
			predictme=1;
		OP_JAL:
			predictme=1;
		OP_REGIMM:
		casex (ifetch_rt[0])
			FUNC_BGEZ:
				predictme=1;
			FUNC_BLTZ:
				predictme=1;
		endcase
	endcase
end

assign squash_stage1 = ((stall_out_stage1&~stall_out_stage2))|~resetn;
assign ctrl_pipereg17_resetn = ~squash_stage1;
assign ctrl_pipereg16_resetn = ~squash_stage1;
assign ctrl_pipereg15_resetn = ~squash_stage1;
assign ctrl_pipereg14_resetn = ~squash_stage1;
assign ctrl_pipereg6_resetn = ~squash_stage1;
assign ctrl_pipereg2_resetn = ~squash_stage1;
assign ctrl_pipereg5_resetn = ~squash_stage1;
assign ctrl_pipereg_resetn = ~squash_stage1;
assign ctrl_pipereg23_resetn = ~squash_stage1;
assign ctrl_pipereg26_resetn = ~squash_stage1;
assign ctrl_pipereg27_resetn = ~squash_stage1;
assign ctrl_pipereg28_resetn = ~squash_stage1;
assign ctrl_pipereg29_resetn = ~squash_stage1;
assign ctrl_pipereg30_resetn = ~squash_stage1;
assign ctrl_pipereg31_resetn = ~squash_stage1;
assign ctrl_pipereg32_resetn = ~squash_stage1;
assign ctrl_pipereg32_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg31_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg30_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg29_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg28_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg27_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg26_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg23_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg5_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg2_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg6_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg14_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg15_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg16_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg17_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_ifetch_squashn = ~((branch_mispred&~(pipereg22_q)) || (cop0_exception));
assign squash_stage2 = ((stall_out_stage2&~stall_out_stage3))|~resetn;
assign ctrl_pipereg21_resetn = ~squash_stage2;
assign ctrl_pipereg20_resetn = ~squash_stage2;
assign ctrl_pipereg19_resetn = ~squash_stage2;
assign ctrl_pipereg18_resetn = ~squash_stage2;
assign ctrl_pipereg7_resetn = ~squash_stage2;
assign ctrl_pipereg8_resetn = ~squash_stage2;
assign ctrl_pipereg25_resetn = ~squash_stage2;
assign ctrl_pipereg25_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg8_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg7_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg18_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg19_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg20_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg21_squashn = ~((0) || (cop0_exception));
assign ctrl_lo_reg_squashn = ~((0) || (cop0_exception));
assign ctrl_hi_reg_squashn = ~((0) || (cop0_exception));
assign ctrl_reg_file_c_squashn = ~((0) || (cop0_exception));
assign squash_stage3 = ((stall_out_stage3&~1'b0))|~resetn;

/****************************** Datapath **************************/
/******************** Hazard Detection Logic ***********************/
assign haz_zeroer0_q_pipereg5_q = (zeroer0_q==pipereg5_q) && (|zeroer0_q);
assign haz_zeroer_q_pipereg5_q = (zeroer_q==pipereg5_q) && (|zeroer_q);

/*************** DATAPATH COMPONENTS **************/
addersub addersub (
	.opB(nop9_q),
	.opA(mux2to1_addersub_opA_out),
	.op(ctrl_addersub_op),
	.result_slt(addersub_result_slt),
	.result(addersub_result));
	defparam
		addersub.WIDTH=32;

logic_unit logic_unit (
	.opB(nop9_q),
	.opA(nop_q),
	.op(ctrl_logic_unit_op),
	.result(logic_unit_result));
	defparam
		logic_unit.WIDTH=32;

ifetch ifetch (
	.clk(clk),
	.resetn(resetn),
	.boot_iaddr(boot_iaddr),
	.boot_idata(boot_idata),
	.boot_iwe(boot_iwe),
	.bus_ecause(ifetch_bus_ecause),
	.bus_squashn(ifetch_bus_squashn),
	.bus_address(ifetch_bus_address),
	.bus_en(ifetch_bus_en),
	.bus_readdata(ifetch_bus_readdata),
	.bus_wait(ifetch_bus_wait),
	.interrupt(cop0_exception),
	.predict_result_rdy(pipereg25_q),
	.predict_en(pipereg23_q),
	.predict_tgt_pc(pipereg6_q),
	.load(pipereg7_q),
	.load_data(pipereg8_q),
	.pcwrop(ctrl_ifetch_pcwrop),
	.op(ctrl_ifetch_op),
	.we(ctrl_ifetch_we),
	.squashn(ctrl_ifetch_squashn),
	.en(ctrl_ifetch_en),
	.pc_out(ifetch_pc_out),
	.instr(ifetch_instr),
	.opcode(ifetch_opcode),
	.func(ifetch_func),
	.rs(ifetch_rs),
	.rt(ifetch_rt),
	.rd(ifetch_rd),
	.instr_index(ifetch_instr_index),
	.offset(ifetch_offset),
	.sa(ifetch_sa),
	.next_pc(ifetch_next_pc),
	.predict_result(ifetch_predict_result),
	.ecause(ifetch_ecause),
	.epc(ifetch_epc));

mul mul (
	.clk(clk),
	.resetn(resetn),
	.sa(mux3to1_mul_sa_out),
	.dst(pipereg5_q),
	.opB(nop10_q),
	.opA(mux2to1_mul_opA_out),
	.op(ctrl_mul_op),
	.start(ctrl_mul_start),
	.stalled(ctrl_mul_stalled),
	.shift_result(mul_shift_result),
	.lo(mul_lo),
	.hi(mul_hi));
	defparam
		mul.WIDTH=32;

div div (
	.clk(clk),
	.resetn(resetn),
	.divider(nop10_q),
	.dividend(nop_q),
	.sign(ctrl_div_sign),
	.en(ctrl_div_en),
	.stalled(ctrl_div_stalled),
	.remainder(div_remainder),
	.quotient(div_quotient));

data_mem data_mem (
	.clk(clk),
	.resetn(resetn),
	.boot_daddr(boot_daddr),
	.boot_ddata(boot_ddata),
	.boot_dwe(boot_dwe),
	.bus_ecause(data_mem_bus_ecause),
	.bus_address(data_mem_bus_address),
	.bus_en(data_mem_bus_en),
	.bus_we(data_mem_bus_we),
	.bus_byteen(data_mem_bus_byteen),
	.bus_writedata(data_mem_bus_writedata),
	.bus_readdata(data_mem_bus_readdata),
	//PETES CHANGE for tracing, was: .bus_wait(data_mem_bus_wait),
	.bus_wait(data_mem_bus_wait|trc_stall),
	.d_address(addersub_result),
	.d_writedata(nop10_q),
	.op(ctrl_data_mem_op),
	.en(ctrl_data_mem_en),
	.stalled(ctrl_data_mem_stalled),
	.d_loadresult(data_mem_d_loadresult),
	.ecause(data_mem_ecause));

reg_file reg_file (
	.clk(clk),
	.resetn(resetn),
	.c_writedatain(nop13_q),
	.c_reg(pipereg5_q),
	.b_reg(zeroer0_q),
	.a_reg(zeroer_q),
	.c_squashn(ctrl_reg_file_c_squashn),
	.c_we(ctrl_reg_file_c_we),
	.b_en(ctrl_reg_file_b_en),
	.a_en(ctrl_reg_file_a_en),
	.b_readdataout(reg_file_b_readdataout),
	.a_readdataout(reg_file_a_readdataout));

pcadder pcadder (
	.offset(signext16_out),
	.pc(ifetch_pc_out),
	.result(pcadder_result));

signext16 signext16 (
	.in(ifetch_offset),
	.out(signext16_out));

merge26lo merge26lo (
	.in2(ifetch_instr_index),
	.in1(ifetch_pc_out),
	.out(merge26lo_out));

branchresolve branchresolve (
	.rt(nop10_q),
	.rs(nop_q),
	.en(ctrl_branchresolve_en),
	.eqz(branchresolve_eqz),
	.gez(branchresolve_gez),
	.gtz(branchresolve_gtz),
	.lez(branchresolve_lez),
	.ltz(branchresolve_ltz),
	.ne(branchresolve_ne),
	.eq(branchresolve_eq));
	defparam
		branchresolve.WIDTH=32;

lo_reg lo_reg (
	.clk(clk),
	.resetn(resetn),
	.d(mux2to1_lo_reg_d_out),
	.squashn(ctrl_lo_reg_squashn),
	.en(ctrl_lo_reg_en),
	.q(lo_reg_q));
	defparam
		lo_reg.WIDTH=32;

hi_reg hi_reg (
	.clk(clk),
	.resetn(resetn),
	.d(mux2to1_hi_reg_d_out),
	.squashn(ctrl_hi_reg_squashn),
	.en(ctrl_hi_reg_en),
	.q(hi_reg_q));
	defparam
		hi_reg.WIDTH=32;

const const11 (
	.out(const11_out));
	defparam
		const11.WIDTH=32,
		const11.VAL=0;

const const12 (
	.out(const12_out));
	defparam
		const12.WIDTH=32,
		const12.VAL=16;

const const (
	.out(const_out));
	defparam
		const.WIDTH=32,
		const.VAL=31;

pipereg pipereg (
	.clk(clk),
	.resetn(ctrl_pipereg_resetn),
	.d(mux2to1_pipereg_d_out),
	.squashn(ctrl_pipereg_squashn),
	.en(ctrl_pipereg_en),
	.q(pipereg_q));
	defparam
		pipereg.WIDTH=32;

pipereg pipereg5 (
	.clk(clk),
	.resetn(ctrl_pipereg5_resetn),
	.d(zeroer4_q),
	.squashn(ctrl_pipereg5_squashn),
	.en(ctrl_pipereg5_en),
	.q(pipereg5_q));
	defparam
		pipereg5.WIDTH=5;

pipereg pipereg2 (
	.clk(clk),
	.resetn(ctrl_pipereg2_resetn),
	.d(ifetch_sa),
	.squashn(ctrl_pipereg2_squashn),
	.en(ctrl_pipereg2_en),
	.q(pipereg2_q));
	defparam
		pipereg2.WIDTH=5;

pipereg pipereg6 (
	.clk(clk),
	.resetn(ctrl_pipereg6_resetn),
	.d(mux2to1_pipereg6_d_out),
	.squashn(ctrl_pipereg6_squashn),
	.en(ctrl_pipereg6_en),
	.q(pipereg6_q));
	defparam
		pipereg6.WIDTH=32;

pipereg pipereg26 (
	.clk(clk),
	.resetn(ctrl_pipereg26_resetn),
	.d(nop13_q),
	.squashn(ctrl_pipereg26_squashn),
	.en(ctrl_pipereg26_en),
	.q(pipereg26_q));
	defparam
		pipereg26.WIDTH=32;

pipereg pipereg8 (
	.clk(clk),
	.resetn(ctrl_pipereg8_resetn),
	.d(nop_q),
	.squashn(ctrl_pipereg8_squashn),
	.en(ctrl_pipereg8_en),
	.q(pipereg8_q));
	defparam
		pipereg8.WIDTH=32;

pipereg pipereg7 (
	.clk(clk),
	.resetn(ctrl_pipereg7_resetn),
	.d(mux6to1_pipereg7_d_out),
	.squashn(ctrl_pipereg7_squashn),
	.en(ctrl_pipereg7_en),
	.q(pipereg7_q));
	defparam
		pipereg7.WIDTH=1;

fakedelay fakedelay (
	.clk(clk),
	.d(ifetch_pc_out),
	.q(fakedelay_q));
	defparam
		fakedelay.WIDTH=32;

pipereg pipereg27 (
	.clk(clk),
	.resetn(ctrl_pipereg27_resetn),
	.d(ifetch_instr),
	.squashn(ctrl_pipereg27_squashn),
	.en(ctrl_pipereg27_en),
	.q(pipereg27_q));
	defparam
		pipereg27.WIDTH=32;

pipereg pipereg28 (
	.clk(clk),
	.resetn(ctrl_pipereg28_resetn),
	.d(ifetch_epc),
	.squashn(ctrl_pipereg28_squashn),
	.en(ctrl_pipereg28_en),
	.q(pipereg28_q));
	defparam
		pipereg28.WIDTH=32;

pipereg pipereg29 (
	.clk(clk),
	.resetn(ctrl_pipereg29_resetn),
	.d(ifetch_rd),
	.squashn(ctrl_pipereg29_squashn),
	.en(ctrl_pipereg29_en),
	.q(pipereg29_q));
	defparam
		pipereg29.WIDTH=32;

pipereg pipereg30 (
	.clk(clk),
	.resetn(ctrl_pipereg30_resetn),
	.d(ifetch_ecause),
	.squashn(ctrl_pipereg30_squashn),
	.en(ctrl_pipereg30_en),
	.q(pipereg30_q));
	defparam
		pipereg30.WIDTH=32;

nop nop (
	.d(mux2to1_nop_d_out),
	.q(nop_q));
	defparam
		nop.WIDTH=32;

nop nop10 (
	.d(mux2to1_nop10_d_out),
	.q(nop10_q));
	defparam
		nop10.WIDTH=32;

nop nop13 (
	.d(mux9to1_nop13_d_out),
	.q(nop13_q));
	defparam
		nop13.WIDTH=32;

nop nop9 (
	.d(mux3to1_nop9_d_out),
	.q(nop9_q));
	defparam
		nop9.WIDTH=32;

zeroer zeroer (
	.d(ifetch_rs),
	.en(ctrl_zeroer_en),
	.q(zeroer_q));
	defparam
		zeroer.WIDTH=5;

zeroer zeroer0 (
	.d(ifetch_rt),
	.en(ctrl_zeroer0_en),
	.q(zeroer0_q));
	defparam
		zeroer0.WIDTH=5;

zeroer zeroer4 (
	.d(mux3to1_zeroer4_d_out),
	.en(ctrl_zeroer4_en),
	.q(zeroer4_q));
	defparam
		zeroer4.WIDTH=5;

cop2 cop2 (
	.clk(clk),
	.resetn(resetn),
	.fromcop2_wait(cop2_fromcop2_wait),
	.fromcop2_en(cop2_fromcop2_en),
	.fromcop2(cop2_fromcop2),
	.tocop2_wait(cop2_tocop2_wait),
	.tocop2_en(cop2_tocop2_en),
	.tocop2(cop2_tocop2),
	.fromcpu(nop10_q),
	.fromcpu_en(ctrl_cop2_fromcpu_en),
	.tocpu_en(ctrl_cop2_tocpu_en),
	.stalled(ctrl_cop2_stalled),
	.tocpu(cop2_tocpu));

cop0 cop0 (
	.clk(clk),
	.resetn(resetn),
	.badvaddr_we(cop0_badvaddr_we),
	.badvaddr_in(cop0_badvaddr_in),
	.ext_cause_in(cop0_ext_cause_in),
	.int_cause_in_stage2(data_mem_ecause),
	.int_cause_in_stage1(pipereg30_q),
	.epc_in(pipereg28_q),
	.fromcpu(nop10_q),
	.dest_addr(pipereg29_q),
	.read_addr(pipereg29_q),
	.instr(pipereg27_q),
	.fromcpu_en(ctrl_cop0_fromcpu_en),
	.tocpu_en(ctrl_cop0_tocpu_en),
	.stalled(ctrl_cop0_stalled),
	.status(cop0_status),
	.tocpu(cop0_tocpu),
	.exception(cop0_exception));

		// Multiplexor mux2to1_mul_opA instantiation
assign mux2to1_mul_opA_out = 
	(ctrl_mux2to1_mul_opA_sel==1) ? nop_q :
	nop9_q;

		// Multiplexor mux2to1_addersub_opA instantiation
assign mux2to1_addersub_opA_out = 
	(ctrl_mux2to1_addersub_opA_sel==1) ? fakedelay_q :
	nop_q;

		// Multiplexor mux3to1_mul_sa instantiation
assign mux3to1_mul_sa_out = 
	(ctrl_mux3to1_mul_sa_sel==2) ? nop_q :
	(ctrl_mux3to1_mul_sa_sel==1) ? const12_out :
	pipereg2_q;

		// Multiplexor mux2to1_hi_reg_d instantiation
assign mux2to1_hi_reg_d_out = 
	(ctrl_mux2to1_hi_reg_d_sel==1) ? div_remainder :
	mul_hi;

		// Multiplexor mux2to1_lo_reg_d instantiation
assign mux2to1_lo_reg_d_out = 
	(ctrl_mux2to1_lo_reg_d_sel==1) ? div_quotient :
	mul_lo;

		// Multiplexor mux9to1_nop13_d instantiation
assign mux9to1_nop13_d_out = 
	(ctrl_mux9to1_nop13_d_sel==8) ? cop2_tocpu :
	(ctrl_mux9to1_nop13_d_sel==7) ? cop0_tocpu :
	(ctrl_mux9to1_nop13_d_sel==6) ? addersub_result :
	(ctrl_mux9to1_nop13_d_sel==5) ? addersub_result_slt :
	(ctrl_mux9to1_nop13_d_sel==4) ? logic_unit_result :
	(ctrl_mux9to1_nop13_d_sel==3) ? mul_shift_result :
	(ctrl_mux9to1_nop13_d_sel==2) ? data_mem_d_loadresult :
	(ctrl_mux9to1_nop13_d_sel==1) ? hi_reg_q :
	lo_reg_q;

		// Multiplexor mux2to1_pipereg_d instantiation
assign mux2to1_pipereg_d_out = 
	(ctrl_mux2to1_pipereg_d_sel==1) ? ifetch_offset :
	signext16_out;

		// Multiplexor mux2to1_pipereg6_d instantiation
assign mux2to1_pipereg6_d_out = 
	(ctrl_mux2to1_pipereg6_d_sel==1) ? pcadder_result :
	merge26lo_out;

		// Multiplexor mux6to1_pipereg7_d instantiation
assign mux6to1_pipereg7_d_out = 
	(ctrl_mux6to1_pipereg7_d_sel==5) ? branchresolve_eq :
	(ctrl_mux6to1_pipereg7_d_sel==4) ? branchresolve_ne :
	(ctrl_mux6to1_pipereg7_d_sel==3) ? branchresolve_lez :
	(ctrl_mux6to1_pipereg7_d_sel==2) ? branchresolve_ltz :
	(ctrl_mux6to1_pipereg7_d_sel==1) ? branchresolve_gez :
	branchresolve_gtz;

		// Multiplexor mux3to1_nop9_d instantiation
assign mux3to1_nop9_d_out = 
	(ctrl_mux3to1_nop9_d_sel==2) ? pipereg_q :
	(ctrl_mux3to1_nop9_d_sel==1) ? nop10_q :
	const11_out;

		// Multiplexor mux3to1_zeroer4_d instantiation
assign mux3to1_zeroer4_d_out = 
	(ctrl_mux3to1_zeroer4_d_sel==2) ? ifetch_rt :
	(ctrl_mux3to1_zeroer4_d_sel==1) ? ifetch_rd :
	const_out;

pipereg pipereg15 (
	.clk(clk),
	.resetn(ctrl_pipereg15_resetn),
	.d(ifetch_func),
	.squashn(ctrl_pipereg15_squashn),
	.en(ctrl_pipereg15_en),
	.q(pipereg15_q));
	defparam
		pipereg15.WIDTH=6;

pipereg pipereg16 (
	.clk(clk),
	.resetn(ctrl_pipereg16_resetn),
	.d(ifetch_rt),
	.squashn(ctrl_pipereg16_squashn),
	.en(ctrl_pipereg16_en),
	.q(pipereg16_q));
	defparam
		pipereg16.WIDTH=5;

pipereg pipereg14 (
	.clk(clk),
	.resetn(ctrl_pipereg14_resetn),
	.d(ifetch_opcode),
	.squashn(ctrl_pipereg14_squashn),
	.en(ctrl_pipereg14_en),
	.q(pipereg14_q));
	defparam
		pipereg14.WIDTH=6;

branch_detector branch_detector (
	.func(ifetch_func),
	.opcode(ifetch_opcode),
	.is_branch(branch_detector_is_branch));

pipereg pipereg17 (
	.clk(clk),
	.resetn(ctrl_pipereg17_resetn),
	.d(ifetch_rs),
	.squashn(ctrl_pipereg17_squashn),
	.en(ctrl_pipereg17_en),
	.q(pipereg17_q));
	defparam
		pipereg17.WIDTH=5;

pipereg pipereg19 (
	.clk(clk),
	.resetn(ctrl_pipereg19_resetn),
	.d(pipereg15_q),
	.squashn(ctrl_pipereg19_squashn),
	.en(ctrl_pipereg19_en),
	.q(pipereg19_q));
	defparam
		pipereg19.WIDTH=6;

pipereg pipereg18 (
	.clk(clk),
	.resetn(ctrl_pipereg18_resetn),
	.d(pipereg14_q),
	.squashn(ctrl_pipereg18_squashn),
	.en(ctrl_pipereg18_en),
	.q(pipereg18_q));
	defparam
		pipereg18.WIDTH=6;

pipereg pipereg20 (
	.clk(clk),
	.resetn(ctrl_pipereg20_resetn),
	.d(pipereg16_q),
	.squashn(ctrl_pipereg20_squashn),
	.en(ctrl_pipereg20_en),
	.q(pipereg20_q));
	defparam
		pipereg20.WIDTH=5;

pipereg pipereg21 (
	.clk(clk),
	.resetn(ctrl_pipereg21_resetn),
	.d(pipereg17_q),
	.squashn(ctrl_pipereg21_squashn),
	.en(ctrl_pipereg21_en),
	.q(pipereg21_q));
	defparam
		pipereg21.WIDTH=5;

pipereg pipereg22 (
	.clk(clk),
	.resetn(resetn),
	.d(branch_detector_is_branch),
	.squashn(~branch_mispred),
	.en(~stall_out_stage1),
	.q(pipereg22_q));
	defparam
		pipereg22.WIDTH=1;

pipereg pipereg23 (
	.clk(clk),
	.resetn(ctrl_pipereg23_resetn),
	.d(predictme),
	.squashn(ctrl_pipereg23_squashn),
	.en(ctrl_pipereg23_en),
	.q(pipereg23_q));
	defparam
		pipereg23.WIDTH=1;

		// Multiplexor mux2to1_nop_d instantiation
assign mux2to1_nop_d_out = 
	(pipereg31_q==1) ? pipereg26_q :
	reg_file_a_readdataout;

pipereg pipereg31 (
	.clk(clk),
	.resetn(ctrl_pipereg31_resetn),
	.d(haz_zeroer_q_pipereg5_q),
	.squashn(ctrl_pipereg31_squashn),
	.en(ctrl_pipereg31_en),
	.q(pipereg31_q));
	defparam
		pipereg31.WIDTH=1;

		// Multiplexor mux2to1_nop10_d instantiation
assign mux2to1_nop10_d_out = 
	(pipereg32_q==1) ? pipereg26_q :
	reg_file_b_readdataout;

pipereg pipereg32 (
	.clk(clk),
	.resetn(ctrl_pipereg32_resetn),
	.d(haz_zeroer0_q_pipereg5_q),
	.squashn(ctrl_pipereg32_squashn),
	.en(ctrl_pipereg32_en),
	.q(pipereg32_q));
	defparam
		pipereg32.WIDTH=1;

pipereg pipereg25 (
	.clk(clk),
	.resetn(ctrl_pipereg25_resetn),
	.d(pipereg23_q),
	.squashn(ctrl_pipereg25_squashn),
	.en(ctrl_pipereg25_en),
	.q(pipereg25_q));
	defparam
		pipereg25.WIDTH=1;

// PETES CHANGE add trace signals
assign trc_data=nop13_q;
assign trc_we=ctrl_reg_file_c_we;
assign trc_addr=pipereg5_q;
assign trc_pipestall=stall_out_stage2;

endmodule
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/top/spram1.v', mode 'r' at 0x7f7855a0c9c0>
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/top/rams.v', mode 'r' at 0x7f7855a0c930>
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/local/local_add_sub.v', mode 'r' at 0x7f7855a0c9c0>
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/local/local_mult.v', mode 'r' at 0x7f7855a0c930>
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/local/local_fifo.v', mode 'r' at 0x7f7855a0c9c0>
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: <open file '/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/local/local_shifter.v', mode 'r' at 0x7f7855a0c930>
//////////////////////////////////////////////////////////////////////
