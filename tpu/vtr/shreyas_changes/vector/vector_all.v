//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vpu.v
//////////////////////////////////////////////////////////////////////
/******************************************************************************
  Vector Control Pipeline

          Stage 1       Stage 2         Stage 3
  ----|--------------|-----------------|-----------------|
      | Decode       | RF/EX           |WB & Send to cpu

 * Note, vs register file is written back to in Stage 3 also, but we check 
 * the vector lanes first to see if they're writing to the vs RF.  If so then
 * we stall.

******************************************************************************/

//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: options.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/top/options.v
//////////////////////////////////////////////////////////////////////
`ifndef _OPTIONS_V_
`define _OPTIONS_V_ 1

`define NO_PLI 1
//`define TEST_BENCH 1
`define USE_INHOUSE_LOGIC
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
`define LGDW 4

// DATA CACHE PREFETCHER - dcache needs to have 64 byte lines
`define LGDWB 7
`define DP 0
// VECTOR DATA CACHE PREFETCHER 0:off, 65535-N:N*veclength, N:pfch N cache lines
`define DPV 0

// VECTOR CORE
//Changing to 3. That is, we now have 8 lanes.
`define LGL 3
`define LGB 0
`define APB 0
`define LGM `LGL
`define LGX `LGL

// VECTOR ISA
`define LGMVL 6
`define LGVPW 1 //chaging the word size of vector processor to 16: support for bfloat16
`define LGLW 4

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

`define MAX_STAGES 14
/****************************** OTHER PARAMS *********************************/

// DRAM
`define LOG2DRAMWIDTHBITS 7

/****************** NUM PIPELINE STAGES in VECTOR PROCESSOR ***************/
//mult consumes 3 cycles
//`define MAX_PIPE_STAGES 7
//matmul consumes 29 cycles
`define MAX_PIPE_STAGES 8
`define MATMUL_STAGES 29

/****************** SIZE OF THE MATMUL UNIT ***************/
`define MAT_MUL_SIZE 8

`endif

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: options.v
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vregfile_base.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vregfile_base.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Base Register File

   - Has one read port (a) and one write port (c)
****************************************************************************/
module vregfile_base (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

parameter WIDTH=32;
parameter NUMREGS=16;
parameter LOG2NUMREGS=4;

input clk;
input resetn;

input [LOG2NUMREGS-1:0] a_reg,c_reg;
output [WIDTH-1:0] a_readdataout;
input [WIDTH-1:0] c_writedatain;
input a_en, c_we;

`ifdef USE_INHOUSE_LOGIC
        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(a_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we),
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

`else
	altsyncram	reg_file1(
				.wren_a (c_we),
				.clock0 (clk),
				.data_a (c_writedatain),
				.address_a (c_reg[LOG2NUMREGS-1:0]),
        .rden_b(a_en),
				.address_b (a_reg[LOG2NUMREGS-1:0]),
				.q_b (a_readdataout)
        // synopsys translate_off
        ,
        .aclr0 (1'b0),
        .aclr1 (1'b0),
        .byteena_a (1'b1),
        .byteena_b (1'b1),
        .data_b (32'b11111111),
        .wren_b (1'b0),
        .clock1 (1'b0),
        .clocken1 (1'b0),
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
		reg_file1.rdcontrol_reg_b = "CLOCK0",
		reg_file1.address_reg_b = "CLOCK0",
		reg_file1.address_aclr_b = "NONE",
		reg_file1.outdata_aclr_b = "NONE",
		reg_file1.read_during_write_mode_mixed_ports = "OLD_DATA",
		reg_file1.ram_block_type = "AUTO",
		reg_file1.intended_device_family = "Stratix";
`endif
endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vregfile_base.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vregfile_control.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vregfile_control.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Control Register File

   - Has one read port (a) and one write port (c)
   - vl promoted as first-class entities
****************************************************************************/
module vregfile_control (
    clk,
    resetn, 

    a_reg, 
    a_en,
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we,

    vl,
    matmul_masks,
    dma_en,
    lane_addr,
    mem_addr,
    num_bytes,
    dma_we,
    dma_busy,
    temp
    );

parameter WIDTH=32;
parameter NUMREGS=32;
parameter LOG2NUMREGS=5;

input clk;
input resetn;

input a_en;
input [LOG2NUMREGS-1:0] a_reg,c_reg;
output reg [WIDTH-1:0] a_readdataout;
input [WIDTH-1:0] c_writedatain;
input c_we;

output reg [WIDTH-1:0] vl;
output reg dma_en;
output reg [WIDTH-1:0] lane_addr;
output reg [WIDTH-1:0] mem_addr;
output reg [WIDTH-1:0] num_bytes;
output reg [WIDTH-1:0] temp;
output reg dma_we;
output reg [3*`MAT_MUL_SIZE-1:0] matmul_masks;
input dma_busy;

wire [WIDTH-1:0] _a_readdataout;

reg mux_sel,next_mux_sel;

always@(posedge clk)begin
  if(!resetn)
    mux_sel <= 1'b0;
  else begin
    if(a_en)
      mux_sel <= next_mux_sel;
  end       
end


always @* begin
  num_bytes[1:0] = 2'h0;
  next_mux_sel = 1'b0;
  if(dma_busy)begin
    if(a_reg == 27)
      next_mux_sel = 1'b1;
    else
      next_mux_sel = 1'b0;
  end
  
  if(mux_sel)
    a_readdataout = dma_busy;
  else
    a_readdataout = _a_readdataout;
end

`ifdef USE_INHOUSE_LOGIC
        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(a_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(_a_readdataout)
      );
      defparam
            reg_file1.AWIDTH=LOG2NUMREGS,
            reg_file1.NUM_WORDS=NUMREGS,
            reg_file1.DWIDTH=WIDTH;
    `ifdef TEST_BENCH
                initial begin
		    $readmemh("vregfile_control.dat",reg_file1.dpram1.ram,'h0);
                end
     `endif 
 `else
  //amana: Somehow this was causing an error with VTR.
  //Since we don't need this anyway, commenting it out.
	//altsyncram	reg_file1(
	//			.wren_a (c_we),
	//			.clock0 (clk),
	//			.address_a (c_reg[LOG2NUMREGS-1:0]),
	//			.data_a (c_writedatain),
  //       .rden_b(a_en),
	//			.address_b (a_reg[LOG2NUMREGS-1:0]),
	//			.q_b (a_readdataout)
  //       // synopsys translate_off
  //       ,
  //       .aclr0 (1'b0),
  //       .aclr1 (1'b0),
  //       .byteena_a (1'b1),
  //       .byteena_b (1'b1),
  //       .data_b (32'b11111111),
  //       .wren_b (1'b0),
  //       .clock1 (1'b0),
  //       .clocken1 (1'b0),
  //       .q_a (),
  //       .clocken0 (1'b1),
  //       .addressstall_a (1'b0),
  //       .addressstall_b (1'b0)
  //       // synopsys translate_on
  //   );
	//defparam
  //   `ifdef TEST_BENCH
	//	reg_file1.init_file = "vregfile_control.dat",
  //   `else
	//	reg_file1.init_file = "vregfile_control.mif",
  //   `endif
	//	reg_file1.operation_mode = "DUAL_PORT",
	//	reg_file1.width_a = WIDTH,
	//	reg_file1.widthad_a = LOG2NUMREGS,
	//	reg_file1.numwords_a = NUMREGS,
	//	reg_file1.width_b = WIDTH,
	//	reg_file1.widthad_b = LOG2NUMREGS,
	//	reg_file1.numwords_b = NUMREGS,
	//	reg_file1.lpm_type = "altsyncram",
	//	reg_file1.width_byteena_a = 1,
	//	reg_file1.outdata_reg_b = "UNREGISTERED",
	//	reg_file1.indata_aclr_a = "NONE",
	//	reg_file1.wrcontrol_aclr_a = "NONE",
	//	reg_file1.address_aclr_a = "NONE",
	//	reg_file1.rdcontrol_reg_b = "CLOCK0",
	//	reg_file1.address_reg_b = "CLOCK0",
	//	reg_file1.address_aclr_b = "NONE",
	//	reg_file1.outdata_aclr_b = "NONE",
	//	reg_file1.read_during_write_mode_mixed_ports = "OLD_DATA",
	//	reg_file1.ram_block_type = "AUTO",
	//	reg_file1.intended_device_family = "Stratix";
  `endif

  always@(posedge clk) begin
    if (!resetn) begin
      vl<=0;
      matmul_masks<=32'hffffffff;
      dma_en <= 0;
      mem_addr <= 0;
      num_bytes[WIDTH-1:2] <= 0;
      lane_addr <= 0;
      dma_we <= 0;
      temp <= 0;
    end
    else begin
      if (c_we) begin
        if (c_reg==0) begin
          vl<=c_writedatain;
        end 
        else if (c_reg==31) begin //a_rows
          matmul_masks[1*`MAT_MUL_SIZE-1:0*`MAT_MUL_SIZE] <= c_writedatain[`MAT_MUL_SIZE-1:0];
          matmul_masks[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE] <= c_writedatain[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE];
          matmul_masks[3*`MAT_MUL_SIZE-1:2*`MAT_MUL_SIZE] <= c_writedatain[3*`MAT_MUL_SIZE-1:2*`MAT_MUL_SIZE];
        end
        else if (c_reg==30) begin 
          dma_en <= c_writedatain[0];
          dma_we <= c_writedatain[1];
          num_bytes[WIDTH-1:2] <= c_writedatain[WIDTH-1:2];
        end
        else if (c_reg==29) begin // Address to the main memory
          mem_addr<=c_writedatain;
        end
        else if (c_reg==28) begin
          lane_addr<=c_writedatain;
        end 
        else if (c_reg==20) begin
          temp<=c_writedatain;
        end 
      end  
    end
  end

endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vregfile_control.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vregfile_flag.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vregfile_flag.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_flag
  (clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_we);

parameter NUMBANKS=1;
parameter LOG2NUMBANKS=0;
parameter WIDTH=32;
parameter NUMREGS=32;
parameter LOG2NUMREGS=5;

parameter NUMREGSPERBANK=NUMREGS/NUMBANKS;
parameter LOG2NUMREGSPERBANK=LOG2NUMREGS-LOG2NUMBANKS;

input clk;
input resetn;

input [NUMBANKS-1:0] a_en;
input [NUMBANKS-1:0] b_en;
input [NUMBANKS-1:0] c_we;

input [NUMBANKS*LOG2NUMREGSPERBANK-1:0] a_reg,b_reg,c_reg;
output [NUMBANKS*WIDTH-1:0] a_readdataout, b_readdataout;
input [NUMBANKS*WIDTH-1:0] c_writedatain;

  genvar k;

  generate
  for (k=0; k<NUMBANKS; k=k+1)
  begin : bank_gen
`ifdef  USE_INHOUSE_LOGIC

        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en[k]),
	    .address_a(c_reg[k* LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
	    .address_b(a_reg[k* LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
	    .wren_a(c_we[k]),
	    .wren_b(1'b0),
	    .data_a(c_writedatain[ k*WIDTH +: WIDTH]),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout[k*WIDTH +: WIDTH])
        );
        defparam
            reg_file1.AWIDTH=LOG2NUMREGSPERBANK,
            reg_file1.NUM_WORDS=NUMREGSPERBANK,
            reg_file1.DWIDTH=WIDTH;

        ram_wrapper reg_file2(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(b_en[k]),
	    .address_a(c_reg[k* LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
	    .address_b(b_reg[k* LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
	    .wren_a(c_we[k]),
	    .wren_b(1'b0),
	    .data_a(c_writedatain[ k*WIDTH +: WIDTH]),
	    .data_b(0),
	    .out_a(),
	    .out_b(b_readdataout[k*WIDTH +: WIDTH])
        );
        defparam
            reg_file2.AWIDTH=LOG2NUMREGSPERBANK,
            reg_file2.NUM_WORDS=NUMREGSPERBANK,
            reg_file2.DWIDTH=WIDTH;

`else

    altsyncram	reg_file1(
          .clock0 (clk),
          .wren_a (c_we[k]),
          .data_a (c_writedatain[k*WIDTH +: WIDTH]),
          .byteena_a (1'b1),
          .address_a (c_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .rden_b(a_en[k]),
          .address_b (a_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .q_b (a_readdataout[k*WIDTH +: WIDTH])
          // synopsys translate_off
          ,
          .aclr0 (1'b0),
          .aclr1 (1'b0),
          .byteena_b (1'b1),
          .data_b (),
          .wren_b (1'b0),
          .clock1 (1'b0),
          .clocken1 (1'b0),
          .q_a (),
          .clocken0 (1'b1),
          .addressstall_a (1'b0),
          .addressstall_b (1'b0)
          // synopsys translate_on
      );
    defparam
      reg_file1.operation_mode = "DUAL_PORT",
      reg_file1.width_a = WIDTH,
      reg_file1.widthad_a = LOG2NUMREGSPERBANK,
      reg_file1.numwords_a = NUMREGSPERBANK,
      reg_file1.width_b = WIDTH,
      reg_file1.widthad_b = LOG2NUMREGSPERBANK,
      reg_file1.numwords_b = NUMREGSPERBANK,
      reg_file1.lpm_type = "altsyncram",
      reg_file1.width_byteena_a = 1,
      reg_file1.outdata_reg_b = "UNREGISTERED",
      reg_file1.indata_aclr_a = "NONE",
      reg_file1.wrcontrol_aclr_a = "NONE",
      reg_file1.address_aclr_a = "NONE",
      reg_file1.rdcontrol_reg_b = "CLOCK0",
      reg_file1.address_reg_b = "CLOCK0",
      reg_file1.address_aclr_b = "NONE",
      reg_file1.outdata_aclr_b = "NONE",
      reg_file1.read_during_write_mode_mixed_ports = "OLD_DATA",
      reg_file1.ram_block_type = "AUTO",
      reg_file1.intended_device_family = "Stratix";

      //Reg file duplicated to avoid contention between 2 read
      //and 1 write
    altsyncram	reg_file2(
          .clock0 (clk),
          .wren_a (c_we[k]),
          .data_a (c_writedatain[k*WIDTH +: WIDTH]),
          .byteena_a (1'b1),
          .address_a (c_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .rden_b(b_en[k]),
          .address_b (b_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .q_b (b_readdataout[k*WIDTH +: WIDTH])
          // synopsys translate_off
          ,
          .aclr0 (1'b0),
          .aclr1 (1'b0),
          .byteena_b (1'b1),
          .data_b (),
          .wren_b (1'b0),
          .clock1 (1'b0),
          .clocken1 (1'b0),
          .q_a (),
          .clocken0 (1'b1),
          .addressstall_a (1'b0),
          .addressstall_b (1'b0)
          // synopsys translate_on
      );
    defparam
      reg_file2.operation_mode = "DUAL_PORT",
      reg_file2.width_a = WIDTH,
      reg_file2.widthad_a = LOG2NUMREGSPERBANK,
      reg_file2.numwords_a = NUMREGSPERBANK,
      reg_file2.width_b = WIDTH,
      reg_file2.widthad_b = LOG2NUMREGSPERBANK,
      reg_file2.numwords_b = NUMREGSPERBANK,
      reg_file2.lpm_type = "altsyncram",
      reg_file2.width_byteena_a = 1,
      reg_file2.outdata_reg_b = "UNREGISTERED",
      reg_file2.indata_aclr_a = "NONE",
      reg_file2.wrcontrol_aclr_a = "NONE",
      reg_file2.address_aclr_a = "NONE",
      reg_file2.rdcontrol_reg_b = "CLOCK0",
      reg_file2.address_reg_b = "CLOCK0",
      reg_file2.address_aclr_b = "NONE",
      reg_file2.outdata_aclr_b = "NONE",
      reg_file2.read_during_write_mode_mixed_ports = "OLD_DATA",
      reg_file2.ram_block_type = "AUTO",
      reg_file2.intended_device_family = "Stratix";
  `endif
  end
  endgenerate

endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vregfile_flag.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vregfile_inc.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vregfile_inc.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          inc Register File

   - Has one read port (a) and one write port (c)
   - inc0 fixed to 0 and unwriteable
****************************************************************************/
module vregfile_inc (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

parameter WIDTH=32;
parameter NUMREGS=8;
parameter LOG2NUMREGS=3;

input clk;
input resetn;

input a_en;
input [LOG2NUMREGS-1:0] a_reg,c_reg;
output [WIDTH-1:0] a_readdataout;
input [WIDTH-1:0] c_writedatain;
input c_we;

`ifdef USE_INHOUSE_LOGIC
        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(a_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we&(|c_reg)),
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

`else
	altsyncram	reg_file1(
				.wren_a (c_we&(|c_reg)),
				.clock0 (clk),
				.address_a (c_reg[LOG2NUMREGS-1:0]),
				.data_a (c_writedatain),
        .rden_b(a_en),
				.address_b (a_reg[LOG2NUMREGS-1:0]),
				.q_b (a_readdataout)
        // synopsys translate_off
        ,
        .aclr0 (1'b0),
        .aclr1 (1'b0),
        .byteena_a (1'b1),
        .byteena_b (1'b1),
        .data_b (32'b11111111),
        .wren_b (1'b0),
        .clock1 (1'b0),
        .clocken1 (1'b0),
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
		reg_file1.rdcontrol_reg_b = "CLOCK0",
		reg_file1.address_reg_b = "CLOCK0",
		reg_file1.address_aclr_b = "NONE",
		reg_file1.outdata_aclr_b = "NONE",
		reg_file1.read_during_write_mode_mixed_ports = "OLD_DATA",
		reg_file1.ram_block_type = "AUTO",
		reg_file1.intended_device_family = "Stratix";
`endif
endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vregfile_inc.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vregfile_scalar.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vregfile_scalar.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Scalar Register File

   - Has one read port (a) and one write port (c)
   - vs0 fixed to 0 and unwriteable
****************************************************************************/
module vregfile_scalar (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

parameter WIDTH=32;
parameter NUMREGS=32;
parameter LOG2NUMREGS=5;

input clk;
input resetn;

input a_en;
input [LOG2NUMREGS-1:0] a_reg,c_reg;
output [WIDTH-1:0] a_readdataout;
input [WIDTH-1:0] c_writedatain;
input c_we;

`ifdef USE_INHOUSE_LOGIC
        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(a_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we & (|c_reg)),
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
 `else

	altsyncram	reg_file1(
				.wren_a (c_we&(|c_reg)),
				.clock0 (clk),
				.address_a (c_reg[LOG2NUMREGS-1:0]),
				.data_a (c_writedatain),
        .rden_b(a_en),
				.address_b (a_reg[LOG2NUMREGS-1:0]),
				.q_b (a_readdataout)
        // synopsys translate_off
        ,
        .aclr0 (1'b0),
        .aclr1 (1'b0),
        .byteena_a (1'b1),
        .byteena_b (1'b1),
        .data_b (32'b11111111),
        .wren_b (1'b0),
        .clock1 (1'b0),
        .clocken1 (1'b0),
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
		reg_file1.rdcontrol_reg_b = "CLOCK0",
		reg_file1.address_reg_b = "CLOCK0",
		reg_file1.address_aclr_b = "NONE",
		reg_file1.outdata_aclr_b = "NONE",
		reg_file1.read_during_write_mode_mixed_ports = "OLD_DATA",
		reg_file1.ram_block_type = "AUTO",
		reg_file1.intended_device_family = "Stratix";
 `endif
endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vregfile_scalar.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vregfile_stride.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vregfile_stride.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Stride Register File

   - Has one read port (a) and one write port (c)
****************************************************************************/
module vregfile_stride (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

parameter WIDTH=32;
parameter NUMREGS=8;
parameter LOG2NUMREGS=3;

input clk;
input resetn;

input a_en;
input [LOG2NUMREGS-1:0] a_reg,c_reg;
output [WIDTH-1:0] a_readdataout;
input [WIDTH-1:0] c_writedatain;
input c_we;

`ifdef USE_INHOUSE_LOGIC
        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(a_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we),
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
 `else
	altsyncram	reg_file1(
				.wren_a (c_we),
				.clock0 (clk),
				.address_a (c_reg[LOG2NUMREGS-1:0]),
				.data_a (c_writedatain),
        .rden_b(a_en),
				.address_b (a_reg[LOG2NUMREGS-1:0]),
				.q_b (a_readdataout)
        // synopsys translate_off
        ,
        .aclr0 (1'b0),
        .aclr1 (1'b0),
        .byteena_a (1'b1),
        .byteena_b (1'b1),
        .data_b (32'b11111111),
        .wren_b (1'b0),
        .clock1 (1'b0),
        .clocken1 (1'b0),
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
		reg_file1.rdcontrol_reg_b = "CLOCK0",
		reg_file1.address_reg_b = "CLOCK0",
		reg_file1.address_aclr_b = "NONE",
		reg_file1.outdata_aclr_b = "NONE",
		reg_file1.read_during_write_mode_mixed_ports = "OLD_DATA",
		reg_file1.ram_block_type = "AUTO",
		reg_file1.intended_device_family = "Stratix";
`endif
endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vregfile_stride.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vregfile_vector.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vregfile_vector.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_vector(clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_byteen, c_we);

parameter NUMBANKS=1;
parameter LOG2NUMBANKS=0;
parameter WIDTH=32;
parameter NUMREGS=32;
parameter LOG2NUMREGS=5;

parameter NUMREGSPERBANK=NUMREGS/NUMBANKS;
parameter LOG2NUMREGSPERBANK=LOG2NUMREGS-LOG2NUMBANKS;

input clk;
input resetn;

input [NUMBANKS-1:0] a_en;
input [NUMBANKS-1:0] b_en;
input [NUMBANKS*LOG2NUMREGSPERBANK-1:0] a_reg,b_reg,c_reg;
output [NUMBANKS*WIDTH-1:0] a_readdataout, b_readdataout;
input [NUMBANKS*WIDTH-1:0] c_writedatain;
input [((WIDTH>=8) ? NUMBANKS*WIDTH/8-1 : NUMBANKS-1):0] c_byteen;
input [NUMBANKS-1:0] c_we;

  genvar k;

  generate
  for (k=0; k<NUMBANKS; k=k+1)
  begin : bank_gen
  `ifdef USE_INHOUSE_LOGIC
          ram_wrapper reg_file1(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
  	    .address_a(c_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
  	    .address_b(a_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
  	    .wren_a(c_we[k] & ((WIDTH>8) ? 1'b1 : c_byteen[k])),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[k*WIDTH +: WIDTH]),
  	    .data_b(0),
  	    .out_a(),
  	    .out_b(a_readdataout[k*WIDTH +: WIDTH])
          );
          defparam
              reg_file1.AWIDTH=LOG2NUMREGSPERBANK,
              reg_file1.NUM_WORDS=NUMREGSPERBANK,
              reg_file1.DWIDTH=WIDTH;
  
          ram_wrapper reg_file2(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(b_en),
  	    .address_a(c_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
  	    .address_b(b_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
  	    .wren_a(c_we[k] & ((WIDTH>8) ? 1'b1 : c_byteen[k])),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[k*WIDTH +: WIDTH]),
  	    .data_b(0),
  	    .out_a(),
  	    .out_b(b_readdataout[k*WIDTH +: WIDTH])
          );
          defparam
              reg_file2.AWIDTH=LOG2NUMREGSPERBANK,
              reg_file2.NUM_WORDS=NUMREGSPERBANK,
              reg_file2.DWIDTH=WIDTH;
  `else
    altsyncram	reg_file1(
          .clock0 (clk),
          .wren_a (c_we[k] & ((WIDTH>8) ? 1'b1 : c_byteen[k])),
          .data_a (c_writedatain[k*WIDTH +: WIDTH]),
          //Not allowed to use this port when width_byteena_a is <= 1
          .byteena_a ( (WIDTH>8) ? c_byteen[k*WIDTH/8 +: WIDTH/8] : 1'b1 ),
          .rden_b (a_en[k]),
          .address_a (c_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .address_b (a_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .q_b (a_readdataout[k*WIDTH +: WIDTH])
          // synopsys translate_off
          ,
          .clock1 (1'b0),
          .aclr0 (1'b0),
          .aclr1 (1'b0),
          .byteena_b (1'b1),
          .data_b (),
          .wren_b (1'b0),
          .clocken1(1'b1),
          .q_a (),
          .clocken0 (1'b1),
          .addressstall_a (1'b0),
          .addressstall_b (1'b0)
          // synopsys translate_on
      );
    defparam
      reg_file1.operation_mode = "DUAL_PORT",
      reg_file1.width_a = WIDTH,
      reg_file1.widthad_a = LOG2NUMREGSPERBANK,
      reg_file1.numwords_a = NUMREGSPERBANK,
      reg_file1.width_b = WIDTH,
      reg_file1.widthad_b = LOG2NUMREGSPERBANK,
      reg_file1.numwords_b = NUMREGSPERBANK,
      reg_file1.lpm_type = "altsyncram",
      reg_file1.width_byteena_a = (WIDTH>=8) ? WIDTH/8 : 1,
      reg_file1.outdata_reg_b = "UNREGISTERED",
      reg_file1.indata_aclr_a = "NONE",
      reg_file1.wrcontrol_aclr_a = "NONE",
      reg_file1.address_aclr_a = "NONE",
      reg_file1.rdcontrol_reg_b = "CLOCK0",
      reg_file1.address_reg_b = "CLOCK0",
      reg_file1.address_aclr_b = "NONE",
      reg_file1.outdata_aclr_b = "NONE",
      reg_file1.read_during_write_mode_mixed_ports = "OLD_DATA",
      reg_file1.ram_block_type = "AUTO",
      reg_file1.intended_device_family = "Stratix";

      //Reg file duplicated to avoid contention between 2 read
      //and 1 write
    altsyncram	reg_file2(
          .clock0 (clk),
          .wren_a (c_we[k] & ((WIDTH>8) ? 1'b1 : c_byteen[k]) ),
          .data_a (c_writedatain[k*WIDTH +: WIDTH]),
          //Not allowed to use this port when width_byteena_a is <= 1
          .byteena_a ( (WIDTH>8) ? c_byteen[k*WIDTH/8 +: WIDTH/8] : 1'b1 ),
          .address_a (c_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .rden_b (b_en[k]),
          .address_b (b_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .q_b (b_readdataout[k*WIDTH +: WIDTH])
          // synopsys translate_off
          ,
          .clock1 (1'b0),
          .aclr0 (1'b0),
          .aclr1 (1'b0),
          .byteena_b (1'b1),
          .data_b (),
          .clocken1(1'b1),
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
      reg_file2.widthad_a = LOG2NUMREGSPERBANK,
      reg_file2.numwords_a = NUMREGSPERBANK,
      reg_file2.width_b = WIDTH,
      reg_file2.widthad_b = LOG2NUMREGSPERBANK,
      reg_file2.numwords_b = NUMREGSPERBANK,
      reg_file2.lpm_type = "altsyncram",
      reg_file2.width_byteena_a = (WIDTH>=8) ? WIDTH/8 : 1,
      reg_file2.outdata_reg_b = "UNREGISTERED",
      reg_file2.indata_aclr_a = "NONE",
      reg_file2.wrcontrol_aclr_a = "NONE",
      reg_file2.address_aclr_a = "NONE",
      reg_file2.rdcontrol_reg_b = "CLOCK0",
      reg_file2.address_reg_b = "CLOCK0",
      reg_file2.address_aclr_b = "NONE",
      reg_file2.outdata_aclr_b = "NONE",
      reg_file2.read_during_write_mode_mixed_ports = "OLD_DATA",
      reg_file2.ram_block_type = "AUTO",
      reg_file2.intended_device_family = "Stratix";
  `endif
  end
  endgenerate

endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vregfile_vector.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vcomponents.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vcomponents.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );
parameter WIDTH=32;
parameter DEPTH=1;
parameter RESETVALUE=0;

input [WIDTH-1:0]  d;
input              clk;
input              resetn;
input  [DEPTH-1:0] en;
input  [DEPTH-1:0] squash;
output [WIDTH*(DEPTH+1)-1:0] q;

reg [WIDTH*DEPTH-1:0] tq;
integer i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ WIDTH-1:0 ]<=RESETVALUE;
    else if (en[0])
      tq[ WIDTH-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<DEPTH; i=i+1)
      if (!resetn || squash[i] )
        tq[i*WIDTH +: WIDTH ]<=RESETVALUE;
      else if (en[i])
        tq[i*WIDTH +: WIDTH ]<=tq[(i-1)*WIDTH +: WIDTH ];
  end

  assign q[WIDTH-1:0]=d;
  assign q[WIDTH*(DEPTH+1)-1:WIDTH]=tq;
endmodule

/****************************************************************************
          Hazard checker
 if mode==0 compare entire width of src & dests,
 if mode==1 compare upper WIDTH-SUBWIDTH bits of src & dests
****************************************************************************/
module hazardchecker (
    src,
    src_valid,
    dst,
    dst_valid,
    dst_mode,
    lt_dst,
    lt_dst_valid,
    lt_mode,
    haz
    );
parameter WIDTH=10;
parameter SUBWIDTH=5;
parameter DEPTH=1;
parameter LTDEPTH=1;

input [WIDTH-1:0]  src;
input              src_valid;
input [WIDTH*DEPTH-1:0]  dst;
input [DEPTH-1:0]    dst_valid;
input [DEPTH-1:0]  dst_mode;
input [WIDTH*LTDEPTH-1:0]  lt_dst;
input [LTDEPTH-1:0]    lt_dst_valid;
input [LTDEPTH-1:0]  lt_mode;
output             haz;

reg             t_haz;
reg             t_haz_lt;

integer i,j;

  always@*
  begin
    t_haz=1'b0;
    t_haz_lt=1'b0;

    for (i=0; i<DEPTH; i=i+1)
      if (dst_mode)
        t_haz=t_haz | ((src[WIDTH-1:SUBWIDTH]==dst[i*WIDTH+SUBWIDTH +: WIDTH-SUBWIDTH ])&dst_valid[i]&src_valid);
      else
        t_haz=t_haz | ((src==dst[i*WIDTH +: WIDTH ])&dst_valid[i]&src_valid);

    //Check if src is less than dest - used for hazards in issuer
    for (j=0; j<LTDEPTH; j=j+1)
      if (lt_mode)
        t_haz_lt=t_haz_lt | ((src[WIDTH-1:SUBWIDTH]==lt_dst[j*WIDTH+SUBWIDTH +: WIDTH-SUBWIDTH ])&lt_dst_valid[j]&src_valid);
      else
        t_haz_lt=t_haz_lt | (((src[WIDTH-1:SUBWIDTH]==lt_dst[j*WIDTH+SUBWIDTH +: WIDTH-SUBWIDTH ])&lt_dst_valid[j]&src_valid) && (src[SUBWIDTH-1:0]>=lt_dst[j*WIDTH +: SUBWIDTH]));
  end

  assign haz=(t_haz|t_haz_lt)&src_valid;

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vcomponents.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vlanes.v.temp
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/vtr/vlanes.v.temp
//////////////////////////////////////////////////////////////////////

/******************************************************************************
 * Vector Pipeline
 *
 *                vl (arrives from vector control pipeline in Stage2)
 *                vs
 *                vc
 *|   Stage1  |  Stage2   |  Stage3  |  Stage4  |  Stage5  | Stage6  |  Stage7 |
 *| instr     |           |          | dispatch |          |         |
 *| decode    | control   | bank     | regfile  | execute  | WB
 *
 *****************************************************************************/

//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vdispatcher.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vdispatcher.v
//////////////////////////////////////////////////////////////////////


module vdispatcher(
    clk,
    resetn,

    shift,
    rotate, //shift must also be high to rotate

    inshift_instr,
    inshift_first,
    inshift_rdelm,
    inshift_wrelm,
    inshift_count,

    increment,
    rdelm_add_sub,
    wrelm_add_sub,
    count_add_sub,
    rdelm_valuetoadd,
    wrelm_valuetoadd,
    count_valuetoadd,

    instr,
    first,
    rdelm,
    wrelm,
    count

    );

parameter NUMENTRIES=4;      
parameter WIDTHINSTR=32;
parameter WIDTHRDELM=32;
parameter WIDTHWRELM=32;
parameter WIDTHCOUNT=32;

input clk;
input resetn;

input  shift;
input  rotate;

input [ WIDTHINSTR-1:0 ] inshift_instr;
input                    inshift_first;
input [ WIDTHRDELM-1:0 ] inshift_rdelm;
input [ WIDTHRDELM-1:0 ] inshift_wrelm;
input [ WIDTHCOUNT-1:0 ] inshift_count;

input [ NUMENTRIES-1:0 ] increment;

input rdelm_add_sub;
input wrelm_add_sub;
input count_add_sub;
input [ WIDTHRDELM-1:0 ] rdelm_valuetoadd;
input [ WIDTHWRELM-1:0 ] wrelm_valuetoadd;
input [ WIDTHCOUNT-1:0 ] count_valuetoadd;

output [ NUMENTRIES*WIDTHINSTR-1:0 ] instr;
output            [ NUMENTRIES-1:0 ] first;
output [ NUMENTRIES*WIDTHRDELM-1:0 ] rdelm;
output [ NUMENTRIES*WIDTHWRELM-1:0 ] wrelm;
output [ NUMENTRIES*WIDTHCOUNT-1:0 ] count;

wire [NUMENTRIES*WIDTHINSTR-1:0 ] inparallel_data_inst_NC;
vdispatcher_shift #(NUMENTRIES,WIDTHINSTR) vdispatcher_instr (
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .squash({NUMENTRIES{1'b0}}),
      .inshift_data(inshift_instr),
      .inparallel_data(inparallel_data_inst_NC),
      .outparallel_data(instr));

wire [ NUMENTRIES-1:0 ] inparallel_data_first_NC;
vdispatcher_shift #(NUMENTRIES,1) vdispatcher_first (
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .squash(increment&~{32'b0,shift&~rotate}),
      .inshift_data(inshift_first),
      .inparallel_data(inparallel_data_first_NC),
      .outparallel_data(first));

wire [ NUMENTRIES*WIDTHRDELM-1:0 ] inparallel_data_rdelm_NC;
wire [ NUMENTRIES-1:0 ] squash_rdelm_NC;
vdispatcher_add #(NUMENTRIES,WIDTHRDELM) vdispatcher_rdelm(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_rdelm_NC),
      .add_sub(rdelm_add_sub),
      .valuetoadd(rdelm_valuetoadd),
      .inshift_data(inshift_rdelm),
      .inparallel_data(inparallel_data_rdelm_NC),
      .outparallel_data(rdelm));

wire [ NUMENTRIES*WIDTHWRELM-1:0 ] inparallel_data_wrelm_NC;
wire [ NUMENTRIES-1:0 ] squash_wrelm_NC;
vdispatcher_add #(NUMENTRIES,WIDTHWRELM) vdispatcher_wrelm(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_wrelm_NC),
      .add_sub(wrelm_add_sub),
      .valuetoadd(wrelm_valuetoadd),
      .inshift_data(inshift_wrelm),
      .inparallel_data(inparallel_data_wrelm_NC),
      .outparallel_data(wrelm));

wire [ NUMENTRIES*WIDTHCOUNT-1:0 ] inparallel_data_count_NC;
wire [ NUMENTRIES-1:0 ] squash_count_NC;
vdispatcher_add #(NUMENTRIES,WIDTHCOUNT) vdispatcher_count(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_count_NC),
      .add_sub(count_add_sub),
      .valuetoadd(count_valuetoadd),
      .inshift_data(inshift_count),
      .inparallel_data(inparallel_data_count_NC),
      .outparallel_data(count));


endmodule



module vdispatcher_shift (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    squash,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

parameter NUMENTRIES=4;      
parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ NUMENTRIES-1:0 ]  squash;

input [ WIDTH-1:0 ] inshift_data;

input [ NUMENTRIES*WIDTH-1:0 ]  inparallel_data;
output [ NUMENTRIES*WIDTH-1:0 ] outparallel_data;

wire [ WIDTH-1:0 ]  shiftin_left;
wire [ WIDTH-1:0 ]  shiftin_right;


  assign shiftin_right = (!rotate) ? inshift_data : 
                      outparallel_data[NUMENTRIES*WIDTH-1:(NUMENTRIES-1)*WIDTH];

  assign shiftin_left = (!rotate) ? 0 : outparallel_data[WIDTH-1:0];

  velmshifter #(NUMENTRIES,WIDTH) velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load),
      .shift(shift),
      .dir_left(1),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe(inparallel_data),
      .outpipe(outparallel_data));


endmodule



module vdispatcher_add (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    increment,
    squash,

    add_sub,
    valuetoadd,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

parameter NUMENTRIES=4;      
parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ NUMENTRIES-1:0 ]  increment;
input [ NUMENTRIES-1:0 ]  squash;

input               add_sub;
input [ WIDTH-1:0 ] valuetoadd;

input [ WIDTH-1:0 ] inshift_data;

input [ NUMENTRIES*WIDTH-1:0 ]  inparallel_data;
output [ NUMENTRIES*WIDTH-1:0 ] outparallel_data;

wire [ WIDTH-1:0 ]  shiftin_right;

reg [ NUMENTRIES*WIDTH-1:0 ] outparallel_data;
reg [ NUMENTRIES*WIDTH-1:0 ] outparallel_added;


  assign shiftin_right = (!rotate) ? inshift_data : 
                    outparallel_added[NUMENTRIES*WIDTH-1:(NUMENTRIES-1)*WIDTH];

  reg [ WIDTH-1:0 ] v;
  integer i;
  always@*
    for (i=0; i<NUMENTRIES; i=i+1)
    begin
      //Saturate at 0xffff... in either direction
      v=(increment[i] && outparallel_data[i*WIDTH +: WIDTH] != {WIDTH{1'b1}}) ?
        valuetoadd : 0;
      if (add_sub==0)
        outparallel_added[i*WIDTH +: WIDTH]=outparallel_data[i*WIDTH+:WIDTH] + v;
      else
        outparallel_added[i*WIDTH +: WIDTH]=outparallel_data[i*WIDTH+:WIDTH] - v;
    end

  always@(posedge clk)
    if (!resetn)
      outparallel_data=0;
    else if (load)
      outparallel_data=inparallel_data;
    else if (shift)
      outparallel_data=(outparallel_added<<WIDTH) | shiftin_right;
      


endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vdispatcher.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vlane_alu.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vlane_alu.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************

  ALU Circuit:

         |    _     flags
 src1 ---0---| \    |
             >+|----+----|Sat|------|M\
 src1 --|M\--|-/         |sum|      |U|---
        |U|                       +-|X/
 src2 --|X/                       |    1
                                  |
      src2 ------------|M\        |
 src1 ----------|&|    |U|--|Sat|-+
                |||----|X/  |siz|
 src2 ---0------|^|      0
         |


              ADD SUB   CMP    CMP.U & | ^ ~| MIN MAX ABS MRG
              S U S U = ! < <= < <=           S U S U

adderopA_sel  0 0 0 0 0 0 0 0  0 0   x x x x  0 0 0 0  1   1
adderopB_sel  0 0 0 0 0 0 0 0  0 0   x x x x  0 0 0 0  1   0
logicopB_sel  x x x x x x x x  x x   0 0 0 0  1 1 1 1  1   1
signed        1 0 1 0 1 1 1 1  0 0   x x x x  1 0 1 0  1   x
addsub        0 0 1 1 1 1 1 1  1 1   x x x x  1 1 1 1  1   0
max           x x x x x x x x  x x   0 0 0 0  0 0 1 1  0   0
min           x x x x x x x x  x x   0 0 0 0  1 1 0 0  0   0
logic[1:0]    x x x x x x x x  x x   1 0 2 3  0 0 0 0  0   0
mux1_sel[1:0] 0 0 0 0 x x x x  x x   1 1 1 1  1 1 1 1  2   3 -- Combine
flag_sel[1:0] x x x x 0 1 2 3  2 3   x x x x  x x x x  x   x /

ALUOP_ZERO    =11'bzz1zzzz0101
ALUOP_ADD     =11'b00z10zzzz00
ALUOP_ADDU    =11'b00z00zzzz00
ALUOP_SUB     =11'b00z11zzzz00
ALUOP_SUBU    =11'b00z01zzzz00
ALUOP_CMP_EQ  =11'b00z11zzzz00
ALUOP_CMP_NEQ =11'b00z11zzzz01
ALUOP_CMP_LT  =11'b00z11zzzz10
ALUOP_CMP_LE  =11'b00z11zzzz11
ALUOP_CMP_LT_U=11'b00z01zzzz10
ALUOP_CMP_LE_U=11'b00z01zzzz11
ALUOP_AND     =11'bzz0zz000101
ALUOP_OR      =11'bzz0zz000001
ALUOP_XOR     =11'bzz0zz001001
ALUOP_NOR     =11'bzz0zz001101
ALUOP_MIN     =11'b00111010001
ALUOP_MIN_U   =11'b00101010001
ALUOP_MAX     =11'b00111100001
ALUOP_MAX_U   =11'b00101100001
ALUOP_ABS     =11'b11111000010
ALUOP_MERGE   =11'b101z0000011

****************************************************************************/

//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vlane_saturate.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vlane_saturate.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Saturate unit

  Takes a signed input and saturates it

  sat sign
   0    x   NOP (pass through)
   1    1   VS[ADD/SUB]
   1    0   VS[ADD/SUB]_U

parameter
  SATSUMOP_NOP=2'b00,
  SATSUMOP_VS =2'b11,
  SATSUMOP_VSU=2'b10;

****************************************************************************/


module vlane_saturatesum(
    in,
    op,
    out
    );

parameter WIDTH=32;

input [WIDTH+2-1:0] in;
input [1:0] op;
output [WIDTH-1:0] out;

reg [WIDTH-1:0] out;

wire op_saturate;
wire op_signed;

  assign op_saturate=op[1];
  assign op_signed=op[0];

wire [WIDTH-1:0] maxunsigned;
wire [WIDTH-1:0] minunsigned;
wire [WIDTH-1:0] maxsigned;
wire [WIDTH-1:0] minsigned;

  assign maxunsigned = {WIDTH{1'b1}};
  assign minunsigned = 0;
  assign maxsigned = {1'b0,{WIDTH-1{1'b1}}};
  assign minsigned = {1'b1,{WIDTH-1{1'b0}}};

wire [WIDTH-1:0] through;

  assign through=in[WIDTH-1:0];

wire [2:0] top3bits=(op_saturate) ? in[WIDTH+2-1:WIDTH-1] : 3'b0 ;

  always@*
    case(top3bits)
      3'b010: out=maxunsigned;
      3'b011: out=maxunsigned;
      3'b001: out=(op_signed) ? maxsigned : through;
      3'b111: out=(op_signed) ? through : minunsigned;
      3'b110: out=(op_signed) ? minsigned : minunsigned;
      default: out=through;
    endcase

endmodule

/****************************************************************************
          Saturate unit for different widths given 32-bit input

  Only works for 32-bit inputs

  Interprets input as signed/unsigned, saturates to signed/unsigned 
  byte/half/word

  NOP (pass through) when size=word and signed==outsigned

   out
   sign sign size
    0    0    00    VSAT_U_W - NOP (pass through)
    0    0    01    VSAT_U_B
    0    0    10    VSAT_U_H
    1    1    00    VSAT_W - NOP (pass through)
    1    1    01    VSAT_B
    1    1    10    VSAT_H
    0    1    00    VSAT_SU_W
    0    1    01    VSAT_SU_B
    0    1    10    VSAT_SU_H

parameter
  SATSIZEOP_VSATUW=4'b0000,
  SATSIZEOP_VSATUB=4'b0001,
  SATSIZEOP_VSATUH=4'b0010,
  SATSIZEOP_VSATW  =4'b1100,
  SATSIZEOP_VSATB  =4'b1101,
  SATSIZEOP_VSATH  =4'b1110,
  SATSIZEOP_VSATSUW=4'b0100,
  SATSIZEOP_VSATSUB=4'b0101,
  SATSIZEOP_VSATSUH=4'b0110;

****************************************************************************/

module vlane_saturatesize(
    in,
    op,
    out
    );

parameter WIDTH=32;

input [WIDTH-1:0] in;
input [3:0] op;
output [WIDTH-1:0] out;

wire op_outsigned;
wire op_signed;
wire op_size;

reg [WIDTH-1:0] out;

  assign op_outsigned=op[3];
  assign op_signed=op[2];
  assign op_size=op[1:0]; //0 - word, 1 - byte, 2 - half

  always@*
    case(op_size)
      2'b01:  //byte
        case({op_signed,op_outsigned})
          2'b11:    // signed
            out = ((in[WIDTH-1])&(!(&in[WIDTH-2:7]))) ? {{WIDTH-8{1'b1}},-128} :
                  ((~in[WIDTH-1]) && (|in[WIDTH-2:7])) ? 127 : in;
          2'b10:    // signed-unsigned
            out = (in[WIDTH-1]) ? 0 :
                  (~in[WIDTH-1]&&(|in[WIDTH-2:8])) ? 255 : in;
          default:  //2'b00: unsigned
            out=(|in[WIDTH-1:8]) ? 255 : in;
         endcase
      2'b10:  //half-word 16-bits
        case({op_signed,op_outsigned})
          2'b11:    // signed
            out=((in[WIDTH-1])&(!(&in[WIDTH-2:15])))? {{WIDTH-16{1'b1}},-32768}:
                  ((~in[WIDTH-1]) && (|in[WIDTH-2:15])) ? 32767 : in;
          2'b10:    // signed-unsigned
            out = (in[WIDTH-1]) ? 0 :
                  (~in[WIDTH-1]&&(|in[WIDTH-2:16])) ? 65535 : in;
          default:  //2'b00: unsigned
            out=(|in[WIDTH-1:16]) ? 65535 : in;
         endcase
      default: 
        case({op_signed,op_outsigned})
          2'b10:    // signed-unsigned
            out = (in[WIDTH-1]) ? 0 : in;
          default:  //2'b00: unsigned
            out=in;
         endcase
    endcase
  
endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vlane_saturate.v
//////////////////////////////////////////////////////////////////////

module vlane_alu(
    clk,
    resetn,

    pipe_en,
    pipe_squashn,

    src1,
    src2,
    mask,

    op,
    satsum_op,
    satsize_op,

    cmp_result,
    result

    );

parameter WIDTH=32;

input clk;
input resetn;

input pipe_en;
input pipe_squashn;

input  [WIDTH-1:0] src1;
input  [WIDTH-1:0] src2;

input  mask;

input  [10:0] op;
input  [1:0] satsum_op;
input  [3:0] satsize_op;

output cmp_result;
output [WIDTH-1:0] result;

wire [WIDTH-1:0] adder_opA;
wire [WIDTH-1:0] adder_opB;
wire [WIDTH+2-1:0] adder_result;

wire [WIDTH-1:0] logic_opA;
wire [WIDTH-1:0] logic_opB;
reg  [WIDTH-1:0] logic_result;

wire [WIDTH-1:0] mux0_result;
wire [WIDTH-1:0] mux1_result;

wire [WIDTH-1:0] satsum_result;
wire [WIDTH-1:0] satsize_result;

wire lt;
wire neq;
wire eq;
wire le;

wire ctrl_adderopA_sel;
wire ctrl_adderopB_sel;
wire ctrl_logicopB_sel;
wire ctrl_signed;
wire ctrl_addsub;
wire ctrl_max;
wire ctrl_min;
wire [1:0] ctrl_logic;
wire [1:0] ctrl_mux1_sel;
wire [1:0] ctrl_flag_sel;

  assign ctrl_adderopA_sel=op[10];
  assign ctrl_adderopB_sel=op[9];
  assign ctrl_logicopB_sel=op[8];
  assign ctrl_signed=op[7];
  assign ctrl_addsub=~op[6];  //Make add 0, sub 1
  assign ctrl_max=op[5];
  assign ctrl_min=op[4];
  assign ctrl_logic=op[3:2];
  assign ctrl_mux1_sel=op[1:0];
  assign ctrl_flag_sel=op[1:0];

  assign adder_opA=(ctrl_adderopA_sel) ? 0 : src1;
  assign adder_opB=(ctrl_adderopB_sel) ? src1 : src2;

  `ifdef USE_INHOUSE_LOGIC
  local_add_sub local_adder_inst(
      .dataa({{2{ctrl_signed&adder_opA[WIDTH-1]}},adder_opA}),
      .datab({{2{ctrl_signed&adder_opB[WIDTH-1]}},adder_opB}),
      .cin(~ctrl_addsub),
      .add_sub(ctrl_addsub),
      .result(adder_result)
  );
  defparam
      local_adder_inst.WIDTH = WIDTH+2,
      local_adder_inst.PIPELINE = 0,
      local_adder_inst.REPRESENTATION = "SIGNED";
  `else
  lpm_add_sub adder_inst(
      .dataa({{2{ctrl_signed&adder_opA[WIDTH-1]}},adder_opA}),
      .datab({{2{ctrl_signed&adder_opB[WIDTH-1]}},adder_opB}),
      .cin(~ctrl_addsub),
      .add_sub(ctrl_addsub),
      .result(adder_result)
          // synopsys translate_off
          , .cout (), .overflow (), .clken (), .clock (), .aclr ()
          // synopsys translate_on
      );
  defparam
      adder_inst.lpm_width=WIDTH+2,
      adder_inst.lpm_pipeline=0,
      adder_inst.lpm_representation="SIGNED";
  `endif

  assign lt=adder_result[WIDTH];
  assign neq=|adder_result;
  assign eq=~neq;
  assign le=lt||eq;

  assign cmp_result= (ctrl_flag_sel==0) ? eq :
                     (ctrl_flag_sel==1) ? neq :
                     (ctrl_flag_sel==2) ? lt : le;

  assign logic_opA=src1;
  assign logic_opB=(ctrl_logicopB_sel) ? 0 : src2;

  always@*
    case(ctrl_logic)
        2'b00:
            logic_result=logic_opA|logic_opB;
        2'b01:
            logic_result=logic_opA&logic_opB;
        2'b10:
            logic_result=logic_opA^logic_opB;
        2'b11:
            logic_result=~(logic_opA|logic_opB);
    endcase

  assign mux0_result=((lt && ctrl_max) || (~lt && ctrl_min)) ?  src2 : 
                                                                logic_result;

  /************* PIPELINE at this point ***************/
  reg [WIDTH-1:0] mux0_result_s2;
  reg [WIDTH+2-1:0] adder_result_s2;
  reg  [3:0] satsize_op_s2;
  reg  [3:0] satsum_op_s2;
  reg ctrl_mux1_sel_s2;

  always@(posedge clk)
  begin
    if (!resetn || !pipe_squashn)
    begin
      mux0_result_s2<=0;
      adder_result_s2<=0;
      satsize_op_s2<=0;
      satsum_op_s2<=0;
      ctrl_mux1_sel_s2<=0;
    end
    else if (pipe_en)
    begin
      mux0_result_s2<=mux0_result;
      adder_result_s2<=adder_result;
      satsize_op_s2<=satsize_op;
      satsum_op_s2<=satsum_op;
      ctrl_mux1_sel_s2<=((ctrl_mux1_sel==0) ||
                         (ctrl_mux1_sel==2 && src1[WIDTH-1]) ||
                         (ctrl_mux1_sel==3 && mask) );
    end
  end

  vlane_saturatesize #(32) satsize(   //Unit only works for 32-bit inputs
      .in(mux0_result_s2),
      .op(satsize_op_s2),
      .out(satsize_result)
      );

  vlane_saturatesum #(WIDTH) satsum(
      .in(adder_result_s2),
      .op(satsum_op_s2),
      .out(satsum_result)
      );

  assign mux1_result= (ctrl_mux1_sel_s2) ? 
                              satsum_result : satsize_result;

  assign result=mux1_result;

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vlane_alu.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vmul_unit.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vmul_unit.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vlane_mulshift.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vlane_mulshift.v
//////////////////////////////////////////////////////////////////////

/****************************************************************************
          MUL unit

Operation table

     half sat  op unsign dir
2      0   0   0   0    0    |  Zero (This value is used to reset multiplier)
14     1   0   1   1    0    |  LMULU Lower MULtiply Unsigned
15     1   0   1   1    1    |  UMULU Upper MULtiply Unsigned
16     1   0   1   0    0    |  LMUL Lower MULtiply signed
17     1   0   1   0    1    |  UMUL Upper MULtiply signed
4      0   0   1   1    0    |  MULLOU
4      0   0   1   1    1    |  MULHIU
6      0   0   1   0    0    |  MULLO
6      0   0   1   0    1    |  MULHI
0      0   0   0   1    0    |  ShiftLeft
8      0   1   0   1    0    |  ShiftLeftSatU
10     0   1   0   0    0    |  ShiftLeftSat
1      0   0   0   1    1    |  ShiftRightLogic
3      0   0   0   0    1    |  ShiftRightArith
****************************************************************************/
module vlane_mulshift(clk, resetn,
            opA, opB, sa,
            op,
            en,
            result
            );
parameter WIDTH=32;
parameter LOG2WIDTH=5;

input clk;
input resetn;

input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
input [LOG2WIDTH-1:0] sa;
input [4:0] op;
input [3:1] en;  //Enable for each pipestage

output [WIDTH-1:0] result;

/********* Control Signals *********/
wire is_signed,dir, is_mul, saturate;
assign is_mul=op[2];      // selects between opB and the computed shift amount
assign is_signed=~op[1];
assign dir=op[0];         // selects between 2^sa and 2^(32-sa) for right shift
assign saturate=op[3];
assign half=op[4];

/********* Circuit Body *********/
wire dum,dum2,dum3;
wire [WIDTH:0] opB_mux_out;
wire [WIDTH-1:0] opA_mux_out;
wire [WIDTH-1:0] opB_mul;
wire [5-1:0] left_sa;     // Amount of left shift required for both left/right
reg [WIDTH:0] decoded_sa;
wire [WIDTH-1:0] hi;
wire [WIDTH-1:0] lo;

assign opA_mux_out = (~half) ? ( opA ) : (WIDTH<2) ? 0 : (dir) ?
              {{WIDTH/2{is_signed&is_mul&opA[WIDTH-1]}},opA[WIDTH-1:WIDTH/2]} :
              {{WIDTH/2{is_signed&is_mul&opA[WIDTH/2-1]}},opA[WIDTH/2-1:0]};

assign opB_mul = (~half) ? opB : (WIDTH<2) ? 0 : (dir) ? 
              {{WIDTH/2{is_signed&is_mul&opB[WIDTH-1]}},opB[WIDTH-1:WIDTH/2]} :
              {{WIDTH/2{is_signed&is_mul&opB[WIDTH/2-1]}},opB[WIDTH/2-1:0]};

assign opB_mux_out=(is_mul) ? {is_signed&opB_mul[WIDTH-1],opB_mul} : decoded_sa;

reg zeroout;
always@(posedge clk)
  if (en[1])
    zeroout<=(op[3:0]==0);
`ifdef USE_INHOUSE_LOGIC
local_mult local_mult_component (
.dataa({is_signed&opA_mux_out[WIDTH-1],opA_mux_out}),
.datab(opB_mux_out),
.clock(clk),
.clken(en[1]),
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
  .dataa ({is_signed&opA_mux_out[WIDTH-1],opA_mux_out}),
  .datab (opB_mux_out),
  .sum(),
  .clock(clk),
  .clken(en[1]),
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
// if A is positive/negative make it maximum/minimum positive/negative
wire [WIDTH-1:0] signedsaturate=
                    (opA_mux_out[WIDTH-1]) ? {1'b1,{WIDTH-1{1'b0}}} : 
                                             {1'b0,{WIDTH-1{1'b1}}};

reg [WIDTH-1:0] saturatedval_s2;
reg saturate_s2;

//Capture saturated value and saturate signal for next stage
always@(posedge clk)
  if (!resetn)
  begin
    saturatedval_s2<=0;
    saturate_s2<=0;
  end
  else if (en[1])
  begin
    saturatedval_s2<=((~is_signed) ? {WIDTH{1'b1}} : signedsaturate);
    saturate_s2<=saturate;
  end

reg sel_hi;

always@(posedge clk)
  if (!resetn)
    sel_hi<=0;
  else if (en[1])
    sel_hi<=(is_mul && dir || ~is_mul && dir && |sa);

assign result =(zeroout) ? 0 : 
                (saturate_s2 && |hi ) ? saturatedval_s2 : 
                (sel_hi) ? hi : lo;


assign {dum3, left_sa}= (dir) ? WIDTH-sa : {1'b0,sa};

integer i;

//Decoder - computes 2^left_sa
always@*
begin
  decoded_sa=0;
  for (i=0; i<WIDTH; i=i+1)
    if (left_sa==i)
      decoded_sa[i]=1'b1;
end


endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vlane_mulshift.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vlane_barrelshifter.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vlane_barrelshifter.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Shifter unit

Opcode Table:

sign_ext dir 
 0        0    |  ShiftLeft
 0        1    |  ShiftRightLogic
 1        1    |  ShiftRightArith
          
****************************************************************************/
module vlane_barrelshifter(clk, resetn,
            opB, sa, 
            op, 
            result);
parameter WIDTH=32;
parameter LOG2WIDTH=5;

//Shifts the first 2 bits in one cycle, the rest in the next cycle
parameter REGISTERBREAK=LOG2WIDTH-2;

input clk;
input resetn;

input [WIDTH-1:0] opB;
input [LOG2WIDTH-1:0] sa;                             // Shift Amount
input [2-1:0] op;

output [WIDTH-1:0] result;


wire sign_ext;
wire shift_direction;
assign sign_ext=op[1];
assign shift_direction=op[0];

wire dum,dum_,dum2;
wire [WIDTH-1:0] partial_result_,partial_result;

`ifdef USE_INHOUSE_LOGIC
 local_shifter local_shifter_inst1(
  .data({sign_ext&opB[WIDTH-1],opB}),
  .distance(sa&(32'hffffffff<<((REGISTERBREAK>0) ? REGISTERBREAK : 0))),
  .direction(shift_direction),
  .result({dum,partial_result})
 );
 defparam
    local_shifter_inst1.LPM_WIDTH = WIDTH+1,
    local_shifter_inst1.LPM_WIDTHDIST = LOG2WIDTH,
    local_shifter_inst1.LPM_SHIFTTYPE="ARITHMETIC";
`else
lpm_clshift shifter_inst1(
    .data({sign_ext&opB[WIDTH-1],opB}),
    .distance(sa&(32'hffffffff<<((REGISTERBREAK>0) ? REGISTERBREAK : 0))),
    .direction(shift_direction),
    .result({dum,partial_result}));
 defparam
    shifter_inst1.lpm_width = WIDTH+1,
    shifter_inst1.lpm_widthdist = LOG2WIDTH,
    shifter_inst1.lpm_shifttype="ARITHMETIC";
`endif

register partial_reg
  ({dum,partial_result},clk,resetn,1'b1,{dum_,partial_result_});
    defparam partial_reg.WIDTH=WIDTH+1;

wire [5-1:0] sa_2;
wire shift_direction_2;

register secondstage (sa, clk,resetn,1'b1,sa_2); 
  defparam secondstage.WIDTH=5;

register secondstagedir (shift_direction, clk,resetn,1'b1,shift_direction_2); 
  defparam secondstagedir.WIDTH=1;

`ifdef USE_INHOUSE_LOGIC
 local_shifter local_shifter_inst2(
  .data({dum_,partial_result_}),
  .distance(sa_2[((REGISTERBREAK>0) ? REGISTERBREAK-1 : 0):0]),
  .direction(shift_direction_2),
  .result({dum2,result})
 );
 defparam
    local_shifter_inst2.LPM_WIDTH = WIDTH+1,
    local_shifter_inst2.LPM_WIDTHDIST = (REGISTERBREAK>0) ? REGISTERBREAK : 1,
    local_shifter_inst2.LPM_SHIFTTYPE ="ARITHMETIC";
`else
lpm_clshift shifter_inst2(
    .data({dum_,partial_result_}),
    .distance(sa_2[((REGISTERBREAK>0) ? REGISTERBREAK-1 : 0):0]),
    .direction(shift_direction_2),
    .result({dum2,result}));
 defparam 
    shifter_inst2.lpm_width = WIDTH+1,
    shifter_inst2.lpm_widthdist = (REGISTERBREAK>0) ? REGISTERBREAK : 1,
    shifter_inst2.lpm_shifttype="ARITHMETIC";
`endif


endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vlane_barrelshifter.v
//////////////////////////////////////////////////////////////////////

/****************************************************************************
          MUL unit

opA/B ---------------------------|\
         |                       | |----| Multiplier |--+------------------
         ------|shiftregister|---|/                     |--|shiftregister|-

 |  Pipe stage 1  |      Pipe stage 2    |    Pipe stage 3
                         done/stall
   operands----Multiply------------barrelshifter ----------

Notes:

  Stalls no matter the vector length - if the mask bits are all off we could
  theoretically skip this computation, but then we would need multiplexing
  logic to order the different results correctly so they line up with their
  lane.  Since we use a shift register to do this, all NUMLANES multiplies are
  performed.

****************************************************************************/

module vmul_unit(clk, resetn,
            op,
            activate,
            en,
            squash,
            stall,
            opA, opB,
            vshamt,
            vmask,
            in_dst,
            in_dst_we,
            out_dst,
            out_dst_we,
            out_dst_mask,
            result
            );
parameter LOG2WIDTH=5;
parameter NUMMULLANES=3;
parameter LOG2NUMLANES=4;
parameter REGIDWIDTH=4;

parameter NUMLANES=2**LOG2NUMLANES;
parameter WIDTH=2**LOG2WIDTH;

input clk;
input resetn;

input [NUMLANES*WIDTH-1:0] opA;
input [NUMLANES*WIDTH-1:0] opB;
input [((LOG2WIDTH==0) ? 1 : LOG2WIDTH)-1:0] vshamt;  // Fixed point rounding
input [NUMLANES-1:0] vmask;
input [4:0] op;
input       activate;
input [3:1] en;  //Enable for each pipestage
input [3:1] squash;  //Squash for each pipestage

input    [REGIDWIDTH-1:0] in_dst;
input                     in_dst_we;
output [3*REGIDWIDTH-1:0] out_dst;
output              [2:0] out_dst_we;
output   [3*NUMLANES-1:0] out_dst_mask;

output stall;
output [NUMLANES*WIDTH-1:0] result;

  /********* Circuit Body *********/
  wire [NUMMULLANES*WIDTH-1:0] mul_opA;
  wire [NUMMULLANES*WIDTH-1:0] mul_opB;
  wire [NUMMULLANES*WIDTH-1:0] mul_result;
  wire [NUMMULLANES*WIDTH-1:0] rshift_result;
  wire [NUMLANES*WIDTH-1:0] result_tmp;

  wire [NUMLANES*WIDTH-1:0] opA_buffered;
  wire [NUMLANES*WIDTH-1:0] opB_buffered;
  wire [NUMLANES-1:0]       mask_buffered;
  wire [NUMLANES*WIDTH-1:0] result_buffered;
  reg  done;
wire [(((3)-(1)+1)*((4)-(0)+1))-1 : 0] ctrl_op;
  wire [3:1] ctrl_activate;                 //3 pipe stages
  //amana: Making modification for VTR
  //wire [((LOG2WIDTH==0) ? 1 : LOG2WIDTH)-1:0] ctrl_vshamt[3:1]; //3 pipe stages
  wire [((LOG2WIDTH==0) ? 1*3 : LOG2WIDTH*3)-1:0] ctrl_vshamt; //3 pipe stages

  //Shift Register for all multiplier operands for lanes without multipliers
  wire [WIDTH*NUMMULLANES-1:0] opA_elmshifter_shiftin_right_NC;
  velmshifter #(NUMLANES/NUMMULLANES,WIDTH*NUMMULLANES) opA_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*NUMMULLANES{1'b0}}),
    .shiftin_right(opA_elmshifter_shiftin_right_NC), 
    .inpipe(opA),
    .outpipe(opA_buffered)
  );

  wire [WIDTH*NUMMULLANES-1:0] opB_elmshifter_shiftin_right_NC;
  velmshifter #(NUMLANES/NUMMULLANES,WIDTH*NUMMULLANES) opB_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*NUMMULLANES{1'b0}}),
    .shiftin_right(opB_elmshifter_shiftin_right_NC), 
    .inpipe(opB),
    .outpipe(opB_buffered)
  );

  wire [NUMMULLANES-1:0] mask_elmshifter_shiftin_right_NC;
  velmshifter #(NUMLANES/NUMMULLANES,NUMMULLANES) mask_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done & ctrl_activate[1]),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({NUMMULLANES{1'b0}}),
    .shiftin_right(mask_elmshifter_shiftin_right_NC), 
    //.inpipe(vmask), //DISABLE - always do all multiplications
    .inpipe({NUMLANES{1'b1}}),
    .outpipe(mask_buffered)
  );

  //Shift Register for all multiplier operands w/ parallel load
  always@(posedge clk)
  begin
    if (!resetn || NUMMULLANES==NUMLANES)
      done<=1;
    else if (done && ctrl_activate[1] && en[1])
      //done<=~(|(vmask>>NUMMULLANES)); // multiply only if mask - DISABLED
      done<=~(|(vmask));
    else
      done<=~( |(mask_buffered >> (2*NUMMULLANES) ));
  end

  assign mul_opA=(done) ? opA : (opA_buffered >> NUMMULLANES*WIDTH);

  assign mul_opB=(done) ? opB : (opB_buffered >> NUMMULLANES*WIDTH);

  assign stall=~done && (ctrl_activate[2]);

  wire [3:1] oppipe_squash_NC; 
  pipe #(5,2) oppipe (
    .d(op),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(oppipe_squash_NC),
    .q({ctrl_op[((3-1)*((4)-(0)+1)+4-0) : ((3-1)*((4)-(0)+1))],ctrl_op[((2-1)*((4)-(0)+1)+4-0) : ((2-1)*((4)-(0)+1))],ctrl_op[((1-1)*((4)-(0)+1)+4-0) : ((1-1)*((4)-(0)+1))]}));

  wire [3:1] activatepipe_squash_NC; 
  pipe #(1,2) activatepipe (
    .d(activate),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(activatepipe_squash_NC),
    .q({ctrl_activate[3],ctrl_activate[2],ctrl_activate[1]}));

  wire [3:1] vshamtpipe_squash_NC; 
  pipe #(((LOG2WIDTH==0) ? 1 : LOG2WIDTH),2) vshamtpipe (
    .d(vshamt),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(vshamtpipe_squash_NC),
    //.q({ctrl_vshamt[3],ctrl_vshamt[2],ctrl_vshamt[1]}));
    .q(ctrl_vshamt));

  //============== Instantiate across lanes =============
  genvar k;
  generate
  for (k=0; k<NUMMULLANES; k=k+1)
  begin : lanes_gen

    vlane_mulshift #(WIDTH,(LOG2WIDTH>0) ? LOG2WIDTH : 1) vmul(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*k +: WIDTH]),
      .opB(mul_opB[WIDTH*k +: WIDTH]),
      .sa( mul_opB[WIDTH*k+((LOG2WIDTH>0)?LOG2WIDTH-1:0) : WIDTH*k] ),
      .op( (done&en[1]) ? ctrl_op[((1-1)*((4)-(0)+1)+4-0) : ((1-1)*((4)-(0)+1))] : ctrl_op[((2-1)*((4)-(0)+1)+4-0) : ((2-1)*((4)-(0)+1))]),
      .result(mul_result[WIDTH*k +: WIDTH])
      );

    vlane_barrelshifter #(WIDTH,(LOG2WIDTH>0)?LOG2WIDTH:1) vshift(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(k+1)-1:WIDTH*k]),
     // .sa(ctrl_vshamt[2][((LOG2WIDTH>0) ? LOG2WIDTH-1:0) : 0]),
      .sa(ctrl_vshamt[2*LOG2WIDTH-1:LOG2WIDTH]),
      .op({~ctrl_op[((2-1)*((4)-(0)+1)+1-1) : ((2-1)*((4)-(0)+1))] ,1'b1}),
      .result(rshift_result[WIDTH*(k+1)-1:WIDTH*k])
      );

  end
  endgenerate


  //Shift Register for all multiplier results
  wire [NUMMULLANES*WIDTH-1:0] shiftin_right_result_elmshifter_NC;
  wire [NUMLANES*WIDTH-1:0] inpipe_result_elmshifter_NC;
  velmshifter #(NUMLANES/NUMMULLANES,WIDTH*NUMMULLANES) result_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(1'b0),
    .shift(~done),
    .dir_left(1'b0),
    .squash(1'b0),
    //Enable/Disable rshifter for fixed-point multiply support
    //.shiftin_left(rshift_result),
    .shiftin_left(mul_result),
    .shiftin_right(shiftin_right_result_elmshifter_NC),
    .inpipe(inpipe_result_elmshifter_NC),
    .outpipe(result_buffered)
  );

  //Enable/Disable rshifter for fixed-point multiply support
  //assign result_tmp=rshift_result;
  assign result_tmp=mul_result;

  assign result=(result_tmp<<((NUMLANES/NUMMULLANES-1)*NUMMULLANES*WIDTH)) |
                (result_buffered>>NUMMULLANES*WIDTH);

  wire [2:1] dstpipe_squash_NC;
  pipe #(REGIDWIDTH,2) dstpipe (
    .d( in_dst ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(dstpipe_squash_NC),
    .q(out_dst));

  pipe #(1,2) dstwepipe (
    .d( in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(squash[2:1]),
    .q(out_dst_we));

  wire [2:1] dstmaskpipe_squash_NC;
  pipe #(NUMLANES,2) dstmaskpipe (
    .d( vmask ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(dstmaskpipe_squash_NC),
    .q(out_dst_mask));


endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vmul_unit.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vmem_unit.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vmem_unit.v
//////////////////////////////////////////////////////////////////////
// THIS UNIT SHOULD BE REDESIGNED!!!!!
// It started off simple with low performance and after adding a bunch of hacks
// to make it perform better it's gotten messy.  A new memory unit should
// special case strided loads and service them early and pipelined, and it
// should have a store buffer with variable width for writes.

/************************
 *
 * VPUWIDTH must equal CONTROLWIDTH must equal 32 
 * NUMPARALLELLANES must be a divisor of NUMLANES
 *
 * op[6] - memop 1-memory operation, 0-shift operation
 *
 * op[5] op[4]
 *   0     0    UNIT
 *   0     1    STRIDE
 *   1     0    INDEX
 *
 * size1 size0 signd ld/st
 * op[3] op[2] op[1] op[0]
 *   0     0     0     0    VLD.u.b
 *   0     1     0     0    VLD.u.h
 *   1     0     0     0    VLD.u.w
 *   0     0     1     0    VLD.b
 *   0     1     1     0    VLD.h
 *   1     0     1     0    VLD.w
 *   0     0     X     1    VST.b
 *   0     1     X     1    VST.h
 *   1     0     X     1    VST.w
 *
 *
 * Ported to handle element shifting as well:
 *
 * Data shifts
 * ===========
 * vins -   L shift vindex
 * vext -     R shift vindex
 * vcompress -  R
 * vexpand -  L 
 * vfins -  L vindex
 * vhalf -        R shift vl/2
 * vexthalf -     R shift vl/2
 * vhalfup -  R shift 2^vindex
 * vhalfdn -  L shift 2^vindex
 *
 * Mask shifting - much harder
 * =============
 *
 * Quartus 5.0: 919 LEs, 137 MHz (CritPath: vip_r to vreaddata)
 *************************/
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: velmshifter_serial.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/velmshifter_serial.v
//////////////////////////////////////////////////////////////////////
/************************
 * An Inter-lane shift register
 *
 * This is much more scalable than a giant barrel shifter/crossbar, however
 * it is also slower in cycles.
 *
 *  dir_left:  1=left, 0=right
 *
 * Synthesis results for NUMLANES=4, WIDTH=32:
 *    131 LEs (Stratix I), 422 MHz
 *************************/

/***************************************************************************
  shiftin_left -> bn-1 <-> bn-2 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

module velmrotator (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    rotate,       // Sets whether ends feedback into beginning
    squash,

    inpipe,
    outpipe

    );

parameter NUMLANES=4;      
parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  rotate;
input [ NUMLANES-1:0 ]  squash;

input [ NUMLANES*WIDTH-1:0 ]  inpipe;
output [ NUMLANES*WIDTH-1:0 ] outpipe;

wire [ WIDTH-1:0 ]  shiftin_left;
wire [ WIDTH-1:0 ]  shiftin_right;


  assign shiftin_right = (!rotate) ? 0 : 
                        outpipe[NUMLANES*WIDTH-1:(NUMLANES-1)*WIDTH];

  assign shiftin_left = (!rotate) ? 0 : outpipe[WIDTH-1:0];

  velmshifter #(NUMLANES,WIDTH) velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load),
      .shift(shift),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe(inpipe),
      .outpipe(outpipe));


endmodule


/******************************************************************************/
/**************************** Shifter w Jump **********************************/
/******************************************************************************/

module velmshifter_jump (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    jump,
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe
    );

parameter NUMLANES=4;      
parameter JUMPSIZE=4;     // We can either shift by 1, or jump by this amount 
parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  jump;              //0-shift by 1, 1 shift by JUMPSIZE
input [ NUMLANES-1:0 ]  squash;

input [ NUMLANES*WIDTH-1:0 ]  inpipe;
output [ NUMLANES*WIDTH-1:0 ] outpipe;

input [ WIDTH-1:0 ]  shiftin_left;
input [ WIDTH-1:0 ]  shiftin_right;

genvar i;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter #(NUMLANES,WIDTH) velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<(WIDTH*JUMPSIZE) :
                      outpipe >>(WIDTH*JUMPSIZE)),
      .outpipe(outpipe));

endmodule

/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

parameter NUMLANES=4;      
parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ NUMLANES-1:0 ]  squash;

input [ NUMLANES*WIDTH-1:0 ]  inpipe;
output [ NUMLANES*WIDTH-1:0 ] outpipe;

input [ WIDTH-1:0 ]  shiftin_left;
input [ WIDTH-1:0 ]  shiftin_right;

wire [ (NUMLANES+1)*WIDTH-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[WIDTH-1:0],
      _outpipe[((NUMLANES>1) ? WIDTH : 0)+:WIDTH], //Support 1 lane
      shiftin_right,
      _outpipe[WIDTH-1:0]);
  defparam velmshifter_laneunit0.WIDTH=WIDTH;

  //Generate everything in between 

  generate
    for (i=1; i<NUMLANES-1; i=i+1)
    begin : elmshift
      velmshifter_laneunit velmshifter_laneunit(clk,resetn,load,shift,dir_left,
          squash[i],
          inpipe[(i+1)*WIDTH-1:i*WIDTH],
          _outpipe[(i+2)*WIDTH-1:(i+1)*WIDTH],
          _outpipe[(i)*WIDTH-1:(i-1)*WIDTH],
          _outpipe[(i+1)*WIDTH-1:i*WIDTH]);
      defparam velmshifter_laneunit.WIDTH=WIDTH;
    end
  endgenerate

  //HANDLE lane NUMLANE specially

    velmshifter_laneunit velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[NUMLANES-1],
      inpipe[NUMLANES*WIDTH-1:(NUMLANES-1)*WIDTH],
      shiftin_left,
      _outpipe[(((NUMLANES>1) ? NUMLANES:2)-2)*WIDTH +: WIDTH], //L=1
      _outpipe[((NUMLANES>1) ? (NUMLANES-1)*WIDTH : WIDTH) +: WIDTH]); //L=1
    defparam velmshifter_laneunitlast.WIDTH=WIDTH;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule



module velmshifter_laneunit (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );

parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ WIDTH-1:0 ]  inpipe;
input [ WIDTH-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ WIDTH-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ WIDTH-1:0 ] outpipe;

reg [ WIDTH-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: velmshifter_serial.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vmem_crossbar.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vmem_crossbar.v
//////////////////////////////////////////////////////////////////////
/************************
 *
 *************************/

module vmem_crossbar (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter INWIDTH=128;       //Bit width of input
parameter LOG2INWIDTH=7;
parameter NUMOUTS=16;        //Number of Outputs
parameter OUTWIDTH=8;        //Bit width of each Output
parameter LOG2OUTWIDTH=3;

parameter SELWIDTH=LOG2INWIDTH-LOG2OUTWIDTH;   // LOG2(INWIDTH/OUTWIDTH) = 4

input                             clk;
input                             resetn;
input  [(SELWIDTH*NUMOUTS)-1 : 0] sel;
input  [INWIDTH-1 : 0]            in;
output [(OUTWIDTH*NUMOUTS)-1 : 0] out;

genvar i;

 generate
   for (i=0; i < NUMOUTS; i=i+1) begin : MEM
     vmem_busmux bmux(clk,resetn,
        sel[(i+1)*SELWIDTH - 1 : i*SELWIDTH],
        in,
        out[(i+1)*OUTWIDTH - 1 : i*OUTWIDTH]);
      defparam bmux.INWIDTH=INWIDTH;
      defparam bmux.LOG2INWIDTH=LOG2INWIDTH;
      defparam bmux.OUTWIDTH=OUTWIDTH;
      defparam bmux.LOG2OUTWIDTH=LOG2OUTWIDTH;
   end
 endgenerate


endmodule



module vmem_busmux (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter INWIDTH=128;
parameter LOG2INWIDTH=7;
parameter OUTWIDTH=8;
parameter LOG2OUTWIDTH=3;

parameter SELWIDTH=LOG2INWIDTH-LOG2OUTWIDTH;   // LOG2(INWIDTH/OUTWIDTH) = 4

input clk;
input resetn;
input  [SELWIDTH-1 : 0] sel;
input  [INWIDTH-1 : 0]  in;
output [OUTWIDTH-1 : 0] out;

reg    [OUTWIDTH-1 : 0] out;

integer k;

  always@*
  begin
    out=0;
    for (k=0; k<INWIDTH/OUTWIDTH; k=k+1)
      if (k==sel)
        out=in[ k*OUTWIDTH +: OUTWIDTH ];
  end


endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vmem_crossbar.v
//////////////////////////////////////////////////////////////////////

module vmem_unit (
    clk,
    resetn,

    enable,       //Is this unit on? (is it less than VL, does the instr use it)
    en, //Is pipeline advancing
    squash, //Is pipeline squashing us
    op,
    stall,
    last_subvector,

    // Control ports
    cbase,
    cstride,
    cprefetch,

    // Vector ports
    vmask,
    vstrideoffset,  //pre-computed stride offsets for each lane (stride*laneid)
    vindex,
    vwritedata,
    voutput,
    voutput_we,

    // Writeback ports
    in_dst,
    in_dst_we,
    out_dst,
    out_dst_we,
    in_vs_dst_we,
    out_vs_dst_we,

    // Vector operations ports
    sa,
    dir_left,

    // Data memory interface
    dmem_en,
    dmem_address,
    dmem_we,
    dmem_byteen,
    dmem_writedata,
    dmem_readdata,
    dmem_cachematch,
    dmem_cachemiss,
    dmem_prefetch,
    dmem_wait
    );

parameter VPUWIDTH=32;       // Width of the Vector processing unit in bits!
parameter NUMLANES=4;        // Number of vector lanes
parameter LOG2NUMLANES=2;    // Log base 2 of Number of vector lanes
parameter NUMPARALLELLANES=2;        // Number of vector lanes
parameter LOG2NUMPARALLELLANES=1;    // Log base 2 of Number of vector lanes
parameter CONTROLWIDTH=32;   // Width of the Control Registers
parameter DMEM_WRITEWIDTH=128;     // Width of write bus to memory
parameter LOG2DMEM_WRITEWIDTH=7;  // Log2 of Width of write bus to memory
parameter DMEM_READWIDTH=128;     // Width of read bus from memory
parameter LOG2DMEM_READWIDTH=7;   // Log2 of Width of read bus from memory
parameter ELMIDWIDTH=2;           // Width of register element identifier
parameter REGIDWIDTH=5;           // Width of register element identifier

parameter       MUNIT_IDLE=0,
                MUNIT_ISSUE=1,
                MUNIT_DONE=2,
                MUNIT_SHIFT=3, 
                MUNIT_XTRAWRITE=4; 
  
parameter ISSUE_IDLE=3'b000,
          ISSUE_INITIAL_INCREMENT=3'b001,
          ISSUE_INCREMENTED=3'b010,
          ISSUE_WAITONE=3'b011,
          ISSUE_WAITING=3'b100;

input clk;
input resetn;

input  enable;
input  [2:0] en; //Is pipeline one
input  [2:0] squash;
input [6:0] op;
output stall;
input last_subvector;

// Control ports
input [ CONTROLWIDTH-1 : 0 ] cbase;
input [ CONTROLWIDTH-1 : 0 ] cstride;
input [ CONTROLWIDTH-1 : 0 ] cprefetch;

// Vector ports
input  [          NUMLANES-1 : 0 ]  vmask;
input  [ NUMLANES*CONTROLWIDTH-1 : 0 ]  vstrideoffset;
input  [ NUMLANES*VPUWIDTH-1 : 0 ]  vindex;
input  [ NUMLANES*VPUWIDTH-1 : 0 ]  vwritedata;
output [ NUMLANES*VPUWIDTH-1 : 0 ]  voutput;
output [          NUMLANES-1 : 0 ]  voutput_we;

input   [REGIDWIDTH-1:0]  in_dst;
input                     in_dst_we;
output   [REGIDWIDTH-1:0] out_dst;
output                    out_dst_we;
input                     in_vs_dst_we;
output                    out_vs_dst_we;

input  [      LOG2NUMLANES-1 : 0 ]  sa;
input                               dir_left;

// Data memory interface
output dmem_en;
output dmem_we;
output  [ 31 : 0 ]                  dmem_address;
output  [ DMEM_WRITEWIDTH/8-1 : 0 ] dmem_byteen;
output  [ DMEM_WRITEWIDTH-1 : 0 ]   dmem_writedata;
input   [ DMEM_READWIDTH-1 : 0 ]    dmem_readdata;
input                               dmem_cachematch;
input                               dmem_cachemiss;
output  [ 31 : 0 ]                  dmem_prefetch;
input                               dmem_wait;

reg     [ 31 : 0 ]                  dmem_address;
reg     [ DMEM_WRITEWIDTH/8-1 : 0 ] dmem_byteen;
reg     [ DMEM_WRITEWIDTH-1 : 0 ]   dmem_writedata;
reg     [ 31 : 0 ]                  dmem_prefetch;

wire  [1:0]  op_pattern;      // 0-Unit, 1-Strided, 2-Indexed
wire  [1:0]  op_size;         // 0-byte, 1-16bits, 2-32bits, 3-64bits
wire         op_signed;       // 0-unsigned, 1-signed
wire         op_we;         // 0-load, 1-store
wire         op_memop;         // 1-memory op, 0-vector shift op

wire  [ NUMPARALLELLANES*VPUWIDTH-1 : 0 ]  __vreaddata;
reg           [ NUMLANES*CONTROLWIDTH-1 : 0 ]  address;
reg              [ NUMLANES*VPUWIDTH-1:0]  vreaddata;
reg                    [ NUMLANES-1 : 0 ]  vreaddata_we;
reg   [ NUMPARALLELLANES*(LOG2DMEM_READWIDTH-5)-1 : 0 ] crossbar_sel;
wire  [ NUMPARALLELLANES*32-1 : 0 ] crossbar;
wire  [ NUMPARALLELLANES*32-1 : 0 ]  _vwritedata;
wire   [ NUMPARALLELLANES*4-1 : 0 ]  _vbyteen;

reg  [ CONTROLWIDTH-1 : 0 ] stride;
wire [ CONTROLWIDTH-1 : 0 ] stride_tmp;
reg  [ CONTROLWIDTH-1 : 0 ] prefetch;
wire [ CONTROLWIDTH-1 : 0 ] t_cprefetch;

wire           [         NUMLANES-1 : 0 ]  vshifted_mask;
wire           [         NUMLANES-1 : 0 ]  vshifted_masksave;
wire          [ NUMLANES*VPUWIDTH-1 : 0 ]  vshifted_writedata;
wire          [ NUMLANES*CONTROLWIDTH-1 : 0 ]  vshifted_address;


reg                           dmem_valid;
reg   [ 31 : 0 ]              dmem_readdata_address;
reg                           cachedata_stillvalid;

reg [2:0] munit_state;
reg [2:0] issue_state;

reg  [ LOG2NUMLANES-1 : 0 ] vpid;
reg  [ NUMLANES-1 : 0 ] done;
wire doneall;

wire  [         NUMLANES-1 : 0 ]  _vreaddata_we;
reg   [ NUMPARALLELLANES-1 : 0 ]  _parhits;
wire  [ NUMPARALLELLANES-1 : 0 ]  parhits;
reg   [ NUMPARALLELLANES-1 : 0 ]  parhits_done;
wire  parhits_doneall;
wire  parhits_all;
wire  parhits_none;
wire  parhits_some;

wire  shifter_dirleft;
wire  shifter_load;
wire  shifter_shift;
wire  shifter_jump;
wire  shifter_jump_s1;

wire addr_sneakyload;
wire addr_advance;
wire addr_rewind;

wire do_quick_load;
reg quick_loaded;

reg  [ LOG2NUMLANES-1 : 0 ] sa_count;
wire [ LOG2NUMLANES-1 : 0 ] next_sa_count;
wire                        next_sa_count_zero;
wire                        sa_zero;
reg  [ NUMLANES*VPUWIDTH-1 : 0 ]  vshiftresult;

genvar i;
integer j;
genvar k;
integer l;
integer m;
integer n;
integer p;

  assign {op_memop,op_pattern,op_size,op_signed,op_we}=op;

  assign stall= (munit_state!=MUNIT_IDLE && munit_state!=MUNIT_DONE);

/************************** Pipeline load stage *******************************/
  reg enable_s2;
  reg last_subvector_s2;
  reg [ REGIDWIDTH-1 : 0 ] in_dst_s2;
  reg [      LOG2NUMLANES-1 : 0 ] sa_s2;
  reg dir_left_s2;
  reg  [1:0]  op_pattern_s2;      // 0-Unit, 1-Strided, 2-Indexed
  reg  [1:0]  op_size_s2;         // 0-byte, 1-16bits, 2-32bits, 3-64bits
  reg         op_signed_s2;       // 0-unsigned, 1-signed
  reg         op_we_s2;         // 0-load, 1-store
  reg         op_memop_s2;         // 1-memory op, 0-vector shift op

  always@(posedge clk)
    if (!resetn)
    begin
      enable_s2<=0;
      last_subvector_s2<=0;
      in_dst_s2<=0;
      sa_s2<=0;
      dir_left_s2<=0;
      {op_memop_s2,op_pattern_s2,op_size_s2,op_signed_s2,op_we_s2}<=0;
    end
    else if (!stall)
    begin
      enable_s2<=enable;
      last_subvector_s2<=last_subvector;
      if (en[0])
        in_dst_s2<=in_dst;
      sa_s2<=sa;
      dir_left_s2<=dir_left;
      {op_memop_s2,op_pattern_s2,op_size_s2,op_signed_s2,op_we_s2}<=op;
    end

/*************************** Vector op logic *********************************/

  assign next_sa_count=sa_count-1;
  assign next_sa_count_zero=(next_sa_count==0);
  assign sa_zero=(sa==0);

  always@(posedge clk)
  begin
    if (!resetn)
      sa_count<=0;
    else if (shifter_load)
      sa_count<=sa;
    else if (shifter_shift)
      sa_count<=next_sa_count;
  end

  // Detect double write one clock cycle early - also support 1 lane
  assign doublewrite=(~op_memop_s2) && (NUMLANES>1) && ( (dir_left_s2) ? 
              (|vshifted_mask[((NUMLANES>1) ? NUMLANES:2)-2:0]) && //support L=1
                (vshifted_mask[NUMLANES-1]&(|vshifted_masksave)) :
              (|vshifted_mask[NUMLANES-1:(NUMLANES>1) ? 1 : 0]) && //support L=1
                (vshifted_mask[0]&(|vshifted_masksave)));

  assign out_dst = {in_dst_s2[REGIDWIDTH-1:ELMIDWIDTH],
                    (munit_state!=MUNIT_XTRAWRITE) ? in_dst_s2[ELMIDWIDTH-1:0] :
                     (dir_left_s2) ? in_dst_s2[ELMIDWIDTH-1:0]+1'b1 : 
                       in_dst_s2[ELMIDWIDTH-1:0]-1'b1};

  //Truncate shifted result to VPW's size
  always@*
    for (p=0; p<NUMLANES; p=p+1)
      vshiftresult[p*VPUWIDTH +: VPUWIDTH]=
              vshifted_address[p*CONTROLWIDTH +: VPUWIDTH];

  assign voutput= (~enable_s2) ? 0 :
                  (op_memop_s2) ? vreaddata : 
                               vshiftresult;
 

  assign voutput_we=(op_memop_s2) ? vreaddata_we : 
                      (munit_state==MUNIT_XTRAWRITE) ? vshifted_masksave : 
                        (munit_state==MUNIT_DONE) ? vshifted_mask : 0;

/*************************** ISSUING LOGIC *********************************/

  assign shifter_load=enable && ~quick_loaded &&
                        (munit_state==MUNIT_IDLE || 
                         munit_state==MUNIT_DONE ||
                         do_quick_load);

  assign shifter_shift= (munit_state==MUNIT_SHIFT) || 
               ((munit_state==MUNIT_ISSUE) && parhits_doneall);

  assign shifter_dirleft=(op_memop_s2) ? 1'b0 : dir_left_s2;
  assign shifter_jump=op_memop_s2 && ~op_pattern_s2[1];
  assign shifter_jump_s1=op_memop && ~op_pattern[1];

  // Address Generator for each lane, or store vector to be shifted
  always@*
    for (m=0; m<NUMLANES; m=m+1)
      address[m*CONTROLWIDTH +: CONTROLWIDTH] = ((op_memop) ? cbase : 0) + 
            ( (op_pattern[1] || ~op_memop)  ? vindex[m*VPUWIDTH +: VPUWIDTH] : 
               (vstrideoffset[m*CONTROLWIDTH +: CONTROLWIDTH]<<op_size));

  wire [NUMLANES-1:0] vwritedata_shifter_squash_NC;
  velmshifter_jump #(NUMLANES,NUMPARALLELLANES,VPUWIDTH) vwritedatashifter(
      .clk(clk),
      .resetn(resetn),    //Don't use if not a store
      .load(shifter_load && op_we),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .jump(shifter_jump),
      .squash(vwritedata_shifter_squash_NC),
      .shiftin_left(0),
      .shiftin_right(0),
      .inpipe(vwritedata),
      .outpipe(vshifted_writedata));

  wire [NUMLANES-1:0] vaddress_shifter_squash_NC;
  velmshifter_jump #(NUMLANES,NUMPARALLELLANES,CONTROLWIDTH) vaddressshifter(
      .clk(clk),
      .resetn(resetn), //consider forcing to 0 if not used
      .load(shifter_load),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .jump(shifter_jump),
      .squash(vaddress_shifter_squash_NC),
      .shiftin_left(0),
      .shiftin_right(0),
      .inpipe( address),
      .outpipe(vshifted_address));

  velmshifter_jump #(NUMLANES,NUMPARALLELLANES,1) vmaskshifter(
      .clk(clk),
      .resetn(resetn),
      .load(shifter_load),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .jump(shifter_jump),
      .squash(0),
      .shiftin_left( 1'b0),
      .shiftin_right( 1'b0 ),
      .inpipe(vmask),
      .outpipe(vshifted_mask));

  //Save spilled over masks for shifting instructions (can only shift by 1)
  wire    [NUMLANES-1 : 0]  vmasks_save_inpipe_NC;
  velmshifter #(NUMLANES,1) vmaskssave(
      .clk(clk),
      .resetn(resetn && ~shifter_load),
      .load(1'b0),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .squash(0),
      .shiftin_left( vshifted_mask[0] ),
      .shiftin_right( vshifted_mask[NUMLANES-1] ),
      .inpipe(vmasks_save_inpipe_NC),
      .outpipe(vshifted_masksave));

  //Stride is maximum of stride specified and cache line size 
  assign stride_tmp = (((op_pattern[0]) ? cstride : 1) << op_size);
  always@(posedge clk)
    if (!resetn || last_subvector_s2&~stall&~shifter_load&~quick_loaded )
      stride<=0;
    else if (shifter_load)
      stride<= (stride_tmp>DMEM_READWIDTH/8) ? 
                      stride_tmp : DMEM_READWIDTH/8;

  wire [15:0] constantprefetch;
  assign constantprefetch=`VECTORPREFETCHES;
  assign t_cprefetch=cprefetch[15:0]<<(65535-`VECTORPREFETCHES);

  //***********************  Prefetch Logic **************************
  //
  //Send stride and vector length to bus.  To do constant prefetching set
  //stride to cache line size and send constant as length
  //******************************************************************
  always@(posedge clk)
    if (!resetn || last_subvector_s2&~stall&~shifter_load&~quick_loaded )
      prefetch<=0;
    else if (shifter_load)
      if (`VECTORPREFETCHES >= 65530)
        prefetch<= { stride_tmp[15:0], t_cprefetch[15:0] };
      else if (`VECTORPREFETCHES == 0)
        prefetch<= 0;
      else
        prefetch<= {16'd0+2**(LOG2DMEM_READWIDTH-3),constantprefetch};

  // State machine for issuing addresses - this is really sneaky!!!!!
  // The cache takes 1 cycle to respond to a cache request and is generally
  // not pipelined - but here we treat it as if it is pipeline by changing
  // the address during a single cache access.  
  //
  // We speculatively increment to the next cache line before we know if the
  // previous cache request hit or missed.  If it hits then we're fine and
  // we get pipelined outputs from the cache, but if not, we have to rewind 
  // our increment and wait for the data to come in.
  //
  // NOTE: We only do this for loads - writes write on the second clock
  always@(posedge clk)
  begin
    if (!resetn)
      issue_state<=ISSUE_IDLE;
    else
      case(issue_state)
        ISSUE_IDLE:
          issue_state<= (addr_sneakyload) ? 
                          ISSUE_INITIAL_INCREMENT: MUNIT_IDLE;
        ISSUE_INITIAL_INCREMENT:
          issue_state<= (doneall && ~addr_sneakyload) ? ISSUE_IDLE :
                      (doneall && addr_sneakyload) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_doneall && !doneall) ? ISSUE_INITIAL_INCREMENT :
                                            ISSUE_INCREMENTED;
        ISSUE_INCREMENTED:
          issue_state<= (doneall && ~addr_sneakyload) ? ISSUE_IDLE :
                      (doneall && addr_sneakyload) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_doneall) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_none) ? ISSUE_WAITONE : ISSUE_INCREMENTED;
        // This stage ignores the speculated result following a miss
        ISSUE_WAITONE:      
          issue_state<= ISSUE_WAITING;
        ISSUE_WAITING:
          issue_state<= (doneall && ~addr_sneakyload) ? ISSUE_IDLE :
                      (doneall && addr_sneakyload) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_doneall) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_some) ? ISSUE_INITIAL_INCREMENT : ISSUE_WAITING;
      endcase
  end

  assign addr_sneakyload=(shifter_load && op_memop && ~op_we && ~op_pattern[1] && enable);
                  //(shifter_shift && ~shifter_load && ~op_we_s2 && enable_s2));

  assign addr_advance=(issue_state==ISSUE_INITIAL_INCREMENT) ||
                      (issue_state==ISSUE_INCREMENTED && parhits_some) ||
                      (issue_state==ISSUE_WAITING && parhits_some) ||
                      (op_we_s2 && parhits_some && ~cachedata_stillvalid);

  assign addr_rewind=(issue_state==ISSUE_INCREMENTED && parhits_none);

  always@(posedge clk)
    if (!resetn || shifter_load || last_subvector_s2&~stall&~shifter_load )
      vpid<=0;
    else if (shifter_shift)
      vpid<=vpid + ((shifter_jump) ? NUMPARALLELLANES : 1'b1);

  // Loads in a new request if it is a memory request. quick_loaded is high 
  // the cycle after the new request has been loaded and instructs the
  // logic below to continue requesting from memory (even though munit_state
  // will be in the DONE state).
  // Note we don't do quickload on last subvector - a speculative miss can 
  // cause the quickloaded memory instruction to re-load its cache line from
  // memory - but if that line was dirty we would have missed the changes.
    // I don't think this is true anymore PY 07/2008
  assign do_quick_load=doneall&enable_s2&op_memop_s2&~op_we_s2&
                       ~last_subvector_s2&enable&op_memop&~op_we&~op_pattern[1];
  always@(posedge clk)
    quick_loaded<=shifter_load&do_quick_load;

  assign dmem_en = ((shifter_jump) ? (|vshifted_mask[NUMPARALLELLANES-1:0]) : 
                                      vshifted_mask[0]) && 
                    (munit_state==MUNIT_ISSUE || quick_loaded);
  assign dmem_we=op_we_s2;

  /*********************
  * We don't want cache miss signal to be an add/sub select signal or else it
  * will propagate through the adder.  We perform the addition/subtraction
  * in parallel with the miss calculation and use the miss as a mux select
  *********************/
  wire [ CONTROLWIDTH-1 : 0 ] dmem_address_next /* synthesis keep */;
  wire [ CONTROLWIDTH-1 : 0 ] dmem_address_prev /* synthesis keep */;
  assign dmem_address_next=dmem_address + stride;
  assign dmem_address_prev=dmem_address - stride;

  always@(posedge clk)
    if (!resetn)
      dmem_address<=0;
    else if (shifter_load)
      dmem_address<=address[CONTROLWIDTH-1:0];
    else if (shifter_shift)
      dmem_address<= (!shifter_jump) ? 
              vshifted_address[((NUMLANES>1) ? 1:0)
                *CONTROLWIDTH +: CONTROLWIDTH] :
              vshifted_address[
                ((NUMLANES<=NUMPARALLELLANES) ? 0 : NUMPARALLELLANES)
                  *CONTROLWIDTH +: CONTROLWIDTH];
    //Fetch next cache line if partial match on initial cache access
    //else if (parhits_some && ~cachedata_stillvalid)
    //else if (addr_advance)
    //  dmem_address<=dmem_address_next;
    //else if (addr_rewind)
    //  dmem_address<=dmem_address_prev;

  always@(posedge clk)
    if (!resetn)
      dmem_prefetch<=0;
    //else if (shifter_load)
    else
      dmem_prefetch<=prefetch[CONTROLWIDTH-1:0];

/*************************** Mem Write LOGIC ********************************/

    // Generate byte/halfword alignment circuitry for each word
  generate
  for (i=0; i<NUMPARALLELLANES; i=i+1)
  begin : write_gen
    vstore_data_translator vstore_data_translator(
      //Pad vshifted_writedata with zeros incase signal is less than 32-bits
      .write_data({32'b0,vshifted_writedata[i*VPUWIDTH +: VPUWIDTH]}),
      .d_address(vshifted_address[i*32 +: 2]),
      .store_size(op_size_s2), 
      .d_byteena(_vbyteen[4*i +: 4]),  
      .d_writedataout(_vwritedata[i*32 +: 32]));
  end
  endgenerate

  always@*
  begin
    dmem_writedata=0;
    dmem_byteen=0;
    for (l=0; l<NUMPARALLELLANES; l=l+1)
      if (dmem_address[31:LOG2DMEM_WRITEWIDTH-3] == 
          vshifted_address[32*l+LOG2DMEM_WRITEWIDTH-3 +: 
                            32-LOG2DMEM_WRITEWIDTH+3])
      begin
        dmem_writedata=dmem_writedata| (_vwritedata[l*32 +: 32] << 
            {vshifted_address[32*l+2 +: LOG2DMEM_WRITEWIDTH-5], {5{1'b0}}});
        if (vshifted_mask[l] && (shifter_jump || (l==0)))
          dmem_byteen=dmem_byteen | (_vbyteen[4*l+:4]<<
            {vshifted_address[32*l+2 +: LOG2DMEM_WRITEWIDTH-5], {2{1'b0}}});
      end
  end

/*************************** Mem Receive LOGIC ********************************/
  reg wastedcache;  //Identifies when a cache request is made but then aborted because the data was found in the previous cache request which is "stillvalid"

  // This is specific to cache timing - 1 cycle
  always@(posedge clk)
    if (!resetn) 
    begin
      dmem_valid<=0;
      dmem_readdata_address<=0;
      wastedcache<=0;
      cachedata_stillvalid<=0;
    end
    else
    begin
      dmem_valid<=dmem_en;
      if (!wastedcache)
        dmem_readdata_address<=dmem_address;
      //WE know for sure data is still valid after the cache hit since it takes
      //two clock cyles to get new cacheline
      //wastedcache<=shifter_shift&cachedata_stillvalid;
      //cachedata_stillvalid<=dmem_valid&~dmem_wait;
      //Disable this since now we speculatively advance the address
      wastedcache<=0;
      cachedata_stillvalid<=0;
    end

  // Find out which parallel lanes hit in the cache
  always@*
    for (j=0; j<NUMPARALLELLANES; j=j+1)
      //This signal tells us a cache request succeeded (for either load/store)
      //Each bit corresponds to a parallel lane
      _parhits[j]= vshifted_mask[j] &&
          ((dmem_valid&(dmem_cachematch&~op_we_s2 || op_we_s2&~dmem_wait)) || 
              (cachedata_stillvalid&~op_we_s2)) &&
            (vshifted_address[j*32+LOG2DMEM_READWIDTH-3 +: 
                             32-LOG2DMEM_READWIDTH+3] ==
               dmem_readdata_address[31:LOG2DMEM_READWIDTH-3]);

  //For operations that don't jump, just look at first bit of _parhits
  assign parhits=(shifter_jump) ? _parhits : {NUMPARALLELLANES{_parhits[0]}};

  // Detect all parallel lanes hitting
  assign parhits_all=&(parhits|~vshifted_mask[NUMPARALLELLANES-1:0]);
  // Detect cache misses
  assign parhits_none=~|(parhits);
  // Detect some misses - fetch next cache line
  assign parhits_some=~parhits_none && ~parhits_all;

  //If NUMLANES<=NUMPARALLELLANES then we will never do a jump, so we make
  //compiler ignore this statement if that's the case
  always@(posedge clk)
    if (!resetn || shifter_load)
      parhits_done<= (shifter_jump_s1) ?  ~vmask : {NUMPARALLELLANES{~vmask[0]}};
    else if ( parhits_doneall )
      parhits_done<= (shifter_jump && (NUMLANES>NUMPARALLELLANES)) ? 
                    ~vshifted_mask[ 
                        ((NUMLANES>NUMPARALLELLANES) ? NUMPARALLELLANES : 0) 
                      +: NUMPARALLELLANES] :
                    {NUMPARALLELLANES{(NUMLANES>1) ? ~vshifted_mask[1] : 1'b0}};
    else           
      parhits_done<= parhits_done|parhits;

  assign parhits_doneall=&(parhits|parhits_done);

  assign _vreaddata_we= ((shifter_jump) ? _parhits : _parhits[0]) 
                          << vpid[LOG2NUMLANES-1:0];

  always@(posedge clk)
    if (!resetn || ~enable_s2)  // Force to zero if unit not used
      vreaddata_we<= 0 ;
    else           // write to regfile if a) is a load, b) we haven't already
      vreaddata_we<= {NUMLANES{~op_we_s2}} & _vreaddata_we & ~done;

  //Select signal for crossbar either uses bits from corresponding parallel
  //lane address or all selects are set to the same if we're not jumping
  always@*
    for (n=0; n<NUMPARALLELLANES; n=n+1)
      crossbar_sel[(LOG2DMEM_READWIDTH-5)*n +: (LOG2DMEM_READWIDTH-5)]=
            (!shifter_jump) ? 
              {NUMPARALLELLANES{vshifted_address[2 +: LOG2DMEM_READWIDTH-5]}} :
              vshifted_address[n*CONTROLWIDTH+2 +: (LOG2DMEM_READWIDTH-5)];

  vmem_crossbar vmem_crossbar(
      .clk(), .resetn(),
      .sel(crossbar_sel),
      .in(dmem_readdata),
      .out(crossbar));
    defparam vmem_crossbar.INWIDTH=DMEM_READWIDTH,
             vmem_crossbar.LOG2INWIDTH=LOG2DMEM_READWIDTH,
             vmem_crossbar.NUMOUTS=NUMPARALLELLANES,
             vmem_crossbar.OUTWIDTH=32,
             vmem_crossbar.LOG2OUTWIDTH=5;

    // Generate byte/halfword alignment circuitry for each word
  generate
  for (k=0; k<NUMPARALLELLANES; k=k+1)
  begin : load_gen
    vload_data_translator load_data_translator(
      .d_readdatain(crossbar[32*(k+1)-1:32*k]),
      .d_address( (shifter_jump) ? vshifted_address[CONTROLWIDTH*k +: 2] :
                                   vshifted_address[1:0]),
      .load_size(op_size_s2[1:0]),
      .load_sign_ext(op_signed_s2),
      .d_loadresult(__vreaddata[VPUWIDTH*(k+1)-1:VPUWIDTH*k])
      );
  end
  endgenerate

  always@(posedge clk)
    //zero if unit not used
    if (!resetn || ~enable_s2 || (~stall&~enable) || op_we_s2)
      vreaddata<= 0 ;
    else                // Don't write to regfile unless this is a load!
      vreaddata<= {NUMLANES/NUMPARALLELLANES{__vreaddata}};

/*************************** DONE LOGIC *********************************/

  // Done register has 1 bit for each lane, we are finished when done is all 1's
  always@(posedge clk)
    if (!resetn)
      done<=0;
    //Don't check for MUNIT_DONE, because of quickload we are reading then
    //else if (shifter_load || (munit_state==MUNIT_DONE) )
    else if (shifter_load || doneall || (munit_state==MUNIT_DONE && (op_we_s2 || op_pattern_s2[1])))
      done<=~vmask;
    else 
      done<=done|_vreaddata_we;

  assign doneall=(&(done|_vreaddata_we));

  always@(posedge clk)
  begin
    if (!resetn)
      munit_state<=MUNIT_IDLE;
    else
      case(munit_state)
        MUNIT_IDLE:
          munit_state<= (enable) ? (op_memop) ? MUNIT_ISSUE : 
                                      (sa_zero) ? MUNIT_DONE : MUNIT_SHIFT : 
                                   MUNIT_IDLE;
        MUNIT_ISSUE:
          munit_state<= doneall ? MUNIT_DONE : MUNIT_ISSUE;
        MUNIT_SHIFT:
          munit_state<= (next_sa_count_zero) ? 
                            (doublewrite) ? MUNIT_XTRAWRITE : MUNIT_DONE : 
                            MUNIT_SHIFT;
        MUNIT_XTRAWRITE:  //Spilled over subvector bounds when shifting 
          munit_state<= MUNIT_DONE;
        MUNIT_DONE:
          munit_state<= (enable) ? (op_memop) ? MUNIT_ISSUE : 
                                      (sa_zero) ? MUNIT_DONE : MUNIT_SHIFT : 
                                   MUNIT_IDLE;
        default:
          munit_state<=MUNIT_IDLE;
      endcase
  end

  pipereg #(1) dstwepipe (
    .d( in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .squashn(1'b1),
    .en( en[0] && ~stall ),
    .q(out_dst_we));

  pipereg #(1) vsdstwepipe (
    .d( in_vs_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .squashn(1'b1),
    .en( en[0] && ~stall ),
    .q(out_vs_dst_we));

endmodule

/****************************************************************************
          Store data translator
          - moves store data to appropriate byte/halfword 
****************************************************************************/
module vstore_data_translator(
    write_data,             // data in least significant position
    d_address,
    store_size,             // 0-byte, 1-16bits, 2-32bits, 3-64bits
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

always @*
begin
    case (store_size)
        2'b00:
        begin
            case(d_address[1:0])
                2'b00: begin d_byteena=4'b1000; 
                        d_writedataout={write_data[7:0],24'b0}; end
                2'b01: begin d_byteena=4'b0100;
                        d_writedataout={8'b0,write_data[7:0],16'b0}; end
                2'b10: begin d_byteena=4'b0010;
                        d_writedataout={16'b0,write_data[7:0],8'b0}; end
                default: begin d_byteena=4'b0001;
                        d_writedataout={24'b0,write_data[7:0]}; end
            endcase
        end
        2'b01:
        begin
            d_writedataout=(d_address[1]) ? {16'b0,write_data[15:0]} : 
                                            {write_data[15:0],16'b0};
            d_byteena=(d_address[1]) ? 4'b0011 : 4'b1100 ;
        end
        default:
        begin
            d_byteena=4'b1111;
            d_writedataout=write_data;
        end
    endcase
end

endmodule

/****************************************************************************
 *           Load data translator
 *- moves read data to appropriate byte/halfword and zero/sign extends
 *****************************************************************************/
module vload_data_translator(
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
        2'b00:
        begin
            case (d_address[1:0])
                2'd0: d_loadresult[7:0]=d_readdatain[31:24];
                2'd1: d_loadresult[7:0]=d_readdatain[23:16];
                2'd2: d_loadresult[7:0]=d_readdatain[15:8];
                default: d_loadresult[7:0]=d_readdatain[7:0];
            endcase
            d_loadresult[31:8]={24{load_sign_ext&d_loadresult[7]}};
        end
        2'b01:
        begin
            case (d_address[1])
                2'd0: d_loadresult[15:0]=d_readdatain[31:16];
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
//// Finish contents of included file: vmem_unit.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vlane_flagalu.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/vlane_flagalu.v
//////////////////////////////////////////////////////////////////////
/******************************************************************************

0  VFAND
1  VFOR
2  VFXOR
3  VFNOR
4  VFCLR
5  VFSET

Not handled here:
  VIOTA
  VCIOTA
  VFPOP
  VFFF1
  VFFL1
  VFSETBF
  VFSETIF
  VFSETOF

******************************************************************************/

module vlane_flagalu (
    clk,
    resetn,

    src1,
    src2,

    op,

    result

    );

input clk;
input resetn;

input src1;
input src2;
input [2:0] op;

output result;

reg result;

  always@*
    case(op)
      0: result=src1&src2;
      1: result=src1|src2;
      2: result=src1^src2;
      3: result=~(src1|src2);
      4: result=1'b0;
      5: result=1'b1;
      default: result=1'b0;
    endcase

endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vlane_flagalu.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: matmul_unit.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/top/matmul_unit.v
//////////////////////////////////////////////////////////////////////

`define DWIDTH 16
`define AWIDTH 10
`define MEM_SIZE 1024

///////////////////////////////////////////////////////////
//MAT_MUL_SIZE refers to the dimension of the matrix
//multiplication performed by the matmul unit. The value 
//8 means it will multiply an 8x8 matrix with an 8x8 matrix.
///////////////////////////////////////////////////////////
//IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP
///////////////////////////////////////////////////////////
//MAT_MUL_SIZE should be equal to number of lanes in the vector processor
///////////////////////////////////////////////////////////
//IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP
///////////////////////////////////////////////////////////

`define MASK_WIDTH 8
`define LOG2_MAT_MUL_SIZE 3

`define BB_MAT_MUL_SIZE `MAT_MUL_SIZE

`define NUM_CYCLES_IN_MAC 3
`define MEM_ACCESS_LATENCY 1
`define REG_DATAWIDTH 32
`define REG_ADDRWIDTH 8
`define ADDR_STRIDE_WIDTH 16
`define MAX_BITS_POOL 3

`define PIPE_STAGES_MATMUL 29

//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: matmul_8x8.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/top/matmul_8x8.v
//////////////////////////////////////////////////////////////////////


module matmul_8x8(
 clk,
 //Reset for the whole matmul
 reset,
 //Reset for the PEs. Typically connected to 'reset'
 pe_reset,
 //When this is asserted, the matmul operation starts. This can remain
 //asserted during the execution, but is not required to.
 start, 
 //This is asserted a cycle after the matmul start operation. It stays
 //asserted until the matmul starts shifting out the output data.
 in_progress,
 //This is not used any more. But it is a pulse, and is asserted when
 //the execution (including output shift out) is finished.
 done,
 //Input data matrix A. For MAT_MUL_SIZE=8, 64 values come in over 8 cycles.
 //8 values in one cycle.
 a_data,
 //Input data matrix B. For MAT_MUL_SIZE=8, 64 values come in over 8 cycles.
 //8 values in one cycle.
 b_data,
 //Output data matrix C. For MAT_MUL_SIZE=8, 64 values come out over 8 cycles.
 //8 values in one cycle.
 c_data_out, 
 //This isn't used anymore. It stays asserted when the output data matrix C
 //is being shifted out.
 c_data_available,
 //Masks for input matrices A and B. These are used when we want to use this 
 //unit to multiply matrices that are less than 8x8 (eg: 6x4 with a 4x5 matrix).
 validity_mask_a_rows,
 validity_mask_a_cols,
 validity_mask_b_rows,
 validity_mask_b_cols
);

 input clk;
 input reset;
 input pe_reset;
 input start;
 output in_progress;
 output done;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data;
 output [`MAT_MUL_SIZE*`DWIDTH-1:0] c_data_out;
 output c_data_available;

 input [`MASK_WIDTH-1:0] validity_mask_a_rows;
 input [`MASK_WIDTH-1:0] validity_mask_a_cols;
 input [`MASK_WIDTH-1:0] validity_mask_b_rows;
 input [`MASK_WIDTH-1:0] validity_mask_b_cols;

//////////////////////////////////////////////////////////////////////////
// Logic for clock counting and when to assert done
//////////////////////////////////////////////////////////////////////////
reg in_progress;
reg done_mat_mul;
wire matmul_op_in_progress;
reg shift_out_data;
reg shift_in_data;
wire start_pulse;
wire [7:0] clk_cnt_for_done;
wire [7:0] clk_cnt_for_latching_c_data;
wire [7:0] clk_cnt_for_shifting_inputs;

assign done = done_mat_mul;

reg start_delayed;
always @(posedge clk) begin
  if (reset) begin
    start_delayed <= 0;
  end 
  else begin
    start_delayed <= start;
  end
end

assign start_pulse = start & (~start_delayed);

//This signal is used in other modules instantiated in this design.
//It stays high during the entire time matmul is working.
//                            _
//start_pulse           _____| |___________________________________________
//                              ___
//shift_in_data         _______|   |_______________________________________
//                                   ________________________
//in_progress           ____________|                        |______________
//                                                            _______
//shift_out_data        _____________________________________|       |______
//                            _______________________________________
//matmul_op_in_progress _____|                                       |______
assign matmul_op_in_progress = start_pulse|shift_in_data|in_progress|shift_out_data;

//This is 7 bits because the expectation is that clock count will be pretty
//small. For large matmuls, this will need to increased to have more bits.
//In general, a systolic multiplier takes 4*N-2+P cycles, where N is the size 
//of the matmul and P is the number of pipleine stages in the MAC block.
reg [7:0] clk_cnt;

//Number of cycles to assert matmul done. This includes the cycles to shift out the results. 
//This is hardcoded here, because this was generated using a script.
assign clk_cnt_for_done = 
                          (34);  

//Number of cycles at which we latch the output data and start shifting it out.
assign clk_cnt_for_latching_c_data =                        
                          (27);  

//Number of cycles at which we finish shifting inputs into the matmul.
//Note that while this shifting is happening, the matmul is calculating
//outputs in its PEs.
assign clk_cnt_for_shifting_inputs =                        
                          (1);  //Ideally this should have been 7, but if we keep this as
                                //7, then stall signal is asserted a bit later than required

always @(posedge clk) begin
  if (reset) begin
    clk_cnt <= 0;
    done_mat_mul <= 0;
    in_progress <= 0;
    shift_out_data <= 0;
    shift_in_data <= 0;
  end
  else if (start_pulse == 1'b1) begin
    clk_cnt <= clk_cnt + 1;
    done_mat_mul <= 0;
    in_progress <= 0;
    shift_out_data <= 0;
    shift_in_data <= 1;
  end
  else if (clk_cnt == clk_cnt_for_shifting_inputs) begin 
    clk_cnt <= clk_cnt + 1;
    done_mat_mul <= 0;
    in_progress <= 1;
    shift_out_data <= 0;
    shift_in_data <= 0;
  end
  else if (clk_cnt == clk_cnt_for_latching_c_data) begin 
    clk_cnt <= clk_cnt + 1;
    done_mat_mul <= 0;
    in_progress <= 0;
    shift_out_data <= 1;
    shift_in_data <= 0;
  end
  else if (clk_cnt == clk_cnt_for_done) begin
    done_mat_mul <= 1;
    clk_cnt <= clk_cnt + 1;
    in_progress <= 0;
    shift_out_data <= 0;
    shift_in_data <= 0;
  end
  else if ((in_progress == 1) || (shift_out_data == 1) || (shift_in_data == 1)) begin
    clk_cnt <= clk_cnt + 1;
  end    
  else begin
    clk_cnt <= 0;
    done_mat_mul <= 0;
    in_progress <= 0;
  end
end

wire [`DWIDTH-1:0] a0_data;
wire [`DWIDTH-1:0] a1_data;
wire [`DWIDTH-1:0] a2_data;
wire [`DWIDTH-1:0] a3_data;
wire [`DWIDTH-1:0] a4_data;
wire [`DWIDTH-1:0] a5_data;
wire [`DWIDTH-1:0] a6_data;
wire [`DWIDTH-1:0] a7_data;
wire [`DWIDTH-1:0] b0_data;
wire [`DWIDTH-1:0] b1_data;
wire [`DWIDTH-1:0] b2_data;
wire [`DWIDTH-1:0] b3_data;
wire [`DWIDTH-1:0] b4_data;
wire [`DWIDTH-1:0] b5_data;
wire [`DWIDTH-1:0] b6_data;
wire [`DWIDTH-1:0] b7_data;
wire [`DWIDTH-1:0] a1_data_delayed_1;
wire [`DWIDTH-1:0] a2_data_delayed_1;
wire [`DWIDTH-1:0] a2_data_delayed_2;
wire [`DWIDTH-1:0] a3_data_delayed_1;
wire [`DWIDTH-1:0] a3_data_delayed_2;
wire [`DWIDTH-1:0] a3_data_delayed_3;
wire [`DWIDTH-1:0] a4_data_delayed_1;
wire [`DWIDTH-1:0] a4_data_delayed_2;
wire [`DWIDTH-1:0] a4_data_delayed_3;
wire [`DWIDTH-1:0] a4_data_delayed_4;
wire [`DWIDTH-1:0] a5_data_delayed_1;
wire [`DWIDTH-1:0] a5_data_delayed_2;
wire [`DWIDTH-1:0] a5_data_delayed_3;
wire [`DWIDTH-1:0] a5_data_delayed_4;
wire [`DWIDTH-1:0] a5_data_delayed_5;
wire [`DWIDTH-1:0] a6_data_delayed_1;
wire [`DWIDTH-1:0] a6_data_delayed_2;
wire [`DWIDTH-1:0] a6_data_delayed_3;
wire [`DWIDTH-1:0] a6_data_delayed_4;
wire [`DWIDTH-1:0] a6_data_delayed_5;
wire [`DWIDTH-1:0] a6_data_delayed_6;
wire [`DWIDTH-1:0] a7_data_delayed_1;
wire [`DWIDTH-1:0] a7_data_delayed_2;
wire [`DWIDTH-1:0] a7_data_delayed_3;
wire [`DWIDTH-1:0] a7_data_delayed_4;
wire [`DWIDTH-1:0] a7_data_delayed_5;
wire [`DWIDTH-1:0] a7_data_delayed_6;
wire [`DWIDTH-1:0] a7_data_delayed_7;
wire [`DWIDTH-1:0] b1_data_delayed_1;
wire [`DWIDTH-1:0] b2_data_delayed_1;
wire [`DWIDTH-1:0] b2_data_delayed_2;
wire [`DWIDTH-1:0] b3_data_delayed_1;
wire [`DWIDTH-1:0] b3_data_delayed_2;
wire [`DWIDTH-1:0] b3_data_delayed_3;
wire [`DWIDTH-1:0] b4_data_delayed_1;
wire [`DWIDTH-1:0] b4_data_delayed_2;
wire [`DWIDTH-1:0] b4_data_delayed_3;
wire [`DWIDTH-1:0] b4_data_delayed_4;
wire [`DWIDTH-1:0] b5_data_delayed_1;
wire [`DWIDTH-1:0] b5_data_delayed_2;
wire [`DWIDTH-1:0] b5_data_delayed_3;
wire [`DWIDTH-1:0] b5_data_delayed_4;
wire [`DWIDTH-1:0] b5_data_delayed_5;
wire [`DWIDTH-1:0] b6_data_delayed_1;
wire [`DWIDTH-1:0] b6_data_delayed_2;
wire [`DWIDTH-1:0] b6_data_delayed_3;
wire [`DWIDTH-1:0] b6_data_delayed_4;
wire [`DWIDTH-1:0] b6_data_delayed_5;
wire [`DWIDTH-1:0] b6_data_delayed_6;
wire [`DWIDTH-1:0] b7_data_delayed_1;
wire [`DWIDTH-1:0] b7_data_delayed_2;
wire [`DWIDTH-1:0] b7_data_delayed_3;
wire [`DWIDTH-1:0] b7_data_delayed_4;
wire [`DWIDTH-1:0] b7_data_delayed_5;
wire [`DWIDTH-1:0] b7_data_delayed_6;
wire [`DWIDTH-1:0] b7_data_delayed_7;


//////////////////////////////////////////////////////////////////////////
// Instantiation of systolic data setup
//////////////////////////////////////////////////////////////////////////
systolic_data_setup u_systolic_data_setup(
.clk(clk),
.reset(reset),
.matmul_op_in_progress(matmul_op_in_progress),
.a_data(a_data),
.b_data(b_data),
.clk_cnt(clk_cnt),
.a0_data(a0_data),
.b0_data(b0_data),
.a1_data_delayed_1(a1_data_delayed_1),
.b1_data_delayed_1(b1_data_delayed_1),
.a2_data_delayed_2(a2_data_delayed_2),
.b2_data_delayed_2(b2_data_delayed_2),
.a3_data_delayed_3(a3_data_delayed_3),
.b3_data_delayed_3(b3_data_delayed_3),
.a4_data_delayed_4(a4_data_delayed_4),
.b4_data_delayed_4(b4_data_delayed_4),
.a5_data_delayed_5(a5_data_delayed_5),
.b5_data_delayed_5(b5_data_delayed_5),
.a6_data_delayed_6(a6_data_delayed_6),
.b6_data_delayed_6(b6_data_delayed_6),
.a7_data_delayed_7(a7_data_delayed_7),
.b7_data_delayed_7(b7_data_delayed_7),

.validity_mask_a_rows(validity_mask_a_rows),
.validity_mask_a_cols(validity_mask_a_cols),
.validity_mask_b_rows(validity_mask_b_rows),
.validity_mask_b_cols(validity_mask_b_cols)

);

wire [`DWIDTH-1:0] a0;
wire [`DWIDTH-1:0] a1;
wire [`DWIDTH-1:0] a2;
wire [`DWIDTH-1:0] a3;
wire [`DWIDTH-1:0] a4;
wire [`DWIDTH-1:0] a5;
wire [`DWIDTH-1:0] a6;
wire [`DWIDTH-1:0] a7;
wire [`DWIDTH-1:0] b0;
wire [`DWIDTH-1:0] b1;
wire [`DWIDTH-1:0] b2;
wire [`DWIDTH-1:0] b3;
wire [`DWIDTH-1:0] b4;
wire [`DWIDTH-1:0] b5;
wire [`DWIDTH-1:0] b6;
wire [`DWIDTH-1:0] b7;

assign a0 = a0_data          ;
assign a1 = a1_data_delayed_1;
assign a2 = a2_data_delayed_2;
assign a3 = a3_data_delayed_3;
assign a4 = a4_data_delayed_4;
assign a5 = a5_data_delayed_5;
assign a6 = a6_data_delayed_6;
assign a7 = a7_data_delayed_7;

assign b0 = b0_data          ;
assign b1 = b1_data_delayed_1;
assign b2 = b2_data_delayed_2;
assign b3 = b3_data_delayed_3;
assign b4 = b4_data_delayed_4;
assign b5 = b5_data_delayed_5;
assign b6 = b6_data_delayed_6;
assign b7 = b7_data_delayed_7;

wire [`DWIDTH-1:0] matrixC0_0;
wire [`DWIDTH-1:0] matrixC0_1;
wire [`DWIDTH-1:0] matrixC0_2;
wire [`DWIDTH-1:0] matrixC0_3;
wire [`DWIDTH-1:0] matrixC0_4;
wire [`DWIDTH-1:0] matrixC0_5;
wire [`DWIDTH-1:0] matrixC0_6;
wire [`DWIDTH-1:0] matrixC0_7;
wire [`DWIDTH-1:0] matrixC1_0;
wire [`DWIDTH-1:0] matrixC1_1;
wire [`DWIDTH-1:0] matrixC1_2;
wire [`DWIDTH-1:0] matrixC1_3;
wire [`DWIDTH-1:0] matrixC1_4;
wire [`DWIDTH-1:0] matrixC1_5;
wire [`DWIDTH-1:0] matrixC1_6;
wire [`DWIDTH-1:0] matrixC1_7;
wire [`DWIDTH-1:0] matrixC2_0;
wire [`DWIDTH-1:0] matrixC2_1;
wire [`DWIDTH-1:0] matrixC2_2;
wire [`DWIDTH-1:0] matrixC2_3;
wire [`DWIDTH-1:0] matrixC2_4;
wire [`DWIDTH-1:0] matrixC2_5;
wire [`DWIDTH-1:0] matrixC2_6;
wire [`DWIDTH-1:0] matrixC2_7;
wire [`DWIDTH-1:0] matrixC3_0;
wire [`DWIDTH-1:0] matrixC3_1;
wire [`DWIDTH-1:0] matrixC3_2;
wire [`DWIDTH-1:0] matrixC3_3;
wire [`DWIDTH-1:0] matrixC3_4;
wire [`DWIDTH-1:0] matrixC3_5;
wire [`DWIDTH-1:0] matrixC3_6;
wire [`DWIDTH-1:0] matrixC3_7;
wire [`DWIDTH-1:0] matrixC4_0;
wire [`DWIDTH-1:0] matrixC4_1;
wire [`DWIDTH-1:0] matrixC4_2;
wire [`DWIDTH-1:0] matrixC4_3;
wire [`DWIDTH-1:0] matrixC4_4;
wire [`DWIDTH-1:0] matrixC4_5;
wire [`DWIDTH-1:0] matrixC4_6;
wire [`DWIDTH-1:0] matrixC4_7;
wire [`DWIDTH-1:0] matrixC5_0;
wire [`DWIDTH-1:0] matrixC5_1;
wire [`DWIDTH-1:0] matrixC5_2;
wire [`DWIDTH-1:0] matrixC5_3;
wire [`DWIDTH-1:0] matrixC5_4;
wire [`DWIDTH-1:0] matrixC5_5;
wire [`DWIDTH-1:0] matrixC5_6;
wire [`DWIDTH-1:0] matrixC5_7;
wire [`DWIDTH-1:0] matrixC6_0;
wire [`DWIDTH-1:0] matrixC6_1;
wire [`DWIDTH-1:0] matrixC6_2;
wire [`DWIDTH-1:0] matrixC6_3;
wire [`DWIDTH-1:0] matrixC6_4;
wire [`DWIDTH-1:0] matrixC6_5;
wire [`DWIDTH-1:0] matrixC6_6;
wire [`DWIDTH-1:0] matrixC6_7;
wire [`DWIDTH-1:0] matrixC7_0;
wire [`DWIDTH-1:0] matrixC7_1;
wire [`DWIDTH-1:0] matrixC7_2;
wire [`DWIDTH-1:0] matrixC7_3;
wire [`DWIDTH-1:0] matrixC7_4;
wire [`DWIDTH-1:0] matrixC7_5;
wire [`DWIDTH-1:0] matrixC7_6;
wire [`DWIDTH-1:0] matrixC7_7;

wire row_latch_en;
//////////////////////////////////////////////////////////////////////////
// Instantiation of the output logic
//////////////////////////////////////////////////////////////////////////
output_logic u_output_logic(
.matmul_op_in_progress(matmul_op_in_progress),
.done_mat_mul(done_mat_mul),
.c_data_out(c_data_out),
.c_data_available(c_data_available),
.clk_cnt(clk_cnt),
.row_latch_en(row_latch_en),
.matrixC0_0(matrixC0_0),
.matrixC0_1(matrixC0_1),
.matrixC0_2(matrixC0_2),
.matrixC0_3(matrixC0_3),
.matrixC0_4(matrixC0_4),
.matrixC0_5(matrixC0_5),
.matrixC0_6(matrixC0_6),
.matrixC0_7(matrixC0_7),
.matrixC1_0(matrixC1_0),
.matrixC1_1(matrixC1_1),
.matrixC1_2(matrixC1_2),
.matrixC1_3(matrixC1_3),
.matrixC1_4(matrixC1_4),
.matrixC1_5(matrixC1_5),
.matrixC1_6(matrixC1_6),
.matrixC1_7(matrixC1_7),
.matrixC2_0(matrixC2_0),
.matrixC2_1(matrixC2_1),
.matrixC2_2(matrixC2_2),
.matrixC2_3(matrixC2_3),
.matrixC2_4(matrixC2_4),
.matrixC2_5(matrixC2_5),
.matrixC2_6(matrixC2_6),
.matrixC2_7(matrixC2_7),
.matrixC3_0(matrixC3_0),
.matrixC3_1(matrixC3_1),
.matrixC3_2(matrixC3_2),
.matrixC3_3(matrixC3_3),
.matrixC3_4(matrixC3_4),
.matrixC3_5(matrixC3_5),
.matrixC3_6(matrixC3_6),
.matrixC3_7(matrixC3_7),
.matrixC4_0(matrixC4_0),
.matrixC4_1(matrixC4_1),
.matrixC4_2(matrixC4_2),
.matrixC4_3(matrixC4_3),
.matrixC4_4(matrixC4_4),
.matrixC4_5(matrixC4_5),
.matrixC4_6(matrixC4_6),
.matrixC4_7(matrixC4_7),
.matrixC5_0(matrixC5_0),
.matrixC5_1(matrixC5_1),
.matrixC5_2(matrixC5_2),
.matrixC5_3(matrixC5_3),
.matrixC5_4(matrixC5_4),
.matrixC5_5(matrixC5_5),
.matrixC5_6(matrixC5_6),
.matrixC5_7(matrixC5_7),
.matrixC6_0(matrixC6_0),
.matrixC6_1(matrixC6_1),
.matrixC6_2(matrixC6_2),
.matrixC6_3(matrixC6_3),
.matrixC6_4(matrixC6_4),
.matrixC6_5(matrixC6_5),
.matrixC6_6(matrixC6_6),
.matrixC6_7(matrixC6_7),
.matrixC7_0(matrixC7_0),
.matrixC7_1(matrixC7_1),
.matrixC7_2(matrixC7_2),
.matrixC7_3(matrixC7_3),
.matrixC7_4(matrixC7_4),
.matrixC7_5(matrixC7_5),
.matrixC7_6(matrixC7_6),
.matrixC7_7(matrixC7_7),

.clk(clk),
.reset(reset)
);

//////////////////////////////////////////////////////////////////////////
// Instantiations of the actual PEs
//////////////////////////////////////////////////////////////////////////
systolic_pe_matrix u_systolic_pe_matrix(
.clk(clk),
.reset(reset),
.pe_reset(pe_reset),
.a0(a0),
.a1(a1),
.a2(a2),
.a3(a3),
.a4(a4),
.a5(a5),
.a6(a6),
.a7(a7),
.b0(b0),
.b1(b1),
.b2(b2),
.b3(b3),
.b4(b4),
.b5(b5),
.b6(b6),
.b7(b7),
.matrixC0_0(matrixC0_0),
.matrixC0_1(matrixC0_1),
.matrixC0_2(matrixC0_2),
.matrixC0_3(matrixC0_3),
.matrixC0_4(matrixC0_4),
.matrixC0_5(matrixC0_5),
.matrixC0_6(matrixC0_6),
.matrixC0_7(matrixC0_7),
.matrixC1_0(matrixC1_0),
.matrixC1_1(matrixC1_1),
.matrixC1_2(matrixC1_2),
.matrixC1_3(matrixC1_3),
.matrixC1_4(matrixC1_4),
.matrixC1_5(matrixC1_5),
.matrixC1_6(matrixC1_6),
.matrixC1_7(matrixC1_7),
.matrixC2_0(matrixC2_0),
.matrixC2_1(matrixC2_1),
.matrixC2_2(matrixC2_2),
.matrixC2_3(matrixC2_3),
.matrixC2_4(matrixC2_4),
.matrixC2_5(matrixC2_5),
.matrixC2_6(matrixC2_6),
.matrixC2_7(matrixC2_7),
.matrixC3_0(matrixC3_0),
.matrixC3_1(matrixC3_1),
.matrixC3_2(matrixC3_2),
.matrixC3_3(matrixC3_3),
.matrixC3_4(matrixC3_4),
.matrixC3_5(matrixC3_5),
.matrixC3_6(matrixC3_6),
.matrixC3_7(matrixC3_7),
.matrixC4_0(matrixC4_0),
.matrixC4_1(matrixC4_1),
.matrixC4_2(matrixC4_2),
.matrixC4_3(matrixC4_3),
.matrixC4_4(matrixC4_4),
.matrixC4_5(matrixC4_5),
.matrixC4_6(matrixC4_6),
.matrixC4_7(matrixC4_7),
.matrixC5_0(matrixC5_0),
.matrixC5_1(matrixC5_1),
.matrixC5_2(matrixC5_2),
.matrixC5_3(matrixC5_3),
.matrixC5_4(matrixC5_4),
.matrixC5_5(matrixC5_5),
.matrixC5_6(matrixC5_6),
.matrixC5_7(matrixC5_7),
.matrixC6_0(matrixC6_0),
.matrixC6_1(matrixC6_1),
.matrixC6_2(matrixC6_2),
.matrixC6_3(matrixC6_3),
.matrixC6_4(matrixC6_4),
.matrixC6_5(matrixC6_5),
.matrixC6_6(matrixC6_6),
.matrixC6_7(matrixC6_7),
.matrixC7_0(matrixC7_0),
.matrixC7_1(matrixC7_1),
.matrixC7_2(matrixC7_2),
.matrixC7_3(matrixC7_3),
.matrixC7_4(matrixC7_4),
.matrixC7_5(matrixC7_5),
.matrixC7_6(matrixC7_6),
.matrixC7_7(matrixC7_7),

.a_data_out(a_data_out),
.b_data_out(b_data_out)
);

endmodule


//////////////////////////////////////////////////////////////////////////
// Output logic
//////////////////////////////////////////////////////////////////////////
module output_logic(
matmul_op_in_progress,
done_mat_mul,
c_data_out,
c_data_available,
clk_cnt,
row_latch_en,
matrixC0_0,
matrixC0_1,
matrixC0_2,
matrixC0_3,
matrixC0_4,
matrixC0_5,
matrixC0_6,
matrixC0_7,
matrixC1_0,
matrixC1_1,
matrixC1_2,
matrixC1_3,
matrixC1_4,
matrixC1_5,
matrixC1_6,
matrixC1_7,
matrixC2_0,
matrixC2_1,
matrixC2_2,
matrixC2_3,
matrixC2_4,
matrixC2_5,
matrixC2_6,
matrixC2_7,
matrixC3_0,
matrixC3_1,
matrixC3_2,
matrixC3_3,
matrixC3_4,
matrixC3_5,
matrixC3_6,
matrixC3_7,
matrixC4_0,
matrixC4_1,
matrixC4_2,
matrixC4_3,
matrixC4_4,
matrixC4_5,
matrixC4_6,
matrixC4_7,
matrixC5_0,
matrixC5_1,
matrixC5_2,
matrixC5_3,
matrixC5_4,
matrixC5_5,
matrixC5_6,
matrixC5_7,
matrixC6_0,
matrixC6_1,
matrixC6_2,
matrixC6_3,
matrixC6_4,
matrixC6_5,
matrixC6_6,
matrixC6_7,
matrixC7_0,
matrixC7_1,
matrixC7_2,
matrixC7_3,
matrixC7_4,
matrixC7_5,
matrixC7_6,
matrixC7_7,

clk,
reset
);

input clk;
input reset;
input matmul_op_in_progress;
input done_mat_mul;
output [`MAT_MUL_SIZE*`DWIDTH-1:0] c_data_out;
output c_data_available;
input [7:0] clk_cnt;
output row_latch_en;
input [`DWIDTH-1:0] matrixC0_0;
input [`DWIDTH-1:0] matrixC0_1;
input [`DWIDTH-1:0] matrixC0_2;
input [`DWIDTH-1:0] matrixC0_3;
input [`DWIDTH-1:0] matrixC0_4;
input [`DWIDTH-1:0] matrixC0_5;
input [`DWIDTH-1:0] matrixC0_6;
input [`DWIDTH-1:0] matrixC0_7;
input [`DWIDTH-1:0] matrixC1_0;
input [`DWIDTH-1:0] matrixC1_1;
input [`DWIDTH-1:0] matrixC1_2;
input [`DWIDTH-1:0] matrixC1_3;
input [`DWIDTH-1:0] matrixC1_4;
input [`DWIDTH-1:0] matrixC1_5;
input [`DWIDTH-1:0] matrixC1_6;
input [`DWIDTH-1:0] matrixC1_7;
input [`DWIDTH-1:0] matrixC2_0;
input [`DWIDTH-1:0] matrixC2_1;
input [`DWIDTH-1:0] matrixC2_2;
input [`DWIDTH-1:0] matrixC2_3;
input [`DWIDTH-1:0] matrixC2_4;
input [`DWIDTH-1:0] matrixC2_5;
input [`DWIDTH-1:0] matrixC2_6;
input [`DWIDTH-1:0] matrixC2_7;
input [`DWIDTH-1:0] matrixC3_0;
input [`DWIDTH-1:0] matrixC3_1;
input [`DWIDTH-1:0] matrixC3_2;
input [`DWIDTH-1:0] matrixC3_3;
input [`DWIDTH-1:0] matrixC3_4;
input [`DWIDTH-1:0] matrixC3_5;
input [`DWIDTH-1:0] matrixC3_6;
input [`DWIDTH-1:0] matrixC3_7;
input [`DWIDTH-1:0] matrixC4_0;
input [`DWIDTH-1:0] matrixC4_1;
input [`DWIDTH-1:0] matrixC4_2;
input [`DWIDTH-1:0] matrixC4_3;
input [`DWIDTH-1:0] matrixC4_4;
input [`DWIDTH-1:0] matrixC4_5;
input [`DWIDTH-1:0] matrixC4_6;
input [`DWIDTH-1:0] matrixC4_7;
input [`DWIDTH-1:0] matrixC5_0;
input [`DWIDTH-1:0] matrixC5_1;
input [`DWIDTH-1:0] matrixC5_2;
input [`DWIDTH-1:0] matrixC5_3;
input [`DWIDTH-1:0] matrixC5_4;
input [`DWIDTH-1:0] matrixC5_5;
input [`DWIDTH-1:0] matrixC5_6;
input [`DWIDTH-1:0] matrixC5_7;
input [`DWIDTH-1:0] matrixC6_0;
input [`DWIDTH-1:0] matrixC6_1;
input [`DWIDTH-1:0] matrixC6_2;
input [`DWIDTH-1:0] matrixC6_3;
input [`DWIDTH-1:0] matrixC6_4;
input [`DWIDTH-1:0] matrixC6_5;
input [`DWIDTH-1:0] matrixC6_6;
input [`DWIDTH-1:0] matrixC6_7;
input [`DWIDTH-1:0] matrixC7_0;
input [`DWIDTH-1:0] matrixC7_1;
input [`DWIDTH-1:0] matrixC7_2;
input [`DWIDTH-1:0] matrixC7_3;
input [`DWIDTH-1:0] matrixC7_4;
input [`DWIDTH-1:0] matrixC7_5;
input [`DWIDTH-1:0] matrixC7_6;
input [`DWIDTH-1:0] matrixC7_7;
wire row_latch_en;


//////////////////////////////////////////////////////////////////////////
// Logic to capture matrix C data from the PEs and shift it out
//////////////////////////////////////////////////////////////////////////
//assign row_latch_en = (clk_cnt==(`MAT_MUL_SIZE + (a_loc+b_loc) * `BB_MAT_MUL_SIZE + 10 +  `NUM_CYCLES_IN_MAC - 1));
//Writing the line above to avoid multiplication:
//assign row_latch_en = (clk_cnt==(`MAT_MUL_SIZE + ((a_loc+b_loc) << `LOG2_MAT_MUL_SIZE) + 10 +  `NUM_CYCLES_IN_MAC - 1));

assign row_latch_en =  
                       ((clk_cnt == 27 ));
    
reg c_data_available;
reg start_capturing_c_data;
integer counter;
reg [8*`DWIDTH-1:0] c_data_out;
reg [8*`DWIDTH-1:0] c_data_out_1;
reg [8*`DWIDTH-1:0] c_data_out_2;
reg [8*`DWIDTH-1:0] c_data_out_3;
reg [8*`DWIDTH-1:0] c_data_out_4;
reg [8*`DWIDTH-1:0] c_data_out_5;
reg [8*`DWIDTH-1:0] c_data_out_6;
reg [8*`DWIDTH-1:0] c_data_out_7;
wire condition_to_start_shifting_output;
assign condition_to_start_shifting_output = 
                          row_latch_en ;  

  
//For larger matmuls, this logic will have more entries in the case statement
always @(posedge clk) begin
  if (reset | ~matmul_op_in_progress) begin
    start_capturing_c_data <= 1'b0;
    c_data_available <= 1'b0;
    c_data_out <= 0;
    counter <= 0;

    c_data_out_1 <= 0;
    c_data_out_2 <= 0;
    c_data_out_3 <= 0;
    c_data_out_4 <= 0;
    c_data_out_5 <= 0;
    c_data_out_6 <= 0;
    c_data_out_7 <= 0;
  end else if (condition_to_start_shifting_output) begin
    start_capturing_c_data <= 1'b1;
    c_data_available <= 1'b1;
    c_data_out <= {matrixC7_7, matrixC6_7, matrixC5_7, matrixC4_7, matrixC3_7, matrixC2_7, matrixC1_7, matrixC0_7};
      c_data_out_1 <= {matrixC7_6, matrixC6_6, matrixC5_6, matrixC4_6, matrixC3_6, matrixC2_6, matrixC1_6, matrixC0_6};
      c_data_out_2 <= {matrixC7_5, matrixC6_5, matrixC5_5, matrixC4_5, matrixC3_5, matrixC2_5, matrixC1_5, matrixC0_5};
      c_data_out_3 <= {matrixC7_4, matrixC6_4, matrixC5_4, matrixC4_4, matrixC3_4, matrixC2_4, matrixC1_4, matrixC0_4};
      c_data_out_4 <= {matrixC7_3, matrixC6_3, matrixC5_3, matrixC4_3, matrixC3_3, matrixC2_3, matrixC1_3, matrixC0_3};
      c_data_out_5 <= {matrixC7_2, matrixC6_2, matrixC5_2, matrixC4_2, matrixC3_2, matrixC2_2, matrixC1_2, matrixC0_2};
      c_data_out_6 <= {matrixC7_1, matrixC6_1, matrixC5_1, matrixC4_1, matrixC3_1, matrixC2_1, matrixC1_1, matrixC0_1};
      c_data_out_7 <= {matrixC7_0, matrixC6_0, matrixC5_0, matrixC4_0, matrixC3_0, matrixC2_0, matrixC1_0, matrixC0_0};

    counter <= counter + 1;
  end else if (done_mat_mul) begin
    start_capturing_c_data <= 1'b0;
    c_data_available <= 1'b0;
    c_data_out <= 0;

    c_data_out_1 <= 0;
    c_data_out_2 <= 0;
    c_data_out_3 <= 0;
    c_data_out_4 <= 0;
    c_data_out_5 <= 0;
    c_data_out_6 <= 0;
    c_data_out_7 <= 0;
  end 
  else if (counter >= `MAT_MUL_SIZE) begin
    c_data_out <= c_data_out_1;

    c_data_out_1 <= c_data_out_2;
    c_data_out_2 <= c_data_out_3;
    c_data_out_3 <= c_data_out_4;
    c_data_out_4 <= c_data_out_5;
    c_data_out_5 <= c_data_out_6;
    c_data_out_6 <= c_data_out_7;
    c_data_out_7 <= 0;
  end
  else if (start_capturing_c_data) begin
    c_data_available <= 1'b1;
    counter <= counter + 1;
    c_data_out <= c_data_out_1;

    c_data_out_1 <= c_data_out_2;
    c_data_out_2 <= c_data_out_3;
    c_data_out_3 <= c_data_out_4;
    c_data_out_4 <= c_data_out_5;
    c_data_out_5 <= c_data_out_6;
    c_data_out_6 <= c_data_out_7;
    c_data_out_7 <= 0;
  end
end

endmodule


//////////////////////////////////////////////////////////////////////////
// Systolic data setup
//////////////////////////////////////////////////////////////////////////
module systolic_data_setup(
clk,
reset,
matmul_op_in_progress,
a_data,
b_data,
clk_cnt,
a0_data,
b0_data,
a1_data_delayed_1,
b1_data_delayed_1,
a2_data_delayed_2,
b2_data_delayed_2,
a3_data_delayed_3,
b3_data_delayed_3,
a4_data_delayed_4,
b4_data_delayed_4,
a5_data_delayed_5,
b5_data_delayed_5,
a6_data_delayed_6,
b6_data_delayed_6,
a7_data_delayed_7,
b7_data_delayed_7,

validity_mask_a_rows,
validity_mask_a_cols,
validity_mask_b_rows,
validity_mask_b_cols
);

input clk;
input reset;
input matmul_op_in_progress;
input [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data;
input [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data;
input [7:0] clk_cnt;
output [`DWIDTH-1:0] a0_data;
output [`DWIDTH-1:0] b0_data;
output [`DWIDTH-1:0] a1_data_delayed_1;
output [`DWIDTH-1:0] b1_data_delayed_1;
output [`DWIDTH-1:0] a2_data_delayed_2;
output [`DWIDTH-1:0] b2_data_delayed_2;
output [`DWIDTH-1:0] a3_data_delayed_3;
output [`DWIDTH-1:0] b3_data_delayed_3;
output [`DWIDTH-1:0] a4_data_delayed_4;
output [`DWIDTH-1:0] b4_data_delayed_4;
output [`DWIDTH-1:0] a5_data_delayed_5;
output [`DWIDTH-1:0] b5_data_delayed_5;
output [`DWIDTH-1:0] a6_data_delayed_6;
output [`DWIDTH-1:0] b6_data_delayed_6;
output [`DWIDTH-1:0] a7_data_delayed_7;
output [`DWIDTH-1:0] b7_data_delayed_7;

input [`MASK_WIDTH-1:0] validity_mask_a_rows;
input [`MASK_WIDTH-1:0] validity_mask_a_cols;
input [`MASK_WIDTH-1:0] validity_mask_b_rows;
input [`MASK_WIDTH-1:0] validity_mask_b_cols;

wire [`DWIDTH-1:0] a0_data;
wire [`DWIDTH-1:0] a1_data;
wire [`DWIDTH-1:0] a2_data;
wire [`DWIDTH-1:0] a3_data;
wire [`DWIDTH-1:0] a4_data;
wire [`DWIDTH-1:0] a5_data;
wire [`DWIDTH-1:0] a6_data;
wire [`DWIDTH-1:0] a7_data;
wire [`DWIDTH-1:0] b0_data;
wire [`DWIDTH-1:0] b1_data;
wire [`DWIDTH-1:0] b2_data;
wire [`DWIDTH-1:0] b3_data;
wire [`DWIDTH-1:0] b4_data;
wire [`DWIDTH-1:0] b5_data;
wire [`DWIDTH-1:0] b6_data;
wire [`DWIDTH-1:0] b7_data;

wire [7:0] a_mem_access_counter;
assign a_mem_access_counter = ((clk_cnt>=8) ? 0 : (matmul_op_in_progress ? (clk_cnt+1) : 0));

wire a_data_valid; //flag that tells whether the data from memory is valid
assign a_data_valid = 
     ((validity_mask_a_cols[0]==1'b0 && a_mem_access_counter==1) ||
      (validity_mask_a_cols[1]==1'b0 && a_mem_access_counter==2) ||
      (validity_mask_a_cols[2]==1'b0 && a_mem_access_counter==3) ||
      (validity_mask_a_cols[3]==1'b0 && a_mem_access_counter==4) ||
      (validity_mask_a_cols[4]==1'b0 && a_mem_access_counter==5) ||
      (validity_mask_a_cols[5]==1'b0 && a_mem_access_counter==6) ||
      (validity_mask_a_cols[6]==1'b0 && a_mem_access_counter==7) ||
      (validity_mask_a_cols[7]==1'b0 && a_mem_access_counter==8)) ?
    
    1'b0 : (a_mem_access_counter >= `MEM_ACCESS_LATENCY);

//////////////////////////////////////////////////////////////////////////
// Logic to delay certain parts of the data received from BRAM A (systolic data setup)
//////////////////////////////////////////////////////////////////////////
assign a0_data = a_data[1*`DWIDTH-1:0*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[0]}};
assign a1_data = a_data[2*`DWIDTH-1:1*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[1]}};
assign a2_data = a_data[3*`DWIDTH-1:2*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[2]}};
assign a3_data = a_data[4*`DWIDTH-1:3*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[3]}};
assign a4_data = a_data[5*`DWIDTH-1:4*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[4]}};
assign a5_data = a_data[6*`DWIDTH-1:5*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[5]}};
assign a6_data = a_data[7*`DWIDTH-1:6*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[6]}};
assign a7_data = a_data[8*`DWIDTH-1:7*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[7]}};

reg [`DWIDTH-1:0] a1_data_delayed_1;
reg [`DWIDTH-1:0] a2_data_delayed_1;
reg [`DWIDTH-1:0] a2_data_delayed_2;
reg [`DWIDTH-1:0] a3_data_delayed_1;
reg [`DWIDTH-1:0] a3_data_delayed_2;
reg [`DWIDTH-1:0] a3_data_delayed_3;
reg [`DWIDTH-1:0] a4_data_delayed_1;
reg [`DWIDTH-1:0] a4_data_delayed_2;
reg [`DWIDTH-1:0] a4_data_delayed_3;
reg [`DWIDTH-1:0] a4_data_delayed_4;
reg [`DWIDTH-1:0] a5_data_delayed_1;
reg [`DWIDTH-1:0] a5_data_delayed_2;
reg [`DWIDTH-1:0] a5_data_delayed_3;
reg [`DWIDTH-1:0] a5_data_delayed_4;
reg [`DWIDTH-1:0] a5_data_delayed_5;
reg [`DWIDTH-1:0] a6_data_delayed_1;
reg [`DWIDTH-1:0] a6_data_delayed_2;
reg [`DWIDTH-1:0] a6_data_delayed_3;
reg [`DWIDTH-1:0] a6_data_delayed_4;
reg [`DWIDTH-1:0] a6_data_delayed_5;
reg [`DWIDTH-1:0] a6_data_delayed_6;
reg [`DWIDTH-1:0] a7_data_delayed_1;
reg [`DWIDTH-1:0] a7_data_delayed_2;
reg [`DWIDTH-1:0] a7_data_delayed_3;
reg [`DWIDTH-1:0] a7_data_delayed_4;
reg [`DWIDTH-1:0] a7_data_delayed_5;
reg [`DWIDTH-1:0] a7_data_delayed_6;
reg [`DWIDTH-1:0] a7_data_delayed_7;


always @(posedge clk) begin
  if (reset || ~matmul_op_in_progress) begin
    a1_data_delayed_1 <= 0;
    a2_data_delayed_1 <= 0;
    a2_data_delayed_2 <= 0;
    a3_data_delayed_1 <= 0;
    a3_data_delayed_2 <= 0;
    a3_data_delayed_3 <= 0;
    a4_data_delayed_1 <= 0;
    a4_data_delayed_2 <= 0;
    a4_data_delayed_3 <= 0;
    a4_data_delayed_4 <= 0;
    a5_data_delayed_1 <= 0;
    a5_data_delayed_2 <= 0;
    a5_data_delayed_3 <= 0;
    a5_data_delayed_4 <= 0;
    a5_data_delayed_5 <= 0;
    a6_data_delayed_1 <= 0;
    a6_data_delayed_2 <= 0;
    a6_data_delayed_3 <= 0;
    a6_data_delayed_4 <= 0;
    a6_data_delayed_5 <= 0;
    a6_data_delayed_6 <= 0;
    a7_data_delayed_1 <= 0;
    a7_data_delayed_2 <= 0;
    a7_data_delayed_3 <= 0;
    a7_data_delayed_4 <= 0;
    a7_data_delayed_5 <= 0;
    a7_data_delayed_6 <= 0;
    a7_data_delayed_7 <= 0;

  end
  else begin
  a1_data_delayed_1 <= a1_data;
  a2_data_delayed_1 <= a2_data;
  a3_data_delayed_1 <= a3_data;
  a4_data_delayed_1 <= a4_data;
  a5_data_delayed_1 <= a5_data;
  a6_data_delayed_1 <= a6_data;
  a7_data_delayed_1 <= a7_data;
  a2_data_delayed_2 <= a2_data_delayed_1;
  a3_data_delayed_2 <= a3_data_delayed_1;
  a3_data_delayed_3 <= a3_data_delayed_2;
  a4_data_delayed_2 <= a4_data_delayed_1;
  a4_data_delayed_3 <= a4_data_delayed_2;
  a4_data_delayed_4 <= a4_data_delayed_3;
  a5_data_delayed_2 <= a5_data_delayed_1;
  a5_data_delayed_3 <= a5_data_delayed_2;
  a5_data_delayed_4 <= a5_data_delayed_3;
  a5_data_delayed_5 <= a5_data_delayed_4;
  a6_data_delayed_2 <= a6_data_delayed_1;
  a6_data_delayed_3 <= a6_data_delayed_2;
  a6_data_delayed_4 <= a6_data_delayed_3;
  a6_data_delayed_5 <= a6_data_delayed_4;
  a6_data_delayed_6 <= a6_data_delayed_5;
  a7_data_delayed_2 <= a7_data_delayed_1;
  a7_data_delayed_3 <= a7_data_delayed_2;
  a7_data_delayed_4 <= a7_data_delayed_3;
  a7_data_delayed_5 <= a7_data_delayed_4;
  a7_data_delayed_6 <= a7_data_delayed_5;
  a7_data_delayed_7 <= a7_data_delayed_6;
 
  end
end

wire [7:0] b_mem_access_counter;
assign b_mem_access_counter = ((clk_cnt>=8) ? 0 : (matmul_op_in_progress ? (clk_cnt+1) : 0));

wire b_data_valid; //flag that tells whether the data from memory is valid
assign b_data_valid = 
     ((validity_mask_b_rows[0]==1'b0 && b_mem_access_counter==1) ||
      (validity_mask_b_rows[1]==1'b0 && b_mem_access_counter==2) ||
      (validity_mask_b_rows[2]==1'b0 && b_mem_access_counter==3) ||
      (validity_mask_b_rows[3]==1'b0 && b_mem_access_counter==4) ||
      (validity_mask_b_rows[4]==1'b0 && b_mem_access_counter==5) ||
      (validity_mask_b_rows[5]==1'b0 && b_mem_access_counter==6) ||
      (validity_mask_b_rows[6]==1'b0 && b_mem_access_counter==7) ||
      (validity_mask_b_rows[7]==1'b0 && b_mem_access_counter==8)) ?
    
        1'b0 : (b_mem_access_counter >= `MEM_ACCESS_LATENCY);

//////////////////////////////////////////////////////////////////////////
// Logic to delay certain parts of the data received from BRAM B (systolic data setup)
//////////////////////////////////////////////////////////////////////////
assign b0_data = b_data[1*`DWIDTH-1:0*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[0]}};
assign b1_data = b_data[2*`DWIDTH-1:1*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[1]}};
assign b2_data = b_data[3*`DWIDTH-1:2*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[2]}};
assign b3_data = b_data[4*`DWIDTH-1:3*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[3]}};
assign b4_data = b_data[5*`DWIDTH-1:4*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[4]}};
assign b5_data = b_data[6*`DWIDTH-1:5*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[5]}};
assign b6_data = b_data[7*`DWIDTH-1:6*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[6]}};
assign b7_data = b_data[8*`DWIDTH-1:7*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[7]}};

reg [`DWIDTH-1:0] b1_data_delayed_1;
reg [`DWIDTH-1:0] b2_data_delayed_1;
reg [`DWIDTH-1:0] b2_data_delayed_2;
reg [`DWIDTH-1:0] b3_data_delayed_1;
reg [`DWIDTH-1:0] b3_data_delayed_2;
reg [`DWIDTH-1:0] b3_data_delayed_3;
reg [`DWIDTH-1:0] b4_data_delayed_1;
reg [`DWIDTH-1:0] b4_data_delayed_2;
reg [`DWIDTH-1:0] b4_data_delayed_3;
reg [`DWIDTH-1:0] b4_data_delayed_4;
reg [`DWIDTH-1:0] b5_data_delayed_1;
reg [`DWIDTH-1:0] b5_data_delayed_2;
reg [`DWIDTH-1:0] b5_data_delayed_3;
reg [`DWIDTH-1:0] b5_data_delayed_4;
reg [`DWIDTH-1:0] b5_data_delayed_5;
reg [`DWIDTH-1:0] b6_data_delayed_1;
reg [`DWIDTH-1:0] b6_data_delayed_2;
reg [`DWIDTH-1:0] b6_data_delayed_3;
reg [`DWIDTH-1:0] b6_data_delayed_4;
reg [`DWIDTH-1:0] b6_data_delayed_5;
reg [`DWIDTH-1:0] b6_data_delayed_6;
reg [`DWIDTH-1:0] b7_data_delayed_1;
reg [`DWIDTH-1:0] b7_data_delayed_2;
reg [`DWIDTH-1:0] b7_data_delayed_3;
reg [`DWIDTH-1:0] b7_data_delayed_4;
reg [`DWIDTH-1:0] b7_data_delayed_5;
reg [`DWIDTH-1:0] b7_data_delayed_6;
reg [`DWIDTH-1:0] b7_data_delayed_7;


always @(posedge clk) begin
  if (reset || ~matmul_op_in_progress) begin
    b1_data_delayed_1 <= 0;
    b2_data_delayed_1 <= 0;
    b2_data_delayed_2 <= 0;
    b3_data_delayed_1 <= 0;
    b3_data_delayed_2 <= 0;
    b3_data_delayed_3 <= 0;
    b4_data_delayed_1 <= 0;
    b4_data_delayed_2 <= 0;
    b4_data_delayed_3 <= 0;
    b4_data_delayed_4 <= 0;
    b5_data_delayed_1 <= 0;
    b5_data_delayed_2 <= 0;
    b5_data_delayed_3 <= 0;
    b5_data_delayed_4 <= 0;
    b5_data_delayed_5 <= 0;
    b6_data_delayed_1 <= 0;
    b6_data_delayed_2 <= 0;
    b6_data_delayed_3 <= 0;
    b6_data_delayed_4 <= 0;
    b6_data_delayed_5 <= 0;
    b6_data_delayed_6 <= 0;
    b7_data_delayed_1 <= 0;
    b7_data_delayed_2 <= 0;
    b7_data_delayed_3 <= 0;
    b7_data_delayed_4 <= 0;
    b7_data_delayed_5 <= 0;
    b7_data_delayed_6 <= 0;
    b7_data_delayed_7 <= 0;

  end
  else begin
  b1_data_delayed_1 <= b1_data;
  b2_data_delayed_1 <= b2_data;
  b3_data_delayed_1 <= b3_data;
  b4_data_delayed_1 <= b4_data;
  b5_data_delayed_1 <= b5_data;
  b6_data_delayed_1 <= b6_data;
  b7_data_delayed_1 <= b7_data;
  b2_data_delayed_2 <= b2_data_delayed_1;
  b3_data_delayed_2 <= b3_data_delayed_1;
  b3_data_delayed_3 <= b3_data_delayed_2;
  b4_data_delayed_2 <= b4_data_delayed_1;
  b4_data_delayed_3 <= b4_data_delayed_2;
  b4_data_delayed_4 <= b4_data_delayed_3;
  b5_data_delayed_2 <= b5_data_delayed_1;
  b5_data_delayed_3 <= b5_data_delayed_2;
  b5_data_delayed_4 <= b5_data_delayed_3;
  b5_data_delayed_5 <= b5_data_delayed_4;
  b6_data_delayed_2 <= b6_data_delayed_1;
  b6_data_delayed_3 <= b6_data_delayed_2;
  b6_data_delayed_4 <= b6_data_delayed_3;
  b6_data_delayed_5 <= b6_data_delayed_4;
  b6_data_delayed_6 <= b6_data_delayed_5;
  b7_data_delayed_2 <= b7_data_delayed_1;
  b7_data_delayed_3 <= b7_data_delayed_2;
  b7_data_delayed_4 <= b7_data_delayed_3;
  b7_data_delayed_5 <= b7_data_delayed_4;
  b7_data_delayed_6 <= b7_data_delayed_5;
  b7_data_delayed_7 <= b7_data_delayed_6;
 
  end
end
endmodule


//////////////////////////////////////////////////////////////////////////
// Systolically connected PEs
//////////////////////////////////////////////////////////////////////////
module systolic_pe_matrix(
clk,
reset,
pe_reset,
a0,
a1,
a2,
a3,
a4,
a5,
a6,
a7,
b0,
b1,
b2,
b3,
b4,
b5,
b6,
b7,
matrixC0_0,
matrixC0_1,
matrixC0_2,
matrixC0_3,
matrixC0_4,
matrixC0_5,
matrixC0_6,
matrixC0_7,
matrixC1_0,
matrixC1_1,
matrixC1_2,
matrixC1_3,
matrixC1_4,
matrixC1_5,
matrixC1_6,
matrixC1_7,
matrixC2_0,
matrixC2_1,
matrixC2_2,
matrixC2_3,
matrixC2_4,
matrixC2_5,
matrixC2_6,
matrixC2_7,
matrixC3_0,
matrixC3_1,
matrixC3_2,
matrixC3_3,
matrixC3_4,
matrixC3_5,
matrixC3_6,
matrixC3_7,
matrixC4_0,
matrixC4_1,
matrixC4_2,
matrixC4_3,
matrixC4_4,
matrixC4_5,
matrixC4_6,
matrixC4_7,
matrixC5_0,
matrixC5_1,
matrixC5_2,
matrixC5_3,
matrixC5_4,
matrixC5_5,
matrixC5_6,
matrixC5_7,
matrixC6_0,
matrixC6_1,
matrixC6_2,
matrixC6_3,
matrixC6_4,
matrixC6_5,
matrixC6_6,
matrixC6_7,
matrixC7_0,
matrixC7_1,
matrixC7_2,
matrixC7_3,
matrixC7_4,
matrixC7_5,
matrixC7_6,
matrixC7_7,

a_data_out,
b_data_out
);

input clk;
input reset;
input pe_reset;
input [`DWIDTH-1:0] a0;
input [`DWIDTH-1:0] a1;
input [`DWIDTH-1:0] a2;
input [`DWIDTH-1:0] a3;
input [`DWIDTH-1:0] a4;
input [`DWIDTH-1:0] a5;
input [`DWIDTH-1:0] a6;
input [`DWIDTH-1:0] a7;
input [`DWIDTH-1:0] b0;
input [`DWIDTH-1:0] b1;
input [`DWIDTH-1:0] b2;
input [`DWIDTH-1:0] b3;
input [`DWIDTH-1:0] b4;
input [`DWIDTH-1:0] b5;
input [`DWIDTH-1:0] b6;
input [`DWIDTH-1:0] b7;
output [`DWIDTH-1:0] matrixC0_0;
output [`DWIDTH-1:0] matrixC0_1;
output [`DWIDTH-1:0] matrixC0_2;
output [`DWIDTH-1:0] matrixC0_3;
output [`DWIDTH-1:0] matrixC0_4;
output [`DWIDTH-1:0] matrixC0_5;
output [`DWIDTH-1:0] matrixC0_6;
output [`DWIDTH-1:0] matrixC0_7;
output [`DWIDTH-1:0] matrixC1_0;
output [`DWIDTH-1:0] matrixC1_1;
output [`DWIDTH-1:0] matrixC1_2;
output [`DWIDTH-1:0] matrixC1_3;
output [`DWIDTH-1:0] matrixC1_4;
output [`DWIDTH-1:0] matrixC1_5;
output [`DWIDTH-1:0] matrixC1_6;
output [`DWIDTH-1:0] matrixC1_7;
output [`DWIDTH-1:0] matrixC2_0;
output [`DWIDTH-1:0] matrixC2_1;
output [`DWIDTH-1:0] matrixC2_2;
output [`DWIDTH-1:0] matrixC2_3;
output [`DWIDTH-1:0] matrixC2_4;
output [`DWIDTH-1:0] matrixC2_5;
output [`DWIDTH-1:0] matrixC2_6;
output [`DWIDTH-1:0] matrixC2_7;
output [`DWIDTH-1:0] matrixC3_0;
output [`DWIDTH-1:0] matrixC3_1;
output [`DWIDTH-1:0] matrixC3_2;
output [`DWIDTH-1:0] matrixC3_3;
output [`DWIDTH-1:0] matrixC3_4;
output [`DWIDTH-1:0] matrixC3_5;
output [`DWIDTH-1:0] matrixC3_6;
output [`DWIDTH-1:0] matrixC3_7;
output [`DWIDTH-1:0] matrixC4_0;
output [`DWIDTH-1:0] matrixC4_1;
output [`DWIDTH-1:0] matrixC4_2;
output [`DWIDTH-1:0] matrixC4_3;
output [`DWIDTH-1:0] matrixC4_4;
output [`DWIDTH-1:0] matrixC4_5;
output [`DWIDTH-1:0] matrixC4_6;
output [`DWIDTH-1:0] matrixC4_7;
output [`DWIDTH-1:0] matrixC5_0;
output [`DWIDTH-1:0] matrixC5_1;
output [`DWIDTH-1:0] matrixC5_2;
output [`DWIDTH-1:0] matrixC5_3;
output [`DWIDTH-1:0] matrixC5_4;
output [`DWIDTH-1:0] matrixC5_5;
output [`DWIDTH-1:0] matrixC5_6;
output [`DWIDTH-1:0] matrixC5_7;
output [`DWIDTH-1:0] matrixC6_0;
output [`DWIDTH-1:0] matrixC6_1;
output [`DWIDTH-1:0] matrixC6_2;
output [`DWIDTH-1:0] matrixC6_3;
output [`DWIDTH-1:0] matrixC6_4;
output [`DWIDTH-1:0] matrixC6_5;
output [`DWIDTH-1:0] matrixC6_6;
output [`DWIDTH-1:0] matrixC6_7;
output [`DWIDTH-1:0] matrixC7_0;
output [`DWIDTH-1:0] matrixC7_1;
output [`DWIDTH-1:0] matrixC7_2;
output [`DWIDTH-1:0] matrixC7_3;
output [`DWIDTH-1:0] matrixC7_4;
output [`DWIDTH-1:0] matrixC7_5;
output [`DWIDTH-1:0] matrixC7_6;
output [`DWIDTH-1:0] matrixC7_7;

output [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data_out;
output [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data_out;

wire [`DWIDTH-1:0] a0_0to0_1, a0_1to0_2, a0_2to0_3, a0_3to0_4, a0_4to0_5, a0_5to0_6, a0_6to0_7, a0_7to0_8;
wire [`DWIDTH-1:0] a1_0to1_1, a1_1to1_2, a1_2to1_3, a1_3to1_4, a1_4to1_5, a1_5to1_6, a1_6to1_7, a1_7to1_8;
wire [`DWIDTH-1:0] a2_0to2_1, a2_1to2_2, a2_2to2_3, a2_3to2_4, a2_4to2_5, a2_5to2_6, a2_6to2_7, a2_7to2_8;
wire [`DWIDTH-1:0] a3_0to3_1, a3_1to3_2, a3_2to3_3, a3_3to3_4, a3_4to3_5, a3_5to3_6, a3_6to3_7, a3_7to3_8;
wire [`DWIDTH-1:0] a4_0to4_1, a4_1to4_2, a4_2to4_3, a4_3to4_4, a4_4to4_5, a4_5to4_6, a4_6to4_7, a4_7to4_8;
wire [`DWIDTH-1:0] a5_0to5_1, a5_1to5_2, a5_2to5_3, a5_3to5_4, a5_4to5_5, a5_5to5_6, a5_6to5_7, a5_7to5_8;
wire [`DWIDTH-1:0] a6_0to6_1, a6_1to6_2, a6_2to6_3, a6_3to6_4, a6_4to6_5, a6_5to6_6, a6_6to6_7, a6_7to6_8;
wire [`DWIDTH-1:0] a7_0to7_1, a7_1to7_2, a7_2to7_3, a7_3to7_4, a7_4to7_5, a7_5to7_6, a7_6to7_7, a7_7to7_8;

wire [`DWIDTH-1:0] b0_0to1_0, b1_0to2_0, b2_0to3_0, b3_0to4_0, b4_0to5_0, b5_0to6_0, b6_0to7_0, b7_0to8_0;
wire [`DWIDTH-1:0] b0_1to1_1, b1_1to2_1, b2_1to3_1, b3_1to4_1, b4_1to5_1, b5_1to6_1, b6_1to7_1, b7_1to8_1;
wire [`DWIDTH-1:0] b0_2to1_2, b1_2to2_2, b2_2to3_2, b3_2to4_2, b4_2to5_2, b5_2to6_2, b6_2to7_2, b7_2to8_2;
wire [`DWIDTH-1:0] b0_3to1_3, b1_3to2_3, b2_3to3_3, b3_3to4_3, b4_3to5_3, b5_3to6_3, b6_3to7_3, b7_3to8_3;
wire [`DWIDTH-1:0] b0_4to1_4, b1_4to2_4, b2_4to3_4, b3_4to4_4, b4_4to5_4, b5_4to6_4, b6_4to7_4, b7_4to8_4;
wire [`DWIDTH-1:0] b0_5to1_5, b1_5to2_5, b2_5to3_5, b3_5to4_5, b4_5to5_5, b5_5to6_5, b6_5to7_5, b7_5to8_5;
wire [`DWIDTH-1:0] b0_6to1_6, b1_6to2_6, b2_6to3_6, b3_6to4_6, b4_6to5_6, b5_6to6_6, b6_6to7_6, b7_6to8_6;
wire [`DWIDTH-1:0] b0_7to1_7, b1_7to2_7, b2_7to3_7, b3_7to4_7, b4_7to5_7, b5_7to6_7, b6_7to7_7, b7_7to8_7;

//////////////////////////////////////////////////////////////////////////
// Instantiations of the actual PEs
//////////////////////////////////////////////////////////////////////////
//For larger matmul, more PEs will be needed
wire effective_rst;
assign effective_rst = reset | pe_reset;

processing_element pe0_0(.reset(effective_rst), .clk(clk),  .in_a(a0),      .in_b(b0),  .out_a(a0_0to0_1), .out_b(b0_0to1_0), .out_c(matrixC0_0));
processing_element pe0_1(.reset(effective_rst), .clk(clk),  .in_a(a0_0to0_1), .in_b(b1),  .out_a(a0_1to0_2), .out_b(b0_1to1_1), .out_c(matrixC0_1));
processing_element pe0_2(.reset(effective_rst), .clk(clk),  .in_a(a0_1to0_2), .in_b(b2),  .out_a(a0_2to0_3), .out_b(b0_2to1_2), .out_c(matrixC0_2));
processing_element pe0_3(.reset(effective_rst), .clk(clk),  .in_a(a0_2to0_3), .in_b(b3),  .out_a(a0_3to0_4), .out_b(b0_3to1_3), .out_c(matrixC0_3));
processing_element pe0_4(.reset(effective_rst), .clk(clk),  .in_a(a0_3to0_4), .in_b(b4),  .out_a(a0_4to0_5), .out_b(b0_4to1_4), .out_c(matrixC0_4));
processing_element pe0_5(.reset(effective_rst), .clk(clk),  .in_a(a0_4to0_5), .in_b(b5),  .out_a(a0_5to0_6), .out_b(b0_5to1_5), .out_c(matrixC0_5));
processing_element pe0_6(.reset(effective_rst), .clk(clk),  .in_a(a0_5to0_6), .in_b(b6),  .out_a(a0_6to0_7), .out_b(b0_6to1_6), .out_c(matrixC0_6));
processing_element pe0_7(.reset(effective_rst), .clk(clk),  .in_a(a0_6to0_7), .in_b(b7),  .out_a(a0_7to0_8), .out_b(b0_7to1_7), .out_c(matrixC0_7));

processing_element pe1_0(.reset(effective_rst), .clk(clk),  .in_a(a1), .in_b(b0_0to1_0),  .out_a(a1_0to1_1), .out_b(b1_0to2_0), .out_c(matrixC1_0));
processing_element pe2_0(.reset(effective_rst), .clk(clk),  .in_a(a2), .in_b(b1_0to2_0),  .out_a(a2_0to2_1), .out_b(b2_0to3_0), .out_c(matrixC2_0));
processing_element pe3_0(.reset(effective_rst), .clk(clk),  .in_a(a3), .in_b(b2_0to3_0),  .out_a(a3_0to3_1), .out_b(b3_0to4_0), .out_c(matrixC3_0));
processing_element pe4_0(.reset(effective_rst), .clk(clk),  .in_a(a4), .in_b(b3_0to4_0),  .out_a(a4_0to4_1), .out_b(b4_0to5_0), .out_c(matrixC4_0));
processing_element pe5_0(.reset(effective_rst), .clk(clk),  .in_a(a5), .in_b(b4_0to5_0),  .out_a(a5_0to5_1), .out_b(b5_0to6_0), .out_c(matrixC5_0));
processing_element pe6_0(.reset(effective_rst), .clk(clk),  .in_a(a6), .in_b(b5_0to6_0),  .out_a(a6_0to6_1), .out_b(b6_0to7_0), .out_c(matrixC6_0));
processing_element pe7_0(.reset(effective_rst), .clk(clk),  .in_a(a7), .in_b(b6_0to7_0),  .out_a(a7_0to7_1), .out_b(b7_0to8_0), .out_c(matrixC7_0));

processing_element pe1_1(.reset(effective_rst), .clk(clk),  .in_a(a1_0to1_1), .in_b(b0_1to1_1),  .out_a(a1_1to1_2), .out_b(b1_1to2_1), .out_c(matrixC1_1));
processing_element pe1_2(.reset(effective_rst), .clk(clk),  .in_a(a1_1to1_2), .in_b(b0_2to1_2),  .out_a(a1_2to1_3), .out_b(b1_2to2_2), .out_c(matrixC1_2));
processing_element pe1_3(.reset(effective_rst), .clk(clk),  .in_a(a1_2to1_3), .in_b(b0_3to1_3),  .out_a(a1_3to1_4), .out_b(b1_3to2_3), .out_c(matrixC1_3));
processing_element pe1_4(.reset(effective_rst), .clk(clk),  .in_a(a1_3to1_4), .in_b(b0_4to1_4),  .out_a(a1_4to1_5), .out_b(b1_4to2_4), .out_c(matrixC1_4));
processing_element pe1_5(.reset(effective_rst), .clk(clk),  .in_a(a1_4to1_5), .in_b(b0_5to1_5),  .out_a(a1_5to1_6), .out_b(b1_5to2_5), .out_c(matrixC1_5));
processing_element pe1_6(.reset(effective_rst), .clk(clk),  .in_a(a1_5to1_6), .in_b(b0_6to1_6),  .out_a(a1_6to1_7), .out_b(b1_6to2_6), .out_c(matrixC1_6));
processing_element pe1_7(.reset(effective_rst), .clk(clk),  .in_a(a1_6to1_7), .in_b(b0_7to1_7),  .out_a(a1_7to1_8), .out_b(b1_7to2_7), .out_c(matrixC1_7));
processing_element pe2_1(.reset(effective_rst), .clk(clk),  .in_a(a2_0to2_1), .in_b(b1_1to2_1),  .out_a(a2_1to2_2), .out_b(b2_1to3_1), .out_c(matrixC2_1));
processing_element pe2_2(.reset(effective_rst), .clk(clk),  .in_a(a2_1to2_2), .in_b(b1_2to2_2),  .out_a(a2_2to2_3), .out_b(b2_2to3_2), .out_c(matrixC2_2));
processing_element pe2_3(.reset(effective_rst), .clk(clk),  .in_a(a2_2to2_3), .in_b(b1_3to2_3),  .out_a(a2_3to2_4), .out_b(b2_3to3_3), .out_c(matrixC2_3));
processing_element pe2_4(.reset(effective_rst), .clk(clk),  .in_a(a2_3to2_4), .in_b(b1_4to2_4),  .out_a(a2_4to2_5), .out_b(b2_4to3_4), .out_c(matrixC2_4));
processing_element pe2_5(.reset(effective_rst), .clk(clk),  .in_a(a2_4to2_5), .in_b(b1_5to2_5),  .out_a(a2_5to2_6), .out_b(b2_5to3_5), .out_c(matrixC2_5));
processing_element pe2_6(.reset(effective_rst), .clk(clk),  .in_a(a2_5to2_6), .in_b(b1_6to2_6),  .out_a(a2_6to2_7), .out_b(b2_6to3_6), .out_c(matrixC2_6));
processing_element pe2_7(.reset(effective_rst), .clk(clk),  .in_a(a2_6to2_7), .in_b(b1_7to2_7),  .out_a(a2_7to2_8), .out_b(b2_7to3_7), .out_c(matrixC2_7));
processing_element pe3_1(.reset(effective_rst), .clk(clk),  .in_a(a3_0to3_1), .in_b(b2_1to3_1),  .out_a(a3_1to3_2), .out_b(b3_1to4_1), .out_c(matrixC3_1));
processing_element pe3_2(.reset(effective_rst), .clk(clk),  .in_a(a3_1to3_2), .in_b(b2_2to3_2),  .out_a(a3_2to3_3), .out_b(b3_2to4_2), .out_c(matrixC3_2));
processing_element pe3_3(.reset(effective_rst), .clk(clk),  .in_a(a3_2to3_3), .in_b(b2_3to3_3),  .out_a(a3_3to3_4), .out_b(b3_3to4_3), .out_c(matrixC3_3));
processing_element pe3_4(.reset(effective_rst), .clk(clk),  .in_a(a3_3to3_4), .in_b(b2_4to3_4),  .out_a(a3_4to3_5), .out_b(b3_4to4_4), .out_c(matrixC3_4));
processing_element pe3_5(.reset(effective_rst), .clk(clk),  .in_a(a3_4to3_5), .in_b(b2_5to3_5),  .out_a(a3_5to3_6), .out_b(b3_5to4_5), .out_c(matrixC3_5));
processing_element pe3_6(.reset(effective_rst), .clk(clk),  .in_a(a3_5to3_6), .in_b(b2_6to3_6),  .out_a(a3_6to3_7), .out_b(b3_6to4_6), .out_c(matrixC3_6));
processing_element pe3_7(.reset(effective_rst), .clk(clk),  .in_a(a3_6to3_7), .in_b(b2_7to3_7),  .out_a(a3_7to3_8), .out_b(b3_7to4_7), .out_c(matrixC3_7));
processing_element pe4_1(.reset(effective_rst), .clk(clk),  .in_a(a4_0to4_1), .in_b(b3_1to4_1),  .out_a(a4_1to4_2), .out_b(b4_1to5_1), .out_c(matrixC4_1));
processing_element pe4_2(.reset(effective_rst), .clk(clk),  .in_a(a4_1to4_2), .in_b(b3_2to4_2),  .out_a(a4_2to4_3), .out_b(b4_2to5_2), .out_c(matrixC4_2));
processing_element pe4_3(.reset(effective_rst), .clk(clk),  .in_a(a4_2to4_3), .in_b(b3_3to4_3),  .out_a(a4_3to4_4), .out_b(b4_3to5_3), .out_c(matrixC4_3));
processing_element pe4_4(.reset(effective_rst), .clk(clk),  .in_a(a4_3to4_4), .in_b(b3_4to4_4),  .out_a(a4_4to4_5), .out_b(b4_4to5_4), .out_c(matrixC4_4));
processing_element pe4_5(.reset(effective_rst), .clk(clk),  .in_a(a4_4to4_5), .in_b(b3_5to4_5),  .out_a(a4_5to4_6), .out_b(b4_5to5_5), .out_c(matrixC4_5));
processing_element pe4_6(.reset(effective_rst), .clk(clk),  .in_a(a4_5to4_6), .in_b(b3_6to4_6),  .out_a(a4_6to4_7), .out_b(b4_6to5_6), .out_c(matrixC4_6));
processing_element pe4_7(.reset(effective_rst), .clk(clk),  .in_a(a4_6to4_7), .in_b(b3_7to4_7),  .out_a(a4_7to4_8), .out_b(b4_7to5_7), .out_c(matrixC4_7));
processing_element pe5_1(.reset(effective_rst), .clk(clk),  .in_a(a5_0to5_1), .in_b(b4_1to5_1),  .out_a(a5_1to5_2), .out_b(b5_1to6_1), .out_c(matrixC5_1));
processing_element pe5_2(.reset(effective_rst), .clk(clk),  .in_a(a5_1to5_2), .in_b(b4_2to5_2),  .out_a(a5_2to5_3), .out_b(b5_2to6_2), .out_c(matrixC5_2));
processing_element pe5_3(.reset(effective_rst), .clk(clk),  .in_a(a5_2to5_3), .in_b(b4_3to5_3),  .out_a(a5_3to5_4), .out_b(b5_3to6_3), .out_c(matrixC5_3));
processing_element pe5_4(.reset(effective_rst), .clk(clk),  .in_a(a5_3to5_4), .in_b(b4_4to5_4),  .out_a(a5_4to5_5), .out_b(b5_4to6_4), .out_c(matrixC5_4));
processing_element pe5_5(.reset(effective_rst), .clk(clk),  .in_a(a5_4to5_5), .in_b(b4_5to5_5),  .out_a(a5_5to5_6), .out_b(b5_5to6_5), .out_c(matrixC5_5));
processing_element pe5_6(.reset(effective_rst), .clk(clk),  .in_a(a5_5to5_6), .in_b(b4_6to5_6),  .out_a(a5_6to5_7), .out_b(b5_6to6_6), .out_c(matrixC5_6));
processing_element pe5_7(.reset(effective_rst), .clk(clk),  .in_a(a5_6to5_7), .in_b(b4_7to5_7),  .out_a(a5_7to5_8), .out_b(b5_7to6_7), .out_c(matrixC5_7));
processing_element pe6_1(.reset(effective_rst), .clk(clk),  .in_a(a6_0to6_1), .in_b(b5_1to6_1),  .out_a(a6_1to6_2), .out_b(b6_1to7_1), .out_c(matrixC6_1));
processing_element pe6_2(.reset(effective_rst), .clk(clk),  .in_a(a6_1to6_2), .in_b(b5_2to6_2),  .out_a(a6_2to6_3), .out_b(b6_2to7_2), .out_c(matrixC6_2));
processing_element pe6_3(.reset(effective_rst), .clk(clk),  .in_a(a6_2to6_3), .in_b(b5_3to6_3),  .out_a(a6_3to6_4), .out_b(b6_3to7_3), .out_c(matrixC6_3));
processing_element pe6_4(.reset(effective_rst), .clk(clk),  .in_a(a6_3to6_4), .in_b(b5_4to6_4),  .out_a(a6_4to6_5), .out_b(b6_4to7_4), .out_c(matrixC6_4));
processing_element pe6_5(.reset(effective_rst), .clk(clk),  .in_a(a6_4to6_5), .in_b(b5_5to6_5),  .out_a(a6_5to6_6), .out_b(b6_5to7_5), .out_c(matrixC6_5));
processing_element pe6_6(.reset(effective_rst), .clk(clk),  .in_a(a6_5to6_6), .in_b(b5_6to6_6),  .out_a(a6_6to6_7), .out_b(b6_6to7_6), .out_c(matrixC6_6));
processing_element pe6_7(.reset(effective_rst), .clk(clk),  .in_a(a6_6to6_7), .in_b(b5_7to6_7),  .out_a(a6_7to6_8), .out_b(b6_7to7_7), .out_c(matrixC6_7));
processing_element pe7_1(.reset(effective_rst), .clk(clk),  .in_a(a7_0to7_1), .in_b(b6_1to7_1),  .out_a(a7_1to7_2), .out_b(b7_1to8_1), .out_c(matrixC7_1));
processing_element pe7_2(.reset(effective_rst), .clk(clk),  .in_a(a7_1to7_2), .in_b(b6_2to7_2),  .out_a(a7_2to7_3), .out_b(b7_2to8_2), .out_c(matrixC7_2));
processing_element pe7_3(.reset(effective_rst), .clk(clk),  .in_a(a7_2to7_3), .in_b(b6_3to7_3),  .out_a(a7_3to7_4), .out_b(b7_3to8_3), .out_c(matrixC7_3));
processing_element pe7_4(.reset(effective_rst), .clk(clk),  .in_a(a7_3to7_4), .in_b(b6_4to7_4),  .out_a(a7_4to7_5), .out_b(b7_4to8_4), .out_c(matrixC7_4));
processing_element pe7_5(.reset(effective_rst), .clk(clk),  .in_a(a7_4to7_5), .in_b(b6_5to7_5),  .out_a(a7_5to7_6), .out_b(b7_5to8_5), .out_c(matrixC7_5));
processing_element pe7_6(.reset(effective_rst), .clk(clk),  .in_a(a7_5to7_6), .in_b(b6_6to7_6),  .out_a(a7_6to7_7), .out_b(b7_6to8_6), .out_c(matrixC7_6));
processing_element pe7_7(.reset(effective_rst), .clk(clk),  .in_a(a7_6to7_7), .in_b(b6_7to7_7),  .out_a(a7_7to7_8), .out_b(b7_7to8_7), .out_c(matrixC7_7));
assign a_data_out = {a7_7to7_8,a6_7to6_8,a5_7to5_8,a4_7to4_8,a3_7to3_8,a2_7to2_8,a1_7to1_8,a0_7to0_8};
assign b_data_out = {b7_7to8_7,b7_6to8_6,b7_5to8_5,b7_4to8_4,b7_3to8_3,b7_2to8_2,b7_1to8_1,b7_0to8_0};

endmodule

module processing_element(
 reset, 
 clk, 
 in_a,
 in_b, 
 out_a, 
 out_b, 
 out_c
 );

 input reset;
 input clk;
 input  [`DWIDTH-1:0] in_a;
 input  [`DWIDTH-1:0] in_b;
 output [`DWIDTH-1:0] out_a;
 output [`DWIDTH-1:0] out_b;
 output [`DWIDTH-1:0] out_c;  //reduced precision

 reg [`DWIDTH-1:0] out_a;
 reg [`DWIDTH-1:0] out_b;
 wire [`DWIDTH-1:0] out_c;

 wire [`DWIDTH-1:0] out_mac;

 assign out_c = out_mac;

 seq_mac u_mac(.a(in_a), .b(in_b), .out(out_mac), .reset(reset), .clk(clk));

 always @(posedge clk)begin
    if(reset) begin
      out_a<=0;
      out_b<=0;
    end
    else begin  
      out_a<=in_a;
      out_b<=in_b;
    end
 end
 
endmodule

module seq_mac(a, b, out, reset, clk);
input [`DWIDTH-1:0] a;
input [`DWIDTH-1:0] b;
input reset;
input clk;
output [`DWIDTH-1:0] out;

reg [2*`DWIDTH-1:0] out_temp;
wire [`DWIDTH-1:0] mul_out;
wire [2*`DWIDTH-1:0] add_out;

reg [`DWIDTH-1:0] a_flopped;
reg [`DWIDTH-1:0] b_flopped;

wire [2*`DWIDTH-1:0] mul_out_temp;
reg [2*`DWIDTH-1:0] mul_out_temp_reg;

always @(posedge clk) begin
  if (reset) begin
    a_flopped <= 0;
    b_flopped <= 0;
  end else begin
    a_flopped <= a;
    b_flopped <= b;
  end
end

//assign mul_out = a * b;
qmult mult_u1(.i_multiplicand(a_flopped), .i_multiplier(b_flopped), .o_result(mul_out_temp));

always @(posedge clk) begin
  if (reset) begin
    mul_out_temp_reg <= 0;
  end else begin
    mul_out_temp_reg <= mul_out_temp;
  end
end

//we just truncate the higher bits of the product
//assign add_out = mul_out + out;
qadd add_u1(.a(out_temp), .b(mul_out_temp_reg), .c(add_out));

always @(posedge clk) begin
  if (reset) begin
    out_temp <= 0;
  end else begin
    out_temp <= add_out;
  end
end

//down cast the result
assign out = 
    (out_temp[2*`DWIDTH-1] == 0) ?  //positive number
        (
           (|(out_temp[2*`DWIDTH-2 : `DWIDTH-1])) ?  //is any bit from 14:7 is 1, that means overlfow
             {out_temp[2*`DWIDTH-1] , {(`DWIDTH-1){1'b1}}} : //sign bit and then all 1s
             {out_temp[2*`DWIDTH-1] , out_temp[`DWIDTH-2:0]} 
        )
        : //negative number
        (
           (|(out_temp[2*`DWIDTH-2 : `DWIDTH-1])) ?  //is any bit from 14:7 is 0, that means overlfow
             {out_temp[2*`DWIDTH-1] , out_temp[`DWIDTH-2:0]} :
             {out_temp[2*`DWIDTH-1] , {(`DWIDTH-1){1'b0}}} //sign bit and then all 0s
        );

endmodule

module qmult(i_multiplicand,i_multiplier,o_result);
input [`DWIDTH-1:0] i_multiplicand;
input [`DWIDTH-1:0] i_multiplier;
output [2*`DWIDTH-1:0] o_result;

assign o_result = i_multiplicand * i_multiplier;
//DW02_mult #(`DWIDTH,`DWIDTH) u_mult(.A(i_multiplicand), .B(i_multiplier), .TC(1'b1), .PRODUCT(o_result));

endmodule

module qadd(a,b,c);
input [2*`DWIDTH-1:0] a;
input [2*`DWIDTH-1:0] b;
output [2*`DWIDTH-1:0] c;

assign c = a + b;
//DW01_add #(`DWIDTH) u_add(.A(a), .B(b), .CI(1'b0), .SUM(c), .CO());
endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: matmul_8x8.v
//////////////////////////////////////////////////////////////////////

module matmul_unit(
 clk,
 resetn,
 activate,
 en,
 squash,
 stall,
 a_data,
 b_data,
 validity_mask_a_rows,
 validity_mask_a_cols,
 validity_mask_b_rows,
 validity_mask_b_cols,
 c_data,
 stall_matmul, 
 vmask,
 in_dst,
 in_dst_we,
 out_dst,
 out_dst_we,
 out_data_avail,
 out_dst_mask
);

parameter REGIDWIDTH=8;
parameter PIPE_STAGES_MATMUL=29;
parameter NUMLANES=8;
parameter MATMUL_STAGES = 35;

 input clk;
 input resetn;
 input activate;
// input [PIPE_STAGES_MATMUL:1] en;  //Enable for each pipestage
// input [PIPE_STAGES_MATMUL:1] squash;  //Squash for each pipestage
 input [NUMLANES-1:0] en;  //Enable for each pipestage
 input [NUMLANES-1:0] squash;  //Squash for each pipestage
 output stall;
 output reg stall_matmul;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data;
 input [`MASK_WIDTH-1:0] validity_mask_a_rows;
 input [`MASK_WIDTH-1:0] validity_mask_a_cols;
 input [`MASK_WIDTH-1:0] validity_mask_b_rows;
 input [`MASK_WIDTH-1:0] validity_mask_b_cols;

 output [`MAT_MUL_SIZE*`DWIDTH-1:0] c_data;
 input [NUMLANES-1:0] vmask;

 input    [REGIDWIDTH-1:0] in_dst;
 input                     in_dst_we;
// output [PIPE_STAGES_MATMUL*REGIDWIDTH-1:0] out_dst;
// output            [PIPE_STAGES_MATMUL-1:0] out_dst_we;
// output   [PIPE_STAGES_MATMUL*NUMLANES-1:0] out_dst_mask;
 output [NUMLANES*REGIDWIDTH-1:0] out_dst;
 output            [NUMLANES-1:0] out_dst_we;
 output   [NUMLANES*NUMLANES-1:0] out_dst_mask;
 output out_data_avail;
//wire [PIPE_STAGES_MATMUL:1] ctrl_activate;
//wire [PIPE_STAGES_MATMUL:1] squash_activatepipe_NC;
wire [NUMLANES-1:0] ctrl_activate;
wire [NUMLANES-1:0] squash_activatepipe_NC;


//pipe #(1,PIPE_STAGES_MATMUL-1) activatepipe (
pipe #(1,NUMLANES) activatepipe (
    .d(activate),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(squash_activatepipe_NC),
    .q(ctrl_activate));

wire in_progress;
assign stall=in_progress;

//The actual matrix multiplier block. This is an abridged
//version of the matmul block used in the TPU v1 design.
matmul_8x8 mat(
    .clk(clk),
    .reset(~resetn),
    .pe_reset(~resetn),
    .start(activate),
    .in_progress(in_progress),
    .done(done_NC),
    .a_data(a_data),
    .b_data(b_data),
    .c_data_out(c_data), 
    .c_data_available(out_data_avail),
    .validity_mask_a_rows(validity_mask_a_rows),
    .validity_mask_a_cols(validity_mask_a_cols),
    .validity_mask_b_rows(validity_mask_b_rows),
    .validity_mask_b_cols(validity_mask_b_cols)
);

reg [5:0] cnt;

always@(posedge clk)begin
   if(!resetn)begin
     cnt <= 0;
   end
   else begin
     if(!activate)
        cnt <= 0;
     else if(activate)
        cnt <= cnt+1;
   end
end

always@(*)begin
  if(cnt < NUMLANES-1)
    stall_matmul = 1'b0;
  else if (cnt >= MATMUL_STAGES)
    stall_matmul = 1'b0;
  else
    stall_matmul = 1'b1; 
end


//pipe #(REGIDWIDTH,PIPE_STAGES_MATMUL-1) dstpipe (
wire [NUMLANES-1:0] squash_dstpipe_NC;
pipe #(REGIDWIDTH,NUMLANES) dstpipe (
    .d(in_dst),  
    .clk(clk),
    .resetn(resetn),
    .en(en[NUMLANES-1:0] & {(NUMLANES){(~stall) | (~stall_matmul) }} ),
    .squash(squash_dstpipe_NC),
    .q(out_dst));

//pipe #(1,PIPE_STAGES_MATMUL-1) dstwepipe (
pipe #(1,NUMLANES) dstwepipe (
    .d(in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[NUMLANES-1:0] & {(NUMLANES){~stall| (~stall_matmul)}} ),
    .squash(squash[NUMLANES-1:1]),
    .q(out_dst_we));

wire [PIPE_STAGES_MATMUL:1] squash_dstmaskpipe_NC;
pipe #(NUMLANES,NUMLANES) dstmaskpipe (
    .d(vmask ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[NUMLANES-1:0] & {(NUMLANES){~stall| (~stall_matmul)}} ),
    .squash(squash_dstmaskpipe_NC),
    .q(out_dst_mask));

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: matmul_unit.v
//////////////////////////////////////////////////////////////////////

`define LO(x,b) ((x)&~({1024{1'b1}}<<b))

module vlanes (
    clk,
    resetn,

    // Instruction interface
    instr,
    instr_en,     // tells when instr is valid and available
    instr_wait,   // if high says vpu is not ready to receive

    stall_in,
    is_stalled,
    has_memop,

    // Control register values 
    vc_in,
    vl_in,
    vbase_in,
    vinc_in,
    vstride_in,
    vs_in,
    matmul_masks_in,
    dma_en,
    mem_addr,
    lane_addr,
    dma_we,
    num_bytes,
    dma_busy,
    // vs Writeback
    vs_writedata,
    vs_we,       // Actually issues write command for vs (after done stalling)
    vs_wetrack,  // says there exist a scalar write operation in pipe
    vs_dst,

    //AXI interface
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
    S_BID    ,  


    // Data memory interface
    dbus_address,
    dbus_en,
    dbus_we,
    dbus_byteen,
    dbus_writedata,
    dbus_readdata,
    dbus_cachematch,
    dbus_cachemiss,
    dbus_prefetch,    //Prefetch hint
    dbus_wait,

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

parameter NUMLANES=4;
parameter LOG2NUMLANES=2;
parameter MVL=128;
parameter LOG2MVL=7;
parameter VPW=4;                  // VP width in number of bytes
parameter LOG2VPW=2;
parameter LANEWIDTH=8*VPW;
parameter LOG2LANEWIDTH=3+LOG2VPW;
parameter NUMBANKS=2;
parameter LOG2NUMBANKS=1;
parameter ALUPERBANK=1;
parameter NUMMEMPARALLELLANES=2;
parameter LOG2NUMMEMPARALLELLANES=1;
parameter NUMMULLANES=NUMLANES;
parameter DMEM_WRITEWIDTH=128;     // Width of write bus to memory
parameter LOG2DMEM_WRITEWIDTH=7;  // Log2 of Width of write bus to memory
parameter DMEM_READWIDTH=128;
parameter LOG2DMEM_READWIDTH=7;
parameter VCWIDTH=32;
parameter VSWIDTH=32;
parameter NUMVSREGS=32;
parameter LOG2NUMVSREGS=5;

parameter VRIDWIDTH=5;
parameter VELMIDWIDTH=LOG2MVL-LOG2NUMLANES;
parameter REGIDWIDTH=VRIDWIDTH+LOG2MVL-LOG2NUMLANES;
parameter BANKREGIDWIDTH=VRIDWIDTH+LOG2MVL-LOG2NUMLANES-LOG2NUMBANKS;

// Register identifier = { vr[0-31], element }

`define VRID_RANGE REGIDWIDTH-1:REGIDWIDTH-VRIDWIDTH
`define VRELM_RANGE REGIDWIDTH-VRIDWIDTH-1:0

// NUMFUS = ALU*NUMBANKS + MUL + MEM + FALU*MEMBANKS + MATMUL(1) + BFLOAT
// UNITS(2) + ACTIVATION + TRP + TRANSPOSE + PERMUTE
parameter NUMFUS=4+2*(NUMBANKS-1)*ALUPERBANK+1+4+2 + 1; //Adding 1 for matmul FU + 6 for BFLOTs, ACTIVATION, TRP, PERMUTE and TRANSPOSE and AXI
parameter FU_ALU=0;
parameter FU_MUL=FU_ALU+(NUMBANKS-1)*ALUPERBANK+1;
parameter FU_MEM=FU_MUL+1;
parameter FU_FALU=FU_MEM+1;
parameter FU_MATMUL=FU_FALU+1;
parameter FU_BFADDER = FU_MATMUL + 1;
parameter FU_BFMULT = FU_BFADDER + 1;
parameter FU_ACT = FU_BFMULT + 1;
parameter FU_TRP = FU_ACT + 1;
parameter FU_TRANSPOSE = FU_TRP + 1;
parameter FU_PERMUTE = FU_TRANSPOSE + 1;
parameter FU_AXI = FU_PERMUTE + 1;

input clk;
input resetn;

//Instruction Format:
//9 - op[21] mask 1/0
//8 - op[23] scalar/vec src2
//7 - op[22] scalar/vec src1
//6:0 - {op[24],op[5:0]} op

input [31:0] instr;
input instr_en;     // tells when instr is valid and available
output instr_wait;   // if high says vpu is not ready to receive

input [3:0] stall_in;  
output [`MAX_PIPE_STAGES-1:0] is_stalled;
output has_memop;
output dma_busy;

// Control register values
input  [ VCWIDTH-1 : 0 ]   vc_in;
input  [ VCWIDTH-1 : 0 ]   vl_in;
input  [ VCWIDTH-1 : 0 ]   vbase_in;
input  [ VCWIDTH-1 : 0 ]   vinc_in;
input  [ VCWIDTH-1 : 0 ]   vstride_in;
input  [ VSWIDTH-1 : 0 ]   vs_in;
input  [3*`MAT_MUL_SIZE-1 : 0]  matmul_masks_in;
input  dma_en;
input  [VCWIDTH-1:0] mem_addr;
input  [VCWIDTH-1:0] lane_addr;
input  dma_we;
input  [VCWIDTH-1:0] num_bytes;

// vs Writeback
output       [ VSWIDTH-1 : 0 ]   vs_writedata;
output                           vs_we;
output               [ 5 : 2 ]   vs_wetrack;
output [ LOG2NUMVSREGS-1 : 0 ]   vs_dst;

// Data memory interface
output  [ 31 : 0 ]  dbus_address;
output              dbus_en;
output              dbus_we;
output  [ (DMEM_WRITEWIDTH/8)-1 : 0 ]   dbus_byteen;
output  [ DMEM_WRITEWIDTH-1 : 0 ]  dbus_writedata;
input   [ DMEM_READWIDTH-1 : 0 ]  dbus_readdata;
input               dbus_cachematch;
input               dbus_cachemiss;
input               dbus_wait;
output  [ 31 : 0 ]  dbus_prefetch;

//DMA signals
output [31:0]                 dma_dbus_address;   
input [DMEM_READWIDTH-1:0]    dma_dbus_readdata;  
output [DMEM_WRITEWIDTH-1:0]  dma_dbus_writedata; 
output [DMEM_WRITEWIDTH/8-1:0]dma_dbus_byteen;
output                        dma_dbus_en;        
output                        dma_dbus_wren;      
output                        dma_dbus_prefetch;  
input                         dma_dbus_wait;      
input                         dma_dbus_data_valid;


// AXI interface
//
output  [6 - 1:0]                    M_AWID;                                                 
output  [LANEWIDTH- 1:0]             M_AWADDR;                                    
output  [7:0]                        M_AWLEN;                                                    
output  [2:0]                        M_AWSIZE;                                                   
output  [1:0]                        M_AWBURST;                                                  
output                               M_AWLOCK;                                                         
output  [3:0]                        M_AWCACHE;                                                  
output  [2:0]                        M_AWPROT;                                                   
output  [3:0]                        M_AWQOS;                                                    
output                               M_AWVALID;                                                        
input                                M_AWREADY;                                                        
output  [NUMLANES*LANEWIDTH-1 : 0]   M_WDATA;                                     
output  [(NUMLANES*LANEWIDTH)/8-1 : 0] M_WSTRB;                                   
output                               M_WLAST;                                                          
output                               M_WVALID;                                                         
input                                M_WREADY;                                                         

output  [6-1 : 0]                    M_ARID;                                                 
output  [LANEWIDTH-1 : 0]            M_ARADDR;                                    
output  [7 : 0]                      M_ARLEN;                                                  
output  [2 : 0]                      M_ARSIZE;                                                 
output  [1 : 0]                      M_ARBURST;                                                
output                               M_ARLOCK;                                                         
output  [3 : 0]                      M_ARCACHE;                                                
output  [2 : 0]                      M_ARPROT;                                                 
output  [3 : 0]                      M_ARQOS;                                                  
output                               M_ARVALID;                                                        
input                                M_ARREADY;                                                        
input   [6-1 : 0]                    M_RID;                                         	    
input   [NUMLANES*LANEWIDTH-1 : 0]   M_RDATA;                                     
input   [1 : 0]                      M_RRESP;                                                  
input                                M_RLAST;                                                          
input                                M_RVALID;                                                         
output                               M_RREADY;                                                         

output                               M_BREADY;                                                         
input                                M_BVALID;                                                         
input   [1 : 0]                      M_BRESP;                                                  
input   [6-1 : 0]                    M_BID;                                                   

input  [6 - 1:0]                    S_AWID;                                                 
input  [LANEWIDTH- 1:0]             S_AWADDR;                                    
input  [7:0]                        S_AWLEN;                                                    
input  [2:0]                        S_AWSIZE;                                                   
input  [1:0]                        S_AWBURST;                                                  
input                               S_AWLOCK;                                                         
input  [3:0]                        S_AWCACHE;                                                  
input  [2:0]                        S_AWPROT;                                                   
input  [3:0]                        S_AWQOS;                                                    
input                               S_AWVALID;                                                        
output                                S_AWREADY;                                                        
input  [NUMLANES*LANEWIDTH-1 : 0]   S_WDATA;                                     
input  [(NUMLANES*LANEWIDTH)/8-1 : 0] S_WSTRB;                                   
input                               S_WLAST;                                                          
input                               S_WVALID;                                                         
output                                S_WREADY;                                                         

input  [6-1 : 0]                    S_ARID;                                                 
input  [LANEWIDTH-1 : 0]            S_ARADDR;                                    
input  [7 : 0]                      S_ARLEN;                                                  
input  [2 : 0]                      S_ARSIZE;                                                 
input  [1 : 0]                      S_ARBURST;                                                
input                               S_ARLOCK;                                                         
input  [3 : 0]                      S_ARCACHE;                                                
input  [2 : 0]                      S_ARPROT;                                                 
input  [3 : 0]                      S_ARQOS;                                                  
input                               S_ARVALID;                                                        
output                                S_ARREADY;                                                        
output   [6-1 : 0]                    S_RID;                                         	    
output   [NUMLANES*LANEWIDTH-1 : 0]   S_RDATA;                                     
output   [1 : 0]                      S_RRESP;                                                  
output                                S_RLAST;                                                          
output                                S_RVALID;                                                         
input                               S_RREADY;                                                         

input                               S_BREADY;                                                         
output                                S_BVALID;                                                         
output   [1 : 0]                      S_BRESP;                                                  
output   [6-1 : 0]                    S_BID;     

//

//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: visa.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/visa.v
//////////////////////////////////////////////////////////////////////
parameter COP2_VADD           = 'b10z0000000;
parameter COP2_VADD_U         = 'b10z0000001;
parameter COP2_VSUB           = 'b10zz000010;
parameter COP2_VSUB_U         = 'b10zz000011;
parameter COP2_VMULHI         = 'b10z0000100;
parameter COP2_VMULHI_U       = 'b10z0000101;
parameter COP2_VDIV           = 'b10zz000110; //Using as matmul
//parameter COP2_VBFADD         = 'b0100000001; //Using BF16 add
//parameter COP2_VBFMULT        = 'b0100000010; //Using BF16 MULT
//parameter COP2_VACT           = 'b0100000011; //Using ACT
//parameter COP2_VTRP           = 'b0100000100; //Using ACT
parameter COP2_VDIV_U         = 'b10zz000111;

//parameter COP2_VMOD           = 'b10zz001000;
parameter COP2_VBFADD           = 'b10zz001000;  // USING bfloat add Instr: vmod.vv vrdest, vrsrc1,vrsrc2

//parameter COP2_VMOD_U         = 'b10zz001001;
parameter COP2_VBFMULT         = 'b10zz001001;  // Using bfloat mult Instr: vmod.u.vv vrdest, vrsrc1,vrsrc2
parameter COP2_VMOD_U         = 'b10zz001001;
parameter COP2_VCMP_EQ        = 'b10zz001010;
parameter COP2_VCMP_NE        = 'b10zz001100;
parameter COP2_VCMP_LT        = 'b10zz001110;
parameter COP2_VCMP_U_LT      = 'b10zz001111;
parameter COP2_VCMP_LE        = 'b10zz010000;
parameter COP2_VCMP_U_LE      = 'b10zz010001;
parameter COP2_VMIN           = 'b10z0010010;
parameter COP2_VMIN_U         = 'b10z0010011;
parameter COP2_VMAX           = 'b10z0010100;
parameter COP2_VMAX_U         = 'b10z0010101;
parameter COP2_VMULLO         = 'b10z0010110;
parameter COP2_VABS           = 'b1000010111;
parameter COP2_VAND           = 'b10z0011000;
parameter COP2_VOR            = 'b10z0011001;
parameter COP2_VXOR           = 'b10z0011010;
parameter COP2_VNOR           = 'b10z0011011;
parameter COP2_VSLL           = 'b10zz011100;
parameter COP2_VSRL           = 'b10zz011101;
parameter COP2_VSRA           = 'b10zz011110;
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
parameter COP2_VSADD          = 'b10z0100010;
parameter COP2_VSADD_U        = 'b10z0100011;
parameter COP2_VSSUB          = 'b10zz100100;
parameter COP2_VSSUB_U        = 'b10zz100101;
parameter COP2_VSRR           = 'b1000100110;
parameter COP2_VSRR_U         = 'b1000100111;
parameter COP2_VSLS           = 'b1000101000;
parameter COP2_VSLS_U         = 'b1000101001;
parameter COP2_VXUMUL         = 'b10z0101010;
parameter COP2_VXUMUL_U       = 'b10z0101011;
parameter COP2_VXLMUL         = 'b10z0101100;
parameter COP2_VXLMUL_U       = 'b10z0101101;
parameter COP2_VXUMADD        = 'b10z0101110;
parameter COP2_VXUMADD_U      = 'b10z0101111;
parameter COP2_VXUMSUB        = 'b10z0110000;
parameter COP2_VXUMSUB_U      = 'b10z0110001;
parameter COP2_VXLMADD        = 'b10z0110010;
parameter COP2_VXLMADD_U      = 'b10z0110011;
parameter COP2_VXLMSUB        = 'b10z0110100;
parameter COP2_VXLMSUB_U      = 'b10z0110101;
parameter COP2_VINS_VV        = 'b1100000000;
parameter COP2_VINS_SV        = 'b1110000001;
parameter COP2_VEXT_VV        = 'b1100000010;
parameter COP2_VEXT_SV        = 'b1100000011;
parameter COP2_VEXT_U_SV      = 'b1100000100;
parameter COP2_VCOMPRESS      = 'b1100000101;
parameter COP2_VEXPAND        = 'b1100000110;
parameter COP2_VMERGE         = 'b11zz000111;
parameter COP2_VFINS          = 'b1110001000;
parameter COP2_VEXTHALF       = 'b1100001001;
parameter COP2_VHALF          = 'b1100001010;
parameter COP2_VHALFUP        = 'b1100001011;
parameter COP2_VHALFDN        = 'b1100001100;
parameter COP2_VSATVL         = 'b1100001101;
parameter COP2_VFAND          = 'b11z0001110;
parameter COP2_VFOR           = 'b11z0001111;
parameter COP2_VFXOR          = 'b11z0010000;
parameter COP2_VFNOR          = 'b11z0010001;
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

parameter BIT_VSSRC2=6;
parameter BIT_VSSRC1=7;

parameter ALUOP_ZERO    =11'b00100000101,
          ALUOP_ADD     =11'b00010000000,
          ALUOP_ADDU    =11'b00000000000,
          ALUOP_SUB     =11'b00011000000,
          ALUOP_SUBU    =11'b00001000000,
          ALUOP_CMP_EQ  =11'b00011000000,
          ALUOP_CMP_NEQ =11'b00011000001,
          ALUOP_CMP_LT  =11'b00011000010,
          ALUOP_CMP_LE  =11'b00011000011,
          ALUOP_CMP_LT_U=11'b00001000010,
          ALUOP_CMP_LE_U=11'b00001000011,
          ALUOP_AND     =11'b00000000101,
          ALUOP_OR      =11'b00000000001,
          ALUOP_XOR     =11'b00000001001,
          ALUOP_NOR     =11'b00000001101,
          ALUOP_MIN     =11'b00111010001,
          ALUOP_MIN_U   =11'b00101010001,
          ALUOP_MAX     =11'b00111100001,
          ALUOP_MAX_U   =11'b00101100001,
          ALUOP_ABS     =11'b11111000010,
          ALUOP_MERGE   =11'b10100000011;

parameter SATSUMOP_NOP=2'b00,
          SATSUMOP_VS =2'b11,
          SATSUMOP_VSU=2'b10;

parameter SATSIZEOP_VSATUW=4'b0000,
          SATSIZEOP_VSATUB=4'b0001,
          SATSIZEOP_VSATUH=4'b0010,
          SATSIZEOP_VSATW  =4'b1100,
          SATSIZEOP_VSATB  =4'b1101,
          SATSIZEOP_VSATH  =4'b1110,
          SATSIZEOP_VSATSUW=4'b0100,
          SATSIZEOP_VSATSUB=4'b0101,
          SATSIZEOP_VSATSUH=4'b0110;

parameter MULOP_ZERO  =5'b00000,
          MULOP_LMULU =5'b10110,
          MULOP_UMULU =5'b10111,
          MULOP_LMUL  =5'b10100,
          MULOP_UMUL  =5'b10101,
          MULOP_MULUHI=5'b00111,
          MULOP_MULULO=5'b00110,
          MULOP_MULHI =5'b00101,
          MULOP_MULLO =5'b00100,
          MULOP_SLL   =5'b00010,
          MULOP_SLS   =5'b01010,
          MULOP_SLSU  =5'b01000,
          MULOP_SRL   =5'b00011,
          MULOP_SRA   =5'b00001;

parameter MEMOP_SHIFT=7'b0000000, 
          MEMOP_LDUB=7'b1000000,
          MEMOP_LDUH=7'b1000100,
          MEMOP_LDUW=7'b1001000,
          MEMOP_LDB=7'b1000010,
          MEMOP_LDH=7'b1000110,
          MEMOP_LDW=7'b1001010,
          MEMOP_LDSUB=7'b1010000,
          MEMOP_LDSUH=7'b1010100,
          MEMOP_LDSUW=7'b1011000,
          MEMOP_LDSB=7'b1010010,
          MEMOP_LDSH=7'b1010110,
          MEMOP_LDSW=7'b1011010,
          MEMOP_LDXUB=7'b1100000,
          MEMOP_LDXUH=7'b1100100,
          MEMOP_LDXUW=7'b1101000,
          MEMOP_LDXB=7'b1100010,
          MEMOP_LDXH=7'b1100110,
          MEMOP_LDXW=7'b1101010,
          MEMOP_STB=7'b1000001,
          MEMOP_STH=7'b1000101,
          MEMOP_STW=7'b1001001,
          MEMOP_STSB=7'b1010001,
          MEMOP_STSH=7'b1010101,
          MEMOP_STSW=7'b1011001,
          MEMOP_STXB=7'b1100001,
          MEMOP_STXH=7'b1100101,
          MEMOP_STXW=7'b1101001;

parameter FLAGOP_AND=3'b000,
          FLAGOP_OR= 3'b001,
          FLAGOP_XOR=3'b010,
          FLAGOP_NOR=3'b011,
          FLAGOP_CLR=3'b100,
          FLAGOP_SET=3'b101;

wire                        [ 9 : 0 ]   ir_op;
wire                        [ 4 : 0 ]   ir_src2;
wire                        [ 4 : 0 ]   ir_src1;
wire                        [ 4 : 0 ]   ir_dst;
wire                                    ir_mask;

// Control register saved values
reg                       pipe_advance_s2_r;
reg   [ VCWIDTH-1 : 0 ]   vc_in_saved;
reg   [ VCWIDTH-1 : 0 ]   vl_in_saved;
reg   [ VCWIDTH-1 : 0 ]   vbase_in_saved;
reg   [ VCWIDTH-1 : 0 ]   vinc_in_saved;
reg   [ VCWIDTH-1 : 0 ]   vstride_in_saved;
reg   [ VSWIDTH-1 : 0 ]   vs_in_saved;

// Control register values
//TODO: Rethink whether the following vars (vc, vl, vbase, vinc, vstride)
//need to be change to use `MAX_PIPE_STAGES. I think these variables are 
//only used for memory operations and for that, we only need 6 stages.
//Not a big deal because most of these are not used much. Synthesis tool
//will optimize out the unnecessary bits anyway.
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((VCWIDTH-1)-(0)+1))-1 : 0] vc;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((VCWIDTH-1)-(0)+1))-1 : 0] vl;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((VCWIDTH-1)-(0)+1))-1 : 0] vbase;
reg   [ VCWIDTH-1 : 0 ]   vbase_s4;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((VCWIDTH-1)-(0)+1))-1 : 0] vinc;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((VCWIDTH-1)-(0)+1))-1 : 0] vstride;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((VSWIDTH-1)-(0)+1))-1 : 0] vs;
reg [(((NUMBANKS-1)-(0)+1)*((VSWIDTH-1)-(0)+1))-1 : 0] vs_s3;
reg [(((NUMFUS-1)-(0)+1)*((VSWIDTH-1)-(0)+1))-1 : 0] vs_s4;
reg [(((NUMBANKS-1)-(0)+1)*((VCWIDTH-1)-(0)+1))-1 : 0] vc_s3;
reg [(((NUMFUS-1)-(0)+1)*((VCWIDTH-1)-(0)+1))-1 : 0] vc_s4;

// Vector register file signals
reg   [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vr_a_reg;
wire     [ NUMBANKS*8*VPW*NUMLANES-1 : 0 ]   _vr_a_readdataout;
reg                         [NUMBANKS-1:0]   vr_a_en;
reg   [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vr_b_reg;
wire     [ NUMBANKS*8*VPW*NUMLANES-1 : 0 ]   _vr_b_readdataout;
reg                         [NUMBANKS-1:0]   vr_b_en;
reg   [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   _vr_c_reg;
reg      [ NUMBANKS*8*VPW*NUMLANES-1 : 0 ]   _vr_c_writedatain;
reg        [ NUMBANKS*VPW*NUMLANES-1 : 0 ]   vr_c_byteen;
reg                         [NUMBANKS-1:0]   vr_c_we;

reg [(((NUMBANKS-1)-(0)+1)*((LANEWIDTH*NUMLANES-1)-(0)+1))-1 : 0] vr_a_readdataout;
reg [(((NUMBANKS-1)-(0)+1)*((LANEWIDTH*NUMLANES-1)-(0)+1))-1 : 0] vr_b_readdataout;
reg [(((NUMBANKS-1)-(0)+1)*((LANEWIDTH*NUMLANES-1)-(0)+1))-1 : 0] vr_c_writedatain;
reg [(((NUMBANKS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] vr_c_reg;

// Flag register file signals
reg       [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vf_a_reg;
wire               [ NUMBANKS*NUMLANES-1 : 0 ]   vf_a_readdataout;
reg                            [ NUMBANKS-1:0]   vf_a_en;
reg       [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vf_b_reg;
wire               [ NUMBANKS*NUMLANES-1 : 0 ]   vf_b_readdataout;
reg                            [ NUMBANKS-1:0]   vf_b_en;
reg       [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vf_c_reg;
reg                [ NUMBANKS*NUMLANES-1 : 0 ]   vf_c_writedatain;
reg                            [ NUMBANKS-1:0]   vf_c_we;

wire [(((NUMFUS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] wb_dst;
wire                     [NUMFUS-1:0]   wb_dst_we;
wire [(((NUMFUS-1)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] wb_dst_mask;

wire [(((NUMFUS-1)-(0)+1)*((`MAX_STAGES-1)-(4)+1))-1 : 0] dst_we;
wire [(((`MAX_STAGES-1)-(4)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] dst[NUMFUS-1:0];
wire [(((`MAX_STAGES-1)-(4)+1)*((NUMLANES-1)-(0)+1))-1 : 0] dst_mask[NUMFUS-1:0];
wire             [ REGIDWIDTH-1 : 0 ]   dst_s2;
reg [(((NUMBANKS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] _dst_s3;
reg [(((NUMBANKS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] dst_s3;
reg     [ NUMBANKS*REGIDWIDTH-1 : 0 ]   t_dst_s3;
reg [(((NUMFUS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] dst_s4;
reg                  [ NUMFUS-1 : 0 ]   dst_we_s4;
reg [(((4)-(4)+1)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1))-1 : 0] alu_dst;
reg [(((4)-(4)+1)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1))-1 : 0] alu_dst_we;
reg [(((4)-(4)+1)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1))-1 : 0] falu_dst;
reg [(((4)-(4)+1)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1))-1 : 0] falu_dst_we;

wire                                    imask_s2;
reg                [ NUMBANKS-1 : 0 ]   imask_s3;

wire             [ REGIDWIDTH-1 : 0 ]   src1_s2;
wire             [ REGIDWIDTH-1 : 0 ]   src2_s2;
reg [(((NUMBANKS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] src1_s3;
reg [(((NUMBANKS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] src2_s3;
reg [(((NUMBANKS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] _src1_s3;
reg [(((NUMBANKS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] _src2_s3;
wire                                    src1scalar_s2;
wire                                    src2scalar_s2;
reg                    [NUMBANKS-1:0]   src1scalar_s3;
reg                    [NUMBANKS-1:0]   src2scalar_s3;
reg                      [NUMFUS-1:0]   src1scalar_s4;
reg                      [NUMFUS-1:0]   src2scalar_s4;

reg [(((NUMBANKS-1)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] lane_en;
reg [(((NUMFUS-1)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] vlane_en;
reg                                     mem_last_subvector_s4;

reg                     [LOG2MVL-1:0]   src_start_delayed;
wire                    [LOG2MVL-1:0]   src_elm;
wire                    [LOG2MVL-1:0]   src_limit;
reg [(((NUMBANKS-1)-(0)+1)*((LOG2MVL-1)-(0)+1))-1 : 0] src_limit_s3;
wire                    [LOG2MVL-1:0]   src_start;

wire                    [VCWIDTH-1:0]   total_shamt;
wire                    [LOG2MVL-1:0]   dst_start;

reg [(((NUMFUS-1)-(0)+1)*((LANEWIDTH*NUMLANES-1)-(0)+1))-1 : 0] vr_src1;
reg [(((NUMFUS-1)-(0)+1)*((LANEWIDTH*NUMLANES-1)-(0)+1))-1 : 0] vr_src2;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   matmul_out;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   bfadder_result_s5;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   bfmult_result_s5;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   act_result_s5;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   trp_out;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   permute_out;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   transpose_out;
reg [(((NUMFUS-1)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] vf_src1;
reg [(((NUMFUS-1)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] vf_src2;
reg [(((NUMFUS-1)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] vmask;
reg [(((NUMBANKS-1)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] vmask_final;

reg        [ VCWIDTH*NUMLANES-1 : 0 ]   vstrideoffset_s4;
wire     [ LANEWIDTH*NUMLANES-1 : 0 ]   load_result_s5;
wire               [ NUMLANES-1 : 0 ]   load_result_mask_s5;

wire [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((LANEWIDTH*NUMLANES-1)-(0)+1))-1 : 0] alu_result_s5;
wire [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] alu_cmpresult_s4;
wire [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] flagalu_result_s4;
wire [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] flagalu_result_s5;
wire [ LANEWIDTH*NUMLANES-1 : 0 ] mulshift_result_s5;

//Support 1 Lane processor
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((((LOG2NUMLANES>0) ? LOG2NUMLANES : 1)-1)-(0)+1))-1 : 0] elmshamt;

//control for TRP unit

wire [1:0] trp_mode_s1,trp_mode_s2;
reg [(((NUMBANKS-1)-(0)+1)*((1)-(0)+1))-1 : 0] trp_mode_s3;
reg [1:0] trp_mode_s4;
wire trp_busy;
wire trp_valid;
wire trp_read;

wire transpose_valid;
wire transpose_busy;

//reg permute_read;
//reg permute_store;
//reg permute_op;

reg ctrl1_vr_a_en; // SRC1
reg ctrl1_vr_b_en; // SRC2
reg ctrl1_vr_c_en; // SRC3
reg ctrl1_vr_d_we; // DEST
reg ctrl1_vf_a_en;
reg ctrl1_vf_b_en;
reg ctrl1_vf_c_we;
reg ctrl1_vs_we;
reg ctrl1_usesvssel;    // 1-if instruction can have .sv/.vs variants
reg [1:0] ctrl1_elmshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
reg [1:0] ctrl1_srcshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
reg ctrl1_srclimit_sel;  //0-vl, 1 vl+vindex
reg [1:0] ctrl1_dstshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
reg ctrl1_setvlto1;
reg ctrl1_mem_dir_left;  //1-left, 0-right
reg ctrl1_rshiftnonzero;
reg [10:0] ctrl1_alu_op;
reg [1:0] ctrl1_satsum_op;
reg [3:0] ctrl1_satsize_op;
reg [4:0] ctrl1_mulshift_op;
reg ctrl1_matmul_en;
reg ctrl1_bfadder_en;
reg ctrl1_bfmult_en;
reg ctrl1_act_en;
reg ctrl1_trp_en;
reg ctrl1_permute_en;
reg ctrl1_permute_op;
reg ctrl1_permute_store;
reg ctrl1_permute_read;
reg ctrl1_transpose_en;
reg ctrl1_axi_en;
reg ctrl1_axi_req_type;
reg ctrl1_memunit_en;
reg ctrl1_mem_en;
reg [6:0] ctrl1_memunit_op;
reg ctrl1_ismasked;
reg [2:0] ctrl1_flagalu_op;
reg ctrl1_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg ctrl1_volatiledest;
reg ctrl1_vrdest_sel;  //0-dest, 1-src2
reg ctrl1_vf_a_sel; //0-0/1 from instr, 1-src1
reg bf_op;

wire [5:4] ctrl_vs_we;
wire [6:0] ctrl_memunit_op[`MAX_PIPE_STAGES-1:1] ;

wire ctrl2_vr_a_en; // SRC1
wire ctrl2_vr_b_en; // SRC2
wire ctrl2_vr_c_we; // DEST
wire ctrl2_vf_a_en;
wire ctrl2_vf_b_en;
wire ctrl2_vf_c_we;
wire ctrl2_vs_we;
wire ctrl2_useslanes;
wire [1:0] ctrl2_elmshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
wire [1:0] ctrl2_srcshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
wire ctrl2_srclimit_sel;  //0-vl, 1 vl+vindex
wire [1:0] ctrl2_dstshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
wire ctrl2_setvlto1;
wire ctrl2_mem_dir_left;  //1-left, 0-right
wire ctrl2_rshiftnonzero;
wire [10:0] ctrl2_alu_op;
wire [1:0] ctrl2_satsum_op;
wire [3:0] ctrl2_satsize_op;
wire [4:0] ctrl2_mulshift_op;
wire ctrl2_memunit_en;
wire ctrl2_mem_en;
wire [6:0] ctrl2_memunit_op;
wire ctrl2_ismasked;
wire [2:0] ctrl2_flagalu_op;
wire ctrl2_vf_wbsel;   //0-flag ALU, 1-normal ALU
wire ctrl2_volatiledest;
wire ctrl2_vf_a_sel; //0-0/1 from instr, 1-src1
wire ctrl2_mulshift_en;
wire ctrl2_matmul_en;
wire ctrl2_bfadder_en;
wire ctrl2_bfmult_en;
wire ctrl2_act_en;
wire ctrl2_trp_en;
wire ctrl2_transpose_en;
wire ctrl2_permute_en;
wire ctrl2_permute_op;
wire ctrl2_permute_store;
wire ctrl2_permute_read;
wire ctrl2_axi_en;
wire ctrl2_axi_req_type;
wire ctrl2_alufalu_en;

reg [NUMBANKS-1:0] ctrl3_vr_a_en; // SRC1
reg [NUMBANKS-1:0] ctrl3_vr_b_en; // SRC2
reg [NUMBANKS-1:0] ctrl3_vr_c_we; // DEST
reg [NUMBANKS-1:0] ctrl3_vf_a_en;
reg [NUMBANKS-1:0] ctrl3_vf_b_en;
reg [NUMBANKS-1:0] ctrl3_vf_c_we;
reg [NUMBANKS-1:0] ctrl3_vs_we;
reg [NUMBANKS-1:0] ctrl3_useslanes;
reg [NUMBANKS-1:0] ctrl3_mem_dir_left;
reg [NUMBANKS-1:0] ctrl3_rshiftnonzero;
reg [(((NUMBANKS-1)-(0)+1)*((10)-(0)+1))-1 : 0] ctrl3_alu_op;
reg [(((NUMBANKS-1)-(0)+1)*((1)-(0)+1))-1 : 0] ctrl3_satsum_op;
reg [(((NUMBANKS-1)-(0)+1)*((3)-(0)+1))-1 : 0] ctrl3_satsize_op;
reg [(((NUMBANKS-1)-(0)+1)*((4)-(0)+1))-1 : 0] ctrl3_mulshift_op;
reg [NUMBANKS-1:0] ctrl3_memunit_en;
reg [NUMBANKS-1:0] ctrl3_mem_en;
reg [(((NUMBANKS-1)-(0)+1)*((6)-(0)+1))-1 : 0] ctrl3_memunit_op;
reg [NUMBANKS-1:0] ctrl3_ismasked;
reg [(((NUMBANKS-1)-(0)+1)*((2)-(0)+1))-1 : 0] ctrl3_flagalu_op;
reg [NUMBANKS-1:0] ctrl3_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg [NUMBANKS-1:0] ctrl3_volatiledest;
reg [NUMBANKS-1:0] ctrl3_vf_a_sel; //0-0/1 from instr, 1-src1
reg [NUMBANKS-1:0] ctrl3_mulshift_en;
reg [NUMBANKS-1:0] ctrl3_matmul_en;
reg [NUMBANKS-1:0] ctrl3_trp_en;
reg [NUMBANKS-1:0] ctrl3_transpose_en;
reg [NUMBANKS-1:0] ctrl3_permute_en;
reg [NUMBANKS-1:0] ctrl3_permute_op;
reg [NUMBANKS-1:0] ctrl3_permute_read;
reg [NUMBANKS-1:0] ctrl3_permute_store;
reg [NUMBANKS-1:0] ctrl3_axi_en;
reg [NUMBANKS-1:0] ctrl3_axi_req_type;
reg [NUMBANKS-1:0] ctrl3_bfadder_en;
reg [NUMBANKS-1:0] ctrl3_bfmult_en;
reg [NUMBANKS-1:0] ctrl3_act_en;
reg [NUMBANKS-1:0] ctrl3_alufalu_en;

reg ctrl4_mem_dir_left;
reg ctrl4_rshiftnonzero;
reg [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((10)-(0)+1))-1 : 0] ctrl4_alu_op;
reg [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((1)-(0)+1))-1 : 0] ctrl4_satsum_op;
reg [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((3)-(0)+1))-1 : 0] ctrl4_satsize_op;
reg [4:0] ctrl4_mulshift_op;
reg ctrl4_memunit_en;
reg ctrl4_mem_en;
reg [6:0] ctrl4_memunit_op;
reg [NUMFUS-1:0] ctrl4_ismasked;
reg [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((2)-(0)+1))-1 : 0] ctrl4_flagalu_op;
//reg ctrl4_vf_wbsel[(NUMBANKS-1)*ALUPERBANK:0];   //0-flag ALU, 1-normal ALU
reg [(NUMBANKS-1)*ALUPERBANK:0] ctrl4_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg ctrl4_volatiledest;
//reg ctrl4_vf_a_sel[(NUMBANKS-1)*ALUPERBANK:0]; //0-0/1 from instr, 1-src1
reg [(NUMBANKS-1)*ALUPERBANK:0] ctrl4_vf_a_sel; //0-0/1 from instr, 1-src1
reg ctrl4_mulshift_en;
reg ctrl4_matmul_en;
reg ctrl4_trp_en;
reg ctrl4_transpose_en;
reg ctrl4_permute_en;
reg ctrl4_permute_op;
reg ctrl4_permute_store;
reg ctrl4_permute_read;
reg ctrl4_bfadder_en;
reg ctrl4_bfmult_en;
reg ctrl4_axi_en;
reg ctrl4_axi_req_type;
reg ctrl4_act_en;

wire ctrl5_mem_en;

wire [`VRELM_RANGE] regid_pad;

integer bd;
genvar  ba;
integer bi;
integer i;
integer j;
genvar  bk;
genvar  k;
integer m;
integer n;
integer b;
integer b3;
integer f3;
integer bn;
integer fn;
integer fn2;
integer bw;

wire [NUMBANKS*LOG2MVL-1:0] rdelm;
wire [NUMBANKS*LOG2MVL-1:0] wrelm;
// MSb of count entries indicates if instruction is dead or not.
wire [NUMBANKS*(LOG2MVL-LOG2NUMLANES+1)-1:0] count;
reg  [NUMBANKS-1:0]   last_subvector;
wire [NUMBANKS-1:0]   first_subvector;
reg  [NUMBANKS-1:0] wrongbank_s3;
reg  [NUMBANKS-1:0] alive_s3;
reg [(((NUMFUS-1)-(0)+1)*((NUMBANKS-1)-(0)+1))-1 : 0] banksel_s4;

wire dispatcher_shift;
wire dispatcher_rotate;

wire [`MAX_STAGES-1:0] internal_pipe_advance;
wire [`MAX_STAGES-1:0] pipe_advance;
wire [`MAX_STAGES-1:0] pipe_squash;
wire stall_srcstart;
wire stall_dispatcher;
wire stall_hazsrc1;
wire stall_hazsrc2;
wire stall_hazfsrc1;
wire stall_hazfsrc2;
wire stall_memunit;
wire _stall_memunit;
wire stall_mulcooldown;
wire stall_mulunit;
wire stall_matmul;

wire temp_stall_matmul;
// DEBUG signals for Modelsim
wire [(((2)-(1)+1)*((7)-(0)+1))-1 : 0] D_instr;
reg [(((NUMBANKS-1)-(0)+1)*((7)-(0)+1))-1 : 0] D_instr_s3;
reg [(((NUMFUS-1)-(0)+1)*((7)-(0)+1))-1 : 0] D_instr_s4;
wire [(((NUMFUS-1)-(0)+1)*((7)-(0)+1))-1 : 0] D_instr_s5;
wire [(((NUMFUS-1)-(0)+1)*((7)-(0)+1))-1 : 0] D_instr_s6;
reg   [NUMFUS-1:0] D_last_subvector_s4;
wire  [NUMFUS-1:0] D_last_subvector_s5;
wire  [NUMFUS-1:0] D_last_subvector_s6;
wire  [NUMFUS-1:0] D_last_subvector_s31;
wire  [NUMFUS-1:0] D_wb_last_subvector;
reg   [NUMBANKS-1:0] D_last_subvector_done;
reg   [NUMBANKS-1:0] D_wb_instrdone;

assign trp_mode_s1 = 2'b00;
  pipe #(8,1) debuginstrpipe (
      .d( {instr[25:24],instr[5:0]} ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[2:1] ),
      .squash( pipe_squash[2:1] ),
      .q( {D_instr[((2-1)*((7)-(0)+1)+7-0) : ((2-1)*((7)-(0)+1))],D_instr[((1-1)*((7)-(0)+1)+7-0) : ((1-1)*((7)-(0)+1))]}));

  genvar Df;
  generate
  for (Df=0; Df<NUMFUS; Df=Df+1)
  begin : Debug_gen

    pipereg #(8) debugintrfupipereg1 (
      .d( D_instr_s4[((Df-0)*((7)-(0)+1)+7-0) : ((Df-0)*((7)-(0)+1))] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s5[((Df-0)*((7)-(0)+1)+7-0) : ((Df-0)*((7)-(0)+1))]));

    pipereg #(8) debugintrfupipereg2 (
      .d( D_instr_s5[((Df-0)*((7)-(0)+1)+7-0) : ((Df-0)*((7)-(0)+1))] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s6[((Df-0)*((7)-(0)+1)+7-0) : ((Df-0)*((7)-(0)+1))]));

    pipereg #(1) debuglastpipereg1 (
      .d( D_last_subvector_s4[Df] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s5[Df]));

    pipereg #(1) debuglastpipereg2 (
      .d( D_last_subvector_s5[Df] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s6[Df]));

  end
  endgenerate


//tell vpu to hold vc,vbase,etc values
assign is_stalled=~internal_pipe_advance | {stall_srcstart,2'b0};

assign has_memop=ctrl1_mem_en|ctrl2_mem_en|(|ctrl3_mem_en)|ctrl4_mem_en|ctrl5_mem_en;

/******************************************************************************/
/************************** Inter-Pipe Signals ********************************/
/******************************************************************************/

/* Memunit is in stage 5 but we stall in stage 4 so we don't squash the
* destination register contents*/

assign pipe_advance=internal_pipe_advance & ~{5'b0,stall_in};

assign internal_pipe_advance[0]=internal_pipe_advance[1];
assign internal_pipe_advance[1]=internal_pipe_advance[2];
assign internal_pipe_advance[2]=internal_pipe_advance[3] && 
                          ~stall_srcstart &&
                          ~stall_dispatcher &&
                          ~stall_hazsrc1 && ~stall_hazsrc2 && 
                          ~stall_hazfsrc1 && ~stall_hazfsrc2 && 
                          ~stall_mulcooldown; 
assign internal_pipe_advance[3]=internal_pipe_advance[4];
assign internal_pipe_advance[4]=internal_pipe_advance[5];
assign internal_pipe_advance[5]=internal_pipe_advance[6] && ~stall_mulunit && ~stall_memunit && ~temp_stall_matmul;
//assign internal_pipe_advance[6]=1'b1;

assign pipe_squash[0]=pipe_advance[1]&~pipe_advance[0];
assign pipe_squash[1]=pipe_advance[2]&~pipe_advance[1];
assign pipe_squash[2]=pipe_advance[3]&~pipe_advance[2];
assign pipe_squash[3]=pipe_advance[4]&~pipe_advance[3];
assign pipe_squash[4]=pipe_advance[5]&~pipe_advance[4];
assign pipe_squash[5]=pipe_advance[6]&~pipe_advance[5];
assign pipe_squash[6]=1'b0;
assign pipe_squash[13:7]= 'h0;
assign internal_pipe_advance[13:6] = {(14-5){1'b1}};
//assign pipe_squash[`MAX_STAGES:7]= 'h0;
//assign internal_pipe_advance[`MAX_STAGES:6] = {(`MAX_STAGES-5){1'b1}};
//assign pipe_squash[6]=1'b0;

/******************************************************************************/
/************************** 1st Pipeline Stage ********************************/
/******************************************************************************/

  assign ir_op={instr[25:22],instr[5:0]}; //10 bits
  assign ir_dst=instr[10:6];
  assign ir_src1=instr[15:11];
  assign ir_src2=instr[20:16];
  assign ir_mask=instr[21];

  // Determine which instruction read from which ports
  always@*
  begin
    ctrl1_vr_a_en=0;
    ctrl1_vr_b_en=0;
    ctrl1_vr_c_en=0;
    ctrl1_vf_a_en=0;
    ctrl1_vf_b_en=0;
    ctrl1_vf_a_sel=0;
    ctrl1_usesvssel=0;
    case(ir_op)
    COP2_VADD:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VADD_U:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VSUB:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VSUB_U:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VMULHI:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VMULHI_U:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    //  COP2_VMOD,
    COP2_VMOD_U:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VCMP_EQ:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VCMP_NE:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VCMP_LT:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VCMP_U_LT:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VCMP_LE:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VCMP_U_LE:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VDIV:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VDIV_U:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VBFADD:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VBFMULT:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VMIN:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VMIN_U:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VMAX:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VMAX_U:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VMULLO:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VABS:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=1;
        end
    COP2_VAND:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VOR:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXOR:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VNOR:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VSLL:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VSRL:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VSRA:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
     //COP2_VSAT_W,
      //COP2_VSAT_SU_W,
      //COP2_VSAT_U_W:
    COP2_VSAT_B:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSAT_H:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VACT:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSAT_SU_B:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSAT_SU_H:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VRED:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSAT_SU_L:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSAT_U_B:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSAT_U_H:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VTRP:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSADD:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VSADD_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VSSUB:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VSSUB_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VSRR:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSRR_U:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSLS:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSLS_U:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VXUMUL:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXUMUL_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXLMUL:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXLMUL_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXUMADD:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXUMADD_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXUMSUB:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXUMSUB_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXLMADD:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXLMADD_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXLMSUB:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXLMSUB_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VINS_VV:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vr_b_en=1;
        end
      //COP2_VINS_SV: doesn't read any vectors or flags
    COP2_VEXT_VV:
        begin
          ctrl1_vr_a_en=1;
        end
    COP2_VEXT_SV:
        begin
          ctrl1_vr_a_en=1;
        end
    COP2_VEXT_U_SV:
        begin
          ctrl1_vr_a_en=1;
        end
    COP2_VCOMPRESS:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VEXPAND:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VMERGE:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
      //COP2_VFINS:
    COP2_VEXTHALF:
        begin
          ctrl1_vr_a_en=1;
        end
    COP2_VHALF:
        begin
          ctrl1_vr_a_en=1;
        end
    COP2_VHALFUP:
        begin
          ctrl1_vr_a_en=1;
        end
    COP2_VHALFDN:
        begin
          ctrl1_vr_a_en=1;
        end
      //COP2_VSATVL:
    COP2_VFAND:
        begin
          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vf_b_en=1;
          ctrl1_vf_a_sel=1;
          ctrl1_usesvssel=1;
        end
    COP2_VFOR:
        begin
          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vf_b_en=1;
          ctrl1_vf_a_sel=1;
          ctrl1_usesvssel=1;
        end
    COP2_VFXOR:
        begin
          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vf_b_en=1;
          ctrl1_vf_a_sel=1;
          ctrl1_usesvssel=1;
        end
    COP2_VFNOR:
        begin
          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vf_b_en=1;
          ctrl1_vf_a_sel=1;
          ctrl1_usesvssel=1;
        end
      //COP2_VFCLR:
      //COP2_VFSET:
    COP2_VIOTA:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
    COP2_VCIOTA:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
    COP2_VFPOP:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
    COP2_VFFF1:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
    COP2_VFFL1:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
    COP2_VFSETBF:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
    COP2_VFSETIF:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
    COP2_VFSETOF:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
      //COP2_VFMT8:
      //COP2_VFMF8:
      //COP2_VFCLR8:
      //COP2_VFOR8:
      //COP2_VFLD:
      //COP2_VBFADD,
      //COP2_VLD_U_W,
      //COP2_VBFMULT:
    COP2_VLD_B:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLD_H:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLD_L:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLD_U_B:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLD_U_H:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLDS_B:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLDS_H:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VBFSUB:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLDS_L:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLDS_U_B:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLDS_U_H:
        begin
          ctrl1_vf_a_en=1;
        end
      //COP2_VTRP,
      //COP2_VRED,
     // COP2_VACT:
    COP2_VLDX_B:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
        end
    COP2_VLDX_H:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
        end
    COP2_VPER_STR:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
        end
    COP2_VLDX_L:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
        end
    COP2_VLDX_U_B:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
        end
    COP2_VLDX_U_H:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
        end
      //COP2_VFST:
    COP2_VST_B:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_b_en=1;
        end
    COP2_VST_H:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_b_en=1;
        end
    COP2_VST_L:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_b_en=1;
        end
    COP2_VSTS_B:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_b_en=1;
        end
    COP2_VSTS_H:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_b_en=1;
        end
    COP2_VSTS_L:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_b_en=1;
        end
    COP2_VPER:
         begin
          ctrl1_vr_b_en=1;
         end
    COP2_VPER_LD:
         begin
          ctrl1_vr_b_en=1;
         end
    COP2_VAXIWR:
         begin
           ctrl1_vr_b_en=1;
         end
    COP2_VAXIRD:
         begin
           ctrl1_vr_c_en=1;
         end
      //COP2_VSTX_W,
      //COP2_VSTXO_W,  AXI Write
    COP2_VSTX_B:
        begin
          ctrl1_vr_b_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSTX_H:
        begin
          ctrl1_vr_b_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSTX_L:
        begin
          ctrl1_vr_b_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSTXO_B:
        begin
          ctrl1_vr_b_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSTXO_H:
        begin
          ctrl1_vr_b_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSTXO_L:
        begin
          ctrl1_vr_b_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_vf_a_en=1;
        end
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  // Decode enables 
  always@*
  begin
    ctrl1_vr_d_we=0;
    ctrl1_vf_c_we=0;
    ctrl1_vs_we=0;
    ctrl1_vrdest_sel=0;
    ctrl1_elmshamt_sel=0;
    ctrl1_srcshamt_sel=0;
    ctrl1_srclimit_sel=0;
    ctrl1_dstshamt_sel=0;
    ctrl1_mem_dir_left=0;
    ctrl1_rshiftnonzero=0;
    ctrl1_memunit_en=0;
    ctrl1_mem_en=0;
    ctrl1_ismasked=0;
    ctrl1_setvlto1=0;
    ctrl1_vf_wbsel=0;
    ctrl1_volatiledest=0;
    ctrl1_permute_store = 1'b0;  // Loading data into the permute block
    ctrl1_permute_op = 1'b0; // processing the permute op
    case(ir_op)
    //  COP2_VMOD,
    COP2_VADD:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VADD_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSUB:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSUB_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VMULHI:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VMULHI_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VDIV:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VDIV_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VBFADD:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VBFMULT:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VMOD_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
     // COP2_VACT,
    COP2_VTRP:
         begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
         end
    COP2_VPER_STR:
         begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
         end
    COP2_VRED:
         begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
         end
    COP2_VCMP_EQ:
        begin
          ctrl1_vf_c_we=1;
          ctrl1_vf_wbsel=1;
          ctrl1_ismasked=1;
        end
    COP2_VCMP_NE:
        begin
          ctrl1_vf_c_we=1;
          ctrl1_vf_wbsel=1;
          ctrl1_ismasked=1;
        end
    COP2_VCMP_LT:
        begin
          ctrl1_vf_c_we=1;
          ctrl1_vf_wbsel=1;
          ctrl1_ismasked=1;
        end
    COP2_VCMP_U_LT:
        begin
          ctrl1_vf_c_we=1;
          ctrl1_vf_wbsel=1;
          ctrl1_ismasked=1;
        end
    COP2_VCMP_LE:
        begin
          ctrl1_vf_c_we=1;
          ctrl1_vf_wbsel=1;
          ctrl1_ismasked=1;
        end
    COP2_VCMP_U_LE:
        begin
          ctrl1_vf_c_we=1;
          ctrl1_vf_wbsel=1;
          ctrl1_ismasked=1;
        end
      //COP2_VSAT_W,
      //COP2_VSAT_SU_W,
      //COP2_VSAT_U_W,
    COP2_VMIN:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VMIN_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VMAX:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VMAX_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VMULLO:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VABS:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VAND:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VOR:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VXOR:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VNOR:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSLL:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSRL:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSRA:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSAT_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSAT_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VACT:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSAT_SU_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSAT_SU_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VRED:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSAT_SU_L:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSAT_U_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSAT_U_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSADD:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSADD_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSSUB:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSSUB_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSRR:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSRR_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSLS:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSLS_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VXUMUL:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXUMUL_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXLMUL:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXLMUL_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXUMADD:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXUMADD_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXUMSUB:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXUMSUB_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXLMADD:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXLMADD_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXLMSUB:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXLMSUB_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VINS_VV:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=0;
          ctrl1_dstshamt_sel=3;
          ctrl1_mem_dir_left=1;
          ctrl1_volatiledest=1;
        end
    COP2_VINS_SV:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_elmshamt_sel=0;
          ctrl1_srcshamt_sel=0;
          ctrl1_dstshamt_sel=3;
          ctrl1_mem_dir_left=1;
          ctrl1_setvlto1=1;
        end
    COP2_VEXT_VV:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=3;
          ctrl1_srclimit_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_volatiledest=1;
        end
    COP2_VEXT_SV:
        begin
          ctrl1_vs_we=1;
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=3;
          ctrl1_srclimit_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_setvlto1=1;
        end
    COP2_VEXT_U_SV:
        begin
          ctrl1_vs_we=1;
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=3;
          ctrl1_srclimit_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_setvlto1=1;
        end
    COP2_VCOMPRESS:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_mem_dir_left=0;
          ctrl1_ismasked=1;
          ctrl1_volatiledest=1;
        end
    COP2_VEXPAND:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_mem_dir_left=1;
          ctrl1_ismasked=1;
          ctrl1_volatiledest=1;
        end
    COP2_VMERGE:
        begin
          ctrl1_vr_d_we=1;
        end
    COP2_VFINS:
        begin
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=0;
          ctrl1_mem_dir_left=1;
        end
    COP2_VEXTHALF:
        begin
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=1;
          ctrl1_srcshamt_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_vr_d_we=1;
          ctrl1_volatiledest=1;
        end
    COP2_VHALF:
        begin
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=1;
          ctrl1_srcshamt_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_vr_d_we=1;
          ctrl1_volatiledest=1;
        end
    COP2_VHALFUP:
        begin
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=2;
          ctrl1_srcshamt_sel=2;
          ctrl1_mem_dir_left=0;
          ctrl1_vr_d_we=1;
          ctrl1_volatiledest=1;
        end
    COP2_VHALFDN:
        begin
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=2;
          ctrl1_srcshamt_sel=0;
          ctrl1_dstshamt_sel=2;
          ctrl1_mem_dir_left=1;
          ctrl1_vr_d_we=1;
          ctrl1_volatiledest=1;
        end
      //COP2_VSATVL:
    COP2_VFAND:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFOR:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFXOR:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFNOR:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFCLR:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFSET:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VIOTA:
        begin
          ctrl1_vr_d_we=1;
        end
    COP2_VCIOTA:
        begin
          ctrl1_vr_d_we=1;
        end
    COP2_VFPOP:
        begin
          ctrl1_vs_we=1;
        end
    COP2_VFFF1:
        begin
          ctrl1_vs_we=1;
        end
    COP2_VFFL1:
        begin
          ctrl1_vs_we=1;
        end
    COP2_VFSETBF:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFSETIF:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFSETOF:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFMT8:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFMF8:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFCLR8:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFOR8:
        begin
          ctrl1_vf_c_we=1;
        end
      //COP2_VFLD,
      //COP2_VBFADD,
      //COP2_VLD_U_W,
      //COP2_VBFMULT:
    COP2_VLD_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLD_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLD_L:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLD_U_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLD_U_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDS_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDS_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VBFSUB:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDS_L:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDS_U_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDS_U_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
     // COP2_VTRP,
     // COP2_VACT:
    COP2_VLDX_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDX_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDX_L:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDX_U_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDX_U_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
      //COP2_VFST:
      //COP2_VRED,
      //COP2_VPER,
      //COP2_VSTX_W,
      //COP2_VSTXO_W,  AXI write
    COP2_VST_B:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VST_H:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VST_L:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTS_B:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTS_H:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTS_L:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTX_B:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTX_H:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTX_L:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTXO_B:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTXO_H:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTXO_L:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VPER_STR:
         begin
           ctrl1_ismasked = 1'b1;
         end
    COP2_VPER:
         begin
           ctrl1_permute_op = 1'b1;
           ctrl1_ismasked = 1'b1;
         end
    COP2_VPER_LD:
         begin
           ctrl1_permute_store = 1'b1;
           ctrl1_ismasked = 1'b1;
         end
    COP2_VAXIWR:
         begin
           ctrl1_ismasked = 1'b1;
         end
    COP2_VAXIRD:
         begin
           ctrl1_ismasked = 1'b1;
         end
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  // Decode instructions FU op codes
  initial
  begin
    ctrl1_alu_op=ALUOP_ZERO^ALUOP_ZERO;  //Aid subsetting by setting to zero
    ctrl1_satsum_op=SATSUMOP_NOP;
    ctrl1_satsize_op=SATSIZEOP_VSATUW;
    ctrl1_mulshift_op=MULOP_ZERO;
    ctrl1_matmul_en=0;
    ctrl1_trp_en = 1'b0;
    ctrl1_permute_en = 1'b0;
    ctrl1_transpose_en = 1'b0;
    ctrl1_bfadder_en = 1'b0;
    ctrl1_bfmult_en = 1'b0;
    ctrl1_act_en = 1'b0;
    ctrl1_axi_en = 1'b0;
    ctrl1_axi_req_type = 1'b0;
    ctrl1_memunit_op=0;
    ctrl1_flagalu_op=FLAGOP_CLR;
    ctrl1_permute_read = 1'b0;
  end
  always@*
  begin
    ctrl1_alu_op=ALUOP_ZERO^ALUOP_ZERO;
    ctrl1_satsum_op=SATSUMOP_NOP;
    ctrl1_satsize_op=SATSIZEOP_VSATUW;
    ctrl1_mulshift_op=MULOP_ZERO;
    ctrl1_memunit_op=0;
    ctrl1_matmul_en=0;
    ctrl1_bfadder_en = 1'b0;
    ctrl1_bfmult_en = 1'b0;
    ctrl1_act_en = 1'b0;
    ctrl1_trp_en = 1'b0;
    ctrl1_transpose_en = 1'b0;
    ctrl1_permute_en = 1'b0;
    ctrl1_permute_read = 1'b0;
    ctrl1_axi_en = 1'b0;
    ctrl1_axi_req_type = 1'b0;
    ctrl1_flagalu_op=FLAGOP_CLR;
    case(ir_op)
      COP2_VADD:      ctrl1_alu_op=ALUOP_ADD^ALUOP_ZERO;
      COP2_VADD_U:    ctrl1_alu_op=ALUOP_ADDU^ALUOP_ZERO;
      COP2_VSUB:      ctrl1_alu_op=ALUOP_SUB^ALUOP_ZERO;
      COP2_VSUB_U:    ctrl1_alu_op=ALUOP_SUBU^ALUOP_ZERO;
      COP2_VMULHI:  ctrl1_mulshift_op=MULOP_MULHI;
      COP2_VMULHI_U:ctrl1_mulshift_op=MULOP_MULUHI;
      COP2_VDIV:    ctrl1_matmul_en=1'b1;  //Note: This is a hack. We're using VDIV opcode for matmul operation
      COP2_VBFADD:    ctrl1_bfadder_en = 1'b1;
      COP2_VBFMULT:   ctrl1_bfmult_en = 1'b1;
      COP2_VACT:      ctrl1_act_en = 1'b1;
      COP2_VTRP:      ctrl1_transpose_en = 1'b1;
      COP2_VRED:      ctrl1_trp_en = 1'b1;
      //COP2_VPER:      ctrl1_permute_en = 1'b1;
      //COP2_VDIV_U,
      //COP2_VMOD,
      //COP2_VMOD_U,
      COP2_VCMP_EQ:   ctrl1_alu_op=ALUOP_CMP_EQ^ALUOP_ZERO;
      COP2_VCMP_NE:   ctrl1_alu_op=ALUOP_CMP_NEQ^ALUOP_ZERO;
      COP2_VCMP_LT:   ctrl1_alu_op=ALUOP_CMP_LT^ALUOP_ZERO;
      COP2_VCMP_U_LT: ctrl1_alu_op=ALUOP_CMP_LT_U^ALUOP_ZERO;
      COP2_VCMP_LE:   ctrl1_alu_op=ALUOP_CMP_LE^ALUOP_ZERO;
      COP2_VCMP_U_LE: ctrl1_alu_op=ALUOP_CMP_LE_U^ALUOP_ZERO;
      COP2_VMIN:      ctrl1_alu_op=ALUOP_MIN^ALUOP_ZERO;
      COP2_VMIN_U:    ctrl1_alu_op=ALUOP_MIN_U^ALUOP_ZERO;
      COP2_VMAX:      ctrl1_alu_op=ALUOP_MAX^ALUOP_ZERO;
      COP2_VMAX_U:    ctrl1_alu_op=ALUOP_MAX_U^ALUOP_ZERO;
      COP2_VMULLO:    ctrl1_mulshift_op=MULOP_MULLO;
      COP2_VABS:      ctrl1_alu_op=ALUOP_ABS^ALUOP_ZERO;
      COP2_VAND:      ctrl1_alu_op=ALUOP_AND^ALUOP_ZERO;
      COP2_VOR:       ctrl1_alu_op=ALUOP_OR^ALUOP_ZERO;
      COP2_VXOR:      ctrl1_alu_op=ALUOP_XOR^ALUOP_ZERO;
      COP2_VNOR:      ctrl1_alu_op=ALUOP_NOR^ALUOP_ZERO;
      COP2_VSLL: ctrl1_mulshift_op=MULOP_SLL;
      COP2_VSRL: ctrl1_mulshift_op=MULOP_SRL;
      COP2_VSRA: ctrl1_mulshift_op=MULOP_SRA;
      COP2_VSAT_B: ctrl1_satsize_op=SATSIZEOP_VSATB;
      COP2_VSAT_H: ctrl1_satsize_op=SATSIZEOP_VSATH;
     // COP2_VSAT_W: ctrl1_satsize_op=SATSIZEOP_VSATW;
      COP2_VSAT_SU_B: ctrl1_satsize_op=SATSIZEOP_VSATSUB;
      COP2_VSAT_SU_H: ctrl1_satsize_op=SATSIZEOP_VSATSUH;
      //COP2_VSAT_SU_W: ctrl1_satsize_op=SATSIZEOP_VSATSUW;
      //COP2_VSAT_SU_L:
      COP2_VSAT_U_B: ctrl1_satsize_op=SATSIZEOP_VSATUB;
      COP2_VSAT_U_H: ctrl1_satsize_op=SATSIZEOP_VSATUH;
      //COP2_VSAT_U_W: ctrl1_satsize_op=SATSIZEOP_VSATUW;
      COP2_VSADD: begin ctrl1_alu_op=ALUOP_ADD^ALUOP_ZERO; ctrl1_satsum_op=SATSUMOP_VS; end
      COP2_VSADD_U: begin 
          ctrl1_alu_op=ALUOP_ADDU^ALUOP_ZERO; 
          ctrl1_satsum_op=SATSUMOP_VSU; 
        end
      COP2_VSSUB: begin ctrl1_alu_op=ALUOP_SUB^ALUOP_ZERO; ctrl1_satsum_op=SATSUMOP_VS; end
      COP2_VSSUB_U: begin 
          ctrl1_alu_op=ALUOP_SUBU^ALUOP_ZERO;
          ctrl1_satsum_op=SATSUMOP_VSU; 
        end
      COP2_VSRR: ctrl1_mulshift_op=MULOP_SRA;
      COP2_VSRR_U: ctrl1_mulshift_op=MULOP_SRL;
      COP2_VSLS: ctrl1_mulshift_op=MULOP_SLS;
      COP2_VSLS_U: ctrl1_mulshift_op=MULOP_SLSU;
      COP2_VXUMUL: ctrl1_mulshift_op=MULOP_UMUL;
      COP2_VXUMUL_U: ctrl1_mulshift_op=MULOP_UMULU;
      COP2_VXLMUL: ctrl1_mulshift_op=MULOP_LMUL;
      COP2_VXLMUL_U: ctrl1_mulshift_op=MULOP_LMULU;
      COP2_VXUMADD: ctrl1_mulshift_op=MULOP_UMUL;
      COP2_VXUMADD_U: ctrl1_mulshift_op=MULOP_UMULU;
      COP2_VXUMSUB: ctrl1_mulshift_op=MULOP_UMUL;
      COP2_VXUMSUB_U: ctrl1_mulshift_op=MULOP_UMULU;
      COP2_VXLMADD: ctrl1_mulshift_op=MULOP_LMUL;
      COP2_VXLMADD_U: ctrl1_mulshift_op=MULOP_LMULU;
      COP2_VXLMSUB: ctrl1_mulshift_op=MULOP_LMUL;
      COP2_VXLMSUB_U: ctrl1_mulshift_op=MULOP_LMULU;
      COP2_VINS_VV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VINS_SV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VEXT_VV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VEXT_SV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VEXT_U_SV: ctrl1_memunit_op=MEMOP_SHIFT;
      //COP2_VCOMPRESS:
      //COP2_VEXPAND:
      COP2_VMERGE:      ctrl1_alu_op=ALUOP_MERGE^ALUOP_ZERO;
      //COP2_VFINS:
      COP2_VEXTHALF: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VHALF: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VHALFUP: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VHALFDN: ctrl1_memunit_op=MEMOP_SHIFT;
      //COP2_VSATVL:
      COP2_VFAND: ctrl1_flagalu_op=FLAGOP_AND;
      COP2_VFOR: ctrl1_flagalu_op=FLAGOP_OR;
      COP2_VFXOR: ctrl1_flagalu_op=FLAGOP_XOR;
      COP2_VFNOR: ctrl1_flagalu_op=FLAGOP_NOR;
      COP2_VFCLR: ctrl1_flagalu_op=FLAGOP_CLR;
      COP2_VFSET: ctrl1_flagalu_op=FLAGOP_SET;
      //COP2_VIOTA,
      //COP2_VCIOTA:
      //COP2_VFPOP,
      //COP2_VFFF1,
      //COP2_VFFL1:
      //COP2_VFSETBF,
      //COP2_VFSETIF,
      //COP2_VFSETOF,
      //COP2_VFMT8,
      //COP2_VFMF8,
      //COP2_VFCLR8,
      //COP2_VFOR8:
      //COP2_VFLD:
      COP2_VLD_B: ctrl1_memunit_op=MEMOP_LDB;
      COP2_VLD_H: ctrl1_memunit_op=MEMOP_LDH;
      //COP2_VBFADD: ctrl1_memunit_op=MEMOP_LDW;
      //COP2_VLD_L,
      COP2_VLD_U_B: ctrl1_memunit_op=MEMOP_LDUB;
      COP2_VLD_U_H: ctrl1_memunit_op=MEMOP_LDUH;
      //COP2_VLD_U_W: ctrl1_memunit_op=MEMOP_LDUW;
      COP2_VLDS_B: ctrl1_memunit_op=MEMOP_LDSB;
      COP2_VLDS_H: ctrl1_memunit_op=MEMOP_LDSH;
      COP2_VBFSUB: ctrl1_memunit_op=MEMOP_LDSW;
      //COP2_VLDS_L:
      COP2_VLDS_U_B: ctrl1_memunit_op=MEMOP_LDSUB;
      COP2_VLDS_U_H: ctrl1_memunit_op=MEMOP_LDSUH;
      //COP2_VBFMULT: ctrl1_memunit_op=MEMOP_LDSUW;
      COP2_VLDX_B: ctrl1_memunit_op=MEMOP_LDXB;
      COP2_VLDX_H: ctrl1_memunit_op=MEMOP_LDXH;
      //COP2_VTRP: ctrl1_memunit_op=MEMOP_LDXW;
      //COP2_VLDX_L:
      COP2_VLDX_U_B: ctrl1_memunit_op=MEMOP_LDXUB;
      COP2_VLDX_U_H: ctrl1_memunit_op=MEMOP_LDXUH;
      //COP2_VACT: ctrl1_memunit_op=MEMOP_LDXUW;
      //COP2_VFST:
      COP2_VST_B: ctrl1_memunit_op=MEMOP_STB;
      COP2_VST_H: ctrl1_memunit_op=MEMOP_STH;
      //COP2_VRED: ctrl1_memunit_op=MEMOP_STW;
      //COP2_VST_L:
      COP2_VSTS_B: ctrl1_memunit_op=MEMOP_STSB;
      COP2_VSTS_H: ctrl1_memunit_op=MEMOP_STSH;
      //COP2_VPER: ctrl1_memunit_op=MEMOP_STSW;
    COP2_VPER_STR:
        begin
           ctrl1_permute_read = 1'b1;
           ctrl1_permute_en = 1'b1;
        end
      COP2_VPER: ctrl1_permute_en = 1'b1;
      COP2_VPER_LD: ctrl1_permute_en = 1'b1;
      //COP2_VSTS_L:
      COP2_VSTX_B: ctrl1_memunit_op=MEMOP_STXB;
      COP2_VSTX_H: ctrl1_memunit_op=MEMOP_STXH;
      //COP2_VSTX_W: ctrl1_memunit_op=MEMOP_STXW;
      //COP2_VSTX_L:
      COP2_VSTXO_B: ctrl1_memunit_op=MEMOP_STXB;
      COP2_VSTXO_H: ctrl1_memunit_op=MEMOP_STXH;
      //COP2_VSTXO_W: ctrl1_memunit_op=MEMOP_STXW; AXI Write
    COP2_VAXIWR:
             begin 
                ctrl1_axi_en = 1'b1; 
                ctrl1_axi_req_type=1'b1;
             end
      COP2_VAXIRD:begin ctrl1_axi_en = 1'b1; ctrl1_axi_req_type=1'b0; end
      //COP2_VSTXO_L:
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  assign regid_pad=0;

  pipereg 
    #(
      2+
      2+
      1+
      2+
      1
    ) pipe1reg_secondstageonly (
      .d( {
        ctrl1_elmshamt_sel,
        ctrl1_srcshamt_sel,
        ctrl1_srclimit_sel,
        ctrl1_dstshamt_sel,
        ctrl1_setvlto1
      }),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[1] ),
      .squashn( 1'b1 ),
      .q({
        ctrl2_elmshamt_sel,
        ctrl2_srcshamt_sel,
        ctrl2_srclimit_sel, //used to get squashed, pretend doesn't need to
        ctrl2_dstshamt_sel,
        ctrl2_setvlto1
      }));

  // *********** Pipeline signals that need to be squashed *********
  pipereg 
    #(
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      3+
      1+
      1+
      2+ // For permute and transpose enable
      3+1+1  // For Permute store,read and op signals + 1 for axi en + 1 axi req tyoe
    ) pipe1regwsquash (
      .d( {
        ctrl1_vr_d_we,
        ctrl1_vf_c_we,
        ctrl1_vs_we,
        ctrl1_vr_a_en|ctrl1_vr_c_en,
        ctrl1_vr_b_en,
        ctrl1_vf_a_en,
        ctrl1_vf_b_en,
        ctrl1_vr_a_en|ctrl1_vr_b_en|ctrl1_vr_c_en|ctrl1_vr_d_we|ctrl1_vf_a_en|ctrl1_vf_b_en|ctrl1_vf_c_we,
        ctrl1_memunit_en,
        ctrl1_mem_en,
        |ctrl1_mulshift_op,
        ctrl1_matmul_en,
        ctrl1_bfadder_en,
        ctrl1_bfmult_en,
        ctrl1_act_en,
        ctrl1_trp_en,
        ctrl1_transpose_en,
        ctrl1_permute_en,
        ctrl1_permute_op,
        ctrl1_permute_store,
        ctrl1_permute_read,
        ctrl1_axi_en,
        ctrl1_axi_req_type,
        (ctrl1_alu_op!=(ALUOP_ZERO^ALUOP_ZERO)) || ctrl1_vf_c_we
      }),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[1] ),
      .squashn( ~pipe_squash[1] ),
      .q({
        ctrl2_vr_c_we,
        ctrl2_vf_c_we,
        ctrl2_vs_we,
        ctrl2_vr_a_en,
        ctrl2_vr_b_en,
        ctrl2_vf_a_en,
        ctrl2_vf_b_en,
        ctrl2_useslanes,
        ctrl2_memunit_en,
        ctrl2_mem_en,
        ctrl2_mulshift_en,
        ctrl2_matmul_en,
        ctrl2_bfadder_en,
        ctrl2_bfmult_en,
        ctrl2_act_en,
        ctrl2_trp_en,
        ctrl2_transpose_en,
        ctrl2_permute_en,
        ctrl2_permute_op,
        ctrl2_permute_store,
        ctrl2_permute_read,
        ctrl2_axi_en,
        ctrl2_axi_req_type,
        ctrl2_alufalu_en
        })
      );

  // *********** Pipeline signals that don't need to be squashed *********
  pipereg 
    #(
      REGIDWIDTH+
      REGIDWIDTH+
      REGIDWIDTH+
      1+
      1+
      1+
      1+
      1+
      1+
      11+
      2+
      4+
      5+
      7+
      1+
      3+
      1+
      1+
      1+
      2 // Sangram added 1 for TRP unit en signal
    ) pipe1reg (
      .d({
        {(ctrl1_vrdest_sel) ? ir_src2 : ir_dst, regid_pad},
        {(ctrl1_vr_c_en ) ? ir_dst : ir_src1, regid_pad},
        {ir_src2,regid_pad},
        ir_op[7] && ctrl1_usesvssel,
        ir_op[6] && ctrl1_usesvssel,
        ir_mask,
        ctrl1_vf_a_sel,
        ctrl1_rshiftnonzero,
        ctrl1_mem_dir_left,
        ctrl1_alu_op,
        ctrl1_satsum_op,
        ctrl1_satsize_op,
        ctrl1_mulshift_op,
        ctrl1_memunit_op,
        ctrl1_ismasked,
        ctrl1_flagalu_op,
        ctrl1_vf_wbsel,
        ctrl1_volatiledest,
        trp_mode_s1
      }),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[1] ),
      .squashn( 1'b1 ),
      .q({
        dst_s2,
        src1_s2,
        src2_s2,
        src1scalar_s2,
        src2scalar_s2,
        imask_s2,
        ctrl2_vf_a_sel,
        ctrl2_rshiftnonzero,
        ctrl2_mem_dir_left,
        ctrl2_alu_op,
        ctrl2_satsum_op,
        ctrl2_satsize_op,
        ctrl2_mulshift_op,
        ctrl2_memunit_op,
        ctrl2_ismasked,
        ctrl2_flagalu_op,
        ctrl2_vf_wbsel,
        ctrl2_volatiledest,
        trp_mode_s2
      }));
  
  wire [6:1] squash_ctrlmemoppipe_NC;
  pipe #(7,5) ctrlmemoppipe (
      .d( ctrl1_memunit_op ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:1] & {4'b1,ctrl2_memunit_en,1'b1} ),
      .squash(squash_ctrlmemoppipe_NC),
      //.squash( pipe_squash[6:1] ),
      .q( {ctrl_memunit_op[6],ctrl_memunit_op[5],ctrl_memunit_op[4],ctrl_memunit_op[3],ctrl_memunit_op[2],ctrl_memunit_op[1]} ));

/******************************************************************************/
/******************************* 2nd Pipeline Stage ***************************/
/******************************************************************************/

  // if src_start!=0 stall pipeline to calculate it and then do haz check

  onecyclestall shamtstall(ctrl2_srcshamt_sel!=0,clk,resetn,stall_srcstart);

  always@(posedge clk)
    if (!resetn || pipe_advance[1] )
      src_start_delayed<=0;
    else 
      src_start_delayed<=src_start;

  assign src_start= ( ctrl2_srcshamt_sel==3 ) ? vc[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))] :           // vcindex
               ( ctrl2_srcshamt_sel==1 ) ? vl[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-1) : ((2-2)*((VCWIDTH-1)-(0)+1))] :   // vl>>2
               ( ctrl2_srcshamt_sel==2 ) ? 1 << vc[((2-2)*((VCWIDTH-1)-(0)+1)+LOG2MVL-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]://2^vcindex
               0;

  assign src_elm=src_start_delayed & (-1<<LOG2NUMLANES);

  assign src_limit= ((ctrl2_setvlto1) ? 0 : vl[((2-2)*((VCWIDTH-1)-(0)+1)+LOG2MVL-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))] - 1'b1) +
                    ((ctrl2_srclimit_sel) ? vc[((2-2)*((VCWIDTH-1)-(0)+1)+LOG2MVL-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))] : 0);


  /******************* Adjust dest to account for shift  ****************/

  // Compute real destination register - accounting for shifting instructions

  assign dst_start= ( ctrl2_dstshamt_sel==3 ) ? vc[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))] :           // vcindex
               ( ctrl2_dstshamt_sel==2 ) ? 1 << vc[((2-2)*((VCWIDTH-1)-(0)+1)+LOG2MVL-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]://2^vcindex
               0;

  //assign dst_elm = {dst_start[LOG2MVL-1:0]>>LOG2NUMLANES,{LOG2NUMLANES{1'b0}}};

  assign total_shamt= ( ctrl2_elmshamt_sel==3 ) ? vc[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))] :          // vcindex
               ( ctrl2_elmshamt_sel==1 ) ? vl[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-1) : ((2-2)*((VCWIDTH-1)-(0)+1))] :    // vl>>2
               ( ctrl2_elmshamt_sel==2 ) ? 1 << vc[((2-2)*((VCWIDTH-1)-(0)+1)+LOG2MVL-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]://2^vcindex
               0;

  /************ Save vc_in values to not stall control pipeline ************/
  always@(posedge clk)
    if (!resetn)
      pipe_advance_s2_r<=0;
    else
      pipe_advance_s2_r<=pipe_advance[1];

  always@(posedge clk)
    if (!resetn)
    begin
      vc_in_saved<=0;
      vl_in_saved<=0;
      vbase_in_saved<=0;
      vinc_in_saved<=0;
      vstride_in_saved<=0;
      vs_in_saved<=0;
    end
    else if (pipe_advance_s2_r)
    begin
      vc_in_saved<=vc_in;
      vl_in_saved<=vl_in;
      vbase_in_saved<=vbase_in;
      vinc_in_saved<=vinc_in;
      vstride_in_saved<=vstride_in;
      vs_in_saved<=vs_in;
    end

  wire [6:2] squash_vcpipe_NC;
  pipe #(VCWIDTH,4) vcpipe (
      .d( (!pipe_advance_s2_r) ? vc_in_saved : vc_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vcpipe_NC),
      .q( {vc[((6-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((6-2)*((VCWIDTH-1)-(0)+1))],vc[((5-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((5-2)*((VCWIDTH-1)-(0)+1))],vc[((4-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((4-2)*((VCWIDTH-1)-(0)+1))],vc[((3-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((3-2)*((VCWIDTH-1)-(0)+1))],vc[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]} ));

  wire [6:2] squash_vlpipe_NC;
  pipe #(VCWIDTH,4) vlpipe (
      .d( (!pipe_advance_s2_r) ? vl_in_saved : vl_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vlpipe_NC),
      .q( {vl[((6-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((6-2)*((VCWIDTH-1)-(0)+1))],vl[((5-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((5-2)*((VCWIDTH-1)-(0)+1))],vl[((4-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((4-2)*((VCWIDTH-1)-(0)+1))],vl[((3-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((3-2)*((VCWIDTH-1)-(0)+1))],vl[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]} ));

  wire [6:2] squash_vbasepipe_NC;
  pipe #(VCWIDTH,4) vbasepipe (
      .d( (!pipe_advance_s2_r) ? vbase_in_saved : vbase_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,(ctrl2_memunit_en|ctrl2_axi_en)} ),
      .squash(squash_vbasepipe_NC),
      .q( {vbase[((6-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((6-2)*((VCWIDTH-1)-(0)+1))],vbase[((5-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((5-2)*((VCWIDTH-1)-(0)+1))],vbase[((4-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((4-2)*((VCWIDTH-1)-(0)+1))],vbase[((3-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((3-2)*((VCWIDTH-1)-(0)+1))],vbase[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]} ));

  wire [6:2] squash_vincpipe_NC;
  pipe #(VCWIDTH,4) vincpipe (
      .d( (!pipe_advance_s2_r) ? vinc_in_saved : vinc_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vincpipe_NC),
      .q( {vinc[((6-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((6-2)*((VCWIDTH-1)-(0)+1))],vinc[((5-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((5-2)*((VCWIDTH-1)-(0)+1))],vinc[((4-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((4-2)*((VCWIDTH-1)-(0)+1))],vinc[((3-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((3-2)*((VCWIDTH-1)-(0)+1))],vinc[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]} ));

  //stride register also used for elmt shamt for vext/vhalf/etc in memunit
  wire [6:2] squash_vstridepipe_NC;
  pipe #(VCWIDTH,4) vstridepipe (
      .d( (ctrl2_memunit_en&~ctrl2_mem_en) ? 
            ((LOG2NUMLANES>0) ?
                  total_shamt[((LOG2NUMLANES>0) ? LOG2NUMLANES : 1)-1:0] : 0) :
          (!pipe_advance_s2_r) ? vstride_in_saved : vstride_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vstridepipe_NC),
      .q( {vstride[((6-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((6-2)*((VCWIDTH-1)-(0)+1))],vstride[((5-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((5-2)*((VCWIDTH-1)-(0)+1))],vstride[((4-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((4-2)*((VCWIDTH-1)-(0)+1))],vstride[((3-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((3-2)*((VCWIDTH-1)-(0)+1))],vstride[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]} ));


  /*****************************    ISSUER    *****************************/

  assign stall_dispatcher=
    //Structural hazard
    (|(ctrl3_memunit_en&~last_subvector) && ctrl2_memunit_en) ||
    (|(ctrl3_mulshift_en&~last_subvector) && ctrl2_mulshift_en) ||
    (|(ctrl3_matmul_en&~last_subvector) && ctrl2_matmul_en) ||
    (|(ctrl3_bfadder_en&~last_subvector) && ctrl2_bfadder_en) ||
    (|(ctrl3_bfmult_en&~last_subvector) && ctrl2_bfmult_en) ||
    (|(ctrl3_act_en&~last_subvector) && ctrl2_act_en) ||
    (|(ctrl3_trp_en&~last_subvector) && ctrl2_trp_en) ||
    (|(ctrl3_transpose_en&~last_subvector) && ctrl2_transpose_en) ||
    (|(ctrl3_permute_en&~last_subvector) && ctrl2_permute_en) ||
    (|(ctrl3_axi_en&~last_subvector) && ctrl2_axi_en) ||
    (|(ctrl3_alufalu_en&~last_subvector) && ctrl2_alufalu_en && ALUPERBANK==0)||
    //Entry 0 is taken
    (dispatcher_rotate && ctrl2_useslanes);

  //assign stall_mulcooldown=|(ctrl3_mulshift_en&last_subvector);
  assign stall_mulcooldown=1'b0;

  assign dispatcher_shift=pipe_advance[3];
  // Rotate if last entry is alive (count not -1) and greater than zero
  assign dispatcher_rotate=(~count[NUMBANKS*(LOG2MVL-LOG2NUMLANES+1)-1]) && 
                          ((count>>((NUMBANKS-1)*(LOG2MVL-LOG2NUMLANES+1)))!=0);

`define DISPATCHWIDTH LOG2MVL+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    REGIDWIDTH+ \
    REGIDWIDTH+ \
    REGIDWIDTH+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    11+ \
    2+ \
    4+ \
    5+ \
    7+ \
    1+ \
    3+ \
    1+ \
    1+ \
    1+ \
    1+ \
    VCWIDTH+ \
    VSWIDTH+ \
    8+ \
    2+ \
    2+3+2 // This is fro Permute and Transpose unit enable + 3 This is for Permute op. store and read signals and +2 for axi signal 

wire [NUMBANKS*(`DISPATCHWIDTH)-1:0] dispatcher_instr;
  vdispatcher #(NUMBANKS, `DISPATCHWIDTH,LOG2MVL,LOG2MVL,LOG2MVL-LOG2NUMLANES+1)
    vdispatcher(
      .clk(clk),
      .resetn(resetn),
      .shift(dispatcher_shift),
      .rotate(dispatcher_rotate), //rotate and shift must be high to rotate
      .inshift_instr( {
          src_limit,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vr_c_we,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vf_c_we,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vs_we,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vr_a_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vr_b_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vf_a_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vf_b_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_useslanes,
          (pipe_squash[2]) ? 1'b0 : ctrl2_memunit_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_mem_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_mulshift_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_matmul_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_bfadder_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_bfmult_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_act_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_trp_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_transpose_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_permute_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_axi_en,
          ctrl2_axi_req_type,
          ctrl2_permute_op,
          ctrl2_permute_store,
          ctrl2_permute_read,
          (pipe_squash[2]) ? 1'b0 : ctrl2_alufalu_en,
          dst_s2,
          src1_s2,
          src2_s2,
          src1scalar_s2,
          src2scalar_s2,
          imask_s2,
          ctrl2_vf_a_sel,
          ctrl2_rshiftnonzero,
          ctrl2_mem_dir_left,
          ctrl2_alu_op,
          ctrl2_satsum_op,
          ctrl2_satsize_op,
          ctrl2_mulshift_op,
          ctrl2_memunit_op,
          ctrl2_ismasked,
          ctrl2_flagalu_op,
          ctrl2_vf_wbsel,
          ctrl2_volatiledest,
          (!pipe_advance_s2_r) ? vc_in_saved : vc_in,
          (!pipe_advance_s2_r) ? vs_in_saved : vs_in,
          (pipe_squash[2]) ? 8'b0 : D_instr[((2-1)*((7)-(0)+1)+7-0) : ((2-1)*((7)-(0)+1))],
          trp_mode_s2
          }),
      .inshift_first(ctrl2_useslanes),
      .inshift_rdelm(src_start),
      .inshift_wrelm(dst_start),
      .inshift_count((pipe_squash[2] || !ctrl2_useslanes) ? -1 : (src_limit[LOG2MVL-1:0]>>LOG2NUMLANES) - (src_elm[LOG2MVL-1:0]>>LOG2NUMLANES) ),
      .increment((~wrongbank_s3)&{NUMBANKS{pipe_advance[3]}}),
      .rdelm_add_sub(0),
      .wrelm_add_sub(0),
      .count_add_sub(1),
      .rdelm_valuetoadd(NUMLANES),
      .wrelm_valuetoadd(NUMLANES),
      .count_valuetoadd(1),
      .instr(dispatcher_instr),
      .first(first_subvector),
      .rdelm(rdelm),
      .wrelm(wrelm),
      .count(count)
    );

  

  /******************* Stall on RAW HAZARDS ************************/

  always@*
    for (bd=0; bd<1+(NUMBANKS-1)*ALUPERBANK; bd=bd+1)
    begin
      alu_dst[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1)) +: ( REGIDWIDTH)]=dst[FU_ALU+bd][4];
      alu_dst_we[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1)+bd-bd) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1))]=dst_we[FU_ALU+bd][4];
      falu_dst[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1)) +: ( REGIDWIDTH)]=dst[FU_FALU+bd][4];
      falu_dst_we[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1)+bd-bd) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1))]=dst_we[FU_FALU+bd][4];
    end


  hazardchecker #(REGIDWIDTH,VELMIDWIDTH,4+(NUMBANKS-1)*ALUPERBANK,NUMBANKS) 
    src1hazchecker(
      .src( src1_s2 |(src_start_delayed[LOG2MVL-1:0]>>LOG2NUMLANES) ),
      .src_valid(ctrl2_vr_a_en),
      .dst({
        alu_dst[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1))],
        dst[FU_MUL][4],
        dst[FU_MEM][4],
        dst[FU_MATMUL][4]
        }),
      .dst_valid({
        alu_dst_we[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1))],
        dst_we[((FU_MUL-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_MUL-0)*((`MAX_STAGES-1)-(4)+1))],
        dst_we[((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1))],
        dst_we[((FU_MATMUL-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_MATMUL-0)*((`MAX_STAGES-1)-(4)+1))]
        }),
      .dst_mode({{3+(NUMBANKS-1)*ALUPERBANK{1'b0}},ctrl4_volatiledest}),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vr_c_we),
      .lt_mode(ctrl3_volatiledest),
      .haz(stall_hazsrc1));

  hazardchecker #(REGIDWIDTH,VELMIDWIDTH,4+(NUMBANKS-1)*ALUPERBANK,NUMBANKS) 
    src2hazchecker(
      .src( src2_s2 |(src_start_delayed[LOG2MVL-1:0]>>LOG2NUMLANES) ),
      .src_valid(ctrl2_vr_b_en),
      .dst({
        alu_dst[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1))],
        dst[FU_MUL][4],
        dst[FU_MEM][4],
        dst[FU_MATMUL][4]
        }),
      .dst_valid({
        alu_dst_we[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1))],
        dst_we[((FU_MUL-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_MUL-0)*((`MAX_STAGES-1)-(4)+1))],
        dst_we[((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1))],
        dst_we[((FU_MATMUL-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_MATMUL-0)*((`MAX_STAGES-1)-(4)+1))]
        }),
      .dst_mode({{3+(NUMBANKS-1)*ALUPERBANK{1'b0}},ctrl4_volatiledest}),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vr_c_we),
      .lt_mode(ctrl3_volatiledest),
      .haz(stall_hazsrc2));

  //Check flag hazards - flags always start at 0th element
  hazardchecker #(REGIDWIDTH,VELMIDWIDTH,1+(NUMBANKS-1)*ALUPERBANK,NUMBANKS) 
    fsrc1hazchecker(
      // Check for mask flag and src1 flag depending on instruction
      .src( (ctrl2_vf_a_sel) ? src1_s2 : (imask_s2<<VELMIDWIDTH)),
      .src_valid(ctrl2_vf_a_en),
      .dst(falu_dst[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1))]),
      .dst_valid(falu_dst_we[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1))]),
      .dst_mode(0),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vf_c_we),
      .lt_mode(0),
      .haz(stall_hazfsrc1));

  hazardchecker #(REGIDWIDTH,VELMIDWIDTH,1+(NUMBANKS-1)*ALUPERBANK,NUMBANKS) 
    fsrc2hazchecker(
      .src( src2_s2 ),
      .src_valid(ctrl2_vf_b_en),
      .dst(falu_dst[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1))]),
      .dst_valid(falu_dst_we[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1))]),
      .dst_mode(0),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vf_c_we),
      .lt_mode(0),
      .haz(stall_hazfsrc2));


/******************************************************************************/
/************************** 3rd Pipeline Stage ********************************/
/******************************************************************************/

  always@*
    for (bi=0; bi<NUMBANKS; bi=bi+1)
    begin
      {
        src_limit_s3[((bi-0)*((LOG2MVL-1)-(0)+1)+LOG2MVL-1-0) : ((bi-0)*((LOG2MVL-1)-(0)+1))],
        ctrl3_vr_c_we[bi],
        ctrl3_vf_c_we[bi],
        ctrl3_vs_we[bi],
        ctrl3_vr_a_en[bi],
        ctrl3_vr_b_en[bi],
        ctrl3_vf_a_en[bi],
        ctrl3_vf_b_en[bi],
        ctrl3_useslanes[bi],
        ctrl3_memunit_en[bi],
        ctrl3_mem_en[bi],
        ctrl3_mulshift_en[bi],
        ctrl3_matmul_en[bi],
        ctrl3_bfadder_en[bi],
        ctrl3_bfmult_en[bi],
        ctrl3_act_en[bi],
        ctrl3_trp_en[bi],
        ctrl3_transpose_en[bi],
        ctrl3_permute_en[bi],
        ctrl3_axi_en[bi],
        ctrl3_axi_req_type[bi],
        ctrl3_permute_op[bi],
        ctrl3_permute_store[bi],
        ctrl3_permute_read[bi],
        ctrl3_alufalu_en[bi],
        _dst_s3[((bi-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bi-0)*((REGIDWIDTH-1)-(0)+1))],
        _src1_s3[((bi-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bi-0)*((REGIDWIDTH-1)-(0)+1))],
        _src2_s3[((bi-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bi-0)*((REGIDWIDTH-1)-(0)+1))],
        src1scalar_s3[bi],
        src2scalar_s3[bi],
        imask_s3[bi],
        ctrl3_vf_a_sel[bi],
        ctrl3_rshiftnonzero[bi],
        ctrl3_mem_dir_left[bi],
        ctrl3_alu_op[((bi-0)*((10)-(0)+1)+10-0) : ((bi-0)*((10)-(0)+1))],
        ctrl3_satsum_op[((bi-0)*((1)-(0)+1)+1-0) : ((bi-0)*((1)-(0)+1))],
        ctrl3_satsize_op[((bi-0)*((3)-(0)+1)+3-0) : ((bi-0)*((3)-(0)+1))],
        ctrl3_mulshift_op[((bi-0)*((4)-(0)+1)+4-0) : ((bi-0)*((4)-(0)+1))],
        ctrl3_memunit_op[((bi-0)*((6)-(0)+1)+6-0) : ((bi-0)*((6)-(0)+1))],
        ctrl3_ismasked[bi],
        ctrl3_flagalu_op[((bi-0)*((2)-(0)+1)+2-0) : ((bi-0)*((2)-(0)+1))],
        ctrl3_vf_wbsel[bi],
        ctrl3_volatiledest[bi],
        vc_s3[((bi-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((bi-0)*((VCWIDTH-1)-(0)+1))],
        vs_s3[((bi-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((bi-0)*((VSWIDTH-1)-(0)+1))],
        D_instr_s3[((bi-0)*((7)-(0)+1)+7-0) : ((bi-0)*((7)-(0)+1))],
        trp_mode_s3[((bi-0)*((1)-(0)+1)+1-0) : ((bi-0)*((1)-(0)+1))]
      } = dispatcher_instr>>(bi*(`DISPATCHWIDTH));

      last_subvector[bi]=~|count[bi*(LOG2MVL-LOG2NUMLANES+1)+:LOG2MVL-LOG2NUMLANES];
      alive_s3[bi]=~count[(bi+1)*(LOG2MVL-LOG2NUMLANES+1)-1];

      for (i=0; i<NUMLANES; i=i+1)
        lane_en[((bi-0)*((NUMLANES-1)-(0)+1)+i-i) : ((bi-0)*((NUMLANES-1)-(0)+1))]= (LOG2NUMLANES==0) || //Support 1 lane
          ~((first_subvector[bi]) && 
              i<`LO(rdelm[bi*LOG2MVL +: LOG2MVL],LOG2NUMLANES) ||
            (last_subvector[bi]) && 
              i>src_limit_s3[((bi-0)*((LOG2MVL-1)-(0)+1)+LOG2MVL-1-0) : ((bi-0)*((LOG2MVL-1)-(0)+1))][((LOG2NUMLANES>0) ? LOG2NUMLANES : 1)-1:0] );
    end


  // ************* Map from issuer to register file banks *************
  always@*
    for (b=0; b<NUMBANKS; b=b+1)
    begin 
      dst_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))]=_dst_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))] | (wrelm[b*LOG2MVL+:LOG2MVL]>>LOG2NUMLANES);
      t_dst_s3[b*REGIDWIDTH +: REGIDWIDTH]=dst_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))];
      src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))]=_src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))] | (rdelm[b*LOG2MVL+:LOG2MVL]>>LOG2NUMLANES);
      src2_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))]=_src2_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))] | (rdelm[b*LOG2MVL+:LOG2MVL]>>LOG2NUMLANES);

      wrongbank_s3[b]=(`LO(rdelm[b*LOG2MVL+:LOG2MVL]>>LOG2NUMLANES,LOG2NUMBANKS)!=b);

      vr_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
      vr_b_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src2_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
      vr_a_en[b]=ctrl3_vr_a_en[b] && pipe_advance[3];
      vr_b_en[b]=ctrl3_vr_b_en[b] && pipe_advance[3];

      vf_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]= 
        {(ctrl3_vf_a_sel[b]) ? _src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))][`VRID_RANGE] : imask_s3[b],
        src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))][`VRELM_RANGE]}>>LOG2NUMBANKS;
      vf_b_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src2_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
      vf_a_en[b]=ctrl3_vf_a_en[b] && pipe_advance[3];
      vf_b_en[b]=ctrl3_vf_b_en[b] && pipe_advance[3];
    end

  // ************* Map from issuer/banks to Functional Units *************
  always@(posedge clk)
    if (!resetn)
    begin
      dst_we_s4=0;
      ctrl4_mulshift_en=0;
      ctrl4_matmul_en=0;
      ctrl4_bfadder_en=0;
      ctrl4_bfmult_en=0;
      ctrl4_act_en=0;
      ctrl4_trp_en=0;
      ctrl4_transpose_en=0;
      ctrl4_permute_en=0;
      ctrl4_permute_op=0;
      ctrl4_permute_store=0;
      ctrl4_permute_read=0;
      ctrl4_axi_en=0;
      ctrl4_axi_req_type=0;
      ctrl4_memunit_en=0;
      ctrl4_mem_en=0;
      ctrl4_volatiledest=0;
    end
    else if (pipe_advance[3])
    begin
      dst_we_s4=0;
      ctrl4_mulshift_en=0;
      ctrl4_matmul_en=0;
      ctrl4_bfadder_en=0;
      ctrl4_bfmult_en=0;
      ctrl4_act_en=0;
      ctrl4_trp_en=0;
      ctrl4_transpose_en=0;
      ctrl4_permute_en=0;
      ctrl4_permute_op=0;
      ctrl4_permute_store=0;
      ctrl4_permute_read=0;
      ctrl4_axi_en=0;
      ctrl4_axi_req_type=0;
      ctrl4_memunit_en=0;
      ctrl4_mem_en=|ctrl3_mem_en;
      ctrl4_volatiledest=0;

      for (f3=0; f3<NUMFUS; f3=f3+1)
      begin
        D_instr_s4[((f3-0)*((7)-(0)+1)+7-0) : ((f3-0)*((7)-(0)+1))]=0;
        D_last_subvector_s4[f3]=0;
        dst_s4[((f3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((f3-0)*((REGIDWIDTH-1)-(0)+1))]=0;
      end

      for (b3=0; b3<NUMBANKS; b3=b3+1)
      begin
        //if instruction is alive && is in correct bank
        if ( alive_s3[b3]  &&     //alive
            ~wrongbank_s3[b3] &&  //correct bank
            ~pipe_squash[3])
          if (ctrl3_mulshift_en[b3])    //is multiply
          begin
            D_instr_s4[((FU_MUL-0)*((7)-(0)+1)+7-0) : ((FU_MUL-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_MUL]=last_subvector[b3];
            ctrl4_mulshift_en=ctrl3_mulshift_en[b3];
            banksel_s4[((FU_MUL-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MUL-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_MUL-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_MUL-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_MUL-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_MUL-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_MUL]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_MUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MUL-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_MUL]=src1scalar_s3[b3];
            src2scalar_s4[FU_MUL]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MUL]=ctrl3_ismasked[b3];
            ctrl4_mulshift_op=ctrl3_mulshift_op[((b3-0)*((4)-(0)+1)+4-0) : ((b3-0)*((4)-(0)+1))];
            ctrl4_rshiftnonzero=ctrl3_rshiftnonzero[b3];
          end
          else if (ctrl3_matmul_en[b3])    //is matmul
          begin
            D_instr_s4[((FU_MATMUL-0)*((7)-(0)+1)+7-0) : ((FU_MATMUL-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_MATMUL]=last_subvector[b3];
            ctrl4_matmul_en=ctrl3_matmul_en[b3];
            banksel_s4[((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_MATMUL-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_MATMUL-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_MATMUL-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_MATMUL-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_MATMUL]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_MATMUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MATMUL-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_MATMUL]=src1scalar_s3[b3];
            src2scalar_s4[FU_MATMUL]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MATMUL]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_bfadder_en[b3])    //is bfloat addition
          begin
            D_instr_s4[((FU_BFADDER-0)*((7)-(0)+1)+7-0) : ((FU_BFADDER-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_BFADDER]=last_subvector[b3];
            ctrl4_bfadder_en=ctrl3_bfadder_en[b3];
            banksel_s4[((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_BFADDER-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_BFADDER-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_BFADDER-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_BFADDER-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_BFADDER]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_BFADDER-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFADDER-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_BFADDER]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFADDER]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFADDER]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_bfmult_en[b3])    //is bfloat addition
          begin
            D_instr_s4[((FU_BFMULT-0)*((7)-(0)+1)+7-0) : ((FU_BFMULT-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_BFMULT]=last_subvector[b3];
            ctrl4_bfmult_en = ctrl3_bfmult_en[b3];
            banksel_s4[((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_BFMULT-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_BFMULT-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_BFMULT-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_BFMULT-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_BFMULT]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_BFMULT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFMULT-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_BFMULT]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFMULT]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFMULT]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_transpose_en[b3])    //is bfloat addition
          begin
            D_instr_s4[((FU_TRANSPOSE-0)*((7)-(0)+1)+7-0) : ((FU_TRANSPOSE-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_TRANSPOSE]=last_subvector[b3];
            ctrl4_transpose_en = ctrl3_transpose_en[b3];
            banksel_s4[((FU_TRANSPOSE-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_TRANSPOSE-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_TRANSPOSE-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_TRANSPOSE-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_TRANSPOSE-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_TRANSPOSE-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_TRANSPOSE]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_TRANSPOSE]=src1scalar_s3[b3];
            src2scalar_s4[FU_TRANSPOSE]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_TRANSPOSE]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_permute_en[b3])    //is bfloat addition
          begin
            D_instr_s4[((FU_PERMUTE-0)*((7)-(0)+1)+7-0) : ((FU_PERMUTE-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_PERMUTE]=last_subvector[b3];
            ctrl4_permute_en = ctrl3_permute_en[b3];
            ctrl4_permute_op = ctrl3_permute_op;
            ctrl4_permute_store = ctrl3_permute_store;
            ctrl4_permute_read = ctrl3_permute_read;
            banksel_s4[((FU_PERMUTE-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_PERMUTE-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_PERMUTE-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_PERMUTE-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_PERMUTE-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_PERMUTE-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_PERMUTE]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_PERMUTE]=src1scalar_s3[b3];
            src2scalar_s4[FU_PERMUTE]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_PERMUTE]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_trp_en[b3])    //is bfloat addition
          begin
            D_instr_s4[((FU_TRP-0)*((7)-(0)+1)+7-0) : ((FU_TRP-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_TRP]=last_subvector[b3];
            ctrl4_trp_en = ctrl3_trp_en[b3];
            trp_mode_s4 = trp_mode_s3[((b3-0)*((1)-(0)+1)+1-0) : ((b3-0)*((1)-(0)+1))];
            banksel_s4[((FU_TRP-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_TRP-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_TRP-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_TRP-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_TRP-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_TRP-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_TRP]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_TRP-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRP-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_TRP]=src1scalar_s3[b3];
            src2scalar_s4[FU_TRP]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_TRP]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_act_en[b3])    //is bfloat addition
          begin
            D_instr_s4[((FU_ACT-0)*((7)-(0)+1)+7-0) : ((FU_ACT-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_ACT]=last_subvector[b3];
            ctrl4_act_en = ctrl3_act_en[b3];
            banksel_s4[((FU_ACT-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_ACT-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_ACT-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_ACT-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_ACT-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_ACT-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_ACT]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_ACT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_ACT-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_ACT]=src1scalar_s3[b3];
            src2scalar_s4[FU_ACT]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_ACT]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_axi_en[b3])    //is axi operation
          begin
            D_instr_s4[((FU_AXI-0)*((7)-(0)+1)+7-0) : ((FU_AXI-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_AXI]=last_subvector[b3];
            ctrl4_axi_en = ctrl3_axi_en[b3];
            ctrl4_axi_req_type = ctrl3_axi_req_type[b3];
            banksel_s4[((FU_AXI-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_AXI-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_AXI-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_AXI-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_AXI-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_AXI-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_AXI]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_AXI-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_AXI-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_AXI]=src1scalar_s3[b3];
            src2scalar_s4[FU_AXI]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_AXI]=ctrl3_ismasked[b3];
            vbase_s4=(first_subvector&ctrl3_axi_en)? vbase[((3-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((3-2)*((VCWIDTH-1)-(0)+1))] : vbase_s4 + (2<<LOG2NUMLANES);
          end
          else if (ctrl3_memunit_en[b3] && !ctrl3_mem_en[b3]) //is memunit shift
          begin
            D_instr_s4[((FU_MEM-0)*((7)-(0)+1)+7-0) : ((FU_MEM-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_MEM]=last_subvector[b3];
            ctrl4_memunit_en=ctrl3_memunit_en[b3];
            banksel_s4[((FU_MEM-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MEM-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_MEM-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_MEM-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            dst_s4[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_MEM]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_MEM-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MEM-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_MEM]=src1scalar_s3[b3];
            src2scalar_s4[FU_MEM]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MEM]=ctrl3_ismasked[b3];
            ctrl4_volatiledest=ctrl3_volatiledest[b3];
            ctrl4_mem_dir_left=ctrl3_mem_dir_left[b3];
            ctrl4_memunit_op=ctrl3_memunit_op[((b3-0)*((6)-(0)+1)+6-0) : ((b3-0)*((6)-(0)+1))];
            mem_last_subvector_s4=last_subvector[b3];
          end
          else if (ctrl3_memunit_en[b3] &&  ctrl3_mem_en[b3]) //is mem operation
          begin
            D_instr_s4[((FU_MEM-0)*((7)-(0)+1)+7-0) : ((FU_MEM-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_MEM]=last_subvector[b3];
            ctrl4_memunit_en=ctrl3_memunit_en[b3];
            banksel_s4[((FU_MEM-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MEM-0)*((NUMBANKS-1)-(0)+1))]=b3;
            // Use vs to store current length for prefetching
            vs_s4[((FU_MEM-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_MEM-0)*((VSWIDTH-1)-(0)+1))]=count[b3*(LOG2MVL-LOG2NUMLANES+1)+:(LOG2MVL-LOG2NUMLANES)]<<LOG2NUMLANES;
            dst_s4[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_MEM]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_MEM-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MEM-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_MEM]=src1scalar_s3[b3];
            src2scalar_s4[FU_MEM]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MEM]=ctrl3_ismasked[b3];
            ctrl4_volatiledest=ctrl3_volatiledest[b3];
            ctrl4_mem_dir_left=ctrl3_mem_dir_left[b3];
            ctrl4_memunit_op=ctrl3_memunit_op[((b3-0)*((6)-(0)+1)+6-0) : ((b3-0)*((6)-(0)+1))];
            //Load base on first subvector or if INDEXED memory operation
            vbase_s4=(|(first_subvector&ctrl3_mem_en) || ctrl_memunit_op[3][5])?
              vbase[3] : vbase_s4 + ((((ctrl_memunit_op[3][4])? vstride[3]: 2)
                      <<ctrl_memunit_op[3][3:2])<<LOG2NUMLANES);
            // Partial Address Gen for each lane - just do multiplication part
            for (m=0; m<NUMLANES; m=m+1)
              vstrideoffset_s4[m*VCWIDTH +: VCWIDTH] = 
                                ((ctrl_memunit_op[3][4]) ? vstride[3] : 1)*m;
            mem_last_subvector_s4=last_subvector[b3];
          end
          else if (ctrl3_alu_op[((b3-0)*((10)-(0)+1)+10-0) : ((b3-0)*((10)-(0)+1))]!=(ALUOP_ZERO^ALUOP_ZERO)) //is ALU
          begin
            D_instr_s4[FU_ALU+b3*ALUPERBANK]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_ALU+b3*ALUPERBANK]=last_subvector[b3];
            banksel_s4[FU_ALU+b3*ALUPERBANK]=b3;
            vs_s4[FU_ALU+b3*ALUPERBANK]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            dst_we_s4[FU_ALU+b3*ALUPERBANK]=ctrl3_vr_c_we[b3];
            dst_s4[FU_ALU+b3*ALUPERBANK]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_FALU+b3*ALUPERBANK]=ctrl3_vf_c_we[b3];
            dst_s4[FU_FALU+b3*ALUPERBANK]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            vlane_en[FU_ALU+b3*ALUPERBANK]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_ALU+b3*ALUPERBANK]=src1scalar_s3[b3];
            src2scalar_s4[FU_ALU+b3*ALUPERBANK]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_ALU+b3*ALUPERBANK]=ctrl3_ismasked[b3];
            ctrl4_alu_op[((b3*ALUPERBANK-0)*((10)-(0)+1)+10-0) : ((b3*ALUPERBANK-0)*((10)-(0)+1))]=ctrl3_alu_op[((b3-0)*((10)-(0)+1)+10-0) : ((b3-0)*((10)-(0)+1))];
            ctrl4_satsum_op[((b3*ALUPERBANK-0)*((1)-(0)+1)+1-0) : ((b3*ALUPERBANK-0)*((1)-(0)+1))]=ctrl3_satsum_op[((b3-0)*((1)-(0)+1)+1-0) : ((b3-0)*((1)-(0)+1))];
            ctrl4_satsize_op[((b3*ALUPERBANK-0)*((3)-(0)+1)+3-0) : ((b3*ALUPERBANK-0)*((3)-(0)+1))]=ctrl3_satsize_op[((b3-0)*((3)-(0)+1)+3-0) : ((b3-0)*((3)-(0)+1))];
            ctrl4_vf_wbsel[b3*ALUPERBANK]=ctrl3_vf_wbsel[b3];
          end
          else if (ctrl3_vf_c_we[b3])
          begin                                    //is FALU
            D_instr_s4[FU_FALU+b3*ALUPERBANK]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_FALU+b3*ALUPERBANK]=last_subvector[b3];
            banksel_s4[FU_FALU+b3*ALUPERBANK]=b3;
            vs_s4[FU_FALU+b3*ALUPERBANK]=|vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            dst_we_s4[FU_FALU+b3*ALUPERBANK]=ctrl3_vf_c_we[b3];
            dst_s4[FU_FALU+b3*ALUPERBANK]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            vlane_en[FU_FALU+b3*ALUPERBANK]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_FALU+b3*ALUPERBANK]=src1scalar_s3[b3];
            src2scalar_s4[FU_FALU+b3*ALUPERBANK]=src2scalar_s3[b3];
            ctrl4_flagalu_op[((b3*ALUPERBANK-0)*((2)-(0)+1)+2-0) : ((b3*ALUPERBANK-0)*((2)-(0)+1))]=ctrl3_flagalu_op[((b3-0)*((2)-(0)+1)+2-0) : ((b3-0)*((2)-(0)+1))];
            ctrl4_vf_wbsel[b3*ALUPERBANK]=ctrl3_vf_wbsel[b3];
            ctrl4_vf_a_sel[b3*ALUPERBANK]=ctrl3_vf_a_sel[b3];
          end
        end
      end


  vregfile_vector 
    #(NUMBANKS,LOG2NUMBANKS,8*VPW*NUMLANES,32*MVL/NUMLANES,REGIDWIDTH) 
    vregfile_vector (
      .clk(clk),
      .resetn(resetn),
      .a_reg(vr_a_reg),
      .a_readdataout(_vr_a_readdataout),
      .a_en(vr_a_en),
      .b_reg(vr_b_reg),
      .b_readdataout(_vr_b_readdataout),
      .b_en(vr_b_en),
      .c_reg(_vr_c_reg),
      .c_writedatain(_vr_c_writedatain),
      .c_byteen(vr_c_byteen),
      .c_we(vr_c_we));

  vregfile_flag 
    #(NUMBANKS,LOG2NUMBANKS,NUMLANES,32*MVL/NUMLANES,REGIDWIDTH) 
    vregfile_flag (
      .clk(clk),
      .resetn(resetn),
      .a_reg(vf_a_reg),
      .a_readdataout(vf_a_readdataout),
      .a_en(vf_a_en),
      .b_reg(vf_b_reg),
      .b_readdataout(vf_b_readdataout),
      .b_en(vf_b_en),
      .c_reg(vf_c_reg),
      .c_writedatain(vf_c_writedatain),
      .c_we(vf_c_we));


  pipereg #(1) sdstwepipereg (
      .d( |ctrl3_vs_we ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[3] ),
      .squashn( ~pipe_squash[3] ),
      .q(ctrl_vs_we[4]));

/******************************************************************************/
/************************** 4th Pipeline Stage ********************************/
/******************************************************************************/

  //Convert register file width (8*VPW) to datpath width (LANEWIDTH)
  always@*
    for (bn=0; bn<NUMBANKS; bn=bn+1)
      for (n=0; n<NUMLANES; n=n+1)
      begin
        vr_a_readdataout[((bn-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)) +: ( LANEWIDTH)] = 
          _vr_a_readdataout[bn*8*VPW*NUMLANES + 8*VPW*n +: 8*VPW];
        vr_b_readdataout[((bn-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)) +: ( LANEWIDTH)] = 
          _vr_b_readdataout[bn*8*VPW*NUMLANES + 8*VPW*n +: 8*VPW];
        _vr_c_writedatain[bn*8*VPW*NUMLANES + 8*VPW*n +: 8*VPW] = 
          vr_c_writedatain[((bn-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)) +: ( LANEWIDTH)];
      end

  //Bank Multiplex for each functional unit
  always@*
  begin
    for (fn=0; fn<=FU_MUL; fn=fn+1)
    begin
      vr_src1[fn] =(src1scalar_s4[fn]) ? {NUMLANES{vs_s4[((fn-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((fn-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_a_readdataout[banksel_s4[((fn-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((fn-0)*((NUMBANKS-1)-(0)+1))]];
      vr_src2[fn] =(src2scalar_s4[fn]) ? {NUMLANES{vs_s4[((fn-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((fn-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_b_readdataout[banksel_s4[((fn-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((fn-0)*((NUMBANKS-1)-(0)+1))]];
      vf_src1[((fn-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((fn-0)*((NUMLANES-1)-(0)+1))] = vf_a_readdataout[banksel_s4[((fn-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((fn-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES];

      vmask[((fn-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((fn-0)*((NUMLANES-1)-(0)+1))] =vlane_en[((fn-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((fn-0)*((NUMLANES-1)-(0)+1))] &
            ((ctrl4_ismasked[fn]) ?
              vf_a_readdataout[banksel_s4[((fn-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((fn-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
              {NUMLANES{1'b1}}) ;
    end

    vr_src1[((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))] = vr_a_readdataout[banksel_s4[((FU_MEM-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MEM-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src2[((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))] = vr_b_readdataout[banksel_s4[((FU_MEM-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MEM-0)*((NUMBANKS-1)-(0)+1))]];
    vmask[((FU_MEM-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MEM-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_MEM-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MEM-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_MEM]) ?  
             vf_a_readdataout[banksel_s4[((FU_MEM-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MEM-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vr_src1[FU_MATMUL] =(src1scalar_s4[FU_MATMUL]) ? {NUMLANES{vs_s4[((FU_MATMUL-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_MATMUL-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_a_readdataout[banksel_s4[((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src2[FU_MATMUL] =(src2scalar_s4[FU_MATMUL]) ? {NUMLANES{vs_s4[((FU_MATMUL-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_MATMUL-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_b_readdataout[banksel_s4[((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src1[FU_BFADDER] =(src1scalar_s4[FU_BFADDER]) ? {NUMLANES{vs_s4[((FU_BFADDER-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_BFADDER-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_a_readdataout[banksel_s4[((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src2[FU_BFADDER] =(src2scalar_s4[FU_BFADDER]) ? {NUMLANES{vs_s4[((FU_BFADDER-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_BFADDER-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_b_readdataout[banksel_s4[((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src1[FU_BFMULT] =(src1scalar_s4[FU_BFMULT]) ? {NUMLANES{vs_s4[((FU_BFMULT-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_BFMULT-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_a_readdataout[banksel_s4[((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src2[FU_BFMULT] =(src2scalar_s4[FU_BFMULT]) ? {NUMLANES{vs_s4[((FU_BFMULT-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_BFMULT-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_b_readdataout[banksel_s4[((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src1[FU_TRP] =(src1scalar_s4[FU_TRP]) ? {NUMLANES{vs_s4[((FU_TRP-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_TRP-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_a_readdataout[banksel_s4[((FU_TRP-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_TRP-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src1[FU_TRANSPOSE] =(src1scalar_s4[FU_TRANSPOSE]) ? {NUMLANES{vs_s4[((FU_TRANSPOSE-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_TRANSPOSE-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_a_readdataout[banksel_s4[((FU_TRANSPOSE-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_TRANSPOSE-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src1[FU_PERMUTE] =(src1scalar_s4[FU_PERMUTE]) ? {NUMLANES{vs_s4[((FU_PERMUTE-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_PERMUTE-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_b_readdataout[banksel_s4[((FU_PERMUTE-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_PERMUTE-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src1[FU_AXI] =(src1scalar_s4[FU_AXI]) ? {NUMLANES{vs_s4[((FU_AXI-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_AXI-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_b_readdataout[banksel_s4[((FU_AXI-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_AXI-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src1[FU_ACT] =(src1scalar_s4[FU_ACT]) ? {NUMLANES{vs_s4[((FU_ACT-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_ACT-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_a_readdataout[banksel_s4[((FU_ACT-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_ACT-0)*((NUMBANKS-1)-(0)+1))]];
    vmask[((FU_MATMUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MATMUL-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_MATMUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MATMUL-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_MATMUL]) ?  
             vf_a_readdataout[banksel_s4[((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vmask[((FU_ACT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_ACT-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_ACT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_ACT-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_ACT]) ?  
             vf_a_readdataout[banksel_s4[((FU_ACT-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_ACT-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vmask[((FU_BFADDER-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFADDER-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_BFADDER-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFADDER-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_BFADDER]) ?  
             vf_a_readdataout[banksel_s4[((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vmask[((FU_BFMULT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFMULT-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_BFMULT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFMULT-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_BFMULT]) ?  
             vf_a_readdataout[banksel_s4[((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vmask[((FU_TRP-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRP-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_TRP-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRP-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_TRP]) ?  
             vf_a_readdataout[banksel_s4[((FU_TRP-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_TRP-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vmask[((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_TRANSPOSE]) ?  
             vf_a_readdataout[banksel_s4[((FU_TRANSPOSE-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_TRANSPOSE-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vmask[((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_PERMUTE]) ?  
             vf_a_readdataout[banksel_s4[((FU_PERMUTE-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_PERMUTE-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;
    vmask[((FU_AXI-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_AXI-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_AXI-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_AXI-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_AXI]) ?  
             vf_a_readdataout[banksel_s4[((FU_AXI-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_AXI-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    for (fn2=FU_FALU; fn2<=FU_FALU+(NUMBANKS-1)*ALUPERBANK; fn2=fn2+1)
    begin
      vf_src1[fn2] =(src1scalar_s4[fn2]&ctrl4_vf_a_sel[fn2-FU_FALU]) ? {NUMLANES{vs_s4[((fn2-0)*((VSWIDTH-1)-(0)+1)+0-0) : ((fn2-0)*((VSWIDTH-1)-(0)+1))]}} :
        vf_a_readdataout[banksel_s4[((fn2-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((fn2-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES];

      //Only FALU uses this
      vf_src2[fn2] = (src2scalar_s4[fn2]) ? {NUMLANES{vs_s4[((fn2-0)*((VSWIDTH-1)-(0)+1)+0-0) : ((fn2-0)*((VSWIDTH-1)-(0)+1))]}} : 
        vf_b_readdataout[banksel_s4[((fn2-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((fn2-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES];
    end

  end

/******************************************************************************/
/************************** 5th Pipeline Stage ********************************/
/******************************************************************************/

  // *********************** FUNCTIONAL UNITS *********************
  //
  // Note that functional units (recently) carry the register they're
  // writing to and a write signal so that once the result is computed
  // it can issue the full register write request.  As a result, it 
  // needs to know the status of the pipeline (pipe_advance and pipe_squash)

  // If mem_unit is stalled, pipeline can still go on if both
  //    i) it's a store hence requiring no writeback bank, AND
  //    ii) another memory instruction isn't waiting on the memunit
  //assign stall_memunit=_stall_memunit && (dst_we[FU_MEM][5] || (ctrl4_memunit_en && ~stall_mulunit && ~stall_matmul));
  assign stall_memunit= 1'b0;

//  pipereg #(1) memen5pipereg (
//      .d( ctrl4_mem_en ),
//      .clk(clk),
//      .resetn(resetn),
//      .en( pipe_advance[4] && ~_stall_memunit ),
//      .squashn( ~pipe_squash[4] | _stall_memunit ),
//      .q(ctrl5_mem_en ));

  pipereg #(1) memen5pipereg (
      .d( ctrl4_mem_en ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4]),
      .squashn( ~pipe_squash[4]),
      .q( ctrl5_mem_en ));

// AXI interface logic
//

wire [REGIDWIDTH-1:0] axi_dst;
wire [NUMLANES-1:0] axi_mask;
wire axi_dst_we;

axi4_master #(
    .P_TARGET_SLAVE_BASE_ADDR(0),                           
    .P_ADDR_WIDTH(11),                                      
    .P_DATA_WIDTH(128)                                      
)inst_axi_master
(
    .req_addr      (vbase_s4[10:0]),
    .req_data      (vr_src1[((FU_AXI-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_AXI-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
    .req_en        (ctrl4_axi_en),
    .req_type      (ctrl4_axi_req_type),
    .req_read_data (axi_out),
    .axi_busy      (),
    .in_dst        (),
    .in_dst_we     (),
    .in_dst_mask   (),
    .out_dst       (axi_dst),
    .out_dst_we    (axi_dst_we),
    .out_dst_mask  (axi_mask),
    .CLOCK         (clk    ),                                                          
    .RESET         (resetn    ),                                                          
    .AWID          (M_AWID     ),                                                           
    .AWADDR        (M_AWADDR   ),                                                         
    .AWLEN         (M_AWLEN    ),                                                          
    .AWSIZE        (M_AWSIZE   ),                                                         
    .AWBURST       (M_AWBURST  ),                                                        
    .AWLOCK        (M_AWLOCK   ),                                                         
    .AWCACHE       (M_AWCACHE  ),                                                        
    .AWPROT        (M_AWPROT   ),                                                         
    .AWQOS         (M_AWQOS    ),                                                          
    .AWVALID       (M_AWVALID  ),                                                        
    .AWREADY       (M_AWREADY  ),                                                        
    .WDATA         (M_WDATA    ),                                                          
    .WSTRB         (M_WSTRB    ),                                                          
    .WLAST         (M_WLAST    ),                                                          
    .WVALID        (M_WVALID   ),                                                         
    .WREADY        (M_WREADY   ),                                                         
    .ARID          (M_ARID     ),                                                           
    .ARADDR        (M_ARADDR   ),                                                         
    .ARLEN         (M_ARLEN    ),                                                          
    .ARSIZE        (M_ARSIZE   ),                                                         
    .ARBURST       (M_ARBURST  ),                                                        
    .ARLOCK        (M_ARLOCK   ),                                                         
    .ARCACHE       (M_ARCACHE  ),                                                        
    .ARPROT        (M_ARPROT   ),                                                         
    .ARQOS         (M_ARQOS    ),                                                          
    .ARVALID       (M_ARVALID  ),                                                        
    .ARREADY       (M_ARREADY  ),                                                        
    .RID           (M_RID      ),                                                            
    .RDATA         (M_RDATA    ),                                                          
    .RRESP         (M_RRESP    ),                                                          
    .RLAST         (M_RLAST    ),                                                          
    .RVALID        (M_RVALID   ),                                                         
    .RREADY        (M_RREADY   ),                                                         
    .BREADY        (M_BREADY   ),                                                         
    .BVALID        (M_BVALID   ),                                                         
    .BRESP         (M_BRESP    ),                                                          
    .BID           (M_BID      )                                                             
);

wire [11:0] axi_addr;
wire [NUMLANES*LANEWIDTH-1:0] axi_data,axi_readdata;
wire axi_req_en;
wire axi_req_type;

axi4_slave #(
    .P_ADDR_WIDTH(11),
    .P_DATA_WIDTH(128)
)
inst_axi_slave
(
    .req_addr      (axi_addr),
    .req_data      (axi_data),
    .req_en        (axi_req_en),
    .req_type      (axi_req_type),
    .req_read_data (axi_readata),
    .axi_busy      (),
    .CLOCK         (clk),                                                          
    .RESET         (resetn),                                                          
    .AWID          (S_AWID     ),                                                           
    .AWADDR        (S_AWADDR   ),                                                         
    .AWLEN         (S_AWLEN    ),                                                          
    .AWSIZE        (S_AWSIZE   ),                                                         
    .AWBURST       (S_AWBURST  ),                                                        
    .AWLOCK        (S_AWLOCK   ),                                                         
    .AWCACHE       (S_AWCACHE  ),                                                        
    .AWPROT        (S_AWPROT   ),                                                         
    .AWQOS         (S_AWQOS    ),                                                          
    .AWVALID       (S_AWVALID  ),                                                        
    .AWREADY       (S_AWREADY  ),                                                        
    .WDATA         (S_WDATA    ),                                                          
    .WSTRB         (S_WSTRB    ),                                                          
    .WLAST         (S_WLAST    ),                                                          
    .WVALID        (S_WVALID   ),                                                         
    .WREADY        (S_WREADY   ),                                                         
    .ARID          (S_ARID     ),                                                           
    .ARADDR        (S_ARADDR   ),                                                         
    .ARLEN         (S_ARLEN    ),                                                          
    .ARSIZE        (S_ARSIZE   ),                                                         
    .ARBURST       (S_ARBURST  ),                                                        
    .ARLOCK        (S_ARLOCK   ),                                                         
    .ARCACHE       (S_ARCACHE  ),                                                        
    .ARPROT        (S_ARPROT   ),                                                         
    .ARQOS         (S_ARQOS    ),                                                          
    .ARVALID       (S_ARVALID  ),                                                        
    .ARREADY       (S_ARREADY  ),                                                        
    .RID           (S_RID      ),                                                            
    .RDATA         (S_RDATA    ),                                                          
    .RRESP         (S_RRESP    ),                                                          
    .RLAST         (S_RLAST    ),                                                          
    .RVALID        (S_RVALID   ),                                                         
    .RREADY        (S_RREADY   ),                                                         
    .BREADY        (S_BREADY   ),                                                         
    .BVALID        (S_BVALID   ),                                                         
    .BRESP         (S_BRESP    ),                                                          
    .BID           (S_BID      )                                                             
);

  //
  //  ///---------------------------------------


// ----------------------------------------


  //============== Memory Unit =============
 
 wire [127:0] dma_lane_wrdata,dma_lane_rddata;
// wire [NUMLANES-1:0] vmem_local_en;
 wire [11*NUMLANES-1:0] dma_lane_addr;

// assign vmem_local_en = {NUMLANES{pipe_advance[4]}};

 wire squash_per_lane_mem_dstpipe_NC;
 wire squash_per_lane_mem_dstmask_NC;

pipe #(REGIDWIDTH,3) lane_mem_dstpipe (
  .d( dst_s4[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash(squash_per_lane_mem_dstpipe_NC),
  .q({dst[FU_MEM][7],dst[FU_MEM][6],dst[FU_MEM][5],dst[FU_MEM][4]}));

pipe #(1,3) lane_mem_dstwepipe (
  .d( dst_we_s4[FU_MEM]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash( pipe_squash[6:4] ),
  .q({dst_we[((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1)+7-7) : ((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1))],dst_we[((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1)+6-6) : ((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1))],dst_we[((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1)+5-5) : ((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1))],dst_we[((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1))]}));

pipe #(NUMLANES,3) lane_mem_dstmaskpipe (
  .d( vmask[((FU_MEM-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MEM-0)*((NUMLANES-1)-(0)+1))]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[4] ),
  .squash(squash_per_lane_mem_dstmask_NC),
  .q({dst_mask[FU_MEM][7],dst_mask[FU_MEM][6],dst_mask[FU_MEM][5],dst_mask[FU_MEM][4]}));

wire     [ LANEWIDTH*NUMLANES-1 : 0 ]   load_mem_result_s5;

wire [11* NUMLANES-1:0] mux_lane_addr;
wire [NUMLANES*LANEWIDTH-1:0] mux_lane_data, mux_lane_rddata;
wire [NUMLANES-1:0] mux_lane_rden, mux_lane_wren;
wire [NUMLANES-1:0] dma_lane_rden,dma_lane_wren;


 dma_axi_mux#(
   .WIDTH(LANEWIDTH),
   .NUMLANES(NUMLANES),
   .ADDRWIDTH(11)
 )inst_dma_axi_mux(
    .dma_addr(dma_lane_addr),
    .dma_data(dma_lane_wrdata),
    .dma_rden(dma_lane_rden),
    .dma_wren(dma_lane_wren),    
    .dma_out(dma_lane_rddata),   
    .axi_addr(axi_addr),   
    .axi_data(axi_data),    
    .axi_req_en(axi_req_en),  
    .axi_req_type(axi_req_type),
    .axi_read_data(axi_readata),
    .mem_addr(mux_lane_addr),    
    .mem_data(mux_lane_data),    
    .mem_rden(mux_lane_rden),    
    .mem_wren(mux_lane_wren),    
    .mem_readdata(mux_lane_rddata)
 );

 vmem_local inst_vmem_local(
   .clk(clk),
   .resetn(resetn),
   .en(pipe_advance[4]),
   .op(ctrl4_memunit_op),
   .address_a(vbase_s4[10:0]),
   .stride_val_a(vstrideoffset_s4[VCWIDTH-1:0]),
   .offset_a(vr_src1[((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
   .data_a(vr_src2[((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
   .out_a(load_mem_result_s5),
   .last_subvector(mem_last_subvector_s4),
   .address_b(mux_lane_addr),
   .rden_b(mux_lane_rden),
   .wren_b(mux_lane_wren),
   .data_b(mux_lane_data),
   .out_b(mux_lane_rddata)
  );

  dma #(
    .NUMLANES(NUMLANES),
    .WIDTH(LANEWIDTH),
    .ADDRWIDTH(11),
    .DMEM_ADDRWIDTH(32)
  )inst_dma(
   .clk(clk),
   .resetn(resetn),
   .mem_addr(mem_addr),
   .num_bytes(num_bytes),
   .dma_en(dma_en),
   .lane_addr(lane_addr),
   .we(dma_we),
  
   .local_addr(dma_lane_addr),
   .local_wren(dma_lane_wren),
   .local_rden(dma_lane_rden),
   .local_wrdata(dma_lane_wrdata),
   .local_rddata(dma_lane_rddata),
  
   .dma_busy(dma_busy),
   .dbus_address   (dma_dbus_address   ),
   .dbus_readdata  (dma_dbus_readdata  ), 
   .dbus_writedata (dma_dbus_writedata ),
   .dbus_byteen    (dma_dbus_byteen    ),
   .dbus_en        (dma_dbus_en        ),
   .dbus_wren      (dma_dbus_wren      ),
   .dbus_prefetch  (dma_dbus_prefetch  ),
   .dbus_wait      (dma_dbus_wait      ),
   .dbus_data_valid(dma_dbus_data_valid)
  );

  vmem_unit vmem_unit(
    .clk(clk),
    .resetn(resetn),
    .enable(ctrl4_memunit_en && ~stall_mulunit && ~stall_matmul),  //unit on and pipe active
    .en(pipe_advance[6:4]),
    .squash(pipe_squash[6:4]),
    .op(ctrl4_memunit_op),
    .stall(_stall_memunit),
    .last_subvector(mem_last_subvector_s4),
    // Parameters ports
    .cbase(vbase_s4),
    .cstride(vstride[((4-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((4-2)*((VCWIDTH-1)-(0)+1))]),
    .cprefetch(vs_s4[((FU_MEM-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_MEM-0)*((VSWIDTH-1)-(0)+1))]),
    // Vector ports
    .vmask(vmask[((FU_MEM-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MEM-0)*((NUMLANES-1)-(0)+1))]),
    .vstrideoffset(vstrideoffset_s4),
    .vindex(vr_src1[((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
    .vwritedata(vr_src2[((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
    .voutput(load_result_s5),
    .voutput_we(load_result_mask_s5),
    // Writeback ports
    .in_dst(dst_s4[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))]),
    .in_dst_we(dst_we_s4[FU_MEM]),
    //.out_dst(dst[FU_MEM][5]),
    .out_dst(),
    //.out_dst_we(dst_we[FU_MEM][5]),
    .out_dst_we(),
    .in_vs_dst_we(ctrl_vs_we[4]),
    .out_vs_dst_we(ctrl_vs_we[5]),
    // Vector operations ports
    .sa(vstride[((4-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((4-2)*((VCWIDTH-1)-(0)+1))]),
    .dir_left(ctrl4_mem_dir_left),
    // Data memory interface
    .dmem_en(dbus_en),
    .dmem_address(dbus_address),
    .dmem_we(dbus_we),
    .dmem_byteen(dbus_byteen),
    .dmem_writedata(dbus_writedata),
    .dmem_readdata(dbus_readdata),
    .dmem_cachematch(dbus_cachematch),
    .dmem_cachemiss(dbus_cachemiss),
    .dmem_prefetch(dbus_prefetch),
    .dmem_wait(dbus_wait)
    );
  defparam 
    vmem_unit.VPUWIDTH=LANEWIDTH,         // Width of the Vector processing unit
    vmem_unit.NUMLANES=NUMLANES,      // Number of vector lanes
    vmem_unit.LOG2NUMLANES=LOG2NUMLANES, 
    vmem_unit.NUMPARALLELLANES=NUMMEMPARALLELLANES,   // Crossbar size
    vmem_unit.LOG2NUMPARALLELLANES=LOG2NUMMEMPARALLELLANES, 
    vmem_unit.CONTROLWIDTH=VCWIDTH,   // Width of the Control Registers
    vmem_unit.DMEM_WRITEWIDTH=DMEM_WRITEWIDTH, // Width of write bus to memory
    vmem_unit.LOG2DMEM_WRITEWIDTH=LOG2DMEM_WRITEWIDTH,
    vmem_unit.DMEM_READWIDTH=DMEM_READWIDTH,   // Width of read bus from memory
    vmem_unit.LOG2DMEM_READWIDTH=LOG2DMEM_READWIDTH,
    vmem_unit.ELMIDWIDTH=REGIDWIDTH-VRIDWIDTH,
    vmem_unit.REGIDWIDTH=REGIDWIDTH;

  assign dst[FU_MEM][4]=dst_s4[FU_MEM];
  assign dst_we[((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1))]=dst_we_s4[FU_MEM];

  //============== Multiplier Unit (spans stages 4-6) =============

  vmul_unit vmul_unit(
    .clk(clk),
    .resetn(resetn),
    .op(ctrl4_mulshift_op),
    .activate(ctrl4_mulshift_en),
    .en(pipe_advance[6:4]),
    .squash(pipe_squash[6:4]),
    .stall(stall_mulunit),
    .opA(vr_src1[((FU_MUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_MUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
    .opB(vr_src2[((FU_MUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_MUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
    .vshamt( (ctrl4_rshiftnonzero) ? vc_s4[((FU_MUL-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_MUL-0)*((VCWIDTH-1)-(0)+1))] : 0 ),
    .vmask(vmask[((FU_MUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MUL-0)*((NUMLANES-1)-(0)+1))]),
    .in_dst(dst_s4[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1))]),
    .in_dst_we(dst_we_s4[FU_MUL]),
    .out_dst({dst[FU_MUL][6],dst[FU_MUL][5],dst[FU_MUL][4]}),
    .out_dst_we(dst_we[((FU_MUL-0)*((`MAX_STAGES-1)-(4)+1)+6-4) : ((FU_MUL-0)*((`MAX_STAGES-1)-(4)+1))]),
    .out_dst_mask({dst_mask[FU_MUL][6],dst_mask[FU_MUL][5],dst_mask[FU_MUL][4]}),
    .result(mulshift_result_s5)
  );
  defparam 
    vmul_unit.LOG2WIDTH=LOG2LANEWIDTH,   // Width of the Vector processing unit
    vmul_unit.NUMMULLANES=NUMMULLANES,   // Number of vector lanes
    vmul_unit.LOG2NUMLANES=LOG2NUMLANES, // Number of vector lanes
    vmul_unit.REGIDWIDTH=REGIDWIDTH;

  //============== ALU Unit =============

  //If APB value is true, create one ALU per bank (per lane)

  generate
  for (bk=0; bk<1+(NUMBANKS-1)*ALUPERBANK; bk=bk+1)
  begin : aluperbank_gen
    for (k=0; k<NUMLANES; k=k+1)
    begin : lanes4_gen

      vlane_alu #(LANEWIDTH) valu(
        .clk(clk), 
        .resetn(resetn),
        .pipe_en(pipe_advance[4]),
        .pipe_squashn(~pipe_squash[4]),
        .src1(vr_src1[FU_ALU+bk][LANEWIDTH*k +: LANEWIDTH]),
        .src2(vr_src2[FU_ALU+bk][LANEWIDTH*k +: LANEWIDTH]),
        .mask(vf_src1[FU_ALU+bk][k]),  //Note: MERGE is not masked
        .op(ctrl4_alu_op[((bk-0)*((10)-(0)+1)+10-0) : ((bk-0)*((10)-(0)+1))]^ALUOP_ZERO),
        .satsum_op(ctrl4_satsum_op[((bk-0)*((1)-(0)+1)+1-0) : ((bk-0)*((1)-(0)+1))]),
        .satsize_op(ctrl4_satsize_op[((bk-0)*((3)-(0)+1)+3-0) : ((bk-0)*((3)-(0)+1))]),
        .cmp_result(alu_cmpresult_s4[((bk-0)*((NUMLANES-1)-(0)+1)+k-k) : ((bk-0)*((NUMLANES-1)-(0)+1))]),
        .result(alu_result_s5[((bk-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)) +: ( LANEWIDTH)])
        );

      vlane_flagalu vflagalu(
        .clk(clk), 
        .resetn(resetn),
        .src1(vf_src1[FU_FALU+bk][k]),
        .src2(vf_src2[FU_FALU+bk][k]),
        .op(ctrl4_flagalu_op[((bk-0)*((2)-(0)+1)+2-0) : ((bk-0)*((2)-(0)+1))]),
        .result(flagalu_result_s4[((bk-0)*((NUMLANES-1)-(0)+1)+k-k) : ((bk-0)*((NUMLANES-1)-(0)+1))])
        );

      pipereg #(1) flagaluresultreg (
        .d( (ctrl4_vf_wbsel[bk]) ? alu_cmpresult_s4[((bk-0)*((NUMLANES-1)-(0)+1)+k-k) : ((bk-0)*((NUMLANES-1)-(0)+1))] : 
                                   flagalu_result_s4[((bk-0)*((NUMLANES-1)-(0)+1)+k-k) : ((bk-0)*((NUMLANES-1)-(0)+1))]),
        .clk(clk),
        .resetn(resetn),
        .en( pipe_advance[4] ), //Advance unless next stage stalled
        .squashn( 1'b1 ),
        .q(flagalu_result_s5[((bk-0)*((NUMLANES-1)-(0)+1)+k-k) : ((bk-0)*((NUMLANES-1)-(0)+1))])
        );
    end

    wire squash_aludstpipe_NC;
    pipe #(REGIDWIDTH,1) aludstpipe (
      .d( dst_s4[FU_ALU+bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_aludstpipe_NC),
      .q({dst[FU_ALU+bk][5],dst[FU_ALU+bk][4]}));

    pipe #(1,3) aludstwepipe (
      .d( dst_we_s4[FU_ALU+bk] & ~ctrl4_vf_wbsel[bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:4] ),
      .squash( pipe_squash[6:4] ),
      .q({dst_we[FU_ALU+bk][5],dst_we[FU_ALU+bk][4]}));

    wire squash_aludstmaskpipe_NC;
    pipe #(NUMLANES,1) aludstmaskpipe (
      .d( vmask[FU_ALU+bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_aludstmaskpipe_NC),
      .q({dst_mask[FU_ALU+bk][5],dst_mask[FU_ALU+bk][4]}));

    wire squash_faludstpipe_NC;
    pipe #(REGIDWIDTH,1) faludstpipe (
      .d( dst_s4[FU_FALU+bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_faludstpipe_NC),
      .q({dst[FU_FALU+bk][5],dst[FU_FALU+bk][4]}));

    pipe #(1,3) faludstwepipe (
      .d( dst_we_s4[FU_FALU+bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:4] ),
      .squash( pipe_squash[6:4] ),
      .q({dst_we[FU_FALU+bk][5],dst_we[FU_FALU+bk][4]}));
  end
  endgenerate


 //This code is just for assigning from signals connected
 //to the matmul, which are linear multi bit signals, to the
 //multi-dimensional signals.
 wire [NUMLANES*REGIDWIDTH-1:0] dst_matmul;
 wire [NUMLANES-1:0] dst_we_matmul;
 wire [NUMLANES*NUMLANES-1:0] dst_mask_matmul;

 assign dst[FU_MATMUL][4] = dst_matmul[(REGIDWIDTH*NUMLANES)-1:REGIDWIDTH*(NUMLANES-1)];
 assign dst_mask[FU_MATMUL][4] = dst_mask_matmul[(NUMLANES*NUMLANES)-1:NUMLANES*(NUMLANES-1)];
 assign dst_we[((FU_MATMUL-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_MATMUL-0)*((`MAX_STAGES-1)-(4)+1))] = dst_we_matmul[NUMLANES-1];


///////////////////////////
// Matmul unit
///////////////////////////
//

wire out_data_avail;

matmul_unit #(REGIDWIDTH,`MATMUL_STAGES,NUMLANES) u_matmul(
.clk(clk),
.resetn(resetn),
.activate(ctrl4_matmul_en),
.en({NUMLANES{pipe_advance[6]}}),
.squash({NUMLANES{pipe_squash[6]}}),
.stall(stall_matmul),
.stall_matmul(temp_stall_matmul),
.a_data(vr_src1[((FU_MATMUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+NUMLANES*LANEWIDTH-1-0) : ((FU_MATMUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
.b_data(vr_src2[((FU_MATMUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+NUMLANES*LANEWIDTH-1-0) : ((FU_MATMUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
.validity_mask_a_rows(matmul_masks_in[1*`MAT_MUL_SIZE-1:0*`MAT_MUL_SIZE]),
.validity_mask_a_cols(matmul_masks_in[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE]),
.validity_mask_b_rows(matmul_masks_in[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE]),
.validity_mask_b_cols(matmul_masks_in[3*`MAT_MUL_SIZE-1:2*`MAT_MUL_SIZE]),
.c_data(matmul_out), 
.vmask(vmask[((FU_MATMUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MATMUL-0)*((NUMLANES-1)-(0)+1))]),
.in_dst(dst_s4[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1))]),
.in_dst_we(dst_we_s4[FU_MATMUL]),
.out_dst(dst_matmul),
.out_dst_we(dst_we_matmul),
.out_data_avail(out_data_avail),
.out_dst_mask(dst_mask_matmul)
);

transpose #(
  .WIDTH(LANEWIDTH),
  .NUMSTAGES(NUMLANES)
)u_transpose(
  .clk(clk),
  .resetn(resetn),
  .en(ctrl4_transpose_en),
  .a(vr_src1[((FU_TRANSPOSE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+NUMLANES*LANEWIDTH-1-0) : ((FU_TRANSPOSE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
  .out(transpose_out),
  .busy(transpose_busy),
  .valid(transpose_valid)
);
 
 wire squash_transpose_dstpipe_NC;
 wire squash_transpose_dstmask_NC;

pipe #(REGIDWIDTH,9) transpose_dstpipe (
  .d( dst_s4[((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1))] ),  
  .clk(clk),
  .resetn(resetn),
  .en(pipe_advance[12:4]),
  .squash(squash_transpose_dstpipe_NC),
  .q({dst[FU_TRANSPOSE][13],
      dst[FU_TRANSPOSE][12],
      dst[FU_TRANSPOSE][11],
      dst[FU_TRANSPOSE][10],
      dst[FU_TRANSPOSE][9],
      dst[FU_TRANSPOSE][8],
      dst[FU_TRANSPOSE][7],
      dst[FU_TRANSPOSE][6],
      dst[FU_TRANSPOSE][5],
      dst[FU_TRANSPOSE][4]}));

pipe #(1,9) transpose_dstwepipe (
  .d( dst_we_s4[FU_TRANSPOSE]),  
  .clk(clk),
  .resetn(resetn),
//  .en(pipe_advance[11:4]),
  .en(9'hff),
//  .squash( pipe_squash[11:4] ),
  .squash(9'h00 ),
  .q({dst_we[((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1)+13-13) : ((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1))],
      dst_we[((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1)+12-12) : ((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1))],
      dst_we[((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1)+11-11) : ((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1))],
      dst_we[((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1)+10-10) : ((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1))],
      dst_we[((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1)+9-9) : ((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1))],
      dst_we[((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1)+8-8) : ((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1))],
      dst_we[((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1)+7-7) : ((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1))],
      dst_we[((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1)+6-6) : ((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1))],
      dst_we[((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1)+5-5) : ((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1))],
      dst_we[((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1))]}));

pipe #(NUMLANES,9) transpose_dstmaskpipe (
  .d(vmask[((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1))]),  
  .clk(clk),
  .resetn(resetn),
  .en(pipe_advance[12:4]),
  .squash(squash_transpose_dstmask_NC),
  .q({dst_mask[FU_TRANSPOSE][13],
      dst_mask[FU_TRANSPOSE][12],
      dst_mask[FU_TRANSPOSE][11],
      dst_mask[FU_TRANSPOSE][10],
      dst_mask[FU_TRANSPOSE][9],
      dst_mask[FU_TRANSPOSE][8],
      dst_mask[FU_TRANSPOSE][7],
      dst_mask[FU_TRANSPOSE][6],
      dst_mask[FU_TRANSPOSE][5],
      dst_mask[FU_TRANSPOSE][4]}));

reg [3*NUMLANES-1:0] permute_row_col_vec;

always@(*)begin
  permute_row_col_vec[3*1-1:3*0] = vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))][0*LANEWIDTH+3 : 0*LANEWIDTH];
  permute_row_col_vec[3*2-1:3*1] = vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))][1*LANEWIDTH+3 : 1*LANEWIDTH];
  permute_row_col_vec[3*3-1:3*2] = vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))][2*LANEWIDTH+3 : 2*LANEWIDTH];
  permute_row_col_vec[3*4-1:3*3] = vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))][3*LANEWIDTH+3 : 3*LANEWIDTH];
  permute_row_col_vec[3*5-1:3*4] = vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))][4*LANEWIDTH+3 : 4*LANEWIDTH];
  permute_row_col_vec[3*6-1:3*5] = vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))][5*LANEWIDTH+3 : 5*LANEWIDTH];
  permute_row_col_vec[3*7-1:3*6] = vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))][6*LANEWIDTH+3 : 6*LANEWIDTH];
  permute_row_col_vec[3*8-1:3*7] = vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))][7*LANEWIDTH+3 : 7*LANEWIDTH];
end

wire [REGIDWIDTH-1:0] permute_dst;
wire [NUMLANES-1:0] permute_mask;
wire permute_dst_we;

permute #(
  .WIDTH(LANEWIDTH),
  .NUMSTAGES(NUMLANES),
  .REGIDWIDTH(REGIDWIDTH)
) u_permute (
  .clk(clk),
  .resetn(resetn),
  .read(ctrl4_permute_read),
  .en(ctrl4_permute_en),
  .row_col_op(ctrl4_permute_op),
  .row_col_num(permute_row_col_vec),
  .load(ctrl4_permute_store),
  .a(vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+NUMLANES*LANEWIDTH-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
  .in_dst(dst_s4[((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1))]),
  .in_dst_we(dst_we_s4[FU_PERMUTE]),
  .in_dst_mask(vmask[((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1))]),
  .out_dst(permute_dst),
  .out_dst_we(permute_dst_we),
  .out_dst_mask(permute_mask),
  .out(permute_out)
); 

//wire squash_permute_dstpipe_NC;
//wire squash_permute_dstmask_NC;
//
//pipe #(REGIDWIDTH,1) permute_dstpipe (
//  .d( dst_s4[FU_PERMUTE] ),  
//  .clk(clk),
//  .resetn(resetn),
//  .en(pipe_advance[4]),
//  .squash(squash_permute_dstpipe_NC),
//  .q({dst[FU_PERMUTE][5],
//      dst[FU_PERMUTE][4]}));
//
//pipe #(1,1) permute_dstwepipe (
//  .d( dst_we_s4[FU_PERMUTE]),  
//  .clk(clk),
//  .resetn(resetn),
////  .en(pipe_advance[11:4]),
//  .en(pipe_advance[4]),
////  .squash( pipe_squash[11:4] ),
//  .squash(0),
//  .q({dst_we[FU_PERMUTE][5],
//      dst_we[FU_PERMUTE][4]}));
//
//pipe #(NUMLANES,1) permute_dstmaskpipe (
//  .d(vmask[FU_PERMUTE]),  
//  .clk(clk),
//  .resetn(resetn),
//  .en(pipe_advance[4]),
//  .squash(squash_permute_dstmask_NC),
//  .q({dst_mask[FU_PERMUTE][5],
//      dst_mask[FU_PERMUTE][4]}));
//
trp_unit #(.WIDTH(LANEWIDTH)) u_trp (
.clk(clk),
.resetn(resetn),
.en(ctrl4_trp_en),
.a(vr_src1[((FU_TRP-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+NUMLANES*LANEWIDTH-1-0) : ((FU_TRP-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
.mode(trp_mode_s4),
.read(trp_read),
.busy(trp_busy),
.valid(trp_valid),
.out(trp_out));

 wire squash_trp_dstpipe_NC;
 wire squash_trp_dstmask_NC;

pipe #(REGIDWIDTH,2) trp_dstpipe (
  .d( dst_s4[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1))] ),  
  .clk(clk),
  .resetn(resetn),
  .en(~trp_busy ),
  .squash(squash_trp_dstpipe_NC),
  .q({dst[FU_TRP][6],dst[FU_TRP][5],dst[FU_TRP][4]}));

pipe #(1,2) trp_dstwepipe (
  .d( dst_we_s4[FU_TRP]),  
  .clk(clk),
  .resetn(resetn),
  .en(~trp_busy ),
  .squash( pipe_squash[4] ),
  .q({dst_we[((FU_TRP-0)*((`MAX_STAGES-1)-(4)+1)+6-6) : ((FU_TRP-0)*((`MAX_STAGES-1)-(4)+1))],dst_we[((FU_TRP-0)*((`MAX_STAGES-1)-(4)+1)+5-5) : ((FU_TRP-0)*((`MAX_STAGES-1)-(4)+1))],dst_we[((FU_TRP-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_TRP-0)*((`MAX_STAGES-1)-(4)+1))]}));

pipe #(NUMLANES,1) trp_dstmaskpipe (
  .d( vmask[((FU_TRP-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRP-0)*((NUMLANES-1)-(0)+1))]),  
  .clk(clk),
  .resetn(resetn),
  .en(~trp_busy),
  .squash(squash_trp_dstmask_NC),
  .q({dst_mask[FU_TRP][5],dst_mask[FU_TRP][4]}));

///////////////////////////
// Bfloat unit
///////////////////////////
genvar g_func;
wire[5*NUMLANES-1:0] bfadd_excp;
wire[5*NUMLANES-1:0] bfmult_excp;
wire bfadder_output_valid;
wire [NUMLANES-1:0] bfmult_output_valid;

wire squash_bfadder_dstmask_NC;
wire squash_bfadder_dstpipe_NC;
wire squash_bfmult_dstmask_NC;
wire squash_bfmult_dstpipe_NC;

wire squash_activation_dstmask_NC;
wire squash_activation_dstpipe_NC;

pipe #(REGIDWIDTH,3) bfadddstpipe (
  .d( dst_s4[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1))] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash(squash_bfadder_dstpipe_NC),
  .q({dst[FU_BFADDER][7],dst[FU_BFADDER][6],dst[FU_BFADDER][5],dst[FU_BFADDER][4]}));

pipe #(1,3) bfadddstwepipe (
  .d( dst_we_s4[FU_BFADDER]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash( pipe_squash[6:4] ),
  .q({dst_we[((FU_BFADDER-0)*((`MAX_STAGES-1)-(4)+1)+7-7) : ((FU_BFADDER-0)*((`MAX_STAGES-1)-(4)+1))],dst_we[((FU_BFADDER-0)*((`MAX_STAGES-1)-(4)+1)+6-6) : ((FU_BFADDER-0)*((`MAX_STAGES-1)-(4)+1))],dst_we[((FU_BFADDER-0)*((`MAX_STAGES-1)-(4)+1)+5-5) : ((FU_BFADDER-0)*((`MAX_STAGES-1)-(4)+1))],dst_we[((FU_BFADDER-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_BFADDER-0)*((`MAX_STAGES-1)-(4)+1))]}));

pipe #(NUMLANES,3) bfadddstmaskpipe (
  .d( vmask[((FU_BFADDER-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFADDER-0)*((NUMLANES-1)-(0)+1))] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash(squash_bfadder_dstmask_NC),
  .q({dst[FU_BFMULT][7],dst[FU_BFMULT][6],dst_mask[FU_BFADDER][5],dst_mask[FU_BFADDER][4]}));

pipe #(REGIDWIDTH,3) bfmultdstpipe (
  .d( dst_s4[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1))] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash(squash_per_lane_mem_NC),
  .q({dst[FU_BFMULT][7],dst[FU_BFMULT][6],dst[FU_BFMULT][5],dst[FU_BFMULT][4]}));

pipe #(1,3) bfmultdstwepipe (
  .d( dst_we_s4[FU_BFMULT]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash( pipe_squash[6:4] ),
  .q({dst_we[((FU_BFMULT-0)*((`MAX_STAGES-1)-(4)+1)+7-7) : ((FU_BFMULT-0)*((`MAX_STAGES-1)-(4)+1))],dst_we[((FU_BFMULT-0)*((`MAX_STAGES-1)-(4)+1)+6-6) : ((FU_BFMULT-0)*((`MAX_STAGES-1)-(4)+1))],dst_we[((FU_BFMULT-0)*((`MAX_STAGES-1)-(4)+1)+5-5) : ((FU_BFMULT-0)*((`MAX_STAGES-1)-(4)+1))],dst_we[((FU_BFMULT-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_BFMULT-0)*((`MAX_STAGES-1)-(4)+1))]}));

pipe #(NUMLANES,3) bfmultdstmaskpipe (
  .d( vmask[((FU_BFMULT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFMULT-0)*((NUMLANES-1)-(0)+1))] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[4] ),
  .squash(squash_bfmult_dstmask_NC),
  .q({dst_mask[FU_BFMULT][7],dst_mask[FU_BFMULT][6],dst_mask[FU_BFMULT][5],dst_mask[FU_BFMULT][4]}));

pipe #(REGIDWIDTH,1) actdstpipe (
  .d( dst_s4[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1))] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[5:4] ),
  .squash(squash_activation_dstpipe_NC),
  .q({dst[FU_ACT][5],dst[FU_ACT][4]}));

pipe #(1,1) actdstwepipe (
  .d( dst_we_s4[FU_ACT]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[5:4] ),
  .squash( pipe_squash[5:4] ),
  .q({dst_we[((FU_ACT-0)*((`MAX_STAGES-1)-(4)+1)+5-5) : ((FU_ACT-0)*((`MAX_STAGES-1)-(4)+1))],dst_we[((FU_ACT-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_ACT-0)*((`MAX_STAGES-1)-(4)+1))]}));

pipe #(NUMLANES,1) actdstmaskpipe (
  .d(vmask[((FU_ACT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_ACT-0)*((NUMLANES-1)-(0)+1))]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[5:4]),
  .squash(squash_activation_dstmask_NC),
  .q({dst_mask[FU_ACT][5],dst_mask[FU_ACT][4]}));

  generate
  for(g_func =0; g_func <NUMLANES; g_func = g_func+1) begin

FPAddSub bfloat_add(
	.clk(clk),
	.rst(~resetn),
        .en(ctrl4_bfadder_en),
	.a(vr_src1[((FU_BFADDER-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)) +: ( LANEWIDTH)]),
	.b(vr_src2[((FU_BFADDER-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)) +: ( LANEWIDTH)]),
	.operation(1'b0),
	.result(bfadder_result_s5[g_func*LANEWIDTH +: LANEWIDTH]),
        .valid(bfadder_output_valid[g_func]),
	.flags(bfadd_excp[5*g_func +: 5]));

FPMult_16 bfloat_mult(
	.clk(clk),
	.rst(~resetn),
        .en(ctrl4_bfmult_en),
	.a(vr_src1[((FU_BFMULT-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)) +: ( LANEWIDTH)]),
	.b(vr_src1[((FU_BFMULT-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)) +: ( LANEWIDTH)]),
	.result(bfmult_result_s5[g_func*LANEWIDTH +: LANEWIDTH]),
        .valid(bfmult_output_valid[g_func]),
	.flags(bfmult_excp[5*g_func +: 5]));


//    bfloat_adder #(REGIDWIDTH) bf_add(
//    .clk(clk),
//    .resetn(resetn),
//    .en(ctrl4_bfadder_en),
//    .stall(stall_bf_adder),
//    .a(vr_src1[FU_BFADDER][g_func * LANEWIDTHE +: LANEWIDTH]),
//    .b(vr_src2[FU_BFADDER][g_func * LANEWIDTHE +: LANEWIDTH]),
//    .out(bfadder_result_s5[g_func*LANEWIDTH +: LANEWIDTH])
//    );
//    
//    bfloat_mult #(REGIDWIDTH) bf_mult(
//    .clk(clk),
//    .resetn(resetn),
//    .en(ctrl4_bfmult_en),
//    .stall(stall_bf_adder),
//    .a(vr_src1[FU_BFMULT][g_func * LANEWIDTHE +: LANEWIDTH]),
//    .b(vr_src2[FU_BFMULT][g_func * LANEWIDTHE +: LANEWIDTH]),
//    .out(bfmult_result_s5[g_func*LANEWIDTH +: LANEWIDTH])
//    );
 
///////////////////////////
// activation unit
///////////////////////////

    activation #(LANEWIDTH) inst_activation(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_ACT-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)) +: ( LANEWIDTH)]),
    .out(act_result_s5[g_func*LANEWIDTH +: LANEWIDTH])
    );
  end
 endgenerate


/******************************************************************************/
/************************** WB Pipeline Stage ********************************/
/******************************************************************************/

  generate
  for (ba=0; ba<1+(NUMBANKS-1)*ALUPERBANK; ba=ba+1)
  begin : wbaluperbank_gen
    assign wb_dst[FU_ALU+ba]=dst[FU_ALU+ba][5];
    assign wb_dst_we[FU_ALU+ba]=dst_we[FU_ALU+ba][5] && ~pipe_squash[5];
    assign wb_dst_mask[FU_ALU+ba]=dst_mask[FU_ALU+ba][5];
    assign D_wb_last_subvector[FU_ALU+ba]=D_last_subvector_s5[FU_ALU+ba];

    assign D_wb_last_subvector[FU_FALU+ba]=D_last_subvector_s5[FU_FALU+ba];
  end
  endgenerate

  assign wb_dst[FU_MEM]=dst[FU_MEM][5];
  assign wb_dst_we[FU_MEM]=dst_we[((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1)+5-5) : ((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1))] && (~pipe_advance[5]|~pipe_squash[5]);
  assign wb_dst_mask[FU_MEM]=dst_mask[FU_MEM][5];
  assign D_wb_last_subvector[FU_MEM]=D_last_subvector_s5[FU_MEM];

  assign wb_dst[FU_MUL]=dst[FU_MUL][5];
  assign wb_dst_we[FU_MUL]=dst_we[((FU_MUL-0)*((`MAX_STAGES-1)-(4)+1)+5-5) : ((FU_MUL-0)*((`MAX_STAGES-1)-(4)+1))] && ~pipe_squash[5];
  assign wb_dst_mask[FU_MUL]=dst_mask[FU_MUL][5];
  assign D_wb_last_subvector[FU_MUL]=D_last_subvector_s5[FU_MUL];


  assign wb_dst[FU_BFADDER] = dst[FU_BFADDER][7];
  assign wb_dst_we[FU_BFADDER] = dst_we[((FU_BFADDER-0)*((`MAX_STAGES-1)-(4)+1)+7-7) : ((FU_BFADDER-0)*((`MAX_STAGES-1)-(4)+1))] && ~pipe_squash[7];
  assign wb_dst_mask[FU_BFADDER] = dst_mask[FU_BFADDER][7];
  assign D_wb_last_subvector[FU_BFADDER] = D_last_subvector_s5[FU_BFADDER];

  assign wb_dst[FU_BFMULT] = dst[FU_BFMULT][7];
  assign wb_dst_we[FU_BFMULT] = dst_we[((FU_BFMULT-0)*((`MAX_STAGES-1)-(4)+1)+7-7) : ((FU_BFMULT-0)*((`MAX_STAGES-1)-(4)+1))] && ~pipe_squash[7];
  assign wb_dst_mask[FU_BFMULT] = dst_mask[FU_BFMULT][7];
  assign D_wb_last_subvector[FU_BFMULT] = D_last_subvector_s5[FU_BFMULT];

  assign wb_dst[FU_ACT] = dst[FU_ACT][5];
  assign wb_dst_we[FU_ACT] = dst_we[((FU_ACT-0)*((`MAX_STAGES-1)-(4)+1)+5-5) : ((FU_ACT-0)*((`MAX_STAGES-1)-(4)+1))] && ~pipe_squash[5];
  assign wb_dst_mask[FU_ACT] = dst_mask[FU_ACT][5];
  assign D_wb_last_subvector[FU_ACT] = D_last_subvector_s5[FU_ACT];

  assign wb_dst[FU_TRANSPOSE] = dst[FU_TRANSPOSE][13];
  assign wb_dst_we[FU_TRANSPOSE] = dst_we[((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1)+13-13) : ((FU_TRANSPOSE-0)*((`MAX_STAGES-1)-(4)+1))];
  assign wb_dst_mask[FU_TRANSPOSE] = dst_mask[FU_TRANSPOSE][13];
  assign D_wb_last_subvector[FU_TRANSPOSE] = D_last_subvector_s5[FU_TRANSPOSE];

  assign wb_dst[((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1))] = permute_dst;
  assign wb_dst_we[FU_PERMUTE] = permute_dst_we;
  assign wb_dst_mask[((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1))] = permute_mask;
  assign D_wb_last_subvector[FU_PERMUTE] = D_last_subvector_s5[FU_PERMUTE];

  assign wb_dst[((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1))] = axi_dst;
  assign wb_dst_we[FU_AXI] = axi_dst_we;
  assign wb_dst_mask[((FU_AXI-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_AXI-0)*((NUMLANES-1)-(0)+1))] = axi_mask;
  assign D_wb_last_subvector[FU_AXI] = D_last_subvector_s5[FU_AXI];

  assign wb_dst[FU_TRP] = dst[FU_TRP][5];
  assign wb_dst_we[FU_TRP] = dst_we[((FU_TRP-0)*((`MAX_STAGES-1)-(4)+1)+5-5) : ((FU_TRP-0)*((`MAX_STAGES-1)-(4)+1))] && trp_valid;
  assign wb_dst_mask[FU_TRP] = dst_mask[FU_TRP][5];
  assign D_wb_last_subvector[FU_TRP] = D_last_subvector_s5[FU_TRP];

  assign wb_dst[FU_MATMUL]=dst[FU_MATMUL][4];
  assign wb_dst_we[FU_MATMUL]=dst_we[((FU_MATMUL-0)*((`MAX_STAGES-1)-(4)+1)+4-4) : ((FU_MATMUL-0)*((`MAX_STAGES-1)-(4)+1))] && out_data_avail;
  assign wb_dst_mask[FU_MATMUL]=dst_mask[FU_MATMUL][4];
  //TODO: There is no code that assigns to the s31 var used below. Need to add that code
  //This is only a debug var, so it doesn't affect functionality
  assign D_wb_last_subvector[FU_MATMUL]=D_last_subvector_s31[FU_MATMUL];

  // ******************  Map functional units to banks ******************
  always@*
    for (bw=0; bw<NUMBANKS; bw=bw+1)
    begin
      vr_c_we[bw]=(wb_dst_we[FU_MUL] && `LO(wb_dst[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_MATMUL] && `LO(wb_dst[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_BFADDER] && `LO(wb_dst[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_BFMULT] && `LO(wb_dst[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_AXI] && `LO(wb_dst[((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_ACT] && `LO(wb_dst[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_TRP] && `LO(wb_dst[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_PERMUTE] && `LO(wb_dst[((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_TRANSPOSE] && `LO(wb_dst[((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_MEM] && `LO(wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_ALU] && `LO(wb_dst[((FU_ALU-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_ALU-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw && ALUPERBANK==0) ||
                  (wb_dst_we[FU_ALU+bw] && ALUPERBANK!=0);

      //TODO: Update this code for matmul. This is for debug only, so skipping it for now.
      //Tells test_bench when to record register file contents.
      //Record if instruction writes to VRF, on last subvector, and not stalled
      D_wb_instrdone[bw] = pipe_advance[5] && (
        ((ALUPERBANK==0) ?
          (dst_we[((FU_ALU-0)*((`MAX_STAGES-1)-(4)+1)+5-5) : ((FU_ALU-0)*((`MAX_STAGES-1)-(4)+1))] && D_wb_last_subvector[FU_ALU] && `LO(wb_dst[FU_ALU],LOG2NUMBANKS)==bw) :
          (dst_we[FU_ALU+bw][5] && D_wb_last_subvector[FU_ALU+bw])) || 
        (dst_we[((FU_MUL-0)*((`MAX_STAGES-1)-(4)+1)+5-5) : ((FU_MUL-0)*((`MAX_STAGES-1)-(4)+1))] && D_wb_last_subvector[FU_MUL] && `LO(wb_dst[FU_MUL],LOG2NUMBANKS)==bw) || 
        (dst_we[((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1)+5-5) : ((FU_MEM-0)*((`MAX_STAGES-1)-(4)+1))] && D_wb_last_subvector[FU_MEM] && `LO(wb_dst[FU_MEM],LOG2NUMBANKS)==bw));

      //Take matmul output
      if (wb_dst_we[FU_MATMUL] && `LO(wb_dst[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_MATMUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MATMUL-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= matmul_out;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MATMUL];
      end
      // Transpose Block writeback      
      else if (wb_dst_we[FU_TRANSPOSE] && `LO(wb_dst[((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= transpose_out;
        //Do ALU and FALU for last subvector
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_TRANSPOSE];
      end
      // permute block writeback
      else if (wb_dst_we[FU_PERMUTE] && `LO(wb_dst[((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= permute_out;
        //Do ALU and FALU for last subvector
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_PERMUTE];
      end
      // trp block  writeback
      else if (wb_dst_we[FU_TRP] && `LO(wb_dst[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_TRP-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRP-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= trp_out;
        //Do ALU and FALU for last subvector
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_TRP];
      end
      //Take bfadder output
      else if (wb_dst_we[FU_AXI] && `LO(wb_dst[((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_AXI-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_AXI-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= axi_out;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_AXI];
      end
      else if (wb_dst_we[FU_BFADDER] && `LO(wb_dst[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_BFADDER-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFADDER-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= bfadder_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_BFADDER];
      end
      else if (wb_dst_we[FU_BFMULT] && `LO(wb_dst[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_BFMULT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFMULT-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= bfmult_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_BFMULT];
      end
      else if (wb_dst_we[FU_ACT] && `LO(wb_dst[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_ACT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_ACT-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= act_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_ACT];
      end
      //Take multiplier output
      else if (wb_dst_we[FU_MUL] && `LO(wb_dst[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_MUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MUL-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= mulshift_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MUL];
      end
      //Take Memory unit output
      else if (wb_dst_we[FU_MEM] && `LO(wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_MEM-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MEM-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= load_mem_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MEM];
      end
      else
      //Take ALU output
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[FU_ALU+bw*ALUPERBANK];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_ALU+bw*ALUPERBANK]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[FU_ALU+bw*ALUPERBANK];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= alu_result_s5[((bw*ALUPERBANK-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw*ALUPERBANK-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))];
        //Do ALU and FALU for last subvector
        D_last_subvector_done[bw]=
          (D_wb_last_subvector[FU_ALU+bw*ALUPERBANK] && `LO(wb_dst[FU_ALU+bw*ALUPERBANK],LOG2NUMBANKS)==bw) |
          (D_wb_last_subvector[FU_FALU+bw*ALUPERBANK] && `LO(dst[FU_FALU+bw*ALUPERBANK][5],LOG2NUMBANKS)==bw);
      end
      for (j=0; j<NUMLANES; j=j+1)
        vr_c_byteen[bw*VPW*NUMLANES + j*VPW +: VPW ]={VPW{vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+j-j) : ((bw-0)*((NUMLANES-1)-(0)+1))]}};

      //*********** Flag writeback ***********
      vf_c_reg[bw*BANKREGIDWIDTH+:BANKREGIDWIDTH]=dst[FU_FALU+bw*ALUPERBANK][5]>>LOG2NUMBANKS;
      vf_c_writedatain[bw*NUMLANES+:NUMLANES]=flagalu_result_s5[((bw*ALUPERBANK-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw*ALUPERBANK-0)*((NUMLANES-1)-(0)+1))];
      vf_c_we[bw]=dst_we[FU_FALU+bw*ALUPERBANK][5] && `LO(dst[FU_FALU+bw*ALUPERBANK][5],LOG2NUMBANKS)==bw;
    end

  //********** Scalar writeback ***********
  assign vs_wetrack={ctrl_vs_we[5:4],|ctrl3_vs_we,ctrl2_vs_we};
  assign vs_we=ctrl_vs_we[5] & load_result_mask_s5[0];
  assign vs_dst=wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))][`VRID_RANGE];
  assign vs_writedata=load_result_s5[LANEWIDTH-1:0];

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vlanes.v.temp
//////////////////////////////////////////////////////////////////////

module vpu (
    clk,
    resetn,

    // Instruction interface
    instr,
    instr_en,     // tells when instr is valid and available
    instr_wait,   // if high says vpu is not ready to receive
    has_memop,    // indicates vector pipeline has a memory operation

    // For mtc2/ctc2 instructions
    scalar_in,    
    scalar_in_en,
    scalar_in_wait,

    // For cfc2 instructions
    scalar_out,
    scalar_out_en,
    scalar_out_wait,
   
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
    S_BID,     
    

    // Data memory interface
    dbus_address,
    dbus_en,
    dbus_we,
    dbus_byteen,
    dbus_writedata,
    dbus_readdata,
    dbus_cachematch,
    dbus_cachemiss,
    dbus_prefetch,
    dbus_wait,

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

parameter LOG2NUMLANES=`LOG2NUMLANES;
parameter LOG2MVL=`LOG2MVL;
parameter LOG2VPW=`LOG2VPW;
parameter LOG2LANEWIDTH=`LOG2LANEWIDTHBITS;
parameter LOG2NUMMEMLANES=`LOG2NUMMEMLANES;
parameter NUMMULLANES=2**`LOG2NUMMULLANES;
parameter LOG2NUMBANKS=`LOG2NUMBANKS;
parameter ALUPERBANK=`ALUPERBANK;

parameter LOG2DMEM_WRITEWIDTH=7;  // Log2 of Width of write bus to cache
parameter LOG2DMEM_READWIDTH=7;   // Log2 of Width of read bus from cache

parameter NUMLANES=2**LOG2NUMLANES;
parameter MVL=2**LOG2MVL;
parameter VPW=2**LOG2VPW;
parameter LANEWIDTH=2**LOG2LANEWIDTH;
parameter DMEM_WRITEWIDTH=2**LOG2DMEM_WRITEWIDTH; //Width of write bus to memory
parameter DMEM_READWIDTH=2**LOG2DMEM_READWIDTH; // Width of read bus from memory
parameter NUMMEMLANES=2**LOG2NUMMEMLANES;
parameter NUMBANKS=2**LOG2NUMBANKS;

parameter VCWIDTH=32;
parameter NUMVCREGS=64;
parameter LOG2NUMVCREGS=6;
parameter NUMNONMEMVCREGS=32;
parameter LOG2NUMNONMEMVCREGS=5;
parameter NUMVBASEREGS=16;
parameter LOG2NUMVBASEREGS=4;
parameter NUMVINCREGS=8;
parameter LOG2NUMVINCREGS=3;
parameter NUMVSTRIDEREGS=8;
parameter LOG2NUMVSTRIDEREGS=3;

parameter VSWIDTH=32;
parameter NUMVSREGS=32;
parameter LOG2NUMVSREGS=5;

parameter BIT_VSSRC2=6;
parameter BIT_VSSRC1=7;

//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: visa.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/vector/visa.v
//////////////////////////////////////////////////////////////////////
parameter COP2_VADD           = 'b10z0000000;
parameter COP2_VADD_U         = 'b10z0000001;
parameter COP2_VSUB           = 'b10zz000010;
parameter COP2_VSUB_U         = 'b10zz000011;
parameter COP2_VMULHI         = 'b10z0000100;
parameter COP2_VMULHI_U       = 'b10z0000101;
parameter COP2_VDIV           = 'b10zz000110; //Using as matmul
//parameter COP2_VBFADD         = 'b0100000001; //Using BF16 add
//parameter COP2_VBFMULT        = 'b0100000010; //Using BF16 MULT
//parameter COP2_VACT           = 'b0100000011; //Using ACT
//parameter COP2_VTRP           = 'b0100000100; //Using ACT
parameter COP2_VDIV_U         = 'b10zz000111;

//parameter COP2_VMOD           = 'b10zz001000;
parameter COP2_VBFADD           = 'b10zz001000;  // USING bfloat add Instr: vmod.vv vrdest, vrsrc1,vrsrc2

//parameter COP2_VMOD_U         = 'b10zz001001;
parameter COP2_VBFMULT         = 'b10zz001001;  // Using bfloat mult Instr: vmod.u.vv vrdest, vrsrc1,vrsrc2
parameter COP2_VMOD_U         = 'b10zz001001;
parameter COP2_VCMP_EQ        = 'b10zz001010;
parameter COP2_VCMP_NE        = 'b10zz001100;
parameter COP2_VCMP_LT        = 'b10zz001110;
parameter COP2_VCMP_U_LT      = 'b10zz001111;
parameter COP2_VCMP_LE        = 'b10zz010000;
parameter COP2_VCMP_U_LE      = 'b10zz010001;
parameter COP2_VMIN           = 'b10z0010010;
parameter COP2_VMIN_U         = 'b10z0010011;
parameter COP2_VMAX           = 'b10z0010100;
parameter COP2_VMAX_U         = 'b10z0010101;
parameter COP2_VMULLO         = 'b10z0010110;
parameter COP2_VABS           = 'b1000010111;
parameter COP2_VAND           = 'b10z0011000;
parameter COP2_VOR            = 'b10z0011001;
parameter COP2_VXOR           = 'b10z0011010;
parameter COP2_VNOR           = 'b10z0011011;
parameter COP2_VSLL           = 'b10zz011100;
parameter COP2_VSRL           = 'b10zz011101;
parameter COP2_VSRA           = 'b10zz011110;
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
parameter COP2_VSADD          = 'b10z0100010;
parameter COP2_VSADD_U        = 'b10z0100011;
parameter COP2_VSSUB          = 'b10zz100100;
parameter COP2_VSSUB_U        = 'b10zz100101;
parameter COP2_VSRR           = 'b1000100110;
parameter COP2_VSRR_U         = 'b1000100111;
parameter COP2_VSLS           = 'b1000101000;
parameter COP2_VSLS_U         = 'b1000101001;
parameter COP2_VXUMUL         = 'b10z0101010;
parameter COP2_VXUMUL_U       = 'b10z0101011;
parameter COP2_VXLMUL         = 'b10z0101100;
parameter COP2_VXLMUL_U       = 'b10z0101101;
parameter COP2_VXUMADD        = 'b10z0101110;
parameter COP2_VXUMADD_U      = 'b10z0101111;
parameter COP2_VXUMSUB        = 'b10z0110000;
parameter COP2_VXUMSUB_U      = 'b10z0110001;
parameter COP2_VXLMADD        = 'b10z0110010;
parameter COP2_VXLMADD_U      = 'b10z0110011;
parameter COP2_VXLMSUB        = 'b10z0110100;
parameter COP2_VXLMSUB_U      = 'b10z0110101;
parameter COP2_VINS_VV        = 'b1100000000;
parameter COP2_VINS_SV        = 'b1110000001;
parameter COP2_VEXT_VV        = 'b1100000010;
parameter COP2_VEXT_SV        = 'b1100000011;
parameter COP2_VEXT_U_SV      = 'b1100000100;
parameter COP2_VCOMPRESS      = 'b1100000101;
parameter COP2_VEXPAND        = 'b1100000110;
parameter COP2_VMERGE         = 'b11zz000111;
parameter COP2_VFINS          = 'b1110001000;
parameter COP2_VEXTHALF       = 'b1100001001;
parameter COP2_VHALF          = 'b1100001010;
parameter COP2_VHALFUP        = 'b1100001011;
parameter COP2_VHALFDN        = 'b1100001100;
parameter COP2_VSATVL         = 'b1100001101;
parameter COP2_VFAND          = 'b11z0001110;
parameter COP2_VFOR           = 'b11z0001111;
parameter COP2_VFXOR          = 'b11z0010000;
parameter COP2_VFNOR          = 'b11z0010001;
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

input clk;
input resetn;

input [31:0] instr;
input instr_en;     // tells when instr is valid and available
output instr_wait;   // if high says vpu is not ready to receive

output has_memop;

// For mtc2/ctc2 instructions
input [31:0] scalar_in;    
input scalar_in_en;
output scalar_in_wait;

// For cfc2 instructions
output [31:0] scalar_out;
output scalar_out_en;
input scalar_out_wait;

// Data memory interface
output  [ 31 : 0 ]  dbus_address;
output              dbus_en;
output              dbus_we;
output  [ (DMEM_WRITEWIDTH/8)-1 : 0 ]   dbus_byteen;
output  [ DMEM_WRITEWIDTH-1 : 0 ]       dbus_writedata;
input   [ DMEM_READWIDTH-1 : 0 ]        dbus_readdata;
input         dbus_cachematch;
input         dbus_cachemiss;
output  [ 31 : 0 ]  dbus_prefetch;
input               dbus_wait;

//DMA interface
output [31:0]                 dma_dbus_address;   
input [DMEM_READWIDTH-1:0]    dma_dbus_readdata;  
output [DMEM_WRITEWIDTH-1:0]  dma_dbus_writedata; 
output [DMEM_WRITEWIDTH/8-1:0]dma_dbus_byteen;
output                        dma_dbus_en;        
output                        dma_dbus_wren;      
output                        dma_dbus_prefetch;  
input                         dma_dbus_wait;      
input                         dma_dbus_data_valid;

//AXI interface
output  [6 - 1:0]                    M_AWID;                                                 
output  [LANEWIDTH- 1:0]             M_AWADDR;                                    
output  [7:0]                        M_AWLEN;                                                    
output  [2:0]                        M_AWSIZE;                                                   
output  [1:0]                        M_AWBURST;                                                  
output                               M_AWLOCK;                                                         
output  [3:0]                        M_AWCACHE;                                                  
output  [2:0]                        M_AWPROT;                                                   
output  [3:0]                        M_AWQOS;                                                    
output                               M_AWVALID;                                                        
input                                M_AWREADY;                                                        
output  [NUMLANES*LANEWIDTH-1 : 0]   M_WDATA;                                     
output  [(NUMLANES*LANEWIDTH)/8-1 : 0] M_WSTRB;                                   
output                               M_WLAST;                                                          
output                               M_WVALID;                                                         
input                                M_WREADY;                                                         

output  [6-1 : 0]                    M_ARID;                                                 
output  [LANEWIDTH-1 : 0]            M_ARADDR;                                    
output  [7 : 0]                      M_ARLEN;                                                  
output  [2 : 0]                      M_ARSIZE;                                                 
output  [1 : 0]                      M_ARBURST;                                                
output                               M_ARLOCK;                                                         
output  [3 : 0]                      M_ARCACHE;                                                
output  [2 : 0]                      M_ARPROT;                                                 
output  [3 : 0]                      M_ARQOS;                                                  
output                               M_ARVALID;                                                        
input                                M_ARREADY;                                                        
input   [6-1 : 0]                    M_RID;                                         	    
input   [NUMLANES*LANEWIDTH-1 : 0]   M_RDATA;                                     
input   [1 : 0]                      M_RRESP;                                                  
input                                M_RLAST;                                                          
input                                M_RVALID;                                                         
output                               M_RREADY;                                                         

output                               M_BREADY;                                                         
input                                M_BVALID;                                                         
input   [1 : 0]                      M_BRESP;                                                  
input   [6-1 : 0]                    M_BID;                                                   

input  [6 - 1:0]                    S_AWID;                                                 
input  [LANEWIDTH- 1:0]             S_AWADDR;                                    
input  [7:0]                        S_AWLEN;                                                    
input  [2:0]                        S_AWSIZE;                                                   
input  [1:0]                        S_AWBURST;                                                  
input                               S_AWLOCK;                                                         
input  [3:0]                        S_AWCACHE;                                                  
input  [2:0]                        S_AWPROT;                                                   
input  [3:0]                        S_AWQOS;                                                    
input                               S_AWVALID;                                                        
output                                S_AWREADY;                                                        
input  [NUMLANES*LANEWIDTH-1 : 0]   S_WDATA;                                     
input  [(NUMLANES*LANEWIDTH)/8-1 : 0] S_WSTRB;                                   
input                               S_WLAST;                                                          
input                               S_WVALID;                                                         
output                                S_WREADY;                                                         

input  [6-1 : 0]                    S_ARID;                                                 
input  [LANEWIDTH-1 : 0]            S_ARADDR;                                    
input  [7 : 0]                      S_ARLEN;                                                  
input  [2 : 0]                      S_ARSIZE;                                                 
input  [1 : 0]                      S_ARBURST;                                                
input                               S_ARLOCK;                                                         
input  [3 : 0]                      S_ARCACHE;                                                
input  [2 : 0]                      S_ARPROT;                                                 
input  [3 : 0]                      S_ARQOS;                                                  
input                               S_ARVALID;                                                        
output                                S_ARREADY;                                                        
output   [6-1 : 0]                    S_RID;                                         	    
output   [NUMLANES*LANEWIDTH-1 : 0]   S_RDATA;                                     
output   [1 : 0]                      S_RRESP;                                                  
output                                S_RLAST;                                                          
output                                S_RVALID;                                                         
input                               S_RREADY;                                                         

input                               S_BREADY;                                                         
output                                S_BVALID;                                                         
output   [1 : 0]                      S_BRESP;                                                  
output   [6-1 : 0]                    S_BID;


reg                        [ 31 : 0 ]   ir;
wire                       [ 31 : 0 ]   ir2;
wire                       [ 31 : 0 ]   ir3;

wire                                    stall1;
wire                                    stall2;
wire                                    stall3;
wire                        [ 3 : 1 ]   internal_stall;
wire                                    squash1;
wire                                    squash2;
wire        [`MAX_PIPE_STAGES-1 : 0 ]   vlanes_stalled;

wire                        [ 5 : 0 ]   ir_cop2;
wire                        [ 9 : 0 ]   ir_op;
wire                        [ 5 : 0 ]   ir_vcr;
wire                        [ 4 : 0 ]   ir_vsr;
wire                        [ 4 : 0 ]   ir_src2;
wire                        [ 4 : 0 ]   ir_src1;
wire                        [ 5 : 0 ]   ir_vcr_r;

wire    [ LOG2NUMNONMEMVCREGS-1 : 0 ]   vc_a_reg;
wire                [ VCWIDTH-1 : 0 ]   vc_a_readdataout;
wire                                    vc_a_en;
wire    [ LOG2NUMNONMEMVCREGS-1 : 0 ]   vc_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vc_c_writedatain;
wire                                    vc_c_we;
wire                [ VCWIDTH-1 : 0 ]   vl;
wire                [ VCWIDTH-1 : 0 ]   dma_en;
wire                [ VCWIDTH-1 : 0 ]   mem_addr;
wire                [ VCWIDTH-1 : 0 ]   num_bytes;
wire                [ VCWIDTH-1 : 0 ]   lane_addr;
wire                [ VCWIDTH-1 : 0 ]   dma_we;
//Masks for the matmul (3 masks. each is 8-bit.)
//1-bit for each row/column element in the matmul. we have an 8x8 matmul)
wire         [3*`MAT_MUL_SIZE-1 : 0 ]   matmul_masks;

wire       [ LOG2NUMVBASEREGS-1 : 0 ]   vbase_a_reg;
wire                [ VCWIDTH-1 : 0 ]   vbase_a_readdataout;
wire                                    vbase_a_en;
wire       [ LOG2NUMVBASEREGS-1 : 0 ]   vbase_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vbase_c_writedatain;
wire                                    vbase_c_we;

wire        [ LOG2NUMVINCREGS-1 : 0 ]   vinc_a_reg;
wire                                    vinc_a_en;
wire                [ VCWIDTH-1 : 0 ]   vinc_a_readdataout;
wire        [ LOG2NUMVINCREGS-1 : 0 ]   vinc_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vinc_c_writedatain;
wire                                    vinc_c_we;

wire     [ LOG2NUMVSTRIDEREGS-1 : 0 ]   vstride_a_reg;
wire                                    vstride_a_en;
wire                [ VCWIDTH-1 : 0 ]   vstride_a_readdataout;
wire     [ LOG2NUMVSTRIDEREGS-1 : 0 ]   vstride_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vstride_c_writedatain;
wire                                    vstride_c_we;

wire          [ LOG2NUMVSREGS-1 : 0 ]   vs_a_reg;
wire                [ VCWIDTH-1 : 0 ]   vs_a_readdataout;
wire                                    vs_a_en;
wire          [ LOG2NUMVSREGS-1 : 0 ]   vs_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vs_c_writedatain;
wire                                    vs_c_we;

wire                [ VCWIDTH-1 : 0 ]   vc_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vl_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vbase_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vinc_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vstride_readdataout;
wire                [ VSWIDTH-1 : 0 ]   vs_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vc_combined_out;
wire                [ VCWIDTH-1 : 0 ]   vc_combined_out_s3;
wire                [ VCWIDTH-1 : 0 ]   _vc_c_writedatain;
wire                [ VCWIDTH-1 : 0 ]   fwddata;
wire                                    fwd_vc;
wire                                    fwd_vl;
wire                                    fwd_vbase;
wire                                    fwd_vinc;
wire                                    fwd_vstride;

reg                                     ctrl_vc_a_en;
reg                                     ctrl_vl_a_en;
reg                                     ctrl_vbase_a_en;
reg                                     ctrl_vinc_a_en;
reg                                     ctrl_vstride_a_en;
reg                                     ctrl_vs_a_en;

wire          [ LOG2NUMVCREGS-1 : 0 ]   vc_rd_reg;
wire          [ LOG2NUMVCREGS-1 : 0 ]   vbase_rd_reg;
wire          [ LOG2NUMVCREGS-1 : 0 ]   vinc_rd_reg;
wire          [ LOG2NUMVCREGS-1 : 0 ]   vstride_rd_reg;
wire          [ LOG2NUMVSREGS-1 : 0 ]   vs_rd_reg;

wire          [ LOG2NUMVSREGS-1 : 0 ]   vlanes_vs_dst;
wire                                    vlanes_vs_we;
wire                        [ 5 : 2 ]   vlanes_vs_wetrack;
wire                [ VSWIDTH-1 : 0 ]   vlanes_vs_writedata;

wire                [ VCWIDTH-1 : 0 ]   vbase_plus_vinc;

wire [LOG2NUMVCREGS-1:0] ir_base;
wire [LOG2NUMVCREGS-1:0] ir_inc;
wire [LOG2NUMVCREGS-1:0] ir_stride;
wire [LOG2NUMVCREGS-1:0] vcdest_s1;
wire [LOG2NUMVCREGS-1:0] vcdest_s2;
wire [LOG2NUMVCREGS-1:0] vsdest_s1;
wire [LOG2NUMVCREGS-1:0] vsdest_s2;
wire       wevalid;
wire       wevalid_s2;
wire       haz_vc;
wire       haz_vl;
wire       haz_vbase;
wire       haz_vinc;
wire       haz_vstride;
wire       haz_vs_RAW;
wire       haz_vs_WAW;
wire dma_busy;
reg cfc_satisfied;

reg ctrl__vc_writedatain_sel;          //1 - vmstc, 0 - mtc;
reg [1:0] ctrl_vc_writedatain_sel;  //0 - vmstc/mtc, 1-vhalf, 2/3-vsatvl
reg ctrl_vc_we;
reg ctrl_vbase_writedatain_sel;    //1 - ctc/mstc, 0 - vld/vst
reg ctrl_scalarin_en;   //1 when ctc2/mtc2
reg ctrl_scalar_out_en;  //1 when cfc2
reg [1:0] ctrl_vcdest_sel;  //0 - ctc/mstc, 1- ld/st, 2 - satvl/half
reg ctrl_vsdest_sel;  //1 - mtc2, 0 - vmcts
reg ctrl_vs_we;
reg ctrl_rdctl_sel;        // 1 - cfc2/vmcts  0 - ld/st,
reg [1:0] ctrl_rdvc_sel;   // 2-vshamt, 3-vindex, other-ir_vcr


wire ctrl_scalarin_en_s2;
wire ctrl_scalar_out_en_s2;
wire      scalar_out_en_s3;
wire ctrl_vc_we_s2;
wire ctrl_vbase_writedatain_sel_s2;
wire ctrl__vc_writedatain_sel_s2;
wire [1:0] ctrl_vc_writedatain_sel_s2;
wire ctrl_vsdest_sel_s2;
wire ctrl_vs_we_s2;

/************************ Instruction Register ******************************/
wire is_cop2;
reg is_cop2_s1;

  assign is_cop2=instr[31:26]==6'b010010;

  always @(posedge clk)
    if (!resetn || (~is_cop2&~stall1) || (~instr_en&~stall1))
      ir<=32'h0;    // NOP  (Used to use VMSTC $vc48,$vs0)
    else if (instr_en&~stall1)
      ir<=instr;

  always @(posedge clk)
    if (!resetn)
      is_cop2_s1<=1'b0;
    else if (instr_en)
      is_cop2_s1<=is_cop2;

  assign instr_wait = stall1 & is_cop2;

/******************************************************************************/
/************************** 1st Pipeline Stage ********************************/
/******************************************************************************/

//Flag instructions which don't use lanes so they don't stall when lanes stalled
  reg ctrl_doesnt_use_lanes;
  always@*
  begin
    ctrl_doesnt_use_lanes=0;
    casez(ir_op)
      0,
      COP2_VSATVL,
      //COP2_VMCTS:  //Omit since vlanes modifies scalar
      COP2_VMSTC,
      COP2_CFC2,
      COP2_CTC2:
        ctrl_doesnt_use_lanes=1;
    endcase
  end

  assign internal_stall[1]=internal_stall[2] | haz_vs_RAW;
  assign stall1=internal_stall[1] | (vlanes_stalled[1]&&~ctrl_doesnt_use_lanes);
  assign squash1 = (stall1&~stall2)|~resetn;

  pipereg IR_reg2(ir,clk,resetn,~stall1,~squash1,ir2);

  assign ir_cop2=ir[31:26];
  assign ir_op={ir[25:22],ir[5:0]}; //10 bits
  assign ir_vcr=ir[15:10];
  assign ir_vsr=ir[15:11];
  assign ir_src1=ir[15:11];
  assign ir_src2=ir[20:16];
  assign ir_base={2'b10,ir[15:12]};
  assign ir_inc={3'b110,ir[11:9]};
  assign ir_stride={3'b111,ir[8:6]};

  assign wevalid = (ir_cop2==6'b010010) & ctrl_vc_we & (vcdest_s1!=48);

  pipereg #(1) wevalid_reg1(wevalid,clk,~squash1, ~stall1,1'b1, wevalid_s2);

  assign vcdest_s1 = (ctrl_vcdest_sel==0) ? ir_vcr :
                    (ctrl_vcdest_sel==1) ? ir_base : 0;

  pipereg #(LOG2NUMVCREGS) vcdest_reg1(vcdest_s1,clk,~squash1, ~stall1,1'b1,
                      vcdest_s2);

  assign vsdest_s1 = (ctrl_vsdest_sel) ? ir_vsr : ir_src2;

  pipereg #(LOG2NUMVSREGS) vsdest_reg1(vsdest_s1,clk,~squash1, ~stall1,1'b1,
                      vsdest_s2);

  pipereg #(LOG2NUMVSREGS) vsdestsel_reg1
        (ctrl_vsdest_sel,clk,~squash1, ~stall1,1'b1, ctrl_vsdest_sel_s2);

  // Before reading all source operands, we need to:
  //  1. Determine what all the sources are
  always@*
  begin
    ctrl_vc_a_en=0;
    ctrl_vl_a_en=0;
    ctrl_vbase_a_en=0;
    ctrl_vinc_a_en=0;
    ctrl_vstride_a_en=0;
    ctrl_vs_a_en=0;
    casez(ir_op)
      COP2_VADD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VADD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VSUB:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSUB_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VMULHI:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMULHI_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VDIV:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VDIV_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      //COP2_VMOD:
      COP2_VBFADD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VMOD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_EQ:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_NE:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_LT:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_U_LT:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_LE:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_U_LE:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VMIN:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMIN_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMAX:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMAX_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMULLO:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VABS:
          ctrl_vl_a_en=1;
      COP2_VAND:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VNOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VSLL:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSRL:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSRA:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSAT_B:
          ctrl_vl_a_en=1;
      COP2_VSAT_H:
          ctrl_vl_a_en=1;
      //COP2_VSAT_W:
      COP2_VACT:
          ctrl_vl_a_en=1;
      COP2_VSAT_SU_B:
          ctrl_vl_a_en=1;
      COP2_VSAT_SU_H:
          ctrl_vl_a_en=1;
      //COP2_VSAT_SU_W:
      COP2_VRED:
          ctrl_vl_a_en=1;
      COP2_VSAT_SU_L:
          ctrl_vl_a_en=1;
      COP2_VSAT_U_B:
          ctrl_vl_a_en=1;
      COP2_VSAT_U_H:
          ctrl_vl_a_en=1;
      //COP2_VSAT_U_W:
      COP2_VTRP:
          ctrl_vl_a_en=1;
      COP2_VSADD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VSADD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VSSUB:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSSUB_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSRR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VSRR_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VSLS:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VSLS_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VXUMUL:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMUL_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMUL:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMUL_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMADD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMADD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMSUB:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMSUB_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMADD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMADD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMSUB:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMSUB_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VINS_VV:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VINS_SV:
      begin
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VEXT_VV:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VEXT_SV:
          ctrl_vc_a_en=1;
      COP2_VEXT_U_SV:
          ctrl_vc_a_en=1;
      COP2_VCOMPRESS:
          ctrl_vl_a_en=1;
      COP2_VEXPAND:
          ctrl_vl_a_en=1;
      COP2_VMERGE:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VFINS:
        begin
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=1;
        end
      COP2_VEXTHALF:
          ctrl_vl_a_en=1;
      COP2_VHALF:
          ctrl_vl_a_en=1;
      COP2_VHALFUP:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VHALFDN:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VSATVL:
          ctrl_vl_a_en=1;
      COP2_VFAND:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VFOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VFXOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VFNOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VFCLR:
          ctrl_vl_a_en=1;
      COP2_VFSET:
          ctrl_vl_a_en=1;
      COP2_VIOTA:
          ctrl_vl_a_en=1;
      COP2_VCIOTA:
          ctrl_vl_a_en=1;
      COP2_VFPOP:
          ctrl_vl_a_en=1;
      COP2_VFFF1:
          ctrl_vl_a_en=1;
      COP2_VFFL1:
          ctrl_vl_a_en=1;
      COP2_VFSETBF:
          ctrl_vl_a_en=1;
      COP2_VFSETIF:
          ctrl_vl_a_en=1;
      COP2_VFSETOF:
          ctrl_vl_a_en=1;
      COP2_VFMT8:
          ctrl_vl_a_en=1;
      COP2_VFMF8:
          ctrl_vl_a_en=1;
      COP2_VFCLR8:
          ctrl_vl_a_en=1;
      COP2_VFOR8:
          ctrl_vl_a_en=1;
      COP2_VFLD:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
     // COP2_VLD_W:
     //   begin
     //     ctrl_vbase_a_en=1;
     //     ctrl_vinc_a_en=1;
     //   end
      COP2_VLD_L:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_U_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_U_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      //COP2_VLD_U_W:
      COP2_VAXIRD:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLDS_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VLDS_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      //COP2_VLDS_W:
      //  begin
      //    ctrl_vbase_a_en=1;
      //    ctrl_vinc_a_en=1;
      //    ctrl_vstride_a_en=1;
      //  end
      COP2_VLDS_L:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VLDS_U_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VLDS_U_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      //COP2_VLDS_U_W:
      //  begin
      //    ctrl_vbase_a_en=1;
      //    ctrl_vinc_a_en=1;
      //    ctrl_vstride_a_en=1;
      //  end
      COP2_VLDX_B:
          ctrl_vbase_a_en=1;
      COP2_VLDX_H:
          ctrl_vbase_a_en=1;
      //COP2_VLDX_W:
      //    ctrl_vbase_a_en=1;
      COP2_VLDX_L:
          ctrl_vbase_a_en=1;
      COP2_VLDX_U_B:
          ctrl_vbase_a_en=1;
      COP2_VLDX_U_H:
          ctrl_vbase_a_en=1;
      //COP2_VLDX_U_W:
      //    ctrl_vbase_a_en=1;
      COP2_VFST:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VST_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VST_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      //COP2_VST_W:
      //  begin
      //    ctrl_vbase_a_en=1;
      //    ctrl_vinc_a_en=1;
      //  end
      COP2_VST_L:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VSTS_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VSTS_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      //COP2_VSTS_W:
      //  begin
      //    ctrl_vbase_a_en=1;
      //    ctrl_vinc_a_en=1;
      //    ctrl_vstride_a_en=1;
      //  end
      COP2_VSTS_L:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VSTX_B:
          ctrl_vbase_a_en=1;
      COP2_VSTX_H:
          ctrl_vbase_a_en=1;
     //COP2_VSTX_W:
      COP2_VPER_STR:
          ctrl_vl_a_en=1;
      COP2_VPER:
          ctrl_vl_a_en=1;
      COP2_VPER_LD:
          ctrl_vl_a_en=1;
      COP2_VSTX_L:
          ctrl_vbase_a_en=1;
      COP2_VSTXO_B:
          ctrl_vbase_a_en=1;
      COP2_VSTXO_H:
          ctrl_vbase_a_en=1;
      //COP2_VSTXO_W:
      COP2_VAXIWR:
          ctrl_vbase_a_en=1;
      COP2_VSTXO_L:
          ctrl_vbase_a_en=1;
      COP2_VMCTS:
        begin
          ctrl_vc_a_en=1;
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VMSTC:
          ctrl_vs_a_en=1;
      COP2_CFC2:
        begin
          ctrl_vc_a_en=1;
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
    endcase
  end

  //  2. Determine if any of the sources have a RAW hazard
  assign haz_vc=(ctrl_vc_a_en&wevalid_s2)&&(vc_rd_reg==vcdest_s2);
  assign haz_vl=(ctrl_vl_a_en&wevalid_s2)&&(vcdest_s2==0);
  assign haz_vbase=(ctrl_vbase_a_en&wevalid_s2)&&(vbase_rd_reg==vcdest_s2);
  assign haz_vinc=(ctrl_vinc_a_en&wevalid_s2)&&(vinc_rd_reg==vcdest_s2)&&(vcdest_s2!=48);
  assign haz_vstride=(ctrl_vstride_a_en&wevalid_s2)&(vstride_rd_reg==vcdest_s2);

  // Coarse-grained checks for VS - don't check registers
  assign haz_vs_RAW=ctrl_vs_a_en && (ctrl_vs_we_s2 || (|vlanes_vs_wetrack));
  assign haz_vs_WAW=ctrl_vs_we_s2 && (|vlanes_vs_wetrack[5:3]);

  pipereg #(VCWIDTH) fwddata_reg (
       (ctrl_vbase_writedatain_sel_s2) ?  vbase_plus_vinc : vc_c_writedatain,
       clk,~squash1, ~stall1,1'b1,fwddata);

  pipereg #(1) fwdvc_reg ( haz_vc,clk,~squash1, ~stall1,1'b1,fwd_vc);
  pipereg #(1) fwdvl_reg ( haz_vl,clk,~squash1, ~stall1,1'b1,fwd_vl);
  pipereg #(1) fwdvbase_reg ( haz_vbase,clk,~squash1, ~stall1,1'b1,fwd_vbase);
  pipereg #(1) fwdvinc_reg ( haz_vinc,clk,~squash1, ~stall1,1'b1,fwd_vinc);
  pipereg #(1) fwdvstride_reg(haz_vstride,clk,~squash1,~stall1,1,fwd_vstride);

  /************************ REGISTER FILES ******************************/

  assign vc_rd_reg= (ctrl_rdvc_sel==2) ? 2 :  //vshamt
                    (ctrl_rdvc_sel==3) ? 3 :  //vindex
                    ir_vcr;
  assign vbase_rd_reg= (ctrl_rdctl_sel) ?  ir_vcr : ir_base;
  assign vinc_rd_reg= (ctrl_rdctl_sel) ?  ir_vcr : ir_inc;
  assign vstride_rd_reg= (ctrl_rdctl_sel) ?  ir_vcr : ir_stride;
  assign vs_rd_reg= (ctrl_vs_a_en & ir_op[BIT_VSSRC1]) ? 
                        ir_src1[LOG2NUMVSREGS-1:0] : 
                        ir_src2[LOG2NUMVSREGS-1:0];

  assign vc_a_reg= vc_rd_reg[LOG2NUMNONMEMVCREGS-1:0];
  assign vbase_a_reg= vbase_rd_reg[LOG2NUMVBASEREGS-1:0];
  assign vinc_a_reg=vinc_rd_reg[LOG2NUMVINCREGS-1:0];
  assign vstride_a_reg=vstride_rd_reg[LOG2NUMVSTRIDEREGS-1:0];

  assign vs_a_reg=vs_rd_reg[LOG2NUMVSREGS-1:0];

  assign vc_a_en=ctrl_vc_a_en&~stall1;
  assign vbase_a_en=ctrl_vbase_a_en&~stall1;
  assign vinc_a_en=ctrl_vinc_a_en&~stall1;
  assign vstride_a_en=ctrl_vstride_a_en&~stall1;
  assign vs_a_en=ctrl_vs_a_en&~stall1;

  wire [VCWIDTH-1:0] temp;
  vregfile_control vregfile_control (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vc_a_reg), 
      .a_en(vc_a_en),
      .a_readdataout(vc_a_readdataout),
      .c_reg(vc_c_reg), 
      .c_writedatain(vc_c_writedatain), 
      .c_we(vc_c_we),
      .vl(vl),
      //The reserved registers vc31, vc30, vc29 are used
      //for the matmul's masks.
      .matmul_masks(matmul_masks),
      .dma_en(dma_en),
      .lane_addr(lane_addr),
      .mem_addr(mem_addr),
      .num_bytes(num_bytes),
      .dma_we(dma_we),
      .dma_busy(dma_busy),
      .temp(temp)
      );
    defparam vregfile_control.WIDTH=VCWIDTH;
    defparam vregfile_control.NUMREGS=NUMNONMEMVCREGS;
    defparam vregfile_control.LOG2NUMREGS=LOG2NUMNONMEMVCREGS;

  vregfile_base vregfile_base (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vbase_a_reg), 
      .a_en(vbase_a_en), 
      .a_readdataout(vbase_a_readdataout),
      .c_reg(vbase_c_reg), 
      .c_writedatain(vbase_c_writedatain), 
      .c_we(vbase_c_we));
    defparam vregfile_base.WIDTH=VCWIDTH;
    defparam vregfile_base.NUMREGS=NUMVBASEREGS;
    defparam vregfile_base.LOG2NUMREGS=LOG2NUMVBASEREGS;

  vregfile_inc vregfile_inc (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vinc_a_reg), 
      .a_en(vinc_a_en), 
      .a_readdataout(vinc_a_readdataout),
      .c_reg(vinc_c_reg), 
      .c_writedatain(vinc_c_writedatain), 
      .c_we(vinc_c_we));
    defparam vregfile_inc.WIDTH=VCWIDTH;
    defparam vregfile_inc.NUMREGS=NUMVINCREGS;
    defparam vregfile_inc.LOG2NUMREGS=LOG2NUMVINCREGS;

  vregfile_stride vregfile_stride (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vstride_a_reg), 
      .a_en(vstride_a_en), 
      .a_readdataout(vstride_a_readdataout),
      .c_reg(vstride_c_reg), 
      .c_writedatain(vstride_c_writedatain), 
      .c_we(vstride_c_we));
    defparam vregfile_stride.WIDTH=VCWIDTH;
    defparam vregfile_stride.NUMREGS=NUMVSTRIDEREGS;
    defparam vregfile_stride.LOG2NUMREGS=LOG2NUMVSTRIDEREGS;

  vregfile_scalar vregfile_scalar (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vs_a_reg), 
      .a_en(vs_a_en), 
      .a_readdataout(vs_a_readdataout),
      .c_reg(vs_c_reg), 
      .c_writedatain(vs_c_writedatain), 
      .c_we(vs_c_we));
    defparam vregfile_scalar.WIDTH=VSWIDTH;
    defparam vregfile_scalar.NUMREGS=NUMVSREGS;
    defparam vregfile_scalar.LOG2NUMREGS=LOG2NUMVSREGS;

  pipereg #(6) vc_reg(ir_vcr,clk,resetn,vc_a_en,1'b1,ir_vcr_r);

  pipereg #(1) r1(ctrl_scalarin_en,clk,~squash1,~stall1,1'b1,ctrl_scalarin_en_s2);
  pipereg #(1)r2(ctrl_scalar_out_en,clk,~squash1,~stall1,1'b1,ctrl_scalar_out_en_s2);
  pipereg #(1) r4(ctrl_vc_we,clk,~squash1,~stall1,1'b1,ctrl_vc_we_s2);
  pipereg #(1) r5(ctrl_vbase_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl_vbase_writedatain_sel_s2);
  pipereg #(1) r6(ctrl__vc_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl__vc_writedatain_sel_s2);
  pipereg #(2) r7(ctrl_vc_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl_vc_writedatain_sel_s2);
  pipereg #(1) r8(ctrl_vs_we,clk,~squash1,~stall1,1'b1,ctrl_vs_we_s2);


  /*********************** Control control signals ****************************/
  always@*
  begin
    ctrl_scalarin_en=0;
    ctrl_scalar_out_en=0;
    ctrl__vc_writedatain_sel=0;
    ctrl_vc_writedatain_sel=0;
    ctrl_vbase_writedatain_sel=0;
    ctrl_vc_we=0;
    ctrl_rdctl_sel=0;
    ctrl_rdvc_sel=0;
    ctrl_vcdest_sel=0;
    ctrl_vsdest_sel=0;
    ctrl_vs_we=0;
    casez(ir_op)
      COP2_VSRR:  ctrl_rdvc_sel=2;
      COP2_VSRR_U:  ctrl_rdvc_sel=2;
      COP2_VSLS:  ctrl_rdvc_sel=2;
      COP2_VSLS_U:  ctrl_rdvc_sel=2;
      COP2_VXUMUL:  ctrl_rdvc_sel=2;
      COP2_VXUMUL_U:  ctrl_rdvc_sel=2;
      COP2_VXLMUL:  ctrl_rdvc_sel=2;
      COP2_VXLMUL_U:  ctrl_rdvc_sel=2;
      COP2_VXUMADD:  ctrl_rdvc_sel=2;
      COP2_VXUMADD_U:  ctrl_rdvc_sel=2;
      COP2_VXUMSUB:  ctrl_rdvc_sel=2;
      COP2_VXUMSUB_U:  ctrl_rdvc_sel=2;
      COP2_VXLMADD:  ctrl_rdvc_sel=2;
      COP2_VXLMADD_U:  ctrl_rdvc_sel=2;
      COP2_VXLMSUB:  ctrl_rdvc_sel=2;
      COP2_VXLMSUB_U:  ctrl_rdvc_sel=2;
      COP2_VINS_VV:  ctrl_rdvc_sel=2;
      COP2_VINS_VV:  ctrl_rdvc_sel=3;
      COP2_VINS_SV:  ctrl_rdvc_sel=3;
      COP2_VEXT_VV:  ctrl_rdvc_sel=3;
      COP2_VEXT_SV:  ctrl_rdvc_sel=3;
      COP2_VEXT_U_SV:  ctrl_rdvc_sel=3;
      COP2_VFINS:  ctrl_rdvc_sel=3;
      COP2_VHALFUP:  ctrl_rdvc_sel=3;
      COP2_VHALFDN:  ctrl_rdvc_sel=3;
      COP2_VHALF:
      begin
          ctrl_vc_writedatain_sel=1;
          ctrl_vcdest_sel=2;
          ctrl_vc_we=1;
      end
      COP2_VSATVL:
      begin
          ctrl_vc_writedatain_sel=2;
          ctrl_vcdest_sel=2;
          ctrl_vc_we=1;
      end
      COP2_VFLD,
      COP2_VLD_B,
      COP2_VLD_H,
     // COP2_VLD_W,
      COP2_VLD_L,
      COP2_VLD_U_B,
      COP2_VLD_U_H,
     // COP2_VLD_U_W,
      COP2_VAXIRD,
      COP2_VLDS_B,
      COP2_VLDS_H,
    //  COP2_VLDS_W,
      COP2_VLDS_L,
      COP2_VLDS_U_B,
      COP2_VLDS_U_H,
    //  COP2_VLDS_U_W,
      COP2_VFST,
      COP2_VST_B,
      COP2_VST_H,
    //  COP2_VST_W,
      COP2_VST_L,
      COP2_VSTS_B,
      COP2_VSTS_H,
    //  COP2_VSTS_W,
      COP2_VSTS_L:
      begin
          ctrl_vbase_writedatain_sel=1;
          ctrl_vcdest_sel=1;
          ctrl_vc_we=1;
      end
      COP2_VMCTS:
      begin
        ctrl_rdctl_sel=1;
        ctrl_vsdest_sel=0;
        ctrl_vs_we=1;
      end
      COP2_VMSTC:
      begin
        ctrl__vc_writedatain_sel=1;
        ctrl_vcdest_sel=0;
        ctrl_vc_we=1;
      end
      COP2_CFC2:
      begin
        ctrl_scalar_out_en=1;
        ctrl_rdctl_sel=1;
      end
      COP2_CTC2:
      begin
        ctrl_scalarin_en=1;
        ctrl_vcdest_sel=0;
        ctrl_vc_we=1;
      end
      COP2_MTC2:
      begin
        ctrl_scalarin_en=1;
        ctrl_vsdest_sel=1;
        ctrl_vs_we=1;
      end
    endcase
  end

/******************************************************************************/
/************************** 2nd Pipeline Stage ********************************/
/******************************************************************************/

  assign internal_stall[2]=internal_stall[3] |
                              (ctrl_scalarin_en_s2&~scalar_in_en) | 
                              haz_vs_WAW;
  assign stall2=internal_stall[2]; // | vlanes_stalled[2];
  assign squash2 = ((stall2&~stall3))|~resetn;

  // Stall scalar if a) we are stalled, are expecting a scalar, and the it is
  // available OR b) if we haven't gotten there yet.
  assign scalar_in_wait=(scalar_in_en&~ctrl_scalarin_en_s2) || 
                        (ctrl_scalarin_en_s2&scalar_in_en&stall2);

  assign vc_readdataout= (fwd_vc) ? fwddata : vc_a_readdataout;
  assign vl_readdataout= (fwd_vl) ? fwddata : vl;
  assign vbase_readdataout= (fwd_vbase) ? fwddata : vbase_a_readdataout;
  assign vinc_readdataout= (fwd_vinc) ? fwddata : vinc_a_readdataout;
  assign vstride_readdataout= (fwd_vstride) ? fwddata : vstride_a_readdataout;
  assign vs_readdataout= vs_a_readdataout;

  assign vc_combined_out=(!ir_vcr_r[5]) ? vc_readdataout :
                         (!ir_vcr_r[4]) ? vbase_readdataout :
                         (!ir_vcr_r[3]) ? vinc_readdataout : 
                         vstride_readdataout;

  pipereg #(VCWIDTH) scalar_out_reg(
      vc_combined_out,clk,resetn,ctrl_scalar_out_en_s2,1'b1,vc_combined_out_s3);

  pipereg #(1) scalar_out_en_reg(
      ctrl_scalar_out_en_s2,clk,~squash2,~stall2,1'b1,scalar_out_en_s3);


  /************** Datapath - Control Regs *******************/

  assign vc_c_we = ctrl_vc_we_s2&~vcdest_s2[5] & ~stall2;
  assign vc_c_reg =  vcdest_s2[LOG2NUMNONMEMVCREGS-1:0]; 
  //temporary to feed to base,inc, and stride regfiles (without vl logic)
  assign _vc_c_writedatain= (ctrl__vc_writedatain_sel_s2) ? vs_a_readdataout :
                                                        scalar_in;
  assign vc_c_writedatain= ( (ctrl_vc_writedatain_sel_s2==0) ? _vc_c_writedatain :
                            //vhalf instruction
                            (ctrl_vc_writedatain_sel_s2==1) ? vl_readdataout>>1:
                            //vsatvl instruction
                            //(vl_readdataout>vc_readdataout) ? vc_readdataout :
                            (vl_readdataout>MVL) ? MVL : 
                            vl_readdataout);

  assign vbase_c_we = ctrl_vc_we_s2 & (vcdest_s2[5:4]==2) & ~stall2;
  assign vbase_c_reg = vcdest_s2[LOG2NUMVBASEREGS-1:0];
  assign vbase_c_writedatain = (ctrl_vbase_writedatain_sel_s2) ?
                                vbase_plus_vinc :
                                _vc_c_writedatain;

  assign vbase_plus_vinc=vbase_readdataout+vinc_readdataout;

  assign vinc_c_we = ctrl_vc_we_s2 & (vcdest_s2[5:3]==6) & ~stall2;
  assign vinc_c_reg = vcdest_s2[LOG2NUMVINCREGS-1:0];
  assign vinc_c_writedatain = _vc_c_writedatain;

  assign vstride_c_we = ctrl_vc_we_s2 & (vcdest_s2[5:3]==7) & ~stall2;
  assign vstride_c_reg = vcdest_s2[LOG2NUMVSTRIDEREGS-1:0];
  assign vstride_c_writedatain = _vc_c_writedatain;

  //FIXME - OR vector writes in, but check for RAW & WAW hazards
  assign vs_c_we = vlanes_vs_we || (ctrl_vs_we_s2 & ~stall2);
  assign vs_c_reg = (vlanes_vs_we) ? vlanes_vs_dst : vsdest_s2;
  assign vs_c_writedatain = (vlanes_vs_we) ? vlanes_vs_writedata : 
                    (ctrl_vsdest_sel_s2) ? scalar_in : vc_combined_out;


/******************************************************************************/
/************************** 3rd Pipeline Stage ********************************/
/******************************************************************************/

  assign internal_stall[3]=scalar_out_en_s3&scalar_out_wait;
  assign stall3=internal_stall[3]; // | vlanes_stalled[3];

  assign scalar_out=vc_combined_out_s3;
  assign scalar_out_en=scalar_out_en_s3&~cfc_satisfied;

  always@(posedge clk)
    cfc_satisfied<=scalar_out_en_s3 & ~scalar_out_wait;

/******************************************************************************/
/************************ Instantiate Vector Lanes ****************************/
/******************************************************************************/

  vlanes vlanes(
    .clk(clk),
    .resetn(resetn),

    // Instruction interface
    .instr(ir),
    .instr_en(is_cop2_s1),    // tells when instr is valid and available
    .instr_wait(),            // if high says vpu is not ready to receive

    .stall_in({internal_stall,1'b0}),
    .is_stalled(vlanes_stalled),
    .has_memop(has_memop),

    // Control register values - 2nd stage
    .vc_in(vc_readdataout),
    .vl_in(vl_readdataout),
    .vbase_in(vbase_readdataout),
    .vinc_in(vinc_readdataout),
    .vstride_in(vstride_readdataout),
    .vs_in(vs_readdataout),
    .matmul_masks_in(matmul_masks),
    .dma_en(dma_en),
    .mem_addr(mem_addr),
    .num_bytes(num_bytes),
    .lane_addr(lane_addr),
    .dma_we(dma_we),
    .dma_busy(dma_busy),
    // vs Writeback
    .vs_dst(vlanes_vs_dst),
    .vs_wetrack(vlanes_vs_wetrack),  //1-bit for each pipe-stage
    .vs_we(vlanes_vs_we),
    .vs_writedata(vlanes_vs_writedata),
    
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
    .dbus_address(dbus_address),
    .dbus_en(dbus_en),
    .dbus_we(dbus_we),
    .dbus_byteen(dbus_byteen),
    .dbus_writedata(dbus_writedata),
    .dbus_readdata(dbus_readdata),
    .dbus_cachematch(dbus_cachematch),
    .dbus_cachemiss(dbus_cachemiss),
    .dbus_prefetch(dbus_prefetch),
    .dbus_wait(dbus_wait),

    //DMA interface
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
  defparam 
    vlanes.NUMLANES=NUMLANES,
    vlanes.LOG2NUMLANES=LOG2NUMLANES,
    vlanes.NUMMEMPARALLELLANES=NUMMEMLANES,
    vlanes.LOG2NUMMEMPARALLELLANES=LOG2NUMMEMLANES,
    vlanes.NUMMULLANES=NUMMULLANES,
    vlanes.MVL=MVL,
    vlanes.LOG2MVL=LOG2MVL,
    vlanes.VPW=VPW,
    vlanes.LOG2VPW=LOG2VPW,
    vlanes.LANEWIDTH=LANEWIDTH,
    vlanes.LOG2LANEWIDTH=LOG2LANEWIDTH,
    vlanes.NUMBANKS=NUMBANKS,
    vlanes.LOG2NUMBANKS=LOG2NUMBANKS,
    vlanes.ALUPERBANK=ALUPERBANK,
    vlanes.DMEM_WRITEWIDTH=DMEM_WRITEWIDTH, 
    vlanes.LOG2DMEM_WRITEWIDTH=LOG2DMEM_WRITEWIDTH,
    vlanes.DMEM_READWIDTH=DMEM_READWIDTH,
    vlanes.LOG2DMEM_READWIDTH=LOG2DMEM_READWIDTH,
    vlanes.VCWIDTH=VCWIDTH,
    vlanes.VSWIDTH=VSWIDTH,
    vlanes.NUMVSREGS=NUMVSREGS,
    vlanes.LOG2NUMVSREGS=LOG2NUMVSREGS;

endmodule

//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/local/spram1.v
//////////////////////////////////////////////////////////////////////
module spram1 (
    clk,
    address,
    wren, 
    data,
    byteen,
    out
);

parameter AWIDTH=10;
parameter NUM_WORDS=1024;
parameter DWIDTH=32;
parameter LOG2DWIDTH = $clog2(DWIDTH);
input clk;
input [(AWIDTH-1):0] address;
input  wren;
input [(DWIDTH/8)-1:0] byteen;
input [(DWIDTH-1):0] data;
output reg [(DWIDTH-1):0] out;

//`ifdef SIMULATION_MEMORY

integer i;
integer k;

reg [(((67108864-1)-(0)+1)*((32-1)-(0)+1))-1 : 0] ram;
//reg [32-1:0] ram[4096-1:0];
reg [25:0] addr;
 
initial
 begin
   //This is TOO big for 256 MB RAM!  We right shift data by 1
       $readmemh("instr.dat",ram,'h100_0000);
       $readmemh("data.dat",ram,'h400_0000>>1);
 end

always@(*) begin
    addr = address << 26-AWIDTH;
end

always@(posedge clk) begin 
  if (wren) begin
      for(k=0; k < DWIDTH/32;k=k+1)begin
          for(i=0; i < 4 ;i=i+1)begin
              if(byteen[((DWIDTH/8-1)-(4*k+i))])
                  ram[addr+k][i*8+:8] <= data[32*k+i*8+:8];
          end
      end
  end
  else begin
      for(i=0; i < DWIDTH/32; i=i+1)begin
          out[32*i+:32] <= ram[addr+i];
      end
  end
end
//`else

/*
//single_port_ram u_single_port_ram(
.addr(address),
.we(wren),
.data(data),
.out(out),
.clk(clk)
);

`endif
*/
endmodule
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/local/rams.v
//////////////////////////////////////////////////////////////////////
module dpram (
	clk,
	address_a,
	address_b,
	wren_a,
	wren_b,
	data_a,
	data_b,
	out_a,
	out_b
);

parameter AWIDTH=10;
parameter NUM_WORDS=1024;
parameter DWIDTH=32;

input clk;
input [(AWIDTH-1):0] address_a;
input [(AWIDTH-1):0] address_b;
input  wren_a;
input  wren_b;
input [(DWIDTH-1):0] data_a;
input [(DWIDTH-1):0] data_b;
output reg [(DWIDTH-1):0] out_a;
output reg [(DWIDTH-1):0] out_b;

`ifndef SIMULATION_MEMORY
 `define SIMULATION_MEMORY
`endif

`ifdef SIMULATION_MEMORY

reg [(((NUM_WORDS-1)-(0)+1)*((DWIDTH-1)-(0)+1))-1 : 0] ram;

always @ (posedge clk) begin 
  if (wren_a) begin
      ram[((address_a-0)*((DWIDTH-1)-(0)+1)+DWIDTH-1-0) : ((address_a-0)*((DWIDTH-1)-(0)+1))] <= data_a;
  end
  else begin
      out_a <= ram[((address_a-0)*((DWIDTH-1)-(0)+1)+DWIDTH-1-0) : ((address_a-0)*((DWIDTH-1)-(0)+1))];
  end
end
  
always @ (posedge clk) begin 
  if (wren_b) begin
      ram[((address_b-0)*((DWIDTH-1)-(0)+1)+DWIDTH-1-0) : ((address_b-0)*((DWIDTH-1)-(0)+1))] <= data_b;
  end 
  else begin
      out_b <= ram[((address_b-0)*((DWIDTH-1)-(0)+1)+DWIDTH-1-0) : ((address_b-0)*((DWIDTH-1)-(0)+1))];
  end
end

`else

dual_port_ram u_dual_port_ram(
.addr1(address_a),
.we1(wren_a),
.data1(data_a),
.out1(out_a),
.addr2(address_b),
.we2(wren_b),
.data2(data_b),
.out2(out_b),
.clk(clk)
);

`endif

endmodule

module spram (
    clk,
	address,
	wren,
	data,
	out
);

parameter AWIDTH=10;
parameter NUM_WORDS=1024;
parameter DWIDTH=32;

input clk;
input [(AWIDTH-1):0] address;
input  wren;
input [(DWIDTH-1):0] data;
output reg [(DWIDTH-1):0] out;

`ifdef SIMULATION_MEMORY

reg [(((NUM_WORDS-1)-(0)+1)*((DWIDTH-1)-(0)+1))-1 : 0] ram;

always @ (posedge clk) begin 
  if (wren) begin
      ram[((address-0)*((DWIDTH-1)-(0)+1)+DWIDTH-1-0) : ((address-0)*((DWIDTH-1)-(0)+1))] <= data;
  end
  else begin
      out <= ram[((address-0)*((DWIDTH-1)-(0)+1)+DWIDTH-1-0) : ((address-0)*((DWIDTH-1)-(0)+1))];
  end
end
  
`else

single_port_ram u_single_port_ram(
.addr(address),
.we(wren),
.data(data),
.out(out),
.clk(clk)
);

`endif

endmodule
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/local/local_add_sub.v
//////////////////////////////////////////////////////////////////////
module local_add_sub(
dataa,
datab,
cin,
add_sub,
result
);

parameter WIDTH = 32;
parameter PIPELINE = 0;
parameter REPRESENTATION = "SIGNED";

input[WIDTH-1:0] dataa;
input[WIDTH-1:0] datab;
input cin;
input add_sub;
output reg [WIDTH-1:0] result;

always @(*)begin
    if(add_sub == 1'b1)
         result = dataa+datab+cin;
    else
         result = dataa - datab;
end

endmodule
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/local/local_mult.v
//////////////////////////////////////////////////////////////////////
module local_mult(
dataa,
datab,
clock,
clken,
aclr,
result
);

parameter LPM_WIDTHA = 32;
parameter LPM_WIDTHB = 32;
parameter LPM_WIDTHP = 64;
parameter LPM_REPRESENTATION = "SIGNED";

input [LPM_WIDTHA-1:0] dataa;
input [LPM_WIDTHB-1:0] datab;
input clock;
input clken;
input aclr;
output reg [LPM_WIDTHP-1:0] result;

wire signed [LPM_WIDTHA-1:0] signedinputA;
wire signed [LPM_WIDTHB-1:0] signedinputB;
wire signed [LPM_WIDTHP-1:0] signedoutputP;

wire [LPM_WIDTHA-1:0] unsignedinputA;
wire [LPM_WIDTHB-1:0] unsignedinputB;
wire [LPM_WIDTHP-1:0] unsignedinputP;

wire gated_clock;

assign signedinputA = dataa;
assign signedinputB = datab;
assign unsignedinputA = dataa;
assign unsignedinputB = datab;

assign signedoutputP = signedinputA * signedinputB;
assign unsignedoutputP = unsignedinputA * unsignedinputB;

assign gated_clock = clock & clken;

always @(posedge gated_clock)begin
    if(aclr)begin
       result <= 0;
    end
    else if(LPM_REPRESENTATION == "SIGNED")
       result <= signedoutputP;
    else
       result <= unsignedoutputP; 
end

endmodule
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/local/local_fifo.v
//////////////////////////////////////////////////////////////////////
module fifo#(
parameter FIFO_WIDTH = 8,
parameter FIFO_DEPTH = 4
         )(
input                      clk,
input                      reset,
input   [FIFO_WIDTH-1:0]   wrdata,
input                      write,

input                       read,
output reg [FIFO_WIDTH-1:0] rddata,
output reg                  full,
output reg                  empty 
);

localparam PTR_WIDTH = $clog2(FIFO_DEPTH);

reg [(((FIFO_DEPTH-1)-(0)+1)*((FIFO_WIDTH-1)-(0)+1))-1 : 0] mem;
reg  [PTR_WIDTH:0]    wrptr,rdptr;
wire [PTR_WIDTH:0]    wrptr_nxt,rdptr_nxt;
integer i;

assign  wrptr_nxt = write? wrptr + 1'b1: wrptr;
assign  rdptr_nxt = read? rdptr + 1'b1:rdptr ;

always@(*) begin
    rddata = mem[rdptr[PTR_WIDTH-1:0]];
end

always@(posedge clk) begin
  if(!reset)begin
    full <= 1'b0;
  end
  else begin
    if(wrptr_nxt[PTR_WIDTH]^ rdptr_nxt[PTR_WIDTH])
  	full <= ~(|(wrptr_nxt[PTR_WIDTH-1:0]^rdptr_nxt[PTR_WIDTH-1:0]));
    else
	full <= 1'b0;
  end 
end

always@(posedge clk) begin
  if(!reset)begin
    empty <= 1'b1;
  end
  else begin
    empty <= ~(|(wrptr_nxt^rdptr_nxt));
  end 
end

always @(posedge clk) begin
 if(!reset)begin
   wrptr <= 'h0;
 end
 else begin
     wrptr <= wrptr_nxt;
 end
end

always@(posedge clk) begin
 if(!reset)begin
   rdptr <= 'h0;
 end
 else begin
     rdptr <= rdptr_nxt;
 end
end

always@(posedge clk) begin
 if(!reset)begin
   for(i = 0; i<FIFO_DEPTH;i=i+1)begin
       mem[((i-0)*((FIFO_WIDTH-1)-(0)+1)+FIFO_WIDTH-1-0) : ((i-0)*((FIFO_WIDTH-1)-(0)+1))] <= 'h0;
   end
 end
 else begin
   if(write)begin
      mem[wrptr[PTR_WIDTH-1:0]]<= wrdata;
   end
 end
end

endmodule

//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/local/local_shifter.v
//////////////////////////////////////////////////////////////////////
module local_shifter(
  data,
  distance,
  direction,
  result
);

parameter LPM_WIDTH = 32;
parameter LPM_WIDTHDIST = 5;
parameter LPM_SHIFTTYPE = "LOGICAL";

input [LPM_WIDTH-1:0] data;
input [LPM_WIDTHDIST-1:0] distance;
input direction;

output reg [LPM_WIDTH-1:0] result;
reg [LPM_WIDTH-1:0] arith_reg; 
always @* begin
  arith_reg = {LPM_WIDTH{1'b1}};
  case(LPM_SHIFTTYPE)
    "LOGICAL":begin 
                  if(direction == 1'b0)begin
                      result = data << distance;
                  end
                  else begin
                      result = data >> distance;
                  end
              end
    "ARITHMATIC":begin 
                   if(direction)begin
                      result = data << distance;
                   end
                   else begin
                      if(data[LPM_WIDTH-1] == 1'b1)
                         result =  ((arith_reg <<(LPM_WIDTH - distance))|| (data >> distance));
                      else
                         result = data >> distance;
                   end
                 end
    default:begin
                  if(direction == 1'b0)begin
                      result = data << distance;
                  end
                  else begin
                      result = data >> distance;
                  end
            end
  endcase
end
endmodule
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/latest/tpu_v2-main/tpu/design/scalar/components.v
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
