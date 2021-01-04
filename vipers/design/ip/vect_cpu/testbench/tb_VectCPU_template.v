// ****************************************
// Filename: tb_VectCPU.v
// Author: Jason Yu
//
// Testbench for entire vector CPU
//
// Copyright (C) 2007 Jason Yu
//
// ****************************************

`timescale 1ns / 100ps
`include "../hdl/mathmacros.v"
`include "../hdl/isa_def.v"

//`define	SIM_DWAIT				// simulate the system fabric generating d_waitrequest in response to cpu generating memory requests
//`define	SIM_DWAIT_MIDTRANSFER	// simulate generating d_waitrequest in the middle of a transfer, instead of immediately in response to a transfer request

`define	MODELSIM

// Number of address bits to main data memory
//`define	DMEM_ADDR_N	22
`define	DMEM_ADDR_N	20

module tb_VectCPU
#(
	// need to specify in decimal due to a bug in SOPC builder
	parameter	RESET_ADDRESS		= 32'h00000000,
	parameter	BREAK_ADDRESS		= 32'h00000000,
	parameter	EXCEPTION_ADDRESS	= 32'h00000000,
	parameter	CPU_ID				= 3,
	// parameter	RESET_ADDRESS		= 32'h00000000,
	// parameter	BREAK_ADDRESS		= 32'h00000000,
	// parameter	EXCEPTION_ADDRESS	= 32'h00000020,
	// parameter	CPU_ID				= 32'h00000003,
	
	// Vector processor primary parameters
**template_replace_line**
)
// Ports
;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local parameters
//---------------------------------------------------------------------------------------------------------------------------------------------------
localparam	MEMWIDTH_BYTES = MEM_WIDTH/8;			// memory width in number of bytes


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
//    integer trace;
integer store_trace;
//    integer exception_trace;


// **** Inputs to CPU ****
reg		clk;
reg		reset_n;

// Instruction master
wire	[MEM_WIDTH-1:0]	d_readdata;
reg						d_waitrequest;
wire	[31:0]			i_readdata;
reg						i_waitrequest;
wire	[31:0]			i_address;
wire					i_read;
wire					i_read_posedge;
reg						i_read_delay;

// Data master
wire	[MEMWIDTH_BYTES-1:0]	d_byteenable;
wire	[MEM_WIDTH-1:0]	d_writedata;
wire	[31:0]			d_address;
wire					d_read;
wire					d_write;
wire	[31:0]			d_irq;

// Memory interface
reg		[31:0]			scalar_d_address;
reg						scalar_d_read;
wire	[31:0]			scalar_d_readdata;
wire					scalar_d_waitrequest;

// **** Local signals ****
wire	[31:0]			instraddr;
reg		[31:0]			imem[2**`DMEM_ADDR_N-1:0];			// instruction memory, 32b address

reg		d_write_delay;
wire	d_write_posedge;
reg		d_read_delay;
wire	d_read_posedge;

wire	[31:0]				lsu_opCode;
reg		[31:0]				vpu_to_lsu_opCode;
wire						id_newInstr_store;
wire						vputolsu_instr_decode_valid;
wire	[MEM_WIDTH-1:0]		writedata_masked;
reg		[MEM_WIDTH-1:0]		byteena_mask;

integer i;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Reset and stimulate clock
//---------------------------------------------------------------------------------------------------------------------------------------------------

initial 
  begin
	// $dumpfile ("dump.vcd");
	// $dumpvars;
	d_waitrequest = 1'b0;
	i_waitrequest = 1'b0;
	
	// Read initial memory values for testing
	$readmemh("imem.hex",imem);

	reset_n = 1'b1;
	clk = 1'b1;							// start clk rising
	#10 reset_n = 1'b0;
	#10 reset_n = 1'b1;
  end

// Toggle the clock
always
  #5 clk <= ~clk;

/*
initial begin
	$dumpfile ("dump.vcd");
	$dumpvars;
end

initial begin
	$display("\t\ttime, \tclk, \treset_n, \topCode");
	$monitor("%d,\t%b,\t%b,\t%d", $time, clk, reset_n, opCode);
end
*/


//---------------------------------------------------------------------------------------------------------------------------------------------------
// opcode memory
//---------------------------------------------------------------------------------------------------------------------------------------------------
assign	i_readdata = imem[instraddr];
assign	instraddr = i_address >> 2;			// instruction is in byte, while instruction is in word

assign	i_read_posedge = i_read & ~i_read_delay;

always @(posedge clk) begin
	i_read_delay <= i_read;
end

// `ifdef	SIM_DWAIT
// Simulate a i_waitrequest signal
// always @(posedge i_read_posedge) begin
// `ifdef	SIM_DWAIT_MIDTRANSFER
	// Wait 1 cycle before asserting, see if it messes up
	// @(posedge clk);
// `endif
	// assert on falling edge
	// @(negedge clk)
	// i_waitrequest = 1'b1;
	// Wait 1 cycle
	// @(posedge clk);
	// @(posedge clk);
	// deassert on falling edge
	// @(negedge clk);
	// i_waitrequest = 1'b0;
// end
// `endif

// always @(*)
	// i_waitrequest = 1'b0;


//---------------------------------------------------------------------------------------------------------------------------------------------------	
// Generate a scalar load request
//---------------------------------------------------------------------------------------------------------------------------------------------------
initial begin
	store_trace = $fopen("store_trace.txt","w");
	$fwrite(store_trace,"time\topcode\taddress\tdata\n");
	// $display("store trace=%d",store_trace);

	scalar_d_read = 1'b0;
	
	// #33
	// after #330
	// d_waitrequest = 1'b1;
	// #4 d_waitrequest = 1'b0;
	
	// #12
	// scalar_d_address = 32'h8;
	// scalar_d_read = 1'b1;
	
	// wait for scalar_d_waitrequest to go high
	// @ (posedge scalar_d_waitrequest);
	// #4 d_waitrequest = 1'b1;
	// #8 d_waitrequest = 1'b0;

	// wait for memory transfer to complete
	// @ (negedge scalar_d_waitrequest);
	// scalar_d_read = 1'b0;
	
	// $display("scalar_d_readdata: %h", scalar_d_readdata);

end


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Simulate d_waitrequest
//---------------------------------------------------------------------------------------------------------------------------------------------------
// `ifdef	SIM_DWAIT	
// detect rising edge of d_write
assign	d_write_posedge = d_write & ~d_write_delay;
assign	d_read_posedge = d_read & ~d_read_delay;

always @(posedge clk) begin
	d_write_delay <= d_write;
	d_read_delay <= d_read;
end

// Simulate a d_waitrequest signal
// always @(posedge d_write_posedge, posedge d_read_posedge) begin
// `ifdef	SIM_DWAIT_MIDTRANSFER
	// Wait 1 cycle before asserting, see if it messes up
	// @(posedge clk);
// `endif
	// assert on falling edge
	// @(negedge clk)
	// d_waitrequest = 1'b1;
	// Wait 2 cycles
	// @(posedge clk);
	// @(posedge clk);
	// @(posedge clk);
	// @(posedge clk);
	// deassert on falling edge
	// @(negedge clk);
	// d_waitrequest = 1'b0;
// end
// `endif

// always @(*)
	// d_waitrequest = 1'b0;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Device under test
//---------------------------------------------------------------------------------------------------------------------------------------------------
VectCPU
#(
	// need to specify in decimal due to a bug in SOPC builder
	.RESET_ADDRESS		(RESET_ADDRESS),
	.BREAK_ADDRESS		(BREAK_ADDRESS),
	.EXCEPTION_ADDRESS	(EXCEPTION_ADDRESS),
	.CPU_ID				(CPU_ID),
	
	// Vector processor primary parameters
	.VPU_WIDTH			(VPU_WIDTH),
	.NUMLANES			(NUMLANES),
	.MEM_WIDTH			(MEM_WIDTH),
	.MINDATAWIDTH		(MINDATAWIDTH)
)
myVectCPU
(
	// Inputs
	.clk			(clk),
	.reset_n			(reset_n),
	.d_irq			(d_irq),
	.d_readdata		(d_readdata),
	.d_waitrequest	(d_waitrequest),
	.i_readdata		(i_readdata),
	.i_waitrequest	(i_waitrequest),

	// Outputs
	.d_byteenable	(d_byteenable),
	.d_writedata	(d_writedata),
	.d_address		(d_address),
	.d_read			(d_read),
	.d_write		(d_write),
	.i_address		(i_address),
	.i_read			(i_read)
`ifdef	MODELSIM
	// Simulation trace signals
	,
	.id_newInstr_store	(id_newInstr_store),
	.vputolsu_instr_decode_valid	(vputolsu_instr_decode_valid),
	.lsu_opCode		(lsu_opCode)
`endif
);


//-------------------------------
// On-chip data memory
//-------------------------------
dMemModel
#(	.MEM_WIDTH		(MEM_WIDTH),
	.DMEM_NUMWORDS	(2**`DMEM_ADDR_N-1)	)
theDataMemory
(
	.byteena		(d_byteenable),
	.clock			(clk),
	.data			(d_writedata),
	.address		(d_address),
	.wren			(d_write),
	.q				(d_readdata)
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Trace of Stores to Memory
//---------------------------------------------------------------------------------------------------------------------------------------------------
// make masked out bits into 'x' to enhance readability
assign	writedata_masked = ({MEM_WIDTH{1'bx}} & ~byteena_mask) | (d_writedata & byteena_mask);


// Expand byte enable to bit-level mask
always @(*) begin
	for (i=0; i<MEM_WIDTH; i=i+1)
		byteena_mask[i] = d_byteenable[i/8];
end

// Capture store traces
always @(posedge clk) begin
	if (d_write && ~d_waitrequest && reset_n) begin
		$display("Writing to store trace: t=%d", $time);
		$fwrite(store_trace,"t=%d | OP=%h | %h: %h\n", $time, vpu_to_lsu_opCode, d_address, writedata_masked);
	end
end


// Capture the store instruction opcode
always @(posedge clk) begin
	if (id_newInstr_store && vputolsu_instr_decode_valid)
		vpu_to_lsu_opCode <= lsu_opCode;
end



//---------------------------------------------------------------------------------------------------------------------------------------------------
// Clean up & close files
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	end_sim_condition;
assign	end_sim_condition = ((i_readdata === 32'hxxxxxxxx) && (lsu_opCode == `NOP_OPCODE) && ~d_write && ~d_read) ? 1'b1 : 1'b0;

always @(posedge end_sim_condition) begin
	// wait #100 to see if anything else happens
	#100
	if (end_sim_condition) begin
		// close the store trace file
		$fclose(store_trace);
		$display("Closing store trace file, t=%d",$time);
	end
end


/***************************************************************/
	// initial
	// #1000 $finish;

/*
`ifdef POWER_POSTSYNTHESIS 
  initial           // NEW
  begin
	$readmemh("instr.rif",imem);
	$readmemh("data.rif",dmem);
	boot_iaddr=0;
	boot_daddr=0;
  end
`endif
*/
/**************** Trace of Write Backs ********************/
/*    initial
begin
	trace=$fopen("modelsim/modelsim_trace.txt","w");
	store_trace=$fopen("modelsim/modelsim_store_trace.txt","w");
end

always@(posedge clk)
begin
	if (reg_file_we && (|reg_file_dst) && reset_n)
		$fwrite(trace,"%d | PC=%h | IR=%h | %h: %h\n",
				$time,
				pc,
				instr,
				reg_file_dst,
				reg_file_data);
end
*/

/**************** Clean up & close files ********************/
/*
always@(data_mem_data && data_mem_we )
  if (data_mem_data==32'hdeaddead && data_mem_we)
  begin
  $fclose(trace);
  $fclose(store_trace);
  end
`endif
*/

/**************** User inserted code ********************/

endmodule
