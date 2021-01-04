//************************************************************************
// Scalable Vector CPU Project -- Top level
// Filename: VectCPU.v
// Author: Jason Yu
//
// Top level of scalable vector processor
//
//  Copyright (C) 2007 Jason Yu
//
//************************************************************************

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on
`ifdef	MODELSIM
	`include "../hdl/define.v"
	`include "../hdl/config_def.v"
	`include "../hdl/isa_def.v"
`else
	`include "define.v"
	`include "config_def.v"
	`include "isa_def.v"
`endif


module	VectCPU

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Customizable parameters
//---------------------------------------------------------------------------------------------------------------------------------------------------
#(
	// need to specify in decimal due to a bug in SOPC builder
	parameter	RESET_ADDRESS		= 0,
	parameter	BREAK_ADDRESS		= 0,
	parameter	EXCEPTION_ADDRESS	= 32,
	parameter	CPU_ID				= 3,
	// parameter	RESET_ADDRESS		= 32'h00000000,
	// parameter	BREAK_ADDRESS		= 32'h00000000,
	// parameter	EXCEPTION_ADDRESS	= 32'h00000020,
	// parameter	CPU_ID				= 32'h00000003,
	
	// Vector processor primary parameters
	parameter	VPU_WIDTH			= 32,
	parameter	NUMLANES			= 8,
	parameter	MEM_WIDTH			= 128,
	parameter	MINDATAWIDTH		= 8,
	
	parameter	MEMWIDTH_BYTES = MEM_WIDTH/8			// memory width in number of bytes
)

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Ports
//---------------------------------------------------------------------------------------------------------------------------------------------------
(
	// Global signals
	input			clk,
	input			reset_n,	
	// Avalon Master 1 : Instruction master
	input	[31:0]				i_readdata,
	input						i_waitrequest,
	// Avalon Master 2: Data master
	input	[31:0]				d_irq,
	input	[MEM_WIDTH-1:0]		d_readdata,	// data from main memory
	input						d_waitrequest,
	
	// Outputs
	// Avalon Master 1: Instruction master
	output	[31:0]				i_address,
	output						i_read,
	// Avalon Master 2: Data master
	output	[MEMWIDTH_BYTES-1:0]	d_byteenable,
	output	[MEM_WIDTH-1:0]		d_writedata,
	output	[31:0]				d_address,
	output						d_read,
	output						d_write	
	
`ifdef	MODELSIM
	// Simulation trace signals
	,
	output						id_newInstr_store,
	output						vputolsu_instr_decode_valid,
	output	[31:0]				lsu_opCode
`endif
);

//---------------------------------------------------------------------------------------------------------------------------------------------------	
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Simulation trace signals
`ifdef	MODELSIM
`else
// Define is not simulation
wire	[31:0]				lsu_opCode;					// portion of the opcode to loadstore controller
wire						vputolsu_instr_decode_valid;
`endif

wire						cpu_fetch_stall;			// Stall the CPU fetch stage
wire	[31:0]				scalarRegOut;				// connection from scalar register to vector register
// Memory interface signals
wire	[31:0]				id_vbase;						// vector base
wire	[31:0]				id_vl_special_instr;
wire						lsu_executing_vector_load;	// ready for the next load instruction
wire	[NUMLANES-1:0]		extLoad_lane_we;			// lane wren from loads
wire	[NUMLANES*VPU_WIDTH-1:0]	memLoadDataToLanes;	// data to processors
wire	[NUMLANES-1:0]		laneStoreQRdreq;		// read request to store queue in lanes
wire						flagStoreQRdreq;			// read request to read flag queue for flag store instruction
wire	[`log2(`VRF_NREG)-1:0]	lsu_vrf_wraddr;
wire						lsu_vflag_reg;				// copy flag register from opcode for writing loaded data back to VRF
wire						id_clear_lsu_data_flag;		// clear the load data waiting flag
wire	[2:0]				lsu_data_waiting_flag;		// Flag to indicate what type of data is waiting the load data transfer queue
wire	[NUMLANES*VPU_WIDTH-1:0]	VLaneDataToMem;
wire	[NUMLANES-1:0]		VLaneFlagToMem;
wire						flagStoreQEmpty;			// whether the store flag data queue is empty

// Scalar CPU memory signals
wire	[31:0]	scalar_d_readdata;
wire	[31:0]	scalar_d_address;
wire	[3:0]	scalar_d_byteenable;
wire			scalar_d_read;
wire			scalar_d_write;
wire	[31:0]	scalar_d_writedata;
wire			scalar_d_waitrequest;
wire	[1:0]	scalar_data_size;						// size of data to be writte to data master: 00 = byte, 01 = halfword, 10 = word
wire			lsu_busy;
wire	[31:0]	vector_opcode;
wire			vector_cop_instruction;					// a vector coprocessor instruction has been fetched

// Scalar to vector data transfer queue
wire	scalarToVectorQ_empty;						// queue is empty
wire	scalarToVectorQ_full;						// queue is full
wire	scalarToVectorQ_rdreq;					// queue read request
wire	scalarToVectorQ_wrreq;

// transfer queue from vector to scalar core
wire	vectorToScalarQ_rdreq;
wire	vectorToScalarQ_wrreq;
wire	[31:0]	vectorToScalarQ_data;
wire			vectorToScalarQ_empty;
wire	[31:0]	vectorToScalarQ_q;
wire			vectorToScalarQ_full;
wire			loadDataQ_full;

wire	[`log2(NUMLANES)-1:0]	lsu_indexOffset_lanesel;	// select which index offset to use
wire	[VPU_WIDTH-1:0]			indexOffset_out;	// offset from lanes for indexed addressing
wire			lsu_indexed_memaccess;
wire	[2:0]	id_memDataWidth;						// data width for memory access; id
wire	[31:0]	vctrl_q;		// control register output
wire	[31:0]	id_vl_minus_one;						// original vector length - 1
wire	[VPU_WIDTH-1:0]	scalarQ_q;

wire	mem_order_q_data;
wire	mem_order_q_rdreq;
wire	mem_order_q_wrreq;
wire	mem_order_q_empty;
wire	mem_order_q_full;
wire	mem_order_q_q;
wire	lsu_wraddr_q_rdreq;
wire	load_done;								// asserted the last cycle of load

wire	predecode_scalarmem_instr;		// predecode scalar memory instruction
wire	predecode_vectormem_instr;		// predecode vector memory instruction

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Signal assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Scalar core
//---------------------------------------------------------------------------------------------------------------------------------------------------
UT_II_Economy_cpu
#(	.RESET_ADDRESS			(RESET_ADDRESS),
	.BREAK_ADDRESS			(BREAK_ADDRESS),
	.EXCEPTION_ADDRESS		(EXCEPTION_ADDRESS),
	.CPU_ID					(CPU_ID)
)
ScalarCore
(
	// Inputs
	.clk					(clk),
	.reset_n				(reset_n),
	.d_irq					(d_irq),
	.d_readdata				(scalar_d_readdata),
	.d_waitrequest			(scalar_d_waitrequest),
	.i_readdata				(i_readdata),
	.i_waitrequest			(i_waitrequest),

	// Inputs from vector core
	.cop_fetch_stall		(cpu_fetch_stall),
	.scalarToVectorQ_full	(scalarToVectorQ_full),

	// vector to scalar transfer queue
	.vectorToScalarQ_empty	(vectorToScalarQ_empty),
	.vectorToScalarQ_q		(vectorToScalarQ_q),

	
	// Bidirectionals

	// Outputs
	.d_address				(scalar_d_address),
	.d_byteenable			(scalar_d_byteenable),
	.d_read					(scalar_d_read),
	.d_write				(scalar_d_write),
	.d_writedata			(scalar_d_writedata),
	.d_data_size			(scalar_data_size),		// size of data to be writte to data master: 00 = byte, 01 = halfword, 10 = word
	.i_address				(i_address),
	.i_read					(i_read),
	
	// Outputs to vector core
	.vector_cop_instruction	(vector_cop_instruction),	// a vector coprocessor instruction has been fetched
	.vector_opcode			(vector_opcode),
	.predecode_scalarmem_instr	(predecode_scalarmem_instr),		// predecode scalar memory instruction
	.predecode_vectormem_instr	(predecode_vectormem_instr),		// predecode vector memory instruction
	.scalarRegOut			(scalarRegOut),
	.scalarToVectorQ_wrreq	(scalarToVectorQ_wrreq),
	
	// vector to scalar transfer queue
	.vectorToScalarQ_rdreq	(vectorToScalarQ_rdreq)
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Queue to decouple scalar and vector cores
//---------------------------------------------------------------------------------------------------------------------------------------------------

// Transfer queue from scalar to vector core
scalarToVectorQ	scalarToVectorQ_inst (
	.clock ( clk ),
	.data ( scalarRegOut ),
	.rdreq ( scalarToVectorQ_rdreq ),
	.wrreq ( scalarToVectorQ_wrreq ),
	.empty ( scalarToVectorQ_empty ),
	.full ( scalarToVectorQ_full ),
	.usedw ( ),
	.q ( scalarQ_q )
);

// Transfer queue from scalar to vector core
scalarToVectorQ	vectorToScalar_fifo (
	.clock ( clk ),
	.data ( vectorToScalarQ_data ),
	.rdreq ( vectorToScalarQ_rdreq ),
	.wrreq ( vectorToScalarQ_wrreq ),
	.empty ( vectorToScalarQ_empty ),
	.full ( vectorToScalarQ_full ),
	.usedw ( ),
	.q ( vectorToScalarQ_q )
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Memory access ordering queue
// - written by scalar and vector cores to keep memory accesses in program order
// - FIFO uses show-ahead FIFO mode to match pipelining for vector memory instruction decoding
//---------------------------------------------------------------------------------------------------------------------------------------------------
// write 0 for scalar access, 1 for vector access
assign	mem_order_q_data = (predecode_scalarmem_instr == 1'b1) ? 1'b0 : 1'b1;
assign	mem_order_q_wrreq = predecode_scalarmem_instr | predecode_vectormem_instr;

mem_order_q	mem_order_q_inst (
	.aclr ( ~reset_n ),
	.clock ( clk ),
	.data ( mem_order_q_data ),
	.rdreq ( mem_order_q_rdreq ),
	.wrreq ( mem_order_q_wrreq ),
	.empty ( mem_order_q_empty ),
	.full ( mem_order_q_full ),
	.q ( mem_order_q_q ),
	.usedw (  )
	);

	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Vector core
//---------------------------------------------------------------------------------------------------------------------------------------------------
VPU_top
#(	// Vector processor primary parameters
	.VPU_WIDTH				(VPU_WIDTH),
	.NUMLANES				(NUMLANES),
	.MEM_WIDTH				(MEM_WIDTH),
	.MINDATAWIDTH			(MINDATAWIDTH)
)
myVPU
(
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),
	.vpu_new_instr				(vector_cop_instruction),
	.opCode						(vector_opcode),		// vector instruction opcode
	.scalarRegIn				(scalarQ_q),			// Input from scalar unit for vector-scalar instructions

	// Memory interface inputs
	.lsu_executing_vector_load	(lsu_executing_vector_load),		// ready for the next load instruction
	.lsu_busy					(lsu_busy),
	.extLoad_lane_we			(extLoad_lane_we),		// lane wren from loads
	.memLoadDataToLanes			(memLoadDataToLanes),			// data to processors
	.laneStoreQRdreq			(laneStoreQRdreq),	// read request to store queue in lanes
	.flagStoreQRdreq			(flagStoreQRdreq),							// read request to read flag queue for flag store instruction
	.lsu_data_waiting_flag		(lsu_data_waiting_flag),				// Flag to indicate what type of data is waiting the load data transfer queue
	.lsu_vrf_wraddr				(lsu_vrf_wraddr),
	.lsu_vflag_reg				(lsu_vflag_reg),		// copy flag register from opcode for writing loaded data back to VRF
	.scalarToVectorQ_empty		(scalarToVectorQ_empty),
	.vectorToScalarQ_full		(vectorToScalarQ_full),
	.lsu_indexOffset_lanesel	(lsu_indexOffset_lanesel),	// select which index offset to use
	.lsu_indexed_memaccess		(lsu_indexed_memaccess),
	.load_done					(load_done),
	
	// Outputs
	.cpu_fetch_stall			(cpu_fetch_stall),		// stall the CPU fetch stage
	.scalarToVectorQ_rdreq		(scalarToVectorQ_rdreq),
	
	// transfer queue from vector to scalar core
	.vectorToScalarQ_wrreq		(vectorToScalarQ_wrreq),
	.vectorToScalarQ_data		(vectorToScalarQ_data),
	.VLaneDataToMem				(VLaneDataToMem),		// memory write inputs
	.VLaneFlagToMem				(VLaneFlagToMem),
	.id_vl_special_instr		(id_vl_special_instr),
	.vl_minus_one				(id_vl_minus_one),						// original vector length - 1
	.vctrl_q					(vctrl_q),
	.vbase						(id_vbase),				// vector base
	.id_memDataWidth			(id_memDataWidth),
	.lsu_opCode					(lsu_opCode),
	
	.vputolsu_instr_decode_valid	(vputolsu_instr_decode_valid),
	.loadDataQ_full				(loadDataQ_full),
	.lsu_wraddr_q_rdreq			(lsu_wraddr_q_rdreq),
	.id_clear_lsu_data_flag		(id_clear_lsu_data_flag),			// clear the load data waiting flag
	.indexOffset_out			(indexOffset_out),	// offset from lanes for indexed addressing
	.flagStoreQEmpty			(flagStoreQEmpty)
	
	// Simulation trace signals
`ifdef	MODELSIM
	,
	.id_newInstr_store			(id_newInstr_store)
`endif
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Interface to memory
//---------------------------------------------------------------------------------------------------------------------------------------------------
memIF
#(	// Vector processor primary parameters
	.VPU_WIDTH				(VPU_WIDTH),
	.NUMLANES				(NUMLANES),
	.MEM_WIDTH				(MEM_WIDTH),
	.MINDATAWIDTH			(MINDATAWIDTH)
)
dmemIF_inst
(
	// Vector VPU Inputs
	.clk						(clk),
	.reset_n					(reset_n),
	.lsu_opCode					(lsu_opCode),
	.laneDataIn					(VLaneDataToMem),		// memory write inputs
	.laneFlagQIn				(VLaneFlagToMem),
	.id_vl_minus_one			(id_vl_minus_one),
	.id_vl_special_instr		(id_vl_special_instr),
	.vctrl_q					(vctrl_q),
	.id_vbase					(id_vbase),
	.id_memDataWidth			(id_memDataWidth),
	.loadDataQ_full				(loadDataQ_full),
	.id_clear_lsu_data_flag		(id_clear_lsu_data_flag),			// clear the load data waiting flag
	.flagStoreQEmpty			(flagStoreQEmpty),
	.lsu_instr_decode_valid		(vputolsu_instr_decode_valid),

	// Scalar processor inputs
	.scalar_d_address			(scalar_d_address),
	.scalar_d_byteenable		(scalar_d_byteenable),
	.scalar_d_read				(scalar_d_read),
	.scalar_d_write				(scalar_d_write),
	.scalar_d_writedata			(scalar_d_writedata),
	.scalar_data_size			(scalar_data_size),
	
	// Inputs from main memory
	.d_waitrequest				(d_waitrequest),
	.d_readdata					(d_readdata),			// main memory read data
	.indexOffset_out			(indexOffset_out),	// offset from lanes for indexed addressing
	
	.mem_order_q_q				(mem_order_q_q),
	.mem_order_q_empty			(mem_order_q_empty),
	.lsu_wraddr_q_rdreq			(lsu_wraddr_q_rdreq),

	// Outputs to memory
	.d_read						(d_read),
	.memWrdata					(d_writedata),			// Memory write Outputs
	.d_write					(d_write),
	.d_byteenable				(d_byteenable),			// data mask
	.d_address					(d_address),

	// Outputs to Scalar processor
	.scalar_d_waitrequest		(scalar_d_waitrequest),
	.scalar_d_readdata			(scalar_d_readdata),
	
	// Outputs to VPU
	.lsu_vrf_wraddr				(lsu_vrf_wraddr),
	.lsu_vflag_reg				(lsu_vflag_reg),			// copy flag register from opcode for writing loaded data back to VRF
	.lsu_executing_vector_load	(lsu_executing_vector_load),
	.lsu_busy					(lsu_busy),
	.data_lane_we				(extLoad_lane_we),
	.memLoadDataToLanes			(memLoadDataToLanes),
	.laneStoreQRdreq			(laneStoreQRdreq),
	.flagStoreQRdreq			(flagStoreQRdreq),				// read request to read flag queue for flag store instruction
	.lsu_indexOffset_lanesel	(lsu_indexOffset_lanesel),	// select which index offset to use
	.lsu_indexed_memaccess		(lsu_indexed_memaccess),
	.lsu_data_waiting_flag		(lsu_data_waiting_flag),				// Flag to indicate what type of data is waiting the load data transfer queue
	
	.mem_order_q_rdreq			(mem_order_q_rdreq),
	.load_done					(load_done)
);

endmodule
