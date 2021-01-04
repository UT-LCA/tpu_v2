//************************************************************************
// Scalable Vector CPU Project -- Pipeline registers
// Filename: VPU_pipeline_regs.v
// Author: Jason Yu
//
// VPU pipeline registers
// - contains pipelining registers for control signals that are not derived
// from the stage decoding logic
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


module VPU_pipeline_regs
#(	// Vector processor primary parameters
	parameter	NUMLANES = 1
)
(
	// ***********
	// ** Inputs **
	// ***********
	input			clk,
	input			reset_n,
	input	[31:0]	instruction_register_q,
	input			vpu_instr_decode_valid,			// a new instruction is decoded
	input			vpu_control_ready,
	input			vpu_decodestage_stall,			// decode stage needs to stall
	input			vpu_executestage_stall,			// execute 1 stage needs to stall
	input			vpu_memorystage_stall,			// memory stage needs to stall
	
	input			id_instr_stalls_pipeline,		// instruction stalls pipeline until complete
	input			id_vectorToScalarQ_wrreq,		// write to vector to scalar core transfer queue

	// input			id_indexed_memaccess,
	// input			id_newInstr_store,
	
	// Other control inputs
	input			id_clusterStoreQWrreq,
	input			id_flagStoreQWrreq,
	
	// input	[`log2(`ELEMPERLANE)-1:0]	s2_vrfWrOffset,		// for stall logic
	input	[`log2(`ELEMPERLANE)-1:0]	s2_vrfOffset,		// for stall logic
	input	[`log2(NUMLANES):0]	cycleLastEnabledLane,	// remainder of dividing vlength/vindex by lane; range = 1 to NUMLANE
	input	[`log2(NUMLANES)-1:0]	cycleFirstLaneIndex,	// 
	
	// ************
	// ** Outputs **
	// ************
	output	[31:0]	s3_opCode,
	output	[31:0]	s4_opCode,						// opcode for pipeline stage 4
	output			s4_instr_stalls_pipeline_done,	// instruction is non-pipelined
	// Control outputs to datapath
	output	[`log2(`ELEMPERLANE)-1:0]	s3_vrfOffset,		// for stall logic
	output	[`log2(`ELEMPERLANE)-1:0]	s4_vrfOffset,		// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
	// Memory interface signals
	output			s3_clusterStoreQWrreq,
	output			s3_flagStoreQWrreq,
	// transfer between scalar and vector cores
	output			s3_vectorToScalarQ_wrreq,		// write to vector to scalar core transfer queue
	output	[`log2(NUMLANES):0]	s3_cycleLastEnabledLane,	// remainder of dividing vlength/vindex by lane; range = 1 to NUMLANE
	output	[`log2(NUMLANES)-1:0]	s3_cycleFirstLaneIndex
);

//---------------------------------------------------------------------------------------------------------------------------------------------------
//	Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	s3_instr_stalls_pipeline;
wire	vpu_control_ready_delay;

//---------------------------------------------------------------------------------------------------------------------------------------------------
//
//---------------------------------------------------------------------------------------------------------------------------------------------------
ff	#(.WIDTH(1))	stage3_vpu_control_ready_delay (.clk(clk), .en(1'b1), .clrn(reset_n), .d(vpu_control_ready), .q(vpu_control_ready_delay));

// vpu_control_ready_delay is used because we need minimum of one cycle of wrreq high
pipeline_reg_sclr	#(.WIDTH(1))
stage3_clusterStoreQWrreq
(	.clk(clk),
	.en(vpu_instr_decode_valid),
	// special case for indexed store
	// .en(vpu_instr_decode_valid | (id_indexed_memaccess & id_clusterStoreQWrreq & ~vpu_control_ready)),
	// .en(vpu_instr_decode_valid | (id_indexed_memaccess & id_newInstr_store)),
	.clrn(reset_n),
	.d(id_clusterStoreQWrreq),
	.sclr(vpu_control_ready_delay & ~vpu_instr_decode_valid),
	// .sclr((~(id_indexed_memaccess & id_newInstr_store) & (vpu_control_ready_delay & ~vpu_instr_decode_valid))
			// | ((id_indexed_memaccess & id_newInstr_store) & vpu_control_ready_delay)),
	.q(s3_clusterStoreQWrreq)
);

pipeline_reg_sclr	#(.WIDTH(1))
stage3_flagStoreQWrreq
(	.clk(clk),
	.en(vpu_instr_decode_valid),
	// special case for indexed store
	// .en(vpu_instr_decode_valid | (id_indexed_memaccess & id_clusterStoreQWrreq & ~vpu_control_ready)),
	// .en(vpu_instr_decode_valid | (id_indexed_memaccess & id_newInstr_store)),
	.clrn(reset_n),
	.d(id_flagStoreQWrreq),
	.sclr(vpu_control_ready_delay & ~vpu_instr_decode_valid),
	// .sclr((~(id_indexed_memaccess & id_newInstr_store) & (vpu_control_ready_delay & ~vpu_instr_decode_valid))
			// | ((id_indexed_memaccess & id_newInstr_store) & vpu_control_ready_delay)),
	.q(s3_flagStoreQWrreq)
);


// these signals control the VRF offset and which lanes to enable, need to be able to change each cycle to support vector length (not enable all lanes for last cycle)
ff	#(.WIDTH(`log2(NUMLANES)+1)) 	stage3_cycleLastEnabledLane (.clk(clk), .en(1'b1), .clrn(reset_n), .d(cycleLastEnabledLane), .q(s3_cycleLastEnabledLane));
ff	#(.WIDTH(`log2(NUMLANES))) 	stage3_cycleFirstLaneIndex (.clk(clk), .en(1'b1), .clrn(reset_n), .d(cycleFirstLaneIndex), .q(s3_cycleFirstLaneIndex));


//---------------------------------------------------------------------------------------------------------------------------------------------------
//	Stage 3: Reg read/Execute 1
//---------------------------------------------------------------------------------------------------------------------------------------------------
// register the opcode
ff_sclr	#(.WIDTH(32)) 	stage3_opCode (.clk(clk), .en((vpu_instr_decode_valid & ~vpu_executestage_stall) | vpu_decodestage_stall), .clrn(reset_n), .d(instruction_register_q), .q(s3_opCode), .sclr(vpu_decodestage_stall));

// signal only needs to be pulsed, and can be reset as soon as the instruction moves to the next pipeline stage
ff_sclr	#(.WIDTH(1))	stage3_vectorToScalarQ_wrreq (.clk(clk), .en(~vpu_executestage_stall | vpu_decodestage_stall), .clrn(reset_n), .d(id_vectorToScalarQ_wrreq & vpu_instr_decode_valid), .q(s3_vectorToScalarQ_wrreq), .sclr(vpu_decodestage_stall));
ff_sclr	#(.WIDTH(1))	stage3_instr_stalls_pipeline (.clk(clk), .en((vpu_instr_decode_valid & ~vpu_executestage_stall) | vpu_decodestage_stall), .clrn(reset_n), .d(id_instr_stalls_pipeline), .q(s3_instr_stalls_pipeline), .sclr(vpu_decodestage_stall));

// Register file offset; to handle multiple elements in a single RF memory
ff	#(.WIDTH(`log2(`ELEMPERLANE)))	stage3_vrfOffset (.clk(clk), .en(1'b1), .clrn(reset_n), .d(s2_vrfOffset), .q(s3_vrfOffset));


//---------------------------------------------------------------------------------------------------------------------------------------------------
//	Stage 4: Mem/Execute 2
//---------------------------------------------------------------------------------------------------------------------------------------------------
// register the opcode
ff_sclr	#(.WIDTH(32))	stage4_opCode (.clk(clk), .en(~vpu_memorystage_stall | vpu_executestage_stall), .clrn(reset_n), .d(s3_opCode), .q(s4_opCode), .sclr(vpu_executestage_stall));

// registers that need to be cleared on execute stage stall
ff_sclr	#(.WIDTH(1))	stage4_instr_stalls_pipeline (.clk(clk), .en(~vpu_memorystage_stall | vpu_executestage_stall), .clrn(reset_n), .d(s3_instr_stalls_pipeline), .q(s4_instr_stalls_pipeline_done), .sclr(vpu_executestage_stall));

// Register file offset; to handle multiple elements in a single RF memory
ff	#(.WIDTH(`log2(`ELEMPERLANE)))	stage4_vrfOffset (.clk(clk), .en(~vpu_memorystage_stall), .clrn(reset_n), .d(s3_vrfOffset), .q(s4_vrfOffset));


endmodule
