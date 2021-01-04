//************************************************************************
// Scalable Vector CPU Project -- Predecode
// Filename: VPU_predecode.v
// Author: Jason Yu
//
// VPU predecode
// - determines whether the next instruction is a vector instruction, and what type
//
//  Copyright (C) 2007 Jason Yu
//
//************************************************************************

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on
`ifdef	MODELSIM
	`include "../hdl/define.v"
	`include "../hdl/isa_def.v"
`else
	`include "define.v"
	`include "isa_def.v"
`endif


module VPU_predecode
(
	input		[31:0]	opCode,
	output		predecode_vector_instr,	// predecoded a vector instruction
	output		predecode_load_instr,	// predecoded a load instruction
	output		predecode_store_instr,
	output		predecode_loadstore		// predecoded load or store instruction
);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
reg		func_is_loadinstr;				// predecoded instruction is a laod
reg		func_is_storeinstr;				// predecoded instruction is a store

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Signal assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------
// predecode whether the next instruction is a vector instruction, use 4 state comparison (===) to avoid 'x' in simulation
// VMSTC is excluded because the scalar core can go ahead and execute its portion of the instruction ahead of the vector core
assign	predecode_vector_instr = (opCode[`OP_OPCODE] === `OPCODE_VRTYPE) || (opCode[`OP_OPCODE] === `OPCODE_VMTYPE) || (opCode[`OP_OPCODE] === `OPCODE_MISCTYPE) ? 1'b1 : 1'b0;
assign	predecode_load_instr = (opCode[`OP_OPCODE] === `OPCODE_VMTYPE) && (func_is_loadinstr === 1'b1) ? 1'b1 : 1'b0;
assign	predecode_store_instr = (opCode[`OP_OPCODE] === `OPCODE_VMTYPE) && (func_is_storeinstr === 1'b1) ? 1'b1 : 1'b0;
assign	predecode_loadstore = (opCode[`OP_OPCODE] === `OPCODE_VMTYPE) && ((func_is_loadinstr === 1'b1) || (func_is_storeinstr === 1'b1)) ? 1'b1 : 1'b0;

// Decode the function field of the next instruction
always @(*) begin
	func_is_loadinstr = 1'b0;
	func_is_storeinstr = 1'b0;
	casex (opCode[`OP_FUNC])
		`FUNC_MEM_UNITLOAD,
		`FUNC_MEM_STRIDEDLOAD,
		`FUNC_MEM_INDEXEDLOAD:	func_is_loadinstr = 1'b1;
		`FUNC_MEM_UNITSTORE,
		`FUNC_MEM_STRIDEDSTORE,
		`FUNC_MEM_INDEXEDSTORE:	func_is_storeinstr = 1'b1;
		default:	;	// do nothing
	endcase
end
	
endmodule
