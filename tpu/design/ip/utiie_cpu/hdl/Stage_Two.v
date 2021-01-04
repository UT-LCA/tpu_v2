/*****************************************************************************
 *                                                                           *
 * Module:       Stage_Two                                                   *
 * Description:                                                              *
 *      This module is the 2nd stage (decode) of the UT IIe processor.       *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on
`ifdef	MODELSIM
	`include "../../vect_cpu/hdl/isa_def.v"
`else
	`include "isa_def.v"
`endif


/*
Stage_Two Stage_Two (
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),

	.executing_fetch_stage		(),
	.executing_decode_stage		(),
	.executing_execute_stage	(),
	.executing_write_back_stage	(),
	.executing_irq_stage		(),
	.pipeline_stalled			(),

	.fetched_instruction		(),

	.program_counter			(),

	.reg_wr_addr				(),
	.reg_wr_addr_loads			(),
	.reg_wr_data				(),
	.reg_wr_data_loads			(),
	.reg_wr_en					(),
	.reg_wr_en_loads			(),

	// Bidirectionals

	// Outputs
	.s3_branch_offset			(),
	.s3_decoded_signals			(),
	.s3_ir						(),
	.s3_is_rtype				(),
	.s3_opA						(),
	.s3_opB						(),
	.s3_opcode					(),
	.s3_rB						(),
	.s3_reg_wr_addr				()
);
defparam Stage_Two.BREAK_ADDRESS		= BREAK_ADDRESS;
defparam Stage_Two.EXCEPTION_ADDRESS	= EXCEPTION_ADDRESS;
*/

module Stage_Two (
	// Inputs
	clk,						// Global Clock
	reset_n,					// if 0, the cpu is being reset, therefore 
								//	 the all registers should be reset

	executing_fetch_stage,		// Fetch stage is active if 1
	executing_decode_stage,		// Decode stage is active if 1
	executing_execute_stage,	// Execute stage is active if 1
	executing_write_back_stage,	// Write Back stage is active if 1
	executing_irq_stage,			// Interrupt stage is active if 1
	pipeline_stalled,			// Pipeline has stalled if 1

	fetched_instruction,		// Contains the new instruction from the 
								//	 fetch unit
	s2_vector_cop_scalar_read,		// whether it is a vector COP scalar read instruction
	vector_cop_scalar_skip,		// a vector only instruction that can be ignored by scalar processor
	s2_vector_cop_scalar_write,	// vector instruction needs to write to scalar reg

	program_counter,			// PC for the current instruction

	reg_wr_addr,				// Address of the register to be written
	reg_wr_addr_loads,			// Address of the register to be written 
								//	 for memory loads
	reg_wr_data,				// Data to be written to the register file
	reg_wr_data_loads,			// Data to be written to the register file
								//	 for memory loads
	reg_wr_en,					// Write enable for the register file portA
	reg_wr_en_loads,			// Write enable for the register file portB

	// Bidirectionals

	// Outputs
	s3_branch_offset,			// The offset for branch instructions
	s3_decoded_signals,			// Decoded information from the decoder unit
	s3_ir,						// The current instruction
	s3_is_rtype,				// Instruction is Rtype
	s3_opA,						// Contains the Operand A's data
	s3_opB,						// Contains the Operand B's data
	s3_opcode,					// Opcode of the current instruction
	s3_rB,						// The register B data
	s3_reg_wr_addr,				// Address for writing the result of the 
								//	 current instruction to the register file
	s3_vector_cop_scalar_read,	// vector COP instruction, read from scalar core
	s3_vector_cop_scalar_write	// vector instruction needs to write to scalar reg
);
	

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter BREAK_ADDRESS		= 32'h00000000;
parameter EXCEPTION_ADDRESS	= 32'h00000020;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input			clk;
input			reset_n;

input			executing_fetch_stage;
input			executing_decode_stage;
input			executing_execute_stage;
input			executing_write_back_stage;
input			executing_irq_stage;
input			pipeline_stalled;

input	[31:0] 	fetched_instruction;
input			s2_vector_cop_scalar_read;		// the vector COP instruction needs to read from scalar register
input			vector_cop_scalar_skip;		// a vector only instruction that can be ignored by scalar processor
input			s2_vector_cop_scalar_write;		// vector instruction needs to write to scalar reg
	
input	[31:0] 	program_counter;

input	[4:0] 	reg_wr_addr;
input	[4:0] 	reg_wr_addr_loads;
input	[31:0] 	reg_wr_data;
input	[31:0] 	reg_wr_data_loads;
input			reg_wr_en;
input			reg_wr_en_loads;

// Bidirectionals

// Outputs
output	[15:0]	s3_branch_offset;
output	[7:0]	s3_decoded_signals;
output	[31:0]	s3_ir;
output			s3_is_rtype;
output	[31:0]	s3_opA;
output	[31:0]	s3_opB;
output	[5:0]	s3_opcode;
output	[31:0]	s3_rB;
output	[4:0]	s3_reg_wr_addr;
output			s3_vector_cop_scalar_read;
output			s3_vector_cop_scalar_write;		// vector instruction needs to write to scalar reg

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire			fetch_stage_completed;
wire			decode_stage_completed;
wire			execute_stage_completed;

wire	[7:0]	decoded_signals;
wire	[31:0]	ir;
wire	[31:0]	opA;
wire	[31:0]	opB;
wire 	[31:0]	reg_data_rA;
wire 	[31:0]	reg_data_rB;
wire			rtype_instruction;
wire	[5:0]	s2_opcode;
wire	[4:0]	s2_reg_wr_addr;

wire	[4:0]	reg_a_rdaddr;

// Internal Registers

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

// assign fetch_stage_completed	= executing_fetch_stage & ~pipeline_stalled;
// Do not continue to move the instruction down the pipeline if it is a vector COP only instruction
assign fetch_stage_completed	= executing_fetch_stage & ~pipeline_stalled & ~vector_cop_scalar_skip;
assign decode_stage_completed	= executing_decode_stage & ~pipeline_stalled;
assign execute_stage_completed	= executing_execute_stage & ~pipeline_stalled;

assign	reg_a_rdaddr = fetched_instruction[31:27];
	

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Instr_Register the_Instr_Register (
	// Inputs
	.clk					(clk),
	.reset_n				(reset_n),

	.fetch_stage_completed	(fetch_stage_completed),
	.fetched_instruction	(fetched_instruction),
	.is_rtype				(rtype_instruction),

	// Bidirectionals

	// Outputs
	.ir						(ir),
	.opcode					(s2_opcode)
);

Decoder_Logic the_Instruction_Decoder (
	// Inputs
	.clk					(clk),
	.reset_n				(reset_n),

	.fetch_stage_completed	(fetch_stage_completed),
	.op_code				(fetched_instruction[5:0]),
	.opx_code				(fetched_instruction[16:11]),

	// Bidirectionals

	// Outputs
	.decoded_signals		(decoded_signals),
	.is_rtype				(rtype_instruction)
);


Register_File the_Register_File (
	// Inputs
	.clk					(clk),
	.reset_n				(reset_n),
	
	.fetch_stage_completed	(fetch_stage_completed),

	.port_a_data_in			(reg_wr_data),
	// .port_a_rd_addr			(fetched_instruction[31:27]),
	.port_a_rd_addr			(reg_a_rdaddr),
	.port_a_wr_addr			(reg_wr_addr),
	.port_a_wr_en			(reg_wr_en),

	.port_b_data_in			(reg_wr_data_loads),
	.port_b_rd_addr			(fetched_instruction[26:22]),
	.port_b_wr_addr			(reg_wr_addr_loads),
	.port_b_wr_en			(reg_wr_en_loads),

	// Bidirectionals

	// Outputs
	.port_a_data_out		(reg_data_rA),
	.port_b_data_out		(reg_data_rB)
);

Reg_Write_Addr_Mux the_Reg_Write_Addr_Mux (
	// Inputs
	.is_rtype		(rtype_instruction),
	.vector_cop_scalar_write	(s2_vector_cop_scalar_write),
	.op_code		(ir[5:0]),
	.rB_addr		(ir[26:22]),
	.rC_addr		(ir[21:17]),
	.vcop_rS_dest_addr	(ir[`OP_SCALAR_RFWRADDR]),

	// Bidirectionals

	// Outputs
	.reg_write_addr	(s2_reg_wr_addr)
);

OpA_Selection the_OpA_Selector (
	// Inputs
	.decoded_signals		(decoded_signals),
	.imm26					(ir[31:6]),
	.pc						(program_counter),
	.rA						(reg_data_rA),
	
	// Bidirectionals

	// Outputs
	.opA					(opA)
);
defparam the_OpA_Selector.BREAK_ADDRESS		= BREAK_ADDRESS;
defparam the_OpA_Selector.EXCEPTION_ADDRESS	= EXCEPTION_ADDRESS;


OpB_Imm_Selection the_OpB_Imm_Selector (
	// Inputs
	.decoded_signals	(decoded_signals),
	.imm16				(ir[21:6]),
	.rB					(reg_data_rB),

	// Bidirectionals

	// Outputs
	.opB				(opB)
);

Pipeline_S2_Regs the_Pipeline_S2_Regs (
	// Inputs
	.clk					(clk),
	.reset_n				(reset_n),

	// .executing_fetch_stage	(executing_fetch_stage),
	.fetch_stage_completed	(fetch_stage_completed),
	.decode_stage_completed	(decode_stage_completed),
	.s2_vector_cop_scalar_read	(s2_vector_cop_scalar_read),
	// .vector_cop_scalar_skip	(vector_cop_scalar_skip),
	.s2_vector_cop_scalar_write	(s2_vector_cop_scalar_write),

	.s2_branch_offset		(ir[21:6]),
	.s2_decoded_signals		(decoded_signals),
	.s2_ir					(ir),
	.s2_is_rtype			(rtype_instruction),
	.s2_opA					(opA),
	.s2_opB					(opB),
	.s2_opcode				(s2_opcode),
	.s2_rB					(reg_data_rB),
	.s2_reg_wr_addr			(s2_reg_wr_addr),

	// Bidirectionals

	// Outputs
	.s3_branch_offset		(s3_branch_offset),
	.s3_decoded_signals		(s3_decoded_signals),
	.s3_ir					(s3_ir),
	.s3_is_rtype			(s3_is_rtype),
	.s3_opA					(s3_opA),
	.s3_opB					(s3_opB),
	.s3_opcode				(s3_opcode),
	.s3_rB					(s3_rB),
	.s3_reg_wr_addr			(s3_reg_wr_addr),
	
	.s3_vector_cop_scalar_write	(s3_vector_cop_scalar_write),
	.s3_vector_cop_scalar_read	(s3_vector_cop_scalar_read)
);

endmodule

/*****************************************************************************
 *                                                                           *
 * Copyright (C) 2006 Blair Fort. All rights reserved.  Blair Fort assumes   *
 * no responsibility or liability arising out of the application or use of   *
 * any information, product, or service described herein except as expressly *
 * agreed to in writing by Blair Fort.                                       *
 *                                                                           *
 * This module is being provided on an "as-is" basis and as an accommodation *
 * and therefore all warranties, representations or guarantees of any kind   *
 * (whether express, implied or statutory) including, without limitation,    *
 * warranties of merchantability, non-infringement, or fitness for a         *
 * particular purpose, are specifically disclaimed.                          *
 *                                                                           *
 *****************************************************************************/

