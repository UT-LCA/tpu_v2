/*****************************************************************************
 *                                                                           *
 * Module:       Stage_One                                                   *
 * Description:                                                              *
 *      This module is the 1st stage (fetch) of the UT IIe processor.        *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Stage_One Stage_One (
	// Inputs
	.clk							(clk),
	.reset_n						(reset_n),

	.branch_offset					(),
	.branch_taken					(),
	.branch_type					(),
	.jmp_addr						(),
	
	.executing_fetch_stage			(),
	.executing_decode_stage			(),
	.executing_execute_stage		(),
	.executing_write_back_stage		(),
	.executing_irq_stage			(),
	.pipeline_stalled				(),

	.data_master_stalled			(),

	.instruction_master_data_in		(),
	.instruction_master_stalled		(),

	// Bidirectionals

	// Outputs
	.fetch_completed				(),
	.fetched_instruction			(),

	.memory_address_to_read			(),
	.read_memory					(),
	
	.s2_pc							(),
	.s3_pc_plus_4					()
);
defparam Stage_One.RESET_ADDRESS = RESET_ADDRESS;
*/

module Stage_One (
	// Inputs
	clk,							// Global Clock
	reset_n,						// if 0, the cpu is being reset

	branch_offset,					// Offset for branches
	branch_taken,					// 1 indicates that a branch has been taken 
									//   and the PC should be updated
	branch_type,					// Indicates the type of control flow 
									//   instruction: 0 - jmp, 1 - branch
	jmp_addr,						// Address to jmp to for none offset 
									//   control flow instructions
	
	executing_fetch_stage,			// Fetch stage is active if 1
	executing_decode_stage,			// Decode stage is active if 1
	executing_execute_stage,		// Execute stage is active if 1
	executing_write_back_stage,		// Write Back stage is active if 1
	executing_irq_stage,			// Interrupt stage is active if 1
	pipeline_stalled,				// Pipeline has stalled if 1

	data_master_stalled,			// if 1 load/store is executing, dont fetch

	instruction_master_data_in,		// Data return from the memory
	instruction_master_stalled,		// 1 indicates that the fetch has not
									//   completed, and the read address and
									//	 read control signal must stay
									//	 constant

	// Bidirectionals

	// Outputs
	fetch_completed,				// 1 indicates that the fetch has completed
	fetched_instruction,			// Fetched instruction
	vector_cop_instruction,			// a vector coprocessor instruction has been fetched
	vector_cop_scalar_skip,			// vector COP instruction only, scalar core can skip it
	s2_vector_cop_scalar_read,			// the vector COP instruction needs to read from scalar register
	s2_vector_cop_scalar_write,		// vector instruction needs to write to scalar reg
	predecode_scalarmem_instr,		// predecode scalar memory instruction
	predecode_vectormem_instr,		// predecode vector memory instruction

	memory_address_to_read,			// Address of the instruction to fetch			
	read_memory,					// 1 indicates memory read
	
	s2_pc,							// Contains the current value of the PC
	s3_pc_plus_4					// Contains the current value of the PC + 4
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter RESET_ADDRESS	= 32'h00000000;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/

// Inputs
input			clk;
input			reset_n;

input	[15:0]	branch_offset;
input			branch_taken;
input			branch_type;
input	[31:0]	jmp_addr;

input			executing_fetch_stage;
input			executing_decode_stage;
input			executing_execute_stage;
input			executing_write_back_stage;
input			executing_irq_stage;
input			pipeline_stalled;

input			data_master_stalled;

input	[31:0]	instruction_master_data_in;
input			instruction_master_stalled;

// Bidirectionals

// Outputs
output			fetch_completed;
output	[31:0]  fetched_instruction;
output			vector_cop_instruction;		// it is a vector coprocessor instruction
output			vector_cop_scalar_skip;		// vector COP instruction only, scalar core can skip it
output			s2_vector_cop_scalar_read;		// the vector COP instruction needs to read from scalar register
output			s2_vector_cop_scalar_write;		// vector instruction needs to write to scalar reg

output			predecode_scalarmem_instr;
output			predecode_vectormem_instr;

output 	[31:0]	memory_address_to_read;
output			read_memory;

output 	[31:0]	s2_pc;
output 	[31:0]	s3_pc_plus_4;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire			branch_stage;
wire			inc_pc;
wire			start_fetch;

wire			vector_cop_scalar_read;
wire			vector_cop_scalar_write;

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

assign s3_pc_plus_4 = s2_pc;

assign branch_stage = ~pipeline_stalled & executing_execute_stage;
// assign inc_pc		= ~pipeline_stalled & executing_decode_stage;
assign inc_pc		= ~pipeline_stalled & (executing_decode_stage | (executing_fetch_stage & vector_cop_scalar_skip));
// assign start_fetch  = executing_fetch_stage & ~executing_irq_stage;
assign start_fetch  = executing_fetch_stage & ~executing_irq_stage & reset_n;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Instruction_Fetch_Unit the_Fetch_Unit (
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),

	.executing_irq_stage		(executing_irq_stage),
	.pipeline_stalled			(pipeline_stalled),
	
	.branch_offset				(branch_offset),
	.branch_stage				(branch_stage),
	.branch_taken				(branch_taken),
	.branch_type				(branch_type),
	.jmp_addr					(jmp_addr),
	
	.data_in_from_memory		(instruction_master_data_in),
	.memory_requests_wait		(instruction_master_stalled),

	.initiate_instruction_fetch	(start_fetch),
	.increment_program_counter	(inc_pc),

	// Bidirectionals

	// Outputs
	.fetched_instruction		(fetched_instruction),
	.fetch_completed			(fetch_completed),
	.vector_cop_scalar_skip		(vector_cop_scalar_skip),
	.vector_cop_instruction		(vector_cop_instruction),		// it is a vector coprocessor instruction
	.vector_cop_scalar_read		(vector_cop_scalar_read),		// the vector COP instruction needs to read from scalar register
	.vector_cop_scalar_write	(vector_cop_scalar_write),		// vector instruction needs to write to scalar reg
	
	.memory_address_to_read		(memory_address_to_read),
	.read_memory				(read_memory),
	
	.program_counter			(s2_pc)
);
defparam the_Fetch_Unit.RESET_ADDRESS = RESET_ADDRESS;


Predecode_Instruction	the_Predecode_Instruction (
	// ***********
	// ** Inputs **
	// ***********
	.fetched_instruction		(fetched_instruction),
	.fetch_completed			(fetch_completed),
	.pipeline_stalled			(pipeline_stalled),

	.scalar_memory_instruction	(predecode_scalarmem_instr),
	.vector_cop_memory_instruction	(predecode_vectormem_instr)
);


/*****************************************************************************
 *                              Pipeline registers to stage 2                             *
 *****************************************************************************/
Pipeline_Reg
#(	.REG_SIZE(1)	)
Pipeline_Reg_vector_cop_scalar_read (
	// Inputs
	.clk			(clk),
	.clk_en			(fetch_completed),
	.reset_n		(reset_n),
	
	.input_signal	(vector_cop_scalar_read),
	.output_signal	(s2_vector_cop_scalar_read)
);

Pipeline_Reg
#(	.REG_SIZE(1)	)
Pipeline_Reg_vector_cop_scalar_write (
	// Inputs
	.clk			(clk),
	.clk_en			(fetch_completed),
	.reset_n		(reset_n),
	
	.input_signal	(vector_cop_scalar_write),
	.output_signal	(s2_vector_cop_scalar_write)
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

