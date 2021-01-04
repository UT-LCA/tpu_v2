/*****************************************************************************
 *                                                                           *
 * Module:       Datapath                                                    *
 * Description:                                                              *
 *      This module is the datapath of the UT IIe processor.                 *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Datapath the_Datapath (
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),

	.executing_fetch_stage		(),
	.executing_decode_stage		(),
	.executing_execute_stage	(),
	.executing_write_back_stage	(),
	.executing_irq_stage		(),
	.pipeline_stalled			(),

	.d_irq						(),
	.d_readdata					(),
	.d_waitrequest				(),

	.i_readdata					(),
	.i_waitrequest				(),

	// Bidirectionals

	// Outputs
	.d_address					(),
	.d_byteenable				(),
	.d_read						(),
	.d_write					(),
	.d_writedata				(),

	.data_master_busy			(),

	.fetch_unit_has_valid_data	(),

	.i_address					(),
	.i_read						(),

	.shifter_done_flag			()
);
defparam the_Datapath.RESET_ADDRESS		= RESET_ADDRESS;
defparam the_Datapath.BREAK_ADDRESS		= BREAK_ADDRESS;
defparam the_Datapath.EXCEPTION_ADDRESS	= EXCEPTION_ADDRESS;
defparam the_Datapath.CPU_ID			= CPU_ID;
*/

module Datapath (
	// Inputs
	clk,						// Global Clock
	reset_n,					// if 0, the cpu is being reset, therefore 
								//	 the all registers should be reset

	executing_fetch_stage,		// Fetch stage is active if 1
	executing_decode_stage,		// Decode stage is active if 1
	executing_execute_stage,	// Execute stage is active if 1
	executing_write_back_stage,	// Write Back stage is active if 1
	executing_irq_stage,		// Interrupt stage is active if 1
	pipeline_stalled,			// Pipeline has stalled if 1

	d_irq,						// Interrupt wires
	d_readdata,					// The data comming in from the data master
	d_waitrequest,				// Data master requests wait

	i_readdata,					// Data read from the memory
	i_waitrequest,				// 1 indicates that the fetch has not
								//   completed, and the read address and
								//	 read control signal must stay
								//	 constant

	vectorToScalarQ_q,			// vector to scalar core transfer queue output
								
	// Bidirectionals

	// Outputs
	d_address,					// The address for the data master
	d_byteenable,				// The byte enable for the data master
	d_read,						// The read enable for the data master
	d_write,					// The write enable for the data master
	d_writedata,				// Data to be written to the data master
	d_data_size,				// size of data to be writte to data master: 00 = byte, 01 = halfword, 10 = word

	data_master_busy,			// There is a load or store instruction  
								//	 in the mem stage

	fetch_unit_has_valid_data,	// 1 indicates that the fetch has completed

	i_address,					// Address of the instruction to fetch			
	i_read,						// 1 indicates memory read

	interrupt_active,			// if 1, external interrupt is active
	shifter_done_flag,			// if 1, Shift has completed
	
	// vector COP output to scalar core
	vector_cop_scalar_skip,		// vector COP instruction only, scalar core can skip it
	
	// Output to the vector core
	vector_cop_instruction,		// a vector coprocessor instruction has been fetched
	vector_opcode,				// opcode to vector core
	scalarRegOut,				// connection from scalar register to vector register
	s3_vector_cop_scalar_read,		// the vector COP instruction needs to read from scalar register
	scalarToVectorQ_wrreq,		// write enable from scalar register to vector data queue
	s3_vector_cop_scalar_write,	// vector instruction writes to scalar reg
	s3_vectorToScalarQ_rdreq,	// vector to scalar transfer queue rdreq
	predecode_scalarmem_instr,		// predecode scalar memory instruction
	predecode_vectormem_instr		// predecode vector memory instruction

);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter RESET_ADDRESS		= 32'h00000000;
parameter BREAK_ADDRESS		= 32'h00000000;
parameter EXCEPTION_ADDRESS	= 32'h00000020;
parameter CPU_ID			= 32'h00000000;

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

input	[31:0]	d_irq;
input	[31:0]	d_readdata;
input			d_waitrequest;

input	[31:0]	i_readdata;
input			i_waitrequest;

// vector to scalar core transfer queue
input	[31:0]	vectorToScalarQ_q;

// Bidirectionals

// Outputs
output	[31:0]	d_address;
output	[3:0]	d_byteenable;
output			d_read;
output			d_write;
output	[31:0]	d_writedata;
output	[1:0]	d_data_size;

output			data_master_busy;

output			fetch_unit_has_valid_data;

output	[31:0]	i_address;
output			i_read;

output			interrupt_active;
output			shifter_done_flag;

// Output to the vector core
output			vector_cop_instruction;		// it is a vector coprocessor instruction
output			vector_cop_scalar_skip;		// vector COP instruction only, scalar core can skip it
output	[31:0]	vector_opcode;		// opcode to vector core
output	[31:0]	scalarRegOut;		// connection from scalar register to vector register
output			s3_vector_cop_scalar_read;		// the vector COP instruction needs to read from scalar register
output			scalarToVectorQ_wrreq;		// write enable from scalar register to vector data queue
output			s3_vector_cop_scalar_write;	// vector instruction writes to scalar reg
output			s3_vectorToScalarQ_rdreq;	// vector to scalar transfer queue rdreq
output			predecode_scalarmem_instr;		// predecode scalar memory instruction
output			predecode_vectormem_instr;		// predecode vector memory instruction


/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire			reg_wr_en;
wire			reg_wr_en_loads;

// Stage One Internal Outputs
wire	[31:0]	fetched_instruction;

wire			s2_vector_cop_scalar_read;
wire			s2_vector_cop_scalar_write;	// vector instruction needs to write to scalar reg
wire	[31:0]	s2_pc;
wire	[31:0]	s3_pc_plus_4;

// Stage Two Internal Outputs
wire	[15:0]	s3_branch_offset;
wire	[7:0]	s3_decoded_signals;
wire	[31:0]	s3_ir;
wire			s3_is_rtype;
wire	[31:0]	s3_opA;
wire	[31:0]	s3_opB;
wire	[5:0]	s3_opcode;
wire	[31:0]	s3_rB;
wire	[4:0]	s3_reg_wr_addr;

// Stage Three Internal Outputs
wire			s3_branch_taken;
wire			s3_branch_type;

wire	[31:0]	s4_adder_result;
wire 			s4_cmp_result;
wire	[31:0]	s4_ctrl_regs;
wire			s4_load_store;
wire	[31:0]	s4_logic_result;
wire	[31:0]	s4_npc_shifter;
wire	[5:0]	s4_opcode;
wire	[4:0]	s4_reg_wr_addr;
wire	[4:0]	s4_reg_wr_addr_loads;
wire			s4_reg_wr_en;
wire			s4_reg_wr_en_loads;
wire	[31:0]	s4_store_data;
wire			s4_vector_cop_scalar_write;

// Stage Four Internal Outputs
wire	[31:0]	s4_reg_wr_data;
wire	[31:0]	s4_reg_wr_data_loads;

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

assign reg_wr_en		= 
		executing_write_back_stage/* & ~pipeline_stalled*/ & s4_reg_wr_en;
assign reg_wr_en_loads	= 
		executing_write_back_stage/* & ~pipeline_stalled*/ & s4_reg_wr_en_loads;

assign	scalarRegOut = s3_opA;		// Connect register output to vector core

// do not use fetched_instruction here because fetched_instruction can be overridden
// by Instruction_Fetch_Unit for scalar only and scalar-vector instructions
assign	vector_opcode = i_readdata;

// Vector COP read from scalar core, write value from scalar register to the scalarToVector queue
// Only enable the wrreq signal for one cycle, unless pipeline is stalled by the queue being full
assign	scalarToVectorQ_wrreq = s3_vector_cop_scalar_read & executing_execute_stage;

		
/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

// Stage 1 Fetch
Stage_One Stage_One (
	// Inputs
	.clk							(clk),
	.reset_n						(reset_n),

	.branch_offset					(s3_branch_offset),
	.branch_taken					(s3_branch_taken),
	.branch_type					(s3_branch_type),
	.jmp_addr						(s3_opA),
	
	.executing_fetch_stage			(executing_fetch_stage),
	.executing_decode_stage			(executing_decode_stage),
	.executing_execute_stage		(executing_execute_stage),
	.executing_write_back_stage		(executing_write_back_stage),
	.executing_irq_stage			(executing_irq_stage),
	.pipeline_stalled				(pipeline_stalled),

	.data_master_stalled			(data_master_busy),

	.instruction_master_data_in		(i_readdata),
	.instruction_master_stalled		(i_waitrequest),

	// Bidirectionals

	// Outputs
	.fetch_completed				(fetch_unit_has_valid_data),
	.fetched_instruction			(fetched_instruction),
	.vector_cop_instruction			(vector_cop_instruction),		// it is a vector coprocessor instruction
	.vector_cop_scalar_skip			(vector_cop_scalar_skip),		// vector COP instruction only, scalar core can skip it
	.s2_vector_cop_scalar_read		(s2_vector_cop_scalar_read),		// the vector COP instruction needs to read from scalar register
	.s2_vector_cop_scalar_write		(s2_vector_cop_scalar_write),		// vector instruction needs to write to scalar reg
	.predecode_scalarmem_instr		(predecode_scalarmem_instr),		// predecode scalar memory instruction
	.predecode_vectormem_instr		(predecode_vectormem_instr),		// predecode vector memory instruction

	.memory_address_to_read			(i_address),
	.read_memory					(i_read),
	
	.s2_pc							(s2_pc),
	.s3_pc_plus_4					(s3_pc_plus_4)
);
defparam Stage_One.RESET_ADDRESS = RESET_ADDRESS;




// Stage 2	Decode
Stage_Two Stage_Two (
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),

	.executing_fetch_stage		(executing_fetch_stage),
	.executing_decode_stage		(executing_decode_stage),
	.executing_execute_stage	(executing_execute_stage),
	.executing_write_back_stage	(executing_write_back_stage),
	.executing_irq_stage		(executing_irq_stage),
	.pipeline_stalled			(pipeline_stalled),

	.fetched_instruction		(fetched_instruction),
	.s2_vector_cop_scalar_read	(s2_vector_cop_scalar_read),
	.s2_vector_cop_scalar_write	(s2_vector_cop_scalar_write),
	.vector_cop_scalar_skip		(vector_cop_scalar_skip),
	
	.program_counter			(s2_pc),

	.reg_wr_addr				(s4_reg_wr_addr),
	.reg_wr_addr_loads			(s4_reg_wr_addr_loads),
	.reg_wr_data				(s4_reg_wr_data),
	.reg_wr_data_loads			(s4_reg_wr_data_loads),
	.reg_wr_en					(reg_wr_en),
	.reg_wr_en_loads			(reg_wr_en_loads),

	// Bidirectionals

	// Outputs
	.s3_branch_offset			(s3_branch_offset),
	.s3_decoded_signals			(s3_decoded_signals),
	.s3_ir						(s3_ir),
	.s3_is_rtype				(s3_is_rtype),
	.s3_opA						(s3_opA),
	.s3_opB						(s3_opB),
	.s3_opcode					(s3_opcode),
	.s3_rB						(s3_rB),
	.s3_reg_wr_addr				(s3_reg_wr_addr),
	.s3_vector_cop_scalar_read	(s3_vector_cop_scalar_read),
	.s3_vector_cop_scalar_write	(s3_vector_cop_scalar_write)		// vector instruction needs to write to scalar reg
);
defparam Stage_Two.BREAK_ADDRESS		= BREAK_ADDRESS;
defparam Stage_Two.EXCEPTION_ADDRESS	= EXCEPTION_ADDRESS;




// Stage 3	Execute
Stage_Three Stage_Three (
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),

	.executing_fetch_stage		(executing_fetch_stage),
	.executing_decode_stage		(executing_decode_stage),
	.executing_execute_stage	(executing_execute_stage),
	.executing_write_back_stage	(executing_write_back_stage),
	.executing_irq_stage		(executing_irq_stage),
	.pipeline_stalled			(pipeline_stalled),

	.d_irq						(d_irq),
	.s3_decoded_signals			(s3_decoded_signals),
	.s3_ir						(s3_ir),
	.s3_is_rtype				(s3_is_rtype),
	.s3_opA						(s3_opA),
	.s3_opB						(s3_opB),
	.s3_opcode					(s3_opcode),
	.s3_pc_plus_4				(s3_pc_plus_4),
	.s3_rB						(s3_rB),
	.s3_reg_wr_addr				(s3_reg_wr_addr),
	.s3_vector_cop_scalar_read	(s3_vector_cop_scalar_read),
	.s3_vector_cop_scalar_write	(s3_vector_cop_scalar_write),

	// Bidirectionals

	// Outputs
	.s3_branch_taken			(s3_branch_taken),
	.s3_branch_type				(s3_branch_type),
	.s3_vectorToScalarQ_rdreq	(s3_vectorToScalarQ_rdreq),

	.s4_adder_result			(s4_adder_result),
	.s4_cmp_result				(s4_cmp_result),
	.s4_ctrl_regs				(s4_ctrl_regs),
	.s4_load_store				(s4_load_store),
	.s4_logic_result			(s4_logic_result),
	.s4_npc_shifter				(s4_npc_shifter),
	.s4_opcode					(s4_opcode),
	.s4_reg_wr_addr				(s4_reg_wr_addr),
	.s4_reg_wr_addr_loads		(s4_reg_wr_addr_loads),
	.s4_reg_wr_en				(s4_reg_wr_en),
	.s4_reg_wr_en_loads			(s4_reg_wr_en_loads),
	.s4_store_data				(s4_store_data),

	.interrupt_active			(interrupt_active),
	.shifter_done_flag			(shifter_done_flag),
	
	.s4_vector_cop_scalar_write	(s4_vector_cop_scalar_write)
);
defparam Stage_Three.CPU_ID = CPU_ID;




// Stage 4	Memory and Writeback
Stage_Four Stage_Four (
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),

	.executing_fetch_stage		(executing_fetch_stage),
	.executing_decode_stage		(executing_decode_stage),
	.executing_execute_stage	(executing_execute_stage),
	.executing_write_back_stage	(executing_write_back_stage),
	.executing_irq_stage		(executing_irq_stage),
	.pipeline_stalled			(pipeline_stalled),

	.d_readdata					(d_readdata),
	.d_waitrequest				(d_waitrequest),

	.s4_adder_result			(s4_adder_result),
	.s4_cmp_result				(s4_cmp_result),
	.s4_ctrl_regs				(s4_ctrl_regs),
	.s4_load_store				(s4_load_store),
	.s4_logic_result			(s4_logic_result),
	.s4_npc_shifter				(s4_npc_shifter),
	.s4_opcode					(s4_opcode),
	.s4_store_data				(s4_store_data),
	
	.s4_vector_cop_scalar_write	(s4_vector_cop_scalar_write),
	.vectorToScalarQ_q			(vectorToScalarQ_q),
	
	// Bidirectionals

	// Outputs
	.d_address					(d_address),
	.d_byteenable				(d_byteenable),
	.d_read						(d_read),
	.d_write					(d_write),
	.d_writedata				(d_writedata),
	.d_data_size				(d_data_size),

	.s4_reg_wr_data				(s4_reg_wr_data),
	.s4_reg_wr_data_loads		(s4_reg_wr_data_loads),

	.data_master_busy			(data_master_busy)
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

