/*****************************************************************************
 *                                                                           *
 * Module:       Stage_Three                                                 *
 * Description:                                                              *
 *      This module is the 3rd stage (execute) of the UT IIe processor.      *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Stage_Three Stage_Three (
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
	.s3_decoded_signals			(),
	.s3_ir						(),
	.s3_is_rtype				(),
	.s3_opA						(),
	.s3_opB						(),
	.s3_opcode					(),
	.s3_pc_plus_4				(),
	.s3_rB						(),
	.s3_reg_wr_addr				(),

	// Bidirectionals

	// Outputs
	.s3_branch_taken			(),
	.s3_branch_type				(),

	.s4_adder_result			(),
	.s4_cmp_result				(),
	.s4_ctrl_regs				(),
	.s4_load_store				(),
	.s4_logic_result			(),
	.s4_npc_shifter				(),
	.s4_opcode					(),
	.s4_reg_wr_addr				(),
	.s4_reg_wr_addr_loads		(),
	.s4_reg_wr_en				(),
	.s4_reg_wr_en_loads			(),
	.s4_store_data				(),

	.interrupt_active			(),
	.shifter_done_flag			()
);
defparam Stage_Three.CPU_ID = CPU_ID;
*/

module Stage_Three (
	// Inputs
	clk,							// Global Clock
	reset_n,						// if 0, the cpu is being reset, therefore 
									//	 the all registers should be reset

	executing_fetch_stage,			// Fetch stage is active if 1
	executing_decode_stage,			// Decode stage is active if 1
	executing_execute_stage,		// Execute stage is active if 1
	executing_write_back_stage,		// Write Back stage is active if 1
	executing_irq_stage,			// Interrupt stage is active if 1
	pipeline_stalled,				// Pipeline has stalled if 1

	d_irq,							// Interrupt wires
	s3_decoded_signals,				// Decoder Signals 
	s3_ir,							// Current Instruction
	s3_is_rtype,					// Instruction is Rtype
	s3_opA,							// Operand A
	s3_opB,							// Operand B
	s3_opcode,						// Opcode of the current instruction
	s3_pc_plus_4,					// Program Counter plus 4
	s3_rB,							// Register B data
	s3_reg_wr_addr,					// Address for writing the result of the 
									//	 current instruction to the Reg File
	s3_vector_cop_scalar_read,		// vector COP instruction read from scalar register
	s3_vector_cop_scalar_write,		// vector instruction needs to write to scalar reg

	// Bidirectionals

	// Outputs
	s3_branch_taken,				// if 1, a branch is being taken
	s3_branch_type,					// 0 - jmp (opA),  
									// 1 - branch with offset (pc + 4 + offset)
	s3_vectorToScalarQ_rdreq,		// read from vector to scalar transfer queue

	s4_adder_result,				// The result of the adder
	s4_cmp_result,					// The result of a comparison operation
	s4_ctrl_regs,					// Control registers read data
	s4_load_store,					// 1, load or store instruction
	s4_logic_result,				// The result of the logic unit
	s4_npc_shifter,					// PC plus 4 or the result of the shifter
	s4_opcode,						// Opcode of the current instruction
	s4_reg_wr_addr,					// Address for writing the result of the 
									//	 current instruction to the Reg File
	s4_reg_wr_addr_loads,			// Address for writing the result of a 
									//	 load instruction to the Reg File
	s4_reg_wr_en,					// if 1, the intruction should write to 
									//	 port a of the register file
	s4_reg_wr_en_loads,				// if 1, the intruction should write to 
									//	 port b of the register file
	s4_store_data,					// Data to be written to the memory

	interrupt_active,				// 1 indicates an external interrupt
	shifter_done_flag,				// if 1, Shift has completed
	
	s4_vector_cop_scalar_write		// vector instruction needs to write to scalar reg
);
	
/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter CPU_ID = 32'h00000000;

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

input	[31:0] 	d_irq;
input	[7:0]	s3_decoded_signals;
input	[31:0] 	s3_ir;
input			s3_is_rtype;
input	[31:0] 	s3_opA;
input	[31:0] 	s3_opB;
input	[5:0] 	s3_opcode;
input	[31:0] 	s3_pc_plus_4;
input	[31:0] 	s3_rB;
input	[4:0]	s3_reg_wr_addr;
input			s3_vector_cop_scalar_read;
input			s3_vector_cop_scalar_write;		// vector instruction needs to write to scalar reg

// Bidirectionals

// Outputs
output			s3_branch_taken;
output			s3_branch_type;
output			s3_vectorToScalarQ_rdreq;		// read from vector to scalar transfer queue

output	[31:0]  s4_adder_result;
output			s4_cmp_result;
output	[31:0]	s4_ctrl_regs;
output			s4_load_store;
output	[31:0]  s4_logic_result;
output	[31:0]	s4_npc_shifter;
output	[5:0] 	s4_opcode;
output	[4:0]	s4_reg_wr_addr;
output	[4:0]	s4_reg_wr_addr_loads;
output		 	s4_reg_wr_en;
output			s4_reg_wr_en_loads;
output	[31:0]  s4_store_data;

output			interrupt_active;
output			shifter_done_flag;

output			s4_vector_cop_scalar_write;		// vector instruction needs to write to scalar reg

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire			execute_stage_completed;

wire	[31:0]	adder_result;
wire 			cmp_flag;
wire	[31:0]	ctrl_regs;
wire	[31:0]	data_to_memory;
wire			load_store;
wire	[31:0]	logic_result;
wire	[31:0]	npc_shift_result;
wire			reg_wr_en;
wire			reg_wr_en_loads;
wire	[31:0]	shifter_result;
wire	[31:0]  store_data;
wire			use_adder_result;
wire			use_comparator_result;
wire			use_ctrl_regs;
wire			use_logic_result;
wire			use_npc_shift_result;
wire			use_shifter_result;

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

assign load_store = (s3_decoded_signals[3:0] == 4'h0);

assign execute_stage_completed	= executing_execute_stage & ~pipeline_stalled;

// read from vector to scalar transfer queue on a scalar write instruction; only assert the signal until the value is read
assign	s3_vectorToScalarQ_rdreq = s3_vector_cop_scalar_write & executing_execute_stage;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

ALU_Logic the_ALU (
	// Inputs
	.clk						(clk),

	.executing_execute_stage	(executing_execute_stage),

	.decoded_signals			(s3_decoded_signals),
	.is_rtype					(s3_is_rtype),
	.opA						(s3_opA),
	.opB						(s3_opB),
	.opcode						(s3_opcode),

	// Bidirectionals

	// Outputs
	.adder_result				(adder_result),
	.cmp_flag					(cmp_flag),
	.logic_result				(logic_result),
	.shifter_result				(shifter_result),

	.shifter_done_flag			(shifter_done_flag),

	.use_adder_result			(use_adder_result),
	.use_comparator_result		(use_comparator_result),
	.use_logic_result			(use_logic_result),
	.use_shifter_result			(use_shifter_result)
);

Branch_Logic the_Branch_Logic (
	// Inputs
	.cmp_flag			(cmp_flag),
	.decoded_signals	(s3_decoded_signals),
	.opcode				(s3_opcode),

	// Bidirectionals

	// Outputs
	.branch_taken		(s3_branch_taken),
	.branch_type		(s3_branch_type)
);

Control_Registers the_Control_Registers (
	// Inputs
	.clk				(clk),
	.reset_n			(reset_n),

	.allow_ctrl_regs_wr	(executing_execute_stage),
	.ctrl_reg_data_in	(s3_opA),
	.d_irq				(d_irq),
	.decoded_signals	(s3_decoded_signals),
	.n_from_ir			(s3_opB[4:0]),
	.opcode				(s3_opcode),
	.stall_pipeline		(pipeline_stalled),
	
	// Bidirectionals

	// Outputs
	.ctrl_reg_data_out	(ctrl_regs),
	.interrupt_active	(interrupt_active),
	.read_ctrl_regs		(use_ctrl_regs)
);
defparam the_Control_Registers.CPU_ID = CPU_ID;

Memory_Data_Out_Align the_Memory_Data_Out_Align (
	// Inputs
	.data_in	(s3_rB),
	.opcode		(s3_opcode),
	
	// Bidirectionals

	// Outputs
	.data_out	(store_data)
);

Next_PC_Shifter_Selection the_Next_PC_Shifter_Selector (
	// Inputs
	.decoded_signals			(s3_decoded_signals),
	.next_pc					(s3_pc_plus_4),
	.shifter_result				(shifter_result),
	.use_shifter_result			(use_shifter_result),
	
	// Bidirectionals

	// Outputs
	.next_pc_shifter_result		(npc_shift_result),
	.use_next_pc_shifter_result	(use_npc_shift_result)
);

Reg_Write_Enable the_Reg_Write_Enable (
	// Inputs
	.decoded_signals	(s3_decoded_signals),
	.opcode				(s3_opcode),
	.s3_vector_cop_scalar_read	(s3_vector_cop_scalar_read),
	.s3_vector_cop_scalar_write	(s3_vector_cop_scalar_write),
	
	// Bidirectionals

	// Outputs
	.reg_wr_en			(reg_wr_en),
	.reg_wr_en_loads	(reg_wr_en_loads)
);

Pipeline_S3_Regs the_Pipeline_S3_Regs (
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),

	.execute_stage_completed	(execute_stage_completed),
	
	.clr_adder_result_n			(use_adder_result),
	.clr_comparator_result_n	(use_comparator_result),
	.clr_ctrl_regs_n			(use_ctrl_regs),
	.clr_logic_result_n			(use_logic_result),
	.clr_npc_shifter_n			(use_npc_shift_result),

	.s3_adder_result			(adder_result),
	.s3_cmp_result				(cmp_flag),
	.s3_ctrl_regs				(ctrl_regs),
	.s3_load_store				(load_store),
	.s3_logic_result			(logic_result),
	.s3_npc_shifter_result		(npc_shift_result),
	.s3_opcode					(s3_opcode),
	.s3_reg_wr_addr				(s3_reg_wr_addr),
	.s3_reg_wr_addr_loads		(s3_reg_wr_addr),
	.s3_reg_wr_en				(reg_wr_en),
	.s3_reg_wr_en_loads			(load_store & s3_opcode[1]),
	.s3_store_data				(store_data),
	
	.s3_vector_cop_scalar_write	(s3_vector_cop_scalar_write),		// vector instruction needs to write to scalar reg

	// Bidirectionals

	// Outputs
	.s4_adder_result			(s4_adder_result),
	.s4_cmp_result				(s4_cmp_result),
	.s4_ctrl_regs				(s4_ctrl_regs),
	.s4_load_store				(s4_load_store),
	.s4_logic_result			(s4_logic_result),
	.s4_npc_shifter_result		(s4_npc_shifter),
	.s4_opcode					(s4_opcode),
	.s4_reg_wr_addr				(s4_reg_wr_addr),
	.s4_reg_wr_addr_loads		(s4_reg_wr_addr_loads),
	.s4_reg_wr_en				(s4_reg_wr_en),
	.s4_reg_wr_en_loads			(s4_reg_wr_en_loads),
	.s4_store_data				(s4_store_data),
	
	.s4_vector_cop_scalar_write	(s4_vector_cop_scalar_write)		// vector instruction needs to write to scalar reg
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

