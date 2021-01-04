/*****************************************************************************
 *                                                                           *
 * Module:       Pipeline_S3_Regs                                            *
 * Description:                                                              *
 *      This module contains the execute stage pipeline registers.           *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Pipeline_S3_Regs the_Pipeline_S3_Regs (
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),

	.execute_stage_completed	(),
	
	.clr_adder_result_n			(),
	.clr_comparator_result_n	(),
	.clr_ctrl_regs_n			(),
	.clr_logic_result_n			(),
	.clr_npc_shifter_n			(),

	.s3_adder_result			(),
	.s3_cmp_result				(),
	.s3_ctrl_regs				(),
	.s3_load_store				(),
	.s3_logic_result			(),
	.s3_npc_shifter_result		(),
	.s3_opcode					(),
	.s3_reg_wr_addr				(),
	.s3_reg_wr_addr_loads		(),
	.s3_reg_wr_en				(),
	.s3_reg_wr_en_loads			(),
	.s3_store_data				(),

	// Bidirectionals

	// Outputs
	.s4_adder_result			(),
	.s4_cmp_result				(),
	.s4_ctrl_regs				(),
	.s4_load_store				(),
	.s4_logic_result			(),
	.s4_npc_shifter_result		(),
	.s4_opcode					(),
	.s4_reg_wr_addr				(),
	.s4_reg_wr_addr_loads		(),
	.s4_reg_wr_en				(),
	.s4_reg_wr_en_loads			(),
	.s4_store_data				()
);
*/

module Pipeline_S3_Regs (
	// Inputs
	clk,
	reset_n,

	execute_stage_completed,
	
	clr_adder_result_n,
	clr_comparator_result_n,
	clr_ctrl_regs_n,
	clr_logic_result_n,
	clr_npc_shifter_n,

	s3_adder_result,
	s3_cmp_result,
	s3_ctrl_regs,
	s3_load_store,
	s3_logic_result,
	s3_npc_shifter_result,
	s3_opcode,
	s3_reg_wr_addr,
	s3_reg_wr_addr_loads,
	s3_reg_wr_en,
	s3_reg_wr_en_loads,
	s3_store_data,
	
	s3_vector_cop_scalar_write,		// vector instruction needs to write to scalar reg

	// Bidirectionals

	// Outputs
	s4_adder_result,
	s4_cmp_result,
	s4_ctrl_regs,
	s4_load_store,
	s4_logic_result,
	s4_npc_shifter_result,
	s4_opcode,
	s4_reg_wr_addr,
	s4_reg_wr_addr_loads,
	s4_reg_wr_en,
	s4_reg_wr_en_loads,
	s4_store_data,
	
	s4_vector_cop_scalar_write
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/
// States

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input 			clk;
input			reset_n;

input			execute_stage_completed;

input			clr_adder_result_n;
input			clr_comparator_result_n;
input			clr_ctrl_regs_n;
input			clr_logic_result_n;
input			clr_npc_shifter_n;

input	[31:0]  s3_adder_result;
input			s3_cmp_result;
input	[31:0]	s3_ctrl_regs;
input			s3_load_store;
input	[31:0]  s3_logic_result;
input	[31:0]	s3_npc_shifter_result;
input	[5:0] 	s3_opcode;
input	[4:0]	s3_reg_wr_addr;
input	[4:0]	s3_reg_wr_addr_loads;
input		 	s3_reg_wr_en;
input			s3_reg_wr_en_loads;
input	[31:0]  s3_store_data;

input			s3_vector_cop_scalar_write;		// vector instruction needs to write to scalar reg

// Bidirectionals

// Outputs
output	[31:0]  s4_adder_result;
output			s4_cmp_result;
output	[31:0]	s4_ctrl_regs;
output			s4_load_store;
output	[31:0]  s4_logic_result;
output	[31:0]	s4_npc_shifter_result;
output	[5:0] 	s4_opcode;
output	[4:0]	s4_reg_wr_addr;
output	[4:0]	s4_reg_wr_addr_loads;
output		 	s4_reg_wr_en;
output			s4_reg_wr_en_loads;
output	[31:0]  s4_store_data;

output			s4_vector_cop_scalar_write;		// vector instruction needs to write to scalar reg

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires

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

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Pipeline_Reg Pipeline_Reg_Adder_Result (
	// inputs
	.clk			(clk),
	.clk_en			(execute_stage_completed),
	.reset_n		(reset_n & (clr_adder_result_n | ~execute_stage_completed)),

	.input_signal	(s3_adder_result),

	// outputs
	.output_signal	(s4_adder_result)
);
defparam Pipeline_Reg_Adder_Result.REG_SIZE = 32;


Pipeline_Reg Pipeline_Reg_Cmp_Result (
	// inputs
	.clk			(clk),
	.clk_en			(execute_stage_completed),
	.reset_n		(reset_n & (clr_comparator_result_n | ~execute_stage_completed)),

	.input_signal	(s3_cmp_result),

	// outputs
	.output_signal	(s4_cmp_result)
);
defparam Pipeline_Reg_Cmp_Result.REG_SIZE = 1;


Pipeline_Reg Pipeline_Reg_Ctrl_Regs (
	// inputs
	.clk			(clk),
	.clk_en			(execute_stage_completed),
	.reset_n		(reset_n & (clr_ctrl_regs_n | ~execute_stage_completed)),

	.input_signal	(s3_ctrl_regs),

	// outputs
	.output_signal	(s4_ctrl_regs)
);
defparam Pipeline_Reg_Ctrl_Regs.REG_SIZE = 32;


Pipeline_Reg Pipeline_Reg_Load_Store (
	// inputs
	.clk			(clk),
	.clk_en			(execute_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s3_load_store),

	// outputs
	.output_signal	(s4_load_store)
);
defparam Pipeline_Reg_Load_Store.REG_SIZE = 1;


Pipeline_Reg Pipeline_Reg_Logic_Result (
	// inputs
	.clk			(clk),
	.clk_en			(execute_stage_completed),
	.reset_n		(reset_n & (clr_logic_result_n | ~execute_stage_completed)),

	.input_signal	(s3_logic_result),

	// outputs
	.output_signal	(s4_logic_result)
);
defparam Pipeline_Reg_Logic_Result.REG_SIZE = 32;


Pipeline_Reg Pipeline_Reg_NPC_Shifter_Result (
	// inputs
	.clk			(clk),
	.clk_en			(execute_stage_completed),
	.reset_n		(reset_n & (clr_npc_shifter_n | ~execute_stage_completed)),

	.input_signal	(s3_npc_shifter_result),

	// outputs
	.output_signal	(s4_npc_shifter_result)
);
defparam Pipeline_Reg_NPC_Shifter_Result.REG_SIZE = 32;


Pipeline_Reg Pipeline_Reg_Opcode (
	// inputs
	.clk			(clk),
	.clk_en			(execute_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s3_opcode),

	// outputs
	.output_signal	(s4_opcode)
);
defparam Pipeline_Reg_Opcode.REG_SIZE = 6;


Pipeline_Reg Pipeline_Reg_Reg_Wr_Addr (
	// inputs
	.clk			(clk),
	.clk_en			(execute_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s3_reg_wr_addr),

	// outputs
	.output_signal	(s4_reg_wr_addr)
);
defparam Pipeline_Reg_Reg_Wr_Addr.REG_SIZE = 5;


Pipeline_Reg Pipeline_Reg_Reg_Wr_Addr_Loads (
	// inputs
	.clk			(clk),
	.clk_en			(execute_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s3_reg_wr_addr_loads),

	// outputs
	.output_signal	(s4_reg_wr_addr_loads)
);
defparam Pipeline_Reg_Reg_Wr_Addr_Loads.REG_SIZE = 5;


Pipeline_Reg Pipeline_Reg_Wr_En_Reg (
	// inputs
	.clk			(clk),
	.clk_en			(execute_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s3_reg_wr_en),

	// outputs
	.output_signal	(s4_reg_wr_en)
);
defparam Pipeline_Reg_Wr_En_Reg.REG_SIZE = 1;


Pipeline_Reg Pipeline_Reg_Wr_En_Reg_Loads (
	// inputs
	.clk			(clk),
	.clk_en			(execute_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s3_reg_wr_en_loads),

	// outputs
	.output_signal	(s4_reg_wr_en_loads)
);
defparam Pipeline_Reg_Wr_En_Reg_Loads.REG_SIZE = 1;


Pipeline_Reg Pipeline_Reg_Store_Data (
	// inputs
	.clk			(clk),
	.clk_en			(execute_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s3_store_data),

	// outputs
	.output_signal	(s4_store_data)
);
defparam Pipeline_Reg_Store_Data.REG_SIZE = 32;


Pipeline_Reg Pipeline_Reg_Vector_Cop_Scalar_Write (
	// inputs
	.clk			(clk),
	.clk_en			(execute_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s3_vector_cop_scalar_write),

	// outputs
	.output_signal	(s4_vector_cop_scalar_write)
);
defparam Pipeline_Reg_Vector_Cop_Scalar_Write.REG_SIZE = 1;

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

