/*****************************************************************************
 *                                                                           *
 * Module:       Pipeline_S2_Regs                                            *
 * Description:                                                              *
 *      This module contains the decode stage pipeline registers.            *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Pipeline_S2_Regs the_Pipeline_S2_Regs (
	// Inputs
	.clk					(clk),
	.reset_n				(reset_n),

	.fetch_stage_completed	(),
	.decode_stage_completed	(),
	
	.s2_branch_offset		(),
	.s2_decoded_signals		(),
	.s2_ir					(),
	.s2_is_rtype			(),
	.s2_opA					(),
	.s2_opB					(),
	.s2_opcode				(),
	.s2_rB					(),
	.s2_reg_wr_addr			(),

	// Bidirectionals

	// Outputs
	.s3_branch_offset		(),
	.s3_decoded_signals		(),
	.s3_ir					(),
	.s3_is_rtype			(),
	.s3_opA					(),
	.s3_opB					(),
	.s3_opcode				(),
	.s3_rB					(),
	.s3_reg_wr_addr			()
);
*/

module Pipeline_S2_Regs (
	// Inputs
	clk,
	reset_n,

	fetch_stage_completed,
	decode_stage_completed,
	
	s2_vector_cop_scalar_read,
	// vector_cop_scalar_skip,
	s2_vector_cop_scalar_write,

	s2_branch_offset,
	s2_decoded_signals,
	s2_ir,
	s2_is_rtype,
	s2_opA,
	s2_opB,
	s2_opcode,
	s2_rB,
	s2_reg_wr_addr,

	// Bidirectionals

	// Outputs
	s3_branch_offset,
	s3_decoded_signals,
	s3_ir,
	s3_is_rtype,
	s3_opA,
	s3_opB,
	s3_opcode,
	s3_rB,
	s3_reg_wr_addr,
	s3_vector_cop_scalar_write,
	s3_vector_cop_scalar_read
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input			clk;
input			reset_n;

input			fetch_stage_completed;
input			decode_stage_completed;

input			s2_vector_cop_scalar_read;			// vector COP instruction read from scalar core
// input			vector_cop_scalar_skip;			// vector COP instruction only, scalar core can ignore it
input			s2_vector_cop_scalar_write;		// vector instruction needs to write to scalar reg

input	[15:0]	s2_branch_offset;
input	[7:0]	s2_decoded_signals;
input	[31:0]	s2_ir;
input			s2_is_rtype;
input	[31:0]	s2_opA;
input	[31:0]	s2_opB;
input	[5:0]	s2_opcode;
input	[31:0]	s2_rB;
input	[4:0]	s2_reg_wr_addr;

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

output			s3_vector_cop_scalar_write;		// vector instruction needs to write to scalar reg
output			s3_vector_cop_scalar_read;			// vector COP instruction read from scalar core

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

// vector_cop_scalar_skip is handled directly at the program_counter
Pipeline_Reg Pipeline_Reg_Branch_Offset (
	// Inputs
	.clk			(clk),
	.clk_en			(decode_stage_completed),
	// .reset_n		(reset_n & ~fetch_stage_completed & ~vector_cop_scalar_skip),
	.reset_n		(reset_n & ~fetch_stage_completed),
	// .sclr			(fetch_stage_completed),
	// .aclr			(~reset_n | vector_cop_scalar_skip),

	.input_signal	(s2_branch_offset),

	// Bidirectionals

	// Outputs
	.output_signal	(s3_branch_offset)
);
defparam Pipeline_Reg_Branch_Offset.REG_SIZE = 16;


Pipeline_Reg Pipeline_Reg_Decoded_Signals (
	// Inputs
	.clk			(clk),
	.clk_en			(decode_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s2_decoded_signals),

	// Bidirectionals

	// Outputs
	.output_signal	(s3_decoded_signals)
);
defparam Pipeline_Reg_Decoded_Signals.REG_SIZE = 8;


Pipeline_Reg Pipeline_Reg_IR (
	// Inputs
	.clk			(clk),
	.clk_en			(decode_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s2_ir),

	// Bidirectionals

	// Outputs
	.output_signal	(s3_ir)
);
defparam Pipeline_Reg_IR.REG_SIZE = 32;


Pipeline_Reg Pipeline_Reg_Is_Rtype (
	// Inputs
	.clk			(clk),
	.clk_en			(decode_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s2_is_rtype),

	// Bidirectionals

	// Outputs
	.output_signal	(s3_is_rtype)
);
defparam Pipeline_Reg_Is_Rtype.REG_SIZE = 1;


Pipeline_Reg Pipeline_Reg_OpA (
	// Inputs
	.clk			(clk),
	.clk_en			(decode_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s2_opA),

	// Bidirectionals

	// Outputs
	.output_signal	(s3_opA)
);
defparam Pipeline_Reg_OpA.REG_SIZE = 32;


Pipeline_Reg Pipeline_Reg_OpB (
	// Inputs
	.clk			(clk),
	.clk_en			(decode_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s2_opB),

	// Bidirectionals

	// Outputs
	.output_signal	(s3_opB)
);
defparam Pipeline_Reg_OpB.REG_SIZE = 32;


Pipeline_Reg Pipeline_Reg_Opcode (
	// Inputs
	.clk			(clk),
	.clk_en			(decode_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s2_opcode),

	// Bidirectionals

	// Outputs
	.output_signal	(s3_opcode)
);
defparam Pipeline_Reg_Opcode.REG_SIZE = 6;


Pipeline_Reg Pipeline_Reg_rB (
	// Inputs
	.clk			(clk),
	.clk_en			(decode_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s2_rB),

	// Bidirectionals

	// Outputs
	.output_signal	(s3_rB)
);
defparam Pipeline_Reg_rB.REG_SIZE = 32;


Pipeline_Reg Pipeline_Reg_reg_wr_addr (
	// Inputs
	.clk			(clk),
	.clk_en			(decode_stage_completed),
	.reset_n		(reset_n),

	.input_signal	(s2_reg_wr_addr),

	// Bidirectionals

	// Outputs
	.output_signal	(s3_reg_wr_addr)
);
defparam Pipeline_Reg_reg_wr_addr.REG_SIZE = 5;


Pipeline_Reg
#(	.REG_SIZE(1)	)
Pipeline_Reg_vector_cop_scalar_read (
	// Inputs
	.clk			(clk),
	.clk_en			(decode_stage_completed),
	.reset_n		(reset_n),
	
	.input_signal	(s2_vector_cop_scalar_read),
	.output_signal	(s3_vector_cop_scalar_read)
);

Pipeline_Reg
#(	.REG_SIZE(1)	)
Pipeline_Reg_vector_cop_scalar_write (
	// Inputs
	.clk			(clk),
	.clk_en			(decode_stage_completed),
	.reset_n		(reset_n),
	
	.input_signal	(s2_vector_cop_scalar_write),
	.output_signal	(s3_vector_cop_scalar_write)
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

