/*****************************************************************************
 *                                                                           *
 * Module:       Reg_Write_Enable                                            *
 * Description:                                                              *
 *      This module calculates whether or not a register file write will     *
 *   occur for the current instruction.                                      *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Reg_Write_Enable the_Reg_Write_Enable (
	// Inputs
	.decoded_signals	(),
	.opcode				(),
	
	// Bidirectionals

	// Outputs
	.reg_wr_en			(),
	.reg_wr_en_loads	()
);
*/

module Reg_Write_Enable (
	// Inputs
	decoded_signals,
	opcode,
	s3_vector_cop_scalar_read,
	s3_vector_cop_scalar_write,
	
	// Bidirectionals

	// Outputs
	reg_wr_en,
	reg_wr_en_loads
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/
// States

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input	[7:0]	decoded_signals;
input	[5:0]	opcode;
input			s3_vector_cop_scalar_read;		// vector COP read from scalar register
input			s3_vector_cop_scalar_write;		// vector instruction needs to write to scalar reg

// Bidirectionals

// Outputs
output			reg_wr_en;
output			reg_wr_en_loads;

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

// assign reg_wr_en = 
		// ((decoded_signals[3] == 1'b0) && (|(decoded_signals[2:0]) == 1'b1)) |
		// (decoded_signals[3:0] == 4'h8) | 
		// (decoded_signals[4] == 1'b1) ? 1'b1 : 1'b0;

// do not write to scalar register if vector COP is reading from scalar register (uses pseudo ADD instruction)
assign reg_wr_en = 
		((((decoded_signals[3] == 1'b0) && (|(decoded_signals[2:0]) == 1'b1)) |
		(decoded_signals[3:0] == 4'h8) | 
		(decoded_signals[4] == 1'b1)) & (~s3_vector_cop_scalar_read))
		| s3_vector_cop_scalar_write	? 1'b1 : 1'b0;

assign reg_wr_en_loads = (decoded_signals[3:0] == 4'h0) ? opcode[1] : 1'b0;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

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

