/*****************************************************************************
 *                                                                           *
 * Module:       Logic_Unit                                                  *
 * Description:                                                              *
 *      This module performs logic instruction operations.                   *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Logic_Unit the_Logic_Unit (
	// Inputs
	.opA			(),
	.opB			(),
	.opcode			(),

	// Bidirectionals

	// Outputs
	.logic_result	()
);
*/

module Logic_Unit (
	// Inputs
	opA,
	opB,
	opcode,

	// Bidirectionals

	// Outputs
	logic_result
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input	[31:0]	opA;
input	[31:0]	opB;
input	[5:0]	opcode;

// Bidirectionals

// Outputs
output	[31:0]	logic_result;

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

assign logic_result =	
		({32{~opcode[4]}} & {32{~opcode[3]}} & ~(opA | opB)) | 	// nor
		({32{~opcode[4]}} & {32{ opcode[3]}} &  (opA & opB)) |	// and
		({32{ opcode[4]}} & {32{~opcode[3]}} &  (opA | opB)) |	// or
		({32{ opcode[4]}} & {32{ opcode[3]}} &  (opA ^ opB));	// xor

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

