/*****************************************************************************
 *                                                                           *
 * Module:       Branch_Logic                                                *
 * Description:                                                              *
 *      This module checks if a branch or jump instruction should be taken.  *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Branch_Logic the_Branch_Logic (
	// Inputs
	.cmp_flag			(),
	.decoded_signals	(),
	.opcode				(),

	// Bidirectionals

	// Outputs
	.branch_taken		(),
	.branch_type		()
);
*/

module Branch_Logic (
	// Inputs
	cmp_flag,
	decoded_signals,
	opcode,

	// Bidirectionals

	// Outputs
	branch_taken,
	branch_type		// 0 - jmp (opA),  1 - branch with offset (pc + 4 + offset)
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input			cmp_flag;
input	[7:0]	decoded_signals;
input	[5:0]	opcode;

// Bidirectionals

// Outputs
output			branch_type;
output			branch_taken;

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

assign branch_type	=	(decoded_signals[3:1] == 3'h5) ? 
							(&(opcode[2:1]) | ~decoded_signals[0]) : 
							1'b1;

assign branch_taken =	(decoded_signals[3:1] == 3'h5)	? 1'b1 :
						(decoded_signals[3:2] == 2'h3)	? cmp_flag : 1'b0; 

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

