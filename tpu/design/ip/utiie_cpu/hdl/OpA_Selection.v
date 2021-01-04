/*****************************************************************************
 *                                                                           *
 * Module:       OpA_Selection      	                                     *
 * Description:                                                              *
 *      This module selects the correct value for operand A.                 *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
OpA_Selection the_OpA_Selector (
	// Inputs
	.decoded_signals		(),
	.imm26					(),
	.pc						(),
	.rA						(),
	
	// Bidirectionals

	// Outputs
	.opA					()
);
defparam the_OpA_Selector.BREAK_ADDRESS		= BREAK_ADDRESS;
defparam the_OpA_Selector.EXCEPTION_ADDRESS	= EXCEPTION_ADDRESS;
*/

module OpA_Selection (
	// Inputs
	decoded_signals,
	imm26,
	pc,
	rA,
	
	// Bidirectionals

	// Outputs
	opA
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
input 	[7:0]	decoded_signals;
input 	[25:0]	imm26;
input 	[31:0]	pc;
input 	[31:0]	rA;

// Bidirectionals

// Outputs
output 	[31:0]	opA;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire	[1:0]	selectors;

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

assign selectors = (decoded_signals[3:0] == 4'hB) ? decoded_signals[7:6] : 2'h1;

assign opA =	(selectors == 2'h1) ?	rA :
				(selectors == 2'h0) ?	{pc[31:28], imm26, 2'b00} :
				(selectors == 2'h2) ?	EXCEPTION_ADDRESS :
										BREAK_ADDRESS;

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

