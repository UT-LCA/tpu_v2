/*****************************************************************************
 *                                                                           *
 * Module:       OpB_Imm_Selection      	                                 *
 * Description:                                                              *
 *      This module selects the correct value for operand B.                 *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
OpB_Imm_Selection the_OpB_Imm_Selector (
	// Inputs
	.decoded_signals	(),
	.imm16				(),
	.rB					(),

	// Bidirectionals

	// Outputs
	.opB				()
);
*/

module OpB_Imm_Selection (
	// Inputs
	decoded_signals,	// [7:6]	2'b00 rB, 2'b01 16'h0000:IMM16, 
						//			2'b10 IMM16:16'h0000, 2'b11 sign(IMM16)
	imm16,
	rB,

	// Bidirectionals

	// Outputs
	opB
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input 	[7:0]	decoded_signals;
input 	[15:0]	imm16;
input 	[31:0] 	rB;

// Bidirectionals

// Outputs
output 	[31:0] 	opB;

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

assign selectors = (decoded_signals[7:6] == 2'b11) ? 
						{imm16[15], 1'b1} : 
						decoded_signals[7:6];
/*
assign opB[31:16] =	(selectors == 2'b00) ? rB[31:16] :
					(selectors == 2'b10) ? imm16 : {16{selectors[1]}};

assign opB[15:00] =	(selectors == 2'b00) ? rB[15:00] :
					(selectors == 2'b10) ? 16'h0000 : imm16;
*/
assign opB[31:16] =	({16{~selectors[1]}} & {16{~selectors[0]}} & rB[31:16]) |
					({16{ selectors[1]}} & {16{~selectors[0]}} & imm16) |
					({16{ selectors[1]}} & {16{ selectors[0]}});

assign opB[15:00] =	({16{~selectors[1]}} & {16{~selectors[0]}} & rB[15:00]) |
					({16{ selectors[0]}} & imm16);
				
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

