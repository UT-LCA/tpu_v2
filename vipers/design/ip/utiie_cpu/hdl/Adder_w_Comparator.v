/*****************************************************************************
 *                                                                           *
 * Module:       Adder_w_Comparator                                          *
 * Description:                                                              *
 *      This module performs add, subtract and compare instructions.         *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Adder_w_Comparator the_Adder_w_Comparator (
	// Inputs
	.operandA		(),
	.operandB		(),
	.sub			(),
	
	// Bidirectionals

	// Outputs
	.result			(),

	.negative_flag	(),
	.overflow_flag	(),
	.zero_flag		()
);
*/

module Adder_w_Comparator (
	// Inputs
	operandA,
	operandB,
	sub,
	
	// Bidirectionals

	// Outputs
	result,

	negative_flag,
	overflow_flag,
	zero_flag
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/
// States

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input	[32:0]	operandA;
input	[32:0]	operandB;
input			sub;

// Bidirectionals

// Outputs
output	[31:0]	result;

output			negative_flag;
output			overflow_flag;
output			zero_flag;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires

// Internal Registers
wire	[32:0]	add_sub_result;

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

assign result			= add_sub_result[31:0];
assign zero_flag		= ((result == 0) ? 1'b1 : 1'b0);
assign negative_flag	= add_sub_result[32];

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/
	
lpm_add_sub	Adder_Subtractor (
	.add_sub	(~sub),
	.dataa		(operandA),
	.datab		(operandB),
	.overflow	(overflow_flag),
	.result		(add_sub_result)
	// synopsys translate_off
	,
	.cin		(),
	.cout		(),
	.clock		(),
	.clken		(),
	.aclr		()
	// synopsys translate_on
);
defparam
	Adder_Subtractor.lpm_direction	= "UNUSED",
	Adder_Subtractor.lpm_hint		= "ONE_INPUT_IS_CONSTANT=NO,CIN_USED=NO",
	Adder_Subtractor.lpm_type		= "LPM_ADD_sub",
	Adder_Subtractor.lpm_width		= 33;

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

