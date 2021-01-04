/*****************************************************************************
 *                                                                           *
 * Module:       Instr_Register                                              *
 * Description:                                                              *
 *      This module is the instruction register for the UT IIe processor.    *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Instr_Register the_Instr_Register (
	// Inputs
	.clk					(clk),
	.reset_n				(reset_n),

	.fetch_stage_completed	(),
	.fetched_instruction	(),
	.is_rtype				(),

	// Bidirectionals

	// Outputs
	.ir						(),
	.opcode					()
);
*/

module Instr_Register (
	// Inputs
	clk,
	reset_n,

	fetch_stage_completed,
	fetched_instruction,
	is_rtype,

	// Bidirectionals

	// Outputs
	ir,
	opcode
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
input	[31:0]	fetched_instruction;
input			is_rtype;

// Bidirectionals

// Outputs
output	[31:0]	ir;
output	[5:0]	opcode;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires

// Internal Registers
reg		[31:0]	ir;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

// always @ (posedge clk)
always @ (posedge clk, negedge reset_n)
begin
	if (reset_n == 1'b0)
	begin
		ir <= 32'h00000000;
	end
	else
	begin
		if (fetch_stage_completed == 1'b1)
		begin
			ir <= fetched_instruction;
		end
	end
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign opcode = (is_rtype == 1'b0) ? ir[5:0] : ir[16:11];

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

