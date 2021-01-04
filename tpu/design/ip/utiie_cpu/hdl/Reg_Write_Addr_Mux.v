/*****************************************************************************
 *                                                                           *
 * Module:       Reg_Write_Addr_Mux                                          *
 * Description:                                                              *
 *      This module selects the correct address to write the instruction's   *
 *   result to, in the register file.                                        *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Reg_Write_Addr_Mux the_Reg_Write_Addr_Mux (
	// Inputs
	.is_rtype		(),
	.op_code		(),
	.rB_addr		(),
	.rC_addr		(),

	// Bidirectionals

	// Outputs
	.reg_write_addr	()
);
*/

module Reg_Write_Addr_Mux (
	// Inputs
	is_rtype,
	vector_cop_scalar_write,		// vector instruction needs to write to scalar reg
	op_code,
	rB_addr,
	rC_addr,
	vcop_rS_dest_addr,				// scalar destination register address for vector instructions

	// Bidirectionals

	// Outputs
	reg_write_addr
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input			is_rtype;
input			vector_cop_scalar_write;		// vector instruction needs to write to scalar reg
input	[5:0]	op_code;
input	[4:0]	rB_addr;
input	[4:0]	rC_addr;
input	[4:0]	vcop_rS_dest_addr;

// Bidirectionals

// Outputs
output	[4:0]	reg_write_addr;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire			select_r31;

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

assign select_r31		=	(op_code == 6'h00) ? 1'b1 : 1'b0;

// if vector instruction writes to scalar register, use vcop_rS_dest_addr
assign reg_write_addr	=	((select_r31 == 1'b1) ? 5'h1F :
							((vector_cop_scalar_write) ? vcop_rS_dest_addr :
							((is_rtype == 1'b1) ? rC_addr : rB_addr)));

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

