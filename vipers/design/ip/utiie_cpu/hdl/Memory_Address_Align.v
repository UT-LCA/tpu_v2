/*****************************************************************************
 *                                                                           *
 * Module:       Memory_Address_Align                                        *
 * Description:                                                              *
 *      This module guarantees that the memory address is aligned correctly. *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Memory_Address_Align the_Memory_Address_Alignment (
	// Inputs
	.address_in		(),
	.opcode			(),
	
	// Bidirectionals

	// Outputs
	.address_out	()
);
*/

module Memory_Address_Align (
	// Inputs
	address_in,
	opcode,
	
	// Bidirectionals

	// Outputs
	address_out
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input	[31:0]	address_in;
input	[5:0]	opcode;

// Bidirectionals

// Outputs
output	[31:0]	address_out;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire	[1:0]	data_size;

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

assign data_size = opcode[4:3];

assign address_out = ((data_size[1] == 1'b1) ? {address_in[31:2], {2{1'b0}}} :
				((data_size[0] == 1'b1) ? {address_in[31:1], 1'b0} :
				address_in));

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

