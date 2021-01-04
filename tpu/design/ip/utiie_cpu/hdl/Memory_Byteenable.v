/*****************************************************************************
 *                                                                           *
 * Module:       Memory_Byteenable                                           *
 * Description:                                                              *
 *      This module creates the byte enable signals for memory instructions. *
 *                                                                           *
 *****************************************************************************/


// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Memory_Byteenable the_Memory_Byteenable (
	// Inputs
	.d_address		(),
	.opcode			(),
	
	// Bidirectionals

	// Outputs
	.d_byteenable	()
);
*/

module Memory_Byteenable (
	// Inputs
	d_address,
	opcode,
	
	// Bidirectionals

	// Outputs
	d_data_size,
	d_byteenable
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input	[31:0]	d_address;
input	[5:0]	opcode;

// Bidirectionals

// Outputs
output	[1:0]	d_data_size;	// size of data to be writte to data master: 00 = byte, 01 = halfword, 10 = word
output	[3:0]	d_byteenable;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire	[1:0]	data_byte_offset;
// wire	[1:0]	d_data_size;

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

assign data_byte_offset = d_address[1:0];
assign d_data_size 		= opcode[4:3];

assign d_byteenable = 
	{(d_data_size[1] | (d_data_size[0] & data_byte_offset[1]) |
		(data_byte_offset[1] & data_byte_offset[0])),
	 (d_data_size[1] | (d_data_size[0] & data_byte_offset[1]) |
		(data_byte_offset[1] & ~data_byte_offset[0])),
	 (d_data_size[1] | (d_data_size[0] & ~data_byte_offset[1])|
		(~data_byte_offset[1] & data_byte_offset[0])),
	 (d_data_size[1] | (d_data_size[0] & ~data_byte_offset[1])|
		(~data_byte_offset[1] & ~data_byte_offset[0]))};

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

