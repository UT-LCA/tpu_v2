/*****************************************************************************
 *                                                                           *
 * Module:       Memory_Data_In_Align                                        *
 * Description:                                                              *
 *      This module aligns the data that was loaded from memory.             *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Memory_Data_In_Align the_Memory_Data_In_Align (
	// Inputs
	.d_address	(),
	.data_in	(),
	.opcode		(),
	
	// Bidirectionals

	// Outputs
	.data_out	()
);
*/

module Memory_Data_In_Align (
	// Inputs
	d_address,
	data_in,
	opcode,
	
	// Bidirectionals

	// Outputs
	data_out
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input	[31:0]	d_address;
input	[31:0]	data_in;
input	[5:0]	opcode;

// Bidirectionals

// Outputs
output	[31:0]	data_out;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire	[2:0]	align_selectors;
wire	[1:0]	byte_selectors;

wire	[31:0]	byte_data;
wire			byte_sign;
wire	[31:0]	half_word_data;
wire			half_word_sign;

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

assign align_selectors	= opcode[4:2];
assign byte_selectors	= d_address[1:0];

assign byte_sign		=
	((byte_selectors == 2'b11) ? (align_selectors[0] & data_in[31]) :
	((byte_selectors == 2'b10) ? (align_selectors[0] & data_in[23]) :
	((byte_selectors == 2'b01) ? (align_selectors[0] & data_in[15]) :
								 (align_selectors[0] & data_in[7]))));
assign half_word_sign	= 
	(byte_selectors[1] & align_selectors[0] & data_in[31]) |
	(~byte_selectors[1] & align_selectors[0] & data_in[15]);


assign byte_data		=
	((byte_selectors == 2'b11) ? ({{24{byte_sign}}, data_in[31:24]}) :
	((byte_selectors == 2'b10) ? ({{24{byte_sign}}, data_in[23:16]}) :
	((byte_selectors == 2'b01) ? ({{24{byte_sign}}, data_in[15: 8]}) :
								 ({{24{byte_sign}}, data_in[ 7: 0]}))));
assign half_word_data	= 
	((byte_selectors[1] == 1'b1) ?  ({{16{half_word_sign}}, data_in[31:16]}) : 
									({{16{half_word_sign}}, data_in[15:0]}));

assign data_out = ((align_selectors[2] == 1'b1) ? data_in :
				((align_selectors[1] == 1'b1) ? half_word_data :
				byte_data));

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

