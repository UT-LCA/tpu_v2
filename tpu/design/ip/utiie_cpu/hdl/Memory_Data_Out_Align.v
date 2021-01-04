/*****************************************************************************
 *                                                                           *
 * Module:       Memory_Data_Out_Align                                       *
 * Description:                                                              *
 *      This module aligns the data to be stored to memory.                  *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Memory_Data_Out_Align the_Memory_Data_Out_Align (
	// Inputs
	.data_in	(),
	.opcode		(),
	
	// Bidirectionals

	// Outputs
	.data_out	()
);
*/

module Memory_Data_Out_Align (
	// Inputs
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
input	[31:0]	data_in;
input	[5:0]	opcode;

// Bidirectionals
output	[31:0]	data_out;

// Outputs

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire	[1:0]	data_size; 	// 1x word, 01 half word, 00 byte

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

assign data_size[1] = opcode[4];
assign data_size[0] = opcode[3];

assign data_out = 
	((data_size[1] == 1'b1) ? data_in :
	((data_size[0] == 1'b1) ? {2{data_in[15:0]}} :
							  {4{data_in[7:0]}}));
					
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

