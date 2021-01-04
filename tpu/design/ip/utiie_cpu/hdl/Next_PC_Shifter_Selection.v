/*****************************************************************************
 *                                                                           *
 * Module:       Next_PC_Shifter_Selection                                   *
 * Description:                                                              *
 *      This module selects between the PC and the shifter result.           *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Next_PC_Shifter_Selection the_Next_PC_Shifter_Selector (
	// Inputs
	.decoded_signals			(),
	.next_pc					(),
	.shifter_result				(),
	.use_shifter_result			(),
	
	// Bidirectionals

	// Outputs
	.next_pc_shifter_result		(),
	.use_next_pc_shifter_result	()
);
*/

module Next_PC_Shifter_Selection (
	// Inputs
	decoded_signals,
	next_pc,
	shifter_result,
	use_shifter_result,
	
	// Bidirectionals

	// Outputs
	next_pc_shifter_result,
	use_next_pc_shifter_result
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input	[7:0]	decoded_signals;
input 	[31:0] 	next_pc;
input 	[31:0] 	shifter_result;
input		 	use_shifter_result;

// Bidirectionals

// Outputs
output 	[31:0] 	next_pc_shifter_result;
output		 	use_next_pc_shifter_result;

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

assign next_pc_shifter_result	= (use_shifter_result) ? 
									shifter_result : 
									next_pc;

assign use_next_pc_shifter_result = 
			use_shifter_result | (~decoded_signals[5] & decoded_signals[4]);

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

