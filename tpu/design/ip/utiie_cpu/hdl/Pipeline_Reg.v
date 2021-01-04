/*****************************************************************************
 *                                                                           *
 * Module:       Pipeline_Reg                                                *
 * Description:                                                              *
 *      This module is a pipeline register.                                  *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Pipeline_Reg Pipeline_Reg (
	// Inputs
	.clk			(clk),
	.clk_en			(),
	.reset_n		(reset_n),

	.input_signal	(),

	// Bidirectionals

	// Outputs
	.output_signal	()
);
*/

module Pipeline_Reg (
	// Inputs
	clk,
	clk_en,
	reset_n,

	input_signal,

	// Bidirectionals

	// Outputs
	output_signal
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter REG_SIZE		= 1;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input					clk;
input					clk_en;
input					reset_n;

input	[REG_SIZE:1]	input_signal;

// Bidirectionals

// Outputs
output	[REG_SIZE:1]	output_signal;

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

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

lpm_ff	the_Register (
	.clock	(clk),
	.enable	(clk_en | ~reset_n),
	.sclr	(~reset_n),

	.data	(input_signal),

	.q		(output_signal)
	// synopsys translate_off
	,
	.sset	(),
	.aset	(),
	.aload	(),
	.aclr	(),
	.sload	()
	// synopsys translate_on
);
defparam
	the_Register.lpm_fftype	= "DFF",
	the_Register.lpm_type	= "LPM_FF",
	the_Register.lpm_width	= REG_SIZE;


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


 
 
/*****************************************************************************
 *                                                                           *
 * Module:       Pipeline_Reg_aclr                                                *
 * Description:                                                              *
 *      This module is a pipeline register with async clear.                                  *
 *                                                                           *
 *****************************************************************************/
 
 module Pipeline_Reg_aclr (
	// Inputs
	clk,
	clk_en,
	// reset_n,
	aclr,
	sclr,

	input_signal,

	// Bidirectionals

	// Outputs
	output_signal
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter REG_SIZE		= 1;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input					clk;
input					clk_en;
// input					reset_n;
input					aclr;
input					sclr;

input	[REG_SIZE:1]	input_signal;

// Bidirectionals

// Outputs
output	[REG_SIZE:1]	output_signal;

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

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

lpm_ff	the_Register (
	.clock	(clk),
	.enable	(clk_en | sclr),
	.aclr	(aclr),
	.sclr	(sclr),

	.data	(input_signal),

	.q		(output_signal)
	// synopsys translate_off
	,
	.sset	(),
	.aset	(),
	.aload	(),
	.sload	()
	// synopsys translate_on
);
defparam
	the_Register.lpm_fftype	= "DFF",
	the_Register.lpm_type	= "LPM_FF",
	the_Register.lpm_width	= REG_SIZE;


endmodule
