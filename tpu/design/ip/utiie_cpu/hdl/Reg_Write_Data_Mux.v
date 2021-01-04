/*****************************************************************************
 *                                                                           *
 * Module:       Reg_Write_Data_Mux                                          *
 * Description:                                                              *
 *      This module selects which data to write to the register file.        *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Reg_Write_Data_Mux the_Reg_Write_Data_Mux (
	// Inputs
	.adder_result	(),
	.cmp_result		(),
	.ctrl_regs		(),
	.logic_result	(),
	.npc_shifter	(),
	
	// Bidirectionals

	// Outputs
	.reg_write_data	()
);
*/

module Reg_Write_Data_Mux (
	// Inputs
	s4_vector_cop_scalar_write,
	vectorToScalarQ_q,
	adder_result,
	cmp_result,
	ctrl_regs,
	logic_result,
	npc_shifter,
	
	// Bidirectionals

	// Outputs
	reg_write_data
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input			s4_vector_cop_scalar_write;		// vector instruction needs to write to scalar reg
input	[31:0]	vectorToScalarQ_q;				// vector to scalar core transfer queue output

input	[31:0]	adder_result;
input			cmp_result;
input	[31:0]	ctrl_regs;
input	[31:0]	logic_result;
input	[31:0]	npc_shifter;

// Bidirectionals

// Outputs
output	[31:0]	reg_write_data;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire	[31:0]	scalar_reg_write_data;

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

// assign reg_write_data[31:1]	= 	adder_result[31:1] | 
								// ctrl_regs[31:1] |
								// logic_result[31:1] | 
								// npc_shifter[31:1];

// assign reg_write_data[0] = 	adder_result[0] | 
							// cmp_result |
							// ctrl_regs[0] |
							// logic_result[0] | 
							// npc_shifter[0];

assign scalar_reg_write_data[31:1]	= 	adder_result[31:1] | 
								ctrl_regs[31:1] |
								logic_result[31:1] | 
								npc_shifter[31:1];

assign scalar_reg_write_data[0] = 	adder_result[0] | 
							cmp_result |
							ctrl_regs[0] |
							logic_result[0] | 
							npc_shifter[0];

assign	reg_write_data = (s4_vector_cop_scalar_write == 1'b1) ? vectorToScalarQ_q : scalar_reg_write_data;


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

