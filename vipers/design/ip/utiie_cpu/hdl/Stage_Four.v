/*****************************************************************************
 *                                                                           *
 * Module:       Stage_Four                                                  *
 * Description:                                                              *
 *      This module is the 4th stage (WB/Mem) of the UT IIe processor.       *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Stage_Four Stage_Four (
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),

	.executing_fetch_stage		(),
	.executing_decode_stage		(),
	.executing_execute_stage	(),
	.executing_write_back_stage	(),
	.executing_irq_stage		(),
	.pipeline_stalled			(),

	.d_readdata					(),
	.d_waitrequest				(),

	.s4_adder_result			(),
	.s4_cmp_result				(),
	.s4_ctrl_regs				(),
	.s4_load_store				(),
	.s4_logic_result			(),
	.s4_npc_shifter				(),
	.s4_opcode					(),
	.s4_store_data				(),

	// Bidirectionals

	// Outputs
	.d_address					(),
	.d_byteenable				(),
	.d_read						(),
	.d_write					(),
	.d_writedata				(),

	.s4_reg_wr_data				(),
	.s4_reg_wr_data_loads		(),

	.data_master_busy			()
);
*/

module Stage_Four (
	// Inputs
	clk,						// Global Clock
	reset_n,					// if 0, the cpu is being reset, therefore 
								//	 the all registers should be reset

	executing_fetch_stage,		// Fetch stage is active if 1
	executing_decode_stage,		// Decode stage is active if 1
	executing_execute_stage,	// Execute stage is active if 1
	executing_write_back_stage,	// Write Back stage is active if 1
	executing_irq_stage,		// Interrupt stage is active if 1
	pipeline_stalled,			// Pipeline has stalled if 1

	d_readdata,					// The data comming in from the data master
	d_waitrequest,				// Data master requests wait

	s4_adder_result,			// The result from the adder
	s4_cmp_result,				// The result of a comparison operation
	s4_ctrl_regs,				// The result of a read from the ctrl registers
	s4_load_store,				// 1 - Load, 0 - Store
	s4_logic_result,			// The result from the logic unit
	s4_npc_shifter,				// The result from the shifter unit or next pc 
	s4_opcode,					// Opcode of the current instruction
	s4_store_data,				// Data to be written to the data master
	s4_vector_cop_scalar_write,	// vector instruction needs to write to scalar reg
	vectorToScalarQ_q,			// vector to scalar core transfer queue output

	// Bidirectionals

	// Outputs
	d_address,					// The address for the data master
	d_byteenable,				// The byte enable for the data master
	d_read,						// The read enable for the data master
	d_write,					// The write enable for the data master
	d_writedata,				// Data to be written to the data master
	d_data_size,				// size of data to be written to data master

	s4_reg_wr_data,				// Data to be written to the register file
	s4_reg_wr_data_loads,		// Data to be written to the register file 
								//	 from a load instruction

	data_master_busy			// Executing a load or store instruction  
);
	
/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/
// States

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input			clk;
input			reset_n;

input			executing_fetch_stage;
input			executing_decode_stage;
input			executing_execute_stage;
input			executing_write_back_stage;
input			executing_irq_stage;
input			pipeline_stalled;

input	[31:0]	d_readdata;
input			d_waitrequest;

input	[31:0] 	s4_adder_result;
input		 	s4_cmp_result;
input	[31:0]	s4_ctrl_regs;
input			s4_load_store;
input	[31:0]	s4_logic_result;
input	[31:0]	s4_npc_shifter;
input	[5:0] 	s4_opcode;
input	[31:0] 	s4_store_data;
input			s4_vector_cop_scalar_write;		// vector instruction needs to write to scalar reg
input	[31:0]	vectorToScalarQ_q;				// vector to scalar core transfer queue output

// Bidirectionals

// Outputs
output	[31:0]  d_address;
output	[3:0]	d_byteenable;
output			d_read;
output			d_write;
output	[31:0]	d_writedata;
output	[1:0]	d_data_size;

output	[31:0]  s4_reg_wr_data;
output	[31:0]  s4_reg_wr_data_loads;

output			data_master_busy;

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

assign d_read	= s4_load_store & s4_opcode[1] & executing_write_back_stage; 
assign d_write	= s4_load_store & ~s4_opcode[1] & executing_write_back_stage;

assign d_writedata	= s4_store_data;

assign data_master_busy		= (d_read | d_write) & d_waitrequest;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Memory_Address_Align the_Memory_Address_Alignment (
	// Inputs
	.address_in		(s4_adder_result),
	.opcode			(s4_opcode),
	
	// Bidirectionals

	// Outputs
	.address_out	(d_address)
);

Memory_Byteenable the_Memory_Byteenable (
	// Inputs
	.d_address		(d_address),
	.opcode			(s4_opcode),
	
	// Bidirectionals

	// Outputs
	.d_data_size	(d_data_size),
	.d_byteenable	(d_byteenable)
);

Memory_Data_In_Align the_Memory_Data_In_Align (
	// Inputs
	.d_address	(d_address),
	.data_in	(d_readdata),
	.opcode		(s4_opcode),
	
	// Bidirectionals

	// Outputs
	.data_out	(s4_reg_wr_data_loads)
);

Reg_Write_Data_Mux the_Reg_Write_Data_Mux (
	// Inputs
	.s4_vector_cop_scalar_write	(s4_vector_cop_scalar_write),
	.vectorToScalarQ_q	(vectorToScalarQ_q),
	.adder_result	(s4_adder_result),
	.cmp_result		(s4_cmp_result),
	.ctrl_regs		(s4_ctrl_regs),
	.logic_result	(s4_logic_result),
	.npc_shifter	(s4_npc_shifter),
	
	// Bidirectionals

	// Outputs
	.reg_write_data	(s4_reg_wr_data)
);

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

