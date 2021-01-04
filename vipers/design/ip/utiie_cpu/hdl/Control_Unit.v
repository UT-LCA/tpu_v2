/*****************************************************************************
 *                                                                           *
 * Module:       Control_Unit                                                *
 * Description:                                                              *
 *      This module is the control unit of the UT IIe processor.             *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Control_Unit the_Control_Unit (
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),

	.data_master_busy			(),
	.fetch_unit_has_valid_data	(),
	.interrupt_active			(),
	.shifter_done_flag			(),

	// Bidirectionals

	// Outputs
	.executing_fetch_stage		(),
	.executing_decode_stage		(),
	.executing_execute_stage	(),
	.executing_write_back_stage	(),
	.executing_irq_stage		(),
	.pipeline_stalled			()
);
*/

module Control_Unit (
	// Inputs
	clk,						// Global Clock
	reset_n,					// if 0, the cpu is being reset, therefore 
								//	 the all registers should be reset

	data_master_busy,			// There is a load or store instruction  
								//	 in the mem stage
	fetch_unit_has_valid_data,	// 1 indicates that the fetch has completed
	interrupt_active,			// if 1, external interrupt is active
	shifter_done_flag,			// if 1, Shift has completed

	s3_vector_cop_scalar_read,		// vector COP instruction, read from scalar core
	vector_cop_scalar_skip,		// vector COP instruction only, scalar core can skip it
	
	// Input from vector core
	cop_fetch_stall,			// stall the instruction fetch
	scalarToVectorQ_full,		// scalar to vector transfer queue is full
	s3_vector_cop_scalar_write,	// vector instruction writes to scalar reg
	vectorToScalarQ_empty,		// vector to scalar transfer queue is empty
	
	// Bidirectionals

	// Outputs
	executing_fetch_stage,		// Fetch stage is active if 1
	executing_decode_stage,		// Decode stage is active if 1
	executing_execute_stage,	// Execute stage is active if 1
	executing_write_back_stage,	// Write Back stage is active if 1
	executing_irq_stage,		// Interrupt stage is active if 1
	pipeline_stalled			// Pipeline has stalled if 1
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

input			data_master_busy;
input			fetch_unit_has_valid_data;
input			interrupt_active;
input			shifter_done_flag;

input			s3_vector_cop_scalar_read;
input			vector_cop_scalar_skip;		// vector COP instruction only, scalar core can skip it

// input from vector core
input			cop_fetch_stall;
input			scalarToVectorQ_full;
input			vectorToScalarQ_empty;		// vector to scalar transfer queue is empty
input			s3_vector_cop_scalar_write;	// vector instruction writes to scalar reg

// Bidirectionals

// Outputs
output			executing_fetch_stage;
output			executing_decode_stage;
output			executing_execute_stage;
output			executing_write_back_stage;
output			executing_irq_stage;
output			pipeline_stalled;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire			pipeline_stalled;
wire			fetch_stage_stall;
wire			execute_stage_stall;
wire			memory_stage_stall;
wire			decode_stage_stall;
wire			scalarToVectorQFull_stall;
wire			vectorToScalarQEmpty_stall;

// Internal Registers
reg		[5:1]	active_stage;

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
	// sync reset
	if (reset_n == 1'b0)
	begin
		// If the reset is active, start with fetch stage
		active_stage		<= 5'b00001;
	end
	else if (pipeline_stalled == 1'b0)
	begin
		if (active_stage[4] == 1'b1)
		begin
			active_stage[5]	<= interrupt_active;
		end
		
		// if a vector COP only instruction is decoded, stay in fetch stage and fetch next instruction
		// ** This is a separate IF condition! **
		if ((active_stage[1] == 1'b1) && vector_cop_scalar_skip)
			active_stage <= active_stage;
		else
			// If the reset is not active, move to the next pipeline stage
			active_stage[4:1]	<= {active_stage[3:1], active_stage[4]};
	end
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign executing_fetch_stage		= active_stage[1];
assign executing_decode_stage		= active_stage[2];
assign executing_execute_stage		= active_stage[3];
assign executing_write_back_stage	= active_stage[4];
assign executing_irq_stage			= active_stage[5];


//assign pipeline_stalled	= active_stage[3] & ~shifter_done_flag;
assign pipeline_stalled		= fetch_stage_stall | 
								decode_stage_stall |
							  execute_stage_stall | 
							  memory_stage_stall;

assign fetch_stage_stall	= (active_stage[1] & ~fetch_unit_has_valid_data) | (active_stage[1] & cop_fetch_stall);
// assign fetch_stage_stall	= (active_stage[1] & ~fetch_unit_has_valid_data);
assign	decode_stage_stall	= scalarToVectorQFull_stall;
assign execute_stage_stall	= (active_stage[3] & ~shifter_done_flag) | vectorToScalarQEmpty_stall;
assign memory_stage_stall	= data_master_busy;

assign	scalarToVectorQFull_stall = scalarToVectorQ_full & executing_decode_stage & s3_vector_cop_scalar_read;
assign	vectorToScalarQEmpty_stall = vectorToScalarQ_empty & executing_execute_stage & s3_vector_cop_scalar_write;

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

