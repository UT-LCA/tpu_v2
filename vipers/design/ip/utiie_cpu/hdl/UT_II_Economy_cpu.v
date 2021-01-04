/*****************************************************************************
 *                                                                           *
 * Module:       UT_II_Economy_cpu                                           *
 * Description:                                                              *
 *      This module is the UT IIe processor.                                 *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

module UT_II_Economy_cpu (
	// Inputs
	clk,
	reset_n,

	d_irq,
	d_readdata,
	d_waitrequest,

	i_readdata,
	i_waitrequest,
	
	// Inputs from vector core
	cop_fetch_stall,
	scalarToVectorQ_full,
	
	// vector to scalar transfer queue
	vectorToScalarQ_empty,
	vectorToScalarQ_q,

	// Bidirectionals

	// Outputs
	d_address,
	d_byteenable,
	d_read,
	d_write,
	d_writedata,
	d_data_size,				// size of data to be writte to data master: 00 = byte, 01 = halfword, 10 = word

	i_address,
	i_read,
	
	// Output to vector core
	//vpu_new_instr,
	vector_cop_instruction,		// a vector coprocessor instruction has been fetched
	vector_opcode,
	predecode_scalarmem_instr,		// predecode scalar memory instruction
	predecode_vectormem_instr,		// predecode vector memory instruction
	
	scalarRegOut,
	scalarToVectorQ_wrreq,
	
	// vector to scalar transfer queue
	vectorToScalarQ_rdreq
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter RESET_ADDRESS		= 32'h00000000;
parameter BREAK_ADDRESS		= 32'h00000000;
parameter EXCEPTION_ADDRESS	= 32'h00000020;
parameter CPU_ID			= 32'h00000003;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input			clk;
input			reset_n;

input	[31:0]	d_irq;
input	[31:0]	d_readdata;
input			d_waitrequest;

input	[31:0]	i_readdata;
input			i_waitrequest;

// Inputs from vector core
input			cop_fetch_stall;	// coprocessor stall instruction fetch
input			scalarToVectorQ_full;		// the write queue from scalar to vector core is full
// Vector to scalar transfer queue
input			vectorToScalarQ_empty;
input	[31:0]	vectorToScalarQ_q;
	
// Bidirectionals

// Outputs
output	[31:0]	d_address;
output	[3:0]	d_byteenable;
output			d_read;
output			d_write;
output	[31:0]	d_writedata;
output	[1:0]	d_data_size;		// size of data to be writte to data master: 00 = byte, 01 = halfword, 10 = word

output	[31:0]	i_address;
output			i_read;

// Output to vector core
output			vector_cop_instruction;		// it is a vector coprocessor instruction
output	[31:0]	vector_opcode;
output			predecode_scalarmem_instr;		// predecode scalar memory instruction
output			predecode_vectormem_instr;		// predecode vector memory instruction
output	[31:0]	scalarRegOut;		// connection from scalar register to vector register
output			scalarToVectorQ_wrreq;		// write enable from scalar register to vector data queue

output			vectorToScalarQ_rdreq;		// vector to scalar transfer queue rdreq



/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire			data_master_busy;
wire			fetch_unit_has_valid_data;
wire			interrupt_active;
wire			shifter_done_flag;

wire			executing_fetch_stage;
wire			executing_decode_stage;
wire			executing_execute_stage;
wire			executing_write_back_stage;
wire			executing_irq_stage;
wire			pipeline_stalled;

wire			s3_vector_cop_scalar_read;
wire			vector_cop_scalar_skip;		// vector COP instruction only, scalar core can skip it
wire			s3_vectorToScalarQ_rdreq;	// vector to scalar transfer queue rdreq
wire			s3_vector_cop_scalar_write;	// vector instruction writes to scalar reg

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

assign	vectorToScalarQ_rdreq = s3_vectorToScalarQ_rdreq;
 
/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Control_Unit the_Control_Unit (
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),

	.data_master_busy			(data_master_busy),
	.fetch_unit_has_valid_data	(fetch_unit_has_valid_data),
	.interrupt_active			(interrupt_active),
	.shifter_done_flag			(shifter_done_flag),

	.s3_vector_cop_scalar_read		(s3_vector_cop_scalar_read),
	.vector_cop_scalar_skip		(vector_cop_scalar_skip),		// vector COP instruction only, scalar core can skip it

	// vector processor inputs
	.cop_fetch_stall			(cop_fetch_stall),
	.scalarToVectorQ_full		(scalarToVectorQ_full),
	.s3_vector_cop_scalar_write	(s3_vector_cop_scalar_write),
	.vectorToScalarQ_empty		(vectorToScalarQ_empty),

	// Bidirectionals

	// Outputs
	.executing_fetch_stage		(executing_fetch_stage),
	.executing_decode_stage		(executing_decode_stage),
	.executing_execute_stage	(executing_execute_stage),
	.executing_write_back_stage	(executing_write_back_stage),
	.executing_irq_stage		(executing_irq_stage),
	.pipeline_stalled			(pipeline_stalled)
);


Datapath the_Datapath (
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),

	.executing_fetch_stage		(executing_fetch_stage),
	.executing_decode_stage		(executing_decode_stage),
	.executing_execute_stage	(executing_execute_stage),
	.executing_write_back_stage	(executing_write_back_stage),
	.executing_irq_stage		(executing_irq_stage),
	.pipeline_stalled			(pipeline_stalled),

	.d_irq						(d_irq),
	.d_readdata					(d_readdata),
	.d_waitrequest				(d_waitrequest),

	.i_readdata					(i_readdata),
	.i_waitrequest				(i_waitrequest),

	.vectorToScalarQ_q			(vectorToScalarQ_q),
	
	// Bidirectionals

	// Outputs
	.d_address					(d_address),
	.d_byteenable				(d_byteenable),
	.d_read						(d_read),
	.d_write					(d_write),
	.d_writedata				(d_writedata),
	.d_data_size				(d_data_size),

	.data_master_busy			(data_master_busy),

	.fetch_unit_has_valid_data	(fetch_unit_has_valid_data),

	.i_address					(i_address),
	.i_read						(i_read),

	.interrupt_active			(interrupt_active),
	.shifter_done_flag			(shifter_done_flag),
	
	.vector_cop_instruction		(vector_cop_instruction),		// it is a vector coprocessor instruction
	.vector_cop_scalar_skip		(vector_cop_scalar_skip),		// vector COP instruction only, scalar core can skip it
	.vector_opcode				(vector_opcode),
	.scalarRegOut				(scalarRegOut),
	.s3_vector_cop_scalar_read		(s3_vector_cop_scalar_read),
	.scalarToVectorQ_wrreq		(scalarToVectorQ_wrreq),
	.s3_vector_cop_scalar_write	(s3_vector_cop_scalar_write),
	.s3_vectorToScalarQ_rdreq	(s3_vectorToScalarQ_rdreq),
	.predecode_scalarmem_instr	(predecode_scalarmem_instr),		// predecode scalar memory instruction
	.predecode_vectormem_instr	(predecode_vectormem_instr)		// predecode vector memory instruction

);
defparam the_Datapath.RESET_ADDRESS		= RESET_ADDRESS;
defparam the_Datapath.BREAK_ADDRESS		= BREAK_ADDRESS;
defparam the_Datapath.EXCEPTION_ADDRESS	= EXCEPTION_ADDRESS;
defparam the_Datapath.CPU_ID			= CPU_ID;

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

