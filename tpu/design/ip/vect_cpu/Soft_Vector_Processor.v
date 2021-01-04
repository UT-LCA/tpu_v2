// *************************************************************************************************************************
// Scalable Vector CPU Project -- wrapper for processor
// Filename: Soft_Vector_Processor.v
// Author: Jason Yu
//
// Wrapper for vector processor for use with SOPC Builder
//
//  Copyright (C) 2007 Jason Yu
//
// *************************************************************************************************************************

// Modelsim
`timescale 1ns / 100ps


module	Soft_Vector_Processor

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Customizable parameters at generation
//---------------------------------------------------------------------------------------------------------------------------------------------------
#(
	// need to specify in decimal due to a bug in SOPC builder
	parameter	RESET_ADDRESS		= 0,
	parameter	BREAK_ADDRESS		= 0,
	parameter	EXCEPTION_ADDRESS	= 32,
	parameter	CPU_ID				= 3,
	// parameter	RESET_ADDRESS		= 32'h00000000,
	// parameter	BREAK_ADDRESS		= 32'h00000000,
	// parameter	EXCEPTION_ADDRESS	= 32'h00000020,
	// parameter	CPU_ID				= 32'h00000003,
	
	// Vector processor primary parameters
	parameter	VPU_WIDTH			= 32,
	parameter	NUMLANES			= 8,
	parameter	MEM_WIDTH			= 128,
	parameter	MINDATAWIDTH		= 8
)

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Port declaration
//---------------------------------------------------------------------------------------------------------------------------------------------------
(
	// Global signals
	input						csi_clockreset_clk,
	input						csi_clockreset_reset_n,
	// input	[31:0]				inr_data_master_irq,
	input	[31:0]				inr_receiver_irq,
	
	// Avalon Master 1 : Instruction master
	input	[31:0]				avm_instruction_master_readdata,
	input						avm_instruction_master_waitrequest,

	// Avalon Master 2: Data master
	// input	[31:0]				avm_data_master_irq,
	input	[MEM_WIDTH-1:0]		avm_data_master_readdata,	// data from main memory
	input						avm_data_master_waitrequest,
	
	// Outputs
	// Avalon Master 1: Instruction master
	output	[31:0]				avm_instruction_master_address,
	output						avm_instruction_master_read,

	// Avalon Master 2: Data master
	output	[MEM_WIDTH/8-1:0]	avm_data_master_byteenable,
	output	[MEM_WIDTH-1:0]		avm_data_master_writedata,
	output	[31:0]				avm_data_master_address,
	output						avm_data_master_read,
	output						avm_data_master_write	
);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Instantiate the processor
//---------------------------------------------------------------------------------------------------------------------------------------------------

VectCPU
// Pass parameters
#(	.RESET_ADDRESS		(RESET_ADDRESS),
	.BREAK_ADDRESS		(BREAK_ADDRESS),
	.EXCEPTION_ADDRESS	(EXCEPTION_ADDRESS),
	.CPU_ID				(CPU_ID),

	// vector processor primary parameters
	.VPU_WIDTH			(VPU_WIDTH),
	.NUMLANES			(NUMLANES),
	.MEM_WIDTH			(MEM_WIDTH),
	.MINDATAWIDTH		(MINDATAWIDTH)
)
the_VectCPU
(
	// Global signals
	.clk					(csi_clockreset_clk),
	.reset_n				(csi_clockreset_reset_n),
	
	// Avalon Master 1 : Instruction master
	.i_readdata				(avm_instruction_master_readdata),
	.i_waitrequest			(avm_instruction_master_waitrequest),

	// Avalon Master 2: Data master
	.d_irq					(inr_receiver_irq),
	.d_readdata				(avm_data_master_readdata),	// data from main memory
	.d_waitrequest			(avm_data_master_waitrequest),
	
	// Outputs
	// Avalon Master 1: Instruction master
	.i_address				(avm_instruction_master_address),
	.i_read					(avm_instruction_master_read),

	// Avalon Master 2: Data master
	.d_byteenable			(avm_data_master_byteenable),
	.d_writedata			(avm_data_master_writedata),
	.d_address				(avm_data_master_address),
	.d_read					(avm_data_master_read),
	.d_write				(avm_data_master_write)
);

endmodule
