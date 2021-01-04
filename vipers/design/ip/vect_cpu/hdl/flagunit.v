//**********************************************************************
// Scalable Vector CPU Project -- Flag logic unit
// Filename: VLane_flagU.v
// Author: Jason Yu
//
// This file contains the flag logical processing unit of Vector Lane
//
//  Copyright (C) 2007 Jason Yu
//
//**********************************************************************

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on
`ifdef	MODELSIM
	`include "../hdl/isa_def.v"
`else
	`include "isa_def.v"
`endif


module flagunit
	#(parameter WIDTH = 1)
	(
		// Primary inputs
		input						clk,
		input	[WIDTH-1:0]			flagInA,			// flag inputs
		input	[WIDTH-1:0]			flagInB,			// flag inputs
		input						scalarRegInIsZero,	// scalar register input is zero (for vector-scalar flag logical operations)
		input	[`ALUOP_WIDTH-1:0]	s3_ALUOp,			// ALUOp for pipe stage 1
	
		// Outputs
		output 	[WIDTH-1:0]	flagOut
	);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------	
reg	[WIDTH-1:0]	s3_flagOut;
reg	[WIDTH-1:0]	s4_flagOut;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Combinational logic
//---------------------------------------------------------------------------------------------------------------------------------------------------	
assign	flagOut = s4_flagOut;

always @(*)
begin
	casex (s3_ALUOp)
		// vector-scalar flag insert
		`ALUOP_VFINS_VS		:	s3_flagOut = ~scalarRegInIsZero;
		// vector-scalar flag logical operations
		`ALUOP_FLAG_VFAND_VS	:	s3_flagOut = flagInA & ~scalarRegInIsZero;
		`ALUOP_FLAG_VFNOR_VS	:	s3_flagOut = ~(flagInA | ~scalarRegInIsZero);
		`ALUOP_FLAG_VFOR_VS		:	s3_flagOut = flagInA | ~scalarRegInIsZero;
		`ALUOP_FLAG_VFXOR_VS	:	s3_flagOut = flagInA ^ ~scalarRegInIsZero;

		// other flag operations
		`ALUOP_FLAG_VFAND	:	s3_flagOut = flagInA & flagInB;
		`ALUOP_FLAG_VFNOR	:	s3_flagOut = ~(flagInA | flagInB);
		`ALUOP_FLAG_VFOR	:	s3_flagOut = flagInA | flagInB;
		`ALUOP_FLAG_VFXOR	:	s3_flagOut = flagInA ^ flagInB;
		`ALUOP_FLAG_VFSET	:	s3_flagOut = {WIDTH{1'b1}};
		`ALUOP_FLAG_VFCLR	:	s3_flagOut = {WIDTH{1'b0}};
		default				:	s3_flagOut = {WIDTH{1'bx}};
	endcase
end


// additional pipeline register to match ALU pipe
always @(posedge clk)
	s4_flagOut <= s3_flagOut;
	
endmodule
