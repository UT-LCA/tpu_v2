/*****************************************************************************
 *                                                                           *
 * Module:       Shifter_Unit                                                *
 * Description:                                                              *
 *      This module performs shift and rotate instructions, by rotate the    *
 *   operand one bit at a time.                                              *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Shifter_Unit the_Shifter_Unit (
	// Inputs
	.clk						(clk),
	
	.executing_execute_stage	(),
	
	.opA						(),
	.opB						(),

//	.decoded_signals	(),
	.is_rtype					(),
	.opcode						(),

	// Bidirectionals

	// Outputs
	.done						(),	
	.shifter_result				()
);
*/

module Shifter_Unit (
	// Inputs
	clk,

	executing_execute_stage,
	
	opA,
	opB,

//	decoded_signals,
	is_rtype,
	opcode,

	// Bidirectionals

	// Outputs
	done,	
	shifter_result
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input 			clk;

input			executing_execute_stage;

input	[31:0] 	opA;
input	[4:0] 	opB;

//input	[7:0]	decoded_signals;
input			is_rtype;
input	[5:0] 	opcode;

// Bidirectionals

// Outputs
output 			done;	
output [31:0] 	shifter_result;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire 			arithmetic;
wire 			direction;
wire 			rotate;

wire 			run_shifter;

wire [31:0] 	current_result;

// Internal Registers
reg [31:0] 		result_reg;
reg [4:0] 		shifter_counter;
reg 			started_shifter;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

always @ (posedge clk or negedge run_shifter)
begin
	if (run_shifter == 1'b0)
	begin
		started_shifter <= 1'b0;
	end
	else
	begin
		if (done == 1'b1)
		begin
			started_shifter <= 1'b0;
		end
		else
		begin
			started_shifter <= 1'b1;
		end
	end
end

// EDIT: Jason Yu, August 14, 07
// Original code
// always @ (posedge clk or negedge started_shifter)
// begin
	// if (started_shifter == 1'b0)
	// begin
		// shifter_counter <= opB;
	// end
	// else
	// begin
		// shifter_counter <= shifter_counter - 5'b00001;
	// end
// end

// EDIT: Jason Yu, August 14, 07
// Quartus synthesized presettable register changed into latch which caused hold time problems
// started_shifter is actually synchronous signal, so just make it synchronous load
always @ (posedge clk)
begin
	if (started_shifter == 1'b0)
	begin
		shifter_counter <= opB;
	end
	else
	begin
		shifter_counter <= shifter_counter - 5'b00001;
	end
end

always @ (posedge clk/* or negedge started_shifter*/)
begin
	//if (run_shifter == 1'b0)
	//begin
	//	result_reg <= 32'h00000000;
	//end
	/*else*/ if (started_shifter == 1'b0)
	begin
		result_reg <= opA;
	end
	else
	begin
		if (done == 1'b0)
		begin
			result_reg <= current_result;
		end
		else
		begin
			result_reg <= result_reg;
		end
	end
end

reg reg_done;

always @ (posedge clk/* or negedge started_shifter*/)
begin
	reg_done <= done;
end


/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign arithmetic 	= opcode[5];
assign direction 	= opcode[3];
assign rotate 		= ~opcode[4];

assign run_shifter	= is_rtype & ~opcode[2] & opcode[1] & 
				(executing_execute_stage | ~reg_done);
	//(decoded_signals[3:0] == 4'h3) ? 1'b1 : 1'b0;

assign shifter_result = (run_shifter == 1'b1) ? result_reg : 32'h00000000;

assign done = (((shifter_counter == 0) && (started_shifter == 1'b1)) || 
				(run_shifter == 1'b0)) ? 1'b1 : 1'b0;

assign current_result[31] = ((direction == 1'b1) ? 
								((result_reg[0] & rotate) | (result_reg[31] & (arithmetic)))
								:
								(result_reg[30]));
assign current_result[30:1] = ((direction == 1'b1) ? 
								result_reg[31:2]
								:
								result_reg[29:0]);
assign current_result[0] = ((direction == 1'b1) ? 
								(result_reg[1])
								:
								(result_reg[31] & rotate));

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

