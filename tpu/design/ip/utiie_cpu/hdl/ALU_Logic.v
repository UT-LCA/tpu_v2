/*****************************************************************************
 *                                                                           *
 * Module:       ALU_Logic                                                   *
 * Description:                                                              *
 *      This module is the ALU for the UT IIe processor.                     *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
ALU_Logic the_ALU (
	// Inputs
	.clk						(clk),
	
	.executing_execute_stage	(),

	.decoded_signals			(),
	.is_rtype					(),
	.opA						(),
	.opB						(),
	.opcode						(),

	// Bidirectionals

	// Outputs
	.adder_result				(),
	.cmp_flag					(),
	.logic_result				(),
	.shifter_result				(),

	.shifter_done_flag			(),

	.use_adder_result			(),
	.use_comparator_result		(),
	.use_logic_result			(),
	.use_shifter_result			()
);
*/

module ALU_Logic (
	// Inputs
	clk,

	executing_execute_stage,

	decoded_signals,
	is_rtype,
	opA,
	opB,
	opcode,

	// Bidirectionals

	// Outputs
	adder_result,
	cmp_flag,
	logic_result,
	shifter_result,

	shifter_done_flag,

	use_adder_result,
	use_comparator_result,
	use_logic_result,
	use_shifter_result
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

input	[7:0]	decoded_signals;
input			is_rtype;
input	[31:0]	opA;
input	[31:0]	opB;
input	[5:0]	opcode;

// Bidirectionals

// Outputs
output	[31:0]	adder_result;
output			cmp_flag;
output	[31:0]	logic_result;
output	[31:0]	shifter_result;

output			shifter_done_flag;

output			use_adder_result;
output			use_comparator_result;
output			use_logic_result;
output			use_shifter_result;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire	[32:0]	operandA;
wire	[32:0]	operandB;

// Adder
wire			signed_op_a;
wire			signed_op_b;
wire			sub_cmp;

// Comparator
wire			cmp_run_eq; // if 1 do eq and not eq; else do lt and ge
wire			cmp_eq_lt;	// if 1 do eq or lt; else do not eq and ge
wire			negative_flag;
wire			overflow_flag;
wire			zero_flag;

// Shifter
wire			arithmetic;
wire			direction;
wire			rotate;

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

assign cmp_flag	= ((cmp_run_eq == 1'b1) ? (~(cmp_eq_lt ^ zero_flag)) :
						(~(cmp_eq_lt ^ (negative_flag ^ overflow_flag)))
					);
						
assign use_adder_result 		= (decoded_signals[3:1] == 3'h0) ? 1'b1 : 1'b0;
assign use_comparator_result 	= (decoded_signals[3:2] == 2'h1) ? 1'b1 : 1'b0;
assign use_logic_result 		= (decoded_signals[3:0] == 4'h2) ? 1'b1 : 1'b0;
assign use_shifter_result 		= (decoded_signals[3:0] == 4'h3) ? 1'b1 : 1'b0;

assign operandA		= {(signed_op_a & opA[31]), opA};
assign operandB		= {(signed_op_b & opB[31]), opB};// ^ {33{sub_cmp}};

assign signed_op_a 	= ((opcode[5:3] == 3'b110) || (opcode[5:3] == 3'b101)) ? 
						1'b0 : 1'b1;
assign signed_op_b 	= signed_op_a;
assign sub_cmp 		= decoded_signals[5];

assign cmp_run_eq 	= decoded_signals[1];
assign cmp_eq_lt 	= decoded_signals[0];

assign arithmetic 	= opcode[5];
assign direction 	= opcode[3];
assign rotate 		= ~opcode[4];


/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Adder_w_Comparator the_Adder_w_Comparator (
	// Inputs
	.operandA		(operandA),
	.operandB		(operandB),
	.sub			(sub_cmp),
	
	// Bidirectionals

	// Outputs
	.result			(adder_result),

	.negative_flag	(negative_flag),
	.overflow_flag	(overflow_flag),
	.zero_flag		(zero_flag)
);

Logic_Unit the_Logic_Unit (
	// Inputs
	.opA			(opA),
	.opB			(opB),
	.opcode			(opcode),

	// Bidirectionals

	// Outputs
	.logic_result	(logic_result)
);
/*
Shifter_Unit the_Shifter (
	// Inputs
	.clk			(clk),
	
	.opA			(opA[31:0]),
	.opB			(opB[4:0]),

	.arithmetic		(arithmetic),
	.direction		(direction),
	.rotate			(rotate),

	.run_shifter	(use_shifter_result),

	// Bidirectionals

	// Outputs
	.done			(shifter_done_flag),	
	.shifter_result	(shifter_result)
);
*/
Shifter_Unit the_Shifter_Unit (
	// Inputs
	.clk						(clk),
	
	.executing_execute_stage	(executing_execute_stage),
	
	.opA						(opA[31:0]),
	.opB						(opB[4:0]),

//	.decoded_signals	(),
	.is_rtype					(is_rtype),
	.opcode						(opcode),

	// Bidirectionals

	// Outputs
	.done						(shifter_done_flag),	
	.shifter_result				(shifter_result)
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

