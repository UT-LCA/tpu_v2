/*****************************************************************************
 *                                                                           *
 * Module:       Program_Counter                                             *
 * Description:                                                              *
 *      This module stores and calculates the program counter                *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Program_Counter the_Program_Counter (
	// Inputs
	.clk				(clk),
	.reset_n			(reset_n),

	.branch_offset		(),
	.branch_stage		(),
	.branch_taken		(),
	.branch_type		(),
	.jmp_addr			(),

	.inc_pc				(),

	// Bidirectionals

	// Outputs
	.program_counter	()
);
*/

module Program_Counter (
	// Inputs
	clk,				// Global Clock
	reset_n,			// if 0, the cpu is being reset
	vector_cop_scalar_skip,

	branch_offset,		// Contains the offset for branches
	branch_stage,		// 1 indicates that the processor is in 
						//   the execute/branch stage
	branch_taken,		// 1 indicates that a branch has been taken 
						//   and the PC should be updated
	branch_type,		// Indicates the type of control flow 
						//   instruction: 0 - jmp, 1 - branch
	jmp_addr,			// Address to jmp to for none offset 
						//   control flow instructions

	inc_pc,				// if 1, the PC should be incremented by 4

	// Bidirectionals

	// Outputs
	program_counter		// the current pc
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/
// States
parameter RESET_ADDRESS = 32'h00000000;


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input			clk;
input			reset_n;
input			vector_cop_scalar_skip;

input	[15:0]	branch_offset;
input			branch_stage;
input			branch_taken;
input			branch_type;
input	[31:0]	jmp_addr;

input			inc_pc;

// Bidirectionals

// Outputs
output 	[31:0]	program_counter;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire			cout_of_two_bit_adder;
wire	[31:0]	pc_plus_offset;
wire	[31:2]	branch_offset_mask;

// Internal Registers
reg		[31:0]	program_counter;

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

// always @(posedge clk)	// sync reset
always @(posedge clk, negedge reset_n)
begin
	if (reset_n == 1'b0)
	begin
		program_counter <= RESET_ADDRESS;
	end
	else if ((branch_stage == 1'b1) && (branch_taken == 1'b1))
	begin
		if (branch_type == 1'b1)
		begin
			program_counter <= pc_plus_offset;
		end
		else
		begin
			program_counter <= jmp_addr;
		end
	end
	else if (inc_pc == 1'b1)
	begin
		program_counter <= pc_plus_offset;
	end
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

// If vector instruction, it cannot be a scalar branch instruction, therefore can simply fetch instruction from next memory location
assign	branch_offset_mask [31:2] = {{16{branch_offset[15]}}, branch_offset[15:2]} & {30{~vector_cop_scalar_skip}};
 
/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

lpm_add_sub	Two_Bit_Adder (
	.dataa		(program_counter[1:0]),
	.datab		(branch_offset[1:0]),
	.cout		(cout_of_two_bit_adder),
	.result		(pc_plus_offset[1:0])
	// synopsys translate_off
	,
	.aclr		(),
	.add_sub	(),
	.cin		(),
	.clken		(),
	.clock		(),
	.overflow	()
	// synopsys translate_on
);
defparam
	Two_Bit_Adder.lpm_width = 2,
	Two_Bit_Adder.lpm_direction = "ADD",
	Two_Bit_Adder.lpm_type = "LPM_ADD_SUB",
	Two_Bit_Adder.lpm_hint = "ONE_INPUT_IS_CONSTANT=NO,CIN_USED=NO";



lpm_add_sub	Thirty_Bit_Adder (
	.dataa		(program_counter[31:2]),
	// .datab		({{16{branch_offset[15]}}, branch_offset[15:2]}),
	.datab		(branch_offset_mask),
	.cin		(inc_pc | cout_of_two_bit_adder),
	.result		(pc_plus_offset[31:2])
	// synopsys translate_off
	,
	.aclr		(),
	.add_sub	(),
	.clken		(),
	.clock		(),
	.cout		(),
	.overflow	()
	// synopsys translate_on
);
defparam
	Thirty_Bit_Adder.lpm_width = 30,
	Thirty_Bit_Adder.lpm_direction = "ADD",
	Thirty_Bit_Adder.lpm_type = "LPM_ADD_SUB",
	Thirty_Bit_Adder.lpm_hint = "ONE_INPUT_IS_CONSTANT=NO,CIN_USED=YES";


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

