/*****************************************************************************
 *                                                                           *
 * Module:       Instruction_Fetch_Unit                                      *
 * Description:                                                              *
 *      This module fetches instructions and stores the program counter.     *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on
`ifdef	MODELSIM
	`include "../../vect_cpu/hdl/isa_def.v"
`else
	`include "isa_def.v"
`endif

/*
Instruction_Fetch_Unit the_Instruction_Fetch_Unit (
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),

	.executing_irq_stage		(),
	.pipeline_stalled			(),

	.branch_offset				(),
	.branch_stage				(),
	.branch_taken				(),
	.branch_type				(),
	.jmp_addr					(),
	
	.data_in_from_memory		(),
	.memory_requests_wait		(),

	.initiate_instruction_fetch	(),
	.increment_program_counter	(),

	// Bidirectionals

	// Outputs
	.fetched_instruction		(),
	.fetch_completed			(),
	
	.memory_address_to_read		(),
	.read_memory				(),
	
	.program_counter			()
);
*/

module Instruction_Fetch_Unit (
	// Inputs
	clk,							// Global Clock
	reset_n,						// if 0, the cpu is being reset

	executing_irq_stage,			// Interrupt stage is active if 1
	pipeline_stalled,				// Pipeline has stalled if 1

	branch_offset,					// Contains the offset for branches
	branch_stage,					// 1 indicates that the processor is in 
									//   the execute/branch stage
	branch_taken,					// 1 indicates that a branch has been taken 
									//   and the PC should be updated
	branch_type,					// Indicates the type of control flow 
									//   instruction: 0 - jmp, 1 - branch
	jmp_addr,						// Address to jmp to for none offset 
									//   control flow instructions
	
	data_in_from_memory,			// Contains the data return from the memory
	memory_requests_wait,			// 1 indicates that the fetch has not
									//   completed, and the read address and
									//	 read control signal must stay
									//	 constant

	initiate_instruction_fetch,		// if 1, fetching should be started
	increment_program_counter,		// if 1, the PC should be incremented by 4

	// Bidirectionals

	// Outputs
	fetched_instruction,			// Contains the fetched instruction
	fetch_completed,				// 1 indicates that the fetch has completed
	vector_cop_instruction,			// a vector coprocessor instruction has been fetched
	vector_cop_scalar_skip,			// vector COP instruction only, scalar core can skip it
	vector_cop_scalar_read,			// the vector COP instruction needs to read from scalar register
	vector_cop_scalar_write,		// vector instruction needs to write to scalar reg
	
	memory_address_to_read,			// Address of the instruction to fetch			
	read_memory,					// 1 indicates memory read
	
	program_counter					// Contains the current value of the PC
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

// States
parameter	STATE_IDLE		= 1'b0, 
			STATE_FETCHING	= 1'b1;

parameter	RESET_ADDRESS	= 32'h00000000;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/

// Inputs
input			clk;
input			reset_n;

input			executing_irq_stage;
input			pipeline_stalled;

input	[15:0]	branch_offset;
input			branch_stage;
input			branch_taken;
input			branch_type;
input	[31:0]	jmp_addr;

input	[31:0]	data_in_from_memory;
input			memory_requests_wait;

input			initiate_instruction_fetch;
input			increment_program_counter;

// Bidirectionals

// Outputs
output reg	[31:0]  fetched_instruction;
output			fetch_completed;
output			vector_cop_scalar_skip;		// vector COP instruction only, scalar core can skip it
output			vector_cop_instruction;		// it is a vector coprocessor instruction
output reg		vector_cop_scalar_read;			// the vector COP instruction needs to read from scalar register
output reg		vector_cop_scalar_write;		// vector instruction needs to write to scalar reg

output 	[31:0]	memory_address_to_read;
output			read_memory;

output 	[31:0]	program_counter;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire 			instr_muli;
wire 			instr_mulxuu;
wire			force_trap;

wire		[5:0]	OP;
wire		[5:0]	OPX;

wire		[5:0]	vector_cop_func;
wire			vector_cop_opx;
// wire			vector_cop_vmstc;
// wire			scalar_vector_instr;
// wire			vector_vc_movtoscalar;		// move from vector control reg to scalar reg
// wire			vector_vextsv;				// vector-scalar extract
wire		[4:0]	s2_vcop_scalar_rdaddr;	// scalar rdaddr field before replacing the instruction

// Internal Registers

// State Machine Registers
reg				ns_fetch;
reg				s_fetch;

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

always @(posedge clk, negedge reset_n)
begin
	// if (reset_n == 1'b0)
	// begin
		// s_fetch <= STATE_FETCHING;
	// end
	if (reset_n == 1'b0)
	begin
		s_fetch <= STATE_FETCHING;
	end
	else
	begin
		s_fetch <= ns_fetch;
	end
end

always @(*)
begin
	// Defaults
	ns_fetch = STATE_IDLE;

    case (s_fetch)
	STATE_IDLE:
		begin
			if (initiate_instruction_fetch == 1'b1)
			begin
				ns_fetch = STATE_FETCHING;
			end
			else
			begin
				ns_fetch = STATE_IDLE;
			end
		end
	STATE_FETCHING:
		begin
			// if ((memory_requests_wait == 1'b1) || (pipeline_stalled == 1'b1))
			if ((memory_requests_wait == 1'b1) || (pipeline_stalled == 1'b1) || vector_cop_scalar_skip)
			begin
				ns_fetch = STATE_FETCHING;
			end
			else
			begin
				ns_fetch = STATE_IDLE;
			end
		end
	default:
		begin
			ns_fetch = STATE_IDLE;
		end
	endcase
end

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

/*****************************************************************************
 *                            Vector coproceesor instruction handling                            *
 *****************************************************************************/
// Use 3-state comparison so signals are valid in simulation when opcode becomes all xxxx

// a vector coprocessor instruction has been fetched
assign	vector_cop_instruction = (((OP === 6'h3D) || (OP === 6'h3E) || (OP === 6'h3F)) & fetch_completed) ? 1'b1 : 1'b0;

// VMSTC instruction: move from scalar to vector control register
assign	vector_cop_opx = data_in_from_memory[12];
assign	vector_cop_func = data_in_from_memory[11:6];

// Vector COP instruction only, scalar core can skip it
// More additions later...
assign	vector_cop_scalar_skip = vector_cop_instruction & (~vector_cop_scalar_read) & (~vector_cop_scalar_write);

// Instructions that need to read from or write to scalar registers
//assign	vector_cop_scalar_read = vector_cop_vmstc | scalar_vector_instr;
//assign	vector_cop_scalar_write = vector_vc_movtoscalar | vector_vextsv;

// Decode whether the vector instruction needs to write to or read from scalar core
always @(*) begin
	// default assignment
	vector_cop_scalar_write = 1'b0;
	vector_cop_scalar_read = 1'b0;
	
	case (OP)
		6'h3D: begin
			case (vector_cop_func)
				// VEXT.VS instruction: vector-scalar extract
				`ALUOP_VEXT,
				`ALUOP_VEXTU_VS:
					if (vector_cop_opx === 1'b1)
						vector_cop_scalar_write = 1'b1;
				// scalar vector instructions which will require reading from scalar register; excluding vector extract (vext.vs)
				default	:
					if (vector_cop_opx === 1'b1)
						vector_cop_scalar_read = 1'b1;
			endcase	
		end
		
		6'h3E: begin
			case (vector_cop_func)
				// VMCTS instruction: move from vector control register to scalar register
				`FUNC_MCTS	:
					if (vector_cop_opx === 1'b1)
						vector_cop_scalar_write = 1'b1;
				
				// VMSTC instruction: move from scalar to vector control register
				`FUNC_MSTC	:
					if (vector_cop_opx === 1'b1)
						vector_cop_scalar_read = 1'b1;
						
				// vector-scalar flag logical instructions
				`ALUOP_FLAG_VFAND_VS,
				`ALUOP_FLAG_VFNOR_VS,
				`ALUOP_FLAG_VFOR_VS,
				`ALUOP_FLAG_VFXOR_VS,
				// VFINS.VS instruction: vector-scalar flag insert
				`ALUOP_VFINS_VS	:
					if (vector_cop_opx === 1'b1)
						vector_cop_scalar_read = 1'b1;
				
				default	: ;
			endcase
		end
		
		6'h3F: begin
			case (vector_cop_func)
				// vector lane local store, scalar data
				`FUNC_MEM_LOCALSTORE	:
					if (vector_cop_opx === 1'b1)
						vector_cop_scalar_read = 1'b1;
						
				default	: ;
			endcase
		end
		default : ;
	endcase
end
	

// scalar vector instructions which will require reading from scalar register; excluding vector extract (vext.sv)
// assign	scalar_vector_instr = ((OP === 6'h3D) && (vector_cop_opx === 1'b1) && (vector_vextsv === 1'b0))
								// and vector lane local store, scalar data
								// || ((OP === 6'h3F) && (vector_cop_func === `FUNC_MEM_LOCALSTORE) && (vector_cop_opx === 1'b1)) ? 1'b1 : 1'b0;
// VMSTC instruction: move from scalar to vector control register
//assign	vector_cop_vmstc = ((OP === 6'h3E) && (vector_cop_func === `FUNC_MSTC) && (vector_cop_opx === 1'b1)) ? 1'b1 : 1'b0;
// VMCTS instruction: move from vector control register to scalar register
//assign	vector_vc_movtoscalar = ((OP === 6'h3E) && (vector_cop_func === `FUNC_MCTS) && (vector_cop_opx === 1'b1)) ? 1'b1 : 1'b0;
// VEXT.SV instruction: vector-scalar extract
//assign	vector_vextsv = ((OP === 6'h3D) && (vector_cop_func === `ALUOP_VEXT) && (vector_cop_opx === 1'b1)) ? 1'b1 : 1'b0;
// VFINS.VS instruction: vector-scalar flag insert
//assign	vector_vfinsvs = ((OP === 6'h3F) && (vector_cop_func === `ALUOP_VFINS_VS) && (vector_cop_opx === 1'b0)) ? 1'b1 : 1'b0;

 
/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign OP = data_in_from_memory[5:0];
assign OPX = data_in_from_memory[16:11];

assign instr_muli = (OP == 6'h24) ? 1'b1 : 1'b0;
assign instr_mulxuu = ((OP == 6'h3A) && (OPX == 6'h07)) ? 1'b1 : 1'b0;


assign force_trap = executing_irq_stage | instr_muli | instr_mulxuu;

// assign fetched_instruction		= (force_trap == 1'b1) ? 
									// 31'h003B683A :
									// data_in_from_memory;

// keep the scalar read field of the original instruction for vector scalar read
// since we're going to replace the actual instruction with "add rC, rA, rB"
assign	s2_vcop_scalar_rdaddr = data_in_from_memory[`OP_SCALAR_RDADDR];

									
// Override the fetched instruction
always @(*) begin
	if (force_trap == 1'b1)
		fetched_instruction = 31'h003B683A;
	// if instruction reads from scalar register, override instruction into a "add rC, rA, rB"
	else if (vector_cop_scalar_read == 1'b1)
		// mask out OPX, OP parts and replace with add; replace rA with vector instruction format scalar rdaddr operand
		fetched_instruction = (data_in_from_memory & 32'h07FE0000) | 32'h0001883A | {s2_vcop_scalar_rdaddr, {27'b0}};
	else if (vector_cop_scalar_write == 1'b1)
		// mask out OPX, OP parts and replace with add
		fetched_instruction = (data_in_from_memory & 32'hFFFE0000) | 32'h0001883A;
	else
		fetched_instruction = data_in_from_memory;
end


assign fetch_completed			= (s_fetch == STATE_FETCHING) & 
									~memory_requests_wait & reset_n;

// immediately fetch the next instruction if the fetched instruction is a vector coprocessor instruction that doesn't need to be processed by the scalar core
// assign fetch_completed			= (s_fetch == STATE_FETCHING) & 
									// ~memory_requests_wait & reset_n & ~vector_cop_scalar_skip;

//assign fetch_completed			= ((s_fetch == STATE_FETCHING) & 
//									~memory_requests_wait & reset_n) |
//									executing_irq_stage;

assign memory_address_to_read	= program_counter;
assign read_memory				= (s_fetch == STATE_FETCHING) | 
									initiate_instruction_fetch;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Program_Counter the_Program_Counter (
	// Inputs
	.clk				(clk),
	.reset_n			(reset_n),
	.vector_cop_scalar_skip	(vector_cop_scalar_skip),

	.branch_offset		(branch_offset),
	.branch_stage		(branch_stage),
	.branch_taken		(branch_taken),
	.branch_type		(branch_type),
	.jmp_addr			(jmp_addr),

	.inc_pc				(increment_program_counter),

	// Bidirectionals

	// Outputs
	.program_counter	(program_counter)
);
defparam the_Program_Counter.RESET_ADDRESS = RESET_ADDRESS;

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

