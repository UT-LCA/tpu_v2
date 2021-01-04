//************************************************************************
// Scalable Vector CPU Project -- Scalar core predecode
// Filename: Predecode_Instruction.v
// Author: Jason Yu
//
// VPU pipeline hazard detection
// - detects all RAW pipeilne hazards
// - detects all other instances where the processor needs to be stalled
// - generates fetch stall signal to scalar core to stall instruction fetching
//
//  Copyright (C) 2008 Jason Yu
//
//************************************************************************

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

module	Predecode_Instruction

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Ports
//---------------------------------------------------------------------------------------------------------------------------------------------------
(
	// ***********
	// ** Inputs **
	// ***********
	input	[31:0]  fetched_instruction,
	input			fetch_completed,
	input			pipeline_stalled,

	output			scalar_memory_instruction,
	output			vector_cop_memory_instruction
);

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
wire	[5:0]	opcode;
// wire			vector_cop_opx;
reg				scalar_memory_instruction_sig;
reg				vector_cop_memory_instruction_sig;

 
/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/
assign	opcode = fetched_instruction[5:0];
// assign	vector_cop_opx = fetched_instruction[12];
assign	scalar_memory_instruction = fetch_completed & (~pipeline_stalled) & scalar_memory_instruction_sig;
assign	vector_cop_memory_instruction = fetch_completed & (~pipeline_stalled) & vector_cop_memory_instruction_sig;


always @(*) begin
	scalar_memory_instruction_sig = 1'b0;
	vector_cop_memory_instruction_sig = 1'b0;
	
	case(opcode)
		// load and store byte, halfword
		6'h03, 6'h05, 6'h07, 6'h0B, 6'h0D, 6'h0F,
		// load and store word
		6'h15, 6'h17,
		// load and store byte io, halfword io
		6'h23, 6'h25, 6'h27, 6'h2B, 6'h2D, 6'h2F,
		// load and store word io
		6'h35, 6'h37:
			scalar_memory_instruction_sig = 1'b1;
		
		// vector memory
		6'h3F: begin
			// if (vector_cop_opx == 1'b0)
				vector_cop_memory_instruction_sig = 1'b1;
			// else
				// vector_cop_memory_instruction_sig = 1'b0;
		end
		default:	;
	endcase
end

endmodule
