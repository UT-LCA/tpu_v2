//************************************************************************
// Scalable Vector CPU Project -- Decoder for shift datapath
// Filename: shiftdecode.v
// Author: Jason Yu
//
// Controller for shift instructions
// - generates control signals to the barrel shifter for logical/arithmetic/barrel shifts
//
//  Copyright (C) 2007 Jason Yu
//
//************************************************************************

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on
`ifdef	MODELSIM
	`include "../hdl/define.v"
	`include "../hdl/isa_def.v"
`else
	`include "define.v"
	`include "isa_def.v"
`endif


module	shiftdecode
#(	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1
)
(
	input								s4_shift_instr,		// instruction is a shift instruction
	input		[2:0]					s4_shift_command,	// shift command; lower 3 bits of ALUOp
	input		[`log2(VPU_WIDTH)-1:0]	s4_shiftAmount,		// shift amount

	output		[`log2(VPU_WIDTH)-1:0]	s4_stageShift,		// select whether to shift for the particular stage
	output reg	[VPU_WIDTH-1:0]			s4_logicalMask		// mask that has zeros in positions that need to be zero'd for logical shifting;
															// used in arith shift as well
);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Internal signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	[`log2(VPU_WIDTH)-1:0]		s4_shiftAmount_right;		// amount to shift, default to the right
wire	[`log2(VPU_WIDTH)-1:0]		s4_shiftAmount_mask;		// amount to shift, masked by shift instruction
wire	[`log2(VPU_WIDTH)-1:0]		shiftAmount_addinA;
wire	[`log2(VPU_WIDTH)-1:0]		shiftAmount_addinB;
reg		leftshift;
integer	i;


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Generate shift signal for each stage
// - each stage selects between passthrough and a fixed shift amount equal to 2^(numstage-stage-1)
//---------------------------------------------------------------------------------------------------------------------------------------------------
// the stage shift signal is identical to the shift amount signal
// each bit in shift amount specifies whether to shift that particular MUX stage
// e.g. to shift 25 (11001), we need to shift by 16, 8, and 1
assign	s4_stageShift = s4_shiftAmount_right;


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Generate shift amount
//---------------------------------------------------------------------------------------------------------------------------------------------------
// not shift unless shift instruction
assign	s4_shiftAmount_mask = (s4_shift_instr == 1'b0) ? {`log2(VPU_WIDTH){1'b0}} : s4_shiftAmount;

// generate 1's complement for subtraction
assign	shiftAmount_addinA = (leftshift == 1'b1) ? VPU_WIDTH : {`log2(VPU_WIDTH){1'b0}};
assign	shiftAmount_addinB = (leftshift == 1'b1) ? ~s4_shiftAmount_mask : s4_shiftAmount_mask;


// determine whether it is a leftshift
always @(*)
begin
	leftshift = 1'b0;
	
	// if shift instruction
	if (s4_shift_instr) begin
		case (s4_shift_command)
			`SHIFT_CMD_VSRA	:	leftshift = 1'b0;
			`SHIFT_CMD_VSLL	:	leftshift = 1'b1;
			`SHIFT_CMD_VSRL	:	leftshift = 1'b0;
			`SHIFT_CMD_VROT	:	leftshift = 1'b0;
			default	:	leftshift = 1'b0;
		endcase
	end
end


// right shift: result = 0 + s4_shiftAmount
// left shift: result = VPU_WIDTH - s4_shiftAmount
lpm_add_sub		shiftAmount_adder
(
	.dataa 		( shiftAmount_addinA ),
	.datab 		( shiftAmount_addinB ),
	.cin 		( leftshift ),				// manually generate cin for subtraction
	.result		( s4_shiftAmount_right )
	// synopsys translate_off
	,
	.cout 		(  ),
	.overflow	(  ),
	.add_sub(),
	.clken (),
	.clock (),
	.aclr ()
	// synopsys translate_on
);
defparam	shiftAmount_adder.lpm_width = `log2(VPU_WIDTH),
			shiftAmount_adder.lpm_direction = "ADD";	// manually generate ~B and cin for subtraction


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Decode the shift amount into the decode mask
// - the decode mask has 0's where the shifted result has 0's, and 1's in other positions
//---------------------------------------------------------------------------------------------------------------------------------------------------

// special decoder that creates the decode mask
always @(*) begin
	// default assignment
	// non-shift instructions, barrel shift instructions
	s4_logicalMask = {VPU_WIDTH{1'b1}};
	i = 0;
	
	// if shift instruction
	if (s4_shift_instr) begin
		case (s4_shift_command)
			`SHIFT_CMD_VSRL,
			`SHIFT_CMD_VSRA	:
			begin
				// shift right
				for (i=0; i<VPU_WIDTH; i=i+1) begin
					if ((VPU_WIDTH-1-i) < s4_shiftAmount)
						s4_logicalMask[i] = 1'b0;
					else
						s4_logicalMask[i] = 1'b1;
				end
			end
			
			`SHIFT_CMD_VSLL	:
			begin
				// shift left
				for (i=0; i<VPU_WIDTH; i=i+1) begin
					if (i < s4_shiftAmount)
						s4_logicalMask[i] = 1'b0;
					else
						s4_logicalMask[i] = 1'b1;
				end
			end
				
			default	:	s4_logicalMask = {VPU_WIDTH{1'b1}};
		endcase
	end
end


endmodule
