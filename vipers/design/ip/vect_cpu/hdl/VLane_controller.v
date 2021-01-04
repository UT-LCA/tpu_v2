//************************************************************************
// Scalable Vector CPU Project -- Vector lane controller
// Filename: VLane_controller.v
// Author: Jason Yu
//
// Controller for a single vector lane
// - handles squashing of instructions in vector lane depending on mask value
//
//  Copyright (C) 2007 Jason Yu
//
//************************************************************************

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on
`ifdef	MODELSIM
	`include "../hdl/define.v"
	`include "../hdl/config_def.v"
	`include "../hdl/isa_def.v"
`else
	`include "define.v"
	`include "config_def.v"
	`include "isa_def.v"
`endif


module VLane_controller
(
	// Inputs
	input						clk,
	input						reset_n,
	input						s3_instr_nonmaskable,	// indicates the instruction is not maskable; but is still subject to vector length
	input						s3_vflag,				// vector mask for current instruction
	input						s3_vl_lane_en,			// lane enable signal computed from vector length
	input	[`ALUOP_WIDTH-1:0]	dec_s3_ALUOp,
	input						dec_s3_vpmem_wren,		// local memory wren
	input	[`ALUOP_WIDTH-1:0]	dec_s4_ALUOp,
	input						dec_s4_vrf_wren,		// vrf write enable from instruction decoder
	input						dec_s4_vflag_wren,		// vflag write enable from instruction decoder

	output						s4_alu_vflag,
	output	[`ALUOP_WIDTH-1:0]	s3_ALUOp,
	output	[`ALUOP_WIDTH-1:0]	s4_ALUOp,
	output						s3_vpmem_wren,			// local memory wren
	output						s4_vrf_wren,			// Outputs to vector lane datapath
	output						s4_vflag_wren
);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// ---- Local signals ----
//---------------------------------------------------------------------------------------------------------------------------------------------------

wire	s3_vmask_sig;
wire	s3_vmask_on;
wire	s3_squash_mask;
wire	s4_vmask;
wire	s4_vmask_on;

//---------------------------------------------------------------------------------------------------------------------------------------------------
//**** Stage 3: Reg read/Execute ****
//---------------------------------------------------------------------------------------------------------------------------------------------------
// nonmaskable signals are still subject to vector length
assign	s3_vmask_sig = (s3_instr_nonmaskable == 1'b1) ? `FLAG_ON_VALUE : s3_vflag;
assign	s3_vmask_on = ( s3_vmask_sig == `FLAG_ON_VALUE ) ? 1'b1 : 1'b0;

// squash the mask (disable instruction) if last cycle and vector length does not include the current lane
assign	s3_squash_mask = ~s3_vl_lane_en;


// Assign stage outputs
assign	s3_ALUOp = dec_s3_ALUOp & {`ALUOP_WIDTH{s3_vmask_on & ~s3_squash_mask}};		// squash the ALUOp
`ifdef	VP_LOCALMEM
assign	s3_vpmem_wren = dec_s3_vpmem_wren & (s3_vmask_on & ~s3_squash_mask);
`endif


//---------------------------------------------------------------------------------------------------------------------------------------------------
//**** Stage 4: Execute 2/Mem ****
//---------------------------------------------------------------------------------------------------------------------------------------------------
// If instruction is not maskable
assign	s4_vmask_on = (s4_vmask == `FLAG_ON_VALUE) ? 1'b1 : 1'b0;

// Assign stage outputs
assign	s4_vrf_wren = dec_s4_vrf_wren & s4_vmask_on;
assign	s4_vflag_wren = dec_s4_vflag_wren & s4_vmask_on;
assign	s4_ALUOp = dec_s4_ALUOp & {`ALUOP_WIDTH{s4_vmask_on}};


// register the mask
generate
genvar gi;
	for (gi=0; gi<1; gi=gi+1)
	begin: vmask_gen
		// if a flag value of 0 is on (do calculation)
		// if (`FLAG_ON_VALUE == 1'b0) begin
			
			// vmask is execution mask
			ff	#(.WIDTH(1))	stage4_vmask
				(.clk(clk), .clrn(reset_n), .en(1'b1), .d(s3_vmask_sig | s3_squash_mask), .q(s4_vmask));
			// vflag is value from vflag register
			ff	#(.WIDTH(1))	stage4_vflag
				(.clk(clk), .clrn(reset_n), .en(1'b1), .d(s3_vflag), .q(s4_alu_vflag));
		// end
			
		// if a flag value of 1 is on (do calculation)
		// else
			// ff	#(.WIDTH(1))	stage4_vmask
				// (.clk(clk), .clrn(reset_n), .en(1'b1), .d(s3_vmask_sig & ~s3_squash_mask), .q(s4_alu_vflag));
	end
endgenerate


endmodule
