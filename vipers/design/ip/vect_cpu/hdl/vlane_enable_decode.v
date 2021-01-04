//************************************************************************
// Scalable Vector CPU Project -- Vector lane enable signal decoder
// Filename: vlane_enable_decode.v
// Author: Jason Yu
//
// Generate enable signals to vlane based on which lanes are enabled for the current instruction
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
`else
	`include "define.v"
	`include "config_def.v"
`endif


module vlane_enable_decode
#(	// Vector processor primary parameters
	parameter	NUMLANES = 1
)
(
	input		s3_decode_onelane_en,					// decode only the last element
	input		[`log2(NUMLANES):0]	s3_cycleLastEnabledLane,		// remainder + 1 of dividing vlength/vindex by NUMLANE; range = 1 to NUMLANE
	input		[`log2(NUMLANES)-1:0]	s3_cycleFirstLaneIndex,			// range = 0 to NUMLANE-1
	
	output reg	[NUMLANES-1:0]			s3_vl_lane_en		// lane enable signal computed from vector length	
);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
integer lanecount;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Signal assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------

// Determine lane squash signals for the last cycle from vector length
// decode lane enable signals from vector length
always @(*) begin
	for (lanecount=0; lanecount<NUMLANES; lanecount=lanecount+1) begin
		// if the operation only needs to operate on the last element
		if (s3_decode_onelane_en && (lanecount == s3_cycleFirstLaneIndex))
			s3_vl_lane_en[lanecount] = 1'b1;
		// if the operation needs to operate on all elements up within the first to last element range
		else if (~s3_decode_onelane_en && (lanecount >= s3_cycleFirstLaneIndex) && (lanecount < s3_cycleLastEnabledLane))
			s3_vl_lane_en[lanecount] = 1'b1;
		// default assignment
		else
			s3_vl_lane_en[lanecount] = 1'b0;
	end
end

endmodule
