//----------------------------------------------------------------------------------------------------
// Filename: MACunit.v
// Author: Jason Yu
//
// Multiply-accumulate unit
// - the unit uses the altmult_add megafunction to instantiate Stratix III style MAC blocks
// - clearing accumulator is accomplished by using aclr0 on first level multiplication output registers,
// then summing the zeros and loading them into the accumulator using accum_sload
// - aclr were not used for clearing because asynchornous clr can disrupt pipelining
//
// altmult_accum configuration for Stratix III:
// - two clocks, two clk_en, 1 aclr
// - 4th aclr input (connect to ~reset_n)
// - 4 multipliers per MAC block
// - 16 bit inputs, 44 bit output
// - variable sign inputs (clock0; input register and pipeline register)
// - register output of accumulator unit (clock1)
// - add operation on first level multipliers
// - enable chainout adder, chainin input
// - enable zero_chainout output (not registered)
// - register output of first level multipliers (clock0)
// - accumulator add operation
//
//  Copyright (C) 2007 Jason Yu
//
//----------------------------------------------------------------------------------------------------

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


module MACunit
#(	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1,
	parameter	NUMLANES = 1
)
(
	input	clk,
	input	reset_n,
	input	s4_MAC_en,										// enable the MAC registers
	input	id_zeroACC,										// zero the MAC accumulator
	input	s3_zeroACC,										// zero the first stage MAC result registers
	input	s4_zeroMACchainout,								// zero the MAC chainout signals
	input	id_signedOp,									// signed operation
	input	[NUMLANES*VPU_WIDTH-1:0]	vrf_q1_group,		// outputs from register file
	input	[NUMLANES*VPU_WIDTH-1:0]	vrf_q2_group,		// outputs from register file		

	output	[NUMLANES*VPU_WIDTH-1:0]	macResultToLane		// truncated result from MAC unit to lanes
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local parameter
//---------------------------------------------------------------------------------------------------------------------------------------------------
localparam	NUM_CLUSTER = NUMLANES / `MAC_NUMMULT;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	[NUM_CLUSTER*`MAC_INTERNAL_WIDTH-1:0]	macResult_group;		// full width result from MAC unit
wire	[NUM_CLUSTER*`MAC_INTERNAL_WIDTH-1:0]	mac_chainin_group;		// cascade chain signal between MAC units; `MAC_INTERNAL_WIDTHb signal per chain
wire	[NUMLANES*VPU_WIDTH-1:0]	vrf_q1_group_zeroed;		// outputs from register file
wire	[NUMLANES*VPU_WIDTH-1:0]	vrf_q2_group_zeroed;		// outputs from register file

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Combinational logic
//---------------------------------------------------------------------------------------------------------------------------------------------------
// this imitates synchronous clear. asynchronous clear may affect the next pipelined instruction if it happens to be a mac instruction
assign	vrf_q1_group_zeroed = (s3_zeroACC == 1'b1) ? {NUMLANES*VPU_WIDTH{1'b0}} : vrf_q1_group;
assign	vrf_q2_group_zeroed = (s3_zeroACC == 1'b1) ? {NUMLANES*VPU_WIDTH{1'b0}} : vrf_q2_group;


// Connect MAC unit results to vector lanes
genvar gi;
generate
	for (gi=0; gi<NUMLANES; gi=gi+1)
	begin: macresulti
	
		// Connect MAC results to the vector lanes with the smallest indices
		// MAC result is from chain out of last MAC unit in the chain
		// Truncate from `MAC_INTERNAL_WIDTH to lower VPU_WIDTH automatically, for now
		if (gi < (NUM_CLUSTER/`MACCHAINLENGTH))
			assign	macResultToLane [ VPU_WIDTH*(gi+1)-1 : VPU_WIDTH*gi ] = macResult_group [ ((gi+1)*`MACCHAINLENGTH-1)*`MAC_INTERNAL_WIDTH + VPU_WIDTH - 1 : ((gi+1)*`MACCHAINLENGTH-1)*`MAC_INTERNAL_WIDTH ];
		else
			// Connect others to zero
			assign	macResultToLane [ VPU_WIDTH*(gi+1)-1 : VPU_WIDTH*gi ] = {VPU_WIDTH{1'b0}};
	end
endgenerate


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Generate a multiply-accum units
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Generate a multiply-accum unit for each cluster
// Inputs from 4 lanes to each MAC
generate
	for (gi=0; gi<NUM_CLUSTER; gi=gi+1)
	begin: multaccumi

		// Make connections in the chain in signal between MAC units
		// Break chain signal after MACCHAINLENGTH
		if (gi %`MACCHAINLENGTH == 0)
			assign	mac_chainin_group [ (gi+1)*`MAC_INTERNAL_WIDTH-1 : gi*`MAC_INTERNAL_WIDTH ] = `MAC_INTERNAL_WIDTH'b0;
			
		// Propagate chainin from previous result
		else
			assign	mac_chainin_group [ (gi+1)*`MAC_INTERNAL_WIDTH-1 : gi*`MAC_INTERNAL_WIDTH ] = macResult_group [ gi*`MAC_INTERNAL_WIDTH-1 : (gi-1)*`MAC_INTERNAL_WIDTH ];
	
	
		// Each multiply-accum unit has:
		// Fixed 16b input, `MAC_INTERNAL_WIDTHb output
		// chainin signal that can be connected to previous chainout for cascade addition
		// zero_chainout signal that masks the result with zero

	// If MAC uses the lower half of VP for multiplication, for the case VPWIDTH != MAC_WIDTH
	`ifdef	MACVPLOWERHALF
		altMultAccum	altMultAccum_inst (
			.accum_sload ( id_zeroACC ),					// sload zero into accumulator
			.ena0 ( 1'b1 ),
 			.ena1 ( s4_MAC_en),								// enable for accumulator stage
			.aclr3	(~reset_n),
			.chainin ( mac_chainin_group [ (gi+1)*`MAC_INTERNAL_WIDTH-1 : gi*`MAC_INTERNAL_WIDTH ] ),
			.clock0	( clk ),								// clock for first level multiplier results and control inputs
			.clock1 (clk),									// clock for accumulator output
			.dataa_0 ( vrf_q1_group_zeroed [ 4*VPU_WIDTH*gi-1 +`MAC_WIDTH : 4*VPU_WIDTH*gi ] ),
			.dataa_1 ( vrf_q1_group_zeroed [ 4*VPU_WIDTH*gi-1 + VPU_WIDTH + `MAC_WIDTH : 4*VPU_WIDTH*gi + VPU_WIDTH ] ),
			.dataa_2 ( vrf_q1_group_zeroed [ 4*VPU_WIDTH*gi-1 + 2*VPU_WIDTH + `MAC_WIDTH : 4*VPU_WIDTH*gi + 2*VPU_WIDTH ] ),
			.dataa_3 ( vrf_q1_group_zeroed [ 4*VPU_WIDTH*gi-1 + 3*VPU_WIDTH + `MAC_WIDTH : 4*VPU_WIDTH*gi + 3*VPU_WIDTH ] ),
			.datab_0 ( vrf_q2_group_zeroed [ 4*VPU_WIDTH*gi-1 +`MAC_WIDTH : 4*VPU_WIDTH*gi ] ),
			.datab_1 ( vrf_q2_group_zeroed [ 4*VPU_WIDTH*gi-1 + VPU_WIDTH + `MAC_WIDTH : 4*VPU_WIDTH*gi + VPU_WIDTH ] ),
			.datab_2 ( vrf_q2_group_zeroed [ 4*VPU_WIDTH*gi-1 + 2*VPU_WIDTH + `MAC_WIDTH : 4*VPU_WIDTH*gi + 2*VPU_WIDTH ] ),
			.datab_3 ( vrf_q2_group_zeroed [ 4*VPU_WIDTH*gi-1 + 3*VPU_WIDTH + `MAC_WIDTH : 4*VPU_WIDTH*gi + 3*VPU_WIDTH ] ),
			.signa ( id_signedOp ),
			.signb ( id_signedOp ),
			.zero_chainout ( s4_zeroMACchainout ),
			.result ( macResult_group [ (gi+1)*`MAC_INTERNAL_WIDTH-1 : gi*`MAC_INTERNAL_WIDTH ] )
		);
	
	// Else MAC uses upper half of VPU for multiplication
	`else
		altMultAccum	altMultAccum_inst (
			.accum_sload ( id_zeroACC ),						// sload zero into accumulator
			.ena0 ( 1'b1 ),
			.ena1 ( s4_MAC_en),								// enable for accumulator stage
			.aclr3	(~reset_n),
			.chainin ( mac_chainin_group [ (gi+1)*`MAC_INTERNAL_WIDTH-1 : gi*`MAC_INTERNAL_WIDTH ] ),
			.clock0	( clk ),								// clock for first level multiplier results and control inputs
			.clock1 (clk),									// clock for accumulator output
			.dataa_0 ( vrf_q1_group_zeroed [ 4*VPU_WIDTH*gi-1 + VPU_WIDTH : 4*VPU_WIDTH*gi + VPU_WIDTH - `MAC_WIDTH ] ),
			.dataa_1 ( vrf_q1_group_zeroed [ 4*VPU_WIDTH*gi-1 + 2*VPU_WIDTH : 4*VPU_WIDTH*gi + 2*VPU_WIDTH - `MAC_WIDTH ] ),
			.dataa_2 ( vrf_q1_group_zeroed [ 4*VPU_WIDTH*gi-1 + 3*VPU_WIDTH : 4*VPU_WIDTH*gi + 3*VPU_WIDTH - `MAC_WIDTH ] ),
			.dataa_3 ( vrf_q1_group_zeroed [ 4*VPU_WIDTH*gi-1 + 4*VPU_WIDTH : 4*VPU_WIDTH*gi + 4*VPU_WIDTH - `MAC_WIDTH ] ),
			.datab_0 ( vrf_q2_group_zeroed [ 4*VPU_WIDTH*gi-1 + VPU_WIDTH : 4*VPU_WIDTH*gi + VPU_WIDTH - `MAC_WIDTH ] ),
			.datab_1 ( vrf_q2_group_zeroed [ 4*VPU_WIDTH*gi-1 + 2*VPU_WIDTH : 4*VPU_WIDTH*gi + 2*VPU_WIDTH - `MAC_WIDTH ] ),
			.datab_2 ( vrf_q2_group_zeroed [ 4*VPU_WIDTH*gi-1 + 3*VPU_WIDTH : 4*VPU_WIDTH*gi + 3*VPU_WIDTH - `MAC_WIDTH ] ),
			.datab_3 ( vrf_q2_group_zeroed [ 4*VPU_WIDTH*gi-1 + 4*VPU_WIDTH : 4*VPU_WIDTH*gi + 4*VPU_WIDTH - `MAC_WIDTH ] ),
			.signa ( id_signedOp ),
			.signb ( id_signedOp ),
			.zero_chainout ( s4_zeroMACchainout ),
			.result ( macResult_group [ (gi+1)*`MAC_INTERNAL_WIDTH-1 : gi*`MAC_INTERNAL_WIDTH ] )
		);
	`endif

	end	// end for
endgenerate

endmodule
