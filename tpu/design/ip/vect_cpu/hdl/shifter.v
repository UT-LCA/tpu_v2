//************************************************************************
// Scalable Vector CPU Project -- Vector lane shifter
// Filename: shifter.v
// Author: Jason Yu
//
// Vector CPU vector lane barrel/logical shifter
// - uses a mask plus barrel shifter to generate logical/arithmetic shifts
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


module shifter
#(	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1
)
(
	input	[VPU_WIDTH-1:0]			datain,
	input	[`log2(VPU_WIDTH)-1:0]	stageShift,		// select whether to shift for the particular stage
	input	[VPU_WIDTH-1:0]			logicalMask,	// mask that has zeros in positions that need to be zero'd for logical shifting;
													// used in arith shift as well
	input							arithShift,		// whether it is an arithmetic shift
	output	[VPU_WIDTH-1:0]			dataout
);
	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local parameter
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Number of shift stages, based on 2-input MUX
localparam	NSTAGES = `log2(VPU_WIDTH);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Internal signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	[(NSTAGES+1) * VPU_WIDTH - 1 : 0]	shiftin;	// define shift signals between all stages
wire	[VPU_WIDTH - 1 : 0]	shiftout;				// output of shifter

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Signal assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------
// assign input and output data
assign	shiftin [ VPU_WIDTH-1 : 0 ]  = datain;
assign	shiftout = shiftin [ (NSTAGES+1)*VPU_WIDTH-1 : NSTAGES*VPU_WIDTH ];

// for arithmetic shift, the sign extended positions are set to (one & sign bit), the other bits are untouched
assign	dataout = (arithShift) ? (shiftout & logicalMask) | ( ~logicalMask & {VPU_WIDTH{datain[VPU_WIDTH-1]}} ) : shiftout & logicalMask;


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Generate the shifter MUX
// - each stage selects between passthrough and a fixed shift amount equal to 2^(numstage-stage-1)
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Each stage is a row of input MUX
// log(VPU_WIDTH) number of stages, starting with shifting neighbouring bits
generate
genvar stagei;
	for (stagei = 0; stagei < NSTAGES; stagei = stagei+1)
	begin: stagei_gen

		// the first stage is shift by 1 bit, stage 2 is 2 bits, stage 3 is 4 bits, etc.
		shifter_muxstage
		#(	.VPU_WIDTH		(VPU_WIDTH),
			.STAGESTRIDE	(`pow2(stagei))
		)
		shiftstage
		(
			.datain				( shiftin[ (stagei+1)*VPU_WIDTH-1 : stagei*VPU_WIDTH ] ),
			.stageShift_sig		( stageShift[stagei] ),
			.dataout			( shiftin[ (stagei+2)*VPU_WIDTH-1 : (stagei+1)*VPU_WIDTH ] )
		);
	
	end	// end for
endgenerate

endmodule



//---------------------------------------------------------------------------------------------------------------------------------------------------
// One stage of the shifter mux
// - Each stage uses VPU_WIDTH of 2-input MUX
// - stageShift_sig controls whether the stage shifts its inputs
// - inherently shifts RIGHT
//---------------------------------------------------------------------------------------------------------------------------------------------------

`timescale 1ns / 100ps

module shifter_muxstage
#(	
	parameter	VPU_WIDTH = 1,
	parameter	STAGESTRIDE = 1				// the separation between inputs to MUX in the stage
)
(
	input	[VPU_WIDTH-1:0]	datain,		// input data
	input	stageShift_sig,					// whether to shift this stage; a 1 does the shift
	output	[VPU_WIDTH-1:0]	dataout		// output data
);
	
// Generate VPU_WIDTH number of 2-input MUX with the parameterized stride
generate
genvar muxi;
	for (muxi = 0; muxi < VPU_WIDTH; muxi = muxi+1)
	begin: muxi_gen
	
		muxMto1	#(.N(1), .M(2), .S(1))
		shift_muxi
			(
				.D 		( { datain[ (muxi + STAGESTRIDE) % VPU_WIDTH ], datain [ muxi ] } ),
				.SEL 	( stageShift_sig ),
				.Z 		( dataout [ muxi ] )
			);		
	
	end	// end for
endgenerate
		
endmodule
