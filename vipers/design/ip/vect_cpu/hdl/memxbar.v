//************************************************************************
// Scalable Vector CPU Project -- Crossbar for aligning memory data
// Filename: memxbar.v
// Author: Jason Yu
//
// Crossbar for rearranging read data from memory
//
//  Copyright (C) 2007 Jason Yu
//
//************************************************************************

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on
`ifdef	MODELSIM
	`include "../hdl/define.v"
`else
	`include "define.v"
`endif


module memxbar
// Parameters
#(	parameter	MEM_WIDTH = 128,
	parameter	MINDATAWIDTH = 16,
	
	// Local parameter
	parameter	NUMSEL = `log2(MEM_WIDTH/MINDATAWIDTH)
)

(
	// **** Input ****
	input	[MEM_WIDTH-1:0]	dataIn,		// data from memory
	input	[NUMSEL*(MEM_WIDTH/MINDATAWIDTH)-1:0]	xbarsel,	// select lines for Xbar
	// **** Output ****
	output	[MEM_WIDTH-1:0]	dataOut		// data to processors
);


// Generate memory MUX
// granularity = MINDATAWIDTH, number of select lines = NUMSEL
generate
	genvar gi;
	for (gi=0; gi < MEM_WIDTH/MINDATAWIDTH; gi=gi+1)
	begin: muxi
		
		// Each individual MUX for each MINDATAWIDTH output
		muxMto1
		#(.N(MINDATAWIDTH), .M(MEM_WIDTH/MINDATAWIDTH), .S(NUMSEL))
		mux
		(	.Z( dataOut [ (gi+1)*MINDATAWIDTH-1 : gi*MINDATAWIDTH ] ),
			.SEL( xbarsel [ (gi+1)*NUMSEL-1 : gi*NUMSEL ] ),
			.D( dataIn )
		);

	end	// end for
endgenerate

endmodule
