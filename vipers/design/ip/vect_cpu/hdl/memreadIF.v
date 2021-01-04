//---------------------------------------------------------------------------------------------------------------------------------------------------
// Scalable Vector CPU Project -- Memory read interface
// Filename: memreadIF.v
// Author: Jason Yu
// Created: September 13, 2007
//
// - connects data from memory read alignment crossbar to vector lanes
// - also does unsigned/signed extension
//
//  Copyright (C) 2007 Jason Yu
//
//---------------------------------------------------------------------------------------------------------------------------------------------------

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


module memreadIF
#(	// Parameters
	parameter	VPU_WIDTH = 1,
	parameter	NUMLANES = 1,
	parameter	MEM_WIDTH = 1,
	parameter	MINDATAWIDTH = 1,
	
	// Local parameter
	parameter	MAX_NUMELEMS = NUMLANES * `ELEMPERLANE,	// number of total data elements
	parameter	MAX_NSUBWORDS = MEM_WIDTH/MINDATAWIDTH,		// maximum number of subwords in a memory group
	parameter	READMUX_NSEL = `log2(MEM_WIDTH/MINDATAWIDTH)		// number of select lines in read MUX
)
(
	// Inputs
	input		[MEM_WIDTH-1:0]				d_readdata,				// main memory read data
	input		[READMUX_NSEL*MAX_NSUBWORDS-1:0]	readXbarSel,	// select to memory crossbar
	input		[2:0]						lsu_memDataWidth,
	input									lsu_signedOp,
	input		[`log2(MAX_NUMELEMS):0]			loadElemCount_preg,	// count of how many elements have been loaded
	input		[`log2(NUMLANES):0]		numActiveLanes_preg,
	input									lsu_flagload,
	input									executing_scalar_load,
	
	// Outputs
	output		[31:0]						scalar_d_readdata,
	output reg	[NUMLANES*VPU_WIDTH-1:0]	memLoadDataToLanes		// data to processors
);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	[NUMLANES*`log2(`ELEMPERLANE)-1:0]	dataElemSel;			// select line for selecting data to vector lane
wire	[MEM_WIDTH-1:0]				readXbarOut;
reg		[NUMLANES*VPU_WIDTH-1:0]	memLoadDataToLanes_sig;
reg		[NUMLANES-1:0]				memLoadFlagToLanes;
reg		[`log2(`ELEMPERLANE)-1:0]	dataElemSel_temp;				// select line for selecting data to vector lane

integer i, j;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Signal assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------
// scalar read data
assign	scalar_d_readdata = memLoadDataToLanes_sig[31:0];

// generate select for MUXs to select which data block to align to the vector lanes
generate
genvar lanei;
	for (lanei=0; lanei<NUMLANES; lanei=lanei+1)
	begin: dataelemseli
		// do not wrap around the lanes
		assign dataElemSel [(lanei+1)*`log2(`ELEMPERLANE)-1 : (lanei)*`log2(`ELEMPERLANE)] =
			((lanei >= (loadElemCount_preg % NUMLANES)) && (lanei < `min(((loadElemCount_preg % NUMLANES) + numActiveLanes_preg), NUMLANES))) ?
				loadElemCount_preg / NUMLANES :
				loadElemCount_preg / NUMLANES + 1;
	end
endgenerate


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Memory read crossbar
// - aligns read data from memory so each lane gets the proper data
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Memory read crossbar
memxbar
#(.MEM_WIDTH(MEM_WIDTH), .MINDATAWIDTH(MINDATAWIDTH))
readXbar
(
	.dataIn					(d_readdata),
	.xbarsel				(readXbarSel),
	.dataOut				(readXbarOut)
);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Do sign/unsigned extension of read memory data
// - data in readXbarOut is laid out in pakced word/halfword/word, with MEM_WIDTH/16 halfwords with halfword access,
// MEM_WIDTH/8 bytes with byte access
//
always @(*) begin
	i = 0;
	j = 0;
	memLoadDataToLanes_sig = {NUMLANES*VPU_WIDTH{1'bx}};
	dataElemSel_temp = {`log2(`ELEMPERLANE){1'bx}};

	if (executing_scalar_load) begin
		for (i=0; i<32; i=i+1)
			memLoadDataToLanes_sig [i] = readXbarOut [i % MEM_WIDTH];
	end
	
	else begin
//---------------------------------------------------------------------------------------------------------------------------------------------------	
	// if the maximum number of subwords is less than number of lanes
	// just directly connect and remaining lanes start again at beginning of readXbarOut
	if (MAX_NSUBWORDS <= NUMLANES) begin
		case (lsu_memDataWidth)

`ifdef	LOAD_WORD_ACCESS_ENABLED
			`MEMACCESS_WORD	: begin
				// for (lanei=0; lanei<NUMLANES; lanei=lanei+1) begin
				for (i=0; i<NUMLANES*VPU_WIDTH; i=i+1)
					memLoadDataToLanes_sig [i] = readXbarOut [i % MEM_WIDTH];
			end
`endif
			
`ifdef	LOAD_HALFWORD_ACCESS_ENABLED
			`MEMACCESS_HALFWORD	: begin
				// iterate through all data to vector lanes
				for (i=0; i<NUMLANES;i=i+1) begin
					// set memaccess data portion
					for (j=i*VPU_WIDTH; j<i*VPU_WIDTH+16; j=j+1)
						memLoadDataToLanes_sig [j] = readXbarOut [(i*16 + j%16) % MEM_WIDTH];
					// set unsigned/signed extension
					for (j=i*VPU_WIDTH+16;j <(i+1)*VPU_WIDTH; j=j+1)
						if (lsu_signedOp == 1'b0)
							// unsigned extension
							memLoadDataToLanes_sig [j] = 1'b0;
						else
							// signed extension
							memLoadDataToLanes_sig [j] = readXbarOut [(i*16 + 15) % MEM_WIDTH];
				end
			end
`endif

`ifdef	LOAD_BYTE_ACCESS_ENABLED
			`MEMACCESS_BYTE	: begin
				// iterate through all data to vector lanes
				for (i=0; i<NUMLANES;i=i+1) begin
					// set memaccess data portion
					for (j=i*VPU_WIDTH; j<i*VPU_WIDTH+8; j=j+1)
						memLoadDataToLanes_sig [j] = readXbarOut [(i*8 + j%8) % MEM_WIDTH];
					// set unsigned/signed extension
					for (j=i*VPU_WIDTH+8;j <(i+1)*VPU_WIDTH; j=j+1)
						if (lsu_signedOp == 1'b0)
							// unsigned extension
							memLoadDataToLanes_sig [j] = 1'b0;
						else
							// set to zero if unsigned, sign extend if signed
							memLoadDataToLanes_sig [j] = readXbarOut [(i*8 + 7) % MEM_WIDTH];
				end
			end
`endif

			default	: begin
				// use word access connection
				// for (i=0; i<NUMLANES*VPU_WIDTH; i=i+1)
					// memLoadDataToLanes_sig [i] = readXbarOut [i % MEM_WIDTH];
				for (i=0; i<NUMLANES*VPU_WIDTH; i=i+1)
					memLoadDataToLanes_sig [i] = 1'bx;
			end
		endcase
	end

//---------------------------------------------------------------------------------------------------------------------------------------------------	
	// if the maximum number of subwords is greater than number of lanes
	// need to create MUX at vector lanes to select which data word writes to a vector lane
	else begin
		case (lsu_memDataWidth)

`ifdef	LOAD_WORD_ACCESS_ENABLED
			`MEMACCESS_WORD	: begin
				// iterate through all data to vector lanes
				for (i=0; i<NUMLANES;i=i+1) begin
					for (j=0; j<`log2(`ELEMPERLANE); j=j+1)
						dataElemSel_temp [j] = dataElemSel [i*`log2(`ELEMPERLANE)+j];

					// set memaccess data portion
					for (j=i*VPU_WIDTH; j<i*VPU_WIDTH+32; j=j+1)
						// dataElemSel is a MUX select
						// dataElemSel selects between the VPU_WIDTH*NUMLANES groups
						memLoadDataToLanes_sig [j] = readXbarOut [(dataElemSel_temp*(NUMLANES*32) + (i%NUMLANES)*32 + j%32) % MEM_WIDTH];
				end
			end
`endif
			
`ifdef	LOAD_HALFWORD_ACCESS_ENABLED
			`MEMACCESS_HALFWORD	: begin

			// iterate through all data to vector lanes
				for (i=0; i<NUMLANES;i=i+1) begin
					for (j=0; j<`log2(`ELEMPERLANE); j=j+1)
						dataElemSel_temp [j] = dataElemSel [i*`log2(`ELEMPERLANE)+j];
					
					// set memaccess data portion
					for (j=i*VPU_WIDTH; j<i*VPU_WIDTH+16; j=j+1)
						memLoadDataToLanes_sig [j] = readXbarOut [(dataElemSel_temp*(NUMLANES*16) + (i%NUMLANES)*16 + j%16) % MEM_WIDTH];
					// set unsigned/signed extension
					for (j=i*VPU_WIDTH+16;j <(i+1)*VPU_WIDTH; j=j+1)
						// set to zero if unsigned, sign extend if signed
						memLoadDataToLanes_sig [j] = readXbarOut [(dataElemSel_temp*(NUMLANES*16) + (i%NUMLANES)*16 + 15) % MEM_WIDTH]  & lsu_signedOp;
				end
			end
`endif

`ifdef	LOAD_BYTE_ACCESS_ENABLED
			`MEMACCESS_BYTE	: begin

			// iterate through all data to vector lanes
				for (i=0; i<NUMLANES;i=i+1) begin
					for (j=0; j<`log2(`ELEMPERLANE); j=j+1)
						dataElemSel_temp [j] = dataElemSel [i*`log2(`ELEMPERLANE)+j];
					
					// set memaccess data portion
					for (j=i*VPU_WIDTH; j<i*VPU_WIDTH+8; j=j+1)
						memLoadDataToLanes_sig [j] = readXbarOut [(dataElemSel_temp*(NUMLANES*8) + (i%NUMLANES)*8 + j%8) % MEM_WIDTH];
					// set unsigned/signed extension
					for (j=i*VPU_WIDTH+8;j <(i+1)*VPU_WIDTH; j=j+1)
						// set to zero if unsigned, sign extend if signed
						memLoadDataToLanes_sig [j] = readXbarOut [(dataElemSel_temp*(NUMLANES*8) + (i%NUMLANES)*8 + 7) % MEM_WIDTH] & lsu_signedOp;
				end
			end
`endif

			default	: begin
				for (i=0; i<NUMLANES*VPU_WIDTH; i=i+1)
					memLoadDataToLanes_sig [j] = 1'bx;
			end
		endcase
	end

	end
end


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Select data for flag data
//---------------------------------------------------------------------------------------------------------------------------------------------------

always @(*) begin
	i = 0;
	// assumes MEM_WIDTH is greater than number of lanes
	// need to create MUX at vector lanes to select which bit writes to a vector lane
	// iterate through all data to vector lanes
	for (i=0; i<NUMLANES;i=i+1) begin
		// set memaccess data portion
		// each lane connects to 1 bit of memory crossbar
		memLoadFlagToLanes [i] = readXbarOut [ ( dataElemSel[`log2(`ELEMPERLANE)-1:0] * (NUMLANES) + (i%NUMLANES)) % MEM_WIDTH ];
	end
end


//---------------------------------------------------------------------------------------------------------------------------------------------------
// MUX flag data on to lane data
//---------------------------------------------------------------------------------------------------------------------------------------------------
always @(*) begin
	i=0;
	// default assignment
	memLoadDataToLanes = memLoadDataToLanes_sig;
	
	// override assignment to LSB of each data word with flag data
	if (lsu_flagload == 1'b1) begin
		for (i=0; i<NUMLANES*VPU_WIDTH; i=i+VPU_WIDTH) begin
			// only create MUX at the bits where it's needed
			if ((i % VPU_WIDTH) == 0)
				memLoadDataToLanes[i] = memLoadFlagToLanes[i/VPU_WIDTH];
			else
				memLoadDataToLanes[i] = 1'bx;
		end
	end
end

endmodule
