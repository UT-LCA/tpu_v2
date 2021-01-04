//-------------------------------------------------------------------------------------------
// Scalable Vector CPU Project -- Memory write interface
// Filename: memwriteIF
// Author: Jason Yu
//
// Memory interface for memory writes
// - contains MUX to select data from lanes
// - rotation crossbar to align write data to the correct position
// - selectable delay to delay part of data for alignment
//
//  Copyright (C) 2007 Jason Yu
//
//------------------------------------------------------------------------------------------

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


module memwriteIF
#(	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1,
	parameter	NUMLANES = 1,
	parameter	MEM_WIDTH = 1,
	parameter	MINDATAWIDTH = 1,
	
	parameter	MAX_NSUBWORDS = MEM_WIDTH/MINDATAWIDTH,			// maximum number of subwords in a memory group
	parameter	NUM_MEMGROUP = (NUMLANES/(MEM_WIDTH/VPU_WIDTH))	// number of "MEM_WIDTH" memory groups to process; division result must be integer
)
(
	// Inputs
	clk,
	executing_scalar_store,
	d_waitrequest,
	lsu_flagstore,
	lsu_vext_vv,
	lsu_vins_vv,
	lsu_memDataWidth,
	scalar_d_writedata,
	laneDataIn,
	laneFlagQIn,
	laneWrselMUXSel,
	delaySubword_en,
	delaySubwordSel,
	memWralignXbarSel,
	alignedDataReg_en,
	alignedFlagReg_en,						// write enable to register flags from data lanes
	alignedFlagReg_clr,						// mask out the aligned flag register due to element index larger than vector length

	// Outputs
	alignedFlagRegQ,						// output from the intermediate flag register
	memWrdata
);

// Inputs
input									clk;
input									executing_scalar_store;
input									d_waitrequest;
input									lsu_flagstore;
input									lsu_vext_vv;
input									lsu_vins_vv;
input	[2:0]							lsu_memDataWidth;			// data width for memory access
input	[31:0]							scalar_d_writedata;			// scalar store data
input	[NUM_MEMGROUP*MEM_WIDTH-1:0]	laneDataIn;					// data from lanes
input	[`max(`log2(NUM_MEMGROUP)-1,0):0]		laneWrselMUXSel;
input	[NUMLANES-1:0]					laneFlagQIn;				// flag from lanes
input	[MEM_WIDTH-1:0]					alignedDataReg_en;			// write enable to register rotated data from rotation crossbar; needs to be bit-granularity to support flag store
input	[MAX_NSUBWORDS-1:0]				alignedFlagReg_en;			// write enable to register flags from data lanes
input	[MAX_NSUBWORDS-1:0]				alignedFlagReg_clr;			// mask out the aligned flag register due to element index larger than vector length
input	[MAX_NSUBWORDS-1:0]				delaySubword_en;			// enable the delay element for the halfword
input	[MAX_NSUBWORDS-1:0]				delaySubwordSel;			// select data from the delay element for the halfword; 1=select delayed value
input	[MAX_NSUBWORDS*`log2(MAX_NSUBWORDS)-1:0]	memWralignXbarSel;		// select for the delay element to memory crossbar

output	[MAX_NSUBWORDS-1:0]				alignedFlagRegQ;			// output from the intermediate flag register
output	[MEM_WIDTH-1:0]					memWrdata;					// data to be written to memory

	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// **** Local signals ****
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	[MEM_WIDTH-1:0]		laneWrselMUXOut;				// output from the lane select MUX
wire	[MEM_WIDTH-1:0]		alignXbarDataOut;				// output from alignment crossbar
wire	[MEM_WIDTH-1:0]		memgroupRegOut;					// output from alignment crossbar, registered
wire	[NUM_MEMGROUP*MEM_WIDTH-1:0]	laneWrselMUXIn;
wire	[(MEM_WIDTH/VPU_WIDTH)-1:0]		laneFlagWrSelMUXOut;
wire	[MEM_WIDTH-1:0]		delayOut;						// output from the selectable delay element
wire	[MEM_WIDTH-1:0]		scalarDataMergeOut;				// signal that includes scalar data if scalar store is specified
reg		[MEM_WIDTH-1:0]		memWidthCompressed;
reg		[MEM_WIDTH-1:0]		memWidthCompressByte;
reg		[MEM_WIDTH-1:0]		memWidthCompressHalfword;
reg		[MEM_WIDTH-1:0]		flagStoreData;
wire	[MEM_WIDTH-1:0]		alignedDataReg_data;
wire	[MEM_WIDTH-1:0]		alignXbarDataIn_pipeline;
reg		[MAX_NSUBWORDS-1:0]	alignedFlagReg;
reg		[MAX_NSUBWORDS-1:0]	flagExpandWord;
reg		[MAX_NSUBWORDS-1:0]	flagExpandHalfword;
reg		[MAX_NSUBWORDS-1:0]	flagExpandByte;
wire	[(MEM_WIDTH/VPU_WIDTH)-1:0]		laneFlagWr_override;

// Loop variables
integer i;
genvar gi;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// **** Signal Assignments ****
//---------------------------------------------------------------------------------------------------------------------------------------------------
assign 	laneWrselMUXIn = laneDataIn;
assign	memWrdata = alignXbarDataOut;

// if the operation does not use vector flag for masking, override the flags to turn them on (flagstore, vext.vv, vins.vv)
// flag on value is 0
assign	laneFlagWr_override = laneFlagWrSelMUXOut & ~{(MEM_WIDTH/VPU_WIDTH){(lsu_flagstore | lsu_vext_vv | lsu_vins_vv)}};

//---------------------------------------------------------------------------------------------------------------------------------------------------
// MUX from lane to memory rotation crossbar
//---------------------------------------------------------------------------------------------------------------------------------------------------
// selects which data lanes to connect to the rotation crossbar
muxMto1
#(.N(MEM_WIDTH), .M(NUM_MEMGROUP), .S(`max(`log2(NUM_MEMGROUP),1)))
laneWrselMUX
(
	.D					(laneWrselMUXIn),
	.SEL				(laneWrselMUXSel),
	.Z					(laneWrselMUXOut)
);

// selects which flags from vector lanes to connect to the intermediate register
// uses same select line as data
muxMto1
#(.N((MEM_WIDTH/VPU_WIDTH)), .M(NUM_MEMGROUP), .S(`max(`log2(NUM_MEMGROUP),1)))
laneFlagWrselMUX
(
	.D					(laneFlagQIn),
	.SEL				(laneWrselMUXSel),
	.Z					(laneFlagWrSelMUXOut)
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Compress the selected lane data into halfword and byte packed data if necessary
//---------------------------------------------------------------------------------------------------------------------------------------------------

// select the data from lane grouping depending on data width
always @(*)
	case (lsu_memDataWidth)
`ifdef	STORE_BYTE_ACCESS_ENABLED
		`MEMACCESS_BYTE: 		memWidthCompressed = memWidthCompressByte;
`endif
`ifdef	STORE_HALFWORD_ACCESS_ENABLED
		`MEMACCESS_HALFWORD:	memWidthCompressed = memWidthCompressHalfword;
`endif
`ifdef	STORE_WORD_ACCESS_ENABLED
		`MEMACCESS_WORD:		memWidthCompressed = laneWrselMUXOut;
`endif
		default:				memWidthCompressed = {MEM_WIDTH{1'bx}};
	endcase

	
`ifdef	STORE_HALFWORD_ACCESS_ENABLED		
// generate halfword data
always @(*) begin
	// Concatenate lower 16b halfword of each 32b data word from the lanes
	for (i=0; i<MEM_WIDTH; i=i+1)
		memWidthCompressHalfword[i] = laneWrselMUXOut[ ((i/16)*VPU_WIDTH + (i%16)) %  MEM_WIDTH ];
end
`endif

`ifdef	STORE_BYTE_ACCESS_ENABLED	
// generate byte data
always @(*) begin
	// Concatenate lower 8b byte of each 32b data word from the lanes
	for (i=0; i<MEM_WIDTH; i=i+1)
		memWidthCompressByte[i] = laneWrselMUXOut[ ((i/8)*VPU_WIDTH + (i%8)) %  MEM_WIDTH ];
end
`endif


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Connect flags from vector lane flag queue to intermediate flag register
// for flag store, flags to be stored are actually written to alignedDataReg
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Select flag arrangement depending on data width
always @(*)
	case (lsu_memDataWidth)
`ifdef	STORE_BYTE_ACCESS_ENABLED
		`MEMACCESS_BYTE: 		alignedFlagReg = flagExpandByte;
`endif
`ifdef	STORE_HALFWORD_ACCESS_ENABLED
		`MEMACCESS_HALFWORD:	alignedFlagReg = flagExpandHalfword;
`endif
`ifdef	STORE_WORD_ACCESS_ENABLED
		`MEMACCESS_WORD:		alignedFlagReg = flagExpandWord;
`endif
		default:				alignedFlagReg = {MAX_NSUBWORDS{1'bx}};
	endcase


`ifdef	STORE_WORD_ACCESS_ENABLED
// generate word data
always @(*) begin
	// Expand each flag to all subwords within a word
	for (i=0; i<MAX_NSUBWORDS; i=i+1)
		flagExpandWord[i] = laneFlagWr_override [ (i/(32/MINDATAWIDTH)) % (MEM_WIDTH/VPU_WIDTH) ];
end
`endif
				
`ifdef	STORE_HALFWORD_ACCESS_ENABLED
// generate halfword data
always @(*) begin
	// Expand each flag to all subwords within a halfword
	for (i=0; i<MAX_NSUBWORDS; i=i+1)
		flagExpandHalfword[i] = laneFlagWr_override [ (i/(16/MINDATAWIDTH)) % (MEM_WIDTH/VPU_WIDTH) ];
end
`endif

`ifdef	STORE_BYTE_ACCESS_ENABLED
// generate halfword data
always @(*) begin
	// Expand the flags to connect to all bits in the intermediate flag register
	for (i=0; i<MAX_NSUBWORDS; i=i+1)
		flagExpandByte[i] = laneFlagWr_override [i % (MEM_WIDTH/VPU_WIDTH)];
end
`endif


//---------------------------------------------------------------------------------------------------------------------------------------------------
// MUX vector flags into the write path for vector flag store instruction
// - for flag store, flags to be stored are actually written to alignedDataReg
//---------------------------------------------------------------------------------------------------------------------------------------------------
// alignedDataReg_data stores flag data for flagstore 
assign	alignedDataReg_data = (lsu_flagstore == 1'b1) ? flagStoreData : memWidthCompressed;

always @(*) begin
	// default zero data
	flagStoreData = {MEM_WIDTH{1'b0}};
	// cyclically repeat the bits
	// ASSUMES (NUMLANES < MEM_WIDTH), or if not, that MEM_WIDTH is multiple of NUMLANES
	for (i=0; i<`min(NUMLANES*`ELEMPERLANE, MEM_WIDTH); i=i+1)
		flagStoreData [i] = laneFlagQIn [i % NUMLANES];
end


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Register data and flag from lanes
// - each flag takes 1 bit of alignedFlagReg_ff, starting at index 0 and up to number of data elements per memory group
// - for flag store, flags to be stored are actually written to alignedDataReg
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Need bit-level enable signal for flag store
generate
	for (gi=0; gi<MEM_WIDTH; gi=gi+1)
	begin: alignedDataRegi
		// Register data
		ff	#(.WIDTH(1))
		alignedDataReg_ff
		(	.clk		(clk),
			.en			(alignedDataReg_en[gi]),
			.clrn		(1'b1),
			.d			(alignedDataReg_data[gi]),
			.q			(memgroupRegOut[gi])
		);
	end
endgenerate

generate
	for (gi=0; gi<MAX_NSUBWORDS; gi=gi+1)
	begin: alignedFlagRegi
		// Register flags
		// if (`FLAG_ON_VALUE == 1'b0)
			// if FLAG_ON is 0, set the flag to clear
			ff	#(.WIDTH(1))	alignedFlagReg_ff (.clk(clk), .en(alignedFlagReg_en[gi] | alignedFlagReg_clr[gi]), .clrn(1'b1), .d( alignedFlagReg [gi] | alignedFlagReg_clr[gi] ), .q( alignedFlagRegQ [gi]));
		// else
			// if FLAG_ON is 1, clear the flag to clear
			// ff	#(.WIDTH(1))	alignedFlagReg_ff (.clk(clk), .en(alignedFlagReg_en[gi]), .clrn(~alignedFlagReg_clr[gi]), .d( alignedFlagReg [gi]), .q( alignedFlagRegQ [gi]));
	end
endgenerate


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Generate the delay elements to align data before writing to memory
//---------------------------------------------------------------------------------------------------------------------------------------------------
generate
	for (gi=0; gi<MAX_NSUBWORDS; gi=gi+1)
	begin: delayi

		delayElement #(.WIDTH(MINDATAWIDTH))
		delay_inst
		(
			// Inputs
			.clk (clk),
			.dataIn ( memgroupRegOut [ ((gi+1)*MINDATAWIDTH)-1 : (gi*MINDATAWIDTH) ] ),
			.delaySubword_en ( delaySubword_en[gi] ),
			.delaySubwordSel ( delaySubwordSel[gi] ),
			
			// Output
			.dataOut ( delayOut [ ((gi+1)*MINDATAWIDTH)-1 : (gi*MINDATAWIDTH) ] )
		);
	end
endgenerate


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Scalar store data select
// - scalar store data is inserted right before the memory alignment crossbar
//---------------------------------------------------------------------------------------------------------------------------------------------------

generate
	for (gi=0; gi<32; gi=gi+1)
	begin: scalarDataMergeOuti
	// select scalar data to be merged into the top 32-bit of the memory write signal
		assign	scalarDataMergeOut [gi] = (executing_scalar_store == 1'b1) ? scalar_d_writedata [gi] : delayOut [gi];
	end
endgenerate

// rest of memory data signal doesn't need to have MUX
generate
	// MEM_WIDTH cannot be less than 32
	for (gi=32; gi<MEM_WIDTH; gi=gi+1)
	begin: scalarDataMergeOut2
		assign	scalarDataMergeOut [gi] = delayOut [gi];
	end
endgenerate


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Pipeline the data to balance pipeline stages in the controller
//---------------------------------------------------------------------------------------------------------------------------------------------------

ff	#(.WIDTH(MEM_WIDTH))
ff_alignXbarDataIn_pipeline
(
	.clk			(clk),
	.en				(~d_waitrequest),
	.clrn			(1'b1),
	.d				( scalarDataMergeOut ),
	.q				( alignXbarDataIn_pipeline )
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Memory rotation crossbar
//---------------------------------------------------------------------------------------------------------------------------------------------------
memxbar
#(	.MEM_WIDTH			(MEM_WIDTH), 
	.MINDATAWIDTH		(MINDATAWIDTH)	)
memWralignXbar
(
	// .dataIn 			( scalarDataMergeOut ),
	.dataIn 			( alignXbarDataIn_pipeline ),
	.xbarsel 			( memWralignXbarSel ),
	.dataOut 			( alignXbarDataOut )
);


endmodule


//---------------------------------------------------------------------------------------------------------------------------------------------------
// **** Selectable delay element ****
//
// - can either select passthrough or delayed value
// - used for delaying some of the subwords to align data for writes with offset
//---------------------------------------------------------------------------------------------------------------------------------------------------

// Align data written to meory
module delayElement
#(	parameter WIDTH = 16 )
(
	// Inputs
	clk,
	dataIn,
	delaySubword_en,
	delaySubwordSel,

	// Output
	dataOut
);

input 	clk;
input	[WIDTH-1:0]	dataIn;
input	delaySubword_en;	// store dataIn into the delay FF
input	delaySubwordSel;	// selects the delayed signal when =1, passthrough when =0
output	[WIDTH-1:0]	dataOut;

reg	[WIDTH-1:0]	dataDelay;

// assign output
assign dataOut = (delaySubwordSel) ? dataDelay : dataIn;

always @(posedge clk)
begin
	if (clk == 1'b1)
		if (delaySubword_en)
			dataDelay <= dataIn;
		else
			dataDelay <= dataDelay;
end

endmodule
