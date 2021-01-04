//************************************************************************
// Scalable Vector CPU Project -- Memory store, memory write logic
// Filename: storeCtr_writemem.v
// Author: Jason Yu
//
// Write from intermediate register to memory logic for vector stores
// 
// - the write side state machine writes data from intermediate register to memory
// - asserts loadNextLaneDataGroup when write from intermediate register is almost done
// to signal read side state machine to start reading more data
//
// Memory address generator for stores
// - reads min((MEM_WIDTH/VPU_WIDTH), numlanes) elements from vector lanes at a time
//   and stores to intermediate register enabled by alignedDataReg_en
// - once intermediate register is filled, the contents are written to memory
//   through the delay and then alignment crossbar network
// - delay network delays elements to align different data blocks
// - crossbar network moves data elements to the correct positions depending on stride and offset
// - elements refer to the data elements from the vector lanes that are processed each cycle
// - subwords refer to the MINDATAWIDTH-sized subwords that make up each data element
// - accesses must be aligned to the width of the data
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


module storeCtr_writemem

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Parameters
// --------------------------------------------------------------------------------------------------------------------------------------------------
#(	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1,
	parameter	NUMLANES = 1,
	parameter	MEM_WIDTH = 1,
	parameter	MINDATAWIDTH = 1,

	// parameters that shouldn't be changed
	parameter	MAX_NSUBWORDS = MEM_WIDTH/MINDATAWIDTH,		// maximum number of subwords in a memory group
	parameter	MAX_NSUBWORDS_NBIT = `log2(MAX_NSUBWORDS)
)
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Ports
// --------------------------------------------------------------------------------------------------------------------------------------------------
(
	// **** Inputs ****
	input				clk,
	input				reset_n,
	input				newInstr_store,						// indicates a new memory store instruction
	input				d_waitrequest,						// memory wait request
	input				scalar_write_grant,					// scalar store; overrides all other operations
	input		[31:0]	scalar_d_address,
	input		[31:0]	baseAddr,							// base address
	input		[`MAXSTRIDE_NBIT-1:0]	stride,				// vector element stride
	input		[MAX_NSUBWORDS-1:0]		alignedFlagRegQ,	// flag from lanes
	input				writeRegDataToMem,					// input from fillreg side, start writing the lane data group to memory
	input				executing_vector_store,				// indicates vector store is being executed, from loadstore controller
	input		[31:0]	lsu_vectorlength_minus_one,
	input		[2:0]	lsu_memDataWidth,					// width of data access from memory (constants defined in define.v)
	input				lsu_unitstride,						// whether it's unit stride, overrides stride
	input				lsu_flagstore,
	input				lsu_vext_vv,
	input				lsu_vins_vv,
	input				lsu_indexed_memaccess,				// indexed vector store
	input		[31:0]	vindex,								// the vindex control register
	input		[`log2(MEM_WIDTH/MINDATAWIDTH)-1:0]		lastElemMemgroupPos,	// position of last element to be transferred
	
	input		[VPU_WIDTH-1:0]	indexOffset_out,				// offset from lanes for indexed addressing

	// **** Outputs ****
	output				vmanip_storeload_sync,
	output reg 	[`log2(NUMLANES*`ELEMPERLANE):0]		indexOffsetCount,	// count how many cycles left in the entire write operation
	output				loadNextLaneDataGroup,				// load the next group of lane data to alignment register
	output		[MAX_NSUBWORDS-1:0]		delaySubword_en,	// enable the delay element for the halfword
	output reg	[MAX_NSUBWORDS-1:0]		delaySubwordSel,	// select data from the delay element for the halfword, 1=select delayed value
	output reg	[MAX_NSUBWORDS_NBIT*MAX_NSUBWORDS-1:0]	memWralignXbarSel,		// select for the delay element to memory crossbar
	// Outputs to memory
	output		[31:0]	mem_addr_0,
	output		[(MEM_WIDTH/MINDATAWIDTH)-1:0]	vector_mem_dm,	// data mask byte enable, 16 bytes in 128b
	output				d_write,
	output				store_done							// asserted the last cycle of transfer
);

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Local parameters
// --------------------------------------------------------------------------------------------------------------------------------------------------
// commonly used data width in this file
// `log2(MAXSTRIDE*4) is maximum stride in bytes
// MAX_NSUBWORDS_NBIT is number of subwords in each block to be written to memory
`define		D_WIDTH				(`MAXSTRIDE_NBIT+2+`log2(NUMLANES)+MAX_NSUBWORDS_NBIT)

localparam	MEMWIDTH_BYTES = MEM_WIDTH/8;							// memory width in number of bytes
localparam	MAX_ELEM_PER_CYCLE = `min(MEM_WIDTH/VPU_WIDTH,NUMLANES);	// number of elements that can be processed per cycle; cannot really be changed

// writeside states
localparam
	Idle = 0,
	WriteInitWait = 1,
	WriteCountState = 2,
	WriteWaitState = 3,
	WriteDelayCycle = 4,
	WriteScalarStore = 5,
	VextVVPassthrough = 6;

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Local Signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
reg		[1:0]						convElemToByteShiftConst;		// Number of positions to shift to convert from units of elements to bytes
wire	[31:0]						baseAligned;					// base address aligned to 128 bit
wire	[`MAXSTRIDE_NBIT+2:0]		localStride;					// local stride signal
wire	[`MAXSTRIDE_NBIT+2:0]		localStrideByte;				// stride in number of bytes
wire	[VPU_WIDTH-1:0]			nextMemAddrIncByte;				// increment to the current memory read address
wire	[VPU_WIDTH-1:0]			nextMemAddrSig;
wire	[`log2(MEMWIDTH_BYTES)-1:0]	offsetByte;						// write offset in bytes
wire								largeStride;					// define largeStride = (localStrideByte > MEMWIDTH_BYTES) ? 1 : 0
reg		[VPU_WIDTH-1:0]			nextMemAddr;					// next memory read address
wire	[VPU_WIDTH-1:0]			nextMemAddr_pipeline;			// next memory read address
reg		[`MAXSTRIDE_NBIT+2+`log2(NUMLANES):0]	currentOffsetByte;	// keep track of the current offset
reg		[`MAXSTRIDE_NBIT+2+`log2(NUMLANES):0]	blockOffsetByte;	// keep track of the starting offset of the (MEM_WIDTH) "block" to be transferred
wire	[`MAXSTRIDE_NBIT+2+`log2(NUMLANES):0]	nextOffsetByte;
wire	[`MAXSTRIDE_NBIT+2+`log2(NUMLANES):0]	addOffsetByte;
wire	[`MAXSTRIDE_NBIT+2:0]		nextMemIncByteLargeStride;
wire	[`log2(MEM_WIDTH/MINDATAWIDTH):0]		numElemPerCluster;	// number of elements in the cluster for this transfer
reg 	[`MAXSTRIDE_NBIT+2:0]					clusterWriteCount;	// count how many cycles left in the write operation
reg 	[`log2(NUMLANES*`ELEMPERLANE):0]		totalWriteCount;	// count how many cycles left in the entire write operation

// `log2(NUMLANES*`ELEMPERLANE) is number of bits needed to encode MVL elements
// plus maxstride in number of bytes (extra two bits for 4 bytes in a word), plus 1 more bit for safety when adding offsets
wire 	[`MAXSTRIDE_NBIT+2+`log2(NUMLANES*`ELEMPERLANE)+1:0]	totalWriteCount_temp;		// count how many cycles left in the entire write operation
wire 	[`MAXSTRIDE_NBIT+2+`log2(NUMLANES*`ELEMPERLANE)+1:0]	totalWriteCount_sig;		// count how many cycles left in the entire write operation
wire 	[`MAXSTRIDE_NBIT+2+`log2(NUMLANES*`ELEMPERLANE)+1-`log2(MEMWIDTH_BYTES):0]		totalWriteCount_ceil;

// the number of bits used for the following signals are calculated as follows:
// maximum stride in bytes = 'log2('MAXSTRIDE*4)
// offsets could get as large as NUMLANES * maxstride for largestride case
// processes "MAX_NSUBWORDS" offsets at a time before resetting the comparison address
// and the _group signals contains the offsets of all MAX_NSUBWORDS
wire	[`D_WIDTH * MAX_NSUBWORDS - 1 : 0]	offsetAddStrideByte_group;		// the values to add to base offset to generate element offsets; 8 values of 6 bits (up to stride 16 * 32b)
reg		[`D_WIDTH * MAX_NSUBWORDS - 1 : 0]	offsetAddStrideByte_group_reg;	// the values to add to base offset to generate element offsets, registered
wire	[`D_WIDTH * MAX_NSUBWORDS - 1 : 0]	elemOffsetByte_group;			// offset in bytes for the elements to be written; 4 values (up to MAXSTRIDE * 32b)
reg		[`D_WIDTH * MAX_NSUBWORDS - 1 : 0]	subwordOffsetByte_group;		// offset in bytes for all the MAX_NSUBWORDS subwords to be written
wire	[`D_WIDTH * MAX_NSUBWORDS - 1 : 0]	subwordOffsetByte_group_pipeline;		// offset in bytes for all the MAX_NSUBWORDS subwords to be written
wire	[`D_WIDTH * MAX_NSUBWORDS - 1 : 0]	subwordOffsetByte_group_minwidth;		// offset in bytes for all the MAX_NSUBWORDS subwords to be written, adjusted to minimum supported width of memIF; some bits are unused

wire	[MAX_NSUBWORDS * MAX_NSUBWORDS-1:0]	XbarSelDecodeOnehot;	// one-hot decode for Xbar select; 8 bits * 8 words
reg		[MAX_NSUBWORDS-1:0]		writeDmSubword;						// write data mask in smallest memory data width granularity
reg		[MAX_NSUBWORDS-1:0]		subwordFlag_group;					// flags for each subword
reg		[MAX_NSUBWORDS-1:0]		subword_decode_delayed;				// delayed subword decode valid signals
wire	[MAX_NSUBWORDS-1:0]		subword_decode_delayed_pipeline;	// delayed subword decode valid signals
reg		[MAX_NSUBWORDS-1:0]		XbarSelDecodeMask;					// Mask to determine whether to decode the word position; 1=decode, 0=don't decode
reg		[2:0]					XbarSelDecodeMask_select;			// select for MUX to select which mask to use for decoding
wire	[2:0]					XbarSelDecodeMask_select_pipeline;	// select for MUX to select which mask to use for decoding
reg		[MAX_NSUBWORDS-1:0]		scalar_combine_subword_decode;		// decode mask for scalar store
wire	[MAX_NSUBWORDS-1:0]		scalar_combine_subword_decode_pipeline;		// decode mask for scalar store
wire	[(`MAXSTRIDE_NBIT+2+MAX_ELEM_PER_CYCLE) : 0]	elemOffsetByte_group_compare;
reg		[(`MAXSTRIDE_NBIT+2+MAX_ELEM_PER_CYCLE) : 0]	elemOffsetByte_group_comparereg;
wire	[(`MAXSTRIDE_NBIT+2+MAX_ELEM_PER_CYCLE) : 0]	elemOffsetByte_group_compare_pipeline;
wire	[(`MAXSTRIDE_NBIT+2+MAX_ELEM_PER_CYCLE) : 0]	elemOffsetByte_group_comparereg_pipeline;
wire	[(`MAXSTRIDE_NBIT+2+MAX_ELEM_PER_CYCLE) : 0]	elemOffsetByte_group_compare_minwidth;
wire	[(`MAXSTRIDE_NBIT+2+MAX_ELEM_PER_CYCLE) : 0]	elemOffsetByte_group_comparereg_minwidth;

wire	[MAX_NSUBWORDS-1:0]	subwordOffsetValid;					// whether the subwords to be written are valid in this cycle
wire	[MAX_NSUBWORDS-1:0]	subwordFlagOn_group;
wire	[MAX_NSUBWORDS-1:0]	subwordFlagOn_group_pipeline;
reg							executing_write;					// currently writing to main memory
reg							firstWriteCycle_en;
wire						firstWriteCycle;
wire						clusterWriteDone;
reg							gotoWriteDelayCycle;				// need to write an extra cycle to write delayed data
wire						clusterWriteFirstCycle;
wire	store_done_sig;											// asserted the last cycle of transfer
wire	store_done_pipeline;									// asserted the last cycle of transfer
wire	d_write_sig;
wire	d_write_pipeline;
wire	nextCycleIs_clusterWriteFirstCycle;						// cycle after "nextCycleIs_clusterWriteFirstCycle" is the first cycle of writing the current cluster
wire	triggerLoadNextGroup;									// synchronize with fillreg state machine
wire	triggerLoadNextGroup_delay;
wire	triggerLoadNextGroup_posedge;

reg		[`D_WIDTH-1:0]		memgroupWriteBytes;
wire	[`D_WIDTH-1:0]		memgroupWriteBytesConst_B;
wire	[`D_WIDTH-1:0]		memgroupWriteBytesConst_HW;
wire	[`D_WIDTH-1:0]		memgroupWriteBytesConst_W;

wire	[VPU_WIDTH-1:0]			indexOffset;				// current index offset
wire	[31:0]						indexOffset_sext;			// sign extend indexed offset if VPU_WIDTH is less than 32
wire	[31:0]						baseAddrIndexAdded;			// base address with added index (index is zero'd for non-index accesses)
wire	[31:0]						baseAddrScalarSel;			// base address after scalar select MUX

wire	[`log2(NUMLANES*`ELEMPERLANE):0]		indexOffsetCount_delay;	// count how many cycles left in the entire write operation
wire	[`log2(NUMLANES*`ELEMPERLANE):0]		indexOffsetCount_delay2;	// count how many cycles left in the entire write operation
reg		[MAX_NSUBWORDS-1:0]	indexedElemOffsetValid;		// data element decode mask for indexed store
reg		executing_vextvv;
wire	executing_vextvv_delay;
wire	vinsvv_storeload_sync;
wire	vextvv_storeload_sync;


genvar gi;
integer i;

// State machine signals
reg	[2:0]	writesideState;				// write side state machine state register
reg	[2:0]	writesideNextstate;

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Conversion factors
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Number of positions to shift to convert from units of elements to bytes for conversion
always @(*)
	case (lsu_memDataWidth)
`ifdef	STORE_WORD_ACCESS_ENABLED
		`MEMACCESS_WORD	:		convElemToByteShiftConst = 2;
`endif
`ifdef	STORE_HALFWORD_ACCESS_ENABLED
		`MEMACCESS_HALFWORD	:	convElemToByteShiftConst = 1;
`endif
`ifdef	STORE_BYTE_ACCESS_ENABLED
		`MEMACCESS_BYTE	:		convElemToByteShiftConst = 0;
`endif
		default	:				convElemToByteShiftConst = 2'bxx;
	endcase

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Signal to start loading next block of data into intermediate register before delay/align crossbar
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Start loading the next set of data into alignment register when there is less than laneDataLoadCount_const cycles left to write to memory on write side
// unit stride never has to wait for write side before loading again
// assign	loadNextLaneDataGroup = (((clusterWriteCount <= (laneDataLoadCount_const+1)) && (clusterWriteCount > 1))
								// || (lsu_unitstride && (laneDataLoadCount == 0))) ? 1'b1 : 1'b0;
// (laneDataLoadCount_const+1) doesn't work with certain corncer cases--data in intermediate register gets overwritten by next data group before being written to memory
// start loading the next memory block data from lane store queues when there are 2 cycles left to write the current memory block
// store queue rdreq is asserted same cycle as loadNextLaneDataGroup
// assign	loadNextLaneDataGroup = ((clusterWriteCount == 2) || (writesideState  == VextVVPassthrough)) ? 1'b1 : 1'b0;

// need to raise signal when clusterWriteCount when there is 3 or less cycles left to write
// fillreg reads data from store queue the cycle after it receives the loadNextLaneDataGroup sync signal, then takes minimum one cycle to register data.  Data is outputted on 3rd cycle
// ~clusterWriteDone makes sure that the trigger signal is reset
assign	triggerLoadNextGroup = ((writesideState == WriteCountState) && (clusterWriteCount <= 3) && ~clusterWriteDone) ? 1'b1 : 1'b0;
assign	triggerLoadNextGroup_posedge = triggerLoadNextGroup & ~triggerLoadNextGroup_delay;
ff	#(.WIDTH(1))	ff_triggerLoadNextGroup_delay (.clk(clk), .en(~d_waitrequest), .clrn(1'b1), .d(triggerLoadNextGroup), .q(triggerLoadNextGroup_delay));

assign	loadNextLaneDataGroup = ((triggerLoadNextGroup_posedge == 1'b1) || (writesideState  == VextVVPassthrough)) ? 1'b1 : 1'b0;


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Assign outputs
// --------------------------------------------------------------------------------------------------------------------------------------------------
assign 	mem_addr_0 = nextMemAddr_pipeline;							// address to memory
assign	d_write = d_write_pipeline;
assign	vector_mem_dm = writeDmSubword;								// Assign memory data mask
assign	store_done = store_done_pipeline;

// vext.vv and vins.vv do not need to actually write to memory
assign	d_write_sig = executing_write & (~lsu_vext_vv) & (~lsu_vins_vv);
ff	#(.WIDTH(1))	ff_d_write_pipeline (.clk(clk), .en(~d_waitrequest), .clrn(reset_n), .d(d_write_sig), .q(d_write_pipeline));

// to be 100% safe
// can also change to: ... && (writesideNextState == Idle) to be 1 cycle shorter?
assign	store_done_sig = ((totalWriteCount == 0) && (writesideState == Idle))
					|| ((writesideState == WriteScalarStore) && ~d_waitrequest) ? 1'b1 : 1'b0;	// finished memory instruction

ff	#(.WIDTH(1))	ff_store_done_pipeline (.clk(clk), .en(~d_waitrequest), .clrn(reset_n), .d(store_done_sig), .q(store_done_pipeline));


assign	vmanip_storeload_sync = (lsu_vext_vv & vextvv_storeload_sync) | (lsu_vins_vv & vinsvv_storeload_sync);
assign	vinsvv_storeload_sync = clusterWriteFirstCycle;
// assign	vextvv_storeload_sync = executing_vextvv_delay;
assign	vextvv_storeload_sync = executing_vextvv;

ff	#(.WIDTH(1))	ff_vextvv_pipeline (.clk(clk), .en(1'b1), .clrn(reset_n), .d(executing_vextvv), .q(executing_vextvv_delay));


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Assign address calculation signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Number of elements in each cluster for this transfer
assign	numElemPerCluster = MEMWIDTH_BYTES >> convElemToByteShiftConst;

// Zero indexOffset for non-indexed memory accesses
assign	indexOffset_sext = {{(32-VPU_WIDTH){indexOffset_out[VPU_WIDTH-1]}}, indexOffset_out};
assign	indexOffset = (lsu_indexed_memaccess == 1'b1) ? indexOffset_sext : {VPU_WIDTH{1'b0}};

assign	baseAddrIndexAdded = baseAddr + indexOffset;								// indexed address
// Scalar processor override
assign	baseAddrScalarSel = (scalar_write_grant == 1'b1) ? scalar_d_address : baseAddrIndexAdded;
assign	baseAligned = baseAddrScalarSel & ~{32'h00000000, {`log2(MEMWIDTH_BYTES){1'b1}}};	// truncate to MEM_WIDTH aligned address

assign 	offsetByte = (lsu_vins_vv == 1'b1) ? vindex << convElemToByteShiftConst : baseAddrScalarSel % MEMWIDTH_BYTES;					// offset from 128 bit alignment in bytes
// indexed memaccess need to set stride to 1 as well, but not assert lsu_unitstride
assign 	localStride = ((lsu_unitstride == 1'b1) || (lsu_indexed_memaccess == 1'b1)) ? 1 : stride;
assign	localStrideByte = localStride << convElemToByteShiftConst;	// stride in number of bytes

// done writing 1 cluster
assign	clusterWriteDone = (clusterWriteCount == 1) ? 1'b1 : 1'b0;
assign 	largeStride = (localStrideByte > MEMWIDTH_BYTES) ? 1'b1 : 1'b0;				// define "largestride" as localStrideByte > 16; largestride requires skipping memory locations

// determine whether the last element to be written is delayed
// essentially a copy of the delay logic but for the last element
// delay if: offset > stride * number of elements per cluster (in appropriate data width)	
// use a FF to help timing analysis; can be multicycle since value will stabilize during initialization state
always @(posedge clk)
	// do not need to delay if indexed access
	gotoWriteDelayCycle <= (((lastElemMemgroupPos * localStrideByte + offsetByte) >= memgroupWriteBytes) && ~lsu_indexed_memaccess) ? 1'b1 : 1'b0;

// on first cycle of each block, if there were delayed subwords, decode those as well
ff	#(.WIDTH(1))	ff_firstWriteCycle (.clk(clk), .en(~d_waitrequest), .clrn(reset_n), .d(firstWriteCycle_en), .q(firstWriteCycle));


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Counter to store how many cycles it takes to write the data group to memory
// - it takes "stride * numElem / MaxNSubword" cycles to transfer all elements
// - numElem is MaxNSubword for 8 bit, MaxNSubword/2 for 16b, MaxNSubword/4 for 32b mode
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Indicate when write is finished; will be done after "stride" cycles
always @(posedge clk, negedge reset_n)
begin
	if (reset_n == 1'b0)
		clusterWriteCount <= 0;
	else if (clk == 1'b1) begin
		if (writesideNextstate == WriteInitWait)
			// so clusterWriteDone is high
			clusterWriteCount <= 1;
		// load the number of cycles and start writing next word when current block is done and data is in intermediate register
		else if (nextCycleIs_clusterWriteFirstCycle == 1'b1) begin
			
			// for large stride, write at one element per cycle, hence need "element per block" number of cycles
			if ((largeStride == 1'b1) || (lsu_indexed_memaccess == 1'b1))
				clusterWriteCount <= numElemPerCluster;
			else
				// Writing each block takes "localStride" cycles
				// do not use localStride - 1 because stride could be zero, which wouldn't work
				clusterWriteCount <= localStride;
		end
		// hold the count value at one when finished, so it can again start next time
		// stops counting at one, otherwise combining (subword_decode_delayed | subwordOffsetValid) in store byte doesn't work properly
		else if ((writesideNextstate == WriteCountState) && (clusterWriteCount > 1) && ~d_waitrequest)
			clusterWriteCount <= clusterWriteCount - 1;
	end
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Counter to count total number of write cycles needed for the instruction
// - optimized for "power of two" number of lanes
// - it takes "stride" cycles to write a complete memory block (filled MEM_WIDTH intermediate register), regardless of offset or data width
// --------------------------------------------------------------------------------------------------------------------------------------------------
// totalWriteCount counts from cycle-1 down to 0
// ceil ( (localStrideByte * (number of elements-1) + offsetByte + elementWidthByte) / how many bytes are written per block (MEMWIDTH_BYTES!)
// elementWidthByte is the location for writing the very last element
assign	totalWriteCount_temp = (localStrideByte * lsu_vectorlength_minus_one - 1 + offsetByte + (1<<convElemToByteShiftConst));
assign	totalWriteCount_sig = totalWriteCount_temp / MEMWIDTH_BYTES;
assign	totalWriteCount_ceil = totalWriteCount_sig;

// Count the cycles that write data to memory
always @(posedge clk, negedge reset_n)
begin
	if (~reset_n)
		totalWriteCount <= 0;
	// do not need to enable the counter for lsu_vext_vv
	else if ((clk == 1'b1) && ~lsu_vext_vv) begin
		// keep initializing until begins writing
		if (writesideState == WriteInitWait)
			// Writing all blocks takes
			// ceil ( (localStride * (number of elements-1) + offset + 1) / how many elements are written per block)
			// 1 cycle less because the first cycle is spent loading the value
			// in the final cycle, totalWriteCount = 0
			// largestride takes same number of cycles as number of subwords in memDataWidth to write a block
			totalWriteCount <= ((largeStride == 1'b1) || (lsu_indexed_memaccess == 1'b1)) ? lsu_vectorlength_minus_one : totalWriteCount_ceil;
			
		// hold the count value at zero when finished, so it can again start next time
		else if ((executing_write == 1'b1) && (writesideNextstate != Idle) && ~d_waitrequest)
			totalWriteCount <= totalWriteCount - 1;
		
	end
end

`ifdef	INDEXED_MEMORY_ACCESS
// Count forward the cycles that write data to memory for index offset select
always @(posedge clk, negedge reset_n)
begin
	if (~reset_n)
		indexOffsetCount <= 0;
	// do not need to enable the counter for lsu_vext_vv
	else if (clk == 1'b1) begin
		// keep initializing until begins writing
		// need to start counting 1 cycle before totalWriteCount due to pipelining
		if (writesideNextstate == WriteInitWait)
			indexOffsetCount <= 0;

		// hold the count value at zero when finished, so it can again start next time
		else if ((lsu_indexed_memaccess == 1'b1) && (writesideNextstate == WriteCountState) && ~d_waitrequest)
			indexOffsetCount <= indexOffsetCount + 1;
		
	end
end
`else
always @(*)
		indexOffsetCount <= 0;
`endif

// Pipeline register
ff	#(.WIDTH(`log2(NUMLANES*`ELEMPERLANE)+1))	ff_indexOffsetCount_delay (.clk(clk), .en(~d_waitrequest), .clrn(reset_n), .d(indexOffsetCount), .q(indexOffsetCount_delay));
ff	#(.WIDTH(`log2(NUMLANES*`ELEMPERLANE)+1))	ff_indexOffsetCount_delay2 (.clk(clk), .en(~d_waitrequest), .clrn(reset_n), .d(indexOffsetCount_delay), .q(indexOffsetCount_delay2));

// ---------------------------------------------------------------------------------------------------------------------------------------
// Calculate next memory address
// ---------------------------------------------------------------------------------------------------------------------------------------
// calculate the next offset position in bytes
// for small offsets, nextoffset = current offset + number of elements per block * stride
// for large offset, nextoffset = current offset + stride
assign	addOffsetByte = (largeStride == 1'b1) ? localStrideByte : ((MEMWIDTH_BYTES>>convElemToByteShiftConst)*localStrideByte % MEMWIDTH_BYTES);
assign	nextOffsetByte = currentOffsetByte + addOffsetByte;

// If it is large stride, increment the memory address by "nextOffsetByte / MEMWIDTH_BYTES"
// otherwise, just increase memory address by one block (MEMWIDTH_BYTES)
// calculate how much to advance memory address; the division does truncation and is needed
assign	nextMemIncByteLargeStride = nextOffsetByte & {{32-`log2(MEMWIDTH_BYTES){1'b1}},{`log2(MEMWIDTH_BYTES){1'b0}}};
// do we need to skip certain memory locations? 
assign 	nextMemAddrIncByte = (largeStride == 1'b1) ? nextMemIncByteLargeStride : MEMWIDTH_BYTES;	
assign 	nextMemAddrSig = nextMemAddr + (nextMemAddrIncByte & {{32-`log2(MEMWIDTH_BYTES){1'b1}},{`log2(MEMWIDTH_BYTES){1'b0}}});

// Is it the first cycle of writing the current cluster/block?
assign	nextCycleIs_clusterWriteFirstCycle = (writeRegDataToMem && clusterWriteDone && ~d_waitrequest) ? 1'b1 : 1'b0;

// need to delay the signal by 1 cycle; cycle after "nextCycleIs_clusterWriteFirstCycle" is the first cycle of writing the current cluster
ff	#(.WIDTH(1))	ff_clusterWriteFirstCycle (.clk(clk), .en(~d_waitrequest), .clrn(reset_n), .d(nextCycleIs_clusterWriteFirstCycle), .q(clusterWriteFirstCycle));

// delay wraddr due to pipelining
ff	#(.WIDTH(32))	ff_nextMemAddr_pipeline (.clk(clk), .en(~d_waitrequest), .clrn(1'b1), .d(nextMemAddr), .q(nextMemAddr_pipeline));
	

// ---------------------------------------------------------------------------------------------------------------------------------------
// Calculate next memory address
// ---------------------------------------------------------------------------------------------------------------------------------------
// Keep track of offset of first element in each data element group
// needed for calculating offsets of data elements in all memory groups
always @(posedge clk)
begin
	// vector-vector extract; configure crossbar to just passthrough data
	if (writesideNextstate == VextVVPassthrough)
		currentOffsetByte <= 0;
	// initialize to offset
	// do not reload scalar address if stalled
	else if ((writesideState == WriteInitWait) || ((writesideNextstate == WriteScalarStore) && ~d_waitrequest))
		currentOffsetByte <= offsetByte;
	// load new offset when one block has been written; or load a new offset for each largeStride write cycle
	else if ((writesideState == WriteCountState) && (clusterWriteDone || largeStride) && ~d_waitrequest)
		currentOffsetByte <= nextOffsetByte % MEMWIDTH_BYTES;
end


// Keep track of the beginning offset of each block
// identical to currentOffsetByte for small strides
// updated when data from the entire group has been transferred to memory
always @(posedge clk)
begin
	// vector-vector extract; configure crossbar to just passthrough data
	if ((writesideNextstate == WriteInitWait) || (writesideNextstate == VextVVPassthrough))
		blockOffsetByte <= 0;

`ifdef	INDEXED_MEMORY_ACCESS
	// for indexed store, store the indexed address offset
	else if ((lsu_indexed_memaccess == 1'b1) && ~d_waitrequest)
		blockOffsetByte <= offsetByte;
`endif

	// initialize to offset
	// do not reload scalar address if stalled
	else if ((writesideState == WriteInitWait) || ((writesideNextstate == WriteScalarStore) && ~d_waitrequest))
		blockOffsetByte <= offsetByte;
	// load new offset when one block has been written
	else if ((writesideState == WriteCountState) && clusterWriteDone && ~d_waitrequest)
		blockOffsetByte <= nextOffsetByte % MEMWIDTH_BYTES;
		
end


// Set next read memory location
always @(posedge clk)
begin
	// ready for transfer
	// do not reload scalar address if stalled
	if ((writesideState == WriteInitWait) || ((writesideNextstate == WriteScalarStore) && ~d_waitrequest)
	// if indexed store, keep loading new indexed memory address
	|| ((lsu_indexed_memaccess == 1'b1) && ~d_waitrequest))
		nextMemAddr <= baseAligned;
	// increment to next memory location
	else if ((writesideState == WriteCountState) && ~d_waitrequest)
		nextMemAddr <= nextMemAddrSig;
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Determine whether need an additional cycle to write data elements delayed by the delay selector
// - need to write an additional cycle if the last element to be written needs to be delayed
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Determine whether to use delayed value align the memory block for writing
// delay the value if
// word access: elem# * stride + offset > MEM_WIDTH/32 * stride
// halfword access: elem# * stride + offset > MEM_WIDTH/16 * stride
// byte access: elem# * stride + offset > MEM_WIDTH/8 * stride
assign	memgroupWriteBytesConst_B = (localStrideByte * (MEM_WIDTH/8));
assign	memgroupWriteBytesConst_HW = (localStrideByte * (MEM_WIDTH/16));
assign	memgroupWriteBytesConst_W = (localStrideByte * (MEM_WIDTH/32));

always @(*)
	case (lsu_memDataWidth)
`ifdef	STORE_WORD_ACCESS_ENABLED
		`MEMACCESS_WORD: 		memgroupWriteBytes = memgroupWriteBytesConst_W;
`endif
`ifdef	STORE_HALFWORD_ACCESS_ENABLED
		`MEMACCESS_HALFWORD: 	memgroupWriteBytes = memgroupWriteBytesConst_HW;
`endif
`ifdef	STORE_BYTE_ACCESS_ENABLED
		`MEMACCESS_BYTE:		memgroupWriteBytes = memgroupWriteBytesConst_B;
`endif
		default:				memgroupWriteBytes = {`D_WIDTH{1'bx}};
	endcase
	


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Write from intermediate register to memory sate machine
// --------------------------------------------------------------------------------------------------------------------------------------------------
// writeside state machine
always @(posedge clk, negedge reset_n) begin
	if (~reset_n)
		writesideState <= Idle;
	else if (clk == 1'b1 && ~d_waitrequest)
		writesideState <= writesideNextstate;
end

// writesideState transition logic
always @ (*)
begin
	case (writesideState)
		Idle:
			// scalar store
			if (scalar_write_grant)
				writesideNextstate <= WriteScalarStore;
			// vector store
			else if (newInstr_store)
				writesideNextstate <= WriteInitWait;
			else
				writesideNextstate <= Idle;

		// wait for intermediate register to be filled for the first data block
		WriteInitWait:
			// vector-vector extract
			if (lsu_vext_vv & writeRegDataToMem)
				writesideNextstate <= VextVVPassthrough;
			else if (writeRegDataToMem)
				writesideNextstate <= WriteCountState;
			else
				writesideNextstate <= WriteInitWait;

		// passthrough state for vector-vector extract
		VextVVPassthrough:
			// when writeRegDataToMem is deasserted, landside has finished writing to the intermediate register
			// writeside only needs to hold write crossbar for 1 more cycle, then it is done
			if (~writeRegDataToMem)
				writesideNextstate <= Idle;
			else
				writesideNextstate <= VextVVPassthrough;

		// writing to main memory
		WriteCountState:
			if (totalWriteCount == 0)
				writesideNextstate <= Idle;
			// order important
			// if last cycle and there is delayed data, write the delayed data
			else if ((totalWriteCount == 1) && (gotoWriteDelayCycle == 1'b1))
				writesideNextstate <= WriteDelayCycle;
			
			// if there is new data to write, keep writing
			else if (writeRegDataToMem)
				writesideNextstate <= WriteCountState;

			// if finished writing this data block, wait for next data block
			else if (clusterWriteDone == 1'b1)
				writesideNextstate <= WriteWaitState;
			
			// if haven't finished writing current block, keep writing
			else
				writesideNextstate <= WriteCountState;

		// waiting for read side to fill intermediate register
		WriteWaitState:
			// make it active only vins.vv incase it can affect operation of other instructions
			if ((lsu_vins_vv == 1'b1) && (totalWriteCount == 0))
				writesideNextstate <= Idle;
			else if (writeRegDataToMem)
				writesideNextstate <= WriteCountState;
				
			else
				writesideNextstate <= WriteWaitState;

		// need to write an extra cycle due to the delayed data values
		WriteDelayCycle:
			writesideNextstate <= Idle;
			
		// scalar write operation
		WriteScalarStore:
			writesideNextstate <= Idle;
	endcase
end


always @(*) begin
	executing_write = 1'b0;
	firstWriteCycle_en = 1'b0;
	executing_vextvv = 1'b0;
	
	case (writesideState)
		WriteInitWait:
			if (writeRegDataToMem)
				firstWriteCycle_en = 1'b1;

		WriteCountState:
			executing_write = 1'b1;

		WriteDelayCycle:
			executing_write = 1'b1;
			
		WriteScalarStore:
			executing_write = 1'b1;
		
		VextVVPassthrough:
			executing_vextvv = 1'b1;

		default:	;
	endcase
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// FOLLOWING LOGIC IS USED TO CONTROL THE DATA ALIGNMENT CROSSBAR
// --------------------------------------------------------------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Stride offset calculation
// - calculate offsets for all subwords to be written taking into account stride
// --------------------------------------------------------------------------------------------------------------------------------------------------
// generate constants for the stride offset calculation
// calculate offset for the four elements
generate
	for (gi=0; gi<MAX_NSUBWORDS; gi=gi+1)
	begin: offsetaddi
		// the offset for each element is just element index % NUMLANES * strideBytes
		assign	offsetAddStrideByte_group [`D_WIDTH*(gi+1)-1 : `D_WIDTH*gi] = (lsu_indexed_memaccess == 1'b1) ? 0 : gi*localStrideByte;
	end
endgenerate

// Register the multiplication output; allowing multicycle to load
// Stride is ready first cycle of initialization
// Output only needs to be ready on first cycle after initialization
always @(posedge clk) begin
	// if (writesideNextstate == WriteScalarStore)
		// offsetAddStrideByte_group_reg <= {`D_WIDTH*MAX_NSUBWORDS{1'b0}};
	if (writesideState == Idle)
		offsetAddStrideByte_group_reg <= {`D_WIDTH*MAX_NSUBWORDS{1'b0}};
	else if (writesideState == WriteInitWait)
		offsetAddStrideByte_group_reg <= offsetAddStrideByte_group;
end


// Generate MAX_NSUBWORDS adders to determine the offsets for each of the data elements
generate
	for (gi=0; gi<MAX_NSUBWORDS; gi=gi+1)
	begin: offseti
		// Offset adder, use Cout signal to determine whether it is valid
		// Width = fit maxstride in bytes + max offset
		lpm_add_sub
		#(.lpm_width(`D_WIDTH), .lpm_direction("ADD"))
		elemOffsetByte_groupAdd_inst
		(
			.dataa		( { {(`log2(MAX_NSUBWORDS)-1){1'b0}}, blockOffsetByte} ),
			.datab		( offsetAddStrideByte_group_reg [ `D_WIDTH*(gi+1)-1 : `D_WIDTH*gi ] ),
			.result		( elemOffsetByte_group [ `D_WIDTH*(gi+1)-1 : `D_WIDTH*gi ] ),
			.cout		()
			,
			.cin(),
			.clken(),
			.overflow(),
			.aclr(),
			.clock(),
			.add_sub()
		);
	end // end for
endgenerate


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Generate offsets for all subwords
// - generate offsets for all subwords
// - masks are used to reduce logic utilization
// --------------------------------------------------------------------------------------------------------------------------------------------------
// expand offsets to offset values for all subwords (byte granularity)
generate
	for (gi=0; gi<MAX_NSUBWORDS; gi=gi+1)
	begin: subwordoffseti

		always @(*) begin
			case (lsu_memDataWidth)
`ifdef	STORE_WORD_ACCESS_ENABLED
				// for word access, need the offset that is the current index/4 + index%4
				// & mask is for synthesis optimization
				`MEMACCESS_WORD: begin
					subwordOffsetByte_group [`D_WIDTH*(gi+1)-1 : `D_WIDTH*gi] =
						(elemOffsetByte_group [`D_WIDTH * ((gi-(gi%(32/MINDATAWIDTH)))*MINDATAWIDTH/32+1) -1 : `D_WIDTH*(gi-(gi%(32/MINDATAWIDTH)))*MINDATAWIDTH/32]  & 32'hFFFFFFFC) + (gi % (32/MINDATAWIDTH));
					subwordFlag_group [gi] = alignedFlagRegQ [gi];
				end
`endif

`ifdef	STORE_HALFWORD_ACCESS_ENABLED
				// for halfword access, need the offset that is the current index/2 + index%2
				// & mask is for synthesis optimization
				`MEMACCESS_HALFWORD: begin
					subwordOffsetByte_group [`D_WIDTH*(gi+1)-1 : `D_WIDTH*gi] =
						(elemOffsetByte_group [`D_WIDTH * ((gi-(gi%(16/MINDATAWIDTH)))*MINDATAWIDTH*2/32+1) -1 : `D_WIDTH*(gi-(gi%(16/MINDATAWIDTH)))*MINDATAWIDTH*2/32]  & 32'hFFFFFFFE) + (gi % (16/MINDATAWIDTH));
					subwordFlag_group [gi] = alignedFlagRegQ [gi];
				end
`endif
				
`ifdef	STORE_BYTE_ACCESS_ENABLED
				// for byte access, need the offset that is the current index
				`MEMACCESS_BYTE: begin
					subwordOffsetByte_group [`D_WIDTH*(gi+1)-1 : `D_WIDTH*gi] = elemOffsetByte_group [`D_WIDTH*(gi+1)-1 : `D_WIDTH*gi];
					subwordFlag_group [gi] = alignedFlagRegQ [gi];
				end
`endif
				default: begin
					subwordOffsetByte_group [`D_WIDTH*(gi+1)-1 : `D_WIDTH*gi] = {`D_WIDTH{1'bx}};
					subwordFlag_group [gi] = 1'bx;
				end
			endcase
		end
	end	// end for
endgenerate	


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Delay data to align between two blocks of data
// - controls the selectable delay
// - delay portions of write data; write delayed data when one group of data has been completely written to memory
// --------------------------------------------------------------------------------------------------------------------------------------------------
assign 	delaySubword_en = (clusterWriteFirstCycle == 1'b1) & ~d_waitrequest ? {MAX_NSUBWORDS{1'b1}} : {MAX_NSUBWORDS{1'b0}};

generate
	for (gi=0; gi<MAX_NSUBWORDS; gi=gi+1)
	begin: delayByteSeli
			
		always @(*)
		begin
			delaySubwordSel[gi] = 1'b0;
			// don't need to delay anything for large stride or indexed store
			// for small stride, delay if: offset > stride * number of elements per MEM_WIDTH (in appropriate data width)
			if ((largeStride == 1'b0) && (lsu_indexed_memaccess == 1'b0))
				delaySubwordSel[gi] = (subwordOffsetByte_group [ `D_WIDTH*(gi+1)-1 : `D_WIDTH*gi ] >= memgroupWriteBytes) ? 1'b1 : 1'b0;
		end
	end
endgenerate


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Check if the element offsets calculated are valid
// - if the calculated offset is out of range (out of the current MEM_WIDTH block), it is invalid and needs to be written on the next cycle
// - range for writing is determined by two comparison values, which give the range of valid offset in bytes from the first write of the current data block
// - generate decode mask
// --------------------------------------------------------------------------------------------------------------------------------------------------

// Adjust the byte offset to minimum width to pass to crossbar decoder/encoder logic
// Minimum memory interface width is 32 bits
generate
	for (gi=0; gi<32/MINDATAWIDTH; gi=gi+1)
	begin: subwordoffseti_minwidth
		assign	subwordOffsetByte_group_minwidth [`D_WIDTH*(gi+1)-1 : `D_WIDTH*gi] =
					// if scalar store, set the subwords within the lowest 32-bit to the correct offset
					(writesideState == WriteScalarStore) ?
					(elemOffsetByte_group [`D_WIDTH-1 : 0] >> `log2(MINDATAWIDTH/8)) + (gi % (32/MINDATAWIDTH))
					// if vector store, adjust the byte offset to minimum width to pass to crossbar decoder/encoder logic
					: subwordOffsetByte_group [`D_WIDTH*(gi+1)-1 : `D_WIDTH*gi] >> `log2(MINDATAWIDTH/8);
	end
endgenerate

// upper bits are not affected by scalar store, so do not need the MUX
generate
	for (gi=32/MINDATAWIDTH; gi<MAX_NSUBWORDS; gi=gi+1)
	begin: subwordoffseti_minwidth_upper
		assign	subwordOffsetByte_group_minwidth [`D_WIDTH*(gi+1)-1 : `D_WIDTH*gi]
				= subwordOffsetByte_group [`D_WIDTH*(gi+1)-1 : `D_WIDTH*gi] >> `log2(MINDATAWIDTH/8);
	end
endgenerate


// Keep track of the threshold for determining whether the data word is valid
assign	elemOffsetByte_group_compare = elemOffsetByte_group_comparereg + nextMemAddrIncByte;
assign	elemOffsetByte_group_compare_minwidth = elemOffsetByte_group_compare >> `log2(MINDATAWIDTH/8);
assign	elemOffsetByte_group_comparereg_minwidth = elemOffsetByte_group_comparereg >> `log2(MINDATAWIDTH/8);

// store to advance the limits the next cycle
always @(posedge clk)
begin
	// clusterWriteCount stays at 0 until write starts, 
	// WriteCountState is enable signal
	if (~d_waitrequest) begin
		if (writeRegDataToMem && (clusterWriteCount <= 1))
			elemOffsetByte_group_comparereg <= 'b0;
		else if (writesideState == WriteCountState)
			elemOffsetByte_group_comparereg <= elemOffsetByte_group_compare;
	end
end


// Pipeline registers
ff	#(.WIDTH(`D_WIDTH * MAX_NSUBWORDS))
ff_subwordOffsetByte_group_pipeline (.clk(clk), .en(~d_waitrequest), .clrn(1'b1), .d(subwordOffsetByte_group_minwidth), .q(subwordOffsetByte_group_pipeline));
ff	#(.WIDTH((`MAXSTRIDE_NBIT+2+MAX_ELEM_PER_CYCLE)+1))
ff_elemOffsetByte_group_compare_pipeline (.clk(clk), .en(~d_waitrequest), .clrn(1'b1), .d(elemOffsetByte_group_compare_minwidth), .q(elemOffsetByte_group_compare_pipeline));
ff	#(.WIDTH((`MAXSTRIDE_NBIT+2+MAX_ELEM_PER_CYCLE)+1))
ff_elemOffsetByte_group_comparereg_pipeline (.clk(clk), .en(~d_waitrequest), .clrn(1'b1), .d(elemOffsetByte_group_comparereg_minwidth), .q(elemOffsetByte_group_comparereg_pipeline));


// --------------------------------------------------------------------------------------------------------------------------------------------------
// compare the calcualted offsets to the limits of the MEM_WIDTH block
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Determine whether each of the 4 element offsets are valid (within the 128b block)
generate
	for (gi=0; gi<MAX_NSUBWORDS; gi=gi+1)
	begin: subwordFlagOni
		// set the flags to always on for vext.vv and vins.vv instructions
		// assign	subwordFlagOn_group[gi] = ((subwordFlag_group[gi] == `FLAG_ON_VALUE) || lsu_vext_vv || lsu_vins_vv) ? 1'b1 : 1'b0;
		
		// flags are already overriden in memwriteIF for vext.vv, vins.vv
		assign	subwordFlagOn_group[gi] = (subwordFlag_group[gi] == `FLAG_ON_VALUE) ? 1'b1 : 1'b0;
	end
endgenerate

// The subword is valid if the offset for the subword is within the memory block being written
// lower limit = elemOffsetByte_group_comparereg; upper limit = elemOffsetByte_group_compare
generate
	for (gi=0; gi<MAX_NSUBWORDS; gi=gi+1)
	begin: elemOffsetValidi
		// 16 bytes in 128b word
		assign	subwordOffsetValid[gi] = ( (subwordOffsetByte_group_pipeline [ `D_WIDTH*(gi+1)-1 : `D_WIDTH*gi ] >= elemOffsetByte_group_comparereg_pipeline) 
											&& (subwordOffsetByte_group_pipeline [ `D_WIDTH*(gi+1)-1 : `D_WIDTH*gi ] < elemOffsetByte_group_compare_pipeline)
											&& (subwordFlagOn_group_pipeline[gi] == 1'b1) ) ? 1'b1 : 1'b0;
	end
endgenerate

`ifdef	INDEXED_MEMORY_ACCESS
// Generate a valid/mask signal for which data element to decode in indexed store
always @(*) begin
	// default
	i=0;
	indexedElemOffsetValid = {MAX_NSUBWORDS{1'b0}};
	
	case (lsu_memDataWidth)
`ifdef	STORE_WORD_ACCESS_ENABLED
	`MEMACCESS_WORD	: begin
		for (i=0; i<MAX_NSUBWORDS; i=i+1) begin
			if ((i >= (((indexOffsetCount_delay2 * 32/MINDATAWIDTH) % MAX_NSUBWORDS))) 
				&& (i <= ((((indexOffsetCount_delay2+1) * 32/MINDATAWIDTH) - 1) % MAX_NSUBWORDS)))
					// only enable if the flag is enabled
					indexedElemOffsetValid[i] = (subwordFlagOn_group_pipeline[i] == 1'b1) ? 1'b1 : 1'b0;
		end
	end
`endif

`ifdef	STORE_HALFWORD_ACCESS_ENABLED
	`MEMACCESS_HALFWORD	: begin
		for (i=0; i< MAX_NSUBWORDS; i=i+1) begin
			if ((i >= (((indexOffsetCount_delay2 * 16/MINDATAWIDTH) % MAX_NSUBWORDS)))
				&& (i <= ((((indexOffsetCount_delay2+1) * 16/MINDATAWIDTH) - 1) % MAX_NSUBWORDS)))
					indexedElemOffsetValid[i] = (subwordFlagOn_group_pipeline[i] == 1'b1) ? 1'b1 : 1'b0;
		end
	end
`endif

`ifdef	STORE_BYTE_ACCESS_ENABLED
	`MEMACCESS_BYTE	: begin
		for (i=0; i< MAX_NSUBWORDS; i=i+1) begin
			if ((i >= (((indexOffsetCount_delay2 * 8/MINDATAWIDTH) % MAX_NSUBWORDS)))
				&& (i <= ((((indexOffsetCount_delay2+1) * 8/MINDATAWIDTH) - 1) % MAX_NSUBWORDS)))
					indexedElemOffsetValid[i] = (subwordFlagOn_group_pipeline[i] == 1'b1) ? 1'b1 : 1'b0;
		end
	end
`endif
	default:	;
	endcase
end
`endif


// Generate delayed datamask bit
always @(posedge clk) begin
	// make sure to reset on new store instruction
	if (writesideState == WriteInitWait)
		subword_decode_delayed <= {MAX_NSUBWORDS{1'b0}};
	// before it starts writing each memory block
	// else if (writeRegDataToMem)
	else if ((clk == 1'b1) && clusterWriteDone && ~d_waitrequest)
		subword_decode_delayed <= delaySubwordSel & subwordFlagOn_group;
end

// Pipeline registers
ff	#(.WIDTH(MAX_NSUBWORDS))
ff_subwordFlagOn_group_pipeline (.clk(clk), .en(~d_waitrequest), .clrn(1'b1), .d(subwordFlagOn_group), .q(subwordFlagOn_group_pipeline));
ff	#(.WIDTH(MAX_NSUBWORDS))
ff_subword_decode_delayed_pipeline (.clk(clk), .en(~d_waitrequest), .clrn(1'b1), .d(subword_decode_delayed), .q(subword_decode_delayed_pipeline));
	
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Decode valid data words
// - one-hot decode the subword offsets, then priority encode them to generate selects to MUX
// --------------------------------------------------------------------------------------------------------------------------------------------------
ff	#(.WIDTH(3))
ff_XbarSelDecodeMask_select_pipeline (.clk(clk), .en(~d_waitrequest), .clrn(1'b1), .d(XbarSelDecodeMask_select), .q(XbarSelDecodeMask_select_pipeline));
// pipeline it to balance pipeline stages
ff	#(.WIDTH(MAX_NSUBWORDS))
ff_scalar_combine_subword_decode_pipeline (.clk(clk), .en(~d_waitrequest), .clrn(1'b1), .d(scalar_combine_subword_decode), .q(scalar_combine_subword_decode_pipeline));

// Select which mask to use for decoding; the select is pipelined
always @(*) begin
	// enable all decoding for Vext.vv
	if (writesideState == VextVVPassthrough)
		XbarSelDecodeMask_select = 3'b001;
	// scalar store overrides vector decode mask
	else if (writesideState == WriteScalarStore)
		XbarSelDecodeMask_select = 3'b010;

`ifdef	INDEXED_MEMORY_ACCESS
	else if (lsu_indexed_memaccess == 1'b1)
		XbarSelDecodeMask_select = 3'b110;
`endif

	// on last cycle of all the store instruction, only write the delayed data if there is delayed data
	else if (writesideState == WriteDelayCycle)
		XbarSelDecodeMask_select = 3'b011;
	// on very first cycle, only decode the non-delayed subwords
	// has same logic as next case because subword_decode_delayed = 0 on very first cycle
	else if (firstWriteCycle)
		XbarSelDecodeMask_select = 3'b100;
	// on first cycle of each block, if there were delayed subwords, decode those as well
	else if (clusterWriteFirstCycle)
		XbarSelDecodeMask_select = 3'b101;
	// intermediate cycles
	else
		XbarSelDecodeMask_select = 3'b000;
end

// Mask to determine whether to decode the word position; 1=decode, 0=don't decode
// generate the data decode mask
always @(*) begin
	case (XbarSelDecodeMask_select_pipeline)
		// enable all decoding for Vext.vv
		// if (writesideState == VextVVPassthrough)
		3'b001	:	XbarSelDecodeMask = {MAX_NSUBWORDS{1'b1}};
		// scalar store overrides vector decode mask
		// else if (writesideState == WriteScalarStore)
		3'b010	:	XbarSelDecodeMask = scalar_combine_subword_decode_pipeline;
		// 3'b010	:	XbarSelDecodeMask = {MAX_NSUBWORDS{1'b1}};
		// on last cycle of all the store instruction, only write the delayed data if there is delayed data
		// else if (writesideState == WriteDelayCycle)
		3'b011	:	XbarSelDecodeMask = subword_decode_delayed_pipeline;
		// on very first cycle, only decode the non-delayed subwords
		// has same logic as next case because subword_decode_delayed = 0 on very first cycle
		// else if (firstWriteCycle)
		3'b100	:	XbarSelDecodeMask = subword_decode_delayed_pipeline | subwordOffsetValid;
		// on first cycle of each block, if there were delayed subwords, decode those as well
		// else if (clusterWriteFirstCycle)
		3'b101	:	XbarSelDecodeMask = subword_decode_delayed_pipeline | subwordOffsetValid;

`ifdef	INDEXED_MEMORY_ACCESS
		3'b110	:	XbarSelDecodeMask = indexedElemOffsetValid;
`endif
		// default intermediate cycles
		3'b000	:	XbarSelDecodeMask = subwordOffsetValid;
		
		default	:	XbarSelDecodeMask = 3'bxxx;
	endcase
end


// decode mask for scalar store; hard code 32b width because the scalar processor is always going to be 32b wide
// the scalar write data is always going to be located in the lower 32b of the memory block
always @(*) begin
	case (lsu_memDataWidth)
		`MEMACCESS_WORD: 		scalar_combine_subword_decode = {MAX_NSUBWORDS{1'b0}} | {32/MINDATAWIDTH{1'b1}};
		// at least enable 1 subword (with `max)
		`MEMACCESS_HALFWORD: 	scalar_combine_subword_decode = {MAX_NSUBWORDS{1'b0}} | {`max(32/(MINDATAWIDTH*2),1){1'b1}};
		// at least enable 1 subword (with `max)
		`MEMACCESS_BYTE:		scalar_combine_subword_decode = {MAX_NSUBWORDS{1'b0}} | {`max(32/(MINDATAWIDTH*4),1){1'b1}};
		default:				scalar_combine_subword_decode = {MAX_NSUBWORDS{1'bx}};
	endcase
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Logic to determine the value for the Xbar MUX selects
// First one-hot decode the offset value
// --------------------------------------------------------------------------------------------------------------------------------------------------
generate
	for (gi=0; gi<MAX_NSUBWORDS; gi=gi+1)
	begin: xbardecodei

		// One hot decode the offset value with enable
		// XbarSelDecodeOnehot is decoded value, the decode mask is used as enable
		onehotdecode_en
		#(	.ENCODEDWIDTH(MAX_NSUBWORDS_NBIT)	)
		XbarSelOnehotDecoder
		(
			.en					( XbarSelDecodeMask[gi] ),
			.encoded			( subwordOffsetByte_group_pipeline [ `D_WIDTH*gi+MAX_NSUBWORDS_NBIT-1 : `D_WIDTH*gi ] ),
			.onehot				( XbarSelDecodeOnehot[MAX_NSUBWORDS*(gi+1)-1 : MAX_NSUBWORDS*gi] )
		);
		
	end	// end for
endgenerate

// Generate priority encoders
generate
	for (gi=0; gi<MAX_NSUBWORDS; gi=gi+1)
	begin: xbarseli
		
		// Priority encode to get the final select value; generate 16 encoders to calculate 16 selects
		// At the same time, generate data masks for the valid values
		always @(*)
		begin
			// default assignment
			writeDmSubword[gi] = 1'b0;
			memWralignXbarSel [MAX_NSUBWORDS_NBIT*(gi+1)-1:MAX_NSUBWORDS_NBIT*gi] = {MAX_NSUBWORDS_NBIT{1'bx}};
		
			// loop backwards so lower indices have higher priority
			for (i=MAX_NSUBWORDS-1; i>=0; i=i-1) begin
				if (XbarSelDecodeOnehot[gi + i*MAX_NSUBWORDS] == 1'b1) begin
					memWralignXbarSel [ MAX_NSUBWORDS_NBIT*(gi+1)-1 : MAX_NSUBWORDS_NBIT*gi ] = i;
					// set data mask by checking whether the offset is valid
					if (XbarSelDecodeMask[i])
						writeDmSubword[gi] = 1'b1;
				end
			end

		end
	end
endgenerate

endmodule
