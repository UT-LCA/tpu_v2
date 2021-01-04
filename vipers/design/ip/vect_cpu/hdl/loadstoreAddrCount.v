//************************************************************************
// Scalable Vector CPU Project -- Load/store address counter
// Filename: loadstoreAddrCount.v
// Author: Jason Yu
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


module loadstoreAddrCount
#(	
	// Parameters
	parameter	NUM_MEMGROUP = 2,
	parameter	MEM_WIDTH = 128,
	parameter	MINDATAWIDTH = 8,

	// Local parameters
	parameter	MAX_NUMELEMS = `NUMLANES * `ELEMPERLANE,	// number of total data elements
	parameter	MAX_NSUBWORDS = MEM_WIDTH/MINDATAWIDTH,							// number of subwords
	parameter	READMUX_NSEL = `log2(MEM_WIDTH/MINDATAWIDTH),					// number of select line on each MUX of read crossbar
	parameter	MEMWIDTH_BYTES = MEM_WIDTH/8									// memory width in number of bytes
)

// Ports
(
	input							clk,
	input		[31:0]				baseAddr,
	input		[31:0]				scalar_d_address,
	input		[`MAXSTRIDE_NBIT-1:0]	stride,						// signed stride, in terms of number of elements
	input		[31:0]				vindex,							// control register, vindex
	input							lsu_vext_vv,					// vector-vector extract instruction
	input							lsu_vins_vv,
	input							s3_unitstride,					// whether it's unit stride; overrides stride
	input							s3_indexed_memaccess,			// indexed memory load
	input		[`VPU_WIDTH-1:0]	indexOffset_out,				// offset from lanes for indexed addressing
	input		[2:0]				s3_memDataWidth,				// width of data access from memory
	
	// Load only signals
	input							lsu_executing_vector_load,
	input		[`log2(MAX_NUMELEMS):0]	loadElemCount,				// counter to cycle through all lanes; need 1 additional bit
	input							vector_load_inc_rdaddr,
	input							s3_newLoad,						// indicates a new memory instruction
	input							lsu_scalar_load,				// scalar memory access
	input							lsu_flagload,
	input							loadDataQ_full,					// the lane load data queue is full
	input							vectorload_waitrequest,			// load data from memory is now valid, otherwise stall
	
	output		[31:0]						baseAddrScalarSel,			// base address after scalar select MUX
	output reg	[`log2(MEMWIDTH_BYTES)-1:0]	offsetByte,
	output reg	[`log2(MEMWIDTH_BYTES)-1:0]	baseOffsetElem,				// number of element offsets; direct from baseAddr, not after adding Index

	output		[31:0]				memAddr,							// address to memory
	output		[`log2(`NUMLANES)-1:0]		lsu_indexOffset_lanesel		// select which lane to get index offset from
);

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
reg		[31:0]						nextMemAddr;				// next memory read address
wire	[31:0]						baseAligned;				// base address aligned to 128 bit
wire	[31:0]						baseAligned_scalar;			// base address for scalar/indexed loads
reg		[1:0]						convElemByteShiftConst;		// Number of positions to shift to convert from units of elements to bytes
wire	[`MAXSTRIDE_NBIT+2:0]		localStride;				// local stride signal
wire	[`MAXSTRIDE_NBIT+2:0]		strideByte;					// stride in number of bytes
wire	[31:0]						indexOffset_sext;			// sign extend indexed offset if VPU_WIDTH is less than 32
wire	[`VPU_WIDTH-1:0]			indexOffset;				// current index offset
wire	[31:0]						baseAddrIndexAdded;			// base address with added index (index is zero'd for non-index accesses)
wire								largeStride;				// whether strideByte > 16
reg		[`MAXSTRIDE_NBIT+2:0]		nextOffsetByteLargeStride;	// important to keep this bit width
wire	pauseLoadController;									// pause the load controller
wire	[`VPU_WIDTH-1:0]			nextMemIncByteLargestride;	// number of bytes to increase read address by for largeStride read
wire	[31:0]						nextMemIncByteAligned;		// increment to the current memory read address


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Strided memory load signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Memory address; indexed memory access does not use registered address signal due to extra cycle latency
assign 	memAddr = (lsu_scalar_load | s3_indexed_memaccess) ? baseAligned_scalar : nextMemAddr;

// pause the controller if eiher the vector lane data queue is full, or need to wait for memory
assign	pauseLoadController = loadDataQ_full | vectorload_waitrequest;

// Memory offset signals
assign 	baseAligned = baseAddr & ~{32'h00000000, {`log2(MEMWIDTH_BYTES){1'b1}}};			// truncate to MEMWIDTH aligned address

// Memory stride signals
assign 	localStride = (s3_unitstride == 1'b1) ? 1 : stride;					// flag load also sets unitstride
assign	strideByte = localStride << convElemByteShiftConst;			// convert stride in elements to stride in bytes


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Indexed/scalar memory load bypass signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
// sign extend index offset if it is less than 32
assign	indexOffset_sext = {{(32-`VPU_WIDTH){indexOffset_out[`VPU_WIDTH-1]}}, indexOffset_out};
assign	lsu_indexOffset_lanesel = (s3_newLoad == 1'b1) ? 0 : loadElemCount % `NUMLANES;		// set to zero on first cycle for processor stalling

// Zero indexOffset for non-indexed memory accesses
assign	indexOffset = (s3_indexed_memaccess == 1'b1) ? indexOffset_sext : `VPU_WIDTH'b0;
assign	baseAddrIndexAdded = baseAddr + indexOffset;								// indexed address

// Scalar processor override
assign	baseAddrScalarSel = (lsu_scalar_load == 1'b1) ? scalar_d_address : baseAddrIndexAdded;
assign 	baseAligned_scalar = baseAddrScalarSel & ~{32'h00000000, {`log2(MEMWIDTH_BYTES){1'b1}}};			// truncate to MEMWIDTH aligned address


// define "largestride" as strideByte > 16; largestride requires skipping memory locations
assign 	largeStride = (strideByte > MEMWIDTH_BYTES) ? 1'b1 : 1'b0;
assign	nextMemIncByteLargestride = (nextOffsetByteLargeStride / MEMWIDTH_BYTES) << `log2(MEMWIDTH_BYTES);		// calculate how much to advance memory address

// assign 	nextMemIncByteAligned = (largeStride == 1'b1) ? nextMemIncByteLargestride : MEMWIDTH_BYTES;



// do we need to skip certain memory locations? 
assign 	nextMemAddrIncByte = (largeStride == 1'b1) ? nextMemIncByteLargeStride : MEMWIDTH_BYTES;	
assign 	nextMemAddrSig = nextMemAddr + (nextMemAddrIncByte & {{32-`log2(MEMWIDTH_BYTES){1'b1}},{`log2(MEMWIDTH_BYTES){1'b0}}});

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Strided memory load signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
// store
// assign 	offsetByte = (lsu_vins_vv) ? vindex << convElemToByteShiftConst : storeAddr % MEMWIDTH_BYTES;					// offset from 128 bit alignment in bytes

// load
// offset from 128 bit alignment in bytes
always @(*) begin
`ifdef	VECTOR_MANIP_INSTRUCTIONS
	if (lsu_vins_vv == 1'b1)
		offsetByte = 0;
	// vext.vv sets offset to be vindex that we're extracting from
	else if (lsu_vext_vv == 1'b1)
		offsetByte = (vindex % `LANES_PER_MEMGROUP) << convElemByteShiftConst;
	// for all other access
	else
`endif
		offsetByte = baseAddr % MEMWIDTH_BYTES;
end

// base address offset in number of elements
always @(*) begin
`ifdef	VECTOR_MANIP_INSTRUCTIONS
	if (lsu_vins_vv == 1'b1)
		baseOffsetElem = 0;
	// vext.vv sets offset to be vindex that we're extracting from
	else if (lsu_vext_vv == 1'b1)
		baseOffsetElem = vindex % `LANES_PER_MEMGROUP;
	// for all other access
	else
`endif
		baseOffsetElem = (baseAddr % MEMWIDTH_BYTES) >> convElemByteShiftConst;
end

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Keep track of offset of current element; needed for skipping memory locations for stride > 8 (16b mode), or stride > 4 (32b mode)
// - only used for largeStride transfers
// - only need to keep track of 1 offset since for these strides there can't be more than 1 valid element per 128b block
// --------------------------------------------------------------------------------------------------------------------------------------------------
// always @(posedge clk) begin
	// if (s3_newLoad == 1'b1)
		// nextOffsetByteLargeStride <= offsetByte + strideByte;
	// else if (clk == 1'b1 && lsu_executing_vector_load && ~pauseLoadController)
		// nextOffsetByteLargeStride <= (nextOffsetByteLargeStride % MEMWIDTH_BYTES) + strideByte;
// end



// Keep track of offset of first element in each data element group
// needed for calculating offsets of data elements in all memory groups
always @(posedge clk)
begin
	// vector-vector extract; configure crossbar to just passthrough data
	// if (writesideNextstate == VextVVPassthrough)
		// currentOffsetByte <= 0;
	// initialize to offset
	// do not reload scalar address if stalled
	if ((writesideState == WriteInitWait) || ((writesideNextstate == WriteScalarStore) && ~d_waitrequest))
		currentOffsetByte <= offsetByte;

	// Load conditions
	else if (s3_newLoad == 1'b1)
		currentOffsetByte <= offsetByte + strideByte;

	// Store condition
	// load new offset when one block has been written; or load a new offset for each largeStride write cycle
	else if (((writesideState == WriteCountState) && (clusterWriteDone || largeStride) && ~d_waitrequest)
	
	// Load condition
			|| (lsu_executing_vector_load && ~pauseLoadController))
		currentOffsetByte <= nextOffsetByte % MEMWIDTH_BYTES;
end


assign	nextOffsetByte = currentOffsetByte + addOffsetByte;
assign	addOffsetByte = ((largeStride == 1'b1) || (lsu_executing_vector_load == 1'b1)) ?
						strideByte : ((MEMWIDTH_BYTES>>convElemToByteShiftConst)*strideByte % MEMWIDTH_BYTES);
assign	nextMemIncByteLargeStride = nextOffsetByte & {{32-`log2(MEMWIDTH_BYTES){1'b1}},{`log2(MEMWIDTH_BYTES){1'b0}}};

// ---------------------------------------------------------------------------------------------------------------------------------------
// Calculate next memory address
// ---------------------------------------------------------------------------------------------------------------------------------------
// Keep track of offset of first element in each data element group
// needed for calculating offsets of data elements in all memory groups
// always @(posedge clk)
// begin
	// vector-vector extract; configure crossbar to just passthrough data
	// if (writesideNextstate == VextVVPassthrough)
		// currentOffsetByte <= 0;
	// initialize to offset
	// do not reload scalar address if stalled
	// else if ((writesideState == WriteInitWait) || ((writesideNextstate == WriteScalarStore) && ~d_waitrequest))
		// currentOffsetByte <= offsetByte;
	// load new offset when one block has been written; or load a new offset for each largeStride write cycle
	// else if ((writesideState == WriteCountState) && (clusterWriteDone || largeStride) && ~d_waitrequest)
		// currentOffsetByte <= nextOffsetByte % MEMWIDTH_BYTES;
// end




// --------------------------------------------------------------------------------------------------------------------------------------------------
// Select next memory address
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Set next read memory location
// always @(posedge clk)
// begin
	// Pause if pauseLoadController
	// if (clk == 1'b1 && lsu_executing_vector_load && ~pauseLoadController) begin
		// Use baseAligned + index offset for index accesses
		// if (s3_newLoad)
			// nextMemAddr <= baseAligned;
		// if lsu_flagload and we have read all MEM_WIDTH bits, advance to next address
		// else if (((lsu_flagload == 1'b1) && (loadElemCount % MEM_WIDTH == 0) && (loadElemCount != 0))
				// for regular loads, only advance address if the current block has been completely read
				// || ((lsu_flagload == 1'b0) && vector_load_inc_rdaddr))
			// nextMemAddr <= nextMemAddr + nextMemIncByteAligned;
	// end
// end


// Set next read memory location
always @(posedge clk)
begin
	// Pause if pauseLoadController
	if (clk == 1'b1) begin
		// load condition
		if ((lsu_executing_vector_load && ~pauseLoadController && s3_newLoad)
		// store condition
			|| 	((writesideState == WriteInitWait) || ((writesideNextstate == WriteScalarStore) && ~d_waitrequest)))
			nextMemAddr <= baseAligned;

		// if lsu_flagload and we have read all MEM_WIDTH bits, advance to next address
		else if ((lsu_executing_vector_load && ~pauseLoadController
			&& (((lsu_flagload == 1'b1) && (loadElemCount % MEM_WIDTH == 0) && (loadElemCount != 0))
			// for regular loads, only advance address if the current block has been completely read
			|| ((lsu_flagload == 1'b0) && vector_load_inc_rdaddr)))
			
			// Store condition
			// increment to next memory location
			|| ((writesideState == WriteCountState) && ~d_waitrequest))
				nextMemAddr <= nextMemAddrSig;
	end
end


// Set next read memory location
// always @(posedge clk)
// begin
	// ready for transfer
	// do not reload scalar address if stalled
	// if ((writesideState == WriteInitWait) || ((writesideNextstate == WriteScalarStore) && ~d_waitrequest))
		// nextMemAddr <= baseAligned;
	// increment to next memory location
	// else if ((writesideState == WriteCountState) && ~d_waitrequest)
		// nextMemAddr <= nextMemAddrSig;
// end

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Convert stride in elements to stride in bytes conversion factor
// --------------------------------------------------------------------------------------------------------------------------------------------------
always @(*) begin
	case (s3_memDataWidth)
`ifdef	LOAD_WORD_ACCESS_ENABLED
		`MEMACCESS_WORD	:		convElemByteShiftConst = `log2(32/8);
`endif
`ifdef	LOAD_HALFWORD_ACCESS_ENABLED
		`MEMACCESS_HALFWORD	:	convElemByteShiftConst = `log2(16/8);
`endif
`ifdef	LOAD_BYTE_ACCESS_ENABLED
		`MEMACCESS_BYTE	:		convElemByteShiftConst = `log2(8/8);
`endif
		default	:				convElemByteShiftConst = 2'bx;
	endcase
end






// --------------------------------------------------------------------------------------------------------------------------------------------------
// Store address counting
// --------------------------------------------------------------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Assign outputs
// --------------------------------------------------------------------------------------------------------------------------------------------------
assign 	mem_addr_0 = nextMemAddr_pipeline;							// address to memory

// delay wraddr due to pipelining
ff	#(.WIDTH(32))	ff_nextMemAddr_pipeline (.clk(clk), .en(~d_waitrequest), .clrn(1'b1), .d(nextMemAddr), .q(nextMemAddr_pipeline));


// ---------------------------------------------------------------------------------------------------------------------------------------
// Calculate next memory address
// ---------------------------------------------------------------------------------------------------------------------------------------
// calculate the next offset position in bytes
// for small offsets, nextoffset = current offset + number of elements per block * stride
// for large offset, nextoffset = current offset + stride
assign	addOffsetByte = (largeStride) ? strideByte : ((MEMWIDTH_BYTES>>convElemToByteShiftConst)*strideByte % MEMWIDTH_BYTES);
assign	nextOffsetByte = currentOffsetByte + addOffsetByte;

// If it is large stride, increment the memory address by "nextOffsetByte / MEMWIDTH_BYTES"
// otherwise, just increase memory address by one block (MEMWIDTH_BYTES)
// calculate how much to advance memory address; the division does truncation and is needed
assign	nextMemIncByteLargeStride = nextOffsetByte & {{32-`log2(MEMWIDTH_BYTES){1'b1}},{`log2(MEMWIDTH_BYTES){1'b0}}};
// do we need to skip certain memory locations? 
assign 	nextMemAddrIncByte = (largeStride == 1'b1) ? nextMemIncByteLargeStride : MEMWIDTH_BYTES;	
// assign 	nextMemAddrSig = nextMemAddr + (nextMemAddrIncByte & {{32-`log2(MEMWIDTH_BYTES){1'b1}},{`log2(MEMWIDTH_BYTES){1'b0}}});


endmodule
