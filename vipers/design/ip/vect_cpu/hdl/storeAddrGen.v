//************************************************************************
// Scalable Vector CPU Project -- Store controller
// Filename: storeAddrGen.v
// Author: Jason Yu
//
// Memory address generator for stores
// - reads 4 elements (MAX_ELEM_PER_CYCLE) from vector lanes at a time
//   and stores to intermediate register enabled by alignedDataReg_en
// - once intermediate register is filled, the contents are written to memory
//   through the delay and then alignment crossbar network
// - delay network delays elements to align different data blocks
// - crossbar network moves data elements to the correct positions depending on stride and offset
// - elements refer to the 4 data elements from the vector lanes that are processed each cycle
// - subwords refer to the byte-sized (actually, MINDATAWIDTH-sized) subword that make up each data element
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


module storeAddrGen
#(	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1,
	parameter	NUMLANES = 1,
	parameter	MEM_WIDTH = 1,
	parameter	MINDATAWIDTH = 1,
	
	// parameters that shouldn't be changed
	parameter	NUM_MEMGROUP = (NUMLANES/(MEM_WIDTH/VPU_WIDTH)),	// number of "MEM_WIDTH" memory groups to process; division result must be integer
	parameter	MAX_NSUBWORDS = MEM_WIDTH/MINDATAWIDTH				// maximum number of subwords in a memory group
)
(
	// **** Inputs ****
	input			clk,
	input			reset_n,
	input			scalar_write_grant,					// scalar store; overrides all other operations
	input	[31:0]	scalar_d_address,
	input	[31:0]	baseAddr,							// address
	input	[`MAXSTRIDE_NBIT-1:0]	stride,				// vector element stride
	input	[31:0]	vindex,								// vindex control register
	input			lsu_unitstride,						// whether it's unit stride, overrides stride
	input	[2:0]	lsu_memDataWidth,					// width of data access from memory (constants defined in define.v)
	input			newInstr_store,						// indicates a new memory store instruction
	input			flagStoreQEmpty,					// whether the store flag data queue is empty
	input			executing_vector_store,				// indicates vector store is being executed, from loadstore controller
	input			d_waitrequest,						// memory wait request
	input	[31:0]	lsu_vectorlength_minus_one,			// vector length
	input	[MAX_NSUBWORDS-1:0]		alignedFlagRegQ,	// flag from lanes
	input			lsu_flagstore,
	input			lsu_vext_vv,						// vector-vector extract
	input			lsu_vins_vv,						// vector-vector insert
	input			lsu_indexed_memaccess,				// indexed vector store
	
	input		[VPU_WIDTH-1:0]	indexOffset_out,				// offset from lanes for indexed addressing
		
	// **** Outputs ****
	output	[`max(`log2(NUM_MEMGROUP)-1,0):0]	laneWrselMUXSel,			// select lines of the write alignment crossbar
	output	[`log2(MAX_NSUBWORDS) * MAX_NSUBWORDS-1:0]	memWralignXbarSel,	// select for the delay element to memory crossbar
	output	[MEM_WIDTH-1:0]			alignedDataReg_en,						// write enable to register rotated data from rotation crossbar; needs to be bit-granularity to support flag store
	output	[MAX_NSUBWORDS-1:0]		alignedFlagReg_en,						// write enable to register flags from data lanes
	output	[MAX_NSUBWORDS-1:0]		alignedFlagReg_clr,						// mask out the aligned flag register due to element index larger than vector length
	output	[MAX_NSUBWORDS-1:0]		delaySubword_en,						// enable the delay element for the halfword
	output	[MAX_NSUBWORDS-1:0]		delaySubwordSel,						// select data from the delay element for the halfword, 1=select delayed value
	output	[NUMLANES-1:0]			laneStoreQRdreq,						// read request for store queue in lanes
	output							flagStoreQRdreq,						// read request to read flag queue for flag store instruction
	output							vmanip_storeload_sync,					// sync with load controller for vext.vv
	output	[`log2(NUMLANES)-1:0]	lsu_indexOffset_lanesel,	// select which index offset to use
	
	// Outputs to memory
	output	[(MEM_WIDTH/MINDATAWIDTH)-1:0]	vector_mem_dm,		// data mask byte enable, 16 bytes in 128b
	output	[31:0]					mem_addr_0,
	output							d_write,
	output							store_done								// asserted the last cycle of transfer
);

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Local Signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
localparam	MEMWIDTH_BYTES = MEM_WIDTH/8;			// memory width in number of bytes

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Local Signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
wire	loadNextLaneDataGroup_writemem;		// signal from writemem side to load the next group of lane data to alignment register
wire	writeRegDataToMem;					// when asserted, notifies WriteMem side of controller to start writing data from intermediate register to memory
wire	[`log2(MEM_WIDTH/MINDATAWIDTH)-1:0]		lastElemMemgroupPos;				// position of last element to be transferred
reg		[`log2(MEM_WIDTH/MINDATAWIDTH)-1:0]		lastElemMemgroupPosMask;
wire	[31:0]	lsu_flagstore_vectorlength_minus_one;					// vector length - 1 for flag store
wire	[31:0]	lsu_effective_vectorlength_minus_one;
wire	[`log2(NUMLANES*`ELEMPERLANE):0]	indexOffsetCount;	// count how many cycles left in the entire write operation
wire	vextvv_storeload_sync;


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Signal assignments
// --------------------------------------------------------------------------------------------------------------------------------------------------
assign	lsu_indexOffset_lanesel = indexOffsetCount % NUMLANES;

// modify vectorlength to handle flag stores
assign	lsu_flagstore_vectorlength_minus_one = lsu_vectorlength_minus_one / VPU_WIDTH;
assign	lsu_effective_vectorlength_minus_one = (lsu_flagstore == 1'b1) ? lsu_flagstore_vectorlength_minus_one : lsu_vectorlength_minus_one;

// synchronize between load and store units for vector manipulation
// assign	vmanip_storeload_sync = writeRegDataToMem;

// --------------------------------------------------------------------------------------------------------------------------------------------------
// position of last element to be transferred in terms of number of elements within MEM_WIDTH
// ((vector length-1) % number of elements transferred per cluster)
// --------------------------------------------------------------------------------------------------------------------------------------------------
assign	lastElemMemgroupPos = (lsu_effective_vectorlength_minus_one % MEMWIDTH_BYTES) & lastElemMemgroupPosMask;

always @(*) begin
	case (lsu_memDataWidth)
`ifdef	STORE_WORD_ACCESS_ENABLED
		// retain the bottom 2 bits
		// lastElemMemgroupPos = lsu_vectorlength_minus_one % (MEMWIDTH_BYTES >> 2);
		`MEMACCESS_WORD	:		lastElemMemgroupPosMask = ~({`log2(MEM_WIDTH/MINDATAWIDTH){1'b1}} << `log2(MEM_WIDTH/32));
`endif
`ifdef	STORE_HALFWORD_ACCESS_ENABLED
		// retain the bottom 3 bits
		// lastElemMemgroupPos = lsu_vectorlength_minus_one % (MEMWIDTH_BYTES >> 1);
		`MEMACCESS_HALFWORD	:	lastElemMemgroupPosMask = ~({`log2(MEM_WIDTH/MINDATAWIDTH){1'b1}} << `log2(MEM_WIDTH/16));
`endif
`ifdef	STORE_BYTE_ACCESS_ENABLED
		// don't mask out anything
		// lastElemMemgroupPos = lsu_vectorlength_minus_one % MEMWIDTH_BYTES;
		`MEMACCESS_BYTE	:		lastElemMemgroupPosMask = ~({`log2(MEM_WIDTH/MINDATAWIDTH){1'b1}} << `log2(MEM_WIDTH/8));
`endif
		default	:				lastElemMemgroupPosMask = {`log2(MEM_WIDTH/MINDATAWIDTH){1'bx}};
	endcase
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Controller to write from vector lane to intermediate register
// - reads store data from vector lanes
// - store data to intermediate register for writing
// --------------------------------------------------------------------------------------------------------------------------------------------------
storeCtr_fillreg
#(	// Configurable parameters
	.VPU_WIDTH		(VPU_WIDTH),
	.NUMLANES		(NUMLANES),
	.MEM_WIDTH		(MEM_WIDTH),
	.MINDATAWIDTH	(MINDATAWIDTH)
)
storeCtr_fillreg_inst
(
	// **** Inputs ****
	.clk						(clk),
	.reset_n					(reset_n),
	.loadNextLaneDataGroup_writemem	(loadNextLaneDataGroup_writemem),	// signal from writemem side to load the next group of lane data to alignment register
	.lsu_unitstride				(lsu_unitstride),			// whether it's unit stride, overrides stride
	.lsu_memDataWidth			(lsu_memDataWidth),			// width of data access from memory (constants defined in define.v)
	.newInstr_store				(newInstr_store),			// indicates a new memory store instruction
	.flagStoreQEmpty			(flagStoreQEmpty),
	.d_waitrequest				(d_waitrequest),			// memory wait request
	.lsu_vectorlength_minus_one		(lsu_vectorlength_minus_one),		// this is pure vector length, not effective; effective VL only for writemem section; fillreg uses same normal VL for flagstore
	.lastElemMemgroupPos		(lastElemMemgroupPos),
	.lsu_flagstore				(lsu_flagstore),
	.lsu_vext_vv				(lsu_vext_vv),
	.vindex						(vindex),
		
	// **** Outputs ****
	.writeRegDataToMem			(writeRegDataToMem),		// when asserted, notifies WriteMem side of controller to start writing data from intermediate register to memory
	.laneWrselMUXSel			(laneWrselMUXSel),			// select lines of the write alignment crossbar
	.alignedDataReg_en			(alignedDataReg_en),		// write enable to register rotated data from rotation crossbar
	.alignedFlagReg_en			(alignedFlagReg_en),		// write enable to register flags from data lanes
	.alignedFlagReg_clr			(alignedFlagReg_clr),		// mask out the aligned flag register due to element index larger than vector length
	.flagStoreQRdreq			(flagStoreQRdreq),
	.laneStoreQRdreq			(laneStoreQRdreq)		// read request for store queue in clusters
);


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Controller to write to memory
// - writes data from intermediate register to memory
// - controls memory write signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
storeCtr_writemem
#(	// Configurable parameters
	.VPU_WIDTH		(VPU_WIDTH),
	.NUMLANES		(NUMLANES),
	.MEM_WIDTH		(MEM_WIDTH),
	.MINDATAWIDTH	(MINDATAWIDTH)
)
storeCtr_writemem_inst
(
	// **** Inputs ****
	.clk						(clk),
	.reset_n					(reset_n),
	.newInstr_store				(newInstr_store),			// indicates a new memory store instruction
	.scalar_write_grant			(scalar_write_grant),		// scalar store; overrides all other operations
	.scalar_d_address			(scalar_d_address),
	.baseAddr					(baseAddr),					// base address
	.stride						(stride),					// vector element stride
	.lsu_memDataWidth			(lsu_memDataWidth),			// width of data access from memory (constants defined in define.v)
	.lsu_unitstride				(lsu_unitstride),			// whether it's unit stride, overrides stride
	.executing_vector_store		(executing_vector_store),	// indicates vector store is being executed, from loadstore controller
	.d_waitrequest				(d_waitrequest),			// memory wait request
	.alignedFlagRegQ			(alignedFlagRegQ),			// flag from lanes
	.writeRegDataToMem			(writeRegDataToMem),		// input from fillreg side, start writing the lane data group to memory
	.lsu_vectorlength_minus_one	(lsu_effective_vectorlength_minus_one),
	.lastElemMemgroupPos		(lastElemMemgroupPos),
	.lsu_flagstore				(lsu_flagstore),
	.lsu_vext_vv				(lsu_vext_vv),
	.lsu_vins_vv				(lsu_vins_vv),
	.lsu_indexed_memaccess		(lsu_indexed_memaccess),
	.vindex						(vindex),
	
	.indexOffset_out			(indexOffset_out),			// indexed addressing offset
		
	// **** Outputs ****
	.vmanip_storeload_sync		(vmanip_storeload_sync),
	.indexOffsetCount			(indexOffsetCount),
	.loadNextLaneDataGroup		(loadNextLaneDataGroup_writemem),	// load the next group of lane data to alignment register
	.memWralignXbarSel			(memWralignXbarSel),		// select for the delay element to memory crossbar
	.delaySubword_en			(delaySubword_en),			// enable the delay element for the halfword
	.delaySubwordSel			(delaySubwordSel),			// select data from the delay element for the halfword, 1=select delayed value
	// Outputs to memory
	.vector_mem_dm				(vector_mem_dm),			// data mask byte enable, 16 bytes in 128b
	.mem_addr_0					(mem_addr_0),
	.d_write					(d_write),
	.store_done					(store_done)				// asserted the last cycle of transfer
);
	
endmodule
