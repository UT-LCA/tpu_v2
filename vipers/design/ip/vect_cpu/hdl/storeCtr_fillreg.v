//************************************************************************
// Scalable Vector CPU Project -- Memory store, intermediate register fill logic
// Filename: storeFillRegCtr.v
// Author: Jason Yu
//
// Write from vector lanes to intermediate register logic
// 
// - the vector lane side state machine writes data from vector lane store queues to the intermediate
// register at MAX_ELEM_PER_CYCLE elements per cycle
// - once the intermediate register is filled, "writeRegDataToMem" is asserted to start
// the write side state machine to write data to memory
// - only up to vector length data elements are written to the intermediate register
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


module storeCtr_fillreg
#(	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1,
	parameter	NUMLANES = 1,
	parameter	MEM_WIDTH = 1,
	parameter	MINDATAWIDTH = 1,
	
	// Parameters that shouldn't be changed but needed in port declaration
	parameter	NUM_MEMGROUP = (NUMLANES/(MEM_WIDTH/VPU_WIDTH)),	// number of "MEM_WIDTH" memory groups to process; division result must be integer
	parameter	MAX_NSUBWORDS = MEM_WIDTH/MINDATAWIDTH		// maximum number of subwords in a memory group
)
(
	// **** Inputs ****
	input							clk,
	input							reset_n,
	input							loadNextLaneDataGroup_writemem,		// signal from writemem side to load the next group of lane data to alignment register
	input							lsu_unitstride,						// whether it's unit stride, overrides stride
	input		[2:0]				lsu_memDataWidth,					// width of data access from memory (constants defined in define.v)
	input							newInstr_store,						// indicates a new memory store instruction
	input							flagStoreQEmpty,					// whether the store flag data queue is empty; it is used in both data store and flag store, whereas the data queue is only used in data store
	input							d_waitrequest,						// memory wait request
	input		[31:0]				lsu_vectorlength_minus_one,	// vector length
	input		[`log2(MEM_WIDTH/MINDATAWIDTH)-1:0]	lastElemMemgroupPos,		// position of last element to be transferred
	input							lsu_flagstore,
	input							lsu_vext_vv,
	input		[31:0]				vindex,								// vindex control register
		
	// **** Outputs ****
	output							writeRegDataToMem,		// when asserted, notifies WriteMem side of controller to start writing data from intermediate register to memory
	output		[`max(`log2(NUM_MEMGROUP)-1,0):0]	laneWrselMUXSel,	// select lines of the write alignment crossbar
	output 		[MEM_WIDTH-1:0]		alignedDataReg_en,		// write enable to register rotated data from rotation crossbar; needs to be bit-granularity to support flag store
	output reg	[MAX_NSUBWORDS-1:0]	alignedFlagReg_en,		// write enable to register flags from data lanes
	output reg	[MAX_NSUBWORDS-1:0]	alignedFlagReg_clr,		// mask out the aligned flag register due to element index larger than vector length
	output reg						flagStoreQRdreq,
	// output reg	[NUM_MEMGROUP-1:0]	clusterStoreQRdreq		// read request for store queue in clusters
	output reg	[NUMLANES-1:0]		laneStoreQRdreq		// read request for store queue in vector lanes
);

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Local Parameters
// --------------------------------------------------------------------------------------------------------------------------------------------------
// parameters that shouldn't be changed
localparam	NUMCLUSTERELEMS = NUM_MEMGROUP * `ELEMPERLANE;			// number of memory groups over all data elements
localparam	MEMWIDTH_BYTES = MEM_WIDTH/8;							// memory width in number of bytes
localparam	FLAGSTORE_GRANULARITY = `min(NUMLANES, VPU_WIDTH);	// granularity of control signals needed to control flag store
localparam	MAX_ELEM_PER_CYCLE = `min(MEM_WIDTH/VPU_WIDTH,NUMLANES);	// number of elements that can be processed per cycle; cannot really be changed
localparam	MVL_LIMIT = NUMLANES * `ELEMPERLANE;

// State machine states
// Two state machines are used, one for writing data from vector lanes to intermediate register (lanesideState)
// the other for writing data from intermediate register to memory (writesideState)
localparam
	Idle = 0,
	LoadDataToIntReg = 1,
	IntRegWait = 2;			// lanesidestate states
	
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Local Signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
wire									writeRegDataToMem_sig;
wire	[`log2(NUMLANES*`ELEMPERLANE):0]	laneSelCount_delay;
reg		[`log2(NUMLANES*`ELEMPERLANE):0]	laneSelCount_inc;
// reg		[`log2(MAX_ELEM_PER_CYCLE):0]	laneDataLoadCount;			// a counter to count how many cycles to fill the intermediate register; maximum MAX_ELEM_PER_CYCLE cycles; use 1 extra bit
reg		[`log2(MEM_WIDTH/`min(FLAGSTORE_GRANULARITY,8))-1:0]		laneDataLoadCount;
reg		[`log2(MEM_WIDTH/`min(FLAGSTORE_GRANULARITY,8))-1:0]		laneDataLoadCount_const;	// value to initialize the load data counter to
reg										laneDataLoadCount_cnt;		// whether to increment the cluster load data counter

wire	[`log2(NUM_MEMGROUP)-1:0]		laneWrselMUXSel_delay;		// select lines of the write alignment crossbar
wire									VLaneStoreQReady;
wire									flagStoreQEmpty_delay;
wire									loadNextLaneDataGroup;

wire	[`log2(NUMLANES*`ELEMPERLANE):0]		totalNumElem_minus_one;	// how many clusters to process for the entire memory instruction

wire	[`log2(NUMCLUSTERELEMS) : 0]	clusterSelCount_delay;		// count to cycle through clusters (ends with value = NUM_MEMGROUP*ELEMPERLANE)
reg		[MEMWIDTH_BYTES-1:0]			alignedDataReg_datawr_subword_en;		// write enable to register rotated data from rotation crossbar; needs to be bit-granularity to support flag store
reg		[MEM_WIDTH-1:0]					alignedDataReg_datawr_bit_en;			// write enable to register rotated data from rotation crossbar; needs to be bit-granularity to support flag store
reg		[MEM_WIDTH/FLAGSTORE_GRANULARITY-1:0]	alignedDataReg_flagwr_en;		// write enable to register rotated data from rotation crossbar; needs to be bit-granularity to support flag store
reg		[MEM_WIDTH-1:0]					alignedDataReg_flagwr_bit_en;			// write enable to register rotated data from rotation crossbar; needs to be bit-granularity to support flag store
wire	[`log2(NUM_MEMGROUP)-1:0]		laneWrselMUXSel_delay_sig;		// select lines of the write alignment crossbar
reg		[`log2(NUMLANES*`ELEMPERLANE):0]	laneSelCount;
	
// Two state machines are used, one for writing data from vector lanes to intermediate register (lanesideState)
// the other for writing data from intermediate register to memory (writesideState)
reg		[1:0]	lanesideState, lanesideNextState;
wire	[1:0]	lanesideLastState;			// lane side state machine state register

integer	i;		// loop variables

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Constants for how many cycles is needed to load data from lanes into intermediate register
// ------------------------------------------------------------------------------------------------------------------------------------------------
// How many cycles maximum it takes to fill the intermediate register with lane data
// cycles = MEMDATA_WIDTH / (MAX_ELEM_PER_CYCLE * data width in bits)
always @(*)
	if (lsu_flagstore == 1'b1)
		// initialized to true value minus 1 since 1st cycle is for loading
		laneDataLoadCount_const = `min((MEM_WIDTH/FLAGSTORE_GRANULARITY)-1, (MVL_LIMIT/FLAGSTORE_GRANULARITY)-1);
	else begin
	// initialized to true value minus 1 since 1st cycle is for loading
	case (lsu_memDataWidth)
`ifdef	STORE_WORD_ACCESS_ENABLED
		`MEMACCESS_WORD:		laneDataLoadCount_const = MEM_WIDTH / (32*MAX_ELEM_PER_CYCLE) - 1;
`endif
`ifdef	STORE_HALFWORD_ACCESS_ENABLED
		`MEMACCESS_HALFWORD:	laneDataLoadCount_const = MEM_WIDTH / (16*MAX_ELEM_PER_CYCLE) - 1;
`endif
`ifdef	STORE_BYTE_ACCESS_ENABLED
		`MEMACCESS_BYTE:		laneDataLoadCount_const = MEM_WIDTH / (8*MAX_ELEM_PER_CYCLE) - 1;
`endif
		default:				laneDataLoadCount_const = 3'bxxx;
	endcase
	end

// --------------------------------------------------------------------------------------------------------------------------------------------------	
// Signal assignments
// --------------------------------------------------------------------------------------------------------------------------------------------------
assign	totalNumElem_minus_one = lsu_vectorlength_minus_one;

// pulse 1 cycle after falling edge of clusterStoreQEmpty; the memory write part can start executing the store
// using flag queue now because it is filled for both normal data stores and flag stores
// assign	VLaneStoreQReady = (~flagStoreQEmpty) & (flagStoreQEmpty_delay);
assign	VLaneStoreQReady = (~flagStoreQEmpty) & newInstr_store;

// Select which vector lanes to read from
// use the delay signal because selects which cluster to read from, but lane data is not available until 1 cycle later
assign	laneWrselMUXSel = laneWrselMUXSel_delay;
assign	laneWrselMUXSel_delay_sig = laneSelCount / (MEM_WIDTH/VPU_WIDTH);
ff	#(.WIDTH(`max(`log2(NUM_MEMGROUP),1)))	ff_laneWrselMUXSel_delay (.clk(clk), .en(~d_waitrequest), .clrn(reset_n), .d( laneWrselMUXSel_delay_sig[`max(`log2(NUM_MEMGROUP)-1,0):0] ), .q(laneWrselMUXSel_delay));

	
// Signal to indicate when to start the write side state machine, only pulsed when data is ready in the intermediate register
// if data has been completely stored into the intermediate register, or if we are loading the last cluster, or if the register has not been completely read
assign	writeRegDataToMem_sig = ((lanesideState == LoadDataToIntReg) && (laneDataLoadCount == 0))
							|| ((lanesideState == LoadDataToIntReg) && (laneSelCount >= totalNumElem_minus_one));

// delay the output signal by 1 cycle due to pipeline stage
ff	#(.WIDTH(1))	ff_writeRegDataToMem_delay (.clk(clk), .en(~d_waitrequest), .clrn(reset_n), .d(writeRegDataToMem_sig), .q(writeRegDataToMem));
	
// delay the store queue empty signal by 1 cycle
ff	#(.WIDTH(1))	ff_flagStoreQEmpty_delay (.clk(clk), .en(~d_waitrequest), .clrn(reset_n), .d(flagStoreQEmpty), .q(flagStoreQEmpty_delay));

// delayed clusterSelCount, used to write data read from store Q to the write intermediate register
assign	clusterSelCount_delay = (lsu_flagstore == 1'b1) ? (laneSelCount_delay/FLAGSTORE_GRANULARITY) : (laneSelCount_delay / (MEM_WIDTH/VPU_WIDTH));
ff	#(.WIDTH(`log2(NUMLANES*`ELEMPERLANE)+1))	ff_laneSelCount_delay (.clk(clk), .en(~d_waitrequest), .clrn(reset_n), .d(laneSelCount), .q(laneSelCount_delay));

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Signal to start loading data into intermediate register before delay/align crossbar
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Start loading next set of data into alignment register when receive "loadNextLaneDataGroup_writemem" signal,
// or continuously load for unitstride, flagstore access
// lsu_vext_vv, lsu_vins_vv not needed because they assert unitstride too and operates the same way
assign	loadNextLaneDataGroup = (loadNextLaneDataGroup_writemem
								// unitstride don't need to wait for write side because only takes 1 cycle to write data in the intermediate register
								| (lsu_unitstride && (laneDataLoadCount == 0)) | (lsu_flagstore && (laneDataLoadCount == 0))) ? 1'b1 : 1'b0;

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Count number of cycles to fill intermediate register
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Counter to count how many cycles are needed to load the intermediate register
always @(posedge clk, negedge reset_n)
begin
	// stop the counter when store finished
	if (~reset_n)
		laneDataLoadCount <= 0;
	else if ((clk == 1'b1) && ~d_waitrequest) begin
		// reset at the beginning of each block to be transferred
		// for unitstride, reset immediately after reaching zero
		// flagstore, vext.vv, vins.vv also assert lsu_unitstride, so they are covered as well		
		// if (VLaneStoreQReady || (lanesideState == IntRegWait) || (lsu_unitstride && (laneDataLoadCount == 0)))
		if	(((lanesideState == Idle) && (lanesideNextState == LoadDataToIntReg))
			|| (lanesideState == IntRegWait)
			|| ((laneDataLoadCount == 0) && (lsu_unitstride == 1'b1)))
			laneDataLoadCount <= laneDataLoadCount_const;

		else if (laneDataLoadCount_cnt == 1'b1)
			laneDataLoadCount <= laneDataLoadCount - 1;
	end
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Select data from vector lanes to pass to memory
// Also doubles to count which bits in alignedDataReg_en to enable in flag store
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Selects to lane write data select MUX
always @(posedge clk)
begin
	// synchronous reset
	if ((newInstr_store & ~lsu_vext_vv) == 1'b1)
		laneSelCount <= 0;
`ifdef	VECTOR_MANIP_INSTRUCTIONS
	// for vext.vv, load beginning cluster select index using vindex
	else if ((newInstr_store & lsu_vext_vv) == 1'b1)
		laneSelCount <= vindex;
`endif
	// move to next lane if can load the lane data to the alignment register
	else if ((lanesideState == LoadDataToIntReg) && ~d_waitrequest)
		laneSelCount <= laneSelCount + laneSelCount_inc;
end

always @(*) begin
	if (lsu_flagstore == 1'b1)
		laneSelCount_inc = FLAGSTORE_GRANULARITY;
	else
		laneSelCount_inc = MEM_WIDTH/VPU_WIDTH;
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Decode lane select signal to read from lane store Q
// - MEM_WIDTH/VPU_WIDTH lanes (elements) will be read at a time
// - if the vector length is not a multiple of (MEM_WIDTH/VPU_WIDTH), the last lane(s)
// may not actually have data to be read, but that doesn't cause any problems
// --------------------------------------------------------------------------------------------------------------------------------------------------
integer j;
reg	[`log2(NUMLANES*`ELEMPERLANE):0]	laneSelCountTemp;

always @(*)
begin
	// default assignments
	i = 0;
	j=0;
	laneSelCountTemp = 0;
	laneStoreQRdreq = {NUMLANES{1'b0}};
	
	if ((lanesideState == LoadDataToIntReg) && ~d_waitrequest) begin
		// wraps around the MSB of laneStoreQRdreq if need to read from lanes with highest and lowest indices
		for (j=0; j<=MEM_WIDTH/VPU_WIDTH-1; j=j+1) begin
			laneSelCountTemp = (laneSelCount + j) % NUMLANES;
			laneStoreQRdreq[laneSelCountTemp] = 1'b1;
		end			
		
	end
end

// read from flag store queues
always @(*) begin
	flagStoreQRdreq = 1'b0;
	if ((lanesideState == LoadDataToIntReg) && ~d_waitrequest) begin
		// if flagstore, read from all lanes
		// ASSUMES (NUMLANES < MEM_WIDTH), or if not, that MEM_WIDTH is multiple of NUMLANES
		if (lsu_flagstore == 1'b1)
			flagStoreQRdreq = 1'b1;
	end
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Write from vector lanes to intermediate register state machine
// --------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge clk, negedge reset_n)
begin
	if (~reset_n)
		lanesideState <= Idle;
	else if (~d_waitrequest)
		lanesideState <= lanesideNextState;
end

// readside state transition logic
always @ (*)
begin
	case (lanesideState)
		Idle:
			// start loading from vector lane store queue when it is not empty
			if (VLaneStoreQReady)
				lanesideNextState <= LoadDataToIntReg;
			else
				lanesideNextState <= Idle;
		
		// writing to intermediate register
		LoadDataToIntReg:
			// done when all data has been read
			if (laneSelCount >= totalNumElem_minus_one)
				lanesideNextState <= Idle;
			// unit stride, flagstore, vext.vv handled differently
			// these instructions never has to wait for write side because each cycle write side will have written out all data stored in intermediate registers
			// flagstore, vextvv, vins.vv all raise the unitstride signal, so they are handled
			else if ((laneDataLoadCount == 0) && ~lsu_unitstride)
				lanesideNextState <= IntRegWait;
			// otherwise keep writing
			else
				lanesideNextState <= LoadDataToIntReg;
		
		// waiting for write side to complete writing
		IntRegWait:
			if (loadNextLaneDataGroup == 1'b1)
				lanesideNextState <= LoadDataToIntReg;
			else
				lanesideNextState <= IntRegWait;

		// should never get here, but added to avoid a latch
		default:	lanesideNextState <= Idle;

	endcase
end


// State machine output
always @(*) begin
	// default assignments
	laneDataLoadCount_cnt = 1'b0;
	
	case (lanesideState)
		Idle:
			;
		// writing to intermediate register
		LoadDataToIntReg:
			laneDataLoadCount_cnt = 1'b1;
		
		// waiting for write side to complete writing
		IntRegWait:
			;
		default:	;
	endcase
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// INTERMEDIATE REGISTER SEPARATES LOGIC HERE
// - logic after this point control loading data into intermediate register, which is 1 cycle after the above logic to control
// reading data from lanes
// --------------------------------------------------------------------------------------------------------------------------------------------------

ff	#(.WIDTH(2))	ff_lanesideLastState (.clk(clk), .en(~d_waitrequest), .clrn(reset_n), .d( lanesideState ), .q(lanesideLastState));

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Generate enable signals to store data from vector lane MUX to intermediate register
// take into account the different data widths
// --------------------------------------------------------------------------------------------------------------------------------------------------
// write enable to select for flag store
assign	alignedDataReg_en = (lsu_flagstore == 1'b1) ? alignedDataReg_flagwr_bit_en : alignedDataReg_datawr_bit_en;


// Replicate from subword level to bit level enables
always @(*) begin
	for (i=0; i<MEM_WIDTH; i=i+1) begin
		alignedDataReg_datawr_bit_en [i] = alignedDataReg_datawr_subword_en [i / MINDATAWIDTH];
		alignedDataReg_flagwr_bit_en [i] = alignedDataReg_flagwr_en [i / FLAGSTORE_GRANULARITY];
	end
end


// Generate NUMLANE level enables for flag store
always @(*) begin
	i = 0;
	alignedDataReg_flagwr_en = {(MEM_WIDTH/FLAGSTORE_GRANULARITY) {1'b0}};

	if ((lanesideLastState == LoadDataToIntReg) && ~0)begin
		// for flag store instruction, only need to load into alignedDataReg
		// ASSUMES (NUMLANES < MEM_WIDTH), or if not, that MEM_WIDTH is multiple of NUMLANES
		for (i=0; i< MEM_WIDTH/FLAGSTORE_GRANULARITY; i=i+1)
			// if ((i >= ((clusterSelCount_delay) % (MEM_WIDTH/FLAGSTORE_GRANULARITY)))
				// && (i <= (((clusterSelCount_delay+1) - 1) % (MEM_WIDTH/FLAGSTORE_GRANULARITY)))) begin
			
			if (i == clusterSelCount_delay) begin			
				alignedDataReg_flagwr_en[i] = 1'b1;
			end
	end
end


// Generate subword level enables for data store
// Generate enable signals to store enable flags for memory store
always @(*) begin
	i = 0;
	alignedDataReg_datawr_subword_en = {MAX_NSUBWORDS{1'b0}};
	alignedFlagReg_en = {MAX_NSUBWORDS{1'b0}};

	case (lsu_memDataWidth)

`ifdef	STORE_WORD_ACCESS_ENABLED
	`MEMACCESS_WORD	: begin
		if ((lanesideLastState == LoadDataToIntReg) && ~d_waitrequest)begin
			for (i=0; i< MAX_NSUBWORDS; i=i+1)
				// calculate which subwords should be written
				// begin at: count * number of subwords per element
				// enable the next: MAX_ELEM_PER_CYCLE * number of subwords per element
				if ((i >= (((clusterSelCount_delay * MAX_ELEM_PER_CYCLE) * 32/MINDATAWIDTH) % MAX_NSUBWORDS))
					&& (i <= (((((clusterSelCount_delay+1)*MAX_ELEM_PER_CYCLE) * 32/MINDATAWIDTH) - 1) % MAX_NSUBWORDS))) begin
					
					alignedDataReg_datawr_subword_en[i] = 1'b1;
					alignedFlagReg_en[i] = 1'b1;
				end
		end
	end
`endif

`ifdef	STORE_HALFWORD_ACCESS_ENABLED
	`MEMACCESS_HALFWORD	: begin
		if ((lanesideLastState == LoadDataToIntReg) && ~d_waitrequest)begin
			for (i=0; i< MAX_NSUBWORDS; i=i+1)
				// calculate which subwords should be written
				// begin at: count * number of subwords per element
				// enable the next: MAX_ELEM_PER_CYCLE * number of subwords per element
				if ((i >= (((clusterSelCount_delay * MAX_ELEM_PER_CYCLE) * 16/MINDATAWIDTH) % MAX_NSUBWORDS))
					&& (i <= (((((clusterSelCount_delay+1)*MAX_ELEM_PER_CYCLE) * 16/MINDATAWIDTH) - 1) % MAX_NSUBWORDS))) begin
					alignedDataReg_datawr_subword_en[i] = 1'b1;
					alignedFlagReg_en[i] = 1'b1;
				end
		end
	end
`endif

`ifdef	STORE_BYTE_ACCESS_ENABLED
	`MEMACCESS_BYTE	: begin
		if ((lanesideLastState == LoadDataToIntReg) && ~d_waitrequest)begin
			for (i=0; i< MAX_NSUBWORDS; i=i+1)
				// calculate which subwords should be written
				// begin at: count * number of subwords per element
				// enable the next: MAX_ELEM_PER_CYCLE * number of subwords per element
				if ((i >= (((clusterSelCount_delay * MAX_ELEM_PER_CYCLE) * 8/MINDATAWIDTH) % MAX_NSUBWORDS))
					&& (i <= (((((clusterSelCount_delay+1)*MAX_ELEM_PER_CYCLE) * 8/MINDATAWIDTH) - 1) % MAX_NSUBWORDS))) begin
					alignedDataReg_datawr_subword_en[i] = 1'b1;
					alignedFlagReg_en[i] = 1'b1;
				end
		end
	end
`endif
	default:	;
	endcase
end



//---------------------------------------------------------------------------------------------------------------------------------------------------
// Mask out flags for data elements that are not written due to index exceeding vector length
//---------------------------------------------------------------------------------------------------------------------------------------------------	
always @(*) begin
	i = 0;
	alignedFlagReg_clr = {MAX_NSUBWORDS{1'b0}};
	
	// clear registers on beginning of store instruction (really for flag store so remaining bits that are written will be zero)
	if (newInstr_store == 1'b1)
		alignedFlagReg_clr = {MAX_NSUBWORDS{1'b1}};

	// if last cycle to transfer data from lanes
	// the comparison is delayed 1 cycle due to using lanesideLastState, need to use laneSelCount > totalNumElem_minus_one (strictly greater than)
	else if ((lanesideLastState == LoadDataToIntReg) && (laneSelCount > totalNumElem_minus_one) && ~d_waitrequest) begin
	// to guarantee it is indeed the last cycle of loading into alignedReg
		// do not clear the elements that need to be written
		// (vector length % number of elements transferred per cluster)

		case (lsu_memDataWidth)
`ifdef	STORE_WORD_ACCESS_ENABLED
			`MEMACCESS_WORD	:
				for (i=0; i< MAX_NSUBWORDS; i=i+1) begin
					if (i >= ((lastElemMemgroupPos+1) * 32/MINDATAWIDTH))
						alignedFlagReg_clr[i] = 1'b1;
				end
`endif

`ifdef	STORE_HALFWORD_ACCESS_ENABLED
			`MEMACCESS_HALFWORD	:
				for (i=0; i< MAX_NSUBWORDS; i=i+1) begin
					if (i >= ((lastElemMemgroupPos+1) * 16/MINDATAWIDTH))
						alignedFlagReg_clr[i] = 1'b1;
				end
`endif
`ifdef	STORE_BYTE_ACCESS_ENABLED
			`MEMACCESS_BYTE	:
				for (i=0; i< MAX_NSUBWORDS; i=i+1) begin
					if (i >= ((lastElemMemgroupPos+1) * 8/MINDATAWIDTH))
						alignedFlagReg_clr[i] = 1'b1;
				end
`endif
			default	:	;
		endcase
	end
	
	else
		// do nothing
		;
end


endmodule
