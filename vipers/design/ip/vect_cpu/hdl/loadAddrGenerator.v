//************************************************************************
// Scalable Vector CPU Project -- Memory load address generator
// Filename: loadAddrGenerator.v
// Author: Jason Yu
//
// Memory load address generator
// - generates addresses to the memory, and controls the alignment read crossbar
// - loadElemCount counts how many data elements have been loaded
// - numActiveLanes calculates how many lanes are active to read loaded data every cycle,
// taking into account offset, stride, and memory width
// - together, loadElemCount and numactivelanes are used to generate the write enable
// signals to the lanes
//
// **** NOTE ****
// - Modelsim doesn't simulate the signal vector_load_inc_rdaddr properly when
// optimizations are turned on.  Turn off all optimizations for this file when simulating.
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


module loadAddrGenerator
#(	
	// Parameters
	parameter	VPU_WIDTH = 1,
	parameter	NUMLANES = 1,
	parameter	MEM_WIDTH = 1,
	parameter	MINDATAWIDTH = 1,
	
	parameter	NELEMXFER_ROM_FILE = "NElemXfer_rom.mif",
	parameter	NELEMREMAINDER_ROM_FILE = "NElemRemainder_rom.mif",

	// Local parameters
	parameter	MAX_NUMELEMS = NUMLANES * `ELEMPERLANE,	// number of total data elements
	parameter	MAX_NSUBWORDS = MEM_WIDTH/MINDATAWIDTH,							// number of subwords
	parameter	READMUX_NSEL = `log2(MEM_WIDTH/MINDATAWIDTH)					// number of select line on each MUX of read crossbar
)

// Ports
(
	// **** Inputs ****
	input							clk,
	input							reset_n,
	input							lsu_flagload,
	input							lsu_vext_vv,					// vector-vector extract instruction
	input							lsu_vins_vv,
	input		[31:0]				baseAddr,
	input							lsu_executing_vector_load,
	input		[31:0]				lsu_vectorlength_minus_one,
	input							s3_unitstride,					// whether it's unit stride; overrides stride
	input		[`MAXSTRIDE_NBIT-1:0]	stride,						// signed stride, in terms of number of elements
	input		[31:0]				vindex,							// control register, vindex
	input		[2:0]				s3_memDataWidth,				// width of data access from memory
	input							start_newLoad,						// indicates a new memory instruction
	input							vectorload_waitrequest,			// load data from memory is now valid, otherwise stall
	input							lsu_indexed_memaccess,			// indexed memory load
	input							loadDataQ_full,					// the lane load data queue is full
	input							vector_manip_loading,			// currently in the load phase of vector manipulation
	input							lsu_scalar_load,				// scalar memory access
	input		[VPU_WIDTH-1:0]	indexOffset_out,				// offset from lanes for indexed addressing
	// Inputs from scalar processor
	input		[31:0]				scalar_d_address,

	// **** Outputs ****
	output 							d_read,							// read from data memory
	output		[31:0]				memAddr,						// address to memory
	output 		[READMUX_NSEL*MAX_NSUBWORDS-1:0]	readXbarSel,	// select to memory crossbar
	output reg	[NUMLANES-1:0]				data_lane_we,			// which lane should be written to.
	output		[`log2(MAX_NUMELEMS):0]		loadElemCount_preg,		// counter to cycle through all lanes; need 1 additional bit; pipeline delayed
	output		[`log2(NUMLANES):0]		numActiveLanes_preg,
	output		[`log2(NUMLANES)-1:0]		lsu_indexOffset_lanesel,	// select which lane to get index offset from
	// output							load_done_posedge					// asserted during last cycle of memory transfer
	output							load_done
);

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Local parameters
// --------------------------------------------------------------------------------------------------------------------------------------------------
localparam	MEMWIDTH_BYTES = MEM_WIDTH/8;						// memory width in number of bytes
	
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
wire	[31:0]						baseAddrIndexAdded;			// base address with added index (index is zero'd for non-index accesses)
wire	[31:0]						baseAddrScalarSel;			// base address after scalar select MUX
wire	[31:0]						baseAligned;				// base address aligned to 128 bit
wire	[31:0]						baseAligned_scalar;			// base address for scalar/indexed loads
wire	[`MAXSTRIDE_NBIT+2:0]		localStride;				// local stride signal
wire	[`MAXSTRIDE_NBIT+2:0]		strideByte;					// stride in number of bytes
wire	[`log2(MAX_NSUBWORDS)-1:0]	strideTrunc;				// stride truncated to only the lowest bits needed to address the number of subwords
wire	[31:0]						nextMemIncByteAligned;		// increment to the current memory read address
wire								strideIsZero;				// Handle zero stride
reg		[`log2(MEMWIDTH_BYTES)-1:0]	baseOffsetElem;				// number of element offsets; direct from baseAddr, not after adding Index
reg		[`log2(MEMWIDTH_BYTES)-1:0]	offsetByte;
wire	[`log2(MEMWIDTH_BYTES)-1:0]	offsetByte_scalar;
reg		[31:0]						nextMemAddr;				// next memory read address
wire								largeStride;				// whether strideByte > 16
reg		[`MAXSTRIDE_NBIT+2:0]		nextOffsetByteLargeStride;	// important to keep this bit width
reg		[`log2(MAX_NUMELEMS):0]		loadElemCount;				// counter to cycle through all lanes; need 1 additional bit
reg		[`log2(MAX_NUMELEMS):0]		loadElemCount_reverse;		// duplicate loadElemCount but count down to simply logic and improve Fmax
wire	[VPU_WIDTH-1:0]			indexOffset;				// current index offset
reg		[1:0]						convElemByteShiftConst;		// Number of positions to shift to convert from units of elements to bytes
reg		[VPU_WIDTH-1:0]			firstElemOffset;			// element offset position of first valid element in the transferred memory block; need full width to calculate whether to skip memory locations
wire	[VPU_WIDTH-1:0]			nextMemIncByteLargestride;	// number of bytes to increase read address by for largeStride read
wire	[31:0]						indexOffset_sext;			// sign extend indexed offset if VPU_WIDTH is less than 32
reg		[`log2(MEMWIDTH_BYTES):0]	offsetElemToSubtract;		// Calculate maximum number of data elements than can be transferred this cycle; numerator for division
reg		[`log2(MEMWIDTH_BYTES):0]	maxNElemXfer_offsetted;		// maximum number of elements that can be transferred this cycle, given offset
reg		[`log2(MEMWIDTH_BYTES):0]	maxNElemXfer_const;			// maximum number of elements transferred in a single memory cycle
wire	[`log2(MEMWIDTH_BYTES):0]	s4_NElemRemainder;
wire	[`log2(MEMWIDTH_BYTES):0]	s4_NElemXfer_rounded;		// number of data elements to tranasfer this cycle
reg		[`log2(NUMLANES):0]		numActiveLanes;				// how many lanes are active for this block of data; up to 8 for 16b unit stride; needs 1 extra bit
reg									lastLoadCycle;
wire								lastLoadCycle_delay;
wire								firstLoadCycle;
wire	[2*(`log2(MEMWIDTH_BYTES)+1)-1:0]	divider_rom_addr;
reg		[READMUX_NSEL*MAX_NSUBWORDS-1:0]	readXbarSel_sig;	// select to memory crossbar
reg		[READMUX_NSEL-1:0]			tempsel;
reg		[READMUX_NSEL-1:0]			tempinc;

wire	pauseLoadController;									// pause the load controller
wire	pauseLoadController_pipeline;
reg		vector_d_read;
// wire	load_done;						// asserted during last cycle of memory transfer
wire	load_done_posedge;
wire	load_done_delayn;
wire	vector_manip_loading_delay;
reg		vector_load_inc_rdaddr;									// increment read address to next block for regular vector read (not scalar or flag load)

integer	i, j, k;		// loop variables

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
// // Memory interface signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
// load_done needs to go low immediately for zero stride load to work properly
assign	load_done = ((loadElemCount_reverse == 0) && ~start_newLoad) ? 1'b1 : 1'b0;	// finished memory instruction
assign	load_done_posedge = load_done & load_done_delayn;

// pause the controller if eiher the vector lane data queue is full, or need to wait for memory
assign	pauseLoadController = loadDataQ_full | vectorload_waitrequest;

// Memory address; indexed memory access does not use registered address signal due to extra cycle latency
assign 	memAddr = ((lsu_scalar_load == 1'b1) || (lsu_indexed_memaccess == 1'b1)) ? baseAligned_scalar : nextMemAddr;
assign 	nextMemIncByteAligned = (largeStride) ? nextMemIncByteLargestride : MEMWIDTH_BYTES;

// define "largestride" as strideByte > 16; largestride requires skipping memory locations
assign 	largeStride = (strideByte > MEMWIDTH_BYTES) ? 1'b1 : 1'b0;
assign	nextMemIncByteLargestride = (nextOffsetByteLargeStride / MEMWIDTH_BYTES) << `log2(MEMWIDTH_BYTES);		// calculate how much to advance memory address

assign	d_read = vector_d_read | lsu_scalar_load;

ff	#(.WIDTH(1))	delayn_load_done_posedge (.clk(clk), .en(1'b1), .clrn(reset_n), .d(~load_done), .q(load_done_delayn));
ff	#(.WIDTH(1))	ff_firstLoadCycle (.clk(clk), .en(1'b1), .clrn(reset_n), .d(start_newLoad), .q(firstLoadCycle));
ff	#(.WIDTH(1))	ff_lastLoadCycle_delay (.clk(clk), .en(1'b1), .clrn(reset_n), .d(lastLoadCycle), .q(lastLoadCycle_delay));
ff	#(.WIDTH(1))	ff_vector_manip_loading (.clk(clk), .en(1'b1), .clrn(reset_n), .d(vector_manip_loading), .q(vector_manip_loading_delay));
ff	#(.WIDTH(1))	ff_pauseLoadController (.clk(clk), .en(1'b1), .clrn(reset_n), .d(pauseLoadController), .q(pauseLoadController_pipeline));

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Strided memory load signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Memory offset signals
assign 	baseAligned = baseAddr & ~{32'h00000000, {`log2(MEMWIDTH_BYTES){1'b1}}};			// truncate to MEMWIDTH aligned address

// Memory stride signals
assign 	localStride = (s3_unitstride) ? 1 : stride;					// flag load also sets unitstride
assign 	strideTrunc = localStride % MAX_NSUBWORDS;					// stride truncated to only the lowest bits needed to address the number of subwords
assign	strideIsZero = (localStride == 0) ? 1'b1 : 1'b0;			// Handle zero-stride
assign	strideByte = localStride << convElemByteShiftConst;			// convert stride in elements to stride in bytes

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Indexed/scalar memory load bypass signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
`ifdef	INDEXED_MEMORY_ACCESS
// sign extend index offset if it is less than 32
assign	indexOffset_sext = {{(32-VPU_WIDTH){indexOffset_out[VPU_WIDTH-1]}}, indexOffset_out};
assign	lsu_indexOffset_lanesel = (start_newLoad == 1'b1) ? 0 : loadElemCount % NUMLANES;		// set to zero on first cycle for processor stalling
`else
assign	indexOffset_sext = 0;
assign	lsu_indexOffset_lanesel = 0;
`endif

// Zero indexOffset for non-indexed memory accesses
assign	indexOffset = (lsu_indexed_memaccess) ? indexOffset_sext : {VPU_WIDTH{1'b0}};
assign	baseAddrIndexAdded = baseAddr + indexOffset;								// indexed address

// Scalar processor override
assign	baseAddrScalarSel = (lsu_scalar_load == 1'b1) ? scalar_d_address : baseAddrIndexAdded;
assign 	baseAligned_scalar = baseAddrScalarSel & ~{32'h00000000, {`log2(MEMWIDTH_BYTES){1'b1}}};			// truncate to MEMWIDTH aligned address
assign 	offsetByte_scalar = baseAddrScalarSel % MEMWIDTH_BYTES;							// offset from 128 bit alignment in bytes

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Strided memory load signals
// --------------------------------------------------------------------------------------------------------------------------------------------------
// offset from 128 bit alignment in bytes
always @(*) begin
`ifdef	VECTOR_MANIP_INSTRUCTIONS
	if (lsu_vins_vv == 1'b1)
		offsetByte = 0;
	// vext.vv sets offset to be vindex that we're extracting from
	else if (lsu_vext_vv == 1'b1)
		offsetByte = (vindex % (MEM_WIDTH/VPU_WIDTH)) << convElemByteShiftConst;
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
		baseOffsetElem = vindex % (MEM_WIDTH/VPU_WIDTH);
	// for all other access
	else
`endif
		baseOffsetElem = (baseAddr % MEMWIDTH_BYTES) >> convElemByteShiftConst;
end


//---------------------------------------------------------------------------------------------------------------------------------------------------
// maximum number of elements transferred in a single memory cycle
//---------------------------------------------------------------------------------------------------------------------------------------------------
always @(*) begin
	case (s3_memDataWidth)
`ifdef	LOAD_WORD_ACCESS_ENABLED
		// `MEMACCESS_WORD	:		maxNElemXfer_const = `min(MEMWIDTH_BYTES >> 2, LOCAL_NUMLANES);	// divide by 32
		`MEMACCESS_WORD	:		maxNElemXfer_const = MEMWIDTH_BYTES >> 2;	// divide by 32
`endif
`ifdef	LOAD_HALFWORD_ACCESS_ENABLED
		// `MEMACCESS_HALFWORD	:	maxNElemXfer_const = `min(MEMWIDTH_BYTES >> 1, LOCAL_NUMLANES);	// divide by 16
		`MEMACCESS_HALFWORD	:	maxNElemXfer_const = MEMWIDTH_BYTES >> 1;	// divide by 16
`endif
`ifdef	LOAD_BYTE_ACCESS_ENABLED
		// `MEMACCESS_BYTE	:		maxNElemXfer_const = `min(MEMWIDTH_BYTES, LOCAL_NUMLANES);		// divide by 8
		`MEMACCESS_BYTE	:		maxNElemXfer_const = MEMWIDTH_BYTES;		// divide by 8
`endif
		default	:				maxNElemXfer_const = 'bx;	// should not get here
	endcase
end

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Request to read from and write to memory memory
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Control when to increment the memory address
always @(*) begin
	// default assignment for normal loads
	vector_load_inc_rdaddr = 1'b1;

	// special case: if number of lanes is less than maximum number of subwords
	// if (NUMLANES < (MEM_WIDTH/MINDATAWIDTH)) begin
	if ((NUMLANES < (MEM_WIDTH/MINDATAWIDTH)) && (NUMLANES*localStride < maxNElemXfer_const)) begin
	
		// if for this access, max number ofsubwords > number of lanes
		// if (NUMLANES*localStride < maxNElemXfer_const) begin
			case (s3_memDataWidth)
`ifdef	LOAD_WORD_ACCESS_ENABLED
			`MEMACCESS_WORD	:
				if ((baseOffsetElem + loadElemCount + numActiveLanes) % (MEMWIDTH_BYTES >> 2) == 0)
					vector_load_inc_rdaddr = 1'b1;
				else
					vector_load_inc_rdaddr = 1'b0;
`endif
`ifdef	LOAD_HALFWORD_ACCESS_ENABLED
			`MEMACCESS_HALFWORD	:
				if ((baseOffsetElem + loadElemCount + numActiveLanes) % (MEMWIDTH_BYTES >> 1) == 0)
					vector_load_inc_rdaddr = 1'b1;
				else	
					vector_load_inc_rdaddr = 1'b0;
`endif
`ifdef	LOAD_BYTE_ACCESS_ENABLED
			`MEMACCESS_BYTE	:
				if ((baseOffsetElem + loadElemCount + numActiveLanes) % (MEMWIDTH_BYTES) == 0)
					vector_load_inc_rdaddr = 1'b1;
				else
					vector_load_inc_rdaddr = 1'b0;
`endif
			default	:	vector_load_inc_rdaddr = 1'b0;	// should not get here
			endcase
		// end
		
		// normal access, max number of subwords <= number of lanes
		// else
			// vector_load_inc_rdaddr = 1'b1;
	end
	
	// number of lanes is more than or equal to number of subwords
	// else
		// vector_load_inc_rdaddr = 1'b1;
end


// Request to read from memory
// First memory address is available the cycle after lsu_newInstr_load is asserted
// load_done is asserted when last memory address is put on rdaddress
always @(posedge clk, negedge reset_n, posedge load_done_posedge) begin
	if (~reset_n || load_done_posedge)
		vector_d_read <= 1'b0;
	else if (clk == 1'b1)
		if (lsu_executing_vector_load && ~vector_load_inc_rdaddr && ~start_newLoad)
			vector_d_read <= 1'b0;
		else if ((start_newLoad & ~lsu_vext_vv & ~lsu_vins_vv) || (lsu_executing_vector_load && vector_load_inc_rdaddr))
			vector_d_read <= 1'b1;
		else
			vector_d_read <= vector_d_read;
end
	

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Keep track of offset of current element; needed for skipping memory locations for stride > 8 (16b mode), or stride > 4 (32b mode)
// - only used for largeStride transfers
// - only need to keep track of 1 offset since for these strides there can't be more than 1 valid element per 128b block
// --------------------------------------------------------------------------------------------------------------------------------------------------
always @(posedge clk) begin
	if (start_newLoad)
		nextOffsetByteLargeStride <= offsetByte + strideByte;
	else if (clk == 1'b1 && lsu_executing_vector_load && ~pauseLoadController)
		nextOffsetByteLargeStride <= (nextOffsetByteLargeStride % MEMWIDTH_BYTES) + strideByte;
end

	
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Select next memory address
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Set next read memory location
always @(posedge clk) begin
	// Pause if pauseLoadController
	// if (clk == 1'b1 && lsu_executing_vector_load && ~pauseLoadController) begin
	if (lsu_executing_vector_load && ~pauseLoadController) begin
		// Use baseAligned + index offset for index accesses
		if (start_newLoad) begin
			nextMemAddr <= baseAligned;
			// $display ("Resetting memory address: time=%d, nextMemAddr=%x, vector_load_inc_rdaddr=%d", $time, nextMemAddr, vector_load_inc_rdaddr);
		end
		// if lsu_flagload and we have read all MEM_WIDTH bits, advance to next address
		else if ((lsu_flagload && ((loadElemCount % MEM_WIDTH) == 0) && (loadElemCount != 0))
				// for regular loads, only advance address if the current block has been completely read
				|| (~lsu_flagload && vector_load_inc_rdaddr)) begin
			// $display ("Incrementing memory address: time=%d, nextMemAddr=%x, nextMemIncByteAligned=%x, vector_load_inc_rdaddr=%d", $time, nextMemAddr, nextMemIncByteAligned, vector_load_inc_rdaddr);
			nextMemAddr <= nextMemAddr + nextMemIncByteAligned;
		end
		else begin
			nextMemAddr <= nextMemAddr;
		end
	end
	else
		nextMemAddr <= nextMemAddr;
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Determine which lanes are being written to
// --------------------------------------------------------------------------------------------------------------------------------------------------
// assign	maxNElemXfer_offsetted = maxNElemXfer_const - offsetElemToSubtract;

always @(*) begin
	// default assignment
	maxNElemXfer_offsetted = maxNElemXfer_const - offsetElemToSubtract;
	
	// if max number of subwords can potentially be greater than number of lanes
	if (NUMLANES < MEM_WIDTH/MINDATAWIDTH) begin
		// if for this access, max number ofsubwords > number of lanes; first cycle uses regular assignment
		if ((NUMLANES*localStride < maxNElemXfer_const) && ~start_newLoad) begin
			// the offset is stored directly in the ROM table
			if (s4_NElemRemainder == 0)
				// offsetElemToSubtract will be zero
				// simplies logic compared to writing "maxNElemXfer_offsetted = maxNElemXfer_const"
				maxNElemXfer_offsetted = maxNElemXfer_const - offsetElemToSubtract;
			else
				maxNElemXfer_offsetted = (s4_NElemRemainder % MEMWIDTH_BYTES);
		end
	end
end	

// Calculate maximum number of data elements than can be transferred this cycle; numerator for division
always @(*) begin
	// default
	offsetElemToSubtract = (firstElemOffset % MEMWIDTH_BYTES);

	// first cycle of lsu_vext_vv uses baseOffsetElem
	if (start_newLoad)
		offsetElemToSubtract = baseOffsetElem;
`ifdef	VECTOR_MANIP_INSTRUCTIONS
	// for all vins.vv, and from 2nd cycle of vext.vv
	else if (lsu_vext_vv | lsu_vins_vv)
		offsetElemToSubtract = 0;
`endif
	
	// normal operation
	else
		offsetElemToSubtract = (firstElemOffset % MEMWIDTH_BYTES);
end

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Calculate element offset position of first valid element in the transferred memory block
// --------------------------------------------------------------------------------------------------------------------------------------------------
always @(*) begin
	// special case: if number of lanes is less than maximum number of subwords
	if (NUMLANES < MEM_WIDTH/MINDATAWIDTH) begin
		if (s4_NElemRemainder == 0)
			firstElemOffset = 0;
		// if for this access, max number ofsubwords > number of lanes
		// if (NUMLANES < maxNElemXfer_const)
		else if (NUMLANES*localStride < maxNElemXfer_const)
			firstElemOffset = s4_NElemRemainder;
		else
			// firstElemOffset = (localStride - s4_NElemRemainder) % maxNElemXfer_const;
			// written this way to avoid a divider to do modulus
			case (s3_memDataWidth)
`ifdef	LOAD_WORD_ACCESS_ENABLED
			`MEMACCESS_WORD	:		firstElemOffset = (localStride - s4_NElemRemainder) % (MEMWIDTH_BYTES >> 2);	// divide by 32
`endif
`ifdef	LOAD_HALFWORD_ACCESS_ENABLED
			`MEMACCESS_HALFWORD	:	firstElemOffset = (localStride - s4_NElemRemainder) % (MEMWIDTH_BYTES >> 1);	// divide by 16
`endif
`ifdef	LOAD_BYTE_ACCESS_ENABLED
			`MEMACCESS_BYTE	:		firstElemOffset = (localStride - s4_NElemRemainder) % MEMWIDTH_BYTES;		// divide by 8
`endif
			default	:				firstElemOffset = 'bx;	// should not get here
			endcase
	end
	
	// hopefully either the last case or this case will be synthesized away
	else begin
		if (s4_NElemRemainder == 0)
			firstElemOffset = 0;
		else begin
		// firstElemOffset = (localStride - s4_NElemRemainder) % maxNElemXfer_const;
		// written this way to avoid a divider to do modulus
		case (s3_memDataWidth)
`ifdef	LOAD_WORD_ACCESS_ENABLED
		`MEMACCESS_WORD	:		firstElemOffset = (localStride - s4_NElemRemainder) % (MEMWIDTH_BYTES >> 2);	// divide by 32
`endif
`ifdef	LOAD_HALFWORD_ACCESS_ENABLED
		`MEMACCESS_HALFWORD	:	firstElemOffset = (localStride - s4_NElemRemainder) % (MEMWIDTH_BYTES >> 1);	// divide by 16
`endif
`ifdef	LOAD_BYTE_ACCESS_ENABLED
		`MEMACCESS_BYTE	:		firstElemOffset = (localStride - s4_NElemRemainder) % MEMWIDTH_BYTES;		// divide by 8
`endif
		default	:				firstElemOffset = 'bx;	// should not get here
		endcase
		end
	end
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// localstride range: 0 to 16 bytes (anything bigger is largestride and handled differently)
// maxElemXfer_offseted range: 1 to 16
// --------------------------------------------------------------------------------------------------------------------------------------------------
(* romstyle = "M9K" *)				// Quartus II synthesis attribute
reg		[`log2(MEMWIDTH_BYTES):0]			NElemXfer_sig_mem_q;
(* romstyle = "M9K" *)				// Quartus II synthesis attribute
reg		[`log2(MEMWIDTH_BYTES):0]			NElemRemainder_mem_q;

assign	divider_rom_addr = {maxNElemXfer_offsetted, localStride [`log2(MEMWIDTH_BYTES):0]};
// localstride range: 0 to 16 bytes (anything bigger is largestride and handled differently)
// maxElemXfer_offseted range: 1 to 16
assign	s4_NElemXfer_rounded = NElemXfer_sig_mem_q [`log2(MEMWIDTH_BYTES):0];
// ROM for remainder of division
assign	s4_NElemRemainder = NElemRemainder_mem_q [`log2(MEMWIDTH_BYTES):0];


// Automatically infer ROM from HDL
always @ (posedge clk) begin
	// clken
	if ((lsu_executing_vector_load || lsu_vext_vv || lsu_vins_vv) && ~pauseLoadController) begin
		// one large case statement
		// ROM address
		case (divider_rom_addr)
		// ROM contents automatically generated by configuration script, and included here
`ifdef	MODELSIM
		`include "../hdl/NElemXfer_rom_init.v"
`else
		`include "NElemXfer_rom_init.v"
`endif
		default:	NElemXfer_sig_mem_q <= 'bx;
		endcase
	end
end

// ROM for quotient of division
// altsyncram	NElemXfer_sig_mem (
	// .clock0 			(clk),
	// .address_a 			(divider_rom_addr),
	// .q_a 				(NElemXfer_sig_mem_q),
	// .aclr0 				(1'b0),
	// .aclr1 				(1'b0),
	// .address_b 			(1'b1),
	// .addressstall_a 	(1'b0),
	// .addressstall_b		(1'b0),
	// .byteena_a 			(1'b1),
	// .byteena_b 			(1'b1),
	// .clock1 			(1'b1),
	// .clocken0 			(lsu_executing_vector_load | lsu_vext_vv | lsu_vins_vv),
	// .clocken1 			(1'b1),
	// .clocken2 			(1'b1),
	// .clocken3 			(1'b1),
	// .data_a 			({`log2(MEMWIDTH_BYTES)+1{1'b1}}),
	// .data_b 			(1'b1),
	// .eccstatus 			(),
	// .q_b 				(),
	// .rden_a 			(1'b1),
	// .rden_b 			(1'b1),
	// .wren_a 			(1'b0),
	// .wren_b 			(1'b0));
// defparam
	// NElemXfer_sig_mem.address_aclr_a = "NONE",
	// NElemXfer_sig_mem.clock_enable_input_a = "NORMAL",
	// NElemXfer_sig_mem.clock_enable_output_a = "BYPASS",
	// NElemXfer_sig_mem.init_file = NELEMXFER_ROM_FILE,
	// NElemXfer_sig_mem.intended_device_family = "Stratix III",
	// NElemXfer_sig_mem.lpm_hint = "ENABLE_RUNTIME_MOD=NO",
	// NElemXfer_sig_mem.lpm_type = "altsyncram",
	// NElemXfer_sig_mem.numwords_a = 2**(2*`log2(MEMWIDTH_BYTES)+2),
	// NElemXfer_sig_mem.operation_mode = "ROM",
	// NElemXfer_sig_mem.outdata_aclr_a = "NONE",
	// NElemXfer_sig_mem.outdata_reg_a = "UNREGISTERED",
	// NElemXfer_sig_mem.widthad_a = 2*`log2(MEMWIDTH_BYTES)+2,
	// NElemXfer_sig_mem.width_a = `log2(MEMWIDTH_BYTES)+1,
	// NElemXfer_sig_mem.width_byteena_a = 1;



// ROM for remainder of division
// Automatically infer ROM from HDL
always @ (posedge clk) begin
	// clken
	if ((lsu_executing_vector_load || lsu_vext_vv || lsu_vins_vv) && ~pauseLoadController) begin
		// one large case statement
		// ROM address
		case (divider_rom_addr)
		// ROM contents automatically generated by configuration script, and included here
`ifdef	MODELSIM
		`include "../hdl/NElemRemainder_rom_init.v"
`else
		`include "NElemRemainder_rom_init.v"
`endif
		default:	NElemRemainder_mem_q <= 'bx;
		endcase
	end
end


// altsyncram	NElemRemainder_mem (
	// .clock0 			(clk),
	// .address_a 			(divider_rom_addr),
	// .q_a 				(NElemRemainder_mem_q),
	// .aclr0 				(1'b0),
	// .aclr1 				(1'b0),
	// .address_b 			(1'b1),
	// .addressstall_a		(1'b0),
	// .addressstall_b 	(1'b0),
	// .byteena_a 			(1'b1),
	// .byteena_b 			(1'b1),
	// .clock1 			(1'b1),
	// .clocken0 			(lsu_executing_vector_load | lsu_vext_vv | lsu_vins_vv),
	// .clocken1 			(1'b1),
	// .clocken2 			(1'b1),
	// .clocken3 			(1'b1),
	// .data_a 			({`log2(MEMWIDTH_BYTES)+1{1'b1}}),
	// .data_b 			(1'b1),
	// .eccstatus 			(),
	// .q_b 				(),
	// .rden_a 			(1'b1),
	// .rden_b 			(1'b1),
	// .wren_a 			(1'b0),
	// .wren_b 			(1'b0));
// defparam
	// NElemRemainder_mem.address_aclr_a = "NONE",
	// NElemRemainder_mem.clock_enable_input_a = "NORMAL",
	// NElemRemainder_mem.clock_enable_output_a = "BYPASS",
	// NElemRemainder_mem.init_file = NELEMREMAINDER_ROM_FILE,
	// NElemRemainder_mem.intended_device_family = "Stratix III",
	// NElemRemainder_mem.lpm_hint = "ENABLE_RUNTIME_MOD=NO",
	// NElemRemainder_mem.lpm_type = "altsyncram",
	// NElemRemainder_mem.numwords_a = 2**(2*`log2(MEMWIDTH_BYTES)+2),
	// NElemRemainder_mem.operation_mode = "ROM",
	// NElemRemainder_mem.outdata_aclr_a = "NONE",
	// NElemRemainder_mem.outdata_reg_a = "UNREGISTERED",
	// NElemRemainder_mem.widthad_a = 2*`log2(MEMWIDTH_BYTES)+2,
	// NElemRemainder_mem.width_a = `log2(MEMWIDTH_BYTES)+1,
	// NElemRemainder_mem.width_byteena_a = 1;

	
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Calculate how many lanes are being written to
// --------------------------------------------------------------------------------------------------------------------------------------------------
always @(*)
begin
	// default assignment
	lastLoadCycle = 1'b0;

	// flag load instruction
	if (lsu_flagload) begin
		// last transfer, take vectorlength into account
		if (loadElemCount_reverse <= `min(NUMLANES, MEM_WIDTH)) begin
			lastLoadCycle = 1'b1;
			numActiveLanes = loadElemCount_reverse;
		end
		// middle transfers, full speed
		else
			numActiveLanes = `min(NUMLANES, MEM_WIDTH);
	end	
	
	// Indexed memory load runs at 1 element per cycle
	// for large stride, there will only be 1 active element per 128-bit memory da
	else if (lsu_indexed_memaccess || largeStride)
		numActiveLanes = 1;
	
	// Zero stride, transfer as quickly as possible
	else if (strideIsZero) begin
		// handle vector length at the last transfer
		if (loadElemCount_reverse <= NUMLANES) begin
			lastLoadCycle = 1'b1;
			numActiveLanes = loadElemCount_reverse;
		end
		// middle transfers, full speed
		else
			numActiveLanes = NUMLANES;
	end
	
	// For all other strides between 1 and largeStride
	else begin
		// if offset at the last transfer
		if (loadElemCount_reverse <= s4_NElemXfer_rounded) begin
			lastLoadCycle = 1'b1;
			numActiveLanes = loadElemCount_reverse;
		end

`ifdef	VECTOR_MANIP_INSTRUCTIONS		
		// if offset at first transfer for vins.vv; last transfer takes priority over first transfer
		else if ((lsu_vins_vv == 1'b1) && (firstLoadCycle == 1'b1))
			numActiveLanes = s4_NElemXfer_rounded - (loadElemCount % (MEM_WIDTH/VPU_WIDTH));
`endif

		// middle transfers, full speed
		else
			// can run at LOCLA_NUMLANES transfers to lanes per cycle if it is zero stride
			numActiveLanes = `min(s4_NElemXfer_rounded, NUMLANES);
	end
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Counter to count how many elements have been written to
// Doubles to count through the lanes that need to be written to
// --------------------------------------------------------------------------------------------------------------------------------------------------
always @(posedge clk, negedge reset_n)
begin
	// Set to a value greater than maximum vector length so the first load instruction can start properly
	if (~reset_n)
		loadElemCount <= {`log2(MAX_NUMELEMS)+1{1'b1}};
	// Pause if pauseLoadController
	// enable if vector loading and not paused, or vins.vv/vext.vv and tranfer and queue not full (data will be lost if full?), or new load
	else if (clk == 1'b1 && 
			((lsu_executing_vector_load && ~pauseLoadController) || 
			((lsu_vext_vv || lsu_vins_vv) && vector_manip_loading && ~loadDataQ_full)
			|| start_newLoad))
		// start loading into vindex position for vins.vv
		if (start_newLoad == 1'b1 && lsu_vins_vv)
			loadElemCount <= vindex;
		else if (start_newLoad)
			loadElemCount <= 0;
		else
			loadElemCount <= loadElemCount + numActiveLanes;
end


// Create a counter that counts in reverse of loadElemCount to simply logic for numActiveLanes and improve Fmax
always @(posedge clk, negedge reset_n)
begin
	// Set to a value greater than maximum vector length so the first load instruction can start properly
	if (~reset_n)
		loadElemCount_reverse <= {`log2(MAX_NUMELEMS)+1{1'b0}};
	// enable if vector loading and not paused, or vins.vv/vext.vv and tranfer and queue not full (data will be lost if full?), or new load
	else if (clk == 1'b1 && 
			((lsu_executing_vector_load && ~pauseLoadController) || 
			((lsu_vext_vv || lsu_vins_vv) && vector_manip_loading && ~loadDataQ_full)
			|| start_newLoad))
		// start with the same count for both normal loads and vins.vv
		if (start_newLoad == 1'b1)
			loadElemCount_reverse <= lsu_vectorlength_minus_one+1;
		else
			loadElemCount_reverse <= loadElemCount_reverse - numActiveLanes;
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// PIPELINE REGISTERS
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Takes at least 2 cycle (1 to setup, 1 to read from memory) before writing to lanes (enable data_lane_we), so clear the value on the 2nd cycle
ff_sclr	#(.WIDTH(`log2(MAX_NUMELEMS)+1))
ff_elemCount
(	.clk(clk),
	.en(~pauseLoadController),
	.clrn(1'b1),
	.sclr(start_newLoad | (~lsu_executing_vector_load && ~lsu_vext_vv && ~lsu_vins_vv)),
	.d(loadElemCount),
	.q(loadElemCount_preg)
);

// Takes at least 2 cycle (1 to setup, 1 to read from memory) before writing to lanes (enable data_lane_we), so clear the value on the 2nd cycle
ff_sclr	#(.WIDTH(`log2(NUMLANES)+1))
ff_numActiveLanes
(	.clk(clk),
	.en(~pauseLoadController),
	.clrn(1'b1),
	.sclr(start_newLoad | (~lsu_executing_vector_load && ~lsu_vext_vv && ~lsu_vins_vv)),
	.d(numActiveLanes),
	.q(numActiveLanes_preg)
);


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Decoder to generate lane write signals
// - uses 
// --------------------------------------------------------------------------------------------------------------------------------------------------

always @(*)
begin
	data_lane_we = `ISA_MAXLANES'b0;
	k=0;
	
	// takes at least 2 cycles before we can start writing
	// 1st cycle masked by ~start_newLoad, 2nd cycle masked by setting loadElemCount_preg and numActiveLanes_preg both to zero
	// if 	((lsu_executing_vector_load && ~pauseLoadController)||
	// use pipelined pauseLoadController signal because data_lane_we is generated by pipelined signals (loadElemCount_preg, numActiveLanes_preg)
	if 	((lsu_executing_vector_load && ~pauseLoadController_pipeline)||
		// start writing 1 cycle after vector_manip_loading goes high, and make sure to stop writing when vector_manip_loading goes low
		((lsu_vext_vv || lsu_vins_vv) && (vector_manip_loading_delay) && ~loadDataQ_full))
		for (k=0; k<MAX_NUMELEMS; k=k+1)
			if ((k >= loadElemCount_preg) && (k < loadElemCount_preg + numActiveLanes_preg))
				data_lane_we[ k % NUMLANES ] = 1'b1;
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Load data alignment crossbar select lines
// --------------------------------------------------------------------------------------------------------------------------------------------------
// Delay the select lines 1 cycle
ff	#(.WIDTH(READMUX_NSEL*MAX_NSUBWORDS))	ff_readXbarSel (.clk(clk), .en(~pauseLoadController), .clrn(1'b1), .d(readXbarSel_sig), .q(readXbarSel));

// Combinational logic for crossbar select lines	
always @(*)
begin
	// default values
	tempinc = 'bx;
	tempsel = 'bx;
	readXbarSel_sig = {READMUX_NSEL*MAX_NSUBWORDS{1'bx}};
	i=0;
	j=0;
	k=0;
	
	// scalar load, generate select signal for 32/MINDATAWIDTH selects no matter what the scalar load width is
	if (lsu_scalar_load == 1'b1) begin		
		for (k=0; k<32/MINDATAWIDTH; k=k+1) begin
			// tempsel = ((offsetByte_scalar >> `log2(MINDATAWIDTH/8)) + k) % MAX_NSUBWORDS;
			// UTIIe expects returned data to be 32-bit aligned; mask least significant 2 bits of byte offset to return 32-bit aligned data
			tempsel = ((offsetByte_scalar & {{(`log2(MEMWIDTH_BYTES)-2){1'b1}},{2'b00}}) + k) % MAX_NSUBWORDS;

			// index j iterates through the READMUX_NSEL bits of the select line for the current subword
			for (j=k*READMUX_NSEL; j<=(1+k)*READMUX_NSEL-1; j=j+1)
				readXbarSel_sig [ j ] = tempsel[ j%READMUX_NSEL ];
		end
	end

	// vector load
	else begin
		case (s3_memDataWidth)

`ifdef	LOAD_WORD_ACCESS_ENABLED
		// 32 bit word operation
		`MEMACCESS_WORD: begin
`ifdef	INDEXED_MEMORY_ACCESS
			// indexed access
			if (lsu_indexed_memaccess) begin
				// iterate over the subwords for the current 32-bit word
				for (k=0; k<32/MINDATAWIDTH; k=k+1) begin
					// tempsel = (offsetByte_scalar + k) % MAX_NSUBWORDS;
					tempsel = ((offsetByte_scalar >> `log2(MINDATAWIDTH/8)) + k) % MAX_NSUBWORDS;
					
					// need to set the xbar selects for the current loadElemCount lane
					// index j iterates through the READMUX_NSEL bits of the select line for the current subword
					// for (j=(((loadElemCount*(32/MINDATAWIDTH)) % MAX_NSUBWORDS) + k)*READMUX_NSEL ;
						// j<=(((loadElemCount*(32/MINDATAWIDTH)) % MAX_NSUBWORDS) + k + 1)*READMUX_NSEL-1; j=j+1)
						// readXbarSel_sig [ j ] = tempsel[ j%READMUX_NSEL ];
						
					for (j=0; j<READMUX_NSEL*MAX_NSUBWORDS; j=j+1) begin
						if ((j>=(((loadElemCount*(32/MINDATAWIDTH)) % MAX_NSUBWORDS) + k)*READMUX_NSEL)
							&& (j<=(((loadElemCount*(32/MINDATAWIDTH)) % MAX_NSUBWORDS) + k + 1)*READMUX_NSEL-1))
						
							readXbarSel_sig [ j ] = tempsel[ j%READMUX_NSEL ];
					end
					
				end
			end
			
			// unit stride
			else if (s3_unitstride || lsu_flagload) begin
`else
			// unit stride
			if (s3_unitstride || lsu_flagload) begin
`endif

			
				// index i iterates through the 32-bit words in the MEMWIDTH memory block
				for (i=0; i<MEM_WIDTH/MINDATAWIDTH; i=i+32/MINDATAWIDTH) begin
					// index k iterates through the subwords that make up a 32-bit word
					for (k=0; k<32/MINDATAWIDTH; k=k+1) begin
						// tempsel = (offsetByte + i + k) % MAX_NSUBWORDS;
						tempsel = ((offsetByte >> `log2(MINDATAWIDTH/8)) + i + k) % MAX_NSUBWORDS;
					
						// index j iterates through the READMUX_NSEL bits of the select line for the current subword
						for (j=(i+k)*READMUX_NSEL; j<=(i+1+k)*READMUX_NSEL-1; j=j+1)
							readXbarSel_sig [ j ] = tempsel[ j%READMUX_NSEL ];
					end
				end
			end
			
			// strided access; also handles zero stride
			else begin
				// index i iterates through 32-bit words
				for (i=0; i<MEM_WIDTH/MINDATAWIDTH; i=i+32/MINDATAWIDTH) begin
					// calculate the increment in the number of subwords % max subwords
					tempinc = (i*strideTrunc) % MAX_NSUBWORDS;
				
					// index k iterates through the subwords that make up a 32-bit word
					for (k=0; k<32/MINDATAWIDTH; k=k+1) begin
						// tempsel = (offsetByte + tempinc + k) % MAX_NSUBWORDS;
						tempsel = ((offsetByte >> `log2(MINDATAWIDTH/8)) + tempinc + k) % MAX_NSUBWORDS;
					
						// index j (bit index) iterates through the READMUX_NSEL bits of the select line for the current subword
						for (j=(i+k)*READMUX_NSEL; j<=(i+1+k)*READMUX_NSEL-1; j=j+1)
							readXbarSel_sig [ j ] = tempsel[ j%READMUX_NSEL ];
					end

				end
			end
		end
`endif

`ifdef	LOAD_HALFWORD_ACCESS_ENABLED
		// 16 bit half word operation
		`MEMACCESS_HALFWORD: begin
			// indexed access
`ifdef	INDEXED_MEMORY_ACCESS
			if (lsu_indexed_memaccess) begin
				for (k=0; k<16/MINDATAWIDTH; k=k+1) begin
					// tempsel = (offsetByte_scalar + k) % MAX_NSUBWORDS;
					tempsel = ((offsetByte_scalar >> `log2(MINDATAWIDTH/8)) + k) % MAX_NSUBWORDS;
					
					// need to set the xbar selects for the current loadElemCount
					// index j iterates through the READMUX_NSEL bits of the select line for the current subword
					// for (j=(((loadElemCount*(16/MINDATAWIDTH)) % MAX_NSUBWORDS) + k)*READMUX_NSEL ;
						// j<=(((loadElemCount*(16/MINDATAWIDTH)) % MAX_NSUBWORDS) + k + 1)*READMUX_NSEL-1; j=j+1)
						// readXbarSel_sig [ j ] = tempsel[ j%READMUX_NSEL ];
						
					for (j=0; j<READMUX_NSEL*MAX_NSUBWORDS; j=j+1) begin
						if ((j>=(((loadElemCount*(16/MINDATAWIDTH)) % MAX_NSUBWORDS) + k)*READMUX_NSEL)
							&& (j<=(((loadElemCount*(16/MINDATAWIDTH)) % MAX_NSUBWORDS) + k + 1)*READMUX_NSEL-1))
						
							readXbarSel_sig [ j ] = tempsel[ j%READMUX_NSEL ];
					end
				end
			end

			// unit stride
			else if (s3_unitstride) begin
`else
			// unit stride
			if (s3_unitstride) begin
`endif
				// index i iterates through the 16-bit haflwords in the MEMWIDTH memory block
				for (i=0; i<MEM_WIDTH/MINDATAWIDTH; i=i+16/MINDATAWIDTH) begin
					// index k iterates through the subwords that make up a 16-bit halfword
					for (k=0; k<16/MINDATAWIDTH; k=k+1) begin
						// tempsel = (offsetByte + i + k) % MAX_NSUBWORDS;
						tempsel = ((offsetByte >> `log2(MINDATAWIDTH/8)) + i + k) % MAX_NSUBWORDS;
					
						// index j iterates through the READMUX_NSEL bits of the select line for the current subword
						for (j=(i+k)*READMUX_NSEL; j<=(i+1+k)*READMUX_NSEL-1; j=j+1)
							readXbarSel_sig [ j ] = tempsel[ j%READMUX_NSEL ];
					end
				end
				
			end
			
			// strided access; also handles zero stride
			// fixed xbar configuration for entire memory access!!
			else begin
				// index i iterates through 16-bit halfwords
				for (i=0; i<MEM_WIDTH/MINDATAWIDTH; i=i+16/MINDATAWIDTH) begin
					// calculate the increment in the number of subwords % max subwords
					tempinc = (i*strideTrunc) % MAX_NSUBWORDS;
				
					// index k iterates through the subwords that make up a 32-bit word
					for (k=0; k<16/MINDATAWIDTH; k=k+1) begin
						// tempsel = (offsetByte + tempinc + k) % MAX_NSUBWORDS;
						tempsel = ((offsetByte >> `log2(MINDATAWIDTH/8)) + tempinc + k) % MAX_NSUBWORDS;
					
						// index j (bit index) iterates through the READMUX_NSEL bits of the select line for the current subword
						for (j=(i+k)*READMUX_NSEL; j<=(i+1+k)*READMUX_NSEL-1; j=j+1)
							readXbarSel_sig [ j ] = tempsel[ j%READMUX_NSEL ];
					end
				end
				
			end
		end
`endif

`ifdef	LOAD_BYTE_ACCESS_ENABLED
		// 8 bit byte access
		`MEMACCESS_BYTE: begin
`ifdef	INDEXED_MEMORY_ACCESS
			// indexed access
			if (lsu_indexed_memaccess) begin
				for (k=0; k<8/MINDATAWIDTH; k=k+1) begin
					// tempsel = (offsetByte_scalar + k) % MAX_NSUBWORDS;
					tempsel = ((offsetByte_scalar >> `log2(MINDATAWIDTH/8)) + k) % MAX_NSUBWORDS;
					
					// need to set the xbar selects for the current loadElemCount
					// index j iterates through the READMUX_NSEL bits of the select line for the current subword
					// for (j=(((loadElemCount*(8/MINDATAWIDTH)) % MAX_NSUBWORDS) + k)*READMUX_NSEL ;
						// j<=(((loadElemCount*(8/MINDATAWIDTH)) % MAX_NSUBWORDS) + k + 1)*READMUX_NSEL-1; j=j+1)
						// readXbarSel_sig [ j ] = tempsel[ j%READMUX_NSEL ];
						
					for (j=0; j<READMUX_NSEL*MAX_NSUBWORDS; j=j+1) begin
						if ((j>=(((loadElemCount*(8/MINDATAWIDTH)) % MAX_NSUBWORDS) + k)*READMUX_NSEL)
							&& (j<=(((loadElemCount*(8/MINDATAWIDTH)) % MAX_NSUBWORDS) + k + 1)*READMUX_NSEL-1))
						
							readXbarSel_sig [ j ] = tempsel[ j%READMUX_NSEL ];
					end
				end
			end

			// unit stride
			else if (s3_unitstride) begin
`else
			// unit stride
			if (s3_unitstride) begin
`endif
				// index i iterates through the 8-bit bytes in the MEMWIDTH memory block
				for (i=0; i<MEM_WIDTH/MINDATAWIDTH; i=i+8/MINDATAWIDTH) begin
					// index k iterates through the subwords that make up a 16-bit halfword
					for (k=0; k<8/MINDATAWIDTH; k=k+1) begin
						// tempsel = (offsetByte + i + k) % MAX_NSUBWORDS;
						tempsel = ((offsetByte >> `log2(MINDATAWIDTH/8)) + i + k) % MAX_NSUBWORDS;
					
						// index j iterates through the READMUX_NSEL bits of the select line for the current subword
						for (j=(i+k)*READMUX_NSEL; j<=(i+1+k)*READMUX_NSEL-1; j=j+1)
							readXbarSel_sig [ j ] = tempsel[ j%READMUX_NSEL ];
					end
				end
			end
			
			// strided access; also handles zero stride
			// fixed xbar configuration for entire memory access!!
			else begin
				// index i iterates through 8-bit bytes
				for (i=0; i<MEM_WIDTH/MINDATAWIDTH; i=i+8/MINDATAWIDTH) begin
					// calculate the increment in the number of subwords % max subwords
					tempinc = (i*strideTrunc) % MAX_NSUBWORDS;
				
					// index k iterates through the subwords that make up a 32-bit word
					for (k=0; k<8/MINDATAWIDTH; k=k+1) begin
						// tempsel = (offsetByte + tempinc + k) % MAX_NSUBWORDS;
						tempsel = ((offsetByte >> `log2(MINDATAWIDTH/8)) + tempinc + k) % MAX_NSUBWORDS;
					
						// index j (bit index) iterates through the READMUX_NSEL bits of the select line for the current subword
						for (j=(i+k)*READMUX_NSEL; j<=(i+1+k)*READMUX_NSEL-1; j=j+1)
							readXbarSel_sig [ j ] = tempsel[ j%READMUX_NSEL ];
					end
				end
			end
		end
`endif
		
		// should never get here
		default:	;	// do nothing
		endcase
	end
end


endmodule
