//**************************************************************************************
// Scalable Vector CPU Project -- Memory operation controller
// Filename: loadstoreController.v
// Author: Jason Yu
//
// The controller issues vector and scalar memory instructions
// - instructions are issued in program order
// - arbitrates scalar/memory accesses
// - stores a copy of the memory instruction opcode for execution
// - stores a flag (lsu_data_waiting_flag) to notify the main processor that
// load data is ready to be read from the load queues
//
//  Copyright (C) 2007 Jason Yu
//
//**************************************************************************************

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


module loadstoreController
(
	input				clk,
	input				reset_n,
	input				lsu_instr_decode_valid,
	input		[31:0]	lsu_opCode,						// portion of the opcode to loadstore controller
	input				d_waitrequest,					// need to wait for memory
	
	// Inputs from scalar core
	input				scalar_d_read,					// scalar data read request
	input				scalar_d_write,					// scalar data write request
	input		[1:0]	scalar_data_size,				// size of data to be writte to data master: 00 = byte, 01 = halfword, 10 = word
	input		[31:0]	id_vl_special_instr,	// effective vector length -1
	input		[31:0]	id_vl_minus_one,				// actual vector length -1
	input		[31:0]	id_vbase,						// base address of memory operation
	input		[`MAXSTRIDE_NBIT-1:0]	id_vstride,			// up to 32 element stride

	// Decoded control signals
	input		[2:0]	id_memDataWidth,
	input				id_clear_lsu_data_flag,			// clear the load data waiting flag
	// Inputs from load and store units
	input				vmanip_storeload_sync,			// start load unit for vector manipulation instructions
	input				load_done_posedge,
	input				store_done_posedge,

	input				mem_order_q_q,
	input				mem_order_q_empty,
	input				lsu_wraddr_q_rdreq,

	// Control signals to the load store unit
	output reg	[2:0]	lsu_memDataWidth,
	output reg			lsu_unitstride,
	output reg			lsu_indexed_memaccess,
	output				lsu_signedOp,
	output				lsu_newInstr_load,				// start a load instruction
	output				lsu_newInstr_store,
	output reg			lsu_flagload,
	output reg			lsu_flagstore,
	output 				lsu_scalar_load,				// indicate scalar access to load address generator
	output reg			lsu_storetoload_bypass_en,		// bypass from store to load datapath
	output		[`log2(`VRF_NREG)-1:0]		lsu_vrf_wraddr,	// copy wraddr from opcode for writing loaded data back to VRF
	output				lsu_vflag_reg,					// copy flag register from opcode for writing loaded data back to VRF
	output		[31:0]	lsu_vl_special_instr,
	output		[31:0]	lsu_vl_minus_one,
	output		[31:0]	lsu_baseAddr,					// base address of memory operation
	output		[`MAXSTRIDE_NBIT-1:0]	lsu_stride,		// up to 32 element stride

	output reg			executing_vector_load,			// executing vector load
	output reg	[2:0]	lsu_data_waiting_flag,			// Flag to indicate what type of data is waiting the load data transfer queue

	output reg			scalar_write_grant,
	output 				lsu_vext_vv,
	output				lsu_vins_vv,
	
	// state signals
	output reg			executing_vector_store,
	output reg			executing_scalar_store,
	output reg			executing_scalar_load,
	output reg			vector_manip_loading,
	output				lsu_busy,						// loadstore unit is busy
	
	output reg			scalar_d_waitrequest,			// pause scalar processor while waiting for data
	output				vectorload_waitrequest,			// pause vector processor while waiting for data
	output				vectorstore_waitrequest,			// pause vector processor while waiting for memory

	output reg			mem_order_q_rdreq

);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	[31:0]	lsu_opCode_q;
wire			lsu_ready;								// the loadstore unit is not busy
reg				scalar_read_grant;
reg				vector_store_grant;
wire			lsu_newInstr_load_sig;
wire			lsu_newInstr_store_sig;
// wire	[2:0]	vector_memDataWidth;
reg		[2:0]	scalar_memDataWidth;
wire	[`ALUOP_WIDTH-1:0]	lsu_ALUOp;
reg				vector_manip_grant;
reg				use_load_controller;
reg				use_store_controller;
reg				lsu_vector_load;
reg				dec_lsu_vext_vv;
reg				dec_lsu_vins_vv;
reg				lsu_vbase_postaccess_update;
reg				vector_manip_pending;
wire			vmanip_storeload_sync_delay;
wire			vmanip_storeload_sync_posedge;

wire			register_lsu_opcode;
wire			lsu_opCode_sclr;

wire		lsu_wraddr_q_wrreq;
wire		lsu_wraddr_q_empty;
wire		lsu_wraddr_q_full;
wire	[`log2(`VRF_NREG)-1:0]	lsu_wraddr_q_data;
wire	[`log2(`VRF_NREG)-1:0]	lsu_wraddr_q_q;

wire		register_lsu_opcode_delay;
wire	executing_vector_load_posedge;
wire	executing_vector_load_delayn;

// state machine signals
reg		[3:0]	state, nextstate;

parameter
	Idle = 0,
	Decode = 1,
	VectorRead = 2,
	VectorWrite = 3,
	ScalarRead = 4,
	ScalarWrite = 5,
	ScalarWriteWait = 6,
	Vector_Manip_Wait = 7,					// need to wait 1 cycle to read from vector control register for vector manipulation (vins.vv, vext.vv)
	Vector_Manip_Storephase = 8,			// vector manipulation instruction, store phase
	Vector_Manip_Loadphase = 9;				// vector manipulation instruction, laod phase
	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Start memory accesses
// - scalar access has higher priority over vector access
// - load and store cannot be granted at the same time since use_load_controller
// and use_store_controller cannot be high simultaneously
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Allow scalar accesses before vector accesses
// Grant priority: 1) scalar load/store, 2) vector load/store
// Grant signals asserted same cycle as appears in ID

// do not need to include the grant signals because lsu_busy hazard is detected when the 2nd memory instruction is in the instruction register, and the loadstore controller is executing
assign	lsu_busy = ~lsu_ready;
// don't need to register as scalar load takes only a single cycle
assign	lsu_scalar_load = scalar_read_grant | executing_scalar_load;

// need the signal the cycle it is granted (1 before actually decoded)
assign	lsu_vext_vv = dec_lsu_vext_vv;
assign	lsu_vins_vv = dec_lsu_vins_vv;

assign	lsu_ready = (state == Idle) ? 1'b1 : 1'b0;
assign	register_lsu_opcode = lsu_instr_decode_valid & lsu_ready;
assign	lsu_vrf_wraddr = lsu_wraddr_q_q;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Signal assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------
assign	vmanip_storeload_sync_posedge = vmanip_storeload_sync & ~vmanip_storeload_sync_delay;
ff	#(.WIDTH(1))	pipe_vmanip_storeload_sync (.clk(clk), .en(1'b1), .clrn(reset_n), .d(vmanip_storeload_sync), .q(vmanip_storeload_sync_delay));

assign	lsu_newInstr_store_sig = vector_store_grant | vector_manip_grant;
// Delay signal by 1 cycle so the signal is when base address/stride have been read
ff	#(.WIDTH(1))	pipe_lsu_newInstr_store (.clk(clk), .en(1'b1), .clrn(reset_n), .d(lsu_newInstr_store_sig), .q(lsu_newInstr_store));

// Delay an extra cycle to register vbase and vstride from vector core
assign	executing_vector_load_posedge = executing_vector_load & executing_vector_load_delayn;
assign	lsu_newInstr_load = executing_vector_load_posedge | (vmanip_storeload_sync_posedge && (state == Vector_Manip_Storephase));
ff	#(.WIDTH(1))	pipe_executing_vector_load (.clk(clk), .en(1'b1), .clrn(reset_n), .d(~executing_vector_load), .q(executing_vector_load_delayn));


// Generate memory width signal from scalar instruction
// No need for "_ENABLED" directives because all three stores are supported regardless of vector memory store granularity
always @(*)
	case(scalar_data_size)
		2'b00:	scalar_memDataWidth = `MEMACCESS_BYTE;
		2'b01:	scalar_memDataWidth = `MEMACCESS_HALFWORD;
		2'b10:	scalar_memDataWidth = `MEMACCESS_WORD;
		default:	scalar_memDataWidth = 3'bx;
	endcase

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Stall the vector and scalar processors when needed
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Processor is waiting for data from memory
assign	vectorload_waitrequest = (executing_vector_load | executing_scalar_load) & d_waitrequest;
assign	vectorstore_waitrequest = (executing_vector_store | executing_scalar_store) & d_waitrequest;


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Sequential logic
//---------------------------------------------------------------------------------------------------------------------------------------------------
always @(posedge clk) begin
	// load memdatawidth for scalar access
	if ((nextstate == ScalarRead) || (nextstate == ScalarWrite))
		lsu_memDataWidth <= scalar_memDataWidth;
	else if (register_lsu_opcode == 1'b1)
		lsu_memDataWidth <= id_memDataWidth;
end

//---------------------------------------------------------------------------------------------------------------------------------------------------
// State machine to track what the loadstore controller is executing
// - loadstore unit can only be executing one operation at a time: scalar read, scaalr write, vector read, vector write
//---------------------------------------------------------------------------------------------------------------------------------------------------

// State transition logic
always @ (posedge clk or negedge reset_n)
begin
	if (~reset_n)
		state <= Idle;
	else if (clk == 1'b1)
		state <= nextstate;
end

	
// State transition logic
always @ (*)
begin
	case (state)
		Idle:
			// if ((lsu_instr_decode_valid == 1'b1) || (scalar_d_read == 1'b1) || (scalar_d_write == 1'b1) || (~mem_order_q_empty))
			// move to decode if vector memory instruction is ready to execute or there is pending scalar memory instruction
			if ((lsu_instr_decode_valid == 1'b1) || ((mem_order_q_empty == 1'b0) && (mem_order_q_q == 1'b0)))
				nextstate = Decode;
			else
				nextstate = Idle;
		
		Decode: begin
			if (dec_lsu_vext_vv | dec_lsu_vins_vv)
				nextstate = Vector_Manip_Wait;
			
			// if memory access order Q not empty, means there is a memory instruction somewhere in the pipeline
			else if (~mem_order_q_empty) begin
				// nextstate = MemoryAccessPending;

				// if scalar request is at the head of queue
				if (mem_order_q_q == 1'b0) begin
					if (scalar_d_read)
						nextstate = ScalarRead;
					else if (scalar_d_write)
						nextstate = ScalarWrite;
					// should never get here
					else
						nextstate = Decode;
				end
				// if vector request is at the head of queue
				else begin
					// allow the next vector load to issue even if there is data in the load data queue; but the load address controller will pause if the queue is full
					// if (lsu_instr_decode_valid & use_load_controller)
					if (use_load_controller)
						nextstate = VectorRead;
					// else if (lsu_instr_decode_valid & use_store_controller)
					else if (use_store_controller)
						nextstate = VectorWrite;
					// should never get here
					else
						nextstate = Decode;
				end
			end
			
			else
				nextstate = Decode;
		
		end
		
		// normally scalar load would finish in 1 cycle, unless stalled by waitrequest
		ScalarRead:
			if (d_waitrequest == 1'b0)
				nextstate = Idle;
			else
				nextstate = ScalarRead;

		// Due to pipelining, scalar write takes 2 cycles
		ScalarWrite:
			nextstate = ScalarWriteWait;
		
		// Due to pipelining, scalar write takes 2 cycles
		ScalarWriteWait:
			if (d_waitrequest == 1'b0)
				nextstate = Idle;
			else
				nextstate = ScalarWriteWait;

		VectorRead:
			if (load_done_posedge == 1'b1)
				nextstate = Idle;
			else
				nextstate = VectorRead;
		
		VectorWrite:
			if (store_done_posedge == 1'b1)
				nextstate = Idle;
			else
				nextstate = VectorWrite;
		
		Vector_Manip_Wait:
			nextstate = Vector_Manip_Storephase;
		
		Vector_Manip_Storephase:
			// wait until the load phase starts
			if (vmanip_storeload_sync == 1'b1)
				nextstate = Vector_Manip_Loadphase;
			else
				nextstate = Vector_Manip_Storephase;

		Vector_Manip_Loadphase:
			// wait until the load portion of the instruction is done
			if (load_done_posedge == 1'b1)
				nextstate = Idle;
			else
				nextstate = Vector_Manip_Loadphase;
		
	endcase
end

// State output logic
always @(*)
begin
	// default assignments
	executing_scalar_load	= 1'b0;
	executing_scalar_store	= 1'b0;
	executing_vector_load	= 1'b0;
	executing_vector_store	= 1'b0;
	scalar_d_waitrequest	= 1'b0;
	scalar_read_grant = 1'b0;
	scalar_write_grant = 1'b0;
	vector_store_grant = 1'b0;
	vector_manip_grant = 1'b0;
	vector_manip_pending = 1'b0;
	vector_manip_loading = 1'b0;
	mem_order_q_rdreq = 1'b0;
	
	case (state)
		Idle:
			// scalar core needs to wait
			if ((scalar_d_read == 1'b1) || (scalar_d_write == 1'b1))
				scalar_d_waitrequest = 1'b1;

		Decode: begin
			// scalar core needs to wait
			if ((scalar_d_read == 1'b1) || (scalar_d_write == 1'b1))
				scalar_d_waitrequest = 1'b1;
				
			if (dec_lsu_vext_vv | dec_lsu_vins_vv)
				// nextstate = Vector_Manip_Wait
				vector_manip_pending = 1'b1;
				
			else if (~mem_order_q_empty) begin

				// if scalar request is at the head of queue
				if (mem_order_q_q == 1'b0) begin
					if (scalar_d_read) begin
						// nextstate = ScalarRead;
						scalar_read_grant = 1'b1;
						mem_order_q_rdreq = 1'b1;
					end
					else if (scalar_d_write) begin
						// nextstate = ScalarWrite;
						scalar_write_grant = 1'b1;
						mem_order_q_rdreq = 1'b1;
					end
					// else
						// nextstate = Idle;
				end
				// if vector request is at the head of queue
				else begin
					// allow the next vector load to issue even if there is data in the load data queue; but the load address controller will pause if the queue is full
					if (use_load_controller) begin
						// nextstate = VectorRead;
						mem_order_q_rdreq = 1'b1;
					end
					else if (use_store_controller) begin
						// nextstate = VectorWrite;
						vector_store_grant = 1'b1;
						mem_order_q_rdreq = 1'b1;
					end
					// else
						// nextstate = Idle;
				end
			end
				
		end
		
		ScalarRead: begin
			executing_scalar_load	= 1'b1;
			// if loadstore unit is executing scalar read and the memory is ready, scalar core can resume
			if (d_waitrequest == 1'b1)
				scalar_d_waitrequest	= 1'b1;
		end
		
		ScalarWrite: begin
			executing_scalar_store	= 1'b1;
			// if loadstore unit is executing scalar store and the memory is ready, scalar core can resume
			if (d_waitrequest == 1'b1)
				scalar_d_waitrequest	= 1'b1;
		end
		
		ScalarWriteWait: begin
			executing_scalar_store	= 1'b1;
			// if loadstore unit is executing scalar store and the memory is ready, scalar core can resume
			if (d_waitrequest == 1'b1)
				scalar_d_waitrequest	= 1'b1;
		end

		VectorRead: begin
			executing_vector_load	= 1'b1;
			// if there is a pending scalar read or write while executing vector memory instruction, stall the scalar core
			if (scalar_d_read || scalar_d_write)
				scalar_d_waitrequest = 1'b1;
		end
		
		VectorWrite: begin
			executing_vector_store	= 1'b1;
			// if there is a pending scalar read or write while executing vector memory instruction, stall the scalar core
			if (scalar_d_read || scalar_d_write)
				scalar_d_waitrequest	= 1'b1;
		end

		Vector_Manip_Wait:
		// grant vector manipulation instruction 1 cycle later than regular load/store to allow reading vindex from vector control register
			vector_manip_grant = 1'b1;

		Vector_Manip_Loadphase:
			vector_manip_loading = 1'b1;
			
		default:	;
	endcase
end


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Indicate data is waiting in the load data queue
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Generate data waiting flag
// 000 = no data waiting
// 001 = memory load data waiting
// 010 = memory flag load data waiting
// 011 = vector extract data waiting
// 100 = vector insert transfer waiting

// Indicate what type of data is loaded in the load data transfer queue
always @(posedge clk, negedge reset_n)
begin
	if (~reset_n)
		lsu_data_waiting_flag <= 3'b000;
	else if (clk == 1'b1) begin
		if (id_clear_lsu_data_flag)
		// clear on Idle state
		// if (lsu_ready)
			lsu_data_waiting_flag <= `LSU_LOAD_DATA_EMPTY;
		// vector load
		else if (executing_vector_load_posedge & lsu_vector_load)
			lsu_data_waiting_flag <= `LSU_LOAD_DATA_WAITING;
		// vector flag load
		else if (executing_vector_load_posedge & lsu_flagload)
			lsu_data_waiting_flag <= `LSU_FLAG_DATA_WAITING;
`ifdef	VECTOR_MANIP_INSTRUCTIONS
		// vector extract
		else if (vector_manip_pending & lsu_vext_vv)
			lsu_data_waiting_flag <= `LSU_VEXT_DATA_WAITING;
		// vector insert
		else if (vector_manip_pending & lsu_vins_vv)
			lsu_data_waiting_flag <= `LSU_VINS_DATA_WAITING;
`endif
	end
end

	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Control signals stored at the LSU for load/store operation
//---------------------------------------------------------------------------------------------------------------------------------------------------
assign	lsu_ALUOp = lsu_opCode_q[`OP_FUNC];
assign	lsu_vflag_reg = lsu_opCode_q[`OP_XMASK];
assign	lsu_signedOp = ~lsu_ALUOp[`OP_SIGNBIT];

assign	lsu_opCode_sclr = (nextstate == Idle) ? 1'b1 : 1'b0;

// Store signals from id
// Load store controller opcode, let synthesis tool optimize away unneeded bits
pipeline_reg_sclr	#(.WIDTH(32))	loadstore_opCode 
(	.clk		(clk),
	.en			(register_lsu_opcode),
	.clrn		(reset_n),
	.d			(lsu_opCode),
	.q			(lsu_opCode_q),
	.sclr		(lsu_opCode_sclr)
);

pipeline_reg_sclr	#(.WIDTH(`MAXSTRIDE_NBIT))	loadstore_vstride
(	.clk		(clk),
	.en			(register_lsu_opcode_delay),	// 1 cycle after latching opcode
	.clrn		(reset_n),
	.d			(id_vstride),
	.q			(lsu_stride),
	.sclr		(lsu_opCode_sclr)
);

pipeline_reg_sclr	#(.WIDTH(32))	loadstore_vbase
(	.clk		(clk),
	.en			(register_lsu_opcode_delay),	// 1 cycle after latching opcode
	.clrn		(reset_n),
	.d			(id_vbase),
	.q			(lsu_baseAddr),
	.sclr		(lsu_opCode_sclr)
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Set effective VL for memory interface core:
// needed for flagstore, vext.vv
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Effective vector length minus one
// for vext.vv and vins.vv, vl_eff = vl + vindex - 1
ff	#(.WIDTH(32))	loadstore_vl_special_instr
(	.clk		(clk),
	.en			(register_lsu_opcode),
	.clrn		(reset_n),
	.d			(id_vl_special_instr),
	.q			(lsu_vl_special_instr)
);

// Actual vector length minus one
ff	#(.WIDTH(32))	loadstore_vl_minus_one
(	.clk		(clk),
	.en			(register_lsu_opcode),
	.clrn		(reset_n),
	.d			(id_vl_minus_one),
	.q			(lsu_vl_minus_one)
);

// Predecoded to balance pipeline stages and improve Fmax
// ff	#(.WIDTH(3))	lsu_memDataWidth_preg 
// (	.clk		(clk),
	// .en			(register_lsu_opcode),
	// .clrn		(reset_n),
	// .d			(id_memDataWidth),
	// .q			(vector_memDataWidth)
// );



//---------------------------------------------------------------------------------------------------------------------------------------------------
// LSU instruction decode
//---------------------------------------------------------------------------------------------------------------------------------------------------

// Decode instruction
always @(*) begin
	// default assignments
	lsu_unitstride = 1'b0;
	lsu_indexed_memaccess = 1'b0;
	lsu_flagload = 1'b0;
	lsu_flagstore = 1'b0;
	lsu_storetoload_bypass_en = 1'b0;
	lsu_vector_load = 1'b0;
	dec_lsu_vext_vv = 1'b0;
	dec_lsu_vins_vv = 1'b0;
	lsu_vbase_postaccess_update = 1'b0;
	
	use_load_controller = 1'b0;
	
	casex (lsu_opCode_q[`OP_OPCODE])
		`OPCODE_VRTYPE: begin
			// Decode instruction
			casex (lsu_ALUOp)
`ifdef	VECTOR_MANIP_INSTRUCTIONS
				// Vector extract
				`ALUOP_VEXT: begin
					// Vext.vv
					if (lsu_opCode_q[`OP_OPX] == `OPX_VRTYPE) begin
						lsu_storetoload_bypass_en = 1'b1;
						lsu_unitstride = 1'b1;					// use unitstride crossbar configuration
						dec_lsu_vext_vv = 1'b1;
						use_load_controller = 1'b1;
						use_store_controller = 1'b1;
					end
				end
`endif

`ifdef	VECTOR_MANIP_INSTRUCTIONS
				// Vector insert
				`ALUOP_VINS: begin
					// Vins.vv
					if (lsu_opCode_q[`OP_OPX] == `OPX_VRTYPE) begin
						lsu_storetoload_bypass_en = 1'b1;
						lsu_unitstride = 1'b1;					// use unitstride crossbar configuration
						dec_lsu_vins_vv = 1'b1;
						use_load_controller = 1'b1;
						use_store_controller = 1'b1;
					end
				end
`endif
				default:	;
			endcase
		end

		`OPCODE_VMTYPE: begin
			// Decode memory instruction
			casex (lsu_ALUOp)
				// flag load
				`FUNC_FLAGLOAD:	begin
					// flag load uses part of unitstride logic
					lsu_unitstride = 1'b1;
					// this is now decoded in VPU_decode as a pre-decode step
					lsu_flagload = 1'b1;
					lsu_vbase_postaccess_update = 1'b1;
					use_load_controller = 1'b1;
				end
				
				// flag store
				`FUNC_FLAGSTORE: begin
					// flag store uses part of unitstride logic
					lsu_unitstride = 1'b1;			// needed to set crossbar configuration
					lsu_flagstore = 1'b1;
					lsu_vbase_postaccess_update = 1'b1;
					use_store_controller = 1'b1;
				end
				
				// s3_unitstride load
				`FUNC_MEM_UNITLOAD: begin
					lsu_unitstride = 1'b1;
					lsu_vector_load = 1'b1;
					lsu_vbase_postaccess_update = 1'b1;
					use_load_controller = 1'b1;
				end
				
				// strided load
				`FUNC_MEM_STRIDEDLOAD: begin
					lsu_vector_load = 1'b1;
					lsu_vbase_postaccess_update = 1'b1;
					use_load_controller = 1'b1;
				end
				
				// Unit stride Store instructions
				`FUNC_MEM_UNITSTORE: begin
					lsu_unitstride = 1'b1;
					lsu_vbase_postaccess_update = 1'b1;
					use_store_controller = 1'b1;
				end
				
				// Strided store instructions
				`FUNC_MEM_STRIDEDSTORE: begin
					lsu_vbase_postaccess_update = 1'b1;
					use_store_controller = 1'b1;
				end

`ifdef	INDEXED_MEMORY_ACCESS
				// Indexed load
				`FUNC_MEM_INDEXEDLOAD: begin
					lsu_indexed_memaccess = 1'b1;
					lsu_vector_load = 1'b1;
					use_load_controller = 1'b1;
				end

				// Indexed store
				`FUNC_MEM_INDEXEDSTORE: begin
					lsu_indexed_memaccess = 1'b1;
					use_store_controller = 1'b1;
					//lsu_unitstride = 1'b1;				// use part of unitstride logic in store controller
				end
`endif
			default:	;
			endcase
		end
	default: 	;
	endcase
end


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Queue to store write address of a vector load instruction so data can be transferred from load buffer
// to target register after memory access
//---------------------------------------------------------------------------------------------------------------------------------------------------
// get wraddr from registered opcode
assign	lsu_wraddr_q_data = lsu_opCode_q[`OP_VMEMWRADDR];
assign	lsu_wraddr_q_wrreq = register_lsu_opcode_delay & use_load_controller;

ff	#(.WIDTH(1))	pipe_register_lsu_opcode (.clk(clk), .en(1'b1), .clrn(reset_n), .d(register_lsu_opcode), .q(register_lsu_opcode_delay));

lsu_wraddr_q	lsu_wraddr_q_inst (
	.clock ( clk ),
	.data ( lsu_wraddr_q_data ),
	.rdreq ( lsu_wraddr_q_rdreq ),
	.wrreq ( lsu_wraddr_q_wrreq ),
	.empty ( lsu_wraddr_q_empty ),
	.full ( lsu_wraddr_q_full ),
	.q ( lsu_wraddr_q_q ),
	.usedw (  )
);

	
endmodule
