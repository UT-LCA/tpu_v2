//************************************************************************
// Scalable Vector CPU Project -- Processor controller
// Filename: VPU_controller.v
// Author: Jason Yu
//
// Controller of scalable vector processor
// - controls issuing multi-cycle instructions
// - generates offset to VRF to access the multiple data elements in the same vector register
// - asserts vpu_control_ready when each instruction has completed
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


module VPU_controller 
#(	// Vector processor primary parameters
	parameter	NUMLANES = 1
)
(
	// ***********
	// ** Inputs **
	// ***********
	input			clk,
	input			reset_n,
	input			vpu_new_instr,				// a new instruction from the scalar CPU
	input			vpu_decodestage_stall,		// read stage needs to stall (blocks next fetched instruction from issuing)
	input			indexed_memaccess_stall,
	input			vpu_decodestage_stop,		// stops decode stage completely (stops issue state machine)
	input			id_instr_multiple_elems,	// instruction processes more than 1 element
	input			id_instr_uses_vlength,		// instruction uses VL to determine number of elements
	input			id_indexed_memaccess,
	input			id_vctrl_vectorlength,		// use vctrl_q for vector length
	input	[31:0]	vl_eff,						// vector length control register
	input	[31:0]	opCode,						// vector instruction opcode
	input	[`log2(`VRF_NREG)-1:0]		lsu_vrf_wraddr,		// wraddr for the executed load instruction
	input			lsu_vflag_reg,				// copy flag register from opcode for writing loaded data back to VRF
	input	[2:0]	lsu_data_waiting_flag,		// there is data in vector load data queue
	input			load_done_posedge,					// asserted the last cycle of load
	input			id_multElemQuot_offset,		// need to change the VRF offset to a value specified by vindex (?)
	input	[31:0]	vctrl_q,					// output of control register
	
	input			id_indexed_load,
	input			id_indexed_store,
	
	// ************
	// ** Outputs **
	// ************
	output		[`log2(NUMLANES)-1:0]	cycleFirstLaneIndex,	// first lane to process in this cycle
	output		[`log2(NUMLANES):0]	cycleLastEnabledLane,		// last lane to process in this cycle
	output				vpu_control_ready,				// indicate to decode that controller is ready for next instruction
	output		[`log2(`ELEMPERLANE)-1:0]	s2_vrfOffset,

	output reg			vpu_instr_decoded,				// a new instruction is decoded
	output reg			alu_interrupt_instr,
	output reg			enter_interrupt_mode_stalled,	// decode stage was stalled when entering interrupt instruction mode; used to recover after
	output 				int_pending_fetch_stall,		// stall decode due to pending interrupting instruction
	output reg			vpu_controller_stall_decode,	// controller stalls decode stage for executing special instructions (read from vctrl_q, etc.)
	output reg	[31:0]	instruction_register_q,			// instruction register that is valid for entire duration of instruction
	output reg			controller_indexed_loadstore,			// controller running indexed store
	output				lsu_wraddr_q_rdreq

);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local parameters
//---------------------------------------------------------------------------------------------------------------------------------------------------
localparam	LSU_VRF_WRADDR_SHIFTAMT = 32 - `log2(`VRF_NREG);	// how much to shift the wraddr by to place it in the correct opcode field
localparam	LSU_VMASK_SHIFTAMT = 13;							// how many bits to shift the vmask to place it in the correct field

// State machine states
localparam
	Idle = 0,						// Idle
	Decode = 1,						// Decoding instruction
	InstrProcessing = 2,			// Processing previous instruction
	IntInstrDecode = 3,				// Decoding an interrupting instruction
	IntInstrProcessing = 4,			// Processing an interrupting instruction
	ReadVctrlReg = 5,				// Waiting for Vindex to be read from control register
	LoadVrfOffsetDecode = 6,		// Updating VRFreadoffset value with vindex
	IntReadVctrlReg = 7,			// Waiting for Vindex to be read from control register; interrupting mode
	IntLoadVrfOffsetDecode = 8,		// Updating VRFreadoffset value with vindex; interrupting mode
	IndexedLoadStore = 9;			// State for indexed store

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Indicate the first cycle of a multi element cycle instruction
reg		[`log2(`ELEMPERLANE)-1:0]	multElemVrfOffset;
reg		multElemVrfOffset_inc;						// how much to increment offset by
wire	[`log2(NUMLANES*`ELEMPERLANE)-1:0]	multElemNumCycles;		// how many cycles will the multi-element instruction take considering VLength
wire	[`log2(`ELEMPERLANE):0]		lastElemQuot;
wire	[`log2(`ELEMPERLANE):0]		firstElemQuot;
wire	[31:0]						firstElem_index;
wire	[`log2(NUMLANES)-1:0]		lastElemRem;	// remainder of dividing vlength/vindex by nlane
wire	[`log2(NUMLANES)-1:0]		firstElemRem;	// remainder of dividing first index to be processed by nlane
wire	multElemLastCycle;				// whether it is the last cycle of a multi element instruction

reg		[31:0]	instruction_register;			// Select opCode or special interrupting instruction
reg		[31:0]	int_opCode;				// interrupting instruction opcode
wire	[31:0]	xferloadqvrf_wraddr;	// wraddr for transferring data from load queue to VRF
wire	[31:0]	xferloadqvrf_vmask;
wire	gotoInterruptMode;
wire	gotoInterruptMode_sig;
reg		gotoInterruptMode_reg;

reg	[3:0]	state, nextstate;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Combinational assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------
assign	s2_vrfOffset = multElemVrfOffset;

// determine whether it is the first cycle for a multi-cycle instruction
assign	multElemLastCycle = (multElemVrfOffset == multElemNumCycles) ? 1'b1 : 1'b0;

// ready to accept next instruction from anywhere (even pseudo instructions)
assign	vpu_control_ready = (((id_instr_multiple_elems && multElemLastCycle) || (~id_instr_multiple_elems)) || (state == Idle) || vpu_decodestage_stall)
							// is not ready in ReadVctrlReg because otherwise fetch could advance even when next instruction should wit for LSU
							&& !((state == ReadVctrlReg) && id_multElemQuot_offset)
							// if id_vctrl_vectorlength asserted, need an extra cycle to read the value from vctrl
							// if nextcycle needs to read vindex to load a new VRFoffset, not ready
							&& !((state == Decode) && (id_multElemQuot_offset || id_vctrl_vectorlength)) ? 1'b1 : 1'b0;

// next cycle will be interrupting instruction decoding
assign	lsu_wraddr_q_rdreq = gotoInterruptMode & vpu_control_ready;

// --------------------------------------------------------------------------------------------------------------------------------------------------
// Local signal assignments
// --------------------------------------------------------------------------------------------------------------------------------------------------	
assign	int_pending_fetch_stall = gotoInterruptMode;

// indexed access does not need interrupting mode
assign	gotoInterruptMode_sig = (load_done_posedge && (lsu_data_waiting_flag != `LSU_LOAD_DATA_EMPTY)) ? 1'b1 : 1'b0;
assign	gotoInterruptMode = gotoInterruptMode_sig | gotoInterruptMode_reg;
assign	xferloadqvrf_wraddr = lsu_vrf_wraddr << LSU_VRF_WRADDR_SHIFTAMT;	// wraddr for loadQvrf interrupt instruction; shift the wraddr to the correct opcode field
assign	xferloadqvrf_vmask = lsu_vflag_reg << LSU_VMASK_SHIFTAMT;


always @(posedge clk, negedge reset_n) begin
	if (~reset_n)
		gotoInterruptMode_reg <= 0;
	else if (clk == 1'b1) begin
		if (alu_interrupt_instr == 1'b1)
			gotoInterruptMode_reg <= 1'b0;
		else if (gotoInterruptMode == 1'b1)
			gotoInterruptMode_reg <= 1'b1;
	end
end

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Calculate how many cycles the instruction needs to execute for
//---------------------------------------------------------------------------------------------------------------------------------------------------

// index of the first element to be decoded is read from vindex for vext/vins instructions (ReadVctrlReg state), or when transferring data after vins.vv
// otherwise first element index is 0
assign	firstElem_index = ((state == ReadVctrlReg) || (state == LoadVrfOffsetDecode)
							|| (state == IntReadVctrlReg) || (state == IntLoadVrfOffsetDecode))
							? vctrl_q : 0;

assign	firstElemQuot = firstElem_index / NUMLANES;
assign	firstElemRem = firstElem_index % NUMLANES;
assign	lastElemQuot = vl_eff/NUMLANES;
assign	lastElemRem = vl_eff % NUMLANES;

assign	cycleFirstLaneIndex = firstElemRem;
assign	cycleLastEnabledLane = ((id_instr_uses_vlength == 1'b1) && (multElemLastCycle == 1'b1)) ? lastElemRem+1 : NUMLANES;

// If instruction is subject to vector length, otherwise process all elements
assign	multElemNumCycles = (id_instr_uses_vlength == 1'b1) ? lastElemQuot : `ELEMPERLANE-1;


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Signals to hold control lines for multi element cycles
//---------------------------------------------------------------------------------------------------------------------------------------------------
	
// **** Counter to support multiple VRF elements in a single memory ****
always @(posedge clk, negedge reset_n) begin
	// use synchronous reset to reset the count
	if (~reset_n)
		multElemVrfOffset <= 0;
	// if stopped, do not change offset
	// else if ((clk == 1'b1) && ~vpu_decodestage_stop && ~vpu_decodestage_stall) begin
	else if (clk == 1'b1) begin
		// regular decode instruction
		if ((nextstate == Decode) || (nextstate == IntInstrDecode) || ((state != IndexedLoadStore) && (nextstate == IndexedLoadStore)))
			multElemVrfOffset <= 0;
		// synchronous load of offset value for vins.sv/vext.sv instruction; load the quotient of division
		else if ((nextstate == LoadVrfOffsetDecode) || (nextstate == IntLoadVrfOffsetDecode))
			multElemVrfOffset <= firstElemQuot;
		// normal operation, increment offset each cycle
		// else if (multElemVrfOffset_inc == 1'b1)
		else if ((multElemVrfOffset_inc == 1'b1) && ~vpu_decodestage_stop && ~vpu_decodestage_stall)
			multElemVrfOffset <= multElemVrfOffset + 1;
	end
end


// --------------------------------------------------------------------------------------------------------------------------------------------------	
// Register instruction opcode
// --------------------------------------------------------------------------------------------------------------------------------------------------	
// Select interrupting isntruction, insert NOP, or use fetched instruction
always @(*) begin
	// executing interrupting instruction
	if (alu_interrupt_instr == 1'b1)
		instruction_register_q = int_opCode;
	// execute normal fetched instruction
	else
		instruction_register_q = instruction_register;
end


// When there is data waiting in the load data transfer queue, interrupt the processor with the transfer pseudo-instruction
always @(posedge clk, negedge reset_n)
begin
	if (~reset_n)
		instruction_register <= 32'h00000000;
	else if ((clk == 1'b1) && ~vpu_decodestage_stop && ~vpu_decodestage_stall) begin
		// if VPU is going into idle, replace with an idle opcode to not corrupt values
		if (nextstate == Idle)
			instruction_register <= `NOP_OPCODE;

		// do not register a new opcode if there is an interrupt instruction to be executed
		else if ((nextstate == Decode) && ~enter_interrupt_mode_stalled)
			instruction_register <= opCode;
			
	end
end

// Select the interrupted instruction instead of normal opcode
always @(posedge clk) begin
	if ((clk == 1'b1) && (nextstate == IntInstrDecode)) begin
		// check how we need to write the data in load data queue
		case (lsu_data_waiting_flag)
			// wraddr is OR'd with the blank opcode
			`LSU_LOAD_DATA_WAITING	:	int_opCode <= `OPCODE_MEM_XFERLOADQVRF | xferloadqvrf_vmask | xferloadqvrf_wraddr;
			// transfer data to vflag for flag load
			`LSU_FLAG_DATA_WAITING	:	int_opCode <= `OPCODE_MEM_XFERLOADQVFLAG | xferloadqvrf_vmask | xferloadqvrf_wraddr;
			// transfer data from loadQ to VRF; unmaskable
			`LSU_VEXT_DATA_WAITING	:	int_opCode <= `OPCODE_MEM_XFERLOADQVRF_NOMASK | xferloadqvrf_vmask | xferloadqvrf_wraddr;
			// transfer data to VRF for vins.vv
			`LSU_VINS_DATA_WAITING	:	int_opCode <= `OPCODE_MEM_XFERLOADQVRF_VINS	| xferloadqvrf_vmask | xferloadqvrf_wraddr;
			default	:	;
		endcase
	end
end


// --------------------------------------------------------------------------------------------------------------------------------------------------
// Flag to store whether there was a stall prior to entering interrupt instruction mode
// - if decodestage was stalled, need to return to Decode state without decoding a new instruction
// --------------------------------------------------------------------------------------------------------------------------------------------------	

always @(posedge clk, negedge reset_n) begin
	if (~reset_n)
		enter_interrupt_mode_stalled <= 1'b0;
	else if (clk == 1'b1)
		// clear the flag if it's on and interrupt mode is finished
		if ((nextstate == Decode) && enter_interrupt_mode_stalled)
			enter_interrupt_mode_stalled <= 1'b0;
		// set the flag if decodestage is stalled and entering interrupting mode
		else if ((nextstate == IntInstrDecode) && vpu_decodestage_stall)
			enter_interrupt_mode_stalled <= 1'b1;
end

// --------------------------------------------------------------------------------------------------------------------------------------------------
// **** Controller state machine ****
// - Decode stage always decodes an instruction
// - an interrupting instruction (transfers to/from memory queues) causes decode stage to not read a new instruction
// - two back-to-back memory instructions causes decode stage to stall with first memory instruction in LSU,
// and 2nd memory instruction stored in IR; upon return, decoder with re-decode the instruction in IR
// - non-pipelined instructions cause decode stage to stall and no new instruction accepted into IR until
// previous instruction has completed
// --------------------------------------------------------------------------------------------------------------------------------------------------

always @(posedge clk, negedge reset_n)
begin
	if (~reset_n)
		state <= Idle;
	else if ((clk == 1'b1) && ~vpu_decodestage_stop)
		state <= nextstate;
end

// State transition logic
always @ (*)
begin
	case (state)
		Idle:
			if (gotoInterruptMode && ~vpu_decodestage_stall)
				nextstate <= IntInstrDecode;
			// if there is a new instruction and the controller is not busy
			// could be stalled because pipeline is processing a non-pipelined instruction
			else if (vpu_new_instr && ~vpu_decodestage_stall)
				nextstate <= Decode;			
			else
				nextstate <= Idle;
			
		Decode:
			if (gotoInterruptMode && vpu_control_ready)
				nextstate <= IntInstrDecode;
				
`ifdef		INDEXED_MEMORY_ACCESS
			// go to indexed loadstore state, after storing write data into store buffer for store, immediately for load
			// else if ((id_indexed_load || id_indexed_store) && vpu_control_ready && ~vpu_decodestage_stall)
			// else if (id_indexed_memaccess && vpu_control_ready && ~vpu_decodestage_stall)
			else if ((id_indexed_load || (id_indexed_store && vpu_control_ready)) && ~vpu_decodestage_stall)
				nextstate <= IndexedLoadStore;
`endif
			// if controller ready; or stay in decode stage if stalled, but mask vpu_instr_decoded
			else if ((vpu_new_instr && vpu_control_ready) || vpu_decodestage_stall)
				nextstate <= Decode;
			// if need to reload vrfoffset to another value
			else if ((id_multElemQuot_offset == 1'b1) || (id_vctrl_vectorlength == 1'b1))
				nextstate <= ReadVctrlReg;
			// if controller is ready but no new instruction
			else if (~vpu_new_instr && vpu_control_ready)
				nextstate <= Idle;
			// if controller not ready
			else
				nextstate <= InstrProcessing;

		// State to wait for Vindex to be read from control register before continuing
		ReadVctrlReg:
			// no state transition to IntInstrDecode... but it's okay, only need to wait 1 extra cycle
		
			if (id_multElemQuot_offset == 1'b1)
				nextstate <= LoadVrfOffsetDecode;
			// if controller ready; stay in decode stage if stalled, but mask vpu_instr_decoded
			else if (vpu_new_instr && vpu_control_ready)
				nextstate <= Decode;
			// if controller is ready but no new instruction
			else if (~vpu_new_instr && vpu_control_ready)
				nextstate <= Idle;
			// if controller not ready
			else
				nextstate <= InstrProcessing;
				
		// Special decode state that also loads VrfOffset to different value
		LoadVrfOffsetDecode:
			if (gotoInterruptMode && vpu_control_ready)
				nextstate <= IntInstrDecode;
			// if controller ready; stay in decode stage if stalled, but mask vpu_instr_decoded
			// else if ((vpu_new_instr && vpu_control_ready) || vpu_decodestage_stall)
			else if (vpu_decodestage_stall)
				nextstate <= LoadVrfOffsetDecode;
			// if controller ready; stay in decode stage if stalled, but mask vpu_instr_decoded
			else if (vpu_new_instr && vpu_control_ready)
				nextstate <= Decode;
			// if controller is ready but no new instruction
			else if (~vpu_new_instr && vpu_control_ready)
				nextstate <= Idle;
			// if controller not ready
			else
				nextstate <= InstrProcessing;
		
		// Processing an instruction; decode stage could be stalled depending on current instruction
		InstrProcessing:
			if (gotoInterruptMode && vpu_control_ready)
				nextstate <= IntInstrDecode;

`ifdef	INDEXED_MEMORY_ACCESS
			// go to indexed load store state, after storing write data into store buffer for stores, immediately for loads
			// else if ((id_indexed_load || id_indexed_store) && vpu_control_ready && ~vpu_decodestage_stall)
			// else if (id_indexed_memaccess && vpu_control_ready && ~vpu_decodestage_stall)
			else if ((id_indexed_load || (id_indexed_store && vpu_control_ready)) && ~vpu_decodestage_stall)
				nextstate <= IndexedLoadStore;
`endif

			// go back to decode for next instruction
			else if (vpu_new_instr && vpu_control_ready && ~vpu_decodestage_stall)
				nextstate <= Decode;
			// no more instructions, go to Idle
			else if (vpu_control_ready && ~vpu_decodestage_stall)
				nextstate <= Idle;
			// else still processing; or if stalled
			else
				nextstate <= InstrProcessing;

		// whether vpu_decodestage_stall or not, does not affect this state transition
		IntInstrDecode:
			if (vpu_control_ready && gotoInterruptMode)
				nextstate <= IntInstrDecode;
			else if (vpu_control_ready && vpu_new_instr)
				nextstate <= Decode;
			// if need to reload vrfoffset to another value
			else if ((id_multElemQuot_offset == 1'b1) || (id_vctrl_vectorlength == 1'b1))
				nextstate <= IntReadVctrlReg;
			else if (vpu_control_ready && ~vpu_new_instr)
				nextstate <= Idle;
			else
				nextstate <= IntInstrProcessing;
				
		// Processing an interrupting instruction; whether vpu_decodestage_stall or not, does not affect this state transition
		IntInstrProcessing:
			// interrupting instruction has highest priority
			if (gotoInterruptMode && vpu_control_ready)
				nextstate <= IntInstrDecode;
			// else if (vpu_new_instr && vpu_control_ready)
			else if ((vpu_new_instr || enter_interrupt_mode_stalled) && vpu_control_ready)
				nextstate <= Decode;
			else if (vpu_control_ready && ~vpu_new_instr)
				nextstate <= Idle;
			// else still processing
			else
				nextstate <= IntInstrProcessing;
		
		// Wait to read from control register; interrupting mode (for vins.vv)
		IntReadVctrlReg:
			nextstate <= IntLoadVrfOffsetDecode;
				
		// Special decode state that also loads VrfOffset to different value; interrupting mode (for vins.vv)
		IntLoadVrfOffsetDecode:
			// nextstate <= IntInstrProcessing;
			if (vpu_control_ready && gotoInterruptMode)
				nextstate <= IntInstrDecode;
			else if (vpu_control_ready && vpu_new_instr)
				nextstate <= Decode;
			else if (vpu_control_ready && ~vpu_new_instr)
				nextstate <= Idle;
			else
				nextstate <= IntInstrProcessing;
		
`ifdef	INDEXED_MEMORY_ACCESS
		IndexedLoadStore:
			if (gotoInterruptMode && vpu_control_ready)
				nextstate <= IntInstrDecode;
			// go back to decode for next instruction
			else if (vpu_new_instr && vpu_control_ready && ~vpu_decodestage_stall)
				nextstate <= Decode;
			// no more instructions, go to Idle
			else if (vpu_control_ready && ~vpu_decodestage_stall)
				nextstate <= Idle;
			// else still processing; or if stalled
			else
				nextstate <= IndexedLoadStore;
`endif
	endcase
end


// Output logic
always @ (*)
begin
	// default assignments
	alu_interrupt_instr = 1'b0;
	multElemVrfOffset_inc = 1'b0;
	vpu_instr_decoded = 1'b0;
	vpu_controller_stall_decode = 1'b0;
	controller_indexed_loadstore = 1'b0;

	case (state)
		Idle :
			// to clear pipeline when idle
			vpu_instr_decoded = 1'b1;
		
		Decode : begin
			if ((id_multElemQuot_offset == 1'b0) && (id_vctrl_vectorlength == 1'b0))
				multElemVrfOffset_inc = 1'b1;
			
			// if instruction does not need to reload VRF offset with different value
			if ((id_multElemQuot_offset == 1'b0) && (id_vctrl_vectorlength == 1'b0))
				vpu_instr_decoded = 1'b1;
			
			// if instruction needs to reload VRF offset with different value
			if ((id_multElemQuot_offset == 1'b1) || (id_vctrl_vectorlength == 1'b1))
				// VPU controller needs to stall the processor to update
				vpu_controller_stall_decode = 1'b1;
		end
		
		ReadVctrlReg: begin
			// if instruction needs to reload VRF offset, stall processor to update
			if (id_multElemQuot_offset == 1'b1)
				vpu_controller_stall_decode = 1'b1;
			// if instruction does not need to reload VRF offset with different value, only read vctrl_q for VL
			if (id_vctrl_vectorlength == 1'b1)
				vpu_instr_decoded = 1'b1;
		end
		
		// Special decode state that also loads VrfOffset to different value
		LoadVrfOffsetDecode: begin
			multElemVrfOffset_inc = 1'b1;
			vpu_instr_decoded = 1'b1;
		end

		// Processing instruction
		InstrProcessing : begin
			// do not increment the offset yet for indexed memory access if decode stage is stalled (by indexed instruction)
			if ((id_indexed_memaccess == 1'b1) && (indexed_memaccess_stall == 1'b1))
				multElemVrfOffset_inc = 1'b0;
			else
				multElemVrfOffset_inc = 1'b1;
		end

		
		// Interrupting instruction decode
		IntInstrDecode :
		begin
			alu_interrupt_instr = 1'b1;
			
			// do not increment the offset yet for indexed memory access
			if ((id_indexed_memaccess == 1'b0) && (id_multElemQuot_offset == 1'b0) && (id_vctrl_vectorlength == 1'b0))
				multElemVrfOffset_inc = 1'b1;
			
			// if instruction does not need to reload VRF offset with different value
			if ((id_multElemQuot_offset == 1'b0) && (id_vctrl_vectorlength == 1'b0))
				vpu_instr_decoded = 1'b1;
			
			// if instruction needs to reload VRF offset with different value
			if ((id_multElemQuot_offset == 1'b1) || (id_vctrl_vectorlength == 1'b1))
				// VPU controller needs to stall the processor to update
				vpu_controller_stall_decode = 1'b1;
		end
		
		// Processing interrupting instruction
		IntInstrProcessing :
		begin
			alu_interrupt_instr = 1'b1;		// indicate it is an interrupting instruction
			multElemVrfOffset_inc = 1'b1;
		end
		
		// Read vector control register; interrupting mode
		IntReadVctrlReg: begin
			alu_interrupt_instr = 1'b1;
			
			// if instruction needs to reload VRF offset, stall processor to update
			if (id_multElemQuot_offset == 1'b1)
				vpu_controller_stall_decode = 1'b1;
		end
		
		// Special decode state that also loads VrfOffset to different value
		IntLoadVrfOffsetDecode: begin
			alu_interrupt_instr = 1'b1;
			multElemVrfOffset_inc = 1'b1;
			vpu_instr_decoded = 1'b1;
		end

`ifdef	INDEXED_MEMORY_ACCESS
		IndexedLoadStore: begin
			controller_indexed_loadstore = 1'b1;
			// do not increment the offset yet for indexed memory access if decode stage is stalled (by indexed instruction)
			if ((id_indexed_memaccess == 1'b1) && (indexed_memaccess_stall == 1'b1))
				multElemVrfOffset_inc = 1'b0;
			else
				multElemVrfOffset_inc = 1'b1;
		end
`endif
		
		default:	;
	endcase
end

endmodule
