//************************************************************************
// Scalable Vector CPU Project -- Hazard detection logic
// Filename: VPU_hazard_detection.v
// Author: Jason Yu
//
// VPU pipeline hazard detection
// - detects all RAW pipeilne hazards
// - detects all other instances where the processor needs to be stalled
// - generates fetch stall signal to scalar core to stall instruction fetching
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


module VPU_hazard_detection

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Customizable parameters
//---------------------------------------------------------------------------------------------------------------------------------------------------
#(	// Vector processor primary parameters
	parameter	NUMLANES = 1
)
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Ports
//---------------------------------------------------------------------------------------------------------------------------------------------------
(
	// ***********
	// ** Inputs **
	// ***********
	input					clk,
	input					reset_n,
	input					vpu_control_ready,				// the controller is ready for the next instruction
	input					vpu_instr_decoded,				// instruction has reached decode stage
	input					predecode_vector_instr,			// next instruction is a vector instruction
	input					lsu_executing_vector_load,		// executing vector load
	
	input	[`log2(`VRF_NREG)-1:0]		lsu_vrf_wraddr,		// wraddr for the executed load instruction
	input	[`log2(`ELEMPERLANE)-1:0]	s2_vrfRdOffset,		// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
	input	[`log2(`VRF_NREG)-1:0]		id_vrf_rdaddr1,
	input	[`log2(`VRF_NREG)-1:0]		id_vrf_rdaddr2,
	input	[`log2(`FLAG_NREG)-1:0]		id_vflag_rdaddr1,
	input	[`log2(`FLAG_NREG)-1:0]		id_vflag_rdaddr2,
	input	[`log2(`ELEMPERLANE)-1:0]	s3_vrfWrOffset,
	input	[`log2(`ELEMPERLANE)-1:0]	s4_vrfWrOffset,		// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
	input	[`log2(`VRF_NREG)-1:0]		s3_vrf_wraddr,
	input	[`log2(`VRF_NREG)-1:0]		s4_vrf_wraddr,
	input	[`log2(`FLAG_NREG)-1:0]		s3_vflag_wraddr,
	input	[`log2(`FLAG_NREG)-1:0]		s4_vflag_wraddr,
	input					dec_s3_vrf_wren,				// for hazard detection
	input					dec_s4_vrf_wren,				// for hazard detection
	input					dec_s3_vflag_wren,				// for hazard detection
	input					dec_s4_vflag_wren,				// for hazard detection
	input					id_instr_reads_vrfrdaddr1,		// does instruction use the operand; for RAW hazard checking
	input					id_instr_reads_vrfrdaddr2,		// does isntruction use the operand; for RAW hazard checking
	input					id_instr_reads_vflagrdaddr1,
	input					id_instr_reads_vflagrdaddr2,
	
	input	[2:0]			lsu_data_waiting_flag,
	input					enter_interrupt_mode_stalled,	// decode stage was stalled when entering interrupt instruction mode; used to recover after
	input					int_pending_fetch_stall,		// stall decode due to pending int instruction
	input					lsu_busy,
	input					id_stall_if_lsu_busy,			// this instruction stalls if LSU is busy
	input					id_instr_stalls_pipeline,
	input					s4_instr_stalls_pipeline_done,
	input					scalarToVectorQ_empty,			// scalar to vector transfer queue is empty
	input					id_scalarToVectorQ_rdreq,		// read request for scalar to vector transfer data queue
	input					s3_vectorToScalarQ_wrreq,
	input					vectorToScalarQ_full,			// vector to scalar transfer queue is full
	input	[`log2(NUMLANES)-1:0]		lsu_indexOffset_lanesel,	// select which index offset to use
	input					lsu_indexed_memaccess,			// input from LSU
	
	input					id_indexed_memaccess,			// input from ID
	input					controller_indexed_loadstore,
	input					load_done_posedge,
	
	// ************
	// ** Outputs **
	// ************
	output					vpu_instr_decode_valid,			// instruction decode is valid, can latch control signals
	output					cpu_fetch_stall,				// stall the fetch stage of the entire CPU
	output					vpu_decodestage_stall,			// read stage needs to stall (blocks next fetched instruction from issuing)
	output					vpu_decodestage_stop,			// stops decode stage completely (stops issue state machine)
	output					vpu_executestage_stall,			// execute 1 stage needs to stall
	output					vpu_memorystage_stall,			// memory stage needs to stall
	output					indexed_memaccess_stall
);

	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local Signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	vrf_raw_hazard;						// VRF read after write hazard
wire	vflag_raw_hazard;					// vflag RAW hazard, including reading vflag for mask values
wire	lsu_raw_hazard;						// LSU write, VRF read RAW hazard
reg		nonpipelined_instr_decode_stall;	// stall the decode stage due to execution of a non-pipelined instruction
wire	readScalarQ_stall;					// stall execute stage while reading from scalar queue
wire	lsu_busy_decodestall;
wire	vectorToScalarQ_full_stall;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Stall Logic
//---------------------------------------------------------------------------------------------------------------------------------------------------
// can issue this instruction if the new instruction is decoded and not stalled
assign	vpu_instr_decode_valid = vpu_instr_decoded & ~vpu_decodestage_stall & ~vpu_executestage_stall & ~vpu_memorystage_stall;

// stall execute stage if vector to scalar transfer queue is full
assign	vpu_executestage_stall = vpu_memorystage_stall | vectorToScalarQ_full_stall;
assign	vpu_memorystage_stall = 1'b0;

// when asserted, decode stage holds current instruction in IR and waits until the stall is deasserted before accepting the next instruction
// Stall the decode stage if register RAW hazard, the memory is stalled at decode, or there is a memory queue transfer pending
assign	vpu_decodestage_stall = (lsu_busy_decodestall
		// stall for non-pipelined instruction when the instruction reaches execute stage, or at decode if LSU is busy (id_instr_stalls_pipeline & lsu_busy)
		// also stall if the current instruction stalls pipeline, and there is interrupting instruction pending (id_instr_stalls_pipeline & int_pending_fetch_stall)
		| (id_instr_stalls_pipeline & (lsu_busy | int_pending_fetch_stall)) | nonpipelined_instr_decode_stall
		| vrf_raw_hazard | lsu_raw_hazard | vflag_raw_hazard
		| indexed_memaccess_stall
		| readScalarQ_stall)
		// if execute stall, do not assert decodestage_stall so pipeline registers before execute stage hold value
		& ~vpu_executestage_stall;

// when asserted, decodestage stops state machine and pipeline registers hold previous value
assign	vpu_decodestage_stop = vpu_executestage_stall;


// Stall the fetch stage if
// 1) the VPU is busy with a vector instruction (~vpu_control_ready)
// 2) or execution is stalled
// 3) there is data waiting to be transferred from load Q to VRF; next instruction will be interrupt transfer (int_pending_fetch_stall)
// 4) decode stage was stalled when entering interrupt instruction mode, need to re-decode the previous instruction held in decode stage (enter_interrupt_mode_stalled)
// and the next instruction is another vector instruction
assign	cpu_fetch_stall = (~vpu_control_ready | vpu_decodestage_stall | vpu_executestage_stall
		| int_pending_fetch_stall | enter_interrupt_mode_stalled
		// need to stall for indexed load store the cycle right before controller starts running IndexedLoadStore state
		| (id_indexed_memaccess & vpu_control_ready & ~controller_indexed_loadstore))
		& predecode_vector_instr & reset_n;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Detect different types of stalls
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Stall decode stage if this instruction needs to use LSU (id_stall_if_lsu_busy), and
// 1) LSU is busy, or
// 2) there is data waiting to be transferred from load queues to VRF (VPU_controller will interrupt execution and go into transfer instruction)
assign	lsu_busy_decodestall = ((id_stall_if_lsu_busy & vpu_instr_decoded) == 1'b1)
								&& ((lsu_busy == 1'b1) || (lsu_data_waiting_flag != `LSU_LOAD_DATA_EMPTY)) ? 1'b1 : 1'b0;

// Stall the processor to perform an indexed memory access
// need to unstall 2 cycles before NUMLANES because it takes an extra cycle to read from VRF after vrfOffset is changed
`ifdef		INDEXED_MEMORY_ACCESS
assign	indexed_memaccess_stall = (controller_indexed_loadstore & lsu_busy & lsu_indexed_memaccess & (lsu_indexOffset_lanesel != NUMLANES-2) & (~load_done_posedge));
`else
assign	indexed_memaccess_stall = 1'b0;
`endif

// Stall if vector to scalar queue full and trying to write
assign	vectorToScalarQ_full_stall = s3_vectorToScalarQ_wrreq & vectorToScalarQ_full;

// Wait until there is data in the scalar to vector transfer queue if instruction needs to read from the queue
assign	readScalarQ_stall = scalarToVectorQ_empty & id_scalarToVectorQ_rdreq & vpu_instr_decoded;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Detect RAW hazard
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Stall issue of next instruction if there is RAW hazard
// also handles RAW between register instruction writing and store instruction reading
assign	vrf_raw_hazard =
	((({s2_vrfRdOffset, id_vrf_rdaddr1} == {s4_vrfWrOffset, s4_vrf_wraddr}) && id_instr_reads_vrfrdaddr1 && dec_s4_vrf_wren) ||
	(({s2_vrfRdOffset, id_vrf_rdaddr2} == {s4_vrfWrOffset, s4_vrf_wraddr}) && id_instr_reads_vrfrdaddr2 && dec_s4_vrf_wren) ||
	(({s2_vrfRdOffset, id_vrf_rdaddr1} == {s3_vrfWrOffset, s3_vrf_wraddr}) && id_instr_reads_vrfrdaddr1 && dec_s3_vrf_wren) ||
	(({s2_vrfRdOffset, id_vrf_rdaddr2} == {s3_vrfWrOffset, s3_vrf_wraddr}) && id_instr_reads_vrfrdaddr2 && dec_s3_vrf_wren)) ? 1'b1 : 1'b0;

// RAW hazard for updating and using flags (even for masking)
assign	vflag_raw_hazard = 
	((({s2_vrfRdOffset, id_vflag_rdaddr1} == {s4_vrfWrOffset, s4_vflag_wraddr}) && id_instr_reads_vflagrdaddr1 && dec_s4_vflag_wren) ||
	(({s2_vrfRdOffset, id_vflag_rdaddr2} == {s4_vrfWrOffset, s4_vflag_wraddr}) && id_instr_reads_vflagrdaddr2 && dec_s4_vflag_wren) ||
	(({s2_vrfRdOffset, id_vflag_rdaddr1} == {s3_vrfWrOffset, s3_vflag_wraddr}) && id_instr_reads_vflagrdaddr1 && dec_s3_vflag_wren) ||
	(({s2_vrfRdOffset, id_vflag_rdaddr2} == {s3_vrfWrOffset, s3_vflag_wraddr}) && id_instr_reads_vflagrdaddr2 && dec_s3_vflag_wren)) ? 1'b1 : 1'b0;

// RAW hazard between load and use
assign	lsu_raw_hazard = (((id_vrf_rdaddr1 == lsu_vrf_wraddr) && id_instr_reads_vrfrdaddr1)
							|| ((id_vrf_rdaddr2 == lsu_vrf_wraddr) && id_instr_reads_vrfrdaddr2))
							&& ((lsu_executing_vector_load == 1'b1) || (int_pending_fetch_stall == 1'b1)) ? 1'b1 : 1'b0;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Generate flag to stall instruction decode while a non-pipelined instruction is executing
//---------------------------------------------------------------------------------------------------------------------------------------------------
always @(posedge clk, negedge reset_n) begin
	if (~reset_n)
		nonpipelined_instr_decode_stall <= 1'b0;
	// need to keep this order
	// if the instruction has completed, stop stalling
	else if (s4_instr_stalls_pipeline_done)
		nonpipelined_instr_decode_stall <= 1'b0;
	// if beginning an instruction that is non-pipelined, start stalling
	else if (id_instr_stalls_pipeline & vpu_instr_decode_valid)
		nonpipelined_instr_decode_stall <= 1'b1;
end

	
endmodule
