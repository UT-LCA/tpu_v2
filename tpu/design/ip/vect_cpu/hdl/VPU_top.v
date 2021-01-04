//************************************************************************
// Scalable Vector CPU Project -- Processing unit top level
// Filename: VPU_top.v
// Author: Jason Yu
//
// Top level of processing unit of the scalable vector coprocessor
// - contains only the processing core
// - memory unit is located in a different file
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


module	VPU_top	

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Customizable parameters
//---------------------------------------------------------------------------------------------------------------------------------------------------
#(	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1,
	parameter	NUMLANES = 1,
	parameter	MEM_WIDTH = 1,
	parameter	MINDATAWIDTH = 1
)
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Ports
//---------------------------------------------------------------------------------------------------------------------------------------------------
(
	// Inputs
	input						clk,
	input						reset_n,
	input						vpu_new_instr,
	input	[31:0]				opCode,						// vector instruction opcode
	input	[31:0]				scalarRegIn,				// Input from scalar unit for vector-scalar instructions

	// Input from memory interface
	input						lsu_executing_vector_load,	// executing vector load
	input						lsu_busy,
	input	[NUMLANES-1:0]		extLoad_lane_we,			// lane load data Q wren; decoupled from VPU normal operation
	input	[NUMLANES*VPU_WIDTH-1:0]	memLoadDataToLanes,	// data from memory load; from memory interface to lanes
	input	[NUMLANES-1:0]		laneStoreQRdreq,			// read request to store queue in lanes
	input						flagStoreQRdreq,			// read request to read flag queue for flag store instruction
	input	[2:0]				lsu_data_waiting_flag,		// Flag to indicate what type of data is waiting the load data transfer queue
	input	[`log2(`VRF_NREG)-1:0]		lsu_vrf_wraddr,
	input						lsu_vflag_reg,				// copy flag register from opcode for writing loaded data back to VRF
	input						scalarToVectorQ_empty,		// scalar to vector transfer queue is empty
	input						vectorToScalarQ_full,
	input	[`log2(NUMLANES)-1:0]		lsu_indexOffset_lanesel,	// select which index offset to use
	input						lsu_indexed_memaccess,
	input						load_done,					// asserted the last cycle of load

	// **** Outputs ****
	output						cpu_fetch_stall,			// stall the scalar CPU fetch stage
	output						scalarToVectorQ_rdreq,		// read request for scalar to vector transfer data queue
	// transfer queue from vector to scalar core
	output						vectorToScalarQ_wrreq,
	output	[31:0]				vectorToScalarQ_data,
	// Outputs to memory interface
	output						vputolsu_instr_decode_valid,	// loadstore unit can decode this instruction now
	output	[31:0]				lsu_opCode,					// portion of the opcode to loadstore controller
	output	[NUMLANES*VPU_WIDTH-1:0]	VLaneDataToMem,		// store data from lanes to memory
	output	[NUMLANES-1:0]		VLaneFlagToMem,				// flag from store data flag queue to memory
	output	[31:0]				vl_minus_one,				// vector length - 1
	output	[31:0]				id_vl_special_instr,
	output	[31:0]				vctrl_q,					// control register output
	output	[31:0]				vbase,						// vector base
	output						flagStoreQEmpty,			// whether the store flag data queue is empty
	output	[2:0]				id_memDataWidth,			// data width for memory access
	output						loadDataQ_full,				// the load data queue is full
	output	[VPU_WIDTH-1:0]		indexOffset_out,			// offset from lanes for indexed addressing
	output						id_clear_lsu_data_flag,		// clear the load data waiting flag
	output						lsu_wraddr_q_rdreq

	// Simulation trace signals
`ifdef	MODELSIM
	,
	output						id_newInstr_store
`endif
);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local parameters
//---------------------------------------------------------------------------------------------------------------------------------------------------
localparam	MVL_LIMIT = NUMLANES * `ELEMPERLANE;
// constant to divide VL by after a copy from accumulator instruction
localparam	ACCVL_DIVISOR = `MACCHAINLENGTH * `MAC_NUMMULT;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// **** Signals ****
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Signals from decode to controller
wire	[`log2(`VRF_NREG)-1:0]		id_vrf_rdaddr1;
wire	[`log2(`VRF_NREG)-1:0]		id_vrf_rdaddr2;
wire	[`log2(`FLAG_NREG)-1:0]		id_vflag_rdaddr1;
wire	[`log2(`FLAG_NREG)-1:0]		id_vflag_rdaddr2;
wire	dec_s4_vrf_wren;
wire	dec_s3_vrf_wren;
wire	dec_s3_vflag_wren;
wire	dec_s4_vflag_wren;
wire	s3_vstype;						// whether instruction is vs-type
wire	s4_vstype;
wire	[`ALUOP_WIDTH-1:0]	dec_s3_ALUOp;
wire	[`ALUOP_WIDTH-1:0]	dec_s4_ALUOp;
wire	s3_instr_nonmaskable;
wire	id_instr_stalls_pipeline;		// instruction stalls pipeline until completes
wire	s4_instr_stalls_pipeline_done;	// instruction that stalls pipeline is complete

// Signals from controller to datapath
wire	[`log2(`ELEMPERLANE)-1:0]	s2_vrfOffset;
wire	[`log2(`ELEMPERLANE)-1:0]	s2_vrfRdOffset;
wire	[`log2(`ELEMPERLANE)-1:0]	s3_vrfOffset;
wire	[`log2(`ELEMPERLANE)-1:0]	s3_vrfWrOffset;
wire	[`log2(`ELEMPERLANE)-1:0]	s4_vrfOffset;
wire	[`log2(`ELEMPERLANE)-1:0]	s4_vrfWrOffset;
wire	[`log2(`VRF_NREG)-1:0]		s3_vrf_wraddr, s4_vrf_wraddr;
wire	[`log2(`FLAG_NREG)-1:0]	s3_vflag_wraddr;
wire	[`log2(`FLAG_NREG)-1:0]	s4_vflag_wraddr;
wire	[2:0]	s4_vrfWriteSel;			// what source to write to VRF
wire	[1:0]	s4_vflagWriteSel;
wire	s3_scalarRegInSel;
wire	id_scalarToVectorQ_rdreq;		// read request for scalar to vector data queue
wire	[2:0]	s3_vectorToScalarQ_data_source;		// vector to scalar tranfer queue write data source
wire	s3_vectorToScalarQ_data_zext;	// zero extend vector to scalar transfer queue write data
wire	id_vectorToScalarQ_wrreq, s3_vectorToScalarQ_wrreq;

wire	id_instr_multiple_elems;
wire	id_instr_uses_vlength;
wire	vpu_control_ready;
wire	s4_multiplyHigh;
wire	id_signedOp;
wire	s3_signedOp, s4_signedOp;
wire	s3_indexed_memaccess;

// Memory interface
wire	s3_laneStoreQWrreq, id_clusterStoreQWrreq;		// share 1 wrreq signal, as the lanes operate in synchrony
wire	s3_loadDataQToVReg;	// transfers data from loaddataQ to vector register
wire	s3_loadDataQ_rdmaskable;	// the rdreq for transferring data from load data Q to VRF is maskable (for vins.vv)

// vector lane local memory signals; declare, but unconnected when not used
wire	dec_s3_vpmem_wren;
wire	s3_vpmem_rden;

// Control register signals
wire	s4_vc_vl_wren;		// vector control reg: vlength

// Vector control register
wire	[`log2(`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC)-1:0]	id_vctrl_rdaddr;
wire	[`log2(`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC)-1:0]	s4_vctrl_wraddr;
wire	s4_vctrl_wren;
wire	id_vctrl_rden;
wire	s3_vc_vctrl_rden;
// Vector control register: base register
wire	[`log2(`VC_NVBASE)-1:0]		id_vc_vbase_rdaddr;
wire	[`log2(`VC_NVBASE)-1:0]		s4_vc_vbase_wraddr;
wire	s4_vc_vbase_wren;
wire	id_vc_vbase_rden;
wire	s4_vbase_add_vinc;

// Accumulator signals
wire	s4_zeroMACchainout;				// zero the MAC chainout
wire	id_zeroACC;
wire	s3_zeroACC;
wire	s4_zeroACC;			// zero the accumulators

// Shifter signals
wire	s3_shift_sv;					// scalar-vector shift (vector data, scalar shift amount)
wire	s4_shift_instr;					// instruction is a shift instruction

// Processor stall signals
wire	vpu_decodestage_stall;			// read stage needs to stall
wire	vpu_decodestage_stop;			// decode stage needs to stop completely
wire	vpu_executestage_stall;			// execute 1 stage needs to stall
wire	vpu_memorystage_stall;			// memory stage needs to stall
wire	int_pending_fetch_stall;		// stall the decode stage due to pending interrupting instruction
wire	vpu_controller_stall_decode;	// controller needs to stall decode stage
wire	combined_decodestage_stall;
wire	indexed_memaccess_stall;

wire	predecode_vector_instr;			// predecoded a vector instruction
wire	alu_interrupt_instr;
wire	enter_interrupt_mode_stalled;			// decode stage was stalled when entering interrupt instruction mode; used to recover after

wire	id_stall_if_lsu_busy;
wire	id_multElemQuot_offset;					// instruction uses multielement quotient as vrf offset
wire	s3_vext_sv;

wire	[31:0]	instruction_register_q;				// register the instruction from instruction fetch
wire	[31:0]	s3_opCode;
wire	[31:0]	s4_opCode;

wire	[NUMLANES*VPU_WIDTH-1:0]	s3_indexMemOffset_group;	// offset for indexed memory access
wire	s3_decode_onelane_en;
wire	[NUMLANES-1:0]		s3_vl_lane_en;			// lane enable signal computed from vector length
wire	id_instr_reads_vrfrdaddr1;		// does instruction use the operand; for RAW hazard checking
wire	id_instr_reads_vrfrdaddr2;		// does isntruction use the operand; for RAW hazard checking
wire	id_instr_reads_vflagrdaddr1;	// does instruction use the operand; for RAW hazard checking
wire	id_instr_reads_vflagrdaddr2;	// does instruction use the operand; for RAW hazard checking
wire	id_indexed_memaccess;
wire	s4_vupshift;
wire	id_vind_plus_vl_vectorlength;			// uses a modified vector length of vectorlength = VL + vindex
wire	id_lsu_instr;					// instruction that requires the loadstore unit

wire	vpu_instr_decoded;
wire	vpu_instr_decode_valid;				// the decoded instruction is valid, can be passed down the pipeline
wire	id_flagStoreQWrreq;
wire	s3_flagStoreQWrreq;
wire	[31:0]	s4_scalarRegIn;
	
wire	[`log2(NUMLANES)-1:0]	cycleFirstLaneIndex, s3_cycleFirstLaneIndex;	// first lane to process in this cycle
wire	[`log2(NUMLANES):0]	cycleLastEnabledLane, s3_cycleLastEnabledLane;		// last lane to process in this cycle; extra bit (range = 1 to NUMLANE)
wire	id_vccacc_vl;
wire	[31:0]	vc_vl;										// vector length
wire	[31:0]	vl_eff_local;								// effective vector length
wire	[31:0]	vl_eff_local_sig;

wire	id_flagstore;
wire	[`log2(NUMLANES)-1:0]		indexOffsetMUX_sel;	// select which index offset to use
wire	s4_MAC_en;

wire	id_indexed_load;
wire	id_indexed_store;
wire	controller_indexed_loadstore;
wire			load_done_delayn;
wire			load_done_posedge;
reg		[31:0]	vl_eff_sel1;
wire	[31:0]	vl_eff_sel2;
wire	s4_vl_accvl_update;			// update VL after a copy from accumulator instruction

wire	[`log2(`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC)-1:0]	s3_vctrl_rdaddr;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Signal assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------
// don't start a LSU instruction when VPU is executing an interrupting instruction
assign	vputolsu_instr_decode_valid = id_lsu_instr & (~alu_interrupt_instr) & vpu_instr_decode_valid;
// transfer queue from vector to scalar core; if the queue is full, do not assert wrreq yet, hazard detection will stall the vpu
assign	vectorToScalarQ_wrreq = s3_vectorToScalarQ_wrreq & ~vectorToScalarQ_full;
// portion of opcode to loadstore unit
assign	lsu_opCode = instruction_register_q;
// read request for scalar to vector transfer data queue, asserted in ID stage
assign	scalarToVectorQ_rdreq = id_scalarToVectorQ_rdreq & vpu_instr_decode_valid;

// stall decode stage
assign	combined_decodestage_stall = vpu_decodestage_stall | vpu_controller_stall_decode;

// Register file read/write offset from grouping multiple elements in one memory
assign	s2_vrfRdOffset = s2_vrfOffset;
assign	s3_vrfWrOffset = s3_vrfOffset;
assign	s4_vrfWrOffset = s4_vrfOffset;

assign	id_indexed_memaccess = id_indexed_load | id_indexed_store;

// Pipeline register to balance pipeline stage for scalarRegIn	
ff	#(.WIDTH(32))	stage4_scalarRegIn	(.clk(clk), .clrn(reset_n), .en(1'b1), .d(scalarRegIn), .q(s4_scalarRegIn));

assign	load_done_posedge = load_done & load_done_delayn;
ff	#(.WIDTH(1))	delayn_load_done_posedge (.clk(clk), .en(1'b1), .clrn(reset_n), .d(~load_done), .q(load_done_delayn));


//---------------------------------------------------------------------------------------------------------------------------------------------------
// this signal is needed to send to LSU, but only valid for instructions that assert 
// flag store = (vl_minus_one / VPU_WIDTH) * VPU_WIDTH + VPU_WIDTH - 1
// vins.vv/vext.vv = VL - 1 + vindex 
//---------------------------------------------------------------------------------------------------------------------------------------------------

assign	id_vl_special_instr = vl_eff_local;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Set effective VL for VPU core:
// vins.vv/vext.vv = VL - 1 + vindex 
// vcczacc/vccacc = `MACCHAINLENGTH - 1
// flag store = (vl_minus_one / VPU_WIDTH) * VPU_WIDTH + VPU_WIDTH - 1
// all other instructions = VL - 1
//---------------------------------------------------------------------------------------------------------------------------------------------------

// limit the vector length so the processor doesn't hang when VL is set to an invalid value
assign	vl_eff_local = `min(vl_eff_local_sig, MVL_LIMIT-1);
assign	vl_minus_one = vc_vl-1;

assign	vl_eff_local_sig = vl_eff_sel1 + vl_eff_sel2;
assign	vl_eff_sel2 = (id_vind_plus_vl_vectorlength == 1'b1) ? vctrl_q : 0;

always @(*) begin
	case ({id_flagstore,id_vccacc_vl})
		2'b10:		vl_eff_sel1 = ((vl_minus_one / VPU_WIDTH) + 1) * VPU_WIDTH - 1;
`ifdef	CLUSTER_MULTACCUM
		// min(vc_vl,NUMLANES) because the accumulators accumulate multiple data elements within the same lane
		// so there are a maximum of NUMLANE values to reduce for vccacc
		2'b01:
			// accumulators accumulate multiple data elements within the same lane
			// so there are a maximum of NUMLANE values to reduce for vccacc
			if (vc_vl > NUMLANES)
				vl_eff_sel1 = (NUMLANES/ACCVL_DIVISOR)-1;
			// limit vl_eff at 0
			else if ((vc_vl/ACCVL_DIVISOR) == 0)
				vl_eff_sel1 = 0;
			// otherwise adjust VL
			else
				vl_eff_sel1 = (vc_vl/ACCVL_DIVISOR)-1;
`endif
		default:	vl_eff_sel1 = vl_minus_one;
	endcase
end


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Decode vector length into lane enable signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
vlane_enable_decode
#(	.NUMLANES		(NUMLANES)
)
vlane_enable_decode_inst
(
	.s3_decode_onelane_en			(s3_decode_onelane_en),					// decode only the last element
	.s3_cycleFirstLaneIndex			(s3_cycleFirstLaneIndex),
	.s3_cycleLastEnabledLane		(s3_cycleLastEnabledLane),		// remainder of dividing vlength/vindex
	.s3_vl_lane_en					(s3_vl_lane_en)			// lane enable signal computed from vector length	
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Select which data to write to vector to scalar core transfer queue
//---------------------------------------------------------------------------------------------------------------------------------------------------

vectorToScalarQ_data_sel
#(	.VPU_WIDTH		(VPU_WIDTH)	)
vectorToScalarQ_data_sel_inst
(
	// Inputs
	.vectorToScalarQ_data_source	(s3_vectorToScalarQ_data_source),		// vector to scalar tranfer queue write data source select
	.vectorToScalarQ_data_zext		(s3_vectorToScalarQ_data_zext),			// whether to zero extend
	.vc_vbase_q					(vbase),
	.vctrl_q					(vctrl_q),
	.vc_vl_q					(vc_vl),
	.indexOffset_out			(indexOffset_out),
	// Outputs
	.vectorToScalarQ_data		(vectorToScalarQ_data)
);


// --------------------------------------------------------------------------------------------------------------------------------------------------
// MUX to select the index offsets supplied from vector lanes
// - the index offset queues are connected to output port A of VRF
// --------------------------------------------------------------------------------------------------------------------------------------------------
assign	indexOffsetMUX_sel = (s3_vext_sv == 1'b1) ? s3_cycleFirstLaneIndex : lsu_indexOffset_lanesel;

muxMto1
#(	.N	(VPU_WIDTH),
	.M	(NUMLANES),
	.S	(`log2(NUMLANES))
)
indexOffsetMUX
(	.D				(s3_indexMemOffset_group),
	.SEL			(indexOffsetMUX_sel),
	.Z				(indexOffset_out)
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Predecode some signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
VPU_predecode	VPU_predecode_inst
(
	.opCode					(opCode),
	.predecode_vector_instr	(predecode_vector_instr),
	.predecode_load_instr	(),	// predecoded a load instruction
	.predecode_store_instr	(),
	.predecode_loadstore	()	// predecoded load or store instruction
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Hazard detection
//---------------------------------------------------------------------------------------------------------------------------------------------------
VPU_hazard_detection
#(	.NUMLANES				(NUMLANES)
)
VPU_hazard_detection_inst
(
	.clk							(clk),
	.reset_n						(reset_n),
	.vpu_control_ready				(vpu_control_ready),
	.vpu_instr_decoded				(vpu_instr_decoded),
	.predecode_vector_instr			(predecode_vector_instr),	// predecoded a vector instruction
	.lsu_executing_vector_load		(lsu_executing_vector_load),
	.id_indexed_memaccess			(id_indexed_memaccess),
	.s2_vrfRdOffset					(s2_vrfRdOffset),			// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
	.id_vrf_rdaddr1					(id_vrf_rdaddr1),
	.id_vrf_rdaddr2					(id_vrf_rdaddr2),
	.id_vflag_rdaddr1				(id_vflag_rdaddr1),
	.id_vflag_rdaddr2				(id_vflag_rdaddr2),
	.s3_vrfWrOffset					(s3_vrfWrOffset),
	.s4_vrfWrOffset					(s4_vrfWrOffset),			// offset to VRF and vflag register files; handle 
	.s3_vrf_wraddr					(s3_vrf_wraddr),
	.s4_vrf_wraddr					(s4_vrf_wraddr),
	.s3_vflag_wraddr				(s3_vflag_wraddr),
	.s4_vflag_wraddr				(s4_vflag_wraddr),
	.lsu_vrf_wraddr					(lsu_vrf_wraddr),
	.id_instr_reads_vrfrdaddr1		(id_instr_reads_vrfrdaddr1),		// does instruction use the operand; for RAW hazard checking
	.id_instr_reads_vrfrdaddr2		(id_instr_reads_vrfrdaddr2),		// does isntruction use the operand; for RAW hazard checking
	.id_instr_reads_vflagrdaddr1	(id_instr_reads_vflagrdaddr1),
	.id_instr_reads_vflagrdaddr2	(id_instr_reads_vflagrdaddr2),
	.lsu_data_waiting_flag			(lsu_data_waiting_flag),				// Flag to indicate what type of data is waiting the load data transfer queue
	.enter_interrupt_mode_stalled	(enter_interrupt_mode_stalled),
	.int_pending_fetch_stall		(int_pending_fetch_stall),
	.lsu_busy						(lsu_busy),
	.id_stall_if_lsu_busy			(id_stall_if_lsu_busy),
	.dec_s4_vrf_wren				(dec_s4_vrf_wren),
	.dec_s3_vrf_wren				(dec_s3_vrf_wren),
	.dec_s3_vflag_wren				(dec_s3_vflag_wren),
	.dec_s4_vflag_wren				(dec_s4_vflag_wren),

	.id_instr_stalls_pipeline		(id_instr_stalls_pipeline),
	.s4_instr_stalls_pipeline_done	(s4_instr_stalls_pipeline_done),
	.scalarToVectorQ_empty			(scalarToVectorQ_empty),	// scalar to vector transfer queue is empty
	.id_scalarToVectorQ_rdreq		(id_scalarToVectorQ_rdreq),

	.s3_vectorToScalarQ_wrreq		(s3_vectorToScalarQ_wrreq),
	.vectorToScalarQ_full			(vectorToScalarQ_full),				// vector to scalar transfer queue is full
	.lsu_indexOffset_lanesel		(lsu_indexOffset_lanesel),
	.lsu_indexed_memaccess			(lsu_indexed_memaccess),
	.controller_indexed_loadstore	(controller_indexed_loadstore),
	.load_done_posedge				(load_done_posedge),
	
	// ************
	// ** Outputs **
	// ************
	.vpu_instr_decode_valid			(vpu_instr_decode_valid),
	.cpu_fetch_stall				(cpu_fetch_stall),			// stall the cpu fetch stage
	.vpu_decodestage_stop			(vpu_decodestage_stop),
	.vpu_decodestage_stall			(vpu_decodestage_stall),	// read stage needs to stall
	.vpu_executestage_stall			(vpu_executestage_stall),	// execute 1 stage needs to stall
	.vpu_memorystage_stall			(vpu_memorystage_stall),		// memory stage needs to stall
	.indexed_memaccess_stall		(indexed_memaccess_stall)
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Decode
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Vector instruction decode
VPU_decode 	VPU_decode_inst
(
	// ---- Inputs ----
	.opCode							(instruction_register_q),
	.lsu_indexed_memaccess			(lsu_indexed_memaccess),

	// ---- Outputs ----
	.id_instr_multiple_elems			(id_instr_multiple_elems),
	.id_instr_uses_vlength				(id_instr_uses_vlength),
	.id_instr_stalls_pipeline		(id_instr_stalls_pipeline),	// instruction stalls pipeline until completes
	// vector to scalar core transfer queue data source
	.id_vectorToScalarQ_wrreq		(id_vectorToScalarQ_wrreq),
	.id_scalarToVectorQ_rdreq		(id_scalarToVectorQ_rdreq),
	// control registers
	.id_vctrl_rdaddr				(id_vctrl_rdaddr),			// Vector control registers
	.id_vctrl_rden					(id_vctrl_rden),
	.id_vc_vbase_rdaddr				(id_vc_vbase_rdaddr),		// Vector control register: base register
	.id_vc_vbase_rden				(id_vc_vbase_rden),

	// Vector local memory; leave unconnected when unused
	.id_vpmem_rden					(),
	
	// Memory interface
	.id_signedOp					(id_signedOp),
	
`ifdef	MODELSIM
	// Simulation trace signals
	.id_newInstr_store				(id_newInstr_store),
`else
	.id_newInstr_store				(),
`endif
	.id_lsu_instr					(id_lsu_instr),
	// Vector register file
	.id_vrf_rdaddr1					(id_vrf_rdaddr1),
	.id_vrf_rdaddr2					(id_vrf_rdaddr2),
	.id_instr_reads_vrfrdaddr1		(id_instr_reads_vrfrdaddr1),		// does instruction use the operand; for RAW hazard checking
	.id_instr_reads_vrfrdaddr2		(id_instr_reads_vrfrdaddr2),		// does isntruction use the operand; for RAW hazard checking
	.id_instr_reads_vflagrdaddr1	(id_instr_reads_vflagrdaddr1),		// does instruction use the operand; for RAW hazard checking
	.id_instr_reads_vflagrdaddr2	(id_instr_reads_vflagrdaddr2),		// does instruction use the operand; for RAW hazard checking
	// Vector flag register
	.id_vflag_rdaddr1				(id_vflag_rdaddr1),
	.id_vflag_rdaddr2				(id_vflag_rdaddr2),
	
	.id_clusterStoreQWrreq			(id_clusterStoreQWrreq),
	.id_clear_lsu_data_flag			(id_clear_lsu_data_flag),
	.id_stall_if_lsu_busy			(id_stall_if_lsu_busy),
	.id_multElemQuot_offset			(id_multElemQuot_offset),
	.id_flagstore					(id_flagstore),
	.id_flagStoreQWrreq				(id_flagStoreQWrreq),
	.id_memDataWidth				(id_memDataWidth),
	.id_zeroACC						(id_zeroACC),
	.id_vind_plus_vl_vectorlength	(id_vind_plus_vl_vectorlength),
	.id_vccacc_vl					(id_vccacc_vl),
	.id_indexed_load				(id_indexed_load),
	.id_indexed_store				(id_indexed_store)
);


// Stage 3 combinational control signal decode
VPU_decode_s3	VPU_decode_s3_inst
(
	// Input
	.s3_opCode						(s3_opCode),
	// Outputs
	.s3_vctrl_rdaddr				(s3_vctrl_rdaddr),
	.s3_vc_vctrl_rden				(s3_vc_vctrl_rden),
	.s3_vstype						(s3_vstype),
	.s3_ALUOp						(dec_s3_ALUOp),
	.s3_instr_nonmaskable			(s3_instr_nonmaskable),
	.s3_vrf_wraddr					(s3_vrf_wraddr),
	.s3_vflag_wraddr				(s3_vflag_wraddr),
	.s3_loadDataQToVReg				(s3_loadDataQToVReg),
	.s3_loadDataQ_rdmaskable		(s3_loadDataQ_rdmaskable),
	.s3_zeroMACchainout				(),
	.s3_zeroACC						(s3_zeroACC),
	.s3_vext_sv						(s3_vext_sv),
	.s3_scalarRegInSel				(s3_scalarRegInSel),
	.s3_signedOp					(s3_signedOp),
	.s3_indexed_memaccess			(s3_indexed_memaccess),
	.s3_decode_onelane_en			(s3_decode_onelane_en),
	.s3_vectorToScalarQ_data_source	(s3_vectorToScalarQ_data_source),
	.s3_vectorToScalarQ_data_zext	(s3_vectorToScalarQ_data_zext),
	.s3_vpmem_wren					(dec_s3_vpmem_wren),
	.s3_vrf_wren					(dec_s3_vrf_wren),
	.s3_vflag_wren					(dec_s3_vflag_wren),
	.s3_vpmem_rden					(s3_vpmem_rden),
	.s3_shift_sv					(s3_shift_sv),
	.s3_MAC_en						()
);


// Stage 4 combinational control signal decode
VPU_decode_s4	VPU_decode_s4_inst
(
	// Inputs
	.s4_opCode						(s4_opCode),
	// Outputs
	.s4_vbase_add_vinc				(s4_vbase_add_vinc),
	.s4_vstype						(s4_vstype),
	.s4_ALUOp						(dec_s4_ALUOp),
	.s4_vrf_wraddr					(s4_vrf_wraddr),
	.s4_vc_vl_wren					(s4_vc_vl_wren),
	.s4_vctrl_wren					(s4_vctrl_wren),
	.s4_vc_vbase_wren				(s4_vc_vbase_wren),
	.s4_vflag_wraddr				(s4_vflag_wraddr),
	.s4_signedOp					(s4_signedOp),
	.s4_multiplyHigh				(s4_multiplyHigh),
	.s4_vrfWriteSel					(s4_vrfWriteSel),
	.s4_vflagWriteSel				(s4_vflagWriteSel),
	.s4_shift_instr					(s4_shift_instr),
	.s4_vctrl_wraddr				(s4_vctrl_wraddr),
	.s4_vc_vbase_wraddr				(s4_vc_vbase_wraddr),
	.s4_vrf_wren					(dec_s4_vrf_wren),
	.s4_vflag_wren					(dec_s4_vflag_wren),
	.s4_vupshift					(s4_vupshift),
	.s4_zeroACC						(s4_zeroACC),
	.s4_MAC_en						(s4_MAC_en),
	.s4_zeroMACchainout				(s4_zeroMACchainout),
	.s4_vl_accvl_update				(s4_vl_accvl_update)			// update VL after a copy from accumulator instruction
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Control Registers
//---------------------------------------------------------------------------------------------------------------------------------------------------
// VPU control registers
VPU_controlreg
#(	.VPU_WIDTH					(VPU_WIDTH),
	.NUMLANES					(NUMLANES),
	//synthesis translate_off
	.CONTROL_REG_INIT_FILE 		("control_reg_other.dat")
	//synthesis translate_on

	//synthesis read_comments_as_HDL on
	//.CONTROL_REG_INIT_FILE 		("control_reg_other.mif")
	//synthesis read_comments_as_HDL off
)
VPU_controlreg_inst (
	.clk							(clk),
	.reset_n						(reset_n),
	.scalarDataIn					(s4_scalarRegIn),			// Input from scalar core, for writing to control registers
	.vpu_instr_decode_valid			(vpu_instr_decode_valid),
	.s4_vbase_add_vinc				(s4_vbase_add_vinc),
	.s4_vc_vl_wren					(s4_vc_vl_wren),
	.s4_vl_accvl_update				(s4_vl_accvl_update),
	
	.id_vctrl_rdaddr				(id_vctrl_rdaddr),
	.id_vctrl_rden					(id_vctrl_rden),
	.s4_vctrl_wraddr				(s4_vctrl_wraddr),
	.s4_vctrl_wren					(s4_vctrl_wren),
	.s3_vc_vctrl_rden				(s3_vc_vctrl_rden),
	.s3_vctrl_rdaddr				(s3_vctrl_rdaddr),

	// Vector control register: base register
	.id_vc_vbase_rdaddr				(id_vc_vbase_rdaddr),
	.id_vc_vbase_rden				(id_vc_vbase_rden),
	.s4_vc_vbase_wraddr				(s4_vc_vbase_wraddr),
	.s4_vc_vbase_wren				(s4_vc_vbase_wren),
	.s4_vrfOffset					(s4_vrfOffset),

	// outputs
	.vc_vbase_q						(vbase),
	.vctrl_q						(vctrl_q),
	.vc_vl_q						(vc_vl)
);
	
	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Controller
// - controls execution of multi-cycle instructions
//---------------------------------------------------------------------------------------------------------------------------------------------------
// VPU main controller
VPU_controller
#(	.NUMLANES						(NUMLANES)
)
VPU_controller_inst
(
	// ---- Inputs ----
	.clk							(clk),
	.reset_n						(reset_n),
	.opCode							(opCode),					// vector instruction opcode
	.lsu_vrf_wraddr					(lsu_vrf_wraddr),			// wraddr for the executed load instruction
	.lsu_vflag_reg					(lsu_vflag_reg),			// copy flag register from opcode for writing loaded data back to VRF
	.vpu_new_instr					(vpu_new_instr),			// a new instruction from the scalar CPU
	.id_instr_multiple_elems		(id_instr_multiple_elems),
	.id_instr_uses_vlength			(id_instr_uses_vlength),
	.vpu_decodestage_stall			(vpu_decodestage_stall),
	.vpu_decodestage_stop			(vpu_decodestage_stop),
	.indexed_memaccess_stall		(indexed_memaccess_stall),
	.id_indexed_memaccess			(id_indexed_memaccess),
	.vl_eff							(vl_eff_local),
	.id_multElemQuot_offset			(id_multElemQuot_offset),
	.id_vctrl_vectorlength			(1'b0),						// no instructions use this anymore; but could be useful in the future
	.vctrl_q						(vctrl_q),
	.id_indexed_load				(id_indexed_load),
	.id_indexed_store				(id_indexed_store),
	.lsu_data_waiting_flag			(lsu_data_waiting_flag),	// Flag to indicate what type of data is waiting the load data transfer queue
	.load_done_posedge				(load_done_posedge),				// asserted the last cycle of load

	//---------------------------------------------
	// Outputs
	//---------------------------------------------
	.vpu_instr_decoded				(vpu_instr_decoded),		// a new instruction is decoded
	.instruction_register_q			(instruction_register_q),	// instruction register that is valid for entire duration of instruction
	.alu_interrupt_instr			(alu_interrupt_instr),
	.enter_interrupt_mode_stalled	(enter_interrupt_mode_stalled),
	.int_pending_fetch_stall		(int_pending_fetch_stall),	// stall decode stage due to pending int
	.cycleFirstLaneIndex			(cycleFirstLaneIndex),		// first lane to process in this cycle
	.cycleLastEnabledLane			(cycleLastEnabledLane),		// last lane to process in this cycle
	.vpu_control_ready				(vpu_control_ready),		// controller is ready for next instruction
	.vpu_controller_stall_decode	(vpu_controller_stall_decode),	// stall decode stage due to executing special instructions (read vctrl, etc)
	.s2_vrfOffset					(s2_vrfOffset),
	.controller_indexed_loadstore	(controller_indexed_loadstore),
	.lsu_wraddr_q_rdreq				(lsu_wraddr_q_rdreq)
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// **** Vector Lanes ****
//---------------------------------------------------------------------------------------------------------------------------------------------------
VLane_group	
#(	.VPU_WIDTH					(VPU_WIDTH),
	.NUMLANES					(NUMLANES)
)
VLane_group_inst
(
	//**********
	// Inputs
	//**********
	.clk							(clk),
	.reset_n						(reset_n),
	.scalarRegIn					(scalarRegIn),		// scalar register input
	.s3_vl_lane_en					(s3_vl_lane_en),		// lane enable signal computed from vector length
	.s3_instr_nonmaskable			(s3_instr_nonmaskable),
	
	// Vector register file
	.s2_vrfRdOffset					(s2_vrfRdOffset),	// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
	.s3_vrfOffset					(s3_vrfOffset),
	.s4_vrfWrOffset					(s4_vrfWrOffset),	// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
	.id_vrf_rdaddr1					(id_vrf_rdaddr1),
	.id_vrf_rdaddr2					(id_vrf_rdaddr2),
	.s4_vrf_wraddr					(s4_vrf_wraddr),
	.id_vrf_rden					(1'b1),
	.dec_s4_vrf_wren				(dec_s4_vrf_wren),
	
	// vector flag register
	.id_vflag_rdaddr1				(id_vflag_rdaddr1),
	.id_vflag_rdaddr2				(id_vflag_rdaddr2),
	.s4_vflag_wraddr				(s4_vflag_wraddr),
	// .id_vflag_rden					(1'b1),
	.dec_s4_vflag_wren				(dec_s4_vflag_wren),
	.s4_vflagWriteSel				(s4_vflagWriteSel),
	
	// Interface to memory
	.memLoadDataToLanes				(memLoadDataToLanes),
	.extLoad_lane_we				(extLoad_lane_we),
	.s3_loadDataQToVReg				(s3_loadDataQToVReg),		// transfers data from loaddataQ to vector register
	.s3_loadDataQ_rdmaskable		(s3_loadDataQ_rdmaskable),
	.laneStoreQRdreq				(laneStoreQRdreq),
	.s3_laneStoreQWrreq			(s3_laneStoreQWrreq),	// share 1 wrreq signal
	.flagStoreQRdreq				(flagStoreQRdreq),
	.s3_flagStoreQWrreq				(s3_flagStoreQWrreq),

	// **** Control inputs ****
	// Control inputs directly from decoder
	.s3_vstype						(s3_vstype),
	.s4_vstype						(s4_vstype),
	.dec_s3_ALUOp					(dec_s3_ALUOp),
	.dec_s4_ALUOp					(dec_s4_ALUOp),
	.id_signedOp					(id_signedOp),
	.s3_signedOp					(s3_signedOp),
	.s4_signedOp					(s4_signedOp),
	.s3_scalarRegInSel				(s3_scalarRegInSel),
	.s4_multiplyHigh				(s4_multiplyHigh),
	.s4_vrfWriteSel					(s4_vrfWriteSel),		// what source to write to VRF
	.s4_vupshift					(s4_vupshift),

	// Local memory
	.s3_vpmem_rden					(s3_vpmem_rden),
	.dec_s3_vpmem_wren				(dec_s3_vpmem_wren),		// local memory wren
	
`ifdef	CLUSTER_MULTACCUM
	.s4_MAC_en						(s4_MAC_en),
	.id_zeroACC						(id_zeroACC),
	.s3_zeroACC						(s3_zeroACC),
	.s4_zeroMACchainout				(s4_zeroMACchainout),	// zeros the chainout signal between MAC units
`endif

	// Shifter signals
	.s3_shift_sv					(s3_shift_sv),
	.s4_shift_instr					(s4_shift_instr),			// instruction is a shift instruction

	//**********
	// Outputs
	//**********
	.VLaneDataToMem					(VLaneDataToMem),
	.VLaneFlagToMem					(VLaneFlagToMem),			// flag from store data flag queue to memory
	.s3_indexMemOffset_group		(s3_indexMemOffset_group),	// offset for indexed memory access
	.loadDataQ_full					(loadDataQ_full),
	.flagStoreQEmpty				(flagStoreQEmpty)			// whether the store flag data queue is empty
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Pipeline registers
//---------------------------------------------------------------------------------------------------------------------------------------------------

VPU_pipeline_regs
#(	.NUMLANES						(NUMLANES)
)
VPU_pipeline_regs_inst
(
	// ---- Inputs ----
	.clk							(clk),
	.reset_n						(reset_n),
	.instruction_register_q			(instruction_register_q),
	.vpu_instr_decode_valid			(vpu_instr_decode_valid),
	.vpu_control_ready				(vpu_control_ready),
	.vpu_decodestage_stall			(combined_decodestage_stall),
	.vpu_executestage_stall			(vpu_executestage_stall),	// execute 1 stage needs to stall
	.vpu_memorystage_stall			(vpu_memorystage_stall),	// memory stage needs to stall
	
	// Inputs from instruction decode
	.id_instr_stalls_pipeline		(id_instr_stalls_pipeline),
	.id_vectorToScalarQ_wrreq		(id_vectorToScalarQ_wrreq),
	
	// VLane control inputs
	.id_clusterStoreQWrreq			(id_clusterStoreQWrreq),
	.id_flagStoreQWrreq				(id_flagStoreQWrreq),
	.s2_vrfOffset					(s2_vrfOffset),
	.cycleLastEnabledLane			(cycleLastEnabledLane),
	.cycleFirstLaneIndex			(cycleFirstLaneIndex),
	
	//---------------------------------------------
	// Outputs
	//---------------------------------------------
	.s3_opCode						(s3_opCode),
	.s4_opCode						(s4_opCode),
	.s3_cycleLastEnabledLane		(s3_cycleLastEnabledLane),
	.s3_cycleFirstLaneIndex			(s3_cycleFirstLaneIndex),
	
	// Control outputs to datapath
	.s4_instr_stalls_pipeline_done	(s4_instr_stalls_pipeline_done),

	// vector register file
	.s3_vrfOffset					(s3_vrfOffset),		// for stall logic
	.s4_vrfOffset					(s4_vrfOffset),

	// Memory interface
	.s3_clusterStoreQWrreq			(s3_laneStoreQWrreq),
	.s3_flagStoreQWrreq				(s3_flagStoreQWrreq),
	.s3_vectorToScalarQ_wrreq		(s3_vectorToScalarQ_wrreq)
);

endmodule
