//************************************************************************
// Scalable Vector CPU Project -- Top level for a vector lane
// Filename: VLane_top.v
// Author: Jason Yu
//
// Top level for a single vector lane
// - contains the datapath, controller
// - contains load and store queues
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


module VLane_top
#(	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1
)
(
	//**********
	// Inputs
	//**********
	input						clk,
	input						reset_n,
	input						s3_instr_nonmaskable,
	input						s3_vl_lane_en,				// lane enable signal computed from vector length
	
	// Cluster multiply-accumulate result
`ifdef	CLUSTER_MULTACCUM
	input	[VPU_WIDTH-1:0]		clusterMACResult,			// clsuter MAC result
`endif

	input	[31:0]				scalarRegIn,				// Input from scalar core
	input						s3_scalarRegInIsZero,
	input	[VPU_WIDTH-1:0]		shiftInNext,				// shift chain
	//input	[VPU_WIDTH-1:0]	shiftInPrev, 

	// Vector register file
	input	[`log2(`ELEMPERLANE)-1:0]	s2_vrfRdOffset,		// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
	input	[`log2(`ELEMPERLANE)-1:0]	s3_vrfOffset,
	input	[`log2(`ELEMPERLANE)-1:0]	s4_vrfWrOffset,		// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
	input	[`log2(`VRF_NREG)-1:0]		id_vrf_rdaddr1,
	input	[`log2(`VRF_NREG)-1:0]		id_vrf_rdaddr2,
	input	[`log2(`VRF_NREG)-1:0]		s4_vrf_wraddr,
	input						id_vrf_rden,
	// Vector flag register
	input  [`log2(`FLAG_NREG)-1:0]    s4_vflag_wraddr,
	input  [`log2(`FLAG_NREG)-1:0]    id_vflag_rdaddr1,
	input  [`log2(`FLAG_NREG)-1:0]    id_vflag_rdaddr2,
	// input						id_vflag_rden,
	input						s3_vstype,					// whether instruction is vs-type
	input						s4_vstype,
	input	[`ALUOP_WIDTH-1:0]	dec_s3_ALUOp,				// ALU operation
	input	[`ALUOP_WIDTH-1:0]	dec_s4_ALUOp,
	input						dec_s3_vpmem_wren,			// local memory wren
	input						dec_s4_vrf_wren,			// vrf write enable from instruction decoder
	input						dec_s4_vflag_wren,			// vflag write enable from instruction decoder
	input	[1:0]				s4_vflagWriteSel,			// which source to write to vflag
	input	[VPU_WIDTH-1:0]		extLoadData,				// memory data
	input						extLoad_lane_we,			// write enable for external memory load
	input						s3_loadDataQToVReg,			// transfers data from loaddataQ to vector register
	input						s3_loadDataQ_rdmaskable,	// the rdreq for transferring data from load data Q to VRF is maskable (for vins.vv)
	input						s3_vpmem_rden,				// local memory rden
	input						s3_scalarRegInSel,
	input						s4_multiplyHigh,			// multiplication: multiply upper half, or multiply full and store upper half
	input						s3_signedOp,				// signed operation
	input						s4_signedOp,
	input	[2:0]				s4_vrfWriteSel,				// what source to write to VRF
	
	// Shifter signals
	input						s3_shift_sv,				// scalar-vector shift (vector data, scalar shift amount)
	input						s4_shift_instr,				// instruction is a shift instruction

	// Store queues
	input		laneStoreQRdreq,
	input		s3_laneStoreQWrreq,
	input		flagStoreQRdreq,
	input		s3_flagStoreQWrreq,
	
	//**********
	// Outputs
	//**********
	// Signals to resources shared between multiple lanes
	output	[VPU_WIDTH-1:0]	s3_vrf_q1,					// output from register file
	output	[VPU_WIDTH-1:0]	s3_scalarSelOut,				// output of scalar register data select MUX
	output						loadDataQ_full,				// load data queue is full
	output						s3_vflag,
	output	[VPU_WIDTH-1:0]	s3_indexMemOffset,			// offset to base address for indexed mem access

	output						laneStoreQEmpty,
	output	[VPU_WIDTH-1:0]	VLaneDataToMem,				// data to store to external memory
	output						laneStoreQFull,
	output						VLaneFlagToMem,
	output						flagStoreQEmpty,

	output	[VPU_WIDTH-1:0]	s3_shiftOut					// shift chain data
);


//---------------------------------------------------------------------------------------------------------------------------------------------------	
// **** Wires ****
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	[`ALUOP_WIDTH-1:0]		s3_ALUOp;
wire	[`ALUOP_WIDTH-1:0]		s4_ALUOp;
wire							s3_vpmem_wren;
wire							s4_vrf_wren;
wire							s4_vflag_wren;
wire							s4_alu_vflag;
wire	[`log2(VPU_WIDTH)-1:0]	s4_stageShift;			// select whether to shift for the particular stage
wire	[VPU_WIDTH-1:0]			s4_logicalMask;			// mask that has zeros in positions that need to be zero'd for logical shifting;
														// used in arith shift as well
wire	[`log2(VPU_WIDTH)-1:0]	s3_shiftAmount;			// shift amount
wire	[`log2(VPU_WIDTH)-1:0]	s4_shiftAmount;			// shift amount
wire	[VPU_WIDTH-1:0]			s3_vrf_q2;				// output from register file

wire	[VPU_WIDTH-1:0]			s3_extStoreData;		// data to store to external memory
wire							loadDataQ_rdreq;
wire	[VPU_WIDTH-1:0]			loadDataQ;				// output from data Q for storing loaded data
wire	[VPU_WIDTH-1:0]			loadDataQ_data;

//---------------------------------------------------------------------------------------------------------------------------------------------------	
// Signal assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------
assign	s3_extStoreData = s3_vrf_q2;					// vB specifies the data to be stored
assign	s3_indexMemOffset = s3_vrf_q1;					// offset to base address for indexed memory access
assign	loadDataQ_rdreq = (~s3_loadDataQ_rdmaskable & s3_loadDataQToVReg) | (s3_loadDataQ_rdmaskable & s3_vl_lane_en & s3_loadDataQToVReg);

	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// **** Vector lane datapath ****
//---------------------------------------------------------------------------------------------------------------------------------------------------
	
VLane_datapath
#(	.VPU_WIDTH				(VPU_WIDTH)	)
VLane_dp_inst
(
	//**********
	// Inputs
	//**********
	.clk					(clk),
	.reset_n				(reset_n),
	.scalarRegIn			(scalarRegIn),		// scalar register input
	.s3_scalarRegInIsZero	(s3_scalarRegInIsZero),
	.shiftInNext			(shiftInNext),		// shift chain
	// .shiftInPrev		(shiftInPrev),
	.extLoadData			(extLoadData),		// memory load data
	.s4_alu_vflag				(s4_alu_vflag),
	
	// Vector register file
	.s2_vrfRdOffset			(s2_vrfRdOffset),	// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
	.s3_vrfOffset			(s3_vrfOffset),
	.s4_vrfWrOffset			(s4_vrfWrOffset),	// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
	.id_vrf_rdaddr1			(id_vrf_rdaddr1),
	.id_vrf_rdaddr2			(id_vrf_rdaddr2),
	.id_vrf_rden			(id_vrf_rden),
	.s4_vrf_wraddr			(s4_vrf_wraddr),
	.s4_vrf_wren			(s4_vrf_wren),
	.s4_vflagWriteSel		(s4_vflagWriteSel),

	// vector flag register
	// .id_vflag_rden			(id_vflag_rden),
	.s4_vflag_wren			(s4_vflag_wren),
	.s4_vflag_wraddr		(s4_vflag_wraddr),
	.id_vflag_rdaddr1		(id_vflag_rdaddr1),
	.id_vflag_rdaddr2		(id_vflag_rdaddr2),

	// **** Control inputs ****
	.s3_scalarRegInSel		(s3_scalarRegInSel),
	.s3_vstype				(s3_vstype),
	.s4_vstype				(s4_vstype),
	.s3_ALUOp				(s3_ALUOp),
	.s4_ALUOp				(s4_ALUOp),
	.s4_multiplyHigh		(s4_multiplyHigh),		// multiplication: multiply upper half, or multiply full and store upper half
	.s3_signedOp			(s3_signedOp),			// signed operation
	.s4_vrfWriteSel			(s4_vrfWriteSel),		// what source to write to VRF
	.s4_signedOp			(s4_signedOp),

	// VP local memory
	.s3_vpmem_wren			(s3_vpmem_wren),		// local memory wren
	.s3_vpmem_rden			(s3_vpmem_rden),		// local memory rden

`ifdef	CLUSTER_MULTACCUM
	.clusterMACResult		(clusterMACResult),		// cluster MAC unit result
`endif

	// Shifter signals
	.s4_stageShift			(s4_stageShift),		// whether to shift each stage
	.s4_logicalMask			(s4_logicalMask),		// mask for logical shift

	.loadDataQ				(loadDataQ),
	
	//**********
	// Outputs
	//**********
	// signals to resources shared between multiple lanes
	.s3_vrf_q1				(s3_vrf_q1),
	.s3_vrf_q2				(s3_vrf_q2),
	.scalarSelOut			(s3_scalarSelOut),
	.s3_shiftOut			(s3_shiftOut),
	// .loadDataQ_full			(loadDataQ_full),
	.loadDataQ_data			(loadDataQ_data),
	.s3_vflag				(s3_vflag)
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Module to generate control signals to the vector lane bit shifter
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Select source for control signals to the vector lane bit shifter
assign	s3_shiftAmount = (s3_shift_sv == 1'b1) ? scalarRegIn [`log2(VPU_WIDTH)-1:0] : s3_vrf_q1[`log2(VPU_WIDTH)-1:0];

// register the shift amount input from output 2 of register file
ff	#(.WIDTH(`log2(VPU_WIDTH)))
stage4_shiftAmount
(	.clk					(clk),
	.en						(1'b1),
	.clrn					(reset_n),
	.d						(s3_shiftAmount),
	.q						(s4_shiftAmount)
);

// Generates shifter control signals
shiftdecode	
#(	.VPU_WIDTH				(VPU_WIDTH)
)
shifterdecode
(
	.s4_shift_instr			(s4_shift_instr),		// instruction is a shift instruction
	.s4_shift_command		(dec_s4_ALUOp[2:0]),
	.s4_shiftAmount			(s4_shiftAmount),		// shift amount
	
	.s4_stageShift			(s4_stageShift),		// select whether to shift for the particular stage
	.s4_logicalMask			(s4_logicalMask)		// mask that has zeros in positions that need to be zero'd for logical shifting;
													// used in arith shift as well
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
//**** VLane controller ****
//---------------------------------------------------------------------------------------------------------------------------------------------------
VLane_controller	VLane_ctr_inst (

	.clk						(clk),
	.reset_n					(reset_n),
	.s3_vflag					(s3_vflag),				// vector mask for current instruction
	.s3_instr_nonmaskable		(s3_instr_nonmaskable),
	.s3_vl_lane_en				(s3_vl_lane_en),

	// Input from master controller
	.dec_s3_ALUOp				(dec_s3_ALUOp),			// ALU operation
	.dec_s3_vpmem_wren			(dec_s3_vpmem_wren),	// local memory wren
	.dec_s4_ALUOp				(dec_s4_ALUOp),
	.dec_s4_vrf_wren			(dec_s4_vrf_wren),		// vrf write enable from instruction decoder
	.dec_s4_vflag_wren			(dec_s4_vflag_wren),	// vflag write enable from instruction decoder

	// Outputs to vector lane datapath
	.s4_alu_vflag				(s4_alu_vflag),
	.s3_ALUOp					(s3_ALUOp),
	.s4_ALUOp					(s4_ALUOp),
	.s3_vpmem_wren				(s3_vpmem_wren),
	.s4_vrf_wren				(s4_vrf_wren),
	.s4_vflag_wren				(s4_vflag_wren)
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Load data Q to store loaded data from external memory
// - share the queue for storing loaded data or loaded flags
//---------------------------------------------------------------------------------------------------------------------------------------------------
VLane_loadstoreDataQ
#(	.VPU_WIDTH		(VPU_WIDTH)	)
VLane_loadDataQ_inst
(
	.clock 			( clk ),
	.data 			( loadDataQ_data ),
	.rdreq 			( loadDataQ_rdreq ),
	.wrreq 			( extLoad_lane_we ),
	.empty 			(  ),
	.full 			( loadDataQ_full ),
	.q 				( loadDataQ )
);
	

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Store data Q to buffer data to store to external memory
//---------------------------------------------------------------------------------------------------------------------------------------------------
VLane_loadstoreDataQ
#(	.VPU_WIDTH		(VPU_WIDTH)	)
VLane_storeDataQ_inst
(
	.clock 			( clk ),
	.data 			( s3_extStoreData ),
	.rdreq 			( laneStoreQRdreq ),
	.wrreq 			( s3_laneStoreQWrreq ),
	.empty 			( laneStoreQEmpty ),
	.full 			( laneStoreQFull ),
	.q 				( VLaneDataToMem )
);


// Flag queue, uses same control signals as data queue
// Doubles for storing flags to memory in flag store instruction
VLane_storeFlagQ	VLane_storeFlagQ_inst
(
	.clock 			( clk ),
	.data 			( s3_vflag ),
	.rdreq 			( laneStoreQRdreq | flagStoreQRdreq ),
	.wrreq 			( s3_laneStoreQWrreq | s3_flagStoreQWrreq ),
	.empty 			( flagStoreQEmpty ),
	.full 			(  ),
	.q 				( VLaneFlagToMem )
);


endmodule
