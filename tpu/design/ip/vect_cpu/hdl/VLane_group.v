//************************************************************************
// Scalable Vector CPU Project -- Wrapper for array of vector lanes
// Filename: VLane_group.v
// Author: Jason Yu
//
// Generates the entire array of vector lanes
// and structures that are shared between vector lanes such as:
// 1) MAC units
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


module VLane_group
#(	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1,
	parameter	NUMLANES = 1
)
(
	input						clk,
	input						reset_n,
	input	[31:0]				scalarRegIn,				// scalar register input
	input	[NUMLANES-1:0]		s3_vl_lane_en,				// lane enable signal computed from vector length	
	input						s3_instr_nonmaskable,

	// Vector register file
	input	[`log2(`ELEMPERLANE)-1:0]	s2_vrfRdOffset,		// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
	input	[`log2(`ELEMPERLANE)-1:0]	s3_vrfOffset,
	input	[`log2(`ELEMPERLANE)-1:0]	s4_vrfWrOffset,		// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
	input	[`log2(`VRF_NREG)-1:0]	id_vrf_rdaddr1,
	input	[`log2(`VRF_NREG)-1:0]	id_vrf_rdaddr2,
	input	[`log2(`VRF_NREG)-1:0]	s4_vrf_wraddr,
	input						id_vrf_rden,
	input						dec_s4_vrf_wren,			// vrf write enable from instruction decoder
	// vector flag register
	input	[`FLAG_NREGADDR-1:0]	id_vflag_rdaddr1,
	input	[`FLAG_NREGADDR-1:0]	id_vflag_rdaddr2,
	input	[`FLAG_NREGADDR-1:0]	s4_vflag_wraddr,
	// input						id_vflag_rden,
	input						dec_s4_vflag_wren,			// vflag write enable from instruction decoder
	input	[1:0]				s4_vflagWriteSel,			// what source to write into flag register
	
	// interface to memory
	input	[NUMLANES*VPU_WIDTH-1:0]	memLoadDataToLanes,	// data to processors
	input	[NUMLANES-1:0]		extLoad_lane_we,
	input						s3_loadDataQToVReg,			// transfers data from loaddataQ to vector register
	input						s3_loadDataQ_rdmaskable,	// the rdreq for transferring data from load data Q to VRF is maskable (for vins.vv)
	input	[NUMLANES-1:0]		laneStoreQRdreq,
	input						s3_laneStoreQWrreq,			// all lanes share 1 wrreq signal
	input						flagStoreQRdreq,
	input						s3_flagStoreQWrreq,
	input						s4_vupshift,
	input						s3_vstype,					// whether instruction is vs-type
	input						s4_vstype,
	input	[`ALUOP_WIDTH-1:0]	dec_s3_ALUOp,				// ALU function
	input	[`ALUOP_WIDTH-1:0]	dec_s4_ALUOp,				// ALU function
	input						dec_s3_vpmem_wren,			// local memory wren
	input						s3_scalarRegInSel,
	input						s4_multiplyHigh,			// multiplication: multiply upper half, or multiply full and store upper half
	input						id_signedOp,
	input						s3_signedOp,
	input						s4_signedOp,
	input	[2:0]				s4_vrfWriteSel,				// what source to write to VRF

	// Local memory
	input						s3_vpmem_rden,				// local memory rden
	
`ifdef CLUSTER_MULTACCUM
	input						s4_MAC_en,
	input						id_zeroACC,
	input						s3_zeroACC,					// zero the first stage MAC result registers
	input						s4_zeroMACchainout,			// zero mask for the MAC chainout/result signal
`endif

	// Shifter signals
	input						s3_shift_sv,				// scalar-vector shift (vector data, scalar shift amount)
	input						s4_shift_instr,				// instruction is a shift instruction

	//**********
	// Outputs
	//**********
	output	[NUMLANES*VPU_WIDTH-1:0]	s3_indexMemOffset_group,	// offset for indexed memory access
	output	[NUMLANES*VPU_WIDTH-1:0]	VLaneDataToMem,		// data from store data queue to memory
	output	[NUMLANES-1:0]				VLaneFlagToMem,		// flag from store data flag queue to memory
	output						loadDataQ_full,
	output						flagStoreQEmpty				// the flag store queue is empty
);

	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Connection signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	[NUMLANES*VPU_WIDTH-1:0]	shiftNextChain;					// shift chain from lane i+1 to i
wire	[NUMLANES*VPU_WIDTH-1:0]	shiftOutChain;					// shift output from lanes
wire	[NUMLANES*VPU_WIDTH-1:0]	s4_shiftOutChain;				// shift output from lanes
// wire	[NUMLANES*VPU_WIDTH-1:0]	laneToMemDataQ;					// data from lane to memory store queue
// wire	[NUMLANES-1:0]				laneToMemFlagQ;					// flag from lane to memory store queue
wire	[NUMLANES*VPU_WIDTH-1:0]	vrf_q1_group;					// output from lane register file
wire	[NUMLANES*VPU_WIDTH-1:0]	vrf_scalar_q2_group;			// output from lane register file
wire	[NUMLANES-1:0]				s3_vmask_group;
wire	[NUMLANES-1:0]				s3_vmask_on_group;
wire	[NUMLANES*VPU_WIDTH-1:0]	macResultToLane;				// truncated result from MAC unit to lanes
wire	[NUMLANES-1:0]				laneStoreQFull;
wire	[NUMLANES-1:0]				laneStoreQEmpty_group;
wire	[NUMLANES-1:0]				loadDataQ_lanei_full;			// the load data queue is full
wire	[NUMLANES-1:0]				flagStoreQEmpty_group;
wire	[`log2(`ELEMPERLANE)-1:0]	s4_vrfWrOffset_upshift;			// offset to VRF and vflag register files for vector upshift
wire	[`log2(`ELEMPERLANE)-1:0]	s4_vrfWrOffset_lastlane;		// offset to VRF and vflag register files for last lane
wire	[NUMLANES-1:0]				s3_laneStoreQWrreq_sig;			// inddividual store cluster queue wrreq signal
wire	[NUMLANES*VPU_WIDTH-1:0]	vrf_q1_group_masked;			// output from lane register file
wire	[NUMLANES*VPU_WIDTH-1:0]	vrf_scalar_q2_group_masked;		// output from lane register file
wire								s3_scalarRegInIsZero;			// scalar register input is zero (for vector-scalar flag logical operations)


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Signal assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------
assign	s3_scalarRegInIsZero = (scalarRegIn == 0) ? 1'b1 : 1'b0;

// Whether the store queue is empty
// All queues are written to at the same time, so if one of the queues ahve data, all queues will have data
// Using flag queue status signals because data queue is written to for flag store as well
// queues are not necessarily written to at the same time
// not empty if any one of the queues are not empty
assign	flagStoreQEmpty = & flagStoreQEmpty_group;

// OR reduce the signal
// assign	loadDataQ_full = | loadDataQ_lanei_full;
// AND reduce the signal
assign	loadDataQ_full = & loadDataQ_lanei_full;

	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Vector lane up shift chain (v[i] <= v[i+1])
// - logic will be synthesized away if VUPSHIFT is off
//---------------------------------------------------------------------------------------------------------------------------------------------------
// For vector element up shift (v[i] <= v[i+1]), the last lane needs to write to a different vector element
assign	s4_vrfWrOffset_upshift = (s4_vrfWrOffset - 1) % `ELEMPERLANE;

`ifdef	VUPSHIFT
assign	s4_vrfWrOffset_lastlane = (s4_vupshift) ? s4_vrfWrOffset_upshift : s4_vrfWrOffset;
`else
assign	s4_vrfWrOffset_lastlane = s4_vrfWrOffset;
`endif

generate
	genvar i;
	for (i=0; i<NUMLANES-1; i=i+1)
	begin: shifti
		assign	shiftNextChain[ (i+1)*VPU_WIDTH-1 : i*VPU_WIDTH ] = s4_shiftOutChain[ (i+2)*VPU_WIDTH-1 : (i+1)*VPU_WIDTH ];
	end
endgenerate

// complete the loop
assign	shiftNextChain[ (NUMLANES)*VPU_WIDTH-1 : (NUMLANES-1)*VPU_WIDTH ] = shiftOutChain[ (1)*VPU_WIDTH-1 : (0)*VPU_WIDTH ];

// already pipelined inside vlane_datapath
assign	s4_shiftOutChain = shiftOutChain;

generate
genvar gi;
	for (gi=0; gi<NUMLANES; gi=gi+1)
	begin: laneStoreQWrreq_i
		// transfer the cluster if any single lane in the cluster is enabled
		assign	s3_laneStoreQWrreq_sig[gi] = s3_laneStoreQWrreq & s3_vl_lane_en [gi];
	end
endgenerate

	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Generate vector lanes
//---------------------------------------------------------------------------------------------------------------------------------------------------
generate
	genvar lane;
	for (lane=0; lane<NUMLANES; lane=lane+1)
	begin:lanei
		case (lane)
		// The last lane
		// Needs a different VRFWrOffset for vector element up shift
		NUMLANES-1: begin

		VLane_top
		#(	.VPU_WIDTH				(VPU_WIDTH)	)
		VLane_inst
		(
			//**********
			// Inputs
			//**********
			.clk					(clk),
			.reset_n				(reset_n),
			.s3_instr_nonmaskable	(s3_instr_nonmaskable),
			.s3_vl_lane_en			(s3_vl_lane_en[lane]),		// lane enable from vector length

			// Vector register file
			.s2_vrfRdOffset			(s2_vrfRdOffset),			// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
			.s3_vrfOffset			(s3_vrfOffset),
			.s4_vrfWrOffset			(s4_vrfWrOffset_lastlane),	// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
			.id_vrf_rdaddr1			(id_vrf_rdaddr1),
			.id_vrf_rdaddr2			(id_vrf_rdaddr2),
			.id_vrf_rden			(id_vrf_rden),
			.s4_vrf_wraddr			(s4_vrf_wraddr),
			.dec_s4_vrf_wren		(dec_s4_vrf_wren),		// vrf write enable from instruction decoder
			// vector flag register
			.id_vflag_rdaddr1		(id_vflag_rdaddr1),
			.id_vflag_rdaddr2		(id_vflag_rdaddr2),
			.s4_vflag_wraddr		(s4_vflag_wraddr),
			.dec_s4_vflag_wren		(dec_s4_vflag_wren),	// vflag write enable from instruction decoder
			// .id_vflag_rden			(id_vflag_rden),
			.s4_vflagWriteSel		(s4_vflagWriteSel),
			// scalar register input
			.scalarRegIn			(scalarRegIn),
			.s3_scalarRegInIsZero	(s3_scalarRegInIsZero),
			.s3_scalarRegInSel		(s3_scalarRegInSel),
			.s3_vstype				(s3_vstype),
			.s4_vstype				(s4_vstype),
			.dec_s3_ALUOp			(dec_s3_ALUOp),
			.dec_s4_ALUOp			(dec_s4_ALUOp),
			// Memory Interface
			.extLoadData			( memLoadDataToLanes[ lane*VPU_WIDTH+(VPU_WIDTH-1) : lane*VPU_WIDTH ] ),
			.extLoad_lane_we		( extLoad_lane_we[lane] ),
			.s3_loadDataQToVReg		(s3_loadDataQToVReg),		// transfers data from loaddataQ to vector register
			.s3_loadDataQ_rdmaskable	(s3_loadDataQ_rdmaskable),
			
			// shift chain
			//.shiftInPrev(shiftchain[NUMLANES-1]),
			.shiftInNext			( shiftNextChain[ (lane+1)*VPU_WIDTH-1 : lane*VPU_WIDTH ] ),
			
			.s4_multiplyHigh		(s4_multiplyHigh),		// multiplication: multiply upper half, or multiply full and store upper half
			.s4_vrfWriteSel			(s4_vrfWriteSel),		// what source to write to VRF
			.s3_signedOp			(s3_signedOp),
			.s4_signedOp			(s4_signedOp),
			
			// VP local memory
			.dec_s3_vpmem_wren		(dec_s3_vpmem_wren),	// local memory wren
			.s3_vpmem_rden			(s3_vpmem_rden),		// local memory rden

			// Signals to resources shared between multiple lanes	
		`ifdef	CLUSTER_MULTACCUM
			.clusterMACResult		( macResultToLane [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ] ),		// result from cluster MAC unit
		`endif
		
			// Shifter signals
			.s3_shift_sv			(s3_shift_sv),
			.s4_shift_instr			(s4_shift_instr),			// instruction is a shift instruction
			
			.laneStoreQRdreq		(laneStoreQRdreq[lane]),
			.s3_laneStoreQWrreq		(s3_laneStoreQWrreq_sig[lane]),
			.flagStoreQRdreq		(flagStoreQRdreq),
			.s3_flagStoreQWrreq		(s3_flagStoreQWrreq),
			
			//**********
			// Outputs
			//**********
			.s3_vrf_q1				( vrf_q1_group [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ] ),	// output from register file
			.s3_scalarSelOut		( vrf_scalar_q2_group [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ] ),	// output from scalar/vrf source select MUX
			.loadDataQ_full			( loadDataQ_lanei_full[lane] ),
			.s3_vflag				( s3_vmask_group [lane] ),
			.s3_indexMemOffset		( s3_indexMemOffset_group [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ] ),
			
			.laneStoreQEmpty		(laneStoreQEmpty_group [lane]),
			.VLaneDataToMem			(VLaneDataToMem [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ]),				// data to store to external memory
			.laneStoreQFull			(laneStoreQFull [lane]),
			.VLaneFlagToMem			(VLaneFlagToMem [lane]),
			.flagStoreQEmpty		(flagStoreQEmpty_group [lane]),
			
			// .s3_extStoreData		( laneToMemDataQ [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ] ),
			.s3_shiftOut			( shiftOutChain [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ] )
		);
		end // case last vector lane
	
		default: begin
		VLane_top
		#(	.VPU_WIDTH				(VPU_WIDTH)	)
		VLane_inst
		(
			//**********
			// Inputs
			//**********
			.clk					(clk),
			.reset_n				(reset_n),
			.s3_instr_nonmaskable	(s3_instr_nonmaskable),
			.s3_vl_lane_en			(s3_vl_lane_en[lane]),		// lane enable from vector length
			
			// Vector register file
			.s2_vrfRdOffset			(s2_vrfRdOffset),	// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
			.s3_vrfOffset			(s3_vrfOffset),
			.s4_vrfWrOffset			(s4_vrfWrOffset),	// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
			.id_vrf_rdaddr1			(id_vrf_rdaddr1),
			.id_vrf_rdaddr2			(id_vrf_rdaddr2),
			.id_vrf_rden			(id_vrf_rden),
			.s4_vrf_wraddr			(s4_vrf_wraddr),
			.dec_s4_vrf_wren		(dec_s4_vrf_wren),			// vrf write enable from instruction decoder
			// vector flag register
			.id_vflag_rdaddr1		(id_vflag_rdaddr1),
			.id_vflag_rdaddr2		(id_vflag_rdaddr2),
			.s4_vflag_wraddr		(s4_vflag_wraddr),
			.dec_s4_vflag_wren		(dec_s4_vflag_wren),		// vflag write enable from instruction decoder
			// .id_vflag_rden			(id_vflag_rden),
			.s4_vflagWriteSel		(s4_vflagWriteSel),
			// scalar register input
			.scalarRegIn			(scalarRegIn),
			.s3_scalarRegInIsZero	(s3_scalarRegInIsZero),
			.s3_scalarRegInSel		(s3_scalarRegInSel),
			
			// Memory Interface
			.extLoadData			( memLoadDataToLanes[ lane*VPU_WIDTH+(VPU_WIDTH-1) : lane*VPU_WIDTH ] ),
			.extLoad_lane_we		( extLoad_lane_we[lane] ),
			.s3_loadDataQToVReg		(s3_loadDataQToVReg),		// transfers data from loaddataQ to vector register
			.s3_loadDataQ_rdmaskable	(s3_loadDataQ_rdmaskable),
			
			// shift chain
			//.shiftInPrev(shiftchain[lane-1]),
			.shiftInNext			( shiftNextChain[ (lane+1)*VPU_WIDTH-1 : lane*VPU_WIDTH ] ),
			
			// **** Control inputs ****
			.s4_multiplyHigh		(s4_multiplyHigh),			// multiplication: multiply upper half, or multiply full and store upper half
			.s4_vrfWriteSel			(s4_vrfWriteSel),			// what source to write to VRF
			.s3_vstype				(s3_vstype),
			.s4_vstype				(s4_vstype),
			.dec_s3_ALUOp			(dec_s3_ALUOp),
			.dec_s4_ALUOp			(dec_s4_ALUOp),
			.s3_signedOp			(s3_signedOp),
			.s4_signedOp			(s4_signedOp),

			// VP local memory
			.dec_s3_vpmem_wren		(dec_s3_vpmem_wren),		// local memory wren
			.s3_vpmem_rden			(s3_vpmem_rden),			// local memory rden
			
			// Signals to resources shared between multiple lanes	
		`ifdef	CLUSTER_MULTACCUM
			.clusterMACResult		( macResultToLane [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ] ),		// result from cluster MAC unit
		`endif
		
			// Shifter signals
			.s3_shift_sv			(s3_shift_sv),
			.s4_shift_instr			(s4_shift_instr),			// instruction is a shift instruction

			.laneStoreQRdreq		(laneStoreQRdreq[lane]),
			.s3_laneStoreQWrreq		(s3_laneStoreQWrreq_sig[lane]),
			.flagStoreQRdreq		(flagStoreQRdreq),
			.s3_flagStoreQWrreq		(s3_flagStoreQWrreq),

			//**********
			// Outputs
			//**********
			.s3_vrf_q1				( vrf_q1_group [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ] ),		// output from register file
			.s3_scalarSelOut		( vrf_scalar_q2_group [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ] ),	// output from scalar/vrf source select MUX
			.loadDataQ_full			( loadDataQ_lanei_full[lane] ),
			.s3_vflag				( s3_vmask_group [lane] ),
			.s3_indexMemOffset		( s3_indexMemOffset_group [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ] ),
			
			.laneStoreQEmpty		(laneStoreQEmpty_group [lane]),
			.VLaneDataToMem			(VLaneDataToMem [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ]),				// data to store to external memory
			.laneStoreQFull			(laneStoreQFull [lane]),
			.VLaneFlagToMem			(VLaneFlagToMem [lane]),
			.flagStoreQEmpty		(flagStoreQEmpty_group [lane]),
			
			// .s3_extStoreData		( laneToMemDataQ [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ] ),
			.s3_shiftOut			( shiftOutChain [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ] )
		);
		end	// default case
		
		endcase
		
	end // end for
endgenerate


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Multiply accumulate for entire cluster
//---------------------------------------------------------------------------------------------------------------------------------------------------
`ifdef CLUSTER_MULTACCUM
// mask the MAC inputs if vector flag is not on
generate
	for (lane=0; lane<NUMLANES; lane=lane+1)
	begin:mac_lanei
		// when to enable the lane input to MAC unit
		assign	s3_vmask_on_group [lane] = ((s3_vmask_group [lane] == `FLAG_ON_VALUE) && ((s3_vl_lane_en[lane] == 1'b1) || (s3_instr_nonmaskable == 1'b1))) ? 1'b1 : 1'b0;
		assign	vrf_q1_group_masked [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ] = {VPU_WIDTH{s3_vmask_on_group [lane]}} & vrf_q1_group [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ];
		// apply mask to inputs
		assign	vrf_scalar_q2_group_masked [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ] = {VPU_WIDTH{s3_vmask_on_group [lane]}} & vrf_scalar_q2_group [ VPU_WIDTH*(lane+1)-1 : VPU_WIDTH*lane ];
	end
endgenerate

// multiply accumulate unit
	MACunit	
	#(	.VPU_WIDTH				(VPU_WIDTH),
		.NUMLANES				(NUMLANES)
	)
	MACunit_inst
	(
		.clk					(clk),
		.reset_n				(reset_n),
		.s4_MAC_en				(s4_MAC_en),
		.id_zeroACC				(id_zeroACC),
		.s3_zeroACC				(s3_zeroACC),
		.s4_zeroMACchainout		(s4_zeroMACchainout),			// zero the MAC chainout signals
		.id_signedOp			(id_signedOp),
		.vrf_q1_group			(vrf_q1_group_masked),			// outputs from register file
		.vrf_q2_group			(vrf_scalar_q2_group_masked),	// outputs from register file
		.macResultToLane		(macResultToLane)				// result to lanes
	);
`endif
	
endmodule
