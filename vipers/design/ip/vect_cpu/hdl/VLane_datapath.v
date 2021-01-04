//************************************************************************
// Scalable Vector CPU Project -- Vector lane datapath
// Filename: VLane_datapath.v
// Author: Jason Yu
//
// VLane datapath contains the processing logic for each vector lane
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


module VLane_datapath
#(	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1
)
(
	//**********
	// Inputs
	//**********
	input						clk,
	input						reset_n,
	input	[31:0]				scalarRegIn,				// Input from scalar core
	input						s3_scalarRegInIsZero,
	input	[VPU_WIDTH-1:0]		shiftInNext,				// shift chain, from lane i+1
	//input	[31:0]	shiftInPrev, 
	input	[VPU_WIDTH-1:0]		extLoadData,				// external memory load data
	input						s4_alu_vflag,					// registered mask
	
	// Vector register file
	input	[`log2(`ELEMPERLANE)-1:0]	s2_vrfRdOffset,		// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
	input	[`log2(`ELEMPERLANE)-1:0]	s3_vrfOffset,
	input	[`log2(`ELEMPERLANE)-1:0]	s4_vrfWrOffset,		// offset to VRF and vflag register files; handle having multiple elements in a RF memory block
	input	[`log2(`VRF_NREG)-1:0]		id_vrf_rdaddr1,
	input	[`log2(`VRF_NREG)-1:0]		id_vrf_rdaddr2,
	input	[`log2(`VRF_NREG)-1:0]		s4_vrf_wraddr,
	input								id_vrf_rden,
	input								s4_vrf_wren,
	
	// Vector flag register
	input  [`log2(`FLAG_NREG)-1:0]	s4_vflag_wraddr,
	input  [`log2(`FLAG_NREG)-1:0]	id_vflag_rdaddr1,
	input  [`log2(`FLAG_NREG)-1:0]	id_vflag_rdaddr2,
	// input						id_vflag_rden,
	input						s4_vflag_wren,
	input	[1:0]				s4_vflagWriteSel,
	input						s3_scalarRegInSel,			// select scalar core source
	input						s4_multiplyHigh,
	input						s3_vstype,					// whether instruction is VS-type
	input						s4_vstype,
	input	[`ALUOP_WIDTH-1:0]	s3_ALUOp,					// ALUOp for pipe stage 1
	input	[`ALUOP_WIDTH-1:0]	s4_ALUOp,					// ALUOp for pipe stage 2
	input						s3_signedOp,				// signed operation for 1st execution stage
	input						s4_signedOp,				// signed operation for 2ndst execution stage
	input	[2:0]				s4_vrfWriteSel,				// which result to write to VRF
	
	// Shifter signals
	input	[`log2(VPU_WIDTH)-1:0]	s4_stageShift,			// whether to shift each stage
	input	[VPU_WIDTH-1:0]		s4_logicalMask,				// mask for logical shift

	input	[VPU_WIDTH-1:0]		loadDataQ,					// output from data Q for storing loaded data
	
`ifdef	CLUSTER_MULTACCUM
	input	[VPU_WIDTH-1:0]		clusterMACResult,			// cluster MAC result
`endif
	input						s3_vpmem_wren,				// local memory wren
	input						s3_vpmem_rden,				// local memory rden
	
	//**********
	// Outputs
	//**********
	output						s3_vflag,					// vector mask for squashing instructions
	output reg 	[VPU_WIDTH-1:0]	s3_vrf_q1,					// output to resources shared between multiple lanes
	output reg 	[VPU_WIDTH-1:0]	s3_vrf_q2,
	output		[VPU_WIDTH-1:0]	scalarSelOut,				// output of scalar register data select MUX
	output		[VPU_WIDTH-1:0]	s3_shiftOut,
	output		[VPU_WIDTH-1:0]	loadDataQ_data
	// output						loadDataQ_full				// load data queue is full

);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// **** Local Signals ****
//---------------------------------------------------------------------------------------------------------------------------------------------------
reg		[VPU_WIDTH-1:0]	vrf_data;

`ifdef	LMEMSHARE
// if VPMem shared between all VPs
wire	[`log2(`VPLM_NWORDS)-1:0]	vpmem_rdaddr;
wire	[`log2(`VPLM_NWORDS)-1:0]	vpmem_wraddr;
`else
// if each VP has own VPMem
wire	[`log2(`ELEMPERLANE*`VPLM_NWORDS)-1:0]	vpmem_rdaddr;
wire	[`log2(`ELEMPERLANE*`VPLM_NWORDS)-1:0]	vpmem_wraddr;
`endif
wire	[VPU_WIDTH-1:0]	vpmem_data;
reg		[VPU_WIDTH-1:0]	vpmem_q;


wire	[VPU_WIDTH-1:0]	shiftResult;
wire	[VPU_WIDTH-1:0]	aluInputA;
wire	[VPU_WIDTH-1:0]	aluInputB_sig;
wire	[VPU_WIDTH-1:0]	ALUResult;
wire					ALUFlagResult;
wire					s3_vflag_q1;
wire					s3_vflag_q2;
reg		[VPU_WIDTH-1:0]		multInputA;		// use `max in case MULTINPUT_WIDTH is set to 0
reg		[VPU_WIDTH-1:0]		multInputB;		// use `max in case MULTINPUT_WIDTH is set to 0
wire	[2*VPU_WIDTH-1:0]	multOutput;				// needs to be longer to avoid compilation error; bits at the top could be unused
reg		[VPU_WIDTH-1:0]	multResult;
wire 					s4_vflagLogicOut;
reg						vflagResult;
wire					arithShift;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Assign outputs
//---------------------------------------------------------------------------------------------------------------------------------------------------
assign	scalarSelOut = (s3_scalarRegInSel) ? scalarRegIn[VPU_WIDTH-1:0] : s3_vrf_q2;
assign	loadDataQ_data = extLoadData;
assign 	s3_vflag = s3_vflag_q1;								// with fracturable lane
assign	aluInputA = s3_vrf_q1;								// Select input A source
assign	aluInputB_sig = scalarSelOut;

// vector element shift uses ALU input B passthrough, and the ALU result register to balance pipeline stages
`ifdef	VUPSHIFT
assign	s3_shiftOut = ALUResult;
`endif

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Set inputs/outputs to multipliers
//---------------------------------------------------------------------------------------------------------------------------------------------------
// first level of logic should be resolved at compile time
always @(*) begin
`ifndef	VLANE_MULTIPLIER_ON
	// synthesize away
	multInputA = 1'b0;
	multInputB = 1'b0;
`else
	// MULHI will copy upper VPU_WIDTH of full width result to destination
	// use full width source, sign extend if signed
	multInputA = aluInputA;
	multInputB = aluInputB_sig;
`endif
end


// Set output from multiplier
// first level of logic should be resolved at compile time
always @(*) begin
`ifndef	VLANE_MULTIPLIER_ON
	// synthesize away
	multResult = {VPU_WIDTH{1'b0}};
`else
	// MULHI will copy upper VPU_WIDTH of full width result to destination
	// multiply high
	if (s4_multiplyHigh == 1'b1)
		multResult = multOutput [2*VPU_WIDTH-1:VPU_WIDTH];
	else
		multResult = multOutput [VPU_WIDTH-1:0];
`endif
end

	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Vector arithmetic unit
//---------------------------------------------------------------------------------------------------------------------------------------------------
ALU
#(	.VPU_WIDTH		(VPU_WIDTH)	)
ALU_inst
(
	.clk			(clk),
	.inA			(aluInputA),
	.inB			(aluInputB_sig),
	.vstype			(s3_vstype),
	.x2_vstype		(s4_vstype),
	.ALUOp			(s3_ALUOp),
	.x2_ALUOp		(s4_ALUOp),
	.x2_flagIn		(s4_alu_vflag),
	.x2_signedOp	(s4_signedOp),

	// **** Primary outputs ****
	.ALUResult		(ALUResult),
	.flagResult		(ALUFlagResult)
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Multiplication unit
// configured with no input registers, one output register
//---------------------------------------------------------------------------------------------------------------------------------------------------
lanemult_mf
#(	.MULT_WIDTH		(VPU_WIDTH)	)
lanemult_mf_inst
(
	.clock0 		( clk ),
	.dataa_0		( multInputA ),
	.datab_0 		( multInputB ),
	.signa 			( s3_signedOp ),
	.signb 			( s3_signedOp ),
	.result 		( multOutput )
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Vector register file
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Infer register file from HDL: 2 read outputs, 1 write input
(* ramstyle = "M9K" *)				// Quartus II synthesis attribute
(* ramstyle = "no_rw_check" *)		// Quartus II synthesis attribute
reg [31:0] vregfile_mem [`VRF_NREG*`ELEMPERLANE-1:0];		// vector register file

// Infer register file from HDL: 2 read outputs, 1 write input
always @ (posedge clk) begin
	if (s4_vrf_wren)
		vregfile_mem [ {s4_vrfWrOffset, s4_vrf_wraddr} ] <= vrf_data;
	if (id_vrf_rden) begin
		s3_vrf_q1 <= vregfile_mem [ {s2_vrfRdOffset, id_vrf_rdaddr1} ];
		s3_vrf_q2 <= vregfile_mem [ {s2_vrfRdOffset, id_vrf_rdaddr2} ];
	end
end

	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Bit barrel/logical shifter
//---------------------------------------------------------------------------------------------------------------------------------------------------
assign	arithShift = (s4_ALUOp[2:0] == `SHIFT_CMD_VSRA) ? 1'b1 : 1'b0;

shifter
#(	.VPU_WIDTH				(VPU_WIDTH)	)
shifter_inst
(
	.datain			( ALUResult ),
	.stageShift		( s4_stageShift ),
	.logicalMask	( s4_logicalMask ),
	.arithShift		( arithShift ),
	.dataout		( shiftResult )
);
	

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Select result
//---------------------------------------------------------------------------------------------------------------------------------------------------

// vector element shifting, and select from functional units
always @(*)
begin
	casex (s4_vrfWriteSel)
		`VRFWR_ALURESULT	:	vrf_data = shiftResult;
		`VRFWR_EXTLOADDATA	:	vrf_data = loadDataQ;
		//`VRFWR_SHIFTINPREV	:	vrf_data = shiftInPrev;
		`VRFWR_MULTRESULT	:	vrf_data = multResult;
		
`ifdef	VUPSHIFT
		`VRFWR_SHIFTINNEXT	:	vrf_data = shiftInNext;		// Vector element up shift
`endif
`ifdef VP_LOCALMEM			
		`VRFWR_LOCALMEM		:	vrf_data = vpmem_q;			// VP local memory result
`endif
	
		// cluster MAC unit result
`ifdef CLUSTER_MULTACCUM
		`VRFWR_CLUSTERMAC	:	vrf_data = clusterMACResult;
`endif
		
		default:		vrf_data = shiftResult;			// default to output from shifter
	endcase
end

	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Vector flag register file
// If trying to infer register file from HDL,
// the two read ports need to be inferred separately because otherwise Quartus doesn't pack them into MLABs
//---------------------------------------------------------------------------------------------------------------------------------------------------

// Use megafunction which can be packed more efficiently by Quartus
vflagreg_mf	vflagreg_mf_inst (
	.clock 				( clk ),
	.data 				( vflagResult ),
	.rdaddress_a 		( {s2_vrfRdOffset, id_vflag_rdaddr1} ),
	.rdaddress_b 		( {s2_vrfRdOffset, id_vflag_rdaddr2} ),
	.wraddress 			( {s4_vrfWrOffset, s4_vflag_wraddr} ),
	.wren 				( s4_vflag_wren ),
	.qa 				( s3_vflag_q1 ),
	.qb 				( s3_vflag_q2 )
);

// Select vflag register write input
always @(*) begin
	case(s4_vflagWriteSel)
		`VFLAGWR_FLAGUNIT	:	vflagResult = s4_vflagLogicOut;
		`VFLAGWR_ALUFLAG	:	vflagResult = ALUFlagResult;
		// if transfer from load queue to flag register, use LSB of load queue output
		`VFLAGWR_EXTLOADDATA	:	vflagResult = loadDataQ[0];
		default:	vflagResult = 1'bx;
	endcase
end


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Flag logic unit
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Flag unit
flagunit
#(.WIDTH(1))
flagunit_inst (
	// Primary inputs
	.clk				(clk),
	.flagInA			(s3_vflag_q1),
	.flagInB			(s3_vflag_q2),
	.scalarRegInIsZero	(s3_scalarRegInIsZero),
	.s3_ALUOp			(s3_ALUOp),
	// Outputs
	.flagOut			(s4_vflagLogicOut)
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// VP local memory
//---------------------------------------------------------------------------------------------------------------------------------------------------
	
`ifdef VP_LOCALMEM
// data input can come from scalar core, or vector register file
assign	vpmem_data = scalarSelOut;

`ifdef LMEMSHARE
// if local memory is shared among VPs in the same vector lane (common address space)
assign	vpmem_wraddr = s3_vrf_q1[`log2(`VPLM_NWORDS)-1:0];		// Register indirect addressing
assign	vpmem_rdaddr = s3_vrf_q1[`log2(`VPLM_NWORDS)-1:0];		// Register indirect addressing
`else
// if each VP has its own local memory (separate address space)
assign	vpmem_wraddr = {s3_vrfOffset, s3_vrf_q1[`log2(`VPLM_NWORDS)-1:0]};		// Register indirect addressing
assign	vpmem_rdaddr = {s3_vrfOffset, s3_vrf_q1[`log2(`VPLM_NWORDS)-1:0]};		// Register indirect addressing
`endif


// Infer register file from HDL: 1 read output, 1 write input
`ifdef LMEMSHARE
	(* ramstyle = "M9K" *)				// Quartus II synthesis attribute
	(* ramstyle = "no_rw_check" *)		// Quartus II synthesis attribute
	reg [VPU_WIDTH-1:0] vpmem [`VPLM_NWORDS-1:0];		// vector local memory
`else
	(* ramstyle = "M9K" *)				// Quartus II synthesis attribute
	(* ramstyle = "no_rw_check" *)		// Quartus II synthesis attribute
	reg [VPU_WIDTH-1:0] vpmem [`ELEMPERLANE*`VPLM_NWORDS-1:0];		// vector local memory
`endif

// Automatically infer Local memory
always @ (posedge clk) begin
	if (s3_vpmem_wren)
		vpmem [ vpmem_wraddr ] <= vpmem_data;
	if (s3_vpmem_rden) begin
		vpmem_q <= vpmem [ vpmem_rdaddr ];
	end
end
	
`endif
	
endmodule
