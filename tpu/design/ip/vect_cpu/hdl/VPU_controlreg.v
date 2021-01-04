//************************************************************************
// Scalable Vector CPU Project -- VPU control registers
// Filename: VPU_controlreg.v
// Author: Jason Yu
//
// Control registers of the VPU
// - also calculates base address increment after a vector memory instruction
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


module VPU_controlreg
#(	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1,
	parameter	NUMLANES = 1,
	
	parameter	CONTROL_REG_INIT_FILE = "control_reg_other.mif"
)
// Ports
(

	input							clk,
	input							reset_n,
	input	[VPU_WIDTH-1:0]			scalarDataIn,				// Input from scalar core, for writing to control registers
	input							vpu_instr_decode_valid,		// decoded signals are valid
	input							s4_vbase_add_vinc,
	input							s4_vc_vl_wren,
	input							s4_vl_accvl_update,			// update VL after a copy from accumulator instruction
	
	// Vector control register: base register
	input	[`log2(`VC_NVBASE)-1:0]	id_vc_vbase_rdaddr,
	input 							id_vc_vbase_rden,
	input	[`log2(`VC_NVBASE)-1:0]	s4_vc_vbase_wraddr,
	input 							s4_vc_vbase_wren,
	input	[`log2(`ELEMPERLANE)-1:0]	s4_vrfOffset,
	
	// Vector control registers
	input	[`log2(`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC)-1:0]	id_vctrl_rdaddr,
	input	[`log2(`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC)-1:0]	s4_vctrl_wraddr,
	input							s4_vctrl_wren,
	input							id_vctrl_rden,
	input							s3_vc_vctrl_rden,
	input	[`log2(`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC)-1:0]	s3_vctrl_rdaddr,

	output reg	[31:0]				vc_vbase_q,
	output		[31:0]				vc_vl_q,
	output reg	[31:0]				vctrl_q
);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local parameters
//---------------------------------------------------------------------------------------------------------------------------------------------------
// constant to divide VL by after a copy from accumulator instruction
localparam	ACCVL_DIVISOR = `MACCHAINLENGTH * `MAC_NUMMULT;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------

reg		[31:0]	vc_vl;		// vector length
wire	[31:0]	vctrl_data;
wire			vc_vbase_wren;
wire	[31:0]	vc_vbase_inc_data;
wire			vc_vbase_rden;
wire			vc_vctrl_rden;
wire	[`log2(`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC)-1:0]	vc_vctrl_rdaddr;
wire	[31:0]	s4_vbase_q;
wire	[31:0]	vc_vl_data;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Signal assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------
assign	vc_vl_q = vc_vl;
assign	vctrl_data = scalarDataIn;

// min(vc_vl,NUMLANES) because the accumulators accumulate multiple data elements within the same lane
// so there are a maximum of NUMLANE values to reduce for vccacc
// max((vc_vl/ACCVL_DIVISOR), 1) because don't want VL to become zero
assign	vc_vl_data = (s4_vl_accvl_update == 1'b1) ? `min(`max(vc_vl/ACCVL_DIVISOR, 1), NUMLANES/ACCVL_DIVISOR) : scalarDataIn;

// update vbase after a vmstc update, or after a memory access; need to qualify with s4_vrfOffset otherwise it will double update
assign	vc_vbase_wren = s4_vc_vbase_wren & (s4_vrfOffset == 0);

// select data to vbase; vbase update must occur in s4 (s2 reads vbase, s3 reads vctrl, s4 write vbase)
assign	vc_vbase_inc_data = (s4_vbase_add_vinc == 1'b1) ? s4_vbase_q + vctrl_q : scalarDataIn;
ff	#(.WIDTH(32)) 	pipe_s4_vbase_q (.clk(clk), .en(1'b1), .clrn(reset_n), .d(vc_vbase_q), .q(s4_vbase_q));

// read from control registers for vector memory access when the mem access is set to the memory unit
assign	vc_vbase_rden = id_vc_vbase_rden & vpu_instr_decode_valid;

// vector manip instructions do not work if qualify id_vctrl_rden with vpu_instr_decode_valid
assign	vc_vctrl_rden = id_vctrl_rden | s3_vc_vctrl_rden;
assign	vc_vctrl_rdaddr = (s3_vc_vctrl_rden == 1'b1) ? s3_vctrl_rdaddr : id_vctrl_rdaddr;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Vector length: VC_VL
//---------------------------------------------------------------------------------------------------------------------------------------------------	
// should flush vector unit when updating.
// need to keep as a standalone register, because relies on reading updated value the same cycle it is written
always @(posedge clk, negedge reset_n) begin
	if (~reset_n)
		vc_vl <= NUMLANES * `ELEMPERLANE;
	else if (s4_vc_vl_wren)
		vc_vl <= vc_vl_data;
end


//---------------------------------------------------------------------------------------------------------------------------------------------------
// VPU Control Registers
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Vector control base registers
(* ram_init_file = CONTROL_REG_INIT_FILE *)			// Quartus II synthesis attribute
(* ramstyle = "M9K" *)				// Quartus II synthesis attribute
(* ramstyle = "no_rw_check" *)		// Quartus II synthesis attribute
reg [31:0] vc_vctrl [`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC-1:0];		// control register

always @ (posedge clk) begin
	if (s4_vctrl_wren)
		vc_vctrl [ s4_vctrl_wraddr ] <= vctrl_data;
	if (vc_vctrl_rden) begin
		vctrl_q <= vc_vctrl [ vc_vctrl_rdaddr ];
	end
end
	
	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Base register: VC_VBASE
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Vector control base registers
(* ramstyle = "M9K" *)				// Quartus II synthesis attribute
(* ramstyle = "no_rw_check" *)		// Quartus II synthesis attribute
reg [31:0] vc_vbase [`VC_NVBASE-1:0];		// control register

always @ (posedge clk) begin
	if (vc_vbase_wren)
		vc_vbase [ s4_vc_vbase_wraddr ] <= vc_vbase_inc_data;
	if (id_vc_vbase_rden) begin
		vc_vbase_q <= vc_vbase [ id_vc_vbase_rdaddr ];
	end
end


endmodule
