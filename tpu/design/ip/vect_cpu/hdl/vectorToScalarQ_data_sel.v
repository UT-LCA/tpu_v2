//************************************************************************
// Scalable Vector CPU Project -- Data select for vector to scalar transfer queue
// Filename: vectorToScalarQ_data_sel.v
// Author: Jason Yu
//
// Selects data to be stored to the vector to scalar transfer queue

//  Copyright (C) 2007 Jason Yu
//
//************************************************************************

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on
`ifdef	MODELSIM
	`include "../hdl/define.v"
`else
	`include "define.v"
`endif


module	vectorToScalarQ_data_sel
#(	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1
)
(
	input	[2:0]	vectorToScalarQ_data_source,		// vector to scalar tranfer queue write data source
	input			vectorToScalarQ_data_zext,		// zero extend vector to scalar transfer queue write data
	input	[31:0]	vc_vbase_q,
	input	[31:0]	vctrl_q,
	input	[31:0]	vc_vl_q,
	input	[VPU_WIDTH-1:0]	indexOffset_out,			// output of the index offset select MUX

	output reg	[31:0]		vectorToScalarQ_data
);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Combinational assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------

always @(*)
	case (vectorToScalarQ_data_source)
		`VTOSQ_DATA_VL		:	vectorToScalarQ_data = vc_vl_q;
		`VTOSQ_DATA_VBASE	:	vectorToScalarQ_data = vc_vbase_q;
		// zero extend
		`VTOSQ_DATA_VINDEX	:	vectorToScalarQ_data = (vectorToScalarQ_data_zext) ? {{(32-VPU_WIDTH){1'b0}}, indexOffset_out} : {{(32-VPU_WIDTH){indexOffset_out[VPU_WIDTH-1]}}, indexOffset_out};
		// output from the control register file by default
		default				:	vectorToScalarQ_data = vctrl_q;
	endcase


endmodule
