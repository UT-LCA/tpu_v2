// ********************************************************************************
// Scalable Vector CPU Project -- Miscellaneous definitions
// Filename: define.v
// Author: Jason Yu
//
// Implementation-specific definitions
//
//  Copyright (C) 2007 Jason Yu
//
// ********************************************************************************

// include math macros
// synthesis translate_on
`ifdef	MODELSIM
	`include "../hdl/mathmacros.v"
`else
	`include "mathmacros.v"
`endif


// Constants for accessing memory: size of data being accessed in memory
`define		MEMACCESS_BYTE			3'b100
`define		MEMACCESS_HALFWORD		3'b010
`define		MEMACCESS_WORD			3'b001

// ***********************
// VPU definitions
// ***********************

// ***********************
// Indicate what type of data is waiting in the load data transfer queue
// 000 = no data waiting
// 001 = memory load data waiting
// 010 = memory flag load data waiting
// 011 = vector extract data waiting
// 100 = vector insert transfer waiting
// ***********************
`define		LSU_LOAD_DATA_EMPTY			3'b000		// no data waiting
`define		LSU_LOAD_DATA_WAITING		3'b001		// regular load data
`define		LSU_FLAG_DATA_WAITING		3'b010		// flag load
`define		LSU_VEXT_DATA_WAITING		3'b011		// vector extract data
`define		LSU_VINS_DATA_WAITING		3'b100		// vector insert data


// **********************
// **** Writing to VRF ****
// **********************
`define		VRFWR_ALURESULT			3'b000
`define		VRFWR_EXTLOADDATA		3'b001
`define		VRFWR_MULTRESULT		3'b010
`define		VRFWR_SHIFTINNEXT		3'b011
//`define		VRFWR_SHIFTINPREV		3'b100
`define		VRFWR_LOCALMEM			3'b101
`define		VRFWR_CLUSTERMAC		3'b111		// TEMPORARY

// Writing to vflag
`define		VFLAGWR_FLAGUNIT			2'b00
`define		VFLAGWR_ALUFLAG				2'b01
`define		VFLAGWR_EXTLOADDATA			2'b10
	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// 	Multiplexer select values, for vector to scalar transfer queue data source selection
//	- for move vector control to scalar register instruction
//---------------------------------------------------------------------------------------------------------------------------------------------------

`define		VTOSQ_DATA_VL		3'b000		// transfer vector length register
`define		VTOSQ_DATA_VCTRL	3'b001		// transfer one of the other control register values
`define		VTOSQ_DATA_VBASE	3'b010		// transfer one of the vbase values
`define		VTOSQ_DATA_VINDEX	3'b011		// transfer from indexoffset select MUX (vext.sv instruction)	
