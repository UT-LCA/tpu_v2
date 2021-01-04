// ********************************************************************************
// Scalable Vector CPU Project -- Configurable parameter definitions
//
// Filename: config_def.v
// Author: Jason Yu
//
// This file contains the VPU configurable parameter definitions
//
//  Copyright (C) 2007 Jason Yu
//
// ********************************************************************************

// include auto generated configuraion parameters
`ifdef	MODELSIM
	`include "../hdl/config_def_auto.v"
`else
	`include "config_def_auto.v"
`endif

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Primary processor parameters
//---------------------------------------------------------------------------------------------------------------------------------------------------
`define		CLUSTER_MULTACCUM			// define this if multiply accumulator is needed
`define		VECTOR_MANIP_INSTRUCTIONS		// if defined, enables vins.vv and vext.vv

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Memory Interface
//---------------------------------------------------------------------------------------------------------------------------------------------------
`define		MAXSTRIDE_NBIT		16		// number of bits to use for storing stride in number of elements

`define		INDEXED_MEMORY_ACCESS		// define if need indexed memory access

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Multiplier / Multiply-Accumulator
//---------------------------------------------------------------------------------------------------------------------------------------------------
`define		VLANE_MULTIPLIER_ON			// define this if vector lane multiplier is needed

`define		MACVPLOWERHALF				// MAC uses lower half of VP data for multiplication
										// for the case VPU_WIDTH != MAC_WIDTH
										// Can leave defined if VPU_WIDTH = MAC_WITH

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local Memory
//---------------------------------------------------------------------------------------------------------------------------------------------------
`define		VP_LOCALMEM					// define this if VP local memory is needed

`define		VPLM_NWORDS			256
`define		LMEMSHARE					// define this if want to have single local memory between all VPs in a lane (same address space)
										// undefine if each VP needs its own local memory (separate address space)

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Vector up shift chain (v[i] <= v[i+1])
//---------------------------------------------------------------------------------------------------------------------------------------------------
`define		VUPSHIFT					// define this if vector upshift is needed


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Stratix III Architecture specific constants
//---------------------------------------------------------------------------------------------------------------------------------------------------
`define		MAC_WIDTH				16		// width of MAC multipliers
`define		MAC_NUMMULT				4		// number of multipliers in MAC block; architecture dependent
`define		MAC_INTERNAL_WIDTH		44		// Internal width of MAC accumulators; set according to architecture
`define		ELEMPERLANE				4		// number of VRF elements to map to each lane; 1, 2, 4
											// normally DO NOT CHANGE
