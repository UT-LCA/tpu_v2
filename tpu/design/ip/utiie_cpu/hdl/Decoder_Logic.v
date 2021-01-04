/*****************************************************************************
 *                                                                           *
 * Module:       Decoder_Logic                                               *
 * Description:                                                              *
 *      This module decodes Nios II instructions into control signals.       *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Decoder_Memory the_Decoder_Memory (
	// Inputs
	.clk		(clk),
	.clk_en		(),

	.op_addr	(),
	.opx_addr	(),

	// Bidirectionals

	// Outputs
	.op_data	(),
	.opx_data	()
);
*/

module Decoder_Memory (
	// Inputs
	clk,
	clk_en,

	op_addr,
	opx_addr,

	// Bidirectionals

	// Outputs
	op_data,
	opx_data
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter INIT_MEM_FILE = "decoder_memory.mif";

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input			clk;
input			clk_en;

input	[5:0]	op_addr;
input	[5:0]	opx_addr;

// Bidirectionals

// Outputs
output	[15:0]  op_data;
output	[15:0]  opx_data;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires

// Internal Registers

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

altsyncram	altsyncram_component (
	// Inputs
	.clock0			(clk),
	.clocken0		(clk_en),
	
	.address_a		(op_addr),
	.address_b		(opx_addr),
	.aclr0			(1'b0),
	.aclr1			(1'b0),
	.addressstall_a	(1'b0),
	.addressstall_b	(1'b0),
	.byteena_a		(1'b1),
	.byteena_b		(1'b1),
	.clock1			(1'b1),
	.clocken1		(1'b1),
	.data_a			(16'hffff),
	.data_b			(16'hffff),
	.rden_b			(1'b1),
	.wren_a			(1'b0),
	.wren_b			(1'b0),

	// Outputs
	.q_a			(op_data),
	.q_b			(opx_data)
);

defparam
	altsyncram_component.operation_mode = "BIDIR_DUAL_PORT",
	altsyncram_component.width_a = 16,
	altsyncram_component.width_b = 16,
	altsyncram_component.widthad_a = 6,
	altsyncram_component.widthad_b = 6,
	altsyncram_component.numwords_a = 64,
	altsyncram_component.numwords_b = 64,
	altsyncram_component.lpm_type = "altsyncram",
	altsyncram_component.width_byteena_a = 1,
	altsyncram_component.width_byteena_b = 1,
	altsyncram_component.outdata_reg_a = "UNREGISTERED",
	altsyncram_component.outdata_reg_b = "UNREGISTERED",
	altsyncram_component.outdata_aclr_a = "NONE",
	altsyncram_component.outdata_aclr_b = "NONE",
	altsyncram_component.address_aclr_a = "NONE",
	altsyncram_component.address_aclr_b = "NONE",
	altsyncram_component.wrcontrol_aclr_a = "NONE",
	altsyncram_component.address_reg_b = "CLOCK0",
	altsyncram_component.indata_reg_b = "CLOCK0",
	altsyncram_component.wrcontrol_aclr_b = "NONE",
	altsyncram_component.wrcontrol_wraddress_reg_b = "CLOCK0",
	altsyncram_component.indata_aclr_a = "NONE",
	altsyncram_component.indata_aclr_b = "NONE",
	altsyncram_component.read_during_write_mode_mixed_ports = "DONT_CARE",
	altsyncram_component.ram_block_type = "AUTO",
	altsyncram_component.power_up_uninitialized = "FALSE",
	altsyncram_component.init_file = INIT_MEM_FILE,
	altsyncram_component.intended_device_family = "Stratix";

endmodule

/*
Decoder_Logic the_Instruction_Decoder (
	// Inputs
	.clk					(clk),
	.reset_n				(reset_n),

	.fetch_stage_completed	(),	
	.op_code				(),
	.opx_code				(),

	// Bidirectionals

	// Outputs
	.decoded_signals		(),
	.is_rtype				()
);
*/

module Decoder_Logic (
	// Inputs
	clk,
	reset_n,

	fetch_stage_completed,
	op_code,
	opx_code,

	// Bidirectionals

	// Outputs
	decoded_signals,
	is_rtype
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input			clk;
input			reset_n;

input			fetch_stage_completed;
input	[5:0]	op_code;
input	[5:0]	opx_code;

// Bidirectionals

// Outputs
output	[7:0]	decoded_signals;
output			is_rtype;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire	[15:0]	op_signals;
wire	[15:0]	opx_signals;

// Internal Registers

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign decoded_signals = (is_rtype == 1'b0) ? 
							op_signals[7:0] : 
							opx_signals[15:8];
							
assign is_rtype = op_signals[5] & op_signals[4] & ~op_signals[3];

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Decoder_Memory the_Decoder_Memory (
	// Inputs
	.clk		(clk),
	.clk_en		(fetch_stage_completed),

	.op_addr	(op_code),
	.opx_addr	(opx_code),

	// Bidirectionals

	// Outputs
	.op_data	(op_signals),
	.opx_data	(opx_signals)
);
//exemplar translate_off
//`ifdef NO_PLI
		defparam the_Decoder_Memory.INIT_MEM_FILE = "decoder_memory.dat";
//`else
//		defparam the_Decoder_Memory.INIT_MEM_FILE = "decoder_memory.hex";
//`endif
	//exemplar translate_on
	//synthesis read_comments_as_HDL on
	//defparam the_Decoder_Memory.INIT_MEM_FILE = "decoder_memory.mif";
	//synthesis read_comments_as_HDL off

endmodule

/*****************************************************************************
 *                                                                           *
 * Copyright (C) 2006 Blair Fort. All rights reserved.  Blair Fort assumes   *
 * no responsibility or liability arising out of the application or use of   *
 * any information, product, or service described herein except as expressly *
 * agreed to in writing by Blair Fort.                                       *
 *                                                                           *
 * This module is being provided on an "as-is" basis and as an accommodation *
 * and therefore all warranties, representations or guarantees of any kind   *
 * (whether express, implied or statutory) including, without limitation,    *
 * warranties of merchantability, non-infringement, or fitness for a         *
 * particular purpose, are specifically disclaimed.                          *
 *                                                                           *
 *****************************************************************************/

