/*****************************************************************************
 *                                                                           *
 * Module:       Register_File                                               *
 * Description:                                                              *
 *      This module is the reister file for the UT IIe processor.            *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Register_Memory the_Register_Memory (
	// inputs
	.clk		(clk),
	.clk_en_a	(),
	.clk_en_b	(),

	.address_a	(),
	.address_b	(),
	.data_a		(),
	.data_b		(),
	.wren_a		(),
	.wren_b		(),

	// outputs
	.q_a		(),
	.q_b		()
);
*/

module Register_Memory (
	// Inputs
	clk,
	clk_en_a,
	clk_en_b,

	address_a,
	address_b,
	data_a,
	data_b,
	wren_a,
	wren_b,

	// Bidirectionals

	// Outputs
	q_a,
	q_b
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/
parameter INIT_REG_MEM_FILE = "reg_memory.mif";

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input			clk;
input			clk_en_a;
input			clk_en_b;

input	[4:0]	address_a;
input	[4:0]	address_b;
input	[31:0]	data_a;
input	[31:0]	data_b;
input			wren_a;
input			wren_b;

// Bidirectionals

// Outputs
output	[31:0]  q_a;
output	[31:0]  q_b;

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
	// inputs
	.clock0			(clk),
	.clock1			(clk),
	.clocken0		(clk_en_a),
	.clocken1		(clk_en_b),

	.address_a		(address_a),
	.address_b		(address_b),
	.data_a			(data_a),
	.data_b			(data_b),
	.wren_a			(wren_a),
	.wren_b			(wren_b),

	.aclr0			(1'b0),
	.aclr1			(1'b0),
	.addressstall_a	(1'b0),
	.addressstall_b	(1'b0),
	.byteena_a		(1'b1),
	.byteena_b		(1'b1),
	.rden_b			(1'b1),

	// outputs
	.q_a			(q_a),
	.q_b			(q_b)
);
	defparam
		altsyncram_component.operation_mode = "BIDIR_DUAL_PORT",
		altsyncram_component.width_a = 32,
		altsyncram_component.width_b = 32,
		altsyncram_component.widthad_a = 5,
		altsyncram_component.widthad_b = 5,
		altsyncram_component.numwords_a = 32,
		altsyncram_component.numwords_b = 32,
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
		altsyncram_component.address_reg_b = "CLOCK1",
		altsyncram_component.indata_reg_b = "CLOCK1",
		altsyncram_component.wrcontrol_aclr_b = "NONE",
		altsyncram_component.wrcontrol_wraddress_reg_b = "CLOCK1",
		altsyncram_component.indata_aclr_a = "NONE",
		altsyncram_component.indata_aclr_b = "NONE",
		altsyncram_component.read_during_write_mode_mixed_ports = "DONT_CARE",
		altsyncram_component.ram_block_type = "AUTO",
		altsyncram_component.power_up_uninitialized = "FALSE",
		altsyncram_component.init_file = INIT_REG_MEM_FILE,
		altsyncram_component.intended_device_family = "Stratix";

endmodule

/*
Register_File the_Register_File (
	// Inputs
	.clk					(clk),
	.reset_n				(reset_n),
	
	.fetch_stage_completed	(),

	.port_a_data_in			(),
	.port_a_rd_addr			(),
	.port_a_wr_addr			(),
	.port_a_wr_en			(),

	.port_b_data_in			(),
	.port_b_rd_addr			(),
	.port_b_wr_addr			(),
	.port_b_wr_en			(),

	// Bidirectionals

	// Outputs
	.port_a_data_out		(),
	.port_b_data_out		()
);
*/

module Register_File (
	// Inputs
	clk,
	reset_n,
	
	fetch_stage_completed,

	port_a_data_in,
	port_a_rd_addr,
	port_a_wr_addr,
	port_a_wr_en,

	port_b_data_in,
	port_b_rd_addr,
	port_b_wr_addr,
	port_b_wr_en,

	// Bidirectionals

	// Outputs
	port_a_data_out,
	port_b_data_out
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

input	[31:0] 	port_a_data_in;
input	[4:0] 	port_a_rd_addr;
input	[4:0] 	port_a_wr_addr;
input			port_a_wr_en;

input	[31:0] 	port_b_data_in;
input	[4:0] 	port_b_rd_addr;
input	[4:0] 	port_b_wr_addr;
input			port_b_wr_en;

// Bidirectionals

// Outputs
output	[31:0] 	port_a_data_out;
output	[31:0] 	port_b_data_out;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire	[4:0]	addr_a;
wire	[4:0]	addr_b;
wire			clk_en_port_a;
wire			clk_en_port_b;
wire			wr_en_a;
wire			wr_en_b;

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

assign addr_a = ((port_a_wr_en == 1'b1) ? port_a_wr_addr : port_a_rd_addr);
assign addr_b = ((port_b_wr_en == 1'b1) ? port_b_wr_addr : port_b_rd_addr);

assign clk_en_port_a = fetch_stage_completed | port_a_wr_en;
assign clk_en_port_b = fetch_stage_completed | port_b_wr_en;

assign wr_en_a = ((port_a_wr_addr == 6'h00) ? 1'b0 : port_a_wr_en);
assign wr_en_b = ((port_b_wr_addr == 6'h00) ? 1'b0 : port_b_wr_en);

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Register_Memory	the_Register_Memory(
	// inputs
	.clk		(clk),
	.clk_en_a	(clk_en_port_a),
	.clk_en_b	(clk_en_port_b),

	.address_a	(addr_a),
	.address_b	(addr_b),
	.data_a		(port_a_data_in),
	.data_b		(port_b_data_in),
	.wren_a		(wr_en_a),
	.wren_b		(wr_en_b),

	// outputs
	.q_a		(port_a_data_out),
	.q_b		(port_b_data_out)
);
	//exemplar translate_off
//`ifdef NO_PLI
		defparam the_Register_Memory.INIT_REG_MEM_FILE = "reg_memory.dat";
//`else
//		defparam the_Register_Memory.INIT_REG_MEM_FILE = "reg_memory.hex";
//`endif
	//exemplar translate_on
	//synthesis read_comments_as_HDL on
	//defparam the_Register_Memory.INIT_REG_MEM_FILE = "reg_memory.mif";
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

