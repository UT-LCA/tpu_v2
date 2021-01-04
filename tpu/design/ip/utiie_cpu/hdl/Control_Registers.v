/*****************************************************************************
 *                                                                           *
 * Module:       Control_Registers                                           *
 * Description:                                                              *
 *      This module contains the control registers and performs reads and    *
 *   writes from/to these control registers.                                 *
 *                                                                           *
 *****************************************************************************/

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on

/*
Control_Registers the_Control_Registers (
	// Inputs
	.clk				(clk),
	.reset_n			(reset_n),

	.allow_ctrl_regs_wr	(),
	.ctrl_reg_data_in	(),
	.d_irq				(),
	.decoded_signals	(),
	.n_from_ir			(),
	.opcode				(),
	.stall_pipeline		(),
	
	// Bidirectionals

	// Outputs
	.ctrl_reg_data_out	(),
	.interrupt_active	(),
	.read_ctrl_regs		()
);
defparam the_Control_Registers.CPU_ID = CPU_ID;
*/

module Control_Registers (
	// Inputs
	clk,
	reset_n,

	allow_ctrl_regs_wr,
	ctrl_reg_data_in,
	d_irq,
	decoded_signals,
	n_from_ir,
	opcode,
	stall_pipeline,
	
	// Bidirectionals

	// Outputs
	ctrl_reg_data_out,
	interrupt_active,
	read_ctrl_regs
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/

parameter CPU_ID = 32'h00000000;

/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input			clk;
input			reset_n;

input			allow_ctrl_regs_wr;
input	[31:0]	ctrl_reg_data_in;
input	[31:0]	d_irq;
input	[7:0]	decoded_signals;
input	[4:0]	n_from_ir;
input	[5:0]	opcode;
input			stall_pipeline;

// Bidirectionals

// Outputs
output	[31:0]	ctrl_reg_data_out;
output			interrupt_active;
output			read_ctrl_regs;

/*****************************************************************************
 *                 Internal wires and registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire	[31:0]	ctrl_reg_5; // Unique Processor ID
wire	[31:0]	masked_irq;

wire	[1:0]	read_low_ctrl_regs;
wire	[1:0]	read_selectors;
wire			restore_ctrl_reg_0;
wire			save_ctrl_reg_0_to_ctrl_reg_1;
wire			save_ctrl_reg_0_to_ctrl_reg_2;
wire			set_PIE_to_zero;
wire			write_en;
wire			write_en_ctrl_reg_0;
wire			write_en_ctrl_reg_1;
wire			write_en_ctrl_reg_2;
wire			write_en_ctrl_reg_3;

// Internal Registers
reg		[ 1:0]	ctrl_reg_0; // Status Register
reg 	[ 1:0]	ctrl_reg_1; // Exception Status Register
reg 	[ 1:0]	ctrl_reg_2; // Break Status Register
reg		[31:0]	ctrl_reg_3; // Interrupt Mask Register

// State Machine Registers

/*****************************************************************************
 *                         Finite State Machine(s)                           *
 *****************************************************************************/

/*****************************************************************************
 *                             Sequential logic                              *
 *****************************************************************************/

// always @ (posedge clk)
always @ (posedge clk, negedge reset_n)
begin
	if (reset_n == 1'b0)
	begin
		ctrl_reg_0 <= 2'b00;
	end
	else if (stall_pipeline == 1'b0) 
	begin
		if (write_en_ctrl_reg_0 == 1'b1)
		begin
			ctrl_reg_0 <= ctrl_reg_data_in[1:0];
		end
		else if (set_PIE_to_zero == 1'b1)
		begin
			ctrl_reg_0[0] <= 1'b0;
		end
		else if (restore_ctrl_reg_0 == 1'b1)
		begin
			if (opcode[3] == 1'b0)
			begin
				ctrl_reg_0 <= ctrl_reg_1;
			end
			else
			begin
				ctrl_reg_0 <= ctrl_reg_2;
			end
		end
	end
end

// always @ (posedge clk)
always @ (posedge clk, negedge reset_n)
begin
	if (reset_n == 1'b0)
	begin
		ctrl_reg_1 <= 2'b00;
	end
	else if (stall_pipeline == 1'b0) 
	begin
		if (write_en_ctrl_reg_1 == 1'b1)
		begin
			ctrl_reg_1 <= ctrl_reg_data_in[1:0];
		end
		else if (save_ctrl_reg_0_to_ctrl_reg_1 == 1'b1)
		begin
			ctrl_reg_1 <= ctrl_reg_0;
		end
	end
end

// always @ (posedge clk)
always @ (posedge clk, negedge reset_n)
begin
	if (reset_n == 1'b0)
	begin
		ctrl_reg_2 <= 2'b00;
	end
	else if (stall_pipeline == 1'b0) 
	begin
		if (write_en_ctrl_reg_2 == 1'b1)
		begin
			ctrl_reg_2 <= ctrl_reg_data_in[1:0];
		end
		else if (save_ctrl_reg_0_to_ctrl_reg_2 == 1'b1)
		begin
			ctrl_reg_2 <= ctrl_reg_0;
		end
	end
end

// always @ (posedge clk)
always @ (posedge clk, negedge reset_n)
begin
	if (reset_n == 1'b0)
	begin
		ctrl_reg_3 <= 32'h00000000;
	end
	else if (stall_pipeline == 1'b0) 
	begin
		if (write_en_ctrl_reg_3 == 1'b1)
		begin
			ctrl_reg_3 <= ctrl_reg_data_in;
		end
	end
end

/*****************************************************************************
 *                            Combinational logic                            *
 *****************************************************************************/

assign ctrl_reg_5	  = CPU_ID;
assign read_selectors = (n_from_ir[3:0] == 4'h3) ?  2'h1 :
						(n_from_ir[3:0] == 4'h4) ?  2'h2 :
						(n_from_ir[3:0] == 4'h5) ?  2'h3 :
													2'b0;
/*
assign ctrl_reg_data_out[31:2] = (read_selectors == 2'h0) ? 30'h00000000 :
								 (read_selectors == 2'h1) ? ctrl_reg_3[31:2] :
								 (read_selectors == 2'h2) ? masked_irq[31:2] :
															ctrl_reg_5[31:2];

assign ctrl_reg_data_out[1:0] =  (read_selectors == 2'h1) ? ctrl_reg_3[1:0] :
								 (read_selectors == 2'h2) ? masked_irq[1:0] :
								 (read_selectors == 2'h3) ? ctrl_reg_5[1:0] :
								 (n_from_ir[1:0] == 2'h0) ? ctrl_reg_0 :
								 (n_from_ir[1:0] == 2'h1) ? ctrl_reg_1 :
															ctrl_reg_2;
*/
assign read_low_ctrl_regs = 
			({2{~n_from_ir[1]}} & {2{~n_from_ir[0]}} & ctrl_reg_0) |
			({2{~n_from_ir[1]}} & {2{ n_from_ir[0]}} & ctrl_reg_1) |
			({2{ n_from_ir[1]}} & {2{~n_from_ir[0]}} & ctrl_reg_2);

		
assign ctrl_reg_data_out[31:2] =
			({30{~read_selectors[1]}} & {30{ read_selectors[0]}} & ctrl_reg_3[31:2]) |
			({30{ read_selectors[1]}} & {30{~read_selectors[0]}} & masked_irq[31:2]) |
			({30{ read_selectors[1]}} & {30{ read_selectors[0]}} & ctrl_reg_5[31:2]);

assign ctrl_reg_data_out[1:0] =  
			({2{~read_selectors[1]}} & {2{~read_selectors[0]}} & read_low_ctrl_regs) |
			({2{~read_selectors[1]}} & {2{ read_selectors[0]}} & ctrl_reg_3[1:0]) |
			({2{ read_selectors[1]}} & {2{~read_selectors[0]}} & masked_irq[1:0]) |
			({2{ read_selectors[1]}} & {2{ read_selectors[0]}} & ctrl_reg_5[1:0]);


assign interrupt_active	= (|(masked_irq)) & (ctrl_reg_0[0]);
assign masked_irq		= ctrl_reg_3 & d_irq;
assign read_ctrl_regs	= (decoded_signals[3:0] == 4'h8) ? 
							decoded_signals[7] : 1'b0; 
	


assign restore_ctrl_reg_0 = 
		((opcode[2:0] == 3'h1) && (decoded_signals[3:0] == 4'hB)) ? 
			allow_ctrl_regs_wr : 1'b0;

assign save_ctrl_reg_0_to_ctrl_reg_1 = 
		((decoded_signals[7:6] == 2'h2) && (decoded_signals[3:1] == 3'h5)) ? 
			allow_ctrl_regs_wr : 1'b0;

assign save_ctrl_reg_0_to_ctrl_reg_2 =
		((decoded_signals[7:6] == 2'h3) && (decoded_signals[3:0] == 4'hB)) ? 
			allow_ctrl_regs_wr : 1'b0;

assign set_PIE_to_zero =	save_ctrl_reg_0_to_ctrl_reg_1 | 
							save_ctrl_reg_0_to_ctrl_reg_2;

assign write_en = 
		((decoded_signals[3:0] == 4'h8) && (n_from_ir[4:2] == 3'h0)) ? 
			~decoded_signals[7] & allow_ctrl_regs_wr : 1'b0;

assign write_en_ctrl_reg_0 = (write_en & (n_from_ir[1:0] == 2'h0));
assign write_en_ctrl_reg_1 = (write_en & (n_from_ir[1:0] == 2'h1));
assign write_en_ctrl_reg_2 = (write_en & (n_from_ir[1:0] == 2'h2));
assign write_en_ctrl_reg_3 = (write_en & (n_from_ir[1:0] == 2'h3));

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

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

