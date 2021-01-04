// ****************************************
// Scalable Vector CPU Project
// Filename: components.v
// Author: Jason Yu
//
// Miscellaneous components
//
//  Copyright (C) 2007 Jason Yu
//
// ****************************************

// Quartus Verilog Template
// Clearable flipflop with enable
// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on


module ff
	#(	parameter	WIDTH = 32)

	(
		d,
		clk, 
		clrn,
		en,
		q
	);


	input [WIDTH-1:0]	d;
	input clk;
	input clrn;
	input en;
	output [WIDTH-1:0]	q;
   
	reg [WIDTH-1:0]	q_reg;

	always @ (posedge clk or negedge clrn)
	begin
		if (clrn == 1'b0)
		begin		
			q_reg <= 'b0;
		end
		else if (clk == 1'b1)
		begin
			if (en == 1'b1)
				q_reg <= d;
			else
				q_reg <= q_reg;
			end
		end

	assign q = q_reg;

endmodule



// ************************************
// **** Flipflop with synchronous clear ****
// ************************************

module ff_sclr
	#(	parameter	WIDTH = 32)

	(
		d,
		clk, 
		clrn,
		sclr,
		en,
		q
	);


	input [WIDTH-1:0]	d;
	input clk;
	input clrn;
	input sclr;
	input en;
	output [WIDTH-1:0]	q;
   
	reg [WIDTH-1:0]	q_reg;

	always @ (posedge clk or negedge clrn)
	begin
		if (clrn == 1'b0)
		begin		
			q_reg <= 'b0;
		end
		else if (clk == 1'b1)
		begin
			if (en == 1'b1)
				if (sclr == 1'b1)
					q_reg <= 'b0;
				else
					q_reg <= d;
			else
				q_reg <= q_reg;
			end
		end

	assign q = q_reg;

endmodule


// ************************************
// **** Flipflop with synchronous clear ****
// ************************************

module pipeline_reg_sclr
	#(	parameter	WIDTH = 32)

	(
		d,
		clk, 
		clrn,
		sclr,
		en,
		q
	);


	input [WIDTH-1:0]	d;
	input clk;
	input clrn;
	input sclr;
	input en;
	output [WIDTH-1:0]	q;
   
	reg [WIDTH-1:0]	q_reg;

	always @ (posedge clk or negedge clrn)
	begin
		if (clrn == 1'b0)
		begin		
			q_reg <= 'b0;
		end
		else if (clk == 1'b1)
		begin
			if ((en == 1'b1) || (sclr == 1'b1))
				if (sclr == 1'b1)
					q_reg <= 'b0;
				else
					q_reg <= d;
			else
				q_reg <= q_reg;
			end
		end

	assign q = q_reg;

endmodule


// **************************
// **** Onehot decoder ****
// **************************
`timescale 1ns/100ps

module onehotdecode
	(
		encoded,
		onehot
	);
	
	parameter	ENCODEDWIDTH = 3;
	
	input		[ENCODEDWIDTH-1:0]	encoded;
	output reg	[2**ENCODEDWIDTH-1:0]	onehot;

	integer i ;
 	
 	always @ (*)
 	begin
 	  	onehot = 0;
 	  	for (i = 0; i < 2**ENCODEDWIDTH; i = i + 1) begin
 	  		if (i == encoded)
				onehot = 2**i;		// exponent function
		end
 	end

endmodule


// ********************************
// **** Onehot decoder with enable****
// ********************************
`timescale 1ns/100ps

module onehotdecode_en
	(
		en,
		encoded,
		onehot
	);
	
	parameter	ENCODEDWIDTH = 3;
	
	input		en;		// decode enable
	input		[ENCODEDWIDTH-1:0]	encoded;
	output reg	[2**ENCODEDWIDTH-1:0]	onehot;

	integer i ;
 	
 	always @ (*)
 	begin
 	  	onehot = 0;
		i = 0;
		if (en == 1'b1) begin
	 	  	for (i = 0; i < 2**ENCODEDWIDTH; i = i + 1) begin
	 	  		if (i == encoded)
					onehot = 2**i;		// exponent function
			end
		end
 	end

endmodule


// **************************
// **** Parameterized MUX ****
// **************************
`timescale 1ns/100ps

module muxMto1
	#(	parameter N = 8, // number of bits wide
		parameter M = 4, // number of inputs
		parameter S = 2  // number of select lines
	)
	(Z, SEL, D);

	localparam W = M * N;

	`define DTOTAL W-1: 0
	`define DWIDTH N-1: 0
	`define SELW S-1: 0
	`define WORDS M-1: 0

	input [`DTOTAL] D;
	input [`SELW]	SEL;
	output [`DWIDTH] Z;

	integer i;
	reg [`DWIDTH] tmp, Z; // tmp will be use to minimize events

	// for bits in the width
	always @(SEL or D) begin
		// add logic to handle corner case of no select needed (direct passthrough)
		// should be optimized away at compile time if not needed?
		if (S == 0) begin
			tmp = {N{1'bx}};
			Z = D;
		end
		// normal operation
		else begin
			for( i= 0; i < N; i = i + 1)
				tmp[i] = D[ N* SEL + i];
			Z = tmp;
		end
	end

endmodule
