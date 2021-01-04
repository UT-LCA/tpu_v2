//**************************************************************************************
// Emulate on-chip data memory
// Used only for simulation
//
//  Copyright (C) 2007 Jason Yu
//
//**************************************************************************************
`timescale 1ns / 100ps

module	dMemModel
#(
	// Vector processor primary parameters
	parameter	MEM_WIDTH			= 64,
	parameter	DMEM_NUMWORDS		= 128,
	
	parameter	MEMWIDTH_BYTES = MEM_WIDTH/8			// memory width in number of bytes
)

(
	input								clock,
	input		[MEMWIDTH_BYTES-1:0]	byteena,
	input		[MEM_WIDTH-1:0]			data,
	input		[31:0]					address,
	input								wren,
	output reg	[MEM_WIDTH-1:0]			q
);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
reg		[MEM_WIDTH-1:0]		wraddress_data;
reg		[31:0]				address_word;
reg		[MEM_WIDTH-1:0]		byteena_mask;
reg		[MEM_WIDTH-1:0]		data_mem[DMEM_NUMWORDS-1:0];		// the data memory

integer i;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Combinational assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Sequential logic
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Expand byte enable to bit-level mask
always @(*) begin
	for (i=0; i<MEM_WIDTH; i=i+1)
		byteena_mask[i] = byteena[i/8];
end

// Data memory
always @(posedge clock) begin
	// default assignments
	i=0;
	address_word = address/MEMWIDTH_BYTES;
	wraddress_data = {MEM_WIDTH{1'bx}};
	
	// read data
	q = data_mem[address_word];

	if (wren == 1'b1) begin
		// blocking assignment
		wraddress_data = data_mem[address_word];
		data_mem[address_word] = (data & byteena_mask) | (wraddress_data & ~byteena_mask);
	end
end


endmodule
