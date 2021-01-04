//*******************************************************************************
// Scalable Vector CPU Project -- Vector lane ALU
// Filename: ALU.v
// Author: Jason Yu
//
// The ALU consists of two adders implemented in two sets of ALMs.
// The first adder performs normal arithmetic instructions, 2nd adder implements
// logical operations using the pre-addition LUT of ALMs, and implements B-A for
// instructions that require a selection between A-B and B-A (vabs, vabsdiff).
// An additional "zero input" signal is used to zero one of the adder's inputs for certain
// instructions.  With these tweaks, the adders can be fitted compactly in ALMs,
// but still provide a good range of functionality.
//
// A result selector selects between results from the two adders, and a copy of
// input B to the ALU (needed for bypassing ALU, merge, max/min, shifts).
//
//  Copyright (C) 2007 Jason Yu
//
// ********************************************************************************

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


module ALU 
#(	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1
)
(
	// **** Inputs ****
	input		clk,
	input		[VPU_WIDTH-1:0]	inA,		// ALU input operands
	input		[VPU_WIDTH-1:0]	inB,		// ALU input operands
	input							vstype,		// whether instruction is VS-type
	input							x2_vstype,
	input		[`ALUOP_WIDTH-1:0]	ALUOp,		// ALU operation
	input		[`ALUOP_WIDTH-1:0]	x2_ALUOp,	// ALU operation for 2nd stage
	input		x2_flagIn,						// vector flag value
	input		x2_signedOp,					// whether comparison is signed or unsigned; 1 = signed, 0 = unsigned
	
	// **** Outputs ****
	output 		[VPU_WIDTH-1:0]	ALUResult,
	output reg	flagResult						//vector flag from comparison/overflow
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// 	Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	[VPU_WIDTH-1:0]	inB_x2;			// keep a copy of input B to the ALU
wire	[VPU_WIDTH-1:0]	inB_sig;
wire	[VPU_WIDTH-1:0]	adderInA;
wire	[VPU_WIDTH-1:0]	adderInB;
reg		[VPU_WIDTH-1:0]	logicInA;
wire	[VPU_WIDTH-1:0]	logicInB;
wire	[VPU_WIDTH-1:0]	arithResult;
wire	[VPU_WIDTH-1:0]	logicResult;
reg		[VPU_WIDTH-1:0]	arithlogicSelResult;
wire	[VPU_WIDTH-1:0]	adderOut_x2, adder2Out_x2;
wire	[VPU_WIDTH-1:0]	adderOut, adder2Out;
wire	adderCout, adderOverflow;
wire	adder2Cout, adder2Overflow;
wire	adderCout_x2, adder2Cout_x2;
wire	adderOverflow_x2, adder2Overflow_x2;
reg		adderOutNegative_x2, adder2OutNegative_x2;
wire	adderOutIsZero_x2;
reg		cmpflagResult;
wire	add_sub, add_sub2;
reg		zeroAdder1B;						// control signal to force input B of adder 1 to zero
reg		zeroAdder2B;						// control signal to force input B of adder 2 to zero
wire	inAmsb_x2, inBmsb_x2;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// 	Combinational assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------

assign	arithResult = adderOut_x2;
assign	logicResult = adder2Out_x2;
assign 	ALUResult = arithlogicSelResult;


// Pipeline input B
ff	#(.WIDTH(VPU_WIDTH)) pipe1_inB 	(.clk(clk), .en(1'b1), .clrn(1'b1), .d(inB), .q(inB_x2));

// Register output of Adder 1
ff	#(.WIDTH(VPU_WIDTH)) pipe2_adderOut 	(.clk(clk), .en(1'b1), .clrn(1'b1), .d(adderOut), .q(adderOut_x2));
ff	#(.WIDTH(1)) pipe2_adderCout 	(.clk(clk), .en(1'b1), .clrn(1'b1), .d(adderCout), .q(adderCout_x2));
ff	#(.WIDTH(1)) pipe2_adderOverflow 	(.clk(clk), .en(1'b1), .clrn(1'b1), .d(adderOverflow), .q(adderOverflow_x2));

// Register output of Adder 2
ff	#(.WIDTH(VPU_WIDTH)) pipe2_adder2Out 	(.clk(clk), .en(1'b1), .clrn(1'b1), .d(adder2Out), .q(adder2Out_x2));
ff	#(.WIDTH(1)) pipe2_adder2Cout 	(.clk(clk), .en(1'b1), .clrn(1'b1), .d(adder2Cout), .q(adder2Cout_x2));
ff	#(.WIDTH(1)) pipe2_adder2Overflow 	(.clk(clk), .en(1'b1), .clrn(1'b1), .d(adder2Overflow), .q(adder2Overflow_x2));

// Register the MSB of the two inputs
ff	#(.WIDTH(1)) pipe1_inAmsb 	(.clk(clk), .en(1'b1), .clrn(1'b1), .d(inA[VPU_WIDTH-1]), .q(inAmsb_x2));
ff	#(.WIDTH(1)) pipe1_inBmsb 	(.clk(clk), .en(1'b1), .clrn(1'b1), .d(inB[VPU_WIDTH-1]), .q(inBmsb_x2));


//---------------------------------------------------------------------------------------------------------------------------------------------------
// 	Select ALU Result
// - decode instruction to select result from one of three sources
// a) adder 1 output
// b) adder 2 output
// c) pipelined input B
//---------------------------------------------------------------------------------------------------------------------------------------------------

always @(*)
begin
	casex (x2_ALUOp)
		// sub_sv takes result from logicResult
		`ALUOP_VSUB_SV	: begin
`ifdef	VUPSHIFT
			// `ALUOP_VESHIFT
			// vector element shift also uses input B passthrough to save a pipeline register
			if ((x2_ALUOp == `ALUOP_VUPSHIFT) && (x2_vstype == 1'b0))
				arithlogicSelResult = inB_x2;
			else
`endif
				// vsub.vs
				arithlogicSelResult = logicResult;		// logic unit
		end
		
		// add/sub takes result from arithResult
		`ALUOP_VADD,
		`ALUOP_VSUB		:	arithlogicSelResult = arithResult;		// add/sub unit
		`OP_LOGIC		:	arithlogicSelResult = logicResult;		// logic unit
		`ALUOP_VMIN		:	arithlogicSelResult = adder2OutNegative_x2 ? inB_x2 : arithResult;			// vmin
		`ALUOP_VMAX		:	arithlogicSelResult = adder2OutNegative_x2 ? arithResult : inB_x2;			// vmax
		`ALUOP_VMERGE	:	arithlogicSelResult = x2_flagIn ? inB_x2 : arithResult;					// vmerge (arithResult = scalar)
		`ALUOP_VABS		:	arithlogicSelResult = adder2OutNegative_x2 ? arithResult : logicResult;	// vabs
		`ALUOP_VABSDIF	:	arithlogicSelResult = adder2OutNegative_x2 ? arithResult : logicResult;	// absolute difference (adder2OutNegative_x2 reflects result of inB-inA)
		`ALUOP_VINS		:	arithlogicSelResult = inB_x2;			// vector insert		
		
		// shift instructions use inB_x2 to preserve input without performing calculations
		`ALUOP_VSRA, `ALUOP_VSRA_SV,
		`ALUOP_VSLL, `ALUOP_VSLL_SV,
		`ALUOP_VSRL, `ALUOP_VSRL_SV,
		`ALUOP_VROT, `ALUOP_VROT_SV:	arithlogicSelResult = inB_x2;

		`ALUOP_VMERGE_SV:	arithlogicSelResult = x2_flagIn ? arithResult : inB_x2;					// vmerge (arithResult = scalar)
		
		default	:	arithlogicSelResult  = {VPU_WIDTH{1'bx}};
	endcase
end

// Select flag result
always @(*)
begin
	casex (x2_ALUOp)
		`ALUOP_CMP_LT	:	cmpflagResult = adderOutNegative_x2;
		`ALUOP_CMP_LE	:	cmpflagResult = adderOutNegative_x2 | adderOutIsZero_x2;
		`ALUOP_CMP_EQ	:	cmpflagResult = adderOutIsZero_x2;
		`ALUOP_CMP_NEQ	:	cmpflagResult = ~adderOutIsZero_x2;
		// ALUOP_CMP_LT_SV overlaps with VABS
		`ALUOP_CMP_LT_SV:	cmpflagResult = ~adderOutNegative_x2 & ~adderOutIsZero_x2;
		`ALUOP_CMP_LE_SV:	cmpflagResult = ~adderOutNegative_x2;
		default			:	cmpflagResult = 1'bx;		// need to change this later
	endcase
end

// 	Flag output
// - generate cout/overflow flags
always @(*)
begin
	casex (x2_ALUOp)
		// output overflow condition flag
		`ALUOP_VADD,
		`ALUOP_VSUB		:	flagResult = adderOverflow_x2;
		// sub.sv uses adder 2
		`ALUOP_VSUB_SV	:	flagResult = adder2Overflow_x2;
		// signed and unsigned max, min
		`ALUOP_VMAX,
		`ALUOP_VMIN		:	flagResult = adderOverflow_x2 | adder2Overflow_x2;
		// logic operations clear flag register
		`OP_LOGIC		:	flagResult = 1'b0;
		default			:	flagResult = cmpflagResult;
	endcase
end



//---------------------------------------------------------------------------------------------------------------------------------------------------
// 	Add/sub unit 1
// Operation (ALUOp[1:0]):
// 00: A+B
// 01: A-B
// 10: A+B
// 11: A-B
//---------------------------------------------------------------------------------------------------------------------------------------------------

// Generate add_sub signal (0 = subtraction, 1 = addition)
assign add_sub = (ALUOp[0] == 1'b1) ? 1'b0 : 1'b1;

// Input A selection
assign	adderInA = inA;
// Input B, complement if subtraction
assign	adderInB = (add_sub == 1'b0) ? ~inB_sig : inB_sig;

// zero adder input
assign	inB_sig = (zeroAdder1B == 1'b1) ? {VPU_WIDTH{1'b0}} : inB;

// Zero input B for adder1
always @(*)
begin
	casex (ALUOp)
		`ALUOP_VMIN		:	zeroAdder1B = 1'b1;
		`ALUOP_VMAX		:	zeroAdder1B = 1'b1;
		`ALUOP_VMERGE	:	zeroAdder1B = 1'b1;
		// vmerge.sv is only vs-type
		`ALUOP_VMERGE_SV:	if (vstype == 1'b1)
								zeroAdder1B = 1'b1;
							// vccacc, but really not needed as instruction doesn't use ALU
							else
								zeroAdder1B = 1'b0;
		// VABS is only VR-type
		`ALUOP_VABS		:	if (vstype == 1'b0)
								// vcmplt.sv
								zeroAdder1B = 1'b1;
							else
								//vabs
								zeroAdder1B = 1'b0;

		default	:	zeroAdder1B = 1'b0;
	endcase
end


lpm_add_sub		ALU_adder
(
	.dataa 		( adderInA ),
	.datab 		( adderInB ),
	.cin 		( ~add_sub ),				// manually generate cin for subtraction
	.result		( adderOut ),
	.cout 		( adderCout ),
	.overflow	( adderOverflow )
	// synopsys translate_off
	,
	.add_sub(),
	.clken (),
	.clock (),
	.aclr ()
	// synopsys translate_on
);
defparam	ALU_adder.lpm_width = VPU_WIDTH,
			ALU_adder.lpm_direction = "ADD";	// manually generate ~B and cin for subtraction


//---------------------------------------------------------------------------------------------------------------------------------------------------
// 	Comparison logic
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Zero detection
assign adderOutIsZero_x2 = (adderOut_x2 == 0) ? 1'b1 : 1'b0;	// result zero detection

// Result negative or overflow detection
always @(*) begin
	// signed comparison
	if (x2_signedOp) begin
		adderOutNegative_x2 = (adderOut_x2[VPU_WIDTH-1] == 1'b1) ? 1'b1 : 1'b0;
	end
	// result is negative for unsigned comparison if result underflows
	// unsigned subtraction underflows if cout = 0
	else
		adderOutNegative_x2 = ~adderCout_x2;
end

//---------------------------------------------------------------------------------------------------------------------------------------------------
// 	Add/sub unit 2
// Operation (ALUOp[1:0]):
// 00: (A & B) + 0
// 01: B-A
// 10: (A | B) + 0
// 11: (A ^ B) + 0
//---------------------------------------------------------------------------------------------------------------------------------------------------

// Generate add_sub signal (0 = subtraction, 1 = addition)
assign	add_sub2 = (ALUOp[1:0] == 2'b01) ? 1'b0 : 1'b1;
// Zero adder2 input B
assign	logicInB = (zeroAdder2B == 1'b1) ? 0 : inB;


// Input A selection
always @(*)
begin
	case (ALUOp[1:0])
		2'b00:	logicInA = inA & inB;
		2'b01:	logicInA = ~inA;		// complement for inB - inA
		2'b10:	logicInA = inA | inB;
		2'b11:	logicInA = inA ^ inB;
	endcase
end

// Zero input B for adder2
always @(*)
begin
	casex (ALUOp)
		`OP_LOGIC	:	zeroAdder2B = 1'b1;
		// vabs is only vr-type
		`ALUOP_VABS	:	if (vstype == 1'b0)
							zeroAdder2B = 1'b1;
						else
							zeroAdder2B = 1'b0;
		default		:	zeroAdder2B = 1'b0;
	endcase
end


// Adder 2
lpm_add_sub		ALU_adder2
(
	.dataa 		( logicInA ),
	.datab 		( logicInB ),
	.cin 		( ~add_sub2 ),				// manually generate cin for subtraction
	.result		( adder2Out ),
	.cout 		( adder2Cout ),
	.overflow	( adder2Overflow )
	// synopsys translate_off
	,
	.add_sub	(),
	.clken 		(),
	.clock 		(),
	.aclr 		()
	// synopsys translate_on
);
defparam	ALU_adder2.lpm_width = VPU_WIDTH,
			ALU_adder2.lpm_direction = "ADD";	// manually generate ~B and cin for subtraction


// Whether adder2Out is negative
// Result negative or overflow detection
always @(*) begin
	// signed comparison
	if (x2_signedOp) begin
		// signed 
		adder2OutNegative_x2 = (adder2Out_x2[VPU_WIDTH-1] == 1'b1) ? 1'b1 : 1'b0;
	end
	// result is negative for unsigned comparison if result underflows
	// unsigned subtraction underflows if cout = 0
	else
		adder2OutNegative_x2 = ~adder2Cout_x2;
end


endmodule
