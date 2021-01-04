//---------------------------------------------------------------------------------------------------------------------------------------------------
// Scalable Vector CPU Project -- ISA-related Definitions
// Filename: isa_def.v
// Author: Jason Yu
//
// This file defines constants related to the ISA such as opcodes, number of registers
//
//  Copyright (C) 2007 Jason Yu
//
//---------------------------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------------------------------------
// VPU Control Registers
//---------------------------------------------------------------------------------------------------------------------------------------------------
`define		VC_NVBASE		16
`define		VC_NVSTRIDE		8
`define		VC_NVINC		8

`define		CREG_MERGE		15
`define 	CREG_OVERFLOW	16
// `define		CREG_CARRY		16

`define		ISA_MAXLANES	128		// maximum number of lanes supported by the ISA

//---------------------------------------------------------------------------------------------------------------------------------------------------
// VPU Control Register Mappings
//---------------------------------------------------------------------------------------------------------------------------------------------------
`define		VPU_NCTRLREG	64
`define		VPU_NCTRLREG_OTHER	32	// number of control registers excluding vbase, vstride, vinc

// Read-write control registers
`define		VC_VL			0		// vector length
`define		VC_VPUW			1		// virtual processor width
`define		VC_VINDEX		2		// element index for scalar insert/extract
// Read only control registers
`define		VC_ACCNCOPY		28		// how many times to execute vccacc/vcczacc to reduce entire vector
`define		VC_NLANE		29		// number of vector lanes
`define		VC_MVL			30		// maximum vector length
`define		VC_LOGMVL		31		// base 2 logarithm of MVL


`define		VC_VSTRIDE		32, 33, 34, 35, 36, 37, 38, 39
`define		VC_VSTRIDE_FIRST	32
`define		VC_VINC			40, 41, 42, 43, 44, 45, 46, 47
`define		VC_VINC_FIRST	40
`define		VC_VBASE		48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63
`define		VC_VBASE_FIRST	48


//---------------------------------------------------------------------------------------------------------------------------------------------------
// VPU Registers
//---------------------------------------------------------------------------------------------------------------------------------------------------

`define	VRF_NREG				64		// number of registers in vector register file
// `define	VRF_NREGADDR			6

`define	FLAG_NREG				32		// number of reginsters in vector flag register file
`define	FLAG_NREGADDR 			5
`define	FLAG_ON_VALUE			1'b0		// flag value that represents enable (do calculation)

//---------------------------------------------------------------------------------------------------------------------------------------------------
//  Opcode
//---------------------------------------------------------------------------------------------------------------------------------------------------

`define	OP_VRFWRADDR 		31:26
`define	OP_VRFRDADDR1		25:20
`define	OP_VRFRDADDR2		19:14
`define	OP_SCALAR_RDADDR	19:15		// scalar register read address for scalar source instructions

`define	OP_VFLAGWRADDR		30:26
`define	OP_VFLAGRDADDR1		24:20
`define	OP_VFLAGRDADDR2		18:14

`define	OP_FUNC				11:6
`define	OP_OPCODE			5:0
`define OP_XMASK			13
`define	OP_OPX				12

`define	OP_VMEMWRADDR		31:26		// same as OP_VRFWRADDR
`define	OP_VBASE			17:14		// vector memory address base
`define	OP_VSTRIDE			22:19		// vector memory stride
`define	OP_VINC				25:23		// memory access base address increment
`define	OP_VOFFSET			25:20		// indexed offset

`define	OP_SSTYPE_SREG		19:15
`define	OP_SSTYPE_VDREG		31:26

`define	OP_SDTYPE_SREG		31:27
`define	OP_SDTYPE_VSREG		25:20

`define	OP_SCALAR_RFWRADDR	31:27		// bit positions in opcode for scalar register write address
`define	OP_FUNC_MEMWIDTH	11:9		// portion of opcode that specifies width of data in memory access

`define	OP_SIGNBIT			3			// the sign bit for memory and ALU instrucionts
`define	OPCODE_WIDTH		6
`define	ALUOP_WIDTH			6

//---------------------------------------------------------------------------------------------------------------------------------------------------
//  Instruction Decode
//---------------------------------------------------------------------------------------------------------------------------------------------------
`define	NOP_OPCODE			32'h0000703a		// vand v0, v0, v0
`define	NOP_FUNC			(6'b000100)			// vand
`define	ALUOP_VSYNC			(6'b000100)			// set vsync to NOP; not needed in this implementation

// Instruction types
`define	OPCODE_VRTYPE		(6'h3D)
`define	OPX_VRTYPE			(1'b0)
`define	OPCODE_VSTYPE		(6'h3D)
`define	OPX_VSTYPE			(1'b1)
`define	OPCODE_VMTYPE		(6'h3F)
`define	OPX_VMTYPE			(1'b0)
`define	OPCODE_MISCTYPE		(6'h3E)
`define	OPX_MISCTYPE		(1'b1)
`define	OPCODE_FLAGTYPE		(6'h3E)
`define	OPX_FLAGTYPE		(1'b1)

// function field for processor internal data transfer instructions
`define	FUNC_MEM_XFERLOADQVRF			(6'b111111)			// associated FUNC field
`define	FUNC_MEM_XFERLOADQVFLAG			(6'b111110)
`define	FUNC_MEM_XFERLOADQVRF_NOMASK	(6'b111101)			// unmaskable transfer
`define	FUNC_MEM_XFERLOADQVRF_VINS		(6'b111100)			// vins.vv, uses vindex as first index to store

// opcode for processor internal data transfer instructions
`define	OPCODE_MEM_XFERLOADQVRF			(32'h00000FFF)		// special instruction to transfer data from load data Q to VRF
`define	OPCODE_MEM_XFERLOADQVFLAG		(32'h00000FBF)
`define	OPCODE_MEM_XFERLOADQVRF_NOMASK	(32'h00000F7F)		// unmaskable transfer
`define	OPCODE_MEM_XFERLOADQVRF_VINS	(32'h00000F3F)		// vins.vv, uses vindex as first index to store

// `define	OPCODE_MEM_XFERVRFSTOREQ	(32'h000003FF)		// special instruction to transfer data from VRF to store data Q
// `define	FUNC_MEM_XFERVRFSTOREQ		(6'b001111)			// associated FUNC field

// Flag instructions
`define	FUNC_FLAGLOGIC				(6'b000???)				// flag logical opeartions, NOT including vfset/vfclr

// Memory instructions: bits for determining size of data in memory
`define	OP_MEMDATASIZE				11:9
`define	FUNC_MEMBYTE				(3'b00?)
`define	FUNC_MEMHALFWORD			(3'b01?)
`define	FUNC_MEMWORD				(3'b100)

// Memory opcode
`define	FUNC_MEM_UNITLOAD			(6'b0??000), (6'b100000)
`define	FUNC_MEM_UNITSTORE			(6'b0??001), (6'b100001)
`define	FUNC_MEM_STRIDEDLOAD		(6'b0??010), (6'b100010)
`define	FUNC_MEM_STRIDEDSTORE		(6'b0??011), (6'b100011)
`define	FUNC_MEM_INDEXEDLOAD		(6'b0??100), (6'b100100)
`define	FUNC_MEM_INDEXEDSTORE		(6'b0??101), (6'b100101), (6'b0??110)
`define	FUNC_MEM_LOCALLOAD			(6'b110000)
`define	FUNC_MEM_LOCALSTORE			(6'b110001)
`define	FUNC_FLAGLOAD				(6'b110010)
`define	FUNC_FLAGSTORE				(6'b110011)


// **** Instruction classes ****
// ALU Operations
// `define	OP_ADDSUB					(6'b00?00?)					// add/sub instructions
//`define	OP_ADDSUB_SV				(6'b11?001)					// VS-type add/sub instructions
`define OP_LOGIC					(6'b000100), (6'b00011?)	// logic instructions
`define	ALUOP_VADD					(6'b00?000)		// addition
`define	ALUOP_VSUB					(6'b00?001)		// subtraction
`define	ALUOP_VSUB_SV				(6'b11?001)					// VS-type subtraction


// **** Specific instructions ****
`define	ALUOP_VMAXMIN			(6'b10??01)
`define ALUOP_VMIN				(6'b10?101)		// vmin
`define ALUOP_VMAX				(6'b10?001)		// vmax
`define ALUOP_VMERGE			(6'b011000)
`define	ALUOP_VMERGE_SV			(6'b111000)		// SV-type merge instruction
// `define	ALUOP_VMERGESV_VESHIFT	(6'b111000)

`define ALUOP_VABS				(6'b110101)		// vabs
`define ALUOP_VABSDIF			(6'b000101)		// absolute difference

`define	ALUOP_MULTHIGH			(6'b10?110)		// multiplication: multiply upper half, or multiply full and store upper half
`define	ALUOP_MULTLOW			(6'b10?111)		// multiplication: multiply lower half, or multiply full and store lower half

`define	ALUOP_VSRA				(6'b010000)		// shift right arithmetic
`define	ALUOP_VSRA_SV			(6'b110000)		// shift right arithmetic; scalar-vector
`define	SHIFT_CMD_VSRA			(3'b000)		// lower 3 bits of ALUOp
`define	ALUOP_VSLL				(6'b010010)		// shift left logical
`define	ALUOP_VSLL_SV			(6'b110010)		// shift left logical; scalar-vector
`define	SHIFT_CMD_VSLL			(3'b010)		// lower 3 bits of ALUOp
`define	ALUOP_VSRL				(6'b010011)		// shift right logical
`define	ALUOP_VSRL_SV			(6'b110011)		// shift right logical; scalar-vector
`define	SHIFT_CMD_VSRL			(3'b011)		// lower 3 bits of ALUOp
`define	ALUOP_VROT				(6'b010100)		// barrel rotate
`define	ALUOP_VROT_SV			(6'b110100)		// barrel rotate; scalar-vector
`define	SHIFT_CMD_VROT			(3'b100)		// lower 3 bits of ALUOp

// ---- Comparison types ----
`define	ALUOP_CMP_LTTYPE		(6'b01?1?1)		// covers less than, and less than or equal
`define	ALUOP_CMP_EQTYPE		(6'b01?001)		// covers eq, neq

// Comparison instructions
`define ALUOP_CMP_EQ			(6'b010001)		// compare equal
`define ALUOP_CMP_NEQ			(6'b011001)		// compare not equal
`define ALUOP_CMP_LT			(6'b01?101)		// compare less than, signed and unsigned
`define ALUOP_CMP_LT_SV			(6'b11?101)		// sv-type less than, signed and unsigned
`define ALUOP_CMP_LE			(6'b01?111)		// compare less than or equal, signed and unsigned
`define ALUOP_CMP_LE_SV			(6'b11?111)		// sv-type less than equal, signed and unsigned

// Move between scalar and vector control registers
`define	FUNC_MSTC				(6'b101000)		// move scalar to control
`define	FUNC_MCTS				(6'b101001)		// move control to scalar

// Multiply accumulate instructions
`define	ALUOP_VMACC				(6'b00?011)		// multiply accumulate
`define	ALUOP_VZACC				(6'b110001)		// zero accumulators
`define	ALUOP_VCCACC			(6'b110000)		// compress copy from accumulators
`define	ALUOP_VCCZACC			(6'b111000)		// compress copy from accumulators and zero accumulators

// Vector element shifting
`define	ALUOP_VUPSHIFT			(6'b110001)

// Insert, extract
`define	ALUOP_VEXT				(6'b100010)		// vector extract
`define	ALUOP_VEXTU_VS			(6'b101010)		// unsigned vector-scalar extract
`define	ALUOP_VINS				(6'b100011)		// vector insert


//---------------------------------------------------------------------------------------------------------------------------------------------------
//  Flag Unit definitions
//---------------------------------------------------------------------------------------------------------------------------------------------------

// Vector flag instructions
`define	ALUOP_FLAG_VFCLR		(6'b000000)
`define ALUOP_FLAG_VFSET		(6'b000001)
`define	ALUOP_FLAG_VFAND		(6'b000100)
`define	ALUOP_FLAG_VFNOR		(6'b000101)
`define	ALUOP_FLAG_VFOR			(6'b000110)
`define	ALUOP_FLAG_VFXOR		(6'b000111)

`define	ALUOP_VFINS_VS			(6'b011010)
`define	ALUOP_FLAG_VFAND_VS		(6'b011100)
`define	ALUOP_FLAG_VFNOR_VS		(6'b011101)
`define	ALUOP_FLAG_VFOR_VS		(6'b011110)
`define	ALUOP_FLAG_VFXOR_VS		(6'b011111)

/*****************************************************************************
  * Copyright (C) 2007 Jason Yu	*
 *****************************************************************************/