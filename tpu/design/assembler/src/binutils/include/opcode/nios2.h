/* nios2.h.  Altera New Jersey opcode list for GAS, the GNU assembler.

   Copyright (C) 2003
   by Nigel Gray (ngray@altera.com).

This file is part of GDB, GAS, and the GNU binutils.

GDB, GAS, and the GNU binutils are free software; you can redistribute
them and/or modify them under the terms of the GNU General Public
License as published by the Free Software Foundation; either version
1, or (at your option) any later version.

GDB, GAS, and the GNU binutils are distributed in the hope that they
will be useful, but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this file; see the file COPYING.  If not, write to the Free
Software Foundation, 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  */

#ifndef _NIOS2_H_
#define _NIOS2_H_


/****************************************************************************
 * This file contains structures, bit masks and shift counts used
 * by the GNU toolchain to define the New Jersey instruction set and
 * access various opcode fields.
 ****************************************************************************/

enum overflow_type
{
  call_target_overflow = 0,
  branch_target_overflow,
  address_offset_overflow,
  signed_immed16_overflow,
  unsigned_immed16_overflow,
  unsigned_immed5_overflow,
  custom_opcode_overflow,
  no_overflow
};

/*---------------------------------------------------------------------------
   This structure holds information for a particular instruction
  ---------------------------------------------------------------------------*/

/* match When assembling, this
     opcode is modified by the arguments to produce the actual opcode
     that is used.  If pinfo is INSN_MACRO, then this is 0.  */

/* mask If pinfo is not INSN_MACRO, then this is a bit mask for the
     relevant portions of the opcode when disassembling.  If the
     actual opcode anded with the match field equals the opcode field,
     then we have found the correct instruction.  If pinfo is
     INSN_MACRO, then this field is the macro identifier.  */

/* For a macro, this is INSN_MACRO.  Otherwise, it is a collection
     of bits describing the instruction, notably any relevant hazard
     information.  */

struct nios2_opcode
{
  const char *name;		/* The name of the instruction.  */
  const char *args;		/* A string describing the arguments for this instruction.  */
  const char *args_test;	/* Like args, but with an extra argument for the expected opcode */
  unsigned long num_args;	/* the number of arguments the instruction takes */
  unsigned long match;		/* The basic opcode for the instruction. */
  unsigned long mask;		/* mask for the opcode field of the instruction */
  unsigned long pinfo;		/* is this a real instruction or instruction macro */
  enum overflow_type overflow_msg;	/* msg template used to generate informative message when fixup overflows */
};

/* This value is used in the nios2_opcode.pinfo field to indicate that the instruction
   is a macro or pseudo-op. This requires special treatment by the assembler, and is
   used by the disassembler to determine whether to check for a nop */
#define NIOS2_INSN_MACRO 		0x80000000
#define NIOS2_INSN_MACRO_MOV	0x80000001
#define NIOS2_INSN_MACRO_MOVI	0x80000002
#define NIOS2_INSN_MACRO_MOVIA  0x80000004

#define NIOS2_INSN_RELAXABLE	0x40000000
#define NIOS2_INSN_UBRANCH		0x00000010
#define NIOS2_INSN_CBRANCH		0x00000020
#define NIOS2_INSN_CALL			0x00000040

#define NIOS2_INSN_ADDI			0x00000080
#define NIOS2_INSN_ANDI			0x00000100
#define NIOS2_INSN_ORI			0x00000200
#define NIOS2_INSN_XORI			0x00000400



/* Associates a register name ($6) with a 5-bit index (eg 6) */
struct nios2_reg
{
  const char *name;
  const int index;
  const int type;
};

/* -------------------------------------------------------------------------
    Bitfield masks for New Jersey instructions
   -------------------------------------------------------------------------*/

/* These are bit masks and shift counts to use to access the various
   fields of an instruction. */

/* Macros for getting and setting an instruction field */
#define GET_INSN_FIELD(X, i)     ((i) & OP_MASK_##X) >> OP_SH_##X
#define SET_INSN_FIELD(X, i, j)  (i) = ((i) &~ (OP_MASK_##X)) | ((j) << OP_SH_##X)


/*
   We include the auto-generated file nios2-isa.h and define the mask
   and shifts below in terms of those in nios2-isa.h. This ensures
   that the binutils and hardware are always in sync
*/

#include "nios2-isa.h"

#define OP_MASK_OP              (IW_OP_MASK << IW_OP_LSB)
#define OP_SH_OP                IW_OP_LSB


/* Masks and shifts for I-type instructions */

#define OP_MASK_IOP             (IW_OP_MASK << IW_OP_LSB)
#define OP_SH_IOP               IW_OP_LSB

#define OP_MASK_IMM16   		(IW_IMM16_MASK << IW_IMM16_LSB)
#define OP_SH_IMM16             IW_IMM16_LSB

#define OP_MASK_IRD             (IW_B_MASK << IW_B_LSB)	// the same as T for I-type
#define OP_SH_IRD               IW_B_LSB

#define OP_MASK_IRT             (IW_B_MASK << IW_B_LSB)
#define OP_SH_IRT               IW_B_LSB

#define OP_MASK_IRS             (IW_A_MASK << IW_A_LSB)
#define OP_SH_IRS               IW_A_LSB

/* Masks and shifts for R-type instructions */

#define OP_MASK_ROP             (IW_OP_MASK << IW_OP_LSB)
#define OP_SH_ROP               IW_OP_LSB

#define OP_MASK_ROPX    		(IW_OPX_MASK << IW_OPX_LSB)
#define OP_SH_ROPX              IW_OPX_LSB

#define OP_MASK_RRD             (IW_C_MASK << IW_C_LSB)
#define OP_SH_RRD               IW_C_LSB

#define OP_MASK_RRT             (IW_B_MASK << IW_B_LSB)
#define OP_SH_RRT               IW_B_LSB

#define OP_MASK_RRS             (IW_A_MASK << IW_A_LSB)
#define OP_SH_RRS               IW_A_LSB

/* Masks and shifts for J-type instructions */

#define OP_MASK_JOP             (IW_OP_MASK << IW_OP_LSB)
#define OP_SH_JOP               IW_OP_LSB

#define OP_MASK_IMM26   		(IW_IMM26_MASK << IW_IMM26_LSB)
#define OP_SH_IMM26             IW_IMM26_LSB

/* Masks and shifts for CTL instructions */

#define OP_MASK_RCTL    0x000007c0
#define OP_SH_RCTL              6

/* break instruction imm5 field */
#define OP_MASK_TRAP_IMM5 0x000007c0
#define OP_SH_TRAP_IMM5   6

/* instruction imm5 field */
#define OP_MASK_IMM5    		(IW_SHIFT_IMM5_MASK << IW_SHIFT_IMM5_LSB)
#define OP_SH_IMM5              IW_SHIFT_IMM5_LSB

/* cache operation fields (type j,i(s)) */
#define OP_MASK_CACHE_OPX       (IW_B_MASK << IW_B_LSB)
#define OP_SH_CACHE_OPX         IW_B_LSB
#define OP_MASK_CACHE_RRS       (IW_A_MASK << IW_A_LSB)
#define OP_SH_CACHE_RRS         IW_A_LSB

/* custom instruction masks */
#define OP_MASK_CUSTOM_A		0x00010000
#define OP_SH_CUSTOM_A				16

#define OP_MASK_CUSTOM_B		0x00008000
#define OP_SH_CUSTOM_B				15

#define OP_MASK_CUSTOM_C		0x00004000
#define OP_SH_CUSTOM_C				14

#define OP_MASK_CUSTOM_N		0x00003fc0
#define OP_SH_CUSTOM_N				6
#define OP_MAX_CUSTOM_N				255

/*
       The following macros define the opcode matches for each
       instruction
       code & OP_MASK_INST == OP_MATCH_INST
 */

/* OP instruction matches */
#define OP_MATCH_ADDI           OP_ADDI
#define OP_MATCH_ANDHI          OP_ANDHI
#define OP_MATCH_ANDI           OP_ANDI
#define OP_MATCH_BEQ            OP_BEQ
#define OP_MATCH_BGE            OP_BGE
#define OP_MATCH_BGEU           OP_BGEU
#define OP_MATCH_BLT            OP_BLT
#define OP_MATCH_BLTU           OP_BLTU
#define OP_MATCH_BNE            OP_BNE
#define OP_MATCH_BR             OP_BR
#define OP_MATCH_FLUSHD         OP_FLUSHD
#define OP_MATCH_FLUSHDA        OP_FLUSHDA
#define OP_MATCH_INITD          OP_INITD
#define OP_MATCH_CALL           OP_CALL
#define OP_MATCH_CMPEQI         OP_CMPEQI
#define OP_MATCH_CMPGEI         OP_CMPGEI
#define OP_MATCH_CMPGEUI        OP_CMPGEUI
#define OP_MATCH_CMPLTI         OP_CMPLTI
#define OP_MATCH_CMPLTUI        OP_CMPLTUI
#define OP_MATCH_CMPNEI         OP_CMPNEI
#define OP_MATCH_LDB            OP_LDB
#define OP_MATCH_LDBIO          OP_LDBIO
#define OP_MATCH_LDBU           OP_LDBU
#define OP_MATCH_LDBUIO         OP_LDBUIO
#define OP_MATCH_LDH            OP_LDH
#define OP_MATCH_LDHIO          OP_LDHIO
#define OP_MATCH_LDHU           OP_LDHU
#define OP_MATCH_LDHUIO         OP_LDHUIO
#define OP_MATCH_LDW            OP_LDW
#define OP_MATCH_LDWIO          OP_LDWIO
#define OP_MATCH_MULI           OP_MULI
#define OP_MATCH_OPX            OP_OPX
#define OP_MATCH_ORHI           OP_ORHI
#define OP_MATCH_ORI            OP_ORI
#define OP_MATCH_STB            OP_STB
#define OP_MATCH_STBIO          OP_STBIO
#define OP_MATCH_STH            OP_STH
#define OP_MATCH_STHIO          OP_STHIO
#define OP_MATCH_STW            OP_STW
#define OP_MATCH_STWIO          OP_STWIO
#define OP_MATCH_CUSTOM         OP_CUSTOM
#define OP_MATCH_XORHI          OP_XORHI
#define OP_MATCH_XORI           OP_XORI
#define OP_MATCH_OPX            OP_OPX

/* OPX instruction values */
#define OP_MATCH_ADD            ((OPX_ADD << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_AND            ((OPX_AND << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_BREAK          ((0x1e << 17) | (OPX_BREAK << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_BRET           ((0xf0000000) | (OPX_BRET << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_CALLR          ((0x1f << 17) | (OPX_CALLR << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_CMPEQ          ((OPX_CMPEQ << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_CMPGE          ((OPX_CMPGE << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_CMPGEU         ((OPX_CMPGEU << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_CMPLT          ((OPX_CMPLT << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_CMPLTU         ((OPX_CMPLTU << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_CMPNE          ((OPX_CMPNE << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_DIV            ((OPX_DIV << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_DIVU           ((OPX_DIVU << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_JMP            ((OPX_JMP << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_MUL            ((OPX_MUL << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_MULXSS         ((OPX_MULXSS << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_MULXSU         ((OPX_MULXSU << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_MULXUU         ((OPX_MULXUU << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_NEXTPC         ((OPX_NEXTPC << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_NOR            ((OPX_NOR << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_OR             ((OPX_OR << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_RDCTL          ((OPX_RDCTL << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_RET            ((0xf8000000) | (OPX_RET << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_ROL            ((OPX_ROL << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_ROLI           ((OPX_ROLI << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_ROR            ((OPX_ROR << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_SLL            ((OPX_SLL << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_SLLI           ((OPX_SLLI << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_SRA            ((OPX_SRA << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_SRAI           ((OPX_SRAI << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_SRL            ((OPX_SRL << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_SRLI           ((OPX_SRLI << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_SUB            ((OPX_SUB << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_SYNC           ((OPX_SYNC << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_TRAP           ((0x1d << 17) | (OPX_TRAP << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_ERET           ((0xe8000000) | (OPX_ERET << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_WRCTL          ((OPX_WRCTL << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_XOR            ((OPX_XOR << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_FLUSHI			((OPX_FLUSHI << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_FLUSHP			((OPX_FLUSHP << IW_OPX_LSB) | (OP_OPX))
#define OP_MATCH_INITI			((OPX_INITI << IW_OPX_LSB) | (OP_OPX))

/*
       Some unusual op masks
*/
#define OP_MASK_BREAK           ((OP_MASK_RRS | OP_MASK_RRT | OP_MASK_RRD | OP_MASK_ROPX | OP_MASK_OP) & 0xfffff03f)
#define OP_MASK_CALLR           ((OP_MASK_RRT | OP_MASK_RRD | OP_MASK_ROPX | OP_MASK_OP))
#define OP_MASK_JMP             ((OP_MASK_RRT | OP_MASK_RRD | OP_MASK_ROPX | OP_MASK_OP))
#define OP_MASK_SYNC            ((OP_MASK_RRT | OP_MASK_RRD | OP_MASK_ROPX | OP_MASK_OP))
#define OP_MASK_TRAP            ((OP_MASK_RRS | OP_MASK_RRT | OP_MASK_RRD | OP_MASK_ROPX | OP_MASK_OP) & 0xfffff83f)
#define OP_MASK_WRCTL           ((OP_MASK_RRT | OP_MASK_RRD | OP_MASK_ROPX | OP_MASK_OP))	/*& 0xfffff83f */
#define OP_MASK_NEXTPC          ((OP_MASK_RRS | OP_MASK_RRT | OP_MASK_ROPX | OP_MASK_OP))
#define OP_MASK_FLUSHI          ((OP_MASK_RRT | OP_MASK_RRD | OP_MASK_ROPX | OP_MASK_OP))
#define OP_MASK_INITI           ((OP_MASK_RRT | OP_MASK_RRD | OP_MASK_ROPX | OP_MASK_OP))

#define OP_MASK_ROLI            ((OP_MASK_RRT | OP_MASK_ROPX | OP_MASK_OP))
#define OP_MASK_SLLI            ((OP_MASK_RRT | OP_MASK_ROPX | OP_MASK_OP))
#define OP_MASK_SRAI            ((OP_MASK_RRT | OP_MASK_ROPX | OP_MASK_OP))
#define OP_MASK_SRLI            ((OP_MASK_RRT | OP_MASK_ROPX | OP_MASK_OP))
#define OP_MASK_RDCTL           ((OP_MASK_RRS | OP_MASK_RRT | OP_MASK_ROPX | OP_MASK_OP))	/*& 0xfffff83f */

#ifndef OP_MASK
#define OP_MASK                         0xffffffff
#endif

/* -------------------------------------------------------------------------
	Opcodes for the Vector-Coprocessor
   ------------------------------------------------------------------------- */
   
/* This section defines the opcodes for the Vector Co-processor instructions.
   The equivalent NIOS2 codes are defined in the autogenerated nios2-isa.h. */

/* Arithmetic and Logical Instructions */
/* Vector-Vector and Scalar-Vector Versions	*/
#define VOP_VADD 0
#define VOP_VSUB 1
#define VOP_VMAC 3
#define VOP_VAND 4
#define VOP_VABSDIFF 5
#define VOP_VOR 6
#define VOP_VXOR 7
#define VOP_VADDU 8
#define VOP_VSUBU 9
#define VOP_VMACU 11
#define VOP_VSRA 16
#define VOP_VCMPEQ 17
#define VOP_VSLL 18
#define VOP_VSRL 19
#define VOP_VROT 20
#define VOP_VCMPLT 21
#define VOP_VDIV 22
#define VOP_VCMPLE 23
#define VOP_VMERGE 24
#define VOP_VCMPNEQ 25
#define VOP_VCMPLTU 29
#define VOP_VDIVU 30
#define VOP_VCMPLEU 31
#define VOP_VMAX 33
#define VOP_VEXT 34
#define VOP_VINS 35
#define VOP_VMIN 37
#define VOP_VMULHI 38
#define VOP_VMULLO 39
#define VOP_VMAXU 41
#define VOP_VEXTU 42
#define VOP_VMINU 45
#define VOP_VMULHIU 46
#define VOP_VMULLOU 47
#define VOP_VCCACC 48
#define VOP_VUPSHIFT 49
#define VOP_VCOMPRESS 50
#define VOP_VEXPAND 51
#define VOP_VABS 53
#define VOP_VCCZACC 56
// #define VOP_VEXTHALF 57
// #define VOP_VCPERM 58

/* Scalar-vector versions */
#define VOP_VSRA_SV 48
#define VOP_VSUB_SV 49
#define VOP_VSLL_SV 50
#define VOP_VSRL_SV 51
#define VOP_VROT_SV 52
#define VOP_VCMPLT_SV 53
#define VOP_VDIV_SV 54
#define VOP_VCMPLE_SV 55
#define VOP_VMERGE_SV 56
#define VOP_VSUBU_SV 57
#define VOP_VCMPLTU_SV 61
#define VOP_VDIVU_SV 62
#define VOP_VCMPLEU_SV 63

/* Memory Instructions */
#define VOP_VLD_B 0
#define VOP_VST_B 1
#define VOP_VLDS_B 2
#define VOP_VSTS_B 3
#define VOP_VLDX_B 4
#define VOP_VSTX_B 5
#define VOP_VSTXO_B 6
#define VOP_VLDU_B 8
#define VOP_VLDSU_B 10
#define VOP_VLDXU_B 12
#define VOP_VLD_H 16
#define VOP_VST_H 17
#define VOP_VLDS_H 18
#define VOP_VSTS_H 19
#define VOP_VLDX_H 20
#define VOP_VSTX_H 21
#define VOP_VSTXO_H 22
#define VOP_VLDU_H 24
#define VOP_VLDSU_H 26
#define VOP_VLDXU_H 28
#define VOP_VLD_W 32
#define VOP_VST_W 33
#define VOP_VLDS_W 34
#define VOP_VSTS_W 35
#define VOP_VLDX_W 36
#define VOP_VSTX_W 37
#define VOP_VSTXO_W 38
// #define VOP_VLDLU 40
#define VOP_VLDL 48
#define VOP_VSTL 49
#define VOP_VFLD 50
#define VOP_VFST 51
// #define VOP_VSTL_VS 57

/* Flag Processing and Misc. Instructions */
#define VOP_VFCLR 0
#define VOP_VFSET 1
#define VOP_VFAND 4
#define VOP_VFNOR 5
#define VOP_VFOR 6
#define VOP_VFXOR 7
#define VOP_VFAND_VS 28
#define VOP_VFNOR_VS 29
#define VOP_VFOR_VS 30
#define VOP_VFXOR_VS 31
#define VOP_VFINS 26
#define VOP_VSYNC 32
#define VOP_VMSTC 40
#define VOP_VMCTS 41

/* OP field codes for the vector instruction types */
// || 13  MASK(1-bit)  13 || 12  OPX(1-bit)  12 || 11  FUNC(6-bit)  6 || 5  OP(6-bit)  0 ||
#define VOP_RTYPE 0x3d
#define VOP_RTYPE_S 0x103d
#define VOP_FIXPT 0x3e
#define VOP_FLAG 0x103e
#define VOP_MEMORY 0x3f
// currently used for local memory scalar store, vstl.vs
#define	VOP_MEMORY_VS	0X103f

//Helper names for vector and scalar registers
#define VECTOR_REG 1
#define SCALAR_REG 0

/* Register index offset value, add this offset to the index above for vmstc/vmcts vector instructions */
#define	VSTRIDE_OFFSET	32
#define	VINC_OFFSET		40
#define	VBASE_OFFSET	48


/* -------------------------------------------------------------------------
    Bitfield masks and instruction matches for the Vector Processor
   -------------------------------------------------------------------------*/

/* Macros for geting and setting an instruction field */
#define GET_VINSN_FIELD(X, i)     ((i) & VOP_MASK_##X) >> VOP_SH_##X
#define SET_VINSN_FIELD(X, i, j)  (i) = ((i) &~ (VOP_MASK_##X)) | ((j) << VOP_SH_##X)

/* General bit masks and shift counts */

#define IW_VOP_LSB				6
#define VOP_MASK_				0x1fff

/* Set the mask bit indicated by assembly instruction: 
instruction		Mask = vmask0
instruction.0	Mask = vmask0 (Not implemented yet)
instruction.1	Mask = vmask1							*/

#define IW_VPMASK_MASK			0x1
#define VOP_MASK_VPMASK			(IW_VPMASK_MASK << VOP_SH_VPMASK)
#define VOP_SH_VPMASK			13

/* Generic mask for any 6-bit vector register */
#define IW_VREG_MASK			0x3f

/* Generic mask for any 5-bit scalar register */
#define IW_SREG_MASK			0x1f



/* Arithmetic + Logical Instructions (= VR, SS, or SD Types) */
/* Note: Vector Flag registers can be used in place of normal vector registers in this
   instruction encoding, and they require no special treatment except that they are
   zero extended from 5-bits to 6-bits. */
   
/* Vector Registers */
#define VOP_MASK_VD				(IW_VREG_MASK << VOP_SH_VD)
#define VOP_SH_VD				26

#define VOP_MASK_VA				(IW_VREG_MASK << VOP_SH_VA)
#define VOP_SH_VA				20

#define VOP_MASK_VB				(IW_VREG_MASK << VOP_SH_VB)
#define VOP_SH_VB				14

/* Scalar register for SS Type */
#define VOP_MASK_RSS			(IW_SREG_MASK << VOP_SH_RSS)
#define VOP_SH_RSS				15

/* Scalar register for SD Type */
#define VOP_MASK_RSD			(IW_SREG_MASK << VOP_SH_RSD)
#define VOP_SH_RSD				27

/* Memory Instructions (= VM, VMS, VMX, or VMSB Types)	*/
#define IW_VINC_MASK			0x07
#define VOP_MASK_VINC			(IW_VINC_MASK << VOP_SH_VINC)
#define VOP_SH_VINC				23

#define IW_VBASE_MASK			0x1f
#define VOP_MASK_VBASE			(IW_VBASE_MASK << VOP_SH_VBASE)
#define VOP_SH_VBASE			14

#define IW_VSTRIDE_MASK			0x0f
#define VOP_MASK_VSTRIDE		(IW_VSTRIDE_MASK << VOP_SH_VSTRIDE)
#define VOP_SH_VSTRIDE			19


/* VOP instruction matches */
/* Note, in most cases .vs version of an instruction uses the same opcode as the .vv version 
   however, the OPX-bit=0 for vv, and 1 for vs */

/* Integer Instructions */
#define VOP_MATCH_VABS			((VOP_VABS << IW_VOP_LSB) | (VOP_RTYPE))

#define VOP_MATCH_VABSDIFF		((VOP_VABSDIFF << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VABSDIFF_VS	((VOP_VABSDIFF << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VADD			((VOP_VADD << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VADD_VS		((VOP_VADD << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VADDU			((VOP_VADDU << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VADDU_VS		((VOP_VADDU << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VSUB			((VOP_VSUB << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VSUB_VS		((VOP_VSUB << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VSUB_SV		((VOP_VSUB_SV << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VSUBU			((VOP_VSUBU << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VSUBU_VS		((VOP_VSUBU << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VSUBU_SV		((VOP_VSUBU_SV << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VMULHI		((VOP_VMULHI << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VMULHI_VS		((VOP_VMULHI << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VMULHIU		((VOP_VMULHIU << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VMULHIU_VS	((VOP_VMULHIU << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VMULLO		((VOP_VMULLO << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VMULLO_VS		((VOP_VMULLO << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VMULLOU		((VOP_VMULLOU << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VMULLOU_VS	((VOP_VMULLOU << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VDIV			((VOP_VDIV << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VDIV_VS		((VOP_VDIV << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VDIV_SV		((VOP_VDIV_SV << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VDIVU			((VOP_VDIVU << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VDIVU_VS		((VOP_VDIVU << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VDIVU_SV		((VOP_VDIVU_SV << IW_VOP_LSB) | (VOP_RTYPE_S))

/* Removed from ISA */
//#define VOP_MATCH_VMOD			((VOP_VMOD << IW_VOP_LSB) | (VOP_RTYPE))
//#define VOP_MATCH_VMOD_VS		((VOP_VMOD << IW_VOP_LSB) | (VOP_RTYPE_S))
//#define VOP_MATCH_VMOD_SV		((VOP_VMOD_SV << IW_VOP_LSB) | (VOP_RTYPE_S))
//#define VOP_MATCH_VMODU			((VOP_VMODU << IW_VOP_LSB) | (VOP_RTYPE))
//#define VOP_MATCH_VMODU_VS		((VOP_VMODU << IW_VOP_LSB) | (VOP_RTYPE_S))
//#define VOP_MATCH_VMODU_SV		((VOP_VMODU_SV << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VSRA			((VOP_VSRA << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VSRA_VS		((VOP_VSRA << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VSRA_SV		((VOP_VSRA_SV << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VMIN			((VOP_VMIN << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VMIN_VS		((VOP_VMIN << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VMINU			((VOP_VMINU << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VMINU_VS		((VOP_VMINU << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VMAX			((VOP_VMAX << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VMAX_VS		((VOP_VMAX << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VMAXU			((VOP_VMAXU << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VMAXU_VS		((VOP_VMAXU << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VCMPEQ		((VOP_VCMPEQ << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VCMPEQ_VS		((VOP_VCMPEQ << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VCMPNEQ		((VOP_VCMPNEQ << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VCMPNEQ_VS	((VOP_VCMPNEQ << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VCMPLT		((VOP_VCMPLT << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VCMPLT_VS		((VOP_VCMPLT << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VCMPLT_SV		((VOP_VCMPLT_SV << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VCMPLTU		((VOP_VCMPLTU << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VCMPLTU_VS	((VOP_VCMPLTU << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VCMPLTU_SV	((VOP_VCMPLTU_SV << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VCMPLE		((VOP_VCMPLE << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VCMPLE_VS		((VOP_VCMPLE << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VCMPLE_SV		((VOP_VCMPLE_SV << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VCMPLEU		((VOP_VCMPLEU << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VCMPLEU_VS	((VOP_VCMPLEU << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VCMPLEU_SV	((VOP_VCMPLEU_SV << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VMAC			((VOP_VMAC << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VMAC_VS		((VOP_VMAC << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VMACU		((VOP_VMACU << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VMACU_VS		((VOP_VMACU << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VCCACC		((VOP_VCCACC << IW_VOP_LSB) | (VOP_RTYPE))

#define VOP_MATCH_VCCZACC			((VOP_VCCZACC << IW_VOP_LSB) | (VOP_RTYPE))

/* Logical Instructions */
#define VOP_MATCH_VAND			((VOP_VAND << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VAND_VS		((VOP_VAND << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VOR			((VOP_VOR << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VOR_VS		((VOP_VOR << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VXOR			((VOP_VXOR << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VXOR_VS		((VOP_VXOR << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VSLL			((VOP_VSLL << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VSLL_VS		((VOP_VSLL << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VSLL_SV		((VOP_VSLL_SV << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VSRL			((VOP_VSRL << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VSRL_VS		((VOP_VSRL << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VSRL_SV		((VOP_VSRL_SV << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VROT			((VOP_VROT << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VROT_VS		((VOP_VROT << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VROT_SV		((VOP_VROT_SV << IW_VOP_LSB) | (VOP_RTYPE_S))

// #define VOP_MATCH_VCPERM		((VOP_VCPERM << IW_VOP_LSB) | (VOP_RTYPE))

/* Memory Instructions */
#define VOP_MATCH_VLD_B			((VOP_VLD_B << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VLD_H			((VOP_VLD_H << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VLD_W			((VOP_VLD_W << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VLDU_B		((VOP_VLDU_B << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VLDU_H		((VOP_VLDU_H << IW_VOP_LSB) | (VOP_MEMORY))
//#define VOP_MATCH_VLDU_W		((VOP_VLDU_W << IW_VOP_LSB) | (VOP_MEMORY))

#define VOP_MATCH_VST_B			((VOP_VST_B << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VST_H			((VOP_VST_H << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VST_W			((VOP_VST_W << IW_VOP_LSB) | (VOP_MEMORY))

#define VOP_MATCH_VLDS_B		((VOP_VLDS_B << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VLDS_H		((VOP_VLDS_H << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VLDS_W		((VOP_VLDS_W << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VLDSU_B		((VOP_VLDSU_B << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VLDSU_H		((VOP_VLDSU_H << IW_VOP_LSB) | (VOP_MEMORY))
//#define VOP_MATCH_VLDSU_W		((VOP_VLDSU_W << IW_VOP_LSB) | (VOP_MEMORY))

#define VOP_MATCH_VSTS_B		((VOP_VSTS_B << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VSTS_H		((VOP_VSTS_H << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VSTS_W		((VOP_VSTS_W << IW_VOP_LSB) | (VOP_MEMORY))

#define VOP_MATCH_VLDX_B		((VOP_VLDX_B << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VLDX_H		((VOP_VLDX_H << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VLDX_W		((VOP_VLDX_W << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VLDXU_B		((VOP_VLDXU_B << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VLDXU_H		((VOP_VLDXU_H << IW_VOP_LSB) | (VOP_MEMORY))
//#define VOP_MATCH_VLDXU_W		((VOP_VLDXU_W << IW_VOP_LSB) | (VOP_MEMORY))

#define VOP_MATCH_VSTX_B		((VOP_VSTX_B << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VSTX_H		((VOP_VSTX_H << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VSTX_W		((VOP_VSTX_W << IW_VOP_LSB) | (VOP_MEMORY))

#define VOP_MATCH_VSTXO_B		((VOP_VSTXO_B << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VSTXO_H		((VOP_VSTXO_H << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VSTXO_W		((VOP_VSTXO_W << IW_VOP_LSB) | (VOP_MEMORY))

#define VOP_MATCH_VLDL			((VOP_VLDL << IW_VOP_LSB) | (VOP_MEMORY))
// #define VOP_MATCH_VLDLU			((VOP_VLDLU << IW_VOP_LSB) | (VOP_MEMORY))
// #define	VOP_MATCH_VLDL_VS		((VOP_VLDL << IW_VOP_LSB) | (VOP_MEMORY_VS))

#define VOP_MATCH_VSTL			((VOP_VSTL << IW_VOP_LSB) | (VOP_MEMORY))
#define VOP_MATCH_VSTL_VS		((VOP_VSTL << IW_VOP_LSB) | (VOP_MEMORY_VS))

#define VOP_MATCH_VFLD			((VOP_VFLD << IW_VOP_LSB) | (VOP_MEMORY))

#define VOP_MATCH_VFST			((VOP_VFST << IW_VOP_LSB) | (VOP_MEMORY))

/* Vector Processing Instructions */
#define VOP_MATCH_VMERGE		((VOP_VMERGE << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VMERGE_VS		((VOP_VMERGE << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VMERGE_SV		((VOP_VMERGE_SV << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VINS			((VOP_VINS << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VINS_VS		((VOP_VINS << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VEXT			((VOP_VEXT << IW_VOP_LSB) | (VOP_RTYPE))
#define VOP_MATCH_VEXT_VS		((VOP_VEXT << IW_VOP_LSB) | (VOP_RTYPE_S))
#define VOP_MATCH_VEXTU_VS		((VOP_VEXTU << IW_VOP_LSB) | (VOP_RTYPE_S))

#define VOP_MATCH_VCOMPRESS		((VOP_VCOMPRESS << IW_VOP_LSB) | (VOP_RTYPE))

#define VOP_MATCH_VEXPAND		((VOP_VEXPAND << IW_VOP_LSB) | (VOP_RTYPE))

#define VOP_MATCH_VUPSHIFT		((VOP_VUPSHIFT << IW_VOP_LSB) | (VOP_RTYPE))

// #define VOP_MATCH_VEXTHALF		((VOP_VEXTHALF << IW_VOP_LSB) | (VOP_RTYPE))

/* Flag Processing/Misc. Instructions */
#define VOP_MATCH_VFINS			((VOP_VFINS << IW_VOP_LSB) | (VOP_FLAG))
#define VOP_MATCH_VFAND			((VOP_VFAND << IW_VOP_LSB) | (VOP_FLAG))
#define VOP_MATCH_VFAND_VS			((VOP_VFAND_VS << IW_VOP_LSB) | (VOP_FLAG))
#define VOP_MATCH_VFOR			((VOP_VFOR << IW_VOP_LSB) | (VOP_FLAG))
#define VOP_MATCH_VFOR_VS			((VOP_VFOR_VS << IW_VOP_LSB) | (VOP_FLAG))
#define VOP_MATCH_VFXOR			((VOP_VFXOR << IW_VOP_LSB) | (VOP_FLAG))
#define VOP_MATCH_VFXOR_VS			((VOP_VFXOR_VS << IW_VOP_LSB) | (VOP_FLAG))
#define VOP_MATCH_VFNOR			((VOP_VFNOR << IW_VOP_LSB) | (VOP_FLAG))
#define VOP_MATCH_VFNOR_VS			((VOP_VFNOR_VS << IW_VOP_LSB) | (VOP_FLAG))
#define VOP_MATCH_VFCLR			((VOP_VFCLR << IW_VOP_LSB) | (VOP_FLAG))
#define VOP_MATCH_VFSET			((VOP_VFSET << IW_VOP_LSB) | (VOP_FLAG))
//#define VOP_MATCH_VFPOP		((VOP_VFPOP << IW_VOP_LSB) | (VOP_FLAG))
//#define VOP_MATCH_VFFF1		((VOP_VFFF1 << IW_VOP_LSB) | (VOP_FLAG))
//#define VOP_MATCH_VFFL1		((VOP_VFFL1 << IW_VOP_LSB) | (VOP_FLAG))
#define VOP_MATCH_VMSTC			((VOP_VMSTC << IW_VOP_LSB) | (VOP_FLAG))
#define VOP_MATCH_VMCTS			((VOP_VMCTS << IW_VOP_LSB) | (VOP_FLAG))
#define VOP_MATCH_VSYNC			((VOP_VSYNC << IW_VOP_LSB) | (VOP_FLAG))


/* These are the data structures we use to hold the instruction information */

extern const struct nios2_opcode nios2_builtin_opcodes[];
extern const int bfd_nios2_num_builtin_opcodes;
extern struct nios2_opcode *nios2_opcodes;
extern int bfd_nios2_num_opcodes;

/* These are the data structures used to hold the operand parsing information */
//extern const struct nios2_arg_parser nios2_arg_parsers[];
//extern struct nios2_arg_parser* nios2_arg_parsers;
//extern const int nios2_num_builtin_arg_parsers;
//extern int nios2_num_arg_parsers;

/* These are the data structures used to hold the register information */
extern const struct nios2_reg nios2_builtin_regs[];
extern struct nios2_reg *nios2_regs;
extern const int nios2_num_builtin_regs;
extern int nios2_num_regs;

/* Machine-independent macro for number of opcodes */

#define NUMOPCODES bfd_nios2_num_opcodes
#define NUMREGISTERS nios2_num_regs;

/* these are used in disassembly to get the correct register names */
#define NUMREGNAMES 32
#define NUMCTLREGNAMES 32
#define CTLREGBASE     42
#define COPROCREGBASE  83
#define NUMCOPROCREGNAMES 32


/* this is made extern so that the assembler can use it to find out
   what instruction caused an error */
extern const struct nios2_opcode *nios2_find_opcode_hash (unsigned long);

/* overflow message strings used in the assembler */
extern char *overflow_msgs[];

#endif // _NIOS2_H
