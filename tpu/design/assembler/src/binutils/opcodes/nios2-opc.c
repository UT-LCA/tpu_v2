/* nios2-opc.c -- Altera New Jersey opcode list.

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

#include <stdio.h>
#include "opcode/nios2.h"

/* Register string table */

const struct nios2_reg nios2_builtin_regs[] = {
  {"zero", 0, SCALAR_REG},
  {"at", 1, SCALAR_REG},			// assembler temporary
  {"r2", 2, SCALAR_REG},
  {"r3", 3, SCALAR_REG},
  {"r4", 4, SCALAR_REG},
  {"r5", 5, SCALAR_REG},
  {"r6", 6, SCALAR_REG},
  {"r7", 7, SCALAR_REG},
  {"r8", 8, SCALAR_REG},
  {"r9", 9, SCALAR_REG},
  {"r10", 10, SCALAR_REG},
  {"r11", 11, SCALAR_REG},
  {"r12", 12, SCALAR_REG},
  {"r13", 13, SCALAR_REG},
  {"r14", 14, SCALAR_REG},
  {"r15", 15, SCALAR_REG},
  {"r16", 16, SCALAR_REG},
  {"r17", 17, SCALAR_REG},
  {"r18", 18, SCALAR_REG},
  {"r19", 19, SCALAR_REG},
  {"r20", 20, SCALAR_REG},
  {"r21", 21, SCALAR_REG},
  {"r22", 22, SCALAR_REG},
  {"r23", 23, SCALAR_REG},
  {"et", 24, SCALAR_REG},
  {"bt", 25, SCALAR_REG},
  {"gp", 26, SCALAR_REG},			/* global pointer */
  {"sp", 27, SCALAR_REG},			/* stack pointer */
  {"fp", 28, SCALAR_REG},			/* frame pointer */
  {"ea", 29, SCALAR_REG},			/* exception return address */
  {"ba", 30, SCALAR_REG},			/* breakpoint return address */
  {"ra", 31, SCALAR_REG},			/* return address */

  /* alternative names for special registers */
  {"r0", 0, SCALAR_REG},
  {"r1", 1, SCALAR_REG},
  {"r24", 24, SCALAR_REG},
  {"r25", 25, SCALAR_REG},
  {"r26", 26, SCALAR_REG},
  {"r27", 27, SCALAR_REG},
  {"r28", 28, SCALAR_REG},
  {"r29", 29, SCALAR_REG},
  {"r30", 30, SCALAR_REG},
  {"r31", 31, SCALAR_REG},

  /* control register names */
  {"status", 0, SCALAR_REG},
  {"estatus", 1, SCALAR_REG},
  {"bstatus", 2, SCALAR_REG},
  {"ienable", 3, SCALAR_REG},
  {"ipending", 4, SCALAR_REG},
  {"cpuid", 5, SCALAR_REG},
  {"ctl6", 6, SCALAR_REG},
  {"ctl7", 7, SCALAR_REG},
  {"pteaddr", 8, SCALAR_REG},
  {"tlbacc", 9, SCALAR_REG},
  {"tlbmisc", 10, SCALAR_REG},
  {"ctl11", 11, SCALAR_REG},
  {"ctl12", 12, SCALAR_REG},
  {"ctl13", 13, SCALAR_REG},
  {"ctl14", 14, SCALAR_REG},
  {"ctl15", 15, SCALAR_REG},
  {"ctl16", 16, SCALAR_REG},
  {"ctl17", 17, SCALAR_REG},
  {"ctl18", 18, SCALAR_REG},
  {"ctl19", 19, SCALAR_REG},
  {"ctl20", 20, SCALAR_REG},
  {"ctl21", 21, SCALAR_REG},
  {"ctl22", 22, SCALAR_REG},
  {"ctl23", 23, SCALAR_REG},
  {"ctl24", 24, SCALAR_REG},
  {"ctl25", 25, SCALAR_REG},
  {"ctl26", 26, SCALAR_REG},
  {"ctl27", 27, SCALAR_REG},
  {"ctl28", 28, SCALAR_REG},
  {"ctl29", 29, SCALAR_REG},
  {"ctl30", 30, SCALAR_REG},
  {"ctl31", 31, SCALAR_REG},
  {"ctl0", 0, SCALAR_REG},
  {"ctl1", 1, SCALAR_REG},
  {"ctl2", 2, SCALAR_REG},
  {"ctl3", 3, SCALAR_REG},
  {"ctl4", 4, SCALAR_REG},
  {"ctl5", 5, SCALAR_REG},
  {"ctl8", 8, SCALAR_REG},
  {"ctl9", 9, SCALAR_REG},
  {"ctl10", 10, SCALAR_REG},

  /* coprocessor register names */
  {"c0", 0, SCALAR_REG},
  {"c1", 1, SCALAR_REG},
  {"c2", 2, SCALAR_REG},
  {"c3", 3, SCALAR_REG},
  {"c4", 4, SCALAR_REG},
  {"c5", 5, SCALAR_REG},
  {"c6", 6, SCALAR_REG},
  {"c7", 7, SCALAR_REG},
  {"c8", 8, SCALAR_REG},
  {"c9", 9, SCALAR_REG},
  {"c10", 10, SCALAR_REG},
  {"c11", 11, SCALAR_REG},
  {"c12", 12, SCALAR_REG},
  {"c13", 13, SCALAR_REG},
  {"c14", 14, SCALAR_REG},
  {"c15", 15, SCALAR_REG},
  {"c16", 16, SCALAR_REG},
  {"c17", 17, SCALAR_REG},
  {"c18", 18, SCALAR_REG},
  {"c19", 19, SCALAR_REG},
  {"c20", 20, SCALAR_REG},
  {"c21", 21, SCALAR_REG},
  {"c22", 22, SCALAR_REG},
  {"c23", 23, SCALAR_REG},
  {"c24", 24, SCALAR_REG},
  {"c25", 25, SCALAR_REG},
  {"c26", 26, SCALAR_REG},
  {"c27", 27, SCALAR_REG},
  {"c28", 28, SCALAR_REG},
  {"c29", 29, SCALAR_REG},
  {"c30", 30, SCALAR_REG},
  {"c31", 31, SCALAR_REG},

  /* Vector Processor Register Names */

  /* Normal Vector Registers */
  {"v0", 0, VECTOR_REG},
  {"v1", 1, VECTOR_REG},
  {"v2", 2, VECTOR_REG},
  {"v3", 3, VECTOR_REG},
  {"v4", 4, VECTOR_REG},
  {"v5", 5, VECTOR_REG},
  {"v6", 6, VECTOR_REG},
  {"v7", 7, VECTOR_REG},
  {"v8", 8, VECTOR_REG},
  {"v9", 9, VECTOR_REG},
  {"v10", 10, VECTOR_REG},
  {"v11", 11, VECTOR_REG},
  {"v12", 12, VECTOR_REG},
  {"v13", 13, VECTOR_REG},
  {"v14", 14, VECTOR_REG},
  {"v15", 15, VECTOR_REG},
  {"v16", 16, VECTOR_REG},
  {"v17", 17, VECTOR_REG},
  {"v18", 18, VECTOR_REG},
  {"v19", 19, VECTOR_REG},
  {"v20", 20, VECTOR_REG},
  {"v21", 21, VECTOR_REG},
  {"v22", 22, VECTOR_REG},
  {"v23", 23, VECTOR_REG},
  {"v24", 24, VECTOR_REG},
  {"v25", 25, VECTOR_REG},
  {"v26", 26, VECTOR_REG},
  {"v27", 27, VECTOR_REG},
  {"v28", 28, VECTOR_REG},
  {"v29", 29, VECTOR_REG},
  {"v30", 30, VECTOR_REG},
  {"v31", 31, VECTOR_REG},
  {"v32", 32, VECTOR_REG},
  {"v33", 33, VECTOR_REG},
  {"v34", 34, VECTOR_REG},
  {"v35", 35, VECTOR_REG},
  {"v36", 36, VECTOR_REG},
  {"v37", 37, VECTOR_REG},
  {"v38", 38, VECTOR_REG},
  {"v39", 39, VECTOR_REG},
  {"v40", 40, VECTOR_REG},
  {"v41", 41, VECTOR_REG},
  {"v42", 42, VECTOR_REG},
  {"v43", 43, VECTOR_REG},
  {"v44", 44, VECTOR_REG},
  {"v45", 45, VECTOR_REG},
  {"v46", 46, VECTOR_REG},
  {"v47", 47, VECTOR_REG},
  {"v48", 48, VECTOR_REG},
  {"v49", 49, VECTOR_REG},
  {"v50", 50, VECTOR_REG},
  {"v51", 51, VECTOR_REG},
  {"v52", 52, VECTOR_REG},
  {"v53", 53, VECTOR_REG},
  {"v54", 54, VECTOR_REG},
  {"v55", 55, VECTOR_REG},
  {"v56", 56, VECTOR_REG},
  {"v57", 57, VECTOR_REG},
  {"v58", 58, VECTOR_REG},
  {"v59", 59, VECTOR_REG},
  {"v60", 60, VECTOR_REG},
  {"v61", 61, VECTOR_REG},
  {"v62", 62, VECTOR_REG},
  {"v63", 63, VECTOR_REG},

  /* Vector Flag Registers */
  {"vfmask0", 0, VECTOR_REG},				/* Default flag for all masked operations */
  {"vfmask1", 1, VECTOR_REG},				/* 2nd flag for masked ops: e.x. vadd.vv.1 */
  {"vfgr0", 2, VECTOR_REG},					
  {"vfgr1", 3, VECTOR_REG},
  {"vfgr2", 4, VECTOR_REG},
  {"vfgr3", 5, VECTOR_REG},
  {"vfgr4", 6, VECTOR_REG},
  {"vfgr5", 7, VECTOR_REG},
  {"vfgr6", 8, VECTOR_REG},
  {"vfgr7", 9, VECTOR_REG},
  {"vfgr8", 10, VECTOR_REG},
  {"vfgr9", 11, VECTOR_REG},
  {"vfgr10", 12, VECTOR_REG},
  {"vfgr11", 13, VECTOR_REG},
  {"vfgr12", 14, VECTOR_REG},
  {"vfgr13", 15, VECTOR_REG},
  {"vfgr14", 16, VECTOR_REG},
  {"vfgr15", 17, VECTOR_REG},
  {"vfgr16", 18, VECTOR_REG},
  {"vfgr17", 19, VECTOR_REG},
  {"vfgr18", 20, VECTOR_REG},
  {"vfgr19", 21, VECTOR_REG},
  {"vfgr20", 22, VECTOR_REG},
  {"vfgr21", 23, VECTOR_REG},
  {"vfgr22", 24, VECTOR_REG},
  {"vfgr23", 25, VECTOR_REG} ,
  {"vfgr24", 26, VECTOR_REG},
  {"vfgr25", 27, VECTOR_REG},
  {"vfgr26", 28, VECTOR_REG},
  {"vfgr27", 29, VECTOR_REG},
  {"vfzero", 30, VECTOR_REG},
  {"vfone", 31, VECTOR_REG},

  /* Vector Control Registers: This table is inconsistant w/ the ISA-Spec version (where vbase0 = 48),
	because this value is needed for the memory instructions.  The actual value (used by the vmstc/vmcts instructions
	will be calcualted by the respective assembling function in tc-nios2.c */
  {"VL", 0, VECTOR_REG},
  {"VPW", 1, VECTOR_REG},
  {"vindex", 2, VECTOR_REG},
  {"vshamt", 3, VECTOR_REG},
  {"ACCncopy", 28, VECTOR_REG},
  {"NLane", 29, VECTOR_REG},
  {"MVL", 30, VECTOR_REG},
  {"logMVL", 31, VECTOR_REG},
  {"vbase0", 0, VECTOR_REG},
  {"vbase1", 1, VECTOR_REG},
  {"vbase2", 2, VECTOR_REG},
  {"vbase3", 3, VECTOR_REG},
  {"vbase4", 4, VECTOR_REG},
  {"vbase5", 5, VECTOR_REG},
  {"vbase6", 6, VECTOR_REG},
  {"vbase7", 7, VECTOR_REG},
  {"vbase8", 8, VECTOR_REG},
  {"vbase9", 9, VECTOR_REG},
  {"vbase10", 10, VECTOR_REG},
  {"vbase11", 11, VECTOR_REG},
  {"vbase12", 12, VECTOR_REG},
  {"vbase13", 13, VECTOR_REG},
  {"vbase14", 14, VECTOR_REG},
  {"vbase15", 15, VECTOR_REG},
  {"vstride0", 0, VECTOR_REG},
  {"vstride1", 1, VECTOR_REG},
  {"vstride2", 2, VECTOR_REG},
  {"vstride3", 3, VECTOR_REG},
  {"vstride4", 4, VECTOR_REG},
  {"vstride5", 5, VECTOR_REG},
  {"vstride6", 6, VECTOR_REG},
  {"vstride7", 7, VECTOR_REG},
  {"vinc0", 0, VECTOR_REG},
  {"vinc1", 1, VECTOR_REG},
  {"vinc2", 2, VECTOR_REG},
  {"vinc3", 3, VECTOR_REG},
  {"vinc4", 4, VECTOR_REG},
  {"vinc5", 5, VECTOR_REG},
  {"vinc6", 6, VECTOR_REG},
  {"vinc7", 7, VECTOR_REG}
};

/* Register index offset value, add this offset to the index above for vmstc/vmcts vector instructions */
/*#define	VSTRIDE_OFFSET	32
#define	VINC_OFFSET		40
#define	VBASE_OFFSET	48*/
/* These values are defined in nios2.h */


#define NIOS2_NUM_REGS \
       ((sizeof nios2_builtin_regs) / (sizeof (nios2_builtin_regs[0])))
const int nios2_num_builtin_regs = NIOS2_NUM_REGS;

/* const removed from the following to allow for dynamic extensions to the
 * built-in instruction set. */
struct nios2_reg *nios2_regs = (struct nios2_reg *) nios2_builtin_regs;
int nios2_num_regs = NIOS2_NUM_REGS;
#undef NIOS2_NUM_REGS

/* overflow message string templates */

char *overflow_msgs[] = {
  "call target address 0x%08x out of range 0x%08x to 0x%08x",
  "branch offset %d out of range %d to %d",
  "%s offset %d out of range %d to %d",
  "immediate value %d out of range %d to %d",
  "immediate value %u out of range %u to %u",
  "immediate value %u out of range %u to %u",
  "custom instruction opcode %u out of range %u to %u",
};



/*--------------------------------------------------------------------------------
   This is the opcode table used by the New Jersey GNU as, disassembler and GDB
  --------------------------------------------------------------------------------*/

/*
       The following letters can appear in the args field of the nios2_opcode
       structure:

       c - a 5-bit control register index or break opcode
       d - a 5-bit destination register index
       s - a 5-bit left source register index
       t - a 5-bit right source register index
       i - a 16-bit signed immediate
       u - a 16-bit unsigned immediate

    j - a 5-bit unsigned immediate
       k - a 6-bit unsigned immediate
       l - an 8-bit unsigned immediate
	   m - a 26-bit unsigned immediate


	   The vector processor requires some new args:

	   vcd - a 6-bit destination control register index
	   vd - a 6-bit destination register index
	   va - a 6-bit left source register index
	   vb - a 6-bit right source register index
	   vfd - a 5-bit (zero extended to 6-bit) destination flag register index		<<--- MSB <= 0
	   vfa - a 5-bit (zero extended to 6-bit) left source flag register index
	   vfb - a 5-bit (zero extended to 6-bit) right source flag register index
	   vbas - a 5-bit base memory address register index
	   vinc - a 3-bit increment register index
	   vstr - a 4-bit memory address stride register index
	   voff - a 6-bit offset register index
	   rs - a 5-bit (scalar) source register index (max 1 scalar argument per vector instruction)
	   rsd - a 5-bit (scalar) destination register index
	   n - a 6-bit vector control regisetr
	   

*/

/* *INDENT-OFF* */
/* FIXME: Re-format for GNU standards */
const struct nios2_opcode nios2_builtin_opcodes[] =
{
   /* name,             args,           args_test     num_args,          match,                  mask,                                   pinfo */
       {"add",          "d,s,t",        "d,s,t,E",     3,         OP_MATCH_ADD,           OP_MASK_ROPX | OP_MASK_ROP,      	      0,									no_overflow },
       {"addi",         "t,s,i",        "t,s,i,E",     3,         OP_MATCH_ADDI,          OP_MASK_IOP,                            NIOS2_INSN_ADDI,						signed_immed16_overflow },
       {"subi",         "t,s,i",        "t,s,i,E",     3,         OP_MATCH_ADDI,          OP_MASK_IOP,                            NIOS2_INSN_MACRO,						signed_immed16_overflow },
       {"and",          "d,s,t",        "d,s,t,E",     3,         OP_MATCH_AND,           OP_MASK_ROPX | OP_MASK_ROP,			  0,									no_overflow },
       {"andhi",        "t,s,u",        "t,s,u,E",     3,         OP_MATCH_ANDHI,         OP_MASK_IOP,                            0,									unsigned_immed16_overflow },
       {"andi",         "t,s,u",        "t,s,u,E",     3,         OP_MATCH_ANDI,          OP_MASK_IOP,                            NIOS2_INSN_ANDI,						unsigned_immed16_overflow },
       {"beq",          "s,t,o",        "s,t,o,E",     3,         OP_MATCH_BEQ,           OP_MASK_IOP,                            NIOS2_INSN_CBRANCH,  					branch_target_overflow },
       {"bge",          "s,t,o",        "s,t,o,E",     3,         OP_MATCH_BGE,           OP_MASK_IOP,                            NIOS2_INSN_CBRANCH,					branch_target_overflow },
       {"bgeu",         "s,t,o",        "s,t,o,E",     3,         OP_MATCH_BGEU,          OP_MASK_IOP,                            NIOS2_INSN_CBRANCH,					branch_target_overflow },
       {"bgt",          "s,t,o",        "s,t,o,E",     3,         OP_MATCH_BLT,           OP_MASK_IOP,                            NIOS2_INSN_MACRO|NIOS2_INSN_CBRANCH,		branch_target_overflow },
       {"bgtu",         "s,t,o",        "s,t,o,E",     3,         OP_MATCH_BLTU,          OP_MASK_IOP,                            NIOS2_INSN_MACRO|NIOS2_INSN_CBRANCH,		branch_target_overflow },
       {"ble",          "s,t,o",        "s,t,o,E",     3,         OP_MATCH_BGE,           OP_MASK_IOP,                            NIOS2_INSN_MACRO|NIOS2_INSN_CBRANCH,		branch_target_overflow },
       {"bleu",         "s,t,o",        "s,t,o,E",     3,         OP_MATCH_BGEU,          OP_MASK_IOP,                            NIOS2_INSN_MACRO|NIOS2_INSN_CBRANCH,		branch_target_overflow },
       {"blt",          "s,t,o",        "s,t,o,E",     3,         OP_MATCH_BLT,           OP_MASK_IOP,                            NIOS2_INSN_CBRANCH,					branch_target_overflow },
       {"bltu",         "s,t,o",        "s,t,o,E",     3,         OP_MATCH_BLTU,          OP_MASK_IOP,                            NIOS2_INSN_CBRANCH,					branch_target_overflow },
       {"bne",          "s,t,o",        "s,t,o,E",     3,         OP_MATCH_BNE,           OP_MASK_IOP,                            NIOS2_INSN_CBRANCH,					branch_target_overflow },
       {"br",           "o",            "o,E",         1,         OP_MATCH_BR,            OP_MASK_IOP,                            NIOS2_INSN_UBRANCH,					branch_target_overflow },
       {"break",        "b",            "b,E",         1,         OP_MATCH_BREAK,         OP_MASK_BREAK,                          0,				 					no_overflow },
       {"bret",         "",             "E",           0,         OP_MATCH_BRET,          OP_MASK,                                0, 									no_overflow },
       {"flushd",       "i(s)",       	"i(s)E",       2,         OP_MATCH_FLUSHD,        OP_MASK_IOP,                            0,									signed_immed16_overflow },
       {"flushda",      "i(s)",       	"i(s)E",       2,         OP_MATCH_FLUSHDA,       OP_MASK_IOP,                            0,									signed_immed16_overflow },
       {"flushi",       "s",       		"s,E",	       1,         OP_MATCH_FLUSHI,        OP_MASK_FLUSHI,			              0,				 					no_overflow },
       {"flushp",       "",       		"E",	       0,         OP_MATCH_FLUSHP,        OP_MASK,	                              0,				 					no_overflow },
 	   {"initd",       "i(s)",       	"i(s)E",       2,         OP_MATCH_INITD,         OP_MASK_IOP,                            0,									signed_immed16_overflow },
       {"initi",       "s",       		"s,E",	       1,         OP_MATCH_INITI,         OP_MASK_INITI,			              0,				 					no_overflow },
       {"call",         "m",            "m,E",         1,         OP_MATCH_CALL,          OP_MASK_IOP,                            NIOS2_INSN_CALL,						call_target_overflow },
       {"callr",        "s",            "s,E",         1,         OP_MATCH_CALLR,         OP_MASK_CALLR,                          0, 									no_overflow },
       {"cmpeq",        "d,s,t",        "d,s,t,E",     3,         OP_MATCH_CMPEQ,         OP_MASK_ROPX | OP_MASK_ROP, 		      0,									no_overflow },
       {"cmpeqi",       "t,s,i",        "t,s,i,E",     3,         OP_MATCH_CMPEQI,        OP_MASK_IOP,                            0,									signed_immed16_overflow },
       {"cmpge",        "d,s,t",        "d,s,t,E",     3,         OP_MATCH_CMPGE,         OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"cmpgei",       "t,s,i",        "t,s,i,E",     3,         OP_MATCH_CMPGEI,        OP_MASK_IOP,                            0,									signed_immed16_overflow },
       {"cmpgeu",       "d,s,t",        "d,s,t,E",     3,         OP_MATCH_CMPGEU,        OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"cmpgeui",      "t,s,u",        "t,s,u,E",     3,         OP_MATCH_CMPGEUI,       OP_MASK_IOP,                            0,									unsigned_immed16_overflow },
       {"cmpgt",        "d,s,t",        "d,s,t,E",     3,         OP_MATCH_CMPLT,         OP_MASK_ROPX | OP_MASK_ROP,             NIOS2_INSN_MACRO, 					no_overflow },
       {"cmpgti",       "t,s,i",        "t,s,i,E",     3,         OP_MATCH_CMPGEI,        OP_MASK_IOP,                            NIOS2_INSN_MACRO,						signed_immed16_overflow },
       {"cmpgtu",       "d,s,t",        "d,s,t,E",     3,         OP_MATCH_CMPLTU,        OP_MASK_ROPX | OP_MASK_ROP,             NIOS2_INSN_MACRO, 					no_overflow },
       {"cmpgtui",      "t,s,u",        "t,s,u,E",     3,         OP_MATCH_CMPGEUI,       OP_MASK_IOP,                            NIOS2_INSN_MACRO,						unsigned_immed16_overflow },
       {"cmple",        "d,s,t",        "d,s,t,E",     3,         OP_MATCH_CMPGE,         OP_MASK_ROPX | OP_MASK_ROP,             NIOS2_INSN_MACRO, 					no_overflow },
       {"cmplei",       "t,s,i",        "t,s,i,E",     3,         OP_MATCH_CMPLTI,        OP_MASK_IOP,                            NIOS2_INSN_MACRO,						signed_immed16_overflow },
       {"cmpleu",       "d,s,t",        "d,s,t,E",     3,         OP_MATCH_CMPGEU,        OP_MASK_ROPX | OP_MASK_ROP,             NIOS2_INSN_MACRO, 					no_overflow },
       {"cmpleui",      "t,s,u",        "t,s,u,E",     3,         OP_MATCH_CMPLTUI,       OP_MASK_IOP,                            NIOS2_INSN_MACRO,						unsigned_immed16_overflow },
       {"cmplt",        "d,s,t",        "d,s,t,E",     3,         OP_MATCH_CMPLT,         OP_MASK_ROPX | OP_MASK_ROP,             0,									no_overflow },
       {"cmplti",       "t,s,i",        "t,s,i,E",     3,         OP_MATCH_CMPLTI,        OP_MASK_IOP,                            0,									signed_immed16_overflow },
       {"cmpltu",       "d,s,t",        "d,s,t,E",     3,         OP_MATCH_CMPLTU,        OP_MASK_ROPX | OP_MASK_ROP,             0,									no_overflow },
       {"cmpltui",      "t,s,u",        "t,s,u,E",     3,         OP_MATCH_CMPLTUI,       OP_MASK_IOP,                            0,									unsigned_immed16_overflow },
       {"cmpne",        "d,s,t",        "d,s,t,E",     3,         OP_MATCH_CMPNE,         OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"cmpnei",       "t,s,i",        "t,s,i,E",     3,         OP_MATCH_CMPNEI,        OP_MASK_IOP,                            0,									signed_immed16_overflow },
       {"div",          "d,s,t",        "d,s,t,E",     3,         OP_MATCH_DIV,           OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"divu",         "d,s,t",        "d,s,t,E",     3,         OP_MATCH_DIVU,          OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"jmp",          "s",            "s,E",         1,         OP_MATCH_JMP,           OP_MASK_JMP,                            0, 									no_overflow },
       {"ldb",          "t,i(s)",       "t,i(s)E",     3,         OP_MATCH_LDB,           OP_MASK_IOP,                            0,									address_offset_overflow },
       {"ldbio",        "t,i(s)",       "t,i(s)E",     3,         OP_MATCH_LDBIO,         OP_MASK_IOP,                            0,									address_offset_overflow },
       {"ldbu",         "t,i(s)",       "t,i(s)E",     3,         OP_MATCH_LDBU,          OP_MASK_IOP,                            0,									address_offset_overflow },
       {"ldbuio",       "t,i(s)",       "t,i(s)E",     3,         OP_MATCH_LDBUIO,        OP_MASK_IOP,                            0,									address_offset_overflow },
       {"ldh",          "t,i(s)",       "t,i(s)E",     3,         OP_MATCH_LDH,           OP_MASK_IOP,                            0,									address_offset_overflow },
       {"ldhio",        "t,i(s)",       "t,i(s)E",     3,         OP_MATCH_LDHIO,         OP_MASK_IOP,                            0,									address_offset_overflow },
       {"ldhu",         "t,i(s)",       "t,i(s)E",     3,         OP_MATCH_LDHU,          OP_MASK_IOP,                            0,									address_offset_overflow },
       {"ldhuio",       "t,i(s)",       "t,i(s)E",     3,         OP_MATCH_LDHUIO,        OP_MASK_IOP,                            0,									address_offset_overflow },
       {"ldw",          "t,i(s)",       "t,i(s)E",     3,         OP_MATCH_LDW,           OP_MASK_IOP,                            0,									address_offset_overflow },
       {"ldwio",        "t,i(s)",       "t,i(s)E",     3,         OP_MATCH_LDWIO,         OP_MASK_IOP,                            0,									address_offset_overflow },
       {"mov",          "d,s",          "d,s,E",       2,         OP_MATCH_ADD,           OP_MASK_RRT|OP_MASK_ROPX|OP_MASK_ROP,   NIOS2_INSN_MACRO_MOV, 				no_overflow },
       {"movhi",        "t,u",          "t,u,E",       2,         OP_MATCH_ORHI,          OP_MASK_IRS|OP_MASK_IOP,                NIOS2_INSN_MACRO_MOVI,				unsigned_immed16_overflow },
       {"movui",        "t,u",          "t,u,E",       2,         OP_MATCH_ORI,           OP_MASK_IRS|OP_MASK_IOP,                NIOS2_INSN_MACRO_MOVI,				unsigned_immed16_overflow },
       {"movi",         "t,i",          "t,i,E",       2,         OP_MATCH_ADDI,          OP_MASK_IRS|OP_MASK_IOP,                NIOS2_INSN_MACRO_MOVI,				signed_immed16_overflow },
       /* movia expands to two instructions so there is no mask or match */
       {"movia",		"t,o",			"t,o,E",		2,		OP_MATCH_ORHI,          OP_MASK_IOP,							  NIOS2_INSN_MACRO_MOVIA,				no_overflow },
       {"mul",          "d,s,t",        "d,s,t,E",      3,        OP_MATCH_MUL,           OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"muli",         "t,s,i",        "t,s,i,E",      3,        OP_MATCH_MULI,          OP_MASK_IOP,                            0,									signed_immed16_overflow },
       {"mulxss",       "d,s,t",        "d,s,t,E",      3,        OP_MATCH_MULXSS,        OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"mulxsu",       "d,s,t",        "d,s,t,E",      3,        OP_MATCH_MULXSU,        OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"mulxuu",       "d,s,t",        "d,s,t,E",      3,        OP_MATCH_MULXUU,        OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"nextpc",       "d",            "d,E",          1,        OP_MATCH_NEXTPC,        OP_MASK_NEXTPC,                         0, 									no_overflow },
       {"nop",          "",             "E",            0,        OP_MATCH_ADD,           OP_MASK,                                NIOS2_INSN_MACRO_MOV,					no_overflow },
       {"nor",          "d,s,t",        "d,s,t,E",      3,        OP_MATCH_NOR,           OP_MASK_ROPX | OP_MASK_ROP,             0,									no_overflow },
       {"or",           "d,s,t",        "d,s,t,E",      3,        OP_MATCH_OR,            OP_MASK_ROPX | OP_MASK_ROP,             0,									no_overflow },
       {"orhi",         "t,s,u",        "t,s,u,E",      3,        OP_MATCH_ORHI,          OP_MASK_IOP,                            0,									unsigned_immed16_overflow },
       {"ori",          "t,s,u",        "t,s,u,E",      3,        OP_MATCH_ORI,           OP_MASK_IOP,                            NIOS2_INSN_ORI,						unsigned_immed16_overflow },
       {"rdctl",        "d,c",          "d,c,E",        2,        OP_MATCH_RDCTL,         OP_MASK_RDCTL,                          0, 									no_overflow },
       {"ret",          "",             "E",            0,        OP_MATCH_RET,           OP_MASK,                                0, 									no_overflow },
       {"rol",          "d,s,t",        "d,s,t,E",      3,      OP_MATCH_ROL,           OP_MASK_ROPX | OP_MASK_ROP,               0, 									no_overflow },
       {"roli",         "d,s,j",        "d,s,j,E",      3,        OP_MATCH_ROLI,          OP_MASK_ROLI,                           0, 									unsigned_immed5_overflow },
       {"ror",          "d,s,t",        "d,s,t,E",      3,        OP_MATCH_ROR,           OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"sll",          "d,s,t",        "d,s,t,E",      3,        OP_MATCH_SLL,           OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"slli",         "d,s,j",        "d,s,j,E",      3,        OP_MATCH_SLLI,          OP_MASK_SLLI,                           0, 									unsigned_immed5_overflow },
       {"sra",          "d,s,t",        "d,s,t,E",      3,        OP_MATCH_SRA,           OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"srai",         "d,s,j",        "d,s,j,E",      3,        OP_MATCH_SRAI,          OP_MASK_SRAI,                           0, 									unsigned_immed5_overflow },
       {"srl",          "d,s,t",        "d,s,t,E",      3,        OP_MATCH_SRL,           OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"srli",         "d,s,j",        "d,s,j,E",      3,        OP_MATCH_SRLI,          OP_MASK_SRLI,                           0, 									unsigned_immed5_overflow },
       {"stb",          "t,i(s)",       "t,i(s)E",      3,        OP_MATCH_STB,           OP_MASK_IOP,                            0,									address_offset_overflow },
       {"stbio",        "t,i(s)",       "t,i(s)E",      3,        OP_MATCH_STBIO,         OP_MASK_IOP,                            0,									address_offset_overflow },
       {"sth",          "t,i(s)",       "t,i(s)E",      3,        OP_MATCH_STH,           OP_MASK_IOP,                            0,									address_offset_overflow },
       {"sthio",        "t,i(s)",       "t,i(s)E",      3,        OP_MATCH_STHIO,         OP_MASK_IOP,                            0,									address_offset_overflow },
       {"stw",          "t,i(s)",       "t,i(s)E",      3,        OP_MATCH_STW,           OP_MASK_IOP,                            0,									address_offset_overflow },
       {"stwio",        "t,i(s)",       "t,i(s)E",      3,        OP_MATCH_STWIO,         OP_MASK_IOP,                            0,									address_offset_overflow },
       {"sub",          "d,s,t",        "d,s,t,E",      3,        OP_MATCH_SUB,           OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"sync",         "",             "E",            0,        OP_MATCH_SYNC,          OP_MASK_SYNC,                           0, 									no_overflow },
       {"trap",         "",             "E",            0,      	OP_MATCH_TRAP,          OP_MASK_TRAP,                         0, 									no_overflow },
       {"eret",         "",             "E",            0,      OP_MATCH_ERET,          OP_MASK,                                  0,									no_overflow },
       {"custom",       "l,d,s,t",      "l,d,s,t,E",    4,        OP_MATCH_CUSTOM,           OP_MASK_ROP,                      0,	 								custom_opcode_overflow },
       {"wrctl",        "c,s",          "c,s,E",        2,        OP_MATCH_WRCTL,         OP_MASK_WRCTL,                          0, 									no_overflow },
       {"xor",          "d,s,t",        "d,s,t,E",      3,        OP_MATCH_XOR,           OP_MASK_ROPX | OP_MASK_ROP,             0, 									no_overflow },
       {"xorhi",        "t,s,u",        "t,s,u,E",      3,        OP_MATCH_XORHI,         OP_MASK_IOP,                            0,									unsigned_immed16_overflow },
       {"xori",         "t,s,u",        "t,s,u,E",      3,        OP_MATCH_XORI,          OP_MASK_IOP,                            NIOS2_INSN_XORI,						unsigned_immed16_overflow },

   /*  name,            args,           args_test     num_args,          match,                  mask,                                   pinfo */

	   /* Vector Co-processor op-codes start here.... */		
	   /* Integer Instructions */		
	   {"vabs",			"d,a",			"d,a,E",		2,		VOP_MATCH_VABS,			VOP_MASK_,								0,										no_overflow },
	   {"vabs.1",		"d,a",			"d,a,E",		2,		VOP_MATCH_VABS,			VOP_MASK_,								0,										no_overflow },

	   {"vabsdiff",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VABSDIFF,		VOP_MASK_,								0,										no_overflow	},
	   {"vabsdiff.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VABSDIFF,		VOP_MASK_,								0,										no_overflow	},
	   {"vabsdiff.vv",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VABSDIFF,		VOP_MASK_,								0,										no_overflow	},
	   {"vabsdiff.vv.1","d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VABSDIFF,		VOP_MASK_,								0,										no_overflow	},
	   {"vabsdiff.vs",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VABSDIFF_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vabsdiff.vs.1","d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VABSDIFF_VS,	VOP_MASK_,								0,										no_overflow	},

	   {"vadd",			"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VADD,			VOP_MASK_,								0,										no_overflow	},
	   {"vadd.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VADD,			VOP_MASK_,								0,										no_overflow	},
	   {"vadd.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VADD,			VOP_MASK_,								0,										no_overflow	},
	   {"vadd.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VADD,			VOP_MASK_,								0,										no_overflow	},
	   {"vadd.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VADD_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vadd.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VADD_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vaddu",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VADDU,		VOP_MASK_,								0,										no_overflow	},
	   {"vaddu.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VADDU,		VOP_MASK_,								0,										no_overflow	},
	   {"vaddu.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VADDU,		VOP_MASK_,								0,										no_overflow	},
	   {"vaddu.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VADDU,		VOP_MASK_,								0,										no_overflow	},
	   {"vaddu.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VADDU_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vaddu.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VADDU_VS,		VOP_MASK_,								0,										no_overflow	},

	   {"vsub",			"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSUB,			VOP_MASK_,								0,										no_overflow	},
	   {"vsub.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSUB,			VOP_MASK_,								0,										no_overflow	},
	   {"vsub.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSUB,			VOP_MASK_,								0,										no_overflow	},
	   {"vsub.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSUB,			VOP_MASK_,								0,										no_overflow	},
	   {"vsub.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VSUB_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vsub.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VSUB_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vsub.sv",		"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VSUB_SV,		VOP_MASK_,								0,										no_overflow	},
	   {"vsub.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VSUB_SV,		VOP_MASK_,								0,										no_overflow	},
	   {"vsubu",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSUBU,		VOP_MASK_,								0,										no_overflow	},
	   {"vsubu.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSUBU,		VOP_MASK_,								0,										no_overflow	},
	   {"vsubu.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSUBU,		VOP_MASK_,								0,										no_overflow	},
	   {"vsubu.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSUBU,		VOP_MASK_,								0,										no_overflow	},
	   {"vsubu.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VSUBU_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vsubu.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VSUBU_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vsubu.sv",		"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VSUBU_SV,		VOP_MASK_,								0,										no_overflow	},
	   {"vsubu.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VSUBU_SV,		VOP_MASK_,								0,										no_overflow	},

	   {"vmulhi",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULHI,		VOP_MASK_,								0,										no_overflow	},
	   {"vmulhi.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULHI,		VOP_MASK_,								0,										no_overflow	},
	   {"vmulhi.vv",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULHI,		VOP_MASK_,								0,										no_overflow	},
	   {"vmulhi.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULHI,		VOP_MASK_,								0,										no_overflow	},
	   {"vmulhi.vs",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMULHI_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vmulhi.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMULHI_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vmulhiu",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULHIU,		VOP_MASK_,								0,										no_overflow	},
	   {"vmulhiu.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULHIU,		VOP_MASK_,								0,										no_overflow	},
	   {"vmulhiu.vv",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULHIU,		VOP_MASK_,								0,										no_overflow	},
	   {"vmulhiu.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULHIU,		VOP_MASK_,								0,										no_overflow	},
	   {"vmulhiu.vs",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMULHIU_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vmulhiu.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMULHIU_VS,	VOP_MASK_,								0,										no_overflow	},

	   {"vmullo",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULLO,		VOP_MASK_,								0,										no_overflow	},
	   {"vmullo.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULLO,		VOP_MASK_,								0,										no_overflow	},
	   {"vmullo.vv",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULLO,		VOP_MASK_,								0,										no_overflow	},
	   {"vmullo.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULLO,		VOP_MASK_,								0,										no_overflow	},
	   {"vmullo.vs",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMULLO_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vmullo.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMULLO_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vmullou",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULLOU,		VOP_MASK_,								0,										no_overflow	},
	   {"vmullou.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULLOU,		VOP_MASK_,								0,										no_overflow	},
	   {"vmullou.vv",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULLOU,		VOP_MASK_,								0,										no_overflow	},
	   {"vmullou.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMULLOU,		VOP_MASK_,								0,										no_overflow	},
	   {"vmullou.vs",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMULLOU_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vmullou.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMULLOU_VS,	VOP_MASK_,								0,										no_overflow	},

	   {"vdiv",			"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VDIV,			VOP_MASK_,								0,										no_overflow	},
	   {"vdiv.1"	,	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VDIV,			VOP_MASK_,								0,										no_overflow	},
	   {"vdiv.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VDIV,			VOP_MASK_,								0,										no_overflow	},
	   {"vdiv.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VDIV,			VOP_MASK_,								0,										no_overflow	},
	   {"vdiv.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VDIV_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vdiv.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VDIV_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vdiv.sv",		"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VDIV_SV,		VOP_MASK_,								0,										no_overflow	},
	   {"vdiv.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VDIV_SV,		VOP_MASK_,								0,										no_overflow	},
	   {"vdivu",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VDIVU,		VOP_MASK_,								0,										no_overflow	},
	   {"vdivu.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VDIVU,		VOP_MASK_,								0,										no_overflow	},
	   {"vdivu.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VDIVU,		VOP_MASK_,								0,										no_overflow	},
	   {"vdivu.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VDIVU,		VOP_MASK_,								0,										no_overflow	},
	   {"vdivu.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VDIVU_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vdivu.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VDIVU_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vdivu.sv",		"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VDIVU_SV,		VOP_MASK_,								0,										no_overflow	},
	   {"vdivu.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VDIVU_SV,		VOP_MASK_,								0,										no_overflow	},

		/* This instruction has been removed from the Vector-ISA */
/*	   {"vmod.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMOD,			VOP_MASK_,								0,										no_overflow	},
	   {"vmod.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMOD,			VOP_MASK_,								0,										no_overflow	},
	   {"vmod.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMOD_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vmod.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMOD_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vmod.sv",		"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VMOD_SV,		VOP_MASK_,								0,										no_overflow	},
	   {"vmod.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VMOD_SV,		VOP_MASK_,								0,										no_overflow	},
	   {"vmodu.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMODU,		VOP_MASK_,								0,										no_overflow	},
	   {"vmodu.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMODU,		VOP_MASK_,								0,										no_overflow	},
	   {"vmodu.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMODU_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vmodu.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMODU_VS,		VOP_MASK_,								0,										no_overflow	}, 
	   {"vmodu.sv",		"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VMODU_SV,		VOP_MASK_,								0,										no_overflow	},
	   {"vmodu.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VMODU_SV,		VOP_MASK_,								0,										no_overflow	},
*/

	   {"vsra",			"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSRA,			VOP_MASK_,								0,										no_overflow	},
	   {"vsra.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSRA,			VOP_MASK_,								0,										no_overflow	},
	   {"vsra.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSRA,			VOP_MASK_,								0,										no_overflow	},
	   {"vsra.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSRA,			VOP_MASK_,								0,										no_overflow	},
	   {"vsra.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VSRA_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vsra.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VSRA_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vsra.sv",		"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VSRA_SV,		VOP_MASK_,								0,										no_overflow	},
	   {"vsra.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VSRA_SV,		VOP_MASK_,								0,										no_overflow	},

	   {"vmin",			"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMIN,			VOP_MASK_,								0,										no_overflow	},
	   {"vmin.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMIN,			VOP_MASK_,								0,										no_overflow	},
	   {"vmin.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMIN,			VOP_MASK_,								0,										no_overflow	},
	   {"vmin.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMIN,			VOP_MASK_,								0,										no_overflow	},
	   {"vmin.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMIN_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vmin.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMIN_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vminu",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMINU,		VOP_MASK_,								0,										no_overflow	},
	   {"vminu.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMINU,		VOP_MASK_,								0,										no_overflow	},
	   {"vminu.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMINU,		VOP_MASK_,								0,										no_overflow	},
	   {"vminu.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMINU,		VOP_MASK_,								0,										no_overflow	},
	   {"vminu.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMINU_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vminu.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMINU_VS,		VOP_MASK_,								0,										no_overflow	},

	   {"vmax",			"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMAX,			VOP_MASK_,								0,										no_overflow	},
	   {"vmax.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMAX,			VOP_MASK_,								0,										no_overflow	},
	   {"vmax.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMAX,			VOP_MASK_,								0,										no_overflow	},
	   {"vmax.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMAX,			VOP_MASK_,								0,										no_overflow	},
	   {"vmax.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMAX_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vmax.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMAX_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vmaxu",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMAXU,		VOP_MASK_,								0,										no_overflow	},
	   {"vmaxu.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMAXU,		VOP_MASK_,								0,										no_overflow	},
	   {"vmaxu.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMAXU,		VOP_MASK_,								0,										no_overflow	},
	   {"vmaxu.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMAXU,		VOP_MASK_,								0,										no_overflow	},
	   {"vmaxu.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMAXU_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vmaxu.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMAXU_VS,		VOP_MASK_,								0,										no_overflow	},

	   {"vcmpe",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPEQ,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpe.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPEQ,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpe.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPEQ,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpe.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPEQ,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpe.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VCMPEQ_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmpe.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VCMPEQ_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmpne",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPNEQ,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpne.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPNEQ,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpne.vv",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPNEQ,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpne.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPNEQ,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpne.vs",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VCMPNEQ_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmpne.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VCMPNEQ_VS,	VOP_MASK_,								0,										no_overflow	},

	   {"vcmplt",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLT,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmplt.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLT,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmplt.vv",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLT,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmplt.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLT,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmplt.vs",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VCMPLT_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmplt.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VCMPLT_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmplt.sv",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VCMPLT_SV,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmplt.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VCMPLT_SV,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmpltu",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLTU,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpltu.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLTU,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpltu.vv",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLTU,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpltu.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLTU,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpltu.vs",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VCMPLTU_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmpltu.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VCMPLTU_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmpltu.sv",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VCMPLTU_SV,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmpltu.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VCMPLTU_SV,	VOP_MASK_,								0,										no_overflow	},

	   {"vcmple",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLE,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmple.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLE,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmple.vv",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLE,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmple.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLE,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmple.vs",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VCMPLE_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmple.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VCMPLE_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmple.sv",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VCMPLE_SV,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmple.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VCMPLE_SV,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmpleu",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLEU,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpleu.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLEU,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpleu.vv",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLEU,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpleu.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCMPLEU,		VOP_MASK_,								0,										no_overflow	},
	   {"vcmpleu.vs",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VCMPLEU_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmpleu.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VCMPLEU_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmpleu.sv",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VCMPLEU_SV,	VOP_MASK_,								0,										no_overflow	},
	   {"vcmpleu.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VCMPLEU_SV,	VOP_MASK_,								0,										no_overflow	},

	   {"vmac",		"a,b",			"a,b,E",		2,		VOP_MATCH_VMAC,		VOP_MASK_,								0,										no_overflow },
	   {"vmac.1",		"a,b",			"a,b,E",		2,		VOP_MATCH_VMAC,		VOP_MASK_,								0,										no_overflow },
	   {"vmac.vv",		"a,b",			"a,b,E",		2,		VOP_MATCH_VMAC,		VOP_MASK_,								0,										no_overflow },
	   {"vmac.vv.1",	"a,b",			"a,b,E",		2,		VOP_MATCH_VMAC,		VOP_MASK_,								0,										no_overflow },
	   {"vmac.vs",		"a,s",			"a,s,E",		2,		VOP_MATCH_VMAC_VS,		VOP_MASK_,								0,										no_overflow },
	   {"vmac.vs.1",	"a,s",			"a,s,E",		2,		VOP_MATCH_VMAC_VS,		VOP_MASK_,								0,										no_overflow },
	   {"vmacu",		"a,b",			"a,b,E",		2,		VOP_MATCH_VMACU,		VOP_MASK_,								0,										no_overflow },
	   {"vmacu.1",		"a,b",			"a,b,E",		2,		VOP_MATCH_VMACU,		VOP_MASK_,								0,										no_overflow },
	   {"vmacu.vv",	"a,b",			"a,b,E",		2,		VOP_MATCH_VMACU,		VOP_MASK_,								0,										no_overflow },
	   {"vmacu.vv.1",	"a,b",			"a,b,E",		2,		VOP_MATCH_VMACU,		VOP_MASK_,								0,										no_overflow },
	   {"vmacu.vs",	"a,s",			"a,s,E",		2,		VOP_MATCH_VMACU_VS,	VOP_MASK_,								0,										no_overflow },
	   {"vmacu.vs.1",	"a,s",			"a,s,E",		2,		VOP_MATCH_VMACU_VS,	VOP_MASK_,								0,										no_overflow },

	   {"vccacc",		"v",			"v,E",			1,		VOP_MATCH_VCCACC,		VOP_MASK_,								0,										no_overflow	},

	   {"vcczacc",		"v",				"v,E",			1,		VOP_MATCH_VCCZACC,		VOP_MASK_,								0,										no_overflow	},

	   /* Logical Instructions */
	   {"vand",			"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VAND,			VOP_MASK_,								0,										no_overflow	},
	   {"vand.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VAND,			VOP_MASK_,								0,										no_overflow	},
	   {"vand.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VAND,			VOP_MASK_,								0,										no_overflow	},
	   {"vand.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VAND,			VOP_MASK_,								0,										no_overflow	},
	   {"vand.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VAND_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vand.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VAND_VS,		VOP_MASK_,								0,										no_overflow	},

	   {"vor",			"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VOR,			VOP_MASK_,								0,										no_overflow	},
	   {"vor.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VOR,			VOP_MASK_,								0,										no_overflow	},
	   {"vor.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VOR,			VOP_MASK_,								0,										no_overflow	},
	   {"vor.vv.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VOR,			VOP_MASK_,								0,										no_overflow	},
	   {"vor.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VOR_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vor.vs.1",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VOR_VS,		VOP_MASK_,								0,										no_overflow	},

	   {"vxor",			"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VXOR,			VOP_MASK_,								0,										no_overflow	},
	   {"vxor.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VXOR,			VOP_MASK_,								0,										no_overflow	},
	   {"vxor.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VXOR,			VOP_MASK_,								0,										no_overflow	},
	   {"vxor.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VXOR,			VOP_MASK_,								0,										no_overflow	},
	   {"vxor.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VXOR_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vxor.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VXOR_VS,		VOP_MASK_,								0,										no_overflow	},

	   {"vsll",			"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSLL,			VOP_MASK_,								0,										no_overflow	},
	   {"vsll.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSLL,			VOP_MASK_,								0,										no_overflow	},
	   {"vsll.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSLL,			VOP_MASK_,								0,										no_overflow	},
	   {"vsll.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSLL,			VOP_MASK_,								0,										no_overflow	},
	   {"vsll.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VSLL_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vsll.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VSLL_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vsll.sv",		"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VSLL_SV,		VOP_MASK_,								0,										no_overflow	},
	   {"vsll.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VSLL_SV,		VOP_MASK_,								0,										no_overflow	},

	   {"vsrl",			"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSRL,			VOP_MASK_,								0,										no_overflow	},
	   {"vsrl.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSRL,			VOP_MASK_,								0,										no_overflow	},
	   {"vsrl.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSRL,			VOP_MASK_,								0,										no_overflow	},
	   {"vsrl.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VSRL,			VOP_MASK_,								0,										no_overflow	},
	   {"vsrl.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VSRL_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vsrl.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VSRL_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vsrl.sv",		"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VSRL_SV,		VOP_MASK_,								0,										no_overflow	},
	   {"vsrl.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VSRL_SV,		VOP_MASK_,								0,										no_overflow	},

	   {"vrot",			"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VROT,			VOP_MASK_,								0,										no_overflow	},
	   {"vrot.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VROT,			VOP_MASK_,								0,										no_overflow	},
	   {"vrot.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VROT,			VOP_MASK_,								0,										no_overflow	},
	   {"vrot.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VROT,			VOP_MASK_,								0,										no_overflow	},
	   {"vrot.vs",		"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VROT_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vrot.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VROT_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vrot.sv",		"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VROT_SV,		VOP_MASK_,								0,										no_overflow	},
	   {"vrot.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VROT_SV,		VOP_MASK_,								0,										no_overflow	},

	   // {"vcperm",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VCPERM,		VOP_MASK_,								0,										no_overflow	},

	   /* Memory Instructions */
	   {"vld.b",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VLD_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vld.b.1",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VLD_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vld.h",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VLD_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vld.h.1",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VLD_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vld.w",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VLD_W,		VOP_MASK_,								0,										no_overflow	},
	   {"vld.w.1",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VLD_W,		VOP_MASK_,								0,										no_overflow	},
	   {"vldu.b",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VLDU_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vldu.b.1",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VLDU_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vldu.h",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VLDU_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vldu.h.1",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VLDU_H,		VOP_MASK_,								0,										no_overflow	},

	   {"vst.b",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VST_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vst.b.1",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VST_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vst.h",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VST_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vst.h.1",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VST_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vst.w",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VST_W,		VOP_MASK_,								0,										no_overflow	},
	   {"vst.w.1",		"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VST_W,		VOP_MASK_,								0,										no_overflow	},

	   {"vlds.b",		"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VLDS_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vlds.b.1",		"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VLDS_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vlds.h",		"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VLDS_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vlds.h.1",		"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VLDS_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vlds.w",		"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VLDS_W,		VOP_MASK_,								0,										no_overflow	},
	   {"vlds.w.1",		"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VLDS_W,		VOP_MASK_,								0,										no_overflow	},
	   {"vldsu.b",		"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VLDSU_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vldsu.b.1",	"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VLDSU_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vldsu.h",		"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VLDSU_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vldsu.h.1",	"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VLDSU_H,		VOP_MASK_,								0,										no_overflow	},

	   {"vsts.b",		"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VSTS_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vsts.b.1",		"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VSTS_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vsts.h",		"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VSTS_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vsts.h.1",		"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VSTS_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vsts.w",		"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VSTS_W,		VOP_MASK_,								0,										no_overflow	},
	   {"vsts.w.1",		"r,b,s,i",		"r,b,s,i,E",	4,		VOP_MATCH_VSTS_W,		VOP_MASK_,								0,										no_overflow	},

	   {"vldx.b",		"d,o,b",		"d,o,b,E",		3,		VOP_MATCH_VLDX_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vldx.b.1",		"d,o,b",		"d,o,b,E",		3,		VOP_MATCH_VLDX_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vldx.h",		"d,o,b",		"d,o,b,E",		3,		VOP_MATCH_VLDX_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vldx.h.1",		"d,o,b",		"d,o,b,E",		3,		VOP_MATCH_VLDX_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vldx.w",		"d,o,b",		"d,o,b,E",		3,		VOP_MATCH_VLDX_W,		VOP_MASK_,								0,										no_overflow	},
	   {"vldx.w.1",		"d,o,b",		"d,o,b,E",		3,		VOP_MATCH_VLDX_W,		VOP_MASK_,								0,										no_overflow	},
	   {"vldxu.b",		"d,o,b",		"d,o,b,E",		3,		VOP_MATCH_VLDXU_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vldxu.b.1",	"d,o,b",		"d,o,b,E",		3,		VOP_MATCH_VLDXU_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vldxu.h",		"d,o,b",		"d,o,b,E",		3,		VOP_MATCH_VLDXU_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vldxu.h.1",	"d,o,b",		"d,o,b,E",		3,		VOP_MATCH_VLDXU_H,		VOP_MASK_,								0,										no_overflow	},
	   
	   {"vstx.b",		"r,o,b",		"r,o,b,E",		3,		VOP_MATCH_VSTX_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vstx.b.1",		"r,o,b",		"r,o,b,E",		3,		VOP_MATCH_VSTX_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vstx.h",		"r,o,b",		"r,o,b,E",		3,		VOP_MATCH_VSTX_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vstx.h.1",		"r,o,b",		"r,o,b,E",		3,		VOP_MATCH_VSTX_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vstx.w",		"r,o,b",		"r,o,b,E",		3,		VOP_MATCH_VSTX_W,		VOP_MASK_,								0,										no_overflow	},
	   {"vstx.w.1",		"r,o,b",		"r,o,b,E",		3,		VOP_MATCH_VSTX_W,		VOP_MASK_,								0,										no_overflow	},
	   
	   {"vstxo.b",		"r,o,b",		"r,o,b,E",		3,		VOP_MATCH_VSTXO_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vstxo.b.1",	"r,o,b",		"r,o,b,E",		3,		VOP_MATCH_VSTXO_B,		VOP_MASK_,								0,										no_overflow	},
	   {"vstxo.h",		"r,o,b",		"r,o,b,E",		3,		VOP_MATCH_VSTXO_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vstxo.h.1",	"r,o,b",		"r,o,b,E",		3,		VOP_MATCH_VSTXO_H,		VOP_MASK_,								0,										no_overflow	},
	   {"vstxo.w",		"r,o,b",		"r,o,b,E",		3,		VOP_MATCH_VSTXO_W,		VOP_MASK_,								0,										no_overflow	},
	   {"vstxo.w.1",	"r,o,b",		"r,o,b,E",		3,		VOP_MATCH_VSTXO_W,		VOP_MASK_,								0,										no_overflow	},
	   
	   {"vldl",			"d,a",			"d,a,E",		2,		VOP_MATCH_VLDL,			VOP_MASK_,								0,										no_overflow	},
	   {"vldl.1",		"d,a",			"d,a,E",		2,		VOP_MATCH_VLDL,			VOP_MASK_,								0,										no_overflow	},
	   {"vldl.vv",		"d,a",			"d,a,E",		2,		VOP_MATCH_VLDL,			VOP_MASK_,								0,										no_overflow	},
	   {"vldl.vv.1",	"d,a",			"d,a,E",		2,		VOP_MATCH_VLDL,			VOP_MASK_,								0,										no_overflow	},	   
	   // {"vldl.vs",		"d,s",			"d,s,E",		2,		VOP_MATCH_VLDL_VS,		VOP_MASK_,								0,										no_overflow	},
	   // {"vldl.vs.1",	"d,s",			"d,s,E",		2,		VOP_MATCH_VLDL_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vstl",			"a,b",			"a,b,E",		2,		VOP_MATCH_VSTL,			VOP_MASK_,								0,										no_overflow	},
	   {"vstl.1",		"a,b",			"a,b,E",		2,		VOP_MATCH_VSTL,			VOP_MASK_,								0,										no_overflow	},
	   {"vstl.vv",		"a,b",			"a,b,E",		2,		VOP_MATCH_VSTL,			VOP_MASK_,								0,										no_overflow	},
	   {"vstl.vv.1",	"a,b",			"a,b,E",		2,		VOP_MATCH_VSTL,			VOP_MASK_,								0,										no_overflow	},
	   {"vstl.vs",		"a,s",			"a,s,E",		2,		VOP_MATCH_VSTL_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vstl.vs.1",	"a,s",			"a,s,E",		2,		VOP_MATCH_VSTL_VS,		VOP_MASK_,								0,										no_overflow	},
	   
	   {"vfld",			"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VFLD,			VOP_MASK_,								0,										no_overflow	},

	   {"vfst",			"r,b,i",		"r,b,i,E",		3,		VOP_MATCH_VFST,			VOP_MASK_,								0,										no_overflow	},
	   
	   /* Vector Processing Instructions */
	   {"vmerge",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMERGE,		VOP_MASK_,								0,										no_overflow	},
	   {"vmerge.1",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMERGE,		VOP_MASK_,								0,										no_overflow	},
	   {"vmerge.vv",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMERGE,		VOP_MASK_,								0,										no_overflow	},
	   {"vmerge.vv.1",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VMERGE,		VOP_MASK_,								0,										no_overflow	},
	   {"vmerge.vs",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMERGE_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vmerge.vs.1",	"d,a,s",		"d,a,s,E",		3,		VOP_MATCH_VMERGE_VS,	VOP_MASK_,								0,										no_overflow	},
	   {"vmerge.sv",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VMERGE_SV,	VOP_MASK_,								0,										no_overflow	},
	   {"vmerge.sv.1",	"d,s,b",		"d,s,b,E",		3,		VOP_MATCH_VMERGE_SV,	VOP_MASK_,								0,										no_overflow	},
	   
	   {"vins",			"d,a",			"d,a,E",		2,		VOP_MATCH_VINS,			VOP_MASK_,								0,										no_overflow	},
	   {"vins.vv",		"d,a",			"d,a,E",		2,		VOP_MATCH_VINS,			VOP_MASK_,								0,										no_overflow	},
	   {"vins.vs",		"d,s",			"d,s,E",		2,		VOP_MATCH_VINS_VS,		VOP_MASK_,								0,										no_overflow	},

	   {"vext",			"d,a",			"d,a,E",		2,		VOP_MATCH_VEXT,			VOP_MASK_,								0,										no_overflow	},
	   {"vext.vv",		"d,a",			"d,a,E",		2,		VOP_MATCH_VEXT,			VOP_MASK_,								0,										no_overflow	},
	   {"vext.vs",		"r,a",			"r,a,E",		2,		VOP_MATCH_VEXT_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vextu.vs",		"r,a",			"r,a,E",		2,		VOP_MATCH_VEXTU_VS,		VOP_MASK_,								0,										no_overflow	},
	   
	   {"vcompress",	"d,a",			"d,a,E",		2,		VOP_MATCH_VCOMPRESS,	VOP_MASK_,								0,										no_overflow	},
	   {"vcompress.1",	"d,a",			"d,a,E",		2,		VOP_MATCH_VCOMPRESS,	VOP_MASK_,								0,										no_overflow	},

	   {"vexpand",		"d,a",			"d,a,E",		2,		VOP_MATCH_VEXPAND,		VOP_MASK_,								0,										no_overflow	},
	   {"vexpand.1",	"d,a",			"d,a,E",		2,		VOP_MATCH_VEXPAND,		VOP_MASK_,								0,										no_overflow	},
	   
	   {"vupshift",		"d,a",			"d,a,E",		2,		VOP_MATCH_VUPSHIFT,		VOP_MASK_,								0,										no_overflow	},

	   // {"vexthalf",		"d,a",			"d,a,E",		2,		VOP_MATCH_VEXTHALF,		VOP_MASK_,								0,										no_overflow	},

	   /* Vector Flag Processing Instructions */
	   {"vfins.vs",		"d,s",			"d,s,E",		2,		VOP_MATCH_VFINS,		VOP_MASK_,								0,										no_overflow	},
	   {"vfand",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VFAND,		VOP_MASK_,								0,										no_overflow	},
	   {"vfand.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VFAND,		VOP_MASK_,								0,										no_overflow	},
	   {"vfand.vs",		"d,a,s",			"d,a,s,E",		3,		VOP_MATCH_VFAND_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vfor",			"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VFOR,			VOP_MASK_,								0,										no_overflow	},
	   {"vfor.vv",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VFOR,			VOP_MASK_,								0,										no_overflow	},
	   {"vfor.vs",		"d,a,s",			"d,a,s,E",		3,		VOP_MATCH_VFOR_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vfxor",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VFXOR,		VOP_MASK_,								0,										no_overflow	},
	   {"vfxor.vv",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VFXOR,		VOP_MASK_,								0,										no_overflow	},
	   {"vfxor.vs",		"d,a,s",			"d,a,s,E",		3,		VOP_MATCH_VFXOR_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vfnor",		"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VFNOR,		VOP_MASK_,								0,										no_overflow	},
	   {"vfnor.vv",	"d,a,b",		"d,a,b,E",		3,		VOP_MATCH_VFNOR,		VOP_MASK_,								0,										no_overflow	},
	   {"vfnor.vs",		"d,a,s",			"d,a,s,E",		3,		VOP_MATCH_VFNOR_VS,		VOP_MASK_,								0,										no_overflow	},
	   {"vfclr",		"v",			"v,E",			1,		VOP_MATCH_VFCLR,		VOP_MASK_,								0,										no_overflow	},
	   {"vfset",		"v",			"v,E",			1,		VOP_MATCH_VFSET,		VOP_MASK_,								0,										no_overflow	},
	   
//	   {"vfpop",		"r,a",			"r,a,E",		2,		VOP_MATCH_VFPOP,		VOP_MASK_,								0,										no_overflow	},
//	   {"vfff1",		"r,a",			"r,a,E",		2,		VOP_MATCH_VFFF1,		VOP_MASK_,								0,										no_overflow	},
//	   {"vffl1",		"r,a",			"r,a,E",		2,		VOP_MATCH_VFFL1,		VOP_MASK_,								0,										no_overflow	},
		
	   /* Miscellaneous Instructions */
	   {"vmstc",		"n,s",			"n,s,E",		2,		VOP_MATCH_VMSTC,		VOP_MASK_,								0,										no_overflow	},
	   {"vmcts",		"r,n",			"r,n,E",		2,		VOP_MATCH_VMCTS,		VOP_MASK_,								0,										no_overflow	},
	   // {"vsync",		"",				"E",			0,		VOP_MATCH_VSYNC,		VOP_MASK_,								0,										no_overflow	},

};
/* *INDENT-ON* */

#define NIOS2_NUM_OPCODES \
       ((sizeof nios2_builtin_opcodes) / (sizeof (nios2_builtin_opcodes[0])))
const int bfd_nios2_num_builtin_opcodes = NIOS2_NUM_OPCODES;

/* const removed from the following to allow for dynamic extensions to the
 * built-in instruction set. */
struct nios2_opcode *nios2_opcodes =
  (struct nios2_opcode *) nios2_builtin_opcodes;
int bfd_nios2_num_opcodes = NIOS2_NUM_OPCODES;
#undef NIOS2_NUM_OPCODES
