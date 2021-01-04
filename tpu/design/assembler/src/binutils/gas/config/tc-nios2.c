/* tc-nios2.c -- assemble code for a New Jersey processor.

   Copyright (C) 2003
   by Nigel Gray (ngray@altera.com).


   This file is part of GAS.

   GAS is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2, or (at your option)
   any later version.

   GAS is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with GAS; see the file COPYING.  If not, write to the Free
   Software Foundation, 59 Temple Place - Suite 330, Boston, MA
   02111-1307, USA.  */


#include <stdio.h>
#include <limits.h>
#include <string.h>
#include <stdlib.h>
#include "as.h"
#include "opcode/nios2.h"
#include "elf/nios2.h"
#include "tc-nios2.h"
#include "bfd.h"
#include "dwarf2dbg.h"
#include "subsegs.h"
#include "safe-ctype.h"

/* We can choose our endianness at run-time, regardless of configuration */
extern int target_big_endian;

#ifndef OBJ_ELF
	/* we are not supporting any other target
	   so we throw a compile time error */
OBJ_ELF not defined
#endif
  typedef enum
{
  relax_section = 0,
  relax_none,
  relax_all
}
relax_optionT;


/* struct contains all assembler options set with .set */
struct
{
  /*
     .set noat -> noat = 1 allows assembly code to use at without warning
     and macro expansions will generate a warning.
     .set at -> noat = 0, assembly code using at will warn
     macro expansions will not generate warnings
   */
  bfd_boolean noat;

  /*
     .set nobreak -> nobreak = 1 allows assembly code to use ba,bt without warning
     .set break -> nobreak = 0, assembly code using ba,bt will warn
   */
  bfd_boolean nobreak;

  /*
     .cmd line option -relax-all allows all branches and calls to be replaced
     with longer versions
     -no-relax inhibits branch/call conversion
     default value is relax_section, which relaxes branches within a section
   */
  relax_optionT relax;

}
nios2_as_options = 
{
FALSE, FALSE, relax_section};


typedef struct nios2_insn_reloc
{
  /* any expression in the instruction is parsed into
     this field which is passed to fix_new_exp() to
     generate a fixup */
  expressionS reloc_expression;

  /* the type of the relocation to be applied */
  bfd_reloc_code_real_type reloc_type;

  /* pc relative */
  unsigned int reloc_pcrel;

  /* the next relocation to be applied to the instruction */
  struct nios2_insn_reloc *reloc_next;
}
nios2_insn_relocS;


/* ------------------------------------------------------------------
	This struct is used by the functions in tc-nios2.c to assemble an
	instruction
   ------------------------------------------------------------------*/

typedef struct nios2_insn_info
{
  /* assembled instruction */
  unsigned long insn_code;
  /* ptr to the relevant bit of the opcode table */
  const struct nios2_opcode *insn_nios2_opcode;
  /* after parsing ptrs to the tokens in the instruction fill this array
     it is terminated with a null pointer ( hence the first +1
     The second +1 is because in some parts of the code the opcode
     is not counted as a token, but still placed in this array*/
  const char *insn_tokens[NIOS2_MAX_INSN_TOKENS + 1 + 1];

  /* this holds information used to generate fixups
     and eventually relocations if it is not null */
  nios2_insn_relocS *insn_reloc;
}
nios2_insn_infoS;


/*
	This struct associates an argument assemble function with
	an argument syntax string. Used by the assembler to find out
	how to parse and assemble a set of instruction operands and return the instruction
	field values
*/

typedef struct nios2_arg_info
{
  const char *args;
  void (*assemble_args_func) (nios2_insn_infoS * insn_info);
}
nios2_arg_infoS;

/*
	This struct is used to convert New Jersey pseudo-ops into the
	corresponding real op
 */
typedef struct nios2_ps_insn_info
{
  const char *pseudo_insn;
  const char *insn;
  const char *arg_modifier;
  void (*arg_modifer_func) (const char *arg, char **parsedArgs, int numArg,
			    int startIndex);
  int num;
  int index;
}
nios2_ps_insn_infoS;



/* function prototypes */
static void NIOS2_CHECK_ASSEMBLY (unsigned int opcode,
				  const char *exp_opcode);
static void s_nios2_sdata (int);
void nios2_assemble_args_dst (nios2_insn_infoS * insn_info);
void nios2_assemble_args_tsi (nios2_insn_infoS * insn_info);
void nios2_assemble_args_tsu (nios2_insn_infoS * insn_info);
void nios2_assemble_args_o (nios2_insn_infoS * insn_info);
void nios2_assemble_args_m (nios2_insn_infoS * insn_info);
void nios2_assemble_args_s (nios2_insn_infoS * insn_info);
void nios2_assemble_args_tis (nios2_insn_infoS * insn_info);
void nios2_assemble_args_dc (nios2_insn_infoS * insn_info);
void nios2_assemble_args_cs (nios2_insn_infoS * insn_info);
void nios2_assemble_args_ldst (nios2_insn_infoS * insn_info);
void nios2_assemble_args_none (nios2_insn_infoS * insn_info);
void nios2_assemble_args_dsj (nios2_insn_infoS * insn_info);
void nios2_assemble_args_is (nios2_insn_infoS * insn_info);
void nios2_assemble_args_sto (nios2_insn_infoS * insn_info);
void nios2_assemble_args_d (nios2_insn_infoS * insn_info);
void nios2_assemble_args_b (nios2_insn_infoS * insn_info);

nios2_insn_relocS *nios2_insn_reloc_new (bfd_reloc_code_real_type reloc_type,
					 unsigned int pcrel);
void nios2_insn_reloc_destroy (nios2_insn_relocS * reloc);
unsigned long nios2_assemble_expression (const char *exprstr,
					 nios2_insn_infoS * insn,
					 nios2_insn_relocS * prev_reloc,
					 bfd_reloc_code_real_type reloc_type,
					 unsigned int pcrel);
char *nios2_consume_separator (char *argStr, const char *separator);
char *nios2_consume_arg (char *argStr, const char *argType);
void nios2_parse_args (char *argStr, const char *parseStr, char **parsedArgs);

void nios2_modify_arg (const char *modifier, char **parsedArgs, int unused,
		       int index);
void nios2_append_arg (const char *append, char **parsedArgs, int numAppend,
		       int startIndex);
void nios2_insert_arg (const char *insert, char **parsedArgs, int numInsert,
		       int startIndex);
void nios2_swap_args (const char *unused, char **parsedArgs, int index_1,
		      int index_2);
void nios2_negate_arg (const char *modifier ATTRIBUTE_UNUSED,
		       char **parsedArgs, int unused ATTRIBUTE_UNUSED,
		       int index);
void nios2_translate_pseudo_insn (nios2_insn_infoS * insn);
valueT md_chars_to_number (char *buf, int n);
void md_number_to_imm (char *buf, valueT val, int n);
void md_number_to_disp (char *buf, valueT val, int n);
void md_number_to_field (char *buf, valueT val, int n);
static void nios2_align (int log_size, const char *pfill, symbolS * sym);
static void s_nios2_ucons (int nbytes);
static void s_nios2_set (int equiv);
static void s_nios2_align (int ignore);
static void s_nios2_text (int);
static void s_nios2_data (int);
static void s_nios2_section (int);
static bfd_boolean nios2_coproc_reg (const char *reg_name);
static void output_insn (void);
static void output_ubranch (void);
static void output_cbranch (void);
static void output_call (void);
static void output_movia (void);
static void output_andi (void);
static void output_addi (void);
static void output_ori (void);
static void output_xori (void);
static int can_evaluate_expr (void);
static int get_expr_value (void);

/* Vector Co-processor function prototypes */
void nios2_assemble_args_vdva (nios2_insn_infoS * insn_info);
void nios2_assemble_args_vdvavb (nios2_insn_infoS * insn_info);
void nios2_assemble_args_vdrsvb (nios2_insn_infoS * insn_info);
void nios2_assemble_args_vdvars (nios2_insn_infoS * insn_info);
void nios2_assemble_args_vavb (nios2_insn_infoS * insn_info);
void nios2_assemble_args_rsvb (nios2_insn_infoS * insn_info);  
void nios2_assemble_args_vd (nios2_insn_infoS * insn_info);
/*void nios2_assemble_args_vdvavc (nios2_insn_infoS * insn_info);
void nios2_assemble_args_vavbvc (nios2_insn_infoS * insn_info);*/
void nios2_assemble_args_vdrs (nios2_insn_infoS * insn_info);
void nios2_assemble_args_rsdva (nios2_insn_infoS * insn_info);
void nios2_assemble_args_rsdvc (nios2_insn_infoS * insn_info);
void nios2_assemble_args_vars (nios2_insn_infoS * insn_info);
void nios2_assemble_args_vm_type (nios2_insn_infoS * insn_info);
void nios2_assemble_args_vms_type (nios2_insn_infoS * insn_info);
void nios2_assemble_args_vmx_load (nios2_insn_infoS * insn_info);
void nios2_assemble_args_vmx_store (nios2_insn_infoS * insn_info);
void nios2_assemble_args_vcrs (nios2_insn_infoS * insn_info);
void nios2_assemble_args_rsvc (nios2_insn_infoS * insn_info);


bfd_boolean nios2_check_overflow (valueT fixup, reloc_howto_type * howto);

/* The known current alignment of the current section.  */
static int nios2_current_align;
static segT nios2_current_align_seg;

/* The last seen label in the current section.  This is used to auto-align
   labels preceeding instructions.  */
static symbolS *nios2_last_label;


static int nios2_auto_align_on = 1;

/* This array holds the chars that always start a comment.  If the
    pre-processor is disabled, these aren't very useful */
const char comment_chars[] = "#";

/* This array holds the chars that only start a comment at the beginning of
   a line.  If the line seems to have the form '# 123 filename'
   .line and .file directives will appear in the pre-processed output */
/* Note that input_file.c hand checks for '#' at the beginning of the
   first line of the input file.  This is because the compiler outputs
   #NO_APP at the beginning of its output.  */
/* Also note that C style comments are always supported.  */
const char line_comment_chars[] = "#";

/* This array holds machine specific line separator characters.  */
const char line_separator_chars[] = ";";

/* Chars that can be used to separate mant from exp in floating point nums */
const char EXP_CHARS[] = "eE";

/* Chars that mean this number is a floating point constant */
/* As in 0f12.456 */
/* or    0d1.2345e12 */
const char FLT_CHARS[] = "rRsSfFdDxXpP";

/* Also be aware that MAXIMUM_NUMBER_OF_CHARS_FOR_FLOAT may have to be
   changed in read.c .  Ideally it shouldn't have to know about it at all,
   but nothing is ideal around here.
 */

/* handle of the OPCODE hash table */
static struct hash_control *nios2_opcode_hash = NULL;

/* handle of the Register hash table */
static struct hash_control *nios2_reg_hash = NULL;

/* handle of the parse args hash table */
static struct hash_control *nios2_arg_hash = NULL;

/* pseudo-op hash table */
static struct hash_control *nios2_ps_hash = NULL;

/* mode of the assembler */
typedef enum
{
  NIOS2_MODE_ASSEMBLE,		// ordinary operation
  NIOS2_MODE_TEST		// hidden mode used for self testing
}
NIOS2_MODE;

static NIOS2_MODE nios2_mode = NIOS2_MODE_ASSEMBLE;

/* this function is used to in self-checking mode
   to check the assembled instruction
   opcode should be the assembled opcode, and exp_opcode
   the parsed string representing the expected opcode */
void
NIOS2_CHECK_ASSEMBLY (unsigned int opcode, const char *exp_opcode)
{
  if (nios2_mode == NIOS2_MODE_TEST)
    {
      if ((exp_opcode) == NULL)
	{
	  as_bad (_("expecting opcode string in self test mode"));
	}
      else if ((opcode) != strtoul ((exp_opcode), NULL, 16))
	{
	  as_bad (_("assembly 0x%08x, expected %s"), (opcode), (exp_opcode));
	}
    }
}

/* Machine-dependent command-line options */

const char *md_shortopts = "r";

struct option md_longopts[] = {
#define OPTION_RELAX_ALL (OPTION_MD_BASE + 0)
  {"relax-all", no_argument, NULL, OPTION_RELAX_ALL},
#define OPTION_NORELAX (OPTION_MD_BASE + 1)
  {"no-relax", no_argument, NULL, OPTION_NORELAX},
#define OPTION_RELAX_SECTION (OPTION_MD_BASE + 2)
  {"relax-section", no_argument, NULL, OPTION_RELAX_SECTION},
#define OPTION_EB (OPTION_MD_BASE + 3)
  {"EB", no_argument, NULL, OPTION_EB},
#define OPTION_EL (OPTION_MD_BASE + 4)
  {"EL", no_argument, NULL, OPTION_EL}
};

size_t md_longopts_size = sizeof (md_longopts);

/* Machine dependent pseudo-ops
   These are actually assembler directives
   format of each entry is

  { "directive", handler_func, param }
*/
const pseudo_typeS md_pseudo_table[] = {
  {"align", s_nios2_align, 0},
  {"text", s_nios2_text, 0},
  {"data", s_nios2_data, 0},
  {"section", s_nios2_section, 0},
  {"section.s", s_nios2_section, 0},
  {"sect", s_nios2_section, 0},
  {"sect.s", s_nios2_section, 0},
  /* .dword and .half are included for
     compatibility with MIPS */
  {"dword", cons, 8},
  {"half", cons, 2},
  /* NIOS2 native word size is 4 bytes, so we override
     the GAS default of 2 */
  {"word", cons, 4},
  /* explicitly unaligned directives */
  {"2byte", s_nios2_ucons, 2},
  {"4byte", s_nios2_ucons, 4},
  {"8byte", s_nios2_ucons, 8},
  {"16byte", s_nios2_ucons, 16},
#ifdef OBJ_ELF
  {"sdata", s_nios2_sdata, 0},
#endif
  {"set", s_nios2_set, 0},
  {NULL, NULL, 0}
};

#define BYTE_F		32764
#define BYTE_B		-32768
#define ABS		(long)0xffffffff	/* special value to indicate non-pc relative jmp */

#define UBRANCH		1
#define UJMP		2
#define CBRANCH		3
#define CJMP		4


#define RELAX_MAX_SIZE(type) nios2_relax_table[nios2_relax_table[type].rlx_more].rlx_length
#define RELAX_SIZE(type) nios2_relax_table[type].rlx_length
#define RELAX_SUBSTATE(type)  type

/* machine dependent relaxations */
struct relax_type nios2_relax_table[] = {
  /* first entry unused (ends relaxation sequence) */
  {1, 1, 0, 0},
  /* unconditional branch */
  {BYTE_F, BYTE_B, 4, 2},	/* br label (label is in range) */
  /* unconditional jmp */
  {ABS, ABS, 12, 0},		/* movhi at, %hi(label) ; ori at, %lo(label) ; jmp at */
  /* conditional branch */
  {BYTE_F, BYTE_B, 4, 4},	/* br{cond} label (label is in range) */
  /* conditional jmp */
  {ABS, ABS, 16, 0},		/* br{opp_cond} skip ; movhi at, %hi(label) ; ori at, %lo(label) ; jmp at ; skip: } */
};


/* this is just the generic relax_frag function but
   amended to include absolute jmps in the relax table
 */

long
nios2_relax_frag (segT segment, fragS * fragP, long stretch)
{
  const relax_typeS *this_type;
  const relax_typeS *start_type;
  relax_substateT next_state;
  relax_substateT this_state;
  long growth;
  offsetT aim;
  addressT target;
  addressT address;
  symbolS *symbolP;
  const relax_typeS *table;

  target = fragP->fr_offset;
  address = fragP->fr_address;
  table = nios2_relax_table;
  this_state = fragP->fr_subtype;
  start_type = this_type = table + this_state;
  symbolP = fragP->fr_symbol;

  if (symbolP)
    {
      fragS *sym_frag;

      sym_frag = symbol_get_frag (symbolP);

#ifndef DIFF_EXPR_OK
#if !defined (MANY_SEGMENTS) && !defined (BFD_ASSEMBLER)
      know ((S_GET_SEGMENT (symbolP) == SEG_ABSOLUTE)
	    || (S_GET_SEGMENT (symbolP) == SEG_DATA)
	    || (S_GET_SEGMENT (symbolP) == SEG_BSS)
	    || (S_GET_SEGMENT (symbolP) == SEG_TEXT));
#endif
      know (sym_frag != NULL);
#endif
      know (!(S_GET_SEGMENT (symbolP) == absolute_section)
	    || sym_frag == &zero_address_frag);
      target += S_GET_VALUE (symbolP);

      /* If frag has yet to be reached on this pass,
         assume it will move by STRETCH just as we did.
         If this is not so, it will be because some frag
         between grows, and that will force another pass.  */

      if (stretch != 0
	  && sym_frag->relax_marker != fragP->relax_marker
	  && S_GET_SEGMENT (symbolP) == segment)
	{
	  target += stretch;
	}
    }


  /* NG we subtract 4 because all pc relative branches are
     from the next instruction */
  aim = target - address - fragP->fr_fix - 4;

  if (aim < 0)
    {
      /* Look backwards.  */
      for (next_state = this_type->rlx_more; next_state;)
	{
	  if (aim >= this_type->rlx_backward
	      || this_type->rlx_backward == ABS)
	    next_state = 0;
	  else
	    {
	      /* Grow to next state.  */
	      this_state = next_state;
	      this_type = table + this_state;
	      next_state = this_type->rlx_more;
	    }
	}
    }
  else
    {
      /* Look forwards.  */

      for (next_state = this_type->rlx_more; next_state;)
	{
	  if (aim <= this_type->rlx_forward || this_type->rlx_forward == ABS)
	    next_state = 0;
	  else
	    {
	      /* Grow to next state.  */
	      this_state = next_state;
	      this_type = table + this_state;
	      next_state = this_type->rlx_more;
	    }
	}
    }


  growth = this_type->rlx_length - start_type->rlx_length;

  if (growth != 0)
    fragP->fr_subtype = this_state;

  return growth;
}

/*--------------------------------------------------------------------------------
   The next table associates pointers to functions which parse the arguments to an
   instruction and fill in the relevant fields of the instruction
  --------------------------------------------------------------------------------*/

const nios2_arg_infoS nios2_arg_info_structs[] = {
  /* args                 assemble_args_func */
  {"d,s,t", nios2_assemble_args_dst},
  {"d,s,t,E", nios2_assemble_args_dst},
  {"t,s,i", nios2_assemble_args_tsi},
  {"t,s,i,E", nios2_assemble_args_tsi},
  {"t,s,u", nios2_assemble_args_tsu},
  {"t,s,u,E", nios2_assemble_args_tsu},
  {"s,t,o", nios2_assemble_args_sto},
  {"s,t,o,E", nios2_assemble_args_sto},
  {"o", nios2_assemble_args_o},
  {"o,E", nios2_assemble_args_o},
  {"s", nios2_assemble_args_s},
  {"s,E", nios2_assemble_args_s},
  {"", nios2_assemble_args_none},
  {"E", nios2_assemble_args_none},
  {"i(s)", nios2_assemble_args_is},
  {"i(s)E", nios2_assemble_args_is},
  {"m", nios2_assemble_args_m},
  {"m,E", nios2_assemble_args_m},
  {"t,i(s)", nios2_assemble_args_tis},
  {"t,i(s)E", nios2_assemble_args_tis},
  {"d,c", nios2_assemble_args_dc},
  {"d,c,E", nios2_assemble_args_dc},
  {"c,s", nios2_assemble_args_cs},
  {"c,s,E", nios2_assemble_args_cs},
  {"l,d,s,t", nios2_assemble_args_ldst},
  {"l,d,s,t,E", nios2_assemble_args_ldst},
  {"d,s,j", nios2_assemble_args_dsj},
  {"d,s,j,E", nios2_assemble_args_dsj},
  {"d", nios2_assemble_args_d},
  {"d,E", nios2_assemble_args_d},
  {"b", nios2_assemble_args_b},
  {"b,E", nios2_assemble_args_b},
  
  /* Vector Processor Arguments */
  {"d,a", nios2_assemble_args_vdva},
  {"d,a,E", nios2_assemble_args_vdva},
  {"d,a,b", nios2_assemble_args_vdvavb},
  {"d,a,b,E", nios2_assemble_args_vdvavb},
  {"d,s,b", nios2_assemble_args_vdrsvb},
  {"d,s,b.E", nios2_assemble_args_vdrsvb},
  {"d,a,s", nios2_assemble_args_vdvars},
  {"d,a,s,E", nios2_assemble_args_vdvars},
  {"a,b", nios2_assemble_args_vavb},
  {"a,b,E", nios2_assemble_args_vavb},
  {"v", nios2_assemble_args_vd},
  {"v,E", nios2_assemble_args_vd},
  {"d,s", nios2_assemble_args_vdrs},
  {"d,s,E", nios2_assemble_args_vdrs},
  {"r,a", nios2_assemble_args_rsdva},
  {"r,a,E", nios2_assemble_args_rsdva},
  {"s,b", nios2_assemble_args_rsvb},
  {"s,b,E", nios2_assemble_args_rsvb},
  {"a,s", nios2_assemble_args_vars},
  {"a,s,E", nios2_assemble_args_vars},
  
  /* Vector miscellaneous instructions */
  {"n,s", nios2_assemble_args_vcrs},
  {"n,s,E", nios2_assemble_args_vcrs},
  {"r,n", nios2_assemble_args_rsvc},
  {"r,n,E", nios2_assemble_args_rsvc},
  
  //Vector Memory
  {"r,b,i", nios2_assemble_args_vm_type},
  {"r,b,i,E", nios2_assemble_args_vm_type},
  {"r,b,s,i", nios2_assemble_args_vms_type},
  {"r,b,s,i,E", nios2_assemble_args_vms_type},
  {"d,o,b", nios2_assemble_args_vmx_load},
  {"d,o,b,E", nios2_assemble_args_vmx_load},
  {"r,o,b", nios2_assemble_args_vmx_store},
  {"r,o,b,E", nios2_assemble_args_vmx_store}
  
    
};

#define NIOS2_NUM_ARGS \
	((sizeof(nios2_arg_info_structs)/sizeof(nios2_arg_info_structs[0])))
const int nios2_num_arg_info_structs = NIOS2_NUM_ARGS;


const nios2_ps_insn_infoS nios2_ps_insn_info_structs[] = {
  /* pseudo-op         real-op         arg                     arg_modifier_func   num         index */
  {"mov", "add", "zero", nios2_append_arg, 1, 3},
  {"movi", "addi", "zero", nios2_insert_arg, 1, 2},
  {"movhi", "orhi", "zero", nios2_insert_arg, 1, 2},
  {"movui", "ori", "zero", nios2_insert_arg, 1, 2},
  {"movia", "orhi", "zero", nios2_insert_arg, 1, 2},
  {"nop", "add", "zero", nios2_append_arg, 3, 1},
  {"bgt", "blt", "", nios2_swap_args, 1, 2},
  {"bgtu", "bltu", "", nios2_swap_args, 1, 2},
  {"ble", "bge", "", nios2_swap_args, 1, 2},
  {"bleu", "bgeu", "", nios2_swap_args, 1, 2},
  {"cmpgt", "cmplt", "", nios2_swap_args, 2, 3},
  {"cmpgtu", "cmpltu", "", nios2_swap_args, 2, 3},
  {"cmple", "cmpge", "", nios2_swap_args, 2, 3},
  {"cmpleu", "cmpgeu", "", nios2_swap_args, 2, 3},
  {"cmpgti", "cmpgei", "+1", nios2_modify_arg, 0, 3},
  {"cmpgtui", "cmpgeui", "+1", nios2_modify_arg, 0, 3},
  {"cmplei", "cmplti", "+1", nios2_modify_arg, 0, 3},
  {"cmpleui", "cmpltui", "+1", nios2_modify_arg, 0, 3},
  {"subi", "addi", "", nios2_negate_arg, 0, 3}
  /* add further pseudo-ops here */
};

#define NIOS2_NUM_PSEUDO_INSNS \
	((sizeof(nios2_ps_insn_info_structs)/sizeof(nios2_ps_insn_info_structs[0])))
const int nios2_num_ps_insn_info_structs = NIOS2_NUM_PSEUDO_INSNS;

/* special relocation directive strings */

struct nios2_special_relocS
{
  const char *string;
  bfd_reloc_code_real_type reloc_type;
};

struct nios2_special_relocS nios2_special_reloc[] = {
  {"%hiadj", BFD_RELOC_NIOS2_HIADJ16},
  {"%hi", BFD_RELOC_NIOS2_HI16},
  {"%lo", BFD_RELOC_NIOS2_LO16},
  {"%gprel", BFD_RELOC_NIOS2_GPREL}
};

#define NIOS2_NUM_SPECIAL_RELOCS \
	(sizeof(nios2_special_reloc)/sizeof(nios2_special_reloc[0]))
const int nios2_num_special_relocs = NIOS2_NUM_SPECIAL_RELOCS;

/*
	The function nios2_modify_arg appends the string modifier to the string contained
	in the argument at index in the array parsedArgs[]
*/
void
nios2_modify_arg (const char *modifier,
		  char **parsedArgs, int unused ATTRIBUTE_UNUSED, int index)
{
  assert (index < NIOS2_MAX_INSN_TOKENS);

/*
    we can't just strcat here because strcat will free the memory pointed to by the first
    argument and allocate new memory - but at this stage, parsedArgs[index] may point into
    the middle of a block of allocated memory, so trying to free it will cause a seg fault.

 */
  char *tmp = parsedArgs[index];
  parsedArgs[index] =
    (char *) malloc (strlen (parsedArgs[index]) + strlen (modifier) + 1);
  strcpy (parsedArgs[index], tmp);
  strcat (parsedArgs[index], modifier);
}


void
nios2_negate_arg (const char *modifier ATTRIBUTE_UNUSED,
		  char **parsedArgs, int unused ATTRIBUTE_UNUSED, int index)
{
  char *tmp = parsedArgs[index];
  parsedArgs[index] =
    (char *) malloc (strlen ("~(") + strlen (parsedArgs[index]) +
		     strlen (")+1") + 1);

  strcpy (parsedArgs[index], "~(");
  strcat (parsedArgs[index], tmp);
  strcat (parsedArgs[index], ")+1");
}

/*
	The function nios2_swap_args swaps the pointers at indices index_1 and
	index_2 in the array parsedArgs[] - this is used for operand swapping
	for comparison operations
  */
void
nios2_swap_args (const char *unused ATTRIBUTE_UNUSED,
		 char **parsedArgs, int index_1, int index_2)
{
  char *tmp;
  assert (index_1 < NIOS2_MAX_INSN_TOKENS && index_2 < NIOS2_MAX_INSN_TOKENS);
  tmp = parsedArgs[index_1];
  parsedArgs[index_1] = parsedArgs[index_2];
  parsedArgs[index_2] = tmp;
}

/*
	This function appends the string append to the array of strings in
	parsedArgs numAppend times starting at index startIndex in the array
*/
void
nios2_append_arg (const char *append, char **parsedArgs, int numAppend,
		  int startIndex)
{
  int i, count;
  char *tmp;

  assert ((startIndex + numAppend) < NIOS2_MAX_INSN_TOKENS);
  i = startIndex;
  count = numAppend;

  if (nios2_mode == NIOS2_MODE_TEST)
    tmp = parsedArgs[startIndex];
  else
    tmp = NULL;

  while (count > 0)
    {
      parsedArgs[i] = (char *) append;
      ++i;
      --count;
    }

  assert (i == (startIndex + numAppend));
  parsedArgs[i] = tmp;
  parsedArgs[i + 1] = NULL;
}

/* This function inserts the string insert numInsert times in the array parsedArgs,
	starting at the index startIndex
 */
void
nios2_insert_arg (const char *insert, char **parsedArgs, int numInsert,
		  int startIndex)
{
  int i, count, from, to;

  assert ((startIndex + numInsert) < NIOS2_MAX_INSN_TOKENS);

  to = startIndex + numInsert;
  from = startIndex;

  /* move the existing arguments up to create space */
  i = NIOS2_MAX_INSN_TOKENS;
  while ((i - numInsert) >= startIndex)
    {
      parsedArgs[i] = parsedArgs[i - numInsert];
      --i;
    }

  i = startIndex;
  count = numInsert;
  while (count > 0)
    {
      parsedArgs[i] = (char *) insert;
      ++i;
      --count;
    }
}

/*
	This function swaps the pseudo-op for a real op
	FIXME - only works for 1-to-1 correspondence
  */
void
nios2_translate_pseudo_insn (nios2_insn_infoS * insn)
{

  nios2_ps_insn_infoS *ps_insn;

  /* find which real insn the pseudo-op transates to and
     switch the insn_info ptr to point to it */
  ps_insn =
    (nios2_ps_insn_infoS *) hash_find (nios2_ps_hash,
				       insn->insn_nios2_opcode->name);

  if (ps_insn != NULL)
    {
      insn->insn_nios2_opcode =
	(struct nios2_opcode *) hash_find (nios2_opcode_hash, ps_insn->insn);
      insn->insn_tokens[0] = insn->insn_nios2_opcode->name;
      // modify the args so they work with the real insn
      ps_insn->arg_modifer_func (ps_insn->arg_modifier,
				 (char **) insn->insn_tokens, ps_insn->num,
				 ps_insn->index);
    }
  else
    {
      // we cannot recover from this
      as_fatal (_("unrecognized pseudo-instruction %s"),
		ps_insn->pseudo_insn);
    }
}

/********************************************************************
	The following functions are called by machine-independent parts of
	the assembler
 ********************************************************************/

/*
   Function : void md_parse_option
              (char** option_ptr, int* argc_ptr, char*** argv_ptr)

   Description :

 */
int
md_parse_option (int c, char *arg ATTRIBUTE_UNUSED)
{
  switch (c)
    {
    case 'r':
      /* hidden option for self-test mode */
      nios2_mode = NIOS2_MODE_TEST;
      break;
    case OPTION_RELAX_ALL:
      nios2_as_options.relax = relax_all;
      break;
    case OPTION_NORELAX:
      nios2_as_options.relax = relax_none;
      break;
    case OPTION_RELAX_SECTION:
      nios2_as_options.relax = relax_section;
      break;
    case OPTION_EB:
      target_big_endian = 1;
      break;
    case OPTION_EL:
      target_big_endian = 0;
      break;
    default:
      return 0;
      break;
    }

  return 1;
}

/*
 * We can choose to be big-endian or little-endian at runtime based
 * on a switch.
 */
const char *
nios2_target_format (void)
{
  return target_big_endian ? "elf32-bignios2" : "elf32-littlenios2";
}

/*
   Function : md_show_usage(FILE* stream)

  Description : machine-dependent usage message
*/
void
md_show_usage (FILE * stream)
{
  fprintf (stream, "\
        NIOS2 options:\n\
  -relax-all            replace all branch and call instructions with jmp and callr sequences\n\
  -relax-section        replace identified out of range branches with jmp sequences (default)\n\
  -no-relax             do not replace any branches or calls\n\
  -EB                   force big-endian byte ordering\n\
  -EL                   force little-endian byte ordering\n");
}

/*
   Function : void md_begin()

   Description :
      This function is called once, at assembler startup time.
      It should set up all the tables, etc. that the MD part of the
	  assembler will need.
*/
void
md_begin ()
{
  int i;
  const char *inserted;

  /* create and fill a hashtable for the New Jersey opcodes, registers and arguments */
  nios2_opcode_hash = hash_new ();
  nios2_reg_hash = hash_new ();
  nios2_arg_hash = hash_new ();
  nios2_ps_hash = hash_new ();

  for (i = 0; i < NUMOPCODES; ++i)
    {
      inserted =
	hash_insert (nios2_opcode_hash, nios2_opcodes[i].name,
		     (PTR) & nios2_opcodes[i]);
      if (inserted != NULL)
	{
	  fprintf (stderr, _("internal error: can't hash `%s': %s\n"),
		   nios2_opcodes[i].name, inserted);
	  /* Probably a memory allocation problem?  Give up now.  */
	  as_fatal (_("Broken assembler.  No assembly attempted."));
	}
    }

  for (i = 0; i < nios2_num_regs; ++i)
    {
      inserted =
	hash_insert (nios2_reg_hash, nios2_regs[i].name,
		     (PTR) & nios2_regs[i]);
      if (inserted != NULL)
	{
	  fprintf (stderr, _("internal error: can't hash `%s': %s\n"),
		   nios2_regs[i].name, inserted);
	  /* Probably a memory allocation problem?  Give up now.  */
	  as_fatal (_("Broken assembler.  No assembly attempted."));
	}

    }

  for (i = 0; i < nios2_num_arg_info_structs; ++i)
    {
      inserted =
	hash_insert (nios2_arg_hash, nios2_arg_info_structs[i].args,
		     (PTR) & nios2_arg_info_structs[i]);
      if (inserted != NULL)
	{
	  fprintf (stderr, _("internal error: can't hash `%s': %s\n"),
		   nios2_arg_info_structs[i].args, inserted);
	  /* Probably a memory allocation problem?  Give up now.  */
	  as_fatal (_("Broken assembler.  No assembly attempted."));
	}
    }

  for (i = 0; i < nios2_num_ps_insn_info_structs; ++i)
    {
      inserted =
	hash_insert (nios2_ps_hash, nios2_ps_insn_info_structs[i].pseudo_insn,
		     (PTR) & nios2_ps_insn_info_structs[i]);
      if (inserted != NULL)
	{
	  fprintf (stderr, _("internal error: can't hash `%s': %s\n"),
		   nios2_ps_insn_info_structs[i].pseudo_insn, inserted);
	  /* Probably a memory allocation problem?  Give up now.  */
	  as_fatal (_("Broken assembler.  No assembly attempted."));
	}
    }

  /* assembler option defaults */
  nios2_as_options.noat = FALSE;
  nios2_as_options.nobreak = FALSE;

  /* debug information is incompatible with relaxation */
  if (debug_type != DEBUG_UNSPECIFIED)
    {
      nios2_as_options.relax = relax_none;
    }

  /* initialize the alignment data */
  nios2_current_align_seg = now_seg;
  nios2_last_label = NULL;
  nios2_current_align = 0;
}




/* made this global to avoid changing one function prototype */
nios2_insn_infoS insn;

/*
   Function: void md_assemble(char* op_str)

   Description: assembles a single line of Nios II assembly
                language
 */
void
md_assemble (char *op_str)
{
  char *argstr; 
  char *op_strdup;
  nios2_arg_infoS *arg_info;
  unsigned long saved_pinfo = 0;

  /* make sure we are aligned on a 4-byte boundary */
  if (nios2_current_align < 2)
    nios2_align (2, NULL, nios2_last_label);
  else if (nios2_current_align > 2)
    nios2_current_align = 2;
  nios2_last_label = NULL;


  /* we don't want to clobber to op_str
     because we want to be able to use it in messages */
  op_strdup = strdup (op_str);

  insn.insn_tokens[0] = strtok (op_strdup, " ");
  argstr = strtok (NULL, "");

  /* assemble the opcode */
  insn.insn_nios2_opcode =
    (struct nios2_opcode *) hash_find (nios2_opcode_hash,
				       insn.insn_tokens[0]);
  insn.insn_reloc = NULL;

  if (insn.insn_nios2_opcode != NULL)
    {
      /* set the opcode for the instruction */
      insn.insn_code = insn.insn_nios2_opcode->match;

      /* parse the arguments pointed to by argstr */
      if (nios2_mode == NIOS2_MODE_ASSEMBLE)
	{
	  nios2_parse_args (argstr, insn.insn_nios2_opcode->args,
			    (char **) &insn.insn_tokens[1]);
	}
      else
	{
	  nios2_parse_args (argstr, insn.insn_nios2_opcode->args_test,
			    (char **) &insn.insn_tokens[1]);
	}

      /* we need to preserve the MOVIA macro as this is clobbered by translate_pseudo_insn */
      if (insn.insn_nios2_opcode->pinfo == NIOS2_INSN_MACRO_MOVIA)
	{
	  saved_pinfo = NIOS2_INSN_MACRO_MOVIA;
	}
      /* if the instruction is an pseudo-instruction, we want to replace it with its
         real equivalent, and then continue */
      if ((insn.insn_nios2_opcode->pinfo & NIOS2_INSN_MACRO) ==
	  NIOS2_INSN_MACRO)
	{
	  nios2_translate_pseudo_insn (&insn);
	}

      /* find the assemble function, and call it */
      arg_info =
	(nios2_arg_infoS *) hash_find (nios2_arg_hash,
				       insn.insn_nios2_opcode->args);
      if (arg_info != NULL)
	{
	  arg_info->assemble_args_func (&insn);

	  if (nios2_as_options.relax != relax_none
	      && insn.insn_nios2_opcode->pinfo & NIOS2_INSN_UBRANCH)
	    output_ubranch ();
	  else if (nios2_as_options.relax != relax_none
		   && insn.insn_nios2_opcode->pinfo & NIOS2_INSN_CBRANCH)
	    output_cbranch ();
	  else if (nios2_as_options.relax == relax_all
		   && insn.insn_nios2_opcode->pinfo & NIOS2_INSN_CALL)
	    output_call ();
	  else if (insn.insn_nios2_opcode->pinfo & NIOS2_INSN_ANDI)
	    output_andi ();
	  else if (insn.insn_nios2_opcode->pinfo & NIOS2_INSN_ORI)
	    output_ori ();
	  else if (insn.insn_nios2_opcode->pinfo & NIOS2_INSN_XORI)
	    output_xori ();
	  else if (insn.insn_nios2_opcode->pinfo & NIOS2_INSN_ADDI)
	    output_addi ();
	  else if (saved_pinfo == NIOS2_INSN_MACRO_MOVIA)
	    output_movia ();
	  else
	    output_insn ();
	}
      else
	{
	  /* the assembler is broken */
	  fprintf (stderr,
		   _("internal error: %s is not a valid argument syntax\n"),
		   insn.insn_nios2_opcode->args);
	  /* Probably a memory allocation problem?  Give up now.  */
	  as_fatal (_("Broken assembler.  No assembly attempted."));
	}
    }
  else
    {
      /* unrecognised instruction - error */
      as_bad (_("unrecognised instruction %s"), insn.insn_tokens[0]);
    }
}

/* output a normal instruction */
static void
output_insn ()
{
  char *f;
  nios2_insn_relocS *reloc;

  f = frag_more (4);
  /* this allocates enough space for the instruction
     and puts it in the current frag */
  md_number_to_chars (f, insn.insn_code, 4);
  /* emit debug info */
  dwarf2_emit_insn (4);
  /* create any fixups */
  reloc = insn.insn_reloc;
  while (reloc != NULL)
    {
      /* this creates any fixups to be acted on later */
      fix_new_exp (frag_now, f - frag_now->fr_literal, 4,
		   &reloc->reloc_expression, reloc->reloc_pcrel,
		   reloc->reloc_type);
      reloc = reloc->reloc_next;
    }
}

/* output an unconditional branch */
static void
output_ubranch ()
{
  char *f;
  nios2_insn_relocS *reloc;
  symbolS *symp;
  offsetT offset;

  reloc = insn.insn_reloc;

  /* if the reloc is NULL, there was an error assembling the branch */
  if (reloc != NULL)
    {

      symp = reloc->reloc_expression.X_add_symbol;
      offset = reloc->reloc_expression.X_add_number;

      /* we must tag debug info here since we can't do it after
         calling frag_var */
      dwarf2_emit_insn (4);

      /* we create a machine dependent frag which can grow
         to accommodate the largest possible instruction sequence
         this may generate */
      f = frag_var (rs_machine_dependent,
		    RELAX_MAX_SIZE (UBRANCH),
		    RELAX_SIZE (UBRANCH),
		    RELAX_SUBSTATE (UBRANCH), symp, offset, NULL);

      md_number_to_chars (f, insn.insn_code, 4);

      /* we leave fixup generation to md_convert_frag */
    }
}

/* output a conditional branch */
static void
output_cbranch ()
{
  char *f;
  nios2_insn_relocS *reloc;
  symbolS *symp;
  offsetT offset;

  reloc = insn.insn_reloc;

  /* if the reloc is NULL, there was an error assembling the branch */
  if (reloc != NULL)
    {

      symp = reloc->reloc_expression.X_add_symbol;
      offset = reloc->reloc_expression.X_add_number;

      /* we must tag debug info here since we can't do it after
         calling frag_var */
      dwarf2_emit_insn (4);

      /* we create a machine dependent frag which can grow
         to accommodate the largest possible instruction sequence
         this may generate */
      f = frag_var (rs_machine_dependent,
		    RELAX_MAX_SIZE (CBRANCH),
		    RELAX_SIZE (CBRANCH),
		    RELAX_SUBSTATE (CBRANCH), symp, offset, NULL);

      md_number_to_chars (f, insn.insn_code, 4);


      /* we leave fixup generation to md_convert_frag */
    }
}

/* Output a call sequence. Since calls are not pc-relative for NIOS2,
   but are page-relative, we cannot tell at any stage in assembly
   whether a call will be out of range since a section may be linked
   at any address. So if we are relaxing, we convert all call instructions
   to long call sequences, and rely on the linker to relax them back to
   short calls */
static void
output_call ()
{
  char *f;
  nios2_insn_relocS *reloc;
  f = frag_more (12);
  /* this allocates enough space for the instruction
     and puts it in the current frag */
  reloc = insn.insn_reloc;

  /* if the reloc is NULL, there was an error assembling the branch */
  if (reloc != NULL)
    {
      md_number_to_chars (f, OP_MATCH_ORHI | 0x00400000, 4);
      dwarf2_emit_insn (4);
      md_number_to_chars (f + 4, OP_MATCH_ORI | 0x08400000, 4);
      dwarf2_emit_insn (4);
      md_number_to_chars (f + 8, OP_MATCH_CALLR | 0x08000000, 4);
      dwarf2_emit_insn (4);
      fix_new (frag_now, f - frag_now->fr_literal, 4,
	       reloc->reloc_expression.X_add_symbol,
	       reloc->reloc_expression.X_add_number, 0,
	       BFD_RELOC_NIOS2_CALLR);


    }
}


static int
can_evaluate_expr ()
{
  /* remove this check for null and the invalid insn "ori r9, 1234" seg faults */
  if (!insn.insn_reloc) 
    {
      /* ??? Ideally we should do something other than as_fatal here as we can continue to assemble.
             However this function (actually the output_* functions) should not have been called 
             in the first place once an illegal instruction had been encountered */
      as_fatal (_("Invalid instruction encountered, cannot recover. No assembly attempted."));
    }

  if (insn.insn_reloc->reloc_expression.X_op == O_constant)
    return 1;

  return 0;
}

static int
get_expr_value ()
{
  int value = 0;
  if (insn.insn_reloc->reloc_expression.X_op == O_constant)
    value = insn.insn_reloc->reloc_expression.X_add_number;
  return value;
}

/* output an addi - will silently convert to
 * orhi if rA = r0 and (expr & 0xffff0000) == 0 */
static void
output_addi ()
{
  int expr_val = 0;
  if (can_evaluate_expr ())
    {
      expr_val = get_expr_value ();
      if (GET_INSN_FIELD (RRS, insn.insn_code) == 0 &&
	  (expr_val & 0xffff) == 0 && expr_val != 0)
	{

	  /* we really want a movhi (orhi) here */
	  insn.insn_code = (insn.insn_code & ~OP_MATCH_ADDI) | OP_MATCH_ORHI;
	  insn.insn_reloc->reloc_expression.X_add_number =
	    (insn.insn_reloc->reloc_expression.X_add_number >> 16) & 0xffff;
	  insn.insn_reloc->reloc_type = BFD_RELOC_NIOS2_U16;
	}
    }

  /* output an instruction */
  output_insn ();
}

static void
output_andi ()
{
  int expr_val = 0;
  if (can_evaluate_expr ())
    {
      expr_val = get_expr_value ();
      if (expr_val != 0 && (expr_val & 0xffff) == 0)
	{
	  /* we really want a movhi (orhi) here */
	  insn.insn_code = (insn.insn_code & ~OP_MATCH_ANDI) | OP_MATCH_ANDHI;
	  insn.insn_reloc->reloc_expression.X_add_number =
	    (insn.insn_reloc->reloc_expression.X_add_number >> 16) & 0xffff;
	  insn.insn_reloc->reloc_type = BFD_RELOC_NIOS2_U16;
	}
    }

  /* output an instruction */
  output_insn ();
}

static void
output_ori ()
{
  int expr_val = 0;
  if (can_evaluate_expr ())
    {
      expr_val = get_expr_value ();
      if (expr_val != 0 && (expr_val & 0xffff) == 0)
	{
	  /* we really want a movhi (orhi) here */
	  insn.insn_code = (insn.insn_code & ~OP_MATCH_ORI) | OP_MATCH_ORHI;
	  insn.insn_reloc->reloc_expression.X_add_number =
	    (insn.insn_reloc->reloc_expression.X_add_number >> 16) & 0xffff;
	  insn.insn_reloc->reloc_type = BFD_RELOC_NIOS2_U16;
	}
    }

  /* output an instruction */
  output_insn ();
}

static void
output_xori ()
{
  int expr_val = 0;
  if (can_evaluate_expr ())
    {
      expr_val = get_expr_value ();
      if (expr_val != 0 && (expr_val & 0xffff) == 0)
	{
	  /* we really want a movhi (orhi) here */
	  insn.insn_code = (insn.insn_code & ~OP_MATCH_XORI) | OP_MATCH_XORHI;
	  insn.insn_reloc->reloc_expression.X_add_number =
	    (insn.insn_reloc->reloc_expression.X_add_number >> 16) & 0xffff;
	  insn.insn_reloc->reloc_type = BFD_RELOC_NIOS2_U16;
	}
    }

  /* output an instruction */
  output_insn ();
}


/* output a movhi/addi pair for the movia pseudo-op */
static void
output_movia ()
{
  char *f;
  nios2_insn_relocS *reloc;
  f = frag_more (8);
  unsigned long reg_index = GET_INSN_FIELD (IRT, insn.insn_code);

  /* this allocates enough space for the instruction
     and puts it in the current frag */
  reloc = insn.insn_reloc;

  /* if the reloc is NULL, there was an error assembling the movia */
  if (reloc != NULL)
    {
      md_number_to_chars (f, insn.insn_code, 4);
      dwarf2_emit_insn (4);
      md_number_to_chars (f + 4,
			  OP_MATCH_ADDI | (reg_index << OP_SH_IRT) |
			  (reg_index << OP_SH_IRS), 4);
      dwarf2_emit_insn (4);
      fix_new (frag_now, f - frag_now->fr_literal, 4,
	       reloc->reloc_expression.X_add_symbol,
	       reloc->reloc_expression.X_add_number, 0,
	       BFD_RELOC_NIOS2_HIADJ16);
      fix_new (frag_now, f + 4 - frag_now->fr_literal, 4,
	       reloc->reloc_expression.X_add_symbol,
	       reloc->reloc_expression.X_add_number, 0, BFD_RELOC_NIOS2_LO16);

    }
}

/*
	Function md_chars_to_number takes the sequence of
	bytes in buf and returns the corresponding value
	in an int. n must be 1, 2 or 4.
 */
valueT
md_chars_to_number (char *buf, int n)
{
  int i;
  valueT val;

  assert (n == 1 || n == 2 || n == 4);

  val = 0;
  if (target_big_endian)
  {
    for (i = 0; i < n; ++i)
      {
        val = val | ((buf[i] & 0xff) << 8 * (n - (i + 1)));
      }
  }
  else
  {
    for (i = 0; i < n; ++i)
      {
        val = val | ((buf[i] & 0xff) << 8 * i);
      }
  }
  return val;
}


/*
	Function : void md_number_to_chars(char *buf, valueT val, int n)

	Description : this function turns a C long int, short int or char
	              into the series of bytes that represent the number
				  on the target machine
 */
void
md_number_to_chars (char *buf, valueT val, int n)
{
  assert (n == 1 || n == 2 || n == 4);
  if (target_big_endian)
    {
      number_to_chars_bigendian (buf, val, n);
    }
  else
    {
      number_to_chars_littleendian (buf, val, n);
    }
}

/*
	Function : void md_number_to_imm(char *buf, valueT val, int n)

	Description : this function is identical to md_number_to_chars
 */
void
md_number_to_imm (char *buf, valueT val, int n)
{
  md_number_to_chars (buf, val, n);
}

/*
	Function : void md_number_to_disp(char *buf, valueT val, int n)

	Description : this function is identical to md_number_to_chars
 */
void
md_number_to_disp (char *buf, valueT val, int n)
{
  md_number_to_chars (buf, val, n);
}

/*
	Function : void md_number_to_field(char *buf, valueT val, int n)

	Description : this function is identical to md_number_to_chars
 */
void
md_number_to_field (char *buf, valueT val, int n)
{
  md_number_to_chars (buf, val, n);
}

/*

   Function : char * md_atof(int type, char *litP,int *sizeP)

   Description :
   Turn a string in input_line_pointer into a floating point constant
   of type TYPE, and store the appropriate bytes in *LITP.  The number
   of LITTLENUMS emitted is stored in *SIZEP.  An error message is
   returned, or NULL on OK.

 */

char *
md_atof (int type, char *litP, int *sizeP)
{
  int prec;
  LITTLENUM_TYPE words[4];
  char *t;
  int i;

  switch (type)
    {
    case 'f':
      prec = 2;
      break;
    case 'd':
      prec = 4;
      break;
    default:
      *sizeP = 0;
      return _("bad call to md_atof");
    }

  t = atof_ieee (input_line_pointer, type, words);
  if (t)
    input_line_pointer = t;

  *sizeP = prec * 2;

  if (! target_big_endian)
    {
      for (i = prec - 1; i >= 0; i--)
	{
	  md_number_to_chars (litP, (valueT) words[i], 2);
	  litP += 2;
	}
    }
  else
    {
      for (i = 0; i < prec; i++)
	{
	  md_number_to_chars (litP, (valueT) words[i], 2);
	  litP += 2;
	}
    }

  return NULL;
}



int md_short_jump_size;
int md_long_jump_size;

void
md_create_short_jump (char *result_ptr ATTRIBUTE_UNUSED,
		      addressT from_addr ATTRIBUTE_UNUSED,
		      addressT to_addr ATTRIBUTE_UNUSED,
		      fragS * frag ATTRIBUTE_UNUSED,
		      symbolS * to_symbol ATTRIBUTE_UNUSED)
{
  abort ();
}

void
md_create_long_jump (char *ptr ATTRIBUTE_UNUSED,
		     addressT from_addr ATTRIBUTE_UNUSED,
		     addressT to_addr ATTRIBUTE_UNUSED,
		     fragS * frag ATTRIBUTE_UNUSED,
		     symbolS * to_symbol ATTRIBUTE_UNUSED)
{
  abort ();
}

int
md_estimate_size_before_relax (fragS * fragp, segT segment ATTRIBUTE_UNUSED)
{
  /* we only support ELF targets */

  switch (nios2_as_options.relax)
    {
    case relax_none:
    case relax_section:
      break;
    case relax_all:
      /* The NIOS2 linker performs relaxation so the assembler
         always assumes the worst case, so that the linker can
         replace with a better case if possible - this way, linker
         relaxation can never cause a short branch to be out of range
       */
      while (nios2_relax_table[fragp->fr_subtype].rlx_more != 0)
	fragp->fr_subtype = nios2_relax_table[fragp->fr_subtype].rlx_more;
      break;
    default:
      abort ();
      break;
    }

  /* return the estimated size of the frag */
  return nios2_relax_table[fragp->fr_subtype].rlx_length;
}


void
md_convert_frag (bfd * headers ATTRIBUTE_UNUSED, segT seg ATTRIBUTE_UNUSED,
		 fragS * fragp)
{
  unsigned char *buffer = fragp->fr_literal + fragp->fr_fix;
  relax_substateT subtype = fragp->fr_subtype;
  unsigned int growth = RELAX_SIZE (subtype);
  unsigned int br_opcode, br_op_a, br_op_b;

  switch (subtype)
    {
    case UBRANCH:
      /* we just need to generate the fixup for the symbol and offset */
      fix_new (fragp, fragp->fr_fix, 4, fragp->fr_symbol, fragp->fr_offset, 1,
	       BFD_RELOC_16_PCREL);
      break;
    case UJMP:
      /* replace ubranch at fr_fix with :
         movhi at, %hi(symbol+offset)
         ori at, %lo(symbol+offset)
         jmp at
       */
      md_number_to_chars (buffer, OP_MATCH_ORHI | 0x00400000, 4);
      md_number_to_chars (buffer + 4, OP_MATCH_ORI | 0x08400000, 4);
      md_number_to_chars (buffer + 8, OP_MATCH_JMP | 0x08000000, 4);
      fix_new (fragp, fragp->fr_fix, 4, fragp->fr_symbol, fragp->fr_offset, 0,
	       BFD_RELOC_NIOS2_UJMP);
      break;
    case CBRANCH:
      /* we just need to generate the fixup for the symbol and offset */
      fix_new (fragp, fragp->fr_fix, 4, fragp->fr_symbol, fragp->fr_offset, 1,
	       BFD_RELOC_16_PCREL);
      break;
    case CJMP:
      /* replace cbranch at fr_fix with :
         b(opposite condition) r, s, skip
         movhi at, %hi(symbol+offset)
         ori at, %lo(symbol+offset)
         jmp at
         skip:
         ...
       */
      br_opcode = md_chars_to_number (buffer, 4);

      switch (br_opcode & OP_MASK_OP)
	{
	case OP_MATCH_BEQ:
	  br_opcode =
	    (br_opcode & ~OP_MASK_OP) | OP_MATCH_BNE | (12 << OP_SH_IMM16);
	  break;
	case OP_MATCH_BNE:
	  br_opcode =
	    (br_opcode & ~OP_MASK_OP) | OP_MATCH_BEQ | (12 << OP_SH_IMM16);
	  break;
	case OP_MATCH_BGE:
	case OP_MATCH_BGEU:
	case OP_MATCH_BLT:
	case OP_MATCH_BLTU:
	  /* swap the operands */
	  br_op_a = (br_opcode & OP_MASK_RRT) << 5;
	  br_op_b = (br_opcode & OP_MASK_RRS) >> 5;
	  br_opcode =
	    (br_opcode & ~(OP_MASK_RRS | OP_MASK_RRT)) | br_op_a | br_op_b |
	    (12 << OP_SH_IMM16);
	  break;
	default:
	  as_bad_where (fragp->fr_file, fragp->fr_line,
			_("expecting conditional branch for relaxation\n"));
	  abort ();
	}

      md_number_to_chars (buffer, br_opcode, 4);
      md_number_to_chars (buffer + 4, OP_MATCH_ORHI | 0x00400000, 4);
      md_number_to_chars (buffer + 8, OP_MATCH_ORI | 0x08400000, 4);
      md_number_to_chars (buffer + 12, OP_MATCH_JMP | 0x08000000, 4);
      fix_new (fragp, fragp->fr_fix + 4, 4, fragp->fr_symbol,
	       fragp->fr_offset, 0, BFD_RELOC_NIOS2_CJMP);
      break;
    default:
      as_bad_where (fragp->fr_file, fragp->fr_line,
		    _("can't relax instruction\n"));
      abort ();
      break;
    }

  fragp->fr_fix += growth;
}


/* round up section size */
valueT
md_section_align (asection * seg ATTRIBUTE_UNUSED, valueT size)
{
  /* I think byte alignment is fine here */
  return size;
}


int
nios2_force_relocation (fixS * fixp)
{
  if (fixp->fx_r_type == BFD_RELOC_VTABLE_INHERIT
      || fixp->fx_r_type == BFD_RELOC_VTABLE_ENTRY
      || fixp->fx_r_type == BFD_RELOC_NIOS2_ALIGN)
    return 1;

  return 0;
}

/* nios2_fix_adjustable is called to see whether a reloc against a defined symbol
   should be converted into a reloc against a section. */

int
nios2_fix_adjustable (fixS * fixp)
{
#ifdef OBJ_ELF
  /* Prevent all adjustments to global symbols.  */
  if (OUTPUT_FLAVOR == bfd_target_elf_flavour
      && (S_IS_EXTERN (fixp->fx_addsy) || S_IS_WEAK (fixp->fx_addsy)))
    return 0;
#endif
  if (fixp->fx_r_type == BFD_RELOC_VTABLE_INHERIT
      || fixp->fx_r_type == BFD_RELOC_VTABLE_ENTRY)
    return 0;

  return 1;
}

/*
	nios2_frob_symbol is called in adjust_reloc_syms through the macro
	tc_frob_symbol - it is used to remove *ABS* references from the
	symbol table
 */
int
nios2_frob_symbol (symbolS * symp)
{
  if ((OUTPUT_FLAVOR == bfd_target_elf_flavour
       && (symp) == section_symbol (absolute_section))
      || !S_IS_DEFINED (symp))
    return 1;
  else
    return 0;
}

/*
  The function tc_gen_reloc creates a relocation structure for the
  fixup fixp, and returns a pointer to it. This structure is passed
  to bfd_install_relocation so that it can be written to the object
  file for linking
*/
arelent *
tc_gen_reloc (asection * section ATTRIBUTE_UNUSED, fixS * fixp)
{
  arelent *reloc;
  reloc = (arelent *) xmalloc (sizeof (arelent));
  reloc->sym_ptr_ptr = (asymbol **) xmalloc (sizeof (asymbol *));
  *reloc->sym_ptr_ptr = symbol_get_bfdsym (fixp->fx_addsy);

  reloc->address = fixp->fx_frag->fr_address + fixp->fx_where;
  reloc->addend = fixp->fx_addnumber;
  reloc->howto = bfd_reloc_type_lookup (stdoutput, fixp->fx_r_type);
  if (reloc->howto == NULL)
    {
      as_bad_where (fixp->fx_file, fixp->fx_line,
		    _("can't represent relocation type %s"),
		    bfd_get_reloc_code_name (fixp->fx_r_type));

      /* Set howto to a garbage value so that we can keep going.  */
      reloc->howto = bfd_reloc_type_lookup (stdoutput, BFD_RELOC_32);
      assert (reloc->howto != NULL);
    }
  return reloc;
}

long
md_pcrel_from (fixS * fixP ATTRIBUTE_UNUSED)
{
  return 0;
}


/* Apply a fixup to the object file.  */
void
md_apply_fix3 (fixS * fixP, valueT * valP, segT seg ATTRIBUTE_UNUSED)
{
  const struct nios2_opcode *opcode;
  enum overflow_type overflow_msg_type;
  bfd_boolean overflowed = FALSE;
  valueT fixup = 0;

  /* assert that the fixup is one we can handle */
  assert (fixP != NULL && valP != NULL &&
	  (fixP->fx_r_type == BFD_RELOC_8 ||
	   fixP->fx_r_type == BFD_RELOC_16 ||
	   fixP->fx_r_type == BFD_RELOC_32 ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_S16 ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_U16 ||
	   fixP->fx_r_type == BFD_RELOC_16_PCREL ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_CALL26 ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_IMM5 ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_CACHE_OPX ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_IMM6 ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_IMM8 ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_HI16 ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_LO16 ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_HIADJ16 ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_GPREL ||
	   fixP->fx_r_type == BFD_RELOC_VTABLE_INHERIT ||
	   fixP->fx_r_type == BFD_RELOC_VTABLE_ENTRY ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_UJMP ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_CJMP ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_CALLR ||
	   fixP->fx_r_type == BFD_RELOC_NIOS2_ALIGN
	   // add other relocs here as we generate them
	  ));


  /* The value passed in valP can be the value of a fully
     resolved expression, or it can be the value of a partially
     resolved expression. In the former case, both fixP->fx_addsy
     and fixP->fx_subsy are NULL, and fixP->fx_offset == *valP, and
     we can fix up the instruction that fixP relates to.
     In the latter case, one or both of fixP->fx_addsy and
     fixP->fx_subsy are not NULL, and fixP->fx_offset may or may not
     equal *valP. We don't need to check for fixP->fx_subsy being null
     because the generic part of the assembler generates an error if
     it is not an absolute symbol */

  if (fixP->fx_addsy != NULL)
    {
      fixP->fx_addnumber = fixP->fx_offset;
      fixP->fx_done = 0;
    }
  else
    {
      valueT value;

      char *buf;
      reloc_howto_type *howto;
      howto = bfd_reloc_type_lookup (stdoutput, fixP->fx_r_type);

      if (howto == NULL)
	{
	  as_bad_where (fixP->fx_file, fixP->fx_line,
			_("relocation is not supported"));
	}
      else
	{
	  fixup += *valP;

	  /* If this is a pc-relative relocation, we need to
	     subtract the current offset within the object file
	     FIXME : for some reason fixP->fx_pcrel isn't 1 when it should be
	     so I'm using the howto structure instead to determine this */
	  if (howto->pc_relative == 1)
	    fixup = fixup - (fixP->fx_frag->fr_address + fixP->fx_where + 4);



	  /* Get the instruction to be fixed up */
	  buf = fixP->fx_frag->fr_literal + fixP->fx_where;
	  value = md_chars_to_number (buf, 4);

	  /* What opcode is the instruction? This will determine
	     whether we check for overflow in immediate values
	     and what error message we get */
	  opcode = nios2_find_opcode_hash (value);
	  overflow_msg_type = opcode->overflow_msg;

	  overflowed = nios2_check_overflow (fixup, howto);


	  if (overflowed)
	    {
	      unsigned int range_min;
	      unsigned int range_max;
	      unsigned int address;
	      switch (overflow_msg_type)
		{
		case call_target_overflow:
		  range_min =
		    ((fixP->fx_frag->fr_address +
		      fixP->fx_where) & 0xf0000000);
		  range_max = range_min + 0x0fffffff;
		  address = fixup | range_min;

		  as_bad_where (fixP->fx_file, fixP->fx_line,
				_(overflow_msgs[call_target_overflow]),
				address, range_min, range_max);
		  break;
		case branch_target_overflow:
		  as_bad_where (fixP->fx_file, fixP->fx_line,
				_(overflow_msgs[branch_target_overflow]),
				fixup, BYTE_B, BYTE_F);
		  break;
		case address_offset_overflow:
		  as_bad_where (fixP->fx_file, fixP->fx_line,
				_(overflow_msgs[address_offset_overflow]),
				opcode->name, fixup, -32768, 32767);
		  break;
		case signed_immed16_overflow:
		  as_bad_where (fixP->fx_file, fixP->fx_line,
				_(overflow_msgs[signed_immed16_overflow]),
				fixup, -32768, 32767);
		  break;
		case unsigned_immed16_overflow:
		  as_bad_where (fixP->fx_file, fixP->fx_line,
				_(overflow_msgs[unsigned_immed16_overflow]),
				fixup, 0, 65535);
		  break;
		case unsigned_immed5_overflow:
		  as_bad_where (fixP->fx_file, fixP->fx_line,
				_(overflow_msgs[unsigned_immed5_overflow]),
				fixup, 0, 31);
		  break;
		case custom_opcode_overflow:
		  as_bad_where (fixP->fx_file, fixP->fx_line,
				_(overflow_msgs[custom_opcode_overflow]),
				fixup, 0, 255);
		  break;
		default:
		  as_bad_where (fixP->fx_file, fixP->fx_line,
				_
				("unspecified overflow in immediate argument"));
		  break;
		}
	    }


	  /* apply the rightshift */
	  (signed) fixup >>= howto->rightshift;

	  /* truncate the fixup to right size */
	  switch (fixP->fx_r_type)
	    {
	    case BFD_RELOC_NIOS2_HI16:
	      fixup = (fixup >> 16) & 0xFFFF;
	      break;
	    case BFD_RELOC_NIOS2_LO16:
	      fixup = fixup & 0xFFFF;
	      break;
	    case BFD_RELOC_NIOS2_HIADJ16:
	      fixup = ((fixup >> 16) & 0xFFFF) + ((fixup >> 15) & 0x01);
	      break;
	    default:
	      fixup =
		(fixup << (32 - howto->bitsize)) >> (32 - howto->bitsize);
	      break;
	    }

	  /* fixup the instruction */
	  value = (value & ~howto->dst_mask) | (fixup << howto->bitpos);
	  md_number_to_chars (buf, value, 4);
	}

      fixP->fx_done = 1;
    }

  if (fixP->fx_r_type == BFD_RELOC_VTABLE_INHERIT)
    {
      fixP->fx_done = 0;
      if (fixP->fx_addsy
	  && !S_IS_DEFINED (fixP->fx_addsy) && !S_IS_WEAK (fixP->fx_addsy))
	S_SET_WEAK (fixP->fx_addsy);
    }
  else if (fixP->fx_r_type == BFD_RELOC_VTABLE_ENTRY)
    {
      fixP->fx_done = 0;
    }
}

bfd_boolean
nios2_check_overflow (valueT fixup, reloc_howto_type * howto)
{
  /* apply the rightshift before checking for overflow */
  (signed) fixup >>= howto->rightshift;

  /* check for overflow - return TRUE if overflow, FALSE if not */
  switch (howto->complain_on_overflow)
    {
    case complain_overflow_dont:
      break;
    case complain_overflow_bitfield:
      if ((fixup >> howto->bitsize) != 0)
	return TRUE;
      break;
    case complain_overflow_signed:
      if ((fixup & 0x80000000) > 0)
	{
	  /* check for negative overflow */
	  if ((signed) fixup < ((signed) 0x80000000 >> howto->bitsize))
	    return TRUE;
	}
      else
	{
	  /* check for positive overflow */
	  if (fixup >= ((unsigned) 1 << (howto->bitsize - 1)))
	    return TRUE;
	}
      break;
    case complain_overflow_unsigned:
      if ((fixup >> howto->bitsize) != 0)
	return TRUE;
      break;
    default:
      as_bad (_("error checking for overflow - broken assembler"));
      break;
    }

  return FALSE;
}

/*
   Function : void md_end()
   Description : Called just before the assembler exits
 */
void
md_end ()
{
  /* FIXME - not yet implemented */
}


/*
	Creates a new nios2_insn_relocS and returns a pointer to it
*/
nios2_insn_relocS *
nios2_insn_reloc_new (bfd_reloc_code_real_type reloc_type, unsigned int pcrel)
{
  nios2_insn_relocS *retval;
  retval = (nios2_insn_relocS *) malloc (sizeof (nios2_insn_relocS));
  if (retval == NULL)
    {
      as_bad (_("can't create relocation"));
      abort ();
    }

  /* fill out the fields with default values */
  retval->reloc_next = NULL;
  retval->reloc_type = reloc_type;
  retval->reloc_pcrel = pcrel;
  return retval;
}

/*
	Frees up memory previously allocated by nios2_insn_reloc_new()
  */
void
nios2_insn_reloc_destroy (nios2_insn_relocS * reloc)
{
  assert (reloc != NULL);
  free (reloc);
}

/*
   Function : nios2_assemble_expression(char* exprstr)

   Description : The various nios2_assemble_* functions call this
                 function to generate an expression from a
				 string representing an expression
				 It then tries to evaluate the expression, and
				 if it can, returns its value.
				 If not, it creates a new nios2_insn_relocS
				 and stores the expression and reloc_type
				 for future use
 */
unsigned long
nios2_assemble_expression (const char *exprstr,
			   nios2_insn_infoS * insn,
			   nios2_insn_relocS * prev_reloc,
			   bfd_reloc_code_real_type reloc_type,
			   unsigned int pcrel)
{
  nios2_insn_relocS *reloc;
  char *saved_line_ptr;
  unsigned short value;
  int i;

  assert (exprstr != NULL);
  assert (insn != NULL);

  /* check for %gprel, %hi, %lo or %hiadj
     change the relocation type
     and advance the ptr to the start of
     the expression proper */
  for (i = 0; i < nios2_num_special_relocs; i++)
    {
      if (strstr (exprstr, nios2_special_reloc[i].string) != NULL)
	{
	  reloc_type = nios2_special_reloc[i].reloc_type;
	  exprstr += strlen (nios2_special_reloc[i].string) + 1;
	  break;
	}
    }

  /* we potentially have a relocation */
  reloc = nios2_insn_reloc_new (reloc_type, pcrel);
  if (prev_reloc != NULL)
    prev_reloc->reloc_next = reloc;
  else
    insn->insn_reloc = reloc;

  /* parse the expression string */
  saved_line_ptr = input_line_pointer;
  input_line_pointer = (char *) exprstr;
  expression (&reloc->reloc_expression);
  input_line_pointer = saved_line_ptr;

  /* this is redundant as the fixup will put this into
     the instruction, but it is included here so that
     self-test mode (-r) works */
  value = 0;
  if (nios2_mode == NIOS2_MODE_TEST)
    {
      if (reloc->reloc_expression.X_op == O_constant)
	value = reloc->reloc_expression.X_add_number;
    }

  return (unsigned long) value;
}

/*
	The function consume_separate takes a pointer into a string
	of instruction tokens (args) and a pointer into a string representing
	the expected sequence of tokens and separators. It finds the first
	instance of the character pointed to by separator in argStr, and
	returns a pointer to the next element of argStr, which is the
	following token in the sequence.
  */
char *
nios2_consume_separator (char *argStr, const char *separator)
{
  char *argPtr;

  /* if we have a opcode reg, expr(reg) type instruction, and
   * we are separating the expr from the (reg), we find the last
   * (, just in case the expression has brackets */

  if (*separator == '(')
    argPtr = strrchr (argStr, *separator);
  else
    argPtr = strchr (argStr, *separator);

  if (argPtr != NULL)
    *argPtr++ = 0;
  else
    as_bad (_("expecting %c near %s"), *separator, argStr);
  return argPtr;
}

/*
	The function consume_arg takes a pointer into a string
	of instruction tokens (args) and a pointer into a string
	representing the expected sequence of tokens and separators.
	It checks whether the first argument in argStr is of the
	expected type, throwing an error if it is not, and returns
	the pointer argStr.
  */
char *
nios2_consume_arg (char *argStr, const char *argType)
{
  char *temp;
  int regno = -1;
  
  switch (*argType)
    {
    case 'c':
      if (strncmp (argStr, "ctl", strlen ("ctl")) != 0
	  && strncmp (argStr, "cpuid", strlen ("cpuid")) != 0
	  && strncmp (argStr, "status", strlen ("status")) != 0
	  && strncmp (argStr, "estatus", strlen ("estatus")) != 0
	  && strncmp (argStr, "bstatus", strlen ("bstatus")) != 0
	  && strncmp (argStr, "ienable", strlen ("ienable")) != 0
	  && strncmp (argStr, "ipending", strlen ("ipending")) != 0)
	{
	  as_bad (_("expecting control register"));
	}
      break;
    case 'd':
    case 's':
    case 't':

      /* we check to make sure we don't have a control register */
      if (strncmp (argStr, "ctl", strlen ("ctl")) == 0
	  || strncmp (argStr, "cpuid", strlen ("cpuid")) == 0
	  || strncmp (argStr, "status", strlen ("status")) == 0
	  || strncmp (argStr, "estatus", strlen ("estatus")) == 0
	  || strncmp (argStr, "bstatus", strlen ("bstatus")) == 0
	  || strncmp (argStr, "ienable", strlen ("ienable")) == 0
	  || strncmp (argStr, "ipending", strlen ("ipending")) == 0)
	{
	  as_bad (_("illegal use of control register"));
	}

      /* and whether coprocessor registers are valid here */
      if (nios2_coproc_reg (argStr)
	  && insn.insn_nios2_opcode->match != OP_MATCH_CUSTOM)
	{
	  as_bad (_("illegal use of coprocessor register\n"));
	}


      /* extract a register number if the register is of the 
         form r[0-9]+, if it is a normal register, set
         regno to its number (0-31), else set regno to -1 */
      if (argStr[0] == 'r' && ISDIGIT (argStr[1]))
        {
	  char *p = argStr;
	  
	  ++p;
	  regno = 0;
	  do
	    {
	      regno *= 10;
	      regno += *p - '0';
	      ++p;
	    }
	  while (ISDIGIT (*p));
	}
      else
        {
	  regno = -1;
	}

      /* and whether we are using at */
      if (!nios2_as_options.noat
	  && (regno == 1
	      || strncmp (argStr, "at", strlen ("at")) == 0))
	{
	  as_warn (_("Register at (r1) can sometimes be corrupted by assembler optimizations.\n"
	             "Use .set noat to turn off those optimizations (and this warning)."));
	}
	
      /* and whether we are using oci registers */
      if (!nios2_as_options.nobreak
	  && (regno == 25
	      || strncmp (argStr, "bt", strlen ("bt")) == 0))
	{
	  as_warn (_("The debugger will corrupt bt (r25). If you don't need to debug this\n"
	             "code then use .set nobreak to turn off this warning."));
	}
	
      if (!nios2_as_options.nobreak
	  && (regno == 30
	      || strncmp (argStr, "ba", strlen ("ba")) == 0))
	{
	  as_warn (_("The debugger will corrupt ba (r30). If you don't need to debug this\n"
	             "code then use .set nobreak to turn off this warning."));
	}
      break;
    case 'i':
    case 'u':
      if (*argStr == '%')
	{
	  if (strstr (argStr, "%hi(") || strstr (argStr, "%lo(")
	      || strstr (argStr, "%hiadj(") || strstr (argStr, "%gprel("))
	    {
	      // we zap the brackets because we don't want them confused with separators
	      temp = strchr (argStr, '(');
	      if (temp != NULL)
		*temp = ' ';
	      temp = strchr (argStr, ')');
	      if (temp != NULL)
		*temp = ' ';
	    }
	  else
	    {
	      as_bad (_("badly formed expression near %s"), argStr);
	    }
	}
      break;
    case 'm':
    case 'j':
    case 'k':
    case 'l':
    case 'b':
      /* we can't have %hi, %lo or %hiadj here */
      if (*argStr == '%')
	as_bad (_("badly formed expression near %s"), argStr);
      break;
    default:
      break;
    }

#if 0    
/* ??? SPR:173865 This is actually supported by the HW but the documentation is a bit funny. 
   The compiler really want the extra register, so let it have it! */
  /* checks for jmp 31 */
  /* TODO: move test that insn is jmp to somewhere better.*/
  if ((strncmp (insn.insn_nios2_opcode->name, "jmp", strlen ("jmp")) == 0) 
      && (regno == 31
	  || strncmp (argStr, "ra", strlen ("ra")) == 0))
    {
      as_bad (_("It is illegal to jump to the address contained in register ra (r31). "
		"To return from subroutines called by call or callr, use ret instead of jmp."));
    }
#endif

  return argStr;
}

/*
	The principal argument parsing function which takes a string
	representing the instruction arguments, and extracts the argument
	tokens
  */
void
nios2_parse_args (char *argStr, const char *parseStr, char **parsedArgs)
{
  char *p;
  char *end = NULL;
  int i;
  p = argStr;
  i = 0;
  bfd_boolean terminate = FALSE;
  
  /* This rest of this function is it too fragile and it mostly works,
     therefore special case this one */
  if (*parseStr == 0 && argStr != 0)
    {
      as_bad (_("too many arguments"));
      parsedArgs[0] = NULL;
      return;
    }
  

  while (p != NULL && !terminate && i < NIOS2_MAX_INSN_TOKENS)
    {
      parsedArgs[i] = nios2_consume_arg (p, parseStr);
      ++parseStr;
      if (*parseStr != '\0')
	{
	  p = nios2_consume_separator (p, parseStr);
	  ++parseStr;
	}
      else
	{
	  /* check that the argument string has no trailing arguments */
	  /* if we've got a %lo etc relocation, we've zapped the brackets with spaces */
	  if (strstr (p, "%lo") == p || strstr (p, "%hi") == p
	      || strstr (p, "%hiadj") == p || strstr (p, "%gprel") == p)
	    end = strpbrk (p, ",");
	  else
	    end = strpbrk (p, " ,");

	  if (end != NULL)
	    as_bad (_("too many arguments"));
	}

      if (*parseStr == '\0' || (p != NULL && *p == '\0'))
	{
	  terminate = TRUE;
	}
      ++i;
    }
  parsedArgs[i] = NULL;

  //This extremely messy bit of code allows for the optional vinc argument in vm, and vms type instructions

  if (*parseStr != '\0' && i==2 && (!strcmp(insn.insn_nios2_opcode->args, "r,b,i") || !strcmp(insn.insn_nios2_opcode->args, "r,b,s,i")))
    as_warn (_("vinc argument left blank, assuming vinc0"));
  else if (*parseStr != '\0' && insn.insn_nios2_opcode->match != OP_MATCH_BREAK)
    as_bad (_("missing argument"));

}


/* checks whether the register name is a coprocessor
   register - returns TRUE if it is, FALSE otherwise */

static bfd_boolean
nios2_coproc_reg (const char *reg_name)
{
  assert (reg_name != NULL);

/* check that we do have a valid register name and that it is a
 * coprocessor register
 * it must begin with c, not be a control register, and be a valid
 * register name */

  if (strncmp (reg_name, "c", 1) == 0 &&
      strncmp (reg_name, "ctl", strlen ("ctl")) != 0 &&
      hash_find (nios2_reg_hash, reg_name) != NULL)
    return TRUE;
  else
    return FALSE;
}


/*********************************************************************
 Argument assemble functions

 Description : All take an instruction argument string, and a pointer
   			   to an instruction opcode. Upon return the insn_opcode
			   has the relevant fields filled in to represent the arg
			   string. The return value is NULL if successful, or
			   an error message if an error was detected
 *********************************************************************/

/* assembles register arguments "dst, src1, src2" */
void
nios2_assemble_args_dst (nios2_insn_infoS * insn_info)
{
  struct nios2_reg *dst, *src1, *src2;

  if (insn_info->insn_tokens[1] != NULL &&
      insn_info->insn_tokens[2] != NULL && insn_info->insn_tokens[3] != NULL)
    {
      dst =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[1]);
      src1 =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[2]);
      src2 =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[3]);

      if (dst == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
      else
	SET_INSN_FIELD (RRD, insn_info->insn_code, dst->index);

      if (src1 == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
      else
	SET_INSN_FIELD (RRS, insn_info->insn_code, src1->index);

      if (src2 == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[3]);
      else
	SET_INSN_FIELD (RRT, insn_info->insn_code, src2->index);


      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[4]);
    }
}


/* assembles arguments successfully parsed by nios2_parse_args_tsi */
void
nios2_assemble_args_tsi (nios2_insn_infoS * insn_info)
{
  struct nios2_reg *dst, *src1;
  unsigned int src2;

  if (insn_info->insn_tokens[1] != NULL &&
      insn_info->insn_tokens[2] != NULL && insn_info->insn_tokens[3] != NULL)
    {
      dst =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[1]);
      src1 =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[2]);
      src2 =
	nios2_assemble_expression (insn_info->insn_tokens[3], insn_info,
				   insn_info->insn_reloc, BFD_RELOC_NIOS2_S16,
				   0);

      if (dst == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
      else
	SET_INSN_FIELD (IRT, insn_info->insn_code, dst->index);

      if (src1 == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
      else
	SET_INSN_FIELD (IRS, insn_info->insn_code, src1->index);

      SET_INSN_FIELD (IMM16, insn_info->insn_code, src2);
      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[4]);

      SET_INSN_FIELD (IMM16, insn_info->insn_code, 0);
    }
}


/* assembles args successfully parsed by nios2_parse_args_tsu */
void
nios2_assemble_args_tsu (nios2_insn_infoS * insn_info)
{
  struct nios2_reg *dst, *src1;
  unsigned int src2;

  if (insn_info->insn_tokens[1] != NULL &&
      insn_info->insn_tokens[2] != NULL && insn_info->insn_tokens[3] != NULL)
    {
      dst =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[1]);
      src1 =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[2]);
      src2 =
	nios2_assemble_expression (insn_info->insn_tokens[3], insn_info,
				   insn_info->insn_reloc, BFD_RELOC_NIOS2_U16,
				   0);

      if (dst == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
      else
	SET_INSN_FIELD (IRT, insn_info->insn_code, dst->index);

      if (src1 == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
      else
	SET_INSN_FIELD (IRS, insn_info->insn_code, src1->index);

      SET_INSN_FIELD (IMM16, insn_info->insn_code, src2);
      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[4]);

      SET_INSN_FIELD (IMM16, insn_info->insn_code, 0);
    }
}


/* assembles args successfully parsed by nios2_parse_args_sti */
void
nios2_assemble_args_sto (nios2_insn_infoS * insn_info)
{
  struct nios2_reg *dst, *src1;
  unsigned int src2;

  if (insn_info->insn_tokens[1] != NULL &&
      insn_info->insn_tokens[2] != NULL && insn_info->insn_tokens[3] != NULL)
    {
      dst =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[1]);
      src1 =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[2]);
      src2 =
	nios2_assemble_expression (insn_info->insn_tokens[3], insn_info,
				   insn_info->insn_reloc, BFD_RELOC_16_PCREL,
				   1);

      if (dst == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
      else
	SET_INSN_FIELD (IRS, insn_info->insn_code, dst->index);

      if (src1 == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
      else
	SET_INSN_FIELD (IRT, insn_info->insn_code, src1->index);

      SET_INSN_FIELD (IMM16, insn_info->insn_code, src2);
      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[4]);

      SET_INSN_FIELD (IMM16, insn_info->insn_code, 0);
    }
}


void
nios2_assemble_args_o (nios2_insn_infoS * insn_info)
{
  unsigned long immed;

  if (insn_info->insn_tokens[1] != NULL)
    {
      immed =
	nios2_assemble_expression (insn_info->insn_tokens[1], insn_info,
				   insn_info->insn_reloc, BFD_RELOC_16_PCREL,
				   1);
      SET_INSN_FIELD (IMM16, insn_info->insn_code, immed);

      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[2]);

      SET_INSN_FIELD (IMM16, insn_info->insn_code, 0);
    }
}


void
nios2_assemble_args_is (nios2_insn_infoS * insn_info)
{
  struct nios2_reg *addr_src;
  unsigned long immed;

  if (insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL)
    {
      addr_src =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[2]);

      immed =
	nios2_assemble_expression (insn_info->insn_tokens[1], insn_info,
				   insn_info->insn_reloc, BFD_RELOC_NIOS2_S16,
				   0);

      SET_INSN_FIELD (IMM16, insn_info->insn_code, immed);

      if (addr_src == NULL)
	as_bad (_("unknown base register %s"), insn_info->insn_tokens[2]);
      else
	SET_INSN_FIELD (RRS, insn_info->insn_code, addr_src->index);

      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[3]);

      SET_INSN_FIELD (IMM16, insn_info->insn_code, 0);
    }
}


void
nios2_assemble_args_m (nios2_insn_infoS * insn_info)
{
  unsigned long immed;
  if (insn_info->insn_tokens[1] != NULL)
    {
      immed =
	nios2_assemble_expression (insn_info->insn_tokens[1], insn_info,
				   insn_info->insn_reloc,
				   BFD_RELOC_NIOS2_CALL26, 0);

      SET_INSN_FIELD (IMM26, insn_info->insn_code, immed);

      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[2]);

      SET_INSN_FIELD (IMM26, insn_info->insn_code, 0);
    }
}


void
nios2_assemble_args_s (nios2_insn_infoS * insn_info)
{
  struct nios2_reg *src;

  if (insn_info->insn_tokens[1] != NULL)
    {
      src =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[1]);

      if (src == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
      else
	SET_INSN_FIELD (RRS, insn_info->insn_code, src->index);

      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[2]);
    }
}


void
nios2_assemble_args_tis (nios2_insn_infoS * insn_info)
{
  struct nios2_reg *addr_src, *dst;
  unsigned long immed;

  if (insn_info->insn_tokens[1] != NULL &&
      insn_info->insn_tokens[2] != NULL && insn_info->insn_tokens[3] != NULL)
    {

      dst =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[1]);
      addr_src =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[3]);
      immed =
	nios2_assemble_expression (insn_info->insn_tokens[2], insn_info,
				   insn_info->insn_reloc, BFD_RELOC_NIOS2_S16,
				   0);


      if (addr_src == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[3]);
      else
	SET_INSN_FIELD (RRS, insn_info->insn_code, addr_src->index);

      if (dst == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
      else
	SET_INSN_FIELD (RRT, insn_info->insn_code, dst->index);

      SET_INSN_FIELD (IMM16, insn_info->insn_code, immed);

      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[4]);

      SET_INSN_FIELD (IMM16, insn_info->insn_code, 0);
    }
}


/* assemble rdctl dst, ctl */
void
nios2_assemble_args_dc (nios2_insn_infoS * insn_info)
{
  struct nios2_reg *dst, *ctl;

  if (insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL)
    {
      ctl =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[2]);
      dst =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[1]);

      if (ctl == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
      else
	SET_INSN_FIELD (RCTL, insn_info->insn_code, ctl->index);

      if (dst == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
      else
	SET_INSN_FIELD (RRD, insn_info->insn_code, dst->index);

      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[3]);
    }
}


/* assemble wrctl ctl, src */
void
nios2_assemble_args_cs (nios2_insn_infoS * insn_info)
{
  struct nios2_reg *src, *ctl;

  if (insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL)
    {
      ctl =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[1]);
      src =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[2]);

      if (ctl == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
      else if (ctl->index == 4)
	as_bad (_("ipending control register (ctl4) is read-only\n"));
      else
	SET_INSN_FIELD (RCTL, insn_info->insn_code, ctl->index);

      if (src == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
      else
	SET_INSN_FIELD (RRS, insn_info->insn_code, src->index);

      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[3]);
    }
}



void
nios2_assemble_args_ldst (nios2_insn_infoS * insn_info)
{
  struct nios2_reg *dst, *src1, *src2;
  unsigned long custom_n;

  if (insn_info->insn_tokens[1] != NULL &&
      insn_info->insn_tokens[2] != NULL &&
      insn_info->insn_tokens[3] != NULL && insn_info->insn_tokens[4] != NULL)
    {
#if 0				/* ??? Unused/half commented out code */
      char *end_p;
      /* custom_n = nios2_strtoul(insn_info->insn_tokens[1], &end_p); */
#endif
      custom_n =
	nios2_assemble_expression (insn_info->insn_tokens[1], insn_info,
				   insn_info->insn_reloc,
				   BFD_RELOC_NIOS2_IMM8, 0);

      dst =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[2]);
      src1 =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[3]);
      src2 =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[4]);

      SET_INSN_FIELD (CUSTOM_N, insn_info->insn_code, custom_n);

      if (dst == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
      else
	SET_INSN_FIELD (RRD, insn_info->insn_code, dst->index);

      if (src1 == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[3]);
      else
	SET_INSN_FIELD (RRS, insn_info->insn_code, src1->index);

      if (src2 == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[4]);
      else
	SET_INSN_FIELD (RRT, insn_info->insn_code, src2->index);

      /* set or clear the bits to indicate whether coprocessor registers are used */
      if (nios2_coproc_reg (insn_info->insn_tokens[2]))
	SET_INSN_FIELD (CUSTOM_C, insn_info->insn_code, 0);
      else
	SET_INSN_FIELD (CUSTOM_C, insn_info->insn_code, 1);

      if (nios2_coproc_reg (insn_info->insn_tokens[3]))
	SET_INSN_FIELD (CUSTOM_A, insn_info->insn_code, 0);
      else
	SET_INSN_FIELD (CUSTOM_A, insn_info->insn_code, 1);

      if (nios2_coproc_reg (insn_info->insn_tokens[4]))
	SET_INSN_FIELD (CUSTOM_B, insn_info->insn_code, 0);
      else
	SET_INSN_FIELD (CUSTOM_B, insn_info->insn_code, 1);


      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[5]);
    }
}


void
nios2_assemble_args_none (nios2_insn_infoS * insn_info ATTRIBUTE_UNUSED)
{
  // nothing to do
}


void
nios2_assemble_args_dsj (nios2_insn_infoS * insn_info)
{
  struct nios2_reg *dst, *src1;
  unsigned int src2;

  if (insn_info->insn_tokens[1] != NULL &&
      insn_info->insn_tokens[2] != NULL && insn_info->insn_tokens[3] != NULL)
    {
      dst =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[1]);
      src1 =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[2]);

      // a 5-bit constant expression
      src2 =
	nios2_assemble_expression (insn_info->insn_tokens[3], insn_info,
				   insn_info->insn_reloc,
				   BFD_RELOC_NIOS2_IMM5, 0);

      if (dst == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
      else
	SET_INSN_FIELD (RRD, insn_info->insn_code, dst->index);

      if (src1 == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
      else
	SET_INSN_FIELD (RRS, insn_info->insn_code, src1->index);

      SET_INSN_FIELD (IMM5, insn_info->insn_code, src2);

      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[4]);

      SET_INSN_FIELD (IMM5, insn_info->insn_code, 0);
    }
}


/* assembles register arguments "dst" */
void
nios2_assemble_args_d (nios2_insn_infoS * insn_info)
{
  struct nios2_reg *dst;

  if (insn_info->insn_tokens[1] != NULL)
    {
      dst =
	(struct nios2_reg *) hash_find (nios2_reg_hash,
					insn_info->insn_tokens[1]);

      if (dst == NULL)
	as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
      else
	SET_INSN_FIELD (RRD, insn_info->insn_code, dst->index);

      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[2]);
    }
}

/* assemble break op */
void
nios2_assemble_args_b (nios2_insn_infoS * insn_info)
{
  unsigned int imm5 = 0;

  if (insn_info->insn_tokens[1] != NULL)
    {
      // a 5-bit constant expression
      imm5 =
	nios2_assemble_expression (insn_info->insn_tokens[1], insn_info,
				   insn_info->insn_reloc,
				   BFD_RELOC_NIOS2_IMM5, 0);

      SET_INSN_FIELD (TRAP_IMM5, insn_info->insn_code, imm5);

      NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[2]);
    }

  SET_INSN_FIELD (TRAP_IMM5, insn_info->insn_code, imm5);

  NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[2]);
}

/* -------------------------------------------------------------- */

/* Vector Co-processor Argurment Assemble Functions					*/

/* -------------------------------------------------------------- */

/* Assembles register arguments: vdst, va */
void nios2_assemble_args_vdva (nios2_insn_infoS * insn_info)
{
	struct nios2_reg *vdst, *vsrc_a;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL)
	{
		vdst = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		vsrc_a = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
		
		if (vdst == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (vdst->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (VD, insn_info->insn_code, vdst->index);

		if (vsrc_a == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (vsrc_a->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[2]);
		else
			SET_VINSN_FIELD (VA, insn_info->insn_code, vsrc_a->index);

		if (strstr(insn_info->insn_tokens[0], ".1") != NULL)
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 1);
		/* Don't technically need to set the mask field to 0, but just to be sure...., also calling SET_VINSN_FIELD
		   will wipe the VPMASK field as well......	*/
		else
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 0);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[3]);
	}
}

void nios2_assemble_args_vdvavb (nios2_insn_infoS * insn_info)
{
struct nios2_reg *vdst, *vsrc_a, *vsrc_b;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL && insn_info->insn_tokens[3] != NULL)
	{
		vdst = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		vsrc_a = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
		vsrc_b = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[3]);
		
		if (vdst == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (vdst->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (VD, insn_info->insn_code, vdst->index);

		if (vsrc_a == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (vsrc_a->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[2]);
		else
			SET_VINSN_FIELD (VA, insn_info->insn_code, vsrc_a->index);

		if (vsrc_b == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[3]);
		else if (vsrc_b->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[3]);
		else
			SET_VINSN_FIELD (VB, insn_info->insn_code, vsrc_b->index);

		if (strstr(insn_info->insn_tokens[0], ".1") != NULL)
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 1);
		/* Don't technically need to set the mask field to 0, but just to be sure.... Also, calling SET_VINSN_FIELD
		   will wipe the VPMASK field as well.	*/
		else
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 0);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[4]);
	}
}

void nios2_assemble_args_vdrsvb (nios2_insn_infoS * insn_info)
{
struct nios2_reg *vdst, *ssrc_a, *vsrc_b;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL && insn_info->insn_tokens[3] != NULL)
	{
		vdst = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		ssrc_a = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
		vsrc_b = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[3]);
		
		if (vdst == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (vdst->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (VD, insn_info->insn_code, vdst->index);

		if (ssrc_a == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (ssrc_a->type == VECTOR_REG)
			as_bad (_("vector register argument %s specified, expected a scalar register"), insn_info->insn_tokens[2]);
		else
			SET_VINSN_FIELD (RSS, insn_info->insn_code, ssrc_a->index);

		/* Always put vector register in VA, for both .vs, or .sv insns */
		if (vsrc_b == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[3]);
		else if (vsrc_b->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[3]);
		else
			SET_VINSN_FIELD (VA, insn_info->insn_code, vsrc_b->index);

		if (strstr(insn_info->insn_tokens[0], ".1") != NULL)
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 1);
		/* Don't technically need to set the mask field to 0, but just to be sure.... Also, calling SET_VINSN_FIELD
		   will wipe the VPMASK field as well.	*/
		else
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 0);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[4]);
	}
}

void nios2_assemble_args_vdvars (nios2_insn_infoS * insn_info)
{
struct nios2_reg *vdst, *vsrc_a, *ssrc_b;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL && insn_info->insn_tokens[3] != NULL)
	{
		vdst = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		vsrc_a = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
		ssrc_b = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[3]);
		
		if (vdst == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (vdst->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (VD, insn_info->insn_code, vdst->index);

		if (vsrc_a == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (vsrc_a->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[2]);
		else
			SET_VINSN_FIELD (VA, insn_info->insn_code, vsrc_a->index);

		if (ssrc_b == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[3]);
		else if (ssrc_b->type == VECTOR_REG)
			as_bad (_("vector register argument %s specified, expected a scalar register"), insn_info->insn_tokens[3]);
		else
			SET_VINSN_FIELD (RSS, insn_info->insn_code, ssrc_b->index);

		if (strstr(insn_info->insn_tokens[0], ".1") != NULL)
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 1);
		/* Don't technically need to set the mask field to 0, but just to be sure.... Also, calling SET_VINSN_FIELD
		   will wipe the VPMASK field as well.	*/
		else
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 0);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[4]);
	}
}

void nios2_assemble_args_vavb (nios2_insn_infoS * insn_info)
{
struct nios2_reg *vsrc_a, *vsrc_b;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] !=NULL)
	{
		vsrc_a = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		vsrc_b = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
		
		if (vsrc_a == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (vsrc_a->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (VA, insn_info->insn_code, vsrc_a->index);

		if (vsrc_b == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (vsrc_b->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[2]);
		else
			SET_VINSN_FIELD (VB, insn_info->insn_code, vsrc_b->index);

		if (strstr(insn_info->insn_tokens[0], ".1") != NULL)
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 1);
		/* Don't technically need to set the mask field to 0, but just to be sure.... Also, calling SET_VINSN_FIELD
		   will wipe the VPMASK field as well.	*/
		else
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 0);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[3]);
	}
}


void nios2_assemble_args_vd (nios2_insn_infoS * insn_info)
{
struct nios2_reg *vdst;
	
	if(insn_info->insn_tokens[1] != NULL)
	{
		vdst = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		
		if (vdst == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (vdst->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (VD, insn_info->insn_code, vdst->index);

		if (strstr(insn_info->insn_tokens[0], ".1") != NULL)
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 1);
		/* Don't technically need to set the mask field to 0, but just to be sure.... Also, calling SET_VINSN_FIELD
		   will wipe the VPMASK field as well.	*/
		else
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 0);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[2]);
	}
}


void nios2_assemble_args_vdrs (nios2_insn_infoS * insn_info)
{
struct nios2_reg *vdst, *ssrc_a;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL)
	{
		vdst = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		ssrc_a = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
		
		if (vdst == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (vdst->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (VD, insn_info->insn_code, vdst->index);

		if (ssrc_a == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (ssrc_a->type == VECTOR_REG)
			as_bad (_("vector register argument %s specified, expected a scalar register"), insn_info->insn_tokens[2]);
		else
			SET_VINSN_FIELD (RSS, insn_info->insn_code, ssrc_a->index);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[3]);
	}
}


/* Vector processor vmstc instruction */
void nios2_assemble_args_vcrs (nios2_insn_infoS * insn_info)
{
struct nios2_reg *vdst, *ssrc_a;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL)
	{
		vdst = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		ssrc_a = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
		
		if (vdst == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (vdst->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[1]);
		
		/* add the control regiser offsets */
		else if (strstr(vdst->name,"vbase") != NULL) {
			/* as_warn (_("vmstc vbase%d"),vdst->index); */
			SET_VINSN_FIELD (VD, insn_info->insn_code, vdst->index + VBASE_OFFSET);
		}
		else if (strstr(vdst->name,"vstride") != NULL) {
			/* as_warn (_("vmstc vstride%d"),vdst->index); */
			SET_VINSN_FIELD (VD, insn_info->insn_code, vdst->index + VSTRIDE_OFFSET);
		}
		else if (strstr(vdst->name,"vinc") != NULL) {
			/* as_warn (_("vmstc vinc%d"),vdst->index); */
			SET_VINSN_FIELD (VD, insn_info->insn_code, vdst->index + VINC_OFFSET);
		}
		else {
			/* as_warn (_("vmstc %d"), vdst->index); */
			SET_VINSN_FIELD (VD, insn_info->insn_code, vdst->index);
		}

		if (ssrc_a == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (ssrc_a->type == VECTOR_REG)
			as_bad (_("vector register argument %s specified, expected a scalar register"), insn_info->insn_tokens[2]);
		else
			SET_VINSN_FIELD (RSS, insn_info->insn_code, ssrc_a->index);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[3]);
	}
}


void nios2_assemble_args_rsvc (nios2_insn_infoS * insn_info)
{
struct nios2_reg *sdst, *vsrc_vc;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL)
	{
		sdst = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		vsrc_vc = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
		
		if (sdst == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (sdst->type == VECTOR_REG)
			as_bad (_("vector register argument %s specified, expected a scalar register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (RSD, insn_info->insn_code, sdst->index);

		if (vsrc_vc == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (vsrc_vc->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[2]);
		
		/* add the control regiser offsets */
		else if (strstr(vsrc_vc->name,"vbase") != NULL) {
			/* as_warn (_("vmcts vbase%d"),vsrc_vc->index); */
			SET_VINSN_FIELD (VA, insn_info->insn_code, vsrc_vc->index + VBASE_OFFSET);
		}
		else if (strstr(vsrc_vc->name,"vstride") != NULL) {
			/* as_warn (_("vmcts vstride%d"),vsrc_vc->index); */
			SET_VINSN_FIELD (VA, insn_info->insn_code, vsrc_vc->index + VSTRIDE_OFFSET);
		}
		else if (strstr(vsrc_vc->name,"vinc") != NULL) {
			/* as_warn (_("vmcts vinc%d"),vsrc_vc->index); */
			SET_VINSN_FIELD (VA, insn_info->insn_code, vsrc_vc->index + VINC_OFFSET);
		}
		else {
			/* as_warn (_("vmcts %d"),vsrc_vc->index); */
			SET_VINSN_FIELD (VA, insn_info->insn_code, vsrc_vc->index);
		}

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[3]);
	}
}


void nios2_assemble_args_rsdva (nios2_insn_infoS * insn_info)
{
struct nios2_reg *sdst, *vsrc_a;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL)
	{
		sdst = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		vsrc_a = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
		
		if (sdst == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (sdst->type == VECTOR_REG)
			as_bad (_("vector register argument %s specified, expected a scalar register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (RSD, insn_info->insn_code, sdst->index);

		if (vsrc_a == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (vsrc_a->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[2]);
		else
			SET_VINSN_FIELD (VA, insn_info->insn_code, vsrc_a->index);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[3]);
	}
}

void nios2_assemble_args_rsvb (nios2_insn_infoS * insn_info)
{
struct nios2_reg *ssrc_a, *vsrc_b;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL)
	{
		ssrc_a = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		vsrc_b = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
		
		if (ssrc_a == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (ssrc_a->type == VECTOR_REG)
			as_bad (_("vector register argument %s specified, expected a scalar register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (RSS, insn_info->insn_code, ssrc_a->index);

		if (vsrc_b == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (vsrc_b->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[2]);
		else
			SET_VINSN_FIELD (VA, insn_info->insn_code, vsrc_b->index);

		if (strstr(insn_info->insn_tokens[0], ".1") != NULL)
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 1);
		/* Don't technically need to set the mask field to 0, but just to be sure.... Also, calling SET_VINSN_FIELD
		   will wipe the VPMASK field as well.	*/
		else
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 0);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[3]);
	}
}

void nios2_assemble_args_vars (nios2_insn_infoS * insn_info)
{
struct nios2_reg *vsrc_a, *ssrc_b;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL)
	{
		vsrc_a = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		ssrc_b = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
		
		if (vsrc_a == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (vsrc_a->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (VA, insn_info->insn_code, vsrc_a->index);

		if (ssrc_b == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (ssrc_b->type == VECTOR_REG)
			as_bad (_("vector register argument %s specified, expected a scalar register"), insn_info->insn_tokens[2]);
		else
			SET_VINSN_FIELD (RSS, insn_info->insn_code, ssrc_b->index);

		if (strstr(insn_info->insn_tokens[0], ".1") != NULL)
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 1);
		/* Don't technically need to set the mask field to 0, but just to be sure.... Also, calling SET_VINSN_FIELD
		   will wipe the VPMASK field as well.	*/
		else
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 0);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[3]);
	}
}


//Memory Functions
//VM-Type: vS/vD,vbase,[vinc]
void nios2_assemble_args_vm_type (nios2_insn_infoS * insn_info)
{
struct nios2_reg *vr, *vbase, *vinc;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL)
	{
		vr = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		vbase = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
				
		if (vr == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (vr->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (VD, insn_info->insn_code, vr->index);

		if (vbase == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (vbase->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[2]);
		else
			SET_VINSN_FIELD (VBASE, insn_info->insn_code, vbase->index);

		if (insn_info->insn_tokens[3] != NULL)
		{
			vinc = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[3]);
			if (vinc == NULL)
				as_bad (_("unknown register %s"), insn_info->insn_tokens[3]);
			else if (vinc->type != VECTOR_REG)
				as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[3]);
			else
				SET_VINSN_FIELD (VINC, insn_info->insn_code, vinc->index);
		}
		else SET_VINSN_FIELD (VINC, insn_info->insn_code, 0);

		if (strstr(insn_info->insn_tokens[0], ".1") != NULL)
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 1);
		/* Don't technically need to set the mask field to 0, but just to be sure.... Also, calling SET_VINSN_FIELD
		   will wipe the VPMASK field as well.	*/
		else
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 0);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[4]);
	}
}

//VMS-Type: vS/vD,vbase,vstride,[vinc]
void nios2_assemble_args_vms_type (nios2_insn_infoS * insn_info)
{
struct nios2_reg *vr, *vbase, *vstride, *vinc;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL && insn_info->insn_tokens[3] != NULL)
	{
		vr = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		vbase = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
		vstride = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[3]);
		
		if (vr == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (vr->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (VD, insn_info->insn_code, vr->index);
		
		if (vbase == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (vbase->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[2]);
		else
			SET_VINSN_FIELD (VBASE, insn_info->insn_code, vbase->index);
		
		if (vstride == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[3]);
		else if (vstride->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[3]);
		else
			SET_VINSN_FIELD (VSTRIDE, insn_info->insn_code, vstride->index);

		if (insn_info->insn_tokens[4] != NULL)
		{
			vinc = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[4]);
			if (vinc == NULL)
				as_bad (_("unknown register %s"), insn_info->insn_tokens[4]);
			else if (vinc->type != VECTOR_REG)
				as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[4]);
			else
				SET_VINSN_FIELD (VINC, insn_info->insn_code, vinc->index);
		}
		else SET_VINSN_FIELD (VINC, insn_info->insn_code, 0);

		if (strstr(insn_info->insn_tokens[0], ".1") != NULL)
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 1);
		/* Don't technically need to set the mask field to 0, but just to be sure.... Also, calling SET_VINSN_FIELD
		   will wipe the VPMASK field as well.	*/
		else
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 0);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[5]);
	}
}

//VMX-Type (Load): vD,voffset,vbase
void nios2_assemble_args_vmx_load (nios2_insn_infoS * insn_info)
{
struct nios2_reg *vdst, *voffset, *vbase;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL && insn_info->insn_tokens[3] != NULL)
	{
		vdst = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		voffset = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
		vbase = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[3]);
		
		if (vdst == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (vdst->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (VD, insn_info->insn_code, vdst->index);

		if (voffset == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (voffset->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[2]);
		else
			SET_VINSN_FIELD (VA, insn_info->insn_code, voffset->index);

		if (vbase == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[3]);
		else if (vbase->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[3]);
		else
			SET_VINSN_FIELD (VBASE, insn_info->insn_code, vbase->index);


		if (strstr(insn_info->insn_tokens[0], ".1") != NULL)
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 1);
		/* Don't technically need to set the mask field to 0, but just to be sure.... Also, calling SET_VINSN_FIELD
		   will wipe the VPMASK field as well.	*/
		else
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 0);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[4]);
	}
}

//VMX-Type (Store): vA,voffset,vbase
void nios2_assemble_args_vmx_store (nios2_insn_infoS * insn_info)
{
struct nios2_reg *voffset, *vsrc, *vbase;
	
	if(insn_info->insn_tokens[1] != NULL && insn_info->insn_tokens[2] != NULL  && insn_info->insn_tokens[3] != NULL)
	{
		vsrc = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[1]);
		voffset = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[2]);
		vbase = (struct nios2_reg *) hash_find (nios2_reg_hash, insn_info->insn_tokens[3]);

		//The VMX-Type encoding places the source vA in the vD field
		if (vsrc == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[1]);
		else if (vsrc->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[2]);
		else
			SET_VINSN_FIELD (VD, insn_info->insn_code, vsrc->index);
			
		if (voffset == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[2]);
		else if (voffset->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[1]);
		else
			SET_VINSN_FIELD (VA, insn_info->insn_code, voffset->index);

		if (vbase == NULL)
			as_bad (_("unknown register %s"), insn_info->insn_tokens[3]);
		else if (vbase->type != VECTOR_REG)
			as_bad (_("scalar register argument %s specified, expected a vector register"), insn_info->insn_tokens[3]);
		else
			SET_VINSN_FIELD (VBASE, insn_info->insn_code, vbase->index);

		if (strstr(insn_info->insn_tokens[0], ".1") != NULL)
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 1);
		/* Don't technically need to set the mask field to 0, but just to be sure.... Also, calling SET_VINSN_FIELD
		   will wipe the VPMASK field as well.	*/
		else
			SET_VINSN_FIELD (VPMASK, insn_info->insn_code, 0);

		NIOS2_CHECK_ASSEMBLY (insn_info->insn_code, insn_info->insn_tokens[4]);
	}
}

/* Machine-dependent assembler directive handling follows */

/*
	.set sets assembler options eg noat/at and is also used
	to set symbol values (.equ, .equiv )
*/
void
s_nios2_set (int equiv)
{
  char *directive = input_line_pointer;
  char delim = get_symbol_end ();
  char *endline;
  endline = input_line_pointer;
  *endline = delim;

  /* we only want to handle ".set XXX" if the
     user has tried ".set XXX, YYY" they are not
     trying a directive. This prevents
     us from polluting the name space */
     
  SKIP_WHITESPACE ();
 
  if (is_end_of_line[(unsigned char) *input_line_pointer]) 
    {
      bfd_boolean done = FALSE;
      *endline = 0;
      
      if (!strcmp (directive, "noat"))
	{
	  done = TRUE;
	  nios2_as_options.noat = TRUE;
	}

      if (!strcmp (directive, "at"))
	{
	  done = TRUE;
	  nios2_as_options.noat = FALSE;
	}

      if (!strcmp (directive, "nobreak"))
	{
	  done = TRUE;
	  nios2_as_options.nobreak = TRUE;
	}

      if (!strcmp (directive, "break"))
	{
	  done = TRUE;
	  nios2_as_options.nobreak = FALSE;
	}

      if (!strcmp (directive, "norelax"))
	{
	  done = TRUE;
	  nios2_as_options.relax = relax_none;
	}
      else if (!strcmp (directive, "relaxsection"))
	{
	  done = TRUE;
	  nios2_as_options.relax = relax_section;
	}
      else if (!strcmp (directive, "relaxall"))
	{
	  done = TRUE;
	  nios2_as_options.relax = relax_all;
	}
	
	
      if (done)
	{
	  *endline = delim;
	  demand_empty_rest_of_line ();
	  return;
	}
    }


  /* If we fall through to here, either we have ".set XXX, YYY"
     or we have ".set XXX" where XXX is unknown or we have 
     a syntax error */
  input_line_pointer = directive;
  *endline = delim;
  s_set (equiv);
}

/* nop fill pattern for text section */
static char const nop[4] = { 0x3a, 0x88, 0x01, 0x00 };

/* nios2_frob_label() is called when after a label is recognized.  */

void
nios2_frob_label (symbolS * lab)
{
  /* Update the label's address with the current output pointer.  */
  symbol_set_frag (lab, frag_now);
  S_SET_VALUE (lab, (valueT) frag_now_fix ());

  /* Record this label for future adjustment after we find out what
     kind of data it references, and the required alignment therewith.  */
  nios2_last_label = lab;
}



/* Hook into cons for auto-alignment.  */

void
nios2_cons_align (int size)
{
  int log_size;
  const char *pfill = NULL;

  log_size = 0;
  while ((size >>= 1) != 0)
    ++log_size;

  if (subseg_text_p (now_seg))
    {
      pfill = (const char *) &nop;
    }
  else
    pfill = NULL;

  if (nios2_auto_align_on)
    nios2_align (log_size, pfill, NULL);

  nios2_last_label = NULL;
}

static void
s_nios2_sdata (int ignore ATTRIBUTE_UNUSED)
{
  int temp;

  temp = get_absolute_expression ();
  subseg_new (".sdata", 0);
  demand_empty_rest_of_line ();
}

/* Map 's' to SHF_NIOS2_GPREL.  */
/* this is from the Alpha code tc-alpha.c */
int
nios2_elf_section_letter (int letter, char **ptr_msg)
{
  if (letter == 's')
    return SHF_NIOS2_GPREL;

  *ptr_msg = _("Bad .section directive: want a,s,w,x,M,S,G,T in string");
  return 0;
}

/* Map SHF_ALPHA_GPREL to SEC_SMALL_DATA.  */
/* this is from the Alpha code tc-alpha.c */
flagword
nios2_elf_section_flags (flagword flags, int attr, int type ATTRIBUTE_UNUSED)
{
  if (attr & SHF_NIOS2_GPREL)
    flags |= SEC_SMALL_DATA;
  return flags;
}

/* explicitly unaligned cons */

static void
s_nios2_ucons (int nbytes)
{
  int hold;
  hold = nios2_auto_align_on;
  nios2_auto_align_on = 0;
  cons (nbytes);
  nios2_auto_align_on = hold;
}

/* Handles all machine-dependent alignment needs */
static void
nios2_align (int log_size, const char *pfill, symbolS * label)
{
  int align;
  long max_alignment = 15;

  /* The front end is prone to changing segments out from under us
     temporarily when -g is in effect.  */
  int switched_seg_p = (nios2_current_align_seg != now_seg);

  align = log_size;
  if (align > max_alignment)
    {
      align = max_alignment;
      as_bad (_("Alignment too large: %d. assumed"), align);
    }
  else if (align < 0)
    {
      as_warn (_("Alignment negative: 0 assumed"));
      align = 0;
    }

  if (align != 0)
    {
      if (subseg_text_p (now_seg) && align >= 2)
	{
	  /* First, make sure we're on a four-byte boundary, in case
	     someone has been putting .byte values the text section.  */
	  if (nios2_current_align < 2 || switched_seg_p)
	    frag_align (2, 0, 0);

	  /* now fill in the alignment pattern */
	  if (pfill != NULL)
	    frag_align_pattern (align, pfill, sizeof nop, 0);
	  else
	    frag_align (align, 0, 0);
	}
      else
	{
	  frag_align (align, 0, 0);
	}

      if (!switched_seg_p)
	nios2_current_align = align;

      /* If the last label was in a different section we can't align it */
      if (label != NULL && !switched_seg_p)
	{
	  symbolS *sym;
	  int label_seen = FALSE;
	  struct frag *old_frag;
	  valueT old_value;
	  valueT new_value;

	  assert (S_GET_SEGMENT (label) == now_seg);

	  old_frag = symbol_get_frag (label);
	  old_value = S_GET_VALUE (label);
	  new_value = (valueT) frag_now_fix ();

	  /* It is possible to have more than one label at a particular
	     address, especially if debugging is enabled, so we must
	     take care to adjust all the labels at this address in this
	     fragment.  To save time we search from the end of the symbol
	     list, backwards, since the symbols we are interested in are
	     almost certainly the ones that were most recently added.
	     Also to save time we stop searching once we have seen at least
	     one matching label, and we encounter a label that is no longer
	     in the target fragment.  Note, this search is guaranteed to
	     find at least one match when sym == label, so no special case
	     code is necessary.  */
	  for (sym = symbol_lastP; sym != NULL; sym = symbol_previous (sym))
	    {
	      if (symbol_get_frag (sym) == old_frag
		  && S_GET_VALUE (sym) == old_value)
		{
		  label_seen = TRUE;
		  symbol_set_frag (sym, frag_now);
		  S_SET_VALUE (sym, new_value);
		}
	      else if (label_seen && symbol_get_frag (sym) != old_frag)
		break;
	    }
	}
      record_alignment (now_seg, align);
    }
}

/* This is called from HANDLE_ALIGN in tc-nios2.h.  */

void
nios2_handle_align (fragS * fragp)
{
  /* If we are expecting to relax in the linker, then we must output a relocation
   * to tell the linker we are aligning code */
  if (nios2_as_options.relax == relax_all
      && (fragp->fr_type == rs_align
	  || fragp->fr_type == rs_align_code)
      && fragp->fr_address + fragp->fr_fix > 0
      && fragp->fr_offset > 1 && now_seg != bss_section)
    fix_new (fragp, fragp->fr_fix, 4, &abs_symbol, fragp->fr_offset, 0,
	     BFD_RELOC_NIOS2_ALIGN);

}

/* Handle the .align pseudo-op.  This aligns to a power of two.  It
   also adjusts any current instruction label.  We treat this the same
   way the MIPS port does: .align 0 turns off auto alignment.  */

static void
s_nios2_align (int ignore ATTRIBUTE_UNUSED)
{
  int align;
  char fill;
  const char *pfill = NULL;
  long max_alignment = 15;


  align = get_absolute_expression ();
  if (align > max_alignment)
    {
      align = max_alignment;
      as_bad (_("Alignment too large: %d. assumed"), align);
    }
  else if (align < 0)
    {
      as_warn (_("Alignment negative: 0 assumed"));
      align = 0;
    }

  if (*input_line_pointer == ',')
    {
      input_line_pointer++;
      fill = get_absolute_expression ();
      pfill = (const char *) &fill;
    }
  else if (subseg_text_p (now_seg))
    {
      pfill = (const char *) &nop;
    }
  else
    {
      pfill = NULL;
      nios2_last_label = NULL;
    }

  if (align != 0)
    {
      nios2_auto_align_on = 1;
      nios2_align (align, pfill, nios2_last_label);
      nios2_last_label = NULL;
    }
  else
    {
      nios2_auto_align_on = 0;
    }

  demand_empty_rest_of_line ();
}


/* Handle the .text pseudo-op.  This is like the usual one, but it
   clears the saved last label and resets known alignment.  */

static void
s_nios2_text (int i)
{
  s_text (i);
  nios2_last_label = NULL;
  nios2_current_align = 0;
  nios2_current_align_seg = now_seg;
}

/* Handle the .data pseudo-op.  This is like the usual one, but it
   clears the saved last label and resets known alignment.  */

static void
s_nios2_data (int i)
{
  s_data (i);
  nios2_last_label = NULL;
  nios2_current_align = 0;
  nios2_current_align_seg = now_seg;
}

/* Handle the .section pseudo-op.  This is like the usual one, but it
   clears the saved last label and resets known alignment.  */

static void
s_nios2_section (int ignore)
{
  obj_elf_section (ignore);
  nios2_last_label = NULL;
  nios2_current_align = 0;
  nios2_current_align_seg = now_seg;
}
