/*
	# Vector add
	# int vector_add(
	#	size_t n,	//elements to add
	#	const unsigned char *A,
	#	const unsigned char *B,
	#	unsigned char *C)
	#
*/

#include "regdef.h"
 
        .ent vector_add
        .globl vector_add
 
        .set noreorder

vector_add:

#define shamt_v		$vs7
	
#define	a_v		$vr0
#define	b_v		$vr1
#define c_v		$vr4

#define dma_ctrl        $vc30
#define mem_add         $vc29
#define lane_add        $vc28
#define dma_busy        $vc27

# we get function arguments in a0-a3
#define	n_pixels_r	a0
#define	a_r		a1
#define	b_r		a2
#define	c_r		a3

#define mvl_r		t1

#define a_addr		vbase1
#define b_addr		vbase2
#define c_addr		vbase3
#define addr_inc	vinc1				

	# code to handle stack here ( 1 register saved)
  # subtract unsigned
	subu    sp, sp, 32
  # store word
	sw  ra, 8(sp)
	
	# set shamt_v, vpw, vfmask0
  # load immediate
	#li	t3, 8 #this is not used anywhere
  ### Sangram
        li t3, 0x10000000  ##Main memory access address
        ctc2 t3,mem_add
        li t3, 0
        ctc2 t3, lane_add  # Local memory per lane start address
        li t3, 257 # numbytes = 64, we = 0, en = 1, (final value = 64 << 2 + 1 = 257)
        ctc2 t3,dma_ctrl
        li t3, 256 # setting the enable bit to low
        ctc2 t3, dma_ctrl
        li t3, 1
target: cfc2 t4,dma_busy
        beq t4,t3,target
  ###
  # move to coprocessor (t3->shamt_v)
	#mtc2	t3, shamt_v #we don't need this in the simple matmul program
  # load immediate
	li	t3, 1
  # control to coprocessor (t3->vpw)
	ctc2	t3, vpw			# set vpw to 8b #this doesn't matter really since we are doing simple adds for +ve numbers. it may matter with signed adds, shifts, etc
  # control from coprocessor (mvl_r <- mvl)
	cfc2	mvl_r, mvl	# read mvl
  # control to coprocessor (mvl_r -> vl)
  ctc2   mvl_r, vl # set vl to maximum	- PETES CHANGE vmctc does not exist
  # set flags to 1
  vfset   $vf0		# initial mask
		 
  # copy scalar register to vector-scalar register
	ctc2	a_r, a_addr
	ctc2	b_r, b_addr
	ctc2	c_r, c_addr
  ctc2  mvl_r, addr_inc   # PETES CHANGE vmctc instruction does not exist
			
  # control to coprocessor (n_pixels_r -> vl)
	ctc2		n_pixels_r, vl
	vsatvl
	
	vld.u.b		a_v, a_addr, addr_inc	# load A	
	vld.u.b		b_v, b_addr, addr_inc	# load B	
  nop
  nop
  nop
	vadd.vv		$vr7, a_v, b_v		# c = a + b
	vadd.vv		$vr8, a_v, b_v		# c = a + b
  nop
  nop
  nop
  #matmul masks 
  # vc31 - a_rows
  # vc30 - a_cols, b_rows
  # vc29 - b_cols
  li t3, 0xf0
  ctc2 t3, $vc31
  ctc2 t3, $vc30
  ctc2 t3, $vc29

	#vadd.vv		c_v, a_v, b_v		# c = a + b
	#vmullo.vv		c_v, a_v, b_v		# c = a * b
	vdiv.vv		c_v, a_v, b_v		# c = a matmul b #hack:  fixme: todo: Using DIV as Matmul
  #nop

	#vadd.vv		$vr9, $vr7, $vr8	#bias operation
	vst.b	  	c_v, c_addr, addr_inc	# store C
  #nop

	# restore stack
	lw	ra, 8(sp)
	j ra
	addu	sp, sp, 32

	.end	vector_add
