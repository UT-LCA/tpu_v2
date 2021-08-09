/*
	# bfloat add
	# int bfloat_add(
	#	size_t n,	//elements to add
	#	const unsigned char *A,
	#	const unsigned char *B,
	#	unsigned char *C)
	#
*/

#include "regdef.h"
 
        .ent vector_permute
        .globl vector_permute
 
        .set noreorder

vector_permute:

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
#define addr_stride     vstride1				

	# code to handle stack here ( 1 register saved)
  # subtract unsigned
	subu    sp, sp, 32
  # store word
	sw  ra, 8(sp)


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
	
	# set shamt_v, vpw, vfmask0
  # load immediate
	#li	t3, 8 #this is not used anywhere
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
	vld.u.b		b_v, b_addr, addr_inc	# load A	
  nop
  nop
  nop

	#vadd.vv		c_v, a_v, b_v		# c = a + b
	#vmullo.vv		c_v, a_v, b_v		# c = a * b
        vsts.w                  a_v, a_addr, addr_stride, addr_inc 
        nop
        nop
        nop
        vstx.w                  b_v, b_v, b_addr
        nop
        nop
        nop
        vldx.w                c_v, b_v, b_addr
  #nop

	#vadd.vv		$vr9, $vr7, $vr8	#bias operation
	vst.b	  	c_v, c_addr, addr_inc	# store C
  #nop

	# restore stack
	lw	ra, 8(sp)
	j ra
	addu	sp, sp, 32

	.end	vector_permute
