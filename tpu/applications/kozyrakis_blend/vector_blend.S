/*
	###########################################################
        # $Id: composit_sched_ai.s,v 1.1 1999/05/27 02:56:43 kozyraki Exp kozyraki $
        ###########################################################

	# Imgage composition function
	# scheduled version
	# with autoincrement addressing
	#
	# should be within 8/9 of optimal (mod conflicts)
	
	# This function blends two images, A and B, by a blend factor y,
	# to produce the output image, C.  The elements of all three
	# images are 8-bit unsigned chars, as is the blend factor.  A
	# blend factor of 0.5 (which is represented by 0x80) will
	# produce an output image that is 50% A and 50% B.  A blend
	# factor of 0 corresponds to 100% B, and y = 1.0 corresponds to
	# 100% A (although 1.0 cannot be represented).

	# int vector_blend(
	#	size_t n,	Pixels to process. 
	#	const unsigned char *A,
	#	const unsigned char *B,
	#	unsigned char y,	a 0.8 fraction 
	#	unsigned char *C)
	#
*/

#include "regdef.h"
 
        .ent vector_blend
        .globl vector_blend
 
        .set noreorder

vector_blend:

#define	y_v		$vs5
#define	minus_y_v	$vs6	
#define shamt_v		$vs7
	
#define	a_v		$vr0
#define	b_v		$vr1
#define ta_v		$vr2
#define tb_v		$vr3
#define c_v		$vr4

#define	n_pixels_r	a0
#define	a_r		a1
#define	b_r		a2
#define	y_r		a3
#PETES CHANGE - a4 does not exist
#define	c_r		t2
#define	minus_y_r	t0
#define mvl_r		t1

#define a_addr		vbase1
#define b_addr		vbase2
#define c_addr		vbase3
#define addr_inc	vinc1				
setup:
   lw          c_r,16(sp)   #PETES CHANGE

	# code to handle stack here ( 1 register saved)
	subu    sp, sp, 32
	sw  ra, 8(sp)
	
	# set shamt_v, vpw, vfmask0
	li	t3, 8
	mtc2	t3, shamt_v
	li	t3, 1
	ctc2	t3, vpw			# set vpw to 16b
	cfc2	mvl_r, mvl	# read mvl
  ctc2   mvl_r, vl # set vl to maximum	- PETES CHANGE vmctc does not exist
  vfset   $vf0		# initial mask
		 
	# produce 1-weigth (1-y)
	sub	minus_y_r, $0, y_r
	andi	minus_y_r, minus_y_r, 0xff
	
	# move y, 1-y to vector scalar registers
	mtc2	y_r, y_v
	mtc2	minus_y_r, minus_y_v

	ctc2	a_r, a_addr
	ctc2	b_r, b_addr
	ctc2	c_r, c_addr
  ctc2  mvl_r, addr_inc   # PETES CHANGE vmctc instruction does not exist
			
	# test if the weight is 0
	beq	y_r, $0, zero_case
  nop                 # PETES CHANGE delay slot scheduling

	
	# this part starts 2 iterations (and prefetches 2 other)
start_loop:
	ctc2		n_pixels_r, vl
	vsatvl
	
	vld.u.b		a_v, a_addr, addr_inc	# load A	
	vld.u.b		b_v, b_addr, addr_inc	# load B	
	vmullo.sv	ta_v, y_v, a_v		# a*y
	vmullo.sv	tb_v, minus_y_v, b_v	# b*(1-y)
	vld.u.b		a_v, a_addr, addr_inc	# load A
	subu		n_pixels_r, n_pixels_r, mvl_r
	vadd.vv		c_v, ta_v, tb_v		# sum = ay+b(1-y)
	vmullo.sv	ta_v, y_v, a_v		# a*y
	vld.u.b		b_v, b_addr, addr_inc	# load B
	vsra.vs		c_v, c_v, shamt_v	# sum >> 8
	blez		n_pixels_r, exit_loop
  nop

loop:	
	vmullo.sv	tb_v, minus_y_v, b_v	# b*(1-y)
	vst.b		c_v, c_addr, addr_inc	# store C
	vld.u.b		a_v, a_addr, addr_inc	# load A
	subu		n_pixels_r, n_pixels_r, mvl_r
	vadd.vv		c_v, ta_v, tb_v		# sum = ay+b(1-y)
	vmullo.sv	ta_v, y_v, a_v		# a*y
	vld.u.b		b_v, b_addr, addr_inc	# load B
	vsra.vs		c_v, c_v, shamt_v	# sum >> 8
	bgtz		n_pixels_r, loop
  nop
	
	
exit_loop:
	
	vst.b		c_v, c_addr, addr_inc	# store C

	
end:	
	# restore stack
	lw	ra, 8(sp)
	j ra
	addu	sp, sp, 32

	
	###################################################################
	
	# case of weight (y==0)
	# more or less the same program without arithmetic ops

zero_case:
	ctc2		n_pixels_r, vl
	vsatvl
zero_loop:	
	vld.u.b		b_v, b_addr, addr_inc		# load B
	vst.b		c_v, c_addr, addr_inc		# store C
	subu		n_pixels_r, n_pixels_r, mvl_r
	bgtz		n_pixels_r, zero_loop
  nop

	j end	
  nop

	.end	vector_blend

    ###########################################################
    # $Log: composit_sched_ai.s,v $
    # Revision 1.1  1999/05/27 02:56:43  kozyraki
    # Initial revision
    #
    # Revision 1.3  1999/05/27 02:42:58  kozyraki
    # version that leads to  6.023 GOPS
    #
    # Revision 1.2  1999/05/24 19:04:01  kozyraki
    # Scheduled version that takes into account the fact
    # that it is better to send all multiplies to one FU
    #
    # Revision 1.1  1999/05/23 20:14:04  kozyraki
    # Initial revision
    #
    ###########################################################
 
