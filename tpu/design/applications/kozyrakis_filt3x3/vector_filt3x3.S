        ###########################################################
        # $Id: filt_ai.s,v 1.1 1999/06/01 02:34:26 kozyraki Exp kozyraki $
        ###########################################################
 
       
	# This function convolves an image by a 3x3 kernel. The
	# elements of the input and output images are 8-bit unsigned
	# chars.  The 9 kernel elements are signed 16-bits and represent
	# fixed point values.  The location of the binary point is
	# determined by shamt input(8 means 8.8 kernel elements, and 0
	# means 16.0 elements). The routine uses a region of the input
	# image defined by n_rows and n_cols.  It writes the result into
	# a (n_rows-2)x(n_cols-2) region of the output image, since
	# boundary pixels (where the kernel falls over the edge of the
	# input image) are ignored. 

	# test file for 8 lanes
	
	# int nv_filt(
	#		size_t n_rows,	/* Rows to process. */
	#		size_t n_cols,	/* Columns to process. */
	#		size_t A_stride,	/* Columns in A image. */
	#		size_t B_stride,	/* Columns in B image. */
	#		unsigned char *A,
	#		unsigned char *B,
	#		short int K[3][3],	/* 3x3 kernel */
	#		int shamt	/* shift amount */ )
 
#include "regdef.h"

	.ent filt
	.globl filt

filt:

	#arguments
#define n_rows_r	a0
#define n_cols_r	a1
#define a_stride_r	a2
#define b_stride_r	a3
#define a_r		t8
#define b_r		t9
#define kptr_r		v0
#define shamt_r		v1

	# scalar variables
#define k00_r		t0
#define k01_r		t1
#define k02_r		t2
#define k10_r		t3
#define k11_r		s0
#define k12_r		s1
#define k20_r		s2
#define k21_r		s3
#define k22_r		s4
#define row_count_r	s5
#define mvl_r		s7
		
	#vector variables
#define in0_v		$vr0
#define in1_v		$vr1
#define in2_v		$vr2
#define prod0_v		$vr3
#define prod1_v		$vr4
#define prod2_v		$vr5
#define tmp1_v		$vr6
#define tmp2_v		$vr7
#define tmp3_v		$vr8

	#vector scalar variables
#define shamt_v		$vs1
#define k00_v		$vs2
#define k01_v		$vs3
#define k02_v		$vs4
#define k10_v		$vs5
#define k11_v		$vs6
#define k12_v		$vs7
#define k20_v		$vs8
#define k21_v		$vs9
#define k22_v		$vs10


#define a_addr		vbase1
#define b_addr		vbase2
#define step1		vinc1
#define step_col_a	vinc2
#define step_col_b	vinc3

setup:

	.set noreorder 

	# test if zero rows
	beqz	n_rows_r, exit
  nop

  lw          a_r,16(sp)   #PETES CHANGE
  lw          b_r,20(sp)   #PETES CHANGE
  lw          kptr_r,24(sp)   #PETES CHANGE
  lw          shamt_r,28(sp)   #PETES CHANGE


	#save variables in stack
        subu    sp, sp, 128
        sw  ra, 8(sp)
        sw  s0, 16(sp)
        sw  s1, 24(sp)
        sw  s2, 32(sp)
        sw  s3, 40(sp)
        sw  s4, 48(sp)
        sw  s5, 56(sp)
        sw  s6, 64(sp)
        sw  s7, 72(sp)

        .frame sp, 0, ra

	# set vpw, vfmask0
	li	t2, 2		# 32-bit vpw...
	ctc2	t2, vpw
	cfc2	mvl_r, mvl
	ctc2	mvl_r, vl
	vfset	$vf0
	
	# get the 3x3 filter from memory (16-bit singed values)
	lh	k00_r,  0(kptr_r)
	lh	k10_r,  2(kptr_r)
	lh	k20_r,  4(kptr_r)
	lh	k01_r,  6(kptr_r)
	lh	k11_r,  8(kptr_r)
	lh	k21_r, 10(kptr_r)
	lh	k02_r, 12(kptr_r)
	lh	k10_r, 14(kptr_r)
	lh	k22_r, 16(kptr_r)

	# move 3x3 filter to vector scalar regiters
	mtc2	k00_r, k00_v
	mtc2	k01_r, k01_v
	mtc2	k02_r, k02_v
	mtc2	k10_r, k10_v
	mtc2	k11_r, k11_v
	mtc2	k12_r, k12_v
	mtc2	k20_r, k20_v
	mtc2	k21_r, k21_v
	mtc2	k22_r, k22_v
	# move shift amount to vector scalar register
	mtc2	shamt_r, shamt_v

	subu	n_rows_r, n_rows_r, 2
	subu	n_cols_r, n_cols_r, 2

	
	li	k00_r, 1
	ctc2	k00_r, step1
	subu	s6, a_stride_r, 2
	ctc2	s6, step_col_a
	ctc2	b_stride_r, step_col_b
	

col_loop:
	ctc2		n_cols_r, vl
	vsatvl

	ctc2		a_r, a_addr
	ctc2		b_r, b_addr
	
	#first row
	vld.u.b		in0_v, a_addr, step1
	vld.u.b		in1_v, a_addr, step1
	addu		row_count_r, n_rows_r, $0
	vmullo.sv	prod0_v, k00_v, in0_v
	vld.u.b		in2_v, a_addr, step_col_a
	vmullo.sv	tmp1_v, k01_v, in1_v
	vmullo.sv	tmp2_v, k02_v, in2_v

	#second row
	vld.u.b		in0_v, a_addr, step1
	vadd.vv		prod0_v, prod0_v, tmp1_v	# from 1st row
	vmullo.sv	tmp1_v, k10_v, in0_v
	vld.u.b		in1_v, a_addr, step1
	vadd.vv		prod0_v, prod0_v, tmp2_v	# from 1st row
	vmullo.sv	tmp2_v, k11_v, in1_v
	vld.u.b		in2_v, a_addr, step_col_a
	vadd.vv		prod0_v, prod0_v, tmp1_v
	vmullo.sv	tmp1_v, k12_v, in2_v
	vadd.vv		prod0_v,prod0_v, tmp2_v
	vmullo.sv	prod1_v, k00_v, in0_v
	vadd.vv		prod0_v,prod0_v, tmp1_v
	vmullo.sv	tmp2_v, k01_v, in1_v
	vld.u.b		in0_v, a_addr, step1		# little prefetching
	vadd.vv		prod1_v,prod1_v, tmp2_v
	vmullo.sv	tmp2_v, k02_v, in2_v
	vadd.vv		prod1_v,prod1_v, tmp2_v
	vmullo.sv	tmp1_v, k20_v, in0_v
row_loop:
	
	
	vld.u.b		in1_v, a_addr, step1
	vmullo.sv	tmp2_v, k10_v, in0_v
	vadd.vv		prod0_v, prod0_v, tmp1_v
	vmullo.sv	tmp1_v, k21_v, in1_v	
	vadd.vv		prod1_v, prod1_v, tmp2_v
	vmullo.sv	tmp2_v, k11_v, in1_v
	vld.u.b		in2_v, a_addr, step_col_a
	vadd.vv		prod0_v, prod0_v, tmp1_v
	
	vmullo.sv	tmp1_v, k22_v, in2_v
	vadd.vv		prod1_v, prod1_v, tmp2_v
	vmullo.sv	tmp2_v, k12_v, in2_v
	vadd.vv		prod0_v, prod0_v, tmp1_v
	vmullo.sv	prod2_v, k00_v, in0_v
	vsra.vs		prod0_v, prod0_v, shamt_v
	vmullo.sv	tmp1_v, k01_v, in1_v
	vst.b		prod0_v, b_addr, step_col_b
	vadd.vv		prod0_v, prod1_v, tmp2_v
	vmullo.sv	tmp3_v, k02_v, in2_v
	vld.u.b		in0_v, a_addr, step1		# little prefetching
	subu		row_count_r, row_count_r, 1
	vadd.vv		prod2_v, prod2_v, tmp1_v
	vmullo.sv	tmp1_v, k20_v, in0_v
	vadd.vv		prod1_v, prod2_v, tmp3_v
	bnez		row_count_r, row_loop
  nop
		
	# loop based on col_count back to col loop
	addu		a_r, a_r, mvl_r
	ctc2		a_r, a_addr
	subu		n_cols_r, n_cols_r, mvl_r
	bgtz		n_cols_r, col_loop
	addu		b_r, b_r, mvl_r	


done:

	#restore stack...
	lw  ra, 8(sp)
	lw  s0, 16(sp)
	lw  s1, 24(sp)
	lw  s2, 32(sp)
	lw  s3, 40(sp)
	lw  s4, 48(sp)
	lw  s5, 56(sp)
	lw  s6, 64(sp)
	lw  s7, 72(sp)
	addu    sp, sp, 128

exit:	
  j ra
  nop
 
    .end    filt
 
    ###########################################################
    # $Log: filt_ai.s,v $
    # Revision 1.1  1999/06/01 02:34:26  kozyraki
    # Initial revision
    #
    # Revision 1.2  1999/05/27 04:44:47  kozyraki
    # Version that goes up to 92.2% of peak...
    #
    # Revision 1.1  1999/05/24 22:24:25  kozyraki
    # Initial revision
    #
    ###########################################################

