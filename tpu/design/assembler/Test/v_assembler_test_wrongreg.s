	.file	"hello_world.c"
	.section	.text
	.align	2
main:
	movhi	r8, %hiadj(4320000)
	addi	r8, r8, %lo(4320000)
	movhi	r2, %hiadj(1920000)
	addi	r2, r2, %lo(1920000)
	sub	sp, sp, r8
	add	r6, sp, r2
	movi	r5, 98
	mov	r2, zero
	movi	r4, 97
.L44:
	add	r3, r6, r2
	add	r7, sp, r2
	stb	r5, 0(r7)
	stb	r4, 0(r3)
	movi	r2, 2
	br	VECTOR
	br	.L44						
VECTOR:								#Expected Output for each Instruction
									#A=1010 B=1011 C=1100 D=1101 E=1110 F=1111
	vabs v0, v1						#											= 0x00100F7D
	vabs.1 r23, v1					#											= 0x00102F7D

	vabsdiff.vv		v0, v1, v2		#0000 0000 0001 0000 1000 0011 0111 1101	= 0x0010837D
	vabsdiff.vv.1	v0, v1, v2		#											= 0x0010A37D
	vabsdiff.sv		v3, r17, v2 	#0000 1100 0010 1000 1001 0011 0111 1101	= 0x0C28937D
	vabsdiff.sv.1	v3, r17, v2		#0000 1100 0010 1000 1011 0011 0111 1101	= 0x0C28B37D

	vadd.vv		v0, v1, v2			#000000 000001 000010 0 0 000000 111101		= 0x0010803D
	vadd.vv.1	v0, v1, v2			#000000 000001 000010 1 0 000000 111101		= 0x0010A03D
	vadd.sv		v5, r18, v2			#000101 000010 10010 0 0 1 000000 111101	= 0x1429103D
	vadd.sv.1	v5, r18, v2			#000101 000010 10010 0 1 1 000000 111101	= 0x1429303D
	vaddu.vv	v0, v1, v2			#000000 000001 000010 0 0 001000 111101		= 0x0010823D
	vaddu.vv.1	v0, v1, v2			#000000 000001 000010 1 0 001000 111101		= 0x0010A23D
	vaddu.sv	v5, r18, v2			#000101 000010 10010 0 0 1 001000 111101	= 0x1429123D
	vaddu.sv.1	v5, r18, v2			#000101 000010 10010 0 1 1 001000 111101	= 0x1429323D

	vsub.vv		v32, v33, v34		#100000 100001 100010 0 0 000001 111101		= 0x8218807D
	vsub.vv.1	v32, v33, v34		#100000 100001 100010 1 0 000001 111101		= 0x8218A07D
	vsub.sv		v32, r23, v34		#100000 100010 10111 0 0 1 000001 111101	= 0x822B907D
	vsub.sv.1	v32, r23, v34		#100000 100010 10111 0 1 1 000001 111101	= 0x822BB07D
	vsub.vs		v32, v34, r23		#100000 100010 10111 0 0 1 110001 111101	= 0x822B9C7D
	vsub.vs.1	v32, v34, r23		#100000 100010 10111 0 1 1 110001 111101	= 0x822BBC7D
	vsubu.vv	v32, v33, v34		#100000 100001 100010 0 0 001001 111101		= 0x8218827D
	vsubu.vv.1	v32, v33, v34		#100000 100001 100010 1 0 001001 111101		= 0x8218A27D
	vsubu.sv	v32, r23, v34		#100000 100010 10111 0 0 1 001001 111101	= 0x822B927D
	vsubu.sv.1	v32, r23, v34		#											= 0x822BB27D
	vsubu.vs	v32, v34, r23		#100000 100010 10111 0 0 1 111001 111101	= 0x822B9E7D
	vsubu.vs.1	v32, v34, r23		#											= 0x822BBE7D

	vmulhi.vv		v0, v20, v21	#000000 010100 010101 0 0 100110 111101		= 0x014549BD
	vmulhi.vv.1		v0, v20, v21	#000000 010100 010101 1 0 100110 111101		= 0x014569BD
	vmulhi.sv		v0, r23, v21	#000000 010101 10111 0 0 1 100110 111101	= 0x015B99BD
	vmulhi.sv.1		v0, r23, v21	#											= 0x015BB9BD
	vmulhiu.vv		v0, v20, v21	#000000	010100 010101 0 0 101110 111101		= 0x01454BBD
	vmulhiu.vv.1	v0, v20, v21	#											= 0x01456BBD
	vmulhiu.sv		v0, r23, v21	#											= 0x015B9BBD
	vmulhiu.sv.1	v0, r23, v21	#											= 0x015BBBBD

	vmullo.vv		v0, v20, v21	#000000 010100 010101 0 0 100111 111101		= 0x014549FD
	vmullo.vv.1		v0, v20, v21	#											= 0x014569FD
	vmullo.sv		v0, r23, v21	#											= 0x015B99FD
	vmullo.sv.1		v0, r23, v21	#											= 0x015BB9FD
	vmullou.vv		v0, v20, v21	#000000	010100 010101 0 0 101111 111101		= 0x01454BFD
	vmullou.vv.1	v0, v20, v21	#											= 0x01456BFD
	vmullou.sv		v0, r23, v21	#											= 0x015B9BFD
	vmullou.sv.1	v0, r23, v21	#											= 0x015BBBFD

	vdiv.vv		v32, v33, v34		#100000 100001 100010 0 0 010110 111101		= 0x821885BD
	vdiv.vv.1	v32, v33, v34		#											= 0x8218A5BD
	vdiv.sv		v32, r23, v34		#100000 100010 10111 0 0 1 010110 111101	= 0x822B95BD
	vdiv.sv.1	v32, r23, v34		#											= 0x822BB5BD
	vdiv.vs		v32, v34, r23		#100000 100010 10111 0 0 1 110110 111101	= 0x822B9DBD
	vdiv.vs.1	v32, v34, r23		#											= 0x822BBDBD
	vdivu.vv	v32, v33, v34		#											= 0x821887BD
	vdivu.vv.1	v32, v33, v34		#											= 0x8218A7BD
	vdivu.sv	v32, r23, v34		#											= 0x822B97BD
	vdivu.sv.1	v32, r23, v34		#											= 0x822BB7BD
	vdivu.vs	v32, v34, r23		#											= 0x822B9FBD
	vdivu.vs.1	v32, v34, r23		#											= 0x822BBFBD

	vsra.vv		v0, v1, v2			#000000 000001 000010 0 0 010000 111101		= 0x0010843D
	vsra.vv.1	v0, v1, v2			#											= 0x0010A43D
	vsra.sv		v0, r23, v32		#000000 100000 10111 0 0 1 010000 111101	= 0x020B943D
	vsra.sv.1	v0, r23, v32		#											= 0x020BB43D
	vsra.vs		v0, v32, r23		#											= 0x020B9C3D
	vsra.vs.1	v0, v32, r23		#											= 0x020BBC3D

	vmin.vv		v10, v11, v12		#001010 001011 001100 0 0 100101 111101		= 0x28B3097D
	vmin.vv.1	v10, v11, v12		#											= 0x28B3297D
	vmin.sv		v10, r22, v12		#001010 001100 10110 0 0 1 100101 111101	= 0x28CB197D
	vmin.sv.1	v10, r22, v12		#											= 0x28CB397D
	vminu.vv	v10, v11, v12		#											= 0x28B30B7D
	vminu.vv.1	v10, v11, v12		#											= 0x28B32B7D
	vminu.sv	v10, r22, v12		#											= 0x28CB1B7D
	vminu.sv.1	v10, r22, v12		#											= 0x28CB3B7D

	vmax.vv		v10, v11, v12		#001010 001011 001100 0 0 100001 111101		= 0x28B3087D
	vmax.vv.1	v10, v11, v12		#											= 0x28B3287D
	vmax.sv		v10, r22, v12		#											= 0x28CB187D
	vmax.sv.1	v10, r22, v12		#											= 0x28CB387D
	vmaxu.vv	v10, v11, v12		#001010 001011 001100 0 0 101001 111101		= 0x28B30A7D
	vmaxu.vv.1	v10, v11, v12		#											= 0x28B32A7D
	vmaxu.sv	v10, r22, v12		#											= 0x28CB1A7D
	vmaxu.sv.1	v10, r22, v12		#											= 0x28CB3A7D

	vcmpe.vv	vfmask0, v11, v12	#000000 001011 001100 0 0 010001 111101		= 0x00B3047D
	vcmpe.vv.1	vfmask0, v11, v12	#											= 0x00B3247D
	vcmpe.sv	vfmask0, r22, v12	#000000 001100 10110 0 0 1 010001 111101	= 0x00CB147D
	vcmpe.sv.1	vfmask0, r22, v12	#											= 0x00CB347D
	vcmpne.vv	vfmask1, v11, v12	#											= 0x04B3067D
	vcmpne.vv.1	vfmask1, v11, v12	#											= 0x04B3267D
	vcmpne.sv	vfmask1, r22, v12	#											= 0x04CB167D
	vcmpne.sv.1	vfmask1, r22, v12	#											= 0x04CB367D

	vcmplt.vv		vfmask0, v11, v12#000000 001011 001100 0 0 010101 111101		= 0x00B3057D
	vcmplt.vv.1		vfmask0, v11, v12#											= 0x00B3257D
	vcmplt.sv		vfmask0, r22, v12#											= 0x00CB157D
	vcmplt.sv.1		vfmask0, r22, v12#											= 0x00CB357D
	vcmpltu.vv		vfmask1, v11, v12#000001 001011 001100 0 0 011101 111101		= 0x04B3077D
	vcmpltu.vv.1	vfmask1, v11, v12#											= 0x04B3277D
	vcmpltu.sv		vfmask1, r22, v12#											= 0x04CB177D
	vcmpltu.sv.1	vfmask1, r22, v12#											= 0x04CB377D

	vcmple.vv		vfmask0, v62, v63#000000 111110 111111 0 0 010111 111101		= 0x03EFC5FD
	vcmple.vv.1		vfmask0, v62, v63#											= 0x03EFE5FD
	vcmple.sv		vfmask0, r24, v63#000000 111111 11000 0 0 1 010111 111101	= 0x03FC15FD
	vcmple.sv.1		vfmask0, r24, v63#											= 0x03FC35FD
	vcmple.vs		vfmask0, v63, r24#000000 111111 11000 0 0 1 110111 111101	= 0x03FC1DFD
	vcmple.vs.1		vfmask0, v63, r24#											= 0x03FC3DFD
	vcmpleu.vv		vfmask0, v62, v63#											= 0x03EFC7FD
	vcmpleu.vv.1	vfmask0, v62, v63#											= 0x03EFE7FD
	vcmpleu.sv		vfmask0, r24, v63#											= 0x03FC17FD
	vcmpleu.sv.1	vfmask0, r24, v63#											= 0x03FC37FD
	vcmpleu.vs		vfmask0, v63, r24#											= 0x03FC1FFD
	vcmpleu.vs.1	vfmask0, v63, r24#											= 0x03FC3FFD

	vmacc.vv		v31, v32		#000000 011111 100000 0 0 000011 111101		= 0x01F800FD
	vmacc.vv.1		v31, v32		#											= 0x01F820FD
	vmacc.sv		r24, v32		#000000 100000 11000 0 0 1 000011 111101	= 0x020C10FD
	vmacc.sv.1		r24, v32		#											= 0x020C30FD
	vmaccu.vv		v31, v32		#											= 0x01F802FD
	vmaccu.vv.1		v31, v32		#											= 0x01F822FD
	vmaccu.sv		r24, v32		#											= 0x020C12FD
	vmaccu.sv.1		r24, v32		#											= 0x020C32FD

	vccacc		v5					#000101 000000 000000 0 0 110000 111101		= 0x14000C3D
	vccacc.1	v5					#											= 0x14002C3D

	vzacc							#000000 000000 000000 0 0 110001 111101		= 0x00000C7D

	vand.vv		v15, v16, v17		#001111 010000 010001 0 0 000100 111101		= 0x3D04413D
	vand.vv.1	v15, v16, v17		#											= 0x3D04613D
	vand.sv		v15, r21, v17		#001111 010001 10101 0 0 1 000100 111101	= 0x3D1A913D
	vand.sv.1	v15, r21, v17		#											= 0x3D1AB13D

	vor.vv		v15, v16, v17		#											= 0x3D0441BD
	vor.vv.1	v15, v16, v17		#											= 0x3D0461BD
	vor.sv		v15, r21, v17		#001111 010001 10101 0 0 1 000110 111101	= 0x3D1A91BD
	vor.sv.1	v15, r21, v17		#											= 0x3D1AB1BD

	vxor.vv		v15, v16, v17		#											= 0x3D0441FD
	vxor.vv.1	v15, v16, v17		#											= 0x3D0461FD
	vxor.sv		v15, r21, v17		#											= 0x3D1A91FD
	vxor.sv.1	v15, r21, v17		#											= 0x3D1AB1FD

	vsll.vv		v60, v61, v62		#111100 111101 111110 0 0 010010 111101		= 0xF3DF84BD
	vsll.vv.1	v60, v61, v62		#											= 0xF3DFA4BD
	vsll.sv		v60, r20, v62		#111100 111110 10100 0 0 1 010010 111101	= 0xF3EA14BD
	vsll.sv.1	v60, r20, v62		#											= 0xF3EA34BD
	vsll.vs		v60, v62, r20		#											= 0xF3EA1CBD
	vsll.vs.1	v60, v62, r20		#											= 0xF3EA3CBD

	vsrl.vv		v60, v61, v62		#											= 0xF3DF84FD
	vsrl.vv.1	v60, v61, v62		#											= 0xF3DFA4FD
	vsrl.sv		v60, r20, v62		#											= 0xF3EA14FD
	vsrl.sv.1	v60, r20, v62		#											= 0xF3EA34FD
	vsrl.vs		v60, v62, r20		#											= 0xF3EA1CFD
	vsrl.vs.1	v60, v62, r20		#											= 0xF3EA3CFD

	vrot.vv		v60, v61, v62		#											= 0xF3DF853D
	vrot.vv.1	v60, v61, v62		#											= 0xF3DFA53D
	vrot.sv		v60, r20, v62		#											= 0xF3EA153D
	vrot.sv.1	v60, r20, v62		#											= 0xF3EA353D
	vrot.vs		v60, v62, r20		#											= 0xF3EA1D3D
	vrot.vs.1	v60, v62, r20		#											= 0xF3EA3D3D

	vcperm		v20, v21, v22		#010100 010101 010110 0 0 111010 111101		= 0x51558EBD

									#[.b/.h/.w] [.1] vD, vbase, [vinc]
	vld.b		v15, vbase31,		#001111 000 0000 11111 0 0 000000 111111	= 0x3C07C03F
	vld.b.1		v15, vbase31, vinc0	#											= 0x3C07E03F
	vld.h		v15, vbase31,		#											= 0x3C07C43F
	vld.h.1		v15, vbase31, vinc0	#											= 0x3C07E43F
	vld.w		v15, vbase31,		#											= 0x3C07C83F
	vld.w.1		v15, vbase31, vinc0	#											= 0x3C07E83F
	vldu.b		v15, vbase31,		#											= 0x3C07C23F
	vldu.b.1	v15, vbase31, vinc0	#											= 0x3C07E23F
	vldu.h		v15, vbase31,		#											= 0x3C07C63F
	vldu.h.1	v15, vbase31, vinc0	#											= 0x3C07E63F

									#[.b/.h/.w] [.1] vA, vbase, [vinc]
	vst.b		v15, vbase31,		#001111 000 0000 11111 0 0 000001 111111	= 0x3C07C07F
	vst.b.1		v15, vbase31, vinc0	#											= 0x3C07E07F
	vst.h		v15, vbase31,		#											= 0x3C07C47F
	vst.h.1		v15, vbase31, vinc0	#											= 0x3C07E47F
	vst.w		v15, vbase31,		#											= 0x3C07C87F
	vst.w.1		v15, vbase31, vinc0	#											= 0x3C07E87F

											#[.b/.h/.w] [.1] vD, [vbase], [vstride], [vinc]
	vlds.b	  v15, vbase31, vstride4, vinc0	#001111 000 0100 11111 0 0 000010 111111 = 0x3C27C0BF
	vlds.b.1  v15, vbase31, vstride4, vinc0	#										 = 0x3C27E0BF
	vlds.h	  v15, vbase31, vstride4, vinc0	#001111 000 0100 11111 0 0 010010 111111 = 0x3C27C4BF
	vlds.h.1  v15, vbase31, vstride4, vinc0	#										 = 0x3C27E4BF
	vlds.w	  v15, vbase31, vstride4, vinc0	#001111 000 0100 11111 0 0 100010 111111 = 0x3C27C8BF
	vlds.w.1  v15, vbase31, vstride4, vinc0	#										 = 0x3C27E8BF
	vldsu.b	  v15, vbase31, vstride4, vinc0	#001111 000 0100 11111 0 0 001010 111111 = 0x3C27C2BF
	vldsu.b.1 v15, vbase31, vstride4, vinc0	#										 = 0x3C27E2BF
	vldsu.h	  v15, vbase31, vstride4, vinc0	#001111 000 0100 11111 0 0 011010 111111 = 0x3C27C6BF
	vldsu.h.1 v15, vbase31, vstride4, vinc0	#										 = 0x3C27E6BF
	
											#[.b/.h/.w] [.1] vA, [vbase], [vstride], [vinc]
	vsts.b	  v15, vbase31, vstride4, vinc0	#001111 000 0100 11111 0 0 000011 111111 = 0x3C27C0FF
	vsts.b.1  v15, vbase31, vstride4, vinc0	#										 = 0x3C27E0FF
	vsts.h	  v15, vbase31, vstride4, vinc0	#001111 000 0100 11111 0 0 010011 111111 = 0x3C27C4FF
	vsts.h.1  v15, vbase31, vstride4, vinc0	#										 = 0x3C27E4FF
	vsts.w	  v15, vbase31, vstride4, vinc0	#001111 000 0100 11111 0 0 100011 111111 = 0x3C27C8FF
	vsts.w.1  v15, vbase31, vstride4, vinc0	#										 = 0x3C27E8FF
	
									#[.b/.h/.w] [.1] vD, vA, vbase
	vldx.b		v32, v33, vbase0	#100000 100001 0 00000 0 0 000100 111111	= 0x8210013F
	vldx.b.1	v32, v33, vbase0	#											= 0x8210213F
	vldx.h		v32, v33, vbase0	#											= 0x8210053F
	vldx.h.1	v32, v33, vbase0	#											= 0x8210253F
	vldx.w		v32, v33, vbase0	#											= 0x8210093F
	vldx.w.1	v32, v33, vbase0	#											= 0x8210293F
	vldxu.b		v32, v33, vbase0	#											= 0x8210033F
	vldxu.b.1	v32, v33, vbase0	#											= 0x8210233F
	vldxu.h		v32, v33, vbase0	#											= 0x8210073F
	vldxu.h.1	v32, v33, vbase0	#											= 0x8210273F

									#[.b/.h/.w] [.1] vA, vB, vbase
	vstx.b		v32, v33, vbase2	#100001 100000 0 00010 0 0 000101 111111	= 0x8600817F
	vstx.b.1	v32, v33, vbase2	#											= 0x8600A17F
	vstx.h		v32, v33, vbase2	#											= 0x8600857F
	vstx.h.1	v32, v33, vbase2	#											= 0x8600A57F
	vstx.w		v32, v33, vbase2	#											= 0x8600897F
	vstx.w.1	v32, v33, vbase2	#											= 0x8600A97F
								
	vstxo.b		v32, v33, vbase2	#											= 0x860081BF
	vstxo.b.1	v32, v33, vbase2	#											= 0x8600A1BF
	vstxo.h		v32, v33, vbase2	#											= 0x860085BF
	vstxo.h.1	v32, v33, vbase2	#											= 0x8600A5BF
	vstxo.w		v32, v33, vbase2	#											= 0x860089BF
	vstxo.w.1	v32, v33, vbase2	#											= 0x8600A9BF
	
									#[.1] vD, vA	OP = 110000
	vldl		v32, v33			#100000 100001 000000 0 0 110000 111111		= 0x82100C3F
	vldl.1		v32, v33			#											= 0x82102C3F

	vstl.vv		v32, v33			#000000 100000 100001 0 0 110001 111111		= 0x02084C7F
	vstl.vv.1	v32, v33			#											= 0x02086C7F
	vstl.vs		v32, r23			#000000 100000 10111 0 0 0 111001 111111	= 0x020B8E7F
	vstl.vs.1	v32, r23			#											= 0x020BAE7F
	
	vfld	vfmask1, vbase0, vinc0	#000001 000 0000 00000 0 0 110010 111111	= 0x04000CBF
	
	vfst	vfmask1, vbase0,		#000001 000 0000 00000 0 0 110011 111111	= 0x04000CFF

	vmerge.vv	v32, v33, v34		#100000 100001 100010 0 0 011000 111101		= 0x8218863D
	vmerge.vv.1	v32, v33, v34		#											= 0x8218A63D
	vmerge.sv	v32, r22, v34		#100000 100010 10110 0 0 1 011000 111101	= 0x822B163D
	vmerge.sv.1	v32, r22, v34		#											= 0x822B363D
	vmerge.vs	v32, v34, r22		#											= 0x822B1E3D
	vmerge.vs.1	v32, v34, r22		#											= 0x822B3E3D

	vins.vv		v32, v33			#100000 100001 000000 0 0 100011 111101		= 0x821008FD
	vins.sv		v32, r23			#100000 000000 10111 0 0 1 100011 111101	= 0x800B98FD
	#Check Vext Code??
	vext.vv		v22, v23			#010110 010111 000000 0 0 100010 111101		= 0x597008BD
	vext.sv		r9, v23				#01001 0 010111 000000 0 1 100010 111101	= 0x497018BD
	vextu.sv	r9, v23				#01001 0 010111 000000 0 1 101010 111101	= 0x49701ABD

	vcompress	v22, v23			#010110 010111 000000 0 0 110010 111101		= 0x59700CBD
	vcompress.1 v22, v23			#											= 0x59702CBD

	vexpand		v22, v23			#010110 010111 000000 0 0 110011 111101		= 0x59700CFD
	vexpand.1	v22, v23			#											= 0x59702CFD

	veshift		v22, v23			#010110 010111 000000 0 0 111000 111101		= 0x59700E3D

	vexthalf	v22, v23			#010110 010111 000000 0 0 111001 111101		= 0x59700E7D

	vfins	vfgr27, r12				#011101 000000 01100 0 0 1 011010 111110	= 0x740616BE
	vfand	vfgr20, vfgr20, vfgr21	#010110 010110 010111 0 1 000100 111110		= 0x5965D13E
	vfor	vfmask0, vfmask1, vfgr0	#000000 000001 000010 0 1 000110 111110		= 0x001091BE
	vfxor	vfmask1, vfgr10, vfgr11	#000001 001100 001101 0 1 000111 111110		= 0x04C351FE
	vfnor	vfgr12, vfgr13, vfgr14	#001110 001111 010000 0 1 000101 111110		= 0x38F4117E
	vfclr	vfmask1					#000001 000000 000000 0 1 000000 111110		= 0x0400103E
	vfset	vfmask1					#000001	000000 000000 0 1 000001 111110		= 0x0400107E

	vmstc	vbase0, r22				#000000 000000 10110 0 0 1 101000 111110	= 0x000B1A3E
	vmcts	r23, vbase1				#10111 0 000001 000000 0 1 101001 111110	= 0xB8101A7E
	vsync							#000000 000000 000000 0 1 100000 111110		= 0x0000183E

	.size	main, .-main
	.ident	"GCC: (GNU) 3.4.1 (Altera Nios II 6.1 b197)"

