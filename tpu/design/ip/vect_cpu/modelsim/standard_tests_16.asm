#--------------------------------------------------------
# Test suite 1: Signed arithmetic instructions
#--------------------------------------------------------
.set noat # turn off assembler optimization warnings
# change vector length
addi	r31, r0, 33					# set vector length
vmstc	VL, r31

vadd.vv		v3, v1, v2
vadd.vs.1	v4, v2, r1		# r1=0x00000001
vsub.vv		v5, v1, v2
vsub.vs.1	v6, v1, r1		# r1=0x00000001
vsub.sv		v7, r3, v1		# r3=0x00000002

vabs 		v8, v7
vabs.1 		v9, v7
vabsdiff.vv		v10, v1, v7
vabsdiff.vv.1	v11, v7, v1
vabsdiff.vs		v12, v2, r1

vmin.vv		v13, v1, v7
vmin.vs.1	v14, v1, r4		# r4=0x00000004
vmax.vv.1	v15, v1, v7
vmax.vs		v16, v12, r4	# r4=0x00000004

vmerge.vv	v17, v1, v7
vmerge.vv.1	v18, v1, v7
vmerge.vs.1	v19, v1, r1
vmerge.sv.1	v20, r1, v1


#	vmulhi.vv		v0, v20, v21
#	vmulhi.vv.1		v0, v20, v21
#	vmulhi.sv		v0, r23, v21
#	vmulhi.sv.1		v0, r23, v21
#	vmulhiu.vv		v0, v20, v21
#	vmulhiu.vv.1	v0, v20, v21
#	vmulhiu.sv		v0, r23, v21
#	vmulhiu.sv.1	v0, r23, v21
#	vmullo.vv		v0, v20, v21
#	vmullo.vv.1		v0, v20, v21
#	vmullo.sv		v0, r23, v21
#	vmullo.sv.1		v0, r23, v21
#	vmullou.vv		v0, v20, v21
#	vmullou.vv.1	v0, v20, v21
#	vmullou.sv		v0, r23, v21
#	vmullou.sv.1	v0, r23, v21

#--------------------------------------------------------
# Test suite 2: Logical instructions
#--------------------------------------------------------
.set noat # turn off assembler optimization warnings
# change vector length
addi	r31, r0, 48					# set vector length
vmstc	VL, r31

vsll.vv.1	v3, v1, v2
vsll.sv		v4, r1, v1		# r1=0x00000001
vsll.vs		v5, v1, r3		# r3=0x00000002

vsub.sv		v6, r11, v0		# r11=0xc0000000
vsra.vv		v7, v2, v6
vsra.sv.1	v8, r1, v6		# r1=0x00000001
vsra.vs		v9, v2, r12		# r12=0x40000000
vsrl.vv		v10, v1, v6
vsrl.sv.1	v11, r1, v1		# r1=0x00000001
vsrl.vs		v12, v1, r11	# r11=0xC0000000
vrot.vv.1	v13, v1, v2
vrot.sv.1	v14, r1, v1
vrot.vs		v15, v1, r3		# r3=0x00000002

vand.vv.1	v16, v1, v0
vand.vs		v17, v1, r1
vand.vs.1	v18, v1, r0		# r0=0x00000000
vor.vv.1	v19, v1, v8
vor.vs		v20, v0, r1
vxor.vv.1	v21, v1, v1
vxor.vs		v22, v1, r1


#--------------------------------------------------------
# Test suite 3: Vector comparison, flag logical instructions
#--------------------------------------------------------
.set noat # turn off assembler optimization warnings
# change vector length
addi	r31, r0, 56					# set vector length
vmstc	VL, r31
vsub.vs		v5, v1, r1

# change vector length
addi	r31, r0, 16					# set vector length
vmstc	VL, r31
vadd.vs		v5, v5, r1

# change vector length
addi	r31, r0, 56					# set vector length
vmstc	VL, r31

vcmpe.vv		vfgr0, v1, v5
vcmpe.vs		vfgr1, v1, r3		# r3=0x00000002
vcmpne.vv.1		vfgr2, v1, v2
vcmpne.vs		vfgr3, v1, r3

vcmplt.vv		vfgr4, v5, v1		# v5 less than v1
vcmplt.sv		vfgr5, r7, v5		# r7=0x00000010
vcmplt.vs.1		vfgr6, v5, r7		# r7=0x00000010

vcmple.vv		vfgr7, v1, v5
vcmple.sv		vfgr8, r7, v5		# r7=0x00000010
vcmple.vs		vfgr9, v5, r7		# r7=0x00000010


# Vector flag logical
#addi		r31, r0, #14
#vmstc		VL, r31

vfand.vv		vfgr10, vfmask1, vfmask1
vfor.vv			vfgr11, vfmask0, vfmask1
vfxor.vv		vfgr12, vfmask1, vfmask1
vfnor.vv		vfgr13, vfmask1, vfmask0
vfand.vs		vfgr14, vfmask1, r0
vfor.vs			vfgr15, vfmask1, r1
vfxor.vs		vfgr16, vfmask1, r1
vfnor.vs		vfgr17, vfmask1, r1


#--------------------------------------------------------
# Unsigned arithmetic instructions
#--------------------------------------------------------
.set noat # turn off assembler optimization warnings
vaddu.vv	v5, v1, v2
vaddu.vs.1	v6, v2, r3		# r3=0x00000002
vsubu.vv	v10, v1, v3		# r3=0x00000002
vsubu.vs.1	v11, v1, r3
vsubu.vs	v12, v1, r3		# r3=0x00000002

#	vminu.vv	v10, v11, v12	
#	vminu.vv.1	v10, v11, v12	
#	vminu.sv	v10, r22, v12	
#	vminu.sv.1	v10, r22, v12	

#	vmaxu.vv	v10, v11, v12	
#	vmaxu.vv.1	v10, v11, v12	
#	vmaxu.sv	v10, r22, v12	
#	vmaxu.sv.1	v10, r22, v12	

vcmpltu.vv		vfmask1, v11, v12
vcmpltu.vv.1	vfmask1, v11, v12
vcmpltu.vs		vfmask1, v12, r22
vcmpltu.vs.1	vfmask1, v12, r22
vcmpltu.sv		vfmask1, r22, v12
vcmpleu.vv		vfmask0, v62, v63
vcmpleu.vv.1	vfmask0, v62, v63
vcmpleu.sv		vfmask0, r24, v63
vcmpleu.sv.1	vfmask0, r24, v63
vcmpleu.vs		vfmask0, v63, r24
vcmpleu.vs.1	vfmask0, v63, r24



#--------------------------------------------------------
# Test suite: Vector loads
#--------------------------------------------------------
.set noat # turn off assembler optimization warnings
# change vector length
addi	r31, r0, 49					# set vector length
vmstc	VL, r31

ldw		r4, 0(r7)		# addr=0x00000010
ldh		r5, 0(r8)		# addr=0x00000016
ldb		r6, 0(r9)		# addr=0x00000019

vld.w	v1, vbase1, vinc0			# base=0x04, inc=0
vlds.w	v2, vbase2, vstride3, vinc0	# base=0x08, stride=3, inc=0
vld.h	v3, vbase5, vinc0			# base=0x12, inc=0
vldsu.h	v4, vbase8, vstride2, vinc0	# base=0x1A, stride=2, inc=0
vldu.b	v5, vbase9, vinc0			# base=0x1B, inc=0
vlds.b	v6, vbase15, vstride5, vinc0	# base=0x21, stride=5, inc=0
vlds.w	v7, vbase14, vstride7, vinc0	# base=0x20, stride=20, inc=0


# change vector length
#addi	r5, r0, 17
#vmstc	VL, r5

# Shift offsets for indexed load
vsll.sv		v33, r2, v32			# shift v32 by r2=1
vsll.sv		v34, r3, v32			# shift v32 by r3=2

# Indexed load
vldx.w		v8, v34, vbase2		# base=0x00000008
vldx.w.1	v9, v34, vbase2		# base=0x00000008
vldx.h		v10, v33, vbase5		# base=0x00000012
vldx.h.1	v11, v33, vbase5		# base=0x00000012
vldxu.h		v12, v33, vbase5		# base=0x00000012
vldxu.h.1	v13, v33, vbase5		# base=0x00000012
vldx.b		v14, v32, vbase13		# base=0x0000001F
vldx.b.1	v15, v32, vbase13		# base=0x0000001F
vldxu.b		v16, v32, vbase13		# base=0x0000001F
vldxu.b.1	v17, v32, vbase13		# base=0x0000001F

# change vector length
addi	r31, r0, 56
vmstc	VL, r31

# Vector flag load
vfld		vfgr0, vbase1, vinc0	# base=0x00000004


#--------------------------------------------------------
# Test suite: Vector stores
#--------------------------------------------------------
.set noat # turn off assembler optimization warnings
# change vector length
addi	r31, r0, 17					# set vector length
vmstc	VL, r31

# Scalar stores
stw		r2, 0(r7)		# data=0x00000001, addr=0x00000010
sth		r2, 0(r8)		# data=0x00000001, addr=0x00000016
stb		r2, 0(r9)		# data=0x00000001, addr=0x00000019


# Okay to test them altogether if only analyzing trace
# the instructions will overwrite each other's results
vst.w		v1, vbase1, vinc5				# base=0x04, inc=0x20
vsts.w.1	v1, vbase1, vstride7, vinc0		# base=0x24, stride=0x14
vst.h		v1, vbase3, vinc0				# base=0x0c
vsts.h		v1, vbase5, vstride4, vinc0		# base=0x12, stride=4
vst.b		v1, vbase13, vinc0
vst.b.1		v1, vbase13, vinc0				# base=0x1f
vsts.b		v1, vbase5, vstride3, vinc0		# base=0x12, stride=3

# change vector length
addi	r31, r0, 32					# set vector length
vmstc	VL, r31
vst.w		v1, vbase1, vinc5				# base=0x24 (after inc), inc=0x20
vsts.w.1	v1, vbase1, vstride7, vinc0		# base=0x44 (after inc), stride=0x14

# change vector length
addi	r31, r0, 49					# set vector length
vmstc	VL, r31
vst.w		v1, vbase1, vinc5				# base=0x64 (after inc), inc=0x20
vsts.w.1	v1, vbase1, vstride7, vinc0		# base=0x84 (after inc), stride=0x14


# change vector length
addi	r31, r0, 8
vmstc	VL, r31

# shift offsets for indexed load
vsll.sv		v33, r2, v32			# shift v32 by r2=1
vsll.sv		v34, r3, v32			# shift v32 by r3=2

# Indexed store
vstx.w		v1, v34, vbase3		# base=0x0000000C
vstx.w.1	v1, v34, vbase14	# base=0x00000020
vstx.h		v1, v33, vbase12	# base=0x0000001E
vstx.h.1	v1, v33, vbase12	# base=0x0000001E
vstx.b		v1, v32, vbase13	# base=0x0000001F
vstx.b.1	v1, v32, vbase13	# base=0x0000001F

# Vector flag store
vfst		vfmask1, vbase4, vinc1	# base=0x00000010, inc=4

addi	r31, r0, 8
vmstc	VL, r31
vfst		vfmask1, vbase4, vinc1	# base=0x00000010, inc=4
addi	r31, r0, 49
vmstc	VL, r31
vfst		vfmask1, vbase4, vinc1	# base=0x00000010, inc=4
addi	r31, r0, 32
vmstc	VL, r31
vfst		vfmask1, vbase4, vinc1	# base=0x00000010, inc=4


#--------------------------------------------------------
# Test suite: Vector vext/vins, vupshift
#--------------------------------------------------------
.set noat # turn off assembler optimization warnings
# change vector length
addi		r31, r0, 30
vmstc		VL, r31
addi		r31, r0, 11
vmstc		vindex, r31

# vext.vv
vext.vv		v2, v1
# vins.vv
vins.vv		v3, v1

# change vector length
addi		r31, r0, 14
vmstc		VL, r31
addi		r31, r0, 8
vmstc		vindex, r31
vext.vv		v4, v1
vins.vv		v5, v1


# change vector length
addi		r31, r0, 16
vmstc		VL, r31
addi		r31, r0, 16
vmstc		vindex, r31
vext.vv		v6, v1
# test if executing an instruction before manip finishes will affect anything
vadd		v15, v13, v14
vins.vv		v7, v1


# test if vmstc stalls until vins.vv is complete
addi		r31, r0, 32
vmstc		VL, r31

# vins.vs, vext.vs
vins.vs		v8, r10		# r10=0x80000000
vext.vs		r30, v1
vextu.vs	r31, v1

# shifts entire vector
vupshift	v9, v1


#--------------------------------------------------------
# Test suite: Local Memory
#--------------------------------------------------------
.set noat # turn off assembler optimization warnings
addi		r31, r0, 14
vmstc		VL, r31

vstl.vs		v1, r10		# r10=0x80000000
vldl.vv		v3, v1
vldl.vv.1	v4, v1

vadd.vs		v2, v0, r1
vstl.vv.1	v2, v2
vldl.vv		v5, v2


#--------------------------------------------------------
# Test suite: VMAC, vcczacc, vccacc
#--------------------------------------------------------
.set noat # turn off assembler optimization warnings
# change vector length
addi		r31, r0, 16
addi		r8, r0, 1
vmstc		VL, r31
vmac.1		v31, v32
vabsdiff		v3, v1, v2
vmac.1		v31, v32
vcczacc		v4

vmstc		VL, r31		# reset VL
vmac	v31, v32
vccacc		v5
vmstc		VL, r31		# reset VL
vmac	v31, v32
vcczacc		v6

# test accumulating with a reduction chain that doesn't span all lanes (set MACL to 0 to test)
vmstc		VL, r31		# reset VL
vadd.vs	v30, v0, r8
vmac	v31, v32
vcczacc		v7
vmac	v30, v7
vcczacc		v8
vmac	v30, v8
vcczacc		v9

vmstc		VL, r31		# reset VL


#--------------------------------------------------------
# Test suite: read after write hazard
#--------------------------------------------------------
.set noat # turn off assembler optimization warnings
# change vector length
addi		r31, r0, 4
vmstc		VL, r31

# RAW test: register RAW
vadd	v3, v1, v2
vadd	v4, v1, v3

# RAW test: register write -> store
vadd	v3, v1, v2
vst.w	v3, vbase0, vinc0

# RAW test: using data from vins
addi		r31, r0, 3
vmstc		vindex, r31
vins.vs		v5, r7
vadd		v6, v5, v1

# RAW test: flag write -> flag store
addi		r31, r0, 32
vmstc		VL, r31
vfor		vfgr0, vfmask0, vfmask1
vfst		vfgr0, vbase0, vinc0

# indexed load followed by vext.vs
# both use the same MUX to read lane elements
addi		r31, r0, 3
vmstc		vindex, r31
addi		r31, r0, 4
vmstc		VL, r31
vsll.sv		v34, r3, v32			# shift v32 by r3=2

vstx.w		v1, v34, vbase3			# base=0x0000000C
vadd.vv		v3, v1, v2

vldx.w		v8, v34, vbase14		# base=0x00000020
vext.vs		r30, v1

# indexed load after vext.vs
vldx.w		v9, v34, vbase14		# base=0x00000020
