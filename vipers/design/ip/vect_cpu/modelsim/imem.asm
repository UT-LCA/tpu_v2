#--------------------------------------------------------
# Test suite: Local Memory
#--------------------------------------------------------
.set noat # turn off assembler optimization warnings
addi		r31, r0, 14
vmstc		VL, r31

vmulhi.vv		v21, v1, v2
vmulhiu.vv.1	v22, v1, v2
vmulhi.vs		v23, v1, r3		# r3=0x00000002
vmullo.vv.1		v24, v1, v2
vmullo.vs		v25, v1, r3		# r3=0x00000002
vmullou.vv.1	v26, v1, v2
