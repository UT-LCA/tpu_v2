GAS LISTING v_assembler_test.s 			page 1


   1              		.file	"hello_world.c"
   2              		.section	.text
   3              		.align	2
   4              	main:
   5 0000 020010B4 		movhi	r8, %hiadj(4320000)
   6 0004 423AC004 		addi	r8, r8, %lo(4320000)
   7 0008 00800774 		movhi	r2, %hiadj(1920000)
   8 000c 10930004 		addi	r2, r2, %lo(1920000)
   9 0010 DA37C83A 		sub	sp, sp, r8
  10 0014 D88D883A 		add	r6, sp, r2
  11 0018 01401884 		movi	r5, 98
  12 001c 0005883A 		mov	r2, zero
  13 0020 01001844 		movi	r4, 97
  14              	.L44:
  15 0024 3087883A 		add	r3, r6, r2
  16 0028 D88F883A 		add	r7, sp, r2
  17 002c 39400005 		stb	r5, 0(r7)
  18 0030 19000005 		stb	r4, 0(r3)
  19 0034 00800084 		movi	r2, 2
  20 0038 00000106 		br	VECTOR
  21 003c 003FF906 		br	.L44						
  22              	VECTOR:								#Expected Output for each Instruction
  23              										#A=1010 B=1011 C=1100 D=1101 E=1110 F=1111
  24 0040 00100D7D 		vabs v0, v1						#											= 0x00100D7D
  25 0044 00102D7D 		vabs.1 v0, v1					#											= 0x00102D7D
  26              	
  27 0048 0010817D 		vabsdiff.vv		v0, v1, v2		#0000 0000 0001 0000 1000 0001 0111 1101	= 0x0010817D
  28 004c 0010A17D 		vabsdiff.vv.1	v0, v1, v2		#											= 0x0010A17D
  29 0050 0C28917D 		vabsdiff.vs		v3, v2, r17 	#0000 1100 0010 1000 1001 0001 0111 1101	= 0x0C28917D
  30 0054 0C28B17D 		vabsdiff.vs.1	v3, v2, r17		#0000 1100 0010 1000 1011 0001 0111 1101	= 0x0C28B17D
  31              	
  32 0058 0010803D 		vadd.vv		v0, v1, v2			#000000 000001 000010 0 0 000000 111101		= 0x0010803D
  33 005c 0010A03D 		vadd.vv.1	v0, v1, v2			#000000 000001 000010 1 0 000000 111101		= 0x0010A03D
  34 0060 1429103D 		vadd.vs		v5, v2, r18			#000101 000010 10010 0 0 1 000000 111101	= 0x1429103D
  35 0064 1429303D 		vadd.vs.1	v5, v2, r18			#000101 000010 10010 0 1 1 000000 111101	= 0x1429303D
  36 0068 0010823D 		vaddu.vv	v0, v1, v2			#000000 000001 000010 0 0 001000 111101		= 0x0010823D
  37 006c 0010A23D 		vaddu.vv.1	v0, v1, v2			#000000 000001 000010 1 0 001000 111101		= 0x0010A23D
  38 0070 1429123D 		vaddu.vs	v5, v2, r18			#000101 000010 10010 0 0 1 001000 111101	= 0x1429123D
  39 0074 1429323D 		vaddu.vs.1	v5, v2, r18			#000101 000010 10010 0 1 1 001000 111101	= 0x1429323D
  40              	
  41 0078 8218807D 		vsub.vv		v32, v33, v34		#100000 100001 100010 0 0 000001 111101		= 0x8218807D
  42 007c 8218A07D 		vsub.vv.1	v32, v33, v34		#100000 100001 100010 1 0 000001 111101		= 0x8218A07D
  43 0080 822B9C7D 		vsub.sv		v32, r23, v34		#100000 100010 10111 0 0 1 110001 111101	= 0x822B9C7D
  44 0084 822BBC7D 		vsub.sv.1	v32, r23, v34		#100000 100010 10111 0 1 1 110001 111101	= 0x822BBC7D
  45 0088 822B907D 		vsub.vs		v32, v34, r23		#100000 100010 10111 0 0 1 000001 111101	= 0x822B907D
  46 008c 822BB07D 		vsub.vs.1	v32, v34, r23		#100000 100010 10111 0 1 1 000001 111101	= 0x822BB07D
  47 0090 8218827D 		vsubu.vv	v32, v33, v34		#100000 100001 100010 0 0 001001 111101		= 0x8218827D
  48 0094 8218A27D 		vsubu.vv.1	v32, v33, v34		#100000 100001 100010 1 0 001001 111101		= 0x8218A27D
  49 0098 822B9E7D 		vsubu.sv	v32, r23, v34		#100000 100010 10111 0 0 1 111001 111101	= 0x822B9E7D
  50 009c 822BBE7D 		vsubu.sv.1	v32, r23, v34		#											= 0x822BBE7D
  51 00a0 822B927D 		vsubu.vs	v32, v34, r23		#100000 100010 10111 0 0 1 001001 111101	= 0x822B927D
  52 00a4 822BB27D 		vsubu.vs.1	v32, v34, r23		#											= 0x822BB27D
  53              	
  54 00a8 014549BD 		vmulhi.vv		v0, v20, v21	#000000 010100 010101 0 0 100110 111101		= 0x014549BD
  55 00ac 014569BD 		vmulhi.vv.1		v0, v20, v21	#000000 010100 010101 1 0 100110 111101		= 0x014569BD
  56 00b0 015B99BD 		vmulhi.vs		v0, v21, r23	#000000 010101 10111 0 0 1 100110 111101	= 0x015B99BD
  57 00b4 015BB9BD 		vmulhi.vs.1		v0, v21, r23	#											= 0x015BB9BD
GAS LISTING v_assembler_test.s 			page 2


  58 00b8 01454BBD 		vmulhiu.vv		v0, v20, v21	#000000	010100 010101 0 0 101110 111101		= 0x01454BBD
  59 00bc 01456BBD 		vmulhiu.vv.1	v0, v20, v21	#											= 0x01456BBD
  60 00c0 015B9BBD 		vmulhiu.vs		v0, v21, r23	#											= 0x015B9BBD
  61 00c4 015BBBBD 		vmulhiu.vs.1	v0, v21, r23	#											= 0x015BBBBD
  62              	
  63 00c8 014549FD 		vmullo.vv		v0, v20, v21	#000000 010100 010101 0 0 100111 111101		= 0x014549FD
  64 00cc 014569FD 		vmullo.vv.1		v0, v20, v21	#											= 0x014569FD
  65 00d0 015B99FD 		vmullo.vs		v0, v21, r23	#											= 0x015B99FD
  66 00d4 015BB9FD 		vmullo.vs.1		v0, v21, r23	#											= 0x015BB9FD
  67 00d8 01454BFD 		vmullou.vv		v0, v20, v21	#000000	010100 010101 0 0 101111 111101		= 0x01454BFD
  68 00dc 01456BFD 		vmullou.vv.1	v0, v20, v21	#											= 0x01456BFD
  69 00e0 015B9BFD 		vmullou.vs		v0, v21, r23	#											= 0x015B9BFD
  70 00e4 015BBBFD 		vmullou.vs.1	v0, v21, r23	#											= 0x015BBBFD
  71              	
  72 00e8 821885BD 		vdiv.vv		v32, v33, v34		#100000 100001 100010 0 0 010110 111101		= 0x821885BD
  73 00ec 8218A5BD 		vdiv.vv.1	v32, v33, v34		#											= 0x8218A5BD
  74 00f0 822B9DBD 		vdiv.sv		v32, r23, v34		#100000 100010 10111 0 0 1 110110 111101	= 0x822B9DBD
  75 00f4 822BBDBD 		vdiv.sv.1	v32, r23, v34		#											= 0x822BBDBD
  76 00f8 822B95BD 		vdiv.vs		v32, v34, r23		#100000 100010 10111 0 0 1 010110 111101	= 0x822B95BD
  77 00fc 822BB5BD 		vdiv.vs.1	v32, v34, r23		#											= 0x822BB5BD
  78 0100 821887BD 		vdivu.vv	v32, v33, v34		#											= 0x821887BD
  79 0104 8218A7BD 		vdivu.vv.1	v32, v33, v34		#											= 0x8218A7BD
  80 0108 822B9FBD 		vdivu.sv	v32, r23, v34		#											= 0x822B9FBD
  81 010c 822BBFBD 		vdivu.sv.1	v32, r23, v34		#											= 0x822BBFBD
  82 0110 822B97BD 		vdivu.vs	v32, v34, r23		#											= 0x822B97BD
  83 0114 822BB7BD 		vdivu.vs.1	v32, v34, r23		#											= 0x822BB7BD
  84              	
  85 0118 0010843D 		vsra.vv		v0, v1, v2			#000000 000001 000010 0 0 010000 111101		= 0x0010843D
  86 011c 0010A43D 		vsra.vv.1	v0, v1, v2			#											= 0x0010A43D
  87 0120 020B9C3D 		vsra.sv		v0, r23, v32		#000000 100000 10111 0 0 1 110000 111101	= 0x020B9C3D
  88 0124 020BBC3D 		vsra.sv.1	v0, r23, v32		#											= 0x020BBC3D
  89 0128 020B943D 		vsra.vs		v0, v32, r23		#											= 0x020B943D
  90 012c 020BB43D 		vsra.vs.1	v0, v32, r23		#											= 0x020BB43D
  91              	
  92 0130 28B3097D 		vmin.vv		v10, v11, v12		#001010 001011 001100 0 0 100101 111101		= 0x28B3097D
  93 0134 28B3297D 		vmin.vv.1	v10, v11, v12		#											= 0x28B3297D
  94 0138 28CB197D 		vmin.vs		v10, v12, r22		#001010 001100 10110 0 0 1 100101 111101	= 0x28CB197D
  95 013c 28CB397D 		vmin.vs.1	v10, v12, r22		#											= 0x28CB397D
  96 0140 28B30B7D 		vminu.vv	v10, v11, v12		#											= 0x28B30B7D
  97 0144 28B32B7D 		vminu.vv.1	v10, v11, v12		#											= 0x28B32B7D
  98 0148 28CB1B7D 		vminu.vs	v10, v12, r22		#											= 0x28CB1B7D
  99 014c 28CB3B7D 		vminu.vs.1	v10, v12, r22		#											= 0x28CB3B7D
 100              	
 101 0150 28B3087D 		vmax.vv		v10, v11, v12		#001010 001011 001100 0 0 100001 111101		= 0x28B3087D
 102 0154 28B3287D 		vmax.vv.1	v10, v11, v12		#											= 0x28B3287D
 103 0158 28CB187D 		vmax.vs		v10, v12, r22		#											= 0x28CB187D
 104 015c 28CB387D 		vmax.vs.1	v10, v12, r22		#											= 0x28CB387D
 105 0160 28B30A7D 		vmaxu.vv	v10, v11, v12		#001010 001011 001100 0 0 101001 111101		= 0x28B30A7D
 106 0164 28B32A7D 		vmaxu.vv.1	v10, v11, v12		#											= 0x28B32A7D
 107 0168 28CB1A7D 		vmaxu.vs	v10, v12, r22		#											= 0x28CB1A7D
 108 016c 28CB3A7D 		vmaxu.vs.1	v10, v12, r22		#											= 0x28CB3A7D
 109              	
 110 0170 00B3047D 		vcmpe.vv	vfmask0, v11, v12	#000000 001011 001100 0 0 010001 111101		= 0x00B3047D
 111 0174 00B3247D 		vcmpe.vv.1	vfmask0, v11, v12	#											= 0x00B3247D
 112 0178 00CB147D 		vcmpe.vs	vfmask0, v12, r22	#000000 001100 10110 0 0 1 010001 111101	= 0x00CB147D
 113 017c 00CB347D 		vcmpe.vs.1	vfmask0, v12, r22	#											= 0x00CB347D
 114 0180 04B3067D 		vcmpne.vv	vfmask1, v11, v12	#											= 0x04B3067D
GAS LISTING v_assembler_test.s 			page 3


 115 0184 04B3267D 		vcmpne.vv.1	vfmask1, v11, v12	#											= 0x04B3267D
 116 0188 04CB167D 		vcmpne.vs	vfmask1, v12, r22	#											= 0x04CB167D
 117 018c 04CB367D 		vcmpne.vs.1	vfmask1, v12, r22	#											= 0x04CB367D
 118              	
 119 0190 00B3057D 		vcmplt.vv		vfmask0, v11, v12#000000 001011 001100 0 0 010101 111101	= 0x00B3057D
 120 0194 00B3257D 		vcmplt.vv.1		vfmask0, v11, v12#											= 0x00B3257D
 121 0198 00CB1D7D 		vcmplt.sv		vfmask0, r22, v12#											= 0x00CB1D7D
 122 019c 00CB3D7D 		vcmplt.sv.1		vfmask0, r22, v12#											= 0x00CB3D7D
 123 01a0 00BB157D 		vcmplt.vs		vfmask0, v11, r22#											= 0x00BB157D
 124 01a4 00BB357D 		vcmplt.vs.1		vfmask0, v11, r22#											= 0x00BB357D
 125 01a8 04B3077D 		vcmpltu.vv		vfmask1, v11, v12#000001 001011 001100 0 0 011101 111101	= 0x04B3077D
 126 01ac 04B3277D 		vcmpltu.vv.1	vfmask1, v11, v12#											= 0x04B3277D
 127 01b0 04CB1F7D 		vcmpltu.sv		vfmask1, r22, v12#											= 0x04CB1F7D
 128 01b4 04CB3F7D 		vcmpltu.sv.1	vfmask1, r22, v12#											= 0x04CB3F7D
 129 01b8 00B6177D 		vcmpltu.vs		vfmask0, v11, r12#											= 0x00B6177D
 130 01bc 00B6377D 		vcmpltu.vs.1	vfmask0, v11, r12#											= 0x00B6377D
 131              	
 132 01c0 03EFC5FD 		vcmple.vv		vfmask0, v62, v63#000000 111110 111111 0 0 010111 111101	= 0x03EFC5FD
 133 01c4 03EFE5FD 		vcmple.vv.1		vfmask0, v62, v63#											= 0x03EFE5FD
 134 01c8 03FC1DFD 		vcmple.sv		vfmask0, r24, v63#000000 111111 11000 0 0 1 110111 111101	= 0x03FC1DFD
 135 01cc 03FC3DFD 		vcmple.sv.1		vfmask0, r24, v63#											= 0x03FC3DFD
 136 01d0 03FC15FD 		vcmple.vs		vfmask0, v63, r24#000000 111111 11000 0 0 1 010111 111101	= 0x03FC15FD
 137 01d4 03FC35FD 		vcmple.vs.1		vfmask0, v63, r24#											= 0x03FC35FD
 138 01d8 03EFC7FD 		vcmpleu.vv		vfmask0, v62, v63#											= 0x03EFC7FD
 139 01dc 03EFE7FD 		vcmpleu.vv.1	vfmask0, v62, v63#											= 0x03EFE7FD
 140 01e0 03FC1FFD 		vcmpleu.sv		vfmask0, r24, v63#											= 0x03FC1FFD
 141 01e4 03FC3FFD 		vcmpleu.sv.1	vfmask0, r24, v63#											= 0x03FC3FFD
 142 01e8 03FC17FD 		vcmpleu.vs		vfmask0, v63, r24#											= 0x03FC17FD
 143 01ec 03FC37FD 		vcmpleu.vs.1	vfmask0, v63, r24#											= 0x03FC37FD
 144              	
 145 01f0 01F800FD 		vmac.vv		v31, v32		#000000 011111 100000 0 0 000011 111101		= 0x01F800FD
 146 01f4 01F820FD 		vmac.vv.1		v31, v32		#											= 0x01F820FD
 147 01f8 020C10FD 		vmac.vs		v32, r24		#000000 100000 11000 0 0 1 000011 111101	= 0x020C10FD
 148 01fc 020C30FD 		vmac.vs.1		v32, r24		#											= 0x020C30FD
 149 0200 01F802FD 		vmacu.vv		v31, v32		#											= 0x01F802FD
 150 0204 01F822FD 		vmacu.vv.1		v31, v32		#											= 0x01F822FD
 151 0208 020C12FD 		vmacu.vs		v32, r24		#											= 0x020C12FD
 152 020c 020C32FD 		vmacu.vs.1		v32, r24		#											= 0x020C32FD
 153              	
 154 0210 14000C3D 		vccacc		v5					#000101 000000 000000 0 0 110000 111101		= 0x14000C3D
 155              	
 156 0214 14000E3D 		vcczacc		v5					#000101 000000 000000 0 0 111000 111101		= 0x14000E3D
 157              	
 158 0218 3D04413D 		vand.vv		v15, v16, v17		#001111 010000 010001 0 0 000100 111101		= 0x3D04413D
 159 021c 3D04613D 		vand.vv.1	v15, v16, v17		#											= 0x3D04613D
 160 0220 3D1A913D 		vand.vs		v15, v17, r21		#001111 010001 10101 0 0 1 000100 111101	= 0x3D1A913D
 161 0224 3D1AB13D 		vand.vs.1	v15, v17, r21		#											= 0x3D1AB13D
 162              	
 163 0228 3D0441BD 		vor.vv		v15, v16, v17		#											= 0x3D0441BD
 164 022c 3D0461BD 		vor.vv.1	v15, v16, v17		#											= 0x3D0461BD
 165 0230 3D1A91BD 		vor.vs		v15, v17, r21		#001111 010001 10101 0 0 1 000110 111101	= 0x3D1A91BD
 166 0234 3D1AB1BD 		vor.vs.1	v15, v17, r21		#											= 0x3D1AB1BD
 167              	
 168 0238 3D0441FD 		vxor.vv		v15, v16, v17		#											= 0x3D0441FD
 169 023c 3D0461FD 		vxor.vv.1	v15, v16, v17		#											= 0x3D0461FD
 170 0240 3D1A91FD 		vxor.vs		v15, v17, r21		#											= 0x3D1A91FD
 171 0244 3D1AB1FD 		vxor.vs.1	v15, v17, r21		#											= 0x3D1AB1FD
GAS LISTING v_assembler_test.s 			page 4


 172              	
 173 0248 F3DF84BD 		vsll.vv		v60, v61, v62		#111100 111101 111110 0 0 010010 111101		= 0xF3DF84BD
 174 024c F3DFA4BD 		vsll.vv.1	v60, v61, v62		#											= 0xF3DFA4BD
 175 0250 F3EA1CBD 		vsll.sv		v60, r20, v62		#111100 111110 10100 0 0 1 110010 111101	= 0xF3EA1CBD
 176 0254 F3EA3CBD 		vsll.sv.1	v60, r20, v62		#											= 0xF3EA3CBD
 177 0258 F3EA14BD 		vsll.vs		v60, v62, r20		#											= 0xF3EA14BD
 178 025c F3EA34BD 		vsll.vs.1	v60, v62, r20		#											= 0xF3EA34BD
 179              	
 180 0260 F3DF84FD 		vsrl.vv		v60, v61, v62		#											= 0xF3DF84FD
 181 0264 F3DFA4FD 		vsrl.vv.1	v60, v61, v62		#											= 0xF3DFA4FD
 182 0268 F3EA1CFD 		vsrl.sv		v60, r20, v62		#											= 0xF3EA1CFD
 183 026c F3EA3CFD 		vsrl.sv.1	v60, r20, v62		#											= 0xF3EA3CFD
 184 0270 F3EA14FD 		vsrl.vs		v60, v62, r20		#											= 0xF3EA14FD
 185 0274 F3EA34FD 		vsrl.vs.1	v60, v62, r20		#											= 0xF3EA34FD
 186              	
 187 0278 F3DF853D 		vrot.vv		v60, v61, v62		#											= 0xF3DF853D
 188 027c F3DFA53D 		vrot.vv.1	v60, v61, v62		#											= 0xF3DFA53D
 189 0280 F3EA1D3D 		vrot.sv		v60, r20, v62		#											= 0xF3EA1D3D
 190 0284 F3EA3D3D 		vrot.sv.1	v60, r20, v62		#											= 0xF3EA3D3D
 191 0288 F3EA153D 		vrot.vs		v60, v62, r20		#											= 0xF3EA153D
 192 028c F3EA353D 		vrot.vs.1	v60, v62, r20		#											= 0xF3EA353D
 193              	
 194              										#[.b/.h/.w] [.1] vD, vbase, [vinc]
 195 0290 3C03C03F 		vld.b		v15, vbase15,		#001111 000 0000 01111 0 0 000000 111111	= 0x3C03C03F
****  Warning:vinc argument left blank, assuming vinc0
 196 0294 3C03E03F 		vld.b.1		v15, vbase15, vinc0	#											= 0x3C03E03F
 197 0298 3C03C43F 		vld.h		v15, vbase15,		#											= 0x3C03C43F
****  Warning:vinc argument left blank, assuming vinc0
 198 029c 3C03E43F 		vld.h.1		v15, vbase15, vinc0	#											= 0x3C03E43F
 199 02a0 3C03C83F 		vld.w		v15, vbase15,		#											= 0x3C03C83F
****  Warning:vinc argument left blank, assuming vinc0
 200 02a4 3C03E83F 		vld.w.1		v15, vbase15, vinc0	#											= 0x3C03E83F
 201 02a8 3C03C23F 		vldu.b		v15, vbase15,		#											= 0x3C03C23F
****  Warning:vinc argument left blank, assuming vinc0
 202 02ac 3C03E23F 		vldu.b.1	v15, vbase15, vinc0	#											= 0x3C03E23F
 203 02b0 3C03C63F 		vldu.h		v15, vbase15,		#											= 0x3C03C63F
****  Warning:vinc argument left blank, assuming vinc0
 204 02b4 3C03E63F 		vldu.h.1	v15, vbase15, vinc0	#											= 0x3C03E63F
 205              	
 206              										#[.b/.h/.w] [.1] vA, vbase, [vinc]
 207 02b8 3C03C07F 		vst.b		v15, vbase15,		#001111 000 0000 11111 0 0 000001 111111	= 0x3C03C07F
****  Warning:vinc argument left blank, assuming vinc0
 208 02bc 3C03E07F 		vst.b.1		v15, vbase15, vinc0	#											= 0x3C03E07F
 209 02c0 3C03C47F 		vst.h		v15, vbase15,		#											= 0x3C03C47F
****  Warning:vinc argument left blank, assuming vinc0
 210 02c4 3C03E47F 		vst.h.1		v15, vbase15, vinc0	#											= 0x3C03E47F
 211 02c8 3C03C87F 		vst.w		v15, vbase15,		#											= 0x3C03C87F
****  Warning:vinc argument left blank, assuming vinc0
 212 02cc 3C03E87F 		vst.w.1		v15, vbase15, vinc0	#											= 0x3C03E87F
 213              	
 214              												#[.b/.h/.w] [.1] vD, [vbase], [vstride], [vinc]
 215 02d0 3C23C0BF 		vlds.b	  v15, vbase15, vstride4, vinc0	#001111 000 0100 01111 0 0 000010 111111 = 0x3C23C0BF
 216 02d4 3C23E0BF 		vlds.b.1  v15, vbase15, vstride4, vinc0	#										 = 0x3C23E0BF
 217 02d8 3C23C4BF 		vlds.h	  v15, vbase15, vstride4, vinc0	#001111 000 0100 01111 0 0 010010 111111 = 0x3C23C4BF
 218 02dc 3C23E4BF 		vlds.h.1  v15, vbase15, vstride4, vinc0	#										 = 0x3C23E4BF
 219 02e0 3C23C8BF 		vlds.w	  v15, vbase15, vstride4, vinc0	#001111 000 0100 01111 0 0 100010 111111 = 0x3C23C8BF
 220 02e4 3C23E8BF 		vlds.w.1  v15, vbase15, vstride4, vinc0	#										 = 0x3C23E8BF
GAS LISTING v_assembler_test.s 			page 5


 221 02e8 3C23C2BF 		vldsu.b	  v15, vbase15, vstride4, vinc0	#001111 000 0100 01111 0 0 001010 111111 = 0x3C23C2BF
 222 02ec 3C23E2BF 		vldsu.b.1 v15, vbase15, vstride4, vinc0	#										 = 0x3C23E2BF
 223 02f0 3C23C6BF 		vldsu.h	  v15, vbase15, vstride4, vinc0	#001111 000 0100 01111 0 0 011010 111111 = 0x3C23C6BF
 224 02f4 3C23E6BF 		vldsu.h.1 v15, vbase15, vstride4, vinc0	#										 = 0x3C23E6BF
 225              		
 226              												#[.b/.h/.w] [.1] vA, [vbase], [vstride], [vinc]
 227 02f8 3C23C0FF 		vsts.b	  v15, vbase15, vstride4, vinc0	#001111 000 0100 01111 0 0 000011 111111 = 0x3C23C0FF
 228 02fc 3C23E0FF 		vsts.b.1  v15, vbase15, vstride4, vinc0	#										 = 0x3C23E0FF
 229 0300 3C23C4FF 		vsts.h	  v15, vbase15, vstride4, vinc0	#001111 000 0100 01111 0 0 010011 111111 = 0x3C23C4FF
 230 0304 3C23E4FF 		vsts.h.1  v15, vbase15, vstride4, vinc0	#										 = 0x3C23E4FF
 231 0308 3C23C8FF 		vsts.w	  v15, vbase15, vstride4, vinc0	#001111 000 0100 01111 0 0 100011 111111 = 0x3C23C8FF
 232 030c 3C23E8FF 		vsts.w.1  v15, vbase15, vstride4, vinc0	#										 = 0x3C23E8FF
 233              		
 234              										#[.b/.h/.w] [.1] vD, voffset, vbase
 235 0310 8210013F 		vldx.b		v32, v33, vbase0	#100000 100001 0 00000 0 0 000100 111111	= 0x8210013F
 236 0314 8210213F 		vldx.b.1	v32, v33, vbase0	#											= 0x8210213F
 237 0318 8210053F 		vldx.h		v32, v33, vbase0	#											= 0x8210053F
 238 031c 8210253F 		vldx.h.1	v32, v33, vbase0	#											= 0x8210253F
 239 0320 8210093F 		vldx.w		v32, v33, vbase0	#											= 0x8210093F
 240 0324 8210293F 		vldx.w.1	v32, v33, vbase0	#											= 0x8210293F
 241 0328 8210033F 		vldxu.b		v32, v33, vbase0	#											= 0x8210033F
 242 032c 8210233F 		vldxu.b.1	v32, v33, vbase0	#											= 0x8210233F
 243 0330 8210073F 		vldxu.h		v32, v33, vbase0	#											= 0x8210073F
 244 0334 8210273F 		vldxu.h.1	v32, v33, vbase0	#											= 0x8210273F
 245              	
 246              										#[.b/.h/.w] [.1] vA, voffset, vbase
 247 0338 8210817F 		vstx.b		v32, v33, vbase2	#100001 100000 0 00010 0 0 000101 111111	= 0x8210817F
 248 033c 8210A17F 		vstx.b.1	v32, v33, vbase2	#											= 0x8210A17F
 249 0340 8210857F 		vstx.h		v32, v33, vbase2	#											= 0x8210857F
 250 0344 8210A57F 		vstx.h.1	v32, v33, vbase2	#											= 0x8210A57F
 251 0348 8210897F 		vstx.w		v32, v33, vbase2	#											= 0x8210897F
 252 034c 8210A97F 		vstx.w.1	v32, v33, vbase2	#											= 0x8210A97F
 253              									
 254              		#vstxo.b		v32, v33, vbase2	#											= 0x821081BF
 255              		#vstxo.b.1	v32, v33, vbase2	#											= 0x8210A1BF
 256              		#vstxo.h		v32, v33, vbase2	#											= 0x821085BF
 257              		#vstxo.h.1	v32, v33, vbase2	#											= 0x8210A5BF
 258              		#vstxo.w		v32, v33, vbase2	#											= 0x821089BF
 259              		#vstxo.w.1	v32, v33, vbase2	#											= 0x8210A9BF
 260              		
 261              										#[.1] vD, vA	OP = 110000
 262 0350 82100C3F 		vldl.vv		v32, v33			#100000 100001 000000 0 0 110000 111111		= 0x82100C3F
 263 0354 82102C3F 		vldl.vv.1	v32, v33			#											= 0x82102C3F
 264 0358 02084C7F 		vstl.vv		v32, v33			#000000 100000 100001 0 0 110001 111111		= 0x02084C7F
 265 035c 02086C7F 		vstl.vv.1	v32, v33			#											= 0x02086C7F
 266 0360 020B9C7F 		vstl.vs		v32, r23			#000000 100000 10111 0 0 1 111001 111111	= 0x020B9E7F
 267 0364 020BBC7F 		vstl.vs.1	v32, r23			#											= 0x020BAE7F
 268              		
 269 0368 04000CBF 		vfld	vfmask1, vbase0, vinc0	#000001 000 0000 00000 0 0 110010 111111	= 0x04000CBF
 270              		
 271 036c 04000CFF 		vfst	vfmask1, vbase0,		#000001 000 0000 00000 0 0 110011 111111	= 0x04000CFF
****  Warning:vinc argument left blank, assuming vinc0
 272              	
 273 0370 8218863D 		vmerge.vv	v32, v33, v34		#100000 100001 100010 0 0 011000 111101		= 0x8218863D
 274 0374 8218A63D 		vmerge.vv.1	v32, v33, v34		#											= 0x8218A63D
 275 0378 822B1E3D 		vmerge.sv	v32, r22, v34		#100000 100010 10110 0 0 1 111000 111101	= 0x822B1E3D
 276 037c 822B3E3D 		vmerge.sv.1	v32, r22, v34		#											= 0x822B3E3D
GAS LISTING v_assembler_test.s 			page 6


 277 0380 822B163D 		vmerge.vs	v32, v34, r22		#											= 0x822B163D
 278 0384 822B363D 		vmerge.vs.1	v32, v34, r22		#											= 0x822B363D
 279              	
 280 0388 821008FD 		vins.vv		v32, v33			#100000 100001 000000 0 0 100011 111101		= 0x821008FD
 281 038c 800B98FD 		vins.vs		v32, r23			#100000 000000 10111 0 0 1 100011 111101	= 0x800B98FD
 282              		#Check Vext Code??
 283 0390 597008BD 		vext.vv		v22, v23			#010110 010111 000000 0 0 100010 111101		= 0x597008BD
 284 0394 497018BD 		vext.vs		r9, v23				#01001 0 010111 000000 0 1 100010 111101	= 0x497018BD
 285 0398 49701ABD 		vextu.vs	r9, v23				#01001 0 010111 000000 0 1 101010 111101	= 0x49701ABD
 286              	
 287              		#vcompress	v22, v23			#010110 010111 000000 0 0 110010 111101		= 0x59700CBD
 288              		#vcompress.1 v22, v23			#											= 0x59702CBD
 289              	
 290              		#vexpand		v22, v23			#010110 010111 000000 0 0 110011 111101		= 0x59700CFD
 291              		#vexpand.1	v22, v23			#											= 0x59702CFD
 292              	
 293 039c 59700C7D 		vupshift		v22, v23			#010110 010111 000000 0 0 110001 111101		= 0x59700C7D
 294              	
 295 03a0 740616BE 		vfins.vs	vfgr27, r12				#011101 000000 01100 0 0 1 011010 111110	= 0x740616BE
 296 03a4 5965D13E 		vfand.vv	vfgr20, vfgr20, vfgr21	#010110 010110 010111 0 1 000100 111110		= 0x5965D13E
 297 03a8 001091BE 		vfor.vv		vfmask0, vfmask1, vfgr0	#000000 000001 000010 0 1 000110 111110		= 0x001091BE
 298 03ac 04C351FE 		vfxor.vv	vfmask1, vfgr10, vfgr11	#000001 001100 001101 0 1 000111 111110		= 0x04C351FE
 299 03b0 38F4117E 		vfnor.vv	vfgr12, vfgr13, vfgr14	#001110 001111 010000 0 1 000101 111110		= 0x38F4117E
 300 03b4 5962173E 		vfand.vs	vfgr20, vfgr20, r4		#010110 010110 001000 0 1 011100 111110		= 0x5962173E
 301 03b8 001217BE 		vfor.vs		vfmask0, vfmask1, r4	#000000 000001 001000 0 1 011110 111110		= 0x001217BE
 302 03bc 04C217FE 		vfxor.vs	vfmask1, vfgr10, r4		#000001 001100 001000 0 1 011111 111110		= 0x04C217FE
 303 03c0 38F2177E 		vfnor.vs	vfgr12, vfgr13, r4		#001110 001111 001000 0 1 011101 111110		= 0x38F2177E
 304 03c4 0400103E 		vfclr	vfmask1					#000001 000000 000000 0 1 000000 111110		= 0x0400103E
 305 03c8 0400107E 		vfset	vfmask1					#000001	000000 000000 0 1 000001 111110		= 0x0400107E
 306              	
 307 03cc C00B1A3E 		vmstc	vbase0, r22				#110000 000000 10110 0 0 1 101000 111110	= 0xC00B1A3E
 308 03d0 000B1A3E 		vmstc	VL, r22					# 0x000B1A3E
 309 03d4 840B1A3E 		vmstc	vstride1, r22			# 0x840B1A3E
 310 03d8 A80B1A3E 		vmstc	vinc2, r22				# 0xA80B1A3E
 311 03dc BB001A7E 		vmcts	r23, vbase0				#10111 0 110000 000000 0 1 101001 111110	= 0xBB101A7E
 312 03e0 BA101A7E 		vmcts	r23, vstride1			# 0xBA101A7E
 313 03e4 BAA01A7E 		vmcts	r23, vinc2				# 0xBAA01A7E
 314              		#vsync							#000000 000000 000000 0 1 100000 111110		= 0x0000183E
 315              	
 316              		.size	main, .-main
 317              		.ident	"GCC: (GNU) 3.4.1 (Altera Nios II 6.1 b197)"
 318              	
GAS LISTING v_assembler_test.s 			page 7


DEFINED SYMBOLS
                            *ABS*:00000000 hello_world.c
  v_assembler_test.s:4      .text:00000000 main
  v_assembler_test.s:22     .text:00000040 VECTOR

NO UNDEFINED SYMBOLS
