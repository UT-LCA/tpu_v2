parameter COP2_VADD           = 'b10z0000000;
parameter COP2_VADD_U         = 'b10z0000001;
parameter COP2_VSUB           = 'b10zz000010;
parameter COP2_VSUB_U         = 'b10zz000011;
parameter COP2_VMULHI         = 'b10z0000100;
parameter COP2_VMULHI_U       = 'b10z0000101;
parameter COP2_VDIV           = 'b10zz000110; //Using as matmul
//parameter COP2_VBFADD         = 'b0100000001; //Using BF16 add
//parameter COP2_VBFMULT        = 'b0100000010; //Using BF16 MULT
//parameter COP2_VACT           = 'b0100000011; //Using ACT
//parameter COP2_VTRP           = 'b0100000100; //Using ACT
parameter COP2_VDIV_U         = 'b10zz000111;

//parameter COP2_VMOD           = 'b10zz001000;
parameter COP2_VBFADD           = 'b10zz001000;  // USING bfloat add Instr: vmod.vv vrdest, vrsrc1,vrsrc2

parameter COP2_VMOD_U         = 'b10zz001001;
parameter COP2_VCMP_EQ        = 'b10zz001010;
parameter COP2_VCMP_NE        = 'b10zz001100;
parameter COP2_VCMP_LT        = 'b10zz001110;
parameter COP2_VCMP_U_LT      = 'b10zz001111;
parameter COP2_VCMP_LE        = 'b10zz010000;
parameter COP2_VCMP_U_LE      = 'b10zz010001;
parameter COP2_VMIN           = 'b10z0010010;
parameter COP2_VMIN_U         = 'b10z0010011;
parameter COP2_VMAX           = 'b10z0010100;
parameter COP2_VMAX_U         = 'b10z0010101;
parameter COP2_VMULLO         = 'b10z0010110;
parameter COP2_VABS           = 'b1000010111;
parameter COP2_VAND           = 'b10z0011000;
parameter COP2_VOR            = 'b10z0011001;
parameter COP2_VXOR           = 'b10z0011010;
parameter COP2_VNOR           = 'b10z0011011;
parameter COP2_VSLL           = 'b10zz011100;
parameter COP2_VSRL           = 'b10zz011101;
parameter COP2_VSRA           = 'b10zz011110;
parameter COP2_VSAT_B         = 'b1000011111;
parameter COP2_VSAT_H         = 'b1001011111;
parameter COP2_VSAT_W         = 'b1010011111;
parameter COP2_VSAT_SU_B      = 'b1000100000;
parameter COP2_VSAT_SU_H      = 'b1001100000;
parameter COP2_VSAT_SU_W      = 'b1010100000;
parameter COP2_VSAT_SU_L      = 'b1011100000;
parameter COP2_VSAT_U_B       = 'b1000100001;
parameter COP2_VSAT_U_H       = 'b1001100001;
parameter COP2_VSAT_U_W       = 'b1010100001;
parameter COP2_VSADD          = 'b10z0100010;
parameter COP2_VSADD_U        = 'b10z0100011;
parameter COP2_VSSUB          = 'b10zz100100;
parameter COP2_VSSUB_U        = 'b10zz100101;
parameter COP2_VSRR           = 'b1000100110;
parameter COP2_VSRR_U         = 'b1000100111;
parameter COP2_VSLS           = 'b1000101000;
parameter COP2_VSLS_U         = 'b1000101001;
parameter COP2_VXUMUL         = 'b10z0101010;
parameter COP2_VXUMUL_U       = 'b10z0101011;
parameter COP2_VXLMUL         = 'b10z0101100;
parameter COP2_VXLMUL_U       = 'b10z0101101;
parameter COP2_VXUMADD        = 'b10z0101110;
parameter COP2_VXUMADD_U      = 'b10z0101111;
parameter COP2_VXUMSUB        = 'b10z0110000;
parameter COP2_VXUMSUB_U      = 'b10z0110001;
parameter COP2_VXLMADD        = 'b10z0110010;
parameter COP2_VXLMADD_U      = 'b10z0110011;
parameter COP2_VXLMSUB        = 'b10z0110100;
parameter COP2_VXLMSUB_U      = 'b10z0110101;
parameter COP2_VINS_VV        = 'b1100000000;
parameter COP2_VINS_SV        = 'b1110000001;
parameter COP2_VEXT_VV        = 'b1100000010;
parameter COP2_VEXT_SV        = 'b1100000011;
parameter COP2_VEXT_U_SV      = 'b1100000100;
parameter COP2_VCOMPRESS      = 'b1100000101;
parameter COP2_VEXPAND        = 'b1100000110;
parameter COP2_VMERGE         = 'b11zz000111;
parameter COP2_VFINS          = 'b1110001000;
parameter COP2_VEXTHALF       = 'b1100001001;
parameter COP2_VHALF          = 'b1100001010;
parameter COP2_VHALFUP        = 'b1100001011;
parameter COP2_VHALFDN        = 'b1100001100;
parameter COP2_VSATVL         = 'b1100001101;
parameter COP2_VFAND          = 'b11z0001110;
parameter COP2_VFOR           = 'b11z0001111;
parameter COP2_VFXOR          = 'b11z0010000;
parameter COP2_VFNOR          = 'b11z0010001;
parameter COP2_VFCLR          = 'b1100010010;
parameter COP2_VFSET          = 'b1100010011;
parameter COP2_VIOTA          = 'b1100010100;
parameter COP2_VCIOTA         = 'b1100010101;
parameter COP2_VFPOP          = 'b1100010110;
parameter COP2_VFFF1          = 'b1100010111;
parameter COP2_VFFL1          = 'b1100011000;
parameter COP2_VFSETBF        = 'b1100011001;
parameter COP2_VFSETIF        = 'b1100011010;
parameter COP2_VFSETOF        = 'b1100011011;
parameter COP2_VFMT8          = 'b1100011100;
parameter COP2_VFMF8          = 'b1100011101;
parameter COP2_VFCLR8         = 'b1100011110;
parameter COP2_VFOR8          = 'b1100011111;
parameter COP2_VFLD           = 'b1100100000;
parameter COP2_VLD_B          = 'b1100100001;
parameter COP2_VLD_H          = 'b1101100001;

//parameter COP2_VLD_W          = 'b1110100001;
//parameter COP2_VBFADD         = 'b1110100001;  // adding bfadder Instr: vld.u.w

parameter COP2_VLD_L          = 'b1111100001;
parameter COP2_VLD_U_B        = 'b1100100010;
parameter COP2_VLD_U_H        = 'b1101100010;

parameter COP2_VLD_U_W        = 'b1110100010;

parameter COP2_VLDS_B         = 'b1100100011;
parameter COP2_VLDS_H         = 'b1101100011;

//parameter COP2_VLDS_W         = 'b1110100011;
parameter COP2_VBFSUB         = 'b1110100011;   // adding bfsub Instr: vlds.w

parameter COP2_VLDS_L         = 'b1111100011;
parameter COP2_VLDS_U_B       = 'b1100100100;
parameter COP2_VLDS_U_H       = 'b1101100100;

//parameter COP2_VLDS_U_W       = 'b1110100100;
parameter COP2_VBFMULT         = 'b1110100100;   // adding bfmult Instr: vlds.u.w

parameter COP2_VLDX_B         = 'b1100100101;
parameter COP2_VLDX_H         = 'b1101100101;

//parameter COP2_VLDX_W         = 'b1110100101;
parameter COP2_VTRP         = 'b1110100101;     // adding transpose instruction: vldx.w

parameter COP2_VLDX_L         = 'b1111100101;
parameter COP2_VLDX_U_B       = 'b1100100110;
parameter COP2_VLDX_U_H       = 'b1101100110;

//parameter COP2_VLDX_U_W       = 'b1110100110;
parameter COP2_VACT       = 'b1110100110;        //adding activation Instr: vldx.u.w 

parameter COP2_VFST           = 'b1100101000;
parameter COP2_VST_B          = 'b1100101001;
parameter COP2_VST_H          = 'b1101101001;

//parameter COP2_VST_W        = 'b1110101001;  // adding reduction Instr: vst.w
parameter COP2_VRED           = 'b1110101001;  // adding reduction Instr: vst.w

parameter COP2_VST_L          = 'b1111101001;
parameter COP2_VSTS_B         = 'b1100101010;
parameter COP2_VSTS_H         = 'b1101101010;

//parameter COP2_VSTS_W         = 'b1110101010;
parameter COP2_VPER           = 'b1110101010;  // adding permute Instr: vsts.w

parameter COP2_VSTS_L         = 'b1111101010;
parameter COP2_VSTX_B         = 'b1100101011;
parameter COP2_VSTX_H         = 'b1101101011;
parameter COP2_VSTX_W         = 'b1110101011;
parameter COP2_VSTX_L         = 'b1111101011;
parameter COP2_VSTXO_B        = 'b1100101100;
parameter COP2_VSTXO_H        = 'b1101101100;
parameter COP2_VSTXO_W        = 'b1110101100;
parameter COP2_VSTXO_L        = 'b1111101100;
parameter COP2_VMCTS          = 'b1101110000;
parameter COP2_VMSTC          = 'b1101110001;
parameter COP2_CFC2           = 'b0000111000;
parameter COP2_CTC2           = 'b0000111010;
parameter COP2_MTC2           = 'b0000111011;
