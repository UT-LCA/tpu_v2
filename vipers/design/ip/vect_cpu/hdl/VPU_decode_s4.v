//************************************************************************
// Scalable Vector CPU Project -- Stage 4 Instruction decode
// Filename: VPU_decode_s4.v
// Author: Jason Yu
//
// Pipeline stage 4 combinational instruction decode
//
//  Copyright (C) 2007 Jason Yu
//
//************************************************************************

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on
`ifdef	MODELSIM
	`include "../hdl/define.v"
	`include "../hdl/isa_def.v"
	`include "../hdl/config_def.v"
`else
	`include "define.v"
	`include "isa_def.v"
	`include "config_def.v"
`endif


module VPU_decode_s4
(
	input		[31:0]	s4_opCode,

	output reg			s4_vbase_add_vinc,
	output				s4_vstype,
	output		[`ALUOP_WIDTH-1:0]	s4_ALUOp,
	output		[`log2(`VRF_NREG)-1:0]	s4_vrf_wraddr,
	output reg			s4_vrf_wren,
	output reg			s4_vflag_wren,
	output reg			s4_vc_vl_wren,
	output reg			s4_vctrl_wren,
	output reg			s4_vc_vbase_wren,
	output reg	[1:0]	s4_vflagWriteSel,
	output reg	[`log2(`FLAG_NREG)-1:0]	s4_vflag_wraddr,
	output				s4_signedOp,
	output reg			s4_multiplyHigh,
	output reg	[2:0]	s4_vrfWriteSel,
	output reg			s4_shift_instr,					// instruction is a shift instruction
	output		[`log2(`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC)-1:0]	s4_vctrl_wraddr,
	output		[`log2(`VC_NVBASE)-1:0]				s4_vc_vbase_wraddr,
	output reg			s4_vupshift,
	output reg			s4_MAC_en,
	output reg			s4_zeroACC,
	output reg			s4_zeroMACchainout,				// zero the MAC chainout
	output reg			s4_vl_accvl_update				// update VL after a copy from accumulator instruction
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	[`log2(`VPU_NCTRLREG)-1:0]	vrf_vmstc_dest;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Combinational assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------
assign	s4_ALUOp = s4_opCode[`OP_FUNC];
assign	vrf_vmstc_dest = s4_opCode[`OP_SSTYPE_VDREG];
assign	s4_vstype = (s4_opCode[`OP_OPX] == `OPX_VSTYPE) ? 1'b1 : 1'b0;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Assign outputs
//---------------------------------------------------------------------------------------------------------------------------------------------------

assign 	s4_vrf_wraddr = s4_opCode[`OP_VRFWRADDR];
// Instructions are unsigend if the OP_SIGNBIT is set; the same bit is used for ALU and memory instructions
assign	s4_signedOp = ~s4_ALUOp[`OP_SIGNBIT];

// Only write to control registers from move scalar to control VMSTC instruction (SSTYPE instruction); automatic truncate
assign	s4_vctrl_wraddr = s4_opCode[`OP_SSTYPE_VDREG];
assign	s4_vc_vbase_wraddr = (s4_vbase_add_vinc == 1'b1) ? s4_opCode[`OP_VBASE] : s4_opCode[`OP_SSTYPE_VDREG] - `VC_VBASE_FIRST;


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Decode control signals
//---------------------------------------------------------------------------------------------------------------------------------------------------

always @(*) begin
	// Default assignments
	s4_vflag_wraddr = s4_opCode[`OP_VFLAGWRADDR];
	s4_vrfWriteSel = `VRFWR_ALURESULT;
	s4_vflagWriteSel = `VFLAGWR_ALUFLAG;
	s4_vrf_wren = 1'b0;
	s4_vflag_wren = 1'b0;
	s4_multiplyHigh = 1'b0;
	
	// Shifter signals
	s4_shift_instr = 1'b0;					// instruction is a shift instruction
	s4_vbase_add_vinc = 1'b0;
	s4_vc_vbase_wren = 1'b0;
	s4_vc_vl_wren = 1'b0;
	s4_vctrl_wren = 1'b0;
	s4_vupshift = 1'b0;
	s4_zeroACC = 1'b0;
	s4_MAC_en = 1'b0;
	s4_zeroMACchainout = 1'b1;				// need to clear chainout by default, otherwise ACC will keep accumulating
	s4_vl_accvl_update = 1'b0;


//----------------------------------------------------------------------------------------------------
// Decode Instruction
//----------------------------------------------------------------------------------------------------
	casex (s4_opCode[`OP_OPCODE])
	
		`OPCODE_VRTYPE: begin
			casex (s4_ALUOp)

//----------------------------------------------------------------------------------------------------
// Instructions that have same vector-vector and vector-scalar opcodes
//----------------------------------------------------------------------------------------------------
				// add/sub instructions
				`ALUOP_VADD,
				`ALUOP_VSUB: begin
					s4_vrf_wren = 1'b1;
					s4_vflag_wren = 1'b1;
					s4_vflagWriteSel = `VFLAGWR_ALUFLAG;
					
					// if (s4_signedOp == 1'b1)
						s4_vflag_wraddr = `CREG_OVERFLOW;	// use a default flag register to store cout/overflow value for add/sub
					// else
						// s4_vflag_wraddr = `CREG_CARRY;	// use a default flag register to store cout/overflow value for add/sub
				end

				// logical instructions
				`OP_LOGIC: begin
					s4_vrf_wren = 1'b1;
					s4_vflag_wren = 1'b1;		// logical instructions clear overflow flag
					s4_vflag_wraddr = `CREG_OVERFLOW;
					s4_vflagWriteSel = `VFLAGWR_ALUFLAG;
				end

				// max/min instructions
				`ALUOP_VMAXMIN: begin
					s4_vrf_wren = 1'b1;
					s4_vflag_wren = 1'b1;	// write overflow flag
					s4_vflag_wraddr = `CREG_OVERFLOW;	// use a default flag register to store overflow value for add/sub
					s4_vflagWriteSel = `VFLAGWR_ALUFLAG;
				end

				// vector merge
				`ALUOP_VMERGE: begin
					// merge instruction
					s4_vrf_wren = 1'b1;
				end

				// comparison: less than, less than or equal
				// comparison: equal, not equal
				`ALUOP_CMP_LTTYPE,
				`ALUOP_CMP_EQTYPE: begin
					s4_vflag_wren = 1'b1;
					s4_vflagWriteSel = `VFLAGWR_ALUFLAG;
					s4_vflag_wraddr = s4_opCode[`OP_VFLAGWRADDR];
				end

				// absolute difference
				`ALUOP_VABSDIF:
					s4_vrf_wren = 1'b1;

				// Multiplication: multiply high
				`ALUOP_MULTHIGH: begin
					s4_vrf_wren = 1'b1;
					s4_vrfWriteSel = `VRFWR_MULTRESULT;
					s4_multiplyHigh = 1'b1;
				end

				// Multiplication: multiply low
				`ALUOP_MULTLOW: begin
					s4_vrf_wren = 1'b1;
					s4_vrfWriteSel = `VRFWR_MULTRESULT;
				end
				
				// shift instructions
				`ALUOP_VSRA,	// arithmetic shift right
				`ALUOP_VSLL,	// shift left logical
				`ALUOP_VSRL,	// shift right logical
				`ALUOP_VROT: 	// barrel rotate (left)
				begin
					s4_vrf_wren = 1'b1;
					s4_shift_instr = 1'b1;					// instruction is a shift instruction
				end


				// Vector insert
				`ALUOP_VINS: begin
					// vector-scalar insert
					if (s4_opCode[`OP_OPX] == `OPX_VSTYPE) begin
						s4_vrf_wren = 1'b1;
					end
				end

//----------------------------------------------------------------------------------------------------
// Opcodes that have different vector-vector and vector-scalar instructions
//----------------------------------------------------------------------------------------------------

				// vs-type add/sub instructions
				`ALUOP_VSUB_SV: begin
					if (s4_opCode[`OP_OPX] == `OPX_VSTYPE) begin
						s4_vrf_wren = 1'b1;
						s4_vflag_wraddr = `CREG_OVERFLOW;	// use a default flag register to store overflow value for add/sub
						s4_vflag_wren = 1'b1;
						s4_vflagWriteSel = `VFLAGWR_ALUFLAG;
						
						// if (s4_signedOp == 1'b1)
							// s4_vflag_wraddr = `CREG_OVERFLOW;	// use a default flag register to store cout/overflow value for add/sub
						// else
							// s4_vflag_wraddr = `CREG_CARRY;	// use a default flag register to store cout/overflow value for add/sub
					end
`ifdef	VUPSHIFT
					// VUPSHIFT instruction
					// VR-type instruction
					else begin
						s4_vupshift = 1'b1;
						s4_vrfWriteSel = `VRFWR_SHIFTINNEXT;
						s4_vrf_wren = 1'b1;
					end
`endif
				end

				
`ifdef	CLUSTER_MULTACCUM
				// Cluster multiply-accumulate
				`ALUOP_VMACC: begin
					s4_MAC_en = 1'b1;
					s4_zeroMACchainout = 1'b1;		// zero chainout when doing MAC
				end
`endif
				
				`ALUOP_VMERGE_SV: begin
					// VMERGE.SV instruction
					if (s4_opCode[`OP_OPX] == `OPX_VSTYPE) begin
						// merge instruction
						s4_vrf_wren = 1'b1;
					end
`ifdef	CLUSTER_MULTACCUM
					else begin
				// compress copy from accumulators and zero: VCCZACC
				// `ALUOP_VCCZACC: begin
					// VR-type instruction
					// if (s4_opCode[`OP_OPX] != `OPX_VSTYPE) begin
						// for copying
						s4_zeroMACchainout = 1'b0;				// let chainout signal accumulate
						s4_vrfWriteSel = `VRFWR_CLUSTERMAC;
						s4_vrf_wren = 1'b1;
						s4_vl_accvl_update = 1'b1;				// update VL after vcczacc
						s4_vc_vl_wren = 1'b1;
						// for zeroing
						s4_zeroACC = 1'b1;
						s4_MAC_en = 1'b1;
					end
				// end
`endif
				end

				`ALUOP_CMP_LT_SV: begin
					// VS-type
					if (s4_opCode[`OP_OPX] == `OPX_VSTYPE) begin
						s4_vflag_wren = 1'b1;
						s4_vflagWriteSel = `VFLAGWR_ALUFLAG;
						s4_vflag_wraddr = s4_opCode[`OP_VFLAGWRADDR];
					end

					// VR-type
					else
					// absolute value
					// `ALUOP_VABS: begin
					// VR-type only
						s4_vrf_wren = 1'b1;
				end

				`ALUOP_CMP_LE_SV: begin
					if (s4_opCode[`OP_OPX] == `OPX_VSTYPE) begin
						s4_vflag_wren = 1'b1;
						s4_vflagWriteSel = `VFLAGWR_ALUFLAG;
						s4_vflag_wraddr = s4_opCode[`OP_VFLAGWRADDR];
					end
				end
				
				// arithmetic shift right (scalar-vector)
				`ALUOP_VSRA_SV: begin
					if (s4_opCode[`OP_OPX] == `OPX_VSTYPE) begin
						s4_vrf_wren = 1'b1;
						s4_shift_instr = 1'b1;					// instruction is a shift instruction
					end
`ifdef	CLUSTER_MULTACCUM
					else begin
				// compress copy from accumulators: VCCACC
				// `ALUOP_VCCACC: begin
					// VR-type instruction
					// if (s4_opCode[`OP_OPX] != `OPX_VSTYPE) begin
						s4_zeroMACchainout = 1'b0;				// let chainout signal accumulate
						s4_vrfWriteSel = `VRFWR_CLUSTERMAC;
						s4_vrf_wren = 1'b1;
						s4_vl_accvl_update = 1'b1;				// update VL after vccacc
						s4_vc_vl_wren = 1'b1;
					end
				// end
`endif
				end
				
				
				`ALUOP_VSLL_SV,		// shift left logical (scalar-vector)
				`ALUOP_VSRL_SV,		// shift right logical (scalar-vector)
				`ALUOP_VROT_SV:		// barrel rotate (left) (scalar-vector)
				begin
					if (s4_opCode[`OP_OPX] == `OPX_VSTYPE) begin
						s4_vrf_wren = 1'b1;
						s4_shift_instr = 1'b1;					// instruction is a shift instruction
					end
				end
				
			default:	;
			endcase
		end
		
		//--------------------------------------------------			
		// Memory Instructions
		//--------------------------------------------------
		`OPCODE_VMTYPE: begin
			s4_vrf_wren = 1'b0;
		
			// Decode memory instruction
			casex (s4_ALUOp)
				// Pseudo instruction to transfer data from memory load queue to VRF
`ifdef	VECTOR_MANIP_INSTRUCTIONS
				`FUNC_MEM_XFERLOADQVRF_VINS,
`endif
				`FUNC_MEM_XFERLOADQVRF,
				`FUNC_MEM_XFERLOADQVRF_NOMASK: begin
					s4_vrf_wren = 1'b1;
					s4_vflag_wren = 1'b0;
					s4_vrfWriteSel = `VRFWR_EXTLOADDATA;
				end
				
				// Pseudo instruction to transfer data from memory load queue to flag register
				`FUNC_MEM_XFERLOADQVFLAG: begin
					s4_vrf_wren = 1'b0;
					s4_vflag_wren = 1'b1;		// for flag load
					s4_vflagWriteSel = `VFLAGWR_EXTLOADDATA;
				end

`ifdef VP_LOCALMEM
				// Load from local memory
				`FUNC_MEM_LOCALLOAD: begin
					s4_vrf_wren = 1'b1;
					s4_vrfWriteSel = `VRFWR_LOCALMEM;
				end
`endif
				
				// update vbase with vinc
				`FUNC_MEM_UNITLOAD,
				`FUNC_MEM_STRIDEDLOAD,
				`FUNC_MEM_UNITSTORE,
				`FUNC_MEM_STRIDEDSTORE,
				`FUNC_FLAGLOAD,
				`FUNC_FLAGSTORE: begin
					s4_vc_vbase_wren = 1'b1;
					s4_vbase_add_vinc = 1'b1;
				end

				default:	;
			endcase
		end
		
		//--------------------------------------------------			
		// Flag/Misc Instructions/Fixed point
		//--------------------------------------------------
		`OPCODE_MISCTYPE: begin
			
			// Flag/Miscellaneous instructions
			if (s4_opCode[`OP_OPX] == `OPX_MISCTYPE) begin
				// decode instruction
				casex (s4_ALUOp)

					// vector-scalar flag insert
					`ALUOP_VFINS_VS: begin
						s4_vflag_wren = 1'b1;
						s4_vflagWriteSel = `VFLAGWR_FLAGUNIT;
					end

					// vector-scalar flag logical instructions
					`ALUOP_FLAG_VFAND_VS,
					`ALUOP_FLAG_VFNOR_VS,
					`ALUOP_FLAG_VFOR_VS,
					`ALUOP_FLAG_VFXOR_VS,
					// flag logical instructions
					`FUNC_FLAGLOGIC: begin
						s4_vflag_wraddr = s4_opCode[`OP_VFLAGWRADDR];
						s4_vflag_wren = 1'b1;
						s4_vflagWriteSel = `VFLAGWR_FLAGUNIT;
					end
					
					// move scalar to control register
					`FUNC_MSTC:	begin
						case (vrf_vmstc_dest)
							`VC_VBASE	:	s4_vc_vbase_wren = 1'b1;
							`VC_VL		:	s4_vc_vl_wren = 1'b1;
							default		:	s4_vctrl_wren = 1'b1;
						endcase
					end

					default:	;
				endcase
			end
		end	// end `OPCODE_MISCTYPE
		default:	;
	endcase
end

endmodule
