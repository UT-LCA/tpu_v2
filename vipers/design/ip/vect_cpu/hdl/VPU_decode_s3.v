//************************************************************************
// Scalable Vector CPU Project -- Stage 3 instruction decode
// Filename: VPU_decode_s3.v
// Author: Jason Yu
//
// Pipeline stage 3 combinational instruction decode
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


module VPU_decode_s3
(
	input		[31:0]	s3_opCode,

	output 		[`log2(`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC)-1:0]	s3_vctrl_rdaddr,
	output reg			s3_vc_vctrl_rden,				// read from control register (for vinc)

	// control signals that can change state of processor
	// clear these control signals when the previous stage (decode) stalls
	output reg			s3_instr_nonmaskable,			// a non maskable instruction
	output				s3_vstype,						// whether instruction is vs-type
	output		[`ALUOP_WIDTH-1:0]		s3_ALUOp,
	output		[`log2(`VRF_NREG)-1:0]	s3_vrf_wraddr,
	output reg	[`log2(`FLAG_NREG)-1:0]	s3_vflag_wraddr,

	output reg			s3_loadDataQToVReg,				// Transfer loaded data from load data Q to VRF
	output reg			s3_loadDataQ_rdmaskable,		// the rdreq for transferring data from load data Q to VRF is maskable (for vins.vv)
	output reg			s3_zeroMACchainout,				// zero the MAC chainout
	output reg			s3_MAC_en,						// enable the MAC
	output reg			s3_zeroACC,						// zero the accumulators
	output reg			s3_vext_sv,
	output reg			s3_decode_onelane_en,			// decode vector lane enable of last element
	output reg			s3_scalarRegInSel,
	output				s3_signedOp,
	output reg			s3_vpmem_wren,					// local memory wren
	output reg			s3_indexed_memaccess,			// indexed memory access
	output reg	[2:0]	s3_vectorToScalarQ_data_source,	// vector to scalar tranfer queue write data source
	output reg			s3_vectorToScalarQ_data_zext,	// zero extend vector to scalar transfer queue write data
	output reg			s3_vrf_wren,					// needed for hazard detection
	output reg			s3_vflag_wren,					// needed for hazard detection
	output reg			s3_vpmem_rden,					// vlane local memory rden
	output reg			s3_shift_sv						// scalar-vector shift (vector data, scalar shift amount)
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	[`log2(`VPU_NCTRLREG)-1:0]	vrf_vmcts_source;
reg		[`log2(`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC)-1:0]	vc_vctrl_reg_rdaddr;
reg		[`log2(`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC)-1:0]	vc_vctrl_offset;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Combinational assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------
assign	s3_ALUOp = s3_opCode[`OP_FUNC];
assign	vrf_vmcts_source = s3_opCode[`OP_SDTYPE_VSREG];

assign 	s3_vrf_wraddr = s3_opCode[`OP_VRFWRADDR];
assign	s3_signedOp = ~s3_ALUOp[`OP_SIGNBIT];
assign	s3_vstype = (s3_opCode[`OP_OPX] == `OPX_VSTYPE) ? 1'b1 : 1'b0;

assign	s3_vctrl_rdaddr = vc_vctrl_reg_rdaddr + vc_vctrl_offset;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Decode control signals
//---------------------------------------------------------------------------------------------------------------------------------------------------

always @(*)
begin
	// default
	s3_vflag_wraddr = s3_opCode[`OP_VFLAGWRADDR];
	s3_vflag_wren = 1'b0;
	
	// default assignments
	s3_vc_vctrl_rden = 1'b0;
	vc_vctrl_reg_rdaddr = 'bx;
	vc_vctrl_offset = 0;
	s3_instr_nonmaskable = 1'b0;			// a non maskable instruction
	s3_loadDataQToVReg = 1'b0;
	s3_zeroMACchainout = 1'b1;				// need to clear chainout by default, otherwise ACC will keep accumulating
	s3_zeroACC = 1'b0;
	s3_scalarRegInSel = 1'b0;
	s3_indexed_memaccess = 1'b0;
	s3_vectorToScalarQ_data_source = 3'bx;
	s3_vectorToScalarQ_data_zext = 1'b0;
	s3_vext_sv = 1'b0;
	s3_decode_onelane_en = 1'b0;
	s3_vpmem_wren = 1'b0;			// local memory wren
	s3_vrf_wren = 1'b0;
	s3_loadDataQ_rdmaskable = 1'b0;
	s3_shift_sv = 1'b0;
	s3_MAC_en = 1'b0;
	s3_vpmem_rden = 1'b0;

//----------------------------------------------------------------------------------------------------
// Decode Instruction
//----------------------------------------------------------------------------------------------------
	casex (s3_opCode[`OP_OPCODE])

//----------------------------------------------------------------------------------------------------
// VR, VS type instruction
//----------------------------------------------------------------------------------------------------
		`OPCODE_VRTYPE: begin
			// default writes to VRF; for hazard detection
			s3_vrf_wren = 1'b1;

			// Select scalar register source if VS or SV instruction
			if (s3_opCode[`OP_OPX] == `OPX_VSTYPE)
				s3_scalarRegInSel = 1'b1;
			
//----------------------------------------------------------------------------------------------------
// Instruction dependent control signals
//----------------------------------------------------------------------------------------------------
			casex (s3_ALUOp)

//----------------------------------------------------------------------------------------------------
// Instructions that have same vector-vector and vector-scalar opcodes
//----------------------------------------------------------------------------------------------------
				// add/sub instructions
				`ALUOP_VADD,
				`ALUOP_VSUB: begin
					s3_vflag_wraddr = `CREG_OVERFLOW;	// use a default flag register to store cout/overflow value for add/sub
					s3_vflag_wren = 1'b1;
				end
					
				// logical instructions
				`OP_LOGIC: begin
					s3_vflag_wraddr = `CREG_OVERFLOW;
					s3_vflag_wren = 1'b1;		// logical instructions clear overflow flag
				end
				
				// max/min instructions
				`ALUOP_VMAXMIN: begin
					s3_vflag_wraddr = `CREG_OVERFLOW;	// use a default flag register to store overflow value for add/sub
					s3_vflag_wren = 1'b1;		// write overflow flag
				end
					
				// comparison: less than, less than or equal
				// comparison: equal, not equal
				`ALUOP_CMP_LTTYPE,
				`ALUOP_CMP_EQTYPE: begin
					s3_vrf_wren = 1'b0;
					s3_vflag_wren = 1'b1;
					s3_vflag_wraddr = s3_opCode[`OP_VFLAGWRADDR];
				end
				
				// vector merge
				`ALUOP_VMERGE:
					// it is non-maskable becasue the flag only selects between data, but all data need to be written to VRF
					s3_instr_nonmaskable = 1'b1;

				// vector extract
				`ALUOP_VEXT: begin
					// vector-scalar extract
					if (s3_opCode[`OP_OPX] == `OPX_VSTYPE) begin
						s3_vext_sv = 1'b1;
						s3_vrf_wren = 1'b0;
						s3_decode_onelane_en = 1'b1;
						// select output of index offset MUX to copy to scalar queue
						s3_vectorToScalarQ_data_source = `VTOSQ_DATA_VINDEX;
						s3_vectorToScalarQ_data_zext = 1'b0;
					end
`ifdef	VECTOR_MANIP_INSTRUCTIONS
					// vector-vector extract
					else
						// does not write immediately because it is handled by memory unit
						s3_vrf_wren = 1'b0;
`endif
				end
				
				// unsigned vector-scalar extract
				`ALUOP_VEXTU_VS: begin
					// vector-scalar format only
					if (s3_opCode[`OP_OPX] == `OPX_VSTYPE) begin
						s3_vext_sv = 1'b1;
						s3_vrf_wren = 1'b0;
						s3_decode_onelane_en = 1'b1;
						// select output of index offset MUX to copy to scalar queue
						s3_vectorToScalarQ_data_source = `VTOSQ_DATA_VINDEX;
						s3_vectorToScalarQ_data_zext = 1'b1;
					end
				end

				// Vector insert
				`ALUOP_VINS: begin
					// vector-scalar insert
					if (s3_opCode[`OP_OPX] == `OPX_VSTYPE) begin
						s3_decode_onelane_en = 1'b1;
					end
`ifdef	VECTOR_MANIP_INSTRUCTIONS
					// vector-vector insert
					else
						// does not write immediately because it is handled by memory unit
						s3_vrf_wren = 1'b0;
`endif
				end

`ifdef	CLUSTER_MULTACCUM
				// Cluster multiply-accumulate
				`ALUOP_VMACC: begin
					s3_zeroMACchainout = 1'b1;		// zero the MAC chainout when we want to write to the accumulators
					s3_vrf_wren = 1'b0;
					s3_MAC_en = 1'b1;
				end
`endif

//----------------------------------------------------------------------------------------------------
// Opcodes that have different vector-vector and vector-scalar instructions
//----------------------------------------------------------------------------------------------------

				// vs-type add/sub instructions
				`ALUOP_VSUB_SV: begin
					if (s3_opCode[`OP_OPX] == `OPX_VSTYPE) begin
						s3_vflag_wren = 1'b1;		// write overflow flag
						s3_vflag_wraddr = `CREG_OVERFLOW;	// use a default flag register to store overflow value for add/sub
					end
				end
				
				// comparison: scalar-vector
				`ALUOP_CMP_LT_SV,
				`ALUOP_CMP_LE_SV: begin
					// only VS type
					if (s3_opCode[`OP_OPX] == `OPX_VSTYPE) begin
						s3_vrf_wren = 1'b0;
						s3_vflag_wren = 1'b1;
						s3_vflag_wraddr = s3_opCode[`OP_VFLAGWRADDR];
					end
				end
				
				// scalar-vector shift instructions
				`ALUOP_VSLL_SV,
				`ALUOP_VSRL_SV,
				`ALUOP_VROT_SV:	begin
					// VS type instruction
					if (s3_opCode[`OP_OPX] == `OPX_VSTYPE) begin
						s3_shift_sv = 1'b1;
						// SV shift instructions use VRF output as ALU input B
						s3_scalarRegInSel = 1'b0;
					end
				end

				// vsra.sv shares opcode with vccacc
				`ALUOP_VSRA_SV: begin
					// VS type instruction
					if (s3_opCode[`OP_OPX] == `OPX_VSTYPE) begin
						s3_shift_sv = 1'b1;
						// SV shift instructions use VRF output as ALU input B
						s3_scalarRegInSel = 1'b0;
					end

`ifdef	CLUSTER_MULTACCUM
				// compress copy from accumulators
				// `ALUOP_VCCACC: begin
					// VR-type instruction
					else begin
						s3_instr_nonmaskable = 1'b1;	// too confusing if maskable (what do the flags control??)
						s3_zeroMACchainout = 1'b0;		// let chainout signal accumulate
					end
`endif
				end
				
				`ALUOP_VMERGE_SV: begin
					// VMERGE.SV instruction
					if (s3_opCode[`OP_OPX] == `OPX_VSTYPE)
						// it is non-maskable becasue the flag only selects between data, but all data need to be written to VRF
						s3_instr_nonmaskable = 1'b1;
`ifdef	CLUSTER_MULTACCUM
					else begin
					// vcczacc: compress copy from accumulators and zero accumulators
					// `ALUOP_VCCZACC: begin
						// VR-type instruction
						// for copying
						s3_instr_nonmaskable = 1'b1;	// too confusing if maskable (what do the flags control??)
						s3_zeroMACchainout = 1'b0;		// let chainout signal accumulate
						// for zeroing
						s3_zeroACC = 1'b1;				// zero the accumulators
						// s3_vrf_wren = 1'b0;
					end
`endif
				end

				default:	;
			endcase
		end

		
		//--------------------------------------------------			
		// Memory Instructions
		//--------------------------------------------------
		`OPCODE_VMTYPE: begin
			s3_vrf_wren = 1'b0;
			
			// Decode memory instruction
			casex (s3_ALUOp)
				// Pseudo instruction to transfer data from memory load queue to VRF
				`FUNC_MEM_XFERLOADQVRF: begin
					s3_loadDataQToVReg = 1'b1;		// Transfer loaded data from load data Q to VRF
					s3_vrf_wren = 1'b1;
				end
				
				// Pseudo instruction to transfer data from memory load queue to VRF; data elements are non-maskable
				`FUNC_MEM_XFERLOADQVRF_NOMASK: begin
					s3_instr_nonmaskable = 1'b1;
					s3_loadDataQToVReg = 1'b1;		// Transfer loaded data from load data Q to VRF
					s3_vrf_wren = 1'b1;
				end

`ifdef	VECTOR_MANIP_INSTRUCTIONS
				// Pseudo instruction to transfer vins.vv data from memory load queue to VRF
				// data elements are non-maskable; loadDataQ_rdreq is maskable to as only elements to be inserted are written to the Q
				`FUNC_MEM_XFERLOADQVRF_VINS: begin
					s3_instr_nonmaskable = 1'b1;
					s3_loadDataQToVReg = 1'b1;		// Transfer loaded data from load data Q to VRF
					s3_loadDataQ_rdmaskable = 1'b1;
					s3_vrf_wren = 1'b1;
				end
`endif

				// Pseudo instruction to transfer data from memory load queue to vector flag register
				`FUNC_MEM_XFERLOADQVFLAG: begin
					s3_loadDataQToVReg = 1'b1;		// transfer loaded flag from load data Q to vector flag regsiter
					s3_instr_nonmaskable = 1'b1;
					s3_vflag_wren = 1'b1;
				end

`ifdef	INDEXED_MEMORY_ACCESS
				`FUNC_MEM_INDEXEDLOAD: begin
					s3_indexed_memaccess = 1'b1;
				end

				// Indexed store
				`FUNC_MEM_INDEXEDSTORE: begin
					s3_indexed_memaccess = 1'b1;
				end
`endif

`ifdef VP_LOCALMEM
				// Load from local memory
				`FUNC_MEM_LOCALLOAD: begin
					s3_vpmem_rden = 1'b1;
				end

				// Store to local memory
				`FUNC_MEM_LOCALSTORE: begin
					s3_vpmem_wren = 1'b1;
					// scalar broadcast store to local memory
					if (s3_opCode[`OP_OPX] == `OPX_VSTYPE)
						s3_scalarRegInSel = 1'b1;
				end
`endif

				// read from control register for increment
				// currently all instructions that read vctrl in stage 1 (VPU_decode) stall if LSU is busy
				// so should have no conflict on read port
				`FUNC_MEM_UNITLOAD,
				`FUNC_MEM_STRIDEDLOAD,
				`FUNC_MEM_UNITSTORE,
				`FUNC_MEM_STRIDEDSTORE,
				`FUNC_FLAGLOAD,
				`FUNC_FLAGSTORE: begin
					s3_vc_vctrl_rden = 1'b1;
					vc_vctrl_reg_rdaddr = s3_opCode[`OP_VINC];
					vc_vctrl_offset = `VC_VINC_FIRST;
				end
				
			default:	;
			endcase
		end


		//--------------------------------------------------			
		// Flag/Misc Instructions/Fixed point
		//--------------------------------------------------
		`OPCODE_MISCTYPE: begin
			
			// Flag/Miscellaneous instructions
			if (s3_opCode[`OP_OPX] == `OPX_MISCTYPE) begin
				// decode instruction
				casex (s3_ALUOp)

					// vector-scalar flag insert
					`ALUOP_VFINS_VS:
						s3_decode_onelane_en = 1'b1;

					// vector-scalar flag logical instructions
					`ALUOP_FLAG_VFAND_VS,
					`ALUOP_FLAG_VFNOR_VS,
					`ALUOP_FLAG_VFOR_VS,
					`ALUOP_FLAG_VFXOR_VS: begin
						s3_instr_nonmaskable = 1'b1;
						s3_vflag_wren = 1'b1;
						s3_vflag_wraddr = s3_opCode[`OP_VFLAGWRADDR];
					end
					
					// flag logical instructions
					`FUNC_FLAGLOGIC: begin
						s3_instr_nonmaskable = 1'b1;
						s3_vflag_wren = 1'b1;
						s3_vflag_wraddr = s3_opCode[`OP_VFLAGWRADDR];
					end
					
					// move scalar to control register
					`FUNC_MSTC:
						s3_instr_nonmaskable = 1'b1;

					// move control register to scalar register
					`FUNC_MCTS: begin
						case (vrf_vmcts_source)
							`VC_VL		:	s3_vectorToScalarQ_data_source = `VTOSQ_DATA_VL;
							`VC_VBASE	:	s3_vectorToScalarQ_data_source = `VTOSQ_DATA_VBASE;
							`VC_VSTRIDE	:	s3_vectorToScalarQ_data_source = `VTOSQ_DATA_VCTRL;
							`VC_VINC	:	s3_vectorToScalarQ_data_source = `VTOSQ_DATA_VCTRL;
							default		:	s3_vectorToScalarQ_data_source = `VTOSQ_DATA_VCTRL;
						endcase
					end
				default:	;
				endcase
			end
		end	// end `OPCODEMISCTYPE

		default:	;
	endcase
end

endmodule
