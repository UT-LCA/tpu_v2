//************************************************************************
// Scalable Vector CPU Project -- Instruction decode
// Filename: VPU_decode.v
// Author: Jason Yu
//
// This is stage 1 instruction decode of VPU
// - generates control signals for the second pipeline stage
//
//  Copyright (C) 2007 Jason Yu
//
//************************************************************************

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on
`ifdef	MODELSIM
	`include "../hdl/define.v"
	`include "../hdl/config_def.v"
	`include "../hdl/isa_def.v"
`else
	`include "define.v"
	`include "config_def.v"
	`include "isa_def.v"
`endif


// ****************
// **** Module ****
// ****************

module VPU_decode (

	// ** Inputs **
	input	[31:0]	opCode,
	input	lsu_indexed_memaccess,				// whether LSU is executing indexed memory

	// ************
	// ** Outputs **
	//************
	output reg	id_instr_multiple_elems,		// instruction processes multiple data elements
	output reg	id_instr_uses_vlength,			// whether the instruction is subject to vector length
	output reg	id_instr_stalls_pipeline,		// instruction stalls the pipeline until it completes
	output reg	id_scalarToVectorQ_rdreq,		// rdreq for the scalar to vector core transfer queue
	output reg	id_vectorToScalarQ_wrreq,

	// Control registers
	output 		[`log2(`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC)-1:0]	id_vctrl_rdaddr,
	output reg	id_vctrl_rden,
	output 		[`log2(`VC_NVBASE)-1:0]		id_vc_vbase_rdaddr,		// Vector control register: base register
	output reg	id_vc_vbase_rden,
	
	// Vector register file
	output		[`log2(`VRF_NREG)-1:0]	id_vrf_rdaddr1,
	output reg	[`log2(`VRF_NREG)-1:0]	id_vrf_rdaddr2,
	output reg	[`FLAG_NREGADDR-1:0]	id_vflag_rdaddr1,
	output 		[`FLAG_NREGADDR-1:0]	id_vflag_rdaddr2,
	output reg	id_instr_reads_vrfrdaddr1,		// does instruction use the operand; for RAW hazard checking
	output reg	id_instr_reads_vrfrdaddr2,		// does isntruction use the operand; for RAW hazard checking
	output reg	id_instr_reads_vflagrdaddr1,	// does instruction use the operand; for RAW hazard checking
	output reg	id_instr_reads_vflagrdaddr2,	// does instruction use the operand; for RAW hazard checking
	
	output 		id_signedOp,
	output reg	id_newInstr_store,
	output reg	id_lsu_instr,
	output reg	id_clusterStoreQWrreq,
	output reg	id_clear_lsu_data_flag,	// clear vector memory load data Q occupied flag
	output reg	id_stall_if_lsu_busy,				// instruction uses the load store unit
	output reg	id_multElemQuot_offset,	// instruction updates the VRF offset with specific value before executing
	output reg	id_flagstore,
	output reg	id_flagStoreQWrreq,
	output reg	[2:0]	id_memDataWidth,
	output reg	id_zeroACC,
	output reg	id_vind_plus_vl_vectorlength,			// uses a modified vector length of vectorlength = VL + vindex
	output reg	id_vccacc_vl,			// use vctrl_q as vector length
	output reg	id_indexed_load,
	output reg	id_indexed_store,

	// vector lane local memory; declare, but do nont connect when omitted
	output reg	id_vpmem_rden
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Registers and wires
//---------------------------------------------------------------------------------------------------------------------------------------------------

wire	[`ALUOP_WIDTH-1:0]	id_ALUOp;
reg		id_mcts;						// Move from scalar to control register destination
reg		[`log2(`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC)-1:0]	vc_vctrl_reg_rdaddr;
reg		[`log2(`VPU_NCTRLREG_OTHER+`VC_NVSTRIDE+`VC_NVINC)-1:0]	vc_vctrl_offset;
	
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Signal assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------

// Function field
assign	id_ALUOp = opCode[`OP_FUNC];
assign	id_signedOp = ~id_ALUOp[`OP_SIGNBIT];
// Assign register fields
assign 	id_vrf_rdaddr1 = opCode[`OP_VRFRDADDR1];
assign	id_vflag_rdaddr2 = opCode[`OP_VFLAGRDADDR2];	// unsigned extend

// Control registers
assign	id_vc_vbase_rdaddr = (id_mcts == 1'b1) ? opCode[`OP_SDTYPE_VSREG] - `VC_VBASE_FIRST : opCode[`OP_VBASE];	// automatic truncate

assign	id_vctrl_rdaddr = vc_vctrl_reg_rdaddr + vc_vctrl_offset;


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Select instruction type
//---------------------------------------------------------------------------------------------------------------------------------------------------
always @(*)
begin
	// Default value is to read mask from opcode
	id_vflag_rdaddr1 = opCode[`OP_XMASK];			// unsigned extend
	id_vrf_rdaddr2 = opCode[`OP_VRFRDADDR2];
	vc_vctrl_reg_rdaddr = opCode[`OP_SDTYPE_VSREG];		// automatic truncate
	vc_vctrl_offset = 0;
	
	// default assignments
	id_instr_multiple_elems = 1'b0;
	id_instr_uses_vlength = 1'b0;
	id_instr_stalls_pipeline = 1'b0;
	id_stall_if_lsu_busy = 1'b0;
	id_instr_reads_vrfrdaddr1 = 1'b0;
	id_instr_reads_vrfrdaddr2 = 1'b0;
	id_instr_reads_vflagrdaddr1 = 1'b0;
	id_instr_reads_vflagrdaddr2 = 1'b0;

	// control signal default values
	id_newInstr_store = 1'b0;
	id_clusterStoreQWrreq = 1'b0;
	id_clear_lsu_data_flag = 1'b0;
	
	// vector lane local memory
	id_vpmem_rden = 1'b0;			// local memory rden

	// Control registers
	id_scalarToVectorQ_rdreq = 1'b0;
	id_vctrl_rden = 1'b0;
	id_vc_vbase_rden = 1'b0;
	id_vectorToScalarQ_wrreq = 1'b0;

	id_mcts = 1'b0;
	id_multElemQuot_offset = 1'b0;
	id_flagstore = 1'b0;
	id_zeroACC = 1'b0;
	id_flagStoreQWrreq = 1'b0;
	id_vind_plus_vl_vectorlength = 1'b0;			// uses a modified vector length of vectorlength = VL + vindex
	id_lsu_instr = 1'b0;
	id_vccacc_vl = 1'b0;
	
	id_indexed_load = 1'b0;
	id_indexed_store = 1'b0;
	
//----------------------------------------------------------------------------------------------------
// Decode Instruction
//----------------------------------------------------------------------------------------------------
	casex (opCode[`OP_OPCODE])

//----------------------------------------------------------------------------------------------------
// VR, VS type instruction
//----------------------------------------------------------------------------------------------------
		`OPCODE_VRTYPE: begin
		
			// default assignmsnts
			id_instr_multiple_elems = 1'b1;				// multi-element per lane instruction
			id_instr_uses_vlength = 1'b1;				// instruction that is subject to vector length
			id_instr_reads_vrfrdaddr1 = 1'b1;
			id_instr_reads_vflagrdaddr1 = 1'b1;		// default needs to read mask

			// Default for vector-vector instructions
			id_instr_reads_vrfrdaddr2 = 1'b1;
			id_scalarToVectorQ_rdreq = 1'b0;
			
			// scalar vector instructions; both .vs and .sv types
			if (opCode[`OP_OPX] == `OPX_VSTYPE) begin
				// if scalar reg is source, it does not use vrf rdaddr
				id_instr_reads_vrfrdaddr2 = 1'b0;
				// read from the scalar to vector queue
				id_scalarToVectorQ_rdreq = 1'b1;
			end

//----------------------------------------------------------------------------------------------------
// Instruction dependent control signals
//----------------------------------------------------------------------------------------------------
			casex (id_ALUOp)
			
//----------------------------------------------------------------------------------------------------
// Instructions that have same vector-vector and vector-scalar opcodes
//----------------------------------------------------------------------------------------------------
				// Vector extract
				`ALUOP_VEXT: begin
					// begin instruction at VRF offset specified by vindex
					id_multElemQuot_offset = 1'b1;
					vc_vctrl_reg_rdaddr = `VC_VINDEX;
					vc_vctrl_offset = 0;
					id_vctrl_rden = 1'b1;
					
					// vector-scalar extract
					if (opCode[`OP_OPX] == `OPX_VSTYPE) begin
						// id_instr_stalls_pipeline = 1'b1;	// stall pipeline until instruction completes
						id_stall_if_lsu_busy = lsu_indexed_memaccess;	// only stall LSU for vext.vs if LSU is executing indexed memory access
																		// vext.vs uses the indexed offset select MUX
						id_instr_multiple_elems = 1'b0;			// not a multi-element per lane instruction
						id_vectorToScalarQ_wrreq = 1'b1;
						id_instr_reads_vrfrdaddr2 = 1'b0;	// does not use 2nd register operand
						// does not read from the scalar to vector queue
						id_scalarToVectorQ_rdreq = 1'b0;
					end
`ifdef	VECTOR_MANIP_INSTRUCTIONS
					// vector-vector extract
					else begin
						id_lsu_instr = 1'b1;
						id_stall_if_lsu_busy = 1'b1;
						id_instr_reads_vrfrdaddr1 = 1'b0;	// does not use 1st register operand
						id_instr_reads_vrfrdaddr2 = 1'b1;
						// vext.vv uses port 2 for data because port 2 is connected to the store data Q
						// and LSU handles vext.vv and reads from store data Q
						id_vrf_rdaddr2 = opCode[`OP_VRFRDADDR1];
						
						id_instr_multiple_elems = 1'b1;				// regular multi element instruction
						id_clusterStoreQWrreq = 1'b1;			// write data to store queue
						id_vind_plus_vl_vectorlength = 1'b1;			// uses a modified vector length of vectorlength = VL + vindex
					end
`endif
				end
				
				// Vector-scalar extract, unsigned
				`ALUOP_VEXTU_VS: begin
					// vector-scalar format only
					if (opCode[`OP_OPX] == `OPX_VSTYPE) begin
						// begin instruction at VRF offset specified by vindex
						id_multElemQuot_offset = 1'b1;
						vc_vctrl_reg_rdaddr = `VC_VINDEX;
						vc_vctrl_offset = 0;
						id_vctrl_rden = 1'b1;

						// id_instr_stalls_pipeline = 1'b1;	// stall pipeline until instruction completes
						id_stall_if_lsu_busy = lsu_indexed_memaccess;	// only stall LSU for vext.vs if LSU is executing indexed memory access
																		// vext.vs uses the indexed offset select MUX
						id_instr_multiple_elems = 1'b0;			// not a multi-element per lane instruction
						id_vectorToScalarQ_wrreq = 1'b1;
						id_instr_reads_vrfrdaddr2 = 1'b0;	// does not use 2nd register operand
						// does not read from the scalar to vector queue
						id_scalarToVectorQ_rdreq = 1'b0;
					end
				end
				
				// Vector insert
				`ALUOP_VINS: begin
					id_vctrl_rden = 1'b1;
					vc_vctrl_reg_rdaddr = `VC_VINDEX;
					vc_vctrl_offset = 0;
					// vector-scalar insert
					if (opCode[`OP_OPX] == `OPX_VSTYPE) begin
						id_stall_if_lsu_busy = 1'b0;		// vins.vs does not use LSU, so do not have to stall
						id_instr_multiple_elems = 1'b0;			// multi-element per lane instruction
						id_multElemQuot_offset = 1'b1;
						id_instr_reads_vrfrdaddr1 = 1'b0;	// does not use 1st register operand
						id_instr_reads_vrfrdaddr2 = 1'b0;	// does not use 2nd register operand
					end

`ifdef	VECTOR_MANIP_INSTRUCTIONS
					// vector-vector insert
					else begin
						id_lsu_instr = 1'b1;
						id_stall_if_lsu_busy = 1'b1;
						id_instr_reads_vrfrdaddr1 = 1'b0;	// does not use 1st register operand
						id_instr_reads_vrfrdaddr2 = 1'b1;	// uses 2nd register operand
						// vins.vv uses port 2 for data because port 2 is connected to the store data Q
						// and LSU handles vext.vv and reads from store data Q
						id_vrf_rdaddr2 = opCode[`OP_VRFRDADDR1];
						
						id_instr_multiple_elems = 1'b1;				// regular multi element instruction
						id_clusterStoreQWrreq = 1'b1;			// write data to store queue
						// id_vind_plus_vl_vectorlength = 1'b1;			// uses a modified vector length of vectorlength = VL + vindex
					end
`endif
				end

				
//----------------------------------------------------------------------------------------------------
// Opcodes that have different vector-vector and vector-scalar instructions
//----------------------------------------------------------------------------------------------------

				`ALUOP_VSLL_SV,
				`ALUOP_VSRL_SV,
				`ALUOP_VROT_SV: begin
					// VS-type instruction
					if (opCode[`OP_OPX] == `OPX_VSTYPE) begin
						// scalar-vector shift instructions use vrf port 2, and not port 1
						// because shift amount is selected between scalar and VRF port 1
						id_vrf_rdaddr2 = opCode[`OP_VRFRDADDR1];
						id_instr_reads_vrfrdaddr1 = 1'b0;
						id_instr_reads_vrfrdaddr2 = 1'b1;
					end
				end


				// vsra.sv and vccacc share same opcode
				// scalar-vector shift instructions use vrf port 2, and not port 1
				`ALUOP_VSRA_SV: begin
				// `ALUOP_VCCACC: begin
					// VS-type instruction
					if (opCode[`OP_OPX] == `OPX_VSTYPE) begin
						id_vrf_rdaddr2 = opCode[`OP_VRFRDADDR1];
						id_instr_reads_vrfrdaddr1 = 1'b0;
						id_instr_reads_vrfrdaddr2 = 1'b1;
					end
`ifdef	CLUSTER_MULTACCUM	
					// VCCACC: compress copy from accumulators
					// VR-type instruction
					else begin
						id_instr_multiple_elems = 1'b0;			// will never have to process more than one cycle of data
						id_vccacc_vl = 1'b1;					// use special vector length for vccacc/vcczacc
						id_instr_reads_vrfrdaddr1 = 1'b0;
						id_instr_reads_vrfrdaddr2 = 1'b0;		// does not use 2nd register operand
						id_instr_reads_vflagrdaddr1 = 1'b0;		// instruction not masked
						id_instr_stalls_pipeline = 1'b1;		// stall pipeline to update VL
					end
`endif
				end

`ifdef	CLUSTER_MULTACCUM
				`ALUOP_VCCZACC: begin
					// VCCZACC: compress copy from accumulators and zero accumulators
					// VR-type instruction
					if (opCode[`OP_OPX] != `OPX_VSTYPE) begin
						id_instr_multiple_elems = 1'b0;			// will never have to process more than one cycle of data
						id_vccacc_vl = 1'b1;					// use special vector length for vccacc/vcczacc
						id_instr_reads_vrfrdaddr1 = 1'b0;
						id_instr_reads_vrfrdaddr2 = 1'b0;		// does not use 2nd register operand
						id_instr_reads_vflagrdaddr1 = 1'b0;		// instruction not masked
						id_zeroACC = 1'b1;						// to zero
						id_instr_stalls_pipeline = 1'b1;		// stall pipeline to update VL
					end
				end
`endif

`ifdef	VUPSHIFT
				// Vector element shift
				`ALUOP_VUPSHIFT: begin
					if (opCode[`OP_OPX] != `OPX_VSTYPE) begin
						id_instr_uses_vlength = 1'b0;		// instruction is not subject to vector length
						id_vrf_rdaddr2 = opCode[`OP_VRFRDADDR1];
						id_instr_reads_vrfrdaddr2 = 1'b0;	// does not use 2nd register operand
					end
				end
`endif

				default:
					;
			endcase	// end id_ALUOp
				
		end	// end VRTYPE
		
		//--------------------------------------------------			
		// Memory Instructions
		//--------------------------------------------------
		`OPCODE_VMTYPE: begin
			
			// default flag for memory instructions
			id_instr_multiple_elems = 1'b0;
			id_instr_uses_vlength = 1'b1;		// instruction that is subject to vector length
			id_instr_reads_vrfrdaddr1 = 1'b0;	// does not use 1st register operand
			id_instr_reads_vrfrdaddr2 = 1'b0;	// does not use 2nd register operand
			id_instr_reads_vflagrdaddr1 = 1'b1;	// instruction masked by default
			id_instr_reads_vflagrdaddr2 = 1'b0;

			// Decode memory instruction
			casex (id_ALUOp)
				// Pseudo instruction to transfer data from memory load queue to VRF
				`FUNC_MEM_XFERLOADQVRF: begin
					id_instr_multiple_elems = 1'b1;		// regular multi element instruction
					id_clear_lsu_data_flag = 1'b1;	// clear the memory load data Q occupied flag
				end
				
				// Pseudo instruction to transfer data from memory load queue to VRF, but not masked
				`FUNC_MEM_XFERLOADQVRF_NOMASK: begin
					id_instr_multiple_elems = 1'b1;		// regular multi element instruction
					id_clear_lsu_data_flag = 1'b1;	// clear the memory load data Q occupied flag
					id_instr_reads_vflagrdaddr1 = 1'b0;	// instruction not masked
				end
				
`ifdef	VECTOR_MANIP_INSTRUCTIONS
				// Pseudo instruction for vins.vv to transfer data from memory load queue to VRF
				`FUNC_MEM_XFERLOADQVRF_VINS: begin	// Pseudo instruction for vins.vv to transfer data from memory load queue to VRF
					id_instr_multiple_elems = 1'b1;		// regular multi element instruction
					id_clear_lsu_data_flag = 1'b1;	// clear the memory load data Q occupied flag
					id_vind_plus_vl_vectorlength = 1'b1;	// use VL+vindex as vector length for the transfer
					
					// begin instruction at VRF offset specified by vindex
					id_multElemQuot_offset = 1'b1;		// instruction updates the VRF offset with specific value before executing
					vc_vctrl_reg_rdaddr = `VC_VINDEX;
					vc_vctrl_offset = 0;
					id_vctrl_rden = 1'b1;
				end
`endif

				// Pseudo instruction to transfer data from memory load queue to vflag
				`FUNC_MEM_XFERLOADQVFLAG: begin
					id_instr_multiple_elems = 1'b1;		// regular multi element instruction
					id_clear_lsu_data_flag = 1'b1;	// clear the memory load data Q occupied flag
					id_instr_reads_vflagrdaddr1 = 1'b0;	// instruction not masked
				end
				
				// s3_unitstride load
				`FUNC_MEM_UNITLOAD:	begin
					id_lsu_instr = 1'b1;
					id_vc_vbase_rden = 1'b1;
					id_stall_if_lsu_busy = 1'b1;
				end

				// Load isntructions
				`FUNC_MEM_STRIDEDLOAD:	begin
					id_lsu_instr = 1'b1;
					id_vc_vbase_rden = 1'b1;
					vc_vctrl_reg_rdaddr = opCode[`OP_VSTRIDE];
					vc_vctrl_offset = `VC_VSTRIDE_FIRST;
					id_vctrl_rden = 1'b1;
					id_stall_if_lsu_busy = 1'b1;
				end

				// Unit stride Store instructions
				`FUNC_MEM_UNITSTORE:	begin
					id_vrf_rdaddr2 = opCode[`OP_VRFWRADDR];
					id_instr_reads_vrfrdaddr2 = 1'b1;	// reads from 2nd register operand
					id_newInstr_store = 1'b1;
					id_lsu_instr = 1'b1;
					id_vc_vbase_rden = 1'b1;
					id_stall_if_lsu_busy = 1'b1;
					id_instr_multiple_elems = 1'b1;		// regular multi element instruction
					id_clusterStoreQWrreq = 1'b1;	// write data to store queue
				end

				// Store instructions
				`FUNC_MEM_STRIDEDSTORE:	begin
					id_vrf_rdaddr2 = opCode[`OP_VRFWRADDR];
					id_instr_reads_vrfrdaddr2 = 1'b1;	// reads from 2nd register operand
					id_newInstr_store = 1'b1;
					id_lsu_instr = 1'b1;
					id_vc_vbase_rden = 1'b1;
					vc_vctrl_reg_rdaddr = opCode[`OP_VSTRIDE];
					vc_vctrl_offset = `VC_VSTRIDE_FIRST;
					id_vctrl_rden = 1'b1;
					id_stall_if_lsu_busy = 1'b1;
					id_instr_multiple_elems = 1'b1;		// regular multi element instruction
					id_clusterStoreQWrreq = 1'b1;	// write data to store queue
				end

`ifdef	INDEXED_MEMORY_ACCESS
				// Indexed load
				`FUNC_MEM_INDEXEDLOAD: begin
					id_lsu_instr = 1'b1;
					id_vc_vbase_rden = 1'b1;
					id_stall_if_lsu_busy = 1'b1;
					id_instr_multiple_elems = 1'b1;		// regular multi element instruction
					// id_instr_multiple_elems = 1'b0;		// indexed load is special case, does not assert this signal for controller
					id_instr_reads_vrfrdaddr1 = 1'b1;	// uses 1st register operand for indexed offset
					id_indexed_load = 1'b1;
				end

				// Indexed store
				`FUNC_MEM_INDEXEDSTORE: begin
					id_vrf_rdaddr2 = opCode[`OP_VRFWRADDR];
					id_instr_reads_vrfrdaddr2 = 1'b1;	// reads from 2nd register operand
					id_instr_reads_vrfrdaddr1 = 1'b1;	// uses 1st register operand for indexed offset
					id_newInstr_store = 1'b1;
					id_lsu_instr = 1'b1;
					id_vc_vbase_rden = 1'b1;
					id_stall_if_lsu_busy = 1'b1;
					id_instr_multiple_elems = 1'b1;		// regular multi element instruction
					id_clusterStoreQWrreq = 1'b1;	// write data to store queue
					id_indexed_store = 1'b1;
				end
`endif
				
`ifdef VP_LOCALMEM
				// Load from local memory
				`FUNC_MEM_LOCALLOAD: begin
					id_vpmem_rden = 1'b1;
					id_instr_multiple_elems = 1'b1;		// regular multi element instruction
					id_instr_reads_vrfrdaddr1 = 1'b1;	// uses 1st register operand
					id_instr_reads_vrfrdaddr2 = 1'b0;	// does not use 2nd register operand
				end
				
				// Store to local memory
				`FUNC_MEM_LOCALSTORE: begin
					id_instr_reads_vrfrdaddr1 = 1'b1;	// uses 1st register operand
					// scalar broadcast store to local memory
					if (opCode[`OP_OPX] == `OPX_VSTYPE) begin
						// if scalar reg is source, it does not use vrf rdaddr
						id_instr_reads_vrfrdaddr2 = 1'b0;
						// read from the scalar to vector queue
						id_scalarToVectorQ_rdreq = 1'b1;
					end
					// vector store to local memory
					else begin
						id_instr_reads_vrfrdaddr2 = 1'b1;
						// do not read from the scalar to vector queue
						id_scalarToVectorQ_rdreq = 1'b0;
					end

					// If virtual processors are sharing one local memory, only need to store for 1 cycle
			`ifdef	LMEMSHARE
					id_instr_multiple_elems = 1'b0;		// regular multi element instruction
			`else
					id_instr_multiple_elems = 1'b1;		// regular multi element instruction
			`endif
				end
`endif

				// Vector flag load
				`FUNC_FLAGLOAD: begin
					id_lsu_instr = 1'b1;
					id_vc_vbase_rden = 1'b1;
					id_stall_if_lsu_busy = 1'b1;
					id_instr_reads_vflagrdaddr1 = 1'b0;	// instruction not masked
				end
				
				// Vector flag store
				`FUNC_FLAGSTORE: begin
					id_vrf_rdaddr2 = opCode[`OP_VRFWRADDR];
					id_flagstore = 1'b1;
					id_newInstr_store = 1'b1;
					id_lsu_instr = 1'b1;
					id_stall_if_lsu_busy = 1'b1;
					id_instr_multiple_elems = 1'b1;					// regular multi element instruction
					id_vc_vbase_rden = 1'b1;
					id_flagStoreQWrreq = 1'b1;					// write data to flag store queue
					id_vflag_rdaddr1 = opCode[`OP_VMEMWRADDR];	// read from R-type wraddr field
					id_instr_reads_vflagrdaddr1 = 1'b1;			// instruction reads vflagrdaddr1 for store data
				end
				
				default:
					;
			endcase
		end	// end VMTYPE

		
		//--------------------------------------------------			
		// Flag/Misc Instructions/Fixed point
		//--------------------------------------------------
		`OPCODE_MISCTYPE: begin
			id_instr_reads_vrfrdaddr1 = 1'b0;	// does not use 1st register operand
			id_instr_reads_vrfrdaddr2 = 1'b0;	// does not use 2nd register operand
			id_instr_reads_vflagrdaddr1 = 1'b0;	// instruction not masked
			id_instr_reads_vflagrdaddr2 = 1'b0;	// instruction not masked
			
			// Flag/Miscellaneous instructions
			if (opCode[`OP_OPX] == `OPX_MISCTYPE) begin
				// decode instruction
				casex (id_ALUOp)
					// vector-scalar flag insert
					`ALUOP_VFINS_VS: begin
						id_vctrl_rden = 1'b1;
						vc_vctrl_reg_rdaddr = `VC_VINDEX;
						vc_vctrl_offset = 0;
						id_instr_uses_vlength = 1'b0;
						id_instr_multiple_elems = 1'b0;			// multi-element per lane instruction
						id_multElemQuot_offset = 1'b1;
						id_instr_reads_vflagrdaddr1 = 1'b0;
						id_instr_reads_vflagrdaddr2 = 1'b0;
						id_scalarToVectorQ_rdreq = 1'b1;
						id_stall_if_lsu_busy = 1'b1;		// stall pipeline if LSU is busy; don't want to modify flag values when LSU may read them
					end

					// vector-scalar flag logical instructions
					`ALUOP_FLAG_VFAND_VS,
					`ALUOP_FLAG_VFNOR_VS,
					`ALUOP_FLAG_VFOR_VS,
					`ALUOP_FLAG_VFXOR_VS: begin
						id_instr_uses_vlength = 1'b1;
						id_instr_multiple_elems = 1'b1;
						id_vflag_rdaddr1 = opCode[`OP_VFLAGRDADDR1];
						id_instr_reads_vflagrdaddr1 = 1'b1;
						id_instr_reads_vflagrdaddr2 = 1'b0;
						id_scalarToVectorQ_rdreq = 1'b1;
						id_stall_if_lsu_busy = 1'b1;		// stall pipeline if LSU is busy; don't want to modify flag values when LSU may read them
					end
					
					// flag logical instructions
					`FUNC_FLAGLOGIC:	begin
						id_instr_uses_vlength = 1'b1;
						id_instr_multiple_elems = 1'b1;
						id_vflag_rdaddr1 = opCode[`OP_VFLAGRDADDR1];
						id_instr_reads_vflagrdaddr1 = 1'b1;
						id_instr_reads_vflagrdaddr2 = 1'b1;
					end
					
					// move scalar to control register
					`FUNC_MSTC:	begin
						id_instr_stalls_pipeline = 1'b1;	// stall pipeline until MSTC instruction completes
						// read from the scalar to vector queue
						id_scalarToVectorQ_rdreq = 1'b1;
						id_stall_if_lsu_busy = 1'b0;		// control register values are copied to LSU, so control register can be modified
					end
					
					// move control register to scalar register
					`FUNC_MCTS: begin
						id_mcts = 1'b1;
						id_instr_stalls_pipeline = 1'b1;	// stall pipeline until MSTC instruction completes
						id_vectorToScalarQ_wrreq = 1'b1;
						// enable all control registers
						id_vctrl_rden = 1'b1;
						id_vc_vbase_rden = 1'b1;
					end
				
					default:	;
				endcase
			end	// end if
		end	// end OPCODE_MISCTYPE
			
		default:	;
		
	endcase

end
	

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Decode signals for loadstore unit
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Determine data width of memory access; use the largest possible width
always @(*)
	// Instructions that use memory crossbar at largest possible width: flagload, flagstore, vins.vv, vext.vv
	if (((opCode[`OP_OPCODE] == `OPCODE_VMTYPE) && ((id_ALUOp == `FUNC_FLAGLOAD) || (id_ALUOp == `FUNC_FLAGSTORE))) ||
		((opCode[`OP_OPCODE] == `OPCODE_VRTYPE) && (opCode[`OP_OPX] != `OPX_VSTYPE) && ((id_ALUOp == `ALUOP_VINS) || (id_ALUOp == `ALUOP_VEXT)))) begin
	
		// use the highest possible width; the last enabled statement will assign the result
`ifdef	STORE_BYTE_ACCESS_ENABLED
		id_memDataWidth = `MEMACCESS_BYTE;
`endif
`ifdef	STORE_HALFWORD_ACCESS_ENABLED
		id_memDataWidth = `MEMACCESS_HALFWORD;
`endif
`ifdef	STORE_WORD_ACCESS_ENABLED
		id_memDataWidth = `MEMACCESS_WORD;
`endif
	end
	
	// Regular memory instruction
	else begin
		casex (opCode[`OP_FUNC_MEMWIDTH])
`ifdef	STORE_BYTE_ACCESS_ENABLED
			`FUNC_MEMBYTE: 		id_memDataWidth = `MEMACCESS_BYTE;
`endif
`ifdef	STORE_HALFWORD_ACCESS_ENABLED
			`FUNC_MEMHALFWORD:	id_memDataWidth = `MEMACCESS_HALFWORD;
`endif
`ifdef	STORE_WORD_ACCESS_ENABLED
			`FUNC_MEMWORD:		id_memDataWidth = `MEMACCESS_WORD;
`endif
			default:			id_memDataWidth = 3'bx;
		endcase
	end

endmodule
