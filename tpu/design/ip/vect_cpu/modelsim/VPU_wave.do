onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/clk
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/reset_n
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/vpu_new_instr
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/myVPU/vpu_instr_decoded
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/vpu_instr_decode_valid
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/myVPU/alu_interrupt_instr
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/vpu_control_ready
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/opCode
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/instruction_register
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/myVPU/instruction_register_q
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/state
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/gotoInterruptMode
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_instr_decode_valid
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_opCode_q
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_busy
add wave -noupdate -divider {Scalar Core}
add wave -noupdate -format Literal -radix binary /tb_VectCPU/myVectCPU/ScalarCore/the_Control_Unit/active_stage
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Control_Unit/vector_cop_scalar_skip
add wave -noupdate -group scalarcore
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/i_read
add wave -noupdate -group scalarcore -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/ScalarCore/i_address
add wave -noupdate -group scalarcore -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/ScalarCore/i_readdata
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/i_waitrequest
add wave -noupdate -group scalarcore -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_One/the_Fetch_Unit/fetched_instruction
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_One/the_Fetch_Unit/pipeline_stalled
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Control_Unit/cop_fetch_stall
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/executing_irq_stage
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Control_Unit/decode_stage_stall
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Control_Unit/execute_stage_stall
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Control_Unit/fetch_stage_stall
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Control_Unit/memory_stage_stall
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_One/the_Fetch_Unit/s_fetch
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_One/the_Fetch_Unit/vector_cop_instruction
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_One/the_Fetch_Unit/vector_cop_opx
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_One/the_Fetch_Unit/vector_cop_scalar_read
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_One/the_Fetch_Unit/vector_cop_scalar_write
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_Two/the_Pipeline_S2_Regs/s2_vector_cop_scalar_read
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_Two/the_Pipeline_S2_Regs/s2_vector_cop_scalar_write
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_Two/the_Pipeline_S2_Regs/decode_stage_completed
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_Two/the_Pipeline_S2_Regs/s3_vector_cop_scalar_read
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_Two/the_Pipeline_S2_Regs/s3_vector_cop_scalar_write
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_One/inc_pc
add wave -noupdate -group scalarcore -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/ScalarCore/vector_opcode
add wave -noupdate -group scalarcore -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/ScalarCore/scalarRegOut
add wave -noupdate -group scalarcore -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/fetched_instruction
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_One/the_Fetch_Unit/fetch_completed
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_One/the_Fetch_Unit/branch_stage
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_One/the_Fetch_Unit/branch_taken
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_One/the_Fetch_Unit/branch_type
add wave -noupdate -group scalarcore -format Literal -radix decimal /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/s3_branch_offset
add wave -noupdate -group scalarcore -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_One/the_Fetch_Unit/program_counter
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/s3_is_rtype
add wave -noupdate -group scalarcore -format Literal /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_Two/decoded_signals
add wave -noupdate -group scalarcore -format Literal /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_Two/the_Register_File/port_a_data_out
add wave -noupdate -group scalarcore -format Literal /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_Two/the_Register_File/port_a_rd_addr
add wave -noupdate -group scalarcore -format Literal /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_Two/the_Register_File/port_b_data_out
add wave -noupdate -group scalarcore -format Literal /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_Two/the_Register_File/port_b_rd_addr
add wave -noupdate -group scalarcore -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/s3_opA
add wave -noupdate -group scalarcore -format Literal -radix decimal /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/s3_opB
add wave -noupdate -group scalarcore -format Literal /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/s3_opcode
add wave -noupdate -group scalarcore -format Literal -radix unsigned /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/s3_reg_wr_addr
add wave -noupdate -group scalarcore -format Literal -radix unsigned /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/s4_reg_wr_addr
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/s4_reg_wr_en
add wave -noupdate -group scalarcore -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/ScalarCore/d_address
add wave -noupdate -group scalarcore -format Literal /tb_VectCPU/myVectCPU/ScalarCore/d_byteenable
add wave -noupdate -group scalarcore -format Literal /tb_VectCPU/myVectCPU/ScalarCore/d_data_size
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/d_read
add wave -noupdate -group scalarcore -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/ScalarCore/d_readdata
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/d_waitrequest
add wave -noupdate -group scalarcore -format Logic /tb_VectCPU/myVectCPU/ScalarCore/d_write
add wave -noupdate -group scalarcore -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/ScalarCore/d_writedata
add wave -noupdate -divider {Scalar to Vector Transfer Queue}
add wave -noupdate -group ScalarToVectorQ
add wave -noupdate -group ScalarToVectorQ -format Literal -radix decimal /tb_VectCPU/myVectCPU/scalarRegOut
add wave -noupdate -group ScalarToVectorQ -format Logic /tb_VectCPU/myVectCPU/scalarToVectorQ_empty
add wave -noupdate -group ScalarToVectorQ -format Logic /tb_VectCPU/myVectCPU/scalarToVectorQ_full
add wave -noupdate -group ScalarToVectorQ -format Literal -radix decimal /tb_VectCPU/myVectCPU/scalarQ_q
add wave -noupdate -group ScalarToVectorQ -format Logic /tb_VectCPU/myVectCPU/scalarToVectorQ_rdreq
add wave -noupdate -group ScalarToVectorQ -format Logic /tb_VectCPU/myVectCPU/scalarToVectorQ_wrreq
add wave -noupdate -group VectorToScalarQ
add wave -noupdate -group VectorToScalarQ -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/vectorToScalar_fifo/data
add wave -noupdate -group VectorToScalarQ -format Logic /tb_VectCPU/myVectCPU/vectorToScalar_fifo/empty
add wave -noupdate -group VectorToScalarQ -format Logic /tb_VectCPU/myVectCPU/vectorToScalar_fifo/full
add wave -noupdate -group VectorToScalarQ -format Literal -radix unsigned /tb_VectCPU/myVectCPU/vectorToScalar_fifo/q
add wave -noupdate -group VectorToScalarQ -format Logic /tb_VectCPU/myVectCPU/vectorToScalar_fifo/rdreq
add wave -noupdate -group VectorToScalarQ -format Logic /tb_VectCPU/myVectCPU/vectorToScalar_fifo/wrreq
add wave -noupdate -divider {VPU Controller}
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/vl_eff
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/myVPU/s3_decode_onelane_en
add wave -noupdate -format Literal /tb_VectCPU/myVectCPU/myVPU/s3_vl_lane_en
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/multElemNumCycles
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/multElemLastCycle
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/multElemVrfOffset
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/s2_vrfOffset
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/multElemVrfOffset_inc
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/myVPU/s3_opCode
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/myVPU/s4_opCode
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/cycleFirstLaneIndex
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/cycleLastEnabledLane
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/firstElem_index
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/firstElemQuot
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/firstElemRem
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/lastElemQuot
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/lastElemRem
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/s3_cycleFirstLaneIndex
add wave -noupdate -format Literal /tb_VectCPU/myVectCPU/myVPU/s3_cycleLastEnabledLane
add wave -noupdate -divider {Control Registers}
add wave -noupdate -expand -group {Control registers}
add wave -noupdate -group {Control registers} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/vc_vl
add wave -noupdate -group {Control registers} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/vc_vl_q
add wave -noupdate -group {Control registers} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/s4_vl_accvl_update
add wave -noupdate -group {Control registers} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/s4_vc_vl_wren
add wave -noupdate -group {Control registers} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/id_vc_vbase_rden
add wave -noupdate -group {Control registers} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/vc_vbase_rden
add wave -noupdate -group {Control registers} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/id_vc_vbase_rdaddr
add wave -noupdate -group {Control registers} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/s4_vc_vbase_wraddr
add wave -noupdate -group {Control registers} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/s4_vc_vbase_wren
add wave -noupdate -group {Control registers} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/s4_vbase_add_vinc
add wave -noupdate -group {Control registers} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/vc_vbase_inc_data
add wave -noupdate -group {Control registers} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/vc_vbase_q
add wave -noupdate -group {Control registers} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/s4_vbase_q
add wave -noupdate -group {Control registers} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/id_vctrl_rdaddr
add wave -noupdate -group {Control registers} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/id_vctrl_rden
add wave -noupdate -group {Control registers} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/s3_vc_vctrl_rden
add wave -noupdate -group {Control registers} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/vc_vctrl_rden
add wave -noupdate -group {Control registers} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/s3_vctrl_rdaddr
add wave -noupdate -group {Control registers} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/vc_vctrl_rdaddr
add wave -noupdate -group {Control registers} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/s4_vctrl_wren
add wave -noupdate -group {Control registers} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/s4_vctrl_wraddr
add wave -noupdate -group {Control registers} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/vctrl_data
add wave -noupdate -group {Control registers} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/vctrl_q
add wave -noupdate -divider {Hazard detection}
add wave -noupdate -group {Hazard detection}
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/alu_interrupt_instr
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/int_pending_fetch_stall
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/enter_interrupt_mode_stalled
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/vrf_raw_hazard
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/vflag_raw_hazard
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/id_instr_reads_vrfrdaddr1
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/id_instr_reads_vrfrdaddr2
add wave -noupdate -group {Hazard detection} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/s3_vrf_wraddr
add wave -noupdate -group {Hazard detection} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/s3_vrfWrOffset
add wave -noupdate -group {Hazard detection} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/s4_vrf_wraddr
add wave -noupdate -group {Hazard detection} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/s4_vrfWrOffset
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/id_instr_reads_vflagrdaddr1
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/id_instr_reads_vflagrdaddr2
add wave -noupdate -group {Hazard detection} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/id_vflag_rdaddr1
add wave -noupdate -group {Hazard detection} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/id_vflag_rdaddr2
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/dec_s3_vrf_wren
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/dec_s4_vrf_wren
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/dec_s3_vflag_wren
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/dec_s4_vflag_wren
add wave -noupdate -group {Hazard detection} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/s3_vflag_wraddr
add wave -noupdate -group {Hazard detection} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/s4_vflag_wraddr
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/lsu_busy
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/lsu_busy_decodestall
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/lsu_raw_hazard
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/indexed_memaccess_stall
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/cpu_fetch_stall
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/vpu_decodestage_stall
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/vpu_executestage_stall
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/readScalarQ_stall
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/vpu_memorystage_stall
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/nonpipelined_instr_decode_stall
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_hazard_detection_inst/id_instr_stalls_pipeline
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_pipeline_regs_inst/s3_instr_stalls_pipeline
add wave -noupdate -group {Hazard detection} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_pipeline_regs_inst/s4_instr_stalls_pipeline_done
add wave -noupdate -divider {Decode signals}
add wave -noupdate -group {Decoder signals}
add wave -noupdate -group {Decoder signals} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s3_instr_nonmaskable}
add wave -noupdate -group {Decoder signals} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/scalarRegIn}
add wave -noupdate -group {Decoder signals} -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s3_ALUOp}
add wave -noupdate -group {Decoder signals} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s2_vrfRdOffset}
add wave -noupdate -group {Decoder signals} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_decode_inst/id_vrf_rdaddr1
add wave -noupdate -group {Decoder signals} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/VPU_decode_inst/id_vrf_rdaddr2
add wave -noupdate -group {Decoder signals} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/id_vrf_rden}
add wave -noupdate -group {Decoder signals} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s4_vrf_wren}
add wave -noupdate -group {Decoder signals} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/id_vflag_rdaddr1}
add wave -noupdate -group {Decoder signals} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/id_vflag_rdaddr2}
add wave -noupdate -group {Decoder signals} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s4_vflag_wren}
add wave -noupdate -group {Decoder signals} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s3_vpmem_wren}
add wave -noupdate -group {Decoder signals} -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/id_vc_vbase_rdaddr
add wave -noupdate -group {Decoder signals} -format Literal /tb_VectCPU/myVectCPU/myVPU/VPU_decode_inst/id_ALUOp
add wave -noupdate -group {Decoder signals} -format Logic /tb_VectCPU/myVectCPU/myVPU/VPU_controller_inst/id_indexed_memaccess
add wave -noupdate -divider {Datapath signals}
add wave -noupdate -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/laneStoreQRdreq
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/myVPU/s3_laneStoreQWrreq
add wave -noupdate -format Literal /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/flagStoreQEmpty_group
add wave -noupdate -format Literal /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/laneStoreQFull
add wave -noupdate -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_storeDataQ_inst/data}
add wave -noupdate -group {VLane_storeDataQ usedw}
add wave -noupdate -group {VLane_storeDataQ usedw} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_storeDataQ_inst/usedw}
add wave -noupdate -group {VLane_storeDataQ usedw} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[1]/genblk1/VLane_inst/VLane_storeDataQ_inst/usedw}
add wave -noupdate -group {VLane_storeDataQ usedw} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[2]/genblk1/VLane_inst/VLane_storeDataQ_inst/usedw}
add wave -noupdate -group {VLane_storeDataQ usedw} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[3]/genblk1/VLane_inst/VLane_storeDataQ_inst/usedw}
add wave -noupdate -group {VLane_storeDataQ usedw} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[4]/genblk1/VLane_inst/VLane_storeDataQ_inst/usedw}
add wave -noupdate -group {VLane_storeDataQ usedw} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[5]/genblk1/VLane_inst/VLane_storeDataQ_inst/usedw}
add wave -noupdate -group {VLane_storeDataQ usedw} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[6]/genblk1/VLane_inst/VLane_storeDataQ_inst/usedw}
add wave -noupdate -group {VLane_storeDataQ usedw} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[7]/genblk1/VLane_inst/VLane_storeDataQ_inst/usedw}
add wave -noupdate -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_storeFlagQ_inst/rdreq}
add wave -noupdate -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_storeFlagQ_inst/wrreq}
add wave -noupdate -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_storeFlagQ_inst/data}
add wave -noupdate -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_storeFlagQ_inst/usedw}
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/indexOffset_out
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/myVPU/indexOffsetMUX_sel
add wave -noupdate -format Literal /tb_VectCPU/myVectCPU/myVPU/s4_vflagWriteSel
add wave -noupdate -expand -group {VLane[0] Datapath}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s3_scalarRegInSel}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s3_loadDataQToVReg}
add wave -noupdate -group {VLane[0] Datapath} -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s3_ALUOp}
add wave -noupdate -group {VLane[0] Datapath} -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s4_ALUOp}
add wave -noupdate -group {VLane[0] Datapath} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s3_vrf_q1}
add wave -noupdate -group {VLane[0] Datapath} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s3_vrf_q2}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/s3_vflag_q1}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/s3_vflag_q2}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s3_vflag}
add wave -noupdate -group {VLane[0] Datapath} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/ALU_inst/inA}
add wave -noupdate -group {VLane[0] Datapath} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/ALU_inst/inB}
add wave -noupdate -group {VLane[0] Datapath} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/ALU_inst/inB_x2}
add wave -noupdate -group {VLane[0] Datapath} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/ALU_inst/adderInB}
add wave -noupdate -group {VLane[0] Datapath} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/ALU_inst/adderOut}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/ALU_inst/adderOverflow}
add wave -noupdate -group {VLane[0] Datapath} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/ALU_inst/adder2Out}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/ALU_inst/adder2Overflow}
add wave -noupdate -group {VLane[0] Datapath} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/ALU_inst/ALUResult}
add wave -noupdate -group {VLane[0] Datapath} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/shiftResult}
add wave -noupdate -group {VLane[0] Datapath} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s4_vrfWrOffset}
add wave -noupdate -group {VLane[0] Datapath} -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s4_vrf_wraddr}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s4_vrf_wren}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/flagunit_inst/flagInA}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/flagunit_inst/flagInB}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/s4_vflagLogicOut}
add wave -noupdate -group {VLane[0] Datapath} -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s4_vflag_wraddr}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s4_vflag_wren}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s3_vpmem_wren}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s3_signedOp}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s4_signedOp}
add wave -noupdate -group {VLane[0] Datapath} -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s4_vrfWriteSel}
add wave -noupdate -group {VLane[0] Datapath} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/extLoad_lane_we}
add wave -noupdate -group {VLane[0] Datapath} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/extLoadData}
add wave -noupdate -group {VLane[0] Datapath} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/shiftInNext}
add wave -noupdate -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/vrf_data}
add wave -noupdate -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/lanemult_mf_inst/dataa_0}
add wave -noupdate -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/lanemult_mf_inst/datab_0}
add wave -noupdate -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/lanemult_mf_inst/signa}
add wave -noupdate -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/lanemult_mf_inst/result}
add wave -noupdate -group {VLane[0] shifter}
add wave -noupdate -group {VLane[0] shifter} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/shifter_inst/datain}
add wave -noupdate -group {VLane[0] shifter} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/shifterdecode/s4_shift_instr}
add wave -noupdate -group {VLane[0] shifter} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/shifterdecode/s4_shiftAmount}
add wave -noupdate -group {VLane[0] shifter} -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/shifter_inst/stageShift}
add wave -noupdate -group {VLane[0] shifter} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/shifter_inst/logicalMask}
add wave -noupdate -group {VLane[0] shifter} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/shifter_inst/arithShift}
add wave -noupdate -group {VLane[0] shifter} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/shifter_inst/dataout}
add wave -noupdate -group {VLane[0] shifter} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/shifter_inst/shiftin}
add wave -noupdate -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/shifterdecode/s4_shift_command}
add wave -noupdate -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/shifterdecode/s4_shiftAmount_right}
add wave -noupdate -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/shifterdecode/shiftAmount_addinA}
add wave -noupdate -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/shifterdecode/shiftAmount_addinB}
add wave -noupdate -group {s4_vrf_wren group}
add wave -noupdate -group {s4_vrf_wren group} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s4_vrf_wren}
add wave -noupdate -group {s4_vrf_wren group} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[1]/genblk1/VLane_inst/s4_vrf_wren}
add wave -noupdate -group {s4_vrf_wren group} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[2]/genblk1/VLane_inst/s4_vrf_wren}
add wave -noupdate -group {s4_vrf_wren group} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[3]/genblk1/VLane_inst/s4_vrf_wren}
add wave -noupdate -group {s4_vflag_wren group}
add wave -noupdate -group {s4_vflag_wren group} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/s4_vflag_wren}
add wave -noupdate -group {s4_vflag_wren group} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[1]/genblk1/VLane_inst/s4_vflag_wren}
add wave -noupdate -group {s4_vflag_wren group} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[2]/genblk1/VLane_inst/s4_vflag_wren}
add wave -noupdate -group {s4_vflag_wren group} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[3]/genblk1/VLane_inst/s4_vflag_wren}
add wave -noupdate -group {Vlane_loadDataQ [0]}
add wave -noupdate -group {Vlane_loadDataQ [0]} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/loadDataQ_full}
add wave -noupdate -group {Vlane_loadDataQ [0]} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_loadDataQ_inst/data}
add wave -noupdate -group {Vlane_loadDataQ [0]} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_loadDataQ_inst/empty}
add wave -noupdate -group {Vlane_loadDataQ [0]} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_loadDataQ_inst/full}
add wave -noupdate -group {Vlane_loadDataQ [0]} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_loadDataQ_inst/q}
add wave -noupdate -group {Vlane_loadDataQ [0]} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_loadDataQ_inst/rdreq}
add wave -noupdate -group {Vlane_loadDataQ [0]} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_loadDataQ_inst/wrreq}
add wave -noupdate -group {Vlane_loadDataQ [0]} -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_loadDataQ_inst/usedw}
add wave -noupdate -group {Vlane_loadDataQ [1]}
add wave -noupdate -group {Vlane_loadDataQ [1]} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[1]/genblk1/VLane_inst/VLane_loadDataQ_inst/data}
add wave -noupdate -group {Vlane_loadDataQ [1]} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[1]/genblk1/VLane_inst/VLane_loadDataQ_inst/empty}
add wave -noupdate -group {Vlane_loadDataQ [1]} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[1]/genblk1/VLane_inst/VLane_loadDataQ_inst/full}
add wave -noupdate -group {Vlane_loadDataQ [1]} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[1]/genblk1/VLane_inst/VLane_loadDataQ_inst/q}
add wave -noupdate -group {Vlane_loadDataQ [1]} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[1]/genblk1/VLane_inst/VLane_loadDataQ_inst/rdreq}
add wave -noupdate -group {Vlane_loadDataQ [1]} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[1]/genblk1/VLane_inst/VLane_loadDataQ_inst/wrreq}
add wave -noupdate -group {Vlane_loadDataQ [1]} -format Literal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[1]/genblk1/VLane_inst/VLane_loadDataQ_inst/usedw}
add wave -noupdate -group Vlane_FlagloadDataQ
add wave -noupdate -group Vlane_FlagloadDataQ -format Logic -radix binary {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[3]/genblk1/VLane_inst/VLane_loadDataQ_inst/q[0]}
add wave -noupdate -group Vlane_FlagloadDataQ -format Logic -radix binary {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[2]/genblk1/VLane_inst/VLane_loadDataQ_inst/q[0]}
add wave -noupdate -group Vlane_FlagloadDataQ -format Logic -radix binary {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[1]/genblk1/VLane_inst/VLane_loadDataQ_inst/q[0]}
add wave -noupdate -group Vlane_FlagloadDataQ -format Logic -radix binary {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_loadDataQ_inst/q[0]}
add wave -noupdate -group {vflag Write Result}
add wave -noupdate -group {vflag Write Result} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/vflagResult}
add wave -noupdate -group {vflag Write Result} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[1]/genblk1/VLane_inst/VLane_dp_inst/vflagResult}
add wave -noupdate -group {vflag Write Result} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[2]/genblk1/VLane_inst/VLane_dp_inst/vflagResult}
add wave -noupdate -group {vflag Write Result} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[3]/genblk1/VLane_inst/VLane_dp_inst/vflagResult}
add wave -noupdate -group {vflag Write Result} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[4]/genblk1/VLane_inst/VLane_dp_inst/vflagResult}
add wave -noupdate -group {vflag Write Result} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[5]/genblk1/VLane_inst/VLane_dp_inst/vflagResult}
add wave -noupdate -group {vflag Write Result} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[6]/genblk1/VLane_inst/VLane_dp_inst/vflagResult}
add wave -noupdate -group {vflag Write Result} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[7]/genblk1/VLane_inst/VLane_dp_inst/vflagResult}
add wave -noupdate -expand -group {MAC Chain}
add wave -noupdate -group {MAC Chain} -format Logic /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/MACunit_inst/id_signedOp
add wave -noupdate -group {MAC Chain} -format Logic /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/MACunit_inst/id_zeroACC
add wave -noupdate -group {MAC Chain} -format Logic /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/MACunit_inst/s3_zeroACC
add wave -noupdate -group {MAC Chain} -format Logic /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/MACunit_inst/s4_MAC_en
add wave -noupdate -group {MAC Chain} -format Logic /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/MACunit_inst/s4_zeroMACchainout
add wave -noupdate -group {MAC Chain} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/MACunit_inst/vrf_q1_group
add wave -noupdate -group {MAC Chain} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/MACunit_inst/vrf_q2_group
add wave -noupdate -group {MAC Chain} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/MACunit_inst/vrf_q1_group_zeroed
add wave -noupdate -group {MAC Chain} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/MACunit_inst/vrf_q2_group_zeroed
add wave -noupdate -group {MAC Chain} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/MACunit_inst/macResultToLane
add wave -noupdate -group {MAC Chain} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/MACunit_inst/macResult_group
add wave -noupdate -group {MAC Chain} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/MACunit_inst/mac_chainin_group
add wave -noupdate -group {VPLocalMem[0]}
add wave -noupdate -group {VPLocalMem[0]} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/vpmem_rdaddr}
add wave -noupdate -group {VPLocalMem[0]} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/vpmem_q}
add wave -noupdate -group {VPLocalMem[0]} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/s3_vpmem_rden}
add wave -noupdate -group {VPLocalMem[0]} -format Literal -radix unsigned {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/vpmem_wraddr}
add wave -noupdate -group {VPLocalMem[0]} -format Literal -radix hexadecimal {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/vpmem_data}
add wave -noupdate -group {VPLocalMem[0]} -format Logic {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/s3_vpmem_wren}
add wave -noupdate -divider {Load/store Controller}
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_opCode_q
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/state
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_busy
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/load_done
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/store_done
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/id_vl_minus_one
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_vl_minus_one
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/lsu_vl_special_instr
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/lsu_load_vl_eff_minus_one
add wave -noupdate -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/lsu_store_vl_eff_minus_one
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/lsu_flagload
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/lsu_vext_vv
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/lsu_vins_vv
add wave -noupdate -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_data_waiting_flag
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/ScalarCore/predecode_scalarmem_instr
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/ScalarCore/predecode_vectormem_instr
add wave -noupdate -format Literal /tb_VectCPU/myVectCPU/ScalarCore/d_data_size
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/mem_order_q_inst/empty
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/mem_order_q_inst/wrreq
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/mem_order_q_inst/data
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/mem_order_q_inst/rdreq
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/mem_order_q_inst/q
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_wraddr_q_empty
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_wraddr_q_rdreq
add wave -noupdate -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_wraddr_q_data
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_wraddr_q_wrreq
add wave -noupdate -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_wraddr_q_q
add wave -noupdate -group LoadstoreController
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/scalar_read_grant
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/scalar_write_grant
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/vector_store_grant
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/executing_scalar_store
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/executing_vector_store
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/executing_scalar_load
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/executing_vector_load
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/d_waitrequest
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/scalar_d_read
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/scalar_d_write
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_unitstride
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_indexed_memaccess
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_signedOp
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_unitstride
add wave -noupdate -group LoadstoreController -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_memDataWidth
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_indexed_memaccess
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_signedOp
add wave -noupdate -group LoadstoreController -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_vrf_wraddr
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_scalar_load
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_newInstr_load
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_newInstr_store
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/store_done_posedge
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/vectorload_waitrequest
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/vectorstore_waitrequest
add wave -noupdate -group LoadstoreController -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadstoreController_inst/lsu_flagload
add wave -noupdate -divider {Load address generator}
add wave -noupdate -group LoadAddrGen
add wave -noupdate -group LoadAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/start_newLoad
add wave -noupdate -group LoadAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/load_done
add wave -noupdate -group LoadAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/pauseLoadController
add wave -noupdate -group LoadAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/baseAddr
add wave -noupdate -group LoadAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/vector_load_inc_rdaddr
add wave -noupdate -group LoadAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/baseAligned_scalar
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/lsu_indexOffset_lanesel
add wave -noupdate -group LoadAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/indexOffset_out
add wave -noupdate -group LoadAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/nextMemIncByteAligned
add wave -noupdate -group LoadAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/nextMemAddr
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/vindex
add wave -noupdate -group LoadAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/s3_unitstride
add wave -noupdate -group LoadAddrGen -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/s3_memDataWidth
add wave -noupdate -group LoadAddrGen -format Literal -radix decimal /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/stride
add wave -noupdate -group LoadAddrGen -format Literal -radix decimal /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/localStride
add wave -noupdate -group LoadAddrGen -format Literal -radix decimal /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/strideByte
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/strideTrunc
add wave -noupdate -group LoadAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/largeStride
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/lsu_vectorlength_minus_one
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/offsetByte_scalar
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/loadElemCount
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/loadElemCount_preg
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/loadElemCount_reverse
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/numActiveLanes
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/numActiveLanes_preg
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/maxNElemXfer_const
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/divider_rom_addr
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/firstElemOffset
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/offsetElemToSubtract
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/maxNElemXfer_offsetted
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/s4_NElemRemainder
add wave -noupdate -group LoadAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/s4_NElemXfer_rounded
add wave -noupdate -group LoadAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/readXbarSel_sig
add wave -noupdate -group LoadAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/readXbarSel
add wave -noupdate -group LoadAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/data_lane_we
add wave -noupdate -group LoadAddrGen -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/tempinc
add wave -noupdate -group LoadAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/vector_manip_loading
add wave -noupdate -group LoadAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/loadAddrGen_inst/vector_manip_loading_delay
add wave -noupdate -group LoadAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/myVPU/memLoadDataToLanes
add wave -noupdate -divider {Memread IF}
add wave -noupdate -expand -group MemreadIF
add wave -noupdate -group MemreadIF -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memreadIF_inst/readXbarSel
add wave -noupdate -group MemreadIF -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/memreadIF_inst/lsu_memDataWidth
add wave -noupdate -group MemreadIF -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/memreadIF_inst/lsu_signedOp
add wave -noupdate -group MemreadIF -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/memreadIF_inst/lsu_flagload
add wave -noupdate -group MemreadIF -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memreadIF_inst/scalar_d_readdata
add wave -noupdate -group MemreadIF -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memreadIF_inst/readXbarOut
add wave -noupdate -group MemreadIF -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/memreadIF_inst/dataElemSel
add wave -noupdate -group MemreadIF -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memreadIF_inst/memLoadDataToLanes
add wave -noupdate -group MemreadIF -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memreadIF_inst/memLoadFlagToLanes
add wave -noupdate -divider {Store address generator}
add wave -noupdate -group storeAddrGen
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/newInstr_store
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/VLaneStoreQReady
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/d_waitrequest
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/lanesideState
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/writesideState
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/store_done
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/triggerLoadNextGroup
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/triggerLoadNextGroup_posedge
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/loadNextLaneDataGroup
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/writeRegDataToMem
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/laneDataLoadCount_const
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/laneDataLoadCount
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/baseAddr
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/indexOffset_sext
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/baseAddrIndexAdded
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/baseAddrScalarSel
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/lsu_unitstride
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/stride
add wave -noupdate -group storeAddrGen -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/lsu_memDataWidth
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/totalNumElem_minus_one
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/laneSelCount
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/laneSelCount_inc
add wave -noupdate -group storeAddrGen -format Literal /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/laneStoreQFull
add wave -noupdate -group storeAddrGen -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/laneStoreQRdreq
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/flagStoreQRdreq
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/indexOffsetCount
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/totalWriteCount
add wave -noupdate -group storeAddrGen -format Literal -radix binary /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/laneWrselMUXSel
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/clusterWriteCount
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/nextCycleIs_clusterWriteFirstCycle
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/clusterWriteFirstCycle
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/clusterWriteDone
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/elemOffsetByte_group_compare
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/elemOffsetByte_group_comparereg
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/memWralignXbarSel
add wave -noupdate -group storeAddrGen -format Literal /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/VLaneFlagToMem
add wave -noupdate -group storeAddrGen -format Literal /tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/s3_vmask_group
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/alignedDataReg_datawr_subword_en
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/alignedDataReg_datawr_bit_en
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/alignedDataReg_en
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/alignedDataReg_flagwr_en
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/alignedDataReg_flagwr_bit_en
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/alignedFlagReg_clr
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_fillreg_inst/alignedFlagReg_en
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/alignedFlagRegQ
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/lastElemMemgroupPos
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/alignedFlagReg_clr
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/subwordFlag_group
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/subwordOffsetValid
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/delaySubword_en
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/delaySubwordSel
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/memgroupWriteBytes
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/gotoWriteDelayCycle
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/mem_addr_0
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/nextMemAddrSig
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/vector_mem_dm
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/baseAligned
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/localStride
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/localStrideByte
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/nextMemIncByteLargeStride
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/nextMemAddrIncByte
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/offsetByte
add wave -noupdate -group storeAddrGen -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/largeStride
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/currentOffsetByte
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/nextOffsetByte
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/blockOffsetByte
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/offsetAddStrideByte_group
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/elemOffsetByte_group
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/subword_decode_delayed
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/XbarSelDecodeOnehot
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/offsetAddStrideByte_group_reg
add wave -noupdate -group storeAddrGen -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/scalar_combine_subword_decode
add wave -noupdate -group storeAddrGen -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/scalar_combine_subword_decode_pipeline
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/subwordOffsetByte_group_minwidth
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/subwordOffsetByte_group_pipeline
add wave -noupdate -group storeAddrGen -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/XbarSelDecodeMask_select_pipeline
add wave -noupdate -group storeAddrGen -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/XbarSelDecodeMask
add wave -noupdate -group storeAddrGen -format Literal -radix unsigned /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/totalWriteCount_sig
add wave -noupdate -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/storeCtr_writemem_inst/scalar_combine_subword_decode
add wave -noupdate -divider {Memory Write IF}
add wave -noupdate -group {Memwrite IF}
add wave -noupdate -group {Memwrite IF} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memwriteIF_inst/laneDataIn
add wave -noupdate -group {Memwrite IF} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memwriteIF_inst/laneWrselMUXIn
add wave -noupdate -group {Memwrite IF} -format Literal /tb_VectCPU/myVectCPU/dmemIF_inst/memwriteIF_inst/laneWrselMUXSel
add wave -noupdate -group {Memwrite IF} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memwriteIF_inst/laneWrselMUXOut
add wave -noupdate -group {Memwrite IF} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memwriteIF_inst/memWidthCompressed
add wave -noupdate -group {Memwrite IF} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memwriteIF_inst/alignedDataReg_en
add wave -noupdate -group {Memwrite IF} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memwriteIF_inst/delaySubword_en
add wave -noupdate -group {Memwrite IF} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memwriteIF_inst/delaySubwordSel
add wave -noupdate -group {Memwrite IF} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memwriteIF_inst/memgroupRegOut
add wave -noupdate -group {Memwrite IF} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memwriteIF_inst/delayOut
add wave -noupdate -group {Memwrite IF} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memwriteIF_inst/alignXbarDataIn_pipeline
add wave -noupdate -group {Memwrite IF} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memwriteIF_inst/memWralignXbarSel
add wave -noupdate -group {Memwrite IF} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memwriteIF_inst/alignXbarDataOut
add wave -noupdate -group {Memwrite IF} -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/memwriteIF_inst/memWrdata
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/dmemIF_inst/storeAddrGen_inst/vmanip_storeload_sync
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/dmemIF_inst/storetoload_bypass
add wave -noupdate -divider {Signals to/from memory}
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/i_read
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/i_address
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/i_readdata
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/i_waitrequest
add wave -noupdate -format Literal /tb_VectCPU/myVectCPU/d_irq
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/d_readdata
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/d_waitrequest
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/d_byteenable
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/d_writedata
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/myVectCPU/d_address
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/d_read
add wave -noupdate -format Logic /tb_VectCPU/myVectCPU/d_write
add wave -noupdate -format Literal -radix hexadecimal /tb_VectCPU/theDataMemory/address_word
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {9820000 ps} 0} {{Cursor 5} {140000 ps} 0} {{Cursor 3} {3869865 ps} 0}
configure wave -namecolwidth 247
configure wave -valuecolwidth 73
configure wave -justifyvalue left
configure wave -signalnamewidth 4
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
update
WaveRestoreZoom {31488 ps} {288992 ps}
