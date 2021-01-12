# TCL File Generated by Component Editor 7.2 on:
# Fri Jan 11 18:01:44 PST 2008
# DO NOT MODIFY

set_source_file "Soft_Vector_Processor.v"
set_module "Soft_Vector_Processor"
set_module_description "Soft vector processor with configurable performance and features"
set_module_property "author" "Jason Yu"
set_module_property "className" "Soft_Vector_Processor"
set_module_property "displayName" "Soft Vector Processor"
set_module_property "group" "Processors"
set_module_property "simulationFiles" [ list "modelsim/control_reg_other.dat" "modelsim/decoder_memory.dat" "modelsim/NElemRemainder_rom.dat" "modelsim/NElemXfer_rom.dat" "modelsim/reg_memory.dat" ]
set_module_property "synthesisFiles" [ list "Soft_Vector_Processor.v" "hdl/altMultAccum.v" "hdl/ALU.v" "hdl/components.v" "hdl/config_def.v" "hdl/config_def_auto.v" "hdl/define.v" "hdl/flagunit.v" "hdl/isa_def.v" "hdl/lanemult_mf.v" "hdl/loadAddrGenerator.v" "hdl/loadstoreController.v" "hdl/MACunit.v" "hdl/mathmacros.v" "hdl/memIF.v" "hdl/memreadIF.v" "hdl/memwriteIF.v" "hdl/memxbar.v" "hdl/scalarToVectorQ.v" "hdl/shiftdecode.v" "hdl/shifter.v" "hdl/storeAddrGen.v" "hdl/storeCtr_fillreg.v" "hdl/storeCtr_writemem.v" "hdl/VectCPU.v" "hdl/vectorToScalarQ_data_sel.v" "hdl/VLane_controller.v" "hdl/VLane_datapath.v" "hdl/vlane_enable_decode.v" "hdl/VLane_group.v" "hdl/VLane_loadstoreDataQ.v" "hdl/VLane_storeFlagQ.v" "hdl/VLane_top.v" "hdl/VPU_controller.v" "hdl/VPU_controlreg.v" "hdl/VPU_decode.v" "hdl/VPU_decode_s3.v" "hdl/VPU_decode_s4.v" "hdl/VPU_hazard_detection.v" "hdl/VPU_pipeline_regs.v" "hdl/VPU_predecode.v" "hdl/VPU_top.v" "../utiie_cpu/hdl/Adder_w_Comparator.v" "../utiie_cpu/hdl/ALU_Logic.v" "../utiie_cpu/hdl/Branch_Logic.v" "../utiie_cpu/hdl/Control_Registers.v" "../utiie_cpu/hdl/Control_Unit.v" "../utiie_cpu/hdl/Datapath.v" "../utiie_cpu/hdl/Decoder_Logic.v" "../utiie_cpu/hdl/Instr_Register.v" "../utiie_cpu/hdl/Instruction_Fetch_Unit.v" "../utiie_cpu/hdl/isa_def.v" "../utiie_cpu/hdl/Logic_Unit.v" "../utiie_cpu/hdl/Memory_Address_Align.v" "../utiie_cpu/hdl/Memory_Byteenable.v" "../utiie_cpu/hdl/Memory_Data_In_Align.v" "../utiie_cpu/hdl/Memory_Data_Out_Align.v" "../utiie_cpu/hdl/Next_PC_Shifter_Selection.v" "../utiie_cpu/hdl/OpA_Selection.v" "../utiie_cpu/hdl/OpB_Imm_Selection.v" "../utiie_cpu/hdl/Pipeline_Reg.v" "../utiie_cpu/hdl/Pipeline_S2_Regs.v" "../utiie_cpu/hdl/Pipeline_S3_Regs.v" "../utiie_cpu/hdl/Program_Counter.v" "../utiie_cpu/hdl/Reg_Write_Addr_Mux.v" "../utiie_cpu/hdl/Reg_Write_Data_Mux.v" "../utiie_cpu/hdl/Reg_Write_Enable.v" "../utiie_cpu/hdl/Register_File.v" "../utiie_cpu/hdl/Shifter_Unit.v" "../utiie_cpu/hdl/Stage_Four.v" "../utiie_cpu/hdl/Stage_One.v" "../utiie_cpu/hdl/Stage_Three.v" "../utiie_cpu/hdl/Stage_Two.v" "../utiie_cpu/hdl/UT_II_Economy_cpu.v" "../vect_cpu/hdl/mem_order_q.v" "../vect_cpu/hdl/lsu_wraddr_q.v" "../vect_cpu/hdl/vflagreg_mf.v" "../utiie_cpu/hdl/Predecode_Instruction.v"]

# Module parameters
add_parameter "RESET_ADDRESS" "integer" "0" "CPU reset address"
add_parameter "BREAK_ADDRESS" "integer" "0" "CPU break address"
add_parameter "EXCEPTION_ADDRESS" "integer" "32" "CPU exception address"
add_parameter "CPU_ID" "integer" "3" "CPU ID"
add_parameter "VPU_WIDTH" "integer" "32" "Vector processor data width (8, 16, 32)"
add_parameter "NUMLANES" "integer" "8" "Number of vector lanes (minimum 4; power of 2)"
add_parameter "MEM_WIDTH" "integer" "128" "Width of external memory interface"
add_parameter "MINDATAWIDTH" "integer" "8" "Minimum addressable data width (8, 16, 32; max VPU_WIDTH)"

# Interface clockreset
add_interface "clockreset" "clock" "sink" "asynchronous"
# Ports in interface clockreset
add_port_to_interface "clockreset" "csi_clockreset_clk" "clk"
add_port_to_interface "clockreset" "csi_clockreset_reset_n" "reset_n"

# Interface receiver
add_interface "receiver" "interrupt" "receiver" "clockreset"
set_interface_property "receiver" "irqScheme" "INDIVIDUAL_REQUESTS"
set_interface_property "receiver" "associatedAddressablePoint" "data_master"
# Ports in interface receiver
add_port_to_interface "receiver" "inr_receiver_irq" "irq"

# Interface instruction_master
add_interface "instruction_master" "avalon" "master" "clockreset"
set_interface_property "instruction_master" "burstOnBurstBoundariesOnly" "false"
set_interface_property "instruction_master" "doStreamReads" "false"
set_interface_property "instruction_master" "linewrapBursts" "false"
set_interface_property "instruction_master" "doStreamWrites" "false"
# Ports in interface instruction_master
add_port_to_interface "instruction_master" "avm_instruction_master_readdata" "readdata"
add_port_to_interface "instruction_master" "avm_instruction_master_waitrequest" "waitrequest"
add_port_to_interface "instruction_master" "avm_instruction_master_address" "address"
add_port_to_interface "instruction_master" "avm_instruction_master_read" "read"

# Interface data_master
add_interface "data_master" "avalon" "master" "clockreset"
set_interface_property "data_master" "burstOnBurstBoundariesOnly" "false"
set_interface_property "data_master" "doStreamReads" "false"
set_interface_property "data_master" "linewrapBursts" "false"
set_interface_property "data_master" "doStreamWrites" "false"
# Ports in interface data_master
add_port_to_interface "data_master" "avm_data_master_waitrequest" "waitrequest"
add_port_to_interface "data_master" "avm_data_master_byteenable" "byteenable"
add_port_to_interface "data_master" "avm_data_master_address" "address"
add_port_to_interface "data_master" "avm_data_master_read" "read"
add_port_to_interface "data_master" "avm_data_master_write" "write"
add_port_to_interface "data_master" "avm_data_master_readdata" "readdata"
add_port_to_interface "data_master" "avm_data_master_writedata" "writedata"