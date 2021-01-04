## Generated SDC file "VectCPU_sys.sdc"

## Copyright (C) 1991-2007 Altera Corporation
## Your use of Altera Corporation's design tools, logic functions 
## and other software and tools, and its AMPP partner logic 
## functions, and any output files from any of the foregoing 
## (including device programming or simulation files), and any 
## associated documentation or information are expressly subject 
## to the terms and conditions of the Altera Program License 
## Subscription Agreement, Altera MegaCore Function License 
## Agreement, or other applicable license agreement, including, 
## without limitation, that your use is for the sole purpose of 
## programming logic devices manufactured by Altera and sold by 
## Altera or its authorized distributors.  Please refer to the 
## applicable agreement for further details.


## VENDOR  "Altera"
## PROGRAM "Quartus II"
## VERSION "Version 7.2 Build 203 02/05/2008 Service Pack 2 SJ Full Version"

## DATE    "Tue May 06 22:12:37 2008"

##
## DEVICE  "EP3SL340F1517C3"
##


#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3



#**************************************************************
# Create Clock
#**************************************************************

# 110MHz
#create_clock -name {clk} -period 9.091 -waveform { 0.000 4.545 } [get_ports {clk}] -add
# 115MHz
# 120MHz
create_clock -name {clk} -period 8.333 -waveform { 0.000 4.167 } [get_ports {clk}] -add


#**************************************************************
# Create Generated Clock
#**************************************************************



#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************



#**************************************************************
# Set Input Delay
#**************************************************************



#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************



#**************************************************************
# Set False Path
#**************************************************************

set_false_path  -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|state.ScalarRead}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}]
set_false_path  -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|state.ScalarWriteWait}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}]
set_false_path  -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|state.ScalarWrite}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}]
set_false_path  -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|VPU_top:myVPU|VLane_group:VLane_group_inst|VLane_top:lanei[8].VLane_inst|VLane_datapath:VLane_dp_inst|altsyncram:vregfile_mem_rtl_*|altsyncram_uri1:auto_generated|ram_block1a0~portb_address_reg*}]  -through [get_pins {the_pio|always0*|combout}] -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|UT_II_Economy_cpu:ScalarCore|Datapath:the_Datapath|Stage_One:Stage_One|Instruction_Fetch_Unit:the_Fetch_Unit|Program_Counter:the_Program_Counter|program_counter[*]}]
set_false_path  -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|*}]  -through [get_pins {the_pio|always0*|combout}] -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|*}]
set_false_path  -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|*}]  -through [get_pins {the_Soft_Vector_Processor_inst_data_master|r_0*|combout}] -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|*}]


#**************************************************************
# Set Multicycle Path
#**************************************************************

set_multicycle_path -setup -start -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|pipeline_reg_sclr:loadstore_opCode|q_reg[*]}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 2
set_multicycle_path -hold -start -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|pipeline_reg_sclr:loadstore_opCode|q_reg[*]}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 1
set_multicycle_path -setup -start -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|lsu_memDataWidth[*]}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 2
set_multicycle_path -hold -start -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|lsu_memDataWidth[*]}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 1
set_multicycle_path -setup -start -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|pipeline_reg_sclr:loadstore_opCode|q_reg[*]}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|gotoWriteDelayCycle}] 2
set_multicycle_path -hold -start -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|pipeline_reg_sclr:loadstore_opCode|q_reg[*]}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|gotoWriteDelayCycle}] 1
set_multicycle_path -setup -start -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|ff:loadstore_vl_special_instr|q_reg[*]}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 2
set_multicycle_path -hold -start -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|ff:loadstore_vl_special_instr|q_reg[*]}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 1
set_multicycle_path -setup -start -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|ff:loadstore_vl_minus_one|q_reg[*]}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 2
set_multicycle_path -hold -start -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|ff:loadstore_vl_minus_one|q_reg[*]}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 1
set_multicycle_path -setup -start -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|pipeline_reg_sclr:loadstore_vstride|q_reg[*]}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 2
set_multicycle_path -hold -start -from  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|pipeline_reg_sclr:loadstore_vstride|q_reg[*]}]  -to  [get_keepers {Soft_Vector_Processor_inst:the_Soft_Vector_Processor_inst|Soft_Vector_Processor:the_Soft_Vector_Processor|VectCPU:the_VectCPU|memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 1


#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************



#**************************************************************
# Set Input Transition
#**************************************************************



#**************************************************************
# Set Load
#**************************************************************

