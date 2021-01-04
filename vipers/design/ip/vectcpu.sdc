## Generated SDC file "vectcpu.sdc"

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
## VERSION "Version 7.2 Build 151 09/26/2007 SJ Full Version"

## DATE    "Wed Feb 06 22:47:15 2008"

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

create_clock -name {clk} -period 5.000 -waveform { 0.000 2.500 } [get_ports {clk}] -add


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

set_false_path  -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|state.ScalarRead}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}]
set_false_path  -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|state.ScalarWriteWait}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}]
set_false_path  -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|state.ScalarWrite}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}]


#**************************************************************
# Set Multicycle Path
#**************************************************************

set_multicycle_path -setup -start -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|ff:lsu_memDataWidth_preg|q_reg[2]}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[3]}] 2
set_multicycle_path -setup -start -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|pipeline_reg_sclr:loadstore_opCode|q_reg[12]}]  -through [get_nets {dmemIF_inst|storeAddrGen_inst|storeCtr_writemem_inst|Mult2|*}] -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[3]}] 2
set_multicycle_path -setup -start -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|pipeline_reg_sclr:loadstore_opCode|q_reg[*]}]  -through [get_nets {dmemIF_inst|storeAddrGen_inst|storeCtr_writemem_inst|Mult2|*}] -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 2
set_multicycle_path -hold -start -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|pipeline_reg_sclr:loadstore_opCode|q_reg[*]}]  -through [get_nets {dmemIF_inst|storeAddrGen_inst|storeCtr_writemem_inst|Mult2|*}] -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 1
set_multicycle_path -setup -start -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|ff:lsu_memDataWidth_preg|q_reg[*]}]  -through [get_nets {dmemIF_inst|storeAddrGen_inst|storeCtr_writemem_inst|Mult2|*}] -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 2
set_multicycle_path -hold -start -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|ff:lsu_memDataWidth_preg|q_reg[*]}]  -through [get_nets {dmemIF_inst|storeAddrGen_inst|storeCtr_writemem_inst|Mult2|*}] -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 1
set_multicycle_path -setup -start -from  [get_keepers *]  -through [get_nets {dmemIF_inst|storeAddrGen_inst|storeCtr_writemem_inst|Mult2|*}] -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 2
set_multicycle_path -hold -start -from  [get_keepers *]  -through [get_nets {dmemIF_inst|storeAddrGen_inst|storeCtr_writemem_inst|Mult2|*}] -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 1
set_multicycle_path -setup -start -from  [get_keepers {VPU_top:myVPU|VPU_controlreg:VPU_controlreg_inst|altsyncram:vc_vstride_rtl_10|altsyncram_shf1:auto_generated|ram_block1a0~portb_address_reg0}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[5]}] 2
set_multicycle_path -hold -start -from  [get_keepers {VPU_top:myVPU|VPU_controlreg:VPU_controlreg_inst|altsyncram:vc_vstride_rtl_10|altsyncram_shf1:auto_generated|ram_block1a0~portb_address_reg*}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 1
set_multicycle_path -setup -start -from  [get_keepers {VPU_top:myVPU|VPU_controlreg:VPU_controlreg_inst|altsyncram:vc_vstride_rtl_10|altsyncram_shf1:auto_generated|ram_block1a0~portb_address_reg*}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 2
set_multicycle_path -setup -start -from  [get_keepers {VPU_top:myVPU|VPU_controlreg:VPU_controlreg_inst|altsyncram:vc_vstride_rtl_10|altsyncram_shf1:auto_generated|ram_block1a0~portb_re_reg}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 2
set_multicycle_path -hold -start -from  [get_keepers {VPU_top:myVPU|VPU_controlreg:VPU_controlreg_inst|altsyncram:vc_vstride_rtl_10|altsyncram_shf1:auto_generated|ram_block1a0~portb_re_reg}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 1
set_multicycle_path -setup -start -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|pipeline_reg_sclr:loadstore_opCode|q_reg[*]~DUPLICATE}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 2
set_multicycle_path -hold -start -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|pipeline_reg_sclr:loadstore_opCode|q_reg[*]~DUPLICATE}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 1
set_multicycle_path -setup -start -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|pipeline_reg_sclr:loadstore_opCode|q_reg[*]}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 2
set_multicycle_path -hold -start -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|pipeline_reg_sclr:loadstore_opCode|q_reg[*]}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 1
set_multicycle_path -setup -start -from  [get_keepers {VPU_top:myVPU|VPU_controlreg:VPU_controlreg_inst|altsyncram:vc_vstride_rtl_10|altsyncram_shf1:auto_generated|ram_block1a0~portb_address_reg*}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|gotoWriteDelayCycle}] 2
set_multicycle_path -hold -start -from  [get_keepers {VPU_top:myVPU|VPU_controlreg:VPU_controlreg_inst|altsyncram:vc_vstride_rtl_10|altsyncram_shf1:auto_generated|ram_block1a0~portb_address_reg*}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|gotoWriteDelayCycle}] 1
set_multicycle_path -setup -start -from  [get_keepers {VPU_top:myVPU|VPU_controlreg:VPU_controlreg_inst|altsyncram:vc_vstride_rtl_10|altsyncram_shf1:auto_generated|ram_block1a0~portb_re_reg}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|gotoWriteDelayCycle}] 2
set_multicycle_path -hold -start -from  [get_keepers {VPU_top:myVPU|VPU_controlreg:VPU_controlreg_inst|altsyncram:vc_vstride_rtl_10|altsyncram_shf1:auto_generated|ram_block1a0~portb_re_reg}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|gotoWriteDelayCycle}] 1
set_multicycle_path -setup -start -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|ff:lsu_memDataWidth_preg|q_reg[*]}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 2
set_multicycle_path -hold -start -from  [get_keepers {memIF:dmemIF_inst|loadstoreController:loadstoreController_inst|ff:lsu_memDataWidth_preg|q_reg[*]}]  -to  [get_keepers {memIF:dmemIF_inst|storeAddrGen:storeAddrGen_inst|storeCtr_writemem:storeCtr_writemem_inst|totalWriteCount[*]}] 1


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

