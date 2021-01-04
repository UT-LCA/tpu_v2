vsim -L altera_mf -L 220model {-voptargs=+acc+Pipeline_S2_Regs +acc+Control_Registers +acc+Register_File +acc+loadAddrGenerator +acc+dMemModel +acc+VLane_datapath +acc+VPU_controlreg +acc+VPU_decode +acc+loadAddrGenerator +acc+VPU_controller +acc+loadstoreController +acc+storeAddrGen +acc+storeCtr_fillreg +acc+storeCtr_writemem +acc+VPU_hazard_detection +acc+memwriteIF +acc+Instruction_Fetch_Unit +acc+Control_Unit +acc+VPU_pipeline_regs +acc+memreadIF +acc+mem_order_q} work.tb_VectCPU 
do VPU_wave.do
do VPU_init_8.do
run 1000ns