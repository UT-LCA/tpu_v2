BASE_DIR = /home/sangramkate
FINAL_FILE_NAME = final
BASE_FILE_NAME = vregfile_vector

SUB_DIR = vector

TPU_SUB_DIR = verilog

VTR_VERILOG_DIR = $(BASE_DIR)/vtr/vtr_flow/benchmarks/verilog/ml_benchmarks/tpu_subunits/$(SUB_DIR)
VTR_TASK_DIR = $(BASE_DIR)/vtr/vtr_flow/tasks
VTR_SCRIPTS_DIR = $(BASE_DIR)/vtr/vtr_flow/scripts

TPU_VERILOG_DIR = $(BASE_DIR)/tpu_v2/tpu/vtr/vector_scripts/${TPU_SUB_DIR}
TPU_VERILOG_FILE = $(BASE_FILE_NAME).v
SCRIPT = $(BASE_FILE_NAME).py

all: $(SCRIPT)
	        rm -rf verilog
		mkdir verilog
		ls verilog/
		python3 $(SCRIPT) $(TPU_SUB_DIR)/$(TPU_VERILOG_FILE)
		python3 gen_config.py $(SUB_DIR) final.v
	        python3 make_one.py 
		\cp $(TPU_VERILOG_DIR)/final.v $(VTR_VERILOG_DIR)
		$(VTR_SCRIPTS_DIR)/run_vtr_task.py $(VTR_TASK_DIR)/tpu
config:
		python3 gen_config.py $(SUB_DIR) $(TPU_VERILOG_FILE)

vtr:
	        python3 make_one.py 
		\cp $(TPU_VERILOG_DIR)/$(TPU_VERILOG_FILE) $(VTR_VERILOG_DIR)
		$(VTR_SCRIPTS_DIR)/run_vtr_task.py $(VTR_TASK_DIR)/tpu

rerun:
		\cp $(TPU_VERILOG_DIR)/final.v $(VTR_VERILOG_DIR)
		$(VTR_SCRIPTS_DIR)/run_vtr_task.py $(VTR_TASK_DIR)/tpu
