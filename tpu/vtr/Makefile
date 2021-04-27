BASE_DIR = /home/sangramkate
BASE_FILE_NAME = system

SUB_DIR = scalar

VTR_VERILOG_DIR = $(BASE_DIR)/vtr-verilog-to-routing/vtr_flow/benchmarks/verilog/ml_benchmarks/tpu_subunits/$(SUB_DIR)
VTR_TASK_DIR = $(BASE_DIR)/vtr-verilog-to-routing/vtr_flow/tasks
VTR_SCRIPTS_DIR = $(BASE_DIR)/vtr-verilog-to-routing/vtr_flow/scripts

TPU_VERILOG_DIR = $(BASE_DIR)/tpu_v2/tpu/vtr/verilog/$(SUB_DIR)
TPU_VERILOG_FILE = $(BASE_FILE_NAME).v
SCRIPT = $(BASE_FILE_NAME).py

all: $(SCRIPT)
		python3 $(SCRIPT) $(SUB_DIR) $(TPU_VERILOG_FILE)
		dos2unix $(TPU_VERILOG_DIR)/$(TPU_VERILOG_FILE)
		python3 gen_config.py $(SUB_DIR) $(TPU_VERILOG_FILE)
		\cp $(TPU_VERILOG_DIR)/$(TPU_VERILOG_FILE) $(VTR_VERILOG_DIR)
		$(VTR_SCRIPTS_DIR)/run_vtr_task.py $(VTR_TASK_DIR)/tpu
change:
		python3 $(SCRIPT) $(SUB_DIR) $(TPU_VERILOG_FILE)
		dos2unix $(TPU_VERILOG_DIR)/$(TPU_VERILOG_FILE)
config:
		python3 gen_config.py $(SUB_DIR) $(TPU_VERILOG_FILE)

vtr:
		\cp $(TPU_VERILOG_DIR)/$(TPU_VERILOG_FILE) $(VTR_VERILOG_DIR)
		$(VTR_SCRIPTS_DIR)/run_vtr_task.py $(VTR_TASK_DIR)/tpu
