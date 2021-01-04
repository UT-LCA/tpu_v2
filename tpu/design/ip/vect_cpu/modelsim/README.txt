-----------------------------------------------------------
Soft-core Vector Processor Project
Implementation version 0.7

Author: Jason Yu
Last updated: April 23, 2007
-----------------------------------------------------------

The following scripts are located in this directory:

VPU_sim.do			ModelSim simulation script
VPU_init.do			ModelSim simulation initialization script
VPU_wave.do			ModelSim simulation waveform script

dmem_init.hex			Automatically generated data memory initialization file for testbench
imem.hex			Instruction memory initialization file for testbench
*.hex				Other memory initialization files for ModelSim simulation

vpu_state_dump.do		ModelSim script to dump all vector register contents to a "dump" directory
parse_state_dump.tcl		Script to parse the dump files and generate an easier-to-read text file

vpu_asm.tcl			Script to assemble a .asm file using the vector cpu assembler and generate a ModelSim .hex file for simulation


-----------------------------
- General processor testing -
-----------------------------

standard_tests.asm contains some predefined instructions used for testing.
"test output" directory contains verified outputs of the processor executing the test instructions.

To run a simulation:
1) Fill imem.asm with some cpu instructions (can copy from standard_tests.asm)
2) Type "tclsh85 vpu_asm.tcl imem.asm imem" to generate imem.hex
3) In ModelSim, type "do VPU_sim.do"
4) Simulation generates store_trace.txt, which is a trace of store instructions to memory
5) In ModelSim, run "do vpu_state_dump.do" to generate a report of vector register contents
6) Do "diff vrf_state_dump.txt test\ output/vrf_state_dump.xxxx.txt" to compare simulation results to verified output. "xxxx" is the name of the test suite.