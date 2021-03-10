
VLOG_ARGS=+define+TEST_BENCH=1

# List all source files, note many files use `include to include dependencies
SPREE_SRC= ../design/scalar/system.v
VESPA_SRC= ../design/vector/vpu.v

BFLOAT_SRC = ../design/bfloat/bfloat_adder.v \
             ../design/bfloat/bfloat_mult.v \
             ../design/bfloat/activation.v \
             ../design/bfloat/reduction_unit.v \
	     ../design/bfloat/transpose.v \
             ../design/bfloat/trp_unit.v 

TESTBENCH_SRC= de3_test_bench.v
DDR_SRC=  ../design/ddr/alt_mem_phy_defines.v \
          ../design/ddr/altmemddr.v \
          ../design/ddr/altmemddr_controller_phy.v \
          ../design/ddr/altmemddr_phy.v \
          ../design/ddr/altmemddr_phy_alt_mem_phy_pll.v \
          ../design/ddr/altmemddr_phy_alt_mem_phy.v \
          ../design/ddr/altmemddr_phy_alt_mem_phy_sequencer_wrapper.vo \
          ../design/ddr/altmemddr_auk_ddr_hp_controller_wrapper.vo 
LOCAL_SRC= ../design/local/local_add_sub.v \
           ../design/local/local_mult.v \
           ../design/local/local_fifo.v \
           ../design/local/dpram1.v \
           ../design/local/local_shifter.v
compile: plifs.sl virtualtype.do filesystem_standalone compilelib compilework

############################# Altera Libraries ###############################

compilelib: altera stratixiii 

altera:
	vlib altera
	vlog -quiet ${QUARTUS_ROOTDIR}/eda/sim_lib/altera_mf.v -work altera
	vlog -quiet ${QUARTUS_ROOTDIR}/eda/sim_lib/220model.v -work altera
	vlog -quiet ${QUARTUS_ROOTDIR}/eda/sim_lib/altera_primitives.v -work altera
	vlog -quiet ${QUARTUS_ROOTDIR}/eda/sim_lib/sgate.v -work altera
stratixiii:
	vlib stratixiii
	vlog -quiet stratixiii_atoms.v -work stratixiii

############################# Compile VESPA ###############################

work:
	vlib work

compilework: work vregfile_control.dat
	vcom ../design/top/tm4_portmux.vhd
	vcom ../design/top/devread.vhd
	vcom ../design/top/devreadburst.vhd
	vcom ../design/top/devwrite.vhd
	vcom ../design/top/devwriteack.vhd
	vlog ${VLOG_ARGS} ../design/top/options.v
	vlog ${VLOG_ARGS} ../design/local/spram1.v
	vlog ${VLOG_ARGS} ../design/local/rams.v
	vlog ${VLOG_ARGS} ../design/local/ram_wrapper.v
	vlog ${VLOG_ARGS} ${LOCAL_SRC}
	vlog ${VLOG_ARGS} +incdir+.. +incdir+../design/top +incdir+../design/vector +incdir+../design/scalar +incdir+../design/ddr ${BFLOAT_SRC}
	vlog ${VLOG_ARGS} +incdir+.. +incdir+../design/top +incdir+../design/vector +incdir+../design/scalar +incdir+../design/ddr ${SPREE_SRC}
	vlog ${VLOG_ARGS} +incdir+.. +incdir+../design/top +incdir+../design/vector +incdir+../design/scalar +incdir+../design/ddr ${VESPA_SRC}
	vlog ${VLOG_ARGS} +incdir+.. +incdir+../design/top +incdir+../design/vector +incdir+../design/scalar +incdir+../design/ddr ${DDR_SRC}
	vlog ${VLOG_ARGS} ../design/ddr/testbench/altmemddr_mem_model.v
	vlog ${VLOG_ARGS} +incdir+.. +incdir+../design/top +incdir+../design/vector +incdir+../design/scalar +incdir+../design/ddr ${TESTBENCH_SRC}

#Convert .mif to .dat format - HACKY
vregfile_control.dat:
	@grep ":" ../design/vector/vregfile_control.mif |cut -f2 -d':' |cut -f1 -d';' >vregfile_control.dat

#Create virtualtype.do script for translating instruction encodings to names
virtualtype.do: virtualtype
	./virtualtype
virtualtype: virtualtype.cc
	g++ -g -O2 virtualtype.cc -o virtualtype

#Build the PLI-based filesystem which processes I/O calls (printf/fopen/etc.)
plifs.sl: plifs.c
	gcc -c -I${MODEL_TECH}/include plifs.c tm4filesystem.c tm4llist.c -fPIC
	ld -G -o plifs.sl plifs.o tm4filesystem.o tm4llist.o
	rm -f plifs.o tm4filesystem.o tm4llist.o

#Handy tool for performing I/O operations from modelsim_filesystem.txt
filesystem_standalone: filesystem_standalone.cc tm4filesystem.c tm4llist.c
	g++ filesystem_standalone.cc tm4filesystem.c tm4llist.c -o filesystem_standalone

clean_plifs: 
	@rm -f plifs.sl plifs.o tm4filesystem.o tm4llist.o

clean: clean_plifs
	rm -rf altera stratixiii work transcript vsim.wlf vsim_stacktrace.vstf
	rm -f instr.dat data.dat cmdlineargs.txt vregfile_control.dat
	rm -f virtualtype filesystem_standalone


.PHONY: compile compilelib compilework clean_plifs 

