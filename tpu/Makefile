TOP=de3
APP=empty

# Set USB Blaster port (use "quartus_pgm -l" to see where your DE3 is)
USBPORT=2

usage:  
	@echo 
	@echo "VESPA Make Commands"
	@echo 
	@echo "  FPGA CAD Flow using Altera Quartus II"
	@echo "    compile    Generate TM ports and synthesize design using Altera Quartus"
	@echo "    download   Configure FPGA on DE3 board specified by variable USBPORT"
	@echo 
	@echo "  Modelsim RTL Simulation"
	@echo "    de3sim_compile   Compile design for Modelsim"
	@echo "    de3sim           Run simulation to completion in command-line mode"
	@echo "    de3sim_gui       Run simulation in GUI mode"
	@echo 
	@echo "  Other commands"
	@echo "    test             Tests presence of vectorized gcc, Quartus, and Modelsim"
	@echo "    clean            Deletes all generated files from synthesis and simulation"
	@echo 

test:  
	@echo 
	@echo "***********  Testing compiler (should say gcc 4.2.0)  ************"
	@echo 
	${SPE_COMPILER}/bin/mips-mips-elf-gcc -v
	@echo 
	@echo "***********  Testing Quartus ************"
	@echo 
	quartus_pgm -v
	@echo "  ----- Trying again using QUARTUS_ROOTDIR environment variable -----"
	${QUARTUS_ROOTDIR}/bin/quartus_pgm -v
	@echo 
	@echo "***********  Testing Modelsim ************"
	@echo 
	vsim -version
	@echo "  ----- Trying again using MODEL_TECH environment variable -----"
	${MODEL_TECH}/vsim -version
	@echo 
	@echo "***********  Success!  You are good to go! ************"

#########################################################################
############################## DE3 STUFF ################################
#########################################################################

compile:	
	@echo
	@echo
	@echo "  ---- Started at: "`date`" ----"
	@echo "    (Skipping tmj - uncomment if HW/SW interface is modified)"
	#tmj de3.ports
	qcmd ${TOP} >& quartus.out
	@grep -i 'Error' quartus.out
	@echo "  ---- Finished at: "`date`" ----"

${TOP}.sof:
	gunzip ${TOP}.sof.gz

download: ${TOP}.sof
	quartus_pgm -z -m JTAG -c ${USBPORT} -o "p;${TOP}.sof"
	gzip -f ${TOP}.sof

spawn:
	mkdir spawn
	-cp * spawn
	cp -r -L altmemddr spree spawn

gziprpt: 
	@-gzip -f *.rpt *.sof

de3cad_clean: gziprpt
	@rm -rf db incremental_db
	@rm -f quartus.out *.pof *.pin *.summary *.smsg *.jdi

de3run_clean:
	@rm -f instr.dat data.dat instr.rif data.rif cmdlineargs.txt trace.txt 

vdecoder: vdecoder.cc
	g++ -g -O2 vdecoder.cc -o vdecoder

quartus:
	quartus  ${TOP}.qpf &

#########################################################################
############################ Modelsim STUFF #############################
#########################################################################

de3sim_compile:
	cd modelsim && make 
	@echo 
	@echo Successfully compiled!!!!  
	@echo 
	@echo Note DDR2 controller requires a few thousand initialization cycles
de3sim:
	cd modelsim && vsim -novopt -quiet -keepstdout -pli plifs.sl -warning 1130 +nowarnTFMPC +nowarnPCDPC -c -do "run -all" -L altera -L stratixiii test_bench
de3sim_runapp: de3sim_getapp de3sim
	tail -12 modelsim/transcript >modelsim_transcripts/$(APP).transcript
de3sim_runapp_gui: de3sim_getapp de3sim_gui
de3sim_gui:
	cd modelsim && vsim -novopt -gui -pli plifs.sl -warning 1130 -do waveadd_de3.do -L altera -L stratixiii test_bench 
de3sim_getapp:
	cd modelsim && getapp ${APP}
de3sim_clean:
	cd modelsim && make clean

#########################################################################
############################# Clean STUFF ###############################
#########################################################################

clean: de3run_clean de3sim_clean

.PHONY: usage compile download test quartus quartus_cmd tm4b de3run_clean clean tm4_clean gziprpt de3sim de3sim_gui de3sim_getapp de3sim_app de3cad_clean
