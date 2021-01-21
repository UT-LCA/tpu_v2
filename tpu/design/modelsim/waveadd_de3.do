#vsim test_bench   (do this at command line instead, needs -pli flags and such)
radix hexadecimal

virtual type {SPECIAL REGIMM J JAL BEQ BNE BLEZ BGTZ
              ADDI ADDIU SLTI SLTIU ANDI ORI XORI LUI
              ERR ERR COP2 ERR ERR ERR ERR ERR
              ERR ERR ERR ERR ERR ERR ERR ERR
              LB LH ERR LW LBU LHU ERR ERR
              SB SH ERR SW ERR ERR ERR ERR
              ERR ERR ERR ERR ERR ERR ERR ERR
              ERR ERR ERR ERR ERR ERR ERR ERR
              } mystateType

virtual type {SPECIAL REGIMM J JAL BEQ BNE BLEZ BGTZ
              ADDI ADDIU SLTI SLTIU ANDI ORI XORI LUI
              ERR ERR COP2 ERR ERR ERR ERR ERR
              ERR ERR ERR ERR ERR ERR ERR ERR
              LB LH ERR LW LBU LHU ERR ERR
              SB SH ERR SW ERR ERR ERR ERR
              ERR ERR ERR ERR ERR ERR ERR ERR
              ERR ERR ERR ERR ERR ERR ERR ERR
              NOP_SLL ERR SRL SRA SLLV ERR SRLV SRAV
              JR JALR ERR ERR SYSCALL BREAK ERR ERR
              MFHI MTHI MFLO MTLO ERR ERR ERR ERR
              MULT MULTU DIV DIVU ERR ERR ERR ERR
              ADD ADDU SUB SUBU AND OR XOR NOR
              ERR ERR SLT SLTU ERR ERR ERR ERR
              ERR ERR ERR ERR ERR ERR ERR ERR
              ERR ERR ERR ERR ERR ERR ERR ERR
              } mystateType2


add wave sim:/test_bench/t/clk;
add wave sim:/test_bench/t/mem_clk;
add wave sim:/test_bench/t/procresetn;
add wave sim:/test_bench/t/tm4_devbus;

add wave -divide "Pipe stages"
add wave sim:/test_bench/pc;
virtual function {(mystateType2)/test_bench/D_pipe1_instr} instr1
virtual function {(mystateType2)/test_bench/D_pipe2_instr} instr2
add wave /test_bench/instr1
add wave /test_bench/instr2
add wave /test_bench/D_*_stall
add wave /test_bench/D_*_squash
add wave sim:/test_bench/t/p/c/p/reg_file_a_readdataout
add wave sim:/test_bench/t/p/c/p/reg_file_b_readdataout
add wave -label c_writedatain sim:/test_bench/t/p/c/p/nop13_q
add wave -label c_reg sim:/test_bench/t/p/c/p/pipereg5_q
add wave -label c_we sim:/test_bench/t/p/c/p/ctrl_reg_file_c_we
add wave sim:/test_bench/t/p/c/p/branch_mispred
add wave -label delay_slot1 sim:/test_bench/t/p/c/p/pipereg20_q
add wave sim:/test_bench/t/p/c/p/ifetch_instr

add wave -divide "CPU bus"
add wave sim:/test_bench/t/p/dcpu_*

#add wave -divide "Ifetch"
#add wave sim:/test_bench/t/p/c/p/ifetch/*

#add wave -divide "DE3 Test Bench"
#add wave sim:/test_bench/*

#add wave -divide "DDR2 Memory Model"
#add wave sim:/test_bench/ddr2/altmemddr_mem_model_ram/*

#add wave -divide "DE3"
#add wave sim:/test_bench/t/*;

#add wave -divide "Mem Hierarchy"
#add wave sim:/test_bench/t/mem_hierarchy/*

#add wave -divide "Mem DPort"
#add wave sim:/test_bench/t/mem_hierarchy/dport/*

#add wave -divide "Dcache"
#add wave sim:/test_bench/t/p/dcache1/*

#add wave -divide "WB Buffer"
#add wave sim:/test_bench/t/p/dcache1/wbbuffer/*

#add wave -divide "DDR Controller"
#add wave sim:/test_bench/t/mem_hierarchy/altmemddr/*;

#add wave -divide "altmemddr_controller_phy"
#add wave sim:/test_bench/t/mem_hierarchy/altmemddr/altmemddr_controller_phy_inst/*;

#add wave -divide "altmemddr_phy"
#add wave sim:/test_bench/t/mem_hierarchy/altmemddr/altmemddr_controller_phy_inst/alt_mem_phy_inst/*;

#add wave -divide "altmemddr_phy_alt_mem_phy_siii_inst"
#add wave sim:/test_bench/t/mem_hierarchy/altmemddr/altmemddr_controller_phy_inst/alt_mem_phy_inst/altmemddr_phy_alt_mem_phy_siii_inst/*;

#add wave -divide "seq - altmemddr_phy_alt_mem_phy_sequencer_wrapper"
#add wave sim:/test_bench/t/mem_hierarchy/altmemddr/altmemddr_controller_phy_inst/alt_mem_phy_inst/altmemddr_phy_alt_mem_phy_siii_inst/seq/*;

#add wave -divide "JDDR Interface"
#add wave sim:/test_bench/t/mem_hierarchy/jddr_inst/*;

#add wave -divide "JDDR DQS Group"
#add wave sim:/test_bench/t/mem_hierarchy/jddr_inst/g_datapath/g_datapath__7/g_ddr_io/*;

#add wave -divide "PortMux"
#add wave sim:/test_bench/t/pm/*

#add wave -divide "FS DevReadBurst"
#add wave sim:/test_bench/t/pm/TM_port5/*

wave zoomrange 0 120ns

######################### Vector Processor Stuff ###########################

#add memory /test_bench/t/p/c/v/vregfile_control/reg_file1/mem_data
#add memory /test_bench/t/p/c/v/vregfile_base/reg_file1/mem_data
#add memory /test_bench/t/p/c/v/vregfile_inc/reg_file1/mem_data
#add memory /test_bench/t/p/c/v/vregfile_scalar/reg_file1/mem_data
#add memory /test_bench/t/p/c/v/vlanes/vregfile_vector/bank_gen\[0\]/reg_file1/mem_data
#add memory /test_bench/t/p/c/v/vlanes/vregfile_flag/bank_gen\[0\]/reg_file1/mem_data
#add memory /test_bench/t/p/dcache1/data/mem_data

do virtualtype.do

#add wave -divide "Processor"
#add wave sim:/test_bench/t/p/*

#add wave -divide "Ifetch"
#add wave sim:/test_bench/t/p/c/p/ifetch/*

#add wave -divide "COP2 signals"
#add wave sim:/test_bench/t/p/c/p/cop2/*

add wave -divide "VPU signals"
add wave sim:/test_bench/pc;
add wave sim:/test_bench/read_vector_next
add wave sim:/test_bench/dst_vector_next
add wave sim:/test_bench/readdata_vector_next
#add wave sim:/test_bench/t/count_*
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr[1]} vinstr1
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr[2]} vinstr2
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s3[0]} vinstr30
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s3[1]} vinstr31
#virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s3[2]} vinstr32
#virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s3[3]} vinstr33
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s4[0]} vinstr4ALU1
#virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s4[1]} vinstr4ALU2
#virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s4[2]} vinstr4ALU3
#virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s4[3]} vinstr4ALU4
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s4[1]} vinstr4MUL
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s4[2]} vinstr4MEM
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s4[3]} vinstr4FALU
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s5[0]} vinstr5ALU1
#virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s5[1]} vinstr5ALU2
#virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s5[2]} vinstr5ALU3
#virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s5[3]} vinstr5ALU4
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s5[1]} vinstr5MUL
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s5[2]} vinstr5MEM
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s5[3]} vinstr5FALU
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s6[0]} vinstr6ALU
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s6[1]} vinstr6MUL
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s6[2]} vinstr6MEM
virtual function {(myVectorType)/test_bench/t/p/c/v/vlanes/D_instr_s6[3]} vinstr6FALU
add wave /test_bench/t/p/c/v/vlanes/vinstr1
add wave /test_bench/t/p/c/v/vlanes/vinstr2
add wave /test_bench/t/p/c/v/vlanes/vinstr30
add wave /test_bench/t/p/c/v/vlanes/vinstr31
#add wave /test_bench/t/p/c/v/vlanes/vinstr32
#add wave /test_bench/t/p/c/v/vlanes/vinstr33
add wave /test_bench/t/p/c/v/vlanes/vinstr4ALU1
#add wave /test_bench/t/p/c/v/vlanes/vinstr4ALU2
#add wave /test_bench/t/p/c/v/vlanes/vinstr4ALU3
#add wave /test_bench/t/p/c/v/vlanes/vinstr4ALU4
add wave /test_bench/t/p/c/v/vlanes/vinstr4MUL
add wave /test_bench/t/p/c/v/vlanes/vinstr4MEM
add wave /test_bench/t/p/c/v/vlanes/vinstr4FALU
add wave /test_bench/t/p/c/v/vlanes/vinstr5ALU1
#add wave /test_bench/t/p/c/v/vlanes/vinstr5ALU2
#add wave /test_bench/t/p/c/v/vlanes/vinstr5ALU3
#add wave /test_bench/t/p/c/v/vlanes/vinstr5ALU4
add wave /test_bench/t/p/c/v/vlanes/vinstr5MUL
add wave /test_bench/t/p/c/v/vlanes/vinstr5MEM
add wave /test_bench/t/p/c/v/vlanes/vinstr5FALU
add wave /test_bench/t/p/c/v/vlanes/vinstr6ALU
add wave /test_bench/t/p/c/v/vlanes/vinstr6MUL
add wave /test_bench/t/p/c/v/vlanes/vinstr6MEM
add wave /test_bench/t/p/c/v/vlanes/vinstr6FALU
add wave sim:/test_bench/t/p/c/v/*
delete wave /test_bench/t/p/c/v/ctrl*

add wave -divide "VLanes "
add wave sim:/test_bench/t/p/c/v/vlanes/*
delete wave /test_bench/t/p/c/v/vlanes/ctrl*

#add wave -divide "HazCheck_src1 "
#add wave sim:/test_bench/t/p/c/v/vlanes/src1hazchecker/*

add wave -divide "Vmem_Fill "
add wave sim:/test_bench/t/dmem_fillwe
add wave sim:/test_bench/t/dmem_filladdr
add wave sim:/test_bench/t/dmem_filldata

add wave -divide "Vmem_unit "
add wave sim:/test_bench/t/p/c/v/vlanes/vmem_unit/*

#add wave -divide "VALU "
#add wave sim:/test_bench/t/p/c/v/vlanes/lanes4_gen\[15\]/valu/*

#add wave -divide "Vmul_unit"
#add wave sim:/test_bench/t/p/c/v/vlanes/vmul_unit/*

#add wave -divide "Vmul_shift"
#add wave sim:/test_bench/t/p/c/v/vlanes/vmul_unit/lanes_gen\[0\]/vmul/*

#add wave -divide "Vbar_shift"
#add wave sim:/test_bench/t/p/c/v/vlanes/barrelshift_gen\[3\]/vshift/*

#mem display test_bench/t/dmem/mem_data

#add wave -divide "Divider"
#add wave sim:/test_bench/t/p/c/p/div/*

run 800000

