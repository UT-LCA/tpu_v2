# create dump directory
mkdir dump

mem save -format hex -outfile ./dump/scalarreg_dump.hex {/tb_VectCPU/myVectCPU/ScalarCore/the_Datapath/Stage_Two/the_Register_File/the_Register_Memory/altsyncram_component/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane0.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane1.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[1]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane2.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[2]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane3.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[3]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane4.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[4]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane5.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[5]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane6.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[6]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane7.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[7]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane8.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[8]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane9.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[9]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane10.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[10]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane11.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[11]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane12.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[12]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane13.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[13]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane14.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[14]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}
mem save -format hex -outfile ./dump/vflag_dump_lane15.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[15]/genblk1/VLane_inst/VLane_dp_inst/vflagreg_mf_inst/alt3pram_component/u0/mem_data}

mem save -format hex -outfile ./dump/vc_vbase_dump.hex {/tb_VectCPU/myVectCPU/myVPU/VPU_controlreg_inst/vc_vbase}
mem save -format hex -outfile ./dump/dmem_dump.hex {/tb_VectCPU/theDataMemory/data_mem}

mem save -format hex -outfile ./dump/vrf_dump_lane0.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[0]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane1.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[1]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane2.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[2]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane3.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[3]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane4.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[4]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane5.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[5]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane6.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[6]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane7.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[7]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane8.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[8]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane9.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[9]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane10.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[10]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane11.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[11]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane12.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[12]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane13.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[13]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane14.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[14]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}
mem save -format hex -outfile ./dump/vrf_dump_lane15.hex {/tb_VectCPU/myVectCPU/myVPU/VLane_group_inst/lanei[15]/genblk1/VLane_inst/VLane_dp_inst/vregfile_mem}


# run dump parser script
source parse_state_dump.tcl

# delete intermediate files
file delete -force dump