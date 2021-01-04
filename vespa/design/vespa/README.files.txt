Below is the hierarchy of the system design.

modelsim/de3_test_bench.v
  de3.v
    options.v
    tm4_portmux.vhd  (This has been replaced with tmj_portmux)
      devread.vhd
      devreadburst.vhd*
      devwrite.vhd
      devwriteack.vhd
    tmj_portmux.v
      simple_dual_port_ram_single_clock.v
      virtual1.v
    processor.v
      mem_icache.v
      mem_dcache_wb.v
        mem_wbbuffer.v
      core.v
        spree/system.v  (Scalar MIPS processor)
          spree/addersub_slt.v
          spree/branchresolve.v
          spree/components.v
          spree/cop0.v
          spree/cop2.v
          spree/data_mem_bus_int.v
          spree/divider.v
          spree/hi_reg.v
          spree/ifetch_pipe_bpred_bus_int.v
            spree/bpred_1bittable.v
          spree/isa.v
          spree/lo_reg.v
          spree/logic_unit.v
          spree/merge26lo.v
          spree/mul_shift_stall.v
          spree/pcadder.v
          spree/reg_file_pipe.v
          spree/signext16.v
        vpu.v  (Vector VIRAM coprocessor)
          visa.v
          vcomponents.v
          vregfile_base.v
          vregfile_control.v
          vregfile_inc.v
          vregfile_scalar.v
          vregfile_stride.v
          vlanes.v
            vlane_alu.v
              vlane_saturate.v
            vlane_flagalu.v
            vmul_unit.v
              vlane_mulshift.v
              vlane_barrelshifter.v
            vmem_unit.v
              velmshifter_serial.v
              vmem_crossbar.v
            vregfile_vector.v
            vregfile_flag.v
    mem_hierarchy.v
      mem_port.v
      altmemddr.v  (DDR2 controller)
    IOV_A3V3_B1V8_C3V3_D3V3.v (set DE3 voltages - requires terasic license found on CD.  The default values work fine so I doubt this is needed at all)

