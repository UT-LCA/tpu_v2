// ******************** Board-level Test Bench *******************
//
// This test bench simulates the VESPA processor, DDR module, and
// even the host-to-circuit communication.  The latter was done 
// for the original TM4 ports package and was never adapted for the 
// DE3 ... but hey, it's simulation so we can pretend it's a DE3
// system running on the TM4 ports package!  This doesn't change the
// execution of the benchmarks since there is no host-to-circuit
// communication during the timed portion of the benchmark.

// The EEMBC benchmarks execute signal_start() and signal_finished() 
// before/after the relevant parts of the execution.  Both of these 
// calls read from the clock timer.  Thus we can limit our collection
// of profiling statistics to being between these two clock reads:
// Profile between clock reads (1), otherwise do complete execution (0)
`define PROFILEEEMBC 1

// The test bench dumps traces of writes to all register files and
// memory.  This is for debugging purposes but can be disabled.
`define DISABLETRACES 0

`timescale 1ns / 1ns

`include "options.v"

`include "de3.v"


module test_bench ;

parameter LOG2DCACHEWIDTHBITS=`LOG2DCACHEWIDTHBITS;
parameter DCACHEWIDTHBITS=2**LOG2DCACHEWIDTHBITS;

parameter I_DATAWIDTH=32;
parameter I_ADDRESSWIDTH=18;            //Byte-Addressable
parameter I_SIZE=65536;                 //in I_DATAWIDTH size words

parameter D_DATAWIDTH=32;
parameter D_BYTEENAWIDTH=D_DATAWIDTH/8; 
parameter D_ADDRESSWIDTH=25;            //For Byte-addressable address
parameter D_SIZE=33554432/4;            //in D_DATAWIDTH size words

parameter D_DATAWIDTH_DRAM=256;
parameter D_ADDRESSWIDTH_DRAM=D_ADDRESSWIDTH-3;
parameter D_SIZE_DRAM=D_SIZE/8;

parameter COUNTER_SIZE=38; //must be >=32

    wire              clk;
    wire             resetn;


    reg    tm4_glbclk0;
    wire [40:0]  tm4_devbus;
    reg [40:0]  _tm4_devbus;

    // DDR Interface Signals for DDR slot A
    wire  [0:0] mem_odt;
    wire  [0:0] mem_cs_n;
    wire  [0:0] mem_cke;
    wire  [12:0]  mem_addr;
    wire  [1:0] mem_ba;
    wire    mem_ras_n;
    wire    mem_cas_n;
    wire    mem_we_n;
    wire  [7:0] mem_dm;
    wire [1:0] mem_clk;
    wire [1:0] mem_clk_n;
    tri  [63:0]  mem_dq = 100'bz;
    tri  [7:0] mem_dqs = 100'bz;
    tri  [7:0] mem_dqsn = 100'bz;

    integer trace;
    integer store_trace;
    integer exception_trace;
    integer fs;
    integer vector_trace;


    de3 t(

        .OSC1_50(tm4_glbclk0),

        `ifdef TEST_BENCH
          .tm4_devbus(tm4_devbus),
        `endif

        .mem_addr          (mem_addr),
        .mem_ba            (mem_ba),
        .mem_cas_n         (mem_cas_n),
        .mem_cke           (mem_cke),
        .mem_clk           (mem_clk),
        .mem_clk_n         (mem_clk_n),
        .mem_cs_n          (mem_cs_n),
        .mem_dm            (mem_dm),
        .mem_dq            (mem_dq),
        .mem_dqs           (mem_dqs),
        .mem_dqsn          (mem_dqsn),
        .mem_odt           (mem_odt),
        .mem_ras_n         (mem_ras_n),
        .mem_we_n          (mem_we_n)

        );


      // Requires data_mem.d_write signal to be present
    wire [32-1:0] pc;
    wire [32-1:0] instr;
    wire reg_file_we;
    wire [5-1:0] reg_file_dst;
    wire [32-1:0] reg_file_data;
    wire data_mem_we;
    wire data_mem_write;
    wire [32-1:0] data_mem_addr;
    wire [32-1:0] data_mem_data;

    assign pc=t.p.c.p.ifetch_next_pc ;
    assign instr= t.p.c.p.ifetch_instr ;
    assign reg_file_we= t.p.c.p.ctrl_reg_file_c_we ;
    assign reg_file_dst= t.p.c.p.pipereg5_q ;
    assign reg_file_data= t.p.c.p.nop13_q ;
    assign data_mem_we= t.p.c.p.ctrl_data_mem_en ;
    assign data_mem_write= t.p.c.p.data_mem.d_write ;
    assign data_mem_addr= t.p.c.p.addersub_result ;
    assign data_mem_data= t.p.c.p.nop10_q ;


    // Count values
    reg profile;
    reg [31:0] count_total;
    reg [31:0] count_lanesstalled;
    reg [31:0] count_vldstalled;
    reg [31:0] count_vststalled;
    reg [31:0] count_vldmiss;
    reg [31:0] count_vstmiss;
    reg [31:0] count_memsubvector;

    /**************** Reset and stimulate clock ********************/

    //50 MHz clock
    initial
      tm4_glbclk0 = 1'b1;
    always
      #10000 tm4_glbclk0 <= ~tm4_glbclk0;

    assign clk=t.clk;


    //Emulate bus transfers on TM4 between FPGA and host
    //tm4_devbus[40] - resetn
    //tm4_devbus[39] - ACKn - don't touch this 
    //tm4_devbus[38] - framen - lower when linux host is communicating with TM4
    //tm4_devbus[37:32] - addr
    //
    //NOTE: this is all hard and specific to the TM4.  I'm too lazy to redo
    //this for the new DE3 ports package so I keep this stuff around.  If you
    //don't care about the supporting file I/O (which is done through PLI with
    //the plifs library) this whole thing can be tossed and replaced by simple
    //delayed reset
    initial 
      begin
        _tm4_devbus={1'b0,2'bz1,6'bz,32'bz};
        #100000 _tm4_devbus={1'b1,2'bz0,6'b01,32'b0}; //Wait for PLL to kick in
        #30000 _tm4_devbus={1'b1,2'bz1,6'b01,32'b0};
        #30000 _tm4_devbus={1'b1,2'bz0,6'b01,32'b1};
        #30000 _tm4_devbus={1'b1,2'bz1,6'b01,32'b1};
        #30000 _tm4_devbus={1'b1,2'bz1,6'bzz,32'bz}; 
      end

    reg [31:0] dummy;
    reg [3:0] devbus_state;
    always@(posedge clk)
    begin
      case(devbus_state)
        0:
        begin
          devbus_state<=(t.fs_writedata[8] && t.fs_writedata_ready) ? 1 :
                        (t.fs_readdata_want) ? 5 : 0;
          if (t.fs_writedata[8] && t.fs_writedata_ready)
            _tm4_devbus<={1'b1,2'bz0,6'b101,32'b1};
        end
        1:
        begin
          devbus_state<=2;
        end
        2:
        begin
          _tm4_devbus={1'b1,2'bz1,6'b101,32'bz};
          devbus_state<=3;
        end
        3:
        begin
          if (tm4_devbus[39]==0)
            $plifs(tm4_devbus[31:0],dummy);
          devbus_state<= (tm4_devbus[39]==0) ? 0 : 3;
        end
        5:  //READ1
        begin
          $plifs(0,_tm4_devbus[31:0]);
          _tm4_devbus={1'b1,2'bz0,6'b110,_tm4_devbus[31:0]};
          //$display("DUT received %x at time %d",_tm4_devbus[7:0],$time);
          devbus_state<= 6;
        end
        6:  //READ2
        begin
          if (tm4_devbus[39]==0)
            _tm4_devbus={1'b1,2'bz1,6'bzzzzzz,32'bz};
          else
            _tm4_devbus={1'b1,2'bz1,6'b110,_tm4_devbus[31:0]};
          devbus_state<= (tm4_devbus[39]==0) ? 0 : 6;
        end
        default:
          devbus_state<= 0;
      endcase
    end

    assign tm4_devbus=_tm4_devbus;
    assign resetn=tm4_devbus[40];

  always@(posedge clk)
    if (t.dead)
    begin

        $tm4fsexit(); //Close plifs filesystem

        /**************** Clean up & close files ********************/
        $fclose(trace);
        $fclose(store_trace);
        $fclose(fs);
        $fclose(vector_trace);

        /**************** Dump Stats ********************/
        $display("count_total_cycles=%d",count_total);
        $display("count_lanesstalled_cycles=%d",count_lanesstalled);
        $display("count_vldstall_cycles=%d",count_vldstalled);
        $display("count_vststall_cycles=%d",count_vststalled);
        $display("count_vldcachemiss_cycles=%d",count_vldmiss);
        $display("count_vstcachemiss_cycles=%d",count_vstmiss);
        $display("count_memsubvector=%d",count_memsubvector);

        /**************** Kill Modelsim ********************/
        $display("!!!=== Stopped at %1d ps",$time);
        $finish;
    end

    /**************** Trace of Write Backs ********************/
    initial
    begin
        trace=$fopen("/tmp/modelsim_trace.txt","w");
        store_trace=$fopen("/tmp/modelsim_store_trace.txt","w");
        fs=$fopen("/tmp/modelsim_filesystem.txt","w");
        vector_trace=$fopen("/tmp/modelsim_vector_trace.txt","w");
    end

    always@(posedge clk)
    begin
        if (reg_file_we && (|reg_file_dst) && resetn && !(`DISABLETRACES))
            $fwrite(trace,"%d | PC=%h | IR=%h | %h: %h\n",
                    $time,
                    pc,
                    instr,
                    reg_file_dst,
                    reg_file_data);
    end

    /**************** Trace of Stores to Memory ********************/

    always@(posedge clk)
    begin
      if (data_mem_we && data_mem_write && resetn && !(`DISABLETRACES) &&
              ((t.p.dcpu_en&~t.p.dcpu_wait) || t.p.dcache_wren))
        $fwrite(store_trace,"%d | PC=%h | IR=%h | %h: %h\n",
          $time,
          pc,
          instr,
          data_mem_addr,
          data_mem_data);
    end

    /**************** User inserted code ********************/
wire [6:0] D_pipe1_instr;
wire D_pipe1_stall;
wire D_pipe1_squash;
wire [6:0] D_pipe2_instr;
wire D_pipe2_stall;
wire D_pipe2_squash;
wire [6:0] D_pipe3_instr;
wire D_pipe3_stall;
wire D_pipe3_squash;

assign D_pipe1_instr = {!(|t.p.c.p.ifetch_opcode), (|t.p.c.p.ifetch_opcode) ? t.p.c.p.ifetch_opcode : t.p.c.p.ifetch_func};
assign D_pipe1_stall = t.p.c.p.stall_out_stage1;
assign D_pipe1_squash = t.p.c.p.squash_stage1|t.p.c.p.branch_mispred;

assign D_pipe2_instr = {!(|t.p.c.p.pipereg14_q), (|t.p.c.p.pipereg14_q) ? t.p.c.p.pipereg14_q : t.p.c.p.pipereg15_q};
assign D_pipe2_stall = t.p.c.p.stall_out_stage2;
assign D_pipe2_squash = t.p.c.p.squash_stage2;

assign D_pipe3_instr = {!(|t.p.c.p.pipereg17_q), (|t.p.c.p.pipereg17_q) ? t.p.c.p.pipereg17_q : t.p.c.p.pipereg18_q};
assign D_pipe3_stall = t.p.c.p.stall_out_stage3;
assign D_pipe3_squash = t.p.c.p.squash_stage3;

  /**************** Filesystem writes ********************/
  always@(posedge clk)
    begin
    if (t.p.dbus_en && t.p.dbus_wren && (t.p.dbus_address==32'h8000_0000) && 
        ~t.p.dcpu_wait && resetn)
      $fwrite(fs,"%d\n", t.p.dbus_writedata[31:24]);
    end


  /**************** Vector Trace ********************/
  reg [t.p.c.v.vlanes.NUMBANKS-1:0] read_vector_next;
  reg [t.p.c.v.vlanes.REGIDWIDTH-1:0] dst_vector_next[t.p.c.v.vlanes.NUMBANKS-1:0];
  reg [t.p.c.v.MVL*32-1:0] readdata_vector_next[t.p.c.v.vlanes.NUMBANKS-1:0];
  reg [t.p.c.v.MVL*32-1:0] readdata_vector_next_tmp[t.p.c.v.vlanes.NUMBANKS-1:0];
  integer i,k,m,b;
  always@(posedge clk)
  for (b=t.p.c.v.vlanes.NUMBANKS-1 ; b>=0; b=b-1)
  begin
    read_vector_next[b]<=t.p.c.v.vlanes.D_wb_instrdone[b];

    //Zero out Elm ID part of destination
    dst_vector_next[b]<={
      t.p.c.v.vlanes.vr_c_reg[b][t.p.c.v.vlanes.REGIDWIDTH-1 : t.p.c.v.vlanes.VELMIDWIDTH], 
      {t.p.c.v.vlanes.VELMIDWIDTH{1'b0}}
      };
    if (read_vector_next[b] && !(`DISABLETRACES))
    begin
      readdata_vector_next[b]=0;

      //Get all subvectors, shift them down, and OR them into result
      for (i=0; i<t.p.c.v.MVL/t.p.c.v.NUMLANES; i=i+1)
      begin
        readdata_vector_next_tmp[b]=0;
        for (k=0; k<t.p.c.v.NUMLANES; k=k+1)  //expand 8*VPW to 32-bits
          readdata_vector_next_tmp[b][k*32+:32]=
            //FIXME - UPDATE_ON_BANKS_CHANGE - this code tracks the currently
            //written vector register values across all banks.  It should be a
            //loop but only constants are allowed in module names so it is
            //unrolled instead.  If there is only 1 bank there shouldn't be any
            //conditional statement below, it should be a straight assignment
            //to the first term (everything in the first curly brace).
           `ifdef USE_INHOUSE_LOGIC
               (((i%t.p.c.v.NUMBANKS)==0) ?
                {{32-t.p.c.v.VPW*8+1{t.p.c.v.vlanes.vregfile_vector.bank_gen[0].reg_file1.ram[ (dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-1]}}, //sign-extend
                t.p.c.v.vlanes.vregfile_vector.bank_gen[0].reg_file1.ram[(dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-2 -: t.p.c.v.VPW*8-1]} :
                //((i%t.p.c.v.NUMBANKS)==1) ?
                {{32-t.p.c.v.VPW*8+1{t.p.c.v.vlanes.vregfile_vector.bank_gen[1].reg_file1.ram[ (dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-1]}}, //sign-extend
                t.p.c.v.vlanes.vregfile_vector.bank_gen[1].reg_file1.ram[(dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-2 -: t.p.c.v.VPW*8-1]} /*:
                ((i%t.p.c.v.NUMBANKS)==2) ?
                {{32-t.p.c.v.VPW*8+1{t.p.c.v.vlanes.vregfile_vector.bank_gen[2].reg_file1.mem_data[ (dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-1]}}, //sign-extend
                t.p.c.v.vlanes.vregfile_vector.bank_gen[2].reg_file1.mem_data[(dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-2 -: t.p.c.v.VPW*8-1]} :
                {{32-t.p.c.v.VPW*8+1{t.p.c.v.vlanes.vregfile_vector.bank_gen[3].reg_file1.mem_data[ (dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-1]}}, //sign-extend
                t.p.c.v.vlanes.vregfile_vector.bank_gen[3].reg_file1.mem_data[(dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-2 -: t.p.c.v.VPW*8-1]} */
             );
           `else
               (((i%t.p.c.v.NUMBANKS)==0) ?
                {{32-t.p.c.v.VPW*8+1{t.p.c.v.vlanes.vregfile_vector.bank_gen[0].reg_file1.altera_syncram_inst.mem_data[ (dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-1]}}, //sign-extend
                t.p.c.v.vlanes.vregfile_vector.bank_gen[0].reg_file1.altera_syncram_inst.mem_data[(dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-2 -: t.p.c.v.VPW*8-1]} :
                //((i%t.p.c.v.NUMBANKS)==1) ?
                {{32-t.p.c.v.VPW*8+1{t.p.c.v.vlanes.vregfile_vector.bank_gen[1].reg_file1.altera_syncram_inst.mem_data[ (dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-1]}}, //sign-extend
                t.p.c.v.vlanes.vregfile_vector.bank_gen[1].reg_file1.altera_syncram_inst.mem_data[(dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-2 -: t.p.c.v.VPW*8-1]} /*:
                ((i%t.p.c.v.NUMBANKS)==2) ?
                {{32-t.p.c.v.VPW*8+1{t.p.c.v.vlanes.vregfile_vector.bank_gen[2].reg_file1.mem_data[ (dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-1]}}, //sign-extend
                t.p.c.v.vlanes.vregfile_vector.bank_gen[2].reg_file1.mem_data[(dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-2 -: t.p.c.v.VPW*8-1]} :
                {{32-t.p.c.v.VPW*8+1{t.p.c.v.vlanes.vregfile_vector.bank_gen[3].reg_file1.mem_data[ (dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-1]}}, //sign-extend
                t.p.c.v.vlanes.vregfile_vector.bank_gen[3].reg_file1.mem_data[(dst_vector_next[b]+i)>>t.p.c.v.LOG2NUMBANKS][(k+1)*t.p.c.v.VPW*8-2 -: t.p.c.v.VPW*8-1]} */
             );
           `endif

        readdata_vector_next[b]=readdata_vector_next[b]| 
         (readdata_vector_next_tmp[b] << (t.p.c.v.NUMLANES*32*i));
      end

      $fwrite(vector_trace,"%d | PC=%h | IR=%h | vr%h: %h\n",
            $time,
            pc,
            32'b0 | t.p.c.v.vlanes.D_instr_s5[t.p.c.v.vlanes.FU_ALU],
            dst_vector_next[b][t.p.c.v.vlanes.REGIDWIDTH-1 : t.p.c.v.vlanes.VELMIDWIDTH],
            readdata_vector_next[b]);
    end

  end

  /**************** Vector Flag Trace ********************/
  reg [t.p.c.v.vlanes.NUMBANKS-1:0] read_vf_next;
  reg [t.p.c.v.vlanes.REGIDWIDTH-1:0] dst_vf_next[t.p.c.v.vlanes.NUMBANKS-1:0];
  reg [t.p.c.v.MVL-1:0] readdata_vf_next[t.p.c.v.vlanes.NUMBANKS-1:0];
  integer j,bf;
  always@(posedge clk)
  for (bf=t.p.c.v.vlanes.NUMBANKS-1 ; bf>=0; bf=bf-1)
  begin
    read_vf_next[bf]<=t.p.c.v.vlanes.vf_c_we[bf] && t.p.c.v.vlanes.D_last_subvector_done[bf];
    //Zero out Elm ID part of destination
    dst_vf_next[bf]<=
      t.p.c.v.vlanes.dst[t.p.c.v.vlanes.FU_FALU][5]&~{32'b0,{t.p.c.v.vlanes.VELMIDWIDTH{1'b1}}} ;
    if (read_vf_next[bf] && !(`DISABLETRACES))
    begin
      //a) Zero out
      readdata_vf_next[bf]= 0;

      //b) Get subvectors, shift them down, and OR them into result
      for (j=0; j<t.p.c.v.MVL/t.p.c.v.NUMLANES; j=j+1)
        readdata_vf_next[bf]=readdata_vf_next[bf]| 
          //FIXME - UPDATE_ON_BANKS_CHANGE - this code tracks the currently
          //written vector register values across all banks.  It should be a
          //loop but only constants are allowed in module names so it is
          //unrolled instead.  If there is only 1 bank there shouldn't be any
          //conditional statement below, it should be a straight assignment
          //to the first term (everything in the first curly brace).
        `ifdef USE_INHOUSE_LOGIC
            (((j%t.p.c.v.NUMBANKS)==0) ?
              (t.p.c.v.vlanes.vregfile_flag.bank_gen[0].reg_file1.ram[ (dst_vf_next[bf]+j)>>t.p.c.v.LOG2NUMBANKS ]<<(t.p.c.v.NUMLANES*j)) :
             //((j%t.p.c.v.NUMBANKS)==1) ?
              (t.p.c.v.vlanes.vregfile_flag.bank_gen[1].reg_file1.ram[ (dst_vf_next[bf]+j)>>t.p.c.v.LOG2NUMBANKS ]<<(t.p.c.v.NUMLANES*j)) /* :
             ((j%t.p.c.v.NUMBANKS)==2) ?
              (t.p.c.v.vlanes.vregfile_flag.bank_gen[2].reg_file1.mem_data[ (dst_vf_next[bf]+j)>>t.p.c.v.LOG2NUMBANKS ]<<(t.p.c.v.NUMLANES*j)) :
              (t.p.c.v.vlanes.vregfile_flag.bank_gen[3].reg_file1.mem_data[ (dst_vf_next[bf]+j)>>t.p.c.v.LOG2NUMBANKS ]<<(t.p.c.v.NUMLANES*j)) */ );
        `else
            (((j%t.p.c.v.NUMBANKS)==0) ?
              (t.p.c.v.vlanes.vregfile_flag.bank_gen[0].reg_file1.altera_syncram_inst.mem_data[ (dst_vf_next[bf]+j)>>t.p.c.v.LOG2NUMBANKS ]<<(t.p.c.v.NUMLANES*j)) :
             //((j%t.p.c.v.NUMBANKS)==1) ?
              (t.p.c.v.vlanes.vregfile_flag.bank_gen[1].reg_file1.altera_syncram_inst.mem_data[ (dst_vf_next[bf]+j)>>t.p.c.v.LOG2NUMBANKS ]<<(t.p.c.v.NUMLANES*j)) /* :
             ((j%t.p.c.v.NUMBANKS)==2) ?
              (t.p.c.v.vlanes.vregfile_flag.bank_gen[2].reg_file1.mem_data[ (dst_vf_next[bf]+j)>>t.p.c.v.LOG2NUMBANKS ]<<(t.p.c.v.NUMLANES*j)) :
              (t.p.c.v.vlanes.vregfile_flag.bank_gen[3].reg_file1.mem_data[ (dst_vf_next[bf]+j)>>t.p.c.v.LOG2NUMBANKS ]<<(t.p.c.v.NUMLANES*j)) */ );
        `endif

      $fwrite(vector_trace,"%d | PC=%h | IR=%h | vf%h: %b\n",
            $time,
            pc,
            32'b0 | t.p.c.v.vlanes.D_instr_s5[t.p.c.v.vlanes.FU_FALU],
            dst_vf_next[bf][t.p.c.v.vlanes.REGIDWIDTH-1 : t.p.c.v.vlanes.VELMIDWIDTH],
            readdata_vf_next[bf]);
    end
  end

  /**************** Vector Scalar Trace ********************/
  reg read_vs_next;
  reg [t.p.c.v.LOG2NUMVSREGS-1:0] dst_vs_next;
  reg [t.p.c.v.VSWIDTH-1:0] readdata_vs_next;
  always@(posedge clk)
  begin
    read_vs_next<=t.p.c.v.vs_c_we && (|t.p.c.v.vs_c_reg);
    dst_vs_next<=t.p.c.v.vs_c_reg;
    if (read_vs_next && !(`DISABLETRACES))
    begin
      `ifdef USE_INHOUSE_LOGIC
           readdata_vs_next=t.p.c.v.vregfile_scalar.reg_file1.ram[dst_vs_next]; 
      `else
           readdata_vs_next=t.p.c.v.vregfile_scalar.reg_file1.altera_syncram_inst.mem_data[dst_vs_next]; 
      `endif

      $fwrite(vector_trace,"%d | PC=%h | IR=%h | vs%h: %h\n",
            $time,
            pc,
            32'b0 | t.p.c.v.vlanes.D_instr[6],
            dst_vs_next,
            readdata_vs_next);
    end
  end

  /**************** Vector Control Trace ********************
  reg read_vc_next;
  reg read_vc_next2;
  reg [t.p.c.v.LOG2NUMVCREGS-1:0] dst_vc_next;
  reg [t.p.c.v.VCWIDTH-1:0] readdata_vc_next;
  always@(posedge clk)
  begin
    read_vc_next<= t.p.c.v.vc_c_we || 
                   t.p.c.v.vbase_c_we ||
                   t.p.c.v.vinc_c_we && (|t.p.c.v.vinc_c_reg) ||
                   t.p.c.v.vstride_c_we;
    dst_vc_next<= (t.p.c.v.vc_c_we) ? t.p.c.v.vc_c_reg :
                   (t.p.c.v.vbase_c_we) ? t.p.c.v.vbase_c_reg +32 :
                   (t.p.c.v.vinc_c_we && (|t.p.c.v.vinc_c_reg)) ? t.p.c.v.vinc_c_reg + 48 :
                   t.p.c.v.vstride_c_reg+56; 
    read_vc_next2<=read_vc_next;
  end

  //Write vc before vr - for loads/stores to match MINT trace
  always@(posedge clk)
  begin
    if (read_vc_next && !(`DISABLETRACES))
    begin
      readdata_vc_next=(dst_vc_next[5]==0) ?
                  t.p.c.v.vregfile_control.reg_file1.mem_data[dst_vc_next[4:0]] :
               (dst_vc_next[5:4]==2'b10) ?
                  t.p.c.v.vregfile_base.reg_file1.mem_data[dst_vc_next[3:0]] : 
               (dst_vc_next[5:3]==3'b110) ?
                  t.p.c.v.vregfile_inc.reg_file1.mem_data[dst_vc_next[2:0]] : 
                  t.p.c.v.vregfile_stride.reg_file1.mem_data[dst_vc_next[2:0]]; 

      $fwrite(vector_trace,"%d | PC=%h | IR=%h | vc%h: %h\n",
            $time,
            pc,
            32'b0 | t.p.c.v.vlanes.D_instr[6],
            dst_vc_next,
            readdata_vc_next);
    end
  end
  */

  /**************** Profiling of certain events  ********************/
  wire evt_lanesstalled;
  wire evt_vldstalled;
  wire evt_vststalled;
  wire evt_vldmiss;
  wire evt_vstmiss;
  wire evt_memsubvector;

  assign evt_lanesstalled=(t.p.c.ifetch_bus_wait && ~t.p.c.ibus_wait) ||
                          (t.p.c.cop2_fromcop2_wait || t.p.c.cop2_tocop2_wait);

  assign evt_vldstalled=evt_lanesstalled && 
                        t.p.c.v.vlanes.stall_memunit && 
                        t.p.c.v.vlanes.vmem_unit.op_memop_s2 && 
                        ~t.p.c.v.vlanes.vmem_unit.op_we_s2;

  assign evt_vststalled=evt_lanesstalled && 
                        t.p.c.v.vlanes.stall_memunit && 
                        t.p.c.v.vlanes.vmem_unit.op_memop_s2 && 
                        t.p.c.v.vlanes.vmem_unit.op_we_s2;

  assign evt_vldmiss=evt_vldstalled && 
                      t.p.c.v.vlanes.vmem_unit.dmem_cachemiss && 
                      ~t.p.c.v.vlanes.vmem_unit.dmem_we && 
                      t.p.c.v.vlanes.vmem_unit.dmem_en;

  assign evt_vstmiss=evt_vststalled && 
                      t.p.c.v.vlanes.vmem_unit.dmem_cachemiss && 
                      t.p.c.v.vlanes.vmem_unit.dmem_we && 
                      t.p.c.v.vlanes.vmem_unit.dmem_en;

  assign evt_memsubvector=t.p.c.v.vlanes.vmem_unit.op_memop && t.p.c.v.vlanes.vmem_unit.enable_s2 && ~t.p.c.v.vlanes.vmem_unit.stall;

  always@(posedge clk)
    if (!resetn)
      profile<=!(`PROFILEEEMBC);
    //Capture single pulse - since bus transaction is valid for 2 cycles
    else if (t.dbus_en && ~t.dbus_wren && t.count_select && t.count_select_r)
      profile<=~profile || !(`PROFILEEEMBC);

  always@(posedge clk)
    if (!resetn)
      count_total<=0;
    else if (profile)
      count_total<=count_total+1;

  always@(posedge clk)
    if (!resetn)
      count_lanesstalled<=0;
    else if (evt_lanesstalled && profile)
      count_lanesstalled<=count_lanesstalled+1;

  always@(posedge clk)
    if (!resetn)
      count_vldstalled<=0;
    else if (evt_vldstalled && profile)
      count_vldstalled<=count_vldstalled+1;

  always@(posedge clk)
    if (!resetn)
      count_vststalled<=0;
    else if (evt_vststalled && profile)
      count_vststalled<=count_vststalled+1;

  always@(posedge clk)
    if (!resetn)
      count_vldmiss<=0;
    else if (evt_vldmiss && profile)
      count_vldmiss<=count_vldmiss+1;

  always@(posedge clk)
    if (!resetn)
      count_vstmiss<=0;
    else if (evt_vstmiss && profile)
      count_vstmiss<=count_vstmiss+1;

  always@(posedge clk)
    if (!resetn)
      count_memsubvector<=0;
    else if (evt_memsubvector && profile)
      count_memsubvector<=count_memsubvector+1;


    /*********************** Emulate DDR ********************/

  altmemddr_mem_model ddr2(
    .mem_addr          (mem_addr),
    .mem_ba            (mem_ba),
    .mem_cas_n         (mem_cas_n),
    .mem_cke           (mem_cke),
    .mem_clk           (mem_clk),
    .mem_clk_n         (mem_clk_n),
    .mem_cs_n          (mem_cs_n),
    .mem_dm            (mem_dm),
    .mem_odt           (mem_odt),
    .mem_ras_n         (mem_ras_n),
    .mem_we_n          (mem_we_n),

    .global_reset_n    (global_reset_n),
    .mem_dq            (mem_dq),
    .mem_dqs           (mem_dqs)
  );

  //assign mem_dqsn=(mem_dqs==8'bzzzz_zzzz) ? 8'bzzzz_zzzz : ~mem_dqs;
  assign (weak1, weak0) mem_dq = 0;
  assign (weak1, weak0) mem_dqs = 0;
  assign (weak1, weak0) mem_dqsn = 1;

endmodule

