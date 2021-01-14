//Legal Notice: (C)2009 Altera Corporation. All rights reserved.  Your
//use of Altera Corporation's design tools, logic functions and other
//software and tools, and its AMPP partner logic functions, and any
//output files any of the foregoing (including device programming or
//simulation files), and any associated documentation or information are
//expressly subject to the terms and conditions of the Altera Program
//License Subscription Agreement or other applicable license agreement,
//including, without limitation, that your use is for the sole purpose
//of programming logic devices manufactured by Altera and sold by Altera
//or its authorized distributors.  Please refer to the applicable
//agreement for further details.

// synthesis translate_off
`timescale 1ns / 1ps
// synthesis translate_on

// turn off superfluous verilog processor warnings 
// altera message_level Level1 
// altera message_off 10034 10035 10036 10037 10230 10240 10030 

module altmemddr_mem_model_ram_module (
                                        // inputs:
                                         data,
                                         rdaddress,
                                         rdclken,
                                         wraddress,
                                         wrclock,
                                         wren,

                                        // outputs:
                                         q
                                      )
;

  output  [127: 0] q;
  input   [127: 0] data;
  input   [ 23: 0] rdaddress;
  input            rdclken;
  input   [ 23: 0] wraddress;
  input            wrclock;
  input            wren;

  //PETES CHANGE - make memory 32 bits to match instr.dat/data.dat
  //reg     [127: 0] mem_array [16777215: 0];
  reg     [ 31: 0] mem_array [67108863 : 0];
  wire    [127: 0] q;
  reg     [ 23: 0] read_address;

  //PETES CHANGE
  initial
  begin
    //This is TOO big for 256 MB RAM!  We right shift data by 1
    $readmemh("instr.dat",mem_array,'h100_0000);
    $readmemh("data.dat",mem_array,'h400_0000>>1);
    $display("Done %m 1");
  end


//synthesis translate_off
//////////////// SIMULATION-ONLY CONTENTS
  always @(rdaddress)
    begin
      read_address = rdaddress;
    end


  // Data read is asynchronous.
  //PETES CHANGE - make memory 32 bits to match instr.dat/data.dat
  //assign q = mem_array[read_address];
  assign q = {mem_array[{read_address,2'd3}],
    mem_array[{read_address,2'd2}],
    mem_array[{read_address,2'd1}],
    mem_array[{read_address,2'd0}]
    };

    always @(posedge wrclock)
    begin
      // Write data
      if (wren)
        //PETES CHANGE - make memory 32 bits to match instr.dat/data.dat
        //mem_array[wraddress] <= data;
        {mem_array[{wraddress,2'd3}],
        mem_array[{wraddress,2'd2}],
        mem_array[{wraddress,2'd1}],
        mem_array[{wraddress,2'd0}]} <=data;

    end



//////////////// END SIMULATION-ONLY CONTENTS

//synthesis translate_on
//synthesis read_comments_as_HDL on
//  always @(rdaddress)
//    begin
//      read_address = rdaddress;
//    end
//
//
//  lpm_ram_dp lpm_ram_dp_component
//    (
//      .data (data),
//      .q (q),
//      .rdaddress (read_address),
//      .rdclken (rdclken),
//      .wraddress (wraddress),
//      .wrclock (wrclock),
//      .wren (wren)
//    );
//
//  defparam lpm_ram_dp_component.lpm_file = "UNUSED",
//           lpm_ram_dp_component.lpm_hint = "USE_EAB=ON",
//           lpm_ram_dp_component.lpm_indata = "REGISTERED",
//           lpm_ram_dp_component.lpm_outdata = "UNREGISTERED",
//           lpm_ram_dp_component.lpm_rdaddress_control = "UNREGISTERED",
//           lpm_ram_dp_component.lpm_width = 128,
//           lpm_ram_dp_component.lpm_widthad = 24,
//           lpm_ram_dp_component.lpm_wraddress_control = "REGISTERED",
//           lpm_ram_dp_component.suppress_memory_conversion_warnings = "ON";
//
//synthesis read_comments_as_HDL off

endmodule



// turn off superfluous verilog processor warnings 
// altera message_level Level1 
// altera message_off 10034 10035 10036 10037 10230 10240 10030 

module altmemddr_mem_model (
                             // inputs:
                              mem_addr,
                              mem_ba,
                              mem_cas_n,
                              mem_cke,
                              mem_clk,
                              mem_clk_n,
                              mem_cs_n,
                              mem_dm,
                              mem_odt,
                              mem_ras_n,
                              mem_we_n,

                             // outputs:
                              global_reset_n,
                              mem_dq,
                              mem_dqs,
                              mem_dqs_n
                           )
;

  output           global_reset_n;
  inout   [ 63: 0] mem_dq;
  inout   [  7: 0] mem_dqs;
  inout   [  7: 0] mem_dqs_n;
  input   [ 12: 0] mem_addr;
  input   [  1: 0] mem_ba;
  input            mem_cas_n;
  input            mem_cke;
  input            mem_clk;
  input            mem_clk_n;
  input            mem_cs_n;
  input   [  7: 0] mem_dm;
  input            mem_odt;
  input            mem_ras_n;
  input            mem_we_n;

  wire    [ 23: 0] CODE;
  wire    [ 12: 0] a;
  wire    [  8: 0] addr_col;
  wire    [  1: 0] ba;
  wire             burst_term;
  reg              burst_term_cmd;
  reg     [  2: 0] burstlength;
  reg              burstmode;
  wire             cas_n;
  wire             cke;
  wire             clk;
  wire    [  2: 0] cmd_code;
  wire             cs_n;
  wire    [  1: 0] current_row;
  wire    [  7: 0] dm;
  reg     [ 15: 0] dm_captured;
  reg     [127: 0] dq_captured;
  wire    [ 63: 0] dq_temp;
  wire             dq_valid;
  wire    [  7: 0] dqs_n_temp;
  wire    [  7: 0] dqs_temp;
  wire             dqs_valid;
  reg              dqs_valid_temp;
  reg     [ 63: 0] first_half_dq;
  wire             global_reset_n;
  reg     [  3: 0] index;
  wire    [127: 0] mem_bytes;
  wire    [ 63: 0] mem_dq;
  wire    [  7: 0] mem_dqs;
  wire    [  7: 0] mem_dqs_n;
  reg     [ 12: 0] open_rows [  3: 0];
  wire             ras_n;
  reg     [ 23: 0] rd_addr_pipe_0;
  reg     [ 23: 0] rd_addr_pipe_1;
  reg     [ 23: 0] rd_addr_pipe_10;
  reg     [ 23: 0] rd_addr_pipe_2;
  reg     [ 23: 0] rd_addr_pipe_3;
  reg     [ 23: 0] rd_addr_pipe_4;
  reg     [ 23: 0] rd_addr_pipe_5;
  reg     [ 23: 0] rd_addr_pipe_6;
  reg     [ 23: 0] rd_addr_pipe_7;
  reg     [ 23: 0] rd_addr_pipe_8;
  reg     [ 23: 0] rd_addr_pipe_9;
  reg     [ 23: 0] rd_burst_counter;
  reg     [ 10: 0] rd_valid_pipe;
  wire    [ 23: 0] read_addr_delayed;
  reg              read_cmd;
  reg              read_cmd_echo;
  wire    [127: 0] read_data;
  wire    [ 63: 0] read_dq;
  wire             read_valid;
  reg              read_valid_r;
  reg              read_valid_r2;
  reg              read_valid_r3;
  reg              read_valid_r4;
  reg              reset_n;
  wire    [ 23: 0] rmw_address;
  reg     [127: 0] rmw_temp;
  reg     [ 63: 0] second_half_dq;
  wire    [ 23: 0] txt_code;
  wire             we_n;
  wire    [ 23: 0] wr_addr_delayed;
  reg     [ 23: 0] wr_addr_delayed_r;
  reg     [ 23: 0] wr_addr_pipe_0;
  reg     [ 23: 0] wr_addr_pipe_1;
  reg     [ 23: 0] wr_addr_pipe_10;
  reg     [ 23: 0] wr_addr_pipe_2;
  reg     [ 23: 0] wr_addr_pipe_3;
  reg     [ 23: 0] wr_addr_pipe_4;
  reg     [ 23: 0] wr_addr_pipe_5;
  reg     [ 23: 0] wr_addr_pipe_6;
  reg     [ 23: 0] wr_addr_pipe_7;
  reg     [ 23: 0] wr_addr_pipe_8;
  reg     [ 23: 0] wr_addr_pipe_9;
  reg     [ 23: 0] wr_burst_counter;
  reg     [ 10: 0] wr_valid_pipe;
  reg     [ 10: 0] write_burst_length_pipe;
  reg              write_cmd;
  reg              write_cmd_echo;
  wire             write_to_ram;
  reg              write_to_ram_r;
  wire             write_valid;
  reg              write_valid_r;
  reg              write_valid_r2;
  reg              write_valid_r3;

  /*
initial
  begin
    $write("\n");
    $write("**********************************************************************\n");
    $write("This testbench includes a generated Altera memory model:\n");
    $write("'altmemddr.v', to simulate accesses to the DDR2 SDRAM memory.\n");
    $write(" \n");
    $write("**********************************************************************\n");
  end
  //Synchronous write when (CODE == 24'h205752 (write))
  altmemddr_mem_model_ram_module altmemddr_mem_model_ram
    (
      .data      (rmw_temp),
      .q         (read_data),
      .rdaddress (rmw_address),
      .rdclken   (1'b1),
      .wraddress (wr_addr_delayed_r),
      .wrclock   (clk),
      .wren      (write_to_ram_r)
    );

  assign clk = mem_clk;
  assign dm = mem_dm;
  assign cke = mem_cke;
  assign cs_n = mem_cs_n;
  assign ras_n = mem_ras_n;
  assign cas_n = mem_cas_n;
  assign we_n = mem_we_n;
  assign ba = mem_ba;
  assign a = mem_addr;
  //generate a fake reset inside the memory model
  assign global_reset_n = reset_n;

  initial 
    begin
      reset_n <= 0;
      #100 reset_n <= 1;
      $display("Done %m 2");
    end
  assign cmd_code = (&cs_n) ? 3'b111 : {ras_n, cas_n, we_n};
  assign CODE = (&cs_n) ? 24'h494e48 : txt_code;
  assign addr_col = a[9 : 1];
  assign current_row = {ba};
  // Decode commands into their actions
  always @(posedge clk or negedge reset_n)
    begin
      if (reset_n == 0)
        begin
          write_cmd_echo <= 0;
          read_cmd_echo <= 0;
        end
      else // No Activity if the clock is
      if (cke)
        begin
          // This is a read command
          if (cmd_code == 3'b101)
              read_cmd <= 1'b1;
          else 
            read_cmd <= 1'b0;
          // This is a write command
          if (cmd_code == 3'b100)
              write_cmd <= 1'b1;
          else 
            write_cmd <= 1'b0;
          // This is to terminate a burst
          if (cmd_code == 3'b110)
              burst_term_cmd <= 1'b1;
          else 
            burst_term_cmd <= 1'b0;
          // This is an activate - store the chip/row/bank address in the same order as the DDR controller
          if (cmd_code == 3'b011)
              open_rows[current_row] <= a;
        end
    end


  // Pipes are flushed here
  always @(posedge clk or negedge reset_n)
    begin
      if (reset_n == 0)
        begin
          wr_addr_pipe_1 <= 0;
          wr_addr_pipe_2 <= 0;
          wr_addr_pipe_3 <= 0;
          wr_addr_pipe_4 <= 0;
          wr_addr_pipe_5 <= 0;
          wr_addr_pipe_6 <= 0;
          wr_addr_pipe_7 <= 0;
          wr_addr_pipe_8 <= 0;
          wr_addr_pipe_9 <= 0;
          wr_addr_pipe_10 <= 0;
          rd_addr_pipe_1 <= 0;
          rd_addr_pipe_2 <= 0;
          rd_addr_pipe_3 <= 0;
          rd_addr_pipe_4 <= 0;
          rd_addr_pipe_5 <= 0;
          rd_addr_pipe_6 <= 0;
          rd_addr_pipe_7 <= 0;
          rd_addr_pipe_8 <= 0;
          rd_addr_pipe_9 <= 0;
          rd_addr_pipe_10 <= 0;
        end
      else // No Activity if the clock is
      if (cke)
        begin
          rd_addr_pipe_10 <= rd_addr_pipe_9;
          rd_addr_pipe_9 <= rd_addr_pipe_8;
          rd_addr_pipe_8 <= rd_addr_pipe_7;
          rd_addr_pipe_7 <= rd_addr_pipe_6;
          rd_addr_pipe_6 <= rd_addr_pipe_5;
          rd_addr_pipe_5 <= rd_addr_pipe_4;
          rd_addr_pipe_4 <= rd_addr_pipe_3;
          rd_addr_pipe_3 <= rd_addr_pipe_2;
          rd_addr_pipe_2 <= rd_addr_pipe_1;
          rd_addr_pipe_1 <= rd_addr_pipe_0;
          rd_valid_pipe[10 : 1] <= rd_valid_pipe[9 : 0];
          rd_valid_pipe[0] <= cmd_code == 3'b101;
          wr_addr_pipe_10 <= wr_addr_pipe_9;
          wr_addr_pipe_9 <= wr_addr_pipe_8;
          wr_addr_pipe_8 <= wr_addr_pipe_7;
          wr_addr_pipe_7 <= wr_addr_pipe_6;
          wr_addr_pipe_6 <= wr_addr_pipe_5;
          wr_addr_pipe_5 <= wr_addr_pipe_4;
          wr_addr_pipe_4 <= wr_addr_pipe_3;
          wr_addr_pipe_3 <= wr_addr_pipe_2;
          wr_addr_pipe_2 <= wr_addr_pipe_1;
          wr_addr_pipe_1 <= wr_addr_pipe_0;
          wr_valid_pipe[10 : 1] <= wr_valid_pipe[9 : 0];
          wr_valid_pipe[0] <= cmd_code == 3'b100;
          wr_addr_delayed_r <= wr_addr_delayed;
          write_burst_length_pipe[10 : 1] <= write_burst_length_pipe[9 : 0];
        end
    end


  // Decode CAS Latency from bits a[6:4]
  always @(posedge clk)
    begin
      // No Activity if the clock is
      if (cke)
          //Load mode register - set CAS latency, burst mode and length
          if (cmd_code == 3'b000 && ba == 2'b00)
            begin
              burstmode <= a[3];
              burstlength <= a[2 : 0] << 1;
              //CAS Latency = 3.0
              if (a[6 : 4] == 3'b011)
                  index <= 4'b0010;
              else //CAS Latency = 4.0
              if (a[6 : 4] == 3'b100)
                  index <= 4'b0011;
              else //CAS Latency = 5.0
              if (a[6 : 4] == 3'b101)
                  index <= 4'b0100;
              else //CAS Latency = 6.0
              if (a[6 : 4] == 3'b110)
                  index <= 4'b0101;
              else 
                index <= 4'b0110;
            end
    end


  // Burst support - make the wr_addr & rd_addr keep counting
  always @(posedge clk or negedge reset_n)
    begin
      if (reset_n == 0)
        begin
          wr_addr_pipe_0 <= 0;
          rd_addr_pipe_0 <= 0;
        end
      else 
        begin
          // Reset write address otherwise if the first write is partial it breaks!
          if (cmd_code == 3'b000 && ba == 2'b00)
            begin
              wr_addr_pipe_0 <= 0;
              wr_burst_counter <= 0;
            end
          else if (cmd_code == 3'b100)
            begin
              wr_addr_pipe_0 <= {ba,open_rows[current_row],addr_col};
              wr_burst_counter[23 : 1] <= {ba,open_rows[current_row],addr_col[8 : 1]};
              wr_burst_counter[0] <= addr_col[0] + 1;
            end
          else if (write_cmd || write_to_ram || write_cmd_echo)
            begin
              wr_addr_pipe_0 <= wr_burst_counter;
              wr_burst_counter[0] <= wr_burst_counter[0] + 1;
            end
          else 
            wr_addr_pipe_0 <= 0;
          // Reset read address otherwise if the first write is partial it breaks!
          if (cmd_code == 3'b000 && ba == 2'b00)
              rd_addr_pipe_0 <= 0;
          else if (cmd_code == 3'b101)
            begin
              rd_addr_pipe_0 <= {ba,open_rows[current_row],addr_col};
              rd_burst_counter[23 : 1] <= {ba,open_rows[current_row],addr_col[8 : 1]};
              rd_burst_counter[0] <= addr_col[0] + 1;
            end
          else if (read_cmd || dq_valid || read_valid || read_cmd_echo)
            begin
              rd_addr_pipe_0 <= rd_burst_counter;
              rd_burst_counter[0] <= rd_burst_counter[0] + 1;
            end
          else 
            rd_addr_pipe_0 <= 0;
        end
    end


  // read data transition from single to double clock rate
  always @(posedge clk)
    begin
      first_half_dq <= read_data[127 : 64];
      second_half_dq <= read_data[63 : 0];
    end


  assign read_dq = clk  ? second_half_dq : first_half_dq;
  assign dq_temp = dq_valid  ? read_dq : {64{1'bz}};
  assign dqs_temp = dqs_valid ? {8{clk}} : {8{1'bz}};
  assign dqs_n_temp = dqs_valid ? {8{~clk}} : {8{1'bz}};
  assign mem_dqs = dqs_temp;
  assign mem_dq = dq_temp;
  assign mem_dqs_n = dqs_n_temp;
  //Pipelining registers for burst counting
  always @(posedge clk)
    begin
      write_valid_r <= write_valid;
      read_valid_r <= read_valid;
      write_valid_r2 <= write_valid_r;
      write_valid_r3 <= write_valid_r2;
      write_to_ram_r <= write_to_ram;
      read_valid_r2 <= read_valid_r;
      read_valid_r3 <= read_valid_r2;
      read_valid_r4 <= read_valid_r3;
    end


  assign write_to_ram = write_valid || write_valid_r;
  assign dq_valid = read_valid_r || read_valid_r2 && burst_term;
  assign dqs_valid = dq_valid || dqs_valid_temp;
  // 
  always @(negedge clk)
    begin
      dqs_valid_temp <= read_valid;
    end


  //capture first half of write data with rising edge of DQS, for simulation use only 1 DQS pin
  always @(posedge mem_dqs[0])
    begin
      #0.1 dq_captured[63 : 0] <= mem_dq[63 : 0];
      #0.1 dm_captured[7 : 0] <= mem_dm[7 : 0];
    end


  //capture second half of write data with falling edge of DQS, for simulation use only 1 DQS pin
  always @(negedge mem_dqs[0])
    begin
      #0.1 dq_captured[127 : 64] <= mem_dq[63 : 0];
      #0.1 dm_captured[15 : 8] <= mem_dm[7 : 0];
    end


  //Support for incomplete writes, do a read-modify-write with mem_bytes and the write data
  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[7 : 0] <= dm_captured[0] ? mem_bytes[7 : 0] : dq_captured[7 : 0];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[15 : 8] <= dm_captured[1] ? mem_bytes[15 : 8] : dq_captured[15 : 8];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[23 : 16] <= dm_captured[2] ? mem_bytes[23 : 16] : dq_captured[23 : 16];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[31 : 24] <= dm_captured[3] ? mem_bytes[31 : 24] : dq_captured[31 : 24];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[39 : 32] <= dm_captured[4] ? mem_bytes[39 : 32] : dq_captured[39 : 32];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[47 : 40] <= dm_captured[5] ? mem_bytes[47 : 40] : dq_captured[47 : 40];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[55 : 48] <= dm_captured[6] ? mem_bytes[55 : 48] : dq_captured[55 : 48];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[63 : 56] <= dm_captured[7] ? mem_bytes[63 : 56] : dq_captured[63 : 56];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[71 : 64] <= dm_captured[8] ? mem_bytes[71 : 64] : dq_captured[71 : 64];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[79 : 72] <= dm_captured[9] ? mem_bytes[79 : 72] : dq_captured[79 : 72];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[87 : 80] <= dm_captured[10] ? mem_bytes[87 : 80] : dq_captured[87 : 80];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[95 : 88] <= dm_captured[11] ? mem_bytes[95 : 88] : dq_captured[95 : 88];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[103 : 96] <= dm_captured[12] ? mem_bytes[103 : 96] : dq_captured[103 : 96];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[111 : 104] <= dm_captured[13] ? mem_bytes[111 : 104] : dq_captured[111 : 104];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[119 : 112] <= dm_captured[14] ? mem_bytes[119 : 112] : dq_captured[119 : 112];
    end


  always @(posedge clk)
    begin
      if (write_to_ram)
          rmw_temp[127 : 120] <= dm_captured[15] ? mem_bytes[127 : 120] : dq_captured[127 : 120];
    end


  //DDR2 has variable write latency too, so use index to select which pipeline stage drives valid
  assign write_valid = (index == 0)? wr_valid_pipe[0] :
    (index == 1)? wr_valid_pipe[1] :
    (index == 2)? wr_valid_pipe[2] :
    (index == 3)? wr_valid_pipe[3] :
    (index == 4)? wr_valid_pipe[4] :
    (index == 5)? wr_valid_pipe[5] :
    (index == 6)? wr_valid_pipe[6] :
    (index == 7)? wr_valid_pipe[7] :
    (index == 8)? wr_valid_pipe[8] :
    (index == 9)? wr_valid_pipe[9] :
    wr_valid_pipe[10];

  //DDR2 has variable write latency too, so use index to select which pipeline stage drives addr
  assign wr_addr_delayed = (index == 0)? wr_addr_pipe_0 :
    (index == 1)? wr_addr_pipe_1 :
    (index == 2)? wr_addr_pipe_2 :
    (index == 3)? wr_addr_pipe_3 :
    (index == 4)? wr_addr_pipe_4 :
    (index == 5)? wr_addr_pipe_5 :
    (index == 6)? wr_addr_pipe_6 :
    (index == 7)? wr_addr_pipe_7 :
    (index == 8)? wr_addr_pipe_8 :
    (index == 9)? wr_addr_pipe_9 :
    wr_addr_pipe_10;

  assign burst_term = 1'b1;
  assign mem_bytes = read_data;
  assign rmw_address = (write_to_ram) ? wr_addr_delayed : read_addr_delayed;
  //use index to select which pipeline stage drives addr
  assign read_addr_delayed = (index == 0)? rd_addr_pipe_0 :
    (index == 1)? rd_addr_pipe_1 :
    (index == 2)? rd_addr_pipe_2 :
    (index == 3)? rd_addr_pipe_3 :
    (index == 4)? rd_addr_pipe_4 :
    (index == 5)? rd_addr_pipe_5 :
    (index == 6)? rd_addr_pipe_6 :
    (index == 7)? rd_addr_pipe_7 :
    (index == 8)? rd_addr_pipe_8 :
    (index == 9)? rd_addr_pipe_9 :
    rd_addr_pipe_10;

  //use index to select which pipeline stage drives valid
  assign read_valid = (index == 0)? rd_valid_pipe[0] :
    (index == 1)? rd_valid_pipe[1] :
    (index == 2)? rd_valid_pipe[2] :
    (index == 3)? rd_valid_pipe[3] :
    (index == 4)? rd_valid_pipe[4] :
    (index == 5)? rd_valid_pipe[5] :
    (index == 6)? rd_valid_pipe[6] :
    (index == 7)? rd_valid_pipe[7] :
    (index == 8)? rd_valid_pipe[8] :
    (index == 9)? rd_valid_pipe[9] :
    rd_valid_pipe[10];


//synthesis translate_off
//////////////// SIMULATION-ONLY CONTENTS
  assign txt_code = (cmd_code == 3'h0)? 24'h4c4d52 :
    (cmd_code == 3'h1)? 24'h415246 :
    (cmd_code == 3'h2)? 24'h505245 :
    (cmd_code == 3'h3)? 24'h414354 :
    (cmd_code == 3'h4)? 24'h205752 :
    (cmd_code == 3'h5)? 24'h205244 :
    (cmd_code == 3'h6)? 24'h425354 :
    (cmd_code == 3'h7)? 24'h4e4f50 :
    24'h424144;


//////////////// END SIMULATION-ONLY CONTENTS

//synthesis translate_on
*/
endmodule

