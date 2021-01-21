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
`timescale 1ps / 1ps
// synthesis translate_on

// turn off superfluous verilog processor warnings 
// altera message_level Level1 
// altera message_off 10034 10035 10036 10037 10230 10240 10030 

module altmemddr_controller_phy (
                                  // inputs:
                                   dqs_delay_ctrl_import,
                                   global_reset_n,
                                   local_address,
                                   local_autopch_req,
                                   local_be,
                                   local_burstbegin,
                                   local_powerdn_req,
                                   local_read_req,
                                   local_refresh_req,
                                   local_self_rfsh_req,
                                   local_size,
                                   local_wdata,
                                   local_write_req,
                                   oct_ctl_rs_value,
                                   oct_ctl_rt_value,
                                   pll_reconfig,
                                   pll_reconfig_counter_param,
                                   pll_reconfig_counter_type,
                                   pll_reconfig_data_in,
                                   pll_reconfig_enable,
                                   pll_reconfig_read_param,
                                   pll_reconfig_soft_reset_en_n,
                                   pll_reconfig_write_param,
                                   pll_ref_clk,
                                   soft_reset_n,

                                  // outputs:
                                   aux_full_rate_clk,
                                   aux_half_rate_clk,
                                   dll_reference_clk,
                                   dqs_delay_ctrl_export,
                                   local_init_done,
                                   local_powerdn_ack,
                                   local_rdata,
                                   local_rdata_error,
                                   local_rdata_valid,
                                   local_ready,
                                   local_refresh_ack,
                                   local_self_rfsh_ack,
                                   local_wdata_req,
                                   mem_addr,
                                   mem_ba,
                                   mem_cas_n,
                                   mem_cke,
                                   mem_clk,
                                   mem_clk_n,
                                   mem_cs_n,
                                   mem_dm,
                                   mem_dq,
                                   mem_dqs,
                                   mem_dqsn,
                                   mem_odt,
                                   mem_ras_n,
                                   mem_reset_n,
                                   mem_we_n,
                                   phy_clk,
                                   pll_reconfig_busy,
                                   pll_reconfig_clk,
                                   pll_reconfig_data_out,
                                   pll_reconfig_reset,
                                   reset_phy_clk_n,
                                   reset_request_n
                                )
;

  output           aux_full_rate_clk;
  output           aux_half_rate_clk;
  output           dll_reference_clk;
  output  [  5: 0] dqs_delay_ctrl_export;
  output           local_init_done;
  output           local_powerdn_ack;
  output  [127: 0] local_rdata;
  output           local_rdata_error;
  output           local_rdata_valid;
  output           local_ready;
  output           local_refresh_ack;
  output           local_self_rfsh_ack;
  output           local_wdata_req;
  output  [ 12: 0] mem_addr;
  output  [  1: 0] mem_ba;
  output           mem_cas_n;
  output  [  0: 0] mem_cke;
  inout   [  1: 0] mem_clk;
  inout   [  1: 0] mem_clk_n;
  output  [  0: 0] mem_cs_n;
  output  [  7: 0] mem_dm;
  inout   [ 63: 0] mem_dq;
  inout   [  7: 0] mem_dqs;
  inout   [  7: 0] mem_dqsn;
  output  [  0: 0] mem_odt;
  output           mem_ras_n;
  output           mem_reset_n;
  output           mem_we_n;
  output           phy_clk;
  output           pll_reconfig_busy;
  output           pll_reconfig_clk;
  output  [  8: 0] pll_reconfig_data_out;
  output           pll_reconfig_reset;
  output           reset_phy_clk_n;
  output           reset_request_n;
  input   [  5: 0] dqs_delay_ctrl_import;
  input            global_reset_n;
  input   [ 23: 0] local_address;
  input            local_autopch_req;
  input   [ 15: 0] local_be;
  input            local_burstbegin;
  input            local_powerdn_req;
  input            local_read_req;
  input            local_refresh_req;
  input            local_self_rfsh_req;
  input   [  1: 0] local_size;
  input   [127: 0] local_wdata;
  input            local_write_req;
  input   [ 13: 0] oct_ctl_rs_value;
  input   [ 13: 0] oct_ctl_rt_value;
  input            pll_reconfig;
  input   [  2: 0] pll_reconfig_counter_param;
  input   [  3: 0] pll_reconfig_counter_type;
  input   [  8: 0] pll_reconfig_data_in;
  input            pll_reconfig_enable;
  input            pll_reconfig_read_param;
  input            pll_reconfig_soft_reset_en_n;
  input            pll_reconfig_write_param;
  input            pll_ref_clk;
  input            soft_reset_n;

  wire             aux_full_rate_clk;
  wire             aux_half_rate_clk;
  wire    [  1: 0] bank_addr;
  wire    [  8: 0] col_addr;
  wire    [ 15: 0] control_be;
  wire    [  7: 0] control_doing_rd_sig_from_ctlr;
  wire             control_doing_rd_sig_to_phy;
  wire    [127: 0] control_wdata;
  wire    [  7: 0] control_wdata_valid_sig_from_ctlr;
  wire             control_wdata_valid_sig_to_phy;
  wire             cs_addr;
  wire    [ 23: 0] ctl_address;
  wire             ctl_autopch_req;
  wire    [ 15: 0] ctl_be;
  wire             ctl_burstbegin_sig;
  wire             ctl_init_done;
  wire    [ 12: 0] ctl_mem_a;
  wire    [  1: 0] ctl_mem_ba;
  wire             ctl_mem_cas_n;
  wire             ctl_mem_cke_h;
  wire             ctl_mem_cke_l;
  wire             ctl_mem_cs_n;
  wire    [  7: 0] ctl_mem_dqs_burst_sig_from_ctlr;
  wire             ctl_mem_dqs_burst_sig_to_phy;
  wire             ctl_mem_odt;
  wire             ctl_mem_ras_n;
  wire    [127: 0] ctl_mem_rdata;
  wire             ctl_mem_rdata_valid;
  wire             ctl_mem_we_n;
  wire             ctl_powerdn_ack;
  wire             ctl_powerdn_req;
  wire    [127: 0] ctl_rdata;
  wire             ctl_rdata_valid;
  wire             ctl_read_req;
  wire             ctl_ready;
  wire             ctl_refresh_ack;
  wire             ctl_refresh_req_sig;
  wire             ctl_self_rfsh_ack;
  wire             ctl_self_rfsh_req;
  wire    [  1: 0] ctl_size;
  wire             ctl_usr_mode_rdy_sig;
  wire    [127: 0] ctl_wdata;
  wire             ctl_wdata_req;
  wire             ctl_write_req;
  wire             dll_reference_clk;
  wire    [  5: 0] dqs_delay_ctrl_export;
  wire    [ 15: 0] local_be_sig;
  wire             local_init_done;
  wire             local_powerdn_ack;
  wire    [127: 0] local_rdata;
  wire             local_rdata_error;
  wire    [127: 0] local_rdata_sig;
  wire             local_rdata_valid;
  wire             local_ready;
  wire             local_refresh_ack;
  wire             local_self_rfsh_ack;
  wire             local_wdata_req;
  wire    [127: 0] local_wdata_sig;
  wire    [ 12: 0] mem_addr;
  wire    [  1: 0] mem_ba;
  wire             mem_cas_n;
  wire    [  0: 0] mem_cke;
  wire    [  1: 0] mem_clk;
  wire    [  1: 0] mem_clk_n;
  wire    [  0: 0] mem_cs_n;
  wire    [  7: 0] mem_dm;
  wire    [ 63: 0] mem_dq;
  wire    [  7: 0] mem_dqs;
  wire    [  7: 0] mem_dqsn;
  wire    [  0: 0] mem_odt;
  wire             mem_ras_n;
  wire             mem_reset_n;
  wire             mem_we_n;
  wire    [ 15: 0] no_connect_dm;
  wire             phy_clk;
  wire             pll_reconfig_busy;
  wire             pll_reconfig_clk;
  wire    [  8: 0] pll_reconfig_data_out;
  wire             pll_reconfig_reset;
  wire             postamble_successful;
  wire             reset_phy_clk_n;
  wire             reset_request_n;
  wire             resynchronisation_successful;
  wire    [ 12: 0] row_addr;
  wire             tie_high;
  wire             tie_low;
  wire             tracking_adjustment_down;
  wire             tracking_adjustment_up;
  wire             tracking_successful;
  assign local_wdata_sig[127 : 0] = local_wdata[127 : 0];
  assign local_be_sig[15 : 0] = local_be[15 : 0];
  assign local_rdata = local_rdata_sig[127 : 0];
  assign tie_low = 0;
  assign tie_high = {1{1'b1}};
  assign cs_addr = 0;
  //


  assign bank_addr = ctl_address[23 : 22];

  assign row_addr = ctl_address[21 : 9];
  assign col_addr = ctl_address[8 : 0];
  assign control_wdata_valid_sig_to_phy = control_wdata_valid_sig_from_ctlr[0];
  assign control_doing_rd_sig_to_phy = control_doing_rd_sig_from_ctlr[0];
  assign ctl_mem_dqs_burst_sig_to_phy = ctl_mem_dqs_burst_sig_from_ctlr[0];
  //Read data comes out of PHY (ctl_mem_rdata) and straight back into the PHY (ctl_rdata) from
  //where it goes to the user interface output (local_rdata). If your controller needs to delay
  //the read data for any reason, you will need to insert your controller between these two ports.
  //This also applies to the rdata_valid signal. 

  altmemddr_auk_ddr_hp_controller_wrapper altmemddr_auk_ddr_hp_controller_wrapper_inst
    (
      .clk (phy_clk),
      .control_be (control_be),
      .control_dm (no_connect_dm),
      .control_doing_rd (control_doing_rd_sig_from_ctlr),
      .control_doing_wr (),
      .control_dqs_burst (ctl_mem_dqs_burst_sig_from_ctlr),
      .control_rdata (ctl_mem_rdata),
      .control_rdata_valid (ctl_mem_rdata_valid),
      .control_wdata (control_wdata),
      .control_wdata_valid (control_wdata_valid_sig_from_ctlr),
      .control_wlat (5'b00000),
      .ddr_a (ctl_mem_a),
      .ddr_ba (ctl_mem_ba),
      .ddr_cas_n (ctl_mem_cas_n),
      .ddr_cke_h (ctl_mem_cke_h),
      .ddr_cke_l (ctl_mem_cke_l),
      .ddr_cs_n (ctl_mem_cs_n),
      .ddr_odt (ctl_mem_odt),
      .ddr_ras_n (ctl_mem_ras_n),
      .ddr_we_n (ctl_mem_we_n),
      .local_autopch_req (ctl_autopch_req),
      .local_bank_addr (bank_addr),
      .local_be (ctl_be),
      .local_burstbegin (ctl_burstbegin_sig),
      .local_col_addr (col_addr),
      .local_cs_addr (cs_addr),
      .local_init_done (ctl_init_done),
      .local_powerdn_ack (ctl_powerdn_ack),
      .local_powerdn_req (ctl_powerdn_req),
      .local_rdata (ctl_rdata),
      .local_rdata_valid (ctl_rdata_valid),
      .local_read_req (ctl_read_req),
      .local_ready (ctl_ready),
      .local_refresh_ack (ctl_refresh_ack),
      .local_refresh_req (ctl_refresh_req_sig),
      .local_row_addr (row_addr),
      .local_self_rfsh_ack (ctl_self_rfsh_ack),
      .local_self_rfsh_req (ctl_self_rfsh_req),
      .local_size (ctl_size),
      .local_wdata (ctl_wdata),
      .local_wdata_req (ctl_wdata_req),
      .local_write_req (ctl_write_req),
      .reset_n (reset_phy_clk_n),
      .seq_cal_complete (1'b1)
    );


  altmemddr_phy alt_mem_phy_inst
    (
      .aux_full_rate_clk (aux_full_rate_clk),
      .aux_half_rate_clk (aux_half_rate_clk),
      .ctl_add_1t_ac_lat (tie_low),
      .ctl_add_1t_odt_lat (tie_low),
      .ctl_add_intermediate_regs (tie_low),
      .ctl_address (ctl_address),
      .ctl_autopch_req (ctl_autopch_req),
      .ctl_be (ctl_be[15 : 0]),
      .ctl_burstbegin (ctl_burstbegin_sig),
      .ctl_doing_rd (control_doing_rd_sig_to_phy),
      .ctl_init_done (ctl_init_done),
      .ctl_mem_addr_h (ctl_mem_a),
      .ctl_mem_addr_l (ctl_mem_a),
      .ctl_mem_ba_h (ctl_mem_ba),
      .ctl_mem_ba_l (ctl_mem_ba),
      .ctl_mem_be (control_be),
      .ctl_mem_cas_n_h (ctl_mem_cas_n),
      .ctl_mem_cas_n_l (ctl_mem_cas_n),
      .ctl_mem_cke_h (ctl_mem_cke_l),
      .ctl_mem_cke_l (ctl_mem_cke_l),
      .ctl_mem_cs_n_h (ctl_mem_cs_n),
      .ctl_mem_cs_n_l (ctl_mem_cs_n),
      .ctl_mem_dqs_burst (ctl_mem_dqs_burst_sig_to_phy),
      .ctl_mem_odt_h (ctl_mem_odt),
      .ctl_mem_odt_l (ctl_mem_odt),
      .ctl_mem_ras_n_h (ctl_mem_ras_n),
      .ctl_mem_ras_n_l (ctl_mem_ras_n),
      .ctl_mem_rdata (ctl_mem_rdata),
      .ctl_mem_rdata_valid (ctl_mem_rdata_valid),
      .ctl_mem_wdata (control_wdata),
      .ctl_mem_wdata_valid (control_wdata_valid_sig_to_phy),
      .ctl_mem_we_n_h (ctl_mem_we_n),
      .ctl_mem_we_n_l (ctl_mem_we_n),
      .ctl_negedge_en (tie_low),
      .ctl_powerdn_ack (ctl_powerdn_ack),
      .ctl_powerdn_req (ctl_powerdn_req),
      .ctl_rdata (ctl_rdata),
      .ctl_rdata_valid (ctl_rdata_valid),
      .ctl_read_req (ctl_read_req),
      .ctl_ready (ctl_ready),
      .ctl_refresh_ack (ctl_refresh_ack),
      .ctl_refresh_req (ctl_refresh_req_sig),
      .ctl_self_rfsh_ack (ctl_self_rfsh_ack),
      .ctl_self_rfsh_req (ctl_self_rfsh_req),
      .ctl_size (ctl_size[1 : 0]),
      .ctl_usr_mode_rdy (ctl_usr_mode_rdy_sig),
      .ctl_wdata (ctl_wdata),
      .ctl_wdata_req (ctl_wdata_req),
      .ctl_write_req (ctl_write_req),
      .dll_reference_clk (dll_reference_clk),
      .dqs_delay_ctrl_export (dqs_delay_ctrl_export),
      .dqs_delay_ctrl_import (dqs_delay_ctrl_import),
      .global_reset_n (global_reset_n),
      .local_address (local_address),
      .local_autopch_req (local_autopch_req),
      .local_be (local_be_sig),
      .local_burstbegin (local_burstbegin),
      .local_init_done (local_init_done),
      .local_powerdn_ack (local_powerdn_ack),
      .local_powerdn_req (local_powerdn_req),
      .local_rdata (local_rdata_sig),
      .local_rdata_valid (local_rdata_valid),
      .local_read_req (local_read_req),
      .local_ready (local_ready),
      .local_refresh_ack (local_refresh_ack),
      .local_refresh_req (local_refresh_req),
      .local_self_rfsh_ack (local_self_rfsh_ack),
      .local_self_rfsh_req (local_self_rfsh_req),
      .local_size (local_size[1 : 0]),
      .local_wdata (local_wdata_sig),
      .local_wdata_req (local_wdata_req),
      .local_write_req (local_write_req),
      .mem_addr (mem_addr),
      .mem_ba (mem_ba),
      .mem_cas_n (mem_cas_n),
      .mem_cke (mem_cke),
      .mem_clk (mem_clk[1 : 0]),
      .mem_clk_n (mem_clk_n[1 : 0]),
      .mem_cs_n (mem_cs_n),
      .mem_dm (mem_dm[7 : 0]),
      .mem_dq (mem_dq),
      .mem_dqs (mem_dqs[7 : 0]),
      .mem_dqsn (mem_dqsn[7 : 0]),
      .mem_odt (mem_odt),
      .mem_ras_n (mem_ras_n),
      .mem_reset_n (mem_reset_n),
      .mem_we_n (mem_we_n),
      .oct_ctl_rs_value (oct_ctl_rs_value),
      .oct_ctl_rt_value (oct_ctl_rt_value),
      .phy_clk (phy_clk),
      .pll_reconfig (pll_reconfig),
      .pll_reconfig_busy (pll_reconfig_busy),
      .pll_reconfig_clk (pll_reconfig_clk),
      .pll_reconfig_counter_param (pll_reconfig_counter_param),
      .pll_reconfig_counter_type (pll_reconfig_counter_type),
      .pll_reconfig_data_in (pll_reconfig_data_in),
      .pll_reconfig_data_out (pll_reconfig_data_out),
      .pll_reconfig_enable (pll_reconfig_enable),
      .pll_reconfig_read_param (pll_reconfig_read_param),
      .pll_reconfig_reset (pll_reconfig_reset),
      .pll_reconfig_write_param (pll_reconfig_write_param),
      .pll_ref_clk (pll_ref_clk),
      .postamble_successful (postamble_successful),
      .reset_phy_clk_n (reset_phy_clk_n),
      .reset_request_n (reset_request_n),
      .resynchronisation_successful (resynchronisation_successful),
      .soft_reset_n (soft_reset_n),
      .tracking_adjustment_down (tracking_adjustment_down),
      .tracking_adjustment_up (tracking_adjustment_up),
      .tracking_successful (tracking_successful)
    );



endmodule

