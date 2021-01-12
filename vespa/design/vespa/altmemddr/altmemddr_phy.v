// megafunction wizard: %altmemphy v8.1%
// GENERATION: XML

// ============================================================
// Megafunction Name(s):
// 			altmemddr_phy_alt_mem_phy
// ============================================================
// Generated by altmemphy 8.1 [Altera, IP Toolbench 1.3.0 Build 163]
// ************************************************************
// THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
// ************************************************************
// Copyright (C) 1991-2009 Altera Corporation
// Any megafunction design, and related net list (encrypted or decrypted),
// support information, device programming or simulation file, and any other
// associated documentation or information provided by Altera or a partner
// under Altera's Megafunction Partnership Program may be used only to
// program PLD devices (but not masked PLD devices) from Altera.  Any other
// use of such megafunction design, net list, support information, device
// programming or simulation file, or any other related documentation or
// information is prohibited for any other purpose, including, but not
// limited to modification, reverse engineering, de-compiling, or use with
// any other silicon devices, unless such use is explicitly licensed under
// a separate agreement with Altera or a megafunction partner.  Title to
// the intellectual property, including patents, copyrights, trademarks,
// trade secrets, or maskworks, embodied in any such megafunction design,
// net list, support information, device programming or simulation file, or
// any other related documentation or information provided by Altera or a
// megafunction partner, remains with Altera, the megafunction partner, or
// their respective licensors.  No other licenses, including any licenses
// needed under any third party's intellectual property, are provided herein.


module altmemddr_phy (
	pll_ref_clk,
	global_reset_n,
	soft_reset_n,
	local_address,
	local_read_req,
	local_wdata,
	local_write_req,
	local_size,
	local_be,
	local_refresh_req,
	local_burstbegin,
	ctl_ready,
	ctl_wdata_req,
	ctl_rdata,
	ctl_rdata_valid,
	ctl_refresh_ack,
	ctl_mem_addr_h,
	ctl_mem_addr_l,
	ctl_mem_ba_h,
	ctl_mem_ba_l,
	ctl_mem_cas_n_h,
	ctl_mem_cas_n_l,
	ctl_mem_cke_h,
	ctl_mem_cke_l,
	ctl_mem_cs_n_h,
	ctl_mem_cs_n_l,
	ctl_mem_odt_h,
	ctl_mem_odt_l,
	ctl_mem_ras_n_h,
	ctl_mem_ras_n_l,
	ctl_mem_we_n_h,
	ctl_mem_we_n_l,
	ctl_mem_be,
	ctl_mem_dqs_burst,
	ctl_mem_wdata,
	ctl_mem_wdata_valid,
	ctl_init_done,
	ctl_doing_rd,
	ctl_add_1t_ac_lat,
	ctl_add_1t_odt_lat,
	ctl_add_intermediate_regs,
	ctl_negedge_en,
	dqs_delay_ctrl_import,
	pll_reconfig_enable,
	pll_reconfig_counter_type,
	pll_reconfig_counter_param,
	pll_reconfig_data_in,
	pll_reconfig_read_param,
	pll_reconfig_write_param,
	pll_reconfig,
	oct_ctl_rs_value,
	oct_ctl_rt_value,
	local_autopch_req,
	local_powerdn_req,
	local_self_rfsh_req,
	ctl_self_rfsh_ack,
	ctl_powerdn_ack,
	reset_request_n,
	phy_clk,
	reset_phy_clk_n,
	aux_half_rate_clk,
	aux_full_rate_clk,
	local_ready,
	local_rdata,
	local_rdata_valid,
	local_init_done,
	local_refresh_ack,
	local_wdata_req,
	ctl_address,
	ctl_read_req,
	ctl_wdata,
	ctl_write_req,
	ctl_size,
	ctl_be,
	ctl_refresh_req,
	ctl_burstbegin,
	ctl_mem_rdata,
	ctl_mem_rdata_valid,
	ctl_rlat,
	ctl_usr_mode_rdy,
	mem_addr,
	mem_ba,
	mem_cas_n,
	mem_cke,
	mem_cs_n,
	mem_dm,
	mem_odt,
	mem_ras_n,
	mem_we_n,
	mem_reset_n,
	resynchronisation_successful,
	postamble_successful,
	tracking_successful,
	tracking_adjustment_up,
	tracking_adjustment_down,
	dqs_delay_ctrl_export,
	dll_reference_clk,
	pll_reconfig_clk,
	pll_reconfig_reset,
	pll_reconfig_data_out,
	pll_reconfig_busy,
	local_self_rfsh_ack,
	local_powerdn_ack,
	ctl_autopch_req,
	ctl_powerdn_req,
	ctl_self_rfsh_req,
	mem_clk,
	mem_clk_n,
	mem_dq,
	mem_dqs,
	mem_dqsn);


	input		pll_ref_clk;
	input		global_reset_n;
	input		soft_reset_n;
	input	[23:0]	local_address;
	input		local_read_req;
	input	[127:0]	local_wdata;
	input		local_write_req;
	input	[1:0]	local_size;
	input	[15:0]	local_be;
	input		local_refresh_req;
	input		local_burstbegin;
	input		ctl_ready;
	input		ctl_wdata_req;
	input	[127:0]	ctl_rdata;
	input		ctl_rdata_valid;
	input		ctl_refresh_ack;
	input	[12:0]	ctl_mem_addr_h;
	input	[12:0]	ctl_mem_addr_l;
	input	[1:0]	ctl_mem_ba_h;
	input	[1:0]	ctl_mem_ba_l;
	input		ctl_mem_cas_n_h;
	input		ctl_mem_cas_n_l;
	input	[0:0]	ctl_mem_cke_h;
	input	[0:0]	ctl_mem_cke_l;
	input	[0:0]	ctl_mem_cs_n_h;
	input	[0:0]	ctl_mem_cs_n_l;
	input	[0:0]	ctl_mem_odt_h;
	input	[0:0]	ctl_mem_odt_l;
	input		ctl_mem_ras_n_h;
	input		ctl_mem_ras_n_l;
	input		ctl_mem_we_n_h;
	input		ctl_mem_we_n_l;
	input	[15:0]	ctl_mem_be;
	input		ctl_mem_dqs_burst;
	input	[127:0]	ctl_mem_wdata;
	input		ctl_mem_wdata_valid;
	input		ctl_init_done;
	input		ctl_doing_rd;
	input		ctl_add_1t_ac_lat;
	input		ctl_add_1t_odt_lat;
	input		ctl_add_intermediate_regs;
	input		ctl_negedge_en;
	input	[5:0]	dqs_delay_ctrl_import;
	input		pll_reconfig_enable;
	input	[3:0]	pll_reconfig_counter_type;
	input	[2:0]	pll_reconfig_counter_param;
	input	[8:0]	pll_reconfig_data_in;
	input		pll_reconfig_read_param;
	input		pll_reconfig_write_param;
	input		pll_reconfig;
	input	[13:0]	oct_ctl_rs_value;
	input	[13:0]	oct_ctl_rt_value;
	input		local_autopch_req;
	input		local_powerdn_req;
	input		local_self_rfsh_req;
	input		ctl_self_rfsh_ack;
	input		ctl_powerdn_ack;
	output		reset_request_n;
	output		phy_clk;
	output		reset_phy_clk_n;
	output		aux_half_rate_clk;
	output		aux_full_rate_clk;
	output		local_ready;
	output	[127:0]	local_rdata;
	output		local_rdata_valid;
	output		local_init_done;
	output		local_refresh_ack;
	output		local_wdata_req;
	output	[23:0]	ctl_address;
	output		ctl_read_req;
	output	[127:0]	ctl_wdata;
	output		ctl_write_req;
	output	[1:0]	ctl_size;
	output	[15:0]	ctl_be;
	output		ctl_refresh_req;
	output		ctl_burstbegin;
	output	[127:0]	ctl_mem_rdata;
	output		ctl_mem_rdata_valid;
	output	[5:0]	ctl_rlat;
	output		ctl_usr_mode_rdy;
	output	[12:0]	mem_addr;
	output	[1:0]	mem_ba;
	output		mem_cas_n;
	output	[0:0]	mem_cke;
	output	[0:0]	mem_cs_n;
	output	[7:0]	mem_dm;
	output	[0:0]	mem_odt;
	output		mem_ras_n;
	output		mem_we_n;
	output		mem_reset_n;
	output		resynchronisation_successful;
	output		postamble_successful;
	output		tracking_successful;
	output		tracking_adjustment_up;
	output		tracking_adjustment_down;
	output	[5:0]	dqs_delay_ctrl_export;
	output		dll_reference_clk;
	output		pll_reconfig_clk;
	output		pll_reconfig_reset;
	output	[8:0]	pll_reconfig_data_out;
	output		pll_reconfig_busy;
	output		local_self_rfsh_ack;
	output		local_powerdn_ack;
	output		ctl_autopch_req;
	output		ctl_powerdn_req;
	output		ctl_self_rfsh_req;
	inout	[1:0]	mem_clk;
	inout	[1:0]	mem_clk_n;
	inout	[63:0]	mem_dq;
	inout	[7:0]	mem_dqs;
	inout	[7:0]	mem_dqsn;

	wire [0:0] signal_wire0 = 1'b0;
	wire [0:0] signal_wire1 = 1'b0;

	altmemddr_phy_alt_mem_phy	altmemddr_phy_alt_mem_phy_inst(
		.pll_ref_clk(pll_ref_clk),
		.global_reset_n(global_reset_n),
		.soft_reset_n(soft_reset_n),
		.local_address(local_address),
		.local_read_req(local_read_req),
		.local_wdata(local_wdata),
		.local_write_req(local_write_req),
		.local_size(local_size),
		.local_be(local_be),
		.local_refresh_req(local_refresh_req),
		.local_burstbegin(local_burstbegin),
		.ctl_ready(ctl_ready),
		.ctl_wdata_req(ctl_wdata_req),
		.ctl_rdata(ctl_rdata),
		.ctl_rdata_valid(ctl_rdata_valid),
		.ctl_refresh_ack(ctl_refresh_ack),
		.ctl_mem_addr_h(ctl_mem_addr_h),
		.ctl_mem_addr_l(ctl_mem_addr_l),
		.ctl_mem_ba_h(ctl_mem_ba_h),
		.ctl_mem_ba_l(ctl_mem_ba_l),
		.ctl_mem_cas_n_h(ctl_mem_cas_n_h),
		.ctl_mem_cas_n_l(ctl_mem_cas_n_l),
		.ctl_mem_cke_h(ctl_mem_cke_h),
		.ctl_mem_cke_l(ctl_mem_cke_l),
		.ctl_mem_cs_n_h(ctl_mem_cs_n_h),
		.ctl_mem_cs_n_l(ctl_mem_cs_n_l),
		.ctl_mem_odt_h(ctl_mem_odt_h),
		.ctl_mem_odt_l(ctl_mem_odt_l),
		.ctl_mem_ras_n_h(ctl_mem_ras_n_h),
		.ctl_mem_ras_n_l(ctl_mem_ras_n_l),
		.ctl_mem_we_n_h(ctl_mem_we_n_h),
		.ctl_mem_we_n_l(ctl_mem_we_n_l),
		.ctl_mem_be(ctl_mem_be),
		.ctl_mem_dqs_burst(ctl_mem_dqs_burst),
		.ctl_mem_wdata(ctl_mem_wdata),
		.ctl_mem_wdata_valid(ctl_mem_wdata_valid),
		.ctl_mem_wps_n(signal_wire0),
		.ctl_mem_rps_n(signal_wire1),
		.ctl_init_done(ctl_init_done),
		.ctl_doing_rd(ctl_doing_rd),
		.ctl_add_1t_ac_lat(ctl_add_1t_ac_lat),
		.ctl_add_1t_odt_lat(ctl_add_1t_odt_lat),
		.ctl_add_intermediate_regs(ctl_add_intermediate_regs),
		.ctl_negedge_en(ctl_negedge_en),
		.dqs_delay_ctrl_import(dqs_delay_ctrl_import),
		.pll_reconfig_enable(pll_reconfig_enable),
		.pll_reconfig_counter_type(pll_reconfig_counter_type),
		.pll_reconfig_counter_param(pll_reconfig_counter_param),
		.pll_reconfig_data_in(pll_reconfig_data_in),
		.pll_reconfig_read_param(pll_reconfig_read_param),
		.pll_reconfig_write_param(pll_reconfig_write_param),
		.pll_reconfig(pll_reconfig),
		.oct_ctl_rs_value(oct_ctl_rs_value),
		.oct_ctl_rt_value(oct_ctl_rt_value),
		.local_autopch_req(local_autopch_req),
		.local_powerdn_req(local_powerdn_req),
		.local_self_rfsh_req(local_self_rfsh_req),
		.ctl_self_rfsh_ack(ctl_self_rfsh_ack),
		.ctl_powerdn_ack(ctl_powerdn_ack),
		.reset_request_n(reset_request_n),
		.phy_clk(phy_clk),
		.reset_phy_clk_n(reset_phy_clk_n),
		.aux_half_rate_clk(aux_half_rate_clk),
		.aux_full_rate_clk(aux_full_rate_clk),
		.local_ready(local_ready),
		.local_rdata(local_rdata),
		.local_rdata_valid(local_rdata_valid),
		.local_init_done(local_init_done),
		.local_refresh_ack(local_refresh_ack),
		.local_wdata_req(local_wdata_req),
		.ctl_address(ctl_address),
		.ctl_read_req(ctl_read_req),
		.ctl_wdata(ctl_wdata),
		.ctl_write_req(ctl_write_req),
		.ctl_size(ctl_size),
		.ctl_be(ctl_be),
		.ctl_refresh_req(ctl_refresh_req),
		.ctl_burstbegin(ctl_burstbegin),
		.ctl_mem_rdata(ctl_mem_rdata),
		.ctl_mem_rdata_valid(ctl_mem_rdata_valid),
		.ctl_rlat(ctl_rlat),
		.ctl_usr_mode_rdy(ctl_usr_mode_rdy),
		.mem_addr(mem_addr),
		.mem_ba(mem_ba),
		.mem_cas_n(mem_cas_n),
		.mem_cke(mem_cke),
		.mem_cs_n(mem_cs_n),
		.mem_d(),
		.mem_dm(mem_dm),
		.mem_odt(mem_odt),
		.mem_ras_n(mem_ras_n),
		.mem_we_n(mem_we_n),
		.mem_reset_n(mem_reset_n),
		.mem_doff_n(),
		.mem_rps_n(),
		.mem_wps_n(),
		.resynchronisation_successful(resynchronisation_successful),
		.postamble_successful(postamble_successful),
		.tracking_successful(tracking_successful),
		.tracking_adjustment_up(tracking_adjustment_up),
		.tracking_adjustment_down(tracking_adjustment_down),
		.dqs_delay_ctrl_export(dqs_delay_ctrl_export),
		.dll_reference_clk(dll_reference_clk),
		.pll_reconfig_clk(pll_reconfig_clk),
		.pll_reconfig_reset(pll_reconfig_reset),
		.pll_reconfig_data_out(pll_reconfig_data_out),
		.pll_reconfig_busy(pll_reconfig_busy),
		.local_self_rfsh_ack(local_self_rfsh_ack),
		.local_powerdn_ack(local_powerdn_ack),
		.ctl_autopch_req(ctl_autopch_req),
		.ctl_powerdn_req(ctl_powerdn_req),
		.ctl_self_rfsh_req(ctl_self_rfsh_req),
		.mem_clk(mem_clk),
		.mem_clk_n(mem_clk_n),
		.mem_dq(mem_dq),
		.mem_dqs(mem_dqs),
		.mem_dqsn(mem_dqsn));

	defparam
		altmemddr_phy_alt_mem_phy_inst.ADDR_CMD_2T_EN = 1,
		altmemddr_phy_alt_mem_phy_inst.ADDR_CMD_ADD_1T = "FALSE",
		altmemddr_phy_alt_mem_phy_inst.ADDR_CMD_NEGEDGE_EN = "FALSE",
		altmemddr_phy_alt_mem_phy_inst.ADDR_COUNT_WIDTH = 4,
		altmemddr_phy_alt_mem_phy_inst.CAPTURE_MIMIC_PATH = 0,
		altmemddr_phy_alt_mem_phy_inst.CLOCK_INDEX_WIDTH = 4,
		altmemddr_phy_alt_mem_phy_inst.DDR_MIMIC_PATH_EN = 1,
		altmemddr_phy_alt_mem_phy_inst.DEDICATED_MEMORY_CLK_EN = 0,
		altmemddr_phy_alt_mem_phy_inst.DLL_EXPORT_IMPORT = "EXPORT",
		altmemddr_phy_alt_mem_phy_inst.DLL_DELAY_BUFFER_MODE = "HIGH",
		altmemddr_phy_alt_mem_phy_inst.DLL_DELAY_CHAIN_LENGTH = 12,
		altmemddr_phy_alt_mem_phy_inst.DQS_OUT_MODE = "DELAY_CHAIN3",
		altmemddr_phy_alt_mem_phy_inst.DQS_PHASE_SETTING = 3,
		altmemddr_phy_alt_mem_phy_inst.DQS_PHASE = 9000,
		altmemddr_phy_alt_mem_phy_inst.DWIDTH_RATIO = 2,
		altmemddr_phy_alt_mem_phy_inst.FAMILY = "Stratix III",
		altmemddr_phy_alt_mem_phy_inst.MEM_TCL = "4.0",
		altmemddr_phy_alt_mem_phy_inst.GENERATE_WRITE_DQS = 1,
		altmemddr_phy_alt_mem_phy_inst.LOCAL_IF_CLK_PS = 3750,
		altmemddr_phy_alt_mem_phy_inst.LOCAL_IF_AWIDTH = 24,
		altmemddr_phy_alt_mem_phy_inst.LOCAL_IF_BURST_LENGTH = 2,
		altmemddr_phy_alt_mem_phy_inst.LOCAL_BURST_LEN_BITS = 2,
		altmemddr_phy_alt_mem_phy_inst.LOCAL_IF_DWIDTH = 128,
		altmemddr_phy_alt_mem_phy_inst.LOCAL_IF_DRATE = "FULL",
		altmemddr_phy_alt_mem_phy_inst.LOCAL_IF_TYPE_AVALON_STR = "false",
		altmemddr_phy_alt_mem_phy_inst.MEM_ADDR_CMD_BUS_COUNT = 1,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_MEMTYPE = "DDR2",
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_DWIDTH = 64,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_BE_WIDTH = 16,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_DQS_WIDTH = 8,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_ROWADDR_WIDTH = 13,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_BANKADDR_WIDTH = 2,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_CS_WIDTH = 1,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_DM_WIDTH = 8,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_DM_PINS_EN = 1,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_DQSN_EN = 1,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_OCT_EN = 1,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_DQ_PER_DQS = 8,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_DQS_CAPTURE_EN = 1,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_POSTAMBLE_EN_WIDTH = 8,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_CLK_PAIR_COUNT = 2,
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_CLK_PS = 3750,
		altmemddr_phy_alt_mem_phy_inst.MIMIC_DEBUG_EN = 0,
		altmemddr_phy_alt_mem_phy_inst.MIMIC_PATH_TRACKING_EN = 1,
		altmemddr_phy_alt_mem_phy_inst.NUM_MIMIC_SAMPLE_CYCLES = 6,
		altmemddr_phy_alt_mem_phy_inst.NUM_DEBUG_SAMPLES_TO_STORE = 4096,
		altmemddr_phy_alt_mem_phy_inst.ODT_ADD_1T = "FALSE",
		altmemddr_phy_alt_mem_phy_inst.PLL_EXPORT_IMPORT = "NONE",
		altmemddr_phy_alt_mem_phy_inst.MEM_IF_CLK_PS_STR = "3750 ps",
		altmemddr_phy_alt_mem_phy_inst.PLL_REF_CLK_PS = 10000,
		altmemddr_phy_alt_mem_phy_inst.PLL_STEPS_PER_CYCLE = 32,
		altmemddr_phy_alt_mem_phy_inst.PLL_TYPE = "ENHANCED",
		altmemddr_phy_alt_mem_phy_inst.POSTAMBLE_INITIAL_LAT = 13,
		altmemddr_phy_alt_mem_phy_inst.POSTAMBLE_AWIDTH = 6,
		altmemddr_phy_alt_mem_phy_inst.POSTAMBLE_CALIBRATION_AND_SETUP_EN = 1,
		altmemddr_phy_alt_mem_phy_inst.POSTAMBLE_HALFT_EN = 0,
		altmemddr_phy_alt_mem_phy_inst.POSTAMBLE_RESYNC_LAT_CTL_EN = 0,
		altmemddr_phy_alt_mem_phy_inst.RDP_INITIAL_LAT = 6,
		altmemddr_phy_alt_mem_phy_inst.RDP_RESYNC_LAT_CTL_EN = 0,
		altmemddr_phy_alt_mem_phy_inst.RESYNC_CALIBRATION_AND_SETUP_EN = 1,
		altmemddr_phy_alt_mem_phy_inst.RESYNC_CALIBRATE_ONLY_ONE_BIT_EN = 0,
		altmemddr_phy_alt_mem_phy_inst.RESYNC_PIPELINE_DEPTH = 0,
		altmemddr_phy_alt_mem_phy_inst.READ_LAT_WIDTH = 6,
		altmemddr_phy_alt_mem_phy_inst.TRAINING_DATA_WIDTH = 32,
		altmemddr_phy_alt_mem_phy_inst.SCAN_CLK_DIVIDE_BY = 5,
		altmemddr_phy_alt_mem_phy_inst.USE_MEM_CLK_FOR_ADDR_CMD_CLK = 0,
		altmemddr_phy_alt_mem_phy_inst.ADDR_CMD_ADD_INTERMEDIATE_REGS = "FALSE",
		altmemddr_phy_alt_mem_phy_inst.QDRII_MEM_DLL_NUM_CLK_CYCLES = 2048,
		altmemddr_phy_alt_mem_phy_inst.PLL_RECONFIG_PORTS_EN = 0,
		altmemddr_phy_alt_mem_phy_inst.REG_DIMM = 0;
endmodule

// =========================================================
// altmemphy Wizard Data
// ===============================
// DO NOT EDIT FOLLOWING DATA
// @Altera, IP Toolbench@
// Warning: If you modify this section, altmemphy Wizard may not be able to reproduce your chosen configuration.
// 
// Retrieval info: <?xml version="1.0"?>
// Retrieval info: <MEGACORE title="ALTMEMPHY"  version="8.1"  build="198"  iptb_version="1.3.0 Build 163"  format_version="120" >
// Retrieval info:  <NETLIST_SECTION class="altera.ipbu.flowbase.netlist.model.DDRPHYMVCModel"  active_core="altmemddr_phy_alt_mem_phy" >
// Retrieval info:   <STATIC_SECTION>
// Retrieval info:    <PRIVATES>
// Retrieval info:     <NAMESPACE name = "parameterization">
// Retrieval info:      <PRIVATE name = "debug_en" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "export_debug_port" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "use_generated_memory_model" value="true"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "dedicated_memory_clk_phase_label" value="Dedicated memory clock phase:"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_clk_mhz" value="266.67"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "quartus_project_exists" value="true"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "local_if_drate" value="Full"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "enable_v72_rsu" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "local_if_clk_mhz_label" value="(266.7 MHz)"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "new_variant" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_memtype" value="DDR2 SDRAM"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "pll_ref_clk_mhz" value="100.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_clk_ps_label" value="(3750 ps)"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "family" value="Stratix III"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "project_family" value="Stratix III"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "speed_grade" value="3"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "dedicated_memory_clk_phase" value="0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "pll_ref_clk_ps_label" value="(10000 ps)"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "avalon_burst_length" value="1"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "WIDTH_RATIO" value="4"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_pchaddr_bit" value="10"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_clk_pair_count" value="2"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "vendor" value="Hynix"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "chip_or_dimm" value="Unbuffered DIMM"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_fmax" value="266.667"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_cs_per_dimm" value="1"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "pre_latency_label" value="Fix read latency at:"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "dedicated_memory_clk_en" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_bankaddr_width" value="2"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_preset_rlat" value="0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "post_latency_label" value="cycles (0 cycles=minimum latency, non-deterministic)"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_dyn_deskew_en" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_cs_width" value="1"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_rowaddr_width" value="13"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "local_if_dwidth_label" value="128"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_dm_pins_en" value="Yes"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_preset" value="Custom Richard HYNIX DDR2 256MB SODIMM 533 (JEDEC DDR2-533 256Mb x 64)"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "fast_simulation_en" value="FAST"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_coladdr_width" value="10"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_dq_per_dqs" value="8"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_dwidth" value="64"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tiha_ps" value="375"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tdsh_ck" value="0.2"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_trfc_ns" value="105.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tqh_ck" value="0.36"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tisa_ps" value="250"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tdss_ck" value="0.2"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_tinit_us" value="200.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_trcd_ns" value="15.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_twtr_ck" value="2"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tdqss_ck" value="0.25"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tqhs_ps" value="400"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tdsa_ps" value="100"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tac_ps" value="500"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tdha_ps" value="225"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_tras_ns" value="45.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_twr_ns" value="15.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tdqsck_ps" value="450"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_trp_ns" value="15.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tdqsq_ps" value="300"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_tmrd_ns" value="8.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_trefi_us" value="7.8"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tcl" value="4.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tcl_40_fmax" value="266.667"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_odt" value="50"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_dll_en" value="Yes"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "ac_phase" value="240"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_drv_str" value="Normal"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_oct_en" value="true"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "input_period" value="0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tcl_60_fmax" value="266.667"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "board_skew_ps" value="20"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_if_dqsn_en" value="true"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "dll_external" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tcl_15_fmax" value="533.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tcl_30_fmax" value="266.667"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_bl" value="4"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "ac_clk_select" value="dedicated"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tcl_50_fmax" value="266.667"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tcl_25_fmax" value="533.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tcl_20_fmax" value="533.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "pll_reconfig_ports_en" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_btype" value="Sequential"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "ctl_ecc_en" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "user_refresh_en" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "local_if_type_avalon" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "ctl_self_refresh_en" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "ctl_autopch_en" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "ctl_powerdn_en" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "tool_context" value="STANDALONE"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_srtr" value="Normal"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_mpr_loc" value="Predefined Pattern"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "dss_tinit_rst_us" value="200.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tcl_90_fmax" value="400.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_rtt_wr" value="Dynamic ODT off"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tcl_100_fmax" value="400.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_pasr" value="Full Array"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_asrm" value="Manual SR Reference (SRT)"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_mpr_oper" value="Predefined Pattern"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tcl_80_fmax" value="400.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_drv_impedance" value="RZQ/7"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_rtt_nom" value="ODT Disabled"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_tcl_70_fmax" value="400.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_wtcl" value="5.0"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_dll_pch" value="Fast Exit"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "mem_atcl" value="Disabled"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "pipeline_commands" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "clk_source_sharing_en" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "phy_if_type_afi" value="false"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "shared_sys_clk_source" value="XX"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "ref_clk_source" value="XX"  type="STRING"  enable="1" />
// Retrieval info:     </NAMESPACE>
// Retrieval info:     <NAMESPACE name = "simgen">
// Retrieval info:      <PRIVATE name = "use_alt_top" value="1"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "alt_top" value="altmemddr_phy_alt_mem_phy_sequencer_wrapper"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "family" value="Stratix III"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "filename" value="altmemddr_phy_alt_mem_phy_sequencer_wrapper.vo"  type="STRING"  enable="1" />
// Retrieval info:     </NAMESPACE>
// Retrieval info:     <NAMESPACE name = "simgen2">
// Retrieval info:      <PRIVATE name = "family" value="Stratix III"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "command" value="--simgen_arbitrary_blackbox=+altmemddr_phy_alt_mem_phy_sequencer_wrapper;+altmemddr_phy_alt_mem_phy_reconfig;+altmemddr_phy_alt_mem_phy_pll;+altmemddr_phy_alt_mem_phy_delay --ini=simgen_tri_bus_opt=on"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "parameter" value="SIMGEN_INITIALIZATION_FILE=Y:\vespa\V1.8.0\DE3_81\altmemddr/altmemddr_phy_simgen_init.txt"  type="STRING"  enable="1" />
// Retrieval info:     </NAMESPACE>
// Retrieval info:     <NAMESPACE name = "simgen_enable">
// Retrieval info:      <PRIVATE name = "language" value="Verilog HDL"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "enabled" value="1"  type="STRING"  enable="1" />
// Retrieval info:     </NAMESPACE>
// Retrieval info:     <NAMESPACE name = "qip">
// Retrieval info:      <PRIVATE name = "gx_libs" value="1"  type="STRING"  enable="1" />
// Retrieval info:     </NAMESPACE>
// Retrieval info:     <NAMESPACE name = "greybox">
// Retrieval info:      <PRIVATE name = "filename" value="altmemddr_phy_syn.v"  type="STRING"  enable="1" />
// Retrieval info:     </NAMESPACE>
// Retrieval info:     <NAMESPACE name = "quartus_settings">
// Retrieval info:      <PRIVATE name = "DEVICE" value="EP3SL150F1152C3ES"  type="STRING"  enable="1" />
// Retrieval info:      <PRIVATE name = "FAMILY" value="Stratix III"  type="STRING"  enable="1" />
// Retrieval info:     </NAMESPACE>
// Retrieval info:     <NAMESPACE name = "serializer"/>
// Retrieval info:    </PRIVATES>
// Retrieval info:    <FILES/>
// Retrieval info:    <PORTS/>
// Retrieval info:    <LIBRARIES/>
// Retrieval info:   </STATIC_SECTION>
// Retrieval info:  </NETLIST_SECTION>
// Retrieval info: </MEGACORE>
// =========================================================
// IPFS_FILES: altmemddr_phy_alt_mem_phy_sequencer_wrapper.vo;
// =========================================================