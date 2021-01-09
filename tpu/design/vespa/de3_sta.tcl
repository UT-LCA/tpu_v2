#Tcl script for generating timing reports from TimeQuest

#All Summaries
report_clock_fmax_summary -panel_name "Fmax Summary"
qsta_utility::generate_all_summary_tables
qsta_utility::generate_all_core_timing_reports "Core Timing Report" 200

#Internal processor paths write to: de3_internalcpu_timing.rpt
report_timing -setup -from_clock [get_clocks { clkx2_pll|altpll_component|auto_generated|pll1|clk[0] }] -to_clock [get_clocks { clkx2_pll|altpll_component|auto_generated|pll1|clk[0] }] -npaths 20 -detail full_path -panel_name {Internal CPU Clock Setup}

#Internal memory system paths write to: de3_internalmem_timing.rpt
report_timing -setup -from_clock [get_clocks { mem_hierarchy|altmemddr|altmemddr_controller_phy_inst|alt_mem_phy_inst|altmemddr_phy_alt_mem_phy_inst|clk|full_rate.pll|altpll_component|auto_generated|pll1|clk[2] }] -to_clock [get_clocks { mem_hierarchy|altmemddr|altmemddr_controller_phy_inst|alt_mem_phy_inst|altmemddr_phy_alt_mem_phy_inst|clk|full_rate.pll|altpll_component|auto_generated|pll1|clk[2] }] -npaths 20 -detail full_path -panel_name {Internal Mem Clock Setup} 
