//

`ifdef ALT_MEM_PHY_DEFINES
`else
`include "alt_mem_phy_defines.v"
`endif

//
module altmemddr_phy_alt_mem_phy_sequencer_wrapper (
                                       seq_clk,
                                       reset_seq_n,

                                       ctl_doing_rd,
                                       ctl_mem_rdata,
                                       ctl_mem_rdata_valid,
                                       ctl_init_done,
                                       ctl_usr_mode_rdy,

                                       mmc_seq_done,
                                       mmc_seq_value,
                                       mux_seq_controller_ready,
                                       mux_seq_wdata_req,

                                       phs_shft_busy,
                                       resync_clk_index,
                                       measure_clk_index,

                                       seq_mux_burstbegin,
                                       seq_mux_size,
                                       seq_mux_address,
                                       seq_mux_read_req,
                                       seq_mux_wdata,
                                       seq_mux_write_req,
                                       seq_pll_inc_dec_n,
                                       seq_pll_start_reconfig,
                                       seq_pll_select,
                                       seq_rdp_dec_read_lat_1x,
                                       seq_rdp_dmx_swap,
                                       seq_rdp_inc_read_lat_1x,
                                       seq_poa_lat_dec_1x,
                                       seq_poa_lat_inc_1x,
                                       seq_poa_protection_override_1x,
                                       seq_mmc_start,

                                       resynchronisation_successful,
                                       postamble_successful,
                                       tracking_successful,
                                       tracking_adjustment_up,
                                       tracking_adjustment_down,
                                                                              
                                       rsu_multiple_valid_latencies_err,
                                       rsu_grt_one_dvw_err,             
                                       rsu_no_dvw_err,                  
                                       rsu_codvw_phase,   
                                       rsu_read_latency
                                      );

//Inserted Generics
  localparam FAMILY                             = "Stratix III";
  localparam CLOCK_INDEX_WIDTH                  = 4;
  localparam DWIDTH_RATIO                       = 2;
  localparam LOCAL_IF_BURST_LENGTH              = 2;
  localparam LOCAL_BURST_LEN_BITS               = 2;
  localparam LOCAL_IF_DWIDTH                    = 128;
  localparam LOCAL_IF_AWIDTH                    = 24;
  localparam [39 : 0] LOCAL_IF_TYPE_AVALON_STR           = "false";
  localparam PLL_STEPS_PER_CYCLE                = 32;
  localparam READ_LAT_WIDTH                     = 6;
  localparam TRAINING_DATA_WIDTH                = 32;
  localparam RESYNC_CALIBRATION_AND_SETUP_EN    = 1;
  localparam POSTAMBLE_CALIBRATION_AND_SETUP_EN = 1;
  localparam MIMIC_PATH_TRACKING_EN             = 1;
  localparam RESYNC_CALIBRATE_ONLY_ONE_BIT_EN   = 0;
  localparam ENABLE_DEBUG                       = 0;
  localparam ENABLE_V72_RSU                     = 0;

input wire                                  seq_clk;
input wire                                  reset_seq_n;

input wire                                  ctl_doing_rd;

// The read data here has come from the read dp module
input wire [LOCAL_IF_DWIDTH - 1 : 0]        ctl_mem_rdata;
output wire                                 ctl_mem_rdata_valid;

input wire                                  ctl_init_done;

output wire                                 ctl_usr_mode_rdy;

input wire                                  mmc_seq_done;
input wire                                  mmc_seq_value;
input wire                                  mux_seq_controller_ready;
input wire                                  mux_seq_wdata_req;

input wire                                  phs_shft_busy;
input wire  [CLOCK_INDEX_WIDTH - 1 : 0]     resync_clk_index;
input wire  [CLOCK_INDEX_WIDTH - 1 : 0]     measure_clk_index;

output wire                                 seq_mux_burstbegin;
output wire [LOCAL_BURST_LEN_BITS - 1 : 0]  seq_mux_size;
output wire [LOCAL_IF_AWIDTH - 1 : 0]       seq_mux_address;
output wire                                 seq_mux_read_req;
output wire [LOCAL_IF_DWIDTH - 1 : 0]       seq_mux_wdata;
output wire                                 seq_mux_write_req;
output wire                                 seq_pll_inc_dec_n;
output wire                                 seq_pll_start_reconfig;
output wire [CLOCK_INDEX_WIDTH - 1 : 0]     seq_pll_select;
output wire                                 seq_rdp_dec_read_lat_1x;
output wire                                 seq_rdp_dmx_swap;
output wire                                 seq_rdp_inc_read_lat_1x;

output wire                                 seq_poa_lat_dec_1x;
output wire                                 seq_poa_lat_inc_1x;
output wire                                 seq_poa_protection_override_1x;

output wire                                 seq_mmc_start;

output wire                                 resynchronisation_successful;
output wire                                 postamble_successful;
output wire                                 tracking_successful;
output wire                                 tracking_adjustment_up;
output wire                                 tracking_adjustment_down;

output wire                                 rsu_multiple_valid_latencies_err;    
output wire                                 rsu_grt_one_dvw_err;                 
output wire                                 rsu_no_dvw_err;                      
output wire [11:0]                          rsu_codvw_phase;   
output wire [READ_LAT_WIDTH-1:0]            rsu_read_latency;

wire                                        training_data_write_successful;
wire                                        phs_shft_busy_for_seq;
wire                                        ctl_init_done_for_seq;

reg [1:0]                                   seq_startup_state;
reg                                         enable_sequencer;
reg                                         phs_shft_busy_ams;


`define SEQ_STARTUP_IDLE                    2'h0
`define SEQ_STARTUP_WAIT_PHS_SHFT_LOW       2'h1
`define SEQ_STARTUP_GO                      2'h2

// The FSM ensures the sequencer operates when both the controller and the PLL reconfig
// block are ready :
generate

    if (FAMILY == "Stratix III" || FAMILY == "Cyclone III" || FAMILY == "Stratix IV" || FAMILY == "StratixIII" || FAMILY == "CycloneIII" || FAMILY == "StratixIV")
    begin : gen_no_seq_fsm
        assign phs_shft_busy_for_seq = phs_shft_busy;
        assign ctl_init_done_for_seq = ctl_init_done;
    end
    
    else
    begin : gen_seq_fsm
    
        assign phs_shft_busy_for_seq = enable_sequencer && phs_shft_busy;
        assign ctl_init_done_for_seq = enable_sequencer && ctl_init_done;                
        
        always @(posedge seq_clk or negedge reset_seq_n)
        begin
        
            if (reset_seq_n == 1'b0)
            begin
                seq_startup_state <= `SEQ_STARTUP_IDLE;
                enable_sequencer  <= 1'b0;
                phs_shft_busy_ams <= 1'b1;
            end
            
            else
            begin
            
                // Metastability protection on the phs_shft_busy input :
                phs_shft_busy_ams <= phs_shft_busy;
                
                casez (seq_startup_state) 
                
                `SEQ_STARTUP_IDLE :
                begin
                
                    if (ctl_init_done == 1'b1)
                    begin            
                    
                        if (phs_shft_busy_ams == 1'b1)
                        begin                
                            seq_startup_state <= `SEQ_STARTUP_WAIT_PHS_SHFT_LOW;                    
                        end
                        else
                        begin
                            seq_startup_state <= `SEQ_STARTUP_GO;                                    
                        end
                        
                    end
                    
                    else
                    begin            
                        seq_startup_state <= `SEQ_STARTUP_IDLE;                                               
                    end
                    
                end
                
                `SEQ_STARTUP_WAIT_PHS_SHFT_LOW :
                begin
                
                    if (phs_shft_busy_ams == 1'b1)
                    begin                
                        seq_startup_state <= `SEQ_STARTUP_WAIT_PHS_SHFT_LOW;                    
                    end
                    else
                    begin
                        seq_startup_state <= `SEQ_STARTUP_GO;                                    
                    end
                
                end
                        
                `SEQ_STARTUP_GO :
                begin
                    seq_startup_state <= `SEQ_STARTUP_GO;                                               
                    enable_sequencer  <= 1'b1;
                end
                
                default :
                begin
                    seq_startup_state <= `SEQ_STARTUP_IDLE;
                    enable_sequencer <= 1'b0;        
                end
                
                endcase
        
            end // not reset       
        end // always
    end // gen_seq_fsm
    
endgenerate    




//#pb_prefix ("sequencer")
alt_mem_phy_sequencer # (
    .DATA_WIDTH_RATIO                                   (DWIDTH_RATIO),
    .PLL_PHASES                                         (PLL_STEPS_PER_CYCLE),
    .TRAINING_DATA_WIDTH                                (TRAINING_DATA_WIDTH),
    .LATENCY_COUNTER_WIDTH                              (READ_LAT_WIDTH),
    .MEMORY_CONTROLLER_AVALON_INTERFACE                 (LOCAL_IF_TYPE_AVALON_STR),
    .MEMORY_CONTROLLER_MAXIMUM_BURST_LENGTH             (LOCAL_IF_BURST_LENGTH),
    .MEMORY_CONTROLLER_MAXIMUM_BURST_LENGTH_WIDTH       (LOCAL_BURST_LEN_BITS),
    .MEMORY_CONTROLLER_ADDRESS_WIDTH                    (LOCAL_IF_AWIDTH),
    .MEMORY_CONTROLLER_DATA_WIDTH                       (LOCAL_IF_DWIDTH),
    .CLOCK_INDEX_WIDTH                                  (CLOCK_INDEX_WIDTH),
    .CLOCK_CYCLES_BETWEEN_MEASURE_CLOCK_PHASE_DETECTION (20000000),
    .METASTABILITY_GUARD_REGISTERS                      (2),
    .ENABLE_RESYNCHRONISATION_CALIBRATION_AND_SETUP     (RESYNC_CALIBRATION_AND_SETUP_EN),
    .ENABLE_ONE_BIT_RESYNCHRONISATION_CALIBRATION       (RESYNC_CALIBRATE_ONLY_ONE_BIT_EN),
    .ENABLE_POSTAMBLE_CALIBRATION_AND_SETUP             (POSTAMBLE_CALIBRATION_AND_SETUP_EN),
    .ENABLE_TRACKING                                    (MIMIC_PATH_TRACKING_EN),
    .ENABLE_INTERNAL_RAM_POWER_UP_INITIALISATION        (0),
    .ENABLE_DEBUGGING_INFORMATION                       (ENABLE_DEBUG),
    .ENABLE_V72_RSU                                     (ENABLE_V72_RSU)
//#pb_prefix ("sequencer")
) alt_mem_phy_sequencer_inst (
    .clock                                              (seq_clk),
    .asynchronous_reset                                 (reset_seq_n),
    .calibrate_and_setup_physical_interface             (ctl_init_done_for_seq),
    .memory_controller_ready                            (mux_seq_controller_ready),
    .memory_controller_write_data_request               (mux_seq_wdata_req),
    .memory_controller_read_data                        (ctl_mem_rdata),
    .memory_controller_read_command_issued              (ctl_doing_rd),
    .resynchronisation_clock_index                      (resync_clk_index),
    .measure_clock_index                                (measure_clk_index),
    .pll_reconfiguring                                  (phs_shft_busy_for_seq),
    .mimic_path_sample_valid                            (mmc_seq_done),
    .mimic_path_sample                                  (mmc_seq_value),
                                                      
    .memory_controller_write_request                    (seq_mux_write_req),
    .memory_controller_read_request                     (seq_mux_read_req),
    .memory_controller_write_data                       (seq_mux_wdata),
    .memory_controller_address                          (seq_mux_address),
    .memory_controller_burst_begin                      (seq_mux_burstbegin),
    .memory_controller_burst_length                     (seq_mux_size),
    .memory_controller_read_data_valid                  (ctl_mem_rdata_valid),
    .pll_reconfigure_request                            (seq_pll_start_reconfig),
    .pll_reconfigure_clock_index                        (seq_pll_select),
    .pll_reconfigure_direction                          (seq_pll_inc_dec_n),
    .dqs_enable_set                                     (seq_poa_protection_override_1x),
    .dqs_enable_latency_decrement                       (seq_poa_lat_dec_1x),
    .dqs_enable_latency_increment                       (seq_poa_lat_inc_1x),
    .mimic_path_sample_request                          (seq_mmc_start),
    .training_data_write_successful                     (training_data_write_successful),
    .postamble_calibration_successful                   (postamble_successful),
    .resynchronisation_clock_phase_incremented          (tracking_adjustment_up),
    .resynchronisation_clock_phase_decremented          (tracking_adjustment_down),
    .physical_interface_calibrated_and_setup            (ctl_usr_mode_rdy),
    .resynchronisation_calibration_successful           (resynchronisation_successful),
    .rsu_multiple_valid_latencies_err                   (rsu_multiple_valid_latencies_err),
    .rsu_grt_one_dvw_err                                (rsu_grt_one_dvw_err),
    .rsu_no_dvw_err                                     (rsu_no_dvw_err),
    .rsu_codvw_phase                                    (rsu_codvw_phase),
    .rsu_read_latency                                   (rsu_read_latency)
);

assign seq_rdp_dmx_swap         = 1'b0;
assign seq_rdp_inc_read_lat_1x  = 1'b0;
assign seq_rdp_dec_read_lat_1x  = 1'b0;

assign tracking_successful = training_data_write_successful && postamble_successful && resynchronisation_successful && ctl_usr_mode_rdy;
    
endmodule




