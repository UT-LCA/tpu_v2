module axi4_slave #
(
    
    parameter integer P_ADDR_WIDTH   = 16,                                      //Width of Address Bus
    parameter integer P_DATA_WIDTH   = 128                                      //Width of Data Bus
)
(
    output  reg [P_ADDR_WIDTH - 1:0] req_addr,
    output  reg [P_DATA_WIDTH - 1:0] req_data,
    output  reg req_en,
    output  reg     req_type,
    input   [P_DATA_WIDTH - 1:0]     req_read_data,

    input  wire CLOCK,                                                          //Global Clock Signal.
    input  wire RESET,                                                          //Global Reset Signal. This Signal is Active Low

    input wire [6 - 1:0] AWID,                                        //Master Interface Write Address ID
    input wire [P_ADDR_WIDTH - 1:0] AWADDR,                                    //Master Interface Write Address
    input wire [7:0] AWLEN,                                                    //Burst length. The burst length gives the exact number of transfers in a burst
    input wire [2:0] AWSIZE,                                                   //Burst size. This signal indicates the size of each transfer in the burst
    input wire [1:0] AWBURST,                                                  //Burst type. The burst type and the size information, determine how the address for each transfer within the burst is calculated.
    input wire AWLOCK,                                                         //Lock type. Provides additional information about the atomic characteristics of the transfer.
    input wire [3:0] AWCACHE,                                                  //Memory type. This signal indicates how transactions are required to progress through a system.
    input wire [2:0] AWPROT,                                                   //Protection type. This signal indicates the privilege and security level of the transaction, and whether  the transaction is a data access or an instruction access.
    input wire [3:0] AWQOS,                                                    //Quality of Service, QoS identifier sent for each write transaction.
    input wire AWVALID,                                                        //Write address valid. This signal indicates that the channel is signaling valid write address and control information.
    output  wire AWREADY,                                                        //Write address ready. This signal indicates that the slave is ready to accept an address and associated control signals
    input wire [P_DATA_WIDTH-1 : 0] WDATA,                                     //Master Interface Write Data.
    input wire [P_DATA_WIDTH/8-1 : 0] WSTRB,                                   //Write strobes. This signal indicates which byte lanes hold valid data. There is one write strobe bit for each eight bits of the write data bus.
    input wire WLAST,                                                          //Write last. This signal indicates the last transfer in a write burst.
    input wire WVALID,                                                         //Write valid. This signal indicates that valid write data and strobes are available
    output  wire WREADY,                                                         //Write ready. This signal indicates that the slave can accept the write data.

    input wire [6-1 : 0] ARID,                                        //Master Interface Read Address.
    input wire [P_ADDR_WIDTH-1 : 0] ARADDR,                                    //Read address. This signal indicates the initial address of a read burst transaction.
    input wire [7 : 0] ARLEN,                                                  //Burst length. The burst length gives the exact number of transfers in a burst
    input wire [2 : 0] ARSIZE,                                                 //Burst size. This signal indicates the size of each transfer in the burst
    input wire [1 : 0] ARBURST,                                                //Burst type. The burst type and the size information, determine how the address for each transfer within the burst is calculated.
    input wire ARLOCK,                                                         //Lock type. Provides additional information about the atomic characteristics of the transfer.
    input wire [3 : 0] ARCACHE,                                                //Memory type. This signal indicates how transactions are required to progress through a system.
    input wire [2 : 0] ARPROT,                                                 //Protection type. This signal indicates the privilege and security level of the transaction, and whether the transaction is a data access or an instruction access.
    input wire [3 : 0] ARQOS,                                                  //Quality of Service, QoS identifier sent for each read transaction
    input wire ARVALID,                                                        //Write address valid. This signal indicates that the channel is signaling valid read address and control information
    output  wire ARREADY,                                                        //Read address ready. This signal indicates that the slave is ready to accept an address and associated control signals
    output  wire [6-1 : 0] RID,                                         //Read ID tag. This signal is the identification tag for the read data group of signals generated by the slave.
    output  wire [P_DATA_WIDTH-1 : 0] RDATA,                                     //Master Read Data
    output  wire [1 : 0] RRESP,                                                  //Read response. This signal indicates the status of the read transfer
    output  wire RLAST,                                                          //Read last. This signal indicates the last transfer in a read burst
    output  wire RVALID,                                                         //Read valid. This signal indicates that the channel is signaling the required read data.
    input wire RREADY,                                                         //Read ready. This signal indicates that the master can accept the read data and response information.

    input wire BREADY,                                                         //Response ready. This signal indicates that the master can accept a write response.
    output  wire BVALID,                                                         //Write response valid. This signal indicates that the channel is signaling a valid write response.
    output  wire [1 : 0] BRESP,                                                  //Write response. This signal indicates the status of the write transaction.
    output  wire [6-1 : 0] BID                                          //Master Interface Write Response.
);

localparam integer C_MASTER_LENGTH = 12;

reg [2:0] slave_state;
localparam [2:0] IDLE = 0;
localparam [2:0] RDREQ_STATE  = 1;
localparam [2:0] WRREQ_STATE  = 2;
localparam [2:0] WRDATA_STATE = 3;
localparam [2:0] RDATA_STATE = 4;


//AXI4 internal tmp signals
reg [6 - 1:0]   s_rid = 0;
reg [P_DATA_WIDTH - 1:0] s_rdata;
reg s_rlast;
reg s_rready;

reg s_bvalid;                                                         //Write response valid. This signal indicates that the channel is signaling a valid write response.
                                           //Master Interface Write Response.
reg [31:0] m_writer_counter = 0;
    
output  wire ARREADY,                                                        //Read address ready. This signal indicates that the slave is ready to accept an address and associated control signals

assign RID = 0;                                         //Read ID tag. This signal is the identification tag for the read data group of signals generated by the slave.
assign RDATA = s_rdata;                                     //Master Read Data
assign RRESP = 0;                                                  //Read response. This signal indicates the status of the read transfer
assign RLAST = s_rlast;                                                          //Read last. This signal indicates the last transfer in a read burst
assign RREADY = s_rready;                                                        //Read ready. This signal indicates that the master can accept the read data and response information.

assign BID = 0;
assign BRESP = 0;
assign BVALID = s_bvalid;

//Read and Read Response (R)
assign RREADY = m_rready;

//Write Address Channel
//Write Data Generator.Data pattern is only a simple incrementing count from 0 for each burst.
always @(posedge CLOCK) begin
    if (RESET == 0) begin
        s_rdata <= 0;
    end else if (RDREQ_STATE) begin
        s_rdata <= req_read_data;
    end else if (RREADY && s_rvalid) begin
        s_wdata <= 0;
    end else begin 
       s_rdata <= s_rdata;
    end
end

//Write Response (B) Channel

//Need this to silent VIP warning
always @(posedge CLOCK) begin
    if (RESET == 0) begin
        s_bvalid <= 0;
    else if(WRDATA_STATE & WVALID)begin
        s_bvalid <= 1'b1;
    end
    end else begin
        s_bvalid <= 0;
    end
end

always @(posedge CLOCK) begin
    if (RESET == 0) begin
        s_rlast <= 0;
        s_rready <= 0;
    else if(RDREQ_STATE & ARVALID)begin
        s_rlast <= 1'b1;
        s_rready <= 1;
    end
    else if(RVALID && s_rready)begin
        s_rlast<= 1'b0;
        s_rready <= 1'b0;
    end
    end else begin
        s_rlast <= s_rlast;
        s_rready <= s_rready;
    end
end


always@(posedge clk)begin
  if(!RESET)begin
      req_addr <= 0;
      req_data <= 0;
      req_type <= 0;  
  end
  else begin
    req_type <= 0;
    if(slave_state == IDLE & AWVALID )begin
       req_addr <= AWADDR;
       req_type <= 1'b0;
    end
    else if(slave_state == IDLE & ARVALID)begin
       req_addr <= ARADDR;
       req_type <= 1'b1;
    end
    if((slave_state = WRREQ_STATE) & WVALID)
       req_data <= WDATA;
end


always@(*)begin
  if(slave_state =+ RDREQ_STATE || slave_state == WRDATA_STATE)
	req_en = 1'b1
  else
        req_en = 1'b0;
end

assign axi_busy = (slave_state == IDLE)? 1'b1:1'b0;

//implement master command interface state machine
always @ (posedge CLOCK) begin
    if (RESET == 0) begin
        slave_state <= IDLE;
    end else begin
        case (slave_state)
            IDLE: begin
                if (AWVALID) begin
                    slave_state <= WRREQ_STATE;
                end
                else if(ARVALID) begin
                    slave_state <= RDREQ_STATE;
                end
            end
            WRREQ_STATE: begin
                if (WVALID)
                    slave_state <= WRDATA_STATE;
                else 
                    slave_state <= WRREQ_STATE;
            end
            WRDATA_STATE: begin
                if (WREADY & WVALID) begin
                    slave_state <= IDLE;
                end
            end
            RDREQ_STATE: begin
                if (RVALID)
                    slave_state <= RDATA_STATE;
                else 
                    slave_state <= RDREQ_STATE;
            end
            RDATA_STATE: begin
                if (RREADY & RREADY) begin
                    slave_state <= IDLE;
                end
            end
            default: begin
                slave_state <= IDLE;
            end
        endcase
    end
end


endmodule
