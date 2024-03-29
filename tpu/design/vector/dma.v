module dma #(parameter NUMLANES=8,
  parameter WIDTH = 16,
  parameter ADDRWIDTH = 8,
  parameter DMEM_WIDTH = NUMLANES * WIDTH,
  parameter DMEM_ADDRWIDTH = 32,
  parameter LOG2DMEMWIDTH= $clog2(DMEM_WIDTH)
)(

input clk,
input resetn,

input [DMEM_ADDRWIDTH-1:0] mem_addr,
input [15:0] num_bytes,
input dma_en,
input [ADDRWIDTH-1:0] lane_addr,
input we, // we = 1 means data from lane_addr will be transfered to main memory

output reg [NUMLANES*ADDRWIDTH-1:0] local_addr,
output reg [NUMLANES-1:0] local_wren,
output reg [NUMLANES-1:0] local_rden,
output reg [NUMLANES*WIDTH-1:0] local_wrdata,
 input     [NUMLANES*WIDTH-1:0] local_rddata,

output     dma_busy,
output reg [DMEM_ADDRWIDTH-1:0]  dbus_address,
 input  [DMEM_WIDTH-1:0]  dbus_readdata,
output  [DMEM_WIDTH-1:0]   dbus_writedata,
output reg [LOG2DMEMWIDTH-3-1:0]   dbus_byteen,
output reg   dbus_en,
output reg   dbus_wren,
output reg   dbus_prefetch,
 input    dbus_wait,   //Goes high 1-cycle after dbus_en
 input    dbus_data_valid
);

parameter IDLE_STATE = 3'b000;
parameter LOAD_REQ_STATE = 3'b001;
parameter STORE_REQ_STATE = 3'b010;
parameter READ_STATE = 3'b011;
parameter LOAD_DATA_STATE = 3'b100;
parameter WRITE_STATE = 3'b101;
parameter UPDATE_RD_COUNT = 3'b110;
parameter UPDATE_WR_COUNT = 3'b111;

reg [15:0] count, next_count;
reg [2:0] pstate,nstate;
reg [NUMLANES*WIDTH-1:0] data_to_dbus, data_to_mem;
reg[31:0]i,j,k;
always@(posedge clk)begin
  if(!resetn)begin
    data_to_dbus <= 0;
    data_to_mem <= 0;
  end
  else begin 
    if(pstate == STORE_REQ_STATE)
      data_to_dbus <= local_rddata;
    if(pstate == LOAD_DATA_STATE)
      for(j=0;j<DMEM_WIDTH/32;j=j+1)
        for(k=0;k<2;k=k+1)
          data_to_mem[(1-k)*16+j*32+:16] <= dbus_readdata[k*16+j*32+:16]; 
  end
end

assign dbus_writedata = (pstate == STORE_REQ_STATE)? local_rddata : data_to_dbus;

always@(posedge clk )begin
  if(!resetn)begin
    count <= 0;
    pstate <= IDLE_STATE;
  end 
  else begin
    count <= next_count;
    pstate <= nstate;
  end 
end

assign dma_busy = (pstate == IDLE_STATE)? 1'b0:1'b1; 

always@(*)begin
  case(pstate)
    IDLE_STATE:begin
                 if((!we) & dma_en )        
                   nstate  = LOAD_REQ_STATE;
                 else  if(we & dma_en)
                   nstate = STORE_REQ_STATE;
                 else
                   nstate = IDLE_STATE;
               end
    LOAD_REQ_STATE:begin
                     if(count != num_bytes)
                       nstate = READ_STATE;
                     else
                       nstate = IDLE_STATE;
                   end
    STORE_REQ_STATE:begin
                     if(count != num_bytes)
                       nstate = WRITE_STATE;
                     else
                       nstate = IDLE_STATE;
                   end
    READ_STATE:begin
                 if(!dbus_wait)
                   nstate = LOAD_DATA_STATE;
                 else
                   nstate = READ_STATE;
               end
    LOAD_DATA_STATE:begin
                        nstate = UPDATE_RD_COUNT;
                    end
    WRITE_STATE:begin
                 if(dbus_wait)
                   nstate = WRITE_STATE;
                 else
                   nstate = UPDATE_WR_COUNT;
               end
    UPDATE_WR_COUNT:begin
                        nstate = STORE_REQ_STATE;
                    end
    UPDATE_RD_COUNT:begin
                        nstate = LOAD_REQ_STATE;
                    end
  endcase
end

always@(*)begin
  dbus_prefetch = 0;
  case(pstate)
    IDLE_STATE: begin
                   dbus_address =0;
                   dbus_byteen = 0;
                   dbus_en = 0; 
                   dbus_wren = 0;
                   next_count = 0;
                   local_wren = 0;
                   local_rden = 0;
                end 
    LOAD_REQ_STATE:begin
                     dbus_address = mem_addr + count;
                     dbus_byteen = 0;
                     dbus_en = 1; 
                     dbus_wren = 0;
                     next_count = count;
                   end
    STORE_REQ_STATE:begin
                     for(i=0; i<NUMLANES;i=i+1)begin
                       local_addr[i*ADDRWIDTH +: ADDRWIDTH] =  lane_addr  + count + i;
                       local_wren[i] = 1'b0;
                       local_rden[i] = 1'b1;
                     end
                     dbus_address = mem_addr + count;
                     dbus_byteen = 16'hffff;
                     dbus_en = 1; 
                     dbus_wren = 1;
                     next_count = count ;
                   end
    READ_STATE:begin
                     dbus_byteen = 0;
                     dbus_en = 0; 
                     dbus_wren = 0;
                     next_count = count;
               end
    LOAD_DATA_STATE:begin
                     dbus_byteen = 0;
                     dbus_en = 0; 
                     dbus_wren = 0;
                     next_count = count;
                    end
    WRITE_STATE:begin
                       dbus_address = mem_addr + count;
                       dbus_byteen = {NUMLANES{2'b11}};
                       dbus_en = 0; 
                       dbus_wren = 0;
                       next_count = count;
               end
    UPDATE_WR_COUNT:begin
                       dbus_address = mem_addr + count;
                       dbus_byteen = {NUMLANES{2'b11}};
                       dbus_en = 0; 
                       dbus_wren = 0;
                       next_count = count;
                       next_count = count + (NUMLANES*WIDTH/8);
                     end
    UPDATE_RD_COUNT: begin
                       for(i=0; i<NUMLANES;i=i+1)begin
                         local_addr[i*ADDRWIDTH +: ADDRWIDTH] =  lane_addr  + count + i;
                         local_wren[i] = 1'b1;
                         local_rden[i] = 1'b0;
                       end
                       local_wrdata = data_to_mem; 
                       dbus_address = mem_addr + count;
                       dbus_byteen = {NUMLANES{2'b00}};
                       dbus_en = 0; 
                       dbus_wren = 0;
                       next_count = count + (NUMLANES*WIDTH/8);
                end
  endcase
end

endmodule
