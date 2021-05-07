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
input [7:0] num_bytes,
input dma_en,
input [ADDRWIDTH-1:0] lane_addr,
input we,

output reg [NUMLANES*ADDRWIDTH-1:0] local_addr,
output reg [NUMLANES-1:0] local_wren,
output reg [NUMLANES*WIDTH-1:0] local_wrdata,
 input     [NUMLANES*WIDTH-1:0] local_rddata,

output    dma_busy,
output [DMEM_ADDRWIDTH-1:0]  dbus_address,
 input [DMEM_WIDTH-1:0]  dbus_readdata,
output [DMEM_WIDTH-1:0]   dbus_writedata,
output [LOG2DMEMWIDTH-3-1:0]   dbus_byteen,
output    dbus_en,
output    dbus_wren,
output    dbus_prefetch,
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
parameter UPDATE_WR_COUNT = 3'b110;

reg [4:0] count, next_count;
reg [1:0] pstate,nstate;
reg [NUMLANES*ADDRWIDTH-1:0] data_to_dbus, data_to_mem;

always@(posedge clk)begin
  if(!reset)begin
    data_to_dbus <= 0;
    data_to_mem <= 0;
  end
  else begin 
    if(pstate == STORE_REQ_STATE)
      data_to_dbus <= local_rddata;
    if((pstate == LOAD_DATA_STATE) && dbus_data_valid )
      data_to_mem <= dbus_readdata; 
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
  case(pstate)begin
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
                 if(dbus_wait & ~dbus_data_valid)
                   nstate = READ_STATE;
                 else
                   nstate = LOAD_DATA_STATE;
               end
    LOAD_DATA_STATE:begin
                        nstate = LOAD_REQ_STATE;
                    end
    WRITE_STATE:begin
                 if(dbus_wait)
                   nstate = WRITE_STATE;
                 else
                   nstate = STORE_REQ_STATE;
               end
    UPDATE_COUNT:begin
                        nstate = WRITE_STATE;
                    end
  endcase
end

always@(*)begin
  case(pstate)begin
    dbus_prefetch = 0;
    IDLE_STATE: begin
                   dbus_address =0;
                   dbus_byteen = 0;
                   dbus_en = 0; 
                   dbus_wren = 0;
                   next_count = 0;
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
                     end
                     dbus_address = mem_addr + count;
                     dbus_byteen = 16'hffff;
                     dbus_en = 1; 
                     dbus_wren = 1;
                     next_count = count ;
                   end
    READ_STATE:begin
                     dbus_address = mem_addr + count;
                     dbus_byteen = 0;
                     dbus_en = 0; 
                     dbus_wren = 0;
                     next_count = count;
               end
    LOAD_DATA_STATE:begin
                       for(i=0; i<NUMLANES;i=i+1)begin
                         local_addr[i*ADDRWIDTH +: ADDRWIDTH] =  lane_addr  + count + i;
                         local_wren[i] = 1'b1;
                         local_wrdata[i*WIDTH+:WIDTH] = data_to_mem; 
                       end
                    end
    WRITE_STATE:begin
                       dbus_address = mem_addr + count;
                       dbus_byteen = {NUMLANES{2'b11}};
                       dbus_en = 0; 
                       dbus_wren = 0;
                       next_count = count;
               end
    UPDATE_WR_STATE:begin
                       dbus_address = mem_addr + count;
                       dbus_byteen = {NUMLANES{2'b11}};
                       dbus_en = 0; 
                       dbus_wren = 0;
                       next_count = count;
                       next_count = count + (NUMLANES*WIDTH/8);
                     end
    UPDATE_RD_COUNT: begin
                       dbus_address = mem_addr + count;
                       dbus_byteen = {NUMLANES{2'b00}};
                       dbus_en = 0; 
                       dbus_wren = 0;
                       next_count = count + (NUMLANES*WIDTH/8);
                end
  endcase
end

endmodule
