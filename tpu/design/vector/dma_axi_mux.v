module dma_axi_mux #(
  parameter ADDRWIDTH = 11,
  parameter NUMLANES = 8,
  parameter WIDTH = 16
)(
  input [NUMLANES*ADDRWIDTH-1:0]   dma_addr,
  input [NUMLANES*WIDTH-1:0]       dma_data,
  input [NUMLANES-1:0]             dma_rden,
  input  [NUMLANES-1:0]            dma_wren,    
  
  output reg [NUMLANES*WIDTH-1:0]  dma_out,   
 
  input [NUMLANES*ADDRWIDTH-1:0]   axi_addr,   
  input [NUMLANES*WIDTH-1:0]       axi_data,    
  input  [NUMLANES-1:0]            axi_req_en,  
  input  [NUMLANES-1:0]            axi_req_type,
  
  output reg [NUMLANES*WIDTH-1:0]  axi_read_data,
  
  output reg [NUMLANES*ADDRWIDTH-1:0]       mem_addr,    
  output reg [NUMLANES*WIDTH-1:0]  mem_data,     
  output reg [NUMLANES-1:0]        mem_rden,     
  output reg [NUMLANES-1:0]        mem_wren,     
  output reg [NUMLANES*WIDTH-1:0]  mem_readdata    
);

reg [NUMLANES-1:0] axi_rden, axi_wren;

always@(*)begin
  axi_rden = 0;
  axi_wren = 0;
  if(axi_req_en)begin
    if(axi_req_type)begin
      axi_wren = {NUMLANES{1'b1}};
    end
    else begin
      axi_rden = {NUMLANES{1'b1}};
    end
  end
end

always@(*)begin
   if(axi_req_en)begin
     mem_addr = axi_addr;
     mem_data = axi_data;
     mem_rden = axi_rden;
     mem_wren = axi_wren;
     dma_out  = mem_readdata;
     axi_read_data = 0;
   end
   else begin
     mem_addr = dma_addr; 
     mem_data = dma_data;
     mem_rden = dma_rden;
     mem_wren = dma_wren;
     dma_out  = mem_readdata;
     axi_read_data = 0;
   end
end
endmodule
