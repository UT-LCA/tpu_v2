module fifo#(
parameter FIFO_WIDTH = 8,
parameter FIFO_DEPTH = 4
         )(
input                      clk,
input                      reset,
input   [FIFO_WIDTH-1:0]   wrdata,
input                      write,

input                       read,
output reg [FIFO_WIDTH-1:0] rddata,
output reg                  full,
output reg                  empty 
);

localparam PTR_WIDTH = $clog2(FIFO_DEPTH);

reg  [FIFO_WIDTH-1:0] mem [FIFO_DEPTH-1:0];
reg  [PTR_WIDTH:0]    wrptr,rdptr;
wire [PTR_WIDTH:0]    wrptr_nxt,rdptr_nxt;
integer i;

assign  wrptr_nxt = write? wrptr + 1'b1: wrptr;
assign  rdptr_nxt = read? rdptr + 1'b1:rdptr ;

always@(*) begin
    rddata = mem[rdptr[PTR_WIDTH-1:0]];
end

always@(posedge clk) begin
  if(!reset)begin
    full <= 1'b0;
  end
  else begin
    if(wrptr_nxt[PTR_WIDTH]^ rdptr_nxt[PTR_WIDTH])
  	full <= ~(|(wrptr_nxt[PTR_WIDTH-1:0]^rdptr_nxt[PTR_WIDTH-1:0]));
    else
	full <= 1'b0;
  end 
end

always@(posedge clk) begin
  if(!reset)begin
    empty <= 1'b1;
  end
  else begin
    empty <= ~(|(wrptr_nxt^rdptr_nxt));
  end 
end

always @(posedge clk) begin
 if(!reset)begin
   wrptr <= 'h0;
 end
 else begin
     wrptr <= wrptr_nxt;
 end
end

always@(posedge clk) begin
 if(!reset)begin
   rdptr <= 'h0;
 end
 else begin
     rdptr <= rdptr_nxt;
 end
end

always@(posedge clk) begin
 if(!reset)begin
   for(i = 0; i<FIFO_DEPTH;i=i+1)begin
       mem[i] <= 'h0;
   end
 end
 else begin
   if(write)begin
      mem[wrptr[PTR_WIDTH-1:0]]<= wrdata;
   end
 end
end

endmodule

