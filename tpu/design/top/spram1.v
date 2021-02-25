module spram1 (
    clk,
    address,
    wren, 
    data,
    byteen,
    out
);

parameter AWIDTH=10;
parameter NUM_WORDS=1024;
parameter DWIDTH=32;
parameter LOG2DWIDTH = $clog2(DWIDTH);
input clk;
input [(AWIDTH-1):0] address;
input  wren;
input [(DWIDTH/8)-1:0] byteen;
input [(DWIDTH-1):0] data;
output reg [(DWIDTH-1):0] out;

integer i;
integer k;

reg [32-1:0] ram[67108864-1:0];
reg [25:0] addr;
 
initial
 begin
   //This is TOO big for 256 MB RAM!  We right shift data by 1
   $readmemh("instr.dat",ram,'h100_0000);
   $readmemh("data.dat",ram,'h400_0000>>1);
 end

always@(*) begin
    addr = address << 26-AWIDTH;
end

always@(posedge clk) begin 
  if (wren) begin
      for(k=0; k < DWIDTH/32;k=k+1)begin
          for(i=0; i < 4 ;i=i+1)begin
              if(byteen[((DWIDTH/8-1)-(4*k+i))])
                  ram[addr+k][i*8+:8] <= data[32*k+i*8+:8];
          end
      end
  end
  else begin
      for(i=0; i < DWIDTH/32; i=i+1)begin
          out[32*i+:32] <= ram[addr+i];
      end
  end
end

endmodule
