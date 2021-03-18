module transpose_32_2_1 (
  input clk,
  input resetn,
  input read,
  input en, 
  input [2 * 32 -1:0] a,
  output reg [2 * 32 -1:0] out,
  output reg busy
);

reg [2 * 32 - 1:0] data0;
reg [2 * 32 - 1:0] data1;
reg [2 * 32 - 1:0] data2;
reg [2 * 32 - 1:0] data3;
reg [2 * 32 - 1:0] data4;
reg [2 * 32 - 1:0] data5;
reg [2 * 32 - 1:0] data6;
reg [2 * 32 - 1:0] data7;
reg [1:0] count;
integer i;

always@(posedge clk)begin
  if(!resetn)
    count <= 'h0;
  else begin
    if(en)
      count <= count + 1;
    else if(read)
      count <= count - 1 ;
  end
end

always@(posedge clk)begin
  if(!resetn)
     busy <= 1'b0;
  else
    if((count == 2-1) && en)
       busy <= 1'b1;
    else if((count == 'h1) && read)
       busy <= 1'b0; 
end

always @(posedge clk)begin
  if(!resetn)begin
    data0 <= 'h0;
    data1 <= 'h0;
    data2 <= 'h0;
    data3 <= 'h0;
    data4 <= 'h0;
    data5 <= 'h0;
    data6 <= 'h0;
    data7 <= 'h0;
    out <= 'h0;
  end
  else begin
    out <= data0;
    if(en)begin
      data0[count*32 +: 32] = a[0*32 +: 32];
      data1[count*32 +: 32] = a[1*32 +: 32];
      data2[count*32 +: 32] = a[2*32 +: 32];
      data3[count*32 +: 32] = a[3*32 +: 32];
      data4[count*32 +: 32] = a[4*32 +: 32];
      data5[count*32 +: 32] = a[5*32 +: 32];
      data6[count*32 +: 32] = a[6*32 +: 32];
      data7[count*32 +: 32] = a[7*32 +: 32];
    end
    else if(busy & read)begin
        data0 <= data1;
        data1 <= data2;
        data2 <= data3;
        data3 <= data4;
        data4 <= data5;
        data5 <= data6;
        data6 <= data7;
    end
    
  end  
end

endmodule
