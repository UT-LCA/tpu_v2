module transpose_10_2_1 (
  input clk,
  input resetn,
  input read,
  input en, 
  input [2 * 10 -1:0] a,
  output reg [2 * 10 -1:0] out,
  output reg busy
);

reg [2 * 10 - 1:0] data0;
reg [2 * 10 - 1:0] data1;
reg [2 * 10 - 1:0] data2;
reg [2 * 10 - 1:0] data3;
reg [2 * 10 - 1:0] data4;
reg [2 * 10 - 1:0] data5;
reg [2 * 10 - 1:0] data6;
reg [2 * 10 - 1:0] data7;
reg [1:0] count;
reg[31:0] i;
reg[31:0] j;
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
          if(count==0)begin
              data0[0*10 +: 10] <= a[0*10 +: 10];
              data1[0*10 +: 10] <= a[1*10 +: 10];
              data2[0*10 +: 10] <= a[2*10 +: 10];
              data3[0*10 +: 10] <= a[3*10 +: 10];
              data4[0*10 +: 10] <= a[4*10 +: 10];
              data5[0*10 +: 10] <= a[5*10 +: 10];
              data6[0*10 +: 10] <= a[6*10 +: 10];
              data7[0*10 +: 10] <= a[7*10 +: 10];
          end   
          else if(count==1)begin
              data0[1*10 +: 10] <= a[0*10 +: 10];
              data1[1*10 +: 10] <= a[1*10 +: 10];
              data2[1*10 +: 10] <= a[2*10 +: 10];
              data3[1*10 +: 10] <= a[3*10 +: 10];
              data4[1*10 +: 10] <= a[4*10 +: 10];
              data5[1*10 +: 10] <= a[5*10 +: 10];
              data6[1*10 +: 10] <= a[6*10 +: 10];
              data7[1*10 +: 10] <= a[7*10 +: 10];
          end   
          else if(count==2)begin
              data0[2*10 +: 10] <= a[0*10 +: 10];
              data1[2*10 +: 10] <= a[1*10 +: 10];
              data2[2*10 +: 10] <= a[2*10 +: 10];
              data3[2*10 +: 10] <= a[3*10 +: 10];
              data4[2*10 +: 10] <= a[4*10 +: 10];
              data5[2*10 +: 10] <= a[5*10 +: 10];
              data6[2*10 +: 10] <= a[6*10 +: 10];
              data7[2*10 +: 10] <= a[7*10 +: 10];
          end   
          else if(count==3)begin
              data0[3*10 +: 10] <= a[0*10 +: 10];
              data1[3*10 +: 10] <= a[1*10 +: 10];
              data2[3*10 +: 10] <= a[2*10 +: 10];
              data3[3*10 +: 10] <= a[3*10 +: 10];
              data4[3*10 +: 10] <= a[4*10 +: 10];
              data5[3*10 +: 10] <= a[5*10 +: 10];
              data6[3*10 +: 10] <= a[6*10 +: 10];
              data7[3*10 +: 10] <= a[7*10 +: 10];
          end   
          else if(count==4)begin
              data0[4*10 +: 10] <= a[0*10 +: 10];
              data1[4*10 +: 10] <= a[1*10 +: 10];
              data2[4*10 +: 10] <= a[2*10 +: 10];
              data3[4*10 +: 10] <= a[3*10 +: 10];
              data4[4*10 +: 10] <= a[4*10 +: 10];
              data5[4*10 +: 10] <= a[5*10 +: 10];
              data6[4*10 +: 10] <= a[6*10 +: 10];
              data7[4*10 +: 10] <= a[7*10 +: 10];
          end   
          else if(count==5)begin
              data0[5*10 +: 10] <= a[0*10 +: 10];
              data1[5*10 +: 10] <= a[1*10 +: 10];
              data2[5*10 +: 10] <= a[2*10 +: 10];
              data3[5*10 +: 10] <= a[3*10 +: 10];
              data4[5*10 +: 10] <= a[4*10 +: 10];
              data5[5*10 +: 10] <= a[5*10 +: 10];
              data6[5*10 +: 10] <= a[6*10 +: 10];
              data7[5*10 +: 10] <= a[7*10 +: 10];
          end   
          else if(count==6)begin
              data0[6*10 +: 10] <= a[0*10 +: 10];
              data1[6*10 +: 10] <= a[1*10 +: 10];
              data2[6*10 +: 10] <= a[2*10 +: 10];
              data3[6*10 +: 10] <= a[3*10 +: 10];
              data4[6*10 +: 10] <= a[4*10 +: 10];
              data5[6*10 +: 10] <= a[5*10 +: 10];
              data6[6*10 +: 10] <= a[6*10 +: 10];
              data7[6*10 +: 10] <= a[7*10 +: 10];
          end   
          else if(count==7)begin
              data0[7*10 +: 10] <= a[0*10 +: 10];
              data1[7*10 +: 10] <= a[1*10 +: 10];
              data2[7*10 +: 10] <= a[2*10 +: 10];
              data3[7*10 +: 10] <= a[3*10 +: 10];
              data4[7*10 +: 10] <= a[4*10 +: 10];
              data5[7*10 +: 10] <= a[5*10 +: 10];
              data6[7*10 +: 10] <= a[6*10 +: 10];
              data7[7*10 +: 10] <= a[7*10 +: 10];
          end   
    end
    else if(busy & read)begin
        data0 <= data1;
        data1 <= data2;
        data2 <= data3;
        data3 <= data4;
        data4 <= data5;
        data5 <= data6;
        data6 <= data7;
        data7 <= 0;
    end
    else begin
        data0 <= data0;
        data1 <= data1;
        data2 <= data2;
        data3 <= data3;
        data4 <= data4;
        data5 <= data5;
        data6 <= data6;
        data7 <= data7;
    end
  end  
end

endmodule
