module transpose #(
  parameter WIDTH = 16,
  parameter NUMSTAGES = 8,
  parameter LOGNUMSTAGE = $clog2(NUMSTAGES)
)(
  input clk,
  input resetn,
  input en, 
  input [NUMSTAGES * WIDTH -1:0] a,
  output reg [NUMSTAGES * WIDTH -1:0] out,
  output reg busy,
  output reg valid
);

reg [NUMSTAGES * WIDTH - 1:0] data0;
reg [NUMSTAGES * WIDTH - 1:0] data1;
reg [NUMSTAGES * WIDTH - 1:0] data2;
reg [NUMSTAGES * WIDTH - 1:0] data3;
reg [NUMSTAGES * WIDTH - 1:0] data4;
reg [NUMSTAGES * WIDTH - 1:0] data5;
reg [NUMSTAGES * WIDTH - 1:0] data6;
reg [NUMSTAGES * WIDTH - 1:0] data7;
reg [LOGNUMSTAGE:0] count;

always@(posedge clk)begin
  if(!resetn)begin
    count <= 'h0;
    valid <= 'h0;
  end
  else begin
    if(en)begin
      count <= count + 1;
      valid <= 0;
    end
    else if(count != 0)begin
      valid <= 1'b1;
      count <= count - 1;
    end
    else begin
      count <= 0;
      valid <= 0;
    end
  end
end

always@(posedge clk)begin
  if(!resetn)
     busy <= 1'b0;
  else
    if((count != 0) || (en == 1'b1))
       busy <= 1'b1;
    else
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
      case(count)
      3'h0:begin
               data0[0*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[0*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[0*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[0*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[0*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[0*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[0*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[0*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      3'h1:begin
               data0[1*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[1*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[1*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[1*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[1*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[1*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[1*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[1*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      3'h2:begin
               data0[2*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[2*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[2*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[2*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[2*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[2*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[2*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[2*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      3'h3:begin
               data0[3*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[3*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[3*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[3*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[3*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[3*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[3*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[3*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      3'h4:begin
               data0[4*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[4*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[4*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[4*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[4*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[4*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[4*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[4*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      3'h5:begin
               data0[5*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[5*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[5*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[5*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[5*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[5*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[5*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[5*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      3'h6:begin
               data0[6*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[6*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[6*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[6*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[6*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[6*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[6*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[6*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      3'h7:begin
               data0[7*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[7*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[7*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[7*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[7*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[7*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[7*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[7*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      endcase
    end
    else if(valid)begin
        data0 <= data1;
        data1 <= data2;
        data2 <= data3;
        data3 <= data4;
        data4 <= data5;
        data5 <= data6;
        data6 <= data7;
        data7 <= 0;
    end
    
  end  
end

endmodule
