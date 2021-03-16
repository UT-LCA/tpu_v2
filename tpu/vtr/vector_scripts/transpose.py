from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class transpose():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width,numstages,lognumstages):
        string = '''\
module transpose_{WIDTH}_{NUMSTAGES}_{LOGNUMSTAGES} (
  input clk,
  input resetn,
  input read,
  input en, 
  input [{NUMSTAGES} * {WIDTH} -1:0] a,
  output reg [{NUMSTAGES} * {WIDTH} -1:0] out,
  output reg busy
);

reg [{NUMSTAGES} * {WIDTH} - 1:0] data0;
reg [{NUMSTAGES} * {WIDTH} - 1:0] data1;
reg [{NUMSTAGES} * {WIDTH} - 1:0] data2;
reg [{NUMSTAGES} * {WIDTH} - 1:0] data3;
reg [{NUMSTAGES} * {WIDTH} - 1:0] data4;
reg [{NUMSTAGES} * {WIDTH} - 1:0] data5;
reg [{NUMSTAGES} * {WIDTH} - 1:0] data6;
reg [{NUMSTAGES} * {WIDTH} - 1:0] data7;
reg [{LOGNUMSTAGES}:0] count;
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
    if((count == {NUMSTAGES}-1) && en)
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
      data0[count*{WIDTH} +: {WIDTH}] = a[0*{WIDTH} +: {WIDTH}];
      data1[count*{WIDTH} +: {WIDTH}] = a[1*{WIDTH} +: {WIDTH}];
      data2[count*{WIDTH} +: {WIDTH}] = a[2*{WIDTH} +: {WIDTH}];
      data3[count*{WIDTH} +: {WIDTH}] = a[3*{WIDTH} +: {WIDTH}];
      data4[count*{WIDTH} +: {WIDTH}] = a[4*{WIDTH} +: {WIDTH}];
      data5[count*{WIDTH} +: {WIDTH}] = a[5*{WIDTH} +: {WIDTH}];
      data6[count*{WIDTH} +: {WIDTH}] = a[6*{WIDTH} +: {WIDTH}];
      data7[count*{WIDTH} +: {WIDTH}] = a[7*{WIDTH} +: {WIDTH}];
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
'''

        return string.format(WIDTH=width,NUMSTAGES=numstages,LOGNUMSTAGES=lognumstages) 

    def write (self, width,numstages,lognumstages):
        self.fp.write(self.make_str(width,numstages,lognumstages))

if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = transpose(fp)
    uut1.write(32,2,1)
    fp.close()
