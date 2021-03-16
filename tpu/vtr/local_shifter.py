import parser

class local_shifter():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, lpm_width, lpm_widthdist, lpm_shifttype):
        string = '''
module local_shifter_{LPM_WIDTH}_{LPM_WIDTHDIST}_{LPM_SHIFTTYPE}(
  data,
  distance,
  direction,
  result
);

parameter LPM_SHIFTTYPE = "{LPM_SHIFTTYPE}";

input [{LPM_WIDTH}-1:0] data;
input [{LPM_WIDTHDIST}-1:0] distance;
input direction;

output reg [{LPM_WIDTH}-1:0] result;
reg [{LPM_WIDTH}-1:0] arith_reg; 
always @* begin
  arith_reg = {{{LPM_WIDTH}{{1'b1}}}};
    if (LPM_SHIFTTYPE == "ARITHMETIC") begin
    if(direction)begin
      result = data << distance;
    end
    else begin
      if(data[{LPM_WIDTH}-1] == 1'b1)
          result =  ((arith_reg <<({LPM_WIDTH} - distance))|| (data >> distance));
      else
          result = data >> distance;
    end
  end
  else begin
    if(direction == 1'b0)begin
        result = data << distance;
    end
    else begin
        result = data >> distance;
    end
  end 
end
endmodule
'''

        return string.format(LPM_WIDTH=lpm_width, LPM_WIDTHDIST=lpm_widthdist, LPM_SHIFTTYPE=lpm_shifttype)

    def write(self, lpm_width, lpm_widthdist, lpm_shifttype):
        self.fp.write(self.make_str(lpm_width, lpm_widthdist, lpm_shifttype))


if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    shifter = local_shifter(fp)
    shifter.write(32, 5, "ARITHMETIC")
    fp.close()