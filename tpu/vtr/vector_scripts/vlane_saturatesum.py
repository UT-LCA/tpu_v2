import parser

class vlane_saturatesum():
  def __init__(self, fp):
    self.fp = fp

  def make_str(self, width):
    string = '''\
/****************************************************************************
          Saturate unit

  Takes a signed input and saturates it

  sat sign
   0    x   NOP (pass through)
   1    1   VS[ADD/SUB]
   1    0   VS[ADD/SUB]_U

parameter
  SATSUMOP_NOP=2'b00,
  SATSUMOP_VS =2'b11,
  SATSUMOP_VSU=2'b10;

****************************************************************************/


module vlane_saturatesum_{WIDTH}(
    in,
    op,
    out
    );

parameter WIDTH={WIDTH};

input [WIDTH+2-1:0] in;
input [1:0] op;
output [WIDTH-1:0] out;

reg [WIDTH-1:0] out;

wire op_saturate;
wire op_signed;

assign op_saturate=op[1];
assign op_signed=op[0];

wire [WIDTH-1:0] maxunsigned;
wire [WIDTH-1:0] minunsigned;
wire [WIDTH-1:0] maxsigned;
wire [WIDTH-1:0] minsigned;

assign maxunsigned = {{WIDTH{{1'b1}}}};
assign minunsigned = 0;
assign maxsigned = {{1'b0,{{WIDTH-1{{1'b1}}}}}};
assign minsigned = {{1'b1,{{WIDTH-1{{1'b0}}}}}};

wire [WIDTH-1:0] through;

assign through=in[WIDTH-1:0];

wire [2:0] top3bits=(op_saturate) ? in[WIDTH+2-1:WIDTH-1] : 3'b0 ;

  always@*
    case(top3bits)
      3'b010: out=maxunsigned;
      3'b011: out=maxunsigned;
      3'b001: out=(op_signed) ? maxsigned : through;
      3'b111: out=(op_signed) ? through : minunsigned;
      3'b110: out=(op_signed) ? minsigned : minunsigned;
      default: out=through;
    endcase

endmodule'''

    return string.format(WIDTH=width)

  def write (self, width):
    self.fp.write(self.make_str(width))


if __name__ == '__main__':
  fp = open(parser.parse(), "w")
  uut = vlane_saturatesum(fp)
  uut.write(32)
  fp.close()
