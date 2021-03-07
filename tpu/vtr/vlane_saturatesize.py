import parser

class vlane_saturatesize():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self):
        string = '''\
/****************************************************************************
          Saturate unit for different widths given 32-bit input

  Only works for 32-bit inputs

  Interprets input as signed/unsigned, saturates to signed/unsigned 
  byte/half/word

  NOP (pass through) when size=word and signed==outsigned

   out
   sign sign size
    0    0    00    VSAT_U_W - NOP (pass through)
    0    0    01    VSAT_U_B
    0    0    10    VSAT_U_H
    1    1    00    VSAT_W - NOP (pass through)
    1    1    01    VSAT_B
    1    1    10    VSAT_H
    0    1    00    VSAT_SU_W
    0    1    01    VSAT_SU_B
    0    1    10    VSAT_SU_H

parameter
  SATSIZEOP_VSATUW=4'b0000,
  SATSIZEOP_VSATUB=4'b0001,
  SATSIZEOP_VSATUH=4'b0010,
  SATSIZEOP_VSATW  =4'b1100,
  SATSIZEOP_VSATB  =4'b1101,
  SATSIZEOP_VSATH  =4'b1110,
  SATSIZEOP_VSATSUW=4'b0100,
  SATSIZEOP_VSATSUB=4'b0101,
  SATSIZEOP_VSATSUH=4'b0110;

****************************************************************************/

module vlane_saturatesize(
    in,
    op,
    out
    );

parameter WIDTH=32;

input [WIDTH-1:0] in;
input [3:0] op;
output [WIDTH-1:0] out;

wire op_outsigned;
wire op_signed;
wire op_size;

reg [WIDTH-1:0] out;

  assign op_outsigned=op[3];
  assign op_signed=op[2];
  assign op_size=op[1:0]; //0 - word, 1 - byte, 2 - half

always@*
case(op_size)
    2'b01:  //byte
    case({{op_signed,op_outsigned}})
        2'b11:    // signed
        out = ((in[WIDTH-1])&(!(&in[WIDTH-2:7]))) ? {{{{WIDTH-8{{1'b1}}}},8'h80}} :
                ((~in[WIDTH-1]) && (|in[WIDTH-2:7])) ? 127 : in;
        2'b10:    // signed-unsigned
        out = (in[WIDTH-1]) ? 0 :
                (~in[WIDTH-1]&&(|in[WIDTH-2:8])) ? 255 : in;
        default:  //2'b00: unsigned
        out=(|in[WIDTH-1:8]) ? 255 : in;
        endcase
    2'b10:  //half-word 16-bits
    case({{op_signed,op_outsigned}})
        2'b11:    // signed
        out=((in[WIDTH-1])&(!(&in[WIDTH-2:15])))? {{{{WIDTH-16{{1'b1}}}},16'h8000}}:
                ((~in[WIDTH-1]) && (|in[WIDTH-2:15])) ? 32767 : in;
        2'b10:    // signed-unsigned
        out = (in[WIDTH-1]) ? 0 :
                (~in[WIDTH-1]&&(|in[WIDTH-2:16])) ? 65535 : in;
        default:  //2'b00: unsigned
        out=(|in[WIDTH-1:16]) ? 65535 : in;
        endcase
    default: 
    case({{op_signed,op_outsigned}})
        2'b10:    // signed-unsigned
        out = (in[WIDTH-1]) ? 0 : in;
        default:  //2'b00: unsigned
        out=in;
        endcase
endcase
  
endmodule'''

        return string.format()

    def write (self):
        self.fp.write(self.make_str())

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = vlane_saturatesize(fp)
    uut.write()
    fp.close()