import parser

class logic_unit():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''
/****************************************************************************
          logic unit
- note ALU must be able to increment PC for JAL type instructions

Operation Table
  op
  0     AND
  1     OR
  2     XOR
  3     NOR
****************************************************************************/
module logic_unit_{WIDTH} (
            opA, opB,
            op,
            result);

input [{WIDTH}-1:0] opA;
input [{WIDTH}-1:0] opB;
input [2-1:0] op;
output [{WIDTH}-1:0] result;

reg [{WIDTH}-1:0] logic_result;

always@(opA or opB or op )
    case(op)
        2'b00:
            logic_result=opA&opB;
        2'b01:
            logic_result=opA|opB;
        2'b10:
            logic_result=opA^opB;
        2'b11:
            logic_result=~(opA|opB);
    endcase

assign result=logic_result;


endmodule
'''       

        return string.format(WIDTH=width)

    def write (self, width):
        self.fp.write(self.make_str(width))

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = logic_unit(fp)
    uut.write(32)
    fp.close()