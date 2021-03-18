from optparse import OptionParser
import re
parser = OptionParser()
(_,args) = parser.parse_args()

class vmem_busmux():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self,inwidth, log2inwidth, outwidth, log2outwidth):
        string = '''\
module vmem_busmux_{INWIDTH}_{LOG2INWIDTH}_{OUTWIDTH}_{LOG2OUTWIDTH} (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter SELWIDTH={LOG2INWIDTH}-{LOG2OUTWIDTH};   // LOG2(INWIDTH/OUTWIDTH) = 4

input clk;
input resetn;
input  [SELWIDTH-1 : 0] sel;
input  [{INWIDTH}-1 : 0]  in;
output [{OUTWIDTH}-1 : 0] out;

reg    [{OUTWIDTH}-1 : 0] out;

integer k;

  always@*
  begin
    out=0;
    for (k=0; k<{INWIDTH}/{OUTWIDTH}; k=k+1)
      if (k==sel)
        out=in[ k*{OUTWIDTH} +: {OUTWIDTH} ];
  end
endmodule
        '''
        return string.format(INWIDTH=inwidth, LOG2INWIDTH = log2inwidth, OUTWIDTH=outwidth, LOG2OUTWIDTH=log2outwidth) 

    def write (self, inwidth, log2inwidth, outwidth, log2outwidth):
        self.fp.write(self.make_str(inwidth, log2inwidth, outwidth, log2outwidth))

class vmem_crossbar():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, inwidth, log2inwidth, numouts, outwidth, log2outwidth):
        string1 = '''\
/************************
 *
 *************************/

module vmem_crossbar_{INWIDTH}_{LOG2INWIDTH}_{NUMOUTS}_{OUTWIDTH}_{LOG2OUTWIDTH} (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter SELWIDTH={LOG2INWIDTH}-{LOG2OUTWIDTH};   // LOG2(INWIDTH/OUTWIDTH) = 4

input                             clk;
input                             resetn;
input  [(SELWIDfTH*{NUMOUTS})-1 : 0] sel;
input  [{INWIDTH}-1 : 0]            in;
output [({OUTWIDTH}*{NUMOUTS})-1 : 0] out;

'''
        string2_basic = '''
     vmem_busmux_{INWIDTH}_{LOG2INWIDTH}_{OUTWIDTH}_{LOG2OUTWIDTH} bmux(clk,resetn,
        sel[(i+1)*SELWIDTH - 1 : i*SELWIDTH],
        in,
        out[(i+1)*{OUTWIDTH} - 1 : i*{OUTWIDTH}]);
'''

        string2 =""
        for i in range (0,numouts):
            #string2 += string2_basic.replace('i',str(i))
            temp = re.sub(r'\bi\b',str(i), string2_basic)
            temp = re.sub(r'bmux','bmux'+str(i), temp)
            string2 += temp

        string2 += "\n endmodule \n"
        string = string1 + string2

        uut = vmem_busmux(self.fp)
        uut.write(inwidth, log2inwidth, outwidth, log2outwidth)

        return string.format(INWIDTH=inwidth, LOG2INWIDTH = log2inwidth, NUMOUTS=numouts ,OUTWIDTH=outwidth, LOG2OUTWIDTH=log2outwidth) 

    def write (self,inwidth, log2inwidth, numouts, outwidth, log2outwidth):
        self.fp.write(self.make_str(inwidth, log2inwidth, numouts, outwidth, log2outwidth))


if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = vmem_crossbar(fp)
    uut2 = vmem_busmux(fp)
    uut1.write(128,7,16,8,3)
    uut2.write(128,7,8,3)
    fp.close()
