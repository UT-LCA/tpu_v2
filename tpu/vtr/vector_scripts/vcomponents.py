from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class pipe():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width,depth):
        string1 = '''\
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
`ifndef MODULE_PIPE_{WIDTH}_{DEPTH}
`define MODULE_PIPE_{WIDTH}_{DEPTH}
module pipe_{WIDTH}_{DEPTH}(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );
'''
        string2a = '''
input d;
input              clk;
input              resetn;
input en;
input squash;
output [1:0] q;

reg  tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash )
      tq <= 0;
    else if (en)
      tq <=d;
  end

  assign q[0] = d;
  assign q[1]=tq;
endmodule
        '''
        string2b = '''
input  d;
input              clk;
input              resetn;
input  [{DEPTH}-1:0] en;
input  [{DEPTH}-1:0] squash;
output [({DEPTH}+1)-1:0] q;

reg [{DEPTH}-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[0] <= 0;
    else if (en[0])
      tq[0] <=d;

    // All the rest registers
    for (i=1; i<{DEPTH}; i=i+1)
      if (!resetn || squash[i] )
        tq[i]<= 0;
      else if (en[i])
        tq[i]<=tq[(i-1)];
  end

  assign q[0] =d;
  assign q[({DEPTH}+1)-1:1]=tq;
endmodule
        '''
        string2c = '''
input [{WIDTH}-1:0]  d;
input              clk;
input              resetn;
input en;
input squash;
output [{WIDTH}*({DEPTH}+1)-1:0] q;

reg [{WIDTH}*{DEPTH}-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash )
      tq[ {WIDTH}-1:0 ]<= 0;
    else if (en)
      tq[ {WIDTH}-1:0 ]<=d;
  end

  assign q[{WIDTH}-1:0]=d;
  assign q[{WIDTH}*({DEPTH}+1)-1:{WIDTH}]=tq;
endmodule
        '''
        string2d = '''
input [{WIDTH}-1:0]  d;
input              clk;
input              resetn;
input  [{DEPTH}-1:0] en;
input  [{DEPTH}-1:0] squash;
output [{WIDTH}*({DEPTH}+1)-1:0] q;

reg [{WIDTH}*{DEPTH}-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ {WIDTH}-1:0 ]<= 0;
    else if (en[0])
      tq[ {WIDTH}-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<{DEPTH}; i=i+1)
      if (!resetn || squash[i] )
        tq[i*{WIDTH} +: {WIDTH} ]<= 0;
      else if (en[i])
        tq[i*{WIDTH} +: {WIDTH} ]<=tq[(i-1)*{WIDTH} +: {WIDTH} ];
  end

  assign q[{WIDTH}-1:0]=d;
  assign q[{WIDTH}*({DEPTH}+1)-1:{WIDTH}]=tq;
endmodule
        '''
        if(width ==1 and depth==1 ):
            string2 = string2a
        # elif(width == 1):
        #     string2 = string2b
        elif (depth==1 ):
            string2 = string2c
        else:
          string2 = string2d
        string = string1+ string2 
        string += "`endif\n"

        return string.format(WIDTH=width, DEPTH=depth)

    def write (self, width,depth):
        self.fp.write(self.make_str(width,depth))

class hazardchecker():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width,subwidth,depth,ltdepth):
        string = '''\
/****************************************************************************
          Hazard checker
 if mode==0 compare entire width of src & dests,
 if mode==1 compare upper WIDTH-SUBWIDTH bits of src & dests
****************************************************************************/
module hazardchecker_{WIDTH}_{SUBWIDTH}_{DEPTH}_{LTDEPTH} (
    src,
    src_valid,
    dst,
    dst_valid,
    dst_mode,
    lt_dst,
    lt_dst_valid,
    lt_mode,
    haz
    );

input [{WIDTH}-1:0]  src;
input              src_valid;
input [{WIDTH}*{DEPTH}-1:0]  dst;
input [{DEPTH}-1:0]    dst_valid;
input [{DEPTH}-1:0]  dst_mode;
input [{WIDTH}*{LTDEPTH}-1:0]  lt_dst;
input [{LTDEPTH}-1:0]    lt_dst_valid;
input [{LTDEPTH}-1:0]  lt_mode;
output             haz;

reg             t_haz;
reg             t_haz_lt;

reg[31:0] i,j;

  always@*
  begin
    t_haz=1'b0;
    t_haz_lt=1'b0;

    for (i=0; i<{DEPTH}; i=i+1)
      if (dst_mode)
        t_haz=t_haz | ((src[{WIDTH}-1:{SUBWIDTH}]==dst[i*{WIDTH}+{SUBWIDTH} +: {WIDTH}-{SUBWIDTH} ])&dst_valid[i]&src_valid);
      else
        t_haz=t_haz | ((src==dst[i*{WIDTH} +: {WIDTH} ])&dst_valid[i]&src_valid);

    //Check if src is less than dest - used for hazards in issuer
    for (j=0; j<{LTDEPTH}; j=j+1)
      if (lt_mode)
        t_haz_lt=t_haz_lt | ((src[{WIDTH}-1:{SUBWIDTH}]==lt_dst[j*{WIDTH}+{SUBWIDTH} +: {WIDTH}-{SUBWIDTH} ])&lt_dst_valid[j]&src_valid);
      else
        t_haz_lt=t_haz_lt | (((src[{WIDTH}-1:{SUBWIDTH}]==lt_dst[j*{WIDTH}+{SUBWIDTH} +: {WIDTH}-{SUBWIDTH} ])&lt_dst_valid[j]&src_valid) && (src[{SUBWIDTH}-1:0]>=lt_dst[j*{WIDTH} +: {SUBWIDTH}]));
  end

  assign haz=(t_haz|t_haz_lt)&src_valid;

endmodule
        '''
        return string.format(WIDTH=width,SUBWIDTH=subwidth,DEPTH=depth,LTDEPTH=ltdepth)

    def write (self, width,subwidth,depth,ltdepth):
        self.fp.write(self.make_str(width,subwidth,depth,ltdepth))

if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = pipe(fp)
    #uut2 = hazardchecker(fp)
    uut1.write(1,3)
    #uut2.write(10,5,1,1)
    fp.close()
