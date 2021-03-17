from optparse import OptionParser
from os import path
import os
import re
parser = OptionParser()
(_,args) = parser.parse_args()

class velmrotator():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, numlanes,width):
        string = '''\
/************************
 * An Inter-lane shift register
 *
 * This is much more scalable than a giant barrel shifter/crossbar, however
 * it is also slower in cycles.
 *
 *  dir_left:  1=left, 0=right
 *
 * Synthesis results for NUMLANES=4, WIDTH=32:
 *    131 LEs (Stratix I), 422 MHz
 *************************/

/***************************************************************************
  shiftin_left -> bn-1 <-> bn-2 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

module velmrotator_{NUMLANES}_{WIDTH} (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    rotate,       // Sets whether ends feedback into beginning
    squash,

    inpipe,
    outpipe

    );

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  rotate;
input [ {NUMLANES}-1:0 ]  squash;

input [ {NUMLANES}*{WIDTH}-1:0 ]  inpipe;
output [ {NUMLANES}*{WIDTH}-1:0 ] outpipe;

wire [ {WIDTH}-1:0 ]  shiftin_left;
wire [ {WIDTH}-1:0 ]  shiftin_right;


  assign shiftin_right = (!rotate) ? 0 : 
                        outpipe[{NUMLANES}*{WIDTH}-1:({NUMLANES}-1)*{WIDTH}];

  assign shiftin_left = (!rotate) ? 0 : outpipe[{WIDTH}-1:0];

  velmshifter_{NUMLANES}_{WIDTH} velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load),
      .shift(shift),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe(inpipe),
      .outpipe(outpipe));

endmodule
        '''
        return string.format(NUMLANES=numlanes, WIDTH=width)

    def write (self, numlanes, width):
        self.fp.write(self.make_str(numlanes,width))

class velmshifter_jump():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, numlanes, jumpsize, width):
        string = '''\

/******************************************************************************/
/**************************** Shifter w Jump **********************************/
/******************************************************************************/

module velmshifter_jump_{WIDTH}_{NUMLANES}_{JUMPSIZE} (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    jump,
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe
    );


input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  jump;              //0-shift by 1, 1 shift by {JUMPSIZE}
input [ {NUMLANES}-1:0 ]  squash;

input [ {NUMLANES}*{WIDTH}-1:0 ]  inpipe;
output [ {NUMLANES}*{WIDTH}-1:0 ] outpipe;

input [ {WIDTH}-1:0 ]  shiftin_left;
input [ {WIDTH}-1:0 ]  shiftin_right;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter_{NUMLANES}_{WIDTH} velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<({WIDTH}*{JUMPSIZE}) :
                      outpipe >>({WIDTH}*{JUMPSIZE})),
      .outpipe(outpipe));

endmodule
        '''
        return string.format(WIDTH=width, NUMLANES=numlanes, JUMPSIZE=jumpsize)

    def write (self, numlanes, jumpsize, width):
        self.fp.write(self.make_str(numlanes,jumpsize,width))

class velmshifter():
    def __init__(self, fp):
        self.fp = fp

    def collapse_bit_slices(self, string):
        all_bit_slices = re.findall(r'\[\d*:\d*\]', string)
        all_bit_slices = list(set(all_bit_slices))
        for slice in all_bit_slices:
            print("slice is: ", slice)
            m = re.search(r'(\d*):(\d*)', slice)
            if m.group(1) == m.group(2):
                string = re.sub(re.escape(slice), "["+m.group(1)+"]", string)
        return string

    def make_str(self, numlanes, width):
        string1 = '''\

/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_{NUMLANES}_{WIDTH} (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ {NUMLANES}-1:0 ]  squash;

input [ {NUMLANES}*{WIDTH}-1:0 ]  inpipe;
output [ {NUMLANES}*{WIDTH}-1:0 ] outpipe;

input [ {WIDTH}-1:0 ]  shiftin_left;
input [ {WIDTH}-1:0 ]  shiftin_right;

wire [ ({NUMLANES}+1)*{WIDTH}-1:0 ] _outpipe;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially
'''
        string2_basic1 = '''
  velmshifter_laneunit_{WIDTH} velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[{WIDTH}-1:0],
      _outpipe[{WIDTH}+:{WIDTH}], //Support 1 lane
      shiftin_right,
      _outpipe[{WIDTH}-1:0]);
 // defparam velmshifter_laneunit0.{WIDTH}={WIDTH};

  //Generate everything in between 

'''
        if width == 1:
            string2_basic1 = string2_basic1.replace("{WIDTH}-1:0","0")
            string2_basic1 = string2_basic1.replace("{WIDTH}+:{WIDTH}","1")
        else:
            string2_basic1 = string2_basic1.replace("{WIDTH}-1:0",str(width-1)+":0")
            string2_basic1 = string2_basic1.replace("{WIDTH}+:{WIDTH}",str(2*width-1)+":"+str(width))

        string2_basic1 = self.collapse_bit_slices(string2_basic1)
     
        string2_basic = '''
      velmshifter_laneunit_{WIDTH} velmshifter_laneunit_(_i_)(clk,resetn,load,shift,dir_left,
          squash[(_i_)],
          inpipe[(_i_+1)*{WIDTH}-1:(_i_)*{WIDTH}],
          _outpipe[(_i_+2)*{WIDTH}-1:(_i_+1)*{WIDTH}],
          _outpipe[(_i_)*{WIDTH}-1:(_i_-1)*{WIDTH}],
          _outpipe[(_i_+1)*{WIDTH}-1:(_i_)*{WIDTH}]);
     // defparam velmshifter_laneunit.{WIDTH}={WIDTH};

'''
        string2 = ""
        for i in range(1,numlanes-1):
            string2_basic = string2_basic.replace('(_i_+1)*{WIDTH}-1',str( ((i+1)*width)-1))
            string2_basic = string2_basic.replace('(_i_+1)*{WIDTH}',str( (i+1) * width))
            string2_basic = string2_basic.replace('(_i_+2)*{WIDTH}-1',str(((i+2) * width)-1))
            string2_basic = string2_basic.replace('(_i_-1)*{WIDTH}',str((i-1)*width))
            string2_basic = string2_basic.replace('(_i_)*{WIDTH}-1',str((i)*width-1))
            string2_basic = string2_basic.replace('(_i_)*{WIDTH}',str((i)*width))
            string2 = string2 + string2_basic.replace('(_i_)',str((i)))

        string2 = self.collapse_bit_slices(string2)

        string3 = '''
  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_{WIDTH} velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[{NUMLANES}-1],
      inpipe[{NUMLANES}*{WIDTH}-1:({NUMLANES}-1)*{WIDTH}],
      shiftin_left,
      _outpipe[({NUMLANES}-2)*{WIDTH} +: {WIDTH}], //L=1
      _outpipe[({NUMLANES}-1)*{WIDTH} +: {WIDTH}]); //L=1
   // defparam velmshifter_laneunitlast.{WIDTH}={WIDTH};

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        '''
        if width==1:
            string3 = string3.replace("({NUMLANES}-2)*{WIDTH} +: {WIDTH}",str((numlanes-2)*width))
            string3 = string3.replace("({NUMLANES}-1)*{WIDTH} +: {WIDTH}",str((numlanes-1)*width))
        else:
            string3 = string3.replace("({NUMLANES}-2)*{WIDTH} +: {WIDTH}", str(((numlanes-2)*width) + width-1) + ":" + str((numlanes-2)*width) )
            string3 = string3.replace("({NUMLANES}-1)*{WIDTH} +: {WIDTH}", str(((numlanes-1)*width) + width-1) + ":" + str((numlanes-1)*width) )
        string3 = string3.replace("({NUMLANES}-1)*{WIDTH}",str((numlanes-1)*width))
        string3 = string3.replace("({NUMLANES}-2)*{WIDTH}",str((numlanes-2)*width))
        string3 = string3.replace("{NUMLANES}*{WIDTH}-1",str(numlanes*width-1))
        string3 = string3.replace("{NUMLANES}-1",str(numlanes-1))

        string3 = self.collapse_bit_slices(string3)

        string = string1 + string2_basic1 + string2 + string3

 
        filename = "verilog/velmshifter_laneunit_"+str(width)
        if(os.path.exists(filename) == False):
            fp = open(filename,'w')
            uut = velmshifter_laneunit(fp)
            uut.write(width)

        return string.format(NUMLANES=numlanes, WIDTH=width)

    def write (self, numlanes, width):
        self.fp.write(self.make_str(numlanes,width))

class velmshifter_laneunit():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string = '''\
module velmshifter_laneunit_{WIDTH} (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );


input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ {WIDTH}-1:0 ]  inpipe;
input [ {WIDTH}-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ {WIDTH}-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ {WIDTH}-1:0 ] outpipe;

reg [ {WIDTH}-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        '''
        return string.format(WIDTH=width)

    def write (self, width):
        self.fp.write(self.make_str(width))

if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = velmrotator(fp)
    uut2 = velmshifter_jump(fp)
    uut3 = velmshifter(fp)
    uut4 = velmshifter_laneunit(fp)
    uut1.write(4,32)
    uut2.write(4,4,32)
    uut3.write(4,32)
    uut4.write(32)
    fp.close()
