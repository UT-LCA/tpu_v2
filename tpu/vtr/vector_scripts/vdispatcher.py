from velmshifter_serial import velmshifter
from optparse import OptionParser
import os
parser = OptionParser()
(_,args) = parser.parse_args()

class vdispatcher():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, numentries, widthinstr, widthrdelm,widthwrelm,widthcount):
        string = '''\

module vdispatcher_{NUMENTRIES}_{WIDTHINSTR}_{WIDTHRDELM}_{WIDTHWRELM}_{WIDTHCOUNT}(
    clk,
    resetn,

    shift,
    rotate, //shift must also be high to rotate

    inshift_instr,
    inshift_first,
    inshift_rdelm,
    inshift_wrelm,
    inshift_count,

    increment,
    rdelm_add_sub,
    wrelm_add_sub,
    count_add_sub,
    rdelm_valuetoadd,
    wrelm_valuetoadd,
    count_valuetoadd,

    instr,
    first,
    rdelm,
    wrelm,
    count

    );

input clk;
input resetn;

input  shift;
input  rotate;

input [ {WIDTHINSTR}-1:0 ] inshift_instr;
input                    inshift_first;
input [ {WIDTHRDELM}-1:0 ] inshift_rdelm;
input [ {WIDTHRDELM}-1:0 ] inshift_wrelm;
input [ {WIDTHCOUNT}-1:0 ] inshift_count;

input [ {NUMENTRIES}-1:0 ] increment;

input rdelm_add_sub;
input wrelm_add_sub;
input count_add_sub;
input [ {WIDTHRDELM}-1:0 ] rdelm_valuetoadd;
input [ {WIDTHWRELM}-1:0 ] wrelm_valuetoadd;
input [ {WIDTHCOUNT}-1:0 ] count_valuetoadd;

output [ {NUMENTRIES}*{WIDTHINSTR}-1:0 ] instr;
output            [ {NUMENTRIES}-1:0 ] first;
output [ {NUMENTRIES}*{WIDTHRDELM}-1:0 ] rdelm;
output [ {NUMENTRIES}*{WIDTHWRELM}-1:0 ] wrelm;
output [ {NUMENTRIES}*{WIDTHCOUNT}-1:0 ] count;

wire [{NUMENTRIES}*{WIDTHINSTR}-1:0 ] inparallel_data_inst_NC;
vdispatcher_shift_{NUMENTRIES}_{WIDTHINSTR} vdispatcher_instr (
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .squash({CBS}{NUMENTRIES}{CBS}1'b0{CBE}{CBE}),
      .inshift_data(inshift_instr),
      .inparallel_data(inparallel_data_inst_NC),
      .outparallel_data(instr));

wire [ {NUMENTRIES}-1:0 ] inparallel_data_first_NC;
vdispatcher_shift_{NUMENTRIES}_1 vdispatcher_first (
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .squash(increment&~{CBS}32'b0,shift&~rotate{CBE}),
      .inshift_data(inshift_first),
      .inparallel_data(inparallel_data_first_NC),
      .outparallel_data(first));

wire [ {NUMENTRIES}*{WIDTHRDELM}-1:0 ] inparallel_data_rdelm_NC;
wire [ {NUMENTRIES}-1:0 ] squash_rdelm_NC;
vdispatcher_add_{NUMENTRIES}_{WIDTHRDELM} vdispatcher_rdelm(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_rdelm_NC),
      .add_sub(rdelm_add_sub),
      .valuetoadd(rdelm_valuetoadd),
      .inshift_data(inshift_rdelm),
      .inparallel_data(inparallel_data_rdelm_NC),
      .outparallel_data(rdelm));

wire [ {NUMENTRIES}*{WIDTHWRELM}-1:0 ] inparallel_data_wrelm_NC;
wire [ {NUMENTRIES}-1:0 ] squash_wrelm_NC;
vdispatcher_add_{NUMENTRIES}_{WIDTHWRELM} vdispatcher_wrelm(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_wrelm_NC),
      .add_sub(wrelm_add_sub),
      .valuetoadd(wrelm_valuetoadd),
      .inshift_data(inshift_wrelm),
      .inparallel_data(inparallel_data_wrelm_NC),
      .outparallel_data(wrelm));

wire [ {NUMENTRIES}*{WIDTHCOUNT}-1:0 ] inparallel_data_count_NC;
wire [ {NUMENTRIES}-1:0 ] squash_count_NC;
vdispatcher_add_{NUMENTRIES}_{WIDTHCOUNT} vdispatcher_count(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_count_NC),
      .add_sub(count_add_sub),
      .valuetoadd(count_valuetoadd),
      .inshift_data(inshift_count),
      .inparallel_data(inparallel_data_count_NC),
      .outparallel_data(count));


endmodule
'''
        filename = "verilog/velmshifter_shift_"+str(numentries)+"_"+str(widthinstr)
        if(os.path.exists(filename) == False):
            fp = open(filename,'w')
            uut = vdispatcher_shift(fp)
            uut.write(numentries,widthinstr)
            fp.close()
        filename = "verilog/velmshifter_shift_"+str(numentries)+"_"+"1"
        if(os.path.exists(filename) == False):
            fp = open(filename,'w')
            uut = vdispatcher_shift(fp)
            uut.write(numentries,1)
            fp.close()

        filename = "verilog/vdispatcher_add_"+str(numentries)+"_"+str(widthrdelm)
        if(os.path.exists(filename) == False):
            fp = open(filename,'w')
            uut = vdispatcher_add(fp)
            uut.write(numentries,widthrdelm)
            fp.close()
        filename = "verilog/vdispatcher_add_"+str(numentries)+"_"+str(widthwrelm)
        if(os.path.exists(filename) == False):
            fp = open(filename,'w')
            uut = vdispatcher_add(fp)
            uut.write(numentries,widthwrelm)
            fp.close()
        filename = "verilog/vdispatcher_add_"+str(numentries)+"_"+str(widthcount)
        if(os.path.exists(filename) == False):
            fp = open(filename,'w')
            uut = vdispatcher_add(fp)
            uut.write(numentries,widthcount)
            fp.close()

        return string.format(NUMENTRIES= numentries, WIDTHINSTR = widthinstr, WIDTHRDELM=widthrdelm, WIDTHWRELM=widthwrelm, WIDTHCOUNT=widthcount , CBS ="{", CBE = "}") 

    def write (self, numentries, widthinstr, widthrdelm,widthwrelm,widthcount):
        self.fp.write(self.make_str(numentries, widthinstr, widthrdelm,widthwrelm,widthcount))

class vdispatcher_shift():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, numentries,width ):
        string = '''\

module vdispatcher_shift_{NUMENTRIES}_{WIDTH} (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    squash,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ {NUMENTRIES}-1:0 ]  squash;

input [ {WIDTH}-1:0 ] inshift_data;

input [ {NUMENTRIES}*{WIDTH}-1:0 ]  inparallel_data;
output [ {NUMENTRIES}*{WIDTH}-1:0 ] outparallel_data;

wire [ {WIDTH}-1:0 ]  shiftin_left;
wire [ {WIDTH}-1:0 ]  shiftin_right;


  assign shiftin_right = (!rotate) ? inshift_data : 
                      outparallel_data[{NUMENTRIES}*{WIDTH}-1:({NUMENTRIES}-1)*{WIDTH}];

  assign shiftin_left = (!rotate) ? 0 : outparallel_data[{WIDTH}-1:0];

  velmshifter_{NUMENTRIES}_{WIDTH} velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load),
      .shift(shift),
      .dir_left(1),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe(inparallel_data),
      .outpipe(outparallel_data));


endmodule

'''
        filename = "verilog/velmshifter_"+str(numentries)+"_"+str(width)
        if(os.path.exists(filename) == False):
            fp = open(filename,'w')
            uut = velmshifter(fp)
            uut.write(numentries,width)
            fp.close()

        return string.format(NUMENTRIES=numentries, WIDTH=width) 

    def write (self, numentries, width):
        self.fp.write(self.make_str(numentries,width))

class vdispatcher_add():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, numentries, width):
        string = '''\


module vdispatcher_add_{NUMENTRIES}_{WIDTH} (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    increment,
    squash,

    add_sub,
    valuetoadd,

    inshift_data,

    inparallel_data,
    outparallel_data

    );


input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ {NUMENTRIES}-1:0 ]  increment;
input [ {NUMENTRIES}-1:0 ]  squash;

input               add_sub;
input [ {WIDTH}-1:0 ] valuetoadd;

input [ {WIDTH}-1:0 ] inshift_data;

input [ {NUMENTRIES}*{WIDTH}-1:0 ]  inparallel_data;
output [ {NUMENTRIES}*{WIDTH}-1:0 ] outparallel_data;

wire [ {WIDTH}-1:0 ]  shiftin_right;

reg [ {NUMENTRIES}*{WIDTH}-1:0 ] outparallel_data;
reg [ {NUMENTRIES}*{WIDTH}-1:0 ] outparallel_added;


  assign shiftin_right = (!rotate) ? inshift_data : 
                    outparallel_added[{NUMENTRIES}*{WIDTH}-1:({NUMENTRIES}-1)*{WIDTH}];

  reg [ {WIDTH}-1:0 ] v;
  reg[31:0] i;
  always@*
    for (i=0; i<{NUMENTRIES}; i=i+1)
    begin
      //Saturate at 0xffff... in either direction
      v=(increment[i] && outparallel_data[i*{WIDTH} +: {WIDTH}] != {CBS}{WIDTH}{CBS}1'b1{CBE}{CBE}) ?
        valuetoadd : 0;
      if (add_sub==0)
        outparallel_added[i*{WIDTH} +: {WIDTH}]=outparallel_data[i*{WIDTH}+:{WIDTH}] + v;
      else
        outparallel_added[i*{WIDTH} +: {WIDTH}]=outparallel_data[i*{WIDTH}+:{WIDTH}] - v;
    end

  always@(posedge clk)
    if (!resetn)
      outparallel_data=0;
    else if (load)
      outparallel_data=inparallel_data;
    else if (shift)
      outparallel_data=(outparallel_added<<{WIDTH}) | shiftin_right;
      


endmodule

'''

        return string.format(NUMENTRIES=numentries, WIDTH=width, CBS ="{", CBE = "}" ) 

    def write (self, numentries, width):
        self.fp.write(self.make_str(numentries,width))

if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = vdispatcher(fp)
    uut1.write(8,32,16,16,8)
    fp.close()
