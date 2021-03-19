from velmshifter_serial import velmshifter
from vlane_mulshift import vlane_mulshift
from vlane_barrelshifter import vlane_barrelshifter
from vcomponents import pipe
from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class vmul_unit():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, log2width,nummullanes,log2numlanes,regidwidth):
        numlanes = pow(2,log2numlanes)
        shifter_lane = int(numlanes/nummullanes)
        width = (pow(2,log2width))
        shifter_width = width * nummullanes
        string1 = '''
         
//`include "vlane_mulshift.v"
//`include "vlane_barrelshifter.v"

/****************************************************************************
          MUL unit

opA/B ---------------------------|\
         |                       | |----| Multiplier |--+------------------
         ------|shiftregister|---|/                     |--|shiftregister|-

 |  Pipe stage 1  |      Pipe stage 2    |    Pipe stage 3
                         done/stall
   operands----Multiply------------barrelshifter ----------

Notes:

  Stalls no matter the vector length - if the mask bits are all off we could
  theoretically skip this computation, but then we would need multiplexing
  logic to order the different results correctly so they line up with their
  lane.  Since we use a shift register to do this, all NUMLANES multiplies are
  performed.

****************************************************************************/

module vmul_unit_{LOG2WIDTH}_{NUMMULLANES}_{LOG2NUMLANES}_{REGIDWIDTH} (clk, resetn,
            op,
            activate,
            en,
            squash,
            stall,
            opA, opB,
            vshamt,
            vmask,
            in_dst,
            in_dst_we,
            out_dst,
            out_dst_we,
            out_dst_mask,
            result
            );

parameter NUMLANES=2**{LOG2NUMLANES};
parameter WIDTH=2**{LOG2WIDTH};

input clk;
input resetn;

input [NUMLANES*WIDTH-1:0] opA;
input [NUMLANES*WIDTH-1:0] opB;
// input [(({LOG2WIDTH}==0) ? 1 : {LOG2WIDTH})-1:0] vshamt;  // Fixed point rounding
// The original version is the above declaration. This is not supported by VTR. So I am using the below version ~ Aatman
input [{LOG2WIDTH}-1:0] vshamt;
input [NUMLANES-1:0] vmask;
input [4:0] op;
input       activate;
input [3:1] en;  //Enable for each pipestage
input [3:1] squash;  //Squash for each pipestage

input    [{REGIDWIDTH}-1:0] in_dst;
input                     in_dst_we;
output [3*{REGIDWIDTH}-1:0] out_dst;
output              [2:0] out_dst_we;
output   [3*NUMLANES-1:0] out_dst_mask;

output stall;
output [NUMLANES*WIDTH-1:0] result;

  /********* Circuit Body *********/
  wire [{NUMMULLANES}*WIDTH-1:0] mul_opA;
  wire [{NUMMULLANES}*WIDTH-1:0] mul_opB;
  wire [{NUMMULLANES}*WIDTH-1:0] mul_result;
  wire [{NUMMULLANES}*WIDTH-1:0] rshift_result;
  wire [NUMLANES*WIDTH-1:0] result_tmp;

  wire [NUMLANES*WIDTH-1:0] opA_buffered;
  wire [NUMLANES*WIDTH-1:0] opB_buffered;
  wire [NUMLANES-1:0]       mask_buffered;
  wire [NUMLANES*WIDTH-1:0] result_buffered;
  reg  done;
  wire [4:0] ctrl_op[3:1];                  //3 pipe stages
  wire [3:1] ctrl_activate;                 //3 pipe stages
  //amana: Making modification for VTR
  //wire [(({LOG2WIDTH}==0) ? 1 : {LOG2WIDTH})-1:0] ctrl_vshamt[3:1]; //3 pipe stages
  // wire [(({LOG2WIDTH}==0) ? 1*3 : {LOG2WIDTH}*3)-1:0] ctrl_vshamt; //3 pipe stages
  wire [{LOG2WIDTH}*3-1:0] ctrl_vshamt; //3 pipe stages

  //Shift Register for all multiplier operands for lanes without multipliers
  wire [WIDTH*{NUMMULLANES}-1:0] opA_elmshifter_shiftin_right_NC;
 
 //TODO: update the parameters
  velmshifter_{SHIFTER_LANE}_{SHIFTER_WIDTH}  opA_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({CBS}WIDTH*{NUMMULLANES}{CBS}1'b0{CBE}{CBE}),
    .shiftin_right(opA_elmshifter_shiftin_right_NC), 
    .inpipe(opA),
    .outpipe(opA_buffered)
  );

  wire [WIDTH*{NUMMULLANES}-1:0] opB_elmshifter_shiftin_right_NC;

  //TODO: update the parameters
  velmshifter_{SHIFTER_LANE}_{SHIFTER_WIDTH} opB_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({CBS}WIDTH*{NUMMULLANES}{CBS}1'b0{CBE}{CBE}),
    .shiftin_right(opB_elmshifter_shiftin_right_NC), 
    .inpipe(opB),
    .outpipe(opB_buffered)
  );

  wire [{NUMMULLANES}-1:0] mask_elmshifter_shiftin_right_NC;
  wire mask_elmshifter_load;

  assign mask_elmshifter_load = done & ctrl_activate[1];

  //TODO: Update the parameters 
  velmshifter_{SHIFTER_LANE}_{NUMMULLANES}  mask_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(mask_elmshifter_load),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({CBS}{NUMMULLANES}{CBS}1'b0{CBE}{CBE}),
    .shiftin_right(mask_elmshifter_shiftin_right_NC), 
    //.inpipe(vmask), //DISABLE - always do all multiplications
    .inpipe({CBS}NUMLANES{CBS}1'b1{CBE}{CBE}),
    .outpipe(mask_buffered)
  );

  //Shift Register for all multiplier operands w/ parallel load
  always@(posedge clk)
  begin
    if (!resetn || {NUMMULLANES}==NUMLANES)
      done<=1;
    else if (done && ctrl_activate[1] && en[1])
      //done<=~(|(vmask>>{NUMMULLANES})); // multiply only if mask - DISABLED
      done<=~(|(vmask));
    else
      done<=~( |(mask_buffered >> (2*{NUMMULLANES}) ));
  end

  assign mul_opA=(done) ? opA : (opA_buffered >> {NUMMULLANES}*WIDTH);

  assign mul_opB=(done) ? opB : (opB_buffered >> {NUMMULLANES}*WIDTH);

  assign stall=~done && (ctrl_activate[2]);

  wire [3:1] oppipe_squash_NC; 
  wire [5*(2+1)-1:0] oppipe_q;

  assign {CBS}ctrl_op[3],ctrl_op[2],ctrl_op[1]{CBE} = oppipe_q;

  pipe_5_2 oppipe (
    .d(op),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(oppipe_squash_NC),
    .q(oppipe_q)
  );

  wire [3:1] activatepipe_squash_NC;
  wire [(2+1)-1:0]  activatepipe_q;

  assign {CBS}ctrl_activate[3],ctrl_activate[2],ctrl_activate[1]{CBE} = activatepipe_q;
  pipe_1_2  activatepipe (
    .d(activate),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(activatepipe_squash_NC),
    .q(activatepipe_q)
  );

  wire [3:1] vshamtpipe_squash_NC; 

//TODO: remove parameters

  pipe_{LOG2WIDTH}_2 vshamtpipe (
    .d(vshamt),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(vshamtpipe_squash_NC),
    //.q({CBS}ctrl_vshamt[3],ctrl_vshamt[2],ctrl_vshamt[1]{CBE}));
    .q(ctrl_vshamt));


'''

#######################################
# Exploring genvar statement using 
# python for loop. the basic string 
# includes the content of generate loop
# Here is the genvar code: 
#
#  genvar k;
#  generate
#  for (k=0; k<{NUMMULLANES}; k=k+1)
#   begin
#       <code>
#   end
#  endgenerate
#######################################


        string2_basic='''\

    wire vmul[k]_en;
    wire [4:0] vmul[k]_op;

    assign vmul[k]_en = en[1] | ~done;
    assign vmul[k]_op = (done&en[1]) ? ctrl_op[1] : ctrl_op[2];

    vlane_mulshift_{WIDTH}_{LOG2WIDTH}  vmul[k](
      .clk(clk),
      .resetn(resetn),
      .en(vmul[k]_en),
      .opA(mul_opA[WIDTH*[k] +: WIDTH]),
      .opB(mul_opB[WIDTH*[k] +: WIDTH]),
      .sa( mul_opB[WIDTH*[k]+{LOG2WIDTH}-1 : WIDTH*[k]] ),
      .op(vmul[k]_op),
      .result(mul_result[WIDTH*[k] +: WIDTH])
      );

    wire [{LOG2WIDTH}-1:0] vshift[k]_sa;
    wire [2-1:0] vshift[k]_op;

    assign vshift[k]_sa = ctrl_vshamt[3*{LOG2WIDTH}-1:2*{LOG2WIDTH}];
    assign vshift[k]_op = {CBS}~ctrl_op[2][1] ,1'b1{CBE};

//TO DO: parameters

    vlane_barrelshifter_{WIDTH}_{LOG2WIDTH} vshift[k](
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*([k]+1)-1:WIDTH*[k]]),
      .sa(vshift[k]_sa),
      .op(vshift[k]_op),
      .result(rshift_result[WIDTH*([k]+1)-1:WIDTH*[k]])
      );

        '''
        string2 ="//============== Instantiate across lanes ============= \
                 "

        for k in range(0,nummullanes):
            string2  = string2 + string2_basic.replace("[k]",str(k)) + "\n"

        string3='''\

//TO DO: parameters

  //Shift Register for all multiplier results
  wire [{NUMMULLANES}*WIDTH-1:0] shiftin_right_result_elmshifter_NC;
  wire [NUMLANES*WIDTH-1:0] inpipe_result_elmshifter_NC;
  wire shift_result_elmshifter;
  assign shift_result_elmshifter = ~done;

  velmshifter_{SHIFTER_LANE}_{SHIFTER_WIDTH}  result_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(1'b0),
    .shift(shift_result_elmshifter),
    .dir_left(1'b0),
    .squash(1'b0),
    //Enable/Disable rshifter for fixed-point multiply support
    //.shiftin_left(rshift_result),
    .shiftin_left(mul_result),
    .shiftin_right(shiftin_right_result_elmshifter_NC),
    .inpipe(inpipe_result_elmshifter_NC),
    .outpipe(result_buffered)
  );

  //Enable/Disable rshifter for fixed-point multiply support
  //assign result_tmp=rshift_result;
  assign result_tmp=mul_result;

  assign result=(result_tmp<<((NUMLANES/{NUMMULLANES}-1)*{NUMMULLANES}*WIDTH)) |
                (result_buffered>>{NUMMULLANES}*WIDTH);

  wire [2:1] dstpipe_squash_NC;
  wire [2-1:0] dstpipe_en;
  assign dstpipe_en = en[2:1] & {CBS}1'b1,~stall{CBE};

  pipe_{REGIDWIDTH}_2 dstpipe (
    .d(in_dst),  
    .clk(clk),
    .resetn(resetn),
    .en(dstpipe_en),
    .squash(dstpipe_squash_NC),
    .q(out_dst));

  wire [2-1:0] dstwepipe_en;
  assign dstwepipe_en = en[2:1] & {CBS}1'b1,~stall{CBE};

  pipe_1_2 dstwepipe (
    .d(in_dst_we),  
    .clk(clk),
    .resetn(resetn),
    .en(dstwepipe_en),
    .squash(squash[2:1]),
    .q(out_dst_we));

  wire [2-1:0] dstmaskpipe_en;
  assign dstmaskpipe_en = en[2:1] & {CBS}1'b1,~stall{CBE};

  wire [2:1] dstmaskpipe_squash_NC;
  pipe_{NUMLANES}_2 dstmaskpipe (
    .d(vmask),  
    .clk(clk),
    .resetn(resetn),
    .en(dstmaskpipe_en),
    .squash(dstmaskpipe_squash_NC),
    .q(out_dst_mask));


endmodule
'''
        string = string1 + string2 + string3
        fp = open("verilog/velmshifter.v",'a')
        uut = velmshifter(fp)
        uut.write(shifter_lane,shifter_width)
        uut.write(shifter_lane,nummullanes)
        fp.close()
        fp = open("verilog/vlane_mulshift.v",'a')
        uut = vlane_mulshift(fp)
        uut.write(width,log2width)
        fp.close()
        fp = open("verilog/vlane_barrelshifter.v",'a')
        uut = vlane_barrelshifter(fp)
        uut.write(width,log2width)
        fp.close()
        fp = open("verilog/pipe.v",'a')
        uut = pipe(fp)
        uut.write(1,2)
        uut.write(5,2)
        if (log2width!=1 and log2width!=5):
          uut.write(log2width,2)
        if (regidwidth!=1 and regidwidth!=5 and regidwidth!=log2width):
          uut.write(regidwidth,2)
        if (numlanes!=1 and numlanes!=5 and numlanes!=log2width and numlanes!= regidwidth):
          uut.write(numlanes,2)
        fp.close()

        return string.format(LOG2WIDTH = log2width, \
                             NUMMULLANES = nummullanes, \
                             NUMLANES = numlanes, \
                             LOG2NUMLANES =log2numlanes, \
                             REGIDWIDTH = regidwidth, \
                             SHIFTER_LANE = shifter_lane, \
                             SHIFTER_WIDTH = shifter_width, \
                             CBS="{", \
                             CBE="}",  \
                             WIDTH=width) 

    def write (self,log2width,nummullanes,log2numlanes,regidwidth):
        self.fp.write(self.make_str(log2width,nummullanes,log2numlanes,regidwidth))


if __name__ == '__main__':
    fp = open(args[0], "w")
    uut1 = vmul_unit(fp)
    uut1.write(5,3,4,4)
    fp.close()
