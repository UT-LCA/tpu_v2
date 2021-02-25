`include "vlane_mulshift.v"
`include "vlane_barrelshifter.v"

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

module vmul_unit(clk, resetn,
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
parameter LOG2WIDTH=5;
parameter NUMMULLANES=3;
parameter LOG2NUMLANES=4;
parameter REGIDWIDTH=4;

parameter NUMLANES=2**LOG2NUMLANES;
parameter WIDTH=2**LOG2WIDTH;

input clk;
input resetn;

input [NUMLANES*WIDTH-1:0] opA;
input [NUMLANES*WIDTH-1:0] opB;
input [((LOG2WIDTH==0) ? 1 : LOG2WIDTH)-1:0] vshamt;  // Fixed point rounding
input [NUMLANES-1:0] vmask;
input [4:0] op;
input       activate;
input [3:1] en;  //Enable for each pipestage
input [3:1] squash;  //Squash for each pipestage

input    [REGIDWIDTH-1:0] in_dst;
input                     in_dst_we;
output [3*REGIDWIDTH-1:0] out_dst;
output              [2:0] out_dst_we;
output   [3*NUMLANES-1:0] out_dst_mask;

output stall;
output [NUMLANES*WIDTH-1:0] result;

  /********* Circuit Body *********/
  wire [NUMMULLANES*WIDTH-1:0] mul_opA;
  wire [NUMMULLANES*WIDTH-1:0] mul_opB;
  wire [NUMMULLANES*WIDTH-1:0] mul_result;
  wire [NUMMULLANES*WIDTH-1:0] rshift_result;
  wire [NUMLANES*WIDTH-1:0] result_tmp;

  wire [NUMLANES*WIDTH-1:0] opA_buffered;
  wire [NUMLANES*WIDTH-1:0] opB_buffered;
  wire [NUMLANES-1:0]       mask_buffered;
  wire [NUMLANES*WIDTH-1:0] result_buffered;
  reg  done;
  wire [4:0] ctrl_op[3:1];                  //3 pipe stages
  wire [3:1] ctrl_activate;                 //3 pipe stages
  wire [((LOG2WIDTH==0) ? 1 : LOG2WIDTH)-1:0] ctrl_vshamt[3:1]; //3 pipe stages

  //Shift Register for all multiplier operands for lanes without multipliers
  wire [WIDTH*NUMMULLANES-1:0] opA_elmshifter_shiftin_right_NC;
  velmshifter #(NUMLANES/NUMMULLANES,WIDTH*NUMMULLANES) opA_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*NUMMULLANES{1'b0}}),
    .shiftin_right(opA_elmshifter_shiftin_right_NC), 
    .inpipe(opA),
    .outpipe(opA_buffered)
  );

  wire [WIDTH*NUMMULLANES-1:0] opB_elmshifter_shiftin_right_NC;
  velmshifter #(NUMLANES/NUMMULLANES,WIDTH*NUMMULLANES) opB_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*NUMMULLANES{1'b0}}),
    .shiftin_right(opB_elmshifter_shiftin_right_NC), 
    .inpipe(opB),
    .outpipe(opB_buffered)
  );

  wire [NUMMULLANES-1:0] mask_elmshifter_shiftin_right_NC;
  velmshifter #(NUMLANES/NUMMULLANES,NUMMULLANES) mask_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done & ctrl_activate[1]),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({NUMMULLANES{1'b0}}),
    .shiftin_right(mask_elmshifter_shiftin_right_NC), 
    //.inpipe(vmask), //DISABLE - always do all multiplications
    .inpipe({NUMLANES{1'b1}}),
    .outpipe(mask_buffered)
  );

  //Shift Register for all multiplier operands w/ parallel load
  always@(posedge clk)
  begin
    if (!resetn || NUMMULLANES==NUMLANES)
      done<=1;
    else if (done && ctrl_activate[1] && en[1])
      //done<=~(|(vmask>>NUMMULLANES)); // multiply only if mask - DISABLED
      done<=~(|(vmask));
    else
      done<=~( |(mask_buffered >> (2*NUMMULLANES) ));
  end

  assign mul_opA=(done) ? opA : (opA_buffered >> NUMMULLANES*WIDTH);

  assign mul_opB=(done) ? opB : (opB_buffered >> NUMMULLANES*WIDTH);

  assign stall=~done && (ctrl_activate[2]);

  wire [3:1] oppipe_squash_NC; 
  pipe #(5,2) oppipe (
    .d(op),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(oppipe_squash_NC),
    .q({ctrl_op[3],ctrl_op[2],ctrl_op[1]}));

  wire [3:1] activatepipe_squash_NC; 
  pipe #(1,2) activatepipe (
    .d(activate),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(activatepipe_squash_NC),
    .q({ctrl_activate[3],ctrl_activate[2],ctrl_activate[1]}));

  wire [3:1] vshamtpipe_squash_NC; 
  pipe #(((LOG2WIDTH==0) ? 1 : LOG2WIDTH),2) vshamtpipe (
    .d(vshamt),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(vshamtpipe_squash_NC),
    .q({ctrl_vshamt[3],ctrl_vshamt[2],ctrl_vshamt[1]}));

  //============== Instantiate across lanes =============
  genvar k;
  generate
  for (k=0; k<NUMMULLANES; k=k+1)
  begin : lanes_gen

    vlane_mulshift #(WIDTH,(LOG2WIDTH>0) ? LOG2WIDTH : 1) vmul(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*k +: WIDTH]),
      .opB(mul_opB[WIDTH*k +: WIDTH]),
      .sa( mul_opB[WIDTH*k+((LOG2WIDTH>0)?LOG2WIDTH-1:0) : WIDTH*k] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*k +: WIDTH])
      );

    vlane_barrelshifter #(WIDTH,(LOG2WIDTH>0)?LOG2WIDTH:1) vshift(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(k+1)-1:WIDTH*k]),
      .sa( ctrl_vshamt[2][((LOG2WIDTH>0) ? LOG2WIDTH-1:0) : 0] ),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(k+1)-1:WIDTH*k])
      );

  end
  endgenerate


  //Shift Register for all multiplier results
  wire [NUMMULLANES*WIDTH-1:0] shiftin_right_result_elmshifter_NC;
  wire [NUMLANES*WIDTH-1:0] inpipe_result_elmshifter_NC;
  velmshifter #(NUMLANES/NUMMULLANES,WIDTH*NUMMULLANES) result_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(1'b0),
    .shift(~done),
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

  assign result=(result_tmp<<((NUMLANES/NUMMULLANES-1)*NUMMULLANES*WIDTH)) |
                (result_buffered>>NUMMULLANES*WIDTH);

  wire [2:1] dstpipe_squash_NC;
  pipe #(REGIDWIDTH,2) dstpipe (
    .d( in_dst ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(dstpipe_squash_NC),
    .q(out_dst));

  pipe #(1,2) dstwepipe (
    .d( in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(squash[2:1]),
    .q(out_dst_we));

  wire [2:1] dstmaskpipe_squash_NC;
  pipe #(NUMLANES,2) dstmaskpipe (
    .d( vmask ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(dstmaskpipe_squash_NC),
    .q(out_dst_mask));


endmodule

