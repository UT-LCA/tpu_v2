
         
//`include "vlane_mulshift.v"
//`include "vlane_barrelshifter.v"

/****************************************************************************
          MUL unit

opA/B ---------------------------|         |                       | |----| Multiplier |--+------------------
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

module vmul_unit_5_4_2_10 (clk, resetn,
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

parameter NUMLANES=2**2;
parameter WIDTH=2**5;

input clk;
input resetn;

input [NUMLANES*WIDTH-1:0] opA;
input [NUMLANES*WIDTH-1:0] opB;
// input [((5==0) ? 1 : 5)-1:0] vshamt;  // Fixed point rounding
// The original version is the above declaration. This is not supported by VTR. So I am using the below version ~ Aatman
input [5-1:0] vshamt;
input [NUMLANES-1:0] vmask;
input [4:0] op;
input       activate;
input [3:1] en;  //Enable for each pipestage
input [3:1] squash;  //Squash for each pipestage

input    [10-1:0] in_dst;
input                     in_dst_we;
output [3*10-1:0] out_dst;
output              [2:0] out_dst_we;
output   [3*NUMLANES-1:0] out_dst_mask;

output stall;
output [NUMLANES*WIDTH-1:0] result;

  /********* Circuit Body *********/
  wire [4*WIDTH-1:0] mul_opA;
  wire [4*WIDTH-1:0] mul_opB;
  wire [4*WIDTH-1:0] mul_result;
  wire [4*WIDTH-1:0] rshift_result;
  wire [NUMLANES*WIDTH-1:0] result_tmp;

  wire [NUMLANES*WIDTH-1:0] opA_buffered;
  wire [NUMLANES*WIDTH-1:0] opB_buffered;
  wire [NUMLANES-1:0]       mask_buffered;
  wire [NUMLANES*WIDTH-1:0] result_buffered;
  reg  done;
  wire [4:0] ctrl_op[3:1];                  //3 pipe stages
  wire [3:1] ctrl_activate;                 //3 pipe stages
  //amana: Making modification for VTR
  //wire [((5==0) ? 1 : 5)-1:0] ctrl_vshamt[3:1]; //3 pipe stages
  // wire [((5==0) ? 1*3 : 5*3)-1:0] ctrl_vshamt; //3 pipe stages
  wire [5*3-1:0] ctrl_vshamt; //3 pipe stages

  //Shift Register for all multiplier operands for lanes without multipliers
  wire [WIDTH*4-1:0] opA_elmshifter_shiftin_right_NC;
 
 //TODO: update the parameters
  velmshifter_1_128  opA_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*4{1'b0}}),
    .shiftin_right(opA_elmshifter_shiftin_right_NC), 
    .inpipe(opA),
    .outpipe(opA_buffered)
  );

  wire [WIDTH*4-1:0] opB_elmshifter_shiftin_right_NC;

  //TODO: update the parameters
  velmshifter_1_128 opB_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*4{1'b0}}),
    .shiftin_right(opB_elmshifter_shiftin_right_NC), 
    .inpipe(opB),
    .outpipe(opB_buffered)
  );

  wire [4-1:0] mask_elmshifter_shiftin_right_NC;
  wire mask_elmshifter_load;

  assign mask_elmshifter_load = done & ctrl_activate[1];

  //TODO: Update the parameters 
  velmshifter_1_4  mask_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(mask_elmshifter_load),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({4{1'b0}}),
    .shiftin_right(mask_elmshifter_shiftin_right_NC), 
    //.inpipe(vmask), //DISABLE - always do all multiplications
    .inpipe({NUMLANES{1'b1}}),
    .outpipe(mask_buffered)
  );

  //Shift Register for all multiplier operands w/ parallel load
  always@(posedge clk)
  begin
    if (!resetn || 4==NUMLANES)
      done<=1;
    else if (done && ctrl_activate[1] && en[1])
      //done<=~(|(vmask>>4)); // multiply only if mask - DISABLED
      done<=~(|(vmask));
    else
      done<=~( |(mask_buffered >> (2*4) ));
  end

  assign mul_opA=(done) ? opA : (opA_buffered >> 4*WIDTH);

  assign mul_opB=(done) ? opB : (opB_buffered >> 4*WIDTH);

  assign stall=~done && (ctrl_activate[2]);

  wire [3:1] oppipe_squash_NC; 
  wire [5*(2+1)-1:0] oppipe_q;

  assign {ctrl_op[3],ctrl_op[2],ctrl_op[1]} = oppipe_q;

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

  assign {ctrl_activate[3],ctrl_activate[2],ctrl_activate[1]} = activatepipe_q;
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

  pipe_5_2 vshamtpipe (
    .d(vshamt),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(vshamtpipe_squash_NC),
    //.q({ctrl_vshamt[3],ctrl_vshamt[2],ctrl_vshamt[1]}));
    .q(ctrl_vshamt));


//============== Instantiate across lanes =============                  
    wire vmul0_en;
    wire [4:0] vmul0_op;

    assign vmul0_en = en[1] | ~done;
    assign vmul0_op = (done&en[1]) ? ctrl_op[1] : ctrl_op[2];

    vlane_mulshift_32_5  vmul0(
      .clk(clk),
      .resetn(resetn),
      .en(vmul0_en),
      .opA(mul_opA[WIDTH*0 +: WIDTH]),
      .opB(mul_opB[WIDTH*0 +: WIDTH]),
      .sa( mul_opB[WIDTH*0+5-1 : WIDTH*0] ),
      .op(vmul0_op),
      .result(mul_result[WIDTH*0 +: WIDTH])
      );

    wire [5-1:0] vshift0_sa;
    wire [2-1:0] vshift0_op;

    assign vshift0_sa = ctrl_vshamt[3*5-1:2*5];
    assign vshift0_op = {~ctrl_op[2][1] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift0(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(0+1)-1:WIDTH*0]),
      .sa(vshift0_sa),
      .op(vshift0_op),
      .result(rshift_result[WIDTH*(0+1)-1:WIDTH*0])
      );

        

    wire vmul1_en;
    wire [4:0] vmul1_op;

    assign vmul1_en = en[1] | ~done;
    assign vmul1_op = (done&en[1]) ? ctrl_op[1] : ctrl_op[2];

    vlane_mulshift_32_5  vmul1(
      .clk(clk),
      .resetn(resetn),
      .en(vmul1_en),
      .opA(mul_opA[WIDTH*1 +: WIDTH]),
      .opB(mul_opB[WIDTH*1 +: WIDTH]),
      .sa( mul_opB[WIDTH*1+5-1 : WIDTH*1] ),
      .op(vmul1_op),
      .result(mul_result[WIDTH*1 +: WIDTH])
      );

    wire [5-1:0] vshift1_sa;
    wire [2-1:0] vshift1_op;

    assign vshift1_sa = ctrl_vshamt[3*5-1:2*5];
    assign vshift1_op = {~ctrl_op[2][1] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift1(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(1+1)-1:WIDTH*1]),
      .sa(vshift1_sa),
      .op(vshift1_op),
      .result(rshift_result[WIDTH*(1+1)-1:WIDTH*1])
      );

        

    wire vmul2_en;
    wire [4:0] vmul2_op;

    assign vmul2_en = en[1] | ~done;
    assign vmul2_op = (done&en[1]) ? ctrl_op[1] : ctrl_op[2];

    vlane_mulshift_32_5  vmul2(
      .clk(clk),
      .resetn(resetn),
      .en(vmul2_en),
      .opA(mul_opA[WIDTH*2 +: WIDTH]),
      .opB(mul_opB[WIDTH*2 +: WIDTH]),
      .sa( mul_opB[WIDTH*2+5-1 : WIDTH*2] ),
      .op(vmul2_op),
      .result(mul_result[WIDTH*2 +: WIDTH])
      );

    wire [5-1:0] vshift2_sa;
    wire [2-1:0] vshift2_op;

    assign vshift2_sa = ctrl_vshamt[3*5-1:2*5];
    assign vshift2_op = {~ctrl_op[2][1] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift2(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(2+1)-1:WIDTH*2]),
      .sa(vshift2_sa),
      .op(vshift2_op),
      .result(rshift_result[WIDTH*(2+1)-1:WIDTH*2])
      );

        

    wire vmul3_en;
    wire [4:0] vmul3_op;

    assign vmul3_en = en[1] | ~done;
    assign vmul3_op = (done&en[1]) ? ctrl_op[1] : ctrl_op[2];

    vlane_mulshift_32_5  vmul3(
      .clk(clk),
      .resetn(resetn),
      .en(vmul3_en),
      .opA(mul_opA[WIDTH*3 +: WIDTH]),
      .opB(mul_opB[WIDTH*3 +: WIDTH]),
      .sa( mul_opB[WIDTH*3+5-1 : WIDTH*3] ),
      .op(vmul3_op),
      .result(mul_result[WIDTH*3 +: WIDTH])
      );

    wire [5-1:0] vshift3_sa;
    wire [2-1:0] vshift3_op;

    assign vshift3_sa = ctrl_vshamt[3*5-1:2*5];
    assign vshift3_op = {~ctrl_op[2][1] ,1'b1};

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift3(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(3+1)-1:WIDTH*3]),
      .sa(vshift3_sa),
      .op(vshift3_op),
      .result(rshift_result[WIDTH*(3+1)-1:WIDTH*3])
      );

        

//TO DO: parameters

  //Shift Register for all multiplier results
  wire [4*WIDTH-1:0] shiftin_right_result_elmshifter_NC;
  wire [NUMLANES*WIDTH-1:0] inpipe_result_elmshifter_NC;
  wire shift_result_elmshifter;
  assign shift_result_elmshifter = ~done;

  velmshifter_1_128  result_elmshifter (
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

  assign result=(result_tmp<<((NUMLANES/4-1)*4*WIDTH)) |
                (result_buffered>>4*WIDTH);

  wire [2:1] dstpipe_squash_NC;
  wire [2-1:0] dstpipe_en;
  assign dstpipe_en = en[2:1] & {1'b1,~stall};

  pipe_10_2 dstpipe (
    .d(in_dst),  
    .clk(clk),
    .resetn(resetn),
    .en(dstpipe_en),
    .squash(dstpipe_squash_NC),
    .q(out_dst));

  wire [2-1:0] dstwepipe_en;
  assign dstwepipe_en = en[2:1] & {1'b1,~stall};

  pipe_1_2 dstwepipe (
    .d(in_dst_we),  
    .clk(clk),
    .resetn(resetn),
    .en(dstwepipe_en),
    .squash(squash[2:1]),
    .q(out_dst_we));

  wire [2-1:0] dstmaskpipe_en;
  assign dstmaskpipe_en = en[2:1] & {1'b1,~stall};

  wire [2:1] dstmaskpipe_squash_NC;
  pipe_4_2 dstmaskpipe (
    .d(vmask),  
    .clk(clk),
    .resetn(resetn),
    .en(dstmaskpipe_en),
    .squash(dstmaskpipe_squash_NC),
    .q(out_dst_mask));


endmodule
