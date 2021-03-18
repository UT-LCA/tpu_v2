\ 
`include "vlane_mulshift.v"
`include "vlane_barrelshifter.v"

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

module vmul_unit_5_8_3_8 (clk, resetn,
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

parameter NUMLANES=2**3;
parameter WIDTH=2**5;

input clk;
input resetn;

input [NUMLANES*WIDTH-1:0] opA;
input [NUMLANES*WIDTH-1:0] opB;
input [((5==0) ? 1 : 5)-1:0] vshamt;  // Fixed point rounding
input [NUMLANES-1:0] vmask;
input [4:0] op;
input       activate;
input [3:1] en;  //Enable for each pipestage
input [3:1] squash;  //Squash for each pipestage

input    [8-1:0] in_dst;
input                     in_dst_we;
output [3*8-1:0] out_dst;
output              [2:0] out_dst_we;
output   [3*NUMLANES-1:0] out_dst_mask;

output stall;
output [NUMLANES*WIDTH-1:0] result;

  /********* Circuit Body *********/
  wire [8*WIDTH-1:0] mul_opA;
  wire [8*WIDTH-1:0] mul_opB;
  wire [8*WIDTH-1:0] mul_result;
  wire [8*WIDTH-1:0] rshift_result;
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
  wire [((5==0) ? 1*3 : 5*3)-1:0] ctrl_vshamt; //3 pipe stages

  //Shift Register for all multiplier operands for lanes without multipliers
  wire [WIDTH*8-1:0] opA_elmshifter_shiftin_right_NC;
 
 //TODO: update the parameters
  velmshifter_1_256  opA_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*8{1'b0}}),
    .shiftin_right(opA_elmshifter_shiftin_right_NC), 
    .inpipe(opA),
    .outpipe(opA_buffered)
  );

  wire [WIDTH*8-1:0] opB_elmshifter_shiftin_right_NC;

  //TODO: update the parameters
  velmshifter_1_256 opB_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*8{1'b0}}),
    .shiftin_right(opB_elmshifter_shiftin_right_NC), 
    .inpipe(opB),
    .outpipe(opB_buffered)
  );

  wire [8-1:0] mask_elmshifter_shiftin_right_NC;

  //TODO: Update the parameters 
  velmshifter_1_8  mask_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done & ctrl_activate[1]),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({8{1'b0}}),
    .shiftin_right(mask_elmshifter_shiftin_right_NC), 
    //.inpipe(vmask), //DISABLE - always do all multiplications
    .inpipe({NUMLANES{1'b1}}),
    .outpipe(mask_buffered)
  );

  //Shift Register for all multiplier operands w/ parallel load
  always@(posedge clk)
  begin
    if (!resetn || 8==NUMLANES)
      done<=1;
    else if (done && ctrl_activate[1] && en[1])
      //done<=~(|(vmask>>8)); // multiply only if mask - DISABLED
      done<=~(|(vmask));
    else
      done<=~( |(mask_buffered >> (2*8) ));
  end

  assign mul_opA=(done) ? opA : (opA_buffered >> 8*WIDTH);

  assign mul_opB=(done) ? opB : (opB_buffered >> 8*WIDTH);

  assign stall=~done && (ctrl_activate[2]);

  wire [3:1] oppipe_squash_NC; 
  pipe_5_2 oppipe (
    .d(op),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(oppipe_squash_NC),
    .q({ctrl_op[3],ctrl_op[2],ctrl_op[1]}));

  wire [3:1] activatepipe_squash_NC; 
  pipe_1_2  activatepipe (
    .d(activate),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(activatepipe_squash_NC),
    .q({ctrl_activate[3],ctrl_activate[2],ctrl_activate[1]}));

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
    vlane_mulshift_32_5  vmul0(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*0 +: WIDTH]),
      .opB(mul_opB[WIDTH*0 +: WIDTH]),
      .sa( mul_opB[WIDTH*0+((5>0)?5-1:0) : WIDTH*0] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*0 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift0(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(0+1)-1:WIDTH*0]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(0+1)-1:WIDTH*0])
      );

        

    vlane_mulshift_32_5  vmul1(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*1 +: WIDTH]),
      .opB(mul_opB[WIDTH*1 +: WIDTH]),
      .sa( mul_opB[WIDTH*1+((5>0)?5-1:0) : WIDTH*1] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*1 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift1(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(1+1)-1:WIDTH*1]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(1+1)-1:WIDTH*1])
      );

        

    vlane_mulshift_32_5  vmul2(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*2 +: WIDTH]),
      .opB(mul_opB[WIDTH*2 +: WIDTH]),
      .sa( mul_opB[WIDTH*2+((5>0)?5-1:0) : WIDTH*2] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*2 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift2(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(2+1)-1:WIDTH*2]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(2+1)-1:WIDTH*2])
      );

        

    vlane_mulshift_32_5  vmul3(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*3 +: WIDTH]),
      .opB(mul_opB[WIDTH*3 +: WIDTH]),
      .sa( mul_opB[WIDTH*3+((5>0)?5-1:0) : WIDTH*3] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*3 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift3(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(3+1)-1:WIDTH*3]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(3+1)-1:WIDTH*3])
      );

        

    vlane_mulshift_32_5  vmul4(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*4 +: WIDTH]),
      .opB(mul_opB[WIDTH*4 +: WIDTH]),
      .sa( mul_opB[WIDTH*4+((5>0)?5-1:0) : WIDTH*4] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*4 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift4(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(4+1)-1:WIDTH*4]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(4+1)-1:WIDTH*4])
      );

        

    vlane_mulshift_32_5  vmul5(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*5 +: WIDTH]),
      .opB(mul_opB[WIDTH*5 +: WIDTH]),
      .sa( mul_opB[WIDTH*5+((5>0)?5-1:0) : WIDTH*5] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*5 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift5(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(5+1)-1:WIDTH*5]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(5+1)-1:WIDTH*5])
      );

        

    vlane_mulshift_32_5  vmul6(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*6 +: WIDTH]),
      .opB(mul_opB[WIDTH*6 +: WIDTH]),
      .sa( mul_opB[WIDTH*6+((5>0)?5-1:0) : WIDTH*6] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*6 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift6(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(6+1)-1:WIDTH*6]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(6+1)-1:WIDTH*6])
      );

        

    vlane_mulshift_32_5  vmul7(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*7 +: WIDTH]),
      .opB(mul_opB[WIDTH*7 +: WIDTH]),
      .sa( mul_opB[WIDTH*7+((5>0)?5-1:0) : WIDTH*7] ),
      .op( (done&en[1]) ? ctrl_op[1] : ctrl_op[2]),
      .result(mul_result[WIDTH*7 +: WIDTH])
      );

    wire temp;
    assign temp = ctrl_vshamt[2*5-1 : 5-1];

//TO DO: parameters

    vlane_barrelshifter_32_5 vshift7(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(7+1)-1:WIDTH*7]),
      .sa(temp),
      .op({~ctrl_op[2][1] ,1'b1}),
      .result(rshift_result[WIDTH*(7+1)-1:WIDTH*7])
      );

        

//TO DO: parameters

  //Shift Register for all multiplier results
  wire [8*WIDTH-1:0] shiftin_right_result_elmshifter_NC;
  wire [NUMLANES*WIDTH-1:0] inpipe_result_elmshifter_NC;
  velmshifter_1_256  result_elmshifter (
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

  assign result=(result_tmp<<((NUMLANES/8-1)*8*WIDTH)) |
                (result_buffered>>8*WIDTH);

  wire [2:1] dstpipe_squash_NC;
  pipe_8_2 dstpipe (
    .d( in_dst ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(dstpipe_squash_NC),
    .q(out_dst));

  pipe_1_2 dstwepipe (
    .d( in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(squash[2:1]),
    .q(out_dst_we));

  wire [2:1] dstmaskpipe_squash_NC;
  pipe_8_2 dstmaskpipe (
    .d( vmask ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(dstmaskpipe_squash_NC),
    .q(out_dst_mask));


endmodule
