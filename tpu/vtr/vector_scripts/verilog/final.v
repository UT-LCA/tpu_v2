/****************************************************************************
          Shifter unit

Opcode Table:

sign_ext dir 
 0        0    |  ShiftLeft
 0        1    |  ShiftRightLogic
 1        1    |  ShiftRightArith
          
****************************************************************************/
module vlane_barrelshifter_32_5(clk, resetn,
            opB, sa, 
            op, 
            result);
//parameter 32=32;
//parameter 5=5;

//Shifts the first 2 bits in one cycle, the rest in the next cycle
//parameter (5-2)=5-2;

input clk;
input resetn;

input [32-1:0] opB;
input [5-1:0] sa;                             // Shift Amount
input [2-1:0] op;

output [32-1:0] result;


wire sign_ext;
wire shift_direction;
assign sign_ext=op[1];
assign shift_direction=op[0];

wire dum,dum_,dum2;
wire [32-1:0] partial_result_,partial_result;
`ifndef USE_INHOUSE_LOGIC
    `define USE_INHOUSE_LOGIC
`endif

`ifdef USE_INHOUSE_LOGIC
wire [33-1:0] local_shifter_inst1_result;
assign {dum,partial_result} = local_shifter_inst1_result;

wire [2-1:0] local_shifter_inst1_distance;
assign local_shifter_inst1_distance = sa&(32'hffffffff<<(((5-2)>0) ? (5-2) : 0));

wire [33-1:0] local_shifter_inst1_data;
assign local_shifter_inst1_data = {sign_ext&opB[32-1],opB};

local_shifter_33_2_ARITHMATIC local_shifter_inst1(
  .data(local_shifter_inst1_data),
  .distance(local_shifter_inst1_distance),
  .direction(shift_direction),
  .result(local_shifter_inst1_result)
);
 //defparam
 //   local_shifter_inst1.LPM_WIDTH = 32+1,
 //   local_shifter_inst1.LPM_WIDTHDIST = 5,
 //   local_shifter_inst1.LPM_SHIFTTYPE="ARITHMETIC";
`else
lpm_clshift shifter_inst1(
    .data({sign_ext&opB[32-1],opB}),
    .distance(sa&(32'hffffffff<<(((5-2)>0) ? (5-2) : 0))),
    .direction(shift_direction),
    .result(dum,partial_result));
 defparam
    shifter_inst1.lpm_width = 32+1,
    shifter_inst1.lpm_widthdist = 5,
    shifter_inst1.lpm_shifttype="ARITHMETIC";
`endif

wire [33-1:0] partial_reg_q;
assign partial_reg_q = {dum_,partial_result_};
register_33 partial_reg
  ({dum,partial_result},clk,resetn,1'b1,partial_reg_q);

wire [5-1:0] sa_2;
wire shift_direction_2;

register_5 secondstage (sa, clk,resetn,1'b1,sa_2); 

register_1 secondstagedir (shift_direction, clk,resetn,1'b1,shift_direction_2); 

`ifdef USE_INHOUSE_LOGIC
wire [33-1:0] local_shifter_inst2_result;
assign {dum2,result} = local_shifter_inst2_result;

wire [2-1:0] local_shifter_inst2_distance;
assign local_shifter_inst2_distance = sa_2[(5-2)-1:0];

wire [33-1:0] local_shifter_inst2_data;
assign local_shifter_inst2_data = {dum_,partial_result_};

local_shifter_33_2_ARITHMATIC local_shifter_inst2(
  .data(local_shifter_inst2_data),
  .distance(local_shifter_inst2_distance),
  .direction(shift_direction_2),
  .result(local_shifter_inst2_result)
);
// defparam
//    local_shifter_inst2.LPM_WIDTH = 32+1,
//   local_shifter_inst2.LPM_WIDTHDIST = ((5-2)>0) ? (5-2) : 1,
//    local_shifter_inst2.LPM_SHIFTTYPE ="ARITHMETIC";
`else
lpm_clshift_33_2_ARITHMATIC shifter_inst2(
    .data({dum_,partial_result_}),
    .distance(sa_2[(((5-2)>0) ? (5-2)-1 : 0):0]),
    .direction(shift_direction_2),
    .result({dum2,resulti}));
 defparam 
    shifter_inst2.lpm_width = 32+1,
    shifter_inst2.lpm_widthdist = ((5-2)>0) ? (5-2) : 1,
    shifter_inst2.lpm_shifttype="ARITHMETIC";
`endif


endmodule

        
module local_shifter_33_2_ARITHMATIC(
  data,
  distance,
  direction,
  result
);

parameter LPM_SHIFTTYPE = "ARITHMATIC";

input [33-1:0] data;
input [2-1:0] distance;
input direction;

output reg [33-1:0] result;
reg [33-1:0] arith_reg; 
always @* begin
  arith_reg = {33{1'b1}};
    if (LPM_SHIFTTYPE == "ARITHMETIC") begin
    if(direction)begin
      result = data << distance;
    end
    else begin
      if(data[33-1] == 1'b1)
          result =  ((arith_reg <<(33 - distance))|| (data >> distance));
      else
          result = data >> distance;
    end
  end
  else begin
    if(direction == 1'b0)begin
        result = data << distance;
    end
    else begin
        result = data >> distance;
    end
  end 
end
endmodule
/****************************************************************************
          Generic Register
****************************************************************************/
module register_33(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [33-1:0] d;
output [33-1:0] q;
reg [33-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule
/****************************************************************************
          Generic Register
****************************************************************************/
module register_5(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [5-1:0] d;
output [5-1:0] q;
reg [5-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule
/****************************************************************************
          Generic Register
****************************************************************************/
module register_1(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [1-1:0] d;
output [1-1:0] q;
reg [1-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule

module velmshifter_laneunit_96 (
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

input [ 96-1:0 ]  inpipe;
input [ 96-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 96-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 96-1:0 ] outpipe;

reg [ 96-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        
\ 
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

module vmul_unit_5_3_4_4 (clk, resetn,
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

parameter NUMLANES=2**4;
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

input    [4-1:0] in_dst;
input                     in_dst_we;
output [3*4-1:0] out_dst;
output              [2:0] out_dst_we;
output   [3*NUMLANES-1:0] out_dst_mask;

output stall;
output [NUMLANES*WIDTH-1:0] result;

  /********* Circuit Body *********/
  wire [3*WIDTH-1:0] mul_opA;
  wire [3*WIDTH-1:0] mul_opB;
  wire [3*WIDTH-1:0] mul_result;
  wire [3*WIDTH-1:0] rshift_result;
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
  wire [WIDTH*3-1:0] opA_elmshifter_shiftin_right_NC;
 
 //TODO: update the parameters
  velmshifter_5_96  opA_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*3{1'b0}}),
    .shiftin_right(opA_elmshifter_shiftin_right_NC), 
    .inpipe(opA),
    .outpipe(opA_buffered)
  );

  wire [WIDTH*3-1:0] opB_elmshifter_shiftin_right_NC;

  //TODO: update the parameters
  velmshifter_5_96 opB_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*3{1'b0}}),
    .shiftin_right(opB_elmshifter_shiftin_right_NC), 
    .inpipe(opB),
    .outpipe(opB_buffered)
  );

  wire [3-1:0] mask_elmshifter_shiftin_right_NC;
  wire mask_elmshifter_load;

  assign mask_elmshifter_load = done & ctrl_activate[1];

  //TODO: Update the parameters 
  velmshifter_5_3  mask_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(mask_elmshifter_load),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({3{1'b0}}),
    .shiftin_right(mask_elmshifter_shiftin_right_NC), 
    //.inpipe(vmask), //DISABLE - always do all multiplications
    .inpipe({NUMLANES{1'b1}}),
    .outpipe(mask_buffered)
  );

  //Shift Register for all multiplier operands w/ parallel load
  always@(posedge clk)
  begin
    if (!resetn || 3==NUMLANES)
      done<=1;
    else if (done && ctrl_activate[1] && en[1])
      //done<=~(|(vmask>>3)); // multiply only if mask - DISABLED
      done<=~(|(vmask));
    else
      done<=~( |(mask_buffered >> (2*3) ));
  end

  assign mul_opA=(done) ? opA : (opA_buffered >> 3*WIDTH);

  assign mul_opB=(done) ? opB : (opB_buffered >> 3*WIDTH);

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

        

//TO DO: parameters

  //Shift Register for all multiplier results
  wire [3*WIDTH-1:0] shiftin_right_result_elmshifter_NC;
  wire [NUMLANES*WIDTH-1:0] inpipe_result_elmshifter_NC;
  wire shift_result_elmshifter;
  assign shift_result_elmshifter = ~done;

  velmshifter_5_96  result_elmshifter (
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

  assign result=(result_tmp<<((NUMLANES/3-1)*3*WIDTH)) |
                (result_buffered>>3*WIDTH);

  wire [2:1] dstpipe_squash_NC;
  wire [2-1:0] dstpipe_en;
  assign dstpipe_en = en[2:1] & {1'b1,~stall};

  pipe_4_2 dstpipe (
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
  pipe_16_2 dstmaskpipe (
    .d(vmask),  
    .clk(clk),
    .resetn(resetn),
    .en(dstmaskpipe_en),
    .squash(dstmaskpipe_squash_NC),
    .q(out_dst_mask));


endmodule

/****************************************************************************
          MUL unit

Operation table

     half sat  op unsign dir
2      0   0   0   0    0    |  Zero (This value is used to reset multiplier)
14     1   0   1   1    0    |  LMULU Lower MULtiply Unsigned
15     1   0   1   1    1    |  UMULU Upper MULtiply Unsigned
16     1   0   1   0    0    |  LMUL Lower MULtiply signed
17     1   0   1   0    1    |  UMUL Upper MULtiply signed
4      0   0   1   1    0    |  MULLOU
4      0   0   1   1    1    |  MULHIU
6      0   0   1   0    0    |  MULLO
6      0   0   1   0    1    |  MULHI
0      0   0   0   1    0    |  ShiftLeft
8      0   1   0   1    0    |  ShiftLeftSatU
10     0   1   0   0    0    |  ShiftLeftSat
1      0   0   0   1    1    |  ShiftRightLogic
3      0   0   0   0    1    |  ShiftRightArith
****************************************************************************/
module vlane_mulshift_32_5(clk, resetn,
            opA, opB, sa,
            op,
            en,
            result
            );
parameter WIDTH=32;
parameter LOG2WIDTH=5;

input clk;
input resetn;

input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
input [LOG2WIDTH-1:0] sa;
input [4:0] op;
input [3:1] en;  //Enable for each pipestage

output [WIDTH-1:0] result;

/********* Control Signals *********/
wire is_signed, dir, is_mul, saturate, half;
assign is_mul=op[2];      // selects between opB and the computed shift amount
assign is_signed=~op[1];
assign dir=op[0];         // selects between 2^sa and 2^(32-sa) for right shift
assign saturate=op[3];
assign half=op[4];

/********* Circuit Body *********/
wire dum,dum2,dum3;
wire [WIDTH:0] opB_mux_out;
wire [WIDTH-1:0] opA_mux_out;
wire [WIDTH-1:0] opB_mul;
wire [5-1:0] left_sa;     // Amount of left shift required for both left/right
reg [WIDTH:0] decoded_sa;
wire [WIDTH-1:0] hi;
wire [WIDTH-1:0] lo;

assign opA_mux_out = (~half) ? ( opA ) : (WIDTH<2) ? 0 : (dir) ?
              {{WIDTH/2{is_signed&is_mul&opA[WIDTH-1]}},opA[WIDTH-1:WIDTH/2]} :
              {{WIDTH/2{is_signed&is_mul&opA[WIDTH/2-1]}},opA[WIDTH/2-1:0]};

assign opB_mul = (~half) ? opB : (WIDTH<2) ? 0 : (dir) ? 
              {{WIDTH/2{is_signed&is_mul&opB[WIDTH-1]}},opB[WIDTH-1:WIDTH/2]} :
              {{WIDTH/2{is_signed&is_mul&opB[WIDTH/2-1]}},opB[WIDTH/2-1:0]};

assign opB_mux_out=(is_mul) ? {is_signed&opB_mul[WIDTH-1],opB_mul} : decoded_sa;

reg zeroout;

`ifndef USE_INHOUSE_LOGIC
  `define USE_INHOUSE_LOGIC
`endif

always@(posedge clk)
  if (en[1])
    zeroout<=(op[3:0]==0);

`ifdef USE_INHOUSE_LOGIC
wire [(WIDTH+1)-1:0] dataa;
wire aclr;
wire [66-1:0] local_mult_component_result;

assign dataa = {is_signed&opA_mux_out[WIDTH-1],opA_mux_out};
assign aclr = ~resetn;
assign {dum2,dum,hi,lo} = local_mult_component_result;

local_mult_33_33_66 local_mult_component (
.dataa(dataa),
.datab(opB_mux_out),
.clock(clk),
.clken(en[1]),
.aclr(aclr),
.result(local_mult_component_result)
);
`else 
lpm_mult  lpm_mult_component (
  .dataa ({is_signed&opA_mux_out[WIDTH-1],opA_mux_out}),
  .datab (opB_mux_out),
  .sum(),
  .clock(clk),
  .clken(en[1]),
  .aclr(~resetn),
  .result ({dum2,dum,hi,lo}));
defparam
  lpm_mult_component.lpm_widtha = WIDTH+1,
  lpm_mult_component.lpm_widthb = WIDTH+1,
  lpm_mult_component.lpm_widthp = 2*WIDTH+2,
  lpm_mult_component.lpm_widths = 1,
  lpm_mult_component.lpm_pipeline = 1,
  lpm_mult_component.lpm_type = "LPM_MULT",
  lpm_mult_component.lpm_representation = "SIGNED",
  lpm_mult_component.lpm_hint = "MAXIMIZE_SPEED=6";
`endif
// if A is positive/negative make it maximum/minimum positive/negative
wire [WIDTH-1:0] signedsaturate=
                    (opA_mux_out[WIDTH-1]) ? {1'b1,{WIDTH-1{1'b0}}} : 
                                             {1'b0,{WIDTH-1{1'b1}}};

reg [WIDTH-1:0] saturatedval_s2;
reg saturate_s2;

//Capture saturated value and saturate signal for next stage
always@(posedge clk)
  if (!resetn)
  begin
    saturatedval_s2<=0;
    saturate_s2<=0;
  end
  else if (en[1])
  begin
    saturatedval_s2<=((~is_signed) ? {WIDTH{1'b1}} : signedsaturate);
    saturate_s2<=saturate;
  end

reg sel_hi;

always@(posedge clk)
  if (!resetn)
    sel_hi<=0;
  else if (en[1])
    sel_hi<=(is_mul && dir || ~is_mul && dir && |sa);

assign result =(zeroout) ? 0 : 
                (saturate_s2 && |hi ) ? saturatedval_s2 : 
                (sel_hi) ? hi : lo;


assign {dum3, left_sa}= (dir) ? WIDTH-sa : {1'b0,sa};

//Decoder - computes 2^left_sa
always@*
begin
    decoded_sa=1<<left_sa;
end
endmodule
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_4_2(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [4-1:0]  d;
input              clk;
input              resetn;
input  [2-1:0] en;
input  [2-1:0] squash;
output [4*(2+1)-1:0] q;

reg [4*2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 4-1:0 ]<= 0;
    else if (en[0])
      tq[ 4-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i*4 +: 4 ]<= 0;
      else if (en[i])
        tq[i*4 +: 4 ]<=tq[(i-1)*4 +: 4 ];
  end

  assign q[4-1:0]=d;
  assign q[4*(2+1)-1:4]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_1_2(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input  d;
input              clk;
input              resetn;
input  [2-1:0] en;
input  [2-1:0] squash;
output [(2+1)-1:0] q;

reg [2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq <= 0;
    else if (en[0])
      tq <=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i]<= 0;
      else if (en[i])
        tq[i]<=tq[(i-1)];
  end

  assign q[0] =d;
  assign q[(2+1)-1:1]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_5_2(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [5-1:0]  d;
input              clk;
input              resetn;
input  [2-1:0] en;
input  [2-1:0] squash;
output [5*(2+1)-1:0] q;

reg [5*2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 5-1:0 ]<= 0;
    else if (en[0])
      tq[ 5-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i*5 +: 5 ]<= 0;
      else if (en[i])
        tq[i*5 +: 5 ]<=tq[(i-1)*5 +: 5 ];
  end

  assign q[5-1:0]=d;
  assign q[5*(2+1)-1:5]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_16_2(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [16-1:0]  d;
input              clk;
input              resetn;
input  [2-1:0] en;
input  [2-1:0] squash;
output [16*(2+1)-1:0] q;

reg [16*2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 16-1:0 ]<= 0;
    else if (en[0])
      tq[ 16-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i*16 +: 16 ]<= 0;
      else if (en[i])
        tq[i*16 +: 16 ]<=tq[(i-1)*16 +: 16 ];
  end

  assign q[16-1:0]=d;
  assign q[16*(2+1)-1:16]=tq;
endmodule
        
module local_mult_33_33_66(
dataa,
datab,
clock,
clken,
aclr,
result
);

input [33-1:0] dataa;
input [33-1:0] datab;
input clock;
input clken;
input aclr;
output reg [66-1:0] result;

wire [33-1:0] unsignedinputA;
wire [33-1:0] unsignedinputB;
wire [66-1:0] unsignedoutputP;

wire gated_clock;

assign unsignedinputA = dataa;
assign unsignedinputB = datab;

assign unsignedoutputP = unsignedinputA * unsignedinputB;

assign gated_clock = clock & clken;

always @(posedge gated_clock)begin
    if(aclr)begin
       result <= 0;
    end
    else
       result <= unsignedoutputP; 
end
endmodule

/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_5_96 (
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
input [ 5-1:0 ]  squash;

input [ 5*96-1:0 ]  inpipe;
output [ 5*96-1:0 ] outpipe;

input [ 96-1:0 ]  shiftin_left;
input [ 96-1:0 ]  shiftin_right;

wire [ (5+1)*96-1:0 ] _outpipe;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_96 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[95:0],
      _outpipe[191:96], //Support 1 lane
      shiftin_right,
      _outpipe[95:0]);
 // defparam velmshifter_laneunit0.96=96;

  //Generate everything in between 


      velmshifter_laneunit_96 velmshifter_laneunit_1(clk,resetn,load,shift,dir_left,
          squash[1],
          inpipe[191:96],
          _outpipe[287:192],
          _outpipe[95:0],
          _outpipe[191:96]);
     // defparam velmshifter_laneunit.96=96;


      velmshifter_laneunit_96 velmshifter_laneunit_2(clk,resetn,load,shift,dir_left,
          squash[2],
          inpipe[191:96],
          _outpipe[287:192],
          _outpipe[95:0],
          _outpipe[191:96]);
     // defparam velmshifter_laneunit.96=96;


      velmshifter_laneunit_96 velmshifter_laneunit_3(clk,resetn,load,shift,dir_left,
          squash[3],
          inpipe[191:96],
          _outpipe[287:192],
          _outpipe[95:0],
          _outpipe[191:96]);
     // defparam velmshifter_laneunit.96=96;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_96 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[4],
      inpipe[479:384],
      shiftin_left,
      _outpipe[383:288], //L=1
      _outpipe[479:384]); //L=1
   // defparam velmshifter_laneunitlast.96=96;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_5_3 (
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
input [ 5-1:0 ]  squash;

input [ 5*3-1:0 ]  inpipe;
output [ 5*3-1:0 ] outpipe;

input [ 3-1:0 ]  shiftin_left;
input [ 3-1:0 ]  shiftin_right;

wire [ (5+1)*3-1:0 ] _outpipe;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_3 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[2:0],
      _outpipe[5:3], //Support 1 lane
      shiftin_right,
      _outpipe[2:0]);
 // defparam velmshifter_laneunit0.3=3;

  //Generate everything in between 


      velmshifter_laneunit_3 velmshifter_laneunit_1(clk,resetn,load,shift,dir_left,
          squash[1],
          inpipe[5:3],
          _outpipe[8:6],
          _outpipe[2:0],
          _outpipe[5:3]);
     // defparam velmshifter_laneunit.3=3;


      velmshifter_laneunit_3 velmshifter_laneunit_2(clk,resetn,load,shift,dir_left,
          squash[2],
          inpipe[5:3],
          _outpipe[8:6],
          _outpipe[2:0],
          _outpipe[5:3]);
     // defparam velmshifter_laneunit.3=3;


      velmshifter_laneunit_3 velmshifter_laneunit_3(clk,resetn,load,shift,dir_left,
          squash[3],
          inpipe[5:3],
          _outpipe[8:6],
          _outpipe[2:0],
          _outpipe[5:3]);
     // defparam velmshifter_laneunit.3=3;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_3 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[4],
      inpipe[14:12],
      shiftin_left,
      _outpipe[11:9], //L=1
      _outpipe[14:12]); //L=1
   // defparam velmshifter_laneunitlast.3=3;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
module velmshifter_laneunit_3 (
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

input [ 3-1:0 ]  inpipe;
input [ 3-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 3-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 3-1:0 ] outpipe;

reg [ 3-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        
