/****************************************************************************

  ALU Circuit:

         |    _     flags
 src1 ---0---| \    |
             >+|----+----|Sat|------|M src1 --|M\--|-/         |sum|      |U|---
        |U|                       +-|X/
 src2 --|X/                       |    1
                                  |
      src2 ------------|M\        |
 src1 ----------|&|    |U|--|Sat|-+
                |||----|X/  |siz|
 src2 ---0------|^|      0
         |


              ADD SUB   CMP    CMP.U & | ^ ~| MIN MAX ABS MRG
              S U S U = ! < <= < <=           S U S U

adderopA_sel  0 0 0 0 0 0 0 0  0 0   x x x x  0 0 0 0  1   1
adderopB_sel  0 0 0 0 0 0 0 0  0 0   x x x x  0 0 0 0  1   0
logicopB_sel  x x x x x x x x  x x   0 0 0 0  1 1 1 1  1   1
signed        1 0 1 0 1 1 1 1  0 0   x x x x  1 0 1 0  1   x
addsub        0 0 1 1 1 1 1 1  1 1   x x x x  1 1 1 1  1   0
max           x x x x x x x x  x x   0 0 0 0  0 0 1 1  0   0
min           x x x x x x x x  x x   0 0 0 0  1 1 0 0  0   0
logic[1:0]    x x x x x x x x  x x   1 0 2 3  0 0 0 0  0   0
mux1_sel[1:0] 0 0 0 0 x x x x  x x   1 1 1 1  1 1 1 1  2   3 -- Combine
flag_sel[1:0] x x x x 0 1 2 3  2 3   x x x x  x x x x  x   x /

ALUOP_ZERO    =11'bzz1zzzz0101
ALUOP_ADD     =11'b00z10zzzz00
ALUOP_ADDU    =11'b00z00zzzz00
ALUOP_SUB     =11'b00z11zzzz00
ALUOP_SUBU    =11'b00z01zzzz00
ALUOP_CMP_EQ  =11'b00z11zzzz00
ALUOP_CMP_NEQ =11'b00z11zzzz01
ALUOP_CMP_LT  =11'b00z11zzzz10
ALUOP_CMP_LE  =11'b00z11zzzz11
ALUOP_CMP_LT_U=11'b00z01zzzz10
ALUOP_CMP_LE_U=11'b00z01zzzz11
ALUOP_AND     =11'bzz0zz000101
ALUOP_OR      =11'bzz0zz000001
ALUOP_XOR     =11'bzz0zz001001
ALUOP_NOR     =11'bzz0zz001101
ALUOP_MIN     =11'b00111010001
ALUOP_MIN_U   =11'b00101010001
ALUOP_MAX     =11'b00111100001
ALUOP_MAX_U   =11'b00101100001
ALUOP_ABS     =11'b11111000010
ALUOP_MERGE   =11'b101z0000011

****************************************************************************/

//`include "vlane_saturate.v"

module vlane_alu_32(
    clk,
    resetn,

    pipe_en,
    pipe_squashn,

    src1,
    src2,
    mask,

    op,
    satsum_op,
    satsize_op,

    cmp_result,
    result

    );

parameter WIDTH=32;

input clk;
input resetn;

input pipe_en;
input pipe_squashn;

input  [WIDTH-1:0] src1;
input  [WIDTH-1:0] src2;

input  mask;

input  [10:0] op;
input  [1:0] satsum_op;
input  [3:0] satsize_op;

output cmp_result;
output [WIDTH-1:0] result;

wire [WIDTH-1:0] adder_opA;
wire [WIDTH-1:0] adder_opB;
wire [WIDTH+2-1:0] adder_result;

wire [WIDTH-1:0] logic_opA;
wire [WIDTH-1:0] logic_opB;
reg  [WIDTH-1:0] logic_result;

wire [WIDTH-1:0] mux0_result;
wire [WIDTH-1:0] mux1_result;

wire [WIDTH-1:0] satsum_result;
wire [WIDTH-1:0] satsize_result;

wire lt;
wire neq;
wire eq;
wire le;

wire ctrl_adderopA_sel;
wire ctrl_adderopB_sel;
wire ctrl_logicopB_sel;
wire ctrl_signed;
wire ctrl_addsub;
wire ctrl_max;
wire ctrl_min;
wire [1:0] ctrl_logic;
wire [1:0] ctrl_mux1_sel;
wire [1:0] ctrl_flag_sel;

  assign ctrl_adderopA_sel=op[10];
  assign ctrl_adderopB_sel=op[9];
  assign ctrl_logicopB_sel=op[8];
  assign ctrl_signed=op[7];
  assign ctrl_addsub=~op[6];  //Make add 0, sub 1
  assign ctrl_max=op[5];
  assign ctrl_min=op[4];
  assign ctrl_logic=op[3:2];
  assign ctrl_mux1_sel=op[1:0];
  assign ctrl_flag_sel=op[1:0];

  assign adder_opA=(ctrl_adderopA_sel) ? 0 : src1;
  assign adder_opB=(ctrl_adderopB_sel) ? src1 : src2;

`ifndef USE_INHOUSE_LOGIC
    `define USE_INHOUSE_LOGIC
`endif

  `ifdef USE_INHOUSE_LOGIC
  wire [(WIDTH+2)-1:0] dataa;
  wire [(WIDTH+2)-1:0] datab;
  wire cin;

  assign dataa = {{2{ctrl_signed&adder_opA[WIDTH-1]}},adder_opA};
  assign datab = {{2{ctrl_signed&adder_opB[WIDTH-1]}},adder_opB};
  assign cin = ~ctrl_addsub;

  local_add_sub_34_0_SIGNED local_adder_inst(
      .dataa(dataa),
      .datab(datab),
      .cin(cin),
      .add_sub(ctrl_addsub),
      .result(adder_result)
  );
  `else
  lpm_add_sub adder_inst(
      .dataa({{2{ctrl_signed&adder_opA[WIDTH-1]}},adder_opA}),
      .datab({{2{ctrl_signed&adder_opB[WIDTH-1]}},adder_opB}),
      .cin(~ctrl_addsub),
      .add_sub(ctrl_addsub),
      .result(adder_result)
          // synopsys translate_off
          , .cout (), .overflow (), .clken (), .clock (), .aclr ()
          // synopsys translate_on
      );
  defparam
      adder_inst.lpm_width=WIDTH+2,
      adder_inst.lpm_pipeline=0,
      adder_inst.lpm_representation="SIGNED";
  `endif

  assign lt=adder_result[WIDTH];
  assign neq=|adder_result;
  assign eq=~neq;
  assign le=lt||eq;

  assign cmp_result= (ctrl_flag_sel==0) ? eq :
                     (ctrl_flag_sel==1) ? neq :
                     (ctrl_flag_sel==2) ? lt : le;

  assign logic_opA=src1;
  assign logic_opB=(ctrl_logicopB_sel) ? 0 : src2;

  always@*
    case(ctrl_logic)
        2'b00:
            logic_result=logic_opA|logic_opB;
        2'b01:
            logic_result=logic_opA&logic_opB;
        2'b10:
            logic_result=logic_opA^logic_opB;
        2'b11:
            logic_result=~(logic_opA|logic_opB);
    endcase

  assign mux0_result=((lt && ctrl_max) || (~lt && ctrl_min)) ?  src2 : 
                                                                logic_result;

  /************* PIPELINE at this point ***************/
  reg [WIDTH-1:0] mux0_result_s2;
  reg [WIDTH+2-1:0] adder_result_s2;
  reg  [3:0] satsize_op_s2;
  reg  [3:0] satsum_op_s2;
  reg ctrl_mux1_sel_s2;

  always@(posedge clk)
  begin
    if (!resetn || !pipe_squashn)
    begin
      mux0_result_s2<=0;
      adder_result_s2<=0;
      satsize_op_s2<=0;
      satsum_op_s2<=0;
      ctrl_mux1_sel_s2<=0;
    end
    else if (pipe_en)
    begin
      mux0_result_s2<=mux0_result;
      adder_result_s2<=adder_result;
      satsize_op_s2<=satsize_op;
      satsum_op_s2<=satsum_op;
      ctrl_mux1_sel_s2<=((ctrl_mux1_sel==0) ||
                         (ctrl_mux1_sel==2 && src1[WIDTH-1]) ||
                         (ctrl_mux1_sel==3 && mask) );
    end
  end

  vlane_saturatesize satsize(   //Unit only works for 32-bit inputs
      .in(mux0_result_s2),
      .op(satsize_op_s2),
      .out(satsize_result)
      );

  vlane_saturatesum_32 satsum(
      .in(adder_result_s2),
      .op(satsum_op_s2),
      .out(satsum_result)
      );

  assign mux1_result= (ctrl_mux1_sel_s2) ? 
                              satsum_result : satsize_result;

  assign result=mux1_result;

endmodule