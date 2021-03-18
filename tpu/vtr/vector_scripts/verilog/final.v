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
module local_add_sub_34_0_SIGNED(
dataa,
datab,
cin,
add_sub,
result
);

input[34-1:0] dataa;
input[34-1:0] datab;
input cin;
input add_sub;
output reg [34-1:0] result;

always @(*)begin
    if(add_sub == 1'b1)
         result = dataa+datab+cin;
    else
         result = dataa - datab;
end

endmodule
/****************************************************************************
          Saturate unit

  Takes a signed input and saturates it

  sat sign
   0    x   NOP (pass through)
   1    1   VS[ADD/SUB]
   1    0   VS[ADD/SUB]_U

parameter
  SATSUMOP_NOP=2'b00,
  SATSUMOP_VS =2'b11,
  SATSUMOP_VSU=2'b10;

****************************************************************************/


module vlane_saturatesum_32(
    in,
    op,
    out
    );

parameter WIDTH=32;

input [WIDTH+2-1:0] in;
input [1:0] op;
output [WIDTH-1:0] out;

reg [WIDTH-1:0] out;

wire op_saturate;
wire op_signed;

assign op_saturate=op[1];
assign op_signed=op[0];

wire [WIDTH-1:0] maxunsigned;
wire [WIDTH-1:0] minunsigned;
wire [WIDTH-1:0] maxsigned;
wire [WIDTH-1:0] minsigned;

assign maxunsigned = {WIDTH{1'b1}};
assign minunsigned = 0;
assign maxsigned = {1'b0,{WIDTH-1{1'b1}}};
assign minsigned = {1'b1,{WIDTH-1{1'b0}}};

wire [WIDTH-1:0] through;

assign through=in[WIDTH-1:0];

wire [2:0] top3bits=(op_saturate) ? in[WIDTH+2-1:WIDTH-1] : 3'b0 ;

  always@*
    case(top3bits)
      3'b010: out=maxunsigned;
      3'b011: out=maxunsigned;
      3'b001: out=(op_signed) ? maxsigned : through;
      3'b111: out=(op_signed) ? through : minunsigned;
      3'b110: out=(op_signed) ? minsigned : minunsigned;
      default: out=through;
    endcase

endmodule
/****************************************************************************
          Saturate unit for different widths given 32-bit input

  Only works for 32-bit inputs

  Interprets input as signed/unsigned, saturates to signed/unsigned 
  byte/half/word

  NOP (pass through) when size=word and signed==outsigned

   out
   sign sign size
    0    0    00    VSAT_U_W - NOP (pass through)
    0    0    01    VSAT_U_B
    0    0    10    VSAT_U_H
    1    1    00    VSAT_W - NOP (pass through)
    1    1    01    VSAT_B
    1    1    10    VSAT_H
    0    1    00    VSAT_SU_W
    0    1    01    VSAT_SU_B
    0    1    10    VSAT_SU_H

parameter
  SATSIZEOP_VSATUW=4'b0000,
  SATSIZEOP_VSATUB=4'b0001,
  SATSIZEOP_VSATUH=4'b0010,
  SATSIZEOP_VSATW  =4'b1100,
  SATSIZEOP_VSATB  =4'b1101,
  SATSIZEOP_VSATH  =4'b1110,
  SATSIZEOP_VSATSUW=4'b0100,
  SATSIZEOP_VSATSUB=4'b0101,
  SATSIZEOP_VSATSUH=4'b0110;

****************************************************************************/

module vlane_saturatesize(
    in,
    op,
    out
    );

parameter WIDTH=32;

input [WIDTH-1:0] in;
input [3:0] op;
output [WIDTH-1:0] out;

wire op_outsigned;
wire op_signed;
wire op_size;

reg [WIDTH-1:0] out;

  assign op_outsigned=op[3];
  assign op_signed=op[2];
  assign op_size=op[1:0]; //0 - word, 1 - byte, 2 - half

always@*
case(op_size)
    2'b01:  //byte
    case({op_signed,op_outsigned})
        2'b11:    // signed
        out = ((in[WIDTH-1])&(!(&in[WIDTH-2:7]))) ? {{WIDTH-8{1'b1}},8'h80} :
                ((~in[WIDTH-1]) && (|in[WIDTH-2:7])) ? 127 : in;
        2'b10:    // signed-unsigned
        out = (in[WIDTH-1]) ? 0 :
                (~in[WIDTH-1]&&(|in[WIDTH-2:8])) ? 255 : in;
        default:  //2'b00: unsigned
        out=(|in[WIDTH-1:8]) ? 255 : in;
        endcase
    2'b10:  //half-word 16-bits
    case({op_signed,op_outsigned})
        2'b11:    // signed
        out=((in[WIDTH-1])&(!(&in[WIDTH-2:15])))? {{WIDTH-16{1'b1}},16'h8000}:
                ((~in[WIDTH-1]) && (|in[WIDTH-2:15])) ? 32767 : in;
        2'b10:    // signed-unsigned
        out = (in[WIDTH-1]) ? 0 :
                (~in[WIDTH-1]&&(|in[WIDTH-2:16])) ? 65535 : in;
        default:  //2'b00: unsigned
        out=(|in[WIDTH-1:16]) ? 65535 : in;
        endcase
    default: 
    case({op_signed,op_outsigned})
        2'b10:    // signed-unsigned
        out = (in[WIDTH-1]) ? 0 : in;
        default:  //2'b00: unsigned
        out=in;
        endcase
endcase
  
endmodule
