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

        