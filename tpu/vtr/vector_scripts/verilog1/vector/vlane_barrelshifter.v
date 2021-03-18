/****************************************************************************
          Shifter unit

Opcode Table:

sign_ext dir 
 0        0    |  ShiftLeft
 0        1    |  ShiftRightLogic
 1        1    |  ShiftRightArith
          
****************************************************************************/
module vlane_barrelshifter(clk, resetn,
            opB, sa, 
            op, 
            result);
parameter 1=32;
parameter 8=5;

//Shifts the first 2 bits in one cycle, the rest in the next cycle
parameter (8-2)=8-2;

input clk;
input resetn;

input [1-1:0] opB;
input [8-1:0] sa;                             // Shift Amount
input [2-1:0] op;

output [1-1:0] result;


wire sign_ext;
wire shift_direction;
assign sign_ext=op[1];
assign shift_direction=op[0];

wire dum,dum_,dum2;
wire [1-1:0] partial_result_,partial_result;

`ifdef USE_INHOUSE_LOGIC
 local_shifter_2_2_ARITHMATIC local_shifter_inst1(
  .data({sign_ext&opB[1-1],opB}),
  .distance(sa&(32'hffffffff<<(((8-2)>0) ? (8-2) : 0))),
  .direction(shift_direction),
  .result(dum,partial_result)
 );
 defparam
    local_shifter_inst1.LPM_WIDTH = 1+1,
    local_shifter_inst1.LPM_WIDTHDIST = 8,
    local_shifter_inst1.LPM_SHIFTTYPE="ARITHMETIC";
`else
lpm_clshift shifter_inst1(
    .data({sign_ext&opB[1-1],opB}),
    .distance(sa&(32'hffffffff<<(((8-2)>0) ? (8-2) : 0))),
    .direction(shift_direction),
    .result(dum,partial_result));
 defparam
    shifter_inst1.lpm_width = 1+1,
    shifter_inst1.lpm_widthdist = 8,
    shifter_inst1.lpm_shifttype="ARITHMETIC";
`endif

register_2 partial_reg
  ({dum,partial_result},clk,resetn,1'b1,{dum_,partial_result_});

wire [5-1:0] sa_2;
wire shift_direction_2;

register_5 secondstage (sa, clk,resetn,1'b1,sa_2); 

register_1 secondstagedir (shift_direction, clk,resetn,1'b1,shift_direction_2); 

`ifdef USE_INHOUSE_LOGIC
 local_shifter_2_2_ARITHMATIC local_shifter_inst2(
  .data({dum_,partial_result_}),
  .distance(sa_2[(((8-2)>0) ? (8-2)-1 : 0):0]),
  .direction(shift_direction_2),
  .result({dum2,result})
 );
 defparam
    local_shifter_inst2.LPM_WIDTH = 1+1,
    local_shifter_inst2.LPM_WIDTHDIST = ((8-2)>0) ? (8-2) : 1,
    local_shifter_inst2.LPM_SHIFTTYPE ="ARITHMETIC";
`else
lpm_clshift_2_2_ARITHMATIC shifter_inst2(
    .data({dum_,partial_result_}),
    .distance(sa_2[(((8-2)>0) ? (8-2)-1 : 0):0]),
    .direction(shift_direction_2),
    .result({dum2,resulti}));
 defparam 
    shifter_inst2.lpm_width = 1+1,
    shifter_inst2.lpm_widthdist = ((8-2)>0) ? (8-2) : 1,
    shifter_inst2.lpm_shifttype="ARITHMETIC";
`endif


endmodule

        /****************************************************************************
          Shifter unit

Opcode Table:

sign_ext dir 
 0        0    |  ShiftLeft
 0        1    |  ShiftRightLogic
 1        1    |  ShiftRightArith
          
****************************************************************************/
module vlane_barrelshifter(clk, resetn,
            opB, sa, 
            op, 
            result);
parameter 1=32;
parameter 8=5;

//Shifts the first 2 bits in one cycle, the rest in the next cycle
parameter (8-2)=8-2;

input clk;
input resetn;

input [1-1:0] opB;
input [8-1:0] sa;                             // Shift Amount
input [2-1:0] op;

output [1-1:0] result;


wire sign_ext;
wire shift_direction;
assign sign_ext=op[1];
assign shift_direction=op[0];

wire dum,dum_,dum2;
wire [1-1:0] partial_result_,partial_result;

`ifdef USE_INHOUSE_LOGIC
 local_shifter_2_2_ARITHMATIC local_shifter_inst1(
  .data({sign_ext&opB[1-1],opB}),
  .distance(sa&(32'hffffffff<<(((8-2)>0) ? (8-2) : 0))),
  .direction(shift_direction),
  .result(dum,partial_result)
 );
 defparam
    local_shifter_inst1.LPM_WIDTH = 1+1,
    local_shifter_inst1.LPM_WIDTHDIST = 8,
    local_shifter_inst1.LPM_SHIFTTYPE="ARITHMETIC";
`else
lpm_clshift shifter_inst1(
    .data({sign_ext&opB[1-1],opB}),
    .distance(sa&(32'hffffffff<<(((8-2)>0) ? (8-2) : 0))),
    .direction(shift_direction),
    .result(dum,partial_result));
 defparam
    shifter_inst1.lpm_width = 1+1,
    shifter_inst1.lpm_widthdist = 8,
    shifter_inst1.lpm_shifttype="ARITHMETIC";
`endif

register_2 partial_reg
  ({dum,partial_result},clk,resetn,1'b1,{dum_,partial_result_});

wire [5-1:0] sa_2;
wire shift_direction_2;

register_5 secondstage (sa, clk,resetn,1'b1,sa_2); 

register_1 secondstagedir (shift_direction, clk,resetn,1'b1,shift_direction_2); 

`ifdef USE_INHOUSE_LOGIC
 local_shifter_2_2_ARITHMATIC local_shifter_inst2(
  .data({dum_,partial_result_}),
  .distance(sa_2[(((8-2)>0) ? (8-2)-1 : 0):0]),
  .direction(shift_direction_2),
  .result({dum2,result})
 );
 defparam
    local_shifter_inst2.LPM_WIDTH = 1+1,
    local_shifter_inst2.LPM_WIDTHDIST = ((8-2)>0) ? (8-2) : 1,
    local_shifter_inst2.LPM_SHIFTTYPE ="ARITHMETIC";
`else
lpm_clshift_2_2_ARITHMATIC shifter_inst2(
    .data({dum_,partial_result_}),
    .distance(sa_2[(((8-2)>0) ? (8-2)-1 : 0):0]),
    .direction(shift_direction_2),
    .result({dum2,resulti}));
 defparam 
    shifter_inst2.lpm_width = 1+1,
    shifter_inst2.lpm_widthdist = ((8-2)>0) ? (8-2) : 1,
    shifter_inst2.lpm_shifttype="ARITHMETIC";
`endif


endmodule

        