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
parameter WIDTH=32;
parameter LOG2WIDTH=5;

//Shifts the first 2 bits in one cycle, the rest in the next cycle
parameter REGISTERBREAK=LOG2WIDTH-2;

input clk;
input resetn;

input [WIDTH-1:0] opB;
input [LOG2WIDTH-1:0] sa;                             // Shift Amount
input [2-1:0] op;

output [WIDTH-1:0] result;


wire sign_ext;
wire shift_direction;
assign sign_ext=op[1];
assign shift_direction=op[0];

wire dum,dum_,dum2;
wire [WIDTH-1:0] partial_result_,partial_result;

`ifdef USE_INHOUSE_LOGIC
 local_shifter local_shifter_inst1(
  .data(),
  .distance(),
  .direction(),
  .result()
 );
 defparam
    local_shifter_inst1.lpm_width = WIDTH+1,
    local_shifter_inst1.lpm_widthdist = LOG2WIDTH,
    local_shifter_inst1.lpm_shifttype="ARITHMETIC";
`else
lpm_clshift shifter_inst1(
    .data({sign_ext&opB[WIDTH-1],opB}),
    .distance(sa&(32'hffffffff<<((REGISTERBREAK>0) ? REGISTERBREAK : 0))),
    .direction(shift_direction),
    .result({dum,partial_result}));
 defparam
    shifter_inst1.lpm_width = WIDTH+1,
    shifter_inst1.lpm_widthdist = LOG2WIDTH,
    shifter_inst1.lpm_shifttype="ARITHMETIC";
`endif

register partial_reg
  ({dum,partial_result},clk,resetn,1'b1,{dum_,partial_result_});
    defparam partial_reg.WIDTH=WIDTH+1;

wire [5-1:0] sa_2;
wire shift_direction_2;

register secondstage (sa, clk,resetn,1'b1,sa_2); 
  defparam secondstage.WIDTH=5;

register secondstagedir (shift_direction, clk,resetn,1'b1,shift_direction_2); 
  defparam secondstagedir.WIDTH=1;

`ifdef USE_INHOUSE_LOGIC
 local_shifter local_shifter_shift2(
  .data(),
  .distance(),
  .direction(),
  .result()
 );
 defparam
    local_shifter_inst1.lpm_width = WIDTH+1,
    local_shifter_inst1.lpm_widthdist = LOG2WIDTH,
    local_shifter_inst1.lpm_shifttype="ARITHMETIC";
`else
lpm_clshift shifter_inst2(
    .data({dum_,partial_result_}),
    .distance(sa_2[((REGISTERBREAK>0) ? REGISTERBREAK-1 : 0):0]),
    .direction(shift_direction_2),
    .result({dum2,result}));
 defparam 
    shifter_inst2.lpm_width = WIDTH+1,
    shifter_inst2.lpm_widthdist = (REGISTERBREAK>0) ? REGISTERBREAK : 1,
    shifter_inst2.lpm_shifttype="ARITHMETIC";
`endif


endmodule

