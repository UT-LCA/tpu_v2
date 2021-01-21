/****************************************************************************
          Shifter unit

Opcode Table:

sign_ext dir 
 0        0    |  ShiftLeft
 0        1    |  ShiftRightLogic
 1        1    |  ShiftRightArith
          
****************************************************************************/
module vbitshifter(
    clk, 
    resetn,
    opB, 
    sa, 
    op, 
    start, 
    stalled,
    result);
parameter WIDTH=32;

input clk;
input resetn;

input [WIDTH-1:0] opB;
input [4:0] sa;                             // Shift Amount
input [2-1:0] op;

input start;
output stalled;

output [WIDTH-1:0] result;
//output carry_out;


wire shift_direction;
assign shift_direction=op[0];

reg [WIDTH-1:0] shifter;
reg shift_state;
reg [4:0] shift_count;

wire hi_bit, lo_bit;

assign hi_bit=shifter[0];
assign lo_bit=shifter[WIDTH-1];

assign stalled = (shift_state);
assign result=shifter;

always @(posedge clk or negedge resetn)
begin
    if (!resetn)
    begin
        shifter<=0;
        shift_state<=0;
        shift_count<=0;
    end
    else
    begin
        if (shift_state==0)
        begin
                if (start)
                begin
                    shift_count<=sa;
                    shifter<=opB;
                    if (|sa)
                        shift_state<=1;
                end
        end
        else
            begin
                if (shift_count==1)
                    shift_state<=0;
                shift_count<=shift_count-1;
                shifter[31]<=(shift_direction) ? hi_bit : shifter[30];
                shifter[30]<=(shift_direction) ? shifter[31] : shifter[29];
                shifter[29]<=(shift_direction) ? shifter[30] : shifter[28];
                shifter[28]<=(shift_direction) ? shifter[29] : shifter[27];
                shifter[27]<=(shift_direction) ? shifter[28] : shifter[26];
                shifter[26]<=(shift_direction) ? shifter[27] : shifter[25];
                shifter[25]<=(shift_direction) ? shifter[26] : shifter[24];
                shifter[24]<=(shift_direction) ? shifter[25] : shifter[23];
                shifter[23]<=(shift_direction) ? shifter[24] : shifter[22];
                shifter[22]<=(shift_direction) ? shifter[23] : shifter[21];
                shifter[21]<=(shift_direction) ? shifter[22] : shifter[20];
                shifter[20]<=(shift_direction) ? shifter[21] : shifter[19];
                shifter[19]<=(shift_direction) ? shifter[20] : shifter[18];
                shifter[18]<=(shift_direction) ? shifter[19] : shifter[17];
                shifter[17]<=(shift_direction) ? shifter[18] : shifter[16];
                shifter[16]<=(shift_direction) ? shifter[17] : shifter[15];
                shifter[15]<=(shift_direction) ? shifter[16] : shifter[14];
                shifter[14]<=(shift_direction) ? shifter[15] : shifter[13];
                shifter[13]<=(shift_direction) ? shifter[14] : shifter[12];
                shifter[12]<=(shift_direction) ? shifter[13] : shifter[11];
                shifter[11]<=(shift_direction) ? shifter[12] : shifter[10];
                shifter[10]<=(shift_direction) ? shifter[11] : shifter[9];
                shifter[9]<=(shift_direction) ? shifter[10] : shifter[8];
                shifter[8]<=(shift_direction) ? shifter[9] : shifter[7];
                shifter[7]<=(shift_direction) ? shifter[8] : shifter[6];
                shifter[6]<=(shift_direction) ? shifter[7] : shifter[5];
                shifter[5]<=(shift_direction) ? shifter[6] : shifter[4];
                shifter[4]<=(shift_direction) ? shifter[5] : shifter[3];
                shifter[3]<=(shift_direction) ? shifter[4] : shifter[2];
                shifter[2]<=(shift_direction) ? shifter[3] : shifter[1];
                shifter[1]<=(shift_direction) ? shifter[2] : shifter[0];
                shifter[0]<=(shift_direction) ? shifter[1] : lo_bit;
            end
    end
end


/*
lpm_clshift shifter_inst(
    .data({sign_ext&opB[WIDTH-1],opB}),
    .distance(sa),
    .direction(shift_direction),
    .result(shifted_result));
defparam 
    shifter_inst.lpm_width = WIDTH+1,
    shifter_inst.lpm_widthdist = 5,
    shifter_inst.lpm_shifttype="ARITHMETIC";
*/

endmodule

