
module vdispatcher_2_157_7_7_3(
    clk,
    resetn,

    shift,
    rotate, //shift must also be high to rotate

    inshift_instr,
    inshift_first,
    inshift_rdelm,
    inshift_wrelm,
    inshift_count,

    increment,
    rdelm_add_sub,
    wrelm_add_sub,
    count_add_sub,
    rdelm_valuetoadd,
    wrelm_valuetoadd,
    count_valuetoadd,

    instr,
    first,
    rdelm,
    wrelm,
    count

    );

input clk;
input resetn;

input  shift;
input  rotate;

input [ 157-1:0 ] inshift_instr;
input                    inshift_first;
input [ 7-1:0 ] inshift_rdelm;
input [ 7-1:0 ] inshift_wrelm;
input [ 3-1:0 ] inshift_count;

input [ 2-1:0 ] increment;

input rdelm_add_sub;
input wrelm_add_sub;
input count_add_sub;
input [ 7-1:0 ] rdelm_valuetoadd;
input [ 7-1:0 ] wrelm_valuetoadd;
input [ 3-1:0 ] count_valuetoadd;

output [ 2*157-1:0 ] instr;
output            [ 2-1:0 ] first;
output [ 2*7-1:0 ] rdelm;
output [ 2*7-1:0 ] wrelm;
output [ 2*3-1:0 ] count;

wire [2*157-1:0 ] inparallel_data_inst_NC;
vdispatcher_shift_2_157 vdispatcher_instr (
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .squash({2{1'b0}}),
      .inshift_data(inshift_instr),
      .inparallel_data(inparallel_data_inst_NC),
      .outparallel_data(instr));

wire [ 2-1:0 ] inparallel_data_first_NC;
vdispatcher_shift_2_1 vdispatcher_first (
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .squash(increment&~{32'b0,shift&~rotate}),
      .inshift_data(inshift_first),
      .inparallel_data(inparallel_data_first_NC),
      .outparallel_data(first));

wire [ 2*7-1:0 ] inparallel_data_rdelm_NC;
wire [ 2-1:0 ] squash_rdelm_NC;
vdispatcher_add_2_7 vdispatcher_rdelm(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_rdelm_NC),
      .add_sub(rdelm_add_sub),
      .valuetoadd(rdelm_valuetoadd),
      .inshift_data(inshift_rdelm),
      .inparallel_data(inparallel_data_rdelm_NC),
      .outparallel_data(rdelm));

wire [ 2*7-1:0 ] inparallel_data_wrelm_NC;
wire [ 2-1:0 ] squash_wrelm_NC;
vdispatcher_add_2_7 vdispatcher_wrelm(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_wrelm_NC),
      .add_sub(wrelm_add_sub),
      .valuetoadd(wrelm_valuetoadd),
      .inshift_data(inshift_wrelm),
      .inparallel_data(inparallel_data_wrelm_NC),
      .outparallel_data(wrelm));

wire [ 2*3-1:0 ] inparallel_data_count_NC;
wire [ 2-1:0 ] squash_count_NC;
vdispatcher_add_2_3 vdispatcher_count(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_count_NC),
      .add_sub(count_add_sub),
      .valuetoadd(count_valuetoadd),
      .inshift_data(inshift_count),
      .inparallel_data(inparallel_data_count_NC),
      .outparallel_data(count));


endmodule
