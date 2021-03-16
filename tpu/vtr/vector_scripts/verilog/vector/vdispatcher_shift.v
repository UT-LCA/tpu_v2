
module vdispatcher_shift (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    squash,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

parameter 2=4;      
parameter 151=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ 2-1:0 ]  squash;

input [ 151-1:0 ] inshift_data;

input [ 2*151-1:0 ]  inparallel_data;
output [ 2*151-1:0 ] outparallel_data;

wire [ 151-1:0 ]  shiftin_left;
wire [ 151-1:0 ]  shiftin_right;


  assign shiftin_right = (!rotate) ? inshift_data : 
                      outparallel_data[2*151-1:(2-1)*151];

  assign shiftin_left = (!rotate) ? 0 : outparallel_data[151-1:0];

  velmshifter_2_151 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load),
      .shift(shift),
      .dir_left(1),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe(inparallel_data),
      .outpipe(outparallel_data));


endmodule


module vdispatcher_shift (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    squash,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

parameter 2=4;      
parameter 1=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ 2-1:0 ]  squash;

input [ 1-1:0 ] inshift_data;

input [ 2*1-1:0 ]  inparallel_data;
output [ 2*1-1:0 ] outparallel_data;

wire [ 1-1:0 ]  shiftin_left;
wire [ 1-1:0 ]  shiftin_right;


  assign shiftin_right = (!rotate) ? inshift_data : 
                      outparallel_data[2*1-1:(2-1)*1];

  assign shiftin_left = (!rotate) ? 0 : outparallel_data[1-1:0];

  velmshifter_2_1 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load),
      .shift(shift),
      .dir_left(1),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe(inparallel_data),
      .outpipe(outparallel_data));


endmodule

