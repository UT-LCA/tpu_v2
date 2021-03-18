

module vdispatcher_add (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    increment,
    squash,

    add_sub,
    valuetoadd,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

parameter 2=4;      
parameter 6=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ 2-1:0 ]  increment;
input [ 2-1:0 ]  squash;

input               add_sub;
input [ 6-1:0 ] valuetoadd;

input [ 6-1:0 ] inshift_data;

input [ 2*6-1:0 ]  inparallel_data;
output [ 2*6-1:0 ] outparallel_data;

wire [ 6-1:0 ]  shiftin_right;

reg [ 2*6-1:0 ] outparallel_data;
reg [ 2*6-1:0 ] outparallel_added;


  assign shiftin_right = (!rotate) ? inshift_data : 
                    outparallel_added[2*6-1:(2-1)*6];

  reg [ 6-1:0 ] v;
  integer i;
  always@*
    for (i=0; i<2; i=i+1)
    begin
      //Saturate at 0xffff... in either direction
      v=(increment[i] && outparallel_data[i*6 +: 6] != {6{1'b1}}) ?
        valuetoadd : 0;
      if (add_sub==0)
        outparallel_added[i*6 +: 6]=outparallel_data[i*6+:6] + v;
      else
        outparallel_added[i*6 +: 6]=outparallel_data[i*6+:6] - v;
    end

  always@(posedge clk)
    if (!resetn)
      outparallel_data=0;
    else if (load)
      outparallel_data=inparallel_data;
    else if (shift)
      outparallel_data=(outparallel_added<<6) | shiftin_right;
      


endmodule



module vdispatcher_add (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    increment,
    squash,

    add_sub,
    valuetoadd,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

parameter 2=4;      
parameter 6=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ 2-1:0 ]  increment;
input [ 2-1:0 ]  squash;

input               add_sub;
input [ 6-1:0 ] valuetoadd;

input [ 6-1:0 ] inshift_data;

input [ 2*6-1:0 ]  inparallel_data;
output [ 2*6-1:0 ] outparallel_data;

wire [ 6-1:0 ]  shiftin_right;

reg [ 2*6-1:0 ] outparallel_data;
reg [ 2*6-1:0 ] outparallel_added;


  assign shiftin_right = (!rotate) ? inshift_data : 
                    outparallel_added[2*6-1:(2-1)*6];

  reg [ 6-1:0 ] v;
  integer i;
  always@*
    for (i=0; i<2; i=i+1)
    begin
      //Saturate at 0xffff... in either direction
      v=(increment[i] && outparallel_data[i*6 +: 6] != {6{1'b1}}) ?
        valuetoadd : 0;
      if (add_sub==0)
        outparallel_added[i*6 +: 6]=outparallel_data[i*6+:6] + v;
      else
        outparallel_added[i*6 +: 6]=outparallel_data[i*6+:6] - v;
    end

  always@(posedge clk)
    if (!resetn)
      outparallel_data=0;
    else if (load)
      outparallel_data=inparallel_data;
    else if (shift)
      outparallel_data=(outparallel_added<<6) | shiftin_right;
      


endmodule



module vdispatcher_add (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    increment,
    squash,

    add_sub,
    valuetoadd,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

parameter 2=4;      
parameter 4=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ 2-1:0 ]  increment;
input [ 2-1:0 ]  squash;

input               add_sub;
input [ 4-1:0 ] valuetoadd;

input [ 4-1:0 ] inshift_data;

input [ 2*4-1:0 ]  inparallel_data;
output [ 2*4-1:0 ] outparallel_data;

wire [ 4-1:0 ]  shiftin_right;

reg [ 2*4-1:0 ] outparallel_data;
reg [ 2*4-1:0 ] outparallel_added;


  assign shiftin_right = (!rotate) ? inshift_data : 
                    outparallel_added[2*4-1:(2-1)*4];

  reg [ 4-1:0 ] v;
  integer i;
  always@*
    for (i=0; i<2; i=i+1)
    begin
      //Saturate at 0xffff... in either direction
      v=(increment[i] && outparallel_data[i*4 +: 4] != {4{1'b1}}) ?
        valuetoadd : 0;
      if (add_sub==0)
        outparallel_added[i*4 +: 4]=outparallel_data[i*4+:4] + v;
      else
        outparallel_added[i*4 +: 4]=outparallel_data[i*4+:4] - v;
    end

  always@(posedge clk)
    if (!resetn)
      outparallel_data=0;
    else if (load)
      outparallel_data=inparallel_data;
    else if (shift)
      outparallel_data=(outparallel_added<<4) | shiftin_right;
      


endmodule

