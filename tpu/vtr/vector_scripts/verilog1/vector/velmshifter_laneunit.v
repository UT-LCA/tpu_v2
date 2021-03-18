module velmshifter_laneunit_151 (
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

parameter 151=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 151-1:0 ]  inpipe;
input [ 151-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 151-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 151-1:0 ] outpipe;

reg [ 151-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        module velmshifter_laneunit_1 (
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

parameter 1=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 1-1:0 ]  inpipe;
input [ 1-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 1-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 1-1:0 ] outpipe;

reg [ 1-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        module velmshifter_laneunit_1 (
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

parameter 1=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 1-1:0 ]  inpipe;
input [ 1-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 1-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 1-1:0 ] outpipe;

reg [ 1-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        module velmshifter_laneunit_256 (
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

parameter 256=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 256-1:0 ]  inpipe;
input [ 256-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 256-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 256-1:0 ] outpipe;

reg [ 256-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        module velmshifter_laneunit_8 (
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

parameter 8=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 8-1:0 ]  inpipe;
input [ 8-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 8-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 8-1:0 ] outpipe;

reg [ 8-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        module velmshifter_laneunit_256 (
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

parameter 256=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 256-1:0 ]  inpipe;
input [ 256-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 256-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 256-1:0 ] outpipe;

reg [ 256-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        module velmshifter_laneunit_8 (
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

parameter 8=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ 8-1:0 ]  inpipe;
input [ 8-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ 8-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ 8-1:0 ] outpipe;

reg [ 8-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule
        