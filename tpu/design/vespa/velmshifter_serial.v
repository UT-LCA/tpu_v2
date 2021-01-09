/************************
 * An Inter-lane shift register
 *
 * This is much more scalable than a giant barrel shifter/crossbar, however
 * it is also slower in cycles.
 *
 *  dir_left:  1=left, 0=right
 *
 * Synthesis results for NUMLANES=4, WIDTH=32:
 *    131 LEs (Stratix I), 422 MHz
 *************************/

/***************************************************************************
  shiftin_left -> bn-1 <-> bn-2 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

module velmrotator (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    rotate,       // Sets whether ends feedback into beginning
    squash,

    inpipe,
    outpipe

    );

parameter NUMLANES=4;      
parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  rotate;
input [ NUMLANES-1:0 ]  squash;

input [ NUMLANES*WIDTH-1:0 ]  inpipe;
output [ NUMLANES*WIDTH-1:0 ] outpipe;

wire [ WIDTH-1:0 ]  shiftin_left;
wire [ WIDTH-1:0 ]  shiftin_right;


  assign shiftin_right = (!rotate) ? 0 : 
                        outpipe[NUMLANES*WIDTH-1:(NUMLANES-1)*WIDTH];

  assign shiftin_left = (!rotate) ? 0 : outpipe[WIDTH-1:0];

  velmshifter #(NUMLANES,WIDTH) velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load),
      .shift(shift),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe(inpipe),
      .outpipe(outpipe));


endmodule


/******************************************************************************/
/**************************** Shifter w Jump **********************************/
/******************************************************************************/

module velmshifter_jump (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    jump,
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe
    );

parameter NUMLANES=4;      
parameter JUMPSIZE=4;     // We can either shift by 1, or jump by this amount 
parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  jump;              //0-shift by 1, 1 shift by JUMPSIZE
input [ NUMLANES-1:0 ]  squash;

input [ NUMLANES*WIDTH-1:0 ]  inpipe;
output [ NUMLANES*WIDTH-1:0 ] outpipe;

input [ WIDTH-1:0 ]  shiftin_left;
input [ WIDTH-1:0 ]  shiftin_right;

genvar i;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter #(NUMLANES,WIDTH) velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<(WIDTH*JUMPSIZE) :
                      outpipe >>(WIDTH*JUMPSIZE)),
      .outpipe(outpipe));

endmodule

/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

parameter NUMLANES=4;      
parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ NUMLANES-1:0 ]  squash;

input [ NUMLANES*WIDTH-1:0 ]  inpipe;
output [ NUMLANES*WIDTH-1:0 ] outpipe;

input [ WIDTH-1:0 ]  shiftin_left;
input [ WIDTH-1:0 ]  shiftin_right;

wire [ (NUMLANES+1)*WIDTH-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[WIDTH-1:0],
      _outpipe[((NUMLANES>1) ? WIDTH : 0)+:WIDTH], //Support 1 lane
      shiftin_right,
      _outpipe[WIDTH-1:0]);
  defparam velmshifter_laneunit0.WIDTH=WIDTH;

  //Generate everything in between 

  generate
    for (i=1; i<NUMLANES-1; i=i+1)
    begin : elmshift
      velmshifter_laneunit velmshifter_laneunit(clk,resetn,load,shift,dir_left,
          squash[i],
          inpipe[(i+1)*WIDTH-1:i*WIDTH],
          _outpipe[(i+2)*WIDTH-1:(i+1)*WIDTH],
          _outpipe[(i)*WIDTH-1:(i-1)*WIDTH],
          _outpipe[(i+1)*WIDTH-1:i*WIDTH]);
      defparam velmshifter_laneunit.WIDTH=WIDTH;
    end
  endgenerate

  //HANDLE lane NUMLANE specially

    velmshifter_laneunit velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[NUMLANES-1],
      inpipe[NUMLANES*WIDTH-1:(NUMLANES-1)*WIDTH],
      shiftin_left,
      _outpipe[(((NUMLANES>1) ? NUMLANES:2)-2)*WIDTH +: WIDTH], //L=1
      _outpipe[((NUMLANES>1) ? (NUMLANES-1)*WIDTH : WIDTH) +: WIDTH]); //L=1
    defparam velmshifter_laneunitlast.WIDTH=WIDTH;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule



module velmshifter_laneunit (
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

parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ WIDTH-1:0 ]  inpipe;
input [ WIDTH-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ WIDTH-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ WIDTH-1:0 ] outpipe;

reg [ WIDTH-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule

