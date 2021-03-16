
/******************************************************************************/
/**************************** Shifter w Jump **********************************/
/******************************************************************************/

module velmshifter_jump_32_8_8 (
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

parameter 8=4;      
parameter 8=4;     // We can either shift by 1, or jump by this amount 
parameter 32=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  jump;              //0-shift by 1, 1 shift by 8
input [ 8-1:0 ]  squash;

input [ 8*32-1:0 ]  inpipe;
output [ 8*32-1:0 ] outpipe;

input [ 32-1:0 ]  shiftin_left;
input [ 32-1:0 ]  shiftin_right;

genvar i;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter_8_32 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<(32*8) :
                      outpipe >>(32*8)),
      .outpipe(outpipe));

endmodule
        
/******************************************************************************/
/**************************** Shifter w Jump **********************************/
/******************************************************************************/

module velmshifter_jump_32_8_8 (
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

parameter 8=4;      
parameter 8=4;     // We can either shift by 1, or jump by this amount 
parameter 32=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  jump;              //0-shift by 1, 1 shift by 8
input [ 8-1:0 ]  squash;

input [ 8*32-1:0 ]  inpipe;
output [ 8*32-1:0 ] outpipe;

input [ 32-1:0 ]  shiftin_left;
input [ 32-1:0 ]  shiftin_right;

genvar i;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter_8_32 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<(32*8) :
                      outpipe >>(32*8)),
      .outpipe(outpipe));

endmodule
        
/******************************************************************************/
/**************************** Shifter w Jump **********************************/
/******************************************************************************/

module velmshifter_jump_1_8_8 (
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

parameter 8=4;      
parameter 8=4;     // We can either shift by 1, or jump by this amount 
parameter 1=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  jump;              //0-shift by 1, 1 shift by 8
input [ 8-1:0 ]  squash;

input [ 8*1-1:0 ]  inpipe;
output [ 8*1-1:0 ] outpipe;

input [ 1-1:0 ]  shiftin_left;
input [ 1-1:0 ]  shiftin_right;

genvar i;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter_8_1 velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<(1*8) :
                      outpipe >>(1*8)),
      .outpipe(outpipe));

endmodule
        