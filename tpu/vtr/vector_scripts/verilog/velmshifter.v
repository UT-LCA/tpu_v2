
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_4_32 (
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

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 4-1:0 ]  squash;

input [ 4*32-1:0 ]  inpipe;
output [ 4*32-1:0 ] outpipe;

input [ 32-1:0 ]  shiftin_left;
input [ 32-1:0 ]  shiftin_right;

wire [ (4+1)*32-1:0 ] _outpipe;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_32 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[31:0],
      _outpipe[63:32], //Support 1 lane
      shiftin_right,
      _outpipe[31:0]);
 // defparam velmshifter_laneunit0.32=32;

  //Generate everything in between 


      velmshifter_laneunit_32 velmshifter_laneunit_1(clk,resetn,load,shift,dir_left,
          squash[1],
          inpipe[63:32],
          _outpipe[95:64],
          _outpipe[31:0],
          _outpipe[63:32]);
     // defparam velmshifter_laneunit.32=32;


      velmshifter_laneunit_32 velmshifter_laneunit_2(clk,resetn,load,shift,dir_left,
          squash[2],
          inpipe[63:32],
          _outpipe[95:64],
          _outpipe[31:0],
          _outpipe[63:32]);
     // defparam velmshifter_laneunit.32=32;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_32 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[3],
      inpipe[127:96],
      shiftin_left,
      _outpipe[95:64], //L=1
      _outpipe[127:96]); //L=1
   // defparam velmshifter_laneunitlast.32=32;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_4_1 (
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

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 4-1:0 ]  squash;

input [ 4*1-1:0 ]  inpipe;
output [ 4*1-1:0 ] outpipe;

input [ 1-1:0 ]  shiftin_left;
input [ 1-1:0 ]  shiftin_right;

wire [ (4+1)*1-1:0 ] _outpipe;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_1 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[0],
      _outpipe[1], //Support 1 lane
      shiftin_right,
      _outpipe[0]);
 // defparam velmshifter_laneunit0.1=1;

  //Generate everything in between 


      velmshifter_laneunit_1 velmshifter_laneunit_1(clk,resetn,load,shift,dir_left,
          squash[1],
          inpipe[1],
          _outpipe[2],
          _outpipe[0],
          _outpipe[1]);
     // defparam velmshifter_laneunit.1=1;


      velmshifter_laneunit_1 velmshifter_laneunit_2(clk,resetn,load,shift,dir_left,
          squash[2],
          inpipe[1],
          _outpipe[2],
          _outpipe[0],
          _outpipe[1]);
     // defparam velmshifter_laneunit.1=1;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_1 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[3],
      inpipe[3],
      shiftin_left,
      _outpipe[2], //L=1
      _outpipe[3]); //L=1
   // defparam velmshifter_laneunitlast.1=1;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        