
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_5_96 (
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
input [ 5-1:0 ]  squash;

input [ 5*96-1:0 ]  inpipe;
output [ 5*96-1:0 ] outpipe;

input [ 96-1:0 ]  shiftin_left;
input [ 96-1:0 ]  shiftin_right;

wire [ (5+1)*96-1:0 ] _outpipe;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_96 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[95:0],
      _outpipe[191:96], //Support 1 lane
      shiftin_right,
      _outpipe[95:0]);
 // defparam velmshifter_laneunit0.96=96;

  //Generate everything in between 


      velmshifter_laneunit_96 velmshifter_laneunit_1(clk,resetn,load,shift,dir_left,
          squash[1],
          inpipe[191:96],
          _outpipe[287:192],
          _outpipe[95:0],
          _outpipe[191:96]);
     // defparam velmshifter_laneunit.96=96;


      velmshifter_laneunit_96 velmshifter_laneunit_2(clk,resetn,load,shift,dir_left,
          squash[2],
          inpipe[191:96],
          _outpipe[287:192],
          _outpipe[95:0],
          _outpipe[191:96]);
     // defparam velmshifter_laneunit.96=96;


      velmshifter_laneunit_96 velmshifter_laneunit_3(clk,resetn,load,shift,dir_left,
          squash[3],
          inpipe[191:96],
          _outpipe[287:192],
          _outpipe[95:0],
          _outpipe[191:96]);
     // defparam velmshifter_laneunit.96=96;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_96 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[4],
      inpipe[479:384],
      shiftin_left,
      _outpipe[383:288], //L=1
      _outpipe[479:384]); //L=1
   // defparam velmshifter_laneunitlast.96=96;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_5_3 (
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
input [ 5-1:0 ]  squash;

input [ 5*3-1:0 ]  inpipe;
output [ 5*3-1:0 ] outpipe;

input [ 3-1:0 ]  shiftin_left;
input [ 3-1:0 ]  shiftin_right;

wire [ (5+1)*3-1:0 ] _outpipe;


/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_3 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[2:0],
      _outpipe[5:3], //Support 1 lane
      shiftin_right,
      _outpipe[2:0]);
 // defparam velmshifter_laneunit0.3=3;

  //Generate everything in between 


      velmshifter_laneunit_3 velmshifter_laneunit_1(clk,resetn,load,shift,dir_left,
          squash[1],
          inpipe[5:3],
          _outpipe[8:6],
          _outpipe[2:0],
          _outpipe[5:3]);
     // defparam velmshifter_laneunit.3=3;


      velmshifter_laneunit_3 velmshifter_laneunit_2(clk,resetn,load,shift,dir_left,
          squash[2],
          inpipe[5:3],
          _outpipe[8:6],
          _outpipe[2:0],
          _outpipe[5:3]);
     // defparam velmshifter_laneunit.3=3;


      velmshifter_laneunit_3 velmshifter_laneunit_3(clk,resetn,load,shift,dir_left,
          squash[3],
          inpipe[5:3],
          _outpipe[8:6],
          _outpipe[2:0],
          _outpipe[5:3]);
     // defparam velmshifter_laneunit.3=3;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_3 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[4],
      inpipe[14:12],
      shiftin_left,
      _outpipe[11:9], //L=1
      _outpipe[14:12]); //L=1
   // defparam velmshifter_laneunitlast.3=3;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        