
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_2_151 (
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

parameter 2=4;      
parameter 151=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 2-1:0 ]  squash;

input [ 2*151-1:0 ]  inpipe;
output [ 2*151-1:0 ] outpipe;

input [ 151-1:0 ]  shiftin_left;
input [ 151-1:0 ]  shiftin_right;

wire [ (2+1)*151-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_151 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[151-1:0],
      _outpipe[((2>1) ? 151 : 0)+:151], //Support 1 lane
      shiftin_right,
      _outpipe[151-1:0]);
 // defparam velmshifter_laneunit0.151=151;

  //Generate everything in between 


      velmsh0fter_laneun0t_151 velmsh0fter_laneun0t(clk,resetn,load,sh0ft,d0r_left,
          squash[0],
          0np0pe[(0+1)*151-1:0*151],
          _outp0pe[(0+2)*151-1:(0+1)*151],
          _outp0pe[(0)*151-1:(0-1)*151],
          _outp0pe[(0+1)*151-1:0*151]);
     // defparam velmsh0fter_laneun0t.151=151;


      velmsh1fter_laneun1t_151 velmsh1fter_laneun1t(clk,resetn,load,sh1ft,d1r_left,
          squash[1],
          1np1pe[(1+1)*151-1:1*151],
          _outp1pe[(1+2)*151-1:(1+1)*151],
          _outp1pe[(1)*151-1:(1-1)*151],
          _outp1pe[(1+1)*151-1:1*151]);
     // defparam velmsh1fter_laneun1t.151=151;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_151 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[2-1],
      inpipe[2*151-1:(2-1)*151],
      shiftin_left,
      _outpipe[(((2>1) ? 2:2)-2)*151 +: 151], //L=1
      _outpipe[((2>1) ? (2-1)*151 : 151) +: 151]); //L=1
   // defparam velmshifter_laneunitlast.151=151;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_2_1 (
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

parameter 2=4;      
parameter 1=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 2-1:0 ]  squash;

input [ 2*1-1:0 ]  inpipe;
output [ 2*1-1:0 ] outpipe;

input [ 1-1:0 ]  shiftin_left;
input [ 1-1:0 ]  shiftin_right;

wire [ (2+1)*1-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_1 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[1-1:0],
      _outpipe[((2>1) ? 1 : 0)+:1], //Support 1 lane
      shiftin_right,
      _outpipe[1-1:0]);
 // defparam velmshifter_laneunit0.1=1;

  //Generate everything in between 


      velmsh0fter_laneun0t_1 velmsh0fter_laneun0t(clk,resetn,load,sh0ft,d0r_left,
          squash[0],
          0np0pe[(0+1)*1-1:0*1],
          _outp0pe[(0+2)*1-1:(0+1)*1],
          _outp0pe[(0)*1-1:(0-1)*1],
          _outp0pe[(0+1)*1-1:0*1]);
     // defparam velmsh0fter_laneun0t.1=1;


      velmsh1fter_laneun1t_1 velmsh1fter_laneun1t(clk,resetn,load,sh1ft,d1r_left,
          squash[1],
          1np1pe[(1+1)*1-1:1*1],
          _outp1pe[(1+2)*1-1:(1+1)*1],
          _outp1pe[(1)*1-1:(1-1)*1],
          _outp1pe[(1+1)*1-1:1*1]);
     // defparam velmsh1fter_laneun1t.1=1;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_1 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[2-1],
      inpipe[2*1-1:(2-1)*1],
      shiftin_left,
      _outpipe[(((2>1) ? 2:2)-2)*1 +: 1], //L=1
      _outpipe[((2>1) ? (2-1)*1 : 1) +: 1]); //L=1
   // defparam velmshifter_laneunitlast.1=1;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_8_1 (
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

parameter 8=4;      
parameter 1=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 8-1:0 ]  squash;

input [ 8*1-1:0 ]  inpipe;
output [ 8*1-1:0 ] outpipe;

input [ 1-1:0 ]  shiftin_left;
input [ 1-1:0 ]  shiftin_right;

wire [ (8+1)*1-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_1 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[1-1:0],
      _outpipe[((8>1) ? 1 : 0)+:1], //Support 1 lane
      shiftin_right,
      _outpipe[1-1:0]);
 // defparam velmshifter_laneunit0.1=1;

  //Generate everything in between 


      velmsh0fter_laneun0t_1 velmsh0fter_laneun0t(clk,resetn,load,sh0ft,d0r_left,
          squash[0],
          0np0pe[(0+1)*1-1:0*1],
          _outp0pe[(0+2)*1-1:(0+1)*1],
          _outp0pe[(0)*1-1:(0-1)*1],
          _outp0pe[(0+1)*1-1:0*1]);
     // defparam velmsh0fter_laneun0t.1=1;


      velmsh1fter_laneun1t_1 velmsh1fter_laneun1t(clk,resetn,load,sh1ft,d1r_left,
          squash[1],
          1np1pe[(1+1)*1-1:1*1],
          _outp1pe[(1+2)*1-1:(1+1)*1],
          _outp1pe[(1)*1-1:(1-1)*1],
          _outp1pe[(1+1)*1-1:1*1]);
     // defparam velmsh1fter_laneun1t.1=1;


      velmsh2fter_laneun2t_1 velmsh2fter_laneun2t(clk,resetn,load,sh2ft,d2r_left,
          squash[2],
          2np2pe[(2+1)*1-1:2*1],
          _outp2pe[(2+2)*1-1:(2+1)*1],
          _outp2pe[(2)*1-1:(2-1)*1],
          _outp2pe[(2+1)*1-1:2*1]);
     // defparam velmsh2fter_laneun2t.1=1;


      velmsh3fter_laneun3t_1 velmsh3fter_laneun3t(clk,resetn,load,sh3ft,d3r_left,
          squash[3],
          3np3pe[(3+1)*1-1:3*1],
          _outp3pe[(3+2)*1-1:(3+1)*1],
          _outp3pe[(3)*1-1:(3-1)*1],
          _outp3pe[(3+1)*1-1:3*1]);
     // defparam velmsh3fter_laneun3t.1=1;


      velmsh4fter_laneun4t_1 velmsh4fter_laneun4t(clk,resetn,load,sh4ft,d4r_left,
          squash[4],
          4np4pe[(4+1)*1-1:4*1],
          _outp4pe[(4+2)*1-1:(4+1)*1],
          _outp4pe[(4)*1-1:(4-1)*1],
          _outp4pe[(4+1)*1-1:4*1]);
     // defparam velmsh4fter_laneun4t.1=1;


      velmsh5fter_laneun5t_1 velmsh5fter_laneun5t(clk,resetn,load,sh5ft,d5r_left,
          squash[5],
          5np5pe[(5+1)*1-1:5*1],
          _outp5pe[(5+2)*1-1:(5+1)*1],
          _outp5pe[(5)*1-1:(5-1)*1],
          _outp5pe[(5+1)*1-1:5*1]);
     // defparam velmsh5fter_laneun5t.1=1;


      velmsh6fter_laneun6t_1 velmsh6fter_laneun6t(clk,resetn,load,sh6ft,d6r_left,
          squash[6],
          6np6pe[(6+1)*1-1:6*1],
          _outp6pe[(6+2)*1-1:(6+1)*1],
          _outp6pe[(6)*1-1:(6-1)*1],
          _outp6pe[(6+1)*1-1:6*1]);
     // defparam velmsh6fter_laneun6t.1=1;


      velmsh7fter_laneun7t_1 velmsh7fter_laneun7t(clk,resetn,load,sh7ft,d7r_left,
          squash[7],
          7np7pe[(7+1)*1-1:7*1],
          _outp7pe[(7+2)*1-1:(7+1)*1],
          _outp7pe[(7)*1-1:(7-1)*1],
          _outp7pe[(7+1)*1-1:7*1]);
     // defparam velmsh7fter_laneun7t.1=1;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_1 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[8-1],
      inpipe[8*1-1:(8-1)*1],
      shiftin_left,
      _outpipe[(((8>1) ? 8:2)-2)*1 +: 1], //L=1
      _outpipe[((8>1) ? (8-1)*1 : 1) +: 1]); //L=1
   // defparam velmshifter_laneunitlast.1=1;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_1_256 (
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

parameter 1=4;      
parameter 256=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 1-1:0 ]  squash;

input [ 1*256-1:0 ]  inpipe;
output [ 1*256-1:0 ] outpipe;

input [ 256-1:0 ]  shiftin_left;
input [ 256-1:0 ]  shiftin_right;

wire [ (1+1)*256-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_256 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[256-1:0],
      _outpipe[((1>1) ? 256 : 0)+:256], //Support 1 lane
      shiftin_right,
      _outpipe[256-1:0]);
 // defparam velmshifter_laneunit0.256=256;

  //Generate everything in between 


      velmsh0fter_laneun0t_256 velmsh0fter_laneun0t(clk,resetn,load,sh0ft,d0r_left,
          squash[0],
          0np0pe[(0+1)*256-1:0*256],
          _outp0pe[(0+2)*256-1:(0+1)*256],
          _outp0pe[(0)*256-1:(0-1)*256],
          _outp0pe[(0+1)*256-1:0*256]);
     // defparam velmsh0fter_laneun0t.256=256;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_256 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[1-1],
      inpipe[1*256-1:(1-1)*256],
      shiftin_left,
      _outpipe[(((1>1) ? 1:2)-2)*256 +: 256], //L=1
      _outpipe[((1>1) ? (1-1)*256 : 256) +: 256]); //L=1
   // defparam velmshifter_laneunitlast.256=256;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_1_8 (
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

parameter 1=4;      
parameter 8=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 1-1:0 ]  squash;

input [ 1*8-1:0 ]  inpipe;
output [ 1*8-1:0 ] outpipe;

input [ 8-1:0 ]  shiftin_left;
input [ 8-1:0 ]  shiftin_right;

wire [ (1+1)*8-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_8 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[8-1:0],
      _outpipe[((1>1) ? 8 : 0)+:8], //Support 1 lane
      shiftin_right,
      _outpipe[8-1:0]);
 // defparam velmshifter_laneunit0.8=8;

  //Generate everything in between 


      velmsh0fter_laneun0t_8 velmsh0fter_laneun0t(clk,resetn,load,sh0ft,d0r_left,
          squash[0],
          0np0pe[(0+1)*8-1:0*8],
          _outp0pe[(0+2)*8-1:(0+1)*8],
          _outp0pe[(0)*8-1:(0-1)*8],
          _outp0pe[(0+1)*8-1:0*8]);
     // defparam velmsh0fter_laneun0t.8=8;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_8 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[1-1],
      inpipe[1*8-1:(1-1)*8],
      shiftin_left,
      _outpipe[(((1>1) ? 1:2)-2)*8 +: 8], //L=1
      _outpipe[((1>1) ? (1-1)*8 : 8) +: 8]); //L=1
   // defparam velmshifter_laneunitlast.8=8;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_1_256 (
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

parameter 1=4;      
parameter 256=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 1-1:0 ]  squash;

input [ 1*256-1:0 ]  inpipe;
output [ 1*256-1:0 ] outpipe;

input [ 256-1:0 ]  shiftin_left;
input [ 256-1:0 ]  shiftin_right;

wire [ (1+1)*256-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_256 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[256-1:0],
      _outpipe[((1>1) ? 256 : 0)+:256], //Support 1 lane
      shiftin_right,
      _outpipe[256-1:0]);
 // defparam velmshifter_laneunit0.256=256;

  //Generate everything in between 


      velmsh0fter_laneun0t_256 velmsh0fter_laneun0t(clk,resetn,load,sh0ft,d0r_left,
          squash[0],
          0np0pe[(0+1)*256-1:0*256],
          _outp0pe[(0+2)*256-1:(0+1)*256],
          _outp0pe[(0)*256-1:(0-1)*256],
          _outp0pe[(0+1)*256-1:0*256]);
     // defparam velmsh0fter_laneun0t.256=256;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_256 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[1-1],
      inpipe[1*256-1:(1-1)*256],
      shiftin_left,
      _outpipe[(((1>1) ? 1:2)-2)*256 +: 256], //L=1
      _outpipe[((1>1) ? (1-1)*256 : 256) +: 256]); //L=1
   // defparam velmshifter_laneunitlast.256=256;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        
/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter_1_8 (
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

parameter 1=4;      
parameter 8=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ 1-1:0 ]  squash;

input [ 1*8-1:0 ]  inpipe;
output [ 1*8-1:0 ] outpipe;

input [ 8-1:0 ]  shiftin_left;
input [ 8-1:0 ]  shiftin_right;

wire [ (1+1)*8-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit_8 velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[8-1:0],
      _outpipe[((1>1) ? 8 : 0)+:8], //Support 1 lane
      shiftin_right,
      _outpipe[8-1:0]);
 // defparam velmshifter_laneunit0.8=8;

  //Generate everything in between 


      velmsh0fter_laneun0t_8 velmsh0fter_laneun0t(clk,resetn,load,sh0ft,d0r_left,
          squash[0],
          0np0pe[(0+1)*8-1:0*8],
          _outp0pe[(0+2)*8-1:(0+1)*8],
          _outp0pe[(0)*8-1:(0-1)*8],
          _outp0pe[(0+1)*8-1:0*8]);
     // defparam velmsh0fter_laneun0t.8=8;


  //HANDLE lane NUMLANE specially

    velmshifter_laneunit_8 velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[1-1],
      inpipe[1*8-1:(1-1)*8],
      shiftin_left,
      _outpipe[(((1>1) ? 1:2)-2)*8 +: 8], //L=1
      _outpipe[((1>1) ? (1-1)*8 : 8) +: 8]); //L=1
   // defparam velmshifter_laneunitlast.8=8;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule
        