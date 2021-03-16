/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_7_5(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [7-1:0]  d;
input              clk;
input              resetn;
input  [5-1:0] en;
input  [5-1:0] squash;
output [7*(5+1)-1:0] q;

reg [7*5-1:0] tq;
integer i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 7-1:0 ]<= 0;
    else if (en[0])
      tq[ 7-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<5; i=i+1)
      if (!resetn || squash[i] )
        tq[i*7 +: 7 ]<= 0;
      else if (en[i])
        tq[i*7 +: 7 ]<=tq[(i-1)*7 +: 7 ];
  end

  assign q[7-1:0]=d;
  assign q[7*(5+1)-1:7]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_32_4(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [32-1:0]  d;
input              clk;
input              resetn;
input  [4-1:0] en;
input  [4-1:0] squash;
output [32*(4+1)-1:0] q;

reg [32*4-1:0] tq;
integer i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 32-1:0 ]<= 0;
    else if (en[0])
      tq[ 32-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<4; i=i+1)
      if (!resetn || squash[i] )
        tq[i*32 +: 32 ]<= 0;
      else if (en[i])
        tq[i*32 +: 32 ]<=tq[(i-1)*32 +: 32 ];
  end

  assign q[32-1:0]=d;
  assign q[32*(4+1)-1:32]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_8_1(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [8-1:0]  d;
input              clk;
input              resetn;
input  [1-1:0] en;
input  [1-1:0] squash;
output [8*(1+1)-1:0] q;

reg [8*1-1:0] tq;
integer i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 8-1:0 ]<= 0;
    else if (en[0])
      tq[ 8-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<1; i=i+1)
      if (!resetn || squash[i] )
        tq[i*8 +: 8 ]<= 0;
      else if (en[i])
        tq[i*8 +: 8 ]<=tq[(i-1)*8 +: 8 ];
  end

  assign q[8-1:0]=d;
  assign q[8*(1+1)-1:8]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_1_3(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [1-1:0]  d;
input              clk;
input              resetn;
input  [3-1:0] en;
input  [3-1:0] squash;
output [1*(3+1)-1:0] q;

reg [1*3-1:0] tq;
integer i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 1-1:0 ]<= 0;
    else if (en[0])
      tq[ 1-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<3; i=i+1)
      if (!resetn || squash[i] )
        tq[i*1 +: 1 ]<= 0;
      else if (en[i])
        tq[i*1 +: 1 ]<=tq[(i-1)*1 +: 1 ];
  end

  assign q[1-1:0]=d;
  assign q[1*(3+1)-1:1]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_8_1(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [8-1:0]  d;
input              clk;
input              resetn;
input  [1-1:0] en;
input  [1-1:0] squash;
output [8*(1+1)-1:0] q;

reg [8*1-1:0] tq;
integer i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 8-1:0 ]<= 0;
    else if (en[0])
      tq[ 8-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<1; i=i+1)
      if (!resetn || squash[i] )
        tq[i*8 +: 8 ]<= 0;
      else if (en[i])
        tq[i*8 +: 8 ]<=tq[(i-1)*8 +: 8 ];
  end

  assign q[8-1:0]=d;
  assign q[8*(1+1)-1:8]=tq;
endmodule
        