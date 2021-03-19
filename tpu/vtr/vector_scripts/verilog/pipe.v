/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_1_2(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input  d;
input              clk;
input              resetn;
input  [2-1:0] en;
input  [2-1:0] squash;
output [(2+1)-1:0] q;

reg [2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq <= 0;
    else if (en[0])
      tq <=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i]<= 0;
      else if (en[i])
        tq[i]<=tq[(i-1)];
  end

  assign q[0] =d;
  assign q[(2+1)-1:1]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_5_2(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [5-1:0]  d;
input              clk;
input              resetn;
input  [2-1:0] en;
input  [2-1:0] squash;
output [5*(2+1)-1:0] q;

reg [5*2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 5-1:0 ]<= 0;
    else if (en[0])
      tq[ 5-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i*5 +: 5 ]<= 0;
      else if (en[i])
        tq[i*5 +: 5 ]<=tq[(i-1)*5 +: 5 ];
  end

  assign q[5-1:0]=d;
  assign q[5*(2+1)-1:5]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_10_2(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [10-1:0]  d;
input              clk;
input              resetn;
input  [2-1:0] en;
input  [2-1:0] squash;
output [10*(2+1)-1:0] q;

reg [10*2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 10-1:0 ]<= 0;
    else if (en[0])
      tq[ 10-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i*10 +: 10 ]<= 0;
      else if (en[i])
        tq[i*10 +: 10 ]<=tq[(i-1)*10 +: 10 ];
  end

  assign q[10-1:0]=d;
  assign q[10*(2+1)-1:10]=tq;
endmodule
        /****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_4_2(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [4-1:0]  d;
input              clk;
input              resetn;
input  [2-1:0] en;
input  [2-1:0] squash;
output [4*(2+1)-1:0] q;

reg [4*2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 4-1:0 ]<= 0;
    else if (en[0])
      tq[ 4-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i*4 +: 4 ]<= 0;
      else if (en[i])
        tq[i*4 +: 4 ]<=tq[(i-1)*4 +: 4 ];
  end

  assign q[4-1:0]=d;
  assign q[4*(2+1)-1:4]=tq;
endmodule
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
reg[31:0] i;

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
reg[31:0] i;

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
module pipe_10_1(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [10-1:0]  d;
input              clk;
input              resetn;
input en;
input squash;
output [10*(1+1)-1:0] q;

reg [10*1-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 10-1:0 ]<= 0;
    else if (en[0])
      tq[ 10-1:0 ]<=d;
  end

  assign q[10-1:0]=d;
  assign q[10*(1+1)-1:10]=tq;
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

input d;
input              clk;
input              resetn;
input en;
input squash;
output [1:0] q;

reg  tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq <= 0;
    else if (en[0])
      tq <=d;
  end

  assign q[0] = d;
  assign q[1]=tq;
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
input en;
input squash;
output [8*(1+1)-1:0] q;

reg [8*1-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 8-1:0 ]<= 0;
    else if (en[0])
      tq[ 8-1:0 ]<=d;
  end

  assign q[8-1:0]=d;
  assign q[8*(1+1)-1:8]=tq;
endmodule
        