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
module pipe_16_2(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );

input [16-1:0]  d;
input              clk;
input              resetn;
input  [2-1:0] en;
input  [2-1:0] squash;
output [16*(2+1)-1:0] q;

reg [16*2-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 16-1:0 ]<= 0;
    else if (en[0])
      tq[ 16-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<2; i=i+1)
      if (!resetn || squash[i] )
        tq[i*16 +: 16 ]<= 0;
      else if (en[i])
        tq[i*16 +: 16 ]<=tq[(i-1)*16 +: 16 ];
  end

  assign q[16-1:0]=d;
  assign q[16*(2+1)-1:16]=tq;
endmodule
        