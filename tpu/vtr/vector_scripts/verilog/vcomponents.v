/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_1_32(
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
input  [32-1:0] en;
input  [32-1:0] squash;
output [(32+1)-1:0] q;

reg [32-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq <= 0;
    else if (en[0])
      tq <=d;

    // All the rest registers
    for (i=1; i<32; i=i+1)
      if (!resetn || squash[i] )
        tq[i]<= 0;
      else if (en[i])
        tq[i]<=tq[(i-1)];
  end

  assign q[0] =d;
  assign q[(32+1)-1:1]=tq;
endmodule
        

/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_10_32(
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
input  [32-1:0] en;
input  [32-1:0] squash;
output [10*(32+1)-1:0] q;

reg [10*32-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 10-1:0 ]<= 0;
    else if (en[0])
      tq[ 10-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<32; i=i+1)
      if (!resetn || squash[i] )
        tq[i*10 +: 10 ]<= 0;
      else if (en[i])
        tq[i*10 +: 10 ]<=tq[(i-1)*10 +: 10 ];
  end

  assign q[10-1:0]=d;
  assign q[10*(32+1)-1:10]=tq;
endmodule
        

/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe_4_32(
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
input  [32-1:0] en;
input  [32-1:0] squash;
output [4*(32+1)-1:0] q;

reg [4*32-1:0] tq;
reg[31:0] i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ 4-1:0 ]<= 0;
    else if (en[0])
      tq[ 4-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<32; i=i+1)
      if (!resetn || squash[i] )
        tq[i*4 +: 4 ]<= 0;
      else if (en[i])
        tq[i*4 +: 4 ]<=tq[(i-1)*4 +: 4 ];
  end

  assign q[4-1:0]=d;
  assign q[4*(32+1)-1:4]=tq;
endmodule
        