module vmem_busmux_128_7_32_5 (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter SELWIDTH=7-5;   // LOG2(INWIDTH/OUTWIDTH) = 4

input clk;
input resetn;
input  [SELWIDTH-1 : 0] sel;
input  [128-1 : 0]  in;
output [32-1 : 0] out;

reg    [32-1 : 0] out;

integer k;

  always@*
  begin
    out=0;
    for (k=0; k<128/32; k=k+1)
      if (k==sel)
        out=in[ k*32 +: 32 ];
  end
endmodule
        /************************
 *
 *************************/

module vmem_crossbar_128_7_2_32_5 (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter SELWIDTH=7-5;   // LOG2(INWIDTH/OUTWIDTH) = 4

input                             clk;
input                             resetn;
input  [(SELWIDTH*2)-1 : 0] sel;
input  [128-1 : 0]            in;
output [(32*2)-1 : 0] out;


     vmem_busmux_128_7_32_5 bmux0(clk,resetn,
        sel[(0+1)*SELWIDTH - 1 : 0*SELWIDTH],
        in,
        out[(0+1)*32 - 1 : 0*32]);

     vmem_busmux_128_7_32_5 bmux1(clk,resetn,
        sel[(1+1)*SELWIDTH - 1 : 1*SELWIDTH],
        in,
        out[(1+1)*32 - 1 : 1*32]);

 endmodule 
