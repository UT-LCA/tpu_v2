/************************
 *
 *************************/

module vmem_crossbar_128_7_16_8_3 (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter SELWIDTH=7-3;   // LOG2(INWIDTH/OUTWIDTH) = 4

input                             clk;
input                             resetn;
input  [(SELWIDfTH*16)-1 : 0] sel;
input  [128-1 : 0]            in;
output [(8*16)-1 : 0] out;


     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(0+1)*SELWIDTH - 1 : 0*SELWIDTH],
        0n,
        out[(0+1)*8 - 1 : 0*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(1+1)*SELWIDTH - 1 : 1*SELWIDTH],
        1n,
        out[(1+1)*8 - 1 : 1*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(2+1)*SELWIDTH - 1 : 2*SELWIDTH],
        2n,
        out[(2+1)*8 - 1 : 2*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(3+1)*SELWIDTH - 1 : 3*SELWIDTH],
        3n,
        out[(3+1)*8 - 1 : 3*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(4+1)*SELWIDTH - 1 : 4*SELWIDTH],
        4n,
        out[(4+1)*8 - 1 : 4*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(5+1)*SELWIDTH - 1 : 5*SELWIDTH],
        5n,
        out[(5+1)*8 - 1 : 5*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(6+1)*SELWIDTH - 1 : 6*SELWIDTH],
        6n,
        out[(6+1)*8 - 1 : 6*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(7+1)*SELWIDTH - 1 : 7*SELWIDTH],
        7n,
        out[(7+1)*8 - 1 : 7*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(8+1)*SELWIDTH - 1 : 8*SELWIDTH],
        8n,
        out[(8+1)*8 - 1 : 8*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(9+1)*SELWIDTH - 1 : 9*SELWIDTH],
        9n,
        out[(9+1)*8 - 1 : 9*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(10+1)*SELWIDTH - 1 : 10*SELWIDTH],
        10n,
        out[(10+1)*8 - 1 : 10*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(11+1)*SELWIDTH - 1 : 11*SELWIDTH],
        11n,
        out[(11+1)*8 - 1 : 11*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(12+1)*SELWIDTH - 1 : 12*SELWIDTH],
        12n,
        out[(12+1)*8 - 1 : 12*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(13+1)*SELWIDTH - 1 : 13*SELWIDTH],
        13n,
        out[(13+1)*8 - 1 : 13*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(14+1)*SELWIDTH - 1 : 14*SELWIDTH],
        14n,
        out[(14+1)*8 - 1 : 14*8]);

     vmem_busmux_128_7_8_3 bmux(clk,resetn,
        sel[(15+1)*SELWIDTH - 1 : 15*SELWIDTH],
        15n,
        out[(15+1)*8 - 1 : 15*8]);

 endmodule 
module vmem_busmux_128_7_8_3 (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter SELWIDTH=7-3;   // LOG2(INWIDTH/OUTWIDTH) = 4

input clk;
input resetn;
input  [SELWIDTH-1 : 0] sel;
input  [128-1 : 0]  in;
output [8-1 : 0] out;

reg    [8-1 : 0] out;

integer k;

  always@*
  begin
    out=0;
    for (k=0; k<128/8; k=k+1)
      if (k==sel)
        out=in[ k*8 +: 8 ];
  end
endmodule
        