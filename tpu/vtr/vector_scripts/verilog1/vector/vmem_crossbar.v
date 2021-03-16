/************************
 *
 *************************/

module vmem_crossbar_128_7_8_32_5 (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter SELWIDTH=7-5;   // LOG2(INWIDTH/OUTWIDTH) = 4

input                             clk;
input                             resetn;
input  [(SELWIDfTH*8)-1 : 0] sel;
input  [128-1 : 0]            in;
output [(32*8)-1 : 0] out;


     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(0+1)*SELWIDTH - 1 : 0*SELWIDTH],
        0n,
        out[(0+1)*32 - 1 : 0*32]);

     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(1+1)*SELWIDTH - 1 : 1*SELWIDTH],
        1n,
        out[(1+1)*32 - 1 : 1*32]);

     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(2+1)*SELWIDTH - 1 : 2*SELWIDTH],
        2n,
        out[(2+1)*32 - 1 : 2*32]);

     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(3+1)*SELWIDTH - 1 : 3*SELWIDTH],
        3n,
        out[(3+1)*32 - 1 : 3*32]);

     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(4+1)*SELWIDTH - 1 : 4*SELWIDTH],
        4n,
        out[(4+1)*32 - 1 : 4*32]);

     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(5+1)*SELWIDTH - 1 : 5*SELWIDTH],
        5n,
        out[(5+1)*32 - 1 : 5*32]);

     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(6+1)*SELWIDTH - 1 : 6*SELWIDTH],
        6n,
        out[(6+1)*32 - 1 : 6*32]);

     vmem_busmux_128_7_32_5 bmux(clk,resetn,
        sel[(7+1)*SELWIDTH - 1 : 7*SELWIDTH],
        7n,
        out[(7+1)*32 - 1 : 7*32]);

 endmodule 
