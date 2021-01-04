/************************
 *
 *************************/

module vmem_crossbar (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter INWIDTH=128;       //Bit width of input
parameter LOG2INWIDTH=7;
parameter NUMOUTS=16;        //Number of Outputs
parameter OUTWIDTH=8;        //Bit width of each Output
parameter LOG2OUTWIDTH=3;

parameter SELWIDTH=LOG2INWIDTH-LOG2OUTWIDTH;   // LOG2(INWIDTH/OUTWIDTH) = 4

input                             clk;
input                             resetn;
input  [(SELWIDTH*NUMOUTS)-1 : 0] sel;
input  [INWIDTH-1 : 0]            in;
output [(OUTWIDTH*NUMOUTS)-1 : 0] out;

genvar i;

 generate
   for (i=0; i < NUMOUTS; i=i+1) begin : MEM
     vmem_busmux bmux(clk,resetn,
        sel[(i+1)*SELWIDTH - 1 : i*SELWIDTH],
        in,
        out[(i+1)*OUTWIDTH - 1 : i*OUTWIDTH]);
      defparam bmux.INWIDTH=INWIDTH;
      defparam bmux.LOG2INWIDTH=LOG2INWIDTH;
      defparam bmux.OUTWIDTH=OUTWIDTH;
      defparam bmux.LOG2OUTWIDTH=LOG2OUTWIDTH;
   end
 endgenerate


endmodule



module vmem_busmux (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter INWIDTH=128;
parameter LOG2INWIDTH=7;
parameter OUTWIDTH=8;
parameter LOG2OUTWIDTH=3;

parameter SELWIDTH=LOG2INWIDTH-LOG2OUTWIDTH;   // LOG2(INWIDTH/OUTWIDTH) = 4

input clk;
input resetn;
input  [SELWIDTH-1 : 0] sel;
input  [INWIDTH-1 : 0]  in;
output [OUTWIDTH-1 : 0] out;

reg    [OUTWIDTH-1 : 0] out;

integer k;

  always@*
  begin
    out=0;
    for (k=0; k<INWIDTH/OUTWIDTH; k=k+1)
      if (k==sel)
        out=in[ k*OUTWIDTH +: OUTWIDTH ];
  end


endmodule
