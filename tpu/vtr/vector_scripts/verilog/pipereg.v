/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_1(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [1-1:0] d;
output [1-1:0] q;
reg [1-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_17(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [17-1:0] d;
output [17-1:0] q;
reg [17-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_1(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [1-1:0] d;
output [1-1:0] q;
reg [1-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_8(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [8-1:0] d;
output [8-1:0] q;
reg [8-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_72(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [72-1:0] d;
output [72-1:0] q;
reg [72-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule