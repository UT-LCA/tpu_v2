/****************************************************************************
          Generic Register
****************************************************************************/
module hi_reg(d,clk,resetn,squashn,en,q);
parameter WIDTH=32;

input clk;
input resetn;
input squashn;
input en;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
reg [WIDTH-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1 && squashn)
		q<=d;
end

endmodule

