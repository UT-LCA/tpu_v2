/****************************************************************************
          Generic Register
****************************************************************************/
module lo_reg_32 (d,clk,resetn,squashn,en,q);

input clk;
input resetn;
input squashn;
input en;
input [32-1:0] d;
output [32-1:0] q;
reg [32-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1 && squashn)
		q<=d;
end

endmodule