/****************************************************************************
          Generic Register
****************************************************************************/
module register_33(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [33-1:0] d;
output [33-1:0] q;
reg [33-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule
/****************************************************************************
          Generic Register
****************************************************************************/
module register_5(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [5-1:0] d;
output [5-1:0] q;
reg [5-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule
/****************************************************************************
          Generic Register
****************************************************************************/
module register_1(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [1-1:0] d;
output [1-1:0] q;
reg [1-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule
