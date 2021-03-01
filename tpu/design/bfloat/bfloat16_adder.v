module bfloat_adder(
 a,
 b,
 out
);

parameter ADDERWIDTH = 16;
parameter MANTISSAWIDTH = 7;
parameter EXPONENTWIDTH = 8;


input [ADDERWIDTH-1:0] a;
input [ADDERWIDTH-1:0] b;
output [ADDERWIDTH-1:0] out;

wire[MANTISSAWIDTH-1:0] a_matissa;
wire[MANTISSAWIDTH-1:0] b_matissa;
wire[EXPONENTWIDTH-1:0] a_expo;
wire[EXPONENTWIDTH-1:0] b_expo;

assign a_mantissa = a[MANTISSAWIDTH-1:0];
assign b_mantissa = b[MANTISSAWIDTH-1:0];
assign a_expo = a[EXPONENTWIDTH+MANTISSAWIDTH-1:MANTISSAWIDTH];
assign b_expo = b[EXPONENTWIDTH+MANTISSAWIDTH-1:MANTISSAWIDTH];
assign a_sign = a[ADDERWIDTH-1];
assign b_sign = b[ADDERWIDTH-1];

reg[MANTISSAWIDTH-1:0]a_shifted_mantissa;
reg[MANTISSAWIDTH-1:0]b_shifted_mantissa;

always@(*)begin
    a_shifted_mantissa = a_mantissa;
    b_shifted_mantissa = b_mantissa;
    if(a_expo > b_expo)
       b_shifted mantissa = b_mantissa >> a_expo - b_expo;
    else
       a_shifted_mantissa = a_mantissa >> b_expo - a_expo; 
end

