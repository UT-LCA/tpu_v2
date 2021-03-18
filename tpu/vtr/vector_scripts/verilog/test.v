
module test_32 (
 input clk,
 input resetn,
 input en,
 input stall,
 input [32-1:0] a,
 input [32-1:0] b,
 output reg[32-1:0] out
);

function [32-1:0] sum;
    input [31:0] x, b;
    begin
        sum = x+b;
    end
endfunction

always@*
    out = sum (a,b);

endmodule
        