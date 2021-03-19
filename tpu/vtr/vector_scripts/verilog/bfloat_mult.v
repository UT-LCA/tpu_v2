
module bfloat_mult_10 (
 input clk,
 input resetn,
 input en,
 input stall,
 input [10-1:0] a,
 input [10-1:0] b,
 output reg[10-1:0] out
);

always@(posedge clk)begin
  if(!resetn)
    out <= 'h0;
  else
    if(en)
      out <= a * b;
end

endmodule
        