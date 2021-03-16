module activation_32(
 input clk,
 input resetn,
 input en,
 input stall,
 input [32-1:0] a,
 output reg[32-1:0] out
);

always@(posedge clk)begin
  if(!resetn)
    out <= 'h0;
  else
    if(en)
      if(a>0)
        out <= a;
      else
        out <= 0;
end

endmodule

        