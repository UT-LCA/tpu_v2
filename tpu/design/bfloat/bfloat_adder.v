module bfloat_adder #(parameter WIDTH=4)(
 input clk,
 input resetn,
 input en,
 input stall,
 input [WIDTH-1:0] a,
 input [WIDTH-1:0] b,
 output reg[WIDTH-1:0] out
);

always@(posedge clk)begin
  if(!resetn)
    out <= 'h0;
  else
    if(en)
      out <= a + b;
end

endmodule
