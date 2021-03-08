module local_shifter_32_5_ARITHMETIC(
  data,
  distance,
  direction,
  result
);

parameter LPM_WIDTH = 32;
parameter LPM_WIDTHDIST = 5;
parameter LPM_SHIFTTYPE = "ARITHMETIC";

input [LPM_WIDTH-1:0] data;
input [LPM_WIDTHDIST-1:0] distance;
input direction;

output reg [LPM_WIDTH-1:0] result;
reg [LPM_WIDTH-1:0] arith_reg; 
always @* begin
  arith_reg = {LPM_WIDTH{1'b1}};
    if (LPM_SHIFTTYPE == "ARITHMETIC") begin
    if(direction)begin
      result = data << distance;
    end
    else begin
      if(data[LPM_WIDTH-1] == 1'b1)
          result =  ((arith_reg <<(LPM_WIDTH - distance))|| (data >> distance));
      else
          result = data >> distance;
    end
  end
  else begin
    if(direction == 1'b0)begin
        result = data << distance;
    end
    else begin
        result = data >> distance;
    end
  end 
end
endmodule