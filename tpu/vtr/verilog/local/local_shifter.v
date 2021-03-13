module local_shifter_32_5_ARITHMETIC(
  data,
  distance,
  direction,
  result
);

parameter LPM_SHIFTTYPE = "ARITHMETIC";

input [32-1:0] data;
input [5-1:0] distance;
input direction;

output reg [32-1:0] result;
reg [32-1:0] arith_reg; 
always @* begin
  arith_reg = {32{1'b1}};
    if (LPM_SHIFTTYPE == "ARITHMETIC") begin
    if(direction)begin
      result = data << distance;
    end
    else begin
      if(data[32-1] == 1'b1)
          result =  ((arith_reg <<(32 - distance))|| (data >> distance));
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