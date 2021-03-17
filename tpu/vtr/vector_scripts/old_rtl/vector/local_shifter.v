module local_shifter_2_2_ARITHMATIC(
  data,
  distance,
  direction,
  result
);

parameter LPM_SHIFTTYPE = "ARITHMATIC";

input [2-1:0] data;
input [2-1:0] distance;
input direction;

output reg [2-1:0] result;
reg [2-1:0] arith_reg; 
always @* begin
  arith_reg = {2{1'b1}};
    if (LPM_SHIFTTYPE == "ARITHMETIC") begin
    if(direction)begin
      result = data << distance;
    end
    else begin
      if(data[2-1] == 1'b1)
          result =  ((arith_reg <<(2 - distance))|| (data >> distance));
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
endmodulemodule local_shifter_2_2_ARITHMATIC(
  data,
  distance,
  direction,
  result
);

parameter LPM_SHIFTTYPE = "ARITHMATIC";

input [2-1:0] data;
input [2-1:0] distance;
input direction;

output reg [2-1:0] result;
reg [2-1:0] arith_reg; 
always @* begin
  arith_reg = {2{1'b1}};
    if (LPM_SHIFTTYPE == "ARITHMETIC") begin
    if(direction)begin
      result = data << distance;
    end
    else begin
      if(data[2-1] == 1'b1)
          result =  ((arith_reg <<(2 - distance))|| (data >> distance));
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