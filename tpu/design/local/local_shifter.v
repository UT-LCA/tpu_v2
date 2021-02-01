module local_shifter(
  data,
  distance,
  direction,
  result
);

parameter LPM_WIDTH = 32;
parameter LPM_WIDTHDIST = 5;
parameter LPM_SHIFTTYPE = "LOGICAL";

input [LPM_WIDTH-1:0] data;
input [LPM_WIDTHDIST-1:0] distance;
input direction;

output reg [LPM_WIDTH-1:0] result;
reg [LPM_WIDTH-1:0] arith_reg; 
always @* begin
  arith_reg = {LPM_WIDTH{1'b1}};
  case(LPM_SHIFTTYPE)
    "LOGICAL":begin 
                  if(direction == 1'b0)begin
                      result = data << distance;
                  end
                  else begin
                      result = data >> distance;
                  end
              end
    "ARITHMATIC":begin 
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
    default:begin
                  if(direction == 1'b0)begin
                      result = data << distance;
                  end
                  else begin
                      result = data >> distance;
                  end
            end
  endcase
end
endmodule
