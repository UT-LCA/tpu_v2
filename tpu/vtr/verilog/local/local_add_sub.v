module local_add_sub_32_0_SIGNED(
dataa,
datab,
cin,
add_sub,
result
);

parameter WIDTH = 32;
parameter PIPELINE = 0;
parameter REPRESENTATION = "SIGNED";

input[WIDTH-1:0] dataa;
input[WIDTH-1:0] datab;
input cin;
input add_sub;
output reg [WIDTH-1:0] result;

always @(*)begin
    if(add_sub == 1'b1)
         result = dataa+datab+cin;
    else
         result = dataa - datab;
end

endmodule