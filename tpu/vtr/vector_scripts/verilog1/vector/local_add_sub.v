module local_add_sub_34_0_SIGNED(
dataa,
datab,
cin,
add_sub,
result
);

input[34-1:0] dataa;
input[34-1:0] datab;
input cin;
input add_sub;
output reg [34-1:0] result;

always @(*)begin
    if(add_sub == 1'b1)
         result = dataa+datab+cin;
    else
         result = dataa - datab;
end

endmodule