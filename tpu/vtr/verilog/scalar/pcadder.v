module pcadder_32(pc, offset, result);

input [32-1:0] pc;
input [32-1:0] offset;
output [32-1:0] result;

wire dum;

assign {dum,result} = pc + {offset[32-3:0],2'b0};

endmodule