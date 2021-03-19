module qadd(a,b,c);
    input [2*`DWIDTH-1:0] a;
    input [2*`DWIDTH-1:0] b;
    output [2*`DWIDTH-1:0] c;

    assign c = a + b;
    //DW01_add #(`DWIDTH) u_add(.A(a), .B(b), .CI(1'b0), .SUM(c), .CO());
endmodule