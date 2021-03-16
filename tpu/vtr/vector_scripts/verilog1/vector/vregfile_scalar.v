/****************************************************************************
          Scalar Register File

   - Has one read port (a) and one write port (c)
   - vs0 fixed to 0 and unwriteable
****************************************************************************/
module vregfile_scalar_32_32_5 (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

input clk;
input resetn;

input a_en;
input [5-1:0] a_reg,c_reg;
output [32-1:0] a_readdataout;
input [32-1:0] c_writedatain;
input c_we;

    ram_wrapper_5_32_32 reg_file1(
        .clk(clk),
        .resetn(resetn),
        .rden_a(1'b0),
        .rden_b(a_en),
        .address_a(c_reg[5-1:0]),
        .address_b(a_reg[5-1:0]),
        .wren_a(c_we & (|c_reg)),
        .wren_b(1'b0),
        .data_a(c_writedatain),
        .data_b(0),
        .out_a(),
        .out_b(a_readdataout)
    );

endmodule
        