/****************************************************************************
          Stride Register File

   - Has one read port (a) and one write port (c)
****************************************************************************/
module vregfile_stride_32_8_3.0 (
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
input [3.0-1:0] a_reg,c_reg;
output [32-1:0] a_readdataout;
input [32-1:0] c_writedatain;
input c_we;

        ram_wrapper_3.0_8_32 reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[3.0-1:0]),
	    .address_b(a_reg[3.0-1:0]),
	    .wren_a(c_we),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout)
        );
endmodule
        