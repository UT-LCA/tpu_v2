/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_flag_2_1_8_256_8
  (clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_we);

parameter 2=1;
parameter 1=0;
parameter 8=32;
parameter 256=32;
parameter 8=5;

parameter 128=256/2;
parameter 7=8-1;

input clk;
input resetn;

input [2-1:0] a_en;
input [2-1:0] b_en;
input [2-1:0] c_we;

input [2*7-1:0] a_reg,b_reg,c_reg;
output [2*8-1:0] a_readdataout, b_readdataout;
input [2*8-1:0] c_writedatain;

genvar k;

generate
for (k=0; k<2; k=k+1)
begin : bank_gen
      ram_wrapper_7_128_8 reg_file1(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(a_en[k]),
          .address_a(c_reg[k* 7 +: 7]),
          .address_b(a_reg[k* 7 +: 7]),
          .wren_a(c_we[k]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ k*8 +: 8]),
          .data_b(0),
          .out_a(),
          .out_b(a_readdataout[k*8 +: 8])
      );

      ram_wrapper_7_128_8 reg_file2(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(b_en[k]),
          .address_a(c_reg[k* 7 +: 7]),
          .address_b(b_reg[k* 7 +: 7]),
          .wren_a(c_we[k]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ k*8 +: 8]),
          .data_b(0),
          .out_a(),
          .out_b(b_readdataout[ k* 8 +: 8])
      );
end
endgenerate

endmodule
