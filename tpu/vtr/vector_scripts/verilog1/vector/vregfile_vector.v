/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_vector_2_1_128_256_8
  (clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_byteen, c_we);

input clk;
input resetn;

input [2-1:0] a_en;
input [2-1:0] b_en;
input [2*7-1:0] a_reg,b_reg,c_reg;
output [2*128-1:0] a_readdataout, b_readdataout;
input [2*128-1:0] c_writedatain;
input [((128>=8) ? 2*128/8-1 : 2-1):0] c_byteen;
input [2-1:0] c_we;

  genvar k;

  generate
  for (k=0; k<2; k=k+1)
  begin : bank_gen
          ram_wrapper_7_128_128 reg_file1(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
  	    .address_a(c_reg[k*7 +: 7]),
  	    .address_b(a_reg[k*7 +: 7]),
  	    .wren_a(c_we[k] & ((128>8) ? 1'b1 : c_byteen[k])),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[k*128 +: 128]),
  	    .data_b(0),
  	    .out_a(),
  	    .out_b(a_readdataout[k*128 +: 128])
          );
  
          ram_wrapper_7_128_128 reg_file2(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(b_en),
  	    .address_a(c_reg[k*7 +: 7]),
  	    .address_b(b_reg[k*7 +: 7]),
  	    .wren_a(c_we[k] & ((128>8) ? 1'b1 : c_byteen[k])),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[k*128 +: 128]),
  	    .data_b(0),
  	    .out_a(),
  	    .out_b(b_readdataout[k*128 +: 128])
          );
  end
  endgenerate

endmodule
