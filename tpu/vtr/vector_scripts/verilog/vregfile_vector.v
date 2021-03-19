/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_vector_2_1_128_1024_10
  (clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_byteen, c_we);

input clk;
input resetn;

input [2-1:0] a_en;
input [2-1:0] b_en;
input [2*9-1:0] a_reg,b_reg,c_reg;
output [2*128-1:0] a_readdataout, b_readdataout;
wire [2*128-1:0] a_temp, b_temp;

input [2*128-1:0] c_writedatain;
input [2*128/8-1:0] c_byteen;
input [2-1:0] c_we;


          ram_wrapper_9_512_128 reg_file1_0(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
  	    .address_a(c_reg[8:0]),
  	    .address_b(a_reg[8:0]),
  	    .wren_a(c_we[0] &  c_byteen[0]),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[0*128 +: 128]),
  	    .data_b(0),
  	    .out_a(a_temp[0*128 +: 128]),
  	    .out_b(a_readdataout[0*128 +: 128])
          );
  
          ram_wrapper_9_512_128 reg_file2_0(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(b_en),
  	    .address_a(c_reg[8:0]),
  	    .address_b(b_reg[8:0]),
  	    .wren_a(c_we[0] &  c_byteen[0]),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[127:0]),
  	    .data_b(0),
  	    .out_a(b_temp[0*128 +: 128]),
  	    .out_b(b_readdataout[127:0])
          );

          ram_wrapper_9_512_128 reg_file1_1(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
  	    .address_a(c_reg[8:0]),
  	    .address_b(a_reg[8:0]),
  	    .wren_a(c_we[1] &  c_byteen[1]),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[1*128 +: 128]),
  	    .data_b(0),
  	    .out_a(a_temp[1*128 +: 128]),
  	    .out_b(a_readdataout[1*128 +: 128])
          );
  
          ram_wrapper_9_512_128 reg_file2_1(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(b_en),
  	    .address_a(c_reg[8:0]),
  	    .address_b(b_reg[8:0]),
  	    .wren_a(c_we[1] &  c_byteen[1]),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[127:0]),
  	    .data_b(0),
  	    .out_a(b_temp[1*128 +: 128]),
  	    .out_b(b_readdataout[127:0])
          );

endmodule

