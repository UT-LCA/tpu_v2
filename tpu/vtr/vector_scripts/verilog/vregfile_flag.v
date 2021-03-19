/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_flag_2_1_4_1024_10
  (clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_we);

input clk;
input resetn;

input [2-1:0] a_en;
input [2-1:0] b_en;
input [2-1:0] c_we;

input [2*9-1:0] a_reg,b_reg,c_reg;
output [2*4-1:0] a_readdataout, b_readdataout;
wire [2*4-1:0] a_temp, b_temp;
input [2*4-1:0] c_writedatain;


      ram_wrapper_9_512_4 reg_file1_0(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(a_en[0]),
          .address_a(c_reg[0* 9 +: 9]),
          .address_b(a_reg[0* 9 +: 9]),
          .wren_a(c_we[0]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ 0*4 +: 4]),
          .data_b(0),
          .out_a(a_temp[ 0*4 +: 4]),
          .out_b(a_readdataout[0*4 +: 4])
      );

      ram_wrapper_9_512_4 reg_file2_0(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(b_en[0]),
          .address_a(c_reg[0* 9 +: 9]),
          .address_b(b_reg[0* 9 +: 9]),
          .wren_a(c_we[0]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ 0*4 +: 4]),
          .data_b(0),
          .out_a(b_temp[ 0*4 +: 4]),
          .out_b(b_readdataout[ 0* 4 +: 4])
      );
      
      ram_wrapper_9_512_4 reg_file1_1(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(a_en[1]),
          .address_a(c_reg[1* 9 +: 9]),
          .address_b(a_reg[1* 9 +: 9]),
          .wren_a(c_we[1]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ 1*4 +: 4]),
          .data_b(0),
          .out_a(a_temp[ 1*4 +: 4]),
          .out_b(a_readdataout[1*4 +: 4])
      );

      ram_wrapper_9_512_4 reg_file2_1(
          .clk(clk),
          .resetn(resetn),
          .rden_a(1'b0),
          .rden_b(b_en[1]),
          .address_a(c_reg[1* 9 +: 9]),
          .address_b(b_reg[1* 9 +: 9]),
          .wren_a(c_we[1]),
          .wren_b(1'b0),
          .data_a(c_writedatain[ 1*4 +: 4]),
          .data_b(0),
          .out_a(b_temp[ 1*4 +: 4]),
          .out_b(b_readdataout[ 1* 4 +: 4])
      );
            
endmodule
