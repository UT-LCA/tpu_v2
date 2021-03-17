/****************************************************************************
          Control Register File

   - Has one read port (a) and one write port (c)
   - vl promoted as first-class entities
****************************************************************************/
module vregfile_control_32_32_5.0 (
    clk,
    resetn, 

    a_reg, 
    a_en,
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we,

    vl,
    matmul_masks
    );

input clk;
input resetn;

input a_en;
input [5.0-1:0] a_reg,c_reg;
output [32-1:0] a_readdataout;
input [32-1:0] c_writedatain;
input c_we;

output [32-1:0] vl;
output [3*8-1:0] matmul_masks;

reg [32-1:0] vl;
reg [32-1:0] matmul_masks;

ram_wrapper_5.0_32_32 reg_file1(
        .clk(clk),
        .resetn(resetn),
        .rden_a(1'b0),
        .rden_b(a_en),
        .address_a(c_reg[5.0-1:0]),
        .address_b(a_reg[5.0-1:0]),
        .wren_a(c_we),
        .wren_b(1'b0),
        .data_a(c_writedatain),
        .data_b(0),
        .out_a(),
        .out_b(a_readdataout)
  );

`ifdef TEST_BENCH
   initial begin
       $readmemh("vregfile_control.dat",reg_file1.dpram1.ram,'h0);
   end
`endif 

always@(posedge clk) begin
  if (!resetn) begin
    vl<=0;
    matmul_masks<=32'hffffffff;
  end
  else begin
    if (c_we) begin
      if (c_reg==0) begin
        vl<=c_writedatain;
      end 
      else if (c_reg==31) begin //a_rows
        matmul_masks[1*8-1:0*8] <= c_writedatain[8-1:0];
      end
      else if (c_reg==30) begin //a_cols, b_rows
        matmul_masks[2*8-1:1*8] <= c_writedatain[8-1:0];
      end
      else if (c_reg==29) begin //b_cols
        matmul_masks[3*8-1:2*8] <= c_writedatain[8-1:0];
      end
    end  
  end
end

endmodule
