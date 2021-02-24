/****************************************************************************
          Control Register File

   - Has one read port (a) and one write port (c)
   - vl promoted as first-class entities
****************************************************************************/
module vregfile_control (
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

parameter WIDTH=32;
parameter NUMREGS=32;
parameter LOG2NUMREGS=5;

input clk;
input resetn;

input a_en;
input [LOG2NUMREGS-1:0] a_reg,c_reg;
output [WIDTH-1:0] a_readdataout;
input [WIDTH-1:0] c_writedatain;
input c_we;

output [WIDTH-1:0] vl;
output [3*`MAT_MUL_SIZE-1:0] matmul_masks;

reg [WIDTH-1:0] vl;
reg [WIDTH-1:0] matmul_masks;

`ifdef USE_INHOUSE_LOGIC
      dpram reg_file1(
	    .clk(clk),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(a_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout)
      );
      defparam
            reg_file1.AWIDTH=LOG2NUMREGS,
            reg_file1.NUM_WORDS=NUMREGS,
            reg_file1.DWIDTH=WIDTH;
      `ifdef TEST_BENCH
        initial begin
		      $readmemh("vregfile_control.dat",reg_file1.ram,'h0);
        end
      `endif 
 `else
	altsyncram	reg_file1(
				.wren_a (c_we),
				.clock0 (clk),
				.address_a (c_reg[LOG2NUMREGS-1:0]),
				.data_a (c_writedatain),
        .rden_b(a_en),
				.address_b (a_reg[LOG2NUMREGS-1:0]),
				.q_b (a_readdataout)
        // synopsys translate_off
        ,
        .aclr0 (1'b0),
        .aclr1 (1'b0),
        .byteena_a (1'b1),
        .byteena_b (1'b1),
        .data_b (32'b11111111),
        .wren_b (1'b0),
        .clock1 (1'b0),
        .clocken1 (1'b0),
        .q_a (),
        .clocken0 (1'b1),
        .addressstall_a (1'b0),
        .addressstall_b (1'b0)
        // synopsys translate_on
    );
	defparam
    `ifdef TEST_BENCH
		reg_file1.init_file = "vregfile_control.dat",
    `else
		reg_file1.init_file = "vregfile_control.mif",
    `endif
		reg_file1.operation_mode = "DUAL_PORT",
		reg_file1.width_a = WIDTH,
		reg_file1.widthad_a = LOG2NUMREGS,
		reg_file1.numwords_a = NUMREGS,
		reg_file1.width_b = WIDTH,
		reg_file1.widthad_b = LOG2NUMREGS,
		reg_file1.numwords_b = NUMREGS,
		reg_file1.lpm_type = "altsyncram",
		reg_file1.width_byteena_a = 1,
		reg_file1.outdata_reg_b = "UNREGISTERED",
		reg_file1.indata_aclr_a = "NONE",
		reg_file1.wrcontrol_aclr_a = "NONE",
		reg_file1.address_aclr_a = "NONE",
		reg_file1.rdcontrol_reg_b = "CLOCK0",
		reg_file1.address_reg_b = "CLOCK0",
		reg_file1.address_aclr_b = "NONE",
		reg_file1.outdata_aclr_b = "NONE",
		reg_file1.read_during_write_mode_mixed_ports = "OLD_DATA",
		reg_file1.ram_block_type = "AUTO",
		reg_file1.intended_device_family = "Stratix";
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
          matmul_masks[1*`MAT_MUL_SIZE-1:0*`MAT_MUL_SIZE] <= c_writedatain[`MAT_MUL_SIZE-1:0];
        end
        else if (c_reg==30) begin //a_cols, b_rows
          matmul_masks[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE] <= c_writedatain[`MAT_MUL_SIZE-1:0];
        end
        else if (c_reg==29) begin //b_cols
          matmul_masks[3*`MAT_MUL_SIZE-1:2*`MAT_MUL_SIZE] <= c_writedatain[`MAT_MUL_SIZE-1:0];
        end
      end  
    end
  end

endmodule

