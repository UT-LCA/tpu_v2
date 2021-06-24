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
    matmul_masks,
    dma_en,
    lane_addr,
    mem_addr,
    num_bytes,
    dma_we,
    dma_busy,
    temp
    );

parameter WIDTH=32;
parameter NUMREGS=32;
parameter LOG2NUMREGS=5;

input clk;
input resetn;

input a_en;
input [LOG2NUMREGS-1:0] a_reg,c_reg;
output reg [WIDTH-1:0] a_readdataout;
input [WIDTH-1:0] c_writedatain;
input c_we;

output reg [WIDTH-1:0] vl;
output reg dma_en;
output reg [WIDTH-1:0] lane_addr;
output reg [WIDTH-1:0] mem_addr;
output reg [WIDTH-1:0] num_bytes;
output reg [WIDTH-1:0] temp;
output reg dma_we;
output reg [3*`MAT_MUL_SIZE-1:0] matmul_masks;
input dma_busy;

wire [WIDTH-1:0] _a_readdataout;

reg mux_sel,next_mux_sel;

always@(posedge clk)begin
  if(!resetn)
    mux_sel <= 1'b0;
  else begin
    if(a_en)
      mux_sel <= next_mux_sel;
  end       
end


always @* begin
  num_bytes[1:0] = 2'h0;
  next_mux_sel = 1'b0;
  if(dma_busy)begin
    if(a_reg == 27)
      next_mux_sel = 1'b1;
    else
      next_mux_sel = 1'b0;
  end
  
  if(mux_sel)
    a_readdataout = dma_busy;
  else
    a_readdataout = _a_readdataout;
end

`ifdef USE_INHOUSE_LOGIC
        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(a_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(_a_readdataout)
      );
      defparam
            reg_file1.AWIDTH=LOG2NUMREGS,
            reg_file1.NUM_WORDS=NUMREGS,
            reg_file1.DWIDTH=WIDTH;
    `ifdef TEST_BENCH
                initial begin
		    $readmemh("vregfile_control.dat",reg_file1.dpram1.ram,'h0);
                end
     `endif 
 `else
  //amana: Somehow this was causing an error with VTR.
  //Since we don't need this anyway, commenting it out.
	//altsyncram	reg_file1(
	//			.wren_a (c_we),
	//			.clock0 (clk),
	//			.address_a (c_reg[LOG2NUMREGS-1:0]),
	//			.data_a (c_writedatain),
  //       .rden_b(a_en),
	//			.address_b (a_reg[LOG2NUMREGS-1:0]),
	//			.q_b (a_readdataout)
  //       // synopsys translate_off
  //       ,
  //       .aclr0 (1'b0),
  //       .aclr1 (1'b0),
  //       .byteena_a (1'b1),
  //       .byteena_b (1'b1),
  //       .data_b (32'b11111111),
  //       .wren_b (1'b0),
  //       .clock1 (1'b0),
  //       .clocken1 (1'b0),
  //       .q_a (),
  //       .clocken0 (1'b1),
  //       .addressstall_a (1'b0),
  //       .addressstall_b (1'b0)
  //       // synopsys translate_on
  //   );
	//defparam
  //   `ifdef TEST_BENCH
	//	reg_file1.init_file = "vregfile_control.dat",
  //   `else
	//	reg_file1.init_file = "vregfile_control.mif",
  //   `endif
	//	reg_file1.operation_mode = "DUAL_PORT",
	//	reg_file1.width_a = WIDTH,
	//	reg_file1.widthad_a = LOG2NUMREGS,
	//	reg_file1.numwords_a = NUMREGS,
	//	reg_file1.width_b = WIDTH,
	//	reg_file1.widthad_b = LOG2NUMREGS,
	//	reg_file1.numwords_b = NUMREGS,
	//	reg_file1.lpm_type = "altsyncram",
	//	reg_file1.width_byteena_a = 1,
	//	reg_file1.outdata_reg_b = "UNREGISTERED",
	//	reg_file1.indata_aclr_a = "NONE",
	//	reg_file1.wrcontrol_aclr_a = "NONE",
	//	reg_file1.address_aclr_a = "NONE",
	//	reg_file1.rdcontrol_reg_b = "CLOCK0",
	//	reg_file1.address_reg_b = "CLOCK0",
	//	reg_file1.address_aclr_b = "NONE",
	//	reg_file1.outdata_aclr_b = "NONE",
	//	reg_file1.read_during_write_mode_mixed_ports = "OLD_DATA",
	//	reg_file1.ram_block_type = "AUTO",
	//	reg_file1.intended_device_family = "Stratix";
  `endif

  always@(posedge clk) begin
    if (!resetn) begin
      vl<=0;
      matmul_masks<=32'hffffffff;
      dma_en <= 0;
      mem_addr <= 0;
      num_bytes[WIDTH-1:2] <= 0;
      lane_addr <= 0;
      dma_we <= 0;
      temp <= 0;
    end
    else begin
      if (c_we) begin
        if (c_reg==0) begin
          vl<=c_writedatain;
        end 
        else if (c_reg==31) begin //a_rows
          matmul_masks[1*`MAT_MUL_SIZE-1:0*`MAT_MUL_SIZE] <= c_writedatain[`MAT_MUL_SIZE-1:0];
          matmul_masks[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE] <= c_writedatain[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE];
          matmul_masks[3*`MAT_MUL_SIZE-1:2*`MAT_MUL_SIZE] <= c_writedatain[3*`MAT_MUL_SIZE-1:2*`MAT_MUL_SIZE];
        end
        else if (c_reg==30) begin 
          dma_en <= c_writedatain[0];
          dma_we <= c_writedatain[1];
          num_bytes[WIDTH-1:2] <= c_writedatain[WIDTH-1:2];
        end
        else if (c_reg==29) begin // Address to the main memory
          mem_addr<=c_writedatain;
        end
        else if (c_reg==28) begin
          lane_addr<=c_writedatain;
        end 
        else if (c_reg==20) begin
          temp<=c_writedatain;
        end 
      end  
    end
  end

endmodule

