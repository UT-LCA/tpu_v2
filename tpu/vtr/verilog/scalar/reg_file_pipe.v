/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
`include "options.v"
module reg_file_32_32_5(clk,resetn, c_squashn,
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_we);

input clk;
input resetn;

input a_en;
input b_en;

input [5-1:0] a_reg,b_reg,c_reg;
output [32-1:0] a_readdataout, b_readdataout;
input [32-1:0] c_writedatain;
input c_we;
input c_squashn;
reg [31:0] i;

`ifndef USE_INHOUSE_LOGIC1
	`define USE_INHOUSE_LOGIC1
`endif

`ifdef USE_INHOUSE_LOGIC1

wire [32-1:0] reg_file1_out_a_nc;
wire reg_file1_wren_a;

assign reg_file1_wren_a = c_we & (|c_reg) & c_squashn;

        ram_wrapper_5_32_32 reg_file1(
	    .clk(clk),
		.resetn(resetn),
		.rden_a(1'b0),
		.rden_b(a_en),
	    .address_a(c_reg[5-1:0]),
	    .address_b(a_reg[5-1:0]),
	    .wren_a(reg_file1_wren_a),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(reg_file1_out_a_nc),
	    .out_b(a_readdataout)
        );
 
// initial begin
//    for(i=0;i<32;i=i+1)
//        reg_file1.dpram1.ram[i]=0;
// end 
         
wire [32-1:0] reg_file2_out_a_nc;
wire reg_file2_wren_a;

assign reg_file2_wren_a = c_we & (|c_reg);
        ram_wrapper_5_32_32 reg_file2(
	    .clk(clk),
		.resetn(resetn),
		.rden_a(1'b0),
		.rden_b(b_en),
	    .address_a(c_reg[5-1:0]),
	    .address_b(b_reg[5-1:0]),
	    .wren_a(reg_file2_wren_a),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(reg_file2_out_a_nc),
	    .out_b(b_readdataout)
        );

// initial begin
//    for(i=0;i<32;i=i+1)
//        reg_file2.dpram1.ram[i]=0;
// end 
`else

	altsyncram	reg_file1(
				.wren_a (c_we & (|c_reg) & c_squashn),
				.clock0 (clk),
        .clock1 (clk),
        .clocken1 (a_en),
				.address_a (c_reg[5-1:0]),
				.address_b (a_reg[5-1:0]),
				.data_a (c_writedatain),
				.q_b (a_readdataout)
        // synopsys translate_off
        ,
        .aclr0 (1'b0),
        .aclr1 (1'b0),
        .byteena_a (1'b1),
        .byteena_b (1'b1),
        .data_b (32'b11111111),
        .wren_b (1'b0),
        .rden_b(1'b1),
        .q_a (),
        .clocken0 (1'b1),
        .addressstall_a (1'b0),
        .addressstall_b (1'b0)
        // synopsys translate_on
    );
	defparam
		reg_file1.operation_mode = "DUAL_PORT",
		reg_file1.width_a = 32,
		reg_file1.widthad_a = 5,
		reg_file1.numwords_a = 32,
		reg_file1.width_b = 32,
		reg_file1.widthad_b = 5,
		reg_file1.numwords_b = 32,
		reg_file1.lpm_type = "altsyncram",
		reg_file1.width_byteena_a = 1,
		reg_file1.outdata_reg_b = "UNREGISTERED",
		reg_file1.indata_aclr_a = "NONE",
		reg_file1.wrcontrol_aclr_a = "NONE",
		reg_file1.address_aclr_a = "NONE",
		reg_file1.rdcontrol_reg_b = "CLOCK1",
		reg_file1.address_reg_b = "CLOCK1",
		reg_file1.address_aclr_b = "NONE",
		reg_file1.outdata_aclr_b = "NONE",
		reg_file1.read_during_write_mode_mixed_ports = "OLD_DATA",
		reg_file1.ram_block_type = "AUTO",
		reg_file1.intended_device_family = "Stratix";

		//Reg file duplicated to avoid contention between 2 read
		//and 1 write
	altsyncram	reg_file2(
				.wren_a (c_we&(|c_reg)),
				.clock0 (clk),
        .clock1 (clk),
        .clocken1 (b_en),
				.address_a (c_reg[5-1:0]),
				.address_b (b_reg[5-1:0]),
				.data_a (c_writedatain),
				.q_b (b_readdataout)
        // synopsys translate_off
        ,
        .aclr0 (1'b0),
        .aclr1 (1'b0),
        .byteena_a (1'b1),
        .byteena_b (1'b1),
        .data_b (32'b11111111),
        .rden_b(1'b1),
        .wren_b (1'b0),
        .q_a (),
        .clocken0 (1'b1),
        .addressstall_a (1'b0),
        .addressstall_b (1'b0)
        // synopsys translate_on
    );
	defparam
		reg_file2.operation_mode = "DUAL_PORT",
		reg_file2.width_a = 32,
		reg_file2.widthad_a = 5,
		reg_file2.numwords_a = 32,
		reg_file2.width_b = 32,
		reg_file2.widthad_b = 5,
		reg_file2.numwords_b = 32,
		reg_file2.lpm_type = "altsyncram",
		reg_file2.width_byteena_a = 1,
		reg_file2.outdata_reg_b = "UNREGISTERED",
		reg_file2.indata_aclr_a = "NONE",
		reg_file2.wrcontrol_aclr_a = "NONE",
		reg_file2.address_aclr_a = "NONE",
		reg_file2.rdcontrol_reg_b = "CLOCK1",
		reg_file2.address_reg_b = "CLOCK1",
		reg_file2.address_aclr_b = "NONE",
		reg_file2.outdata_aclr_b = "NONE",
		reg_file2.read_during_write_mode_mixed_ports = "OLD_DATA",
		reg_file2.ram_block_type = "AUTO",
		reg_file2.intended_device_family = "Stratix";

`endif

endmodulemodule ram_wrapper_5_32_32 (
	clk,
        resetn,
	address_a,
	address_b,
        rden_a,
        rden_b,
	wren_a,
	wren_b,
	data_a,
	data_b,
	out_a,
	out_b
);

input clk;
input resetn;
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output [(32-1):0] out_a;
output [(32-1):0] out_b;

reg [(5-1):0] q_address_a;
reg [(5-1):0] q_address_b;
reg [(5-1):0] mux_address_b;

dpram_5_32_32 dpram1(
    .clk(clk),
    .address_a(address_a),
    .address_b(mux_address_b),
    .wren_a(wren_a),
    .wren_b(wren_b),
    .data_a(data_a),
    .data_b(data_b),
    .out_a(out_a),
    .out_b(out_b)
);

always@(posedge clk)begin
   if(!resetn)begin
     q_address_a <= 'h0;
     q_address_b <= 'h0;
   end
   else begin
     if(rden_b)
       q_address_b <= address_b;
   end
end

always@(*)begin
  if(rden_b)   
    mux_address_b = address_b;
  else
    mux_address_b = q_address_b; 
end

endmodulemodule dpram_5_32_32 (
	clk,
	address_a,
	address_b,
	wren_a,
	wren_b,
	data_a,
	data_b,
	out_a,
	out_b
);

input clk;
input [(5-1):0] address_a;
input [(5-1):0] address_b;
input  wren_a;
input  wren_b;
input [(32-1):0] data_a;
input [(32-1):0] data_b;
output reg [(32-1):0] out_a;
output reg [(32-1):0] out_b;

`ifdef SIMULATION_MEMORY

reg [32-1:0] ram[32-1:0];

always @ (posedge clk) begin 
  if (wren_a) begin
      ram[address_a] <= data_a;
  end
  else begin
      out_a <= ram[address_a];
  end
end
  
always @ (posedge clk) begin 
  if (wren_b) begin
      ram[address_b] <= data_b;
  end 
  else begin
      out_b <= ram[address_b];
  end
end

`else

dual_port_ram u_dual_port_ram(
.addr1(address_a),
.we1(wren_a),
.data1(data_a),
.out1(out_a),
.addr2(address_b),
.we2(wren_b),
.data2(data_b),
.out2(out_b),
.clk(clk)
);

`endif

endmodule