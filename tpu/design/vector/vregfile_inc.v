/****************************************************************************
          inc Register File

   - Has one read port (a) and one write port (c)
   - inc0 fixed to 0 and unwriteable
****************************************************************************/
module vregfile_inc (
    clk,
    resetn, 

    a_reg, 
    a_en, 
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we
    );

parameter WIDTH=32;
parameter NUMREGS=8;
parameter LOG2NUMREGS=3;

input clk;
input resetn;

input a_en;
input [LOG2NUMREGS-1:0] a_reg,c_reg;
output [WIDTH-1:0] a_readdataout;
input [WIDTH-1:0] c_writedatain;
input c_we;

	altsyncram	reg_file1(
				.wren_a (c_we&(|c_reg)),
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

endmodule

