from rams import dpram
from ram_wrapper import ram_wrapper
import parser

class reg_file():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width, numregs, log2numregs):
        string = '''\
/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
`include "options.v"
module reg_file_{WIDTH}_{NUMREGS}_{LOG2NUMREGS}(clk,resetn, c_squashn,
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_we);

input clk;
input resetn;

input a_en;
input b_en;

input [{LOG2NUMREGS}-1:0] a_reg,b_reg,c_reg;
output [{WIDTH}-1:0] a_readdataout, b_readdataout;
input [{WIDTH}-1:0] c_writedatain;
input c_we;
input c_squashn;
reg [31:0] i;

`ifndef USE_INHOUSE_LOGIC1
	`define USE_INHOUSE_LOGIC1
`endif

`ifdef USE_INHOUSE_LOGIC1

wire [{WIDTH}-1:0] reg_file1_out_a_nc;
wire reg_file1_wren_a;

assign reg_file1_wren_a = c_we & (|c_reg) & c_squashn;

        ram_wrapper_{LOG2NUMREGS}_{NUMREGS}_{WIDTH} reg_file1(
	    .clk(clk),
		.resetn(resetn),
		.rden_a(1'b0),
		.rden_b(a_en),
	    .address_a(c_reg[{LOG2NUMREGS}-1:0]),
	    .address_b(a_reg[{LOG2NUMREGS}-1:0]),
	    .wren_a(reg_file1_wren_a),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(reg_file1_out_a_nc),
	    .out_b(a_readdataout)
        );
 
// initial begin
//    for(i=0;i<{NUMREGS};i=i+1)
//        reg_file1.dpram1.ram[i]=0;
// end 
         
wire [{WIDTH}-1:0] reg_file2_out_a_nc;
wire reg_file2_wren_a;

assign reg_file2_wren_a = c_we & (|c_reg);
        ram_wrapper_{LOG2NUMREGS}_{NUMREGS}_{WIDTH} reg_file2(
	    .clk(clk),
		.resetn(resetn),
		.rden_a(1'b0),
		.rden_b(b_en),
	    .address_a(c_reg[{LOG2NUMREGS}-1:0]),
	    .address_b(b_reg[{LOG2NUMREGS}-1:0]),
	    .wren_a(reg_file2_wren_a),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(reg_file2_out_a_nc),
	    .out_b(b_readdataout)
        );

// initial begin
//    for(i=0;i<{NUMREGS};i=i+1)
//        reg_file2.dpram1.ram[i]=0;
// end 
`else

	altsyncram	reg_file1(
				.wren_a (c_we & (|c_reg) & c_squashn),
				.clock0 (clk),
        .clock1 (clk),
        .clocken1 (a_en),
				.address_a (c_reg[{LOG2NUMREGS}-1:0]),
				.address_b (a_reg[{LOG2NUMREGS}-1:0]),
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
		reg_file1.width_a = {WIDTH},
		reg_file1.widthad_a = {LOG2NUMREGS},
		reg_file1.numwords_a = {NUMREGS},
		reg_file1.width_b = {WIDTH},
		reg_file1.widthad_b = {LOG2NUMREGS},
		reg_file1.numwords_b = {NUMREGS},
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
				.address_a (c_reg[{LOG2NUMREGS}-1:0]),
				.address_b (b_reg[{LOG2NUMREGS}-1:0]),
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
		reg_file2.width_a = {WIDTH},
		reg_file2.widthad_a = {LOG2NUMREGS},
		reg_file2.numwords_a = {NUMREGS},
		reg_file2.width_b = {WIDTH},
		reg_file2.widthad_b = {LOG2NUMREGS},
		reg_file2.numwords_b = {NUMREGS},
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

endmodule'''       

        return string.format(WIDTH=width, NUMREGS=numregs, LOG2NUMREGS=log2numregs)

    def write (self, width, numregs, log2numregs):
        self.fp.write(self.make_str(width, numregs, log2numregs))

if __name__ == '__main__':
	fp = open(parser.parse(), "w")
	uut = reg_file(fp)
	uut.write(32, 32, 5)
	fp.close()
	fp = open(parser.parse(), "a")
	fp.write("\r\r")
	wrapper = ram_wrapper(fp)
	wrapper.write(5, 32, 32)
	fp.write("\r\r")
	ram = dpram(fp)
	ram.write(5, 32, 32)
	fp.close()