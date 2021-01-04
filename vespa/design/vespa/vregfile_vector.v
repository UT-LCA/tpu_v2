/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_vector(clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_byteen, c_we);

parameter NUMBANKS=1;
parameter LOG2NUMBANKS=0;
parameter WIDTH=32;
parameter NUMREGS=32;
parameter LOG2NUMREGS=5;

parameter NUMREGSPERBANK=NUMREGS/NUMBANKS;
parameter LOG2NUMREGSPERBANK=LOG2NUMREGS-LOG2NUMBANKS;

input clk;
input resetn;

input [NUMBANKS-1:0] a_en;
input [NUMBANKS-1:0] b_en;
input [NUMBANKS*LOG2NUMREGSPERBANK-1:0] a_reg,b_reg,c_reg;
output [NUMBANKS*WIDTH-1:0] a_readdataout, b_readdataout;
input [NUMBANKS*WIDTH-1:0] c_writedatain;
input [((WIDTH>=8) ? NUMBANKS*WIDTH/8-1 : NUMBANKS-1):0] c_byteen;
input [NUMBANKS-1:0] c_we;

  genvar k;

  generate
  for (k=0; k<NUMBANKS; k=k+1)
  begin : bank_gen

    altsyncram	reg_file1(
          .clock0 (clk),
          .wren_a (c_we[k] & ((WIDTH>8) ? 1'b1 : c_byteen[k])),
          .data_a (c_writedatain[k*WIDTH +: WIDTH]),
          //Not allowed to use this port when width_byteena_a is <= 1
          .byteena_a ( (WIDTH>8) ? c_byteen[k*WIDTH/8 +: WIDTH/8] : 1'b1 ),
          .rden_b (a_en[k]),
          .address_a (c_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .address_b (a_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .q_b (a_readdataout[k*WIDTH +: WIDTH])
          // synopsys translate_off
          ,
          .clock1 (1'b0),
          .aclr0 (1'b0),
          .aclr1 (1'b0),
          .byteena_b (1'b1),
          .data_b (),
          .wren_b (1'b0),
          .clocken1(1'b1),
          .q_a (),
          .clocken0 (1'b1),
          .addressstall_a (1'b0),
          .addressstall_b (1'b0)
          // synopsys translate_on
      );
    defparam
      reg_file1.operation_mode = "DUAL_PORT",
      reg_file1.width_a = WIDTH,
      reg_file1.widthad_a = LOG2NUMREGSPERBANK,
      reg_file1.numwords_a = NUMREGSPERBANK,
      reg_file1.width_b = WIDTH,
      reg_file1.widthad_b = LOG2NUMREGSPERBANK,
      reg_file1.numwords_b = NUMREGSPERBANK,
      reg_file1.lpm_type = "altsyncram",
      reg_file1.width_byteena_a = (WIDTH>=8) ? WIDTH/8 : 1,
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

      //Reg file duplicated to avoid contention between 2 read
      //and 1 write
    altsyncram	reg_file2(
          .clock0 (clk),
          .wren_a (c_we[k] & ((WIDTH>8) ? 1'b1 : c_byteen[k]) ),
          .data_a (c_writedatain[k*WIDTH +: WIDTH]),
          //Not allowed to use this port when width_byteena_a is <= 1
          .byteena_a ( (WIDTH>8) ? c_byteen[k*WIDTH/8 +: WIDTH/8] : 1'b1 ),
          .address_a (c_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .rden_b (b_en[k]),
          .address_b (b_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .q_b (b_readdataout[k*WIDTH +: WIDTH])
          // synopsys translate_off
          ,
          .clock1 (1'b0),
          .aclr0 (1'b0),
          .aclr1 (1'b0),
          .byteena_b (1'b1),
          .data_b (),
          .clocken1(1'b1),
          .wren_b (1'b0),
          .q_a (),
          .clocken0 (1'b1),
          .addressstall_a (1'b0),
          .addressstall_b (1'b0)
          // synopsys translate_on
      );
    defparam
      reg_file2.operation_mode = "DUAL_PORT",
      reg_file2.width_a = WIDTH,
      reg_file2.widthad_a = LOG2NUMREGSPERBANK,
      reg_file2.numwords_a = NUMREGSPERBANK,
      reg_file2.width_b = WIDTH,
      reg_file2.widthad_b = LOG2NUMREGSPERBANK,
      reg_file2.numwords_b = NUMREGSPERBANK,
      reg_file2.lpm_type = "altsyncram",
      reg_file2.width_byteena_a = (WIDTH>=8) ? WIDTH/8 : 1,
      reg_file2.outdata_reg_b = "UNREGISTERED",
      reg_file2.indata_aclr_a = "NONE",
      reg_file2.wrcontrol_aclr_a = "NONE",
      reg_file2.address_aclr_a = "NONE",
      reg_file2.rdcontrol_reg_b = "CLOCK0",
      reg_file2.address_reg_b = "CLOCK0",
      reg_file2.address_aclr_b = "NONE",
      reg_file2.outdata_aclr_b = "NONE",
      reg_file2.read_during_write_mode_mixed_ports = "OLD_DATA",
      reg_file2.ram_block_type = "AUTO",
      reg_file2.intended_device_family = "Stratix";

  end
  endgenerate

endmodule

