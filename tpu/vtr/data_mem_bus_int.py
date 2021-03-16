from components import register
import parser

class data_mem():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, d_addresswidth, dm_datawidth, dm_byteenawidth, dm_addresswidth, dm_size):
        string = '''
/******************************************************************************
            Data memory and interface

Operation table:

  load/store sign size1 size0    |   Operation
7     0       1     1     1      |      LB
5     0       1     0     1      |      LH
0     0       X     0     0      |      LW
3     0       0     1     1      |      LBU
1     0       0     0     1      |      LHU
11    1       X     1     1      |      SB
9     1       X     0     1      |      SH
8     1       X     0     0      |      SW

******************************************************************************/

module data_mem_{D_ADDRESSWIDTH}_{DM_DATAWIDTH}_{DM_BYTEENAWIDTH}_{DM_ADDRESSWIDTH}_{DM_SIZE}( clk, resetn, en, stalled,
    d_writedata,
    d_address,
    op,
    d_loadresult,

    ecause,

    boot_daddr, 
    boot_ddata, 
    boot_dwe, 

    bus_address,
    bus_byteen,
    bus_we,
    bus_en,
    bus_writedata,
    bus_readdata,
    bus_wait,
    bus_ecause
                );

input clk;
input resetn;
input en;
output stalled;

output [31:0] ecause; 

input [31:0] boot_daddr;
input [31:0] boot_ddata;
input boot_dwe;

input [{D_ADDRESSWIDTH}-1:0] d_address;
input [4-1:0] op;
input [{DM_DATAWIDTH}-1:0] d_writedata;
output [{DM_DATAWIDTH}-1:0] d_loadresult;

output [32-1:0] bus_address;
output [4-1:0] bus_byteen;
output         bus_we;
output         bus_en;
output [32-1:0] bus_writedata;
input  [32-1:0] bus_readdata;
input           bus_wait;
input  [32-1:0] bus_ecause;

wire [{DM_BYTEENAWIDTH}-1:0] d_byteena;
wire [{DM_DATAWIDTH}-1:0] d_readdatain;
wire [{DM_DATAWIDTH}-1:0] d_writedatamem;
wire d_write;
wire [1:0] d_address_latched;

// not connected ports
wire resetn_nc;
assign resetn_nc = resetn;
wire [31:0] boot_daddr_nc;
assign boot_daddr_nc = boot_daddr;
wire [31:0] boot_ddata_nc;
assign boot_ddata_nc = boot_ddata;
wire boot_dwe_nc;
assign boot_dwe_nc = boot_dwe;

assign d_write=op[3];

assign ecause=bus_ecause;

register_2 d_address_reg(d_address[1:0],clk,1'b1,en,d_address_latched);
                
store_data_translator_{DM_DATAWIDTH} sdtrans_inst(
    .write_data(d_writedata),
    .d_address(d_address[1:0]),
    .store_size(op[1:0]),
    .d_byteena(d_byteena),
    .d_writedataout(d_writedatamem));

load_data_translator_{DM_DATAWIDTH} ldtrans_inst(
    .d_readdatain(d_readdatain),
    .d_address(d_address_latched[1:0]),
    .load_size(op[1:0]),
    .load_sign_ext(op[2]),
    .d_loadresult(d_loadresult));
  
assign bus_address=d_address;
assign bus_byteen=d_byteena;
assign bus_we=d_write;
assign bus_en=en;
assign bus_writedata=d_writedatamem;
assign d_readdatain=bus_readdata;
assign stalled=bus_wait;

/*
altsyncram  dmem (
            .wren_a (d_write&en&(~d_address[31])),
            .clock0 (clk),
            .clocken0 (),
            .clock1 (clk),
            .clocken1 (boot_dwe),
            `ifdef TEST_BENCH
            .aclr0(~resetn), 
            `endif
            .byteena_a (d_byteena),
            .address_a (d_address[DM_ADDRESSWIDTH+2-1:2]),
            .data_a (d_writedatamem),
            .wren_b (boot_dwe), .data_b (boot_ddata), .address_b (boot_daddr), 
            // synopsys translate_off
            .rden_b (), 
            .aclr1 (), .byteena_b (),
            .addressstall_a (), .addressstall_b (), .q_b (),
            // synopsys translate_on
            .q_a (d_readdatain)
    
);  
    defparam
        dmem.intended_device_family = "Stratix",
        dmem.width_a = DM_DATAWIDTH,
        dmem.widthad_a = DM_ADDRESSWIDTH-2,
        dmem.numwords_a = DM_SIZE,
        dmem.width_byteena_a = DM_BYTEENAWIDTH,
        dmem.operation_mode = "BIDIR_DUAL_PORT",
        dmem.width_b = DM_DATAWIDTH,
        dmem.widthad_b = DM_ADDRESSWIDTH-2,
        dmem.numwords_b = DM_SIZE,
        dmem.width_byteena_b = 1,
        dmem.outdata_reg_a = "UNREGISTERED",
        dmem.address_reg_b = "CLOCK1",
        dmem.wrcontrol_wraddress_reg_b = "CLOCK1",
        dmem.wrcontrol_aclr_a = "NONE",
        dmem.address_aclr_a = "NONE",
        dmem.outdata_aclr_a = "NONE",
        dmem.byteena_aclr_a = "NONE",
        dmem.byte_size = 8,
        `ifdef TEST_BENCH
          dmem.indata_aclr_a = "CLEAR0",
          dmem.init_file = "data.rif",
        `endif
        `ifdef QUARTUS_SIM
          dmem.init_file = "data.mif",
          dmem.ram_block_type = "M4K",
        `else
          dmem.ram_block_type = "MEGARAM",
        `endif
        dmem.lpm_type = "altsyncram";
*/
  
endmodule
'''
        return string.format(D_ADDRESSWIDTH=d_addresswidth, DM_DATAWIDTH=dm_datawidth, DM_BYTEENAWIDTH=dm_byteenawidth,DM_ADDRESSWIDTH=dm_addresswidth, DM_SIZE=dm_size)

    def write (self, d_addresswidth, dm_datawidth, dm_byteenawidth, dm_addresswidth, dm_size):
        self.fp.write(self.make_str(d_addresswidth, dm_datawidth, dm_byteenawidth, dm_addresswidth, dm_size))



class store_data_translator():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string='''
/****************************************************************************
          Store data translator
          - moves store data to appropriate byte/halfword 
          - interfaces with altera blockrams
****************************************************************************/
module store_data_translator_{WIDTH}(
    write_data,             // data in least significant position
    d_address,
    store_size,
    d_byteena,
    d_writedataout);        // shifted data to coincide with address

input [{WIDTH}-1:0] write_data;
input [1:0] d_address;
input [1:0] store_size;
output [3:0] d_byteena;
output [{WIDTH}-1:0] d_writedataout;

reg [3:0] d_byteena;
reg [{WIDTH}-1:0] d_writedataout;

always @(write_data or d_address or store_size)
begin
    case (store_size)
        2'b11:
            case(d_address[1:0])
                0: 
                begin 
                    d_byteena=4'b1000; 
                    d_writedataout={{write_data[7:0],24'b0}}; 
                end
                1: 
                begin 
                    d_byteena=4'b0100; 
                    d_writedataout={{8'b0,write_data[7:0],16'b0}}; 
                end
                2: 
                begin 
                    d_byteena=4'b0010; 
                    d_writedataout={{16'b0,write_data[7:0],8'b0}}; 
                end
                default: 
                begin 
                    d_byteena=4'b0001; 
                    d_writedataout={{24'b0,write_data[7:0]}}; 
                end
            endcase
        2'b01:
            case(d_address[1])
                0: 
                begin 
                    d_byteena=4'b1100; 
                    d_writedataout={{write_data[15:0],16'b0}}; 
                end
                default: 
                begin 
                    d_byteena=4'b0011; 
                    d_writedataout={{16'b0,write_data[15:0]}}; 
                end
            endcase
        default:
        begin
            d_byteena=4'b1111;
            d_writedataout=write_data;
        end
    endcase
end
endmodule
'''

        return string.format(WIDTH=width)

    def write (self, width):
        self.fp.write(self.make_str(width))


class load_data_translator():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, width):
        string='''
/****************************************************************************
          Load data translator
          - moves read data to appropriate byte/halfword and zero/sign extends
****************************************************************************/
module load_data_translator_{WIDTH}(
    d_readdatain,
    d_address,
    load_size,
    load_sign_ext,
    d_loadresult);

input [{WIDTH}-1:0] d_readdatain;
input [1:0] d_address;
input [1:0] load_size;
input load_sign_ext;
output [{WIDTH}-1:0] d_loadresult;

reg [{WIDTH}-1:0] d_loadresult;

always @(d_readdatain or d_address or load_size or load_sign_ext)
begin
    case (load_size)
        2'b11:
        begin
            case (d_address[1:0])
                0: d_loadresult[7:0]=d_readdatain[31:24];
                1: d_loadresult[7:0]=d_readdatain[23:16];
                2: d_loadresult[7:0]=d_readdatain[15:8];
                default: d_loadresult[7:0]=d_readdatain[7:0];
            endcase
            d_loadresult[31:8]={{24{{load_sign_ext&d_loadresult[7]}}}};
        end
        2'b01:
        begin
            case (d_address[1])
                0: d_loadresult[15:0]=d_readdatain[31:16];
                default: d_loadresult[15:0]=d_readdatain[15:0];
            endcase
            d_loadresult[31:16]={{16{{load_sign_ext&d_loadresult[15]}}}};
        end
        default:
            d_loadresult=d_readdatain;
    endcase
end

endmodule
'''       

        return string.format(WIDTH=width)

    def write (self, width):
        self.fp.write(self.make_str(width))

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = data_mem(fp)
    uut.write(32, 32, 4, 16, 16384)
    fp.close()
    fp = open(parser.parse(), "a")
    fp.write("\r\r")
    store = store_data_translator(fp)
    store.write(32)
    fp.write("\r\r")
    load = load_data_translator(fp)
    load.write(32) 
    fp.write("\r\r")
    reg = register(fp)
    reg.write(2) 
    fp.close()