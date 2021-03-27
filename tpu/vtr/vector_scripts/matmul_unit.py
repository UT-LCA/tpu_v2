from matmul_8x8 import matmul_8x8
from matmul_defines import matmul_defines
from vcomponents import pipe
from optparse import OptionParser

parser = OptionParser()
(_,args) = parser.parse_args()

class matmul_unit():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, regidwidth, pipe_stages_matmul, numlanes):
        string = '''\

module matmul_unit_{REGIDWIDTH}_{PIPE_STAGES_MATMUL}_{NUMLANES}(
 clk,
 resetn,
 activate,
 en,
 squash,
 stall,
 a_data,
 b_data,
 validity_mask_a_rows,
 validity_mask_a_cols,
 validity_mask_b_rows,
 validity_mask_b_cols,
 c_data, 
 vmask,
 in_dst,
 in_dst_we,
 out_dst,
 out_dst_we,
 out_dst_mask
);

 input clk;
 input resetn;
 input activate;
 input [{PIPE_STAGES_MATMUL}:1] en;  //Enable for each pipestage
 input [{PIPE_STAGES_MATMUL}:1] squash;  //Squash for each pipestage
 output stall;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data;
 input [`MASK_WIDTH-1:0] validity_mask_a_rows;
 input [`MASK_WIDTH-1:0] validity_mask_a_cols;
 input [`MASK_WIDTH-1:0] validity_mask_b_rows;
 input [`MASK_WIDTH-1:0] validity_mask_b_cols;

 output [`MAT_MUL_SIZE*`DWIDTH-1:0] c_data;
 input [{NUMLANES}-1:0] vmask;

 input    [{REGIDWIDTH}-1:0] in_dst;
 input                     in_dst_we;
 output [{PIPE_STAGES_MATMUL}*{REGIDWIDTH}-1:0] out_dst;
 output            [{PIPE_STAGES_MATMUL}-1:0] out_dst_we;
 output   [{PIPE_STAGES_MATMUL}*{NUMLANES}-1:0] out_dst_mask;

wire [{PIPE_STAGES_MATMUL}:1] ctrl_activate;
wire [{PIPE_STAGES_MATMUL}:1] squash_activatepipe_NC;
pipe_1_{PIPE_STAGES_MATMUL_MINUS_ONE} activatepipe (
    .d(activate),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(squash_activatepipe_NC),
    .q(ctrl_activate));

wire in_progress;
assign stall=in_progress;

//The actual matrix multiplier block. This is an abridged
//version of the matmul block used in the TPU v1 design.
matmul_8x8 mat(
    .clk(clk),
    .reset(~resetn),
    .pe_reset(~resetn),
    .start(activate),
    .in_progress(in_progress),
    .done(done_NC),
    .a_data(a_data),
    .b_data(b_data),
    .c_data_out(c_data), 
    .c_data_available(c_data_available),
    .validity_mask_a_rows(validity_mask_a_rows),
    .validity_mask_a_cols(validity_mask_a_cols),
    .validity_mask_b_rows(validity_mask_b_rows),
    .validity_mask_b_cols(validity_mask_b_cols)
);

wire [{PIPE_STAGES_MATMUL}:1] squash_dstpipe_NC;
pipe_{REGIDWIDTH}_{PIPE_STAGES_MATMUL_MINUS_ONE} dstpipe (
    .d(in_dst ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[{PIPE_STAGES_MATMUL}-1:1] & {{1'b1,{{({PIPE_STAGES_MATMUL}-2){{~stall}}}}}} ),
    .squash(squash_dstpipe_NC),
    .q(out_dst));

pipe_1_{PIPE_STAGES_MATMUL_MINUS_ONE} dstwepipe (
    .d(in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[{PIPE_STAGES_MATMUL}-1:1] & {{1'b1,{{({PIPE_STAGES_MATMUL}-2){{~stall}}}}}} ),
    .squash(squash[{PIPE_STAGES_MATMUL}-1:1]),
    .q(out_dst_we));

wire [{PIPE_STAGES_MATMUL}:1] squash_dstmaskpipe_NC;
pipe_{NUMLANES}_{PIPE_STAGES_MATMUL_MINUS_ONE} dstmaskpipe (
    .d(vmask ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[{PIPE_STAGES_MATMUL}-1:1] & {{1'b1,{{({PIPE_STAGES_MATMUL}-2){{~stall}}}}}} ),
    .squash(squash_dstmaskpipe_NC),
    .q(out_dst_mask));
endmodule'''

        fp = open("verilog/vcomponents.v", "a")
        uut = pipe(fp)
        uut.write(1, pipe_stages_matmul-1)
        fp.write("\n\n")
        uut.write(regidwidth, pipe_stages_matmul-1)
        uut.write(numlanes, pipe_stages_matmul-1)
        fp.close()
        fp = open("verilog/matmul_8x8.v", "a")
        uut = matmul_8x8(fp)
        uut.write()
        fp.close()

        return string.format(REGIDWIDTH=regidwidth, PIPE_STAGES_MATMUL=pipe_stages_matmul, NUMLANES=numlanes, PIPE_STAGES_MATMUL_MINUS_ONE = pipe_stages_matmul-1)

    def write(self, regidwidth, pipe_stages_matmul, numlanes):
        self.fp.write(self.make_str(regidwidth, pipe_stages_matmul, numlanes))


if __name__ == '__main__':
    fp = open(args[0], "w")
    uut = matmul_unit(fp)
    uut.write(10, 29, 4)
    fp.close()