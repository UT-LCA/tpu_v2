from rams import dpram
import parser

class branchpredict():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, pcwidth, tabledepth, log2tabledepth, tablewidth):
        string = '''\
module branchpredict_{PCWIDTH}_{TABLEDEPTH}_{LOG2TABLEDEPTH}_{TABLEWIDTH} ( clk, resetn,
    predict,
    prediction,
    pc_predict,
    result_rdy,
    result,
    pc_result);

input clk;
input resetn;

// Prediction Port
input predict;                  // When high tells predictor to predict in next cycle
input [{PCWIDTH}-1:0] pc_predict; // The PC value for which to predict 
output reg prediction;              // The actual prediction 1-taken, 0-nottaken
wire prediction_temp;

// Prediction Result Port - tells us if the prediction made at pc_result was taken
input result_rdy;               // The branch has been resolved when result_rdy goes hi
input [{PCWIDTH}-1:0] pc_result;  // The PC value that this result is for
input result;                   // The actual result 1-taken, 0-nottaken

wire resetn_nc;
wire predict_nc;
wire [{PCWIDTH}-1:0] pc_predict_local;
wire [{PCWIDTH}-1:0] pc_result_local;

assign resetn_nc = resetn;
assign predict_nc = predict;
assign pc_predict_local = pc_predict;
assign pc_result_local = pc_result;

wire [{LOG2TABLEDEPTH}-1:0] address_b;

assign address_b=pc_predict_local[{LOG2TABLEDEPTH}+2-1:2];

`ifndef USE_INHOUSE_LOGIC
    `define USE_INHOUSE_LOGIC
`endif

`ifdef USE_INHOUSE_LOGIC
    wire [{TABLEWIDTH}-1:0] pred_table_out_a_nc;

    dpram_{LOG2TABLEDEPTH}_{TABLEDEPTH}_{TABLEWIDTH} pred_table(
	.clk(clk),
	.address_a(pc_result_local[{LOG2TABLEDEPTH}+2-1:2]),
	.address_b(address_b),
	.wren_a(result_rdy),
	.wren_b(0),
	.data_a(result),
	.data_b(0),
	.out_a(pred_table_out_a_nc),
	.out_b(prediction_temp)
    );
  // HACK...HACK....HACK
  // Somehow abc was thinking that output of dpram is a combinational port. Though the port is sequential as per the architecture file (agilex_arch.auto_layout.xml). The input address (address_b, pc_predict) comes from the parent module and the output data (out_b, prediction) goes to parent module without any logic in between. The output and input of a dpram are connected combinatoraly in the parent module. So abc thinks that here is a combinatoral loop.
    always @(posedge clk)
    begin
        prediction <= prediction_temp;
    end
`else

	altsyncram	pred_table(
				.clock0 (clk),
				.wren_a (result_rdy),
				.address_a (pc_result[LOG2TABLEDEPTH+2-1:2]),
				.data_a (result),
				.address_b (address_b),
        .clock1 (clk),
        .clocken1 (predict),
				.q_b (prediction)
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
		pred_table.operation_mode = "DUAL_PORT",
		pred_table.width_a = TABLEWIDTH,
		pred_table.widthad_a = LOG2TABLEDEPTH,
		pred_table.numwords_a = TABLEDEPTH,
		pred_table.width_b = TABLEWIDTH,
		pred_table.widthad_b = LOG2TABLEDEPTH,
		pred_table.numwords_b = TABLEDEPTH,
		pred_table.lpm_type = "altsyncram",
		pred_table.width_byteena_a = 1,
		pred_table.outdata_reg_b = "UNREGISTERED",
		pred_table.indata_aclr_a = "NONE",
		pred_table.wrcontrol_aclr_a = "NONE",
		pred_table.address_aclr_a = "NONE",
		pred_table.rdcontrol_reg_b = "CLOCK1",
		pred_table.address_reg_b = "CLOCK1",
		pred_table.address_aclr_b = "NONE",
		pred_table.outdata_aclr_b = "NONE",
		pred_table.read_during_write_mode_mixed_ports = "OLD_DATA",
		pred_table.ram_block_type = "AUTO",
		pred_table.intended_device_family = "Stratix";

`endif
endmodule'''       

        return string.format(PCWIDTH=pcwidth, TABLEDEPTH=tabledepth, LOG2TABLEDEPTH=log2tabledepth, TABLEWIDTH=tablewidth)

    def write (self, pcwidth, tabledepth, log2tabledepth, tablewidth):
        self.fp.write(self.make_str(pcwidth, tabledepth, log2tabledepth, tablewidth))

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = branchpredict(fp)
    uut.write(32, 4096, 12, 1)
    fp.close()
    fp = open(parser.parse(), "a")
    fp.write("\r\r")
    ram = dpram(fp)
    ram.write(12, 4096, 1)
    fp.close()