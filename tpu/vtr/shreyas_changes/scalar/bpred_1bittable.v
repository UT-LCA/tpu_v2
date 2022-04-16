

`define USE_INHOUSE_LOGIC



module dpram (

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



parameter AWIDTH=10;

parameter NUM_WORDS=1024;

parameter DWIDTH=32;



input clk;

input [(AWIDTH-1):0] address_a;

input [(AWIDTH-1):0] address_b;

input  wren_a;

input  wren_b;

input [(DWIDTH-1):0] data_a;

input [(DWIDTH-1):0] data_b;

output reg [(DWIDTH-1):0] out_a;

output reg [(DWIDTH-1):0] out_b;



//remove this 

//`ifndef SIMULATION_MEMORY

// `define SIMULATION_MEMORY

//`endif



`ifdef SIMULATION_MEMORY



reg [DWIDTH-1:0] ram[NUM_WORDS-1:0];



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



module branchpredict ( clk, resetn,

    predict,

    prediction,

    pc_predict,

    result_rdy,

    result,

    pc_result);

parameter PCWIDTH=32;

parameter TABLEDEPTH=4096;

parameter LOG2TABLEDEPTH=12;

parameter TABLEWIDTH=1;



input clk;

input resetn;



// Prediction Port

input predict;                  // When high tells predictor to predict in next cycle

input [PCWIDTH-1:0] pc_predict; // The PC value for which to predict 

output prediction;              // The actual prediction 1-taken, 0-nottaken



// Prediction Result Port - tells us if the prediction made at pc_result was taken

input result_rdy;               // The branch has been resolved when result_rdy goes hi

input [PCWIDTH-1:0] pc_result;  // The PC value that this result is for

input result;                   // The actual result 1-taken, 0-nottaken



wire [LOG2TABLEDEPTH-1:0] address_b;



assign address_b=pc_predict[LOG2TABLEDEPTH+2-1:2];



`ifdef USE_INHOUSE_LOGIC

    dpram pred_table(

	.clk(clk),

	.address_a(pc_result[LOG2TABLEDEPTH+2-1:2]),

	.address_b(address_b),

	.wren_a(result_rdy),

	.wren_b(0),

	.data_a(result),

	.data_b(0),

	.out_a(),

	.out_b(prediction)

    );

    defparam

        pred_table.AWIDTH=LOG2TABLEDEPTH,

        pred_table.NUM_WORDS=TABLEDEPTH,

        pred_table.DWIDTH=TABLEWIDTH;



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



endmodule




