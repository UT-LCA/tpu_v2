/****************************************************************************
            Fetch Unit with branch prediction

  IMPORTANT: tgt_pc should arrive 1 cycle after instruction to account for delay slot.
  Also, we compress the prediction PC's by one bit since delay slots prevent consecutive branches.
            
op
  0  Conditional PC write
  1  UnConditional PC write

****************************************************************************/

module ifetch_67108896_32_14_16384(clk,resetn,
        en,         // enable increment (stage 1)
        squashn,
        we,         // enable pc update (later stage)
        op,
        load,
        load_data,

        pcwrop,     // differentiates between unconditionals: 1-unconditional
        predict_tgt_pc,
        predict_en, // enable pc update (early prediction stage)
        predict_result_rdy,
        predict_result,

        interrupt,
        epc,
        ecause,

        pc_out,
        next_pc,

  boot_iaddr, 
  boot_idata, 
  boot_iwe,

    bus_address,
    bus_en,
    bus_readdata,
    bus_wait,
    bus_squashn,
    bus_ecause,

        opcode,
        rs,
        rt,
        rd,
        sa,
        offset,
        instr_index,
        func,
        instr);

input [31:0] boot_iaddr;
input [31:0] boot_idata;
input boot_iwe;

output [32-1:0] bus_address;
output         bus_en;
input  [32-1:0] bus_readdata;
input           bus_wait;
output         bus_squashn;
input  [32-1:0] bus_ecause;

input clk;
input resetn;
input en;     // PC increment enable
input we;     // PC write enable
input squashn;// squash fetch
input op;     // determines if conditional or unconditional branch
input load;
input [32-1:0] load_data;

input pcwrop;
input [32-1:0] predict_tgt_pc;
input predict_en;
input predict_result_rdy;
output predict_result;

input  interrupt; 
output [32-1:0] epc; 
output [31:0] ecause; 

output [32-1:0] pc_out;   // output pc + 1 shifted left 2 bits
output [32-1:0] next_pc;
output [31:26] opcode;
output [25:21] rs;
output [20:16] rt;
output [15:11] rd;
output [10:6] sa;
output [15:0] offset;
output [25:0] instr_index;
output [5:0] func;
output [32-1:0] instr;


wire [32-1:0] pc_plus_1;
reg [32-1:0] pc;
wire ctrl_load;

// prediction stuff
wire prediction;
wire prediction_saved;
wire predict_en_saved;
wire [32-1:0] pc_rollbacknottaken;
wire [32-1:0] pc_rollback;

reg [32-1:0] _next_pc;
reg pc_load_en;
reg predict_result;

// tolerating waits stuff
reg [32-1:0] pc_save_data;
reg pc_save;
wire pc_advance;

wire is_branch;
wire is_delayslot;  //Tells us if we're fetching a delay slot

assign pc_advance=(en&~bus_wait);

assign ctrl_load=(load&~op|op);
  
assign bus_address=next_pc;
assign bus_en=(en|~squashn)&resetn;
assign bus_squashn=squashn;

/******
* WARNING: pipeline-specific - because we know the stage after fetching never
* stalls, we don't need to freeze the result.  So 'en' is a don't care here
******/
assign instr=(bus_wait) ? 'h021 : bus_readdata;

/*  DEBUG: using onchip memory and synthetic stalls
assign instr=(imem_wait) ? 'h021 : imem_readdata;

reg [5:0] count;
  always@(posedge clk)
  if (!resetn)
    count<=0;
  else
    count<=count+1;
assign imem_wait=(count[2]);
altsyncram  imem (
    .clock0 (clk),
    .clocken0 (bus_en|~resetn),
    .clock1 (clk),                              // changed
    .clocken1 (boot_iwe),                       // changed
    `ifdef TEST_BENCH
    .aclr0(~resetn), 
    `endif
    .address_a (next_pc[32-1:2]),
    .wren_b (boot_iwe), .data_b (boot_idata), .address_b (boot_iaddr), //changed

    // synopsys translate_off
    .wren_a (), .rden_b (), .data_a (), 
    .aclr1 (), .byteena_a (), .byteena_b (),
    .addressstall_a (), .addressstall_b (), .q_b (),
    // synopsys translate_on
    
    .q_a (imem_readdata)
    );
    defparam
        imem.intended_device_family = "Stratix",
        imem.width_a = 32, 
        imem.widthad_a = 14,
        imem.numwords_a = 16384,
        imem.operation_mode = "BIDIR_DUAL_PORT",    // changed
        imem.width_b = 32,                 // new
        imem.widthad_b = 14,            // new
        imem.numwords_b = 16384,                   // new
        imem.outdata_reg_b = "UNREGISTERED",
        imem.outdata_reg_a = "UNREGISTERED",
        imem.address_reg_b = "CLOCK1",              // new
        imem.wrcontrol_wraddress_reg_b = "CLOCK1",  // new
        imem.width_byteena_a = 1,
        `ifdef TEST_BENCH
        imem.address_aclr_a = "CLEAR0",
        imem.outdata_aclr_a = "CLEAR0",
        imem.init_file = "instr.rif",
        `endif
        `ifdef QUARTUS_SIM
          imem.init_file = "instr.mif",
          imem.ram_block_type = "AUTO",
        `else
          imem.ram_block_type = "MEGARAM",
        `endif
        imem.lpm_type = "altsyncram";
*/

wire dummy;

assign {dummy,pc_plus_1} = pc + 4;
assign pc_out=pc_plus_1;

//For delay slot instruction - point to branch
assign epc=(is_delayslot) ? pc - 4 : pc;
//Insert your interrupt pending flags here 
assign ecause=bus_ecause;

assign opcode=instr[31:26];
assign rs=instr[25:21];
assign rt=instr[20:16];
assign rd=instr[15:11];
assign sa=instr[10:6];
assign offset=instr[15:0]; 
assign instr_index=instr[25:0];
assign func=instr[5:0];

//************** BRANCH PREDICTION stuff ************
// When predict_en is asserted we accept the prediction signal's value and
// adjust the PC accordingly.  In addition, we buffer everything needed to
// verify and rollback.  
// When predict_result_rdy is asserted we examine ctrl_load and compare the
// pipeline's intended change to the PC with the one we've done here.
// Note that prediction must happen on last delay slot instruction meaning that
// both the predict_en assertion and tgt_pc must come at that time


// Backup PC for both taken/not taken paths
wire [32-1:0] pcrollback_d;
wire pcrollback_en;

assign pcrollback_d = (prediction) ? pc_plus_1 : predict_tgt_pc;
assign pcrollback_en = en&predict_en;

register_32 pcrollback( pcrollback_d, 
    clk, resetn, pcrollback_en, pc_rollback);
  //defparam pcrollback.WIDTH=32;

wire pcrollbacknt_en;
assign pcrollbacknt_en = en&predict_en;

register_32 pcrollbacknt( pc, clk, resetn, pcrollbacknt_en, pc_rollbacknottaken);
 // defparam pcrollbacknt.WIDTH=32;

//register_32 pcrollbacktk(predict_tgt_pc, clk, resetn,predict_en, 
    //pc_rollback);
  //defparam pcrollbacktk.WIDTH=32;

wire [2-1: 0] buf_predict_d;
wire buf_predict_en;

assign buf_predict_d = {prediction,predict_en&(pcwrop!=1'b1)};
assign buf_predict_en = en&predict_en;

register_2 buf_predict(buf_predict_d,clk,resetn,buf_predict_en, 
    {prediction_saved,predict_en_saved});
  //defparam buf_predict.WIDTH=2;
  //predict_en_saved saves if it wa

/*** Saving Business
 * When a write to the PC happens deeper in the pipe while the ifetch is frozen
 * we originally stalled that branch also.  Now we save the new PC and load it
 * in once the ifetch becomes available.
 */
reg squash_save;
always@(posedge clk or negedge resetn) 
  if (!resetn)
    squash_save<=0;
  else if ( ~squashn || pc_advance)  // Capture squash requests when we're busy
    squash_save<=~squashn&~pc_advance;

always@(posedge clk or negedge resetn) 
  if (!resetn)
    pc_save<=0;
  else if ( pc_load_en || pc_advance)  // Capture we and advance to clear it
    pc_save<=pc_load_en&~pc_advance;  // zero the save if we're enabled

always@(posedge clk or negedge resetn)
  if (!resetn)
    pc_save_data<=0;
  else if (pc_load_en)  // Capture we, and advance to clear captured data
    pc_save_data<=_next_pc;


always@(posedge clk or negedge resetn)
  if (!resetn)
    pc<='h400_0000;                 // 0x400_0000/4
  else if (pc_advance)
    pc<=_next_pc;


reg [2:0] debug;

//always@(prediction_saved or predict_en_saved or prediction or en or predict_en or ctrl_load or predict_result_rdy or pc_plus_1 or load_data or we or predict_tgt_pc or pc_rollback or pc_rollbacknottaken or pc or pc_advance)
always@*
  begin
    if (interrupt)
    begin   // If interrupt occurs, jump to 67108896
      _next_pc=67108896;
      pc_load_en=1;
      debug=5;
    end
    else if (ctrl_load & !predict_result_rdy)
    begin   // No prediction, do indirect jump
      _next_pc=load_data;
      pc_load_en=1;
      debug=1;
    end
    else if (predict_en & prediction)
    begin   // Make a prediction to take
      _next_pc=predict_tgt_pc;
      pc_load_en=en;
      debug=2;
    end
    else if (predict_result_rdy & (ctrl_load!=prediction_saved) & predict_en_saved)
    begin   // Mispredict - restore branch
      _next_pc=pc_rollback;
      pc_load_en=1;
      debug=3;
    end
    else if (pc_save)
    begin   // If bus is stalled and a pc load happens, save it and restore it
            // once the bus unstalls, deal with the squash + protect delay slot
      _next_pc=pc_save_data;
      pc_load_en=pc_advance;
      debug=4;
    end
    else
    begin
        _next_pc=pc_plus_1;
        pc_load_en=0;
        debug=0;
    end
end

// Separated from above always block because not dependent on pc_advance
always@*
  begin
    if (ctrl_load & !predict_result_rdy)
      predict_result=~we;  // Only squash proc pipeline when not stalled
    else if (predict_result_rdy & (ctrl_load!=prediction_saved) & predict_en_saved)
      predict_result=~we;
    else if (pc_save)
      predict_result=~(squash_save&~is_delayslot);
    else
      predict_result=1;  // Used to flush pipe
end

assign next_pc=(pc_advance) ? _next_pc : pc;


/********************* Delay slot protection *******************************/
// We have to do the protection here since we've separated this ifetch and its
// stalls from the processor pipeline, we just emit nops.  SPREE automatically
// protects delay slots but since we can't tell it when the delay slot is
// stalled, we have to do it here.

branch_detector branch_detector (
  .func(func),
  .opcode(opcode),
  .is_branch(is_branch));

pipereg_1 pipereg (
  .clk(clk),
  .resetn(resetn),
  .d(is_branch),
  .squashn(1'b1),
  .en(pc_advance),
  .q(is_delayslot));
  //defparam
  //  pipereg.WIDTH=1;
/***************************************************************************/

wire prediction_tmp;
wire predict_result_rdy_tmp;

assign predict_result_rdy_tmp=predict_result_rdy&predict_en_saved;

branchpredict_32_4096_12_1 bpredictor ( 
    .clk(clk),
    .resetn(resetn),
    .predict(en),
    .prediction(prediction_tmp),
    .pc_predict({next_pc[32-1:3],3'b0}),
    .result_rdy(predict_result_rdy_tmp),
    .result(ctrl_load),
    .pc_result({pc_rollbacknottaken[32-1:3],3'b0}) );

assign prediction=(pcwrop!=1) ? prediction_tmp :1;

endmodule

/****************************************************************************
          Generic Register
****************************************************************************/
module register_32(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [32-1:0] d;
output [32-1:0] q;
reg [32-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule

/****************************************************************************
          Generic Register
****************************************************************************/
module register_2(d,clk,resetn,en,q);

input clk;
input resetn;
input en;
input [2-1:0] d;
output [2-1:0] q;
reg [2-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1)
		q<=d;
end

endmodule

/****************************************************************************
          Branch detector
****************************************************************************/
module branch_detector(opcode, func, is_branch);
input [5:0] opcode;
input [5:0] func;
output is_branch;

wire is_special;
wire [5:0] func_local;

assign func_local = func & 6'b111000;

assign is_special=!(|opcode);
assign is_branch=((!(|opcode[5:3])) && !is_special) || 
                  ((is_special)&&(func_local==6'b001000));

endmodule

/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg_1(d,clk,resetn,en,squashn,q);

input clk;
input resetn;
input en;
input squashn;
input [1-1:0] d;
output [1-1:0] q;
reg [1-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule

module branchpredict_32_4096_12_1 ( clk, resetn,
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
input [32-1:0] pc_predict; // The PC value for which to predict 
output prediction;              // The actual prediction 1-taken, 0-nottaken

// Prediction Result Port - tells us if the prediction made at pc_result was taken
input result_rdy;               // The branch has been resolved when result_rdy goes hi
input [32-1:0] pc_result;  // The PC value that this result is for
input result;                   // The actual result 1-taken, 0-nottaken

wire resetn_nc;
wire predict_nc;
wire [32-1:0] pc_predict_local;
wire [32-1:0] pc_result_local;

assign resetn_nc = resetn;
assign predict_nc = predict;
assign pc_predict_local = pc_predict;
assign pc_result_local = pc_result;

wire [12-1:0] address_b;

assign address_b=pc_predict_local[12+2-1:2];

`ifndef USE_INHOUSE_LOGIC
    `define USE_INHOUSE_LOGIC
`endif

`ifdef USE_INHOUSE_LOGIC
    dpram_12_4096_1 pred_table(
	.clk(clk),
	.address_a(pc_result_local[12+2-1:2]),
	.address_b(address_b),
	.wren_a(result_rdy),
	.wren_b(0),
	.data_a(result),
	.data_b(0),
	.out_a(),
	.out_b(prediction)
    );

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

module dpram_12_4096_1 (
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

parameter AWIDTH = 12;
parameter NUM_WORDS = 4096;
parameter DWIDTH = 1;

input clk;
input [(AWIDTH-1):0] address_a;
input [(AWIDTH-1):0] address_b;
input  wren_a;
input  wren_b;
input [(DWIDTH-1):0] data_a;
input [(DWIDTH-1):0] data_b;
output reg [(DWIDTH-1):0] out_a;
output reg [(DWIDTH-1):0] out_b;

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