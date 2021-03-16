from components import register
from components import pipereg
from components import branch_detector
from bpred_1bittable import branchpredict
from rams import dpram
import parser

class ifetch():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self, exception_address, i_datawidth, i_addresswidth, i_size):
        string = '''
/****************************************************************************
            Fetch Unit with branch prediction

  IMPORTANT: tgt_pc should arrive 1 cycle after instruction to account for delay slot.
  Also, we compress the prediction PC's by one bit since delay slots prevent consecutive branches.
            
op
  0  Conditional PC write
  1  UnConditional PC write

****************************************************************************/

module ifetch_{EXCEPTION_ADDRESS}_{I_DATAWIDTH}_{I_ADDRESSWIDTH}_{I_SIZE}(clk,resetn,
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
input [{I_DATAWIDTH}-1:0] load_data;

input pcwrop;
input [{I_DATAWIDTH}-1:0] predict_tgt_pc;
input predict_en;
input predict_result_rdy;
output predict_result;

input  interrupt; 
output [{I_DATAWIDTH}-1:0] epc; 
output [31:0] ecause; 

output [{I_DATAWIDTH}-1:0] pc_out;   // output pc + 1 shifted left 2 bits
output [{I_DATAWIDTH}-1:0] next_pc;
output [31:26] opcode;
output [25:21] rs;
output [20:16] rt;
output [15:11] rd;
output [10:6] sa;
output [15:0] offset;
output [25:0] instr_index;
output [5:0] func;
output [{I_DATAWIDTH}-1:0] instr;


wire [{I_DATAWIDTH}-1:0] pc_plus_1;
reg [{I_DATAWIDTH}-1:0] pc;
wire ctrl_load;

// prediction stuff
wire prediction;
wire prediction_saved;
wire predict_en_saved;
wire [{I_DATAWIDTH}-1:0] pc_rollbacknottaken;
wire [{I_DATAWIDTH}-1:0] pc_rollback;

//not connect ports
wire [31:0] boot_iaddr_nc;
assign boot_iaddr_nc = boot_iaddr;
wire [31:0] boot_idata_nc;
assign boot_idata_nc = boot_idata;
wire  boot_iwe_nc;
assign boot_iwe_nc = boot_iwe;

reg [{I_DATAWIDTH}-1:0] _next_pc;
reg pc_load_en;
reg predict_result;

// tolerating waits stuff
reg [{I_DATAWIDTH}-1:0] pc_save_data;
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
    .address_a (next_pc[{I_DATAWIDTH}-1:2]),
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
        imem.width_a = {I_DATAWIDTH}, 
        imem.widthad_a = {I_ADDRESSWIDTH},
        imem.numwords_a = {I_SIZE},
        imem.operation_mode = "BIDIR_DUAL_PORT",    // changed
        imem.width_b = {I_DATAWIDTH},                 // new
        imem.widthad_b = {I_ADDRESSWIDTH},            // new
        imem.numwords_b = {I_SIZE},                   // new
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

assign {{dummy,pc_plus_1}} = pc + 4;
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
wire [{I_DATAWIDTH}-1:0] pcrollback_d;
wire pcrollback_en;

assign pcrollback_d = (prediction) ? pc_plus_1 : predict_tgt_pc;
assign pcrollback_en = en&predict_en;

register_{I_DATAWIDTH} pcrollback( pcrollback_d, 
    clk, resetn, pcrollback_en, pc_rollback);
  //defparam pcrollback.WIDTH={I_DATAWIDTH};

wire pcrollbacknt_en;
assign pcrollbacknt_en = en&predict_en;

register_{I_DATAWIDTH} pcrollbacknt( pc, clk, resetn, pcrollbacknt_en, pc_rollbacknottaken);
 // defparam pcrollbacknt.WIDTH={I_DATAWIDTH};

//register_{I_DATAWIDTH} pcrollbacktk(predict_tgt_pc, clk, resetn,predict_en, 
    //pc_rollback);
  //defparam pcrollbacktk.WIDTH={I_DATAWIDTH};

wire [2-1: 0] buf_predict_d;
wire buf_predict_en;

assign buf_predict_d = {{prediction,predict_en&(pcwrop!=1'b1)}};
assign buf_predict_en = en&predict_en;

register_2 buf_predict(buf_predict_d,clk,resetn,buf_predict_en, 
    {{prediction_saved,predict_en_saved}});
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
always@(*)
  begin
    if (interrupt)
    begin   // If interrupt occurs, jump to {EXCEPTION_ADDRESS}
      _next_pc={EXCEPTION_ADDRESS};
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
    .pc_predict({{next_pc[{I_DATAWIDTH}-1:3],3'b0}}),
    .result_rdy(predict_result_rdy_tmp),
    .result(ctrl_load),
    .pc_result({{pc_rollbacknottaken[{I_DATAWIDTH}-1:3],3'b0}}) );

assign prediction=(pcwrop!=1) ? prediction_tmp :1;

endmodule
'''
        
        return string.format(EXCEPTION_ADDRESS=exception_address, I_DATAWIDTH=i_datawidth, I_ADDRESSWIDTH=i_addresswidth, I_SIZE=i_size)

    def write (self, exception_address, i_datawidth, i_addresswidth, i_size):
        self.fp.write(self.make_str(exception_address, i_datawidth, i_addresswidth, i_size))

if __name__ == '__main__':
    fp = open(parser.parse(), "w")
    uut = ifetch(fp)
    uut.write(67108896, 32, 14, 16384)
    fp.close()
    fp = open(parser.parse(), "a")
    fp.write("\r\r")
    reg1 = register(fp)
    reg1.write(32)
    fp.write("\r\r")
    reg2 = register(fp)
    reg2.write(2) 
    fp.write("\r\r")
    br_detect = branch_detector(fp)
    br_detect.write() 
    fp.write("\r\r")
    pipereg = pipereg(fp)
    pipereg.write(1) 
    fp.write("\r\r")
    br_pred = branchpredict(fp)
    br_pred.write(32, 4096, 12, 1)
    fp.write("\r\r")
    ram = dpram(fp)
    ram.write(12, 4096, 1) 
    fp.close()