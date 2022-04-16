module dpram1 (
    clk,
    address_a,
    address_b,
    wren_a, 
    wren_b, 
    data_a,
    data_b,
    byteen_a,
    byteen_b,
    out_a,
    out_b
);

parameter AWIDTH=10;
parameter NUM_WORDS=1024;
parameter DWIDTH=32;
parameter LOG2DWIDTH = $clog2(DWIDTH);

input clk;
input [(AWIDTH-1):0] address_a;
input [(AWIDTH-1):0] address_b;
input  wren_a;
input  wren_b;
input [(DWIDTH/8)-1:0] byteen_a;
input [(DWIDTH/8)-1:0] byteen_b;
input [(DWIDTH-1):0] data_a;
input [(DWIDTH-1):0] data_b;
output reg [(DWIDTH-1):0] out_a;
output reg [(DWIDTH-1):0] out_b;

//`ifdef SIMULATION_MEMORY
//
//integer i;
//integer k;
//
////reg [32-1:0] ram[268435456-1:0];
//reg [32-1:0] ram[67108864-1:0];
////reg [32-1:0] ram[4096-1:0];
//reg [25:0] addr_a;
//reg [25:0] addr_b;
// 
//initial
// begin
//   //This is TOO big for 256 MB RAM!  We right shift data by 1
//      $readmemh("instr.dat",ram,'h100_0000);
////      $readmemh("instr.dat",ram,'h400_0000);
//       $readmemh("data.dat",ram,'h400_0000 >> 1);
////      $readmemh("instr.dat",ram,'h1000_0000);
// end
//
//always@(*) begin
//    addr_a = address_a << 26-AWIDTH;
//    addr_b = address_b << 26-AWIDTH;
//   // addr_a = address_a;
//   // addr_b = address_b;
//end
//
//always@(posedge clk) begin 
//  if (wren_a) begin
//      for(k=0; k < DWIDTH/32;k=k+1)begin
//          for(i=0; i < 4 ;i=i+1)begin
//              if(byteen_a[((DWIDTH/8-1)-(4*k+i))])
//                  ram[addr_a+k][i*8+:8] <= data_a[32*k+i*8+:8];
//          end
//      end
//  end
//  else begin
//      for(i=0; i < DWIDTH/32; i=i+1)begin
//          out_a[32*i+:32] <= ram[addr_a+i];
//      end
//  end
//  if (wren_b) begin
//      for(k=0; k < DWIDTH/32;k=k+1)begin
//          for(i=0; i < 4 ;i=i+1)begin
//              if(byteen_b[((DWIDTH/8-1)-(4*k+i))])
//                  ram[addr_b+k][i*8+:8] <= data_b[32*k+i*8+:8];
//          end
//      end
//  end
//  else begin
//      for(i=0; i < DWIDTH/32; i=i+1)begin
//          out_b[32*i+:32] <= ram[addr_b+i];
//      end
//  end
//end
//`else

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


//`endif

endmodule
//`include "bpred_1bittable.v"
`define USE_INHOUSE_LOGIC1
`define USE_INHOUSE_LOGIC

/*
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
*/

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

//`include "cop0.v"

/*******
 * SPREE limitation - by not specifying stall signal name and assuming
 * "stalled" requires you to have only one opcode port which stalls
 *
 * We get around this since both INPUT&OUTPUT are in the same stage so we 
 * can use the same stall signal.
 *******/

module cop0(
    clk,
    resetn,
    stalled,

    instr,

    exception,

    read_addr,
    dest_addr,
    fromcpu,
    fromcpu_en,
    tocpu,
    tocpu_en,

    epc_in,
    ext_cause_in,
    int_cause_in_stage1,  //very weak - implement OR in SPREE instead
    int_cause_in_stage2,
    status,

    badvaddr_in,
    badvaddr_we
    );

parameter NUMSTAGESTIMES32=64;
parameter NUMSTAGES=NUMSTAGESTIMES32/32;

input clk;
input resetn;
output stalled;

input   [31:0] instr;

output exception;

input   [4:0]  read_addr;
input   [4:0]  dest_addr;
input   [31:0] fromcpu;
input          fromcpu_en;
output  [31:0] tocpu;
input          tocpu_en;

input  [31:0] epc_in;

input  [31:0] ext_cause_in;
input  [31:0] int_cause_in_stage1;
input  [31:0] int_cause_in_stage2;

output [31:0] status;
input  [31:0] badvaddr_in;
input         badvaddr_we;

wire [31:0] cause_in;

reg [31:0] epc_out;
reg [31:0] cause_out;
reg [31:0] status;
reg [31:0] badvaddr_out;

reg  [31:0] tocpu;

  assign cause_in=ext_cause_in | int_cause_in_stage1 | 
                                 int_cause_in_stage2;

  always@(posedge clk)
    if (!resetn)
      epc_out<=0;
    else if (fromcpu_en && dest_addr==14)
      epc_out<=fromcpu;
    else if (exception)
      epc_out<=epc_in;

  always@(posedge clk)
    if (!resetn)
      cause_out<=0;
    else if (fromcpu_en && dest_addr==13)
      cause_out<=fromcpu;
    else
      cause_out<=cause_in;

  always@(posedge clk)
    if (!resetn)
      status<=0;
    else if (fromcpu_en && dest_addr==12)
      status<=fromcpu;
    else if (exception)
      status[5:0]<={status[3:0],2'b0};

  always@(posedge clk)
    if (!resetn)
      badvaddr_out<=0;
    else if (fromcpu_en && dest_addr==8)
      badvaddr_out<=fromcpu;
    else if (badvaddr_we)
      badvaddr_out<=badvaddr_in;

  always@(posedge clk)
      tocpu <= (read_addr==14) ? epc_out : 
               (read_addr==13) ? cause_out : 
               (read_addr==8)  ? badvaddr_out : status;

  // 1 cycle stall
  multicyclestall mc(tocpu_en,0,clk,resetn,stalled);
  //assign stalled= 0;

  assign exception = ((|(cause_in[15:8] & status[15:8])) && status[0]);

endmodule

module multicyclestall(request, devwait,clk,resetn,stalled);
input request;
input devwait;
input clk;
input resetn;
output stalled;

  reg T;

  always@(posedge clk)
    if (~resetn)
      T<=0;
    else
      T<=stalled;

  assign stalled=(T) ? devwait : request;
endmodule

//`include "cop2.v"

module cop2(
    clk,
    resetn,
    stalled,

    fromcpu,
    fromcpu_en,
    tocpu,
    tocpu_en,

    //Global I/O
    tocop2,
    tocop2_en,
    tocop2_wait,
    fromcop2,
    fromcop2_en,
    fromcop2_wait
    );

input clk;
input resetn;
output stalled;

input   [31:0] fromcpu;
input          fromcpu_en;
output  [31:0] tocpu;
input          tocpu_en;

output  [31:0] tocop2;
output         tocop2_en;
input          tocop2_wait;
input   [31:0] fromcop2;
input          fromcop2_en;
output         fromcop2_wait;


  assign tocop2=fromcpu;
  assign tocop2_en=fromcpu_en;

  assign tocpu=fromcop2;
  assign fromcop2_wait=fromcop2_en&~tocpu_en;   //assign 1 if pipe is stalled 

  assign stalled= (fromcpu_en & tocop2_wait) || (tocpu_en & ~fromcop2_en);


endmodule

//`include "lo_reg.v"
/****************************************************************************
          Generic Register
****************************************************************************/
module lo_reg(d,clk,resetn,squashn,en,q);
parameter WIDTH=32;

input clk;
input resetn;
input squashn;
input en;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
reg [WIDTH-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1 && squashn)
		q<=d;
end

endmodule


//`include "hi_reg.v"
/****************************************************************************
          Generic Register
****************************************************************************/
module hi_reg(d,clk,resetn,squashn,en,q);
parameter WIDTH=32;

input clk;
input resetn;
input squashn;
input en;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
reg [WIDTH-1:0] q;

always @(posedge clk or negedge resetn)		//asynchronous reset
begin
	if (resetn==0)
		q<=0;
	else if (en==1 && squashn)
		q<=d;
end

endmodule


//`include "data_mem_bus_int.v"

/*
module zeroer(d,en,q);
parameter WIDTH=32;

input en;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
assign q= (en) ? d : 0;

endmodule
*/
/*
module const (out);

parameter WIDTH=32;
parameter VAL=31;

output [WIDTH-1:0] out;

assign out=VAL;

endmodule
*/
/*
module branch_detector(opcode, func, is_branch);
input [5:0] opcode;
input [5:0] func;
output is_branch;

wire is_special;

assign is_special=!(|opcode);
assign is_branch=((!(|opcode[5:3])) && !is_special) || 
                  ((is_special)&&(func[5:3]==3'b001));

endmodule
*/
/*
module nop(d,q);
parameter WIDTH=32;

input [WIDTH-1:0] d;
output [WIDTH-1:0] q;

  assign q=d;

endmodule
*/
module register(d,clk,resetn,en,q);
parameter WIDTH=32;

input clk;
input resetn;
input en;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
reg [WIDTH-1:0] q;

always @(posedge clk or negedge resetn)     //asynchronous reset
begin
    if (resetn==0)
        q<=0;
    else if (en==1)
        q<=d;
end

endmodule

/*
module fakedelay(d,clk,q);
parameter WIDTH=32;
input [WIDTH-1:0] d;
input clk;
output [WIDTH-1:0] q;

assign q=d;

endmodule
*/
/*
module pipereg(d,clk,resetn,en,squashn,q);
parameter WIDTH=32;

input clk;
input resetn;
input en;
input squashn;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
reg [WIDTH-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule
*/
module data_mem( clk, resetn, en, stalled,
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

parameter D_ADDRESSWIDTH=32;

parameter DM_DATAWIDTH=32;
parameter DM_BYTEENAWIDTH=4;             // usually should be DM_DATAWIDTH/8
parameter DM_ADDRESSWIDTH=16;
parameter DM_SIZE=16384;

input clk;
input resetn;
input en;
output stalled;

output [31:0] ecause; 

input [31:0] boot_daddr;
input [31:0] boot_ddata;
input boot_dwe;

input [D_ADDRESSWIDTH-1:0] d_address;
input [4-1:0] op;
input [DM_DATAWIDTH-1:0] d_writedata;
output [DM_DATAWIDTH-1:0] d_loadresult;

output [32-1:0] bus_address;
output [4-1:0] bus_byteen;
output         bus_we;
output         bus_en;
output [32-1:0] bus_writedata;
input  [32-1:0] bus_readdata;
input           bus_wait;
input  [32-1:0] bus_ecause;

wire [DM_BYTEENAWIDTH-1:0] d_byteena;
wire [DM_DATAWIDTH-1:0] d_readdatain;
wire [DM_DATAWIDTH-1:0] d_writedatamem;
wire d_write;
wire [1:0] d_address_latched;

assign d_write=op[3];

assign ecause=bus_ecause;

register d_address_reg(d_address[1:0],clk,1'b1,en,d_address_latched);
    defparam d_address_reg.WIDTH=2;
                
store_data_translator sdtrans_inst(
    .write_data(d_writedata),
    .d_address(d_address[1:0]),
    .store_size(op[1:0]),
    .d_byteena(d_byteena),
    .d_writedataout(d_writedatamem));

load_data_translator ldtrans_inst(
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



/****************************************************************************
          Store data translator
          - moves store data to appropriate byte/halfword 
          - interfaces with altera blockrams
****************************************************************************/
module store_data_translator(
    write_data,             // data in least significant position
    d_address,
    store_size,
    d_byteena,
    d_writedataout);        // shifted data to coincide with address
parameter WIDTH=32;

input [WIDTH-1:0] write_data;
input [1:0] d_address;
input [1:0] store_size;
output [3:0] d_byteena;
output [WIDTH-1:0] d_writedataout;

reg [3:0] d_byteena;
reg [WIDTH-1:0] d_writedataout;

always @(write_data or d_address or store_size)
begin
    case (store_size)
        2'b11:
            case(d_address[1:0])
                0: 
                begin 
                    d_byteena=4'b1000; 
                    d_writedataout={write_data[7:0],24'b0}; 
                end
                1: 
                begin 
                    d_byteena=4'b0100; 
                    d_writedataout={8'b0,write_data[7:0],16'b0}; 
                end
                2: 
                begin 
                    d_byteena=4'b0010; 
                    d_writedataout={16'b0,write_data[7:0],8'b0}; 
                end
                default: 
                begin 
                    d_byteena=4'b0001; 
                    d_writedataout={24'b0,write_data[7:0]}; 
                end
            endcase
        2'b01:
            case(d_address[1])
                0: 
                begin 
                    d_byteena=4'b1100; 
                    d_writedataout={write_data[15:0],16'b0}; 
                end
                default: 
                begin 
                    d_byteena=4'b0011; 
                    d_writedataout={16'b0,write_data[15:0]}; 
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

/****************************************************************************
          Load data translator
          - moves read data to appropriate byte/halfword and zero/sign extends
****************************************************************************/
module load_data_translator(
    d_readdatain,
    d_address,
    load_size,
    load_sign_ext,
    d_loadresult);
parameter WIDTH=32;

input [WIDTH-1:0] d_readdatain;
input [1:0] d_address;
input [1:0] load_size;
input load_sign_ext;
output [WIDTH-1:0] d_loadresult;

reg [WIDTH-1:0] d_loadresult;

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
            d_loadresult[31:8]={24{load_sign_ext&d_loadresult[7]}};
        end
        2'b01:
        begin
            case (d_address[1])
                0: d_loadresult[15:0]=d_readdatain[31:16];
                default: d_loadresult[15:0]=d_readdatain[15:0];
            endcase
            d_loadresult[31:16]={16{load_sign_ext&d_loadresult[15]}};
        end
        default:
            d_loadresult=d_readdatain;
    endcase
end

endmodule


//`include "divider.v"

module div(en,resetn,stalled,quotient,remainder,dividend,divider,sign,clk);

   input         clk;
   input         resetn;
   input         sign;
   input         en;
   input [31:0]  dividend, divider;
   output [31:0] quotient, remainder;
   output        stalled;

   reg [31:0]    quotient, quotient_temp;
   reg [63:0]    dividend_copy, divider_copy, diff;
   reg           negative_output;
   
   wire [31:0]   remainder = (!negative_output) ? 
                             dividend_copy[31:0] : 
                             ~dividend_copy[31:0] + 1'b1;

   reg [5:0]     bits; 

   parameter IDLE=2'b00,
             DIVIDING=2'b01,
             DONE=2'b10;

   reg [1:0] state;

   always@(posedge clk)
     if (!resetn)
       state<=0;
     else
       case(state)
        IDLE: state<=(en) ? DIVIDING : IDLE;
        DIVIDING: state<=(bits==5'd1) ? DONE : DIVIDING;
        DONE: state<= IDLE;
        default: state<=0;
       endcase

   assign stalled = (state==DIVIDING) || (state==IDLE && en);
   //assign stalled = (bits==0 && en) || (|bits);

   always @( posedge clk ) 
     if (!resetn)
     begin

        bits = 0;
        quotient = 0;
        quotient_temp = 0;
        dividend_copy = 0;
        divider_copy = 0;
        negative_output =0;
        diff=0;

     end
     else if( en && state==IDLE) begin

        bits = 6'd32;
        quotient = 0;
        quotient_temp = 0;
        dividend_copy = (!sign || !dividend[31]) ? 
                        {32'd0,dividend} : 
                        {32'd0,~dividend + 1'b1};
        divider_copy = (!sign || !divider[31]) ? 
                       {1'b0,divider,31'd0} : 
                       {1'b0,~divider + 1'b1,31'd0};

        negative_output = sign &&
                          ((divider[31] && !dividend[31]) 
                        ||(!divider[31] && dividend[31]));
        
     end 
     else if ( bits > 0 ) begin

        diff = dividend_copy - divider_copy;

        quotient_temp = quotient_temp << 1;

        if( !diff[63] ) begin

           dividend_copy = diff;
           quotient_temp[0] = 1'd1;

        end

        quotient = (!negative_output) ? 
                   quotient_temp : 
                   ~quotient_temp + 1'b1;

        divider_copy = divider_copy >> 1;
        bits = bits - 1'b1;

     end
endmodule

//`include "mul_shift_stall.v"
module mul(clk, resetn, start, stalled, dst,
            opA, opB, sa,
            op,
            shift_result,
            hi, lo);
parameter WIDTH=32;

input clk;
input resetn;

input start;
output stalled;

input [4:0] dst;

input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
input [5-1:0] sa;
input [2:0] op;

output [WIDTH-1:0] shift_result;
output [WIDTH-1:0] hi;
output [WIDTH-1:0] lo;

/********* Control Signals *********/
wire is_signed,dir, is_mul;
assign is_mul=op[2];      // selects between opB and the computed shift amount
assign is_signed=op[1];
assign dir=op[0];         // selects between 2^sa and 2^(32-sa) for right shift

/********* Circuit Body *********/
wire dum,dum2,dum3;
wire [WIDTH:0] opB_mux_out;
wire [5-1:0] left_sa;     // Amount of left shift required for both left/right
reg [WIDTH:0] decoded_sa;

assign opB_mux_out= (is_mul) ? {is_signed&opB[WIDTH-1],opB} : decoded_sa;

`ifdef USE_INHOUSE_LOGIC
local_mult local_mult_component (
.dataa({is_signed&opA[WIDTH-1],opA}),
.datab(opB_mux_out),
.clock(clk),
.clken(1'b1),
.aclr(~resetn),
.result({dum2,dum,hi,lo})
);
defparam
 local_mult_component.LPM_WIDTHA = WIDTH + 1,
 local_mult_component.LPM_WIDTHB = WIDTH + 1,
 local_mult_component.LPM_WIDTHP = 2*WIDTH + 2,
 local_mult_component.LPM_REPRESENTATION = "SIGNED";

`else
 
lpm_mult  lpm_mult_component (
  .dataa ({is_signed&opA[WIDTH-1],opA}),
  .datab (opB_mux_out),
  .sum(),
  .clock(clk),
  .clken(),
  .aclr(~resetn),
  .result ({dum2,dum,hi,lo}));
defparam
  lpm_mult_component.lpm_widtha = WIDTH+1,
  lpm_mult_component.lpm_widthb = WIDTH+1,
  lpm_mult_component.lpm_widthp = 2*WIDTH+2,
  lpm_mult_component.lpm_widths = 1,
  lpm_mult_component.lpm_pipeline = 1,
  lpm_mult_component.lpm_type = "LPM_MULT",
  lpm_mult_component.lpm_representation = "SIGNED",
  lpm_mult_component.lpm_hint = "MAXIMIZE_SPEED=6";
`endif

assign shift_result= (dir && |sa) ? hi : lo;


assign {dum3, left_sa}= (dir) ? 32-sa : {1'b0,sa};

always@(left_sa or dir)
begin
  decoded_sa=0;
  case(left_sa)
    0: decoded_sa[0]=1;
    1: decoded_sa[1]=1;
    2: decoded_sa[2]=1;
    3: decoded_sa[3]=1;
    4: decoded_sa[4]=1;
    5: decoded_sa[5]=1;
    6: decoded_sa[6]=1;
    7: decoded_sa[7]=1;
    8: decoded_sa[8]=1;
    9: decoded_sa[9]=1;
    10: decoded_sa[10]=1;
    11: decoded_sa[11]=1;
    12: decoded_sa[12]=1;
    13: decoded_sa[13]=1;
    14: decoded_sa[14]=1;
    15: decoded_sa[15]=1;
    16: decoded_sa[16]=1;
    17: decoded_sa[17]=1;
    18: decoded_sa[18]=1;
    19: decoded_sa[19]=1;
    20: decoded_sa[20]=1;
    21: decoded_sa[21]=1;
    22: decoded_sa[22]=1;
    23: decoded_sa[23]=1;
    24: decoded_sa[24]=1;
    25: decoded_sa[25]=1;
    26: decoded_sa[26]=1;
    27: decoded_sa[27]=1;
    28: decoded_sa[28]=1;
    29: decoded_sa[29]=1;
    30: decoded_sa[30]=1;
    31: decoded_sa[31]=1;
  endcase
end

// 1 cycle stall state machine
onecyclestall staller((start&is_mul)|(start&(|dst)&~is_mul),clk,resetn,stalled);
endmodule

/*
module local_mult(
dataa,
datab,
clock,
clken,
aclr,
result
);

parameter LPM_WIDTHA = 32;
parameter LPM_WIDTHB = 32;
parameter LPM_WIDTHP = 64;
parameter LPM_REPRESENTATION = "SIGNED";

input [LPM_WIDTHA-1:0] dataa;
input [LPM_WIDTHB-1:0] datab;
input clock;
input clken;
input aclr;
output reg [LPM_WIDTHP-1:0] result;

wire signed [LPM_WIDTHA-1:0] signedinputA;
wire signed [LPM_WIDTHB-1:0] signedinputB;
wire signed [LPM_WIDTHP-1:0] signedoutputP;

wire [LPM_WIDTHA-1:0] unsignedinputA;
wire [LPM_WIDTHB-1:0] unsignedinputB;
wire [LPM_WIDTHP-1:0] unsignedinputP;

wire gated_clock;

assign signedinputA = dataa;
assign signedinputB = datab;
assign unsignedinputA = dataa;
assign unsignedinputB = datab;

assign signedoutputP = signedinputA * signedinputB;
assign unsignedoutputP = unsignedinputA * unsignedinputB;

assign gated_clock = clock & clken;

always @(posedge gated_clock)begin
    if(aclr)begin
       result <= 0;
    end
    else if(LPM_REPRESENTATION == "SIGNED")
       result <= signedoutputP;
    else
       result <= unsignedoutputP; 
end

endmodule
*/

/*
module onecyclestall(request,clk,resetn,stalled);
input request;
input clk;
input resetn;
output stalled;

  reg T,Tnext;

  // State machine for Stalling 1 cycle
  always@(request or T)
  begin
    case(T) 
      1'b0: Tnext=request;
      1'b1: Tnext=0;
    endcase 
  end       
  always@(posedge clk)
    if (~resetn)
      T<=0; 
    else    
      T<=Tnext;
  assign stalled=(request&~T);
endmodule
*/
//`include "logic_unit.v"
module logic_unit (
            opA, opB,
            op,
            result);
parameter WIDTH=32;


input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
input [2-1:0] op;
output [WIDTH-1:0] result;

reg [WIDTH-1:0] logic_result;

always@(opA or opB or op )
    case(op)
        2'b00:
            logic_result=opA&opB;
        2'b01:
            logic_result=opA|opB;
        2'b10:
            logic_result=opA^opB;
        2'b11:
            logic_result=~(opA|opB);
    endcase

assign result=logic_result;


endmodule

//`include "addersub_slt.v"


/*
module local_add_sub(
dataa,
datab,
cin,
add_sub,
result
);

parameter WIDTH = 32;
parameter PIPELINE = 0;
parameter REPRESENTATION = "SIGNED";

input[WIDTH-1:0] dataa;
input[WIDTH-1:0] datab;
input cin;
input add_sub;
output reg [WIDTH-1:0] result;

always @(*)begin
    if(add_sub == 1'b1)
         result = dataa+datab+cin;
    else
         result = dataa - datab;
end

endmodule
*/

module addersub (
            opA, opB,
            op, 
            result,
            result_slt );

parameter WIDTH=32;


input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
//input carry_in;
input [3-1:0] op;

output [WIDTH-1:0] result;
output result_slt;

wire carry_out;
wire [WIDTH:0] sum;

// Mux between sum, and slt
wire is_slt;
wire signext;
wire addsub;

assign is_slt=op[2];
assign signext=op[1];
assign addsub=op[0];

assign result=sum[WIDTH-1:0];
//assign result_slt[WIDTH-1:1]={31{1'b0}};
//assign result_slt[0]=sum[WIDTH];
assign result_slt=sum[WIDTH];

`ifdef USE_INHOUSE_LOGIC
  local_add_sub local_adder_inst(
      .dataa({signext&opA[WIDTH-1],opA}),
      .datab({signext&opB[WIDTH-1],opB}),
      .cin(~addsub),
      .add_sub(addsub),
      .result(sum)
  );
  defparam
      local_adder_inst.WIDTH = WIDTH+1,
      local_adder_inst.PIPELINE = 0,
      local_adder_inst.REPRESENTATION = "SIGNED";
`else
lpm_add_sub adder_inst(
    .dataa({signext&opA[WIDTH-1],opA}),
    .datab({signext&opB[WIDTH-1],opB}),
    .cin(~addsub),
    .add_sub(addsub),
    .result(sum)
        // synopsys translate_off
        ,
        .cout (),
        .clken (),
        .clock (),
        .overflow (),
        .aclr ()
        // synopsys translate_on
    );
defparam 
    adder_inst.lpm_width=WIDTH+1,
    adder_inst.lpm_representation="SIGNED";
`endif

assign carry_out=sum[WIDTH];


endmodule

//`include "merge26lo.v"

module merge26lo(in1, in2, out);
input [31:0] in1;
input [25:0] in2;
output [31:0] out;

assign out[31:0]={in1[31:28],in2[25:0],2'b0};
endmodule

//`include "branchresolve.v"

module branchresolve ( en, rs, rt, eq, ne, ltz, lez, gtz, gez, eqz);
parameter WIDTH=32;
input en;
input [WIDTH-1:0] rs;
input [WIDTH-1:0] rt;
output eq;
output ne;
output ltz;
output lez;
output gtz;
output gez;
output eqz;

assign eq=(en)&(rs==rt);
assign ne=(en)&~eq;
assign eqz=(en)&~(|rs);
assign ltz=(en)&rs[WIDTH-1];
assign lez=(en)&rs[WIDTH-1] | eqz;
assign gtz=(en)&(~rs[WIDTH-1]) & ~eqz;
assign gez=(en)&(~rs[WIDTH-1]);

endmodule

//`include "pcadder.v"

module pcadder(pc, offset, result);
parameter PC_WIDTH=32;

input [PC_WIDTH-1:0] pc;
input [PC_WIDTH-1:0] offset;
output [PC_WIDTH-1:0] result;

wire dum;

assign {dum,result} = pc + {offset[PC_WIDTH-3:0],2'b0};

endmodule

//`include "signext16.v"
module signext16 ( in, out);

input [15:0] in;
output [31:0] out;

assign out={{16{in[15]}},in[15:0]};

endmodule
//`include "reg_file_pipe.v"


module reg_file(clk,resetn, c_squashn,
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_we);

parameter WIDTH=32;
parameter NUMREGS=32;
parameter LOG2NUMREGS=5;

input clk;
input resetn;

input a_en;
input b_en;

input [LOG2NUMREGS-1:0] a_reg,b_reg,c_reg;
output [WIDTH-1:0] a_readdataout, b_readdataout;
input [WIDTH-1:0] c_writedatain;
input c_we;
input c_squashn;
integer i;


`ifdef USE_INHOUSE_LOGIC1
        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(a_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we & (|c_reg) & c_squashn),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout)
        );
        defparam
            reg_file1.AWIDTH=LOG2NUMREGS,
            reg_file1.NUM_WORDS=NUMREGS,
            reg_file1.DWIDTH=WIDTH;
 
 initial begin
    for(i=0;i<NUMREGS;i=i+1)
        reg_file1.dpram1.ram[i]=0;
 end 
         
        ram_wrapper reg_file2(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(b_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(b_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we & (|c_reg)),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(b_readdataout)
        );
        defparam
            reg_file2.AWIDTH=LOG2NUMREGS,
            reg_file2.NUM_WORDS=NUMREGS,
            reg_file2.DWIDTH=WIDTH;

 initial begin
    for(i=0;i<NUMREGS;i=i+1)
        reg_file2.dpram1.ram[i]=0;
 end 
`else

	altsyncram	reg_file1(
				.wren_a (c_we & (|c_reg) & c_squashn),
				.clock0 (clk),
        .clock1 (clk),
        .clocken1 (a_en),
				.address_a (c_reg[LOG2NUMREGS-1:0]),
				.address_b (a_reg[LOG2NUMREGS-1:0]),
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
				.address_a (c_reg[LOG2NUMREGS-1:0]),
				.address_b (b_reg[LOG2NUMREGS-1:0]),
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
		reg_file2.width_a = WIDTH,
		reg_file2.widthad_a = LOG2NUMREGS,
		reg_file2.numwords_a = NUMREGS,
		reg_file2.width_b = WIDTH,
		reg_file2.widthad_b = LOG2NUMREGS,
		reg_file2.numwords_b = NUMREGS,
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

endmodule

/*
module ram_wrapper (
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

parameter AWIDTH=10;
parameter NUM_WORDS=1024;
parameter DWIDTH=32;

input clk;
input resetn;
input [(AWIDTH-1):0] address_a;
input [(AWIDTH-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(DWIDTH-1):0] data_a;
input [(DWIDTH-1):0] data_b;
output [(DWIDTH-1):0] out_a;
output [(DWIDTH-1):0] out_b;

reg [(AWIDTH-1):0] q_address_a;
reg [(AWIDTH-1):0] q_address_b;
reg [(AWIDTH-1):0] mux_address_b;

dpram dpram1(
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
defparam
    dpram1.AWIDTH=AWIDTH,
    dpram1.NUM_WORDS=NUM_WORDS,
    dpram1.DWIDTH=DWIDTH;

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

endmodule
*/

//`include "ifetch_pipe_bpred_bus_int.v"
module ifetch(clk,resetn,
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

parameter EXCEPTION_ADDRESS=32'h400_0020;
parameter I_DATAWIDTH=32;
parameter I_ADDRESSWIDTH=14;
parameter I_SIZE=16384;

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
input [I_DATAWIDTH-1:0] load_data;

input pcwrop;
input [I_DATAWIDTH-1:0] predict_tgt_pc;
input predict_en;
input predict_result_rdy;
output predict_result;

input  interrupt; 
output [I_DATAWIDTH-1:0] epc; 
output [31:0] ecause; 

output [I_DATAWIDTH-1:0] pc_out;   // output pc + 1 shifted left 2 bits
output [I_DATAWIDTH-1:0] next_pc;
output [31:26] opcode;
output [25:21] rs;
output [20:16] rt;
output [15:11] rd;
output [10:6] sa;
output [15:0] offset;
output [25:0] instr_index;
output [5:0] func;
output [I_DATAWIDTH-1:0] instr;


wire [I_DATAWIDTH-1:0] pc_plus_1;
reg [I_DATAWIDTH-1:0] pc;
wire ctrl_load;

// prediction stuff
wire prediction;
wire prediction_saved;
wire predict_en_saved;
wire [I_DATAWIDTH-1:0] pc_rollbacknottaken;
wire [I_DATAWIDTH-1:0] pc_rollback;

reg [I_DATAWIDTH-1:0] _next_pc;
reg pc_load_en;
reg predict_result;

// tolerating waits stuff
reg [I_DATAWIDTH-1:0] pc_save_data;
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
    .address_a (next_pc[I_DATAWIDTH-1:2]),
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
        imem.width_a = I_DATAWIDTH, 
        imem.widthad_a = I_ADDRESSWIDTH,
        imem.numwords_a = I_SIZE,
        imem.operation_mode = "BIDIR_DUAL_PORT",    // changed
        imem.width_b = I_DATAWIDTH,                 // new
        imem.widthad_b = I_ADDRESSWIDTH,            // new
        imem.numwords_b = I_SIZE,                   // new
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
register pcrollback( (prediction) ? pc_plus_1 : predict_tgt_pc, 
    clk, resetn,en&predict_en, pc_rollback);
  defparam pcrollback.WIDTH=I_DATAWIDTH;

register pcrollbacknt( pc, clk, resetn,en&predict_en, pc_rollbacknottaken);
  defparam pcrollbacknt.WIDTH=I_DATAWIDTH;

//register pcrollbacktk(predict_tgt_pc, clk, resetn,predict_en, 
    //pc_rollback);
  //defparam pcrollbacktk.WIDTH=I_DATAWIDTH;

register buf_predict({prediction,predict_en&(pcwrop!=1)},clk,resetn,en&predict_en, 
    {prediction_saved,predict_en_saved});
  defparam buf_predict.WIDTH=2;
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
    _next_pc=pc_plus_1;
    pc_load_en=0;
    debug=0;

    if (interrupt)
    begin   // If interrupt occurs, jump to EXCEPTION_ADDRESS
      _next_pc=EXCEPTION_ADDRESS;
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
end

// Separated from above always block because not dependent on pc_advance
always@*
  begin
    predict_result=1;  // Used to flush pipe
    if (ctrl_load & !predict_result_rdy)
      predict_result=~we;  // Only squash proc pipeline when not stalled
    else if (predict_result_rdy & (ctrl_load!=prediction_saved) & predict_en_saved)
      predict_result=~we;
    else if (pc_save)
      predict_result=~(squash_save&~is_delayslot);
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

pipereg pipereg (
  .clk(clk),
  .resetn(resetn),
  .d(is_branch),
  .squashn(1'b1),
  .en(pc_advance),
  .q(is_delayslot));
  defparam
    pipereg.WIDTH=1;
/***************************************************************************/

wire prediction_tmp;
wire predict_result_rdy_tmp;

assign predict_result_rdy_tmp=predict_result_rdy&predict_en_saved;

branchpredict bpredictor ( 
    .clk(clk),
    .resetn(resetn),
    .predict(en),
    .prediction(prediction_tmp),
    .pc_predict({next_pc[I_DATAWIDTH-1:3],2'b0}),
    .result_rdy(predict_result_rdy_tmp),
    .result(ctrl_load),
    .pc_result({pc_rollbacknottaken[I_DATAWIDTH-1:3],2'b0}) );

assign prediction=(pcwrop!=1) ? prediction_tmp :1;

endmodule

//`include "components.v"




module system ( 
	clk,
	resetn,
	boot_iaddr,
	boot_idata,
	boot_iwe,
	boot_daddr,
	boot_ddata,
	boot_dwe,
	ifetch_bus_ecause,
	ifetch_bus_squashn,
	ifetch_bus_address,
	ifetch_bus_en,
	ifetch_bus_readdata,
	ifetch_bus_wait,
	data_mem_bus_ecause,
	data_mem_bus_address,
	data_mem_bus_en,
	data_mem_bus_we,
	data_mem_bus_byteen,
	data_mem_bus_writedata,
	data_mem_bus_readdata,
	data_mem_bus_wait,
	cop2_fromcop2_wait,
	cop2_fromcop2_en,
	cop2_fromcop2,
	cop2_tocop2_wait,
	cop2_tocop2_en,
	cop2_tocop2,
	cop0_badvaddr_we,
	cop0_badvaddr_in,
	cop0_ext_cause_in,

  // PETES CHANGE for tracing
  trc_addr,
  trc_data,
  trc_we,
  trc_stall,
  trc_pipestall,

	nop10_q
	);

/************************* IO Declarations *********************/
//`include "isa.v"

parameter     OP_SPECIAL      = 6'b000000;
parameter     OP_REGIMM       = 6'b000001;
parameter     OP_J            = 6'b000010;
parameter     OP_JAL          = 6'b000011;
parameter     OP_BEQ          = 6'b000100;
parameter     OP_BNE          = 6'b000101;
parameter     OP_BLEZ         = 6'b000110;
parameter     OP_BGTZ         = 6'b000111;

parameter     OP_ADDI         = 6'b001000;
parameter     OP_ADDIU        = 6'b001001;
parameter     OP_SLTI         = 6'b001010;
parameter     OP_SLTIU        = 6'b001011;
parameter     OP_ANDI         = 6'b001100;
parameter     OP_ORI          = 6'b001101;
parameter     OP_XORI         = 6'b001110;
parameter     OP_LUI          = 6'b001111;

parameter     OP_LB           = 6'b100000;
parameter     OP_LH           = 6'b100001;
parameter     OP_LWL          = 6'b100010;
parameter     OP_LW           = 6'b100011;
parameter     OP_LBU          = 6'b100100;
parameter     OP_LHU          = 6'b100101;
parameter     OP_LWR          = 6'b100110;

parameter     OP_SB           = 6'b101x00;
parameter     OP_SH           = 6'b101x01;
parameter     OP_SWL          = 6'b101010;
parameter     OP_SW           = 6'b101x11;
parameter     OP_SWR          = 6'b101110;

/****** FUNCTION CLASS - bits 5...0 *******/
parameter     FUNC_SLL        = 6'b000000;
parameter     FUNC_SRL        = 6'b000010;
parameter     FUNC_SRA        = 6'b000011;
parameter     FUNC_SLLV       = 6'b000100;
parameter     FUNC_SRLV       = 6'b000110;
parameter     FUNC_SRAV       = 6'b000111;

parameter     FUNC_JR         = 6'b001xx0;
parameter     FUNC_JALR       = 6'b001xx1;

parameter     FUNC_MFHI       = 6'bx10x00;
parameter     FUNC_MTHI       = 6'bx10x01;
parameter     FUNC_MFLO       = 6'bx10x10;
parameter     FUNC_MTLO       = 6'bx10x11;

parameter     FUNC_MULT       = 6'bx11x00;
parameter     FUNC_MULTU      = 6'bx11x01;
parameter     FUNC_DIV        = 6'bx11x10;
parameter     FUNC_DIVU       = 6'bx11x11;

parameter     FUNC_ADD        = 6'b100000;
parameter     FUNC_ADDU       = 6'b100001;
parameter     FUNC_SUB        = 6'b100010;
parameter     FUNC_SUBU       = 6'b100011;
parameter     FUNC_AND        = 6'b100100;
parameter     FUNC_OR         = 6'b100101;
parameter     FUNC_XOR        = 6'b100110;
parameter     FUNC_NOR        = 6'b100111;

parameter     FUNC_SLT        = 6'b101010;
parameter     FUNC_SLTU       = 6'b101011;

/****** REGIMM Class - bits 20...16 *******/
parameter     FUNC_BLTZ       = 1'b0;
parameter     FUNC_BGEZ       = 1'b1;

parameter     OP_COP2       = 6'b010010;
parameter     COP2_FUNC_CFC2     = 6'b111000;
parameter     COP2_FUNC_CTC2     = 6'b111010;
parameter     COP2_FUNC_MTC2     = 6'b111011;

parameter     OP_COP0       = 6'b010000;
parameter     COP0_MFC0     = 5'b00000;
parameter     COP0_MTC0     = 5'b00100;

//parameter     FUNC_BLTZAL     = 5'b10000;
//parameter     FUNC_BGEZAL     = 5'b10001;

/****** 
 * Original REGIMM class, compressed above to save decode logic
parameter     FUNC_BLTZ       = 5'b00000;
parameter     FUNC_BGEZ       = 5'b00001;
parameter     FUNC_BLTZAL     = 5'b10000;
parameter     FUNC_BGEZAL     = 5'b10001;
*/

input clk;
input resetn;
input [31:0] boot_iaddr;
input [31:0] boot_idata;
input boot_iwe;
input [31:0] boot_daddr;
input [31:0] boot_ddata;
input boot_dwe;
input	[ 31 : 0 ]	ifetch_bus_ecause;
output	ifetch_bus_squashn;
output	[ 31 : 0 ]	ifetch_bus_address;
output	ifetch_bus_en;
input	[ 31 : 0 ]	ifetch_bus_readdata;
input	ifetch_bus_wait;
input	[ 31 : 0 ]	data_mem_bus_ecause;
output	[ 31 : 0 ]	data_mem_bus_address;
output	data_mem_bus_en;
output	data_mem_bus_we;
output	[ 3 : 0 ]	data_mem_bus_byteen;
output	[ 31 : 0 ]	data_mem_bus_writedata;
input	[ 31 : 0 ]	data_mem_bus_readdata;
input	data_mem_bus_wait;
output	cop2_fromcop2_wait;
input	cop2_fromcop2_en;
input	[ 31 : 0 ]	cop2_fromcop2;
input	cop2_tocop2_wait;
output	cop2_tocop2_en;
output	[ 31 : 0 ]	cop2_tocop2;
input	cop0_badvaddr_we;
input	[ 31 : 0 ]	cop0_badvaddr_in;
input	[ 31 : 0 ]	cop0_ext_cause_in;
output [31:0] nop10_q;

// PETES CHANGE for tracing
output  [ 4 : 0 ]   trc_addr;
output  [ 31 : 0 ]  trc_data;
output              trc_we;
input               trc_stall;
output              trc_pipestall;


/*********************** Signal Declarations *******************/
wire	branch_mispred;
wire	stall_2nd_delayslot;
wire	has_delayslot;
wire	haz_zeroer0_q_pipereg5_q;
wire	haz_zeroer_q_pipereg5_q;
		// Datapath signals declarations
wire	addersub_result_slt;
wire	[ 31 : 0 ]	addersub_result;
wire	[ 31 : 0 ]	logic_unit_result;
wire	[ 31 : 0 ]	ifetch_pc_out;
wire	[ 31 : 0 ]	ifetch_instr;
wire	[ 5 : 0 ]	ifetch_opcode;
wire	[ 5 : 0 ]	ifetch_func;
wire	[ 4 : 0 ]	ifetch_rs;
wire	[ 4 : 0 ]	ifetch_rt;
wire	[ 4 : 0 ]	ifetch_rd;
wire	[ 25 : 0 ]	ifetch_instr_index;
wire	[ 15 : 0 ]	ifetch_offset;
wire	[ 4 : 0 ]	ifetch_sa;
wire	[ 31 : 0 ]	ifetch_next_pc;
wire	ifetch_predict_result;
wire	[ 31 : 0 ]	ifetch_ecause;
wire	[ 31 : 0 ]	ifetch_epc;
wire	[ 31 : 0 ]	mul_shift_result;
wire	[ 31 : 0 ]	mul_lo;
wire	[ 31 : 0 ]	mul_hi;
wire	ctrl_mul_stalled;
wire	[ 31 : 0 ]	div_remainder;
wire	[ 31 : 0 ]	div_quotient;
wire	ctrl_div_stalled;
wire	[ 31 : 0 ]	data_mem_d_loadresult;
wire	[ 31 : 0 ]	data_mem_ecause;
wire	ctrl_data_mem_stalled;
wire	[ 31 : 0 ]	reg_file_b_readdataout;
wire	[ 31 : 0 ]	reg_file_a_readdataout;
wire	[ 31 : 0 ]	pcadder_result;
wire	[ 31 : 0 ]	signext16_out;
wire	[ 31 : 0 ]	merge26lo_out;
wire	branchresolve_eqz;
wire	branchresolve_gez;
wire	branchresolve_gtz;
wire	branchresolve_lez;
wire	branchresolve_ltz;
wire	branchresolve_ne;
wire	branchresolve_eq;
wire	[ 31 : 0 ]	lo_reg_q;
wire	[ 31 : 0 ]	hi_reg_q;
wire	[ 31 : 0 ]	const11_out;
wire	[ 31 : 0 ]	const12_out;
wire	[ 31 : 0 ]	const_out;
wire	[ 31 : 0 ]	pipereg_q;
wire	[ 4 : 0 ]	pipereg5_q;
wire	[ 4 : 0 ]	pipereg2_q;
wire	[ 31 : 0 ]	pipereg6_q;
wire	[ 31 : 0 ]	pipereg26_q;
wire	[ 31 : 0 ]	pipereg8_q;
wire	pipereg7_q;
wire	[ 31 : 0 ]	fakedelay_q;
wire	[ 31 : 0 ]	pipereg27_q;
wire	[ 31 : 0 ]	pipereg28_q;
wire	[ 31 : 0 ]	pipereg29_q;
wire	[ 31 : 0 ]	pipereg30_q;
wire	[ 31 : 0 ]	nop_q;
wire	[ 31 : 0 ]	nop10_q;
wire	[ 31 : 0 ]	nop13_q;
wire	[ 31 : 0 ]	nop9_q;
wire	[ 4 : 0 ]	zeroer_q;
wire	[ 4 : 0 ]	zeroer0_q;
wire	[ 4 : 0 ]	zeroer4_q;
wire	[ 31 : 0 ]	cop2_tocpu;
wire	ctrl_cop2_stalled;
wire	[ 31 : 0 ]	cop0_status;
wire	[ 31 : 0 ]	cop0_tocpu;
wire	cop0_exception;
wire	ctrl_cop0_stalled;
wire	[ 31 : 0 ]	mux2to1_mul_opA_out;
wire	[ 31 : 0 ]	mux2to1_addersub_opA_out;
wire	[ 4 : 0 ]	mux3to1_mul_sa_out;
wire	[ 31 : 0 ]	mux2to1_hi_reg_d_out;
wire	[ 31 : 0 ]	mux2to1_lo_reg_d_out;
wire	[ 31 : 0 ]	mux9to1_nop13_d_out;
wire	[ 31 : 0 ]	mux2to1_pipereg_d_out;
wire	[ 31 : 0 ]	mux2to1_pipereg6_d_out;
wire	mux6to1_pipereg7_d_out;
wire	[ 31 : 0 ]	mux3to1_nop9_d_out;
wire	[ 4 : 0 ]	mux3to1_zeroer4_d_out;
wire	[ 5 : 0 ]	pipereg15_q;
wire	[ 4 : 0 ]	pipereg16_q;
wire	[ 5 : 0 ]	pipereg14_q;
wire	branch_detector_is_branch;
wire	[ 4 : 0 ]	pipereg17_q;
wire	[ 5 : 0 ]	pipereg19_q;
wire	[ 5 : 0 ]	pipereg18_q;
wire	[ 4 : 0 ]	pipereg20_q;
wire	[ 4 : 0 ]	pipereg21_q;
wire	pipereg22_q;
wire	pipereg23_q;
wire	[ 31 : 0 ]	mux2to1_nop_d_out;
wire	pipereg31_q;
wire	[ 31 : 0 ]	mux2to1_nop10_d_out;
wire	pipereg32_q;
wire	pipereg25_q;
/***************** Control Signals ***************/
		//Decoded Opcode signal declarations
reg	[ 2 : 0 ]	ctrl_mux6to1_pipereg7_d_sel;
reg	[ 3 : 0 ]	ctrl_mux9to1_nop13_d_sel;
reg	[ 1 : 0 ]	ctrl_mux3to1_nop9_d_sel;
reg	ctrl_mux2to1_lo_reg_d_sel;
reg	ctrl_mux2to1_hi_reg_d_sel;
reg	ctrl_mux2to1_mul_opA_sel;
reg	[ 1 : 0 ]	ctrl_mux3to1_mul_sa_sel;
reg	ctrl_mux2to1_addersub_opA_sel;
reg	ctrl_mux2to1_pipereg6_d_sel;
reg	[ 1 : 0 ]	ctrl_mux3to1_zeroer4_d_sel;
reg	ctrl_mux2to1_pipereg_d_sel;
reg	ctrl_zeroer4_en;
reg	ctrl_zeroer0_en;
reg	ctrl_zeroer_en;
reg	ctrl_ifetch_pcwrop;
reg	ctrl_ifetch_op;
reg	[ 2 : 0 ]	ctrl_addersub_op;
reg	[ 3 : 0 ]	ctrl_data_mem_op;
reg	ctrl_div_sign;
reg	[ 2 : 0 ]	ctrl_mul_op;
reg	[ 1 : 0 ]	ctrl_logic_unit_op;
		//Enable signal declarations
reg	ctrl_cop0_fromcpu_en;
reg	ctrl_cop0_tocpu_en;
reg	ctrl_cop2_fromcpu_en;
reg	ctrl_cop2_tocpu_en;
reg	ctrl_lo_reg_en;
reg	ctrl_ifetch_we;
reg	ctrl_hi_reg_en;
reg	ctrl_branchresolve_en;
reg	ctrl_reg_file_c_we;
reg	ctrl_reg_file_b_en;
reg	ctrl_reg_file_a_en;
reg	ctrl_ifetch_en;
reg	ctrl_data_mem_en;
reg	ctrl_div_en;
reg	ctrl_mul_start;
		//Other Signals
wire	squash_stage3;
wire	stall_out_stage3;
wire	squash_stage2;
wire	stall_out_stage2;
wire	ctrl_pipereg25_squashn;
wire	ctrl_pipereg8_squashn;
wire	ctrl_pipereg7_squashn;
wire	ctrl_pipereg18_squashn;
wire	ctrl_pipereg19_squashn;
wire	ctrl_pipereg20_squashn;
wire	ctrl_pipereg21_squashn;
wire	ctrl_pipereg25_resetn;
wire	ctrl_pipereg8_resetn;
wire	ctrl_pipereg7_resetn;
wire	ctrl_pipereg18_resetn;
wire	ctrl_pipereg19_resetn;
wire	ctrl_pipereg20_resetn;
wire	ctrl_pipereg21_resetn;
wire	ctrl_pipereg25_en;
wire	ctrl_pipereg8_en;
wire	ctrl_pipereg7_en;
wire	ctrl_pipereg18_en;
wire	ctrl_pipereg19_en;
wire	ctrl_pipereg20_en;
wire	ctrl_pipereg21_en;
wire	squash_stage1;
wire	stall_out_stage1;
wire	ctrl_pipereg32_squashn;
wire	ctrl_pipereg31_squashn;
wire	ctrl_pipereg30_squashn;
wire	ctrl_pipereg29_squashn;
wire	ctrl_pipereg28_squashn;
wire	ctrl_pipereg27_squashn;
wire	ctrl_pipereg26_squashn;
wire	ctrl_pipereg23_squashn;
wire	ctrl_pipereg_squashn;
wire	ctrl_pipereg5_squashn;
wire	ctrl_pipereg2_squashn;
wire	ctrl_pipereg6_squashn;
wire	ctrl_pipereg14_squashn;
wire	ctrl_pipereg15_squashn;
wire	ctrl_pipereg16_squashn;
wire	ctrl_pipereg17_squashn;
wire	ctrl_pipereg32_resetn;
wire	ctrl_pipereg31_resetn;
wire	ctrl_pipereg30_resetn;
wire	ctrl_pipereg29_resetn;
wire	ctrl_pipereg28_resetn;
wire	ctrl_pipereg27_resetn;
wire	ctrl_pipereg26_resetn;
wire	ctrl_pipereg23_resetn;
wire	ctrl_pipereg_resetn;
wire	ctrl_pipereg5_resetn;
wire	ctrl_pipereg2_resetn;
wire	ctrl_pipereg6_resetn;
wire	ctrl_pipereg14_resetn;
wire	ctrl_pipereg15_resetn;
wire	ctrl_pipereg16_resetn;
wire	ctrl_pipereg17_resetn;
wire	ctrl_pipereg32_en;
wire	ctrl_pipereg31_en;
wire	ctrl_pipereg30_en;
wire	ctrl_pipereg29_en;
wire	ctrl_pipereg28_en;
wire	ctrl_pipereg27_en;
wire	ctrl_pipereg26_en;
wire	ctrl_pipereg23_en;
wire	ctrl_pipereg_en;
wire	ctrl_pipereg5_en;
wire	ctrl_pipereg2_en;
wire	ctrl_pipereg6_en;
wire	ctrl_pipereg14_en;
wire	ctrl_pipereg15_en;
wire	ctrl_pipereg16_en;
wire	ctrl_pipereg17_en;
reg	predictme;


/****************************** Control **************************/
		//Decode Logic for Opcode and Multiplex Select signals
always@(ifetch_opcode or ifetch_func or ifetch_rt or ifetch_rs)
begin
		// Initialize control opcodes to zero
	ctrl_mux2to1_pipereg6_d_sel = 0;
	ctrl_mux3to1_zeroer4_d_sel = 0;
	ctrl_mux2to1_pipereg_d_sel = 0;
	ctrl_zeroer4_en = 0;
	ctrl_zeroer0_en = 0;
	ctrl_zeroer_en = 0;
	
	casex (ifetch_opcode)
		OP_ADDI:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_ADDIU:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_ANDI:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_BEQ:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_BGTZ:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
			ctrl_zeroer_en = 1;
		end
		OP_BLEZ:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
			ctrl_zeroer_en = 1;
		end
		OP_BNE:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_COP0:
		casex (ifetch_rs)
			COP0_MFC0:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 2;
				ctrl_zeroer4_en = 1;
			end
			COP0_MTC0:
				ctrl_zeroer0_en = 1;
		endcase
		OP_COP2:
		casex (ifetch_func)
			COP2_FUNC_CFC2:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 2;
				ctrl_zeroer4_en = 1;
			end
			COP2_FUNC_CTC2:
				ctrl_zeroer0_en = 1;
			COP2_FUNC_MTC2:
				ctrl_zeroer0_en = 1;
		endcase
		OP_J:
			ctrl_mux2to1_pipereg6_d_sel = 0;
		OP_JAL:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 0;
			ctrl_zeroer4_en = 1;
		end
		OP_LB:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_LBU:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_LH:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_LHU:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_LUI:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
		end
		OP_LW:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_ORI:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_REGIMM:
		casex (ifetch_rt[0])
			FUNC_BGEZ:
			begin
				ctrl_mux2to1_pipereg6_d_sel = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_BLTZ:
			begin
				ctrl_mux2to1_pipereg6_d_sel = 1;
				ctrl_zeroer_en = 1;
			end
		endcase
		OP_SB:
		begin
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SH:
		begin
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SLTI:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SLTIU:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SPECIAL:
		casex (ifetch_func)
			FUNC_ADD:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_ADDU:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_AND:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_DIV:
			begin
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_DIVU:
			begin
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_JALR:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_JR:
				ctrl_zeroer_en = 1;
			FUNC_MFHI:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
			end
			FUNC_MFLO:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
			end
			FUNC_MULT:
			begin
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_MULTU:
			begin
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_NOR:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_OR:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SLL:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
			end
			FUNC_SLLV:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SLT:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SLTU:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SRA:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
			end
			FUNC_SRAV:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SRL:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
			end
			FUNC_SRLV:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SUB:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SUBU:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_XOR:
			begin
				ctrl_mux3to1_zeroer4_d_sel = 1;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
		endcase
		OP_SW:
		begin
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_XORI:
		begin
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
			ctrl_zeroer_en = 1;
		end
	endcase
end
		//Logic for enable signals in Pipe Stage 1
always@(ifetch_opcode or ifetch_func or ifetch_rt[0] or ifetch_rs or stall_out_stage2)
begin
	ctrl_reg_file_b_en = 1 &~stall_out_stage2;
	ctrl_reg_file_a_en = 1 &~stall_out_stage2;
	ctrl_ifetch_en = 1 &~stall_out_stage2;
end
		//Decode Logic for Opcode and Multiplex Select signals
always@(pipereg14_q or pipereg15_q or pipereg16_q or pipereg17_q)
begin
		// Initialize control opcodes to zero
	ctrl_mux6to1_pipereg7_d_sel = 0;
	ctrl_mux9to1_nop13_d_sel = 0;
	ctrl_mux3to1_nop9_d_sel = 0;
	ctrl_mux2to1_lo_reg_d_sel = 0;
	ctrl_mux2to1_hi_reg_d_sel = 0;
	ctrl_mux2to1_mul_opA_sel = 0;
	ctrl_mux3to1_mul_sa_sel = 0;
	ctrl_mux2to1_addersub_opA_sel = 0;
	ctrl_ifetch_pcwrop = 0;
	ctrl_addersub_op = 0;
	ctrl_data_mem_op = 0;
	ctrl_div_sign = 0;
	ctrl_mul_op = 0;
	ctrl_logic_unit_op = 0;
	
	casex (pipereg14_q)
		OP_ADDI:
		begin
			ctrl_mux9to1_nop13_d_sel = 6;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
		end
		OP_ADDIU:
		begin
			ctrl_mux9to1_nop13_d_sel = 6;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 1;
		end
		OP_ANDI:
		begin
			ctrl_mux9to1_nop13_d_sel = 4;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_logic_unit_op = 0;
		end
		OP_BEQ:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 5;
			ctrl_ifetch_pcwrop = 0;
		end
		OP_BGTZ:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_ifetch_pcwrop = 0;
		end
		OP_BLEZ:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 3;
			ctrl_ifetch_pcwrop = 0;
		end
		OP_BNE:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 4;
			ctrl_ifetch_pcwrop = 0;
		end
		OP_COP0:
		casex (pipereg17_q)
			COP0_MFC0:
				ctrl_mux9to1_nop13_d_sel = 7;
		endcase
		OP_COP2:
		casex (pipereg15_q)
			COP2_FUNC_CFC2:
				ctrl_mux9to1_nop13_d_sel = 8;
		endcase
		OP_J:
			ctrl_ifetch_pcwrop = 1;
		OP_JAL:
		begin
			ctrl_mux9to1_nop13_d_sel = 6;
			ctrl_mux2to1_addersub_opA_sel = 1;
			ctrl_ifetch_pcwrop = 1;
			ctrl_addersub_op = 1;
		end
		OP_LB:
		begin
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 7;
		end
		OP_LBU:
		begin
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 3;
		end
		OP_LH:
		begin
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 5;
		end
		OP_LHU:
		begin
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 1;
		end
		OP_LUI:
		begin
			ctrl_mux9to1_nop13_d_sel = 3;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 1;
			ctrl_mul_op = 0;
		end
		OP_LW:
		begin
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 0;
		end
		OP_ORI:
		begin
			ctrl_mux9to1_nop13_d_sel = 4;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_logic_unit_op = 1;
		end
		OP_REGIMM:
		casex (pipereg16_q[0])
			FUNC_BGEZ:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 1;
				ctrl_ifetch_pcwrop = 0;
			end
			FUNC_BLTZ:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 2;
				ctrl_ifetch_pcwrop = 0;
			end
		endcase
		OP_SB:
		begin
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 11;
		end
		OP_SH:
		begin
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 9;
		end
		OP_SLTI:
		begin
			ctrl_mux9to1_nop13_d_sel = 5;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 6;
		end
		OP_SLTIU:
		begin
			ctrl_mux9to1_nop13_d_sel = 5;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 4;
		end
		OP_SPECIAL:
		casex (pipereg15_q)
			FUNC_ADD:
			begin
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_addersub_op = 3;
			end
			FUNC_ADDU:
			begin
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_addersub_op = 1;
			end
			FUNC_AND:
			begin
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_logic_unit_op = 0;
			end
			FUNC_DIV:
			begin
				ctrl_mux2to1_lo_reg_d_sel = 1;
				ctrl_mux2to1_hi_reg_d_sel = 1;
				ctrl_div_sign = 1;
			end
			FUNC_DIVU:
			begin
				ctrl_mux2to1_lo_reg_d_sel = 1;
				ctrl_mux2to1_hi_reg_d_sel = 1;
				ctrl_div_sign = 0;
			end
			FUNC_JALR:
			begin
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux2to1_addersub_opA_sel = 1;
				ctrl_addersub_op = 1;
			end
			FUNC_MFHI:
				ctrl_mux9to1_nop13_d_sel = 1;
			FUNC_MFLO:
				ctrl_mux9to1_nop13_d_sel = 0;
			FUNC_MULT:
			begin
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 1;
				ctrl_mul_op = 6;
			end
			FUNC_MULTU:
			begin
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 1;
				ctrl_mul_op = 4;
			end
			FUNC_NOR:
			begin
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_logic_unit_op = 3;
			end
			FUNC_OR:
			begin
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_logic_unit_op = 1;
			end
			FUNC_SLL:
			begin
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mul_op = 0;
			end
			FUNC_SLLV:
			begin
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 2;
				ctrl_mul_op = 0;
			end
			FUNC_SLT:
			begin
				ctrl_mux9to1_nop13_d_sel = 5;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_addersub_op = 6;
			end
			FUNC_SLTU:
			begin
				ctrl_mux9to1_nop13_d_sel = 5;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_addersub_op = 4;
			end
			FUNC_SRA:
			begin
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mul_op = 3;
			end
			FUNC_SRAV:
			begin
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 2;
				ctrl_mul_op = 3;
			end
			FUNC_SRL:
			begin
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mul_op = 1;
			end
			FUNC_SRLV:
			begin
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 2;
				ctrl_mul_op = 1;
			end
			FUNC_SUB:
			begin
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_addersub_op = 0;
			end
			FUNC_SUBU:
			begin
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_addersub_op = 2;
			end
			FUNC_XOR:
			begin
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_logic_unit_op = 2;
			end
		endcase
		OP_SW:
		begin
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 8;
		end
		OP_XORI:
		begin
			ctrl_mux9to1_nop13_d_sel = 4;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_logic_unit_op = 2;
		end
	endcase
end
		//Logic for enable signals in Pipe Stage 2
always@(pipereg14_q or pipereg15_q or pipereg16_q[0] or pipereg17_q or stall_out_stage3 or ctrl_mul_stalled or ctrl_data_mem_stalled or ctrl_div_stalled or ctrl_cop2_stalled or ctrl_cop0_stalled)
begin
	ctrl_cop0_fromcpu_en = 0;
	ctrl_cop0_tocpu_en = 0;
	ctrl_cop2_fromcpu_en = 0;
	ctrl_cop2_tocpu_en = 0;
	ctrl_lo_reg_en = 0;
	ctrl_hi_reg_en = 0;
	ctrl_branchresolve_en = 0;
	ctrl_reg_file_c_we = 0;
	ctrl_data_mem_en = 0;
	ctrl_div_en = 0;
	ctrl_mul_start = 0;
	casex (pipereg14_q)
		OP_ADDI:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_ADDIU:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_ANDI:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_BEQ:
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_BGTZ:
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_BLEZ:
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_BNE:
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_COP0:
		casex (pipereg17_q)
			COP0_MFC0:
			begin
				ctrl_cop0_tocpu_en = 1 &~stall_out_stage3;
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			end
			COP0_MTC0:
				ctrl_cop0_fromcpu_en = 1 &~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		endcase
		OP_COP2:
		casex (pipereg15_q)
			COP2_FUNC_CFC2:
			begin
				ctrl_cop2_tocpu_en = 1 &~stall_out_stage3;
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			end
			COP2_FUNC_CTC2:
				ctrl_cop2_fromcpu_en = 1 &~ctrl_cop0_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			COP2_FUNC_MTC2:
				ctrl_cop2_fromcpu_en = 1 &~ctrl_cop0_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		endcase
		OP_JAL:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_LB:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		end
		OP_LBU:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		end
		OP_LH:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		end
		OP_LHU:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		end
		OP_LUI:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_mul_start = 1 &~stall_out_stage3;
		end
		OP_LW:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		end
		OP_ORI:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_REGIMM:
		casex (pipereg16_q[0])
			FUNC_BGEZ:
				ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_BLTZ:
				ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		endcase
		OP_SB:
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		OP_SH:
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		OP_SLTI:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_SLTIU:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		OP_SPECIAL:
		casex (pipereg15_q)
			FUNC_ADD:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_ADDU:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_AND:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_DIV:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_div_en = 1 &~stall_out_stage3;
			end
			FUNC_DIVU:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_div_en = 1 &~stall_out_stage3;
			end
			FUNC_JALR:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_MFHI:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_MFLO:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_MULT:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_MULTU:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_NOR:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_OR:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_SLL:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_SLLV:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_SLT:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_SLTU:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_SRA:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_SRAV:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_SRL:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_SRLV:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
			end
			FUNC_SUB:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_SUBU:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			FUNC_XOR:
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
		endcase
		OP_SW:
			ctrl_data_mem_en = 1 &~stall_out_stage3;
		OP_XORI:
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
	endcase
end
		//Decode Logic for Opcode and Multiplex Select signals
always@(pipereg18_q or pipereg19_q or pipereg20_q or pipereg21_q)
begin
		// Initialize control opcodes to zero
	ctrl_ifetch_op = 0;
	
	casex (pipereg18_q)
		OP_BEQ:
			ctrl_ifetch_op = 0;
		OP_BGTZ:
			ctrl_ifetch_op = 0;
		OP_BLEZ:
			ctrl_ifetch_op = 0;
		OP_BNE:
			ctrl_ifetch_op = 0;
		OP_REGIMM:
		casex (pipereg20_q[0])
			FUNC_BGEZ:
				ctrl_ifetch_op = 0;
			FUNC_BLTZ:
				ctrl_ifetch_op = 0;
		endcase
		OP_SPECIAL:
		casex (pipereg19_q)
			FUNC_JALR:
				ctrl_ifetch_op = 1;
			FUNC_JR:
				ctrl_ifetch_op = 1;
		endcase
	endcase
end
		//Logic for enable signals in Pipe Stage 3
always@(pipereg18_q or pipereg19_q or pipereg20_q[0] or pipereg21_q or 1'b0)
begin
	ctrl_ifetch_we = 0;
	casex (pipereg18_q)
		OP_BEQ:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_BGTZ:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_BLEZ:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_BNE:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_REGIMM:
		casex (pipereg20_q[0])
			FUNC_BGEZ:
				ctrl_ifetch_we = 1 &~1'b0;
			FUNC_BLTZ:
				ctrl_ifetch_we = 1 &~1'b0;
		endcase
		OP_SPECIAL:
		casex (pipereg19_q)
			FUNC_JALR:
				ctrl_ifetch_we = 1 &~1'b0;
			FUNC_JR:
				ctrl_ifetch_we = 1 &~1'b0;
		endcase
	endcase
end

/********* Stall Network & PipeReg Control ********/
assign stall_out_stage1 = stall_out_stage2;
assign ctrl_pipereg17_en = ~stall_out_stage1;
assign ctrl_pipereg16_en = ~stall_out_stage1;
assign ctrl_pipereg15_en = ~stall_out_stage1;
assign ctrl_pipereg14_en = ~stall_out_stage1;
assign ctrl_pipereg6_en = ~stall_out_stage1;
assign ctrl_pipereg2_en = ~stall_out_stage1;
assign ctrl_pipereg5_en = ~stall_out_stage1;
assign ctrl_pipereg_en = ~stall_out_stage1;
assign ctrl_pipereg23_en = ~stall_out_stage1;
assign ctrl_pipereg26_en = ~stall_out_stage1;
assign ctrl_pipereg27_en = ~stall_out_stage1;
assign ctrl_pipereg28_en = ~stall_out_stage1;
assign ctrl_pipereg29_en = ~stall_out_stage1;
assign ctrl_pipereg30_en = ~stall_out_stage1;
assign ctrl_pipereg31_en = ~stall_out_stage1;
assign ctrl_pipereg32_en = ~stall_out_stage1;
assign stall_out_stage2 = stall_out_stage3|ctrl_cop0_stalled|ctrl_cop2_stalled|ctrl_div_stalled|ctrl_data_mem_stalled|ctrl_mul_stalled;
assign ctrl_pipereg21_en = ~stall_out_stage2;
assign ctrl_pipereg20_en = ~stall_out_stage2;
assign ctrl_pipereg19_en = ~stall_out_stage2;
assign ctrl_pipereg18_en = ~stall_out_stage2;
assign ctrl_pipereg7_en = ~stall_out_stage2;
assign ctrl_pipereg8_en = ~stall_out_stage2;
assign ctrl_pipereg25_en = ~stall_out_stage2;
assign stall_out_stage3 = 1'b0;
assign branch_mispred = (!ifetch_predict_result);
assign stall_2nd_delayslot = branch_detector_is_branch&has_delayslot;
assign has_delayslot = pipereg22_q;

		//Identify branches that will be predicted
always@(ifetch_opcode or ifetch_func or ifetch_rt[0] or ifetch_rs)
begin
	predictme=0;
	casex (ifetch_opcode)
		OP_BEQ:
			predictme=1;
		OP_BGTZ:
			predictme=1;
		OP_BLEZ:
			predictme=1;
		OP_BNE:
			predictme=1;
		OP_J:
			predictme=1;
		OP_JAL:
			predictme=1;
		OP_REGIMM:
		casex (ifetch_rt[0])
			FUNC_BGEZ:
				predictme=1;
			FUNC_BLTZ:
				predictme=1;
		endcase
	endcase
end

assign squash_stage1 = ((stall_out_stage1&~stall_out_stage2))|~resetn;
assign ctrl_pipereg17_resetn = ~squash_stage1;
assign ctrl_pipereg16_resetn = ~squash_stage1;
assign ctrl_pipereg15_resetn = ~squash_stage1;
assign ctrl_pipereg14_resetn = ~squash_stage1;
assign ctrl_pipereg6_resetn = ~squash_stage1;
assign ctrl_pipereg2_resetn = ~squash_stage1;
assign ctrl_pipereg5_resetn = ~squash_stage1;
assign ctrl_pipereg_resetn = ~squash_stage1;
assign ctrl_pipereg23_resetn = ~squash_stage1;
assign ctrl_pipereg26_resetn = ~squash_stage1;
assign ctrl_pipereg27_resetn = ~squash_stage1;
assign ctrl_pipereg28_resetn = ~squash_stage1;
assign ctrl_pipereg29_resetn = ~squash_stage1;
assign ctrl_pipereg30_resetn = ~squash_stage1;
assign ctrl_pipereg31_resetn = ~squash_stage1;
assign ctrl_pipereg32_resetn = ~squash_stage1;
assign ctrl_pipereg32_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg31_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg30_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg29_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg28_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg27_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg26_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg23_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg5_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg2_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg6_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg14_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg15_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg16_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_pipereg17_squashn = ~((branch_mispred&~(pipereg22_q&~stall_out_stage1 | 1&stall_out_stage1)) || (cop0_exception));
assign ctrl_ifetch_squashn = ~((branch_mispred&~(pipereg22_q)) || (cop0_exception));
assign squash_stage2 = ((stall_out_stage2&~stall_out_stage3))|~resetn;
assign ctrl_pipereg21_resetn = ~squash_stage2;
assign ctrl_pipereg20_resetn = ~squash_stage2;
assign ctrl_pipereg19_resetn = ~squash_stage2;
assign ctrl_pipereg18_resetn = ~squash_stage2;
assign ctrl_pipereg7_resetn = ~squash_stage2;
assign ctrl_pipereg8_resetn = ~squash_stage2;
assign ctrl_pipereg25_resetn = ~squash_stage2;
assign ctrl_pipereg25_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg8_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg7_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg18_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg19_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg20_squashn = ~((0) || (cop0_exception));
assign ctrl_pipereg21_squashn = ~((0) || (cop0_exception));
assign ctrl_lo_reg_squashn = ~((0) || (cop0_exception));
assign ctrl_hi_reg_squashn = ~((0) || (cop0_exception));
assign ctrl_reg_file_c_squashn = ~((0) || (cop0_exception));
assign squash_stage3 = ((stall_out_stage3&~1'b0))|~resetn;

/****************************** Datapath **************************/
/******************** Hazard Detection Logic ***********************/
assign haz_zeroer0_q_pipereg5_q = (zeroer0_q==pipereg5_q) && (|zeroer0_q);
assign haz_zeroer_q_pipereg5_q = (zeroer_q==pipereg5_q) && (|zeroer_q);

/*************** DATAPATH COMPONENTS **************/
addersub addersub (
	.opB(nop9_q),
	.opA(mux2to1_addersub_opA_out),
	.op(ctrl_addersub_op),
	.result_slt(addersub_result_slt),
	.result(addersub_result));
	defparam
		addersub.WIDTH=32;

logic_unit logic_unit (
	.opB(nop9_q),
	.opA(nop_q),
	.op(ctrl_logic_unit_op),
	.result(logic_unit_result));
	defparam
		logic_unit.WIDTH=32;

ifetch ifetch (
	.clk(clk),
	.resetn(resetn),
	.boot_iaddr(boot_iaddr),
	.boot_idata(boot_idata),
	.boot_iwe(boot_iwe),
	.bus_ecause(ifetch_bus_ecause),
	.bus_squashn(ifetch_bus_squashn),
	.bus_address(ifetch_bus_address),
	.bus_en(ifetch_bus_en),
	.bus_readdata(ifetch_bus_readdata),
	.bus_wait(ifetch_bus_wait),
	.interrupt(cop0_exception),
	.predict_result_rdy(pipereg25_q),
	.predict_en(pipereg23_q),
	.predict_tgt_pc(pipereg6_q),
	.load(pipereg7_q),
	.load_data(pipereg8_q),
	.pcwrop(ctrl_ifetch_pcwrop),
	.op(ctrl_ifetch_op),
	.we(ctrl_ifetch_we),
	.squashn(ctrl_ifetch_squashn),
	.en(ctrl_ifetch_en),
	.pc_out(ifetch_pc_out),
	.instr(ifetch_instr),
	.opcode(ifetch_opcode),
	.func(ifetch_func),
	.rs(ifetch_rs),
	.rt(ifetch_rt),
	.rd(ifetch_rd),
	.instr_index(ifetch_instr_index),
	.offset(ifetch_offset),
	.sa(ifetch_sa),
	.next_pc(ifetch_next_pc),
	.predict_result(ifetch_predict_result),
	.ecause(ifetch_ecause),
	.epc(ifetch_epc));

mul mul (
	.clk(clk),
	.resetn(resetn),
	.sa(mux3to1_mul_sa_out),
	.dst(pipereg5_q),
	.opB(nop10_q),
	.opA(mux2to1_mul_opA_out),
	.op(ctrl_mul_op),
	.start(ctrl_mul_start),
	.stalled(ctrl_mul_stalled),
	.shift_result(mul_shift_result),
	.lo(mul_lo),
	.hi(mul_hi));
	defparam
		mul.WIDTH=32;

div div (
	.clk(clk),
	.resetn(resetn),
	.divider(nop10_q),
	.dividend(nop_q),
	.sign(ctrl_div_sign),
	.en(ctrl_div_en),
	.stalled(ctrl_div_stalled),
	.remainder(div_remainder),
	.quotient(div_quotient));

data_mem data_mem (
	.clk(clk),
	.resetn(resetn),
	.boot_daddr(boot_daddr),
	.boot_ddata(boot_ddata),
	.boot_dwe(boot_dwe),
	.bus_ecause(data_mem_bus_ecause),
	.bus_address(data_mem_bus_address),
	.bus_en(data_mem_bus_en),
	.bus_we(data_mem_bus_we),
	.bus_byteen(data_mem_bus_byteen),
	.bus_writedata(data_mem_bus_writedata),
	.bus_readdata(data_mem_bus_readdata),
	//PETES CHANGE for tracing, was: .bus_wait(data_mem_bus_wait),
	.bus_wait(data_mem_bus_wait|trc_stall),
	.d_address(addersub_result),
	.d_writedata(nop10_q),
	.op(ctrl_data_mem_op),
	.en(ctrl_data_mem_en),
	.stalled(ctrl_data_mem_stalled),
	.d_loadresult(data_mem_d_loadresult),
	.ecause(data_mem_ecause));

reg_file reg_file (
	.clk(clk),
	.resetn(resetn),
	.c_writedatain(nop13_q),
	.c_reg(pipereg5_q),
	.b_reg(zeroer0_q),
	.a_reg(zeroer_q),
	.c_squashn(ctrl_reg_file_c_squashn),
	.c_we(ctrl_reg_file_c_we),
	.b_en(ctrl_reg_file_b_en),
	.a_en(ctrl_reg_file_a_en),
	.b_readdataout(reg_file_b_readdataout),
	.a_readdataout(reg_file_a_readdataout));

pcadder pcadder (
	.offset(signext16_out),
	.pc(ifetch_pc_out),
	.result(pcadder_result));

signext16 signext16 (
	.in(ifetch_offset),
	.out(signext16_out));

merge26lo merge26lo (
	.in2(ifetch_instr_index),
	.in1(ifetch_pc_out),
	.out(merge26lo_out));

branchresolve branchresolve (
	.rt(nop10_q),
	.rs(nop_q),
	.en(ctrl_branchresolve_en),
	.eqz(branchresolve_eqz),
	.gez(branchresolve_gez),
	.gtz(branchresolve_gtz),
	.lez(branchresolve_lez),
	.ltz(branchresolve_ltz),
	.ne(branchresolve_ne),
	.eq(branchresolve_eq));
	defparam
		branchresolve.WIDTH=32;

lo_reg lo_reg (
	.clk(clk),
	.resetn(resetn),
	.d(mux2to1_lo_reg_d_out),
	.squashn(ctrl_lo_reg_squashn),
	.en(ctrl_lo_reg_en),
	.q(lo_reg_q));
	defparam
		lo_reg.WIDTH=32;

hi_reg hi_reg (
	.clk(clk),
	.resetn(resetn),
	.d(mux2to1_hi_reg_d_out),
	.squashn(ctrl_hi_reg_squashn),
	.en(ctrl_hi_reg_en),
	.q(hi_reg_q));
	defparam
		hi_reg.WIDTH=32;

const const11 (
	.out(const11_out));
	defparam
		const11.WIDTH=32,
		const11.VAL=0;

const const12 (
	.out(const12_out));
	defparam
		const12.WIDTH=32,
		const12.VAL=16;

const const (
	.out(const_out));
	defparam
		const.WIDTH=32,
		const.VAL=31;

pipereg pipereg (
	.clk(clk),
	.resetn(ctrl_pipereg_resetn),
	.d(mux2to1_pipereg_d_out),
	.squashn(ctrl_pipereg_squashn),
	.en(ctrl_pipereg_en),
	.q(pipereg_q));
	defparam
		pipereg.WIDTH=32;

pipereg pipereg5 (
	.clk(clk),
	.resetn(ctrl_pipereg5_resetn),
	.d(zeroer4_q),
	.squashn(ctrl_pipereg5_squashn),
	.en(ctrl_pipereg5_en),
	.q(pipereg5_q));
	defparam
		pipereg5.WIDTH=5;

pipereg pipereg2 (
	.clk(clk),
	.resetn(ctrl_pipereg2_resetn),
	.d(ifetch_sa),
	.squashn(ctrl_pipereg2_squashn),
	.en(ctrl_pipereg2_en),
	.q(pipereg2_q));
	defparam
		pipereg2.WIDTH=5;

pipereg pipereg6 (
	.clk(clk),
	.resetn(ctrl_pipereg6_resetn),
	.d(mux2to1_pipereg6_d_out),
	.squashn(ctrl_pipereg6_squashn),
	.en(ctrl_pipereg6_en),
	.q(pipereg6_q));
	defparam
		pipereg6.WIDTH=32;

pipereg pipereg26 (
	.clk(clk),
	.resetn(ctrl_pipereg26_resetn),
	.d(nop13_q),
	.squashn(ctrl_pipereg26_squashn),
	.en(ctrl_pipereg26_en),
	.q(pipereg26_q));
	defparam
		pipereg26.WIDTH=32;

pipereg pipereg8 (
	.clk(clk),
	.resetn(ctrl_pipereg8_resetn),
	.d(nop_q),
	.squashn(ctrl_pipereg8_squashn),
	.en(ctrl_pipereg8_en),
	.q(pipereg8_q));
	defparam
		pipereg8.WIDTH=32;

pipereg pipereg7 (
	.clk(clk),
	.resetn(ctrl_pipereg7_resetn),
	.d(mux6to1_pipereg7_d_out),
	.squashn(ctrl_pipereg7_squashn),
	.en(ctrl_pipereg7_en),
	.q(pipereg7_q));
	defparam
		pipereg7.WIDTH=1;

fakedelay fakedelay (
	.clk(clk),
	.d(ifetch_pc_out),
	.q(fakedelay_q));
	defparam
		fakedelay.WIDTH=32;

pipereg pipereg27 (
	.clk(clk),
	.resetn(ctrl_pipereg27_resetn),
	.d(ifetch_instr),
	.squashn(ctrl_pipereg27_squashn),
	.en(ctrl_pipereg27_en),
	.q(pipereg27_q));
	defparam
		pipereg27.WIDTH=32;

pipereg pipereg28 (
	.clk(clk),
	.resetn(ctrl_pipereg28_resetn),
	.d(ifetch_epc),
	.squashn(ctrl_pipereg28_squashn),
	.en(ctrl_pipereg28_en),
	.q(pipereg28_q));
	defparam
		pipereg28.WIDTH=32;

pipereg pipereg29 (
	.clk(clk),
	.resetn(ctrl_pipereg29_resetn),
	.d(ifetch_rd),
	.squashn(ctrl_pipereg29_squashn),
	.en(ctrl_pipereg29_en),
	.q(pipereg29_q));
	defparam
		pipereg29.WIDTH=32;

pipereg pipereg30 (
	.clk(clk),
	.resetn(ctrl_pipereg30_resetn),
	.d(ifetch_ecause),
	.squashn(ctrl_pipereg30_squashn),
	.en(ctrl_pipereg30_en),
	.q(pipereg30_q));
	defparam
		pipereg30.WIDTH=32;

nop nop (
	.d(mux2to1_nop_d_out),
	.q(nop_q));
	defparam
		nop.WIDTH=32;

nop nop10 (
	.d(mux2to1_nop10_d_out),
	.q(nop10_q));
	defparam
		nop10.WIDTH=32;

nop nop13 (
	.d(mux9to1_nop13_d_out),
	.q(nop13_q));
	defparam
		nop13.WIDTH=32;

nop nop9 (
	.d(mux3to1_nop9_d_out),
	.q(nop9_q));
	defparam
		nop9.WIDTH=32;

zeroer zeroer (
	.d(ifetch_rs),
	.en(ctrl_zeroer_en),
	.q(zeroer_q));
	defparam
		zeroer.WIDTH=5;

zeroer zeroer0 (
	.d(ifetch_rt),
	.en(ctrl_zeroer0_en),
	.q(zeroer0_q));
	defparam
		zeroer0.WIDTH=5;

zeroer zeroer4 (
	.d(mux3to1_zeroer4_d_out),
	.en(ctrl_zeroer4_en),
	.q(zeroer4_q));
	defparam
		zeroer4.WIDTH=5;

cop2 cop2 (
	.clk(clk),
	.resetn(resetn),
	.fromcop2_wait(cop2_fromcop2_wait),
	.fromcop2_en(cop2_fromcop2_en),
	.fromcop2(cop2_fromcop2),
	.tocop2_wait(cop2_tocop2_wait),
	.tocop2_en(cop2_tocop2_en),
	.tocop2(cop2_tocop2),
	.fromcpu(nop10_q),
	.fromcpu_en(ctrl_cop2_fromcpu_en),
	.tocpu_en(ctrl_cop2_tocpu_en),
	.stalled(ctrl_cop2_stalled),
	.tocpu(cop2_tocpu));

cop0 cop0 (
	.clk(clk),
	.resetn(resetn),
	.badvaddr_we(cop0_badvaddr_we),
	.badvaddr_in(cop0_badvaddr_in),
	.ext_cause_in(cop0_ext_cause_in),
	.int_cause_in_stage2(data_mem_ecause),
	.int_cause_in_stage1(pipereg30_q),
	.epc_in(pipereg28_q),
	.fromcpu(nop10_q),
	.dest_addr(pipereg29_q),
	.read_addr(pipereg29_q),
	.instr(pipereg27_q),
	.fromcpu_en(ctrl_cop0_fromcpu_en),
	.tocpu_en(ctrl_cop0_tocpu_en),
	.stalled(ctrl_cop0_stalled),
	.status(cop0_status),
	.tocpu(cop0_tocpu),
	.exception(cop0_exception));

		// Multiplexor mux2to1_mul_opA instantiation
assign mux2to1_mul_opA_out = 
	(ctrl_mux2to1_mul_opA_sel==1) ? nop_q :
	nop9_q;

		// Multiplexor mux2to1_addersub_opA instantiation
assign mux2to1_addersub_opA_out = 
	(ctrl_mux2to1_addersub_opA_sel==1) ? fakedelay_q :
	nop_q;

		// Multiplexor mux3to1_mul_sa instantiation
assign mux3to1_mul_sa_out = 
	(ctrl_mux3to1_mul_sa_sel==2) ? nop_q :
	(ctrl_mux3to1_mul_sa_sel==1) ? const12_out :
	pipereg2_q;

		// Multiplexor mux2to1_hi_reg_d instantiation
assign mux2to1_hi_reg_d_out = 
	(ctrl_mux2to1_hi_reg_d_sel==1) ? div_remainder :
	mul_hi;

		// Multiplexor mux2to1_lo_reg_d instantiation
assign mux2to1_lo_reg_d_out = 
	(ctrl_mux2to1_lo_reg_d_sel==1) ? div_quotient :
	mul_lo;

		// Multiplexor mux9to1_nop13_d instantiation
assign mux9to1_nop13_d_out = 
	(ctrl_mux9to1_nop13_d_sel==8) ? cop2_tocpu :
	(ctrl_mux9to1_nop13_d_sel==7) ? cop0_tocpu :
	(ctrl_mux9to1_nop13_d_sel==6) ? addersub_result :
	(ctrl_mux9to1_nop13_d_sel==5) ? addersub_result_slt :
	(ctrl_mux9to1_nop13_d_sel==4) ? logic_unit_result :
	(ctrl_mux9to1_nop13_d_sel==3) ? mul_shift_result :
	(ctrl_mux9to1_nop13_d_sel==2) ? data_mem_d_loadresult :
	(ctrl_mux9to1_nop13_d_sel==1) ? hi_reg_q :
	lo_reg_q;

		// Multiplexor mux2to1_pipereg_d instantiation
assign mux2to1_pipereg_d_out = 
	(ctrl_mux2to1_pipereg_d_sel==1) ? ifetch_offset :
	signext16_out;

		// Multiplexor mux2to1_pipereg6_d instantiation
assign mux2to1_pipereg6_d_out = 
	(ctrl_mux2to1_pipereg6_d_sel==1) ? pcadder_result :
	merge26lo_out;

		// Multiplexor mux6to1_pipereg7_d instantiation
assign mux6to1_pipereg7_d_out = 
	(ctrl_mux6to1_pipereg7_d_sel==5) ? branchresolve_eq :
	(ctrl_mux6to1_pipereg7_d_sel==4) ? branchresolve_ne :
	(ctrl_mux6to1_pipereg7_d_sel==3) ? branchresolve_lez :
	(ctrl_mux6to1_pipereg7_d_sel==2) ? branchresolve_ltz :
	(ctrl_mux6to1_pipereg7_d_sel==1) ? branchresolve_gez :
	branchresolve_gtz;

		// Multiplexor mux3to1_nop9_d instantiation
assign mux3to1_nop9_d_out = 
	(ctrl_mux3to1_nop9_d_sel==2) ? pipereg_q :
	(ctrl_mux3to1_nop9_d_sel==1) ? nop10_q :
	const11_out;

		// Multiplexor mux3to1_zeroer4_d instantiation
assign mux3to1_zeroer4_d_out = 
	(ctrl_mux3to1_zeroer4_d_sel==2) ? ifetch_rt :
	(ctrl_mux3to1_zeroer4_d_sel==1) ? ifetch_rd :
	const_out;

pipereg pipereg15 (
	.clk(clk),
	.resetn(ctrl_pipereg15_resetn),
	.d(ifetch_func),
	.squashn(ctrl_pipereg15_squashn),
	.en(ctrl_pipereg15_en),
	.q(pipereg15_q));
	defparam
		pipereg15.WIDTH=6;

pipereg pipereg16 (
	.clk(clk),
	.resetn(ctrl_pipereg16_resetn),
	.d(ifetch_rt),
	.squashn(ctrl_pipereg16_squashn),
	.en(ctrl_pipereg16_en),
	.q(pipereg16_q));
	defparam
		pipereg16.WIDTH=5;

pipereg pipereg14 (
	.clk(clk),
	.resetn(ctrl_pipereg14_resetn),
	.d(ifetch_opcode),
	.squashn(ctrl_pipereg14_squashn),
	.en(ctrl_pipereg14_en),
	.q(pipereg14_q));
	defparam
		pipereg14.WIDTH=6;

branch_detector branch_detector (
	.func(ifetch_func),
	.opcode(ifetch_opcode),
	.is_branch(branch_detector_is_branch));

pipereg pipereg17 (
	.clk(clk),
	.resetn(ctrl_pipereg17_resetn),
	.d(ifetch_rs),
	.squashn(ctrl_pipereg17_squashn),
	.en(ctrl_pipereg17_en),
	.q(pipereg17_q));
	defparam
		pipereg17.WIDTH=5;

pipereg pipereg19 (
	.clk(clk),
	.resetn(ctrl_pipereg19_resetn),
	.d(pipereg15_q),
	.squashn(ctrl_pipereg19_squashn),
	.en(ctrl_pipereg19_en),
	.q(pipereg19_q));
	defparam
		pipereg19.WIDTH=6;

pipereg pipereg18 (
	.clk(clk),
	.resetn(ctrl_pipereg18_resetn),
	.d(pipereg14_q),
	.squashn(ctrl_pipereg18_squashn),
	.en(ctrl_pipereg18_en),
	.q(pipereg18_q));
	defparam
		pipereg18.WIDTH=6;

pipereg pipereg20 (
	.clk(clk),
	.resetn(ctrl_pipereg20_resetn),
	.d(pipereg16_q),
	.squashn(ctrl_pipereg20_squashn),
	.en(ctrl_pipereg20_en),
	.q(pipereg20_q));
	defparam
		pipereg20.WIDTH=5;

pipereg pipereg21 (
	.clk(clk),
	.resetn(ctrl_pipereg21_resetn),
	.d(pipereg17_q),
	.squashn(ctrl_pipereg21_squashn),
	.en(ctrl_pipereg21_en),
	.q(pipereg21_q));
	defparam
		pipereg21.WIDTH=5;

pipereg pipereg22 (
	.clk(clk),
	.resetn(resetn),
	.d(branch_detector_is_branch),
	.squashn(~branch_mispred),
	.en(~stall_out_stage1),
	.q(pipereg22_q));
	defparam
		pipereg22.WIDTH=1;

pipereg pipereg23 (
	.clk(clk),
	.resetn(ctrl_pipereg23_resetn),
	.d(predictme),
	.squashn(ctrl_pipereg23_squashn),
	.en(ctrl_pipereg23_en),
	.q(pipereg23_q));
	defparam
		pipereg23.WIDTH=1;

		// Multiplexor mux2to1_nop_d instantiation
assign mux2to1_nop_d_out = 
	(pipereg31_q==1) ? pipereg26_q :
	reg_file_a_readdataout;

pipereg pipereg31 (
	.clk(clk),
	.resetn(ctrl_pipereg31_resetn),
	.d(haz_zeroer_q_pipereg5_q),
	.squashn(ctrl_pipereg31_squashn),
	.en(ctrl_pipereg31_en),
	.q(pipereg31_q));
	defparam
		pipereg31.WIDTH=1;

		// Multiplexor mux2to1_nop10_d instantiation
assign mux2to1_nop10_d_out = 
	(pipereg32_q==1) ? pipereg26_q :
	reg_file_b_readdataout;

pipereg pipereg32 (
	.clk(clk),
	.resetn(ctrl_pipereg32_resetn),
	.d(haz_zeroer0_q_pipereg5_q),
	.squashn(ctrl_pipereg32_squashn),
	.en(ctrl_pipereg32_en),
	.q(pipereg32_q));
	defparam
		pipereg32.WIDTH=1;

pipereg pipereg25 (
	.clk(clk),
	.resetn(ctrl_pipereg25_resetn),
	.d(pipereg23_q),
	.squashn(ctrl_pipereg25_squashn),
	.en(ctrl_pipereg25_en),
	.q(pipereg25_q));
	defparam
		pipereg25.WIDTH=1;

// PETES CHANGE add trace signals
assign trc_data=nop13_q;
assign trc_we=ctrl_reg_file_c_we;
assign trc_addr=pipereg5_q;
assign trc_pipestall=stall_out_stage2;

endmodule
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vpu.v
//////////////////////////////////////////////////////////////////////
/******************************************************************************
  Vector Control Pipeline

          Stage 1       Stage 2         Stage 3
  ----|--------------|-----------------|-----------------|
      | Decode       | RF/EX           |WB & Send to cpu

 * Note, vs register file is written back to in Stage 3 also, but we check 
 * the vector lanes first to see if they're writing to the vs RF.  If so then
 * we stall.

******************************************************************************/

//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: options.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/top/options.v
//////////////////////////////////////////////////////////////////////
`ifndef _OPTIONS_V_
`define _OPTIONS_V_ 1

`define NO_PLI 1
//`define TEST_BENCH 1
//`define USE_INHOUSE_LOGIC
//`define SIMULATION_MEMORY
// Replaces altera blocks with local logic files

/************************** ABBREVIEATED NAMES *****************************/
// Note: LG = Log base 2
//
// Default configuration:
//    8KB Icache (LGID=13) with 16 byte cache line size (LGIW=4)
//   32KB Dcache (LGDD=15) with 64 byte cache line size (LGDD=6)
//    Data prefetching off (DP=0, DPV=0)
//    16 Vector Lanes (LGL=4)
//    64 Maximum Vector Length (LGMVL=6)
//    32-bit (4-byte) Vector lane width (LGVPW=2)
//    32-bit Vector lane datapath width (LGLW=5)
//    16 Memory Crossbar lanes (LGM=LGL)
//    16 Multiplier lanes (LGX=LGL)
//    2 Register Banks (LGB=1)
//    Disable ALU per Bank (APB=0)


// INSTR CACHE
`define LGID 13
`define LGIW 4

// DATA CACHE
`define LGDD 15
`define LGDW 4

// DATA CACHE PREFETCHER - dcache needs to have 64 byte lines
`define LGDWB 7
`define DP 0
// VECTOR DATA CACHE PREFETCHER 0:off, 65535-N:N*veclength, N:pfch N cache lines
`define DPV 0

// VECTOR CORE
//Changing to 3. That is, we now have 8 lanes.
`define LGL 3
`define LGB 0
`define APB 0
`define LGM `LGL
`define LGX `LGL

// VECTOR ISA
`define LGMVL 6
`define LGVPW 1 //chaging the word size of vector processor to 16: support for bfloat16
`define LGLW 4

/****************************** FULL NAMES *********************************/

// INSTR CACHE
`define LOG2ICACHEDEPTHBYTES `LGID
`define LOG2ICACHEWIDTHBITS (`LGIW+3)

// DATA CACHE
`define LOG2DCACHEDEPTHBYTES `LGDD
`define LOG2DCACHEWIDTHBITS (`LGDW+3)

// DATA CACHE PREFETCHER - dcache needs to have 64 byte lines
`define LOG2DATAWBBUFFERSIZE `LGDWB
`define DEFAULTDCACHEPREFETCHES `DP
// VECTOR DATA CACHE PREFETCHER 0:off, 65535:vectorlength, N:pfch N cache lines
`define VECTORPREFETCHES `DPV

// VECTOR CORE
`define LOG2NUMLANES `LGL
`define LOG2MVL `LGMVL
`define LOG2VPW `LGVPW
`define LOG2LANEWIDTHBITS `LGLW
`define LOG2NUMMEMLANES `LGM
`define LOG2NUMMULLANES `LGX
`define LOG2NUMBANKS `LGB
`define ALUPERBANK `APB

`define MAX_STAGES 14
/****************************** OTHER PARAMS *********************************/

// DRAM
`define LOG2DRAMWIDTHBITS 7

/****************** NUM PIPELINE STAGES in VECTOR PROCESSOR ***************/
//mult consumes 3 cycles
//`define MAX_PIPE_STAGES 7
//matmul consumes 29 cycles
`define MAX_PIPE_STAGES 8
`define MATMUL_STAGES 29

/****************** SIZE OF THE MATMUL UNIT ***************/
`define MAT_MUL_SIZE 8

`endif

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: options.v
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vregfile_base.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vregfile_base.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Base Register File

   - Has one read port (a) and one write port (c)
****************************************************************************/
module vregfile_base (
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
parameter NUMREGS=16;
parameter LOG2NUMREGS=4;

input clk;
input resetn;

input [LOG2NUMREGS-1:0] a_reg,c_reg;
output [WIDTH-1:0] a_readdataout;
input [WIDTH-1:0] c_writedatain;
input a_en, c_we;

`ifdef USE_INHOUSE_LOGIC
        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(a_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout)
        );
        defparam
            reg_file1.AWIDTH=LOG2NUMREGS,
            reg_file1.NUM_WORDS=NUMREGS,
            reg_file1.DWIDTH=WIDTH;

`else
 //giving error in vtr even with yosys flow
  /*
	altsyncram	reg_file1(
				.wren_a (c_we),
				.clock0 (clk),
				.data_a (c_writedatain),
				.address_a (c_reg[LOG2NUMREGS-1:0]),
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
 */
`endif
 
endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vregfile_base.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vregfile_control.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vregfile_control.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Control Register File

   - Has one read port (a) and one write port (c)
   - vl promoted as first-class entities
****************************************************************************/
module vregfile_control (
    clk,
    resetn, 

    a_reg, 
    a_en,
    a_readdataout,

    c_reg, 
    c_writedatain, 
    c_we,

    vl,
    matmul_masks,
    dma_en,
    lane_addr,
    mem_addr,
    num_bytes,
    dma_we,
    dma_busy,
    temp
    );

parameter WIDTH=32;
parameter NUMREGS=32;
parameter LOG2NUMREGS=5;

input clk;
input resetn;

input a_en;
input [LOG2NUMREGS-1:0] a_reg,c_reg;
output reg [WIDTH-1:0] a_readdataout;
input [WIDTH-1:0] c_writedatain;
input c_we;

output reg [WIDTH-1:0] vl;
output reg dma_en;
output reg [WIDTH-1:0] lane_addr;
output reg [WIDTH-1:0] mem_addr;
output reg [WIDTH-1:0] num_bytes;
output reg [WIDTH-1:0] temp;
output reg dma_we;
output reg [3*`MAT_MUL_SIZE-1:0] matmul_masks;
input dma_busy;

wire [WIDTH-1:0] _a_readdataout;

reg mux_sel,next_mux_sel;

always@(posedge clk)begin
  if(!resetn)
    mux_sel <= 1'b0;
  else begin
    if(a_en)
      mux_sel <= next_mux_sel;
  end       
end


always @* begin
  num_bytes[1:0] = 2'h0;
  next_mux_sel = 1'b0;
  if(dma_busy)begin
    if(a_reg == 27)
      next_mux_sel = 1'b1;
    else
      next_mux_sel = 1'b0;
  end
  
  if(mux_sel)
    a_readdataout = dma_busy;
  else
    a_readdataout = _a_readdataout;
end

`ifdef USE_INHOUSE_LOGIC
        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(a_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(_a_readdataout)
      );
      defparam
            reg_file1.AWIDTH=LOG2NUMREGS,
            reg_file1.NUM_WORDS=NUMREGS,
            reg_file1.DWIDTH=WIDTH;
    `ifdef TEST_BENCH
                initial begin
		    $readmemh("vregfile_control.dat",reg_file1.dpram1.ram,'h0);
                end
     `endif 
 `else
  //amana: Somehow this was causing an error with VTR.
  //Since we don't need this anyway, commenting it out.
	//altsyncram	reg_file1(
	//			.wren_a (c_we),
	//			.clock0 (clk),
	//			.address_a (c_reg[LOG2NUMREGS-1:0]),
	//			.data_a (c_writedatain),
  //       .rden_b(a_en),
	//			.address_b (a_reg[LOG2NUMREGS-1:0]),
	//			.q_b (a_readdataout)
  //       // synopsys translate_off
  //       ,
  //       .aclr0 (1'b0),
  //       .aclr1 (1'b0),
  //       .byteena_a (1'b1),
  //       .byteena_b (1'b1),
  //       .data_b (32'b11111111),
  //       .wren_b (1'b0),
  //       .clock1 (1'b0),
  //       .clocken1 (1'b0),
  //       .q_a (),
  //       .clocken0 (1'b1),
  //       .addressstall_a (1'b0),
  //       .addressstall_b (1'b0)
  //       // synopsys translate_on
  //   );
	//defparam
  //   `ifdef TEST_BENCH
	//	reg_file1.init_file = "vregfile_control.dat",
  //   `else
	//	reg_file1.init_file = "vregfile_control.mif",
  //   `endif
	//	reg_file1.operation_mode = "DUAL_PORT",
	//	reg_file1.width_a = WIDTH,
	//	reg_file1.widthad_a = LOG2NUMREGS,
	//	reg_file1.numwords_a = NUMREGS,
	//	reg_file1.width_b = WIDTH,
	//	reg_file1.widthad_b = LOG2NUMREGS,
	//	reg_file1.numwords_b = NUMREGS,
	//	reg_file1.lpm_type = "altsyncram",
	//	reg_file1.width_byteena_a = 1,
	//	reg_file1.outdata_reg_b = "UNREGISTERED",
	//	reg_file1.indata_aclr_a = "NONE",
	//	reg_file1.wrcontrol_aclr_a = "NONE",
	//	reg_file1.address_aclr_a = "NONE",
	//	reg_file1.rdcontrol_reg_b = "CLOCK0",
	//	reg_file1.address_reg_b = "CLOCK0",
	//	reg_file1.address_aclr_b = "NONE",
	//	reg_file1.outdata_aclr_b = "NONE",
	//	reg_file1.read_during_write_mode_mixed_ports = "OLD_DATA",
	//	reg_file1.ram_block_type = "AUTO",
	//	reg_file1.intended_device_family = "Stratix";
  `endif

  always@(posedge clk) begin
    if (!resetn) begin
      vl<=0;
      matmul_masks<=32'hffffffff;
      dma_en <= 0;
      mem_addr <= 0;
      num_bytes[WIDTH-1:2] <= 0;
      lane_addr <= 0;
      dma_we <= 0;
      temp <= 0;
    end
    else begin
      if (c_we) begin
        if (c_reg==0) begin
          vl<=c_writedatain;
        end 
        else if (c_reg==31) begin //a_rows
          matmul_masks[1*`MAT_MUL_SIZE-1:0*`MAT_MUL_SIZE] <= c_writedatain[`MAT_MUL_SIZE-1:0];
          matmul_masks[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE] <= c_writedatain[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE];
          matmul_masks[3*`MAT_MUL_SIZE-1:2*`MAT_MUL_SIZE] <= c_writedatain[3*`MAT_MUL_SIZE-1:2*`MAT_MUL_SIZE];
        end
        else if (c_reg==30) begin 
          dma_en <= c_writedatain[0];
          dma_we <= c_writedatain[1];
          num_bytes[WIDTH-1:2] <= c_writedatain[WIDTH-1:2];
        end
        else if (c_reg==29) begin // Address to the main memory
          mem_addr<=c_writedatain;
        end
        else if (c_reg==28) begin
          lane_addr<=c_writedatain;
        end 
        else if (c_reg==20) begin
          temp<=c_writedatain;
        end 
      end  
    end
  end

endmodule


//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vregfile_control.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vregfile_flag.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vregfile_flag.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Register File

   - Has two read ports (a and b) and one write port (c)
   - sel chooses the register to be read/written
****************************************************************************/
module vregfile_flag
  (clk,resetn, 
	a_reg, a_readdataout, a_en,
	b_reg, b_readdataout, b_en,
	c_reg, c_writedatain, c_we);

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
input [NUMBANKS-1:0] c_we;

input [NUMBANKS*LOG2NUMREGSPERBANK-1:0] a_reg,b_reg,c_reg;
output [NUMBANKS*WIDTH-1:0] a_readdataout, b_readdataout;
input [NUMBANKS*WIDTH-1:0] c_writedatain;

  genvar k;

  generate
  for (k=0; k<NUMBANKS; k=k+1)
  begin : bank_gen
`ifdef  USE_INHOUSE_LOGIC

        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en[k]),
	    .address_a(c_reg[k* LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
	    .address_b(a_reg[k* LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
	    .wren_a(c_we[k]),
	    .wren_b(1'b0),
	    .data_a(c_writedatain[ k*WIDTH +: WIDTH]),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout[k*WIDTH +: WIDTH])
        );
        defparam
            reg_file1.AWIDTH=LOG2NUMREGSPERBANK,
            reg_file1.NUM_WORDS=NUMREGSPERBANK,
            reg_file1.DWIDTH=WIDTH;

        ram_wrapper reg_file2(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(b_en[k]),
	    .address_a(c_reg[k* LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
	    .address_b(b_reg[k* LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
	    .wren_a(c_we[k]),
	    .wren_b(1'b0),
	    .data_a(c_writedatain[ k*WIDTH +: WIDTH]),
	    .data_b(0),
	    .out_a(),
	    .out_b(b_readdataout[k*WIDTH +: WIDTH])
        );
        defparam
            reg_file2.AWIDTH=LOG2NUMREGSPERBANK,
            reg_file2.NUM_WORDS=NUMREGSPERBANK,
            reg_file2.DWIDTH=WIDTH;

`else
    /*
    altsyncram	reg_file1(
          .clock0 (clk),
          .wren_a (c_we[k]),
          .data_a (c_writedatain[k*WIDTH +: WIDTH]),
          .byteena_a (1'b1),
          .address_a (c_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .rden_b(a_en[k]),
          .address_b (a_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .q_b (a_readdataout[k*WIDTH +: WIDTH])
          // synopsys translate_off
          ,
          .aclr0 (1'b0),
          .aclr1 (1'b0),
          .byteena_b (1'b1),
          .data_b (),
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
      reg_file1.widthad_a = LOG2NUMREGSPERBANK,
      reg_file1.numwords_a = NUMREGSPERBANK,
      reg_file1.width_b = WIDTH,
      reg_file1.widthad_b = LOG2NUMREGSPERBANK,
      reg_file1.numwords_b = NUMREGSPERBANK,
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

      //Reg file duplicated to avoid contention between 2 read
      //and 1 write
    altsyncram	reg_file2(
          .clock0 (clk),
          .wren_a (c_we[k]),
          .data_a (c_writedatain[k*WIDTH +: WIDTH]),
          .byteena_a (1'b1),
          .address_a (c_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .rden_b(b_en[k]),
          .address_b (b_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
          .q_b (b_readdataout[k*WIDTH +: WIDTH])
          // synopsys translate_off
          ,
          .aclr0 (1'b0),
          .aclr1 (1'b0),
          .byteena_b (1'b1),
          .data_b (),
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
      reg_file2.operation_mode = "DUAL_PORT",
      reg_file2.width_a = WIDTH,
      reg_file2.widthad_a = LOG2NUMREGSPERBANK,
      reg_file2.numwords_a = NUMREGSPERBANK,
      reg_file2.width_b = WIDTH,
      reg_file2.widthad_b = LOG2NUMREGSPERBANK,
      reg_file2.numwords_b = NUMREGSPERBANK,
      reg_file2.lpm_type = "altsyncram",
      reg_file2.width_byteena_a = 1,
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
  */
  `endif
  
  end
  endgenerate

endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vregfile_flag.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vregfile_inc.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vregfile_inc.v
//////////////////////////////////////////////////////////////////////
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

`ifdef USE_INHOUSE_LOGIC
        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(a_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we&(|c_reg)),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout)
        );
        defparam
            reg_file1.AWIDTH=LOG2NUMREGS,
            reg_file1.NUM_WORDS=NUMREGS,
            reg_file1.DWIDTH=WIDTH;

`else
 /*
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
  */
`endif
endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vregfile_inc.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vregfile_scalar.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vregfile_scalar.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Scalar Register File

   - Has one read port (a) and one write port (c)
   - vs0 fixed to 0 and unwriteable
****************************************************************************/
module vregfile_scalar (
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
parameter NUMREGS=32;
parameter LOG2NUMREGS=5;

input clk;
input resetn;

input a_en;
input [LOG2NUMREGS-1:0] a_reg,c_reg;
output [WIDTH-1:0] a_readdataout;
input [WIDTH-1:0] c_writedatain;
input c_we;

`ifdef USE_INHOUSE_LOGIC
        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(a_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we & (|c_reg)),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout)
        );
        defparam
            reg_file1.AWIDTH=LOG2NUMREGS,
            reg_file1.NUM_WORDS=NUMREGS,
            reg_file1.DWIDTH=WIDTH;
 `else
  /*
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
  */
 `endif
endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vregfile_scalar.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vregfile_stride.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vregfile_stride.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Stride Register File

   - Has one read port (a) and one write port (c)
****************************************************************************/
module vregfile_stride (
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

`ifdef USE_INHOUSE_LOGIC
        ram_wrapper reg_file1(
	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
	    .address_a(c_reg[LOG2NUMREGS-1:0]),
	    .address_b(a_reg[LOG2NUMREGS-1:0]),
	    .wren_a(c_we),
	    .wren_b(1'b0),
	    .data_a(c_writedatain),
	    .data_b(0),
	    .out_a(),
	    .out_b(a_readdataout)
        );
        defparam
            reg_file1.AWIDTH=LOG2NUMREGS,
            reg_file1.NUM_WORDS=NUMREGS,
            reg_file1.DWIDTH=WIDTH;
 `else
 /*
	altsyncram	reg_file1(
				.wren_a (c_we),
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
  */
`endif
endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vregfile_stride.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vregfile_vector.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vregfile_vector.v
//////////////////////////////////////////////////////////////////////
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
  `ifdef USE_INHOUSE_LOGIC
          ram_wrapper reg_file1(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(a_en),
  	    .address_a(c_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
  	    .address_b(a_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
  	    .wren_a(c_we[k] & ((WIDTH>8) ? 1'b1 : c_byteen[k])),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[k*WIDTH +: WIDTH]),
  	    .data_b(0),
  	    .out_a(),
  	    .out_b(a_readdataout[k*WIDTH +: WIDTH])
          );
          defparam
              reg_file1.AWIDTH=LOG2NUMREGSPERBANK,
              reg_file1.NUM_WORDS=NUMREGSPERBANK,
              reg_file1.DWIDTH=WIDTH;
  
          ram_wrapper reg_file2(
  	    .clk(clk),
            .resetn(resetn),
            .rden_a(1'b0),
            .rden_b(b_en),
  	    .address_a(c_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
  	    .address_b(b_reg[k*LOG2NUMREGSPERBANK +: LOG2NUMREGSPERBANK]),
  	    .wren_a(c_we[k] & ((WIDTH>8) ? 1'b1 : c_byteen[k])),
  	    .wren_b(1'b0),
  	    .data_a(c_writedatain[k*WIDTH +: WIDTH]),
  	    .data_b(0),
  	    .out_a(),
  	    .out_b(b_readdataout[k*WIDTH +: WIDTH])
          );
          defparam
              reg_file2.AWIDTH=LOG2NUMREGSPERBANK,
              reg_file2.NUM_WORDS=NUMREGSPERBANK,
              reg_file2.DWIDTH=WIDTH;
  `else
  /*
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
    */
  `endif
  end
  endgenerate

endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vregfile_vector.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vcomponents.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vcomponents.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Pipeline register - for transmitting a signal down several stages

  DEPTH - number of actual pipeline registers needed
****************************************************************************/
module pipe(
    d,
    clk,
    resetn,
    en,
    squash,
    q
    );
parameter WIDTH=32;
parameter DEPTH=1;
parameter RESETVALUE=0;

input [WIDTH-1:0]  d;
input              clk;
input              resetn;
input  [DEPTH-1:0] en;
input  [DEPTH-1:0] squash;
output [WIDTH*(DEPTH+1)-1:0] q;

reg [WIDTH*DEPTH-1:0] tq;
integer i;

  always@(posedge clk)
  begin
    // 1st register
    if (!resetn || squash[0] )
      tq[ WIDTH-1:0 ]<=RESETVALUE;
    else if (en[0])
      tq[ WIDTH-1:0 ]<=d;

    // All the rest registers
    for (i=1; i<DEPTH; i=i+1)
      if (!resetn || squash[i] )
        tq[i*WIDTH +: WIDTH ]<=RESETVALUE;
      else if (en[i])
        tq[i*WIDTH +: WIDTH ]<=tq[(i-1)*WIDTH +: WIDTH ];
  end

  assign q[WIDTH-1:0]=d;
  assign q[WIDTH*(DEPTH+1)-1:WIDTH]=tq;
endmodule

/****************************************************************************
          Hazard checker
 if mode==0 compare entire width of src & dests,
 if mode==1 compare upper WIDTH-SUBWIDTH bits of src & dests
****************************************************************************/
module hazardchecker (
    src,
    src_valid,
    dst,
    dst_valid,
    dst_mode,
    lt_dst,
    lt_dst_valid,
    lt_mode,
    haz
    );
parameter WIDTH=10;
parameter SUBWIDTH=5;
parameter DEPTH=1;
parameter LTDEPTH=1;

input [WIDTH-1:0]  src;
input              src_valid;
input [WIDTH*DEPTH-1:0]  dst;
input [DEPTH-1:0]    dst_valid;
input [DEPTH-1:0]  dst_mode;
input [WIDTH*LTDEPTH-1:0]  lt_dst;
input [LTDEPTH-1:0]    lt_dst_valid;
input [LTDEPTH-1:0]  lt_mode;
output             haz;

reg             t_haz;
reg             t_haz_lt;

integer i,j;

  always@*
  begin
    t_haz=1'b0;
    t_haz_lt=1'b0;

    for (i=0; i<DEPTH; i=i+1)
      if (dst_mode)
        t_haz=t_haz | ((src[WIDTH-1:SUBWIDTH]==dst[i*WIDTH+SUBWIDTH +: WIDTH-SUBWIDTH ])&dst_valid[i]&src_valid);
      else
        t_haz=t_haz | ((src==dst[i*WIDTH +: WIDTH ])&dst_valid[i]&src_valid);

    //Check if src is less than dest - used for hazards in issuer
    for (j=0; j<LTDEPTH; j=j+1)
      if (lt_mode)
        t_haz_lt=t_haz_lt | ((src[WIDTH-1:SUBWIDTH]==lt_dst[j*WIDTH+SUBWIDTH +: WIDTH-SUBWIDTH ])&lt_dst_valid[j]&src_valid);
      else
        t_haz_lt=t_haz_lt | (((src[WIDTH-1:SUBWIDTH]==lt_dst[j*WIDTH+SUBWIDTH +: WIDTH-SUBWIDTH ])&lt_dst_valid[j]&src_valid) && (src[SUBWIDTH-1:0]>=lt_dst[j*WIDTH +: SUBWIDTH]));
  end

  assign haz=(t_haz|t_haz_lt)&src_valid;

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vcomponents.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vlanes.v.temp
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/vtr/vlanes.v.temp
//////////////////////////////////////////////////////////////////////

/******************************************************************************
 * Vector Pipeline
 *
 *                vl (arrives from vector control pipeline in Stage2)
 *                vs
 *                vc
 *|   Stage1  |  Stage2   |  Stage3  |  Stage4  |  Stage5  | Stage6  |  Stage7 |
 *| instr     |           |          | dispatch |          |         |
 *| decode    | control   | bank     | regfile  | execute  | WB
 *
 *****************************************************************************/

//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vdispatcher.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vdispatcher.v
//////////////////////////////////////////////////////////////////////


module vdispatcher(
    clk,
    resetn,

    shift,
    rotate, //shift must also be high to rotate

    inshift_instr,
    inshift_first,
    inshift_rdelm,
    inshift_wrelm,
    inshift_count,

    increment,
    rdelm_add_sub,
    wrelm_add_sub,
    count_add_sub,
    rdelm_valuetoadd,
    wrelm_valuetoadd,
    count_valuetoadd,

    instr,
    first,
    rdelm,
    wrelm,
    count

    );

parameter NUMENTRIES=4;      
parameter WIDTHINSTR=32;
parameter WIDTHRDELM=32;
parameter WIDTHWRELM=32;
parameter WIDTHCOUNT=32;

input clk;
input resetn;

input  shift;
input  rotate;

input [ WIDTHINSTR-1:0 ] inshift_instr;
input                    inshift_first;
input [ WIDTHRDELM-1:0 ] inshift_rdelm;
input [ WIDTHRDELM-1:0 ] inshift_wrelm;
input [ WIDTHCOUNT-1:0 ] inshift_count;

input [ NUMENTRIES-1:0 ] increment;

input rdelm_add_sub;
input wrelm_add_sub;
input count_add_sub;
input [ WIDTHRDELM-1:0 ] rdelm_valuetoadd;
input [ WIDTHWRELM-1:0 ] wrelm_valuetoadd;
input [ WIDTHCOUNT-1:0 ] count_valuetoadd;

output [ NUMENTRIES*WIDTHINSTR-1:0 ] instr;
output            [ NUMENTRIES-1:0 ] first;
output [ NUMENTRIES*WIDTHRDELM-1:0 ] rdelm;
output [ NUMENTRIES*WIDTHWRELM-1:0 ] wrelm;
output [ NUMENTRIES*WIDTHCOUNT-1:0 ] count;

wire [NUMENTRIES*WIDTHINSTR-1:0 ] inparallel_data_inst_NC;
vdispatcher_shift #(NUMENTRIES,WIDTHINSTR) vdispatcher_instr (
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .squash({NUMENTRIES{1'b0}}),
      .inshift_data(inshift_instr),
      .inparallel_data(inparallel_data_inst_NC),
      .outparallel_data(instr));

wire [ NUMENTRIES-1:0 ] inparallel_data_first_NC;
vdispatcher_shift #(NUMENTRIES,1) vdispatcher_first (
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .squash(increment&~{32'b0,shift&~rotate}),
      .inshift_data(inshift_first),
      .inparallel_data(inparallel_data_first_NC),
      .outparallel_data(first));

wire [ NUMENTRIES*WIDTHRDELM-1:0 ] inparallel_data_rdelm_NC;
wire [ NUMENTRIES-1:0 ] squash_rdelm_NC;
vdispatcher_add #(NUMENTRIES,WIDTHRDELM) vdispatcher_rdelm(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_rdelm_NC),
      .add_sub(rdelm_add_sub),
      .valuetoadd(rdelm_valuetoadd),
      .inshift_data(inshift_rdelm),
      .inparallel_data(inparallel_data_rdelm_NC),
      .outparallel_data(rdelm));

wire [ NUMENTRIES*WIDTHWRELM-1:0 ] inparallel_data_wrelm_NC;
wire [ NUMENTRIES-1:0 ] squash_wrelm_NC;
vdispatcher_add #(NUMENTRIES,WIDTHWRELM) vdispatcher_wrelm(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_wrelm_NC),
      .add_sub(wrelm_add_sub),
      .valuetoadd(wrelm_valuetoadd),
      .inshift_data(inshift_wrelm),
      .inparallel_data(inparallel_data_wrelm_NC),
      .outparallel_data(wrelm));

wire [ NUMENTRIES*WIDTHCOUNT-1:0 ] inparallel_data_count_NC;
wire [ NUMENTRIES-1:0 ] squash_count_NC;
vdispatcher_add #(NUMENTRIES,WIDTHCOUNT) vdispatcher_count(
      .clk(clk),
      .resetn(resetn),      
      .load(1'b0),
      .shift(shift),
      .rotate(rotate),
      .increment(increment),
      .squash(squash_count_NC),
      .add_sub(count_add_sub),
      .valuetoadd(count_valuetoadd),
      .inshift_data(inshift_count),
      .inparallel_data(inparallel_data_count_NC),
      .outparallel_data(count));


endmodule



module vdispatcher_shift (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    squash,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

parameter NUMENTRIES=4;      
parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ NUMENTRIES-1:0 ]  squash;

input [ WIDTH-1:0 ] inshift_data;

input [ NUMENTRIES*WIDTH-1:0 ]  inparallel_data;
output [ NUMENTRIES*WIDTH-1:0 ] outparallel_data;

wire [ WIDTH-1:0 ]  shiftin_left;
wire [ WIDTH-1:0 ]  shiftin_right;


  assign shiftin_right = (!rotate) ? inshift_data : 
                      outparallel_data[NUMENTRIES*WIDTH-1:(NUMENTRIES-1)*WIDTH];

  assign shiftin_left = (!rotate) ? 0 : outparallel_data[WIDTH-1:0];

  velmshifter #(NUMENTRIES,WIDTH) velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load),
      .shift(shift),
      .dir_left(1),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe(inparallel_data),
      .outpipe(outparallel_data));


endmodule



module vdispatcher_add (
    clk,
    resetn,

    load,
    shift,
    rotate,       // Sets whether ends feedback into beginning
    increment,
    squash,

    add_sub,
    valuetoadd,

    inshift_data,

    inparallel_data,
    outparallel_data

    );

parameter NUMENTRIES=4;      
parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  rotate;
input [ NUMENTRIES-1:0 ]  increment;
input [ NUMENTRIES-1:0 ]  squash;

input               add_sub;
input [ WIDTH-1:0 ] valuetoadd;

input [ WIDTH-1:0 ] inshift_data;

input [ NUMENTRIES*WIDTH-1:0 ]  inparallel_data;
output [ NUMENTRIES*WIDTH-1:0 ] outparallel_data;

wire [ WIDTH-1:0 ]  shiftin_right;

reg [ NUMENTRIES*WIDTH-1:0 ] outparallel_data;
reg [ NUMENTRIES*WIDTH-1:0 ] outparallel_added;


  assign shiftin_right = (!rotate) ? inshift_data : 
                    outparallel_added[NUMENTRIES*WIDTH-1:(NUMENTRIES-1)*WIDTH];

  reg [ WIDTH-1:0 ] v;
  integer i;
  always@*
    for (i=0; i<NUMENTRIES; i=i+1)
    begin
      //Saturate at 0xffff... in either direction
      v=(increment[i] && outparallel_data[i*WIDTH +: WIDTH] != {WIDTH{1'b1}}) ?
        valuetoadd : 0;
      if (add_sub==0)
        outparallel_added[i*WIDTH +: WIDTH]=outparallel_data[i*WIDTH+:WIDTH] + v;
      else
        outparallel_added[i*WIDTH +: WIDTH]=outparallel_data[i*WIDTH+:WIDTH] - v;
    end

  always@(posedge clk)
    if (!resetn)
      outparallel_data=0;
    else if (load)
      outparallel_data=inparallel_data;
    else if (shift)
      outparallel_data=(outparallel_added<<WIDTH) | shiftin_right;
      


endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vdispatcher.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vlane_alu.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vlane_alu.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************

  ALU Circuit:

         |    _     flags
 src1 ---0---| \    |
             >+|----+----|Sat|------|M\
 src1 --|M\--|-/         |sum|      |U|---
        |U|                       +-|X/
 src2 --|X/                       |    1
                                  |
      src2 ------------|M\        |
 src1 ----------|&|    |U|--|Sat|-+
                |||----|X/  |siz|
 src2 ---0------|^|      0
         |


              ADD SUB   CMP    CMP.U & | ^ ~| MIN MAX ABS MRG
              S U S U = ! < <= < <=           S U S U

adderopA_sel  0 0 0 0 0 0 0 0  0 0   x x x x  0 0 0 0  1   1
adderopB_sel  0 0 0 0 0 0 0 0  0 0   x x x x  0 0 0 0  1   0
logicopB_sel  x x x x x x x x  x x   0 0 0 0  1 1 1 1  1   1
signed        1 0 1 0 1 1 1 1  0 0   x x x x  1 0 1 0  1   x
addsub        0 0 1 1 1 1 1 1  1 1   x x x x  1 1 1 1  1   0
max           x x x x x x x x  x x   0 0 0 0  0 0 1 1  0   0
min           x x x x x x x x  x x   0 0 0 0  1 1 0 0  0   0
logic[1:0]    x x x x x x x x  x x   1 0 2 3  0 0 0 0  0   0
mux1_sel[1:0] 0 0 0 0 x x x x  x x   1 1 1 1  1 1 1 1  2   3 -- Combine
flag_sel[1:0] x x x x 0 1 2 3  2 3   x x x x  x x x x  x   x /

ALUOP_ZERO    =11'b00100000101
ALUOP_ADD     =11'b00010000000
ALUOP_ADDU    =11'b00000000000
ALUOP_SUB     =11'b00011000000
ALUOP_SUBU    =11'b00001000000
ALUOP_CMP_EQ  =11'b00011000000
ALUOP_CMP_NEQ =11'b00011000001
ALUOP_CMP_LT  =11'b00011000010
ALUOP_CMP_LE  =11'b00011000011
ALUOP_CMP_LT_U=11'b00001000010
ALUOP_CMP_LE_U=11'b00001000011
ALUOP_AND     =11'b00000000101
ALUOP_OR      =11'b00000000001
ALUOP_XOR     =11'b00000001001
ALUOP_NOR     =11'b00000001101
ALUOP_MIN     =11'b00111010001
ALUOP_MIN_U   =11'b00101010001
ALUOP_MAX     =11'b00111100001
ALUOP_MAX_U   =11'b00101100001
ALUOP_ABS     =11'b11111000010
ALUOP_MERGE   =11'b10100000011

****************************************************************************/

//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vlane_saturate.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vlane_saturate.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Saturate unit

  Takes a signed input and saturates it

  sat sign
   0    x   NOP (pass through)
   1    1   VS[ADD/SUB]
   1    0   VS[ADD/SUB]_U

parameter
  SATSUMOP_NOP=2'b00,
  SATSUMOP_VS =2'b11,
  SATSUMOP_VSU=2'b10;

****************************************************************************/


module vlane_saturatesum(
    in,
    op,
    out
    );

parameter WIDTH=32;

input [WIDTH+2-1:0] in;
input [1:0] op;
output [WIDTH-1:0] out;

reg [WIDTH-1:0] out;

wire op_saturate;
wire op_signed;

  assign op_saturate=op[1];
  assign op_signed=op[0];

wire [WIDTH-1:0] maxunsigned;
wire [WIDTH-1:0] minunsigned;
wire [WIDTH-1:0] maxsigned;
wire [WIDTH-1:0] minsigned;

  assign maxunsigned = {WIDTH{1'b1}};
  assign minunsigned = 0;
  assign maxsigned = {1'b0,{WIDTH-1{1'b1}}};
  assign minsigned = {1'b1,{WIDTH-1{1'b0}}};

wire [WIDTH-1:0] through;

  assign through=in[WIDTH-1:0];

wire [2:0] top3bits=(op_saturate) ? in[WIDTH+2-1:WIDTH-1] : 3'b0 ;

  always@*
    case(top3bits)
      3'b010: out=maxunsigned;
      3'b011: out=maxunsigned;
      3'b001: out=(op_signed) ? maxsigned : through;
      3'b111: out=(op_signed) ? through : minunsigned;
      3'b110: out=(op_signed) ? minsigned : minunsigned;
      default: out=through;
    endcase

endmodule

/****************************************************************************
          Saturate unit for different widths given 32-bit input

  Only works for 32-bit inputs

  Interprets input as signed/unsigned, saturates to signed/unsigned 
  byte/half/word

  NOP (pass through) when size=word and signed==outsigned

   out
   sign sign size
    0    0    00    VSAT_U_W - NOP (pass through)
    0    0    01    VSAT_U_B
    0    0    10    VSAT_U_H
    1    1    00    VSAT_W - NOP (pass through)
    1    1    01    VSAT_B
    1    1    10    VSAT_H
    0    1    00    VSAT_SU_W
    0    1    01    VSAT_SU_B
    0    1    10    VSAT_SU_H

parameter
  SATSIZEOP_VSATUW=4'b0000,
  SATSIZEOP_VSATUB=4'b0001,
  SATSIZEOP_VSATUH=4'b0010,
  SATSIZEOP_VSATW  =4'b1100,
  SATSIZEOP_VSATB  =4'b1101,
  SATSIZEOP_VSATH  =4'b1110,
  SATSIZEOP_VSATSUW=4'b0100,
  SATSIZEOP_VSATSUB=4'b0101,
  SATSIZEOP_VSATSUH=4'b0110;

****************************************************************************/

module vlane_saturatesize(
    in,
    op,
    out
    );

parameter WIDTH=32;

input [WIDTH-1:0] in;
input [3:0] op;
output [WIDTH-1:0] out;

wire op_outsigned;
wire op_signed;
wire op_size;

reg [WIDTH-1:0] out;

  assign op_outsigned=op[3];
  assign op_signed=op[2];
  assign op_size=op[1:0]; //0 - word, 1 - byte, 2 - half

  always@*
    case(op_size)
      2'b01:  //byte
        case({op_signed,op_outsigned})
          2'b11:    // signed
            out = ((in[WIDTH-1])&(!(&in[WIDTH-2:7]))) ? {{WIDTH-8{1'b1}},-128} :
                  ((~in[WIDTH-1]) && (|in[WIDTH-2:7])) ? 127 : in;
          2'b10:    // signed-unsigned
            out = (in[WIDTH-1]) ? 0 :
                  (~in[WIDTH-1]&&(|in[WIDTH-2:8])) ? 255 : in;
          default:  //2'b00: unsigned
            out=(|in[WIDTH-1:8]) ? 255 : in;
         endcase
      2'b10:  //half-word 16-bits
        case({op_signed,op_outsigned})
          2'b11:    // signed
            out=((in[WIDTH-1])&(!(&in[WIDTH-2:15])))? {{WIDTH-16{1'b1}},-32768}:
                  ((~in[WIDTH-1]) && (|in[WIDTH-2:15])) ? 32767 : in;
          2'b10:    // signed-unsigned
            out = (in[WIDTH-1]) ? 0 :
                  (~in[WIDTH-1]&&(|in[WIDTH-2:16])) ? 65535 : in;
          default:  //2'b00: unsigned
            out=(|in[WIDTH-1:16]) ? 65535 : in;
         endcase
      default: 
        case({op_signed,op_outsigned})
          2'b10:    // signed-unsigned
            out = (in[WIDTH-1]) ? 0 : in;
          default:  //2'b00: unsigned
            out=in;
         endcase
    endcase
  
endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vlane_saturate.v
//////////////////////////////////////////////////////////////////////

module vlane_alu(
    clk,
    resetn,

    pipe_en,
    pipe_squashn,

    src1,
    src2,
    mask,

    op,
    satsum_op,
    satsize_op,

    cmp_result,
    result

    );

parameter WIDTH=32;

input clk;
input resetn;

input pipe_en;
input pipe_squashn;

input  [WIDTH-1:0] src1;
input  [WIDTH-1:0] src2;

input  mask;

input  [10:0] op;
input  [1:0] satsum_op;
input  [3:0] satsize_op;

output cmp_result;
output [WIDTH-1:0] result;

wire [WIDTH-1:0] adder_opA;
wire [WIDTH-1:0] adder_opB;
wire [WIDTH+2-1:0] adder_result;

wire [WIDTH-1:0] logic_opA;
wire [WIDTH-1:0] logic_opB;
reg  [WIDTH-1:0] logic_result;

wire [WIDTH-1:0] mux0_result;
wire [WIDTH-1:0] mux1_result;

wire [WIDTH-1:0] satsum_result;
wire [WIDTH-1:0] satsize_result;

wire lt;
wire neq;
wire eq;
wire le;

wire ctrl_adderopA_sel;
wire ctrl_adderopB_sel;
wire ctrl_logicopB_sel;
wire ctrl_signed;
wire ctrl_addsub;
wire ctrl_max;
wire ctrl_min;
wire [1:0] ctrl_logic;
wire [1:0] ctrl_mux1_sel;
wire [1:0] ctrl_flag_sel;

  assign ctrl_adderopA_sel=op[10];
  assign ctrl_adderopB_sel=op[9];
  assign ctrl_logicopB_sel=op[8];
  assign ctrl_signed=op[7];
  assign ctrl_addsub=~op[6];  //Make add 0, sub 1
  assign ctrl_max=op[5];
  assign ctrl_min=op[4];
  assign ctrl_logic=op[3:2];
  assign ctrl_mux1_sel=op[1:0];
  assign ctrl_flag_sel=op[1:0];

  assign adder_opA=(ctrl_adderopA_sel) ? 0 : src1;
  assign adder_opB=(ctrl_adderopB_sel) ? src1 : src2;

  `ifdef USE_INHOUSE_LOGIC
  local_add_sub local_adder_inst(
      .dataa({{2{ctrl_signed&adder_opA[WIDTH-1]}},adder_opA}),
      .datab({{2{ctrl_signed&adder_opB[WIDTH-1]}},adder_opB}),
      .cin(~ctrl_addsub),
      .add_sub(ctrl_addsub),
      .result(adder_result)
  );
  defparam
      local_adder_inst.WIDTH = WIDTH+2,
      local_adder_inst.PIPELINE = 0,
      local_adder_inst.REPRESENTATION = "SIGNED";
  `else
  /*commenting due to error in yosys 
  lpm_add_sub adder_inst(
      .dataa({{2{ctrl_signed&adder_opA[WIDTH-1]}},adder_opA}),
      .datab({{2{ctrl_signed&adder_opB[WIDTH-1]}},adder_opB}),
      .cin(~ctrl_addsub),
      .add_sub(ctrl_addsub),
      .result(adder_result)
          // synopsys translate_off
          , .cout (), .overflow (), .clken (), .clock (), .aclr ()
          // synopsys translate_on
      );
  defparam
      adder_inst.lpm_width=WIDTH+2,
      adder_inst.lpm_pipeline=0,
      adder_inst.lpm_representation="SIGNED";
  */
  `endif

  assign lt=adder_result[WIDTH];
  assign neq=|adder_result;
  assign eq=~neq;
  assign le=lt||eq;

  assign cmp_result= (ctrl_flag_sel==0) ? eq :
                     (ctrl_flag_sel==1) ? neq :
                     (ctrl_flag_sel==2) ? lt : le;

  assign logic_opA=src1;
  assign logic_opB=(ctrl_logicopB_sel) ? 0 : src2;

  always@*
    case(ctrl_logic)
        2'b00:
            logic_result=logic_opA|logic_opB;
        2'b01:
            logic_result=logic_opA&logic_opB;
        2'b10:
            logic_result=logic_opA^logic_opB;
        2'b11:
            logic_result=~(logic_opA|logic_opB);
    endcase

  assign mux0_result=((lt && ctrl_max) || (~lt && ctrl_min)) ?  src2 : 
                                                                logic_result;

  /************* PIPELINE at this point ***************/
  reg [WIDTH-1:0] mux0_result_s2;
  reg [WIDTH+2-1:0] adder_result_s2;
  reg  [3:0] satsize_op_s2;
  reg  [3:0] satsum_op_s2;
  reg ctrl_mux1_sel_s2;

  always@(posedge clk)
  begin
    if (!resetn || !pipe_squashn)
    begin
      mux0_result_s2<=0;
      adder_result_s2<=0;
      satsize_op_s2<=0;
      satsum_op_s2<=0;
      ctrl_mux1_sel_s2<=0;
    end
    else if (pipe_en)
    begin
      mux0_result_s2<=mux0_result;
      adder_result_s2<=adder_result;
      satsize_op_s2<=satsize_op;
      satsum_op_s2<=satsum_op;
      ctrl_mux1_sel_s2<=((ctrl_mux1_sel==0) ||
                         (ctrl_mux1_sel==2 && src1[WIDTH-1]) ||
                         (ctrl_mux1_sel==3 && mask) );
    end
  end

  vlane_saturatesize #(32) satsize(   //Unit only works for 32-bit inputs
      .in(mux0_result_s2),
      .op(satsize_op_s2),
      .out(satsize_result)
      );

  vlane_saturatesum #(WIDTH) satsum(
      .in(adder_result_s2),
      .op(satsum_op_s2),
      .out(satsum_result)
      );

  assign mux1_result= (ctrl_mux1_sel_s2) ? 
                              satsum_result : satsize_result;

  assign result=mux1_result;

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vlane_alu.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vmul_unit.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vmul_unit.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vlane_mulshift.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vlane_mulshift.v
//////////////////////////////////////////////////////////////////////

/****************************************************************************
          MUL unit

Operation table

     half sat  op unsign dir
2      0   0   0   0    0    |  Zero (This value is used to reset multiplier)
14     1   0   1   1    0    |  LMULU Lower MULtiply Unsigned
15     1   0   1   1    1    |  UMULU Upper MULtiply Unsigned
16     1   0   1   0    0    |  LMUL Lower MULtiply signed
17     1   0   1   0    1    |  UMUL Upper MULtiply signed
4      0   0   1   1    0    |  MULLOU
4      0   0   1   1    1    |  MULHIU
6      0   0   1   0    0    |  MULLO
6      0   0   1   0    1    |  MULHI
0      0   0   0   1    0    |  ShiftLeft
8      0   1   0   1    0    |  ShiftLeftSatU
10     0   1   0   0    0    |  ShiftLeftSat
1      0   0   0   1    1    |  ShiftRightLogic
3      0   0   0   0    1    |  ShiftRightArith
****************************************************************************/
module vlane_mulshift(clk, resetn,
            opA, opB, sa,
            op,
            en,
            result
            );
parameter WIDTH=32;
parameter LOG2WIDTH=5;

input clk;
input resetn;

input [WIDTH-1:0] opA;
input [WIDTH-1:0] opB;
input [LOG2WIDTH-1:0] sa;
input [4:0] op;
input [3:1] en;  //Enable for each pipestage

output [WIDTH-1:0] result;

/********* Control Signals *********/
wire is_signed,dir, is_mul, saturate;
assign is_mul=op[2];      // selects between opB and the computed shift amount
assign is_signed=~op[1];
assign dir=op[0];         // selects between 2^sa and 2^(32-sa) for right shift
assign saturate=op[3];
assign half=op[4];

/********* Circuit Body *********/
wire dum,dum2,dum3;
wire [WIDTH:0] opB_mux_out;
wire [WIDTH-1:0] opA_mux_out;
wire [WIDTH-1:0] opB_mul;
wire [5-1:0] left_sa;     // Amount of left shift required for both left/right
reg [WIDTH:0] decoded_sa;
wire [WIDTH-1:0] hi;
wire [WIDTH-1:0] lo;

assign opA_mux_out = (~half) ? ( opA ) : (WIDTH<2) ? 0 : (dir) ?
              {{WIDTH/2{is_signed&is_mul&opA[WIDTH-1]}},opA[WIDTH-1:WIDTH/2]} :
              {{WIDTH/2{is_signed&is_mul&opA[WIDTH/2-1]}},opA[WIDTH/2-1:0]};

assign opB_mul = (~half) ? opB : (WIDTH<2) ? 0 : (dir) ? 
              {{WIDTH/2{is_signed&is_mul&opB[WIDTH-1]}},opB[WIDTH-1:WIDTH/2]} :
              {{WIDTH/2{is_signed&is_mul&opB[WIDTH/2-1]}},opB[WIDTH/2-1:0]};

assign opB_mux_out=(is_mul) ? {is_signed&opB_mul[WIDTH-1],opB_mul} : decoded_sa;

reg zeroout;
always@(posedge clk)
  if (en[1])
    zeroout<=(op[3:0]==0);
`ifdef USE_INHOUSE_LOGIC
local_mult local_mult_component (
.dataa({is_signed&opA_mux_out[WIDTH-1],opA_mux_out}),
.datab(opB_mux_out),
.clock(clk),
.clken(en[1]),
.aclr(~resetn),
.result({dum2,dum,hi,lo})
);
defparam
 local_mult_component.LPM_WIDTHA = WIDTH + 1,
 local_mult_component.LPM_WIDTHB = WIDTH + 1,
 local_mult_component.LPM_WIDTHP = 2*WIDTH + 2,
 local_mult_component.LPM_REPRESENTATION = "SIGNED";
`else 
/* 
lpm_mult  lpm_mult_component (
  .dataa ({is_signed&opA_mux_out[WIDTH-1],opA_mux_out}),
  .datab (opB_mux_out),
  .sum(),
  .clock(clk),
  .clken(en[1]),
  .aclr(~resetn),
  .result ({dum2,dum,hi,lo}));
defparam
  lpm_mult_component.lpm_widtha = WIDTH+1,
  lpm_mult_component.lpm_widthb = WIDTH+1,
  lpm_mult_component.lpm_widthp = 2*WIDTH+2,
  lpm_mult_component.lpm_widths = 1,
  lpm_mult_component.lpm_pipeline = 1,
  lpm_mult_component.lpm_type = "LPM_MULT",
  lpm_mult_component.lpm_representation = "SIGNED",
  lpm_mult_component.lpm_hint = "MAXIMIZE_SPEED=6";
  */
`endif
// if A is positive/negative make it maximum/minimum positive/negative
wire [WIDTH-1:0] signedsaturate=
                    (opA_mux_out[WIDTH-1]) ? {1'b1,{WIDTH-1{1'b0}}} : 
                                             {1'b0,{WIDTH-1{1'b1}}};

reg [WIDTH-1:0] saturatedval_s2;
reg saturate_s2;

//Capture saturated value and saturate signal for next stage
always@(posedge clk)
  if (!resetn)
  begin
    saturatedval_s2<=0;
    saturate_s2<=0;
  end
  else if (en[1])
  begin
    saturatedval_s2<=((~is_signed) ? {WIDTH{1'b1}} : signedsaturate);
    saturate_s2<=saturate;
  end

reg sel_hi;

always@(posedge clk)
  if (!resetn)
    sel_hi<=0;
  else if (en[1])
    sel_hi<=(is_mul && dir || ~is_mul && dir && |sa);

assign result =(zeroout) ? 0 : 
                (saturate_s2 && |hi ) ? saturatedval_s2 : 
                (sel_hi) ? hi : lo;


assign {dum3, left_sa}= (dir) ? WIDTH-sa : {1'b0,sa};

integer i;

//Decoder - computes 2^left_sa
always@*
begin
  decoded_sa=0;
  for (i=0; i<WIDTH; i=i+1)
    if (left_sa==i)
      decoded_sa[i]=1'b1;
end


endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vlane_mulshift.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vlane_barrelshifter.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vlane_barrelshifter.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          Shifter unit

Opcode Table:

sign_ext dir 
 0        0    |  ShiftLeft
 0        1    |  ShiftRightLogic
 1        1    |  ShiftRightArith
          
****************************************************************************/
module vlane_barrelshifter(clk, resetn,
            opB, sa, 
            op, 
            result);
parameter WIDTH=32;
parameter LOG2WIDTH=5;

//Shifts the first 2 bits in one cycle, the rest in the next cycle
parameter REGISTERBREAK=LOG2WIDTH-2;

input clk;
input resetn;

input [WIDTH-1:0] opB;
input [LOG2WIDTH-1:0] sa;                             // Shift Amount
input [2-1:0] op;

output [WIDTH-1:0] result;


wire sign_ext;
wire shift_direction;
assign sign_ext=op[1];
assign shift_direction=op[0];

wire dum,dum_,dum2;
wire [WIDTH-1:0] partial_result_,partial_result;

`ifdef USE_INHOUSE_LOGIC
 local_shifter local_shifter_inst1(
  .data({sign_ext&opB[WIDTH-1],opB}),
  .distance(sa&(32'hffffffff<<((REGISTERBREAK>0) ? REGISTERBREAK : 0))),
  .direction(shift_direction),
  .result({dum,partial_result})
 );
 defparam
    local_shifter_inst1.LPM_WIDTH = WIDTH+1,
    local_shifter_inst1.LPM_WIDTHDIST = LOG2WIDTH,
    local_shifter_inst1.LPM_SHIFTTYPE="ARITHMETIC";
`else
/*
lpm_clshift shifter_inst1(
    .data({sign_ext&opB[WIDTH-1],opB}),
    .distance(sa&(32'hffffffff<<((REGISTERBREAK>0) ? REGISTERBREAK : 0))),
    .direction(shift_direction),
    .result({dum,partial_result}));
 defparam
    shifter_inst1.lpm_width = WIDTH+1,
    shifter_inst1.lpm_widthdist = LOG2WIDTH,
    shifter_inst1.lpm_shifttype="ARITHMETIC";
  */
`endif

register_sync partial_reg
  ({dum,partial_result},clk,resetn,1'b1,{dum_,partial_result_});
    defparam partial_reg.WIDTH=WIDTH+1;

wire [5-1:0] sa_2;
wire shift_direction_2;

register_sync secondstage (sa, clk,resetn,1'b1,sa_2); 
  defparam secondstage.WIDTH=5;

register_sync secondstagedir (shift_direction, clk,resetn,1'b1,shift_direction_2); 
  defparam secondstagedir.WIDTH=1;

`ifdef USE_INHOUSE_LOGIC
 local_shifter local_shifter_inst2(
  .data({dum_,partial_result_}),
  .distance(sa_2[((REGISTERBREAK>0) ? REGISTERBREAK-1 : 0):0]),
  .direction(shift_direction_2),
  .result({dum2,result})
 );
 defparam
    local_shifter_inst2.LPM_WIDTH = WIDTH+1,
    local_shifter_inst2.LPM_WIDTHDIST = (REGISTERBREAK>0) ? REGISTERBREAK : 1,
    local_shifter_inst2.LPM_SHIFTTYPE ="ARITHMETIC";
`else
/*
lpm_clshift shifter_inst2(
    .data({dum_,partial_result_}),
    .distance(sa_2[((REGISTERBREAK>0) ? REGISTERBREAK-1 : 0):0]),
    .direction(shift_direction_2),
    .result({dum2,result}));
 defparam 
    shifter_inst2.lpm_width = WIDTH+1,
    shifter_inst2.lpm_widthdist = (REGISTERBREAK>0) ? REGISTERBREAK : 1,
    shifter_inst2.lpm_shifttype="ARITHMETIC";
    */
`endif


endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vlane_barrelshifter.v
//////////////////////////////////////////////////////////////////////

/****************************************************************************
          MUL unit

opA/B ---------------------------|\
         |                       | |----| Multiplier |--+------------------
         ------|shiftregister|---|/                     |--|shiftregister|-

 |  Pipe stage 1  |      Pipe stage 2    |    Pipe stage 3
                         done/stall
   operands----Multiply------------barrelshifter ----------

Notes:

  Stalls no matter the vector length - if the mask bits are all off we could
  theoretically skip this computation, but then we would need multiplexing
  logic to order the different results correctly so they line up with their
  lane.  Since we use a shift register to do this, all NUMLANES multiplies are
  performed.

****************************************************************************/

module vmul_unit(clk, resetn,
            op,
            activate,
            en,
            squash,
            stall,
            opA, opB,
            vshamt,
            vmask,
            in_dst,
            in_dst_we,
            out_dst,
            out_dst_we,
            out_dst_mask,
            result
            );
parameter LOG2WIDTH=5;
parameter NUMMULLANES=3;
parameter LOG2NUMLANES=4;
parameter REGIDWIDTH=4;

parameter NUMLANES=2**LOG2NUMLANES;
parameter WIDTH=2**LOG2WIDTH;

input clk;
input resetn;

input [NUMLANES*WIDTH-1:0] opA;
input [NUMLANES*WIDTH-1:0] opB;
input [((LOG2WIDTH==0) ? 1 : LOG2WIDTH)-1:0] vshamt;  // Fixed point rounding
input [NUMLANES-1:0] vmask;
input [4:0] op;
input       activate;
input [3:1] en;  //Enable for each pipestage
input [3:1] squash;  //Squash for each pipestage

input    [REGIDWIDTH-1:0] in_dst;
input                     in_dst_we;
output [3*REGIDWIDTH-1:0] out_dst;
output              [2:0] out_dst_we;
output   [3*NUMLANES-1:0] out_dst_mask;

output stall;
output [NUMLANES*WIDTH-1:0] result;

  /********* Circuit Body *********/
  wire [NUMMULLANES*WIDTH-1:0] mul_opA;
  wire [NUMMULLANES*WIDTH-1:0] mul_opB;
  wire [NUMMULLANES*WIDTH-1:0] mul_result;
  wire [NUMMULLANES*WIDTH-1:0] rshift_result;
  wire [NUMLANES*WIDTH-1:0] result_tmp;

  wire [NUMLANES*WIDTH-1:0] opA_buffered;
  wire [NUMLANES*WIDTH-1:0] opB_buffered;
  wire [NUMLANES-1:0]       mask_buffered;
  wire [NUMLANES*WIDTH-1:0] result_buffered;
  reg  done;
wire [(((3)-(1)+1)*((4)-(0)+1))-1 : 0] ctrl_op;
  wire [3:1] ctrl_activate;                 //3 pipe stages
  //amana: Making modification for VTR
  //wire [((LOG2WIDTH==0) ? 1 : LOG2WIDTH)-1:0] ctrl_vshamt[3:1]; //3 pipe stages
  wire [((LOG2WIDTH==0) ? 1*3 : LOG2WIDTH*3)-1:0] ctrl_vshamt; //3 pipe stages

  //Shift Register for all multiplier operands for lanes without multipliers
  wire [WIDTH*NUMMULLANES-1:0] opA_elmshifter_shiftin_right_NC;
  velmshifter #(NUMLANES/NUMMULLANES,WIDTH*NUMMULLANES) opA_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*NUMMULLANES{1'b0}}),
    .shiftin_right(opA_elmshifter_shiftin_right_NC), 
    .inpipe(opA),
    .outpipe(opA_buffered)
  );

  wire [WIDTH*NUMMULLANES-1:0] opB_elmshifter_shiftin_right_NC;
  velmshifter #(NUMLANES/NUMMULLANES,WIDTH*NUMMULLANES) opB_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({WIDTH*NUMMULLANES{1'b0}}),
    .shiftin_right(opB_elmshifter_shiftin_right_NC), 
    .inpipe(opB),
    .outpipe(opB_buffered)
  );

  wire [NUMMULLANES-1:0] mask_elmshifter_shiftin_right_NC;
  velmshifter #(NUMLANES/NUMMULLANES,NUMMULLANES) mask_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(done & ctrl_activate[1]),
    .shift(1'b1),
    .dir_left(1'b0),
    .squash(1'b0),
    .shiftin_left({NUMMULLANES{1'b0}}),
    .shiftin_right(mask_elmshifter_shiftin_right_NC), 
    //.inpipe(vmask), //DISABLE - always do all multiplications
    .inpipe({NUMLANES{1'b1}}),
    .outpipe(mask_buffered)
  );

  //Shift Register for all multiplier operands w/ parallel load
  always@(posedge clk)
  begin
    if (!resetn || NUMMULLANES==NUMLANES)
      done<=1;
    else if (done && ctrl_activate[1] && en[1])
      //done<=~(|(vmask>>NUMMULLANES)); // multiply only if mask - DISABLED
      done<=~(|(vmask));
    else
      done<=~( |(mask_buffered >> (2*NUMMULLANES) ));
  end

  assign mul_opA=(done) ? opA : (opA_buffered >> NUMMULLANES*WIDTH);

  assign mul_opB=(done) ? opB : (opB_buffered >> NUMMULLANES*WIDTH);

  assign stall=~done && (ctrl_activate[2]);

  wire [3:1] oppipe_squash_NC; 
  pipe #(5,2) oppipe (
    .d(op),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(oppipe_squash_NC),
    .q({ctrl_op[((3-1)*((4)-(0)+1)+4-0) : ((3-1)*((4)-(0)+1))],ctrl_op[((2-1)*((4)-(0)+1)+4-0) : ((2-1)*((4)-(0)+1))],ctrl_op[((1-1)*((4)-(0)+1)+4-0) : ((1-1)*((4)-(0)+1))]}));

  wire [3:1] activatepipe_squash_NC; 
  pipe #(1,2) activatepipe (
    .d(activate),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(activatepipe_squash_NC),
    .q({ctrl_activate[3],ctrl_activate[2],ctrl_activate[1]}));

  wire [3:1] vshamtpipe_squash_NC; 
  pipe #(((LOG2WIDTH==0) ? 1 : LOG2WIDTH),2) vshamtpipe (
    .d(vshamt),
    .clk(clk),
    .resetn(resetn),
    .en(en),
    .squash(vshamtpipe_squash_NC),
    //.q({ctrl_vshamt[3],ctrl_vshamt[2],ctrl_vshamt[1]}));
    .q(ctrl_vshamt));

  //============== Instantiate across lanes =============
  genvar k;
  generate
  for (k=0; k<NUMMULLANES; k=k+1)
  begin : lanes_gen

    vlane_mulshift #(WIDTH,(LOG2WIDTH>0) ? LOG2WIDTH : 1) vmul(
      .clk(clk),
      .resetn(resetn),
      .en(en[1] | ~done),
      .opA(mul_opA[WIDTH*k +: WIDTH]),
      .opB(mul_opB[WIDTH*k +: WIDTH]),
      .sa( mul_opB[WIDTH*k+((LOG2WIDTH>0)?LOG2WIDTH-1:0) : WIDTH*k] ),
      .op( (done&en[1]) ? ctrl_op[((1-1)*((4)-(0)+1)+4-0) : ((1-1)*((4)-(0)+1))] : ctrl_op[((2-1)*((4)-(0)+1)+4-0) : ((2-1)*((4)-(0)+1))]),
      .result(mul_result[WIDTH*k +: WIDTH])
      );

    vlane_barrelshifter #(WIDTH,(LOG2WIDTH>0)?LOG2WIDTH:1) vshift(
      .clk(clk),
      .resetn(resetn),
      .opB(mul_result[WIDTH*(k+1)-1:WIDTH*k]),
     // .sa(ctrl_vshamt[2][((LOG2WIDTH>0) ? LOG2WIDTH-1:0) : 0]),
      .sa(ctrl_vshamt[2*LOG2WIDTH-1:LOG2WIDTH]),
      .op({~ctrl_op[((2-1)*((4)-(0)+1)+1-1) : ((2-1)*((4)-(0)+1))] ,1'b1}),
      .result(rshift_result[WIDTH*(k+1)-1:WIDTH*k])
      );

  end
  endgenerate


  //Shift Register for all multiplier results
  wire [NUMMULLANES*WIDTH-1:0] shiftin_right_result_elmshifter_NC;
  wire [NUMLANES*WIDTH-1:0] inpipe_result_elmshifter_NC;
  velmshifter #(NUMLANES/NUMMULLANES,WIDTH*NUMMULLANES) result_elmshifter (
    .clk(clk),
    .resetn(resetn),
    .load(1'b0),
    .shift(~done),
    .dir_left(1'b0),
    .squash(1'b0),
    //Enable/Disable rshifter for fixed-point multiply support
    //.shiftin_left(rshift_result),
    .shiftin_left(mul_result),
    .shiftin_right(shiftin_right_result_elmshifter_NC),
    .inpipe(inpipe_result_elmshifter_NC),
    .outpipe(result_buffered)
  );

  //Enable/Disable rshifter for fixed-point multiply support
  //assign result_tmp=rshift_result;
  assign result_tmp=mul_result;

  assign result=(result_tmp<<((NUMLANES/NUMMULLANES-1)*NUMMULLANES*WIDTH)) |
                (result_buffered>>NUMMULLANES*WIDTH);

  wire [2:1] dstpipe_squash_NC;
  pipe #(REGIDWIDTH,2) dstpipe (
    .d( in_dst ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(dstpipe_squash_NC),
    .q(out_dst));

  pipe #(1,2) dstwepipe (
    .d( in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(squash[2:1]),
    .q(out_dst_we));

  wire [2:1] dstmaskpipe_squash_NC;
  pipe #(NUMLANES,2) dstmaskpipe (
    .d( vmask ),  
    .clk(clk),
    .resetn(resetn),
    .en( en[2:1] & {1'b1,~stall} ),
    .squash(dstmaskpipe_squash_NC),
    .q(out_dst_mask));


endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vmul_unit.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vmem_unit.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vmem_unit.v
//////////////////////////////////////////////////////////////////////
// THIS UNIT SHOULD BE REDESIGNED!!!!!
// It started off simple with low performance and after adding a bunch of hacks
// to make it perform better it's gotten messy.  A new memory unit should
// special case strided loads and service them early and pipelined, and it
// should have a store buffer with variable width for writes.

/************************
 *
 * VPUWIDTH must equal CONTROLWIDTH must equal 32 
 * NUMPARALLELLANES must be a divisor of NUMLANES
 *
 * op[6] - memop 1-memory operation, 0-shift operation
 *
 * op[5] op[4]
 *   0     0    UNIT
 *   0     1    STRIDE
 *   1     0    INDEX
 *
 * size1 size0 signd ld/st
 * op[3] op[2] op[1] op[0]
 *   0     0     0     0    VLD.u.b
 *   0     1     0     0    VLD.u.h
 *   1     0     0     0    VLD.u.w
 *   0     0     1     0    VLD.b
 *   0     1     1     0    VLD.h
 *   1     0     1     0    VLD.w
 *   0     0     X     1    VST.b
 *   0     1     X     1    VST.h
 *   1     0     X     1    VST.w
 *
 *
 * Ported to handle element shifting as well:
 *
 * Data shifts
 * ===========
 * vins -   L shift vindex
 * vext -     R shift vindex
 * vcompress -  R
 * vexpand -  L 
 * vfins -  L vindex
 * vhalf -        R shift vl/2
 * vexthalf -     R shift vl/2
 * vhalfup -  R shift 2^vindex
 * vhalfdn -  L shift 2^vindex
 *
 * Mask shifting - much harder
 * =============
 *
 * Quartus 5.0: 919 LEs, 137 MHz (CritPath: vip_r to vreaddata)
 *************************/
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: velmshifter_serial.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/velmshifter_serial.v
//////////////////////////////////////////////////////////////////////
/************************
 * An Inter-lane shift register
 *
 * This is much more scalable than a giant barrel shifter/crossbar, however
 * it is also slower in cycles.
 *
 *  dir_left:  1=left, 0=right
 *
 * Synthesis results for NUMLANES=4, WIDTH=32:
 *    131 LEs (Stratix I), 422 MHz
 *************************/

/***************************************************************************
  shiftin_left -> bn-1 <-> bn-2 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

module velmrotator (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    rotate,       // Sets whether ends feedback into beginning
    squash,

    inpipe,
    outpipe

    );

parameter NUMLANES=4;      
parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  rotate;
input [ NUMLANES-1:0 ]  squash;

input [ NUMLANES*WIDTH-1:0 ]  inpipe;
output [ NUMLANES*WIDTH-1:0 ] outpipe;

wire [ WIDTH-1:0 ]  shiftin_left;
wire [ WIDTH-1:0 ]  shiftin_right;


  assign shiftin_right = (!rotate) ? 0 : 
                        outpipe[NUMLANES*WIDTH-1:(NUMLANES-1)*WIDTH];

  assign shiftin_left = (!rotate) ? 0 : outpipe[WIDTH-1:0];

  velmshifter #(NUMLANES,WIDTH) velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load),
      .shift(shift),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe(inpipe),
      .outpipe(outpipe));


endmodule


/******************************************************************************/
/**************************** Shifter w Jump **********************************/
/******************************************************************************/

module velmshifter_jump (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    jump,
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe
    );

parameter NUMLANES=4;      
parameter JUMPSIZE=4;     // We can either shift by 1, or jump by this amount 
parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  jump;              //0-shift by 1, 1 shift by JUMPSIZE
input [ NUMLANES-1:0 ]  squash;

input [ NUMLANES*WIDTH-1:0 ]  inpipe;
output [ NUMLANES*WIDTH-1:0 ] outpipe;

input [ WIDTH-1:0 ]  shiftin_left;
input [ WIDTH-1:0 ]  shiftin_right;

genvar i;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  velmshifter #(NUMLANES,WIDTH) velmshift (
      .clk(clk),
      .resetn(resetn),      
      .load(load || (shift&jump)),
      .shift(shift&~jump),
      .dir_left(dir_left),
      .squash(squash),
      .shiftin_left(shiftin_left),
      .shiftin_right(shiftin_right),
      .inpipe( (load) ? inpipe : (dir_left) ? 
                      outpipe <<(WIDTH*JUMPSIZE) :
                      outpipe >>(WIDTH*JUMPSIZE)),
      .outpipe(outpipe));

endmodule

/******************************************************************************/
/******************************** Shifter *************************************/
/******************************************************************************/

module velmshifter (
    clk,
    resetn,

    load,
    shift,
    dir_left,     // Sets the direction, 1=left, 0=right
    squash,

    shiftin_left,
    shiftin_right,

    inpipe,
    outpipe

    );

parameter NUMLANES=4;      
parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input [ NUMLANES-1:0 ]  squash;

input [ NUMLANES*WIDTH-1:0 ]  inpipe;
output [ NUMLANES*WIDTH-1:0 ] outpipe;

input [ WIDTH-1:0 ]  shiftin_left;
input [ WIDTH-1:0 ]  shiftin_right;

wire [ (NUMLANES+1)*WIDTH-1:0 ] _outpipe;

genvar i;
genvar j;

/***************************************************************************
  shiftin_left -> bn <-> bn-1 <-> ... <-> b1 <-> b0 <- shiftin_right
***************************************************************************/

  //HANDLE lane 0 specially

  velmshifter_laneunit velmshifter_laneunit0(clk,resetn,load,shift,dir_left,
      squash[0],
      inpipe[WIDTH-1:0],
      _outpipe[((NUMLANES>1) ? WIDTH : 0)+:WIDTH], //Support 1 lane
      shiftin_right,
      _outpipe[WIDTH-1:0]);
  defparam velmshifter_laneunit0.WIDTH=WIDTH;

  //Generate everything in between 

  generate
    for (i=1; i<NUMLANES-1; i=i+1)
    begin : elmshift
      velmshifter_laneunit velmshifter_laneunit(clk,resetn,load,shift,dir_left,
          squash[i],
          inpipe[(i+1)*WIDTH-1:i*WIDTH],
          _outpipe[(i+2)*WIDTH-1:(i+1)*WIDTH],
          _outpipe[(i)*WIDTH-1:(i-1)*WIDTH],
          _outpipe[(i+1)*WIDTH-1:i*WIDTH]);
      defparam velmshifter_laneunit.WIDTH=WIDTH;
    end
  endgenerate

  //HANDLE lane NUMLANE specially

    velmshifter_laneunit velmshifter_laneunitlast(
      clk,resetn,load,shift,dir_left,
      squash[NUMLANES-1],
      inpipe[NUMLANES*WIDTH-1:(NUMLANES-1)*WIDTH],
      shiftin_left,
      _outpipe[(((NUMLANES>1) ? NUMLANES:2)-2)*WIDTH +: WIDTH], //L=1
      _outpipe[((NUMLANES>1) ? (NUMLANES-1)*WIDTH : WIDTH) +: WIDTH]); //L=1
    defparam velmshifter_laneunitlast.WIDTH=WIDTH;

    //Support L=1 - give _outpipe more bits but ignore them
assign outpipe=_outpipe;

endmodule



module velmshifter_laneunit (
    clk,
    resetn,

    load,
    shift,
    dir_left,
    squash,

    inpipe,
    inxlaneleft,    // Cross-lane input from lane i+1
    inxlaneright,    // Cross-lane input from lane i-1
    outpipe

    );

parameter WIDTH=32;       // Width of the shifter

input clk;
input resetn;

input  load;
input  shift;
input  dir_left;
input  squash;

input [ WIDTH-1:0 ]  inpipe;
input [ WIDTH-1:0 ]  inxlaneleft;    // X-lane input lane i-1
input [ WIDTH-1:0 ]  inxlaneright;    // X-lane input lane i-1
output [ WIDTH-1:0 ] outpipe;

reg [ WIDTH-1:0 ] outpipe;   // Cross-lane output lane i+1


  always@(posedge clk)
    if (resetn==0 || squash)
      outpipe<=0;
    else if (load)
      outpipe<=inpipe;
    else if (shift)
      outpipe<=(dir_left) ? inxlaneright : inxlaneleft;

endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: velmshifter_serial.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vmem_crossbar.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vmem_crossbar.v
//////////////////////////////////////////////////////////////////////
/************************
 *
 *************************/

module vmem_crossbar (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter INWIDTH=128;       //Bit width of input
parameter LOG2INWIDTH=7;
parameter NUMOUTS=16;        //Number of Outputs
parameter OUTWIDTH=8;        //Bit width of each Output
parameter LOG2OUTWIDTH=3;

parameter SELWIDTH=LOG2INWIDTH-LOG2OUTWIDTH;   // LOG2(INWIDTH/OUTWIDTH) = 4

input                             clk;
input                             resetn;
input  [(SELWIDTH*NUMOUTS)-1 : 0] sel;
input  [INWIDTH-1 : 0]            in;
output [(OUTWIDTH*NUMOUTS)-1 : 0] out;

genvar i;

 generate
   for (i=0; i < NUMOUTS; i=i+1) begin : MEM
     vmem_busmux bmux(clk,resetn,
        sel[(i+1)*SELWIDTH - 1 : i*SELWIDTH],
        in,
        out[(i+1)*OUTWIDTH - 1 : i*OUTWIDTH]);
      defparam bmux.INWIDTH=INWIDTH;
      defparam bmux.LOG2INWIDTH=LOG2INWIDTH;
      defparam bmux.OUTWIDTH=OUTWIDTH;
      defparam bmux.LOG2OUTWIDTH=LOG2OUTWIDTH;
   end
 endgenerate


endmodule



module vmem_busmux (
    clk,
    resetn,
    sel,
    in,
    out
    );

parameter INWIDTH=128;
parameter LOG2INWIDTH=7;
parameter OUTWIDTH=8;
parameter LOG2OUTWIDTH=3;

parameter SELWIDTH=LOG2INWIDTH-LOG2OUTWIDTH;   // LOG2(INWIDTH/OUTWIDTH) = 4

input clk;
input resetn;
input  [SELWIDTH-1 : 0] sel;
input  [INWIDTH-1 : 0]  in;
output [OUTWIDTH-1 : 0] out;

reg    [OUTWIDTH-1 : 0] out;

integer k;

  always@*
  begin
    out=0;
    for (k=0; k<INWIDTH/OUTWIDTH; k=k+1)
      if (k==sel)
        out=in[ k*OUTWIDTH +: OUTWIDTH ];
  end


endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vmem_crossbar.v
//////////////////////////////////////////////////////////////////////

module vmem_unit (
    clk,
    resetn,

    enable,       //Is this unit on? (is it less than VL, does the instr use it)
    en, //Is pipeline advancing
    squash, //Is pipeline squashing us
    op,
    stall,
    last_subvector,

    // Control ports
    cbase,
    cstride,
    cprefetch,

    // Vector ports
    vmask,
    vstrideoffset,  //pre-computed stride offsets for each lane (stride*laneid)
    vindex,
    vwritedata,
    voutput,
    voutput_we,

    // Writeback ports
    in_dst,
    in_dst_we,
    out_dst,
    out_dst_we,
    in_vs_dst_we,
    out_vs_dst_we,

    // Vector operations ports
    sa,
    dir_left,

    // Data memory interface
    dmem_en,
    dmem_address,
    dmem_we,
    dmem_byteen,
    dmem_writedata,
    dmem_readdata,
    dmem_cachematch,
    dmem_cachemiss,
    dmem_prefetch,
    dmem_wait
    );

parameter VPUWIDTH=32;       // Width of the Vector processing unit in bits!
parameter NUMLANES=4;        // Number of vector lanes
parameter LOG2NUMLANES=2;    // Log base 2 of Number of vector lanes
parameter NUMPARALLELLANES=2;        // Number of vector lanes
parameter LOG2NUMPARALLELLANES=1;    // Log base 2 of Number of vector lanes
parameter CONTROLWIDTH=32;   // Width of the Control Registers
parameter DMEM_WRITEWIDTH=128;     // Width of write bus to memory
parameter LOG2DMEM_WRITEWIDTH=7;  // Log2 of Width of write bus to memory
parameter DMEM_READWIDTH=128;     // Width of read bus from memory
parameter LOG2DMEM_READWIDTH=7;   // Log2 of Width of read bus from memory
parameter ELMIDWIDTH=2;           // Width of register element identifier
parameter REGIDWIDTH=5;           // Width of register element identifier

parameter       MUNIT_IDLE=0,
                MUNIT_ISSUE=1,
                MUNIT_DONE=2,
                MUNIT_SHIFT=3, 
                MUNIT_XTRAWRITE=4; 
  
parameter ISSUE_IDLE=3'b000,
          ISSUE_INITIAL_INCREMENT=3'b001,
          ISSUE_INCREMENTED=3'b010,
          ISSUE_WAITONE=3'b011,
          ISSUE_WAITING=3'b100;

input clk;
input resetn;

input  enable;
input  [2:0] en; //Is pipeline one
input  [2:0] squash;
input [6:0] op;
output stall;
input last_subvector;

// Control ports
input [ CONTROLWIDTH-1 : 0 ] cbase;
input [ CONTROLWIDTH-1 : 0 ] cstride;
input [ CONTROLWIDTH-1 : 0 ] cprefetch;

// Vector ports
input  [          NUMLANES-1 : 0 ]  vmask;
input  [ NUMLANES*CONTROLWIDTH-1 : 0 ]  vstrideoffset;
input  [ NUMLANES*VPUWIDTH-1 : 0 ]  vindex;
input  [ NUMLANES*VPUWIDTH-1 : 0 ]  vwritedata;
output [ NUMLANES*VPUWIDTH-1 : 0 ]  voutput;
output [          NUMLANES-1 : 0 ]  voutput_we;

input   [REGIDWIDTH-1:0]  in_dst;
input                     in_dst_we;
output   [REGIDWIDTH-1:0] out_dst;
output                    out_dst_we;
input                     in_vs_dst_we;
output                    out_vs_dst_we;

input  [      LOG2NUMLANES-1 : 0 ]  sa;
input                               dir_left;

// Data memory interface
output dmem_en;
output dmem_we;
output  [ 31 : 0 ]                  dmem_address;
output  [ DMEM_WRITEWIDTH/8-1 : 0 ] dmem_byteen;
output  [ DMEM_WRITEWIDTH-1 : 0 ]   dmem_writedata;
input   [ DMEM_READWIDTH-1 : 0 ]    dmem_readdata;
input                               dmem_cachematch;
input                               dmem_cachemiss;
output  [ 31 : 0 ]                  dmem_prefetch;
input                               dmem_wait;

reg     [ 31 : 0 ]                  dmem_address;
reg     [ DMEM_WRITEWIDTH/8-1 : 0 ] dmem_byteen;
reg     [ DMEM_WRITEWIDTH-1 : 0 ]   dmem_writedata;
reg     [ 31 : 0 ]                  dmem_prefetch;

wire  [1:0]  op_pattern;      // 0-Unit, 1-Strided, 2-Indexed
wire  [1:0]  op_size;         // 0-byte, 1-16bits, 2-32bits, 3-64bits
wire         op_signed;       // 0-unsigned, 1-signed
wire         op_we;         // 0-load, 1-store
wire         op_memop;         // 1-memory op, 0-vector shift op

wire  [ NUMPARALLELLANES*VPUWIDTH-1 : 0 ]  __vreaddata;
reg           [ NUMLANES*CONTROLWIDTH-1 : 0 ]  address;
reg              [ NUMLANES*VPUWIDTH-1:0]  vreaddata;
reg                    [ NUMLANES-1 : 0 ]  vreaddata_we;
reg   [ NUMPARALLELLANES*(LOG2DMEM_READWIDTH-5)-1 : 0 ] crossbar_sel;
wire  [ NUMPARALLELLANES*32-1 : 0 ] crossbar;
wire  [ NUMPARALLELLANES*32-1 : 0 ]  _vwritedata;
wire   [ NUMPARALLELLANES*4-1 : 0 ]  _vbyteen;

reg  [ CONTROLWIDTH-1 : 0 ] stride;
wire [ CONTROLWIDTH-1 : 0 ] stride_tmp;
reg  [ CONTROLWIDTH-1 : 0 ] prefetch;
wire [ CONTROLWIDTH-1 : 0 ] t_cprefetch;

wire           [         NUMLANES-1 : 0 ]  vshifted_mask;
wire           [         NUMLANES-1 : 0 ]  vshifted_masksave;
wire          [ NUMLANES*VPUWIDTH-1 : 0 ]  vshifted_writedata;
wire          [ NUMLANES*CONTROLWIDTH-1 : 0 ]  vshifted_address;


reg                           dmem_valid;
reg   [ 31 : 0 ]              dmem_readdata_address;
reg                           cachedata_stillvalid;

reg [2:0] munit_state;
reg [2:0] issue_state;

reg  [ LOG2NUMLANES-1 : 0 ] vpid;
reg  [ NUMLANES-1 : 0 ] done;
wire doneall;

wire  [         NUMLANES-1 : 0 ]  _vreaddata_we;
reg   [ NUMPARALLELLANES-1 : 0 ]  _parhits;
wire  [ NUMPARALLELLANES-1 : 0 ]  parhits;
reg   [ NUMPARALLELLANES-1 : 0 ]  parhits_done;
wire  parhits_doneall;
wire  parhits_all;
wire  parhits_none;
wire  parhits_some;

wire  shifter_dirleft;
wire  shifter_load;
wire  shifter_shift;
wire  shifter_jump;
wire  shifter_jump_s1;

wire addr_sneakyload;
wire addr_advance;
wire addr_rewind;

wire do_quick_load;
reg quick_loaded;

reg  [ LOG2NUMLANES-1 : 0 ] sa_count;
wire [ LOG2NUMLANES-1 : 0 ] next_sa_count;
wire                        next_sa_count_zero;
wire                        sa_zero;
reg  [ NUMLANES*VPUWIDTH-1 : 0 ]  vshiftresult;

genvar i;
integer j;
genvar k;
integer l;
integer m;
integer n;
integer p;

  assign {op_memop,op_pattern,op_size,op_signed,op_we}=op;

  assign stall= (munit_state!=MUNIT_IDLE && munit_state!=MUNIT_DONE);

/************************** Pipeline load stage *******************************/
  reg enable_s2;
  reg last_subvector_s2;
  reg [ REGIDWIDTH-1 : 0 ] in_dst_s2;
  reg [      LOG2NUMLANES-1 : 0 ] sa_s2;
  reg dir_left_s2;
  reg  [1:0]  op_pattern_s2;      // 0-Unit, 1-Strided, 2-Indexed
  reg  [1:0]  op_size_s2;         // 0-byte, 1-16bits, 2-32bits, 3-64bits
  reg         op_signed_s2;       // 0-unsigned, 1-signed
  reg         op_we_s2;         // 0-load, 1-store
  reg         op_memop_s2;         // 1-memory op, 0-vector shift op

  always@(posedge clk)
    if (!resetn)
    begin
      enable_s2<=0;
      last_subvector_s2<=0;
      in_dst_s2<=0;
      sa_s2<=0;
      dir_left_s2<=0;
      {op_memop_s2,op_pattern_s2,op_size_s2,op_signed_s2,op_we_s2}<=0;
    end
    else if (!stall)
    begin
      enable_s2<=enable;
      last_subvector_s2<=last_subvector;
      if (en[0])
        in_dst_s2<=in_dst;
      sa_s2<=sa;
      dir_left_s2<=dir_left;
      {op_memop_s2,op_pattern_s2,op_size_s2,op_signed_s2,op_we_s2}<=op;
    end

/*************************** Vector op logic *********************************/

  assign next_sa_count=sa_count-1;
  assign next_sa_count_zero=(next_sa_count==0);
  assign sa_zero=(sa==0);

  always@(posedge clk)
  begin
    if (!resetn)
      sa_count<=0;
    else if (shifter_load)
      sa_count<=sa;
    else if (shifter_shift)
      sa_count<=next_sa_count;
  end

  // Detect double write one clock cycle early - also support 1 lane
  assign doublewrite=(~op_memop_s2) && (NUMLANES>1) && ( (dir_left_s2) ? 
              (|vshifted_mask[((NUMLANES>1) ? NUMLANES:2)-2:0]) && //support L=1
                (vshifted_mask[NUMLANES-1]&(|vshifted_masksave)) :
              (|vshifted_mask[NUMLANES-1:(NUMLANES>1) ? 1 : 0]) && //support L=1
                (vshifted_mask[0]&(|vshifted_masksave)));

  assign out_dst = {in_dst_s2[REGIDWIDTH-1:ELMIDWIDTH],
                    (munit_state!=MUNIT_XTRAWRITE) ? in_dst_s2[ELMIDWIDTH-1:0] :
                     (dir_left_s2) ? in_dst_s2[ELMIDWIDTH-1:0]+1'b1 : 
                       in_dst_s2[ELMIDWIDTH-1:0]-1'b1};

  //Truncate shifted result to VPW's size
  always@*
    for (p=0; p<NUMLANES; p=p+1)
      vshiftresult[p*VPUWIDTH +: VPUWIDTH]=
              vshifted_address[p*CONTROLWIDTH +: VPUWIDTH];

  assign voutput= (~enable_s2) ? 0 :
                  (op_memop_s2) ? vreaddata : 
                               vshiftresult;
 

  assign voutput_we=(op_memop_s2) ? vreaddata_we : 
                      (munit_state==MUNIT_XTRAWRITE) ? vshifted_masksave : 
                        (munit_state==MUNIT_DONE) ? vshifted_mask : 0;

/*************************** ISSUING LOGIC *********************************/

  assign shifter_load=enable && ~quick_loaded &&
                        (munit_state==MUNIT_IDLE || 
                         munit_state==MUNIT_DONE ||
                         do_quick_load);

  assign shifter_shift= (munit_state==MUNIT_SHIFT) || 
               ((munit_state==MUNIT_ISSUE) && parhits_doneall);

  assign shifter_dirleft=(op_memop_s2) ? 1'b0 : dir_left_s2;
  assign shifter_jump=op_memop_s2 && ~op_pattern_s2[1];
  assign shifter_jump_s1=op_memop && ~op_pattern[1];

  // Address Generator for each lane, or store vector to be shifted
  always@*
    for (m=0; m<NUMLANES; m=m+1)
      address[m*CONTROLWIDTH +: CONTROLWIDTH] = ((op_memop) ? cbase : 0) + 
            ( (op_pattern[1] || ~op_memop)  ? vindex[m*VPUWIDTH +: VPUWIDTH] : 
               (vstrideoffset[m*CONTROLWIDTH +: CONTROLWIDTH]<<op_size));

  wire [NUMLANES-1:0] vwritedata_shifter_squash_NC;
  velmshifter_jump #(NUMLANES,NUMPARALLELLANES,VPUWIDTH) vwritedatashifter(
      .clk(clk),
      .resetn(resetn),    //Don't use if not a store
      .load(shifter_load && op_we),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .jump(shifter_jump),
      .squash(vwritedata_shifter_squash_NC),
      .shiftin_left(0),
      .shiftin_right(0),
      .inpipe(vwritedata),
      .outpipe(vshifted_writedata));

  wire [NUMLANES-1:0] vaddress_shifter_squash_NC;
  velmshifter_jump #(NUMLANES,NUMPARALLELLANES,CONTROLWIDTH) vaddressshifter(
      .clk(clk),
      .resetn(resetn), //consider forcing to 0 if not used
      .load(shifter_load),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .jump(shifter_jump),
      .squash(vaddress_shifter_squash_NC),
      .shiftin_left(0),
      .shiftin_right(0),
      .inpipe( address),
      .outpipe(vshifted_address));

  velmshifter_jump #(NUMLANES,NUMPARALLELLANES,1) vmaskshifter(
      .clk(clk),
      .resetn(resetn),
      .load(shifter_load),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .jump(shifter_jump),
      .squash(0),
      .shiftin_left( 1'b0),
      .shiftin_right( 1'b0 ),
      .inpipe(vmask),
      .outpipe(vshifted_mask));

  //Save spilled over masks for shifting instructions (can only shift by 1)
  wire    [NUMLANES-1 : 0]  vmasks_save_inpipe_NC;
  velmshifter #(NUMLANES,1) vmaskssave(
      .clk(clk),
      .resetn(resetn && ~shifter_load),
      .load(1'b0),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .squash(0),
      .shiftin_left( vshifted_mask[0] ),
      .shiftin_right( vshifted_mask[NUMLANES-1] ),
      .inpipe(vmasks_save_inpipe_NC),
      .outpipe(vshifted_masksave));

  //Stride is maximum of stride specified and cache line size 
  assign stride_tmp = (((op_pattern[0]) ? cstride : 1) << op_size);
  always@(posedge clk)
    if (!resetn || last_subvector_s2&~stall&~shifter_load&~quick_loaded )
      stride<=0;
    else if (shifter_load)
      stride<= (stride_tmp>DMEM_READWIDTH/8) ? 
                      stride_tmp : DMEM_READWIDTH/8;

  wire [15:0] constantprefetch;
  assign constantprefetch=`VECTORPREFETCHES;
  assign t_cprefetch=cprefetch[15:0]<<(65535-`VECTORPREFETCHES);

  //***********************  Prefetch Logic **************************
  //
  //Send stride and vector length to bus.  To do constant prefetching set
  //stride to cache line size and send constant as length
  //******************************************************************
  always@(posedge clk)
    if (!resetn || last_subvector_s2&~stall&~shifter_load&~quick_loaded )
      prefetch<=0;
    else if (shifter_load)
      if (`VECTORPREFETCHES >= 65530)
        prefetch<= { stride_tmp[15:0], t_cprefetch[15:0] };
      else if (`VECTORPREFETCHES == 0)
        prefetch<= 0;
      else
        prefetch<= {16'd0+2**(LOG2DMEM_READWIDTH-3),constantprefetch};

  // State machine for issuing addresses - this is really sneaky!!!!!
  // The cache takes 1 cycle to respond to a cache request and is generally
  // not pipelined - but here we treat it as if it is pipeline by changing
  // the address during a single cache access.  
  //
  // We speculatively increment to the next cache line before we know if the
  // previous cache request hit or missed.  If it hits then we're fine and
  // we get pipelined outputs from the cache, but if not, we have to rewind 
  // our increment and wait for the data to come in.
  //
  // NOTE: We only do this for loads - writes write on the second clock
  always@(posedge clk)
  begin
    if (!resetn)
      issue_state<=ISSUE_IDLE;
    else
      case(issue_state)
        ISSUE_IDLE:
          issue_state<= (addr_sneakyload) ? 
                          ISSUE_INITIAL_INCREMENT: MUNIT_IDLE;
        ISSUE_INITIAL_INCREMENT:
          issue_state<= (doneall && ~addr_sneakyload) ? ISSUE_IDLE :
                      (doneall && addr_sneakyload) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_doneall && !doneall) ? ISSUE_INITIAL_INCREMENT :
                                            ISSUE_INCREMENTED;
        ISSUE_INCREMENTED:
          issue_state<= (doneall && ~addr_sneakyload) ? ISSUE_IDLE :
                      (doneall && addr_sneakyload) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_doneall) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_none) ? ISSUE_WAITONE : ISSUE_INCREMENTED;
        // This stage ignores the speculated result following a miss
        ISSUE_WAITONE:      
          issue_state<= ISSUE_WAITING;
        ISSUE_WAITING:
          issue_state<= (doneall && ~addr_sneakyload) ? ISSUE_IDLE :
                      (doneall && addr_sneakyload) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_doneall) ? ISSUE_INITIAL_INCREMENT :
                      (parhits_some) ? ISSUE_INITIAL_INCREMENT : ISSUE_WAITING;
      endcase
  end

  assign addr_sneakyload=(shifter_load && op_memop && ~op_we && ~op_pattern[1] && enable);
                  //(shifter_shift && ~shifter_load && ~op_we_s2 && enable_s2));

  assign addr_advance=(issue_state==ISSUE_INITIAL_INCREMENT) ||
                      (issue_state==ISSUE_INCREMENTED && parhits_some) ||
                      (issue_state==ISSUE_WAITING && parhits_some) ||
                      (op_we_s2 && parhits_some && ~cachedata_stillvalid);

  assign addr_rewind=(issue_state==ISSUE_INCREMENTED && parhits_none);

  always@(posedge clk)
    if (!resetn || shifter_load || last_subvector_s2&~stall&~shifter_load )
      vpid<=0;
    else if (shifter_shift)
      vpid<=vpid + ((shifter_jump) ? NUMPARALLELLANES : 1'b1);

  // Loads in a new request if it is a memory request. quick_loaded is high 
  // the cycle after the new request has been loaded and instructs the
  // logic below to continue requesting from memory (even though munit_state
  // will be in the DONE state).
  // Note we don't do quickload on last subvector - a speculative miss can 
  // cause the quickloaded memory instruction to re-load its cache line from
  // memory - but if that line was dirty we would have missed the changes.
    // I don't think this is true anymore PY 07/2008
  assign do_quick_load=doneall&enable_s2&op_memop_s2&~op_we_s2&
                       ~last_subvector_s2&enable&op_memop&~op_we&~op_pattern[1];
  always@(posedge clk)
    quick_loaded<=shifter_load&do_quick_load;

  assign dmem_en = ((shifter_jump) ? (|vshifted_mask[NUMPARALLELLANES-1:0]) : 
                                      vshifted_mask[0]) && 
                    (munit_state==MUNIT_ISSUE || quick_loaded);
  assign dmem_we=op_we_s2;

  /*********************
  * We don't want cache miss signal to be an add/sub select signal or else it
  * will propagate through the adder.  We perform the addition/subtraction
  * in parallel with the miss calculation and use the miss as a mux select
  *********************/
  wire [ CONTROLWIDTH-1 : 0 ] dmem_address_next /* synthesis keep */;
  wire [ CONTROLWIDTH-1 : 0 ] dmem_address_prev /* synthesis keep */;
  assign dmem_address_next=dmem_address + stride;
  assign dmem_address_prev=dmem_address - stride;

  always@(posedge clk)
    if (!resetn)
      dmem_address<=0;
    else if (shifter_load)
      dmem_address<=address[CONTROLWIDTH-1:0];
    else if (shifter_shift)
      dmem_address<= (!shifter_jump) ? 
              vshifted_address[((NUMLANES>1) ? 1:0)
                *CONTROLWIDTH +: CONTROLWIDTH] :
              vshifted_address[
                ((NUMLANES<=NUMPARALLELLANES) ? 0 : NUMPARALLELLANES)
                  *CONTROLWIDTH +: CONTROLWIDTH];
    //Fetch next cache line if partial match on initial cache access
    //else if (parhits_some && ~cachedata_stillvalid)
    //else if (addr_advance)
    //  dmem_address<=dmem_address_next;
    //else if (addr_rewind)
    //  dmem_address<=dmem_address_prev;

  always@(posedge clk)
    if (!resetn)
      dmem_prefetch<=0;
    //else if (shifter_load)
    else
      dmem_prefetch<=prefetch[CONTROLWIDTH-1:0];

/*************************** Mem Write LOGIC ********************************/

    // Generate byte/halfword alignment circuitry for each word
  generate
  for (i=0; i<NUMPARALLELLANES; i=i+1)
  begin : write_gen
    vstore_data_translator vstore_data_translator(
      //Pad vshifted_writedata with zeros incase signal is less than 32-bits
      .write_data({32'b0,vshifted_writedata[i*VPUWIDTH +: VPUWIDTH]}),
      .d_address(vshifted_address[i*32 +: 2]),
      .store_size(op_size_s2), 
      .d_byteena(_vbyteen[4*i +: 4]),  
      .d_writedataout(_vwritedata[i*32 +: 32]));
  end
  endgenerate

  always@*
  begin
    dmem_writedata=0;
    dmem_byteen=0;
    for (l=0; l<NUMPARALLELLANES; l=l+1)
      if (dmem_address[31:LOG2DMEM_WRITEWIDTH-3] == 
          vshifted_address[32*l+LOG2DMEM_WRITEWIDTH-3 +: 
                            32-LOG2DMEM_WRITEWIDTH+3])
      begin
        dmem_writedata=dmem_writedata| (_vwritedata[l*32 +: 32] << 
            {vshifted_address[32*l+2 +: LOG2DMEM_WRITEWIDTH-5], {5{1'b0}}});
        if (vshifted_mask[l] && (shifter_jump || (l==0)))
          dmem_byteen=dmem_byteen | (_vbyteen[4*l+:4]<<
            {vshifted_address[32*l+2 +: LOG2DMEM_WRITEWIDTH-5], {2{1'b0}}});
      end
  end

/*************************** Mem Receive LOGIC ********************************/
  reg wastedcache;  //Identifies when a cache request is made but then aborted because the data was found in the previous cache request which is "stillvalid"

  // This is specific to cache timing - 1 cycle
  always@(posedge clk)
    if (!resetn) 
    begin
      dmem_valid<=0;
      dmem_readdata_address<=0;
      wastedcache<=0;
      cachedata_stillvalid<=0;
    end
    else
    begin
      dmem_valid<=dmem_en;
      if (!wastedcache)
        dmem_readdata_address<=dmem_address;
      //WE know for sure data is still valid after the cache hit since it takes
      //two clock cyles to get new cacheline
      //wastedcache<=shifter_shift&cachedata_stillvalid;
      //cachedata_stillvalid<=dmem_valid&~dmem_wait;
      //Disable this since now we speculatively advance the address
      wastedcache<=0;
      cachedata_stillvalid<=0;
    end

  // Find out which parallel lanes hit in the cache
  always@*
    for (j=0; j<NUMPARALLELLANES; j=j+1)
      //This signal tells us a cache request succeeded (for either load/store)
      //Each bit corresponds to a parallel lane
      _parhits[j]= vshifted_mask[j] &&
          ((dmem_valid&(dmem_cachematch&~op_we_s2 || op_we_s2&~dmem_wait)) || 
              (cachedata_stillvalid&~op_we_s2)) &&
            (vshifted_address[j*32+LOG2DMEM_READWIDTH-3 +: 
                             32-LOG2DMEM_READWIDTH+3] ==
               dmem_readdata_address[31:LOG2DMEM_READWIDTH-3]);

  //For operations that don't jump, just look at first bit of _parhits
  assign parhits=(shifter_jump) ? _parhits : {NUMPARALLELLANES{_parhits[0]}};

  // Detect all parallel lanes hitting
  assign parhits_all=&(parhits|~vshifted_mask[NUMPARALLELLANES-1:0]);
  // Detect cache misses
  assign parhits_none=~|(parhits);
  // Detect some misses - fetch next cache line
  assign parhits_some=~parhits_none && ~parhits_all;

  //If NUMLANES<=NUMPARALLELLANES then we will never do a jump, so we make
  //compiler ignore this statement if that's the case
  always@(posedge clk)
    if (!resetn || shifter_load)
      parhits_done<= (shifter_jump_s1) ?  ~vmask : {NUMPARALLELLANES{~vmask[0]}};
    else if ( parhits_doneall )
      parhits_done<= (shifter_jump && (NUMLANES>NUMPARALLELLANES)) ? 
                    ~vshifted_mask[ 
                        ((NUMLANES>NUMPARALLELLANES) ? NUMPARALLELLANES : 0) 
                      +: NUMPARALLELLANES] :
                    {NUMPARALLELLANES{(NUMLANES>1) ? ~vshifted_mask[1] : 1'b0}};
    else           
      parhits_done<= parhits_done|parhits;

  assign parhits_doneall=&(parhits|parhits_done);

  assign _vreaddata_we= ((shifter_jump) ? _parhits : _parhits[0]) 
                          << vpid[LOG2NUMLANES-1:0];

  always@(posedge clk)
    if (!resetn || ~enable_s2)  // Force to zero if unit not used
      vreaddata_we<= 0 ;
    else           // write to regfile if a) is a load, b) we haven't already
      vreaddata_we<= {NUMLANES{~op_we_s2}} & _vreaddata_we & ~done;

  //Select signal for crossbar either uses bits from corresponding parallel
  //lane address or all selects are set to the same if we're not jumping
  always@*
    for (n=0; n<NUMPARALLELLANES; n=n+1)
      crossbar_sel[(LOG2DMEM_READWIDTH-5)*n +: (LOG2DMEM_READWIDTH-5)]=
            (!shifter_jump) ? 
              {NUMPARALLELLANES{vshifted_address[2 +: LOG2DMEM_READWIDTH-5]}} :
              vshifted_address[n*CONTROLWIDTH+2 +: (LOG2DMEM_READWIDTH-5)];

  vmem_crossbar vmem_crossbar(
      .clk(), .resetn(),
      .sel(crossbar_sel),
      .in(dmem_readdata),
      .out(crossbar));
    defparam vmem_crossbar.INWIDTH=DMEM_READWIDTH,
             vmem_crossbar.LOG2INWIDTH=LOG2DMEM_READWIDTH,
             vmem_crossbar.NUMOUTS=NUMPARALLELLANES,
             vmem_crossbar.OUTWIDTH=32,
             vmem_crossbar.LOG2OUTWIDTH=5;

    // Generate byte/halfword alignment circuitry for each word
  generate
  for (k=0; k<NUMPARALLELLANES; k=k+1)
  begin : load_gen
    vload_data_translator load_data_translator(
      .d_readdatain(crossbar[32*(k+1)-1:32*k]),
      .d_address( (shifter_jump) ? vshifted_address[CONTROLWIDTH*k +: 2] :
                                   vshifted_address[1:0]),
      .load_size(op_size_s2[1:0]),
      .load_sign_ext(op_signed_s2),
      .d_loadresult(__vreaddata[VPUWIDTH*(k+1)-1:VPUWIDTH*k])
      );
  end
  endgenerate

  always@(posedge clk)
    //zero if unit not used
    if (!resetn || ~enable_s2 || (~stall&~enable) || op_we_s2)
      vreaddata<= 0 ;
    else                // Don't write to regfile unless this is a load!
      vreaddata<= {NUMLANES/NUMPARALLELLANES{__vreaddata}};

/*************************** DONE LOGIC *********************************/

  // Done register has 1 bit for each lane, we are finished when done is all 1's
  always@(posedge clk)
    if (!resetn)
      done<=0;
    //Don't check for MUNIT_DONE, because of quickload we are reading then
    //else if (shifter_load || (munit_state==MUNIT_DONE) )
    else if (shifter_load || doneall || (munit_state==MUNIT_DONE && (op_we_s2 || op_pattern_s2[1])))
      done<=~vmask;
    else 
      done<=done|_vreaddata_we;

  assign doneall=(&(done|_vreaddata_we));

  always@(posedge clk)
  begin
    if (!resetn)
      munit_state<=MUNIT_IDLE;
    else
      case(munit_state)
        MUNIT_IDLE:
          munit_state<= (enable) ? (op_memop) ? MUNIT_ISSUE : 
                                      (sa_zero) ? MUNIT_DONE : MUNIT_SHIFT : 
                                   MUNIT_IDLE;
        MUNIT_ISSUE:
          munit_state<= doneall ? MUNIT_DONE : MUNIT_ISSUE;
        MUNIT_SHIFT:
          munit_state<= (next_sa_count_zero) ? 
                            (doublewrite) ? MUNIT_XTRAWRITE : MUNIT_DONE : 
                            MUNIT_SHIFT;
        MUNIT_XTRAWRITE:  //Spilled over subvector bounds when shifting 
          munit_state<= MUNIT_DONE;
        MUNIT_DONE:
          munit_state<= (enable) ? (op_memop) ? MUNIT_ISSUE : 
                                      (sa_zero) ? MUNIT_DONE : MUNIT_SHIFT : 
                                   MUNIT_IDLE;
        default:
          munit_state<=MUNIT_IDLE;
      endcase
  end

  pipereg #(1) dstwepipe (
    .d( in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .squashn(1'b1),
    .en( en[0] && ~stall ),
    .q(out_dst_we));

  pipereg #(1) vsdstwepipe (
    .d( in_vs_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .squashn(1'b1),
    .en( en[0] && ~stall ),
    .q(out_vs_dst_we));

endmodule

/****************************************************************************
          Store data translator
          - moves store data to appropriate byte/halfword 
****************************************************************************/
module vstore_data_translator(
    write_data,             // data in least significant position
    d_address,
    store_size,             // 0-byte, 1-16bits, 2-32bits, 3-64bits
    d_byteena,
    d_writedataout);        // shifted data to coincide with address
parameter WIDTH=32;

input [WIDTH-1:0] write_data;
input [1:0] d_address;
input [1:0] store_size;
output [3:0] d_byteena;
output [WIDTH-1:0] d_writedataout;

reg [3:0] d_byteena;
reg [WIDTH-1:0] d_writedataout;

always @*
begin
    case (store_size)
        2'b00:
        begin
            case(d_address[1:0])
                2'b00: begin d_byteena=4'b1000; 
                        d_writedataout={write_data[7:0],24'b0}; end
                2'b01: begin d_byteena=4'b0100;
                        d_writedataout={8'b0,write_data[7:0],16'b0}; end
                2'b10: begin d_byteena=4'b0010;
                        d_writedataout={16'b0,write_data[7:0],8'b0}; end
                default: begin d_byteena=4'b0001;
                        d_writedataout={24'b0,write_data[7:0]}; end
            endcase
        end
        2'b01:
        begin
            d_writedataout=(d_address[1]) ? {16'b0,write_data[15:0]} : 
                                            {write_data[15:0],16'b0};
            d_byteena=(d_address[1]) ? 4'b0011 : 4'b1100 ;
        end
        default:
        begin
            d_byteena=4'b1111;
            d_writedataout=write_data;
        end
    endcase
end

endmodule

/****************************************************************************
 *           Load data translator
 *- moves read data to appropriate byte/halfword and zero/sign extends
 *****************************************************************************/
module vload_data_translator(
    d_readdatain,
    d_address,
    load_size,
    load_sign_ext,
    d_loadresult);
parameter WIDTH=32;

input [WIDTH-1:0] d_readdatain;
input [1:0] d_address;
input [1:0] load_size;
input load_sign_ext;
output [WIDTH-1:0] d_loadresult;

reg [WIDTH-1:0] d_loadresult;

always @(d_readdatain or d_address or load_size or load_sign_ext)
begin
    case (load_size)
        2'b00:
        begin
            case (d_address[1:0])
                2'd0: d_loadresult[7:0]=d_readdatain[31:24];
                2'd1: d_loadresult[7:0]=d_readdatain[23:16];
                2'd2: d_loadresult[7:0]=d_readdatain[15:8];
                default: d_loadresult[7:0]=d_readdatain[7:0];
            endcase
            d_loadresult[31:8]={24{load_sign_ext&d_loadresult[7]}};
        end
        2'b01:
        begin
            case (d_address[1])
                2'd0: d_loadresult[15:0]=d_readdatain[31:16];
                default: d_loadresult[15:0]=d_readdatain[15:0];
            endcase
            d_loadresult[31:16]={16{load_sign_ext&d_loadresult[15]}};
        end
        default:
            d_loadresult=d_readdatain;
    endcase
end

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vmem_unit.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: vlane_flagalu.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vlane_flagalu.v
//////////////////////////////////////////////////////////////////////
/******************************************************************************

0  VFAND
1  VFOR
2  VFXOR
3  VFNOR
4  VFCLR
5  VFSET

Not handled here:
  VIOTA
  VCIOTA
  VFPOP
  VFFF1
  VFFL1
  VFSETBF
  VFSETIF
  VFSETOF

******************************************************************************/

module vlane_flagalu (
    clk,
    resetn,

    src1,
    src2,

    op,

    result

    );

input clk;
input resetn;

input src1;
input src2;
input [2:0] op;

output result;

reg result;

  always@*
    case(op)
      0: result=src1&src2;
      1: result=src1|src2;
      2: result=src1^src2;
      3: result=~(src1|src2);
      4: result=1'b0;
      5: result=1'b1;
      default: result=1'b0;
    endcase

endmodule

//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vlane_flagalu.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: matmul_unit.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/top/matmul_unit.v
//////////////////////////////////////////////////////////////////////

`define DWIDTH 16
`define AWIDTH 10
`define MEM_SIZE 1024

///////////////////////////////////////////////////////////
//MAT_MUL_SIZE refers to the dimension of the matrix
//multiplication performed by the matmul unit. The value 
//8 means it will multiply an 8x8 matrix with an 8x8 matrix.
///////////////////////////////////////////////////////////
//IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP
///////////////////////////////////////////////////////////
//MAT_MUL_SIZE should be equal to number of lanes in the vector processor
///////////////////////////////////////////////////////////
//IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP IMP
///////////////////////////////////////////////////////////

`define MASK_WIDTH 8
`define LOG2_MAT_MUL_SIZE 3

`define BB_MAT_MUL_SIZE `MAT_MUL_SIZE

`define NUM_CYCLES_IN_MAC 3
`define MEM_ACCESS_LATENCY 1
`define REG_DATAWIDTH 32
`define REG_ADDRWIDTH 8
`define ADDR_STRIDE_WIDTH 16
`define MAX_BITS_POOL 3

`define PIPE_STAGES_MATMUL 29

//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: matmul_8x8.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/top/matmul_8x8.v
//////////////////////////////////////////////////////////////////////


module matmul_8x8(
 clk,
 //Reset for the whole matmul
 reset,
 //Reset for the PEs. Typically connected to 'reset'
 pe_reset,
 //When this is asserted, the matmul operation starts. This can remain
 //asserted during the execution, but is not required to.
 start, 
 //This is asserted a cycle after the matmul start operation. It stays
 //asserted until the matmul starts shifting out the output data.
 in_progress,
 //This is not used any more. But it is a pulse, and is asserted when
 //the execution (including output shift out) is finished.
 done,
 //Input data matrix A. For MAT_MUL_SIZE=8, 64 values come in over 8 cycles.
 //8 values in one cycle.
 a_data,
 //Input data matrix B. For MAT_MUL_SIZE=8, 64 values come in over 8 cycles.
 //8 values in one cycle.
 b_data,
 //Output data matrix C. For MAT_MUL_SIZE=8, 64 values come out over 8 cycles.
 //8 values in one cycle.
 c_data_out, 
 //This isn't used anymore. It stays asserted when the output data matrix C
 //is being shifted out.
 c_data_available,
 //Masks for input matrices A and B. These are used when we want to use this 
 //unit to multiply matrices that are less than 8x8 (eg: 6x4 with a 4x5 matrix).
 validity_mask_a_rows,
 validity_mask_a_cols,
 validity_mask_b_rows,
 validity_mask_b_cols
);

 input clk;
 input reset;
 input pe_reset;
 input start;
 output in_progress;
 output done;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data;
 output [`MAT_MUL_SIZE*`DWIDTH-1:0] c_data_out;
 output c_data_available;

 input [`MASK_WIDTH-1:0] validity_mask_a_rows;
 input [`MASK_WIDTH-1:0] validity_mask_a_cols;
 input [`MASK_WIDTH-1:0] validity_mask_b_rows;
 input [`MASK_WIDTH-1:0] validity_mask_b_cols;

//////////////////////////////////////////////////////////////////////////
// Logic for clock counting and when to assert done
//////////////////////////////////////////////////////////////////////////
reg in_progress;
reg done_mat_mul;
wire matmul_op_in_progress;
reg shift_out_data;
reg shift_in_data;
wire start_pulse;
wire [7:0] clk_cnt_for_done;
wire [7:0] clk_cnt_for_latching_c_data;
wire [7:0] clk_cnt_for_shifting_inputs;

assign done = done_mat_mul;

reg start_delayed;
always @(posedge clk) begin
  if (reset) begin
    start_delayed <= 0;
  end 
  else begin
    start_delayed <= start;
  end
end

assign start_pulse = start & (~start_delayed);

//This signal is used in other modules instantiated in this design.
//It stays high during the entire time matmul is working.
//                            _
//start_pulse           _____| |___________________________________________
//                              ___
//shift_in_data         _______|   |_______________________________________
//                                   ________________________
//in_progress           ____________|                        |______________
//                                                            _______
//shift_out_data        _____________________________________|       |______
//                            _______________________________________
//matmul_op_in_progress _____|                                       |______
assign matmul_op_in_progress = start_pulse|shift_in_data|in_progress|shift_out_data;

//This is 7 bits because the expectation is that clock count will be pretty
//small. For large matmuls, this will need to increased to have more bits.
//In general, a systolic multiplier takes 4*N-2+P cycles, where N is the size 
//of the matmul and P is the number of pipleine stages in the MAC block.
reg [7:0] clk_cnt;

//Number of cycles to assert matmul done. This includes the cycles to shift out the results. 
//This is hardcoded here, because this was generated using a script.
assign clk_cnt_for_done = 
                          (34);  

//Number of cycles at which we latch the output data and start shifting it out.
assign clk_cnt_for_latching_c_data =                        
                          (27);  

//Number of cycles at which we finish shifting inputs into the matmul.
//Note that while this shifting is happening, the matmul is calculating
//outputs in its PEs.
assign clk_cnt_for_shifting_inputs =                        
                          (1);  //Ideally this should have been 7, but if we keep this as
                                //7, then stall signal is asserted a bit later than required

always @(posedge clk) begin
  if (reset) begin
    clk_cnt <= 0;
    done_mat_mul <= 0;
    in_progress <= 0;
    shift_out_data <= 0;
    shift_in_data <= 0;
  end
  else if (start_pulse == 1'b1) begin
    clk_cnt <= clk_cnt + 1;
    done_mat_mul <= 0;
    in_progress <= 0;
    shift_out_data <= 0;
    shift_in_data <= 1;
  end
  else if (clk_cnt == clk_cnt_for_shifting_inputs) begin 
    clk_cnt <= clk_cnt + 1;
    done_mat_mul <= 0;
    in_progress <= 1;
    shift_out_data <= 0;
    shift_in_data <= 0;
  end
  else if (clk_cnt == clk_cnt_for_latching_c_data) begin 
    clk_cnt <= clk_cnt + 1;
    done_mat_mul <= 0;
    in_progress <= 0;
    shift_out_data <= 1;
    shift_in_data <= 0;
  end
  else if (clk_cnt == clk_cnt_for_done) begin
    done_mat_mul <= 1;
    clk_cnt <= clk_cnt + 1;
    in_progress <= 0;
    shift_out_data <= 0;
    shift_in_data <= 0;
  end
  else if ((in_progress == 1) || (shift_out_data == 1) || (shift_in_data == 1)) begin
    clk_cnt <= clk_cnt + 1;
  end    
  else begin
    clk_cnt <= 0;
    done_mat_mul <= 0;
    in_progress <= 0;
  end
end

wire [`DWIDTH-1:0] a0_data;
wire [`DWIDTH-1:0] a1_data;
wire [`DWIDTH-1:0] a2_data;
wire [`DWIDTH-1:0] a3_data;
wire [`DWIDTH-1:0] a4_data;
wire [`DWIDTH-1:0] a5_data;
wire [`DWIDTH-1:0] a6_data;
wire [`DWIDTH-1:0] a7_data;
wire [`DWIDTH-1:0] b0_data;
wire [`DWIDTH-1:0] b1_data;
wire [`DWIDTH-1:0] b2_data;
wire [`DWIDTH-1:0] b3_data;
wire [`DWIDTH-1:0] b4_data;
wire [`DWIDTH-1:0] b5_data;
wire [`DWIDTH-1:0] b6_data;
wire [`DWIDTH-1:0] b7_data;
wire [`DWIDTH-1:0] a1_data_delayed_1;
wire [`DWIDTH-1:0] a2_data_delayed_1;
wire [`DWIDTH-1:0] a2_data_delayed_2;
wire [`DWIDTH-1:0] a3_data_delayed_1;
wire [`DWIDTH-1:0] a3_data_delayed_2;
wire [`DWIDTH-1:0] a3_data_delayed_3;
wire [`DWIDTH-1:0] a4_data_delayed_1;
wire [`DWIDTH-1:0] a4_data_delayed_2;
wire [`DWIDTH-1:0] a4_data_delayed_3;
wire [`DWIDTH-1:0] a4_data_delayed_4;
wire [`DWIDTH-1:0] a5_data_delayed_1;
wire [`DWIDTH-1:0] a5_data_delayed_2;
wire [`DWIDTH-1:0] a5_data_delayed_3;
wire [`DWIDTH-1:0] a5_data_delayed_4;
wire [`DWIDTH-1:0] a5_data_delayed_5;
wire [`DWIDTH-1:0] a6_data_delayed_1;
wire [`DWIDTH-1:0] a6_data_delayed_2;
wire [`DWIDTH-1:0] a6_data_delayed_3;
wire [`DWIDTH-1:0] a6_data_delayed_4;
wire [`DWIDTH-1:0] a6_data_delayed_5;
wire [`DWIDTH-1:0] a6_data_delayed_6;
wire [`DWIDTH-1:0] a7_data_delayed_1;
wire [`DWIDTH-1:0] a7_data_delayed_2;
wire [`DWIDTH-1:0] a7_data_delayed_3;
wire [`DWIDTH-1:0] a7_data_delayed_4;
wire [`DWIDTH-1:0] a7_data_delayed_5;
wire [`DWIDTH-1:0] a7_data_delayed_6;
wire [`DWIDTH-1:0] a7_data_delayed_7;
wire [`DWIDTH-1:0] b1_data_delayed_1;
wire [`DWIDTH-1:0] b2_data_delayed_1;
wire [`DWIDTH-1:0] b2_data_delayed_2;
wire [`DWIDTH-1:0] b3_data_delayed_1;
wire [`DWIDTH-1:0] b3_data_delayed_2;
wire [`DWIDTH-1:0] b3_data_delayed_3;
wire [`DWIDTH-1:0] b4_data_delayed_1;
wire [`DWIDTH-1:0] b4_data_delayed_2;
wire [`DWIDTH-1:0] b4_data_delayed_3;
wire [`DWIDTH-1:0] b4_data_delayed_4;
wire [`DWIDTH-1:0] b5_data_delayed_1;
wire [`DWIDTH-1:0] b5_data_delayed_2;
wire [`DWIDTH-1:0] b5_data_delayed_3;
wire [`DWIDTH-1:0] b5_data_delayed_4;
wire [`DWIDTH-1:0] b5_data_delayed_5;
wire [`DWIDTH-1:0] b6_data_delayed_1;
wire [`DWIDTH-1:0] b6_data_delayed_2;
wire [`DWIDTH-1:0] b6_data_delayed_3;
wire [`DWIDTH-1:0] b6_data_delayed_4;
wire [`DWIDTH-1:0] b6_data_delayed_5;
wire [`DWIDTH-1:0] b6_data_delayed_6;
wire [`DWIDTH-1:0] b7_data_delayed_1;
wire [`DWIDTH-1:0] b7_data_delayed_2;
wire [`DWIDTH-1:0] b7_data_delayed_3;
wire [`DWIDTH-1:0] b7_data_delayed_4;
wire [`DWIDTH-1:0] b7_data_delayed_5;
wire [`DWIDTH-1:0] b7_data_delayed_6;
wire [`DWIDTH-1:0] b7_data_delayed_7;


//////////////////////////////////////////////////////////////////////////
// Instantiation of systolic data setup
//////////////////////////////////////////////////////////////////////////
systolic_data_setup u_systolic_data_setup(
.clk(clk),
.reset(reset),
.matmul_op_in_progress(matmul_op_in_progress),
.a_data(a_data),
.b_data(b_data),
.clk_cnt(clk_cnt),
.a0_data(a0_data),
.b0_data(b0_data),
.a1_data_delayed_1(a1_data_delayed_1),
.b1_data_delayed_1(b1_data_delayed_1),
.a2_data_delayed_2(a2_data_delayed_2),
.b2_data_delayed_2(b2_data_delayed_2),
.a3_data_delayed_3(a3_data_delayed_3),
.b3_data_delayed_3(b3_data_delayed_3),
.a4_data_delayed_4(a4_data_delayed_4),
.b4_data_delayed_4(b4_data_delayed_4),
.a5_data_delayed_5(a5_data_delayed_5),
.b5_data_delayed_5(b5_data_delayed_5),
.a6_data_delayed_6(a6_data_delayed_6),
.b6_data_delayed_6(b6_data_delayed_6),
.a7_data_delayed_7(a7_data_delayed_7),
.b7_data_delayed_7(b7_data_delayed_7),

.validity_mask_a_rows(validity_mask_a_rows),
.validity_mask_a_cols(validity_mask_a_cols),
.validity_mask_b_rows(validity_mask_b_rows),
.validity_mask_b_cols(validity_mask_b_cols)

);

wire [`DWIDTH-1:0] a0;
wire [`DWIDTH-1:0] a1;
wire [`DWIDTH-1:0] a2;
wire [`DWIDTH-1:0] a3;
wire [`DWIDTH-1:0] a4;
wire [`DWIDTH-1:0] a5;
wire [`DWIDTH-1:0] a6;
wire [`DWIDTH-1:0] a7;
wire [`DWIDTH-1:0] b0;
wire [`DWIDTH-1:0] b1;
wire [`DWIDTH-1:0] b2;
wire [`DWIDTH-1:0] b3;
wire [`DWIDTH-1:0] b4;
wire [`DWIDTH-1:0] b5;
wire [`DWIDTH-1:0] b6;
wire [`DWIDTH-1:0] b7;

assign a0 = a0_data          ;
assign a1 = a1_data_delayed_1;
assign a2 = a2_data_delayed_2;
assign a3 = a3_data_delayed_3;
assign a4 = a4_data_delayed_4;
assign a5 = a5_data_delayed_5;
assign a6 = a6_data_delayed_6;
assign a7 = a7_data_delayed_7;

assign b0 = b0_data          ;
assign b1 = b1_data_delayed_1;
assign b2 = b2_data_delayed_2;
assign b3 = b3_data_delayed_3;
assign b4 = b4_data_delayed_4;
assign b5 = b5_data_delayed_5;
assign b6 = b6_data_delayed_6;
assign b7 = b7_data_delayed_7;

wire [`DWIDTH-1:0] matrixC0_0;
wire [`DWIDTH-1:0] matrixC0_1;
wire [`DWIDTH-1:0] matrixC0_2;
wire [`DWIDTH-1:0] matrixC0_3;
wire [`DWIDTH-1:0] matrixC0_4;
wire [`DWIDTH-1:0] matrixC0_5;
wire [`DWIDTH-1:0] matrixC0_6;
wire [`DWIDTH-1:0] matrixC0_7;
wire [`DWIDTH-1:0] matrixC1_0;
wire [`DWIDTH-1:0] matrixC1_1;
wire [`DWIDTH-1:0] matrixC1_2;
wire [`DWIDTH-1:0] matrixC1_3;
wire [`DWIDTH-1:0] matrixC1_4;
wire [`DWIDTH-1:0] matrixC1_5;
wire [`DWIDTH-1:0] matrixC1_6;
wire [`DWIDTH-1:0] matrixC1_7;
wire [`DWIDTH-1:0] matrixC2_0;
wire [`DWIDTH-1:0] matrixC2_1;
wire [`DWIDTH-1:0] matrixC2_2;
wire [`DWIDTH-1:0] matrixC2_3;
wire [`DWIDTH-1:0] matrixC2_4;
wire [`DWIDTH-1:0] matrixC2_5;
wire [`DWIDTH-1:0] matrixC2_6;
wire [`DWIDTH-1:0] matrixC2_7;
wire [`DWIDTH-1:0] matrixC3_0;
wire [`DWIDTH-1:0] matrixC3_1;
wire [`DWIDTH-1:0] matrixC3_2;
wire [`DWIDTH-1:0] matrixC3_3;
wire [`DWIDTH-1:0] matrixC3_4;
wire [`DWIDTH-1:0] matrixC3_5;
wire [`DWIDTH-1:0] matrixC3_6;
wire [`DWIDTH-1:0] matrixC3_7;
wire [`DWIDTH-1:0] matrixC4_0;
wire [`DWIDTH-1:0] matrixC4_1;
wire [`DWIDTH-1:0] matrixC4_2;
wire [`DWIDTH-1:0] matrixC4_3;
wire [`DWIDTH-1:0] matrixC4_4;
wire [`DWIDTH-1:0] matrixC4_5;
wire [`DWIDTH-1:0] matrixC4_6;
wire [`DWIDTH-1:0] matrixC4_7;
wire [`DWIDTH-1:0] matrixC5_0;
wire [`DWIDTH-1:0] matrixC5_1;
wire [`DWIDTH-1:0] matrixC5_2;
wire [`DWIDTH-1:0] matrixC5_3;
wire [`DWIDTH-1:0] matrixC5_4;
wire [`DWIDTH-1:0] matrixC5_5;
wire [`DWIDTH-1:0] matrixC5_6;
wire [`DWIDTH-1:0] matrixC5_7;
wire [`DWIDTH-1:0] matrixC6_0;
wire [`DWIDTH-1:0] matrixC6_1;
wire [`DWIDTH-1:0] matrixC6_2;
wire [`DWIDTH-1:0] matrixC6_3;
wire [`DWIDTH-1:0] matrixC6_4;
wire [`DWIDTH-1:0] matrixC6_5;
wire [`DWIDTH-1:0] matrixC6_6;
wire [`DWIDTH-1:0] matrixC6_7;
wire [`DWIDTH-1:0] matrixC7_0;
wire [`DWIDTH-1:0] matrixC7_1;
wire [`DWIDTH-1:0] matrixC7_2;
wire [`DWIDTH-1:0] matrixC7_3;
wire [`DWIDTH-1:0] matrixC7_4;
wire [`DWIDTH-1:0] matrixC7_5;
wire [`DWIDTH-1:0] matrixC7_6;
wire [`DWIDTH-1:0] matrixC7_7;

wire row_latch_en;
//////////////////////////////////////////////////////////////////////////
// Instantiation of the output logic
//////////////////////////////////////////////////////////////////////////
output_logic u_output_logic(
.matmul_op_in_progress(matmul_op_in_progress),
.done_mat_mul(done_mat_mul),
.c_data_out(c_data_out),
.c_data_available(c_data_available),
.clk_cnt(clk_cnt),
.row_latch_en(row_latch_en),
.matrixC0_0(matrixC0_0),
.matrixC0_1(matrixC0_1),
.matrixC0_2(matrixC0_2),
.matrixC0_3(matrixC0_3),
.matrixC0_4(matrixC0_4),
.matrixC0_5(matrixC0_5),
.matrixC0_6(matrixC0_6),
.matrixC0_7(matrixC0_7),
.matrixC1_0(matrixC1_0),
.matrixC1_1(matrixC1_1),
.matrixC1_2(matrixC1_2),
.matrixC1_3(matrixC1_3),
.matrixC1_4(matrixC1_4),
.matrixC1_5(matrixC1_5),
.matrixC1_6(matrixC1_6),
.matrixC1_7(matrixC1_7),
.matrixC2_0(matrixC2_0),
.matrixC2_1(matrixC2_1),
.matrixC2_2(matrixC2_2),
.matrixC2_3(matrixC2_3),
.matrixC2_4(matrixC2_4),
.matrixC2_5(matrixC2_5),
.matrixC2_6(matrixC2_6),
.matrixC2_7(matrixC2_7),
.matrixC3_0(matrixC3_0),
.matrixC3_1(matrixC3_1),
.matrixC3_2(matrixC3_2),
.matrixC3_3(matrixC3_3),
.matrixC3_4(matrixC3_4),
.matrixC3_5(matrixC3_5),
.matrixC3_6(matrixC3_6),
.matrixC3_7(matrixC3_7),
.matrixC4_0(matrixC4_0),
.matrixC4_1(matrixC4_1),
.matrixC4_2(matrixC4_2),
.matrixC4_3(matrixC4_3),
.matrixC4_4(matrixC4_4),
.matrixC4_5(matrixC4_5),
.matrixC4_6(matrixC4_6),
.matrixC4_7(matrixC4_7),
.matrixC5_0(matrixC5_0),
.matrixC5_1(matrixC5_1),
.matrixC5_2(matrixC5_2),
.matrixC5_3(matrixC5_3),
.matrixC5_4(matrixC5_4),
.matrixC5_5(matrixC5_5),
.matrixC5_6(matrixC5_6),
.matrixC5_7(matrixC5_7),
.matrixC6_0(matrixC6_0),
.matrixC6_1(matrixC6_1),
.matrixC6_2(matrixC6_2),
.matrixC6_3(matrixC6_3),
.matrixC6_4(matrixC6_4),
.matrixC6_5(matrixC6_5),
.matrixC6_6(matrixC6_6),
.matrixC6_7(matrixC6_7),
.matrixC7_0(matrixC7_0),
.matrixC7_1(matrixC7_1),
.matrixC7_2(matrixC7_2),
.matrixC7_3(matrixC7_3),
.matrixC7_4(matrixC7_4),
.matrixC7_5(matrixC7_5),
.matrixC7_6(matrixC7_6),
.matrixC7_7(matrixC7_7),

.clk(clk),
.reset(reset)
);

//////////////////////////////////////////////////////////////////////////
// Instantiations of the actual PEs
//////////////////////////////////////////////////////////////////////////
systolic_pe_matrix u_systolic_pe_matrix(
.clk(clk),
.reset(reset),
.pe_reset(pe_reset),
.a0(a0),
.a1(a1),
.a2(a2),
.a3(a3),
.a4(a4),
.a5(a5),
.a6(a6),
.a7(a7),
.b0(b0),
.b1(b1),
.b2(b2),
.b3(b3),
.b4(b4),
.b5(b5),
.b6(b6),
.b7(b7),
.matrixC0_0(matrixC0_0),
.matrixC0_1(matrixC0_1),
.matrixC0_2(matrixC0_2),
.matrixC0_3(matrixC0_3),
.matrixC0_4(matrixC0_4),
.matrixC0_5(matrixC0_5),
.matrixC0_6(matrixC0_6),
.matrixC0_7(matrixC0_7),
.matrixC1_0(matrixC1_0),
.matrixC1_1(matrixC1_1),
.matrixC1_2(matrixC1_2),
.matrixC1_3(matrixC1_3),
.matrixC1_4(matrixC1_4),
.matrixC1_5(matrixC1_5),
.matrixC1_6(matrixC1_6),
.matrixC1_7(matrixC1_7),
.matrixC2_0(matrixC2_0),
.matrixC2_1(matrixC2_1),
.matrixC2_2(matrixC2_2),
.matrixC2_3(matrixC2_3),
.matrixC2_4(matrixC2_4),
.matrixC2_5(matrixC2_5),
.matrixC2_6(matrixC2_6),
.matrixC2_7(matrixC2_7),
.matrixC3_0(matrixC3_0),
.matrixC3_1(matrixC3_1),
.matrixC3_2(matrixC3_2),
.matrixC3_3(matrixC3_3),
.matrixC3_4(matrixC3_4),
.matrixC3_5(matrixC3_5),
.matrixC3_6(matrixC3_6),
.matrixC3_7(matrixC3_7),
.matrixC4_0(matrixC4_0),
.matrixC4_1(matrixC4_1),
.matrixC4_2(matrixC4_2),
.matrixC4_3(matrixC4_3),
.matrixC4_4(matrixC4_4),
.matrixC4_5(matrixC4_5),
.matrixC4_6(matrixC4_6),
.matrixC4_7(matrixC4_7),
.matrixC5_0(matrixC5_0),
.matrixC5_1(matrixC5_1),
.matrixC5_2(matrixC5_2),
.matrixC5_3(matrixC5_3),
.matrixC5_4(matrixC5_4),
.matrixC5_5(matrixC5_5),
.matrixC5_6(matrixC5_6),
.matrixC5_7(matrixC5_7),
.matrixC6_0(matrixC6_0),
.matrixC6_1(matrixC6_1),
.matrixC6_2(matrixC6_2),
.matrixC6_3(matrixC6_3),
.matrixC6_4(matrixC6_4),
.matrixC6_5(matrixC6_5),
.matrixC6_6(matrixC6_6),
.matrixC6_7(matrixC6_7),
.matrixC7_0(matrixC7_0),
.matrixC7_1(matrixC7_1),
.matrixC7_2(matrixC7_2),
.matrixC7_3(matrixC7_3),
.matrixC7_4(matrixC7_4),
.matrixC7_5(matrixC7_5),
.matrixC7_6(matrixC7_6),
.matrixC7_7(matrixC7_7),

.a_data_out(a_data_out),
.b_data_out(b_data_out)
);

endmodule


//////////////////////////////////////////////////////////////////////////
// Output logic
//////////////////////////////////////////////////////////////////////////
module output_logic(
matmul_op_in_progress,
done_mat_mul,
c_data_out,
c_data_available,
clk_cnt,
row_latch_en,
matrixC0_0,
matrixC0_1,
matrixC0_2,
matrixC0_3,
matrixC0_4,
matrixC0_5,
matrixC0_6,
matrixC0_7,
matrixC1_0,
matrixC1_1,
matrixC1_2,
matrixC1_3,
matrixC1_4,
matrixC1_5,
matrixC1_6,
matrixC1_7,
matrixC2_0,
matrixC2_1,
matrixC2_2,
matrixC2_3,
matrixC2_4,
matrixC2_5,
matrixC2_6,
matrixC2_7,
matrixC3_0,
matrixC3_1,
matrixC3_2,
matrixC3_3,
matrixC3_4,
matrixC3_5,
matrixC3_6,
matrixC3_7,
matrixC4_0,
matrixC4_1,
matrixC4_2,
matrixC4_3,
matrixC4_4,
matrixC4_5,
matrixC4_6,
matrixC4_7,
matrixC5_0,
matrixC5_1,
matrixC5_2,
matrixC5_3,
matrixC5_4,
matrixC5_5,
matrixC5_6,
matrixC5_7,
matrixC6_0,
matrixC6_1,
matrixC6_2,
matrixC6_3,
matrixC6_4,
matrixC6_5,
matrixC6_6,
matrixC6_7,
matrixC7_0,
matrixC7_1,
matrixC7_2,
matrixC7_3,
matrixC7_4,
matrixC7_5,
matrixC7_6,
matrixC7_7,

clk,
reset
);

input clk;
input reset;
input matmul_op_in_progress;
input done_mat_mul;
output [`MAT_MUL_SIZE*`DWIDTH-1:0] c_data_out;
output c_data_available;
input [7:0] clk_cnt;
output row_latch_en;
input [`DWIDTH-1:0] matrixC0_0;
input [`DWIDTH-1:0] matrixC0_1;
input [`DWIDTH-1:0] matrixC0_2;
input [`DWIDTH-1:0] matrixC0_3;
input [`DWIDTH-1:0] matrixC0_4;
input [`DWIDTH-1:0] matrixC0_5;
input [`DWIDTH-1:0] matrixC0_6;
input [`DWIDTH-1:0] matrixC0_7;
input [`DWIDTH-1:0] matrixC1_0;
input [`DWIDTH-1:0] matrixC1_1;
input [`DWIDTH-1:0] matrixC1_2;
input [`DWIDTH-1:0] matrixC1_3;
input [`DWIDTH-1:0] matrixC1_4;
input [`DWIDTH-1:0] matrixC1_5;
input [`DWIDTH-1:0] matrixC1_6;
input [`DWIDTH-1:0] matrixC1_7;
input [`DWIDTH-1:0] matrixC2_0;
input [`DWIDTH-1:0] matrixC2_1;
input [`DWIDTH-1:0] matrixC2_2;
input [`DWIDTH-1:0] matrixC2_3;
input [`DWIDTH-1:0] matrixC2_4;
input [`DWIDTH-1:0] matrixC2_5;
input [`DWIDTH-1:0] matrixC2_6;
input [`DWIDTH-1:0] matrixC2_7;
input [`DWIDTH-1:0] matrixC3_0;
input [`DWIDTH-1:0] matrixC3_1;
input [`DWIDTH-1:0] matrixC3_2;
input [`DWIDTH-1:0] matrixC3_3;
input [`DWIDTH-1:0] matrixC3_4;
input [`DWIDTH-1:0] matrixC3_5;
input [`DWIDTH-1:0] matrixC3_6;
input [`DWIDTH-1:0] matrixC3_7;
input [`DWIDTH-1:0] matrixC4_0;
input [`DWIDTH-1:0] matrixC4_1;
input [`DWIDTH-1:0] matrixC4_2;
input [`DWIDTH-1:0] matrixC4_3;
input [`DWIDTH-1:0] matrixC4_4;
input [`DWIDTH-1:0] matrixC4_5;
input [`DWIDTH-1:0] matrixC4_6;
input [`DWIDTH-1:0] matrixC4_7;
input [`DWIDTH-1:0] matrixC5_0;
input [`DWIDTH-1:0] matrixC5_1;
input [`DWIDTH-1:0] matrixC5_2;
input [`DWIDTH-1:0] matrixC5_3;
input [`DWIDTH-1:0] matrixC5_4;
input [`DWIDTH-1:0] matrixC5_5;
input [`DWIDTH-1:0] matrixC5_6;
input [`DWIDTH-1:0] matrixC5_7;
input [`DWIDTH-1:0] matrixC6_0;
input [`DWIDTH-1:0] matrixC6_1;
input [`DWIDTH-1:0] matrixC6_2;
input [`DWIDTH-1:0] matrixC6_3;
input [`DWIDTH-1:0] matrixC6_4;
input [`DWIDTH-1:0] matrixC6_5;
input [`DWIDTH-1:0] matrixC6_6;
input [`DWIDTH-1:0] matrixC6_7;
input [`DWIDTH-1:0] matrixC7_0;
input [`DWIDTH-1:0] matrixC7_1;
input [`DWIDTH-1:0] matrixC7_2;
input [`DWIDTH-1:0] matrixC7_3;
input [`DWIDTH-1:0] matrixC7_4;
input [`DWIDTH-1:0] matrixC7_5;
input [`DWIDTH-1:0] matrixC7_6;
input [`DWIDTH-1:0] matrixC7_7;
wire row_latch_en;


//////////////////////////////////////////////////////////////////////////
// Logic to capture matrix C data from the PEs and shift it out
//////////////////////////////////////////////////////////////////////////
//assign row_latch_en = (clk_cnt==(`MAT_MUL_SIZE + (a_loc+b_loc) * `BB_MAT_MUL_SIZE + 10 +  `NUM_CYCLES_IN_MAC - 1));
//Writing the line above to avoid multiplication:
//assign row_latch_en = (clk_cnt==(`MAT_MUL_SIZE + ((a_loc+b_loc) << `LOG2_MAT_MUL_SIZE) + 10 +  `NUM_CYCLES_IN_MAC - 1));

assign row_latch_en =  
                       ((clk_cnt == 27 ));
    
reg c_data_available;
reg start_capturing_c_data;
integer counter;
reg [8*`DWIDTH-1:0] c_data_out;
reg [8*`DWIDTH-1:0] c_data_out_1;
reg [8*`DWIDTH-1:0] c_data_out_2;
reg [8*`DWIDTH-1:0] c_data_out_3;
reg [8*`DWIDTH-1:0] c_data_out_4;
reg [8*`DWIDTH-1:0] c_data_out_5;
reg [8*`DWIDTH-1:0] c_data_out_6;
reg [8*`DWIDTH-1:0] c_data_out_7;
wire condition_to_start_shifting_output;
assign condition_to_start_shifting_output = 
                          row_latch_en ;  

  
//For larger matmuls, this logic will have more entries in the case statement
always @(posedge clk) begin
  if (reset | ~matmul_op_in_progress) begin
    start_capturing_c_data <= 1'b0;
    c_data_available <= 1'b0;
    c_data_out <= 0;
    counter <= 0;

    c_data_out_1 <= 0;
    c_data_out_2 <= 0;
    c_data_out_3 <= 0;
    c_data_out_4 <= 0;
    c_data_out_5 <= 0;
    c_data_out_6 <= 0;
    c_data_out_7 <= 0;
  end else if (condition_to_start_shifting_output) begin
    start_capturing_c_data <= 1'b1;
    c_data_available <= 1'b1;
    c_data_out <= {matrixC7_7, matrixC6_7, matrixC5_7, matrixC4_7, matrixC3_7, matrixC2_7, matrixC1_7, matrixC0_7};
      c_data_out_1 <= {matrixC7_6, matrixC6_6, matrixC5_6, matrixC4_6, matrixC3_6, matrixC2_6, matrixC1_6, matrixC0_6};
      c_data_out_2 <= {matrixC7_5, matrixC6_5, matrixC5_5, matrixC4_5, matrixC3_5, matrixC2_5, matrixC1_5, matrixC0_5};
      c_data_out_3 <= {matrixC7_4, matrixC6_4, matrixC5_4, matrixC4_4, matrixC3_4, matrixC2_4, matrixC1_4, matrixC0_4};
      c_data_out_4 <= {matrixC7_3, matrixC6_3, matrixC5_3, matrixC4_3, matrixC3_3, matrixC2_3, matrixC1_3, matrixC0_3};
      c_data_out_5 <= {matrixC7_2, matrixC6_2, matrixC5_2, matrixC4_2, matrixC3_2, matrixC2_2, matrixC1_2, matrixC0_2};
      c_data_out_6 <= {matrixC7_1, matrixC6_1, matrixC5_1, matrixC4_1, matrixC3_1, matrixC2_1, matrixC1_1, matrixC0_1};
      c_data_out_7 <= {matrixC7_0, matrixC6_0, matrixC5_0, matrixC4_0, matrixC3_0, matrixC2_0, matrixC1_0, matrixC0_0};

    counter <= counter + 1;
  end else if (done_mat_mul) begin
    start_capturing_c_data <= 1'b0;
    c_data_available <= 1'b0;
    c_data_out <= 0;

    c_data_out_1 <= 0;
    c_data_out_2 <= 0;
    c_data_out_3 <= 0;
    c_data_out_4 <= 0;
    c_data_out_5 <= 0;
    c_data_out_6 <= 0;
    c_data_out_7 <= 0;
  end 
  else if (counter >= `MAT_MUL_SIZE) begin
    c_data_out <= c_data_out_1;

    c_data_out_1 <= c_data_out_2;
    c_data_out_2 <= c_data_out_3;
    c_data_out_3 <= c_data_out_4;
    c_data_out_4 <= c_data_out_5;
    c_data_out_5 <= c_data_out_6;
    c_data_out_6 <= c_data_out_7;
    c_data_out_7 <= 0;
  end
  else if (start_capturing_c_data) begin
    c_data_available <= 1'b1;
    counter <= counter + 1;
    c_data_out <= c_data_out_1;

    c_data_out_1 <= c_data_out_2;
    c_data_out_2 <= c_data_out_3;
    c_data_out_3 <= c_data_out_4;
    c_data_out_4 <= c_data_out_5;
    c_data_out_5 <= c_data_out_6;
    c_data_out_6 <= c_data_out_7;
    c_data_out_7 <= 0;
  end
end

endmodule


//////////////////////////////////////////////////////////////////////////
// Systolic data setup
//////////////////////////////////////////////////////////////////////////
module systolic_data_setup(
clk,
reset,
matmul_op_in_progress,
a_data,
b_data,
clk_cnt,
a0_data,
b0_data,
a1_data_delayed_1,
b1_data_delayed_1,
a2_data_delayed_2,
b2_data_delayed_2,
a3_data_delayed_3,
b3_data_delayed_3,
a4_data_delayed_4,
b4_data_delayed_4,
a5_data_delayed_5,
b5_data_delayed_5,
a6_data_delayed_6,
b6_data_delayed_6,
a7_data_delayed_7,
b7_data_delayed_7,

validity_mask_a_rows,
validity_mask_a_cols,
validity_mask_b_rows,
validity_mask_b_cols
);

input clk;
input reset;
input matmul_op_in_progress;
input [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data;
input [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data;
input [7:0] clk_cnt;
output [`DWIDTH-1:0] a0_data;
output [`DWIDTH-1:0] b0_data;
output [`DWIDTH-1:0] a1_data_delayed_1;
output [`DWIDTH-1:0] b1_data_delayed_1;
output [`DWIDTH-1:0] a2_data_delayed_2;
output [`DWIDTH-1:0] b2_data_delayed_2;
output [`DWIDTH-1:0] a3_data_delayed_3;
output [`DWIDTH-1:0] b3_data_delayed_3;
output [`DWIDTH-1:0] a4_data_delayed_4;
output [`DWIDTH-1:0] b4_data_delayed_4;
output [`DWIDTH-1:0] a5_data_delayed_5;
output [`DWIDTH-1:0] b5_data_delayed_5;
output [`DWIDTH-1:0] a6_data_delayed_6;
output [`DWIDTH-1:0] b6_data_delayed_6;
output [`DWIDTH-1:0] a7_data_delayed_7;
output [`DWIDTH-1:0] b7_data_delayed_7;

input [`MASK_WIDTH-1:0] validity_mask_a_rows;
input [`MASK_WIDTH-1:0] validity_mask_a_cols;
input [`MASK_WIDTH-1:0] validity_mask_b_rows;
input [`MASK_WIDTH-1:0] validity_mask_b_cols;

wire [`DWIDTH-1:0] a0_data;
wire [`DWIDTH-1:0] a1_data;
wire [`DWIDTH-1:0] a2_data;
wire [`DWIDTH-1:0] a3_data;
wire [`DWIDTH-1:0] a4_data;
wire [`DWIDTH-1:0] a5_data;
wire [`DWIDTH-1:0] a6_data;
wire [`DWIDTH-1:0] a7_data;
wire [`DWIDTH-1:0] b0_data;
wire [`DWIDTH-1:0] b1_data;
wire [`DWIDTH-1:0] b2_data;
wire [`DWIDTH-1:0] b3_data;
wire [`DWIDTH-1:0] b4_data;
wire [`DWIDTH-1:0] b5_data;
wire [`DWIDTH-1:0] b6_data;
wire [`DWIDTH-1:0] b7_data;

wire [7:0] a_mem_access_counter;
assign a_mem_access_counter = ((clk_cnt>=8) ? 0 : (matmul_op_in_progress ? (clk_cnt+1) : 0));

wire a_data_valid; //flag that tells whether the data from memory is valid
assign a_data_valid = 
     ((validity_mask_a_cols[0]==1'b0 && a_mem_access_counter==1) ||
      (validity_mask_a_cols[1]==1'b0 && a_mem_access_counter==2) ||
      (validity_mask_a_cols[2]==1'b0 && a_mem_access_counter==3) ||
      (validity_mask_a_cols[3]==1'b0 && a_mem_access_counter==4) ||
      (validity_mask_a_cols[4]==1'b0 && a_mem_access_counter==5) ||
      (validity_mask_a_cols[5]==1'b0 && a_mem_access_counter==6) ||
      (validity_mask_a_cols[6]==1'b0 && a_mem_access_counter==7) ||
      (validity_mask_a_cols[7]==1'b0 && a_mem_access_counter==8)) ?
    
    1'b0 : (a_mem_access_counter >= `MEM_ACCESS_LATENCY);

//////////////////////////////////////////////////////////////////////////
// Logic to delay certain parts of the data received from BRAM A (systolic data setup)
//////////////////////////////////////////////////////////////////////////
assign a0_data = a_data[1*`DWIDTH-1:0*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[0]}};
assign a1_data = a_data[2*`DWIDTH-1:1*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[1]}};
assign a2_data = a_data[3*`DWIDTH-1:2*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[2]}};
assign a3_data = a_data[4*`DWIDTH-1:3*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[3]}};
assign a4_data = a_data[5*`DWIDTH-1:4*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[4]}};
assign a5_data = a_data[6*`DWIDTH-1:5*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[5]}};
assign a6_data = a_data[7*`DWIDTH-1:6*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[6]}};
assign a7_data = a_data[8*`DWIDTH-1:7*`DWIDTH] & {`DWIDTH{a_data_valid}} & {`DWIDTH{validity_mask_a_rows[7]}};

reg [`DWIDTH-1:0] a1_data_delayed_1;
reg [`DWIDTH-1:0] a2_data_delayed_1;
reg [`DWIDTH-1:0] a2_data_delayed_2;
reg [`DWIDTH-1:0] a3_data_delayed_1;
reg [`DWIDTH-1:0] a3_data_delayed_2;
reg [`DWIDTH-1:0] a3_data_delayed_3;
reg [`DWIDTH-1:0] a4_data_delayed_1;
reg [`DWIDTH-1:0] a4_data_delayed_2;
reg [`DWIDTH-1:0] a4_data_delayed_3;
reg [`DWIDTH-1:0] a4_data_delayed_4;
reg [`DWIDTH-1:0] a5_data_delayed_1;
reg [`DWIDTH-1:0] a5_data_delayed_2;
reg [`DWIDTH-1:0] a5_data_delayed_3;
reg [`DWIDTH-1:0] a5_data_delayed_4;
reg [`DWIDTH-1:0] a5_data_delayed_5;
reg [`DWIDTH-1:0] a6_data_delayed_1;
reg [`DWIDTH-1:0] a6_data_delayed_2;
reg [`DWIDTH-1:0] a6_data_delayed_3;
reg [`DWIDTH-1:0] a6_data_delayed_4;
reg [`DWIDTH-1:0] a6_data_delayed_5;
reg [`DWIDTH-1:0] a6_data_delayed_6;
reg [`DWIDTH-1:0] a7_data_delayed_1;
reg [`DWIDTH-1:0] a7_data_delayed_2;
reg [`DWIDTH-1:0] a7_data_delayed_3;
reg [`DWIDTH-1:0] a7_data_delayed_4;
reg [`DWIDTH-1:0] a7_data_delayed_5;
reg [`DWIDTH-1:0] a7_data_delayed_6;
reg [`DWIDTH-1:0] a7_data_delayed_7;


always @(posedge clk) begin
  if (reset || ~matmul_op_in_progress) begin
    a1_data_delayed_1 <= 0;
    a2_data_delayed_1 <= 0;
    a2_data_delayed_2 <= 0;
    a3_data_delayed_1 <= 0;
    a3_data_delayed_2 <= 0;
    a3_data_delayed_3 <= 0;
    a4_data_delayed_1 <= 0;
    a4_data_delayed_2 <= 0;
    a4_data_delayed_3 <= 0;
    a4_data_delayed_4 <= 0;
    a5_data_delayed_1 <= 0;
    a5_data_delayed_2 <= 0;
    a5_data_delayed_3 <= 0;
    a5_data_delayed_4 <= 0;
    a5_data_delayed_5 <= 0;
    a6_data_delayed_1 <= 0;
    a6_data_delayed_2 <= 0;
    a6_data_delayed_3 <= 0;
    a6_data_delayed_4 <= 0;
    a6_data_delayed_5 <= 0;
    a6_data_delayed_6 <= 0;
    a7_data_delayed_1 <= 0;
    a7_data_delayed_2 <= 0;
    a7_data_delayed_3 <= 0;
    a7_data_delayed_4 <= 0;
    a7_data_delayed_5 <= 0;
    a7_data_delayed_6 <= 0;
    a7_data_delayed_7 <= 0;

  end
  else begin
  a1_data_delayed_1 <= a1_data;
  a2_data_delayed_1 <= a2_data;
  a3_data_delayed_1 <= a3_data;
  a4_data_delayed_1 <= a4_data;
  a5_data_delayed_1 <= a5_data;
  a6_data_delayed_1 <= a6_data;
  a7_data_delayed_1 <= a7_data;
  a2_data_delayed_2 <= a2_data_delayed_1;
  a3_data_delayed_2 <= a3_data_delayed_1;
  a3_data_delayed_3 <= a3_data_delayed_2;
  a4_data_delayed_2 <= a4_data_delayed_1;
  a4_data_delayed_3 <= a4_data_delayed_2;
  a4_data_delayed_4 <= a4_data_delayed_3;
  a5_data_delayed_2 <= a5_data_delayed_1;
  a5_data_delayed_3 <= a5_data_delayed_2;
  a5_data_delayed_4 <= a5_data_delayed_3;
  a5_data_delayed_5 <= a5_data_delayed_4;
  a6_data_delayed_2 <= a6_data_delayed_1;
  a6_data_delayed_3 <= a6_data_delayed_2;
  a6_data_delayed_4 <= a6_data_delayed_3;
  a6_data_delayed_5 <= a6_data_delayed_4;
  a6_data_delayed_6 <= a6_data_delayed_5;
  a7_data_delayed_2 <= a7_data_delayed_1;
  a7_data_delayed_3 <= a7_data_delayed_2;
  a7_data_delayed_4 <= a7_data_delayed_3;
  a7_data_delayed_5 <= a7_data_delayed_4;
  a7_data_delayed_6 <= a7_data_delayed_5;
  a7_data_delayed_7 <= a7_data_delayed_6;
 
  end
end

wire [7:0] b_mem_access_counter;
assign b_mem_access_counter = ((clk_cnt>=8) ? 0 : (matmul_op_in_progress ? (clk_cnt+1) : 0));

wire b_data_valid; //flag that tells whether the data from memory is valid
assign b_data_valid = 
     ((validity_mask_b_rows[0]==1'b0 && b_mem_access_counter==1) ||
      (validity_mask_b_rows[1]==1'b0 && b_mem_access_counter==2) ||
      (validity_mask_b_rows[2]==1'b0 && b_mem_access_counter==3) ||
      (validity_mask_b_rows[3]==1'b0 && b_mem_access_counter==4) ||
      (validity_mask_b_rows[4]==1'b0 && b_mem_access_counter==5) ||
      (validity_mask_b_rows[5]==1'b0 && b_mem_access_counter==6) ||
      (validity_mask_b_rows[6]==1'b0 && b_mem_access_counter==7) ||
      (validity_mask_b_rows[7]==1'b0 && b_mem_access_counter==8)) ?
    
        1'b0 : (b_mem_access_counter >= `MEM_ACCESS_LATENCY);

//////////////////////////////////////////////////////////////////////////
// Logic to delay certain parts of the data received from BRAM B (systolic data setup)
//////////////////////////////////////////////////////////////////////////
assign b0_data = b_data[1*`DWIDTH-1:0*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[0]}};
assign b1_data = b_data[2*`DWIDTH-1:1*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[1]}};
assign b2_data = b_data[3*`DWIDTH-1:2*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[2]}};
assign b3_data = b_data[4*`DWIDTH-1:3*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[3]}};
assign b4_data = b_data[5*`DWIDTH-1:4*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[4]}};
assign b5_data = b_data[6*`DWIDTH-1:5*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[5]}};
assign b6_data = b_data[7*`DWIDTH-1:6*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[6]}};
assign b7_data = b_data[8*`DWIDTH-1:7*`DWIDTH] & {`DWIDTH{b_data_valid}} & {`DWIDTH{validity_mask_b_cols[7]}};

reg [`DWIDTH-1:0] b1_data_delayed_1;
reg [`DWIDTH-1:0] b2_data_delayed_1;
reg [`DWIDTH-1:0] b2_data_delayed_2;
reg [`DWIDTH-1:0] b3_data_delayed_1;
reg [`DWIDTH-1:0] b3_data_delayed_2;
reg [`DWIDTH-1:0] b3_data_delayed_3;
reg [`DWIDTH-1:0] b4_data_delayed_1;
reg [`DWIDTH-1:0] b4_data_delayed_2;
reg [`DWIDTH-1:0] b4_data_delayed_3;
reg [`DWIDTH-1:0] b4_data_delayed_4;
reg [`DWIDTH-1:0] b5_data_delayed_1;
reg [`DWIDTH-1:0] b5_data_delayed_2;
reg [`DWIDTH-1:0] b5_data_delayed_3;
reg [`DWIDTH-1:0] b5_data_delayed_4;
reg [`DWIDTH-1:0] b5_data_delayed_5;
reg [`DWIDTH-1:0] b6_data_delayed_1;
reg [`DWIDTH-1:0] b6_data_delayed_2;
reg [`DWIDTH-1:0] b6_data_delayed_3;
reg [`DWIDTH-1:0] b6_data_delayed_4;
reg [`DWIDTH-1:0] b6_data_delayed_5;
reg [`DWIDTH-1:0] b6_data_delayed_6;
reg [`DWIDTH-1:0] b7_data_delayed_1;
reg [`DWIDTH-1:0] b7_data_delayed_2;
reg [`DWIDTH-1:0] b7_data_delayed_3;
reg [`DWIDTH-1:0] b7_data_delayed_4;
reg [`DWIDTH-1:0] b7_data_delayed_5;
reg [`DWIDTH-1:0] b7_data_delayed_6;
reg [`DWIDTH-1:0] b7_data_delayed_7;


always @(posedge clk) begin
  if (reset || ~matmul_op_in_progress) begin
    b1_data_delayed_1 <= 0;
    b2_data_delayed_1 <= 0;
    b2_data_delayed_2 <= 0;
    b3_data_delayed_1 <= 0;
    b3_data_delayed_2 <= 0;
    b3_data_delayed_3 <= 0;
    b4_data_delayed_1 <= 0;
    b4_data_delayed_2 <= 0;
    b4_data_delayed_3 <= 0;
    b4_data_delayed_4 <= 0;
    b5_data_delayed_1 <= 0;
    b5_data_delayed_2 <= 0;
    b5_data_delayed_3 <= 0;
    b5_data_delayed_4 <= 0;
    b5_data_delayed_5 <= 0;
    b6_data_delayed_1 <= 0;
    b6_data_delayed_2 <= 0;
    b6_data_delayed_3 <= 0;
    b6_data_delayed_4 <= 0;
    b6_data_delayed_5 <= 0;
    b6_data_delayed_6 <= 0;
    b7_data_delayed_1 <= 0;
    b7_data_delayed_2 <= 0;
    b7_data_delayed_3 <= 0;
    b7_data_delayed_4 <= 0;
    b7_data_delayed_5 <= 0;
    b7_data_delayed_6 <= 0;
    b7_data_delayed_7 <= 0;

  end
  else begin
  b1_data_delayed_1 <= b1_data;
  b2_data_delayed_1 <= b2_data;
  b3_data_delayed_1 <= b3_data;
  b4_data_delayed_1 <= b4_data;
  b5_data_delayed_1 <= b5_data;
  b6_data_delayed_1 <= b6_data;
  b7_data_delayed_1 <= b7_data;
  b2_data_delayed_2 <= b2_data_delayed_1;
  b3_data_delayed_2 <= b3_data_delayed_1;
  b3_data_delayed_3 <= b3_data_delayed_2;
  b4_data_delayed_2 <= b4_data_delayed_1;
  b4_data_delayed_3 <= b4_data_delayed_2;
  b4_data_delayed_4 <= b4_data_delayed_3;
  b5_data_delayed_2 <= b5_data_delayed_1;
  b5_data_delayed_3 <= b5_data_delayed_2;
  b5_data_delayed_4 <= b5_data_delayed_3;
  b5_data_delayed_5 <= b5_data_delayed_4;
  b6_data_delayed_2 <= b6_data_delayed_1;
  b6_data_delayed_3 <= b6_data_delayed_2;
  b6_data_delayed_4 <= b6_data_delayed_3;
  b6_data_delayed_5 <= b6_data_delayed_4;
  b6_data_delayed_6 <= b6_data_delayed_5;
  b7_data_delayed_2 <= b7_data_delayed_1;
  b7_data_delayed_3 <= b7_data_delayed_2;
  b7_data_delayed_4 <= b7_data_delayed_3;
  b7_data_delayed_5 <= b7_data_delayed_4;
  b7_data_delayed_6 <= b7_data_delayed_5;
  b7_data_delayed_7 <= b7_data_delayed_6;
 
  end
end
endmodule


//////////////////////////////////////////////////////////////////////////
// Systolically connected PEs
//////////////////////////////////////////////////////////////////////////
module systolic_pe_matrix(
clk,
reset,
pe_reset,
a0,
a1,
a2,
a3,
a4,
a5,
a6,
a7,
b0,
b1,
b2,
b3,
b4,
b5,
b6,
b7,
matrixC0_0,
matrixC0_1,
matrixC0_2,
matrixC0_3,
matrixC0_4,
matrixC0_5,
matrixC0_6,
matrixC0_7,
matrixC1_0,
matrixC1_1,
matrixC1_2,
matrixC1_3,
matrixC1_4,
matrixC1_5,
matrixC1_6,
matrixC1_7,
matrixC2_0,
matrixC2_1,
matrixC2_2,
matrixC2_3,
matrixC2_4,
matrixC2_5,
matrixC2_6,
matrixC2_7,
matrixC3_0,
matrixC3_1,
matrixC3_2,
matrixC3_3,
matrixC3_4,
matrixC3_5,
matrixC3_6,
matrixC3_7,
matrixC4_0,
matrixC4_1,
matrixC4_2,
matrixC4_3,
matrixC4_4,
matrixC4_5,
matrixC4_6,
matrixC4_7,
matrixC5_0,
matrixC5_1,
matrixC5_2,
matrixC5_3,
matrixC5_4,
matrixC5_5,
matrixC5_6,
matrixC5_7,
matrixC6_0,
matrixC6_1,
matrixC6_2,
matrixC6_3,
matrixC6_4,
matrixC6_5,
matrixC6_6,
matrixC6_7,
matrixC7_0,
matrixC7_1,
matrixC7_2,
matrixC7_3,
matrixC7_4,
matrixC7_5,
matrixC7_6,
matrixC7_7,

a_data_out,
b_data_out
);

input clk;
input reset;
input pe_reset;
input [`DWIDTH-1:0] a0;
input [`DWIDTH-1:0] a1;
input [`DWIDTH-1:0] a2;
input [`DWIDTH-1:0] a3;
input [`DWIDTH-1:0] a4;
input [`DWIDTH-1:0] a5;
input [`DWIDTH-1:0] a6;
input [`DWIDTH-1:0] a7;
input [`DWIDTH-1:0] b0;
input [`DWIDTH-1:0] b1;
input [`DWIDTH-1:0] b2;
input [`DWIDTH-1:0] b3;
input [`DWIDTH-1:0] b4;
input [`DWIDTH-1:0] b5;
input [`DWIDTH-1:0] b6;
input [`DWIDTH-1:0] b7;
output [`DWIDTH-1:0] matrixC0_0;
output [`DWIDTH-1:0] matrixC0_1;
output [`DWIDTH-1:0] matrixC0_2;
output [`DWIDTH-1:0] matrixC0_3;
output [`DWIDTH-1:0] matrixC0_4;
output [`DWIDTH-1:0] matrixC0_5;
output [`DWIDTH-1:0] matrixC0_6;
output [`DWIDTH-1:0] matrixC0_7;
output [`DWIDTH-1:0] matrixC1_0;
output [`DWIDTH-1:0] matrixC1_1;
output [`DWIDTH-1:0] matrixC1_2;
output [`DWIDTH-1:0] matrixC1_3;
output [`DWIDTH-1:0] matrixC1_4;
output [`DWIDTH-1:0] matrixC1_5;
output [`DWIDTH-1:0] matrixC1_6;
output [`DWIDTH-1:0] matrixC1_7;
output [`DWIDTH-1:0] matrixC2_0;
output [`DWIDTH-1:0] matrixC2_1;
output [`DWIDTH-1:0] matrixC2_2;
output [`DWIDTH-1:0] matrixC2_3;
output [`DWIDTH-1:0] matrixC2_4;
output [`DWIDTH-1:0] matrixC2_5;
output [`DWIDTH-1:0] matrixC2_6;
output [`DWIDTH-1:0] matrixC2_7;
output [`DWIDTH-1:0] matrixC3_0;
output [`DWIDTH-1:0] matrixC3_1;
output [`DWIDTH-1:0] matrixC3_2;
output [`DWIDTH-1:0] matrixC3_3;
output [`DWIDTH-1:0] matrixC3_4;
output [`DWIDTH-1:0] matrixC3_5;
output [`DWIDTH-1:0] matrixC3_6;
output [`DWIDTH-1:0] matrixC3_7;
output [`DWIDTH-1:0] matrixC4_0;
output [`DWIDTH-1:0] matrixC4_1;
output [`DWIDTH-1:0] matrixC4_2;
output [`DWIDTH-1:0] matrixC4_3;
output [`DWIDTH-1:0] matrixC4_4;
output [`DWIDTH-1:0] matrixC4_5;
output [`DWIDTH-1:0] matrixC4_6;
output [`DWIDTH-1:0] matrixC4_7;
output [`DWIDTH-1:0] matrixC5_0;
output [`DWIDTH-1:0] matrixC5_1;
output [`DWIDTH-1:0] matrixC5_2;
output [`DWIDTH-1:0] matrixC5_3;
output [`DWIDTH-1:0] matrixC5_4;
output [`DWIDTH-1:0] matrixC5_5;
output [`DWIDTH-1:0] matrixC5_6;
output [`DWIDTH-1:0] matrixC5_7;
output [`DWIDTH-1:0] matrixC6_0;
output [`DWIDTH-1:0] matrixC6_1;
output [`DWIDTH-1:0] matrixC6_2;
output [`DWIDTH-1:0] matrixC6_3;
output [`DWIDTH-1:0] matrixC6_4;
output [`DWIDTH-1:0] matrixC6_5;
output [`DWIDTH-1:0] matrixC6_6;
output [`DWIDTH-1:0] matrixC6_7;
output [`DWIDTH-1:0] matrixC7_0;
output [`DWIDTH-1:0] matrixC7_1;
output [`DWIDTH-1:0] matrixC7_2;
output [`DWIDTH-1:0] matrixC7_3;
output [`DWIDTH-1:0] matrixC7_4;
output [`DWIDTH-1:0] matrixC7_5;
output [`DWIDTH-1:0] matrixC7_6;
output [`DWIDTH-1:0] matrixC7_7;

output [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data_out;
output [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data_out;

wire [`DWIDTH-1:0] a0_0to0_1, a0_1to0_2, a0_2to0_3, a0_3to0_4, a0_4to0_5, a0_5to0_6, a0_6to0_7, a0_7to0_8;
wire [`DWIDTH-1:0] a1_0to1_1, a1_1to1_2, a1_2to1_3, a1_3to1_4, a1_4to1_5, a1_5to1_6, a1_6to1_7, a1_7to1_8;
wire [`DWIDTH-1:0] a2_0to2_1, a2_1to2_2, a2_2to2_3, a2_3to2_4, a2_4to2_5, a2_5to2_6, a2_6to2_7, a2_7to2_8;
wire [`DWIDTH-1:0] a3_0to3_1, a3_1to3_2, a3_2to3_3, a3_3to3_4, a3_4to3_5, a3_5to3_6, a3_6to3_7, a3_7to3_8;
wire [`DWIDTH-1:0] a4_0to4_1, a4_1to4_2, a4_2to4_3, a4_3to4_4, a4_4to4_5, a4_5to4_6, a4_6to4_7, a4_7to4_8;
wire [`DWIDTH-1:0] a5_0to5_1, a5_1to5_2, a5_2to5_3, a5_3to5_4, a5_4to5_5, a5_5to5_6, a5_6to5_7, a5_7to5_8;
wire [`DWIDTH-1:0] a6_0to6_1, a6_1to6_2, a6_2to6_3, a6_3to6_4, a6_4to6_5, a6_5to6_6, a6_6to6_7, a6_7to6_8;
wire [`DWIDTH-1:0] a7_0to7_1, a7_1to7_2, a7_2to7_3, a7_3to7_4, a7_4to7_5, a7_5to7_6, a7_6to7_7, a7_7to7_8;

wire [`DWIDTH-1:0] b0_0to1_0, b1_0to2_0, b2_0to3_0, b3_0to4_0, b4_0to5_0, b5_0to6_0, b6_0to7_0, b7_0to8_0;
wire [`DWIDTH-1:0] b0_1to1_1, b1_1to2_1, b2_1to3_1, b3_1to4_1, b4_1to5_1, b5_1to6_1, b6_1to7_1, b7_1to8_1;
wire [`DWIDTH-1:0] b0_2to1_2, b1_2to2_2, b2_2to3_2, b3_2to4_2, b4_2to5_2, b5_2to6_2, b6_2to7_2, b7_2to8_2;
wire [`DWIDTH-1:0] b0_3to1_3, b1_3to2_3, b2_3to3_3, b3_3to4_3, b4_3to5_3, b5_3to6_3, b6_3to7_3, b7_3to8_3;
wire [`DWIDTH-1:0] b0_4to1_4, b1_4to2_4, b2_4to3_4, b3_4to4_4, b4_4to5_4, b5_4to6_4, b6_4to7_4, b7_4to8_4;
wire [`DWIDTH-1:0] b0_5to1_5, b1_5to2_5, b2_5to3_5, b3_5to4_5, b4_5to5_5, b5_5to6_5, b6_5to7_5, b7_5to8_5;
wire [`DWIDTH-1:0] b0_6to1_6, b1_6to2_6, b2_6to3_6, b3_6to4_6, b4_6to5_6, b5_6to6_6, b6_6to7_6, b7_6to8_6;
wire [`DWIDTH-1:0] b0_7to1_7, b1_7to2_7, b2_7to3_7, b3_7to4_7, b4_7to5_7, b5_7to6_7, b6_7to7_7, b7_7to8_7;

//////////////////////////////////////////////////////////////////////////
// Instantiations of the actual PEs
//////////////////////////////////////////////////////////////////////////
//For larger matmul, more PEs will be needed
wire effective_rst;
assign effective_rst = reset | pe_reset;

processing_element pe0_0(.reset(effective_rst), .clk(clk),  .in_a(a0),      .in_b(b0),  .out_a(a0_0to0_1), .out_b(b0_0to1_0), .out_c(matrixC0_0));
processing_element pe0_1(.reset(effective_rst), .clk(clk),  .in_a(a0_0to0_1), .in_b(b1),  .out_a(a0_1to0_2), .out_b(b0_1to1_1), .out_c(matrixC0_1));
processing_element pe0_2(.reset(effective_rst), .clk(clk),  .in_a(a0_1to0_2), .in_b(b2),  .out_a(a0_2to0_3), .out_b(b0_2to1_2), .out_c(matrixC0_2));
processing_element pe0_3(.reset(effective_rst), .clk(clk),  .in_a(a0_2to0_3), .in_b(b3),  .out_a(a0_3to0_4), .out_b(b0_3to1_3), .out_c(matrixC0_3));
processing_element pe0_4(.reset(effective_rst), .clk(clk),  .in_a(a0_3to0_4), .in_b(b4),  .out_a(a0_4to0_5), .out_b(b0_4to1_4), .out_c(matrixC0_4));
processing_element pe0_5(.reset(effective_rst), .clk(clk),  .in_a(a0_4to0_5), .in_b(b5),  .out_a(a0_5to0_6), .out_b(b0_5to1_5), .out_c(matrixC0_5));
processing_element pe0_6(.reset(effective_rst), .clk(clk),  .in_a(a0_5to0_6), .in_b(b6),  .out_a(a0_6to0_7), .out_b(b0_6to1_6), .out_c(matrixC0_6));
processing_element pe0_7(.reset(effective_rst), .clk(clk),  .in_a(a0_6to0_7), .in_b(b7),  .out_a(a0_7to0_8), .out_b(b0_7to1_7), .out_c(matrixC0_7));

processing_element pe1_0(.reset(effective_rst), .clk(clk),  .in_a(a1), .in_b(b0_0to1_0),  .out_a(a1_0to1_1), .out_b(b1_0to2_0), .out_c(matrixC1_0));
processing_element pe2_0(.reset(effective_rst), .clk(clk),  .in_a(a2), .in_b(b1_0to2_0),  .out_a(a2_0to2_1), .out_b(b2_0to3_0), .out_c(matrixC2_0));
processing_element pe3_0(.reset(effective_rst), .clk(clk),  .in_a(a3), .in_b(b2_0to3_0),  .out_a(a3_0to3_1), .out_b(b3_0to4_0), .out_c(matrixC3_0));
processing_element pe4_0(.reset(effective_rst), .clk(clk),  .in_a(a4), .in_b(b3_0to4_0),  .out_a(a4_0to4_1), .out_b(b4_0to5_0), .out_c(matrixC4_0));
processing_element pe5_0(.reset(effective_rst), .clk(clk),  .in_a(a5), .in_b(b4_0to5_0),  .out_a(a5_0to5_1), .out_b(b5_0to6_0), .out_c(matrixC5_0));
processing_element pe6_0(.reset(effective_rst), .clk(clk),  .in_a(a6), .in_b(b5_0to6_0),  .out_a(a6_0to6_1), .out_b(b6_0to7_0), .out_c(matrixC6_0));
processing_element pe7_0(.reset(effective_rst), .clk(clk),  .in_a(a7), .in_b(b6_0to7_0),  .out_a(a7_0to7_1), .out_b(b7_0to8_0), .out_c(matrixC7_0));

processing_element pe1_1(.reset(effective_rst), .clk(clk),  .in_a(a1_0to1_1), .in_b(b0_1to1_1),  .out_a(a1_1to1_2), .out_b(b1_1to2_1), .out_c(matrixC1_1));
processing_element pe1_2(.reset(effective_rst), .clk(clk),  .in_a(a1_1to1_2), .in_b(b0_2to1_2),  .out_a(a1_2to1_3), .out_b(b1_2to2_2), .out_c(matrixC1_2));
processing_element pe1_3(.reset(effective_rst), .clk(clk),  .in_a(a1_2to1_3), .in_b(b0_3to1_3),  .out_a(a1_3to1_4), .out_b(b1_3to2_3), .out_c(matrixC1_3));
processing_element pe1_4(.reset(effective_rst), .clk(clk),  .in_a(a1_3to1_4), .in_b(b0_4to1_4),  .out_a(a1_4to1_5), .out_b(b1_4to2_4), .out_c(matrixC1_4));
processing_element pe1_5(.reset(effective_rst), .clk(clk),  .in_a(a1_4to1_5), .in_b(b0_5to1_5),  .out_a(a1_5to1_6), .out_b(b1_5to2_5), .out_c(matrixC1_5));
processing_element pe1_6(.reset(effective_rst), .clk(clk),  .in_a(a1_5to1_6), .in_b(b0_6to1_6),  .out_a(a1_6to1_7), .out_b(b1_6to2_6), .out_c(matrixC1_6));
processing_element pe1_7(.reset(effective_rst), .clk(clk),  .in_a(a1_6to1_7), .in_b(b0_7to1_7),  .out_a(a1_7to1_8), .out_b(b1_7to2_7), .out_c(matrixC1_7));
processing_element pe2_1(.reset(effective_rst), .clk(clk),  .in_a(a2_0to2_1), .in_b(b1_1to2_1),  .out_a(a2_1to2_2), .out_b(b2_1to3_1), .out_c(matrixC2_1));
processing_element pe2_2(.reset(effective_rst), .clk(clk),  .in_a(a2_1to2_2), .in_b(b1_2to2_2),  .out_a(a2_2to2_3), .out_b(b2_2to3_2), .out_c(matrixC2_2));
processing_element pe2_3(.reset(effective_rst), .clk(clk),  .in_a(a2_2to2_3), .in_b(b1_3to2_3),  .out_a(a2_3to2_4), .out_b(b2_3to3_3), .out_c(matrixC2_3));
processing_element pe2_4(.reset(effective_rst), .clk(clk),  .in_a(a2_3to2_4), .in_b(b1_4to2_4),  .out_a(a2_4to2_5), .out_b(b2_4to3_4), .out_c(matrixC2_4));
processing_element pe2_5(.reset(effective_rst), .clk(clk),  .in_a(a2_4to2_5), .in_b(b1_5to2_5),  .out_a(a2_5to2_6), .out_b(b2_5to3_5), .out_c(matrixC2_5));
processing_element pe2_6(.reset(effective_rst), .clk(clk),  .in_a(a2_5to2_6), .in_b(b1_6to2_6),  .out_a(a2_6to2_7), .out_b(b2_6to3_6), .out_c(matrixC2_6));
processing_element pe2_7(.reset(effective_rst), .clk(clk),  .in_a(a2_6to2_7), .in_b(b1_7to2_7),  .out_a(a2_7to2_8), .out_b(b2_7to3_7), .out_c(matrixC2_7));
processing_element pe3_1(.reset(effective_rst), .clk(clk),  .in_a(a3_0to3_1), .in_b(b2_1to3_1),  .out_a(a3_1to3_2), .out_b(b3_1to4_1), .out_c(matrixC3_1));
processing_element pe3_2(.reset(effective_rst), .clk(clk),  .in_a(a3_1to3_2), .in_b(b2_2to3_2),  .out_a(a3_2to3_3), .out_b(b3_2to4_2), .out_c(matrixC3_2));
processing_element pe3_3(.reset(effective_rst), .clk(clk),  .in_a(a3_2to3_3), .in_b(b2_3to3_3),  .out_a(a3_3to3_4), .out_b(b3_3to4_3), .out_c(matrixC3_3));
processing_element pe3_4(.reset(effective_rst), .clk(clk),  .in_a(a3_3to3_4), .in_b(b2_4to3_4),  .out_a(a3_4to3_5), .out_b(b3_4to4_4), .out_c(matrixC3_4));
processing_element pe3_5(.reset(effective_rst), .clk(clk),  .in_a(a3_4to3_5), .in_b(b2_5to3_5),  .out_a(a3_5to3_6), .out_b(b3_5to4_5), .out_c(matrixC3_5));
processing_element pe3_6(.reset(effective_rst), .clk(clk),  .in_a(a3_5to3_6), .in_b(b2_6to3_6),  .out_a(a3_6to3_7), .out_b(b3_6to4_6), .out_c(matrixC3_6));
processing_element pe3_7(.reset(effective_rst), .clk(clk),  .in_a(a3_6to3_7), .in_b(b2_7to3_7),  .out_a(a3_7to3_8), .out_b(b3_7to4_7), .out_c(matrixC3_7));
processing_element pe4_1(.reset(effective_rst), .clk(clk),  .in_a(a4_0to4_1), .in_b(b3_1to4_1),  .out_a(a4_1to4_2), .out_b(b4_1to5_1), .out_c(matrixC4_1));
processing_element pe4_2(.reset(effective_rst), .clk(clk),  .in_a(a4_1to4_2), .in_b(b3_2to4_2),  .out_a(a4_2to4_3), .out_b(b4_2to5_2), .out_c(matrixC4_2));
processing_element pe4_3(.reset(effective_rst), .clk(clk),  .in_a(a4_2to4_3), .in_b(b3_3to4_3),  .out_a(a4_3to4_4), .out_b(b4_3to5_3), .out_c(matrixC4_3));
processing_element pe4_4(.reset(effective_rst), .clk(clk),  .in_a(a4_3to4_4), .in_b(b3_4to4_4),  .out_a(a4_4to4_5), .out_b(b4_4to5_4), .out_c(matrixC4_4));
processing_element pe4_5(.reset(effective_rst), .clk(clk),  .in_a(a4_4to4_5), .in_b(b3_5to4_5),  .out_a(a4_5to4_6), .out_b(b4_5to5_5), .out_c(matrixC4_5));
processing_element pe4_6(.reset(effective_rst), .clk(clk),  .in_a(a4_5to4_6), .in_b(b3_6to4_6),  .out_a(a4_6to4_7), .out_b(b4_6to5_6), .out_c(matrixC4_6));
processing_element pe4_7(.reset(effective_rst), .clk(clk),  .in_a(a4_6to4_7), .in_b(b3_7to4_7),  .out_a(a4_7to4_8), .out_b(b4_7to5_7), .out_c(matrixC4_7));
processing_element pe5_1(.reset(effective_rst), .clk(clk),  .in_a(a5_0to5_1), .in_b(b4_1to5_1),  .out_a(a5_1to5_2), .out_b(b5_1to6_1), .out_c(matrixC5_1));
processing_element pe5_2(.reset(effective_rst), .clk(clk),  .in_a(a5_1to5_2), .in_b(b4_2to5_2),  .out_a(a5_2to5_3), .out_b(b5_2to6_2), .out_c(matrixC5_2));
processing_element pe5_3(.reset(effective_rst), .clk(clk),  .in_a(a5_2to5_3), .in_b(b4_3to5_3),  .out_a(a5_3to5_4), .out_b(b5_3to6_3), .out_c(matrixC5_3));
processing_element pe5_4(.reset(effective_rst), .clk(clk),  .in_a(a5_3to5_4), .in_b(b4_4to5_4),  .out_a(a5_4to5_5), .out_b(b5_4to6_4), .out_c(matrixC5_4));
processing_element pe5_5(.reset(effective_rst), .clk(clk),  .in_a(a5_4to5_5), .in_b(b4_5to5_5),  .out_a(a5_5to5_6), .out_b(b5_5to6_5), .out_c(matrixC5_5));
processing_element pe5_6(.reset(effective_rst), .clk(clk),  .in_a(a5_5to5_6), .in_b(b4_6to5_6),  .out_a(a5_6to5_7), .out_b(b5_6to6_6), .out_c(matrixC5_6));
processing_element pe5_7(.reset(effective_rst), .clk(clk),  .in_a(a5_6to5_7), .in_b(b4_7to5_7),  .out_a(a5_7to5_8), .out_b(b5_7to6_7), .out_c(matrixC5_7));
processing_element pe6_1(.reset(effective_rst), .clk(clk),  .in_a(a6_0to6_1), .in_b(b5_1to6_1),  .out_a(a6_1to6_2), .out_b(b6_1to7_1), .out_c(matrixC6_1));
processing_element pe6_2(.reset(effective_rst), .clk(clk),  .in_a(a6_1to6_2), .in_b(b5_2to6_2),  .out_a(a6_2to6_3), .out_b(b6_2to7_2), .out_c(matrixC6_2));
processing_element pe6_3(.reset(effective_rst), .clk(clk),  .in_a(a6_2to6_3), .in_b(b5_3to6_3),  .out_a(a6_3to6_4), .out_b(b6_3to7_3), .out_c(matrixC6_3));
processing_element pe6_4(.reset(effective_rst), .clk(clk),  .in_a(a6_3to6_4), .in_b(b5_4to6_4),  .out_a(a6_4to6_5), .out_b(b6_4to7_4), .out_c(matrixC6_4));
processing_element pe6_5(.reset(effective_rst), .clk(clk),  .in_a(a6_4to6_5), .in_b(b5_5to6_5),  .out_a(a6_5to6_6), .out_b(b6_5to7_5), .out_c(matrixC6_5));
processing_element pe6_6(.reset(effective_rst), .clk(clk),  .in_a(a6_5to6_6), .in_b(b5_6to6_6),  .out_a(a6_6to6_7), .out_b(b6_6to7_6), .out_c(matrixC6_6));
processing_element pe6_7(.reset(effective_rst), .clk(clk),  .in_a(a6_6to6_7), .in_b(b5_7to6_7),  .out_a(a6_7to6_8), .out_b(b6_7to7_7), .out_c(matrixC6_7));
processing_element pe7_1(.reset(effective_rst), .clk(clk),  .in_a(a7_0to7_1), .in_b(b6_1to7_1),  .out_a(a7_1to7_2), .out_b(b7_1to8_1), .out_c(matrixC7_1));
processing_element pe7_2(.reset(effective_rst), .clk(clk),  .in_a(a7_1to7_2), .in_b(b6_2to7_2),  .out_a(a7_2to7_3), .out_b(b7_2to8_2), .out_c(matrixC7_2));
processing_element pe7_3(.reset(effective_rst), .clk(clk),  .in_a(a7_2to7_3), .in_b(b6_3to7_3),  .out_a(a7_3to7_4), .out_b(b7_3to8_3), .out_c(matrixC7_3));
processing_element pe7_4(.reset(effective_rst), .clk(clk),  .in_a(a7_3to7_4), .in_b(b6_4to7_4),  .out_a(a7_4to7_5), .out_b(b7_4to8_4), .out_c(matrixC7_4));
processing_element pe7_5(.reset(effective_rst), .clk(clk),  .in_a(a7_4to7_5), .in_b(b6_5to7_5),  .out_a(a7_5to7_6), .out_b(b7_5to8_5), .out_c(matrixC7_5));
processing_element pe7_6(.reset(effective_rst), .clk(clk),  .in_a(a7_5to7_6), .in_b(b6_6to7_6),  .out_a(a7_6to7_7), .out_b(b7_6to8_6), .out_c(matrixC7_6));
processing_element pe7_7(.reset(effective_rst), .clk(clk),  .in_a(a7_6to7_7), .in_b(b6_7to7_7),  .out_a(a7_7to7_8), .out_b(b7_7to8_7), .out_c(matrixC7_7));
assign a_data_out = {a7_7to7_8,a6_7to6_8,a5_7to5_8,a4_7to4_8,a3_7to3_8,a2_7to2_8,a1_7to1_8,a0_7to0_8};
assign b_data_out = {b7_7to8_7,b7_6to8_6,b7_5to8_5,b7_4to8_4,b7_3to8_3,b7_2to8_2,b7_1to8_1,b7_0to8_0};

endmodule

module processing_element(
 reset, 
 clk, 
 in_a,
 in_b, 
 out_a, 
 out_b, 
 out_c
 );

 input reset;
 input clk;
 input  [`DWIDTH-1:0] in_a;
 input  [`DWIDTH-1:0] in_b;
 output [`DWIDTH-1:0] out_a;
 output [`DWIDTH-1:0] out_b;
 output [`DWIDTH-1:0] out_c;  //reduced precision

 reg [`DWIDTH-1:0] out_a;
 reg [`DWIDTH-1:0] out_b;
 wire [`DWIDTH-1:0] out_c;

 wire [`DWIDTH-1:0] out_mac;

 assign out_c = out_mac;

 seq_mac u_mac(.a(in_a), .b(in_b), .out(out_mac), .reset(reset), .clk(clk));

 always @(posedge clk)begin
    if(reset) begin
      out_a<=0;
      out_b<=0;
    end
    else begin  
      out_a<=in_a;
      out_b<=in_b;
    end
 end
 
endmodule

module seq_mac(a, b, out, reset, clk);
input [`DWIDTH-1:0] a;
input [`DWIDTH-1:0] b;
input reset;
input clk;
output [`DWIDTH-1:0] out;

reg [2*`DWIDTH-1:0] out_temp;
wire [`DWIDTH-1:0] mul_out;
wire [2*`DWIDTH-1:0] add_out;

reg [`DWIDTH-1:0] a_flopped;
reg [`DWIDTH-1:0] b_flopped;

wire [2*`DWIDTH-1:0] mul_out_temp;
reg [2*`DWIDTH-1:0] mul_out_temp_reg;

always @(posedge clk) begin
  if (reset) begin
    a_flopped <= 0;
    b_flopped <= 0;
  end else begin
    a_flopped <= a;
    b_flopped <= b;
  end
end

//assign mul_out = a * b;
qmult mult_u1(.i_multiplicand(a_flopped), .i_multiplier(b_flopped), .o_result(mul_out_temp));

always @(posedge clk) begin
  if (reset) begin
    mul_out_temp_reg <= 0;
  end else begin
    mul_out_temp_reg <= mul_out_temp;
  end
end

//we just truncate the higher bits of the product
//assign add_out = mul_out + out;
qadd add_u1(.a(out_temp), .b(mul_out_temp_reg), .c(add_out));

always @(posedge clk) begin
  if (reset) begin
    out_temp <= 0;
  end else begin
    out_temp <= add_out;
  end
end

//down cast the result
assign out = 
    (out_temp[2*`DWIDTH-1] == 0) ?  //positive number
        (
           (|(out_temp[2*`DWIDTH-2 : `DWIDTH-1])) ?  //is any bit from 14:7 is 1, that means overlfow
             {out_temp[2*`DWIDTH-1] , {(`DWIDTH-1){1'b1}}} : //sign bit and then all 1s
             {out_temp[2*`DWIDTH-1] , out_temp[`DWIDTH-2:0]} 
        )
        : //negative number
        (
           (|(out_temp[2*`DWIDTH-2 : `DWIDTH-1])) ?  //is any bit from 14:7 is 0, that means overlfow
             {out_temp[2*`DWIDTH-1] , out_temp[`DWIDTH-2:0]} :
             {out_temp[2*`DWIDTH-1] , {(`DWIDTH-1){1'b0}}} //sign bit and then all 0s
        );

endmodule

module qmult(i_multiplicand,i_multiplier,o_result);
input [`DWIDTH-1:0] i_multiplicand;
input [`DWIDTH-1:0] i_multiplier;
output [2*`DWIDTH-1:0] o_result;

assign o_result = i_multiplicand * i_multiplier;
//DW02_mult #(`DWIDTH,`DWIDTH) u_mult(.A(i_multiplicand), .B(i_multiplier), .TC(1'b1), .PRODUCT(o_result));

endmodule

module qadd(a,b,c);
input [2*`DWIDTH-1:0] a;
input [2*`DWIDTH-1:0] b;
output [2*`DWIDTH-1:0] c;

assign c = a + b;
//DW01_add #(`DWIDTH) u_add(.A(a), .B(b), .CI(1'b0), .SUM(c), .CO());
endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: matmul_8x8.v
//////////////////////////////////////////////////////////////////////

module matmul_unit(
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
 stall_matmul, 
 vmask,
 in_dst,
 in_dst_we,
 out_dst,
 out_dst_we,
 out_data_avail,
 out_dst_mask
);

parameter REGIDWIDTH=8;
parameter PIPE_STAGES_MATMUL=29;
parameter NUMLANES=8;
parameter MATMUL_STAGES = 35;

 input clk;
 input resetn;
 input activate;
// input [PIPE_STAGES_MATMUL:1] en;  //Enable for each pipestage
// input [PIPE_STAGES_MATMUL:1] squash;  //Squash for each pipestage
 input [NUMLANES-1:0] en;  //Enable for each pipestage
 input [NUMLANES-1:0] squash;  //Squash for each pipestage
 output stall;
 output reg stall_matmul;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] a_data;
 input [`MAT_MUL_SIZE*`DWIDTH-1:0] b_data;
 input [`MASK_WIDTH-1:0] validity_mask_a_rows;
 input [`MASK_WIDTH-1:0] validity_mask_a_cols;
 input [`MASK_WIDTH-1:0] validity_mask_b_rows;
 input [`MASK_WIDTH-1:0] validity_mask_b_cols;

 output [`MAT_MUL_SIZE*`DWIDTH-1:0] c_data;
 input [NUMLANES-1:0] vmask;

 input    [REGIDWIDTH-1:0] in_dst;
 input                     in_dst_we;
// output [PIPE_STAGES_MATMUL*REGIDWIDTH-1:0] out_dst;
// output            [PIPE_STAGES_MATMUL-1:0] out_dst_we;
// output   [PIPE_STAGES_MATMUL*NUMLANES-1:0] out_dst_mask;
 output [NUMLANES*REGIDWIDTH-1:0] out_dst;
 output            [NUMLANES-1:0] out_dst_we;
 output   [NUMLANES*NUMLANES-1:0] out_dst_mask;
 output out_data_avail;
//wire [PIPE_STAGES_MATMUL:1] ctrl_activate;
//wire [PIPE_STAGES_MATMUL:1] squash_activatepipe_NC;
wire [NUMLANES-1:0] ctrl_activate;
wire [NUMLANES-1:0] squash_activatepipe_NC;


//pipe #(1,PIPE_STAGES_MATMUL-1) activatepipe (
pipe #(1,NUMLANES) activatepipe (
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
    .c_data_available(out_data_avail),
    .validity_mask_a_rows(validity_mask_a_rows),
    .validity_mask_a_cols(validity_mask_a_cols),
    .validity_mask_b_rows(validity_mask_b_rows),
    .validity_mask_b_cols(validity_mask_b_cols)
);

reg [5:0] cnt;

always@(posedge clk)begin
   if(!resetn)begin
     cnt <= 0;
   end
   else begin
     if(!activate)
        cnt <= 0;
     else if(activate)
        cnt <= cnt+1;
   end
end

always@(*)begin
  if(cnt < NUMLANES-1)
    stall_matmul = 1'b0;
  else if (cnt >= MATMUL_STAGES)
    stall_matmul = 1'b0;
  else
    stall_matmul = 1'b1; 
end


//pipe #(REGIDWIDTH,PIPE_STAGES_MATMUL-1) dstpipe (
wire [NUMLANES-1:0] squash_dstpipe_NC;
pipe #(REGIDWIDTH,NUMLANES) dstpipe (
    .d(in_dst),  
    .clk(clk),
    .resetn(resetn),
    .en(en[NUMLANES-1:0] & {(NUMLANES){(~stall) | (~stall_matmul) }} ),
    .squash(squash_dstpipe_NC),
    .q(out_dst));

//pipe #(1,PIPE_STAGES_MATMUL-1) dstwepipe (
pipe #(1,NUMLANES) dstwepipe (
    .d(in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[NUMLANES-1:0] & {(NUMLANES){~stall| (~stall_matmul)}} ),
    .squash(squash[NUMLANES-1:1]),
    .q(out_dst_we));

wire [PIPE_STAGES_MATMUL:1] squash_dstmaskpipe_NC;
pipe #(NUMLANES,NUMLANES) dstmaskpipe (
    .d(vmask ),  
    .clk(clk),
    .resetn(resetn),
    .en(en[NUMLANES-1:0] & {(NUMLANES){~stall| (~stall_matmul)}} ),
    .squash(squash_dstmaskpipe_NC),
    .q(out_dst_mask));

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: matmul_unit.v
//////////////////////////////////////////////////////////////////////

`define LO(x,b) ((x)&~({1024{1'b1}}<<b))

module vlanes (
    clk,
    resetn,

    // Instruction interface
    instr,
    instr_en,     // tells when instr is valid and available
    instr_wait,   // if high says vpu is not ready to receive

    stall_in,
    is_stalled,
    has_memop,

    // Control register values 
    vc_in,
    vl_in,
    vbase_in,
    vinc_in,
    vstride_in,
    vs_in,
    matmul_masks_in,
    dma_en,
    mem_addr,
    lane_addr,
    dma_we,
    num_bytes,
    dma_busy,
    // vs Writeback
    vs_writedata,
    vs_we,       // Actually issues write command for vs (after done stalling)
    vs_wetrack,  // says there exist a scalar write operation in pipe
    vs_dst,

    //AXI interface
    M_AWID    ,
    M_AWADDR  ,
    M_AWLEN   ,
    M_AWSIZE  ,
    M_AWBURST ,
    M_AWLOCK  ,
    M_AWCACHE ,
    M_AWPROT  ,
    M_AWQOS   ,
    M_AWVALID ,
    M_AWREADY ,
    M_WDATA   ,
    M_WSTRB   ,
    M_WLAST   ,
    M_WVALID  ,
    M_WREADY  ,
    M_ARID    ,
    M_ARADDR  ,
    M_ARLEN   ,
    M_ARSIZE  ,
    M_ARBURST ,
    M_ARLOCK  ,
    M_ARCACHE ,
    M_ARPROT  ,
    M_ARQOS   ,
    M_ARVALID ,
    M_ARREADY ,
    M_RID     ,
    M_RDATA   ,
    M_RRESP   ,
    M_RLAST   ,
    M_RVALID  ,
    M_RREADY  ,
    M_BREADY  ,
    M_BVALID  ,
    M_BRESP   ,
    M_BID     ,

    S_AWID   , 
    S_AWADDR , 
    S_AWLEN  , 
    S_AWSIZE , 
    S_AWBURST, 
    S_AWLOCK , 
    S_AWCACHE, 
    S_AWPROT , 
    S_AWQOS  , 
    S_AWVALID, 
    S_AWREADY, 
    S_WDATA  , 
    S_WSTRB  , 
    S_WLAST  , 
    S_WVALID , 
    S_WREADY , 
    S_ARID   , 
    S_ARADDR , 
    S_ARLEN  , 
    S_ARSIZE , 
    S_ARBURST, 
    S_ARLOCK , 
    S_ARCACHE, 
    S_ARPROT , 
    S_ARQOS  , 
    S_ARVALID, 
    S_ARREADY, 
    S_RID    , 
    S_RDATA  , 
    S_RRESP  , 
    S_RLAST  , 
    S_RVALID , 
    S_RREADY , 
    S_BREADY , 
    S_BVALID , 
    S_BRESP  , 
    S_BID    ,  


    // Data memory interface
    dbus_address,
    dbus_en,
    dbus_we,
    dbus_byteen,
    dbus_writedata,
    dbus_readdata,
    dbus_cachematch,
    dbus_cachemiss,
    dbus_prefetch,    //Prefetch hint
    dbus_wait,

    dma_dbus_address,   
    dma_dbus_readdata,  
    dma_dbus_writedata, 
    dma_dbus_byteen,
    dma_dbus_en,        
    dma_dbus_wren,      
    dma_dbus_prefetch,  
    dma_dbus_wait,      
    dma_dbus_data_valid   
    );

parameter NUMLANES=4;
parameter LOG2NUMLANES=2;
parameter MVL=128;
parameter LOG2MVL=7;
parameter VPW=4;                  // VP width in number of bytes
parameter LOG2VPW=2;
parameter LANEWIDTH=8*VPW;
parameter LOG2LANEWIDTH=3+LOG2VPW;
parameter NUMBANKS=2;
parameter LOG2NUMBANKS=1;
parameter ALUPERBANK=1;
parameter NUMMEMPARALLELLANES=2;
parameter LOG2NUMMEMPARALLELLANES=1;
parameter NUMMULLANES=NUMLANES;
parameter DMEM_WRITEWIDTH=128;     // Width of write bus to memory
parameter LOG2DMEM_WRITEWIDTH=7;  // Log2 of Width of write bus to memory
parameter DMEM_READWIDTH=128;
parameter LOG2DMEM_READWIDTH=7;
parameter VCWIDTH=32;
parameter VSWIDTH=32;
parameter NUMVSREGS=32;
parameter LOG2NUMVSREGS=5;

parameter VRIDWIDTH=5;
parameter VELMIDWIDTH=LOG2MVL-LOG2NUMLANES;
parameter REGIDWIDTH=VRIDWIDTH+LOG2MVL-LOG2NUMLANES;
parameter BANKREGIDWIDTH=VRIDWIDTH+LOG2MVL-LOG2NUMLANES-LOG2NUMBANKS;

// Register identifier = { vr[0-31], element }

`define VRID_RANGE REGIDWIDTH-1:REGIDWIDTH-VRIDWIDTH
`define VRELM_RANGE REGIDWIDTH-VRIDWIDTH-1:0

// NUMFUS = ALU*NUMBANKS + MUL + MEM + FALU*MEMBANKS + MATMUL(1) + BFLOAT
// UNITS(2) + ACTIVATION + TRP + TRANSPOSE + PERMUTE
parameter NUMFUS=4+2*(NUMBANKS-1)*ALUPERBANK+1+4+2 + 1; //Adding 1 for matmul FU + 6 for BFLOTs, ACTIVATION, TRP, PERMUTE and TRANSPOSE and AXI
parameter FU_ALU=0;
parameter FU_MUL=FU_ALU+(NUMBANKS-1)*ALUPERBANK+1;
parameter FU_MEM=FU_MUL+1;
parameter FU_FALU=FU_MEM+1;
parameter FU_MATMUL=FU_FALU+1;
parameter FU_BFADDER = FU_MATMUL + 1;
parameter FU_BFMULT = FU_BFADDER + 1;
parameter FU_ACT = FU_BFMULT + 1;
parameter FU_TRP = FU_ACT + 1;
parameter FU_TRANSPOSE = FU_TRP + 1;
parameter FU_PERMUTE = FU_TRANSPOSE + 1;
parameter FU_AXI = FU_PERMUTE + 1;

input clk;
input resetn;

//Instruction Format:
//9 - op[21] mask 1/0
//8 - op[23] scalar/vec src2
//7 - op[22] scalar/vec src1
//6:0 - {op[24],op[5:0]} op

input [31:0] instr;
input instr_en;     // tells when instr is valid and available
output instr_wait;   // if high says vpu is not ready to receive

input [3:0] stall_in;  
output [`MAX_PIPE_STAGES-1:0] is_stalled;
output has_memop;
output dma_busy;

// Control register values
input  [ VCWIDTH-1 : 0 ]   vc_in;
input  [ VCWIDTH-1 : 0 ]   vl_in;
input  [ VCWIDTH-1 : 0 ]   vbase_in;
input  [ VCWIDTH-1 : 0 ]   vinc_in;
input  [ VCWIDTH-1 : 0 ]   vstride_in;
input  [ VSWIDTH-1 : 0 ]   vs_in;
input  [3*`MAT_MUL_SIZE-1 : 0]  matmul_masks_in;
input  dma_en;
input  [VCWIDTH-1:0] mem_addr;
input  [VCWIDTH-1:0] lane_addr;
input  dma_we;
input  [VCWIDTH-1:0] num_bytes;

// vs Writeback
output       [ VSWIDTH-1 : 0 ]   vs_writedata;
output                           vs_we;
output               [ 5 : 2 ]   vs_wetrack;
output [ LOG2NUMVSREGS-1 : 0 ]   vs_dst;

// Data memory interface
output  [ 31 : 0 ]  dbus_address;
output              dbus_en;
output              dbus_we;
output  [ (DMEM_WRITEWIDTH/8)-1 : 0 ]   dbus_byteen;
output  [ DMEM_WRITEWIDTH-1 : 0 ]  dbus_writedata;
input   [ DMEM_READWIDTH-1 : 0 ]  dbus_readdata;
input               dbus_cachematch;
input               dbus_cachemiss;
input               dbus_wait;
output  [ 31 : 0 ]  dbus_prefetch;

//DMA signals
output [31:0]                 dma_dbus_address;   
input [DMEM_READWIDTH-1:0]    dma_dbus_readdata;  
output [DMEM_WRITEWIDTH-1:0]  dma_dbus_writedata; 
output [DMEM_WRITEWIDTH/8-1:0]dma_dbus_byteen;
output                        dma_dbus_en;        
output                        dma_dbus_wren;      
output                        dma_dbus_prefetch;  
input                         dma_dbus_wait;      
input                         dma_dbus_data_valid;


// AXI interface
//
output  [6 - 1:0]                    M_AWID;                                                 
output  [LANEWIDTH- 1:0]             M_AWADDR;                                    
output  [7:0]                        M_AWLEN;                                                    
output  [2:0]                        M_AWSIZE;                                                   
output  [1:0]                        M_AWBURST;                                                  
output                               M_AWLOCK;                                                         
output  [3:0]                        M_AWCACHE;                                                  
output  [2:0]                        M_AWPROT;                                                   
output  [3:0]                        M_AWQOS;                                                    
output                               M_AWVALID;                                                        
input                                M_AWREADY;                                                        
output  [NUMLANES*LANEWIDTH-1 : 0]   M_WDATA;                                     
output  [(NUMLANES*LANEWIDTH)/8-1 : 0] M_WSTRB;                                   
output                               M_WLAST;                                                          
output                               M_WVALID;                                                         
input                                M_WREADY;                                                         

output  [6-1 : 0]                    M_ARID;                                                 
output  [LANEWIDTH-1 : 0]            M_ARADDR;                                    
output  [7 : 0]                      M_ARLEN;                                                  
output  [2 : 0]                      M_ARSIZE;                                                 
output  [1 : 0]                      M_ARBURST;                                                
output                               M_ARLOCK;                                                         
output  [3 : 0]                      M_ARCACHE;                                                
output  [2 : 0]                      M_ARPROT;                                                 
output  [3 : 0]                      M_ARQOS;                                                  
output                               M_ARVALID;                                                        
input                                M_ARREADY;                                                        
input   [6-1 : 0]                    M_RID;                                         	    
input   [NUMLANES*LANEWIDTH-1 : 0]   M_RDATA;                                     
input   [1 : 0]                      M_RRESP;                                                  
input                                M_RLAST;                                                          
input                                M_RVALID;                                                         
output                               M_RREADY;                                                         

output                               M_BREADY;                                                         
input                                M_BVALID;                                                         
input   [1 : 0]                      M_BRESP;                                                  
input   [6-1 : 0]                    M_BID;                                                   

input  [6 - 1:0]                    S_AWID;                                                 
input  [LANEWIDTH- 1:0]             S_AWADDR;                                    
input  [7:0]                        S_AWLEN;                                                    
input  [2:0]                        S_AWSIZE;                                                   
input  [1:0]                        S_AWBURST;                                                  
input                               S_AWLOCK;                                                         
input  [3:0]                        S_AWCACHE;                                                  
input  [2:0]                        S_AWPROT;                                                   
input  [3:0]                        S_AWQOS;                                                    
input                               S_AWVALID;                                                        
output                                S_AWREADY;                                                        
input  [NUMLANES*LANEWIDTH-1 : 0]   S_WDATA;                                     
input  [(NUMLANES*LANEWIDTH)/8-1 : 0] S_WSTRB;                                   
input                               S_WLAST;                                                          
input                               S_WVALID;                                                         
output                                S_WREADY;                                                         

input  [6-1 : 0]                    S_ARID;                                                 
input  [LANEWIDTH-1 : 0]            S_ARADDR;                                    
input  [7 : 0]                      S_ARLEN;                                                  
input  [2 : 0]                      S_ARSIZE;                                                 
input  [1 : 0]                      S_ARBURST;                                                
input                               S_ARLOCK;                                                         
input  [3 : 0]                      S_ARCACHE;                                                
input  [2 : 0]                      S_ARPROT;                                                 
input  [3 : 0]                      S_ARQOS;                                                  
input                               S_ARVALID;                                                        
output                                S_ARREADY;                                                        
output   [6-1 : 0]                    S_RID;                                         	    
output   [NUMLANES*LANEWIDTH-1 : 0]   S_RDATA;                                     
output   [1 : 0]                      S_RRESP;                                                  
output                                S_RLAST;                                                          
output                                S_RVALID;                                                         
input                               S_RREADY;                                                         

input                               S_BREADY;                                                         
output                                S_BVALID;                                                         
output   [1 : 0]                      S_BRESP;                                                  
output   [6-1 : 0]                    S_BID;     

//

//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: visa.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/visa.v
//////////////////////////////////////////////////////////////////////
parameter COP2_VADD           = 'b1000000000;
parameter COP2_VADD_U         = 'b1000000001;
parameter COP2_VSUB           = 'b1000000010;
parameter COP2_VSUB_U         = 'b1000000011;
parameter COP2_VMULHI         = 'b1000000100;
parameter COP2_VMULHI_U       = 'b1000000101;
parameter COP2_VDIV           = 'b1000000110; //Using as matmul
//parameter COP2_VBFADD         = 'b0100000001; //Using BF16 add
//parameter COP2_VBFMULT        = 'b0100000010; //Using BF16 MULT
//parameter COP2_VACT           = 'b0100000011; //Using ACT
//parameter COP2_VTRP           = 'b0100000100; //Using ACT
parameter COP2_VDIV_U         = 'b1000000111;

//parameter COP2_VMOD           = 'b1000001000;
parameter COP2_VBFADD           = 'b1000001000;  // USING bfloat add Instr: vmod.vv vrdest, vrsrc1,vrsrc2
parameter COP2_VBFMULT         = 'b1000001001;  // Using bfloat mult Instr: vmod.u.vv vrdest, vrsrc1,vrsrc2
parameter COP2_VMOD_U         = 'b1000001001;
parameter COP2_VCMP_EQ        = 'b1000001010;
parameter COP2_VCMP_NE        = 'b1000001100;
parameter COP2_VCMP_LT        = 'b1000001110;
parameter COP2_VCMP_U_LT      = 'b1000001111;
parameter COP2_VCMP_LE        = 'b1000010000;
parameter COP2_VCMP_U_LE      = 'b1000010001;
parameter COP2_VMIN           = 'b1000010010;
parameter COP2_VMIN_U         = 'b1000010011;
parameter COP2_VMAX           = 'b1000010100;
parameter COP2_VMAX_U         = 'b1000010101;
parameter COP2_VMULLO         = 'b1000010110;
parameter COP2_VABS           = 'b1000010111;
parameter COP2_VAND           = 'b1000011000;
parameter COP2_VOR            = 'b1000011001;
parameter COP2_VXOR           = 'b1000011010;
parameter COP2_VNOR           = 'b1000011011;
parameter COP2_VSLL           = 'b1000011100;
parameter COP2_VSRL           = 'b1000011101;
parameter COP2_VSRA           = 'b1000011110;
parameter COP2_VSAT_B         = 'b1000011111;
parameter COP2_VSAT_H         = 'b1001011111;
//parameter COP2_VSAT_W         = 'b1010011111;
parameter COP2_VACT         = 'b1010011111;  // Using activation instruction: vsat.w vrdest,vrsrc
parameter COP2_VSAT_SU_B      = 'b1000100000;
parameter COP2_VSAT_SU_H      = 'b1001100000;
//parameter COP2_VSAT_SU_W      = 'b1010100000;
parameter COP2_VRED     = 'b1010100000;   // Using reduction instruction: vsat.su.w vrdest,vrsrc
parameter COP2_VSAT_SU_L      = 'b1011100000;
parameter COP2_VSAT_U_B       = 'b1000100001;
parameter COP2_VSAT_U_H       = 'b1001100001;
//parameter COP2_VSAT_U_W       = 'b1010100001;
parameter COP2_VTRP       = 'b1010100001;   // Using transpose instruction: vsat.u.w vrdest, vrsrc 
parameter COP2_VSADD          = 'b1000100010;
parameter COP2_VSADD_U        = 'b1000100011;
parameter COP2_VSSUB          = 'b1000100100;
parameter COP2_VSSUB_U        = 'b1000100101;
parameter COP2_VSRR           = 'b1000100110;
parameter COP2_VSRR_U         = 'b1000100111;
parameter COP2_VSLS           = 'b1000101000;
parameter COP2_VSLS_U         = 'b1000101001;
parameter COP2_VXUMUL         = 'b1000101010;
parameter COP2_VXUMUL_U       = 'b1000101011;
parameter COP2_VXLMUL         = 'b1000101100;
parameter COP2_VXLMUL_U       = 'b1000101101;
parameter COP2_VXUMADD        = 'b1000101110;
parameter COP2_VXUMADD_U      = 'b1000101111;
parameter COP2_VXUMSUB        = 'b1000110000;
parameter COP2_VXUMSUB_U      = 'b1000110001;
parameter COP2_VXLMADD        = 'b1000110010;
parameter COP2_VXLMADD_U      = 'b1000110011;
parameter COP2_VXLMSUB        = 'b1000110100;
parameter COP2_VXLMSUB_U      = 'b1000110101;
parameter COP2_VINS_VV        = 'b1100000000;
parameter COP2_VINS_SV        = 'b1110000001;
parameter COP2_VEXT_VV        = 'b1100000010;
parameter COP2_VEXT_SV        = 'b1100000011;
parameter COP2_VEXT_U_SV      = 'b1100000100;
parameter COP2_VCOMPRESS      = 'b1100000101;
parameter COP2_VEXPAND        = 'b1100000110;
parameter COP2_VMERGE         = 'b1100000111;
parameter COP2_VFINS          = 'b1110001000;
parameter COP2_VEXTHALF       = 'b1100001001;
parameter COP2_VHALF          = 'b1100001010;
parameter COP2_VHALFUP        = 'b1100001011;
parameter COP2_VHALFDN        = 'b1100001100;
parameter COP2_VSATVL         = 'b1100001101;
parameter COP2_VFAND          = 'b1100001110;
parameter COP2_VFOR           = 'b1100001111;
parameter COP2_VFXOR          = 'b1100010000;
parameter COP2_VFNOR          = 'b1100010001;
parameter COP2_VFCLR          = 'b1100010010;
parameter COP2_VFSET          = 'b1100010011;
parameter COP2_VIOTA          = 'b1100010100;
parameter COP2_VCIOTA         = 'b1100010101;
parameter COP2_VFPOP          = 'b1100010110;
parameter COP2_VFFF1          = 'b1100010111;
parameter COP2_VFFL1          = 'b1100011000;
parameter COP2_VFSETBF        = 'b1100011001;
parameter COP2_VFSETIF        = 'b1100011010;
parameter COP2_VFSETOF        = 'b1100011011;
parameter COP2_VFMT8          = 'b1100011100;
parameter COP2_VFMF8          = 'b1100011101;
parameter COP2_VFCLR8         = 'b1100011110;
parameter COP2_VFOR8          = 'b1100011111;
parameter COP2_VFLD           = 'b1100100000;
parameter COP2_VLD_B          = 'b1100100001;
parameter COP2_VLD_H          = 'b1101100001;

//parameter COP2_VLD_W          = 'b1110100001;
//parameter COP2_VBFADD         = 'b1110100001;  // adding bfadder Instr: vld.u.w

parameter COP2_VLD_L          = 'b1111100001;
parameter COP2_VLD_U_B        = 'b1100100010;
parameter COP2_VLD_U_H        = 'b1101100010;

//parameter COP2_VLD_U_W        = 'b1110100010;
parameter COP2_VAXIRD	        = 'b1110100010;  // adding an instruction for AXI Load;

parameter COP2_VLDS_B         = 'b1100100011;
parameter COP2_VLDS_H         = 'b1101100011;

//parameter COP2_VLDS_W         = 'b1110100011;
parameter COP2_VBFSUB         = 'b1110100011;   // adding bfsub Instr: vlds.w

parameter COP2_VLDS_L         = 'b1111100011;
parameter COP2_VLDS_U_B       = 'b1100100100;
parameter COP2_VLDS_U_H       = 'b1101100100;

//parameter COP2_VLDS_U_W       = 'b1110100100;
//parameter COP2_VBFMULT         = 'b1110100100;   // adding bfmult Instr: vlds.u.w

parameter COP2_VLDX_B         = 'b1100100101;
parameter COP2_VLDX_H         = 'b1101100101;

//parameter COP2_VLDX_W         = 'b1110100101;
parameter COP2_VPER_STR         = 'b1110100101;     // adding transpose instruction: vldx.w

parameter COP2_VLDX_L         = 'b1111100101;
parameter COP2_VLDX_U_B       = 'b1100100110;
parameter COP2_VLDX_U_H       = 'b1101100110;

//parameter COP2_VLDX_U_W       = 'b1110100110;
//parameter COP2_VPER     = 'b1110100110;        //adding activation Instr: vldx.u.w 

parameter COP2_VFST           = 'b1100101000;
parameter COP2_VST_B          = 'b1100101001;
parameter COP2_VST_H          = 'b1101101001;

//parameter COP2_VST_W        = 'b1110101001;  // adding reduction Instr: vst.w
//parameter COP2_VRED           = 'b1110101001;  // adding reduction Instr: vst.w

parameter COP2_VST_L          = 'b1111101001;
parameter COP2_VSTS_B         = 'b1100101010;
parameter COP2_VSTS_H         = 'b1101101010;

//parameter COP2_VSTS_W         = 'b1110101010;
parameter COP2_VPER_LD           = 'b1110101010;  // adding permute Instr: vsts.w

parameter COP2_VSTS_L         = 'b1111101010;
parameter COP2_VSTX_B         = 'b1100101011;
parameter COP2_VSTX_H         = 'b1101101011;
//parameter COP2_VSTX_W         = 'b1110101011;
parameter COP2_VPER         = 'b1110101011;  // adding permute operation: vstx.w Vsrc,Vbase, 
parameter COP2_VSTX_L         = 'b1111101011;
parameter COP2_VSTXO_B        = 'b1100101100;
parameter COP2_VSTXO_H        = 'b1101101100;
//parameter COP2_VSTXO_W        = 'b1110101100;  
parameter COP2_VAXIWR        = 'b1110101100;  // adding an instruction for axi write
parameter COP2_VSTXO_L        = 'b1111101100;
parameter COP2_VMCTS          = 'b1101110000;
parameter COP2_VMSTC          = 'b1101110001;
parameter COP2_CFC2           = 'b0000111000;
parameter COP2_CTC2           = 'b0000111010;
parameter COP2_MTC2           = 'b0000111011;
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: visa.v
//////////////////////////////////////////////////////////////////////

parameter BIT_VSSRC2=6;
parameter BIT_VSSRC1=7;

parameter ALUOP_ZERO    =11'b00100000101,
          ALUOP_ADD     =11'b00010000000,
          ALUOP_ADDU    =11'b00000000000,
          ALUOP_SUB     =11'b00011000000,
          ALUOP_SUBU    =11'b00001000000,
          ALUOP_CMP_EQ  =11'b00011000000,
          ALUOP_CMP_NEQ =11'b00011000001,
          ALUOP_CMP_LT  =11'b00011000010,
          ALUOP_CMP_LE  =11'b00011000011,
          ALUOP_CMP_LT_U=11'b00001000010,
          ALUOP_CMP_LE_U=11'b00001000011,
          ALUOP_AND     =11'b00000000101,
          ALUOP_OR      =11'b00000000001,
          ALUOP_XOR     =11'b00000001001,
          ALUOP_NOR     =11'b00000001101,
          ALUOP_MIN     =11'b00111010001,
          ALUOP_MIN_U   =11'b00101010001,
          ALUOP_MAX     =11'b00111100001,
          ALUOP_MAX_U   =11'b00101100001,
          ALUOP_ABS     =11'b11111000010,
          ALUOP_MERGE   =11'b10100000011;

parameter SATSUMOP_NOP=2'b00,
          SATSUMOP_VS =2'b11,
          SATSUMOP_VSU=2'b10;

parameter SATSIZEOP_VSATUW=4'b0000,
          SATSIZEOP_VSATUB=4'b0001,
          SATSIZEOP_VSATUH=4'b0010,
          SATSIZEOP_VSATW  =4'b1100,
          SATSIZEOP_VSATB  =4'b1101,
          SATSIZEOP_VSATH  =4'b1110,
          SATSIZEOP_VSATSUW=4'b0100,
          SATSIZEOP_VSATSUB=4'b0101,
          SATSIZEOP_VSATSUH=4'b0110;

parameter MULOP_ZERO  =5'b00000,
          MULOP_LMULU =5'b10110,
          MULOP_UMULU =5'b10111,
          MULOP_LMUL  =5'b10100,
          MULOP_UMUL  =5'b10101,
          MULOP_MULUHI=5'b00111,
          MULOP_MULULO=5'b00110,
          MULOP_MULHI =5'b00101,
          MULOP_MULLO =5'b00100,
          MULOP_SLL   =5'b00010,
          MULOP_SLS   =5'b01010,
          MULOP_SLSU  =5'b01000,
          MULOP_SRL   =5'b00011,
          MULOP_SRA   =5'b00001;

parameter MEMOP_SHIFT=7'b0000000, 
          MEMOP_LDUB=7'b1000000,
          MEMOP_LDUH=7'b1000100,
          MEMOP_LDUW=7'b1001000,
          MEMOP_LDB=7'b1000010,
          MEMOP_LDH=7'b1000110,
          MEMOP_LDW=7'b1001010,
          MEMOP_LDSUB=7'b1010000,
          MEMOP_LDSUH=7'b1010100,
          MEMOP_LDSUW=7'b1011000,
          MEMOP_LDSB=7'b1010010,
          MEMOP_LDSH=7'b1010110,
          MEMOP_LDSW=7'b1011010,
          MEMOP_LDXUB=7'b1100000,
          MEMOP_LDXUH=7'b1100100,
          MEMOP_LDXUW=7'b1101000,
          MEMOP_LDXB=7'b1100010,
          MEMOP_LDXH=7'b1100110,
          MEMOP_LDXW=7'b1101010,
          MEMOP_STB=7'b1000001,
          MEMOP_STH=7'b1000101,
          MEMOP_STW=7'b1001001,
          MEMOP_STSB=7'b1010001,
          MEMOP_STSH=7'b1010101,
          MEMOP_STSW=7'b1011001,
          MEMOP_STXB=7'b1100001,
          MEMOP_STXH=7'b1100101,
          MEMOP_STXW=7'b1101001;

parameter FLAGOP_AND=3'b000,
          FLAGOP_OR= 3'b001,
          FLAGOP_XOR=3'b010,
          FLAGOP_NOR=3'b011,
          FLAGOP_CLR=3'b100,
          FLAGOP_SET=3'b101;

wire                        [ 9 : 0 ]   ir_op;
wire                        [ 4 : 0 ]   ir_src2;
wire                        [ 4 : 0 ]   ir_src1;
wire                        [ 4 : 0 ]   ir_dst;
wire                                    ir_mask;

// Control register saved values
reg                       pipe_advance_s2_r;
reg   [ VCWIDTH-1 : 0 ]   vc_in_saved;
reg   [ VCWIDTH-1 : 0 ]   vl_in_saved;
reg   [ VCWIDTH-1 : 0 ]   vbase_in_saved;
reg   [ VCWIDTH-1 : 0 ]   vinc_in_saved;
reg   [ VCWIDTH-1 : 0 ]   vstride_in_saved;
reg   [ VSWIDTH-1 : 0 ]   vs_in_saved;

// Control register values
//TODO: Rethink whether the following vars (vc, vl, vbase, vinc, vstride)
//need to be change to use `MAX_PIPE_STAGES. I think these variables are 
//only used for memory operations and for that, we only need 6 stages.
//Not a big deal because most of these are not used much. Synthesis tool
//will optimize out the unnecessary bits anyway.
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((VCWIDTH-1)-(0)+1))-1 : 0] vc;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((VCWIDTH-1)-(0)+1))-1 : 0] vl;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((VCWIDTH-1)-(0)+1))-1 : 0] vbase;
reg   [ VCWIDTH-1 : 0 ]   vbase_s4;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((VCWIDTH-1)-(0)+1))-1 : 0] vinc;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((VCWIDTH-1)-(0)+1))-1 : 0] vstride;
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((VSWIDTH-1)-(0)+1))-1 : 0] vs;
reg [(((NUMBANKS-1)-(0)+1)*((VSWIDTH-1)-(0)+1))-1 : 0] vs_s3;
reg [(((NUMFUS-1)-(0)+1)*((VSWIDTH-1)-(0)+1))-1 : 0] vs_s4;
reg [(((NUMBANKS-1)-(0)+1)*((VCWIDTH-1)-(0)+1))-1 : 0] vc_s3;
reg [(((NUMFUS-1)-(0)+1)*((VCWIDTH-1)-(0)+1))-1 : 0] vc_s4;

// Vector register file signals
reg   [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vr_a_reg;
wire     [ NUMBANKS*8*VPW*NUMLANES-1 : 0 ]   _vr_a_readdataout;
reg                         [NUMBANKS-1:0]   vr_a_en;
reg   [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vr_b_reg;
wire     [ NUMBANKS*8*VPW*NUMLANES-1 : 0 ]   _vr_b_readdataout;
reg                         [NUMBANKS-1:0]   vr_b_en;
reg   [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   _vr_c_reg;
reg      [ NUMBANKS*8*VPW*NUMLANES-1 : 0 ]   _vr_c_writedatain;
reg        [ NUMBANKS*VPW*NUMLANES-1 : 0 ]   vr_c_byteen;
reg                         [NUMBANKS-1:0]   vr_c_we;

reg [(((NUMBANKS-1)-(0)+1)*((LANEWIDTH*NUMLANES-1)-(0)+1))-1 : 0] vr_a_readdataout;
reg [(((NUMBANKS-1)-(0)+1)*((LANEWIDTH*NUMLANES-1)-(0)+1))-1 : 0] vr_b_readdataout;
reg [(((NUMBANKS-1)-(0)+1)*((LANEWIDTH*NUMLANES-1)-(0)+1))-1 : 0] vr_c_writedatain;
reg [(((NUMBANKS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] vr_c_reg;

// Flag register file signals
reg       [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vf_a_reg;
wire               [ NUMBANKS*NUMLANES-1 : 0 ]   vf_a_readdataout;
reg                            [ NUMBANKS-1:0]   vf_a_en;
reg       [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vf_b_reg;
wire               [ NUMBANKS*NUMLANES-1 : 0 ]   vf_b_readdataout;
reg                            [ NUMBANKS-1:0]   vf_b_en;
reg       [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vf_c_reg;
reg                [ NUMBANKS*NUMLANES-1 : 0 ]   vf_c_writedatain;
reg                            [ NUMBANKS-1:0]   vf_c_we;

wire             [ REGIDWIDTH*NUMFUS-1 : 0 ]   wb_dst;
wire                     [NUMFUS-1:0]   wb_dst_we;
wire [(((NUMFUS-1)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] wb_dst_mask;

//wire [(((NUMFUS-1)-(0)+1)*((`MAX_STAGES-1)-(4)+1))-1 : 0] dst_we;
wire [(`MAX_STAGES-4)*NUMFUS-1:0] dst_we;
wire [(`MAX_STAGES-4) * NUMFUS * REGIDWIDTH-1 : 0 ] dst;

wire [(((`MAX_STAGES-1)-(4)+1)*((NUMLANES-1)-(0)+1)*NUMFUS)-1 : 0] dst_mask;
wire             [ REGIDWIDTH-1 : 0 ]   dst_s2;
reg [(((NUMBANKS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] _dst_s3;
reg [(((NUMBANKS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] dst_s3;
reg     [ NUMBANKS*REGIDWIDTH-1 : 0 ]   t_dst_s3;
reg [(((NUMFUS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] dst_s4;
reg                  [ NUMFUS-1 : 0 ]   dst_we_s4;
reg [(((4)-(4)+1)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1))-1 : 0] alu_dst;
reg [(((4)-(4)+1)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1))-1 : 0] alu_dst_we;
reg [(((4)-(4)+1)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1))-1 : 0] falu_dst;
reg [(((4)-(4)+1)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1))-1 : 0] falu_dst_we;

wire                                    imask_s2;
reg                [ NUMBANKS-1 : 0 ]   imask_s3;

wire             [ REGIDWIDTH-1 : 0 ]   src1_s2;
wire             [ REGIDWIDTH-1 : 0 ]   src2_s2;
reg [(((NUMBANKS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] src1_s3;
reg [(((NUMBANKS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] src2_s3;
reg [NUMBANKS*REGIDWIDTH-1:0] _src1_s3;
reg [(((NUMBANKS-1)-(0)+1)*((REGIDWIDTH-1)-(0)+1))-1 : 0] _src2_s3;
wire                                    src1scalar_s2;
wire                                    src2scalar_s2;
reg                    [NUMBANKS-1:0]   src1scalar_s3;
reg                    [NUMBANKS-1:0]   src2scalar_s3;
reg                      [NUMFUS-1:0]   src1scalar_s4;
reg                      [NUMFUS-1:0]   src2scalar_s4;

reg [(((NUMBANKS-1)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] lane_en;
reg [(((NUMFUS-1)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] vlane_en;
reg                                     mem_last_subvector_s4;

reg                     [LOG2MVL-1:0]   src_start_delayed;
wire                    [LOG2MVL-1:0]   src_elm;
wire                    [LOG2MVL-1:0]   src_limit;
reg                     [LOG2MVL*NUMBANKS-1:0]   src_limit_s3;
wire                    [LOG2MVL-1:0]   src_start;

wire                    [VCWIDTH-1:0]   total_shamt;
wire                    [LOG2MVL-1:0]   dst_start;
             //output of various blocks
reg [(((NUMFUS-1)-(0)+1)*((LANEWIDTH*NUMLANES-1)-(0)+1))-1 : 0] vr_src1;
reg [(((NUMFUS-1)-(0)+1)*((LANEWIDTH*NUMLANES-1)-(0)+1))-1 : 0] vr_src2;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   matmul_out;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   bfadder_result_s5;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   bfmult_result_s5;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   act_result_s5;
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   trp_out; //output of reduction unit
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   permute_out; //output of permute block
wire         [ LANEWIDTH*NUMLANES-1 : 0 ]   transpose_out;
reg [(((NUMFUS-1)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] vf_src1;
reg [(((NUMFUS-1)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] vf_src2;
reg [(((NUMFUS-1)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] vmask;
reg [(((NUMBANKS-1)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] vmask_final;

reg        [ VCWIDTH*NUMLANES-1 : 0 ]   vstrideoffset_s4;
wire     [ LANEWIDTH*NUMLANES-1 : 0 ]   load_result_s5;
wire               [ NUMLANES-1 : 0 ]   load_result_mask_s5;

wire [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((LANEWIDTH*NUMLANES-1)-(0)+1))-1 : 0] alu_result_s5;
wire [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] alu_cmpresult_s4;
wire [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] flagalu_result_s4;
wire [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((NUMLANES-1)-(0)+1))-1 : 0] flagalu_result_s5;
wire [ LANEWIDTH*NUMLANES-1 : 0 ] mulshift_result_s5;

//Support 1 Lane processor
wire [(((`MAX_PIPE_STAGES-1)-(2)+1)*((((LOG2NUMLANES>0) ? LOG2NUMLANES : 1)-1)-(0)+1))-1 : 0] elmshamt;

//control for TRP unit

wire [1:0] trp_mode_s1,trp_mode_s2;
reg [(((NUMBANKS-1)-(0)+1)*((1)-(0)+1))-1 : 0] trp_mode_s3;
reg [1:0] trp_mode_s4;
wire trp_busy;
wire trp_valid;
wire trp_read;

wire transpose_valid;
wire transpose_busy;

//reg permute_read;
//reg permute_store;
//reg permute_op;

reg ctrl1_vr_a_en; // SRC1
reg ctrl1_vr_b_en; // SRC2
reg ctrl1_vr_c_en; // SRC3
reg ctrl1_vr_d_we; // DEST
reg ctrl1_vf_a_en;
reg ctrl1_vf_b_en;
reg ctrl1_vf_c_we;
reg ctrl1_vs_we;
reg ctrl1_usesvssel;    // 1-if instruction can have .sv/.vs variants
reg [1:0] ctrl1_elmshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
reg [1:0] ctrl1_srcshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
reg ctrl1_srclimit_sel;  //0-vl, 1 vl+vindex
reg [1:0] ctrl1_dstshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
reg ctrl1_setvlto1;
reg ctrl1_mem_dir_left;  //1-left, 0-right
reg ctrl1_rshiftnonzero;
reg [10:0] ctrl1_alu_op;
reg [1:0] ctrl1_satsum_op;
reg [3:0] ctrl1_satsize_op;
reg [4:0] ctrl1_mulshift_op;
reg ctrl1_matmul_en;
reg ctrl1_bfadder_en;
reg ctrl1_bfmult_en;
reg ctrl1_act_en;
reg ctrl1_trp_en;
reg ctrl1_permute_en;
reg ctrl1_permute_op;
reg ctrl1_permute_store;
reg ctrl1_permute_read;
reg ctrl1_transpose_en;
reg ctrl1_axi_en;
reg ctrl1_axi_req_type;
reg ctrl1_memunit_en;
reg ctrl1_mem_en;
reg [6:0] ctrl1_memunit_op;
reg ctrl1_ismasked;
reg [2:0] ctrl1_flagalu_op;
reg ctrl1_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg ctrl1_volatiledest;
reg ctrl1_vrdest_sel;  //0-dest, 1-src2
reg ctrl1_vf_a_sel; //0-0/1 from instr, 1-src1
reg bf_op;

wire [5:4] ctrl_vs_we;
wire [6:0] ctrl_memunit_op[`MAX_PIPE_STAGES-1:1] ;

wire ctrl2_vr_a_en; // SRC1
wire ctrl2_vr_b_en; // SRC2
wire ctrl2_vr_c_we; // DEST
wire ctrl2_vf_a_en;
wire ctrl2_vf_b_en;
wire ctrl2_vf_c_we;
wire ctrl2_vs_we;
wire ctrl2_useslanes;
wire [1:0] ctrl2_elmshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
wire [1:0] ctrl2_srcshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
wire ctrl2_srclimit_sel;  //0-vl, 1 vl+vindex
wire [1:0] ctrl2_dstshamt_sel;   //0-0, 3-vcindex, 1-vl>>1, 2-2^vcindex
wire ctrl2_setvlto1;
wire ctrl2_mem_dir_left;  //1-left, 0-right
wire ctrl2_rshiftnonzero;
wire [10:0] ctrl2_alu_op;
wire [1:0] ctrl2_satsum_op;
wire [3:0] ctrl2_satsize_op;
wire [4:0] ctrl2_mulshift_op;
wire ctrl2_memunit_en;
wire ctrl2_mem_en;
wire [6:0] ctrl2_memunit_op;
wire ctrl2_ismasked;
wire [2:0] ctrl2_flagalu_op;
wire ctrl2_vf_wbsel;   //0-flag ALU, 1-normal ALU
wire ctrl2_volatiledest;
wire ctrl2_vf_a_sel; //0-0/1 from instr, 1-src1
wire ctrl2_mulshift_en;
wire ctrl2_matmul_en;
wire ctrl2_bfadder_en;
wire ctrl2_bfmult_en;
wire ctrl2_act_en;
wire ctrl2_trp_en;
wire ctrl2_transpose_en;
wire ctrl2_permute_en;
wire ctrl2_permute_op;
wire ctrl2_permute_store;
wire ctrl2_permute_read;
wire ctrl2_axi_en;
wire ctrl2_axi_req_type;
wire ctrl2_alufalu_en;

reg [NUMBANKS-1:0] ctrl3_vr_a_en; // SRC1
reg [NUMBANKS-1:0] ctrl3_vr_b_en; // SRC2
reg [NUMBANKS-1:0] ctrl3_vr_c_we; // DEST
reg [NUMBANKS-1:0] ctrl3_vf_a_en;
reg [NUMBANKS-1:0] ctrl3_vf_b_en;
reg [NUMBANKS-1:0] ctrl3_vf_c_we;
reg [NUMBANKS-1:0] ctrl3_vs_we;
reg [NUMBANKS-1:0] ctrl3_useslanes;
reg [NUMBANKS-1:0] ctrl3_mem_dir_left;
reg [NUMBANKS-1:0] ctrl3_rshiftnonzero;
reg [(((NUMBANKS-1)-(0)+1)*((10)-(0)+1))-1 : 0] ctrl3_alu_op;
reg [(((NUMBANKS-1)-(0)+1)*((1)-(0)+1))-1 : 0] ctrl3_satsum_op;
reg [(((NUMBANKS-1)-(0)+1)*((3)-(0)+1))-1 : 0] ctrl3_satsize_op;
reg [(((NUMBANKS-1)-(0)+1)*((4)-(0)+1))-1 : 0] ctrl3_mulshift_op;
reg [NUMBANKS-1:0] ctrl3_memunit_en;
reg [NUMBANKS-1:0] ctrl3_mem_en;
reg [(((NUMBANKS-1)-(0)+1)*((6)-(0)+1))-1 : 0] ctrl3_memunit_op;
reg [NUMBANKS-1:0] ctrl3_ismasked;
reg [(((NUMBANKS-1)-(0)+1)*((2)-(0)+1))-1 : 0] ctrl3_flagalu_op;
reg [NUMBANKS-1:0] ctrl3_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg [NUMBANKS-1:0] ctrl3_volatiledest;
reg [NUMBANKS-1:0] ctrl3_vf_a_sel; //0-0/1 from instr, 1-src1
reg [NUMBANKS-1:0] ctrl3_mulshift_en;
reg [NUMBANKS-1:0] ctrl3_matmul_en;
reg [NUMBANKS-1:0] ctrl3_trp_en;
reg [NUMBANKS-1:0] ctrl3_transpose_en;
reg [NUMBANKS-1:0] ctrl3_permute_en;
reg [NUMBANKS-1:0] ctrl3_permute_op;
reg [NUMBANKS-1:0] ctrl3_permute_read;
reg [NUMBANKS-1:0] ctrl3_permute_store;
reg [NUMBANKS-1:0] ctrl3_axi_en;
reg [NUMBANKS-1:0] ctrl3_axi_req_type;
reg [NUMBANKS-1:0] ctrl3_bfadder_en;
reg [NUMBANKS-1:0] ctrl3_bfmult_en;
reg [NUMBANKS-1:0] ctrl3_act_en;
reg [NUMBANKS-1:0] ctrl3_alufalu_en;

reg ctrl4_mem_dir_left;
reg ctrl4_rshiftnonzero;
reg [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((10)-(0)+1))-1 : 0] ctrl4_alu_op;
reg [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((1)-(0)+1))-1 : 0] ctrl4_satsum_op;
reg [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((3)-(0)+1))-1 : 0] ctrl4_satsize_op;
reg [4:0] ctrl4_mulshift_op;
reg ctrl4_memunit_en;
reg ctrl4_mem_en;
reg [6:0] ctrl4_memunit_op;
reg [NUMFUS-1:0] ctrl4_ismasked;
reg [((((NUMBANKS-1)*ALUPERBANK)-(0)+1)*((2)-(0)+1))-1 : 0] ctrl4_flagalu_op;
//reg ctrl4_vf_wbsel[(NUMBANKS-1)*ALUPERBANK:0];   //0-flag ALU, 1-normal ALU
reg [(NUMBANKS-1)*ALUPERBANK:0] ctrl4_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg ctrl4_volatiledest;
//reg ctrl4_vf_a_sel[(NUMBANKS-1)*ALUPERBANK:0]; //0-0/1 from instr, 1-src1
reg [(NUMBANKS-1)*ALUPERBANK:0] ctrl4_vf_a_sel; //0-0/1 from instr, 1-src1
reg ctrl4_mulshift_en;
reg ctrl4_matmul_en;
reg ctrl4_trp_en;
reg ctrl4_transpose_en;
reg ctrl4_permute_en;
reg ctrl4_permute_op;
reg ctrl4_permute_store;
reg ctrl4_permute_read;
reg ctrl4_bfadder_en;
reg ctrl4_bfmult_en;
reg ctrl4_axi_en;
reg ctrl4_axi_req_type;
reg ctrl4_act_en;

wire ctrl5_mem_en;

wire [`VRELM_RANGE] regid_pad;

integer bd;
genvar  ba;
integer bi;
integer i;
integer j;
genvar  bk;
genvar  k;
integer m;
integer n;
integer b;
integer b3;
integer f3;
integer bn;
integer fn;
integer fn2;
integer bw;

wire [NUMBANKS*LOG2MVL-1:0] rdelm;
wire [NUMBANKS*LOG2MVL-1:0] wrelm;
// MSb of count entries indicates if instruction is dead or not.
wire [NUMBANKS*(LOG2MVL-LOG2NUMLANES+1)-1:0] count;
reg  [NUMBANKS-1:0]   last_subvector;
wire [NUMBANKS-1:0]   first_subvector;
reg  [NUMBANKS-1:0] wrongbank_s3;
reg  [NUMBANKS-1:0] alive_s3;
reg [(((NUMFUS-1)-(0)+1)*((NUMBANKS-1)-(0)+1))-1 : 0] banksel_s4;

wire dispatcher_shift;
wire dispatcher_rotate;

wire [`MAX_STAGES-1:0] internal_pipe_advance;
wire [`MAX_STAGES-1:0] pipe_advance;
wire [`MAX_STAGES-1:0] pipe_squash;
wire stall_srcstart;
wire stall_dispatcher;
wire stall_hazsrc1;
wire stall_hazsrc2;
wire stall_hazfsrc1;
wire stall_hazfsrc2;
wire stall_memunit;
wire _stall_memunit;
wire stall_mulcooldown;
wire stall_mulunit;
wire stall_matmul;

wire temp_stall_matmul;
// DEBUG signals for Modelsim
wire [(((2)-(1)+1)*((7)-(0)+1))-1 : 0] D_instr;
reg [(((NUMBANKS-1)-(0)+1)*((7)-(0)+1))-1 : 0] D_instr_s3;
reg [(((NUMFUS-1)-(0)+1)*((7)-(0)+1))-1 : 0] D_instr_s4;
wire [(((NUMFUS-1)-(0)+1)*((7)-(0)+1))-1 : 0] D_instr_s5;
wire [(((NUMFUS-1)-(0)+1)*((7)-(0)+1))-1 : 0] D_instr_s6;
reg   [NUMFUS-1:0] D_last_subvector_s4;
wire  [NUMFUS-1:0] D_last_subvector_s5;
wire  [NUMFUS-1:0] D_last_subvector_s6;
wire  [NUMFUS-1:0] D_last_subvector_s31;
wire  [NUMFUS-1:0] D_wb_last_subvector;
reg   [NUMBANKS-1:0] D_last_subvector_done;
reg   [NUMBANKS-1:0] D_wb_instrdone;

assign trp_mode_s1 = 2'b00;
  pipe #(8,1) debuginstrpipe (
      .d( {instr[25:24],instr[5:0]} ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[2:1] ),
      .squash( pipe_squash[2:1] ),
      .q( {D_instr[((2-1)*((7)-(0)+1)+7-0) : ((2-1)*((7)-(0)+1))],D_instr[((1-1)*((7)-(0)+1)+7-0) : ((1-1)*((7)-(0)+1))]}));

  genvar Df;
  generate
  for (Df=0; Df<NUMFUS; Df=Df+1)
  begin : Debug_gen

    pipereg #(8) debugintrfupipereg1 (
      .d( D_instr_s4[((Df-0)*((7)-(0)+1)+7-0) : ((Df-0)*((7)-(0)+1))] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s5[((Df-0)*((7)-(0)+1)+7-0) : ((Df-0)*((7)-(0)+1))]));

    pipereg #(8) debugintrfupipereg2 (
      .d( D_instr_s5[((Df-0)*((7)-(0)+1)+7-0) : ((Df-0)*((7)-(0)+1))] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_instr_s6[((Df-0)*((7)-(0)+1)+7-0) : ((Df-0)*((7)-(0)+1))]));

    pipereg #(1) debuglastpipereg1 (
      .d( D_last_subvector_s4[Df] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s5[Df]));

    pipereg #(1) debuglastpipereg2 (
      .d( D_last_subvector_s5[Df] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn( ~pipe_squash[4] ),
      .q(D_last_subvector_s6[Df]));

  end
  endgenerate


//tell vpu to hold vc,vbase,etc values
assign is_stalled=~internal_pipe_advance | {stall_srcstart,2'b0};
//debug signals
assign has_memop=ctrl1_mem_en|ctrl2_mem_en|(|ctrl3_mem_en)|ctrl4_mem_en|ctrl5_mem_en;

/******************************************************************************/
/************************** Inter-Pipe Signals ********************************/
/******************************************************************************/

/* Memunit is in stage 5 but we stall in stage 4 so we don't squash the
* destination register contents*/
//do we have to advance the pipeline stage (stall condition checking)
assign pipe_advance=internal_pipe_advance & ~{5'b0,stall_in};
//advance pipeline stage 0 only if next pipeline stage is advancing
assign internal_pipe_advance[0]=internal_pipe_advance[1];
assign internal_pipe_advance[1]=internal_pipe_advance[2];
assign internal_pipe_advance[2]=internal_pipe_advance[3] &&  //checking for hazard conditions as well
                          ~stall_srcstart &&                     
                          ~stall_dispatcher && //if vl-length is set more than 8, dispatcher will stall since it has only 8 lanes
                          ~stall_hazsrc1 && ~stall_hazsrc2 && 
                          ~stall_hazfsrc1 && ~stall_hazfsrc2 && 
                          ~stall_mulcooldown; 
assign internal_pipe_advance[3]=internal_pipe_advance[4];
assign internal_pipe_advance[4]=internal_pipe_advance[5];
assign internal_pipe_advance[5]=internal_pipe_advance[6] && ~stall_mulunit && ~stall_memunit && ~temp_stall_matmul; 
                                                                                                //matmul stall may or may not be reqd
//assign internal_pipe_advance[6]=1'b1;

//putting nop signals - previous pipeline stage stall
assign pipe_squash[0]=pipe_advance[1]&~pipe_advance[0];
assign pipe_squash[1]=pipe_advance[2]&~pipe_advance[1];
assign pipe_squash[2]=pipe_advance[3]&~pipe_advance[2];
assign pipe_squash[3]=pipe_advance[4]&~pipe_advance[3];
assign pipe_squash[4]=pipe_advance[5]&~pipe_advance[4];
assign pipe_squash[5]=pipe_advance[6]&~pipe_advance[5];
assign pipe_squash[6]=1'b0;
assign pipe_squash[13:7]= 'h0;

//This is for the pipeline stages we added and we don't expect stalls here , hence set to 1
assign internal_pipe_advance[13:6] = {(14-5){1'b1}};
//assign pipe_squash[`MAX_STAGES:7]= 'h0;
//assign internal_pipe_advance[`MAX_STAGES:6] = {(`MAX_STAGES-5){1'b1}};
//assign pipe_squash[6]=1'b0;

/******************************************************************************/
/************************** 1st Pipeline Stage ********************************/
/******************************************************************************/

  assign ir_op={instr[25:22],instr[5:0]}; //10 bits
  assign ir_dst=instr[10:6];
  assign ir_src1=instr[15:11];
  assign ir_src2=instr[20:16];
  assign ir_mask=instr[21];

  // Determine which instruction read from which ports
  always@*
  begin //This regfile has 2 read ports and 1 write port;hence 3 enables 
    ctrl1_vr_a_en=0;
    ctrl1_vr_b_en=0;
    ctrl1_vr_c_en=0;
    ctrl1_vf_a_en=0; //don't really use vf - vector flags or vf_sel signals
    ctrl1_vf_b_en=0;
    ctrl1_vf_a_sel=0;
    ctrl1_usesvssel=0;
    case(ir_op) //decode the registers based on the IR op (current instr)
    COP2_VADD:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VADD_U:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VSUB:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VSUB_U:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VMULHI:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VMULHI_U:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    //  COP2_VMOD,
    COP2_VMOD_U:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VCMP_EQ:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VCMP_NE:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VCMP_LT:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VCMP_U_LT:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VCMP_LE:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VCMP_U_LE:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VDIV:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VDIV_U:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VBFADD:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VMIN:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VMIN_U:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VMAX:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VMAX_U:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VMULLO:
        begin
          bf_op = 0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VABS:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=1;
        end
    COP2_VAND:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VOR:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXOR:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VNOR:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VSLL:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VSRL:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
    COP2_VSRA:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
     //COP2_VSAT_W,
      //COP2_VSAT_SU_W,
      //COP2_VSAT_U_W:
    COP2_VSAT_B:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSAT_H:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VACT:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSAT_SU_B:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSAT_SU_H:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VRED:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSAT_SU_L:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSAT_U_B:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSAT_U_H:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VTRP:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSADD:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VSADD_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VSSUB:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VSSUB_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VSRR:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSRR_U:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSLS:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSLS_U:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VXUMUL:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXUMUL_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXLMUL:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXLMUL_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXUMADD:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXUMADD_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXUMSUB:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXUMSUB_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXLMADD:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXLMADD_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXLMSUB:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VXLMSUB_U:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
    COP2_VINS_VV:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vr_b_en=1;
        end
      //COP2_VINS_SV: doesn't read any vectors or flags
    COP2_VEXT_VV:
        begin
          ctrl1_vr_a_en=1;
        end
    COP2_VEXT_SV:
        begin
          ctrl1_vr_a_en=1;
        end
    COP2_VEXT_U_SV:
        begin
          ctrl1_vr_a_en=1;
        end
    COP2_VCOMPRESS:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VEXPAND:
        begin
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VMERGE:
        begin
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
      //COP2_VFINS:
    COP2_VEXTHALF:
        begin
          ctrl1_vr_a_en=1;
        end
    COP2_VHALF:
        begin
          ctrl1_vr_a_en=1;
        end
    COP2_VHALFUP:
        begin
          ctrl1_vr_a_en=1;
        end
    COP2_VHALFDN:
        begin
          ctrl1_vr_a_en=1;
        end
      //COP2_VSATVL:
    COP2_VFAND:
        begin
          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vf_b_en=1;
          ctrl1_vf_a_sel=1;
          ctrl1_usesvssel=1;
        end
    COP2_VFOR:
        begin
          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vf_b_en=1;
          ctrl1_vf_a_sel=1;
          ctrl1_usesvssel=1;
        end
    COP2_VFXOR:
        begin
          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vf_b_en=1;
          ctrl1_vf_a_sel=1;
          ctrl1_usesvssel=1;
        end
    COP2_VFNOR:
        begin
          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vf_b_en=1;
          ctrl1_vf_a_sel=1;
          ctrl1_usesvssel=1;
        end
      //COP2_VFCLR:
      //COP2_VFSET:
    COP2_VIOTA:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
    COP2_VCIOTA:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
    COP2_VFPOP:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
    COP2_VFFF1:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
    COP2_VFFL1:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
    COP2_VFSETBF:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
    COP2_VFSETIF:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
    COP2_VFSETOF:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
      //COP2_VFMT8:
      //COP2_VFMF8:
      //COP2_VFCLR8:
      //COP2_VFOR8:
      //COP2_VFLD:
      //COP2_VBFADD,
      //COP2_VLD_U_W,
    COP2_VLD_B:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLD_H:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLD_L:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLD_U_B:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLD_U_H:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLDS_B:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLDS_H:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VBFSUB:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLDS_L:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLDS_U_B:
        begin
          ctrl1_vf_a_en=1;
        end
    COP2_VLDS_U_H:
        begin
          ctrl1_vf_a_en=1;
        end
    //COP2_VBFMULT:
    //    begin
    //      ctrl1_vf_a_en=1;
    //    end
      //COP2_VTRP,
      //COP2_VRED,
     // COP2_VACT:
    COP2_VLDX_B:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
        end
    COP2_VLDX_H:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
        end
    COP2_VPER_STR:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
        end
    COP2_VLDX_L:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
        end
    COP2_VLDX_U_B:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
        end
    COP2_VLDX_U_H:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
        end
      //COP2_VFST:
    COP2_VST_B:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_b_en=1;
        end
    COP2_VST_H:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_b_en=1;
        end
    COP2_VST_L:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_b_en=1;
        end
    COP2_VSTS_B:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_b_en=1;
        end
    COP2_VSTS_H:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_b_en=1;
        end
    COP2_VSTS_L:
        begin
          ctrl1_vf_a_en=1;
          ctrl1_vr_b_en=1;
        end
    COP2_VPER:
         begin
          ctrl1_vr_b_en=1;
         end
    COP2_VPER_LD:
         begin
          ctrl1_vr_b_en=1;
         end
    COP2_VAXIWR:
         begin
           ctrl1_vr_b_en=1;
         end
    COP2_VAXIRD:
         begin
           ctrl1_vr_c_en=1;
         end
      //COP2_VSTX_W,
      //COP2_VSTXO_W,  AXI Write
    COP2_VSTX_B:
        begin
          ctrl1_vr_b_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSTX_H:
        begin
          ctrl1_vr_b_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSTX_L:
        begin
          ctrl1_vr_b_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSTXO_B:
        begin
          ctrl1_vr_b_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSTXO_H:
        begin
          ctrl1_vr_b_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_vf_a_en=1;
        end
    COP2_VSTXO_L:
        begin
          ctrl1_vr_b_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_vf_a_en=1;
        end
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  // Decode enables ; some of the enable signals for the functional units
  always@*
  begin
    ctrl1_vr_d_we=0;
    ctrl1_vf_c_we=0;
    ctrl1_vs_we=0;
    ctrl1_vrdest_sel=0;
    ctrl1_elmshamt_sel=0;
    ctrl1_srcshamt_sel=0;
    ctrl1_srclimit_sel=0;
    ctrl1_dstshamt_sel=0;
    ctrl1_mem_dir_left=0;
    ctrl1_rshiftnonzero=0;
    ctrl1_memunit_en=0;
    ctrl1_mem_en=0;
    ctrl1_ismasked=0;
    ctrl1_setvlto1=0;
    ctrl1_vf_wbsel=0;
    ctrl1_volatiledest=0;
    ctrl1_permute_store = 1'b0;  // Loading data into the permute block
    ctrl1_permute_op = 1'b0; // processing the permute op
    case(ir_op)
    //  COP2_VMOD,
    COP2_VADD:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VADD_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSUB:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSUB_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VMULHI:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VMULHI_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VDIV:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VDIV_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VBFADD:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VBFMULT:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VMOD_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
     // COP2_VACT,
    COP2_VTRP:
         begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
         end
    COP2_VPER_STR:
         begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
         end
    COP2_VRED:
         begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
         end
    COP2_VCMP_EQ:
        begin
          ctrl1_vf_c_we=1;
          ctrl1_vf_wbsel=1;
          ctrl1_ismasked=1;
        end
    COP2_VCMP_NE:
        begin
          ctrl1_vf_c_we=1;
          ctrl1_vf_wbsel=1;
          ctrl1_ismasked=1;
        end
    COP2_VCMP_LT:
        begin
          ctrl1_vf_c_we=1;
          ctrl1_vf_wbsel=1;
          ctrl1_ismasked=1;
        end
    COP2_VCMP_U_LT:
        begin
          ctrl1_vf_c_we=1;
          ctrl1_vf_wbsel=1;
          ctrl1_ismasked=1;
        end
    COP2_VCMP_LE:
        begin
          ctrl1_vf_c_we=1;
          ctrl1_vf_wbsel=1;
          ctrl1_ismasked=1;
        end
    COP2_VCMP_U_LE:
        begin
          ctrl1_vf_c_we=1;
          ctrl1_vf_wbsel=1;
          ctrl1_ismasked=1;
        end
      //COP2_VSAT_W,
      //COP2_VSAT_SU_W,
      //COP2_VSAT_U_W,
    COP2_VMIN:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VMIN_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VMAX:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VMAX_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VMULLO:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VABS:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VAND:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VOR:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VXOR:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VNOR:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSLL:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSRL:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSRA:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSAT_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSAT_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VACT:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSAT_SU_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSAT_SU_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VRED:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSAT_SU_L:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSAT_U_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSAT_U_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSADD:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSADD_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSSUB:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSSUB_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSRR:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSRR_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSLS:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VSLS_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
    COP2_VXUMUL:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXUMUL_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXLMUL:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXLMUL_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXUMADD:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXUMADD_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXUMSUB:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXUMSUB_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXLMADD:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXLMADD_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXLMSUB:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VXLMSUB_U:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
    COP2_VINS_VV:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=0;
          ctrl1_dstshamt_sel=3;
          ctrl1_mem_dir_left=1;
          ctrl1_volatiledest=1;
        end
    COP2_VINS_SV:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_elmshamt_sel=0;
          ctrl1_srcshamt_sel=0;
          ctrl1_dstshamt_sel=3;
          ctrl1_mem_dir_left=1;
          ctrl1_setvlto1=1;
        end
    COP2_VEXT_VV:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=3;
          ctrl1_srclimit_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_volatiledest=1;
        end
    COP2_VEXT_SV:
        begin
          ctrl1_vs_we=1;
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=3;
          ctrl1_srclimit_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_setvlto1=1;
        end
    COP2_VEXT_U_SV:
        begin
          ctrl1_vs_we=1;
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=3;
          ctrl1_srclimit_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_setvlto1=1;
        end
    COP2_VCOMPRESS:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_mem_dir_left=0;
          ctrl1_ismasked=1;
          ctrl1_volatiledest=1;
        end
    COP2_VEXPAND:
        begin
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_mem_dir_left=1;
          ctrl1_ismasked=1;
          ctrl1_volatiledest=1;
        end
    COP2_VMERGE:
        begin
          ctrl1_vr_d_we=1;
        end
    COP2_VFINS:
        begin
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=0;
          ctrl1_mem_dir_left=1;
        end
    COP2_VEXTHALF:
        begin
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=1;
          ctrl1_srcshamt_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_vr_d_we=1;
          ctrl1_volatiledest=1;
        end
    COP2_VHALF:
        begin
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=1;
          ctrl1_srcshamt_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_vr_d_we=1;
          ctrl1_volatiledest=1;
        end
    COP2_VHALFUP:
        begin
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=2;
          ctrl1_srcshamt_sel=2;
          ctrl1_mem_dir_left=0;
          ctrl1_vr_d_we=1;
          ctrl1_volatiledest=1;
        end
    COP2_VHALFDN:
        begin
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=2;
          ctrl1_srcshamt_sel=0;
          ctrl1_dstshamt_sel=2;
          ctrl1_mem_dir_left=1;
          ctrl1_vr_d_we=1;
          ctrl1_volatiledest=1;
        end
      //COP2_VSATVL:
    COP2_VFAND:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFOR:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFXOR:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFNOR:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFCLR:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFSET:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VIOTA:
        begin
          ctrl1_vr_d_we=1;
        end
    COP2_VCIOTA:
        begin
          ctrl1_vr_d_we=1;
        end
    COP2_VFPOP:
        begin
          ctrl1_vs_we=1;
        end
    COP2_VFFF1:
        begin
          ctrl1_vs_we=1;
        end
    COP2_VFFL1:
        begin
          ctrl1_vs_we=1;
        end
    COP2_VFSETBF:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFSETIF:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFSETOF:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFMT8:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFMF8:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFCLR8:
        begin
          ctrl1_vf_c_we=1;
        end
    COP2_VFOR8:
        begin
          ctrl1_vf_c_we=1;
        end
      //COP2_VFLD,
      //COP2_VLD_U_W,
    COP2_VLD_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLD_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VBFADD:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLD_L:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLD_U_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLD_U_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDS_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDS_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VBFSUB:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDS_L:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDS_U_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDS_U_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    //COP2_VBFMULT:
    //   begin
    //     ctrl1_vr_d_we=1;
    //     ctrl1_vrdest_sel=1;
    //     ctrl1_memunit_en=1;
    //     ctrl1_mem_en=1;
    //     ctrl1_ismasked=1;
    //   end
     // COP2_VTRP,
     // COP2_VACT:
    COP2_VLDX_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDX_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDX_L:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDX_U_B:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VLDX_U_H:
        begin
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
      //COP2_VFST:
      //COP2_VRED,
      //COP2_VPER,
      //COP2_VSTX_W,
      //COP2_VSTXO_W,  AXI write
    COP2_VST_B:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VST_H:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VST_L:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTS_B:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTS_H:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTS_L:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTX_B:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTX_H:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTX_L:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTXO_B:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTXO_H:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VSTXO_L:
        begin
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
    COP2_VPER_STR:
         begin
           ctrl1_ismasked = 1'b1;
         end
    COP2_VPER:
         begin
           ctrl1_permute_op = 1'b1;
           ctrl1_ismasked = 1'b1;
         end
    COP2_VPER_LD:
         begin
           ctrl1_permute_store = 1'b1;
           ctrl1_ismasked = 1'b1;
         end
    COP2_VAXIWR:
         begin
           ctrl1_ismasked = 1'b1;
         end
    COP2_VAXIRD:
         begin
           ctrl1_ismasked = 1'b1;
         end
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  // Decode instructions FU op codes
  initial
  begin
    ctrl1_alu_op=ALUOP_ZERO^ALUOP_ZERO;  //Aid subsetting by setting to zero
    ctrl1_satsum_op=SATSUMOP_NOP;
    ctrl1_satsize_op=SATSIZEOP_VSATUW;
    ctrl1_mulshift_op=MULOP_ZERO;
    ctrl1_matmul_en=0;
    ctrl1_trp_en = 1'b0;
    ctrl1_permute_en = 1'b0;
    ctrl1_transpose_en = 1'b0;
    ctrl1_bfadder_en = 1'b0;
    ctrl1_bfmult_en = 1'b0;
    ctrl1_act_en = 1'b0;
    ctrl1_axi_en = 1'b0;
    ctrl1_axi_req_type = 1'b0;
    ctrl1_memunit_op=0;
    ctrl1_flagalu_op=FLAGOP_CLR;
    ctrl1_permute_read = 1'b0;
  end
  always@* //enable signals for our functional units
  begin    
    ctrl1_alu_op=ALUOP_ZERO^ALUOP_ZERO;
    ctrl1_satsum_op=SATSUMOP_NOP;
    ctrl1_satsize_op=SATSIZEOP_VSATUW;
    ctrl1_mulshift_op=MULOP_ZERO;
    ctrl1_memunit_op=0;
    ctrl1_matmul_en=0;
    ctrl1_bfadder_en = 1'b0;
    ctrl1_bfmult_en = 1'b0;
    ctrl1_act_en = 1'b0;
    ctrl1_trp_en = 1'b0;
    ctrl1_transpose_en = 1'b0;
    ctrl1_permute_en = 1'b0;
    ctrl1_permute_read = 1'b0;
    ctrl1_axi_en = 1'b0;
    ctrl1_axi_req_type = 1'b0;
    ctrl1_flagalu_op=FLAGOP_CLR;
    case(ir_op)
      COP2_VADD:      ctrl1_alu_op=ALUOP_ADD^ALUOP_ZERO;
      COP2_VADD_U:    ctrl1_alu_op=ALUOP_ADDU^ALUOP_ZERO;
      COP2_VSUB:      ctrl1_alu_op=ALUOP_SUB^ALUOP_ZERO;
      COP2_VSUB_U:    ctrl1_alu_op=ALUOP_SUBU^ALUOP_ZERO;
      COP2_VMULHI:  ctrl1_mulshift_op=MULOP_MULHI;
      COP2_VMULHI_U:ctrl1_mulshift_op=MULOP_MULUHI;
      COP2_VDIV:    ctrl1_matmul_en=1'b1;  //Note: This is a hack. We're using VDIV opcode for matmul operation
      COP2_VBFADD:    ctrl1_bfadder_en = 1'b1;
      //COP2_VBFMULT:   ctrl1_bfmult_en = 1'b1;
      COP2_VACT:      ctrl1_act_en = 1'b1;
      COP2_VTRP:      ctrl1_transpose_en = 1'b1;
      COP2_VRED:      ctrl1_trp_en = 1'b1;
      //COP2_VPER:      ctrl1_permute_en = 1'b1;
      //COP2_VDIV_U,
      //COP2_VMOD,
      //COP2_VMOD_U,
      COP2_VCMP_EQ:   ctrl1_alu_op=ALUOP_CMP_EQ^ALUOP_ZERO;
      COP2_VCMP_NE:   ctrl1_alu_op=ALUOP_CMP_NEQ^ALUOP_ZERO;
      COP2_VCMP_LT:   ctrl1_alu_op=ALUOP_CMP_LT^ALUOP_ZERO;
      COP2_VCMP_U_LT: ctrl1_alu_op=ALUOP_CMP_LT_U^ALUOP_ZERO;
      COP2_VCMP_LE:   ctrl1_alu_op=ALUOP_CMP_LE^ALUOP_ZERO;
      COP2_VCMP_U_LE: ctrl1_alu_op=ALUOP_CMP_LE_U^ALUOP_ZERO;
      COP2_VMIN:      ctrl1_alu_op=ALUOP_MIN^ALUOP_ZERO;
      COP2_VMIN_U:    ctrl1_alu_op=ALUOP_MIN_U^ALUOP_ZERO;
      COP2_VMAX:      ctrl1_alu_op=ALUOP_MAX^ALUOP_ZERO;
      COP2_VMAX_U:    ctrl1_alu_op=ALUOP_MAX_U^ALUOP_ZERO;
      COP2_VMULLO:    ctrl1_mulshift_op=MULOP_MULLO;
      COP2_VABS:      ctrl1_alu_op=ALUOP_ABS^ALUOP_ZERO;
      COP2_VAND:      ctrl1_alu_op=ALUOP_AND^ALUOP_ZERO;
      COP2_VOR:       ctrl1_alu_op=ALUOP_OR^ALUOP_ZERO;
      COP2_VXOR:      ctrl1_alu_op=ALUOP_XOR^ALUOP_ZERO;
      COP2_VNOR:      ctrl1_alu_op=ALUOP_NOR^ALUOP_ZERO;
      COP2_VSLL: ctrl1_mulshift_op=MULOP_SLL;
      COP2_VSRL: ctrl1_mulshift_op=MULOP_SRL;
      COP2_VSRA: ctrl1_mulshift_op=MULOP_SRA;
      COP2_VSAT_B: ctrl1_satsize_op=SATSIZEOP_VSATB;
      COP2_VSAT_H: ctrl1_satsize_op=SATSIZEOP_VSATH;
     // COP2_VSAT_W: ctrl1_satsize_op=SATSIZEOP_VSATW;
      COP2_VSAT_SU_B: ctrl1_satsize_op=SATSIZEOP_VSATSUB;
      COP2_VSAT_SU_H: ctrl1_satsize_op=SATSIZEOP_VSATSUH;
      //COP2_VSAT_SU_W: ctrl1_satsize_op=SATSIZEOP_VSATSUW;
      //COP2_VSAT_SU_L:
      COP2_VSAT_U_B: ctrl1_satsize_op=SATSIZEOP_VSATUB;
      COP2_VSAT_U_H: ctrl1_satsize_op=SATSIZEOP_VSATUH;
      //COP2_VSAT_U_W: ctrl1_satsize_op=SATSIZEOP_VSATUW;
      COP2_VSADD: begin ctrl1_alu_op=ALUOP_ADD^ALUOP_ZERO; ctrl1_satsum_op=SATSUMOP_VS; end
      COP2_VSADD_U: begin 
          ctrl1_alu_op=ALUOP_ADDU^ALUOP_ZERO; 
          ctrl1_satsum_op=SATSUMOP_VSU; 
        end
      COP2_VSSUB: begin ctrl1_alu_op=ALUOP_SUB^ALUOP_ZERO; ctrl1_satsum_op=SATSUMOP_VS; end
      COP2_VSSUB_U: begin 
          ctrl1_alu_op=ALUOP_SUBU^ALUOP_ZERO;
          ctrl1_satsum_op=SATSUMOP_VSU; 
        end
      COP2_VSRR: ctrl1_mulshift_op=MULOP_SRA;
      COP2_VSRR_U: ctrl1_mulshift_op=MULOP_SRL;
      COP2_VSLS: ctrl1_mulshift_op=MULOP_SLS;
      COP2_VSLS_U: ctrl1_mulshift_op=MULOP_SLSU;
      COP2_VXUMUL: ctrl1_mulshift_op=MULOP_UMUL;
      COP2_VXUMUL_U: ctrl1_mulshift_op=MULOP_UMULU;
      COP2_VXLMUL: ctrl1_mulshift_op=MULOP_LMUL;
      COP2_VXLMUL_U: ctrl1_mulshift_op=MULOP_LMULU;
      COP2_VXUMADD: ctrl1_mulshift_op=MULOP_UMUL;
      COP2_VXUMADD_U: ctrl1_mulshift_op=MULOP_UMULU;
      COP2_VXUMSUB: ctrl1_mulshift_op=MULOP_UMUL;
      COP2_VXUMSUB_U: ctrl1_mulshift_op=MULOP_UMULU;
      COP2_VXLMADD: ctrl1_mulshift_op=MULOP_LMUL;
      COP2_VXLMADD_U: ctrl1_mulshift_op=MULOP_LMULU;
      COP2_VXLMSUB: ctrl1_mulshift_op=MULOP_LMUL;
      COP2_VXLMSUB_U: ctrl1_mulshift_op=MULOP_LMULU;
      COP2_VINS_VV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VINS_SV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VEXT_VV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VEXT_SV: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VEXT_U_SV: ctrl1_memunit_op=MEMOP_SHIFT;
      //COP2_VCOMPRESS:
      //COP2_VEXPAND:
      COP2_VMERGE:      ctrl1_alu_op=ALUOP_MERGE^ALUOP_ZERO;
      //COP2_VFINS:
      COP2_VEXTHALF: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VHALF: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VHALFUP: ctrl1_memunit_op=MEMOP_SHIFT;
      COP2_VHALFDN: ctrl1_memunit_op=MEMOP_SHIFT;
      //COP2_VSATVL:
      COP2_VFAND: ctrl1_flagalu_op=FLAGOP_AND;
      COP2_VFOR: ctrl1_flagalu_op=FLAGOP_OR;
      COP2_VFXOR: ctrl1_flagalu_op=FLAGOP_XOR;
      COP2_VFNOR: ctrl1_flagalu_op=FLAGOP_NOR;
      COP2_VFCLR: ctrl1_flagalu_op=FLAGOP_CLR;
      COP2_VFSET: ctrl1_flagalu_op=FLAGOP_SET;
      //COP2_VIOTA,
      //COP2_VCIOTA:
      //COP2_VFPOP,
      //COP2_VFFF1,
      //COP2_VFFL1:
      //COP2_VFSETBF,
      //COP2_VFSETIF,
      //COP2_VFSETOF,
      //COP2_VFMT8,
      //COP2_VFMF8,
      //COP2_VFCLR8,
      //COP2_VFOR8:
      //COP2_VFLD:
      COP2_VLD_B: ctrl1_memunit_op=MEMOP_LDB;
      COP2_VLD_H: ctrl1_memunit_op=MEMOP_LDH;
      //COP2_VBFADD: ctrl1_memunit_op=MEMOP_LDW;
      //COP2_VLD_L,
      COP2_VLD_U_B: ctrl1_memunit_op=MEMOP_LDUB;
      COP2_VLD_U_H: ctrl1_memunit_op=MEMOP_LDUH;
      //COP2_VLD_U_W: ctrl1_memunit_op=MEMOP_LDUW;
      COP2_VLDS_B: ctrl1_memunit_op=MEMOP_LDSB;
      COP2_VLDS_H: ctrl1_memunit_op=MEMOP_LDSH;
      COP2_VBFSUB: ctrl1_memunit_op=MEMOP_LDSW;
      //COP2_VLDS_L:
      COP2_VLDS_U_B: ctrl1_memunit_op=MEMOP_LDSUB;
      COP2_VLDS_U_H: ctrl1_memunit_op=MEMOP_LDSUH;
      COP2_VBFMULT: ctrl1_memunit_op=MEMOP_LDSUW;
      COP2_VLDX_B: ctrl1_memunit_op=MEMOP_LDXB;
      COP2_VLDX_H: ctrl1_memunit_op=MEMOP_LDXH;
      //COP2_VTRP: ctrl1_memunit_op=MEMOP_LDXW;
      //COP2_VLDX_L:
      COP2_VLDX_U_B: ctrl1_memunit_op=MEMOP_LDXUB;
      COP2_VLDX_U_H: ctrl1_memunit_op=MEMOP_LDXUH;
      //COP2_VACT: ctrl1_memunit_op=MEMOP_LDXUW;
      //COP2_VFST:
      COP2_VST_B: ctrl1_memunit_op=MEMOP_STB;
      COP2_VST_H: ctrl1_memunit_op=MEMOP_STH;
      //COP2_VRED: ctrl1_memunit_op=MEMOP_STW;
      //COP2_VST_L:
      COP2_VSTS_B: ctrl1_memunit_op=MEMOP_STSB;
      COP2_VSTS_H: ctrl1_memunit_op=MEMOP_STSH;
      //COP2_VPER: ctrl1_memunit_op=MEMOP_STSW;
      COP2_VPER_STR: //permute store instruction
        begin
           ctrl1_permute_read = 1'b1;
           ctrl1_permute_en = 1'b1;
        end
      COP2_VPER: ctrl1_permute_en = 1'b1;
      COP2_VPER_LD: ctrl1_permute_en = 1'b1;
      //COP2_VSTS_L:
      COP2_VSTX_B: ctrl1_memunit_op=MEMOP_STXB;
      COP2_VSTX_H: ctrl1_memunit_op=MEMOP_STXH;
      //COP2_VSTX_W: ctrl1_memunit_op=MEMOP_STXW;
      //COP2_VSTX_L:
      COP2_VSTXO_B: ctrl1_memunit_op=MEMOP_STXB;
      COP2_VSTXO_H: ctrl1_memunit_op=MEMOP_STXH;
      //COP2_VSTXO_W: ctrl1_memunit_op=MEMOP_STXW; AXI Write
    COP2_VAXIWR:
             begin 
                ctrl1_axi_en = 1'b1; 
                ctrl1_axi_req_type=1'b1;
             end
      COP2_VAXIRD:begin ctrl1_axi_en = 1'b1; ctrl1_axi_req_type=1'b0; end
      //COP2_VSTXO_L:
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  assign regid_pad=0;

  pipereg 
    #( //one pipeline stage with total number of registers
      2+
      2+
      1+
      2+
      1
    ) pipe1reg_secondstageonly (
      .d( {
        ctrl1_elmshamt_sel,
        ctrl1_srcshamt_sel,
        ctrl1_srclimit_sel,
        ctrl1_dstshamt_sel,
        ctrl1_setvlto1
      }),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[1] ), //only enable register if pipe advance
      .squashn( 1'b1 ),
      .q({
        ctrl2_elmshamt_sel,
        ctrl2_srcshamt_sel,
        ctrl2_srclimit_sel, //used to get squashed, pretend doesn't need to
        ctrl2_dstshamt_sel,
        ctrl2_setvlto1
      }));

  // *********** Pipeline signals that need to be squashed *********
  pipereg 
    #(
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      1+
      3+
      1+
      1+
      2+ // For permute and transpose enable
      3+1+1  // For Permute store,read and op signals + 1 for axi en + 1 axi req tyoe
    ) pipe1regwsquash (
      .d( {
        ctrl1_vr_d_we,
        ctrl1_vf_c_we,
        ctrl1_vs_we,
        ctrl1_vr_a_en|ctrl1_vr_c_en,
        ctrl1_vr_b_en,
        ctrl1_vf_a_en,
        ctrl1_vf_b_en,
        ctrl1_vr_a_en|ctrl1_vr_b_en|ctrl1_vr_c_en|ctrl1_vr_d_we|ctrl1_vf_a_en|ctrl1_vf_b_en|ctrl1_vf_c_we,
        ctrl1_memunit_en,
        ctrl1_mem_en,
        |ctrl1_mulshift_op,
        ctrl1_matmul_en,
        ctrl1_bfadder_en,
        ctrl1_bfmult_en,
        ctrl1_act_en,
        ctrl1_trp_en,
        ctrl1_transpose_en,
        ctrl1_permute_en,
        ctrl1_permute_op,
        ctrl1_permute_store,
        ctrl1_permute_read,
        ctrl1_axi_en,
        ctrl1_axi_req_type,
        (ctrl1_alu_op!=(ALUOP_ZERO^ALUOP_ZERO)) || ctrl1_vf_c_we
      }),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[1] ),
      .squashn( ~pipe_squash[1] ),
      .q({
        ctrl2_vr_c_we,
        ctrl2_vf_c_we,
        ctrl2_vs_we,
        ctrl2_vr_a_en,
        ctrl2_vr_b_en,
        ctrl2_vf_a_en,
        ctrl2_vf_b_en,
        ctrl2_useslanes,
        ctrl2_memunit_en,
        ctrl2_mem_en,
        ctrl2_mulshift_en,
        ctrl2_matmul_en,
        ctrl2_bfadder_en,
        ctrl2_bfmult_en,
        ctrl2_act_en,
        ctrl2_trp_en,
        ctrl2_transpose_en,
        ctrl2_permute_en,
        ctrl2_permute_op,
        ctrl2_permute_store,
        ctrl2_permute_read,
        ctrl2_axi_en,
        ctrl2_axi_req_type,
        ctrl2_alufalu_en
        })
      );

  // *********** Pipeline signals that don't need to be squashed *********
  pipereg 
    #(
      REGIDWIDTH+
      REGIDWIDTH+
      REGIDWIDTH+
      1+
      1+
      1+
      1+
      1+
      1+
      11+
      2+
      4+
      5+
      7+
      1+
      3+
      1+
      1+
      1+
      2 // Sangram added 1 for TRP unit en signal
    ) pipe1reg (
      .d({
        {(ctrl1_vrdest_sel) ? ir_src2 : ir_dst, regid_pad},
        {(ctrl1_vr_c_en ) ? ir_dst : ir_src1, regid_pad},
        {ir_src2,regid_pad},
        ir_op[7] && ctrl1_usesvssel,
        ir_op[6] && ctrl1_usesvssel,
        ir_mask,
        ctrl1_vf_a_sel,
        ctrl1_rshiftnonzero,
        ctrl1_mem_dir_left,
        ctrl1_alu_op,
        ctrl1_satsum_op,
        ctrl1_satsize_op,
        ctrl1_mulshift_op,
        ctrl1_memunit_op,
        ctrl1_ismasked,
        ctrl1_flagalu_op,
        ctrl1_vf_wbsel,
        ctrl1_volatiledest,
        trp_mode_s1 //selects mode for trp unit (for reduction operation)
      }),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[1] ),
      .squashn( 1'b1 ),
      .q({
        dst_s2,
        src1_s2,
        src2_s2,
        src1scalar_s2,
        src2scalar_s2,
        imask_s2,
        ctrl2_vf_a_sel,
        ctrl2_rshiftnonzero,
        ctrl2_mem_dir_left,
        ctrl2_alu_op,
        ctrl2_satsum_op,
        ctrl2_satsize_op,
        ctrl2_mulshift_op,
        ctrl2_memunit_op,
        ctrl2_ismasked,
        ctrl2_flagalu_op,
        ctrl2_vf_wbsel,
        ctrl2_volatiledest,
        trp_mode_s2
      }));
  
  wire [6:1] squash_ctrlmemoppipe_NC;
  pipe #(7,5) ctrlmemoppipe (
      .d( ctrl1_memunit_op ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:1] & {4'b1,ctrl2_memunit_en,1'b1} ),
      .squash(squash_ctrlmemoppipe_NC),
      //.squash( pipe_squash[6:1] ),
      .q( {ctrl_memunit_op[6],ctrl_memunit_op[5],ctrl_memunit_op[4],ctrl_memunit_op[3],ctrl_memunit_op[2],ctrl_memunit_op[1]} ));

/******************************************************************************/
/******************************* 2nd Pipeline Stage ***************************/
/******************************************************************************/

  // if src_start!=0 stall pipeline to calculate it and then do haz check

  onecyclestall shamtstall(ctrl2_srcshamt_sel!=0,clk,resetn,stall_srcstart);

  always@(posedge clk)
    if (!resetn || pipe_advance[1] )
      src_start_delayed<=0;
    else 
      src_start_delayed<=src_start;

  assign src_start= ( ctrl2_srcshamt_sel==3 ) ? vc[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))] :           // vcindex
               ( ctrl2_srcshamt_sel==1 ) ? vl[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-1) : ((2-2)*((VCWIDTH-1)-(0)+1))] :   // vl>>2 ; vl is from the vector control code 
               ( ctrl2_srcshamt_sel==2 ) ? 1 << vc[((2-2)*((VCWIDTH-1)-(0)+1)+LOG2MVL-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]://2^vcindex
               0;

  assign src_elm=src_start_delayed & (-1<<LOG2NUMLANES);

  assign src_limit= ((ctrl2_setvlto1) ? 0 : vl[((2-2)*((VCWIDTH-1)-(0)+1)+LOG2MVL-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))] - 1'b1) +
                    ((ctrl2_srclimit_sel) ? vc[((2-2)*((VCWIDTH-1)-(0)+1)+LOG2MVL-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))] : 0);


  /******************* Adjust dest to account for shift  ****************/

  // Compute real destination register - accounting for shifting instructions

  assign dst_start= ( ctrl2_dstshamt_sel==3 ) ? vc[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))] :           // vcindex
               ( ctrl2_dstshamt_sel==2 ) ? 1 << vc[((2-2)*((VCWIDTH-1)-(0)+1)+LOG2MVL-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]://2^vcindex
               0;

  //assign dst_elm = {dst_start[LOG2MVL-1:0]>>LOG2NUMLANES,{LOG2NUMLANES{1'b0}}};

  assign total_shamt= ( ctrl2_elmshamt_sel==3 ) ? vc[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))] :          // vcindex
               ( ctrl2_elmshamt_sel==1 ) ? vl[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-1) : ((2-2)*((VCWIDTH-1)-(0)+1))] :    // vl>>2
               ( ctrl2_elmshamt_sel==2 ) ? 1 << vc[((2-2)*((VCWIDTH-1)-(0)+1)+LOG2MVL-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]://2^vcindex
               0;

  /************ Save vc_in values to not stall control pipeline ************/
  always@(posedge clk)
    if (!resetn)
      pipe_advance_s2_r<=0;
    else
      pipe_advance_s2_r<=pipe_advance[1];

  always@(posedge clk)
    if (!resetn)
    begin
      vc_in_saved<=0;
      vl_in_saved<=0;
      vbase_in_saved<=0;
      vinc_in_saved<=0;
      vstride_in_saved<=0;
      vs_in_saved<=0;
    end
    else if (pipe_advance_s2_r)
    begin
      vc_in_saved<=vc_in;
      vl_in_saved<=vl_in;
      vbase_in_saved<=vbase_in;
      vinc_in_saved<=vinc_in;
      vstride_in_saved<=vstride_in;
      vs_in_saved<=vs_in;
    end

  wire [6:2] squash_vcpipe_NC;
  //transfer control pipelines based on pipe_advance
  pipe #(VCWIDTH,4) vcpipe (
      .d( (!pipe_advance_s2_r) ? vc_in_saved : vc_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vcpipe_NC),
      .q( {vc[((6-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((6-2)*((VCWIDTH-1)-(0)+1))],vc[((5-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((5-2)*((VCWIDTH-1)-(0)+1))],vc[((4-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((4-2)*((VCWIDTH-1)-(0)+1))],vc[((3-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((3-2)*((VCWIDTH-1)-(0)+1))],vc[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]} ));

  wire [6:2] squash_vlpipe_NC; //transferring vl_Length
  pipe #(VCWIDTH,4) vlpipe (
      .d( (!pipe_advance_s2_r) ? vl_in_saved : vl_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vlpipe_NC),
      .q( {vl[((6-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((6-2)*((VCWIDTH-1)-(0)+1))],vl[((5-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((5-2)*((VCWIDTH-1)-(0)+1))],vl[((4-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((4-2)*((VCWIDTH-1)-(0)+1))],vl[((3-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((3-2)*((VCWIDTH-1)-(0)+1))],vl[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]} ));

  wire [6:2] squash_vbasepipe_NC; //base address
  pipe #(VCWIDTH,4) vbasepipe (
      .d( (!pipe_advance_s2_r) ? vbase_in_saved : vbase_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,(ctrl2_memunit_en|ctrl2_axi_en)} ),
      .squash(squash_vbasepipe_NC),
      .q( {vbase[((6-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((6-2)*((VCWIDTH-1)-(0)+1))],vbase[((5-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((5-2)*((VCWIDTH-1)-(0)+1))],vbase[((4-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((4-2)*((VCWIDTH-1)-(0)+1))],vbase[((3-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((3-2)*((VCWIDTH-1)-(0)+1))],vbase[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]} ));

  wire [6:2] squash_vincpipe_NC;
  pipe #(VCWIDTH,4) vincpipe (
      .d( (!pipe_advance_s2_r) ? vinc_in_saved : vinc_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vincpipe_NC),
      .q( {vinc[((6-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((6-2)*((VCWIDTH-1)-(0)+1))],vinc[((5-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((5-2)*((VCWIDTH-1)-(0)+1))],vinc[((4-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((4-2)*((VCWIDTH-1)-(0)+1))],vinc[((3-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((3-2)*((VCWIDTH-1)-(0)+1))],vinc[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]} ));

  //stride register also used for elmt shamt for vext/vhalf/etc in memunit
  wire [6:2] squash_vstridepipe_NC;
  pipe #(VCWIDTH,4) vstridepipe (
      .d( (ctrl2_memunit_en&~ctrl2_mem_en) ? 
            ((LOG2NUMLANES>0) ?
                  total_shamt[((LOG2NUMLANES>0) ? LOG2NUMLANES : 1)-1:0] : 0) :
          (!pipe_advance_s2_r) ? vstride_in_saved : vstride_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:2] & {4'b1,ctrl2_memunit_en} ),
      .squash(squash_vstridepipe_NC),
      .q( {vstride[((6-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((6-2)*((VCWIDTH-1)-(0)+1))],vstride[((5-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((5-2)*((VCWIDTH-1)-(0)+1))],vstride[((4-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((4-2)*((VCWIDTH-1)-(0)+1))],vstride[((3-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((3-2)*((VCWIDTH-1)-(0)+1))],vstride[((2-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((2-2)*((VCWIDTH-1)-(0)+1))]} ));


  /*****************************    ISSUER    *****************************/
  //dispatching to the execution units
  assign stall_dispatcher=
    //Structural hazard
    //last_subvector- last cycle in vector operation (will be high in last cycle)
    //have this for all the extra functional units that are added as well
    //active low??
    (|(ctrl3_memunit_en&~last_subvector) && ctrl2_memunit_en) ||
    (|(ctrl3_mulshift_en&~last_subvector) && ctrl2_mulshift_en) ||
    (|(ctrl3_matmul_en&~last_subvector) && ctrl2_matmul_en) ||
    (|(ctrl3_bfadder_en&~last_subvector) && ctrl2_bfadder_en) ||
    (|(ctrl3_bfmult_en&~last_subvector) && ctrl2_bfmult_en) ||
    (|(ctrl3_act_en&~last_subvector) && ctrl2_act_en) ||
    (|(ctrl3_trp_en&~last_subvector) && ctrl2_trp_en) ||
    (|(ctrl3_transpose_en&~last_subvector) && ctrl2_transpose_en) ||
    (|(ctrl3_permute_en&~last_subvector) && ctrl2_permute_en) ||
    (|(ctrl3_axi_en&~last_subvector) && ctrl2_axi_en) ||
    (|(ctrl3_alufalu_en&~last_subvector) && ctrl2_alufalu_en && ALUPERBANK==0)||
    //Entry 0 is taken
    (dispatcher_rotate && ctrl2_useslanes);

  //assign stall_mulcooldown=|(ctrl3_mulshift_en&last_subvector);
  assign stall_mulcooldown=1'b0;

  assign dispatcher_shift=pipe_advance[3];
  // Rotate if last entry is alive (count not -1) and greater than zero
  assign dispatcher_rotate=(~count[NUMBANKS*(LOG2MVL-LOG2NUMLANES+1)-1]) && 
                          ((count>>((NUMBANKS-1)*(LOG2MVL-LOG2NUMLANES+1)))!=0);

`define DISPATCHWIDTH LOG2MVL+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    REGIDWIDTH+ \
    REGIDWIDTH+ \
    REGIDWIDTH+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    1+ \
    11+ \
    2+ \
    4+ \
    5+ \
    7+ \
    1+ \
    3+ \
    1+ \
    1+ \
    1+ \
    1+ \
    VCWIDTH+ \
    VSWIDTH+ \
    8+ \
    2+ \
    2+3+2 // This is fro Permute and Transpose unit enable + 3 This is for Permute op. store and read signals and +2 for axi signal 

wire [NUMBANKS*(`DISPATCHWIDTH)-1:0] dispatcher_instr;
  vdispatcher #(NUMBANKS, `DISPATCHWIDTH,LOG2MVL,LOG2MVL,LOG2MVL-LOG2NUMLANES+1)
    vdispatcher(
      .clk(clk),
      .resetn(resetn),
      .shift(dispatcher_shift),
      .rotate(dispatcher_rotate), //rotate and shift must be high to rotate
      .inshift_instr( {
          src_limit,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vr_c_we,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vf_c_we,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vs_we,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vr_a_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vr_b_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vf_a_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_vf_b_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_useslanes,
          (pipe_squash[2]) ? 1'b0 : ctrl2_memunit_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_mem_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_mulshift_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_matmul_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_bfadder_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_bfmult_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_act_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_trp_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_transpose_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_permute_en,
          (pipe_squash[2]) ? 1'b0 : ctrl2_axi_en,
          ctrl2_axi_req_type,
          ctrl2_permute_op,
          ctrl2_permute_store,
          ctrl2_permute_read,
          (pipe_squash[2]) ? 1'b0 : ctrl2_alufalu_en,
          dst_s2,
          src1_s2,
          src2_s2,
          src1scalar_s2,
          src2scalar_s2,
          imask_s2,
          ctrl2_vf_a_sel,
          ctrl2_rshiftnonzero,
          ctrl2_mem_dir_left,
          ctrl2_alu_op,
          ctrl2_satsum_op,
          ctrl2_satsize_op,
          ctrl2_mulshift_op,
          ctrl2_memunit_op,
          ctrl2_ismasked,
          ctrl2_flagalu_op,
          ctrl2_vf_wbsel,
          ctrl2_volatiledest,
          (!pipe_advance_s2_r) ? vc_in_saved : vc_in,
          (!pipe_advance_s2_r) ? vs_in_saved : vs_in,
          (pipe_squash[2]) ? 8'b0 : D_instr[((2-1)*((7)-(0)+1)+7-0) : ((2-1)*((7)-(0)+1))],
          trp_mode_s2
          }),
      .inshift_first(ctrl2_useslanes),
      .inshift_rdelm(src_start),
      .inshift_wrelm(dst_start),
      .inshift_count((pipe_squash[2] || !ctrl2_useslanes) ? -1 : (src_limit[LOG2MVL-1:0]>>LOG2NUMLANES) - (src_elm[LOG2MVL-1:0]>>LOG2NUMLANES) ),
      .increment((~wrongbank_s3)&{NUMBANKS{pipe_advance[3]}}),
      .rdelm_add_sub(0),
      .wrelm_add_sub(0),
      .count_add_sub(1),
      .rdelm_valuetoadd(NUMLANES),
      .wrelm_valuetoadd(NUMLANES),
      .count_valuetoadd(1),
      .instr(dispatcher_instr), //will take all control unit signals from previous stage and will pack into this instr
      .first(first_subvector),
      .rdelm(rdelm),
      .wrelm(wrelm),
      .count(count)
    );

  

  /******************* Stall on RAW HAZARDS ************************/
  //removed for loop for number of banks, bring it back again if needed
  always@*
    for (bd=0; bd<1+(NUMBANKS-1)*ALUPERBANK; bd=bd+1)
    begin
      alu_dst[0*REGIDWIDTH +: REGIDWIDTH] = dst[(FU_ALU+bd)*(`MAX_STAGES-4)*(REGIDWIDTH)+: REGIDWIDTH];

      alu_dst_we[0] = dst_we[(FU_ALU+bd)*(`MAX_STAGES-4)];

      falu_dst[0*REGIDWIDTH +: REGIDWIDTH] = dst[(FU_ALU+bd)*(`MAX_STAGES-4)*(REGIDWIDTH)+: REGIDWIDTH];

      falu_dst_we[0] = dst_we[(FU_FALU+bd)*(`MAX_STAGES-4)];
    end

  //TODO: hazard checker for the pipeline- need to update this for the other functional units 
  //src1_s2 , src2_s2 (source 1, stage 1 & son on) 
  hazardchecker #(REGIDWIDTH,VELMIDWIDTH,4+(NUMBANKS-1)*ALUPERBANK,NUMBANKS) 
    src1hazchecker(
      .src( src1_s2 |(src_start_delayed[LOG2MVL-1:0]>>LOG2NUMLANES) ),
      .src_valid(ctrl2_vr_a_en),
      .dst({
        alu_dst[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1))],
        dst[(`MAX_STAGES-4)*(FU_MUL*REGIDWIDTH)+ REGIDWIDTH-1:(`MAX_STAGES-4)*(FU_MUL*REGIDWIDTH) ],
        dst[(`MAX_STAGES-4)*(FU_MEM*REGIDWIDTH)+ REGIDWIDTH-1 : (`MAX_STAGES-4)*(FU_MEM*REGIDWIDTH)],
        dst[(`MAX_STAGES-4)*(FU_MATMUL*REGIDWIDTH)+REGIDWIDTH-1: (`MAX_STAGES-4)*(FU_MATMUL*REGIDWIDTH)]
        }),
      .dst_valid({
        alu_dst_we[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1))],
        dst_we[(FU_MUL)*(`MAX_STAGES-4)],
        dst_we[(FU_MEM)*(`MAX_STAGES-4)],
        dst_we[(FU_MATMUL)*(`MAX_STAGES-4)]
        }),
      .dst_mode({{3+(NUMBANKS-1)*ALUPERBANK{1'b0}},ctrl4_volatiledest}),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vr_c_we),
      .lt_mode(ctrl3_volatiledest),
      .haz(stall_hazsrc1));

  hazardchecker #(REGIDWIDTH,VELMIDWIDTH,4+(NUMBANKS-1)*ALUPERBANK,NUMBANKS) 
    src2hazchecker(
      .src( src2_s2 |(src_start_delayed[LOG2MVL-1:0]>>LOG2NUMLANES) ),
      .src_valid(ctrl2_vr_b_en),
      .dst({
        alu_dst[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1))],
        dst[(`MAX_STAGES-4)*(FU_MUL*REGIDWIDTH)+REGIDWIDTH-1: (`MAX_STAGES-4)*(FU_MUL*REGIDWIDTH)],
        dst[(`MAX_STAGES-4)*(FU_MEM*REGIDWIDTH)+REGIDWIDTH-1: (`MAX_STAGES-4)*(FU_MEM*REGIDWIDTH)],
        dst[(`MAX_STAGES-4)*(FU_MATMUL*REGIDWIDTH)+REGIDWIDTH-1: (`MAX_STAGES-4)*(FU_MATMUL*REGIDWIDTH)]
        }),
      .dst_valid({
        alu_dst_we[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1))],
        dst_we[(FU_MUL)*(`MAX_STAGES-4) ],
        dst_we[(FU_MEM)*(`MAX_STAGES-4)],
        dst_we[(FU_MATMUL)*(`MAX_STAGES-4)]
        }),
      .dst_mode({{3+(NUMBANKS-1)*ALUPERBANK{1'b0}},ctrl4_volatiledest}),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vr_c_we),
      .lt_mode(ctrl3_volatiledest),
      .haz(stall_hazsrc2));

  //Check flag hazards - flags always start at 0th element
  //there are no instrutions like branches which use the flags
  hazardchecker #(REGIDWIDTH,VELMIDWIDTH,1+(NUMBANKS-1)*ALUPERBANK,NUMBANKS) 
    fsrc1hazchecker(
      // Check for mask flag and src1 flag depending on instruction
      .src( (ctrl2_vf_a_sel) ? src1_s2 : (imask_s2<<VELMIDWIDTH)),
      .src_valid(ctrl2_vf_a_en),
      .dst(falu_dst[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1))]),
      .dst_valid(falu_dst_we[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1))]),
      .dst_mode(0),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vf_c_we),
      .lt_mode(0),
      .haz(stall_hazfsrc1));

  hazardchecker #(REGIDWIDTH,VELMIDWIDTH,1+(NUMBANKS-1)*ALUPERBANK,NUMBANKS) 
    fsrc2hazchecker(
      .src( src2_s2 ),
      .src_valid(ctrl2_vf_b_en),
      .dst(falu_dst[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)*REGIDWIDTH-1)-(0)+1))]),
      .dst_valid(falu_dst_we[((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1)+(1+(NUMBANKS-1)*ALUPERBANK)-1-0) : ((4-4)*(((1+(NUMBANKS-1)*ALUPERBANK)-1)-(0)+1))]),
      .dst_mode(0),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vf_c_we),
      .lt_mode(0),
      .haz(stall_hazfsrc2));


/******************************************************************************/
/************************** 3rd Pipeline Stage ********************************/
/******************************************************************************/

  always@*
    for (bi=0; bi<NUMBANKS; bi=bi+1) //NUMBANKS is currently 1
    begin
      {
        src_limit_s3[((bi-0)*((LOG2MVL-1)-(0)+1)+LOG2MVL-1-0) : ((bi-0)*((LOG2MVL-1)-(0)+1))],
        ctrl3_vr_c_we[bi],
        ctrl3_vf_c_we[bi],
        ctrl3_vs_we[bi],
        ctrl3_vr_a_en[bi],
        ctrl3_vr_b_en[bi],
        ctrl3_vf_a_en[bi],
        ctrl3_vf_b_en[bi],
        ctrl3_useslanes[bi],
        ctrl3_memunit_en[bi],
        ctrl3_mem_en[bi],
        ctrl3_mulshift_en[bi],
        ctrl3_matmul_en[bi],
        ctrl3_bfadder_en[bi],
        ctrl3_bfmult_en[bi],
        ctrl3_act_en[bi],
        ctrl3_trp_en[bi],
        ctrl3_transpose_en[bi],
        ctrl3_permute_en[bi],
        ctrl3_axi_en[bi],
        ctrl3_axi_req_type[bi],
        ctrl3_permute_op[bi],
        ctrl3_permute_store[bi],
        ctrl3_permute_read[bi],
        ctrl3_alufalu_en[bi],
        _dst_s3[((bi-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bi-0)*((REGIDWIDTH-1)-(0)+1))],
        _src1_s3[((bi-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bi-0)*((REGIDWIDTH-1)-(0)+1))],
        _src2_s3[((bi-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bi-0)*((REGIDWIDTH-1)-(0)+1))],
        src1scalar_s3[bi],
        src2scalar_s3[bi],
        imask_s3[bi],
        ctrl3_vf_a_sel[bi],
        ctrl3_rshiftnonzero[bi],
        ctrl3_mem_dir_left[bi],
        ctrl3_alu_op[((bi-0)*((10)-(0)+1)+10-0) : ((bi-0)*((10)-(0)+1))],
        ctrl3_satsum_op[((bi-0)*((1)-(0)+1)+1-0) : ((bi-0)*((1)-(0)+1))],
        ctrl3_satsize_op[((bi-0)*((3)-(0)+1)+3-0) : ((bi-0)*((3)-(0)+1))],
        ctrl3_mulshift_op[((bi-0)*((4)-(0)+1)+4-0) : ((bi-0)*((4)-(0)+1))],
        ctrl3_memunit_op[((bi-0)*((6)-(0)+1)+6-0) : ((bi-0)*((6)-(0)+1))],
        ctrl3_ismasked[bi],
        ctrl3_flagalu_op[((bi-0)*((2)-(0)+1)+2-0) : ((bi-0)*((2)-(0)+1))],
        ctrl3_vf_wbsel[bi],
        ctrl3_volatiledest[bi],
        vc_s3[((bi-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((bi-0)*((VCWIDTH-1)-(0)+1))],
        vs_s3[((bi-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((bi-0)*((VSWIDTH-1)-(0)+1))],
        D_instr_s3[((bi-0)*((7)-(0)+1)+7-0) : ((bi-0)*((7)-(0)+1))],
        trp_mode_s3[((bi-0)*((1)-(0)+1)+1-0) : ((bi-0)*((1)-(0)+1))]
      } = dispatcher_instr>>(bi*(`DISPATCHWIDTH));
      //dispatcher instr will be split into many control signals for the next stage
      //dispatcher might give same control signals for multiple cycles
      last_subvector[bi]=~|count[bi*(LOG2MVL-LOG2NUMLANES+1)+:LOG2MVL-LOG2NUMLANES];
      alive_s3[bi]=~count[(bi+1)*(LOG2MVL-LOG2NUMLANES+1)-1];

      for (i=0; i<NUMLANES; i=i+1)
        lane_en[((bi-0)*((NUMLANES-1)-(0)+1)+i-i) : ((bi-0)*((NUMLANES-1)-(0)+1))]= (LOG2NUMLANES==0) || //Support 1 lane
          ~((first_subvector[bi]) && 
              i<`LO(rdelm[bi*LOG2MVL +: LOG2MVL],LOG2NUMLANES) ||
            (last_subvector[bi]) && 
              i>( LOG2NUMLANES>0?(src_limit_s3[(bi*LOG2NUMLANES)+LOG2NUMLANES-1:(bi*LOG2NUMLANES)]):
                                 src_limit_s3[bi*LOG2NUMLANES:bi*LOG2NUMLANES]));
    end


  // ************* Map from issuer to register file banks *************
  
  always@* //NUMBANKS=1 for now, hence removing for loop
    for (b=0; b<NUMBANKS; b=b+1)
    begin 
      dst_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))]=_dst_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))] | (wrelm[b*LOG2MVL+:LOG2MVL]>>LOG2NUMLANES);
      t_dst_s3[b*REGIDWIDTH +: REGIDWIDTH]=dst_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))];
      src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))]=_src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))] | (rdelm[b*LOG2MVL+:LOG2MVL]>>LOG2NUMLANES);
      src2_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))]=_src2_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))] | (rdelm[b*LOG2MVL+:LOG2MVL]>>LOG2NUMLANES);

      wrongbank_s3[b]=(`LO(rdelm[b*LOG2MVL+:LOG2MVL]>>LOG2NUMLANES,LOG2NUMBANKS)!=b);
      //taking in values into the vector registers
      vr_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
      vr_b_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src2_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
      vr_a_en[b]=ctrl3_vr_a_en[b] && pipe_advance[3];
      vr_b_en[b]=ctrl3_vr_b_en[b] && pipe_advance[3];

      vf_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]= {(ctrl3_vf_a_sel[b]) ? _src1_s3[(b*REGIDWIDTH)+REGIDWIDTH-1:b*(REGIDWIDTH)+REGIDWIDTH-5] : imask_s3[b],
                                                                                      src1_s3[b*(REGIDWIDTH)+REGIDWIDTH-VRIDWIDTH-1:0]}>>LOG2NUMBANKS;
      vf_b_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src2_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
      vf_a_en[b]=ctrl3_vf_a_en[b] && pipe_advance[3];
      vf_b_en[b]=ctrl3_vf_b_en[b] && pipe_advance[3];
    end

  // ************* Map from issuer/banks to Functional Units *************
  always@(posedge clk)
    if (!resetn)
    begin
      dst_we_s4=0;
      ctrl4_mulshift_en=0;
      ctrl4_matmul_en=0;
      ctrl4_bfadder_en=0;
      ctrl4_bfmult_en=0;
      ctrl4_act_en=0;
      ctrl4_trp_en=0;
      ctrl4_transpose_en=0;
      ctrl4_permute_en=0;
      ctrl4_permute_op=0;
      ctrl4_permute_store=0;
      ctrl4_permute_read=0;
      ctrl4_axi_en=0;
      ctrl4_axi_req_type=0;
      ctrl4_memunit_en=0;
      ctrl4_mem_en=0;
      ctrl4_volatiledest=0;
    end
    else if (pipe_advance[3])
    begin
      dst_we_s4=0;
      ctrl4_mulshift_en=0;
      ctrl4_matmul_en=0;
      ctrl4_bfadder_en=0;
      ctrl4_bfmult_en=0;
      ctrl4_act_en=0;
      ctrl4_trp_en=0;
      ctrl4_transpose_en=0;
      ctrl4_permute_en=0;
      ctrl4_permute_op=0;
      ctrl4_permute_store=0;
      ctrl4_permute_read=0;
      ctrl4_axi_en=0;
      ctrl4_axi_req_type=0;
      ctrl4_memunit_en=0;
      ctrl4_mem_en=|ctrl3_mem_en;
      ctrl4_volatiledest=0;

      for (f3=0; f3<NUMFUS; f3=f3+1)
      begin
        D_instr_s4[((f3-0)*((7)-(0)+1)+7-0) : ((f3-0)*((7)-(0)+1))]=0;
        D_last_subvector_s4[f3]=0;
        dst_s4[((f3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((f3-0)*((REGIDWIDTH-1)-(0)+1))]=0;
      end
      //Controls will go to different functional units based on the enale signal
      for (b3=0; b3<NUMBANKS; b3=b3+1)
      begin
        //if instruction is alive && is in correct bank
        if ( alive_s3[b3]  &&     //alive
            ~wrongbank_s3[b3] &&  //correct bank
            ~pipe_squash[3])
          if (ctrl3_mulshift_en[b3])    //is multiply
          begin
            D_instr_s4[((FU_MUL-0)*((7)-(0)+1)+7-0) : ((FU_MUL-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_MUL]=last_subvector[b3];
            ctrl4_mulshift_en=ctrl3_mulshift_en[b3];
            banksel_s4[((FU_MUL-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MUL-0)*((NUMBANKS-1)-(0)+1))]=b3; //since non blocking statement taking from the last bank
            vs_s4[((FU_MUL-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_MUL-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_MUL-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_MUL-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_MUL]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_MUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MUL-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_MUL]=src1scalar_s3[b3]; //can take scalars as inputs as well
            src2scalar_s4[FU_MUL]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MUL]=ctrl3_ismasked[b3];
            ctrl4_mulshift_op=ctrl3_mulshift_op[((b3-0)*((4)-(0)+1)+4-0) : ((b3-0)*((4)-(0)+1))];
            ctrl4_rshiftnonzero=ctrl3_rshiftnonzero[b3];
          end
          else if (ctrl3_matmul_en[b3])    //is matmul
          begin
            D_instr_s4[((FU_MATMUL-0)*((7)-(0)+1)+7-0) : ((FU_MATMUL-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_MATMUL]=last_subvector[b3];
            ctrl4_matmul_en=ctrl3_matmul_en[b3];
            banksel_s4[((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_MATMUL-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_MATMUL-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_MATMUL-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_MATMUL-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_MATMUL]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_MATMUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MATMUL-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_MATMUL]=src1scalar_s3[b3];
            src2scalar_s4[FU_MATMUL]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MATMUL]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_bfadder_en[b3])    //is bfloat addition
          begin
            D_instr_s4[((FU_BFADDER-0)*((7)-(0)+1)+7-0) : ((FU_BFADDER-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_BFADDER]=last_subvector[b3];
            ctrl4_bfadder_en=ctrl3_bfadder_en[b3];
            banksel_s4[((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_BFADDER-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_BFADDER-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_BFADDER-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_BFADDER-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_BFADDER]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_BFADDER-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFADDER-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_BFADDER]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFADDER]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFADDER]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_bfmult_en[b3])    //is bfloat addition
          begin
            D_instr_s4[((FU_BFMULT-0)*((7)-(0)+1)+7-0) : ((FU_BFMULT-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_BFMULT]=last_subvector[b3];
            ctrl4_bfmult_en = ctrl3_bfmult_en[b3];
            banksel_s4[((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_BFMULT-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_BFMULT-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_BFMULT-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_BFMULT-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_BFMULT]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_BFMULT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFMULT-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_BFMULT]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFMULT]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFMULT]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_transpose_en[b3])    //is transpose
          begin
            D_instr_s4[((FU_TRANSPOSE-0)*((7)-(0)+1)+7-0) : ((FU_TRANSPOSE-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_TRANSPOSE]=last_subvector[b3];
            ctrl4_transpose_en = ctrl3_transpose_en[b3];
            banksel_s4[((FU_TRANSPOSE-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_TRANSPOSE-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_TRANSPOSE-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_TRANSPOSE-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_TRANSPOSE-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_TRANSPOSE-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_TRANSPOSE]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_TRANSPOSE]=src1scalar_s3[b3];
            src2scalar_s4[FU_TRANSPOSE]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_TRANSPOSE]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_permute_en[b3])    //is permute operation
          begin
            D_instr_s4[((FU_PERMUTE-0)*((7)-(0)+1)+7-0) : ((FU_PERMUTE-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_PERMUTE]=last_subvector[b3];
            ctrl4_permute_en = ctrl3_permute_en[b3]; 
            ctrl4_permute_op = ctrl3_permute_op;
            ctrl4_permute_store = ctrl3_permute_store;
            ctrl4_permute_read = ctrl3_permute_read;
            banksel_s4[((FU_PERMUTE-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_PERMUTE-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_PERMUTE-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_PERMUTE-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_PERMUTE-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_PERMUTE-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_PERMUTE]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_PERMUTE]=src1scalar_s3[b3];
            src2scalar_s4[FU_PERMUTE]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_PERMUTE]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_trp_en[b3])    //is trp unit
          begin
            D_instr_s4[((FU_TRP-0)*((7)-(0)+1)+7-0) : ((FU_TRP-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_TRP]=last_subvector[b3];
            ctrl4_trp_en = ctrl3_trp_en[b3];
            trp_mode_s4 = trp_mode_s3[((b3-0)*((1)-(0)+1)+1-0) : ((b3-0)*((1)-(0)+1))];
            banksel_s4[((FU_TRP-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_TRP-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_TRP-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_TRP-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_TRP-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_TRP-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_TRP]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_TRP-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRP-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_TRP]=src1scalar_s3[b3];
            src2scalar_s4[FU_TRP]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_TRP]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_act_en[b3])    //is activation
          begin
            D_instr_s4[((FU_ACT-0)*((7)-(0)+1)+7-0) : ((FU_ACT-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_ACT]=last_subvector[b3];
            ctrl4_act_en = ctrl3_act_en[b3];
            banksel_s4[((FU_ACT-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_ACT-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_ACT-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_ACT-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_ACT-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_ACT-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_ACT]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_ACT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_ACT-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_ACT]=src1scalar_s3[b3];
            src2scalar_s4[FU_ACT]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_ACT]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_axi_en[b3])    //is axi operation
          begin
            D_instr_s4[((FU_AXI-0)*((7)-(0)+1)+7-0) : ((FU_AXI-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_AXI]=last_subvector[b3];
            ctrl4_axi_en = ctrl3_axi_en[b3];
            ctrl4_axi_req_type = ctrl3_axi_req_type[b3];
            banksel_s4[((FU_AXI-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_AXI-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_AXI-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_AXI-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            vc_s4[((FU_AXI-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_AXI-0)*((VCWIDTH-1)-(0)+1))]=vc_s3[((b3-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((b3-0)*((VCWIDTH-1)-(0)+1))];
            dst_s4[((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_AXI]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_AXI-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_AXI-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_AXI]=src1scalar_s3[b3];
            src2scalar_s4[FU_AXI]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_AXI]=ctrl3_ismasked[b3];
            vbase_s4=(first_subvector&ctrl3_axi_en)? vbase[((3-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((3-2)*((VCWIDTH-1)-(0)+1))] : vbase_s4 + (2<<LOG2NUMLANES);
          end
          else if (ctrl3_memunit_en[b3] && !ctrl3_mem_en[b3]) //is memunit shift
          begin
            D_instr_s4[((FU_MEM-0)*((7)-(0)+1)+7-0) : ((FU_MEM-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_MEM]=last_subvector[b3];
            ctrl4_memunit_en=ctrl3_memunit_en[b3];
            banksel_s4[((FU_MEM-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MEM-0)*((NUMBANKS-1)-(0)+1))]=b3;
            vs_s4[((FU_MEM-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_MEM-0)*((VSWIDTH-1)-(0)+1))]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            dst_s4[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_MEM]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_MEM-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MEM-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_MEM]=src1scalar_s3[b3];
            src2scalar_s4[FU_MEM]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MEM]=ctrl3_ismasked[b3];
            ctrl4_volatiledest=ctrl3_volatiledest[b3];
            ctrl4_mem_dir_left=ctrl3_mem_dir_left[b3];
            ctrl4_memunit_op=ctrl3_memunit_op[((b3-0)*((6)-(0)+1)+6-0) : ((b3-0)*((6)-(0)+1))];
            mem_last_subvector_s4=last_subvector[b3];
          end
          else if (ctrl3_memunit_en[b3] &&  ctrl3_mem_en[b3]) //is mem operation
          begin
            D_instr_s4[((FU_MEM-0)*((7)-(0)+1)+7-0) : ((FU_MEM-0)*((7)-(0)+1))]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_MEM]=last_subvector[b3];
            ctrl4_memunit_en=ctrl3_memunit_en[b3];
            banksel_s4[((FU_MEM-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MEM-0)*((NUMBANKS-1)-(0)+1))]=b3;
            // Use vs to store current length for prefetching
            vs_s4[((FU_MEM-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_MEM-0)*((VSWIDTH-1)-(0)+1))]=count[b3*(LOG2MVL-LOG2NUMLANES+1)+:(LOG2MVL-LOG2NUMLANES)]<<LOG2NUMLANES;
            dst_s4[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_MEM]=ctrl3_vr_c_we[b3];
            vlane_en[((FU_MEM-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MEM-0)*((NUMLANES-1)-(0)+1))]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_MEM]=src1scalar_s3[b3];
            src2scalar_s4[FU_MEM]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MEM]=ctrl3_ismasked[b3];
            ctrl4_volatiledest=ctrl3_volatiledest[b3];
            ctrl4_mem_dir_left=ctrl3_mem_dir_left[b3];
            ctrl4_memunit_op=ctrl3_memunit_op[((b3-0)*((6)-(0)+1)+6-0) : ((b3-0)*((6)-(0)+1))];
            //Load base on first subvector or if INDEXED memory operation
            vbase_s4=(|(first_subvector&ctrl3_mem_en) || ctrl_memunit_op[3][5])?
              vbase[3] : vbase_s4 + ((((ctrl_memunit_op[3][4])? vstride[3]: 2)
                      <<ctrl_memunit_op[3][3:2])<<LOG2NUMLANES);
            // Partial Address Gen for each lane - just do multiplication part
            for (m=0; m<NUMLANES; m=m+1)
              vstrideoffset_s4[m*VCWIDTH +: VCWIDTH] = 
                                ((ctrl_memunit_op[3][4]) ? vstride[3] : 1)*m;
            mem_last_subvector_s4=last_subvector[b3];
          end
          else if (ctrl3_alu_op[((b3-0)*((10)-(0)+1)+10-0) : ((b3-0)*((10)-(0)+1))]!=(ALUOP_ZERO^ALUOP_ZERO)) //is ALU
          begin
            D_instr_s4[FU_ALU+b3*ALUPERBANK]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_ALU+b3*ALUPERBANK]=last_subvector[b3];
            banksel_s4[FU_ALU+b3*ALUPERBANK]=b3;
            vs_s4[FU_ALU+b3*ALUPERBANK]=vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            dst_we_s4[FU_ALU+b3*ALUPERBANK]=ctrl3_vr_c_we[b3];
            dst_s4[FU_ALU+b3*ALUPERBANK]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            dst_we_s4[FU_FALU+b3*ALUPERBANK]=ctrl3_vf_c_we[b3];
            dst_s4[FU_FALU+b3*ALUPERBANK]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            vlane_en[FU_ALU+b3*ALUPERBANK]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_ALU+b3*ALUPERBANK]=src1scalar_s3[b3];
            src2scalar_s4[FU_ALU+b3*ALUPERBANK]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_ALU+b3*ALUPERBANK]=ctrl3_ismasked[b3];
            ctrl4_alu_op[((b3*ALUPERBANK-0)*((10)-(0)+1)+10-0) : ((b3*ALUPERBANK-0)*((10)-(0)+1))]=ctrl3_alu_op[((b3-0)*((10)-(0)+1)+10-0) : ((b3-0)*((10)-(0)+1))];
            ctrl4_satsum_op[((b3*ALUPERBANK-0)*((1)-(0)+1)+1-0) : ((b3*ALUPERBANK-0)*((1)-(0)+1))]=ctrl3_satsum_op[((b3-0)*((1)-(0)+1)+1-0) : ((b3-0)*((1)-(0)+1))];
            ctrl4_satsize_op[((b3*ALUPERBANK-0)*((3)-(0)+1)+3-0) : ((b3*ALUPERBANK-0)*((3)-(0)+1))]=ctrl3_satsize_op[((b3-0)*((3)-(0)+1)+3-0) : ((b3-0)*((3)-(0)+1))];
            ctrl4_vf_wbsel[b3*ALUPERBANK]=ctrl3_vf_wbsel[b3];
          end
          else if (ctrl3_vf_c_we[b3])
          begin                                    //is FALU
            D_instr_s4[FU_FALU+b3*ALUPERBANK]=D_instr_s3[((b3-0)*((7)-(0)+1)+7-0) : ((b3-0)*((7)-(0)+1))];
            D_last_subvector_s4[FU_FALU+b3*ALUPERBANK]=last_subvector[b3];
            banksel_s4[FU_FALU+b3*ALUPERBANK]=b3;
            vs_s4[FU_FALU+b3*ALUPERBANK]=|vs_s3[((b3-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((b3-0)*((VSWIDTH-1)-(0)+1))];
            dst_we_s4[FU_FALU+b3*ALUPERBANK]=ctrl3_vf_c_we[b3];
            dst_s4[FU_FALU+b3*ALUPERBANK]=dst_s3[((b3-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((b3-0)*((REGIDWIDTH-1)-(0)+1))];
            vlane_en[FU_FALU+b3*ALUPERBANK]=lane_en[((b3-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((b3-0)*((NUMLANES-1)-(0)+1))];
            src1scalar_s4[FU_FALU+b3*ALUPERBANK]=src1scalar_s3[b3];
            src2scalar_s4[FU_FALU+b3*ALUPERBANK]=src2scalar_s3[b3];
            ctrl4_flagalu_op[((b3*ALUPERBANK-0)*((2)-(0)+1)+2-0) : ((b3*ALUPERBANK-0)*((2)-(0)+1))]=ctrl3_flagalu_op[((b3-0)*((2)-(0)+1)+2-0) : ((b3-0)*((2)-(0)+1))];
            ctrl4_vf_wbsel[b3*ALUPERBANK]=ctrl3_vf_wbsel[b3];
            ctrl4_vf_a_sel[b3*ALUPERBANK]=ctrl3_vf_a_sel[b3];
          end
        end
      end

  //vector register 
  vregfile_vector 
    #(NUMBANKS,LOG2NUMBANKS,8*VPW*NUMLANES,32*MVL/NUMLANES,REGIDWIDTH) 
    vregfile_vector (
      .clk(clk),
      .resetn(resetn),
      .a_reg(vr_a_reg),
      .a_readdataout(_vr_a_readdataout),
      .a_en(vr_a_en),
      .b_reg(vr_b_reg),
      .b_readdataout(_vr_b_readdataout),
      .b_en(vr_b_en),
      .c_reg(_vr_c_reg),
      .c_writedatain(_vr_c_writedatain),
      .c_byteen(vr_c_byteen),
      .c_we(vr_c_we));

  vregfile_flag 
    #(NUMBANKS,LOG2NUMBANKS,NUMLANES,32*MVL/NUMLANES,REGIDWIDTH) 
    vregfile_flag (
      .clk(clk),
      .resetn(resetn),
      .a_reg(vf_a_reg),
      .a_readdataout(vf_a_readdataout),
      .a_en(vf_a_en),
      .b_reg(vf_b_reg),
      .b_readdataout(vf_b_readdataout),
      .b_en(vf_b_en),
      .c_reg(vf_c_reg),
      .c_writedatain(vf_c_writedatain),
      .c_we(vf_c_we));


  pipereg #(1) sdstwepipereg (
      .d( |ctrl3_vs_we ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[3] ),
      .squashn( ~pipe_squash[3] ),
      .q(ctrl_vs_we[4]));

/******************************************************************************/
/************************** 4th Pipeline Stage ********************************/
/******************************************************************************/

  //Convert register file width (8*VPW) to datpath width (LANEWIDTH)
  always@*
    for (bn=0; bn<NUMBANKS; bn=bn+1)
      for (n=0; n<NUMLANES; n=n+1)
      begin
        vr_a_readdataout[((bn-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)) +: ( LANEWIDTH)] = 
          _vr_a_readdataout[bn*8*VPW*NUMLANES + 8*VPW*n +: 8*VPW];
        vr_b_readdataout[((bn-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)) +: ( LANEWIDTH)] = 
          _vr_b_readdataout[bn*8*VPW*NUMLANES + 8*VPW*n +: 8*VPW];
        _vr_c_writedatain[bn*8*VPW*NUMLANES + 8*VPW*n +: 8*VPW] = 
          vr_c_writedatain[((bn-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)) +: ( LANEWIDTH)];
      end

  //Bank Multiplex for each functional unit
  always@*
  begin
    for (fn=0; fn<=FU_MUL; fn=fn+1)
    begin
      vr_src1[fn*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES] =(src1scalar_s4[fn]) ? {NUMLANES{vs_s4[((fn-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((fn-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_a_readdataout[banksel_s4[((fn-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((fn-0)*((NUMBANKS-1)-(0)+1))]];
      vr_src2[fn*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES] =(src2scalar_s4[fn]) ? {NUMLANES{vs_s4[((fn-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((fn-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_b_readdataout[banksel_s4[((fn-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((fn-0)*((NUMBANKS-1)-(0)+1))]];
      vf_src1[((fn-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((fn-0)*((NUMLANES-1)-(0)+1))] = vf_a_readdataout[banksel_s4[((fn-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((fn-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES];

      vmask[((fn-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((fn-0)*((NUMLANES-1)-(0)+1))] =vlane_en[((fn-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((fn-0)*((NUMLANES-1)-(0)+1))] &
            ((ctrl4_ismasked[fn]) ?
              vf_a_readdataout[banksel_s4[((fn-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((fn-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
              {NUMLANES{1'b1}}) ;
    end

    vr_src1[((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))] = vr_a_readdataout[banksel_s4[((FU_MEM-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MEM-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src2[((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))] = vr_b_readdataout[banksel_s4[((FU_MEM-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MEM-0)*((NUMBANKS-1)-(0)+1))]];
    vmask[((FU_MEM-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MEM-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_MEM-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MEM-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_MEM]) ?  
             vf_a_readdataout[banksel_s4[((FU_MEM-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MEM-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;
    //source operand for the functional units, whether from vector register file or the scalar source
    //for scalar replicate the same data across all the lanes
    vr_src1[FU_MATMUL*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES] =(src1scalar_s4[FU_MATMUL]) ? {NUMLANES{vs_s4[((FU_MATMUL-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_MATMUL-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_a_readdataout[banksel_s4[((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src2[FU_MATMUL*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES] =(src2scalar_s4[FU_MATMUL]) ? {NUMLANES{vs_s4[((FU_MATMUL-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_MATMUL-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_b_readdataout[banksel_s4[((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src1[FU_BFADDER*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES] =(src1scalar_s4[FU_BFADDER]) ? {NUMLANES{vs_s4[((FU_BFADDER-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_BFADDER-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_a_readdataout[banksel_s4[((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src2[FU_BFADDER*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES] =(src2scalar_s4[FU_BFADDER]) ? {NUMLANES{vs_s4[((FU_BFADDER-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_BFADDER-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_b_readdataout[banksel_s4[((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src1[FU_BFMULT*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES] =(src1scalar_s4[FU_BFMULT]) ? {NUMLANES{vs_s4[((FU_BFMULT-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_BFMULT-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_a_readdataout[banksel_s4[((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src2[FU_BFMULT*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES] =(src2scalar_s4[FU_BFMULT]) ? {NUMLANES{vs_s4[((FU_BFMULT-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_BFMULT-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_b_readdataout[banksel_s4[((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src1[FU_TRP*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES] =(src1scalar_s4[FU_TRP]) ? {NUMLANES{vs_s4[((FU_TRP-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_TRP-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_a_readdataout[banksel_s4[((FU_TRP-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_TRP-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src1[FU_TRANSPOSE*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES] =(src1scalar_s4[FU_TRANSPOSE]) ? {NUMLANES{vs_s4[((FU_TRANSPOSE-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_TRANSPOSE-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_a_readdataout[banksel_s4[((FU_TRANSPOSE-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_TRANSPOSE-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src1[FU_PERMUTE*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES] =(src1scalar_s4[FU_PERMUTE]) ? {NUMLANES{vs_s4[((FU_PERMUTE-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_PERMUTE-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_b_readdataout[banksel_s4[((FU_PERMUTE-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_PERMUTE-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src1[FU_AXI*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES] =(src1scalar_s4[FU_AXI]) ? {NUMLANES{vs_s4[((FU_AXI-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_AXI-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_b_readdataout[banksel_s4[((FU_AXI-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_AXI-0)*((NUMBANKS-1)-(0)+1))]];
    vr_src1[FU_ACT*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES] =(src1scalar_s4[FU_ACT]) ? {NUMLANES{vs_s4[((FU_ACT-0)*((VSWIDTH-1)-(0)+1)+LANEWIDTH-1-0) : ((FU_ACT-0)*((VSWIDTH-1)-(0)+1))]}} : 
                                    vr_a_readdataout[banksel_s4[((FU_ACT-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_ACT-0)*((NUMBANKS-1)-(0)+1))]];
    
    //TODO: change it, don't use vf_a.. i.e flag registers for creating this mask
    vmask[((FU_MATMUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MATMUL-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_MATMUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MATMUL-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_MATMUL]) ?  
             vf_a_readdataout[banksel_s4[((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_MATMUL-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vmask[((FU_ACT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_ACT-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_ACT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_ACT-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_ACT]) ?  
             vf_a_readdataout[banksel_s4[((FU_ACT-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_ACT-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vmask[((FU_BFADDER-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFADDER-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_BFADDER-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFADDER-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_BFADDER]) ?  
             vf_a_readdataout[banksel_s4[((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFADDER-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vmask[((FU_BFMULT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFMULT-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_BFMULT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFMULT-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_BFMULT]) ?  
             vf_a_readdataout[banksel_s4[((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_BFMULT-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vmask[((FU_TRP-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRP-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_TRP-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRP-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_TRP]) ?  
             vf_a_readdataout[banksel_s4[((FU_TRP-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_TRP-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vmask[((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_TRANSPOSE]) ?  
             vf_a_readdataout[banksel_s4[((FU_TRANSPOSE-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_TRANSPOSE-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    vmask[((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_PERMUTE]) ?  
             vf_a_readdataout[banksel_s4[((FU_PERMUTE-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_PERMUTE-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;
    vmask[((FU_AXI-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_AXI-0)*((NUMLANES-1)-(0)+1))] =  vlane_en[((FU_AXI-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_AXI-0)*((NUMLANES-1)-(0)+1))] &
           ((ctrl4_ismasked[FU_AXI]) ?  
             vf_a_readdataout[banksel_s4[((FU_AXI-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((FU_AXI-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES] :
             {NUMLANES{1'b1}}) ;

    for (fn2=FU_FALU; fn2<=FU_FALU+(NUMBANKS-1)*ALUPERBANK; fn2=fn2+1)
    begin
      vf_src1[fn2] =(src1scalar_s4[fn2]&ctrl4_vf_a_sel[fn2-FU_FALU]) ? {NUMLANES{vs_s4[((fn2-0)*((VSWIDTH-1)-(0)+1)+0-0) : ((fn2-0)*((VSWIDTH-1)-(0)+1))]}} :
        vf_a_readdataout[banksel_s4[((fn2-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((fn2-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES];

      //Only FALU uses this
      vf_src2[fn2] = (src2scalar_s4[fn2]) ? {NUMLANES{vs_s4[((fn2-0)*((VSWIDTH-1)-(0)+1)+0-0) : ((fn2-0)*((VSWIDTH-1)-(0)+1))]}} : 
        vf_b_readdataout[banksel_s4[((fn2-0)*((NUMBANKS-1)-(0)+1)+NUMBANKS-1-0) : ((fn2-0)*((NUMBANKS-1)-(0)+1))]*NUMLANES +: NUMLANES];
    end

  end

/******************************************************************************/
/************************** 5th Pipeline Stage ********************************/
/******************************************************************************/

  // *********************** FUNCTIONAL UNITS *********************
  //
  // Note that functional units (recently) carry the register they're
  // writing to and a write signal so that once the result is computed
  // it can issue the full register write request.  As a result, it 
  // needs to know the status of the pipeline (pipe_advance and pipe_squash)

  // If mem_unit is stalled, pipeline can still go on if both
  //    i) it's a store hence requiring no writeback bank, AND
  //    ii) another memory instruction isn't waiting on the memunit
  //assign stall_memunit=_stall_memunit && (dst_we[FU_MEM][5] || (ctrl4_memunit_en && ~stall_mulunit && ~stall_matmul));
  assign stall_memunit= 1'b0;

//  pipereg #(1) memen5pipereg (
//      .d( ctrl4_mem_en ),
//      .clk(clk),
//      .resetn(resetn),
//      .en( pipe_advance[4] && ~_stall_memunit ),
//      .squashn( ~pipe_squash[4] | _stall_memunit ),
//      .q(ctrl5_mem_en ));

  pipereg #(1) memen5pipereg (
      .d( ctrl4_mem_en ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4]),
      .squashn( ~pipe_squash[4]),
      .q( ctrl5_mem_en ));

// AXI interface logic
//

wire [REGIDWIDTH-1:0] axi_dst;
wire [NUMLANES-1:0] axi_mask;
wire axi_dst_we;

axi4_master #(
    .P_TARGET_SLAVE_BASE_ADDR(0),                           
    .P_ADDR_WIDTH(11),                                      
    .P_DATA_WIDTH(128)                                      
)inst_axi_master
(
    .req_addr      (vbase_s4[10:0]),
    .req_data      (vr_src1[FU_AXI*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES]),
    .req_en        (ctrl4_axi_en),
    .req_type      (ctrl4_axi_req_type),
    .req_read_data (axi_out),
    .axi_busy      (),
    .in_dst        (),
    .in_dst_we     (),
    .in_dst_mask   (),
    .out_dst       (axi_dst),
    .out_dst_we    (axi_dst_we),
    .out_dst_mask  (axi_mask),
    .CLOCK         (clk    ),                                                          
    .RESET         (resetn    ),                                                          
    .AWID          (M_AWID     ),                                                           
    .AWADDR        (M_AWADDR   ),                                                         
    .AWLEN         (M_AWLEN    ),                                                          
    .AWSIZE        (M_AWSIZE   ),                                                         
    .AWBURST       (M_AWBURST  ),                                                        
    .AWLOCK        (M_AWLOCK   ),                                                         
    .AWCACHE       (M_AWCACHE  ),                                                        
    .AWPROT        (M_AWPROT   ),                                                         
    .AWQOS         (M_AWQOS    ),                                                          
    .AWVALID       (M_AWVALID  ),                                                        
    .AWREADY       (M_AWREADY  ),                                                        
    .WDATA         (M_WDATA    ),                                                          
    .WSTRB         (M_WSTRB    ),                                                          
    .WLAST         (M_WLAST    ),                                                          
    .WVALID        (M_WVALID   ),                                                         
    .WREADY        (M_WREADY   ),                                                         
    .ARID          (M_ARID     ),                                                           
    .ARADDR        (M_ARADDR   ),                                                         
    .ARLEN         (M_ARLEN    ),                                                          
    .ARSIZE        (M_ARSIZE   ),                                                         
    .ARBURST       (M_ARBURST  ),                                                        
    .ARLOCK        (M_ARLOCK   ),                                                         
    .ARCACHE       (M_ARCACHE  ),                                                        
    .ARPROT        (M_ARPROT   ),                                                         
    .ARQOS         (M_ARQOS    ),                                                          
    .ARVALID       (M_ARVALID  ),                                                        
    .ARREADY       (M_ARREADY  ),                                                        
    .RID           (M_RID      ),                                                            
    .RDATA         (M_RDATA    ),                                                          
    .RRESP         (M_RRESP    ),                                                          
    .RLAST         (M_RLAST    ),                                                          
    .RVALID        (M_RVALID   ),                                                         
    .RREADY        (M_RREADY   ),                                                         
    .BREADY        (M_BREADY   ),                                                         
    .BVALID        (M_BVALID   ),                                                         
    .BRESP         (M_BRESP    ),                                                          
    .BID           (M_BID      )                                                             
);

wire [11:0] axi_addr;
wire [NUMLANES*LANEWIDTH-1:0] axi_data,axi_readdata;
wire axi_req_en;
wire axi_req_type;

axi4_slave #(
    .P_ADDR_WIDTH(11),
    .P_DATA_WIDTH(128)
)
inst_axi_slave
(
    .req_addr      (axi_addr),
    .req_data      (axi_data),
    .req_en        (axi_req_en),
    .req_type      (axi_req_type),
    .req_read_data (axi_readata),
    .axi_busy      (),
    .CLOCK         (clk),                                                          
    .RESET         (resetn),                                                          
    .AWID          (S_AWID     ),                                                           
    .AWADDR        (S_AWADDR   ),                                                         
    .AWLEN         (S_AWLEN    ),                                                          
    .AWSIZE        (S_AWSIZE   ),                                                         
    .AWBURST       (S_AWBURST  ),                                                        
    .AWLOCK        (S_AWLOCK   ),                                                         
    .AWCACHE       (S_AWCACHE  ),                                                        
    .AWPROT        (S_AWPROT   ),                                                         
    .AWQOS         (S_AWQOS    ),                                                          
    .AWVALID       (S_AWVALID  ),                                                        
    .AWREADY       (S_AWREADY  ),                                                        
    .WDATA         (S_WDATA    ),                                                          
    .WSTRB         (S_WSTRB    ),                                                          
    .WLAST         (S_WLAST    ),                                                          
    .WVALID        (S_WVALID   ),                                                         
    .WREADY        (S_WREADY   ),                                                         
    .ARID          (S_ARID     ),                                                           
    .ARADDR        (S_ARADDR   ),                                                         
    .ARLEN         (S_ARLEN    ),                                                          
    .ARSIZE        (S_ARSIZE   ),                                                         
    .ARBURST       (S_ARBURST  ),                                                        
    .ARLOCK        (S_ARLOCK   ),                                                         
    .ARCACHE       (S_ARCACHE  ),                                                        
    .ARPROT        (S_ARPROT   ),                                                         
    .ARQOS         (S_ARQOS    ),                                                          
    .ARVALID       (S_ARVALID  ),                                                        
    .ARREADY       (S_ARREADY  ),                                                        
    .RID           (S_RID      ),                                                            
    .RDATA         (S_RDATA    ),                                                          
    .RRESP         (S_RRESP    ),                                                          
    .RLAST         (S_RLAST    ),                                                          
    .RVALID        (S_RVALID   ),                                                         
    .RREADY        (S_RREADY   ),                                                         
    .BREADY        (S_BREADY   ),                                                         
    .BVALID        (S_BVALID   ),                                                         
    .BRESP         (S_BRESP    ),                                                          
    .BID           (S_BID      )                                                             
);

  //
  //  ///---------------------------------------


// ----------------------------------------


  //============== Memory Unit =============
 
 wire [127:0] dma_lane_wrdata,dma_lane_rddata;
// wire [NUMLANES-1:0] vmem_local_en;
 wire [11*NUMLANES-1:0] dma_lane_addr;

// assign vmem_local_en = {NUMLANES{pipe_advance[4]}};

 wire squash_per_lane_mem_dstpipe_NC;
 wire squash_per_lane_mem_dstmask_NC;

//destination value
pipe #(REGIDWIDTH,3) lane_mem_dstpipe (
  .d( dst_s4[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash(squash_per_lane_mem_dstpipe_NC),

  .q(dst[(`MAX_STAGES-4)*(FU_MATMUL*REGIDWIDTH)+ 4*REGIDWIDTH-1: 
     (`MAX_STAGES-4)*(FU_MATMUL*REGIDWIDTH)]));
    
//for write enable
pipe #(1,3) lane_mem_dstwepipe (
  .d( dst_we_s4[FU_MEM]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash( pipe_squash[6:4] ),
  .q(dst_we[FU_MEM*(`MAX_STAGES-4)+3-:4]));

//if a lane is masked, you won't write into that lane
pipe #(NUMLANES,3) lane_mem_dstmaskpipe (
  .d( vmask[((FU_MEM-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MEM-0)*((NUMLANES-1)-(0)+1))]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[4] ),
  .squash(squash_per_lane_mem_dstmask_NC),

  .q(dst_mask[FU_MEM*(`MAX_STAGES-4)+4*(NUM_LANES)-1-:4*NUM_LANES]));

wire     [ LANEWIDTH*NUMLANES-1 : 0 ]   load_mem_result_s5;

wire [11* NUMLANES-1:0] mux_lane_addr;
wire [NUMLANES*LANEWIDTH-1:0] mux_lane_data, mux_lane_rddata;
wire [NUMLANES-1:0] mux_lane_rden, mux_lane_wren;
wire [NUMLANES-1:0] dma_lane_rden,dma_lane_wren;

 //choose data from axi or dma
 //can read and write through axi (the master will initiaite the transactions)
 dma_axi_mux#(
   .WIDTH(LANEWIDTH),
   .NUMLANES(NUMLANES),
   .ADDRWIDTH(11)
 )inst_dma_axi_mux(
    .dma_addr(dma_lane_addr),
    .dma_data(dma_lane_wrdata),
    .dma_rden(dma_lane_rden),
    .dma_wren(dma_lane_wren),    
    .dma_out(dma_lane_rddata),   
    .axi_addr(axi_addr),   
    .axi_data(axi_data),    
    .axi_req_en(axi_req_en),  
    .axi_req_type(axi_req_type),
    .axi_read_data(axi_readata),
    .mem_addr(mux_lane_addr),    
    .mem_data(mux_lane_data),    
    .mem_rden(mux_lane_rden),    
    .mem_wren(mux_lane_wren),    
    .mem_readdata(mux_lane_rddata)
 );

 vmem_local inst_vmem_local(
   .clk(clk),
   .resetn(resetn),
   .en(pipe_advance[4]),
   .op(ctrl4_memunit_op),
   .address_a(vbase_s4[10:0]),
   .stride_val_a(vstrideoffset_s4[VCWIDTH-1:0]),
   .offset_a(vr_src1[FU_MEM*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES]),
   .data_a(vr_src2[((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
   .out_a(load_mem_result_s5),
   .last_subvector(mem_last_subvector_s4),
   .address_b(mux_lane_addr),
   .rden_b(mux_lane_rden),
   .wren_b(mux_lane_wren),
   .data_b(mux_lane_data),
   .out_b(mux_lane_rddata)
  );

  dma #(
    .NUMLANES(NUMLANES),
    .WIDTH(LANEWIDTH),
    .ADDRWIDTH(11),
    .DMEM_ADDRWIDTH(32)
  )inst_dma(
   .clk(clk),
   .resetn(resetn),
   .mem_addr(mem_addr),
   .num_bytes(num_bytes),
   .dma_en(dma_en),
   .lane_addr(lane_addr),
   .we(dma_we),
  
   .local_addr(dma_lane_addr),
   .local_wren(dma_lane_wren),
   .local_rden(dma_lane_rden),
   .local_wrdata(dma_lane_wrdata),
   .local_rddata(dma_lane_rddata),
  
   .dma_busy(dma_busy),
   .dbus_address   (dma_dbus_address   ),
   .dbus_readdata  (dma_dbus_readdata  ), 
   .dbus_writedata (dma_dbus_writedata ),
   .dbus_byteen    (dma_dbus_byteen    ),
   .dbus_en        (dma_dbus_en        ),
   .dbus_wren      (dma_dbus_wren      ),
   .dbus_prefetch  (dma_dbus_prefetch  ),
   .dbus_wait      (dma_dbus_wait      ),
   .dbus_data_valid(dma_dbus_data_valid)
  );
  //TODO: remove vmem unit since it is not used
  vmem_unit vmem_unit(
    .clk(clk),
    .resetn(resetn),
    .enable(ctrl4_memunit_en && ~stall_mulunit && ~stall_matmul),  //unit on and pipe active
    .en(pipe_advance[6:4]),
    .squash(pipe_squash[6:4]),
    .op(ctrl4_memunit_op),
    .stall(_stall_memunit),
    .last_subvector(mem_last_subvector_s4),
    // Parameters ports
    .cbase(vbase_s4),
    .cstride(vstride[((4-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((4-2)*((VCWIDTH-1)-(0)+1))]),
    .cprefetch(vs_s4[((FU_MEM-0)*((VSWIDTH-1)-(0)+1)+VSWIDTH-1-0) : ((FU_MEM-0)*((VSWIDTH-1)-(0)+1))]),
    // Vector ports
    .vmask(vmask[((FU_MEM-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MEM-0)*((NUMLANES-1)-(0)+1))]),
    .vstrideoffset(vstrideoffset_s4),
    .vindex(vr_src1[FU_MEM*LANEWIDTH*NUMLANES+LANEWIDTH*NUMLANES-1-:LANEWIDTH*NUMLANES]),
    .vwritedata(vr_src2[((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_MEM-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
    .voutput(load_result_s5),
    .voutput_we(load_result_mask_s5),
    // Writeback ports
    .in_dst(dst_s4[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))]),
    .in_dst_we(dst_we_s4[FU_MEM]),
    //.out_dst(dst[FU_MEM][5]),
    .out_dst(),
    //.out_dst_we(dst_we[FU_MEM][5]),
    .out_dst_we(),
    .in_vs_dst_we(ctrl_vs_we[4]),
    .out_vs_dst_we(ctrl_vs_we[5]),
    // Vector operations ports
    .sa(vstride[((4-2)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((4-2)*((VCWIDTH-1)-(0)+1))]),
    .dir_left(ctrl4_mem_dir_left),
    // Data memory interface
    .dmem_en(dbus_en),
    .dmem_address(dbus_address),
    .dmem_we(dbus_we),
    .dmem_byteen(dbus_byteen),
    .dmem_writedata(dbus_writedata),
    .dmem_readdata(dbus_readdata),
    .dmem_cachematch(dbus_cachematch),
    .dmem_cachemiss(dbus_cachemiss),
    .dmem_prefetch(dbus_prefetch),
    .dmem_wait(dbus_wait)
    );
  defparam 
    vmem_unit.VPUWIDTH=LANEWIDTH,         // Width of the Vector processing unit
    vmem_unit.NUMLANES=NUMLANES,      // Number of vector lanes
    vmem_unit.LOG2NUMLANES=LOG2NUMLANES, 
    vmem_unit.NUMPARALLELLANES=NUMMEMPARALLELLANES,   // Crossbar size
    vmem_unit.LOG2NUMPARALLELLANES=LOG2NUMMEMPARALLELLANES, 
    vmem_unit.CONTROLWIDTH=VCWIDTH,   // Width of the Control Registers
    vmem_unit.DMEM_WRITEWIDTH=DMEM_WRITEWIDTH, // Width of write bus to memory
    vmem_unit.LOG2DMEM_WRITEWIDTH=LOG2DMEM_WRITEWIDTH,
    vmem_unit.DMEM_READWIDTH=DMEM_READWIDTH,   // Width of read bus from memory
    vmem_unit.LOG2DMEM_READWIDTH=LOG2DMEM_READWIDTH,
    vmem_unit.ELMIDWIDTH=REGIDWIDTH-VRIDWIDTH,
    vmem_unit.REGIDWIDTH=REGIDWIDTH;

  assign dst[(`MAX_STAGES-4)*(FU_MEM*REGIDWIDTH) +REGIDWIDTH-1:(`MAX_STAGES-4)*(FU_MEM*REGIDWIDTH) ]=dst_s4[FU_MEM];
  assign dst_we[FU_MEM*(`MAX_STAGES-4)]=dst_we_s4[FU_MEM];

  //============== Multiplier Unit (spans stages 4-6) =============
  wire [3*REGIDWIDTH-1:0] temp_vmul_dst;
  vmul_unit vmul_unit(
    .clk(clk),
    .resetn(resetn),
    .op(ctrl4_mulshift_op),
    .activate(ctrl4_mulshift_en),
    .en(pipe_advance[6:4]),
    .squash(pipe_squash[6:4]),
    .stall(stall_mulunit),
    .opA(vr_src1[((FU_MUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_MUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
    .opB(vr_src2[((FU_MUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((FU_MUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
    .vshamt( (ctrl4_rshiftnonzero) ? vc_s4[((FU_MUL-0)*((VCWIDTH-1)-(0)+1)+VCWIDTH-1-0) : ((FU_MUL-0)*((VCWIDTH-1)-(0)+1))] : 0 ),
    .vmask(vmask[((FU_MUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MUL-0)*((NUMLANES-1)-(0)+1))]),
    .in_dst(dst_s4[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1))]),
    .in_dst_we(dst_we_s4[FU_MUL]),
    .out_dst(temp_vmul_dst),
    .out_dst_we(dst_we[(FU_MUL)*(`MAX_STAGES-4)+2+1-1-:3]),

    .out_dst_mask(dst_mask[FU_MUL*(`MAX_STAGES-4)+3*(NUM_LANES)-1-:3*NUM_LANES]),
    .result(mulshift_result_s5)
  );
  defparam 
    vmul_unit.LOG2WIDTH=LOG2LANEWIDTH,   // Width of the Vector processing unit
    vmul_unit.NUMMULLANES=NUMMULLANES,   // Number of vector lanes
    vmul_unit.LOG2NUMLANES=LOG2NUMLANES, // Number of vector lanes
    vmul_unit.REGIDWIDTH=REGIDWIDTH;

  assign dst[(`MAX_STAGES-4)*(FU_MUL*REGIDWIDTH)+ 3*REGIDWIDTH -1: (`MAX_STAGES-4)*(FU_MUL*REGIDWIDTH)] = temp_vmul_dst[3*REGIDWIDTH-1:2*REGIDWIDTH] ;
  assign dst[(`MAX_STAGES-4)*(FU_MUL*REGIDWIDTH)+ 2*REGIDWIDTH -1: (`MAX_STAGES-4)*(FU_MUL*REGIDWIDTH)] = temp_vmul_dst[2*REGIDWIDTH-1:1*REGIDWIDTH] ;
  assign dst[(`MAX_STAGES-4)*(FU_MUL*REGIDWIDTH)+ REGIDWIDTH-1: (`MAX_STAGES-4)*(FU_MUL*REGIDWIDTH)] = temp_vmul_dst[1*REGIDWIDTH-1:0*REGIDWIDTH] ;
  //============== ALU Unit =============

  //If APB value is true, create one ALU per bank (per lane)
  wire alu_cmpresult_s4_temp;
  generate
  for (bk=0; bk<1+(NUMBANKS-1)*ALUPERBANK; bk=bk+1)
  begin : aluperbank_gen
    for (k=0; k<NUMLANES; k=k+1)
    begin : lanes4_gen

      vlane_alu #(LANEWIDTH) valu(
        .clk(clk), 
        .resetn(resetn),
        .pipe_en(pipe_advance[4]),
        .pipe_squashn(~pipe_squash[4]),
        .src1(vr_src1[(FU_ALU+bk)*(LANEWIDTH*NUMLANES)+LANEWIDTH*k -1 +:LANEWIDTH]),
        .src2(vr_src2[(FU_ALU+bk)*(LANEWIDTH*NUMLANES)+LANEWIDTH*k -1 +:LANEWIDTH]),
        .mask(vf_src1[(FU_ALU+bk)*(LANEWIDTH*NUMLANES)+k-1-:1]),  //Note: MERGE is not masked
        .op(ctrl4_alu_op[((bk-0)*((10)-(0)+1)+10-0) : ((bk-0)*((10)-(0)+1))]^ALUOP_ZERO),
        .satsum_op(ctrl4_satsum_op[((bk-0)*((1)-(0)+1)+1-0) : ((bk-0)*((1)-(0)+1))]),
        .satsize_op(ctrl4_satsize_op[((bk-0)*((3)-(0)+1)+3-0) : ((bk-0)*((3)-(0)+1))]),
        .cmp_result(alu_cmpresult_s4_temp),
        .result(alu_result_s5[((bk-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)) +: ( LANEWIDTH)])
        );
      assign alu_cmpresult_s4[((bk-0)*((NUMLANES-1)-(0)+1)+k)]= alu_cmpresult_s4_temp;

      vlane_flagalu vflagalu(
        .clk(clk), 
        .resetn(resetn),
        .src1(vf_src1[(FU_FALU+bk)*(LANEWIDTH*NUMLANES)+k-1-:1]),
        .src2(vf_src2[(FU_FALU+bk)*(LANEWIDTH*NUMLANES)+k-1-:1]),
        .op(ctrl4_flagalu_op[((bk-0)*((2)-(0)+1)+2-0) : ((bk-0)*((2)-(0)+1))]),
        .result(flagalu_result_s4[((bk-0)*((NUMLANES-1)-(0)+1)+k-k) : ((bk-0)*((NUMLANES-1)-(0)+1))])
        );

      pipereg #(1) flagaluresultreg (
        .d( (ctrl4_vf_wbsel[bk]) ? alu_cmpresult_s4[((bk-0)*((NUMLANES-1)-(0)+1)+k-k) : ((bk-0)*((NUMLANES-1)-(0)+1))] : 
                                   flagalu_result_s4[((bk-0)*((NUMLANES-1)-(0)+1)+k-k) : ((bk-0)*((NUMLANES-1)-(0)+1))]),
        .clk(clk),
        .resetn(resetn),
        .en( pipe_advance[4] ), //Advance unless next stage stalled
        .squashn( 1'b1 ),
        .q(flagalu_result_s5[((bk-0)*((NUMLANES-1)-(0)+1)+k-k) : ((bk-0)*((NUMLANES-1)-(0)+1))])
        );
    end

    wire squash_aludstpipe_NC;
    wire [REGIDWIDTH*(1+1)-1:0] aludstpipe_bk_q;
    pipe #(REGIDWIDTH,1) aludstpipe (
      .d( dst_s4[FU_ALU+bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_aludstpipe_NC),
      .q(dst[(`MAX_STAGES-4)*(FU_ALU+bk)*REGIDWIDTH+ 2*REGIDWIDTH -1-: 2*REGIDWIDTH]));

    pipe #(1,3) aludstwepipe (
      .d( dst_we_s4[FU_ALU+bk] & ~ctrl4_vf_wbsel[bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:4] ),
      .squash( pipe_squash[6:4] ),
      .q(dst_we[(`MAX_STAGES-4)*(FU_ALU+bk)+1+1-1 -:2]));
  
           
    
    wire squash_aludstmaskpipe_NC;
    pipe #(NUMLANES,1) aludstmaskpipe (
      .d( vmask[FU_ALU+bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_aludstmaskpipe_NC),
      .q(dst_mask[(FU_ALU+bk)*(`MAX_STAGES-4)+2*(NUM_LANES)-1-:2*NUM_LANES]));

    wire squash_faludstpipe_NC;
    wire [REGIDWIDTH*(1+1)-1:0] faludstpipe_bk_q;
    pipe #(REGIDWIDTH,1) faludstpipe (
      .d( dst_s4[FU_FALU+bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squash(squash_faludstpipe_NC),
      .q(dst[(`MAX_STAGES-4)*(FU_FALU+bk)*REGIDWIDTH+ 2*REGIDWIDTH-1-: 2*REGIDWIDTH]));
    

    wire[1:0] faludstwepipe_bk_q;
    pipe #(1,3) faludstwepipe (
      .d( dst_we_s4[FU_FALU+bk] ),  
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[6:4] ),
      .squash( pipe_squash[6:4] ),
      .q(dst_we[(`MAX_STAGES-4)*(FU_FALU+bk)+1+1-1 -:2]));
  end
  endgenerate


 //This code is just for assigning from signals connected
 //to the matmul, which are linear multi bit signals, to the
 //multi-dimensional signals.
 wire [NUMLANES*REGIDWIDTH-1:0] dst_matmul;
 wire [NUMLANES-1:0] dst_we_matmul;
 wire [NUMLANES*NUMLANES-1:0] dst_mask_matmul;

 assign dst[(`MAX_STAGES-4)*(FU_MATMUL)*REGIDWIDTH +REGIDWIDTH-1-:REGIDWIDTH] = dst_matmul[(REGIDWIDTH*NUMLANES)-1:REGIDWIDTH*(NUMLANES-1)];
 assign dst_mask[FU_MATMUL*(`MAX_STAGES-4)+NUM_LANES-1-:NUM_LANES] = dst_mask_matmul[(NUMLANES*NUMLANES)-1:NUMLANES*(NUMLANES-1)];
 assign dst_we[(FU_MATMUL)*(`MAX_STAGES-4)] = dst_we_matmul[NUMLANES-1];


///////////////////////////
// Matmul unit
///////////////////////////
//

wire out_data_avail;

matmul_unit #(REGIDWIDTH,`MATMUL_STAGES,NUMLANES) u_matmul(
.clk(clk),
.resetn(resetn),
.activate(ctrl4_matmul_en),
.en({NUMLANES{pipe_advance[6]}}),
.squash({NUMLANES{pipe_squash[6]}}),
.stall(stall_matmul),
.stall_matmul(temp_stall_matmul),
.a_data(vr_src1[((FU_MATMUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+NUMLANES*LANEWIDTH-1-0) : ((FU_MATMUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
.b_data(vr_src2[((FU_MATMUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+NUMLANES*LANEWIDTH-1-0) : ((FU_MATMUL-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
.validity_mask_a_rows(matmul_masks_in[1*`MAT_MUL_SIZE-1:0*`MAT_MUL_SIZE]),
.validity_mask_a_cols(matmul_masks_in[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE]),
.validity_mask_b_rows(matmul_masks_in[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE]),
.validity_mask_b_cols(matmul_masks_in[3*`MAT_MUL_SIZE-1:2*`MAT_MUL_SIZE]),
.c_data(matmul_out), 
.vmask(vmask[((FU_MATMUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MATMUL-0)*((NUMLANES-1)-(0)+1))]),
.in_dst(dst_s4[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1))]),
.in_dst_we(dst_we_s4[FU_MATMUL]),
.out_dst(dst_matmul),
.out_dst_we(dst_we_matmul),
.out_data_avail(out_data_avail),
.out_dst_mask(dst_mask_matmul)
);

transpose #(
  .WIDTH(LANEWIDTH),
  .NUMSTAGES(NUMLANES)
)u_transpose(
  .clk(clk),
  .resetn(resetn),
  .en(ctrl4_transpose_en),
  .a(vr_src1[((FU_TRANSPOSE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+NUMLANES*LANEWIDTH-1-0) : ((FU_TRANSPOSE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
  .out(transpose_out),
  .busy(transpose_busy),
  .valid(transpose_valid)
);
 
 wire squash_transpose_dstpipe_NC;
 wire squash_transpose_dstmask_NC;

pipe #(REGIDWIDTH,9) transpose_dstpipe (
  .d( dst_s4[((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1))] ),  
  .clk(clk),
  .resetn(resetn),
  .en(pipe_advance[12:4]),
  .squash(squash_transpose_dstpipe_NC),
  .q(dst[FU_TRANPOSE*((`MAX_STAGES-4)*REGIDWIDTH)+10*REGIDWIDTH -1-:10*REGIDWIDTH]));


pipe #(1,9) transpose_dstwepipe (
  .d( dst_we_s4[FU_TRANSPOSE]),  
  .clk(clk),
  .resetn(resetn),
//  .en(pipe_advance[11:4]),
  .en(9'hff),
//  .squash( pipe_squash[11:4] ),
  .squash(9'h00 ),
   
  .q(dst_we[(FU_TRANSPOSE-0)*(`MAX_STAGES-4)+9+1-1:(FU_TRANSPOSE-0)*(`MAX_STAGES-4)]));


pipe #(NUMLANES,9) transpose_dstmaskpipe (
  .d(vmask[((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1))]),  
  .clk(clk),
  .resetn(resetn),
  .en(pipe_advance[12:4]),
  .squash(squash_transpose_dstmask_NC),

  .q(dst_mask[(FU_TRANSPOSE)*(`MAX_STAGES-4)+10*(NUM_LANES)-1-:10*NUM_LANES]));

reg [3*NUMLANES-1:0] permute_row_col_vec;

always@(*)begin

  permute_row_col_vec[3*1-1:3*0] =vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+3-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))];
  permute_row_col_vec[3*2-1:3*1] =vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES*1+3-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)+LANEWIDTH*NUMLANES*1-(0)+1))];
  permute_row_col_vec[3*3-1:3*2] =vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES*2+3-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)+LANEWIDTH*NUMLANES*2-(0)+1))];
  permute_row_col_vec[3*4-1:3*3] =vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES*3+3-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)+LANEWIDTH*NUMLANES*3-(0)+1))];
  permute_row_col_vec[3*5-1:3*4] =vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES*4+3-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)+LANEWIDTH*NUMLANES*4-(0)+1))];
  permute_row_col_vec[3*6-1:3*5] =vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES*5+3-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)+LANEWIDTH*NUMLANES*5-(0)+1))];
  permute_row_col_vec[3*7-1:3*6] =vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES*6+3-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)+LANEWIDTH*NUMLANES*6-(0)+1))];
  permute_row_col_vec[3*8-1:3*7] =vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES*7+3-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)+LANEWIDTH*NUMLANES*7-(0)+1))];
end

wire [REGIDWIDTH-1:0] permute_dst;
wire [NUMLANES-1:0] permute_mask;
wire permute_dst_we;

permute #(
  .WIDTH(LANEWIDTH),
  .NUMSTAGES(NUMLANES),
  .REGIDWIDTH(REGIDWIDTH)
) u_permute (
  .clk(clk),
  .resetn(resetn),
  .read(ctrl4_permute_read),
  .en(ctrl4_permute_en),
  .row_col_op(ctrl4_permute_op),
  .row_col_num(permute_row_col_vec),
  .load(ctrl4_permute_store),
  .a(vr_src1[((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+NUMLANES*LANEWIDTH-1-0) : ((FU_PERMUTE-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
  .in_dst(dst_s4[((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1))]),
  .in_dst_we(dst_we_s4[FU_PERMUTE]),
  .in_dst_mask(vmask[((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1))]),
  .out_dst(permute_dst),
  .out_dst_we(permute_dst_we),
  .out_dst_mask(permute_mask),
  .out(permute_out)
); 

//wire squash_permute_dstpipe_NC;
//wire squash_permute_dstmask_NC;
//
//pipe #(REGIDWIDTH,1) permute_dstpipe (
//  .d( dst_s4[FU_PERMUTE] ),  
//  .clk(clk),
//  .resetn(resetn),
//  .en(pipe_advance[4]),
//  .squash(squash_permute_dstpipe_NC),
//  .q({dst[FU_PERMUTE][5],
//      dst[FU_PERMUTE][4]}));
//
//pipe #(1,1) permute_dstwepipe (
//  .d( dst_we_s4[FU_PERMUTE]),  
//  .clk(clk),
//  .resetn(resetn),
////  .en(pipe_advance[11:4]),
//  .en(pipe_advance[4]),
////  .squash( pipe_squash[11:4] ),
//  .squash(0),
//  .q({dst_we[FU_PERMUTE][5],
//      dst_we[FU_PERMUTE][4]}));
//
//pipe #(NUMLANES,1) permute_dstmaskpipe (
//  .d(vmask[FU_PERMUTE]),  
//  .clk(clk),
//  .resetn(resetn),
//  .en(pipe_advance[4]),
//  .squash(squash_permute_dstmask_NC),
//  .q({dst_mask[FU_PERMUTE][5],
//      dst_mask[FU_PERMUTE][4]}));
//
trp_unit #(.WIDTH(LANEWIDTH)) u_trp (
.clk(clk),
.resetn(resetn),
.en(ctrl4_trp_en),
.a(vr_src1[((FU_TRP-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+NUMLANES*LANEWIDTH-1-0) : ((FU_TRP-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]),
.mode(trp_mode_s4),
.read(trp_read),
.busy(trp_busy),
.valid(trp_valid),
.out(trp_out));

 wire squash_trp_dstpipe_NC;
 wire squash_trp_dstmask_NC;

pipe #(REGIDWIDTH,2) trp_dstpipe (
  .d( dst_s4[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1))] ),  
  .clk(clk),
  .resetn(resetn),
  .en(~trp_busy ),
  .squash(squash_trp_dstpipe_NC),
  
  .q(dst[(`MAX_STAGES-4)*(FU_TRP)*REGIDWIDTH+ 3*REGIDWIDTH-: 3*REGIDWIDTH]));

pipe #(1,2) trp_dstwepipe (
  .d( dst_we_s4[FU_TRP]),  
  .clk(clk),
  .resetn(resetn),
  .en(~trp_busy ),
  .squash( pipe_squash[4] ),
  .q(dst_we[(FU_TRP)*(`MAX_STAGES-4)+2+1-1:(FU_TRP)*(`MAX_STAGES-4)]));

pipe #(NUMLANES,1) trp_dstmaskpipe (
  .d( vmask[((FU_TRP-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRP-0)*((NUMLANES-1)-(0)+1))]),  
  .clk(clk),
  .resetn(resetn),
  .en(~trp_busy),
  .squash(squash_trp_dstmask_NC),

  .q(dst_mask[(FU_TRP)*(`MAX_STAGES-4)+2*(NUM_LANES)-1-:2*NUM_LANES]));

///////////////////////////
// Bfloat unit
///////////////////////////
genvar g_func;
wire[5*NUMLANES-1:0] bfadd_excp;
wire[5*NUMLANES-1:0] bfmult_excp;
wire [NUMLANES-1:0] bfadder_output_valid;
wire [NUMLANES-1:0] bfmult_output_valid;

wire squash_bfadder_dstmask_NC;
wire squash_bfadder_dstpipe_NC;
wire squash_bfmult_dstmask_NC;
wire squash_bfmult_dstpipe_NC;

wire squash_activation_dstmask_NC;
wire squash_activation_dstpipe_NC;

pipe #(REGIDWIDTH,3) bfadddstpipe (
  .d( dst_s4[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1))] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash(squash_bfadder_dstpipe_NC),
  .q(dst[(`MAX_STAGES-4)*(FU_BFADDER)*REGIDWIDTH+ 4*REGIDWIDTH -1-: 4*REGIDWIDTH]
));
  
pipe #(1,3) bfadddstwepipe (
  .d( dst_we_s4[FU_BFADDER]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash( pipe_squash[6:4] ),
  .q(dst_we[(FU_BFADDER)*(`MAX_STAGES-4)+3+1-1:(FU_BFADDER)*(`MAX_STAGES-4)]));

pipe #(NUMLANES,3) bfadddstmaskpipe (
  .d( vmask[((FU_BFADDER-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFADDER-0)*((NUMLANES-1)-(0)+1))] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash(squash_bfadder_dstmask_NC),
  .q(dst_mask[(`MAX_STAGES-4)*(FU_BFADDER)*REGIDWIDTH+ 4*REGIDWIDTH -1-: 4*REGIDWIDTH]));
  //above line different from original rtl , probably a small mistake

pipe #(REGIDWIDTH,3) bfmultdstpipe (
  .d( dst_s4[FU_BFMULT] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash(squash_per_lane_mem_NC),

  .q(dst[(`MAX_STAGES-4)*(FU_BFMULT)*REGIDWIDTH+ 4*REGIDWIDTH -1-: 4*REGIDWIDTH]));

pipe #(1,3) bfmultdstwepipe (
  .d( dst_we_s4[FU_BFMULT]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[6:4] ),
  .squash( pipe_squash[6:4] ),

  .q(dst_we[(FU_BFMULT)*(`MAX_STAGES-4)+3+1-1:(FU_BFMULT)*(`MAX_STAGES-4)]));

pipe #(NUMLANES,3) bfmultdstmaskpipe (
  .d( vmask[FU_BFMULT] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[4] ),
  .squash(squash_bfmult_dstmask_NC),
  .q(dst_mask[(`MAX_STAGES-4)*(FU_BFMULT)*NUMLANES+ 4*NUMLANES -1-: 4*NUMLANES]));

pipe #(REGIDWIDTH,1) actdstpipe (
  .d( dst_s4[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1))] ),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[5:4] ),
  .squash(squash_activation_dstpipe_NC),

  .q(dst[(`MAX_STAGES-4)*(FU_ACT)*REGIDWIDTH+ 2*REGIDWIDTH -1-: 2*REGIDWIDTH]));

pipe #(1,1) actdstwepipe (
  .d( dst_we_s4[FU_ACT]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[5:4] ),
  .squash( pipe_squash[5:4] ),
  .q(dst_we[(FU_ACT)*(`MAX_STAGES-4)+1+1-1:(FU_ACT)*(`MAX_STAGES-4)]));

pipe #(NUMLANES,1) actdstmaskpipe (
  .d(vmask[((FU_ACT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_ACT-0)*((NUMLANES-1)-(0)+1))]),  
  .clk(clk),
  .resetn(resetn),
  .en( pipe_advance[5:4]),
  .squash(squash_activation_dstmask_NC),

  .q(dst_mask[(FU_ACT)*(`MAX_STAGES-4)+2*(NUM_LANES)-1-:2*NUM_LANES]));



generate
for(g_func =0; g_func <NUMLANES; g_func = g_func+1) begin

FPAddSub bfloat_add(
  .clk(clk),
  .rst(~resetn),
  .en(ctrl4_bfadder_en),
  .a(vr_src1[(FU_BFADDER)*(LANEWIDTH*NUMLANES)+(g_func)*LANEWIDTH+LANEWIDTH-1-:LANEWIDTH]),
  .b(vr_src2[(FU_BFADDER)*(LANEWIDTH*NUMLANES)+(g_func)*LANEWIDTH+LANEWIDTH-1-:LANEWIDTH]),
  .operation(1'b0),
  .result(bfadder_result_s5[g_func*LANEWIDTH +: LANEWIDTH]),
        .valid(bfadder_output_valid[g_func]),
  .flags(bfadd_excp[5*g_func +: 5]));

FPMult_16 bfloat_mult(
  .clk(clk),
  .rst(~resetn),
  .en(ctrl4_bfmult_en),
  .a(vr_src1[(FU_BFMULT)*(LANEWIDTH*NUMLANES)+(g_func)*LANEWIDTH+LANEWIDTH-1-:LANEWIDTH]),
  .b(vr_src2[(FU_BFMULT)*(LANEWIDTH*NUMLANES)+(g_func)*LANEWIDTH+LANEWIDTH-1-:LANEWIDTH]),
  .result(bfmult_result_s5[g_func*LANEWIDTH +: LANEWIDTH]),
        .valid(bfmult_output_valid[g_func]),
  .flags(bfmult_excp[5*g_func +: 5]));



//    bfloat_adder #(REGIDWIDTH) bf_add(
//    .clk(clk),
//    .resetn(resetn),
//    .en(ctrl4_bfadder_en),
//    .stall(stall_bf_adder),
//    .a(vr_src1[FU_BFADDER][g_func * LANEWIDTHE +: LANEWIDTH]),
//    .b(vr_src2[FU_BFADDER][g_func * LANEWIDTHE +: LANEWIDTH]),
//    .out(bfadder_result_s5[g_func*LANEWIDTH +: LANEWIDTH])
//    );
//    
//    bfloat_mult #(REGIDWIDTH) bf_mult(
//    .clk(clk),
//    .resetn(resetn),
//    .en(ctrl4_bfmult_en),
//    .stall(stall_bf_adder),
//    .a(vr_src1[FU_BFMULT][g_func * LANEWIDTHE +: LANEWIDTH]),
//    .b(vr_src2[FU_BFMULT][g_func * LANEWIDTHE +: LANEWIDTH]),
//    .out(bfmult_result_s5[g_func*LANEWIDTH +: LANEWIDTH])
//    );
 
///////////////////////////
// activation unit
///////////////////////////

    activation #(LANEWIDTH) inst_activation(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[((FU_ACT-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)) +LANEWIDTH-1-: LANEWIDTH]),
    .out(act_result_s5[g_func*LANEWIDTH +: LANEWIDTH])
    );
  end
 endgenerate

/******************************************************************************/
/************************** WB Pipeline Stage ********************************/
/******************************************************************************/
  //taking the wb_dst from all the different units
  generate
  for (ba=0; ba<1+(NUMBANKS-1)*ALUPERBANK; ba=ba+1)
  begin : wbaluperbank_gen

    assign wb_dst[(FU_ALU+ba)*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH]=dst[(`MAX_STAGES-4)*(FU_ALU+ba)*REGIDWIDTH+ 2*REGIDWIDTH -1-:REGIDWIDTH];
    assign wb_dst_we[FU_ALU+ba]=dst_we[(FU_ALU+ba)*(`MAX_STAGES-4)+1]  && ~pipe_squash[5];
    assign wb_dst_mask[FU_ALU+ba]=dst_mask[(FU_ALU+ba)*(`MAX_STAGES-4)+2*(NUM_LANES)-1-:NUM_LANES];
    assign D_wb_last_subvector[FU_ALU+ba]=D_last_subvector_s5[FU_ALU+ba];

    assign D_wb_last_subvector[FU_FALU+ba]=D_last_subvector_s5[FU_FALU+ba];
  end
  endgenerate

  assign wb_dst[FU_MEM*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH]=dst[(`MAX_STAGES-4)*(FU_MEM)*REGIDWIDTH+ 2*REGIDWIDTH -1-:REGIDWIDTH];
  assign wb_dst_we[FU_MEM]= dst_we[(FU_MEM)*(`MAX_STAGES-4)+1] && (~pipe_advance[5]|~pipe_squash[5]);
  assign wb_dst_mask[FU_MEM]= dst_mask[(FU_MEM)*(`MAX_STAGES-4)+2*(NUM_LANES)-1-:NUM_LANES];
  assign D_wb_last_subvector[FU_MEM]=D_last_subvector_s5[FU_MEM];

  assign wb_dst[FU_MUL*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH]=dst[(`MAX_STAGES-4)*(FU_MUL)*REGIDWIDTH+ 2*REGIDWIDTH -1-:REGIDWIDTH];
  assign wb_dst_we[FU_MUL]=dst_we[(FU_MUL)*(`MAX_STAGES-4)+1+1-1-:1] && ~pipe_squash[5];
  assign wb_dst_mask[FU_MUL]= dst_mask[(FU_MUL)*(`MAX_STAGES-4)+2*(NUM_LANES)-1-:NUM_LANES];
  assign D_wb_last_subvector[FU_MUL]=D_last_subvector_s5[FU_MUL];


  assign wb_dst[FU_BFADDER*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH] = dst[(`MAX_STAGES-4)*(FU_BFADDER)*REGIDWIDTH+ 5*REGIDWIDTH -1-:REGIDWIDTH];
  assign wb_dst_we[FU_BFADDER] = dst_we[(FU_BFADDER)*(`MAX_STAGES-4)+4] && ~pipe_squash[7];
  assign wb_dst_mask[FU_BFADDER] = dst_mask[(FU_MUL)*(`MAX_STAGES-4)+5*(NUM_LANES)-1-:NUM_LANES];
  assign D_wb_last_subvector[FU_BFADDER] = D_last_subvector_s5[FU_BFADDER];

  assign wb_dst[FU_BFMULT*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH] = dst[(`MAX_STAGES-4)*(FU_BFMULT)*REGIDWIDTH+ 5*REGIDWIDTH -1-:REGIDWIDTH];
  assign wb_dst_we[FU_BFMULT] = dst_we[(FU_BFMULT)*(`MAX_STAGES-4)+4] && ~pipe_squash[7];
  assign wb_dst_mask[FU_BFMULT] = dst_mask[(FU_BFMULT)*(`MAX_STAGES-4)+5*(NUM_LANES)-1-:NUM_LANES];
  assign D_wb_last_subvector[FU_BFMULT] = D_last_subvector_s5[FU_BFMULT];

  assign wb_dst[FU_ACT*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH] = dst[(`MAX_STAGES-4)*(FU_ACT)*REGIDWIDTH+ 2*REGIDWIDTH -1-:REGIDWIDTH];
  assign wb_dst_we[FU_ACT] = dst_we[(FU_ACT)*(`MAX_STAGES-4)+1]&& ~pipe_squash[5];
  assign wb_dst_mask[FU_ACT] = dst_mask[(FU_ACT)*(`MAX_STAGES-4)+2*(NUM_LANES)-1-:NUM_LANES];
  assign D_wb_last_subvector[FU_ACT] = D_last_subvector_s5[FU_ACT];

  assign wb_dst[FU_TRANSPOSE*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH] = dst[(`MAX_STAGES-4)*(FU_TRANSPOSE)*REGIDWIDTH+ 10*REGIDWIDTH -1-:REGIDWIDTH];
  assign wb_dst_we[FU_TRANSPOSE] = dst_we[(FU_TRANSPOSE)*(`MAX_STAGES-4)+9];
  assign wb_dst_mask[FU_TRANSPOSE] =  dst_mask[(FU_TRANSPOSE)*(`MAX_STAGES-4)+10*(NUM_LANES)-1-:NUM_LANES];
  assign D_wb_last_subvector[FU_TRANSPOSE] = D_last_subvector_s5[FU_TRANSPOSE];

  assign wb_dst[((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1))] = permute_dst;
  assign wb_dst_we[FU_PERMUTE] = permute_dst_we;
  assign wb_dst_mask[((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1))] = permute_mask;
  assign D_wb_last_subvector[FU_PERMUTE] = D_last_subvector_s5[FU_PERMUTE];

  assign wb_dst[((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1))] = axi_dst;
  assign wb_dst_we[FU_AXI] = axi_dst_we;
  assign wb_dst_mask[((FU_AXI-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_AXI-0)*((NUMLANES-1)-(0)+1))] = axi_mask;
  assign D_wb_last_subvector[FU_AXI] = D_last_subvector_s5[FU_AXI];

  assign wb_dst[FU_TRP*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH] = dst[(`MAX_STAGES-4)*(FU_TRP)*REGIDWIDTH+ 2*REGIDWIDTH-1 -:REGIDWIDTH];
  assign wb_dst_we[FU_TRP] = dst_we[(FU_TRP)*(`MAX_STAGES-4)+1] && trp_valid;
  assign wb_dst_mask[FU_TRP] =  dst_mask[(FU_TRP)*(`MAX_STAGES-4)+2*(NUM_LANES)-1-:NUM_LANES];
  assign D_wb_last_subvector[FU_TRP] = D_last_subvector_s5[FU_TRP];

  assign wb_dst[FU_MATMUL*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH]=dst[(`MAX_STAGES-4)*(FU_MATMUL)*REGIDWIDTH+ 2*REGIDWIDTH -1-:REGIDWIDTH];
  assign wb_dst_we[FU_MATMUL]= dst_we[(FU_MATMUL)*(`MAX_STAGES-4)] && out_data_avail;
  assign wb_dst_mask[FU_MATMUL]= dst_mask[(FU_MATMUL)*(`MAX_STAGES-4)+NUM_LANES-1-:NUM_LANES];
  //TODO: There is no code that assigns to the s31 var used below. Need to add that code
  //This is only a debug var, so it doesn't affect functionality
  assign D_wb_last_subvector[FU_MATMUL]=D_last_subvector_s31[FU_MATMUL];

  // ******************  Map functional units to banks ******************
  always@*
    for (bw=0; bw<NUMBANKS; bw=bw+1)
    begin 
      //do we want to write or not?
      vr_c_we[bw]=(wb_dst_we[FU_MUL] && `LO(wb_dst[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_MATMUL] && `LO(wb_dst[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_BFADDER] && `LO(wb_dst[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_BFMULT] && `LO(wb_dst[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_AXI] && `LO(wb_dst[((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_ACT] && `LO(wb_dst[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_TRP] && `LO(wb_dst[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_PERMUTE] && `LO(wb_dst[((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_TRANSPOSE] && `LO(wb_dst[((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_MEM] && `LO(wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw) ||
                  (wb_dst_we[FU_ALU] && `LO(wb_dst[((FU_ALU-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_ALU-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw && ALUPERBANK==0) ||
                  (wb_dst_we[FU_ALU+bw] && ALUPERBANK!=0);

      //TODO: Update this code for matmul. This is for debug only, so skipping it for now.
      //Tells test_bench when to record register file contents.
      //Record if instruction writes to VRF, on last subvector, and not stalled
      D_wb_instrdone[bw] = pipe_advance[5] && (
        ((ALUPERBANK==0) ?
          (dst_we[(FU_ALU)*(`MAX_STAGES-4)+1] && D_wb_last_subvector[FU_ALU] && `LO(wb_dst[FU_ALU*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH],LOG2NUMBANKS)==bw) :
          (dst_we[(FU_ALU+bw)*(`MAX_STAGES-4)+1] && D_wb_last_subvector[FU_ALU+bw])) || 
        (dst_we[(FU_MUL)*(`MAX_STAGES-4)+1] && D_wb_last_subvector[FU_MUL] && `LO(wb_dst[FU_MUL*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH],LOG2NUMBANKS)==bw) || 
        (dst_we[(FU_MEM)*(`MAX_STAGES-4)+1] && D_wb_last_subvector[FU_MEM] && `LO(wb_dst[FU_MEM*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH],LOG2NUMBANKS)==bw));

      //Take matmul output 
      //writing the output to the destination 
      //vr_c_reg has the destination addr, and vr_c_writedatain has the data to be written
      if (wb_dst_we[FU_MATMUL] && `LO(wb_dst[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_MATMUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MATMUL-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MATMUL-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= matmul_out;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MATMUL];
      end
      // Transpose Block writeback      
      else if (wb_dst_we[FU_TRANSPOSE] && `LO(wb_dst[((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRANSPOSE-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRANSPOSE-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= transpose_out;
        //Do ALU and FALU for last subvector
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_TRANSPOSE];
      end
      // permute block writeback
      else if (wb_dst_we[FU_PERMUTE] && `LO(wb_dst[((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_PERMUTE-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_PERMUTE-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= permute_out;
        //Do ALU and FALU for last subvector
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_PERMUTE];
      end
      // trp block  writeback
      else if (wb_dst_we[FU_TRP] && `LO(wb_dst[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_TRP-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_TRP-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_TRP-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= trp_out;
        //Do ALU and FALU for last subvector
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_TRP];
      end
      //Take bfadder output
      else if (wb_dst_we[FU_AXI] && `LO(wb_dst[((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_AXI-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_AXI-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_AXI-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= axi_out;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_AXI];
      end
      else if (wb_dst_we[FU_BFADDER] && `LO(wb_dst[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_BFADDER-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFADDER-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFADDER-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= bfadder_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_BFADDER];
      end
      else if (wb_dst_we[FU_BFMULT] && `LO(wb_dst[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_BFMULT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_BFMULT-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_BFMULT-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= bfmult_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_BFMULT];
      end
      else if (wb_dst_we[FU_ACT] && `LO(wb_dst[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_ACT-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_ACT-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_ACT-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= act_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_ACT];
      end
      //Take multiplier output
      else if (wb_dst_we[FU_MUL] && `LO(wb_dst[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_MUL-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MUL-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MUL-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= mulshift_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MUL];
      end
      //Take Memory unit output
      else if (wb_dst_we[FU_MEM] && `LO(wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))],LOG2NUMBANKS)==bw)
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[((FU_MEM-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((FU_MEM-0)*((NUMLANES-1)-(0)+1))];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((FU_MEM-0)*((REGIDWIDTH-1)-(0)+1))];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= load_mem_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MEM];
      end
      else
      //Take ALU output
      begin
        vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw-0)*((NUMLANES-1)-(0)+1))]=wb_dst_mask[FU_ALU+bw*ALUPERBANK];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[(FU_ALU+bw*ALUPERBANK)*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH]>>LOG2NUMBANKS;
        vr_c_reg[((bw-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1-0) : ((bw-0)*((REGIDWIDTH-1)-(0)+1))]= wb_dst[(FU_ALU+bw*ALUPERBANK)*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH];
        vr_c_writedatain[((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))]= alu_result_s5[((bw*ALUPERBANK-0)*((LANEWIDTH*NUMLANES-1)-(0)+1)+LANEWIDTH*NUMLANES-1-0) : ((bw*ALUPERBANK-0)*((LANEWIDTH*NUMLANES-1)-(0)+1))];
        //Do ALU and FALU for last subvector
        D_last_subvector_done[bw]=
          (D_wb_last_subvector[FU_ALU+bw*ALUPERBANK] && `LO(wb_dst[(FU_ALU+bw*ALUPERBANK)*REGIDWIDTH+REGIDWIDTH-1-:REGIDWIDTH],LOG2NUMBANKS)==bw) |

          (D_wb_last_subvector[FU_FALU+bw*ALUPERBANK] && `LO(dst[(`MAX_STAGES-4)*(FU_FALU+bw*ALUPERBANK)* REGIDWIDTH+2*REGIDWIDTH -1-:REGIDWIDTH],LOG2NUMBANKS)==bw);

                                                               
      end
      for (j=0; j<NUMLANES; j=j+1)
        vr_c_byteen[bw*VPW*NUMLANES + j*VPW +: VPW ]={VPW{vmask_final[((bw-0)*((NUMLANES-1)-(0)+1)+j-j) : ((bw-0)*((NUMLANES-1)-(0)+1))]}};

      //flag register writeback, not really using for now
      //*********** Flag writeback ***********

      vf_c_reg[bw*BANKREGIDWIDTH+:BANKREGIDWIDTH]=dst[(`MAX_STAGES-4)*(FU_FALU+bw*ALUPERBANK)*REGIDWIDTH+ 2*REGIDWIDTH -1-:REGIDWIDTH]>>LOG2NUMBANKS;
      vf_c_writedatain[bw*NUMLANES+:NUMLANES]=flagalu_result_s5[((bw*ALUPERBANK-0)*((NUMLANES-1)-(0)+1)+NUMLANES-1-0) : ((bw*ALUPERBANK-0)*((NUMLANES-1)-(0)+1))];
      vf_c_we[bw]=dst_we[(FU_FALU+bw*ALUPERBANK)*(`MAX_STAGES-4)+1] && `LO(dst[(FU_FALU+bw*ALUPERBANK)*(`MAX_STAGES-4)+5],LOG2NUMBANKS)==bw;
      
    end

  //********** Scalar writeback ***********
  assign vs_wetrack={ctrl_vs_we[5:4],|ctrl3_vs_we,ctrl2_vs_we};
  assign vs_we=ctrl_vs_we[5] & load_result_mask_s5[0];
  assign vs_dst= wb_dst[(FU_MEM*REGIDWIDTH)+ REGIDWIDTH-1:(FU_MEM*REGIDWIDTH)+REGIDWIDTH-VRIDWIDTH];
  assign vs_writedata=load_result_s5[LANEWIDTH-1:0];

endmodule
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: vlanes.v.temp
//////////////////////////////////////////////////////////////////////

module vpu (
    clk,
    resetn,

    // Instruction interface
    instr,
    instr_en,     // tells when instr is valid and available
    instr_wait,   // if high says vpu is not ready to receive
    has_memop,    // indicates vector pipeline has a memory operation

    // For mtc2/ctc2 instructions
    scalar_in,    
    scalar_in_en,
    scalar_in_wait,

    // For cfc2 instructions
    scalar_out,
    scalar_out_en,
    scalar_out_wait,
   
    // AXI interface
    M_AWID    ,
    M_AWADDR  ,
    M_AWLEN   ,
    M_AWSIZE  ,
    M_AWBURST ,
    M_AWLOCK  ,
    M_AWCACHE ,
    M_AWPROT  ,
    M_AWQOS   ,
    M_AWVALID ,
    M_AWREADY ,
    M_WDATA   ,
    M_WSTRB   ,
    M_WLAST   ,
    M_WVALID  ,
    M_WREADY  ,
    M_ARID    ,
    M_ARADDR  ,
    M_ARLEN   ,
    M_ARSIZE  ,
    M_ARBURST ,
    M_ARLOCK  ,
    M_ARCACHE ,
    M_ARPROT  ,
    M_ARQOS   ,
    M_ARVALID ,
    M_ARREADY ,
    M_RID     ,
    M_RDATA   ,
    M_RRESP   ,
    M_RLAST   ,
    M_RVALID  ,
    M_RREADY  ,
    M_BREADY  ,
    M_BVALID  ,
    M_BRESP   ,
    M_BID     ,

    S_AWID   , 
    S_AWADDR , 
    S_AWLEN  , 
    S_AWSIZE , 
    S_AWBURST, 
    S_AWLOCK , 
    S_AWCACHE, 
    S_AWPROT , 
    S_AWQOS  , 
    S_AWVALID, 
    S_AWREADY, 
    S_WDATA  , 
    S_WSTRB  , 
    S_WLAST  , 
    S_WVALID , 
    S_WREADY , 
    S_ARID   , 
    S_ARADDR , 
    S_ARLEN  , 
    S_ARSIZE , 
    S_ARBURST, 
    S_ARLOCK , 
    S_ARCACHE, 
    S_ARPROT , 
    S_ARQOS  , 
    S_ARVALID, 
    S_ARREADY, 
    S_RID    , 
    S_RDATA  , 
    S_RRESP  , 
    S_RLAST  , 
    S_RVALID , 
    S_RREADY , 
    S_BREADY , 
    S_BVALID , 
    S_BRESP  , 
    S_BID,     
    

    // Data memory interface
    dbus_address,
    dbus_en,
    dbus_we,
    dbus_byteen,
    dbus_writedata,
    dbus_readdata,
    dbus_cachematch,
    dbus_cachemiss,
    dbus_prefetch,
    dbus_wait,

    dma_dbus_address,   
    dma_dbus_readdata,  
    dma_dbus_writedata, 
    dma_dbus_byteen,
    dma_dbus_en,        
    dma_dbus_wren,      
    dma_dbus_prefetch,  
    dma_dbus_wait,      
    dma_dbus_data_valid   
    );

parameter LOG2NUMLANES=`LOG2NUMLANES;
parameter LOG2MVL=`LOG2MVL;
parameter LOG2VPW=`LOG2VPW;
parameter LOG2LANEWIDTH=`LOG2LANEWIDTHBITS;
parameter LOG2NUMMEMLANES=`LOG2NUMMEMLANES;
parameter NUMMULLANES=2**`LOG2NUMMULLANES;
parameter LOG2NUMBANKS=`LOG2NUMBANKS;
parameter ALUPERBANK=`ALUPERBANK;

parameter LOG2DMEM_WRITEWIDTH=7;  // Log2 of Width of write bus to cache
parameter LOG2DMEM_READWIDTH=7;   // Log2 of Width of read bus from cache

parameter NUMLANES=2**LOG2NUMLANES;
parameter MVL=2**LOG2MVL;
parameter VPW=2**LOG2VPW;
parameter LANEWIDTH=2**LOG2LANEWIDTH;
parameter DMEM_WRITEWIDTH=2**LOG2DMEM_WRITEWIDTH; //Width of write bus to memory
parameter DMEM_READWIDTH=2**LOG2DMEM_READWIDTH; // Width of read bus from memory
parameter NUMMEMLANES=2**LOG2NUMMEMLANES;
parameter NUMBANKS=2**LOG2NUMBANKS;

parameter VCWIDTH=32;
parameter NUMVCREGS=64;
parameter LOG2NUMVCREGS=6;
parameter NUMNONMEMVCREGS=32;
parameter LOG2NUMNONMEMVCREGS=5;
parameter NUMVBASEREGS=16;
parameter LOG2NUMVBASEREGS=4;
parameter NUMVINCREGS=8;
parameter LOG2NUMVINCREGS=3;
parameter NUMVSTRIDEREGS=8;
parameter LOG2NUMVSTRIDEREGS=3;

parameter VSWIDTH=32;
parameter NUMVSREGS=32;
parameter LOG2NUMVSREGS=5;

parameter BIT_VSSRC2=6;
parameter BIT_VSSRC1=7;

//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: visa.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/visa.v
//////////////////////////////////////////////////////////////////////
parameter COP2_VADD           = 'b1000000000;
parameter COP2_VADD_U         = 'b1000000001;
parameter COP2_VSUB           = 'b1000000010;
parameter COP2_VSUB_U         = 'b1000000011;
parameter COP2_VMULHI         = 'b1000000100;
parameter COP2_VMULHI_U       = 'b1000000101;
parameter COP2_VDIV           = 'b1000000110; //Using as matmul
//parameter COP2_VBFADD         = 'b0100000001; //Using BF16 add
//parameter COP2_VBFMULT        = 'b0100000010; //Using BF16 MULT
//parameter COP2_VACT           = 'b0100000011; //Using ACT
//parameter COP2_VTRP           = 'b0100000100; //Using ACT
parameter COP2_VDIV_U         = 'b1000000111;

//parameter COP2_VMOD           = 'b1000001000;
parameter COP2_VBFADD           = 'b1000001000;  // USING bfloat add Instr: vmod.vv vrdest, vrsrc1,vrsrc2

parameter COP2_VMOD_U         = 'b1000001001;
parameter COP2_VCMP_EQ        = 'b1000001010;
parameter COP2_VCMP_NE        = 'b1000001100;
parameter COP2_VCMP_LT        = 'b1000001110;
parameter COP2_VCMP_U_LT      = 'b1000001111;
parameter COP2_VCMP_LE        = 'b1000010000;
parameter COP2_VCMP_U_LE      = 'b1000010001;
parameter COP2_VMIN           = 'b1000010010;
parameter COP2_VMIN_U         = 'b1000010011;
parameter COP2_VMAX           = 'b1000010100;
parameter COP2_VMAX_U         = 'b1000010101;
parameter COP2_VMULLO         = 'b1000010110;
parameter COP2_VABS           = 'b1000010111;
parameter COP2_VAND           = 'b1000011000;
parameter COP2_VOR            = 'b1000011001;
parameter COP2_VXOR           = 'b1000011010;
parameter COP2_VNOR           = 'b1000011011;
parameter COP2_VSLL           = 'b1000011100;
parameter COP2_VSRL           = 'b1000011101;
parameter COP2_VSRA           = 'b1000011110;
parameter COP2_VSAT_B         = 'b1000011111;
parameter COP2_VSAT_H         = 'b1001011111;
//parameter COP2_VSAT_W         = 'b1010011111;
parameter COP2_VACT         = 'b1010011111;  // Using activation instruction: vsat.w vrdest,vrsrc
parameter COP2_VSAT_SU_B      = 'b1000100000;
parameter COP2_VSAT_SU_H      = 'b1001100000;
//parameter COP2_VSAT_SU_W      = 'b1010100000;
parameter COP2_VRED     = 'b1010100000;   // Using reduction instruction: vsat.su.w vrdest,vrsrc
parameter COP2_VSAT_SU_L      = 'b1011100000;
parameter COP2_VSAT_U_B       = 'b1000100001;
parameter COP2_VSAT_U_H       = 'b1001100001;
//parameter COP2_VSAT_U_W       = 'b1010100001;
parameter COP2_VTRP       = 'b1010100001;   // Using transpose instruction: vsat.u.w vrdest, vrsrc 
parameter COP2_VSADD          = 'b1000100010;
parameter COP2_VSADD_U        = 'b1000100011;
parameter COP2_VSSUB          = 'b1000100100;
parameter COP2_VSSUB_U        = 'b1000100101;
parameter COP2_VSRR           = 'b1000100110;
parameter COP2_VSRR_U         = 'b1000100111;
parameter COP2_VSLS           = 'b1000101000;
parameter COP2_VSLS_U         = 'b1000101001;
parameter COP2_VXUMUL         = 'b1000101010;
parameter COP2_VXUMUL_U       = 'b1000101011;
parameter COP2_VXLMUL         = 'b1000101100;
parameter COP2_VXLMUL_U       = 'b1000101101;
parameter COP2_VXUMADD        = 'b1000101110;
parameter COP2_VXUMADD_U      = 'b1000101111;
parameter COP2_VXUMSUB        = 'b1000110000;
parameter COP2_VXUMSUB_U      = 'b1000110001;
parameter COP2_VXLMADD        = 'b1000110010;
parameter COP2_VXLMADD_U      = 'b1000110011;
parameter COP2_VXLMSUB        = 'b1000110100;
parameter COP2_VXLMSUB_U      = 'b1000110101;
parameter COP2_VINS_VV        = 'b1100000000;
parameter COP2_VINS_SV        = 'b1110000001;
parameter COP2_VEXT_VV        = 'b1100000010;
parameter COP2_VEXT_SV        = 'b1100000011;
parameter COP2_VEXT_U_SV      = 'b1100000100;
parameter COP2_VCOMPRESS      = 'b1100000101;
parameter COP2_VEXPAND        = 'b1100000110;
parameter COP2_VMERGE         = 'b1100000111;
parameter COP2_VFINS          = 'b1110001000;
parameter COP2_VEXTHALF       = 'b1100001001;
parameter COP2_VHALF          = 'b1100001010;
parameter COP2_VHALFUP        = 'b1100001011;
parameter COP2_VHALFDN        = 'b1100001100;
parameter COP2_VSATVL         = 'b1100001101;
parameter COP2_VFAND          = 'b1100001110;
parameter COP2_VFOR           = 'b1100001111;
parameter COP2_VFXOR          = 'b1100010000;
parameter COP2_VFNOR          = 'b1100010001;
parameter COP2_VFCLR          = 'b1100010010;
parameter COP2_VFSET          = 'b1100010011;
parameter COP2_VIOTA          = 'b1100010100;
parameter COP2_VCIOTA         = 'b1100010101;
parameter COP2_VFPOP          = 'b1100010110;
parameter COP2_VFFF1          = 'b1100010111;
parameter COP2_VFFL1          = 'b1100011000;
parameter COP2_VFSETBF        = 'b1100011001;
parameter COP2_VFSETIF        = 'b1100011010;
parameter COP2_VFSETOF        = 'b1100011011;
parameter COP2_VFMT8          = 'b1100011100;
parameter COP2_VFMF8          = 'b1100011101;
parameter COP2_VFCLR8         = 'b1100011110;
parameter COP2_VFOR8          = 'b1100011111;
parameter COP2_VFLD           = 'b1100100000;
parameter COP2_VLD_B          = 'b1100100001;
parameter COP2_VLD_H          = 'b1101100001;

//parameter COP2_VLD_W          = 'b1110100001;
//parameter COP2_VBFADD         = 'b1110100001;  // adding bfadder Instr: vld.u.w

parameter COP2_VLD_L          = 'b1111100001;
parameter COP2_VLD_U_B        = 'b1100100010;
parameter COP2_VLD_U_H        = 'b1101100010;

//parameter COP2_VLD_U_W        = 'b1110100010;
parameter COP2_VAXIRD	        = 'b1110100010;  // adding an instruction for AXI Load;

parameter COP2_VLDS_B         = 'b1100100011;
parameter COP2_VLDS_H         = 'b1101100011;

//parameter COP2_VLDS_W         = 'b1110100011;
parameter COP2_VBFSUB         = 'b1110100011;   // adding bfsub Instr: vlds.w

parameter COP2_VLDS_L         = 'b1111100011;
parameter COP2_VLDS_U_B       = 'b1100100100;
parameter COP2_VLDS_U_H       = 'b1101100100;

//parameter COP2_VLDS_U_W       = 'b1110100100;
parameter COP2_VBFMULT         = 'b1110100100;   // adding bfmult Instr: vlds.u.w

parameter COP2_VLDX_B         = 'b1100100101;
parameter COP2_VLDX_H         = 'b1101100101;

//parameter COP2_VLDX_W         = 'b1110100101;
parameter COP2_VPER_STR         = 'b1110100101;     // adding transpose instruction: vldx.w

parameter COP2_VLDX_L         = 'b1111100101;
parameter COP2_VLDX_U_B       = 'b1100100110;
parameter COP2_VLDX_U_H       = 'b1101100110;

//parameter COP2_VLDX_U_W       = 'b1110100110;
//parameter COP2_VPER     = 'b1110100110;        //adding activation Instr: vldx.u.w 

parameter COP2_VFST           = 'b1100101000;
parameter COP2_VST_B          = 'b1100101001;
parameter COP2_VST_H          = 'b1101101001;

//parameter COP2_VST_W        = 'b1110101001;  // adding reduction Instr: vst.w
//parameter COP2_VRED           = 'b1110101001;  // adding reduction Instr: vst.w

parameter COP2_VST_L          = 'b1111101001;
parameter COP2_VSTS_B         = 'b1100101010;
parameter COP2_VSTS_H         = 'b1101101010;

//parameter COP2_VSTS_W         = 'b1110101010;
parameter COP2_VPER_LD           = 'b1110101010;  // adding permute Instr: vsts.w

parameter COP2_VSTS_L         = 'b1111101010;
parameter COP2_VSTX_B         = 'b1100101011;
parameter COP2_VSTX_H         = 'b1101101011;
//parameter COP2_VSTX_W         = 'b1110101011;
parameter COP2_VPER         = 'b1110101011;  // adding permute operation: vstx.w Vsrc,Vbase, 
parameter COP2_VSTX_L         = 'b1111101011;
parameter COP2_VSTXO_B        = 'b1100101100;
parameter COP2_VSTXO_H        = 'b1101101100;
//parameter COP2_VSTXO_W        = 'b1110101100;  
parameter COP2_VAXIWR        = 'b1110101100;  // adding an instruction for axi write
parameter COP2_VSTXO_L        = 'b1111101100;
parameter COP2_VMCTS          = 'b1101110000;
parameter COP2_VMSTC          = 'b1101110001;
parameter COP2_CFC2           = 'b0000111000;
parameter COP2_CTC2           = 'b0000111010;
parameter COP2_MTC2           = 'b0000111011;
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: visa.v
//////////////////////////////////////////////////////////////////////

input clk;
input resetn;

input [31:0] instr;
input instr_en;     // tells when instr is valid and available
output instr_wait;   // if high says vpu is not ready to receive

output has_memop;

// For mtc2/ctc2 instructions
input [31:0] scalar_in;    
input scalar_in_en;
output scalar_in_wait;

// For cfc2 instructions
output [31:0] scalar_out;
output scalar_out_en;
input scalar_out_wait;

// Data memory interface
output  [ 31 : 0 ]  dbus_address;
output              dbus_en;
output              dbus_we;
output  [ (DMEM_WRITEWIDTH/8)-1 : 0 ]   dbus_byteen;
output  [ DMEM_WRITEWIDTH-1 : 0 ]       dbus_writedata;
input   [ DMEM_READWIDTH-1 : 0 ]        dbus_readdata;
input         dbus_cachematch;
input         dbus_cachemiss;
output  [ 31 : 0 ]  dbus_prefetch;
input               dbus_wait;

//DMA interface
output [31:0]                 dma_dbus_address;   
input [DMEM_READWIDTH-1:0]    dma_dbus_readdata;  
output [DMEM_WRITEWIDTH-1:0]  dma_dbus_writedata; 
output [DMEM_WRITEWIDTH/8-1:0]dma_dbus_byteen;
output                        dma_dbus_en;        
output                        dma_dbus_wren;      
output                        dma_dbus_prefetch;  
input                         dma_dbus_wait;      
input                         dma_dbus_data_valid;

//AXI interface
output  [6 - 1:0]                    M_AWID;                                                 
output  [LANEWIDTH- 1:0]             M_AWADDR;                                    
output  [7:0]                        M_AWLEN;                                                    
output  [2:0]                        M_AWSIZE;                                                   
output  [1:0]                        M_AWBURST;                                                  
output                               M_AWLOCK;                                                         
output  [3:0]                        M_AWCACHE;                                                  
output  [2:0]                        M_AWPROT;                                                   
output  [3:0]                        M_AWQOS;                                                    
output                               M_AWVALID;                                                        
input                                M_AWREADY;                                                        
output  [NUMLANES*LANEWIDTH-1 : 0]   M_WDATA;                                     
output  [(NUMLANES*LANEWIDTH)/8-1 : 0] M_WSTRB;                                   
output                               M_WLAST;                                                          
output                               M_WVALID;                                                         
input                                M_WREADY;                                                         

output  [6-1 : 0]                    M_ARID;                                                 
output  [LANEWIDTH-1 : 0]            M_ARADDR;                                    
output  [7 : 0]                      M_ARLEN;                                                  
output  [2 : 0]                      M_ARSIZE;                                                 
output  [1 : 0]                      M_ARBURST;                                                
output                               M_ARLOCK;                                                         
output  [3 : 0]                      M_ARCACHE;                                                
output  [2 : 0]                      M_ARPROT;                                                 
output  [3 : 0]                      M_ARQOS;                                                  
output                               M_ARVALID;                                                        
input                                M_ARREADY;                                                        
input   [6-1 : 0]                    M_RID;                                                 
input   [NUMLANES*LANEWIDTH-1 : 0]   M_RDATA;                                     
input   [1 : 0]                      M_RRESP;                                                  
input                                M_RLAST;                                                          
input                                M_RVALID;                                                         
output                               M_RREADY;                                                         

output                               M_BREADY;                                                         
input                                M_BVALID;                                                         
input   [1 : 0]                      M_BRESP;                                                  
input   [6-1 : 0]                    M_BID;                                                   

input  [6 - 1:0]                    S_AWID;                                                 
input  [LANEWIDTH- 1:0]             S_AWADDR;                                    
input  [7:0]                        S_AWLEN;                                                    
input  [2:0]                        S_AWSIZE;                                                   
input  [1:0]                        S_AWBURST;                                                  
input                               S_AWLOCK;                                                         
input  [3:0]                        S_AWCACHE;                                                  
input  [2:0]                        S_AWPROT;                                                   
input  [3:0]                        S_AWQOS;                                                    
input                               S_AWVALID;                                                        
output                                S_AWREADY;                                                        
input  [NUMLANES*LANEWIDTH-1 : 0]   S_WDATA;                                     
input  [(NUMLANES*LANEWIDTH)/8-1 : 0] S_WSTRB;                                   
input                               S_WLAST;                                                          
input                               S_WVALID;                                                         
output                                S_WREADY;                                                         

input  [6-1 : 0]                    S_ARID;                                                 
input  [LANEWIDTH-1 : 0]            S_ARADDR;                                    
input  [7 : 0]                      S_ARLEN;                                                  
input  [2 : 0]                      S_ARSIZE;                                                 
input  [1 : 0]                      S_ARBURST;                                                
input                               S_ARLOCK;                                                         
input  [3 : 0]                      S_ARCACHE;                                                
input  [2 : 0]                      S_ARPROT;                                                 
input  [3 : 0]                      S_ARQOS;                                                  
input                               S_ARVALID;                                                        
output                                S_ARREADY;                                                        
output   [6-1 : 0]                    S_RID;                                                
output   [NUMLANES*LANEWIDTH-1 : 0]   S_RDATA;                                     
output   [1 : 0]                      S_RRESP;                                                  
output                                S_RLAST;                                                          
output                                S_RVALID;                                                         
input                               S_RREADY;                                                         

input                               S_BREADY;                                                         
output                                S_BVALID;                                                         
output   [1 : 0]                      S_BRESP;                                                  
output   [6-1 : 0]                    S_BID;


reg                        [ 31 : 0 ]   ir;
wire                       [ 31 : 0 ]   ir2;
wire                       [ 31 : 0 ]   ir3;

wire                                    stall1;
wire                                    stall2;
wire                                    stall3;
wire                        [ 3 : 1 ]   internal_stall;
wire                                    squash1;
wire                                    squash2;
wire        [`MAX_PIPE_STAGES-1 : 0 ]   vlanes_stalled;

wire                        [ 5 : 0 ]   ir_cop2;
wire                        [ 9 : 0 ]   ir_op;
wire                        [ 5 : 0 ]   ir_vcr;
wire                        [ 4 : 0 ]   ir_vsr;
wire                        [ 4 : 0 ]   ir_src2;
wire                        [ 4 : 0 ]   ir_src1;
wire                        [ 5 : 0 ]   ir_vcr_r;

wire    [ LOG2NUMNONMEMVCREGS-1 : 0 ]   vc_a_reg;
wire                [ VCWIDTH-1 : 0 ]   vc_a_readdataout;
wire                                    vc_a_en;
wire    [ LOG2NUMNONMEMVCREGS-1 : 0 ]   vc_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vc_c_writedatain;
wire                                    vc_c_we;
wire                [ VCWIDTH-1 : 0 ]   vl;
wire                [ VCWIDTH-1 : 0 ]   dma_en;
wire                [ VCWIDTH-1 : 0 ]   mem_addr;
wire                [ VCWIDTH-1 : 0 ]   num_bytes;
wire                [ VCWIDTH-1 : 0 ]   lane_addr;
wire                [ VCWIDTH-1 : 0 ]   dma_we;
//Masks for the matmul (3 masks. each is 8-bit.)
//1-bit for each row/column element in the matmul. we have an 8x8 matmul)
wire         [3*`MAT_MUL_SIZE-1 : 0 ]   matmul_masks;

wire       [ LOG2NUMVBASEREGS-1 : 0 ]   vbase_a_reg;
wire                [ VCWIDTH-1 : 0 ]   vbase_a_readdataout;
wire                                    vbase_a_en;
wire       [ LOG2NUMVBASEREGS-1 : 0 ]   vbase_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vbase_c_writedatain;
wire                                    vbase_c_we;

wire        [ LOG2NUMVINCREGS-1 : 0 ]   vinc_a_reg;
wire                                    vinc_a_en;
wire                [ VCWIDTH-1 : 0 ]   vinc_a_readdataout;
wire        [ LOG2NUMVINCREGS-1 : 0 ]   vinc_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vinc_c_writedatain;
wire                                    vinc_c_we;

wire     [ LOG2NUMVSTRIDEREGS-1 : 0 ]   vstride_a_reg;
wire                                    vstride_a_en;
wire                [ VCWIDTH-1 : 0 ]   vstride_a_readdataout;
wire     [ LOG2NUMVSTRIDEREGS-1 : 0 ]   vstride_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vstride_c_writedatain;
wire                                    vstride_c_we;

wire          [ LOG2NUMVSREGS-1 : 0 ]   vs_a_reg;
wire                [ VCWIDTH-1 : 0 ]   vs_a_readdataout;
wire                                    vs_a_en;
wire          [ LOG2NUMVSREGS-1 : 0 ]   vs_c_reg;
wire                [ VCWIDTH-1 : 0 ]   vs_c_writedatain;
wire                                    vs_c_we;

wire                [ VCWIDTH-1 : 0 ]   vc_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vl_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vbase_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vinc_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vstride_readdataout;
wire                [ VSWIDTH-1 : 0 ]   vs_readdataout;
wire                [ VCWIDTH-1 : 0 ]   vc_combined_out;
wire                [ VCWIDTH-1 : 0 ]   vc_combined_out_s3;
wire                [ VCWIDTH-1 : 0 ]   _vc_c_writedatain;
wire                [ VCWIDTH-1 : 0 ]   fwddata;
wire                                    fwd_vc;
wire                                    fwd_vl;
wire                                    fwd_vbase;
wire                                    fwd_vinc;
wire                                    fwd_vstride;

reg                                     ctrl_vc_a_en;
reg                                     ctrl_vl_a_en;
reg                                     ctrl_vbase_a_en;
reg                                     ctrl_vinc_a_en;
reg                                     ctrl_vstride_a_en;
reg                                     ctrl_vs_a_en;

wire          [ LOG2NUMVCREGS-1 : 0 ]   vc_rd_reg;
wire          [ LOG2NUMVCREGS-1 : 0 ]   vbase_rd_reg;
wire          [ LOG2NUMVCREGS-1 : 0 ]   vinc_rd_reg;
wire          [ LOG2NUMVCREGS-1 : 0 ]   vstride_rd_reg;
wire          [ LOG2NUMVSREGS-1 : 0 ]   vs_rd_reg;

wire          [ LOG2NUMVSREGS-1 : 0 ]   vlanes_vs_dst;
wire                                    vlanes_vs_we;
wire                        [ 5 : 2 ]   vlanes_vs_wetrack;
wire                [ VSWIDTH-1 : 0 ]   vlanes_vs_writedata;

wire                [ VCWIDTH-1 : 0 ]   vbase_plus_vinc;

wire [LOG2NUMVCREGS-1:0] ir_base;
wire [LOG2NUMVCREGS-1:0] ir_inc;
wire [LOG2NUMVCREGS-1:0] ir_stride;
wire [LOG2NUMVCREGS-1:0] vcdest_s1;
wire [LOG2NUMVCREGS-1:0] vcdest_s2;
wire [LOG2NUMVCREGS-1:0] vsdest_s1;
wire [LOG2NUMVCREGS-1:0] vsdest_s2;
wire       wevalid;
wire       wevalid_s2;
wire       haz_vc;
wire       haz_vl;
wire       haz_vbase;
wire       haz_vinc;
wire       haz_vstride;
wire       haz_vs_RAW;
wire       haz_vs_WAW;
wire dma_busy;
reg cfc_satisfied;

reg ctrl__vc_writedatain_sel;          //1 - vmstc, 0 - mtc;
reg [1:0] ctrl_vc_writedatain_sel;  //0 - vmstc/mtc, 1-vhalf, 2/3-vsatvl
reg ctrl_vc_we;
reg ctrl_vbase_writedatain_sel;    //1 - ctc/mstc, 0 - vld/vst
reg ctrl_scalarin_en;   //1 when ctc2/mtc2
reg ctrl_scalar_out_en;  //1 when cfc2
reg [1:0] ctrl_vcdest_sel;  //0 - ctc/mstc, 1- ld/st, 2 - satvl/half
reg ctrl_vsdest_sel;  //1 - mtc2, 0 - vmcts
reg ctrl_vs_we;
reg ctrl_rdctl_sel;        // 1 - cfc2/vmcts  0 - ld/st,
reg [1:0] ctrl_rdvc_sel;   // 2-vshamt, 3-vindex, other-ir_vcr


wire ctrl_scalarin_en_s2;
wire ctrl_scalar_out_en_s2;
wire      scalar_out_en_s3;
wire ctrl_vc_we_s2;
wire ctrl_vbase_writedatain_sel_s2;
wire ctrl__vc_writedatain_sel_s2;
wire [1:0] ctrl_vc_writedatain_sel_s2;
wire ctrl_vsdest_sel_s2;
wire ctrl_vs_we_s2;

/************************ Instruction Register ******************************/
wire is_cop2;
reg is_cop2_s1;

  assign is_cop2=instr[31:26]==6'b010010;

  always @(posedge clk)
    if (!resetn || (~is_cop2&~stall1) || (~instr_en&~stall1))
      ir<=32'h0;    // NOP  (Used to use VMSTC $vc48,$vs0)
    else if (instr_en&~stall1)
      ir<=instr;

  always @(posedge clk)
    if (!resetn)
      is_cop2_s1<=1'b0;
    else if (instr_en)
      is_cop2_s1<=is_cop2;

  assign instr_wait = stall1 & is_cop2;

/******************************************************************************/
/************************** 1st Pipeline Stage ********************************/
/******************************************************************************/

//Flag instructions which don't use lanes so they don't stall when lanes stalled
  reg ctrl_doesnt_use_lanes;
  always@*
  begin
    ctrl_doesnt_use_lanes=0;
    casez(ir_op)
      0,
      COP2_VSATVL,
      //COP2_VMCTS:  //Omit since vlanes modifies scalar
      COP2_VMSTC,
      COP2_CFC2,
      COP2_CTC2:
        ctrl_doesnt_use_lanes=1;
    endcase
  end

  assign internal_stall[1]=internal_stall[2] | haz_vs_RAW;
  assign stall1=internal_stall[1] | (vlanes_stalled[1]&&~ctrl_doesnt_use_lanes);
  assign squash1 = (stall1&~stall2)|~resetn;

  pipereg IR_reg2(ir,clk,resetn,~stall1,~squash1,ir2);

  assign ir_cop2=ir[31:26];
  assign ir_op={ir[25:22],ir[5:0]}; //10 bits
  assign ir_vcr=ir[15:10];
  assign ir_vsr=ir[15:11];
  assign ir_src1=ir[15:11];
  assign ir_src2=ir[20:16];
  assign ir_base={2'b10,ir[15:12]};
  assign ir_inc={3'b110,ir[11:9]};
  assign ir_stride={3'b111,ir[8:6]};

  assign wevalid = (ir_cop2==6'b010010) & ctrl_vc_we & (vcdest_s1!=48);

  pipereg #(1) wevalid_reg1(wevalid,clk,~squash1, ~stall1,1'b1, wevalid_s2);

  assign vcdest_s1 = (ctrl_vcdest_sel==0) ? ir_vcr :
                    (ctrl_vcdest_sel==1) ? ir_base : 0;

  pipereg #(LOG2NUMVCREGS) vcdest_reg1(vcdest_s1,clk,~squash1, ~stall1,1'b1,
                      vcdest_s2);

  assign vsdest_s1 = (ctrl_vsdest_sel) ? ir_vsr : ir_src2;

  pipereg #(LOG2NUMVSREGS) vsdest_reg1(vsdest_s1,clk,~squash1, ~stall1,1'b1,
                      vsdest_s2);

  pipereg #(LOG2NUMVSREGS) vsdestsel_reg1
        (ctrl_vsdest_sel,clk,~squash1, ~stall1,1'b1, ctrl_vsdest_sel_s2);

  // Before reading all source operands, we need to:
  //  1. Determine what all the sources are
  always@*
  begin
    ctrl_vc_a_en=0;
    ctrl_vl_a_en=0;
    ctrl_vbase_a_en=0;
    ctrl_vinc_a_en=0;
    ctrl_vstride_a_en=0;
    ctrl_vs_a_en=0;
    casez(ir_op)
      COP2_VADD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VADD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VSUB:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSUB_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VMULHI:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMULHI_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VDIV:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VDIV_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      //COP2_VMOD:
      COP2_VBFADD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VMOD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_EQ:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_NE:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_LT:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_U_LT:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_LE:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VCMP_U_LE:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VMIN:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMIN_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMAX:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMAX_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VMULLO:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VABS:
          ctrl_vl_a_en=1;
      COP2_VAND:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VNOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VSLL:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSRL:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSRA:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSAT_B:
          ctrl_vl_a_en=1;
      COP2_VSAT_H:
          ctrl_vl_a_en=1;
      //COP2_VSAT_W:
      COP2_VACT:
          ctrl_vl_a_en=1;
      COP2_VSAT_SU_B:
          ctrl_vl_a_en=1;
      COP2_VSAT_SU_H:
          ctrl_vl_a_en=1;
      //COP2_VSAT_SU_W:
      COP2_VRED:
          ctrl_vl_a_en=1;
      COP2_VSAT_SU_L:
          ctrl_vl_a_en=1;
      COP2_VSAT_U_B:
          ctrl_vl_a_en=1;
      COP2_VSAT_U_H:
          ctrl_vl_a_en=1;
      //COP2_VSAT_U_W:
      COP2_VTRP:
          ctrl_vl_a_en=1;
      COP2_VSADD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VSADD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VSSUB:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSSUB_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VSRR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VSRR_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VSLS:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VSLS_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VXUMUL:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMUL_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMUL:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMUL_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMADD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMADD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMSUB:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXUMSUB_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMADD:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMADD_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMSUB:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VXLMSUB_U:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VINS_VV:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VINS_SV:
      begin
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VEXT_VV:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VEXT_SV:
          ctrl_vc_a_en=1;
      COP2_VEXT_U_SV:
          ctrl_vc_a_en=1;
      COP2_VCOMPRESS:
          ctrl_vl_a_en=1;
      COP2_VEXPAND:
          ctrl_vl_a_en=1;
      COP2_VMERGE:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1]|ir_op[BIT_VSSRC2];
        end
      COP2_VFINS:
        begin
          ctrl_vc_a_en=1;
          ctrl_vs_a_en=1;
        end
      COP2_VEXTHALF:
          ctrl_vl_a_en=1;
      COP2_VHALF:
          ctrl_vl_a_en=1;
      COP2_VHALFUP:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VHALFDN:
        begin
          ctrl_vl_a_en=1;
          ctrl_vc_a_en=1;
        end
      COP2_VSATVL:
          ctrl_vl_a_en=1;
      COP2_VFAND:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VFOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VFXOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VFNOR:
        begin
          ctrl_vl_a_en=1;
          ctrl_vs_a_en=ir_op[BIT_VSSRC1];
        end
      COP2_VFCLR:
          ctrl_vl_a_en=1;
      COP2_VFSET:
          ctrl_vl_a_en=1;
      COP2_VIOTA:
          ctrl_vl_a_en=1;
      COP2_VCIOTA:
          ctrl_vl_a_en=1;
      COP2_VFPOP:
          ctrl_vl_a_en=1;
      COP2_VFFF1:
          ctrl_vl_a_en=1;
      COP2_VFFL1:
          ctrl_vl_a_en=1;
      COP2_VFSETBF:
          ctrl_vl_a_en=1;
      COP2_VFSETIF:
          ctrl_vl_a_en=1;
      COP2_VFSETOF:
          ctrl_vl_a_en=1;
      COP2_VFMT8:
          ctrl_vl_a_en=1;
      COP2_VFMF8:
          ctrl_vl_a_en=1;
      COP2_VFCLR8:
          ctrl_vl_a_en=1;
      COP2_VFOR8:
          ctrl_vl_a_en=1;
      COP2_VFLD:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
     // COP2_VLD_W:
     //   begin
     //     ctrl_vbase_a_en=1;
     //     ctrl_vinc_a_en=1;
     //   end
      COP2_VLD_L:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_U_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLD_U_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      //COP2_VLD_U_W:
      COP2_VAXIRD:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VLDS_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VLDS_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      //COP2_VLDS_W:
      //  begin
      //    ctrl_vbase_a_en=1;
      //    ctrl_vinc_a_en=1;
      //    ctrl_vstride_a_en=1;
      //  end
      COP2_VLDS_L:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VLDS_U_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VLDS_U_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      //COP2_VLDS_U_W:
      //  begin
      //    ctrl_vbase_a_en=1;
      //    ctrl_vinc_a_en=1;
      //    ctrl_vstride_a_en=1;
      //  end
      COP2_VLDX_B:
          ctrl_vbase_a_en=1;
      COP2_VLDX_H:
          ctrl_vbase_a_en=1;
      //COP2_VLDX_W:
      //    ctrl_vbase_a_en=1;
      COP2_VLDX_L:
          ctrl_vbase_a_en=1;
      COP2_VLDX_U_B:
          ctrl_vbase_a_en=1;
      COP2_VLDX_U_H:
          ctrl_vbase_a_en=1;
      //COP2_VLDX_U_W:
      //    ctrl_vbase_a_en=1;
      COP2_VFST:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VST_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VST_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      //COP2_VST_W:
      //  begin
      //    ctrl_vbase_a_en=1;
      //    ctrl_vinc_a_en=1;
      //  end
      COP2_VST_L:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
      COP2_VSTS_B:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VSTS_H:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      //COP2_VSTS_W:
      //  begin
      //    ctrl_vbase_a_en=1;
      //    ctrl_vinc_a_en=1;
      //    ctrl_vstride_a_en=1;
      //  end
      COP2_VSTS_L:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VSTX_B:
          ctrl_vbase_a_en=1;
      COP2_VSTX_H:
          ctrl_vbase_a_en=1;
     //COP2_VSTX_W:
      COP2_VPER_STR:
          ctrl_vl_a_en=1;
      COP2_VPER:
          ctrl_vl_a_en=1;
      COP2_VPER_LD:
          ctrl_vl_a_en=1;
      COP2_VSTX_L:
          ctrl_vbase_a_en=1;
      COP2_VSTXO_B:
          ctrl_vbase_a_en=1;
      COP2_VSTXO_H:
          ctrl_vbase_a_en=1;
      //COP2_VSTXO_W:
      COP2_VAXIWR:
          ctrl_vbase_a_en=1;
      COP2_VSTXO_L:
          ctrl_vbase_a_en=1;
      COP2_VMCTS:
        begin
          ctrl_vc_a_en=1;
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VMSTC:
          ctrl_vs_a_en=1;
      COP2_CFC2:
        begin
          ctrl_vc_a_en=1;
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
    endcase
  end

  //  2. Determine if any of the sources have a RAW hazard
  assign haz_vc=(ctrl_vc_a_en&wevalid_s2)&&(vc_rd_reg==vcdest_s2);
  assign haz_vl=(ctrl_vl_a_en&wevalid_s2)&&(vcdest_s2==0);
  assign haz_vbase=(ctrl_vbase_a_en&wevalid_s2)&&(vbase_rd_reg==vcdest_s2);
  assign haz_vinc=(ctrl_vinc_a_en&wevalid_s2)&&(vinc_rd_reg==vcdest_s2)&&(vcdest_s2!=48);
  assign haz_vstride=(ctrl_vstride_a_en&wevalid_s2)&(vstride_rd_reg==vcdest_s2);

  // Coarse-grained checks for VS - don't check registers
  assign haz_vs_RAW=ctrl_vs_a_en && (ctrl_vs_we_s2 || (|vlanes_vs_wetrack));
  assign haz_vs_WAW=ctrl_vs_we_s2 && (|vlanes_vs_wetrack[5:3]);

  pipereg #(VCWIDTH) fwddata_reg (
       (ctrl_vbase_writedatain_sel_s2) ?  vbase_plus_vinc : vc_c_writedatain,
       clk,~squash1, ~stall1,1'b1,fwddata);

  pipereg #(1) fwdvc_reg ( haz_vc,clk,~squash1, ~stall1,1'b1,fwd_vc);
  pipereg #(1) fwdvl_reg ( haz_vl,clk,~squash1, ~stall1,1'b1,fwd_vl);
  pipereg #(1) fwdvbase_reg ( haz_vbase,clk,~squash1, ~stall1,1'b1,fwd_vbase);
  pipereg #(1) fwdvinc_reg ( haz_vinc,clk,~squash1, ~stall1,1'b1,fwd_vinc);
  pipereg #(1) fwdvstride_reg(haz_vstride,clk,~squash1,~stall1,1,fwd_vstride);

  /************************ REGISTER FILES ******************************/

  assign vc_rd_reg= (ctrl_rdvc_sel==2) ? 2 :  //vshamt
                    (ctrl_rdvc_sel==3) ? 3 :  //vindex
                    ir_vcr;
  assign vbase_rd_reg= (ctrl_rdctl_sel) ?  ir_vcr : ir_base;
  assign vinc_rd_reg= (ctrl_rdctl_sel) ?  ir_vcr : ir_inc;
  assign vstride_rd_reg= (ctrl_rdctl_sel) ?  ir_vcr : ir_stride;
  assign vs_rd_reg= (ctrl_vs_a_en & ir_op[BIT_VSSRC1]) ? 
                        ir_src1[LOG2NUMVSREGS-1:0] : 
                        ir_src2[LOG2NUMVSREGS-1:0];

  assign vc_a_reg= vc_rd_reg[LOG2NUMNONMEMVCREGS-1:0];
  assign vbase_a_reg= vbase_rd_reg[LOG2NUMVBASEREGS-1:0];
  assign vinc_a_reg=vinc_rd_reg[LOG2NUMVINCREGS-1:0];
  assign vstride_a_reg=vstride_rd_reg[LOG2NUMVSTRIDEREGS-1:0];

  assign vs_a_reg=vs_rd_reg[LOG2NUMVSREGS-1:0];

  assign vc_a_en=ctrl_vc_a_en&~stall1;
  assign vbase_a_en=ctrl_vbase_a_en&~stall1;
  assign vinc_a_en=ctrl_vinc_a_en&~stall1;
  assign vstride_a_en=ctrl_vstride_a_en&~stall1;
  assign vs_a_en=ctrl_vs_a_en&~stall1;

  wire [VCWIDTH-1:0] temp;
  vregfile_control vregfile_control (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vc_a_reg), 
      .a_en(vc_a_en),
      .a_readdataout(vc_a_readdataout),
      .c_reg(vc_c_reg), 
      .c_writedatain(vc_c_writedatain), 
      .c_we(vc_c_we),
      .vl(vl),
      //The reserved registers vc31, vc30, vc29 are used
      //for the matmul's masks.
      .matmul_masks(matmul_masks),
      .dma_en(dma_en),
      .lane_addr(lane_addr),
      .mem_addr(mem_addr),
      .num_bytes(num_bytes),
      .dma_we(dma_we),
      .dma_busy(dma_busy),
      .temp(temp)
      );
    defparam vregfile_control.WIDTH=VCWIDTH;
    defparam vregfile_control.NUMREGS=NUMNONMEMVCREGS;
    defparam vregfile_control.LOG2NUMREGS=LOG2NUMNONMEMVCREGS;

  vregfile_base vregfile_base (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vbase_a_reg), 
      .a_en(vbase_a_en), 
      .a_readdataout(vbase_a_readdataout),
      .c_reg(vbase_c_reg), 
      .c_writedatain(vbase_c_writedatain), 
      .c_we(vbase_c_we));
    defparam vregfile_base.WIDTH=VCWIDTH;
    defparam vregfile_base.NUMREGS=NUMVBASEREGS;
    defparam vregfile_base.LOG2NUMREGS=LOG2NUMVBASEREGS;

  vregfile_inc vregfile_inc (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vinc_a_reg), 
      .a_en(vinc_a_en), 
      .a_readdataout(vinc_a_readdataout),
      .c_reg(vinc_c_reg), 
      .c_writedatain(vinc_c_writedatain), 
      .c_we(vinc_c_we));
    defparam vregfile_inc.WIDTH=VCWIDTH;
    defparam vregfile_inc.NUMREGS=NUMVINCREGS;
    defparam vregfile_inc.LOG2NUMREGS=LOG2NUMVINCREGS;

  vregfile_stride vregfile_stride (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vstride_a_reg), 
      .a_en(vstride_a_en), 
      .a_readdataout(vstride_a_readdataout),
      .c_reg(vstride_c_reg), 
      .c_writedatain(vstride_c_writedatain), 
      .c_we(vstride_c_we));
    defparam vregfile_stride.WIDTH=VCWIDTH;
    defparam vregfile_stride.NUMREGS=NUMVSTRIDEREGS;
    defparam vregfile_stride.LOG2NUMREGS=LOG2NUMVSTRIDEREGS;

  vregfile_scalar vregfile_scalar (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vs_a_reg), 
      .a_en(vs_a_en), 
      .a_readdataout(vs_a_readdataout),
      .c_reg(vs_c_reg), 
      .c_writedatain(vs_c_writedatain), 
      .c_we(vs_c_we));
    defparam vregfile_scalar.WIDTH=VSWIDTH;
    defparam vregfile_scalar.NUMREGS=NUMVSREGS;
    defparam vregfile_scalar.LOG2NUMREGS=LOG2NUMVSREGS;

  pipereg #(6) vc_reg(ir_vcr,clk,resetn,vc_a_en,1'b1,ir_vcr_r);

  pipereg #(1) r1(ctrl_scalarin_en,clk,~squash1,~stall1,1'b1,ctrl_scalarin_en_s2);
  pipereg #(1)r2(ctrl_scalar_out_en,clk,~squash1,~stall1,1'b1,ctrl_scalar_out_en_s2);
  pipereg #(1) r4(ctrl_vc_we,clk,~squash1,~stall1,1'b1,ctrl_vc_we_s2);
  pipereg #(1) r5(ctrl_vbase_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl_vbase_writedatain_sel_s2);
  pipereg #(1) r6(ctrl__vc_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl__vc_writedatain_sel_s2);
  pipereg #(2) r7(ctrl_vc_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl_vc_writedatain_sel_s2);
  pipereg #(1) r8(ctrl_vs_we,clk,~squash1,~stall1,1'b1,ctrl_vs_we_s2);


  /*********************** Control control signals ****************************/
  always@*
  begin
    ctrl_scalarin_en=0;
    ctrl_scalar_out_en=0;
    ctrl__vc_writedatain_sel=0;
    ctrl_vc_writedatain_sel=0;
    ctrl_vbase_writedatain_sel=0;
    ctrl_vc_we=0;
    ctrl_rdctl_sel=0;
    ctrl_rdvc_sel=0;
    ctrl_vcdest_sel=0;
    ctrl_vsdest_sel=0;
    ctrl_vs_we=0;
    casez(ir_op)
      COP2_VSRR:  ctrl_rdvc_sel=2;
      COP2_VSRR_U:  ctrl_rdvc_sel=2;
      COP2_VSLS:  ctrl_rdvc_sel=2;
      COP2_VSLS_U:  ctrl_rdvc_sel=2;
      COP2_VXUMUL:  ctrl_rdvc_sel=2;
      COP2_VXUMUL_U:  ctrl_rdvc_sel=2;
      COP2_VXLMUL:  ctrl_rdvc_sel=2;
      COP2_VXLMUL_U:  ctrl_rdvc_sel=2;
      COP2_VXUMADD:  ctrl_rdvc_sel=2;
      COP2_VXUMADD_U:  ctrl_rdvc_sel=2;
      COP2_VXUMSUB:  ctrl_rdvc_sel=2;
      COP2_VXUMSUB_U:  ctrl_rdvc_sel=2;
      COP2_VXLMADD:  ctrl_rdvc_sel=2;
      COP2_VXLMADD_U:  ctrl_rdvc_sel=2;
      COP2_VXLMSUB:  ctrl_rdvc_sel=2;
      COP2_VXLMSUB_U:  ctrl_rdvc_sel=2;
      COP2_VINS_VV:  ctrl_rdvc_sel=2;
      COP2_VINS_VV:  ctrl_rdvc_sel=3;
      COP2_VINS_SV:  ctrl_rdvc_sel=3;
      COP2_VEXT_VV:  ctrl_rdvc_sel=3;
      COP2_VEXT_SV:  ctrl_rdvc_sel=3;
      COP2_VEXT_U_SV:  ctrl_rdvc_sel=3;
      COP2_VFINS:  ctrl_rdvc_sel=3;
      COP2_VHALFUP:  ctrl_rdvc_sel=3;
      COP2_VHALFDN:  ctrl_rdvc_sel=3;
      COP2_VHALF:
      begin
          ctrl_vc_writedatain_sel=1;
          ctrl_vcdest_sel=2;
          ctrl_vc_we=1;
      end
      COP2_VSATVL:
      begin
          ctrl_vc_writedatain_sel=2;
          ctrl_vcdest_sel=2;
          ctrl_vc_we=1;
      end
      COP2_VFLD,
      COP2_VLD_B,
      COP2_VLD_H,
     // COP2_VLD_W,
      COP2_VLD_L,
      COP2_VLD_U_B,
      COP2_VLD_U_H,
     // COP2_VLD_U_W,
      COP2_VAXIRD,
      COP2_VLDS_B,
      COP2_VLDS_H,
    //  COP2_VLDS_W,
      COP2_VLDS_L,
      COP2_VLDS_U_B,
      COP2_VLDS_U_H,
    //  COP2_VLDS_U_W,
      COP2_VFST,
      COP2_VST_B,
      COP2_VST_H,
    //  COP2_VST_W,
      COP2_VST_L,
      COP2_VSTS_B,
      COP2_VSTS_H,
    //  COP2_VSTS_W,
      COP2_VSTS_L:
      begin
          ctrl_vbase_writedatain_sel=1;
          ctrl_vcdest_sel=1;
          ctrl_vc_we=1;
      end
      COP2_VMCTS:
      begin
        ctrl_rdctl_sel=1;
        ctrl_vsdest_sel=0;
        ctrl_vs_we=1;
      end
      COP2_VMSTC:
      begin
        ctrl__vc_writedatain_sel=1;
        ctrl_vcdest_sel=0;
        ctrl_vc_we=1;
      end
      COP2_CFC2:
      begin
        ctrl_scalar_out_en=1;
        ctrl_rdctl_sel=1;
      end
      COP2_CTC2:
      begin
        ctrl_scalarin_en=1;
        ctrl_vcdest_sel=0;
        ctrl_vc_we=1;
      end
      COP2_MTC2:
      begin
        ctrl_scalarin_en=1;
        ctrl_vsdest_sel=1;
        ctrl_vs_we=1;
      end
    endcase
  end

/******************************************************************************/
/************************** 2nd Pipeline Stage ********************************/
/******************************************************************************/

  assign internal_stall[2]=internal_stall[3] |
                              (ctrl_scalarin_en_s2&~scalar_in_en) | 
                              haz_vs_WAW;
  assign stall2=internal_stall[2]; // | vlanes_stalled[2];
  assign squash2 = ((stall2&~stall3))|~resetn;

  // Stall scalar if a) we are stalled, are expecting a scalar, and the it is
  // available OR b) if we haven't gotten there yet.
  assign scalar_in_wait=(scalar_in_en&~ctrl_scalarin_en_s2) || 
                        (ctrl_scalarin_en_s2&scalar_in_en&stall2);

  assign vc_readdataout= (fwd_vc) ? fwddata : vc_a_readdataout;
  assign vl_readdataout= (fwd_vl) ? fwddata : vl;
  assign vbase_readdataout= (fwd_vbase) ? fwddata : vbase_a_readdataout;
  assign vinc_readdataout= (fwd_vinc) ? fwddata : vinc_a_readdataout;
  assign vstride_readdataout= (fwd_vstride) ? fwddata : vstride_a_readdataout;
  assign vs_readdataout= vs_a_readdataout;

  assign vc_combined_out=(!ir_vcr_r[5]) ? vc_readdataout :
                         (!ir_vcr_r[4]) ? vbase_readdataout :
                         (!ir_vcr_r[3]) ? vinc_readdataout : 
                         vstride_readdataout;

  pipereg #(VCWIDTH) scalar_out_reg(
      vc_combined_out,clk,resetn,ctrl_scalar_out_en_s2,1'b1,vc_combined_out_s3);

  pipereg #(1) scalar_out_en_reg(
      ctrl_scalar_out_en_s2,clk,~squash2,~stall2,1'b1,scalar_out_en_s3);


  /************** Datapath - Control Regs *******************/

  assign vc_c_we = ctrl_vc_we_s2&~vcdest_s2[5] & ~stall2;
  assign vc_c_reg =  vcdest_s2[LOG2NUMNONMEMVCREGS-1:0]; 
  //temporary to feed to base,inc, and stride regfiles (without vl logic)
  assign _vc_c_writedatain= (ctrl__vc_writedatain_sel_s2) ? vs_a_readdataout :
                                                        scalar_in;
  assign vc_c_writedatain= ( (ctrl_vc_writedatain_sel_s2==0) ? _vc_c_writedatain :
                            //vhalf instruction
                            (ctrl_vc_writedatain_sel_s2==1) ? vl_readdataout>>1:
                            //vsatvl instruction
                            //(vl_readdataout>vc_readdataout) ? vc_readdataout :
                            (vl_readdataout>MVL) ? MVL : 
                            vl_readdataout);

  assign vbase_c_we = ctrl_vc_we_s2 & (vcdest_s2[5:4]==2) & ~stall2;
  assign vbase_c_reg = vcdest_s2[LOG2NUMVBASEREGS-1:0];
  assign vbase_c_writedatain = (ctrl_vbase_writedatain_sel_s2) ?
                                vbase_plus_vinc :
                                _vc_c_writedatain;

  assign vbase_plus_vinc=vbase_readdataout+vinc_readdataout;

  assign vinc_c_we = ctrl_vc_we_s2 & (vcdest_s2[5:3]==6) & ~stall2;
  assign vinc_c_reg = vcdest_s2[LOG2NUMVINCREGS-1:0];
  assign vinc_c_writedatain = _vc_c_writedatain;

  assign vstride_c_we = ctrl_vc_we_s2 & (vcdest_s2[5:3]==7) & ~stall2;
  assign vstride_c_reg = vcdest_s2[LOG2NUMVSTRIDEREGS-1:0];
  assign vstride_c_writedatain = _vc_c_writedatain;

  //FIXME - OR vector writes in, but check for RAW & WAW hazards
  assign vs_c_we = vlanes_vs_we || (ctrl_vs_we_s2 & ~stall2);
  assign vs_c_reg = (vlanes_vs_we) ? vlanes_vs_dst : vsdest_s2;
  assign vs_c_writedatain = (vlanes_vs_we) ? vlanes_vs_writedata : 
                    (ctrl_vsdest_sel_s2) ? scalar_in : vc_combined_out;


/******************************************************************************/
/************************** 3rd Pipeline Stage ********************************/
/******************************************************************************/

  assign internal_stall[3]=scalar_out_en_s3&scalar_out_wait;
  assign stall3=internal_stall[3]; // | vlanes_stalled[3];

  assign scalar_out=vc_combined_out_s3;
  assign scalar_out_en=scalar_out_en_s3&~cfc_satisfied;

  always@(posedge clk)
    cfc_satisfied<=scalar_out_en_s3 & ~scalar_out_wait;

/******************************************************************************/
/************************ Instantiate Vector Lanes ****************************/
/******************************************************************************/

  vlanes vlanes(
    .clk(clk),
    .resetn(resetn),

    // Instruction interface
    .instr(ir),
    .instr_en(is_cop2_s1),    // tells when instr is valid and available
    .instr_wait(),            // if high says vpu is not ready to receive

    .stall_in({internal_stall,1'b0}),
    .is_stalled(vlanes_stalled),
    .has_memop(has_memop),

    // Control register values - 2nd stage
    .vc_in(vc_readdataout),
    .vl_in(vl_readdataout),
    .vbase_in(vbase_readdataout),
    .vinc_in(vinc_readdataout),
    .vstride_in(vstride_readdataout),
    .vs_in(vs_readdataout),
    .matmul_masks_in(matmul_masks),
    .dma_en(dma_en),
    .mem_addr(mem_addr),
    .num_bytes(num_bytes),
    .lane_addr(lane_addr),
    .dma_we(dma_we),
    .dma_busy(dma_busy),
    // vs Writeback
    .vs_dst(vlanes_vs_dst),
    .vs_wetrack(vlanes_vs_wetrack),  //1-bit for each pipe-stage
    .vs_we(vlanes_vs_we),
    .vs_writedata(vlanes_vs_writedata),
    
    // AXI interface
    .M_AWID    (M_AWID    ),
    .M_AWADDR  (M_AWADDR  ),
    .M_AWLEN   (M_AWLEN   ),
    .M_AWSIZE  (M_AWSIZE  ),
    .M_AWBURST (M_AWBURST ),
    .M_AWLOCK  (M_AWLOCK  ),
    .M_AWCACHE (M_AWCACHE ),
    .M_AWPROT  (M_AWPROT  ),
    .M_AWQOS   (M_AWQOS   ),
    .M_AWVALID (M_AWVALID ),
    .M_AWREADY (M_AWREADY ),
    .M_WDATA   (M_WDATA   ),
    .M_WSTRB   (M_WSTRB   ),
    .M_WLAST   (M_WLAST   ),
    .M_WVALID  (M_WVALID  ),
    .M_WREADY  (M_WREADY  ),
    .M_ARID    (M_ARID    ),
    .M_ARADDR  (M_ARADDR  ),
    .M_ARLEN   (M_ARLEN   ),
    .M_ARSIZE  (M_ARSIZE  ),
    .M_ARBURST (M_ARBURST ),
    .M_ARLOCK  (M_ARLOCK  ),
    .M_ARCACHE (M_ARCACHE ),
    .M_ARPROT  (M_ARPROT  ),
    .M_ARQOS   (M_ARQOS   ),
    .M_ARVALID (M_ARVALID ),
    .M_ARREADY (M_ARREADY ),
    .M_RID     (M_RID     ),
    .M_RDATA   (M_RDATA   ),
    .M_RRESP   (M_RRESP   ),
    .M_RLAST   (M_RLAST   ),
    .M_RVALID  (M_RVALID  ),
    .M_RREADY  (M_RREADY  ),
    .M_BREADY  (M_BREADY  ),
    .M_BVALID  (M_BVALID  ),
    .M_BRESP   (M_BRESP   ),
    .M_BID     (M_BID     ),
    .S_AWID    (S_AWID   ), 
    .S_AWADDR  (S_AWADDR ), 
    .S_AWLEN   (S_AWLEN  ), 
    .S_AWSIZE  (S_AWSIZE ), 
    .S_AWBURST (S_AWBURST), 
    .S_AWLOCK  (S_AWLOCK ), 
    .S_AWCACHE (S_AWCACHE), 
    .S_AWPROT  (S_AWPROT ), 
    .S_AWQOS   (S_AWQOS  ), 
    .S_AWVALID (S_AWVALID), 
    .S_AWREADY (S_AWREADY), 
    .S_WDATA   (S_WDATA  ), 
    .S_WSTRB   (S_WSTRB  ), 
    .S_WLAST   (S_WLAST  ), 
    .S_WVALID  (S_WVALID ), 
    .S_WREADY  (S_WREADY ), 
    .S_ARID    (S_ARID   ), 
    .S_ARADDR  (S_ARADDR ), 
    .S_ARLEN   (S_ARLEN  ), 
    .S_ARSIZE  (S_ARSIZE ), 
    .S_ARBURST (S_ARBURST), 
    .S_ARLOCK  (S_ARLOCK ), 
    .S_ARCACHE (S_ARCACHE), 
    .S_ARPROT  (S_ARPROT ), 
    .S_ARQOS   (S_ARQOS  ), 
    .S_ARVALID (S_ARVALID), 
    .S_ARREADY (S_ARREADY), 
    .S_RID     (S_RID    ), 
    .S_RDATA   (S_RDATA  ), 
    .S_RRESP   (S_RRESP  ), 
    .S_RLAST   (S_RLAST  ), 
    .S_RVALID  (S_RVALID ), 
    .S_RREADY  (S_RREADY ), 
    .S_BREADY  (S_BREADY ), 
    .S_BVALID  (S_BVALID ), 
    .S_BRESP   (S_BRESP  ), 
    .S_BID     (S_BID    ), 

    // Data memory interface
    .dbus_address(dbus_address),
    .dbus_en(dbus_en),
    .dbus_we(dbus_we),
    .dbus_byteen(dbus_byteen),
    .dbus_writedata(dbus_writedata),
    .dbus_readdata(dbus_readdata),
    .dbus_cachematch(dbus_cachematch),
    .dbus_cachemiss(dbus_cachemiss),
    .dbus_prefetch(dbus_prefetch),
    .dbus_wait(dbus_wait),

    //DMA interface
    .dma_dbus_address   (dma_dbus_address), 
    .dma_dbus_readdata  (dma_dbus_readdata), 
    .dma_dbus_writedata (dma_dbus_writedata),
    .dma_dbus_byteen    (dma_dbus_byteen),
    .dma_dbus_en    (dma_dbus_en),       
    .dma_dbus_wren  (dma_dbus_wren),     
    .dma_dbus_prefetch  (dma_dbus_prefetch), 
    .dma_dbus_wait  (dma_dbus_wait),     
    .dma_dbus_data_valid(dma_dbus_data_valid)
    );
  defparam 
    vlanes.NUMLANES=NUMLANES,
    vlanes.LOG2NUMLANES=LOG2NUMLANES,
    vlanes.NUMMEMPARALLELLANES=NUMMEMLANES,
    vlanes.LOG2NUMMEMPARALLELLANES=LOG2NUMMEMLANES,
    vlanes.NUMMULLANES=NUMMULLANES,
    vlanes.MVL=MVL,
    vlanes.LOG2MVL=LOG2MVL,
    vlanes.VPW=VPW,
    vlanes.LOG2VPW=LOG2VPW,
    vlanes.LANEWIDTH=LANEWIDTH,
    vlanes.LOG2LANEWIDTH=LOG2LANEWIDTH,
    vlanes.NUMBANKS=NUMBANKS,
    vlanes.LOG2NUMBANKS=LOG2NUMBANKS,
    vlanes.ALUPERBANK=ALUPERBANK,
    vlanes.DMEM_WRITEWIDTH=DMEM_WRITEWIDTH, 
    vlanes.LOG2DMEM_WRITEWIDTH=LOG2DMEM_WRITEWIDTH,
    vlanes.DMEM_READWIDTH=DMEM_READWIDTH,
    vlanes.LOG2DMEM_READWIDTH=LOG2DMEM_READWIDTH,
    vlanes.VCWIDTH=VCWIDTH,
    vlanes.VSWIDTH=VSWIDTH,
    vlanes.NUMVSREGS=NUMVSREGS,
    vlanes.LOG2NUMVSREGS=LOG2NUMVSREGS;

endmodule

/*
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/local/spram1.v
//////////////////////////////////////////////////////////////////////
module spram1 (
    clk,
    address,
    wren, 
    data,
    byteen,
    out
);

parameter AWIDTH=10;
parameter NUM_WORDS=1024;
parameter DWIDTH=32;
parameter LOG2DWIDTH = $clog2(DWIDTH);
input clk;
input [(AWIDTH-1):0] address;
input  wren;
input [(DWIDTH/8)-1:0] byteen;
input [(DWIDTH-1):0] data;
output reg [(DWIDTH-1):0] out;

//`ifdef SIMULATION_MEMORY

integer i;
integer k;

reg [32-1:0] ram[67108864-1:0];
//reg [32-1:0] ram[4096-1:0];
reg [25:0] addr;
 
initial
 begin
   //This is TOO big for 256 MB RAM!  We right shift data by 1
       $readmemh("instr.dat",ram,'h100_0000);
       $readmemh("data.dat",ram,'h400_0000>>1);
 end

always@(*) begin
    addr = address << 26-AWIDTH;
end

always@(posedge clk) begin 
  if (wren) begin
      for(k=0; k < DWIDTH/32;k=k+1)begin
          for(i=0; i < 4 ;i=i+1)begin
              if(byteen[((DWIDTH/8-1)-(4*k+i))])
                  ram[addr+k][i*8+:8] <= data[32*k+i*8+:8];
          end
      end
  end
  else begin
      for(i=0; i < DWIDTH/32; i=i+1)begin
          out[32*i+:32] <= ram[addr+i];
      end
  end
end
//`else

/*
//single_port_ram u_single_port_ram(
//.addr(address),
//.we(wren),
//.data(data),
//.out(out),
//.clk(clk)
//);
//`endif

endmodule
*/
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/local/rams.v
//////////////////////////////////////////////////////////////////////
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

`ifndef SIMULATION_MEMORY
 `define SIMULATION_MEMORY
`endif

`ifdef SIMULATION_MEMORY

reg [(((NUM_WORDS-1)-(0)+1)*((DWIDTH-1)-(0)+1))-1 : 0] ram;

always @ (posedge clk) begin 
  if (wren_a) begin
      ram[((address_a-0)*((DWIDTH-1)-(0)+1)+DWIDTH-1-0) : ((address_a-0)*((DWIDTH-1)-(0)+1))] <= data_a;
  end
  else begin
      out_a <= ram[((address_a-0)*((DWIDTH-1)-(0)+1)+DWIDTH-1-0) : ((address_a-0)*((DWIDTH-1)-(0)+1))];
  end
end
  
always @ (posedge clk) begin 
  if (wren_b) begin
      ram[((address_b-0)*((DWIDTH-1)-(0)+1)+DWIDTH-1-0) : ((address_b-0)*((DWIDTH-1)-(0)+1))] <= data_b;
  end 
  else begin
      out_b <= ram[((address_b-0)*((DWIDTH-1)-(0)+1)+DWIDTH-1-0) : ((address_b-0)*((DWIDTH-1)-(0)+1))];
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

module spram (
    clk,
	address,
	wren,
	data,
	out
);

parameter AWIDTH=10;
parameter NUM_WORDS=1024;
parameter DWIDTH=32;

input clk;
input [(AWIDTH-1):0] address;
input  wren;
input [(DWIDTH-1):0] data;
output reg [(DWIDTH-1):0] out;

`ifdef SIMULATION_MEMORY

reg [(((NUM_WORDS-1)-(0)+1)*((DWIDTH-1)-(0)+1))-1 : 0] ram;

always @ (posedge clk) begin 
  if (wren) begin
      ram[((address-0)*((DWIDTH-1)-(0)+1)+DWIDTH-1-0) : ((address-0)*((DWIDTH-1)-(0)+1))] <= data;
  end
  else begin
      out <= ram[((address-0)*((DWIDTH-1)-(0)+1)+DWIDTH-1-0) : ((address-0)*((DWIDTH-1)-(0)+1))];
  end
end
  
`else

single_port_ram u_single_port_ram(
.addr(address),
.we(wren),
.data(data),
.out(out),
.clk(clk)
);

`endif

endmodule//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/local/local_add_sub.v
//////////////////////////////////////////////////////////////////////
module local_add_sub(
dataa,
datab,
cin,
add_sub,
result
);

parameter WIDTH = 32;
parameter PIPELINE = 0;
parameter REPRESENTATION = "SIGNED";

input[WIDTH-1:0] dataa;
input[WIDTH-1:0] datab;
input cin;
input add_sub;
output reg [WIDTH-1:0] result;

always @(*)begin
    if(add_sub == 1'b1)
         result = dataa+datab+cin;
    else
         result = dataa - datab;
end

endmodule//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/local/local_mult.v
//////////////////////////////////////////////////////////////////////
module local_mult(
dataa,
datab,
clock,
clken,
aclr,
result
);

parameter LPM_WIDTHA = 32;
parameter LPM_WIDTHB = 32;
parameter LPM_WIDTHP = 64;
parameter LPM_REPRESENTATION = "SIGNED";

input [LPM_WIDTHA-1:0] dataa;
input [LPM_WIDTHB-1:0] datab;
input clock;
input clken;
input aclr;
output reg [LPM_WIDTHP-1:0] result;

wire signed [LPM_WIDTHA-1:0] signedinputA;
wire signed [LPM_WIDTHB-1:0] signedinputB;
wire signed [LPM_WIDTHP-1:0] signedoutputP;

wire [LPM_WIDTHA-1:0] unsignedinputA;
wire [LPM_WIDTHB-1:0] unsignedinputB;
wire [LPM_WIDTHP-1:0] unsignedinputP;

wire gated_clock;

assign signedinputA = dataa;
assign signedinputB = datab;
assign unsignedinputA = dataa;
assign unsignedinputB = datab;

assign signedoutputP = signedinputA * signedinputB;
assign unsignedoutputP = unsignedinputA * unsignedinputB;

assign gated_clock = clock & clken;

always @(posedge gated_clock)begin
    if(aclr)begin
       result <= 0;
    end
    else if(LPM_REPRESENTATION == "SIGNED")
       result <= signedoutputP;
    else
       result <= unsignedoutputP; 
end

endmodule//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/local/local_fifo.v
//////////////////////////////////////////////////////////////////////
module fifo#(
parameter FIFO_WIDTH = 8,
parameter FIFO_DEPTH = 4
         )(
input                      clk,
input                      reset,
input   [FIFO_WIDTH-1:0]   wrdata,
input                      write,

input                       read,
output reg [FIFO_WIDTH-1:0] rddata,
output reg                  full,
output reg                  empty 
);

localparam PTR_WIDTH = $clog2(FIFO_DEPTH);

reg [(((FIFO_DEPTH-1)-(0)+1)*((FIFO_WIDTH-1)-(0)+1))-1 : 0] mem;
reg  [PTR_WIDTH:0]    wrptr,rdptr;
wire [PTR_WIDTH:0]    wrptr_nxt,rdptr_nxt;
integer i;

assign  wrptr_nxt = write? wrptr + 1'b1: wrptr;
assign  rdptr_nxt = read? rdptr + 1'b1:rdptr ;

always@(*) begin
    rddata = mem[rdptr[PTR_WIDTH-1:0]];
end

always@(posedge clk) begin
  if(!reset)begin
    full <= 1'b0;
  end
  else begin
    if(wrptr_nxt[PTR_WIDTH]^ rdptr_nxt[PTR_WIDTH])
  	full <= ~(|(wrptr_nxt[PTR_WIDTH-1:0]^rdptr_nxt[PTR_WIDTH-1:0]));
    else
	full <= 1'b0;
  end 
end

always@(posedge clk) begin
  if(!reset)begin
    empty <= 1'b1;
  end
  else begin
    empty <= ~(|(wrptr_nxt^rdptr_nxt));
  end 
end

always @(posedge clk) begin
 if(!reset)begin
   wrptr <= 'h0;
 end
 else begin
     wrptr <= wrptr_nxt;
 end
end

always@(posedge clk) begin
 if(!reset)begin
   rdptr <= 'h0;
 end
 else begin
     rdptr <= rdptr_nxt;
 end
end

always@(posedge clk) begin
 if(!reset)begin
   for(i = 0; i<FIFO_DEPTH;i=i+1)begin
       mem[((i-0)*((FIFO_WIDTH-1)-(0)+1)+FIFO_WIDTH-1-0) : ((i-0)*((FIFO_WIDTH-1)-(0)+1))] <= 'h0;
   end
 end
 else begin
   if(write)begin
      mem[wrptr[PTR_WIDTH-1:0]]<= wrdata;
   end
 end
end

endmodule//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/local/local_shifter.v
//////////////////////////////////////////////////////////////////////
module local_shifter(
  data,
  distance,
  direction,
  result
);

parameter LPM_WIDTH = 32;
parameter LPM_WIDTHDIST = 5;
parameter LPM_SHIFTTYPE = "LOGICAL";

input [LPM_WIDTH-1:0] data;
input [LPM_WIDTHDIST-1:0] distance;
input direction;

output reg [LPM_WIDTH-1:0] result;
reg [LPM_WIDTH-1:0] arith_reg; 
always @* begin
  arith_reg = {LPM_WIDTH{1'b1}};
  case(LPM_SHIFTTYPE)
    "LOGICAL":begin 
                  if(direction == 1'b0)begin
                      result = data << distance;
                  end
                  else begin
                      result = data >> distance;
                  end
              end
    "ARITHMATIC":begin 
                   if(direction)begin
                      result = data << distance;
                   end
                   else begin
                      if(data[LPM_WIDTH-1] == 1'b1)
                         result =  ((arith_reg <<(LPM_WIDTH - distance))|| (data >> distance));
                      else
                         result = data >> distance;
                   end
                 end
    default:begin
                  if(direction == 1'b0)begin
                      result = data << distance;
                  end
                  else begin
                      result = data >> distance;
                  end
            end
  endcase
end
endmodule//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/top/options.v
//////////////////////////////////////////////////////////////////////
`ifndef _OPTIONS_V_
`define _OPTIONS_V_ 1

`define NO_PLI 1
//`define TEST_BENCH 1
`define USE_INHOUSE_LOGIC
//`define SIMULATION_MEMORY
// Replaces altera blocks with local logic files

/************************** ABBREVIEATED NAMES *****************************/
// Note: LG = Log base 2
//
// Default configuration:
//    8KB Icache (LGID=13) with 16 byte cache line size (LGIW=4)
//   32KB Dcache (LGDD=15) with 64 byte cache line size (LGDD=6)
//    Data prefetching off (DP=0, DPV=0)
//    16 Vector Lanes (LGL=4)
//    64 Maximum Vector Length (LGMVL=6)
//    32-bit (4-byte) Vector lane width (LGVPW=2)
//    32-bit Vector lane datapath width (LGLW=5)
//    16 Memory Crossbar lanes (LGM=LGL)
//    16 Multiplier lanes (LGX=LGL)
//    2 Register Banks (LGB=1)
//    Disable ALU per Bank (APB=0)


// INSTR CACHE
`define LGID 13
`define LGIW 4

// DATA CACHE
`define LGDD 15
`define LGDW 4

// DATA CACHE PREFETCHER - dcache needs to have 64 byte lines
`define LGDWB 7
`define DP 0
// VECTOR DATA CACHE PREFETCHER 0:off, 65535-N:N*veclength, N:pfch N cache lines
`define DPV 0

// VECTOR CORE
//Changing to 3. That is, we now have 8 lanes.
`define LGL 3
`define LGB 0
`define APB 0
`define LGM `LGL
`define LGX `LGL

// VECTOR ISA
`define LGMVL 6
`define LGVPW 1 //chaging the word size of vector processor to 16: support for bfloat16
`define LGLW 4

/****************************** FULL NAMES *********************************/

// INSTR CACHE
`define LOG2ICACHEDEPTHBYTES `LGID
`define LOG2ICACHEWIDTHBITS (`LGIW+3)

// DATA CACHE
`define LOG2DCACHEDEPTHBYTES `LGDD
`define LOG2DCACHEWIDTHBITS (`LGDW+3)

// DATA CACHE PREFETCHER - dcache needs to have 64 byte lines
`define LOG2DATAWBBUFFERSIZE `LGDWB
`define DEFAULTDCACHEPREFETCHES `DP
// VECTOR DATA CACHE PREFETCHER 0:off, 65535:vectorlength, N:pfch N cache lines
`define VECTORPREFETCHES `DPV

// VECTOR CORE
`define LOG2NUMLANES `LGL
`define LOG2MVL `LGMVL
`define LOG2VPW `LGVPW
`define LOG2LANEWIDTHBITS `LGLW
`define LOG2NUMMEMLANES `LGM
`define LOG2NUMMULLANES `LGX
`define LOG2NUMBANKS `LGB
`define ALUPERBANK `APB

`define MAX_STAGES 14
/****************************** OTHER PARAMS *********************************/

// DRAM
`define LOG2DRAMWIDTHBITS 7

/****************** NUM PIPELINE STAGES in VECTOR PROCESSOR ***************/
//mult consumes 3 cycles
//`define MAX_PIPE_STAGES 7
//matmul consumes 29 cycles
`define MAX_PIPE_STAGES 8
`define MATMUL_STAGES 29

/****************** SIZE OF THE MATMUL UNIT ***************/
`define MAT_MUL_SIZE 8

`endif
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/scalar/ram_wrapper.v
//////////////////////////////////////////////////////////////////////
module ram_wrapper (
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

parameter AWIDTH=10;
parameter NUM_WORDS=1024;
parameter DWIDTH=32;

input clk;
input resetn;
input [(AWIDTH-1):0] address_a;
input [(AWIDTH-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(DWIDTH-1):0] data_a;
input [(DWIDTH-1):0] data_b;
output [(DWIDTH-1):0] out_a;
output [(DWIDTH-1):0] out_b;

reg [(AWIDTH-1):0] q_address_a;
reg [(AWIDTH-1):0] q_address_b;
reg [(AWIDTH-1):0] mux_address_b;

dpram dpram1(
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
defparam
    dpram1.AWIDTH=AWIDTH,
    dpram1.NUM_WORDS=NUM_WORDS,
    dpram1.DWIDTH=DWIDTH;

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

endmodule


//////////////////////////////////////////////////////////////////////
//// Ending contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/scalar/ram_wrapper.v
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/scalar/components.v
//////////////////////////////////////////////////////////////////////


/****************************************************************************
          Generic Register - synchronous reset
****************************************************************************/
module register_sync(d,clk,resetn,en,q);
parameter WIDTH=32;

input clk;
input resetn;
input en;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
reg [WIDTH-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule

/****************************************************************************
          Generic Pipelined Register

          - Special component, components starting with "pipereg" have
          their enables treated independently of instructrions that use them.
          - They are enabled whenever the stage is active and not stalled
****************************************************************************/
module pipereg(d,clk,resetn,en,squashn,q);
parameter WIDTH=32;

input clk;
input resetn;
input en;
input squashn;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
reg [WIDTH-1:0] q;

always @(posedge clk)   //synchronous reset
begin
  if (resetn==0 || squashn==0)
    q<=0;
  else if (en==1)
    q<=d;
end

endmodule

/****************************************************************************
          Generic Pipelined Register 2 -OLD: If not enabled, queues squash

          - This piperegister stalls the reset signal as well
module pipereg_full(d,clk,resetn,squashn,en,q);
parameter WIDTH=32;

input clk;
input resetn;
input en;
input squashn;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
reg [WIDTH-1:0] q;
reg squash_save;

  always @(posedge clk)   //synchronous reset
  begin
    if (resetn==0 || (squashn==0 && en==1) || (squash_save&en))
      q<=0;
    else if (en==1)
      q<=d;
  end

  always @(posedge clk)
  begin
    if (resetn==1 && squashn==0 && en==0)
      squash_save<=1;
    else
      squash_save<=0;
  end
endmodule
****************************************************************************/


                
/****************************************************************************
          One cycle - Pipeline delay register
****************************************************************************/
module pipedelayreg(d,en,clk,resetn,squashn,dst,stalled,q);
parameter WIDTH=32;
input [WIDTH-1:0] d;
input [4:0] dst;
input en;
input clk;
input resetn;
input squashn;
output stalled;
output [WIDTH-1:0] q;

  reg [WIDTH-1:0] q;
  reg T,Tnext;

  // State machine for Stalling 1 cycle
  always@(en or T or dst)
  begin
    case(T) 
      0: Tnext=en&(|dst);
      1: Tnext=0;
    endcase 
  end       
  always@(posedge clk)
    if (~resetn)
      T<=0; 
    else    
      T<=Tnext;

  always @(posedge clk)   //synchronous reset
  begin
    if (resetn==0 || squashn==0)
      q<=0;
    else if (en==1)
      q<=d;
  end

  assign stalled=(en&~T&(|dst));
endmodule

/****************************************************************************
          Fake Delay
****************************************************************************/
module fakedelay(d,clk,q);
parameter WIDTH=32;
input [WIDTH-1:0] d;
input clk;
output [WIDTH-1:0] q;

assign q=d;

endmodule

/****************************************************************************
          Zeroer
****************************************************************************/
module zeroer(d,en,q);
parameter WIDTH=32;

input en;
input [WIDTH-1:0] d;
output [WIDTH-1:0] q;
assign q= (en) ? d : 0;

endmodule

/****************************************************************************
          NOP - used to hack position of multiplexors
****************************************************************************/
module nop(d,q);
parameter WIDTH=32;

input [WIDTH-1:0] d;
output [WIDTH-1:0] q;

  assign q=d;

endmodule

/****************************************************************************
          Const
****************************************************************************/
module const (out);

parameter WIDTH=32;
parameter VAL=31;

output [WIDTH-1:0] out;

assign out=VAL;

endmodule

/****************************************************************************
          Branch detector
****************************************************************************/
module branch_detector(opcode, func, is_branch);
input [5:0] opcode;
input [5:0] func;
output is_branch;

wire is_special;

assign is_special=!(|opcode);
assign is_branch=((!(|opcode[5:3])) && !is_special) || 
                  ((is_special)&&(func[5:3]==3'b001));

endmodule

//////////////////////////////////////////////////////////////////////
//// Ending contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/scalar/components.v
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/bfloat/activation.v
//////////////////////////////////////////////////////////////////////
module activation #(parameter WIDTH = 4)(
 input clk,
 input resetn,
 input en,
 input stall,
 input [WIDTH-1:0] a,
 output reg[WIDTH-1:0] out
);

always@(posedge clk)begin
  if(!resetn)
    out <= 'h0;
  else
    if(en)
      if(a[WIDTH-1] == 1'b0)
        out <= a;
      else
        out <= 0;
end

endmodule

//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/bfloat/activation.v
//////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/bfloat/FPAddSub.v
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
//
// Create Date:    01:56:20 09/07/2012 
// Module Name:    FPAddSub
// Project Name:   Floating Point Project
// Author:       Fredrik Brosser
//
// Description:  Top Module for a 32-bit floating point adder/subtractor.
//             Follows the IEEE754 Single Precision standard.
//             Supports only the default rounding mode.
//
//  Inputs:
//        a (32 bits)     : Single precision IEEE754 floating point number
//        b (32 bits)     : Single precision IEEE754 floating point number
//        operation (1 bit) : Single control bit. 0/Addition, 1/Subtraction
//
//
// Outputs:
//        result (32 bits)  : Result of the operation, in IEEE754 Single format
//        flags  (5 bits) : Flags indicating exceptions:
//                      Bit 4: Overflow
//                      Bit 3: Underflow
//                      Bit 2: Divide by Zero
//                      Bit 1: Invalid/NaN
//                      Bit 0: Inexact
//
//////////////////////////////////////////////////////////////////////////////////
`define EXPONENT 5
`define MANTISSA 10
`define ACTUAL_MANTISSA 11
`define EXPONENT_LSB 10
`define EXPONENT_MSB 14
`define MANTISSA_LSB 0
`define MANTISSA_MSB 9
`define MANTISSA_MUL_SPLIT_LSB 3
`define MANTISSA_MUL_SPLIT_MSB 9
`define SIGN 1
`define SIGN_LOC 15
`define DWIDTH (`SIGN+`EXPONENT+`MANTISSA)
`define IEEE_COMPLIANCE 1

module FPAddSub(
    clk,
    rst,
                en,
    a,
    b,
    operation,      // 0 add, 1 sub
    result,
                valid,
    flags
  );
  
  // Clock and reset
  input clk ;                   // Clock signal
  input rst ;                   // Reset (active high, resets pipeline registers)
  
  // Input ports
  input en ;
  input [`DWIDTH-1:0] a ;               // Input A, a 32-bit floating point number
  input [`DWIDTH-1:0] b ;               // Input B, a 32-bit floating point number
  input operation ;               // Operation select signal
  
  // Output ports
  output valid;
  output [`DWIDTH-1:0] result ;           // Result of the operation
  output [4:0] flags ;              // Flags indicating exceptions according to IEEE754
  
  // Pipeline Registers
  //reg [79:0] pipe_1;              // Pipeline register PreAlign->Align1
  reg [`DWIDTH*2+15:0] pipe_1;              // Pipeline register PreAlign->Align1
        wire[`DWIDTH*2+15:0] stage_0_1;

  //reg [67:0] pipe_2;              // Pipeline register Align1->Align3
  reg [`MANTISSA*2+`EXPONENT+13:0] pipe_2;              // Pipeline register Align1->Align3
        wire[`MANTISSA*2+`EXPONENT+13:0] stage_1_2;

  //reg [76:0] pipe_3;  68            // Pipeline register Align1->Align3
  reg [`MANTISSA*2+`EXPONENT+14:0] pipe_3;              // Pipeline register Align1->Align3

  //reg [69:0] pipe_4;              // Pipeline register Align3->Execute
  reg [`MANTISSA*2+`EXPONENT+15:0] pipe_4;              // Pipeline register Align3->Execute
        wire[`MANTISSA*2+`EXPONENT+15:0] stage_3_4;

  //reg [51:0] pipe_5;              // Pipeline register Execute->Normalize
  reg [`DWIDTH+`EXPONENT+11:0] pipe_5;              // Pipeline register Execute->Normalize
        wire[`DWIDTH+`EXPONENT+11:0] stage_4_5;

  //reg [56:0] pipe_6;              // Pipeline register Nomalize->NormalizeShift1
  reg [`DWIDTH+`EXPONENT+16:0] pipe_6;              // Pipeline register Nomalize->NormalizeShift1

  //reg [56:0] pipe_7;              // Pipeline register NormalizeShift2->NormalizeShift3
  reg [`DWIDTH+`EXPONENT+16:0] pipe_7;              // Pipeline register NormalizeShift2->NormalizeShift3
        wire[`DWIDTH+`EXPONENT+16:0] stage_6_7;

  //reg [54:0] pipe_8;              // Pipeline register NormalizeShift3->Round
  reg [`EXPONENT*2+`MANTISSA+15:0] pipe_8;              // Pipeline register NormalizeShift3->Round
        wire[`EXPONENT*2+`MANTISSA+15:0] stage_7_8;

  //reg [40:0] pipe_9;              // Pipeline register NormalizeShift3->Round
  reg [`DWIDTH+8:0] pipe_9;             // Pipeline register NormalizeShift3->Round
  
  // Internal wires between modules
  wire [`DWIDTH-2:0] Aout_0 ;             // A - sign
  wire [`DWIDTH-2:0] Bout_0 ;             // B - sign
  wire Opout_0 ;                  // A's sign
  wire Sa_0 ;                   // A's sign
  wire Sb_0 ;                   // B's sign
  wire MaxAB_1 ;                  // Indicates the larger of A and B(0/A, 1/B)
  wire [`EXPONENT-1:0] CExp_1 ;             // Common Exponent
  wire [4:0] Shift_1 ;              // Number of steps to smaller mantissa shift right (align)
  wire [`MANTISSA-1:0] Mmax_1 ;             // Larger mantissa
  wire [4:0] InputExc_0 ;           // Input numbers are exceptions
  wire [9:0] ShiftDet_0 ;
  wire [`MANTISSA-1:0] MminS_1 ;            // Smaller mantissa after 0/16 shift
  wire [`MANTISSA:0] MminS_2 ;            // Smaller mantissa after 0/4/8/12 shift
  wire [`MANTISSA:0] Mmin_3 ;             // Smaller mantissa after 0/1/2/3 shift
  wire [`DWIDTH:0] Sum_4 ;
  wire PSgn_4 ;
  wire Opr_4 ;
  wire [4:0] Shift_5 ;              // Number of steps to shift sum left (normalize)
  wire [`DWIDTH:0] SumS_5 ;             // Sum after 0/16 shift
  wire [`DWIDTH:0] SumS_6 ;             // Sum after 0/16 shift
  wire [`DWIDTH:0] SumS_7 ;             // Sum after 0/16 shift
  wire [`MANTISSA-1:0] NormM_8 ;            // Normalized mantissa
  wire [`EXPONENT:0] NormE_8;             // Adjusted exponent
  wire ZeroSum_8 ;                // Zero flag
  wire NegE_8 ;                 // Flag indicating negative exponent
  wire R_8 ;                    // Round bit
  wire S_8 ;                    // Final sticky bit
  wire FG_8 ;                   // Final sticky bit
  wire [`DWIDTH-1:0] P_int ;
  wire EOF ;
        reg q1_valid;
        reg q2_valid;
        reg q3_valid;
        
  assign valid = q3_valid;
  // Prepare the operands for alignment and check for exceptions
  FPAddSub_PrealignModule PrealignModule
  ( // Inputs
    a, b, operation,
    // Outputs
    Sa_0, Sb_0, ShiftDet_0[9:0], InputExc_0[4:0], Aout_0[`DWIDTH-2:0], Bout_0[`DWIDTH-2:0], Opout_0) ;
  
        assign stage_0_1 = {Opout_0, Aout_0[`DWIDTH-2:0], Bout_0[`DWIDTH-2:0], Sa_0, Sb_0, ShiftDet_0[9:0], InputExc_0[4:0]};
  
  // Prepare the operands for alignment and check for exceptions
  FPAddSub_AlignModule AlignModule
  ( // Inputs
    //pipe_1[14+2*`DWIDTH:16+`DWIDTH], pipe_1[15+`DWIDTH:17], pipe_1[14:5],
    stage_0_1[14+2*`DWIDTH:16+`DWIDTH], stage_0_1[15+`DWIDTH:17], stage_0_1[14:5],
    // Outputs
    CExp_1[`EXPONENT-1:0], MaxAB_1, Shift_1[4:0], MminS_1[`MANTISSA-1:0], Mmax_1[`MANTISSA-1:0]) ;  

        assign stage_1_2 = {stage_0_1[`DWIDTH*2+15], stage_0_1[16:15], MaxAB_1, CExp_1[`EXPONENT-1:0], Shift_1[4:0], Mmax_1[`MANTISSA-1:0], stage_0_1[4:0], MminS_1[`MANTISSA-1:0]};
          
  // Alignment Shift Stage 1
  FPAddSub_AlignShift1 AlignShift1
  (  // Inputs
    stage_1_2[`MANTISSA-1:0], stage_1_2[2*`MANTISSA+9:2*`MANTISSA+7],
    // Outputs
    MminS_2[`MANTISSA:0]) ;

  // Alignment Shift Stage 3 and compution of guard and sticky bits
  FPAddSub_AlignShift2 AlignShift2  
  (  // Inputs
    pipe_3[`MANTISSA:0], pipe_3[2*`MANTISSA+7:2*`MANTISSA+6],
    // Outputs
    Mmin_3[`MANTISSA:0]) ;
            
        assign stage_3_4 = {pipe_3[`MANTISSA*2+`EXPONENT+14:`MANTISSA+1], Mmin_3[`MANTISSA:0]};
  // Perform mantissa addition
  FPAddSub_ExecutionModule ExecutionModule
  (  // Inputs
    stage_3_4[`MANTISSA*2+5:`MANTISSA+6], stage_3_4[`MANTISSA:0], stage_3_4[`MANTISSA*2+`EXPONENT+13], stage_3_4[`MANTISSA*2+`EXPONENT+12], stage_3_4[`MANTISSA*2+`EXPONENT+11], stage_3_4[`MANTISSA*2+`EXPONENT+14],
    // Outputs
    Sum_4[`DWIDTH:0], PSgn_4, Opr_4) ;
        assign stage_4_5 = {stage_3_4[2*`MANTISSA+`EXPONENT+14], PSgn_4, Opr_4, stage_3_4[2*`MANTISSA+`EXPONENT+13:2*`MANTISSA+11], stage_3_4[`MANTISSA+5:`MANTISSA+1], Sum_4[`DWIDTH:0]};
  
  // Prepare normalization of result
  FPAddSub_NormalizeModule NormalizeModule
  (  // Inputs
    stage_4_5[`DWIDTH:0], 
    // Outputs
    SumS_5[`DWIDTH:0], Shift_5[4:0]) ;
          
  // Normalization Shift Stage 1
  FPAddSub_NormalizeShift1 NormalizeShift1
  (  // Inputs
    pipe_6[`DWIDTH:0], pipe_6[`DWIDTH+`EXPONENT+14:`DWIDTH+`EXPONENT+11],
    // Outputs
    SumS_7[`DWIDTH:0]) ;
        
        assign stage_6_7 = {pipe_6[`DWIDTH+`EXPONENT+16:`DWIDTH+1], SumS_7[`DWIDTH:0]};
    
  // Normalization Shift Stage 3 and final guard, sticky and round bits
  FPAddSub_NormalizeShift2 NormalizeShift2
  (  // Inputs
    stage_6_7[`DWIDTH:0], stage_6_7[`DWIDTH+`EXPONENT+5:`DWIDTH+6], stage_6_7[`DWIDTH+`EXPONENT+15:`DWIDTH+`EXPONENT+11],
    // Outputs
    NormM_8[`MANTISSA-1:0], NormE_8[`EXPONENT:0], ZeroSum_8, NegE_8, R_8, S_8, FG_8) ;
       
        assign stage_7_8 = {FG_8, stage_6_7[`DWIDTH+`EXPONENT+16], stage_6_7[`DWIDTH+`EXPONENT+10], stage_6_7[`DWIDTH+`EXPONENT+8:`DWIDTH+1], NormM_8[`MANTISSA-1:0], NormE_8[`EXPONENT:0], ZeroSum_8, NegE_8, R_8, S_8};

  // Round and put result together
  FPAddSub_RoundModule RoundModule
  (  // Inputs
     stage_7_8[3], stage_7_8[4+`EXPONENT:4], stage_7_8[`EXPONENT+`MANTISSA+4:5+`EXPONENT], stage_7_8[1], stage_7_8[0], stage_7_8[`EXPONENT*2+`MANTISSA+15], stage_7_8[`EXPONENT*2+`MANTISSA+12], stage_7_8[`EXPONENT*2+`MANTISSA+11], stage_7_8[`EXPONENT*2+`MANTISSA+14], stage_7_8[`EXPONENT*2+`MANTISSA+10], 
    // Outputs
    P_int[`DWIDTH-1:0], EOF) ;
  
  // Check for exceptions
  FPAddSub_ExceptionModule Exceptionmodule
  (  // Inputs
    pipe_9[8+`DWIDTH:9], pipe_9[8], pipe_9[7], pipe_9[6], pipe_9[5:1], pipe_9[0], 
    // Outputs
    result[`DWIDTH-1:0], flags[4:0]) ;      
  
  always @ (posedge clk) begin  
    if(rst) begin
      //pipe_1 = 0;
      //pipe_2 = 0;
      pipe_3 = 0;
      //pipe_4 = 0;
      //pipe_5 = 0;
      pipe_6 = 0;
      //pipe_7 = 0;
      //pipe_8 = 0;
      pipe_9 = 0;
    end 
    else begin
    
      //pipe_1 = {Opout_0, Aout_0[`DWIDTH-2:0], Bout_0[`DWIDTH-2:0], Sa_0, Sb_0, ShiftDet_0[9:0], InputExc_0[4:0]} ;  
      /* PIPE_2 :
        [67] operation
        [66] Sa_0
        [65] Sb_0
        [64] MaxAB_0
        [63:56] CExp_0
        [55:51] Shift_0
        [50:28] Mmax_0
        [27:23] InputExc_0
        [22:0] MminS_1
      */
      //pipe_2 = {pipe_1[`DWIDTH*2+15], pipe_1[16:15], MaxAB_1, CExp_1[`EXPONENT-1:0], Shift_1[4:0], Mmax_1[`MANTISSA-1:0], pipe_1[4:0], MminS_1[`MANTISSA-1:0]} ;  
      /* PIPE_3 :
        [68] operation
        [67] Sa_0
        [66] Sb_0
        [65] MaxAB_0
        [64:57] CExp_0
        [56:52] Shift_0
        [51:29] Mmax_0
        [28:24] InputExc_0
        [23:0] MminS_1
      */
      pipe_3 = {stage_1_2[`MANTISSA*2+`EXPONENT+13:`MANTISSA], MminS_2[`MANTISSA:0]} ;  
      /* PIPE_4 :
        [68] operation
        [67] Sa_0
        [66] Sb_0
        [65] MaxAB_0
        [64:57] CExp_0
        [56:52] Shift_0
        [51:29] Mmax_0
        [28:24] InputExc_0
        [23:0] Mmin_3
      */          
      //pipe_4 = {pipe_3[`MANTISSA*2+`EXPONENT+14:`MANTISSA+1], Mmin_3[`MANTISSA:0]} ;  
      /* PIPE_5 :
        [51] operation
        [50] PSgn_4
        [49] Opr_4
        [48] Sa_0
        [47] Sb_0
        [46] MaxAB_0
        [45:38] CExp_0
        [37:33] InputExc_0
        [32:0] Sum_4
      */          
      //pipe_5 = {pipe_4[2*`MANTISSA+`EXPONENT+14], PSgn_4, Opr_4, pipe_4[2*`MANTISSA+`EXPONENT+13:2*`MANTISSA+11], pipe_4[`MANTISSA+5:`MANTISSA+1], Sum_4[`DWIDTH:0]} ;
      /* PIPE_6 :
        [56] operation
        [55:51] Shift_5
        [50] PSgn_4
        [49] Opr_4
        [48] Sa_0
        [47] Sb_0
        [46] MaxAB_0
        [45:38] CExp_0
        [37:33] InputExc_0
        [32:0] Sum_4
      */          
      pipe_6 = {stage_4_5[`EXPONENT+`EXPONENT+11], Shift_5[4:0], stage_4_5[`DWIDTH+`EXPONENT+10:`DWIDTH+1], SumS_5[`DWIDTH:0]} ;  
      /* pipe_7 :
        [56] operation
        [55:51] Shift_5
        [50] PSgn_4
        [49] Opr_4
        [48] Sa_0
        [47] Sb_0
        [46] MaxAB_0
        [45:38] CExp_0
        [37:33] InputExc_0
        [32:0] Sum_4
      */            
      //pipe_7 = {pipe_6[`DWIDTH+`EXPONENT+16:`DWIDTH+1], SumS_7[`DWIDTH:0]} ;  
      /* pipe_8:
        [54] FG_8 
        [53] operation
        [52] PSgn_4
        [51] Sa_0
        [50] Sb_0
        [49] MaxAB_0
        [48:41] CExp_0
        [40:36] InputExc_8
        [35:13] NormM_8 
        [12:4] NormE_8
        [3] ZeroSum_8
        [2] NegE_8
        [1] R_8
        [0] S_8
      */        
      //pipe_8 = {FG_8, pipe_6[`DWIDTH+`EXPONENT+16], pipe_6[`DWIDTH+`EXPONENT+10], pipe_6[`DWIDTH+`EXPONENT+8:`DWIDTH+1], NormM_8[`MANTISSA-1:0], NormE_8[`EXPONENT:0], ZeroSum_8, NegE_8, R_8, S_8} ; 
      /* pipe_9:
        [40:9] P_int
        [8] NegE_8
        [7] R_8
        [6] S_8
        [5:1] InputExc_8
        [0] EOF
      */        
      pipe_9 = {P_int[`DWIDTH-1:0], stage_6_7[2], stage_6_7[1], stage_6_7[0], stage_6_7[`EXPONENT+`MANTISSA+9:`EXPONENT+`MANTISSA+5], EOF} ;  
    end
  end   


always@(posedge clk)begin
  if(rst)begin
      q1_valid = 0;
      q2_valid = 0;
      q3_valid = 0;
  end
  else begin
      q1_valid <= en;
      q2_valid <= q1_valid;
      q3_valid <= q2_valid;
  end
end
  
endmodule

//////////////////////////////////////////////////////////////////////////////////
//
// Create Date:     16:49:15 10/16/2012 
// Module Name:     FPAddSub_PrealignModule
// Project Name:    Floating Point Project
// Author:        Fredrik Brosser
//
// Description:   The pre-alignment module is responsible for taking the inputs
//              apart and checking the parts for exceptions.
//              The exponent difference is also calculated in this module.
//
//////////////////////////////////////////////////////////////////////////////////

module FPAddSub_PrealignModule(
    A,
    B,
    operation,
    Sa,
    Sb,
    ShiftDet,
    InputExc,
    Aout,
    Bout,
    Opout
  );
  
  // Input ports
  input [`DWIDTH-1:0] A ;                   // Input A, a 32-bit floating point number
  input [`DWIDTH-1:0] B ;                   // Input B, a 32-bit floating point number
  input operation ;
  
  // Output ports
  output Sa ;                       // A's sign
  output Sb ;                       // B's sign
  output [9:0] ShiftDet ;
  output [4:0] InputExc ;               // Input numbers are exceptions
  output [`DWIDTH-2:0] Aout ;
  output [`DWIDTH-2:0] Bout ;
  output Opout ;
  
  // Internal signals                 // If signal is high...
  wire ANaN ;                       // A is a NaN (Not-a-Number)
  wire BNaN ;                       // B is a NaN
  wire AInf ;                       // A is infinity
  wire BInf ;                       // B is infinity
  wire [`EXPONENT-1:0] DAB ;                    // ExpA - ExpB          
  wire [`EXPONENT-1:0] DBA ;                    // ExpB - ExpA  
  
  assign ANaN = &(A[`DWIDTH-2:`DWIDTH-1-`EXPONENT]) & |(A[`MANTISSA-1:0]) ;   // All one exponent and not all zero mantissa - NaN
  assign BNaN = &(B[`DWIDTH-2:`DWIDTH-1-`EXPONENT]) & |(B[`MANTISSA-1:0]);    // All one exponent and not all zero mantissa - NaN
  assign AInf = &(A[`DWIDTH-2:`DWIDTH-1-`EXPONENT]) & ~|(A[`MANTISSA-1:0]) ;  // All one exponent and all zero mantissa - Infinity
  assign BInf = &(B[`DWIDTH-2:`DWIDTH-1-`EXPONENT]) & ~|(B[`MANTISSA-1:0]) ;  // All one exponent and all zero mantissa - Infinity
  
  // Put all flags into exception vector
  assign InputExc = {(ANaN | BNaN | AInf | BInf), ANaN, BNaN, AInf, BInf} ;
  
  //assign DAB = (A[30:23] - B[30:23]) ;
  //assign DBA = (B[30:23] - A[30:23]) ;
  assign DAB = (A[`DWIDTH-2:`MANTISSA] + ~(B[`DWIDTH-2:`MANTISSA]) + 1) ;
  assign DBA = (B[`DWIDTH-2:`MANTISSA] + ~(A[`DWIDTH-2:`MANTISSA]) + 1) ;
  
  assign Sa = A[`DWIDTH-1] ;                  // A's sign bit
  assign Sb = B[`DWIDTH-1] ;                  // B's sign bit
  assign ShiftDet = {DBA[4:0], DAB[4:0]} ;    // Shift data
  assign Opout = operation ;
  assign Aout = A[`DWIDTH-2:0] ;
  assign Bout = B[`DWIDTH-2:0] ;
  
endmodule

//////////////////////////////////////////////////////////////////////////////////
//
// Create Date:     16:49:15 10/16/2012 
// Module Name:     FPAddSub_AlignModule
// Project Name:    Floating Point Project
// Author:        Fredrik Brosser
//
// Description:   The alignment module determines the larger input operand and
//              sets the mantissas, shift and common exponent accordingly.
//
//////////////////////////////////////////////////////////////////////////////////

module FPAddSub_AlignModule (
    A,
    B,
    ShiftDet,
    CExp,
    MaxAB,
    Shift,
    Mmin,
    Mmax
  );
  
  // Input ports
  input [`DWIDTH-2:0] A ;               // Input A, a 32-bit floating point number
  input [`DWIDTH-2:0] B ;               // Input B, a 32-bit floating point number
  input [9:0] ShiftDet ;
  
  // Output ports
  output [`EXPONENT-1:0] CExp ;             // Common Exponent
  output MaxAB ;                  // Incidates larger of A and B (0/A, 1/B)
  output [4:0] Shift ;              // Number of steps to smaller mantissa shift right
  output [`MANTISSA-1:0] Mmin ;             // Smaller mantissa 
  output [`MANTISSA-1:0] Mmax ;             // Larger mantissa
  
  // Internal signals
  //wire BOF ;                    // Check for shifting overflow if B is larger
  //wire AOF ;                    // Check for shifting overflow if A is larger
  
  assign MaxAB = (A[`DWIDTH-2:0] < B[`DWIDTH-2:0]) ;  
  //assign BOF = ShiftDet[9:5] < 25 ;   // Cannot shift more than 25 bits
  //assign AOF = ShiftDet[4:0] < 25 ;   // Cannot shift more than 25 bits
  
  // Determine final shift value
  //assign Shift = MaxAB ? (BOF ? ShiftDet[9:5] : 5'b11001) : (AOF ? ShiftDet[4:0] : 5'b11001) ;
  
  assign Shift = MaxAB ? ShiftDet[9:5] : ShiftDet[4:0] ;
  
  // Take out smaller mantissa and append shift space
  assign Mmin = MaxAB ? A[`MANTISSA-1:0] : B[`MANTISSA-1:0] ; 
  
  // Take out larger mantissa 
  assign Mmax = MaxAB ? B[`MANTISSA-1:0]: A[`MANTISSA-1:0] ;  
  
  // Common exponent
  assign CExp = (MaxAB ? B[`MANTISSA+`EXPONENT-1:`MANTISSA] : A[`MANTISSA+`EXPONENT-1:`MANTISSA]) ;   
  
endmodule

//////////////////////////////////////////////////////////////////////////////////
//
// Create Date:    16:49:36 10/16/2012 
// Module Name:    FPAddSub_AlignShift1
// Project Name:   Floating Point Project
// Author:       Fredrik Brosser
//
// Description:  Alignment shift stage 1, performs 16|12|8|4 shift
//
//////////////////////////////////////////////////////////////////////////////////

module FPAddSub_AlignShift1(
    MminP,
    Shift,
    Mmin
  );
  
  // Input ports
  input [`MANTISSA-1:0] MminP ;           // Smaller mantissa after 16|12|8|4 shift
  input [2:0] Shift ;           // Shift amount
  
  // Output ports
  output [`MANTISSA:0] Mmin ;           // The smaller mantissa
  
  // Internal signals
  reg   [`MANTISSA:0]   Lvl1;
  reg   [`MANTISSA:0]   Lvl2;
  wire    [2*`MANTISSA+1:0]    Stage1;  
  integer           i;                // Loop variable
  
  always @(*) begin           
    // Rotate by 16?
    //Lvl1 <= Shift[2] ? {17'b00000000000000001, MminP[22:16]} : {1'b1, MminP}; 
    Lvl1 <= Shift[2] ? {11'b0000000000} : {1'b1, MminP}; 
    
  end
  
  assign Stage1 = { 11'b0, Lvl1};
  
  always @(*) begin             // Rotate {0 | 4 | 8 | 12} bits
    case (Shift[1:0])
      // Rotate by 0  
      2'b00:  Lvl2 <= Stage1[`MANTISSA:0];            
      // Rotate by 4  
      2'b01:  begin for (i=0; i<=`MANTISSA; i=i+1) begin Lvl2[i] <= Stage1[i+4]; end Lvl2[`MANTISSA:`MANTISSA-3] <= 0; end
      // Rotate by 8
      2'b10:  begin for (i=0; i<=`MANTISSA; i=i+1) begin Lvl2[i] <= Stage1[i+8]; end Lvl2[`MANTISSA:`MANTISSA-7] <= 0; end
      // Rotate by 12 
      2'b11: Lvl2[`MANTISSA: 0] <= 0; 
      //2'b11:  begin for (i=0; i<=`MANTISSA; i=i+1) begin Lvl2[i] <= Stage1[i+12]; end Lvl2[`MANTISSA:`MANTISSA-12] <= 0; end
    endcase
  end
  
  // Assign output to next shift stage
  assign Mmin = Lvl2;
  
endmodule

//////////////////////////////////////////////////////////////////////////////////
//
// Create Date:    16:50:05 10/16/2012 
// Module Name:    FPAddSub_AlignShift2
// Project Name:   Floating Point Project
// Author:       Fredrik Brosser
//
// Description:  Alignment shift stage 2, performs 3|2|1 shift
//
//////////////////////////////////////////////////////////////////////////////////

module FPAddSub_AlignShift2(
    MminP,
    Shift,
    Mmin
  );
  
  // Input ports
  input [`MANTISSA:0] MminP ;           // Smaller mantissa after 16|12|8|4 shift
  input [1:0] Shift ;           // Shift amount
  
  // Output ports
  output [`MANTISSA:0] Mmin ;           // The smaller mantissa
  
  // Internal Signal
  reg   [`MANTISSA:0]   Lvl3;
  wire    [2*`MANTISSA+1:0]    Stage2;  
  integer           j;               // Loop variable
  
  assign Stage2 = {11'b0, MminP};

  always @(*) begin    // Rotate {0 | 1 | 2 | 3} bits
    case (Shift[1:0])
      // Rotate by 0
      2'b00:  Lvl3 <= Stage2[`MANTISSA:0];   
      // Rotate by 1
      2'b01:  begin for (j=0; j<=`MANTISSA; j=j+1)  begin Lvl3[j] <= Stage2[j+1]; end Lvl3[`MANTISSA] <= 0; end 
      // Rotate by 2
      2'b10:  begin for (j=0; j<=`MANTISSA; j=j+1)  begin Lvl3[j] <= Stage2[j+2]; end Lvl3[`MANTISSA:`MANTISSA-1] <= 0; end 
      // Rotate by 3
      2'b11:  begin for (j=0; j<=`MANTISSA; j=j+1)  begin Lvl3[j] <= Stage2[j+3]; end Lvl3[`MANTISSA:`MANTISSA-2] <= 0; end     
    endcase
  end
  
  // Assign output
  assign Mmin = Lvl3;           // Take out smaller mantissa        

endmodule

//////////////////////////////////////////////////////////////////////////////////
//
// Create Date:    11:35:05 09/05/2012 
// Module Name:    FPAddSub_ExecutionModule 
// Project Name:   Floating Point Project
// Author:       Fredrik Brosser
//
// Description:  Module that executes the addition or subtraction on mantissas.
//
//////////////////////////////////////////////////////////////////////////////////

module FPAddSub_ExecutionModule(
    Mmax,
    Mmin,
    Sa,
    Sb,
    MaxAB,
    OpMode,
    Sum,
    PSgn,
    Opr
    );

  // Input ports
  input [`MANTISSA-1:0] Mmax ;          // The larger mantissa
  input [`MANTISSA:0] Mmin ;          // The smaller mantissa
  input Sa ;                // Sign bit of larger number
  input Sb ;                // Sign bit of smaller number
  input MaxAB ;             // Indicates the larger number (0/A, 1/B)
  input OpMode ;              // Operation to be performed (0/Add, 1/Sub)
  
  // Output ports
  output [`DWIDTH:0] Sum ;          // The result of the operation
  output PSgn ;             // The sign for the result
  output Opr ;              // The effective (performed) operation

  assign Opr = (OpMode^Sa^Sb);    // Resolve sign to determine operation

  // Perform effective operation
  assign Sum = (OpMode^Sa^Sb) ? ({1'b1, Mmax, 5'b00000} - {Mmin, 5'b00000}) : ({1'b1, Mmax, 5'b00000} + {Mmin, 5'b00000}) ;
  
  // Assign result sign
  assign PSgn = (MaxAB ? Sb : Sa) ;

endmodule

//////////////////////////////////////////////////////////////////////////////////
//
// Create Date:    16:05:07 09/07/2012
// Module Name:    FBAddSub_NormalizeModule
// Project Name:   Floating Point Project
// Author:       Fredrik Brosser
//
// Description:  Determine the normalization shift amount and perform 16-shift
//
//////////////////////////////////////////////////////////////////////////////////

module FPAddSub_NormalizeModule(
    Sum,
    Mmin,
    Shift
    );

  // Input ports
  input [`DWIDTH:0] Sum ;         // Mantissa sum including hidden 1 and GRS
  
  // Output ports
  output [`DWIDTH:0] Mmin ;         // Mantissa after 16|0 shift
  output [4:0] Shift ;          // Shift amount
  
  // Determine normalization shift amount by finding leading nought
  assign Shift =  ( 
    Sum[16] ? 5'b00000 :   
    Sum[15] ? 5'b00001 : 
    Sum[14] ? 5'b00010 : 
    Sum[13] ? 5'b00011 : 
    Sum[12] ? 5'b00100 : 
    Sum[11] ? 5'b00101 : 
    Sum[10] ? 5'b00110 : 
    Sum[9] ? 5'b00111 :
    Sum[8] ? 5'b01000 :
    Sum[7] ? 5'b01001 :
    Sum[6] ? 5'b01010 :
    Sum[5] ? 5'b01011 :
    Sum[4] ? 5'b01100 : 5'b01101
  //  Sum[19] ? 5'b01101 :
  //  Sum[18] ? 5'b01110 :
  //  Sum[17] ? 5'b01111 :
  //  Sum[16] ? 5'b10000 :
  //  Sum[15] ? 5'b10001 :
  //  Sum[14] ? 5'b10010 :
  //  Sum[13] ? 5'b10011 :
  //  Sum[12] ? 5'b10100 :
  //  Sum[11] ? 5'b10101 :
  //  Sum[10] ? 5'b10110 :
  //  Sum[9] ? 5'b10111 :
  //  Sum[8] ? 5'b11000 :
  //  Sum[7] ? 5'b11001 : 5'b11010
  );
  
  reg   [`DWIDTH:0]   Lvl1;
  
  always @(*) begin
    // Rotate by 16?
    Lvl1 <= Shift[4] ? {Sum[8:0], 8'b00000000} : Sum; 
  end
  
  // Assign outputs
  assign Mmin = Lvl1;           // Take out smaller mantissa

endmodule

//////////////////////////////////////////////////////////////////////////////////
//
// Create Date:    16:49:36 10/16/2012 
// Module Name:    FPAddSub_NormalizeShift1 
// Project Name:   Floating Point Project
// Author:       Fredrik Brosser
//
// Description:  Normalization shift stage 1, performs 12|8|4|3|2|1|0 shift
//
//////////////////////////////////////////////////////////////////////////////////

module FPAddSub_NormalizeShift1(
    MminP,
    Shift,
    Mmin
  );
  
  // Input ports
  input [`DWIDTH:0] MminP ;           // Smaller mantissa after 16|12|8|4 shift
  input [3:0] Shift ;           // Shift amount
  
  // Output ports
  output [`DWIDTH:0] Mmin ;           // The smaller mantissa
  
  reg   [`DWIDTH:0]   Lvl2;
  wire    [2*`DWIDTH+1:0]    Stage1;  
  reg   [`DWIDTH:0]   Lvl3;
  wire    [2*`DWIDTH+1:0]    Stage2;  
  integer           i;                // Loop variable
  
  assign Stage1 = {MminP, MminP};

  always @(*) begin             // Rotate {0 | 4 | 8 | 12} bits
    case (Shift[3:2])
      // Rotate by 0
      2'b00: Lvl2 <= Stage1[`DWIDTH:0];           
      // Rotate by 4
      2'b01: begin for (i=2*`DWIDTH+1; i>=`DWIDTH+1; i=i-1) begin Lvl2[i-33] <= Stage1[i-4]; end Lvl2[3:0] <= 0; end
      // Rotate by 8
      2'b10: begin for (i=2*`DWIDTH+1; i>=`DWIDTH+1; i=i-1) begin Lvl2[i-33] <= Stage1[i-8]; end Lvl2[7:0] <= 0; end
      // Rotate by 12
      2'b11: begin for (i=2*`DWIDTH+1; i>=`DWIDTH+1; i=i-1) begin Lvl2[i-33] <= Stage1[i-12]; end Lvl2[11:0] <= 0; end
    endcase
  end
  
  assign Stage2 = {Lvl2, Lvl2};

  always @(*) begin               // Rotate {0 | 1 | 2 | 3} bits
    case (Shift[1:0])
      // Rotate by 0
      2'b00:  Lvl3 <= Stage2[`DWIDTH:0];
      // Rotate by 1
      2'b01: begin for (i=2*`DWIDTH+1; i>=`DWIDTH+1; i=i-1) begin Lvl3[i-`DWIDTH-1] <= Stage2[i-1]; end Lvl3[0] <= 0; end 
      // Rotate by 2
      2'b10: begin for (i=2*`DWIDTH+1; i>=`DWIDTH+1; i=i-1) begin Lvl3[i-`DWIDTH-1] <= Stage2[i-2]; end Lvl3[1:0] <= 0; end
      // Rotate by 3
      2'b11: begin for (i=2*`DWIDTH+1; i>=`DWIDTH+1; i=i-1) begin Lvl3[i-`DWIDTH-1] <= Stage2[i-3]; end Lvl3[2:0] <= 0; end
    endcase
  end
  
  // Assign outputs
  assign Mmin = Lvl3;           // Take out smaller mantissa      
  
endmodule

//////////////////////////////////////////////////////////////////////////////////
//
// Create Date:    17:34:18 10/16/2012 
// Module Name:    FPAddSub_NormalizeShift2 
// Project Name:   Floating Point Project
// Author:       Fredrik Brosser
//
// Description:  Normalization shift stage 2, calculates post-normalization
//             mantissa and exponent, as well as the bits used in rounding    
//
//////////////////////////////////////////////////////////////////////////////////

module FPAddSub_NormalizeShift2(
    PSSum,
    CExp,
    Shift,
    NormM,
    NormE,
    ZeroSum,
    NegE,
    R,
    S,
    FG
  );
  
  // Input ports
  input [`DWIDTH:0] PSSum ;         // The Pre-Shift-Sum
  input [`EXPONENT-1:0] CExp ;
  input [4:0] Shift ;         // Amount to be shifted

  // Output ports
  output [`MANTISSA-1:0] NormM ;        // Normalized mantissa
  output [`EXPONENT:0] NormE ;          // Adjusted exponent
  output ZeroSum ;            // Zero flag
  output NegE ;             // Flag indicating negative exponent
  output R ;                // Round bit
  output S ;                // Final sticky bit
  output FG ;

  // Internal signals
  wire MSBShift ;           // Flag indicating that a second shift is needed
  wire [`EXPONENT:0] ExpOF ;          // MSB set in sum indicates overflow
  wire [`EXPONENT:0] ExpOK ;          // MSB not set, no adjustment
  
  // Calculate normalized exponent and mantissa, check for all-zero sum
  assign MSBShift = PSSum[`DWIDTH] ;    // Check MSB in unnormalized sum
  assign ZeroSum = ~|PSSum ;      // Check for all zero sum
  assign ExpOK = CExp - Shift ;   // Adjust exponent for new normalized mantissa
  assign NegE = ExpOK[`EXPONENT] ;      // Check for exponent overflow
  assign ExpOF = CExp - Shift + 1'b1 ;    // If MSB set, add one to exponent(x2)
  assign NormE = MSBShift ? ExpOF : ExpOK ;     // Check for exponent overflow
  assign NormM = PSSum[`DWIDTH-1:`EXPONENT+1] ;   // The new, normalized mantissa
  
  // Also need to compute sticky and round bits for the rounding stage
  assign FG = PSSum[`EXPONENT] ; 
  assign R = PSSum[`EXPONENT-1] ;
  assign S = |PSSum[`EXPONENT-2:0] ;
  
endmodule

//////////////////////////////////////////////////////////////////////////////////
//
// Create Date:    11:33:28 09/11/2012 
// Module Name:    FPAddSub_RoundModule 
// Project Name:   Floating Point Project
// Author:       Fredrik Brosser
//
// Description:  Performs 'Round to nearest, tie to even'-rounding on the
//             normalized mantissa according to the G, R, S bits. Calculates
//             final result and checks for exponent overflow.
//
//////////////////////////////////////////////////////////////////////////////////

module FPAddSub_RoundModule(
    ZeroSum,
    NormE,
    NormM,
    R,
    S,
    G,
    Sa,
    Sb,
    Ctrl,
    MaxAB,
    Z,
    EOF
    );

  // Input ports
  input ZeroSum ;         // Sum is zero
  input [`EXPONENT:0] NormE ;       // Normalized exponent
  input [`MANTISSA-1:0] NormM ;       // Normalized mantissa
  input R ;             // Round bit
  input S ;             // Sticky bit
  input G ;
  input Sa ;              // A's sign bit
  input Sb ;              // B's sign bit
  input Ctrl ;            // Control bit (operation)
  input MaxAB ;
  
  // Output ports
  output [`DWIDTH-1:0] Z ;          // Final result
  output EOF ;
  
  // Internal signals
  wire [`MANTISSA:0] RoundUpM ;     // Rounded up sum with room for overflow
  wire [`MANTISSA-1:0] RoundM ;       // The final rounded sum
  wire [`EXPONENT:0] RoundE ;       // Rounded exponent (note extra bit due to poential overflow  )
  wire RoundUp ;            // Flag indicating that the sum should be rounded up
        wire FSgn;
  wire ExpAdd ;           // May have to add 1 to compensate for overflow 
  wire RoundOF ;            // Rounding overflow
  
  // The cases where we need to round upwards (= adding one) in Round to nearest, tie to even
  assign RoundUp = (G & ((R | S) | NormM[0])) ;
  
  // Note that in the other cases (rounding down), the sum is already 'rounded'
  assign RoundUpM = (NormM + 1) ;               // The sum, rounded up by 1
  assign RoundM = (RoundUp ? RoundUpM[`MANTISSA-1:0] : NormM) ;   // Compute final mantissa 
  assign RoundOF = RoundUp & RoundUpM[`MANTISSA] ;        // Check for overflow when rounding up

  // Calculate post-rounding exponent
  assign ExpAdd = (RoundOF ? 1'b1 : 1'b0) ;         // Add 1 to exponent to compensate for overflow
  assign RoundE = ZeroSum ? 5'b00000 : (NormE + ExpAdd) ;               // Final exponent

  // If zero, need to determine sign according to rounding
  assign FSgn = (ZeroSum & (Sa ^ Sb)) | (ZeroSum ? (Sa & Sb & ~Ctrl) : ((~MaxAB & Sa) | ((Ctrl ^ Sb) & (MaxAB | Sa)))) ;

  // Assign final result
  assign Z = {FSgn, RoundE[`EXPONENT-1:0], RoundM[`MANTISSA-1:0]} ;
  
  // Indicate exponent overflow
  assign EOF = RoundE[`EXPONENT];
  
endmodule

//////////////////////////////////////////////////////////////////////////////////
//
// Create Date:    13:00:02 16/11/2012 
// Module Name:    FPAddSub_ExceptionModule 
// Project Name:   Floating Point Project
// Author:       Fredrik Brosser
//
// Description:  Check the final result for exception conditions and set
//             flags accordingly.
//
//////////////////////////////////////////////////////////////////////////////////

module FPAddSub_ExceptionModule(
    Z,
    NegE,
    R,
    S,
    InputExc,
    EOF,
    P,
    Flags
    );
   
  // Input ports
  input [`DWIDTH-1:0] Z ;         // Final product
  input NegE ;            // Negative exponent?
  input R ;             // Round bit
  input S ;             // Sticky bit
  input [4:0] InputExc ;      // Exceptions in inputs A and B
  input EOF ;
  
  // Output ports
  output [`DWIDTH-1:0] P ;          // Final result
  output [4:0] Flags ;        // Exception flags
  
  // Internal signals
  wire Overflow ;         // Overflow flag
  wire Underflow ;          // Underflow flag
  wire DivideByZero ;       // Divide-by-Zero flag (always 0 in Add/Sub)
  wire Invalid ;            // Invalid inputs or result
  wire Inexact ;            // Result is inexact because of rounding
  
  // Exception flags
  
  // Result is too big to be represented
  assign Overflow = EOF | InputExc[1] | InputExc[0] ;
  
  // Result is too small to be represented
  assign Underflow = NegE & (R | S);
  
  // Infinite result computed exactly from finite operands
  assign DivideByZero = &(Z[`MANTISSA+`EXPONENT-1:`MANTISSA]) & ~|(Z[`MANTISSA+`EXPONENT-1:`MANTISSA]) & ~InputExc[1] & ~InputExc[0];
  
  // Invalid inputs or operation
  assign Invalid = |(InputExc[4:2]) ;
  
  // Inexact answer due to rounding, overflow or underflow
  assign Inexact = (R | S) | Overflow | Underflow;
  
  // Put pieces together to form final result
  assign P = Z ;
  
  // Collect exception flags  
  assign Flags = {Overflow, Underflow, DivideByZero, Invalid, Inexact} ;  
  
endmodule


//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/bfloat/FPAddSub.v
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/bfloat/FPMult_16.v
//////////////////////////////////////////////////////////////////////
//`define EXPONENT 5
//`define MANTISSA 10
`define EXPONENT 5
`define MANTISSA 10
`define ACTUAL_MANTISSA 11
`define EXPONENT_LSB 10
`define EXPONENT_MSB 14
`define MANTISSA_LSB 0
`define MANTISSA_MSB 9
`define MANTISSA_MUL_SPLIT_LSB 3
`define MANTISSA_MUL_SPLIT_MSB 9
`define SIGN 1
`define SIGN_LOC 15
`define DWIDTH (`SIGN+`EXPONENT+`MANTISSA)
`define IEEE_COMPLIANCE 1
//////////////////////////////////////////////////////////////////////////////////
//
// Create Date:    08:40:21 09/19/2012 
// Module Name:    FPMult
// Project Name:   Floating Point Project
// Author:       Fredrik Brosser
//
//////////////////////////////////////////////////////////////////////////////////

module FPMult_16(
    clk,
    rst,
                en,
    a,
    b,
    result,
                valid,
    flags
    );
  
  // Input Ports
  input clk ;             // Clock
  input rst ;
        input en;             // Reset signal
  input [`DWIDTH-1:0] a;          // Input A, a 32-bit floating point number
  input [`DWIDTH-1:0] b;          // Input B, a 32-bit floating point number
  
  // Output ports
  output valid;
  output [`DWIDTH-1:0] result ;         // Product, result of the operation, 32-bit FP number
  output [4:0] flags ;        // Flags indicating exceptions according to IEEE754
  
  // Internal signals
  wire [31:0] Z_int ;       // Product, result of the operation, 32-bit FP number
  wire [4:0] Flags_int ;      // Flags indicating exceptions according to IEEE754
  
  wire Sa ;             // A's sign
  wire Sb ;             // B's sign
  wire Sp ;             // Product sign
  wire [`EXPONENT-1:0] Ea ;         // A's exponent
  wire [`EXPONENT-1:0] Eb ;         // B's exponent
  wire [2*`MANTISSA+1:0] Mp ;         // Product mantissa
  wire [4:0] InputExc ;     // Exceptions in inputs
  wire [`MANTISSA-1:0] NormM ;        // Normalized mantissa
  wire [`EXPONENT:0] NormE ;        // Normalized exponent
  wire [`MANTISSA:0] RoundM ;       // Normalized mantissa
  wire [`EXPONENT:0] RoundE ;       // Normalized exponent
  wire [`MANTISSA:0] RoundMP ;        // Normalized mantissa
  wire [`EXPONENT:0] RoundEP ;        // Normalized exponent
  wire GRS ;

  //reg [63:0] pipe_0;      // Pipeline register Input->Prep
  reg [2*`DWIDTH-1:0] pipe_0;     // Pipeline register Input->Prep
  wire [2*`DWIDTH-1:0] stage_0;     // Pipeline register Input->Prep

  //reg [92:0] pipe_1;      // Pipeline register Prep->Execute
  reg [3*`MANTISSA+2*`EXPONENT+7:0] pipe_1;     // Pipeline register Prep->Execute
        wire [3*`MANTISSA+2*`EXPONENT+7:0] stage_1; 
  //reg [38:0] pipe_2;      // Pipeline register Execute->Normalize
  reg [`MANTISSA+`EXPONENT+7:0] pipe_2;     // Pipeline register Execute->Normalize
  reg q1_valid;
        reg q2_valid;
        reg q3_valid;
  //reg [72:0] pipe_3;      // Pipeline register Normalize->Round
  reg [2*`MANTISSA+2*`EXPONENT+10:0] pipe_3;      // Pipeline register Normalize->Round

  //reg [36:0] pipe_4;      // Pipeline register Round->Output
  reg [`DWIDTH+4:0] pipe_4;     // Pipeline register Round->Output
  
  assign result = pipe_4[`DWIDTH+4:5] ;
  assign flags = pipe_4[4:0] ;
  
        assign stage_0 = {a,b};
        assign valid = q3_valid;
  // Prepare the operands for alignment and check for exceptions
  FPMult_PrepModule PrepModule(clk, rst, stage_0[2*`DWIDTH-1:`DWIDTH], stage_0[`DWIDTH-1:0], Sa, Sb, Ea[`EXPONENT-1:0], Eb[`EXPONENT-1:0], Mp[2*`MANTISSA+1:0], InputExc[4:0]) ;
      
        assign stage_1 = {stage_0[`DWIDTH+`MANTISSA-1:`DWIDTH], stage_0[8:0], Sa, Sb, Ea[`EXPONENT-1:0], Eb[`EXPONENT-1:0], Mp[2*`MANTISSA+1:0], InputExc[4:0]};

  // Perform (unsigned) mantissa multiplication
  FPMult_ExecuteModule ExecuteModule(stage_1[3*`MANTISSA+`EXPONENT*2+7:2*`MANTISSA+2*`EXPONENT+8], stage_1[2*`MANTISSA+2*`EXPONENT+7:2*`MANTISSA+7], stage_1[2*`MANTISSA+6:5], stage_1[2*`MANTISSA+2*`EXPONENT+6:2*`MANTISSA+`EXPONENT+7], stage_1[2*`MANTISSA+`EXPONENT+6:2*`MANTISSA+7], stage_1[2*`MANTISSA+2*`EXPONENT+8], stage_1[2*`MANTISSA+2*`EXPONENT+7], Sp, NormE[`EXPONENT:0], NormM[`MANTISSA-1:0], GRS) ;

  // Round result and if necessary, perform a second (post-rounding) normalization step
  FPMult_NormalizeModule NormalizeModule(pipe_2[`MANTISSA-1:0], pipe_2[`MANTISSA+`EXPONENT:`MANTISSA], RoundE[`EXPONENT:0], RoundEP[`EXPONENT:0], RoundM[`MANTISSA:0], RoundMP[`MANTISSA:0]) ;    

  // Round result and if necessary, perform a second (post-rounding) normalization step
  //FPMult_RoundModule RoundModule(pipe_3[47:24], pipe_3[23:0], pipe_3[65:57], pipe_3[56:48], pipe_3[66], pipe_3[67], pipe_3[72:68], Z_int[31:0], Flags_int[4:0]) ;   
  FPMult_RoundModule RoundModule(pipe_3[2*`MANTISSA+1:`MANTISSA+1], pipe_3[`MANTISSA:0], pipe_3[2*`MANTISSA+2*`EXPONENT+3:2*`MANTISSA+`EXPONENT+3], pipe_3[2*`MANTISSA+`EXPONENT+2:2*`MANTISSA+2], pipe_3[2*`MANTISSA+2*`EXPONENT+4], pipe_3[2*`MANTISSA+2*`EXPONENT+5], pipe_3[2*`MANTISSA+2*`EXPONENT+10:2*`MANTISSA+2*`EXPONENT+6], Z_int[`DWIDTH-1:0], Flags_int[4:0]) ;    

  always @ (posedge clk) begin  
    if(rst) begin
      //pipe_0 = 0;
      //pipe_1 = 0;
      pipe_2 = 0; 
      pipe_3 = 0;
      pipe_4 = 0;
    end 
    else begin    
      /* PIPE 0
        [63:32] A
        [31:0] B
      */
      //pipe_0 = {a, b} ;

      /* PIPE 1
        [70] Sa
        [69] Sb
        [68:61] Ea
        [60:53] Eb
        [52:5] Mp
        [4:0] InputExc
      */
      //pipe_1 <= {pipe_0[`DWIDTH+`MANTISSA-1:`DWIDTH], pipe_0[`MANTISSA_MUL_SPLIT_LSB-1:0], Sa, Sb, Ea[`EXPONENT-1:0], Eb[`EXPONENT-1:0], Mp[2*`MANTISSA-1:0], InputExc[4:0]} ;
      //pipe_1 = {pipe_0[`DWIDTH+`MANTISSA-1:`DWIDTH], pipe_0[8:0], Sa, Sb, Ea[`EXPONENT-1:0], Eb[`EXPONENT-1:0], Mp[2*`MANTISSA+1:0], InputExc[4:0]} ;
      /* PIPE 2
        [38:34] InputExc
        [33] GRS
        [32] Sp
        [31:23] NormE
        [22:0] NormM
      */
      pipe_2 <= {stage_1[4:0], GRS, Sp, NormE[`EXPONENT:0], NormM[`MANTISSA-1:0]} ;
      /* PIPE 3
        [72:68] InputExc
        [67] GRS
        [66] Sp 
        [65:57] RoundE
        [56:48] RoundEP
        [47:24] RoundM
        [23:0] RoundMP
      */
      pipe_3 <= {pipe_2[`EXPONENT+`MANTISSA+7:`EXPONENT+`MANTISSA+1], RoundE[`EXPONENT:0], RoundEP[`EXPONENT:0], RoundM[`MANTISSA:0], RoundMP[`MANTISSA:0]} ;
      /* PIPE 4
        [36:5] Z
        [4:0] Flags
      */        
      pipe_4 <= {Z_int[`DWIDTH-1:0], Flags_int[4:0]} ;
    end
  end
always@(posedge clk)begin
  if(rst)begin
       q1_valid <= 0;
       q2_valid <= 0;
       q3_valid <= 0;
  end
  else begin
       q1_valid <= en;
       q2_valid <= q1_valid;
       q3_valid <= q2_valid;
  end
end


    
endmodule


module FPMult_PrepModule (
    clk,
    rst,
    a,
    b,
    Sa,
    Sb,
    Ea,
    Eb,
    Mp,
    InputExc
  );
  
  // Input ports
  input clk ;
  input rst ;
  input [`DWIDTH-1:0] a ;               // Input A, a 32-bit floating point number
  input [`DWIDTH-1:0] b ;               // Input B, a 32-bit floating point number
  
  // Output ports
  output Sa ;                   // A's sign
  output Sb ;                   // B's sign
  output [`EXPONENT-1:0] Ea ;               // A's exponent
  output [`EXPONENT-1:0] Eb ;               // B's exponent
  output [2*`MANTISSA+1:0] Mp ;             // Mantissa product
  output [4:0] InputExc ;           // Input numbers are exceptions
  
  // Internal signals             // If signal is high...
  wire ANaN ;                   // A is a signalling NaN
  wire BNaN ;                   // B is a signalling NaN
  wire AInf ;                   // A is infinity
  wire BInf ;                   // B is infinity
    wire [`MANTISSA-1:0] Ma;
    wire [`MANTISSA-1:0] Mb;
  
  assign ANaN = &(a[`DWIDTH-2:`MANTISSA]) &  |(a[`DWIDTH-2:`MANTISSA]) ;      // All one exponent and not all zero mantissa - NaN
  assign BNaN = &(b[`DWIDTH-2:`MANTISSA]) &  |(b[`MANTISSA-1:0]);     // All one exponent and not all zero mantissa - NaN
  assign AInf = &(a[`DWIDTH-2:`MANTISSA]) & ~|(a[`DWIDTH-2:`MANTISSA]) ;    // All one exponent and all zero mantissa - Infinity
  assign BInf = &(b[`DWIDTH-2:`MANTISSA]) & ~|(b[`DWIDTH-2:`MANTISSA]) ;    // All one exponent and all zero mantissa - Infinity
  
  // Check for any exceptions and put all flags into exception vector
  assign InputExc = {(ANaN | BNaN | AInf | BInf), ANaN, BNaN, AInf, BInf} ;
  //assign InputExc = {(ANaN | ANaN | BNaN |BNaN), ANaN, ANaN, BNaN,BNaN} ;
  
  // Take input numbers apart
  assign Sa = a[`DWIDTH-1] ;              // A's sign
  assign Sb = b[`DWIDTH-1] ;              // B's sign
  assign Ea = a[`DWIDTH-2:`MANTISSA];           // Store A's exponent in Ea, unless A is an exception
  assign Eb = b[`DWIDTH-2:`MANTISSA];           // Store B's exponent in Eb, unless B is an exception 
//    assign Ma = a[`MANTISSA_MSB:`MANTISSA_LSB];
  //  assign Mb = b[`MANTISSA_MSB:`MANTISSA_LSB];
  


  //assign Mp = ({4'b0001, a[`MANTISSA-1:0]}*{4'b0001, b[`MANTISSA-1:9]}) ;
  assign Mp = ({1'b1,a[`MANTISSA-1:0]}*{1'b1, b[`MANTISSA-1:0]}) ;

  
    //We multiply part of the mantissa here
    //Full mantissa of A
    //Bits MANTISSA_MUL_SPLIT_MSB:MANTISSA_MUL_SPLIT_LSB of B
   // wire [`ACTUAL_MANTISSA-1:0] inp_A;
   // wire [`ACTUAL_MANTISSA-1:0] inp_B;
   // assign inp_A = {1'b1, Ma};
   // assign inp_B = {{(`MANTISSA-(`MANTISSA_MUL_SPLIT_MSB-`MANTISSA_MUL_SPLIT_LSB+1)){1'b0}}, 1'b1, Mb[`MANTISSA_MUL_SPLIT_MSB:`MANTISSA_MUL_SPLIT_LSB]};
   // DW02_mult #(`ACTUAL_MANTISSA,`ACTUAL_MANTISSA) u_mult(.A(inp_A), .B(inp_B), .TC(1'b0), .PRODUCT(Mp));
endmodule


module FPMult_ExecuteModule(
    a,
    b,
    MpC,
    Ea,
    Eb,
    Sa,
    Sb,
    Sp,
    NormE,
    NormM,
    GRS
    );

  // Input ports
  input [`MANTISSA-1:0] a ;
  input [2*`EXPONENT:0] b ;
  input [2*`MANTISSA+1:0] MpC ;
  input [`EXPONENT-1:0] Ea ;            // A's exponent
  input [`EXPONENT-1:0] Eb ;            // B's exponent
  input Sa ;                // A's sign
  input Sb ;                // B's sign
  
  // Output ports
  output Sp ;               // Product sign
  output [`EXPONENT:0] NormE ;                          // Normalized exponent
  output [`MANTISSA-1:0] NormM ;                        // Normalized mantissa
  output GRS ;
  
  wire [2*`MANTISSA+1:0] Mp ;
  
  assign Sp = (Sa ^ Sb) ;                       // Equal signs give a positive product
  
   // wire [`ACTUAL_MANTISSA-1:0] inp_a;
   // wire [`ACTUAL_MANTISSA-1:0] inp_b;
   // assign inp_a = {1'b1, a};
   // assign inp_b = {{(`MANTISSA-`MANTISSA_MUL_SPLIT_LSB){1'b0}}, 1'b0, b};
   // DW02_mult #(`ACTUAL_MANTISSA,`ACTUAL_MANTISSA) u_mult(.A(inp_a), .B(inp_b), .TC(1'b0), .PRODUCT(Mp_temp));
   // DW01_add #(2*`ACTUAL_MANTISSA) u_add(.A(Mp_temp), .B(MpC<<`MANTISSA_MUL_SPLIT_LSB), .CI(1'b0), .SUM(Mp), .CO());

  //assign Mp = (MpC<<(2*`EXPONENT+1)) + ({4'b0001, a[`MANTISSA-1:0]}*{1'b0, b[2*`EXPONENT:0]}) ;
  assign Mp = MpC;


  assign NormM = (Mp[2*`MANTISSA+1] ? Mp[2*`MANTISSA:`MANTISSA+1] : Mp[2*`MANTISSA-1:`MANTISSA]);   // Check for overflow
  assign NormE = (Ea + Eb + Mp[2*`MANTISSA+1]);               // If so, increment exponent
  
  assign GRS = ((Mp[`MANTISSA]&(Mp[`MANTISSA+1]))|(|Mp[`MANTISSA-1:0])) ;
  
endmodule

module FPMult_NormalizeModule(
    NormM,
    NormE,
    RoundE,
    RoundEP,
    RoundM,
    RoundMP
    );

  // Input Ports
  input [`MANTISSA-1:0] NormM ;                 // Normalized mantissa
  input [`EXPONENT:0] NormE ;                 // Normalized exponent

  // Output Ports
  output [`EXPONENT:0] RoundE ;
  output [`EXPONENT:0] RoundEP ;
  output [`MANTISSA:0] RoundM ;
  output [`MANTISSA:0] RoundMP ; 
  
  assign RoundE = NormE - 15 ;
  assign RoundEP = NormE - 14 ;
  assign RoundM = NormM ;
  assign RoundMP = NormM ;

endmodule

module FPMult_RoundModule(
    RoundM,
    RoundMP,
    RoundE,
    RoundEP,
    Sp,
    GRS,
    InputExc,
    Z,
    Flags
    );

  // Input Ports
  input [`MANTISSA:0] RoundM ;                  // Normalized mantissa
  input [`MANTISSA:0] RoundMP ;                 // Normalized exponent
  input [`EXPONENT:0] RoundE ;                  // Normalized mantissa + 1
  input [`EXPONENT:0] RoundEP ;                 // Normalized exponent + 1
  input Sp ;                        // Product sign
  input GRS ;
  input [4:0] InputExc ;
  
  // Output Ports
  output [`DWIDTH-1:0] Z ;                    // Final product
  output [4:0] Flags ;
  
  // Internal Signals
  wire [`EXPONENT:0] FinalE ;                 // Rounded exponent
  wire [`MANTISSA:0] FinalM;
  wire [`MANTISSA:0] PreShiftM;
  
  assign PreShiftM = GRS ? RoundMP : RoundM ; // Round up if R and (G or S)
  
  // Post rounding normalization (potential one bit shift> use shifted mantissa if there is overflow)
  assign FinalM = (PreShiftM[`MANTISSA] ? {1'b0, PreShiftM[`MANTISSA:1]} : PreShiftM[`MANTISSA:0]) ;
  
  assign FinalE = (PreShiftM[`MANTISSA] ? RoundEP : RoundE) ; // Increment exponent if a shift was done
  
  assign Z = {Sp, FinalE[`EXPONENT-1:0], FinalM[`MANTISSA-1:0]} ;   // Putting the pieces together
  assign Flags = InputExc[4:0];

endmodule



//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/bfloat/FPMult_16.v
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/bfloat/permute.v
//////////////////////////////////////////////////////////////////////
module permute #(
  parameter WIDTH = 16,
  parameter NUMSTAGES = 8,
  parameter REGIDWIDTH = 8,
  parameter LOGNUMSTAGES = $clog2(NUMSTAGES)
)(
 input clk,
 input resetn,
 input read,
 input en,
 input row_col_op,
 input [NUMSTAGES*LOGNUMSTAGES-1:0] row_col_num,

 input load,
 input [NUMSTAGES * WIDTH-1:0]a,

 input [REGIDWIDTH-1:0] in_dst,
 input in_dst_we,
 input [NUMSTAGES-1:0] in_dst_mask,

 output reg [REGIDWIDTH-1:0] out_dst,
 output reg [NUMSTAGES-1:0] out_dst_mask,
 output reg out_dst_we,
 output reg [NUMSTAGES * WIDTH-1 : 0] out
);

parameter READ_STATE = 2'b00;
parameter ROWOP_STATE = 2'b01;
parameter COLOP_STATE = 2'b10;
parameter WRITE_STATE = 2'b11;

wire [NUMSTAGES * NUMSTAGES * WIDTH - 1:0] data;

reg [NUMSTAGES * LOGNUMSTAGES-1:0 ] rowsel;
reg [NUMSTAGES * LOGNUMSTAGES-1:0 ] colsel;

reg [LOGNUMSTAGES:0] count;
reg [31:0] i;

reg [1:0] p_state, n_state;
reg opsel;
reg [NUMSTAGES-1:0] load_en;
reg shuffle;
reg [NUMSTAGES * NUMSTAGES * WIDTH -1: 0] data_in;

always@(posedge clk)begin
  if(!resetn)begin
    out_dst <= 0;
    out_dst_we <= 0;
    out_dst_mask <= 0;
  end
  else begin
    out_dst <= in_dst;
    out_dst_we <= in_dst_we;
    out_dst_mask <= in_dst_mask;
  end
end


genvar g_i,g_j;

generate
  for(g_i = 0; g_i < NUMSTAGES; g_i = g_i + 1)begin
    for(g_j = 0; g_j < NUMSTAGES; g_j = g_j + 1)begin
       permute_elm #(.WIDTH(WIDTH), .MUXWIDTH(NUMSTAGES), .MUXSEL(LOGNUMSTAGES)) inst_elm(
         .clk(clk),
         .resetn(resetn),
         .row_sel(rowsel[g_i * LOGNUMSTAGES +: LOGNUMSTAGES]),
         .col_sel(colsel[g_j * LOGNUMSTAGES +: LOGNUMSTAGES]),
         .row_col_op(opsel),
         .in_row_data(data[((g_i+1) * NUMSTAGES * WIDTH)-1 : g_i * (NUMSTAGES * WIDTH)]),
         .in_col_data({data[(( 0 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ],
                       data[(( 1 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ],
                       data[(( 2 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ],
                       data[(( 3 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ],
                       data[(( 4 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ],
                       data[(( 5 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ],
                       data[(( 6 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ],
                       data[(( 7 * NUMSTAGES * WIDTH) + g_j * WIDTH) +: WIDTH ]
                      }),
         .en(shuffle),
         .load(load_en[g_i]),
         .data_in(data_in[g_i * (NUMSTAGES * WIDTH) + g_j * WIDTH +: WIDTH]),
         .out_data(data[(g_i * NUMSTAGES * WIDTH) + (g_j * WIDTH) +: WIDTH])
       );
    end
  end
endgenerate

always@(*) begin
  data_in = 'h0;
  case(count)
    4'h0:data_in[1 * (NUMSTAGES * WIDTH)-1: 0 * (NUMSTAGES*WIDTH)] = a;
    4'h1:data_in[2 * (NUMSTAGES * WIDTH)-1: 1 * (NUMSTAGES*WIDTH)] = a;
    4'h2:data_in[3 * (NUMSTAGES * WIDTH)-1: 2 * (NUMSTAGES*WIDTH)] = a;
    4'h3:data_in[4 * (NUMSTAGES * WIDTH)-1: 3 * (NUMSTAGES*WIDTH)] = a;
    4'h4:data_in[5 * (NUMSTAGES * WIDTH)-1: 4 * (NUMSTAGES*WIDTH)] = a;
    4'h5:data_in[6 * (NUMSTAGES * WIDTH)-1: 5 * (NUMSTAGES*WIDTH)] = a;
    4'h6:data_in[7 * (NUMSTAGES * WIDTH)-1: 6 * (NUMSTAGES*WIDTH)] = a;
    4'h7:data_in[8 * (NUMSTAGES * WIDTH)-1: 7 * (NUMSTAGES*WIDTH)] = a;
  endcase
end

always@(*) begin
  out = 0;
  case(count)
    4'h1: out = data[8*(NUMSTAGES * WIDTH) - 1 : 7 * (NUMSTAGES* WIDTH)];
    4'h2: out = data[7*(NUMSTAGES * WIDTH) - 1 : 6 * (NUMSTAGES* WIDTH)];
    4'h3: out = data[6*(NUMSTAGES * WIDTH) - 1 : 5 * (NUMSTAGES* WIDTH)];
    4'h4: out = data[5*(NUMSTAGES * WIDTH) - 1 : 4 * (NUMSTAGES* WIDTH)];
    4'h5: out = data[4*(NUMSTAGES * WIDTH) - 1 : 3 * (NUMSTAGES* WIDTH)];
    4'h6: out = data[3*(NUMSTAGES * WIDTH) - 1 : 2 * (NUMSTAGES* WIDTH)];
    4'h7: out = data[2*(NUMSTAGES * WIDTH) - 1 : 1 * (NUMSTAGES* WIDTH)];
    4'h8: out = data[1*(NUMSTAGES * WIDTH) - 1 : 0 * (NUMSTAGES* WIDTH)];
  endcase
end


always@(posedge clk)begin
  if(!resetn)
    count <= 'h0;
  else begin
    if(en & load)begin
      count <= count + 1;
    end
    else if(en & read)
      count <= count - 1 ;
  end
end

always@(*)begin
  load_en = 8'h0;
  if(en & load)begin
      case(count)
        3'h0: load_en = 8'h1;
        3'h1: load_en = 8'h2;
        3'h2: load_en = 8'h4;
        3'h3: load_en = 8'h8;
        3'h4: load_en = 8'h10;
        3'h5: load_en = 8'h20;
        3'h6: load_en = 8'h40;
        3'h7: load_en = 8'h80;
      endcase
  end
end


always@(*)begin
    shuffle = 1'b0;
    rowsel = {3'h7,3'h6,3'h5,3'h4,3'h3,3'h2,3'h1,3'h0};
    colsel = {3'h7,3'h6,3'h5,3'h4,3'h3,3'h2,3'h1,3'h0};
    if(en & ~load & ~read)begin
        shuffle = 1'b1;
        rowsel = row_col_num; 
        colsel = row_col_num;
        if(row_col_op)
            opsel = 1'b1; 
        else
            opsel = 1'b0; 
    end
end

endmodule


module permute_elm #(
  parameter MUXWIDTH = 8,
  parameter WIDTH = 16,
  parameter MUXSEL = $clog2(MUXWIDTH)
)(
  input clk,
  input resetn,
  input [MUXSEL-1:0] row_sel,
  input [MUXSEL-1:0] col_sel,
  input row_col_op,
  input [MUXWIDTH*WIDTH-1:0] in_row_data,  
  input [MUXWIDTH*WIDTH-1:0] in_col_data,  

  input load,
  input en,
  input [WIDTH-1:0] data_in,

  output reg [WIDTH-1:0] out_data
);

reg [WIDTH-1:0] row_data;
reg [WIDTH-1:0] col_data;
reg [WIDTH-1:0] n_data;

always@(*)begin
  case(row_sel)
    3'h0:begin row_data = in_row_data[1 * WIDTH -1: 0 * WIDTH]; end
    3'h1:begin row_data = in_row_data[2 * WIDTH -1: 1 * WIDTH]; end
    3'h2:begin row_data = in_row_data[3 * WIDTH -1: 2 * WIDTH]; end
    3'h3:begin row_data = in_row_data[4 * WIDTH -1: 3 * WIDTH]; end
    3'h4:begin row_data = in_row_data[5 * WIDTH -1: 4 * WIDTH]; end
    3'h5:begin row_data = in_row_data[6 * WIDTH -1: 5 * WIDTH]; end
    3'h6:begin row_data = in_row_data[7 * WIDTH -1: 6 * WIDTH]; end
    3'h7:begin row_data = in_row_data[8 * WIDTH -1: 7 * WIDTH]; end
  endcase
  case(col_sel)
    3'h0:begin col_data = in_col_data[1 * WIDTH -1: 0 * WIDTH]; end
    3'h1:begin col_data = in_col_data[2 * WIDTH -1: 1 * WIDTH]; end
    3'h2:begin col_data = in_col_data[3 * WIDTH -1: 2 * WIDTH]; end
    3'h3:begin col_data = in_col_data[4 * WIDTH -1: 3 * WIDTH]; end
    3'h4:begin col_data = in_col_data[5 * WIDTH -1: 4 * WIDTH]; end
    3'h5:begin col_data = in_col_data[6 * WIDTH -1: 5 * WIDTH]; end
    3'h6:begin col_data = in_col_data[7 * WIDTH -1: 6 * WIDTH]; end
    3'h7:begin col_data = in_col_data[8 * WIDTH -1: 7 * WIDTH]; end
  endcase
  
  if(row_col_op)
    n_data = row_data;
  else
    n_data = col_data; 
end

always@(posedge clk)begin
  if(~resetn)
    out_data <= 'h0;
  else
   if(load)
     out_data <= data_in;
   else if(en)
     out_data <= n_data;
end

endmodule

//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/bfloat/permute.v
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/bfloat/reduction_unit.v
//////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////
//Module to reduce multiple values (add, max, min) into one final result.
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// Numerics. We use fixed point format:
//  Most significant 8 bits represent integer part and Least significant 8 bits
//  represent fraction part
//  i.e. IIIIIIIIFFFFFFFF = IIIIIIII.FFFFFFFF
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// There are 32 inputs to the reduction unit. We use a tree structure to reduce the 32 values.
// It is assumed that the number of addressses supplied (end_addr - start_addr + 1) is a multiple
// of 32. If the real application needs to reduce a number of values that are not a multiple of
// 32, then the application must pad the values in the input BRAM appropriately
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// A user is expected to use the resetn signal everytime before starting a new reduction operation.
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// Accumulation is done in 20 bits (16 + log(16))
//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
// Each entry of the RAM contains `NUM_INPUTS (which is 32) values. So,
// 1 read of the RAM provides all the inputs required for going through
// the reduction unit once. 
//////////////////////////////////////////////////////////////////


////////////////////////////////////////////
// Top module
////////////////////////////////////////////
module reduction_layer#(
 parameter DWIDTH = 16,
 parameter LOGDWIDTH = $clog2(DWIDTH)
)(
  input clk,
  input resetn, //resets the processing elements
  input en, //indicates valid reduction operation
  input  [1:0] reduction_type, //can have 3 values: 0 (Add), 1 (Max), 2 (Min)
  //input read,
  input  [8 * DWIDTH -1:0] a, // input data to reduction logic
  output [8 * DWIDTH -1:0] reduced_out, //output
  output reg done, //output is valid when done is 1
  output reg busy,
  output reg valid
);

wire [DWIDTH + LOGDWIDTH-1:0] reduced_out_unrounded;

reduction_unit ucu(
  .clk(clk),
  .resetn(resetn),
  .en(en),
  .inp0(a[1*DWIDTH-1:0*DWIDTH]), 
  .inp1(a[2*DWIDTH-1:1*DWIDTH]), 
  .inp2(a[3*DWIDTH-1:2*DWIDTH]), 
  .inp3(a[4*DWIDTH-1:3*DWIDTH]), 
  .inp4(a[5*DWIDTH-1:4*DWIDTH]), 
  .inp5(a[6*DWIDTH-1:5*DWIDTH]), 
  .inp6(a[7*DWIDTH-1:6*DWIDTH]), 
  .inp7(a[8*DWIDTH-1:7*DWIDTH]), 
  .mode(reduction_type),
  .outp(reduced_out_unrounded)
);
defparam
    ucu.DWIDTH = DWIDTH,
    ucu.LOGDWIDTH = LOGDWIDTH;

////////////////////////////////////////////////////////////////
// Rounding of the output of reduction unit (from 20 bits to 16 bits).
// This is required only when reduction type is "sum"
////////////////////////////////////////////////////////////////
rounding #(DWIDTH+LOGDWIDTH, DWIDTH) u_round(.i_data(reduced_out_unrounded), .o_data(reduced_out_add));

assign reduced_out_1 = (reduction_type==2'b0) ? reduced_out_add : reduced_out_unrounded[DWIDTH-1:0];
//assign reduced_out = {8{reduced_out_1}};
assign reduced_out = {8{reduced_out_unrounded[DWIDTH-1:0]}};

reg[2:0] count;

always@(posedge clk)begin
  if(!resetn)begin
    count <= 3'h0;
    valid <= 1'b0;
  end
  else begin
    if(en)
        count <= 3'h4;
    if( ~en & (count != 3'h0))
        count <= count - 1;
    if(~en & (count == 3'h2))
        valid <= 1'b1;
    else
        valid <= 1'b0;
  end
end

always@(*)begin
  if(count != 0)begin
    busy = 1'b1;
  end
  else begin
    busy = 1'b0;
  end
end

always@(posedge clk)begin
  if(!resetn)begin
    done <= 1'b0;
  end
  else begin
    if(count == 8)
      done <= 1'b1;
    else if(count ==0)
      done <= 1'b0;
  end
end

endmodule


///////////////////////////////////////////////////////
// Processing element. Finds sum, min or max depending on mode
///////////////////////////////////////////////////////
module reduction_processing_element #(
  parameter IN_DWIDTH = 16,
  parameter OUT_DWIDTH = 4
)(
  A, B, OUT, MODE
);

input [IN_DWIDTH-1:0] A;
input [IN_DWIDTH-1:0] B;
output [OUT_DWIDTH-1:0] OUT;
input [1:0] MODE;

wire [OUT_DWIDTH-1:0] greater;
wire [OUT_DWIDTH-1:0] smaller;
wire [OUT_DWIDTH-1:0] sum;

assign greater = (A>B) ? A : B;
assign smaller = (A<B) ? A : B;
assign sum = A + B;

assign OUT = (MODE==0) ? sum : 
             (MODE==1) ? greater :
             smaller;

endmodule


///////////////////////////////////////////////////////
// Reduction unit. It's a tree of processing elements.
// There are 32 inputs and one output and 6 stages. 
//
// The output is
// wider (more bits) than the inputs. It has logN more
// bits (if N is the number of bits in the inputs). This
// is based on https://zipcpu.com/dsp/2017/07/22/rounding.html.
// 
// The last stage is special. It adds the previous 
// result. This is useful when we have more than 32 inputs
// to reduce. We send the next set of 32 inputs in the next
// clock after the first set. 
// 
// Each stage of the tree is pipelined.
///////////////////////////////////////////////////////
module reduction_unit(
  clk,
  resetn,
  en,
  inp0, 
  inp1, 
  inp2, 
  inp3, 
  inp4, 
  inp5, 
  inp6, 
  inp7, 

  mode,
  outp
);
  parameter DWIDTH = 16;
  parameter LOGDWIDTH = 4;

  input clk;
  input resetn;
  input en;
  input  [DWIDTH-1 : 0] inp0; 
  input  [DWIDTH-1 : 0] inp1; 
  input  [DWIDTH-1 : 0] inp2; 
  input  [DWIDTH-1 : 0] inp3; 
  input  [DWIDTH-1 : 0] inp4; 
  input  [DWIDTH-1 : 0] inp5; 
  input  [DWIDTH-1 : 0] inp6; 
  input  [DWIDTH-1 : 0] inp7; 

  input [1:0] mode;
  output [DWIDTH+LOGDWIDTH-1 : 0] outp;

  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute0_out_stage3;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] compute0_out_stage3_reg;
  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute1_out_stage3;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] compute1_out_stage3_reg;
  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute2_out_stage3;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] compute2_out_stage3_reg;
  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute3_out_stage3;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] compute3_out_stage3_reg;

  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute0_out_stage2;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] compute0_out_stage2_reg;
  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute1_out_stage2;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] compute1_out_stage2_reg;

  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute0_out_stage1;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] compute0_out_stage1_reg;

  wire   [DWIDTH+LOGDWIDTH-1 : 0] compute0_out_stage0;
  reg    [DWIDTH+LOGDWIDTH-1 : 0] outp;

  always @(posedge clk) begin
    if (~resetn) begin
      outp <= 0;
      compute0_out_stage3_reg <= 0;
      compute1_out_stage3_reg <= 0;
      compute2_out_stage3_reg <= 0;
      compute3_out_stage3_reg <= 0;
      compute0_out_stage2_reg <= 0;
      compute1_out_stage2_reg <= 0;
      compute0_out_stage1_reg <= 0;
    end

    else begin
      if(en)begin
        compute0_out_stage3_reg <= compute0_out_stage3;
        compute1_out_stage3_reg <= compute1_out_stage3;
        compute2_out_stage3_reg <= compute2_out_stage3;
        compute3_out_stage3_reg <= compute3_out_stage3;
      end
      else begin
        compute0_out_stage3_reg <= 0;
        compute1_out_stage3_reg <= 0;
        compute2_out_stage3_reg <= 0;
        compute3_out_stage3_reg <= 0;
      end

      compute0_out_stage2_reg <= compute0_out_stage2;
      compute1_out_stage2_reg <= compute1_out_stage2;

      compute0_out_stage1_reg <= compute0_out_stage1;

      outp <= compute0_out_stage0;
    end
  end
  reduction_processing_element #(.IN_DWIDTH(DWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute0_stage3(.A(inp0),.B(inp1),.OUT(compute0_out_stage3), .MODE(mode));
  reduction_processing_element #(.IN_DWIDTH(DWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute1_stage3(.A(inp2),.B(inp3),.OUT(compute1_out_stage3), .MODE(mode));
  reduction_processing_element #(.IN_DWIDTH(DWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute2_stage3(.A(inp4),.B(inp5),.OUT(compute2_out_stage3), .MODE(mode));
  reduction_processing_element #(.IN_DWIDTH(DWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute3_stage3(.A(inp6),.B(inp7),.OUT(compute3_out_stage3), .MODE(mode));

  reduction_processing_element #(.IN_DWIDTH(DWIDTH+LOGDWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute0_stage2(.A(compute0_out_stage3_reg),.B(compute1_out_stage3_reg),.OUT(compute0_out_stage2), .MODE(mode));
  reduction_processing_element #(.IN_DWIDTH(DWIDTH+LOGDWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute1_stage2(.A(compute2_out_stage3_reg),.B(compute3_out_stage3_reg),.OUT(compute1_out_stage2), .MODE(mode));

  reduction_processing_element #(.IN_DWIDTH(DWIDTH+LOGDWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute0_stage1(.A(compute0_out_stage2_reg),.B(compute1_out_stage2_reg),.OUT(compute0_out_stage1), .MODE(mode));

  reduction_processing_element #(.IN_DWIDTH(DWIDTH+LOGDWIDTH),.OUT_DWIDTH(DWIDTH+LOGDWIDTH)) compute0_stage0(.A(outp),       .B(compute0_out_stage1_reg),     .OUT(compute0_out_stage0), .MODE(mode));

endmodule
///////////////////////////////////////////////////////
//Rounding logic based on convergent rounding described 
//here: https://zipcpu.com/dsp/2017/07/22/rounding.html
///////////////////////////////////////////////////////
module rounding( i_data, o_data );
parameter IWID = 32;
parameter OWID = 16;
input  [IWID-1:0] i_data;
output [OWID-1:0] o_data;

wire [IWID-1:0] w_convergent;

assign  w_convergent = i_data[(IWID-1):0]
      + { {(OWID){1'b0}},
        i_data[(IWID-OWID)],
        {(IWID-OWID-1){!i_data[(IWID-OWID)]}}};

assign o_data = w_convergent[(IWID-1):(IWID-OWID)];

endmodule

//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/bfloat/reduction_unit.v
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/bfloat/transpose.v
//////////////////////////////////////////////////////////////////////
module transpose #(
  parameter WIDTH = 16,
  parameter NUMSTAGES = 8,
  parameter LOGNUMSTAGE = $clog2(NUMSTAGES)
)(
  input clk,
  input resetn,
  input en, 
  input [NUMSTAGES * WIDTH -1:0] a,
  output reg [NUMSTAGES * WIDTH -1:0] out,
  output reg busy,
  output reg valid
);

reg [NUMSTAGES * WIDTH - 1:0] data0;
reg [NUMSTAGES * WIDTH - 1:0] data1;
reg [NUMSTAGES * WIDTH - 1:0] data2;
reg [NUMSTAGES * WIDTH - 1:0] data3;
reg [NUMSTAGES * WIDTH - 1:0] data4;
reg [NUMSTAGES * WIDTH - 1:0] data5;
reg [NUMSTAGES * WIDTH - 1:0] data6;
reg [NUMSTAGES * WIDTH - 1:0] data7;
reg [LOGNUMSTAGE:0] count;

always@(posedge clk)begin
  if(!resetn)begin
    count <= 'h0;
    valid <= 'h0;
  end
  else begin
    if(en)begin
      count <= count + 1;
      valid <= 0;
    end
    else if(count != 0)begin
      valid <= 1'b1;
      count <= count - 1;
    end
    else begin
      count <= 0;
      valid <= 0;
    end
  end
end

always@(posedge clk)begin
  if(!resetn)
     busy <= 1'b0;
  else
    if((count != 0) || (en == 1'b1))
       busy <= 1'b1;
    else
       busy <= 1'b0; 
end

always @(posedge clk)begin
  if(!resetn)begin
    data0 <= 'h0;
    data1 <= 'h0;
    data2 <= 'h0;
    data3 <= 'h0;
    data4 <= 'h0;
    data5 <= 'h0;
    data6 <= 'h0;
    data7 <= 'h0;
    out <= 'h0;
  end
  else begin
    out <= data0;
    if(en)begin
      case(count)
      3'h0:begin
               data0[0*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[0*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[0*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[0*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[0*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[0*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[0*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[0*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      3'h1:begin
               data0[1*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[1*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[1*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[1*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[1*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[1*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[1*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[1*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      3'h2:begin
               data0[2*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[2*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[2*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[2*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[2*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[2*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[2*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[2*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      3'h3:begin
               data0[3*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[3*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[3*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[3*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[3*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[3*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[3*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[3*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      3'h4:begin
               data0[4*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[4*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[4*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[4*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[4*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[4*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[4*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[4*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      3'h5:begin
               data0[5*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[5*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[5*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[5*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[5*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[5*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[5*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[5*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      3'h6:begin
               data0[6*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[6*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[6*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[6*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[6*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[6*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[6*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[6*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      3'h7:begin
               data0[7*WIDTH +: WIDTH] <= a[0*WIDTH +: WIDTH];
               data1[7*WIDTH +: WIDTH] <= a[1*WIDTH +: WIDTH];
               data2[7*WIDTH +: WIDTH] <= a[2*WIDTH +: WIDTH];
               data3[7*WIDTH +: WIDTH] <= a[3*WIDTH +: WIDTH];
               data4[7*WIDTH +: WIDTH] <= a[4*WIDTH +: WIDTH];
               data5[7*WIDTH +: WIDTH] <= a[5*WIDTH +: WIDTH];
               data6[7*WIDTH +: WIDTH] <= a[6*WIDTH +: WIDTH];
               data7[7*WIDTH +: WIDTH] <= a[7*WIDTH +: WIDTH];
           end
      endcase
    end
    else if(valid)begin
        data0 <= data1;
        data1 <= data2;
        data2 <= data3;
        data3 <= data4;
        data4 <= data5;
        data5 <= data6;
        data6 <= data7;
        data7 <= 0;
    end
    
  end  
end

endmodule

//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/bfloat/transpose.v
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/bfloat/trp_unit.v
//////////////////////////////////////////////////////////////////////
module trp_unit #(parameter WIDTH=4)(
 input clk,
 input resetn,
 input en,
 input [8*WIDTH-1:0] a,
 input [1:0] mode,
 input read,
 output busy,
 output reg valid,
 output reg[8*WIDTH-1:0] out
);

reg en_reduction;
reg en_transpose;
reg read_transpose;
reg read_reduction;
wire transpose_busy;
wire reduction_busy;
wire reduction_done;
wire [8*WIDTH-1:0] reduction_out;
wire [8*WIDTH-1:0] transpose_out;
wire reduction_valid; 
 
//assign busy = transpose_busy || reduction_busy;
assign busy =  reduction_busy;

always@(*)begin
//  if(mode == 2'b11)begin
//    en_transpose = en;
//    en_reduction = 1'b0;
//    read_transpose = read;
//    read_reduction = 1'b0;
//  end
//  else begin
//    en_transpose = 1'b0;
    en_reduction = en;
//    read_transpose = 1'b0;
//    read_reduction = read;
//  end
end

always@(*)begin
 // if(transpose_busy)begin
 //   out = transpose_out;
 //   valid = 1'b1;
 // end
 // else if(reduction_done)begin
    out = reduction_out;
    valid = reduction_valid;
 // end
 // else begin
 //   out = 'h0;
 //   valid = 1'b0;
 // end
end

reduction_layer #(
  .DWIDTH(WIDTH)
) u_reduction_layer(
  .clk(clk),
  .resetn(resetn),
  .en(en_reduction),
  .reduction_type(mode),
  .a(a),
  .reduced_out(reduction_out),
  .done(reduction_done),
  .busy(reduction_busy),
  .valid(reduction_valid)
);

//transpose u_transpose(
//  .clk(clk),
//  .resetn(resetn),
//  .read(),
//  .en(en_transpose), 
//  .a(a),
//  .out(transpose_out),
//  .busy(transpose_busy)
//);

endmodule

//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/bfloat/trp_unit.v
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/dma.v
//////////////////////////////////////////////////////////////////////
module dma #(parameter NUMLANES=8,
  parameter WIDTH = 16,
  parameter ADDRWIDTH = 8,
  parameter DMEM_WIDTH = NUMLANES * WIDTH,
  parameter DMEM_ADDRWIDTH = 32,
  parameter LOG2DMEMWIDTH= $clog2(DMEM_WIDTH)
)(

input clk,
input resetn,

input [DMEM_ADDRWIDTH-1:0] mem_addr,
input [15:0] num_bytes,
input dma_en,
input [ADDRWIDTH-1:0] lane_addr,
input we, // we = 1 means data from lane_addr will be transfered to main memory

output reg [NUMLANES*ADDRWIDTH-1:0] local_addr,
output reg [NUMLANES-1:0] local_wren,
output reg [NUMLANES-1:0] local_rden,
output reg [NUMLANES*WIDTH-1:0] local_wrdata,
 input     [NUMLANES*WIDTH-1:0] local_rddata,

output     dma_busy,
output reg [DMEM_ADDRWIDTH-1:0]  dbus_address,
 input  [DMEM_WIDTH-1:0]  dbus_readdata,
output  [DMEM_WIDTH-1:0]   dbus_writedata,
output reg [LOG2DMEMWIDTH-3-1:0]   dbus_byteen,
output reg   dbus_en,
output reg   dbus_wren,
output reg   dbus_prefetch,
 input    dbus_wait,   //Goes high 1-cycle after dbus_en
 input    dbus_data_valid
);

parameter IDLE_STATE = 3'b000;
parameter LOAD_REQ_STATE = 3'b001;
parameter STORE_REQ_STATE = 3'b010;
parameter READ_STATE = 3'b011;
parameter LOAD_DATA_STATE = 3'b100;
parameter WRITE_STATE = 3'b101;
parameter UPDATE_RD_COUNT = 3'b110;
parameter UPDATE_WR_COUNT = 3'b111;

reg [15:0] count, next_count;
reg [2:0] pstate,nstate;
reg [NUMLANES*WIDTH-1:0] data_to_dbus, data_to_mem;
reg[31:0]i,j,k;
always@(posedge clk)begin
  if(!resetn)begin
    data_to_dbus <= 0;
    data_to_mem <= 0;
  end
  else begin 
    if(pstate == STORE_REQ_STATE)
      data_to_dbus <= local_rddata;
    if(pstate == LOAD_DATA_STATE)
      for(j=0;j<DMEM_WIDTH/32;j=j+1)
        for(k=0;k<2;k=k+1)
          data_to_mem[(1-k)*16+j*32+:16] <= dbus_readdata[k*16+j*32+:16]; 
  end
end

assign dbus_writedata = (pstate == STORE_REQ_STATE)? local_rddata : data_to_dbus;

always@(posedge clk )begin
  if(!resetn)begin
    count <= 0;
    pstate <= IDLE_STATE;
  end 
  else begin
    count <= next_count;
    pstate <= nstate;
  end 
end

assign dma_busy = (pstate == IDLE_STATE)? 1'b0:1'b1; 

always@(*)begin
  case(pstate)
    IDLE_STATE:begin
                 if((!we) & dma_en )        
                   nstate  = LOAD_REQ_STATE;
                 else  if(we & dma_en)
                   nstate = STORE_REQ_STATE;
                 else
                   nstate = IDLE_STATE;
               end
    LOAD_REQ_STATE:begin
                     if(count != num_bytes)
                       nstate = READ_STATE;
                     else
                       nstate = IDLE_STATE;
                   end
    STORE_REQ_STATE:begin
                     if(count != num_bytes)
                       nstate = WRITE_STATE;
                     else
                       nstate = IDLE_STATE;
                   end
    READ_STATE:begin
                 if(!dbus_wait)
                   nstate = LOAD_DATA_STATE;
                 else
                   nstate = READ_STATE;
               end
    LOAD_DATA_STATE:begin
                        nstate = UPDATE_RD_COUNT;
                    end
    WRITE_STATE:begin
                 if(dbus_wait)
                   nstate = WRITE_STATE;
                 else
                   nstate = UPDATE_WR_COUNT;
               end
    UPDATE_WR_COUNT:begin
                        nstate = STORE_REQ_STATE;
                    end
    UPDATE_RD_COUNT:begin
                        nstate = LOAD_REQ_STATE;
                    end
  endcase
end

always@(*)begin
  dbus_prefetch = 0;
  case(pstate)
    IDLE_STATE: begin
                   dbus_address =0;
                   dbus_byteen = 0;
                   dbus_en = 0; 
                   dbus_wren = 0;
                   next_count = 0;
                   local_wren = 0;
                   local_rden = 0;
                end 
    LOAD_REQ_STATE:begin
                     dbus_address = mem_addr + count;
                     dbus_byteen = 0;
                     dbus_en = 1; 
                     dbus_wren = 0;
                     next_count = count;
                   end
    STORE_REQ_STATE:begin
                     for(i=0; i<NUMLANES;i=i+1)begin
                       local_addr[i*ADDRWIDTH +: ADDRWIDTH] =  lane_addr  + count + i;
                       local_wren[i] = 1'b0;
                       local_rden[i] = 1'b1;
                     end
                     dbus_address = mem_addr + count;
                     dbus_byteen = 16'hffff;
                     dbus_en = 1; 
                     dbus_wren = 1;
                     next_count = count ;
                   end
    READ_STATE:begin
                     dbus_byteen = 0;
                     dbus_en = 0; 
                     dbus_wren = 0;
                     next_count = count;
               end
    LOAD_DATA_STATE:begin
                     dbus_byteen = 0;
                     dbus_en = 0; 
                     dbus_wren = 0;
                     next_count = count;
                    end
    WRITE_STATE:begin
                       dbus_address = mem_addr + count;
                       dbus_byteen = {NUMLANES{2'b11}};
                       dbus_en = 0; 
                       dbus_wren = 0;
                       next_count = count;
               end
    UPDATE_WR_COUNT:begin
                       dbus_address = mem_addr + count;
                       dbus_byteen = {NUMLANES{2'b11}};
                       dbus_en = 0; 
                       dbus_wren = 0;
                       next_count = count;
                       next_count = count + (NUMLANES*WIDTH/8);
                     end
    UPDATE_RD_COUNT: begin
                       for(i=0; i<NUMLANES;i=i+1)begin
                         local_addr[i*ADDRWIDTH +: ADDRWIDTH] =  lane_addr  + count + i;
                         local_wren[i] = 1'b1;
                         local_rden[i] = 1'b0;
                       end
                       local_wrdata = data_to_mem; 
                       dbus_address = mem_addr + count;
                       dbus_byteen = {NUMLANES{2'b00}};
                       dbus_en = 0; 
                       dbus_wren = 0;
                       next_count = count + (NUMLANES*WIDTH/8);
                end
  endcase
end

endmodule

//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/dma.v
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vmem_local.v
//////////////////////////////////////////////////////////////////////
module vmem_local (
clk,
resetn,
en,

op,
address_a,
stride_val_a,
offset_a,
data_a,
out_a,
last_subvector,

address_b,
rden_b,
wren_b,
data_b,
out_b
);


parameter NUMLANES = 8;
parameter DATAWORDSIZE = 16;
parameter VCWIDTH = 32;  
parameter MEMDEPTH = 2048; 
parameter LOGMEMDEPTH = $clog2(MEMDEPTH);

parameter MEM_IDLE = 1'b0;
parameter MEM_STRIDE = 1'b1;

input clk;
input resetn;
input [LOGMEMDEPTH-1:0] address_a;
input [NUMLANES*LOGMEMDEPTH-1:0] address_b;
input en;
input [6:0] op;
input rden_b;
input wren_b;
input last_subvector;
input [NUMLANES*DATAWORDSIZE-1:0] data_a;
input [NUMLANES*DATAWORDSIZE-1:0] data_b;
output [NUMLANES*DATAWORDSIZE-1:0] out_a;
output [NUMLANES*DATAWORDSIZE-1:0] out_b;
input [VCWIDTH-1:0] stride_val_a;
input [NUMLANES*16-1:0] offset_a;

wire stride_req_a;
wire [VCWIDTH-1:0] stride;
wire index_req_a;
reg wren_a,rden_a; 
reg req_we_a;
reg req_we_b;
reg [NUMLANES*LOGMEMDEPTH-1:0] reg_address_a;
reg [NUMLANES*LOGMEMDEPTH-1:0] reg_address_b;

wire  [1:0]  op_pattern;       // 0-Unit, 1-Strided, 2-Indexed
wire  [1:0]  op_size;          // 0-byte, 1-16bits, 2-32bits, 3-64bits
wire         op_signed;        // 0-unsigned, 1-signed
wire         op_we;            // 0-load, 1-store
wire         op_memop;         // 1-memory op, 0-vector shift op
genvar  g_mem;
reg [31:0] i;

reg [NUMLANES-1:0] g_en;

assign {op_memop,op_pattern,op_size,op_signed,op_we}=op;
//assign rden_a = (op_memop & (~op_we))? 1'b1:1'b0;
//assign wren_a = (op_memop & op_we)? 1'b1:1'b0;
assign stride = (op_pattern == 2'b01)? stride_val_a: 1;
assign stride_req_a = (~op_pattern[1] & op_memop)? 1'b1:1'b0;
assign index_req_a =  (op_pattern[1] & op_memop)? 1'b1: 1'b0;

//Lane 0 Memory address space : 0,8,16,24,...,(MEMDEPTH*8-1)
//Lane 1 Memory address space : 0+1,8+1,16+1,24+1,...,(MEMDEPTH*8-1+1)

reg last_subvector_q;

always@(posedge clk)begin 
  if(!resetn)
    last_subvector_q <= 1'b0;
  else
    last_subvector_q <= last_subvector;
end

wire mem_op_valid;
assign mem_op_valid = ~(last_subvector & last_subvector_q); 

always@(*)begin
  if(op_memop & (~op_we))
    rden_a = 1'b1;
  else
    rden_a = 1'b0;

  if(op_memop & op_we & mem_op_valid)
    wren_a = 1'b1;
  else
    wren_a = 1'b0;

  for(i=0; i< NUMLANES; i=i+1)begin
    if(i[2:0] == address_a[2:0])
      g_en[i] = 1'b1;
    else
      g_en[i] = 1'b0;
    
    reg_address_a[i*LOGMEMDEPTH +: LOGMEMDEPTH] = address_a + i * stride;
    reg_address_b[i*LOGMEMDEPTH +: LOGMEMDEPTH] = address_b[i*LOGMEMDEPTH +: LOGMEMDEPTH];
    if(index_req_a)begin
        reg_address_a[i*LOGMEMDEPTH +: LOGMEMDEPTH] = address_a + offset_a[i*16+:16];
    end
  end
end

generate 
    for(g_mem =0; g_mem < NUMLANES ; g_mem = g_mem+1 )begin:gen_memperlane
        per_lane_mem_wrapper #(.AWIDTH(LOGMEMDEPTH),.NUM_WORDS(MEMDEPTH), .DWIDTH(DATAWORDSIZE)) inst_mem(
      .clk(clk),
            .resetn(resetn),
      .address_a(reg_address_a[g_mem*LOGMEMDEPTH +: LOGMEMDEPTH]),
      .address_b(reg_address_b[g_mem*LOGMEMDEPTH +: LOGMEMDEPTH]),
            .rden_a(rden_a & en),
            .rden_b(rden_b),
      .wren_a(wren_a & en),
      .wren_b(wren_b),
      .data_a(data_a[g_mem*DATAWORDSIZE +: DATAWORDSIZE]),
      .data_b(data_b[g_mem*DATAWORDSIZE +: DATAWORDSIZE]),
      .out_a(out_a[g_mem*DATAWORDSIZE +: DATAWORDSIZE]),
      .out_b(out_b[g_mem*DATAWORDSIZE +: DATAWORDSIZE])
      //.out_b()
        );
    end
endgenerate

endmodule

//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/vmem_local.v
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/dma_axi_mux.v
//////////////////////////////////////////////////////////////////////
module dma_axi_mux #(
  parameter ADDRWIDTH = 11,
  parameter NUMLANES = 8,
  parameter WIDTH = 16
)(
  input [NUMLANES*ADDRWIDTH-1:0]   dma_addr,
  input [NUMLANES*WIDTH-1:0]       dma_data,
  input [NUMLANES-1:0]             dma_rden,
  input  [NUMLANES-1:0]            dma_wren,    
  
  output reg [NUMLANES*WIDTH-1:0]  dma_out,   
 
  input [ADDRWIDTH-1:0]            axi_addr,   
  input [NUMLANES*WIDTH-1:0]       axi_data,    
  input                            axi_req_en,  
  input                            axi_req_type,
  
  output reg [NUMLANES*WIDTH-1:0]  axi_read_data,
  
  output reg [NUMLANES*ADDRWIDTH-1:0]       mem_addr,    
  output reg [NUMLANES*WIDTH-1:0]  mem_data,     
  output reg [NUMLANES-1:0]        mem_rden,     
  output reg [NUMLANES-1:0]        mem_wren,     
  input [NUMLANES*WIDTH-1:0]  mem_readdata    
);

reg [NUMLANES-1:0] axi_rden, axi_wren;
reg [NUMLANES*ADDRWIDTH-1:0] axi_lane_addr;

always@(*)begin
  axi_lane_addr[(1*ADDRWIDTH)-1 : 0*ADDRWIDTH ] = axi_addr + 11'h0;
  axi_lane_addr[2*ADDRWIDTH-1:1*ADDRWIDTH] = axi_addr + 11'h1;
  axi_lane_addr[3*ADDRWIDTH-1:2*ADDRWIDTH] = axi_addr + 11'h2;
  axi_lane_addr[4*ADDRWIDTH-1:3*ADDRWIDTH] = axi_addr + 11'h3;
  axi_lane_addr[5*ADDRWIDTH-1:4*ADDRWIDTH] = axi_addr + 11'h4;
  axi_lane_addr[6*ADDRWIDTH-1:5*ADDRWIDTH] = axi_addr + 11'h5;
  axi_lane_addr[7*ADDRWIDTH-1:6*ADDRWIDTH] = axi_addr + 11'h6;
  axi_lane_addr[8*ADDRWIDTH-1:7*ADDRWIDTH] = axi_addr + 11'h7;
end

always@(*)begin
  axi_rden = 0;
  axi_wren = 0;
  if(axi_req_en)begin
    if(axi_req_type)begin
      axi_wren = {NUMLANES{1'b1}};
    end
    else begin
      axi_rden = {NUMLANES{1'b1}};
    end
  end
end

always@(*)begin
   if(axi_req_en)begin
     mem_addr = axi_lane_addr;
     mem_data = axi_data;
     mem_rden = axi_rden;
     mem_wren = axi_wren;
     dma_out  = mem_readdata;
     axi_read_data = 0;
   end
   else begin
     mem_addr = dma_addr; 
     mem_data = dma_data;
     mem_rden = dma_rden;
     mem_wren = dma_wren;
     dma_out  = mem_readdata;
     axi_read_data = 0;
   end
end
endmodule


//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/dma_axi_mux.v
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/interface/axi4_slave.v
//////////////////////////////////////////////////////////////////////
module axi4_slave #
(
    
    parameter integer P_ADDR_WIDTH   = 16,                                      //Width of Address Bus
    parameter integer P_DATA_WIDTH   = 128                                      //Width of Data Bus
)
(
    output  reg [P_ADDR_WIDTH - 1:0] req_addr,
    output  reg [P_DATA_WIDTH - 1:0] req_data,
    output  reg req_en,
    output  reg     req_type,
    output axi_busy,
    input   [P_DATA_WIDTH - 1:0]     req_read_data,

    input  wire CLOCK,                                                          //Global Clock Signal.
    input  wire RESET,                                                          //Global Reset Signal. This Signal is Active Low

    input wire [6 - 1:0] AWID,                                        //Master Interface Write Address ID
    input wire [P_ADDR_WIDTH - 1:0] AWADDR,                                    //Master Interface Write Address
    input wire [7:0] AWLEN,                                                    //Burst length. The burst length gives the exact number of transfers in a burst
    input wire [2:0] AWSIZE,                                                   //Burst size. This signal indicates the size of each transfer in the burst
    input wire [1:0] AWBURST,                                                  //Burst type. The burst type and the size information, determine how the address for each transfer within the burst is calculated.
    input wire AWLOCK,                                                         //Lock type. Provides additional information about the atomic characteristics of the transfer.
    input wire [3:0] AWCACHE,                                                  //Memory type. This signal indicates how transactions are required to progress through a system.
    input wire [2:0] AWPROT,                                                   //Protection type. This signal indicates the privilege and security level of the transaction, and whether  the transaction is a data access or an instruction access.
    input wire [3:0] AWQOS,                                                    //Quality of Service, QoS identifier sent for each write transaction.
    input wire AWVALID,                                                        //Write address valid. This signal indicates that the channel is signaling valid write address and control information.
    output  wire AWREADY,                                                        //Write address ready. This signal indicates that the slave is ready to accept an address and associated control signals
    input wire [P_DATA_WIDTH-1 : 0] WDATA,                                     //Master Interface Write Data.
    input wire [P_DATA_WIDTH/8-1 : 0] WSTRB,                                   //Write strobes. This signal indicates which byte lanes hold valid data. There is one write strobe bit for each eight bits of the write data bus.
    input wire WLAST,                                                          //Write last. This signal indicates the last transfer in a write burst.
    input wire WVALID,                                                         //Write valid. This signal indicates that valid write data and strobes are available
    output  wire WREADY,                                                         //Write ready. This signal indicates that the slave can accept the write data.

    input wire [6-1 : 0] ARID,                                        //Master Interface Read Address.
    input wire [P_ADDR_WIDTH-1 : 0] ARADDR,                                    //Read address. This signal indicates the initial address of a read burst transaction.
    input wire [7 : 0] ARLEN,                                                  //Burst length. The burst length gives the exact number of transfers in a burst
    input wire [2 : 0] ARSIZE,                                                 //Burst size. This signal indicates the size of each transfer in the burst
    input wire [1 : 0] ARBURST,                                                //Burst type. The burst type and the size information, determine how the address for each transfer within the burst is calculated.
    input wire ARLOCK,                                                         //Lock type. Provides additional information about the atomic characteristics of the transfer.
    input wire [3 : 0] ARCACHE,                                                //Memory type. This signal indicates how transactions are required to progress through a system.
    input wire [2 : 0] ARPROT,                                                 //Protection type. This signal indicates the privilege and security level of the transaction, and whether the transaction is a data access or an instruction access.
    input wire [3 : 0] ARQOS,                                                  //Quality of Service, QoS identifier sent for each read transaction
    input wire ARVALID,                                                        //Write address valid. This signal indicates that the channel is signaling valid read address and control information
    output  wire ARREADY,                                                        //Read address ready. This signal indicates that the slave is ready to accept an address and associated control signals
    output  wire [6-1 : 0] RID,                                         //Read ID tag. This signal is the identification tag for the read data group of signals generated by the slave.
    output  wire [P_DATA_WIDTH-1 : 0] RDATA,                                     //Master Read Data
    output  wire [1 : 0] RRESP,                                                  //Read response. This signal indicates the status of the read transfer
    output  wire RLAST,                                                          //Read last. This signal indicates the last transfer in a read burst
    output  wire RVALID,                                                         //Read valid. This signal indicates that the channel is signaling the required read data.
    input wire RREADY,                                                         //Read ready. This signal indicates that the master can accept the read data and response information.

    input wire BREADY,                                                         //Response ready. This signal indicates that the master can accept a write response.
    output  wire BVALID,                                                         //Write response valid. This signal indicates that the channel is signaling a valid write response.
    output  wire [1 : 0] BRESP,                                                  //Write response. This signal indicates the status of the write transaction.
    output  wire [6-1 : 0] BID                                          //Master Interface Write Response.
);

localparam integer C_MASTER_LENGTH = 12;

reg [2:0] slave_state;
localparam [2:0] IDLE = 0;
localparam [2:0] RDREQ_STATE  = 1;
localparam [2:0] WRREQ_STATE  = 2;
localparam [2:0] WRDATA_STATE = 3;
localparam [2:0] RDATA_STATE = 4;

reg [P_ADDR_WIDTH - 1:0] q_req_addr;

//AXI4 internal tmp signals
reg [6 - 1:0]   s_rid = 0;
wire [P_DATA_WIDTH - 1:0] s_rdata;
reg s_rlast;
reg s_rvalid;

reg s_bvalid;                                                         //Write response valid. This signal indicates that the channel is signaling a valid write response.
                                           //Master Interface Write Response.
reg [31:0] m_writer_counter = 0;
reg [2:0] count;
reg q_wrreq_valid;


wire s_awready;
wire s_arready;
wire s_wready;

assign AWREADY = s_awready;
assign WREADY = s_wready;
assign ARREADY = s_arready;

assign RID = 0;                                         //Read ID tag. This signal is the identification tag for the read data group of signals generated by the slave.
assign RDATA = s_rdata;                                     //Master Read Data
assign RRESP = 0;                                                  //Read response. This signal indicates the status of the read transfer
assign RLAST = s_rlast;                                                          //Read last. This signal indicates the last transfer in a read burst
assign RVALID = s_rvalid;                                                        //Read ready. This signal indicates that the master can accept the read data and response information.

assign BID = 0;
assign BRESP = 0;
assign BVALID = s_bvalid;

//Read and Read Response (R)
//assign RREADY = m_rready;

//Write Address Channel
//Write Data Generator.Data pattern is only a simple incrementing count from 0 for each burst.
//always @(posedge CLOCK) begin
//    if (RESET == 0) begin
//        s_rdata <= 0;
//    end else if (((slave_state == IDLE) & (ARVALID))|(slave_state == RDATA_STATE)) begin
//        s_rdata <= req_read_data;
//    end else  begin
//        s_rdata <= 0;
//    end
//end

assign s_rdata = req_read_data;
assign s_awready = 1'b1;
assign s_arready = 1'b1;
assign s_wready = 1'b1;

//Write Response (B) Channel

//Need this to silent VIP warning
always @(posedge CLOCK) begin
    if (RESET == 0) begin
        s_bvalid <= 0;
    end
    else if((slave_state == WRDATA_STATE))begin
        s_bvalid <= 1'b1;
    end
    else begin
        s_bvalid <= 0;
    end
end

always @(posedge CLOCK)begin
  if(!RESET)begin
    s_rvalid <= 0;
  end else begin
     if((slave_state == IDLE) & ARVALID)
       s_rvalid <= 1'b1;
     else if((slave_state == RDATA_STATE) & (count==7))
       s_rvalid <= 1'b0;
     else
       s_rvalid <= s_rvalid;
  end
end

always @(posedge CLOCK) begin
    if (RESET == 0) begin
        s_rlast <= 0;
    end
    else if((slave_state == RDATA_STATE) & (count == 7))begin
        s_rlast <= 1'b1;
    end
    else begin
        s_rlast <=1'b0;
    end
end

always@(posedge CLOCK)begin
  if(!RESET)begin
    q_req_addr <= 0;
  end else begin
     if (slave_state == WRDATA_STATE)
        q_req_addr <= q_req_addr + 16;
     else if(slave_state == RDATA_STATE)
        q_req_addr <= q_req_addr + 16;
     else if(AWVALID)
        q_req_addr <= AWADDR;
     else if(ARVALID)
        q_req_addr <= ARADDR;
    
    
  end
end

always@(posedge CLOCK)begin
  if(!RESET)begin
    q_wrreq_valid <= 1'b0;
  end
  else begin
    q_wrreq_valid <= AWVALID; 
  end
end


always@(*) begin
  if(slave_state == IDLE)begin
     if(ARVALID)begin
        req_type = 1'b0;
        req_addr = ARADDR;
        req_data = 0;
     end
     if(q_wrreq_valid)begin
        req_type = 1'b1;
        req_addr = q_req_addr;
        req_data = 0;
     end
     else begin
        req_type = 1'b0;
        req_addr = 0;
        req_data = 0;  
     end
  end
  if(slave_state == WRDATA_STATE)begin
    req_type = 1'b1;
    req_addr = q_req_addr;  
    req_data = WDATA;
  end
  else if(slave_state == RDATA_STATE)begin
    req_type = 1'b0;
    req_addr = q_req_addr;
    req_data = 0;
  end
  else begin
    req_type = 1'b0;
    req_addr = 0;
    req_data = 0;
  end        
end

always@(*)begin
  if(slave_state == WRDATA_STATE || slave_state == RDATA_STATE)
  req_en = 1'b1;
  else
        req_en = 1'b0;
end

assign axi_busy = (slave_state == IDLE)? 1'b1:1'b0;

//implement master command interface state machine
always @ (posedge CLOCK) begin
    if (RESET == 0) begin
        count <= 0;
        slave_state <= IDLE;
    end else begin
        case (slave_state)
            IDLE: begin
                if (AWVALID) begin
                    slave_state <= WRDATA_STATE;
                end
                else if(ARVALID) begin
                    slave_state <= RDATA_STATE;
                end
                count <= 0;
            end
            WRREQ_STATE: begin
                if (WVALID)
                    slave_state <= WRDATA_STATE;
                else 
                    slave_state <= WRREQ_STATE;
            end
            WRDATA_STATE: begin
                if ((count == 7) & WREADY & WVALID) begin
                    slave_state <= IDLE;
                end else begin
                    count <= count + 1;
                end 
            end
            RDREQ_STATE: begin
                if (RVALID)
                    slave_state <= RDATA_STATE;
                else 
                    slave_state <= RDREQ_STATE;
            end
            RDATA_STATE: begin
                if ((count == 7) & RREADY & RREADY) begin
                    slave_state <= IDLE;
                end else
                    count <= count + 1;
            end
            default: begin
                slave_state <= IDLE;
            end
        endcase
    end
end


endmodule


//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/interface/axi4_slave.v
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/interface/axi4_master.v
//////////////////////////////////////////////////////////////////////

module axi4_master #
(
    parameter P_TARGET_SLAVE_BASE_ADDR     = 32'h0,                             //Base address of targeted slave
    
    parameter integer P_ADDR_WIDTH   = 32,                                      //Width of Address Bus
    parameter integer P_DATA_WIDTH   = 128                                      //Width of Data Bus
)
(
    input  wire [P_ADDR_WIDTH - 1:0] req_addr,
    input  wire [P_DATA_WIDTH - 1:0] req_data,
    input  wire req_en,
    input       req_type,
    output reg [P_DATA_WIDTH - 1:0]     req_read_data,
    output      axi_busy,
    input [7:0] in_dst,
    input   in_dst_we,
    input [7:0]  in_dst_mask, 

    output [7:0] out_dst,
    output   out_dst_we,
    output [7:0]  out_dst_mask, 

    input  wire CLOCK,                                                          //Global Clock Signal.
    input  wire RESET,                                                          //Global Reset Signal. This Signal is Active Low

    output wire [6 - 1:0] AWID,                                        //Master Interface Write Address ID
    output wire [P_ADDR_WIDTH - 1:0] AWADDR,                                    //Master Interface Write Address
    output wire [7:0] AWLEN,                                                    //Burst length. The burst length gives the exact number of transfers in a burst
    output wire [2:0] AWSIZE,                                                   //Burst size. This signal indicates the size of each transfer in the burst
    output wire [1:0] AWBURST,                                                  //Burst type. The burst type and the size information, determine how the address for each transfer within the burst is calculated.
    output wire AWLOCK,                                                         //Lock type. Provides additional information about the atomic characteristics of the transfer.
    output wire [3:0] AWCACHE,                                                  //Memory type. This signal indicates how transactions are required to progress through a system.
    output wire [2:0] AWPROT,                                                   //Protection type. This signal indicates the privilege and security level of the transaction, and whether  the transaction is a data access or an instruction access.
    output wire [3:0] AWQOS,                                                    //Quality of Service, QoS identifier sent for each write transaction.
    output wire AWVALID,                                                        //Write address valid. This signal indicates that the channel is signaling valid write address and control information.
    input  wire AWREADY,                                                        //Write address ready. This signal indicates that the slave is ready to accept an address and associated control signals
    output wire [P_DATA_WIDTH-1 : 0] WDATA,                                     //Master Interface Write Data.
    output wire [P_DATA_WIDTH/8-1 : 0] WSTRB,                                   //Write strobes. This signal indicates which byte lanes hold valid data. There is one write strobe bit for each eight bits of the write data bus.
    output wire WLAST,                                                          //Write last. This signal indicates the last transfer in a write burst.
    output wire WVALID,                                                         //Write valid. This signal indicates that valid write data and strobes are available
    input  wire WREADY,                                                         //Write ready. This signal indicates that the slave can accept the write data.

    output wire [6-1 : 0] ARID,                                        //Master Interface Read Address.
    output wire [P_ADDR_WIDTH-1 : 0] ARADDR,                                    //Read address. This signal indicates the initial address of a read burst transaction.
    output wire [7 : 0] ARLEN,                                                  //Burst length. The burst length gives the exact number of transfers in a burst
    output wire [2 : 0] ARSIZE,                                                 //Burst size. This signal indicates the size of each transfer in the burst
    output wire [1 : 0] ARBURST,                                                //Burst type. The burst type and the size information, determine how the address for each transfer within the burst is calculated.
    output wire ARLOCK,                                                         //Lock type. Provides additional information about the atomic characteristics of the transfer.
    output wire [3 : 0] ARCACHE,                                                //Memory type. This signal indicates how transactions are required to progress through a system.
    output wire [2 : 0] ARPROT,                                                 //Protection type. This signal indicates the privilege and security level of the transaction, and whether the transaction is a data access or an instruction access.
    output wire [3 : 0] ARQOS,                                                  //Quality of Service, QoS identifier sent for each read transaction
    output wire ARVALID,                                                        //Write address valid. This signal indicates that the channel is signaling valid read address and control information
    input  wire ARREADY,                                                        //Read address ready. This signal indicates that the slave is ready to accept an address and associated control signals
    input  wire [6-1 : 0] RID,                                         //Read ID tag. This signal is the identification tag for the read data group of signals generated by the slave.
    input  wire [P_DATA_WIDTH-1 : 0] RDATA,                                     //Master Read Data
    input  wire [1 : 0] RRESP,                                                  //Read response. This signal indicates the status of the read transfer
    input  wire RLAST,                                                          //Read last. This signal indicates the last transfer in a read burst
    input  wire RVALID,                                                         //Read valid. This signal indicates that the channel is signaling the required read data.
    output wire RREADY,                                                         //Read ready. This signal indicates that the master can accept the read data and response information.

    output wire BREADY,                                                         //Response ready. This signal indicates that the master can accept a write response.
    input  wire BVALID,                                                         //Write response valid. This signal indicates that the channel is signaling a valid write response.
    input  wire [1 : 0] BRESP,                                                  //Write response. This signal indicates the status of the write transaction.
    input  wire [6-1 : 0] BID                                          //Master Interface Write Response.
);

//Function called clogb2 that returns an integer which has the value of the
//ceiling of the log base 2
////function integer clogb2 (input integer bit_depth);
////begin
////    for(clogb2=0; bit_depth>0; clogb2=clogb2+1)
////        bit_depth = bit_depth >> 1;
////    end
////endfunction
//
////P_WRITE_TRANSACTIONS_NUM is the width of the index counter for number of
//write or read transaction.
////localparam integer P_WRITE_TRANSACTIONS_NUM = clogb2(P_WRITE_BURSTS - 1);
//
////Burst length for transactions, in P_DATA_WIDTHs. Non-2^n lengths will
//eventually cause bursts across 4K address boundaries.
//localparam integer C_MASTER_LENGTH = 12;
//
////Total number of burst transfers is master length divided by burst length
//and burst size
localparam integer C_NO_BURSTS_REQ = 12;

//= C_MASTER_LENGTH-clogb2((P_WRITE_BURSTS*P_DATA_WIDTH/8)-1);
//
//// Example State machine to initialize counter, initialize write
//transactions, initialize read transactions and comparison of read data with
//the written data words.

//Total number of burst transfers is master length divided by burst length and burst size
//localparam integer C_NO_BURSTS_REQ = C_MASTER_LENGTH-clogb2((P_WRITE_BURSTS*P_DATA_WIDTH/8)-1);

// Example State machine to initialize counter, initialize write transactions, initialize read transactions and comparison of read data with the written data words.
reg [1:0] master_state;
localparam [1:0] IDLE = 0;
localparam [1:0] REQ_STATE  = 1;
localparam [1:0] WRDATA_STATE = 2;
localparam [1:0] RDATA_STATE = 3;

//reg [P_ADDR_WIDTH - 1:0] m_write_addr = 0;
//reg [P_DATA_WIDTH - 1:0] m_write_data = 0;
//reg [6 - 1:0]   m_write_awid = 0;

//AXI4 internal tmp signals
reg [6 - 1:0]   m_awid = 0;
reg [P_ADDR_WIDTH - 1:0] m_awaddr;
reg m_awvalid;
reg [P_DATA_WIDTH - 1:0] m_wdata;
reg m_wlast;
reg m_wvalid;
reg m_bready;
reg [P_ADDR_WIDTH - 1:0] m_araddr;
reg m_arvalid;
reg m_rready;
reg [P_ADDR_WIDTH - 1:0] q_req_addr;
reg [P_DATA_WIDTH - 1:0] q_req_data;
reg [P_DATA_WIDTH - 1:0] q2_req_data;
reg q_req_type;
reg [2:0] count;
//reg [P_WRITE_TRANSACTIONS_NUM:0] write_index;                                   //write beat count in a burst
//reg [P_WRITE_TRANSACTIONS_NUM:0] read_index;                                    //read beat count in a burst
//wire [P_WRITE_TRANSACTIONS_NUM + 2: 0] burst_size_bytes;                        //size of P_WRITE_BURSTS length burst in bytes

//The burst counters are used to track the number of burst transfers of 
//P_WRITE_BURSTS burst length needed to transfer 2^C_MASTER_LENGTH bytes of data.
//reg [C_NO_BURSTS_REQ : 0] write_burst_counter;
//reg [C_NO_BURSTS_REQ : 0] read_burst_counter;
// reg start_single_burst_write;
// reg start_single_burst_read;
// reg writes_done;
// reg reads_done;
// reg error_reg;
reg compare_done;
//reg read_mismatch;
//reg burst_write_active;
//reg burst_read_active;
//reg [P_DATA_WIDTH - 1: 0] expected_rdata;

//Interface response error flags
//wire write_resp_error;
//wire read_resp_error;
//wire wnext;
//wire rnext;
//reg  init_txn_ff;
//reg  init_txn_ff2;
//reg  init_txn_edge;
//wire init_txn_pulse;

reg [31:0] m_writer_counter = 0;

// I/O Connections assignments
//assign WRITE_READY = ((AWREADY && WREADY) || current_state == IDLE) ? 1 : 0;
//assign WRITE_READY = (AWREADY && WREADY) ? 1 : 0;

//Write Address (AW)
assign AWID = m_awid + m_writer_counter;
assign AWADDR = P_TARGET_SLAVE_BASE_ADDR + m_awaddr;                            //The AXI address is a concatenation of the target base address + active offset range
assign AWLEN= 8'h0;                                               //Burst LENgth is number of transaction beats, minus 1
assign AWSIZE = 3'b100;                                 //Size should be P_DATA_WIDTH, in 2^SIZE bytes, otherwise narrow bursts are used
assign AWBURST = 2'b01;                                                         //INCR burst type is usually used, except for keyhole bursts
assign AWCACHE = 4'b1100;
assign AWPROT = 3'h0;
assign AWQOS = 4'h0;
assign AWVALID = m_awvalid;
assign AWLOCK = 1'b0;                                                           //Update value to 4'b0011 if coherent accesses to be used via the Zynq ACP port.
                                                                                //Not Allocated, Modifiable, not Bufferable. Not Bufferable since this example is meant to test memory, not intermediate cache.
//Write Data(W)
assign WDATA = m_wdata;                                                         //All bursts are complete and aligned in this example
assign WSTRB = {(P_DATA_WIDTH / 8){1'b1}};
assign WLAST = m_wlast;
assign WVALID = m_wvalid;

//Write Response (B)
assign BREADY = m_bready;

//Read Address (AR)
assign ARID = 'b0;
assign ARADDR = P_TARGET_SLAVE_BASE_ADDR + m_araddr;                            //Burst Length is number of transaction beats, minus 1
assign ARLEN = 8'h0;                                               //Size should be P_DATA_WIDTH, in 2^n bytes, otherwise narrow bursts are used
assign ARSIZE = 3'b100;                                 //INCR burst type is usually used, except for keyhole bursts
assign ARBURST = 2'b01;
assign ARCACHE = 4'b1100;
assign ARPROT = 3'h0;
assign ARQOS = 4'h0;
assign ARVALID = m_arvalid;
assign ARLOCK = 1'b0;                                                           //Update value to 4'b0011 if coherent accesses to be used via the Zynq ACP port. 
                                                                                //Not Allocated, Modifiable, not Bufferable. Not Bufferable since this example is meant to test memory, not intermediate cache.

//Read and Read Response (R)
assign RREADY = m_rready;

//Example design I/O
//assign TXN_DONE = compare_done;
//assign burst_size_bytes = P_WRITE_BURSTS * P_DATA_WIDTH / 8;                    //Burst size in bytes
//assign init_txn_pulse = (!init_txn_ff2) && init_txn_ff;

//Write Address Channel
always @(posedge CLOCK) begin
    if (RESET == 0) begin
        m_awvalid <= 0;
    end else if ((master_state == IDLE) & req_en & req_type) begin
        m_awvalid <= 1;
        //m_writer_counter <= m_writer_counter + 1;
    end else if (AWREADY && m_awvalid) begin                                    //Once asserted, VALIDs cannot be de-asserted, so m_awvalid must wait until transaction is accepted
        m_awvalid <= 0;
        //m_writer_counter <= m_writer_counter - 1;
    end else begin
        m_awvalid <= m_awvalid;
    end
end

always @(posedge CLOCK) begin
    if (RESET == 0) begin
        m_awaddr <= 0;
    end else if ((master_state ==IDLE) & req_en & req_type) begin
        m_awaddr <= req_addr;
    end else if (AWREADY && m_awvalid) begin
        m_awaddr <= 0;
    end else begin
        m_awaddr <= m_awaddr;
    end
end

always @(posedge CLOCK) begin
    if (RESET == 0) begin
        m_araddr <= 0;
    end else if ((master_state ==IDLE) & req_en & req_type) begin
        m_araddr <= req_addr;
    end else if (AWREADY && m_awvalid) begin
        m_araddr <= 0;
    end else begin
        m_araddr <= m_awaddr;
    end
end

always @(posedge CLOCK) begin
    if (RESET == 0) begin
        m_wvalid <= 0;
    end else if ((master_state == REQ_STATE) & q_req_type) begin
        m_wvalid <= 1;
    end else if ((count==7) & WREADY && m_wvalid) begin
        m_wvalid <= 0;
    end else begin
        m_wvalid <= m_wvalid;
    end
end

always @(posedge CLOCK) begin
    if (RESET == 0) begin
        m_wlast <= 0;
    end else if ((master_state == WRDATA_STATE) & (count == 7)) begin
        m_wlast <= 1;
    end else if (WREADY && m_wvalid) begin
        m_wlast <= 0;
    end else begin
        m_wlast <= m_wlast;
    end
end

//Write Data Generator.Data pattern is only a simple incrementing count from 0 for each burst.
always @(posedge CLOCK) begin
    if (RESET == 0) begin
        m_wdata <= 0;
    end else if (((master_state == REQ_STATE) & q_req_type) | (master_state == WRDATA_STATE)) begin
        m_wdata <= q_req_data;
    end else if(master_state == IDLE)begin
        m_wdata <= 0;
    end else begin 
        m_wdata <= m_wdata;
    end
end

//Write Response (B) Channel
 always @(posedge CLOCK) begin
    if (RESET == 0) begin
        m_bready <= 0;
    end else begin
        m_bready <= 1;
        //m_bready <= m_bready;
    end
end

//Need this to silent VIP warning
always @(posedge CLOCK) begin
    if (RESET == 0) begin
        m_arvalid <= 0;
    end
    else if((master_state == IDLE) & req_en & (~req_type))begin
        m_arvalid <= 1'b1;
    end
    else if(ARVALID && ARREADY)begin
        m_arvalid <= 1'b0;
    end
    else begin
        m_arvalid <= m_arvalid;
    end
end

//Need this to silent VIP warning
always @(posedge CLOCK) begin
    if (RESET == 0) begin
        m_rready <= 0;
        req_read_data <= 0;
    end
    else if(RDATA_STATE)begin
        m_rready <= 1'b1;
    end
    else if(RVALID && m_rready)begin
        m_rready <= 1'b0;
        req_read_data <= RDATA;
    end
    else begin
        m_rready <= m_rready;
    end
end

always@(posedge CLOCK)begin
  if(!RESET)begin
    q_req_type <= 1'b0;
    q_req_addr <= 32'h0;
    q_req_data <= 128'h0;
    q2_req_data <= 128'h0;
  end
  else begin
      q_req_type <= req_type;
      q_req_addr <= req_addr;
      q_req_data <= req_data;
      q2_req_data <= q_req_data;
  end
end

assign axi_busy = (master_state == IDLE)? 1'b1:1'b0;


//implement master command interface state machine
always @ (posedge CLOCK) begin
    if (RESET == 0) begin
        master_state <= IDLE;
        count <= 0;
    end else begin
        case (master_state)
            IDLE: begin
                if (req_en) begin
                    master_state <= REQ_STATE;
                end else begin
                    master_state <= IDLE;
                end
                count <= 0;
            end
            REQ_STATE: begin
                if ((q_req_type == 1'b1) &&(AWVALID & AWREADY))
                    master_state <= WRDATA_STATE;
                else if ((q_req_type == 1'b0) &&(ARVALID & ARREADY))
                    master_state <= RDATA_STATE;
                count <= 0;
            end
            WRDATA_STATE: begin
                if (WREADY) begin
                    if(count == 3'h7)
                        master_state <= IDLE;
                    else
                        master_state <= WRDATA_STATE;
                    count <= count + 3'h1 ;
                end
            end
            RDATA_STATE: begin
                if (RREADY) begin
                    if(count == 3'h7)
                        master_state <= IDLE;
                    else
                        master_state <= RDATA_STATE;
                end
                    count <= count + 3'h1 ;
            end
            default: begin
                master_state <= IDLE;
                count <= 0;
            end
        endcase
    end
end


endmodule

//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/interface/axi4_master.v
//////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector_scripts//verilog/onecycle_stall.v
//////////////////////////////////////////////////////////////////////
/****************************************************************************
          One cycle Stall circuit
****************************************************************************/
module onecyclestall(request,clk,resetn,stalled);
input request;
input clk;
input resetn;
output stalled;

  reg T,Tnext;

  // State machine for Stalling 1 cycle
  always@(request or T)
  begin
    case(T) 
      1'b0: Tnext=request;
      1'b1: Tnext=0;
    endcase 
  end       
  always@(posedge clk)
    if (~resetn)
      T<=0; 
    else    
      T<=Tnext;
  assign stalled=(request&~T);
endmodule

//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector_scripts/verilog/onecycle_stall.v
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/local/per_lane_mem_wrapper.v
//////////////////////////////////////////////////////////////////////

module per_lane_mem_wrapper (
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

parameter AWIDTH=10;
parameter NUM_WORDS=1024;
parameter DWIDTH=32;

input clk;
input resetn;
input [(AWIDTH-1):0] address_a;
input [(AWIDTH-1):0] address_b;
input  wren_a;
input  wren_b;
input  rden_a;
input  rden_b;
input [(DWIDTH-1):0] data_a;
input [(DWIDTH-1):0] data_b;
output [(DWIDTH-1):0] out_a;
output [(DWIDTH-1):0] out_b;

reg [(AWIDTH-1):0] q_address_a;
reg [(AWIDTH-1):0] q_address_b;
reg [(AWIDTH-1):0] mux_address_b;
reg [(AWIDTH-1):0] mux_address_a;

dpram inst_dpram(
    .clk(clk),
    .address_a(mux_address_a),
    .address_b(mux_address_b),
    .wren_a(wren_a),
    .wren_b(wren_b),
    .data_a(data_a),
    .data_b(data_b),
    .out_a(out_a),
    .out_b(out_b)
);
defparam
    inst_dpram.AWIDTH=AWIDTH,
    inst_dpram.NUM_WORDS=NUM_WORDS,
    inst_dpram.DWIDTH=DWIDTH;

always@(posedge clk)begin
   if(!resetn)begin
     q_address_a <= 'h0;
     q_address_b <= 'h0;
   end
   else begin
     if(rden_a | wren_a)
       q_address_a <= address_a;
     if(rden_b | wren_b)
       q_address_b <= address_b;
   end
end

always@(*)begin

  if(rden_a | wren_a)   
    mux_address_a = address_a;
  else
    mux_address_a = q_address_a; 

  if(rden_b | wren_b)   
    mux_address_b = address_b;
  else
    mux_address_b = q_address_b;
 
end

endmodule
//////////////////////////////////////////////////////////////////////
//// Finshing contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector_scripts/local/per_lane_mem_wrapper.v
//////////////////////////////////////////////////////////////////////

//Declared here but applies to whole design (thanks to `includes)!
//`include "options.v"


module core (
    clk,
    resetn,
    rt_dataout, // Dummy output - use this to prevent design from being
                // synthesized away if using on-chip memory

    ibus_en,
    ibus_address,
    ibus_readdata,
    ibus_wait,

  // PETES CHANGE for tracing
  trc_addr,
  trc_data,
  trc_we,
  trc_stall,
  trc_pipestall,

  // AXI interface
   M_AWID    ,
    M_AWADDR  ,
    M_AWLEN   ,
    M_AWSIZE  ,
    M_AWBURST ,
    M_AWLOCK  ,
    M_AWCACHE ,
    M_AWPROT  ,
    M_AWQOS   ,
    M_AWVALID ,
    M_AWREADY ,
    M_WDATA   ,
    M_WSTRB   ,
    M_WLAST   ,
    M_WVALID  ,
    M_WREADY  ,
    M_ARID    ,
    M_ARADDR  ,
    M_ARLEN   ,
    M_ARSIZE  ,
    M_ARBURST ,
    M_ARLOCK  ,
    M_ARCACHE ,
    M_ARPROT  ,
    M_ARQOS   ,
    M_ARVALID ,
    M_ARREADY ,
    M_RID     ,
    M_RDATA   ,
    M_RRESP   ,
    M_RLAST   ,
    M_RVALID  ,
    M_RREADY  ,
    M_BREADY  ,
    M_BVALID  ,
    M_BRESP   ,
    M_BID     ,

    S_AWID   , 
    S_AWADDR , 
    S_AWLEN  , 
    S_AWSIZE , 
    S_AWBURST, 
    S_AWLOCK , 
    S_AWCACHE, 
    S_AWPROT , 
    S_AWQOS  , 
    S_AWVALID, 
    S_AWREADY, 
    S_WDATA  , 
    S_WSTRB  , 
    S_WLAST  , 
    S_WVALID , 
    S_WREADY , 
    S_ARID   , 
    S_ARADDR , 
    S_ARLEN  , 
    S_ARSIZE , 
    S_ARBURST, 
    S_ARLOCK , 
    S_ARCACHE, 
    S_ARPROT , 
    S_ARQOS  , 
    S_ARVALID, 
    S_ARREADY, 
    S_RID    , 
    S_RDATA  , 
    S_RRESP  , 
    S_RLAST  , 
    S_RVALID , 
    S_RREADY , 
    S_BREADY , 
    S_BVALID , 
    S_BRESP  , 
    S_BID ,    
  
  // Databus interface
    dbus_address,
    dbus_readdata,
    dbus_writedata,
    dbus_byteen,
    dbus_readdata_line,
    dbus_writedata_line,
    dbus_byteen_line,
    dbus_en,
    dbus_wren,
    dbus_cachematch,
    dbus_cachemiss,
    dbus_prefetch,
    dbus_wait,

    scalar_dbus_address,
    scalar_dbus_readdata,
    scalar_dbus_writedata,
    scalar_dbus_byteen,
    scalar_dbus_en,
    scalar_dbus_wren,

    dma_dbus_address,   
    dma_dbus_readdata,  
    dma_dbus_writedata, 
    dma_dbus_byteen,
    dma_dbus_en,        
    dma_dbus_wren,      
    dma_dbus_prefetch,  
    dma_dbus_wait,      
    dma_dbus_data_valid   
    );

parameter LOG2DCACHEWIDTHBITS=`LOG2DCACHEWIDTHBITS;
parameter DCACHEWIDTHBITS=2**LOG2DCACHEWIDTHBITS;

input              clk;
input              resetn;
output [31:0]      rt_dataout;

output        ibus_en;             // Instruction bus signals
output [31:0] ibus_address;
input  [31:0] ibus_readdata;
input         ibus_wait;

output [31:0] scalar_dbus_address;    // Data bus signals
input  [31:0] scalar_dbus_readdata;
output [31:0] scalar_dbus_writedata;
output [3:0]  scalar_dbus_byteen;
output        scalar_dbus_en;
output        scalar_dbus_wren;

output [31:0] dbus_address;    // Data bus signals
input  [31:0] dbus_readdata;
input  [DCACHEWIDTHBITS-1:0] dbus_readdata_line;
output [31:0] dbus_writedata;
output [3:0]  dbus_byteen;
output [DCACHEWIDTHBITS-1:0] dbus_writedata_line;
output [DCACHEWIDTHBITS/8-1:0]  dbus_byteen_line;
output        dbus_en;
output        dbus_wren;
input         dbus_cachematch;
input         dbus_cachemiss;
output [31:0] dbus_prefetch;
input         dbus_wait;

// PETES CHANGE for tracing
output  [ 4 : 0 ]   trc_addr;
output  [ 31 : 0 ]  trc_data;
output              trc_we;
input               trc_stall;
output              trc_pipestall;

// AXI interface

 output  [6 - 1:0]     M_AWID;                                                 
 output  [16- 1:0]     M_AWADDR;                                    
 output  [7:0]         M_AWLEN;                                                    
 output  [2:0]         M_AWSIZE;                                                   
 output  [1:0]         M_AWBURST;                                                  
 output                M_AWLOCK;                                                         
 output  [3:0]         M_AWCACHE;                                                  
 output  [2:0]         M_AWPROT;                                                   
 output  [3:0]         M_AWQOS;                                                    
 output                M_AWVALID;                                                        
 input                 M_AWREADY;                                                        
 output  [128-1 : 0]   M_WDATA;                                     
 output  [128/8-1 : 0] M_WSTRB;                                   
 output                M_WLAST;                                                          
 output                M_WVALID;                                                         
 input                 M_WREADY;                                                         

 output  [6-1 : 0]     M_ARID;                                                 
 output  [16-1 : 0]    M_ARADDR;                                    
 output  [7 : 0]       M_ARLEN;                                                  
 output  [2 : 0]       M_ARSIZE;                                                 
 output  [1 : 0]       M_ARBURST;                                                
 output                M_ARLOCK;                                                         
 output  [3 : 0]       M_ARCACHE;                                                
 output  [2 : 0]       M_ARPROT;                                                 
 output  [3 : 0]       M_ARQOS;                                                  
 output                M_ARVALID;                                                        
 input                 M_ARREADY;                                                        
 input   [6-1 : 0]     M_RID;                                         	    
 input   [128-1 : 0]   M_RDATA;                                     
 input   [1 : 0]       M_RRESP;                                                  
 input                 M_RLAST;                                                          
 input                 M_RVALID;                                                         
 output                M_RREADY;                                                         

 output                M_BREADY;                                                         
 input                 M_BVALID;                                                         
 input   [1 : 0]       M_BRESP;                                                  
 input   [6-1 : 0]     M_BID;                                                   

 input  [6 - 1:0]       S_AWID;                                                 
 input  [16- 1:0]       S_AWADDR;                                    
 input  [7:0]           S_AWLEN;                                                    
 input  [2:0]           S_AWSIZE;                                                   
 input  [1:0]           S_AWBURST;                                                  
 input                  S_AWLOCK;                                                         
 input  [3:0]           S_AWCACHE;                                                  
 input  [2:0]           S_AWPROT;                                                   
 input  [3:0]           S_AWQOS;                                                    
 input                  S_AWVALID;                                                        
 output                 S_AWREADY;                                                        
 input  [128-1 : 0]     S_WDATA;                                     
 input  [128/8-1 : 0]   S_WSTRB;                                   
 input                  S_WLAST;                                                          
 input                  S_WVALID;                                                         
 output                 S_WREADY;                                                         

 input  [6-1 : 0]       S_ARID;                                                 
 input  [16-1 : 0]      S_ARADDR;                                    
 input  [7 : 0]         S_ARLEN;                                                  
 input  [2 : 0]         S_ARSIZE;                                                 
 input  [1 : 0]         S_ARBURST;                                                
 input                  S_ARLOCK;                                                         
 input  [3 : 0]         S_ARCACHE;                                                
 input  [2 : 0]         S_ARPROT;                                                 
 input  [3 : 0]         S_ARQOS;                                                  
 input                  S_ARVALID;                                                        
 output                 S_ARREADY;                                                        
 output   [6-1 : 0]     S_RID;                                         	    
 output   [128-1 : 0]   S_RDATA;                                     
 output   [1 : 0]       S_RRESP;                                                  
 output                 S_RLAST;                                                          
 output                 S_RVALID;                                                         
 input                  S_RREADY;                                                         

 input                  S_BREADY;                                                         
 output                 S_BVALID;                                                         
 output   [1 : 0]       S_BRESP;                                                  
 output   [6-1 : 0]     S_BID;                                                  


//DMA changes
output [31:0]                 dma_dbus_address;   
input  [DCACHEWIDTHBITS-1:0]    dma_dbus_readdata;  
output [DCACHEWIDTHBITS-1:0]  dma_dbus_writedata; 
output [DCACHEWIDTHBITS/8-1:0]dma_dbus_byteen;
output                        dma_dbus_en;        
output                        dma_dbus_wren;      
output                        dma_dbus_prefetch;  
input                         dma_dbus_wait;      
input                         dma_dbus_data_valid;

//`include "isa.v"
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: isa.v
//////////////////////////////////////////////////////////////////////
parameter     OP_SPECIAL      = 6'b000000;
parameter     OP_REGIMM       = 6'b000001;
parameter     OP_J            = 6'b000010;
parameter     OP_JAL          = 6'b000011;
parameter     OP_BEQ          = 6'b000100;
parameter     OP_BNE          = 6'b000101;
parameter     OP_BLEZ         = 6'b000110;
parameter     OP_BGTZ         = 6'b000111;

parameter     OP_ADDI         = 6'b001000;
parameter     OP_ADDIU        = 6'b001001;
parameter     OP_SLTI         = 6'b001010;
parameter     OP_SLTIU        = 6'b001011;
parameter     OP_ANDI         = 6'b001100;
parameter     OP_ORI          = 6'b001101;
parameter     OP_XORI         = 6'b001110;
parameter     OP_LUI          = 6'b001111;

parameter     OP_LB           = 6'b100000;
parameter     OP_LH           = 6'b100001;
parameter     OP_LWL          = 6'b100010;
parameter     OP_LW           = 6'b100011;
parameter     OP_LBU          = 6'b100100;
parameter     OP_LHU          = 6'b100101;
parameter     OP_LWR          = 6'b100110;

parameter     OP_SB           = 6'b101x00;
parameter     OP_SH           = 6'b101x01;
parameter     OP_SWL          = 6'b101010;
parameter     OP_SW           = 6'b101x11;
parameter     OP_SWR          = 6'b101110;

/****** FUNCTION CLASS - bits 5...0 *******/
parameter     FUNC_SLL        = 6'b000000;
parameter     FUNC_SRL        = 6'b000010;
parameter     FUNC_SRA        = 6'b000011;
parameter     FUNC_SLLV       = 6'b000100;
parameter     FUNC_SRLV       = 6'b000110;
parameter     FUNC_SRAV       = 6'b000111;

parameter     FUNC_JR         = 6'b001xx0;
parameter     FUNC_JALR       = 6'b001xx1;

parameter     FUNC_MFHI       = 6'bx10x00;
parameter     FUNC_MTHI       = 6'bx10x01;
parameter     FUNC_MFLO       = 6'bx10x10;
parameter     FUNC_MTLO       = 6'bx10x11;

parameter     FUNC_MULT       = 6'bx11x00;
parameter     FUNC_MULTU      = 6'bx11x01;
parameter     FUNC_DIV        = 6'bx11x10;
parameter     FUNC_DIVU       = 6'bx11x11;

parameter     FUNC_ADD        = 6'b100000;
parameter     FUNC_ADDU       = 6'b100001;
parameter     FUNC_SUB        = 6'b100010;
parameter     FUNC_SUBU       = 6'b100011;
parameter     FUNC_AND        = 6'b100100;
parameter     FUNC_OR         = 6'b100101;
parameter     FUNC_XOR        = 6'b100110;
parameter     FUNC_NOR        = 6'b100111;

parameter     FUNC_SLT        = 6'b101010;
parameter     FUNC_SLTU       = 6'b101011;

/****** REGIMM Class - bits 20...16 *******/
parameter     FUNC_BLTZ       = 1'b0;
parameter     FUNC_BGEZ       = 1'b1;

parameter     OP_COP2       = 6'b010010;
parameter     COP2_FUNC_CFC2     = 6'b111000;
parameter     COP2_FUNC_CTC2     = 6'b111010;
parameter     COP2_FUNC_MTC2     = 6'b111011;

parameter     OP_COP0       = 6'b010000;
parameter     COP0_MFC0     = 5'b00000;
parameter     COP0_MTC0     = 5'b00100;

//parameter     FUNC_BLTZAL     = 5'b10000;
//parameter     FUNC_BGEZAL     = 5'b10001;

/****** 
 * Original REGIMM class, compressed above to save decode logic
parameter     FUNC_BLTZ       = 5'b00000;
parameter     FUNC_BGEZ       = 5'b00001;
parameter     FUNC_BLTZAL     = 5'b10000;
parameter     FUNC_BGEZAL     = 5'b10001;
*/
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: isa.v
//////////////////////////////////////////////////////////////////////

//`include "visa.v"
//////////////////////////////////////////////////////////////////////
//// Starting contents of included file: visa.v
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//// Starting contents of file: /mnt/f/ceremorphic/lizy/tpu_v2/tpu/design/vector/visa.v
//////////////////////////////////////////////////////////////////////
parameter COP2_VADD           = 'b1000000000;
parameter COP2_VADD_U         = 'b1000000001;
parameter COP2_VSUB           = 'b1000000010;
parameter COP2_VSUB_U         = 'b1000000011;
parameter COP2_VMULHI         = 'b1000000100;
parameter COP2_VMULHI_U       = 'b1000000101;
parameter COP2_VDIV           = 'b1000000110; //Using as matmul
//parameter COP2_VBFADD         = 'b0100000001; //Using BF16 add
//parameter COP2_VBFMULT        = 'b0100000010; //Using BF16 MULT
//parameter COP2_VACT           = 'b0100000011; //Using ACT
//parameter COP2_VTRP           = 'b0100000100; //Using ACT
parameter COP2_VDIV_U         = 'b1000000111;

//parameter COP2_VMOD           = 'b1000001000;
parameter COP2_VBFADD           = 'b1000001000;  // USING bfloat add Instr: vmod.vv vrdest, vrsrc1,vrsrc2
parameter COP2_VBFMULT         = 'b1000001001;  // Using bfloat mult Instr: vmod.u.vv vrdest, vrsrc1,vrsrc2
parameter COP2_VMOD_U         = 'b1000001001;
parameter COP2_VCMP_EQ        = 'b1000001010;
parameter COP2_VCMP_NE        = 'b1000001100;
parameter COP2_VCMP_LT        = 'b1000001110;
parameter COP2_VCMP_U_LT      = 'b1000001111;
parameter COP2_VCMP_LE        = 'b1000010000;
parameter COP2_VCMP_U_LE      = 'b1000010001;
parameter COP2_VMIN           = 'b1000010010;
parameter COP2_VMIN_U         = 'b1000010011;
parameter COP2_VMAX           = 'b1000010100;
parameter COP2_VMAX_U         = 'b1000010101;
parameter COP2_VMULLO         = 'b1000010110;
parameter COP2_VABS           = 'b1000010111;
parameter COP2_VAND           = 'b1000011000;
parameter COP2_VOR            = 'b1000011001;
parameter COP2_VXOR           = 'b1000011010;
parameter COP2_VNOR           = 'b1000011011;
parameter COP2_VSLL           = 'b1000011100;
parameter COP2_VSRL           = 'b1000011101;
parameter COP2_VSRA           = 'b1000011110;
parameter COP2_VSAT_B         = 'b1000011111;
parameter COP2_VSAT_H         = 'b1001011111;
//parameter COP2_VSAT_W         = 'b1010011111;
parameter COP2_VACT         = 'b1010011111;  // Using activation instruction: vsat.w vrdest,vrsrc
parameter COP2_VSAT_SU_B      = 'b1000100000;
parameter COP2_VSAT_SU_H      = 'b1001100000;
//parameter COP2_VSAT_SU_W      = 'b1010100000;
parameter COP2_VRED     = 'b1010100000;   // Using reduction instruction: vsat.su.w vrdest,vrsrc
parameter COP2_VSAT_SU_L      = 'b1011100000;
parameter COP2_VSAT_U_B       = 'b1000100001;
parameter COP2_VSAT_U_H       = 'b1001100001;
//parameter COP2_VSAT_U_W       = 'b1010100001;
parameter COP2_VTRP       = 'b1010100001;   // Using transpose instruction: vsat.u.w vrdest, vrsrc 
parameter COP2_VSADD          = 'b1000100010;
parameter COP2_VSADD_U        = 'b1000100011;
parameter COP2_VSSUB          = 'b1000100100;
parameter COP2_VSSUB_U        = 'b1000100101;
parameter COP2_VSRR           = 'b1000100110;
parameter COP2_VSRR_U         = 'b1000100111;
parameter COP2_VSLS           = 'b1000101000;
parameter COP2_VSLS_U         = 'b1000101001;
parameter COP2_VXUMUL         = 'b1000101010;
parameter COP2_VXUMUL_U       = 'b1000101011;
parameter COP2_VXLMUL         = 'b1000101100;
parameter COP2_VXLMUL_U       = 'b1000101101;
parameter COP2_VXUMADD        = 'b1000101110;
parameter COP2_VXUMADD_U      = 'b1000101111;
parameter COP2_VXUMSUB        = 'b1000110000;
parameter COP2_VXUMSUB_U      = 'b1000110001;
parameter COP2_VXLMADD        = 'b1000110010;
parameter COP2_VXLMADD_U      = 'b1000110011;
parameter COP2_VXLMSUB        = 'b1000110100;
parameter COP2_VXLMSUB_U      = 'b1000110101;
parameter COP2_VINS_VV        = 'b1100000000;
parameter COP2_VINS_SV        = 'b1110000001;
parameter COP2_VEXT_VV        = 'b1100000010;
parameter COP2_VEXT_SV        = 'b1100000011;
parameter COP2_VEXT_U_SV      = 'b1100000100;
parameter COP2_VCOMPRESS      = 'b1100000101;
parameter COP2_VEXPAND        = 'b1100000110;
parameter COP2_VMERGE         = 'b1100000111;
parameter COP2_VFINS          = 'b1110001000;
parameter COP2_VEXTHALF       = 'b1100001001;
parameter COP2_VHALF          = 'b1100001010;
parameter COP2_VHALFUP        = 'b1100001011;
parameter COP2_VHALFDN        = 'b1100001100;
parameter COP2_VSATVL         = 'b1100001101;
parameter COP2_VFAND          = 'b1100001110;
parameter COP2_VFOR           = 'b1100001111;
parameter COP2_VFXOR          = 'b1100010000;
parameter COP2_VFNOR          = 'b1100010001;
parameter COP2_VFCLR          = 'b1100010010;
parameter COP2_VFSET          = 'b1100010011;
parameter COP2_VIOTA          = 'b1100010100;
parameter COP2_VCIOTA         = 'b1100010101;
parameter COP2_VFPOP          = 'b1100010110;
parameter COP2_VFFF1          = 'b1100010111;
parameter COP2_VFFL1          = 'b1100011000;
parameter COP2_VFSETBF        = 'b1100011001;
parameter COP2_VFSETIF        = 'b1100011010;
parameter COP2_VFSETOF        = 'b1100011011;
parameter COP2_VFMT8          = 'b1100011100;
parameter COP2_VFMF8          = 'b1100011101;
parameter COP2_VFCLR8         = 'b1100011110;
parameter COP2_VFOR8          = 'b1100011111;
parameter COP2_VFLD           = 'b1100100000;
parameter COP2_VLD_B          = 'b1100100001;
parameter COP2_VLD_H          = 'b1101100001;

//parameter COP2_VLD_W          = 'b1110100001;
//parameter COP2_VBFADD         = 'b1110100001;  // adding bfadder Instr: vld.u.w

parameter COP2_VLD_L          = 'b1111100001;
parameter COP2_VLD_U_B        = 'b1100100010;
parameter COP2_VLD_U_H        = 'b1101100010;

//parameter COP2_VLD_U_W        = 'b1110100010;
parameter COP2_VAXIRD	        = 'b1110100010;  // adding an instruction for AXI Load;

parameter COP2_VLDS_B         = 'b1100100011;
parameter COP2_VLDS_H         = 'b1101100011;

//parameter COP2_VLDS_W         = 'b1110100011;
parameter COP2_VBFSUB         = 'b1110100011;   // adding bfsub Instr: vlds.w

parameter COP2_VLDS_L         = 'b1111100011;
parameter COP2_VLDS_U_B       = 'b1100100100;
parameter COP2_VLDS_U_H       = 'b1101100100;

//parameter COP2_VLDS_U_W       = 'b1110100100;
//parameter COP2_VBFMULT         = 'b1110100100;   // adding bfmult Instr: vlds.u.w

parameter COP2_VLDX_B         = 'b1100100101;
parameter COP2_VLDX_H         = 'b1101100101;

//parameter COP2_VLDX_W         = 'b1110100101;
parameter COP2_VPER_STR         = 'b1110100101;     // adding transpose instruction: vldx.w

parameter COP2_VLDX_L         = 'b1111100101;
parameter COP2_VLDX_U_B       = 'b1100100110;
parameter COP2_VLDX_U_H       = 'b1101100110;

//parameter COP2_VLDX_U_W       = 'b1110100110;
//parameter COP2_VPER     = 'b1110100110;        //adding activation Instr: vldx.u.w 

parameter COP2_VFST           = 'b1100101000;
parameter COP2_VST_B          = 'b1100101001;
parameter COP2_VST_H          = 'b1101101001;

//parameter COP2_VST_W        = 'b1110101001;  // adding reduction Instr: vst.w
//parameter COP2_VRED           = 'b1110101001;  // adding reduction Instr: vst.w

parameter COP2_VST_L          = 'b1111101001;
parameter COP2_VSTS_B         = 'b1100101010;
parameter COP2_VSTS_H         = 'b1101101010;

//parameter COP2_VSTS_W         = 'b1110101010;
parameter COP2_VPER_LD           = 'b1110101010;  // adding permute Instr: vsts.w

parameter COP2_VSTS_L         = 'b1111101010;
parameter COP2_VSTX_B         = 'b1100101011;
parameter COP2_VSTX_H         = 'b1101101011;
//parameter COP2_VSTX_W         = 'b1110101011;
parameter COP2_VPER         = 'b1110101011;  // adding permute operation: vstx.w Vsrc,Vbase, 
parameter COP2_VSTX_L         = 'b1111101011;
parameter COP2_VSTXO_B        = 'b1100101100;
parameter COP2_VSTXO_H        = 'b1101101100;
//parameter COP2_VSTXO_W        = 'b1110101100;  
parameter COP2_VAXIWR        = 'b1110101100;  // adding an instruction for axi write
parameter COP2_VSTXO_L        = 'b1111101100;
parameter COP2_VMCTS          = 'b1101110000;
parameter COP2_VMSTC          = 'b1101110001;
parameter COP2_CFC2           = 'b0000111000;
parameter COP2_CTC2           = 'b0000111010;
parameter COP2_MTC2           = 'b0000111011;
//////////////////////////////////////////////////////////////////////
//// Finish contents of included file: visa.v
//////////////////////////////////////////////////////////////////////

    wire [31:0] p_dbus_address;    // Processor's data bus signals
    wire [31:0] p_dbus_writedata;
    wire [3:0]  p_dbus_byteen;
    wire [DCACHEWIDTHBITS/8-1:0]  p_dbus_byteen_line;
    wire        p_dbus_en;
    wire        p_dbus_wren;
    wire  [31:0] p_dbus_readdata;
    wire         p_dbus_wait;

    wire [31:0] v_dbus_address;    // VPU's data bus signals
    wire [DCACHEWIDTHBITS-1:0] v_dbus_writedata;
    wire [DCACHEWIDTHBITS/8-1:0]  v_dbus_byteen;
    wire        v_dbus_en;
    wire        v_dbus_wren;
    wire  [DCACHEWIDTHBITS-1:0] v_dbus_readdata;
    wire [31:0] v_dbus_prefetch;
    wire         v_dbus_wait;

    wire cop2_fromcop2_wait;
    wire cop2_fromcop2_en;
    wire [31:0] cop2_fromcop2;
    wire cop2_tocop2_wait;
    wire cop2_tocop2_en;
    wire [31:0] cop2_tocop2;

    wire vpu_stalled;
    wire vpu_has_memop;
    reg ibus_en_r;
    wire instr_en;
    wire ifetch_bus_wait;
    wire ifetch_squashn;

    reg is_cop2;              //Instr is a coprocessor 2 instr
    reg is_scalar_cop2;       //Instr executes both scalar and vpu
    reg is_vec_cop2;          //Instr executes only in vpu
    reg is_scalar_memop;      //Scalar memory operation 
    reg is_vec_memop;         //Vector memory operation 

    wire [31:0] ibus_ecause;
    wire [31:0] dbus_ecause;
    wire [31:0] device_ecause;
    wire [31:0] badvaddr;
    wire        badvaddr_we;

    // Do some instruction decoding to see when we're allowed to issue
    always@*
    begin
      is_cop2=0;
      is_scalar_cop2=0;
      is_vec_cop2=0;
      is_scalar_memop=0;
      is_vec_memop=0;

      casex (ibus_readdata[31:26])
        OP_COP2:
        begin
          is_cop2=1;
          casex (ibus_readdata[5:0])
            COP2_FUNC_CFC2,
            COP2_FUNC_CTC2,
            COP2_FUNC_MTC2: is_scalar_cop2=1;
            default: is_vec_cop2=1;
          endcase
          casez ({ibus_readdata[25:22],ibus_readdata[5:0]})
            COP2_VFLD,
            COP2_VLD_B,
            COP2_VLD_H,
           // COP2_VBFADD,
            COP2_VLD_L,
            COP2_VLD_U_B,
            COP2_VLD_U_H,
            //COP2_VLD_U_W,
            COP2_VLDS_B,
            COP2_VLDS_H,
            COP2_VBFSUB,
            COP2_VLDS_L,
            COP2_VLDS_U_B,
            COP2_VLDS_U_H,
            COP2_VBFMULT,
            COP2_VLDX_B,
            COP2_VLDX_H,
            COP2_VTRP,
            COP2_VLDX_L,
            COP2_VLDX_U_B,
            COP2_VLDX_U_H,
            COP2_VACT,
            COP2_VFST,
            COP2_VST_B,
            COP2_VST_H,
            COP2_VRED,
            COP2_VST_L,
            COP2_VSTS_B,
            COP2_VSTS_H,
            COP2_VPER,
            COP2_VSTS_L,
            COP2_VSTX_B,
            COP2_VSTX_H,
            //COP2_VSTX_W,
            COP2_VSTX_L,
            COP2_VSTXO_B,
            COP2_VSTXO_H,
            //COP2_VSTXO_W,
            COP2_VSTXO_L: is_vec_memop=1;
          endcase
        end
        OP_LB,
        OP_LBU,
        OP_LH,
        OP_LHU,
        OP_LW,
        OP_SB,
        OP_SH,
        OP_SW: is_scalar_memop=1;
      endcase
    end

    //Stall scalar processor when:
    //  (a) instr not fetched (ibus_wait)
    //  (b) vpu is stalled (vpu_stalled) and next instr is_cop2
    //  (c) vpu has mem operation
    assign ifetch_bus_wait=ibus_wait||
                            (vpu_stalled && is_cop2) || 
                            (vpu_has_memop && is_scalar_memop);

    //Submit valid instr to VPU when
    //  (a) instr is fetched (ibus_wait)
    //  (b) scalar cpu is not staled (ibus_en)
    //  (c) scalar cpu isn't squashing the instruction (ifetch_squashn)
    //       - Do we need (c) now that vec insns not allowed in delay slot?
    //  (d) vpu has a mem op (vpu_has_memop) in which case scalar is stalling
    //  Note that a vec_memop will not be issued when a scalar mem_op is in
    //  flight because the pipe will stall right after issuing the scalar memop
    assign instr_en=ibus_en&ifetch_squashn&~ibus_wait&
                        ~(vpu_has_memop && is_scalar_memop);


    /*********************** SPREE scalar MIPS processor ********************
    * This processor was generated by SPREE which automatically produces the
    * system module that implements your described processor
    ************************************************************************/
   reg scalar_dbus_wait;

    system p
      (
      .clk     (clk),
      .resetn (resetn),

      .ifetch_bus_en(ibus_en),
      .ifetch_bus_address(ibus_address),
      .ifetch_bus_readdata(ibus_readdata),
      .ifetch_bus_wait(ifetch_bus_wait),
      .ifetch_bus_squashn(ifetch_squashn),
      .ifetch_bus_ecause(ibus_ecause),


      .data_mem_bus_address(scalar_dbus_address),
      .data_mem_bus_readdata(scalar_dbus_readdata),
      .data_mem_bus_writedata(scalar_dbus_writedata),
      .data_mem_bus_byteen(scalar_dbus_byteen),
      .data_mem_bus_en(scalar_dbus_en),
      .data_mem_bus_we(scalar_dbus_wren),
      .data_mem_bus_wait(scalar_dbus_wait),
      .data_mem_bus_ecause(dbus_ecause),

      .cop2_fromcop2_wait(cop2_fromcop2_wait),
      .cop2_fromcop2_en(cop2_fromcop2_en),
      .cop2_fromcop2(cop2_fromcop2),
      .cop2_tocop2_wait(cop2_tocop2_wait),
      .cop2_tocop2_en(cop2_tocop2_en),
      .cop2_tocop2(cop2_tocop2),

      .cop0_ext_cause_in(device_ecause),
      .cop0_badvaddr_in(badvaddr),
      .cop0_badvaddr_we(badvaddr_we),

      // PETES CHANGE for tracing
      .trc_addr(trc_addr),
      .trc_data(trc_data),
      .trc_we(trc_we),
      .trc_stall(trc_stall),
      .trc_pipestall(trc_pipestall),

      . nop10_q (rt_dataout)
      );
    reg scalar_cache_req_state;
    

   
    always@(*)begin
      case(scalar_cache_req_state)
        1'b0:begin
               scalar_dbus_wait = scalar_dbus_en;
             end
        1'b1:begin
               scalar_dbus_wait = 1'b0;
             end
      endcase
    end

    always@(posedge clk)begin
        if(!resetn)
          scalar_cache_req_state = 1'b0;
        else begin
          if(scalar_dbus_en & (scalar_cache_req_state == 1'b0))
             scalar_cache_req_state <= 1'b1;
          else
             scalar_cache_req_state <= 1'b0;
        end
    end

    always@(posedge clk)
      if(!resetn)
        ibus_en_r<=0;
      else if(~ibus_en_r || ~vpu_stalled)
        ibus_en_r<=ibus_en;

    /********************** Exception processing *********************/
    assign ibus_ecause=0;     //This is for instruction fetching exceptions
    assign dbus_ecause=0;     //This is for data access exceptions
    assign device_ecause=0;   //This is for external device interrupts

    //Register exception to create one pulsed write to badvaddr
    reg ibus_exception_r;
    reg dbus_exception_r;
    always@(posedge clk)
    begin
      ibus_exception_r<=(ibus_ecause!=0);
      dbus_exception_r<=(dbus_ecause!=0);
    end

    assign badvaddr=(ibus_ecause!=0) ? ibus_address : dbus_address;
    assign badvaddr_we=(ibus_ecause!=0  && !ibus_exception_r) ||
      (dbus_ecause!=0  && !dbus_exception_r);
    /********************** /Exception processing *********************/

    vpu v(
      .clk(clk),
      .resetn(resetn),

      // Instruction interface
      .instr(ibus_readdata),
      .instr_en(instr_en), // instr is valid and available
      .instr_wait(vpu_stalled),   // if high says vpu is not ready to receive

      .has_memop(vpu_has_memop),

      // For mtc2/ctc2 instructions
      .scalar_in(cop2_tocop2),
      .scalar_in_en(cop2_tocop2_en),
      .scalar_in_wait(cop2_tocop2_wait),

      // For cfc2 instructions
      .scalar_out(cop2_fromcop2),
      .scalar_out_en(cop2_fromcop2_en),
      .scalar_out_wait(cop2_fromcop2_wait),

      // AXI interface
      .M_AWID    (M_AWID    ),
      .M_AWADDR  (M_AWADDR  ),
      .M_AWLEN   (M_AWLEN   ),
      .M_AWSIZE  (M_AWSIZE  ),
      .M_AWBURST (M_AWBURST ),
      .M_AWLOCK  (M_AWLOCK  ),
      .M_AWCACHE (M_AWCACHE ),
      .M_AWPROT  (M_AWPROT  ),
      .M_AWQOS   (M_AWQOS   ),
      .M_AWVALID (M_AWVALID ),
      .M_AWREADY (M_AWREADY ),
      .M_WDATA   (M_WDATA   ),
      .M_WSTRB   (M_WSTRB   ),
      .M_WLAST   (M_WLAST   ),
      .M_WVALID  (M_WVALID  ),
      .M_WREADY  (M_WREADY  ),
      .M_ARID    (M_ARID    ),
      .M_ARADDR  (M_ARADDR  ),
      .M_ARLEN   (M_ARLEN   ),
      .M_ARSIZE  (M_ARSIZE  ),
      .M_ARBURST (M_ARBURST ),
      .M_ARLOCK  (M_ARLOCK  ),
      .M_ARCACHE (M_ARCACHE ),
      .M_ARPROT  (M_ARPROT  ),
      .M_ARQOS   (M_ARQOS   ),
      .M_ARVALID (M_ARVALID ),
      .M_ARREADY (M_ARREADY ),
      .M_RID     (M_RID     ),
      .M_RDATA   (M_RDATA   ),
      .M_RRESP   (M_RRESP   ),
      .M_RLAST   (M_RLAST   ),
      .M_RVALID  (M_RVALID  ),
      .M_RREADY  (M_RREADY  ),
      .M_BREADY  (M_BREADY  ),
      .M_BVALID  (M_BVALID  ),
      .M_BRESP   (M_BRESP   ),
      .M_BID     (M_BID     ),
      .S_AWID    (S_AWID   ), 
      .S_AWADDR  (S_AWADDR ), 
      .S_AWLEN   (S_AWLEN  ), 
      .S_AWSIZE  (S_AWSIZE ), 
      .S_AWBURST (S_AWBURST), 
      .S_AWLOCK  (S_AWLOCK ), 
      .S_AWCACHE (S_AWCACHE), 
      .S_AWPROT  (S_AWPROT ), 
      .S_AWQOS   (S_AWQOS  ), 
      .S_AWVALID (S_AWVALID), 
      .S_AWREADY (S_AWREADY), 
      .S_WDATA   (S_WDATA  ), 
      .S_WSTRB   (S_WSTRB  ), 
      .S_WLAST   (S_WLAST  ), 
      .S_WVALID  (S_WVALID ), 
      .S_WREADY  (S_WREADY ), 
      .S_ARID    (S_ARID   ), 
      .S_ARADDR  (S_ARADDR ), 
      .S_ARLEN   (S_ARLEN  ), 
      .S_ARSIZE  (S_ARSIZE ), 
      .S_ARBURST (S_ARBURST), 
      .S_ARLOCK  (S_ARLOCK ), 
      .S_ARCACHE (S_ARCACHE), 
      .S_ARPROT  (S_ARPROT ), 
      .S_ARQOS   (S_ARQOS  ), 
      .S_ARVALID (S_ARVALID), 
      .S_ARREADY (S_ARREADY), 
      .S_RID     (S_RID    ), 
      .S_RDATA   (S_RDATA  ), 
      .S_RRESP   (S_RRESP  ), 
      .S_RLAST   (S_RLAST  ), 
      .S_RVALID  (S_RVALID ), 
      .S_RREADY  (S_RREADY ), 
      .S_BREADY  (S_BREADY ), 
      .S_BVALID  (S_BVALID ), 
      .S_BRESP   (S_BRESP  ), 
      .S_BID     (S_BID    ),
      // Data memory interface

      .dbus_address(v_dbus_address),
      .dbus_readdata(v_dbus_readdata),
      .dbus_writedata(v_dbus_writedata),
      .dbus_byteen(v_dbus_byteen),
      .dbus_en(v_dbus_en),
      .dbus_we(v_dbus_wren),
      .dbus_cachematch(dbus_cachematch),
      .dbus_cachemiss(dbus_cachemiss),
      .dbus_prefetch(v_dbus_prefetch),
      .dbus_wait(v_dbus_wait),

      .dma_dbus_address	(dma_dbus_address), 
      .dma_dbus_readdata	(dma_dbus_readdata), 
      .dma_dbus_writedata	(dma_dbus_writedata),
      .dma_dbus_byteen	(dma_dbus_byteen),
      .dma_dbus_en	(dma_dbus_en),       
      .dma_dbus_wren	(dma_dbus_wren),     
      .dma_dbus_prefetch	(dma_dbus_prefetch), 
      .dma_dbus_wait	(dma_dbus_wait),     
      .dma_dbus_data_valid(dma_dbus_data_valid)
    );
    defparam v.LOG2DMEM_WRITEWIDTH=LOG2DCACHEWIDTHBITS,
             v.LOG2DMEM_READWIDTH=LOG2DCACHEWIDTHBITS;


  /********* Arbitrate between scalar SPREE and vector coprocessor *********/

  assign p_dbus_byteen_line=(p_dbus_byteen<<
    {p_dbus_address[LOG2DCACHEWIDTHBITS-3-1:2],2'b0});

  // Vector processor should take priority since it's request would have 
  // have been issued before the scalar's (since it has a deeper pipeline)

 // assign dbus_address= (v_dbus_en) ? v_dbus_address : p_dbus_address;
 // assign dbus_writedata= p_dbus_writedata;
 // assign dbus_byteen=  p_dbus_byteen;
 // assign dbus_writedata_line= (v_dbus_en) ? v_dbus_writedata : {DCACHEWIDTHBITS/32{p_dbus_writedata}};
 // assign dbus_byteen_line= (v_dbus_en) ? v_dbus_byteen : p_dbus_byteen_line;
 // assign dbus_wren= (v_dbus_en) ? v_dbus_wren : p_dbus_wren;
 // assign dbus_en=p_dbus_en || v_dbus_en;
 // assign dbus_prefetch= v_dbus_prefetch;


  assign dbus_address= (v_dbus_en) ? v_dbus_address : 0;
  assign dbus_writedata= 0;
  assign dbus_byteen=  0;
  assign dbus_writedata_line= (v_dbus_en) ? v_dbus_writedata : {DCACHEWIDTHBITS/32{32'h0}};
  assign dbus_byteen_line= (v_dbus_en) ? v_dbus_byteen : 0;
  assign dbus_wren= (v_dbus_en) ? v_dbus_wren : 0;
  assign dbus_en= v_dbus_en;
  assign dbus_prefetch= v_dbus_prefetch;

  assign p_dbus_readdata=dbus_readdata;
  assign v_dbus_readdata=dbus_readdata_line;
  //Loads/stores need to wait for vpu to finish with theirs - hence vpu_stalled
  assign p_dbus_wait=p_dbus_en&dbus_wait;
  assign v_dbus_wait=v_dbus_en&dbus_wait;

endmodule
/******
 *                         +--fill_we
 *     +------------------|&|--------------------+
 *     +-----| D$ |                              |
 * fill ---- |    |---|cacheaddr==filladdr|------+
 *                    |\      &&          |   
 *                    |/   cache dirty    |
 *
 * Must ensure that prefetched cache lines don't overwrite dirty lines (we
 * evict the dirty lines and write them out to memory, BUT if we're prefetching
 * the same memory location that is dirty in the cache we have to cancel that
 * prefetch
 */

//`include "mem_wbbuffer.v"

module mem_dcache_wb (

    resetn,

    // CPU data bus
    bus_clk,
    bus_address,
    bus_readdata,
    bus_readdata_line,
    bus_writedata,
    bus_byteen,
    bus_en,
    bus_wren,
    bus_dirty,
    bus_dirtywe,
    bus_wait,

    // Mem hierarchy
    mem_clk,

    mem_filladdr,   //For writing into cache
    mem_filldata,
    mem_fillwe,
    mem_fillrddirty,

    mem_wbaddr,   //For writing into memory from cache
    mem_wbdata,
    mem_wbwe,
    mem_wback,

    bus_flush,    // runs on mem_clk

    // Cache signals
    cache_hit,
    cache_miss,
 
    mem_dcache_address,
    mem_dcache_data,
    mem_dcache_out,
    mem_dcache_byteen,
    mem_dcache_wren
    );

// Only need to set THESE TWO PARAMETERS
parameter LOG2CACHELINESIZE=7;            // In bits, subtract 3 for bytes
parameter LOG2CACHEDEPTH=6;
parameter LOG2DRAMWIDTHBITS=7;            // In bits, subtract 3 for bytes

parameter CACHELINESIZE=2**LOG2CACHELINESIZE; 
parameter CACHEDEPTH=2**LOG2CACHEDEPTH;  
parameter DRAMWIDTHBITS=2**LOG2DRAMWIDTHBITS;

parameter TAGSIZE=32-LOG2CACHEDEPTH-LOG2CACHELINESIZE+3;

`define TAGRANGE 31:32-TAGSIZE
`define OFFSETRANGE LOG2CACHEDEPTH-1+LOG2CACHELINESIZE-3:LOG2CACHELINESIZE-3
// selects 32-bit word
`define OFFSETWORDRANGE LOG2CACHEDEPTH-1+LOG2CACHELINESIZE-3:2


input           resetn;

// CPU data bus
input           bus_clk;
input   [31:0]  bus_address;
output  [31:0]  bus_readdata;
output  [CACHELINESIZE-1:0] bus_readdata_line;
input   [CACHELINESIZE-1:0]  bus_writedata;
input   [CACHELINESIZE/8-1:0]   bus_byteen;
input           bus_en;
input           bus_wren;
input           bus_dirty;
input           bus_dirtywe;
output          bus_wait;
input           bus_flush;

// Mem hierarchy
input                       mem_clk;
input [31:0]                mem_filladdr;
input [CACHELINESIZE-1:0]   mem_filldata;
input                       mem_fillwe;
input                       mem_fillrddirty;

output [31:0]               mem_wbaddr; 
output [DRAMWIDTHBITS-1:0]  mem_wbdata;
output                      mem_wbwe;
input                       mem_wback;

// Cache signals
output cache_hit;
output cache_miss;

input  [31:0]                mem_dcache_address;
input  [CACHELINESIZE-1:0]   mem_dcache_data;
output [31:0]                mem_dcache_out;
input  [CACHELINESIZE/8-1:0] mem_dcache_byteen;
input                        mem_dcache_wren;


wire [31:0]              mem_dirtyaddr; 
wire [CACHELINESIZE-1:0] mem_dirtydata;
wire                     mem_dirtywe;

wire                     mem_validout;
wire                     mem_dirtyout;
wire [TAGSIZE-1:0]       mem_tagout;
wire [CACHELINESIZE-1:0] mem_lineout;

wire [TAGSIZE-1:0]       cache_tagout;
wire                     cache_validout;
wire [CACHELINESIZE-1:0] cache_dataout;

wire                     tagmatch;
reg  [32-1:0] bus_address_saved;
wire  [LOG2CACHEDEPTH-1:0] mem_ndx_saved;

wire  cache_hit;

reg  [LOG2CACHEDEPTH-1:0] count;

reg   [CACHELINESIZE/8-1:0]   bus_byteen_t;
wire [LOG2CACHELINESIZE-5-1:0] wordsel_saved;
reg  [31:0]  bus_readdata;
wire [CACHELINESIZE-1:0] bus_writedata_t;

reg [31:0]                  mem_wbaddr_r;  // captures address for writeback
reg [CACHELINESIZE-1:0]     mem_wbdata_r;
wire                        mem_wbwe_pulse;
wire                        mem_wbload;
reg [31:0]                  mem_filladdr_r; // don't prefetch over dirty lines
reg [31:0]                  mem_filladdr_r2;
wire                        t_mem_fillwe;

  always@(posedge mem_clk)
    if (bus_flush)
      count <= count + 1'b1;

  // Must flush for CACHEDEPTH cycles
//  altsyncram status
//    (
//     .clock0    (mem_clk),
//     .wren_a    ((bus_flush) ? 1'b1 : t_mem_fillwe),
//     .data_a    ((bus_flush) ? 1'b0 : 1'b1),
//     .address_a ((bus_flush) ? count : mem_filladdr[`OFFSETRANGE]),
//     .q_a       (mem_validout),
//     .clock1    (bus_clk),
//     .clocken1  (bus_en),
//     .address_b (bus_address[`OFFSETRANGE]),
//     .q_b       (cache_validout)
//    );
//  defparam
//   status.operation_mode = "BIDIR_DUAL_PORT",
//   status.width_a = 1, 
//   status.widthad_a = LOG2CACHEDEPTH,
//   status.width_b = 1,
//   status.widthad_b = LOG2CACHEDEPTH,
//   status.outdata_reg_a = "UNREGISTERED",
//   status.outdata_reg_b = "UNREGISTERED",
//   //status.rdcontrol_reg_a = "CLOCK0",
//   //status.address_reg_a = "CLOCK0",
//   status.rdcontrol_reg_b = "CLOCK1",
//   status.address_reg_b = "CLOCK1",
//   status.read_during_write_mode_mixed_ports = "OLD_DATA";
//
//  // Must flush for CACHEDEPTH cycles
//  altsyncram dirty
//    (
//     .clock0    (bus_clk),
//     .clocken0  (bus_en),
//     .address_a (bus_address[`OFFSETRANGE]),
//     .wren_a    (bus_dirtywe),
//     .data_a    (bus_dirty),
//
//     .clock1    (mem_clk),
//     .wren_b    (t_mem_fillwe),
//     .data_b    (1'b0),
//     .address_b (mem_filladdr[`OFFSETRANGE]),
//     .q_b       (mem_dirtyout)
//    );
//  defparam
//   dirty.operation_mode = "BIDIR_DUAL_PORT",
//   dirty.width_a = 1, 
//   dirty.widthad_a = LOG2CACHEDEPTH,
//   dirty.width_b = 1,
//   dirty.widthad_b = LOG2CACHEDEPTH,
//   dirty.outdata_reg_a = "UNREGISTERED",
//   dirty.outdata_reg_b = "UNREGISTERED",
//   //dirty.rdcontrol_reg_a = "CLOCK0",
//   //dirty.address_reg_a = "CLOCK0",
//   dirty.rdcontrol_reg_b = "CLOCK1",
//   dirty.address_reg_b = "CLOCK1",
//   dirty.read_during_write_mode_mixed_ports = "OLD_DATA";
//
//
//  altsyncram tags
//    (
//     .clock0    (mem_clk),
//     .wren_a    (t_mem_fillwe),
//     .data_a    (mem_filladdr[`TAGRANGE]),
//     .address_a (mem_filladdr[`OFFSETRANGE]),
//     .q_a       (mem_tagout),
//
//     .clock1    (bus_clk),
//     .clocken1  (bus_en),
//     .address_b (bus_address[`OFFSETRANGE]),
//     .q_b       (cache_tagout)
//    );
//  defparam
//   tags.operation_mode = "BIDIR_DUAL_PORT",
//   tags.width_a = TAGSIZE, 
//   tags.widthad_a = LOG2CACHEDEPTH,
//   tags.width_b = TAGSIZE,
//   tags.widthad_b = LOG2CACHEDEPTH,
//   tags.outdata_reg_a = "UNREGISTERED",
//   tags.outdata_reg_b = "UNREGISTERED",
//   //tags.rdcontrol_reg_a = "CLOCK0",
//   //tags.address_reg_a = "CLOCK0",
//   tags.rdcontrol_reg_b = "CLOCK1",
//   tags.address_reg_b = "CLOCK1",
//   tags.read_during_write_mode_mixed_ports = "OLD_DATA";

`ifdef USE_INHOUSE_LOGIC
 dpram1 data0 (
    .clk(bus_clk),
    .address_a(bus_address[28:7]),
 //   .address_a(bus_address[31:4]),
    .address_b(mem_dcache_address),
    .wren_a(bus_wren),
    .wren_b(mem_dcache_wren),
    .data_a(bus_writedata_t),
    .data_b(mem_dcache_data),
    .byteen_a(bus_byteen_t),
    .byteen_b(mem_dcache_byteen),
    .out_a(cache_dataout),
    .out_b(mem_dcache_out)
 );
 defparam 
 //   data0.AWIDTH=28,
    data0.AWIDTH=22,
    data0.NUM_WORDS = 67108864,
    data0.DWIDTH= 128;

`else
  altsyncram data
    (
     .clock0    (bus_clk),
     .clocken0  (bus_en),
     .wren_a    (bus_wren),
     .byteena_a (bus_byteen_t),
     .data_a    (bus_writedata_t),
     .address_a (bus_address[`OFFSETRANGE]),
     .q_a       (cache_dataout),
     .clock1    (mem_clk),
     .wren_b    (t_mem_fillwe),
     .data_b    (mem_filldata),
     .address_b (mem_filladdr[`OFFSETRANGE]),
     .q_b       (mem_lineout)
    );
  defparam
    data.operation_mode = "BIDIR_DUAL_PORT",
    data.width_a = CACHELINESIZE,                           //32-bit specific
    data.widthad_a = LOG2CACHEDEPTH,
    data.width_byteena_a = CACHELINESIZE/8,
    data.width_b = CACHELINESIZE,
    data.widthad_b = LOG2CACHEDEPTH,
    data.outdata_reg_a = "UNREGISTERED",
    data.outdata_reg_b = "UNREGISTERED",
    //data.rdcontrol_reg_a = "CLOCK0",
    //data.address_reg_a = "CLOCK0",
    data.rdcontrol_reg_b = "CLOCK1",
    data.address_reg_b = "CLOCK1";
`endif

    // Save offset to make sure still asking for same data
  always@(posedge bus_clk or negedge resetn)
    if (!resetn)
      bus_address_saved<=0;
    else 
      bus_address_saved<=bus_address;

//  register memtag(mem_filladdr[`OFFSETRANGE],mem_clk,resetn,1'b1,mem_ndx_saved);
//    defparam memtag.WIDTH=LOG2CACHEDEPTH;

//  assign tagmatch=(cache_tagout==bus_address_saved[`TAGRANGE]);

//  assign cache_hit = tagmatch && cache_validout;
  assign cache_hit = 1'b1;

  reg bus_en_r;
  always@(posedge bus_clk)
    bus_en_r<=bus_en;

//  assign cache_miss=bus_en_r && ~cache_hit;
  assign cache_miss= 1'b0;

//  assign bus_wait=bus_en_r & ~cache_hit;
  assign bus_wait= 1'b0;

  // DIRTY BIT processing
//  wire [LOG2CACHELINESIZE-3-1:0] zeros;
//  assign zeros=0;
//  assign mem_dirtydata=mem_lineout;
//  assign mem_dirtyaddr={mem_tagout,mem_ndx_saved,zeros}; 
//  assign mem_dirtywe=(mem_validout) & (mem_dirtyout);

  //Word muxing - assign byteena, datain, dataout
  assign wordsel_saved=bus_address_saved[LOG2CACHELINESIZE-3:3];


  always @*
      bus_byteen_t=bus_byteen;

  assign bus_writedata_t = bus_writedata;

  integer i;
  always @*
  begin
    bus_readdata<=cache_dataout[31:0];
    for (i=0; i<CACHELINESIZE/32; i=i+1)
      if (wordsel_saved==i)
        bus_readdata<=cache_dataout[32*i +: 32];
  end


  assign bus_readdata_line=cache_dataout;

  /********************* Write Back Buffer ***********************/

  /*
  * filladdr      ----Ox00----
  * fillrddirty   ____----____
  * wbwe_valid        ____----____
  * addrmatch     ------------0x00----
  */

  //reg wbwe_valid;
  //reg wbwe_valid_r;
  //always @(posedge mem_clk)
  //  wbwe_valid<=mem_fillrddirty;
  //always @(posedge mem_clk)
  //  wbwe_valid_r<=wbwe_valid;

  //always@(posedge mem_clk)
  //  mem_filladdr_r<={mem_filladdr[31:LOG2CACHELINESIZE-3],zeros};
  //always@(posedge mem_clk)
  //  mem_filladdr_r2<=mem_filladdr_r;

  ////Assume mem_dirtywe is ready
  //reg mem_dirtyaddrmatch;
  //reg mem_dirtywe_r;
  //reg [31:0] mem_dirtyaddr_r;
  //always@(posedge mem_clk)
  //  mem_dirtyaddr_r=mem_dirtyaddr;
  //always@*
  //  mem_dirtyaddrmatch=(mem_filladdr_r2[`TAGRANGE]==mem_dirtyaddr_r[`TAGRANGE]);
  //always@(posedge mem_clk)
  //  mem_dirtywe_r=mem_dirtywe;

  ////wbwe_valid - pulse from memory system saying new cache line is about to come
  ////mem_dirtywe - tells whether cache line currently targetted is dirty
  ////mem_dirtyaddrmatch - if prefetching data that already is cached and is
  ////dirty do not put in WB buffer (since we don't put it in cache)
  //assign mem_wbwe_pulse = wbwe_valid_r && mem_dirtywe_r && !mem_dirtyaddrmatch;

  ////Don't overwrite dirty line with prefetched line if it's the same address
  ////Cache line has to be >= 4 x Dram size to do this
  //reg mem_fillwe_block;
  //always@(posedge mem_clk)
  //  mem_fillwe_block = (mem_dirtywe_r && mem_dirtyaddrmatch);
  //assign t_mem_fillwe = mem_fillwe && !mem_fillwe_block;

  //assign mem_wbload=mem_wbwe_pulse; 
  //always @(posedge mem_clk)
  //  mem_wbaddr_r<=mem_dirtyaddr; 
  //always @(posedge mem_clk)
  //  mem_wbdata_r<=mem_dirtydata; 

  //reg resetn133;
  //always @(posedge mem_clk)
  //  resetn133<=resetn;

  //mem_wbbuffer wbbuffer(
  //  .clk(mem_clk),
  //  .resetn(resetn133),
  //  .wbaddr(mem_wbaddr_r),
  //  .wbdata(mem_wbdata_r),
  //  .wbwe(mem_wbload),
  //  .wbfull(),
  //  .mem_address(mem_wbaddr),
  //  .mem_writedata(mem_wbdata),
  //  .mem_wren(mem_wbwe),
  //  .mem_ack(mem_wback));
  //defparam 
  //  wbbuffer.LOG2WBDATAWIDTH=LOG2CACHELINESIZE,
  //  wbbuffer.LOG2MEMDATAWIDTH=LOG2DRAMWIDTHBITS;

endmodule

/******
 * 1-cycle cache (not including initial registering)
 *    0th cycle: register addresses into block RAMs
 *    1st cycle: Read out tags and status and do comparison
 */

module mem_icache (

    resetn,

    // CPU data bus
    bus_clk,
    bus_address,
    bus_readdata,
    bus_writedata,
    bus_byteen,
    bus_en,
    bus_wren,
    bus_wait,

    // Mem hierarchy
    mem_clk,
    mem_filladdr,
    mem_filldata,
    mem_fillwe,

    bus_flush,    // runs on mem_clk

    // Cache signals
    cache_hit,
    cache_miss,

    mem_icache_address,
    mem_icache_data,
    mem_icache_out,
    mem_icache_byteen,
    mem_icache_wren
    );

                // In bits, subtract 3 for bytes
parameter LOG2CACHELINESIZE=7;
parameter CACHELINESIZE=2**LOG2CACHELINESIZE; // 128

parameter LOG2CACHEDEPTH=6;
parameter CACHEDEPTH=2**LOG2CACHEDEPTH;  // 64

parameter TAGSIZE=32-LOG2CACHEDEPTH-LOG2CACHELINESIZE+3;

`define TAGRANGE 31:32-TAGSIZE
`define OFFSETRANGE LOG2CACHEDEPTH-1+LOG2CACHELINESIZE-3:LOG2CACHELINESIZE-3
// selects 32-bit word
`define OFFSETWORDRANGE LOG2CACHEDEPTH-1+LOG2CACHELINESIZE-3:2


input           resetn;

// CPU data bus
input           bus_clk;
input   [31:0]  bus_address;
output  [31:0]  bus_readdata;
input   [31:0]  bus_writedata;
input   [3:0]   bus_byteen;
input           bus_en;
input           bus_wren;
output          bus_wait;
input           bus_flush;

// Mem hierarchy
input                       mem_clk;
input [31:0]                mem_filladdr;
input [CACHELINESIZE-1:0]   mem_filldata;
input                       mem_fillwe;

// Cache signals
output cache_hit;
output cache_miss;

wire [TAGSIZE-1:0]       cache_tagout;
wire                     cache_validout;
wire [32-1:0]            cache_dataout;
wire [CACHELINESIZE-1:0] cache_lineout;

input  [31:0]                mem_icache_address;
input  [CACHELINESIZE-1:0]   mem_icache_data;
output [31:0]                mem_icache_out;
input  [CACHELINESIZE/8-1:0] mem_icache_byteen;
input                        mem_icache_wren;

wire                     tagmatch;
reg  [32-1:0] bus_address_saved;

parameter [1:0] S_IDLE=2'b00, S_TAGLOOKUP=2'b01, S_RESULT=2'b10;
reg [1:0] cache_state;

wire  cache_hit;

reg  [LOG2CACHEDEPTH-1:0] count;

  always@(posedge mem_clk)
    if (bus_flush)
      count <= count + 1'b1;

//  // Must flush for CACHEDEPTH cycles
//  altsyncram status
//    (
//     .clock0    (mem_clk),
//     .wren_a    ((bus_flush) ? 1'b1 : mem_fillwe),
//     .data_a    ((bus_flush) ? 0 : 1),
//     .address_a ((bus_flush) ? count : mem_filladdr[`OFFSETRANGE]),
//     .clock1    (bus_clk),
//     .clocken1  (bus_en),
//     .address_b (bus_address[`OFFSETRANGE]),
//     .q_b       (cache_validout)
//    );
//  defparam
//   status.operation_mode = "DUAL_PORT",
//   status.width_a = 1, 
//   status.widthad_a = LOG2CACHEDEPTH,
//   status.width_b = 1,
//   status.widthad_b = LOG2CACHEDEPTH,
//   status.outdata_reg_a = "UNREGISTERED",
//   status.outdata_reg_b = "UNREGISTERED",
//   //status.rdcontrol_reg_a = "CLOCK0",
//   status.rdcontrol_reg_b = "CLOCK1",
//   //status.address_reg_a = "CLOCK0",
//   status.address_reg_b = "CLOCK1",
//   status.read_during_write_mode_mixed_ports = "OLD_DATA";
//
//
//  altsyncram tags
//    (
//     .clock0    (mem_clk),
//     .wren_a    (mem_fillwe),
//     .data_a    (mem_filladdr[`TAGRANGE]),
//     .address_a (mem_filladdr[`OFFSETRANGE]),
//
//     .clock1    (bus_clk),
//     .clocken1  (bus_en),
//     .address_b (bus_address[`OFFSETRANGE]),
//     .q_b       (cache_tagout)
//    );
//  defparam
//   tags.operation_mode = "DUAL_PORT",
//   tags.width_a = TAGSIZE, 
//   tags.widthad_a = LOG2CACHEDEPTH,
//   tags.width_b = TAGSIZE,
//   tags.widthad_b = LOG2CACHEDEPTH,
//   tags.outdata_reg_a = "UNREGISTERED",
//   tags.outdata_reg_b = "UNREGISTERED",
//   //tags.rdcontrol_reg_a = "CLOCK0",
//   tags.rdcontrol_reg_b = "CLOCK1",
//   //tags.address_reg_a = "CLOCK0",
//   tags.address_reg_b = "CLOCK1",
//   tags.read_during_write_mode_mixed_ports = "OLD_DATA";

`ifdef USE_INHOUSE_LOGIC
 dpram1 data1 (
    .clk(bus_clk),
    .address_a(bus_address[27:2]),
    .address_b(mem_icache_address[27:2]),
    .wren_a(1'b0),
    .wren_b(mem_icache_wren),
    .data_a(0),
    .data_b(mem_icache_data),
    .byteen_a(-1),
    .byteen_b(mem_icache_byteen),
    .out_a(cache_dataout),
    .out_b(mem_icache_out)
 );
 defparam 
    data1.AWIDTH=26,
    data1.NUM_WORDS = 67108864,
    data1.DWIDTH= 32;

`else

  altsyncram data
    (
     .clock0    (bus_clk),
     .clocken0  (bus_en),
     .wren_a    (bus_wren),
     //.byteena_a (bus_byteen),
     .data_a    (bus_writedata),
     .address_a (bus_address[`OFFSETWORDRANGE]),
     .q_a       (cache_dataout),
     .clock1    (mem_clk),
     .wren_b    (mem_fillwe),
     .data_b    (mem_filldata),
     .address_b (mem_filladdr[`OFFSETRANGE])
     //.q_b       (cache_lineout)
    );
  defparam
    data.operation_mode = "BIDIR_DUAL_PORT",
    data.width_a = 32,                                   //32-bit specific
    data.widthad_a = LOG2CACHEDEPTH+(LOG2CACHELINESIZE-5),  //32-bit specific
    //data.WIDTH_BYTEENA_A = 4,
    data.width_b = CACHELINESIZE,
    data.widthad_b = LOG2CACHEDEPTH,
    data.outdata_reg_a = "UNREGISTERED",
    data.outdata_reg_b = "UNREGISTERED",
    //data.rdcontrol_reg_a = "CLOCK0",
    data.rdcontrol_reg_b = "CLOCK1",
    //data.address_reg_a = "CLOCK0",
    data.address_reg_b = "CLOCK1";

`endif

  assign bus_readdata=cache_dataout;

    // Save offset to make sure still asking for same data
  always@(posedge bus_clk or negedge resetn)
    if (!resetn)
      bus_address_saved<=0;
    else 
      bus_address_saved<=bus_address;

//  assign tagmatch=(cache_tagout==bus_address_saved[`TAGRANGE]);

//  assign cache_hit = tagmatch && cache_validout;
  assign cache_hit = 1'b1;

  assign cache_miss=bus_en && ~cache_hit;

  assign bus_wait=bus_en & ~cache_hit;


endmodule


//`include "core.v"
//`include "mem_icache.v"
//`include "mem_dcache_wb.v"

module processor (
    clk,
    mem_clk,
    resetn,
    rt_dataout, // Dummy output - use this to prevent design from being
                // synthesized away if using on-chip memory

    ibus_en,
    ibus_address,
    ibus_wait,

    // Instruction cache fill - on DRAM clock 133MHz
    imem_filladdr,
    imem_filldata,
    imem_fillwe,
    
    scalar_dbus_address,
    scalar_dbus_en,
    scalar_dbus_writedata,
    
    dbus_address,
    dbus_readdata,
    dbus_writedata,
    dbus_byteen,
    dbus_en,
    dbus_wren,
    dbus_prefetch,
    dbus_wait,    //Goes high 1-cycle after dbus_en

    // PETES CHANGE for tracing
    trc_addr,
    trc_data,
    trc_we,
    trc_stall,
    trc_pipestall,

    // Data fill/WB 133MHz
    dmem_filladdr,
    dmem_filldata,
    dmem_fillwe,
    dmem_fillrddirty,
    dmem_wbaddr,
    dmem_wbdata,
    dmem_wbwe,
    dmem_wback,

    mem_dcache_address,
    mem_dcache_data,
    mem_dcache_out,
    mem_dcache_byteen,
    mem_dcache_wren,

    mem_icache_address,
    mem_icache_data,
    mem_icache_out,
    mem_icache_byteen,
    mem_icache_wren,

    //DMA interface
    dma_dbus_address,   
    dma_dbus_readdata,  
    dma_dbus_writedata, 
    dma_dbus_byteen,
    dma_dbus_en,        
    dma_dbus_wren,      
    dma_dbus_prefetch,  
    dma_dbus_wait,      
    dma_dbus_data_valid,
    // AXI Interface
    M_AWID    ,
    M_AWADDR  ,
    M_AWLEN   ,
    M_AWSIZE  ,
    M_AWBURST ,
    M_AWLOCK  ,
    M_AWCACHE ,
    M_AWPROT  ,
    M_AWQOS   ,
    M_AWVALID ,
    M_AWREADY ,
    M_WDATA   ,
    M_WSTRB   ,
    M_WLAST   ,
    M_WVALID  ,
    M_WREADY  ,
    M_ARID    ,
    M_ARADDR  ,
    M_ARLEN   ,
    M_ARSIZE  ,
    M_ARBURST ,
    M_ARLOCK  ,
    M_ARCACHE ,
    M_ARPROT  ,
    M_ARQOS   ,
    M_ARVALID ,
    M_ARREADY ,
    M_RID     ,
    M_RDATA   ,
    M_RRESP   ,
    M_RLAST   ,
    M_RVALID  ,
    M_RREADY  ,
    M_BREADY  ,
    M_BVALID  ,
    M_BRESP   ,
    M_BID     ,

    S_AWID   , 
    S_AWADDR , 
    S_AWLEN  , 
    S_AWSIZE , 
    S_AWBURST, 
    S_AWLOCK , 
    S_AWCACHE, 
    S_AWPROT , 
    S_AWQOS  , 
    S_AWVALID, 
    S_AWREADY, 
    S_WDATA  , 
    S_WSTRB  , 
    S_WLAST  , 
    S_WVALID , 
    S_WREADY , 
    S_ARID   , 
    S_ARADDR , 
    S_ARLEN  , 
    S_ARSIZE , 
    S_ARBURST, 
    S_ARLOCK , 
    S_ARCACHE, 
    S_ARPROT , 
    S_ARQOS  , 
    S_ARVALID, 
    S_ARREADY, 
    S_RID    , 
    S_RDATA  , 
    S_RRESP  , 
    S_RLAST  , 
    S_RVALID , 
    S_RREADY , 
    S_BREADY , 
    S_BVALID , 
    S_BRESP  , 
    S_BID 
    );


parameter LOG2DCACHEWIDTHBITS=`LOG2DCACHEWIDTHBITS;
parameter DCACHEWIDTHBITS=2**LOG2DCACHEWIDTHBITS;
parameter LOG2DCACHEDEPTH=`LOG2DCACHEDEPTHBYTES -(`LOG2DCACHEWIDTHBITS-3);

parameter LOG2ICACHEWIDTHBITS=`LOG2ICACHEWIDTHBITS;
parameter ICACHEWIDTHBITS=2**LOG2ICACHEWIDTHBITS;
parameter LOG2ICACHEDEPTH=`LOG2ICACHEDEPTHBYTES -(`LOG2ICACHEWIDTHBITS-3);

parameter LOG2DRAMWIDTHBITS=`LOG2DRAMWIDTHBITS;
parameter DRAMWIDTHBITS=2**LOG2DRAMWIDTHBITS;

// AXI params


//
output [31:0] scalar_dbus_writedata;
output scalar_dbus_en;
output [31:0] scalar_dbus_address;

input              clk;
input              mem_clk;
input              resetn;
output [31:0]      rt_dataout;

output        ibus_en;             // Instruction bus signals
output [31:0] ibus_address;
input         ibus_wait;

input [31:0]    imem_filladdr;
input [ICACHEWIDTHBITS-1:0]   imem_filldata;
input           imem_fillwe;

output [31:0] dbus_address;    // Data bus signals
input  [31:0] dbus_readdata;
output [31:0] dbus_writedata;
output [3:0]  dbus_byteen;
output        dbus_en;
output        dbus_wren;
output [31:0] dbus_prefetch;
input         dbus_wait;

    // Mem hierarchy
input [31:0]    dmem_filladdr;
input [DCACHEWIDTHBITS-1:0]   dmem_filldata;
input           dmem_fillwe;
input           dmem_fillrddirty;

output [31:0]   dmem_wbaddr;
output [DRAMWIDTHBITS-1:0]  dmem_wbdata;
output          dmem_wbwe;
input           dmem_wback;

// PETES CHANGE for tracing
output  [ 4 : 0 ]   trc_addr;
output  [ 31 : 0 ]  trc_data;
output              trc_we;
input               trc_stall;
output              trc_pipestall;

input  [31:0]                  mem_dcache_address;
input  [DCACHEWIDTHBITS-1:0]   mem_dcache_data;
output [DCACHEWIDTHBITS-1:0]   mem_dcache_out;
input  [DCACHEWIDTHBITS/8-1:0] mem_dcache_byteen;
input                          mem_dcache_wren;

input  [31:0]                  mem_icache_address;
input  [31-1:0]   mem_icache_data;
output [31-1:0]   mem_icache_out;
input  [32/8-1:0] mem_icache_byteen;
input                          mem_icache_wren;

// DMA signals
output [31:0]                   dma_dbus_address;   
input  [DCACHEWIDTHBITS-1:0]    dma_dbus_readdata;  
output [DCACHEWIDTHBITS-1:0]    dma_dbus_writedata; 
output [DCACHEWIDTHBITS/8-1:0]  dma_dbus_byteen;
output                          dma_dbus_en;        
output                          dma_dbus_wren;      
output                          dma_dbus_prefetch;  
input                           dma_dbus_wait;      
input                           dma_dbus_data_valid;

// AXI interface
 output  [6 - 1:0]     M_AWID;                                                 
 output  [16- 1:0]     M_AWADDR;                                    
 output  [7:0]         M_AWLEN;                                                    
 output  [2:0]         M_AWSIZE;                                                   
 output  [1:0]         M_AWBURST;                                                  
 output                M_AWLOCK;                                                         
 output  [3:0]         M_AWCACHE;                                                  
 output  [2:0]         M_AWPROT;                                                   
 output  [3:0]         M_AWQOS;                                                    
 output                M_AWVALID;                                                        
 input                 M_AWREADY;                                                        
 output  [128-1 : 0]   M_WDATA;                                     
 output  [128/8-1 : 0] M_WSTRB;                                   
 output                M_WLAST;                                                          
 output                M_WVALID;                                                         
 input                 M_WREADY;                                                         

 output  [6-1 : 0]     M_ARID;                                                 
 output  [16-1 : 0]    M_ARADDR;                                    
 output  [7 : 0]       M_ARLEN;                                                  
 output  [2 : 0]       M_ARSIZE;                                                 
 output  [1 : 0]       M_ARBURST;                                                
 output                M_ARLOCK;                                                         
 output  [3 : 0]       M_ARCACHE;                                                
 output  [2 : 0]       M_ARPROT;                                                 
 output  [3 : 0]       M_ARQOS;                                                  
 output                M_ARVALID;                                                        
 input                 M_ARREADY;                                                        
 input   [6-1 : 0]     M_RID;                                         	    
 input   [128-1 : 0]   M_RDATA;                                     
 input   [1 : 0]       M_RRESP;                                                  
 input                 M_RLAST;                                                          
 input                 M_RVALID;                                                         
 output                M_RREADY;                                                         

 output                M_BREADY;                                                         
 input                 M_BVALID;                                                         
 input   [1 : 0]       M_BRESP;                                                  
 input   [6-1 : 0]     M_BID;                                                   

 input  [6 - 1:0]       S_AWID;                                                 
 input  [16- 1:0]       S_AWADDR;                                    
 input  [7:0]           S_AWLEN;                                                    
 input  [2:0]           S_AWSIZE;                                                   
 input  [1:0]           S_AWBURST;                                                  
 input                  S_AWLOCK;                                                         
 input  [3:0]           S_AWCACHE;                                                  
 input  [2:0]           S_AWPROT;                                                   
 input  [3:0]           S_AWQOS;                                                    
 input                  S_AWVALID;                                                        
 output                 S_AWREADY;                                                        
 input  [128-1 : 0]     S_WDATA;                                     
 input  [128/8-1 : 0]   S_WSTRB;                                   
 input                  S_WLAST;                                                          
 input                  S_WVALID;                                                         
 output                 S_WREADY;                                                         

 input  [6-1 : 0]       S_ARID;                                                 
 input  [16-1 : 0]      S_ARADDR;                                    
 input  [7 : 0]         S_ARLEN;                                                  
 input  [2 : 0]         S_ARSIZE;                                                 
 input  [1 : 0]         S_ARBURST;                                                
 input                  S_ARLOCK;                                                         
 input  [3 : 0]         S_ARCACHE;                                                
 input  [2 : 0]         S_ARPROT;                                                 
 input  [3 : 0]         S_ARQOS;                                                  
 input                  S_ARVALID;                                                        
 output                 S_ARREADY;                                                        
 output   [6-1 : 0]     S_RID;                                         	    
 output   [128-1 : 0]   S_RDATA;                                     
 output   [1 : 0]       S_RRESP;                                                  
 output                 S_RLAST;                                                          
 output                 S_RVALID;                                                         
 input                  S_RREADY;                                                         

 input                  S_BREADY;                                                         
 output                 S_BVALID;                                                         
 output   [1 : 0]       S_BRESP;                                                  
 output   [6-1 : 0]     S_BID;                                                  

// 
  wire [31:0] dcpu_address;    // Processor's data bus signals
  wire [31:0] dcpu_writedata;
  wire [3:0]  dcpu_byteen;
  wire        dcpu_en;
  wire        dcpu_wren;
  wire [31:0] dcpu_prefetch;    // Processor's data bus signals
  wire [31:0] dcpu_readdata;
  wire [DCACHEWIDTHBITS-1:0] dcpu_readdata_line;
  wire [DCACHEWIDTHBITS-1:0] dcpu_writedata_line;
  wire [DCACHEWIDTHBITS/8-1:0]  dcpu_byteen_line;
  wire         dcpu_wait;


  wire        icache_en;             // Instruction bus signals
  wire [31:0] icache_address;
  wire [31:0] icache_readdata;
  wire        icache_wait;
  wire icache_hit;
  wire icache_miss;


  wire [31:0] dcache_address;    // Processor's data bus signals
  reg [DCACHEWIDTHBITS-1:0] dcache_writedata;
  reg [DCACHEWIDTHBITS/8-1:0]  dcache_byteen;
  wire        dcache_en;
  wire        dcache_wren;
  wire  [31:0] dcache_readdata;
  wire  [DCACHEWIDTHBITS-1:0] dcache_readdata_line;
  wire         dcache_wait;

  wire        dcache_hit;
  wire        dcache_miss;

  reg [3:0] dcache_state,icache_state;
  parameter [2:0] CACHE_IDLE=3'b000, 
          CACHE_TAGLOOKUP=3'b001,
          CACHE_ISSUE=3'b010,
          CACHE_STOREMISS=3'b011,
          CACHE_LOADMISS=3'b100;

  reg [4-1:0]  icache_timeout;
  reg [4-1:0]  dcache_timeout;

 

/*************************  AXI interface  **************************/


/*************************  Processor Core **************************/

  wire [31:0]    scalar_dbus_address; 
  wire [31:0]    scalar_dbus_readdata; 
  wire [31:0]    scalar_dbus_writedata;
  wire [3:0]    scalar_dbus_byteen; 
  wire     scalar_dbus_en;       
  wire     scalar_dbus_wren;    

 

  core c (
    .clk(clk),
    .resetn(resetn),
    .rt_dataout(rt_dataout),
    .ibus_en(icache_en),
    .ibus_address(icache_address),
    .ibus_readdata((icache_hit) ? icache_readdata : 0),
    .ibus_wait(icache_wait),

    // PETES CHANGE for tracing
    .trc_addr(trc_addr),
    .trc_data(trc_data),
    .trc_we(trc_we),
    .trc_stall(trc_stall),
    .trc_pipestall(trc_pipestall),

    // AXI interface
    .M_AWID    (M_AWID    ),
    .M_AWADDR  (M_AWADDR  ),
    .M_AWLEN   (M_AWLEN   ),
    .M_AWSIZE  (M_AWSIZE  ),
    .M_AWBURST (M_AWBURST ),
    .M_AWLOCK  (M_AWLOCK  ),
    .M_AWCACHE (M_AWCACHE ),
    .M_AWPROT  (M_AWPROT  ),
    .M_AWQOS   (M_AWQOS   ),
    .M_AWVALID (M_AWVALID ),
    .M_AWREADY (M_AWREADY ),
    .M_WDATA   (M_WDATA   ),
    .M_WSTRB   (M_WSTRB   ),
    .M_WLAST   (M_WLAST   ),
    .M_WVALID  (M_WVALID  ),
    .M_WREADY  (M_WREADY  ),
    .M_ARID    (M_ARID    ),
    .M_ARADDR  (M_ARADDR  ),
    .M_ARLEN   (M_ARLEN   ),
    .M_ARSIZE  (M_ARSIZE  ),
    .M_ARBURST (M_ARBURST ),
    .M_ARLOCK  (M_ARLOCK  ),
    .M_ARCACHE (M_ARCACHE ),
    .M_ARPROT  (M_ARPROT  ),
    .M_ARQOS   (M_ARQOS   ),
    .M_ARVALID (M_ARVALID ),
    .M_ARREADY (M_ARREADY ),
    .M_RID     (M_RID     ),
    .M_RDATA   (M_RDATA   ),
    .M_RRESP   (M_RRESP   ),
    .M_RLAST   (M_RLAST   ),
    .M_RVALID  (M_RVALID  ),
    .M_RREADY  (M_RREADY  ),
    .M_BREADY  (M_BREADY  ),
    .M_BVALID  (M_BVALID  ),
    .M_BRESP   (M_BRESP   ),
    .M_BID     (M_BID     ),
    .S_AWID    (S_AWID   ), 
    .S_AWADDR  (S_AWADDR ), 
    .S_AWLEN   (S_AWLEN  ), 
    .S_AWSIZE  (S_AWSIZE ), 
    .S_AWBURST (S_AWBURST), 
    .S_AWLOCK  (S_AWLOCK ), 
    .S_AWCACHE (S_AWCACHE), 
    .S_AWPROT  (S_AWPROT ), 
    .S_AWQOS   (S_AWQOS  ), 
    .S_AWVALID (S_AWVALID), 
    .S_AWREADY (S_AWREADY), 
    .S_WDATA   (S_WDATA  ), 
    .S_WSTRB   (S_WSTRB  ), 
    .S_WLAST   (S_WLAST  ), 
    .S_WVALID  (S_WVALID ), 
    .S_WREADY  (S_WREADY ), 
    .S_ARID    (S_ARID   ), 
    .S_ARADDR  (S_ARADDR ), 
    .S_ARLEN   (S_ARLEN  ), 
    .S_ARSIZE  (S_ARSIZE ), 
    .S_ARBURST (S_ARBURST), 
    .S_ARLOCK  (S_ARLOCK ), 
    .S_ARCACHE (S_ARCACHE), 
    .S_ARPROT  (S_ARPROT ), 
    .S_ARQOS   (S_ARQOS  ), 
    .S_ARVALID (S_ARVALID), 
    .S_ARREADY (S_ARREADY), 
    .S_RID     (S_RID    ), 
    .S_RDATA   (S_RDATA  ), 
    .S_RRESP   (S_RRESP  ), 
    .S_RLAST   (S_RLAST  ), 
    .S_RVALID  (S_RVALID ), 
    .S_RREADY  (S_RREADY ), 
    .S_BREADY  (S_BREADY ), 
    .S_BVALID  (S_BVALID ), 
    .S_BRESP   (S_BRESP  ), 
    .S_BID     (S_BID    ),
    
    .scalar_dbus_address  (scalar_dbus_address  ),
    .scalar_dbus_readdata (scalar_dbus_readdata ),
    .scalar_dbus_writedata(scalar_dbus_writedata),
    .scalar_dbus_byteen   (scalar_dbus_byteen   ),
    .scalar_dbus_en       (scalar_dbus_en       ),
    .scalar_dbus_wren     (scalar_dbus_wren     ),

    .dbus_address(dcpu_address),
    .dbus_readdata(dcpu_readdata),
    .dbus_writedata(dcpu_writedata),
    .dbus_byteen(dcpu_byteen),
    .dbus_readdata_line(dcpu_readdata_line),
    .dbus_writedata_line(dcpu_writedata_line),
    .dbus_byteen_line(dcpu_byteen_line),
    .dbus_en(dcpu_en),
    .dbus_wren(dcpu_wren),
    .dbus_cachematch(dcache_hit),
    .dbus_cachemiss(dcache_miss),
    .dbus_prefetch(dcpu_prefetch),
    .dbus_wait(dcpu_wait),
    
    .dma_dbus_address	(dma_dbus_address), 
    .dma_dbus_readdata	(dma_dbus_readdata), 
    .dma_dbus_writedata	(dma_dbus_writedata),
    .dma_dbus_byteen	(dma_dbus_byteen),
    .dma_dbus_en	(dma_dbus_en),       
    .dma_dbus_wren	(dma_dbus_wren),     
    .dma_dbus_prefetch	(dma_dbus_prefetch), 
    .dma_dbus_wait	(dma_dbus_wait),     
    .dma_dbus_data_valid (dma_dbus_data_valid)
    );


/*************************  Instr Cache **************************/

  reg cache_flush;
  always@(posedge mem_clk)
    cache_flush<=~resetn;

  mem_icache icache1 (
    .resetn(resetn),

    // CPU data bus
    .bus_clk(clk),
    .bus_address(icache_address),
    .bus_readdata(icache_readdata),
    .bus_en(icache_en),
    .bus_wait(icache_wait),

    // Mem hierarchy
    .mem_clk(mem_clk),
    .mem_filladdr(imem_filladdr),
    .mem_filldata(imem_filldata),
    .mem_fillwe(imem_fillwe),
    .bus_flush(cache_flush), // Runs at 133 MHz

    // Cache signals
    .cache_hit(icache_hit),
    .cache_miss(icache_miss),

    .mem_icache_address  (scalar_dbus_address),
    .mem_icache_data     (scalar_dbus_writedata),
    .mem_icache_out      (scalar_dbus_readdata),
    .mem_icache_byteen   (scalar_dbus_byteen),
    .mem_icache_wren     (scalar_dbus_wren)
    );
    defparam icache1.LOG2CACHEDEPTH=LOG2ICACHEDEPTH,
      icache1.LOG2CACHELINESIZE=LOG2ICACHEWIDTHBITS;

  always@(posedge clk)
    if (!resetn)
      icache_state<=CACHE_IDLE;
    else
      case(icache_state)
        CACHE_IDLE:
          icache_state<= (icache_en && ~icache_address[31]) ? CACHE_TAGLOOKUP : 
                                                          CACHE_IDLE;
        CACHE_TAGLOOKUP:
          icache_state<= (icache_hit || ~icache_en) ? CACHE_IDLE : CACHE_ISSUE;
        CACHE_ISSUE:
          icache_state<= (ibus_wait) ? CACHE_ISSUE : CACHE_LOADMISS;
        CACHE_LOADMISS:
          icache_state<= (icache_hit || (|icache_timeout)==0) ?  CACHE_IDLE : 
                              CACHE_LOADMISS;
        default:
          icache_state<=CACHE_IDLE;
      endcase

      // Timeout if ibus_wait has been low for 4 cycles
  always@(posedge clk)
    if (icache_state==CACHE_ISSUE)
      icache_timeout<={4{1'b1}};
    else
      icache_timeout<=(ibus_wait<<3) | (icache_timeout>>1);

  // Send single pulse to bus, unless already a request stalling
  assign ibus_en= (icache_state==CACHE_ISSUE);

  assign ibus_address=icache_address;


/*************************  Data Cache **************************/


  assign dcache_address=dcpu_address;
  //Since we know we need 1st cycle just to check tags, we register the writes
  always@(posedge clk)
  begin
    dcache_writedata<=dcpu_writedata_line;
    dcache_byteen<=dcpu_byteen_line;
  end
  assign dcache_wren = dcpu_wren&dcache_hit&
        (dcache_state==CACHE_TAGLOOKUP || dcache_state==CACHE_STOREMISS);
  assign dcache_en = dcpu_en;

  mem_dcache_wb dcache1 (
    .resetn(resetn),

    // CPU data bus    
    .bus_clk(clk),
    .bus_address(dcache_address),
    .bus_writedata(dcache_writedata),
    .bus_byteen(dcache_byteen),
    .bus_wren(dcache_wren),
    .bus_dirty(1'b1),   //Every store is dirty (even on store miss)
    .bus_dirtywe(dcache_wren),
    .bus_readdata(dcache_readdata),
    .bus_readdata_line(dcache_readdata_line),
    .bus_en(dcache_en),

    // Mem hierarchy
    .mem_clk(mem_clk),
    .mem_filladdr(dmem_filladdr),
    .mem_filldata(dmem_filldata),
    .mem_fillwe(dmem_fillwe),
    .mem_fillrddirty(dmem_fillrddirty),
    .bus_flush(cache_flush), // Runs at 133 MHz

    // Write back outputs for writing into DRAM, wbwe accounts for dirty bit
    .mem_wbaddr(dmem_wbaddr),
    .mem_wbdata(dmem_wbdata),
    .mem_wbwe(dmem_wbwe),
    .mem_wback(dmem_wback),

    // Cache signals
    .cache_hit(dcache_hit), //Checks only match - doesn't consider if en was on
    .cache_miss(dcache_miss), //Checks if not in the cache AND if enabled

    .mem_dcache_address(mem_dcache_address),
    .mem_dcache_data(mem_dcache_data),
    .mem_dcache_out(mem_dcache_out),
    .mem_dcache_byteen(mem_dcache_byteen),
    .mem_dcache_wren(mem_dcache_wren)
    );
    defparam 
      dcache1.LOG2DRAMWIDTHBITS=LOG2DRAMWIDTHBITS,
      dcache1.LOG2CACHELINESIZE=LOG2DCACHEWIDTHBITS,
      dcache1.LOG2CACHEDEPTH=LOG2DCACHEDEPTH;

  reg [32-1:0] dcpu_address_r;
  always@(posedge clk)
    dcpu_address_r<=dcpu_address;

  assign dcpu_readdata= (dcpu_address_r[31]) ? dbus_readdata : dcache_readdata;
  assign dcpu_readdata_line = dcache_readdata_line;

  //If this is an uncached access ([31]) wait on bus, otherwise wait on cache
  multicyclestall staller(dcpu_en,
      dcache_wait|(dcpu_address_r[31]&dbus_wait),
      clk,resetn,dcpu_wait);

  // CPU should already stall 1st cycle, so register everything
  reg dcpu_en_r;
  always@(posedge clk)
    dcpu_en_r<=dcpu_en;

  assign dcache_wait= (~dcpu_address_r[31] && 
                             dcache_state==CACHE_IDLE && dcpu_en_r) ||
                   (dcache_state==CACHE_TAGLOOKUP && dcache_miss) ||
                   (dcache_state==CACHE_ISSUE && dcache_miss) ||
                   (dcache_state==CACHE_STOREMISS && dcache_miss) ||
                   (dcache_state==CACHE_LOADMISS && dcache_miss);

  always@(posedge clk)
    if (!resetn)
      dcache_state<=CACHE_IDLE;
    else
      case(dcache_state)
        CACHE_IDLE:
          dcache_state<= (dcpu_en && ~dcpu_address[31]) ? CACHE_TAGLOOKUP : 
                                                          CACHE_IDLE;
        CACHE_TAGLOOKUP:
          dcache_state<= (dcache_hit || ~dcpu_en) ? CACHE_IDLE : CACHE_ISSUE;
        CACHE_ISSUE:
          dcache_state<= (dbus_wait) ? CACHE_ISSUE :
            (dbus_wren) ?  CACHE_STOREMISS : CACHE_LOADMISS;
        CACHE_STOREMISS:
          dcache_state<= (dcache_hit || (|dcache_timeout)==0) ?  CACHE_IDLE : 
                        CACHE_STOREMISS;
        CACHE_LOADMISS:
          dcache_state<= (dcache_hit || (|dcache_timeout)==0) ?  CACHE_IDLE : 
                        CACHE_LOADMISS;
        default:
          dcache_state<=CACHE_IDLE;
      endcase

      // Timeout if dbus_wait has been low for 4 cycles
  always@(posedge clk)
    if (dcache_state==CACHE_ISSUE)
      dcache_timeout<={4{1'b1}};
    else
      dcache_timeout<=(dbus_wait<<3) | (dcache_timeout>>1);

      // Go to mem if miss happened or if non-mem region addr>0x80000000
      // For memory requests (0-0x7fffffff) send single pulse 
      // For peripherals (0xffffffff-0x80000000) hold dbus_en high
  assign dbus_en= (dcpu_address[31] && dcpu_en) ||
                  //Don't issue yet - wait one more cycle for vpu to rewind
                  (dcache_state==CACHE_ISSUE);

  //assign dbus_address=dcpu_address;  //not dcpu_address_r since bus must react to 0x80000000
  assign dbus_address=dcpu_address;
  assign dbus_writedata=dcpu_writedata;
  assign dbus_byteen=dcpu_byteen;
  assign dbus_wren=dcpu_wren;
  assign dbus_prefetch=dcpu_prefetch;


endmodule