/*****
 * Protocol:
 *   0. Activate trace by reading addr 
 *   1. Read address continuously, if it's 0 skip, otherwise read data
 */

module trace (
    clk,
    resetn,

    activated,

    reg_file_we,
    reg_file_addr,
    reg_file_data,

    pipestalled,    // tells us if trace pipe stage is stalled
    stallpipe,  // trace issues a pipestalled to the pipe

    addr,
    addr_ready,
    addr_want,

    data,
    data_ready,
    data_want
    );

input  clk;
input  resetn;

input  activated;

input reg_file_we;
input [4:0]  reg_file_addr;
input [31:0] reg_file_data;

input pipestalled;
output stallpipe;

output [4:0] addr;
output addr_ready;
input  addr_want;

output [31:0] data;
output data_ready;
input  data_want;

  reg [1:0] addr_state;
  reg [1:0] data_state;
  reg [2:0] wait_state;
  parameter [1:0] IDLE=2'b01, TRANSFER=2'b10;
  parameter [2:0] S_IDLE=3'b000, 
                  S_FULL1=3'b001, 
                  S_FULL2=3'b010, 
                  S_WAIT=3'b011, 
                  S_FINISH=3'b100;
  //reg       activated;

  reg data_ready;
  reg addr_ready;

  reg stallpipe;
  wire [4:0] addr_t;
  wire [4:0] fifoaddr;
  wire [31:0] fifodata;
  wire empty,full;

  //Change to always capture - So I can see writebacks in signal tap
  //register r1(reg_file_addr,clk,resetn,reg_file_we&activated&(wait_state==S_IDLE),addr_t);
  register r1(fifoaddr,clk,resetn,(wait_state==S_FULL2),addr_t);
    defparam r1.WIDTH=5;

  //register r2(reg_file_data,clk,resetn,reg_file_we&activated&(wait_state==S_IDLE),data);
  register r2(fifodata,clk,resetn,(wait_state==S_FULL2),data);
    defparam r2.WIDTH=32;

	scfifo	scfifo_component (
				.rdreq ((wait_state==S_FULL1)&~empty),
				.clock (clk),
				.wrreq (reg_file_we & (|reg_file_addr) & activated & ~pipestalled & (wait_state==S_IDLE)),
				.data ({reg_file_addr,reg_file_data}),
				.empty (empty),
				.q ({fifoaddr,fifodata}),
				.sclr (!resetn),
				.almost_full (full),
				.full ()
				// synopsys translate_off
				,
				.aclr (),
				.almost_empty (),
				.usedw ()
				// synopsys translate_on
				);
	defparam
		scfifo_component.add_ram_output_register = "OFF",
		scfifo_component.intended_device_family = "Stratix",
		scfifo_component.lpm_numwords = 256,
		scfifo_component.lpm_showahead = "OFF",
		scfifo_component.lpm_type = "scfifo",
		scfifo_component.lpm_width = 37,
		scfifo_component.lpm_widthu = 8,
		scfifo_component.overflow_checking = "ON",
		scfifo_component.underflow_checking = "ON",
		scfifo_component.almost_full_value = 250,
		scfifo_component.use_eab = "ON";

  assign addr=(wait_state==S_WAIT) ? addr_t : 5'b0;

  // FSM for waiting
  always@(posedge clk or negedge resetn)
  if (resetn==0)
  begin
    stallpipe<=0;
    wait_state<=S_IDLE;
  end
  else
    case(wait_state)
      S_IDLE:   // 1. Wait for write to non-zero address
      begin
        stallpipe<=full & activated  & ~pipestalled;
        //stallpipe<=(reg_file_we & (|reg_file_addr) & activated  & ~pipestalled);
        wait_state<= (full & activated & ~pipestalled) ? S_FULL1 :S_IDLE;
      end
      S_FULL1:   // 2. Get data out of FIFO
      begin
        stallpipe<=1'b1;
        wait_state<= (!empty) ? S_FULL2 : S_IDLE;
      end
      S_FULL2:   // 2. Latch into registers
      begin
        stallpipe<=1'b1;
        //wait_state<= (!empty) ? S_WAIT : S_IDLE;
        wait_state<= S_WAIT;
      end
      S_WAIT:   // 2. Wait for PC to notice it needs to be read
      begin
        stallpipe<=1'b1;
        wait_state<= (data_state!=TRANSFER) ? S_WAIT : S_FINISH;
      end
      S_FINISH: // 3. Wait for data read to finish
      begin
        stallpipe<=1'b1;
        wait_state<= (data_state==TRANSFER) ? S_FINISH : S_FULL1;
      end
      default:
      begin
        stallpipe<=1'b0;
        wait_state<=S_IDLE;
      end
    endcase

    /* This would miss the first couple instruction
  // Activation detection
  always@(posedge clk or negedge resetn)
    if (resetn==0)
      activated<=1'b0;
    else if (addr_state==TRANSFER)
      activated<=1'b1;
      */

  // FSM for address handshake
  always@(posedge clk or negedge resetn)
  if (resetn==0)
  begin
    addr_ready=1'b0;
    addr_state=IDLE;
  end
  else
    case(addr_state)
      IDLE:
      begin
        addr_ready=1'b1;
        addr_state= (addr_want) ? TRANSFER : IDLE;
      end
      TRANSFER:
      begin
        addr_ready=1'b0;
        addr_state= (addr_want) ? TRANSFER : IDLE;
      end
      default:
      begin
        addr_ready=1'b0;
        addr_state=IDLE;
      end
    endcase

  // FSM for data handshake
  always@(posedge clk or negedge resetn)
  if (resetn==0)
  begin
    data_ready=1'b0;
    data_state=IDLE;
  end
  else
    case(data_state)
      IDLE:
      begin
        data_ready=1'b1;
        data_state= (data_want) ? TRANSFER : IDLE;
      end
      TRANSFER:
      begin
        data_ready=1'b0;
        data_state= (data_want) ? TRANSFER : IDLE;
      end
      default:
      begin
        data_ready=1'b0;
        data_state=IDLE;
      end
      endcase



endmodule
