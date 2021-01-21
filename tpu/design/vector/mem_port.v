/****
 * State machine is as follows:
 ****/

module mem_port (
    clk,
    resetn,

    bus_address,
    bus_writedata,
    bus_byteen,
    bus_numblocks,
    bus_prefetch,   //{stride[15:0],length[15:0]}
    bus_en,
    bus_wren,
    bus_wait_r,  // goes high 1 cycle AFTER bus_en
    bus_idle,

    clk_mem,
    resetn_mem,
    mem_address,
    mem_writedata,
    mem_byteen,
    mem_numblocks,
    mem_wren,
    mem_en,
    mem_ack,  // acknowledges receiving wait request
    mem_wait

    );

parameter CPU_DATAWIDTH=32;
parameter CPU_ADDRWIDTH=32;

input clk;
input resetn;

input   [CPU_ADDRWIDTH-1:0]  bus_address;
input   [CPU_DATAWIDTH-1:0]  bus_writedata;
input   [3:0]   bus_byteen;
input   [15:0]  bus_numblocks;
input   [31:0]  bus_prefetch;
input           bus_en;
input           bus_wren;
output          bus_wait_r;
output          bus_idle;

input clk_mem;
input resetn_mem;
output reg [31:0]                mem_address;
output reg [CPU_DATAWIDTH-1:0]   mem_writedata;
output reg [3:0]                 mem_byteen;
output reg [15:0]                mem_numblocks;
output reg                       mem_wren;
output reg                       mem_en;
output reg                       mem_ack;
input                        mem_wait;

reg bus_wait_r;
reg mem_wait_r;

reg [31:0]                   t_mem_address;
reg [CPU_DATAWIDTH-1:0]      t_mem_writedata;
reg [3:0]                    t_mem_byteen;
reg [15:0]                   t_mem_numblocks;
reg                          t_mem_wren;
reg                          t_mem_en;
reg                          t_mem_ack;

reg [15:0]                   t_mem_prefetch_length;
reg [15:0]                   t_mem_prefetch_stride;

parameter [2:0] BUS_IDLE=3'b0, BUS_START=3'b01, BUS_BUSY=3'b010, 
  BUS_DONE=3'b011, BUS_PREFETCH=3'b100;
reg [2:0] bus_state;

  // register bus signals to ensure they arrive together (since mem_ is async)
  always @(negedge resetn or posedge clk)
  begin
    if (!resetn)
    begin
      t_mem_address<=0;
      t_mem_writedata<=0;
      t_mem_byteen<=0;
      t_mem_numblocks<=0;
      t_mem_prefetch_stride<=0;
      t_mem_prefetch_length<=0;
      t_mem_wren<=0;
      t_mem_en<=0;
      t_mem_ack<=0;
      bus_wait_r <= 0;
      bus_state<=BUS_IDLE;
    end
    else
    begin
      case (bus_state)
        BUS_IDLE:
          if (bus_en)
          begin
            t_mem_address<=bus_address[31:0]; //[31:0]
            t_mem_writedata<=bus_writedata;
            t_mem_byteen<=bus_byteen;
            t_mem_numblocks<=bus_numblocks;
            t_mem_prefetch_stride<=bus_prefetch[31:16];
            t_mem_prefetch_length<=bus_prefetch[15:0];
            t_mem_wren<=bus_wren;
            t_mem_en<=bus_en;
            t_mem_ack<=1'b0;
            bus_state<=BUS_START;
          end
        BUS_START:   // Must wait for wait to go high (else we'll fly past BUSY)
          begin
            bus_state <= (mem_wait_r==0) ? BUS_START : BUS_BUSY;
            t_mem_ack<=mem_wait_r;
          end
        BUS_BUSY:
          begin
            bus_state <= (mem_wait_r) ? BUS_BUSY : BUS_DONE;
            t_mem_en<=mem_wait_r;
            t_mem_ack<=mem_wait_r;
          end
        BUS_DONE:  
          bus_state<= (t_mem_prefetch_length==0) ? BUS_IDLE : BUS_PREFETCH;
        BUS_PREFETCH:
          begin
            t_mem_address<=t_mem_address+t_mem_prefetch_stride;
            t_mem_en<=1'b1;
            t_mem_prefetch_length<=t_mem_prefetch_length-1;
            t_mem_ack<=1'b0;
            bus_state<=BUS_START;
          end
        default:
          bus_state<=BUS_IDLE;
      endcase
      bus_wait_r <= ((bus_state==BUS_IDLE)&&bus_en) ||
                    (bus_state==BUS_START) || (bus_state==BUS_BUSY);
    end             
  end

  assign bus_idle=(bus_state==BUS_IDLE);

  always @(posedge clk)
    mem_wait_r<=mem_wait;

  always @(posedge clk_mem)
  begin

    if (t_mem_ack && (~mem_wait))
      mem_en<=1'b0;
    else
      mem_en<=t_mem_en;

    mem_address<=t_mem_address;
    mem_writedata<=t_mem_writedata;
    mem_byteen<=t_mem_byteen;
    mem_numblocks<=t_mem_numblocks;
    //mem_prefetch_stride<=t_mem_prefetch_stride;
    //mem_prefetch_length<=t_mem_prefetch_length;
    mem_wren<=t_mem_wren;
    mem_ack<=t_mem_ack;
  end

endmodule

