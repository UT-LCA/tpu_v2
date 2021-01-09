/****************
* This fifo assumes it receives WBDATAWIDTH data to write once every 
* WBDATAWIDTH/MEMDATAWIDTH cycles. 
*****************/

module mem_wbbuffer(
  clk,
  resetn,

  wbaddr,
  wbdata,
  wbwe,
  wbfull,

  mem_address,
  mem_writedata,
  mem_wren,
  mem_ack    //Synchronous acknowledge - pops data out of FIFO
);

parameter ADDRWIDTH=32;
parameter LOG2WBDATAWIDTH=7;
parameter LOG2MEMDATAWIDTH=7;
parameter LOG2FIFODEPTH=7;  //in MEMDATAWIDTH size words

parameter WBDATAWIDTH=2**LOG2WBDATAWIDTH;
parameter MEMDATAWIDTH=2**LOG2MEMDATAWIDTH;
parameter FIFODEPTH=2**LOG2FIFODEPTH;

input  clk;
input  resetn;

input  [ADDRWIDTH-1:0] wbaddr;
input  [WBDATAWIDTH-1:0] wbdata;
input  wbwe;
output wbfull;

output [ADDRWIDTH-1:0] mem_address;
output [MEMDATAWIDTH-1:0] mem_writedata;
output mem_wren;
input  mem_ack;

reg  [ADDRWIDTH-1:0] wbaddr_r;
reg  [WBDATAWIDTH-1:0] wbdata_r;
reg  [WBDATAWIDTH/MEMDATAWIDTH-1:0] wbload;
reg  [WBDATAWIDTH/MEMDATAWIDTH-1:0] wbaddrpop_count;
wire wbaddrpop;

wire empty;

  always @(posedge clk)
    if (!resetn)
      wbload<=0;
    else if (wbload==0 || wbload[0])  //Allow wbwe only once per cache line
      wbload<=(wbwe<<(WBDATAWIDTH/MEMDATAWIDTH-1)) | (wbload>>1);
    else 
      wbload<=wbload>>1;

  // Wb Address
  always @(posedge clk)
    if (!resetn)
      wbaddr_r<=0;
    else if (wbwe && ((wbload>>1)==0))
      wbaddr_r<=wbaddr;

  // Wb Data - Chop into MEM-sized chunks 
  integer j;
  always @(posedge clk)
    if (!resetn)
      wbdata_r<=0;
    else if (wbwe && ((wbload>>1)==0))
      wbdata_r<=wbdata;
    else if (wbload!=0)
    begin
      for (j=1; j<WBDATAWIDTH/MEMDATAWIDTH; j=j+1)
        wbdata_r[MEMDATAWIDTH*(j-1) +: MEMDATAWIDTH]<=
      wbdata_r[MEMDATAWIDTH*j +: MEMDATAWIDTH];
    end

  // Wb Address - we have one address per WBDATAWIDTH/MEMDATAWIDTH data
  always @(posedge clk)
    if (!resetn)
      wbaddrpop_count<=0;
    //Allow only one pop per WBDATAWIDTH/MEMDATAWIDTH cycles
    else if (wbaddrpop_count==0 || wbaddrpop_count[0])
      wbaddrpop_count<=(mem_ack<<(WBDATAWIDTH/MEMDATAWIDTH-1)) | 
                                    (wbaddrpop_count>>1);
    else if (mem_ack)
      wbaddrpop_count<=(wbaddrpop_count>>1);
    assign wbaddrpop=mem_ack && 
            (WBDATAWIDTH/MEMDATAWIDTH==1 || wbaddrpop_count==2);

    //DEBUG signals
wire [LOG2FIFODEPTH-LOG2WBDATAWIDTH+LOG2MEMDATAWIDTH-1:0] used_addr;
wire [LOG2FIFODEPTH-1:0] used_data;

	scfifo	wbaddrfifo (
				.clock (clk),
				.wrreq (wbload[WBDATAWIDTH/MEMDATAWIDTH-1]),
				.data (wbaddr_r),
				.rdreq (wbaddrpop&~empty),
				.q (mem_address),
				.sclr (!resetn), 
        .usedw (used_addr)
				// synopsys translate_off
				, 
				.almost_full (),
				.empty (),
        .full () , .aclr (), .almost_empty ()
				// synopsys translate_on
				);
	defparam
		wbaddrfifo.add_ram_output_register = "OFF",
		wbaddrfifo.intended_device_family = "Stratix",
		wbaddrfifo.lpm_numwords = FIFODEPTH/(WBDATAWIDTH/MEMDATAWIDTH),
		wbaddrfifo.lpm_showahead = "ON",
		wbaddrfifo.lpm_type = "scfifo",
		wbaddrfifo.lpm_width = 32,
		wbaddrfifo.lpm_widthu = LOG2FIFODEPTH-LOG2WBDATAWIDTH+LOG2MEMDATAWIDTH,
		wbaddrfifo.overflow_checking = "ON",
		wbaddrfifo.underflow_checking = "ON",
		wbaddrfifo.almost_full_value = 0,
		wbaddrfifo.use_eab = "ON";
  
	scfifo	wbdatafifo (
				.clock (clk),
				.wrreq ((wbload!=0)),
				.data (wbdata_r),
				.almost_full (wbfull),
				.rdreq (mem_ack&~empty),
				.empty (empty),
				.q (mem_writedata),
				.sclr (!resetn), 
        .usedw (used_data)
				// synopsys translate_off
				, 
        .full () , .aclr (), .almost_empty ()
				// synopsys translate_on
				);
	defparam
		wbdatafifo.add_ram_output_register = "OFF",
		wbdatafifo.intended_device_family = "Stratix",
		wbdatafifo.lpm_numwords = FIFODEPTH,
		wbdatafifo.lpm_showahead = "ON",
		wbdatafifo.lpm_type = "scfifo",
		wbdatafifo.lpm_width = MEMDATAWIDTH,
		wbdatafifo.lpm_widthu = LOG2FIFODEPTH,
		wbdatafifo.overflow_checking = "ON",
		wbdatafifo.underflow_checking = "ON",
		wbdatafifo.almost_full_value = FIFODEPTH-2*WBDATAWIDTH/MEMDATAWIDTH,
		wbdatafifo.use_eab = "ON";

  assign mem_wren=!empty;
  
endmodule
