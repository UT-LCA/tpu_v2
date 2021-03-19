 
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

module vmem_unit_32_4_2_2_1_32_128_7_128_7_5_10 (
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
input [ 32-1 : 0 ] cbase;
input [ 32-1 : 0 ] cstride;
input [ 32-1 : 0 ] cprefetch;

// Vector ports
input  [          4-1 : 0 ]  vmask;
input  [ 4*32-1 : 0 ]  vstrideoffset;
input  [ 4*32-1 : 0 ]  vindex;
input  [ 4*32-1 : 0 ]  vwritedata;
output [ 4*32-1 : 0 ]  voutput;
output [          4-1 : 0 ]  voutput_we;

input   [10-1:0]  in_dst;
input                     in_dst_we;
output   [10-1:0] out_dst;
output                    out_dst_we;
input                     in_vs_dst_we;
output                    out_vs_dst_we;

input  [      2-1 : 0 ]  sa;
input                               dir_left;

// Data memory interface
output dmem_en;
output dmem_we;
output  [ 31 : 0 ]                  dmem_address;
output  [ 128/8-1 : 0 ] dmem_byteen;
output  [ 128-1 : 0 ]   dmem_writedata;
input   [ 128-1 : 0 ]    dmem_readdata;
input                               dmem_cachematch;
input                               dmem_cachemiss;
output  [ 31 : 0 ]                  dmem_prefetch;
input                               dmem_wait;

reg     [ 31 : 0 ]                  dmem_address;
reg     [ 128/8-1 : 0 ] dmem_byteen;
reg     [ 128-1 : 0 ]   dmem_writedata;
reg     [ 31 : 0 ]                  dmem_prefetch;

wire  [1:0]  op_pattern;      // 0-Unit, 1-Strided, 2-Indexed
wire  [1:0]  op_size;         // 0-byte, 1-16bits, 2-32bits, 3-64bits
wire         op_signed;       // 0-unsigned, 1-signed
wire         op_we;         // 0-load, 1-store
wire         op_memop;         // 1-memory op, 0-vector shift op

wire  [ 2*32-1 : 0 ]  __vreaddata;
reg           [ 4*32-1 : 0 ]  address;
reg              [ 4*32-1:0]  vreaddata;
reg                    [ 4-1 : 0 ]  vreaddata_we;
reg   [ 2*(7-5)-1 : 0 ] crossbar_sel;
wire  [ 2*32-1 : 0 ] crossbar;
wire  [ 2*32-1 : 0 ]  _vwritedata;
wire   [ 2*4-1 : 0 ]  _vbyteen;

reg  [ 32-1 : 0 ] stride;
wire [ 32-1 : 0 ] stride_tmp;
reg  [ 32-1 : 0 ] prefetch;
wire [ 32-1 : 0 ] t_cprefetch;

wire           [         4-1 : 0 ]  vshifted_mask;
wire           [         4-1 : 0 ]  vshifted_masksave;
wire          [ 4*32-1 : 0 ]  vshifted_writedata;
wire          [ 4*32-1 : 0 ]  vshifted_address;


reg                           dmem_valid;
reg   [ 31 : 0 ]              dmem_readdata_address;
reg                           cachedata_stillvalid;

reg [2:0] munit_state;
reg [2:0] issue_state;

reg  [ 2-1 : 0 ] vpid;
reg  [ 4-1 : 0 ] done;
wire doneall;

wire  [         4-1 : 0 ]  _vreaddata_we;
reg   [ 2-1 : 0 ]  _parhits;
wire  [ 2-1 : 0 ]  parhits;
reg   [ 2-1 : 0 ]  parhits_done;
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

reg  [ 2-1 : 0 ] sa_count;
wire [ 2-1 : 0 ] next_sa_count;
wire                        next_sa_count_zero;
wire                        sa_zero;
reg  [ 4*32-1 : 0 ]  vshiftresult;

genvar i;
reg [31:0] j;
genvar k;
reg [31:0] l;
reg [31:0] m;
reg [31:0] n;
reg [31:0] p;

  assign {op_memop,op_pattern,op_size,op_signed,op_we}=op;

  assign stall= (munit_state!=MUNIT_IDLE && munit_state!=MUNIT_DONE);

/************************** Pipeline load stage *******************************/
  reg enable_s2;
  reg last_subvector_s2;
  reg [ 10-1 : 0 ] in_dst_s2;
  reg [      2-1 : 0 ] sa_s2;
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
      op_memop_s2<=0;
      op_pattern_s2<=0;
      op_size_s2<=0;
      op_signed_s2<=0;
      op_we_s2<=0;
    end
    else if (!stall)
    begin
      enable_s2<=enable;
      last_subvector_s2<=last_subvector;
      if (en[0])
        in_dst_s2<=in_dst;
      sa_s2<=sa;
      dir_left_s2<=dir_left;
      //{op_memop_s2,op_pattern_s2,op_size_s2,op_signed_s2,op_we_s2}<=op;
      op_memop_s2<=op[6];
      op_pattern_s2<=op[5:4];
      op_size_s2<=op[3:2];
      op_signed_s2<=op[1];
      op_we_s2<=op[0];
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

  wire doublewrite;
  // Detect double write one clock cycle early - also support 1 lane
  assign doublewrite=(~op_memop_s2) && (4>1) && ( (dir_left_s2) ? 
              (|vshifted_mask[(4)-2:0]) && //support L=1
                (vshifted_mask[4-1]&(|vshifted_masksave)) :
              (|vshifted_mask[4-1:1]) && //support L=1
                (vshifted_mask[0]&(|vshifted_masksave)));

  assign out_dst = {in_dst_s2[10-1:5],
                    (munit_state!=MUNIT_XTRAWRITE) ? in_dst_s2[5-1:0] :
                     (dir_left_s2) ? in_dst_s2[5-1:0]+1'b1 : 
                       in_dst_s2[5-1:0]-1'b1};

  //Truncate shifted result to VPW's size
  always@*
    for (p=0; p<4; p=p+1)
      vshiftresult[p*32 +: 32]=
              vshifted_address[p*32 +: 32];

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
    for (m=0; m<4; m=m+1)
      address[m*32 +: 32] = ((op_memop) ? cbase : 0) + 
            ( (op_pattern[1] || ~op_memop)  ? vindex[m*32 +: 32] : 
               (vstrideoffset[m*32 +: 32]<<op_size));

  wire [4-1:0] vwritedata_shifter_squash_NC;
  velmshifter_jump_32_4_2 vwritedatashifter(
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

  wire [4-1:0] vaddress_shifter_squash_NC;
  velmshifter_jump_32_4_2 vaddressshifter(
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

  velmshifter_jump_1_4_2 vmaskshifter(
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
  wire    [4-1 : 0]  vmasks_save_inpipe_NC;
  velmshifter_4_1  vmaskssave(
      .clk(clk),
      .resetn(resetn && ~shifter_load),
      .load(1'b0),
      .shift(shifter_shift),
      .dir_left(shifter_dirleft),
      .squash(0),
      .shiftin_left( vshifted_mask[0] ),
      .shiftin_right( vshifted_mask[4-1] ),
      .inpipe(vmasks_save_inpipe_NC),
      .outpipe(vshifted_masksave));

  //Stride is maximum of stride specified and cache line size 
  assign stride_tmp = (((op_pattern[0]) ? cstride : 1) << op_size);
  always@(posedge clk)
    if (!resetn || last_subvector_s2&~stall&~shifter_load&~quick_loaded )
      stride<=0;
    else if (shifter_load)
      stride<= (stride_tmp>128/8) ? 
                      stride_tmp : 128/8;

  wire [15:0] constantprefetch;
  assign constantprefetch=`VECTORPREFETCHES;
  assign t_cprefetch=cprefetch[15:0]<<(65535-`VECTORPREFETCHES);

  //***********************  Prefetch Logic **************************
  //
  //Send stride and vector length to bus.  To do constant prefetching set
  //stride to cache line size and send constant as length
  //******************************************************************
  wire [15:0] temp_prefetch;
  assign temp_prefetch = 16'd0+2**(7-3);
  always@(posedge clk)
    if (!resetn || last_subvector_s2&~stall&~shifter_load&~quick_loaded )
      prefetch<=0;
    else if (shifter_load)
      if (`VECTORPREFETCHES >= 65530)
        prefetch<= { stride_tmp[15:0], t_cprefetch[15:0] };
      else if (`VECTORPREFETCHES == 0)
        prefetch<= 0;
      else
        prefetch<= { temp_prefetch,constantprefetch };

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
      vpid<=vpid + ((shifter_jump) ? 2 : 1'b1);

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

  assign dmem_en = ((shifter_jump) ? (|vshifted_mask[2-1:0]) : 
                                      vshifted_mask[0]) && 
                    (munit_state==MUNIT_ISSUE || quick_loaded);
  assign dmem_we=op_we_s2;

  /*********************
  * We don't want cache miss signal to be an add/sub select signal or else it
  * will propagate through the adder.  We perform the addition/subtraction
  * in parallel with the miss calculation and use the miss as a mux select
  *********************/
  wire [ 32-1 : 0 ] dmem_address_next /* synthesis keep */;
  wire [ 32-1 : 0 ] dmem_address_prev /* synthesis keep */;
  assign dmem_address_next=dmem_address + stride;
  assign dmem_address_prev=dmem_address - stride;

  always@(posedge clk)
    if (!resetn)
      dmem_address<=0;
    else if (shifter_load)
      dmem_address<=address[32-1:0];
    else if (shifter_shift)

      dmem_address<= (!shifter_jump) ? 
              vshifted_address[
                2*32-1 : 32] :
              vshifted_address[
                 (2)*32+32-1 : (2)*32];
                   
    //Fetch next cache line if partial match on initial cache access
    //else if (parhits_some && ~cachedata_stillvalid)
    else if (addr_advance)
      dmem_address<=dmem_address_next;
    else if (addr_rewind)
      dmem_address<=dmem_address_prev;

  always@(posedge clk)
    if (!resetn)
      dmem_prefetch<=0;
    //else if (shifter_load)
    else
      dmem_prefetch<=prefetch[32-1:0];

/*************************** Mem Write LOGIC ********************************/

// Generate byte/halfword alignment circuitry for each word                  

       vstore_data_translator vstore_data_translator_0(
         //Pad vshifted_writedata with zeros incase signal is less than 32-bits
      .write_data({32'b0,vshifted_writedata[0*32+32-1 : 0*32]}),
      .d_address(vshifted_address[0*32+2-1 : 0*32]),
      .store_size(op_size_s2), 
      .d_byteena(_vbyteen[4*0+4-1 : 4*0]),  
      .d_writedataout(_vwritedata[0*32+32-1 : 0*32]));

        


       vstore_data_translator vstore_data_translator_1(
         //Pad vshifted_writedata with zeros incase signal is less than 32-bits
      .write_data({32'b0,vshifted_writedata[1*32+32-1 : 1*32]}),
      .d_address(vshifted_address[1*32+2-1 : 1*32]),
      .store_size(op_size_s2), 
      .d_byteena(_vbyteen[4*1+4-1 : 4*1]),  
      .d_writedataout(_vwritedata[1*32+32-1 : 1*32]));

        


  always@*
  begin
    dmem_writedata=0;
    dmem_byteen=0;
    for (l=0; l<2; l=l+1)
      if (dmem_address[31:7-3] == 
          vshifted_address[32*l+7-3 +: 
                            32-7+3])
      begin
        dmem_writedata=dmem_writedata| (_vwritedata[l*32 +: 32] << 
            {vshifted_address[32*l+2 +: 7-5], {5{1'b0}}});
        if (vshifted_mask[l] && (shifter_jump || (l==0)))
          dmem_byteen=dmem_byteen | (_vbyteen[4*l+:4]<<
            {vshifted_address[32*l+2 +: 7-5], {2{1'b0}}});
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
    for (j=0; j<2; j=j+1)
      //This signal tells us a cache request succeeded (for either load/store)
      //Each bit corresponds to a parallel lane
      _parhits[j]= vshifted_mask[j] &&
          ((dmem_valid&(dmem_cachematch&~op_we_s2 || op_we_s2&~dmem_wait)) || 
              (cachedata_stillvalid&~op_we_s2)) &&
            (vshifted_address[j*32+7-3 +: 
                             32-7+3] ==
               dmem_readdata_address[31:7-3]);

  //For operations that don't jump, just look at first bit of _parhits
  assign parhits=(shifter_jump) ? _parhits : {2{_parhits[0]}};

  // Detect all parallel lanes hitting
  assign parhits_all=&(parhits|~vshifted_mask[2-1:0]);
  // Detect cache misses
  assign parhits_none=~|(parhits);
  // Detect some misses - fetch next cache line
  assign parhits_some=~parhits_none && ~parhits_all;

  //If 4<=2 then we will never do a jump, so we make
  //compiler ignore this statement if that's the case
  always@(posedge clk)
    if (!resetn || shifter_load)
      parhits_done<= (shifter_jump_s1) ?  ~vmask : {2{~vmask[0]}};
    else if ( parhits_doneall )
    
      parhits_done<= (shifter_jump && (4>2)) ? 
                    ~vshifted_mask[ 
                        2*(2)-1 : 2] :
                    {2{(4>1) ? ~vshifted_mask[1] : 1'b0}};
                    
    else           
      parhits_done<= parhits_done|parhits;

  assign parhits_doneall=&(parhits|parhits_done);

  assign _vreaddata_we= ((shifter_jump) ? _parhits : _parhits[0]) 
                          << vpid[2-1:0];

  always@(posedge clk)
    if (!resetn || ~enable_s2)  // Force to zero if unit not used
      vreaddata_we<= 0 ;
    else           // write to regfile if a) is a load, b) we haven't already
      vreaddata_we<= {4{~op_we_s2}} & _vreaddata_we & ~done;

  //Select signal for crossbar either uses bits from corresponding parallel
  //lane address or all selects are set to the same if we're not jumping
  always@*
    for (n=0; n<2; n=n+1)
      crossbar_sel[(7-5)*n +: (7-5)]=
            (!shifter_jump) ? 
              {2{vshifted_address[2 +: 7-5]}} :
              vshifted_address[n*32+2 +: (7-5)];

  vmem_crossbar_128_7_2_32_5 vmem_crossbar(
      .clk(clk), .resetn(resetn),
      .sel(crossbar_sel),
      .in(dmem_readdata),
      .out(crossbar));

                // Generate byte/halfword alignment circuitry for each word                   

      vload_data_translator load_data_translator0(
      .d_readdatain(crossbar[32*(0+1)-1:32*0]),
      .d_address( (shifter_jump) ? vshifted_address[32*0+2-1 : 32*0] :
                                   vshifted_address[1:0]),
      .load_size(op_size_s2[1:0]),
      .load_sign_ext(op_signed_s2),
      .d_loadresult(__vreaddata[32*(0+1)-1:32*0])
      );

        


      vload_data_translator load_data_translator1(
      .d_readdatain(crossbar[32*(1+1)-1:32*1]),
      .d_address( (shifter_jump) ? vshifted_address[32*1+2-1 : 32*1] :
                                   vshifted_address[1:0]),
      .load_size(op_size_s2[1:0]),
      .load_sign_ext(op_signed_s2),
      .d_loadresult(__vreaddata[32*(1+1)-1:32*1])
      );

        


always@(posedge clk)
    //zero if unit not used
    if (!resetn || ~enable_s2 || (~stall&~enable) || op_we_s2)
      vreaddata<= 0 ;
    else                // Don't write to regfile unless this is a load!
      vreaddata<= {4/2{__vreaddata}};

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

  pipereg_1  dstwepipe (
    .d( in_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .squashn(1'b1),
    .en( en[0] && ~stall ),
    .q(out_dst_we));

  pipereg_1 vsdstwepipe (
    .d( in_vs_dst_we ),  
    .clk(clk),
    .resetn(resetn),
    .squashn(1'b1),
    .en( en[0] && ~stall ),
    .q(out_vs_dst_we));

endmodule

