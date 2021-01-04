//************************************************************************
// Scalable Vector CPU Project -- Memory unit
// Filename: memIF.v
// Author: Jason Yu
//
// The memory unit contains logic for performing scalar/vector loads/stores
// - loadstoreController is the controller for memory operations
// - memreadIF is the datapath for loads
// - loadAddrGen is the controller for loads
// - memwriteIF is the datapath for stores
// - storeAddrGen is the controller for stores
//
//  Copyright (C) 2007 Jason Yu
//
//************************************************************************

// synthesis translate_off
`timescale 1ns / 100ps
// synthesis translate_on
`ifdef	MODELSIM
	`include "../hdl/define.v"
	`include "../hdl/config_def.v"
	`include "../hdl/isa_def.v"
`else
	`include "define.v"
	`include "config_def.v"
	`include "isa_def.v"
`endif


module memIF

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Customizable parameters
//---------------------------------------------------------------------------------------------------------------------------------------------------
#(	// Parameters
	// Vector processor primary parameters
	parameter	VPU_WIDTH = 1,
	parameter	NUMLANES = 1,
	parameter	MEM_WIDTH = 1,
	parameter	MINDATAWIDTH = 1,
	
	// Local parameter
	parameter	MEMWIDTH_BYTES = MEM_WIDTH/8,					// memory width in number of bytes
	parameter	MAX_NUMELEMS = NUMLANES * `ELEMPERLANE,			// number of total data elements
	parameter	MAX_NSUBWORDS = MEM_WIDTH/MINDATAWIDTH,			// maximum number of subwords in a memory group
	parameter	READMUX_NSEL = `log2(MEM_WIDTH/MINDATAWIDTH)	// number of select lines in read MUX
)
//---------------------------------------------------------------------------------------------------------------------------------------------------
// Ports
//---------------------------------------------------------------------------------------------------------------------------------------------------
(
	// **** Inputs ****
	input				clk,
	input				reset_n,
	input				d_waitrequest,					// need to wait for memory
	input				lsu_instr_decode_valid,			// an instruction that requires the LSU can be decoded now
	input	[31:0]		lsu_opCode,						// portion of the opcode to loadstore controller
	input	[NUMLANES*VPU_WIDTH-1:0]		laneDataIn,		// data from lanes
	input	[NUMLANES-1:0]	laneFlagQIn,				// flag from lanes
	// inputs from vector core
	input	[31:0]		id_vbase,						// base address of memory operation
	input	[31:0]		vctrl_q,							// vctrl_q control register
	input	[2:0]		id_memDataWidth,				// data width for memory access; id
	input	[31:0]		id_vl_special_instr,		// effective vector length -1; for vext.vv, it is Vindex-1+VL
	input	[31:0]		id_vl_minus_one,				// actual vector length -1; loadaddrgen needs actual vector length for vext.vv, not vl_eff
	input				loadDataQ_full,
	input				id_clear_lsu_data_flag,			// clear the load data waiting flag
	input				flagStoreQEmpty,				// whether the store flag data queue is empty
	// Inputs from scalar processor
	input	[31:0]		scalar_d_address,
	input	[3:0]		scalar_d_byteenable,			// scalar store byte enable
	input				scalar_d_read,
	input				scalar_d_write,
	input	[31:0]		scalar_d_writedata,
	input	[1:0]		scalar_data_size,				// size of data to be writte to data master: 00 = byte, 01 = halfword, 10 = word
	// Inputs from main memory
	input	[MEM_WIDTH-1:0]	d_readdata,					// main memory read data
	input	[VPU_WIDTH-1:0]	indexOffset_out,		// offset from lanes for indexed addressing

	input				mem_order_q_q,
	input				mem_order_q_empty,
	input				lsu_wraddr_q_rdreq,
	
	// **** Outputs ****
	output					d_read,						// read from memory
	output	[31:0]			d_address,
	output					d_write,
	output	[MEMWIDTH_BYTES-1:0]	d_byteenable,		// data mask byte enable, 16 bytes in 128b
	// Outputs to scalar processor
	output					scalar_d_waitrequest,		// pause scalar processor while waiting for data
	output	[31:0]			scalar_d_readdata,
	// Outputs to datapath
	output	[`log2(`VRF_NREG)-1:0]	lsu_vrf_wraddr,
	output						lsu_vflag_reg,			// copy flag register from opcode for writing loaded data back to VRF
	output					lsu_executing_vector_load,							// ready for the next load instruction
	output					lsu_busy,
	output	[NUMLANES-1:0]	data_lane_we,
	output	[NUMLANES*VPU_WIDTH-1:0]	memLoadDataToLanes,		// data to processors
	output	[2:0]			lsu_data_waiting_flag,		// Flag to indicate what type of data is waiting the load data transfer queue
	output	[`log2(NUMLANES)-1:0]	lsu_indexOffset_lanesel,	// select which index offset to use
	output					lsu_indexed_memaccess,
	// Memory write Outputs
	output	[MEM_WIDTH-1:0]	memWrdata,					// data to be written to memory
	output	[NUMLANES-1:0]	laneStoreQRdreq,		// read request to store queue in lanes
	output					flagStoreQRdreq,				// read request to read flag queue for flag store instruction
	
	output					mem_order_q_rdreq,
	output					load_done

);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local parameters
//---------------------------------------------------------------------------------------------------------------------------------------------------
localparam	NUM_MEMGROUP = (NUMLANES/(MEM_WIDTH/VPU_WIDTH));	// number of "MEM_WIDTH" memory groups to process; division result must be integer

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Local signals
//---------------------------------------------------------------------------------------------------------------------------------------------------
wire	[31:0]						mem_addr_0;				// write address
wire	[31:0]						mem_rdaddr_0;			// read address
wire 	[READMUX_NSEL*MAX_NSUBWORDS-1:0]	readXbarSel;	// select to memory crossbar
wire	[`max(`log2(NUM_MEMGROUP)-1,0):0]	laneWrselMUXSel;
wire	[MAX_NSUBWORDS-1:0]			delaySubword_en;		// enable the delay element for the halfword
wire	[MAX_NSUBWORDS-1:0]			delaySubwordSel;		// select data from the delay element for the halfword; 1=select delayed value
wire	[MAX_NSUBWORDS*`log2(MAX_NSUBWORDS)-1:0]	memWralignXbarSel;		// select for the delay element to memory crossbar
wire	[MEM_WIDTH-1:0]				alignedDataReg_en;		// write enable to register rotated data from rotation crossbar; needs to be bit-granularity to support flag store
wire	[MAX_NSUBWORDS-1:0]			alignedFlagRegQ;		// flag from lanes
wire	[MAX_NSUBWORDS-1:0]			alignedFlagReg_en;
wire	[MAX_NSUBWORDS-1:0]			alignedFlagReg_clr;		// mask out the aligned flag register due to element index larger than vector length
wire	[`log2(NUMLANES):0]			numActiveLanes_preg;
wire								lsu_unitstride;
wire	[2:0]						lsu_memDataWidth;
wire								lsu_scalar_load;						// indicate it is a scalar memory read
wire								lsu_signedOp;
wire								lsu_newInstr_load;						// start a vector memory read
wire								lsu_newInstr_store;						// start a vector memory write
wire	[31:0]						lsu_vl_special_instr;
wire	[`log2(MAX_NUMELEMS):0]		loadElemCount_preg;
wire	[31:0]						lsu_baseAddr;							// base address of memory operation
wire	[`MAXSTRIDE_NBIT-1:0]		lsu_stride;					// up to 32 element stride
wire	[MAX_NSUBWORDS-1:0]			vector_mem_dm;				// data mask
wire	[MEMWIDTH_BYTES-1:0]		vector_mem_dm_sig;			// data mask
wire								lsu_flagload;
wire								lsu_flagstore;
wire	[MEM_WIDTH-1:0]				storetoload_bypass;
wire	[MEM_WIDTH-1:0]				loaddata_in;
wire								lsu_vext_vv;
wire								lsu_vins_vv;
wire								vectorload_waitrequest;					// pause vector processor while waiting for data
wire								vectorstore_waitrequest;				// pause vector processor while waiting for memory
wire								store_done;								// asserted the last cycle of store
wire								executing_vector_store;
wire								executing_scalar_store;
wire								executing_scalar_load;
wire								vector_manip_loading;
wire								scalar_write_grant;
wire								lsu_storetoload_bypass_en;
wire								vmanip_storeload_sync;
wire	[31:0]						lsu_vl_minus_one;
wire	[31:0]						lsu_load_vl_eff_minus_one;
wire	[31:0]						lsu_store_vl_eff_minus_one;
wire	[`log2(NUMLANES)-1:0]		load_indexOffset_lanesel;	// select which index offset to use
wire	[`log2(NUMLANES)-1:0]		store_indexOffset_lanesel;	// select which index offset to use

reg		[MEMWIDTH_BYTES/4-1:0]		scalar_wordenable;			// word-granularity (32-bit) enable signal
wire	[`max(`log2(MEMWIDTH_BYTES/4)-1,0):0]	daddr_scalar_write_pos;		// 32-bit word position for the current scalar write

wire								load_done_delayn;
wire								load_done_posedge;
wire								store_done_delayn;
wire								store_done_posedge;

integer i;

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Signal assignments
//---------------------------------------------------------------------------------------------------------------------------------------------------
assign	d_address = ((executing_vector_store == 1'b1) || (executing_scalar_store == 1'b1)) ? mem_addr_0 : mem_rdaddr_0;
assign	lsu_indexOffset_lanesel = (executing_vector_store == 1'b1) ? store_indexOffset_lanesel : load_indexOffset_lanesel;

`ifdef	VECTOR_MANIP_INSTRUCTIONS
assign	loaddata_in = (lsu_storetoload_bypass_en) ? storetoload_bypass : d_readdata;
`else
assign	loaddata_in = d_readdata;
`endif

// generate a pulse on positive edge of load_done
assign	load_done_posedge = load_done & load_done_delayn;
assign	store_done_posedge = store_done & store_done_delayn;
ff	#(.WIDTH(1))	delayn_load_done_posedge (.clk(clk), .en(1'b1), .clrn(reset_n), .d(~load_done), .q(load_done_delayn));
ff	#(.WIDTH(1))	delayn_store_done_posedge (.clk(clk), .en(1'b1), .clrn(reset_n), .d(~store_done), .q(store_done_delayn));



//---------------------------------------------------------------------------------------------------------------------------------------------------
// Set effective VL for memory interface core:
// vins.vv: lsu_load_vl_eff_minus_one = VL - 1; lsu_store_vl_eff_minus_one = VL - 1
// vext.vv: lsu_load_vl_eff_minus_one = VL - 1; lsu_store_vl_eff_minus_one = VL + vindex
// flag store lsu_store_vl_eff_minus_one = (vl_minus_one / VPU_WIDTH) * VPU_WIDTH + VPU_WIDTH - 1
// all other instructions vl_eff_minus_one = VL - 1
//---------------------------------------------------------------------------------------------------------------------------------------------------
assign	lsu_load_vl_eff_minus_one = lsu_vl_minus_one;
assign	lsu_store_vl_eff_minus_one = ((lsu_vext_vv == 1'b1) || (lsu_flagstore == 1'b1)) ? lsu_vl_special_instr : lsu_vl_minus_one;


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Generate byte enable for scalar writes
//---------------------------------------------------------------------------------------------------------------------------------------------------
// decode address to 32-bit word enable (shift 2 to go from byte address to 32-bit word)
// assign lower log2(MEMWIDTH_BYTES/4) bits after shifting
assign	daddr_scalar_write_pos = scalar_d_address >> 2;

// all byteenable signals must be high during reads according to Avalon-MM Interface Specification
assign	d_byteenable = ((executing_vector_store == 1'b1) || (executing_scalar_store == 1'b1)) ?
						vector_mem_dm_sig : {MEMWIDTH_BYTES{1'b1}};

// Create byte enable signal
// if scalar store, use scalar byteenable, qualify with a 32-bit word-select signal (within the MEM_WIDTH memory block)
// if vector store, connect data mask signals to memory byte enable for different MINDATAWIDTH
// Each data mask signal is connected to MINDATAWIDTH/8 byte enable signals
genvar gi;
generate
	for (gi=0; gi<MEMWIDTH_BYTES; gi=gi+1)
	begin: mem_dm_i
		assign	vector_mem_dm_sig [gi] = (executing_scalar_store == 1'b1) ?
					(scalar_d_byteenable[gi % 4] & scalar_wordenable[gi/4]) : vector_mem_dm [ gi / (MINDATAWIDTH/8) ];
	end
endgenerate

// decode address to 32-bit word enable for scalar writes
always @(*) begin
	// default
	scalar_wordenable = {MEMWIDTH_BYTES/4{1'b0}};
	
	for (i=0; i<MEMWIDTH_BYTES/4; i=i+1) begin
		if (i == daddr_scalar_write_pos)
			scalar_wordenable[i] = 1'b1;
	end
end


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Load store controller
// - controls the load store unit
// - issues load/store instructions from vector and scalar cores
//---------------------------------------------------------------------------------------------------------------------------------------------------

// Load/store controller; controls all vector/scalar memory accesses
loadstoreController	loadstoreController_inst
(
	// **** Inputs ****
	.clk						(clk),
	.reset_n					(reset_n),
	.lsu_instr_decode_valid		(lsu_instr_decode_valid),
	.lsu_opCode					(lsu_opCode),				// portion of the opcode to loadstore controller
	.id_memDataWidth			(id_memDataWidth),			// data width for memory access
	.d_waitrequest				(d_waitrequest),			// need to wait for memory
	// Inputs from scalar core
	.scalar_d_read				(scalar_d_read),			// scalar data read request
	.scalar_d_write				(scalar_d_write),			// scalar data write request
	.scalar_data_size			(scalar_data_size),			// size of data to be writte to data master: 00 = byte, 01 = halfword, 10 = word
	// inputs from vector core
	.id_clear_lsu_data_flag		(id_clear_lsu_data_flag),			// clear the load data waiting flag
	.id_vl_special_instr		(id_vl_special_instr),
	.id_vl_minus_one			(id_vl_minus_one),
	.id_vbase					(id_vbase),
	.id_vstride					(vctrl_q),
	// Inputs from load and store units
	.vmanip_storeload_sync		(vmanip_storeload_sync),
	.load_done_posedge			(load_done_posedge),		// load instruction has finished; asserted the last cycle of the load
	.store_done_posedge			(store_done_posedge),		// store instruction done; asserted the last cycle of the store
	
	.mem_order_q_q				(mem_order_q_q),
	.mem_order_q_empty			(mem_order_q_empty),
	.lsu_wraddr_q_rdreq			(lsu_wraddr_q_rdreq),


	// **** Outputs ****
	// Control signals to the load store unit
	.lsu_unitstride				(lsu_unitstride),
	.lsu_memDataWidth			(lsu_memDataWidth),
	.lsu_indexed_memaccess		(lsu_indexed_memaccess),
	.lsu_signedOp				(lsu_signedOp),
	.lsu_vrf_wraddr				(lsu_vrf_wraddr),
	.lsu_vflag_reg				(lsu_vflag_reg),			// copy flag register from opcode for writing loaded data back to VRF
	.lsu_newInstr_load			(lsu_newInstr_load),		// start a load instruction
	.lsu_newInstr_store			(lsu_newInstr_store),		// start a store instruction
	.lsu_scalar_load			(lsu_scalar_load),			// indicate scalar access to load address generator
	.lsu_vl_special_instr		(lsu_vl_special_instr),
	.lsu_vl_minus_one			(lsu_vl_minus_one),
	.lsu_baseAddr				(lsu_baseAddr),
	.lsu_stride					(lsu_stride),
	.lsu_flagload				(lsu_flagload),
	.lsu_flagstore				(lsu_flagstore),
	.lsu_vext_vv				(lsu_vext_vv),
	.lsu_vins_vv				(lsu_vins_vv),
	
	.executing_vector_load		(lsu_executing_vector_load),
	.lsu_data_waiting_flag		(lsu_data_waiting_flag),				// Flag to indicate what type of data is waiting the load data transfer queue
	.lsu_busy					(lsu_busy),
	.lsu_storetoload_bypass_en	(lsu_storetoload_bypass_en),
	
	// state signals
	.scalar_write_grant			(scalar_write_grant),
	.executing_vector_store		(executing_vector_store),
	.executing_scalar_store		(executing_scalar_store),
	.executing_scalar_load		(executing_scalar_load),
	.vector_manip_loading		(vector_manip_loading),
	
	// signals to main memory
	.scalar_d_waitrequest		(scalar_d_waitrequest),		// pause scalar processor while waiting for data
	.vectorload_waitrequest		(vectorload_waitrequest),	// pause vector processor while waiting for data
	.vectorstore_waitrequest	(vectorstore_waitrequest),	// pause vector processor while waiting for memory
	
	.mem_order_q_rdreq			(mem_order_q_rdreq)
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Memory write to read bypass
// - bypasses data from the store datapath to load datapath
// - used for vector manipulation instructions
//---------------------------------------------------------------------------------------------------------------------------------------------------

`ifdef	VECTOR_MANIP_INSTRUCTIONS
// Register data
ff	#(.WIDTH(MEM_WIDTH))
storetoload_bypass_ff
(	.clk		(clk),
	.en			(lsu_storetoload_bypass_en),
	.clrn		(1'b1),
	.d			(memWrdata),
	.q			(storetoload_bypass)
);
`endif

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Memory read interface
// - aligns read data from memory so each lane gets the proper data
// - signed/unsigned extension
//---------------------------------------------------------------------------------------------------------------------------------------------------

// Memory read crossbar
memreadIF
#(	// Vector processor primary parameters
	.VPU_WIDTH				(VPU_WIDTH),
	.NUMLANES				(NUMLANES),
	.MEM_WIDTH				(MEM_WIDTH),
	.MINDATAWIDTH			(MINDATAWIDTH)
)
memreadIF_inst
(
	// Inputs
	.d_readdata				(loaddata_in),				// main memory read data
	.readXbarSel			(readXbarSel),			// select to memory crossbar
	.lsu_memDataWidth		(lsu_memDataWidth),
	.lsu_signedOp			(lsu_signedOp),
	.loadElemCount_preg		(loadElemCount_preg),
	.numActiveLanes_preg	(numActiveLanes_preg),
	.lsu_flagload			(lsu_flagload),
	.executing_scalar_load	(executing_scalar_load),
	// Outputs
	.scalar_d_readdata		(scalar_d_readdata),		// data to scalar core
	.memLoadDataToLanes		(memLoadDataToLanes)				// data to processors
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Load address generator
// - controls loading of data into the load data queue
//---------------------------------------------------------------------------------------------------------------------------------------------------

loadAddrGenerator
#(	// Vector processor primary parameters
	.VPU_WIDTH				(VPU_WIDTH),
	.NUMLANES				(NUMLANES),
	.MEM_WIDTH				(MEM_WIDTH),
	.MINDATAWIDTH			(MINDATAWIDTH),

	//synthesis translate_off
	.NELEMXFER_ROM_FILE 		("NElemXfer_rom.dat"),
	.NELEMREMAINDER_ROM_FILE	("NElemRemainder_rom.dat")
	//synthesis translate_on

	//synthesis read_comments_as_HDL on
	//.NELEMXFER_ROM_FILE 		("NElemXfer_rom.mif"),
	//.NELEMREMAINDER_ROM_FILE	("NElemRemainder_rom.mif")
	//synthesis read_comments_as_HDL off
)
loadAddrGen_inst	(
	// Inputs
	.clk						(clk),
	.reset_n					(reset_n),
	.lsu_vext_vv				(lsu_vext_vv),
	.lsu_vins_vv				(lsu_vins_vv),
	.lsu_flagload				(lsu_flagload),
	.baseAddr					(lsu_baseAddr),
	.lsu_executing_vector_load	(lsu_executing_vector_load),
	.lsu_vectorlength_minus_one		(lsu_load_vl_eff_minus_one),
	.s3_unitstride				(lsu_unitstride),
	.stride						(lsu_stride),
	.vindex						(vctrl_q),
	.s3_memDataWidth			(lsu_memDataWidth),
	.start_newLoad					(lsu_newInstr_load),
	.vectorload_waitrequest		(vectorload_waitrequest),
	.lsu_indexed_memaccess		(lsu_indexed_memaccess),
	.loadDataQ_full				(loadDataQ_full),
	.vector_manip_loading		(vector_manip_loading),
	// Scalar processor inputs
	.lsu_scalar_load			(lsu_scalar_load),				// indicate scalar access to load address generator
	.scalar_d_address			(scalar_d_address),
	.indexOffset_out			(indexOffset_out),	// offset from lanes for indexed addressing

	// Outputs
	.memAddr					(mem_rdaddr_0),
	.d_read						(d_read),					// read from data memory
	.readXbarSel				(readXbarSel),
	.data_lane_we				(data_lane_we),
	.loadElemCount_preg			(loadElemCount_preg),
	.numActiveLanes_preg		(numActiveLanes_preg),
	.lsu_indexOffset_lanesel	(load_indexOffset_lanesel),	// select which index offset to use
	.load_done					(load_done)
);


//---------------------------------------------------------------------------------------------------------------------------------------------------
// Memory write interface
// - contains MUX to select data from lanes
// - crossbar to rotate data to correct position for writing
// - selectable delay to align write data to correct address
//---------------------------------------------------------------------------------------------------------------------------------------------------

memwriteIF
#(	// Vector processor primary parameters
	.VPU_WIDTH				(VPU_WIDTH),
	.NUMLANES				(NUMLANES),
	.MEM_WIDTH				(MEM_WIDTH),
	.MINDATAWIDTH			(MINDATAWIDTH)
)
memwriteIF_inst
(
	// Inputs
	.clk					(clk),
	.executing_scalar_store	(executing_scalar_store),
	.d_waitrequest			(d_waitrequest),
	.lsu_flagstore			(lsu_flagstore),
	.lsu_vext_vv			(lsu_vext_vv),
	.lsu_vins_vv			(lsu_vins_vv),
	.lsu_memDataWidth		(lsu_memDataWidth),
	.scalar_d_writedata		(scalar_d_writedata),
	.laneDataIn				(laneDataIn),
	.laneWrselMUXSel		(laneWrselMUXSel),
	.laneFlagQIn			(laneFlagQIn),
	.alignedDataReg_en		(alignedDataReg_en),
	.alignedFlagReg_en		(alignedFlagReg_en),
	.alignedFlagReg_clr		(alignedFlagReg_clr),
	.delaySubword_en		(delaySubword_en),
	.delaySubwordSel		(delaySubwordSel),
	.memWralignXbarSel		(memWralignXbarSel),

	// Outputs
	.alignedFlagRegQ		(alignedFlagRegQ),	// flag from lanes
	.memWrdata				(memWrdata)
);

//---------------------------------------------------------------------------------------------------------------------------------------------------
// Store address generator
//---------------------------------------------------------------------------------------------------------------------------------------------------

storeAddrGen
#(	// Vector processor primary parameters
	.VPU_WIDTH				(VPU_WIDTH),
	.NUMLANES				(NUMLANES),
	.MEM_WIDTH				(MEM_WIDTH),
	.MINDATAWIDTH			(MINDATAWIDTH)
)
storeAddrGen_inst (
	// Inputs
	.clk					(clk),
	.reset_n				(reset_n),
	.scalar_write_grant		(scalar_write_grant),
	.scalar_d_address		(scalar_d_address),
	.baseAddr				(lsu_baseAddr),
	.stride					(lsu_stride),
	.vindex					(vctrl_q),
	.lsu_unitstride			(lsu_unitstride),
	.lsu_memDataWidth		(lsu_memDataWidth),
	.newInstr_store			(lsu_newInstr_store),
	.flagStoreQEmpty		(flagStoreQEmpty),
	.d_waitrequest			(vectorstore_waitrequest),
	.executing_vector_store	(executing_vector_store),
	.lsu_vectorlength_minus_one		(lsu_store_vl_eff_minus_one),
	.alignedFlagRegQ		(alignedFlagRegQ),
	.lsu_flagstore			(lsu_flagstore),
	.lsu_vext_vv			(lsu_vext_vv),
	.lsu_vins_vv			(lsu_vins_vv),
	.lsu_indexed_memaccess	(lsu_indexed_memaccess),
	
	.indexOffset_out		(indexOffset_out),
	
	// Outputs
	.laneWrselMUXSel		(laneWrselMUXSel),
	.alignedDataReg_en		(alignedDataReg_en),
	.alignedFlagReg_en		(alignedFlagReg_en),
	.alignedFlagReg_clr		(alignedFlagReg_clr),
	.delaySubword_en		(delaySubword_en),
	.delaySubwordSel		(delaySubwordSel),
	.memWralignXbarSel		(memWralignXbarSel),
	.laneStoreQRdreq		(laneStoreQRdreq),
	.flagStoreQRdreq		(flagStoreQRdreq),							// read request to read flag queue for flag store instruction
	.vmanip_storeload_sync	(vmanip_storeload_sync),					// sync with load controller for vext.vv
	.lsu_indexOffset_lanesel	(store_indexOffset_lanesel),
	
	// Outputs to memory
	.mem_addr_0				(mem_addr_0),
	.d_write				(d_write),
	.vector_mem_dm			(vector_mem_dm),				// data mask
	.store_done				(store_done)
);


endmodule
