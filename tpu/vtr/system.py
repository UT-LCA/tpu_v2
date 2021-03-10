from isa import isa
import parser

class system():
    def __init__(self, fp):
        self.fp = fp

    def make_str(self):
        string = '''\
`include "bpred_1bittable.v"
`include "cop0.v"
`include "cop2.v"
`include "lo_reg.v"
`include "hi_reg.v"
`include "data_mem_bus_int.v"
`include "divider.v"
`include "mul_shift_stall.v"
`include "logic_unit.v"
`include "addersub_slt.v"
`include "merge26lo.v"
`include "branchresolve.v"
`include "pcadder.v"
`include "signext16.v"
`include "reg_file_pipe.v"
`include "ifetch_pipe_bpred_bus_int.v"
`include "components.v"


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
`include "isa.v"
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
	// 	// Initialize control opcodes to zero
	// ctrl_mux2to1_pipereg6_d_sel = 0;
	// ctrl_mux3to1_zeroer4_d_sel = 0;
	// ctrl_mux2to1_pipereg_d_sel = 0;
	// ctrl_zeroer4_en = 0;
	// ctrl_zeroer0_en = 0;
	// ctrl_zeroer_en = 0;
	
	case (ifetch_opcode)
		OP_ADDI:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_ADDIU:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_ANDI:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_BEQ:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
            ctrl_mux3to1_zeroer4_d_sel = 0;
            ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_BGTZ:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
            ctrl_mux3to1_zeroer4_d_sel = 0;
            ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_BLEZ:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
            ctrl_mux3to1_zeroer4_d_sel = 0;
            ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_BNE:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 1;
            ctrl_mux3to1_zeroer4_d_sel = 0;
            ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_COP0:
		case (ifetch_rs)
			COP0_MFC0:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 2;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
                ctrl_zeroer_en = 0;
			end
			COP0_MTC0:
            begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
                ctrl_zeroer_en = 0;
            end
		endcase
		OP_COP2:
		case (ifetch_func)
			COP2_FUNC_CFC2:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 2;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
                ctrl_zeroer_en = 0;
			end
			COP2_FUNC_CTC2:
            begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
                ctrl_zeroer_en = 0;
            end
			COP2_FUNC_MTC2:
            begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
                ctrl_zeroer_en = 0;
            end
		endcase
		OP_J: 
        begin
			ctrl_mux2to1_pipereg6_d_sel = 0;
            ctrl_mux3to1_zeroer4_d_sel = 0;
            ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
            ctrl_zeroer0_en = 0;
            ctrl_zeroer_en = 0;
        end
		OP_JAL:
		begin
			ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 0;
            ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
            ctrl_zeroer_en = 0;
		end
		OP_LB:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_LBU:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_LH:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_LHU:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_LUI:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
            ctrl_zeroer_en = 0;
		end
		OP_LW:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_ORI:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_REGIMM:
		case (ifetch_rt[0])
			FUNC_BGEZ:
			begin
				ctrl_mux2to1_pipereg6_d_sel = 1;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
                ctrl_zeroer0_en = 0;
				ctrl_zeroer_en = 1;
			end
			FUNC_BLTZ:
			begin
				ctrl_mux2to1_pipereg6_d_sel = 1;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
                ctrl_zeroer0_en = 0;
				ctrl_zeroer_en = 1;
			end
		endcase
		OP_SB:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
            ctrl_mux3to1_zeroer4_d_sel = 0;
			ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SH:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
            ctrl_mux3to1_zeroer4_d_sel = 0;
			ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_SLTI:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_SLTIU:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 0;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
			ctrl_zeroer_en = 1;
		end
		OP_SPECIAL:
		case (ifetch_func)
			FUNC_ADD:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_ADDU:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_AND:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_DIV:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_DIVU:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_JALR:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
				ctrl_zeroer_en = 1;
			end
			FUNC_JR:
            begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
                ctrl_zeroer0_en = 0;
				ctrl_zeroer_en = 1;
            end
			FUNC_MFHI:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
                ctrl_zeroer_en = 0;
			end
			FUNC_MFLO:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
                ctrl_zeroer0_en = 0;
                ctrl_zeroer_en = 0;
			end
			FUNC_MULT:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_MULTU:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
                ctrl_mux3to1_zeroer4_d_sel = 0;
                ctrl_mux2to1_pipereg_d_sel = 0;
                ctrl_zeroer4_en = 0;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_NOR:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_OR:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SLL:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
                ctrl_zeroer_en = 0;
			end
			FUNC_SLLV:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SLT:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SLTU:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SRA:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
                ctrl_zeroer_en = 0;
			end
			FUNC_SRAV:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SRL:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
                ctrl_zeroer_en = 0;
			end
			FUNC_SRLV:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SUB:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_SUBU:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
			FUNC_XOR:
			begin
                ctrl_mux2to1_pipereg6_d_sel = 0;
				ctrl_mux3to1_zeroer4_d_sel = 1;
                ctrl_mux2to1_pipereg_d_sel = 0;
				ctrl_zeroer4_en = 1;
				ctrl_zeroer0_en = 1;
				ctrl_zeroer_en = 1;
			end
		endcase
		OP_SW:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
            ctrl_mux3to1_zeroer4_d_sel = 0;
			ctrl_mux2to1_pipereg_d_sel = 0;
            ctrl_zeroer4_en = 0;
			ctrl_zeroer0_en = 1;
			ctrl_zeroer_en = 1;
		end
		OP_XORI:
		begin
            ctrl_mux2to1_pipereg6_d_sel = 0;
			ctrl_mux3to1_zeroer4_d_sel = 2;
			ctrl_mux2to1_pipereg_d_sel = 1;
			ctrl_zeroer4_en = 1;
            ctrl_zeroer0_en = 0;
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
	// ctrl_mux6to1_pipereg7_d_sel = 0;
	// ctrl_mux9to1_nop13_d_sel = 0;
	// ctrl_mux3to1_nop9_d_sel = 0;
	// ctrl_mux2to1_lo_reg_d_sel = 0;
	// ctrl_mux2to1_hi_reg_d_sel = 0;
	// ctrl_mux2to1_mul_opA_sel = 0;
	// ctrl_mux3to1_mul_sa_sel = 0;
	// ctrl_mux2to1_addersub_opA_sel = 0;
	// ctrl_ifetch_pcwrop = 0;
	// ctrl_addersub_op = 0;
	// ctrl_data_mem_op = 0;
	// ctrl_div_sign = 0;
	// ctrl_mul_op = 0;
	// ctrl_logic_unit_op = 0;
	
	case (pipereg14_q)
		OP_ADDI:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 6;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_ADDIU:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 6;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 1;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_ANDI:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 4;
			ctrl_mux3to1_nop9_d_sel = 2;
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
		end
		OP_BEQ:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 5;
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
		end
		OP_BGTZ:
		begin
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
		end
		OP_BLEZ:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 3;
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
		end
		OP_BNE:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 4;
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
		end
		OP_COP0:
		case (pipereg17_q)
			COP0_MFC0:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 7;
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
			end
		endcase
		OP_COP2:
		case (pipereg15_q)
			COP2_FUNC_CFC2:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 8;
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
			end
		endcase
		OP_J:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 0;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 1;
			ctrl_addersub_op = 0;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_JAL:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 6;
			ctrl_mux3to1_nop9_d_sel = 0;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 1;
			ctrl_ifetch_pcwrop = 1;
			ctrl_addersub_op = 1;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_LB:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 7;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_LBU:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 3;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_LH:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 5;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_LHU:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 1;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_LUI:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 3;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 1;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 0;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_LW:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 2;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_ORI:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 4;
			ctrl_mux3to1_nop9_d_sel = 2;
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
			ctrl_logic_unit_op = 1;
		end
		OP_REGIMM:
		case (pipereg16_q[0])
			FUNC_BGEZ:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 1;
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
			end
			FUNC_BLTZ:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 2;
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
			end
		endcase
		OP_SB:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 11;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_SH:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 9;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_SLTI:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 5;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 6;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_SLTIU:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 5;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 4;
			ctrl_data_mem_op = 0;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_SPECIAL:
		case (pipereg15_q)
			FUNC_ADD:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 3;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_ADDU:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 1;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_AND:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
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
			end
			FUNC_DIV:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 1;
				ctrl_mux2to1_hi_reg_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 1;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_DIVU:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 1;
				ctrl_mux2to1_hi_reg_d_sel = 1;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_JALR:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 1;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 1;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MFHI:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 1;
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
			end
			FUNC_MFLO:
			begin
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
			end
			FUNC_MULT:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 1;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 6;
				ctrl_logic_unit_op = 0;
			end
			FUNC_MULTU:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 0;
				ctrl_mux3to1_nop9_d_sel = 0;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 1;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 4;
				ctrl_logic_unit_op = 0;
			end
			FUNC_NOR:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
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
				ctrl_logic_unit_op = 3;
			end
			FUNC_OR:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
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
				ctrl_logic_unit_op = 1;
			end
			FUNC_SLL:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
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
			end
			FUNC_SLLV:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 2;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SLT:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 5;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 6;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SLTU:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 5;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 4;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SRA:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 3;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SRAV:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 2;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 3;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SRL:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 1;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SRLV:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 3;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 2;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 0;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 1;
				ctrl_logic_unit_op = 0;
			end
			FUNC_SUB:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
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
			end
			FUNC_SUBU:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 6;
				ctrl_mux3to1_nop9_d_sel = 1;
				ctrl_mux2to1_lo_reg_d_sel = 0;
				ctrl_mux2to1_hi_reg_d_sel = 0;
				ctrl_mux2to1_mul_opA_sel = 0;
				ctrl_mux3to1_mul_sa_sel = 0;
				ctrl_mux2to1_addersub_opA_sel = 0;
				ctrl_ifetch_pcwrop = 0;
				ctrl_addersub_op = 2;
				ctrl_data_mem_op = 0;
				ctrl_div_sign = 0;
				ctrl_mul_op = 0;
				ctrl_logic_unit_op = 0;
			end
			FUNC_XOR:
			begin
				ctrl_mux6to1_pipereg7_d_sel = 0;
				ctrl_mux9to1_nop13_d_sel = 4;
				ctrl_mux3to1_nop9_d_sel = 1;
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
				ctrl_logic_unit_op = 2;
			end
		endcase
		OP_SW:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 0;
			ctrl_mux3to1_nop9_d_sel = 2;
			ctrl_mux2to1_lo_reg_d_sel = 0;
			ctrl_mux2to1_hi_reg_d_sel = 0;
			ctrl_mux2to1_mul_opA_sel = 0;
			ctrl_mux3to1_mul_sa_sel = 0;
			ctrl_mux2to1_addersub_opA_sel = 0;
			ctrl_ifetch_pcwrop = 0;
			ctrl_addersub_op = 3;
			ctrl_data_mem_op = 8;
			ctrl_div_sign = 0;
			ctrl_mul_op = 0;
			ctrl_logic_unit_op = 0;
		end
		OP_XORI:
		begin
			ctrl_mux6to1_pipereg7_d_sel = 0;
			ctrl_mux9to1_nop13_d_sel = 4;
			ctrl_mux3to1_nop9_d_sel = 2;
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
	case (pipereg14_q)
		OP_ADDI:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_ADDIU:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_ANDI:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_BEQ:
        begin
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_BGTZ:
        begin
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_BLEZ:
        begin
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_BNE:
        begin
			ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_COP0:
		case (pipereg17_q)
			COP0_MFC0:
			begin
				ctrl_cop0_tocpu_en = 1 &~stall_out_stage3;
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
			end
			COP0_MTC0:
				ctrl_cop0_fromcpu_en = 1 &~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
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
		endcase
		OP_COP2:
		case (pipereg15_q)
			COP2_FUNC_CFC2:
			begin
				ctrl_cop2_tocpu_en = 1 &~stall_out_stage3;
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
			end
			COP2_FUNC_CTC2:
            begin
				ctrl_cop2_fromcpu_en = 1 &~ctrl_cop0_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			COP2_FUNC_MTC2:
            begin
				ctrl_cop2_fromcpu_en = 1 &~ctrl_cop0_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
		endcase
		OP_JAL:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_LB:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
		end
		OP_LBU:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
		end
		OP_LH:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
		end
		OP_LHU:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
		end
		OP_LUI:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_mul_start = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
		end
		OP_LW:
		begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
		end
		OP_ORI:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_REGIMM:
		case (pipereg16_q[0])
			FUNC_BGEZ:
            begin
				ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_BLTZ:
            begin
				ctrl_branchresolve_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
		endcase
		OP_SB:
        begin
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_SH:
        begin
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_SLTI:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_SLTIU:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_SPECIAL:
		case (pipereg15_q)
			FUNC_ADD:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_ADDU:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_AND:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_DIV:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_div_en = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_mul_start = 0;
			end
			FUNC_DIVU:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_div_en = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_mul_start = 0;
			end
			FUNC_JALR:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_MFHI:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_MFLO:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_MULT:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_MULTU:
			begin
				ctrl_lo_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_hi_reg_en = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_reg_file_c_we = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_NOR:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_OR:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_SLL:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_SLLV:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_SLT:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_SLTU:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_SRA:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_SRAV:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_SRL:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_SRLV:
			begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
				ctrl_mul_start = 1 &~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
			end
			FUNC_SUB:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_SUBU:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
			FUNC_XOR:
            begin
				ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
                ctrl_cop0_fromcpu_en = 0;
                ctrl_cop0_tocpu_en = 0;
                ctrl_cop2_fromcpu_en = 0;
                ctrl_cop2_tocpu_en = 0;
                ctrl_lo_reg_en = 0;
                ctrl_hi_reg_en = 0;
                ctrl_branchresolve_en = 0;
                ctrl_data_mem_en = 0;
                ctrl_div_en = 0;
                ctrl_mul_start = 0;
            end
		endcase
		OP_SW:
        begin
			ctrl_data_mem_en = 1 &~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_reg_file_c_we = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
		OP_XORI:
        begin
			ctrl_reg_file_c_we = 1 &~ctrl_cop0_stalled&~ctrl_cop2_stalled&~ctrl_div_stalled&~ctrl_data_mem_stalled&~ctrl_mul_stalled&~stall_out_stage3;
            ctrl_cop0_fromcpu_en = 0;
            ctrl_cop0_tocpu_en = 0;
            ctrl_cop2_fromcpu_en = 0;
            ctrl_cop2_tocpu_en = 0;
            ctrl_lo_reg_en = 0;
            ctrl_hi_reg_en = 0;
            ctrl_branchresolve_en = 0;
            ctrl_data_mem_en = 0;
            ctrl_div_en = 0;
            ctrl_mul_start = 0;
        end
	endcase
end
		//Decode Logic for Opcode and Multiplex Select signals
always@(pipereg18_q or pipereg19_q or pipereg20_q or pipereg21_q)
begin
	
	case (pipereg18_q)
		OP_BEQ:
			ctrl_ifetch_op = 0;
		OP_BGTZ:
			ctrl_ifetch_op = 0;
		OP_BLEZ:
			ctrl_ifetch_op = 0;
		OP_BNE:
			ctrl_ifetch_op = 0;
		OP_REGIMM:
		case (pipereg20_q[0])
			FUNC_BGEZ:
				ctrl_ifetch_op = 0;
			FUNC_BLTZ:
				ctrl_ifetch_op = 0;
		endcase
		OP_SPECIAL:
		case (pipereg19_q)
			FUNC_JALR:
				ctrl_ifetch_op = 1;
			FUNC_JR:
				ctrl_ifetch_op = 1;
		endcase
        default: 
            ctrl_ifetch_op = 0;
	endcase
end
		//Logic for enable signals in Pipe Stage 3
always@(pipereg18_q or pipereg19_q or pipereg20_q[0] or pipereg21_q or 1'b0)
begin
	
	case (pipereg18_q)
		OP_BEQ:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_BGTZ:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_BLEZ:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_BNE:
			ctrl_ifetch_we = 1 &~1'b0;
		OP_REGIMM:
		case (pipereg20_q[0])
			FUNC_BGEZ:
				ctrl_ifetch_we = 1 &~1'b0;
			FUNC_BLTZ:
				ctrl_ifetch_we = 1 &~1'b0;
		endcase
		OP_SPECIAL:
		case (pipereg19_q)
			FUNC_JALR:
				ctrl_ifetch_we = 1 &~1'b0;
			FUNC_JR:
				ctrl_ifetch_we = 1 &~1'b0;
		endcase
        default:
            ctrl_ifetch_we = 0;
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
	case (ifetch_opcode)
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
		case (ifetch_rt[0])
			FUNC_BGEZ:
				predictme=1;
			FUNC_BLTZ:
				predictme=1;
		endcase
        default:
            predictme=0;
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

endmodule'''       

        return string.format(WIDTH=width, NUMREGS=numregs, LOG2NUMREGS=log2numregs)

    def write (self, width, numregs, log2numregs):
        self.fp.write(self.make_str(width, numregs, log2numregs))

if __name__ == '__main__':
	fp = open(parser.parse(), "w")
	isa = isa(fp)
	uut.write()
	fp.close()
	fp = open(parser.parse(), "a")
	fp.write("\r\r")
	uut = system(fp)
	uut.write()