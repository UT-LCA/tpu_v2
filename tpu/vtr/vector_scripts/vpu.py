from vlanes import vlanes
from vregfile_scalar import vregfile_scalar
from vregfile_inc import vregfile_inc
from vregfile_base import vregfile_base
from vregfile_control import vregfile_control
from vregfile_stride import vregfile_stride
from components import pipereg
from math import log
from optparse import OptionParser
import os
import re

parser = OptionParser()
(_,args) = parser.parse_args()

class vpu():
    def __init__(self, fp):
        self.fp = fp


    ####################################################################
    # This function handles case statements in vpu.v file.
    # It is sufficiently generic, but not generic to work for all files.
    ####################################################################
    def handle_case_stmt(self, case_init, case_stmt):
      case_result_string = "case(ir_op)\n"
      case_contents_ongoing = False
      dict_for_case_init = {}
      lines_in_case_init = case_init.split('\n')
      list_of_case_items = []

      #First iterate over the statements that are outside the actual
      #cases and that form the default initializations. We create a 
      #dict out of these.
      for line in lines_in_case_init:
        s = re.search(r'\s*(\w*)\s*=\s*(.*)\s*;', line) #line containing initialization
        b = re.match(r'^\s*$', line) #blank line
        assert(s is not None or b is not None)
        if s is not None:
          dict_for_case_init[s.group(1)] = s.group(2)
      
      #Now let's go over each of the actual cases.
      lines_in_case_stmt = case_stmt.split('\n')
      for line in lines_in_case_stmt:
        #Any line can be any of the following lines
        blank_line = re.match(r'^\s*$', line) 
        #Ex: COP2_VLD_B:
        case_item_line = re.search(r'\s*\w*:', line)
        begin_line = re.search(r'\bbegin\b', line)
        end_line = re.search(r'\bend\b', line)
        case_body_line = re.search(r'\s*(\w*)\s*=\s*(.*)\s*;', line)
        #Ex: COP2_VLD_B,
        #    COP2_VLD_L:
        continued_case_item_line = re.match(r'^\s*(\w*)\s*,\s*$', line)

        if blank_line is not None:
          continue

        #just create a list of case items
        elif continued_case_item_line is not None:
          list_of_case_items.append(continued_case_item_line.group(1))

        #if a new case_item started, but there was no 'end' to the previous one
        #let's write the contents out
        elif case_item_line is not None and case_contents_ongoing is True:
          case_result_string += "begin\n"
          for key in temp_dict:
            value = temp_dict[key]
            case_result_string += "{k} = {v};\n".format(k=key, v=value)
          case_result_string += "end\n"

          #do the same for all case items
          for citem in list_of_case_items:
            case_result_string += citem+":\n"
            case_result_string += "begin\n"
            for key in temp_dict:
              value = temp_dict[key]
              case_result_string += "{k} = {v};\n".format(k=key, v=value)
            case_result_string += "end\n"

          list_of_case_items = []

          temp_dict = {}
          case_contents_ongoing = False
          case_result_string += line+"\n"
          case_contents_ongoing = True
          temp_dict = dict_for_case_init.copy()

        #a fresh case item is here
        elif case_item_line is not None:
          case_result_string += line+"\n"
          case_contents_ongoing = True
          temp_dict = dict_for_case_init.copy()
        
        elif begin_line is not None:
          case_result_string += line+"\n"

        #contents of the body of a case item. let's keep collecting.
        elif case_body_line is not None and case_contents_ongoing is True:
          #override the value in dictionary if present, otherwise add new entry
          temp_dict[case_body_line.group(1)] = case_body_line.group(2)

        #at the end now. let's write it out.
        elif end_line is not None:
          for key in temp_dict:
            value = temp_dict[key]
            case_result_string += "{k} = {v};\n".format(k=key, v=value)
          case_result_string += line+"\n"

          #do the same for all case items
          for citem in list_of_case_items:
            case_result_string += citem+":\n"
            case_result_string += "begin\n"
            for key in temp_dict:
              value = temp_dict[key]
              case_result_string += "{k} = {v};\n".format(k=key, v=value)
            case_result_string += "end\n"

          list_of_case_items = []

          temp_dict = {}
          case_contents_ongoing = False

        else:
          print("Shouldn't have come here")
          raise SystemExit(0)

      #let's write the 'default' section of the case statement
      case_result_string += "default:\n"
      case_result_string += "begin\n"
      temp_dict = dict_for_case_init.copy()
      for key in temp_dict:
        value = temp_dict[key]
        case_result_string += "{k} = {v};\n".format(k=key, v=value)
      case_result_string += "end\n"
      case_result_string +=  "endcase\n"

      return case_result_string

    def make_str(self,log2dmem_writewidth,log2dmem_readwidth):
        numlanes = 8
        log2numlanes = 3
        nummemparallellanes = 8 
        log2nummemparallellanes = 3
        nummullanes = 8
        mvl = 64
        log2mvl = 6  
        vpw = 2 
        log2vpw = 1 
        lanewidth = 8 * vpw  
        log2lanewidth = log2vpw + 3 
        numbanks = 2 
        log2numbanks= 1 
        aluperbank = 0
        dmem_writewidth = pow(2,log2dmem_writewidth) 
        dmem_readwidth = pow(2,log2dmem_readwidth)
        vcwidth = 32 
        vswidth =  32
        numvsregs =  32
        log2numvsregs = 5 
   
        numnonmemvcregs = 32 
        numvbaseregs    = 16 
        numvincregs     = 8
        numvstrideregs  = 8

        log2numnonmemvcregs = int(log(numnonmemvcregs,2)) 
        log2numvbaseregs    = int(log(numvbaseregs,2))     
        log2numvincregs     = int(log(numvincregs,2))     
        log2numvstrideregs  = int(log(numvstrideregs,2))  

        string = ''' 

/******************************************************************************
  Vector Control Pipeline

          Stage 1       Stage 2         Stage 3
  ----|--------------|-----------------|-----------------|
      | Decode       | RF/EX           |WB & Send to cpu

 * Note, vs register file is written back to in Stage 3 also, but we check 
 * the vector lanes first to see if they're writing to the vs RF.  If so then
 * we stall.

******************************************************************************/

//`include "options.v"

//`include "vregfile_base.v"
//`include "vregfile_control.v"
//`include "vregfile_flag.v"
//`include "vregfile_inc.v"
//`include "vregfile_scalar.v"
//`include "vregfile_stride.v"
//`include "vregfile_vector.v"
//`include "vcomponents.v"
//`include "vlanes.v"

module vpu_{LOG2DMEM_WRITEWIDTH}_{LOG2DMEM_READWIDTH} (
    clk,
    resetn,

    // Instruction interface
    instr,
    instr_en,     // tells when instr is valid and available
    instr_wait,   // if high says vpu is not ready to receive
    has_memop,    // indicates vector pipeline has a memory operation
    mulshift_result,
    // For mtc2/ctc2 instructions
    scalar_in,    
    scalar_in_en,
    scalar_in_wait,

    // For cfc2 instructions
    scalar_out,
    scalar_out_en,
    scalar_out_wait,

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
    dbus_wait

    );

parameter LOG2NUMLANES=`LOG2NUMLANES;
parameter LOG2MVL=`LOG2MVL;
parameter LOG2VPW=`LOG2VPW;
parameter LOG2LANEWIDTH=`LOG2LANEWIDTHBITS;
parameter LOG2NUMMEMLANES=`LOG2NUMMEMLANES;
parameter NUMMULLANES=2**`LOG2NUMMULLANES;
parameter LOG2NUMBANKS=`LOG2NUMBANKS;
parameter ALUPERBANK=`ALUPERBANK;

parameter NUMLANES=2**LOG2NUMLANES;
parameter MVL=2**LOG2MVL;
parameter VPW=2**LOG2VPW;
parameter LANEWIDTH=2**LOG2LANEWIDTH;
parameter DMEM_WRITEWIDTH=2**{LOG2DMEM_WRITEWIDTH}; //Width of write bus to memory
parameter DMEM_READWIDTH=2**{LOG2DMEM_READWIDTH}; // Width of read bus from memory
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

// `include "visa.v"
'''

        try:
          visa_file = open("visa.v", "r")
          for line in visa_file:
            string += line+"\n"
          visa_file.close()
        except:
          print("Unable to open file")
          raise SystemExit(0)

        string += ''' 
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
output [{LANEWIDTH}*{NUMLANES}-1:0] mulshift_result;

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
    case(ir_op)
      0:
        ctrl_doesnt_use_lanes=1;
      COP2_VSATVL:
        ctrl_doesnt_use_lanes=1;
      //COP2_VMCTS:  //Omit since vlanes modifies scalar
      COP2_VMSTC:
        ctrl_doesnt_use_lanes=1;
      COP2_CFC2:
        ctrl_doesnt_use_lanes=1;
      COP2_CTC2:
        ctrl_doesnt_use_lanes=1;
    endcase
  end

  assign internal_stall[1]=internal_stall[2] | haz_vs_RAW;
  assign stall1=internal_stall[1] | (vlanes_stalled[1]&&~ctrl_doesnt_use_lanes);
  assign squash1 = (stall1&~stall2)|~resetn;

  pipereg_32 IR_reg2(ir,clk,resetn,~stall1,~squash1,ir2);

  assign ir_cop2=ir[31:26];
  assign ir_op={CBS}ir[25:22],ir[5:0]{CBE}; //10 bits
  assign ir_vcr=ir[15:10];
  assign ir_vsr=ir[15:11];
  assign ir_src1=ir[15:11];
  assign ir_src2=ir[20:16];
  assign ir_base={CBS}2'b10,ir[15:12]{CBE};
  assign ir_inc={CBS}3'b110,ir[11:9]{CBE};
  assign ir_stride={CBS}3'b111,ir[8:6]{CBE};

  assign wevalid = (ir_cop2==6'b010010) & ctrl_vc_we & (vcdest_s1!=48);

  pipereg_1 wevalid_reg1(wevalid,clk,~squash1, ~stall1,1'b1, wevalid_s2);

  assign vcdest_s1 = (ctrl_vcdest_sel==0) ? ir_vcr :
                    (ctrl_vcdest_sel==1) ? ir_base : 0;

  pipereg_{LOG2NUMVCREGS} vcdest_reg1(vcdest_s1,clk,~squash1, ~stall1,1'b1,
                      vcdest_s2);

  assign vsdest_s1 = (ctrl_vsdest_sel) ? ir_vsr : ir_src2;

  pipereg_{LOG2NUMVSREGS} vsdest_reg1(vsdest_s1,clk,~squash1, ~stall1,1'b1,
                      vsdest_s2);

  pipereg_{LOG2NUMVSREGS} vsdestsel_reg1
        (ctrl_vsdest_sel,clk,~squash1, ~stall1,1'b1, ctrl_vsdest_sel_s2);

  // Before reading all source operands, we need to:
  //  1. Determine what all the sources are
  always@*
  begin
  '''

        case_init = '''
    ctrl_vc_a_en=0;
    ctrl_vl_a_en=0;
    ctrl_vbase_a_en=0;
    ctrl_vinc_a_en=0;
    ctrl_vstride_a_en=0;
    ctrl_vs_a_en=0;
    '''
    #casez(ir_op)
        case_stmt = '''
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
      COP2_VMOD:
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
      COP2_VSAT_W:
          ctrl_vl_a_en=1;
      COP2_VSAT_SU_B:
          ctrl_vl_a_en=1;
      COP2_VSAT_SU_H:
          ctrl_vl_a_en=1;
      COP2_VSAT_SU_W:
          ctrl_vl_a_en=1;
      COP2_VSAT_SU_L:
          ctrl_vl_a_en=1;
      COP2_VSAT_U_B:
          ctrl_vl_a_en=1;
      COP2_VSAT_U_H:
          ctrl_vl_a_en=1;
      COP2_VSAT_U_W:
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
      COP2_VLD_W:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
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
      COP2_VLD_U_W:
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
      COP2_VLDS_W:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
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
      COP2_VLDS_U_W:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
      COP2_VLDX_B:
          ctrl_vbase_a_en=1;
      COP2_VLDX_H:
          ctrl_vbase_a_en=1;
      COP2_VLDX_W:
          ctrl_vbase_a_en=1;
      COP2_VLDX_L:
          ctrl_vbase_a_en=1;
      COP2_VLDX_U_B:
          ctrl_vbase_a_en=1;
      COP2_VLDX_U_H:
          ctrl_vbase_a_en=1;
      COP2_VLDX_U_W:
          ctrl_vbase_a_en=1;
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
      COP2_VST_W:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
        end
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
      COP2_VSTS_W:
        begin
          ctrl_vbase_a_en=1;
          ctrl_vinc_a_en=1;
          ctrl_vstride_a_en=1;
        end
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
      COP2_VSTX_W:
          ctrl_vbase_a_en=1;
      COP2_VSTX_L:
          ctrl_vbase_a_en=1;
      COP2_VSTXO_B:
          ctrl_vbase_a_en=1;
      COP2_VSTXO_H:
          ctrl_vbase_a_en=1;
      COP2_VSTXO_W:
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
'''
#    endcase
        string += self.handle_case_stmt(case_init, case_stmt)

        string += '''
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

  pipereg_{VCWIDTH} fwddata_reg (
       (ctrl_vbase_writedatain_sel_s2) ?  vbase_plus_vinc : vc_c_writedatain,
       clk,~squash1, ~stall1,1'b1,fwddata);

  pipereg_1  fwdvc_reg ( haz_vc,clk,~squash1, ~stall1,1'b1,fwd_vc);
  pipereg_1  fwdvl_reg ( haz_vl,clk,~squash1, ~stall1,1'b1,fwd_vl);
  pipereg_1  fwdvbase_reg ( haz_vbase,clk,~squash1, ~stall1,1'b1,fwd_vbase);
  pipereg_1  fwdvinc_reg ( haz_vinc,clk,~squash1, ~stall1,1'b1,fwd_vinc);
  pipereg_1  fwdvstride_reg(haz_vstride,clk,~squash1,~stall1,1,fwd_vstride);

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

  vregfile_control_{VCWIDTH}_{NUMNONMEMVCREGS}_{LOG2NUMNONMEMVCREGS} vregfile_control (
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
      .matmul_masks(matmul_masks)
      );

  vregfile_base_{VCWIDTH}_{NUMVBASEREGS}_{LOG2NUMVBASEREGS} vregfile_base (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vbase_a_reg), 
      .a_en(vbase_a_en), 
      .a_readdataout(vbase_a_readdataout),
      .c_reg(vbase_c_reg), 
      .c_writedatain(vbase_c_writedatain), 
      .c_we(vbase_c_we));

  vregfile_inc_{VCWIDTH}_{NUMVINCREGS}_{LOG2NUMVINCREGS} vregfile_inc (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vinc_a_reg), 
      .a_en(vinc_a_en), 
      .a_readdataout(vinc_a_readdataout),
      .c_reg(vinc_c_reg), 
      .c_writedatain(vinc_c_writedatain), 
      .c_we(vinc_c_we));

  vregfile_stride_{VCWIDTH}_{NUMVSTRIDEREGS}_{LOG2NUMVSTRIDEREGS} vregfile_stride (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vstride_a_reg), 
      .a_en(vstride_a_en), 
      .a_readdataout(vstride_a_readdataout),
      .c_reg(vstride_c_reg), 
      .c_writedatain(vstride_c_writedatain), 
      .c_we(vstride_c_we));

  vregfile_scalar_{VSWIDTH}_{NUMVSREGS}_{LOG2NUMVSREGS} vregfile_scalar (
      .clk(clk),
      .resetn(resetn), 
      .a_reg(vs_a_reg), 
      .a_en(vs_a_en), 
      .a_readdataout(vs_a_readdataout),
      .c_reg(vs_c_reg), 
      .c_writedatain(vs_c_writedatain), 
      .c_we(vs_c_we));

////////////////////////////////////////////////////////////      
//    parameters sequence for regfiles
//
//    defparam WIDTH=VSWIDTH;
//    defparam NUMREGS=NUMVSREGS;
//    defparam LOG2NUMREGS=LOG2NUMVSREGS;
////////////////////////////////////////////////////////////
  pipereg_6 vc_reg(ir_vcr,clk,resetn,vc_a_en,1'b1,ir_vcr_r);

  pipereg_1 r1(ctrl_scalarin_en,clk,~squash1,~stall1,1'b1,ctrl_scalarin_en_s2);
  pipereg_1 r2(ctrl_scalar_out_en,clk,~squash1,~stall1,1'b1,ctrl_scalar_out_en_s2);
  pipereg_1 r4(ctrl_vc_we,clk,~squash1,~stall1,1'b1,ctrl_vc_we_s2);
  pipereg_1 r5(ctrl_vbase_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl_vbase_writedatain_sel_s2);
  pipereg_1 r6(ctrl__vc_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl__vc_writedatain_sel_s2);
  pipereg_2 r7(ctrl_vc_writedatain_sel,clk,~squash1,~stall1,1'b1,ctrl_vc_writedatain_sel_s2);
  pipereg_1 r8(ctrl_vs_we,clk,~squash1,~stall1,1'b1,ctrl_vs_we_s2);


  /*********************** Control control signals ****************************/
  always@*
  begin
  '''

        case_init = '''
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
    '''

    #casez(ir_op)
        case_stmt = '''
      COP2_VSRR:  
      ctrl_rdvc_sel=2;
      COP2_VSRR_U:  
ctrl_rdvc_sel=2;
      COP2_VSLS:  
ctrl_rdvc_sel=2;
      COP2_VSLS_U:  
ctrl_rdvc_sel=2;
      COP2_VXUMUL:  
ctrl_rdvc_sel=2;
      COP2_VXUMUL_U:  
ctrl_rdvc_sel=2;
      COP2_VXLMUL:  
ctrl_rdvc_sel=2;
      COP2_VXLMUL_U:  
ctrl_rdvc_sel=2;
      COP2_VXUMADD:  
ctrl_rdvc_sel=2;
      COP2_VXUMADD_U:  
ctrl_rdvc_sel=2;
      COP2_VXUMSUB:  
ctrl_rdvc_sel=2;
      COP2_VXUMSUB_U:  
ctrl_rdvc_sel=2;
      COP2_VXLMADD:  
ctrl_rdvc_sel=2;
      COP2_VXLMADD_U:  
ctrl_rdvc_sel=2;
      COP2_VXLMSUB:  
ctrl_rdvc_sel=2;
      COP2_VXLMSUB_U:  
ctrl_rdvc_sel=2;
      COP2_VINS_VV:  
ctrl_rdvc_sel=2;
      COP2_VINS_VV:  
ctrl_rdvc_sel=3;
      COP2_VINS_SV:  
ctrl_rdvc_sel=3;
      COP2_VEXT_VV:  
ctrl_rdvc_sel=3;
      COP2_VEXT_SV:  
ctrl_rdvc_sel=3;
      COP2_VEXT_U_SV:  
ctrl_rdvc_sel=3;
      COP2_VFINS:  
ctrl_rdvc_sel=3;
      COP2_VHALFUP:  
ctrl_rdvc_sel=3;
      COP2_VHALFDN:  
ctrl_rdvc_sel=3;
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
      COP2_VLD_W,
      COP2_VLD_L,
      COP2_VLD_U_B,
      COP2_VLD_U_H,
      COP2_VLD_U_W,
      COP2_VLDS_B,
      COP2_VLDS_H,
      COP2_VLDS_W,
      COP2_VLDS_L,
      COP2_VLDS_U_B,
      COP2_VLDS_U_H,
      COP2_VLDS_U_W,
      COP2_VFST,
      COP2_VST_B,
      COP2_VST_H,
      COP2_VST_W,
      COP2_VST_L,
      COP2_VSTS_B,
      COP2_VSTS_H,
      COP2_VSTS_W,
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
      '''
    #endcase

        string += self.handle_case_stmt(case_init, case_stmt)

        string += '''    
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

  pipereg_{VCWIDTH} scalar_out_reg(
      vc_combined_out,clk,resetn,ctrl_scalar_out_en_s2,1'b1,vc_combined_out_s3);

  pipereg_1 scalar_out_en_reg(
      ctrl_scalar_out_en_s2,clk,~squash2,~stall2,1'b1,scalar_out_en_s3);


  /************** Datapath - Control Regs *******************/

  assign vc_c_we = ctrl_vc_we_s2&~vcdest_s2[5] & ~stall2;
  assign vc_c_reg = vcdest_s2[LOG2NUMNONMEMVCREGS-1:0]; 
  //temporary to feed to base,inc, and stride regfiles (without vl logic)
  assign _vc_c_writedatain= (ctrl__vc_writedatain_sel_s2) ? vs_a_readdataout :
                                                        scalar_in;
  assign vc_c_writedatain= (ctrl_vc_writedatain_sel_s2==0) ? _vc_c_writedatain :
                            //vhalf instruction
                            (ctrl_vc_writedatain_sel_s2==1) ? vl_readdataout>>1:
                            //vsatvl instruction
                            //(vl_readdataout>vc_readdataout) ? vc_readdataout :
                            (vl_readdataout>MVL) ? MVL : 
                            vl_readdataout;

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

  vlanes_{NUMLANES}_{LOG2NUMLANES}_{NUMMEMPARALLELLANES}_{LOG2NUMMEMPARALLELLANES}_{NUMMULLANES}_{MVL}_{LOG2MVL}_{VPW}_{LOG2VPW}_{LANEWIDTH}_{LOG2LANEWIDTH}_{NUMBANKS}_{LOG2NUMBANKS}_{ALUPERBANK}_{DMEM_WRITEWIDTH}_{LOG2DMEM_WRITEWIDTH}_{DMEM_READWIDTH}_{LOG2DMEM_READWIDTH}_{VCWIDTH}_{VSWIDTH}_{NUMVSREGS}_{LOG2NUMVSREGS} vlanes(
    .clk(clk),
    .resetn(resetn),

    // Instruction interface
    .instr(ir),
    .instr_en(is_cop2_s1),    // tells when instr is valid and available
    .instr_wait(),            // if high says vpu is not ready to receive

    .stall_in({CBS}internal_stall,1'b0{CBE}),
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
    .mulshift_result_s5(mulshift_result),
    // vs Writeback
    .vs_dst(vlanes_vs_dst),
    .vs_wetrack(vlanes_vs_wetrack),  //1-bit for each pipe-stage
    .vs_we(vlanes_vs_we),
    .vs_writedata(vlanes_vs_writedata),

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
    .dbus_wait(dbus_wait)
    );
  
///////////////////////////////////////////////////
//    vlanes parameter sequence for reference
//
//    vlanes.NUMLANES=NUMLANES,
//    vlanes.LOG2NUMLANES=LOG2NUMLANES,
//    vlanes.NUMMEMPARALLELLANES=NUMMEMLANES,
//    vlanes.LOG2NUMMEMPARALLELLANES=LOG2NUMMEMLANES,
//    vlanes.NUMMULLANES=NUMMULLANES,
//    vlanes.MVL=MVL,
//    vlanes.LOG2MVL=LOG2MVL,
//    vlanes.VPW=VPW,
//    vlanes.LOG2VPW=LOG2VPW,
//    vlanes.LANEWIDTH=LANEWIDTH,
//    vlanes.LOG2LANEWIDTH=LOG2LANEWIDTH,
//    vlanes.NUMBANKS=NUMBANKS,
//    vlanes.LOG2NUMBANKS=LOG2NUMBANKS,
//    vlanes.ALUPERBANK=ALUPERBANK,
//    vlanes.DMEM_WRITEWIDTH=DMEM_WRITEWIDTH, 
//    vlanes.LOG2DMEM_WRITEWIDTH={LOG2DMEM_WRITEWIDTH},
//    vlanes.DMEM_READWIDTH=DMEM_READWIDTH,
//    vlanes.LOG2DMEM_READWIDTH={LOG2DMEM_READWIDTH},
//    vlanes.VCWIDTH=VCWIDTH,
//    vlanes.VSWIDTH=VSWIDTH,
//    vlanes.NUMVSREGS=NUMVSREGS,
//    vlanes.LOG2NUMVSREGS=LOG2NUMVSREGS;
//////////////////////////////////////////////////////////
endmodule

                 '''
        fp = open("verilog/vregfile_scalar.v",'a')
        uut = vregfile_scalar(fp)
        uut.write(vcwidth,numvsregs,log2numvsregs)
        fp.close()
        fp = open("verilog/vregfile_stride.v",'a')
        uut = vregfile_stride(fp)
        uut.write(vcwidth,numvstrideregs,log2numvstrideregs)
        fp.close()
        fp = open("verilog/vregfile_inc.v",'a')
        uut = vregfile_inc(fp)
        uut.write(vcwidth,numvincregs,log2numvincregs)
        fp.close()
        fp = open("verilog/vregfile_base.v",'a')
        uut = vregfile_base(fp)
        uut.write(vcwidth,numvbaseregs,log2numvbaseregs)
        fp.close()
        fp = open("verilog/vregfile_control.v",'a')
        uut = vregfile_control(fp)
        uut.write(vcwidth,numnonmemvcregs,log2numnonmemvcregs,8)
        fp.close()
        fp = open("verilog/vlanes.v",'a')
        uut = vlanes(fp)
        uut.write(numlanes, log2numlanes, mvl, log2mvl, vpw, log2vpw, numbanks, log2numbanks, aluperbank, nummemparallellanes, log2nummemparallellanes, dmem_writewidth,log2dmem_writewidth, dmem_readwidth, log2dmem_readwidth, vcwidth, vswidth, numvsregs, log2numvsregs, 5)
        fp.close()
        fp = open("verilog/pipereg.v",'a')
        uut = pipereg(fp)
        uut.write(32)
        uut.write(log2numnonmemvcregs)
        uut.write(log2numvsregs)
        uut.write(vcwidth)
        uut.write(6)
        uut.write(2)
        uut.write(1)
        fp.close()


        return string.format(LOG2DMEM_WRITEWIDTH=log2dmem_writewidth , \
                             LOG2DMEM_READWIDTH= log2dmem_readwidth , \
                             NUMLANES = numlanes , \
                             LOG2NUMLANES = log2numlanes , \
                             NUMMEMPARALLELLANES = nummemparallellanes , \
                             LOG2NUMMEMPARALLELLANES = log2nummemparallellanes , \
                             NUMMULLANES = nummullanes , \
                             MVL = mvl , \
                             LOG2MVL = log2mvl , \
                             VPW = vpw , \
                             LOG2VPW = log2vpw , \
                             LANEWIDTH = lanewidth , \
                             LOG2LANEWIDTH = log2lanewidth , \
                             NUMBANKS = numbanks , \
                             LOG2NUMBANKS = log2numbanks, \
                             ALUPERBANK = aluperbank , \
                             DMEM_WRITEWIDTH = dmem_writewidth , \
                             DMEM_READWIDTH = dmem_readwidth , \
                             VCWIDTH = vcwidth , \
                             VSWIDTH = vswidth , \
                             NUMVSREGS = numvsregs , \
                             LOG2NUMVSREGS = log2numvsregs , \
                             NUMNONMEMVCREGS = numnonmemvcregs , \
                             NUMVCREGS = 64 , \
                             LOG2NUMVCREGS = 6 , \
                             NUMVBASEREGS = numvbaseregs , \
                             NUMVINCREGS = numvincregs , \
                             NUMVSTRIDEREGS = numvstrideregs , \
                             LOG2NUMNONMEMVCREGS = log2numnonmemvcregs , \
                             LOG2NUMVBASEREGS = log2numvbaseregs , \
                             LOG2NUMVINCREGS = log2numvincregs , \
                             LOG2NUMVSTRIDEREGS = log2numvstrideregs , \
                             CBS = "{" , \
                             CBE = "}" \
                             )
    def write (self,log2dmem_writewidth,log2dmem_readwidth):
        self.fp.write(self.make_str(log2dmem_writewidth,log2dmem_readwidth))


if __name__ == '__main__':
    fp = open(args[0], "w")
    #fp = open("verilog/vpu.v", "w")
    uut1 = vpu(fp)
    uut1.write(7,7)
    fp.close()
