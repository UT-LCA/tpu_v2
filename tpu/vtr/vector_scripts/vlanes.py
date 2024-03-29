from vdispatcher import vdispatcher
from vregfile_vector import vregfile_vector
from vregfile_flag import vregfile_flag
from vmem_unit import vmem_unit
from vmul_unit import vmul_unit
from vlane_alu import vlane_alu
from vlane_flagalu import vlane_flagalu
from matmul_unit import matmul_unit
from trp_unit import trp_unit
from bfloat_adder import bfloat_adder
from bfloat_mult import bfloat_mult
from activation import activation
from vcomponents import pipe,hazardchecker
from components import pipereg,onecyclestall

from math import log
import re
from optparse import OptionParser
parser = OptionParser()
(_,args) = parser.parse_args()

class vlanes():
    def __init__(self, fp):
        self.fp = fp

    def process_case_stmt(self,infile):
        capture_case_item_code = False
        case_stmt_start = False
        case_item_list = []
        captured_case_items = []
        line_num = 0
        outfile =""
    
        #Go over each line
        for line in infile.splitlines():
            line_num = line_num+1
            
            c_stmt_begin = None
            c_stmt_end = None
    
            c_stmt_begin = re.search(r'((\bcase\b)|(\bcasez\b))\(.*\)', line)
            c_stmt_end = re.search(r'endcase', line)
    
            #Case statement handling begins. ODIN doesn't support multiple case_items
            #separated by comma. See: https://github.com/verilog-to-routing/vtr-verilog-to-routing/issues/1676
            #So, we are converting to having single case_item only.
            if c_stmt_begin is not None:
                case_stmt_start = True
                #Replace casez with case while we're at it. ODIN doesn't like casez.
                outfile += (re.sub(r'casez', 'case', line))
                outfile += "\n"
            elif c_stmt_end is not None:
                case_stmt_start = False
                outfile += (line) 
                outfile += "\n"
            elif case_stmt_start is True:
                #Is this a line that has one of the multiple case_items
                #Assumption: one case item mentioned per line
                #Eg: COP2_VADD,
                case_item_multi = re.match(r'\s+([A-Z\d_]*),\s*$', line)
                if case_item_multi is not None:
                    case_item_list.append(case_item_multi.group(1))
                    continue
    
                #Is this a line that has the last case_item in a multi
                #case item list or the only case_item in a non-multi case
                #item list
                case_item_single = re.match(r'\s+([A-Z\d_]*):\s*$', line)
                if case_item_single is not None:
                    case_item_list.append(case_item_single.group(1))
                    continue
    
                #We created a list containing the names of case_items for this
                #piece of code
                #Assumption: Each case_item's code is enclosed in begin and end
                if len(case_item_list) > 0:
                    #We will capture the contents between the begin and end.
                    #These are to be copied for each case_item
                    begin_in_case = re.search(r'\s+begin', line)
                    if begin_in_case is not None:
                        captured_case_items.append(line)
                        capture_case_item_code = True
                        continue
                    end_in_case = re.search(r'\s+end', line)
                    if end_in_case is not None:
                        captured_case_items.append(line)
                        capture_case_item_code = False
                        #Start printing each case_item's code one by one.
                        #Basically, we just copy all lines in begin-end over
                        #and over for each case_item. 
                        for case_item in case_item_list:
                            outfile += "    "+case_item+":\n"
                            outfile += "\n"
                            outfile += (''.join(captured_case_items))
                            outfile += "\n"
                        case_item_list = []
                        captured_case_items = []
                        continue
                    if capture_case_item_code is True:
                        captured_case_items.append(line)
                        continue
                outfile += line
                outfile += "\n"
            
            #If we don't meet any of the criteria above, just write it to outfile
            else:
                outfile += line 
                outfile += "\n"
        return outfile

    def make_str(self, numlanes, log2numlanes, mvl, log2mvl, vpw, log2vpw, numbanks, log2numbanks, aluperbank, nummemparallellanes, log2nummemparallellanes, dmem_writewidth,log2dmem_writewidth, dmem_readwidth, log2dmem_readwidth, vcwidth, vswidth, numvsregs, log2numvsregs, vridwidth):
        lanewidth = 8*vpw
        log2lanewidth = 3+log2vpw
        nummullanes = numlanes
        pipe1regwidth = 3*(5+log2mvl-log2numlanes) + 41
        dispatcherwidth = 63 + 3*(5+log2mvl-log2numlanes) + vcwidth + vswidth
        log2numlanesp1 = 1+ log2numlanes
        totalalus = 4+(numbanks-1)*aluperbank
        totalflagalus = 1+(numbanks-1)*aluperbank
        regidwidth =  (vridwidth+log2mvl-log2numlanes)
        velmidwidth = log2mvl-log2numlanes
        totalvpw = 8 * vpw * numlanes
        totalregs = int(32 * mvl / numlanes)
        regwidthminusvr = regidwidth-vridwidth
        log2nummullanes = log(nummullanes,2)
        string_dispatchwidth ='''{LOG2MVL}+ \
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
    {REGIDWIDTH}+ \
    {REGIDWIDTH}+ \
    {REGIDWIDTH}+ \
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
    {VCWIDTH}+ \
    {VSWIDTH}+ \
    8'''
        string_dispatchwidth = string_dispatchwidth.format(LOG2MVL=log2mvl, REGIDWIDTH=regidwidth, VCWIDTH=vcwidth, VSWIDTH=vswidth)
        string_dispatchwidth = re.sub(r'\\', '', string_dispatchwidth)
        val = eval(string_dispatchwidth)
        string_dispatchwidth = "`define DISPATCHWIDTH" + " " + str(val) + "\n"

        string1 = ''' 

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

//`include "vdispatcher.v"
//`include "vlane_alu.v"
//`include "vmul_unit.v"
//`include "vmem_unit.v"
//`include "vlane_flagalu.v"
//`include "matmul_unit.v"

// `define LO(x,b) ((x)&~({CBS}1024{CBS}1'b1{CBE}{CBE}<<b))

module vlanes_{NUMLANES}_{LOG2NUMLANES}_{NUMMEMPARALLELLANES}_{LOG2NUMMEMPARALLELLANES}_{NUMMULLANES}_{MVL}_{LOG2MVL}_{VPW}_{LOG2VPW}_{LANEWIDTH}_{LOG2LANEWIDTH}_{NUMBANKS}_{LOG2NUMBANKS}_{ALUPERBANK}_{DMEM_WRITEWIDTH}_{LOG2DMEM_WRITEWIDTH}_{DMEM_READWIDTH}_{LOG2DMEM_READWIDTH}_{VCWIDTH}_{VSWIDTH}_{NUMVSREGS}_{LOG2NUMVSREGS} (
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
    mulshift_result_s5,
    // vs Writeback
    vs_writedata,
    vs_we,       // Actually issues write command for vs (after done stalling)
    vs_wetrack,  // says there exist a scalar write operation in pipe
    vs_dst,

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
    dbus_wait

    );

parameter NUMMEMPARALLELLANES=2;
parameter LOG2NUMMEMPARALLELLANES=1;
parameter LOG2NUMMULLANES={NUMLANES};

parameter VRIDWIDTH=5;
parameter VELMIDWIDTH={LOG2MVL}-{LOG2NUMLANES};
parameter REGIDWIDTH=VRIDWIDTH+{LOG2MVL}-{LOG2NUMLANES};
parameter BANKREGIDWIDTH=VRIDWIDTH+{LOG2MVL}-{LOG2NUMLANES}-{LOG2NUMBANKS};

// Register identifier = {CBS} vr[0-31], element {CBE}

// `define VRID_RANGE (REGIDWIDTH-1:REGIDWIDTH-VRIDWIDTH)
// `define VRELM_RANGE (REGIDWIDTH-VRIDWIDTH-1:0)

// NUMFUS = ALU*{NUMBANKS} + MUL + MEM + FALU*MEMBANKS + MATMUL(1) + BFLOAT
// UNITS(2) + ACTIVATION + TRP
parameter NUMFUS=4+2*({NUMBANKS}-1)*{ALUPERBANK}+1+5; //Adding 1 for matmul FU
parameter FU_ALU=0;
parameter FU_MUL=FU_ALU+({NUMBANKS}-1)*{ALUPERBANK}+1;
parameter FU_MEM=FU_MUL+1;
parameter FU_FALU=FU_MEM+2*({NUMBANKS}-1)*{ALUPERBANK}+1;
parameter FU_MATMUL=FU_FALU+1;
parameter FU_BFADDER = FU_MATMUL + 1;
parameter FU_BFMULT = FU_BFADDER + 1;
parameter FU_ACT = FU_BFMULT + 1;
parameter FU_TRP = FU_ACT + 1;

input clk;
input resetn;

//Instruction Format:
//9 - op[21] mask 1/0
//8 - op[23] scalar/vec src2
//7 - op[22] scalar/vec src1
//6:0 - {CBS}op[24],op[5:0]{CBE} op

input [31:0] instr;
input instr_en;     // tells when instr is valid and available
output instr_wait;   // if high says vpu is not ready to receive

input [3:0] stall_in;  
output [`MAX_PIPE_STAGES-1:0] is_stalled;
output has_memop;

// Control register values
input  [ {VCWIDTH}-1 : 0 ]   vc_in;
input  [ {VCWIDTH}-1 : 0 ]   vl_in;
input  [ {VCWIDTH}-1 : 0 ]   vbase_in;
input  [ {VCWIDTH}-1 : 0 ]   vinc_in;
input  [ {VCWIDTH}-1 : 0 ]   vstride_in;
input  [ {VSWIDTH}-1 : 0 ]   vs_in;
input  [3*`MAT_MUL_SIZE-1 : 0]  matmul_masks_in;

// vs Writeback
output       [ {VSWIDTH}-1 : 0 ]   vs_writedata;
output                           vs_we;
output               [ 5 : 2 ]   vs_wetrack;
output [ {LOG2NUMVSREGS}-1 : 0 ]   vs_dst;

// Data memory interface
output  [ 31 : 0 ]  dbus_address;
output              dbus_en;
output              dbus_we;
output  [ ({DMEM_WRITEWIDTH}/8)-1 : 0 ]   dbus_byteen;
output  [ {DMEM_WRITEWIDTH}-1 : 0 ]  dbus_writedata;
input   [ {DMEM_READWIDTH}-1 : 0 ]  dbus_readdata;
input               dbus_cachematch;
input               dbus_cachemiss;
input               dbus_wait;
output  [ 31 : 0 ]  dbus_prefetch;

output [ {LANEWIDTH}*{NUMLANES}-1 : 0 ] mulshift_result_s5;

//`include "visa.v"
parameter COP2_VADD           = 'b1000000000;
parameter COP2_VADD_U         = 'b1000000001;
parameter COP2_VSUB           = 'b1000000010;
parameter COP2_VSUB_U         = 'b1000000011;
parameter COP2_VMULHI         = 'b1000000100;
parameter COP2_VMULHI_U       = 'b1000000101;
parameter COP2_VDIV           = 'b1000000110; //Using as matmul
parameter COP2_VBFADD         = 'b0100000001; //Using BF16 add
parameter COP2_VBFMULT        = 'b0100000010; //Using BF16 MULT
parameter COP2_VACT           = 'b0100000011; //Using ACT
parameter COP2_VTRP           = 'b0100000100; //Using ACT
parameter COP2_VDIV_U         = 'b1000000111;
parameter COP2_VMOD           = 'b1000001000;
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
parameter COP2_VSAT_W         = 'b1010011111;
parameter COP2_VSAT_SU_B      = 'b1000100000;
parameter COP2_VSAT_SU_H      = 'b1001100000;
parameter COP2_VSAT_SU_W      = 'b1010100000;
parameter COP2_VSAT_SU_L      = 'b1011100000;
parameter COP2_VSAT_U_B       = 'b1000100001;
parameter COP2_VSAT_U_H       = 'b1001100001;
parameter COP2_VSAT_U_W       = 'b1010100001;
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
parameter COP2_VLD_W          = 'b1110100001;
parameter COP2_VLD_L          = 'b1111100001;
parameter COP2_VLD_U_B        = 'b1100100010;
parameter COP2_VLD_U_H        = 'b1101100010;
parameter COP2_VLD_U_W        = 'b1110100010;
parameter COP2_VLDS_B         = 'b1100100011;
parameter COP2_VLDS_H         = 'b1101100011;
parameter COP2_VLDS_W         = 'b1110100011;
parameter COP2_VLDS_L         = 'b1111100011;
parameter COP2_VLDS_U_B       = 'b1100100100;
parameter COP2_VLDS_U_H       = 'b1101100100;
parameter COP2_VLDS_U_W       = 'b1110100100;
parameter COP2_VLDX_B         = 'b1100100101;
parameter COP2_VLDX_H         = 'b1101100101;
parameter COP2_VLDX_W         = 'b1110100101;
parameter COP2_VLDX_L         = 'b1111100101;
parameter COP2_VLDX_U_B       = 'b1100100110;
parameter COP2_VLDX_U_H       = 'b1101100110;
parameter COP2_VLDX_U_W       = 'b1110100110;
parameter COP2_VFST           = 'b1100101000;
parameter COP2_VST_B          = 'b1100101001;
parameter COP2_VST_H          = 'b1101101001;
parameter COP2_VST_W          = 'b1110101001;
parameter COP2_VST_L          = 'b1111101001;
parameter COP2_VSTS_B         = 'b1100101010;
parameter COP2_VSTS_H         = 'b1101101010;
parameter COP2_VSTS_W         = 'b1110101010;
parameter COP2_VSTS_L         = 'b1111101010;
parameter COP2_VSTX_B         = 'b1100101011;
parameter COP2_VSTX_H         = 'b1101101011;
parameter COP2_VSTX_W         = 'b1110101011;
parameter COP2_VSTX_L         = 'b1111101011;
parameter COP2_VSTXO_B        = 'b1100101100;
parameter COP2_VSTXO_H        = 'b1101101100;
parameter COP2_VSTXO_W        = 'b1110101100;
parameter COP2_VSTXO_L        = 'b1111101100;
parameter COP2_VMCTS          = 'b1101110000;
parameter COP2_VMSTC          = 'b1101110001;
parameter COP2_CFC2           = 'b0000111000;
parameter COP2_CTC2           = 'b0000111010;
parameter COP2_MTC2           = 'b0000111011;


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
reg   [ {VCWIDTH}-1 : 0 ]   vc_in_saved;
reg   [ {VCWIDTH}-1 : 0 ]   vl_in_saved;
reg   [ {VCWIDTH}-1 : 0 ]   vbase_in_saved;
reg   [ {VCWIDTH}-1 : 0 ]   vinc_in_saved;
reg   [ {VCWIDTH}-1 : 0 ]   vstride_in_saved;
reg   [ {VSWIDTH}-1 : 0 ]   vs_in_saved;

// Control register values
//TODO: Rethink whether the following vars (vc, vl, vbase, vinc, vstride)
//need to be change to use `MAX_PIPE_STAGES. I think these variables are 
//only used for memory operations and for that, we only need 6 stages.
//Not a big deal because most of these are not used much. Synthesis tool
//will optimize out the unnecessary bits anyway.
wire  [ {VCWIDTH}-1 : 0 ]   vc[`MAX_PIPE_STAGES-1:2];
wire  [ {VCWIDTH}-1 : 0 ]   vl[`MAX_PIPE_STAGES-1:2];
wire  [ {VCWIDTH}-1 : 0 ]   vbase[`MAX_PIPE_STAGES-1:2];
reg   [ {VCWIDTH}-1 : 0 ]   vbase_s4;
wire  [ {VCWIDTH}-1 : 0 ]   vinc[`MAX_PIPE_STAGES-1:2];
wire  [ {VCWIDTH}-1 : 0 ]   vstride[`MAX_PIPE_STAGES-1:2];
wire  [ {VSWIDTH}-1 : 0 ]   vs[`MAX_PIPE_STAGES-1:2];
reg   [ {VSWIDTH}-1 : 0 ]   vs_s3[{NUMBANKS}-1:0];
reg   [ {VSWIDTH}-1 : 0 ]   vs_s4[NUMFUS-1:0];
reg   [ {VCWIDTH}-1 : 0 ]   vc_s3[{NUMBANKS}-1:0];
reg   [ {VCWIDTH}-1 : 0 ]   vc_s4[NUMFUS-1:0];

// Vector register file signals
reg   [ {NUMBANKS}*BANKREGIDWIDTH-1 : 0 ]   vr_a_reg;
wire     [ {NUMBANKS}*8*{VPW}*{NUMLANES}-1 : 0 ]   _vr_a_readdataout;
reg                         [{NUMBANKS}-1:0]   vr_a_en;
reg   [ {NUMBANKS}*BANKREGIDWIDTH-1 : 0 ]   vr_b_reg;
wire     [ {NUMBANKS}*8*{VPW}*{NUMLANES}-1 : 0 ]   _vr_b_readdataout;
reg                         [{NUMBANKS}-1:0]   vr_b_en;
reg   [ {NUMBANKS}*BANKREGIDWIDTH-1 : 0 ]   _vr_c_reg;
reg      [ {NUMBANKS}*8*{VPW}*{NUMLANES}-1 : 0 ]   _vr_c_writedatain;
reg        [ {NUMBANKS}*{VPW}*{NUMLANES}-1 : 0 ]   vr_c_byteen;
reg                         [{NUMBANKS}-1:0]   vr_c_we;

reg          [ {LANEWIDTH}*{NUMLANES}-1 : 0 ]   vr_a_readdataout[{NUMBANKS}-1:0];
reg          [ {LANEWIDTH}*{NUMLANES}-1 : 0 ]   vr_b_readdataout[{NUMBANKS}-1:0];
reg          [ {LANEWIDTH}*{NUMLANES}-1 : 0 ]   vr_c_writedatain[{NUMBANKS}-1:0];
reg           [ REGIDWIDTH-1 : 0 ]   vr_c_reg[{NUMBANKS}-1:0]; //for testbench and debugging

// Flag register file signals
reg       [ {NUMBANKS}*BANKREGIDWIDTH-1 : 0 ]   vf_a_reg;
wire               [ {NUMBANKS}*{NUMLANES}-1 : 0 ]   vf_a_readdataout;
reg                            [ {NUMBANKS}-1:0]   vf_a_en;
reg       [ {NUMBANKS}*BANKREGIDWIDTH-1 : 0 ]   vf_b_reg;
wire               [ {NUMBANKS}*{NUMLANES}-1 : 0 ]   vf_b_readdataout;
reg                            [ {NUMBANKS}-1:0]   vf_b_en;
reg       [ {NUMBANKS}*BANKREGIDWIDTH-1 : 0 ]   vf_c_reg;
reg                [ {NUMBANKS}*{NUMLANES}-1 : 0 ]   vf_c_writedatain;
reg                            [ {NUMBANKS}-1:0]   vf_c_we;

wire             [ REGIDWIDTH-1 : 0 ]   wb_dst[NUMFUS-1:0];
wire                     [NUMFUS-1:0]   wb_dst_we;
wire               [ {NUMLANES}-1 : 0 ]   wb_dst_mask[NUMFUS-1:0];

wire                        [ `MAX_PIPE_STAGES-1 : 4 ]   dst_we[NUMFUS-1:0];
wire             [(`MAX_PIPE_STAGES-4) * NUMFUS * REGIDWIDTH-1 : 0 ] dst;
wire               [ (`MAX_PIPE_STAGES-4) * NUMFUS * {NUMLANES}-1 : 0 ]   dst_mask;
wire             [ REGIDWIDTH-1 : 0 ]   dst_s2;
reg              [ REGIDWIDTH-1 : 0 ]   _dst_s3[{NUMBANKS}-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   dst_s3[{NUMBANKS}-1:0];
reg     [ {NUMBANKS}*REGIDWIDTH-1 : 0 ]   t_dst_s3;
reg              [ REGIDWIDTH-1 : 0 ]   dst_s4[NUMFUS-1:0];
reg                  [ NUMFUS-1 : 0 ]   dst_we_s4;
reg  [(1+({NUMBANKS}-1)*{ALUPERBANK})*REGIDWIDTH-1:0] alu_dst[4:4];
reg  [(1+({NUMBANKS}-1)*{ALUPERBANK})-1:0]            alu_dst_we[4:4];
reg  [(1+({NUMBANKS}-1)*{ALUPERBANK})*REGIDWIDTH-1:0] falu_dst[4:4];
reg  [(1+({NUMBANKS}-1)*{ALUPERBANK})-1:0]            falu_dst_we[4:4];

wire                                    imask_s2;
reg                [ {NUMBANKS}-1 : 0 ]   imask_s3;

wire             [ REGIDWIDTH-1 : 0 ]   src1_s2;
wire             [ REGIDWIDTH-1 : 0 ]   src2_s2;
reg              [ REGIDWIDTH-1 : 0 ]   src1_s3[{NUMBANKS}-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   src2_s3[{NUMBANKS}-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   _src1_s3[{NUMBANKS}-1:0];
reg              [ REGIDWIDTH-1 : 0 ]   _src2_s3[{NUMBANKS}-1:0];
wire                                    src1scalar_s2;
wire                                    src2scalar_s2;
reg                    [{NUMBANKS}-1:0]   src1scalar_s3;
reg                    [{NUMBANKS}-1:0]   src2scalar_s3;
reg                      [NUMFUS-1:0]   src1scalar_s4;
reg                      [NUMFUS-1:0]   src2scalar_s4;

reg                    [{NUMLANES}-1:0]   lane_en[{NUMBANKS}-1:0];
reg                    [{NUMLANES}-1:0]   vlane_en[NUMFUS-1:0];
reg                                     mem_last_subvector_s4;

reg                     [{LOG2MVL}-1:0]   src_start_delayed;
wire                    [{LOG2MVL}-1:0]   src_elm;
wire                    [{LOG2MVL}-1:0]   src_limit;
reg                     [{LOG2MVL}-1:0]   src_limit_s3[{NUMBANKS}-1:0];
wire                    [{LOG2MVL}-1:0]   src_start;

wire                    [{VCWIDTH}-1:0]   total_shamt;
wire                    [{LOG2MVL}-1:0]   dst_start;

reg          [ {LANEWIDTH}*{NUMLANES}-1 : 0 ]   vr_src1[NUMFUS-1:0];
reg          [ {LANEWIDTH}*{NUMLANES}-1 : 0 ]   vr_src2[NUMFUS-1:0];
wire         [ {LANEWIDTH}*{NUMLANES}-1 : 0 ]   matmul_out;
wire         [ {LANEWIDTH}*{NUMLANES}-1 : 0 ]   bfadder_result_s5;
wire         [ {LANEWIDTH}*{NUMLANES}-1 : 0 ]   bfmult_result_s5;
wire         [ {LANEWIDTH}*{NUMLANES}-1 : 0 ]   act_result_s5;
wire         [ {LANEWIDTH}*{NUMLANES}-1 : 0 ]   trp_out;
reg                [ {NUMLANES}-1 : 0 ]   vf_src1[NUMFUS-1:0];
reg                [ {NUMLANES}-1 : 0 ]   vf_src2[NUMFUS-1:0];
reg                [ {NUMLANES}-1 : 0 ]   vmask[NUMFUS-1:0];
reg                [ {NUMLANES}-1 : 0 ]   vmask_final[{NUMBANKS}-1:0];

reg        [ {VCWIDTH}*{NUMLANES}-1 : 0 ]   vstrideoffset_s4;
wire     [ {LANEWIDTH}*{NUMLANES}-1 : 0 ]   load_result_s5;
wire               [ {NUMLANES}-1 : 0 ]   load_result_mask_s5;

wire [ {LANEWIDTH}*{NUMLANES}-1 : 0 ] alu_result_s5[({NUMBANKS}-1)*{ALUPERBANK}:0];
wire           [ {NUMLANES}-1 : 0 ] alu_cmpresult_s4[({NUMBANKS}-1)*{ALUPERBANK}:0];
wire           [ {NUMLANES}-1 : 0 ] flagalu_result_s4[({NUMBANKS}-1)*{ALUPERBANK}:0];
wire           [ {NUMLANES}-1 : 0 ] flagalu_result_s5[({NUMBANKS}-1)*{ALUPERBANK}:0];

//Support 1 Lane processor
// wire [(({LOG2NUMLANES}>0) ? {LOG2NUMLANES} : 1)-1:0] elmshamt[`MAX_PIPE_STAGES-1:2];
wire [{LOG2NUMLANES}-1:0] elmshamt[`MAX_PIPE_STAGES-1:2];

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
reg ctrl1_memunit_en;
reg ctrl1_mem_en;
reg [6:0] ctrl1_memunit_op;
reg ctrl1_ismasked;
reg [2:0] ctrl1_flagalu_op;
reg ctrl1_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg ctrl1_volatiledest;
reg ctrl1_vrdest_sel;  //0-dest, 1-src2
reg ctrl1_vf_a_sel; //0-0/1 from instr, 1-src1

wire [5:4] ctrl_vs_we;
wire [6:0] ctrl_memunit_op[33-1:1];

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
wire ctrl2_alufalu_en;

reg [{NUMBANKS}-1:0] ctrl3_vr_a_en; // SRC1
reg [{NUMBANKS}-1:0] ctrl3_vr_b_en; // SRC2
reg [{NUMBANKS}-1:0] ctrl3_vr_c_we; // DEST
reg [{NUMBANKS}-1:0] ctrl3_vf_a_en;
reg [{NUMBANKS}-1:0] ctrl3_vf_b_en;
reg [{NUMBANKS}-1:0] ctrl3_vf_c_we;
reg [{NUMBANKS}-1:0] ctrl3_vs_we;
reg [{NUMBANKS}-1:0] ctrl3_useslanes;
reg [{NUMBANKS}-1:0] ctrl3_mem_dir_left;
reg [{NUMBANKS}-1:0] ctrl3_rshiftnonzero;
reg [10:0] ctrl3_alu_op[{NUMBANKS}-1:0];
reg [1:0] ctrl3_satsum_op[{NUMBANKS}-1:0];
reg [3:0] ctrl3_satsize_op[{NUMBANKS}-1:0];
reg [4:0] ctrl3_mulshift_op[{NUMBANKS}-1:0];
reg [{NUMBANKS}-1:0] ctrl3_memunit_en;
reg [{NUMBANKS}-1:0] ctrl3_mem_en;
reg [6:0] ctrl3_memunit_op[{NUMBANKS}-1:0];
reg [{NUMBANKS}-1:0] ctrl3_ismasked;
reg [2:0] ctrl3_flagalu_op[{NUMBANKS}-1:0];
reg [{NUMBANKS}-1:0] ctrl3_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg [{NUMBANKS}-1:0] ctrl3_volatiledest;
reg [{NUMBANKS}-1:0] ctrl3_vf_a_sel; //0-0/1 from instr, 1-src1
reg [{NUMBANKS}-1:0] ctrl3_mulshift_en;
reg [{NUMBANKS}-1:0] ctrl3_matmul_en;
reg [{NUMBANKS}-1:0] ctrl3_trp_en;
reg [{NUMBANKS}-1:0] ctrl3_bfadder_en;
reg [{NUMBANKS}-1:0] ctrl3_bfmult_en;
reg [{NUMBANKS}-1:0] ctrl3_act_en;
reg [{NUMBANKS}-1:0] ctrl3_alufalu_en;

reg ctrl4_mem_dir_left;
reg ctrl4_rshiftnonzero;
reg [10:0] ctrl4_alu_op[({NUMBANKS}-1)*{ALUPERBANK}:0];
reg [1:0] ctrl4_satsum_op[({NUMBANKS}-1)*{ALUPERBANK}:0];
reg [3:0] ctrl4_satsize_op[({NUMBANKS}-1)*{ALUPERBANK}:0];
reg [4:0] ctrl4_mulshift_op;
reg ctrl4_memunit_en;
reg ctrl4_mem_en;
reg [6:0] ctrl4_memunit_op;
reg [NUMFUS-1:0] ctrl4_ismasked;
reg [2:0] ctrl4_flagalu_op[({NUMBANKS}-1)*{ALUPERBANK}:0];
//reg ctrl4_vf_wbsel[({NUMBANKS}-1)*{ALUPERBANK}:0];   //0-flag ALU, 1-normal ALU
reg [({NUMBANKS}-1)*{ALUPERBANK}:0] ctrl4_vf_wbsel;   //0-flag ALU, 1-normal ALU
reg ctrl4_volatiledest;
//reg ctrl4_vf_a_sel[({NUMBANKS}-1)*{ALUPERBANK}:0]; //0-0/1 from instr, 1-src1
reg [({NUMBANKS}-1)*{ALUPERBANK}:0] ctrl4_vf_a_sel; //0-0/1 from instr, 1-src1
reg ctrl4_mulshift_en;
reg ctrl4_matmul_en;
reg ctrl4_trp_en;
reg ctrl4_bfadder_en;
reg ctrl4_bfmult_en;
reg ctrl4_act_en;

wire ctrl5_mem_en;

wire [REGIDWIDTH-VRIDWIDTH-1:0] regid_pad;

reg[31:0] bd;
//genvar  ba;
reg[31:0] bi;
reg[31:0] i;
reg[31:0] j;
//genvar  bk;
//genvar  k;
reg[31:0] m;
reg[31:0] n;
reg[31:0] b;
reg[31:0] b3;
reg[31:0] f3;
reg[31:0] bn;
reg[31:0] fn;
reg[31:0] fn2;
reg[31:0] bw;

wire [{NUMBANKS}*{LOG2MVL}-1:0] rdelm;
wire [{NUMBANKS}*{LOG2MVL}-1:0] wrelm;
// MSb of count entries indicates if instruction is dead or not.
wire [{NUMBANKS}*({LOG2MVL}-{LOG2NUMLANES}+1)-1:0] count;
reg  [{NUMBANKS}-1:0]   last_subvector;
wire [{NUMBANKS}-1:0]   first_subvector;
reg  [{NUMBANKS}-1:0] wrongbank_s3;
reg  [{NUMBANKS}-1:0] alive_s3;
reg  [{NUMBANKS}-1:0] banksel_s4[NUMFUS-1:0];

wire dispatcher_shift;
wire dispatcher_rotate;

wire [`MAX_PIPE_STAGES-1:0] internal_pipe_advance;
wire [`MAX_PIPE_STAGES-1:0] pipe_advance;
wire [`MAX_PIPE_STAGES-1:0] pipe_squash;
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

// DEBUG signals for Modelsim
wire  [7:0] d_instr[2:1];
reg   [7:0] d_instr_s3[{NUMBANKS}-1:0];
reg   [7:0] d_instr_s4[NUMFUS-1:0];
wire  [7:0] d_instr_s5[NUMFUS-1:0];
wire  [7:0] d_instr_s6[NUMFUS-1:0];
reg   [NUMFUS-1:0] D_last_subvector_s4;
wire  [NUMFUS-1:0] D_last_subvector_s5;
wire  [NUMFUS-1:0] D_last_subvector_s6;
wire  [NUMFUS-1:0] D_last_subvector_s31;
wire  [NUMFUS-1:0] D_wb_last_subvector;
reg   [{NUMBANKS}-1:0] D_last_subvector_done;
reg   [{NUMBANKS}-1:0] D_wb_instrdone;

// Module instance
  wire [15:0] debuginstrpipe_q;
  //assign {CBS}d_instr[2],d_instr[1]{CBE} = debuginstrpipe_q;
  assign d_instr[2] = debuginstrpipe_q[15:8];
  assign d_instr[1] = debuginstrpipe_q[7:0];

  pipe_8_1  debuginstrpipe (
      .d( {CBS}instr[25:24],instr[5:0]{CBE} ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[1] ),
      .squash( pipe_squash[1] ),
      .q(debuginstrpipe_q));

'''
        string2_basic ='''

// Module instance
    wire debugintrfupipereg1_Df_squashn;
    assign debugintrfupipereg1_Df_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg1_Df (
      .d( d_instr_s4[Df] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg1_Df_squashn),
      .q(d_instr_s5[Df]));

// Module instance
    wire debugintrfupipereg2_Df_squashn;
    assign debugintrfupipereg2_Df_squashn = ~pipe_squash[4];

    pipereg_8 debugintrfupipereg2_Df (
      .d( d_instr_s5[Df] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debugintrfupipereg2_Df_squashn),
      .q(d_instr_s6[Df]));

// Module instance
    wire debuglastpipereg1_Df_squashn;
    assign debuglastpipereg1_Df_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg1_Df (
      .d( D_last_subvector_s4[Df] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg1_Df_squashn),
      .q(D_last_subvector_s5[Df]));

// Module instance
    wire debuglastpipereg2_Df_squashn;
    assign debuglastpipereg2_Df_squashn = ~pipe_squash[4];

    pipereg_1 debuglastpipereg2_Df (
      .d( D_last_subvector_s5[Df] ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] ),
      .squashn(debuglastpipereg2_Df_squashn),
      .q(D_last_subvector_s6[Df]));

'''
        string2 = ""
        for Df in range(0,(9+2*(numbanks-1)*aluperbank)):
            string2 += string2_basic.replace("Df",str(Df))

        string3 ='''

//tell vpu to hold vc,vbase,etc values
assign is_stalled=~internal_pipe_advance | {CBS}stall_srcstart,2'b0{CBE};

assign has_memop=ctrl1_mem_en|ctrl2_mem_en|(|ctrl3_mem_en)|ctrl4_mem_en|ctrl5_mem_en;

/******************************************************************************/
/************************** Inter-Pipe Signals ********************************/
/******************************************************************************/

/* Memunit is in stage 5 but we stall in stage 4 so we don't squash the
* destination register contents*/

assign pipe_advance=internal_pipe_advance & ~{CBS}3'b0,stall_in{CBE};

assign internal_pipe_advance[0]=internal_pipe_advance[1];
assign internal_pipe_advance[1]=internal_pipe_advance[2];
assign internal_pipe_advance[2]=internal_pipe_advance[3] && 
                          ~stall_srcstart &&
                          ~stall_dispatcher &&
                          ~stall_hazsrc1 && ~stall_hazsrc2 && 
                          ~stall_hazfsrc1 && ~stall_hazfsrc2 && 
                          ~stall_mulcooldown;
assign internal_pipe_advance[3]=internal_pipe_advance[4];
assign internal_pipe_advance[4]=internal_pipe_advance[5];
assign internal_pipe_advance[5]=internal_pipe_advance[6] && ~stall_mulunit && ~stall_memunit;
//assign internal_pipe_advance[6]=1'b1;

assign pipe_squash[0]=pipe_advance[1]&~pipe_advance[0];
assign pipe_squash[1]=pipe_advance[2]&~pipe_advance[1];
assign pipe_squash[2]=pipe_advance[3]&~pipe_advance[2];
assign pipe_squash[3]=pipe_advance[4]&~pipe_advance[3];
assign pipe_squash[4]=pipe_advance[5]&~pipe_advance[4];
assign pipe_squash[5]=pipe_advance[6]&~pipe_advance[5];
//assign pipe_squash[6]=1'b0;

//The code below basically replicates the statements above for
//pipe stages from 6 to MAX_PIPE_STAGES. Using generate statement
//to reduce typing.

'''
        string4_basic = '''

  assign pipe_squash[pipe_stage] = pipe_advance[pipe_stage+1] & ~pipe_advance[pipe_stage];
  if (pipe_stage==(`MAX_PIPE_STAGES-2)) begin
    assign internal_pipe_advance[pipe_stage] = internal_pipe_advance[pipe_stage+1] && ~stall_matmul;
  end
  else begin
    assign internal_pipe_advance[pipe_stage] = internal_pipe_advance[pipe_stage+1];
  end

'''
        string4 = ""
        for pipe_stage in range(6,32):
            string4 += string4_basic.replace("pipe_stage",str(pipe_stage))  

        string5='''

assign pipe_squash[`MAX_PIPE_STAGES-1]=1'b0;
assign internal_pipe_advance[`MAX_PIPE_STAGES-1]=1'b1;

/******************************************************************************/
/************************** 1st Pipeline Stage ********************************/
/******************************************************************************/

  assign ir_op={CBS}instr[25:22],instr[5:0]{CBE}; //10 bits
  assign ir_dst=instr[10:6];
  assign ir_src1=instr[15:11];
  assign ir_src2=instr[20:16];
  assign ir_mask=instr[21];

  // Determine which instruction read from which ports
  always@*
  begin
//    ctrl1_vr_a_en=0;
//    ctrl1_vr_b_en=0;
//    ctrl1_vr_c_en=0;
//    ctrl1_vf_a_en=0;
//    ctrl1_vf_b_en=0;
//    ctrl1_vf_a_sel=0;
//    ctrl1_usesvssel=0;
    case(ir_op)
      COP2_VADD,
      COP2_VADD_U:
        begin
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VSUB,
      COP2_VSUB_U:
        begin
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
      COP2_VMULHI,
      COP2_VMULHI_U:
        begin
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VMOD,
      COP2_VMOD_U,
      COP2_VCMP_EQ,
      COP2_VCMP_NE,
      COP2_VCMP_LT,
      COP2_VCMP_U_LT,
      COP2_VCMP_LE,
      COP2_VCMP_U_LE:
        begin
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
      COP2_VDIV,
      COP2_VDIV_U,
      COP2_VMIN,
      COP2_VMIN_U,
      COP2_VMAX,
      COP2_VMAX_U,
      COP2_VMULLO:
        begin
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VABS:
        begin
          ctrl1_vr_b_en=0;
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_usesvssel=0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=1;
        end
      COP2_VAND,
      COP2_VOR,
      COP2_VXOR,
      COP2_VNOR:
        begin
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VSLL,
      COP2_VSRL,
      COP2_VSRA:
        begin
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_usesvssel=1;
        end
      COP2_VSAT_B,
      COP2_VSAT_H,
      COP2_VSAT_W,
      COP2_VSAT_SU_B,
      COP2_VSAT_SU_H,
      COP2_VSAT_SU_W,
      COP2_VSAT_SU_L,
      COP2_VSAT_U_B,
      COP2_VSAT_U_H,
      COP2_VSAT_U_W:
        begin
          ctrl1_vr_b_en=0;
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_usesvssel=0;
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
      COP2_VSADD,
      COP2_VSADD_U:
        begin
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VSSUB,
      COP2_VSSUB_U:
        begin
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VSRR,
      COP2_VSRR_U,
      COP2_VSLS,
      COP2_VSLS_U:
        begin
          ctrl1_vr_b_en=0;
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_usesvssel=0;
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
      COP2_VXUMUL,
      COP2_VXUMUL_U,
      COP2_VXLMUL,
      COP2_VXLMUL_U:
        begin
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VXUMADD,
      COP2_VXUMADD_U,
      COP2_VXUMSUB,
      COP2_VXUMSUB_U,
      COP2_VXLMADD,
      COP2_VXLMADD_U,
      COP2_VXLMSUB,
      COP2_VXLMSUB_U:
        begin
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=1;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_usesvssel=1;
        end
      COP2_VINS_VV:
        begin
          ctrl1_vr_c_en=0;
          ctrl1_vf_a_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_usesvssel=0;
          ctrl1_vr_a_en=1;
          ctrl1_vr_b_en=1;
        end
      //COP2_VINS_SV: doesn't read any vectors or flags
      COP2_VEXT_VV,
      COP2_VEXT_SV,
      COP2_VEXT_U_SV:
        begin
          ctrl1_vr_b_en=0;
          ctrl1_vr_c_en=0;
          ctrl1_vf_a_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_usesvssel=0;
          ctrl1_vr_a_en=1;
        end
      COP2_VCOMPRESS,
      COP2_VEXPAND:
        begin
          ctrl1_vr_b_en=0;
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_usesvssel=0;
          ctrl1_vr_a_en=1;
          ctrl1_vf_a_en=1;
        end
      COP2_VMERGE:
        begin
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_vr_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vr_b_en=~ir_op[BIT_VSSRC2];
          ctrl1_vf_a_en=1;
          ctrl1_usesvssel=1;
        end
      //COP2_VFINS:
      COP2_VEXTHALF,
      COP2_VHALF,
      COP2_VHALFUP,
      COP2_VHALFDN:
        begin
          ctrl1_vr_b_en=0;
          ctrl1_vr_c_en=0;
          ctrl1_vf_a_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_usesvssel=0;
          ctrl1_vr_a_en=1;
        end
      //COP2_VSATVL:
      COP2_VFAND,
      COP2_VFOR,
      COP2_VFXOR,
      COP2_VFNOR:
        begin
          ctrl1_vr_a_en=0;
          ctrl1_vr_b_en=0;
          ctrl1_vr_c_en=0;
          ctrl1_vf_a_en=~ir_op[BIT_VSSRC1];
          ctrl1_vf_b_en=1;
          ctrl1_vf_a_sel=1;
          ctrl1_usesvssel=1;
        end
      //COP2_VFCLR:
      //COP2_VFSET:
      COP2_VIOTA,
      COP2_VCIOTA,
      COP2_VFPOP,
      COP2_VFFF1,
      COP2_VFFL1,
      COP2_VFSETBF,
      COP2_VFSETIF,
      COP2_VFSETOF:
        begin
          ctrl1_vr_a_en=0;
          ctrl1_vr_b_en=0;
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_usesvssel=0;
          ctrl1_vf_a_en=1;
          ctrl1_vf_a_sel=1;
        end
      //COP2_VFMT8:
      //COP2_VFMF8:
      //COP2_VFCLR8:
      //COP2_VFOR8:
      //COP2_VFLD:
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
      COP2_VLDS_U_W:
        begin
          ctrl1_vr_a_en=0;
          ctrl1_vr_b_en=0;
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_usesvssel=0;
          ctrl1_vf_a_en=1;
        end
      COP2_VLDX_B,
      COP2_VLDX_H,
      COP2_VLDX_W,
      COP2_VLDX_L,
      COP2_VLDX_U_B,
      COP2_VLDX_U_H,
      COP2_VLDX_U_W:
        begin
          ctrl1_vr_a_en=0;
          ctrl1_vr_b_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_usesvssel=0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_c_en=1;
        end
      //COP2_VFST:
      COP2_VST_B,
      COP2_VST_H,
      COP2_VST_W,
      COP2_VST_L,
      COP2_VSTS_B,
      COP2_VSTS_H,
      COP2_VSTS_W,
      COP2_VSTS_L:
        begin
          ctrl1_vr_a_en=0;
          ctrl1_vr_c_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_usesvssel=0;
          ctrl1_vf_a_en=1;
          ctrl1_vr_b_en=1;
        end
      COP2_VSTX_B,
      COP2_VSTX_H,
      COP2_VSTX_W,
      COP2_VSTX_L,
      COP2_VSTXO_B,
      COP2_VSTXO_H,
      COP2_VSTXO_W,
      COP2_VSTXO_L:
        begin
          ctrl1_vr_a_en=0;
          ctrl1_vf_b_en=0;
          ctrl1_vf_a_sel=0;
          ctrl1_usesvssel=0;
          ctrl1_vr_b_en=1;
          ctrl1_vr_c_en=1;
          ctrl1_vf_a_en=1;
        end
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  // Decode enables 
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
    case(ir_op)
      COP2_VADD,
      COP2_VADD_U,
      COP2_VSUB,
      COP2_VSUB_U,
      COP2_VMULHI,
      COP2_VMULHI_U,
      COP2_VDIV,
      COP2_VDIV_U,
      COP2_VMOD,
      COP2_VMOD_U:
        begin
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
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
          ctrl1_volatiledest=0;
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
      COP2_VCMP_EQ,
      COP2_VCMP_NE,
      COP2_VCMP_LT,
      COP2_VCMP_U_LT,
      COP2_VCMP_LE,
      COP2_VCMP_U_LE:
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
          ctrl1_setvlto1=0;
          ctrl1_volatiledest=0;
          ctrl1_vf_c_we=1;
          ctrl1_vf_wbsel=1;
          ctrl1_ismasked=1;
        end
      COP2_VMIN,
      COP2_VMIN_U,
      COP2_VMAX,
      COP2_VMAX_U,
      COP2_VMULLO,
      COP2_VABS,
      COP2_VAND,
      COP2_VOR,
      COP2_VXOR,
      COP2_VNOR,
      COP2_VSLL,
      COP2_VSRL,
      COP2_VSRA,
      COP2_VSAT_B,
      COP2_VSAT_H,
      COP2_VSAT_W,
      COP2_VSAT_SU_B,
      COP2_VSAT_SU_H,
      COP2_VSAT_SU_W,
      COP2_VSAT_SU_L,
      COP2_VSAT_U_B,
      COP2_VSAT_U_H,
      COP2_VSAT_U_W,
      COP2_VSADD,
      COP2_VSADD_U,
      COP2_VSSUB,
      COP2_VSSUB_U,
      COP2_VSRR,
      COP2_VSRR_U,
      COP2_VSLS,
      COP2_VSLS_U:
        begin
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
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
          ctrl1_volatiledest=0;
          ctrl1_vr_d_we=1;
          ctrl1_ismasked=1;
        end
      COP2_VXUMUL,
      COP2_VXUMUL_U,
      COP2_VXLMUL,
      COP2_VXLMUL_U:
        begin
          ctrl1_vf_c_we=0;
          ctrl1_vs_we=0;
          ctrl1_vrdest_sel=0;
          ctrl1_elmshamt_sel=0;
          ctrl1_srcshamt_sel=0;
          ctrl1_srclimit_sel=0;
          ctrl1_dstshamt_sel=0;
          ctrl1_mem_dir_left=0;
          ctrl1_memunit_en=0;
          ctrl1_mem_en=0;
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
          ctrl1_volatiledest=0;
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
      COP2_VXUMADD,
      COP2_VXUMADD_U,
      COP2_VXUMSUB,
      COP2_VXUMSUB_U,
      COP2_VXLMADD,
      COP2_VXLMADD_U,
      COP2_VXLMSUB,
      COP2_VXLMSUB_U:
        begin
          ctrl1_vf_c_we=0;
          ctrl1_vs_we=0;
          ctrl1_vrdest_sel=0;
          ctrl1_elmshamt_sel=0;
          ctrl1_srcshamt_sel=0;
          ctrl1_srclimit_sel=0;
          ctrl1_dstshamt_sel=0;
          ctrl1_mem_dir_left=0;
          ctrl1_memunit_en=0;
          ctrl1_mem_en=0;
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
          ctrl1_volatiledest=0;
          ctrl1_vr_d_we=1;
          ctrl1_rshiftnonzero=1;
          ctrl1_ismasked=1;
        end
      COP2_VINS_VV:
        begin
          ctrl1_vf_c_we=0;
          ctrl1_vs_we=0;
          ctrl1_vrdest_sel=0;
          ctrl1_srclimit_sel=0;
          ctrl1_rshiftnonzero=0;
          ctrl1_mem_en=0;
          ctrl1_ismasked=0;
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
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
          ctrl1_vf_c_we=0;
          ctrl1_vs_we=0;
          ctrl1_vrdest_sel=0;
          ctrl1_srclimit_sel=0;
          ctrl1_rshiftnonzero=0;
          ctrl1_mem_en=0;
          ctrl1_ismasked=0;
          ctrl1_vf_wbsel=0;
          ctrl1_volatiledest=0;
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
          ctrl1_vf_c_we=0;
          ctrl1_vs_we=0;
          ctrl1_vrdest_sel=0;
          ctrl1_dstshamt_sel=0;
          ctrl1_rshiftnonzero=0;
          ctrl1_mem_en=0;
          ctrl1_ismasked=0;
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=3;
          ctrl1_srclimit_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_volatiledest=1;
        end
      COP2_VEXT_SV,
      COP2_VEXT_U_SV:
        begin
          ctrl1_vr_d_we=0;
          ctrl1_vf_c_we=0;
          ctrl1_vrdest_sel=0;
          ctrl1_dstshamt_sel=0;
          ctrl1_rshiftnonzero=0;
          ctrl1_mem_en=0;
          ctrl1_ismasked=0;
          ctrl1_vf_wbsel=0;
          ctrl1_volatiledest=0;
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
          ctrl1_vf_c_we=0;
          ctrl1_vs_we=0;
          ctrl1_vrdest_sel=0;
          ctrl1_elmshamt_sel=0;
          ctrl1_srcshamt_sel=0;
          ctrl1_srclimit_sel=0;
          ctrl1_dstshamt_sel=0;
          ctrl1_rshiftnonzero=0;
          ctrl1_mem_en=0;
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_mem_dir_left=0;
          ctrl1_ismasked=1;
          ctrl1_volatiledest=1;
        end
      COP2_VEXPAND:
        begin
          ctrl1_vf_c_we=0;
          ctrl1_vs_we=0;
          ctrl1_vrdest_sel=0;
          ctrl1_elmshamt_sel=0;
          ctrl1_srcshamt_sel=0;
          ctrl1_srclimit_sel=0;
          ctrl1_dstshamt_sel=0;
          ctrl1_rshiftnonzero=0;
          ctrl1_mem_en=0;
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
          ctrl1_memunit_en=1;
          ctrl1_vr_d_we=1;
          ctrl1_mem_dir_left=1;
          ctrl1_ismasked=1;
          ctrl1_volatiledest=1;
        end
      COP2_VMERGE:
        begin
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
          ctrl1_vr_d_we=1;
        end
      COP2_VFINS:
        begin
          ctrl1_vr_d_we=0;
          ctrl1_vf_c_we=0;
          ctrl1_vs_we=0;
          ctrl1_vrdest_sel=0;
          ctrl1_srclimit_sel=0;
          ctrl1_dstshamt_sel=0;
          ctrl1_rshiftnonzero=0;
          ctrl1_memunit_en=0;
          ctrl1_mem_en=0;
          ctrl1_ismasked=0;
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
          ctrl1_volatiledest=0;
          ctrl1_elmshamt_sel=3;
          ctrl1_srcshamt_sel=0;
          ctrl1_mem_dir_left=1;
        end
      COP2_VEXTHALF,
      COP2_VHALF:
        begin
          ctrl1_vf_c_we=0;
          ctrl1_vs_we=0;
          ctrl1_vrdest_sel=0;
          ctrl1_srclimit_sel=0;
          ctrl1_dstshamt_sel=0;
          ctrl1_rshiftnonzero=0;
          ctrl1_mem_en=0;
          ctrl1_ismasked=0;
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=1;
          ctrl1_srcshamt_sel=1;
          ctrl1_mem_dir_left=0;
          ctrl1_vr_d_we=1;
          ctrl1_volatiledest=1;
        end
      COP2_VHALFUP:
        begin
          ctrl1_vf_c_we=0;
          ctrl1_vs_we=0;
          ctrl1_vrdest_sel=0;
          ctrl1_srcshamt_sel=0;
          ctrl1_srclimit_sel=0;
          ctrl1_dstshamt_sel=0;
          ctrl1_rshiftnonzero=0;
          ctrl1_mem_en=0;
          ctrl1_ismasked=0;
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=2;
          ctrl1_srcshamt_sel=2;
          ctrl1_mem_dir_left=0;
          ctrl1_vr_d_we=1;
          ctrl1_volatiledest=1;
        end
      COP2_VHALFDN:
        begin
          ctrl1_vf_c_we=0;
          ctrl1_vs_we=0;
          ctrl1_vrdest_sel=0;
          ctrl1_srclimit_sel=0;
          ctrl1_rshiftnonzero=0;
          ctrl1_mem_en=0;
          ctrl1_ismasked=0;
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
          ctrl1_memunit_en=1;
          ctrl1_elmshamt_sel=2;
          ctrl1_srcshamt_sel=0;
          ctrl1_dstshamt_sel=2;
          ctrl1_mem_dir_left=1;
          ctrl1_vr_d_we=1;
          ctrl1_volatiledest=1;
        end
      //COP2_VSATVL:
      COP2_VFAND,
      COP2_VFOR,
      COP2_VFXOR,
      COP2_VFNOR,
      COP2_VFCLR,
      COP2_VFSET:
        begin
          ctrl1_vr_d_we=0;
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
          ctrl1_vf_c_we=1;
        end
      COP2_VIOTA,
      COP2_VCIOTA:
        begin
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
          ctrl1_vr_d_we=1;
        end
      COP2_VFPOP,
      COP2_VFFF1,
      COP2_VFFL1:
        begin
          ctrl1_vr_d_we=0;
          ctrl1_vf_c_we=0;
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
          ctrl1_vs_we=1;
        end
      COP2_VFSETBF,
      COP2_VFSETIF,
      COP2_VFSETOF,
      COP2_VFMT8,
      COP2_VFMF8,
      COP2_VFCLR8,
      COP2_VFOR8:
        begin
          ctrl1_vr_d_we=0;
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
          ctrl1_vf_c_we=1;
        end
      //COP2_VFLD,
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
      COP2_VLDS_U_W:
        begin
          ctrl1_vf_c_we=0;
          ctrl1_vs_we=0;
          ctrl1_elmshamt_sel=0;
          ctrl1_srcshamt_sel=0;
          ctrl1_srclimit_sel=0;
          ctrl1_dstshamt_sel=0;
          ctrl1_mem_dir_left=0;
          ctrl1_rshiftnonzero=0;
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
          ctrl1_volatiledest=0;
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
      COP2_VLDX_B,
      COP2_VLDX_H,
      COP2_VLDX_W,
      COP2_VLDX_L,
      COP2_VLDX_U_B,
      COP2_VLDX_U_H,
      COP2_VLDX_U_W:
        begin
          ctrl1_vf_c_we=0;
          ctrl1_vs_we=0;
          ctrl1_elmshamt_sel=0;
          ctrl1_srcshamt_sel=0;
          ctrl1_srclimit_sel=0;
          ctrl1_dstshamt_sel=0;
          ctrl1_mem_dir_left=0;
          ctrl1_rshiftnonzero=0;
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
          ctrl1_volatiledest=0;
          ctrl1_vr_d_we=1;
          ctrl1_vrdest_sel=1;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
        end
      //COP2_VFST:
      COP2_VST_B,
      COP2_VST_H,
      COP2_VST_W,
      COP2_VST_L,
      COP2_VSTS_B,
      COP2_VSTS_H,
      COP2_VSTS_W,
      COP2_VSTS_L,
      COP2_VSTX_B,
      COP2_VSTX_H,
      COP2_VSTX_W,
      COP2_VSTX_L,
      COP2_VSTXO_B,
      COP2_VSTXO_H,
      COP2_VSTXO_W,
      COP2_VSTXO_L:
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
          ctrl1_setvlto1=0;
          ctrl1_vf_wbsel=0;
          ctrl1_volatiledest=0;
          ctrl1_memunit_en=1;
          ctrl1_mem_en=1;
          ctrl1_ismasked=1;
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
    ctrl1_bfadder_en = 1'b0;
    ctrl1_bfmult_en = 1'b0;
    ctrl1_act_en = 1'b0;
    ctrl1_memunit_op=0;
    ctrl1_flagalu_op=FLAGOP_CLR;
  end
  always@*
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
      COP2_VBFMULT:   ctrl1_bfmult_en = 1'b1;
      COP2_VACT:      ctrl1_act_en = 1'b1;
      COP2_VTRP:      ctrl1_trp_en = 1'b1;
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
      COP2_VSAT_W: ctrl1_satsize_op=SATSIZEOP_VSATW;
      COP2_VSAT_SU_B: ctrl1_satsize_op=SATSIZEOP_VSATSUB;
      COP2_VSAT_SU_H: ctrl1_satsize_op=SATSIZEOP_VSATSUH;
      COP2_VSAT_SU_W: ctrl1_satsize_op=SATSIZEOP_VSATSUW;
      //COP2_VSAT_SU_L:
      COP2_VSAT_U_B: ctrl1_satsize_op=SATSIZEOP_VSATUB;
      COP2_VSAT_U_H: ctrl1_satsize_op=SATSIZEOP_VSATUH;
      COP2_VSAT_U_W: ctrl1_satsize_op=SATSIZEOP_VSATUW;
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
      COP2_VLD_W: ctrl1_memunit_op=MEMOP_LDW;
      //COP2_VLD_L,
      COP2_VLD_U_B: ctrl1_memunit_op=MEMOP_LDUB;
      COP2_VLD_U_H: ctrl1_memunit_op=MEMOP_LDUH;
      COP2_VLD_U_W: ctrl1_memunit_op=MEMOP_LDUW;
      COP2_VLDS_B: ctrl1_memunit_op=MEMOP_LDSB;
      COP2_VLDS_H: ctrl1_memunit_op=MEMOP_LDSH;
      COP2_VLDS_W: ctrl1_memunit_op=MEMOP_LDSW;
      //COP2_VLDS_L:
      COP2_VLDS_U_B: ctrl1_memunit_op=MEMOP_LDSUB;
      COP2_VLDS_U_H: ctrl1_memunit_op=MEMOP_LDSUH;
      COP2_VLDS_U_W: ctrl1_memunit_op=MEMOP_LDSUW;
      COP2_VLDX_B: ctrl1_memunit_op=MEMOP_LDXB;
      COP2_VLDX_H: ctrl1_memunit_op=MEMOP_LDXH;
      COP2_VLDX_W: ctrl1_memunit_op=MEMOP_LDXW;
      //COP2_VLDX_L:
      COP2_VLDX_U_B: ctrl1_memunit_op=MEMOP_LDXUB;
      COP2_VLDX_U_H: ctrl1_memunit_op=MEMOP_LDXUH;
      COP2_VLDX_U_W: ctrl1_memunit_op=MEMOP_LDXUW;
      //COP2_VFST:
      COP2_VST_B: ctrl1_memunit_op=MEMOP_STB;
      COP2_VST_H: ctrl1_memunit_op=MEMOP_STH;
      COP2_VST_W: ctrl1_memunit_op=MEMOP_STW;
      //COP2_VST_L:
      COP2_VSTS_B: ctrl1_memunit_op=MEMOP_STSB;
      COP2_VSTS_H: ctrl1_memunit_op=MEMOP_STSH;
      COP2_VSTS_W: ctrl1_memunit_op=MEMOP_STSW;
      //COP2_VSTS_L:
      COP2_VSTX_B: ctrl1_memunit_op=MEMOP_STXB;
      COP2_VSTX_H: ctrl1_memunit_op=MEMOP_STXH;
      COP2_VSTX_W: ctrl1_memunit_op=MEMOP_STXW;
      //COP2_VSTX_L:
      COP2_VSTXO_B: ctrl1_memunit_op=MEMOP_STXB;
      COP2_VSTXO_H: ctrl1_memunit_op=MEMOP_STXH;
      COP2_VSTXO_W: ctrl1_memunit_op=MEMOP_STXW;
      //COP2_VSTXO_L:
      //COP2_VMCTS:
      //COP2_VMSTC:
      //COP2_CFC2:
    endcase
  end

  assign regid_pad=0;

// Module instance
  wire [7:0] pipe1reg_secondstageonly_q;
  assign {CBS}
        ctrl2_elmshamt_sel,
        ctrl2_srcshamt_sel,
        ctrl2_srclimit_sel, //used to get squashed, pretend doesn't need to
        ctrl2_dstshamt_sel,
        ctrl2_setvlto1
      {CBE} = pipe1reg_secondstageonly_q;
  pipereg_8  pipe1reg_secondstageonly (
      .d( {CBS}
        ctrl1_elmshamt_sel,
        ctrl1_srcshamt_sel,
        ctrl1_srclimit_sel,
        ctrl1_dstshamt_sel,
        ctrl1_setvlto1
      {CBE}),
      .clk(clk),
      .resetn(resetn),
      .en(pipe_advance[1]),
      .squashn(1'b1),
      .q(pipe1reg_secondstageonly_q));

  // *********** Pipeline signals that need to be squashed *********

//module instance
  wire [16:0] pipe1regwsquash_q;
  wire pipe1regwsquash_squashn;
  wire pipe1regwsquash_d1;
  wire pipe1regwsquash_d2;
  wire pipe1regwsquash_d3;
  wire pipe1regwsquash_d4;

  assign {CBS}
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
        ctrl2_alufalu_en
        {CBE} = pipe1regwsquash_q;

  assign pipe1regwsquash_squashn = ~pipe_squash[1];

  assign pipe1regwsquash_d1 = ctrl1_vr_a_en|ctrl1_vr_c_en;
  assign pipe1regwsquash_d2 = ctrl1_vr_a_en|ctrl1_vr_b_en|ctrl1_vr_c_en| ctrl1_vr_d_we|ctrl1_vf_a_en|ctrl1_vf_b_en|ctrl1_vf_c_we;
  assign pipe1regwsquash_d3 = |ctrl1_mulshift_op;
  assign pipe1regwsquash_d4 = (ctrl1_alu_op!=(ALUOP_ZERO^ALUOP_ZERO)) || ctrl1_vf_c_we;

  pipereg_17  pipe1regwsquash (
      .d( {CBS}
        ctrl1_vr_d_we,
        ctrl1_vf_c_we,
        ctrl1_vs_we,
        pipe1regwsquash_d1,
        ctrl1_vr_b_en,
        ctrl1_vf_a_en,
        ctrl1_vf_b_en,
        pipe1regwsquash_d2,
        ctrl1_memunit_en,
        ctrl1_mem_en,
        pipe1regwsquash_d3,
        ctrl1_matmul_en,
        ctrl1_bfadder_en,
        ctrl1_bfmult_en,
        ctrl1_act_en,
        ctrl1_trp_en,
        pipe1regwsquash_d4
      {CBE}),
      .clk(clk),
      .resetn(resetn),
      .en(pipe_advance[1]),
      .squashn(pipe1regwsquash_squashn),
      .q(pipe1regwsquash_q)
      );

  // *********** Pipeline signals that don't need to be squashed *********
  wire [{PIPE1REGWIDTH}:0] pipe1reg_q;

  assign {CBS}
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
        ctrl2_volatiledest
      {CBE} = pipe1reg_q;

  pipereg_{PIPE1REGWIDTH}  pipe1reg (
      .d( {CBS}
        (ctrl1_vrdest_sel) ? {CBS}{CBS}REGIDWIDTH-VRIDWIDTH{CBS}1'b0{CBE}{CBE},ir_src2{CBE} : {CBS}ir_dst, regid_pad{CBE},
        (ctrl1_vr_c_en ) ? {CBS}{CBS}REGIDWIDTH-VRIDWIDTH{CBS}1'b0{CBE}{CBE},ir_dst{CBE} : {CBS}ir_src1, regid_pad{CBE},
        {CBS}ir_src2,regid_pad{CBE},
        ir_op[7] & ctrl1_usesvssel,
        ir_op[6] & ctrl1_usesvssel,
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
        ctrl1_volatiledest
      {CBE}),
      .clk(clk),
      .resetn(resetn),
      .en(pipe_advance[1]),
      .squashn( 1'b1 ),
      .q(pipe1reg_q));
  
  wire [6:1] squash_ctrlmemoppipe_NC;
  wire [5-1:0] ctrlmemoppipe_en;
  wire [7*(5+1)-1:0] ctrlmemoppipe_q;

  assign ctrlmemoppipe_en = pipe_advance[6:1] & {CBS}4'b1,ctrl2_memunit_en,1'b1{CBE};
  assign {CBS}ctrl_memunit_op[6],ctrl_memunit_op[5],ctrl_memunit_op[4],ctrl_memunit_op[3],ctrl_memunit_op[2],ctrl_memunit_op[1]{CBE} = ctrlmemoppipe_q;

// module instance
  pipe_7_5  ctrlmemoppipe (
      .d(ctrl1_memunit_op),
      .clk(clk),
      .resetn(resetn),
      .en(ctrlmemoppipe_en),
      .squash(squash_ctrlmemoppipe_NC),
      //.squash( pipe_squash[6:1] ),
      .q(ctrlmemoppipe_q));

/******************************************************************************/
/******************************* 2nd Pipeline Stage ***************************/
/******************************************************************************/

  // if src_start!=0 stall pipeline to calculate it and then do haz check
  wire shamtstall1;
  assign shamtstall1 = ctrl2_srcshamt_sel!=0;
  onecyclestall shamtstall(shamtstall1,clk,resetn,stall_srcstart);

  always@(posedge clk)
    if (!resetn || pipe_advance[1] )
      src_start_delayed<=0;
    else 
      src_start_delayed<=src_start;

  assign src_start= ( ctrl2_srcshamt_sel==3 ) ? vc[2] :           // vcindex
               ( ctrl2_srcshamt_sel==1 ) ? vl[2][{VCWIDTH}-1:1] :   // vl>>2
               ( ctrl2_srcshamt_sel==2 ) ? 1 << vc[2][{LOG2MVL}-1:0]://2^vcindex
               0;

  assign src_elm=src_start_delayed & (-1<<{LOG2NUMLANES});

  assign src_limit= ((ctrl2_setvlto1) ? 0 : vl[2][{LOG2MVL}-1:0] - 1'b1) +
                    ((ctrl2_srclimit_sel) ? vc[2][{LOG2MVL}-1:0] : 0);


  /******************* Adjust dest to account for shift  ****************/

  // Compute real destination register - accounting for shifting instructions

  assign dst_start= ( ctrl2_dstshamt_sel==3 ) ? vc[2] :           // vcindex
               ( ctrl2_dstshamt_sel==2 ) ? 1 << vc[2][{LOG2MVL}-1:0]://2^vcindex
               0;

  //assign dst_elm = {CBS}dst_start[{LOG2MVL}-1:0]>>{LOG2NUMLANES},{CBS}{LOG2NUMLANES}{CBS}1'b0{CBE}{CBE}{CBE};

  assign total_shamt= ( ctrl2_elmshamt_sel==3 ) ? vc[2] :          // vcindex
               ( ctrl2_elmshamt_sel==1 ) ? vl[2][{VCWIDTH}-1:1] :    // vl>>2
               ( ctrl2_elmshamt_sel==2 ) ? 1 << vc[2][{LOG2MVL}-1:0]://2^vcindex
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

  wire [5:2] squash_vcpipe_NC;
  wire [{VCWIDTH}-1:0] vcpipe_d;
  wire [{VCWIDTH}*(4+1)-1:0] vcpipe_q;
  wire [4-1:0] vcpipe_en;

  assign vcpipe_d = (!pipe_advance_s2_r) ? vc_in_saved : vc_in;
  assign {CBS}vc[6],vc[5],vc[4],vc[3],vc[2]{CBE} = vcpipe_q;
  assign vcpipe_en = pipe_advance[5:2] & {CBS}3'b1,ctrl2_memunit_en{CBE};

//module instance 
 pipe_{VCWIDTH}_4 vcpipe (
      .d(vcpipe_d),
      .clk(clk),
      .resetn(resetn),
      .en(vcpipe_en),
      .squash(squash_vcpipe_NC),
      .q(vcpipe_q));

  wire [5:2] squash_vlpipe_NC;
  
  wire [5*{VCWIDTH}-1:0] vlpipe_q;
  assign {CBS}vl[6],vl[5],vl[4],vl[3],vl[2]{CBE} = vlpipe_q;
//module instance 
  pipe_{VCWIDTH}_4 vlpipe (
      .d( (!pipe_advance_s2_r) ? vl_in_saved : vl_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[5:2] & {CBS}3'b1,ctrl2_memunit_en{CBE} ),
      .squash(squash_vlpipe_NC),
      .q(vlpipe_q));

  wire [5:2] squash_vbasepipe_NC;

//module instance 
  pipe_{VCWIDTH}_4 vbasepipe (
      .d( (!pipe_advance_s2_r) ? vbase_in_saved : vbase_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[5:2] & {CBS}3'b1,ctrl2_memunit_en{CBE} ),
      .squash(squash_vbasepipe_NC),
      .q( {CBS}vbase[6],vbase[5],vbase[4],vbase[3],vbase[2]{CBE} ));

  wire [5:2] squash_vincpipe_NC;

//module instance 
  pipe_{VCWIDTH}_4 vincpipe (
      .d( (!pipe_advance_s2_r) ? vinc_in_saved : vinc_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[5:2] & {CBS}3'b1,ctrl2_memunit_en{CBE} ),
      .squash(squash_vincpipe_NC),
      .q( {CBS}vinc[6],vinc[5],vinc[4],vinc[3],vinc[2]{CBE} ));

  //stride register also used for elmt shamt for vext/vhalf/etc in memunit
  wire [5:2] squash_vstridepipe_NC;

//module instance 
  pipe_{VCWIDTH}_4 vstridepipe (
      .d( (ctrl2_memunit_en&~ctrl2_mem_en) ? 
            ( 
                  total_shamt[{LOG2NUMLANES}-1:0] ) :
          (!pipe_advance_s2_r) ? vstride_in_saved : vstride_in ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[5:2] & {CBS}3'b1,ctrl2_memunit_en{CBE} ),
      .squash(squash_vstridepipe_NC),
      .q( {CBS}vstride[6],vstride[5],vstride[4],vstride[3],vstride[2]{CBE} ));


  /*****************************    ISSUER    *****************************/

  assign stall_dispatcher=
    //Structural hazard
    (|(ctrl3_memunit_en&~last_subvector) && ctrl2_memunit_en) ||
    (|(ctrl3_mulshift_en&~last_subvector) && ctrl2_mulshift_en) ||
    (|(ctrl3_matmul_en&~last_subvector) && ctrl2_matmul_en) ||
    (|(ctrl3_bfadder_en&~last_subvector) && ctrl2_bfadder_en) ||
    (|(ctrl3_bfmult_en&~last_subvector) && ctrl2_bfmult_en) ||
    (|(ctrl3_act_en&~last_subvector) && ctrl2_act_en) ||
    (|(ctrl3_trp_en&~last_subvector) && ctrl2_trp_en) ||
    (|(ctrl3_alufalu_en&~last_subvector) && ctrl2_alufalu_en && {ALUPERBANK}==0)||
    //Entry 0 is taken
    (dispatcher_rotate && ctrl2_useslanes);

  //assign stall_mulcooldown=|(ctrl3_mulshift_en&last_subvector);
  assign stall_mulcooldown=1'b0;

  assign dispatcher_shift=pipe_advance[3];
  // Rotate if last entry is alive (count not -1) and greater than zero
  assign dispatcher_rotate=(~count[{NUMBANKS}*({LOG2MVL}-{LOG2NUMLANES}+1)-1]) && 
                          ((count>>(({NUMBANKS}-1)*({LOG2MVL}-{LOG2NUMLANES}+1)))!=0);

wire [{NUMBANKS}*(`DISPATCHWIDTH)-1:0] dispatcher_instr;

// module instance
  vdispatcher_{NUMBANKS}_{DISPATCHERWIDTH}_{LOG2MVL}_{LOG2MVL}_{LOG2NUMLANESP1} vdispatcher(
      .clk(clk),
      .resetn(resetn),
      .shift(dispatcher_shift),
      .rotate(dispatcher_rotate), //rotate and shift must be high to rotate
      .inshift_instr( {CBS}
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
          (pipe_squash[2]) ? 8'b0 : d_instr[2]
          {CBE}),
      .inshift_first(ctrl2_useslanes),
      .inshift_rdelm(src_start),
      .inshift_wrelm(dst_start),
      .inshift_count((pipe_squash[2] || !ctrl2_useslanes) ? -1 : (src_limit[{LOG2MVL}-1:0]>>{LOG2NUMLANES}) - (src_elm[{LOG2MVL}-1:0]>>{LOG2NUMLANES}) ),
      .increment((~wrongbank_s3)&{CBS}{NUMBANKS}{CBS}pipe_advance[3]{CBE}{CBE}),
      .rdelm_add_sub(0),
      .wrelm_add_sub(0),
      .count_add_sub(1),
      .rdelm_valuetoadd({NUMLANES}),
      .wrelm_valuetoadd({NUMLANES}),
      .count_valuetoadd(1),
      .instr(dispatcher_instr),
      .first(first_subvector),
      .rdelm(rdelm),
      .wrelm(wrelm),
      .count(count)
    );

  

  /******************* Stall on RAW HAZARDS ************************/

  always@*
    for (bd=0; bd<1+({NUMBANKS}-1)*{ALUPERBANK}; bd=bd+1)
    begin
      alu_dst[4][bd*REGIDWIDTH +: REGIDWIDTH] = dst[ 4*(NUMFUS*REGIDWIDTH) + (FU_ALU+bd) * REGIDWIDTH +: REGIDWIDTH];

      alu_dst_we[4][bd] = dst_we[FU_ALU+bd][4];

      falu_dst[4][bd*REGIDWIDTH +: REGIDWIDTH] = dst[4*(NUMFUS*REGIDWIDTH) + (FU_ALU+bd) * REGIDWIDTH +: REGIDWIDTH];

      falu_dst_we[4][bd] = dst_we[FU_FALU+bd][4];
    end

//module instance 
  hazardchecker_{REGIDWIDTH}_{VELMIDWIDTH}_{TOTALALUS}_{NUMBANKS} src1hazchecker(
      .src( src1_s2 |(src_start_delayed[{LOG2MVL}-1:0]>>{LOG2NUMLANES}) ),
      .src_valid(ctrl2_vr_a_en),
      .dst({CBS}
        alu_dst,
        dst[4*(NUMFUS*REGIDWIDTH)+(FU_MUL) * REGIDWIDTH +: REGIDWIDTH],
        dst[4*(NUMFUS*REGIDWIDTH)+(FU_MEM) * REGIDWIDTH +: REGIDWIDTH],
        dst[4*(NUMFUS*REGIDWIDTH)+(FU_MATMUL) * REGIDWIDTH +: REGIDWIDTH]
        {CBE}),
    //  .dst(),
      .dst_valid({CBS}
        alu_dst_we[4],
        dst_we[FU_MUL][4],
        dst_we[FU_MEM][4],
        dst_we[FU_MATMUL][4]
        {CBE}),
      .dst_mode({CBS}{CBS}3+({NUMBANKS}-1)*{ALUPERBANK}{CBS}1'b0{CBE}{CBE},ctrl4_volatiledest{CBE}),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vr_c_we),
      .lt_mode(ctrl3_volatiledest),
      .haz(stall_hazsrc1));
//module instance 

  hazardchecker_{REGIDWIDTH}_{VELMIDWIDTH}_{TOTALALUS}_{NUMBANKS} src2hazchecker(
      .src( src2_s2 |(src_start_delayed[{LOG2MVL}-1:0]>>{LOG2NUMLANES}) ),
      .src_valid(ctrl2_vr_b_en),
      .dst({CBS}
        alu_dst,
        dst[4*(NUMFUS*REGIDWIDTH)+(FU_MUL) * REGIDWIDTH +: REGIDWIDTH],
        dst[4*(NUMFUS*REGIDWIDTH)+(FU_MEM) * REGIDWIDTH +: REGIDWIDTH],
        dst[4*(NUMFUS*REGIDWIDTH)+(FU_MATMUL) * REGIDWIDTH +: REGIDWIDTH]
        {CBE}),
     // .dst(),
      .dst_valid({CBS}
        alu_dst_we[4],
        dst_we[FU_MUL][4],
        dst_we[FU_MEM][4],
        dst_we[FU_MATMUL][4]
        {CBE}),
      .dst_mode({CBS}{CBS}3+({NUMBANKS}-1)*{ALUPERBANK}{CBS}1'b0{CBE}{CBE},ctrl4_volatiledest{CBE}),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vr_c_we),
      .lt_mode(ctrl3_volatiledest),
      .haz(stall_hazsrc2));

  //Check flag hazards - flags always start at 0th element
//module instance 
  hazardchecker_{REGIDWIDTH}_{VELMIDWIDTH}_{TOTALFLAGALUS}_{NUMBANKS} fsrc1hazchecker(
      // Check for mask flag and src1 flag depending on instruction
      .src( (ctrl2_vf_a_sel) ? src1_s2 : (imask_s2<<VELMIDWIDTH)),
      .src_valid(ctrl2_vf_a_en),
      .dst(falu_dst),
      .dst_valid(falu_dst_we),
      .dst_mode(0),
      //.lt_dst(t_dst_s3),
      .lt_dst(0),
      //.lt_dst_valid(ctrl3_vf_c_we),
      .lt_dst_valid(0),
      .lt_mode(0),
      .haz(stall_hazfsrc1));

//module instance 
  hazardchecker_{REGIDWIDTH}_{VELMIDWIDTH}_{TOTALFLAGALUS}_{NUMBANKS} fsrc2hazchecker(
      .src( src2_s2 ),
      .src_valid(ctrl2_vf_b_en),
      .dst(falu_dst),
      .dst_valid(falu_dst_we),
      .dst_mode(0),
      .lt_dst(t_dst_s3),
      .lt_dst_valid(ctrl3_vf_c_we),
      .lt_mode(0),
      .haz(stall_hazfsrc2));


/******************************************************************************/
/************************** 3rd Pipeline Stage ********************************/
/******************************************************************************/

  always@*
    for (bi=0; bi<{NUMBANKS}; bi=bi+1)
    begin
      {CBS}
        src_limit_s3[bi],
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
        ctrl3_alufalu_en[bi],
        _dst_s3[bi],
        _src1_s3[bi],
        _src2_s3[bi],
        src1scalar_s3[bi],
        src2scalar_s3[bi],
        imask_s3[bi],
        ctrl3_vf_a_sel[bi],
        ctrl3_rshiftnonzero[bi],
        ctrl3_mem_dir_left[bi],
        ctrl3_alu_op[bi],
        ctrl3_satsum_op[bi],
        ctrl3_satsize_op[bi],
        ctrl3_mulshift_op[bi],
        ctrl3_memunit_op[bi],
        ctrl3_ismasked[bi],
        ctrl3_flagalu_op[bi],
        ctrl3_vf_wbsel[bi],
        ctrl3_volatiledest[bi],
        vc_s3[bi],
        vs_s3[bi],
        d_instr_s3[bi]
      {CBE} = dispatcher_instr>>(bi*(`DISPATCHWIDTH));

      last_subvector[bi]=~|count[bi*({LOG2MVL}-{LOG2NUMLANES}+1)+:{LOG2MVL}-{LOG2NUMLANES}];
      alive_s3[bi]=~count[(bi+1)*({LOG2MVL}-{LOG2NUMLANES}+1)-1];

//      for (i=0; i<{NUMLANES}; i=i+1)
//        lane_en[bi][i]= ({LOG2NUMLANES}==0) || //Support 1 lane
//          ~((first_subvector[bi]) && 
//              i<`LO(rdelm[bi*{LOG2MVL} +: {LOG2MVL}],{LOG2NUMLANES}) ||
//            (last_subvector[bi]) && 
//              i>src_limit_s3[bi][(({LOG2NUMLANES}>0) ? {LOG2NUMLANES} : 1)-1:0] );
    end


  /************* Map from issuer to register file banks *************/
  always@*
    for (b=0; b<{NUMBANKS}; b=b+1)
    begin 
      dst_s3[b]=_dst_s3[b] | (wrelm[b*{LOG2MVL}+:{LOG2MVL}]>>{LOG2NUMLANES});
      t_dst_s3[b*REGIDWIDTH +: REGIDWIDTH]=dst_s3[b];
      src1_s3[b]=_src1_s3[b] | (rdelm[b*{LOG2MVL}+:{LOG2MVL}]>>{LOG2NUMLANES});
      src2_s3[b]=_src2_s3[b] | (rdelm[b*{LOG2MVL}+:{LOG2MVL}]>>{LOG2NUMLANES});

      // wrongbank_s3[b]=(`LO(rdelm[b*{LOG2MVL}+:{LOG2MVL}]>>{LOG2NUMLANES},{LOG2NUMBANKS})!=b);
       wrongbank_s3[b]=(((rdelm[b*{LOG2MVL}+:{LOG2MVL}]>>{LOG2NUMLANES})&({CBS}{CBS}({REGIDWIDTH}-{LOG2NUMBANKS}){CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE})) != b );

      vr_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src1_s3[b]>>{LOG2NUMBANKS};
      vr_b_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH]=src2_s3[b]>>{LOG2NUMBANKS};
      vr_a_en[b]=ctrl3_vr_a_en[b] && pipe_advance[3];
      vr_b_en[b]=ctrl3_vr_b_en[b] && pipe_advance[3];
     
    if ((ctrl3_vf_a_sel[b]))
    begin
      vf_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH] = _src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-1) : ((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-VRIDWIDTH)] >> 1;
    end
    else
    begin
      vf_a_reg[b*BANKREGIDWIDTH +: BANKREGIDWIDTH] = {CBS}imask_s3[b], src1_s3[((b-0)*((REGIDWIDTH-1)-(0)+1)+REGIDWIDTH-VRIDWIDTH-1-0) : ((b-0)*((REGIDWIDTH-1)-(0)+1)+0)]{CBE} >> 1;
    end
      vf_a_en[b]=ctrl3_vf_a_en[b] & pipe_advance[3];
      vf_b_en[b]=ctrl3_vf_b_en[b] & pipe_advance[3];
    end

  /************* Map from issuer/banks to Functional Units *************/
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
      ctrl4_memunit_en=0;
      ctrl4_mem_en=|ctrl3_mem_en;
      ctrl4_volatiledest=0;

      for (f3=0; f3<NUMFUS; f3=f3+1)
      begin
        d_instr_s4[f3]=0;
        D_last_subvector_s4[f3]=0;
        dst_s4[f3]=0;
      end
      for (b3=0; b3<{NUMBANKS}; b3=b3+1)
      begin
        //if instruction is alive && is in correct bank
        if ( alive_s3[b3]  &&     //alive
            ~wrongbank_s3[b3] &&  //correct bank
            ~pipe_squash[3])
          if (ctrl3_mulshift_en[b3])    //is multiply
          begin
            d_instr_s4[FU_MUL]=d_instr_s3[b3];
            D_last_subvector_s4[FU_MUL]=last_subvector[b3];
            ctrl4_mulshift_en=ctrl3_mulshift_en[b3];
            banksel_s4[FU_MUL]=b3;
            vs_s4[FU_MUL]=vs_s3[b3];
            vc_s4[FU_MUL]=vc_s3[b3];
            dst_s4[FU_MUL]=dst_s3[b3];
            dst_we_s4[FU_MUL]=ctrl3_vr_c_we[b3];
            vlane_en[FU_MUL]=lane_en[b3];
            src1scalar_s4[FU_MUL]=src1scalar_s3[b3];
            src2scalar_s4[FU_MUL]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MUL]=ctrl3_ismasked[b3];
            ctrl4_mulshift_op=ctrl3_mulshift_op[b3];
            ctrl4_rshiftnonzero=ctrl3_rshiftnonzero[b3];
          end
          else if (ctrl3_matmul_en[b3])    //is matmul
          begin
            d_instr_s4[FU_MATMUL]=d_instr_s3[b3];
            D_last_subvector_s4[FU_MATMUL]=last_subvector[b3];
            ctrl4_matmul_en=ctrl3_matmul_en[b3];
            banksel_s4[FU_MATMUL]=b3;
            vs_s4[FU_MATMUL]=vs_s3[b3];
            vc_s4[FU_MATMUL]=vc_s3[b3];
            dst_s4[FU_MATMUL]=dst_s3[b3];
            dst_we_s4[FU_MATMUL]=ctrl3_vr_c_we[b3];
            vlane_en[FU_MATMUL]=lane_en[b3];
            src1scalar_s4[FU_MATMUL]=src1scalar_s3[b3];
            src2scalar_s4[FU_MATMUL]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MATMUL]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_bfadder_en[b3])    //is bfloat addition
          begin
            d_instr_s4[FU_BFADDER]=d_instr_s3[b3];
            D_last_subvector_s4[FU_BFADDER]=last_subvector[b3];
            ctrl4_bfadder_en=ctrl3_bfadder_en[b3];
            banksel_s4[FU_BFADDER]=b3;
            vs_s4[FU_BFADDER]=vs_s3[b3];
            vc_s4[FU_BFADDER]=vc_s3[b3];
            dst_s4[FU_BFADDER]=dst_s3[b3];
            dst_we_s4[FU_BFADDER]=ctrl3_vr_c_we[b3];
            vlane_en[FU_BFADDER]=lane_en[b3];
            src1scalar_s4[FU_BFADDER]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFADDER]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFADDER]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_bfmult_en[b3])    //is bfloat addition
          begin
            d_instr_s4[FU_BFMULT]=d_instr_s3[b3];
            D_last_subvector_s4[FU_BFMULT]=last_subvector[b3];
            ctrl4_bfmult_en = ctrl3_bfmult_en[b3];
            banksel_s4[FU_BFMULT]=b3;
            vs_s4[FU_BFMULT]=vs_s3[b3];
            vc_s4[FU_BFMULT]=vc_s3[b3];
            dst_s4[FU_BFMULT]=dst_s3[b3];
            dst_we_s4[FU_BFMULT]=ctrl3_vr_c_we[b3];
            vlane_en[FU_BFMULT]=lane_en[b3];
            src1scalar_s4[FU_BFMULT]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFMULT]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFMULT]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_trp_en[b3])    //is bfloat addition
          begin
            d_instr_s4[FU_BFMULT]=d_instr_s3[b3];
            D_last_subvector_s4[FU_BFMULT]=last_subvector[b3];
            ctrl4_act_en = ctrl3_act_en[b3];
            banksel_s4[FU_BFMULT]=b3;
            vs_s4[FU_BFMULT]=vs_s3[b3];
            vc_s4[FU_BFMULT]=vc_s3[b3];
            dst_s4[FU_BFMULT]=dst_s3[b3];
            dst_we_s4[FU_BFMULT]=ctrl3_vr_c_we[b3];
            vlane_en[FU_BFMULT]=lane_en[b3];
            src1scalar_s4[FU_BFMULT]=src1scalar_s3[b3];
            src2scalar_s4[FU_BFMULT]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_BFMULT]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_act_en[b3])    //is bfloat addition
          begin
            d_instr_s4[FU_TRP]=d_instr_s3[b3];
            D_last_subvector_s4[FU_TRP]=last_subvector[b3];
            ctrl4_act_en = ctrl3_act_en[b3];
            banksel_s4[FU_TRP]=b3;
            vs_s4[FU_TRP]=vs_s3[b3];
            vc_s4[FU_TRP]=vc_s3[b3];
            dst_s4[FU_TRP]=dst_s3[b3];
            dst_we_s4[FU_TRP]=ctrl3_vr_c_we[b3];
            vlane_en[FU_TRP]=lane_en[b3];
            src1scalar_s4[FU_TRP]=src1scalar_s3[b3];
            src2scalar_s4[FU_TRP]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_TRP]=ctrl3_ismasked[b3];
          end
          else if (ctrl3_memunit_en[b3] && !ctrl3_mem_en[b3]) //is memunit shift
          begin
            d_instr_s4[FU_MEM]=d_instr_s3[b3];
            D_last_subvector_s4[FU_MEM]=last_subvector[b3];
            ctrl4_memunit_en=ctrl3_memunit_en[b3];
            banksel_s4[FU_MEM]=b3;
            vs_s4[FU_MEM]=vs_s3[b3];
            dst_s4[FU_MEM]=dst_s3[b3];
            dst_we_s4[FU_MEM]=ctrl3_vr_c_we[b3];
            vlane_en[FU_MEM]=lane_en[b3];
            src1scalar_s4[FU_MEM]=src1scalar_s3[b3];
            src2scalar_s4[FU_MEM]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MEM]=ctrl3_ismasked[b3];
            ctrl4_volatiledest=ctrl3_volatiledest[b3];
            ctrl4_mem_dir_left=ctrl3_mem_dir_left[b3];
            ctrl4_memunit_op=ctrl3_memunit_op[b3];
            mem_last_subvector_s4=last_subvector[b3];
          end
          else if (ctrl3_memunit_en[b3] &&  ctrl3_mem_en[b3]) //is mem operation
          begin
            d_instr_s4[FU_MEM]=d_instr_s3[b3];
            D_last_subvector_s4[FU_MEM]=last_subvector[b3];
            ctrl4_memunit_en=ctrl3_memunit_en[b3];
            banksel_s4[FU_MEM]=b3;
            // Use vs to store current length for prefetching
            vs_s4[FU_MEM]=count[b3*({LOG2MVL}-{LOG2NUMLANES}+1)+:({LOG2MVL}-{LOG2NUMLANES})]<<{LOG2NUMLANES};
            dst_s4[FU_MEM]=dst_s3[b3];
            dst_we_s4[FU_MEM]=ctrl3_vr_c_we[b3];
            vlane_en[FU_MEM]=lane_en[b3];
            src1scalar_s4[FU_MEM]=src1scalar_s3[b3];
            src2scalar_s4[FU_MEM]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_MEM]=ctrl3_ismasked[b3];
            ctrl4_volatiledest=ctrl3_volatiledest[b3];
            ctrl4_mem_dir_left=ctrl3_mem_dir_left[b3];
            ctrl4_memunit_op=ctrl3_memunit_op[b3];
            //Load base on first subvector or if INDEXED memory operation
//            vbase_s4=(|(first_subvector&ctrl3_mem_en) || ctrl_memunit_op[3][5])? vbase[3] : vbase_s4 + ((((ctrl_memunit_op[3][4])? vstride[3]: 1)<<ctrl_memunit_op[3][3:2])<<{LOG2NUMLANES});
//            // Partial Address Gen for each lane - just do multiplication part
//            for (m=0; m<{NUMLANES}; m=m+1)
//              vstrideoffset_s4[m*{VCWIDTH} +: {VCWIDTH}] = ((ctrl_memunit_op[3][4]) ? vstride[3] : 1)*m;
            mem_last_subvector_s4=last_subvector[b3];
          end
          else if (ctrl3_alu_op[b3]!=(ALUOP_ZERO^ALUOP_ZERO)) //is ALU
          begin
            d_instr_s4[FU_ALU+b3*{ALUPERBANK}]=d_instr_s3[b3];
            D_last_subvector_s4[FU_ALU+b3*{ALUPERBANK}]=last_subvector[b3];
            banksel_s4[FU_ALU+b3*{ALUPERBANK}]=b3;
            vs_s4[FU_ALU+b3*{ALUPERBANK}]=vs_s3[b3];
            dst_we_s4[FU_ALU+b3*{ALUPERBANK}]=ctrl3_vr_c_we[b3];
            dst_s4[FU_ALU+b3*{ALUPERBANK}]=dst_s3[b3];
            dst_we_s4[FU_FALU+b3*{ALUPERBANK}]=ctrl3_vf_c_we[b3];
            dst_s4[FU_FALU+b3*{ALUPERBANK}]=dst_s3[b3];
            vlane_en[FU_ALU+b3*{ALUPERBANK}]=lane_en[b3];
            src1scalar_s4[FU_ALU+b3*{ALUPERBANK}]=src1scalar_s3[b3];
            src2scalar_s4[FU_ALU+b3*{ALUPERBANK}]=src2scalar_s3[b3];
            ctrl4_ismasked[FU_ALU+b3*{ALUPERBANK}]=ctrl3_ismasked[b3];
            ctrl4_alu_op[b3*{ALUPERBANK}]=ctrl3_alu_op[b3];
            ctrl4_satsum_op[b3*{ALUPERBANK}]=ctrl3_satsum_op[b3];
            ctrl4_satsize_op[b3*{ALUPERBANK}]=ctrl3_satsize_op[b3];
            ctrl4_vf_wbsel[b3*{ALUPERBANK}]=ctrl3_vf_wbsel[b3];
          end
          else if (ctrl3_vf_c_we[b3])
          begin                                    //is FALU
            d_instr_s4[FU_FALU+b3*{ALUPERBANK}]=d_instr_s3[b3];
            D_last_subvector_s4[FU_FALU+b3*{ALUPERBANK}]=last_subvector[b3];
            banksel_s4[FU_FALU+b3*{ALUPERBANK}]=b3;
            vs_s4[FU_FALU+b3*{ALUPERBANK}]=|vs_s3[b3];
            dst_we_s4[FU_FALU+b3*{ALUPERBANK}]=ctrl3_vf_c_we[b3];
            dst_s4[FU_FALU+b3*{ALUPERBANK}]=dst_s3[b3];
            vlane_en[FU_FALU+b3*{ALUPERBANK}]=lane_en[b3];
            src1scalar_s4[FU_FALU+b3*{ALUPERBANK}]=src1scalar_s3[b3];
            src2scalar_s4[FU_FALU+b3*{ALUPERBANK}]=src2scalar_s3[b3];
            ctrl4_flagalu_op[b3*{ALUPERBANK}]=ctrl3_flagalu_op[b3];
            ctrl4_vf_wbsel[b3*{ALUPERBANK}]=ctrl3_vf_wbsel[b3];
            ctrl4_vf_a_sel[b3*{ALUPERBANK}]=ctrl3_vf_a_sel[b3];
          end
        end
      end

// module instance
  vregfile_vector_{NUMBANKS}_{LOG2NUMBANKS}_{TOTALVPW}_{TOTALREGS}_{REGIDWIDTH} vregfile_vector (
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

// module instance
  vregfile_flag_{NUMBANKS}_{LOG2NUMBANKS}_{NUMLANES}_{TOTALREGS}_{REGIDWIDTH} vregfile_flag (
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


// module instance
  pipereg_1 sdstwepipereg (
      .d( |ctrl3_vs_we ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[3] ),
      .squashn( ~pipe_squash[3] ),
      .q(ctrl_vs_we[4]));

/******************************************************************************/
/************************** 4th Pipeline Stage ********************************/
/******************************************************************************/

  //Convert register file width (8*{VPW}) to datpath width ({LANEWIDTH})
  always@*
    for (bn=0; bn<{NUMBANKS}; bn=bn+1)
      for (n=0; n<{NUMLANES}; n=n+1)
      begin
        vr_a_readdataout[bn][{LANEWIDTH}*n +: {LANEWIDTH}] =          _vr_a_readdataout[bn*8*{VPW}*{NUMLANES} + 8*{VPW}*n +: 8*{VPW}];
        vr_b_readdataout[bn][{LANEWIDTH}*n +: {LANEWIDTH}] =           _vr_b_readdataout[bn*8*{VPW}*{NUMLANES} + 8*{VPW}*n +: 8*{VPW}];
        _vr_c_writedatain[bn*8*{VPW}*{NUMLANES} + 8*{VPW}*n +: 8*{VPW}] =      vr_c_writedatain[bn][{LANEWIDTH}*n +: {LANEWIDTH}];
      end

  //Bank Multiplex for each functional unit
 always@*
 begin
   for (fn=0; fn<=FU_MUL; fn=fn+1)
   begin
     vr_src1[fn] =(src1scalar_s4[fn]) ? {CBS}{NUMLANES}{CBS}vs_s4[fn][{LANEWIDTH}-1:0]{CBE}{CBE} :(banksel_s4[fn]? vr_a_readdataout[1]: vr_a_readdataout[0]);

     vr_src2[fn] =(src2scalar_s4[fn]) ? {CBS}{NUMLANES}{CBS}vs_s4[fn][{LANEWIDTH}-1:0]{CBE}{CBE} :(banksel_s4[fn]? vr_b_readdataout[1]: vr_b_readdataout[0]);

     vf_src1[fn] = banksel_s4[fn] ? vf_a_readdataout[1*{NUMLANES} +: {NUMLANES}] : vf_a_readdataout[0*{NUMLANES} +: {NUMLANES}] ;

     vmask[fn] =vlane_en[fn] & ((ctrl4_ismasked[fn]) ? ( banksel_s4[fn]? vf_a_readdataout[1*{NUMLANES} +: {NUMLANES}]: vf_a_readdataout[0*{NUMLANES} +: {NUMLANES}] ) : {CBS}{NUMLANES}{CBS}1'b1{CBE}{CBE});
   end

   vr_src1[FU_MEM] = banksel_s4[FU_MEM]? vr_a_readdataout[1]: vr_a_readdataout[0] ;

   vr_src2[FU_MEM] = banksel_s4[FU_MEM]? vr_b_readdataout[1]: vr_b_readdataout[0];

   vmask[FU_MEM] =  vlane_en[FU_MEM] & ((ctrl4_ismasked[FU_MEM]) ?(banksel_s4[FU_MEM] ?  vf_a_readdataout[1*{NUMLANES} +: {NUMLANES}] :  vf_a_readdataout[0*{NUMLANES} +: {NUMLANES}] ) : {CBS}{NUMLANES}{CBS}1'b1{CBE}{CBE}) ;

   vr_src1[FU_MATMUL] =(src1scalar_s4[FU_MATMUL]) ? {CBS}{NUMLANES}{CBS}vs_s4[FU_MATMUL][{LANEWIDTH}-1:0]{CBE}{CBE} :(banksel_s4[FU_MATMUL]? vr_a_readdataout[1]: vr_a_readdataout[1]);

   vr_src2[FU_MATMUL] =(src2scalar_s4[FU_MATMUL]) ? {CBS}{NUMLANES}{CBS}vs_s4[FU_MATMUL][{LANEWIDTH}-1:0]{CBE}{CBE} :(banksel_s4[FU_MATMUL]? vr_b_readdataout[1]: vr_b_readdataout[1]);

   vmask[FU_MATMUL] =  vlane_en[FU_MATMUL] & ((ctrl4_ismasked[FU_MATMUL]) ? (banksel_s4[FU_MATMUL]? vf_a_readdataout[1*{NUMLANES} +: {NUMLANES}] : vf_a_readdataout[0*{NUMLANES} +: {NUMLANES}] ) : {CBS}{NUMLANES}{CBS}1'b1{CBE}{CBE});

   for (fn2=FU_FALU; fn2<=FU_FALU+({NUMBANKS}-1)*{ALUPERBANK}; fn2=fn2+1)
   begin
     vf_src1[fn2] =(src1scalar_s4[fn2]&ctrl4_vf_a_sel[fn2-FU_FALU]) ? {CBS}{NUMLANES}{CBS}vs_s4[fn2][0]{CBE}{CBE} : (banksel_s4[fn2]? vf_a_readdataout[1*{NUMLANES} +: {NUMLANES}] : vf_a_readdataout[0*{NUMLANES} +: {NUMLANES}]);

   //Only FALU uses this
     vf_src2[fn2] = (src2scalar_s4[fn2]) ? {CBS}{NUMLANES}{CBS}vs_s4[fn2][0]{CBE}{CBE} : (banksel_s4[fn2]? vf_b_readdataout[1*{NUMLANES} +: {NUMLANES}]:vf_b_readdataout[0*{NUMLANES} +: {NUMLANES}]);
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
  assign stall_memunit=_stall_memunit && (dst_we[FU_MEM][5] ||(ctrl4_memunit_en && ~stall_mulunit && ~stall_matmul));

//module instance 

  pipereg_1 memen5pipereg (
      .d( ctrl4_mem_en ),
      .clk(clk),
      .resetn(resetn),
      .en( pipe_advance[4] && ~_stall_memunit ),
      .squashn( ~pipe_squash[4] | _stall_memunit ),
      .q(ctrl5_mem_en ));

 //============== Memory Unit =============

//module instance 

  wire vmem_unit_out_dst_we;
  assign dst_we[FU_MEM][5] = vmem_unit_out_dst_we;

  vmem_unit_{LANEWIDTH}_{NUMLANES}_{LOG2NUMLANES}_{NUMMEMPARALLELLANES}_{LOG2NUMMEMPARALLELLANES}_{VCWIDTH}_{DMEM_WRITEWIDTH}_{LOG2DMEM_WRITEWIDTH}_{DMEM_READWIDTH}_{LOG2DMEM_READWIDTH}_{REGWIDTHMINUSVR}_{REGIDWIDTH} vmem_unit(
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
    .cstride(vstride[4]),
    .cprefetch(vs_s4[FU_MEM]),
    // Vector ports
    .vmask(vmask[FU_MEM]),
    .vstrideoffset(vstrideoffset_s4),
    .vindex(vr_src1[FU_MEM]),
    .vwritedata(vr_src2[FU_MEM]),
    .voutput(load_result_s5),
    .voutput_we(load_result_mask_s5),
    // Writeback ports
    .in_dst(dst_s4[FU_MEM]),
    .in_dst_we(dst_we_s4[FU_MEM]),
    .out_dst(dst[5*(NUMFUS*REGIDWIDTH)+ FU_MEM * REGIDWIDTH +: REGIDWIDTH]),
    .out_dst_we(vmem_unit_out_dst_we),
    .in_vs_dst_we(ctrl_vs_we[4]),
    .out_vs_dst_we(ctrl_vs_we[5]),
    // Vector operations ports
    .sa(vstride[4]),
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


  assign dst[4*(NUMFUS*REGIDWIDTH)+FU_MEM * REGIDWIDTH +: REGIDWIDTH] =dst_s4[FU_MEM];
  assign dst_we[FU_MEM][4]=dst_we_s4[FU_MEM];

  //============== Multiplier Unit (spans stages 4-6) =============

//module instance 

  wire [2:0] vmul_unit_out_dst_we;
  assign dst_we[FU_MUL][6:4] = vmul_unit_out_dst_we;

  wire [3*REGIDWIDTH-1:0] temp_vmul_dst;
  wire [3*{NUMLANES}-1:0] temp_vmul_mask;

   vmul_unit_{LOG2LANEWIDTH}_{NUMMULLANES}_{LOG2NUMLANES}_{REGIDWIDTH} vmul_unit(
     .clk(clk),
     .resetn(resetn),
     .op(ctrl4_mulshift_op),
     .activate(ctrl4_mulshift_en),
     .en(pipe_advance[6:4]),
     .squash(pipe_squash[6:4]),
     .stall(stall_mulunit),
     .opA(vr_src1[FU_MUL]),
     .opB(vr_src2[FU_MUL]),
     .vshamt( (ctrl4_rshiftnonzero) ? vc_s4[FU_MUL] : 0 ),
     .vmask(vmask[FU_MUL]),
     .in_dst(dst_s4[FU_MUL]),
     .in_dst_we(dst_we_s4[FU_MUL]),
     .out_dst(temp_vmul_dst),
     .out_dst_we(vmul_unit_out_dst_we),
     .out_dst_mask(temp_vmul_mask),
     .result(mulshift_result_s5)
   );
    assign dst[6*(NUMFUS*REGIDWIDTH) + FU_MUL*REGIDWIDTH +: REGIDWIDTH] = temp_vmul_dst[3*REGIDWIDTH-1:2*REGIDWIDTH] ;
    assign dst[5*(NUMFUS*REGIDWIDTH) + FU_MUL*REGIDWIDTH +: REGIDWIDTH] = temp_vmul_dst[2*REGIDWIDTH-1:1*REGIDWIDTH] ;
    assign dst[4*(NUMFUS*REGIDWIDTH) + FU_MUL*REGIDWIDTH +: REGIDWIDTH] = temp_vmul_dst[1*REGIDWIDTH-1:0*REGIDWIDTH] ;
    assign dst_mask[6*(NUMFUS*{NUMLANES})+FU_MUL*{NUMLANES} +: {NUMLANES}] = temp_vmul_mask[3*{NUMLANES}-1:2*{NUMLANES}];
    assign dst_mask[5*(NUMFUS*{NUMLANES})+FU_MUL*{NUMLANES} +: {NUMLANES}] = temp_vmul_mask[2*{NUMLANES}-1:1*{NUMLANES}];
    assign dst_mask[4*(NUMFUS*{NUMLANES})+FU_MUL*{NUMLANES} +: {NUMLANES}] = temp_vmul_mask[1*{NUMLANES}-1:0*{NUMLANES}];
  //============== ALU Unit =============

  //If APB value is true, create one ALU per bank (per lane)

'''
        string6_basic1='''
 
 //module instance 
     wire valu_kk_bk_mask;
     wire valu_kk_bk_cmp_result;
 
     assign valu_kk_bk_mask = vf_src1[FU_ALU+bk][kk+1:kk];
     assign alu_cmpresult_s4[bk][kk+1:kk] = valu_kk_bk_cmp_result;
 
     vlane_alu_{LANEWIDTH} valu_kk_bk(
         .clk(clk), 
         .resetn(resetn),
         .pipe_en(pipe_advance[4]),
         .pipe_squashn(~pipe_squash[4]),
         .src1(vr_src1[FU_ALU+bk][{LANEWIDTH}*kk +: {LANEWIDTH}]),
         .src2(vr_src2[FU_ALU+bk][{LANEWIDTH}*kk +: {LANEWIDTH}]),
         .mask(valu_kk_bk_mask),  //Note: MERGE is not masked
         .op(ctrl4_alu_op[bk]^ALUOP_ZERO),
         .satsum_op(ctrl4_satsum_op[bk]),
         .satsize_op(ctrl4_satsize_op[bk]),
         .cmp_result(valu_kk_bk_cmp_result),
         .result(alu_result_s5[bk][{LANEWIDTH}*kk +: {LANEWIDTH}])
         );
 //module instance 
 
     wire vflagalu_kk_bk_src1;
     wire vflagalu_kk_bk_src2;
     wire vflagalu_kk_bk_result;
 
     assign vflagalu_kk_bk_src1 = vf_src1[FU_FALU+bk][kk];
     assign vflagalu_kk_bk_src2 = vf_src2[FU_FALU+bk][kk];
     assign flagalu_result_s4[bk][kk] = vflagalu_kk_bk_result;
 
       vlane_flagalu vflagalu_kk_bk(
         .clk(clk), 
         .resetn(resetn),
         .src1(vflagalu_kk_bk_src1),
         .src2(vflagalu_kk_bk_src2),
         .op(ctrl4_flagalu_op[bk]),
         .result(vflagalu_kk_bk_result)
         );
 
   wire flagaluresultreg_kk_bk_q;
   assign flagalu_result_s5[bk][kk] = flagaluresultreg_kk_bk_q;
 //module instance 
       pipereg_1 flagaluresultreg_kk_bk (
         .d( (ctrl4_vf_wbsel[bk]) ? alu_cmpresult_s4[bk][kk] : flagalu_result_s4[bk][kk]),
         .clk(clk),
         .resetn(resetn),
         .en( pipe_advance[4] ), //Advance unless next stage stalled
         .squashn( 1'b1 ),
         .q(flagaluresultreg_kk_bk_q)
         );
'''
        string6 =""
        for k in range(numlanes):
            temp = string6_basic1.replace("kk",str(k))
            string6 += temp

        string6_basic2='''

 //module instance 
     wire [{REGIDWIDTH}*(1+1)-1:0] aludstpipe_bk_q;
     assign {CBS}dst[5*(NUMFUS*REGIDWIDTH)+(FU_ALU+bk)*REGIDWIDTH +: REGIDWIDTH],dst[4*(NUMFUS*REGIDWIDTH)+(FU_ALU+bk)*REGIDWIDTH +: REGIDWIDTH]{CBE} = aludstpipe_bk_q;
     pipe_{REGIDWIDTH}_1 aludstpipe_bk (
       .d( dst_s4[FU_ALU+bk] ),  
       .clk(clk),
       .resetn(resetn),
       .en( pipe_advance[4] ),
       .squash(squash_aludstpipe_NC),
       .q(aludstpipe_bk_q));
 
 //module instance 
 
   wire [1:0] aludstwepipe_bk_q;
   assign {CBS}dst_we[FU_ALU+bk][5],dst_we[FU_ALU+bk][4]{CBE} = aludstwepipe_bk_q;
 
    pipe_1_3 aludstwepipe_bk (
       .d( dst_we_s4[FU_ALU+bk] & ~ctrl4_vf_wbsel[bk] ),  
       .clk(clk),
       .resetn(resetn),
       .en( pipe_advance[6:4] ),
       .squash( pipe_squash[6:4] ),
       .q(aludstwepipe_bk_q));
 
 wire [{NUMLANES}*(1+1)-1:0] aludstmaskpipe_bk_q;
 assign {CBS}dst_mask[5*(NUMFUS*{NUMLANES})+(FU_ALU+bk)*{NUMLANES} +: {NUMLANES}],dst_mask[4*(NUMFUS*{NUMLANES})+(FU_ALU+bk)*{NUMLANES} +: {NUMLANES}]{CBE} = aludstmaskpipe_bk_q;
 
 //module instance 
     pipe_{NUMLANES}_1 aludstmaskpipe_bk (
       .d( vmask[FU_ALU+bk] ),  
       .clk(clk),
       .resetn(resetn),
       .en( pipe_advance[4] ),
       .squash(squash_aludstmaskpipe_NC),
       .q(aludstmaskpipe_bk_q));
 
     wire [{REGIDWIDTH}*(1+1)-1:0] faludstpipe_bk_q;
     assign {CBS}dst[5*(NUMFUS*REGIDWIDTH)+(FU_FALU+bk)*REGIDWIDTH +: REGIDWIDTH],dst[5*(NUMFUS*REGIDWIDTH)+(FU_FALU+bk)*REGIDWIDTH +: REGIDWIDTH]{CBE} = faludstpipe_bk_q;
 
 //module instance 
     pipe_{REGIDWIDTH}_1 faludstpipe_bk (
       .d( dst_s4[FU_FALU+bk] ),  
       .clk(clk),
       .resetn(resetn),
       .en( pipe_advance[4] ),
       .squash(squash_faludstpipe_NC),
       .q(faludstpipe_bk_q));
 
   wire [1:0] faludstwepipe_bk_q;
   assign {CBS}dst_we[FU_FALU+bk][5],dst_we[FU_FALU+bk][4]{CBE} = faludstwepipe_bk_q;
 //module instance 
     pipe_1_3 faludstwepipe_bk (
       .d( dst_we_s4[FU_FALU+bk] ),  
       .clk(clk),
       .resetn(resetn),
       .en( pipe_advance[6:4] ),
       .squash( pipe_squash[6:4] ),
       .q(faludstwepipe_bk_q));
 
'''
        string6_basic2 = string6 + string6_basic2
        string6 = "  wire squash_aludstpipe_NC;" + "\n" + "wire squash_faludstpipe_NC;" +"\n" + "wire squash_aludstmaskpipe_NC;" +"\n"

        for bk in range(1+(numbanks-1)*aluperbank):
            temp = string6_basic2.replace("bk",str(bk))
            string6 += temp

        string7='''

  //This code is just for assigning from signals connected
  //to the matmul, which are linear multi bit signals, to the
  //multi-dimensional signals.
  wire [`MATMUL_STAGES*REGIDWIDTH-1:0] dst_matmul;
  wire [`MATMUL_STAGES*{NUMLANES}-1:0] dst_mask_matmul;

'''
        string8_basic ='''

      assign dst[(gg+4)*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH] = dst_matmul[REGIDWIDTH*(gg+1)-1:REGIDWIDTH*gg];
      assign dst_mask[(gg+4)*(NUMFUS*{NUMLANES})+(FU_MATMUL)*{NUMLANES} +: {NUMLANES}] = dst_mask_matmul[{NUMLANES}*(gg+1)-1:{NUMLANES}*gg];

'''
        string8 = ""
        for g in range(0,29):
            string8 += string8_basic.replace("gg",str(g))

        string9 ='''

///////////////////////////
// Matmul unit
///////////////////////////

//module instance 

wire [`MAX_PIPE_STAGES-1:4] temp_matmul_we;
wire [`MAT_MUL_SIZE * `DWIDTH-1:0] matmul_in0,matmul_in1;

assign matmul_in0 = vr_src1[FU_MATMUL];
assign matmul_in1 = vr_src2[FU_MATMUL];

 matmul_unit_{REGIDWIDTH}_29_{NUMLANES} u_matmul(
 .clk(clk),
 .resetn(resetn),
 .activate(ctrl4_matmul_en),
 .en(pipe_advance[`MAX_PIPE_STAGES-1:4]),
 .squash(pipe_squash[`MAX_PIPE_STAGES-1:4]),
 .stall(stall_matmul),
 .a_data(matmul_in0),
 .b_data(matmul_in1),
 .validity_mask_a_rows(matmul_masks_in[1*`MAT_MUL_SIZE-1:0*`MAT_MUL_SIZE]),
 .validity_mask_a_cols(matmul_masks_in[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE]),
 .validity_mask_b_rows(matmul_masks_in[2*`MAT_MUL_SIZE-1:1*`MAT_MUL_SIZE]),
 .validity_mask_b_cols(matmul_masks_in[3*`MAT_MUL_SIZE-1:2*`MAT_MUL_SIZE]),
 .c_data(matmul_out), 
 .vmask(vmask[FU_MATMUL]),
 .in_dst(dst_s4[FU_MATMUL]),
 .in_dst_we(dst_we_s4[FU_MATMUL]),
 .out_dst(dst_matmul),
 .out_dst_we(temp_matmul_we),
 .out_dst_mask(dst_mask_matmul)
 );

assign dst_we[FU_MATMUL] = temp_matmul_we;

//module instance 

trp_unit_{LANEWIDTH} u_trp (
.clk(clk),
.resetn(resetn),
.en(ctrl4_trp_en),
.a(vr_src1[FU_TRP][{NUMLANES}*{LANEWIDTH}-1:0]),
.mode(),
.read(),
.busy(),
.valid(),
.out(trp_out)
);

'''
        string10_basic='''
 
//TODO:Generate statement 5
///////////////////////////
// Bfloat unit
///////////////////////////

//module instance 

    bfloat_adder_{LANEWIDTH} bf_add_g_func(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfadder_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFADDER][g_func * {LANEWIDTH} +: {LANEWIDTH}]),
    .b(vr_src2[FU_BFADDER][g_func * {LANEWIDTH} +: {LANEWIDTH}]),
    .out(bfadder_result_s5[g_func*{LANEWIDTH} +: {LANEWIDTH}])
    );
    
//module instance 
    bfloat_mult_{LANEWIDTH} bf_mult_g_func(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_bfmult_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_BFMULT][g_func * {LANEWIDTH} +: {LANEWIDTH}]),
    .b(vr_src2[FU_BFMULT][g_func * {LANEWIDTH} +: {LANEWIDTH}]),
    .out(bfmult_result_s5[g_func*{LANEWIDTH} +: {LANEWIDTH}])
    );
 
///////////////////////////
// activation unit
///////////////////////////

//module instance 
    activation_{LANEWIDTH} inst_activation_g_func(
    .clk(clk),
    .resetn(resetn),
    .en(ctrl4_act_en),
    .stall(stall_bf_adder),
    .a(vr_src1[FU_ACT][g_func * {LANEWIDTH} +: {LANEWIDTH}]),
    .out(act_result_s5[g_func*{LANEWIDTH} +: {LANEWIDTH}])
    );

'''
        string10 = ""
        for g_func in range(0,numlanes):
            string10 += string10_basic.replace("g_func",str(g_func))

        string11_basic='''

/******************************************************************************/
/************************** WB Pipeline Stage ********************************/
/******************************************************************************/

    assign wb_dst[FU_ALU+ba]=dst[5*(NUMFUS*REGIDWIDTH)+(FU_ALU+ba)*REGIDWIDTH +: REGIDWIDTH];
    assign wb_dst_we[FU_ALU+ba]=dst_we[FU_ALU+ba][5] && ~pipe_squash[5];
    assign wb_dst_mask[FU_ALU+ba]=dst_mask[5*(NUMFUS*{NUMLANES})+(FU_ALU+ba)*{NUMLANES} +: {NUMLANES}];
    assign D_wb_last_subvector[FU_ALU+ba]=D_last_subvector_s5[FU_ALU+ba];

    assign D_wb_last_subvector[FU_FALU+ba]=D_last_subvector_s5[FU_FALU+ba];

'''
        string11 = ""
        for ba in range(0,1+((numbanks-1)*aluperbank)):
            string11 = string11_basic.replace("ba",str(ba))
       
        string12='''

  assign wb_dst[FU_MEM]=dst[5*(NUMFUS*REGIDWIDTH)+(FU_MEM)*REGIDWIDTH +: REGIDWIDTH];
  assign wb_dst_we[FU_MEM]=dst_we[FU_MEM][5] && (~pipe_advance[5]|~pipe_squash[5]);
  assign wb_dst_mask[FU_MEM]=load_result_mask_s5;
  assign D_wb_last_subvector[FU_MEM]=D_last_subvector_s5[FU_MEM];

  assign wb_dst[FU_MUL]=dst[5*(NUMFUS*REGIDWIDTH)+(FU_MUL)*REGIDWIDTH +: REGIDWIDTH];
  assign wb_dst_we[FU_MUL]=dst_we[FU_MUL][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_MUL]=dst_mask[5*(NUMFUS*{NUMLANES})+(FU_MUL)*{NUMLANES} +: {NUMLANES}];
  assign D_wb_last_subvector[FU_MUL]=D_last_subvector_s5[FU_MUL];


  assign wb_dst[FU_BFADDER] = dst[5*(NUMFUS*REGIDWIDTH)+(FU_BFADDER)*REGIDWIDTH +: REGIDWIDTH];
  assign wb_dst_we[FU_BFADDER] = dst_we[FU_BFADDER][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_BFADDER] = dst_mask[5*(NUMFUS*{NUMLANES})+(FU_BFADDER)*{NUMLANES} +: {NUMLANES}];
  assign D_wb_last_subvector[FU_BFADDER] = D_last_subvector_s5[FU_BFADDER];

  assign wb_dst[FU_BFMULT] = dst[5*(NUMFUS*REGIDWIDTH)+(FU_BFMULT)*REGIDWIDTH +: REGIDWIDTH];
  assign wb_dst_we[FU_BFMULT] = dst_we[FU_BFMULT][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_BFMULT] = dst_mask[5*(NUMFUS*{NUMLANES})+(FU_BFMULT)*{NUMLANES} +: {NUMLANES}];
  assign D_wb_last_subvector[FU_BFMULT] = D_last_subvector_s5[FU_BFMULT];

  assign wb_dst[FU_ACT] = dst[5*(NUMFUS*REGIDWIDTH)+(FU_ACT)*REGIDWIDTH +: REGIDWIDTH];
  assign wb_dst_we[FU_ACT] = dst_we[FU_ACT][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_ACT] = dst_mask[5*(NUMFUS*{NUMLANES})+(FU_ACT)*{NUMLANES} +: {NUMLANES}];
  assign D_wb_last_subvector[FU_ACT] = D_last_subvector_s5[FU_ACT];

  assign wb_dst[FU_MATMUL]=dst[5*(NUMFUS*REGIDWIDTH)+(FU_MATMUL)*REGIDWIDTH +: REGIDWIDTH];
  assign wb_dst_we[FU_MATMUL]=dst_we[FU_MATMUL][5] && ~pipe_squash[5];
  assign wb_dst_mask[FU_MATMUL]=dst_mask[5*(NUMFUS*{NUMLANES})+(FU_MATMUL)*{NUMLANES} +: {NUMLANES}];
  //TODO: There is no code that assigns to the s31 var used below. Need to add that code
  //This is only a debug var, so it doesn't affect functionality
  assign D_wb_last_subvector[FU_MATMUL]=D_last_subvector_s31[FU_MATMUL];

  // ******************  Map functional units to banks ******************
  always@*
    for (bw=0; bw<{NUMBANKS}; bw=bw+1)
    begin
      vr_c_we[bw]=(wb_dst_we[FU_MUL]     && (wb_dst[FU_MUL]    &{CBS}{CBS}{REGIDWIDTH}-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE})==bw) ||
                  (wb_dst_we[FU_MATMUL]  && (wb_dst[FU_MATMUL] &{CBS}{CBS}{REGIDWIDTH}-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE})==bw) ||
                  (wb_dst_we[FU_BFADDER] && (wb_dst[FU_BFADDER]&{CBS}{CBS}{REGIDWIDTH}-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE})==bw) ||
                  (wb_dst_we[FU_BFMULT]  && (wb_dst[FU_BFMULT] &{CBS}{CBS}{REGIDWIDTH}-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE})==bw) ||
                  (wb_dst_we[FU_ACT]     && (wb_dst[FU_ACT]    &{CBS}{CBS}{REGIDWIDTH}-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE})==bw) ||
                  (wb_dst_we[FU_MEM]     && (wb_dst[FU_MEM]    &{CBS}{CBS}{REGIDWIDTH}-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE})==bw) ||
                  (wb_dst_we[FU_ALU]     && (wb_dst[FU_ALU]    &{CBS}{CBS}{REGIDWIDTH}-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE})==bw && {ALUPERBANK}==0) ||
                  (wb_dst_we[FU_ALU+bw]  && {ALUPERBANK}!=0);
      //TODO: Update this code for matmul. This is for debug only, so skipping //it for now.
      //Tells test_bench when to record register file contents.

      //Record if instruction writes to VRF, on last subvector, and not stalled
      D_wb_instrdone[bw] = pipe_advance[5] && (
        (({ALUPERBANK}==0) ?
          (dst_we[FU_ALU][5] && D_wb_last_subvector[FU_ALU] && ((wb_dst[FU_ALU] && {CBS}{CBS}{REGIDWIDTH}-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE})==bw)) :
          (dst_we[FU_ALU+bw][5] && D_wb_last_subvector[FU_ALU+bw]) || 
          (dst_we[FU_MUL][5] && D_wb_last_subvector[FU_MUL] && ((wb_dst[FU_MUL] && {CBS}{CBS}{REGIDWIDTH}-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE})==bw))|| 
          (dst_we[FU_MEM][5] && D_wb_last_subvector[FU_MEM] && ((wb_dst[FU_MEM] && {CBS}{CBS}{REGIDWIDTH}-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE})==bw))));
      //Take matmul output
      if (wb_dst_we[FU_MATMUL] && (wb_dst[FU_MATMUL] && ({CBS}{CBS}32-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE}))==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_MATMUL];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_MATMUL]>>{LOG2NUMBANKS};
        vr_c_reg[bw]= wb_dst[FU_MATMUL];
        vr_c_writedatain[bw]= matmul_out;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MATMUL];
      end      
      else if(wb_dst_we[FU_TRP] && (wb_dst[FU_TRP] && ({CBS}{CBS}32-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE}))==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_TRP];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_TRP]>>{LOG2NUMBANKS};
        vr_c_reg[bw]= wb_dst[FU_TRP];
//        vr_c_writedatain[bw]= trp_out;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_TRP];
      end
      //Take bfadder output
      else if (wb_dst_we[FU_BFADDER] && (wb_dst[FU_BFADDER] && ({CBS}{CBS}32-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE})) ==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_BFADDER];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_BFADDER]>>{LOG2NUMBANKS};
        vr_c_reg[bw]= wb_dst[FU_BFADDER];
//        vr_c_writedatain[bw]= bfadder_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_BFADDER];
      end
      else if (wb_dst_we[FU_BFMULT] && (wb_dst[FU_BFMULT] && ({CBS}{CBS}32-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE}))==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_BFMULT];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_BFMULT]>>{LOG2NUMBANKS};
        vr_c_reg[bw]= wb_dst[FU_BFMULT];
//        vr_c_writedatain[bw]= bfmult_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_BFMULT];
      end
      else if (wb_dst_we[FU_ACT] && (wb_dst[FU_ACT] &&({CBS}{CBS}32-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE}))==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_ACT];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_ACT]>>{LOG2NUMBANKS};
        vr_c_reg[bw]= wb_dst[FU_ACT];
//        vr_c_writedatain[bw]= act_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_ACT];
      end
      //Take multiplier output
      else if (wb_dst_we[FU_MUL] && (wb_dst[FU_MUL] && ({CBS}{CBS}32-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE}))==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_MUL];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_MUL]>>{LOG2NUMBANKS};
        vr_c_reg[bw]= wb_dst[FU_MUL];
        vr_c_writedatain[bw]= mulshift_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MUL];
      end
      //Take Memory unit output
      else if (wb_dst_we[FU_MEM] && (wb_dst[FU_MEM]&&({CBS}{CBS}32-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE}))==bw)
      begin
        vmask_final[bw]=wb_dst_mask[FU_MEM];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_MEM]>>{LOG2NUMBANKS};
        vr_c_reg[bw]= wb_dst[FU_MEM];
        vr_c_writedatain[bw]= load_result_s5;
        D_last_subvector_done[bw]=D_wb_last_subvector[FU_MEM];
      end
      else
      //Take ALU output
      begin
        vmask_final[bw]=wb_dst_mask[FU_ALU+bw*{ALUPERBANK}];
        _vr_c_reg[bw*BANKREGIDWIDTH +: BANKREGIDWIDTH]=wb_dst[FU_ALU+bw*{ALUPERBANK}]>>{LOG2NUMBANKS};
        vr_c_reg[bw]= wb_dst[FU_ALU+bw*{ALUPERBANK}];
        vr_c_writedatain[bw]= alu_result_s5[bw*{ALUPERBANK}];
        //Do ALU and FALU for last subvector
        D_last_subvector_done[bw]=
          (D_wb_last_subvector[FU_ALU+bw*{ALUPERBANK}] && (wb_dst[FU_ALU+bw*{ALUPERBANK}] & ({CBS}{CBS}32-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE})) ==bw) | 
          (D_wb_last_subvector[FU_FALU+bw*{ALUPERBANK}] && (dst[5*(NUMFUS * REGIDWIDTH) + ((FU_FALU+bw)*{ALUPERBANK}) * REGIDWIDTH +: REGIDWIDTH] &({CBS}{CBS}32-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE}))==bw);
      end


      //Generate byte enable from mask
      for (j=0; j<{NUMLANES}; j=j+1)
        vr_c_byteen[bw*{VPW}*{NUMLANES} + j*{VPW} +: {VPW} ]={CBS}{VPW}{CBS}vmask_final[bw][j]{CBE}{CBE};

      //*********** Flag writeback ***********
    //  vf_c_reg[bw*BANKREGIDWIDTH+:BANKREGIDWIDTH]=dst[5*(NUMFUS* REGIDWIDTH) + (FU_FALU+bw*{ALUPERBANK})*REGIDWIDTH +: REGIDWIDTH]>>{LOG2NUMBANKS};
      vf_c_writedatain[bw*{NUMLANES}+:{NUMLANES}]=flagalu_result_s5[bw*{ALUPERBANK}];
    //  vf_c_we[bw]= (dst_we[FU_FALU+bw*{ALUPERBANK}][5] && (dst[5*(NUMFUS*REGIDWIDTH)+ (FU_FALU+bw*{ALUPERBANK}) * REGIDWIDTH +: REGIDWIDTH] & ({CBS}{CBS}32-{LOG2NUMBANKS}{CBS}1'b0{CBE}{CBE},{CBS}{LOG2NUMBANKS}{CBS}1'b1{CBE}{CBE}{CBE}))==bw);
    end

  //********** Scalar writeback ***********
  assign vs_wetrack={CBS}ctrl_vs_we[5:4],|ctrl3_vs_we,ctrl2_vs_we{CBE};
  assign vs_we=ctrl_vs_we[5] & load_result_mask_s5[0];
  assign vs_dst=wb_dst[FU_MEM][REGIDWIDTH-1:REGIDWIDTH-VRIDWIDTH];
  assign vs_writedata=load_result_s5[{LANEWIDTH}-1:0];

endmodule

'''
        string = string_dispatchwidth + string1 + string2 + string3 + string4 + string5 + string6 + \
                  string7 + string8 + string9 + string10 + string11 + string12
        
        #string = string_dispatchwidth + string1 + string2 + string3 +string4 + string5 + "\n" + "endmodule"
        output_string = self.process_case_stmt(string) 
        fp = open("verilog/vdispatcher.v",'a')
        uut = vdispatcher(fp)
        uut.write(numbanks,dispatcherwidth,log2mvl,log2mvl,log2numlanesp1)
        fp.close()
        fp = open("verilog/vregfile_vector.v",'a')
        uut = vregfile_vector(fp)
        uut.write(numbanks,log2numbanks,totalvpw,totalregs,regidwidth)
        fp.close()
        fp = open("verilog/vregfile_flag.v",'a')
        uut = vregfile_flag(fp)
        uut.write(numbanks,log2numbanks,numlanes,totalregs,regidwidth)
        fp.close()
        fp = open("verilog/vmem_unit.v",'a')
        uut = vmem_unit(fp)
        uut.write(lanewidth,numlanes,log2numlanes,nummemparallellanes,log2nummemparallellanes,vcwidth,dmem_writewidth,log2dmem_writewidth,dmem_readwidth,log2dmem_readwidth,regwidthminusvr,regidwidth)
        fp.close()
        fp = open("verilog/vmul_unit.v",'a')
        uut = vmul_unit(fp)
        uut.write(log2lanewidth,nummullanes,log2numlanes,regidwidth)
        fp.close()
        #fp = open("verilog/vmul_unit.v",'a')
        #uut = vmul_unit(fp)
        #uut.write(log2lanewidth,nummullanes,log2numlanes,regidwidth)
        #fp.close()
        fp = open("verilog/vlane_alu.v",'a')
        uut = vlane_alu(fp)
        uut.write(lanewidth)
        fp.close()
        fp = open("verilog/vlane_flagalu.v",'a')
        uut = vlane_flagalu(fp)
        uut.write()
        fp.close()
        fp = open("verilog/matmul_unit.v",'a')
        uut = matmul_unit(fp)
        uut.write(regidwidth,29,numlanes)
        fp.close()
        fp = open("verilog/trp_unit.v",'a')
        uut = trp_unit(fp)
        uut.write(lanewidth)
        fp.close()
        fp = open("verilog/bfloat_adder.v",'a')
        uut = bfloat_adder(fp)
        uut.write(lanewidth)
        fp.close()
        fp = open("verilog/bfloat_mult.v",'a')
        uut = bfloat_mult(fp)
        uut.write(lanewidth)
        fp.close()
        fp = open("verilog/activation.v",'a')
        uut = activation(fp)
        uut.write(lanewidth)
        fp.close()
        fp = open("verilog/pipereg.v",'a')
        uut = pipereg(fp)
        uut.write(17)
        uut.write(1)
        uut.write(8)
        uut.write(pipe1regwidth)
        fp.close()
        fp = open("verilog/pipe.v",'a')
        uut = pipe(fp)
        uut.write(7,5)
        uut.write(1,3)
        uut.write(8,1)
        uut.write(vcwidth,4)
        uut.write(regidwidth,1)
        uut.write(numlanes, 1)
        fp.close()
        fp = open("verilog/onecyclestall.v",'a')
        uut = onecyclestall(fp)
        uut.write()
        fp.close()
        fp = open("verilog/hazardchecker.v",'a')
        uut = hazardchecker(fp)
        uut.write(regidwidth,velmidwidth,totalflagalus,numbanks)
        uut.write(regidwidth,velmidwidth,totalalus,numbanks)
        fp.close()

        return output_string.format( NUMLANES = numlanes, \
                              LOG2NUMLANES = log2numlanes, \
                              MVL = mvl, \
                              LOG2MVL = log2mvl, \
                              VPW = vpw, \
                              LOG2VPW = log2vpw, \
                              LANEWIDTH  = lanewidth, \
                              LOG2LANEWIDTH = log2lanewidth, \
                              NUMBANKS = numbanks, \
                              LOG2NUMBANKS = log2numbanks, \
                              ALUPERBANK = aluperbank, \
                              NUMMEMPARALLELLANES = nummemparallellanes, \
                              LOG2NUMMEMPARALLELLANES = log2nummemparallellanes, \
                              NUMMULLANES = nummullanes, \
                              LOG2NUMMULLANES = log2nummullanes, \
                              DMEM_WRITEWIDTH = dmem_writewidth, \
                              LOG2DMEM_WRITEWIDTH = log2dmem_writewidth, \
                              DMEM_READWIDTH = dmem_readwidth, \
                              LOG2DMEM_READWIDTH = log2dmem_readwidth, \
                              VCWIDTH = vcwidth, \
                              VSWIDTH = vswidth, \
                              NUMVSREGS  = numvsregs, \
                              LOG2NUMVSREGS = log2numvsregs, \
                              VRIDWIDTH = vridwidth, \
                              PIPE1REGWIDTH = pipe1regwidth ,\
                              DISPATCHERWIDTH = dispatcherwidth ,\
                              LOG2NUMLANESP1 = log2numlanesp1 ,\
                              TOTALALUS = totalalus ,\
                              TOTALFLAGALUS = totalflagalus ,\
                              REGIDWIDTH = regidwidth ,\
                              VELMIDWIDTH = velmidwidth ,\
                              TOTALVPW = totalvpw ,\
                              TOTALREGS = totalregs ,\
                              REGWIDTHMINUSVR = regwidthminusvr ,\
                              CBS = "{" , \
                              CBE = "}" \
                            )

    def write(self, numlanes, log2numlanes, mvl, log2mvl, vpw, log2vpw, numbanks, log2numbanks, aluperbank, nummemparallellanes, log2nummemparallellanes, dmem_writewidth,log2dmem_writewidth, dmem_readwidth, log2dmem_readwidth, vcwidth, vswidth, numvsregs, log2numvsregs, vridwidth):
        self.fp.write(self.make_str(numlanes, log2numlanes, mvl, log2mvl, vpw, log2vpw, numbanks, log2numbanks, aluperbank, nummemparallellanes, log2nummemparallellanes, dmem_writewidth,log2dmem_writewidth, dmem_readwidth, log2dmem_readwidth, vcwidth, vswidth, numvsregs, log2numvsregs, vridwidth))



if __name__ == '__main__':
    numlanes = 8 
    log2numlanes = 2
    mvl = 64
    log2mvl = 6
    vpw = 4
    log2vpw = 2
    numbanks = 2
    log2numbanks = 1
    aluperbank = 1
    nummemparallellanes = 2
    log2nummemparallellanes = 1
    dmem_writewidth = 128
    log2dmem_writewidth = 7
    dmem_readwidth = 128
    log2dmem_readwidth = 7
    vcwidth = 32
    vswidth = 32
    numvsregs = 32
    log2numvsregs = 5
    vridwidth = 5

    fp = open(args[0], "w")
    uut1 = vlanes(fp)
    uut1.write(numlanes, log2numlanes, mvl, log2mvl, vpw, log2vpw, numbanks, log2numbanks, aluperbank, nummemparallellanes, log2nummemparallellanes, dmem_writewidth,log2dmem_writewidth, dmem_readwidth, log2dmem_readwidth, vcwidth, vswidth, numvsregs, log2numvsregs, vridwidth)
    fp.close()
