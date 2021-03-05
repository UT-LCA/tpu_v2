import re
import os

TOT="/export/aman/tpu_v2/"

#dictionary of bit_vector instances.
#key - file+array identifier
#value - array object
dict_of_bit_vector = {}

class bit_vector:
    def __init__(self, name, \
                 bit_vector_dim_max, \
                 bit_vector_dim_min, \
                 packed_dim_len, \
                 unpacked_dim_len, \
                 packed_dim_min, \
                 unpacked_dim_min,\
                 packed_dim_max,\
                 unpacked_dim_max):
        self.name = name
        self.bit_vector_dim_max = bit_vector_dim_max
        self.bit_vector_dim_min = bit_vector_dim_min
        self.packed_dim_len = packed_dim_len
        self.unpacked_dim_len = unpacked_dim_len
        self.packed_dim_min = packed_dim_min
        self.unpacked_dim_min = unpacked_dim_min
        self.packed_dim_max = packed_dim_max
        self.unpacked_dim_max = unpacked_dim_max

    
    #[unpacked_ref_max : unpacked_ref_min] [packed_ref_max : packed_ref_min]
    def gen_bitvector_based_ref(self, unpacked_ref_max, unpacked_ref_min, packed_ref_max, packed_ref_min):
        # vc[2] -> vc[None:2][None:None]
        if unpacked_ref_max is None and unpacked_ref_min is not None and packed_ref_max is None and packed_ref_min is None:
            unpacked_ref_max = unpacked_ref_min
            packed_ref_max = self.packed_dim_max
            packed_ref_min = self.packed_dim_min
        # vc[2:1] -> vc[2:1][None:None]
        if unpacked_ref_max is not None and unpacked_ref_min is not None and packed_ref_max is None and packed_ref_min is None:
            packed_ref_max = self.packed_dim_max
            packed_ref_min = self.packed_dim_min
        # vc[2][2] -> vc[None:2][None:2]
        if unpacked_ref_max is None and unpacked_ref_min is not None and packed_ref_max is None and packed_ref_min is not None:
            unpacked_ref_max = unpacked_ref_min
            packed_ref_max = packed_ref_min
        #vc[2][VCWIDTH-1:0] -> vc[None:2][VCWIDTH-1:0]
        if unpacked_ref_max is None and unpacked_ref_min is not None and packed_ref_max is not None and packed_ref_min is not None:
            unpacked_ref_max = unpacked_ref_min
        #vc[2:1][VCWIDTH-1:0] -> vc[2:1][VCWIDTH-1:0]
        if unpacked_ref_max is not None and unpacked_ref_min is not None and packed_ref_max is not None and packed_ref_min is not None:
            pass

        #Now find the new dims
        #min
        bit_vector_ref_min_dim_1 = "("+unpacked_ref_min+"-"+self.unpacked_dim_min+")*"+self.packed_dim_len
        bit_vector_ref_max_dim_1 = bit_vector_ref_min_dim_1+"+"+packed_ref_max+"-"+packed_ref_min
        bit_vector_ref_min_dim_2 = "("+unpacked_ref_max+"-"+self.unpacked_dim_min+")*"+self.packed_dim_len
        bit_vector_ref_max_dim_2 = bit_vector_ref_min_dim_2+"+"+packed_ref_max+"-"+packed_ref_min

        #bit_vector_ref_max_dim_2:bit_vector_ref_min_dim_1
        return (bit_vector_ref_max_dim_2,bit_vector_ref_min_dim_1)


def print_contents(infile, outfile, possible_dirs):
    outfile.write("//////////////////////////////////////////////////////////////////////\n")
    outfile.write("//// Starting contents of file: {}\n".format(infile.name))
    outfile.write("//////////////////////////////////////////////////////////////////////\n")
    capture_case_item_code = False
    case_stmt_start = False
    case_item_list = []
    captured_case_items = []
    line_num = 0
    for line in infile:
        line_num = line_num+1
        
        include_line = re.search(r'`include "(.*)"', line)
        #TESTING include_line = None

        #`timescale`
        timescale_line = re.search(r'`timescale', line)

        #`define SIMULATION_MEMORY`
        sim_mem_def_line = re.match(r'`define SIMULATION_MEMORY', line)

        #declaration of an array
        #eg:wire  [ VCWIDTH-1 : 0 ]   vc[`MAX_PIPE_STAGES-1:2];
        array_decl_line = re.search(r'((wire)|(reg))\s*\[(.*):(.*)\]\s+(.*)\[(.*):(.*)\];', line)

        #declaration of a bitvector
        #reg       [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vf_a_reg;
        bitvector_decl_line = re.search(r'((wire)|(reg))\s*\[(.*):(.*)\]\s+([\w]*);', line)

        #lines using an array
        #arr[]
        array_line_type1 = re.findall(r'[\w]+\s*\[\w*?\]', line)
        #arr[][]
        array_line_type2 = re.findall(r'[\w]+\s*\[\w*?\]\s*\[\w*?\]', line)
        #TODO: What if we have a line that has both arr[] and arr[][].
        #Example: vs[2] & vc[2][VCWIDTH-1]

        #comments
        comment_line = re.search(r'^\s*//', line)

        #io's
        io_line = re.search(r'input|output', line)

        #only handle case statements in vlanes.v file
        c_stmt_begin = None
        c_stmt_end = None
        if "vlanes.v" in infile.name:
            c_stmt_begin = re.search(r'((\bcase\b)|(\bcasez\b))\(.*\)', line)
            c_stmt_end = re.search(r'endcase', line)

        if comment_line is not None:
            outfile.write(line)
            continue
        elif io_line is not None:
            outfile.write(line)
            continue
        elif bitvector_decl_line is not None and array_decl_line is None:
            outfile.write(line)
            continue
        elif array_decl_line is not None and bitvector_decl_line is None:
            packed_dim_max = array_decl_line.group(4).strip()
            packed_dim_min = array_decl_line.group(5).strip()
            unpacked_dim_max = array_decl_line.group(7).strip()
            unpacked_dim_min = array_decl_line.group(8).strip()
            array_identifier = array_decl_line.group(6).strip()
            #assert(eval(packed_dim_min)==0), "The line is {}".format(line)
            unpacked_dim_len = "(("+unpacked_dim_max+")-("+unpacked_dim_min+")+1)"
            packed_dim_len = "(("+packed_dim_max+")-("+packed_dim_min+")+1)"
            bit_vector_len = "("+unpacked_dim_len+"*"+packed_dim_len+")"
            #bit_vector_dim_min = packed_dim_min
            #bit_vector_dim_max = "(("+packed_dim_len+")*("+unpacked_dim_len+"))-1"
            #bit_vector_decl = "["+bit_vector_dim_max+" : "+bit_vector_dim_min+"] "+array_identifier+";\n"
            bit_vector_dim_min = "0"
            bit_vector_dim_max = bit_vector_len+"-1"
            bit_vector_decl = "["+bit_vector_dim_max+" : "+bit_vector_dim_min+"] "+array_identifier+";\n"
            outfile.write(bit_vector_decl)

            bvt_obj = bit_vector(array_identifier,\
                                 bit_vector_dim_max,\
                                 bit_vector_dim_min,\
                                 packed_dim_len,\
                                 unpacked_dim_len,\
                                 packed_dim_min,\
                                 unpacked_dim_min,\
                                 packed_dim_max,\
                                 unpacked_dim_max)
            dict_of_bit_vector[infile.name + ":" + array_identifier] = bvt_obj

        elif len(array_line_type2) != 0:
            for array_usage in array_line_type2:
                m = re.search(r'([\w\d]+)\s*\[(.*?)\]\s*\[(.*?)\]', array_usage)
                assert (m is not None)
                name = m.group(1)
                key = infile.name + ":" + name
                if key not in dict_of_bit_vector:
                    #print("Found usage of x[], but this must be a bitvector")
                    pass
                else:
                    #print("Found usage of x[], that is an array. Replacing it with bitvector")
                    #vc[2][3] -> vc[None:2][None:3]
                    if ":" not in m.group(2) and ":" not in m.group(3):
                        unpacked_ref_max = None
                        unpacked_ref_min = m.group(2)
                        packed_ref_max = None
                        packed_ref_min = m.group(3)
                    #vc[2][VCWIDTH-1:0] -> vc[None:2][VCWIDTH-1:0]
                    elif ":" not in m.group(2) and ":" in m.group(3):
                        vals = m.group(3).split(":")
                        assert(len(vals) == 2), "Line number is {}. Line is {}".format(line_num, line)
                        unpacked_ref_max = None
                        unpacked_ref_min = m.group(2)
                        packed_ref_max = vals[0]
                        packed_ref_min = vals[1]
                    #vc[2:1][VCWIDTH-1:0] -> vc[2:1][VCWIDTH-1:0]
                    elif ":" in m.group(2) and ":" in m.group(3):
                        vals = m.group(2).split(":")
                        assert(len(vals) == 2)
                        unpacked_ref_max = vals[0]
                        unpacked_ref_min = vals[1]
                        vals = m.group(3).split(":")
                        assert(len(vals) == 2)
                        packed_ref_max = vals[0]
                        packed_ref_min = vals[1]
                    else:
                        assert(0), "Shouldn't have reached here. Line number is {}. Line is {}. Array usage is {}".format(line_num, line, array_usage)

                    (max, min) = dict_of_bit_vector[key].gen_bitvector_based_ref(unpacked_ref_max, unpacked_ref_min, packed_ref_max, packed_ref_min)
                    bitvector_usage = name + "[(" + max + ") : (" + min + ")]"
                    line = re.sub(re.escape(array_usage), bitvector_usage, line)
            outfile.write(line)

        elif len(array_line_type1) != 0:
            for array_usage in array_line_type1:
                m = re.search(r'([\w\d]+)\s*\[(.*?)\]', array_usage)
                assert (m is not None)
                name = m.group(1)
                key = infile.name + ":" + name
                if key not in dict_of_bit_vector:
                    #print("Found usage of x[], but this must be a bitvector")
                    pass
                else:
                    #print("Found usage of x[], that is an array. Replacing it with bitvector")
                    # vc[2] -> vc[None:2][None:None]
                    # vc[2:1] -> vc[2:1][None:None]
                    if ":" in m.group(2):
                        vals = m.group(2).split(":")
                        assert(len(vals) == 2)
                        unpacked_ref_max = vals[0]
                        unpacked_ref_min = vals[1]
                    else: 
                        unpacked_ref_max = None
                        unpacked_ref_min = m.group(2)
                    packed_ref_max = None
                    packed_ref_min = None
                    (max, min) = dict_of_bit_vector[key].gen_bitvector_based_ref(unpacked_ref_max, unpacked_ref_min, packed_ref_max, packed_ref_min)
                    bitvector_usage = name + "[(" + max + ") : (" + min + ")]"
                    line = re.sub(re.escape(array_usage), bitvector_usage, line)
            outfile.write(line)

        elif c_stmt_begin is not None:
            case_stmt_start = True
            outfile.write(re.sub(r'casez', 'case', line))
        elif c_stmt_end is not None:
            case_stmt_start = False
            outfile.write(line) 
        elif case_stmt_start is True:
            case_item_multi = re.match(r'\s+([A-Z\d_]*),\s*$', line)
            if case_item_multi is not None:
                case_item_list.append(case_item_multi.group(1))
                continue
            case_item_single = re.match(r'\s+([A-Z\d_]*):\s*$', line)
            if case_item_single is not None:
                case_item_list.append(case_item_single.group(1))
                continue
            if len(case_item_list) > 0:
                begin_in_case = re.search(r'\s+begin', line)
                if begin_in_case is not None:
                    captured_case_items.append(line)
                    capture_case_item_code = True
                    continue
                end_in_case = re.search(r'\s+end', line)
                if end_in_case is not None:
                    captured_case_items.append(line)
                    capture_case_item_code = False
                    #start printing each case statement one by one
                    for case_item in case_item_list:
                        outfile.write("    "+case_item+":\n")
                        outfile.write(''.join(captured_case_items))
                    case_item_list = []
                    captured_case_items = []
                    continue
                if capture_case_item_code is True:
                    captured_case_items.append(line)
                    continue
            outfile.write(line)
        elif include_line is not None:
            included_filename = include_line.group(1)
            outfile.write("//////////////////////////////////////////////////////////////////////\n")
            outfile.write("//// Starting contents of included file: {}\n".format(included_filename))
            outfile.write("//////////////////////////////////////////////////////////////////////\n")
            print(included_filename)
            #check for file
            found_dir = ""
            for dir in possible_dirs:
                if (os.path.exists(TOT+dir+included_filename)):
                    found_dir = dir  
                    break
            if found_dir == "":
                print("File {} not found in possible directories".format(included_filename))
                raise SystemExit(0)
            else:
                print("File {} found in directory {}".format(included_filename, found_dir))
            included_f = open(TOT+found_dir+included_filename, "r")
            print_contents(included_f, outfile, possible_dirs)
            outfile.write("//////////////////////////////////////////////////////////////////////\n")
            outfile.write("//// Finish contents of included file: {}\n".format(included_filename))
            outfile.write("//////////////////////////////////////////////////////////////////////\n")
        elif timescale_line is not None or sim_mem_def_line is not None:
            continue
        else:
            outfile.write(line) 
 
##Scalar processor
#scalar_all_fname = "scalar_all.v"
#scalar_all_fhandle = open(scalar_all_fname, "w")
#
#scalar_dir = TOT+"tpu/design/scalar/"
#scalar_main_fname = scalar_dir+"system.v"
#scalar_main_fhandle = open(scalar_main_fname, "r")
#
#possible_dirs = ["tpu/design/scalar/", \
#                 "tpu/design/top/"]
#
#scalar_all_fhandle.write("`define USE_INHOUSE_LOGIC\n")
#print_contents(scalar_main_fhandle, scalar_all_fhandle, possible_dirs)

#Vector processor
vector_all_fname = "vector_all.v"
vector_all_fhandle = open(vector_all_fname, "w")

vector_dir = TOT+"tpu/design/vector/"
vector_main_fname = vector_dir+"vpu.v"
#TESTING vector_main_fname = vector_dir+"vlanes.v"
vector_main_fhandle = open(vector_main_fname, "r")

possible_dirs = ["tpu/design/vector/", \
                 "tpu/design/top/"]

vector_all_fhandle.write("`define USE_INHOUSE_LOGIC\n")
print_contents(vector_main_fhandle, vector_all_fhandle, possible_dirs)

#Adding contents of local modules
local_file_list = ["tpu/design/local/spram1.v", \
                   "tpu/design/local/rams.v", \
                   "tpu/design/local/local_add_sub.v", \
                   "tpu/design/local/local_mult.v", \
                   "tpu/design/local/local_fifo.v", \
                   "tpu/design/local/local_shifter.v"]

for local_fname in local_file_list:
    local_fhandle = open(TOT+local_fname, "r")
    print_contents(local_fhandle, vector_all_fhandle, possible_dirs)
    #print_contents(local_fhandle, scalar_all_fhandle, possible_dirs)

#Adding contents of components (from scalar into vector)
components_file_list = ["tpu/design/scalar/components.v"]

for components_fname in components_file_list:
    components_fhandle = open(TOT+components_fname, "r")
    print_contents(components_fhandle, vector_all_fhandle, possible_dirs)

#Commenting this for now. Somehow multiple lines get deleted if this is done from inside the script
#os.system("dos2unix {}".format(vector_all_fname))
#os.system("dos2unix {}".format(scalar_all_fname))
