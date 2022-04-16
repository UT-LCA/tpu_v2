import re
import os
import argparse

##################################################
# Globals
##################################################
#dictionary of bit_vector/array instances.
#key - file+array identifier
#value - array object
dict_of_bit_vector = {}

##################################################
# Class representing bit vectors or arrays
# We will create objects of this class as we find
# array declarations. When we find an array reference
# in the code, we will call a method to return
# the resulting bitvector's dimensions (msb:lsb)
##################################################
class bit_vector:
    #A declaration of an array is like:
    #reg [A:B] arr[C:D];
    #A:B is the packed dimension
    #C:D is the unpacked dimension
    #Assumption: There are no multidimensional arrays in the code
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
        #dimensions of the resulting bitvector
        self.bit_vector_dim_max = bit_vector_dim_max
        self.bit_vector_dim_min = bit_vector_dim_min
        #length of the packed and unpacked dimensions of the original array
        self.packed_dim_len = packed_dim_len
        self.unpacked_dim_len = unpacked_dim_len
        #LSB of the packed and unpacked dimensions of the original array
        self.packed_dim_min = packed_dim_min
        self.unpacked_dim_min = unpacked_dim_min
        #MSB of the packed and unpacked dimensions of the original array
        self.packed_dim_max = packed_dim_max
        self.unpacked_dim_max = unpacked_dim_max

    #A reference can be of many types. The generic form is:
    #arr [unpacked_ref_max : unpacked_ref_min] [packed_ref_max : packed_ref_min]
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
        bit_vector_ref_min_dim_1 = "("+unpacked_ref_min+"-"+self.unpacked_dim_min+")*"+self.packed_dim_len
        bit_vector_ref_max_dim_1 = bit_vector_ref_min_dim_1+"+"+packed_ref_max+"-"+packed_ref_min
        bit_vector_ref_min_dim_2 = "("+unpacked_ref_max+"-"+self.unpacked_dim_min+")*"+self.packed_dim_len
        bit_vector_ref_max_dim_2 = bit_vector_ref_min_dim_2+"+"+packed_ref_max+"-"+packed_ref_min

        #bit_vector_ref_max_dim_2:bit_vector_ref_min_dim_1
        return (bit_vector_ref_max_dim_2,bit_vector_ref_min_dim_1)


##################################################
# Process the contents of infile and then print them 
# into outfile. This function is recursive. Whenever 
# we find another file included in a file (`include)
# this function is called again.
##################################################
def create_one_file(TOT, infile, outfile, possible_dirs):
    outfile.write("//////////////////////////////////////////////////////////////////////\n")
    outfile.write("//// Starting contents of file: {}\n".format(infile.name))
    outfile.write("//////////////////////////////////////////////////////////////////////\n")

    #Go over each line
    for line in infile:
        
        include_line = re.search(r'`include\s*"(.*)"', line)
        #TESTING include_line = None

        #`timescale`
        timescale_line = re.search(r'`timescale', line)

        #`define SIMULATION_MEMORY`
        sim_mem_def_line = re.match(r'`define SIMULATION_MEMORY', line)
   
        #for removing don't cares
        if (('0z' in line) or ('z0' in line) or ('z1' in line) or ('1z' in line)):
            #import pdb;pdb.set_trace()
            line= line.replace('z','0')

        #A `include line. We just call this function again
        if include_line is not None:
            included_filename = include_line.group(1)

            # Special handling of vlanes.v
            if included_filename == "vlanes.v":
                included_filename = "vlanes.v.temp"

            outfile.write("//////////////////////////////////////////////////////////////////////\n")
            outfile.write("//// Starting contents of included file: {}\n".format(included_filename))
            outfile.write("//////////////////////////////////////////////////////////////////////\n")
            print(included_filename)

            #check for file being present in the list of possible directories
            found_dir = ""
            for dir in possible_dirs:
                if (os.path.exists(TOT+dir+included_filename)):
                    found_dir = dir  
                    break
            if found_dir == "":
                print("File {} not found in possible directories {}".format(included_filename, str(possible_dirs)))
                raise SystemExit(0)
            else:
                print("File {} found in directory {}".format(included_filename, found_dir))

            #Recursive call
            included_f = open(TOT+found_dir+included_filename, "r")
            create_one_file(TOT,included_f, outfile, possible_dirs)
            outfile.write("//////////////////////////////////////////////////////////////////////\n")
            outfile.write("//// Finish contents of included file: {}\n".format(included_filename))
            outfile.write("//////////////////////////////////////////////////////////////////////\n")

        #For these lines, we don't want to print them in output file
        elif timescale_line is not None or sim_mem_def_line is not None:
            continue
            
        #If we don't meet any of the criteria above, just write it to outfile
        else:
            outfile.write(line) 


##################################################
# Process the case statements in infile 
##################################################
def process_case_stmt(infile, outfile):
    capture_case_item_code = False
    case_stmt_start = False
    case_item_list = []
    captured_case_items = []
    line_num = 0

    #Go over each line
    for line in infile:
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
            outfile.write(re.sub(r'casez', 'case', line))
        elif c_stmt_end is not None:
            case_stmt_start = False
            outfile.write(line) 
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
                        outfile.write("    "+case_item+":\n")
                        outfile.write(''.join(captured_case_items))
                    case_item_list = []
                    captured_case_items = []
                    continue
                if capture_case_item_code is True:
                    captured_case_items.append(line)
                    continue
            outfile.write(line)
        
        #If we don't meet any of the criteria above, just write it to outfile
        else:
            outfile.write(line) 



##################################################
# Process arrays and change them to bitvectors
##################################################
def process_arrays(infile, outfile):
    line_num = 0

    #Go over each line
    for line in infile:
        line_num = line_num+1
        
        ############################################### 
        # First, we have regex to identify type of line
        ############################################### 

        #declaration of an array
        #eg:wire  [ VCWIDTH-1 : 0 ]   vc[`MAX_PIPE_STAGES-1:2];
        array_decl_line = re.search(r'((wire)|(reg))\s*\[(.*):(.*)\]\s+(.*)\[(.*):(.*)\];', line)

        #declaration of a bitvector
        #reg       [ NUMBANKS*BANKREGIDWIDTH-1 : 0 ]   vf_a_reg;
        bitvector_decl_line = re.search(r'((wire)|(reg))\s*\[(.*):(.*)\]\s+([\w]*);', line)

        #lines using an array
        #arr[]
        array_line_type1 = re.findall(r'[\w]+\s*\[[\w\-:\* ]*?\]', line)
        #arr[][]
        array_line_type2 = re.findall(r'[\w]+\s*\[[\w\-:\* ]*?\]\s*\[[\w\-:\* ]*?\]', line)
        #TODO: What if we have a line that has both arr[] and arr[][].
        #Example: vs[2] & vc[2][VCWIDTH-1]

        #arr[][ something +: something]
        array_line_type3 = re.findall(r'[\w]+\s*\[\s*[\w\-:\* ]*?\s*\]\s*\[\s*[\w\-\* ]*?\s*\+:\s*[\w\-\* ]*?\s*\]', line)

        #comments
        comment_line = re.search(r'^\s*//', line)

        #io's
        io_line = re.search(r'input|output', line)

        ############################################### 
        # Now we start processing lines based on what we found in them
        ############################################### 

        # Lines with comments are directly copied to outfile
        if comment_line is not None:
            outfile.write(line)
            continue

        # Lines with input/output statements are directly copied to outfile
        elif io_line is not None:
            outfile.write(line)
            continue

        # Lines with bitvector declarations are directly copied to outfile
        elif bitvector_decl_line is not None and array_decl_line is None:
            outfile.write(line)
            continue

        # Lines with array declarations. We'll create array/bitvector
        # objects and store them in global dict
        elif array_decl_line is not None and bitvector_decl_line is None:
            packed_dim_max = array_decl_line.group(4).strip()
            packed_dim_min = array_decl_line.group(5).strip()
            unpacked_dim_max = array_decl_line.group(7).strip()
            unpacked_dim_min = array_decl_line.group(8).strip()
            array_identifier = array_decl_line.group(6).strip()
            #assert(eval(packed_dim_min)==0), "The line is {}".format(line)

            # Find lengths of each dimension. Note that these are not
            # integers, but strings.
            unpacked_dim_len = "(("+unpacked_dim_max+")-("+unpacked_dim_min+")+1)"
            packed_dim_len = "(("+packed_dim_max+")-("+packed_dim_min+")+1)"

            # Length of the resulting bitvector is the product of the 
            # length of unpacked and packed dimensions
            bit_vector_len = "("+unpacked_dim_len+"*"+packed_dim_len+")"

            # Create declaration of new bitvector
            bit_vector_dim_min = "0"
            bit_vector_dim_max = bit_vector_len+"-1"
            bit_vector_decl = array_decl_line.group(1)+" ["+bit_vector_dim_max+" : "+bit_vector_dim_min+"] "+array_identifier+";\n"
            outfile.write(bit_vector_decl)

            # Create array/bitvector object and store in dict
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

        # Lines with array references of type 3 (arr[][something +: something])
        elif len(array_line_type3) != 0:
            for array_usage in array_line_type3:
                m = re.search(r'([\w]+)\s*\[(\s*[\w\-:\* ]*?)\s*\]\s*\[(\s*[\w\-\* ]*?\s*\+:\s*[\w\-\* ]*?)\s*\]', array_usage)
                #If we are here, this search should return something because we just used the same regex
                assert (m is not None)

                name = m.group(1)

                #If this array doesn't exist in the dict, that means it's not really an array
                #could be a bitvector
                key = infile.name + ":" + name
                if key not in dict_of_bit_vector:
                    #print("Found usage of x[], but this must be a bitvector")
                    pass
                else:
                    #Find the msb and lsb of each dimension of the array
                    #print("Found usage of x[], that is an array. Replacing it with bitvector")
                    #alu_dst[4][bd*REGIDWIDTH +: REGIDWIDTH] -> alu_dst[None:4][None:bd*REGWIDTH]
                    if "+:" in m.group(3) and "+:" not in m.group(2):
                        unpacked_ref_max = None
                        unpacked_ref_min = m.group(2)
                        packed_ref_max = None
                        vals = m.group(3).split("+:")
                        assert(len(vals) == 2), "Line number is {}. Line is {}".format(line_num, line)
                        packed_ref_min = vals[0]
                        length = vals[1]
                    else:
                        assert(0), "Shouldn't have reached here. Line number is {}. Line is {}. Array usage is {}".format(line_num, line, array_usage)

                    #Calculate the msb and lsb of the new bitvector
                    (max, min) = dict_of_bit_vector[key].gen_bitvector_based_ref(unpacked_ref_max, unpacked_ref_min, packed_ref_max, packed_ref_min)
                    bitvector_usage = name + "[(" + min + ") +: (" + length + ")]"

                    #Replace the array reference with the new bitvector reference
                    line = re.sub(re.escape(array_usage), bitvector_usage, line)
            outfile.write(line)



        # Lines with array references of type 2 (arr[][])
        elif len(array_line_type2) != 0:
            for array_usage in array_line_type2:
                m = re.search(r'([\w]+)\s*\[([\w\-:\* ]*?)\]\s*\[([\w\-:\* ]*?)\]', array_usage)
                #If we are here, this search should return something because we just used the same regex
                assert (m is not None)
                name = m.group(1)

                #If this array doesn't exist in the dict, that means it's not really an array
                #could be a bitvector
                key = infile.name + ":" + name
                if key not in dict_of_bit_vector:
                    #print("Found usage of x[], but this must be a bitvector")
                    pass
                else:
                    #Find the msb and lsb of each dimension of the array
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
                        assert(len(vals) == 2), "Line number is {}. Line is {}".format(line_num, line)
                        unpacked_ref_max = vals[0]
                        unpacked_ref_min = vals[1]
                        vals = m.group(3).split(":")
                        assert(len(vals) == 2), "Line number is {}. Line is {}".format(line_num, line)
                        packed_ref_max = vals[0]
                        packed_ref_min = vals[1]
                    else:
                        assert(0), "Shouldn't have reached here. Line number is {}. Line is {}. Array usage is {}".format(line_num, line, array_usage)

                    #Calculate the msb and lsb of the new bitvector
                    (max, min) = dict_of_bit_vector[key].gen_bitvector_based_ref(unpacked_ref_max, unpacked_ref_min, packed_ref_max, packed_ref_min)
                    bitvector_usage = name + "[(" + max + ") : (" + min + ")]"

                    #Replace the array reference with the new bitvector reference
                    line = re.sub(re.escape(array_usage), bitvector_usage, line)
            outfile.write(line)

        # Lines with array references of type 1 (arr[])
        elif len(array_line_type1) != 0:
            for array_usage in array_line_type1:
                m = re.search(r'([\w]+)\s*\[([\w\-:\* ]*?)\]', array_usage)
                #If we are here, this search should return something because we just used the same regex
                assert (m is not None)
                name = m.group(1)
                key = infile.name + ":" + name

                #If this array doesn't exist in the dict, that means it's not really an array
                #could be a bitvector
                if key not in dict_of_bit_vector:
                    #print("Found usage of x[], but this must be a bitvector")
                    pass
                else:
                    #Find the msb and lsb of each dimension of the array
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

                    #Calculate the msb and lsb of the new bitvector
                    (max, min) = dict_of_bit_vector[key].gen_bitvector_based_ref(unpacked_ref_max, unpacked_ref_min, packed_ref_max, packed_ref_min)
                    bitvector_usage = name + "[(" + max + ") : (" + min + ")]"

                    #Replace the array reference with the new bitvector reference
                    line = re.sub(re.escape(array_usage), bitvector_usage, line)
            outfile.write(line)

        #If we don't meet any of the criteria above, just write it to outfile
        else:
            outfile.write(line) 


##################################################
# Parse command line arguments
##################################################
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-t",
                        "--tot",
                        action='store',
                        default="/export/aman/tpu_v2/",
                        type=str,
                        help='Top of the git repo')
    parser.add_argument("-o",
                        "--outfile",
                        action='store',
                        default="vector_all.v",
                        type=str,
                        help='Name of output file')
    args = parser.parse_args()
    return args

##################################################
# main()
##################################################
if __name__ == "__main__":
    #####################################
    #Parse command line arguments
    #####################################
    args = parse_args()
    TOT = args.tot
    outfile_name = args.outfile

    #TODO: For now, I'm just handling the vector part of the code
    #and collecting it in a vector_all file. We need to collect
    #the code for full TPU in one file.

    #####################################
    #Files to process
    #####################################

    print("Processing to handle case statements in vlanes.v...")
    #Special handling of vlanes.v file
    #We need to handle case statements in vlanes.v file.
    #if we need to handle them in other files, we can add those.
    vlanes_in_fname = TOT+"tpu/design/vector/vlanes.v"
    vlanes_in_fhandle = open(vlanes_in_fname, "r")
    vlanes_out_fname = "vlanes.v.temp"
    vlanes_out_fhandle = open(vlanes_out_fname, "w")
    process_case_stmt(vlanes_in_fhandle, vlanes_out_fhandle)
    vlanes_in_fhandle.close()
    vlanes_out_fhandle.close()

    print("Processing to create one file...")
    #Name of the output file
    vector_all_fname = outfile_name +".1"
    vector_all_fhandle = open(vector_all_fname, "w")
    
    #Name of the main processor file that hierarchically includes other files
    vector_dir = TOT+"tpu/design/vector/"
    vector_main_fname = vector_dir+"vpu.v"
    #TESTING vector_main_fname = vector_dir+"vlanes.v"
    vector_main_fhandle = open(vector_main_fname, "r")
    
    #Directories where the included files can be found
    possible_dirs = ["tpu/design/vector/", \
                     "tpu/design/top/", \
                     "tpu/vtr/"]
    
    #vector_all_fhandle.write("`define USE_INHOUSE_LOGIC\n")
    create_one_file(TOT,vector_main_fhandle, vector_all_fhandle, possible_dirs)
    vector_main_fhandle.close()

    #####################################
    #Adding contents of local modules. These are to be added separately
    #####################################
    local_file_list = ["tpu/design/local/spram1.v", \
                       "tpu/design/local/rams.v", \
                       "tpu/design/local/local_add_sub.v", \
                       "tpu/design/local/local_mult.v", \
                       "tpu/design/local/local_fifo.v", \
                       "tpu/design/local/local_shifter.v",\
                        "tpu/design/top/options.v"]
    
    for local_fname in local_file_list:
        local_fhandle = open(TOT+local_fname, "r")
        create_one_file(TOT,local_fhandle, vector_all_fhandle, possible_dirs)
        local_fhandle.close()
    
    #####################################
    #Adding contents of components (from scalar into vector)
    #####################################
    
    components_file_list = ["tpu/design/scalar/components.v"]
    """
    for components_fname in components_file_list:
        components_fhandle = open(TOT+components_fname, "r")
        create_one_file(TOT,components_fhandle, vector_all_fhandle, possible_dirs)
        components_fhandle.close()
    """
    vector_all_fhandle.close()
    
    
    #####################################
    #Now let's post process the one file we've created
    #####################################
    print("Processing to handle arrays...")
    in_fname = outfile_name +".1"
    in_fhandle = open(in_fname, "r")
    out_fname = outfile_name 
    out_fhandle = open(out_fname, "w")
    process_arrays(in_fhandle, out_fhandle)
    in_fhandle.close()
    out_fhandle.close()