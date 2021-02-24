import re
import os


def print_contents(infile, outfile, possible_dirs):
    outfile.write("//////////////////////////////////////////////////////////////////////\n")
    outfile.write("//// Starting contents of file: {}\n".format(infile))
    outfile.write("//////////////////////////////////////////////////////////////////////\n")
    for line in infile:
        m = re.match(r'`include "(.*)"', line)
        n = re.match(r'`timescale', line)
        if m is not None:
            included_filename = m.group(1)
            outfile.write("//////////////////////////////////////////////////////////////////////\n")
            outfile.write("//// Starting contents of included file: {}\n".format(included_filename))
            outfile.write("//////////////////////////////////////////////////////////////////////\n")
            print(included_filename)
            #check for file
            found_dir = ""
            for dir in possible_dirs:
                if (os.path.exists(dir+included_filename)):
                    found_dir = dir  
                    break
            if found_dir == "":
                print("File {} not found in possible directories".format(included_filename))
                raise SystemExit(0)
            else:
                print("File {} found in directory {}".format(included_filename, found_dir))
            included_f = open(found_dir+included_filename, "r")
            print_contents(included_f, outfile, possible_dirs)
            outfile.write("//////////////////////////////////////////////////////////////////////\n")
            outfile.write("//// Finish contents of included file: {}\n".format(included_filename))
            outfile.write("//////////////////////////////////////////////////////////////////////\n")
        elif n is not None:
            continue
        else:
            outfile.write(line) 
 
#Scalar processor
scalar_dir = "/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/"
scalar_all_fname = "scalar_all.v"
scalar_all_fhandle = open(scalar_all_fname, "w")

scalar_main_fname = scalar_dir+"system.v"
scalar_main_fhandle = open(scalar_main_fname, "r")

possible_dirs = ["/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/", \
                 "/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/top/"]

scalar_all_fhandle.write("`define USE_INHOUSE_LOGIC\n")
print_contents(scalar_main_fhandle, scalar_all_fhandle, possible_dirs)

#Vector processor
vector_dir = "/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/vector/"
vector_all_fname = "vector_all.v"
vector_all_fhandle = open(vector_all_fname, "w")

vector_main_fname = vector_dir+"vpu.v"
vector_main_fhandle = open(vector_main_fname, "r")

possible_dirs = ["/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/vector/", \
                 "/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/top/"]

vector_all_fhandle.write("`define USE_INHOUSE_LOGIC\n")
print_contents(vector_main_fhandle, vector_all_fhandle, possible_dirs)

#Adding contents of local modules
local_file_list = ["/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/top/spram1.v", \
                   "/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/top/rams.v", \
                   "/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/local/local_add_sub.v", \
                   "/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/local/local_mult.v", \
                   "/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/local/local_fifo.v", \
                   "/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/local/local_shifter.v"]

for local_fname in local_file_list:
    local_fhandle = open(local_fname, "r")
    print_contents(local_fhandle, vector_all_fhandle, possible_dirs)
    print_contents(local_fhandle, scalar_all_fhandle, possible_dirs)

#Adding contents of components (from scalar into vector)
components_file_list = ["/home/projects/ljohn/aarora1/tpu_v2/2/tpu_v2/tpu/design/scalar/components.v"]

for components_fname in components_file_list:
    components_fhandle = open(components_fname, "r")
    print_contents(components_fhandle, vector_all_fhandle, possible_dirs)

#Commenting this for now. Somehow multiple lines get deleted if this is done from inside the script
#os.system("dos2unix {}".format(vector_all_fname))
#os.system("dos2unix {}".format(scalar_all_fname))
