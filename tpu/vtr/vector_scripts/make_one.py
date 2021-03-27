from options import options
from matmul_defines import matmul_defines

from gen_tpu_for_vtr import bit_vector
from gen_tpu_for_vtr import process_arrays
from gen_tpu_for_vtr import process_LO

import os
entries = os.listdir("verilog")

data = ""
for entry in entries:
    filename = "verilog/" + entry
    f = open(filename,'r')
    data += f.read()
    data += "\n"

f = open("verilog/final.v","w")
uut = options(f)
uut.write()
uut = matmul_defines(f)
uut.write()
f.close()

f = open("verilog/final.v",'a')
f.write(data)
f.close()

f = open("verilog/final.v", 'r')
f2 = open("verilog/final2.v", 'w')
process_arrays(f, f2)
f2.close()
f.close()

f2 = open("verilog/final2.v", "r")
f1 = open("verilog/final1.v", "w")
process_LO(f2, f1)
f1.close()
f2.close()
