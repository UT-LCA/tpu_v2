from options import options
from matmul_defines import matmul_defines

from gen_tpu_for_vtr import bit_vector
from gen_tpu_for_vtr import process_arrays

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
f1 = open("verilog/final1.v", 'w')
process_arrays(f, f1)
f1.close()
f.close()
