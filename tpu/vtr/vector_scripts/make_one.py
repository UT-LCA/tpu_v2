from options import options
from matmul_defines import matmul_defines

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

f.close()
f = open("verilog/final.v",'a')
f.write(data)
f.close()
