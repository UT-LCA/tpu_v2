import os
entries = os.listdir("verilog")

data = ""
for entry in entries:
    filename = "verilog/" + entry
    f = open(filename,'r')
    data += f.read()
    data += "\n"

f.close()
f = open("verilog/final.v",'w')
f.write(data)
f.close()
