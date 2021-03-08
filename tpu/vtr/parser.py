import argparse

def parse():
    parser = argparse.ArgumentParser(description="verilog file name")
    parser.add_argument('sub_dir', type=str, help='verilog sub directory')
    parser.add_argument('v_file', type=str, help='verilog file name')
    args = parser.parse_args()
    
    return "verilog"+"/"+args.sub_dir+"/"+args.v_file