import argparse

class gen_config():
    def __init__(self):
        pass

    def write(self, sub_dir, v_file):
        base_dir = "/home/aatman/Desktop/utexas_thesis"
        config_path = base_dir + "/vtr-verilog-to-routing/vtr_flow/tasks/tpu/config/config.txt"
        fp = open(config_path, "w")

        string = '''\
##############################################
# Configuration file for running experiments
##############################################

# Path to directory of circuits to use
circuits_dir=benchmarks/verilog/ml_benchmarks/tpu_subunits/{sub_dir}

# Path to directory of architectures to use
#archs_dir=arch/timing
archs_dir=arch/COFFE_22nm

# Add circuits to list to sweep
circuit_list_add={verilog_file}
#circuit_list_add=simple_mem.v
#circuit_list_add=ch_intrinsics.v

# Add architectures to list to sweep
#arch_list_add=k6_frac_N10_mem32K_40nm_aman.xml
#arch_list_add=k6_frac_N10_mem32K_40nm.xml
arch_list_add=agilex_like_arch.auto_layout.xml

# Parse info and how to parse
parse_file=vpr_standard.txt

# Pass requirements
pass_requirements_file=pass_requirements.txt'''

        string = string.format(sub_dir = sub_dir, verilog_file=v_file)
        fp.write(string)
        fp.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="verilog file name")
    parser.add_argument('sub_dir', type=str, help='verilog sub directory')
    parser.add_argument('v_file', type=str, help='verilog file name')
    args = parser.parse_args()

    config_obj = gen_config()

    config_obj.write(args.sub_dir, args.v_file)
