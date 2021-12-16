
Directory structure

--final
	--scalar
		--system.v
	--vector
		--vector_all.v
		--vector_fin.v
		--config.txt
                --diff.patch

System.v - contains the scalar code that currently passes the yosys flow

vector_all.v - File that was generated from tpu/vtr/gen_tpu_for_vtr.py. In order to regenerate this file run-

python3 gen_tpu_for_vtr.py -t <path to tpu-v2>

vector_fin.v - File that was used for the yosys+odin flow. This was obtained with modifications 
               to vector_all.v as described in docu.pdf

diff.patch- This shows the changes that were made to vector_all.v to obtain vector_fin.v

config.txt- The config file that was used to run the vtr task

