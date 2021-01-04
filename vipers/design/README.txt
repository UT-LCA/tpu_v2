-----------------------------------------------------------
Soft-core Vector Processor Project
Implementation version 0.8

Author: Jason Yu
Last updated: April 23, 2007
-----------------------------------------------------------

The current implementation has the following default parameters:
NLane			8
MVL			32
VPUW			32
MEMWIDTH		128
MemMinWidth		8
MACL			2
LMemN			256
LMemShare		On


-----------------------------------------------------------
To use the processor in SOPC Builder (7.2)
-----------------------------------------------------------
1) Put the "ip" directory in your Quartus project directory

2) Create a Quartus project and open the SOPC Builder.  The processor should be listed under
the "Processors" category.  Add the component and set parameters accordingly.

3) Generate the system in SOPC Builder.

4) Run the configuration Tcl script in "ip/vect_cpu/vectcpu_config.tcl"

The script generates or configures the following files:

mif/NElemXfer_rom.mif
modelsim/NElemXfer_rom.dat
mif/NElemRemainder_rom.mif
modelsim/NElemRemainder_rom.dat
hdl/NElemRemainder_rom_init.v
hdlNElemXfer_rom_init.v
mif/control_reg_other.mif
modelsim/control_reg_other.dat
modelsim/control_reg_other.hex
hdl/config_def_auto.v
modelsim/dmem_init.hex
testbench/tb_VectCPU.v

Use the same parameters as set previously.


5) Configure additional features/parameters by manually editing "ip/vect_cpu/hdl/config_def.v" (optional)

6) Copy mif files from "ip/vect_cpu/mif" and "ip/utiie_cpu/mif" directories to the
Quartus project directory.

7) Compile the system!


-----------------------------------------------------------
To use the processor in Nios II EDS (7.2)
-----------------------------------------------------------
The vector processor contains no debug core, so software cannot be directly downloaded to it.
However, using the debug function of a second Nios II (economy, or any other) core,
it is possible to use the vector processor with the Nios II EDS.
Debug capabilities will obviously be unavailable.

1) Create/copy a SOPC Builder system

a) Create a system that contains, a minimum, the following components:
- Nios II/e processor
- Soft vector processor
- small on-chip instruction memory for Nios II to store reset code (I'll call this nios_imem)
- larger on-chip (or off-chip) instruction memory to store program code, and exception code
(I'll call this shared_imem; could be off-chip sram, etc.)
- a memory for storing exception code

b) Create the instruction and data master connections.
- only the instruction and data masters of Nios II/e need access to nios_imem
- instruction masters of both Nios II/e and Soft vector processors
and data master of Nios II/e need to be connected to shared_imem
- all memories that will store program code need to be connected to Nios II/e data master
(in addition to desired instruction master) because Nios II EDS
uses the Nios II data master (jtag debug module) to download code

c) Configure nios_imem to use a custom memory initialization file (I'll call this my_nios_imem.hex)

d) Create my_nios_imem.hex memory initialization file with the following two 32-it words at the beginning:
0x0001883A    (corresponds to "nop")
0x003FFE06    (corresponds to "br -1")

This puts the Nios II/e in an infinite loop so the Soft vector processor can take over the system.

e) Set the reset vector of Nios II/e to nios_imem

f) Set the reset vector of Soft vector processor to shared_imem

g) Set both processors' exception address to exception code memory

h) Generate system, compile, program FPGA


Create a new project in Nios II EDS, targetting the SOPC system

1) In project system library properties, set program memory region (.text) to shared_imem.
Other regions can be set to any valid location.

2) Copy the custom vector processor assembler (as-new.exe) to the following location:
(you should backup the original version first)

C:\altera\72\nios2eds\bin\nios2-gnutools\H-i686-pc-cygwin\nios2-elf\bin\as.exe

3) Write vector assembly inside Nios II EDS where needed using gcc extended assembly.

4) Compile the project in Nios II EDS and run in hardware!


-----------------------------------------------------------
To setup the processor for simulation
-----------------------------------------------------------
The default ModelSim simulation directory is "ip/vect_cpu/modelsim"

1) Run the configuration Tcl script in "ip/vect_cpu/vectcpu_config.tcl"

The script generates or configures the following files:

mif/NElemXfer_rom.mif
modelsim/NElemXfer_rom.dat
mif/NElemRemainder_rom.mif
modelsim/NElemRemainder_rom.dat
hdl/NElemRemainder_rom_init.v
hdl/NElemXfer_rom_init.v
mif/control_reg_other.mif
modelsim/control_reg_other.dat
modelsim/control_reg_other.hex
hdl/config_def_auto.v
modelsim/dmem_init.hex
testbench/tb_VectCPU.v

Use the same parameters as set previously.


2) Configure additional features/parameters by manually editing "ip/vect_cpu/hdl/config_def.v" (optional)

3) Put instructions (in hex) for the processor to execute into "ip/vect_cpu/modelsim/imem.hex"
in the ModelSim directory

4) Pass the Verilog definition "+define+MODELSIM" to all Verilog source files when compiling

5) Turn off all optimizations for loadAddrGenerator.v

6) Simulate the "ip/testbench/tb_VectCPU" testbench


-----------------------------------------------------------
To use the processor in device families other than Stratix III
-----------------------------------------------------------
The processor uses Altera megafunctions for Stratix III family,
and these megafunctions need to be changed for other device families.

1) Open Quartus II Megafunction Wizard

2) Edit Megafunction Wizard generated files in "ip/vect_cpu/hdl" directory, keep a copy of old files
(lanemult_mf.v, VLane_loadstoreDataQ.v, VLane_storeFlagQ.v, scalarToVectorQ.v)

3) Change the device family to the desired family

4) Generate new Verilog source files from Megafunction Wizard

5) Compare new and old Verilog source files and insert Verilog parameters into new files where they existed in old files


To use MAC units in other device families:

6) To use the MAC units, edit the MAC megafunction generated file (altMultAccum.v) and change device family

7) Edit "ip/vect_cpu/hdl/config_def.v" and modify architecture-dependent definitions for the MAC units
(e.g. number of multipliers that feed the adder/accumulator, width of MAC unit, width of multipliers)



-----------------------------------------------------------
Current Limitations
-----------------------------------------------------------
- minimum number of lanes is 4, number of lanes should be a power of 2,
and VPUW*NUMLANES / MEM_WIDTH should be an integer > 0
- maximum number of lanes must be <= memory interface width
