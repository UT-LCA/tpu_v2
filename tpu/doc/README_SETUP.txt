VESPA release
Peter Yiannacouras
July 7, 2009


Overview
========

VESPA is a parameterized Verilog implementation of a full soft vector
processor designed for the Terasic DE3 board.  Since it was designed for Altera
FPGAs it uses Altera specific libraries and hence requires the Altera Quartus
II CAD software.  Even Modelsim simulation of the system requires the Altera
simulation libraries used in vespa/modelsim/Makefile.  You will also need to
license both Quartus and Modelsim.


Setup - for C shell (tcsh), adapt to your own shell environment
=====

1. Edit the CSHRC file so that the SPE_HOME environment variable points to 
   the directory where the applications/, vespa/, and compiler-vector/ 
   directories reside.

2. Setup Modelsim and Quartus in your environment as in the CSHRC file.  Make
   sure both are licensed, their respective bin/ directories are in your PATH,
   and the MODEL_TECH and QUARTUS_ROOTDIR environment variables are defined.

3. Add "source <fullpath>/CSHRC" into your .cshrc file 

4. Logout and log back in 

Everything is built on a 32-bit x86 Linux distribution.  I highly recommend
using a compatible machine to avoid rebuilding GCC.  My output from uname is
shown below but I've run it on many other 32-bit x86 Linux and Debian distros.

Linux 2.6.18-6-686 #1 SMP i686 GNU/Linux

If you do need to rebuild the compiler, use the BUILD and CLEAN scripts in 
the compiler/ directory.

Testing the Setup
=================

Go to the vespa/ directory and type "make test".  This should verify that the
compiler, Quartud CAD software, and Modelsim simulation tool are accessible
by the VESPA framework.


Getting Started
===============

  Compile Benchmark:
    1. Go to applications/kozyrakis_blend
    2. Type: make clean
    3. Type: make
    4. Observe that kozyrakis_blend.data.dat and kozyrakis_blend.instr.dat
    were generated.  These are the instruction and data memory images.

  Simulate Benchmark on VESPA in Modelsim:
    0. Ensure you have Modelsim installed, licensed, and in your PATH
    1. Go to vespa/
    2. Type: make de3sim_compile
    3. In vespa/modelsim create a copy of kozyrakis_blend.data.dat named
    data.dat, similarly create instr.dat from kozyrakis_blend.instr.dat.
    4. Type: make de3sim_gui
    5. Modelsim should open and you should see many signals in the Wave window
    6. In Modelsim, to execute the benchmark till it terminates, type: run -all

 WARNING: You may need to rebuild plifs.sl for your version of Modelsim.  In 
 the vespa/modelsim directory type: make cleanplifs plifs.sl

For more details, see vespa/README.txt

