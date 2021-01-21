Peter Yiannacouras
Nov 1, 2008

Overview
========

This VESPA processor consists of our full TM4 hardware design including: the
scalar MIPS processor, the VIRAM vector coprocessor, L1 caches, DDR
controller, a counter, and a dead flag indicating end of execution.  
Execution is hardwired to begin at address 0x04000000 which is where the 
.text section begins, while .data begins at 0x10000000.


HOWTO - run a benchmark in Modelsim
===================================

0. Let this directory be called PWD.  Make sure you've setup Quartus and
Modelsim according to README_SETUP.txt

1. In PWD, type "make de3sim_compile"
2. Copy the <benchmark>.instr.dat and <benchmark>.data.dat of the benchmark
you want to run into PWD/modelsim
3. In PWD to simulate either:
  a) Graphically with waveforms using "make de3sim_gui"
  b) With the command line using "make de3sim" 
       - this will do a 'run -all' in Modelsim, the testbench will quit when
       the program has finished executing
4. To run another benchmark, repeat steps 2 and 3.

Warning: In simulation we simulate with 256KB instruction memory and a 32MB
data memory.  This can be changed in modelsim/de3_test_bench.v.

Warning: Any call to printf takes LONG to simulate!


HOWTO - run a benchmark on the DE3
===================================

Setting up communication between the host PC and the DE3 is very specific to
the host PC system.  The University of Toronto Transmogrifier Ports Package is
used for this in VESPA and is invoked with the "tmj" command in the Makefile.
If you're on the University of Toronto EECG system, simply add ~tm4/bin to your
PATH.  If not, you can download the JTAG-based ports package (designed for a
Linux-based system) from

http://www.eecg.toronto.edu/~tm4/tmjports/

Otherwise a custom communication layer can be designed after stripping out the
the ports package support in de3.v. 

Assuming tmj is setup you can run a benchmark by doing the following:

  1. Synthesize VESPA using "make compile".  This will invoke tmj
  2. Configure the FPGA using "make download"
  3. Go to tmj/src and type "make" to build tmj/bin/de3benchmark
  4. Copy instr.dat and data.dat from benchmark to vespa/ directory (with those
     same names, similar to what was done for Modelsim)
  5. Change directories back to vespa/
  6. Execute the benchmark on the DE3 by typing "../tmj/bin/de3benchmark"


Parameterization
================

The "setoption" script can be used to set the different parameters in VESPA,
type "./setoption" with no parameters to see help.  All the parameters are
found in options.v but setoption will also do the instruction subsetting
customization.  Also modifying MVL requires changes to other files that are
done automatically by setoption.

WARNING:  Changing the number of banks (vector chaining) requires manual
changes to the de3_test_bench.v in two places.  Search for FIXME in that file
for more help.


Peripherals
===========

A description of the peripherals included in the system follows:

  counter
  -------
    Located at memory mapped address: 0x80000100
    The counter is 38 bits wide, but an application can only access
    the upper 32 bits, hence, from the program's perspective the 
    counter increments every 64 cycles.  Programs should write 0 to the 
    counter on startup.  

  dead
  ----
    Located at memory mapped address: 0x80000104
    A program should write 0xdeaddead to this address when the program
    wants to terminate.


Assumed Memory Map
==================

  0x04000000-0x0FFFFFFF: .text
  0x10000000-0x7FFFFFFF: .data
  0x80000000-0xfFFFFFFF: uncached memory I/O space
      0x80000000: filesystem stream
      0x80000100: counter (only upper 32-bits accessible)
      0x80000104: dead flag (indicates end of program execution)


Software Tools
==============

There are also some software tools in this directory that you may need to
rebuild for your system before using.

  vdecoder
  --------
    Generates visa.v which defines the instruction encodings used by the 
    hardware to decode vector instructions.  To build this do:
        rm -f vdecoder
        make vdecoder
    To create visa.v use:
        ./vdecoder [instr.dat] >visa.v
    The instr.dat argument is optional and points to the .dat file that
    stores the instruction stream of a benchmark you want to subset.  Any
    instructions not found in instr.dat will be disabled.

  virtualtype
  -----------
    Generates virtualtype.do which tells Modelsim of the instruction 
    encodings so that we see instruction names rather than encodings.
    This tool is built automatically, but you can build it with:
        cd modelsim
        rm -f virtualtype
        make virtualtype
    To create virtualtype.do use the following in the modelsim/ directory:
        ./virtualtype

  plifs
  -----
    You don't need to worry about this tool, it is built automatically by "make
    de3sim_compile".  If it's causing problems it may be removed.  The plifs
    uses Modelsim's Verilog PLI interface to allow your simulated benchmark to
    access files on your workstation.  It is designed to emulate the filesystem
    support we've built over PCI on the TM4.  It is necessary only to support
    stdio in the simulated program.

    To build plifs.sl, make sure the environment variable MODEL_TECH is
    defined.  You can then build it using: 
        cd modelsim && make cleanplifs plifs.sl

