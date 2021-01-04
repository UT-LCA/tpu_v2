---------------------------------
Vector-ISA Assembler Instructions

Last updated: December 22, 2007
---------------------------------

1) Download Altera's GNUTools Source Code at: 
ftp://ftp.altera.com/outgoing/download/support/ip/processors/nios2/gnu/niosii-gnutools-src-6.1.tgz

2) Extract Files

3) Move the source files in this folder to the following directories in your NiosII GNUtools source:
nios2.h ---> /binutils/include/opcode
nios2-opc.c ---> /binutils/opcodes
tc-nios2.c ---> /binutils/gas/config

4) Launch Cygwin, and navigate to /binutils and run "configure --target=nios2"

5) Nagivate to /binutils/gas and run "configure --target=nios2"

5) Compile the assembler only by running "make all-gas" in /binutils


---------------------------------
Assembler usage
---------------------------------
Example usage:

as-new.exe -ahlms -EB v_assembler_test.s > v_assembler_test.asm

Type "as-new.exe --help" for more information.


---------------------------------
Testing
---------------------------------
All testing files can be located in the /Test directory.