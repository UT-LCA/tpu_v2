LSI= $(SPE_COMPILER)/tools/lsi.ld
START= $(SPE_COMPILER)/tools/start.nocmdline.o

CFLAGS= -O3 -mips1 -mgp32 -msoft-float -mlong32
ASFLAGS= -O3 -mips1
LDFLAGS= -T $(LSI) -L $(SPE_COMPILER)/tools -L $(SPE_COMPILER)/mips-mips-elf/lib/soft-float -L $(SPE_COMPILER_LIBPATH) -lm -lc -lgcc -lspe

CC=$(SPE_COMPILER)/bin/mips-mips-elf-gcc
CXX=$(SPE_COMPILER)/bin/mips-mips-elf-g++
AS=$(SPE_COMPILER)/bin/mips-mips-elf-as
NM=$(SPE_COMPILER)/bin/mips-mips-elf-nm
LD=$(SPE_COMPILER)/bin/mips-mips-elf-ld

