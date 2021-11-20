export SPE_HOME ../../../../
SPE_HOME=../../../../
export SPE_COMPILER ${SPE_HOME}/tpu_v2/tpu/compiler-vector
SPE_COMPILER=${SPE_HOME}/tpu_v2/tpu/compiler-vector
export SPE_COMPILER_LIBPATH ${SPE_COMPILER}/lib/gcc/mips-mips-elf/4.2.0/soft-float
SPE_COMPILER_LIBPATH=${SPE_COMPILER}/lib/gcc/mips-mips-elf/4.2.0/soft-float

TARGET=bfloat16_mult
SRC=bfloat16_mult.c bfloat_mult.S
OBJS=bfloat16_mult.o bfloat_mult.o
include ../Makefile.inc
include ../Makefile.sim


######################## For cross compiled GCC ##########################

all: $(TARGET)

# START must be linked first!!! Since RTL model blindly executes from 400000
$(TARGET): $(OBJS)
	@$(LD) $(START) $(OBJS) $(LDFLAGS) -o $(TARGET)

.c.o: 
	@$(CC) $(CFLAGS) -c -o $*.o $<
.S.o: 
	@$(CC) $(ASFLAGS) -c -o $*.o $<
.s.o: 
	@$(AS) $(ASFLAGS) -o $*.o $<
.c.s:	
	@$(CC) $(CFLAGS) -S -o $*.s $<

clean: clean_sim
	@rm -f $(TARGET) $(OBJS) $(TARGET).srec 
