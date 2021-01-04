#-------------------------------------------------------------------------
# Tcl script to parse assembly listing file to generate
# instruction memory initialization file
#
# Generates the following files:
# ./imem.hex
#
# Date created: Jan 14, 2008
#
# Copyright (c) 2008 Jason Yu
#-------------------------------------------------------------------------
#package require Tcl 8.5

# Set path and filenames
set	imem_init_name "./imem_init.hex"

# Look for command-line parameters
if {$argc != 2} {
	puts "Usage: lst2imem.tcl input_lst_name output_hex_name"
	exit
}

set lst_name [lindex $argv 0]
set hex_name [lindex $argv 1]


#-------------------------------------------------------------------------
# Parse listing file that looks as follows.
# Want comments, opcode, mnemonic (insert // to comment out)
#-------------------------------------------------------------------------
#
# GAS LISTING imem.asm 			page 1
# 
# 
#    1              	#--------------------------------------------------------
#    2              	# Vector loads
#    3              	#--------------------------------------------------------
#    4              	# change vector length
#    5 0000 00C006C4 	addi	r3, r0, 27					# set vector length
#    6 0004 00019A3E 	vmstc	VL, r3
#    7              	
#    8 0008 0400483F 	vld.w	v1, vbase1, vinc0			# base=0x04, inc=0
#    9 000c 081888BF 	vlds.w	v2, vbase2, vstride3, vinc0	# base=0x08, stride=3, inc=0
#   10 0010 0C01443F 	vld.h	v3, vbase5, vinc0			# base=0x12, inc=0
#   11 0014 101206BF 	vldsu.h	v4, vbase8, vstride2, vinc0	# base=0x1A, stride=2, inc=0
#   12 0018 1402423F 	vldu.b	v5, vbase9, vinc0			# base=0x1B, inc=0
#   13 001c 182BC0BF 	vlds.b	v6, vbase15, vstride5, vinc0	# base=0x21, stride=5, inc=0
#   14 0020 1C3C08BF 	vlds.w	v7, vbase16, vstride7, vinc0	# base=0x20, stride=20, inc=0
# GAS LISTING imem.asm 			page 2
# 
# 
# NO DEFINED SYMBOLS
# 
# NO UNDEFINED SYMBOLS
# 
#-------------------------------------------------------------------------

puts "Processing listing file [lindex $argv 0]\n"
set lst_file [open $lst_name r]
set hex_file [open $hex_name w 0666]

# need to define variables first
set match {}
set opcode {}
set restofline {}

# Parse file
while { [gets $lst_file line] >= 0 } {
	# match a whole line of comments (line number, comment)
	set match_comment [regexp {^\s*[0-9]+\s+(\#.+)} $line match restofline]
	# puts "match_comment=$match_comment, match=$match, restofline=$restofline"
	
	# if line isn't a line of comment, look for code
	if {$match_comment == 0} {
		# match line number, mem location, (opcode)
		set match_code [regexp {^\s*[0-9]+\s+[0-9a-fA-F]+\s+([0-9a-fA-F]{8})\s+(.+)} $line match opcode restofline]
		# puts "match_code=$match_code, match=$match, opcode=$opcode"
		
		if {$match_code == 1} {
			# comment out restofline
			puts "$opcode\t\/\/ $restofline"
			puts $hex_file "$opcode\t\/\/ $restofline"
		}
	} else {
		puts $restofline
		puts $hex_file $restofline
	}
}

puts "\nGenerated output file [lindex $argv 1]"