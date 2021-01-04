#-------------------------------------------------------------------------
# Tcl script to configure the vector processor
# Generates ROM data files and configuration HDL files
#
# Copyright (c) 2007 Jason Yu
#-------------------------------------------------------------------------
package require Tcl 8.5

#-------------------------------------------------------------------------
# Procedures
#-------------------------------------------------------------------------
proc mylog2 {num} {
	set i 1
	set base 2
	set val 2
	while {$num > $val} {
		set val [expr {$val*$base}]
		incr i
	}
	return $i
}

proc log {base x} {expr {log($x)/log($base)}}


# proc to write quartus memory initialization file (.mif)
# inputs: memory depth, .mif filename, array data as location-value pair
proc write_mif_file {width depth writefile init_array} {
	upvar $init_array data_array
	set writehandle [open $writefile w 0666]
	puts $writehandle "-- Copyright (C) 1991-2007 Altera Corporation,\n -- Your use of Altera Corporations design tools, logic functions \n-- and other software and tools, and its AMPP partner logic \n-- functions, and any output files from any of the foregoing \n-- (including device programming or simulation files), and any \n-- associated documentation or information are expressly subject \n-- to the terms and conditions of the Altera Program License \n-- Subscription Agreement, Altera MegaCore Function License \n-- Agreement, or other applicable license agreement, including, \n-- without limitation, that your use is for the sole purpose of \n-- programming logic devices manufactured by Altera and sold by \n-- Altera or its authorized distributors.  Please refer to the \n-- applicable agreement for further details."
	puts $writehandle "-- Quartus II generated Memory Initialization File (.mif)\n"
	puts $writehandle "WIDTH=$width;"
	puts $writehandle "DEPTH=$depth;"
	puts $writehandle "ADDRESS_RADIX=UNS;"
	puts $writehandle "DATA_RADIX=UNS;\n"
	puts $writehandle "CONTENT BEGIN"

	for {set addr 0} {$addr < $depth} {incr addr} {
		# check if initial value has been set
		if {[info exists data_array($addr)]} {
			puts $writehandle "\t$addr\t:\t$data_array($addr);"
		} else {
			# write zero if value not initialized
			puts $writehandle "\t$addr\t:\t0;"
		}
	}

	puts $writehandle "END;"
	close $writehandle
}


# proc to write ModelSim dat file (.dat)
# inputs: starting address (decimal), .dat filename, array data as location-value pair
proc write_dat_file {start_addr_hex depth writefile init_array} {
	upvar $init_array data_array
	set writehandle [open $writefile w 0666]
	puts $writehandle "@$start_addr_hex"

	for {set addr 0} {$addr < $depth} {incr addr} {
		# check if initial value has been set
		if {[info exists data_array($addr)]} {
			puts $writehandle [format %08x $data_array($addr)]
		} else {
			# write zero if value not initialized
			puts $writehandle [format %08x 0]
		}
	}
	close $writehandle
}


# proc to write ModelSim hex file (.hex)
# inputs: starting address (decimal), ..hex filename, array data as location-value pair
proc write_hex_file {depth writefile init_array} {
	upvar $init_array data_array
	set writehandle [open $writefile w 0666]

	for {set addr 0} {$addr < $depth} {incr addr} {
		# check if initial value has been set
		if {[info exists data_array($addr)]} {
			puts $writehandle [format %08x $data_array($addr)]
		} else {
			# write zero if value not initialized
			puts $writehandle [format %08x 0]
		}
	}
	close $writehandle
}



#-------------------------------------------------------------------------
# Set path and filenames
#-------------------------------------------------------------------------
set NElemXfer_rom_mif_name "./mif/NElemXfer_rom.mif"
set NElemXfer_rom_dat_name "./modelsim/NElemXfer_rom.dat"
set NElemRemainder_rom_mif_name "./mif/NElemRemainder_rom.mif"
set NElemRemainder_rom_dat_name "./modelsim/NElemRemainder_rom.dat"
set	NElemRemainder_rom_include_name "./hdl/NElemRemainder_rom_init.v"
set	NElemXfer_rom_include_name "./hdl/NElemXfer_rom_init.v"
set controlreg_mif_name "./mif/control_reg_other.mif"
set controlreg_dat_name "./modelsim/control_reg_other.dat"
set controlreg_hex_name "./modelsim/control_reg_other.hex"
set controlreg_dat_temp_name "./modelsim/control_reg_other.tmp.dat"
set controlreg_hex_temp_name "./modelsim/control_reg_other.tmp.hex"
set config_def_name "./hdl/config_def_auto.v"
set	dmem_init_name "./modelsim/dmem_init.hex"
set	vstride_hex_name "./modelsim/vstride.hex"
set	vinc_hex_name "./modelsim/vinc.hex"
set	tb_template_name "./testbench/tb_VectCPU_template.v"
set	tb_name "./testbench/tb_VectCPU.v"
set	replace_symbol "**template_replace_line**"
set	controlreg_dat_start_addr_hex "00000000"

set dmem_numword 512


#-------------------------------------------------------------------------
# Set Stratix III architecture parameters
#-------------------------------------------------------------------------
# number of multipliers per MAC accumulator
set	device_mac_nmult 4
# maximum MAC cascade chain length
set	max_macl 4


#-------------------------------------------------------------------------
# Get user configuration
#-------------------------------------------------------------------------
puts "\nPlease enter the number of vector lanes:"
set numlanes [gets stdin]
puts "\nPlease enter the VPU width (8, 16, 32):"
set vpu_width [gets stdin]
puts "\nPlease enter the width of the external memory interface (32, 64, 128):"
set mem_width [gets stdin]

puts "\nPlease enter the minimum accessible data width from main memory (8, 16, 32):"
set mindatawidth [gets stdin]

# error check mindatawidth
while {$vpu_width < $mindatawidth} {
	puts "Error in MINDATAWIDTH: MINDATAWIDTH greater than VPUW!"
	puts "\nPlease enter the minimum accessible data width from main memory (8, 16, 32):"
	set mindatawidth [gets stdin]
}

puts "\nPlease enter the accumulate chain length (1, 2, 4... maximum possible value is log2(numlanes/4) for Stratix III):"
set macl [gets stdin]

# error check MACL
while {$macl*$device_mac_nmult > $numlanes || $macl > $max_macl} {
	puts "Error in MACL: MACL too long for architecture and number of lanes!"
	puts "\nPlease enter the accumulate chain length (1, 2, 4; maximum possible value is min( log2(numlanes/4), 4 )):"
	set macl [gets stdin]
}


# setup MAC units
#puts "\nWould you like to turn on MAC units (y, n)"
#set input_tolower [string tolower [gets stdin]]

#if {$input_tolower eq "y"} {
	#set MAC_enabled 1
	#puts "\nPlease enter the accumulate chain length (1, 2, 4; up to min( log2(numlanes/4), 4 )):"
	#set macl [gets stdin]
#} else {
	#set MAC_enabled 0
	#set macl 0
#}

# setup local memories
#puts "\n Would you like to include local memories (y,n)"
#set input_tolower [string tolower [gets stdin]]

# local memory on
#if {$input_tolower eq "y"} {
	#set LM_enabled 1
	#puts "\nPlease enter the number of (VPU width bit) local memory words:"
	#set LM_numword [gets stdin]
	#puts "\nIs the local memory shared between virtual processors within a lane?"
	#puts "(each VP sees the same physical memory; beware of memory consistency issues) (y,n)"
	
	#if {$input_tolower eq "y"} {
		#set LM_share 1
	#} else {
		#set LM_share 0
	#}
# no local memory
#} else {
	#set LM_enabled 0
	#set LM_numword 0
#}


# set other parameters
set memwidth_bytes [expr {$mem_width/8}]


#-------------------------------------------------------------------------
# Initialize data array
#-------------------------------------------------------------------------
#vc0 = vector length
set controlreg_array(0) [expr {$numlanes*4}]
#vc1 = VPU width
set controlreg_array(1) $vpu_width
#vc28 = ACCncopy
set controlreg_array(28) [expr {int([tcl::mathfunc::ceil [log [expr {$device_mac_nmult * $macl}] $numlanes]])}]
#vc29 = numlanes
set controlreg_array(29) $numlanes
#vc30 = MVL
set controlreg_array(30) [expr {$numlanes*4}]
#vc31 = log2(MVL)
set controlreg_array(31) [mylog2 [expr {$numlanes*4}]]

# read vstride values from file
set controlreg_addr 32
set vstride_hex [open $vstride_hex_name r]
while { [gets $vstride_hex line] >= 0 } {
	# convert from hex to decimal
	set controlreg_array($controlreg_addr) [expr 0x$line]
	incr controlreg_addr
}
close $vstride_hex

# read vinc values from file
set vinc_hex [open $vinc_hex_name r]
while { [gets $vinc_hex line] >= 0 } {
	# convert from hex to decimal
	set controlreg_array($controlreg_addr) [expr 0x$line]
	incr controlreg_addr
}
close $vinc_hex

#puts [array get controlreg_array]


#-------------------------------------------------------------------------
# Generate data for loadAddrGen divider_rom ROM table
#-------------------------------------------------------------------------
set localstride_nbit [expr {[mylog2 $memwidth_bytes] +1}]
set	maxNElemXfer_nbit [expr {[mylog2 $memwidth_bytes]+1}]
#puts "localstride_nbit = $localstride_nbit"
#puts "maxNElemXfer_nbit = $maxNElemXfer_nbit"

# max_localstride is memwidth_bytes-1;
# any greater stride would be "largestride" and use different hardware
set	max_localstride [expr {$memwidth_bytes-1}]
set	maxNElemXfer_offsetted_limit $memwidth_bytes

# ROM tables
# Number of elements that can be transferred this cycle
#NElemXfer_rom = zeros(1,2^(localstride_nbit+maxNElemXfer_nbit));
set	NElemXfer_rom {}
# How many elements remain in the current block after the current transfer
#NElemRemainder_rom = zeros(1,2^(localstride_nbit+maxNElemXfer_nbit));
set	NElemRemainder_rom {}

for {set i 0} {$i < [expr {2**[expr {$localstride_nbit+$maxNElemXfer_nbit}]}]} {incr i} {
	lappend NElemXfer_rom 0
	lappend NElemRemainder_rom 0
}


# Generate data for ROM tables
# iterate through all strides
for {set stride 0} {$stride <= $max_localstride} {incr stride} {
    for {set maxNElemXfer_offsetted 0} {$maxNElemXfer_offsetted <= $maxNElemXfer_offsetted_limit} {incr maxNElemXfer_offsetted} {
        set ind [expr {$maxNElemXfer_offsetted * 2**$maxNElemXfer_nbit + $stride}]
        
		# if {$ind == 34} {
			# puts "stride = $stride, maxNElemXfer_offsetted = $maxNElemXfer_offsetted"
		# }
		
        # Zero stride case, transfer as quickly as possible
        if {$stride == 0} {
            lset NElemXfer_rom $ind $numlanes
            lset NElemRemainder_rom $ind 0
        # special case, if number of elements that can be transferred is
        # more than number of lanes * stride, need to limit to numlane
        } elseif {$maxNElemXfer_offsetted > [expr {$numlanes*$stride}]} {
            lset NElemXfer_rom $ind $numlanes
            # set remainder to be number of elements left to be transferred
            # in the current block
            lset NElemRemainder_rom $ind [expr {$maxNElemXfer_offsetted-$numlanes*$stride}]

            #puts "stride = $stride"
            #puts "maxNElemXfer_offsetted = $maxNElemXfer_offsetted"
            #puts "NElemXfer_rom\[$ind] = [lindex $NElemXfer_rom $ind]"
            #puts "NElemRemainder_rom\[$ind] = [lindex $NElemRemainder_rom $ind]"
        # normal operation
        } else {
            lset NElemXfer_rom $ind [expr {int(ceil(double($maxNElemXfer_offsetted) / $stride))}]
            lset NElemRemainder_rom $ind [expr {$maxNElemXfer_offsetted % $stride}]
		}
	}
}


#-------------------------------------------------------------------------
# Write data to .mif and .dat files
#-------------------------------------------------------------------------
# Write NElemXfer_rom.mif
set NElemXfer_rom_mif [open $NElemXfer_rom_mif_name w 0666]

puts $NElemXfer_rom_mif "-- Copyright (C) 1991-2007 Altera Corporation,\n -- Your use of Altera Corporations design tools, logic functions \n-- and other software and tools, and its AMPP partner logic \n-- functions, and any output files from any of the foregoing \n-- (including device programming or simulation files), and any \n-- associated documentation or information are expressly subject \n-- to the terms and conditions of the Altera Program License \n-- Subscription Agreement, Altera MegaCore Function License \n-- Agreement, or other applicable license agreement, including, \n-- without limitation, that your use is for the sole purpose of \n-- programming logic devices manufactured by Altera and sold by \n-- Altera or its authorized distributors.  Please refer to the \n-- applicable agreement for further details."
puts $NElemXfer_rom_mif "-- Quartus II generated Memory Initialization File (.mif)\n"
puts $NElemXfer_rom_mif "WIDTH=8;"
puts $NElemXfer_rom_mif "DEPTH=1024;\n"
puts $NElemXfer_rom_mif "ADDRESS_RADIX=UNS;"
puts $NElemXfer_rom_mif "DATA_RADIX=UNS;\n"
puts $NElemXfer_rom_mif "CONTENT BEGIN"

for {set addr 0} {$addr <= [expr {2**($localstride_nbit+$maxNElemXfer_nbit)-1}]} {incr addr} {
    puts $NElemXfer_rom_mif "\t$addr\t:\t[lindex $NElemXfer_rom $addr];"
}
puts $NElemXfer_rom_mif "END;"
close $NElemXfer_rom_mif
puts "$NElemXfer_rom_mif_name written!"


# Write NElemXfer_rom.dat
set NElemXfer_rom_dat [open $NElemXfer_rom_dat_name w 0666]

puts $NElemXfer_rom_dat "@00000000"

for {set addr 0} {$addr <= [expr {2**($localstride_nbit+$maxNElemXfer_nbit)-1}]} {incr addr} {
    puts $NElemXfer_rom_dat "[string toupper [format %02x [lindex $NElemXfer_rom $addr]]]"
}
close $NElemXfer_rom_dat
puts "$NElemXfer_rom_dat_name written!"



#-------------------------------------------------------
# Write NElemRemainder_rom.mif/.dat
#-------------------------------------------------------
set NElemRemainder_rom_mif [open $NElemRemainder_rom_mif_name w 0666]
puts $NElemRemainder_rom_mif "-- Copyright (C) 1991-2007 Altera Corporation,\n -- Your use of Altera Corporations design tools, logic functions \n-- and other software and tools, and its AMPP partner logic \n-- functions, and any output files from any of the foregoing \n-- (including device programming or simulation files), and any \n-- associated documentation or information are expressly subject \n-- to the terms and conditions of the Altera Program License \n-- Subscription Agreement, Altera MegaCore Function License \n-- Agreement, or other applicable license agreement, including, \n-- without limitation, that your use is for the sole purpose of \n-- programming logic devices manufactured by Altera and sold by \n-- Altera or its authorized distributors.  Please refer to the \n-- applicable agreement for further details."
puts $NElemRemainder_rom_mif "-- Quartus II generated Memory Initialization File (.mif)\n"
puts $NElemRemainder_rom_mif "WIDTH=8;"
puts $NElemRemainder_rom_mif "DEPTH=1024;\n"
puts $NElemRemainder_rom_mif "ADDRESS_RADIX=UNS;"
puts $NElemRemainder_rom_mif "DATA_RADIX=UNS;\n"
puts $NElemRemainder_rom_mif "CONTENT BEGIN"

for {set addr 0} {$addr <= [expr {2**($localstride_nbit+$maxNElemXfer_nbit)-1}]} {incr addr} {
    puts $NElemRemainder_rom_mif "\t$addr\t:\t[lindex $NElemRemainder_rom $addr];"
}
puts $NElemRemainder_rom_mif "END;"
close $NElemRemainder_rom_mif
puts "$NElemRemainder_rom_mif_name written!"


# Write NElemRemainder_rom.dat
set NElemRemainder_rom_dat [open $NElemRemainder_rom_dat_name w 0666]
puts $NElemRemainder_rom_dat "@00000000"

for {set addr 0} {$addr <= [expr {2**($localstride_nbit+$maxNElemXfer_nbit)-1}]} {incr addr} {
    puts $NElemRemainder_rom_dat "[string toupper [format %02x [lindex $NElemRemainder_rom $addr]]]"
}
close $NElemRemainder_rom_dat
puts "$NElemRemainder_rom_dat_name written!"



#-------------------------------------------------------
# Write Verilog files for ROM initial contents to include in code
#-------------------------------------------------------
set	NElemRemainder_rom_include [open $NElemRemainder_rom_include_name w 0666]

set NElemRemainder_rom_addrwidth [expr {2*[mylog2 $memwidth_bytes]+2}]
set NElemRemainder_rom_addrwidth_h [expr {int(ceil(double($NElemRemainder_rom_addrwidth)/4))}]
set NElemRemainder_rom_datawidth [expr {[mylog2 $memwidth_bytes]+1}]
set NElemRemainder_rom_datawidth_h [expr {int(ceil(double($NElemRemainder_rom_datawidth)/4))}]
set hex_format "x"
set	addrformat "%0$NElemRemainder_rom_addrwidth_h$hex_format"
set dataformat "%0$NElemRemainder_rom_datawidth_h$hex_format"

for {set addr 0} {$addr <= [expr {2**($localstride_nbit+$maxNElemXfer_nbit)-1}]} {incr addr} {
	puts $NElemRemainder_rom_include "$NElemRemainder_rom_addrwidth'h[format $addrformat $addr]: NElemRemainder_mem_q <= $NElemRemainder_rom_datawidth'h[format $dataformat [lindex $NElemRemainder_rom $addr]];"
}
close $NElemRemainder_rom_include
puts "$NElemRemainder_rom_include_name written!"



set	NElemXfer_rom_include [open $NElemXfer_rom_include_name w 0666]

for {set addr 0} {$addr <= [expr {2**($localstride_nbit+$maxNElemXfer_nbit)-1}]} {incr addr} {
	puts $NElemXfer_rom_include "$NElemRemainder_rom_addrwidth'h[format $addrformat $addr]: NElemXfer_sig_mem_q <= $NElemRemainder_rom_datawidth'h[format $dataformat [lindex $NElemXfer_rom $addr]];"
}
close $NElemXfer_rom_include
puts "$NElemXfer_rom_include_name written!"



#-------------------------------------------------------
# Generate read-only control register contents
# controlreg_other.mif
#-------------------------------------------------------

write_mif_file 32 48 $controlreg_mif_name controlreg_array
puts "$controlreg_mif_name written!"


#-------------------------------------------------------
# Generate read-only control register contents
#-------------------------------------------------------

write_dat_file $controlreg_dat_start_addr_hex 48 $controlreg_dat_name controlreg_array
write_hex_file 48 $controlreg_hex_name controlreg_array
puts "$controlreg_dat_name written!"
puts "$controlreg_hex_name written!"


#-------------------------------------------------------
# Generate config_def_auto.v definitions
#-------------------------------------------------------
set config_def [open $config_def_name w 0666]

puts $config_def "// Scalable Vector CPU Project -- Additional configuration definitions"
puts $config_def "// Auto generated by configuration Tcl script"
puts $config_def "// Copyright (c) 2007 Jason Yu\n"

# macl
puts $config_def "// MACCHAINLENGTH * MAC_NUMMULT cannot be greater than NUMLANES"
puts $config_def "`define \tMACCHAINLENGTH \t$macl\n"


# write definitions for different memory access widths
if {[expr {$vpu_width >= 8}] && [expr {$mindatawidth <= 8}]} {
	puts $config_def "`define \tLOAD_BYTE_ACCESS_ENABLED"
	puts $config_def "`define \tSTORE_BYTE_ACCESS_ENABLED"
}
if {[expr {$vpu_width >= 16}] && [expr {$mindatawidth <= 16}]} {
	puts $config_def "`define \tLOAD_HALFWORD_ACCESS_ENABLED"
	puts $config_def "`define \tSTORE_HALFWORD_ACCESS_ENABLED"
}
if {[expr {$vpu_width >= 32}] && [expr {$mindatawidth <= 32}]} {
	puts $config_def "`define \tLOAD_WORD_ACCESS_ENABLED"
	puts $config_def "`define \tSTORE_WORD_ACCESS_ENABLED"
}

puts "$config_def_name written!"


#-------------------------------------------------------------------------
# Generate a sample initialization file for data memory
#
# 128bit:
# 127 ...         63 ...         0
# 77776666555544443333222211110000
# ffffeeeeddddccccbbbbaaaa99998888
# 00001111222233334444555566667777
# 88889999aaaabbbbccccddddeeeeffff
#-------------------------------------------------------------------------
set dmem_init [open $dmem_init_name w 0666]
puts $dmem_init "// format=hex addressradix=h dataradix=h version=1.0 wordsperline=1"

set forward 0
set start 0
set increment [expr {$mem_width/16}]
set end $increment
set value $start

for {set word 0} {$word < $dmem_numword} {incr word} {

	# clear variable for new line
	set line {}

	# alternate counting backwards and forwards
		for {set value $start} {$value < $end} {incr value} {
			# if counting backwards
			if {$forward == 0} {
				set char [format %1x $value]
			} else {
				set char [format %1x [expr {15-$value}]]
			}
			# repeat hex character to 16-bit
			# append to left hand side
			set line [concat [string repeat $char 4] $line]
		}
		
		# if finished counting one way
		if {$end == 16} {
			set start 0
			set end $increment
			# flip to other direction
			set forward [expr {!$forward}]
		# other cases
		} else {
			set start $end
			set end [expr {$increment+$end}]
		}

	# trim spaces from line
	set line [string map {" " ""} $line]
	#puts $line
	puts $dmem_init $line
}

puts "$dmem_init_name generated."


#-------------------------------------------------------
# Write parameters into Verilog testbench
#-------------------------------------------------------
set	tb_template [open $tb_template_name r]
set	gen_testbench [open $tb_name w 0666]

# text to insert into testbench
set tb_insert_text "\tparameter\tVPU_WIDTH\t\t\t= $vpu_width,
	parameter\tNUMLANES\t\t\t= $numlanes,
	parameter\tMEM_WIDTH\t\t\t= $mem_width,
	parameter\tMINDATAWIDTH\t\t= $mindatawidth"

while { [gets $tb_template line] >= 0 } {
    if {[string equal $line $replace_symbol]} {
		puts $gen_testbench $tb_insert_text
	} else {
		puts $gen_testbench $line
	}
}

close $tb_template
close $gen_testbench
puts "$tb_name generated!"


#-------------------------------------------------------
# Configuration summary
#-------------------------------------------------------
puts "\n\n-------------------------"
puts "- Configured parameters -"
puts "-------------------------"
puts "NUMLANES\t$numlanes"
puts "MVL\t\t[expr {$numlanes*4}]"
puts "VPUW\t\t$vpu_width"
puts "MEM_WIDTH\t$mem_width"
puts "MINDATAWIDTH\t$mindatawidth"
puts "MACL\t\t$macl"