#-------------------------------------------------------------------------
# Tcl script to parse state dump files and generate a more readable file
#
# Generates the following files:
# ./dump/state_dump.txt
#
# Date created: Jan 14, 2008
#
# Copyright (c) 2008 Jason Yu
#-------------------------------------------------------------------------

set vrf_dump_path "./dump/"
set vrf_dump_name "./dump/vrf_dump_lane"
set vflag_dump_name "./dump/vflag_dump_lane"
set scalar_dump_name "./dump/scalarreg_dump.hex"
set dump_ext ".hex"
set vrf_output_name "vrf_dump_summary.txt"
set elemperlane 4
set numvreg 64
set numvflag 32


#-------------------------------------------------------------------------
# Check command line parameters
#-------------------------------------------------------------------------

#-------------------------------------------------------------------------
# Dump scalar register contents
#-------------------------------------------------------------------------
set filehandle [open $scalar_dump_name r]
set vrf_output [open $vrf_output_name w 0666]

set scalar_data {}

# Parse the file
while { [gets $filehandle line] >= 0 } {
	set matched_data {}
	
	# line of data looks like
	# @0 00000001 00000001 00000001 00000001 00000001 00000001 00000001 00006666
	# match beginning @number
	if { [regexp {^\s*\@[\da-fA-F]+\s+(.+)} $line match matched_data] == 1 } {
		# match each hex number, including 'x' and 'z'
		while { [regexp {^([\da-fA-Fzx]+)\s*(.*)} $matched_data match data_value remainder] == 1 } {
			#set vrf_lane_data [concat $vrf_lane_data $data_value]
			#puts $matched_data
			lappend scalar_data $data_value
			set matched_data $remainder
		}
	}
}

puts $vrf_output "Scalar registers:\n$scalar_data"
close $filehandle
	



#-------------------------------------------------------------------------
# Parse vector register input files
#-------------------------------------------------------------------------
# list of sorted files
set vrf_dump_files [lsort [eval glob $vrf_dump_name*$dump_ext]]
puts $vrf_dump_files
set numfiles [llength $vrf_dump_files]
puts "Number of dump files = $numfiles"

# List that holds all data, its elements are lists of contents of each lane VRF
set vrf_data_list {}
#set vrf_data_array {}
set match {}
set remainder {}
set lanecount 0

# Parse each file
foreach filename $vrf_dump_files {
	# set filehandle [open $filename r]
	set filehandle [open $vrf_dump_name$lanecount$dump_ext r]
	puts "Reading: $vrf_dump_name$lanecount$dump_ext"
	set vrf_lane_data {}
	
	# Parse the file
	while { [gets $filehandle line] >= 0 } {
		set matched_data {}
		
		# if not line of comment
		#if { [regexp {^\/\/} $match] == 0}

		# line of data looks like
		# @0 00000001 00000001 00000001 00000001 00000001 00000001 00000001 00006666
		# match beginning @number
		if { [regexp {^\s*\@[\da-fA-F]+\s+(.+)} $line match matched_data] == 1 } {
			# match each hex number, including 'x' and 'z'
			while { [regexp {^([\da-fA-Fzx]+)\s*(.*)} $matched_data match data_value remainder] == 1 } {
				#set vrf_lane_data [concat $vrf_lane_data $data_value]
				#puts $matched_data
				lappend vrf_lane_data $data_value
				set matched_data $remainder
			}
		}
	}
	close $filehandle
	
	#puts $vrf_lane_data
	puts "vlane$lanecount number of elements = [llength $vrf_lane_data]"
	set vrf_data_array($lanecount) $vrf_lane_data
	incr lanecount
}

#puts "lanecount=$lanecount"


#-------------------------------------------------------------------------
# Parse vector flag dump files
#-------------------------------------------------------------------------
# list of sorted files
set vflag_dump_files [lsort [eval glob $vflag_dump_name*$dump_ext]]
puts $vflag_dump_files
puts "Number of dump files = [llength $vflag_dump_files]"

# List that holds all data, its elements are lists of contents of each lane VRF
set vflag_data_list {}
set match {}
set remainder {}
set lanecount 0

# Parse each file
foreach filename $vflag_dump_files {
	set filehandle [open $vflag_dump_name$lanecount$dump_ext r]
	#set filehandle [open $filename r]
	set vflag_lane_data {}
	
	# Parse the file
	while { [gets $filehandle line] >= 0 } {
		set matched_data {}
		
		# if not line of comment
		#if { [regexp {^\/\/} $match] == 0}

		# line of data looks like
		# @0 00000001 00000001 00000001 00000001 00000001 00000001 00000001 00006666
		# match beginning @number
		if { [regexp {^\s*\@[\da-fA-F]+\s+(.+)} $line match matched_data] == 1 } {
			# match each binary number, including 'x' and 'z'
			while { [regexp {^([\dzx])\s*(.*)} $matched_data match data_value remainder] == 1 } {
				#set vrf_lane_data [concat $vrf_lane_data $data_value]
				#puts $matched_data
				lappend vflag_lane_data $data_value
				set matched_data $remainder
			}
		}
	}
	close $filehandle
	
	#puts $vflag_lane_data
	puts "vflag$lanecount number of elements = [llength $vflag_lane_data]"
	set vflag_data_array($lanecount) $vflag_lane_data
	incr lanecount
}

#puts "lanecount=$lanecount"


#-------------------------------------------------------------------------
# Generate a cleaner output file; group contents of each vector register
#-------------------------------------------------------------------------
#set numlanes [array size vrf_data_array]	# don't know why Modelsim Tcl has a different size
set numlanes $lanecount
#puts [array names vrf_data_array]
puts "numlanes=$numlanes"

# iterate through lanes

set vreg_data {}
set vflag_data {}
set vreg 0

# iterate through vector registers
for {set vreg 0} {$vreg < $numvreg} {incr vreg} {
	# clear data
	set vreg_data {}
	
	# iterate through elemperlane
	for {set iter 0} {$iter < $elemperlane} {incr iter} {
		# iterate through lanes
		for {set lane 0} {$lane < $numlanes} {incr lane} {
			lappend vreg_data [lindex $vrf_data_array($lane) [expr {$vreg+$iter*$numvreg}]]
		}
	}
	#puts "[llength $vreg_data]"
	set vreg_grouped_data($vreg) $vreg_data
}

# iterate through vector flag registers
for {set vreg 0} {$vreg < $numvflag} {incr vreg} {
	# clear data
	set vflag_data {}
	
	# iterate through elemperlane
	for {set iter 0} {$iter < $elemperlane} {incr iter} {
		# iterate through lanes
		for {set lane 0} {$lane < $numlanes} {incr lane} {
			lappend vflag_data [lindex $vflag_data_array($lane) [expr {$vreg+$iter*$numvflag}]]
		}
	}
	#puts "[llength $vreg_data]"
	set vflag_grouped_data($vreg) $vflag_data
}


# Dump data to output file

# dump vector registers
for {set vreg 0} {$vreg < $numvreg} {incr vreg} {
	puts $vrf_output "\nVreg $vreg:"
	puts $vrf_output $vreg_grouped_data($vreg)
}

# dump vector flag registers
for {set vreg 0} {$vreg < $numvflag} {incr vreg} {
	puts $vrf_output "\nVflagreg $vreg:"
	puts $vrf_output $vflag_grouped_data($vreg)
}


close $vrf_output
puts "Generated $vrf_output_name"


#-------------------------------------------------------------------------
# delete dump files if "--nodelete" not specified
#-------------------------------------------------------------------------
#if {[expr {$argc != 1} || {[string compare [lindex $argv 0] "--nodelete"] != 0}]} {
	#puts "\nDeleting dump files... specify --nodelete to preserve dump files"
	#puts "Be careful with --nodelete, script scans entire dump directory for files"
	#exec ./rm $vrf_dump_path*
#}
