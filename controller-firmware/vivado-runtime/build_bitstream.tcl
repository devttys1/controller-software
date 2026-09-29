# Create the controller_firmware project from the saved project script and build bitfile.bit.bin.
# Run by build_bitstream.sh: vivado -mode batch -source build_bitstream.tcl -tclargs <work dir> [jobs]
#
# The work directory holds the contents of controller-firmware/Vivado, the generated
# autogen_sources/controller.v, and controller_firmware_linux.tcl (the saved project
# script with its Windows paths replaced by build_bitstream.sh).

set work [file normalize [lindex $argv 0]]
set jobs [expr {[llength $argv] > 1 ? [lindex $argv 1] : 8}]
cd $work

# fresh project every build, so nothing from a previous build leaks in
file delete -force controller_firmware
source controller_firmware_linux.tcl

# the saved project runs "export bit.bin.tcl" after write_bitstream, which uses Windows paths,
# so bootgen runs at the end of this script instead
set_property STEPS.WRITE_BITSTREAM.TCL.POST "" [get_runs impl_1]

# the controller's interrupts (update running/done, DMA cycle running/done) must all reach the PS,
# otherwise the software waits forever for "update done". IRQ_F2P's width is derived during block design
# validation, which a fresh project skips because the saved design is marked as validated, leaving it at
# 1 bit; force the validation so it follows the 16-bit Controller_0/pl_ps_interrupts, then check it
open_bd_design [get_files controller_firmware.bd]
validate_bd_design -force
save_bd_design
set irq_width [expr {[get_property LEFT [get_bd_pins processing_system7_0/IRQ_F2P]] + 1}]
puts "BUILD RESULT: IRQ_F2P width $irq_width"
if {$irq_width < 4} {
    error "processing_system7_0/IRQ_F2P is only $irq_width bit(s) wide after validation"
}

# optional synthesis strategy override (SYNTH_STRATEGY), e.g. Flow_PerfOptimized_high
if {[info exists ::env(SYNTH_STRATEGY)] && $::env(SYNTH_STRATEGY) ne ""} {
    set_property strategy $::env(SYNTH_STRATEGY) [get_runs synth_1]
    puts "BUILD RESULT: synth_1 strategy $::env(SYNTH_STRATEGY)"
}

update_compile_order -fileset sources_1
launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1

if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    error "implementation failed, see $work/controller_firmware/controller_firmware.runs/impl_1/runme.log"
}

open_run impl_1
report_timing_summary -file timing_summary.rpt
report_utilization -file utilization.rpt
set wns [get_property STATS.WNS [get_runs impl_1]]
set whs [get_property STATS.WHS [get_runs impl_1]]
puts "BUILD RESULT: WNS $wns ns, WHS $whs ns"
if {$wns < 0 || $whs < 0} {
    puts "BUILD WARNING: timing not met, see timing_summary.rpt"
}

file copy -force controller_firmware/controller_firmware.runs/impl_1/controller_firmware_top.bit bitfile.bit
set bif [open convert_linux.bif w]
puts $bif "the_ROM_image:\n{\n    [file normalize bitfile.bit]\n}"
close $bif
exec bootgen -w -arch zynq -image convert_linux.bif -process_bitstream bin
puts "BUILD RESULT: $work/bitfile.bit.bin"
