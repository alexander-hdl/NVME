# =============================================================================
# run_mig_sanity.tcl - optional sanity simulation of the generated MIG `mig_ddr3` with the Micron
#                      DDR3 model that ships with the MIG example design.  NOT part of the regression.
#   Spec: STAND/RECORDER_SPEC.md 5.3 / 9.2 (WP4).
#
#   1. fpga/scripts/mig_ip.tcl must have been run (standalone) so that fpga/build/mig_check/ip exists
#      (this script runs it if the generated IP is missing);
#   2. the example-design testbench sim_tb_top (traffic generator + ddr3_model x16, SIM_BYPASS_INIT_CAL=FAST,
#      400 MHz) is compiled and run with xsim: calibration, then ~50 us of write/read traffic with
#      data compare.  PASS = "Calibration Done" and "TEST PASSED" in the log.
#
#   vivado -mode batch -source sim/scripts/run_mig_sanity.tcl -log fpga/build/mig_check/sanity.log \
#          -journal fpga/build/mig_check/sanity.jou
# Output: sim/work/mig_sanity/ (log: xsim.log).  Exit code 0 = passed, 1 = failed.
# =============================================================================
set repo [file normalize [file join [file dirname [file normalize [info script]]] .. ..]]
set gen  [file join $repo fpga build mig_check ip mig_ddr3 mig_ddr3]
set exd  [file join $gen example_design]

if {![file exists [file join $exd sim sim_tb_top.v]]} {
    puts "INFO: generated MIG not found, running fpga/scripts/mig_ip.tcl first"
    # mig_ip.tcl exits the process when standalone: run it in a child Vivado
    exec cmd /c vivado -mode batch -source [file join $repo fpga scripts mig_ip.tcl] \
        -log [file join $repo fpga build mig_check mig_check.log] -journal [file join $repo fpga build mig_check mig_check.jou] >@stdout
}

set work [file join $repo sim work mig_sanity]
file delete -force $work
file mkdir $work
foreach f [glob -directory [file join $exd sim] *] { if {[file isfile $f]} { file copy -force $f $work } }
file copy -force [file join $exd rtl example_top.v] $work
foreach f [glob -directory [file join $exd rtl traffic_gen] *.v] { file copy -force $f $work }

# the .prj lists files of the example *project* layout: point it to the generated user_design tree
set fh [open [file join $work xsim_files.prj] r]; set prj [read $fh]; close $fh
set prj [string map [list "../mig_ddr3_ex.srcs/sources_1/ip/mig_ddr3/mig_ddr3/user_design/rtl" [file join $gen user_design rtl] \
                          {$XILINX_VIVADO} $::env(XILINX_VIVADO)] $prj]
set fh [open [file join $work xsim_files.prj] w]; puts -nonewline $fh $prj; close $fh

# run to the end of the testbench ($finish)
set fh [open [file join $work run_all.tcl] w]; puts $fh "run all\nquit"; close $fh

cd $work
puts "INFO: xelab ..."
exec cmd /c xelab work.sim_tb_top work.glbl -prj xsim_files.prj -L unisims_ver -L secureip -s xsim_test -debug off \
    -timescale 1ps/1ps >@stdout 2>@stderr
puts "INFO: xsim ... (a few minutes)"
exec cmd /c xsim xsim_test -t run_all.tcl -log xsim.log >@stdout 2>@stderr

set fh [open xsim.log r]; set log [read $fh]; close $fh
set ok [expr {[string first "Calibration Done" $log] >= 0 && [string first "TEST PASSED" $log] >= 0}]
if {$ok} { puts "MIG SANITY PASSED (Calibration Done, TEST PASSED)"; exit 0 }
puts "MIG SANITY FAILED (see $work/xsim.log)"
exit 1
