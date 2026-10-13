# =============================================================================
# build.tcl - batch build of nvme_host_top (synthesis, optional implementation + bitstream)
#
#   vivado -mode batch -source fpga/scripts/build.tcl -log fpga/build/build.log -journal fpga/build/build.jou \
#          -tclargs <stage> [jobs=N] [recreate]
#
#   stage   synth  (default) : IP generation + out-of-context IP synthesis + synth_1,
#                              reports in fpga/build/reports/
#           impl             : synth + impl_1 up to route_design (post-route reports)
#           bit              : synth + impl + write_bitstream (fpga/build/nvme_host_top.bit)
#   jobs=N  parallel runs (default 4)
#   recreate  : always re-run create_project.tcl (default: re-use fpga/build/nvme_host/nvme_host.xpr)
#   loose_io  : (bit only) demote the DRC checks NSTD-1 / UCIO-1 (unconstrained I/O standard / pin) to
#               warnings so that a bitstream can be written while the user-side ports (data stream,
#               control/status) still have no package pins.  Do NOT use this for a real board image.
#
# The project is created by fpga/scripts/create_project.tcl when it does not exist yet.
# NOTE (Vivado 2022.2 on this machine): the project flow (launch_runs) is used because the
# pcie_7x_0 IP is synthesised out-of-context in its own run and its XDC / DCP are picked up
# automatically.  Everything is written below fpga/build/ (git-ignored).
# =============================================================================

set script_dir [file dirname [file normalize [info script]]]
set repo_root  [file normalize [file join $script_dir .. ..]]
set build_dir  [file join $repo_root fpga build]
set xpr        [file join $build_dir nvme_host nvme_host.xpr]
set rpt_dir    [file join $build_dir reports]

set stage    synth
set jobs     4
set recreate 0
set loose_io 0
foreach a $argv {
    switch -glob -- $a {
        synth - impl - bit { set stage $a }
        jobs=*             { set jobs [string range $a 5 end] }
        recreate           { set recreate 1 }
        loose_io           { set loose_io 1 }
        default            { puts "WARNING: ignoring unknown argument '$a'" }
    }
}
file mkdir $rpt_dir

if {$recreate || ![file exists $xpr]} {
    source [file join $script_dir create_project.tcl]
} else {
    open_project $xpr
}

proc check_run {run} {
    set st [get_property STATUS [get_runs $run]]
    puts "INFO: run $run status: $st"
    if {![string match "*Complete*" $st] || [string match "*ERROR*" $st]} {
        error "run $run did not complete successfully ($st)"
    }
}

# ---------------------------------------------------------------------------
# synthesis (IP out-of-context run first, then the top)
# ---------------------------------------------------------------------------
reset_run synth_1
set ip_runs [get_runs -quiet *pcie_7x_0*synth*]
if {[llength $ip_runs]} {
    reset_run [lindex $ip_runs 0]
    launch_runs $ip_runs -jobs $jobs
    foreach r $ip_runs { wait_on_run $r; check_run $r }
}
launch_runs synth_1 -jobs $jobs
wait_on_run synth_1
check_run synth_1

open_run synth_1 -name synth_1
report_utilization    -file [file join $rpt_dir synth_utilization.rpt]
report_utilization -hierarchical -file [file join $rpt_dir synth_utilization_hier.rpt]
report_timing_summary -delay_type min_max -max_paths 10 -file [file join $rpt_dir synth_timing_summary.rpt]
report_clock_networks -file [file join $rpt_dir synth_clock_networks.rpt]
report_methodology    -file [file join $rpt_dir synth_methodology.rpt]
write_checkpoint -force [file join $build_dir nvme_host_top_synth.dcp]
puts "INFO: synthesis finished, reports in $rpt_dir"
if {$stage eq "synth"} { exit 0 }

# ---------------------------------------------------------------------------
# implementation / bitstream
# ---------------------------------------------------------------------------
close_design
if {$stage eq "bit"} { set last write_bitstream } else { set last route_design }
reset_run impl_1
if {$loose_io} {
    set_property STEPS.WRITE_BITSTREAM.TCL.PRE [file join $script_dir bit_pre_loose_io.tcl] [get_runs impl_1]
} else {
    set_property STEPS.WRITE_BITSTREAM.TCL.PRE {} [get_runs impl_1]
}
launch_runs impl_1 -to_step $last -jobs $jobs
wait_on_run impl_1
check_run impl_1

open_run impl_1 -name impl_1
report_utilization    -file [file join $rpt_dir route_utilization.rpt]
report_timing_summary -delay_type min_max -max_paths 10 -report_unconstrained -file [file join $rpt_dir route_timing_summary.rpt]
report_clock_interaction -delay_type min_max -file [file join $rpt_dir route_clock_interaction.rpt]
report_drc            -file [file join $rpt_dir route_drc.rpt]
report_power          -file [file join $rpt_dir route_power.rpt]
write_checkpoint -force [file join $build_dir nvme_host_top_routed.dcp]
if {$stage eq "bit"} {
    set bit [glob -nocomplain [file join $build_dir nvme_host nvme_host.runs impl_1 *.bit]]
    if {[llength $bit]} { file copy -force [lindex $bit 0] [file join $build_dir nvme_host_top.bit] }
}
puts "INFO: $stage finished, reports in $rpt_dir"
exit 0
