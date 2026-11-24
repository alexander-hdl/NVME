# power_table.tcl - offline FPGA power table for the recorder (STAND/RECORDER_SPEC.md 8.4, WP9)
#
# Vectorless estimate from the routed design: for each mode and junction temperature,
# set switching activity on the data-path nets (proportional to the data rate), run
# report_power and collect the result into fpga/power/power_table.csv.
# The PC tool (tools/recorder.py) interpolates this table with md, stp and XADC Tj.
#
# Usage:
#   vivado -mode batch -source fpga/scripts/power_table.tcl -tclargs <routed.dcp> [out.csv]
# Environment overrides (optional):
#   PT_DATA_NETS   Tcl list of hierarchical net patterns of the data path
#   PT_CTRL_TOGGLE default toggle rate (%) of all other logic (control, clocks excluded)
#
# Output columns: mode,rate_Bps,tj_C,p_static_W,p_dyn_W,p_vccint_W,p_vccaux_W,p_vcco_W,p_mgt_W
#   p_static_W/p_dyn_W: "Device Static (W)" / "Dynamic (W)" of report_power (their sum is the
#   total on-chip power); p_v*_W: rail power = V x total current from the supply summary.
# NOTE: vectorless estimate, accuracy about +-30 %; not a measurement.

set dcp [lindex $argv 0]
if {$dcp eq ""} { puts "usage: vivado -mode batch -source power_table.tcl -tclargs <routed.dcp> \[out.csv\]"; exit 1 }
set script_dir [file dirname [file normalize [info script]]]
set out_csv [expr {[llength $argv] > 1 ? [lindex $argv 1] : [file join $script_dir .. power power_table.csv]}]
file mkdir [file dirname $out_csv]

# --- configuration ---------------------------------------------------------
# (mode  rate_Bps)  -- rate 0 for modes without data flow
set points {
    {IDLE 0}
    {MON  0}
    {REC  100e6} {REC 280e6} {REC 400e6} {REC 800e6}
    {READ 887e6}
}
set tj_list {25 50 70 85}
# 16 B per ui_clk (100 MHz) = 1.6 GB/s is the maximum stream (NCO step 2^32)
set rate_max 1.6e9
# bit-toggle probability of random data per clock at the maximum stream: 50 %
set tr_data_max 50.0
set data_nets {*u_rec_framer *u_ddr_fifo *u_data_ring *u_rec_verify *u_vrf_tap}
if {[info exists ::env(PT_DATA_NETS)]} { set data_nets $::env(PT_DATA_NETS) }
set tr_ctrl 2.0
if {[info exists ::env(PT_CTRL_TOGGLE)]} { set tr_ctrl $::env(PT_CTRL_TOGGLE) }

# --- helpers ----------------------------------------------------------------
proc parse_power {rpt} {
    # returns dict: static dyn vccint vccaux vcco mgt   (W)
    set fh [open $rpt r]; set txt [read $fh]; close $fh
    set d [dict create static 0.0 dyn 0.0 vccint 0.0 vccaux 0.0 vcco 0.0 mgt 0.0]
    foreach line [split $txt "\n"] {
        if {![string match "|*" $line]} continue
        set cols [lmap c [split [string trim $line "| "] "|"] {string trim $c}]
        set name [lindex $cols 0]
        if {$name eq "Device Static (W)"} { dict set d static [lindex $cols 1]; continue }
        if {$name eq "Dynamic (W)"}       { dict set d dyn    [lindex $cols 1]; continue }
        # supply summary row: Source | Voltage (V) | Total (A) | ...
        if {[llength $cols] >= 3 && [string is double -strict [lindex $cols 1]] && \
            [string is double -strict [lindex $cols 2]]} {
            set p [expr {[lindex $cols 1] * [lindex $cols 2]}]
            switch -glob -nocase $name {
                Vccint            { dict set d vccint [expr {[dict get $d vccint] + $p}] }
                Vccaux            { dict set d vccaux [expr {[dict get $d vccaux] + $p}] }
                Vccaux_io         { dict set d vccaux [expr {[dict get $d vccaux] + $p}] }
                Vcco*             { dict set d vcco   [expr {[dict get $d vcco] + $p}] }
                MGT*              { dict set d mgt    [expr {[dict get $d mgt] + $p}] }
            }
        }
    }
    return $d
}

# --- run ---------------------------------------------------------------------
open_checkpoint $dcp
set_param tcl.collectionResultDisplayLimit 0   ;# data-path net lists have thousands of nets
set tmp [file join [file dirname [file normalize $out_csv]] _power_tmp.rpt]

set data_objs {}
foreach pat $data_nets {
    # match nets INSIDE the instance (full hierarchical name), not top-level nets named after it
    set n [get_nets -hierarchical -quiet -filter "NAME =~ ${pat}/*"]
    if {[llength $n] == 0} { puts "WARNING: no nets match data pattern '$pat'" }
    lappend data_objs {*}$n
}
if {[llength $data_objs] == 0} { puts "WARNING: no data-path nets found; rate will not influence the estimate" }

set ofh [open $out_csv w]
puts $ofh "mode,rate_Bps,tj_C,p_static_W,p_dyn_W,p_vccint_W,p_vccaux_W,p_vcco_W,p_mgt_W"
foreach pt $points {
    lassign $pt mode rate
    set rate [expr {double($rate)}]
    reset_switching_activity -all
    set_switching_activity -default_toggle_rate $tr_ctrl -default_static_probability 0.5
    if {[llength $data_objs]} {
        set tr [expr {$tr_data_max * $rate / $rate_max}]
        if {$tr > 0.0} {
            set_switching_activity -toggle_rate $tr -static_probability 0.5 $data_objs
        } else {
            set_switching_activity -toggle_rate 0.0 -static_probability 0.5 $data_objs
        }
    }
    foreach tj $tj_list {
        set_operating_conditions -junction_temp $tj
        report_power -file $tmp
        set r [parse_power $tmp]
        puts $ofh [format "%s,%.0f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f" $mode $rate $tj \
            [dict get $r static] [dict get $r dyn] [dict get $r vccint] [dict get $r vccaux] \
            [dict get $r vcco] [dict get $r mgt]]
        flush $ofh
    }
}
close $ofh
file delete -force $tmp
puts "power table written: $out_csv"
close_design
