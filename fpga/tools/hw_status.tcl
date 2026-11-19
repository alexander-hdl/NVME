# =============================================================================
# hw_status.tcl - AX7A200B stand over JTAG (Vivado Hardware Manager, batch mode):
# program the bitstream, read the VIO status of ax7a200b_top, press the remote buttons.
#
#   vivado -mode batch -nojournal -nolog -source fpga/tools/hw_status.tcl -tclargs <cmd> [<cmd> ...]
#     program        load fpga/build/ax7a200b_top.bit (+ .ltx), volatile (JTAG only, flash untouched)
#     status         read and decode all VIO inputs once
#     watch=<s>      read every second for <s> seconds (one line each)
#     reset          pulse the VIO board reset (new PERST#/bring-up cycle, clears START/STOP)
#     start | stop   pulse the VIO START / STOP button.  START runs enum + NVMe init + RUN; data is written to the SSD
#                    (from LBA 0) ONLY if the generator is enabled (gen_on); with gen_off (default) RUN writes nothing
#     capture        arm the ILA on the first TX TLP, press START, save fpga/build/ila_capture.csv
#     capture_enum   same, trigger = sequencer enters ENUM
#     rate_bps=<R>   real-time source of R bytes/s (backlog = memory needed beyond the ring); 0 = off
#     len_gib=<G> | len_cmds=<N>  run length (overrides len=); 0 = back to len=
#     until=<s>      poll every s seconds until done / error, print the rate of each interval (PROFILE:)
#     len=<0..3>     generator length: 0 1 GiB (default), 1 8 GiB, 2 64 GiB, 3 unlimited (set BEFORE start)
#     rate=<0..3>    generator rate: 0 full (default), 1 1/2, 2 1/4, 3 1/8
#     verify_on | verify_off  verify run: START reads len= GiB back with NVMe Read and compares it with the pattern
#                    (results: VERIFY: in the status line); recording: verify_off
#     format_on | format_off  NVMe Format NVM (SES=0, all data lost) during the bring-up of the next START
#     gen_on | gen_off  generator enable (VIO probe_out3, 0 after programming)
#     probes         list the VIO probes with their properties (debugging this script)
#     bitname=<name> (first argument only) use fpga/build/<name>.bit/.ltx
#   Commands run in the given order, e.g.  -tclargs program watch=5 status
# =============================================================================
set repo_root [file normalize [file join [file dirname [info script]] .. ..]]
# optional first argument bitname=<name>: use fpga/build/<name>.bit/.ltx instead of ax7a200b_top
set bitname ax7a200b_top
if {[llength $argv] > 0 && [string match bitname=* [lindex $argv 0]]} {
    set bitname [string range [lindex $argv 0] 8 end]; set argv [lrange $argv 1 end]
} elseif {[llength $argv] > 1 && [lindex $argv 0] eq "bitname"} {   ;# vivado.bat splits key=value
    set bitname [lindex $argv 1]; set argv [lrange $argv 2 end]
}
set bit [file join $repo_root fpga build $bitname.bit]
set ltx [file join $repo_root fpga build $bitname.ltx]

open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target [lindex [get_hw_targets] 0]
set dev [lindex [get_hw_devices xc7a200t*] 0]
current_hw_device $dev

proc load_probes {} {
    global dev ltx
    set_property PROBES.FILE $ltx $dev
    set_property FULL_PROBES.FILE $ltx $dev
    refresh_hw_device $dev
}

proc vio {} { return [lindex [get_hw_vios -of_objects [current_hw_device]] 0] }
# probe value by net name (names in the .ltx come from the nets connected to u_vio)
proc rv {name} {
    set p [get_hw_probes $name -of_objects [vio]]
    set_property INPUT_VALUE_RADIX HEX $p
    return [expr "0x[get_property INPUT_VALUE $p]"]
}

# probe value by VIO input port number (for the counters: their net names are cnt_s1*, tm_s1*)
# a port may be split into several probes (one per connected net): combine them by PROBE_PORT_BITS
proc rp {port} {
    set v 0
    foreach p [get_hw_probes -of_objects [vio] -filter {TYPE == vio_input}] {
        if {[get_property PROBE_PORT $p] == $port} {
            set_property INPUT_VALUE_RADIX HEX $p
            set v [expr {$v | ([expr "0x[get_property INPUT_VALUE $p]"] << [get_property PROBE_PORT_BITS $p])}]
        }
    }
    return $v
}

set ltssm_names {
    00 Detect.Quiet 01 Detect.Quiet 02 Detect.Active 03 Detect.Active 04 Polling.Active 05 Polling.Config
    06 Polling.Compliance 07 Polling.Compliance 08 Polling.Compliance 09 Polling.Compliance 0A Polling.Compliance
    0B Config.Linkwidth.Start 0C Config.Linkwidth.Start 0D Config.Linkwidth.Accept 0E Config.Linkwidth.Accept
    0F Config.Lanenum.Wait 10 Config.Lanenum.Accept 11 Config.Complete 12 Config.Complete 13 Config.Complete
    14 Config.Complete 15 Config.Idle 16 L0 20 Recovery.RcvrLock 21 Recovery.RcvrCfg 22 Recovery.Speed
    23 Recovery.Speed 24 Recovery.Idle 25 Hot.Reset 26 Disabled 2B Loopback
}
set err_names {00 none 01 E_LINK_LOST 02 E_ENUM 03 E_ADMIN 04 E_IO_TIMEOUT 05 E_LBA_RANGE 06 E_LINK_TIMEOUT}

proc status {} {
    global ltssm_names err_names
    refresh_hw_vio [vio]
    set lt  [format %02X [rv ltssm_s1]]
    set err [format %02X [rv s_err]]
    set ltn  [expr {[dict exists $ltssm_names $lt] ? [dict get $ltssm_names $lt] : "?"}]
    set errn [expr {[dict exists $err_names $err] ? [dict get $err_names $err] : "?"}]
    set wid  [lindex {x1 x2 x4 ?} [rv s_width]]
    set bytes [rp 6]
    set run_cyc [rp 10]; set span_cyc [rp 11]
    set c [list "bytes=$bytes" "cmds=[rp 7]" "cmd_err=[rp 8]" "max_lat=[rp 9]"]
    if {$run_cyc > 0} {
        lappend c [format "run=%.3fs rate=%.1fMB/s" [expr {$run_cyc / 125e6}] [expr {$bytes / ($run_cyc / 125e6) / 1e6}]]
    }
    if {$span_cyc > 0 && $bytes > 131072} {
        lappend c [format "span=%.3fs sustained=%.1fMB/s" [expr {$span_cyc / 125e6}] [expr {($bytes - 131072) / ($span_cyc / 125e6) / 1e6}]]
    }
    set nsze [rp 17]; set lbads [rp 18]
    if {$nsze > 0} { lappend c [format "cap=%.1fGB" [expr {double($nsze) * (1 << $lbads) / 1e9}]] }
    set bmax [rp 19]; set bnow [rp 20]; set due [rp 21]
    if {$due > 0} {
        lappend c [format "src=%.3fGB backlog_now=%.1fKiB backlog_max=%.1fKiB" [expr {$due * 16 / 1e9}] [expr {$bnow * 16 / 1024.0}] [expr {$bmax * 16 / 1024.0}]]
    }
    global last_bytes last_cyc last_done last_err
    set last_bytes $bytes; set last_cyc $run_cyc; set last_done [rv s_done]; set last_err [expr {[rv s_err] != 0 || [rv s_lba]}]
    set fm [rp 22]
    if {$fm != 0} { lappend c [format "FORMAT: done=%d took=%d ms" [expr {($fm >> 32) & 1}] [expr {$fm & 0xFFFFFFFF}]] }
    set vf [rp 16]
    if {$vf & 2} {
        set vb [rp 12]; set ve [rp 13]; set el [rp 14]; set eg [rp 15]
        lappend c [format "VERIFY: limit_done=%d checked=%s err_beats=%d first_err_lba=%s exp=%08X got=%08X"             [expr {$vf & 1}] $vb $ve $el [expr {($eg >> 32) & 0xFFFFFFFF}] [expr {$eg & 0xFFFFFFFF}]]
    }
    puts [format "STATUS: link_up=%d gen2=%d width=%s ltssm=0x%s(%s) enum=%d nvme_ready=%d run=%d done=%d | err=0x%s(%s) detail=0x%02X flags=0x%02X lba_err=%d cmd_err_nz=%d | perst_pin=%d link_lost=%d link_fail=%d retries=%d start=%d stop=%d | %s"         [rv s_link_up] [rv s_gen2] $wid $lt $ltn [rv s_enum] [rv s_ready] [rv s_run] [rv s_done]         $err $errn [rv det_s1] [rv s_flags] [rv s_lba] [rv s_cmd_err_nz]         [rv perst_pin_s] [rv link_lost] [rv link_fail] [rv link_retries] [rv start_lat] [rv stop_lat] [join $c " "]]
}

proc pulse {name} {
    set p [get_hw_probes $name -of_objects [vio]]
    set_property OUTPUT_VALUE 1 $p; commit_hw_vio $p
    after 200
    set_property OUTPUT_VALUE 0 $p; commit_hw_vio $p
    puts "PULSE: $name"
}

# which cnt_s1* probe is which counter: decided from the probe's bit range (PROBE_PORT_BITS / name), see "probes"

# vivado.bat splits 'key=value' into 'key value': merge them back
set argv_m {}
foreach a $argv {
    if {[llength $argv_m] && [lindex $argv_m end] in {watch len rate rate_bps len_cmds len_gib until}} { lset argv_m end "[lindex $argv_m end]=$a" } else { lappend argv_m $a }
}
set argv $argv_m

set loaded 0
foreach a $argv {
    switch -glob -- $a {
        program {
            set_property PROGRAM.FILE $bit $dev
            set_property PROBES.FILE $ltx $dev
            set_property FULL_PROBES.FILE $ltx $dev
            program_hw_devices $dev
            refresh_hw_device $dev
            set loaded 1
            puts "PROGRAM: done=[get_property REGISTER.CONFIG_STATUS.BIT14_DONE_PIN $dev]"
        }
        default {
            if {!$loaded} { load_probes; set loaded 1 }
            switch -glob -- $a {
                probes {
                    foreach p [get_hw_probes -of_objects [vio]] {
                        set props {}
                        foreach k [list_property $p] { if {[string match -nocase "*PORT*" $k] || [string match -nocase "*BIT*" $k] || [string match -nocase "*MAP*" $k]} { lappend props "$k=[get_property $k $p]" } }
                        puts "PROBE: [get_property NAME $p] type=[get_property TYPE $p] width=[get_property WIDTH $p] $props"
                    }
                }
                status   { status }
                watch=*  { set n [string range $a 6 end]; for {set i 0} {$i < $n} {incr i} { status; after 1000 } }
                reset    { pulse vio_rst }
                start    { pulse vio_start }
                stop     { pulse vio_stop }
                capture - capture_enum {
                    # ILA u_ila: trigger on the first TX beat (dbg_uc[6] tx_tvalid & [7] tx_tready), or with
                    # capture_enum on the sequencer entering ENUM (dbg_uc[3:0] = 2); then START
                    set ila [lindex [get_hw_ilas -of_objects [current_hw_device]] 0]
                    set pr  [lindex [get_hw_probes -of_objects $ila] 0]
                    set v [string repeat X 304]
                    if {$a eq "capture"} { set bits {6 1 7 1} } else { set bits {0 0 1 1 2 0 3 0} }
                    foreach {b x} $bits { set i [expr {303 - $b}]; set v [string replace $v $i $i $x] }
                    set_property TRIGGER_COMPARE_VALUE "eq304'b$v" $pr
                    set_property CONTROL.TRIGGER_POSITION 16 $ila
                    set_property CONTROL.DATA_DEPTH 4096 $ila
                    run_hw_ila $ila
                    after 300
                    pulse vio_start
                    if {[catch {wait_on_hw_ila -timeout 1 $ila} e]} { puts "CAPTURE: no trigger ($e)" } else {
                        set d [upload_hw_ila_data $ila]
                        set csv [file join $repo_root fpga build ila_capture.csv]
                        write_hw_ila_data -force -csv_file $csv $d
                        puts "CAPTURE: written $csv"
                    }
                }
                rate_bps=* {
                    # real-time source: R bytes/s -> step = R * 2^32 / 2e9 (0 = off, back to rate=)
                    set r [string range $a 9 end]
                    set step [expr {wide(round(double($r) * 4294967296.0 / 2.0e9))}]
                    if {$r > 0 && $step == 0} { set step 1 }
                    if {$step > 4294967295} { set step 4294967295 }
                    set p [get_hw_probes vio_rate_step -of_objects [vio]]
                    set_property OUTPUT_VALUE_RADIX UNSIGNED $p; set_property OUTPUT_VALUE $step $p; commit_hw_vio $p
                    puts [format "RATE_BPS: %s B/s -> step %d (= %.3f B/s)" $r $step [expr {$step * 2.0e9 / 4294967296.0}]]
                }
                len_cmds=* - len_gib=* {
                    if {[string match len_gib=* $a]} { set n [expr {wide([string range $a 8 end]) * 8192}] } else { set n [string range $a 9 end] }
                    set p [get_hw_probes vio_len_cmds -of_objects [vio]]
                    set_property OUTPUT_VALUE_RADIX UNSIGNED $p; set_property OUTPUT_VALUE $n $p; commit_hw_vio $p
                    puts [format "LEN_CMDS: %d commands = %.3f GiB" $n [expr {$n / 8192.0}]]
                }
                until=* {
                    # poll every N s until done or error (max 2 h); print the rate of each interval
                    set iv [string range $a 6 end]
                    global last_bytes last_cyc last_done last_err
                    set pb 0; set pc 0; set t0 [clock seconds]
                    while {1} {
                        status
                        if {$last_cyc > $pc} {
                            puts [format "PROFILE: t=%.1fs bytes=%.3fGB interval_rate=%.1fMB/s" [expr {$last_cyc / 125e6}] [expr {$last_bytes / 1e9}] [expr {($last_bytes - $pb) / (($last_cyc - $pc) / 125e6) / 1e6}]]
                        }
                        set pb $last_bytes; set pc $last_cyc
                        if {$last_done || $last_err || [clock seconds] - $t0 > 7200} break
                        after [expr {int($iv * 1000)}]
                    }
                }
                len=*    { set p [get_hw_probes vio_len_sel -of_objects [vio]];  set_property OUTPUT_VALUE [string range $a 4 end] $p; commit_hw_vio $p; puts "LEN: [string range $a 4 end] (0 1 GiB, 1 8 GiB, 2 64 GiB, 3 unlimited)" }
                rate=*   { set p [get_hw_probes vio_rate_sel -of_objects [vio]]; set_property OUTPUT_VALUE [string range $a 5 end] $p; commit_hw_vio $p; puts "RATE: [string range $a 5 end] (0 full, 1 1/2, 2 1/4, 3 1/8)" }
                verify_on  { set p [get_hw_probes vio_verify -of_objects [vio]]; set_property OUTPUT_VALUE 1 $p; commit_hw_vio $p; puts "VERIFY: on (Read + compare, generator off)" }
                verify_off { set p [get_hw_probes vio_verify -of_objects [vio]]; set_property OUTPUT_VALUE 0 $p; commit_hw_vio $p; puts "VERIFY: off" }
                format_on  { set p [get_hw_probes vio_format -of_objects [vio]]; set_property OUTPUT_VALUE 1 $p; commit_hw_vio $p; puts "FORMAT: requested for the next START (erases the SSD)" }
                format_off { set p [get_hw_probes vio_format -of_objects [vio]]; set_property OUTPUT_VALUE 0 $p; commit_hw_vio $p; puts "FORMAT: off" }
                gen_on   { set p [get_hw_probes vio_gen_en -of_objects [vio]]; set_property OUTPUT_VALUE 1 $p; commit_hw_vio $p; puts "GEN: on" }
                gen_off  { set p [get_hw_probes vio_gen_en -of_objects [vio]]; set_property OUTPUT_VALUE 0 $p; commit_hw_vio $p; puts "GEN: off" }
                default  { puts "WARNING: unknown command $a" }
            }
        }
    }
}
close_hw_target
