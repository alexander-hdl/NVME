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
#     len=<0..3>     generator length: 0 1 GiB (default), 1 8 GiB, 2 64 GiB, 3 unlimited (set BEFORE start)
#     rate=<0..3>    generator rate: 0 full (default), 1 1/2, 2 1/4, 3 1/8
#     gen_on | gen_off  generator enable (VIO probe_out3, 0 after programming)
#     probes         list the VIO probes with their properties (debugging this script)
#   Commands run in the given order, e.g.  -tclargs program watch=5 status
# =============================================================================
set repo_root [file normalize [file join [file dirname [info script]] .. ..]]
set bit [file join $repo_root fpga build ax7a200b_top.bit]
set ltx [file join $repo_root fpga build ax7a200b_top.ltx]

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
proc rp {port} {
    foreach p [get_hw_probes -of_objects [vio] -filter {TYPE == vio_input}] {
        if {[get_property PROBE_PORT $p] == $port} {
            set_property INPUT_VALUE_RADIX HEX $p
            return [expr "0x[get_property INPUT_VALUE $p]"]
        }
    }
    return 0
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
    if {[llength $argv_m] && [lindex $argv_m end] in {watch len rate}} { lset argv_m end "[lindex $argv_m end]=$a" } else { lappend argv_m $a }
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
                len=*    { set p [get_hw_probes vio_len_sel -of_objects [vio]];  set_property OUTPUT_VALUE [string range $a 4 end] $p; commit_hw_vio $p; puts "LEN: [string range $a 4 end] (0 1 GiB, 1 8 GiB, 2 64 GiB, 3 unlimited)" }
                rate=*   { set p [get_hw_probes vio_rate_sel -of_objects [vio]]; set_property OUTPUT_VALUE [string range $a 5 end] $p; commit_hw_vio $p; puts "RATE: [string range $a 5 end] (0 full, 1 1/2, 2 1/4, 3 1/8)" }
                gen_on   { set p [get_hw_probes vio_gen_en -of_objects [vio]]; set_property OUTPUT_VALUE 1 $p; commit_hw_vio $p; puts "GEN: on" }
                gen_off  { set p [get_hw_probes vio_gen_en -of_objects [vio]]; set_property OUTPUT_VALUE 0 $p; commit_hw_vio $p; puts "GEN: off" }
                default  { puts "WARNING: unknown command $a" }
            }
        }
    }
}
close_hw_target
