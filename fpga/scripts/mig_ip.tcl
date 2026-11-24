# =============================================================================
# mig_ip.tcl - create the MIG 7-series IP `mig_ddr3` reproducibly (no GUI) and check it.
#
# Spec: STAND/RECORDER_SPEC.md 5.1-5.3 (WP4).  Inputs (tracked):
#   fpga/ip/mig_ddr3_a7/mig_ddr3.prj      hand-written MIG project (XML)
#   fpga/ip/mig_ddr3_a7/expected_pins.csv `port,pin` of the 71 DDR3 pins (spec 5.2)
#
# Two ways to use it
#   1. from create_project.tcl:   source fpga/scripts/mig_ip.tcl
#      (uses the caller's variable $ip_dir; the IP lands in $ip_dir/mig_ddr3; errors are Tcl errors)
#   2. standalone batch check (scratch project in fpga/build/mig_check, not the main project):
#        vivado -mode batch -source fpga/scripts/mig_ip.tcl -log fpga/build/mig_check/mig_check.log \
#               -journal fpga/build/mig_check/mig_check.jou
#      prints "MIG CHECK PASSED" and exits 0, or prints "MIG CHECK FAILED: ..." and exits 1.
#
# What is checked (any failure = error / exit 1)
#   a. key fields of the .prj: tCK (TimePeriod) 2500, DataWidth 32, XADC_En Disabled, SystemClock
#      No Buffer, ReferenceClock Use System Clock, InternalVref (reported, 0/1), 4:1, native interface, ...
#   b. the 71 pins of the .prj = expected_pins.csv
#   c. generate_target all raises no CRITICAL WARNING and no ERROR
#   d. PACKAGE_PIN lines of the generated mig_ddr3.xdc = expected_pins.csv (all 71, nothing else)
#   e. the generated mig.prj (normalised by MIG) still has the key fields; the generated RTL top
#      has app_addr[28:0], app_wdf_data[255:0], device_temp_i[11:0], sys_clk_i, no clk_ref_i;
#      the generated XDC sets INTERNAL_VREF 0.750 on bank 35 iff InternalVref = 1
# Test hooks (negative controls): env MIG_PRJ / MIG_PINS_CSV override the input files,
#   MIG_SKIP_PRECHECK=1 skips (a)/(b) so that (d) can be exercised, MIG_SELFTEST_CW=1 raises a
#   CRITICAL WARNING during generation to prove that (c) notices it.
# =============================================================================

set _mig_script [file normalize [info script]]
set _mig_repo   [file normalize [file join [file dirname $_mig_script] .. ..]]
set _mig_prj    [file join $_mig_repo fpga ip mig_ddr3_a7 mig_ddr3.prj]
set _mig_csv    [file join $_mig_repo fpga ip mig_ddr3_a7 expected_pins.csv]
if {[info exists ::env(MIG_PRJ)]}      { set _mig_prj $::env(MIG_PRJ) }
if {[info exists ::env(MIG_PINS_CSV)]} { set _mig_csv $::env(MIG_PINS_CSV) }
set _mig_standalone [expr {![info exists ip_dir]}]
set _mig_part   xc7a200tfbg484-2

proc mig_fail {msg} {
    global _mig_standalone
    puts "MIG CHECK FAILED: $msg"
    if {$_mig_standalone} { exit 1 } else { error "MIG CHECK FAILED: $msg" }
}

proc mig_read {path} {
    set f [open $path r]; fconfigure $f -encoding utf-8
    set d [read $f]; close $f
    return [string map [list "\r" "" "﻿" ""] $d]
}

# <Tag>value</Tag> (first occurrence), trimmed
proc mig_field {xml tag} {
    if {![regexp "<$tag\[^>\]*>(\[^<\]*)</$tag>" $xml -> v]} { return "" }
    return [string trim $v]
}

# {port pin} dict from the <Pin ... PADName="x" ... name="y"/> lines of a .prj
proc mig_prj_pins {xml} {
    set d [dict create]
    foreach {all attrs} [regexp -all -inline {<Pin ([^>]*)/>} $xml] {
        if {[regexp {PADName="([^"]*)"} $attrs -> pad] && [regexp { name="([^"]*)"} $attrs -> nm]} {
            if {[string match "ddr3_*" $nm]} { dict set d $nm $pad }
        }
    }
    return $d
}

proc mig_csv_pins {path} {
    set d [dict create]
    set first 1
    foreach line [split [mig_read $path] "\n"] {
        set line [string trim $line]
        if {$line eq ""} continue
        if {$first} { set first 0; continue }
        lassign [split $line ","] p n
        dict set d $p $n
    }
    return $d
}

proc mig_cmp_pins {have want what} {
    set bad {}
    dict for {p n} $want {
        if {![dict exists $have $p]} { lappend bad "$p missing" } elseif {[dict get $have $p] ne $n} {
            lappend bad "$p: $what has [dict get $have $p], expected $n"
        }
    }
    dict for {p n} $have { if {![dict exists $want $p]} { lappend bad "$p unexpected in $what" } }
    if {[llength $bad]} { mig_fail "pins ($what): [join [lrange $bad 0 9] {; }]" }
}

# expected values of the key fields
set _mig_fields {
    ModuleName mig_ddr3          TargetFPGA {xc7a200t-fbg484/-2}   XADC_En Disabled
    SystemClock {No Buffer}      ReferenceClock {Use System Clock} SysResetPolarity {ACTIVE LOW}
    TimePeriod 2500              PHYRatio 4:1                      InputClkFreq 200
    DataWidth 32                 DataMask 1                        ECC Disabled
    PortInterface NATIVE         MemoryVoltage 1.5V                UserMemoryAddressMap ROW_BANK_COLUMN
    RowAddress 15                ColAddress 10                     BankAddress 3
    emrRTT RZQ/4                 emrOutputDriveStrength RZQ/7      mrCasLatency 6
    mr2CasWriteLatency 5         Debug_En OFF
}

proc mig_check_fields {xml what} {
    global _mig_fields
    foreach {tag want} $_mig_fields {
        set have [mig_field $xml $tag]
        if {$have ne $want} { mig_fail "$what: <$tag> is '$have', expected '$want'" }
    }
    set md [mig_field $xml MemoryDevice]
    if {![string match -nocase "DDR3_SDRAM/Components/MT41J256M16XX-125" $md]} { mig_fail "$what: MemoryDevice '$md'" }
    set iv [mig_field $xml InternalVref]
    if {$iv ne "0" && $iv ne "1"} { mig_fail "$what: <InternalVref> '$iv' is not 0/1" }
    return $iv
}

# ---------------------------------------------------------------- a, b: static checks
set _mig_xml  [mig_read $_mig_prj]
set _mig_want [mig_csv_pins $_mig_csv]
if {[dict size $_mig_want] != 71} { mig_fail "expected_pins.csv has [dict size $_mig_want] pins, not 71" }
if {![info exists ::env(MIG_SKIP_PRECHECK)]} {
    set _mig_vref [mig_check_fields $_mig_xml ".prj"]
    mig_cmp_pins [mig_prj_pins $_mig_xml] $_mig_want ".prj"
    puts "MIG: .prj fields ok (tCK=2500, 32 bit, XADC Disabled, No Buffer/Use System Clock, InternalVref=$_mig_vref), 71 pins = expected_pins.csv"
} else {
    set _mig_vref [mig_field $_mig_xml InternalVref]
}

# ---------------------------------------------------------------- IP creation
if {$_mig_standalone} {
    set _mig_dir [file join $_mig_repo fpga build mig_check]
    file mkdir $_mig_dir
    set _mig_ip_dir [file join $_mig_dir ip]
    file delete -force $_mig_ip_dir [file join $_mig_dir prj]
    create_project -force mig_check [file join $_mig_dir prj] -part $_mig_part
} else {
    set _mig_ip_dir $ip_dir
}
file mkdir $_mig_ip_dir

set _mig_cw0 [get_msg_config -severity {CRITICAL WARNING} -count]
set _mig_er0 [get_msg_config -severity ERROR -count]

# A rerun (recreate) finds the old IP folder; Vivado then puts the new IP in mig_ddr3_1 and the
# relative XML_INPUT_FILE below would not see the .prj.  Remove old folders, copy the .prj next to the .xci.
foreach _d [glob -nocomplain -directory $_mig_ip_dir mig_ddr3 mig_ddr3_*] { file delete -force $_d }
create_ip -name mig_7series -vendor xilinx.com -library ip -module_name mig_ddr3 -dir $_mig_ip_dir
file copy -force $_mig_prj [file join [file dirname [get_property IP_FILE [get_ips mig_ddr3]]] mig_ddr3.prj]
set_property -dict [list CONFIG.XML_INPUT_FILE {mig_ddr3.prj} CONFIG.RESET_BOARD_INTERFACE {Custom} \
    CONFIG.MIG_DONT_TOUCH_PARAM {Custom} CONFIG.BOARD_MIG_PARAM {Custom}] [get_ips mig_ddr3]
if {[info exists ::env(MIG_SELFTEST_CW)]} { send_msg_id {MIGCHK 1-1} {CRITICAL WARNING} "self-test critical warning" }
generate_target all [get_ips mig_ddr3]

# ---------------------------------------------------------------- c: messages
set _mig_ncw [expr {[get_msg_config -severity {CRITICAL WARNING} -count] - $_mig_cw0}]
set _mig_ner [expr {[get_msg_config -severity ERROR -count] - $_mig_er0}]
if {$_mig_ncw > 0 || $_mig_ner > 0} { mig_fail "generate_target raised $_mig_ncw CRITICAL WARNING(s) and $_mig_ner ERROR(s)" }
puts "MIG: generate_target all - 0 CRITICAL WARNING, 0 ERROR"

# ---------------------------------------------------------------- d, e: generated files
set _mig_gen [file join $_mig_ip_dir mig_ddr3 mig_ddr3]
set _mig_xdc [file join $_mig_gen user_design constraints mig_ddr3.xdc]
set _mig_top [file join $_mig_gen user_design rtl mig_ddr3.v]
set _mig_gprj [file join $_mig_gen mig.prj]
foreach f [list $_mig_xdc $_mig_top $_mig_gprj] { if {![file exists $f]} { mig_fail "generated file missing: $f" } }

set _mig_xdc_txt [mig_read $_mig_xdc]
set _mig_got [dict create]
foreach {all pin port} [regexp -all -inline {set_property\s+PACKAGE_PIN\s+(\S+)\s+\[get_ports\s+\{([^\}]+)\}\s*\]} $_mig_xdc_txt] {
    dict set _mig_got $port $pin
}
mig_cmp_pins $_mig_got $_mig_want "generated mig_ddr3.xdc"
puts "MIG: generated XDC has [dict size $_mig_got] PACKAGE_PINs, all = expected_pins.csv"

if {![info exists ::env(MIG_SKIP_PRECHECK)]} { mig_check_fields [mig_read $_mig_gprj] "generated mig.prj" }

set _mig_rtl [mig_read $_mig_top]
foreach pat {
    {input\s*\[28:0\]\s*app_addr\y} {input\s*\[255:0\]\s*app_wdf_data\y} {output\s*\[255:0\]\s*app_rd_data\y}
    {input\s*\[11:0\]\s*device_temp_i\y} {input\s+sys_clk_i\y} {input\s+sys_rst\y} {output\s+ui_clk\y}
    {output\s+init_calib_complete\y} {inout\s*\[31:0\]\s*ddr3_dq\y}
} {
    if {![regexp -- $pat $_mig_rtl]} { mig_fail "generated RTL top: port pattern '$pat' not found" }
}
if {[regexp {clk_ref_i} $_mig_rtl]} { mig_fail "generated RTL top has clk_ref_i (ReferenceClock must be Use System Clock)" }
foreach {par val} {DQ_WIDTH 32 ADDR_WIDTH 29 ROW_WIDTH 15 COL_WIDTH 10 BANK_WIDTH 3 CLKIN_PERIOD 5000 MMCM_VCO 800} {
    if {![regexp "parameter\\s+$par\\s*=\\s*$val\\y" [mig_read [file join $_mig_gen user_design rtl mig_ddr3_mig.v]]]} {
        mig_fail "generated mig_ddr3_mig.v: parameter $par != $val"
    }
}
set _mig_has_vref [regexp {set_property\s+INTERNAL_VREF\s+0\.750\s+\[get_iobanks\s+35\]} $_mig_xdc_txt]
if {$_mig_vref eq "1" && !$_mig_has_vref} { mig_fail "InternalVref=1 but the generated XDC has no INTERNAL_VREF 0.750 for bank 35" }
if {$_mig_vref eq "0" &&  $_mig_has_vref} { mig_fail "InternalVref=0 but the generated XDC sets INTERNAL_VREF" }
puts "MIG: generated RTL ports/parameters ok (app 29/256 bit, device_temp_i, sys_clk_i, no clk_ref_i), INTERNAL_VREF consistent ($_mig_vref)"

puts "MIG CHECK PASSED"
if {$_mig_standalone} { exit 0 }
