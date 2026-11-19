# =============================================================================
# create_project.tcl - Vivado 2022.2 project for the NVMe host
#                      (XC7K325T-2, pcie_7x v3.3 Root Port Gen2 x4, 128-bit @ 125 MHz)
#
# Usage:  vivado -mode batch -source fpga/scripts/create_project.tcl
#         vivado -mode batch -source fpga/scripts/create_project.tcl -tclargs regen_ip
#         vivado -mode batch -source fpga/scripts/create_project.tcl -tclargs top=nvme_host_top
#   or from the GUI Tcl console:  source fpga/scripts/create_project.tcl
#
#   default    : if fpga/ip/pcie_7x_0/pcie_7x_0.xci exists it is imported (re-used as is),
#                otherwise the IP is created from the property list below.
#   regen_ip   : ignore the tracked .xci, create the IP from the property list and
#                overwrite fpga/ip/pcie_7x_0/pcie_7x_0.xci.
#   board=<b>  : kc705 (default: XC7K325T-2, project fpga/build/nvme_host, IP fpga/ip/pcie_7x_0/) or
#                ax7a200b (ALINX AX7A200B, XC7A200T-2FBG484I, project fpga/build/nvme_host_ax7a200b,
#                IP fpga/ip/pcie_7x_0_a7/, top ax7a200b_top, constraints ax7a200b_*.xdc;
#                see STAND/AX7A200B_ARCHITECTURE.md).  The IP keeps the module name pcie_7x_0.
#   top=<name> : project top module.  Default kc705_top (board top, rtl/board/kc705_top.sv, all pins in
#                kc705_board.xdc).  top=nvme_host_top builds the bare host (no pins for the user-side
#                ports, only a bitstream with `loose_io`): only kc705_fmc_m2*.xdc are used plus a
#                generated fpga/build/nvme_host_top_extra.xdc with the free_clk / data_clk clocks.
#
# What it does
#   1. creates the project  fpga/build/nvme_host  for xc7k325tffg900-2
#   2. adds every file listed in rtl/filelist.f (SystemVerilog)
#   3. creates / imports the IP pcie_7x_0 (ARCHITECTURE.md section 1, D2) and generates all
#      output products (generate_target all) - property names verified on Vivado 2022.2,
#      pcie_7x v3.3 (see fpga/README.md for the table)
#   4. adds fpga/constraints/kc705_board.xdc, kc705_fmc_m2.xdc, kc705_fmc_m2_cdc.xdc and sets kc705_top
#      (or nvme_host_top, see top=) as the top module
#
# Everything generated stays inside fpga/build/ (git-ignored); only the .xci under
# fpga/ip/pcie_7x_0/ is tracked.
# =============================================================================

set script_dir [file dirname [file normalize [info script]]]
set repo_root  [file normalize [file join $script_dir .. ..]]
set build_dir  [file join $repo_root fpga build]

# vivado.bat splits 'key=value' into 'key value': merge them back
set argv_m {}
foreach a $argv {
    if {[llength $argv_m] && [lindex $argv_m end] in {top jobs board}} { lset argv_m end "[lindex $argv_m end]=$a" } else { lappend argv_m $a }
}
set argv $argv_m
set regen_ip 0
set top_name {}
set board    kc705
if {[info exists argv]} {
    if {[lsearch -exact $argv regen_ip] >= 0} { set regen_ip 1 }
    foreach a $argv {
        if {[string match top=* $a]}   { set top_name [string range $a 4 end] }
        if {[string match board=* $a]} { set board    [string range $a 6 end] }
    }
}
switch -- $board {
    kc705 {
        set proj_name nvme_host
        set part      xc7k325tffg900-2
        set xci_repo  [file join $repo_root fpga ip pcie_7x_0 pcie_7x_0.xci]
        set ip_dir    [file join $build_dir ip]
        if {$top_name eq ""} { set top_name kc705_top }
        if {$top_name ni {kc705_top nvme_host_top}} { error "top=$top_name : must be kc705_top or nvme_host_top" }
    }
    ax7a200b {
        set proj_name nvme_host_ax7a200b
        set part      xc7a200tfbg484-2
        set xci_repo  [file join $repo_root fpga ip pcie_7x_0_a7 pcie_7x_0.xci]
        set ip_dir    [file join $build_dir ip_a7]
        if {$top_name eq ""} { set top_name ax7a200b_top }
        if {$top_name ne "ax7a200b_top"} { error "board=ax7a200b supports only top=ax7a200b_top" }
    }
    default { error "board=$board : must be kc705 or ax7a200b" }
}
set proj_dir [file join $build_dir $proj_name]

file mkdir $build_dir
create_project -force $proj_name $proj_dir -part $part
set_property target_language Verilog [current_project]

# ---------------------------------------------------------------------------
# 1. RTL from rtl/filelist.f  (paths relative to the repo root, '#' comments)
# ---------------------------------------------------------------------------
set fh [open [file join $repo_root rtl filelist.f] r]
set rtl_files {}
while {[gets $fh line] >= 0} {
    set line [string trim [lindex [split $line "#"] 0]]
    if {$line eq ""} { continue }
    lappend rtl_files [file normalize [file join $repo_root $line]]
}
close $fh
add_files -norecurse -fileset sources_1 $rtl_files
foreach f $rtl_files { set_property file_type SystemVerilog [get_files $f] }
update_compile_order -fileset sources_1

# ---------------------------------------------------------------------------
# 2. pcie_7x_0 : Root Port, Gen2 x4, 128-bit AXI-S, 125 MHz user clock
# ---------------------------------------------------------------------------
# The options are applied in stages (order matters: the IP's GUI rules re-derive
# dependent parameters, e.g. Device_Port_Type resets the class code, Interface_Width
# resets User_Clk_Freq) and every value is read back and checked.
set ip_cfg_errors 0
proc ip_stage {ip lst} {
    global ip_cfg_errors
    if {[catch {set_property -dict $lst $ip} err]} {
        puts "ERROR: ip_stage failed for {$lst} : $err"
        incr ip_cfg_errors
    }
    foreach {k v} $lst {
        set got [get_property $k $ip]
        if {[string tolower $got] ne [string tolower $v]} {
            puts "ERROR: $k : wanted '$v', IP has '$got'"
            incr ip_cfg_errors
        }
    }
}

if {!$regen_ip && [file exists $xci_repo]} {
    puts "INFO: importing tracked IP $xci_repo"
    import_ip $xci_repo
    set ip [get_ips pcie_7x_0]
} else {
    puts "INFO: creating pcie_7x_0 from the property list"
    file mkdir $ip_dir
    create_ip -name pcie_7x -vendor xilinx.com -library ip -version 3.3 -module_name pcie_7x_0 \
              -dir $ip_dir
    set ip [get_ips pcie_7x_0]

    # --- role / shared logic (clocking + GT_COMMON inside the core: no pipe_clk/qpll ports) ---
    ip_stage $ip {CONFIG.mode_selection Advanced}
    ip_stage $ip {CONFIG.Device_Port_Type Root_Port_of_PCI_Express_Root_Complex}
    ip_stage $ip {CONFIG.en_ext_clk false}
    ip_stage $ip {CONFIG.shared_logic_in_core true}
    # --- link: x4, 5.0 GT/s, 128-bit AXI-S, 125 MHz user clock, 100 MHz refclk ---
    ip_stage $ip {CONFIG.Maximum_Link_Width X4  CONFIG.Link_Speed 5.0_GT/s}
    ip_stage $ip {CONFIG.Interface_Width 128_bit}
    ip_stage $ip {CONFIG.User_Clk_Freq 125  CONFIG.Ref_Clk_Freq 100_MHz  CONFIG.PCIe_Blk_Locn X0Y0}
    ip_stage $ip {CONFIG.Trgt_Link_Speed 4'h2}
    # --- payload / buffering: MPS 256, "High" performance level.  Buf_Opt_BMA stays false:
    #     BMA=true shrinks the NP header credits to 4 (and PD to 32) to enlarge the completion
    #     buffer, but this Root Port is the *target* of the SSD's many outstanding MRd/MWr
    #     (12 NP header credits with BMA=false, ARCHITECTURE.md 5.x / research/03 section 5).
    ip_stage $ip {CONFIG.Max_Payload_Size 256_bytes  CONFIG.Perf_Level High  CONFIG.Buf_Opt_BMA false}
    # --- Root Port class code PCI-PCI bridge 0x060400 (Base_Class is forced to 06 by the Root Port mode) ---
    ip_stage $ip {CONFIG.Class_Code_Base 06  CONFIG.Class_Code_Sub 04  CONFIG.Class_Code_Interface 00}
    # --- BAR0: the IP refuses a configuration with every BAR disabled ("At least one Bar must be
    #     selected"), also in Root Port mode.  BAR0 stays at its default (enabled, 2 KB memory);
    #     the Root Port never programs it and the core does no BAR filtering in Root Port mode. ---
    # --- features not needed ---
    ip_stage $ip {CONFIG.Legacy_Interrupt NONE  CONFIG.MSI_Enabled false  CONFIG.MSIx_Enabled false}
    ip_stage $ip {CONFIG.AER_Enabled false  CONFIG.RECRC_Check 0  CONFIG.AER_ECRC_Check_Capable false  CONFIG.AER_ECRC_Gen_Capable false}
    # --- "Enable Message Routing" (messages forwarded to the AXI-S RX interface): all off ---
    ip_stage $ip {CONFIG.En_route_err_cor false  CONFIG.En_route_err_nfl false  CONFIG.En_route_err_ftl false \
                  CONFIG.En_route_inta false  CONFIG.En_route_intb false  CONFIG.En_route_intc false  CONFIG.En_route_intd false \
                  CONFIG.En_route_pm_pme false  CONFIG.En_route_pme_to_ack false}
    # --- Link Control RCB 64 B, extended tags on by default, rx_np_ok (not rx_np_req) flow control ---
    ip_stage $ip {CONFIG.RCB 64_byte  CONFIG.Extended_Tag_Field true  CONFIG.Extended_Tag_Default true}
    ip_stage $ip {CONFIG.Receive_NP_Request false}
    ip_stage $ip {CONFIG.PCIe_Debug_Ports false}
    if {$ip_cfg_errors > 0} { error "pcie_7x_0 configuration failed ($ip_cfg_errors problem(s)), see ERROR lines above" }
}

# ---------------------------------------------------------------------------
# 2b. ax7a200b only: VIO for JTAG status / remote START-STOP (rtl/board/ax7a200b_top.sv, u_vio)
# ---------------------------------------------------------------------------
if {$board eq "ax7a200b"} {
    set vio_xci [file join $repo_root fpga ip vio_status_a7 vio_status.xci]
    if {!$regen_ip && [file exists $vio_xci]} {
        import_ip $vio_xci
    } else {
        create_ip -name vio -vendor xilinx.com -library ip -module_name vio_status -dir $ip_dir
        set_property -dict [list CONFIG.C_NUM_PROBE_IN 10 CONFIG.C_NUM_PROBE_OUT 4             CONFIG.C_PROBE_IN0_WIDTH 8  CONFIG.C_PROBE_IN1_WIDTH 16  CONFIG.C_PROBE_IN2_WIDTH 8             CONFIG.C_PROBE_IN3_WIDTH 2  CONFIG.C_PROBE_IN4_WIDTH 7  CONFIG.C_PROBE_IN5_WIDTH 6             CONFIG.C_PROBE_IN6_WIDTH 64 CONFIG.C_PROBE_IN7_WIDTH 64 CONFIG.C_PROBE_IN8_WIDTH 32             CONFIG.C_PROBE_IN9_WIDTH 32             CONFIG.C_PROBE_OUT0_WIDTH 1 CONFIG.C_PROBE_OUT1_WIDTH 1 CONFIG.C_PROBE_OUT2_WIDTH 1 CONFIG.C_PROBE_OUT3_WIDTH 1] [get_ips vio_status]
        file mkdir [file dirname $vio_xci]
        file copy -force [get_property IP_FILE [get_ips vio_status]] $vio_xci
    }
    generate_target all [get_ips vio_status]
    catch { create_ip_run [get_ips vio_status] }

    # ILA on user_clk (u_ila): 304-bit debug bus of nvme_host_top, 4096 samples
    set ila_xci [file join $repo_root fpga ip ila_pcie_a7 ila_pcie.xci]
    if {!$regen_ip && [file exists $ila_xci]} {
        import_ip $ila_xci
    } else {
        create_ip -name ila -vendor xilinx.com -library ip -module_name ila_pcie -dir $ip_dir
        set_property -dict [list CONFIG.C_NUM_OF_PROBES 1 CONFIG.C_PROBE0_WIDTH 304 CONFIG.C_DATA_DEPTH 4096 CONFIG.C_TRIGIN_EN false CONFIG.C_ADV_TRIGGER false] [get_ips ila_pcie]
        file mkdir [file dirname $ila_xci]
        file copy -force [get_property IP_FILE [get_ips ila_pcie]] $ila_xci
    }
    generate_target all [get_ips ila_pcie]
    catch { create_ip_run [get_ips ila_pcie] }
}

# generate everything (synthesis sources, instantiation template, simulation, constraints) and
# make the IP synthesise out-of-context in its own run
generate_target all $ip
catch { create_ip_run $ip }

# keep the tracked copy of the .xci in fpga/ip/ (only the .xci is tracked)
if {$regen_ip || ![file exists $xci_repo]} {
    file mkdir [file dirname $xci_repo]
    file copy -force [get_property IP_FILE $ip] $xci_repo
    puts "INFO: wrote $xci_repo"
}

# ---------------------------------------------------------------------------
# 3. Constraints and top
# ---------------------------------------------------------------------------
if {$board eq "ax7a200b"} {
    add_files -fileset constrs_1 -norecurse [file join $repo_root fpga constraints ax7a200b_board.xdc]
    add_files -fileset constrs_1 -norecurse [file join $repo_root fpga constraints ax7a200b_cdc.xdc]
    set_property USED_IN {implementation} [get_files ax7a200b_cdc.xdc]
} elseif {$top_name eq "kc705_top"} {
    add_files -fileset constrs_1 -norecurse [file join $repo_root fpga constraints kc705_board.xdc]
} else {
    # bare nvme_host_top: the clocks that kc705_board.xdc / kc705_top provide, plus the SYSCLK pins
    set xf [open [file join $build_dir nvme_host_top_extra.xdc] w]
    puts $xf "# generated by create_project.tcl (top=nvme_host_top)"
    puts $xf "create_clock -name free_clk -period 5.000 \[get_ports free_clk_p\]"
    puts $xf "create_clock -name data_clk -period 8.000 \[get_ports data_clk\]"
    puts $xf "set_property PACKAGE_PIN AD12 \[get_ports free_clk_p\]"
    puts $xf "set_property PACKAGE_PIN AD11 \[get_ports free_clk_n\]"
    puts $xf "set_property IOSTANDARD LVDS \[get_ports {free_clk_p free_clk_n}\]"
    puts $xf "set_property CFGBVS GND \[current_design\]"
    puts $xf "set_property CONFIG_VOLTAGE 1.8 \[current_design\]"
    puts $xf "set_property BITSTREAM.GENERAL.COMPRESS TRUE \[current_design\]"
    close $xf
    add_files -fileset constrs_1 -norecurse [file join $build_dir nvme_host_top_extra.xdc]
}
if {$board eq "kc705"} {
    add_files -fileset constrs_1 -norecurse [file join $repo_root fpga constraints kc705_fmc_m2.xdc]
    add_files -fileset constrs_1 -norecurse [file join $repo_root fpga constraints kc705_fmc_m2_cdc.xdc]
    # CDC clock groups refer to the IP's clocks, which do not exist while synth_design treats the IP as a black box
    set_property USED_IN {implementation} [get_files kc705_fmc_m2_cdc.xdc]
}
if {$top_name eq "nvme_host_top"} {
    # data_clk is a separate clock only here (kc705_top runs the stream on user_clk)
    set xf [open [file join $build_dir nvme_host_top_extra_impl.xdc] w]
    puts $xf "set_clock_groups -asynchronous -group \[get_clocks data_clk\] -group \[get_clocks userclk2\]"
    close $xf
    add_files -fileset constrs_1 -norecurse [file join $build_dir nvme_host_top_extra_impl.xdc]
    set_property USED_IN {implementation} [get_files nvme_host_top_extra_impl.xdc]
}
set_property top $top_name [current_fileset]
update_compile_order -fileset sources_1

# Optional: generics for a quick synthesis check
# set_property generic {N_SLOTS=8 SLOT_BYTES=131072} [current_fileset]

puts "Project created in $proj_dir"
puts "Next: fpga/scripts/build.tcl (synthesis / implementation / bitstream) or launch_runs synth_1."
