# =============================================================================
# create_project.tcl - Vivado 2022.2 project for the NVMe host
#                      (XC7K325T-2, pcie_7x v3.3 Root Port Gen2 x4, 128-bit @ 125 MHz)
#
# Usage:  vivado -mode batch -source fpga/scripts/create_project.tcl
#         vivado -mode batch -source fpga/scripts/create_project.tcl -tclargs regen_ip
#   or from the GUI Tcl console:  source fpga/scripts/create_project.tcl
#
#   default    : if fpga/ip/pcie_7x_0/pcie_7x_0.xci exists it is imported (re-used as is),
#                otherwise the IP is created from the property list below.
#   regen_ip   : ignore the tracked .xci, create the IP from the property list and
#                overwrite fpga/ip/pcie_7x_0/pcie_7x_0.xci.
#
# What it does
#   1. creates the project  fpga/build/nvme_host  for xc7k325tffg900-2
#   2. adds every file listed in rtl/filelist.f (SystemVerilog)
#   3. creates / imports the IP pcie_7x_0 (ARCHITECTURE.md section 1, D2) and generates all
#      output products (generate_target all) - property names verified on Vivado 2022.2,
#      pcie_7x v3.3 (see fpga/README.md for the table)
#   4. adds fpga/constraints/kc705_fmc_m2.xdc and sets nvme_host_top as the top module
#
# Everything generated stays inside fpga/build/ (git-ignored); only the .xci under
# fpga/ip/pcie_7x_0/ is tracked.
# =============================================================================

set script_dir [file dirname [file normalize [info script]]]
set repo_root  [file normalize [file join $script_dir .. ..]]
set build_dir  [file join $repo_root fpga build]
set proj_name  nvme_host
set proj_dir   [file join $build_dir $proj_name]
set part       xc7k325tffg900-2
set xci_repo   [file join $repo_root fpga ip pcie_7x_0 pcie_7x_0.xci]

set regen_ip 0
if {[info exists argv] && [lsearch -exact $argv regen_ip] >= 0} { set regen_ip 1 }

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
    file mkdir [file join $build_dir ip]
    create_ip -name pcie_7x -vendor xilinx.com -library ip -version 3.3 -module_name pcie_7x_0 \
              -dir [file join $build_dir ip]
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
add_files -fileset constrs_1 -norecurse [file join $repo_root fpga constraints kc705_fmc_m2.xdc]
add_files -fileset constrs_1 -norecurse [file join $repo_root fpga constraints kc705_fmc_m2_cdc.xdc]
# CDC clock groups refer to the IP's clocks, which do not exist while synth_design treats the IP as a black box
set_property USED_IN {implementation} [get_files kc705_fmc_m2_cdc.xdc]
set_property top nvme_host_top [current_fileset]
update_compile_order -fileset sources_1

# Optional: generics for a quick synthesis check
# set_property generic {N_SLOTS=8 SLOT_BYTES=131072} [current_fileset]

puts "Project created in $proj_dir"
puts "Next: fpga/scripts/build.tcl (synthesis / implementation / bitstream) or launch_runs synth_1."
