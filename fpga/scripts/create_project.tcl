# =============================================================================
# create_project.tcl - Vivado 2022.2 project skeleton for the NVMe host
#                      (XC7K325T-2, pcie_7x Root Port Gen2 x4, 128-bit @ 125 MHz)
#
# Usage (from anywhere):   vivado -mode batch -source fpga/scripts/create_project.tcl
#   or from the GUI Tcl console:  source fpga/scripts/create_project.tcl
#
# What it does
#   1. creates the project  fpga/build/nvme_host  for xc7k325tffg900-2
#   2. adds every file listed in rtl/filelist.f (SystemVerilog)
#   3. creates the IP  pcie_7x_0  with the Root Port options of ARCHITECTURE.md section 1
#   4. adds fpga/constraints/kc705_fmc_m2.xdc and sets nvme_host_top as the top module
#
# SKELETON: the pcie_7x property names/values below are written from the IP's
# documented GUI options.  Each property is applied individually inside `catch`,
# and failures are printed as  "WARNING: ..."  - check them against
#     report_property [get_ips pcie_7x_0]
# and open the IP in the GUI once to confirm Root Port / Gen2 x4 / 128 bit / 125 MHz.
# =============================================================================

set script_dir [file dirname [file normalize [info script]]]
set repo_root  [file normalize [file join $script_dir .. ..]]
set proj_name  nvme_host
set proj_dir   [file join $repo_root fpga build $proj_name]
set part       xc7k325tffg900-2

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
create_ip -name pcie_7x -vendor xilinx.com -library ip -version 3.3 -module_name pcie_7x_0 \
          -dir [file join $repo_root fpga ip]

proc set_ip_prop {ip name value} {
    if {[catch {set_property -dict [list CONFIG.$name $value] $ip} err]} {
        puts "WARNING: could not set CONFIG.$name = $value : $err"
    }
}
set ip [get_ips pcie_7x_0]

# --- block / role ---
set_ip_prop $ip mode_selection                 Advanced
set_ip_prop $ip Device_Port_Type               Root_Port_of_PCI_Express_Root_Complex
set_ip_prop $ip PCIe_Blk_Locn                  X0Y0
# --- link ---
set_ip_prop $ip Maximum_Link_Width             X4
set_ip_prop $ip Link_Speed                     5.0_GT/s
set_ip_prop $ip Trgt_Link_Speed                4'h2
set_ip_prop $ip Ref_Clk_Freq                   100_MHz
# --- AXI-S interface ---
set_ip_prop $ip Interface_Width                128_bit
set_ip_prop $ip User_Clk_Freq                  125        ;# 125 MHz user clock (128-bit @ Gen2 x4)
# --- payload / performance ---
set_ip_prop $ip Max_Payload_Size               256_bytes
set_ip_prop $ip Perf_Level                     High
# --- configuration space of the Root Port: PCI-PCI bridge class code 0x060400, no BARs ---
set_ip_prop $ip Class_Code_Base                06
set_ip_prop $ip Class_Code_Sub                 04
set_ip_prop $ip Class_Code_Interface           00
set_ip_prop $ip Bar0_Enabled                   false     ;# BARs are not used by this Root Port design
# --- features not needed ---
set_ip_prop $ip Legacy_Interrupt               None
set_ip_prop $ip MSI_Enabled                    false
set_ip_prop $ip MSIx_Enabled                   false

generate_target all $ip
catch { create_ip_run $ip }

# ---------------------------------------------------------------------------
# 3. Constraints and top
# ---------------------------------------------------------------------------
add_files -fileset constrs_1 -norecurse [file join $repo_root fpga constraints kc705_fmc_m2.xdc]
set_property top nvme_host_top [current_fileset]
update_compile_order -fileset sources_1

# Optional: generics for a quick synthesis check
# set_property generic {N_SLOTS=8 SLOT_BYTES=131072} [current_fileset]

puts "Project created in $proj_dir"
puts "Next: review the WARNINGs above, assign the pins in fpga/constraints/kc705_fmc_m2.xdc, then run synthesis."
