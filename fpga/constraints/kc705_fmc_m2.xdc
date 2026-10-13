# =============================================================================
# kc705_fmc_m2.xdc - constraints for nvme_host_top on a KC705 (XC7K325T-2 FFG900)
# with an M.2 NVMe SSD on an FMC HPC adapter board (e.g. the Opsero FMC M.2 adapter).
#
# STATUS: EXAMPLE / SKELETON.  Everything in the "BOARD-SPECIFIC PINS" section is a
# placeholder to be verified against the KC705 user guide (UG810), the FMC adapter
# schematic and the constraints generated with pcie_7x_0.  Items marked TODO are
# mandatory before the first hardware run.
# =============================================================================

# ---------------------------------------------------------------------------
# Clocks
# ---------------------------------------------------------------------------
# 100 MHz PCIe reference clock from the adapter -> FMC HPC GBTCLK0_M2C
create_clock -name sys_clk -period 10.000 [get_ports sys_clk_p]

# free-running board oscillator for link_bringup / PERST# timing: KC705 SYSCLK, 200 MHz LVDS
# (pcie_7x_0 uses IBUFDS_GTE2.O -> BUFG internally, so IBUFDS_GTE2.ODIV2 cannot be used for this)
create_clock -name free_clk -period 5.000 [get_ports free_clk_p]

# user data clock (set by whoever produces the stream).
# TODO: set the real period.  8.000 ns (125 MHz) is a PLACEHOLDER so that the async FIFO in
# data_ring is timed at all; without a clock the data_clk domain is unconstrained.
create_clock -name data_clk -period 8.000 [get_ports data_clk]

# ---------------------------------------------------------------------------
# Clock-domain crossings: see kc705_fmc_m2_cdc.xdc (implementation only, because the
# clocks of the pcie_7x_0 IP do not exist while synth_design runs it as a black box).
#   user_clk (pcie_7x userclk2, 125 MHz) <-> link_clk (free_clk 200 MHz via BUFG) <-> data_clk (user)
# Crossings: 2-FF synchronisers (ASYNC_REG) and reset synchronisers in nvme_host_top and
# link_bringup, plus the gray-coded async FIFO in data_ring.
# ---------------------------------------------------------------------------
# Pure synchroniser inputs:
set_false_path -to [get_cells -hier -filter {ASYNC_REG == TRUE}]

# ---------------------------------------------------------------------------
# BOARD-SPECIFIC PINS  (example, verify)
# ---------------------------------------------------------------------------
# --- 100 MHz reference clock : FMC HPC GBTCLK0_M2C (KC705 package pins C8 / C7)  [TODO verify] ---
set_property PACKAGE_PIN C8 [get_ports sys_clk_p]
set_property PACKAGE_PIN C7 [get_ports sys_clk_n]

# --- free-running 200 MHz board clock (KC705 SYSCLK_P/N, bank 33)  [TODO verify against UG810] ---
set_property PACKAGE_PIN AD12 [get_ports free_clk_p]
set_property PACKAGE_PIN AD11 [get_ports free_clk_n]
set_property IOSTANDARD LVDS  [get_ports {free_clk_p free_clk_n}]

# --- PERST# to the M.2 slot (FMC LA pin on the adapter; KC705 package pin C25)  [TODO verify] ---
set_property PACKAGE_PIN C25     [get_ports pcie_perst_n]
set_property IOSTANDARD LVCMOS25 [get_ports pcie_perst_n]     ;# VADJ = 2.5 V on the KC705 HPC
set_property DRIVE 8             [get_ports pcie_perst_n]
set_property SLEW SLOW           [get_ports pcie_perst_n]

# --- GTX lanes: FMC HPC DP0..DP3 -> GTXE2_CHANNEL_X0Y12 .. X0Y15 (quad 118)  [TODO verify] ---
# The package pins of the GTX lanes follow from the LOC.  The IP's own XDC places the GTs at
# X0Y7..X0Y4 (quad 116, the default for PCIE_X0Y0); for the FMC HPC the lanes are moved to quad 118
# here.  These LOCs are applied after the IP's XDC and override it.
# Hierarchy path verified against the generated pcie_7x_0 (u_pcie = pcie7x_wrapper, u_pcie_7x = pcie_7x_0).
set_property LOC GTXE2_CHANNEL_X0Y12 [get_cells {u_pcie/u_pcie_7x/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[0].gt_wrapper_i/gtx_channel.gtxe2_channel_i}]
set_property LOC GTXE2_CHANNEL_X0Y13 [get_cells {u_pcie/u_pcie_7x/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[1].gt_wrapper_i/gtx_channel.gtxe2_channel_i}]
set_property LOC GTXE2_CHANNEL_X0Y14 [get_cells {u_pcie/u_pcie_7x/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[2].gt_wrapper_i/gtx_channel.gtxe2_channel_i}]
set_property LOC GTXE2_CHANNEL_X0Y15 [get_cells {u_pcie/u_pcie_7x/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[3].gt_wrapper_i/gtx_channel.gtxe2_channel_i}]
# pci_exp_txp/txn/rxp/rxn[3:0] are GT pins: no PACKAGE_PIN / IOSTANDARD needed.
# Lane order / polarity on the adapter: see the adapter schematic (the core handles
# lane reversal and polarity inversion automatically).

# --- board reset: the KC705 CPU_RESET push button is ACTIVE HIGH, board_rst_n is active low.
# TODO: invert externally or in a board wrapper; example pin only:
# set_property PACKAGE_PIN AB7     [get_ports board_rst_n]
# set_property IOSTANDARD LVCMOS15 [get_ports board_rst_n]

# --- control / status / user data stream: TODO assign to FMC LA pins, user GPIO or a
# --- second design (data_clk and s_data_* come from the data producer) ---
# set_property PACKAGE_PIN <pin> [get_ports ctrl_start]
# set_property PACKAGE_PIN <pin> [get_ports ctrl_stop]
# set_property PACKAGE_PIN <pin> [get_ports status_link_up]     ;# e.g. KC705 GPIO LEDs (LVCMOS15)
# set_property PACKAGE_PIN <pin> [get_ports status_gen2]
# set_property PACKAGE_PIN <pin> [get_ports status_nvme_ready]
# set_property PACKAGE_PIN <pin> [get_ports status_running]
# set_property PACKAGE_PIN <pin> [get_ports status_done]
# The wide statistic buses (stat_*) and the data stream are meant for a parent design or
# ILA/VIO; for a stand-alone bring-up wrap nvme_host_top in a board-level top.

# ---------------------------------------------------------------------------
# General
# ---------------------------------------------------------------------------
set_property CFGBVS GND                      [current_design]
set_property CONFIG_VOLTAGE 1.8              [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
