# =============================================================================
# kc705_fmc_m2.xdc - FMC / PCIe part of the constraints for nvme_host_top on a KC705
# (XC7K325T-2 FFG900) with an M.2 NVMe SSD on an FMC HPC adapter board (e.g. the Opsero FMC M.2
# adapter): PCIe reference clock, PERST#, GT LOCs.  SYSCLK / buttons / LEDs / DIP / CPU_RESET /
# config properties are in kc705_board.xdc (board top kc705_top), the CDC groups in
# kc705_fmc_m2_cdc.xdc.
#
# STATUS: the pins below are taken from the documents, not verified against the KC705 user
# guide (UG810) and the FMC adapter schematic.  Items marked TODO are mandatory before the
# first hardware run.
# =============================================================================

# ---------------------------------------------------------------------------
# Clocks
# ---------------------------------------------------------------------------
# 100 MHz PCIe reference clock from the adapter -> FMC HPC GBTCLK0_M2C
create_clock -name sys_clk -period 10.000 [get_ports sys_clk_p]

# free_clk (KC705 SYSCLK 200 MHz, link_bringup / PERST# timing) and its pins: kc705_board.xdc.
# data_clk: kc705_top feeds nvme_host_top.data_clk from user_clk_out (userclk2 of the IP), so the
# data stream needs no clock of its own.

# ---------------------------------------------------------------------------
# Clock-domain crossings: see kc705_fmc_m2_cdc.xdc (implementation only, because the
# clocks of the pcie_7x_0 IP do not exist while synth_design runs it as a black box).
#   user_clk (pcie_7x userclk2, 125 MHz = data_clk) <-> link_clk (free_clk 200 MHz via BUFG)
# Crossings: 2-FF synchronisers (ASYNC_REG) and reset synchronisers in nvme_host_top,
# kc705_top and link_bringup, plus the gray-coded async FIFO in data_ring.
# ---------------------------------------------------------------------------
# Pure synchroniser inputs:
set_false_path -to [get_cells -hier -filter {ASYNC_REG == TRUE}]

# ---------------------------------------------------------------------------
# FMC / PCIe pins  (verify)
# ---------------------------------------------------------------------------
# --- 100 MHz reference clock : FMC HPC GBTCLK0_M2C (KC705 package pins C8 / C7)  [TODO verify] ---
set_property PACKAGE_PIN C8 [get_ports sys_clk_p]
set_property PACKAGE_PIN C7 [get_ports sys_clk_n]

# --- PERST# to the M.2 slot (FMC LA pin on the adapter; KC705 package pin C25)  [TODO verify] ---
set_property PACKAGE_PIN C25     [get_ports pcie_perst_n]
set_property IOSTANDARD LVCMOS25 [get_ports pcie_perst_n]     ;# VADJ = 2.5 V on the KC705 HPC
set_property DRIVE 8             [get_ports pcie_perst_n]
set_property SLEW SLOW           [get_ports pcie_perst_n]

# --- GTX lanes: FMC HPC DP0..DP3 -> GTXE2_CHANNEL_X0Y12 .. X0Y15 (quad 118)  [TODO verify] ---
# The package pins of the GTX lanes follow from the LOC.  The IP's own XDC places the GTs at
# X0Y7..X0Y4 (quad 116, the default for PCIE_X0Y0); for the FMC HPC the lanes are moved to quad 118
# here.  These LOCs are applied after the IP's XDC and override it.
# Hierarchy path verified against the generated pcie_7x_0 (u_pcie = pcie7x_wrapper, u_pcie_7x = pcie_7x_0);
# In kc705_top nvme_host_top is instance u_host, so the path gets that prefix (nvme_host_top as top: none).
set gt_pfx [expr {[llength [get_cells -quiet u_host]] ? {u_host/} : {}}]
set_property LOC GTXE2_CHANNEL_X0Y12 [get_cells "${gt_pfx}u_pcie/u_pcie_7x/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[0].gt_wrapper_i/gtx_channel.gtxe2_channel_i"]
set_property LOC GTXE2_CHANNEL_X0Y13 [get_cells "${gt_pfx}u_pcie/u_pcie_7x/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[1].gt_wrapper_i/gtx_channel.gtxe2_channel_i"]
set_property LOC GTXE2_CHANNEL_X0Y14 [get_cells "${gt_pfx}u_pcie/u_pcie_7x/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[2].gt_wrapper_i/gtx_channel.gtxe2_channel_i"]
set_property LOC GTXE2_CHANNEL_X0Y15 [get_cells "${gt_pfx}u_pcie/u_pcie_7x/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[3].gt_wrapper_i/gtx_channel.gtxe2_channel_i"]
# pci_exp_txp/txn/rxp/rxn[3:0] are GT pins: no PACKAGE_PIN / IOSTANDARD needed.
# Lane order / polarity on the adapter: see the adapter schematic (the core handles
# lane reversal and polarity inversion automatically).
