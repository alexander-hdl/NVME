# =============================================================================
# kc705_fmc_m2_cdc.xdc - clock-domain-crossing constraints, IMPLEMENTATION ONLY
# (create_project.tcl sets USED_IN = implementation).
#
# The clocks txoutclk_x0y0 / clk_125mhz_x0y0 / clk_250mhz_x0y0 / clk_*_mux_x0y0 / userclk1 /
# userclk2 / mmcm_fb are created by the XDC of the pcie_7x_0 IP, free_clk (link_bringup)
# and data_clk are created in kc705_fmc_m2.xdc.
# userclk2 (8 ns) is the user_clk_out / AXI-S clock of the core.
# =============================================================================
set_clock_groups -asynchronous \
  -group [get_clocks free_clk] \
  -group [get_clocks {txoutclk_x0y0 clk_125mhz_x0y0 clk_250mhz_x0y0 clk_125mhz_mux_x0y0 clk_250mhz_mux_x0y0 userclk1 userclk2 mmcm_fb}] \
  -group [get_clocks data_clk]
