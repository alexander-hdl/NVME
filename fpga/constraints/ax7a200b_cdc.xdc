# =============================================================================
# ax7a200b_cdc.xdc - IMPLEMENTATION ONLY (create_project.tcl sets USED_IN = implementation):
# clock groups (the IP's clocks do not exist while synth_design treats pcie_7x_0 as a black box)
# and the GTP LOCs of the four IP lanes.
#
# GTP sites of bank 216 (Vivado 2025.2 package query, xc7a200tfbg484):
#   GTPE2_CHANNEL_X0Y7 = ch3 (TX D7/C7, RX D9/C9)    -> slot lane 0  -> IP lane 0
#   GTPE2_CHANNEL_X0Y6 = ch2 (TX B6/A6, RX B10/A10)  -> slot lane 1  -> IP lane 1
#   GTPE2_CHANNEL_X0Y5 = ch1 (TX D5/C5, RX D11/C11)  -> SFP2 cage    -> IP lane 2 (cage empty: not detected)
#   GTPE2_CHANNEL_X0Y4 = ch0 (TX B4/A4, RX B8/A8)    -> SFP1 cage    -> IP lane 3 (cage empty: not detected)
# The x4-configured core trains to x2 (variant A, STAND/AX7A200B_ARCHITECTURE.md 4.2).
# =============================================================================
set_clock_groups -asynchronous \
  -group [get_clocks free_clk] \
  -group [get_clocks {txoutclk_x0y0 clk_125mhz_x0y0 clk_250mhz_x0y0 clk_125mhz_mux_x0y0 clk_250mhz_mux_x0y0 userclk1 userclk2 mmcm_fb}]

set gt_pfx {u_host/u_pcie/u_pcie_7x/inst/inst/gt_top_i/pipe_wrapper_i}
set_property LOC GTPE2_CHANNEL_X0Y7 [get_cells "${gt_pfx}/pipe_lane[0].gt_wrapper_i/gtp_channel.gtpe2_channel_i"]
set_property LOC GTPE2_CHANNEL_X0Y6 [get_cells "${gt_pfx}/pipe_lane[1].gt_wrapper_i/gtp_channel.gtpe2_channel_i"]
set_property LOC GTPE2_CHANNEL_X0Y5 [get_cells "${gt_pfx}/pipe_lane[2].gt_wrapper_i/gtp_channel.gtpe2_channel_i"]
set_property LOC GTPE2_CHANNEL_X0Y4 [get_cells "${gt_pfx}/pipe_lane[3].gt_wrapper_i/gtp_channel.gtpe2_channel_i"]
