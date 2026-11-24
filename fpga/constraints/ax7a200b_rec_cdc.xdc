# =============================================================================
# ax7a200b_rec_cdc.xdc - recorder prototype (ax7a200b_rec_top, WP8, spec 8.3), IMPLEMENTATION ONLY
# (create_project.tcl sets USED_IN = implementation: the pcie_7x clocks do not exist while synth_design treats the
# IP as a black box).
#
# Clock structure
#   free_clk (200 MHz, R4/T4) -> BUFG link_clk -> mig_ddr3 sys_clk_i -> MIG PLL: 400 MHz memory clock, ui_clk 100 MHz
#   (derived clocks of free_clk, one family: related, timed synchronously)
#   pcie_7x: txoutclk / clk_125mhz / clk_250mhz / userclk1 / userclk2 (user_clk 125 MHz) -> second family;
#   the families are NOT in a clock group: set_max_delay -datapath_only 5.0 both ways (see below)
# Crossings (spec 4.3): all of them go through rtl/common primitives (cdc_pulse, cdc_bus, cdc_sync, axis_fifo_async,
# tdp_ram_2clk).  Toggle / synchroniser flip-flops carry ASYNC_REG (false path to their D pins in ax7a200b_rec.xdc);
# the data of cdc_bus (hold_q -> dst_data_q) is bounded by set_max_delay -datapath_only and set_bus_skew.
# =============================================================================
# link_clk and all MIG clocks (derived from free_clk) = one family; PCIe = the other (M7).
# NO set_clock_groups / set_false_path between the families: UG903 precedence is clock groups > false path >
# max delay, so a clock group would silently override every set_max_delay below (e.g. cdc_bus hold_q -> dst_data_q
# of u_cdc_bus_corestat, user_clk -> link_clk) and leave the data unbounded.  Instead, every path between the
# families is bounded by set_max_delay -datapath_only 5.0 (no skew/uncertainty of unrelated clocks is applied);
# the paths into the synchroniser flip-flops are the only exception: ax7a200b_rec.xdc false-paths them (D pins of
# ASYNC_REG cells), a false path overrides max delay for exactly those pins.
set rec_fam_a [get_clocks -include_generated_clocks free_clk]
set rec_fam_b [get_clocks {txoutclk_x0y0 clk_125mhz_x0y0 clk_250mhz_x0y0 clk_125mhz_mux_x0y0 clk_250mhz_mux_x0y0 userclk1 userclk2 mmcm_fb}]
set_max_delay -datapath_only 5.0 -from $rec_fam_a -to $rec_fam_b
set_max_delay -datapath_only 5.0 -from $rec_fam_b -to $rec_fam_a

# Explicit (more specific) bound of the cdc_bus data crossings, same value as the family-wide one above.
# cdc_bus data: source hold register -> destination register, at most one fastest clock period (5 ns) of data path delay
set_max_delay -datapath_only 5.0 \
  -from [get_cells -hier -filter {NAME =~ *u_cdc_bus*/hold_q_reg*}] \
  -to   [get_cells -hier -filter {NAME =~ *u_cdc_bus*/dst_data_q_reg*}]

# multi-bit crossings: skew bound (also applies to false paths and clock groups) (I6)
set_bus_skew -from [get_cells -hier -filter {NAME =~ *wr_gray_q_reg*}] \
             -to   [get_cells -hier -filter {NAME =~ *wr_gray_sync1_q_reg*}] 4.0
set_bus_skew -from [get_cells -hier -filter {NAME =~ *rd_gray_q_reg*}] \
             -to   [get_cells -hier -filter {NAME =~ *rd_gray_sync1_q_reg*}] 4.0
# NO set_bus_skew on cdc_bus (hold_q -> dst_data_q): cdc_bus is a hold-and-toggle handshake - the data is held stable for
# HOLD_CYC source cycles and is captured only after the toggle has crossed the synchroniser, so the per-bit skew between
# hold_q and dst_data_q is irrelevant; only the set_max_delay -datapath_only bound above must hold.  Skew bounds are kept
# for the gray pointers of the async FIFOs only (all bits change together and are sampled at arbitrary times).
# 4.0 ns < the shortest period of the participating domains (5 ns).
# After a build: report_cdc (no unexplained Critical), report_bus_skew, report_clock_interaction.

# GTP sites of bank 216 (same as ax7a200b_cdc.xdc; hierarchy u_host/u_pcie is identical in ax7a200b_rec_top)
set gt_pfx {u_host/u_pcie/u_pcie_7x/inst/inst/gt_top_i/pipe_wrapper_i}
set_property LOC GTPE2_CHANNEL_X0Y7 [get_cells "${gt_pfx}/pipe_lane[0].gt_wrapper_i/gtp_channel.gtpe2_channel_i"]
set_property LOC GTPE2_CHANNEL_X0Y6 [get_cells "${gt_pfx}/pipe_lane[1].gt_wrapper_i/gtp_channel.gtpe2_channel_i"]
set_property LOC GTPE2_CHANNEL_X0Y5 [get_cells "${gt_pfx}/pipe_lane[2].gt_wrapper_i/gtp_channel.gtpe2_channel_i"]
set_property LOC GTPE2_CHANNEL_X0Y4 [get_cells "${gt_pfx}/pipe_lane[3].gt_wrapper_i/gtp_channel.gtpe2_channel_i"]
