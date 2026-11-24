# =============================================================================
# rtl/filelist_rec.f - RTL file list of the recorder prototype (board=ax7a200b_rec, top ax7a200b_rec_top, WP8).
# Same conventions as filelist.f (compile order, paths relative to the repository root, '#' comments).
# The old board tops (kc705_top, ax7a200b_top) are not part of this list.  Not in the list: the generated IPs
# pcie_7x_0, vio_status, ila_pcie, mig_ddr3 (created by create_project.tcl / mig_ip.tcl).
# Simulation lists: sim/tb/tb_rec_top.f (pcie7x_wrapper_sim / mig_app_model / vio_status_sim instead of the IPs).
# =============================================================================

# --- packages (must be first) ---
rtl/nvme_host_pkg.sv
rtl/rec/rec_pkg.sv

# --- common primitives (+ CDC primitives, WP0) ---
rtl/common/axis_skid.sv
rtl/common/axis_fifo_sync.sv
rtl/common/axis_fifo_async.sv
rtl/common/sdp_ram.sv
rtl/common/dw_shift.sv
rtl/common/timer_ms.sv
rtl/common/cdc_sync.sv
rtl/common/cdc_pulse.sv
rtl/common/cdc_bus.sv
rtl/common/tdp_ram_2clk.sv

# --- PCIe / NVMe / host core / host top ---
rtl/pcie/pcie7x_rx_adapt.sv
rtl/pcie/pcie7x_tx_adapt.sv
rtl/pcie/tlp_rx_demux.sv
rtl/pcie/tlp_tx_arb.sv
rtl/pcie/rp_requester.sv
rtl/pcie/req_mux.sv
rtl/pcie/link_bringup.sv
rtl/pcie/rp_cfg_init.sv
rtl/pcie/pcie_enum.sv
rtl/pcie/pcie7x_wrapper.sv
rtl/nvme/prp_synth.sv
rtl/nvme/hostmem_rams.sv
rtl/nvme/hostmem_completer.sv
rtl/nvme/hostmem_writer.sv
rtl/nvme/data_ring.sv
rtl/nvme/nvme_admin.sv
rtl/nvme/nvme_write_engine.sv
rtl/nvme/verify_checker.sv
rtl/nvme/host_sequencer.sv
rtl/nvme/nvme_host_core.sv
rtl/nvme_host_top.sv

# --- recorder (rtl/rec) ---
rtl/rec/uart_rx.sv
rtl/rec/uart_tx.sv
rtl/rec/rec_cmd_parser.sv
rtl/rec/rec_tx_fmt.sv
rtl/rec/rec_ctrl.sv
rtl/rec/xadc_mon.sv
rtl/rec/rec_cdc_bridge.sv
rtl/rec/rec_profile.sv
rtl/rec/rec_crc128.sv
rtl/rec/rec_framer.sv
rtl/rec/rec_sink_chk.sv
rtl/rec/ddr_memtest.sv
rtl/rec/ddr_fifo.sv
rtl/rec/vrf_tap.sv
rtl/rec/rec_verify.sv

# --- board top ---
rtl/board/ax7a200b_rec_top.sv
