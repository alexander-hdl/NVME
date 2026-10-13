# =============================================================================
# RTL file list, in compile order (paths relative to the repository root).
# Used by fpga/scripts/create_project.tcl.  Comment lines start with '#'.
# NOTE: rtl/pcie/pcie7x_wrapper.sv, rtl/nvme_host_top.sv and rtl/board/kc705_top.sv (via nvme_host_top) instantiate the Vivado IP
# `pcie_7x_0` and the Xilinx primitives IBUFDS_GTE2 / BUFG: they cannot be elaborated
# in xsim without the IP.  Simulation lists (sim/tb/tb_*.f) exclude those two files.
# =============================================================================

# --- package (must be first) ---
rtl/nvme_host_pkg.sv

# --- common primitives ---
rtl/common/axis_skid.sv
rtl/common/axis_fifo_sync.sv
rtl/common/axis_fifo_async.sv
rtl/common/sdp_ram.sv
rtl/common/dw_shift.sv
rtl/common/timer_ms.sv

# --- PCIe layer ---
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

# --- NVMe / host memory layer ---
rtl/nvme/prp_synth.sv
rtl/nvme/hostmem_rams.sv
rtl/nvme/hostmem_completer.sv
rtl/nvme/hostmem_writer.sv
rtl/nvme/data_ring.sv
rtl/nvme/nvme_admin.sv
rtl/nvme/nvme_write_engine.sv
rtl/nvme/host_sequencer.sv
rtl/nvme/nvme_host_core.sv

# --- top ---
rtl/nvme_host_top.sv

# --- board top (KC705: pins in fpga/constraints/kc705_board.xdc; also holds the test-data generator) ---
rtl/board/kc705_top.sv
