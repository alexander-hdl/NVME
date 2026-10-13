// =============================================================================
// pcie7x_wrapper.sv - thin wrapper around the Vivado-generated IP `pcie_7x_0`
// (7 Series Integrated Block for PCI Express v3.3, Root Port, Gen2 x4,
// 128-bit AXI-Stream @ 125 MHz).  ARCHITECTURE.md section 4 / 5.18.
//
// * The 100 MHz reference clock enters through IBUFDS_GTE2; its `O` output is the
//   core's sys_clk.  The ODIV2 output is exported (refclk_odiv2) so the top level
//   can build the free-running clock of link_bringup (BUFG in nvme_host_top).
// * All Root Port options that the design does not use are tied off here.
// * The IP itself is created by fpga/scripts/create_project.tcl.  It is NOT part of
//   rtl/filelist.f: for simulation use the core-facing model in sim/models.
//
// IMPORTANT: port names of `pcie_7x_0` below follow the PG054 v3.3 Root Port
// example design from memory.  Every line marked  "verify against generated
// pcie_7x_0 stub"  must be checked against
//   fpga/ip/pcie_7x_0/pcie_7x_0_stub.v   (or the IP's instantiation template)
// after the IP has been generated (names with/without `_n`, widths).
// =============================================================================
module pcie7x_wrapper (
  // ---- reference clock / reset ----
  input  logic         sys_clk_p,
  input  logic         sys_clk_n,
  input  logic         sys_rst_n,          // async, active low (from link_bringup)
  output logic         refclk_odiv2,       // IBUFDS_GTE2.ODIV2 (fabric-usable copy of the refclk)

  // ---- serial lanes ----
  output logic [3:0]   pci_exp_txp,
  output logic [3:0]   pci_exp_txn,
  input  logic [3:0]   pci_exp_rxp,
  input  logic [3:0]   pci_exp_rxn,

  // ---- user interface ----
  output logic         user_clk_out,
  output logic         user_reset_out,
  output logic         user_lnk_up,

  output logic [127:0] m_axis_rx_tdata,
  output logic [15:0]  m_axis_rx_tkeep,
  output logic         m_axis_rx_tlast,
  output logic         m_axis_rx_tvalid,
  input  logic         m_axis_rx_tready,
  output logic [21:0]  m_axis_rx_tuser,
  input  logic         rx_np_ok,

  input  logic [127:0] s_axis_tx_tdata,
  input  logic [15:0]  s_axis_tx_tkeep,
  input  logic         s_axis_tx_tlast,
  input  logic         s_axis_tx_tvalid,
  output logic         s_axis_tx_tready,
  input  logic [3:0]   s_axis_tx_tuser,
  output logic [5:0]   tx_buf_av,
  input  logic         tx_cfg_gnt,

  // ---- configuration management port (Root Port's own config space) ----
  input  logic [31:0]  cfg_mgmt_di,
  input  logic [3:0]   cfg_mgmt_byte_en,
  input  logic [9:0]   cfg_mgmt_dwaddr,
  input  logic         cfg_mgmt_wr_en,
  input  logic         cfg_mgmt_rd_en,
  output logic [31:0]  cfg_mgmt_do,
  output logic         cfg_mgmt_rd_wr_done,
  input  logic         cfg_mgmt_wr_readonly,
  input  logic         cfg_mgmt_wr_rw1c_as_rw,

  // ---- physical layer status (for link_bringup) ----
  output logic [5:0]   pl_ltssm_state,
  output logic         pl_sel_lnk_rate,
  output logic [1:0]   pl_sel_lnk_width,
  output logic         pl_link_gen2_cap,
  output logic         pl_link_partner_gen2_supported
);

  // ---------------------------------------------------------------------------
  // Reference clock buffer
  // ---------------------------------------------------------------------------
  logic sys_clk;
  IBUFDS_GTE2 #(
    .CLKCM_CFG    ("TRUE"),
    .CLKRCV_TRST  ("TRUE"),
    .CLKSWING_CFG (2'b11)
  ) u_refclk_ibuf (
    .O     (sys_clk),
    .ODIV2 (refclk_odiv2),
    .CEB   (1'b0),
    .I     (sys_clk_p),
    .IB    (sys_clk_n)
  );

  // ---------------------------------------------------------------------------
  // Unused outputs of the IP (kept as nets so the instantiation is explicit)
  // ---------------------------------------------------------------------------
  logic        tx_cfg_req_unused;
  logic        tx_err_drop_unused;
  logic        tx_cmd_err_unused;
  logic        user_app_rdy_unused;
  logic        pl_phy_lnk_up_unused;
  logic [1:0]  pl_lane_reversal_mode_unused;
  logic [2:0]  pl_tx_pm_state_unused;
  logic [1:0]  pl_rx_pm_state_unused;
  logic        pl_link_upcfg_cap_unused;
  logic [2:0]  pl_initial_link_width_unused;
  logic        pl_directed_change_done_unused;
  logic        pl_received_hot_rst_unused;

  pcie_7x_0 u_pcie_7x (
    // ---- PCI Express lanes ----
    .pci_exp_txp (pci_exp_txp),
    .pci_exp_txn (pci_exp_txn),
    .pci_exp_rxp (pci_exp_rxp),
    .pci_exp_rxn (pci_exp_rxn),

    // ---- clock / reset ----
    .sys_clk         (sys_clk),
    .sys_rst_n       (sys_rst_n),
    .user_clk_out    (user_clk_out),
    .user_reset_out  (user_reset_out),
    .user_lnk_up     (user_lnk_up),
    .user_app_rdy    (user_app_rdy_unused),            // verify against generated pcie_7x_0 stub

    // ---- AXI-S transmit ----
    .s_axis_tx_tdata  (s_axis_tx_tdata),
    .s_axis_tx_tkeep  (s_axis_tx_tkeep),
    .s_axis_tx_tlast  (s_axis_tx_tlast),
    .s_axis_tx_tvalid (s_axis_tx_tvalid),
    .s_axis_tx_tready (s_axis_tx_tready),
    .s_axis_tx_tuser  (s_axis_tx_tuser),
    .tx_buf_av        (tx_buf_av),
    .tx_cfg_req       (tx_cfg_req_unused),
    .tx_cfg_gnt       (tx_cfg_gnt),
    .tx_err_drop      (tx_err_drop_unused),             // verify against generated pcie_7x_0 stub
    .tx_cmd_err       (tx_cmd_err_unused),              // verify against generated pcie_7x_0 stub

    // ---- AXI-S receive ----
    .m_axis_rx_tdata  (m_axis_rx_tdata),
    .m_axis_rx_tkeep  (m_axis_rx_tkeep),
    .m_axis_rx_tlast  (m_axis_rx_tlast),
    .m_axis_rx_tvalid (m_axis_rx_tvalid),
    .m_axis_rx_tready (m_axis_rx_tready),
    .m_axis_rx_tuser  (m_axis_rx_tuser),
    .rx_np_ok         (rx_np_ok),
    .rx_np_req        (1'b1),                           // verify against generated pcie_7x_0 stub (unused in rx_np_ok mode)

    // ---- configuration management (cfg_mgmt) ----
    .cfg_mgmt_di               (cfg_mgmt_di),
    .cfg_mgmt_byte_en          (cfg_mgmt_byte_en),
    .cfg_mgmt_dwaddr           (cfg_mgmt_dwaddr),
    .cfg_mgmt_wr_en            (cfg_mgmt_wr_en),
    .cfg_mgmt_rd_en            (cfg_mgmt_rd_en),
    .cfg_mgmt_do               (cfg_mgmt_do),
    .cfg_mgmt_rd_wr_done       (cfg_mgmt_rd_wr_done),
    .cfg_mgmt_wr_readonly      (cfg_mgmt_wr_readonly),
    .cfg_mgmt_wr_rw1c_as_rw    (cfg_mgmt_wr_rw1c_as_rw),

    // ---- Root Port specific tie-offs (research/03 section 4) ----
    .cfg_ds_bus_number      (8'h00),
    .cfg_ds_device_number   (5'h00),
    .cfg_ds_function_number (3'h0),
    .cfg_pm_send_pme_to     (1'b0),                     // verify against generated pcie_7x_0 stub (polarity / name)

    // ---- error reporting inputs: unused ----
    .cfg_err_cor            (1'b0),                     // verify against generated pcie_7x_0 stub
    .cfg_err_ur             (1'b0),                     // (some versions name these cfg_err_*_n, active low:
    .cfg_err_ecrc           (1'b0),                     //  tie the _n variants to 1'b1 in that case)
    .cfg_err_cpl_timeout    (1'b0),
    .cfg_err_cpl_abort      (1'b0),
    .cfg_err_cpl_unexpect   (1'b0),
    .cfg_err_posted         (1'b0),
    .cfg_err_locked         (1'b0),
    .cfg_err_tlp_cpl_header (48'h0),

    // ---- interrupts / power management inputs: unused ----
    .cfg_interrupt          (1'b0),                     // verify against generated pcie_7x_0 stub
    .cfg_trn_pending        (1'b0),
    .cfg_dsn                (64'h0),

    // ---- physical layer control: nothing is forced ----
    .pl_directed_link_change      (2'b00),
    .pl_directed_link_width       (2'b00),
    .pl_directed_link_speed       (1'b0),
    .pl_directed_link_auton       (1'b0),
    .pl_upstream_prefer_deemph    (1'b1),
    .pl_transmit_hot_rst          (1'b0),
    .pl_downstream_deemph_source  (1'b0),

    // ---- physical layer status ----
    .pl_ltssm_state                  (pl_ltssm_state),
    .pl_sel_lnk_rate                 (pl_sel_lnk_rate),
    .pl_sel_lnk_width                (pl_sel_lnk_width),
    .pl_link_gen2_cap                (pl_link_gen2_cap),
    .pl_link_partner_gen2_supported  (pl_link_partner_gen2_supported),
    .pl_lane_reversal_mode           (pl_lane_reversal_mode_unused),
    .pl_phy_lnk_up                   (pl_phy_lnk_up_unused),
    .pl_tx_pm_state                  (pl_tx_pm_state_unused),
    .pl_rx_pm_state                  (pl_rx_pm_state_unused),
    .pl_link_upcfg_cap               (pl_link_upcfg_cap_unused),
    .pl_initial_link_width           (pl_initial_link_width_unused),
    .pl_directed_change_done         (pl_directed_change_done_unused),
    .pl_received_hot_rst             (pl_received_hot_rst_unused)
  );

endmodule
