// =============================================================================
// nvme_host_top.sv - board-level top: PCIe Root Port (pcie_7x) + NVMe host.
// ARCHITECTURE.md section 5.18.
//
//   sys_clk_p/n --IBUFDS_GTE2--> pcie_7x ---- user_clk (125 MHz) ----> nvme_host_core
//   free_clk_p/n --IBUFDS--BUFG--> link_clk (free running) --> link_bringup
//                                       |-> sys_rst_n (pcie_7x)   |-> pcie_perst_n (SSD)
//
// Clock domains
//   link_clk   : refclk-derived free-running clock; link_bringup only.  Its outputs
//                (link_ready / gen2 / width) are 2-FF synchronised into user_clk here.
//   user_clk   : pcie_7x user_clk_out; everything in nvme_host_core.  ctrl_start/stop
//                are 2-FF synchronised into it.
//   data_clk   : user data stream; crossed inside data_ring.  data_rst is a
//                reset synchroniser (async assert, sync release) on data_clk, asserted by
//                the board reset OR the core reset so both sides of the async FIFO restart.
//
// LINK_CLK_HZ must be the real frequency of the free_clk_p/n board oscillator
// (KC705 SYSCLK = 200 MHz).  It cannot come from IBUFDS_GTE2.ODIV2: the pcie_7x IP already
// feeds a BUFG from IBUFDS_GTE2.O and ODIV2 shares that output path (route_design fails with
// "Route 35-54 IBUFDS_GTE2.O->BUFGCTRL.I0 unroutable" when both are used).
// SIM_FAST shortens all "millisecond" timers (microsecond ticks).
// =============================================================================
module nvme_host_top
  import nvme_host_pkg::*;
#(
  parameter int unsigned N_SLOTS            = N_SLOTS_DEF,
  parameter int unsigned SLOT_BYTES         = SLOT_BYTES_DEF,
  parameter bit          SIM_FAST           = 1'b0,
  parameter int unsigned REQ_TIMEOUT_CYCLES = 1_250_000,
  parameter int unsigned LINK_CLK_HZ        = 200_000_000,
  parameter logic [63:0] START_LBA          = 64'd0
) (
  // ---- board ----
  input  logic         sys_clk_p,            // 100 MHz PCIe reference clock (shared with the SSD)
  input  logic         sys_clk_n,
  input  logic         free_clk_p,           // free-running board oscillator (KC705 SYSCLK 200 MHz), runs link_bringup
  input  logic         free_clk_n,
  input  logic         board_rst_n,          // board reset, active low
  output logic         pcie_perst_n,         // to the M.2 PERST#
  output logic [3:0]   pci_exp_txp,
  output logic [3:0]   pci_exp_txn,
  input  logic [3:0]   pci_exp_rxp,
  input  logic [3:0]   pci_exp_rxn,

  // ---- user data stream (full 16-byte beats) ----
  input  logic         data_clk,
  input  logic [127:0] s_data_tdata,
  input  logic         s_data_tvalid,
  output logic         s_data_tready,

  // ---- control / status ----
  input  logic         ctrl_start,
  input  logic         ctrl_stop,
  output logic         status_link_up,
  output logic         status_gen2,
  output logic         status_enum_done,
  output logic         status_nvme_ready,
  output logic         status_running,
  output logic         status_done,
  output logic [7:0]   status_error,
  output logic [7:0]   status_error_detail,   // extra: fail_step of the failing module
  output logic [1:0]   status_link_width,     // extra
  output logic         status_lba_range_err,  // extra: namespace end reached
  output logic [63:0]  stat_bytes_done,
  output logic [63:0]  stat_cmds_done,        // extra
  output logic [31:0]  stat_cmd_errors,
  output logic [31:0]  stat_max_latency,
  output logic         user_clk_out
);

  // ===========================================================================
  // pcie_7x + reference clock
  // ===========================================================================
  logic         free_clk, link_clk;
  logic         sys_rst_n, perst_n_i;
  logic         user_clk, user_reset, user_lnk_up;

  logic [127:0] m_axis_rx_tdata;  logic [15:0] m_axis_rx_tkeep;  logic m_axis_rx_tlast;
  logic         m_axis_rx_tvalid, m_axis_rx_tready;               logic [21:0] m_axis_rx_tuser;
  logic         rx_np_ok;
  logic [127:0] s_axis_tx_tdata;  logic [15:0] s_axis_tx_tkeep;  logic s_axis_tx_tlast;
  logic         s_axis_tx_tvalid, s_axis_tx_tready;               logic [3:0] s_axis_tx_tuser;
  logic [5:0]   tx_buf_av;
  logic         tx_cfg_gnt;

  logic [31:0]  cfg_mgmt_di, cfg_mgmt_do;
  logic [3:0]   cfg_mgmt_byte_en;
  logic [9:0]   cfg_mgmt_dwaddr;
  logic         cfg_mgmt_wr_en, cfg_mgmt_rd_en, cfg_mgmt_rd_wr_done, cfg_mgmt_wr_readonly, cfg_mgmt_wr_rw1c_as_rw;

  logic [5:0]   pl_ltssm_state;
  logic         pl_sel_lnk_rate, pl_link_gen2_cap, pl_link_partner_gen2_supported;
  logic [1:0]   pl_sel_lnk_width;

  pcie7x_wrapper u_pcie (
    .sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n), .sys_rst_n(sys_rst_n),
    .pci_exp_txp(pci_exp_txp), .pci_exp_txn(pci_exp_txn), .pci_exp_rxp(pci_exp_rxp), .pci_exp_rxn(pci_exp_rxn),
    .user_clk_out(user_clk), .user_reset_out(user_reset), .user_lnk_up(user_lnk_up),
    .m_axis_rx_tdata(m_axis_rx_tdata), .m_axis_rx_tkeep(m_axis_rx_tkeep), .m_axis_rx_tlast(m_axis_rx_tlast),
    .m_axis_rx_tvalid(m_axis_rx_tvalid), .m_axis_rx_tready(m_axis_rx_tready), .m_axis_rx_tuser(m_axis_rx_tuser),
    .rx_np_ok(rx_np_ok),
    .s_axis_tx_tdata(s_axis_tx_tdata), .s_axis_tx_tkeep(s_axis_tx_tkeep), .s_axis_tx_tlast(s_axis_tx_tlast),
    .s_axis_tx_tvalid(s_axis_tx_tvalid), .s_axis_tx_tready(s_axis_tx_tready), .s_axis_tx_tuser(s_axis_tx_tuser),
    .tx_buf_av(tx_buf_av), .tx_cfg_gnt(tx_cfg_gnt),
    .cfg_mgmt_di(cfg_mgmt_di), .cfg_mgmt_byte_en(cfg_mgmt_byte_en), .cfg_mgmt_dwaddr(cfg_mgmt_dwaddr),
    .cfg_mgmt_wr_en(cfg_mgmt_wr_en), .cfg_mgmt_rd_en(cfg_mgmt_rd_en), .cfg_mgmt_do(cfg_mgmt_do),
    .cfg_mgmt_rd_wr_done(cfg_mgmt_rd_wr_done), .cfg_mgmt_wr_readonly(cfg_mgmt_wr_readonly),
    .cfg_mgmt_wr_rw1c_as_rw(cfg_mgmt_wr_rw1c_as_rw),
    .pl_ltssm_state(pl_ltssm_state), .pl_sel_lnk_rate(pl_sel_lnk_rate), .pl_sel_lnk_width(pl_sel_lnk_width),
    .pl_link_gen2_cap(pl_link_gen2_cap), .pl_link_partner_gen2_supported(pl_link_partner_gen2_supported));

  assign user_clk_out = user_clk;

  // ===========================================================================
  // Link bring-up on the free-running refclk-derived clock
  // ===========================================================================
  IBUFDS u_free_clk_ibuf (.I(free_clk_p), .IB(free_clk_n), .O(free_clk));
  BUFG   u_link_clk_bufg (.I(free_clk), .O(link_clk));

  // board reset -> link_clk domain (async assert, synchronous release)
  (* ASYNC_REG = "TRUE" *) logic [1:0] link_rst_sync;
  always_ff @(posedge link_clk or negedge board_rst_n) begin
    if (!board_rst_n) link_rst_sync <= 2'b11;
    else              link_rst_sync <= {link_rst_sync[0], 1'b0};
  end
  logic link_rst;
  assign link_rst = link_rst_sync[1];

  logic       link_ready_lc, link_gen2_lc;
  logic [1:0] link_width_lc;

  link_bringup #(
    .CLK_HZ(LINK_CLK_HZ), .SIM_FAST(SIM_FAST)
  ) u_link_bringup (
    .clk(link_clk), .rst(link_rst),
    .user_lnk_up(user_lnk_up), .pl_ltssm_state(pl_ltssm_state), .pl_sel_lnk_rate(pl_sel_lnk_rate),
    .pl_sel_lnk_width(pl_sel_lnk_width), .pl_link_gen2_cap(pl_link_gen2_cap),
    .pl_link_partner_gen2_supported(pl_link_partner_gen2_supported),
    .perst_n(perst_n_i), .sys_rst_n(sys_rst_n),
    .link_ready(link_ready_lc), .link_gen2(link_gen2_lc), .link_width(link_width_lc));

  assign pcie_perst_n = perst_n_i;

  // ===========================================================================
  // Synchronisers into user_clk
  // ===========================================================================
  (* ASYNC_REG = "TRUE" *) logic [1:0] link_ready_sync, link_gen2_sync, ctrl_start_sync, ctrl_stop_sync;
  (* ASYNC_REG = "TRUE" *) logic [1:0] link_width_sync0, link_width_sync1;   // per-bit 2-FF (quasi-static)
  always_ff @(posedge user_clk) begin
    link_ready_sync    <= {link_ready_sync[0],  link_ready_lc};
    link_gen2_sync     <= {link_gen2_sync[0],   link_gen2_lc};
    link_width_sync0   <= link_width_lc;
    link_width_sync1   <= link_width_sync0;
    ctrl_start_sync    <= {ctrl_start_sync[0],  ctrl_start};
    ctrl_stop_sync     <= {ctrl_stop_sync[0],   ctrl_stop};
  end

  logic       link_ready_uc, link_gen2_uc, ctrl_start_uc, ctrl_stop_uc;
  logic [1:0] link_width_uc;
  assign link_ready_uc = link_ready_sync[1];
  assign link_gen2_uc  = link_gen2_sync[1];
  assign link_width_uc = link_width_sync1;
  assign ctrl_start_uc = ctrl_start_sync[1];
  assign ctrl_stop_uc  = ctrl_stop_sync[1];

  // data_clk reset: board reset or core reset, released synchronously on data_clk
  logic data_rst_async;
  assign data_rst_async = ~board_rst_n | user_reset;
  (* ASYNC_REG = "TRUE" *) logic [1:0] data_rst_sync;
  always_ff @(posedge data_clk or posedge data_rst_async) begin
    if (data_rst_async) data_rst_sync <= 2'b11;
    else                data_rst_sync <= {data_rst_sync[0], 1'b0};
  end

  // ===========================================================================
  // NVMe host core
  // ===========================================================================
  nvme_host_core #(
    .N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES), .SIM_FAST(SIM_FAST),
    .REQ_TIMEOUT_CYCLES(REQ_TIMEOUT_CYCLES), .START_LBA(START_LBA)
  ) u_core (
    .user_clk(user_clk), .user_rst(user_reset),
    .m_axis_rx_tdata(m_axis_rx_tdata), .m_axis_rx_tkeep(m_axis_rx_tkeep), .m_axis_rx_tlast(m_axis_rx_tlast),
    .m_axis_rx_tvalid(m_axis_rx_tvalid), .m_axis_rx_tready(m_axis_rx_tready), .m_axis_rx_tuser(m_axis_rx_tuser),
    .rx_np_ok(rx_np_ok),
    .s_axis_tx_tdata(s_axis_tx_tdata), .s_axis_tx_tkeep(s_axis_tx_tkeep), .s_axis_tx_tlast(s_axis_tx_tlast),
    .s_axis_tx_tvalid(s_axis_tx_tvalid), .s_axis_tx_tready(s_axis_tx_tready), .s_axis_tx_tuser(s_axis_tx_tuser),
    .tx_buf_av(tx_buf_av), .tx_cfg_gnt(tx_cfg_gnt),
    .cfg_mgmt_di(cfg_mgmt_di), .cfg_mgmt_byte_en(cfg_mgmt_byte_en), .cfg_mgmt_dwaddr(cfg_mgmt_dwaddr),
    .cfg_mgmt_wr_en(cfg_mgmt_wr_en), .cfg_mgmt_rd_en(cfg_mgmt_rd_en), .cfg_mgmt_do(cfg_mgmt_do),
    .cfg_mgmt_rd_wr_done(cfg_mgmt_rd_wr_done), .cfg_mgmt_wr_readonly(cfg_mgmt_wr_readonly),
    .cfg_mgmt_wr_rw1c_as_rw(cfg_mgmt_wr_rw1c_as_rw),
    .link_ready(link_ready_uc), .link_gen2(link_gen2_uc), .link_width(link_width_uc),
    .data_clk(data_clk), .data_rst(data_rst_sync[1]),
    .s_data_tdata(s_data_tdata), .s_data_tvalid(s_data_tvalid), .s_data_tready(s_data_tready),
    .ctrl_start(ctrl_start_uc), .ctrl_stop(ctrl_stop_uc),
    .status_link_up(status_link_up), .status_gen2(status_gen2), .status_link_width(status_link_width),
    .status_enum_done(status_enum_done), .status_nvme_ready(status_nvme_ready),
    .status_running(status_running), .status_done(status_done),
    .status_error(status_error), .status_error_detail(status_error_detail),
    .status_seq_state(), .status_lba_range_err(status_lba_range_err), .status_shutdown_timeout(),
    .stat_bytes_done(stat_bytes_done), .stat_cmds_done(stat_cmds_done), .stat_cmd_errors(stat_cmd_errors),
    .stat_max_latency(stat_max_latency), .stat_last_status(), .stat_next_lba(), .stat_beats_in(),
    .stat_stall_cycles(), .stat_sq_full_stall_cycles(), .stat_unexpected_cpl(),
    .stat_lbads(), .stat_nsze(), .stat_mdts());

endmodule
