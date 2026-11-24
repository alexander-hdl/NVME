// =============================================================================
// pcie7x_wrapper_sim.sv - simulation replacement of rtl/pcie/pcie7x_wrapper.sv (WP8).
// Same module name and ports as the real wrapper (which holds the Vivado IP pcie_7x_0), so rtl/nvme_host_top.sv and the
// board top compile unchanged.  Inside: pcie7x_core_model (Root Port core, user_clk 125 MHz) + nvme_ssd_model; the
// SSD's PERST# is the sys_rst_n that link_bringup drives.  The testbench reaches the SSD as <top>.u_host.u_pcie.u_ssd.
// The serial lanes are unused (outputs 0).
// =============================================================================
`timescale 1ns/1ps
module pcie7x_wrapper (
  input  logic         sys_clk_p,
  input  logic         sys_clk_n,
  input  logic         sys_rst_n,
  output logic [3:0]   pci_exp_txp,
  output logic [3:0]   pci_exp_txn,
  input  logic [3:0]   pci_exp_rxp,
  input  logic [3:0]   pci_exp_rxn,
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
  input  logic [31:0]  cfg_mgmt_di,
  input  logic [3:0]   cfg_mgmt_byte_en,
  input  logic [9:0]   cfg_mgmt_dwaddr,
  input  logic         cfg_mgmt_wr_en,
  input  logic         cfg_mgmt_rd_en,
  output logic [31:0]  cfg_mgmt_do,
  output logic         cfg_mgmt_rd_wr_done,
  input  logic         cfg_mgmt_wr_readonly,
  input  logic         cfg_mgmt_wr_rw1c_as_rw,
  output logic [5:0]   pl_ltssm_state,
  output logic         pl_sel_lnk_rate,
  output logic [1:0]   pl_sel_lnk_width,
  output logic         pl_link_gen2_cap,
  output logic         pl_link_partner_gen2_supported
);
  assign pci_exp_txp = 4'b0;
  assign pci_exp_txn = 4'b1111;
  logic tx_cfg_req;

  pcie7x_core_model u_core (
    .sys_rst_n(sys_rst_n), .user_clk_out(user_clk_out), .user_reset_out(user_reset_out), .user_lnk_up(user_lnk_up),
    .m_axis_rx_tdata(m_axis_rx_tdata), .m_axis_rx_tkeep(m_axis_rx_tkeep), .m_axis_rx_tlast(m_axis_rx_tlast),
    .m_axis_rx_tvalid(m_axis_rx_tvalid), .m_axis_rx_tready(m_axis_rx_tready), .m_axis_rx_tuser(m_axis_rx_tuser),
    .rx_np_ok(rx_np_ok), .rx_np_req(1'b0),
    .s_axis_tx_tdata(s_axis_tx_tdata), .s_axis_tx_tkeep(s_axis_tx_tkeep), .s_axis_tx_tlast(s_axis_tx_tlast),
    .s_axis_tx_tvalid(s_axis_tx_tvalid), .s_axis_tx_tready(s_axis_tx_tready), .s_axis_tx_tuser(s_axis_tx_tuser),
    .tx_buf_av(tx_buf_av), .tx_cfg_req(tx_cfg_req), .tx_cfg_gnt(tx_cfg_gnt),
    .cfg_mgmt_di(cfg_mgmt_di), .cfg_mgmt_byte_en(cfg_mgmt_byte_en), .cfg_mgmt_dwaddr(cfg_mgmt_dwaddr),
    .cfg_mgmt_wr_en(cfg_mgmt_wr_en), .cfg_mgmt_rd_en(cfg_mgmt_rd_en), .cfg_mgmt_do(cfg_mgmt_do),
    .cfg_mgmt_rd_wr_done(cfg_mgmt_rd_wr_done), .cfg_mgmt_wr_readonly(cfg_mgmt_wr_readonly),
    .cfg_mgmt_wr_rw1c_as_rw(cfg_mgmt_wr_rw1c_as_rw),
    .pl_ltssm_state(pl_ltssm_state), .pl_sel_lnk_rate(pl_sel_lnk_rate), .pl_sel_lnk_width(pl_sel_lnk_width),
    .pl_link_gen2_cap(pl_link_gen2_cap), .pl_link_partner_gen2_supported(pl_link_partner_gen2_supported),
    .link_up_delay_cycles(500), .gen2_delay_cycles(300));

  // PERST# as on the stand with PERST_DRIVE = 0: the link_bringup pulses never reach the SSD (its reset comes from the
  // adapter's supervisor); the SSD model supports one release only.  Later sessions restart it by CC.EN = 0 / 1.
  logic ssd_up = 1'b0;
  always @(posedge sys_rst_n) ssd_up = 1'b1;
  nvme_ssd_model u_ssd (.clk(user_clk_out), .perst_n(sys_rst_n | ssd_up));
endmodule
