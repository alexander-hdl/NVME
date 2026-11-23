// =============================================================================
// nvme_host_core.sv - everything between the pcie_7x AXI-S ports and the user
// data stream.  One clock domain (user_clk, 125 MHz) except the user data input
// (data_clk), which is crossed inside data_ring.   ARCHITECTURE.md section 4
//
//  pcie_7x RX ---> pcie7x_rx_adapt ---> tlp_rx_demux --+-- Cpl  --> rp_requester
//                                                      +-- MRd  --> hostmem_completer
//                                                      +-- MWr  --> hostmem_writer
//  pcie_7x TX <--- pcie7x_tx_adapt <--- tlp_tx_arb <---+-- CplD <-- hostmem_completer
//                                                      +-- req  <-- rp_requester
//
//  rp_requester command port  <-- req_mux <-- port 0: pcie_enum | nvme_admin (never active together)
//                                          <-- port 1: nvme_write_engine (doorbells)
//
//  hostmem_rams: asq / iosq / acq / iocq / ident RAMs + PRP synthesis.
//     asq    <- nvme_admin            iosq   <- nvme_write_engine
//     acq    -> nvme_admin            iocq   -> nvme_write_engine
//     ident  -> nvme_admin            (acq/iocq/ident are written by hostmem_writer)
//  data_ring: user stream -> BRAM ring -> read by the completer (RSEL_DATA);
//     slot_full -> nvme_write_engine, slot_free <- nvme_write_engine.
//  verify_checker: verify mode only - the SSD's Read data (MWr into the DATA region, decoded
//     by hostmem_writer as wr_en_data) is compared with the test pattern on the fly.
//  host_sequencer: link -> enum -> Root Port cfg -> NVMe init -> run -> drain
//     -> flush -> shutdown (see host_sequencer.sv).
//
// Inputs ctrl_start / ctrl_stop / link_ready / link_gen2 / link_width must already be
// synchronous to user_clk (nvme_host_top synchronises them).  Stop semantics: stop the data
// stream first, then assert ctrl_stop; a partially filled ring slot is never written to the SSD.
//
// link_lost / link_fail are the STICKY flags of link_bringup (link_clk domain, cleared only by
// the board reset).  They are asynchronous to user_clk and are synchronised HERE (2-FF,
// ASYNC_REG, no reset on purpose: user_reset must not wipe them).  link_lost keeps the
// sequencer in IDLE with E_LINK_LOST after a link drop (the IP's user_reset would otherwise
// erase the error and the still-high ctrl_start would restart the recording over its own data);
// link_fail gives E_LINK_TIMEOUT.  Both default to 0 when left unconnected.
//
// Verify mode (verify_mode = 1, quasi-static, set before ctrl_start; ARCHITECTURE.md 5.19).  The
// write engine issues NVMe Reads of the LBA range that a previous run recorded (the first
// verify_cmds commands of SLOT_BYTES each, starting at START_LBA; 0 = unlimited) instead of
// Writes; verify_checker compares every received byte with the pattern.  The user keeps the
// data stream OFF (data_ring stays empty) and ends the run with ctrl_stop when cmd_limit_done
// is high.  slot_free is not forwarded to data_ring in this mode.  vfy_* hold the result (valid
// once the sequencer is past RUN; the checker needs 8 cycles after the last data beat).
// vfy_* are cleared by the restart pulse of every run (init_start).
//
// Format NVM (format_req = 1, quasi-static, sampled at init_start): nvme_admin issues Format NVM
// (SES=0) between Identify Namespace and Set Features, i.e. before the I/O queues exist - the SSD
// returns to the "fresh" state (all LBAs deallocated).  format_done / format_ms report the result.
// The sequencer waits in ADMIN_INIT as long as nvme_admin needs (FORMAT_TIMEOUT_MS in nvme_admin).
//
// Data path.  s_data_tready is held low while the core is in reset, while data_rst is
// active and until the sequencer is in RUN (accept_enable = running) - see data_ring.
// The write engine is restarted (pointers, flags, statistics) on every init_start, so
// DONE/ERROR -> IDLE -> start runs the whole flow again without a reset.
//
// status_flags[7:0] (all sticky until reset / the next init_start where noted):
//   [0] shutdown_timeout  SHST=10b not seen (or a bus error) during the NVMe shutdown
//   [1] doorbell_err      a doorbell MWr was not completed with SC
//   [2] io_timeout        a Write / the Flush did not complete within IO_TIMEOUT_MS
//   [3] flush_skipped     the Flush was not executed (hung command after io_timeout)
//   [4] link_fail         link never came up (link_bringup retries exhausted)
//   [5] link_lost         the link dropped after link-up (survives user_reset)
//   [6] io_error          a Write completed with status != 0 (stat_first_error_lba = its SLBA)
//   [7] rx_drop           an unexpected TLP was dropped (demux: IO/Cfg/unknown/poisoned, writer: unmapped MWr).
//                         Messages are not counted here but in stat_rx_msgs: they are discarded by design.
// =============================================================================
module nvme_host_core
  import nvme_host_pkg::*;
#(
  parameter int unsigned N_SLOTS            = N_SLOTS_DEF,
  parameter int unsigned SLOT_BYTES         = SLOT_BYTES_DEF,
  parameter bit          SIM_FAST           = 1'b0,
  parameter int unsigned REQ_TIMEOUT_CYCLES = 1_250_000,        // 10 ms @ 125 MHz
  parameter int unsigned CLK_HZ             = 125_000_000,
  parameter int unsigned IO_TIMEOUT_MS      = 30000,            // write engine: per-command timeout (0 = off)
  parameter logic [63:0] START_LBA          = 64'd0               // first LBA written
) (
  input  logic         user_clk,
  input  logic         user_rst,                 // synchronous, active high

  // ---- pcie_7x core, AXI-S RX (128 bit) ----
  input  logic [127:0] m_axis_rx_tdata,
  input  logic [15:0]  m_axis_rx_tkeep,
  input  logic         m_axis_rx_tlast,
  input  logic         m_axis_rx_tvalid,
  output logic         m_axis_rx_tready,
  input  logic [21:0]  m_axis_rx_tuser,
  output logic         rx_np_ok,
  // ---- pcie_7x core, AXI-S TX (128 bit) ----
  output logic [127:0] s_axis_tx_tdata,
  output logic [15:0]  s_axis_tx_tkeep,
  output logic         s_axis_tx_tlast,
  output logic         s_axis_tx_tvalid,
  input  logic         s_axis_tx_tready,
  output logic [3:0]   s_axis_tx_tuser,
  input  logic [5:0]   tx_buf_av,
  output logic         tx_cfg_gnt,
  // ---- pcie_7x core, management port (Root Port's own config space) ----
  output logic [31:0]  cfg_mgmt_di,
  output logic [3:0]   cfg_mgmt_byte_en,
  output logic [9:0]   cfg_mgmt_dwaddr,
  output logic         cfg_mgmt_wr_en,
  output logic         cfg_mgmt_rd_en,
  input  logic [31:0]  cfg_mgmt_do,
  input  logic         cfg_mgmt_rd_wr_done,
  output logic         cfg_mgmt_wr_readonly,
  output logic         cfg_mgmt_wr_rw1c_as_rw,

  // ---- link status (already synchronised to user_clk) ----
  input  logic         link_ready,
  input  logic         link_gen2,
  input  logic [1:0]   link_width,
  // sticky link_bringup flags, link_clk domain (synchronised inside; unconnected = 0)
  input  logic         link_lost = 1'b0,
  input  logic         link_fail = 1'b0,

  // ---- user data stream (data_clk domain; full 16-byte beats only) ----
  input  logic         data_clk,
  input  logic         data_rst,
  input  logic [127:0] s_data_tdata,
  input  logic         s_data_tvalid,
  output logic         s_data_tready,

  // ---- control (synchronous to user_clk) ----
  input  logic         ctrl_start,
  input  logic         ctrl_stop,
  input  logic         verify_mode = 1'b0,       // 1: read back and compare instead of recording
  input  logic [31:0]  verify_cmds = 32'd0,    // verify: number of Read commands (0 = unlimited)
  input  logic         format_req = 1'b0,        // 1: issue NVMe Format NVM during bring-up (sampled at init_start; quasi-static)

  // ---- status ----
  output logic         status_link_up,
  output logic         status_gen2,
  output logic [1:0]   status_link_width,
  output logic         status_enum_done,
  output logic         status_nvme_ready,
  output logic         status_running,
  output logic         status_done,
  output logic [7:0]   status_error,
  output logic [7:0]   status_error_detail,
  output logic [3:0]   status_seq_state,
  output logic         status_lba_range_err,
  output logic         status_shutdown_timeout,
  output logic [7:0]   status_flags,             // see the bit map in the header
  output logic [63:0]  stat_bytes_done,
  output logic [63:0]  stat_cmds_done,
  output logic [31:0]  stat_cmd_errors,
  output logic [31:0]  stat_max_latency,
  output logic         stat_cpl_evt,             // pulse: a data command completed (write engine cpl_evt)
  output logic [31:0]  stat_cpl_lat,             // its latency in user_clk cycles
  output logic [15:0]  stat_rx_msgs,             // received Messages (discarded), saturating, cleared by user_rst
  output logic [15:0]  stat_last_status,
  output logic [63:0]  stat_next_lba,
  output logic [63:0]  stat_beats_in,
  output logic [31:0]  stat_stall_cycles,
  output logic [31:0]  stat_sq_full_stall_cycles,
  output logic [15:0]  stat_unexpected_cpl,
  output logic [3:0]   stat_lbads,
  output logic [63:0]  stat_nsze,
  output logic [7:0]   stat_mdts,
  output logic [63:0]  stat_first_error_lba,     // SLBA of the first failed Write (status_flags[6])
  output logic         format_done,              // sticky: Format NVM completed OK (cleared at init_start)
  output logic [31:0]  format_ms,                // ms the Format NVM took (SQ doorbell -> CQE)
  // ---- verify results (user_clk) ----
  output logic         cmd_limit_done,           // verify_cmds data commands submitted and completed
  output logic [63:0]  vfy_bytes,                // bytes compared
  output logic [31:0]  vfy_err_beats,            // 16-byte beats with a mismatch
  output logic [63:0]  vfy_first_err_lba,        // LBA of the first mismatching beat
  output logic [31:0]  vfy_first_err_exp,        // expected / received DW of its first bad DW
  output logic [31:0]  vfy_first_err_got
);

  // ===========================================================================
  // Common
  // ===========================================================================
  logic tick_ms;
  timer_ms #(.CLK_HZ(CLK_HZ), .SIM_FAST(SIM_FAST ? 1 : 0)) u_timer_ms (
    .clk(user_clk), .rst(user_rst), .tick_ms(tick_ms));

  assign tx_cfg_gnt = 1'b1;     // never delay the core's own config completions

  // ===========================================================================
  // Internal TLP streams (tlp_axis: tdata/tkeep/tlast/tuser[1:0]/tvalid/tready)
  // ===========================================================================
  // RX adapter -> demux
  logic [127:0] rx_tdata;  logic [15:0] rx_tkeep;  logic rx_tlast;  logic [1:0] rx_tuser;  logic rx_tvalid;  logic rx_tready;
  // demux -> requester (completions)
  logic [127:0] cpl_tdata; logic [15:0] cpl_tkeep; logic cpl_tlast; logic [1:0] cpl_tuser; logic cpl_tvalid; logic cpl_tready;
  // demux -> completer (memory reads)
  logic [127:0] mrd_tdata; logic [15:0] mrd_tkeep; logic mrd_tlast; logic [1:0] mrd_tuser; logic mrd_tvalid; logic mrd_tready;
  // demux -> writer (memory writes)
  logic [127:0] mwr_tdata; logic [15:0] mwr_tkeep; logic mwr_tlast; logic [1:0] mwr_tuser; logic mwr_tvalid; logic mwr_tready;
  // completer -> arbiter
  logic [127:0] cc_tdata;  logic [15:0] cc_tkeep;  logic cc_tlast;  logic [1:0] cc_tuser;  logic cc_tvalid;  logic cc_tready;
  // requester -> arbiter
  logic [127:0] rq_tdata;  logic [15:0] rq_tkeep;  logic rq_tlast;  logic [1:0] rq_tuser;  logic rq_tvalid;  logic rq_tready;
  // arbiter -> TX adapter
  logic [127:0] tx_tdata;  logic [15:0] tx_tkeep;  logic tx_tlast;  logic [1:0] tx_tuser;  logic tx_tvalid;  logic tx_tready;

  logic np_backpressure;
  logic [4:0] np_count;
  logic demux_drop_inc, writer_drop_inc, demux_msg_inc;

  pcie7x_rx_adapt u_rx_adapt (
    .clk(user_clk), .rst(user_rst),
    .m_axis_rx_tdata(m_axis_rx_tdata), .m_axis_rx_tkeep(m_axis_rx_tkeep), .m_axis_rx_tlast(m_axis_rx_tlast),
    .m_axis_rx_tvalid(m_axis_rx_tvalid), .m_axis_rx_tready(m_axis_rx_tready), .m_axis_rx_tuser(m_axis_rx_tuser),
    .rx_np_ok(rx_np_ok), .np_backpressure(np_backpressure), .np_ext_count(np_count),
    .m_tlp_tdata(rx_tdata), .m_tlp_tkeep(rx_tkeep), .m_tlp_tlast(rx_tlast), .m_tlp_tuser(rx_tuser),
    .m_tlp_tvalid(rx_tvalid), .m_tlp_tready(rx_tready));

  tlp_rx_demux u_rx_demux (
    .clk(user_clk), .rst(user_rst),
    .s_tlp_tdata(rx_tdata), .s_tlp_tkeep(rx_tkeep), .s_tlp_tlast(rx_tlast), .s_tlp_tuser(rx_tuser),
    .s_tlp_tvalid(rx_tvalid), .s_tlp_tready(rx_tready),
    .m_cpl_tdata(cpl_tdata), .m_cpl_tkeep(cpl_tkeep), .m_cpl_tlast(cpl_tlast), .m_cpl_tuser(cpl_tuser),
    .m_cpl_tvalid(cpl_tvalid), .m_cpl_tready(cpl_tready),
    .m_mrd_tdata(mrd_tdata), .m_mrd_tkeep(mrd_tkeep), .m_mrd_tlast(mrd_tlast), .m_mrd_tuser(mrd_tuser),
    .m_mrd_tvalid(mrd_tvalid), .m_mrd_tready(mrd_tready),
    .m_mwr_tdata(mwr_tdata), .m_mwr_tkeep(mwr_tkeep), .m_mwr_tlast(mwr_tlast), .m_mwr_tuser(mwr_tuser),
    .m_mwr_tvalid(mwr_tvalid), .m_mwr_tready(mwr_tready),
    .drop_cnt_inc(demux_drop_inc), .msg_cnt_inc(demux_msg_inc));

  tlp_tx_arb u_tx_arb (
    .clk(user_clk), .rst(user_rst),
    .s_cpl_tdata(cc_tdata), .s_cpl_tkeep(cc_tkeep), .s_cpl_tlast(cc_tlast), .s_cpl_tuser(cc_tuser),
    .s_cpl_tvalid(cc_tvalid), .s_cpl_tready(cc_tready),
    .s_req_tdata(rq_tdata), .s_req_tkeep(rq_tkeep), .s_req_tlast(rq_tlast), .s_req_tuser(rq_tuser),
    .s_req_tvalid(rq_tvalid), .s_req_tready(rq_tready),
    .m_tlp_tdata(tx_tdata), .m_tlp_tkeep(tx_tkeep), .m_tlp_tlast(tx_tlast), .m_tlp_tuser(tx_tuser),
    .m_tlp_tvalid(tx_tvalid), .m_tlp_tready(tx_tready));

  pcie7x_tx_adapt u_tx_adapt (
    .clk(user_clk), .rst(user_rst),
    .s_tlp_tdata(tx_tdata), .s_tlp_tkeep(tx_tkeep), .s_tlp_tlast(tx_tlast), .s_tlp_tuser(tx_tuser),
    .s_tlp_tvalid(tx_tvalid), .s_tlp_tready(tx_tready),
    .s_axis_tx_tdata(s_axis_tx_tdata), .s_axis_tx_tkeep(s_axis_tx_tkeep), .s_axis_tx_tlast(s_axis_tx_tlast),
    .s_axis_tx_tvalid(s_axis_tx_tvalid), .s_axis_tx_tready(s_axis_tx_tready), .s_axis_tx_tuser(s_axis_tx_tuser),
    .tx_buf_av(tx_buf_av));

  // ===========================================================================
  // Requester + command-port multiplexing
  // ===========================================================================
  // r_*  : rp_requester command port
  logic        r_req_valid, r_req_ready, r_rsp_valid;
  logic [1:0]  r_req_op;
  logic [31:0] r_req_addr, r_req_wdata, r_rsp_rdata;
  logic [3:0]  r_req_be;
  logic [2:0]  r_rsp_status;
  // p0 : shared by pcie_enum and nvme_admin
  logic        p0_req_valid, p0_req_ready, p0_rsp_valid;
  logic [1:0]  p0_req_op;
  logic [31:0] p0_req_addr, p0_req_wdata, p0_rsp_rdata;
  logic [3:0]  p0_req_be;
  logic [2:0]  p0_rsp_status;
  // enum command port
  logic        en_req_valid;
  logic [1:0]  en_req_op;
  logic [31:0] en_req_addr, en_req_wdata;
  logic [3:0]  en_req_be;
  // admin command port
  logic        ad_req_valid;
  logic [1:0]  ad_req_op;
  logic [31:0] ad_req_addr, ad_req_wdata;
  logic [3:0]  ad_req_be;
  // p1 : write engine doorbells
  logic        p1_req_valid, p1_req_ready, p1_rsp_valid;
  logic [1:0]  p1_req_op;
  logic [31:0] p1_req_addr, p1_req_wdata, p1_rsp_rdata;
  logic [3:0]  p1_req_be;
  logic [2:0]  p1_rsp_status;

  // pcie_enum runs first and finishes before nvme_admin starts, so the two never request
  // together; the command port is simply steered by whoever has req_valid (enum first).
  assign p0_req_valid = en_req_valid | ad_req_valid;
  assign p0_req_op    = en_req_valid ? en_req_op    : ad_req_op;
  assign p0_req_addr  = en_req_valid ? en_req_addr  : ad_req_addr;
  assign p0_req_wdata = en_req_valid ? en_req_wdata : ad_req_wdata;
  assign p0_req_be    = en_req_valid ? en_req_be    : ad_req_be;

  logic [15:0] unexpected_cpl_cnt;
  logic        ext_tag_en;                 // from pcie_enum (also programmed into the Root Port by rp_cfg_init)

  rp_requester u_requester (
    .clk(user_clk), .rst(user_rst),
    .req_valid(r_req_valid), .req_ready(r_req_ready), .req_op(r_req_op), .req_addr(r_req_addr),
    .req_wdata(r_req_wdata), .req_be(r_req_be),
    .rsp_valid(r_rsp_valid), .rsp_status(r_rsp_status), .rsp_rdata(r_rsp_rdata),
    .timeout_cycles(32'(REQ_TIMEOUT_CYCLES)),
    .m_req_tdata(rq_tdata), .m_req_tkeep(rq_tkeep), .m_req_tlast(rq_tlast), .m_req_tvalid(rq_tvalid),
    .m_req_tuser(rq_tuser), .m_req_tready(rq_tready),
    .s_cpl_tdata(cpl_tdata), .s_cpl_tkeep(cpl_tkeep), .s_cpl_tlast(cpl_tlast), .s_cpl_tvalid(cpl_tvalid),
    .s_cpl_tuser(cpl_tuser), .s_cpl_tready(cpl_tready),
    .requester_id(RP_ID), .ext_tag_en(ext_tag_en), .unexpected_cpl_cnt(unexpected_cpl_cnt));

  req_mux u_req_mux (
    .clk(user_clk), .rst(user_rst),
    .p0_req_valid(p0_req_valid), .p0_req_ready(p0_req_ready), .p0_req_op(p0_req_op), .p0_req_addr(p0_req_addr),
    .p0_req_wdata(p0_req_wdata), .p0_req_be(p0_req_be),
    .p0_rsp_valid(p0_rsp_valid), .p0_rsp_status(p0_rsp_status), .p0_rsp_rdata(p0_rsp_rdata),
    .p1_req_valid(p1_req_valid), .p1_req_ready(p1_req_ready), .p1_req_op(p1_req_op), .p1_req_addr(p1_req_addr),
    .p1_req_wdata(p1_req_wdata), .p1_req_be(p1_req_be),
    .p1_rsp_valid(p1_rsp_valid), .p1_rsp_status(p1_rsp_status), .p1_rsp_rdata(p1_rsp_rdata),
    .r_req_valid(r_req_valid), .r_req_ready(r_req_ready), .r_req_op(r_req_op), .r_req_addr(r_req_addr),
    .r_req_wdata(r_req_wdata), .r_req_be(r_req_be),
    .r_rsp_valid(r_rsp_valid), .r_rsp_status(r_rsp_status), .r_rsp_rdata(r_rsp_rdata));

  // ===========================================================================
  // Host memory: completer, writer, RAMs, data ring
  // ===========================================================================
  logic [31:0]  cpl_rd_addr;
  logic [1:0]   cpl_rd_sel;
  logic         cpl_rd_en;
  logic [127:0] cpl_rd_data;
  logic         cpl_rd_en_data;
  logic [127:0] data_rd_data;
  logic [2:0]   mps_code;
  logic         seq_running;                // sequencer is in RUN (data_ring accept_enable)

  hostmem_completer #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES)) u_completer (
    .clk(user_clk), .rst(user_rst),
    .s_mrd_tdata(mrd_tdata), .s_mrd_tkeep(mrd_tkeep), .s_mrd_tlast(mrd_tlast), .s_mrd_tuser(mrd_tuser),
    .s_mrd_tvalid(mrd_tvalid), .s_mrd_tready(mrd_tready),
    .np_backpressure(np_backpressure), .np_count(np_count),
    .m_cpl_tdata(cc_tdata), .m_cpl_tkeep(cc_tkeep), .m_cpl_tlast(cc_tlast), .m_cpl_tuser(cc_tuser),
    .m_cpl_tvalid(cc_tvalid), .m_cpl_tready(cc_tready),
    .mps_code(mps_code), .completer_id(RP_ID),
    .rd_addr(cpl_rd_addr), .rd_sel(cpl_rd_sel), .rd_en(cpl_rd_en), .rd_data(cpl_rd_data));

  logic         hw_wr_en_ident, hw_wr_en_acq, hw_wr_en_iocq, hw_wr_en_data;
  logic [31:0]  hw_wr_beat_addr;
  logic [127:0] hw_wr_data;
  logic [15:0]  hw_wr_be;

  hostmem_writer #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES)) u_writer (
    .clk(user_clk), .rst(user_rst),
    .s_mwr_tdata(mwr_tdata), .s_mwr_tkeep(mwr_tkeep), .s_mwr_tlast(mwr_tlast), .s_mwr_tuser(mwr_tuser),
    .s_mwr_tvalid(mwr_tvalid), .s_mwr_tready(mwr_tready),
    .wr_en_ident(hw_wr_en_ident), .wr_en_acq(hw_wr_en_acq), .wr_en_iocq(hw_wr_en_iocq),
    .wr_en_data(hw_wr_en_data),
    .wr_beat_addr(hw_wr_beat_addr), .wr_data(hw_wr_data), .wr_be(hw_wr_be),
    .drop_cnt_inc(writer_drop_inc));

  // RAM ports
  logic         asq_wr_en;   logic [6:0] asq_wr_addr;   logic [127:0] asq_wr_data;
  logic         iosq_wr_en;  logic [7:0] iosq_wr_addr;  logic [127:0] iosq_wr_data;
  logic         acq_rd_en;   logic [4:0] acq_rd_addr;   logic [127:0] acq_rd_data;
  logic         iocq_rd_en;  logic [5:0] iocq_rd_addr;  logic [127:0] iocq_rd_data;
  logic         ident_rd_en; logic [7:0] ident_rd_addr; logic [127:0] ident_rd_data;
  logic         clear_cqs, clear_done;

  hostmem_rams #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES)) u_rams (
    .clk(user_clk), .rst(user_rst),
    .asq_wr_en(asq_wr_en), .asq_wr_addr(asq_wr_addr), .asq_wr_data(asq_wr_data),
    .iosq_wr_en(iosq_wr_en), .iosq_wr_addr(iosq_wr_addr), .iosq_wr_data(iosq_wr_data),
    .hw_wr_en_ident(hw_wr_en_ident), .hw_wr_en_acq(hw_wr_en_acq), .hw_wr_en_iocq(hw_wr_en_iocq),
    .hw_wr_beat_addr(hw_wr_beat_addr), .hw_wr_data(hw_wr_data), .hw_wr_be(hw_wr_be),
    .clear_cqs(clear_cqs), .clear_done(clear_done),
    .cpl_rd_en(cpl_rd_en), .cpl_rd_sel(cpl_rd_sel), .cpl_rd_addr(cpl_rd_addr), .cpl_rd_data(cpl_rd_data),
    .cpl_rd_en_data(cpl_rd_en_data), .data_rd_data(data_rd_data),
    .acq_rd_en(acq_rd_en), .acq_rd_addr(acq_rd_addr), .acq_rd_data(acq_rd_data),
    .iocq_rd_en(iocq_rd_en), .iocq_rd_addr(iocq_rd_addr), .iocq_rd_data(iocq_rd_data),
    .ident_rd_en(ident_rd_en), .ident_rd_addr(ident_rd_addr), .ident_rd_data(ident_rd_data));

  logic [N_SLOTS-1:0]       slot_full, slot_free, eng_slot_free;
  logic [N_SLOTS*64-1:0]    slot_lba_flat;
  // in verify mode the ring never filled the slot: data_ring must not see a free pulse
  assign slot_free = eng_slot_free & {N_SLOTS{~verify_mode}};
  logic [$clog2(N_SLOTS):0] slots_used;
  logic                     ring_input_stalled;

  data_ring #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES)) u_data_ring (
    .data_clk(data_clk), .data_rst(data_rst),
    .s_data_tdata(s_data_tdata), .s_data_tvalid(s_data_tvalid), .s_data_tready(s_data_tready),
    .clk(user_clk), .rst(user_rst), .accept_enable(seq_running),
    .slot_full(slot_full), .slot_free(slot_free), .slots_used(slots_used),
    .input_stalled(ring_input_stalled), .beats_in(stat_beats_in), .stall_cycles(stat_stall_cycles),
    .rd_en(cpl_rd_en_data), .rd_addr(cpl_rd_addr), .rd_data(data_rd_data));

  // ===========================================================================
  // Control: sequencer, enumeration, Root Port config, admin, write engine
  // ===========================================================================
  logic        seq_enum_start, seq_rpcfg_start, seq_init_start, seq_we_enable, seq_flush_req, seq_shutdown_start;
  logic        enum_done, enum_fail, rpcfg_done, init_done, init_fail, flush_done, shutdown_done;
  logic [7:0]  enum_fail_step, init_fail_step;
  logic        all_slots_free, slots_idle;
  logic        seq_done;
  logic [7:0]  seq_error_code, seq_error_detail;
  logic        io_timeout, flush_skipped, io_error, doorbell_err;

  // sticky link flags: link_clk -> user_clk (no reset: user_reset must not clear them)
  (* ASYNC_REG = "TRUE" *) logic [1:0] link_lost_sync = 2'b00, link_fail_sync = 2'b00;
  always_ff @(posedge user_clk) begin
    link_lost_sync <= {link_lost_sync[0], link_lost};
    link_fail_sync <= {link_fail_sync[0], link_fail};
  end
  logic link_lost_uc, link_fail_uc;
  assign link_lost_uc = link_lost_sync[1];
  assign link_fail_uc = link_fail_sync[1];

  host_sequencer u_sequencer (
    .clk(user_clk), .rst(user_rst),
    .start(ctrl_start), .stop(ctrl_stop),
    .link_ready(link_ready), .link_lost(link_lost_uc), .link_fail(link_fail_uc),
    .io_timeout(io_timeout), .lba_range_err(status_lba_range_err),
    .enum_done(enum_done), .enum_fail(enum_fail), .enum_fail_step(enum_fail_step),
    .rpcfg_done(rpcfg_done),
    .init_done(init_done), .init_fail(init_fail), .init_fail_step(init_fail_step),
    .flush_done(flush_done), .shutdown_done(shutdown_done), .all_slots_free(all_slots_free),
    .enum_start(seq_enum_start), .rpcfg_start(seq_rpcfg_start), .init_start(seq_init_start),
    .we_enable(seq_we_enable), .flush_req(seq_flush_req), .shutdown_start(seq_shutdown_start),
    .state(status_seq_state), .error_code(seq_error_code), .error_detail(seq_error_detail),
    .running(seq_running), .done(seq_done));

  assign all_slots_free = slots_idle && (slot_full == '0);

  logic        class_ok;
  logic [7:0]  pcie_cap_off;
  logic [31:0] vid_did, bar0_size_mask, devcap;

  // CRS retry delay = 1 ms (1 us with SIM_FAST) in user_clk cycles
  pcie_enum #(
    .CRS_RETRY_DELAY_CYCLES(SIM_FAST ? (CLK_HZ / 1_000_000) : (CLK_HZ / 1_000))
  ) u_enum (
    .clk(user_clk), .rst(user_rst),
    .start(seq_enum_start), .done(enum_done), .fail(enum_fail), .fail_step(enum_fail_step),
    .mps_code(mps_code), .ext_tag_en(ext_tag_en), .vid_did(vid_did), .bar0_size_mask(bar0_size_mask),
    .class_ok(class_ok), .pcie_cap_off(pcie_cap_off), .devcap(devcap),
    .req_valid(en_req_valid), .req_ready(p0_req_ready), .req_op(en_req_op), .req_addr(en_req_addr),
    .req_wdata(en_req_wdata), .req_be(en_req_be),
    .rsp_valid(p0_rsp_valid), .rsp_status(p0_rsp_status), .rsp_rdata(p0_rsp_rdata));

  rp_cfg_init u_rp_cfg_init (
    .clk(user_clk), .rst(user_rst),
    .start(seq_rpcfg_start), .done(rpcfg_done), .mps_code(mps_code), .ext_tag_en(ext_tag_en),
    .cfg_mgmt_di(cfg_mgmt_di), .cfg_mgmt_byte_en(cfg_mgmt_byte_en), .cfg_mgmt_dwaddr(cfg_mgmt_dwaddr),
    .cfg_mgmt_wr_en(cfg_mgmt_wr_en), .cfg_mgmt_rd_en(cfg_mgmt_rd_en), .cfg_mgmt_do(cfg_mgmt_do),
    .cfg_mgmt_rd_wr_done(cfg_mgmt_rd_wr_done),
    .cfg_mgmt_wr_readonly(cfg_mgmt_wr_readonly), .cfg_mgmt_wr_rw1c_as_rw(cfg_mgmt_wr_rw1c_as_rw));

  logic [3:0]  lbads, dstrd;
  logic [63:0] nsze;
  logic [7:0]  mdts;
  logic        vwc;
  logic [15:0] admin_cqe_status;
  logic        shutdown_timeout;
  logic [7:0]  dbg_asq_tail, dbg_acq_head;

  nvme_admin #(.SLOT_BYTES(SLOT_BYTES)) u_admin (
    .clk(user_clk), .rst(user_rst), .tick_ms(tick_ms),
    .start_init(seq_init_start), .format_req(format_req), .format_done(format_done), .format_ms(format_ms),
    .init_done(init_done), .init_fail(init_fail), .fail_step(init_fail_step),
    .cqe_status_last(admin_cqe_status),
    .start_shutdown(seq_shutdown_start), .shutdown_done(shutdown_done), .shutdown_timeout(shutdown_timeout),
    .lbads(lbads), .nsze(nsze), .mdts(mdts), .dstrd(dstrd), .vwc(vwc),
    .clear_cqs(clear_cqs), .clear_done(clear_done),
    .dbg_asq_tail(dbg_asq_tail), .dbg_acq_head(dbg_acq_head),
    .req_valid(ad_req_valid), .req_ready(p0_req_ready), .req_op(ad_req_op), .req_addr(ad_req_addr),
    .req_wdata(ad_req_wdata), .req_be(ad_req_be),
    .rsp_valid(p0_rsp_valid), .rsp_status(p0_rsp_status), .rsp_rdata(p0_rsp_rdata),
    .asq_wr_en(asq_wr_en), .asq_wr_addr(asq_wr_addr), .asq_wr_data(asq_wr_data),
    .acq_rd_en(acq_rd_en), .acq_rd_addr(acq_rd_addr), .acq_rd_data(acq_rd_data),
    .ident_rd_en(ident_rd_en), .ident_rd_addr(ident_rd_addr), .ident_rd_data(ident_rd_data));

  logic [6:0] sq_tail, cq_head;
  logic       sq_full_stall;

  nvme_write_engine #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES), .IO_TIMEOUT_MS(IO_TIMEOUT_MS)) u_write_engine (
    .clk(user_clk), .rst(user_rst), .tick_ms(tick_ms), .restart(seq_init_start),
    // verify run: no new Read after STOP (DRAIN only waits for the outstanding ones); a recording
    // keeps submitting in DRAIN until the ring is empty
    .enable(seq_we_enable && (!verify_mode || seq_running)), .start_lba(START_LBA), .lbads(lbads), .nsze(nsze), .dstrd(dstrd),
    .slot_full(slot_full), .slot_free(eng_slot_free), .slot_lba_flat(slot_lba_flat),
    .read_mode(verify_mode), .max_cmds(verify_cmds), .cmd_limit_done(cmd_limit_done),
    .iosq_wr_en(iosq_wr_en), .iosq_wr_addr(iosq_wr_addr), .iosq_wr_data(iosq_wr_data),
    .iocq_rd_en(iocq_rd_en), .iocq_rd_addr(iocq_rd_addr), .iocq_rd_data(iocq_rd_data),
    .req_valid(p1_req_valid), .req_ready(p1_req_ready), .req_op(p1_req_op), .req_addr(p1_req_addr),
    .req_wdata(p1_req_wdata), .req_be(p1_req_be),
    .rsp_valid(p1_rsp_valid), .rsp_status(p1_rsp_status),
    .flush_req(seq_flush_req), .flush_done(flush_done),
    .next_lba(stat_next_lba), .bytes_done(stat_bytes_done), .cmds_done(stat_cmds_done),
    .cmd_errors(stat_cmd_errors), .last_status(stat_last_status),
    .max_latency_cycles(stat_max_latency), .cpl_evt(stat_cpl_evt), .cpl_lat(stat_cpl_lat),
    .sq_full_stall(sq_full_stall),
    .sq_full_stall_cycles(stat_sq_full_stall_cycles),
    .sq_tail(sq_tail), .cq_head(cq_head), .slots_idle(slots_idle),
    .lba_range_err(status_lba_range_err), .doorbell_err(doorbell_err),
    .io_timeout(io_timeout), .flush_skipped(flush_skipped), .io_error(io_error),
    .first_error_lba(stat_first_error_lba));

  verify_checker #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES), .START_LBA(START_LBA)) u_verify (
    .clk(user_clk), .rst(user_rst), .clear(seq_init_start), .lbads(lbads), .slot_lba_flat(slot_lba_flat),
    .wr_en(hw_wr_en_data), .wr_beat_addr(hw_wr_beat_addr), .wr_data(hw_wr_data), .wr_be(hw_wr_be),
    .chk_bytes(vfy_bytes), .err_beats(vfy_err_beats), .first_err_lba(vfy_first_err_lba),
    .first_err_exp(vfy_first_err_exp), .first_err_got(vfy_first_err_got), .err_seen());

  // ===========================================================================
  // Status
  // ===========================================================================
  logic enum_ok_q;
  always_ff @(posedge user_clk) begin
    if (user_rst)         enum_ok_q <= 1'b0;
    else if (enum_done)   enum_ok_q <= 1'b1;
    else if (seq_enum_start) enum_ok_q <= 1'b0;
  end

  logic rx_drop_q;
  always_ff @(posedge user_clk) begin
    if (user_rst)                            rx_drop_q <= 1'b0;
    else if (demux_drop_inc || writer_drop_inc) rx_drop_q <= 1'b1;
  end

  always_ff @(posedge user_clk) begin
    if (user_rst)                                   stat_rx_msgs <= 16'd0;
    else if (demux_msg_inc && stat_rx_msgs != 16'hFFFF) stat_rx_msgs <= stat_rx_msgs + 16'd1;
  end

  assign status_flags = {rx_drop_q, io_error, link_lost_uc, link_fail_uc,
                         flush_skipped, io_timeout, doorbell_err, shutdown_timeout};

  assign status_link_up          = link_ready;
  assign status_gen2             = link_gen2;
  assign status_link_width       = link_width;
  assign status_enum_done        = enum_ok_q;
  assign status_nvme_ready       = init_done;
  assign status_running          = seq_running;
  assign status_done             = seq_done;
  assign status_error            = seq_error_code;
  assign status_error_detail     = seq_error_detail;
  assign status_shutdown_timeout = shutdown_timeout;
  assign stat_unexpected_cpl     = unexpected_cpl_cnt;
  assign stat_lbads              = lbads;
  assign stat_nsze               = nsze;
  assign stat_mdts               = mdts;

endmodule
