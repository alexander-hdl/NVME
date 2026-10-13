// =============================================================================
// tb_top.sv — full-flow test of nvme_host_core against the behavioural models
//
//   data source (data_clk 100 MHz) --> nvme_host_core (DUT) <--AXI-S/cfg_mgmt--> pcie7x_core_model
//                                                                                  <--TLP--> nvme_ssd_model
//
// DUT parameters (fast simulation): N_SLOTS=4, SLOT_BYTES=16384 (=> NLB per command = 32
// LBAs of 512 B), SIM_FAST=1, REQ_TIMEOUT_CYCLES=20000.
//
// Stimulus
//   * sys_rst_n of the core model released after 1 us; the core model raises user_lnk_up
//     and later pl_sel_lnk_rate (Gen2).  Instead of link_bringup, this tb drives
//     link_ready=1, link_gen2=1, link_width=2 (synchronous to user_clk) 100 cycles after
//     user_lnk_up && pl_sel_lnk_rate.
//   * ctrl_start = 1 from the beginning.
//   * The data source streams TOTAL_BEATS = 65536 beats (1 MiB) at full rate; beat n is
//     {~n, n ^ 32'hA5A5A5A5, n*7, n} (32-bit beat counter n).  Every accepted byte is also
//     stored in ref_mem (tdata[7:0] = byte at the lowest address, ARCHITECTURE.md §3).
//   * When the source has finished, ctrl_stop = 1 (stop the stream first, then stop).
//   * Wait for status_done (status_error != 0 or a timeout -> $fatal).
//
// Checks (each failure is an $error; the test ends with "TEST PASSED" or $fatal)
//   T-1  data: u_ssd.check_range(0, 2048 LBAs, ref_mem) == 0 mismatching bytes
//   T-2  stat_bytes_done == 1 MiB and stat_cmds_done == 64 (successful Writes only: minus one
//        command with +inject_err), 64 Write commands executed by the SSD
//   T-3  stat_cmd_errors == 0 (== 1 and stat_last_status == injected status with +inject_err),
//        status_error == 0
//   T-4  the sequencer (status_seq_state) visited IDLE, LINK, ENUM, RPCFG, ADMIN_INIT, RUN,
//        DRAIN, FLUSH, SHUTDOWN, DONE in this order
//   T-5  enumeration as seen by the SSD: BAR0 sizing write (all ones), BAR0 = 0xC000_0000,
//        BAR1 = 0, DevCtl MPS=1 (min(MPSS 2, MPS_CAP 1)), MRRS=2, ETE=1, Command = 0x0406
//   T-6  Root Port via cfg_mgmt: Command = 0x0006 (MSE+BME), DevCtl MPS = 1
//   T-7  NVMe init order: AQA/ASQ/ACQ (values checked) written before the CC write with EN=1;
//        admin commands exactly: Identify(CNS 1), Identify(CNS 0), Set Features(FID 7),
//        Create CQ, Create SQ, ..., Delete SQ, Delete CQ
//   T-8  shutdown: Flush after the last Write, then Delete SQ, Delete CQ, then CC.SHN=01b,
//        then a CSTS read returning SHST=10b
//   T-9  no $error in the models (u_core.err_count == 0, u_ssd.err_count == 0)
//   Metric: TX-bus valid-beat ratio in the steady-state window (first .. last data CplD
//   accepted by the core) is printed (ARCHITECTURE.md §7 target >= 0.9 with tready always
//   high; note the 100 MHz x 16 B = 1.6 GB/s source limits the ratio to roughly 0.85).
//
// Plusargs  (xsim.bat on Windows mangles '=': both "+txstall=N" and "+txstallN" work)
//   +straddle      RX framing randomisation: straddle 50 %, SOF@DW2 25 %, RX gaps 5 %
//   +txstall=N     s_axis_tx_tready low with probability N % per cycle
//   +cqesplit      CQEs split (8+8 B, P last) 50 % and coalesced (2 per MWr) 50 %
//   +inject_err    the 11th Write gets an error status -> expect stat_cmd_errors == 1 and the
//                  run continues normally
//   +nobme         negative self-check of the models: the Root Port behaves as if BME=0 ->
//                  the SSD's first DMA gets UR; PASSES only if the models flag an $error
//   +stress        straddle + txstall=10 + cqesplit + random MRd gaps + rx_np_ok lag 2
//   +verbose       models print every TLP
// =============================================================================
`timescale 1ns/1ps

module tb_top;
  import pcie_sim_pkg::*;

  localparam int unsigned N_SLOTS     = 4;
  localparam int unsigned SLOT_BYTES  = 16384;
  localparam int unsigned TOTAL_BEATS = 65536;               // 1 MiB
  localparam int unsigned TOTAL_BYTES = TOTAL_BEATS * 16;
  localparam int unsigned N_CMDS      = TOTAL_BYTES / SLOT_BYTES;
  localparam int unsigned N_LBAS      = TOTAL_BYTES / 512;

  // ---------------------------------------------------------------------------
  // Signals
  // ---------------------------------------------------------------------------
  logic         sys_rst_n;
  wire          user_clk, user_reset, user_lnk_up;
  wire  [127:0] m_axis_rx_tdata;  wire [15:0] m_axis_rx_tkeep; wire m_axis_rx_tlast, m_axis_rx_tvalid;
  wire          m_axis_rx_tready; wire [21:0] m_axis_rx_tuser; wire rx_np_ok;
  wire  [127:0] s_axis_tx_tdata;  wire [15:0] s_axis_tx_tkeep; wire s_axis_tx_tlast, s_axis_tx_tvalid;
  wire          s_axis_tx_tready; wire [3:0] s_axis_tx_tuser;
  wire  [5:0]   tx_buf_av; wire tx_cfg_req, tx_cfg_gnt;
  wire  [31:0]  cfg_mgmt_di, cfg_mgmt_do; wire [3:0] cfg_mgmt_byte_en; wire [9:0] cfg_mgmt_dwaddr;
  wire          cfg_mgmt_wr_en, cfg_mgmt_rd_en, cfg_mgmt_rd_wr_done, cfg_mgmt_wr_readonly, cfg_mgmt_wr_rw1c_as_rw;
  wire  [5:0]   pl_ltssm_state; wire pl_sel_lnk_rate; wire [1:0] pl_sel_lnk_width;
  wire          pl_link_gen2_cap, pl_link_partner_gen2_supported;

  logic         link_ready, link_gen2; logic [1:0] link_width;
  logic         data_clk, data_rst;
  logic [127:0] s_data_tdata; logic s_data_tvalid; wire s_data_tready;
  logic         ctrl_start, ctrl_stop;

  wire          status_link_up, status_gen2, status_enum_done, status_nvme_ready, status_running, status_done;
  wire  [1:0]   status_link_width;
  wire  [7:0]   status_error, status_error_detail;
  wire  [3:0]   status_seq_state;
  wire          status_lba_range_err, status_shutdown_timeout;
  wire  [63:0]  stat_bytes_done, stat_cmds_done, stat_next_lba, stat_beats_in, stat_nsze;
  wire  [31:0]  stat_cmd_errors, stat_max_latency, stat_stall_cycles, stat_sq_full_stall_cycles;
  wire  [15:0]  stat_last_status, stat_unexpected_cpl;
  wire  [3:0]   stat_lbads;
  wire  [7:0]   stat_mdts;
  wire  [7:0]   status_flags;
  wire  [63:0]  stat_first_error_lba;

  // ---------------------------------------------------------------------------
  // Models
  // ---------------------------------------------------------------------------
  pcie7x_core_model u_core (
    .sys_rst_n(sys_rst_n), .user_clk_out(user_clk), .user_reset_out(user_reset), .user_lnk_up(user_lnk_up),
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

  nvme_ssd_model u_ssd (.clk(user_clk), .perst_n(sys_rst_n));

  // ---------------------------------------------------------------------------
  // DUT
  // ---------------------------------------------------------------------------
  nvme_host_core #(
    .N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES), .SIM_FAST(1'b1), .REQ_TIMEOUT_CYCLES(20000)
  ) dut (
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
    .link_ready(link_ready), .link_gen2(link_gen2), .link_width(link_width),
    .data_clk(data_clk), .data_rst(data_rst),
    .s_data_tdata(s_data_tdata), .s_data_tvalid(s_data_tvalid), .s_data_tready(s_data_tready),
    .ctrl_start(ctrl_start), .ctrl_stop(ctrl_stop),
    .status_link_up(status_link_up), .status_gen2(status_gen2), .status_link_width(status_link_width),
    .status_enum_done(status_enum_done), .status_nvme_ready(status_nvme_ready), .status_running(status_running),
    .status_done(status_done), .status_error(status_error), .status_error_detail(status_error_detail),
    .status_seq_state(status_seq_state), .status_lba_range_err(status_lba_range_err),
    .status_shutdown_timeout(status_shutdown_timeout),
    .stat_bytes_done(stat_bytes_done), .stat_cmds_done(stat_cmds_done), .stat_cmd_errors(stat_cmd_errors),
    .stat_max_latency(stat_max_latency), .stat_last_status(stat_last_status), .stat_next_lba(stat_next_lba),
    .stat_beats_in(stat_beats_in), .stat_stall_cycles(stat_stall_cycles),
    .stat_sq_full_stall_cycles(stat_sq_full_stall_cycles), .stat_unexpected_cpl(stat_unexpected_cpl),
    .stat_lbads(stat_lbads), .stat_nsze(stat_nsze), .stat_mdts(stat_mdts),
    .status_flags(status_flags), .stat_first_error_lba(stat_first_error_lba));

  // ---------------------------------------------------------------------------
  // Error bookkeeping
  // ---------------------------------------------------------------------------
  int tb_errs = 0;
  function automatic void tb_err(input string s);
    tb_errs++;
    $error("[tb_top @%0t] %s", $time, s);
  endfunction

  // ---------------------------------------------------------------------------
  // Data source (data_clk = 100 MHz) and reference memory
  // ---------------------------------------------------------------------------
  byte unsigned ref_mem[];
  int unsigned  src_cnt = 0;
  bit           src_done = 0;

  initial begin data_clk = 1'b0; forever #5 data_clk = ~data_clk; end

  function automatic logic [127:0] pattern(input logic [31:0] n);
    return {~n, n ^ 32'hA5A5A5A5, n * 32'd7, n};
  endfunction

  always @(posedge data_clk) begin
    if (data_rst) begin
      s_data_tvalid <= 1'b0;
      s_data_tdata  <= '0;
    end else begin
      if (s_data_tvalid && s_data_tready) begin
        for (int j = 0; j < 16; j++) ref_mem[16*src_cnt + j] = s_data_tdata[8*j +: 8];
        src_cnt = src_cnt + 1;
        if (src_cnt == TOTAL_BEATS) src_done = 1;
      end
      if (src_cnt < TOTAL_BEATS) begin
        s_data_tvalid <= 1'b1;
        s_data_tdata  <= pattern(src_cnt);
      end else begin
        s_data_tvalid <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Sequencer state trace and TX-bus activity (per-cycle cumulative counters)
  // ---------------------------------------------------------------------------
  logic [3:0] seq_trace[$];
  longint     tx_valid_cum[$];     // index = u_core.cyc at the sampling edge
  longint     tx_beat_cum[$];
  longint     vcnt = 0, bcnt = 0, cyc0 = -1;

  always @(posedge user_clk) begin
    if (!user_reset && (seq_trace.size() == 0 || seq_trace[$] !== status_seq_state)) seq_trace.push_back(status_seq_state);
    if (cyc0 < 0) cyc0 = u_core.cyc;
    if (s_axis_tx_tvalid === 1'b1) vcnt++;
    if (s_axis_tx_tvalid === 1'b1 && s_axis_tx_tready === 1'b1) bcnt++;
    tx_valid_cum.push_back(vcnt);
    tx_beat_cum.push_back(bcnt);
  end

  // ---------------------------------------------------------------------------
  // Plusargs
  // ---------------------------------------------------------------------------
  bit nobme = 0, inject = 0;

  function automatic void apply_plusargs();
    int n;
    if ($test$plusargs("verbose")) begin u_core.verbose = 1; u_ssd.verbose = 1; end
    if ($test$plusargs("straddle") || $test$plusargs("stress")) begin
      u_core.straddle_pct = 50; u_core.sof_dw2_pct = 25; u_core.rx_gap_pct = 5;
      $display("[tb_top] knob: RX straddle 50%%, SOF@DW2 25%%, RX gaps 5%%");
    end
    if ($value$plusargs("txstall=%d", n) || $value$plusargs("txstall%d", n)) begin
      u_core.tx_stall_pct = n;
      $display("[tb_top] knob: s_axis_tx_tready stall %0d%%", n);
    end else if ($test$plusargs("stress")) u_core.tx_stall_pct = 10;
    if ($test$plusargs("cqesplit") || $test$plusargs("stress")) begin
      u_ssd.cqe_split_pct = 50; u_ssd.cqe_coalesce_pct = 50;
      $display("[tb_top] knob: CQE split 50%%, coalesce 50%%");
    end
    if ($test$plusargs("stress")) begin
      u_ssd.mrd_gap_max = 2;
      $display("[tb_top] knob: stress (txstall 10%%, MRd gaps)");
    end
    if ($test$plusargs("inject_err")) begin
      inject = 1;
      u_ssd.inject_status_at_cmd = 10;
      $display("[tb_top] knob: error status injected into I/O Write #10");
    end
    if ($test$plusargs("nobme")) begin
      nobme = 1;
      u_core.force_bme_off = 1;
      $display("[tb_top] NEGATIVE self-check: Root Port BME forced to 0 -> the models MUST report an $error");
    end
  endfunction

  // ---------------------------------------------------------------------------
  // Event-log helpers
  // ---------------------------------------------------------------------------
  function automatic int find_mmio_wr(input logic [15:0] off, input logic [31:0] val, input logic [31:0] mask, input int from = 0);
    for (int i = from; i < u_ssd.ev_log.size(); i++)
      if (u_ssd.ev_log[i].kind == EV_MMIO_WR && u_ssd.ev_log[i].a == off && ((u_ssd.ev_log[i].b[31:0] ^ val) & mask) == 0)
        return i;
    return -1;
  endfunction

  function automatic int find_cfg_wr(input logic [11:0] off, input logic [31:0] val, input logic [31:0] mask);
    for (int i = 0; i < u_ssd.ev_log.size(); i++)
      if (u_ssd.ev_log[i].kind == EV_CFGWR && u_ssd.ev_log[i].a == off && ((u_ssd.ev_log[i].b[31:0] ^ val) & mask) == 0)
        return i;
    return -1;
  endfunction

  // ---------------------------------------------------------------------------
  // Checks after the run
  // ---------------------------------------------------------------------------
  task automatic final_checks();
    int          e, i_aqa, i_asq, i_acq, i_en, i_flush, i_lastwr, i_delsq, i_delcq, i_shn, i_shst;
    int          adm[$], adm_idx[$];
    logic [31:0] adm_cdw10[$];
    logic [3:0]  exp_states[$];
    longint      c0, c1;
    logic [15:0] rp_dc;

    // ---- T-1 data ----
    e = u_ssd.check_range(0, N_LBAS, ref_mem);
    if (e != 0) tb_err($sformatf("T-1: %0d data bytes differ in LBAs 0..%0d", e, N_LBAS - 1));
    else $display("[tb_top] T-1 data OK: %0d LBAs (%0d bytes) match the reference", N_LBAS, TOTAL_BYTES);
    if (u_ssd.lba_mem.num() != N_LBAS) tb_err($sformatf("T-1: SSD holds %0d LBAs, expected %0d", u_ssd.lba_mem.num(), N_LBAS));

    // ---- T-2 / T-3 statistics ----
    // stat_bytes_done / stat_cmds_done count successful Writes only (nvme_write_engine.sv)
    if (stat_bytes_done != TOTAL_BYTES - (inject ? SLOT_BYTES : 0))
      tb_err($sformatf("T-2: stat_bytes_done = %0d, expected %0d", stat_bytes_done, TOTAL_BYTES - (inject ? SLOT_BYTES : 0)));
    if (stat_cmds_done != N_CMDS - (inject ? 1 : 0))
      tb_err($sformatf("T-2: stat_cmds_done = %0d, expected %0d", stat_cmds_done, N_CMDS - (inject ? 1 : 0)));
    if (inject && stat_last_status[14:0] != u_ssd.inject_status_value)
      tb_err($sformatf("T-3: stat_last_status = %04h, expected the injected %04h", stat_last_status, u_ssd.inject_status_value));
    if (u_ssd.n_io_writes != N_CMDS)    tb_err($sformatf("T-2: SSD executed %0d Writes, expected %0d", u_ssd.n_io_writes, N_CMDS));
    if (stat_cmd_errors != (inject ? 1 : 0))
      tb_err($sformatf("T-3: stat_cmd_errors = %0d, expected %0d", stat_cmd_errors, inject ? 1 : 0));
    if (status_error != 0) tb_err($sformatf("T-3: status_error = %02h (detail %02h)", status_error, status_error_detail));
    // status_flags: [6] io_error only with +inject_err, everything else 0 (no timeout, no drop, no link flag)
    if (status_flags !== (inject ? 8'h40 : 8'h00))
      tb_err($sformatf("T-3: status_flags = %02h, expected %02h", status_flags, inject ? 8'h40 : 8'h00));
    if (inject && stat_first_error_lba !== 64'(10 * (SLOT_BYTES / 512)))
      tb_err($sformatf("T-3: stat_first_error_lba = %0d, expected %0d (SLBA of the 11th Write)", stat_first_error_lba, 10 * (SLOT_BYTES / 512)));

    // ---- T-4 sequencer states ----
    exp_states = {4'd0, 4'd1, 4'd2, 4'd3, 4'd4, 4'd5, 4'd6, 4'd7, 4'd8, 4'd9};
    if (seq_trace != exp_states) tb_err($sformatf("T-4: sequencer state trace %p, expected %p", seq_trace, exp_states));

    // ---- T-5 enumeration seen by the SSD ----
    if (find_cfg_wr(12'h010, 32'hFFFF_C004, 32'hFFFF_FFFF) < 0) tb_err("T-5: no BAR0 sizing write (all ones) seen");
    if (find_cfg_wr(12'h010, 32'hC000_0004, 32'hFFFF_FFFF) < 0) tb_err("T-5: BAR0 never set to 0xC000_0000");
    if (find_cfg_wr(12'h014, 32'h0000_0000, 32'hFFFF_FFFF) < 0) tb_err("T-5: BAR1 never written with 0");
    if (u_ssd.cfg[4] != 32'hC000_0004 || u_ssd.cfg[5] != 0) tb_err($sformatf("T-5: final BAR0/1 = %08h/%08h", u_ssd.cfg[4], u_ssd.cfg[5]));
    if (u_ssd.cfg[1][15:0] != 16'h0406) tb_err($sformatf("T-5: SSD Command = %04h, expected 0406", u_ssd.cfg[1][15:0]));
    if (u_ssd.cfg[34][7:5] != 3'd1 || u_ssd.cfg[34][14:12] != 3'd2 || u_ssd.cfg[34][8] != 1'b1)
      tb_err($sformatf("T-5: SSD DevCtl = %04h (want MPS=1, MRRS=2, ETE=1)", u_ssd.cfg[34][15:0]));
    // ---- T-6 Root Port ----
    if (u_core.rp_command() != 16'h0006) tb_err($sformatf("T-6: RP Command = %04h, expected 0006", u_core.rp_command()));
    rp_dc = u_core.rp_devctl();
    if (rp_dc[7:5] != 3'd1) tb_err($sformatf("T-6: RP DevCtl = %04h, MPS != 1", rp_dc));

    // ---- T-7 NVMe init order ----
    i_aqa = find_mmio_wr(16'h24, 32'h001F_001F, '1);
    i_asq = find_mmio_wr(16'h28, 32'h1000_0000, '1);
    i_acq = find_mmio_wr(16'h30, 32'h1000_1000, '1);
    i_en  = find_mmio_wr(16'h14, 32'h0046_0001, '1);
    if (i_aqa < 0 || i_asq < 0 || i_acq < 0 || i_en < 0)
      tb_err($sformatf("T-7: missing init write (AQA %0d, ASQ %0d, ACQ %0d, CC.EN %0d)", i_aqa, i_asq, i_acq, i_en));
    else if (!(i_aqa < i_en && i_asq < i_en && i_acq < i_en))
      tb_err("T-7: AQA/ASQ/ACQ not all written before CC.EN=1");
    if (find_mmio_wr(16'h2C, 0, '1) < 0 || find_mmio_wr(16'h34, 0, '1) < 0) tb_err("T-7: ASQ/ACQ high halves not written");
    foreach (u_ssd.ev_log[i])
      if (u_ssd.ev_log[i].kind == EV_ADMIN) begin
        adm.push_back(u_ssd.ev_log[i].a); adm_idx.push_back(i); adm_cdw10.push_back(u_ssd.ev_log[i].b[63:32]);
      end
    if (adm.size() != 7 || adm[0] != 8'h06 || adm_cdw10[0][7:0] != 1 || adm[1] != 8'h06 || adm_cdw10[1][7:0] != 0 ||
        adm[2] != 8'h09 || adm_cdw10[2][7:0] != 7 || adm[3] != 8'h05 || adm[4] != 8'h01 || adm[5] != 8'h00 || adm[6] != 8'h04)
      tb_err($sformatf("T-7/T-8: admin command sequence %p, expected [06(CNS1) 06(CNS0) 09(FID7) 05 01 00 04]", adm));

    // ---- T-8 shutdown ----
    i_flush = -1; i_lastwr = -1;
    foreach (u_ssd.ev_log[i])
      if (u_ssd.ev_log[i].kind == EV_IO) begin
        if (u_ssd.ev_log[i].a == 8'h01) i_lastwr = i;
        if (u_ssd.ev_log[i].a == 8'h00) begin
          if (i_flush >= 0) tb_err("T-8: more than one Flush");
          i_flush = i;
        end
      end
    if (i_flush < 0) tb_err("T-8: no Flush seen");
    else if (i_flush < i_lastwr) tb_err("T-8: Flush executed before the last Write");
    if (adm.size() == 7) begin
      i_delsq = adm_idx[5]; i_delcq = adm_idx[6];
      if (i_flush > i_delsq) tb_err("T-8: Delete SQ before Flush");
      i_shn = find_mmio_wr(16'h14, 32'h0000_4000, 32'h0000_C000, i_delcq);
      if (i_shn < 0) tb_err("T-8: no CC.SHN=01b write after Delete CQ");
      else begin
        i_shst = -1;
        for (int i = i_shn; i < u_ssd.ev_log.size(); i++)
          if (u_ssd.ev_log[i].kind == EV_MMIO_RD && u_ssd.ev_log[i].a == 16'h1C && u_ssd.ev_log[i].b[3:2] == 2'b10) begin
            i_shst = i; break;
          end
        if (i_shst < 0) tb_err("T-8: CSTS.SHST=10b never polled after CC.SHN");
      end
    end

    // ---- T-9 models ----
    if (u_core.err_count != 0) tb_err($sformatf("T-9: core model reported %0d error(s)", u_core.err_count));
    if (u_ssd.err_count  != 0) tb_err($sformatf("T-9: SSD model reported %0d error(s)", u_ssd.err_count));

    // ---- throughput metric ----
    c0 = u_ssd.data_cpl_first_cycle - cyc0;
    c1 = u_ssd.data_cpl_last_cycle  - cyc0;
    if (c0 >= 0 && c1 > c0 && c1 < tx_valid_cum.size()) begin
      real vr, br, mbs;
      vr  = real'(tx_valid_cum[c1] - tx_valid_cum[c0]) / real'(c1 - c0);
      br  = real'(tx_beat_cum[c1]  - tx_beat_cum[c0])  / real'(c1 - c0);
      mbs = real'(TOTAL_BYTES) / (real'(c1 - c0) * 8.0e-9) / 1.0e6;
      $display("[tb_top] steady-state window: %0d cycles (first..last data CplD)", c1 - c0);
      $display("[tb_top] TX-bus valid-beat ratio = %0.3f (accepted beats/cycle = %0.3f), payload ~%0.0f MB/s",
               vr, br, mbs);
    end else
      $display("[tb_top] throughput window not available (c0=%0d c1=%0d)", c0, c1);
  endtask

  // ---------------------------------------------------------------------------
  // Main
  // ---------------------------------------------------------------------------
  initial begin
    ref_mem = new[TOTAL_BYTES];
    sys_rst_n  = 1'b0;
    data_rst   = 1'b1;
    link_ready = 1'b0; link_gen2 = 1'b0; link_width = 2'b00;
    ctrl_start = 1'b0; ctrl_stop = 1'b0;
    apply_plusargs();
    #(1us);
    sys_rst_n = 1'b1;
    repeat (20) @(posedge data_clk);
    data_rst = 1'b0;
    @(posedge user_clk);
    ctrl_start <= 1'b1;

    // link_bringup equivalent
    wait (user_lnk_up === 1'b1 && pl_sel_lnk_rate === 1'b1);
    repeat (100) @(posedge user_clk);
    link_ready <= 1'b1; link_gen2 <= 1'b1; link_width <= 2'd2;
    $display("[tb_top @%0t] link_ready asserted (LTSSM %02h, Gen2 x4)", $time, pl_ltssm_state);

    if (nobme) begin
      fork
        begin
          wait (u_ssd.err_count + u_core.err_count > 0);
          repeat (10) @(posedge user_clk);
          $display("[tb_top] +nobme: the models reported %0d error(s) as expected (Root Port BME=0)",
                   u_ssd.err_count + u_core.err_count);
          $display("TEST PASSED");
          $finish;
        end
        begin
          #(2ms);
          $fatal(1, "[tb_top] +nobme: the models did NOT report the missing Root Port BME");
        end
      join_none
      wait (0);
    end

    // stream, then stop
    wait (status_running === 1'b1);
    $display("[tb_top @%0t] RUN (NSZE %0d, LBADS %0d, MDTS %0d)", $time, stat_nsze, stat_lbads, stat_mdts);
    wait (src_done);
    $display("[tb_top @%0t] data source finished (%0d beats)", $time, src_cnt);
    repeat (200) @(posedge user_clk);
    ctrl_stop <= 1'b1;
    fork
      wait (status_done === 1'b1);
      wait (status_error != 0 && status_error !== 'x);
    join_any
    disable fork;
    repeat (50) @(posedge user_clk);
    $display("[tb_top @%0t] sequencer state %0d, status_error %02h/%02h, bytes %0d, cmds %0d, cmd_errors %0d, max_latency %0d, unexpected_cpl %0d",
             $time, status_seq_state, status_error, status_error_detail, stat_bytes_done, stat_cmds_done,
             stat_cmd_errors, stat_max_latency, stat_unexpected_cpl);
    final_checks();
    $display("--------------------------------------------------------------------------");
    $display(" tb_top summary: core TX TLPs %0d, RX TLPs %0d (straddles %0d, SOF@DW2 %0d)",
             u_core.tx_tlps, u_core.rx_tlps, u_core.n_straddles, u_core.n_sof_dw2);
    $display("   SSD: MRd %0d, CplD %0d, MWr %0d, Writes %0d, Flush %0d, CQEs %0d, data %0d B",
             u_ssd.n_mrd, u_ssd.n_cpld, u_ssd.n_mwr, u_ssd.n_io_writes, u_ssd.n_flush, u_ssd.n_cqe, u_ssd.data_bytes);
    $display("   errors: tb %0d, core model %0d, SSD model %0d", tb_errs, u_core.err_count, u_ssd.err_count);
    $display("--------------------------------------------------------------------------");
    if (tb_errs == 0 && u_core.err_count == 0 && u_ssd.err_count == 0) $display("TEST PASSED");
    else $fatal(1, "TEST FAILED");
    $finish;
  end

  // Global timeout with a diagnostic dump
  initial begin
    #(20ms);
    $display("[tb_top] TIMEOUT: seq state %0d, error %02h/%02h, running %0d, bytes %0d, cmds %0d, src %0d beats, SSD writes %0d",
             status_seq_state, status_error, status_error_detail, status_running, stat_bytes_done, stat_cmds_done,
             src_cnt, u_ssd.n_io_writes);
    $fatal(1, "[tb_top] global timeout");
  end

endmodule
