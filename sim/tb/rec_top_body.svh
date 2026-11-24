// =============================================================================
// rec_top_body.svh - body of the recorder system tests (WP8).  Included by sim/tb/tb_rec_top*.sv, which define
//   `REC_TB_NAME (module name) and `REC_SCEN_DEFAULT (comma list of scenarios; +scen=<list> overrides).
//
// DUT   : ax7a200b_rec_top (SIM_FAST=1, USE_MIG_MODEL=1) with the real nvme_host_top / link_bringup / core;
//         the PCIe IP is replaced by pcie7x_wrapper_sim (core model + nvme_ssd_model), the MIG by mig_app_model.
// Stim  : the PC side is rec_uart_bfm (921600 8N1 with real bit timing) on the uart_rx / uart_tx pins; the same
//         commands as tools/recorder.py (cmd_* helpers), replies are checked field by field against the expectations
//         of recorder.py (OK_SPEC / EV_SPEC) and spec 2.3 / 2.6.  Every received line is dumped to
//         lines_dump.txt (feed it to `python sim/vectors/check_fmt_lines.py` = recorder.parse_line(strict=True)).
// Models: SSD = dut.u_host.u_pcie.u_ssd (lba_mem, knobs), MIG model = dut.g_mig_model.u_mig (knobs).
// Volumes are scaled down (xsim speed, <= 2 min per test) compared with spec 9.3; see INTEGRATION_NOTES "WP8".
// =============================================================================
`timescale 1ns/1ps

// optional per-test-file settings (defined before the include): REC_MIG_PERIOD_PS, REC_MIG_JITTER_PS,
// REC_CALIB_FAIL (1: MIG model never calibrates), REC_IO_TIMEOUT_MS (core write-engine timeout, SIM_FAST "ms" = 1 us)
`ifndef REC_MIG_PERIOD_PS
 `define REC_MIG_PERIOD_PS 10000
`endif
`ifndef REC_MIG_JITTER_PS
 `define REC_MIG_JITTER_PS 0
`endif
`ifndef REC_CALIB_FAIL
 `define REC_CALIB_FAIL 0
`endif

module `REC_TB_NAME;
  import rec_pkg::*;

  localparam int MIG_PERIOD_PS = `REC_MIG_PERIOD_PS;
  localparam int MIG_JITTER    = `REC_MIG_JITTER_PS;

  `define SSD dut.u_host.u_pcie.u_ssd
  `define MIG dut.g_mig_model.u_mig

  // ---------------------------------------------------------------- clocks / pins
  logic free_clk_p = 1'b0, sys_clk_p = 1'b0;
  wire  free_clk_n = ~free_clk_p, sys_clk_n = ~sys_clk_p;
  initial forever #2.5 free_clk_p = ~free_clk_p;
  initial forever #5.0 sys_clk_p  = ~sys_clk_p;

  logic        rst_key_n = 1'b1;
  logic [3:0]  key_n     = 4'hF;
  wire  [3:0]  led_n;
  wire         uart_tx, uart_rx;
  wire         ssd_pwr_en;
  wire         pcie_perst_n;
  wire  [3:0]  pci_exp_txp, pci_exp_txn;
  wire  [31:0] ddr3_dq;
  wire  [3:0]  ddr3_dqs_p, ddr3_dqs_n, ddr3_dm;
  wire  [14:0] ddr3_addr;
  wire  [2:0]  ddr3_ba;
  wire         ddr3_ras_n, ddr3_cas_n, ddr3_we_n, ddr3_reset_n;
  wire  [0:0]  ddr3_ck_p, ddr3_ck_n, ddr3_cke, ddr3_cs_n, ddr3_odt;

  ax7a200b_rec_top #(
    .SIM_FAST(1'b1), .USE_MIG_MODEL(1'b1), .DEBOUNCE_BITS(4), .REQ_TIMEOUT_CYCLES(200000),
    .MIG_UI_PERIOD_PS(MIG_PERIOD_PS), .MIG_JITTER_PS(MIG_JITTER),
    .XADC_SIM_FILE("../../vectors/xadc_stim.txt")
  ) dut (
    .sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n), .free_clk_p(free_clk_p), .free_clk_n(free_clk_n),
    .pcie_perst_n(pcie_perst_n), .pci_exp_txp(pci_exp_txp), .pci_exp_txn(pci_exp_txn),
    .pci_exp_rxp(4'b0), .pci_exp_rxn(4'b1111),
    .rst_key_n(rst_key_n), .key_n(key_n), .led_n(led_n), .uart_tx(uart_tx), .uart_rx(uart_rx),
    .ssd_pwr_en(ssd_pwr_en),
    .ddr3_dq(ddr3_dq), .ddr3_dqs_p(ddr3_dqs_p), .ddr3_dqs_n(ddr3_dqs_n), .ddr3_addr(ddr3_addr), .ddr3_ba(ddr3_ba),
    .ddr3_ras_n(ddr3_ras_n), .ddr3_cas_n(ddr3_cas_n), .ddr3_we_n(ddr3_we_n), .ddr3_reset_n(ddr3_reset_n),
    .ddr3_ck_p(ddr3_ck_p), .ddr3_ck_n(ddr3_ck_n), .ddr3_cke(ddr3_cke), .ddr3_cs_n(ddr3_cs_n),
    .ddr3_dm(ddr3_dm), .ddr3_odt(ddr3_odt));

  rec_uart_bfm u_bfm (.txd(uart_rx), .rxd(uart_tx));

`ifdef REC_IO_TIMEOUT_MS
  defparam dut.u_host.u_core.IO_TIMEOUT_MS = `REC_IO_TIMEOUT_MS;
`endif

  // ---------------------------------------------------------------- check helpers
  int errs = 0;
  string scen_list;
  string cur_scen = "";
  function automatic bit want(input string s);
    // scen_list is a comma list; match whole items
    string l;
    l = {",", scen_list, ","};
    for (int i = 0; i + s.len() + 2 <= l.len(); i++)
      if (l.substr(i, i + s.len() + 1) == {",", s, ","}) return 1'b1;
    return 1'b0;
  endfunction

  task automatic fail(input string m);
    errs++;
    $display("[%0t] FAIL %s: %s", $time, cur_scen, m);
  endtask
  task automatic ck(input bit c, input string m);
    if (!c) fail(m);
  endtask
  task automatic ck_eq(input string what, input longint got, input longint exp);
    if (got !== exp) fail($sformatf("%s: got %h, expected %h", what, got, exp));
  endtask
  task automatic ck_f(input string line, input string key, input longint exp);
    longint v;
    v = u_bfm.fld(line, key);
    if (v !== exp) fail($sformatf("%s: got %h expected %h in [%s]", key, v, exp, line));
  endtask
  task automatic ck_pfx(input string line, input string pfx);
    if (!(line.len() >= pfx.len() && line.substr(0, pfx.len() - 1) == pfx)) fail($sformatf("expected prefix '%s', got [%s]", pfx, line));
  endtask

  task automatic say(input string s);
    $display("[%0t] %s: %s", $time, cur_scen, s);
  endtask

  // command with reply check "OK <name>" / returns the reply
  task automatic cmd_ok(input string c, input string pfx, output string r);
    u_bfm.cmd(c, r);
    if (r.len() == 0) fail($sformatf("no reply to '%s'", c));
    else ck_pfx(r, pfx);
  endtask
  task automatic cmd_err(input string c, input int code);
    string r, e;
    u_bfm.cmd(c, r);
    e = $sformatf("ERR %02x", code);
    e = {e.substr(0, 3), e.substr(4, 5).toupper()};
    if (r != e) fail($sformatf("'%s': expected [%s], got [%s]", c, e, r));
  endtask

  // wait for the next line with prefix after index `from`; fails on timeout
  task automatic wait_ev(input string pfx, input int from, input int tmo_us, output string l, output int idx);
    u_bfm.wait_line(pfx, from, tmo_us, idx);
    if (idx < 0) begin fail($sformatf("timeout waiting for '%s' (%0d us)", pfx, tmo_us)); l = ""; end
    else l = u_bfm.lines[idx];
  endtask

  // wait until a "!S st=0" line after `from` (session finished, rec_ctrl idle)
  task automatic wait_idle(input int from, input int tmo_us);
    int i, t;
    bit ok;
    t = 0; ok = 0;
    while (!ok && t < tmo_us) begin
      for (i = from; i < u_bfm.lines.size(); i++)
        if (u_bfm.lines[i].len() > 9 && u_bfm.lines[i].substr(0, 7) == "!S st=0 ") ok = 1;
      if (!ok) begin #(1000.0); t++; end
    end
    if (!ok) fail("timeout waiting for !S st=0");
  endtask

  // ---------------------------------------------------------------- SSD / block checks
  function automatic logic [127:0] ssd_beat(input longint blkno, input int beat, input longint base_lba = 0);
    logic [127:0] v;
    longint lba;
    int off;
    // 4096-byte block = 8 LBAs of 512 B; beat = 16 bytes
    lba = base_lba + blkno * 8 + (beat * 16) / 512;
    off = (beat * 16) % 512;
    v = '0;
    if (`SSD.lba_mem.exists(lba)) begin
      for (int b = 0; b < 16; b++) v[8*b +: 8] = `SSD.lba_mem[lba][off + b];
    end
    return v;
  endfunction
  function automatic bit ssd_has(input longint blkno, input longint base_lba = 0);
    return `SSD.lba_mem.exists(base_lba + blkno * 8);
  endfunction

  // check n blocks from block 0: header, CRC, seq/pos chain; bad = number of bad blocks.
  task automatic check_blocks(input longint n, input logic [31:0] exp_sid, input bit allow_lost,
                              output int bad, output longint last_seq, output longint last_lost);
    logic [127:0] b0, b1, b2, b255;
    logic [31:0]  crc;
    longint       pos, seq, lost;
    bad = 0; last_seq = 0; last_lost = 0;
    for (longint k = 0; k < n; k++) begin
      if (!ssd_has(k)) begin bad++; if (bad < 5) say($sformatf("block %0d missing on the SSD", k)); continue; end
      b0   = ssd_beat(k, 0);
      b1   = ssd_beat(k, 1);
      b2   = ssd_beat(k, 2);
      b255 = ssd_beat(k, 255);
      crc  = CRC32_INIT;
      for (int j = 0; j < 255; j++) crc = crc32_128(crc, ssd_beat(k, j));
      pos  = b0[111:64];
      seq  = b1[47:0];
      lost = b2[95:64];
      if (b0[31:0] != BLK_MAGIC || b0[63:32] != exp_sid || pos != k || b255[127:96] != BLK_MAGIC_END ||
          b255[95:64] != ~crc || b255[63:32] != pos[31:0] || b255[31:0] != seq[31:0]) begin
        bad++;
        if (bad < 5) say($sformatf("block %0d bad: magic %h sid %h pos %0d seq %0d crc %h/%h trailer %h", k, b0[31:0], b0[63:32],
                                  pos, seq, b255[95:64], ~crc, b255));
      end else if (!allow_lost && seq != pos) begin
        bad++;
        if (bad < 5) say($sformatf("block %0d seq %0d != pos", k, seq));
      end
      last_seq = seq; last_lost = lost;
    end
  endtask

  // ---------------------------------------------------------------- common session helpers
  task automatic setup_rec(input logic [31:0] sid, input int len_slots, input logic [31:0] step, input int ddr_mib);
    string r;
    cmd_ok($sformatf("SID %08X", sid), "OK SID", r);
    cmd_ok($sformatf("LEN %08X", len_slots), "OK LEN", r);
    cmd_ok($sformatf("RATE %08X", step), "OK RATE", r);
    cmd_ok($sformatf("DDRSZ %04X", ddr_mib), "OK DDRSZ", r);
  endtask

  // ---------------------------------------------------------------- scenarios
  `include "rec_top_scen.svh"

  // ---------------------------------------------------------------- main
  initial begin
    if (!$value$plusargs("scen=%s", scen_list)) scen_list = `REC_SCEN_DEFAULT;
    $display("%s: scenarios %s", `"`REC_TB_NAME`", scen_list);
    // repeated sessions overwrite LBA 0..
    `SSD.allow_overwrite = 1'b1;
    // the stand's link is Gen2 x2 (<= 1 GB/s): throttle the model's upstream (read data) path to ~0.5 beat per user clock,
    // otherwise the unthrottled Gen2 x4 model (2 GB/s bursts) overflows the 512-entry vrf_tap FIFO (audit/INTEGRATION_NOTES WP8)
    dut.u_host.u_pcie.u_core.rx_midpkt_gap_pct = 50;
    if (`REC_CALIB_FAIL) `MIG.calib_fail = 1'b1;
    // reset key pulse
    rst_key_n = 1'b0; #2000; rst_key_n = 1'b1;
    run_scenarios();
    u_bfm.dump("lines_dump.txt");
    ck(u_bfm.framing_errs == 0, "UART framing errors from the device");
    ck(`SSD.err_count == 0, "nvme_ssd_model reported errors");
    if (errs == 0) $display("TEST PASSED");
    else           $fatal(1, "%0d check(s) failed", errs);
    $finish;
  end

  // optional trace of the verify tap stream (debug, +trace via scenario D2)
  int trace_n = 0;
  always @(posedge dut.ui_clk) if (trace_n > 0 && dut.vrf_tvalid && dut.vrf_tready) begin
    trace_n--;
    $display("[%0t] TAP lba=%h off=%h be=%h d=%h", $time, dut.vrf_tdata[192:161], dut.vrf_tdata[160:144], dut.vrf_tdata[143:128], dut.vrf_tdata[127:0]);
  end

  // progress (1 line per 2 ms of simulated time) and global watchdog
  initial forever begin #(2_000_000.0); $display("[%0t] ... %s", $time, cur_scen); end
  initial begin
    #(3_000_000_000.0);   // 3 s of simulated time
    $fatal(1, "global timeout");
  end
endmodule
