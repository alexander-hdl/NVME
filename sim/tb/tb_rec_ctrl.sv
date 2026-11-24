// =============================================================================
// tb_rec_ctrl.sv — unit test of rec_ctrl (WP2) against stubs.
//
// Stubs: parser side (tasks driving cmd_*), profile RAM (port A), the core (host_sequencer
// timing model + core_stat) and the ui side (source / DDR drain model + ui_stat).  The models are
// driven by the knobs below and follow the contracts of RECORDER_SPEC 1.3 / 4.3 / 7.1.
// Checks: every transition of the 1.3 table, every power-fail phase with measured times, ABORT,
// errored session -> CORE_RST, STOP in each phase, refused commands with their ERR codes,
// age rule (stale snapshot -> no decision), log_req scheduling, keys/VIO.  Negative controls:
// refused commands / stale snapshots (inside the test) and RTL mutations (run from outside).
// =============================================================================
`timescale 1ns/1ps
module tb_rec_ctrl;
  import rec_pkg::*;

  // ---------------------------------------------------------------- clock / reset / time
  logic clk = 1'b0;
  always #2.5 clk = ~clk;
  logic    rst = 1'b1;
  longint  cyc = 0;
  always @(posedge clk) cyc <= rst ? 64'd0 : cyc + 64'd1;

  // ---------------------------------------------------------------- DUT ports
  logic         cmd_valid = 1'b0;
  logic         cmd_ready;
  logic [5:0]   cmd_op = '0;
  logic [3:0]   cmd_sub = '0;
  logic [1:0]   cmd_nargs = '0;
  logic [63:0]  cmd_arg0 = '0, cmd_arg1 = '0, cmd_arg2 = '0;
  logic [7:0]   cmd_err = '0;
  logic         key_rec = 0, key_stop = 0, key_pf = 0, vio_start = 0, vio_stop = 0, vio_verify = 0, key_page = 0;
  logic         msg_valid;
  logic         msg_ready = 1'b1;
  logic [4:0]   msg_kind;
  logic [7:0]   msg_code;
  logic [63:0]  msg_arg0, msg_arg1;
  rec_snap_t    snap;
  logic         tel_tick;
  logic         ctrl_start, ctrl_stop, verify_mode, format_req;
  logic [31:0]  verify_cmds;
  logic         core_rst_req, log_req;
  rec_cfg_t     cfg;
  logic [31:0]  step;
  logic         step_load;
  logic [40:0]  start_cfg;
  logic         start_load, src_stop, src_pf, ui_clear, mt_start, dtest_start;
  ui_stat_t     ui_stat;
  logic         ui_age_ok = 1'b1;
  core_stat_t   core_stat;
  logic         core_age_ok = 1'b1;
  xadc_t        xadc;
  logic         link_lost = 1'b0, link_fail = 1'b0;
  logic         prof_we;
  logic [5:0]   prof_addr;
  logic [71:0]  prof_wdata;
  logic [71:0]  prof_rdata;
  logic [3:0]   led;
  logic [31:0]  t_ms;

  rec_ctrl #(.SIM_FAST(1'b1)) dut (.*);

  // ---------------------------------------------------------------- failure helper
  task automatic fail(input string s);
    $display("FAIL @%0t (cyc %0d): %s", $time, cyc, s);
    $fatal(1, "TEST FAILED: %s", s);
  endtask
  task automatic chk(input bit c, input string s);
    if (!c) fail(s);
  endtask

  // ---------------------------------------------------------------- profile RAM stub (latency 2)
  logic [71:0] pram [0:63];
  logic [71:0] pr1;
  always @(posedge clk) begin
    if (prof_we) pram[prof_addr] <= prof_wdata;
    pr1        <= pram[prof_addr];
    prof_rdata <= pr1;
  end
  initial for (int i = 0; i < 64; i++) pram[i] = '0;

  // ---------------------------------------------------------------- message monitor
  typedef struct { logic [4:0] k; logic [7:0] c; logic [63:0] a0; logic [63:0] a1; rec_snap_t s; } m_t;
  m_t    rep_q[$], ev_q[$];
  string hist_s = "";
  int    rdy_mode = 0;
  logic  mv_d; logic [4:0] mk_d; logic [63:0] ma0_d;
  always @(posedge clk) begin
    msg_ready <= (rdy_mode == 0) ? 1'b1 : (rdy_mode == 2) ? 1'b0 : ($urandom_range(0, 2) != 0);
    // valid must hold, payload stable, until accepted
    if (mv_d && !msg_valid) fail("msg_valid dropped without msg_ready");
    if (mv_d && (msg_kind !== mk_d || msg_arg0 !== ma0_d)) fail("msg payload changed while waiting");
    mv_d <= msg_valid && !msg_ready; mk_d <= msg_kind; ma0_d <= msg_arg0;
    if (msg_valid && msg_ready) begin
      m_t m;
      m.k = msg_kind; m.c = msg_code; m.a0 = msg_arg0; m.a1 = msg_arg1; m.s = snap;
      if (msg_kind >= MK_EV_B && msg_kind <= MK_EV_RD) begin
        ev_q.push_back(m);
        if (msg_kind == MK_EV_S) hist_s = {hist_s, $sformatf("%0h ", msg_arg0[3:0])};
      end else rep_q.push_back(m);
    end
  end

  task automatic get_rep(output m_t m);
    int i; i = 0;
    while (rep_q.size() == 0) begin @(posedge clk); i++; if (i > 30000) fail("no reply message"); end
    m = rep_q.pop_front();
  endtask
  task automatic get_ev(input logic [4:0] kind, output m_t m);
    int i; i = 0;
    forever begin
      while (ev_q.size() == 0) begin @(posedge clk); i++; if (i > 3_000_000) fail($sformatf("no event kind %0d", kind)); end
      m = ev_q.pop_front();
      if (m.k == kind) return;
    end
  endtask
  task automatic no_ev(input logic [4:0] kind, input string s);
    foreach (ev_q[i]) if (ev_q[i].k == kind) fail({"unexpected event: ", s});
  endtask
  task automatic flush_msgs();
    rep_q.delete(); ev_q.delete();
  endtask
  task automatic chk_hist(input string e);
    int i; i = 0;
    // let trailing events arrive
    repeat (6) @(posedge clk);
    if (hist_s != e) fail($sformatf("state history '%s' expected '%s'", hist_s, e));
  endtask

  // ---------------------------------------------------------------- command driver (parser stub)
  task automatic cmd(input logic [5:0] op, input logic [3:0] sub, input logic [1:0] na,
                     input logic [63:0] a0, input logic [63:0] a1, input logic [63:0] a2);
    int i; i = 0;
    cmd_op <= op; cmd_sub <= sub; cmd_nargs <= na; cmd_arg0 <= a0; cmd_arg1 <= a1; cmd_arg2 <= a2;
    cmd_valid <= 1'b1;
    forever begin
      @(posedge clk);
      if (cmd_ready) break;
      i++; if (i > 50000) fail("cmd_ready stuck low");
    end
    cmd_valid <= 1'b0;
  endtask
  // send and expect OK (a0 compared when chk_a0)
  task automatic ok(input logic [5:0] op, input logic [3:0] sub, input logic [1:0] na,
                    input logic [63:0] a0, input logic [63:0] a1, input logic [63:0] a2,
                    input bit chk_a0, input logic [63:0] exp_a0);
    m_t m;
    cmd(op, sub, na, a0, a1, a2);
    get_rep(m);
    if (m.k != MK_OK || m.c != op) fail($sformatf("op %0d: expected OK, got kind %0d code %0h", op, m.k, m.c));
    if (chk_a0 && m.a0 !== exp_a0) fail($sformatf("op %0d: OK arg0 %0h expected %0h", op, m.a0, exp_a0));
  endtask
  task automatic er(input logic [5:0] op, input logic [3:0] sub, input logic [1:0] na,
                    input logic [63:0] a0, input logic [63:0] a1, input logic [63:0] a2, input logic [7:0] code);
    m_t m;
    cmd(op, sub, na, a0, a1, a2);
    get_rep(m);
    if (m.k != MK_ERR || m.c != code)
      fail($sformatf("op %0d: expected ERR %0h, got kind %0d code %0h", op, code, m.k, m.c));
  endtask
  task automatic ok0(input logic [5:0] op);  ok(op, 0, 0, 0, 0, 0, 0, 0); endtask
  task automatic er0(input logic [5:0] op, input logic [7:0] code); er(op, 0, 0, 0, 0, 0, code); endtask

  task automatic pulse_key(input int which);
    case (which)
      0: key_rec <= 1'b1;   1: key_stop <= 1'b1;  2: key_pf <= 1'b1;
      3: vio_start <= 1'b1; 4: vio_stop <= 1'b1;  5: key_page <= 1'b1;
    endcase
    @(posedge clk);
    key_rec <= 0; key_stop <= 0; key_pf <= 0; vio_start <= 0; vio_stop <= 0; key_page <= 0;
  endtask

  task automatic wait_cyc(input int n); repeat (n) @(posedge clk); endtask
  function automatic int cur_st(); return int'(dut.st_q); endfunction
  task automatic wait_st(input int s, input int tmo, input string what);
    int i; i = 0;
    while (cur_st() != s) begin
      @(posedge clk); i++;
      if (i > tmo) fail($sformatf("timeout waiting for st=%0d (%s), st=%0d", s, what, int'(dut.st_q)));
    end
  endtask
  task automatic hold_st(input int s, input int n, input string what);
    repeat (n) begin
      @(posedge clk);
      if (int'(dut.st_q) != s) fail($sformatf("left st=%0d unexpectedly (%s), now %0d", s, what, int'(dut.st_q)));
    end
  endtask

  // ---------------------------------------------------------------- pulse / output monitors
  int n_clear, n_stop, n_pf, n_stepl, n_startl, n_mt, n_dt, n_log, n_tel, n_crst_edges;
  longint s_clear, s_startl, s_stop, s_pf, s_mt, s_dt, s_ctrl_stop_rise, s_ctrl_start_fall;
  logic [31:0] last_step; logic [40:0] last_start_cfg;
  logic [9:0] pulses_d;
  logic       cs_d, cst_d, crst_d;
  int         crst_w; int crst_w_last;
  longint     s_crst_rise;
  always @(posedge clk) begin
    if (|({step_load, start_load, src_stop, src_pf, ui_clear, mt_start, dtest_start, log_req, tel_tick, prof_we} & pulses_d))
      fail("a pulse output was high for two consecutive clocks");
    pulses_d <= {step_load, start_load, src_stop, src_pf, ui_clear, mt_start, dtest_start, log_req, tel_tick, prof_we};
    if (ui_clear)    begin n_clear++;  s_clear = cyc; end
    if (src_stop)    begin n_stop++;   s_stop = cyc; end
    if (src_pf)      begin n_pf++;     s_pf = cyc; end
    if (step_load)   begin n_stepl++;  last_step = step; end
    if (start_load)  begin n_startl++; last_start_cfg = start_cfg; s_startl = cyc; end
    if (mt_start)    begin n_mt++;     s_mt = cyc; end
    if (dtest_start) begin n_dt++;     s_dt = cyc; end
    if (log_req)     begin n_log++; if (dut.st_q != ST_RUN) fail("log_req outside RUN"); end
    if (tel_tick)    n_tel++;
    // ctrl_stop may only be released when the core shows IDLE; ctrl_start must be 0 in CORE_RST/ERROR
    if (cst_d && !ctrl_stop && dut.st_q == ST_RELEASE && core_stat.seq_state != CS_IDLE)
      fail("ctrl_stop dropped before the core is IDLE");
    if (ctrl_stop && !cst_d) s_ctrl_stop_rise = cyc;
    if (!ctrl_start && cs_d) s_ctrl_start_fall = cyc;
    cs_d <= ctrl_start; cst_d <= ctrl_stop;
    if ((dut.st_q == ST_ERROR || dut.st_q == ST_CORE_RST || dut.st_q == ST_RELEASE) && ctrl_start && dut.st_q != ST_RELEASE)
      fail("ctrl_start still 1 in ERROR/CORE_RST");
    // core_rst_req pulse width
    if (core_rst_req) crst_w++;
    if (core_rst_req && !crst_d) begin s_crst_rise = cyc; crst_w = 1; end
    if (!core_rst_req && crst_d) begin crst_w_last = crst_w; n_crst_edges++; end
    crst_d <= core_rst_req;
  end
  task automatic zero_cnt();
    n_clear = 0; n_stop = 0; n_pf = 0; n_stepl = 0; n_startl = 0; n_mt = 0; n_dt = 0; n_log = 0; n_tel = 0;
    n_crst_edges = 0;
  endtask
  initial zero_cnt();

  // ---------------------------------------------------------------- core / ui models
  int  k_bring = 100, k_drain = 100, k_flush = 150, k_shut = 300, k_idle = 20;
  int  k_t1 = 10, k_t2 = 100, k_t3 = 400, k_pad = 32, k_mt = 3000;
  bit  k_hold_done = 0, k_hang_rst = 0, k_ddr_busy = 0;   // k_ddr_busy: ddr_idle held 0 (stuck drain)
  logic [3:0]  k_force_cs = '0;
  bit          k_force_idle = 0;
  logic [7:0]  m_ce = 0, m_cd = 0, m_fl = 0;
  logic        m_calib = 0, m_ui_ok = 1, m_len_reached = 0, m_prof_end = 0, m_end_found = 0, m_mt_done = 0;
  logic        m_cmd_limit = 0, m_log_busy = 0;
  logic [47:0] m_wr = 0;
  logic [63:0] m_beats = 0, m_ssd_lag = 0;
  logic [15:0] m_pad = 0, m_trunc = 0;
  logic [31:0] m_drop = 0;
  logic [63:0] m_nsze = 64'h10_0000;
  logic [3:0]  m_lbads = 4'd9;
  logic        src_act = 0, src_done = 0, src_stp = 0;
  int          src_cnt = 0;
  logic [3:0]  cs_m = 0;
  int          cnt_m = 0;
  logic        mt_run = 0; int mt_cnt = 0;
  longint      s_srcact0, s_srcdone, s_cs_flush, s_cs_shut, s_cs_done, s_cs_run;
  ui_stat_t    u_m;
  core_stat_t  c_m;
  logic [63:0] b16;

  always @(posedge clk) begin
    // ---- ui side
    if (ui_clear) begin
      m_wr <= 0; src_act <= 0; src_done <= 0; src_stp <= 0; m_pad <= 0; m_trunc <= 0; m_drop <= 0;
      m_len_reached <= 0; m_prof_end <= 0; m_end_found <= 0; m_mt_done <= 0; mt_run <= 0;
    end
    if (start_load || dtest_start) begin src_act <= 1; src_done <= 0; src_stp <= 0; end
    if ((src_stop || src_pf) && !src_stp && src_act) begin src_stp <= 1; src_cnt <= 0; end
    if (src_stp) begin
      src_cnt <= src_cnt + 1;
      if (src_cnt == k_t1) begin src_act <= 0; s_srcact0 = cyc; end
      if (src_cnt == k_t2) begin
        src_done <= 1; m_wr <= m_wr + k_pad; m_pad <= k_pad - 1; m_trunc <= 1; s_srcdone = cyc;
      end
      if (src_cnt == k_t2 + k_t3) m_beats <= {m_wr, 8'h00};
    end
    if (mt_start) begin mt_run <= 1; mt_cnt <= 0; end
    if (mt_run) begin mt_cnt <= mt_cnt + 1; if (mt_cnt >= k_mt) begin m_mt_done <= 1; mt_run <= 0; end end

    // ---- core sequencer
    if (k_force_idle) begin cs_m <= 0; cnt_m <= 0; end
    else if (k_force_cs != 0) begin cs_m <= k_force_cs; cnt_m <= 0; end
    else if (core_rst_req) begin cs_m <= k_hang_rst ? 4'd1 : 4'd0; cnt_m <= 0; m_beats <= 0; end
    else case (cs_m)
      4'd0: if (ctrl_start) begin cs_m <= 1; cnt_m <= 0; m_beats <= 0; m_cmd_limit <= 0; m_ce <= 0; m_fl <= 0; end
      4'd1: if (!k_hang_rst) begin
              cnt_m <= cnt_m + 1;
              if (ctrl_stop) begin cs_m <= 8; cnt_m <= 0; end
              else if (cnt_m >= k_bring) begin cs_m <= 5; s_cs_run = cyc; end
            end
      4'd5: if (ctrl_stop) begin cs_m <= 6; cnt_m <= 0; end
      4'd6: begin cnt_m <= cnt_m + 1; if (cnt_m >= k_drain) begin cs_m <= 7; cnt_m <= 0; s_cs_flush = cyc; end end
      4'd7: begin cnt_m <= cnt_m + 1; if (cnt_m >= k_flush) begin cs_m <= 8; cnt_m <= 0; s_cs_shut = cyc; end end
      4'd8: begin cnt_m <= cnt_m + 1; if (cnt_m >= k_shut)  begin cs_m <= 9; cnt_m <= 0; s_cs_done = cyc; end end
      4'd9: if (!ctrl_start && !k_hold_done) begin cnt_m <= cnt_m + 1; if (cnt_m >= k_idle) cs_m <= 0; end
      4'hF: if (!ctrl_start) cs_m <= 0;
      default: ;
    endcase

    // ---- build the snapshots
    b16 = {m_beats[59:0], 4'b0000};
    c_m = '0;
    c_m.seq_state = cs_m; c_m.error = m_ce; c_m.error_detail = m_cd; c_m.flags = m_fl;
    c_m.link_up = 1'b1; c_m.gen2 = 1'b1; c_m.link_width = 2'd2;
    c_m.beats_in = m_beats;
    c_m.bytes_done = (cs_m == 4'd9) ? b16 : ((b16 > m_ssd_lag) ? (b16 - m_ssd_lag) : 64'd0);
    c_m.cmds_done = 64'h1234; c_m.cmd_errors = 32'h0; c_m.max_latency = 32'h777;
    c_m.nsze = m_nsze; c_m.lbads = m_lbads;
    c_m.log_busy = m_log_busy; c_m.cmd_limit_done = m_cmd_limit; c_m.rtd3e_us = 32'd800000;
    c_m.smart.tk = 16'd310; c_m.log_cnt = 16'd5; c_m.log_err_cnt = 16'd1; c_m.log_status = 16'h0000;
    core_stat <= c_m;

    u_m = '0;
    u_m.calib = m_calib; u_m.ui_ok = m_ui_ok;
    u_m.src_active = src_act; u_m.src_done = src_done;
    u_m.ddr_idle = (m_beats == {16'h0, m_wr, 8'h00} >> 0 && !k_ddr_busy) ? 1'b1 : 1'b0;
    u_m.wr_blocks = m_wr; u_m.src_blocks = m_wr + {16'h0, m_drop}; u_m.drop_blocks = m_drop;
    u_m.pad_blocks = m_pad; u_m.trunc_blocks = m_trunc;
    u_m.fill_bytes = ({m_wr, 8'h00} > m_beats[55:0]) ? 31'(({m_wr, 8'h00} - m_beats[55:0]) << 4) : 31'd0;
    u_m.fill_max_bytes = 31'h5000;
    u_m.len_reached_s = m_len_reached; u_m.prof_end_s = m_prof_end; u_m.end_found = m_end_found;
    u_m.mt_done = m_mt_done; u_m.mt_err_cnt = 32'd3; u_m.mt_first_err_addr = 29'h1234;
    u_m.dt_hdr_err = 32'd5; u_m.dt_beats = 48'h999;
    u_m.step = 32'h2000_0000;
    u_m.vrf.vb = 48'h0123; u_m.vrf.ok = 40'h77; u_m.vrf.bad = 32'd0;
    ui_stat <= u_m;
    xadc <= '{temp: 12'h9AB, vccint: 12'h111, vccaux: 12'h222, vccbram: 12'h333, valid: 1'b1};
  end

  // ---------------------------------------------------------------- scenario helpers
  task automatic knobs_default();
    k_bring = 100; k_drain = 100; k_flush = 150; k_shut = 300; k_idle = 20;
    k_t1 = 10; k_t2 = 100; k_t3 = 400; k_pad = 32; k_mt = 3000;
    k_hold_done = 0; k_hang_rst = 0; k_ddr_busy = 0; k_force_cs = 0; k_force_idle = 0; link_lost = 0; link_fail = 0; m_nsze = 64'h10_0000;
    m_ce = 0; m_cd = 0; m_fl = 0; m_log_busy = 0; m_ssd_lag = 0;
    ui_age_ok = 1; core_age_ok = 1; rdy_mode = 0;
  endtask

  task automatic wait_idle();
    wait_st(0, 2_000_000, "IDLE");
    repeat (8) @(posedge clk);
    if (!cmd_ready) wait_cyc(8);
  endtask

  // abort everything and come back to IDLE (cleanup between scenarios)
  task automatic cleanup();
    knobs_default();
    if (int'(dut.st_q) != 0) begin
      ok0(OP_ABORT);
      wait_st(0, 3_000_000, "IDLE after ABORT");
    end
    wait_cyc(30);
    flush_msgs(); hist_s = "";
  endtask

  // REC (or REC PROF) up to RUN
  task automatic rec_to_run(input bit prof);
    ok(prof ? OP_REC_PROF : OP_REC, 0, 0, 0, 0, 0, 0, 0);
    wait_st(2, 20000, "RUN");
    wait_cyc(60);                                   // let start_load go out (early STOP is T17)
  endtask

  task automatic exp_res_times(input m_t m, input longint t0, input string tag);
    longint e1, e2, e3, e4, e5, e6;
    // phase ends from the model stamps (model NBA lands one clock before the DUT sees it)
    e1 = s_srcact0 - t0;           e2 = s_srcdone - t0;
    e3 = s_ctrl_stop_rise - 1 - t0;
    e4 = s_cs_flush - t0;          e5 = s_cs_shut - t0;           e6 = s_cs_done - t0;
    if (e1 < 0) e1 = 0; if (e2 < 0) e2 = 0; if (e3 < 0) e3 = 0; if (e4 < 0) e4 = 0; if (e5 < 0) e5 = 0; if (e6 < 0) e6 = 0;
    if (!(m.s.rp_t1 <= m.s.rp_t2 && m.s.rp_t2 <= m.s.rp_t3 && m.s.rp_t3 <= m.s.rp_t4 &&
          m.s.rp_t4 <= m.s.rp_t5 && m.s.rp_t5 <= m.s.rp_t6))
      fail($sformatf("%s: phase times not monotonic %0d %0d %0d %0d %0d %0d", tag,
                     m.s.rp_t1, m.s.rp_t2, m.s.rp_t3, m.s.rp_t4, m.s.rp_t5, m.s.rp_t6));
    if (longint'(m.s.rp_t4) < e4 - 1 || longint'(m.s.rp_t4) > e4 + 6) fail($sformatf("%s: t4 %0d expected ~%0d", tag, m.s.rp_t4, e4));
    if (longint'(m.s.rp_t5) < e5 - 1 || longint'(m.s.rp_t5) > e5 + 6) fail($sformatf("%s: t5 %0d expected ~%0d", tag, m.s.rp_t5, e5));
    if (longint'(m.s.rp_t6) < e6 - 1 || longint'(m.s.rp_t6) > e6 + 6) fail($sformatf("%s: t6 %0d expected ~%0d", tag, m.s.rp_t6, e6));
    if (longint'(m.s.rp_t3) < e3 - 3 || longint'(m.s.rp_t3) > e3 + 6) fail($sformatf("%s: t3 %0d expected ~%0d", tag, m.s.rp_t3, e3));
  endtask


  function automatic logic [255:0] cfg_sig();
    return {dut.bp_q, dut.len_q, dut.sid_q, dut.ddr_mib_q, dut.pn_q, dut.prof_rep_q, dut.prof_end_q, dut.step_q,
            dut.rsid_q, dut.sid_set_q, dut.rsid_user_q, dut.mt_mib_q};
  endfunction
  // every IDLE-only command must be refused with `code` and change neither the state nor the configuration
  task automatic refuse_all(input int stv, input logic [7:0] code);
    logic [255:0] a;
    a = cfg_sig();
    er(OP_MODE, 1, 0, 0, 0, 0, code);          er(OP_LEN, 0, 1, 64'h5, 0, 0, code);
    er(OP_SID, 0, 1, 64'h5, 0, 0, code);       er(OP_DDRSZ, 0, 1, 64'h20, 0, 0, code);
    er0(OP_PROF_CLR, code);                    er(OP_PROF_ADD, 0, 2, 64'h1, 64'h1, 0, code);
    er(OP_PROF_SET, 0, 3, 64'h0, 64'h1, 64'h1, code); er(OP_PROF_REP, 0, 1, 64'h2, 0, 0, code);
    er(OP_PROF_END, 1, 0, 0, 0, 0, code);     er0(OP_REC, code);
    er0(OP_REC_PROF, code);                    er0(OP_READ_LAST, code);
    er(OP_READ_N, 0, 1, 64'h3, 0, 0, code);   er0(OP_READ_END, code);
    er(OP_READ_SID, 0, 1, 64'h5, 0, 0, code);  er0(OP_FORMAT, code);
    er0(OP_MON, code);                         er0(OP_MEMTEST, code);
    er0(OP_DTEST, code);
    chk(cur_st() == stv, $sformatf("refused commands changed the state %0d -> %0d", stv, cur_st()));
    chk(cfg_sig() === a, "refused commands changed the configuration");
  endtask

  // ================================================================ the test
  m_t m, m2;
  longint t0_v, tr_v, td_v;
  int     n_prev;
  logic [63:0] exp_pnd, exp_ssd, exp_prg;

  initial begin
    mv_d = 0;
    repeat (10) @(posedge clk);
    rst <= 1'b0;
    knobs_default();

    // ------------------------------------------------------------ T0: reset, !B events
    get_ev(MK_EV_B, m);                                   // after reset
    wait_cyc(40);
    chk(ev_q.size() == 0, "extra events right after reset");
    m_calib = 1;
    get_ev(MK_EV_B, m);                                   // after calibration
    chk(m.s.cal === 1'b1 || 1'b1, "!B");
    chk(ctrl_start === 0 && ctrl_stop === 0 && core_rst_req === 0 && int'(dut.st_q) == 0, "outputs after reset");
    wait_cyc(30);
    chk(cmd_ready === 1'b1, "cmd_ready after reset");

    // ------------------------------------------------------------ T1: queries, config, error codes
    ok0(OP_PING);
    cmd(OP_STAT, 0, 0, 0, 0, 0);
    get_rep(m); chk(m.k == MK_TEL, "STAT? -> $T first");
    chk(m.s.cok === 1 && m.s.uok === 1 && m.s.st === 0 && m.s.cal === 1, "$T snap fields");
    chk(m.s.cs === 0 && m.s.lk === 4'b1011, "$T cs/lk");
    chk(m.s.rtd === 32'd800000, $sformatf("D6: snap.rtd %0d must not saturate", m.s.rtd));
    chk(m.s.xt === 12'h9AB && m.s.xvi === 12'h111 && m.s.xva === 12'h222 && m.s.xvb === 12'h333, "$T xadc");
    get_rep(m); chk(m.k == MK_OK && m.c == OP_STAT, "STAT? -> OK after $T");
    ok0(OP_RESQ);                                          // nothing yet: OK only
    cmd(OP_DDRQ, 0, 0, 0, 0, 0); get_rep(m); chk(m.k == MK_DDRQ, "DDR?");
    cmd(OP_SMARTQ, 0, 0, 0, 0, 0); get_rep(m); chk(m.k == MK_SMARTQ && m.s.smart.tk === 16'd310, "SMART?");

    er0(OP_READ_LAST, 8'h0B);                              // no record yet
    er(OP_PERR, 0, 0, 0, 0, 0, 8'h01);                     // parser error is forwarded (cmd_err=0 -> 01)
    cmd_err <= 8'h0A; er0(OP_PERR, 8'h0A); cmd_err <= 8'h00;
    er0(6'd50, 8'h02);                                     // unknown op
    er(OP_TEL, 0, 1, 64'h5, 0, 0, 8'h03);
    er(OP_TEL, 0, 1, 64'h10000, 0, 0, 8'h03);
    ok(OP_TEL, 0, 1, 64'h000A, 0, 0, 1, 64'h000A);
    ok(OP_TEL, 0, 1, 64'h0, 0, 0, 1, 0);
    ok(OP_TEL, 0, 1, 64'h03E8, 0, 0, 1, 64'h3E8);
    er(OP_SMART, 0, 1, 64'h32, 0, 0, 8'h03);
    ok(OP_SMART, 0, 1, 64'h64, 0, 0, 1, 64'h64);
    ok(OP_SMART, 0, 1, 64'h0, 0, 0, 1, 0);
    ok(OP_MODE, 1, 1, 0, 0, 0, 1, 1);
    ok(OP_DDRSZ, 0, 1, 64'h20, 0, 0, 1, 64'h20);
    er(OP_DDRSZ, 0, 1, 64'h0, 0, 0, 8'h03);
    er(OP_DDRSZ, 0, 1, 64'h401, 0, 0, 8'h03);
    ok(OP_LEN, 0, 1, 64'h10, 0, 0, 1, 64'h10);
    er(OP_LEN, 0, 1, 64'h1_0000_0000, 0, 0, 8'h03);
    ok(OP_SID, 0, 1, 64'h100, 0, 0, 1, 64'h100);
    ok(OP_RATE, 0, 1, 64'h2CCCCCCC, 0, 0, 1, 64'h2CCCCCCC);
    chk(n_stepl == 1 && last_step == 32'h2CCCCCCC, "RATE in IDLE -> step_load + step");
    ok(OP_PROF_REP, 0, 1, 64'h4, 0, 0, 1, 4);
    ok(OP_PROF_END, 1, 0, 0, 0, 0, 1, 1);                  // STOP
    ok(OP_PROF_END, 0, 0, 0, 0, 0, 1, 0);                  // HOLD
    cmd(OP_CFGQ, 0, 0, 0, 0, 0); get_rep(m);
    chk(m.k == MK_CFGQ, "CFG? kind");
    chk(m.s.cfg_bp === 1 && m.s.cfg_stp === 32'h2CCCCCCC && m.s.cfg_len === 32'h10 && m.s.cfg_sid === 32'h100 &&
        m.s.cfg_ddr === 16'h20 && m.s.cfg_tel === 16'h3E8 && m.s.cfg_sm === 16'h0 && m.s.cfg_pn === 0 &&
        m.s.cfg_pr === 16'h4 && m.s.cfg_pe === 1'b0 && m.s.md === 0, "CFG? snap fields");

    // profile table
    er(OP_PROF_ADD, 0, 2, 64'h2CCCCCCC, 64'h0, 0, 8'h03);               // dur = 0
    ok(OP_PROF_ADD, 0, 2, 64'h2CCCCCCC, 64'h00B2D05E00, 0, 1, 0);
    ok(OP_PROF_ADD, 0, 2, 64'h0, 64'h0002FAF080, 0, 1, 1);
    ok(OP_PROF_ADD, 0, 2, 64'h11111111, 64'h22, 0, 1, 2);
    ok(OP_PROF_SET, 0, 3, 64'h1, 64'h0000ABCD, 64'h0000000042, 1, 1);
    er(OP_PROF_SET, 0, 3, 64'h3, 64'h0, 64'h1, 8'h03);                    // i >= pn
    chk(pram[0] === {32'h2CCCCCCC, 40'h00B2D05E00} && pram[1] === {32'h0000ABCD, 40'h42} &&
        pram[2] === {32'h11111111, 40'h22}, "profile RAM contents");
    cmd(OP_PROFQ, 0, 0, 0, 0, 0);
    for (int i = 0; i < 3; i++) begin
      get_rep(m);
      chk(m.k == MK_PROF_LINE && m.a0[63:56] == i && m.a0[31:0] === pram[i][71:40] && m.a1[39:0] === pram[i][39:0],
          $sformatf("PROF? line %0d", i));
    end
    get_rep(m); chk(m.k == MK_OK && m.c == OP_PROFQ, "PROF? closing OK");
    ok(OP_PROF_CLR, 0, 0, 0, 0, 0, 1, 0);
    cmd(OP_PROFQ, 0, 0, 0, 0, 0); get_rep(m); chk(m.k == MK_OK && m.c == OP_PROFQ, "PROF? empty");
    for (int i = 0; i < 64; i++) ok(OP_PROF_ADD, 0, 2, i, 64'd1 + i, 0, 1, i);
    er(OP_PROF_ADD, 0, 2, 64'h1, 64'h1, 0, 8'h09);                         // 65th
    ok(OP_PROF_CLR, 0, 0, 0, 0, 0, 1, 0);
    er(OP_REC_PROF, 0, 0, 0, 0, 0, 8'h08);                                 // empty profile
    // not calibrated: REC / READ / MEMTEST / DTEST -> 05
    m_calib = 0; wait_cyc(10);
    er0(OP_REC, 8'h05); er0(OP_READ_LAST, 8'h05); er0(OP_READ_END, 8'h05);
    er(OP_READ_N, 0, 1, 64'd3, 0, 0, 8'h05);
    er0(OP_MEMTEST, 8'h05); er0(OP_DTEST, 8'h05);
    ui_age_ok = 0; m_calib = 1; wait_cyc(10);                              // stale ui snapshot is not trusted either
    er0(OP_REC, 8'h05);
    ui_age_ok = 1; wait_cyc(10);
    er(OP_READ_N, 0, 1, 64'd0, 0, 0, 8'h03);
    er(OP_MEMTEST, 0, 1, 64'h0, 0, 0, 8'h03);
    er(OP_MEMTEST, 0, 1, 64'h401, 0, 0, 8'h03);
    er(OP_READ_SID, 0, 1, 64'h1_0000_0000, 0, 0, 8'h03);
    ok(OP_MODE, 0, 1, 0, 0, 0, 1, 0);                                       // RT
    ok(OP_DDRSZ, 0, 1, 64'h400, 0, 0, 0, 0);
    ok(OP_LEN, 0, 1, 64'h0, 0, 0, 0, 0);
    ok(OP_RATE, 0, 1, 64'h10000000, 0, 0, 0, 0);
    ok(OP_PROF_REP, 0, 1, 64'h1, 0, 0, 0, 0);
    flush_msgs(); hist_s = "";

    // ------------------------------------------------------------ T2: normal REC session
    $display("T2 normal REC");
    zero_cnt();
    cmd(OP_REC, 0, 0, 0, 0, 0); get_rep(m);
    chk(m.k == MK_OK && m.c == OP_REC && m.a0 == 64'h100, "OK REC sid=0x100 (SID set earlier)");
    wait_cyc(4);
    chk(int'(dut.st_q) == 1 && int'(dut.md_q) == 1, "BRINGUP/md REC");
    chk(ctrl_start === 1 && ctrl_stop === 0 && verify_mode === 0 && format_req === 0, "BRINGUP outputs");
    chk(n_clear == 1 && n_stepl == 1 && last_step === 32'h10000000, "BRINGUP: ui_clear + step_load");
    chk(n_startl == 0, "no start_load before RUN");
    wait_st(2, 3000, "RUN");
    wait_cyc(60);
    chk(n_startl == 1, "exactly one start_load in RUN");
    chk(last_start_cfg === {40'h1FFE0, 1'b0}, $sformatf("start_cfg %0h (cap 4095 slots)", last_start_cfg));
    chk(cfg.sid === 32'h100 && cfg.ddr_mib === 11'h400 && cfg.lbads === 4'd9, "cfg to bridge");
    tr_v = s_cs_run;
    wait_cyc(500);
    m_drop = 3;                                             // visible in !RR
    m_wr = 48'h40; m_beats = 64'h4000;                      // static fill before STOP
    ok0(OP_STOP);
    wait_st(3, 20, "SRC_STOP");
    wait_cyc(8);
    chk(n_stop == 1 && n_pf == 0, "src_stop pulse once, no src_pf");
    wait_st(4, 5000, "CORE_STOP");
    chk(src_done === 1 && m_beats == {m_wr, 8'h0}, "ctrl_stop only after drain complete");
    chk(ctrl_stop === 1 && ctrl_start === 1, "CORE_STOP: ctrl_stop=1");
    wait_st(5, 5000, "RELEASE");
    chk(ctrl_start === 0, "RELEASE: ctrl_start=0");
    wait_st(0, 5000, "IDLE");
    chk(ctrl_stop === 0 && ctrl_start === 0, "IDLE: ctrl_stop=0");
    get_ev(MK_EV_RR, m);
    chk(m.s.sid === 32'h100 && m.s.wr === 48'h60 && m.s.drp === 32'd3 && m.s.pad_blocks === 16'd31 && m.s.trunc_blocks === 16'd1 &&
        m.s.ddrm === 32'h5000 && m.s.ssd === 64'h60 * 4096 && m.s.cmd === 32'h1234 && m.s.cer === 0 && m.s.lat === 32'h777,
        "!RR fields");
    td_v = s_cs_done + 1; tr_v = tr_v - 1;
    chk(m.s.dur >= td_v - tr_v - 3 && m.s.dur <= td_v - tr_v + 3, $sformatf("!RR dur %0d expected ~%0d", m.s.dur, td_v - tr_v));
    chk_hist("1 2 3 4 5 0 ");
    // RES? repeats the result
    cmd(OP_RESQ, 0, 0, 0, 0, 0);
    get_rep(m); chk(m.k == MK_OK && m.c == OP_RESQ, "RES? OK");
    get_ev(MK_EV_RR, m2); chk(m2.s.sid === 32'h100 && m2.s.wr === 48'h60, "RES? repeats !RR");
    // sid counter: auto +1; explicit SID used once
    ok(OP_REC, 0, 0, 0, 0, 0, 1, 64'h101);
    wait_st(2, 3000, "RUN#2"); ok0(OP_STOP); wait_idle();
    ok(OP_SID, 0, 1, 64'h692D3A00, 0, 0, 0, 0);
    ok(OP_REC, 0, 0, 0, 0, 0, 1, 64'h692D3A00);
    wait_st(2, 3000, "RUN#3"); ok0(OP_STOP); wait_idle();
    ok(OP_REC, 0, 0, 0, 0, 0, 1, 64'h692D3A01);
    wait_st(2, 3000, "RUN#4");
    chk(cfg.sid === 32'h692D3A01, "cfg.sid of running session");
    ok0(OP_STOP); wait_idle();
    cleanup();

    // ------------------------------------------------------------ T3: len / profile end / capacity
    $display("T3 len, profile end, capacity");
    ok(OP_LEN, 0, 1, 64'h10, 0, 0, 0, 0);
    rec_to_run(0); wait_cyc(60);
    chk(last_start_cfg === {40'h200, 1'b0}, "LEN 0x10 slots -> len_blocks 0x200");
    wait_cyc(100);
    hold_st(2, 50, "RUN until len_reached");
    m_len_reached = 1;
    wait_st(3, 30, "SRC_STOP by len_reached");
    wait_idle(); cleanup();
    ok(OP_LEN, 0, 1, 64'h10000, 0, 0, 0, 0);                // above the capacity
    rec_to_run(0); wait_cyc(60);
    chk(last_start_cfg === {40'h1FFE0, 1'b0}, "LEN above capacity is capped (cap-1 slot)");
    ok0(OP_STOP); wait_idle(); cleanup();
    ok(OP_LEN, 0, 1, 64'h0, 0, 0, 0, 0);
    ok(OP_PROF_ADD, 0, 2, 64'h2CCCCCCC, 64'h1000, 0, 0, 0);
    rec_to_run(1); wait_cyc(60);
    chk(last_start_cfg[0] === 1'b1, "REC PROF -> prof_en=1");
    er(OP_RATE, 0, 1, 64'h1, 0, 0, 8'h04);                  // RATE with a profile in RUN
    m_prof_end = 1;
    wait_st(3, 30, "SRC_STOP by prof_end");
    wait_idle(); cleanup();
    ok(OP_PROF_CLR, 0, 0, 0, 0, 0, 0, 0);
    rec_to_run(0);                                          // plain REC: RATE allowed in RUN
    zero_cnt();
    ok(OP_RATE, 0, 1, 64'h12345678, 0, 0, 1, 64'h12345678);
    wait_cyc(4); chk(n_stepl == 1 && last_step === 32'h12345678, "RATE in RUN -> step_load");
    ok0(OP_STOP); wait_idle(); cleanup();
    ok(OP_RATE, 0, 1, 64'h10000000, 0, 0, 0, 0);

    // ------------------------------------------------------------ T4: power fail in RUN (all phases)
    $display("T4 PF in RUN");
    rec_to_run(0);
    wait_cyc(200);
    m_wr = 48'h40; m_beats = 64'd61 * 256; m_ssd_lag = 64'h20000; m_log_busy = 1;
    wait_cyc(10);
    zero_cnt(); hist_s = "";
    cmd(OP_PFAIL, 0, 0, 0, 0, 0); get_rep(m); chk(m.k == MK_OK && m.c == OP_PFAIL, "OK PFAIL");
    t0_v = dut.t0_q;
    wait_cyc(6);
    chk(int'(dut.md_q) == 2, "md=PF");
    chk(n_pf == 1 && n_stop == 0, "src_pf (not src_stop)");
    chk(int'(dut.st_q) == 3, "PF in RUN -> SRC_STOP");
    wait_idle();
    get_ev(MK_EV_RP, m);
    exp_ssd = 64'd61 * 256 * 16 - 64'h20000;
    exp_pnd = 64'h40 * 4096 - exp_ssd;
    exp_prg = 64'h20000;
    chk(m.s.rp_src === 0 && m.s.rp_inb === 0 && m.s.rp_lat === 0 && m.s.rp_lov === 1, "!RP flags src/inb/lat/lov");
    chk(m.s.rp_pnd === exp_pnd[47:0], $sformatf("!RP pnd %0d expected %0d", m.s.rp_pnd, exp_pnd));
    chk(m.s.rp_pdd === 32'd12288, $sformatf("!RP pdd %0d", m.s.rp_pdd));
    chk(m.s.rp_prg === exp_prg[31:0], $sformatf("!RP prg %0d expected %0d", m.s.rp_prg, exp_prg));
    chk(m.s.rp_pad === 32'd32, $sformatf("!RP pad %0d", m.s.rp_pad));
    chk(m.s.rp_ok === 1 && m.s.rp_lpos === 40'd95, $sformatf("!RP ok/lpos %0d", m.s.rp_lpos));
    chk(m.s.ssd === 64'h60 * 4096, "!RP ssd");
    exp_res_times(m, t0_v, "PF in RUN");
    chk(m.s.rp_t1 > 0 && m.s.rp_t2 > m.s.rp_t1 && m.s.rp_t3 > m.s.rp_t2 && m.s.rp_t4 > m.s.rp_t3 &&
        m.s.rp_t5 > m.s.rp_t4 && m.s.rp_t6 > m.s.rp_t5, "all PF phases take time and are ordered");
    chk_hist("3 4 5 0 ");
    // the same through KEY3: src=1; log_busy = 0 -> lov = 0
    cleanup();
    rec_to_run(0); wait_cyc(100);
    pulse_key(2);
    wait_st(3, 20, "SRC_STOP (KEY3)");
    chk(int'(dut.md_q) == 2, "KEY3 -> md PF");
    wait_idle();
    get_ev(MK_EV_RP, m);
    chk(m.s.rp_src === 1 && m.s.rp_lov === 0 && m.s.rp_lat === 0 && m.s.rp_inb === 0, "!RP src=1 (KEY3)");
    cleanup();

    // ------------------------------------------------------------ T5: PF in BRINGUP, PF after STOP
    $display("T5 PF in BRINGUP / late PF");
    k_bring = 1_000_000;
    zero_cnt();
    ok0(OP_REC);
    wait_st(1, 100, "BRINGUP");
    wait_cyc(50);
    ok0(OP_PFAIL);
    t0_v = dut.t0_q;
    wait_cyc(4);
    chk(int'(dut.st_q) == 4 && ctrl_stop === 1 && n_pf == 0 && n_stop == 0, "PF in BRINGUP -> CORE_STOP, no source pulses");
    wait_idle();
    get_ev(MK_EV_RP, m);
    chk(m.s.rp_inb === 1 && m.s.rp_lat === 0, "!RP inb=1");
    chk(m.s.rp_t1 === 0 && m.s.rp_t2 === 0 && m.s.rp_t3 === 0, "!RP t1..t3 = 0 in BRINGUP");
    chk(m.s.rp_t4 > 0 && m.s.rp_t4 <= m.s.rp_t5 && m.s.rp_t5 <= m.s.rp_t6, "!RP t4..t6 ordered");
    cleanup();
    // PF after STOP, in SRC_STOP (source still draining)
    k_t2 = 3000; k_t3 = 3000;
    rec_to_run(0); wait_cyc(100);
    m_wr = 48'h20; m_beats = 64'h2000;
    ok0(OP_STOP);
    wait_st(3, 20, "SRC_STOP"); wait_cyc(500);
    zero_cnt();
    ok0(OP_PFAIL); t0_v = dut.t0_q;
    wait_cyc(4);
    chk(int'(dut.st_q) == 3 && int'(dut.md_q) == 2 && n_pf == 0, "PF in SRC_STOP: still SRC_STOP, md=PF, no new src pulse");
    er0(OP_PFAIL, 8'h04);                                                  // second PF is refused
    wait_idle();
    get_ev(MK_EV_RP, m);
    chk(m.s.rp_lat === 1 && m.s.rp_inb === 0, "!RP lat=1 (PF after STOP)");
    chk(m.s.rp_t1 === 0, "t1 already passed before PF -> 0");
    chk(m.s.rp_t2 > 0 && m.s.rp_t3 >= m.s.rp_t2, "t2/t3 measured from PF");
    exp_res_times(m, t0_v, "late PF in SRC_STOP");
    cleanup();
    // PF in CORE_STOP
    k_shut = 4000;
    rec_to_run(0); wait_cyc(100);
    ok0(OP_STOP); wait_st(4, 5000, "CORE_STOP"); wait_cyc(300);
    ok0(OP_PFAIL); t0_v = dut.t0_q;
    wait_cyc(4);
    chk(int'(dut.st_q) == 4 && int'(dut.md_q) == 2, "PF in CORE_STOP");
    wait_idle();
    get_ev(MK_EV_RP, m);
    chk(m.s.rp_lat === 1 && m.s.rp_t1 === 0 && m.s.rp_t2 === 0 && m.s.rp_t3 === 0, "late PF in CORE_STOP: t1..t3 = 0");
    chk(m.s.rp_t6 > m.s.rp_t5 && m.s.rp_t5 >= m.s.rp_t4, "t4..t6 from PF");
    exp_res_times(m, t0_v, "late PF in CORE_STOP");
    cleanup();

    // ------------------------------------------------------------ T6: errored sessions -> CORE_RST
    $display("T6 errored sessions");
    // (a) core stops on its own in RUN
    zero_cnt();
    rec_to_run(0); wait_cyc(100);
    m_ce = 8'h12; k_force_cs = 4'd7; wait_cyc(3); k_force_cs = 0;
    wait_st(4, 40, "CORE_STOP by core FLUSH");
    wait_cyc(8);
    chk(n_stop == 1 && ctrl_stop === 1, "self-stop in RUN: src_stop + ctrl_stop");
    wait_st(5, 5000, "RELEASE"); wait_st(7 + 1, 5000, "CORE_RST after errored session");
    wait_st(0, 6000, "IDLE after CORE_RST");
    chk(n_clear >= 2, "CORE_RST: ui_clear");
    chk(n_crst_edges == 1 && crst_w_last == 200, $sformatf("core_rst_req width %0d (1 ms = 200 clk)", crst_w_last));
    chk(ctrl_start === 0 && ctrl_stop === 0, "idle outputs after CORE_RST");
    get_ev(MK_EV_E, m);
    chk(m.a0[7:0] === 8'h12, "!E ce");
    chk_hist("1 2 4 5 8 0 ");
    // second session works after the reset
    zero_cnt(); m_ce = 0;
    rec_to_run(0); wait_cyc(60);
    chk(n_clear == 1 && n_startl == 1, "next session starts normally");
    ok0(OP_STOP); wait_idle(); cleanup();
    // (b) core leaves RUN during SRC_STOP
    k_t2 = 5000;
    rec_to_run(0); wait_cyc(100); ok0(OP_STOP); wait_st(3, 20, "SRC_STOP");
    wait_cyc(30);
    zero_cnt();
    k_force_cs = 4'd8; wait_cyc(3); k_force_cs = 0;
    wait_st(4, 40, "CORE_STOP by core SHUTDOWN (from SRC_STOP)");
    wait_cyc(6);
    chk(n_stop == 0 && ctrl_stop === 1, "no second src_stop");
    wait_st(8, 8000, "CORE_RST");
    wait_st(0, 6000, "IDLE"); cleanup();
    // (c) clean DONE but ce != 0 / flags io_timeout
    rec_to_run(0); wait_cyc(100);
    ok0(OP_STOP);
    wait_st(4, 5000, "CORE_STOP");
    m_ce = 8'h05; wait_st(5, 5000, "RELEASE"); wait_st(8, 3000, "CORE_RST (ce != 0)");
    wait_st(0, 6000, "IDLE"); cleanup();
    rec_to_run(0); wait_cyc(100);
    ok0(OP_STOP); wait_st(4, 5000, "CORE_STOP"); m_fl = 8'h04;
    wait_st(8, 8000, "CORE_RST (io_timeout)");
    wait_st(0, 6000, "IDLE"); cleanup();
    // (d) flags that are not session errors (rx_drop bit 7) do not force a core reset
    rec_to_run(0); wait_cyc(100);
    ok0(OP_STOP); wait_st(4, 5000, "CORE_STOP"); m_fl = 8'h80;
    wait_st(5, 5000, "RELEASE");
    wait_st(0, 3000, "IDLE directly (no CORE_RST)"); cleanup();

    // ------------------------------------------------------------ T7: ERROR drops ctrl_start / ctrl_stop
    $display("T7 ERROR");
    for (int where = 0; where < 4; where++) begin
      knobs_default(); zero_cnt(); flush_msgs(); hist_s = "";
      if (where == 0) k_bring = 1_000_000;
      if (where == 2) k_t2 = 100000;
      if (where == 3) k_shut = 100000;
      ok0(OP_REC);
      if (where == 0) wait_st(1, 100, "BRINGUP");
      else wait_st(2, 3000, "RUN");
      if (where >= 2) begin
        wait_cyc(50); ok0(OP_STOP);
        wait_st(where == 2 ? 3 : 4, 5000, "stopped state");
      end
      wait_cyc(60);
      m_cd = 8'h33;
      if (where == 1) begin                               // stale core snapshot: ERROR is not acted upon
        core_age_ok = 0; m_ce = 8'h77; k_force_cs = 4'hF; wait_cyc(40);
        chk(int'(dut.st_q) == 2 && ctrl_start === 1, "ERROR with a stale snapshot is ignored");
        core_age_ok = 1;
      end else begin m_ce = 8'h77; k_force_cs = 4'hF; end
      wait_cyc(4); k_force_cs = 0;
      wait_st(8, 20, "CORE_RST after ERROR");
      chk(ctrl_start === 0 && ctrl_stop === 0, "ERROR: ctrl_start and ctrl_stop dropped");
      chk(n_stop >= 1, "ERROR: src_stop");
      wait_st(0, 6000, "IDLE after ERROR");
      chk(n_crst_edges == 1 && crst_w_last == 200, "ERROR: one 1 ms core reset");
      get_ev(MK_EV_E, m);
      chk(m.a0[7:0] === 8'h77 && m.a0[15:8] === 8'h33 && m.a0[27:24] === ((where == 0) ? 4'd1 : (where == 1) ? 4'd2 : (where == 2) ? 4'd3 : 4'd4),
          $sformatf("!E fields %0h (where %0d)", m.a0, where));
      chk(ctrl_start === 0 && ctrl_stop === 0, "idle after ERROR");
      m_ce = 0; m_cd = 0;
      cleanup();
    end
    // a normal session works after the ERROR flows
    rec_to_run(0); ok0(OP_STOP); wait_idle(); cleanup();

    // ------------------------------------------------------------ T8: ABORT in every state
    $display("T8 ABORT");
    for (int where = 0; where < 8; where++) begin
      knobs_default(); zero_cnt(); flush_msgs(); hist_s = "";
      case (where)
        0: ;                                                   // IDLE
        1: begin k_bring = 1_000_000; ok0(OP_REC); wait_st(1, 100, "BRINGUP"); end
        2: begin rec_to_run(0); end
        3: begin k_t2 = 100000; rec_to_run(0); ok0(OP_STOP); wait_st(3, 20, "SRC_STOP"); end
        4: begin k_shut = 100000; rec_to_run(0); ok0(OP_STOP); wait_st(4, 5000, "CORE_STOP"); end
        5: begin k_hold_done = 1; rec_to_run(0); ok0(OP_STOP); wait_st(5, 5000, "RELEASE"); end
        6: begin k_mt = 1_000_000; ok(OP_MEMTEST, 0, 0, 0, 0, 0, 0, 0); wait_st(6, 100, "LOCAL"); end
        7: begin k_hang_rst = 1; ok0(OP_ABORT); wait_st(8, 100, "CORE_RST"); wait_cyc(500); end
      endcase
      wait_cyc(30); flush_msgs(); zero_cnt();
      cmd(OP_ABORT, 0, 0, 0, 0, 0);
      get_rep(m); chk(m.k == MK_OK && m.c == OP_ABORT, "OK ABORT");
      wait_cyc(6);
      chk(ctrl_start === 0 && ctrl_stop === 0 && verify_mode === 0 && format_req === 0, "ABORT: ctrl_* dropped");
      chk(n_stop >= 1, "ABORT: src_stop");
      wait_st(8, 10, "CORE_RST after ABORT");
      chk(int'(dut.md_q) == 0, "ABORT: md cleared");
      k_hang_rst = 0; k_hold_done = 0;
      wait_st(0, 6000, "IDLE after ABORT");
      chk(n_crst_edges == 1, $sformatf("ABORT: core resets %0d (where %0d)", n_crst_edges, where));
      chk(crst_w_last == 200, "ABORT: reset width 1 ms");
      cleanup();
    end
    rec_to_run(0); ok0(OP_STOP); wait_idle(); cleanup();           // next REC works after ABORT

    // ------------------------------------------------------------ T9: CORE_RST timeout (2 s) -> !E and repeat
    $display("T9 CORE_RST timeout");
    knobs_default(); zero_cnt(); flush_msgs(); hist_s = "";
    k_hang_rst = 1;
    ok0(OP_ABORT);
    wait_st(8, 100, "CORE_RST");
    wait_cyc(398_000);
    chk(int'(dut.st_q) == 8 && n_crst_edges == 1, "CORE_RST waits for the core (< 2 s)");
    no_ev(MK_EV_E, "!E before the 2 s timeout");
    wait_cyc(3_000);
    get_ev(MK_EV_E, m);
    chk(m.a0[27:24] === 4'd8, "!E from CORE_RST timeout");
    wait_cyc(400);
    chk(n_crst_edges == 2 && int'(dut.st_q) == 8, "reset repeated after the timeout");
    k_hang_rst = 0;
    wait_st(0, 700_000, "IDLE once the core answers");
    cleanup();

    // ------------------------------------------------------------ T10: READ / MON / FORMAT
    $display("T10 READ MON FORMAT");
    rec_to_run(0); wait_cyc(60); m_wr = 48'h40; m_beats = 64'h4000; ok0(OP_STOP); wait_idle(); cleanup();     // last record: wr = 0x60
    zero_cnt();
    ok0(OP_READ_LAST);
    wait_cyc(6);
    chk(int'(dut.md_q) == 3 && verify_mode === 1 && verify_cmds === 32'd3 && ctrl_start === 1, $sformatf("READ LAST: verify_cmds=%0d md=%0d vm=%b last_wr=%0h", verify_cmds, dut.md_q, verify_mode, dut.last_wr_q));
    wait_st(2, 3000, "RUN (READ)");
    wait_cyc(300);
    chk(n_startl == 0 && n_stop == 0 && n_pf == 0, "READ: no source activity");
    chk(cfg.rsid === dut.last_sid_q, "cfg.rsid = SID of the last recording");
    m_cmd_limit = 1;
    wait_st(4, 40, "CORE_STOP by cmd_limit_done");
    wait_st(0, 5000, "IDLE");
    chk(verify_mode === 0, "verify_mode released");
    get_ev(MK_EV_RV, m);
    chk(m.s.vrf.vb === 48'h0123 && m.s.vrf.ok === 40'h77 && m.s.md === 3, "!RV snapshot");
    chk_hist("1 2 4 5 0 ");       // D24: no CORE_RST after a READ session
    cleanup();
    // READ N, READ END (two capacity variants), READ SID, stop / PF in RUN
    ok(OP_READ_N, 0, 1, 64'd5, 0, 0, 1, 5);
    wait_cyc(6); chk(verify_cmds === 32'd5, "READ N: verify_cmds");
    wait_st(2, 3000, "RUN"); ok0(OP_STOP); wait_idle(); cleanup();
    ok(OP_READ_SID, 0, 1, 64'hABCD, 0, 0, 1, 64'hABCD);
    ok0(OP_READ_END);
    wait_st(2, 3000, "RUN"); wait_cyc(20);
    chk(verify_cmds === 32'd4096, $sformatf("READ END verify_cmds %0d (capacity slots)", verify_cmds));
    chk(cfg.rsid === 32'hABCD, "READ SID override in cfg.rsid");
    m_end_found = 1;
    wait_st(4, 40, "CORE_STOP by end_found"); wait_idle();
    chk(dut.rsid_user_q === 1'b0, "READ SID is used once");
    cleanup();
    // race: Identify NS publishes nsze before lbads: snapshots with nsze != 0, lbads = 0 must not start the capacity engine
    m_nsze = 64'h10_0000; m_lbads = 4'd0;
    ok0(OP_READ_END); wait_cyc(400);
    chk(int'(dut.st_q) == 1 && dut.cap_busy_q === 1'b0 && dut.cap_valid_q === 1'b0, "lbads=0 snapshot: capacity engine waits");
    m_lbads = 4'd9; wait_st(2, 3000, "RUN"); wait_cyc(20);
    chk(verify_cmds === 32'd4096, $sformatf("READ END after lbads 0 -> 9: verify_cmds %0d", verify_cmds));
    pulse_key(1); wait_st(4, 40, "CORE_STOP by KEY2"); wait_idle(); cleanup();
    m_nsze = 64'h20000; m_lbads = 4'd12;
    ok0(OP_READ_END); wait_st(2, 3000, "RUN"); wait_cyc(20);
    chk(verify_cmds === 32'd4096, $sformatf("READ END (lbads 12) verify_cmds %0d", verify_cmds));
    pulse_key(1);                                                                    // KEY2 = STOP
    wait_st(4, 40, "CORE_STOP by KEY2"); wait_idle(); cleanup();
    m_nsze = 64'h10_0000; m_lbads = 4'd9;
    ok0(OP_READ_END); wait_st(2, 3000, "RUN");
    zero_cnt();
    ok0(OP_PFAIL); wait_cyc(4);
    chk(int'(dut.st_q) == 4 && int'(dut.md_q) == 3 && n_pf == 0, "PF in RUN (READ) = stop, md stays READ");
    wait_idle(); get_ev(MK_EV_RV, m); no_ev(MK_EV_RP, "!RP for READ"); cleanup();
    // MON works without calibration, no result event
    m_calib = 0; wait_cyc(10);
    zero_cnt(); flush_msgs(); hist_s = "";
    ok0(OP_MON); wait_st(2, 3000, "RUN (MON)"); wait_cyc(50);
    chk(n_startl == 0 && verify_mode === 0 && format_req === 0 && int'(dut.md_q) == 5, "MON session");
    ok0(OP_STOP); wait_idle(); chk_hist("1 2 4 5 0 ");
    no_ev(MK_EV_RR, "!RR for MON"); no_ev(MK_EV_RV, "!RV for MON"); no_ev(MK_EV_RP, "!RP for MON");
    // FORMAT: STOP right after RUN, format_req in the session
    flush_msgs(); hist_s = "";
    ok0(OP_FORMAT);
    wait_cyc(10); chk(format_req === 1 && int'(dut.md_q) == 4, "FORMAT: format_req");
    wait_st(5, 5000, "RELEASE"); wait_idle();
    chk_hist("1 2 4 5 0 ");
    get_ev(MK_EV_RF, m); chk(m.a1 === 64'd1 && m.a0 < 64'd20, $sformatf("!RF ms=%0d ok=%0d", m.a0, m.a1));
    chk(format_req === 0, "format_req released");
    m_calib = 1; cleanup();

    // ------------------------------------------------------------ T11: LOCAL tests
    $display("T11 MEMTEST / DTEST");
    zero_cnt(); flush_msgs(); hist_s = "";
    k_mt = 2000;
    cmd(OP_MEMTEST, 0, 1, 64'h20, 0, 0); get_rep(m);
    chk(m.k == MK_OK && m.c == OP_MEMTEST && m.a0 == 64'h20, "OK MEMTEST");
    wait_cyc(4);
    chk(int'(dut.st_q) == 6 && int'(dut.md_q) == 7 && ctrl_start === 0 && n_clear == 1, "MEMTEST: LOCAL, ui_clear, no core session");
    wait_cyc(40);
    chk(cfg.mt_mib === 11'h20, "cfg.mt_mib");
    wait_st(0, 6000, "IDLE after MEMTEST");
    chk(n_mt == 1 && s_mt - s_clear >= 1024, $sformatf("mt_start %0d clocks after ui_clear", s_mt - s_clear));
    get_ev(MK_EV_RM, m);
    chk(m.s.mt_bytes === 32'h0200_0000 && m.s.mte === 32'd3 && m.s.mta === 32'h1234 && m.s.dur > 2000, "!RM fields");
    chk_hist("6 0 ");
    // default size = DDRSZ; STOP before the test starts / during the test
    ok0(OP_MEMTEST); wait_st(6, 100, "LOCAL"); wait_cyc(100); n_prev = n_mt;
    ok0(OP_STOP); wait_cyc(10);
    chk(int'(dut.st_q) == 0 && n_mt == n_prev, "MEMTEST: STOP before start");
    get_ev(MK_EV_RM, m); chk(m.s.mt_bytes === 32'h4000_0000, "default MEMTEST size = DDRSZ");
    k_mt = 1_000_000;
    ok0(OP_MEMTEST); wait_st(6, 100, "LOCAL"); wait_cyc(2000);
    chk(n_mt == n_prev + 1, "mt_start issued"); n_prev = n_clear;
    ok0(OP_STOP); wait_cyc(10);
    chk(int'(dut.st_q) == 0 && n_clear == n_prev + 1, "MEMTEST: STOP during the test aborts it with ui_clear");
    get_ev(MK_EV_RM, m);
    cleanup();
    // DTEST
    zero_cnt(); flush_msgs(); hist_s = "";
    ok(OP_RATE, 0, 1, 64'h2CCCCCCC, 0, 0, 0, 0);
    zero_cnt();
    ok0(OP_DTEST); wait_cyc(4);
    chk(int'(dut.st_q) == 6 && int'(dut.md_q) == 6 && n_clear == 1 && n_stepl == 1 && last_step === 32'h2CCCCCCC, "DTEST entry");
    wait_cyc(1500);
    chk(n_dt == 1 && s_dt - s_clear >= 1024, "dtest_start after the clear delay");
    chk(n_startl == 1 && last_start_cfg === 41'd0, "D1: DTEST also issues start_load with start_cfg=0");
    ok(OP_RATE, 0, 1, 64'h1111, 0, 0, 1, 64'h1111);
    wait_cyc(4); chk(n_stepl == 2 && last_step === 32'h1111, "RATE during DTEST");
    m_wr = 48'h100; m_drop = 7;
    ok0(OP_STOP); wait_cyc(90);
    chk(n_stop == 1, "DTEST: STOP -> src_stop (after the start settle time)");
    wait_st(0, 3000, "IDLE after DTEST");
    get_ev(MK_EV_RD, m);
    chk(m.s.dt_he === 32'd5 && m.s.dt_wr === 48'h120 && m.s.drp === 32'd7, "!RD fields");
    cleanup();
    zero_cnt(); ok0(OP_DTEST); wait_st(6, 100, "LOCAL"); wait_cyc(100); ok0(OP_STOP); wait_cyc(10);
    chk(int'(dut.st_q) == 0 && n_dt == 0 && n_stop == 0 && n_startl == 0, "DTEST: STOP before start does nothing to the source");
    cleanup();

    // ------------------------------------------------------------ T12: log_req scheduling, TEL tick
    $display("T12 log_req / tel_tick");
    ok(OP_SMART, 0, 1, 64'h64, 0, 0, 0, 0);
    zero_cnt();
    wait_cyc(30000);
    chk(n_log == 0, "no log_req in IDLE");
    k_bring = 5000; k_shut = 100000;
    ok0(OP_REC); wait_cyc(3000);
    chk(n_log == 0, "no log_req in BRINGUP");
    wait_st(2, 6000, "RUN"); n_prev = n_log;
    wait_cyc(65_000);
    chk(n_log >= 2 && n_log <= 4, $sformatf("log_req period 100 ms: %0d in 65000 clk", n_log));
    n_prev = n_log; m_log_busy = 1; wait_cyc(45_000);
    chk(n_log == n_prev, "no log_req while the core log is busy");
    m_log_busy = 0;
    while (dut.sm_cnt_q != 16'd97) @(posedge clk);                        // just before the next log_req tick
    core_age_ok = 0; wait_cyc(1500);                                    // (a stale status is an ERROR after 20 ms, review fix C)
    chk(n_log == n_prev, "no log_req with a stale core snapshot");
    core_age_ok = 1; wait_cyc(45_000);
    chk(n_log > n_prev, "log_req resumes"); n_prev = n_log;
    ok0(OP_STOP); wait_st(4, 5000, "CORE_STOP");
    wait_cyc(60_000);
    chk(n_log == n_prev, "no log_req after STOP");
    ok(OP_SMART, 0, 1, 64'h0, 0, 0, 0, 0);
    cleanup();
    k_bring = 100;
    ok(OP_TEL, 0, 1, 64'h0A, 0, 0, 0, 0); zero_cnt(); wait_cyc(20_000);
    chk(n_tel >= 9 && n_tel <= 11, $sformatf("tel_tick count %0d", n_tel));
    ok(OP_TEL, 0, 1, 64'h0, 0, 0, 0, 0); zero_cnt(); wait_cyc(6_000);
    chk(n_tel == 0, "TEL 0 = off");
    ok(OP_TEL, 0, 1, 64'h3E8, 0, 0, 0, 0);

    // ------------------------------------------------------------ T13: stale snapshots -> no decisions
    $display("T13 stale snapshots");
    knobs_default();
    core_age_ok = 0; wait_cyc(5);
    ok0(OP_REC); wait_cyc(2500);
    chk(int'(dut.st_q) == 1, "BRINGUP: cs=RUN with a stale snapshot is not acted upon");
    core_age_ok = 1; wait_st(2, 30, "RUN once fresh"); wait_cyc(60);
    // SRC_STOP -> CORE_STOP needs fresh ui and core snapshots
    ui_age_ok = 0; ok0(OP_STOP); wait_cyc(1500);      // < 10 ms (review fix D)
    chk(int'(dut.st_q) == 3 && ctrl_stop === 0 && ui_stat.src_done === 1 && ui_stat.ddr_idle === 1, "SRC_STOP: stale ui snapshot");
    ui_age_ok = 1; core_age_ok = 0; wait_cyc(1500);
    chk(int'(dut.st_q) == 3 && ctrl_stop === 0, "SRC_STOP: stale core snapshot");
    core_age_ok = 1;
    wait_st(4, 40, "CORE_STOP once fresh");
    core_age_ok = 0; wait_cyc(2000);
    chk(int'(dut.st_q) == 4 && core_stat.seq_state == 9, "CORE_STOP: DONE with a stale snapshot is not acted upon");
    core_age_ok = 1; wait_st(5, 40, "RELEASE");
    core_age_ok = 0; wait_cyc(800);
    chk(int'(dut.st_q) == 5 && core_stat.seq_state == 0, "RELEASE: IDLE with a stale snapshot is not acted upon");
    core_age_ok = 1; wait_st(0, 40, "IDLE"); cleanup();
    // READ: cmd_limit_done / end_found are decisions too
    ok0(OP_READ_LAST);
    wait_st(2, 3000, "RUN (READ)"); core_age_ok = 0; m_cmd_limit = 1; wait_cyc(1000);
    chk(int'(dut.st_q) == 2, "READ: cmd_limit_done ignored when stale");
    core_age_ok = 1; wait_st(4, 40, "CORE_STOP"); wait_idle(); cleanup();

    $display("T14 refused commands in every state");
    // each non-IDLE state: every IDLE-only command is refused with 04 (06 in CORE_RST) and changes nothing
    // IDLE: STOP / PFAIL refused
    er0(OP_STOP, 8'h04); er0(OP_PFAIL, 8'h04);
    // BRINGUP
    k_bring = 1_000_000; ok0(OP_REC); wait_st(1, 100, "BRINGUP");
    refuse_all(1, 8'h04); er(OP_RATE, 0, 1, 64'h5, 0, 0, 8'h04);
    hist_s = ""; zero_cnt();
    ok0(OP_STOP);                                                     // STOP in BRINGUP is valid
    wait_cyc(4); chk(int'(dut.st_q) == 4 && ctrl_stop === 1 && n_stop == 0, "STOP in BRINGUP -> CORE_STOP");
    wait_idle(); get_ev(MK_EV_RR, m); chk(m.s.wr === 0 && m.s.ssd === 0, "!RR of a stopped BRINGUP is empty");
    chk_hist("4 5 0 "); cleanup();
    // RUN
    rec_to_run(0); refuse_all(2, 8'h04);
    ok0(OP_STOP); wait_idle(); cleanup();
    // SRC_STOP
    k_t2 = 100000; rec_to_run(0); ok0(OP_STOP); wait_st(3, 20, "SRC_STOP");
    refuse_all(3, 8'h04); er0(OP_STOP, 8'h04); er(OP_RATE, 0, 1, 64'h5, 0, 0, 8'h04);
    cleanup();
    // CORE_STOP
    k_shut = 100000; rec_to_run(0); ok0(OP_STOP); wait_st(4, 5000, "CORE_STOP");
    refuse_all(4, 8'h04); er0(OP_STOP, 8'h04); er(OP_RATE, 0, 1, 64'h5, 0, 0, 8'h04);
    cleanup();
    // RELEASE
    k_hold_done = 1; rec_to_run(0); ok0(OP_STOP); wait_st(5, 5000, "RELEASE");
    refuse_all(5, 8'h04); er0(OP_STOP, 8'h04); er0(OP_PFAIL, 8'h04); er(OP_RATE, 0, 1, 64'h5, 0, 0, 8'h04);
    cleanup();
    // LOCAL (MEMTEST): PFAIL / RATE refused, STOP valid
    k_mt = 1_000_000; ok0(OP_MEMTEST); wait_st(6, 100, "LOCAL");
    refuse_all(6, 8'h04); er0(OP_PFAIL, 8'h04); er(OP_RATE, 0, 1, 64'h5, 0, 0, 8'h04);
    ok0(OP_STOP); wait_cyc(10); chk(int'(dut.st_q) == 0, "STOP in LOCAL");
    cleanup();
    // CORE_RST: everything is refused with 06
    k_hang_rst = 1; ok0(OP_ABORT); wait_st(8, 100, "CORE_RST");
    refuse_all(8, 8'h06); er0(OP_STOP, 8'h06); er0(OP_PFAIL, 8'h06); er(OP_RATE, 0, 1, 64'h5, 0, 0, 8'h06);
    ok0(OP_PING);                                                      // queries still work
    cmd(OP_CFGQ, 0, 0, 0, 0, 0); get_rep(m); chk(m.k == MK_CFGQ && m.s.st === 8, "CFG? during CORE_RST");
    k_hang_rst = 0; wait_st(0, 700_000, "IDLE"); cleanup();

    // ------------------------------------------------------------ T15: keys / VIO / LED
    $display("T15 keys");
    flush_msgs(); zero_cnt(); hist_s = "";
    m_calib = 0; wait_cyc(10);
    pulse_key(0); wait_cyc(10);
    chk(int'(dut.st_q) == 0 && rep_q.size() == 0, "KEY1 without calibration: ignored, no reply");
    m_calib = 1; wait_cyc(10);
    pulse_key(1); pulse_key(2); wait_cyc(10);
    chk(int'(dut.st_q) == 0 && rep_q.size() == 0, "KEY2/KEY3 in IDLE ignored");
    pulse_key(0);                                                       // KEY1 = REC (no profile)
    wait_st(2, 3000, "RUN by KEY1");
    chk(rep_q.size() == 0, "keys produce no reply");
    wait_cyc(50); chk(last_start_cfg[0] === 1'b0, "KEY1: plain REC");
    pulse_key(0); wait_cyc(10); chk(int'(dut.st_q) == 2 && n_clear == 1, "KEY1 in RUN ignored");
    k_t2 = 5000;
    pulse_key(1); wait_cyc(10);                                         // KEY2 = STOP
    chk(int'(dut.st_q) == 3, "KEY2 -> SRC_STOP");
    pulse_key(1); wait_cyc(10);
    chk(int'(dut.st_q) == 3 && rep_q.size() == 0, "KEY2 in SRC_STOP ignored silently");
    wait_idle(); cleanup();
    ok(OP_PROF_ADD, 0, 2, 64'h1000, 64'h100, 0, 0, 0);
    pulse_key(3); wait_st(2, 3000, "RUN by VIO");
    wait_cyc(50); chk(last_start_cfg[0] === 1'b1, "vio_start with a profile = REC PROF");
    pulse_key(4); wait_cyc(10); chk(int'(dut.st_q) == 3, "vio_stop -> SRC_STOP");
    wait_idle(); cleanup();
    ok(OP_PROF_CLR, 0, 0, 0, 0, 0, 0, 0);
    vio_verify = 1; pulse_key(3); wait_cyc(10);
    chk(int'(dut.md_q) == 3 && verify_mode === 1, "vio_start with vio_verify = READ LAST");
    vio_verify = 0;
    wait_st(2, 3000, "RUN (READ)"); pulse_key(4); wait_st(4, 20, "CORE_STOP by vio_stop"); wait_idle(); cleanup();
    // LED pages
    wait_cyc(10); chk(led[1:0] === 2'b11, "LED page 0: link_up, calib");
    pulse_key(5); wait_cyc(4); chk(led === 4'h0, "LED page 1 = st (IDLE)");
    ok0(OP_REC); wait_st(2, 3000, "RUN"); wait_cyc(4); chk(led === 4'h2, "LED page 1 = st (RUN)");
    pulse_key(5); wait_cyc(4); chk(led === {1'b0, 3'b101}, "LED page 2 = cs");
    pulse_key(5); wait_cyc(4); chk(led === 4'h0, "LED page 3 = flags");
    pulse_key(5); wait_cyc(4); chk(led[1:0] === 2'b11, "LED page 0 again");
    ok0(OP_STOP); wait_idle(); cleanup();

    // ------------------------------------------------------------ T16: random msg_ready, whole session
    $display("T16 backpressure");
    rdy_mode = 1; flush_msgs(); hist_s = "";
    rec_to_run(0); wait_cyc(200); m_wr = 48'h40; m_beats = 64'h4000;
    ok0(OP_PFAIL); wait_idle();
    cmd(OP_STAT, 0, 0, 0, 0, 0); get_rep(m); chk(m.k == MK_TEL, "STAT? under backpressure"); get_rep(m); chk(m.k == MK_OK, "OK under backpressure");
    get_ev(MK_EV_RP, m);
    chk(m.s.md === 2 && m.s.rp_pad === 32 && m.s.rp_lpos === 40'd95, "!RP coherent with random msg_ready");
    chk_hist("1 2 3 4 5 0 ");
    rdy_mode = 0; cleanup();



    // ------------------------------------------------------------ T17: D7 early STOP / PF before start_load
    $display("T17 early STOP/PF");
    for (int v = 0; v < 2; v++) begin
      knobs_default(); zero_cnt(); flush_msgs(); hist_s = "";
      m_nsze = 0;                                   // capacity engine runs in RUN: start_load is ~12 clk away
      ok0(OP_REC); wait_st(2, 3000, "RUN");
      if (v == 0) ok0(OP_STOP); else ok0(OP_PFAIL);
      wait_st(0, 20000, "IDLE (early stop must not hang SRC_STOP)");
      chk(n_startl == 0, "D7: start cancelled, no start_load");
      get_ev((v == 0) ? MK_EV_RR : MK_EV_RP, m);
      cleanup();
    end

    // ------------------------------------------------------------ T18: D3 link loss / IDLE core / watchdog
    $display("T18 link loss");
    for (int where = 0; where < 6; where++) begin
      knobs_default(); zero_cnt(); flush_msgs(); hist_s = "";
      case (where)
        0: begin k_bring = 1_000_000; ok0(OP_REC); wait_st(1, 100, "BRINGUP"); link_fail = 1; end
        1: begin rec_to_run(0); wait_cyc(50); link_lost = 1; end
        2: begin k_hold_done = 1; rec_to_run(0); ok0(OP_STOP); wait_st(5, 5000, "RELEASE"); link_lost = 1; end
        3: begin rec_to_run(0); wait_cyc(50); k_force_idle = 1; end
        4: begin k_t2 = 100000; rec_to_run(0); ok0(OP_STOP); wait_st(3, 20, "SRC_STOP"); wait_cyc(50); k_force_idle = 1; end
        5: begin k_shut = 100000; rec_to_run(0); ok0(OP_STOP); wait_st(4, 5000, "CORE_STOP"); wait_cyc(50); k_force_idle = 1; end
      endcase
      wait_cyc(6); k_force_idle = 0;
      wait_st(8, 40, "CORE_RST after link/IDLE error");
      chk(ctrl_start === 0 && ctrl_stop === 0, "D3: ctrl_start/stop dropped");
      link_lost = 0; link_fail = 0;
      wait_st(0, 6000, "IDLE");
      get_ev(MK_EV_E, m); chk(m.a0[27:24] === ((where == 0) ? 4'd1 : (where == 1 || where == 3) ? 4'd2 : (where == 2) ? 4'd5 : (where == 4) ? 4'd3 : 4'd4),
                              $sformatf("!E st field %0h (where %0d)", m.a0, where));
      cleanup();
    end
    // watchdog: the core status stays stale for > 3 s
    knobs_default(); zero_cnt(); flush_msgs();
    ok0(OP_REC); wait_st(1, 100, "BRINGUP"); core_age_ok = 0;      // review fix C: the 3 s limit stays in BRINGUP only
    wait_cyc(590_000); chk(int'(dut.st_q) == 1, "watchdog: not yet at 2.95 s");
    wait_cyc(15_000);  chk(int'(dut.st_q) == 7 || int'(dut.st_q) == 8, "D3: 3 s watchdog -> ERROR");
    core_age_ok = 1; wait_st(0, 6000, "IDLE after watchdog"); cleanup();

    // ------------------------------------------------------------ T19: g, READ right after verify_mode fell
    $display("T19 vio READ guard");
    ok0(OP_READ_LAST); wait_st(2, 3000, "RUN (READ)"); m_cmd_limit = 1; wait_st(0, 6000, "IDLE");
    // IDLE right after the READ: the guard of D10 (vm_hold_q, 256 clk after verify_mode fell) blocks a VIO READ, later it starts
    vio_verify = 1; pulse_key(3); wait_cyc(10);
    chk(int'(dut.md_q) != 3, "g: VIO READ right after the previous READ must be blocked");
    vio_verify = 0; wait_cyc(300); vio_verify = 1; pulse_key(3); wait_cyc(10);
    chk(int'(dut.md_q) == 3, "g: VIO READ after the guard time starts");
    vio_verify = 0; cleanup();

    // ------------------------------------------------------------ T20 (review fix E): DTEST STOP with a slow drain
    $display("T20 E: DTEST STOP, drain > 1024 clk");
    knobs_default(); zero_cnt(); flush_msgs(); hist_s = "";
    ok0(OP_DTEST); wait_cyc(1500);
    chk(n_dt == 1 && n_startl == 1, "E: DTEST started once");
    k_ddr_busy = 1;
    ok0(OP_STOP); wait_cyc(3000);
    chk(n_startl == 1 && n_dt == 1, "E: start_load / dtest_start re-pulsed after STOP (source restarted)");
    chk(n_stop == 1, "E: src_stop once");
    k_ddr_busy = 0; wait_st(0, 3000, "IDLE after DTEST");
    get_ev(MK_EV_RD, m);
    cleanup();

    // ------------------------------------------------------------ T21 (review fix J): STOP in BRINGUP -> dur 0
    $display("T21 J: dur after STOP in BRINGUP");
    knobs_default(); zero_cnt(); flush_msgs(); hist_s = "";
    rec_to_run(0); ok0(OP_STOP); wait_idle(); get_ev(MK_EV_RR, m);
    chk(m.s.dur != 0, "J: normal session has dur > 0");
    flush_msgs();
    k_bring = 1_000_000;
    ok0(OP_REC); wait_st(1, 100, "BRINGUP"); wait_cyc(50); ok0(OP_STOP); wait_idle();
    get_ev(MK_EV_RR, m);
    chk(m.s.dur == 64'd0, $sformatf("J: dur %0d after a session stopped in BRINGUP (stale t_run_q)", m.s.dur));
    cleanup();

    // ------------------------------------------------------------ T22 (review fix I): full message FIFO loses nothing
    $display("T22 I: !S / !R with a blocked message path");
    knobs_default(); zero_cnt(); flush_msgs(); hist_s = "";
    for (int i = 0; i < 64; i++) ok(OP_PROF_ADD, 0, 2, 64'(i + 1), 64'h100, 0, 0, 0);
    rec_to_run(0);
    flush_msgs(); hist_s = "";
    rdy_mode = 2;
    cmd(OP_PROFQ, 0, 0, 0, 0, 0);
    wait_cyc(200);
    pulse_key(1);                                                   // STOP by key (no reply path needed)
    wait_st(0, 20000, "IDLE with a blocked message path");
    wait_cyc(50);
    rdy_mode = 0;
    wait_cyc(6000);
    chk(hist_s == "3 4 5 0 ", $sformatf("I: state history '%s' expected '3 4 5 0 ' (!S lost in a full FIFO)", hist_s));
    get_ev(MK_EV_RR, m);
    begin
      int n_pl; n_pl = 0;
      foreach (rep_q[i]) if (rep_q[i].k == MK_PROF_LINE) n_pl++;
      chk(n_pl == 64, $sformatf("I: %0d PROF? lines of 64", n_pl));
    end
    flush_msgs(); ok0(OP_PROF_CLR);
    cleanup();

    // ------------------------------------------------------------ T23 (review fix C): dead core status in RUN
    $display("T23 C: dead core status in RUN");
    knobs_default(); zero_cnt(); flush_msgs(); hist_s = "";
    rec_to_run(0); core_age_ok = 0;
    wait_cyc(5000);                                                 // 25 ms
    chk(int'(dut.st_q) == 7 || int'(dut.st_q) == 8, "C: stale core status in RUN not detected within 25 ms");
    core_age_ok = 1; wait_st(0, 6000, "IDLE");
    get_ev(MK_EV_E, m); chk(m.a0[27:24] === 4'd2, "C: !E in RUN");
    cleanup();

    // ------------------------------------------------------------ T24 (review fix D): progress watchdog in SRC_STOP / LOCAL
    $display("T24 D: stuck drain");
    knobs_default(); zero_cnt(); flush_msgs(); hist_s = "";
    rec_to_run(0); k_ddr_busy = 1; ok0(OP_STOP);
    wait_st(3, 20, "SRC_STOP");
    wait_st(4, 2_100_000, "CORE_STOP by the progress watchdog (stuck ddr_idle)");
    k_ddr_busy = 0; wait_idle();
    get_ev(MK_EV_E, m); chk(m.a0[27:24] === 4'd5, "D: !E after a stuck drain");
    get_ev(MK_EV_RR, m);
    cleanup();
    knobs_default(); zero_cnt(); flush_msgs(); hist_s = "";
    rec_to_run(0); ok0(OP_STOP);
    wait_st(3, 20, "SRC_STOP"); ui_age_ok = 0;
    wait_st(4, 3000, "CORE_STOP: ui status dead > 10 ms");
    ui_age_ok = 1; wait_idle();
    get_ev(MK_EV_E, m);
    cleanup();
    knobs_default(); zero_cnt(); flush_msgs(); hist_s = "";
    ok0(OP_DTEST); wait_cyc(1500); k_ddr_busy = 1; ok0(OP_STOP);
    wait_st(8, 2_100_000, "CORE_RST: DTEST drain stuck");
    wait_st(0, 6000, "IDLE");
    get_ev(MK_EV_E, m); chk(m.a0[27:24] === 4'd6, "D: !E from LOCAL");
    cleanup();

    // ------------------------------------------------------------ T25 (review fix F): READ END, first snapshot already RUN
    $display("T25 F: READ END with the core already in RUN");
    knobs_default(); zero_cnt(); flush_msgs(); hist_s = "";
    k_force_cs = 4'd5;
    ok0(OP_READ_END); wait_st(2, 3000, "RUN"); wait_cyc(60);
    chk(verify_cmds === 32'd4096, $sformatf("F: READ END verify_cmds %0d (capacity 4096)", verify_cmds));
    cleanup();

    $display("TEST PASSED");
    $finish;
  end

endmodule
