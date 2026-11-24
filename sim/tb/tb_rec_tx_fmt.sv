// =============================================================================
// tb_rec_tx_fmt.sv — rec_tx_fmt + uart_tx (WP1) at 921600 baud, decoded from the pin by an
// independent bit-level receiver.  Every message kind / opcode is compared byte for byte with
// a reference string that is built here from the spec tables (2.3, 2.5, 2.6, 7.2).
// Also: snapshot copied at acceptance (inputs scrambled right after the handshake), messages
// without text, tel_tick (idle, merged, behind a long line), priority of a waiting message over a
// pending tick, lines never interleaved, FIFO back-pressure (> 2040 bytes queued), reset.
// Negative controls: a deliberately wrong reference line is flagged by the comparator; three
// sources (tick + two messages) collide while a line is running: none is cut or merged.
// All received lines are also written to lines_dump.txt (checked with tools/recorder.py by
// sim/vectors/check_fmt_lines.py).
// =============================================================================
`timescale 1ns/1ps

module tb_rec_tx_fmt;
  import rec_pkg::*;

  logic clk = 1'b0;
  logic rst = 1'b1;
  always #2.5 clk = ~clk;
  localparam int DIV = 217;

  logic        msg_valid = 1'b0, msg_ready;
  logic [4:0]  msg_kind = '0;
  logic [7:0]  msg_code = '0;
  logic [63:0] msg_arg0 = '0, msg_arg1 = '0;
  rec_snap_t   snap;
  logic        tel_tick = 1'b0;
  logic        txd;

  rec_tx_fmt u_dut (
    .clk(clk), .rst(rst), .msg_valid(msg_valid), .msg_ready(msg_ready), .msg_kind(msg_kind),
    .msg_code(msg_code), .msg_arg0(msg_arg0), .msg_arg1(msg_arg1), .snap(snap),
    .tel_tick(tel_tick), .txd(txd));

  int errors = 0;
  task automatic check(input bit cond, input string msg);
    if (!cond) begin errors++; $display("ERROR: %s", msg); end
  endtask

  // ------------------------------------------------------------ pin receiver
  string lines [$];
  string cur = "";
  int    badstop = 0;
  int    nbytes = 0;
  integer fd;
  initial begin
    fd = $fopen("lines_dump.txt", "w");
    forever begin
      logic [7:0] b;
      @(negedge txd);
      repeat (DIV / 2) @(posedge clk);
      if (txd !== 1'b0) begin errors++; $display("ERROR: start bit glitch"); end
      for (int i = 0; i < 8; i++) begin
        repeat (DIV) @(posedge clk);
        b[i] = txd;
      end
      repeat (DIV) @(posedge clk);
      if (txd !== 1'b1) begin badstop++; end
      nbytes++;
      if (b == 8'h0A) begin
        lines.push_back(cur);
        $fdisplay(fd, "%s", cur);
        cur = "";
      end else begin
        cur = $sformatf("%s%c", cur, b);
      end
    end
  end

  // ------------------------------------------------------------ helpers
  task automatic wait_clks(input int n);
    repeat (n) @(posedge clk);
    #1;
  endtask

  function automatic string hx(input logic [63:0] v, input int w);
    string r;
    string digs;
    digs = "0123456789ABCDEF";
    r = "";
    for (int i = w - 1; i >= 0; i--) r = $sformatf("%s%c", r, digs.getc(int'(v[4*i +: 4])));
    return r;
  endfunction
  function automatic string kv(input string k, input logic [63:0] v, input int w);
    return $sformatf(" %s=%s", k, hx(v, w));
  endfunction

  localparam int SNW = $bits(rec_snap_t);
  function automatic rec_snap_t rnd_snap();
    logic [((SNW + 31) / 32) * 32 - 1 : 0] v;
    for (int i = 0; i < ((SNW + 31) / 32); i++) v[i*32 +: 32] = $urandom;
    return rec_snap_t'(v[SNW-1:0]);
  endfunction
  function automatic logic [63:0] rnd64();
    return {$urandom, $urandom};
  endfunction

  // reference lines (spec tables), independent of the ROM in the DUT
  function automatic string exp_T(input rec_snap_t s);
    return {"$T v=01", kv("t", s.t_ms, 8), kv("st", s.st, 1), kv("md", s.md, 1), kv("cok", s.cok, 1),
            kv("uok", s.uok, 1), kv("cs", s.cs, 1), kv("ce", s.ce, 2), kv("cd", s.cd, 2),
            kv("fl", s.fl, 2), kv("lk", s.lk, 1), kv("cal", s.cal, 1), kv("stp", s.stp, 8),
            kv("seg", s.seg, 2), kv("rep", s.rep, 4), kv("src", s.src, 12), kv("wr", s.wr, 12),
            kv("drp", s.drp, 8), kv("ddr", s.ddr, 8), kv("ddrm", s.ddrm, 8), kv("bpm", s.bpm, 10),
            kv("rng", s.rng, 8), kv("ssd", s.ssd, 16), kv("cmd", s.cmd, 8), kv("cer", s.cer, 8),
            kv("lat", s.lat, 8), kv("vb", s.vb, 12), kv("ve", s.ve, 8), kv("tk", s.tk, 4),
            kv("cw", s.cw, 2), kv("smo", s.smo, 4), kv("sme", s.sme, 4), kv("xt", s.xt, 3),
            kv("xvi", s.xvi, 3), kv("xva", s.xva, 3), kv("xvb", s.xvb, 3)};
  endfunction
  function automatic string exp_cfg(input rec_snap_t s);
    return {"OK CFG", kv("md", s.md, 1), kv("bp", s.cfg_bp, 1), kv("stp", s.cfg_stp, 8),
            kv("len", s.cfg_len, 8), kv("sid", s.cfg_sid, 8), kv("ddr", s.cfg_ddr, 4),
            kv("tel", s.cfg_tel, 4), kv("sm", s.cfg_sm, 4), kv("pn", s.cfg_pn, 2),
            kv("pr", s.cfg_pr, 4), kv("pe", s.cfg_pe, 1), kv("rsid", s.cfg_rsid, 8)};
  endfunction
  function automatic string exp_smartq(input logic [63:0] a0, input rec_snap_t s);
    return {"OK SMART", kv("cw", s.smart.cw, 2), kv("tk", s.smart.tk, 4), kv("sp", s.smart.sp, 2),
            kv("us", s.smart.sp_used, 2), kv("dur", s.smart.dur, 16), kv("duw", s.smart.duw, 16),
            kv("pc", s.smart.pc, 8), kv("poh", s.smart.poh, 8), kv("uns", s.smart.uns, 8),
            kv("me", s.smart.me, 8), kv("wt", s.smart.wt, 8), kv("ct", s.smart.ct, 8),
            kv("t1", s.smart.t1, 4), kv("ok", s.smo, 4), kv("er", s.sme, 4), kv("st", a0, 4),
            kv("rtd", s.rtd, 8)};
  endfunction
  function automatic string exp_ddr(input rec_snap_t s);
    return {"OK DDR", kv("cal", s.cal, 1), kv("ddr", s.ddr, 8), kv("ddrm", s.ddrm, 8),
            kv("mte", s.mte, 8), kv("mta", s.mta, 8)};
  endfunction

  function automatic string exp_ok(input int code, input logic [63:0] a0, input rec_snap_t s);
    case (code)
      OP_PING:       return {"OK PING", kv("ver", REC_VER, 2), kv("bld", 32'h20261124, 8), kv("cal", s.cal, 1), kv("uok", s.uok, 1)};
      OP_CFGQ:       return exp_cfg(s);
      OP_TEL:        return {"OK TEL", kv("tel", a0, 4)};
      OP_SMART:      return {"OK SMART", kv("sm", a0, 4)};
      OP_SMARTQ:     return exp_smartq(a0, s);
      OP_MODE:       return {"OK MODE", kv("bp", a0, 1)};
      OP_RATE:       return {"OK RATE", kv("stp", a0, 8)};
      OP_LEN:        return {"OK LEN", kv("len", a0, 8)};
      OP_SID:        return {"OK SID", kv("sid", a0, 8)};
      OP_DDRSZ:      return {"OK DDRSZ", kv("ddr", a0, 4)};
      OP_PROF_CLR:   return {"OK PROF", kv("pn", a0, 2)};
      OP_PROF_ADD,
      OP_PROF_SET:   return {"OK PROF", kv("i", a0, 2)};
      OP_PROF_REP:   return {"OK PROF", kv("pr", a0, 4)};
      OP_PROF_END:   return {"OK PROF", kv("pe", a0, 1)};
      OP_PROFQ:      return {"OK PROF", kv("pn", s.cfg_pn, 2), kv("pr", s.cfg_pr, 4), kv("pe", s.cfg_pe, 1)};
      OP_REC,
      OP_REC_PROF:   return {"OK REC", kv("sid", a0, 8)};
      OP_STOP:       return "OK STOP";
      OP_PFAIL:      return "OK PFAIL";
      OP_ABORT:      return "OK ABORT";
      OP_READ_LAST,
      OP_READ_N,
      OP_READ_END:   return "OK READ";
      OP_READ_SID:   return {"OK READ", kv("rsid", a0, 8)};
      OP_FORMAT:     return "OK FORMAT";
      OP_MON:        return "OK MON";
      OP_MEMTEST:    return {"OK MEMTEST", kv("mib", a0, 4)};
      OP_DTEST:      return "OK DTEST";
      OP_DDRQ:       return exp_ddr(s);
      default:       return "OK";                       // STAT?, RES?, NOP, PERR ...
    endcase
  endfunction

  function automatic string exp_str(input int kind, input int code, input logic [63:0] a0,
                                    input logic [63:0] a1, input rec_snap_t s);
    case (kind)
      MK_OK:        return exp_ok(code, a0, s);
      MK_ERR:       return {"ERR ", hx(code, 2)};
      MK_PROF_LINE: return {"P", kv("i", a0[63:56], 2), kv("stp", a0, 8), kv("dur", a1, 10)};
      MK_TEL:       return exp_T(s);
      MK_CFGQ:      return exp_cfg(s);
      MK_SMARTQ:    return exp_smartq(a0, s);
      MK_DDRQ:      return exp_ddr(s);
      MK_EV_B:      return {"!B", kv("ver", REC_VER, 2), kv("cal", s.cal, 1), kv("uok", s.uok, 1)};
      MK_EV_S:      return {"!S", kv("st", a0[3:0], 1), kv("md", a0[7:4], 1), kv("t", a1, 8)};
      MK_EV_E:      return {"!E", kv("ce", a0[7:0], 2), kv("cd", a0[15:8], 2), kv("fl", a0[23:16], 2), kv("st", a0[27:24], 1)};
      MK_EV_RR:     return {"!RR", kv("sid", s.sid, 8), kv("wr", s.wr, 12), kv("drp", s.drp, 8),
                            kv("pad", s.pad_blocks, 8), kv("trc", s.trunc_blocks, 8), kv("ddrm", s.ddrm, 8),
                            kv("ssd", s.ssd, 16), kv("cmd", s.cmd, 8), kv("cer", s.cer, 8),
                            kv("lat", s.lat, 8), kv("dur", s.dur, 16)};
      MK_EV_RP:     return {"!RP", kv("src", s.rp_src, 1), kv("inb", s.rp_inb, 1), kv("lat", s.rp_lat, 1),
                            kv("lov", s.rp_lov, 1), kv("pnd", s.rp_pnd, 12), kv("pdd", s.rp_pdd, 8),
                            kv("prg", s.rp_prg, 8), kv("pad", s.rp_pad, 8), kv("t1", s.rp_t1, 16),
                            kv("t2", s.rp_t2, 16), kv("t3", s.rp_t3, 16), kv("t4", s.rp_t4, 16),
                            kv("t5", s.rp_t5, 16), kv("t6", s.rp_t6, 16), kv("ok", s.rp_ok, 1),
                            kv("rtd", s.rtd, 8), kv("ssd", s.ssd, 16), kv("lpos", s.rp_lpos, 10)};
      MK_EV_RV:     return {"!RV", kv("vb", s.vrf.vb, 12), kv("ok", s.vrf.ok, 10), kv("bad", s.vrf.bad, 8),
                            kv("crc", s.vrf.crc, 8), kv("pe", s.vrf.pe, 8), kv("ooo", s.vrf.ooo, 8),
                            kv("prt", s.vrf.prt, 8), kv("fbp", s.vrf.fbp, 10), kv("mvp", s.vrf.mvp, 10),
                            kv("lsq", s.vrf.lsq, 16), kv("lts", s.vrf.lts, 16), kv("lst", s.vrf.lst, 8),
                            kv("pad", s.vrf.pad, 8), kv("fep", s.vrf.fep, 10), kv("feb", s.vrf.feb, 2),
                            kv("exp", s.vrf.exp_dw, 8), kv("got", s.vrf.got_dw, 8),
                            kv("end", s.end_found, 1), kv("ovf", s.vrf.ovf, 1),
                            kv("lfl", s.vrf.mvp_flags, 2), kv("dur", s.dur, 16)};   // WP8: lfl
      MK_EV_RF:     return {"!RF", kv("ms", a0, 8), kv("ok", a1, 1)};
      MK_EV_RM:     return {"!RM", kv("byt", s.mt_bytes, 8), kv("err", s.mte, 8), kv("adr", s.mta, 8),
                            kv("dur", s.dur, 16)};
      MK_EV_RD:     return {"!RD", kv("wr", s.dt_wr, 12), kv("drp", s.drp, 8), kv("he", s.dt_he, 8),
                            kv("dur", s.dur, 16)};
      default:      return "";
    endcase
  endfunction

  string expq [$];

  // present a message, wait for the handshake, then scramble every input (the DUT must have copied)
  task automatic send_msg(input int kind, input int code, input logic [63:0] a0, input logic [63:0] a1,
                          input rec_snap_t s, input bit expect_text = 1'b1);
    snap = s; msg_kind = 5'(kind); msg_code = 8'(code); msg_arg0 = a0; msg_arg1 = a1;
    msg_valid = 1'b1;
    @(posedge clk);
    while (!msg_ready) @(posedge clk);       // handshake at this edge (valid && ready)
    #1;
    msg_valid = 1'b0;
    if (expect_text) expq.push_back(exp_str(kind, code, a0, a1, s));
    snap = rnd_snap(); msg_kind = 5'($urandom); msg_code = 8'($urandom); msg_arg0 = rnd64(); msg_arg1 = rnd64();
  endtask

  // wait until n more lines arrived, or fail
  task automatic wait_lines(input int n_total, input int timeout_ms);
    int t;
    t = 0;
    while (lines.size() < n_total && t < timeout_ms * 1000) begin
      wait_clks(200);       // 1 us
      t++;
    end
    check(lines.size() >= n_total, $sformatf("timeout: %0d of %0d lines", lines.size(), n_total));
  endtask

  bit neg_mode = 1'b0;
  int neg_detected = 0;
  task automatic compare_all(input string tag);
    int n;
    bit bad;
    bad = 1'b0;
    n = (lines.size() < expq.size()) ? lines.size() : expq.size();
    if (lines.size() != expq.size()) begin
      bad = 1'b1;
      if (!neg_mode) $display("ERROR [%s] %0d lines received, %0d expected", tag, lines.size(), expq.size());
    end
    for (int i = 0; i < n; i++)
      if (lines[i] != expq[i]) begin
        bad = 1'b1;
        if (!neg_mode) $display("ERROR [%s] line %0d differs: got [%s] exp [%s]", tag, i, lines[i], expq[i]);
      end
    if (bad) begin if (neg_mode) neg_detected++; else errors++; end
    lines.delete(); expq.delete();
  endtask

  task automatic collide(input int n_pre, input int settle_clks);
    rec_snap_t sx, sy, st_;
    sx = rnd_snap(); sy = rnd_snap(); st_ = rnd_snap();
    for (int i = 0; i < n_pre; i++) send_msg(MK_TEL, 0, 0, 0, rnd_snap());
    wait_clks(settle_clks);
    check(!msg_ready, "collide: formatter must be busy here");
    tel_tick = 1'b1; wait_clks(1); tel_tick = 1'b0; wait_clks(100);
    snap = sx; msg_kind = 5'(MK_OK); msg_code = 8'(OP_PFAIL); msg_arg0 = 0; msg_arg1 = 0;
    msg_valid = 1'b1;                                       // X waits for ready
    tel_tick = 1'b1; wait_clks(1); tel_tick = 1'b0;
    @(posedge clk);
    while (!msg_ready) @(posedge clk);
    #1; msg_valid = 1'b0;
    expq.push_back("OK PFAIL");
    send_msg(MK_EV_S, 0, 0, 0, sy);                         // Y is presented while the tick is pending
    snap = st_;                                             // the tick line copies this
    expq.push_back(exp_T(st_));
    wait_lines(expq.size(), 150);
    wait_clks(1_000_000);                                   // 5 ms: no further line may appear
    compare_all($sformatf("three sources n_pre=%0d", n_pre));
  endtask

  // ------------------------------------------------------------ main
  rec_snap_t s0, sT;
  int ncode;
  int kinds [] = '{MK_OK, MK_ERR, MK_PROF_LINE, MK_TEL, MK_CFGQ, MK_SMARTQ, MK_DDRQ, MK_EV_B, MK_EV_S, MK_EV_E,
                   MK_EV_RR, MK_EV_RP, MK_EV_RV, MK_EV_RF, MK_EV_RM, MK_EV_RD};
  int ops [] = '{OP_NOP, OP_PING, OP_STAT, OP_CFGQ, OP_RESQ, OP_TEL, OP_SMART, OP_SMARTQ, OP_MODE, OP_RATE, OP_LEN,
                 OP_SID, OP_DDRSZ, OP_PROF_CLR, OP_PROF_ADD, OP_PROF_SET, OP_PROF_REP, OP_PROF_END, OP_PROFQ,
                 OP_REC, OP_REC_PROF, OP_STOP, OP_PFAIL, OP_READ_LAST, OP_READ_N, OP_READ_END, OP_READ_SID,
                 OP_FORMAT, OP_MON, OP_MEMTEST, OP_DTEST, OP_DDRQ, OP_ABORT, OP_PERR};

  initial begin
    snap = rnd_snap();
    wait_clks(10);
    rst = 1'b0;
    wait_clks(20);
    check(txd === 1'b1, "txd idle high");

    // ---------------------------------------------------------------- every kind and opcode
    foreach (ops[i]) begin
      send_msg(MK_OK, ops[i], rnd64(), rnd64(), rnd_snap());
    end
    foreach (kinds[i]) begin
      if (kinds[i] != MK_OK) begin
        ncode = (kinds[i] == MK_ERR) ? 8'h0A : (kinds[i] == MK_PROF_LINE) ? 8'h3F : 8'h00;
        send_msg(kinds[i], ncode, rnd64(), rnd64(), rnd_snap());
      end
    end
    // error codes of spec 2.4
    begin
      automatic int codes [] = '{ERR_SYNTAX, ERR_UNKNOWN, ERR_RANGE, ERR_STATE, ERR_NOCAL, ERR_CORE, ERR_PROF_EMPTY,
                       ERR_PROF_FULL, ERR_LONG, ERR_NOREC, ERR_BUSY, ERR_UART, 8'hFF, 8'h00};
      foreach (codes[i]) send_msg(MK_ERR, codes[i], 0, 0, rnd_snap());
    end
    // extreme values: all ones and all zeros snapshots, 64-bit args
    s0 = '0;  send_msg(MK_TEL, 0, '0, '0, s0);
    s0 = '1;  send_msg(MK_TEL, 0, '1, '1, s0);
    s0 = '1;  send_msg(MK_EV_RV, 0, '1, '1, s0);
    s0 = '1;  send_msg(MK_EV_RP, 0, '1, '1, s0);
    send_msg(MK_PROF_LINE, 8'h3F, 64'hFFFFFFFF_2CCCCCCC, 64'hFFFFFFFF_FFB2D05E, rnd_snap());   // stp = low 32, dur = low 40
    // messages without text are consumed silently
    send_msg(MK_NONE, 0, 0, 0, rnd_snap(), 1'b0);
    send_msg(5'd31, 0, 0, 0, rnd_snap(), 1'b0);
    send_msg(MK_OK, OP_PING, 64'h20261201, 0, rnd_snap());
    wait_lines(expq.size(), 120);
    wait_clks(20000);
    compare_all("all kinds");

    // ---------------------------------------------------------------- tel_tick when idle
    sT = rnd_snap();
    snap = sT;
    tel_tick = 1'b1; wait_clks(1); tel_tick = 1'b0;
    expq.push_back(exp_T(sT));
    wait_lines(1, 20);
    wait_clks(5000);
    compare_all("tick idle");

    // ---------------------------------------------------------------- three sources collide
    // A: while a $T line is being formatted (the formatter is busy for a few microseconds):
    //    tick, message X, tick arrive, then message Y is presented right after X.
    // B: the same while the formatter is stalled in the middle of the 6th $T line (FIFO full).
    // Expected in both: the running line(s) whole, X, Y (messages before the pending tick), then
    // exactly ONE tick line; nothing is cut, merged or doubled.
    collide(1, 150);
    collide(6, 20000);

    // ---------------------------------------------------------------- FIFO back-pressure
    // 7 x $T = 2.5 KB queued faster than the UART drains: the formatter has to wait (>= 2040 bytes)
    for (int i = 0; i < 7; i++) send_msg(MK_TEL, 0, 0, 0, rnd_snap());
    send_msg(MK_OK, OP_STOP, 0, 0, rnd_snap());
    wait_lines(expq.size(), 120);
    wait_clks(20000);
    compare_all("backpressure");
    check(badstop == 0, "stop bit of every byte is high");

    // ---------------------------------------------------------------- negative control of the comparator
    neg_mode = 1'b1;
    send_msg(MK_OK, OP_MODE, 64'h1, 0, rnd_snap());
    wait_lines(1, 5);
    wait_clks(2000);
    expq[0] = "OK MODE bp=0";                       // wrong digit
    compare_all("neg digit");
    send_msg(MK_OK, OP_PING, 64'h20261201, 0, rnd_snap());
    wait_lines(1, 5);
    wait_clks(2000);
    expq[0] = "";                                    // line missing / extra
    expq.delete();
    compare_all("neg extra line");
    send_msg(MK_OK, OP_STOP, 0, 0, rnd_snap());
    wait_lines(1, 5);
    wait_clks(2000);
    expq.push_back("OK STOP"); expq.push_back("OK STOP");   // one line missing
    compare_all("neg missing line");
    neg_mode = 1'b0;
    check(neg_detected == 3, $sformatf("negative control: %0d of 3 detected", neg_detected));

    // ---------------------------------------------------------------- reset
    send_msg(MK_OK, OP_PING, 64'h1, 0, rnd_snap(), 1'b0);
    wait_clks(30000);
    rst = 1'b1; wait_clks(5); rst = 1'b0; wait_clks(50);
    // the partial line that was on the wire is garbage; after the reset a clean message must arrive
    wait_clks(20000);
    cur = ""; lines.delete(); expq.delete();
    send_msg(MK_OK, OP_STOP, 0, 0, rnd_snap());
    wait_lines(1, 10);
    wait_clks(5000);
    compare_all("after reset");

    $fclose(fd);
    check(badstop == 0, "stop bits");
    if (errors != 0) $fatal(1, "tb_rec_tx_fmt: %0d errors", errors);
    $display("TEST PASSED");
    $finish;
  end

  initial begin
    #900ms;
    $fatal(1, "tb_rec_tx_fmt: timeout");
  end
endmodule
