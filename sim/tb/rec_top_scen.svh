// scenarios of the recorder system tests (included into the module body, see rec_top_body.svh)

  task automatic wait_boot();
    string l; int idx;
    wait_ev("!B", 0, 5000, l, idx);
  endtask

  // one REC session: send `c`, wait for !RR and the return to IDLE
  task automatic do_rec(input string c, input int tmo_us, output string rr, output int mark);
    string r; int idx;
    mark = u_bfm.lines.size();
    cmd_ok(c, "OK REC", r);
    wait_ev("!RR", mark, tmo_us, rr, idx);
    wait_idle(mark, 100000);
  endtask

  // READ <what>: returns the !RV line
  task automatic do_read(input string c, input int tmo_us, output string rv);
    string r; int mark, idx;
    mark = u_bfm.lines.size();
    cmd_ok(c, "OK READ", r);
    wait_ev("!RV", mark, tmo_us, rv, idx);
    wait_idle(mark, 100000);
  endtask

  // ---------------------------------------------------------------- S1: PING, CFG?, TEL (+ text-path negatives)
  task automatic s1_ping_cfg_tel();
    string r, l;
    int i, n, nx, mark, idx;
    longint v;
    cur_scen = "S1";
    nx = 0;
    say("PING / CFG? / TEL");
    wait_ev("!B ver=01 cal=1 uok=1", 0, 8000, l, idx);
    // first the defaults, before the 1 ms default telemetry period (SIM_FAST) floods the wire with 3.9 ms lines
    cmd_ok("CFG?", "OK CFG", r);
    ck_f(r, "md", 0); ck_f(r, "bp", 0); ck_f(r, "stp", 32'h10000000); ck_f(r, "len", 0); ck_f(r, "ddr", 16'h0400);
    ck_f(r, "tel", 16'h03E8); ck_f(r, "sm", 16'h03E8); ck_f(r, "pn", 0); ck_f(r, "pr", 1); ck_f(r, "pe", 1);
    cmd_ok("PING", "OK PING", r);
    ck_f(r, "ver", 16'h01); ck_f(r, "bld", 32'h20261124); ck_f(r, "cal", 1); ck_f(r, "uok", 1);
    // TEL: lines of 355 characters, fields of an idle recorder
    mark = u_bfm.lines.size();
    cmd_ok("TEL 0FA0", "OK TEL", r);       // 4000 "ms" = 4 ms with SIM_FAST: about one line time
    ck_f(r, "tel", 16'h0FA0);
    wait_ev("$T", mark, 20000, l, idx);
    u_bfm.wait_line("$T", idx + 1, 20000, i);
    cmd_ok("TEL 0000", "OK TEL", r);
    #(4_500_000.0);   // let the line in flight drain
    n = 0;
    for (i = mark; i < u_bfm.lines.size(); i++) begin
      l = u_bfm.lines[i];
      if (l.len() >= 3 && l.substr(0, 2) == "$T ") begin
        n++;
        ck_eq("$T length", l.len(), 355);
        ck_f(l, "v", 1); ck_f(l, "st", 0); ck_f(l, "md", 0); ck_f(l, "cal", 1); ck_f(l, "uok", 1); ck_f(l, "cok", 1);
        ck_f(l, "stp", 0); ck_f(l, "drp", 0); ck_f(l, "cs", 0);          // idle NCO: step 0 (the configured step is in CFG?)
        if (u_bfm.fld(l, "xvi") != 0) begin                                // first XADC refresh not yet complete otherwise
          v = u_bfm.fld(l, "xvi");  ck(v >= 12'h500 && v <= 12'h5A0, $sformatf("xvi %h not ~1.0 V", v));
          v = u_bfm.fld(l, "xt");   ck(v >= 12'hA00 && v <= 12'hC00, $sformatf("xt %h not ~50 C", v));
          nx++;
        end
        ck(u_bfm.fld(l, "t") >= 0, "t missing");
      end
    end
    ck(n >= 2, $sformatf("expected >= 2 $T lines, got %0d", n));
    ck(nx >= 1, "no $T line with valid XADC codes");
    say($sformatf("%0d $T lines checked", n));
    // negatives: unknown keyword, range, bad stop bit, long line, line before the reply
    cmd_err("FROB", 8'h02);
    cmd_err("TEL 0005", 8'h03);
    cmd_err("TEL", 8'h01);
    mark = u_bfm.lines.size();
    u_bfm.send_bad_stop("P");
    u_bfm.send_line("ING");
    wait_ev("ERR", mark, 5000, l, idx);
    ck(l == "ERR 0D", $sformatf("bad stop bit: [%s]", l));
    cmd_err("PING AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", 8'h0A);
    // ERR 0C (line before the reply) cannot be provoked over the wire: rec_ctrl builds a reply within microseconds, a
    // 5-character line needs 54 us; it is covered by tb_cmd_parser / tb_rec_ctrl (notes WP8).
    cmd_ok("PING", "OK PING", r);
  endtask

  // ---------------------------------------------------------------- S9: MEMTEST, DTEST (no SSD involved)
  task automatic s9_memtest_dtest();
    string r, l;
    int mark, idx;
    longint wr;
    cur_scen = "S9";
    say("MEMTEST / DTEST");
    cmd_ok("DDRSZ 0001", "OK DDRSZ", r);
    // clean pass
    mark = u_bfm.lines.size();
    cmd_ok("MEMTEST 0001", "OK MEMTEST", r);
    ck_f(r, "mib", 1);
    wait_ev("!RM", mark, 100000, l, idx);
    ck_f(l, "err", 0); ck_f(l, "adr", 0);
    ck(u_bfm.fld(l, "byt") > 0, "!RM byt = 0");
    wait_idle(mark, 20000);
    // negative: one flipped bit in the MIG read data -> exact address, both passes
    `MIG.inject_err_addr = 64;
    mark = u_bfm.lines.size();
    cmd_ok("MEMTEST 0001", "OK MEMTEST", r);
    wait_ev("!RM", mark, 100000, l, idx);
    ck_f(l, "err", 2); ck_f(l, "adr", 64);
    wait_idle(mark, 20000);
    `MIG.inject_err_addr = -1;
    // MEMTEST STOP aborts a running test (audit D5), the FIFO is usable afterwards
    mark = u_bfm.lines.size();
    cmd_ok("MEMTEST 0040", "OK MEMTEST", r);
    #(150_000.0);
    cmd_ok("STOP", "OK STOP", r);
    wait_idle(mark, 20000);
    #(20_000.0);
    ck(dut.mt_busy === 1'b0, "memtest still busy after STOP");
    // DTEST: source -> DDR FIFO -> rec_sink_chk; the core and the SSD see nothing
    cmd_ok("SID 692D3A00", "OK SID", r);
    cmd_ok("RATE 80000000", "OK RATE", r);
    cmd_ok("DDRSZ 0004", "OK DDRSZ", r);
    mark = u_bfm.lines.size();
    cmd_ok("DTEST", "OK DTEST", r);
    #(250_000.0);
    ck_eq("core beats_in during DTEST", dut.stat_beats_in, 0);
    ck_eq("SSD writes during DTEST", `SSD.n_io_writes, 0);
    cmd_ok("STOP", "OK STOP", r);
    wait_ev("!RD", mark, 20000, l, idx);
    wait_idle(mark, 20000);
    ck_f(l, "he", 0); ck_f(l, "drp", 0);
    wr = u_bfm.fld(l, "wr");
    ck(wr > 0 && (wr % 32) == 0, $sformatf("!RD wr=%0d (not a multiple of 32 blocks)", wr));
    ck_eq("sink beats = wr*256", dut.dt_beats, wr * 256);
    // negative: a bit flipped in DDR word 0 (block 0 header) -> he >= 1
    `MIG.inject_err_addr = 0;
    mark = u_bfm.lines.size();
    cmd_ok("DTEST", "OK DTEST", r);
    #(250_000.0);
    cmd_ok("STOP", "OK STOP", r);
    wait_ev("!RD", mark, 20000, l, idx);
    wait_idle(mark, 20000);
    ck(u_bfm.fld(l, "he") >= 1, "DTEST header corruption not detected");
    `MIG.inject_err_addr = -1;
    ck_eq("SSD writes after DTEST", `SSD.n_io_writes, 0);
  endtask

  // ---------------------------------------------------------------- S2: REC 4 MiB, READ LAST, read-path negatives
  localparam logic [31:0] SID2 = 32'h692D3A00;
  task automatic s2_rec_read();
    string r, rr, rv, l;
    int mark, idx, bad;
    longint ls, ll;
    cur_scen = "S2";
    say("REC 4 MiB + READ LAST");
    setup_rec(SID2, 32, 32'h80000000, 32);
    do_rec("REC", 120000, rr, mark);
    // !RR: 1024 source blocks + empty TRUNC + 31 PAD = 1056 blocks = 33 commands
    ck_f(rr, "sid", SID2); ck_f(rr, "wr", 12'h420); ck_f(rr, "drp", 0); ck_f(rr, "pad", 31); ck_f(rr, "trc", 1);
    ck_f(rr, "ssd", 32'h420000); ck_f(rr, "cmd", 33); ck_f(rr, "cer", 0);
    ck(u_bfm.fld(rr, "ddrm") > 0, "ddrm = 0");
    ck(u_bfm.fld(rr, "dur") > 0, "dur = 0");
    // data on the SSD = reference blocks
    check_blocks(1056, SID2, 1'b0, bad, ls, ll);
    ck_eq("bad blocks on SSD", bad, 0);
    ck_eq("last seq", ls, 1055);
    ck(!ssd_has(1056), "block beyond the session written");
    ck_eq("SSD writes", `SSD.n_io_writes, 33);
    // RES? repeats the last !RR
    mark = u_bfm.lines.size();
    cmd_ok("RES?", "OK", r);
    wait_ev("!RR", mark, 5000, l, idx);
    ck(l == rr, "RES? did not repeat !RR");
    // READ LAST
    do_read("READ LAST", 120000, rv);
    ck_f(rv, "vb", 32'h420000); ck_f(rv, "ok", 12'h420); ck_f(rv, "bad", 0); ck_f(rv, "crc", 0); ck_f(rv, "pe", 0);
    ck_f(rv, "ooo", 0); ck_f(rv, "prt", 0); ck_f(rv, "fbp", 40'hFFFFFFFFFF); ck_f(rv, "mvp", 12'h41F);
    ck_f(rv, "lsq", 12'h41F); ck_f(rv, "lst", 0); ck_f(rv, "pad", 31); ck_f(rv, "ovf", 0); ck_f(rv, "end", 0);
    ck_eq("SSD reads", `SSD.n_io_reads, 33);
    // ---- negatives of the read path (2 slots = 64 blocks) ----
    // 1. one payload bit flipped on the read path: block 10, beat 100, byte 3 -> pe=1, fep = {pos 10, beat 100}, feb = DW 0
    `SSD.corrupt_read_at_byte = 10 * 4096 + 100 * 16 + 3;
    do_read("READ 00000002", 60000, rv);
    `SSD.corrupt_read_at_byte = -1;
    ck_f(rv, "pe", 1); ck_f(rv, "fep", 40'h0A64); ck_f(rv, "feb", 0); ck_f(rv, "crc", 1); ck_f(rv, "ok", 63);
    ck(u_bfm.fld(rv, "exp") != u_bfm.fld(rv, "got"), "exp = got for a payload error");
    // 2. wrong SID: every block bad, first bad position 0
    cmd_ok("READ SID 12345678", "OK READ", r);
    ck_f(r, "rsid", 32'h12345678);
    do_read("READ 00000002", 60000, rv);
    ck_f(rv, "fbp", 0); ck_f(rv, "ok", 0); ck_f(rv, "bad", 64);
    // 3. CRC of block 5 corrupted in the SSD memory: the CRC counter and the bad block move, no payload error
    `SSD.lba_mem[5 * 8 + 7][504] = `SSD.lba_mem[5 * 8 + 7][504] ^ 8'h01;
    cmd_ok("READ SID 692D3A00", "OK READ", r);
    do_read("READ 00000002", 60000, rv);
    `SSD.lba_mem[5 * 8 + 7][504] = `SSD.lba_mem[5 * 8 + 7][504] ^ 8'h01;
    ck_f(rv, "crc", 1); ck_f(rv, "bad", 1); ck_f(rv, "pe", 0); ck_f(rv, "ok", 63); ck_f(rv, "fbp", 5);
    // 4. clean again (the contexts were cleared)
    do_read("READ 00000002", 60000, rv);
    ck_f(rv, "bad", 0); ck_f(rv, "ok", 64);
  endtask

  // ---------------------------------------------------------------- S4: overflow, RT drops
  task automatic s4_overflow();
    string r, rr, rv, l;
    int mark, idx, bad;
    longint ls, ll, drp;
    cur_scen = "S4";
    say("overflow: DDRSZ 1, SSD pause");
    setup_rec(32'h5A5A0004, 32, 32'hFFFFFFFF, 1);
    `SSD.stall_at_cmd = `SSD.io_cmd_cnt + 2;
    `SSD.stall_cycles = 500000;                          // 4 ms of SSD pause at 125 MHz
    do_rec("REC", 200000, rr, mark);
    `SSD.stall_at_cmd = -1;
    drp = u_bfm.fld(rr, "drp");
    ck(drp > 0, "no block dropped in the overflow test");
    ck_f(rr, "cer", 0);
    say($sformatf("dropped %0d blocks, wr=%0h", drp, u_bfm.fld(rr, "wr")));
    check_blocks(u_bfm.fld(rr, "wr"), 32'h5A5A0004, 1'b1, bad, ls, ll);
    ck_eq("bad blocks (seq-lost==pos chain)", bad, 0);
    ck_eq("lost of the last block = drp", ll, drp);
    do_read("READ LAST", 200000, rv);
    ck_f(rv, "bad", 0); ck_f(rv, "crc", 0); ck_f(rv, "pe", 0); ck_f(rv, "ooo", 0);
    ck_f(rv, "ok", u_bfm.fld(rr, "wr")); ck_f(rv, "lst", drp);
    ck_f(rv, "ovf", 0);
  endtask

  // ---------------------------------------------------------------- S3: profile of 3 segments x REP 2, exact ts
  task automatic s3_profile();
    string r, rr, l;
    int mark, idx, i, bad;
    longint ls, ll;
    logic [31:0] st [3];
    longint      du [3];
    longint      tsg [0:27];
    logic [31:0] acc;
    longint      nb, c, per, cc, tsb, ts0;
    int          seg_g;
    logic [127:0] b1, b2;
    cur_scen = "S3";
    say("profile 3 segments x REP 2");
    st[0] = 32'h80000000; du[0] = 5120;      // 0.5 beat/clk: 10 blocks
    st[1] = 32'h20000000; du[1] = 8192;      // 0.125 beat/clk: 4 blocks
    st[2] = 32'h00000000; du[2] = 4096;      // pause
    per = du[0] + du[1] + du[2];
    cmd_ok("SID 3C3C0003", "OK SID", r);
    cmd_ok("LEN 00000000", "OK LEN", r);
    cmd_ok("DDRSZ 0020", "OK DDRSZ", r);
    cmd_ok("PROF CLR", "OK PROF", r); ck_f(r, "pn", 0);
    cmd_err("REC PROF", 8'h08);                               // empty profile
    for (i = 0; i < 3; i++) begin
      cmd_ok($sformatf("PROF ADD %08X %010X", st[i], du[i]), "OK PROF", r);
      ck_f(r, "i", i);
    end
    cmd_ok("PROF REP 0002", "OK PROF", r); ck_f(r, "pr", 2);
    cmd_ok("PROF END STOP", "OK PROF", r); ck_f(r, "pe", 1);
    mark = u_bfm.lines.size();
    cmd_ok("PROF?", "OK PROF", r);
    ck_f(r, "pn", 3); ck_f(r, "pr", 2); ck_f(r, "pe", 1);
    ck(u_bfm.find_line("P i=02 stp=00000000 dur=0000001000", mark) >= 0, "PROF? data line of segment 2 missing");
    // golden NCO: acc starts at 0xFFFFFFFF, one add per clock, a carry = one due beat
    acc = 32'hFFFFFFFF; nb = 0; i = 0;
    for (c = 0; c < 2 * per && i < 28; c++) begin
      cc = c % per;
      seg_g = (cc < du[0]) ? 0 : (cc < du[0] + du[1]) ? 1 : 2;
      if ({1'b0, acc} + {1'b0, st[seg_g]} >= 33'h1_0000_0000) begin
        if ((nb % 256) == 0) begin tsg[nb / 256] = c; i++; end
        nb++;
      end
      acc = acc + st[seg_g];
    end
    ck_eq("golden blocks", i, 28);
    do_rec("REC PROF", 100000, rr, mark);
    check_blocks(28, 32'h3C3C0003, 1'b0, bad, ls, ll);
    ck_eq("bad blocks", bad, 0);
    b1 = ssd_beat(0, 1);
    ts0 = b1[127:64];
    for (i = 0; i < 28; i++) begin
      b1 = ssd_beat(i, 1); b2 = ssd_beat(i, 2);
      tsb = b1[127:64];
      cc = tsg[i] % per;
      seg_g = (cc < du[0]) ? 0 : (cc < du[0] + du[1]) ? 1 : 2;
      if ((tsb - ts0) !== (tsg[i] - tsg[0]))
        fail($sformatf("block %0d ts %0d, golden %0d (relative to block 0)", i, tsb - ts0, tsg[i] - tsg[0]));
      if (b2[21:16] != seg_g || b2[47:32] != (tsg[i] / per) || !b2[8 + FLG_PROF])
        fail($sformatf("block %0d seg/rep/flags %0d/%0d/%02h, golden %0d/%0d", i, b2[21:16], b2[47:32], b2[15:8], seg_g, tsg[i] / per));
    end
    // the profile ends by itself (END STOP): TRUNC + PAD to 32 blocks follow
    ck(u_bfm.fld(rr, "wr") % 32 == 0, "wr not a multiple of 32");
    ck_f(rr, "drp", 0);
  endtask


  // ---------------------------------------------------------------- generic clean REC + READ LAST (S11, S7)
  // returns the !RR line; checks counters of an undisturbed session of len_slots x 128 KiB
  task automatic rec_clean(input logic [31:0] sid, input int len_slots, input logic [31:0] step, input int ddr_mib,
                           input bit do_read_last, output string rr);
    string r, rv;
    int mark, bad;
    longint ls, ll, nblk;
    nblk = 32 * len_slots + 32;                          // source blocks + empty TRUNC + PAD up to the slot
    setup_rec(sid, len_slots, step, ddr_mib);
    do_rec("REC", 400000, rr, mark);
    ck_f(rr, "sid", sid); ck_f(rr, "wr", nblk); ck_f(rr, "drp", 0); ck_f(rr, "trc", 1); ck_f(rr, "cer", 0);
    ck_f(rr, "ssd", nblk * 4096); ck_f(rr, "cmd", len_slots + 1);
    check_blocks(nblk, sid, 1'b0, bad, ls, ll);
    ck_eq("bad blocks on SSD", bad, 0);
    if (do_read_last) begin
      do_read("READ LAST", 400000, rv);
      ck_f(rv, "ok", nblk); ck_f(rv, "bad", 0); ck_f(rv, "crc", 0); ck_f(rv, "pe", 0); ck_f(rv, "ooo", 0);
      ck_f(rv, "ovf", 0); ck_f(rv, "vb", nblk * 4096);
    end
  endtask

  // ---------------------------------------------------------------- S11: MIG ui_clk 100.3 MHz + jitter (I12)
  task automatic s11_jitter();
    string rr;
    cur_scen = "S11";
    say($sformatf("REC + READ LAST with ui_clk period %0d ps, jitter %0d ps", MIG_PERIOD_PS, MIG_JITTER));
    rec_clean(32'h0B0B0011, 64, 32'h80000000, 32, 1'b1, rr);
  endtask

  // ---------------------------------------------------------------- S7: SMART during REC
  task automatic s7_smart();
    string r, rr0, rr1, rv, l;
    int mark, idx;
    longint d0, d1;
    cur_scen = "S7";
    say("SMART 100 us during REC, baseline without");
    `SSD.smart_temp = 16'd323; `SSD.smart_spare = 8'd97; `SSD.smart_used = 8'd3; `SSD.smart_pc = 32'd9;
    cmd_ok("SMART 0000", "OK SMART", r);
    rec_clean(32'h0B0B0007, 16, 32'h80000000, 32, 1'b0, rr0);
    ck_eq("n_log without SMART", `SSD.n_log, 0);
    cmd_ok("SMART 0064", "OK SMART", r); ck_f(r, "sm", 16'h0064);
    rec_clean(32'h0B0B0107, 16, 32'h80000000, 32, 1'b0, rr1);
    cmd_ok("SMART?", "OK SMART", r);                       // counters are per session: read before the next one starts
    ck(u_bfm.fld(r, "ok") >= 10, "smo (ok=) < 10");
    ck_f(r, "er", 0); ck_f(r, "cw", 0); ck_f(r, "tk", 323); ck_f(r, "sp", 97); ck_f(r, "us", 3); ck_f(r, "pc", 9);
    ck_f(r, "poh", 123); ck_f(r, "uns", 2); ck_f(r, "rtd", 32'h000F4240);
    d0 = u_bfm.fld(rr0, "dur"); d1 = u_bfm.fld(rr1, "dur");
    say($sformatf("dur without SMART %0d, with %0d link clocks, logs %0d", d0, d1, `SSD.n_log));
    ck(`SSD.n_log >= 10, "fewer than 10 SMART logs during the recording");
    ck(d1 * 100 <= d0 * 101, $sformatf("recording %0d clocks with SMART vs %0d without: slower than 1 %%", d1, d0));
    do_read("READ LAST", 400000, rv);
    ck_f(rv, "bad", 0); ck_f(rv, "ok", 32 * 16 + 32); ck_f(rv, "ovf", 0);
    // negative: the SSD refuses the log page -> sme grows, the recording and the shutdown stay normal
    `SSD.log_status = 15'h0002;
    rec_clean(32'h0B0B0207, 16, 32'h80000000, 32, 1'b0, rr1);
    cmd_ok("SMART?", "OK SMART", r);
    `SSD.log_status = 15'h0;
    ck(u_bfm.fld(r, "er") >= 5, "sme (er=) did not grow with log_status != 0");
    cmd_ok("SMART 0000", "OK SMART", r);
  endtask

  // ---------------------------------------------------------------- S8: MON, FORMAT YES ; S10: READ END after Format
  task automatic s8_mon_format_s10();
    string r, rv, l;
    int mark, idx, w0;
    cur_scen = "S8";
    say("MON, FORMAT");
    cmd_ok("SMART 0064", "OK SMART", r);
    w0 = `SSD.n_io_writes;
    mark = u_bfm.lines.size();
    cmd_ok("MON", "OK MON", r);
    wait_ev("!S st=2 md=5", mark, 20000, l, idx);
    #(600_000.0);
    cmd_ok("STOP", "OK STOP", r);
    wait_idle(mark, 40000);
    ck_eq("data commands during MON", `SSD.n_io_writes - w0, 0);
    ck(`SSD.n_log >= 3, "no SMART log during MON");
    cmd_ok("SMART 0000", "OK SMART", r);
    ck(`SSD.lba_mem.size() > 0, "precondition: SSD holds data before Format");
    mark = u_bfm.lines.size();
    cmd_ok("FORMAT YES", "OK FORMAT", r);
    wait_ev("!RF", mark, 100000, l, idx);
    ck_f(l, "ok", 1);
    wait_idle(mark, 40000);
    ck_eq("Format NVM commands", `SSD.n_format, 1);
    ck_eq("lba_mem entries after Format", `SSD.lba_mem.size(), 0);
    cur_scen = "S10";
    say("READ END after Format");
    do_read("READ END", 100000, rv);
    ck_f(rv, "end", 1); ck_f(rv, "fbp", 0); ck_f(rv, "ok", 0);
  endtask

  // ---------------------------------------------------------------- S5: PFAIL in RUN with data in DDR, READ END
  task automatic s5_pfail();
    string r, rp, rv, l;
    int mark, idx, bad;
    longint ssd_t0, ssd_end, pnd, pad, lpos, ls, ll;
    longint t [1:6];
    cur_scen = "S5";
    say("PFAIL in RUN, DDR filling behind a paused SSD");
    setup_rec(32'h0B0B0005, 0, 32'h80000000, 4);
    `SSD.stall_at_cmd = `SSD.io_cmd_cnt + 3;
    `SSD.stall_cycles = 400000;
    mark = u_bfm.lines.size();
    cmd_ok("REC", "OK REC", r);
    wait_ev("!S st=2 md=1", mark, 40000, l, idx);
    #(1_500_000.0);
    cmd_ok("PFAIL", "OK PFAIL", r);
    ssd_t0 = dut.stat_bytes_done;
    wait_ev("!RP", mark, 100000, rp, idx);
    wait_idle(mark, 100000);
    `SSD.stall_at_cmd = -1;
    ck_f(rp, "src", 0); ck_f(rp, "inb", 0); ck_f(rp, "lat", 0); ck_f(rp, "ok", 1); ck_f(rp, "rtd", 32'h000F4240);
    ck(u_bfm.fld(rp, "pdd") > 0, "pdd (DDR fill at t0) = 0 although the SSD was paused");
    for (int k = 1; k <= 6; k++) t[k] = u_bfm.fld(rp, $sformatf("t%0d", k));
    ck(t[1] > 0 && t[1] <= t[2] && t[2] <= t[3] && t[3] <= t[4] && t[4] <= t[5] && t[5] <= t[6],
       $sformatf("phases not monotone: %0d %0d %0d %0d %0d %0d", t[1], t[2], t[3], t[4], t[5], t[6]));
    ck(t[1] < t[2], "t1 == t2: src_active did not fall at the pf (audit D4)");
    pnd = u_bfm.fld(rp, "pnd"); pad = u_bfm.fld(rp, "pad"); ssd_end = u_bfm.fld(rp, "ssd"); lpos = u_bfm.fld(rp, "lpos");
    ck(pnd > 0, "pnd = 0");
    ck((ssd_end - ssd_t0 - pad * 4096 - pnd) <= 131072 && (pnd - (ssd_end - ssd_t0 - pad * 4096)) <= 131072,
       $sformatf("pnd %0d vs ssd_end %0d - ssd(t0) %0d - pad %0d*4096", pnd, ssd_end, ssd_t0, pad));
    say($sformatf("pnd=%0d pad=%0d ssd(t0)=%0d ssd_end=%0d lpos=%0d t1..t6=%0d %0d %0d %0d %0d %0d",
                  pnd, pad, ssd_t0, ssd_end, lpos, t[1], t[2], t[3], t[4], t[5], t[6]));
    check_blocks(lpos + 1, 32'h0B0B0005, 1'b0, bad, ls, ll);
    ck_eq("bad blocks on SSD", bad, 0);
    do_read("READ END", 200000, rv);
    // the valid chain ends exactly at the session: ok = wr, first invalid block = wr, mvp = lpos (later commands read zeros = bad)
    ck_f(rv, "end", 1); ck_f(rv, "mvp", lpos); ck_f(rv, "ok", lpos + 1); ck_f(rv, "fbp", lpos + 1); ck_f(rv, "ovf", 0);
  endtask

  // ---------------------------------------------------------------- S6: PF in BRINGUP, KEY3, PF after STOP
  task automatic s6_pf_variants();
    string r, rp, rv, l;
    int mark, idx;
    cur_scen = "S6";
    say("PF in BRINGUP");
    setup_rec(32'h0B0B0006, 0, 32'h80000000, 4);
    `SSD.rdy_delay_cycles = 60000;                       // CSTS.RDY after 0.5 ms: the NVMe init of the core is still running when PFAIL arrives
    mark = u_bfm.lines.size();
    cmd_ok("REC", "OK REC", r);
    cmd_ok("PFAIL", "OK PFAIL", r);
    wait_ev("!RP", mark, 100000, rp, idx);
    wait_idle(mark, 100000);
    `SSD.rdy_delay_cycles = 200;
    ck_f(rp, "inb", 1); ck_f(rp, "src", 0); ck_f(rp, "lat", 0); ck_f(rp, "t1", 0); ck_f(rp, "t2", 0); ck_f(rp, "t3", 0);
    say("KEY3 in RUN");
    mark = u_bfm.lines.size();
    cmd_ok("REC", "OK REC", r);
    wait_ev("!S st=2 md=1", mark, 40000, l, idx);
    #(300_000.0);
    key_n[2] = 1'b0; #(2_000.0); key_n[2] = 1'b1;
    wait_ev("!RP", mark, 100000, rp, idx);
    wait_idle(mark, 100000);
    ck_f(rp, "src", 1); ck_f(rp, "inb", 0); ck_f(rp, "lat", 0); ck_f(rp, "ok", 1);
    say("PF after STOP");
    `SSD.stall_at_cmd = `SSD.io_cmd_cnt + 3;
    `SSD.stall_cycles = 400000;
    mark = u_bfm.lines.size();
    cmd_ok("REC", "OK REC", r);
    wait_ev("!S st=2 md=1", mark, 40000, l, idx);
    #(1_200_000.0);
    cmd_ok("STOP", "OK STOP", r);
    #(100_000.0);
    cmd_ok("PFAIL", "OK PFAIL", r);
    wait_ev("!RP", mark, 100000, rp, idx);
    wait_idle(mark, 100000);
    `SSD.stall_at_cmd = -1;
    ck_f(rp, "lat", 1); ck_f(rp, "inb", 0); ck_f(rp, "ok", 1);
  endtask

  // ---------------------------------------------------------------- S12: core error during REC, then a new REC (I1)
  task automatic s12_core_error();
    string r, rr, rv, l;
    int mark, idx, bad;
    longint ls, ll;
    cur_scen = "S12";
    say("io_timeout during REC -> CORE_RST -> new REC from LBA 0");
    setup_rec(32'h0B0B0012, 0, 32'h40000000, 32);
    `SSD.stall_at_cmd = `SSD.io_cmd_cnt + 4;
    `SSD.stall_cycles = 150000;                           // 1.2 ms > IO_TIMEOUT_MS of the core (0.8 ms, set by the test file), and over long before CORE_RST
    mark = u_bfm.lines.size();
    cmd_ok("REC", "OK REC", r);
    wait_ev("!E", mark, 400000, l, idx);
    ck(u_bfm.fld(l, "fl") >= 0 && (u_bfm.fld(l, "fl") & 8'h04) != 0, $sformatf("!E without io_timeout flag: [%s]", l));
    wait_ev("!RR", mark, 400000, rr, idx);
    wait_idle(mark, 400000);
    wait_ev("!S st=8", mark, 400000, l, idx);             // CORE_RST after an errored session
    `SSD.stall_at_cmd = -1;
    // let the SSD model finish its pause and the core come back
    #(500_000.0);
    ck_f(rr, "sid", 32'h0B0B0012);
    // second session
    rec_clean(32'h0B0B0112, 8, 32'h80000000, 32, 1'b1, rr);
  endtask

  // ---------------------------------------------------------------- negatives: ABORT in RUN, calib_fail
  task automatic n_abort();
    string r, rr, l;
    int mark, idx;
    cur_scen = "NEG_ABORT";
    say("ABORT in RUN, then a normal REC");
    setup_rec(32'h0B0B00AB, 0, 32'h80000000, 32);
    mark = u_bfm.lines.size();
    cmd_ok("REC", "OK REC", r);
    wait_ev("!S st=2 md=1", mark, 40000, l, idx);
    #(200_000.0);
    cmd_ok("ABORT", "OK ABORT", r);
    wait_ev("!S st=8", mark, 40000, l, idx);
    wait_idle(mark, 100000);
    rec_clean(32'h0B0B01AB, 4, 32'h80000000, 32, 1'b1, rr);
  endtask

  // link loss during RUN (audit D3): link_lost forced for 2 us (the sticky flag of link_bringup) -> ERROR -> CORE_RST -> IDLE, then a normal REC
  task automatic n_link_lost();
    string r, rr, l;
    int mark, idx;
    cur_scen = "NEG_LINK";
    say("link_lost during RUN -> ERROR -> CORE_RST, then a normal REC");
    setup_rec(32'h0B0B0071, 0, 32'h40000000, 32);
    mark = u_bfm.lines.size();
    cmd_ok("REC", "OK REC", r);
    wait_ev("!S st=2 md=1", mark, 40000, l, idx);
    #(200_000.0);
    force dut.link_lost = 1'b1;
    #(2_000.0);
    force dut.link_lost = 1'b0;                           // back to the real value before the release (xsim keeps a released variable's last value)
    #(100.0);
    release dut.link_lost;
    wait_ev("!E", mark, 40000, l, idx);
    ck_f(l, "st", 2);                                     // state at the event: RUN
    wait_ev("!S st=8", mark, 40000, l, idx);              // CORE_RST
    wait_idle(mark, 100000);
    rec_clean(32'h0B0B0171, 4, 32'h80000000, 32, 1'b1, rr);
  endtask

  task automatic n_calib_fail();
    string r, l;
    int mark, idx;
    cur_scen = "NEG_CALIB";
    say("calib_fail: REC/READ/DTEST/MEMTEST -> ERR 05, MON and FORMAT work");
    wait_ev("!B", 0, 5000, l, idx);
    ck_f(l, "cal", 0);
    cmd_ok("PING", "OK PING", r); ck_f(r, "cal", 0);
    cmd_err("REC", 8'h05);
    cmd_err("READ LAST", 8'h05);
    cmd_err("DTEST", 8'h05);
    cmd_err("MEMTEST 0001", 8'h05);
    mark = u_bfm.lines.size();
    cmd_ok("MON", "OK MON", r);
    wait_ev("!S st=2 md=5", mark, 40000, l, idx);
    cmd_ok("STOP", "OK STOP", r);
    wait_idle(mark, 40000);
    mark = u_bfm.lines.size();
    cmd_ok("FORMAT YES", "OK FORMAT", r);
    wait_ev("!RF", mark, 100000, l, idx);
    ck_f(l, "ok", 1);
    wait_idle(mark, 40000);
  endtask

  task automatic d2_debug();
    string r, rr, rv;
    int mark, bad;
    longint ls, ll;
    cur_scen = "D2";
    setup_rec(SID2, 1, 32'h80000000, 8);
    do_rec("REC", 60000, rr, mark);
    say(rr);
    check_blocks(64, SID2, 1'b0, bad, ls, ll);
    ck_eq("bad", bad, 0);
    trace_n = 40;
    do_read("READ LAST", 60000, rv);
    say(rv);
    say($sformatf("lbads=%0d cfg_ui.rsid=%h", dut.cfg_ui.lbads, dut.cfg_ui.rsid));
  endtask

  task automatic d3_debug();
    string r, rr, rv;
    int mark, idx;
    cur_scen = "D3";
    rec_clean(SID2, 2, 32'h80000000, 8, 1'b1, rr);
    do_read("READ 00000002", 60000, rv);
    setup_rec(32'h5A5A0004, 32, 32'hFFFFFFFF, 1);
    `SSD.stall_at_cmd = `SSD.io_cmd_cnt + 2;
    `SSD.stall_cycles = 500000;
    mark = u_bfm.lines.size();
    cmd_ok("REC", "OK REC", r);
    repeat (14) begin
      #(1_000_000.0);
      say($sformatf("seq=%0d admin_state=%0d in_log=%b n_log=%0d io_cmd_cnt=%0d stall_until=%0d cyc=%0d", dut.stat_seq_state,
          dut.u_host.u_core.u_admin.state, dut.u_host.u_core.u_admin.in_log, `SSD.n_log, `SSD.io_cmd_cnt, `SSD.stall_until, `SSD.cyc));
      say($sformatf("  rec st=%0d fr_done=%b fr_active=%b src_blocks=%0d wr_blocks=%0d ddr_idle=%b fill=%0d beats_in=%0d bytes_done=%0d cmds=%0d errs=%0d",
          dut.u_rec_ctrl.st_q, dut.fr_done, dut.fr_active, dut.src_blocks, dut.wr_blocks, dut.ddr_idle, dut.fill_bytes,
          dut.stat_beats_in, dut.stat_bytes_done, dut.stat_cmds_done, dut.stat_cmd_errors));
      say($sformatf("  we: enable=%b flush_req=%b read_mode=%b slot_full=%b busy=%b wait_clear=%b next_slot=%0d outst=%0d io_to=%b lba_err=%b limit=%b sub_state=%0d max_cmds=%0d sub_cnt=%0d",
          dut.u_host.u_core.u_write_engine.enable, dut.u_host.u_core.u_write_engine.flush_req, dut.u_host.u_core.u_write_engine.read_mode,
          dut.u_host.u_core.u_write_engine.slot_full, dut.u_host.u_core.u_write_engine.busy, dut.u_host.u_core.u_write_engine.wait_clear,
          dut.u_host.u_core.u_write_engine.next_slot, dut.u_host.u_core.u_write_engine.outstanding, dut.u_host.u_core.u_write_engine.io_timeout,
          dut.u_host.u_core.u_write_engine.lba_range_err, dut.u_host.u_core.u_write_engine.limit_hit, dut.u_host.u_core.u_write_engine.sub_state,
          dut.u_host.u_core.u_write_engine.max_cmds, dut.u_host.u_core.u_write_engine.sub_cnt));
    end
  endtask

  // ---------------------------------------------------------------- S13: RESET key while DDR reads are in flight (review fix A)
  // The MIG is reset only at POR; reads still in flight return after the RESET key.  Old design: rd_infl_q 0 -> -1 = 3FF,
  // ddr_idle never returns, the next MEMTEST / REC never starts (only a power cycle helped).
  // The pulse must be shorter than the MIG read latency (set to 150 cycles = 1.5 us below), otherwise the reads return
  // while the reset is still active and nothing is lost: a short key press / glitch is the case that hung the old design.
  task automatic rst_key_pulse();
    rst_key_n = 1'b0; #300; rst_key_n = 1'b1;
  endtask
  task automatic s13_reset_in_flight();
    string r, l, rr;
    int mark, idx, t;
    cur_scen = "S13";
    say("RESET key during MEMTEST read phase, then during DTEST; then MEMTEST + REC + READ must complete");
    `MIG.rd_latency = 150;
    cmd_ok("DDRSZ 0001", "OK DDRSZ", r);
    // 1. MEMTEST: reset while reads are in flight
    cmd_ok("MEMTEST 0001", "OK MEMTEST", r);
    t = 0;
    while (!(dut.u_ddr_fifo.u_ddr_memtest.st_q == 3'd2 && dut.u_ddr_fifo.u_ddr_memtest.infl_q > 8'd8) && t < 5000) begin #(1000.0); t++; end
    ck(t < 5000, "memtest never reached the read phase with reads in flight");
    say($sformatf("reset key: memtest infl=%0d", dut.u_ddr_fifo.u_ddr_memtest.infl_q));
    mark = u_bfm.lines.size();
    rst_key_pulse();
    wait_ev("!B", mark, 5000, l, idx);
    cmd_ok("TEL 0000", "OK TEL", r);                      // the default telemetry would saturate the UART
    #(20_000.0);
    ck(dut.ddr_idle === 1'b1, $sformatf("ddr_idle=%b after reset (rd_infl=%h)", dut.ddr_idle, dut.u_ddr_fifo.rd_infl_q));
    ck(dut.mt_busy === 1'b0, "memtest busy after reset");
    mark = u_bfm.lines.size();
    cmd_ok("MEMTEST 0001", "OK MEMTEST", r);
    wait_ev("!RM", mark, 100000, l, idx);
    ck_f(l, "err", 0);
    wait_idle(mark, 20000);
    // 2. DTEST (source -> DDR FIFO -> sink, no SSD): reset while the FIFO has reads in flight
    cmd_ok("SID 692D3A13", "OK SID", r);
    cmd_ok("RATE 80000000", "OK RATE", r);
    cmd_ok("DDRSZ 0004", "OK DDRSZ", r);
    cmd_ok("DTEST", "OK DTEST", r);
    t = 0;
    while (!(dut.u_ddr_fifo.rd_infl_q > 10'd8) && t < 20000) begin #(1000.0); t++; end
    ck(t < 20000, "DTEST never had DDR reads in flight");
    say($sformatf("reset key: fifo rd_infl=%0d", dut.u_ddr_fifo.rd_infl_q));
    mark = u_bfm.lines.size();
    rst_key_pulse();
    wait_ev("!B", mark, 5000, l, idx);
    cmd_ok("TEL 0000", "OK TEL", r);
    #(20_000.0);
    ck(dut.ddr_idle === 1'b1, $sformatf("ddr_idle=%b after reset (rd_infl=%h)", dut.ddr_idle, dut.u_ddr_fifo.rd_infl_q));
    ck(dut.dt_mode_q === 1'b0, "DTEST latch still set after reset");
    `MIG.rd_latency = 30;
    // 3. normal session afterwards
    rec_clean(32'h0B0B0014, 8, 32'h80000000, 32, 1'b1, rr);
    u_bfm.framing_errs = 0;                              // the RESET key cut a UART frame (!S line) in two places: expected
  endtask

  task automatic run_scenarios();
    string r;
    wait_boot();
    if (!want("S1")) cmd_ok("TEL 0000", "OK TEL", r);
    if (want("D2")) d2_debug();
    if (want("D3")) d3_debug();
    if (want("S1")) s1_ping_cfg_tel();
    if (want("S9")) s9_memtest_dtest();
    if (want("S2")) s2_rec_read();
    if (want("S4")) s4_overflow();
    if (want("S3")) s3_profile();
    if (want("S11")) s11_jitter();
    if (want("S7")) s7_smart();
    if (want("S8")) s8_mon_format_s10();
    if (want("S5")) s5_pfail();
    if (want("S6")) s6_pf_variants();
    if (want("S12")) s12_core_error();
    if (want("S13")) s13_reset_in_flight();
    if (want("NEG_ABORT")) n_abort();
    if (want("NEG_LINK")) n_link_lost();
    if (want("NEG_CALIB")) n_calib_fail();
  endtask
