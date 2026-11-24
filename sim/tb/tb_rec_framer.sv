// =============================================================================
// tb_rec_framer.sv - rec_framer + rec_crc128 + rec_sink_chk (WP3).
//   T0  CRC engine vs sim/vectors/crc_vectors.hex (11 vectors back to back, one beat per clock)
//   T1  const rate, stop in the middle of a block: block fields, ts (cycle exact vs a separate
//       NCO model), seg/rep/ddr_kib at block start, TRUNC valid, PAD to 32, LAST, seq/pos rules
//   T2  pf: PF on TRUNC/PAD/LAST only; T3 stop on a block boundary: empty TRUNC
//   T4  RT drops (space_ok falls in the middle of blocks): whole blocks, seq-lost=pos, DROP flag
//   T5  backpressure (m_tready low for long): accepted blocks intact, RT drops, BP no loss
//   T6  len_blocks: one len_reached pulse, sticky level, self-terminating; second session
//   T7  rate accuracy 50 / 280 / 800 MB/s over 1e6 clocks (exact NCO model), sink checker
//   T8  RT, empty DDR, step 0xC0000000 / 0xFFFFFFFF: 64 blocks, no drops
//   N*  negative controls: corrupted block bytes, bad CRC vector, sink checker errors
// Blocks are checked by an independent reference (bitwise zlib CRC over the captured bytes).
// Writes sim/vectors/blocks_*.hex (128-bit words, MSB first) for WP6 / tools/rec_decode.py.
// =============================================================================
`timescale 1ns/1ps

module tb_rec_framer;
  import rec_pkg::*;

  localparam int CAPN = 40000;                 // beats (156 blocks)

  logic clk = 0, rst = 1, clear = 0, start = 0, stop = 0, pf = 0, bp_mode = 0, prof_en = 0;
  logic [31:0] sid = 32'h692D_3A00;
  logic [39:0] len_blocks = 0;
  logic [31:0] step = 0;
  logic [5:0]  seg_idx = 0;
  logic [15:0] rep_cnt = 0;
  logic        space_ok = 1;
  logic [31:0] ddr_fill_kib = 0;
  wire  [127:0] m_tdata;
  wire          m_tvalid;
  logic         m_tready = 1;
  wire  [47:0]  src_blocks, wr_blocks;
  wire  [31:0]  drop_blocks;
  wire  [15:0]  pad_blocks, trunc_blocks;
  wire  [39:0]  bp_max_beats;
  wire  [15:0]  rt_stall_max;
  wire          len_reached, len_reached_s, active, done;

  always #5 clk = ~clk;

  rec_framer dut (
    .clk(clk), .rst(rst), .clear(clear), .start(start), .stop(stop), .pf(pf),
    .bp_mode(bp_mode), .prof_en(prof_en), .sid(sid), .len_blocks(len_blocks), .step(step),
    .seg_idx(seg_idx), .rep_cnt(rep_cnt), .space_ok(space_ok), .ddr_fill_kib(ddr_fill_kib),
    .m_tdata(m_tdata), .m_tvalid(m_tvalid), .m_tready(m_tready),
    .src_blocks(src_blocks), .wr_blocks(wr_blocks), .drop_blocks(drop_blocks),
    .pad_blocks(pad_blocks), .trunc_blocks(trunc_blocks), .bp_max_beats(bp_max_beats),
    .rt_stall_max(rt_stall_max), .len_reached(len_reached), .len_reached_s(len_reached_s),
    .active(active), .done(done));

  // sink checker on the live stream
  wire  [47:0] sk_beats;
  wire  [31:0] sk_err;
  wire         sk_rdy;
  rec_sink_chk u_sink (.clk(clk), .rst(rst), .clear(clear), .s_tdata(m_tdata),
                       .s_tvalid(m_tvalid && m_tready), .s_tready(sk_rdy), .exp_sid(sid),
                       .beats(sk_beats), .hdr_err(sk_err));

  // sink checker driven by the testbench (negative control)
  logic [127:0] s2_data = 0;
  logic         s2_valid = 0, s2_clear = 0;
  logic [31:0]  s2_sid = 32'h692D_3A00;
  wire  [47:0]  s2_beats;
  wire  [31:0]  s2_err;
  wire          s2_rdy;
  rec_sink_chk u_sink2 (.clk(clk), .rst(rst), .clear(s2_clear), .s_tdata(s2_data), .s_tvalid(s2_valid),
                        .s_tready(s2_rdy), .exp_sid(s2_sid), .beats(s2_beats), .hdr_err(s2_err));

  // CRC engine on its own (T0)
  logic         cr_first = 0, cr_en = 0;
  logic [127:0] cr_d = 0;
  wire  [31:0]  cr_q;
  rec_crc128 u_crc_t (.clk(clk), .first(cr_first), .en(cr_en), .d(cr_d), .crc_q(cr_q));

  int errors = 0;
  int neg_ok = 0;

  // ---------------------------------------------------------------- random
  logic [31:0] rnd = 32'h1BADB002;
  function automatic logic [31:0] nextr();
    rnd ^= rnd << 13; rnd ^= rnd >> 17; rnd ^= rnd << 5;
    return rnd;
  endfunction

  // ---------------------------------------------------------------- stream capture
  logic [127:0] cap [CAPN];
  int           cap_n = 0;
  bit           cap_en = 1;
  always @(posedge clk) begin
    if (clear) cap_n = 0;
    else if (m_tvalid && m_tready && cap_en) begin
      if (cap_n < CAPN) cap[cap_n] = m_tdata;
      cap_n = cap_n + 1;
    end
  end

  // ready driver: rdy_pct < 100 = random; rdy_low = forced low
  int rdy_pct = 100;
  bit rdy_low = 0;
  always @(negedge clk) begin
    if (rdy_low) m_tready = 0;
    else if (rdy_pct >= 100) m_tready = 1;
    else m_tready = ((nextr() % 100) < rdy_pct);
  end

  // ---------------------------------------------------------------- independent NCO model
  logic [63:0] exp_ts [4096];
  logic [5:0]  seg_hist [65536];
  logic [15:0] rep_hist [65536];
  logic [31:0] kib_hist [65536];
  bit          m_on = 0, m_frozen = 0, m_use_dut = 0;
  longint      m_cyc = 0, m_car = 0;
  logic [31:0] m_acc;
  longint      car_at_stop = 0;
  logic [32:0] m_sum;
  always @(posedge clk) begin
    if (rst || clear) begin
      m_on = 0; m_frozen = 0; m_cyc = 0; m_car = 0;
    end else if (start && !m_on) begin
      m_on = 1; m_frozen = 0; m_cyc = 0; m_car = 0; m_acc = 32'hFFFF_FFFF;
    end else if (m_on) begin
      if (stop || pf || len_reached) begin
        if (!m_frozen) car_at_stop = m_car;
        m_frozen = 1;
      end
      if (m_use_dut && !dut.nco_en) begin
        // BP wait: model follows the DUT enable (the NCO arithmetic is still checked)
      end else if (!m_frozen) begin
        m_sum = {1'b0, m_acc} + {1'b0, step};
        m_acc = m_sum[31:0];
        if (m_sum[32]) begin
          if ((m_car % 256) == 0 && (m_car / 256) < 4096) exp_ts[m_car / 256] = m_cyc;
          m_car = m_car + 1;
        end
      end
      if (m_cyc < 65536) begin
        seg_hist[m_cyc] = seg_idx; rep_hist[m_cyc] = rep_cnt; kib_hist[m_cyc] = ddr_fill_kib;
      end
      m_cyc = m_cyc + 1;
    end
  end

  // ---------------------------------------------------------------- block reference decode
  function automatic logic [7:0] cb(input int bi, input int off);
    return cap[bi * 256 + off / 16][8 * (off % 16) +: 8];
  endfunction
  function automatic logic [31:0] u32(input int bi, input int off);
    return {cb(bi, off + 3), cb(bi, off + 2), cb(bi, off + 1), cb(bi, off)};
  endfunction
  function automatic logic [63:0] u64(input int bi, input int off);
    return {u32(bi, off + 4), u32(bi, off)};
  endfunction
  function automatic logic [15:0] u16(input int bi, input int off);
    return {cb(bi, off + 1), cb(bi, off)};
  endfunction

  function automatic logic [31:0] ref_crc(input int bi);
    logic [31:0] c;
    c = 32'hFFFF_FFFF;
    for (int i = 0; i < 4080; i++) begin
      c = c ^ {24'h0, cb(bi, i)};
      for (int k = 0; k < 8; k++) c = c[0] ? ((c >> 1) ^ 32'hEDB8_8320) : (c >> 1);
    end
    return ~c;
  endfunction

  // per-block records
  logic [63:0] b_pos [CAPN/256];
  logic [63:0] b_seq [CAPN/256];
  logic [63:0] b_ts  [CAPN/256];
  logic [7:0]  b_flags [CAPN/256];
  logic [15:0] b_seg [CAPN/256];
  logic [15:0] b_rep [CAPN/256];
  logic [15:0] b_valid [CAPN/256];
  logic [31:0] b_lost [CAPN/256];
  logic [31:0] b_kib [CAPN/256];
  int          nblk = 0;
  int          chk_err = 0;

  // structural / CRC / payload check of every captured block; quiet=1 -> no messages
  task automatic check_blocks(input bit quiet);
    logic [63:0] pos, seq;
    logic [31:0] n;
    logic [127:0] eb;
    chk_err = 0;
    if (cap_n % 256 != 0) begin chk_err++; if (!quiet) $display("ERROR: %0d beats captured (not x256)", cap_n); end
    nblk = cap_n / 256;
    for (int bi = 0; bi < nblk; bi++) begin
      int e0;
      e0 = chk_err;
      pos = u64(bi, 8); seq = u64(bi, 16);
      b_pos[bi] = pos; b_seq[bi] = seq; b_ts[bi] = u64(bi, 24);
      b_flags[bi] = cb(bi, 33); b_seg[bi] = u16(bi, 34); b_rep[bi] = u16(bi, 36);
      b_valid[bi] = u16(bi, 38); b_lost[bi] = u32(bi, 40); b_kib[bi] = u32(bi, 44);
      if (u32(bi, 0) !== BLK_MAGIC) chk_err++;
      if (u32(bi, 4) !== sid) chk_err++;
      if (pos !== bi) chk_err++;
      if (cb(bi, 32) !== 8'd1) chk_err++;
      if (b_valid[bi] > 252) chk_err++;
      if (seq - b_lost[bi] !== pos) chk_err++;
      if (u32(bi, 4080) !== seq[31:0]) chk_err++;
      if (u32(bi, 4084) !== pos[31:0]) chk_err++;
      if (u32(bi, 4092) !== BLK_MAGIC_END) chk_err++;
      if (u32(bi, 4088) !== ref_crc(bi)) chk_err++;
      for (int b = 3; b <= 254; b++) begin
        n = {pos[23:0], 8'(b)};
        eb = {~n, n ^ 32'hA5A5_A5A5, n * 32'd7, n};
        if (cap[bi * 256 + b] !== eb) begin chk_err++; break; end
      end
      if (!quiet && chk_err != e0) $display("ERROR: block %0d failed %0d check(s)", bi, chk_err - e0);
    end
  endtask

  // classification of a finished session; returns natural block count in nat_cnt
  int nat_cnt, trunc_cnt, pad_cnt;
  task automatic check_session(input bit exp_pf, input bit exp_bp, input bit exp_prof,
                               input int exp_trunc_valid);
    int phase;      // 0 natural, 1 after TRUNC (PAD only)
    logic [7:0] f;
    check_blocks(0);
    errors += chk_err;
    nat_cnt = 0; trunc_cnt = 0; pad_cnt = 0; phase = 0;
    if (nblk % 32 != 0) begin errors++; $display("ERROR: %0d blocks, not a multiple of 32", nblk); end
    for (int bi = 0; bi < nblk; bi++) begin
      f = b_flags[bi];
      if (f[FLG_FIRST] !== (bi == 0)) begin errors++; $display("ERROR: FIRST flag block %0d", bi); end
      if (f[FLG_PROF] !== exp_prof || f[FLG_BP] !== exp_bp) begin errors++; $display("ERROR: PROF/BP flags block %0d", bi); end
      if (f[FLG_LAST] !== (bi == nblk - 1)) begin errors++; $display("ERROR: LAST flag block %0d", bi); end
      if (bi > 0 && f[FLG_DROP] !== (b_seq[bi] != b_seq[bi - 1] + 1)) begin errors++; $display("ERROR: DROP flag block %0d", bi); end
      if (f[FLG_PAD]) begin
        pad_cnt++;
        if (phase == 0) begin errors++; $display("ERROR: PAD before TRUNC (block %0d)", bi); end
        if (b_valid[bi] != 0 || f[FLG_TRUNC]) begin errors++; $display("ERROR: PAD fields block %0d", bi); end
      end else if (f[FLG_TRUNC]) begin
        trunc_cnt++;
        if (phase != 0) begin errors++; $display("ERROR: second TRUNC"); end
        phase = 1;
        if (exp_trunc_valid >= 0 && b_valid[bi] != exp_trunc_valid) begin
          errors++; $display("ERROR: TRUNC valid %0d expected %0d", b_valid[bi], exp_trunc_valid);
        end
      end else begin
        nat_cnt++;
        if (phase != 0) begin errors++; $display("ERROR: natural block after TRUNC (block %0d)", bi); end
        if (b_valid[bi] != 252) begin errors++; $display("ERROR: natural valid %0d block %0d", b_valid[bi], bi); end
      end
      if (f[FLG_PF] !== (exp_pf && (f[FLG_TRUNC] || f[FLG_PAD] || f[FLG_LAST]))) begin
        errors++; $display("ERROR: PF flag block %0d (flags %h)", bi, f);
      end
    end
    if (trunc_cnt != 1) begin errors++; $display("ERROR: %0d TRUNC blocks", trunc_cnt); end
    if (src_blocks != nblk + drop_blocks) begin errors++; $display("ERROR: src_blocks %0d nblk %0d drop %0d", src_blocks, nblk, drop_blocks); end
    if (wr_blocks != nblk) begin errors++; $display("ERROR: wr_blocks %0d nblk %0d", wr_blocks, nblk); end
    if (pad_blocks != pad_cnt || trunc_blocks != trunc_cnt) begin errors++; $display("ERROR: pad/trunc counters %0d/%0d vs %0d/%0d", pad_blocks, trunc_blocks, pad_cnt, trunc_cnt); end
    if (!done || active) begin errors++; $display("ERROR: done/active %b/%b", done, active); end
    if (sk_beats != cap_n || sk_err != 0) begin errors++; $display("ERROR: sink beats %0d (cap %0d) hdr_err %0d", sk_beats, cap_n, sk_err); end
  endtask

  // ts of natural / in-progress blocks against the NCO model (RT: every source block is known)
  task automatic check_ts();
    for (int bi = 0; bi < nblk; bi++) begin
      logic [7:0] f;
      f = b_flags[bi];
      if (!f[FLG_PAD] && b_valid[bi] != 0 || (f[FLG_TRUNC] && b_valid[bi] != 0)) begin
        if (b_seq[bi] < 4096 && b_ts[bi] !== exp_ts[b_seq[bi]]) begin
          errors++; $display("ERROR: ts block %0d seq %0d got %0d model %0d", bi, b_seq[bi], b_ts[bi], exp_ts[b_seq[bi]]);
        end
        if (m_cyc > 0 && b_ts[bi] < 65536) begin
          if (b_seg[bi] !== seg_hist[b_ts[bi]] || b_rep[bi] !== rep_hist[b_ts[bi]] || b_kib[bi] !== kib_hist[b_ts[bi]]) begin
            errors++; $display("ERROR: seg/rep/kib block %0d at ts %0d: %0d/%0d/%0d vs %0d/%0d/%0d", bi, b_ts[bi],
                               b_seg[bi], b_rep[bi], b_kib[bi], seg_hist[b_ts[bi]], rep_hist[b_ts[bi]], kib_hist[b_ts[bi]]);
          end
        end
      end
    end
  endtask

  // ---------------------------------------------------------------- helpers
  task automatic cyc(input int n);
    repeat (n) @(negedge clk);
  endtask

  task automatic do_clear();
    @(negedge clk); clear = 1; @(negedge clk); clear = 0;
    @(negedge clk);
  endtask

  task automatic pulse_start();
    @(negedge clk); start = 1; @(negedge clk); start = 0;
  endtask

  task automatic wait_done(input int maxc);
    int n;
    n = 0;
    while (!done && n < maxc) begin @(negedge clk); n++; end
    if (!done) begin errors++; $display("ERROR: timeout waiting for done"); end
    cyc(8);
  endtask

  task automatic dump_blocks(input string fn);
    int fd;
    fd = $fopen(fn, "w");
    if (fd == 0) begin errors++; $display("ERROR: cannot write %s", fn); return; end
    $fdisplay(fd, "// %s - generated by tb_rec_framer (WP3); %0d blocks of 256 x 128-bit words, MSB first", fn, cap_n / 256);
    for (int i = 0; i < cap_n && i < CAPN; i++) $fdisplay(fd, "%032h", cap[i]);
    $fclose(fd);
  endtask

  // random slow-changing space_ok / ready modulation
  int sp_hold = 0;
  bit sp_rand = 0;
  int sp_pct = 70;
  always @(negedge clk) begin
    if (sp_rand) begin
      if (sp_hold == 0) begin
        space_ok = ((nextr() % 100) < sp_pct);
        sp_hold = 50 + (nextr() % 1500);
      end else sp_hold--;
    end
  end

  // CRC vectors
  logic [127:0] vec [0:4095];
  int vec_n;
  task automatic t0_crc();
    int bad;
    bad = 0;
    for (int i = 0; i < 4096; i++) vec[i] = 128'bx;
    $readmemh("../../vectors/crc_vectors.hex", vec);
    vec_n = 0;
    while (vec_n < 16 && ^vec[vec_n * 256 + 255] !== 1'bx) vec_n++;
    if (vec_n < 9) begin errors++; $display("ERROR: %0d CRC vectors loaded (file has 9)", vec_n); end
    // one beat per clock, vectors back to back
    @(negedge clk);
    for (int v = 0; v < vec_n; v++) begin
      for (int b = 0; b < 255; b++) begin
        cr_first = (b == 0); cr_en = 1; cr_d = vec[v * 256 + b];
        if (b == 0 && v > 0 && ~cr_q !== vec[(v - 1) * 256 + 255][31:0]) begin
          bad++; $display("ERROR: CRC vector %0d: %h expected %h", v - 1, ~cr_q, vec[(v - 1) * 256 + 255][31:0]);
        end
        @(negedge clk);
      end
    end
    cr_en = 0; cr_first = 0;
    @(negedge clk);
    if (vec_n > 0 && ~cr_q !== vec[(vec_n - 1) * 256 + 255][31:0]) begin
      bad++; $display("ERROR: last CRC vector: %h expected %h", ~cr_q, vec[(vec_n - 1) * 256 + 255][31:0]);
    end
    errors += bad;
    $display("  T0: %0d CRC vectors at line rate, %0d errors", vec_n, bad);
    // negative control: one flipped data bit must change the CRC
    for (int b = 0; b < 255; b++) begin
      cr_first = (b == 0); cr_en = 1; cr_d = vec[b];
      if (b == 100) cr_d[37] = ~cr_d[37];
      @(negedge clk);
    end
    cr_en = 0;
    @(negedge clk);
    if (~cr_q !== vec[255][31:0]) neg_ok++;
    else begin errors++; $display("ERROR: corrupted vector produced the right CRC"); end
  endtask

  // ---------------------------------------------------------------- main
  function automatic real absr(input real x);
    return (x < 0.0) ? -x : x;
  endfunction
  longint due_dut, due_ref;
  real    rate_mb, rate_nom;
  logic [31:0] rstep [3];
  real         rmb   [3];

  initial begin
    repeat (4) @(negedge clk);
    rst = 0;
    repeat (4) @(negedge clk);

    // ---- T0
    $display("T0: CRC vectors");
    t0_crc();

    // ---- T1: const rate 0.5 beat/clk (800 MB/s), stop in the middle of a block
    $display("T1: stop in the middle of a block");
    do_clear();
    step = 32'h8000_0000; prof_en = 1; bp_mode = 0; space_ok = 1; ddr_fill_kib = 5;
    pulse_start();
    // vary seg/rep/kib while running
    for (int i = 0; i < 9; i++) begin
      cyc(150 + (nextr() % 300));
      seg_idx = nextr(); rep_cnt = nextr(); ddr_fill_kib = nextr() % 100000;
    end
    if (!active) begin errors++; $display("ERROR: active low while running"); end
    @(negedge clk); stop = 1; @(negedge clk); stop = 0;
    @(negedge clk);
    if (active || done) begin errors++; $display("ERROR: active=%b done=%b after stop (active must fall at stop, before done)", active, done); end
    wait_done(100000);
    begin
      int r;
      r = int'(car_at_stop % 256);
      $display("  carries at stop %0d (r=%0d)", car_at_stop, r);
      check_session(0, 0, 1, (r == 0) ? 0 : ((r > 3) ? r - 3 : 0));
      if (r == 0) $display("  (stop landed on a block boundary)");
    end
    check_ts();
    $display("  %0d natural + TRUNC + %0d PAD = %0d blocks", nat_cnt, pad_cnt, nblk);
    if (nat_cnt < 5) begin errors++; $display("ERROR: too few natural blocks"); end
    dump_blocks("../../vectors/blocks_stop.hex");

    // ---- T2: pf, space_ok low part of the time (drops), BP flag off
    $display("T2: pf with drops");
    do_clear();
    step = 32'h8000_0000; prof_en = 0; bp_mode = 0; space_ok = 1; sp_rand = 1; sp_pct = 60;
    pulse_start();
    cyc(9000);
    @(negedge clk); pf = 1; @(negedge clk); pf = 0;
    sp_rand = 0; space_ok = 1;
    wait_done(100000);
    check_session(1, 0, 0, -1);
    check_ts();
    $display("  %0d natural, %0d dropped, %0d blocks, src %0d", nat_cnt, drop_blocks, nblk, src_blocks);
    if (drop_blocks == 0) begin errors++; $display("ERROR: no drops generated"); end
    if (nblk != nat_cnt + 1 + pad_cnt) begin errors++; $display("ERROR: block classes"); end
    for (int bi = 1; bi < nblk; bi++)
      if (b_lost[bi] < b_lost[bi - 1]) begin errors++; $display("ERROR: lost decreases at %0d", bi); end
    dump_blocks("../../vectors/blocks_pf_drop.hex");

    // ---- T3: stop right after a block completed -> empty TRUNC
    $display("T3: stop on a block boundary");
    do_clear();
    step = 32'hC000_0000; prof_en = 0; space_ok = 1;
    pulse_start();
    cyc(3000);
    while (!dut.g_done_carry) @(negedge clk);
    @(negedge clk);              // the completing carry has been consumed
    stop = 1; @(negedge clk); stop = 0;
    wait_done(100000);
    check_session(0, 0, 0, 0);
    check_ts();
    if (dut.g_beat_q != 0) begin errors++; $display("ERROR: g_beat after stop"); end
    // trunc block must carry valid 0 and pad up to x32 (>= 1 TRUNC + pads)
    if (b_valid[nat_cnt] != 0) begin errors++; $display("ERROR: boundary TRUNC valid"); end

    // ---- T4: stop with space_ok low (stop while dropping) and stop at start
    $display("T4: stop during a dropped block / immediately after start");
    do_clear();
    step = 32'h8000_0000; space_ok = 1;
    pulse_start();
    cyc(1000);
    space_ok = 0;                    // block 2 starts at ~1024 and is dropped
    cyc(200);
    @(negedge clk); stop = 1; @(negedge clk); stop = 0;
    space_ok = 1;
    wait_done(100000);
    check_session(0, 0, 0, 0);
    do_clear();
    pulse_start();
    @(negedge clk); stop = 1; @(negedge clk); stop = 0;
    wait_done(100000);
    check_session(0, 0, 0, -1);
    if (nblk != 32) begin errors++; $display("ERROR: start+stop gave %0d blocks", nblk); end

    // ---- T5a: backpressure in RT: 15 % ready, input 0.25 beat/clk -> drops, accepted blocks intact
    $display("T5a: RT with m_tready = 15 %%");
    do_clear();
    step = 32'h4000_0000; space_ok = 1; rdy_pct = 15;
    pulse_start();
    cyc(30000);
    rdy_pct = 100;
    @(negedge clk); stop = 1; @(negedge clk); stop = 0;
    wait_done(200000);
    check_session(0, 0, 0, -1);
    check_ts();
    $display("  drops %0d, rt_stall_max %0d", drop_blocks, rt_stall_max);
    if (drop_blocks == 0 || rt_stall_max == 0) begin errors++; $display("ERROR: expected drops and rt_stall_max > 0"); end

    // ---- T5b: debt inside an accepted block: ready low for 1500 clocks in the middle of a block
    $display("T5b: long m_tready low");
    do_clear();
    step = 32'h4000_0000; space_ok = 1; rdy_pct = 100;
    pulse_start();
    cyc(2100);                       // block 1 is being sent (from ~2045)
    rdy_low = 1; cyc(1500); rdy_low = 0;
    cyc(5000);
    @(negedge clk); stop = 1; @(negedge clk); stop = 0;
    wait_done(100000);
    check_session(0, 0, 0, -1);
    check_ts();

    // ---- T5c: BP: no drops under 20 % ready and space_ok windows, source waits
    $display("T5c: BP mode");
    do_clear();
    step = 32'h8000_0000; bp_mode = 1; space_ok = 1; rdy_pct = 20; sp_rand = 1; sp_pct = 50;
    m_use_dut = 1;
    pulse_start();
    cyc(40000);
    sp_rand = 0; space_ok = 1; rdy_pct = 100;
    @(negedge clk); stop = 1; @(negedge clk); stop = 0;
    wait_done(200000);
    m_use_dut = 0;
    check_session(0, 1, 0, -1);
    $display("  natural %0d blocks, drops %0d, bp_max_beats %0d", nat_cnt, drop_blocks, bp_max_beats);
    if (drop_blocks != 0) begin errors++; $display("ERROR: BP dropped blocks"); end
    if (nat_cnt < 20) begin errors++; $display("ERROR: BP made too little progress (%0d)", nat_cnt); end
    bp_mode = 0; rdy_pct = 100;

    // ---- T6: len_blocks
    $display("T6: len_blocks");
    do_clear();
    step = 32'h8000_0000; prof_en = 1; len_blocks = 8; space_ok = 1; seg_idx = 3; rep_cnt = 2;
    pulse_start();
    begin
      int lp;
      lp = 0;
      for (int i = 0; i < 20000 && !done; i++) begin
        @(negedge clk);
        if (len_reached) lp++;
      end
      if (lp != 1) begin errors++; $display("ERROR: len_reached pulses %0d", lp); end
    end
    cyc(8);
    if (!len_reached_s) begin errors++; $display("ERROR: len_reached_s not sticky"); end
    check_session(0, 0, 1, 0);
    if (nat_cnt != 8) begin errors++; $display("ERROR: %0d natural blocks for len 8", nat_cnt); end
    check_ts();
    dump_blocks("../../vectors/blocks_len.hex");
    do_clear();
    if (len_reached_s) begin errors++; $display("ERROR: len_reached_s not cleared"); end
    // a second session after clear is identical in structure (len 40: 40 + TRUNC + 23 PAD = 64)
    len_blocks = 40; prof_en = 0; seg_idx = 0; rep_cnt = 0;
    pulse_start();
    wait_done(100000);
    check_session(0, 0, 0, 0);
    if (nat_cnt != 40 || nblk != 64) begin errors++; $display("ERROR: len 40 -> %0d natural, %0d total", nat_cnt, nblk); end
    len_blocks = 0;

    // ---- T6b: len with drops: the dropped block counts toward len
    do_clear();
    len_blocks = 12; step = 32'h8000_0000; space_ok = 1; sp_rand = 1; sp_pct = 50;
    pulse_start();
    wait_done(100000);
    sp_rand = 0; space_ok = 1;
    check_session(0, 0, 0, 0);
    if (src_blocks != nat_cnt + drop_blocks + 1 + pad_cnt || (nat_cnt + drop_blocks) != 12) begin
      errors++; $display("ERROR: len 12 with drops: nat %0d drop %0d", nat_cnt, drop_blocks);
    end
    len_blocks = 0;

    // ---- N1: corrupted blocks are detected by the checker
    $display("N1: corrupted block data");
    do_clear();
    step = 32'h8000_0000; pulse_start(); cyc(3000);
    @(negedge clk); stop = 1; @(negedge clk); stop = 0;
    wait_done(100000);
    check_session(0, 0, 0, -1);
    begin
      logic [127:0] sv;
      int idx;
      // payload bit
      idx = 1 * 256 + 100; sv = cap[idx]; cap[idx][70] = ~cap[idx][70];
      check_blocks(1); if (chk_err > 0) neg_ok++; else begin errors++; $display("ERROR: payload flip not detected"); end
      cap[idx] = sv;
      // header field (seq) -> CRC and seq_lo mismatch
      idx = 2 * 256 + 1; sv = cap[idx]; cap[idx][3] = ~cap[idx][3];
      check_blocks(1); if (chk_err > 0) neg_ok++; else begin errors++; $display("ERROR: header flip not detected"); end
      cap[idx] = sv;
      // CRC field
      idx = 3 * 256 + 255; sv = cap[idx]; cap[idx][70] = ~cap[idx][70];
      check_blocks(1); if (chk_err > 0) neg_ok++; else begin errors++; $display("ERROR: CRC flip not detected"); end
      cap[idx] = sv;
      check_blocks(1); if (chk_err != 0) begin errors++; $display("ERROR: restored data not clean"); end
    end

    // ---- N2: sink checker: 1 bad magic, 1 bad sid, 1 bad pos -> hdr_err = 3
    $display("N2: sink checker");
    s2_clear = 1; @(negedge clk); s2_clear = 0;
    for (int bi = 0; bi < 8; bi++) begin
      for (int b = 0; b < 256; b++) begin
        s2_data = cap[bi * 256 + b]; s2_valid = ((nextr() % 4) != 0) || (b == 255);
        if (b == 0 && s2_valid == 0) s2_valid = 1;
        if (b == 0 && bi == 2) s2_data[0] = ~s2_data[0];
        if (b == 0 && bi == 4) s2_data[40] = ~s2_data[40];
        if (b == 0 && bi == 6) s2_data[100] = ~s2_data[100];
        @(negedge clk);
        if (!s2_valid) begin b--; end
      end
    end
    s2_valid = 0; @(negedge clk);
    if (s2_err != 3 || s2_beats != 8 * 256) begin errors++; $display("ERROR: sink2 hdr_err %0d beats %0d", s2_err, s2_beats); end
    else neg_ok++;
    // a clean replay shows 0 errors
    s2_clear = 1; @(negedge clk); s2_clear = 0;
    for (int bi = 0; bi < 8; bi++) for (int b = 0; b < 256; b++) begin s2_data = cap[bi * 256 + b]; s2_valid = 1; @(negedge clk); end
    s2_valid = 0; @(negedge clk);
    if (s2_err != 0) begin errors++; $display("ERROR: clean replay hdr_err %0d", s2_err); end

    // ---- T7: rate accuracy
    $display("T7: rate accuracy over 1e6 clocks");
    cap_en = 0;
    rmb[0] = 50.0e6; rmb[1] = 280.0e6; rmb[2] = 800.0e6;
    for (int k = 0; k < 3; k++) begin
      real exact;
      rstep[k] = 32'($rtoi($floor(rmb[k] * 4294967296.0 / (16.0 * 100.0e6) + 0.5)));
      do_clear();
      step = rstep[k]; space_ok = 1; rdy_pct = 100; prof_en = 0; bp_mode = 0; len_blocks = 0;
      pulse_start();
      cyc(999_999);                 // the sample below is clock 1_000_000 (index 999_999)
      @(negedge clk);        // state after 1_000_000 clocks (carries of clocks 0 .. 999_999)
      due_dut = (dut.g_beat_q == 0) ? longint'(src_blocks) * 256
                                    : (longint'(src_blocks) - 1) * 256 + dut.g_beat_q;
      due_ref = (longint'(32'hFFFF_FFFF) + longint'(1_000_000) * longint'(rstep[k])) >>> 32;
      exact   = real'(1_000_000) * real'(rstep[k]) / 4294967296.0;
      rate_mb = real'(due_dut) * 16.0 * 100.0e6 / 1.0e6 / 1.0e6;
      rate_nom = rmb[k] / 1.0e6;
      $display("  step %h: due %0d (model %0d), %.4f MB/s measured (nominal %.1f)", rstep[k], due_dut, due_ref, rate_mb, rate_nom);
      if (due_dut != due_ref) begin errors++; $display("ERROR: NCO beats %0d != model %0d", due_dut, due_ref); end
      if (absr(real'(due_dut) - exact) > 1.0) begin errors++; $display("ERROR: rate error > 1 beat"); end
      if (absr(rate_mb - rate_nom) / rate_nom > 1.0e-4) begin errors++; $display("ERROR: rate deviates from nominal by > 100 ppm"); end
      // the emitted stream follows: at most 3 blocks behind, header checker clean
      if (sk_err != 0 || wr_blocks * 256 + 3 * 256 < due_dut) begin errors++; $display("ERROR: stream wr %0d sk_err %0d", wr_blocks, sk_err); end
      if (drop_blocks != 0) begin errors++; $display("ERROR: drops at %0d MB/s", int'(rate_nom)); end
    end
    cap_en = 1;

    // ---- T8: RT, empty DDR, step above 0.5 beat/clk: no drops (beat 0 of block N+1 meets a full W)
    $display("T8: RT high rate, space_ok=1, no drops");
    begin
      logic [31:0] hs [2];
      hs[0] = 32'hC000_0000; hs[1] = 32'hFFFF_FFFF;
      for (int k = 0; k < 2; k++) begin
        do_clear();
        step = hs[k]; space_ok = 1; rdy_pct = 100; prof_en = 0; bp_mode = 0; len_blocks = 64;
        pulse_start();
        wait_done(100000);
        len_blocks = 0;
        check_session(0, 0, 0, -1);
        $display("  step %h: %0d natural, %0d dropped", hs[k], nat_cnt, drop_blocks);
        if (drop_blocks != 0) begin errors++; $display("ERROR: step %h lost %0d blocks with empty DDR", hs[k], drop_blocks); end
        if (nat_cnt != 64) begin errors++; $display("ERROR: step %h natural blocks %0d != 64", hs[k], nat_cnt); end
      end
    end

    // ---- N3: a wrong rate model is detected (step off by 1 LSB over 1e6 clocks)
    $display("N3: a step off by 4096 LSB changes the model result");
    begin
      longint alt;
      alt = (longint'(32'hFFFF_FFFF) + longint'(1_000_000) * (longint'(rstep[2]) + 4096)) >>> 32;
      if (alt != due_ref) neg_ok++; else begin errors++; $display("ERROR: rate model insensitive"); end
    end

    if (errors == 0 && neg_ok >= 6) $display("TEST PASSED");
    else begin $display("TEST FAILED errors=%0d neg_ok=%0d", errors, neg_ok); $fatal(1, "tb_rec_framer failed"); end
    $finish;
  end

  initial begin
    #2000ms;
    $fatal(1, "tb_rec_framer timeout");
  end

endmodule
