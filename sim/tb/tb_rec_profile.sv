// =============================================================================
// tb_rec_profile.sv - rec_profile (WP3): cycle-exact segment boundaries, REP 1/3/0, HOLD/STOP,
// 1-clock segments at full rate, const mode with rate_upd, nseg=0 / 65 (engine does not start),
// stop mid-profile, second session after clear.  The table is a real tdp_ram_2clk.
// Reference: a clock-by-clock expected trace is built from the table (build_exp) and compared
// with the outputs at every clock.  Negative control: a deliberately corrupted expectation must
// be reported (mismatch counter), and the engine must not start with nseg = 0.
// =============================================================================
`timescale 1ns/1ps

module tb_rec_profile;
  import rec_pkg::*;

  localparam int NMAX = 20000;

  logic clk = 0, rst = 1, clear = 0, start = 0, stop = 0, prof_en = 0, end_stop = 1, rate_upd = 0;
  logic [6:0]  nseg = 0;
  logic [15:0] rep  = 1;
  logic [31:0] const_step = 0;
  wire         rd_en;
  wire  [5:0]  rd_addr;
  wire  [71:0] rd_data;
  wire  [31:0] step;
  wire  [5:0]  seg_idx;
  wire  [15:0] rep_cnt;
  wire         prof_active, prof_end, prof_end_s;

  logic        tw_we = 0;
  logic [5:0]  tw_addr = 0;
  logic [71:0] tw_data = 0;

  always #5 clk = ~clk;

  tdp_ram_2clk #(.W(72), .DEPTH(64)) u_ram (
    .a_clk(clk), .a_we(tw_we), .a_addr(tw_addr), .a_wdata(tw_data), .a_rdata(),
    .b_clk(clk), .b_en(rd_en), .b_addr(rd_addr), .b_rdata(rd_data));

  rec_profile dut (
    .clk(clk), .rst(rst), .clear(clear), .start(start), .stop(stop), .prof_en(prof_en),
    .nseg(nseg), .rep(rep), .end_stop(end_stop), .const_step(const_step), .rate_upd(rate_upd),
    .rd_en(rd_en), .rd_addr(rd_addr), .rd_data(rd_data),
    .step(step), .seg_idx(seg_idx), .rep_cnt(rep_cnt), .prof_active(prof_active),
    .prof_end(prof_end), .prof_end_s(prof_end_s));

  // table copy and expected trace
  logic [31:0] t_step [64];
  logic [39:0] t_dur  [64];
  logic [31:0] exp_step [NMAX];
  logic [5:0]  exp_seg  [NMAX];
  logic [15:0] exp_rep  [NMAX];
  logic        exp_act  [NMAX];
  int          exp_len;
  int          exp_end_t;      // clock index of the prof_end pulse (-1 none)
  int          errors = 0;
  int          neg_detected = 0;

  task automatic cyc(input int n);
    repeat (n) @(negedge clk);
  endtask

  task automatic wr_entry(input int i, input logic [31:0] s, input logic [39:0] d);
    @(negedge clk);
    tw_we = 1; tw_addr = i[5:0]; tw_data = {s, d};
    t_step[i] = s; t_dur[i] = d;
    @(negedge clk);
    tw_we = 0;
  endtask

  task automatic do_clear();
    @(negedge clk); clear = 1; @(negedge clk); clear = 0;
  endtask

  // expected trace; clock 0 = first clock with step_0
  task automatic build_exp(input int ns, input int rp, input bit hold, input int nclk);
    int t, pass;
    bit fin;
    logic [31:0] last_step;
    exp_end_t = -1;
    t = 0; pass = 0; fin = 0;
    last_step = 0;
    while (t < nclk && !fin) begin
      for (int k = 0; k < ns && !fin; k++) begin
        int d;
        d = (t_dur[k] == 0) ? 1 : int'(t_dur[k]);
        for (int j = 0; j < d && t < nclk; j++) begin
          exp_step[t] = t_step[k]; exp_seg[t] = k[5:0]; exp_rep[t] = pass[15:0]; exp_act[t] = 1;
          last_step = t_step[k];
          t++;
        end
        if (k == ns - 1 && rp != 0 && pass == rp - 1) begin
          fin = 1;
          exp_end_t = t - 1;
        end
      end
      pass++;
    end
    // after the end
    if (fin) begin
      int ls; logic [5:0] sg; logic [15:0] rr;
      ls = t - 1; sg = exp_seg[ls]; rr = exp_rep[ls];
      for (int u = t; u < nclk; u++) begin
        exp_step[u] = hold ? last_step : 32'd0;
        exp_seg[u]  = sg; exp_rep[u] = rr;
        exp_act[u]  = hold;
      end
      if (hold) exp_end_t = -1;
    end
    exp_len = nclk;
  endtask

  // run: pulse start, compare nclk clocks; returns mismatches
  int mism;
  int pe_cnt;
  task automatic run_cmp(input int nclk, input bit inject, input int stop_at);
    mism = 0; pe_cnt = 0;
    if (inject) exp_step[3] = exp_step[3] ^ 32'h0000_0001;
    @(negedge clk); start = 1;
    @(negedge clk); start = 0;
    for (int t = 0; t < nclk; t++) begin
      // sampled at the negedge: values of clock t
      if (step !== exp_step[t] || seg_idx !== exp_seg[t] || rep_cnt !== exp_rep[t] ||
          prof_active !== exp_act[t]) begin
        mism++;
        if (!inject && mism < 5)
          $display("ERROR: t=%0d step %h/%h seg %0d/%0d rep %0d/%0d act %b/%b", t, step, exp_step[t],
                   seg_idx, exp_seg[t], rep_cnt, exp_rep[t], prof_active, exp_act[t]);
      end
      if (prof_end) begin
        pe_cnt++;
        if (t != exp_end_t + 1) begin
          // prof_end is registered: visible in the clock after the last clock of the last segment
          mism++;
          if (!inject) $display("ERROR: prof_end at t=%0d expected %0d", t, exp_end_t + 1);
        end
      end
      if (t == stop_at) begin stop = 1; end
      @(negedge clk);
      stop = 0;
    end
    if (exp_end_t >= 0 && pe_cnt != 1) begin mism++; if (!inject) $display("ERROR: prof_end count %0d", pe_cnt); end
    if (exp_end_t < 0 && pe_cnt != 0) begin mism++; if (!inject) $display("ERROR: unexpected prof_end"); end
  endtask

  task automatic expect_ok(input string name);
    if (mism != 0) begin errors++; $display("ERROR: %s: %0d mismatches", name, mism); end
    else $display("  %s ok", name);
  endtask

  // fill table with a given pattern
  task automatic load3(input int rp);
    wr_entry(0, 32'h1000_0000, 40'd5);
    wr_entry(1, 32'h2000_0000, 40'd1);
    wr_entry(2, 32'h0000_0000, 40'd3);
    nseg = 3; rep = rp[15:0];
  endtask

  logic [31:0] rnd = 32'hC0FFEE01;
  function automatic logic [31:0] nextr();
    rnd ^= rnd << 13; rnd ^= rnd >> 17; rnd ^= rnd << 5;
    return rnd;
  endfunction

  initial begin
    repeat (4) @(negedge clk);
    rst = 0;
    repeat (4) @(negedge clk);

    // ---- C1: 3 segments, REP 1, STOP
    $display("C1: 3 segments rep 1 STOP");
    prof_en = 1; end_stop = 1;
    load3(1); do_clear(); cyc(20);
    build_exp(3, 1, 0, 40);
    run_cmp(40, 0, -1); expect_ok("C1");
    if (!prof_end_s) begin errors++; $display("ERROR: prof_end_s not sticky"); end
    cyc(5);
    if (!prof_end_s) begin errors++; $display("ERROR: prof_end_s dropped"); end
    do_clear(); cyc(2);
    if (prof_end_s) begin errors++; $display("ERROR: prof_end_s not cleared by clear"); end

    // ---- C2: REP 3 STOP, second session after clear
    $display("C2: rep 3 STOP");
    load3(3); do_clear(); cyc(20);
    build_exp(3, 3, 0, 60);
    run_cmp(60, 0, -1); expect_ok("C2");

    // ---- C3: REP 2 HOLD
    $display("C3: rep 2 HOLD");
    end_stop = 0; load3(2); do_clear(); cyc(20);
    build_exp(3, 2, 1, 60);
    run_cmp(60, 0, -1); expect_ok("C3");
    if (prof_end_s) begin errors++; $display("ERROR: prof_end_s set in HOLD"); end

    // ---- C4: REP 0 endless, stop mid-pass
    $display("C4: rep 0 endless + stop");
    end_stop = 1; load3(0); do_clear(); cyc(20);
    build_exp(3, 0, 0, 100);
    run_cmp(95, 0, 94);
    expect_ok("C4 (until stop)");
    cyc(3);
    if (prof_active) begin errors++; $display("ERROR: prof_active after stop"); end

    // ---- C5: 64 segments, all 1 clock, random steps, REP 3 (full-rate FIFO)
    $display("C5: 64 x 1-clock segments, rep 3");
    for (int i = 0; i < 64; i++) wr_entry(i, nextr(), 40'd1);
    nseg = 64; rep = 3; do_clear(); cyc(20);
    build_exp(64, 3, 0, 220);
    run_cmp(220, 0, -1); expect_ok("C5");

    // ---- C6: nseg = 1
    $display("C6: nseg=1");
    wr_entry(0, 32'h4000_0000, 40'd1);
    nseg = 1; rep = 5; do_clear(); cyc(20);
    build_exp(1, 5, 0, 12);
    run_cmp(12, 0, -1); expect_ok("C6a (dur 1, rep 5)");
    wr_entry(0, 32'h4000_0001, 40'd7);
    nseg = 1; rep = 2; do_clear(); cyc(20);
    build_exp(1, 2, 0, 20);
    run_cmp(20, 0, -1); expect_ok("C6b (dur 7, rep 2)");

    // ---- C7: random 64 segments, dur 1..6 (and a dur=0 entry treated as 1), rep 2
    $display("C7: 64 random segments");
    for (int i = 0; i < 64; i++) wr_entry(i, nextr(), 40'(nextr() % 7));
    nseg = 64; rep = 2; do_clear(); cyc(20);
    build_exp(64, 2, 0, 900);
    run_cmp(900, 0, -1); expect_ok("C7");

    // ---- C8: long segments (dur > 2^16) and a short one
    $display("C8: long durations");
    wr_entry(0, 32'hAAAA_0001, 40'd70000);
    wr_entry(1, 32'hBBBB_0002, 40'd2);
    wr_entry(2, 32'hCCCC_0003, 40'd66000);
    nseg = 3; rep = 1; do_clear(); cyc(20);
    c8_long();

    // ---- C9: const mode, rate_upd
    $display("C9: const mode + rate_upd");
    prof_en = 0; const_step = 32'h0123_4567; do_clear(); cyc(10);
    @(negedge clk); start = 1; @(negedge clk); start = 0;
    if (step !== 32'h0123_4567 || prof_active) begin errors++; $display("ERROR: const step at start %h", step); end
    cyc(5);
    @(negedge clk); const_step = 32'h89AB_CDEF; rate_upd = 1;
    @(negedge clk); rate_upd = 0;
    if (step !== 32'h89AB_CDEF) begin errors++; $display("ERROR: rate_upd not applied (%h)", step); end
    cyc(3);
    @(negedge clk); const_step = 32'h1111_1111;          // no rate_upd: step must not change
    cyc(3);
    if (step !== 32'h89AB_CDEF) begin errors++; $display("ERROR: step changed without rate_upd"); end
    @(negedge clk); rate_upd = 1; @(negedge clk); rate_upd = 0;
    if (step !== 32'h1111_1111) begin errors++; $display("ERROR: second rate_upd"); end

    // ---- C10: start right after clear (FIFO not yet filled): profile begins late, then exact
    $display("C10: start immediately after clear");
    prof_en = 1; end_stop = 1; load3(1);
    do_clear();
    c10_late();

    // ---- negative controls
    $display("N1: corrupted expectation is detected");
    load3(1); do_clear(); cyc(20);
    build_exp(3, 1, 0, 40);
    run_cmp(40, 1, -1);
    if (mism == 0) begin errors++; $display("ERROR: injected mismatch not detected"); end
    else neg_detected++;
    cyc(5);

    $display("N2: nseg = 0 and nseg = 65: engine does not start");
    for (int k = 0; k < 2; k++) begin
      nseg = (k == 0) ? 7'd0 : 7'd65; rep = 1; do_clear(); cyc(20);
      @(negedge clk); start = 1; @(negedge clk); start = 0;
      for (int t = 0; t < 60; t++) begin
        if (prof_active || step !== 0 || prof_end) begin errors++; $display("ERROR: engine started with nseg=%0d", nseg); break; end
        @(negedge clk);
      end
    end
    neg_detected++;

    if (errors == 0 && neg_detected == 2) $display("TEST PASSED");
    else begin $display("TEST FAILED errors=%0d neg=%0d", errors, neg_detected); $fatal(1, "tb_rec_profile failed"); end
    $finish;
  end

  // long durations: compare segment boundaries analytically
  task automatic c8_long();
    @(negedge clk); start = 1; @(negedge clk); start = 0;
    // clock t=0 is now
    for (int t = 0; t < 136010; t++) begin
      logic [31:0] e; logic a; logic [5:0] sg;
      if (t < 70000) begin e = 32'hAAAA_0001; sg = 0; a = 1; end
      else if (t < 70002) begin e = 32'hBBBB_0002; sg = 1; a = 1; end
      else if (t < 136002) begin e = 32'hCCCC_0003; sg = 2; a = 1; end
      else begin e = 0; sg = 2; a = 0; end
      if (step !== e || seg_idx !== sg || prof_active !== a) begin
        errors++; $display("ERROR C8 t=%0d step %h exp %h seg %0d act %b", t, step, e, seg_idx, prof_active);
        break;
      end
      if (prof_end && t != 136002) begin errors++; $display("ERROR C8 prof_end at %0d", t); end
      @(negedge clk);
    end
    $display("  C8 done");
  endtask

  // start 1 clock after clear: wait for prof_active, then the trace must be exact
  task automatic c10_late();
    int begin_t;
    build_exp(3, 1, 0, 40);
    @(negedge clk); start = 1; @(negedge clk); start = 0;
    begin_t = -1;
    for (int w = 0; w < 20; w++) begin
      if (prof_active) begin begin_t = w; break; end
      @(negedge clk);
    end
    if (begin_t < 0) begin errors++; $display("ERROR: late start never began"); end
    else begin
      mism = 0;
      for (int t = 0; t < 40; t++) begin
        if (step !== exp_step[t] || seg_idx !== exp_seg[t] || prof_active !== exp_act[t]) mism++;
        @(negedge clk);
      end
      expect_ok($sformatf("C10 (begins %0d clocks late)", begin_t));
    end
  endtask

  initial begin
    #50ms;
    $fatal(1, "tb_rec_profile timeout");
  end

endmodule
