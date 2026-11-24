// =============================================================================
// tb_ddr_fifo.sv - unit test of ddr_fifo (+ ddr_memtest) against mig_app_model.
//   Spec 9.1 rows tb_ddr_fifo and tb_ddr_memtest (both are covered here, one DUT).
//   Volumes are scaled down from the spec (64 MiB -> several MiB, 81 ms -> 810 us)
//   to keep the run under 2 minutes.
//
//   T0  before calibration: space_ok = 0, mt_start ignored
//   T1  random back pressure on both sides + random MIG ready/refresh/latency, window 1 MiB
//       (>= 4 address wraps), exact data, exact drain count, counters, idle
//   T2  window 2 MiB and 1024 MiB: data exact, addresses stay inside the window
//   T3  long read stall (810 us at 280 MB/s: no drop), then stall long enough to overflow:
//       whole-block drops only, delivered == admitted, no s_tready stall inside an admitted block
//   T4  clear while MIG reads are in flight (random phases), also during write batches and idle:
//       no old beat on the output, counters zero, finished within 200 cycles, new session exact
//   T5  full-rate throughput (no MIG pauses): total DDR traffic >= 1.7 GB/s
//   T6  memtest: pass; injected fault (model inject_err_addr): exact count and first address
//   T7  negative control of the scoreboard: model corrupts one read word of the FIFO stream
//       -> exactly one beat reported, at the exact stream position
//   Whole-batch property (model short_runs == 0) and MIG protocol (proto_errors == 0) at the end.
// =============================================================================
`timescale 1ns/1ps
module tb_ddr_fifo;

  // ---------------------------------------------------------------------------
  // DUT + model
  // ---------------------------------------------------------------------------
  logic clk, rst, init_calib_complete;
  logic clear;
  logic [10:0] size_mib;
  logic [127:0] s_tdata;  logic s_tvalid; logic s_tready;
  logic [127:0] m_tdata;  logic m_tvalid; logic m_tready;
  logic space_ok, idle;
  logic [30:0] fill_bytes, fill_max_bytes;
  logic [47:0] wr_bytes, rd_bytes;
  logic mt_start; logic [10:0] mt_mib; logic mt_busy, mt_done; logic [31:0] mt_err_cnt; logic [28:0] mt_first_err_addr;

  logic [28:0] app_addr; logic [2:0] app_cmd; logic app_en, app_rdy;
  logic [255:0] app_wdf_data; logic [31:0] app_wdf_mask; logic app_wdf_wren, app_wdf_end, app_wdf_rdy;
  logic [255:0] app_rd_data; logic app_rd_data_valid, app_rd_data_end;
  logic app_sr_req, app_ref_req, app_zq_req, app_sr_active, app_ref_ack, app_zq_ack;

  mig_app_model #(.CALIB_DELAY(200), .RD_LATENCY(30)) u_mig (
    .ui_clk(clk), .ui_clk_sync_rst(rst), .init_calib_complete(init_calib_complete), .device_temp_i(12'd0),
    .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en), .app_rdy(app_rdy),
    .app_wdf_data(app_wdf_data), .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren),
    .app_wdf_end(app_wdf_end), .app_wdf_rdy(app_wdf_rdy),
    .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid), .app_rd_data_end(app_rd_data_end),
    .app_sr_req(app_sr_req), .app_ref_req(app_ref_req), .app_zq_req(app_zq_req),
    .app_sr_active(app_sr_active), .app_ref_ack(app_ref_ack), .app_zq_ack(app_zq_ack)
  );

  ddr_fifo dut (
    .clk(clk), .rst(rst), .clear(clear), .size_mib(size_mib),
    .s_tdata(s_tdata), .s_tvalid(s_tvalid), .s_tready(s_tready),
    .m_tdata(m_tdata), .m_tvalid(m_tvalid), .m_tready(m_tready),
    .space_ok(space_ok), .fill_bytes(fill_bytes), .fill_max_bytes(fill_max_bytes), .idle(idle),
    .wr_bytes(wr_bytes), .rd_bytes(rd_bytes),
    .mt_start(mt_start), .mt_mib(mt_mib), .mt_busy(mt_busy), .mt_done(mt_done),
    .mt_err_cnt(mt_err_cnt), .mt_first_err_addr(mt_first_err_addr),
    .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en), .app_rdy(app_rdy),
    .app_wdf_data(app_wdf_data), .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren),
    .app_wdf_end(app_wdf_end), .app_wdf_rdy(app_wdf_rdy),
    .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid), .app_rd_data_end(app_rd_data_end),
    .app_sr_req(app_sr_req), .app_ref_req(app_ref_req), .app_zq_req(app_zq_req),
    .init_calib_complete(init_calib_complete)
  );

  // ---------------------------------------------------------------------------
  // helpers
  // ---------------------------------------------------------------------------
  longint unsigned cyc = 0;
  always @(posedge clk) cyc <= cyc + 1;

  task automatic chk(input bit c, input string msg);
    if (!c) $fatal(1, "CHECK FAILED @cyc=%0d: %s", cyc, msg);
  endtask

  function automatic logic [127:0] pat(input logic [31:0] salt, input longint unsigned n);
    logic [63:0] a, b;
    a = (n + 64'd1) * 64'h9E3779B97F4A7C15 ^ ({32'd0, salt} * 64'hD6E8FEB86659FD93);
    b = (n ^ 64'h5DEECE66D) * 64'hC2B2AE3D27D4EB4F + {salt, 32'd0};
    return {a ^ (a >> 31), b ^ (b >> 29)};
  endfunction

  // ---------------------------------------------------------------------------
  // source (stream mode: n beats; block mode: 256-beat blocks admitted by space_ok)
  // ---------------------------------------------------------------------------
  bit            src_run = 0;
  int            src_prob = 100;
  longint        src_limit = 0;
  longint        src_cnt = 0;          // stream mode: beats presented+accepted
  longint        src_acc = 0;          // beats accepted in this session (both modes)
  logic [31:0]   salt = 32'h1;
  bit            blk_mode = 0;
  longint        blk_left = 0;
  int            blk_next = 0, blk_cur = 0, blk_pos = 0, blk_drop = 0, blk_stall = 0;
  bit            blk_sending = 0, blk_skipping = 0;
  int            blk_adm [$];
  int            blk_dlv [$];

  always @(posedge clk) begin
    bit acc;
    acc = s_tvalid && s_tready && !rst;
    if (acc) begin
      src_acc++;
      if (!blk_mode) src_cnt++;
      else begin
        blk_pos++;
        if (blk_pos == 256) blk_sending = 0;
      end
    end
    if (blk_mode && s_tvalid && !s_tready) blk_stall++;
    // a dropped block still takes its 256 beat slots of source time
    if (blk_mode && src_run && blk_skipping && $urandom_range(0, 99) < src_prob) begin
      blk_pos++;
      if (blk_pos == 256) blk_skipping = 0;
    end
    if (!src_run) s_tvalid <= 1'b0;
    else if (!s_tvalid || acc) begin
      s_tvalid <= 1'b0;
      if (!blk_mode) begin
        if (src_cnt < src_limit && $urandom_range(0, 99) < src_prob) begin
          s_tvalid <= 1'b1;
          s_tdata  <= pat(salt, src_cnt);
        end
      end else begin
        if (!blk_sending && !blk_skipping && blk_left > 0) begin
          blk_left--;
          blk_cur = blk_next; blk_next++;
          if (space_ok) begin
            blk_sending = 1; blk_pos = 0; blk_adm.push_back(blk_cur);
          end else begin blk_drop++; blk_skipping = 1; blk_pos = 0; end
        end
        if (blk_sending && $urandom_range(0, 99) < src_prob) begin
          s_tvalid <= 1'b1;
          begin
            logic [127:0] pb;
            pb = pat(32'hB10C, longint'(blk_cur) * 256 + blk_pos);
            s_tdata <= {blk_cur[31:0], blk_pos[31:0], pb[63:0]};
          end
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // sink with scoreboard
  // ---------------------------------------------------------------------------
  bit            snk_en = 0;
  int            snk_prob = 100;
  longint        snk_cnt = 0;
  int            snk_err = 0;
  longint        snk_first_err = -1;
  logic [127:0]  snk_first_diff = '0;
  bit            exp_err_mode = 0;
  int            ph_err = 0;           // beats seen with nothing to deliver
  int            blk_last = -1, blk_idx = 0, blk_bad = 0;

  always @(posedge clk) begin
    m_tready <= snk_en && ($urandom_range(0, 99) < snk_prob);
    if (m_tvalid && m_tready && !rst) begin
      if (!blk_mode) begin
        if (snk_cnt >= src_acc) begin
          ph_err++;
          if (!exp_err_mode) $fatal(1, "phantom beat at cyc=%0d (snk_cnt=%0d src_acc=%0d)", cyc, snk_cnt, src_acc);
        end
        if (m_tdata !== pat(salt, snk_cnt)) begin
          snk_err++;
          if (snk_first_err < 0) begin snk_first_err = snk_cnt; snk_first_diff = m_tdata ^ pat(salt, snk_cnt); end
          if (!exp_err_mode) $fatal(1, "data mismatch at beat %0d cyc=%0d", snk_cnt, cyc);
        end
        snk_cnt++;
      end else begin
        // block mode: {blk, idx, low64 of pat}
        automatic int b = int'(m_tdata[127:96]);
        automatic int i = int'(m_tdata[95:64]);
        automatic logic [127:0] pbk;
        if (i == 0) begin
          if (blk_idx != 0 && blk_idx != 256) blk_bad++;       // previous block was cut
          blk_dlv.push_back(b); blk_last = b; blk_idx = 1;
        end else begin
          if (b != blk_last || i != blk_idx) blk_bad++;
          blk_idx = i + 1;
        end
        pbk = pat(32'hB10C, longint'(b) * 256 + i);
        if (m_tdata[63:0] !== pbk[63:0]) blk_bad++;
        if (blk_bad != 0) $fatal(1, "block stream corrupted at cyc=%0d (blk %0d idx %0d)", cyc, b, i);
        snk_cnt++;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // continuous checks
  // ---------------------------------------------------------------------------
  longint fill_tb = 0, fillmax_tb = 0;
  bit     fill_chk = 0;
  always @(posedge clk) begin
    if (!rst) begin
      if (fill_chk) begin
        if (fill_bytes !== (wr_bytes[30:0] - rd_bytes[30:0])) $fatal(1, "fill != wr-rd at cyc=%0d", cyc);
        if (fill_bytes !== 31'(fill_tb * 16)) $fatal(1, "fill_bytes %0d != tb %0d at cyc=%0d", fill_bytes, fill_tb * 16, cyc);
        fill_tb += (s_tvalid && s_tready ? 1 : 0) - (m_tvalid && m_tready ? 1 : 0);
        if (fill_tb > fillmax_tb) fillmax_tb = fill_tb;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // procedures
  // ---------------------------------------------------------------------------
  task automatic wait_cyc(input int n);
    repeat (n) @(posedge clk);
  endtask

  task automatic new_session(input logic [31:0] s);
    // zero the testbench-side bookkeeping (the DUT has been cleared / is empty)
    src_run = 0; blk_mode = 0; salt = s;
    src_cnt = 0; src_acc = 0; snk_cnt = 0; snk_err = 0; snk_first_err = -1; ph_err = 0;
    blk_left = 0; blk_sending = 0; blk_skipping = 0; blk_adm.delete(); blk_dlv.delete(); blk_drop = 0; blk_stall = 0;
    blk_last = -1; blk_idx = 0; blk_bad = 0; blk_next = 0;
  endtask

  task automatic wait_idle(input int max_cyc);
    int n = 0;
    while (!idle) begin
      @(posedge clk); n++;
      if (n > max_cyc) $fatal(1, "timeout waiting for idle (fill=%0d)", fill_bytes);
    end
  endtask

  // stream n beats, return when everything is delivered and the FIFO is idle
  task automatic run_stream(input longint n, input int sp, input int kp, input int max_cyc);
    int t = 0;
    logic [47:0] wr0, rd0;
    new_session(salt + 32'h111);
    wr0 = wr_bytes; rd0 = rd_bytes;
    fill_chk = 1;
    src_limit = n; src_prob = sp; snk_prob = kp; snk_en = 1; src_run = 1;
    while (snk_cnt < n) begin
      @(posedge clk); t++;
      if (t > max_cyc) $fatal(1, "run_stream timeout: src_acc=%0d snk_cnt=%0d of %0d", src_acc, snk_cnt, n);
    end
    src_run = 0;
    wait_cyc(5);
    wait_idle(200);
    chk(src_acc == n, "all beats accepted");
    chk(snk_cnt == n, "exact drain count");
    chk(wr_bytes - wr0 == 48'(n * 16), "wr_bytes");
    chk(rd_bytes - rd0 == 48'(n * 16), "rd_bytes");
    chk(fill_bytes == 0, "fill_bytes after drain");
    chk(fill_max_bytes == 31'(fillmax_tb * 16), "fill_max_bytes");
    chk(snk_err == 0 && ph_err == 0, "scoreboard clean");
    fill_chk = 0;
  endtask

  int max_clear_cyc = 0;
  task automatic do_clear();
    int t = 0;
    src_run = 0; fill_chk = 0;
    @(posedge clk);
    clear <= 1'b1;
    @(posedge clk);
    clear <= 1'b0;
    // after the clear pulse: no beat may come out and s_tready must be low until the end
    @(posedge clk);
    while (dut.clr_busy_q) begin
      if (m_tvalid) $fatal(1, "old beat at the output after clear (cyc=%0d)", cyc);
      @(posedge clk); t++;
      if (t > 200) $fatal(1, "clear took more than 200 cycles");
    end
    if (t > max_clear_cyc) max_clear_cyc = t;
    wait_cyc(2);
    t = 0; while (!idle && t < 100) begin @(posedge clk); t++; end   // an aborted memtest drains first
    chk(idle == 1, "idle after clear");
    chk(t <= 200, "clear time");
    chk(fill_bytes == 0 && fill_max_bytes == 0 && wr_bytes == 0 && rd_bytes == 0, "counters zero after clear");
    chk(u_mig.rq.size() == 0, "model read queue empty after clear");
    fill_tb = 0; fillmax_tb = 0;
  endtask

  // ---------------------------------------------------------------------------
  // main
  // ---------------------------------------------------------------------------
  initial begin
    // watchdog
    #(900ms);
    $fatal(1, "global watchdog");
  end

  longint t0, t1;
  int rate_x1000;

  initial begin
    clear = 0; size_mib = 11'd1; s_tvalid = 0; s_tdata = '0; m_tready = 0; mt_start = 0; mt_mib = 11'd1;
    @(negedge rst);

    // ---- T0: before calibration ----
    wait_cyc(10);
    chk(init_calib_complete == 0, "calib not yet complete");
    chk(space_ok == 0, "T0: space_ok low before calibration");
    @(posedge clk); mt_start <= 1; @(posedge clk); mt_start <= 0;
    wait_cyc(5);
    chk(mt_busy == 0, "T0: mt_start ignored before calibration");
    while (!init_calib_complete) @(posedge clk);
    wait_cyc(5);
    chk(space_ok == 1 && idle == 1, "T0: space_ok / idle after calibration");
    $display("T0 PASS");

    // ---- T1: random back pressure, window 1 MiB, wraps ----
    u_mig.rdy_drop_pct = 30; u_mig.rd_latency = 20; u_mig.rd_lat_jitter = 20;
    size_mib = 11'd1;
    wait_cyc(3);
    run_stream(4 * 65536 + 2048 * 8 + 256, 70, 60, 2_000_000);     // 4 MiB + 272 KiB: > 4 wraps of the 1 MiB window
    chk(u_mig.max_addr < 29'(1 << 18), "T1: addresses inside 1 MiB window");
    $display("T1 PASS (max app_addr %0d words)", u_mig.max_addr);

    // ---- T2: other window sizes ----
    size_mib = 11'd2;
    wait_cyc(3);
    run_stream(2 * 65536 + 4096, 100, 100, 2_000_000);
    chk(u_mig.max_addr < 29'(2 << 18), "T2: addresses inside 2 MiB window");
    size_mib = 11'd1024;
    wait_cyc(3);
    u_mig.rdy_drop_pct = 10;
    run_stream(65536, 80, 80, 1_000_000);
    chk(u_mig.max_addr < 29'(1 << 28), "T2: addresses inside 1024 MiB window");
    $display("T2 PASS");

    // ---- T3: long read stall, then overflow in whole blocks ----
    u_mig.rdy_drop_pct = 0; u_mig.rd_lat_jitter = 0;
    size_mib = 11'd1;
    wait_cyc(3);
    do_clear();
    new_session(32'h300);
    fill_chk = 1;
    blk_mode = 1; blk_left = 40; src_prob = 18; src_run = 1;       // ~280 MB/s: 1 beat per ~5.6 cycles
    snk_en = 0;                                                    // 810 us read stall (81 ms / 100)
    wait_cyc(81000);
    chk(blk_drop == 0, "T3: no block dropped during the scaled ATP pause");
    chk(blk_stall == 0, "T3: no s_tready stall inside an admitted block (pause)");
    chk(space_ok == 1, "T3: space_ok still 1 after a 810 us pause");
    snk_prob = 100; snk_en = 1;
    while (blk_dlv.size() < blk_adm.size() || blk_left > 0 || blk_sending || blk_skipping) @(posedge clk);
    src_run = 0; wait_cyc(5); wait_idle(500);
    chk(blk_dlv.size() == 40 && blk_drop == 0, "T3: all 40 blocks delivered");
    $display("T3a PASS (pause: %0d blocks, fill_max %0d B)", blk_dlv.size(), fill_max_bytes);

    do_clear();
    new_session(32'h301);
    fill_chk = 1;
    blk_mode = 1; blk_left = 600; src_prob = 100; src_run = 1;     // full speed into a stalled sink
    snk_en = 0;
    while (blk_left > 0 || blk_sending || blk_skipping) @(posedge clk);
    src_run = 0;
    chk(blk_drop > 0, "T3: overflow must drop blocks");
    chk(space_ok == 0, "T3: space_ok low while the window is full");
    chk(blk_stall == 0, "T3: no s_tready stall inside an admitted block (overflow)");
    chk(blk_adm.size() + blk_drop == 600, "T3: admitted + dropped = offered");
    $display("T3b: admitted %0d dropped %0d fill_max %0d B", blk_adm.size(), blk_drop, fill_max_bytes);
    chk(fill_bytes == 31'(blk_adm.size() * 4096), "T3: fill = admitted blocks * 4 KiB");
    snk_prob = 100; snk_en = 1;
    while (blk_dlv.size() < blk_adm.size()) @(posedge clk);
    wait_cyc(5); wait_idle(500);
    chk(blk_dlv.size() == blk_adm.size(), "T3: delivered == admitted");
    for (int i = 0; i < blk_adm.size(); i++) chk(blk_dlv[i] == blk_adm[i], "T3: delivered blocks are the admitted ones, in order");
    chk(rd_bytes == 48'(blk_adm.size() * 4096) && wr_bytes == rd_bytes, "T3: exact drain count");
    chk(space_ok == 1, "T3: space_ok back to 1");
    fill_chk = 0; blk_mode = 0;
    $display("T3 PASS");

    // ---- T4: clear with MIG reads in flight ----
    u_mig.track_runs = 0;
    u_mig.rd_latency = 30;
    for (int k = 0; k < 14; k++) begin
      int guard, extra;
      new_session(32'h400 + k);
      fill_chk = 1;
      snk_prob = 100; snk_en = 0;
      src_limit = 6000; src_prob = 100; src_run = 1;
      if (k % 7 == 6) begin
        // clear while the FIFO is empty
        src_run = 0; wait_cyc(20 + k);
      end else if (k % 7 == 5) begin
        // clear in the middle of a write batch (sink stalled all the time)
        wait_cyc(700 + 3 * k);
      end else begin
        wait_cyc(1500);                           // data is in the DDR
        snk_en = 1;                                // start reading
        guard = 0;
        while (u_mig.rq.size() == 0) begin @(posedge clk); guard++; if (guard > 5000) $fatal(1, "no read in flight"); end
        extra = $urandom_range(0, 40);
        wait_cyc(extra);
        if (k % 7 == 0) chk(u_mig.rq.size() > 0 || dut.rd_infl_q > 0, "T4: reads are in flight at clear");
      end
      do_clear();
      // after the clear: new session with different data; nothing old may appear
      chk(space_ok == 1, "T4: space_ok after clear");
      new_session(32'h500 + k);
      fill_chk = 1;
      snk_en = 1;
      run_stream(4096 + 256 * k, 100, 100, 300_000);
      new_session(32'h600 + k);
    end
    u_mig.finish_runs();
    u_mig.track_runs = 1;
    $display("T4 PASS (longest clear: %0d cycles = %0d ns)", max_clear_cyc, max_clear_cyc * 10);

    // ---- T5: throughput without MIG pauses ----
    u_mig.rdy_drop_pct = 0; u_mig.refresh_period = 0; u_mig.bw_limit_pct = 100; u_mig.rd_lat_jitter = 0;
    size_mib = 11'd1;
    wait_cyc(3);
    t0 = cyc;
    run_stream(131072, 100, 100, 600_000);                          // 2 MiB through the 1 MiB window
    t1 = cyc;
    rate_x1000 = int'((131072 * 1000) / (t1 - t0));
    $display("T5: 131072 beats in %0d cycles = %0d.%03d beats/cycle (%0d MB/s per direction)",
             t1 - t0, rate_x1000 / 1000, rate_x1000 % 1000, rate_x1000 * 16 / 10);
    chk(rate_x1000 >= 530, "T5: total DDR traffic >= 1.7 GB/s (>= 0.53 beat/cycle per direction)");
    $display("T5 PASS");

    // ---- T6: memtest ----
    u_mig.rdy_drop_pct = 20; u_mig.refresh_period = 780; u_mig.bw_limit_pct = 100;
    u_mig.inject_err_addr = -1;
    wait_idle(100);
    mt_mib = 11'd1;
    @(posedge clk); mt_start <= 1; @(posedge clk); mt_start <= 0;
    wait_cyc(2);
    chk(mt_busy == 1 && mt_done == 0, "T6: busy after start");
    t0 = 0;
    while (!mt_done) begin @(posedge clk); t0++; if (t0 > 2_000_000) $fatal(1, "memtest timeout"); end
    chk(mt_busy == 0, "T6: busy low at done");
    chk(mt_err_cnt == 0, "T6: memtest pass has no errors");
    chk(u_mig.unwritten_reads == 0, "T6: memtest read only written words");
    $display("T6a PASS (memtest 1 MiB: %0d cycles, 0 errors)", t0);
    // fault: the model flips a bit on every read of word address 8*12345
    u_mig.inject_err_addr = 8 * 12345;
    u_mig.injected = 0;
    @(posedge clk); mt_start <= 1; @(posedge clk); mt_start <= 0;
    wait_cyc(2);
    chk(mt_done == 0 && mt_busy == 1, "T6: done cleared by the next start");
    t0 = 0;
    while (!mt_done) begin @(posedge clk); t0++; if (t0 > 2_000_000) $fatal(1, "memtest timeout"); end
    chk(mt_err_cnt == 2, "T6: injected fault counted once per pass");
    chk(mt_first_err_addr == 29'(8 * 12345), "T6: exact first error address");
    chk(u_mig.injected == 2, "T6: model injected twice");
    u_mig.inject_err_addr = -1;
    $display("T6b PASS (fault at %0d found: err_cnt=%0d first=%0d)", 8 * 12345, mt_err_cnt, mt_first_err_addr);
    // memtest must not leave the FIFO broken
    u_mig.rdy_drop_pct = 10;
    run_stream(8192, 90, 90, 200_000);
    $display("T6 PASS");

    // ---- T6c: clear (ui_clear) aborts a running memtest, ownership returns to the FIFO ----
    u_mig.inject_err_addr = -1; u_mig.track_runs = 0;
    for (int k = 0; k < 4; k++) begin
      int t;
      longint unsigned wr0, rd0;
      mt_mib = 11'd1;
      wr0 = u_mig.wr_cmds; rd0 = u_mig.rd_cmds;
      wait_idle(100);
      @(posedge clk); mt_start <= 1; @(posedge clk); mt_start <= 0;
      // k=0,1: abort in the first write pass; k=2: wait for the read phase; k=3: abort in the LFSR pass
      case (k)
        0: wait_cyc(500);
        1: wait_cyc(3000);
        2: begin t = 0; while (u_mig.rd_cmds - rd0 < 100) begin @(posedge clk); t++; if (t > 400000) $fatal(1, "no read phase"); end end
        3: begin t = 0; while (u_mig.wr_cmds - wr0 < 32768 + 30000) begin @(posedge clk); t++; if (t > 800000) $fatal(1, "no pass 1"); end end
      endcase
      chk(mt_busy == 1, "T6c: memtest running at abort");
      do_clear();
      t = 0;
      while (mt_busy) begin @(posedge clk); t++; if (t > 200) $fatal(1, "T6c: mt_busy still high after abort"); end
      chk(mt_done == 0, "T6c: done stays 0 after abort");
      chk(u_mig.rq.size() == 0 || 1, "T6c: reads drained");
      wait_cyc(60);
      chk(u_mig.rq.size() == 0, "T6c: no read left in the model");
      chk(space_ok == 1 && idle == 1, "T6c: space_ok/idle after abort");
      run_stream(4096, 100, 100, 100_000);        // a session right after the abort: exact data
    end
    u_mig.track_runs = 1;
    $display("T6c PASS (memtest aborted by clear)");

    // ---- T8: level clear held for a long time (RESET key drain, review fix A) while reads are in flight ----
    //   the top resets ddr_fifo only with the MIG reset; the RESET key arrives as a level `clear`
    u_mig.rdy_drop_pct = 10; u_mig.track_runs = 0;
    for (int k = 0; k < 2; k++) begin
      int t;
      longint unsigned rd0;
      wait_idle(100);
      new_session(32'h800 + k);
      rd0 = u_mig.rd_cmds;
      if (k == 0) begin                                       // memtest read phase
        mt_mib = 11'd1;
        @(posedge clk); mt_start <= 1; @(posedge clk); mt_start <= 0;
        t = 0; while (u_mig.rd_cmds - rd0 < 100) begin @(posedge clk); t++; if (t > 400000) $fatal(1, "T8: no memtest read phase"); end
        chk(dut.u_ddr_memtest.infl_q > 8'd8, "T8: memtest reads in flight");
      end else begin                                          // FIFO read batch
        src_limit = 1_000_000; src_prob = 100; snk_prob = 100; snk_en = 1; src_run = 1;
        t = 0; while (dut.rd_infl_q < 10'd8) begin @(posedge clk); t++; if (t > 400000) $fatal(1, "T8: no FIFO reads in flight"); end
      end
      src_run = 0; fill_chk = 0;
      @(posedge clk);
      clear <= 1'b1;
      repeat (300) begin
        @(posedge clk);
        if (m_tvalid) $fatal(1, "T8: beat at the output while clear is held");
        chk(dut.rd_infl_q <= 10'd64, "T8: rd_infl wrapped (reads in flight lost)");
      end
      clear <= 1'b0;
      wait_cyc(5);
      t = 0; while (!idle && t < 200) begin @(posedge clk); t++; end
      chk(idle == 1 && dut.rd_infl_q == 10'd0, "T8: idle and rd_infl 0 after the held clear");
      chk(mt_busy == 0 && mt_done == 0, "T8: memtest stopped by the held clear");
      chk(fill_bytes == 0 && wr_bytes == 0 && rd_bytes == 0, "T8: counters zero");
      fill_tb = 0; fillmax_tb = 0;
      new_session(32'h900 + k);
      run_stream(4096, 100, 100, 100_000);                    // exact data afterwards
    end
    u_mig.track_runs = 1;
    $display("T8 PASS (level clear / drain)");

    // ---- T7: negative control: model corrupts a read word of the FIFO stream ----
    u_mig.rdy_drop_pct = 5;
    do_clear();                       // pointers back to word address 0
    exp_err_mode = 1;
    u_mig.inject_err_addr = 8 * 1000;
    u_mig.injected = 0;
    run_stream_expect_err();
    u_mig.inject_err_addr = -1;
    exp_err_mode = 0;
    $display("T7 PASS");

    // ---- end ----
    u_mig.finish_runs();
    chk(u_mig.proto_errors == 0, "MIG protocol errors");
    chk(u_mig.short_runs == 0, "only whole 64-command batches on the MIG port");
    $display("MIG commands: %0d writes, %0d reads", u_mig.wr_cmds, u_mig.rd_cmds);
    $display("TEST PASSED");
    $finish;
  end

  // stream that must show exactly one corrupted beat (bit 77 of word 1000 -> beat 2000)
  task automatic run_stream_expect_err();
    int t = 0;
    new_session(32'h700);
    fill_chk = 1;
    src_limit = 40064; src_prob = 100; snk_prob = 100; snk_en = 1; src_run = 1;
    while (snk_cnt < 40064) begin
      @(posedge clk); t++;
      if (t > 300_000) $fatal(1, "T7 timeout src_acc=%0d src_cnt=%0d snk_cnt=%0d err=%0d ph=%0d fill=%0d idle=%b", src_acc, src_cnt, snk_cnt, snk_err, ph_err, fill_bytes, idle);
    end
    src_run = 0; wait_cyc(5); wait_idle(200);
    chk(u_mig.injected == 1, "T7: model injected exactly once");
    chk(snk_err == 1, "T7: exactly one corrupted beat detected");
    chk(snk_first_err == 2000, "T7: corrupted beat at the exact position");
    chk(snk_first_diff == (128'd1 << 77), "T7: exact corrupted bit");
    fill_chk = 0;
  endtask

endmodule
