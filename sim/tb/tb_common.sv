// =============================================================================
// tb_common.sv — smoke/regression test of the rtl/common primitives
//   axis_skid, axis_fifo_sync, axis_fifo_async, sdp_ram, timer_ms
// Each is driven with random traffic and compared with a trivial reference.
// =============================================================================
`timescale 1ns/1ps

module tb_common;

  logic clk = 1'b0;
  logic rst = 1'b1;
  always #4 clk = ~clk;                       // 125 MHz

  int errors = 0;
  task automatic fail(input string msg);
    errors++;
    $display("[%0t] ERROR: %s", $time, msg);
    if (errors > 10) $fatal(1, "too many errors");
  endtask

  initial begin
    repeat (8) @(posedge clk);
    rst <= 1'b0;
  end

  // ===========================================================================
  // axis_skid and axis_fifo_sync: chain  src -> skid -> fifo -> skid -> sink
  // ===========================================================================
  localparam int NB = 20000;
  logic [127:0] a_d; logic [15:0] a_k; logic a_l; logic [1:0] a_u; logic a_v, a_r;
  logic [127:0] b_d; logic [15:0] b_k; logic b_l; logic [1:0] b_u; logic b_v, b_r;
  logic [127:0] c_d; logic [15:0] c_k; logic c_l; logic [1:0] c_u; logic c_v, c_r;
  logic [127:0] d_d; logic [15:0] d_k; logic d_l; logic [1:0] d_u; logic d_v, d_r;
  logic [4:0]   f_count; logic f_af;
  int   chain_mode = 0;                       // 0 random, 1 full rate
  int   src_n = 0, snk_n = 0;
  int   full_rate_cycles = 0, full_rate_beats = 0;

  axis_skid #(.W(128), .UW(2)) u_sk1 (
    .clk(clk), .rst(rst),
    .s_tdata(a_d), .s_tkeep(a_k), .s_tlast(a_l), .s_tuser(a_u), .s_tvalid(a_v), .s_tready(a_r),
    .m_tdata(b_d), .m_tkeep(b_k), .m_tlast(b_l), .m_tuser(b_u), .m_tvalid(b_v), .m_tready(b_r));

  axis_fifo_sync #(.W(128), .UW(2), .DEPTH(16)) u_ff (
    .clk(clk), .rst(rst),
    .s_tdata(b_d), .s_tkeep(b_k), .s_tlast(b_l), .s_tuser(b_u), .s_tvalid(b_v), .s_tready(b_r),
    .m_tdata(c_d), .m_tkeep(c_k), .m_tlast(c_l), .m_tuser(c_u), .m_tvalid(c_v), .m_tready(c_r),
    .count(f_count), .almost_full(f_af));

  axis_skid #(.W(128), .UW(2)) u_sk2 (
    .clk(clk), .rst(rst),
    .s_tdata(c_d), .s_tkeep(c_k), .s_tlast(c_l), .s_tuser(c_u), .s_tvalid(c_v), .s_tready(c_r),
    .m_tdata(d_d), .m_tkeep(d_k), .m_tlast(d_l), .m_tuser(d_u), .m_tvalid(d_v), .m_tready(d_r));

  function automatic logic [127:0] mkdata(input int n);
    return {32'(n), 32'(n * 7 + 1), 32'(n ^ 32'hA5A5), 32'hC0DE0000 + 32'(n)};
  endfunction

  always @(posedge clk) begin
    if (!rst) begin
      if (a_v && a_r) src_n++;
      if (!a_v || a_r) begin
        if (src_n < NB && (chain_mode == 1 || $urandom_range(0, 99) < 60)) begin
          a_v <= 1'b1; a_d <= mkdata(src_n); a_k <= 16'(src_n); a_l <= src_n[0]; a_u <= 2'(src_n);
        end else a_v <= 1'b0;
      end
      d_r <= (chain_mode == 1) ? 1'b1 : ($urandom_range(0, 99) < 55);
      if (d_v && d_r) begin
        if (d_d !== mkdata(snk_n) || d_k !== 16'(snk_n) || d_l !== snk_n[0] || d_u !== 2'(snk_n))
          fail($sformatf("chain beat %0d mismatch", snk_n));
        snk_n++;
      end
      if (f_count > 16) fail("fifo count out of range");
      if (f_af !== (f_count >= 14)) fail("almost_full wrong");
      if (chain_mode == 1 && src_n > 100 && src_n < NB - 100) begin
        full_rate_cycles++;
        if (d_v && d_r) full_rate_beats++;
      end
    end else begin
      a_v <= 1'b0; d_r <= 1'b0;
    end
  end

  initial begin
    wait (!rst);
    wait (snk_n == NB / 2);
    chain_mode = 1;                           // second half at full rate
  end

  // ===========================================================================
  // axis_fifo_async: two clock ratios
  // ===========================================================================
  logic wclk = 1'b0, rclk = 1'b0;
  always #4   wclk = ~wclk;                   // 125 MHz
  always #3.3 rclk = ~rclk;                   // ~151 MHz
  logic wrst = 1'b1, rrst = 1'b1;
  logic [63:0] aw_d, ar_d; logic aw_v, aw_r, ar_v, ar_r;
  int   aw_n = 0, ar_n = 0;
  localparam int NA = 20000;

  axis_fifo_async #(.W(64), .DEPTH(32)) u_af (
    .s_clk(wclk), .s_rst(wrst), .s_tdata(aw_d), .s_tvalid(aw_v), .s_tready(aw_r),
    .m_clk(rclk), .m_rst(rrst), .m_tdata(ar_d), .m_tvalid(ar_v), .m_tready(ar_r));

  initial begin
    aw_v = 0; ar_r = 0;
    repeat (6) @(posedge wclk); wrst <= 1'b0;
    repeat (2) @(posedge rclk); rrst <= 1'b0;
  end

  always @(posedge wclk) if (!wrst) begin
    if (aw_v && aw_r) aw_n++;
    if (!aw_v || aw_r) begin
      if (aw_n < NA && $urandom_range(0, 99) < 80) begin aw_v <= 1'b1; aw_d <= {32'hFACE0000 + 32'(aw_n), 32'(aw_n * 3)}; end
      else aw_v <= 1'b0;
    end
  end

  always @(posedge rclk) if (!rrst) begin
    ar_r <= ($urandom_range(0, 99) < 70);
    if (ar_v && ar_r) begin
      if (ar_d !== {32'hFACE0000 + 32'(ar_n), 32'(ar_n * 3)}) fail($sformatf("async fifo beat %0d mismatch", ar_n));
      ar_n++;
    end
  end

  // ===========================================================================
  // sdp_ram: random byte-enable writes and reads vs. a model
  // ===========================================================================
  localparam int RD = 64;
  logic ram_we, ram_re;
  logic [15:0] ram_be;
  logic [5:0] ram_wa, ram_ra;
  logic [127:0] ram_wd, ram_rd;
  logic [127:0] model [RD];
  logic [127:0] exp_pipe [3];
  bit           v_pipe [3];
  logic [127:0] last_rd;
  int ram_checks = 0;

  sdp_ram #(.DEPTH(RD), .W(128)) u_ram (
    .clk(clk), .wr_en(ram_we), .wr_be(ram_be), .wr_addr(ram_wa), .wr_data(ram_wd),
    .rd_en(ram_re), .rd_addr(ram_ra), .rd_data(ram_rd));

  initial begin
    for (int i = 0; i < RD; i++) model[i] = '0;
    last_rd = '0;
    for (int i = 0; i < 3; i++) begin v_pipe[i] = 0; exp_pipe[i] = '0; end
    ram_we = 0; ram_re = 0;
  end

  always @(posedge clk) begin
    if (!rst) begin
      // model of what was read 2 cycles ago (read-first semantics: old data when written simultaneously)
      if (v_pipe[1]) last_rd = exp_pipe[1];
      if (ram_rd !== last_rd) fail($sformatf("ram rd_data %h expected %h", ram_rd, last_rd));
      else ram_checks++;
      // shift the pipe with the access issued in this cycle (sampled by the RAM at this edge)
      v_pipe[1] = v_pipe[0]; exp_pipe[1] = exp_pipe[0];
      v_pipe[0] = ram_re;    exp_pipe[0] = model[ram_ra];
      if (ram_we) for (int b = 0; b < 16; b++) if (ram_be[b]) model[ram_wa][8*b +: 8] = ram_wd[8*b +: 8];
      // next stimulus
      ram_we <= ($urandom_range(0, 99) < 50);
      ram_wa <= $urandom_range(0, RD - 1);
      ram_be <= 16'($urandom);
      ram_wd <= {$urandom, $urandom, $urandom, $urandom};
      ram_re <= ($urandom_range(0, 99) < 50);
      ram_ra <= $urandom_range(0, RD - 1);
    end
  end

  // ===========================================================================
  // timer_ms
  // ===========================================================================
  logic tick_fast, tick_slow;
  timer_ms #(.CLK_HZ(125_000_000), .SIM_FAST(1)) u_tf (.clk(clk), .rst(rst), .tick_ms(tick_fast));
  timer_ms #(.CLK_HZ(4000), .SIM_FAST(0))        u_ts (.clk(clk), .rst(rst), .tick_ms(tick_slow));
  int last_f = -1, last_s = -1, cyc = 0, n_f = 0, n_s = 0;
  always @(posedge clk) begin
    cyc++;
    if (!rst) begin
      if (tick_fast) begin
        if (last_f >= 0 && cyc - last_f != 125) fail($sformatf("timer fast period %0d", cyc - last_f));
        last_f = cyc; n_f++;
      end
      if (tick_slow) begin
        if (last_s >= 0 && cyc - last_s != 4) fail($sformatf("timer slow period %0d", cyc - last_s));
        last_s = cyc; n_s++;
      end
    end
  end

  // ===========================================================================
  initial begin
    wait (snk_n == NB && ar_n == NA);
    repeat (50) @(posedge clk);
    if (full_rate_beats < full_rate_cycles - 4)
      fail($sformatf("skid+fifo chain not full rate: %0d beats in %0d cycles", full_rate_beats, full_rate_cycles));
    $display("chain: %0d beats ok, full-rate window %0d beats / %0d cycles; async fifo %0d beats; ram checks %0d; ticks fast=%0d slow=%0d",
             snk_n, full_rate_beats, full_rate_cycles, ar_n, ram_checks, n_f, n_s);
    if (n_f < 10 || n_s < 10) fail("timer did not tick");
    if (errors == 0) $display("TEST PASSED");
    else $fatal(1, "TEST FAILED: %0d errors", errors);
    $finish;
  end

  initial begin
    #50_000_000;
    $fatal(1, "TIMEOUT snk_n=%0d ar_n=%0d", snk_n, ar_n);
  end

endmodule
