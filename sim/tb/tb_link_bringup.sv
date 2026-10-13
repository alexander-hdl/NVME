// =============================================================================
// tb_link_bringup.sv - testbench for link_bringup (SIM_FAST=1: 1 "ms" = 1 us)
// Timers used: PERST_HOLD 5 us, POST_PERST 4 us, GEN2_WAIT 6 us, clock 100 MHz.
// A tiny model of the core's status outputs reacts to sys_rst_n / perst_n.
//
// Checks
//   * both resets low during reset and for >= PERST_HOLD after reset release;
//   * sys_rst_n rises BEFORE perst_n; perst_n never high while sys_rst_n low;
//   * link_ready only after user_lnk_up + POST_PERST (+ Gen2 wait rules):
//       scenario 1  Gen2 reached late (inside the wait window): ready at that moment
//       scenario 2  both sides Gen2 capable but 5 GT/s never reached: ready after
//                   POST_PERST + GEN2_WAIT, link_gen2 = 0
//       scenario 3  local core not Gen2 capable: no Gen2 wait at all
//       scenario 4  link drop in READY -> link_ready low at once, full PERST# cycle
//   * link drop before READY (during POST_PERST) restarts the sequence.
//   * link_lost: 0 until READY was reached and dropped; then sticky (also after the link retrains);
//     a drop before READY does not set it; only rst clears it.
//   * link-up timeout (second DUT, LINK_TIMEOUT 10 us, 2 retries): the PERST# cycle is re-run
//     twice, then link_fail=1 and both resets stay asserted; link_retries counts; a link that comes
//     up on a retry reaches READY and clears the retry count; rst clears link_fail.
// =============================================================================
`timescale 1ns/1ps
module tb_link_bringup;

  localparam int unsigned CLK_HZ        = 100_000_000;
  localparam int unsigned PERST_HOLD_US = 5;
  localparam int unsigned POST_US       = 4;
  localparam int unsigned GEN2_US       = 6;
  localparam real         CLK_NS        = 10.0;

  logic clk = 1'b0;
  logic rst = 1'b1;
  always #(CLK_NS/2) clk = ~clk;

  logic       user_lnk_up = 1'b0;
  logic [5:0] pl_ltssm_state = 6'h00;
  logic       pl_sel_lnk_rate = 1'b0;
  logic [1:0] pl_sel_lnk_width = 2'b00;
  logic       pl_link_gen2_cap = 1'b1;
  logic       pl_link_partner_gen2_supported = 1'b1;
  wire        perst_n, sys_rst_n, link_ready, link_gen2, link_lost, link_fail;
  wire  [1:0] link_retries;
  wire  [1:0] link_width;

  link_bringup #(
    .CLK_HZ(CLK_HZ), .PERST_HOLD_MS(PERST_HOLD_US), .POST_PERST_MS(POST_US),
    .GEN2_WAIT_MS(GEN2_US), .SIM_FAST(1'b1)
  ) dut (.*);

  // ---- second DUT: link-up timeout / retry ----
  logic rst2 = 1'b1, up2 = 1'b0;
  wire  perst2_n, sys2_n, rdy2, gen2_2, lost2, fail2;
  wire  [1:0] width2, retries2;
  link_bringup #(
    .CLK_HZ(CLK_HZ), .PERST_HOLD_MS(2), .POST_PERST_MS(2), .GEN2_WAIT_MS(2),
    .LINK_TIMEOUT_MS(10), .LINK_RETRY_MAX(2), .SIM_FAST(1'b1)
  ) dut2 (
    .clk(clk), .rst(rst2), .user_lnk_up(up2), .pl_ltssm_state(6'h16), .pl_sel_lnk_rate(1'b0),
    .pl_sel_lnk_width(2'b10), .pl_link_gen2_cap(1'b0), .pl_link_partner_gen2_supported(1'b0),
    .perst_n(perst2_n), .sys_rst_n(sys2_n), .link_ready(rdy2), .link_gen2(gen2_2), .link_width(width2),
    .link_lost(lost2), .link_fail(fail2), .link_retries(retries2));
  int n_perst2_rise = 0;
  always @(posedge perst2_n) n_perst2_rise++;

  int errors = 0;
  real t_perst_gap = 0.0;
  task automatic err(input string msg);
    errors++;
    $error("%s", msg);
  endtask

  // ---- time stamps of edges ----
  real t_sys_rise, t_sys_fall, t_perst_rise, t_perst_fall, t_rdy_rise, t_rdy_fall;
  always @(posedge sys_rst_n) t_sys_rise   = $realtime;
  always @(negedge sys_rst_n) t_sys_fall   = $realtime;
  always @(posedge perst_n)   t_perst_rise = $realtime;
  always @(negedge perst_n)   t_perst_fall = $realtime;
  always @(posedge link_ready) t_rdy_rise  = $realtime;
  always @(negedge link_ready) t_rdy_fall  = $realtime;

  real t_lnk_drop = 0.0;

  // ---- invariants ----
  always @(posedge clk) begin
    if (perst_n && !sys_rst_n) err("perst_n high while sys_rst_n is low");
    if (link_ready && !(perst_n && sys_rst_n)) err("link_ready high while a reset is asserted");
    if (link_ready && !user_lnk_up && (($realtime - t_lnk_drop) > 10*CLK_NS) && t_lnk_drop > 0)
      err("link_ready still high 10 cycles after the link dropped");
  end

  // ---- core status model: in reset the link is down ----
  always @(negedge sys_rst_n) begin
    user_lnk_up      = 1'b0;
    pl_sel_lnk_rate  = 1'b0;
    pl_ltssm_state   = 6'h00;
    pl_sel_lnk_width = 2'b00;
  end

  task automatic link_comes_up(input real delay_us);
    #(delay_us * 1000.0);
    user_lnk_up      = 1'b1;
    pl_ltssm_state   = 6'h16;      // L0
    pl_sel_lnk_rate  = 1'b0;       // trained at 2.5 GT/s first
    pl_sel_lnk_width = 2'b10;      // x4
  endtask

  task automatic link_drops();
    user_lnk_up     = 1'b0;
    pl_sel_lnk_rate = 1'b0;
    pl_ltssm_state  = 6'h0E;
    t_lnk_drop      = $realtime;
  endtask

  // one full bring-up scenario starting with the DUT in HOLD
  task automatic bring_up(input string name, input bit cap, input real gen2_after_us,
                          input real exp_min_us, input real exp_max_us, input bit exp_gen2);
    real t_lnk, d, t_p, t_r;
    pl_link_gen2_cap = cap;
    @(posedge perst_n);
    t_p = $realtime;
    // ordering and spacing of the release
    if (!(t_sys_rise < t_p)) err({name, ": sys_rst_n must rise before perst_n"});
    link_comes_up(1.0);
    t_lnk = $realtime;
    if (gen2_after_us >= 0.0) begin
      fork
        begin
          #(gen2_after_us * 1000.0);
          pl_sel_lnk_rate = 1'b1;
        end
      join_none
    end
    // must NOT be ready one cycle before the earliest allowed time
    #(exp_min_us * 1000.0 - 3*CLK_NS);
    if (link_ready !== 1'b0) err({name, ": link_ready too early"});
    @(posedge link_ready);
    t_r = $realtime;
    d = (t_r - t_lnk) / 1000.0;
    $display("[%0t] %s: link_ready %0.3f us after user_lnk_up (allowed %0.2f..%0.2f), gen2=%0b width=%0b",
             $time, name, d, exp_min_us, exp_max_us, link_gen2, link_width);
    if (d < exp_min_us || d > exp_max_us) err($sformatf("%s: link_ready at %0.3f us outside window", name, d));
    if (link_gen2 !== exp_gen2)           err({name, ": link_gen2 wrong"});
    if (link_width !== 2'b10)             err({name, ": link_width wrong"});
    t_perst_gap = t_p - t_sys_rise;
    #(2.0 * 1000.0);
    if (link_ready !== 1'b1) err({name, ": link_ready must stay high"});
  endtask

  // link drop in READY: check the complete re-reset
  task automatic drop_and_check(input string name);
    real t_s;
    link_drops();
    #(10 * CLK_NS);
    if (link_ready !== 1'b0) err({name, ": link_ready not low after link drop"});
    if (perst_n !== 1'b0 || sys_rst_n !== 1'b0) err({name, ": resets must be asserted after link drop"});
    @(posedge sys_rst_n);
    t_s = $realtime;
    if ((t_s - t_sys_fall) < PERST_HOLD_US * 1000.0)
      err($sformatf("%s: re-reset hold %0.1f ns shorter than %0d us", name, t_s - t_sys_fall, PERST_HOLD_US));
    $display("[%0t] %s: re-reset hold %0.3f us", $time, name, (t_s - t_sys_fall)/1000.0);
  endtask

  initial begin
    real t_rel, t_s;
    // ---- reset behaviour ----
    repeat (5) @(posedge clk);
    if (perst_n !== 1'b0 || sys_rst_n !== 1'b0) err("resets must be low during rst");
    @(posedge clk); rst <= 1'b0; t_rel = $realtime + CLK_NS;
    // initial hold
    @(posedge sys_rst_n);
    t_s = $realtime;
    if ((t_s - t_rel) < PERST_HOLD_US * 1000.0 - CLK_NS)
      err($sformatf("initial hold %0.1f ns shorter than %0d us", t_s - t_rel, PERST_HOLD_US));
    $display("[%0t] initial hold %0.3f us", $time, (t_s - t_rel)/1000.0);

    if (link_lost !== 1'b0 || link_fail !== 1'b0) err("link_lost/link_fail must be 0 after reset");
    // scenario 1: Gen2 shows up 7 us after link-up (after POST_PERST 4 us, before GEN2 timeout at 10 us)
    bring_up("S1 gen2-late", 1'b1, 7.0, 7.0, 7.1, 1'b1);
    if (link_lost !== 1'b0) err("S1: link_lost set although the link never dropped");
    $display("sys_rst_n -> perst_n gap %0.1f ns", t_perst_gap);
    if (t_perst_gap < 5*CLK_NS) err("perst_n released too soon after sys_rst_n");
    drop_and_check("S1 drop");
    if (link_lost !== 1'b1) err("S1: link_lost not set after a drop in READY");

    // scenario 2: Gen2 never reached although both sides claim support
    bring_up("S2 gen2-never", 1'b1, -1.0, 10.0, 10.1, 1'b0);
    drop_and_check("S2 drop");

    // scenario 3: not Gen2 capable -> only POST_PERST
    bring_up("S3 gen1-only", 1'b0, -1.0, 4.0, 4.1, 1'b0);
    drop_and_check("S3 drop");

    // scenario 5: Gen2 already there when POST_PERST ends -> ready right at POST_PERST
    bring_up("S4 gen2-early", 1'b1, 1.0, 4.0, 4.1, 1'b1);

    // scenario 6: link drop BEFORE ready restarts the whole sequence
    drop_and_check("S5 drop");
    @(posedge perst_n);
    link_comes_up(1.0);
    #(2.0 * 1000.0);                       // 2 us into POST_PERST (4 us)
    if (link_ready !== 1'b0) err("S6: link_ready must still be low");
    link_drops();
    #(10 * CLK_NS);
    if (perst_n !== 1'b0 || sys_rst_n !== 1'b0) err("S6: resets must be asserted after an early link drop");
    @(posedge sys_rst_n);
    bring_up("S6 after-restart", 1'b1, -1.0, 10.0, 10.1, 1'b0);
    if (link_lost !== 1'b1) err("S6: link_lost must stay sticky after the link retrained");
    if (link_fail !== 1'b0 || link_retries !== 2'd0) err("S6: link_fail/link_retries wrong");

    // link_lost is cleared by rst only
    @(posedge clk); rst <= 1'b1; repeat (3) @(posedge clk); rst <= 1'b0;
    @(posedge clk);
    if (link_lost !== 1'b0) err("rst must clear link_lost");

    // ---- link-up timeout: second DUT ----
    begin
      real t0, t1;
      repeat (3) @(posedge clk);
      rst2 <= 1'b0;
      // up2 stays 0: attempts 1..3 (initial + 2 retries), then FAIL
      wait (fail2 === 1'b1);
      t1 = $realtime;
      repeat (50) @(posedge clk);
      if (n_perst2_rise !== 3) err($sformatf("timeout: %0d PERST# releases, expected 3 (1 + 2 retries)", n_perst2_rise));
      if (retries2 !== 2'd2)   err($sformatf("timeout: link_retries %0d, expected 2", retries2));
      if (perst2_n !== 1'b0 || sys2_n !== 1'b0) err("timeout: both resets must stay asserted in FAIL");
      if (rdy2 !== 1'b0 || lost2 !== 1'b0) err("timeout: link_ready/link_lost must be 0");
      if (fail2 !== 1'b1) err("timeout: link_fail must stay set");
      $display("[%0t] link-up timeout: %0d PERST# releases, retries=%0d, link_fail=%0b", $time, n_perst2_rise, retries2, fail2);
      // rst clears it; this time the link comes up on the 2nd attempt
      rst2 <= 1'b1; repeat (3) @(posedge clk); rst2 <= 1'b0;
      repeat (2) @(posedge clk);
      if (fail2 !== 1'b0 || retries2 !== 2'd0) err("rst must clear link_fail / link_retries");
      n_perst2_rise = 0;
      @(posedge perst2_n);                          // first attempt: do not bring the link up
      wait (retries2 === 2'd1);                     // first timeout counted
      @(posedge perst2_n);                          // second attempt
      #(1.0 * 1000.0); up2 = 1'b1;
      wait (rdy2 === 1'b1);
      if (retries2 !== 2'd0) err("link_retries must be cleared once READY is reached");
      if (fail2 !== 1'b0)    err("link_fail must stay 0 when the link came up on a retry");
      $display("[%0t] link-up on retry: READY, retries=%0d", $time, retries2);
    end

    if (errors != 0) $fatal(1, "TEST FAILED: %0d error(s)", errors);
    $display("TEST PASSED");
    $finish;
  end

  initial begin
    #500_000;
    $fatal(1, "TEST FAILED: global watchdog");
  end
endmodule
