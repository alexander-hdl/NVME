// tb_xadc_mon (WP7): xadc_mon with the UNISIM XADC model and an analog stimulus file.
// Checks: decoded raw codes against known T/V (tolerance), valid, upd pulse
// shape and rate, saturation for out-of-range inputs, reset behaviour.
`timescale 1ns/1ps
module tb_xadc_mon;
  import rec_pkg::*;

  logic  clk = 1'b0, rst = 1'b1;
  xadc_t xadc;
  logic  upd;
  int    errors = 0;

  always #2.5 clk = ~clk;   // 200 MHz

  xadc_mon #(.SIM_MONITOR_FILE("../../vectors/xadc_stim.txt")) dut (
    .clk(clk), .rst(rst), .xadc(xadc), .upd(upd)
  );

  function automatic int t_code(real c);  return int'((c + 273.15) * 4096.0 / 503.975); endfunction
  function automatic int v_code(real v);  return int'(v * 4096.0 / 3.0); endfunction

  task automatic chk(string what, int got, int exp_c, int tol);
    if (got < exp_c - tol || got > exp_c + tol) begin
      $display("ERROR: %s got %0d expected %0d +-%0d", what, got, exp_c, tol);
      errors++;
    end
  endtask

  // Returns 1 if the check failed (used by the negative self-test).
  function automatic bit in_tol(int got, int exp_c, int tol);
    return (got >= exp_c - tol && got <= exp_c + tol);
  endfunction

  // upd pulse width / valid-before-upd monitor
  int upd_cnt = 0;
  time last_upd = 0;
  always @(posedge clk) begin
    if (upd && !rst) begin
      upd_cnt++;
      if (!xadc.valid) begin $display("ERROR: upd without valid"); errors++; end
      last_upd = $time;
    end
  end
  // upd must be exactly 1 clk wide
  logic upd_d = 1'b0;
  always @(posedge clk) begin
    upd_d <= upd;
    if (upd && upd_d) begin $display("ERROR: upd wider than 1 clk"); errors++; end
  end

  task automatic wait_ns(longint t_ns);
    if ($time < t_ns) #(t_ns - $time);
  endtask

  task automatic wait_upds(int n);
    repeat (n) @(posedge upd);
    @(posedge clk);
  endtask

  task automatic check_all(string tag, real tc, real vi, real va, real vb, int tol);
    chk({tag, " temp"},    xadc.temp,    t_code(tc), tol);
    chk({tag, " vccint"},  xadc.vccint,  v_code(vi), tol);
    chk({tag, " vccaux"},  xadc.vccaux,  v_code(va), tol);
    chk({tag, " vccbram"}, xadc.vccbram, v_code(vb), tol);
  endtask

  initial begin
    if (in_tol(100, 90, 3) || !in_tol(92, 90, 3)) begin
      $display("ERROR: tolerance self-test"); errors++;
    end
    repeat (20) @(posedge clk);
    rst = 1'b0;
    if (xadc.valid !== 1'b0) begin $display("ERROR: valid before first update"); errors++; end

    // phase 1 (t=0..600us): 50 C, 1.0/1.8/1.0 V.  2 updates: averaging flushed
    wait_upds(2);
    $display("phase1 t=%0t temp=%0d vint=%0d vaux=%0d vbram=%0d", $time, xadc.temp, xadc.vccint, xadc.vccaux, xadc.vccbram);
    check_all("p1", 50.0, 1.0, 1.8, 1.0, 4);
    if (!xadc.valid) begin $display("ERROR: valid not set"); errors++; end

    // phase 2 (600us..): 85 C, 0.95/1.75/0.98 V
    wait_ns(700000);
    wait_upds(3);
    $display("phase2 t=%0t temp=%0d vint=%0d vaux=%0d vbram=%0d", $time, xadc.temp, xadc.vccint, xadc.vccaux, xadc.vccbram);
    check_all("p2", 85.0, 0.95, 1.75, 0.98, 4);

    // phase 3 (1200us..): out of range -> saturate at 4095
    wait_ns(1300000);
    wait_upds(3);
    $display("phase3 t=%0t temp=%0d vint=%0d vaux=%0d vbram=%0d", $time, xadc.temp, xadc.vccint, xadc.vccaux, xadc.vccbram);
    chk("p3 sat temp",    xadc.temp,    4095, 0);
    chk("p3 sat vccint",  xadc.vccint,  4095, 0);
    chk("p3 sat vccaux",  xadc.vccaux,  4095, 0);
    chk("p3 sat vccbram", xadc.vccbram, 4095, 0);

    // phase 4 (1800us..): -40 C, 0 V -> low end
    wait_ns(1900000);
    wait_upds(3);
    $display("phase4 t=%0t temp=%0d vint=%0d vaux=%0d vbram=%0d", $time, xadc.temp, xadc.vccint, xadc.vccaux, xadc.vccbram);
    check_all("p4", -40.0, 0.0, 0.0, 0.0, 4);

    // reset in the middle: valid and upd clear, then recovery
    @(posedge clk); rst = 1'b1;
    repeat (3) @(posedge clk);
    if (xadc.valid !== 1'b0 || upd !== 1'b0) begin $display("ERROR: valid/upd not cleared by reset"); errors++; end
    rst = 1'b0;
    wait_ns(2500000);
    wait_upds(3);
    $display("phase5 t=%0t temp=%0d vint=%0d vaux=%0d vbram=%0d", $time, xadc.temp, xadc.vccint, xadc.vccaux, xadc.vccbram);
    check_all("p5", 50.0, 1.0, 1.8, 1.0, 4);
    if (!xadc.valid) begin $display("ERROR: valid not set after reset"); errors++; end
    if (upd_cnt < 10) begin $display("ERROR: too few updates %0d", upd_cnt); errors++; end

    if (errors == 0) $display("TEST PASSED");
    else $fatal(1, "tb_xadc_mon: %0d errors", errors);
    $finish;
  end

  initial begin
    #6000000;
    $fatal(1, "tb_xadc_mon: timeout");
  end
endmodule
