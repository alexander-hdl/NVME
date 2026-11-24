// =============================================================================
// tb_cdc.sv — WP0 unit test: cdc_pulse, cdc_bus, cdc_sync, tdp_ram_2clk, rec_cdc_bridge.
//
//  * tb_cdc_case (8 clock pairs 100/125/200 MHz with random phase and a slight detune so
//    that the phase relation sweeps; HOLD_CYC = ceil(6*f_src/f_dst)+2):
//      - cdc_pulse : N pulses at minimum spacing, none lost / doubled / wider than 1 cycle;
//        negative: reset of the source (also with a pulse during it), reset of the destination
//        with a pulse sent during it, stop of the destination clock -> no false pulse.
//      - cdc_bus one-shot: random loads (also while busy), every accepted load delivered once,
//        in order, whole; tag +1; busy length HOLD_CYC-1, toggles >= HOLD_CYC apart; source
//        reset keeps spacing, busy never sticks; destination reset swallows a transfer.
//      - cdc_bus REPEAT: coherent (untorn) data, non-decreasing, update rate, tag.
//      - cdc_sync: quasi-static values arrive.
//    One extra case has HOLD_CYC=2 against the contract and MUST report errors (built-in
//    negative control of the test itself).
//  * tdp_ram_2clk: latency 2 on both ports, read-first on A, concurrent A write / B read.
//  * rec_cdc_bridge: all crossings with replicated-word snapshots (any torn snapshot is
//    visible), freshness lag, parked step/start loads, pulses, age flags (1 ms), reset.
// =============================================================================
`timescale 1ns/1ps

// ---------------------------------------------------------------- clock with random phase
module tb_cdc_clk #(
  parameter real PERIOD = 10.0
) (
  input  logic run,
  output logic clk
);
  initial begin
    clk = 1'b0;
    #($urandom_range(0, 9999) * 0.001);
    forever begin
      #(PERIOD / 2.0);
      if (run) clk = ~clk;
    end
  end
endmodule

// ---------------------------------------------------------------- one clock pair
module tb_cdc_case #(
  parameter real PS = 10.0,          // source period, ns
  parameter real PD = 10.0,          // destination period, ns (nominal)
  parameter int  HOLD_FORCE = 0,     // 0: contract value
  parameter bit  EXPECT_FAIL = 1'b0,
  parameter int  NPULSE = 600,
  parameter int  NBUS = 300,
  parameter int  TREP = 6000
) (
  output logic done,
  output int   errors
);
  localparam int HOLD = (HOLD_FORCE != 0) ? HOLD_FORCE : (int'($ceil(6.0 * PD / PS)) + 2);
  localparam int GAPC = int'(3.0 * PD / PS) + 4;     // min pulse spacing in source cycles

  logic sclk, dclk;
  logic drun = 1'b1;
  tb_cdc_clk #(.PERIOD(PS))          u_sclk (.run(1'b1), .clk(sclk));
  tb_cdc_clk #(.PERIOD(PD * 1.0017)) u_dclk (.run(drun), .clk(dclk));

  int err = 0;
  assign errors = err;
  task automatic bad(input string m);
    err++;
    if (!EXPECT_FAIL && err <= 5) $display("[%0t] ERROR case PS=%0.3f PD=%0.3f: %s", $time, PS, PD, m);
  endtask

  // ------------------------------------------------------------ cdc_pulse
  logic p_srst = 1'b1, p_drst = 1'b1, p_pulse = 1'b0, p_dpulse;
  int   s_cnt = 0, d_cnt = 0, lost_ok = 0;
  logic p_last = 1'b0;

  cdc_pulse u_pulse (.src_clk(sclk), .src_rst(p_srst), .src_pulse(p_pulse),
                     .dst_clk(dclk), .dst_rst(p_drst), .dst_pulse(p_dpulse));

  always @(posedge sclk) if (p_pulse && !p_srst) s_cnt++;
  always @(posedge dclk) begin
    if (p_dpulse) d_cnt++;
    if (p_dpulse && p_last) bad("pulse wider than 1 cycle");
    p_last <= p_dpulse;
    if (d_cnt > s_cnt - lost_ok) bad("more dst pulses than src pulses (false/doubled pulse)");
  end

  task automatic send_pulse();
    @(posedge sclk); p_pulse <= 1'b1;
    @(posedge sclk); p_pulse <= 1'b0;
  endtask

  task automatic check_pulse_count(input string where);
    repeat (10) @(posedge dclk);
    if (d_cnt != s_cnt - lost_ok) bad($sformatf("%s: dst %0d != src %0d - lost %0d", where, d_cnt, s_cnt, lost_ok));
  endtask

  // ------------------------------------------------------------ cdc_bus one-shot
  logic          b1_srst = 1'b1, b1_drst = 1'b1, b1_load = 1'b0, b1_busy, b1_upd, b1_run = 1'b0, b1_man = 1'b0;
  logic [63:0]   b1_data = '0, b1_man_data = '0, b1_dout;
  logic [7:0]    b1_tag;
  int            b1_prob = 50;
  logic [63:0]   b1_q[$];
  int            b1_acc = 0, b1_del = 0;
  logic [7:0]    b1_tag_last = 8'd0;
  logic [63:0]   b1_last = '0;
  int            b1_cyc = 0, b1_last_acc_cyc = -1000, b1_busy_run = 0;
  logic          b1_srst_d = 1'b0, b1_taint = 1'b0;

  cdc_bus #(.W(64), .HOLD_CYC(HOLD), .REPEAT(1'b0)) u_bus1 (
    .src_clk(sclk), .src_rst(b1_srst), .src_data(b1_data), .src_load(b1_load), .src_busy(b1_busy),
    .dst_clk(dclk), .dst_rst(b1_drst), .dst_data(b1_dout), .dst_upd(b1_upd), .dst_tag(b1_tag));

  // stimulus: new random data every cycle, load with probability b1_prob %
  always @(posedge sclk) begin
    if (b1_run) begin
      b1_load <= ($urandom_range(0, 99) < b1_prob);
      b1_data <= {$urandom, $urandom};
    end else begin
      b1_load <= b1_man;
      b1_data <= b1_man_data;
    end
  end

  // source-side monitor (samples what the DUT samples)
  always @(posedge sclk) begin
    b1_cyc <= b1_cyc + 1;
    b1_srst_d <= b1_srst;
    if (b1_load && !b1_busy && !b1_srst) begin
      b1_q.push_back(b1_data);
      b1_acc++;
      if (b1_cyc - b1_last_acc_cyc < HOLD) bad($sformatf("toggles %0d cycles apart < HOLD_CYC", b1_cyc - b1_last_acc_cyc));
      b1_last_acc_cyc <= b1_cyc;
    end
    if (b1_srst || b1_srst_d) b1_taint = 1'b1;               // a run touched by a reset is not measured
    if (b1_busy) b1_busy_run = b1_busy_run + 1;
    else if (b1_busy_run != 0) begin
      if (b1_busy_run != HOLD - 1 && !b1_taint) bad($sformatf("busy lasted %0d cycles, expected %0d", b1_busy_run, HOLD - 1));
      b1_busy_run = 0;
      b1_taint = 1'b0;
    end
  end

  // destination-side monitor
  logic [63:0] b1_dprev = '0;
  always @(posedge dclk) begin
    b1_dprev <= b1_dout;
    if (b1_upd) begin
      if (b1_q.size() == 0) bad("bus one-shot: unexpected dst_upd");
      else begin
        automatic logic [63:0] e = b1_q.pop_front();
        if (b1_dout !== e) bad($sformatf("bus one-shot: data %h expected %h", b1_dout, e));
      end
      if (b1_tag !== b1_tag_last + 8'd1) bad($sformatf("bus tag %0d after %0d", b1_tag, b1_tag_last));
      b1_tag_last <= b1_tag;
      b1_del++;
    end else if (b1_dout !== b1_dprev && !b1_drst) bad("bus one-shot: dst_data changed without dst_upd");
    if (b1_drst) b1_tag_last <= 8'd0;
  end

  // ------------------------------------------------------------ cdc_bus REPEAT
  logic          b2_srst = 1'b1, b2_drst = 1'b1, b2_run = 1'b0, b2_busy, b2_upd;
  logic [31:0]   b2_x = 32'd0;
  logic [63:0]   b2_dout, b2_dprev = '0;
  logic [7:0]    b2_tag, b2_tag_last = 8'd0;
  logic [31:0]   b2_xlast = 32'd0;
  int            b2_cnt = 0;

  cdc_bus #(.W(64), .HOLD_CYC(HOLD), .REPEAT(1'b1)) u_bus2 (
    .src_clk(sclk), .src_rst(b2_srst), .src_data({~b2_x, b2_x}), .src_load(1'b0), .src_busy(b2_busy),
    .dst_clk(dclk), .dst_rst(b2_drst), .dst_data(b2_dout), .dst_upd(b2_upd), .dst_tag(b2_tag));

  always @(posedge sclk) if (b2_run && $urandom_range(0, 2) == 0) b2_x <= b2_x + $urandom_range(1, 5);

  always @(posedge dclk) begin
    b2_dprev <= b2_dout;
    if (b2_upd) begin
      b2_cnt++;
      if (b2_dout[63:32] !== ~b2_dout[31:0]) bad($sformatf("repeat bus torn data %h", b2_dout));
      if (b2_dout[31:0] < b2_xlast) bad("repeat bus data went backwards");
      b2_xlast <= b2_dout[31:0];
      if (b2_tag !== b2_tag_last + 8'd1) bad("repeat bus tag not +1");
      b2_tag_last <= b2_tag;
    end else if (b2_dout !== b2_dprev && !b2_drst) bad("repeat bus: data changed without dst_upd");
    if (b2_drst) begin b2_tag_last <= 8'd0; b2_xlast <= 32'd0; end
  end

  // ------------------------------------------------------------ cdc_sync
  logic [3:0] sy_d = 4'h0;
  logic [3:0] sy_q2, sy_q3;
  cdc_sync #(.W(4), .STAGES(2)) u_sync2 (.clk(dclk), .d(sy_d), .q(sy_q2));
  cdc_sync #(.W(4), .STAGES(3)) u_sync3 (.clk(dclk), .d(sy_d), .q(sy_q3));

  // ------------------------------------------------------------ sequence
  initial begin
    done = 1'b0;
    // ---- reset everything (both sides, a few cycles)
    repeat (12) @(posedge sclk);
    @(posedge dclk); p_drst <= 1'b0; b1_drst <= 1'b0; b2_drst <= 1'b0;
    repeat (6) @(posedge sclk);
    @(posedge sclk); p_srst <= 1'b0; b1_srst <= 1'b0; b2_srst <= 1'b0;
    repeat (20) @(posedge sclk);

    // ================= cdc_pulse
    for (int i = 0; i < NPULSE; i++) begin
      send_pulse();
      case ($urandom_range(0, 5))
        0:       repeat (GAPC + $urandom_range(0, 40)) @(posedge sclk);
        default: repeat (GAPC + $urandom_range(0, 2)) @(posedge sclk);
      endcase
    end
    check_pulse_count("burst");

    // source reset while idle / with a pulse during the reset: no pulse
    for (int i = 0; i < 6; i++) begin
      @(posedge sclk); p_srst <= 1'b1;
      repeat ($urandom_range(1, 6)) @(posedge sclk);
      if (i[0]) begin p_pulse <= 1'b1; @(posedge sclk); p_pulse <= 1'b0; end   // blocked by src_rst
      @(posedge sclk); p_srst <= 1'b0;
      repeat (GAPC + 10) @(posedge sclk);
      check_pulse_count("source reset");
    end
    // destination reset while idle: no pulse
    for (int i = 0; i < 6; i++) begin
      @(posedge dclk); p_drst <= 1'b1;
      repeat ($urandom_range(3, 9)) @(posedge dclk);
      @(posedge dclk); p_drst <= 1'b0;
      repeat (12) @(posedge dclk);
      check_pulse_count("destination reset");
    end
    // pulse sent during the destination reset: swallowed (copy-without-pulse contract)
    for (int i = 0; i < 6; i++) begin
      @(posedge dclk); p_drst <= 1'b1;
      repeat (2) @(posedge dclk);
      send_pulse();
      lost_ok++;
      repeat (6) @(posedge dclk);
      @(posedge dclk); p_drst <= 1'b0;
      repeat (12) @(posedge dclk);
      check_pulse_count("pulse during destination reset");
    end
    // destination clock stopped: no false pulse on restart; a pulse issued just before the
    // stop is still delivered exactly once
    for (int i = 0; i < 6; i++) begin
      if (i >= 3) send_pulse();
      @(posedge sclk);
      drun = 1'b0;
      #(30.0 * PD);
      drun = 1'b1;
      repeat (14) @(posedge dclk);
      check_pulse_count("destination clock stop");
    end

    // ================= cdc_bus one-shot
    b1_run = 1'b1;
    for (int ph = 0; ph < 3; ph++) begin
      b1_prob = (ph == 0) ? 100 : (ph == 1) ? 30 : 4;
      begin
        automatic int target = b1_acc + NBUS / 3;
        while (b1_acc < target) @(posedge sclk);
      end
    end
    // source reset in the middle of traffic
    b1_prob = 50;
    for (int i = 0; i < 8; i++) begin
      repeat ($urandom_range(5, 60)) @(posedge sclk);
      @(posedge sclk); b1_srst <= 1'b1;
      repeat ($urandom_range(1, 5)) @(posedge sclk);
      @(posedge sclk); b1_srst <= 1'b0;
    end
    repeat (HOLD + 4) @(posedge sclk);
    b1_srst <= 1'b0;
    // busy must come back to 0 after a reset (not stick)
    b1_run = 1'b0;
    repeat (2 * HOLD + 4) @(posedge sclk);
    if (b1_busy) bad("busy stuck after reset");
    repeat (60) @(posedge dclk);
    if (b1_q.size() != 0) bad($sformatf("bus one-shot: %0d accepted transfers not delivered", b1_q.size()));
    if (b1_acc != b1_del) bad($sformatf("bus one-shot: accepted %0d delivered %0d", b1_acc, b1_del));
    // destination reset swallows a transfer sent during it, nothing appears afterwards
    for (int i = 0; i < 4; i++) begin
      @(posedge dclk); b1_drst <= 1'b1;
      repeat (2) @(posedge dclk);
      @(posedge sclk); b1_man <= 1'b1; b1_man_data <= {$urandom, $urandom};
      @(posedge sclk); @(posedge sclk); b1_man <= 1'b0;
      b1_q.delete();                              // that transfer must NOT arrive
      repeat (int'(HOLD * PS / PD) + 10) @(posedge dclk);
      @(posedge dclk); b1_drst <= 1'b0;
      repeat (HOLD + 20) @(posedge dclk);
      if (b1_q.size() != 0) bad("bus one-shot: queue not empty");
    end
    b1_acc = 0; b1_del = 0;
    // normal traffic after the destination reset: tag restarts, everything delivered
    b1_run = 1'b1; b1_prob = 40;
    begin
      automatic int target = b1_acc + 60;
      while (b1_acc < target) @(posedge sclk);
    end
    b1_run = 1'b0;
    repeat (HOLD + 40) @(posedge sclk);
    repeat (40) @(posedge dclk);
    if (b1_q.size() != 0 || b1_acc != b1_del) bad($sformatf("bus one-shot after dst reset: acc %0d del %0d", b1_acc, b1_del));

    // ================= cdc_bus REPEAT
    b2_run = 1'b1;
    b2_cnt = 0;
    repeat (TREP) @(posedge sclk);
    begin
      automatic int exp_n = int'(real'(TREP) / real'(HOLD));
      if (b2_cnt < exp_n - 2 || b2_cnt > exp_n + 1) bad($sformatf("repeat bus: %0d updates, expected about %0d", b2_cnt, exp_n));
    end
    // resets of either side in the middle of the stream
    for (int i = 0; i < 4; i++) begin
      repeat (60) @(posedge sclk);
      @(posedge sclk); b2_srst <= 1'b1;
      repeat (3) @(posedge sclk);
      @(posedge sclk); b2_srst <= 1'b0;
      repeat (60) @(posedge sclk);
      @(posedge dclk); b2_drst <= 1'b1;
      repeat (4) @(posedge dclk);
      @(posedge dclk); b2_drst <= 1'b0;
    end
    b2_cnt = 0;
    repeat (4 * HOLD + 40) @(posedge sclk);
    if (b2_cnt == 0) bad("repeat bus: no updates after resets");
    b2_run = 1'b0;

    // ================= cdc_sync
    for (int i = 0; i < 16; i++) begin
      @(posedge sclk); sy_d <= $urandom;
      #(12.0 * PD);
      if (sy_q2 !== sy_d || sy_q3 !== sy_d) bad("cdc_sync value did not arrive");
    end

    done = 1'b1;
  end
endmodule


// ---------------------------------------------------------------- tdp_ram_2clk
module tb_cdc_ram (
  output logic done,
  output int   errors
);
  localparam int RW = 72, RD = 64;
  logic aclk, bclk;
  tb_cdc_clk #(.PERIOD(8.0))          u_aclk (.run(1'b1), .clk(aclk));
  tb_cdc_clk #(.PERIOD(10.0 * 1.003)) u_bclk (.run(1'b1), .clk(bclk));

  logic          a_we = 1'b0, b_en = 1'b0;
  logic [5:0]    a_addr = '0, b_addr = '0;
  logic [RW-1:0] a_wdata = '0, a_rdata, b_rdata;
  logic [RW-1:0] model [0:RD-1];
  logic [RW-1:0] ma1, ma2, mb1, mb2;
  int            err = 0, wr_cnt = 0, a_chk = 0, b_chk = 0, b_en_cnt = 0;
  int            mode = 0;          // 0: A only, 1: B reads (A reads), 2: A writes low half, B reads high half
  logic          run = 1'b0;
  assign errors = err;

  tdp_ram_2clk #(.W(RW), .DEPTH(RD)) u_ram (
    .a_clk(aclk), .a_we(a_we), .a_addr(a_addr), .a_wdata(a_wdata), .a_rdata(a_rdata),
    .b_clk(bclk), .b_en(b_en), .b_addr(b_addr), .b_rdata(b_rdata));

  initial begin
    for (int i = 0; i < RD; i++) model[i] = '0;
    ma1 = '0; ma2 = '0; mb1 = '0; mb2 = '0;
  end

  always @(posedge aclk) begin
    if (run) begin
      a_we    <= (mode != 1) && ($urandom_range(0, 2) == 0);
      a_addr  <= (mode == 2) ? {1'b0, 5'($urandom)} : 6'($urandom);
      a_wdata <= {$urandom, $urandom, 8'($urandom)};
    end else a_we <= 1'b0;
  end
  always @(posedge bclk) begin
    if (run && mode != 0) begin
      b_en   <= ($urandom_range(0, 3) != 0);
      b_addr <= (mode == 2) ? {1'b1, 5'($urandom)} : 6'($urandom);
    end else b_en <= 1'b0;
  end

  // reference: 2 register stages, A read-first
  always @(posedge aclk) begin
    if (a_rdata !== ma2) begin err++; if (err < 5) $display("ERROR: ram port A read %h expected %h", a_rdata, ma2); end
    a_chk++;
    ma2 = ma1;
    ma1 = model[a_addr];
    if (a_we) begin model[a_addr] = a_wdata; wr_cnt++; end
  end
  always @(posedge bclk) begin
    if (b_rdata !== mb2) begin err++; if (err < 5) $display("ERROR: ram port B read %h expected %h", b_rdata, mb2); end
    b_chk++;
    mb2 = mb1;
    if (b_en) begin mb1 = model[b_addr]; b_en_cnt++; end
  end

  initial begin
    done = 1'b0;
    repeat (10) @(posedge aclk);
    for (mode = 0; mode < 3; mode++) begin
      run = 1'b1;
      repeat (3000) @(posedge aclk);
      run = 1'b0;
      repeat (10) @(posedge aclk);
    end
    if (wr_cnt < 1000 || b_en_cnt < 1000) begin err++; $display("ERROR: ram test did too little (wr %0d b %0d)", wr_cnt, b_en_cnt); end
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------- rec_cdc_bridge
module tb_cdc_bridge (
  output logic done,
  output int   errors
);
  import rec_pkg::*;
  localparam int UIW = $bits(ui_stat_t), CSW = $bits(core_stat_t), CFW = $bits(rec_cfg_t);

  logic lclk, uiclk, usclk;
  logic ui_run = 1'b1, us_run = 1'b1;
  tb_cdc_clk #(.PERIOD(5.0))           u_l  (.run(1'b1),   .clk(lclk));
  tb_cdc_clk #(.PERIOD(10.0 * 1.0011)) u_ui (.run(ui_run), .clk(uiclk));
  tb_cdc_clk #(.PERIOD(8.0 * 0.9993))  u_us (.run(us_run), .clk(usclk));

  int err = 0;
  assign errors = err;
  task automatic bad(input string m);
    err++;
    if (err <= 8) $display("[%0t] ERROR bridge: %s", $time, m);
  endtask

  logic lrst = 1'b1, uirst = 1'b1, usrst = 1'b1;
  rec_cfg_t    cfg = '0;
  logic [31:0] step = '0;
  logic        step_load = 1'b0;
  logic [40:0] start_cfg = '0;
  logic        start_load = 1'b0;
  logic        p_stop = 1'b0, p_pf = 1'b0, p_clr = 1'b0, p_mt = 1'b0, p_dt = 1'b0, p_log = 1'b0;
  logic [11:0] xadc_temp = '0;
  ui_stat_t    ui_stat;
  logic        ui_age_ok;
  core_stat_t  core_stat;
  logic        core_age_ok;
  rec_cfg_t    cfg_ui;
  logic [31:0] step_ui;
  logic        rate_upd;
  logic [40:0] start_cfg_ui;
  logic        src_start, o_stop, o_pf, o_clr, o_mt, o_dt, o_log;
  logic [11:0] device_temp;
  ui_stat_t    ui_stat_in;
  core_stat_t  core_stat_in;

  function automatic logic [2047:0] repw(input logic [31:0] x);
    for (int i = 0; i < 64; i++) repw[32*i +: 32] = x;
  endfunction

  logic [31:0]   xu = '0, xs = '0, cx = '0;
  logic          cx_run = 1'b0;
  logic [2047:0] uw, sw, cw;
  assign uw = repw(xu);
  assign sw = repw(xs);
  assign cw = repw(cx);
  assign ui_stat_in   = ui_stat_t'(uw[UIW-1:0]);
  assign core_stat_in = core_stat_t'(sw[CSW-1:0]);

  rec_cdc_bridge dut (
    .link_clk(lclk), .link_rst(lrst),
    .cfg(cfg), .step(step), .step_load(step_load), .start_cfg(start_cfg), .start_load(start_load),
    .src_stop(p_stop), .src_pf(p_pf), .ui_clear(p_clr), .mt_start(p_mt), .dtest_start(p_dt),
    .log_req(p_log), .xadc_temp(xadc_temp),
    .ui_stat(ui_stat), .ui_age_ok(ui_age_ok), .core_stat(core_stat), .core_age_ok(core_age_ok),
    .ui_clk(uiclk), .ui_rst(uirst),
    .cfg_ui(cfg_ui), .step_ui(step_ui), .rate_upd(rate_upd), .start_cfg_ui(start_cfg_ui), .src_start(src_start),
    .src_stop_ui(o_stop), .src_pf_ui(o_pf), .ui_clear_ui(o_clr), .mt_start_ui(o_mt), .dtest_start_ui(o_dt),
    .device_temp(device_temp), .ui_stat_in(ui_stat_in),
    .user_clk(usclk), .user_rst(usrst), .log_req_u(o_log), .core_stat_in(core_stat_in));

  always @(posedge uiclk) if (!uirst) xu <= xu + 1;
  always @(posedge usclk) if (!usrst) xs <= xs + 1;
  always @(posedge lclk)  if (cx_run) cx <= cx + 1;
  always @(posedge lclk)  if (cx_run) cfg <= rec_cfg_t'(cw[CFW-1:0]);

  // ---- snapshot checks (link side): untorn, monotonic, fresh
  logic [2047:0] uv, uv_p = '0, sv, sv_p = '0;
  always @* begin
    uv = '0; uv[UIW-1:0] = ui_stat;
    sv = '0; sv[CSW-1:0] = core_stat;
  end
  logic [31:0] u_last = '0, s_last = '0;
  int          u_chg = 0, s_chg = 0;
  logic        chk_en = 1'b0;
  always @(posedge lclk) if (chk_en) begin
    uv_p <= uv; sv_p <= sv;
    if (uv !== uv_p) begin
      u_chg++;
      for (int i = 0; i < UIW / 32; i++) if (uv[32*i +: 32] !== uv[31:0]) bad("ui_stat torn");
      if (uv[31:0] < u_last) bad("ui_stat went backwards");
      if (int'(xu) - int'(uv[31:0]) > 40 || int'(xu) - int'(uv[31:0]) < 0) bad($sformatf("ui_stat lag %0d", int'(xu) - int'(uv[31:0])));
      u_last <= uv[31:0];
    end
    if (sv !== sv_p) begin
      s_chg++;
      for (int i = 0; i < CSW / 32; i++) if (sv[32*i +: 32] !== sv[31:0]) bad("core_stat torn");
      if (sv[31:0] < s_last) bad("core_stat went backwards");
      if (int'(xs) - int'(sv[31:0]) > 40 || int'(xs) - int'(sv[31:0]) < 0) bad($sformatf("core_stat lag %0d", int'(xs) - int'(sv[31:0])));
      s_last <= sv[31:0];
    end
  end

  // ---- cfg_ui untorn (115 bits = three full words + 19 bits)
  logic [CFW-1:0] cfg_p = '0;
  logic           cfg_chk = 1'b0;
  logic [CFW-1:0] cfg_c;
  always @(posedge uiclk) begin
    cfg_p <= cfg_ui;
    cfg_c  = cfg_ui;
    if (cfg_chk && cfg_ui !== cfg_p) begin
      if (cfg_c[31:0] !== cfg_c[63:32] || cfg_c[63:32] !== cfg_c[95:64] || cfg_c[CFW-1:96] !== cfg_c[CFW-97:0]) bad("cfg_ui torn");
    end
  end

  // ---- pulse counters
  int n_in [6]  = '{0, 0, 0, 0, 0, 0};
  int n_out[6]  = '{0, 0, 0, 0, 0, 0};
  always @(posedge lclk) if (!lrst) begin
    if (p_stop) n_in[0]++;
    if (p_pf)   n_in[1]++;
    if (p_clr)  n_in[2]++;
    if (p_mt)   n_in[3]++;
    if (p_dt)   n_in[4]++;
    if (p_log)  n_in[5]++;
  end
  always @(posedge uiclk) begin
    if (o_stop) n_out[0]++;
    if (o_pf)   n_out[1]++;
    if (o_clr)  n_out[2]++;
    if (o_mt)   n_out[3]++;
    if (o_dt)   n_out[4]++;
  end
  always @(posedge usclk) if (o_log) n_out[5]++;

  task automatic check_pulses(input string w);
    repeat (40) @(posedge lclk);
    for (int i = 0; i < 6; i++)
      if (n_in[i] != n_out[i]) bad($sformatf("%s: pulse channel %0d in %0d out %0d", w, i, n_in[i], n_out[i]));
  endtask

  // ---- step / start delivery recorders
  logic [31:0] st_log [$];
  logic [40:0] sc_log [$];
  int          rate_n = 0, start_n = 0;
  always @(posedge uiclk) begin
    if (rate_upd)  begin st_log.push_back(step_ui);       rate_n++;  end
    if (src_start) begin sc_log.push_back(start_cfg_ui);  start_n++; end
  end

  task automatic load_step(input logic [31:0] v);
    @(posedge lclk); step <= v; step_load <= 1'b1;
    @(posedge lclk); step_load <= 1'b0;
  endtask
  task automatic load_start(input logic [40:0] v);
    @(posedge lclk); start_cfg <= v; start_load <= 1'b1;
    @(posedge lclk); start_load <= 1'b0;
  endtask

  initial begin
    done = 1'b0;
    // ---- reset
    repeat (6) @(posedge lclk);
    @(posedge uiclk); uirst <= 1'b0;
    @(posedge usclk); usrst <= 1'b0;
    repeat (4) @(posedge lclk);
    @(posedge lclk); lrst <= 1'b0;
    if (ui_age_ok !== 1'b0 || core_age_ok !== 1'b0) bad("age flags not 0 right after reset");
    repeat (80) @(posedge lclk);
    if (!ui_age_ok)   bad("ui_age_ok not set after first update");
    if (!core_age_ok) bad("core_age_ok not set after first update");
    chk_en = 1'b1;
    repeat (3000) @(posedge lclk);
    if (u_chg < 100 || s_chg < 100) bad($sformatf("too few snapshots: ui %0d core %0d", u_chg, s_chg));

    // ---- X1 / X8 pulses
    for (int i = 0; i < 300; i++) begin
      automatic logic [5:0] m = 6'($urandom);
      @(posedge lclk);
      {p_stop, p_pf, p_clr, p_mt, p_dt, p_log} <= m;
      @(posedge lclk);
      {p_stop, p_pf, p_clr, p_mt, p_dt, p_log} <= 6'd0;
      repeat ($urandom_range(25, 45)) @(posedge lclk);
    end
    check_pulses("random");
    if (n_in[0] < 100) bad("pulse test too short");
    // reset of ui / user domain with the link idle: no false pulses
    for (int i = 0; i < 4; i++) begin
      @(posedge uiclk); uirst <= 1'b1; @(posedge usclk); usrst <= 1'b1;
      repeat (6) @(posedge usclk);
      @(posedge uiclk); uirst <= 1'b0; @(posedge usclk); usrst <= 1'b0;
      repeat (40) @(posedge lclk);
      check_pulses("ui/user reset");
    end

    // ---- X2 cfg: static values
    for (int i = 0; i < 12; i++) begin
      automatic logic [CFW-1:0] r;
      for (int k = 0; k < CFW; k++) r[k] = $urandom;
      @(posedge lclk); cfg <= rec_cfg_t'(r);
      repeat (60) @(posedge lclk);
      if (cfg_ui !== rec_cfg_t'(r)) bad("cfg_ui != cfg");
    end
    // cfg changing every cycle: untorn
    cfg_chk = 1'b1; cx_run = 1'b1;
    repeat (3000) @(posedge lclk);
    cx_run = 1'b0;
    repeat (40) @(posedge lclk);
    cfg_chk = 1'b0;

    // ---- X2a step: single loads
    for (int i = 0; i < 20; i++) begin
      automatic logic [31:0] v = $urandom;
      automatic int n0 = rate_n;
      load_step(v);
      repeat (60) @(posedge lclk);
      if (rate_n != n0 + 1) bad($sformatf("step single: %0d updates", rate_n - n0));
      else if (st_log[$] !== v) bad("step single: wrong value");
    end
    // bursts (consecutive loads): newest wins, every delivered value was loaded, order kept
    for (int i = 0; i < 12; i++) begin
      automatic int n = $urandom_range(2, 10);
      automatic logic [31:0] vals [$];
      automatic int n0 = rate_n;
      automatic int last_idx = -1;
      for (int k = 0; k < n; k++) begin
        vals.push_back($urandom);
        @(posedge lclk); step <= vals[k]; step_load <= 1'b1;
      end
      @(posedge lclk); step_load <= 1'b0;
      repeat (120) @(posedge lclk);
      if (rate_n - n0 < 1 || rate_n - n0 > n) bad($sformatf("step burst: %0d updates for %0d loads", rate_n - n0, n));
      if (st_log[$] !== vals[n-1]) bad("step burst: final value is not the newest");
      for (int k = n0; k < rate_n; k++) begin
        automatic int idx = -1;
        for (int j = 0; j < n; j++) if (vals[j] === st_log[k] && j > last_idx) begin idx = j; break; end
        if (idx < 0) bad("step burst: delivered value invented or out of order");
        else last_idx = idx;
      end
    end

    // ---- X2b start_cfg
    for (int i = 0; i < 20; i++) begin
      automatic logic [40:0] v = 41'({$urandom, $urandom});
      automatic int n0 = start_n;
      load_start(v);
      repeat (60) @(posedge lclk);
      if (start_n != n0 + 1) bad($sformatf("start single: %0d updates", start_n - n0));
      else if (sc_log[$] !== v) bad("start single: wrong value");
    end
    for (int i = 0; i < 12; i++) begin
      automatic int n = $urandom_range(2, 10);
      automatic logic [40:0] vals [$];
      automatic int n0 = start_n;
      for (int k = 0; k < n; k++) begin
        vals.push_back(41'({$urandom, $urandom}));
        @(posedge lclk); start_cfg <= vals[k]; start_load <= 1'b1;
      end
      @(posedge lclk); start_load <= 1'b0;
      repeat (120) @(posedge lclk);
      if (start_n - n0 < 1 || start_n - n0 > n) bad("start burst: update count");
      if (sc_log[$] !== vals[n-1]) bad("start burst: final value is not the newest");
    end

    // ---- X14 temperature
    for (int i = 0; i < 12; i++) begin
      automatic logic [11:0] t = 12'($urandom);
      @(posedge lclk); xadc_temp <= t;
      repeat (60) @(posedge lclk);
      if (device_temp !== t) bad("device_temp mismatch");
    end

    // ---- age flags (1 ms), ui side
    ui_run = 1'b0;
    #(800_000.0);
    if (!ui_age_ok)   bad("ui_age_ok dropped before 1 ms");
    if (!core_age_ok) bad("core_age_ok dropped while the user clock runs");
    #(350_000.0);
    if (ui_age_ok)    bad("ui_age_ok still 1 after 1.15 ms without updates");
    if (!core_age_ok) bad("core_age_ok dropped while the user clock runs (2)");
    ui_run = 1'b1;
    repeat (200) @(posedge lclk);
    if (!ui_age_ok)   bad("ui_age_ok did not recover");
    // user side
    us_run = 1'b0;
    #(800_000.0);
    if (!core_age_ok) bad("core_age_ok dropped before 1 ms");
    #(350_000.0);
    if (core_age_ok)  bad("core_age_ok still 1 after 1.15 ms without updates");
    if (!ui_age_ok)   bad("ui_age_ok dropped while the ui clock runs");
    us_run = 1'b1;
    repeat (200) @(posedge lclk);
    if (!core_age_ok) bad("core_age_ok did not recover");

    check_pulses("final");
    done = 1'b1;
  end
endmodule

// ---------------------------------------------------------------- top
module tb_cdc;
  localparam int NCASE = 9;
  logic [NCASE-1:0] case_done;
  int               case_err [NCASE];

  tb_cdc_case #(.PS(5.0),  .PD(10.0))  c0 (.done(case_done[0]), .errors(case_err[0]));   // 200 -> 100
  tb_cdc_case #(.PS(10.0), .PD(5.0))   c1 (.done(case_done[1]), .errors(case_err[1]));   // 100 -> 200
  tb_cdc_case #(.PS(8.0),  .PD(5.0))   c2 (.done(case_done[2]), .errors(case_err[2]));   // 125 -> 200
  tb_cdc_case #(.PS(5.0),  .PD(8.0))   c3 (.done(case_done[3]), .errors(case_err[3]));   // 200 -> 125
  tb_cdc_case #(.PS(10.0), .PD(8.0))   c4 (.done(case_done[4]), .errors(case_err[4]));   // 100 -> 125
  tb_cdc_case #(.PS(8.0),  .PD(10.0))  c5 (.done(case_done[5]), .errors(case_err[5]));   // 125 -> 100
  tb_cdc_case #(.PS(9.97), .PD(10.0))  c6 (.done(case_done[6]), .errors(case_err[6]));   // 100.3 -> 100
  tb_cdc_case #(.PS(5.0),  .PD(5.0))   c7 (.done(case_done[7]), .errors(case_err[7]));   // 200 -> 200
  // built-in negative control: HOLD_CYC far below the contract (200 -> 100 needs 14) must fail
  tb_cdc_case #(.PS(5.0), .PD(10.0), .HOLD_FORCE(2), .EXPECT_FAIL(1'b1), .NPULSE(100), .NBUS(150), .TREP(1500))
                                       c8 (.done(case_done[8]), .errors(case_err[8]));

  logic ram_done, br_done;
  int   ram_err, br_err;
  tb_cdc_ram    u_ram (.done(ram_done), .errors(ram_err));
  tb_cdc_bridge u_br  (.done(br_done),  .errors(br_err));

  initial begin
    #(4_000_000.0);
    $fatal(1, "tb_cdc: watchdog timeout");
  end

  initial begin
    int total;
    wait (&case_done && ram_done && br_done);
    total = ram_err + br_err;
    for (int i = 0; i < 8; i++) total += case_err[i];
    $display("errors: ram=%0d bridge=%0d cases=%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d  hold-violation case=%0d (must be >0)",
             ram_err, br_err, case_err[0], case_err[1], case_err[2], case_err[3], case_err[4],
             case_err[5], case_err[6], case_err[7], case_err[8]);
    if (case_err[8] == 0) begin
      total++;
      $display("ERROR: negative control (HOLD_CYC=2) did not produce errors");
    end
    if (total != 0) $fatal(1, "tb_cdc: %0d errors", total);
    $display("TEST PASSED");
    $finish;
  end
endmodule
