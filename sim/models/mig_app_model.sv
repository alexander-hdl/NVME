// =============================================================================
// mig_app_model.sv - behavioural model of the 7-series MIG DDR3 native app interface
//                    (x32, BL8, 4:1, 256-bit app data).  SIMULATION ONLY.
//
// Ports follow the `mig_ddr3` IP (app_*, ui_clk, ui_clk_sync_rst, init_calib_complete,
// device_temp_i); the DDR3 pins, sys_clk_i/clk_ref_i/sys_rst are not modelled.
// Spec: STAND/RECORDER_SPEC.md 9.2.
//
// Behaviour
//   * ui_clk generated here: period UI_PERIOD_PS, per-cycle random jitter +-JITTER_PS.
//   * ui_clk_sync_rst high for the first 20 cycles; init_calib_complete rises calib_delay
//     cycles later (never while calib_fail=1).
//   * Memory = associative array of 256-bit words indexed by app_addr[28:3].
//   * app_rdy / app_wdf_rdy are registered outputs, low: before calibration, during refresh
//     (refresh_len cycles every refresh_period), randomly (rdy_drop_pct, independent for the two),
//     and by the bandwidth limiter (bw_limit_pct: average commands per ui cycle).
//   * Write data and write command are paired in order (data may precede the command).
//   * Read data returns in order, one word per cycle, rd_latency (+ random 0..rd_lat_jitter) cycles
//     after the command is accepted.
//   * Protocol checks (each violation increments proto_errors and prints MIG_MODEL_ERROR):
//     no command / write data before calibration, command and write data held (same value)
//     until accepted, app_addr[2:0] = 0 and < 2^28 words, cmd is read/write only, wdf_end = wren,
//     wdf_mask = 0, write data not later than WDF_MAX_LAG cycles after its command and not more than
//     WDF_MAX_LEAD words ahead of the commands.
//   * Statistics: wr_cmds, rd_cmds, unwritten_reads (reads of never written words),
//     short_runs (runs of equal-direction commands that are not a multiple of 64 or do not start
//     at a 2 KiB boundary: call finish_runs() before reading it), injected (inject_err_addr hits).
//   * inject_err_addr (word address, as app_addr; -1 = off): a read of that address returns
//     the data with bit 77 flipped.
// =============================================================================
`timescale 1ps/1ps
module mig_app_model #(
  parameter int UI_PERIOD_PS   = 10000,
  parameter int JITTER_PS      = 0,
  parameter int CALIB_DELAY    = 200,
  parameter bit CALIB_FAIL     = 1'b0,
  parameter int RD_LATENCY     = 30,
  parameter int RD_LAT_JITTER  = 0,
  parameter int RDY_DROP_PCT   = 0,
  parameter int REFRESH_PERIOD = 780,
  parameter int REFRESH_LEN    = 14,
  parameter int BW_LIMIT_PCT   = 100,
  parameter int WDF_MAX_LAG    = 4,
  parameter int WDF_MAX_LEAD   = 8
) (
  output logic          ui_clk,
  output logic          ui_clk_sync_rst,
  output logic          init_calib_complete,
  input  logic [11:0]   device_temp_i,
  input  logic [28:0]   app_addr,
  input  logic [2:0]    app_cmd,
  input  logic          app_en,
  output logic          app_rdy,
  input  logic [255:0]  app_wdf_data,
  input  logic [31:0]   app_wdf_mask,
  input  logic          app_wdf_wren,
  input  logic          app_wdf_end,
  output logic          app_wdf_rdy,
  output logic [255:0]  app_rd_data,
  output logic          app_rd_data_valid,
  output logic          app_rd_data_end,
  input  logic          app_sr_req,
  input  logic          app_ref_req,
  input  logic          app_zq_req,
  output logic          app_sr_active,
  output logic          app_ref_ack,
  output logic          app_zq_ack
);

  // ---- knobs (change from the testbench at run time) ----
  int      calib_delay    = CALIB_DELAY;
  bit      calib_fail     = CALIB_FAIL;
  int      rd_latency     = RD_LATENCY;
  int      rd_lat_jitter  = RD_LAT_JITTER;
  int      rdy_drop_pct   = RDY_DROP_PCT;
  int      refresh_period = REFRESH_PERIOD;
  int      refresh_len    = REFRESH_LEN;
  int      bw_limit_pct   = BW_LIMIT_PCT;
  longint  inject_err_addr = -1;
  bit      track_runs     = 1'b1;

  // ---- statistics ----
  longint unsigned wr_cmds = 0, rd_cmds = 0, cycles = 0;
  int unsigned     proto_errors = 0, unwritten_reads = 0, short_runs = 0, injected = 0;
  logic [28:0]     max_addr = '0;

  logic [255:0] mem [longint unsigned];

  typedef struct {
    longint unsigned ready;
    logic [255:0]    data;
  } rsp_t;

  rsp_t         rq [$];
  logic [255:0] wdq [$];
  logic [28:0]  wcq [$];

  assign app_sr_active = 1'b0;
  assign app_ref_ack   = 1'b0;
  assign app_zq_ack    = 1'b0;

  // ---- clock ----
  initial begin
    ui_clk = 1'b0;
    forever begin
      int p;
      p = UI_PERIOD_PS;
      if (JITTER_PS > 0) p = p + $urandom_range(0, 2 * JITTER_PS) - JITTER_PS;
      #(p / 2) ui_clk = 1'b1;
      #(p - p / 2) ui_clk = 1'b0;
    end
  end

  task automatic err(input string msg);
    proto_errors++;
    $display("MIG_MODEL_ERROR t=%0t: %s", $time, msg);
  endtask

  // run-length bookkeeping
  bit          run_v = 0, run_rd = 0;
  int unsigned run_len = 0;
  logic [28:0] run_start = '0;

  task automatic close_run();
    if (run_v && track_runs) begin
      if ((run_len % 64) != 0 || (run_start % 512) != 0) short_runs++;
    end
    run_v = 0;
  endtask

  task automatic finish_runs();
    close_run();
  endtask

  task automatic note_cmd(input bit is_rd, input logic [28:0] a);
    if (run_v && run_rd != is_rd) close_run();
    if (!run_v) begin
      run_v = 1; run_rd = is_rd; run_len = 0; run_start = a;
    end
    run_len++;
  endtask

  // ---- per-cycle behaviour ----
  int          rst_cnt = 0;
  int          calib_cnt = 0;
  int          refr_cnt = 0;
  int          credit = 100;
  int          lag_cnt = 0;
  bit          prev_en = 0, prev_rdy = 0, prev_wren = 0, prev_wdf_rdy = 0;
  logic [28:0] prev_addr;
  logic [2:0]  prev_cmd;
  logic [255:0] prev_wdata;
  longint unsigned last_ready = 0;

  initial begin
    ui_clk_sync_rst = 1'b1;
    init_calib_complete = 1'b0;
    app_rdy = 1'b0;
    app_wdf_rdy = 1'b0;
    app_rd_data = '0;
    app_rd_data_valid = 1'b0;
    app_rd_data_end = 1'b0;
  end

  always @(posedge ui_clk) begin
    bit acc_c, acc_d, refr, rdy_nx, wdf_nx;
    logic [255:0] rdw;
    longint unsigned rdy_t;
    cycles++;

    // reset and calibration sequencing
    if (rst_cnt < 20) begin
      rst_cnt++;
      ui_clk_sync_rst <= 1'b1;
    end else begin
      ui_clk_sync_rst <= 1'b0;
      if (!init_calib_complete && !calib_fail) begin
        calib_cnt++;
        if (calib_cnt >= calib_delay) init_calib_complete <= 1'b1;
      end
    end

    acc_c = (app_en === 1'b1) && app_rdy && !ui_clk_sync_rst;
    acc_d = (app_wdf_wren === 1'b1) && app_wdf_rdy && !ui_clk_sync_rst;

    // ---- protocol checks ----
    if (!ui_clk_sync_rst) begin
    if (!init_calib_complete && (app_en || app_wdf_wren)) err("command or write data before calibration");
    if (prev_en && !prev_rdy) begin
      if (!app_en)                                  err("app_en dropped before app_rdy");
      else if (app_addr != prev_addr || app_cmd != prev_cmd) err("app_addr/app_cmd changed while waiting for app_rdy");
    end
    if (prev_wren && !prev_wdf_rdy) begin
      if (!app_wdf_wren)                            err("app_wdf_wren dropped before app_wdf_rdy");
      else if (app_wdf_data !== prev_wdata)         err("app_wdf_data changed while waiting for app_wdf_rdy");
    end
    if (app_en) begin
      if (app_addr[2:0] != 3'd0)                    err("app_addr not a multiple of 8 (BL8)");
      if (app_cmd != 3'b000 && app_cmd != 3'b001)   err("unsupported app_cmd");
    end
    if (app_wdf_wren !== app_wdf_end)               err("app_wdf_end != app_wdf_wren (4:1 BL8 is one beat)");
    if (app_wdf_wren && app_wdf_mask != 32'h0)      err("app_wdf_mask not 0");
    if ((app_en === 1'bx) || (app_wdf_wren === 1'bx)) err("X on app_en / app_wdf_wren");
    end
    prev_en = app_en; prev_rdy = app_rdy; prev_addr = app_addr; prev_cmd = app_cmd;
    prev_wren = app_wdf_wren; prev_wdf_rdy = app_wdf_rdy; prev_wdata = app_wdf_data;

    // ---- accept write data / command ----
    if (acc_d) wdq.push_back(app_wdf_data);
    if (acc_c && app_cmd == 3'b000) begin
      wcq.push_back(app_addr);
      wr_cmds++;
      note_cmd(0, app_addr);
    end
    while (wcq.size() > 0 && wdq.size() > 0) begin
      logic [28:0] wa;
      wa = wcq.pop_front();
      mem[wa[28:3]] = wdq.pop_front();
    end
    if (wcq.size() > 0) begin
      lag_cnt++;
      if (lag_cnt > WDF_MAX_LAG) begin err("write data later than WDF_MAX_LAG cycles after its command"); lag_cnt = 0; wcq.delete(); end
    end else lag_cnt = 0;
    if (wdq.size() > WDF_MAX_LEAD) begin err("write data too far ahead of commands"); wdq.delete(); end

    // ---- accept read command ----
    if (acc_c && app_cmd == 3'b001) begin
      rd_cmds++;
      note_cmd(1, app_addr);
      if (mem.exists(app_addr[28:3])) rdw = mem[app_addr[28:3]];
      else begin
        unwritten_reads++;
        rdw = {8{32'hDEAD_BEEF}} ^ {227'd0, app_addr};
      end
      if (inject_err_addr >= 0 && longint'(app_addr) == inject_err_addr) begin
        rdw[77] = ~rdw[77];
        injected++;
      end
      rdy_t = cycles + rd_latency + ((rd_lat_jitter > 0) ? $urandom_range(0, rd_lat_jitter) : 0);
      if (rdy_t <= last_ready) rdy_t = last_ready + 1;
      last_ready = rdy_t;
      rq.push_back('{ready: rdy_t, data: rdw});
    end
    if (acc_c && app_addr > max_addr) max_addr = app_addr;
    if (acc_c && app_addr[28]) err("app_addr beyond 1 GiB");

    // ---- read response ----
    if (rq.size() > 0 && rq[0].ready <= cycles) begin
      app_rd_data       <= rq[0].data;
      app_rd_data_valid <= 1'b1;
      app_rd_data_end   <= 1'b1;
      rq.pop_front();
    end else begin
      app_rd_data_valid <= 1'b0;
      app_rd_data_end   <= 1'b0;
    end

    // ---- ready generation for the next cycle ----
    if (refresh_period > 0) begin
      refr_cnt++;
      if (refr_cnt >= refresh_period + refresh_len) refr_cnt = 0;
    end
    refr = (refresh_period > 0) && (refr_cnt >= refresh_period);
    credit = credit + bw_limit_pct;
    if (credit > 100 + bw_limit_pct) credit = 100 + bw_limit_pct;
    if (acc_c) credit = credit - 100;
    rdy_nx = init_calib_complete && !refr && (credit >= 100) && ($urandom_range(0, 99) >= rdy_drop_pct);
    wdf_nx = init_calib_complete && !refr && ($urandom_range(0, 99) >= rdy_drop_pct);
    app_rdy     <= rdy_nx;
    app_wdf_rdy <= wdf_nx;
  end

endmodule
