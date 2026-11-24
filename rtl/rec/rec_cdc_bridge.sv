// =============================================================================
// rec_cdc_bridge.sv — all recorder clock-domain crossings X1..X4, X8, X10, X14 (WP0).
//
// Domains : link_clk (200 MHz), ui_clk (MIG, 100 MHz), user_clk (pcie, 125 MHz); ports are
//           grouped and commented per domain.  Built only from cdc_pulse / cdc_bus.
// Spec    : STAND/RECORDER_SPEC.md 4.3 and 4.4.  Port names as in 4.4; link_clk / link_rst
//           are added (4.4 lists no link clock/reset names for this module).
//
//  X1   link->ui   src_stop, src_pf, ui_clear, mt_start, dtest_start   cdc_pulse
//  X2   link->ui   cfg                                                  cdc_bus REPEAT=1
//  X2a  link->ui   step (+rate_upd = dst_upd)                           cdc_bus one-shot
//  X2b  link->ui   start_cfg {len_blocks[39:0], prof_en} (+src_start)   cdc_bus one-shot
//  X4   ui->link   ui_stat (+ age counter -> ui_age_ok)                 cdc_bus REPEAT=1
//  X8   link->user log_req                                              cdc_pulse
//  X10  user->link core_stat (+ age counter -> core_age_ok)             cdc_bus REPEAT=1
//  X14  link->ui   xadc_temp -> device_temp                             cdc_bus REPEAT=1
//
// X2a / X2b: a load issued while the previous transfer is still in flight (HOLD_CYC
// cycles) is not dropped: the newest value is parked and sent as soon as the bus is free
// (load -> delivery latency = 2 link cycles + cdc_bus latency; loads closer than HOLD_CYC
// merge, the newest wins).
// *_age_ok = 0 until the first update after reset and whenever no update arrived for
// AGE_LIMIT_US (1 ms) (spec 4.3 "age").
// F_*_MHZ set HOLD_CYC = ceil(6*f_src/f_dst)+2 per crossing and the age limit.
// =============================================================================
module rec_cdc_bridge
  import rec_pkg::*;
#(
  parameter int F_LINK_MHZ = 200,
  parameter int F_UI_MHZ   = 100,
  parameter int F_USER_MHZ = 125
) (
  // ---- link_clk domain
  input  logic         link_clk,
  input  logic         link_rst,
  input  rec_cfg_t     cfg,                // link
  input  logic [31:0]  step,               // link
  input  logic         step_load,          // link
  input  logic [40:0]  start_cfg,          // link
  input  logic         start_load,         // link
  input  logic         src_stop,           // link
  input  logic         src_pf,             // link
  input  logic         ui_clear,           // link
  input  logic         mt_start,           // link
  input  logic         dtest_start,        // link
  input  logic         log_req,            // link
  input  logic [11:0]  xadc_temp,          // link
  output ui_stat_t     ui_stat,            // link
  output logic         ui_age_ok,          // link
  output core_stat_t   core_stat,          // link
  output logic         core_age_ok,        // link
  // ---- ui_clk domain
  input  logic         ui_clk,
  input  logic         ui_rst,
  output rec_cfg_t     cfg_ui,             // ui
  output logic [31:0]  step_ui,            // ui
  output logic         rate_upd,           // ui
  output logic [40:0]  start_cfg_ui,       // ui
  output logic         src_start,          // ui
  output logic         src_stop_ui,        // ui
  output logic         src_pf_ui,          // ui
  output logic         ui_clear_ui,        // ui
  output logic         mt_start_ui,        // ui
  output logic         dtest_start_ui,     // ui
  output logic [11:0]  device_temp,        // ui
  input  ui_stat_t     ui_stat_in,         // ui
  // ---- user_clk domain
  input  logic         user_clk,
  input  logic         user_rst,
  output logic         log_req_u,          // user
  input  core_stat_t   core_stat_in        // user
);

  // hold time ceil(6*f_src/f_dst)+2
  function automatic int hold_cyc(input int f_src, input int f_dst);
    return (6 * f_src + f_dst - 1) / f_dst + 2;
  endfunction

  localparam int H_L2U = hold_cyc(F_LINK_MHZ, F_UI_MHZ);
  localparam int H_U2L = hold_cyc(F_UI_MHZ,   F_LINK_MHZ);
  localparam int H_C2L = hold_cyc(F_USER_MHZ, F_LINK_MHZ);
  localparam int AGE_LIMIT = F_LINK_MHZ * AGE_LIMIT_US;     // link cycles
  localparam int AGE_W     = $clog2(AGE_LIMIT + 2);

  // ------------------------------------------------------------------ X1 pulses link -> ui
  cdc_pulse u_cdc_pulse_stop  (.src_clk(link_clk), .src_rst(link_rst), .src_pulse(src_stop),
                               .dst_clk(ui_clk), .dst_rst(ui_rst), .dst_pulse(src_stop_ui));
  cdc_pulse u_cdc_pulse_pf    (.src_clk(link_clk), .src_rst(link_rst), .src_pulse(src_pf),
                               .dst_clk(ui_clk), .dst_rst(ui_rst), .dst_pulse(src_pf_ui));
  cdc_pulse u_cdc_pulse_clear (.src_clk(link_clk), .src_rst(link_rst), .src_pulse(ui_clear),
                               .dst_clk(ui_clk), .dst_rst(ui_rst), .dst_pulse(ui_clear_ui));
  cdc_pulse u_cdc_pulse_mt    (.src_clk(link_clk), .src_rst(link_rst), .src_pulse(mt_start),
                               .dst_clk(ui_clk), .dst_rst(ui_rst), .dst_pulse(mt_start_ui));
  cdc_pulse u_cdc_pulse_dt    (.src_clk(link_clk), .src_rst(link_rst), .src_pulse(dtest_start),
                               .dst_clk(ui_clk), .dst_rst(ui_rst), .dst_pulse(dtest_start_ui));

  // ------------------------------------------------------------------ X8 log_req link -> user
  cdc_pulse u_cdc_pulse_log   (.src_clk(link_clk), .src_rst(link_rst), .src_pulse(log_req),
                               .dst_clk(user_clk), .dst_rst(user_rst), .dst_pulse(log_req_u));

  // ------------------------------------------------------------------ X2 cfg (repeat)
  localparam int CFG_W = $bits(rec_cfg_t);
  logic [CFG_W-1:0] cfg_bus;
  logic             cfg_upd_unused;
  logic [7:0]       cfg_tag_unused;
  logic             cfg_busy_unused;

  cdc_bus #(.W(CFG_W), .HOLD_CYC(H_L2U), .REPEAT(1'b1)) u_cdc_bus_cfg (
    .src_clk(link_clk), .src_rst(link_rst), .src_data(CFG_W'(cfg)), .src_load(1'b0), .src_busy(cfg_busy_unused),
    .dst_clk(ui_clk), .dst_rst(ui_rst), .dst_data(cfg_bus), .dst_upd(cfg_upd_unused), .dst_tag(cfg_tag_unused));
  assign cfg_ui = rec_cfg_t'(cfg_bus);

  // ------------------------------------------------------------------ X2a step (one-shot, parked load)
  logic [31:0] step_pend_q;
  logic        step_pend_v_q;
  logic        step_busy, step_issue;
  logic [7:0]  step_tag_unused;

  assign step_issue = step_pend_v_q && !step_busy;

  always_ff @(posedge link_clk) begin
    if (link_rst) step_pend_v_q <= 1'b0;
    else begin
      if (step_issue) step_pend_v_q <= 1'b0;
      if (step_load)  step_pend_v_q <= 1'b1;
    end
    if (step_load) step_pend_q <= step;
  end

  cdc_bus #(.W(32), .HOLD_CYC(H_L2U), .REPEAT(1'b0)) u_cdc_bus_step (
    .src_clk(link_clk), .src_rst(link_rst), .src_data(step_pend_q), .src_load(step_issue), .src_busy(step_busy),
    .dst_clk(ui_clk), .dst_rst(ui_rst), .dst_data(step_ui), .dst_upd(rate_upd), .dst_tag(step_tag_unused));

  // ------------------------------------------------------------------ X2b start_cfg (one-shot, parked load)
  logic [40:0] start_pend_q;
  logic        start_pend_v_q;
  logic        start_busy, start_issue;
  logic [7:0]  start_tag_unused;

  assign start_issue = start_pend_v_q && !start_busy;

  always_ff @(posedge link_clk) begin
    if (link_rst) start_pend_v_q <= 1'b0;
    else begin
      if (start_issue) start_pend_v_q <= 1'b0;
      if (start_load)  start_pend_v_q <= 1'b1;
    end
    if (start_load) start_pend_q <= start_cfg;
  end

  cdc_bus #(.W(41), .HOLD_CYC(H_L2U), .REPEAT(1'b0)) u_cdc_bus_start (
    .src_clk(link_clk), .src_rst(link_rst), .src_data(start_pend_q), .src_load(start_issue), .src_busy(start_busy),
    .dst_clk(ui_clk), .dst_rst(ui_rst), .dst_data(start_cfg_ui), .dst_upd(src_start), .dst_tag(start_tag_unused));

  // ------------------------------------------------------------------ X14 xadc temperature (repeat)
  logic       temp_upd_unused, temp_busy_unused;
  logic [7:0] temp_tag_unused;

  cdc_bus #(.W(12), .HOLD_CYC(H_L2U), .REPEAT(1'b1)) u_cdc_bus_temp (
    .src_clk(link_clk), .src_rst(link_rst), .src_data(xadc_temp), .src_load(1'b0), .src_busy(temp_busy_unused),
    .dst_clk(ui_clk), .dst_rst(ui_rst), .dst_data(device_temp), .dst_upd(temp_upd_unused), .dst_tag(temp_tag_unused));

  // ------------------------------------------------------------------ X4 ui_stat ui -> link (repeat)
  localparam int UIS_W = $bits(ui_stat_t);
  logic [UIS_W-1:0] uis_bus;
  logic             uis_upd;
  logic             uis_busy_unused;
  logic [7:0]       uis_tag_unused;
  logic [AGE_W-1:0] uis_age_q;

  cdc_bus #(.W(UIS_W), .HOLD_CYC(H_U2L), .REPEAT(1'b1)) u_cdc_bus_uistat (
    .src_clk(ui_clk), .src_rst(ui_rst), .src_data(UIS_W'(ui_stat_in)), .src_load(1'b0), .src_busy(uis_busy_unused),
    .dst_clk(link_clk), .dst_rst(link_rst), .dst_data(uis_bus), .dst_upd(uis_upd), .dst_tag(uis_tag_unused));
  assign ui_stat = ui_stat_t'(uis_bus);

  always_ff @(posedge link_clk) begin
    if (link_rst) begin
      uis_age_q <= AGE_W'(AGE_LIMIT + 1);
      ui_age_ok <= 1'b0;
    end else begin
      if (uis_upd) uis_age_q <= '0;
      else if (uis_age_q != AGE_W'(AGE_LIMIT + 1)) uis_age_q <= uis_age_q + 1'b1;
      ui_age_ok <= uis_upd || (uis_age_q < AGE_W'(AGE_LIMIT));
    end
  end

  // ------------------------------------------------------------------ X10 core_stat user -> link (repeat)
  localparam int CRS_W = $bits(core_stat_t);
  logic [CRS_W-1:0] crs_bus;
  logic             crs_upd;
  logic             crs_busy_unused;
  logic [7:0]       crs_tag_unused;
  logic [AGE_W-1:0] crs_age_q;

  cdc_bus #(.W(CRS_W), .HOLD_CYC(H_C2L), .REPEAT(1'b1)) u_cdc_bus_corestat (
    .src_clk(user_clk), .src_rst(user_rst), .src_data(CRS_W'(core_stat_in)), .src_load(1'b0), .src_busy(crs_busy_unused),
    .dst_clk(link_clk), .dst_rst(link_rst), .dst_data(crs_bus), .dst_upd(crs_upd), .dst_tag(crs_tag_unused));
  assign core_stat = core_stat_t'(crs_bus);

  always_ff @(posedge link_clk) begin
    if (link_rst) begin
      crs_age_q   <= AGE_W'(AGE_LIMIT + 1);
      core_age_ok <= 1'b0;
    end else begin
      if (crs_upd) crs_age_q <= '0;
      else if (crs_age_q != AGE_W'(AGE_LIMIT + 1)) crs_age_q <= crs_age_q + 1'b1;
      core_age_ok <= crs_upd || (crs_age_q < AGE_W'(AGE_LIMIT));
    end
  end

endmodule
