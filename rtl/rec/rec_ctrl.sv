// =============================================================================
// rec_ctrl.sv — recorder controller (WP2): command executor + session state machine.
//
// Domain  : link_clk (200 MHz), one reset (`rst` = rec_rst, synchronous, active 1).
//           Everything coming from the other domains arrives through rec_cdc_bridge
//           (ui_stat / core_stat + `*_age_ok`); this module never reads a foreign clock.
// Spec    : STAND/RECORDER_SPEC.md 1.2, 1.3 (state table), 2.3 / 2.4 (commands, ERR codes),
//           2.6 (events), 4.3 (age rule), 4.4 (ports of WP2), 6 (log_req), 7 (power fail).
// Protocol: command side  : cmd_valid/cmd_ready, one command at a time; cmd_ready is a register
//                           and drops for the time the reply is built.  A reply is one or more
//                           messages; the message FIFO (16 deep) keeps reply/event order.
//           message side  : msg_valid/msg_ready (valid holds until ready), `snap` is registered
//                           together with msg_* (coherent at the accept cycle).  For result events
//                           (MK_EV_RR/RP/RV/RF/RM/RD) `snap` shows the result image latched at the
//                           end of the session (RES? repeats it), for all others the live image.
//
// Message encoding (see INTEGRATION_NOTES.md "WP2"):
//   MK_OK        code = rec_op_e of the command, arg0 = echo value (sid / i / step / n ...)
//   MK_ERR       code = ERR code
//   MK_PROF_LINE arg0 = {i[7:0], 24'h0, step[31:0]}, arg1 = {24'h0, dur[39:0]}
//   MK_EV_S      arg0 = {56'h0, md[3:0], st[3:0]}, arg1 = {32'h0, t_ms}
//   MK_EV_E      arg0 = {36'h0, st[3:0], fl[7:0], cd[7:0], ce[7:0]}  (st = state at the event)
//   MK_EV_RF     arg0 = ms (session time from ctrl_start to DONE), arg1 = ok
//   MK_SMARTQ    arg0 = {48'h0, log_status}
//   others       everything from `snap`
//
// Decisions use a snapshot only while its `*_age_ok` is 1 (spec 4.3).  Pulses are 1 clock.
// =============================================================================
module rec_ctrl
  import rec_pkg::*;
#(
  parameter bit SIM_FAST = 1'b0          // 1: "millisecond" = 200 link clocks
) (
  input  logic         clk,              // link
  input  logic         rst,              // link (rec_rst)

  // ---- command parser (link)
  input  logic         cmd_valid,        // link
  output logic         cmd_ready,        // link
  input  logic [5:0]   cmd_op,           // link  rec_op_e
  input  logic [3:0]   cmd_sub,          // link  MODE: 0 RT / 1 BP; PROF END: 0 HOLD / 1 STOP
  input  logic [1:0]   cmd_nargs,        // link
  input  logic [63:0]  cmd_arg0,         // link
  input  logic [63:0]  cmd_arg1,         // link
  input  logic [63:0]  cmd_arg2,         // link
  input  logic [7:0]   cmd_err,          // link  (OP_PERR)

  // ---- keys / VIO (link, one-clock pulses after debouncing)
  input  logic         key_rec,          // link
  input  logic         key_stop,         // link
  input  logic         key_pf,           // link
  input  logic         vio_start,        // link
  input  logic         vio_stop,         // link
  input  logic         vio_verify,       // link
  input  logic         key_page,         // link

  // ---- to rec_tx_fmt (link)
  output logic         msg_valid,        // link
  input  logic         msg_ready,        // link
  output logic [4:0]   msg_kind,         // link  rec_msg_kind_e
  output logic [7:0]   msg_code,         // link
  output logic [63:0]  msg_arg0,         // link
  output logic [63:0]  msg_arg1,         // link
  output rec_snap_t    snap,             // link
  output logic         tel_tick,         // link

  // ---- to the core (X7, X8) and reset (link)
  output logic         ctrl_start,       // link
  output logic         ctrl_stop,        // link
  output logic         verify_mode,      // link
  output logic         format_req,       // link
  output logic [31:0]  verify_cmds,      // link
  output logic         core_rst_req,     // link  (1 ms pulse -> core_rst_n)
  output logic         log_req,          // link  (X8)

  // ---- to the bridge (X1, X2, X2a, X2b) (link)
  output rec_cfg_t     cfg,              // link
  output logic [31:0]  step,             // link
  output logic         step_load,        // link
  output logic [40:0]  start_cfg,        // link  {len_blocks[39:0], prof_en}
  output logic         start_load,       // link
  output logic         src_stop,         // link
  output logic         src_pf,           // link
  output logic         ui_clear,         // link
  output logic         mt_start,         // link
  output logic         dtest_start,      // link

  // ---- from the bridge / XADC (link)
  input  ui_stat_t     ui_stat,          // link
  input  logic         ui_age_ok,        // link
  input  core_stat_t   core_stat,        // link
  input  logic         core_age_ok,      // link
  input  xadc_t        xadc,             // link
  input  logic         link_lost,        // link  sticky flag of link_bringup (direct from nvme_host_top)
  input  logic         link_fail,        // link  sticky flag of link_bringup

  // ---- profile table, port A of tdp_ram_2clk (link)
  output logic         prof_we,          // link
  output logic [5:0]   prof_addr,        // link
  output logic [71:0]  prof_wdata,       // link  {step[31:0], dur[39:0]}
  input  logic [71:0]  prof_rdata,       // link  (latency 2)

  output logic [3:0]   led,              // link  (1 = on)
  output logic [31:0]  t_ms              // link
);

  // ------------------------------------------------------------------ constants
  localparam int unsigned MS_CYC    = SIM_FAST ? 200 : 200_000;  // link clocks per millisecond
  localparam int unsigned CLR_DLY   = 1024;       // ui_clear -> start of a LOCAL test (>= 2 us)
  localparam int unsigned SETTLE    = 64;         // start_load -> first src_stop/src_pf
  localparam int unsigned FRESH_CYC = 256;        // core reset release -> snapshot counted as fresh
  localparam int unsigned CRST_TMO  = 2001 * MS_CYC;   // CORE_RST: 1 ms reset + 2 s wait
  localparam int unsigned WD_BRING  = 3000 * MS_CYC;   // core status stale in BRINGUP (REC start may wait for the SSD)
  localparam int unsigned WD_RUN    = 20 * MS_CYC;     // core status stale in RUN .. RELEASE (review fix C)
  localparam int unsigned WD_PROG   = 10000 * MS_CYC;  // SRC_STOP / LOCAL stop without any progress (review fix D); > a paused SSD (core io_timeout 30 s is the real bound)
  localparam int unsigned WD_UI     = 10 * MS_CYC;     // ui status stale in SRC_STOP / LOCAL stop (review fix D)
  localparam int unsigned DT_WAIT   = 512;        // DTEST: stop -> src_done is trusted
  localparam logic [31:0] STEP_DEF  = 32'h1000_0000;  // 100 MB/s at 100 MHz ui_clk
  localparam int unsigned MQ_D      = 16;

  // result-event kinds are the only kinds that replace `snap` by the latched image
  function automatic logic is_res_kind(input logic [4:0] k);
    return (k == MK_EV_RR) || (k == MK_EV_RP) || (k == MK_EV_RV) || (k == MK_EV_RF) ||
           (k == MK_EV_RM) || (k == MK_EV_RD);
  endfunction

  // ------------------------------------------------------------------ registers
  rec_state_e   st_q;
  rec_mode_e    md_q;
  rec_state_e   st_n;                  // temporaries of the single always_ff
  rec_mode_e    md_n;

  logic [63:0]  t64_q;
  logic [31:0]  t_ms_q;
  logic [17:0]  ms_div_q;
  logic         ms_tick_q;
  logic [15:0]  tel_cnt_q, sm_cnt_q;

  // configuration (changes only in IDLE)
  logic         bp_q;
  logic [31:0]  step_q, len_q, sid_q, sid_use_q, last_sid_q, rsid_q;
  logic         sid_set_q, rsid_user_q;
  logic [10:0]  ddr_mib_q, mt_mib_q;
  logic [15:0]  tel_ms_q, sm_ms_q;
  logic [6:0]   pn_q;
  logic [15:0]  prof_rep_q;
  logic         prof_end_q;
  logic         prof_en_q;
  logic [3:0]   lbads_q;
  logic         have_last_q;
  logic [47:0]  last_wr_q;

  // command stage
  logic         exec_q, xs_q;
  logic [5:0]   op_q;
  logic [3:0]   sub_q;
  logic [1:0]   nargs_q;
  logic [63:0]  a0_q, a1_q, a2_q;
  logic [7:0]   err_q;
  logic [7:0]   x_err;
  logic         calib_ok;
  logic         accept;
  logic         exec_n, pq_busy_n;
  // decode stage: x_err is registered one clock before the command executes (timing); the execute stage re-decodes
  // when state / mode / calibration changed in between
  logic         dec1_q, dec_q, chk_q, moved_q, dec_cal_q;
  logic         f_a0_hi48, f_a0_hi32, f_a0_zero, f_a0_bad_ddr, f_a0_lt10, f_a0_lt100, f_a0_ge_pn;
  logic         f_a1_zero, f_a1_hi24, f_a1_hi32, f_a2_zero, f_a2_hi24;
  logic [7:0]   x_err_q;
  rec_state_e   dec_st_q;
  rec_mode_e    dec_md_q;

  // profile readback
  logic         pq_busy_q;
  logic [6:0]   pq_i_q;
  logic [2:0]   pq_ph_q;

  // session
  logic         errored_q, src_stopped_q, pf_late_q, pf_inb_q, pf_src_q, pf_lov_q;
  logic [1:0]   sp_kind_q;             // pending source pulse: 1 stop, 2 pf
  logic [6:0]   settle_q;
  logic [2:0]   rs_q;                  // REC start sequence in RUN
  logic [47:0]  base_q;                // beats_base
  logic [63:0]  t0_q, t_run_q, t_start_q, t_done_q, lt0_q;
  logic [63:0]  tab_q [1:6];
  logic [6:1]   tv_q;
  logic [3:0]   fin_q;
  logic         fin_act_q;
  logic [4:0]   last_res_kind_q;
  logic [1:0]   rd_kind_q;             // 0 LAST, 1 N, 2 END
  logic         vc_loaded_q;
  logic [15:0]  pad0_q;
  logic [47:0]  rp_pnd_q;
  logic [31:0]  rp_pdd_q, rp_prg_q;
  logic [31:0]  lt_cnt_q;
  logic         lt_stop_q;
  logic [31:0]  crst_cnt_q;
  logic [31:0]  rf_ms_q, ms0_q;
  logic         rf_ok_q;
  logic         done_led_q;
  logic [1:0]   page_q;
  logic         calib_prev_q;
  logic [2:0]   drain_hold_q;
  logic         ran_q;                 // session reached RUN
  logic [31:0]  wd_q;                  // core snapshot stale for this many clocks
  logic [31:0]  prog_q;                // clocks without progress of wr_blocks / bytes_done / fill (reset on state change, STOP)
  logic [31:0]  uwd_q;                 // ui status stale for this many clocks
  logic [142:0] prog_sig_q;
  logic         prog_chg_q;
  logic         prog_clr;
  logic [8:0]   vm_hold_q;             // blocks READ right after verify_mode fell
  logic         src_done_eff, link_err, wd_exp, idle_err;
  logic         lw_err_q, idle_err_q;
  logic         cs_err_q, cs_run_q, cs_idle_q, cs_fl_q, cs_sd_q, cs_done_q;   // registered decode of the core status (valid = core_age_ok)

  // capacity engine
  logic         cap_busy_q, cap_valid_q;
  logic [4:0]   cap_cnt_q;
  logic [63:0]  cap_val_q, cap_lim_q, len_slots_q;

  // derived arithmetic pipelines (registered, recomputed every clock)
  logic [47:0]  d_beats_q;
  // pipelined arithmetic of the finalisation / READ start (registered every clock, inputs are static when used)
  logic [63:0]  tdiff_q [1:6];
  logic [6:1]   tgt_q;
  logic [63:0]  dur_run_q, dur_fmt_q;
  logic [31:0]  rp_pad_q;
  logic [39:0]  rp_lpos_q;
  logic [31:0]  vc_last_q;
  logic [47:0]  lw31_q;
  logic         drain_eq_q;
  logic [31:0]  rng_q, prg_q;
  logic [47:0]  pnd_q;

  // pending events
  logic         pend_b_q, pend_e_q, pend_res_q;
  logic [27:0]  ev_e_q;

  // message FIFO
  logic [4:0]   mq_k  [MQ_D];
  logic [7:0]   mq_c  [MQ_D];
  logic [63:0]  mq_a0 [MQ_D];
  logic [63:0]  mq_a1 [MQ_D];
  logic         mq_r  [MQ_D];          // entry is a result-event kind (snap = latched image)
  logic [3:0]   mq_wp_q, mq_rp_q;
  logic [4:0]   mq_cnt_q;
  logic [4:0]   mq_occ;                // entries in the FIFO + in the two staging registers
  // staging registers of the two push ports (timing: the push data is computed from the command / state machine
  // logic, the FIFO is written one clock later)
  logic         sa_v_q, sb_v_q;
  logic [4:0]   sa_k_q, sb_k_q;
  logic [7:0]   sa_c_q, sb_c_q;
  logic [63:0]  sa_a0_q, sa_a1_q, sb_a0_q, sb_a1_q;
  logic         msg_res_q;             // msg_kind is a result-event kind
  // pre-stage between the FIFO and the output registers (timing: the snap select depends only on registers)
  logic         pre_v_q, pre_r_q;
  logic [4:0]   pre_k_q;
  logic [7:0]   pre_c_q;
  logic [63:0]  pre_a0_q, pre_a1_q;

  // !S queue (review fix I): state-change events wait here while the message FIFO has no room on port B
  logic [7:0]   sq_st [8];
  logic [31:0]  sq_t  [8];
  logic [2:0]   sq_wp_q, sq_rp_q;
  logic [3:0]   sq_cnt_q;
  logic         b_room, s_new, sq_push, sq_pop;

  rec_snap_t    snap_live, res_q;

  // single-cycle temporaries (blocking, used only inside the main always_ff)
  logic         pa_v, pb_v;
  logic [4:0]   pa_k, pb_k;
  logic [7:0]   pa_c, pb_c;
  logic [63:0]  pa_a0, pa_a1, pb_a0, pb_a1;
  // start / stop requests: produced by the command stage / keys (gx_*), registered (go_*_r), consumed by the state
  // machine one clock later (timing: the command decode is not in the state machine's path)
  logic         gx_rec, gx_prof, gx_read, gx_mon, gx_format, gx_mem, gx_dtest;
  logic [1:0]   gx_rdk;
  logic [63:0]  gx_n;
  logic         gx_ev_stop, gx_ev_pf, gx_ev_pf_key, gx_ev_abort;
  logic         go_rec_r, go_prof_r, go_read_r, go_mon_r, go_format_r, go_mem_r, go_dtest_r;
  logic [1:0]   go_rdk_r;
  logic [63:0]  go_n_r;
  logic         ev_stop_r, ev_pf_r, ev_pf_key_r, ev_abort_r;
  logic         go_rec, go_prof, go_read, go_mon, go_format, go_mem, go_dtest;
  logic [1:0]   go_rdk;
  logic [63:0]  go_n;
  logic         ev_stop, ev_pf, ev_pf_key, ev_abort;
  assign go_rec = go_rec_r;       assign go_prof = go_prof_r;     assign go_read = go_read_r;
  assign go_mon = go_mon_r;       assign go_format = go_format_r; assign go_mem = go_mem_r;
  assign go_dtest = go_dtest_r;   assign go_rdk = go_rdk_r;       assign go_n = go_n_r;
  assign ev_stop = ev_stop_r;     assign ev_pf = ev_pf_r;         assign ev_pf_key = ev_pf_key_r;
  assign ev_abort = ev_abort_r;
  // state change seen one clock after it happened (st_prev_q): !S event, progress watchdog clear, ERROR entry actions
  rec_state_e   st_prev_q;
  logic         s_chg;
  assign s_chg = (st_q != st_prev_q);
  // deferred result image capture (one clock after the state machine decided) and registered helpers
  logic         rcap_loc_q, rcap_clr_q, rcap_core_q;
  rec_mode_e    rcap_md_q;
  logic [63:0]  ldur_q;
  logic         nsze_nz_q;
  logic         prog_to_q, uwd_to_q;       // registered watchdog expiry (prog_q is cleared one clock after a state change: gated by s_chg)
  logic         wr0_q;
  logic [39:0]  lpos_sub_q;
  logic         do_cap_q;
  rec_snap_t    res_core_img, res_loc_img;
  logic         do_capture;
  logic [31:0]  sid_use_n;
  logic         errored_n;

  // ------------------------------------------------------------------ small helpers
  assign calib_ok = ui_age_ok && ui_stat.ui_ok && ui_stat.calib;

  function automatic logic [7:0] refuse_code(input rec_state_e s);
    return (s == ST_CORE_RST || s == ST_ERROR) ? 8'(ERR_CORE) : 8'(ERR_STATE);
  endfunction

  function automatic logic pf_ok(input rec_state_e s, input rec_mode_e m);
    case (s)
      ST_BRINGUP, ST_RUN:         return (m != MD_PF);
      ST_SRC_STOP, ST_CORE_STOP:  return (m == MD_REC);
      default:                    return 1'b0;
    endcase
  endfunction

  function automatic logic stop_ok(input rec_state_e s);
    return (s == ST_BRINGUP) || (s == ST_RUN) || (s == ST_LOCAL);
  endfunction

  function automatic logic [4:0] res_kind_of(input rec_mode_e m);
    case (m)
      MD_REC:     return MK_EV_RR;
      MD_PF:      return MK_EV_RP;
      MD_READ:    return MK_EV_RV;
      MD_FORMAT:  return MK_EV_RF;
      MD_MEMTEST: return MK_EV_RM;
      MD_DTEST:   return MK_EV_RD;
      default:    return MK_NONE;
    endcase
  endfunction

  task automatic push_a(input logic [4:0] k, input logic [7:0] c, input logic [63:0] a0, input logic [63:0] a1);
    pa_v = 1'b1; pa_k = k; pa_c = c; pa_a0 = a0; pa_a1 = a1;
  endtask

  task automatic push_b(input logic [4:0] k, input logic [7:0] c, input logic [63:0] a0, input logic [63:0] a1);
    pb_v = 1'b1; pb_k = k; pb_c = c; pb_a0 = a0; pb_a1 = a1;
  endtask

  // ------------------------------------------------------------------ command decode: ERR code
  // 0 = the command is accepted in the current state with these arguments.
  always_comb begin
    x_err = 8'h00;
    case (op_q)
      OP_NOP, OP_PING, OP_STAT, OP_CFGQ, OP_RESQ, OP_SMARTQ, OP_PROFQ, OP_DDRQ, OP_ABORT: x_err = 8'h00;
      OP_PERR:   x_err = (err_q == 8'h00) ? 8'(ERR_SYNTAX) : err_q;
      OP_TEL:    if (f_a0_hi48 || f_a0_lt10) x_err = ERR_RANGE;
      OP_SMART:  if (f_a0_hi48 || f_a0_lt100) x_err = ERR_RANGE;
      OP_MODE, OP_PROF_CLR, OP_PROF_END, OP_READ_SID: begin
        if (st_q != ST_IDLE) x_err = refuse_code(st_q);
        else if (op_q == OP_READ_SID && f_a0_hi32) x_err = ERR_RANGE;
      end
      OP_LEN, OP_SID: begin
        if (st_q != ST_IDLE) x_err = refuse_code(st_q);
        else if (f_a0_hi32) x_err = ERR_RANGE;
      end
      OP_DDRSZ: begin
        if (st_q != ST_IDLE) x_err = refuse_code(st_q);
        else if (f_a0_bad_ddr) x_err = ERR_RANGE;
      end
      OP_RATE: begin
        if (!(st_q == ST_IDLE || (st_q == ST_RUN && md_q == MD_REC && !prof_en_q) ||
              (st_q == ST_LOCAL && md_q == MD_DTEST))) x_err = refuse_code(st_q);
        else if (f_a0_hi32) x_err = ERR_RANGE;
      end
      OP_PROF_ADD: begin
        if (st_q != ST_IDLE) x_err = refuse_code(st_q);
        else if (pn_q == 7'(PROF_N)) x_err = ERR_PROF_FULL;
        else if (f_a0_hi32 || f_a1_zero || f_a1_hi24) x_err = ERR_RANGE;
      end
      OP_PROF_SET: begin
        if (st_q != ST_IDLE) x_err = refuse_code(st_q);
        else if (f_a0_ge_pn || f_a1_hi32 || f_a2_zero || f_a2_hi24) x_err = ERR_RANGE;
      end
      OP_PROF_REP: begin
        if (st_q != ST_IDLE) x_err = refuse_code(st_q);
        else if (f_a0_hi48) x_err = ERR_RANGE;
      end
      OP_REC, OP_REC_PROF: begin
        if (st_q != ST_IDLE) x_err = refuse_code(st_q);
        else if (!calib_ok) x_err = ERR_NOCAL;
        else if (op_q == OP_REC_PROF && pn_q == 0) x_err = ERR_PROF_EMPTY;
      end
      OP_STOP: if (!stop_ok(st_q)) x_err = refuse_code(st_q);
      OP_PFAIL: if (!pf_ok(st_q, md_q)) x_err = refuse_code(st_q);
      OP_READ_LAST, OP_READ_N, OP_READ_END: begin
        if (st_q != ST_IDLE) x_err = refuse_code(st_q);
        else if (!calib_ok) x_err = ERR_NOCAL;
        else if (op_q == OP_READ_LAST && !have_last_q) x_err = ERR_NOREC;
        else if (op_q == OP_READ_N && (f_a0_zero || f_a0_hi32)) x_err = ERR_RANGE;
      end
      OP_FORMAT, OP_MON: if (st_q != ST_IDLE) x_err = refuse_code(st_q);
      OP_MEMTEST, OP_DTEST: begin
        if (st_q != ST_IDLE) x_err = refuse_code(st_q);
        else if (!calib_ok) x_err = ERR_NOCAL;
        else if (op_q == OP_MEMTEST && nargs_q != 0 && f_a0_bad_ddr) x_err = ERR_RANGE;
      end
      default: x_err = ERR_UNKNOWN;
    endcase
  end

  // ------------------------------------------------------------------ live snapshot image
  always_comb begin
    snap_live = '0;
    snap_live.t_ms     = t_ms_q;
    snap_live.st       = st_q;
    snap_live.md       = md_q;
    snap_live.cok      = core_age_ok;
    snap_live.uok      = ui_age_ok;
    snap_live.cs       = core_stat.seq_state;
    snap_live.ce       = core_stat.error;
    snap_live.cd       = core_stat.error_detail;
    snap_live.fl       = core_stat.flags;
    snap_live.lk       = {core_stat.link_width, core_stat.gen2, core_stat.link_up};
    snap_live.cal      = ui_stat.calib;
    snap_live.stp      = ui_stat.step;
    snap_live.seg      = {2'b00, ui_stat.seg_idx};
    snap_live.rep      = ui_stat.rep_cnt;
    snap_live.src      = ui_stat.src_blocks;
    snap_live.wr       = ui_stat.wr_blocks;
    snap_live.drp      = ui_stat.drop_blocks;
    snap_live.ddr      = {1'b0, ui_stat.fill_bytes};
    snap_live.ddrm     = {1'b0, ui_stat.fill_max_bytes};
    snap_live.bpm      = ui_stat.bp_max_beats;
    snap_live.rng      = rng_q;
    snap_live.ssd      = core_stat.bytes_done;
    snap_live.cmd      = core_stat.cmds_done[31:0];
    snap_live.cer      = core_stat.cmd_errors;
    snap_live.lat      = core_stat.max_latency;
    snap_live.vb       = ui_stat.vrf.vb;
    snap_live.ve       = ui_stat.vrf.ve;
    snap_live.tk       = core_stat.smart.tk;
    snap_live.cw       = core_stat.smart.cw;
    snap_live.smo      = core_stat.log_cnt;
    snap_live.sme      = core_stat.log_err_cnt;
    snap_live.xt       = xadc.temp;
    snap_live.xvi      = xadc.vccint;
    snap_live.xva      = xadc.vccaux;
    snap_live.xvb      = xadc.vccbram;
    snap_live.cfg_bp   = bp_q;
    snap_live.cfg_stp  = step_q;
    snap_live.cfg_len  = len_q;
    snap_live.cfg_sid  = sid_q;
    snap_live.cfg_ddr  = {5'b0, ddr_mib_q};
    snap_live.cfg_tel  = tel_ms_q;
    snap_live.cfg_sm   = sm_ms_q;
    snap_live.cfg_pn   = {1'b0, pn_q};
    snap_live.cfg_pr   = prof_rep_q;
    snap_live.cfg_pe   = prof_end_q;
    snap_live.cfg_rsid = rsid_user_q ? rsid_q : last_sid_q;
    snap_live.smart    = core_stat.smart;
    snap_live.vrf      = ui_stat.vrf;
    snap_live.end_found = ui_stat.end_found;
    snap_live.mte      = ui_stat.mt_err_cnt;
    snap_live.mta      = {3'b000, ui_stat.mt_first_err_addr};
    snap_live.mt_bytes = {1'b0, mt_mib_q, 20'b0};
    snap_live.dt_he    = ui_stat.dt_hdr_err;
    snap_live.dt_wr    = ui_stat.wr_blocks;
    snap_live.pad_blocks   = ui_stat.pad_blocks;
    snap_live.trunc_blocks = ui_stat.trunc_blocks;
    snap_live.dur      = 64'd0;
    snap_live.sid      = sid_use_q;
    snap_live.rtd      = core_stat.rtd3e_us;
  end

  // result images (captured one clock after the state machine decided; all inputs are static then)
  always_comb begin
    res_core_img = snap_live;
    res_core_img.st = ST_CORE_STOP; res_core_img.md = rcap_md_q;
    res_core_img.rp_src = pf_src_q; res_core_img.rp_inb = pf_inb_q; res_core_img.rp_lat = pf_late_q;
    res_core_img.rp_lov = pf_lov_q;
    res_core_img.rp_pnd = rp_pnd_q; res_core_img.rp_pdd = rp_pdd_q; res_core_img.rp_prg = rp_prg_q;
    res_core_img.rp_pad = rp_pad_q;
    res_core_img.rp_ok  = (core_stat.error == 8'h00) && !core_stat.flags[0];
    res_core_img.rp_lpos = rp_lpos_q;
    res_loc_img = snap_live;
    res_loc_img.st = ST_LOCAL; res_loc_img.md = rcap_md_q; res_loc_img.dur = ldur_q;
  end

  // ------------------------------------------------------------------ the machine
  always_ff @(posedge clk) begin : main
    integer k;
    // ---- defaults: pulses
    step_load   <= 1'b0;
    rcap_loc_q  <= 1'b0;
    rcap_core_q <= 1'b0;
    start_load  <= 1'b0;
    src_stop    <= 1'b0;
    src_pf      <= 1'b0;
    ui_clear    <= 1'b0;
    mt_start    <= 1'b0;
    dtest_start <= 1'b0;
    log_req     <= 1'b0;
    tel_tick    <= 1'b0;
    prof_we     <= 1'b0;
    ms_tick_q   <= 1'b0;
    pa_v = 1'b0; pb_v = 1'b0;
    pa_k = '0; pb_k = '0; pa_c = '0; pb_c = '0; pa_a0 = '0; pa_a1 = '0; pb_a0 = '0; pb_a1 = '0;
    gx_rec = 1'b0; gx_prof = 1'b0; gx_read = 1'b0; gx_mon = 1'b0; gx_format = 1'b0;
    gx_mem = 1'b0; gx_dtest = 1'b0; gx_rdk = 2'd0; gx_n = 64'd0;
    gx_ev_stop = 1'b0; gx_ev_pf = 1'b0; gx_ev_pf_key = 1'b0; gx_ev_abort = 1'b0;
    do_capture = 1'b0;
    st_n = st_q; md_n = md_q;
    errored_n = errored_q;
    exec_n = 1'b0; pq_busy_n = pq_busy_q;
    sid_use_n = sid_set_q ? sid_q : (sid_q + 32'd1);

    st_prev_q <= st_q;
    ldur_q    <= t64_q - lt0_q;
    nsze_nz_q <= (core_stat.nsze != 64'd0);
    prog_to_q <= (prog_q >= WD_PROG);
    uwd_to_q  <= (uwd_q >= WD_UI);

    // ---- time base
    t64_q <= t64_q + 64'd1;
    if (ms_div_q == 18'(MS_CYC - 1)) begin
      ms_div_q <= 18'd0; t_ms_q <= t_ms_q + 32'd1; ms_tick_q <= 1'b1;
    end else ms_div_q <= ms_div_q + 18'd1;

    // ---- arithmetic pipelines (all registered)
    d_beats_q  <= core_stat.beats_in[47:0] - base_q;
    drain_eq_q <= (d_beats_q == {ui_stat.wr_blocks[39:0], 8'h00});
    for (k = 1; k <= 6; k = k + 1) begin
      tdiff_q[k] <= tab_q[k] - t0_q;
      tgt_q[k]   <= (tab_q[k] > t0_q);
    end
    dur_run_q  <= t_done_q - t_run_q;
    dur_fmt_q  <= t_done_q - t_start_q;
    rp_pad_q   <= {16'b0, (ui_stat.pad_blocks + ui_stat.trunc_blocks)} - {16'b0, pad0_q};
    wr0_q      <= (ui_stat.wr_blocks == 48'd0);
    lpos_sub_q <= ui_stat.wr_blocks[39:0] - 40'd1;
    rp_lpos_q  <= wr0_q ? 40'd0 : lpos_sub_q;
    lw31_q     <= last_wr_q + 48'd31;
    vc_last_q  <= (|lw31_q[47:37]) ? 32'hFFFF_FFFF : lw31_q[36:5];       // (last_wr + 31) >> 5, saturated
    rng_q      <= d_beats_q[31:0] - core_stat.bytes_done[35:4];
    prg_q      <= {d_beats_q[27:0], 4'b0000} - core_stat.bytes_done[31:0];
    pnd_q      <= {ui_stat.wr_blocks[35:0], 12'h000} - core_stat.bytes_done[47:0];
    src_done_eff = ui_stat.src_done || (rs_q == 3'd5);   // start cancelled by an early STOP/PF: nothing to wait for
    if (core_age_ok && ui_age_ok && src_done_eff && ui_stat.ddr_idle && drain_eq_q)
      drain_hold_q <= (drain_hold_q == 3'd7) ? drain_hold_q : drain_hold_q + 3'd1;
    else drain_hold_q <= 3'd0;
    if (core_age_ok) lbads_q <= core_stat.lbads;
    if (verify_mode) vm_hold_q <= 9'd256; else if (vm_hold_q != 9'd0) vm_hold_q <= vm_hold_q - 9'd1;
    // core status watchdog (BRINGUP..RELEASE) and link flags
    if (core_age_ok || st_q == ST_IDLE || st_q == ST_LOCAL || st_q == ST_ERROR || st_q == ST_CORE_RST) wd_q <= 32'd0;
    else if (wd_q != 32'hFFFF_FFFF) wd_q <= wd_q + 32'd1;
    wd_exp   = (wd_q >= ((st_q == ST_BRINGUP) ? WD_BRING : WD_RUN));
    // progress / ui-status watchdogs (SRC_STOP, LOCAL after STOP): registered change detector, no absolute timeout
    prog_sig_q <= {ui_stat.wr_blocks, core_stat.bytes_done, ui_stat.fill_bytes};
    prog_chg_q <= ({ui_stat.wr_blocks, core_stat.bytes_done, ui_stat.fill_bytes} != prog_sig_q);
    if (ui_age_ok) uwd_q <= 32'd0; else if (uwd_q != 32'hFFFF_FFFF) uwd_q <= uwd_q + 32'd1;
    link_err = link_lost || link_fail;
    cs_err_q  <= core_age_ok && core_stat.seq_state == CS_ERROR;
    cs_run_q  <= core_age_ok && core_stat.seq_state == CS_RUN;
    cs_idle_q <= core_age_ok && core_stat.seq_state == CS_IDLE;
    cs_fl_q   <= core_age_ok && (core_stat.seq_state == CS_FLUSH || core_stat.seq_state == CS_SHUTDOWN || core_stat.seq_state == CS_DONE);
    cs_sd_q   <= core_age_ok && (core_stat.seq_state == CS_SHUTDOWN || core_stat.seq_state == CS_DONE);
    cs_done_q <= core_age_ok && core_stat.seq_state == CS_DONE;
    lw_err_q <= link_err || wd_exp;       // registered (timing): the error is acted on one clock later
    idle_err = ran_q && cs_idle_q &&
               (st_q == ST_RUN || st_q == ST_SRC_STOP || st_q == ST_CORE_STOP);
    idle_err_q <= idle_err;

    // ---- capacity engine: cap_val_q = capacity in 128 KiB slots (nsze >> (17 - lbads))
    if (cap_busy_q) begin
      if (cap_cnt_q == 5'd0) begin cap_busy_q <= 1'b0; cap_valid_q <= 1'b1; end
      else begin cap_val_q <= {1'b0, cap_val_q[63:1]}; cap_cnt_q <= cap_cnt_q - 5'd1; end
    end

    // ---- command stage: accept
    accept = cmd_valid && cmd_ready;
    if (accept) begin
      dec1_q <= 1'b1; xs_q <= 1'b0;
      op_q <= cmd_op; sub_q <= cmd_sub; nargs_q <= cmd_nargs;
      a0_q <= cmd_arg0; a1_q <= cmd_arg1; a2_q <= cmd_arg2; err_q <= cmd_err;
    end

    // ---- command stage: operand flags (compares of the 64-bit arguments), registered
    if (dec1_q) begin
      dec1_q <= 1'b0; dec_q <= 1'b1;
      f_a0_hi48 <= (a0_q[63:16] != 0);  f_a0_hi32 <= (a0_q[63:32] != 0);  f_a0_zero <= (a0_q == 0);
      f_a0_bad_ddr <= (a0_q == 0) || (a0_q > 64'h400);
      f_a0_lt10 <= (a0_q[15:0] != 0 && a0_q[15:0] < 16'h000A);  f_a0_lt100 <= (a0_q[15:0] != 0 && a0_q[15:0] < 16'h0064);
      f_a0_ge_pn <= (a0_q >= {57'b0, pn_q});
      f_a1_zero <= (a1_q == 0);  f_a1_hi24 <= (a1_q[63:40] != 0);  f_a1_hi32 <= (a1_q[63:32] != 0);
      f_a2_zero <= (a2_q == 0);  f_a2_hi24 <= (a2_q[63:40] != 0);
    end

    // ---- command stage: decode (register the ERR code of x_err)
    if (dec_q) begin
      dec_q <= 1'b0; chk_q <= 1'b1;
      x_err_q <= x_err; dec_st_q <= st_q; dec_md_q <= md_q; dec_cal_q <= calib_ok;
    end
    // ---- command stage: did state / mode / calibration move since the decode?  (registered: not in the execute path)
    if (chk_q) begin
      chk_q <= 1'b0; exec_q <= 1'b1;
      moved_q <= (dec_st_q != st_q || dec_md_q != md_q || dec_cal_q != calib_ok);
    end

    // ---- command stage: execute (one cycle; two-part replies take a second one)
    if (exec_q) begin
      exec_n = 1'b0;
      if (!xs_q && moved_q) begin
        dec_q <= 1'b1;                  // the state moved since the decode: decode again, nothing is executed
      end else if (x_err_q != 8'h00) begin
        push_a(MK_ERR, x_err, 64'd0, 64'd0);
      end else begin
        case (op_q)
          OP_NOP: ;
          OP_PING:   push_a(MK_OK, op_q, 64'd0, 64'd0);
          OP_STAT: begin
            if (!xs_q) begin push_a(MK_TEL, 8'd0, 64'd0, 64'd0); exec_n = 1'b1; xs_q <= 1'b1; end
            else push_a(MK_OK, op_q, 64'd0, 64'd0);
          end
          OP_CFGQ:   push_a(MK_CFGQ, op_q, 64'd0, 64'd0);
          OP_RESQ: begin
            if (!xs_q && last_res_kind_q != MK_NONE) begin
              push_a(last_res_kind_q, 8'd0, (last_res_kind_q == MK_EV_RF) ? {32'b0, rf_ms_q} : 64'd0,
                     (last_res_kind_q == MK_EV_RF) ? {63'b0, rf_ok_q} : 64'd0);
              exec_n = 1'b1; xs_q <= 1'b1;
            end else push_a(MK_OK, op_q, 64'd0, 64'd0);
          end
          OP_SMARTQ: push_a(MK_SMARTQ, op_q, {48'b0, core_stat.log_status}, 64'd0);
          OP_DDRQ:   push_a(MK_DDRQ, op_q, 64'd0, 64'd0);
          OP_TEL: begin
            tel_ms_q <= a0_q[15:0]; tel_cnt_q <= 16'd0; push_a(MK_OK, op_q, a0_q, 64'd0);
          end
          OP_SMART: begin
            sm_ms_q <= a0_q[15:0]; sm_cnt_q <= 16'd0; push_a(MK_OK, op_q, a0_q, 64'd0);
          end
          OP_MODE: begin bp_q <= sub_q[0]; push_a(MK_OK, op_q, {63'b0, sub_q[0]}, 64'd0); end
          OP_RATE: begin
            step_q <= a0_q[31:0]; step <= a0_q[31:0]; step_load <= 1'b1;
            push_a(MK_OK, op_q, a0_q, 64'd0);
          end
          OP_LEN:  begin len_q <= a0_q[31:0]; push_a(MK_OK, op_q, a0_q, 64'd0); end
          OP_SID:  begin sid_q <= a0_q[31:0]; sid_set_q <= 1'b1; push_a(MK_OK, op_q, a0_q, 64'd0); end
          OP_DDRSZ: begin ddr_mib_q <= a0_q[10:0]; push_a(MK_OK, op_q, a0_q, 64'd0); end
          OP_PROF_CLR: begin pn_q <= 7'd0; push_a(MK_OK, op_q, 64'd0, 64'd0); end
          OP_PROF_ADD: begin
            prof_we <= 1'b1; prof_addr <= pn_q[5:0]; prof_wdata <= {a0_q[31:0], a1_q[39:0]};
            pn_q <= pn_q + 7'd1; push_a(MK_OK, op_q, {57'b0, pn_q}, 64'd0);
          end
          OP_PROF_SET: begin
            prof_we <= 1'b1; prof_addr <= a0_q[5:0]; prof_wdata <= {a1_q[31:0], a2_q[39:0]};
            push_a(MK_OK, op_q, a0_q, 64'd0);
          end
          OP_PROF_REP: begin prof_rep_q <= a0_q[15:0]; push_a(MK_OK, op_q, a0_q, 64'd0); end
          OP_PROF_END: begin prof_end_q <= sub_q[0]; push_a(MK_OK, op_q, {63'b0, sub_q[0]}, 64'd0); end
          OP_PROFQ: begin
            if (pn_q == 0) push_a(MK_OK, op_q, 64'd0, 64'd0);
            else begin pq_busy_n = 1'b1; pq_i_q <= 7'd0; pq_ph_q <= 3'd0; end
          end
          OP_REC, OP_REC_PROF: begin
            gx_rec = 1'b1; gx_prof = (op_q == OP_REC_PROF);
            push_a(MK_OK, op_q, {32'b0, sid_use_n}, 64'd0);
          end
          OP_STOP:  begin gx_ev_stop = 1'b1; push_a(MK_OK, op_q, 64'd0, 64'd0); end
          OP_PFAIL: begin gx_ev_pf = 1'b1; push_a(MK_OK, op_q, 64'd0, 64'd0); end
          OP_ABORT: begin gx_ev_abort = 1'b1; push_a(MK_OK, op_q, 64'd0, 64'd0); end
          OP_READ_SID: begin
            rsid_q <= a0_q[31:0]; rsid_user_q <= 1'b1; push_a(MK_OK, op_q, a0_q, 64'd0);
          end
          OP_READ_LAST: begin gx_read = 1'b1; gx_rdk = 2'd0; push_a(MK_OK, op_q, 64'd0, 64'd0); end
          OP_READ_N:    begin gx_read = 1'b1; gx_rdk = 2'd1; gx_n = a0_q; push_a(MK_OK, op_q, a0_q, 64'd0); end
          OP_READ_END:  begin gx_read = 1'b1; gx_rdk = 2'd2; push_a(MK_OK, op_q, 64'd0, 64'd0); end
          OP_FORMAT:    begin gx_format = 1'b1; push_a(MK_OK, op_q, 64'd0, 64'd0); end
          OP_MON:       begin gx_mon = 1'b1; push_a(MK_OK, op_q, 64'd0, 64'd0); end
          OP_MEMTEST: begin
            gx_mem = 1'b1; gx_n = (nargs_q != 0) ? a0_q : {53'b0, ddr_mib_q};
            push_a(MK_OK, op_q, gx_n, 64'd0);
          end
          OP_DTEST:     begin gx_dtest = 1'b1; push_a(MK_OK, op_q, 64'd0, 64'd0); end
          default: ;
        endcase
      end
      if (!exec_n) exec_q <= 1'b0;
    end else if (!dec_q && !dec1_q && !chk_q) begin
      // keys / VIO (a command in the same cycle wins)
      if (vio_start && vio_verify && st_q == ST_IDLE) begin
        if (calib_ok && have_last_q && vm_hold_q == 9'd0) begin gx_read = 1'b1; gx_rdk = 2'd0; end
      end else if ((key_rec || vio_start) && st_q == ST_IDLE && calib_ok) begin
        gx_rec = 1'b1; gx_prof = (pn_q != 7'd0);
      end
    end
    if (exec_n) exec_q <= 1'b1;

    // keys: STOP / PF
    if ((key_stop || vio_stop) && stop_ok(st_q)) gx_ev_stop = 1'b1;
    if (key_pf && pf_ok(st_q, md_q)) begin gx_ev_pf = 1'b1; gx_ev_pf_key = 1'b1; end

    // registered requests of the command stage
    go_rec_r <= gx_rec; go_prof_r <= gx_prof; go_read_r <= gx_read; go_mon_r <= gx_mon; go_format_r <= gx_format;
    go_mem_r <= gx_mem; go_dtest_r <= gx_dtest; go_rdk_r <= gx_rdk; go_n_r <= gx_n;
    ev_stop_r <= gx_ev_stop; ev_pf_r <= gx_ev_pf; ev_pf_key_r <= gx_ev_pf_key; ev_abort_r <= gx_ev_abort;

    // ---- profile readback sequence (PROF?)
    if (pq_busy_q) begin
      case (pq_ph_q)
        3'd0: begin prof_addr <= pq_i_q[5:0]; pq_ph_q <= 3'd1; end
        3'd1, 3'd2, 3'd3: pq_ph_q <= pq_ph_q + 3'd1;
        3'd4: begin
          if (mq_occ <= 5'd10) begin
            push_a(MK_PROF_LINE, 8'd0, {1'b0, pq_i_q, 24'b0, prof_rdata[71:40]}, {24'b0, prof_rdata[39:0]});
            pq_ph_q <= 3'd0;
            pq_i_q  <= pq_i_q + 7'd1;
            if (pq_i_q + 7'd1 == pn_q) pq_ph_q <= 3'd5;
          end
        end
        3'd5: begin
          if (mq_occ <= 5'd12) begin push_a(MK_OK, OP_PROFQ, 64'd0, 64'd0); pq_busy_n = 1'b0; end
        end
        default: pq_busy_n = 1'b0;
      endcase
    end
    pq_busy_q <= pq_busy_n;

    // ---- pending source pulse (ordering behind start_load)
    if (settle_q != 0) settle_q <= settle_q - 7'd1;
    if (sp_kind_q != 0 && settle_q == 0) begin
      if (sp_kind_q == 2'd1) src_stop <= 1'b1; else src_pf <= 1'b1;
      sp_kind_q <= 2'd0;
    end

    // ================================================================== state machine
    // common: observations of the core state used by several states
    case (st_q)
      // ------------------------------------------------------------------ IDLE
      ST_IDLE: begin
        if (go_rec || go_read || go_mon || go_format) begin
          st_n = ST_BRINGUP; ui_clear <= 1'b1; ctrl_start <= 1'b1; ctrl_stop <= 1'b0;
          errored_n = 1'b0; ran_q <= 1'b0; src_stopped_q <= 1'b0; pf_late_q <= 1'b0; pf_inb_q <= 1'b0;
          pf_src_q <= 1'b0; pf_lov_q <= 1'b0; sp_kind_q <= 2'd0; rs_q <= 3'd0; tv_q <= '0;
          cap_valid_q <= 1'b0; cap_busy_q <= 1'b0; vc_loaded_q <= 1'b0; fin_act_q <= 1'b0;
          done_led_q <= 1'b0; t_start_q <= t64_q; ms0_q <= t_ms_q; pend_res_q <= 1'b0;
          // WP8: no command limit in a REC / MON / FORMAT session: verify_cmds of an earlier READ would otherwise stop the
          // write engine after that many commands (core limit_hit applies to Writes too); READ below loads its own value
          verify_cmds <= 32'd0;
          if (go_rec) begin
            md_n = MD_REC; prof_en_q <= go_prof;
            sid_use_q <= sid_use_n; last_sid_q <= sid_use_n; sid_q <= sid_use_n; sid_set_q <= 1'b0;
            step <= step_q; step_load <= 1'b1;
          end else if (go_read) begin
            md_n = MD_READ; rd_kind_q <= go_rdk; verify_mode <= 1'b1;
            if (go_rdk == 2'd0) begin
              verify_cmds <= vc_last_q;       // registered every clock; last_wr_q is static in IDLE
              vc_loaded_q <= 1'b1;
            end else if (go_rdk == 2'd1) begin
              verify_cmds <= go_n[31:0]; vc_loaded_q <= 1'b1;
            end else verify_cmds <= 32'd0;
          end else if (go_mon) md_n = MD_MON;
          else begin md_n = MD_FORMAT; format_req <= 1'b1; end
        end else if (go_mem || go_dtest) begin
          st_n = ST_LOCAL; ui_clear <= 1'b1; lt_cnt_q <= 32'd0; lt_stop_q <= 1'b0; lt0_q <= t64_q;
          pend_res_q <= 1'b0;
          if (go_mem) begin md_n = MD_MEMTEST; mt_mib_q <= go_n[10:0]; end
          else begin md_n = MD_DTEST; step <= step_q; step_load <= 1'b1; end
        end
      end

      // ------------------------------------------------------------------ BRINGUP
      ST_BRINGUP: begin
        // capacity (REC and READ END): start once the core knows the namespace
        if (!cap_valid_q && !cap_busy_q && core_age_ok && nsze_nz_q &&
            (md_q == MD_REC || md_q == MD_PF || (md_q == MD_READ && rd_kind_q == 2'd2))) begin
          cap_busy_q <= 1'b1; cap_val_q <= core_stat.nsze;
          cap_cnt_q <= (5'd17 - {1'b0, core_stat.lbads});
        end
        if (md_q == MD_READ && rd_kind_q == 2'd2 && !vc_loaded_q && cap_valid_q) begin
          verify_cmds <= (cap_val_q > 64'hFFFF_FFFF) ? 32'hFFFF_FFFF : cap_val_q[31:0];
          vc_loaded_q <= 1'b1;
        end
        if (cs_err_q) begin
          st_n = ST_ERROR;
        end else if (ev_abort) begin
          // handled below
        end else if (ev_pf) begin
          st_n = ST_CORE_STOP; ctrl_stop <= 1'b1; do_capture = 1'b1;
          pf_inb_q <= 1'b1; pf_src_q <= ev_pf_key;
          if (md_q == MD_REC) md_n = MD_PF;
          for (k = 1; k <= 3; k = k + 1) tab_q[k] <= t64_q;    // no source yet: phases 1..3 are empty
          tv_q[3:1] <= 3'b111;
        end else if (ev_stop) begin
          st_n = ST_CORE_STOP; ctrl_stop <= 1'b1;
          for (k = 1; k <= 3; k = k + 1) tab_q[k] <= t64_q;
          tv_q[3:1] <= 3'b111;
        end else if (cs_run_q &&
            !(md_q == MD_READ && rd_kind_q == 2'd2 && !vc_loaded_q)) begin   // review fix F: READ END waits for its capacity limit
          st_n = ST_RUN; ran_q <= 1'b1; base_q <= core_stat.beats_in[47:0]; t_run_q <= t64_q;
          sm_cnt_q <= 16'd0; rs_q <= 3'd0;
        end
      end

      // ------------------------------------------------------------------ RUN
      ST_RUN: begin
        // REC start sequence: capacity -> len_blocks -> start_load (X2b)
        if (md_q == MD_REC && rs_q != 3'd4 && rs_q != 3'd5) begin
          case (rs_q)
            3'd0: begin
              if (cap_valid_q) rs_q <= 3'd1;
              else if (!cap_busy_q && core_age_ok) begin
                cap_busy_q <= 1'b1; cap_val_q <= core_stat.nsze;
                cap_cnt_q <= (5'd17 - {1'b0, core_stat.lbads});
              end
            end
            3'd1: begin
              cap_lim_q <= (cap_val_q >= 64'd2) ? (cap_val_q - 64'd1) : cap_val_q; rs_q <= 3'd2;
            end
            3'd2: begin
              if (len_q == 32'd0) len_slots_q <= cap_lim_q;
              else if (cap_lim_q != 64'd0 && {32'b0, len_q} > cap_lim_q) len_slots_q <= cap_lim_q;
              else len_slots_q <= {32'b0, len_q};
              rs_q <= 3'd3;
            end
            default: begin
              start_cfg <= {len_slots_q[34:0], 5'b00000, prof_en_q}; start_load <= 1'b1;
              settle_q <= 7'(SETTLE); rs_q <= 3'd4;
            end
          endcase
        end
        // own timer; the stop decision below also gates log_req
        if (cs_err_q) begin
          st_n = ST_ERROR;
        end else if (md_q == MD_FORMAT) begin
          st_n = ST_CORE_STOP; ctrl_stop <= 1'b1; tab_q[3] <= t64_q; tv_q[3] <= 1'b1;
        end else if (cs_fl_q) begin
          // the core stopped on its own (io_timeout, lba_range_err ...)
          st_n = ST_CORE_STOP; ctrl_stop <= 1'b1; errored_n = 1'b1;
          tab_q[3] <= t64_q; tv_q[3] <= 1'b1;
          if (md_q == MD_REC && !src_stopped_q) begin
            src_stopped_q <= 1'b1; sp_kind_q <= 2'd1;
          end
        end else if (md_q == MD_REC) begin
          if (ev_pf) begin
            st_n = ST_SRC_STOP; md_n = MD_PF; do_capture = 1'b1; src_stopped_q <= 1'b1; sp_kind_q <= 2'd2;
            pf_src_q <= ev_pf_key;
            if (rs_q != 3'd4) rs_q <= 3'd5;
          end else if (ev_stop || (ui_age_ok && rs_q == 3'd4 && (ui_stat.len_reached_s || ui_stat.prof_end_s))) begin
            st_n = ST_SRC_STOP; src_stopped_q <= 1'b1; sp_kind_q <= 2'd1;
            if (rs_q != 3'd4) rs_q <= 3'd5;
          end
        end else begin
          // READ / MON
          if (ev_pf || ev_stop ||
              (md_q == MD_READ && ((core_age_ok && core_stat.cmd_limit_done) || (ui_age_ok && ui_stat.end_found)))) begin
            st_n = ST_CORE_STOP; ctrl_stop <= 1'b1; tab_q[3] <= t64_q; tv_q[3] <= 1'b1;
          end
        end
      end

      // ------------------------------------------------------------------ SRC_STOP
      ST_SRC_STOP: begin
        if (ui_age_ok && !tv_q[1] && (!ui_stat.src_active || rs_q == 3'd5)) begin tab_q[1] <= t64_q; tv_q[1] <= 1'b1; end
        if (ui_age_ok && !tv_q[2] && src_done_eff)    begin tab_q[2] <= t64_q; tv_q[2] <= 1'b1; end
        if (cs_err_q) begin
          st_n = ST_ERROR;
        end else if (cs_fl_q) begin
          st_n = ST_CORE_STOP; ctrl_stop <= 1'b1; errored_n = 1'b1;
          tab_q[3] <= t64_q; tv_q[3] <= 1'b1;
          if (!src_stopped_q) begin src_stopped_q <= 1'b1; sp_kind_q <= 2'd1; end
        end else if (drain_hold_q >= 3'd2) begin
          st_n = ST_CORE_STOP; ctrl_stop <= 1'b1; tab_q[3] <= t64_q; tv_q[3] <= 1'b1;
          if (!tv_q[1]) begin tab_q[1] <= t64_q; tv_q[1] <= 1'b1; end
          if (!tv_q[2]) begin tab_q[2] <= t64_q; tv_q[2] <= 1'b1; end
        end else if (prog_to_q || uwd_to_q) begin
          // review fix D: the drain makes no progress (stuck ddr_idle / drain_eq) or ui_clk is dead: stop the core, report !E
          st_n = ST_CORE_STOP; ctrl_stop <= 1'b1; errored_n = 1'b1; tab_q[3] <= t64_q; tv_q[3] <= 1'b1;
          if (!tv_q[1]) begin tab_q[1] <= t64_q; tv_q[1] <= 1'b1; end
          if (!tv_q[2]) begin tab_q[2] <= t64_q; tv_q[2] <= 1'b1; end
        end else if (ev_pf) begin
          md_n = MD_PF; do_capture = 1'b1; pf_late_q <= 1'b1; pf_src_q <= ev_pf_key;
          for (k = 1; k <= 6; k = k + 1) if (tv_q[k]) tab_q[k] <= t64_q;
        end
      end

      // ------------------------------------------------------------------ CORE_STOP
      ST_CORE_STOP: begin
        if (md_q == MD_REC || md_q == MD_PF) begin
          if (ui_age_ok && !tv_q[1] && (!ui_stat.src_active || rs_q == 3'd5)) begin tab_q[1] <= t64_q; tv_q[1] <= 1'b1; end
          if (ui_age_ok && !tv_q[2] && src_done_eff)    begin tab_q[2] <= t64_q; tv_q[2] <= 1'b1; end
        end
        if (cs_fl_q && !tv_q[4]) begin
          tab_q[4] <= t64_q; tv_q[4] <= 1'b1;
        end
        if (cs_sd_q && !tv_q[5]) begin
          tab_q[5] <= t64_q; tv_q[5] <= 1'b1;
        end
        if (cs_err_q) begin
          st_n = ST_ERROR;
        end else if (cs_done_q) begin
          st_n = ST_RELEASE; ctrl_start <= 1'b0; t_done_q <= t64_q;
          tab_q[6] <= t64_q; tv_q[6] <= 1'b1;
          errored_n = errored_q || (core_stat.error != 8'h00) || core_stat.flags[0] || core_stat.flags[2] ||
                      core_stat.flags[3];
          fin_act_q <= 1'b1; fin_q <= 4'd0;
          rf_ms_q <= t_ms_q - ms0_q; rf_ok_q <= !errored_n;
          // result image: captured one clock later (rcap_core_q), from registered values
          rcap_core_q <= 1'b1; rcap_md_q <= md_q;
          if (md_q == MD_REC || md_q == MD_PF) begin
            have_last_q <= 1'b1; last_wr_q <= ui_stat.wr_blocks;
          end
          if (md_q == MD_READ) rsid_user_q <= 1'b0;
          if (res_kind_of(md_q) != MK_NONE) last_res_kind_q <= res_kind_of(md_q);
          if (errored_n) begin
            pend_e_q <= 1'b1; ev_e_q <= {ST_RELEASE, core_stat.flags, core_stat.error_detail, core_stat.error};
          end
        end else if (ev_pf && md_q == MD_REC) begin
          md_n = MD_PF; do_capture = 1'b1; pf_late_q <= 1'b1; pf_src_q <= ev_pf_key;
          for (k = 1; k <= 6; k = k + 1) if (tv_q[k]) tab_q[k] <= t64_q;
        end
      end

      // ------------------------------------------------------------------ RELEASE
      ST_RELEASE: begin
        if (!fin_act_q && cs_idle_q) begin
          ctrl_stop <= 1'b0; verify_mode <= 1'b0; format_req <= 1'b0;
          done_led_q <= !errored_q;
          // D24: a READ needs no CORE_RST: nvme_write_engine keeps the write-ring slot apart from the Read slot
          if (errored_q) begin st_n = ST_CORE_RST; crst_cnt_q <= 32'd0; end
          else begin st_n = ST_IDLE; md_n = MD_NONE; end
        end
      end

      // ------------------------------------------------------------------ LOCAL
      ST_LOCAL: begin
        if (lt_cnt_q != 32'hFFFF_FFFF) lt_cnt_q <= lt_cnt_q + 32'd1;
        if (md_q == MD_MEMTEST) begin
          if (lt_cnt_q == CLR_DLY) mt_start <= 1'b1;
          if (ev_stop || (lt_cnt_q > CLR_DLY + 256 && ui_age_ok && ui_stat.mt_done)) begin
            st_n = ST_IDLE; do_capture = 1'b0;
            rcap_loc_q <= 1'b1; rcap_md_q <= md_q; rcap_clr_q <= (lt_cnt_q > CLR_DLY);
            last_res_kind_q <= MK_EV_RM; pend_res_q <= 1'b1;
          end
        end else begin
          if (!lt_stop_q && lt_cnt_q == CLR_DLY) begin   // review fix E: STOP restarts lt_cnt_q, a slow drain must not restart the source
            dtest_start <= 1'b1;
            start_cfg <= 41'd0; start_load <= 1'b1;   // X2b -> src_start: framer/profile run at the constant step, len 0
            settle_q <= 7'(SETTLE);
          end
          if (!lt_stop_q && ev_stop) begin
            if (lt_cnt_q <= CLR_DLY) begin
              st_n = ST_IDLE;               // never started: nothing to report but the event
              rcap_loc_q <= 1'b1; rcap_md_q <= md_q; rcap_clr_q <= 1'b0;
              last_res_kind_q <= MK_EV_RD; pend_res_q <= 1'b1;
            end else begin
              lt_stop_q <= 1'b1; sp_kind_q <= 2'd1; lt_cnt_q <= 32'd0;
            end
          end
          if (lt_stop_q && lt_cnt_q >= DT_WAIT && ui_age_ok && ui_stat.src_done && ui_stat.ddr_idle) begin
            st_n = ST_IDLE;
            rcap_loc_q <= 1'b1; rcap_md_q <= md_q; rcap_clr_q <= 1'b0;
            last_res_kind_q <= MK_EV_RD; pend_res_q <= 1'b1;
          end else if (lt_stop_q && (prog_to_q || uwd_to_q)) begin
            st_n = ST_ERROR;                // review fix D: the DTEST drain never finishes
          end
        end
        if (st_n == ST_IDLE) md_n = MD_NONE;
      end

      // ------------------------------------------------------------------ ERROR (one cycle)
      ST_ERROR: begin
        st_n = ST_CORE_RST; crst_cnt_q <= 32'd0;
      end

      // ------------------------------------------------------------------ CORE_RST
      ST_CORE_RST: begin
        crst_cnt_q <= crst_cnt_q + 32'd1;
        if (crst_cnt_q == 32'd16) ui_clear <= 1'b1;
        if (crst_cnt_q >= MS_CYC + FRESH_CYC && cs_idle_q) begin
          st_n = ST_IDLE; md_n = MD_NONE;
        end else if (crst_cnt_q >= CRST_TMO) begin
          // the core did not come back: report and repeat the reset
          crst_cnt_q <= 32'd0; pend_e_q <= 1'b1;
          ev_e_q <= {ST_CORE_RST, core_stat.flags, core_stat.error_detail, core_stat.error};
        end
      end

      default: st_n = ST_IDLE;
    endcase

    // prog_q only runs in SRC_STOP and in a LOCAL test after STOP (the only places that look at it): it is 0 on entry
    prog_clr = prog_chg_q || ev_stop || !(st_q == ST_SRC_STOP || (st_q == ST_LOCAL && lt_stop_q));
    if (prog_clr) prog_q <= 32'd0; else if (prog_q != 32'hFFFF_FFFF) prog_q <= prog_q + 32'd1;

    // D3: link loss, a core that fell back to IDLE mid-session, or a dead core status -> ERROR
    if (((st_q >= ST_BRINGUP && st_q <= ST_RELEASE) && lw_err_q) || idle_err_q) begin
      st_n = ST_ERROR; fin_act_q <= 1'b0; pend_res_q <= 1'b0;
    end

    // entering ERROR: drop everything towards the core, stop the source
    if (st_n == ST_ERROR && st_q != ST_ERROR) begin          // single flip-flops only: st_n is late
      src_stop <= 1'b1; ctrl_start <= 1'b0; ctrl_stop <= 1'b0; verify_mode <= 1'b0; format_req <= 1'b0;
      sp_kind_q <= 2'd0;
    end
    if (st_q == ST_ERROR && st_prev_q != ST_ERROR) begin     // the !E event: one clock later (wide data)
      pend_e_q <= 1'b1; ev_e_q <= {st_prev_q, core_stat.flags, core_stat.error_detail, core_stat.error};
    end

    // ABORT overrides every state
    if (ev_abort) begin
      st_n = ST_CORE_RST; ctrl_start <= 1'b0; ctrl_stop <= 1'b0; src_stop <= 1'b1;
      verify_mode <= 1'b0; format_req <= 1'b0; sp_kind_q <= 2'd0; fin_act_q <= 1'b0; pend_res_q <= 1'b0;
      crst_cnt_q <= 32'd0; md_n = MD_NONE;
    end

    // t0 snapshot of the power-fail measurements
    do_cap_q <= do_capture;
    if (do_cap_q) begin                     // one clock after the decision (timing)
      t0_q <= t64_q; rp_pnd_q <= pnd_q; rp_pdd_q <= {1'b0, ui_stat.fill_bytes}; rp_prg_q <= prg_q;
      pad0_q <= ui_stat.pad_blocks + ui_stat.trunc_blocks; pf_lov_q <= core_stat.log_busy;
    end

    // ---- deferred result image of a LOCAL test (MEMTEST / DTEST)
    if (rcap_core_q) res_q <= res_core_img;
    if (rcap_loc_q) begin
      res_q <= res_loc_img;
      if (rcap_clr_q) ui_clear <= 1'b1;       // after the image: ui_clear resets the ui status counters
    end

    // ---- finalisation after DONE (phase times, duration), one step per clock
    if (fin_act_q && st_q == ST_RELEASE) begin
      fin_q <= fin_q + 4'd1;
      case (fin_q)
        4'd0: ;
        4'd1: if (!tv_q[5]) tab_q[5] <= tab_q[6];
        4'd2: if (!tv_q[4]) tab_q[4] <= tab_q[5];
        4'd3: if (!tv_q[3]) tab_q[3] <= tab_q[4];
        4'd4: if (!tv_q[2]) tab_q[2] <= tab_q[3];
        4'd5: if (!tv_q[1]) tab_q[1] <= tab_q[2];
        4'd6:  ;                                           // tab_q settled; tdiff_q / tgt_q follow one clock later
        4'd7:  res_q.rp_t1 <= tgt_q[1] ? tdiff_q[1] : 64'd0;
        4'd8:  res_q.rp_t2 <= tgt_q[2] ? tdiff_q[2] : 64'd0;
        4'd9:  res_q.rp_t3 <= tgt_q[3] ? tdiff_q[3] : 64'd0;
        4'd10: res_q.rp_t4 <= tgt_q[4] ? tdiff_q[4] : 64'd0;
        4'd11: res_q.rp_t5 <= tgt_q[5] ? tdiff_q[5] : 64'd0;
        4'd12: res_q.rp_t6 <= tgt_q[6] ? tdiff_q[6] : 64'd0;
        4'd13: begin
          res_q.dur <= ran_q ? dur_run_q : 64'd0;   // review fix J: no RUN -> t_run_q is stale
          if (md_q == MD_FORMAT) res_q.dur <= dur_fmt_q;
        end
        default: begin
          fin_act_q <= 1'b0;
          if (res_kind_of(md_q) != MK_NONE) pend_res_q <= 1'b1;
        end
      endcase
    end

    // ---- state change bookkeeping
    st_q <= st_n;
    md_q <= md_n;
    errored_q <= errored_n;
    // review fix I: port B is refused by the message FIFO when it is nearly full (cnt >= MQ_D-1).  !S goes through a small
    // queue (strict order, highest priority); the deferred !R / !E / !B latches are cleared only when the push is accepted.
    // staged entries count; port A of this cycle is unknown here (timing) and is assumed to push
    b_room = (mq_occ + 5'd1) < 5'(MQ_D - 1);
    s_new  = s_chg;                      // the new state is st_q / md_q now
    sq_push = 1'b0; sq_pop = 1'b0;
    if (sq_cnt_q != 4'd0) begin
      if (b_room) begin
        push_b(MK_EV_S, 8'd0, {56'b0, sq_st[sq_rp_q]}, {32'b0, sq_t[sq_rp_q]});
        sq_pop = 1'b1;
      end
      sq_push = s_new;
    end else if (s_new) begin
      if (b_room) push_b(MK_EV_S, 8'd0, {56'b0, md_q, st_q}, {32'b0, t_ms_q});
      else sq_push = 1'b1;
    end
    if (sq_push) begin sq_st[sq_wp_q] <= {md_q, st_q}; sq_t[sq_wp_q] <= t_ms_q; sq_wp_q <= sq_wp_q + 3'd1; end
    if (sq_pop) sq_rp_q <= sq_rp_q + 3'd1;
    sq_cnt_q <= sq_cnt_q + {3'b0, sq_push} - {3'b0, sq_pop};

    // ---- deferred events (port B, only when !S did not use it)
    calib_prev_q <= ui_stat.calib;
    if (ui_stat.calib && !calib_prev_q) pend_b_q <= 1'b1;
    if (!pb_v && b_room) begin
      if (pend_res_q) begin
        push_b(last_res_kind_q, 8'd0, (last_res_kind_q == MK_EV_RF) ? {32'b0, rf_ms_q} : 64'd0,
               (last_res_kind_q == MK_EV_RF) ? {63'b0, rf_ok_q} : 64'd0);
        pend_res_q <= 1'b0;
      end else if (pend_e_q) begin
        push_b(MK_EV_E, 8'd0, {36'b0, ev_e_q}, 64'd0);
        pend_e_q <= 1'b0;
      end else if (pend_b_q) begin
        push_b(MK_EV_B, 8'd0, 64'd0, 64'd0);
        pend_b_q <= 1'b0;
      end
    end

    // ---- message FIFO: stage the pushes (A then B), write the staged ones, pop into the output stage
    sa_v_q <= pa_v; sa_k_q <= pa_k; sa_c_q <= pa_c; sa_a0_q <= pa_a0; sa_a1_q <= pa_a1;
    sb_v_q <= pb_v; sb_k_q <= pb_k; sb_c_q <= pb_c; sb_a0_q <= pb_a0; sb_a1_q <= pb_a1;
    begin : fifo
      logic [3:0] wp;
      logic [4:0] cnt;
      logic       pop, mv;
      logic       nres;
      wp  = mq_wp_q;
      cnt = mq_cnt_q;
      if (sa_v_q) begin
        mq_k[wp] <= sa_k_q; mq_c[wp] <= sa_c_q; mq_a0[wp] <= sa_a0_q; mq_a1[wp] <= sa_a1_q;
        mq_r[wp] <= is_res_kind(sa_k_q);
        wp = wp + 4'd1; cnt = cnt + 5'd1;
      end
      if (sb_v_q && cnt < 5'(MQ_D - 1)) begin
        mq_k[wp] <= sb_k_q; mq_c[wp] <= sb_c_q; mq_a0[wp] <= sb_a0_q; mq_a1[wp] <= sb_a1_q;
        mq_r[wp] <= is_res_kind(sb_k_q);
        wp = wp + 4'd1; cnt = cnt + 5'd1;
      end
      mv  = pre_v_q && (!msg_valid || msg_ready);               // pre-stage -> output registers
      pop = (mq_cnt_q != 5'd0) && (!pre_v_q || mv);              // FIFO -> pre-stage
      if (pop) begin
        pre_v_q <= 1'b1; pre_k_q <= mq_k[mq_rp_q]; pre_c_q <= mq_c[mq_rp_q];
        pre_a0_q <= mq_a0[mq_rp_q]; pre_a1_q <= mq_a1[mq_rp_q]; pre_r_q <= mq_r[mq_rp_q];
        mq_rp_q <= mq_rp_q + 4'd1; cnt = cnt - 5'd1;
      end else if (mv) pre_v_q <= 1'b0;
      if (mv) begin
        msg_valid <= 1'b1; msg_kind <= pre_k_q; msg_code <= pre_c_q;
        msg_arg0 <= pre_a0_q; msg_arg1 <= pre_a1_q; msg_res_q <= pre_r_q;
        nres = pre_r_q;
      end else if (msg_valid && msg_ready) begin
        msg_valid <= 1'b0; nres = 1'b0;
      end else begin
        nres = msg_valid && msg_res_q;
      end
      mq_wp_q  <= wp;
      mq_cnt_q <= cnt;
      snap <= nres ? res_q : snap_live;
    end

    // ---- command ready (registered): one command at a time, room for the reply
    cmd_ready <= !accept && !exec_n && !exec_q && !dec_q && !dec1_q && !chk_q && !pq_busy_n && (mq_occ <= 5'd8);

    // ---- periodic: TEL tick and Get Log Page requests
    if (ms_tick_q) begin
      if (tel_ms_q != 16'd0) begin
        if (tel_cnt_q + 16'd1 >= tel_ms_q) begin tel_tick <= 1'b1; tel_cnt_q <= 16'd0; end
        else tel_cnt_q <= tel_cnt_q + 16'd1;
      end
      if (st_q == ST_RUN && sm_ms_q != 16'd0) begin
        if (sm_cnt_q + 16'd1 >= sm_ms_q) begin
          sm_cnt_q <= 16'd0;
          if (st_n == ST_RUN && core_age_ok && !core_stat.log_busy) log_req <= 1'b1;
        end else sm_cnt_q <= sm_cnt_q + 16'd1;
      end
    end
    if (st_q != ST_RUN && sm_cnt_q != 16'd0 && !ms_tick_q) sm_cnt_q <= 16'd0;

    // ---- configuration towards the bridge (registered every clock)
    cfg.bp       <= bp_q;
    cfg.sid      <= (st_q == ST_IDLE) ? (sid_set_q ? sid_q : sid_q + 32'd1) : sid_use_q;
    cfg.rsid     <= rsid_user_q ? rsid_q : last_sid_q;
    cfg.ddr_mib  <= ddr_mib_q;
    cfg.prof_n   <= pn_q;
    cfg.prof_rep <= prof_rep_q;
    cfg.prof_end <= prof_end_q;
    cfg.mt_mib   <= (st_q == ST_LOCAL && md_q == MD_MEMTEST) ? mt_mib_q : ddr_mib_q;
    cfg.lbads    <= lbads_q;

    // ---- LEDs (1 = on)
    if (key_page) page_q <= page_q + 2'd1;
    case (page_q)
      2'd0: led <= {(errored_q || core_stat.error != 8'h00) ? 1'b1 : t_ms_q[9],
                    (st_q == ST_RUN) ? t_ms_q[8] : (done_led_q && st_q == ST_IDLE),
                    ui_stat.calib, core_stat.link_up};
      2'd1: led <= st_q;
      2'd2: led <= core_stat.seq_state;
      default: led <= core_stat.flags[3:0];
    endcase

    // ================================================================== reset
    if (rst) begin
      st_q <= ST_IDLE; md_q <= MD_NONE; errored_q <= 1'b0;
      t64_q <= 64'd0; t_ms_q <= 32'd0; ms_div_q <= 18'd0; ms_tick_q <= 1'b0;
      tel_cnt_q <= 16'd0; sm_cnt_q <= 16'd0;
      bp_q <= 1'b0; step_q <= STEP_DEF; len_q <= 32'd0; sid_q <= 32'd0; sid_use_q <= 32'd0;
      last_sid_q <= 32'd0; rsid_q <= 32'd0; sid_set_q <= 1'b0; rsid_user_q <= 1'b0;
      ddr_mib_q <= 11'h400; mt_mib_q <= 11'h400; tel_ms_q <= 16'h03E8; sm_ms_q <= 16'h03E8;
      pn_q <= 7'd0; prof_rep_q <= 16'd1; prof_end_q <= 1'b1; prof_en_q <= 1'b0; lbads_q <= 4'd0;
      have_last_q <= 1'b0; last_wr_q <= 48'd0;
      exec_q <= 1'b0; dec_q <= 1'b0; xs_q <= 1'b0; pq_busy_q <= 1'b0; pq_i_q <= 7'd0; pq_ph_q <= 3'd0;
      src_stopped_q <= 1'b0; pf_late_q <= 1'b0; pf_inb_q <= 1'b0; pf_src_q <= 1'b0; pf_lov_q <= 1'b0;
      sp_kind_q <= 2'd0; settle_q <= 7'd0; rs_q <= 3'd0; base_q <= 48'd0; tv_q <= '0;
      rf_ms_q <= 32'd0; rf_ok_q <= 1'b0; ms0_q <= 32'd0;
      fin_q <= 4'd0; fin_act_q <= 1'b0; last_res_kind_q <= MK_NONE; rd_kind_q <= 2'd0; vc_loaded_q <= 1'b0;
      lt_cnt_q <= 32'd0; lt_stop_q <= 1'b0; crst_cnt_q <= 32'd0; done_led_q <= 1'b0; page_q <= 2'd0;
      calib_prev_q <= 1'b0; drain_hold_q <= 3'd0; ran_q <= 1'b0; wd_q <= 32'd0; vm_hold_q <= 9'd0; prog_q <= 32'd0; uwd_q <= 32'd0; prog_sig_q <= '0; prog_chg_q <= 1'b0;
      cap_busy_q <= 1'b0; cap_valid_q <= 1'b0; cap_cnt_q <= 5'd0;
      pend_b_q <= 1'b1; pend_e_q <= 1'b0; pend_res_q <= 1'b0;
      sq_wp_q <= 3'd0; sq_rp_q <= 3'd0; sq_cnt_q <= 4'd0;
      sa_v_q <= 1'b0; sb_v_q <= 1'b0; msg_res_q <= 1'b0;
      mq_wp_q <= 4'd0; mq_rp_q <= 4'd0; mq_cnt_q <= 5'd0; msg_valid <= 1'b0; cmd_ready <= 1'b0;
      ctrl_start <= 1'b0; ctrl_stop <= 1'b0; verify_mode <= 1'b0; format_req <= 1'b0; verify_cmds <= 32'd0;
      core_rst_req <= 1'b0; step <= STEP_DEF; start_cfg <= 41'd0;
      step_load <= 1'b0; start_load <= 1'b0; src_stop <= 1'b0; src_pf <= 1'b0; ui_clear <= 1'b0;
      mt_start <= 1'b0; dtest_start <= 1'b0; log_req <= 1'b0; tel_tick <= 1'b0; prof_we <= 1'b0;
      res_q <= '0;
      go_rec_r <= 1'b0; go_prof_r <= 1'b0; go_read_r <= 1'b0; go_mon_r <= 1'b0; go_format_r <= 1'b0;
      go_mem_r <= 1'b0; go_dtest_r <= 1'b0; go_rdk_r <= 2'd0; go_n_r <= 64'd0;
      ev_stop_r <= 1'b0; ev_pf_r <= 1'b0; ev_pf_key_r <= 1'b0; ev_abort_r <= 1'b0;
      st_prev_q <= ST_IDLE; rcap_loc_q <= 1'b0; rcap_core_q <= 1'b0; dec1_q <= 1'b0; chk_q <= 1'b0; pre_v_q <= 1'b0; rcap_md_q <= MD_NONE;
    end else begin
      core_rst_req <= (st_q == ST_CORE_RST) && !ev_abort && (crst_cnt_q < MS_CYC);
    end
  end

  assign mq_occ = mq_cnt_q + 5'(sa_v_q) + 5'(sb_v_q);

  assign t_ms = t_ms_q;

endmodule
