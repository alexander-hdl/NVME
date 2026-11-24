// =============================================================================
// ax7a200b_rec_top.sv - recorder prototype top (WP8) for the ALINX AX7A200B + M.2 adapter.
// Spec: STAND/RECORDER_SPEC.md 4.1-4.5, 8.3; connection table: STAND/RECORDER_AUDIT.md; notes: INTEGRATION_NOTES.md.
//
//   UART (L14/L15) -> rec_cmd_parser -> rec_ctrl -> rec_tx_fmt -> UART          (link_clk 200 MHz)
//   rec_ctrl -> rec_cdc_bridge -> ui_clk (MIG 100 MHz): rec_profile -> rec_framer -> ddr_fifo (DDR3) -> data_ring
//   nvme_host_top (PCIe Root Port + NVMe host core, user_clk 125 MHz) -> vrf_tap -> rec_verify (ui_clk)
//   DTEST: ddr_fifo output goes to rec_sink_chk instead of the core (latch dt_mode_q, audit D1).
//   The old ax7a200b_top stays untouched (regression bitstream).
//
// Domains: link_clk = nvme_host_top.link_clk_out, ui_clk = MIG ui_clk (or mig_app_model), user_clk = PCIe user clock.
// Resets (spec 4.2): rec_rst (POR | RESET key, link), core_rst_n (RESET | VIO probe_out0 | rec_ctrl.core_rst_req | POR,
//   registered, async input of nvme_host_top), mig_sys_rst_n (POR only), ui_rst (ui_clk_sync_rst | rec_rst, async assert /
//   sync release in ui_clk), user_rst = nvme_host_top.user_rst_out.
//
// USE_MIG_MODEL (spec 4.4): 0 = Xilinx IP mig_ddr3 (synthesis), 1 = sim/models/mig_app_model.sv at the app interface
//   (simulation; DDR3 pins are then undriven).  Choice documented in INTEGRATION_NOTES.md, "WP8".
//   Simulation also substitutes rtl/pcie/pcie7x_wrapper.sv by sim/models/pcie7x_wrapper_sim.sv (same module name, same
//   ports) and the VIO IP by sim/models/vio_status_sim.sv; everything else is the real RTL (incl. nvme_host_top).
//
// WP8 exceptions to CODING_RULES: power-up values by declaration initialiser (POR counter, core_rst_n register);
//   the top has `inout` (DDR3 data, PERST#), as allowed for tops.
// =============================================================================
module ax7a200b_rec_top
  import rec_pkg::*;
#(
  parameter bit          SIM_FAST      = 1'b0,
  parameter int unsigned N_SLOTS       = 4,
  parameter bit          PERST_DRIVE   = 1'b0,
  parameter bit          ILA_EN        = 1'b0,
  parameter bit          USE_MIG_MODEL = 1'b0,
  parameter int unsigned DEBOUNCE_BITS = 21,                  // 2^21 link clocks = 10.5 ms
  parameter logic [31:0] BLD_DATE      = 32'h2026_1124,       // PING bld, yyyymmdd as hex digits; set per build
  parameter int unsigned UART_BAUD     = 921_600,
  parameter int unsigned REQ_TIMEOUT_CYCLES = 1_250_000,
  // simulation only (USE_MIG_MODEL = 1)
  parameter int          MIG_UI_PERIOD_PS = 10000,
  parameter int          MIG_JITTER_PS    = 0,
  parameter int          MIG_CALIB_DELAY  = 200,
  parameter string       XADC_SIM_FILE    = "design.txt"
) (
  // ---- clocks ----
  input  logic         sys_clk_p,            // 100 MHz PCIe refclk (F10/E10)
  input  logic         sys_clk_n,
  input  logic         free_clk_p,           // 200 MHz SYS_CLK (R4/T4)
  input  logic         free_clk_n,
  // ---- PCIe ----
  inout  wire          pcie_perst_n,         // L16
  output logic [3:0]   pci_exp_txp,
  output logic [3:0]   pci_exp_txn,
  input  logic [3:0]   pci_exp_rxp,
  input  logic [3:0]   pci_exp_rxn,
  // ---- user I/O ----
  input  logic         rst_key_n,            // RESET key (F15), active low
  input  logic [3:0]   key_n,                // KEY1..KEY4, active low
  output logic [3:0]   led_n,                // LED1..LED4, active low
  output logic         uart_tx,              // L15
  input  logic         uart_rx,              // L14
  output logic         ssd_pwr_en,           // J11-3 = G21, constant 1 (future SSD power switch)
  // ---- DDR3 (module bank 34/35) ----
  inout  wire  [31:0]  ddr3_dq,
  inout  wire  [3:0]   ddr3_dqs_p,
  inout  wire  [3:0]   ddr3_dqs_n,
  output logic [14:0]  ddr3_addr,
  output logic [2:0]   ddr3_ba,
  output logic         ddr3_ras_n,
  output logic         ddr3_cas_n,
  output logic         ddr3_we_n,
  output logic         ddr3_reset_n,
  output logic [0:0]   ddr3_ck_p,
  output logic [0:0]   ddr3_ck_n,
  output logic [0:0]   ddr3_cke,
  output logic [0:0]   ddr3_cs_n,
  output logic [3:0]   ddr3_dm,
  output logic [0:0]   ddr3_odt
);

  localparam int unsigned SLOT_BYTES = nvme_host_pkg::SLOT_BYTES_DEF;   // 128 KiB per command (rec_verify CTX_SHIFT = 5)

  assign ssd_pwr_en = 1'b1;

  // ---------------------------------------------------------------------------
  // clocks / reset nets
  // ---------------------------------------------------------------------------
  logic link_clk, user_clk, ui_clk;
  logic rec_rst, ui_rst, user_rst;
  logic ui_ok_q;
  logic core_rst_req;
  logic vio_rst, vio_start_lvl, vio_stop_lvl, vio_verify;

  // ---- POR (link_clk): counter runs from configuration; power-up value 0 by declaration initialiser ----
  localparam int unsigned POR_BITS = SIM_FAST ? 6 : 17;       // 2^17 clocks = 0.65 ms
  logic [POR_BITS:0] por_cnt_q = '0;
  logic              por_done;
  assign por_done = por_cnt_q[POR_BITS];
  always_ff @(posedge link_clk)
    if (!por_done) por_cnt_q <= por_cnt_q + 1'b1;

  logic rst_key_s;                                            // RESET key synchronised (level, 1 = pressed)
  cdc_sync #(.W(1)) u_cdc_sync_rstkey (.clk(link_clk), .d(~rst_key_n), .q(rst_key_s));   // quasi-static key level

  logic rec_rst_q = 1'b1;
  always_ff @(posedge link_clk) rec_rst_q <= ~por_done | rst_key_s;
  assign rec_rst = rec_rst_q;

  // core reset: registered (glitch free), asynchronous input of nvme_host_top.board_rst_n; released after POR
  logic core_rst_n_q = 1'b0;
  always_ff @(posedge link_clk) core_rst_n_q <= por_done & ~rst_key_s & ~vio_rst & ~core_rst_req;

  // MIG reset (active low): POR only, the RESET key does not touch the calibration
  logic mig_sys_rst_n_q = 1'b0;
  always_ff @(posedge link_clk) mig_sys_rst_n_q <= por_done;

  // ui_rst: MIG ui_clk_sync_rst OR rec_rst; asynchronous assert, synchronous release in ui_clk (the only async reset use)
  logic mig_ui_rst;
  logic ui_rst_a;
  assign ui_rst_a = mig_ui_rst | rec_rst;
  (* ASYNC_REG = "TRUE" *) logic [1:0] ui_rst_sync_q;
  always_ff @(posedge ui_clk or posedge ui_rst_a) begin
    if (ui_rst_a) ui_rst_sync_q <= 2'b11;
    else          ui_rst_sync_q <= {ui_rst_sync_q[0], 1'b0};
  end
  assign ui_rst = ui_rst_sync_q[1];
  always_ff @(posedge ui_clk) ui_ok_q <= ~ui_rst;

  // ---------------------------------------------------------------------------
  // nvme_host_top (PCIe Root Port + NVMe host core)
  // ---------------------------------------------------------------------------
  logic         ctrl_start, ctrl_stop, verify_mode, format_req, log_req_u;
  logic [31:0]  verify_cmds;
  logic [127:0] s_data_tdata;
  logic         s_data_tvalid, s_data_tready;
  logic         host_perst_n;

  logic         status_link_up, status_gen2, status_enum_done, status_nvme_ready, status_running, status_done;
  logic [7:0]   status_error, status_error_detail, status_flags;
  logic [1:0]   status_link_width, link_retries;
  logic         status_lba_range_err;
  logic [63:0]  stat_bytes_done, stat_cmds_done, stat_beats_in, stat_nsze;
  logic [31:0]  stat_cmd_errors, stat_max_latency;
  logic [15:0]  stat_rx_msgs;
  logic [3:0]   stat_seq_state, stat_lbads;
  logic         format_done;
  logic [31:0]  format_ms;
  logic         cmd_limit_done;
  logic         log_busy;
  logic [nvme_host_pkg::SMART_W-1:0] log_smart;
  logic [15:0]  log_cnt, log_err_cnt, log_status;
  logic [31:0]  rtd3e_us;
  logic         link_lost, link_fail;
  logic [5:0]   dbg_ltssm;
  logic [303:0] dbg_uc;
  logic         tap_wr_en, tap_verify_mode;
  logic [31:0]  tap_wr_addr;
  logic [127:0] tap_wr_data;
  logic [15:0]  tap_wr_be;
  logic [N_SLOTS*64-1:0] tap_slot_lba_flat;

  assign pcie_perst_n = PERST_DRIVE ? host_perst_n : 1'bz;

  nvme_host_top #(
    .N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES), .SIM_FAST(SIM_FAST),
    .REQ_TIMEOUT_CYCLES(REQ_TIMEOUT_CYCLES), .VERIFY_INTERNAL(1'b0)
  ) u_host (
    .sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n), .free_clk_p(free_clk_p), .free_clk_n(free_clk_n),
    .board_rst_n(core_rst_n_q), .pcie_perst_n(host_perst_n),
    .pci_exp_txp(pci_exp_txp), .pci_exp_txn(pci_exp_txn), .pci_exp_rxp(pci_exp_rxp), .pci_exp_rxn(pci_exp_rxn),
    .data_clk(ui_clk), .s_data_tdata(s_data_tdata), .s_data_tvalid(s_data_tvalid), .s_data_tready(s_data_tready),
    .ctrl_start(ctrl_start), .ctrl_stop(ctrl_stop), .verify_mode(verify_mode), .verify_cmds(verify_cmds),
    .format_req(format_req), .log_req(log_req_u),
    .status_link_up(status_link_up), .status_gen2(status_gen2), .status_enum_done(status_enum_done),
    .status_nvme_ready(status_nvme_ready), .status_running(status_running), .status_done(status_done),
    .status_error(status_error), .status_error_detail(status_error_detail),
    .status_link_width(status_link_width), .status_lba_range_err(status_lba_range_err),
    .stat_bytes_done(stat_bytes_done), .stat_cmds_done(stat_cmds_done), .stat_cmd_errors(stat_cmd_errors),
    .stat_max_latency(stat_max_latency), .stat_cpl_evt(), .stat_cpl_lat(), .stat_rx_msgs(stat_rx_msgs),
    .status_flags(status_flags), .stat_first_error_lba(),
    .format_done(format_done), .format_ms(format_ms),
    .log_busy(log_busy), .log_done(), .log_err(), .log_smart(log_smart),
    .log_cnt(log_cnt), .log_err_cnt(log_err_cnt), .log_status(log_status), .rtd3e_us(rtd3e_us),
    .tap_wr_en(tap_wr_en), .tap_wr_addr(tap_wr_addr), .tap_wr_data(tap_wr_data), .tap_wr_be(tap_wr_be),
    .tap_slot_lba_flat(tap_slot_lba_flat), .tap_verify_mode(tap_verify_mode),
    .stat_beats_in(stat_beats_in), .stat_seq_state(stat_seq_state),
    .stat_lbads(stat_lbads), .stat_nsze(stat_nsze), .cmd_limit_done(cmd_limit_done),
    .vfy_bytes(), .vfy_err_beats(), .vfy_first_err_lba(), .vfy_first_err_exp(), .vfy_first_err_got(),
    .link_retries(link_retries), .link_lost(link_lost), .link_fail(link_fail),
    .link_clk_out(link_clk), .user_clk_out(user_clk), .user_rst_out(user_rst),
    .dbg_ltssm_state(dbg_ltssm), .dbg_uc(dbg_uc));

  generate if (ILA_EN) begin : g_ila
    ila_pcie u_ila (.clk(user_clk), .probe0(dbg_uc));
  end endgenerate

  // ---------------------------------------------------------------------------
  // link_clk: text path
  // ---------------------------------------------------------------------------
  logic [7:0]  rx_data;
  logic        rx_valid, rx_ferr;
  uart_rx #(.CLK_HZ(LINK_CLK_HZ), .BAUD(UART_BAUD)) u_uart_rx (
    .clk(link_clk), .rst(rec_rst), .rxd(uart_rx), .m_data(rx_data), .m_valid(rx_valid), .frame_err(rx_ferr));

  logic        cmd_valid, cmd_ready;
  logic [5:0]  cmd_op;
  logic [3:0]  cmd_sub;
  logic [1:0]  cmd_nargs;
  logic [63:0] cmd_arg0, cmd_arg1, cmd_arg2;
  logic [7:0]  cmd_err;
  rec_cmd_parser u_rec_cmd_parser (
    .clk(link_clk), .rst(rec_rst), .rx_data(rx_data), .rx_valid(rx_valid), .rx_ferr(rx_ferr),
    .cmd_valid(cmd_valid), .cmd_ready(cmd_ready), .cmd_op(cmd_op), .cmd_sub(cmd_sub), .cmd_nargs(cmd_nargs),
    .cmd_arg0(cmd_arg0), .cmd_arg1(cmd_arg1), .cmd_arg2(cmd_arg2), .cmd_err(cmd_err));

  logic        msg_valid, msg_ready, tel_tick;
  logic [4:0]  msg_kind;
  logic [7:0]  msg_code;
  logic [63:0] msg_arg0, msg_arg1;
  rec_snap_t   snap;
  rec_tx_fmt #(.CLK_HZ(LINK_CLK_HZ), .BAUD(UART_BAUD), .BLD_DATE(BLD_DATE)) u_rec_tx_fmt (
    .clk(link_clk), .rst(rec_rst), .msg_valid(msg_valid), .msg_ready(msg_ready), .msg_kind(msg_kind),
    .msg_code(msg_code), .msg_arg0(msg_arg0), .msg_arg1(msg_arg1), .snap(snap), .tel_tick(tel_tick),
    .txd(uart_tx));

  // ---- keys (debounce + 1-clock pulses) and VIO edges ----
  logic [3:0] key_db_q, key_db_d_q;
  generate for (genvar k = 0; k < 4; k++) begin : g_key
    logic                      lvl_s;
    logic [DEBOUNCE_BITS-1:0]  cnt_q;
    logic                      db_q;
    cdc_sync #(.W(1)) u_cdc_sync_key (.clk(link_clk), .d(~key_n[k]), .q(lvl_s));        // quasi-static key level
    always_ff @(posedge link_clk) begin
      if (rec_rst) begin
        cnt_q <= '0;
        db_q  <= 1'b0;
      end else if (lvl_s == db_q) begin
        cnt_q <= '0;
      end else begin
        cnt_q <= cnt_q + 1'b1;
        if (&cnt_q) db_q <= lvl_s;
      end
    end
    assign key_db_q[k] = db_q;
  end endgenerate
  logic vio_start_d_q, vio_stop_d_q;
  always_ff @(posedge link_clk) begin
    key_db_d_q    <= key_db_q;
    vio_start_d_q <= vio_start_lvl;
    vio_stop_d_q  <= vio_stop_lvl;
  end
  logic key_rec, key_stop, key_pf, key_page, vio_start_p, vio_stop_p;
  assign key_rec     = key_db_q[0] & ~key_db_d_q[0];
  assign key_stop    = key_db_q[1] & ~key_db_d_q[1];
  assign key_pf      = key_db_q[2] & ~key_db_d_q[2];
  assign key_page    = key_db_q[3] & ~key_db_d_q[3];
  assign vio_start_p = vio_start_lvl & ~vio_start_d_q;
  assign vio_stop_p  = vio_stop_lvl  & ~vio_stop_d_q;

  // ---- XADC ----
  xadc_t xadc;
  xadc_mon #(.SIM_MONITOR_FILE(XADC_SIM_FILE)) u_xadc_mon (.clk(link_clk), .rst(rec_rst), .xadc(xadc), .upd());

  // ---- rec_ctrl ----
  rec_cfg_t    cfg;
  logic [31:0] step, t_ms;
  logic        step_load, start_load, src_stop, src_pf, ui_clear, mt_start, dtest_start, log_req;
  logic [40:0] start_cfg;
  ui_stat_t    ui_stat;
  logic        ui_age_ok, core_age_ok;
  core_stat_t  core_stat;
  logic        prof_we;
  logic [5:0]  prof_addr;
  logic [71:0] prof_wdata, prof_rdata;
  logic [3:0]  led;

  rec_ctrl #(.SIM_FAST(SIM_FAST)) u_rec_ctrl (
    .clk(link_clk), .rst(rec_rst),
    .cmd_valid(cmd_valid), .cmd_ready(cmd_ready), .cmd_op(cmd_op), .cmd_sub(cmd_sub), .cmd_nargs(cmd_nargs),
    .cmd_arg0(cmd_arg0), .cmd_arg1(cmd_arg1), .cmd_arg2(cmd_arg2), .cmd_err(cmd_err),
    .key_rec(key_rec), .key_stop(key_stop), .key_pf(key_pf), .vio_start(vio_start_p), .vio_stop(vio_stop_p),
    .vio_verify(vio_verify), .key_page(key_page),
    .msg_valid(msg_valid), .msg_ready(msg_ready), .msg_kind(msg_kind), .msg_code(msg_code),
    .msg_arg0(msg_arg0), .msg_arg1(msg_arg1), .snap(snap), .tel_tick(tel_tick),
    .ctrl_start(ctrl_start), .ctrl_stop(ctrl_stop), .verify_mode(verify_mode), .format_req(format_req),
    .verify_cmds(verify_cmds), .core_rst_req(core_rst_req), .log_req(log_req),
    .cfg(cfg), .step(step), .step_load(step_load), .start_cfg(start_cfg), .start_load(start_load),
    .src_stop(src_stop), .src_pf(src_pf), .ui_clear(ui_clear), .mt_start(mt_start), .dtest_start(dtest_start),
    .ui_stat(ui_stat), .ui_age_ok(ui_age_ok), .core_stat(core_stat), .core_age_ok(core_age_ok),
    .xadc(xadc), .link_lost(link_lost), .link_fail(link_fail),
    .prof_we(prof_we), .prof_addr(prof_addr), .prof_wdata(prof_wdata), .prof_rdata(prof_rdata),
    .led(led), .t_ms(t_ms));

  assign led_n = ~led;

  // ---------------------------------------------------------------------------
  // profile table X3: A = link (rec_ctrl), B = ui (rec_profile)
  // ---------------------------------------------------------------------------
  logic        prof_rd_en;
  logic [5:0]  prof_rd_addr;
  logic [71:0] prof_rd_data;
  tdp_ram_2clk #(.W(72), .DEPTH(PROF_N)) u_tdp_ram_2clk_prof (
    .a_clk(link_clk), .a_we(prof_we), .a_addr(prof_addr), .a_wdata(prof_wdata), .a_rdata(prof_rdata),
    .b_clk(ui_clk), .b_en(prof_rd_en), .b_addr(prof_rd_addr), .b_rdata(prof_rd_data));

  // ---------------------------------------------------------------------------
  // CDC bridge (X1, X2, X2a, X2b, X4, X8, X10, X14)
  // ---------------------------------------------------------------------------
  rec_cfg_t    cfg_ui;
  logic [31:0] step_ui;
  logic        rate_upd;
  logic [40:0] start_cfg_ui;
  logic        src_start, src_stop_ui, src_pf_ui, ui_clear_ui, mt_start_ui, dtest_start_ui;
  logic [11:0] device_temp;
  ui_stat_t    ui_stat_in;
  core_stat_t  core_stat_in;

  rec_cdc_bridge #(.F_LINK_MHZ(200), .F_UI_MHZ(100), .F_USER_MHZ(125)) u_rec_cdc_bridge (
    .link_clk(link_clk), .link_rst(rec_rst),
    .cfg(cfg), .step(step), .step_load(step_load), .start_cfg(start_cfg), .start_load(start_load),
    .src_stop(src_stop), .src_pf(src_pf), .ui_clear(ui_clear), .mt_start(mt_start), .dtest_start(dtest_start),
    .log_req(log_req), .xadc_temp(xadc.temp),
    .ui_stat(ui_stat), .ui_age_ok(ui_age_ok), .core_stat(core_stat), .core_age_ok(core_age_ok),
    .ui_clk(ui_clk), .ui_rst(ui_rst),
    .cfg_ui(cfg_ui), .step_ui(step_ui), .rate_upd(rate_upd), .start_cfg_ui(start_cfg_ui), .src_start(src_start),
    .src_stop_ui(src_stop_ui), .src_pf_ui(src_pf_ui), .ui_clear_ui(ui_clear_ui), .mt_start_ui(mt_start_ui),
    .dtest_start_ui(dtest_start_ui), .device_temp(device_temp), .ui_stat_in(ui_stat_in),
    .user_clk(user_clk), .user_rst(user_rst), .log_req_u(log_req_u), .core_stat_in(core_stat_in));

  // core_stat_in: user_clk registers of nvme_host_top (audit 1.5)
  always_comb begin
    core_stat_in               = '0;
    core_stat_in.seq_state     = stat_seq_state;
    core_stat_in.error         = status_error;
    core_stat_in.error_detail  = status_error_detail;
    core_stat_in.flags         = status_flags;
    core_stat_in.link_up       = status_link_up;
    core_stat_in.gen2          = status_gen2;
    core_stat_in.link_width    = status_link_width;
    core_stat_in.lba_range_err = status_lba_range_err;
    core_stat_in.bytes_done    = stat_bytes_done;
    core_stat_in.cmds_done     = stat_cmds_done;
    core_stat_in.cmd_errors    = stat_cmd_errors;
    core_stat_in.max_latency   = stat_max_latency;
    core_stat_in.beats_in      = stat_beats_in;
    core_stat_in.lbads         = stat_lbads;
    core_stat_in.nsze          = stat_nsze;
    core_stat_in.smart         = smart_t'(log_smart);
    core_stat_in.log_cnt       = log_cnt;
    core_stat_in.log_err_cnt   = log_err_cnt;
    core_stat_in.log_busy      = log_busy;
    core_stat_in.log_status    = log_status;
    core_stat_in.rtd3e_us      = rtd3e_us;
    core_stat_in.cmd_limit_done = cmd_limit_done;
  end

  // ---------------------------------------------------------------------------
  // ui_clk: MIG / model
  // ---------------------------------------------------------------------------
  logic         calib;
  logic [28:0]  app_addr_f;                     // from ddr_fifo (29 bit, word address of the 1 GiB window)
  logic [27:0]  app_addr;                       // MIG prj: row15 + bank3 + col10 = 28 bits
  logic [2:0]   app_cmd;
  logic         app_en, app_rdy;
  logic [255:0] app_wdf_data, app_rd_data;
  logic [31:0]  app_wdf_mask;
  logic         app_wdf_wren, app_wdf_end, app_wdf_rdy, app_rd_data_valid, app_rd_data_end;
  logic         app_sr_req, app_ref_req, app_zq_req;
  assign app_addr = app_addr_f[27:0];           // window <= 1 GiB = 2^28 words, bit 28 is always 0

  generate if (USE_MIG_MODEL) begin : g_mig_model
    mig_app_model #(.UI_PERIOD_PS(MIG_UI_PERIOD_PS), .JITTER_PS(MIG_JITTER_PS), .CALIB_DELAY(MIG_CALIB_DELAY)) u_mig (
      .ui_clk(ui_clk), .ui_clk_sync_rst(mig_ui_rst), .init_calib_complete(calib), .device_temp_i(device_temp),
      .app_addr({1'b0, app_addr}), .app_cmd(app_cmd), .app_en(app_en), .app_rdy(app_rdy),
      .app_wdf_data(app_wdf_data), .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren),
      .app_wdf_end(app_wdf_end), .app_wdf_rdy(app_wdf_rdy),
      .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid), .app_rd_data_end(app_rd_data_end),
      .app_sr_req(app_sr_req), .app_ref_req(app_ref_req), .app_zq_req(app_zq_req),
      .app_sr_active(), .app_ref_ack(), .app_zq_ack());
    assign ddr3_addr    = '0;
    assign ddr3_ba      = '0;
    assign ddr3_ras_n   = 1'b1;
    assign ddr3_cas_n   = 1'b1;
    assign ddr3_we_n    = 1'b1;
    assign ddr3_reset_n = 1'b0;
    assign ddr3_ck_p    = '0;
    assign ddr3_ck_n    = '1;
    assign ddr3_cke     = '0;
    assign ddr3_cs_n    = '1;
    assign ddr3_dm      = '0;
    assign ddr3_odt     = '0;
  end else begin : g_mig_ip
    // Xilinx MIG 7-series (IP generated by fpga/scripts/mig_ip.tcl); sys_clk_i = link_clk (BUFG, "No Buffer"),
    // sys_rst active low.  calib_tap_* exist only with `define SKIP_CALIB (not used).
    mig_ddr3 u_mig (
      .ddr3_dq(ddr3_dq), .ddr3_dqs_n(ddr3_dqs_n), .ddr3_dqs_p(ddr3_dqs_p),
      .ddr3_addr(ddr3_addr), .ddr3_ba(ddr3_ba), .ddr3_ras_n(ddr3_ras_n), .ddr3_cas_n(ddr3_cas_n),
      .ddr3_we_n(ddr3_we_n), .ddr3_reset_n(ddr3_reset_n), .ddr3_ck_p(ddr3_ck_p), .ddr3_ck_n(ddr3_ck_n),
      .ddr3_cke(ddr3_cke), .ddr3_cs_n(ddr3_cs_n), .ddr3_dm(ddr3_dm), .ddr3_odt(ddr3_odt),
      .sys_clk_i(link_clk),
      .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en),
      .app_wdf_data(app_wdf_data), .app_wdf_end(app_wdf_end), .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren),
      .app_rd_data(app_rd_data), .app_rd_data_end(app_rd_data_end), .app_rd_data_valid(app_rd_data_valid),
      .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy),
      .app_sr_req(app_sr_req), .app_ref_req(app_ref_req), .app_zq_req(app_zq_req),
      .app_sr_active(), .app_ref_ack(), .app_zq_ack(),
      .ui_clk(ui_clk), .ui_clk_sync_rst(mig_ui_rst), .init_calib_complete(calib),
      .device_temp_i(device_temp), .device_temp(),
      .sys_rst(mig_sys_rst_n_q));
  end endgenerate

  // ---------------------------------------------------------------------------
  // ui_clk: source -> DDR FIFO -> core / DTEST sink
  // ---------------------------------------------------------------------------
  logic [31:0]  prof_step;
  logic [5:0]   seg_idx;
  logic [15:0]  rep_cnt;
  logic         prof_active, prof_end_s;
  rec_profile u_rec_profile (
    .clk(ui_clk), .rst(ui_rst), .clear(ui_clear_ui), .start(src_start), .stop(src_stop_ui | src_pf_ui),
    .prof_en(start_cfg_ui[0]), .nseg(cfg_ui.prof_n), .rep(cfg_ui.prof_rep), .end_stop(cfg_ui.prof_end),
    .const_step(step_ui), .rate_upd(rate_upd),
    .rd_en(prof_rd_en), .rd_addr(prof_rd_addr), .rd_data(prof_rd_data),
    .step(prof_step), .seg_idx(seg_idx), .rep_cnt(rep_cnt), .prof_active(prof_active),
    .prof_end(), .prof_end_s(prof_end_s));

  logic [127:0] fr_tdata;
  logic         fr_tvalid, fr_tready;
  logic         space_ok;
  logic [30:0]  fill_bytes, fill_max_bytes;
  logic [47:0]  src_blocks, wr_blocks, ddr_wr_bytes, ddr_rd_bytes;
  logic [31:0]  drop_blocks;
  logic [15:0]  pad_blocks, trunc_blocks, rt_stall_max;
  logic [39:0]  bp_max_beats;
  logic         len_reached_s, fr_active, fr_done;
  rec_framer u_rec_framer (
    .clk(ui_clk), .rst(ui_rst), .clear(ui_clear_ui), .start(src_start), .stop(src_stop_ui), .pf(src_pf_ui),
    .bp_mode(cfg_ui.bp), .prof_en(start_cfg_ui[0]), .sid(cfg_ui.sid), .len_blocks(start_cfg_ui[40:1]),
    .step(prof_step), .seg_idx(seg_idx), .rep_cnt(rep_cnt), .space_ok(space_ok),
    .ddr_fill_kib({11'd0, fill_bytes[30:10]}),
    .m_tdata(fr_tdata), .m_tvalid(fr_tvalid), .m_tready(fr_tready),
    .src_blocks(src_blocks), .wr_blocks(wr_blocks), .drop_blocks(drop_blocks), .pad_blocks(pad_blocks),
    .trunc_blocks(trunc_blocks), .bp_max_beats(bp_max_beats), .rt_stall_max(rt_stall_max),
    .len_reached(), .len_reached_s(len_reached_s), .active(fr_active), .done(fr_done));

  logic [127:0] fifo_m_tdata;
  logic         fifo_m_tvalid, fifo_m_tready;
  logic         ddr_idle, mt_busy, mt_done;
  logic [31:0]  mt_err_cnt;
  logic [28:0]  mt_first_err_addr;
  // ddr_fifo is reset by the MIG only; the RESET key (rec_rst) comes in as a drain (`clear`): reads in flight in the MIG are
  // awaited, then pointers/counters are zeroed (review fix A).  Level clear while the key is down.
  logic rec_rst_ui, fifo_clear;
  cdc_sync #(.W(1)) u_cdc_sync_recrst (.clk(ui_clk), .d(rec_rst), .q(rec_rst_ui));
  assign fifo_clear = ui_clear_ui | rec_rst_ui;
  ddr_fifo u_ddr_fifo (
    .clk(ui_clk), .rst(mig_ui_rst), .clear(fifo_clear), .size_mib(cfg_ui.ddr_mib),
    .s_tdata(fr_tdata), .s_tvalid(fr_tvalid), .s_tready(fr_tready),
    .m_tdata(fifo_m_tdata), .m_tvalid(fifo_m_tvalid), .m_tready(fifo_m_tready),
    .space_ok(space_ok), .fill_bytes(fill_bytes), .fill_max_bytes(fill_max_bytes), .idle(ddr_idle),
    .wr_bytes(ddr_wr_bytes), .rd_bytes(ddr_rd_bytes),
    .mt_start(mt_start_ui), .mt_mib(cfg_ui.mt_mib), .mt_busy(mt_busy), .mt_done(mt_done),
    .mt_err_cnt(mt_err_cnt), .mt_first_err_addr(mt_first_err_addr),
    .app_addr(app_addr_f), .app_cmd(app_cmd), .app_en(app_en), .app_rdy(app_rdy),
    .app_wdf_data(app_wdf_data), .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren),
    .app_wdf_end(app_wdf_end), .app_wdf_rdy(app_wdf_rdy),
    .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid), .app_rd_data_end(app_rd_data_end),
    .app_sr_req(app_sr_req), .app_ref_req(app_ref_req), .app_zq_req(app_zq_req),
    .init_calib_complete(calib));

  // DTEST latch (audit D1): set by dtest_start_ui, cleared by ui_clear_ui / ui_rst.  While set, the DDR FIFO output goes
  // to rec_sink_chk and the core's data stream sees no valid beat (data_ring stays idle).
  logic dt_mode_q;
  always_ff @(posedge ui_clk) begin
    if (ui_rst || ui_clear_ui) dt_mode_q <= 1'b0;
    else if (dtest_start_ui)   dt_mode_q <= 1'b1;
  end
  logic         sink_tready;
  logic [47:0]  dt_beats;
  logic [31:0]  dt_hdr_err;
  assign s_data_tdata  = fifo_m_tdata;
  assign s_data_tvalid = fifo_m_tvalid & ~dt_mode_q;
  assign fifo_m_tready = dt_mode_q ? sink_tready : s_data_tready;

  rec_sink_chk u_rec_sink_chk (
    .clk(ui_clk), .rst(ui_rst), .clear(ui_clear_ui),
    .s_tdata(fifo_m_tdata), .s_tvalid(fifo_m_tvalid & dt_mode_q), .s_tready(sink_tready),
    .exp_sid(cfg_ui.sid), .beats(dt_beats), .hdr_err(dt_hdr_err));

  // ---------------------------------------------------------------------------
  // read-back verification: core tap (user_clk) -> vrf_tap -> X15 -> rec_verify (ui_clk)
  // ---------------------------------------------------------------------------
  logic [192:0] vrf_tdata;
  logic         vrf_tvalid, vrf_tready, vrf_ovf;
  vrf_tap #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES)) u_vrf_tap (
    .uclk(user_clk), .urst(user_rst), .wr_en(tap_wr_en), .wr_beat_addr(tap_wr_addr), .wr_data(tap_wr_data),
    .wr_be(tap_wr_be), .slot_lba_flat(tap_slot_lba_flat), .enable(tap_verify_mode),
    .vclk(ui_clk), .vrst(ui_rst), .m_tdata(vrf_tdata), .m_tvalid(vrf_tvalid), .m_tready(vrf_tready), .ovf(vrf_ovf));

  vrf_res_t     vrf_res;
  logic         end_found;
  logic [7:0]   mvp_flags;
  rec_verify #(.START_LBA(64'd0), .N_CTX(8), .CTX_SHIFT(5)) u_rec_verify (
    .clk(ui_clk), .rst(ui_rst), .clear(ui_clear_ui), .exp_sid(cfg_ui.rsid), .lbads(cfg_ui.lbads),
    .s_tdata(vrf_tdata), .s_tvalid(vrf_tvalid), .s_tready(vrf_tready), .tap_ovf(vrf_ovf),
    .res(vrf_res), .end_found(end_found), .mvp_flags(mvp_flags));

  // ui_stat_in (audit 1.3)
  always_comb begin
    ui_stat_in                   = '0;
    ui_stat_in.src_blocks        = src_blocks;
    ui_stat_in.wr_blocks         = wr_blocks;
    ui_stat_in.drop_blocks       = drop_blocks;
    ui_stat_in.pad_blocks        = pad_blocks;
    ui_stat_in.trunc_blocks      = trunc_blocks;
    ui_stat_in.bp_max_beats      = bp_max_beats;
    ui_stat_in.rt_stall_max      = rt_stall_max;
    ui_stat_in.step              = prof_step;
    ui_stat_in.seg_idx           = seg_idx;
    ui_stat_in.rep_cnt           = rep_cnt;
    ui_stat_in.prof_active       = prof_active;
    ui_stat_in.src_active        = fr_active;
    ui_stat_in.src_done          = fr_done;
    ui_stat_in.len_reached_s     = len_reached_s;
    ui_stat_in.prof_end_s        = prof_end_s;
    ui_stat_in.fill_bytes        = fill_bytes;
    ui_stat_in.fill_max_bytes    = fill_max_bytes;
    ui_stat_in.ddr_idle          = ddr_idle;
    ui_stat_in.ddr_wr_bytes      = ddr_wr_bytes;
    ui_stat_in.ddr_rd_bytes      = ddr_rd_bytes;
    ui_stat_in.calib             = calib;
    ui_stat_in.ui_ok             = ui_ok_q;
    ui_stat_in.mt_busy           = mt_busy;
    ui_stat_in.mt_done           = mt_done;
    ui_stat_in.mt_err_cnt        = mt_err_cnt;
    ui_stat_in.mt_first_err_addr = mt_first_err_addr;
    ui_stat_in.dt_beats          = dt_beats;
    ui_stat_in.dt_hdr_err        = dt_hdr_err;
    ui_stat_in.vrf               = vrf_res;
    ui_stat_in.end_found         = end_found;
  end

  // ---------------------------------------------------------------------------
  // VIO (JTAG), link_clk.  Spec 4.5: probe_in19/20/21 = ddrm/16, ddr/16, drp; probe_out0 -> core_rst_n,
  // probe_out1/2 -> rec_ctrl (levels, edge-detected above), probe_out6 -> vio_verify; other outputs unused.
  // Core status bits come from the X10 snapshot (core_stat, link domain) or through cdc_sync (quasi-static levels).
  // ---------------------------------------------------------------------------
  logic [3:0]  st4_s;                    // {enum_done, nvme_ready, running, done}: user_clk levels
  cdc_sync #(.W(4)) u_cdc_sync_st (.clk(link_clk), .d({status_enum_done, status_nvme_ready, status_running, status_done}), .q(st4_s));
  logic [32:0] fmt_s;                    // {format_done, format_ms}: static once done, display only
  cdc_sync #(.W(33)) u_cdc_sync_fmt (.clk(link_clk), .d({format_done, format_ms}), .q(fmt_s));
  logic [15:0] msg_s;                    // received PCIe Messages: quasi-static counter, display only
  cdc_sync #(.W(16)) u_cdc_sync_msg (.clk(link_clk), .d(stat_rx_msgs), .q(msg_s));
  logic [5:0]  ltssm_s;                  // pl_ltssm_state (pipe clock domain), display only
  cdc_sync #(.W(6)) u_cdc_sync_ltssm (.clk(link_clk), .d(dbg_ltssm), .q(ltssm_s));
  logic        perst_pin_s;
  cdc_sync #(.W(1)) u_cdc_sync_perst (.clk(link_clk), .d(pcie_perst_n), .q(perst_pin_s));

  logic cmd_err_nz;
  assign cmd_err_nz = |core_stat.cmd_errors;

  vio_status u_vio (
    .clk(link_clk),
    .probe_in0({cmd_err_nz, core_stat.lba_range_err, st4_s[0], st4_s[1], st4_s[2], st4_s[3], core_stat.gen2, core_stat.link_up}),
    .probe_in1({core_stat.error_detail, core_stat.error}),
    .probe_in2(core_stat.flags),
    .probe_in3(core_stat.link_width),
    .probe_in4({ctrl_stop, ctrl_start, perst_pin_s, link_fail, link_lost, link_retries}),
    .probe_in5(ltssm_s),
    .probe_in6(core_stat.bytes_done),
    .probe_in7(core_stat.cmds_done),
    .probe_in8(core_stat.cmd_errors),
    .probe_in9(core_stat.max_latency),
    .probe_in10(ui_stat.src_blocks[39:0]),                       // was run_cyc in ax7a200b_top
    .probe_in11(ui_stat.wr_blocks[39:0]),                        // was span_cyc
    .probe_in12({16'd0, ui_stat.vrf.vb}),                        // verified bytes
    .probe_in13(ui_stat.vrf.ve),                                 // erroneous beats
    .probe_in14({24'd0, ui_stat.vrf.fep}),                       // first payload error position
    .probe_in15({ui_stat.vrf.exp_dw, ui_stat.vrf.got_dw}),
    .probe_in16({vio_verify, core_stat.cmd_limit_done}),
    .probe_in17(core_stat.nsze),
    .probe_in18(core_stat.lbads),
    .probe_in19({9'd0, ui_stat.fill_max_bytes} >> 4),            // ddrm / 16
    .probe_in20({9'd0, ui_stat.fill_bytes} >> 4),                // ddr / 16
    .probe_in21({8'd0, ui_stat.drop_blocks}),                    // drp
    .probe_in22(fmt_s),
    .probe_in23(104'd0),                                         // slow-command log: not in the recorder
    .probe_in24(32'd0),
    .probe_in25(msg_s),
    .probe_out0(vio_rst),
    .probe_out1(vio_start_lvl),
    .probe_out2(vio_stop_lvl),
    .probe_out3(),
    .probe_out4(),
    .probe_out5(),
    .probe_out6(vio_verify),
    .probe_out7(),
    .probe_out8(),
    .probe_out9(),
    .probe_out10(),
    .probe_out11());

endmodule
