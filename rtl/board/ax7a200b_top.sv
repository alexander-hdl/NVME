// =============================================================================
// ax7a200b_top.sv - board-level top for the ALINX AX7A200B (AC7A200 core, XC7A200T-2FBG484I) plugged into the
// passive PCIe-slot -> M.2 adapter STAND/PCB/PCIe-m2 (Si52142 common 100 MHz refclk), see
// STAND/AX7A200B_ARCHITECTURE.md.  Wraps nvme_host_top unchanged; pins in fpga/constraints/ax7a200b_*.xdc.
//
//   AC7A200 SYS_CLK 200 MHz (R4/T4) --> nvme_host_top (free_clk -> link_clk) --+
//   slot REFCLK 100 MHz (F10/E10)   --> pcie_7x refclk                         |  link_clk: debouncers, latches, LEDs
//   GTP x4 (IP lanes 0/1 = slot lanes 0/1; lanes 2/3 = SFP cages, left empty)   |
//     -> the link trains to x2 (variant A of the architecture document)        +- user_clk (125 MHz): generator
//   PERST# : FPGA pin L16 = carrier PCIE_PERST = edge finger A11 -> adapter slot -> jumper XP1 (PERST_M2# <-> PERST_HOST#).
//
// User interface (ALINX keys are ACTIVE LOW: pressed = 0; LEDs are ACTIVE LOW: 0 = on)
//   RESET (F15)  board reset: new PERST# / bring-up cycle, clears the latches and the generator (= new run)
//   KEY1         START (latched until RESET)
//   KEY2         STOP  (only after START; generator stops, 4096 user clocks later ctrl_stop)
//   KEY3 held    LED page 1: {LED1..LED4} = {link_lost, link_fail, lba_range_err, cmd_errors != 0}
//   KEY4 held    LED page 2: {LED1..LED4} = {link_width[1], link_width[0], gen2, enum_done}   (x2 -> width = 01)
//   main page    LED1 link_up, LED2 nvme_ready, LED3 running (blinks) / done (steady),
//                LED4 error (steady) or heartbeat (~3 Hz blink, no error)
//   UART TX      idle high for now (status_uart comes in a later step)
//
// There are no DIP switches on this board: rate and length come from the VIO (probe_out5 rate_sel, probe_out4
// len_sel, both 0 after configuration = full rate, 1 GiB; same encoding as kc705_top's DIP switches; set them
// BEFORE START).  Formerly the parameters GEN_RATE_SEL / GEN_LEN_SEL (same
// encoding as kc705_top's DIP switches).  The test pattern is identical to kc705_top / sim/tb/tb_top.sv:
//   beat n = { ~n, n ^ 32'hA5A5A5A5, n * 7, n }, START_LBA = 0, check with fpga/tools/check_pattern.py.
//
// Debug: VIO core `vio_status` (IP fpga/ip/vio_status_a7/, clocked by link_clk) shows every status bit, the LTSSM state and
// the statistic counters over JTAG (fpga/tools/hw_status.tcl) and adds remote START / STOP / reset buttons.  Wide counters
// are captured per bit through 2 FFs from user_clk: a value may be torn while it changes (display only).
//
// NOTE: the generator below is a copy of the one in kc705_top (trial build); it will move into a shared
// rtl/board/pattern_gen.sv.  kc705_debounce is reused from kc705_top.sv.
// =============================================================================
module ax7a200b_top #(
  parameter bit          SIM_FAST      = 1'b0,
  parameter int unsigned DEBOUNCE_BITS = 21,       // 2^21 link_clk cycles (200 MHz) = 10.5 ms
  parameter int unsigned N_SLOTS       = 4,        // 4 x 128 KiB ring: enough for Gen2 x2 (architecture doc 4.3)
  // PERST# on L16 (edge A11): 0 = OBSERVE only (pin is an input; the SSD reset comes from the adapter's
  // SYS_RESET# supervisor / button SA1 via XP1 = PERST_M2# + PERST_HOST# + SYS_RESET#; link_bringup's PERST#
  // pulses then have no effect), 1 = DRIVE (Root Port drives PERST# through A11; XP1 = PERST_M2# <-> PERST_HOST# only)
  parameter bit          PERST_DRIVE   = 1'b0
) (
  // ---- clocks ----
  input  logic         sys_clk_p,            // 100 MHz PCIe refclk from the slot (adapter Si52142), MGTREFCLK1_216
  input  logic         sys_clk_n,
  input  logic         free_clk_p,           // AC7A200 SYS_CLK 200 MHz (bank 34)
  input  logic         free_clk_n,
  // ---- PCIe ----
  inout  wire          pcie_perst_n,         // L16 <-> edge A11 <-> adapter XP1 <-> M.2 PERST# (see PERST_DRIVE)
  output logic [3:0]   pci_exp_txp,
  output logic [3:0]   pci_exp_txn,
  input  logic [3:0]   pci_exp_rxp,
  input  logic [3:0]   pci_exp_rxn,
  // ---- AX7A200B user I/O ----
  input  logic         rst_key_n,            // RESET key, active low
  input  logic [3:0]   key_n,                // KEY1..KEY4, active low
  output logic [3:0]   led_n,                // LED1..LED4, active low
  output logic         uart_tx               // USB-UART (idle high)
);

  assign uart_tx = 1'b1;

  // ===========================================================================
  // nvme_host_top
  // ===========================================================================
  logic         board_rst_n;
  logic         vio_rst, vio_start, vio_stop;                // VIO probe_out (link_clk)
  logic         vio_gen_en;                                  // VIO probe_out3: generator enable (0 after configuration)
  logic [1:0]   vio_len_sel, vio_rate_sel;
  logic         vio_verify;                                  // VIO probe_out6: 1 = verify run (NVMe Read + compare), generator forced off
  logic [31:0]  vio_rate_step;                               // VIO probe_out7: real-time source rate, R = 2 GB/s * step / 2^32 (0 = rate_sel)
  logic [31:0]  vio_len_cmds;                                // VIO probe_out8: run length in 128 KiB commands (0 = len_sel)
  logic         vio_format;                                  // VIO probe_out9: NVMe Format NVM (SES=0) during the next bring-up
  logic         format_done;
  logic [31:0]  format_ms;
  logic [3:0]   stat_lbads;
  logic [63:0]  stat_nsze;
  logic [31:0]  verify_cmds;                                 // from len_sel: 1 / 8 / 64 GiB of 128 KiB commands, 0 = unlimited
  logic         cmd_limit_done;
  logic [63:0]  vfy_bytes, vfy_first_err_lba;
  logic [31:0]  vfy_err_beats, vfy_first_err_exp, vfy_first_err_got;                   // VIO probe_out4 / 5: 0 1 GiB, 1 8 GiB, 2 64 GiB, 3 unlimited /
                                                             //   0 full, 1 1/2, 2 1/4, 3 1/8
  assign board_rst_n = rst_key_n & ~vio_rst;

  logic         link_clk, user_clk;
  logic         ctrl_start, ctrl_stop;
  logic [127:0] s_data_tdata;
  logic         s_data_tvalid, s_data_tready;

  logic         status_link_up, status_gen2, status_enum_done, status_nvme_ready, status_running, status_done;
  logic [7:0]   status_error, status_error_detail, status_flags;
  logic [1:0]   status_link_width, link_retries;
  logic         status_lba_range_err;
  logic [31:0]  stat_cmd_errors;
  logic         link_lost, link_fail;
  logic [63:0]  stat_bytes_done, stat_cmds_done;
  logic [31:0]  stat_max_latency;
  logic         stat_cpl_evt;
  logic [31:0]  stat_cpl_lat;
  logic [31:0]  vio_slow_thr;                                // VIO probe_out10: slow-command threshold, user_clk cycles (0 = off)
  logic [9:0]   vio_slow_addr;                               // VIO probe_out11: slow-command log read address
  logic [103:0] slow_rd_data;
  logic [31:0]  slow_count;
  logic [5:0]   dbg_ltssm;
  logic         host_perst_n;                                // PERST# requested by link_bringup
  logic [303:0] dbg_uc;                                      // user_clk debug bus -> ILA

  assign pcie_perst_n = PERST_DRIVE ? host_perst_n : 1'bz;

  nvme_host_top #(
    .N_SLOTS(N_SLOTS),
    .SIM_FAST(SIM_FAST)
  ) u_host (
    .sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n),
    .free_clk_p(free_clk_p), .free_clk_n(free_clk_n),
    .board_rst_n(board_rst_n), .pcie_perst_n(host_perst_n),
    .pci_exp_txp(pci_exp_txp), .pci_exp_txn(pci_exp_txn), .pci_exp_rxp(pci_exp_rxp), .pci_exp_rxn(pci_exp_rxn),
    .data_clk(user_clk),
    .s_data_tdata(s_data_tdata), .s_data_tvalid(s_data_tvalid), .s_data_tready(s_data_tready),
    .ctrl_start(ctrl_start), .ctrl_stop(ctrl_stop),
    .status_link_up(status_link_up), .status_gen2(status_gen2), .status_enum_done(status_enum_done),
    .status_nvme_ready(status_nvme_ready), .status_running(status_running), .status_done(status_done),
    .status_error(status_error), .status_error_detail(status_error_detail),
    .status_link_width(status_link_width), .status_lba_range_err(status_lba_range_err),
    .stat_bytes_done(stat_bytes_done), .stat_cmds_done(stat_cmds_done), .stat_cmd_errors(stat_cmd_errors),
    .stat_max_latency(stat_max_latency), .stat_cpl_evt(stat_cpl_evt), .stat_cpl_lat(stat_cpl_lat),
    .status_flags(status_flags), .stat_first_error_lba(), .link_retries(link_retries),
    .link_lost(link_lost), .link_fail(link_fail),
    .link_clk_out(link_clk), .user_clk_out(user_clk), .dbg_ltssm_state(dbg_ltssm), .dbg_uc(dbg_uc),
    .verify_mode(vio_verify), .verify_cmds(verify_cmds), .cmd_limit_done(cmd_limit_done),
    .stat_lbads(stat_lbads), .stat_nsze(stat_nsze),
    .format_req(vio_format), .format_done(format_done), .format_ms(format_ms),
    .vfy_bytes(vfy_bytes), .vfy_err_beats(vfy_err_beats), .vfy_first_err_lba(vfy_first_err_lba),
    .vfy_first_err_exp(vfy_first_err_exp), .vfy_first_err_got(vfy_first_err_got));

  // verify run length = the recording length selected with len_sel (SLOT_BYTES = 128 KiB per command)
  always_ff @(posedge link_clk)
    verify_cmds <= (vio_len_cmds != 32'd0) ? vio_len_cmds :  // explicit length
                   (vio_len_sel == 2'd0) ? 32'd8192 :      // 1 GiB
                   (vio_len_sel == 2'd1) ? 32'd65536 :     // 8 GiB
                   (vio_len_sel == 2'd2) ? 32'd524288 :    // 64 GiB
                                           32'd0;          // unlimited

  // ILA on user_clk: TLP-level view of the PCIe core interface (bit map in nvme_host_top, dbg_uc)
  ila_pcie u_ila (.clk(user_clk), .probe0(dbg_uc));

  // ===========================================================================
  // link_clk domain: reset, debouncers, start/stop latches, heartbeat, LEDs
  // ===========================================================================
  (* ASYNC_REG = "TRUE" *) logic [1:0] lrst_sync;
  always_ff @(posedge link_clk or negedge board_rst_n) begin
    if (!board_rst_n) lrst_sync <= 2'b11;
    else              lrst_sync <= {lrst_sync[0], 1'b0};
  end
  logic lrst;
  assign lrst = lrst_sync[1];

  logic sw_start, sw_stop, sw_p1, sw_p2;
  kc705_debounce #(.BITS(DEBOUNCE_BITS)) u_db_k1 (.clk(link_clk), .rst(lrst), .d(~key_n[0]), .q(sw_start));
  kc705_debounce #(.BITS(DEBOUNCE_BITS)) u_db_k2 (.clk(link_clk), .rst(lrst), .d(~key_n[1]), .q(sw_stop));
  kc705_debounce #(.BITS(DEBOUNCE_BITS)) u_db_k3 (.clk(link_clk), .rst(lrst), .d(~key_n[2]), .q(sw_p1));
  kc705_debounce #(.BITS(DEBOUNCE_BITS)) u_db_k4 (.clk(link_clk), .rst(lrst), .d(~key_n[3]), .q(sw_p2));

  logic start_lat, stop_lat;
  logic gen_quiet;                                         // user_clk domain
  (* ASYNC_REG = "TRUE" *) logic [1:0] quiet_sync;         // generator finished: user_clk -> link_clk
  (* ASYNC_REG = "TRUE" *) logic [1:0] lim_sync;           // cmd_limit_done (verify run finished): user_clk -> link_clk

  always_ff @(posedge link_clk) begin
    quiet_sync <= {quiet_sync[0], gen_quiet};
    lim_sync   <= {lim_sync[0], cmd_limit_done & vio_verify};
    if (lrst) begin
      start_lat  <= 1'b0;
      stop_lat   <= 1'b0;
      ctrl_start <= 1'b0;
      ctrl_stop  <= 1'b0;
    end else begin
      if (sw_start || vio_start)              start_lat <= 1'b1;
      if ((sw_stop || vio_stop) && start_lat) stop_lat  <= 1'b1;
      ctrl_start <= start_lat;
      // stop after the generator has stopped (recording) or after all Reads completed (verify run)
      ctrl_stop  <= start_lat && (quiet_sync[1] || lim_sync[1]);
    end
  end

  // status user_clk -> link_clk (per-bit 2-FF; static or sticky for >> 1 link_clk)
  logic cmd_err_nz;
  always_ff @(posedge user_clk) cmd_err_nz <= |stat_cmd_errors;

  logic [26:0] st_raw;
  assign st_raw = {cmd_err_nz, status_lba_range_err, status_link_width, status_error, status_flags,
                   status_done, status_running, status_nvme_ready, status_enum_done, status_gen2, status_link_up};
  (* ASYNC_REG = "TRUE" *) logic [26:0] st_s0, st_s1;
  always_ff @(posedge link_clk) begin
    st_s0 <= st_raw;
    st_s1 <= st_s0;
  end

  logic       s_link_up, s_gen2, s_enum, s_ready, s_run, s_done, s_cmd_err_nz, s_lba;
  logic [7:0] s_flags, s_err;
  logic [1:0] s_width;
  assign {s_cmd_err_nz, s_lba, s_width, s_err, s_flags, s_done, s_run, s_ready, s_enum, s_gen2, s_link_up} = st_s1;

  // ---- debug: VIO (JTAG) ------------------------------------------------------------
  (* ASYNC_REG = "TRUE" *) logic [5:0]   ltssm_s0, ltssm_s1;
  (* ASYNC_REG = "TRUE" *) logic [1:0]   perst_pin_s;       // actual level on L16 (= SSD PERST#)
  (* ASYNC_REG = "TRUE" *) logic [7:0]   det_s0, det_s1;    // status_error_detail (fail_step of the failing module)
  (* ASYNC_REG = "TRUE" *) logic [191:0] cnt_s0, cnt_s1;    // {max_latency, cmd_errors, cmds_done, bytes_done}
  (* ASYNC_REG = "TRUE" *) logic [79:0]  tm_s0, tm_s1;      // {span_cyc, run_cyc} (user_clk timers, see below)
  (* ASYNC_REG = "TRUE" *) logic [223:0] vfy_s0, vfy_s1;    // verify results (static once the run is done)
  (* ASYNC_REG = "TRUE" *) logic [67:0]  ns_s0, ns_s1;      // {lbads, nsze}
  (* ASYNC_REG = "TRUE" *) logic [32:0]  fmt_s0, fmt_s1;    // {format_done, format_ms}
  (* ASYNC_REG = "TRUE" *) logic [135:0] slw_s0, slw_s1;    // {slow_count, slow_rd_data} (read between JTAG polls)
  (* ASYNC_REG = "TRUE" *) logic [119:0] src_s0, src_s1;    // {due, backlog_now, backlog_max} (beats, 40 bit each)
  always_ff @(posedge link_clk) begin
    ltssm_s0 <= dbg_ltssm;
    ltssm_s1 <= ltssm_s0;
    perst_pin_s <= {perst_pin_s[0], pcie_perst_n};
    det_s0   <= status_error_detail;
    det_s1   <= det_s0;
    cnt_s0   <= {stat_max_latency, stat_cmd_errors, stat_cmds_done, stat_bytes_done};
    cnt_s1   <= cnt_s0;
    tm_s0    <= {span_cyc, run_cyc};
    tm_s1    <= tm_s0;
    vfy_s0   <= {vfy_first_err_exp, vfy_first_err_got, vfy_first_err_lba, vfy_err_beats, vfy_bytes};
    vfy_s1   <= vfy_s0;
    ns_s0    <= {stat_lbads, stat_nsze};
    ns_s1    <= ns_s0;
    fmt_s0   <= {format_done, format_ms};
    fmt_s1   <= fmt_s0;
    slw_s0   <= {slow_count, slow_rd_data};
    slw_s1   <= slw_s0;
    src_s0   <= {due_cnt, backlog_q, backlog_max};
    src_s1   <= src_s0;
  end

  vio_status u_vio (
    .clk(link_clk),
    .probe_in0({s_cmd_err_nz, s_lba, s_done, s_run, s_ready, s_enum, s_gen2, s_link_up}),  // 8
    .probe_in1({det_s1, s_err}),                                                          // 16 {error_detail, status_error}
    .probe_in2(s_flags),                                                                  // 8 status_flags
    .probe_in3(s_width),                                                                  // 2 link width (01 = x2)
    .probe_in4({stop_lat, start_lat, perst_pin_s[1], link_fail, link_lost, link_retries}),  // 7 [4] = PERST# pin level
    .probe_in5(ltssm_s1),                                                                 // 6 pl_ltssm_state
    .probe_in6(cnt_s1[63:0]),                                                             // 64 stat_bytes_done
    .probe_in7(cnt_s1[127:64]),                                                           // 64 stat_cmds_done
    .probe_in8(cnt_s1[159:128]),                                                          // 32 stat_cmd_errors
    .probe_in9(cnt_s1[191:160]),                                                          // 32 stat_max_latency
    .probe_in10(tm_s1[39:0]),                                                             // 40 run_cyc  (RUN + DRAIN)
    .probe_in11(tm_s1[79:40]),                                                            // 40 span_cyc (1st..last completion)
    .probe_in12(vfy_s1[63:0]),                                                            // 64 vfy_bytes
    .probe_in13(vfy_s1[95:64]),                                                           // 32 vfy_err_beats
    .probe_in14(vfy_s1[159:96]),                                                          // 64 vfy_first_err_lba
    .probe_in15(vfy_s1[223:160]),                                                         // 64 {first_err_exp, first_err_got}
    .probe_in16({vio_verify, lim_sync[1]}),                                               // 2 {verify mode, cmd_limit_done}
    .probe_in17(ns_s1[63:0]),                                                             // 64 namespace size (LBAs)
    .probe_in18(ns_s1[67:64]),                                                            // 4 lbads
    .probe_in19(src_s1[39:0]),                                                            // 40 backlog_max (beats of 16 B)
    .probe_in20(src_s1[79:40]),                                                           // 40 backlog now (beats)
    .probe_in21(src_s1[119:80]),                                                          // 40 beats produced by the source (due)
    .probe_in22(fmt_s1),                                                                  // 33 {format_done, format_ms}
    .probe_in23(slw_s1[103:0]),                                                           // 104 slow log entry {t_cpl, idx, lat}
    .probe_in24(slw_s1[135:104]),                                                         // 32 slow commands counted
    .probe_out0(vio_rst),                                                                 // 1 board reset (hold high)
    .probe_out1(vio_start),                                                               // 1 START
    .probe_out2(vio_stop),                                                                // 1 STOP
    .probe_out3(vio_gen_en),                                                              // 1 generator enable
    .probe_out4(vio_len_sel),                                                             // 2 length select
    .probe_out5(vio_rate_sel),                                                            // 2 rate select
    .probe_out6(vio_verify),                                                              // 1 verify run
    .probe_out7(vio_rate_step),                                                           // 32 real-time source rate step
    .probe_out8(vio_len_cmds),                                                            // 32 run length in commands
    .probe_out9(vio_format),                                                              // 1 Format NVM at the next bring-up
    .probe_out10(vio_slow_thr),                                                           // 32 slow-command threshold (cycles)
    .probe_out11(vio_slow_addr));                                                         // 10 slow log read address

  logic [25:0] hb_cnt;
  always_ff @(posedge link_clk) hb_cnt <= hb_cnt + 1'b1;      // free running, no reset needed

  logic err_any;
  assign err_any = (|s_err) | (|s_flags[6:0]) | link_lost | link_fail;

  logic [3:0] led;                                            // [0] = LED1
  always_ff @(posedge link_clk) begin
    if      (sw_p1) led <= {s_cmd_err_nz, s_lba, link_fail, link_lost};
    else if (sw_p2) led <= {s_enum, s_gen2, s_width[0], s_width[1]};
    else            led <= {err_any | hb_cnt[25],
                            s_done | (s_run & hb_cnt[23]),
                            s_ready,
                            s_link_up};
  end
  assign led_n = ~led;

  // ===========================================================================
  // user_clk domain: test-data generator (copy of kc705_top, rate/length from parameters)
  // ===========================================================================
  (* ASYNC_REG = "TRUE" *) logic [1:0] urst_sync;
  always_ff @(posedge user_clk or negedge board_rst_n) begin
    if (!board_rst_n) urst_sync <= 2'b11;
    else              urst_sync <= {urst_sync[0], 1'b0};
  end
  logic urst;
  assign urst = urst_sync[1];

  (* ASYNC_REG = "TRUE" *) logic [1:0] halt_sync, gen_en_sync;
  always_ff @(posedge user_clk) begin
    halt_sync   <= {halt_sync[0], stop_lat};
    gen_en_sync <= {gen_en_sync[0], vio_gen_en & ~vio_verify};   // verify run: generator always off
  end
  logic halt;
  assign halt = halt_sync[1];

  // ---- run timers (user_clk = 125 MHz), cleared by the board / VIO reset ----------------------------------
  //  run_cyc  : cycles in sequencer state RUN (5) or DRAIN (6)  -> bytes / run_cyc  = rate of the whole recording
  //  span_cyc : cycles from the first to the last change of stat_bytes_done (first .. last Write completion)
  //             -> (bytes - one command) / span_cyc = sustained SSD rate without the start-up
  logic [39:0] run_cyc, span_cyc, span_cnt;
  logic [63:0] bytes_q;
  logic        span_on;
  always_ff @(posedge user_clk) begin
    bytes_q <= stat_bytes_done;
    if (urst) begin
      run_cyc  <= '0;
      span_cyc <= '0;
      span_cnt <= '0;
      span_on  <= 1'b0;
    end else begin
      if (dbg_uc[3:0] == 4'd5 || dbg_uc[3:0] == 4'd6) run_cyc <= run_cyc + 40'd1;
      if (span_on) span_cnt <= span_cnt + 40'd1;
      if (stat_bytes_done != bytes_q) begin
        if (!span_on) span_on <= 1'b1;              // first completion: start counting
        else          span_cyc <= span_cnt + 40'd1; // every later completion: latch the elapsed time
      end
    end
  end

  // ---- slow-command log (debug): every data command with latency >= vio_slow_thr, see slow_cmd_log.sv ------
  (* ASYNC_REG = "TRUE" *) logic [41:0] slw_c0, slw_c1;       // {read address, threshold} (quasi-static)
  always_ff @(posedge user_clk) begin
    slw_c0 <= {vio_slow_addr, vio_slow_thr};
    slw_c1 <= slw_c0;
  end
  slow_cmd_log #(.DEPTH(1024)) u_slow_log (
    .clk(user_clk), .rst(urst),
    .evt(stat_cpl_evt), .lat(stat_cpl_lat), .t_now(run_cyc), .idx(stat_cmds_done[31:0]),
    .thr(slw_c1[31:0]), .rd_addr(slw_c1[41:32]),
    .rd_data(slow_rd_data), .count(slow_count));

  // rate / length select from the VIO (quasi-static, set before START)
  (* ASYNC_REG = "TRUE" *) logic [67:0] sel_s0, sel_s1;
  logic [1:0]  rate_sel, len_sel;
  logic [31:0] rate_step, len_cmds;
  always_ff @(posedge user_clk) begin
    sel_s0 <= {vio_len_cmds, vio_rate_step, vio_len_sel, vio_rate_sel};
    sel_s1 <= sel_s0;
  end
  assign {len_cmds, rate_step, len_sel, rate_sel} = sel_s1;
  logic rt_mode;                                   // real-time source model (rate_step != 0)
  assign rt_mode = (rate_step != 32'd0);

  logic [2:0] phase;
  logic       tick;
  always_comb begin
    case (rate_sel)
      2'd0:    tick = 1'b1;
      2'd1:    tick = (phase[0]   == 1'b0);
      2'd2:    tick = (phase[1:0] == 2'b0);
      default: tick = (phase      == 3'b0);
    endcase
  end

  logic [39:0] limit_q;
  logic        unlimited_q;
  always_ff @(posedge user_clk) begin
    limit_q <= (len_cmds != 32'd0) ? ({8'd0, len_cmds} << 13) :   // len_cmds x 128 KiB = x 8192 beats
               (len_sel == 2'd0)   ? (40'd1 << 26) :              // 1 GiB  = 2^26 beats of 16 B
               (len_sel == 2'd1)   ? (40'd1 << 29) :              // 8 GiB
                                     (40'd1 << 32);               // 64 GiB
    unlimited_q <= (len_cmds == 32'd0) && (len_sel == 2'd3);
  end
  logic [39:0] sent;
  logic at_limit, can_send;
  assign at_limit = !unlimited_q && (sent == limit_q);

  // ---- real-time source model (rate_step != 0) -----------------------------------------------------------
  // A real sensor cannot wait.  The source "produces" one 16-byte beat whenever the phase accumulator
  // overflows (R = 16 B x 125 MHz x step / 2^32 = 2 GB/s x step / 2^32, resolution ~0.47 B/s) and counts it
  // in due_cnt.  The generator hands beats to the host as fast as s_data_tready allows; the beats produced but
  // not yet taken (backlog = due_cnt - sent) are what an extra FIFO in front of the host would have to hold.
  // backlog_max x 16 B is therefore the memory a lossless recorder needs IN ADDITION to the on-chip ring
  // (N_SLOTS x 128 KiB + the data_ring async FIFO) at this rate on this SSD.
  logic [31:0] acc;
  logic        acc_c;
  logic [39:0] due_cnt, backlog_q, backlog_max;
  logic        src_on;
  // The source starts with the recording (sequencer in RUN), as a real sensor is enabled once the recorder is
  // ready; otherwise the bring-up time (enumeration + NVMe init, ~0.1 s) would show up as backlog.
  logic run_seen;
  always_ff @(posedge user_clk)
    if (urst) run_seen <= 1'b0; else if (status_running) run_seen <= 1'b1;
  assign src_on = gen_en_sync[1] && run_seen && !halt && (unlimited_q || (due_cnt != limit_q));
  always_ff @(posedge user_clk) begin
    if (urst) begin
      acc         <= '0;
      acc_c       <= 1'b0;
      due_cnt     <= '0;
      backlog_q   <= '0;
      backlog_max <= '0;
    end else begin
      if (rt_mode && src_on) {acc_c, acc} <= {1'b0, acc} + {1'b0, rate_step};
      else                   acc_c <= 1'b0;
      if (acc_c) due_cnt <= due_cnt + 40'd1;
      backlog_q <= due_cnt - sent;                               // stage 1: subtract
      if (backlog_q > backlog_max) backlog_max <= backlog_q;     // stage 2: compare
    end
  end

  // gen_en = 0: no beat is produced, the ring stays empty and RUN submits no Write (bring-up test without
  // touching the SSD contents).  With the VIO it starts at 0; set it to 1 BEFORE START for a recording.
  // rt_mode: a beat may be handed over only if the source has produced it (due_cnt > sent).
  assign can_send = gen_en_sync[1] && !halt && !at_limit && (!rt_mode || (due_cnt != sent));

  logic [31:0]  n, n7;
  logic         gen_valid;
  logic [127:0] gen_data;
  always_ff @(posedge user_clk) begin
    phase <= phase + 1'b1;
    if (urst) begin
      gen_valid <= 1'b0;
      gen_data  <= '0;
      n         <= '0;
      n7        <= '0;
      sent      <= '0;
    end else if (!gen_valid || s_data_tready) begin
      if (can_send && (rt_mode || tick)) begin
        gen_valid <= 1'b1;
        gen_data  <= {~n, n ^ 32'hA5A5A5A5, n7, n};
        n         <= n + 32'd1;
        n7        <= n7 + 32'd7;
        sent      <= sent + 40'd1;
      end else begin
        gen_valid <= 1'b0;
      end
    end
  end
  assign s_data_tdata  = gen_data;
  assign s_data_tvalid = gen_valid;

  logic [12:0] quiet_cnt;
  always_ff @(posedge user_clk) begin
    if (urst || !((halt || at_limit) && !gen_valid)) quiet_cnt <= '0;
    else if (!quiet_cnt[12])                         quiet_cnt <= quiet_cnt + 1'b1;
    gen_quiet <= quiet_cnt[12];
  end

endmodule
