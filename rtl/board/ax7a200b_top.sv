// =============================================================================
// ax7a200b_top.sv - board-level top for the ALINX AX7A200B (AC7A200 core, XC7A200T-2FBG484I) plugged into the
// passive PCIe-slot -> M.2 adapter STAND/PCB/PCIe-m2 (Si52142 common 100 MHz refclk), see
// STAND/AX7A200B_ARCHITECTURE.md.  Wraps nvme_host_top unchanged; pins in fpga/constraints/ax7a200b_*.xdc.
//
//   AC7A200 SYS_CLK 200 MHz (R4/T4) --> nvme_host_top (free_clk -> link_clk) --+
//   slot REFCLK 100 MHz (F10/E10)   --> pcie_7x refclk                         |  link_clk: debouncers, latches, LEDs
//   GTP x4 (IP lanes 0/1 = slot lanes 0/1; lanes 2/3 = SFP cages, left empty)   |
//     -> the link trains to x2 (variant A of the architecture document)        +- user_clk (125 MHz): generator
//   PERST# : FPGA pin G21 (J11 pin 3) -> wire -> adapter jumper XP1 (PERST_M2#); the edge PERST# is not wired.
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
// There are no DIP switches on this board: rate and length are parameters (GEN_RATE_SEL / GEN_LEN_SEL, same
// encoding as kc705_top's DIP switches).  The test pattern is identical to kc705_top / sim/tb/tb_top.sv:
//   beat n = { ~n, n ^ 32'hA5A5A5A5, n * 7, n }, START_LBA = 0, check with fpga/tools/check_pattern.py.
//
// NOTE: the generator below is a copy of the one in kc705_top (trial build); it will move into a shared
// rtl/board/pattern_gen.sv.  kc705_debounce is reused from kc705_top.sv.
// =============================================================================
module ax7a200b_top #(
  parameter bit          SIM_FAST      = 1'b0,
  parameter int unsigned DEBOUNCE_BITS = 21,       // 2^21 link_clk cycles (200 MHz) = 10.5 ms
  parameter int unsigned N_SLOTS       = 4,        // 4 x 128 KiB ring: enough for Gen2 x2 (architecture doc 4.3)
  parameter logic [1:0]  GEN_RATE_SEL  = 2'd0,     // 0 full, 1 1/2, 2 1/4, 3 1/8
  parameter logic [1:0]  GEN_LEN_SEL   = 2'd0      // 0 1 GiB, 1 8 GiB, 2 64 GiB, 3 unlimited
) (
  // ---- clocks ----
  input  logic         sys_clk_p,            // 100 MHz PCIe refclk from the slot (adapter Si52142), MGTREFCLK1_216
  input  logic         sys_clk_n,
  input  logic         free_clk_p,           // AC7A200 SYS_CLK 200 MHz (bank 34)
  input  logic         free_clk_n,
  // ---- PCIe ----
  output logic         pcie_perst_n,         // G21 -> wire -> adapter XP1 PERST_M2#
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
  assign board_rst_n = rst_key_n;

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

  nvme_host_top #(
    .N_SLOTS(N_SLOTS),
    .SIM_FAST(SIM_FAST)
  ) u_host (
    .sys_clk_p(sys_clk_p), .sys_clk_n(sys_clk_n),
    .free_clk_p(free_clk_p), .free_clk_n(free_clk_n),
    .board_rst_n(board_rst_n), .pcie_perst_n(pcie_perst_n),
    .pci_exp_txp(pci_exp_txp), .pci_exp_txn(pci_exp_txn), .pci_exp_rxp(pci_exp_rxp), .pci_exp_rxn(pci_exp_rxn),
    .data_clk(user_clk),
    .s_data_tdata(s_data_tdata), .s_data_tvalid(s_data_tvalid), .s_data_tready(s_data_tready),
    .ctrl_start(ctrl_start), .ctrl_stop(ctrl_stop),
    .status_link_up(status_link_up), .status_gen2(status_gen2), .status_enum_done(status_enum_done),
    .status_nvme_ready(status_nvme_ready), .status_running(status_running), .status_done(status_done),
    .status_error(status_error), .status_error_detail(status_error_detail),
    .status_link_width(status_link_width), .status_lba_range_err(status_lba_range_err),
    .stat_bytes_done(), .stat_cmds_done(), .stat_cmd_errors(stat_cmd_errors), .stat_max_latency(),
    .status_flags(status_flags), .stat_first_error_lba(), .link_retries(link_retries),
    .link_lost(link_lost), .link_fail(link_fail),
    .link_clk_out(link_clk), .user_clk_out(user_clk));

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

  always_ff @(posedge link_clk) begin
    quiet_sync <= {quiet_sync[0], gen_quiet};
    if (lrst) begin
      start_lat  <= 1'b0;
      stop_lat   <= 1'b0;
      ctrl_start <= 1'b0;
      ctrl_stop  <= 1'b0;
    end else begin
      if (sw_start)             start_lat <= 1'b1;
      if (sw_stop && start_lat) stop_lat  <= 1'b1;
      ctrl_start <= start_lat;
      ctrl_stop  <= start_lat && quiet_sync[1];
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

  (* ASYNC_REG = "TRUE" *) logic [1:0] halt_sync;
  always_ff @(posedge user_clk) halt_sync <= {halt_sync[0], stop_lat};
  logic halt;
  assign halt = halt_sync[1];

  logic [2:0] phase;
  logic       tick;
  always_comb begin
    case (GEN_RATE_SEL)
      2'd0:    tick = 1'b1;
      2'd1:    tick = (phase[0]   == 1'b0);
      2'd2:    tick = (phase[1:0] == 2'b0);
      default: tick = (phase      == 3'b0);
    endcase
  end

  localparam logic [32:0] LIMIT = (GEN_LEN_SEL == 2'd0) ? (33'd1 << 26) :   // 1 GiB  = 2^26 beats of 16 B
                                  (GEN_LEN_SEL == 2'd1) ? (33'd1 << 29) :   // 8 GiB
                                                          (33'd1 << 32);    // 64 GiB
  logic [32:0] sent;
  logic at_limit, can_send;
  assign at_limit = (GEN_LEN_SEL != 2'd3) && (sent == LIMIT);
  assign can_send = !halt && !at_limit;

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
      if (can_send && tick) begin
        gen_valid <= 1'b1;
        gen_data  <= {~n, n ^ 32'hA5A5A5A5, n7, n};
        n         <= n + 32'd1;
        n7        <= n7 + 32'd7;
        sent      <= sent + 33'd1;
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
