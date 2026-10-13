// =============================================================================
// kc705_top.sv - board-level top for the KC705 (XC7K325T-2 FFG900) + M.2 NVMe SSD on an FMC HPC
// adapter.  Wraps nvme_host_top, gives every pin a home (fpga/constraints/kc705_board.xdc +
// kc705_fmc_m2.xdc) and contains an on-chip test-data generator, so a real bitstream can be built
// WITHOUT `loose_io`.  (research/04_rtl_review.md B1)
//
//   KC705 SYSCLK 200 MHz --> nvme_host_top (free_clk -> link_clk) --+
//   FMC GBTCLK0 100 MHz  --> pcie_7x refclk                         |  link_clk (free running) = board logic:
//   GT x4 <-> M.2 SSD                                               |  debouncers, start/stop latches, LEDs
//                                                                   +- user_clk (125 MHz) = test-data generator
//
// User interface (all push buttons / switches of the KC705 are active HIGH)
//   CPU_RESET   board reset (inverted to board_rst_n).  Starts a complete new PERST# / bring-up cycle and
//               clears the start/stop latches and the generator: this is the way to run again.
//   GPIO_SW_C   START: latches ctrl_start (stays set until CPU_RESET).
//   GPIO_SW_S   STOP : latches the stop request (only after START).  The generator stops producing beats first,
//               waits 4096 user clocks (33 us) for the stream to leave the ring, then ctrl_stop is asserted.
//   GPIO_SW_N/E/W  hold to switch the 8 LEDs to a debug page (see below).
//   GPIO_DIP_SW[1:0]  generator rate    : 00 full rate (one 128-bit beat per user clock = 2 GB/s),
//                                         01 1/2, 10 1/4, 11 1/8   (set BEFORE pressing START)
//   GPIO_DIP_SW[3:2]  total length      : 00 1 GiB, 01 8 GiB, 10 64 GiB, 11 unlimited (until STOP / namespace end).
//                                         The SSD must be big enough, otherwise the run ends with E_LBA_RANGE.
//                                         The fixed lengths are multiples of any SLOT_BYTES (<= 2 MiB), so no partial slot
//                                         is lost; with STOP in "unlimited" mode the last partially filled ring slot is not written.
// LEDs (main page, GPIO_LED_0 .. 7)
//   0 link_up   1 gen2   2 enum_done   3 nvme_ready   4 running   5 done
//   6 error (any of status_error, status_flags[6:0], link_fail, link_lost)
//   7 heartbeat (link_clk, bit 25 of a 26-bit counter, ~3 Hz blink; frozen = FPGA / SYSCLK dead)
// Debug pages (while the button is held; bit n = GPIO_LED_n)
//   GPIO_SW_N : status_flags[7:0]   (shutdown_timeout, doorbell_err, io_timeout, flush_skipped, link_fail, link_lost, io_error, rx_drop)
//   GPIO_SW_W : status_error[7:0]   (E_* code of host_sequencer)
//   GPIO_SW_E : {cmd_errors != 0, lba_range_err, link_width[1:0] (10 = x4), link_lost, link_fail, link_retries[1:0]}
//
// Test pattern (identical to sim/tb/tb_top.sv): beat number n = 0, 1, 2, ... (32 bit, wraps after 2^32 beats = 64 GiB),
//   128-bit AXI-S beat = { ~n, n ^ 32'hA5A5A5A5, n * 7, n }   (bits [127:96] .. [31:0]; little-endian in memory)
// START_LBA = 0, so beat n lies at byte offset 16*n of the namespace (LBA 0 = the first 512 B = 32 beats).
// The recorded data can be checked on a PC (SSD in a USB-NVMe adapter or a second slot):
//     dd if=/dev/nvme1n1 of=rec.img bs=1M count=1024 iflag=direct
//     python fpga/tools/check_pattern.py rec.img                      (or the device itself: ... /dev/nvme1n1 --bytes 1G)
// check_pattern.py verifies the beat pattern from LBA 0, reports the first mismatch and the verified byte count.
//
// Clocks: the generator runs on user_clk (= data_clk of nvme_host_top, so the async FIFO in data_ring sees a
// synchronous clock pair).  The buttons / LEDs / start-stop latches run on the free-running link_clk, so they work while
// user_clk is dead (before the first link-up, after link_fail).  All crossings are 2-FF ASYNC_REG synchronisers of
// quasi-static or sticky signals; ctrl_start / ctrl_stop are synchronised into user_clk inside nvme_host_top.
// =============================================================================
module kc705_top #(
  parameter bit          SIM_FAST      = 1'b0,
  parameter int unsigned DEBOUNCE_BITS = 21        // 2^21 link_clk cycles (200 MHz) = 10.5 ms
) (
  // ---- clocks ----
  input  logic         sys_clk_p,            // 100 MHz PCIe refclk from the adapter (FMC GBTCLK0_M2C)
  input  logic         sys_clk_n,
  input  logic         free_clk_p,           // KC705 SYSCLK 200 MHz LVDS
  input  logic         free_clk_n,
  // ---- PCIe ----
  output logic         pcie_perst_n,         // to the M.2 slot (FMC)
  output logic [3:0]   pci_exp_txp,
  output logic [3:0]   pci_exp_txn,
  input  logic [3:0]   pci_exp_rxp,
  input  logic [3:0]   pci_exp_rxn,
  // ---- KC705 user I/O (active-high buttons) ----
  input  logic         cpu_reset,            // CPU_RESET push button
  input  logic         gpio_sw_c,            // START
  input  logic         gpio_sw_s,            // STOP
  input  logic         gpio_sw_n,            // LED page: status_flags
  input  logic         gpio_sw_e,            // LED page: link info
  input  logic         gpio_sw_w,            // LED page: status_error
  input  logic [3:0]   gpio_dip_sw,
  output logic [7:0]   gpio_led
);

  // ===========================================================================
  // nvme_host_top
  // ===========================================================================
  logic         board_rst_n;
  assign board_rst_n = ~cpu_reset;

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

  logic sw_start, sw_stop, sw_n, sw_e, sw_w;
  kc705_debounce #(.BITS(DEBOUNCE_BITS)) u_db_c (.clk(link_clk), .rst(lrst), .d(gpio_sw_c), .q(sw_start));
  kc705_debounce #(.BITS(DEBOUNCE_BITS)) u_db_s (.clk(link_clk), .rst(lrst), .d(gpio_sw_s), .q(sw_stop));
  kc705_debounce #(.BITS(DEBOUNCE_BITS)) u_db_n (.clk(link_clk), .rst(lrst), .d(gpio_sw_n), .q(sw_n));
  kc705_debounce #(.BITS(DEBOUNCE_BITS)) u_db_e (.clk(link_clk), .rst(lrst), .d(gpio_sw_e), .q(sw_e));
  kc705_debounce #(.BITS(DEBOUNCE_BITS)) u_db_w (.clk(link_clk), .rst(lrst), .d(gpio_sw_w), .q(sw_w));

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
      // stop only after the generator has stopped (STOP button or the selected length reached)
      ctrl_stop  <= start_lat && quiet_sync[1];
    end
  end

  // status from user_clk -> link_clk (per-bit 2-FF; all of it is static or sticky for >> 1 link_clk)
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

  // link_lost / link_fail come straight from link_bringup (link_clk domain, valid while user_clk is dead)
  logic err_any;
  assign err_any = (|s_err) | (|s_flags[6:0]) | link_lost | link_fail;

  always_ff @(posedge link_clk) begin
    if      (sw_n) gpio_led <= s_flags;
    else if (sw_w) gpio_led <= s_err;
    else if (sw_e) gpio_led <= {s_cmd_err_nz, s_lba, s_width, link_lost, link_fail, link_retries};
    else           gpio_led <= {hb_cnt[25], err_any, s_done, s_run, s_ready, s_enum, s_gen2, s_link_up};
  end

  // ===========================================================================
  // user_clk domain: test-data generator
  // ===========================================================================
  (* ASYNC_REG = "TRUE" *) logic [1:0] urst_sync;
  always_ff @(posedge user_clk or negedge board_rst_n) begin
    if (!board_rst_n) urst_sync <= 2'b11;
    else              urst_sync <= {urst_sync[0], 1'b0};
  end
  logic urst;
  assign urst = urst_sync[1];

  (* ASYNC_REG = "TRUE" *) logic [1:0] halt_sync;
  (* ASYNC_REG = "TRUE" *) logic [3:0] dip_s0, dip_s1;
  always_ff @(posedge user_clk) begin
    halt_sync <= {halt_sync[0], stop_lat};
    dip_s0    <= gpio_dip_sw;
    dip_s1    <= dip_s0;
  end
  logic halt;
  assign halt = halt_sync[1];

  logic [1:0] rate_sel, len_sel;
  assign rate_sel = dip_s1[1:0];
  assign len_sel  = dip_s1[3:2];

  logic [2:0] phase;
  logic       tick;
  always_comb begin
    case (rate_sel)
      2'd0:    tick = 1'b1;                 // full rate
      2'd1:    tick = (phase[0]   == 1'b0); // 1/2
      2'd2:    tick = (phase[1:0] == 2'b0); // 1/4
      default: tick = (phase      == 3'b0); // 1/8
    endcase
  end

  logic [32:0] sent, limit;
  always_comb begin
    case (len_sel)
      2'd0:    limit = 33'd1 << 26;         // 1 GiB  = 2^26 beats of 16 B
      2'd1:    limit = 33'd1 << 29;         // 8 GiB
      default: limit = 33'd1 << 32;         // 64 GiB
    endcase
  end
  logic at_limit, can_send;
  assign at_limit = (len_sel != 2'd3) && (sent == limit);
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
        n7        <= n7 + 32'd7;            // = n * 7 (mod 2^32)
        sent      <= sent + 33'd1;
      end else begin
        gen_valid <= 1'b0;
      end
    end
  end
  assign s_data_tdata  = gen_data;
  assign s_data_tvalid = gen_valid;

  // generator finished (STOP requested or length reached, last beat accepted) + 4096 clocks of margin so
  // that the stream has left the async FIFO / ring before ctrl_stop is asserted
  logic [12:0] quiet_cnt;
  always_ff @(posedge user_clk) begin
    if (urst || !((halt || at_limit) && !gen_valid)) quiet_cnt <= '0;
    else if (!quiet_cnt[12])                         quiet_cnt <= quiet_cnt + 1'b1;
    gen_quiet <= quiet_cnt[12];
  end

endmodule

// -----------------------------------------------------------------------------
// push-button debouncer: 2-FF synchroniser + "stable for 2^BITS clocks" filter
// -----------------------------------------------------------------------------
module kc705_debounce #(
  parameter int unsigned BITS = 21
) (
  input  logic clk,
  input  logic rst,        // synchronous, active high
  input  logic d,          // asynchronous button level
  output logic q
);
  (* ASYNC_REG = "TRUE" *) logic [1:0] s;
  logic [BITS-1:0] cnt;
  always_ff @(posedge clk) begin
    s <= {s[0], d};
    if (rst) begin
      cnt <= '0;
      q   <= 1'b0;
    end else if (s[1] == q) begin
      cnt <= '0;
    end else begin
      cnt <= cnt + 1'b1;
      if (&cnt) q <= s[1];
    end
  end
endmodule
