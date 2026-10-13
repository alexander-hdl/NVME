// =============================================================================
// link_bringup.sv - PERST# sequencing and link-up / Gen2 wait (ARCHITECTURE.md 5.12)
// -----------------------------------------------------------------------------
// Runs on a FREE-RUNNING clock (not user_clk: user_clk only exists once the
// pcie_7x MMCM has locked, which needs the very reset this module generates).
// The status inputs coming from the core (user_lnk_up, pl_*) belong to the
// user_clk domain, so they are first passed through 2-FF synchronisers here.
// All outputs are registered; the integrator synchronises link_ready /
// link_gen2 / link_width into user_clk.
//
// State machine
//
//   HOLD      perst_n=0, sys_rst_n=0 for PERST_HOLD_MS
//   REL_SYS   sys_rst_n=1 (core + GTs leave reset), perst_n still 0 for a few
//             clocks, so the core is out of reset before the SSD is
//   REL_PERST perst_n=1 (SSD leaves reset); the SSD now starts link training
//   WAIT_LNK  wait for user_lnk_up
//   WAIT_POST wait POST_PERST_MS after link-up (PCIe CEM: >=100 ms between
//             PERST# release and the first config request; counting from
//             link-up is the conservative reading of that rule)
//   WAIT_GEN2 wait until the link reached 5 GT/s in L0 (pl_sel_lnk_rate &&
//             ltssm==L0), or GEN2_WAIT_MS elapsed. Skipped at once when either
//             side is not Gen2 capable (nothing to wait for).
//   READY     link_ready=1. If user_lnk_up drops: link_ready=0 and a complete
//             PERST# cycle is done again (HOLD).  The drop also sets the STICKY flag
//             link_lost (see below).
//   If the link drops in WAIT_POST / WAIT_GEN2 the same restart happens (no link_lost:
//   no data run can have started before READY).
//   FAIL      reached when WAIT_LNK times out (user_lnk_up not seen within LINK_TIMEOUT_MS
//             after PERST# release) LINK_RETRY_MAX times in a row: each timeout before
//             that re-runs the whole PERST# cycle (HOLD) and counts one retry in
//             link_retries.  FAIL keeps perst_n = sys_rst_n = 0 (SSD and core stay in
//             reset) and link_fail = 1 until `rst`.
//
// Sticky flags (link_clk domain, cleared ONLY by `rst`, i.e. the board reset - NOT by the
// pcie_7x user_reset_out, which wipes everything in nvme_host_core on every PERST# cycle):
//   link_lost   the link dropped after READY.  The IP resets the whole host core during
//               the PERST# cycle, so the core cannot remember it; this flag survives.
//               The integrator synchronises it into user_clk (nvme_host_core input
//               `link_lost`, 2-FF inside the core): host_sequencer then refuses to leave
//               IDLE and reports E_LINK_LOST, so a recording that was interrupted is never
//               silently restarted from START_LBA (which would overwrite recorded data).
//   link_fail   the link never came up (see FAIL).  Reported as E_LINK_TIMEOUT.
//   link_retries  number of PERST# re-runs caused by link-up timeouts (2 bits: LINK_RETRY_MAX <= 3;
//               up to LINK_RETRY_MAX + 1 attempts in total: the first one plus the retries).
// These three are also meant for LEDs / a VIO / ILA in the link_clk domain: they are valid
// while the core is still held in reset (user_clk dead, every other status pin frozen).
//
// SIM_FAST=1: the "millisecond" tick becomes a microsecond tick, so the
// default 100 ms timers last 100 us in simulation.
// =============================================================================
module link_bringup #(
  // CLK_HZ must be the real frequency of `clk`.  In nvme_host_top `clk` is the
  // free-running board oscillator (free_clk_p/n -> IBUFDS -> BUFG; KC705 SYSCLK = 200 MHz),
  // not IBUFDS_GTE2.ODIV2 (unroutable together with the IP's O->BUFG, see pcie7x_wrapper).
  parameter int unsigned CLK_HZ         = 50_000_000,
  parameter int unsigned PERST_HOLD_MS  = 100,
  parameter int unsigned POST_PERST_MS  = 100,
  parameter int unsigned GEN2_WAIT_MS   = 100,
  parameter int unsigned LINK_TIMEOUT_MS = 2000,    // WAIT_LNK: user_lnk_up must rise within this time
  parameter int unsigned LINK_RETRY_MAX  = 3,       // PERST# re-runs after a timeout before link_fail
  parameter bit          SIM_FAST       = 1'b0
) (
  input  logic       clk,                 // free-running
  input  logic       rst,                 // synchronous, active high

  input  logic       user_lnk_up,
  input  logic [5:0] pl_ltssm_state,
  input  logic       pl_sel_lnk_rate,     // 1 = 5.0 GT/s
  input  logic [1:0] pl_sel_lnk_width,    // 2'b10 = x4
  input  logic       pl_link_gen2_cap,
  input  logic       pl_link_partner_gen2_supported,

  // PERST# polarity: perst_n is ACTIVE LOW and goes straight to the M.2 PERST# pin.
  // A board whose reset input is active high (Opsero FMC M.2 adapters take an
  // active-high reset on the FMC connector) needs ONE inverter between this output and
  // the pin (e.g. `assign pcie_perst = ~perst_n;` at the top level, or an OBUF with
  // an inverted LUT) - the logic here does not change.
  output logic       perst_n,             // to the SSD (active low)
  output logic       sys_rst_n,           // to pcie_7x
  output logic       link_ready,
  output logic       link_gen2,
  output logic [1:0] link_width,
  output logic       link_lost,           // sticky: link dropped after READY (cleared by rst only)
  output logic       link_fail,           // sticky: link-up timeout, retries exhausted (cleared by rst only)
  output logic [1:0] link_retries         // PERST# re-runs caused by link-up timeouts
);

  // ---------------------------------------------------------------------------
  // Constants
  // ---------------------------------------------------------------------------
  localparam logic [5:0] LTSSM_L0 = 6'h16;

  // clocks per tick: 1 ms (or 1 us in SIM_FAST); at least 1
  localparam int unsigned TICKS_PER_SEC = SIM_FAST ? 1_000_000 : 1_000;
  localparam int unsigned TICK_DIV_RAW  = CLK_HZ / TICKS_PER_SEC;
  localparam int unsigned TICK_DIV      = (TICK_DIV_RAW < 1) ? 1 : TICK_DIV_RAW;

  // sys_rst_n is released this many clocks before perst_n
  localparam int unsigned SYS_TO_PERST_CLKS = 16;

  typedef enum logic [2:0] {
    S_HOLD,
    S_REL_SYS,
    S_REL_PERST,
    S_WAIT_LNK,
    S_WAIT_POST,
    S_WAIT_GEN2,
    S_READY,
    S_FAIL
  } state_t;

  state_t state;

  // ---------------------------------------------------------------------------
  // Input synchronisers (core status is in the user_clk domain)
  // ---------------------------------------------------------------------------
  (* ASYNC_REG = "TRUE" *) logic        lnk_up_s1,  lnk_up_s;
  (* ASYNC_REG = "TRUE" *) logic [5:0]  ltssm_s1,   ltssm_s;
  (* ASYNC_REG = "TRUE" *) logic        rate_s1,    rate_s;
  (* ASYNC_REG = "TRUE" *) logic [1:0]  width_s1,   width_s;
  (* ASYNC_REG = "TRUE" *) logic        cap_s1,     cap_s;
  (* ASYNC_REG = "TRUE" *) logic        peer_s1,    peer_s;

  always_ff @(posedge clk) begin
    lnk_up_s1 <= user_lnk_up;                    lnk_up_s <= lnk_up_s1;
    ltssm_s1  <= pl_ltssm_state;                 ltssm_s  <= ltssm_s1;
    rate_s1   <= pl_sel_lnk_rate;                rate_s   <= rate_s1;
    width_s1  <= pl_sel_lnk_width;               width_s  <= width_s1;
    cap_s1    <= pl_link_gen2_cap;               cap_s    <= cap_s1;
    peer_s1   <= pl_link_partner_gen2_supported; peer_s   <= peer_s1;
  end

  // "Gen2 reached" must be seen on two consecutive samples: the multi-bit
  // ltssm bus can show transient values after the synchroniser.
  logic gen2_now_q, gen2_stable;
  always_ff @(posedge clk) begin
    gen2_now_q  <= rate_s && (ltssm_s == LTSSM_L0);
    gen2_stable <= gen2_now_q && rate_s && (ltssm_s == LTSSM_L0);
  end

  wire both_gen2_capable = cap_s && peer_s;

  // ---------------------------------------------------------------------------
  // Timebase: tick_div counts clocks, ms_cnt counts ticks since state entry
  // ---------------------------------------------------------------------------
  logic [$clog2(TICK_DIV+1)-1:0] div_cnt;
  logic [31:0]                   ms_cnt;
  logic [7:0]                    gap_cnt;
  logic                          timer_clr;    // restart the tick timer (state change)

  wire tick = (div_cnt == TICK_DIV - 1);

  always_ff @(posedge clk) begin
    if (rst || timer_clr) begin
      div_cnt <= '0;
      ms_cnt  <= '0;
    end else if (tick) begin
      div_cnt <= '0;
      if (ms_cnt != 32'hFFFF_FFFF) ms_cnt <= ms_cnt + 32'd1;
    end else begin
      div_cnt <= div_cnt + 1'b1;
    end
  end

  // ---------------------------------------------------------------------------
  // FSM
  // ---------------------------------------------------------------------------
  state_t next_state;
  logic [1:0] retry_cnt;

  always_comb begin
    next_state = state;
    case (state)
      S_HOLD:       if (ms_cnt >= PERST_HOLD_MS)               next_state = S_REL_SYS;
      S_REL_SYS:    if (gap_cnt == 8'(SYS_TO_PERST_CLKS))     next_state = S_REL_PERST;
      S_REL_PERST:                                             next_state = S_WAIT_LNK;
      S_WAIT_LNK:   if (lnk_up_s)                              next_state = S_WAIT_POST;
                    else if (ms_cnt >= LINK_TIMEOUT_MS)
                      next_state = (retry_cnt >= 2'(LINK_RETRY_MAX)) ? S_FAIL : S_HOLD;
      S_WAIT_POST:  if (!lnk_up_s)                             next_state = S_HOLD;
                    else if (ms_cnt >= POST_PERST_MS)          next_state = S_WAIT_GEN2;
      S_WAIT_GEN2:  if (!lnk_up_s)                             next_state = S_HOLD;
                    else if (gen2_stable || !both_gen2_capable ||
                             ms_cnt >= GEN2_WAIT_MS)           next_state = S_READY;
      S_READY:      if (!lnk_up_s)                             next_state = S_HOLD;
      S_FAIL:                                                  next_state = S_FAIL;
      default:                                                 next_state = S_HOLD;
    endcase
  end

  assign timer_clr = (next_state != state);

  always_ff @(posedge clk) begin
    if (rst) begin
      state      <= S_HOLD;
      gap_cnt    <= '0;
      perst_n    <= 1'b0;
      sys_rst_n  <= 1'b0;
      link_ready <= 1'b0;
      link_gen2  <= 1'b0;
      link_width <= 2'b00;
      link_lost  <= 1'b0;
      link_fail  <= 1'b0;
      retry_cnt  <= 2'd0;
    end else begin
      state <= next_state;

      // sticky flags (only `rst` clears them)
      if (state == S_READY && next_state == S_HOLD) link_lost <= 1'b1;
      if (next_state == S_FAIL)                     link_fail <= 1'b1;
      // retry bookkeeping: count a link-up timeout that restarts the PERST# cycle;
      // a link that reached READY starts a new retry budget
      if (state == S_WAIT_LNK && next_state == S_HOLD) retry_cnt <= retry_cnt + 2'd1;
      else if (next_state == S_READY)                  retry_cnt <= 2'd0;

      // small counter used only for the sys_rst_n -> perst_n gap
      if (state == S_REL_SYS) gap_cnt <= gap_cnt + 8'd1;
      else                    gap_cnt <= '0;

      // outputs: registered, derived from the NEXT state so they change in the
      // same cycle as the state register
      perst_n    <= (next_state == S_WAIT_LNK) || (next_state == S_WAIT_POST) ||
                    (next_state == S_WAIT_GEN2) || (next_state == S_READY);
      sys_rst_n  <= (next_state != S_HOLD) && (next_state != S_FAIL);
      link_ready <= (next_state == S_READY);

      if (next_state == S_READY) begin
        link_gen2  <= rate_s;
        link_width <= width_s;
      end else begin
        link_gen2  <= 1'b0;
        link_width <= 2'b00;
      end
    end
  end

  assign link_retries = retry_cnt;

endmodule
