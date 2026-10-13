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
//             PERST# cycle is done again (HOLD).
//   If the link drops in WAIT_POST / WAIT_GEN2 the same restart happens.
//
// SIM_FAST=1: the "millisecond" tick becomes a microsecond tick, so the
// default 100 ms timers last 100 us in simulation.
// =============================================================================
module link_bringup #(
  parameter int unsigned CLK_HZ        = 100_000_000,
  parameter int unsigned PERST_HOLD_MS = 100,
  parameter int unsigned POST_PERST_MS = 100,
  parameter int unsigned GEN2_WAIT_MS  = 100,
  parameter bit          SIM_FAST      = 1'b0
) (
  input  logic       clk,                 // free-running
  input  logic       rst,                 // synchronous, active high

  input  logic       user_lnk_up,
  input  logic [5:0] pl_ltssm_state,
  input  logic       pl_sel_lnk_rate,     // 1 = 5.0 GT/s
  input  logic [1:0] pl_sel_lnk_width,    // 2'b10 = x4
  input  logic       pl_link_gen2_cap,
  input  logic       pl_link_partner_gen2_supported,

  output logic       perst_n,             // to the SSD
  output logic       sys_rst_n,           // to pcie_7x
  output logic       link_ready,
  output logic       link_gen2,
  output logic [1:0] link_width
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
    S_READY
  } state_t;

  state_t state;

  // ---------------------------------------------------------------------------
  // Input synchronisers (core status is in the user_clk domain)
  // ---------------------------------------------------------------------------
  logic        lnk_up_s1,  lnk_up_s;
  logic [5:0]  ltssm_s1,   ltssm_s;
  logic        rate_s1,    rate_s;
  logic [1:0]  width_s1,   width_s;
  logic        cap_s1,     cap_s;
  logic        peer_s1,    peer_s;

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

  always_comb begin
    next_state = state;
    case (state)
      S_HOLD:       if (ms_cnt >= PERST_HOLD_MS)               next_state = S_REL_SYS;
      S_REL_SYS:    if (gap_cnt == 8'(SYS_TO_PERST_CLKS))     next_state = S_REL_PERST;
      S_REL_PERST:                                             next_state = S_WAIT_LNK;
      S_WAIT_LNK:   if (lnk_up_s)                              next_state = S_WAIT_POST;
      S_WAIT_POST:  if (!lnk_up_s)                             next_state = S_HOLD;
                    else if (ms_cnt >= POST_PERST_MS)          next_state = S_WAIT_GEN2;
      S_WAIT_GEN2:  if (!lnk_up_s)                             next_state = S_HOLD;
                    else if (gen2_stable || !both_gen2_capable ||
                             ms_cnt >= GEN2_WAIT_MS)           next_state = S_READY;
      S_READY:      if (!lnk_up_s)                             next_state = S_HOLD;
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
    end else begin
      state <= next_state;

      // small counter used only for the sys_rst_n -> perst_n gap
      if (state == S_REL_SYS) gap_cnt <= gap_cnt + 8'd1;
      else                    gap_cnt <= '0;

      // outputs: registered, derived from the NEXT state so they change in the
      // same cycle as the state register
      perst_n    <= (next_state == S_WAIT_LNK) || (next_state == S_WAIT_POST) ||
                    (next_state == S_WAIT_GEN2) || (next_state == S_READY);
      sys_rst_n  <= (next_state != S_HOLD);
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

endmodule
