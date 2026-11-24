// =============================================================================
// cdc_bus.sv — multi-bit bus transfer between clock domains, no acknowledge (WP0).
//
// Domains : src_clk -> dst_clk.
// Ports   : src_clk, src_rst, src_data[W], src_load, src_busy (registered) ;
//           dst_clk, dst_rst, dst_data[W], dst_upd (1 dst cycle, data valid the same
//           cycle), dst_tag[7:0] (counts updates, wraps; valid with dst_upd).
// Method  : the source copies src_data into hold_q and flips a toggle, then keeps hold_q
//           unchanged for HOLD_CYC src cycles (src_busy=1 meanwhile; a toggle is never
//           issued more often than every HOLD_CYC cycles).  The destination
//           synchronises the toggle (2 FF, ASYNC_REG), and one cycle after seeing it
//           change loads hold_q into dst_data_q (no ASYNC_REG: constrained by
//           set_max_delay -datapath_only / set_bus_skew, spec 8.3) and raises dst_upd.
// REPEAT  : 0 = src_load starts a transfer (ignored while src_busy);
//           1 = continuous: every HOLD_CYC cycles the current src_data is sent (src_load unused).
// HOLD_CYC: >= ceil(6 * f_src / f_dst) + 2 (>= 2).  Names hold_q / dst_data_q are used by XDC.
// Reset   : (I7) the toggle is NOT reset; src_rst stops new transfers and restarts the hold
//           timer (src_busy stays 1 during reset and for HOLD_CYC-1 cycles after it, so the
//           spacing of toggles is >= HOLD_CYC even across a reset); the destination copies the synchronised toggle during reset and the
//           first cycle after it without raising dst_upd.  dst_data_q is cleared.
// Spec    : STAND/RECORDER_SPEC.md 4.3.
// =============================================================================
module cdc_bus #(
  parameter int W        = 32,
  parameter int HOLD_CYC = 16,
  parameter bit REPEAT   = 1'b0
) (
  input  logic         src_clk,
  input  logic         src_rst,
  input  logic [W-1:0] src_data,     // src
  input  logic         src_load,     // src
  output logic         src_busy,     // src
  input  logic         dst_clk,
  input  logic         dst_rst,
  output logic [W-1:0] dst_data,     // dst
  output logic         dst_upd,      // dst
  output logic [7:0]   dst_tag       // dst
);

  localparam int CW = (HOLD_CYC > 2) ? $clog2(HOLD_CYC) : 1;

  // ------------------------------------------------------------ source
  logic          tgl_q = 1'b0;   // start value 0 at configuration (CODING_RULES 1: toggle register)
  logic [W-1:0]  hold_q;
  logic [CW-1:0] cnt_q;
  (* max_fanout = 24 *) logic busy_q;   // enable of the wide hold_q register: replicated by the tools
  logic          load_ok;


  assign load_ok = !src_rst && !busy_q && (REPEAT || src_load);

  always_ff @(posedge src_clk) begin
    if (src_rst) begin
      // reset keeps the "hold" guarantee: after reset the first transfer waits HOLD_CYC cycles
      cnt_q  <= CW'(HOLD_CYC - 1);
      busy_q <= (HOLD_CYC > 1);
    end else if (load_ok) begin
      tgl_q  <= ~tgl_q;
      cnt_q  <= CW'(HOLD_CYC - 1);
      busy_q <= (HOLD_CYC > 1);
    end else if (cnt_q != '0) begin
      cnt_q  <= cnt_q - 1'b1;
      busy_q <= (cnt_q != CW'(1));
    end
  end

  // data register: no reset and no src_rst in its enable (reset fan-out of a wide bus).  It may load while src_rst is
  // active or in the first cycle after configuration; hold_q is only looked at after a toggle, which is gated by src_rst.
  always_ff @(posedge src_clk)
    if (!busy_q && (REPEAT || src_load)) hold_q <= src_data;

  assign src_busy = busy_q;

  // ------------------------------------------------------------ destination
  (* ASYNC_REG = "TRUE" *) logic tgl_cdc_s0_q;
  (* ASYNC_REG = "TRUE" *) logic tgl_cdc_s1_q;
  logic         prev_q;
  logic         rst_d_q;
  (* max_fanout = 24 *) logic chg_q;   // change seen, take hold_q on the next cycle (enable of the wide dst_data_q)
  logic [W-1:0] dst_data_q;
  logic [7:0]   tag_q;

  always_ff @(posedge dst_clk) begin
    tgl_cdc_s0_q <= tgl_q;
    tgl_cdc_s1_q <= tgl_cdc_s0_q;
    rst_d_q      <= dst_rst;
    prev_q       <= tgl_cdc_s1_q;
    chg_q        <= !dst_rst && !rst_d_q && (tgl_cdc_s1_q ^ prev_q);
    dst_upd      <= 1'b0;
    if (dst_rst) begin
      dst_data_q <= '0;
      tag_q      <= '0;
    end else if (chg_q) begin
      dst_data_q <= hold_q;
      tag_q      <= tag_q + 8'd1;
      dst_upd    <= 1'b1;
    end
  end

  assign dst_data = dst_data_q;
  assign dst_tag  = tag_q;

endmodule
