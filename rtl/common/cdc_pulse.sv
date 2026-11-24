// =============================================================================
// cdc_pulse.sv — single-clock pulse transfer between clock domains (WP0).
//
// Domains : src_clk (source) -> dst_clk (destination).
// Ports   : src_clk, src_rst, src_pulse (1 src_clk cycle) ;
//           dst_clk, dst_rst, dst_pulse (registered, exactly 1 dst_clk cycle).
// Method  : toggle flip-flop in the source, 2 FF synchroniser (ASYNC_REG) and an
//           edge detector in the destination.
// Limits  : consecutive source pulses must be >= 3 dst_clk periods + 1 src_clk period apart
//           (closer pulses merge in the synchroniser).  Pulses emitted while the
//           destination is stopped or in reset may be lost (users repeat or poll state).
// Reset   : (I7, spec 4.3) the source toggle is NOT reset (its start value 0 is set at
//           configuration), so a source reset creates no pulse; src_rst only blocks new
//           pulses.  The destination, while in reset and in the first cycle after it, copies
//           the synchronised toggle without producing a pulse.
// Spec    : STAND/RECORDER_SPEC.md 4.3.
// =============================================================================
module cdc_pulse (
  input  logic src_clk,
  input  logic src_rst,
  input  logic src_pulse,    // src
  input  logic dst_clk,
  input  logic dst_rst,
  output logic dst_pulse     // dst
);

  // ------------------------------------------------------------ source
  logic tgl_q = 1'b0;        // start value 0 at configuration (CODING_RULES 1: toggle register)

  always_ff @(posedge src_clk)
    if (!src_rst && src_pulse) tgl_q <= ~tgl_q;

  // ------------------------------------------------------------ destination
  (* ASYNC_REG = "TRUE" *) logic tgl_cdc_s0_q;
  (* ASYNC_REG = "TRUE" *) logic tgl_cdc_s1_q;
  logic prev_q;
  logic rst_d_q;             // dst_rst delayed by one cycle: first cycle after reset also copies

  always_ff @(posedge dst_clk) begin
    tgl_cdc_s0_q <= tgl_q;
    tgl_cdc_s1_q <= tgl_cdc_s0_q;
    rst_d_q      <= dst_rst;
    prev_q       <= tgl_cdc_s1_q;
    dst_pulse    <= !dst_rst && !rst_d_q && (tgl_cdc_s1_q ^ prev_q);
  end

endmodule
