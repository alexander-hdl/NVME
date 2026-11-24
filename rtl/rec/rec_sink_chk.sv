// =============================================================================
// rec_sink_chk.sv - DTEST sink: counts beats and checks block headers (WP3).
//
// Domain  : ui_clk.
// Ports   : clk, rst, clear ; s_tdata[127:0], s_tvalid, s_tready (always 1) ;
//           exp_sid ; beats[47:0] (accepted beats), hdr_err[31:0] (blocks whose beat 0 has a
//           bad magic, sid or pos).  AXIS, 256 beats per block, aligned to the first beat after
//           reset/clear.
// Check   : beat 0: magic == BLK_MAGIC, sid == exp_sid, pos == expected (0,1,2,... strictly in
//           order; a missing or repeated block is one error and keeps failing afterwards).
// Spec    : STAND/RECORDER_SPEC.md 4.4 (WP3 rec_sink_chk), 3.1.
// =============================================================================
module rec_sink_chk (
  input  logic         clk,
  input  logic         rst,
  input  logic         clear,
  input  logic [127:0] s_tdata,    // ui
  input  logic         s_tvalid,   // ui
  output logic         s_tready,   // ui
  input  logic [31:0]  exp_sid,    // ui
  output logic [47:0]  beats,      // ui
  output logic [31:0]  hdr_err     // ui
);
  import rec_pkg::*;

  logic [7:0]  j_q;
  logic [63:0] pos_q;
  logic        bad;

  assign s_tready = 1'b1;
  assign bad = (s_tdata[31:0] != BLK_MAGIC) || (s_tdata[63:32] != exp_sid) || (s_tdata[127:64] != pos_q);

  always_ff @(posedge clk) begin
    if (rst || clear) begin
      beats   <= '0;
      hdr_err <= '0;
      j_q     <= '0;
      pos_q   <= '0;
    end else if (s_tvalid) begin
      beats <= beats + 48'd1;
      j_q   <= j_q + 8'd1;
      if (j_q == 8'd0) begin
        if (bad) hdr_err <= hdr_err + 32'd1;
        pos_q <= pos_q + 64'd1;
      end
    end
  end

endmodule
