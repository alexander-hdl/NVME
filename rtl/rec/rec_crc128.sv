// =============================================================================
// rec_crc128.sv - CRC-32 (zlib) engine, one 128-bit beat per clock (WP3).
//
// Domain  : ui_clk (any single clock).
// Ports   : clk ; first (this beat is beat 0: state seeds from CRC32_INIT) ;
//           en (beat valid, state updates) ; d[127:0] (byte k = d[8k+7:8k]) ;
//           crc_q = raw state register.  The zlib value after the last beat is ~crc_q.
// Spec    : STAND/RECORDER_SPEC.md 3.1 (CRC over bytes 0..4079), crc32_128 from rec_pkg.
// =============================================================================
module rec_crc128 (
  input  logic         clk,
  input  logic         first,     // ui
  input  logic         en,        // ui
  input  logic [127:0] d,         // ui
  output logic [31:0]  crc_q      // ui, raw state (no xorout)
);
  import rec_pkg::*;

  always_ff @(posedge clk)
    if (en) crc_q <= crc32_128(first ? CRC32_INIT : crc_q, d);

endmodule
