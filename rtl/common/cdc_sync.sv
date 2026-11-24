// =============================================================================
// cdc_sync.sv — N-stage flip-flop synchroniser (WP0).
//
// Domain  : clk (destination).  d comes from another domain (no logic on it).
// Ports   : clk, d[W] (async, other domain), q[W] (registered, clk).  STAGES >= 2.
// USE ONLY for independent bits or quasi-static / sticky values: a multi-bit
// vector is NOT crossed coherently.  Multi-bit data goes through cdc_bus.
// Spec    : STAND/RECORDER_SPEC.md 4.3, CODING_RULES.md 3.
// =============================================================================
module cdc_sync #(
  parameter int W      = 1,
  parameter int STAGES = 2
) (
  input  logic         clk,
  input  logic [W-1:0] d,        // other domain
  output logic [W-1:0] q         // clk
);

  (* ASYNC_REG = "TRUE" *) logic [W-1:0] d_cdc_s0_q;                 // first FF, takes the metastability
  (* ASYNC_REG = "TRUE" *) logic [W-1:0] d_cdc_sn_q [1:STAGES-1];    // following stages

  always_ff @(posedge clk) begin
    d_cdc_s0_q    <= d;
    d_cdc_sn_q[1] <= d_cdc_s0_q;
    for (int i = 2; i < STAGES; i++) d_cdc_sn_q[i] <= d_cdc_sn_q[i-1];
  end

  assign q = d_cdc_sn_q[STAGES-1];

endmodule
