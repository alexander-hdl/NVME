// =============================================================================
// timer_ms.sv — millisecond tick generator.
//
// A free-running counter reloads every DIV clocks and emits a one-clock pulse
// on tick_ms.  DIV = CLK_HZ/1000 (1 ms) or, when SIM_FAST=1, CLK_HZ/1_000_000
// (1 us) so that "milliseconds" cost 1000x fewer simulated cycles.
// =============================================================================
module timer_ms #(
  parameter int CLK_HZ   = 125_000_000,
  parameter int SIM_FAST = 0
) (
  input  logic clk,
  input  logic rst,
  output logic tick_ms
);

  localparam int DIV   = (SIM_FAST != 0) ? (CLK_HZ / 1_000_000) : (CLK_HZ / 1_000);
  localparam int CNT_W = (DIV > 1) ? $clog2(DIV) : 1;

  logic [CNT_W-1:0] cnt_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      cnt_q   <= '0;
      tick_ms <= 1'b0;
    end else if (cnt_q == CNT_W'(DIV - 1)) begin
      cnt_q   <= '0;
      tick_ms <= 1'b1;
    end else begin
      cnt_q   <= cnt_q + 1'b1;
      tick_ms <= 1'b0;
    end
  end

endmodule
