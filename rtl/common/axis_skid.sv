// =============================================================================
// axis_skid.sv — AXI-Stream register slice ("skid buffer"), full throughput.
//
// Purpose : cut the combinational path between s_tready and m_tready (and
//           register the data path) without losing a single beat of bandwidth.
//
// Algorithm: two storage slots.
//   * main slot  (m_*_q)  : drives the master port.
//   * skid slot  (sk_*_q) : catches one beat when the sink stalls in the same
//                           cycle in which the source was already promised
//                           "ready".
//   s_tready is simply ~sk_valid_q, i.e. it is a REGISTERED signal.
//   - main slot free (or being emptied this cycle):
//         skid occupied -> main <= skid;  skid empty -> main <= input
//   - main slot busy and stalled:
//         accepted input goes to the skid slot (s_tready drops next cycle).
// Steady state with m_tready=1 passes one beat per cycle with 1 cycle latency.
// =============================================================================
module axis_skid #(
  parameter int W  = 128,
  parameter int UW = 2
) (
  input  logic              clk,
  input  logic              rst,
  input  logic [W-1:0]      s_tdata,
  input  logic [W/8-1:0]    s_tkeep,
  input  logic              s_tlast,
  input  logic [UW-1:0]     s_tuser,
  input  logic              s_tvalid,
  output logic              s_tready,
  output logic [W-1:0]      m_tdata,
  output logic [W/8-1:0]    m_tkeep,
  output logic              m_tlast,
  output logic [UW-1:0]     m_tuser,
  output logic              m_tvalid,
  input  logic              m_tready
);

  localparam int PW = W + W/8 + 1 + UW;     // packed beat width

  logic [PW-1:0] in_word;
  logic [PW-1:0] m_word_q, sk_word_q;
  logic          m_valid_q, sk_valid_q;

  assign in_word  = {s_tuser, s_tlast, s_tkeep, s_tdata};
  assign s_tready = ~sk_valid_q;

  // control (reset)
  always_ff @(posedge clk) begin
    if (rst) begin
      m_valid_q  <= 1'b0;
      sk_valid_q <= 1'b0;
    end else begin
      if (!m_valid_q || m_tready) begin
        // main slot is free or is being emptied this cycle
        if (sk_valid_q) begin
          m_valid_q  <= 1'b1;
          sk_valid_q <= 1'b0;
        end else begin
          m_valid_q  <= s_tvalid;
        end
      end else if (s_tvalid && s_tready) begin
        // main stalled, input accepted -> park it in the skid slot
        sk_valid_q <= 1'b1;
      end
    end
  end

  // data (no reset)
  always_ff @(posedge clk) begin
    if (!m_valid_q || m_tready) begin
      if (sk_valid_q)    m_word_q <= sk_word_q;
      else if (s_tvalid) m_word_q <= in_word;
    end else if (s_tvalid && s_tready) begin
      sk_word_q <= in_word;
    end
  end

  assign {m_tuser, m_tlast, m_tkeep, m_tdata} = m_word_q;
  assign m_tvalid = m_valid_q;

endmodule
