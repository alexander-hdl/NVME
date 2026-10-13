// =============================================================================
// axis_fifo_sync.sv — synchronous AXI-Stream FIFO, first-word-fall-through.
//
// Stores {tuser, tlast, tkeep, tdata} per beat in a small memory array with
// asynchronous read (maps to distributed RAM for small DEPTH).
//   * wr_ptr / rd_ptr are AW-bit wrapping pointers (DEPTH is a power of 2).
//   * count_q (AW+1 bits) is the occupancy; it is the single source of truth
//     for full (count == DEPTH) and empty (count == 0).
//   * Push and pop may happen in the same cycle (full-rate streaming).
//   * s_tready is low only when the FIFO is completely full (a push is not
//     accepted in the same cycle in which a full FIFO is popped).
// almost_full = (count >= DEPTH-2), usable as a conservative back-pressure hint.
// =============================================================================
module axis_fifo_sync #(
  parameter int W     = 128,
  parameter int UW    = 2,
  parameter int DEPTH = 16
) (
  input  logic                    clk,
  input  logic                    rst,
  input  logic [W-1:0]            s_tdata,
  input  logic [W/8-1:0]          s_tkeep,
  input  logic                    s_tlast,
  input  logic [UW-1:0]           s_tuser,
  input  logic                    s_tvalid,
  output logic                    s_tready,
  output logic [W-1:0]            m_tdata,
  output logic [W/8-1:0]          m_tkeep,
  output logic                    m_tlast,
  output logic [UW-1:0]           m_tuser,
  output logic                    m_tvalid,
  input  logic                    m_tready,
  output logic [$clog2(DEPTH):0]  count,
  output logic                    almost_full
);

  localparam int AW = $clog2(DEPTH);
  localparam int PW = W + W/8 + 1 + UW;

  logic [PW-1:0] mem [0:DEPTH-1];
  logic [AW-1:0] wr_ptr_q, rd_ptr_q;
  logic [AW:0]   count_q;

  logic push, pop;
  assign s_tready = ~count_q[AW];             // full when count == DEPTH
  assign m_tvalid = (count_q != '0);
  assign push     = s_tvalid && s_tready;
  assign pop      = m_tvalid && m_tready;

  always_ff @(posedge clk) begin
    if (rst) begin
      wr_ptr_q <= '0;
      rd_ptr_q <= '0;
      count_q  <= '0;
    end else begin
      if (push) wr_ptr_q <= wr_ptr_q + 1'b1;
      if (pop)  rd_ptr_q <= rd_ptr_q + 1'b1;
      count_q <= count_q + (AW+1)'(push) - (AW+1)'(pop);
    end
  end

  always_ff @(posedge clk) begin
    if (push) mem[wr_ptr_q] <= {s_tuser, s_tlast, s_tkeep, s_tdata};
  end

  assign {m_tuser, m_tlast, m_tkeep, m_tdata} = mem[rd_ptr_q];
  assign count       = count_q;
  assign almost_full = (count_q >= (AW+1)'(DEPTH-2));

endmodule
