// =============================================================================
// axis_fifo_async.sv — dual-clock FIFO (gray-code pointers, 2-FF synchronisers).
//
// Data only (no tkeep/tlast/tuser).  First-word-fall-through at the master port.
//
//  write side (s_clk)                       read side (m_clk)
//  ------------------                       -----------------
//  wr_bin -> wr_gray --(2FF)--------------> wr_gray_sync   -> empty compare
//  rd_gray_sync <--(2FF)-------------------- rd_gray       (full compare)
//
//  * Memory is written in s_clk and read SYNCHRONOUSLY in m_clk (so it maps to
//    block RAM).  The read side has one output register (out_*_q) which is the
//    FWFT head; it is refilled whenever it is empty or being consumed and the
//    FIFO is not empty -> one beat per m_clk cycle sustained.
//  * Full is computed on the write side from the NEXT write pointer and the
//    synchronised read pointer: gray(wr) == {~gray(rd)[AW:AW-1], gray(rd)[AW-2:0]}.
//    It is registered (conservative: becomes false only after the read pointer
//    has crossed the synchroniser).
//  * Capacity is DEPTH beats in the memory (+1 in the read output register).
//  * IMPORTANT: s_rst and m_rst must be asserted together (both pointers are
//    cleared independently, one without the other leaves the FIFO inconsistent).
// =============================================================================
module axis_fifo_async #(
  parameter int W     = 128,
  parameter int DEPTH = 512
) (
  input  logic          s_clk,
  input  logic          s_rst,
  input  logic [W-1:0]  s_tdata,
  input  logic          s_tvalid,
  output logic          s_tready,
  input  logic          m_clk,
  input  logic          m_rst,
  output logic [W-1:0]  m_tdata,
  output logic          m_tvalid,
  input  logic          m_tready
);

  localparam int AW = $clog2(DEPTH);

  logic [W-1:0] mem [0:DEPTH-1];

  function automatic logic [AW:0] bin2gray(input logic [AW:0] b);
    return (b >> 1) ^ b;
  endfunction

  // ---------------------------------------------------------------- write side
  logic [AW:0] wr_bin_q, wr_gray_q;
  logic        full_q;
  (* ASYNC_REG = "TRUE" *) logic [AW:0] rd_gray_sync1_q, rd_gray_sync2_q;
  logic [AW:0] rd_gray_q;                    // read-domain register (crossing)

  logic        push;
  logic [AW:0] wr_bin_n, wr_gray_n;

  assign s_tready  = ~full_q;
  assign push      = s_tvalid && ~full_q;
  assign wr_bin_n  = wr_bin_q + (AW+1)'(push);
  assign wr_gray_n = bin2gray(wr_bin_n);

  always_ff @(posedge s_clk) begin
    if (s_rst) begin
      wr_bin_q        <= '0;
      wr_gray_q       <= '0;
      full_q          <= 1'b0;
      rd_gray_sync1_q <= '0;
      rd_gray_sync2_q <= '0;
    end else begin
      wr_bin_q        <= wr_bin_n;
      wr_gray_q       <= wr_gray_n;
      rd_gray_sync1_q <= rd_gray_q;
      rd_gray_sync2_q <= rd_gray_sync1_q;
      full_q          <= (wr_gray_n == {~rd_gray_sync2_q[AW:AW-1], rd_gray_sync2_q[AW-2:0]});
    end
  end

  always_ff @(posedge s_clk) begin
    if (push) mem[wr_bin_q[AW-1:0]] <= s_tdata;
  end

  // ----------------------------------------------------------------- read side
  logic [AW:0] rd_bin_q;
  (* ASYNC_REG = "TRUE" *) logic [AW:0] wr_gray_sync1_q, wr_gray_sync2_q;
  logic          out_valid_q;
  logic [W-1:0]  out_data_q;
  logic          empty, fetch;
  logic [AW:0]   rd_bin_n;

  assign empty    = (rd_gray_q == wr_gray_sync2_q);
  assign fetch    = ~empty && (~out_valid_q || m_tready);
  assign rd_bin_n = rd_bin_q + 1'b1;

  always_ff @(posedge m_clk) begin
    if (m_rst) begin
      rd_bin_q        <= '0;
      rd_gray_q       <= '0;
      out_valid_q     <= 1'b0;
      wr_gray_sync1_q <= '0;
      wr_gray_sync2_q <= '0;
    end else begin
      wr_gray_sync1_q <= wr_gray_q;
      wr_gray_sync2_q <= wr_gray_sync1_q;
      if (fetch) begin
        rd_bin_q    <= rd_bin_n;
        rd_gray_q   <= bin2gray(rd_bin_n);
        out_valid_q <= 1'b1;
      end else if (m_tready) begin
        out_valid_q <= 1'b0;
      end
    end
  end

  always_ff @(posedge m_clk) begin
    if (fetch) out_data_q <= mem[rd_bin_q[AW-1:0]];
  end

  assign m_tdata  = out_data_q;
  assign m_tvalid = out_valid_q;

endmodule
