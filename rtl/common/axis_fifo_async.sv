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
//  * RESET (cross-domain).  Each side's reset is also synchronised (2 FF, ASYNC_REG)
//    into the OTHER domain and ORed into that side's local reset:
//        write side is held in reset while  s_rst | m_rst_sync
//        read  side is held in reset while  m_rst | s_rst_sync
//    so a reset of either domain clears both pointers and the FIFO stays held until
//    BOTH domains are out of reset (plus one settle cycle per side).
//  * s_tready is LOW while the write side is held in reset (s_rst, the synchronised
//    m_rst, or the settle cycle after them) and m_tvalid is LOW while the read side is
//    held in reset.  AXI-S therefore never completes a handshake that the FIFO
//    drops: "tvalid && tready" always means "the beat is stored".
//    s_rst is allowed to assert asynchronously (reset synchroniser with async assert),
//    which is why s_tready uses it combinationally (tready is a function of the reset
//    only through that one OR gate).
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

  // m_rst seen from the write side (2-FF synchroniser; power-up value = "in reset")
  (* ASYNC_REG = "TRUE" *) logic m_rst_s1_q = 1'b1, m_rst_s2_q = 1'b1;
  logic s_hold_q = 1'b1;                      // settle cycle after a reset
  logic s_rst_i;                              // write side held in reset
  always_ff @(posedge s_clk) begin
    m_rst_s1_q <= m_rst;
    m_rst_s2_q <= m_rst_s1_q;
    s_hold_q   <= s_rst | m_rst_s2_q;
  end
  assign s_rst_i = s_rst | m_rst_s2_q;

  assign s_tready  = ~full_q & ~s_rst_i & ~s_hold_q;
  assign push      = s_tvalid && s_tready;
  assign wr_bin_n  = wr_bin_q + (AW+1)'(push);
  assign wr_gray_n = bin2gray(wr_bin_n);

  always_ff @(posedge s_clk) begin
    if (s_rst_i) begin
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

  // s_rst seen from the read side (s_rst may assert asynchronously: plain 2-FF synchroniser)
  (* ASYNC_REG = "TRUE" *) logic s_rst_s1_q = 1'b1, s_rst_s2_q = 1'b1;
  logic m_hold_q = 1'b1;
  logic m_rst_i;                              // read side held in reset
  always_ff @(posedge m_clk) begin
    s_rst_s1_q <= s_rst;
    s_rst_s2_q <= s_rst_s1_q;
    m_hold_q   <= m_rst | s_rst_s2_q;
  end
  assign m_rst_i = m_rst | s_rst_s2_q;

  assign empty    = (rd_gray_q == wr_gray_sync2_q);
  assign fetch    = ~empty && (~out_valid_q || m_tready);
  assign rd_bin_n = rd_bin_q + 1'b1;

  always_ff @(posedge m_clk) begin
    if (m_rst_i) begin
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
  assign m_tvalid = out_valid_q & ~m_rst_i & ~m_hold_q;

endmodule
