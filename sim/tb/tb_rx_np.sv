// =============================================================================
// tb_rx_np.sv - rx_np_ok accounting of pcie7x_rx_adapt (review m5)
//
// rx_np_ok must drop when the number of non-posted requests queued END TO END reaches
// HDR_FIFO_DEPTH + NP_EXT_DEPTH - 2 = 22:  MRd headers inside the adapter's header FIFO
// (counted here) + np_ext_count (the completer's request FIFO occupancy, driven by the tb).
// The sink is stalled, so every MRd stays in the header FIFO.
//
//   A  np_ext_count = 16 : rx_np_ok high for 1..5 queued MRd, low from the 6th on, high again after the drain
//   B  np_ext_count = 14 : low only when the 8th (FIFO full) is queued
//   C  np_ext_count = 0  : stays high even with 8 queued
//   D  np_backpressure   : forces rx_np_ok low at once (completer request FIFO almost full)
//   E  Posted/completion headers are NOT counted: 8 MWr headers queued -> rx_np_ok stays high with np_ext_count = 16
// =============================================================================
`timescale 1ns/1ps
module tb_rx_np;
  import nvme_host_pkg::*;

  logic clk = 1'b0, rst = 1'b1;
  always #4 clk = ~clk;

  logic [127:0] m_axis_rx_tdata = '0;  logic [15:0] m_axis_rx_tkeep = 16'hFFFF;  logic m_axis_rx_tlast = 1'b0;
  logic         m_axis_rx_tvalid = 1'b0, m_axis_rx_tready;  logic [21:0] m_axis_rx_tuser = '0;
  logic         rx_np_ok, np_backpressure = 1'b0;
  logic [4:0]   np_ext_count = 5'd0;
  logic [127:0] m_tlp_tdata;  logic [15:0] m_tlp_tkeep;  logic m_tlp_tlast;  logic [1:0] m_tlp_tuser;
  logic         m_tlp_tvalid, m_tlp_tready = 1'b0;

  pcie7x_rx_adapt dut (.*);

  int errors = 0;
  task automatic fail(input string m);
    errors++;
    $display("[%0t] ERROR: %s", $time, m);
  endtask

  // header-only 3DW TLP at DW0: SOF (tuser[14]), EOF (tuser[21]) at DW2 (tuser[20:19] = 2)
  task automatic send_hdr(input logic [7:0] ft);
    logic [31:0] h0;
    h0 = {ft, 1'b0, 3'b000, 4'b0000, 1'b0, 1'b0, 2'b00, 2'b00, 10'd0};
    h0[30] = ft[6];
    @(posedge clk);
    m_axis_rx_tdata  <= {32'hDEAD_BEEF, 32'h0000_0100, 32'h0001_00FF, h0};
    if (ft[6]) m_axis_rx_tdata[9:0] <= 10'd1;                 // MWr: Length = 1 DW (payload follows in lane 3)
    else       m_axis_rx_tdata[9:0] <= 10'd1;
    m_axis_rx_tuser  <= 22'h0;
    m_axis_rx_tuser[14]    <= 1'b1;                           // SOF at DW0
    m_axis_rx_tuser[21]    <= !ft[6];                         // EOF only for header-only TLPs
    m_axis_rx_tuser[20:19] <= 2'd2;
    m_axis_rx_tvalid <= 1'b1;
    @(posedge clk);
    while (!m_axis_rx_tready) @(posedge clk);
    m_axis_rx_tvalid <= 1'b0;
    if (ft[6]) begin                                          // MWr: second beat with EOF (not needed for the count)
      @(posedge clk);
      m_axis_rx_tdata  <= 128'h1234;
      m_axis_rx_tuser  <= 22'h0;
      m_axis_rx_tuser[21]    <= 1'b1;
      m_axis_rx_tuser[20:19] <= 2'd0;
      m_axis_rx_tvalid <= 1'b1;
      @(posedge clk);
      while (!m_axis_rx_tready) @(posedge clk);
      m_axis_rx_tvalid <= 1'b0;
    end
  endtask

  task automatic drain();
    m_tlp_tready = 1'b1;
    repeat (80) @(posedge clk);
    m_tlp_tready = 1'b0;
    repeat (4) @(posedge clk);
  endtask

  initial begin
    repeat (5) @(posedge clk);
    rst <= 1'b0;
    repeat (5) @(posedge clk);
    if (rx_np_ok !== 1'b1) fail("rx_np_ok must be high when idle");

    // ---- A ----
    np_ext_count = 5'd16;
    for (int k = 1; k <= 8; k++) begin
      send_hdr(FT_MRD32);
      repeat (4) @(posedge clk);
      if (rx_np_ok !== (k < 6)) fail($sformatf("A: %0d MRd queued, rx_np_ok = %0b, expected %0b", k, rx_np_ok, k < 6));
    end
    drain();
    if (rx_np_ok !== 1'b1) fail("A: rx_np_ok must recover after the drain");

    // ---- B ----
    np_ext_count = 5'd14;
    for (int k = 1; k <= 8; k++) begin
      send_hdr(FT_MRD32);
      repeat (4) @(posedge clk);
      if (rx_np_ok !== (k < 8)) fail($sformatf("B: %0d MRd queued (+14), rx_np_ok = %0b, expected %0b", k, rx_np_ok, k < 8));
    end
    drain();

    // ---- C ----
    np_ext_count = 5'd0;
    for (int k = 1; k <= 8; k++) begin
      send_hdr(FT_MRD32);
      repeat (4) @(posedge clk);
      if (rx_np_ok !== 1'b1) fail($sformatf("C: %0d MRd queued, rx_np_ok low with an empty completer FIFO", k));
    end
    drain();

    // ---- D ----
    np_backpressure = 1'b1;
    repeat (4) @(posedge clk);
    if (rx_np_ok !== 1'b0) fail("D: np_backpressure must force rx_np_ok low");
    np_backpressure = 1'b0;
    repeat (4) @(posedge clk);
    if (rx_np_ok !== 1'b1) fail("D: rx_np_ok must recover");

    // ---- E: posted headers are not counted ----
    np_ext_count = 5'd21;                        // one more counted request reaches the limit of 22
    send_hdr(FT_MWR32);                          // queued behind the stalled sink
    repeat (4) @(posedge clk);
    if (rx_np_ok !== 1'b1) fail("E: a queued MWr header must not count as non-posted");
    send_hdr(FT_MRD32);
    repeat (4) @(posedge clk);
    if (rx_np_ok !== 1'b0) fail("E: one MRd on top of 21 queued in the completer must drop rx_np_ok");
    drain();
    np_ext_count = 5'd0;
    repeat (4) @(posedge clk);
    if (rx_np_ok !== 1'b1) fail("E: rx_np_ok must recover after the drain");

    if (errors == 0) $display("TEST PASSED");
    else $fatal(1, "TEST FAILED: %0d errors", errors);
    $finish;
  end

  initial begin
    #2_000_000;
    $fatal(1, "TIMEOUT");
  end
endmodule
