// =============================================================================
// tb_uart.sv — uart_tx / uart_rx (WP1): loopback at 921600 baud, 200 MHz clock.
//   * 4 KiB back-to-back at the nominal rate; 1 KiB each with the transmitter 1.8 % fast
//     and 1.8 % slow (DIV 213 / 221 instead of 217);
//   * bit timing of uart_tx measured on the pin (start, 8 data, stop = 217 clocks each);
//   * negative controls: bad stop bit -> frame_err, no byte, receiver recovers; start-bit glitch
//     ignored; break (line low) gives exactly one frame_err; a deliberately wrong expected
//     stream is detected by the loopback checker.
// =============================================================================
`timescale 1ns/1ps

// one tx -> rx pair with a checker; EXP_FLIP >= 0 makes the checker expect a wrong byte there
module uart_pair #(
  parameter int TX_CLK_HZ = 200_000_000,
  parameter int NBYTES    = 4096,
  parameter int EXP_FLIP  = -1
) (
  input  logic clk,
  input  logic rst,
  output logic done,
  output int   errors,
  output int   ferrs
);
  logic [7:0] s_data;
  logic       s_valid, s_ready, txd;
  logic [7:0] m_data;
  logic       m_valid, frame_err;
  int         sent = 0, got = 0;

  function automatic logic [7:0] pat(input int i);
    logic [31:0] x;
    x = 32'(i) * 32'h9E3779B1 + 32'h1234567;
    return x[23:16] ^ x[7:0] ^ 8'(i);
  endfunction

  uart_tx #(.CLK_HZ(TX_CLK_HZ), .BAUD(921_600)) u_tx (
    .clk(clk), .rst(rst), .s_data(s_data), .s_valid(s_valid), .s_ready(s_ready), .txd(txd));
  uart_rx #(.CLK_HZ(200_000_000), .BAUD(921_600)) u_rx (
    .clk(clk), .rst(rst), .rxd(txd), .m_data(m_data), .m_valid(m_valid), .frame_err(frame_err));

  assign s_data  = pat(sent);
  assign s_valid = !rst && (sent < NBYTES);

  always @(posedge clk) begin
    if (rst) begin
      sent <= 0; got <= 0; errors <= 0; ferrs <= 0; done <= 1'b0;
    end else begin
      if (s_valid && s_ready) sent <= sent + 1;
      if (frame_err) ferrs <= ferrs + 1;
      if (m_valid) begin
        logic [7:0] e;
        e = pat(got);
        if (got == EXP_FLIP) e = ~e;
        if (m_data !== e) errors <= errors + 1;
        got <= got + 1;
        if (got == NBYTES - 1) done <= 1'b1;
      end
    end
  end
endmodule

module tb_uart;
  logic clk = 1'b0;
  logic rst = 1'b1;
  always #2.5 clk = ~clk;      // 200 MHz

  localparam int DIV = 217;

  // ------------------------------------------------------------ loopback pairs
  logic d0, d1, d2, d3;
  int   e0, e1, e2, e3, f0, f1, f2, f3;
  uart_pair #(.TX_CLK_HZ(200_000_000),     .NBYTES(4096))                 p_nom  (clk, rst, d0, e0, f0);
  uart_pair #(.TX_CLK_HZ(213 * 921_600),   .NBYTES(1024))                 p_fast (clk, rst, d1, e1, f1);
  uart_pair #(.TX_CLK_HZ(221 * 921_600),   .NBYTES(1024))                 p_slow (clk, rst, d2, e2, f2);
  uart_pair #(.TX_CLK_HZ(200_000_000),     .NBYTES(64), .EXP_FLIP(10))    p_neg  (clk, rst, d3, e3, f3);

  // ------------------------------------------------------------ bit timing of uart_tx on the pin
  logic [7:0] t_data = 8'h00;
  logic       t_valid = 1'b0, t_ready, t_txd;
  uart_tx #(.CLK_HZ(200_000_000), .BAUD(921_600)) u_tx_t (
    .clk(clk), .rst(rst), .s_data(t_data), .s_valid(t_valid), .s_ready(t_ready), .txd(t_txd));

  // ------------------------------------------------------------ manual receiver stimulus
  logic       r_rxd = 1'b1;
  logic [7:0] r_data;
  logic       r_valid, r_ferr;
  uart_rx #(.CLK_HZ(200_000_000), .BAUD(921_600)) u_rx_m (
    .clk(clk), .rst(rst), .rxd(r_rxd), .m_data(r_data), .m_valid(r_valid), .frame_err(r_ferr));
  int r_nvalid = 0, r_nferr = 0;
  logic [7:0] r_last;
  always @(posedge clk) begin
    if (rst) begin r_nvalid <= 0; r_nferr <= 0; end
    else begin
      if (r_valid) begin r_nvalid <= r_nvalid + 1; r_last <= r_data; end
      if (r_ferr)  r_nferr <= r_nferr + 1;
    end
  end

  task automatic wait_clks(input int n);
    repeat (n) @(posedge clk);
    #1;
  endtask

  // drive one frame on r_rxd, DIV clocks per bit; stop_bit = 0 makes a framing error
  task automatic send_frame(input logic [7:0] b, input logic stop_bit);
    r_rxd = 1'b0;                    wait_clks(DIV);
    for (int i = 0; i < 8; i++) begin r_rxd = b[i]; wait_clks(DIV); end
    r_rxd = stop_bit;                wait_clks(DIV);
    r_rxd = 1'b1;
  endtask

  int errors = 0;
  task automatic check(input bit cond, input string msg);
    if (!cond) begin errors++; $display("ERROR: %s", msg); end
  endtask

  initial begin
    // --------------------------------------------------------- pin timing of uart_tx
    wait_clks(10);
    rst = 1'b0;
    wait_clks(5);
    check(t_txd === 1'b1, "tx idles high");
    begin
      logic [9:0] expect_bits;
      expect_bits = {1'b1, 8'h5A, 1'b0};           // stop, data (0x5A), start - LSB first on the wire
      t_data  = 8'h5A;
      t_valid = 1'b1;
      wait_clks(1);                                    // the byte is taken at this edge, start bit begins
      t_valid = 1'b0;
      for (int i = 0; i < 10; i++) begin
        // sample at the middle of bit i and check the next boundary exactly DIV clocks later
        wait_clks(DIV / 2);
        check(t_txd === expect_bits[i], $sformatf("tx bit %0d value", i));
        wait_clks(DIV - DIV / 2);
      end
      // exactly 10 bit times after the start edge the line must be idle high and tx ready again
      wait_clks(3);
      check(t_txd === 1'b1, "tx high after the frame");
      check(t_ready === 1'b1, "tx ready after the frame");
    end

    // --------------------------------------------------------- receiver corner cases
    // a good byte
    send_frame(8'hA7, 1'b1);
    wait_clks(20);
    check(r_nvalid == 1 && r_last == 8'hA7 && r_nferr == 0, "good byte");
    // negative control: bad stop bit -> frame_err, no byte
    send_frame(8'h3C, 1'b0);
    wait_clks(20);
    check(r_nferr == 1, "bad stop bit gives frame_err");
    check(r_nvalid == 1, "bad stop bit gives no byte");
    // the receiver recovers: next good frame
    wait_clks(5);
    send_frame(8'hC3, 1'b1);
    wait_clks(20);
    check(r_nvalid == 2 && r_last == 8'hC3 && r_nferr == 1, "recovery after frame error");
    // start-bit glitch (40 clocks low, shorter than half a bit): ignored, no byte, no error
    r_rxd = 1'b0; wait_clks(40); r_rxd = 1'b1;
    wait_clks(2 * DIV * 10);
    check(r_nvalid == 2 && r_nferr == 1, "start glitch ignored");
    // break: line low for 3 frame times -> exactly one frame_err, no bytes, then recovery
    r_rxd = 1'b0; wait_clks(3 * 10 * DIV); r_rxd = 1'b1;
    wait_clks(2 * DIV);
    check(r_nferr == 2 && r_nvalid == 2, "break gives one frame_err");
    send_frame(8'h81, 1'b1);
    wait_clks(20);
    check(r_nvalid == 3 && r_last == 8'h81, "recovery after break");
    // back-to-back frames with one stop bit and nothing in between
    send_frame(8'h11, 1'b1); send_frame(8'h22, 1'b1); send_frame(8'h33, 1'b1);
    wait_clks(20);
    check(r_nvalid == 6 && r_last == 8'h33, "three frames back to back");

    // --------------------------------------------------------- loopbacks
    wait (d0 && d1 && d2 && d3 || 1'b0);
    wait_clks(3000);
    check(e0 == 0 && f0 == 0, $sformatf("nominal 4 KiB: errors %0d ferr %0d", e0, f0));
    check(e1 == 0 && f1 == 0, $sformatf("tx 1.8%% fast: errors %0d ferr %0d", e1, f1));
    check(e2 == 0 && f2 == 0, $sformatf("tx 1.8%% slow: errors %0d ferr %0d", e2, f2));
    // negative control: the checker with a wrong expected byte must see exactly one error
    check(d3 && e3 == 1, $sformatf("deliberate wrong expectation detected exactly once: %0d", e3));

    if (errors != 0) $fatal(1, "tb_uart: %0d errors", errors);
    $display("TEST PASSED");
    $finish;
  end

  initial begin
    #200ms;
    $fatal(1, "tb_uart: timeout");
  end
endmodule
