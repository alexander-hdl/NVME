// =============================================================================
// uart_tx.sv — UART transmitter, 8N1 (WP1).
//
// Domain  : link_clk (200 MHz).  Spec: STAND/RECORDER_SPEC.md 2.1, 4.4 (WP1).
// Ports   : s_data/s_valid/s_ready  stream input; a byte is taken when s_valid && s_ready
//                                    (s_ready is a register, it drops the clock after the take)
//           txd                     serial output, register, idles high
// Timing  : DIV = round(CLK_HZ / BAUD) = 217 clocks per bit; s_ready rises at the end of the
//           stop bit, so back-to-back bytes have no gap except one clock.
// =============================================================================
module uart_tx #(
  parameter int CLK_HZ = 200_000_000,
  parameter int BAUD   = 921_600
) (
  input  logic       clk,
  input  logic       rst,
  input  logic [7:0] s_data,    // link
  input  logic       s_valid,   // link
  output logic       s_ready,   // link
  output logic       txd        // pin
);
  localparam int DIV = (CLK_HZ + BAUD / 2) / BAUD;
  localparam int CW  = $clog2(DIV + 1);

  logic          busy_q;
  logic [CW-1:0] cnt_q;
  logic [3:0]    bitn_q;        // 0 start, 1..8 data, 9 stop
  logic [8:0]    sh_q;          // data bits, then ones

  always_ff @(posedge clk) begin
    if (rst) begin
      busy_q  <= 1'b0;
      s_ready <= 1'b0;
      txd     <= 1'b1;
      cnt_q   <= '0;
      bitn_q  <= '0;
    end else if (!busy_q) begin
      if (s_valid && s_ready) begin
        sh_q    <= {1'b1, s_data};
        txd     <= 1'b0;                 // start bit
        busy_q  <= 1'b1;
        s_ready <= 1'b0;
        cnt_q   <= CW'(DIV - 1);
        bitn_q  <= 4'd0;
      end else begin
        s_ready <= 1'b1;
      end
    end else begin
      if (cnt_q == '0) begin
        cnt_q <= CW'(DIV - 1);
        if (bitn_q == 4'd9) begin        // stop bit finished
          busy_q  <= 1'b0;
          s_ready <= 1'b1;
        end else begin
          txd     <= sh_q[0];
          sh_q    <= {1'b1, sh_q[8:1]};
          bitn_q  <= bitn_q + 1'b1;
        end
      end else begin
        cnt_q <= cnt_q - 1'b1;
      end
    end
  end
endmodule
