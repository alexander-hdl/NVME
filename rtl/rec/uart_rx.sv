// =============================================================================
// uart_rx.sv — UART receiver, 8N1 (WP1).
//
// Domain  : link_clk (200 MHz).  Spec: STAND/RECORDER_SPEC.md 2.1, 4.4 (WP1).
// Ports   : rxd        asynchronous pin, synchronised here by 2 FF (X12)
//           m_data     received byte, valid with m_valid (stays until the next byte)
//           m_valid    1-clock pulse after a correct stop bit
//           frame_err  1-clock pulse when the stop bit is 0 (the byte is dropped, no m_valid);
//                      after that the receiver waits for the line to go high again
// Timing  : DIV = round(CLK_HZ / BAUD) = 217 clocks per bit.  The start bit is verified in
//           its middle (a glitch shorter than half a bit is ignored), data bits and the stop
//           bit are sampled in the middle of the bit.  Tolerates about +-3 % baud error.
// =============================================================================
module uart_rx #(
  parameter int CLK_HZ = 200_000_000,
  parameter int BAUD   = 921_600
) (
  input  logic       clk,
  input  logic       rst,
  input  logic       rxd,         // async pin
  output logic [7:0] m_data,      // link
  output logic       m_valid,     // link, 1 clock
  output logic       frame_err    // link, 1 clock
);
  localparam int DIV  = (CLK_HZ + BAUD / 2) / BAUD;
  localparam int HALF = DIV / 2;
  localparam int CW   = $clog2(DIV + 1);

  (* ASYNC_REG = "TRUE" *) logic rxd_cdc_s0_q;
  (* ASYNC_REG = "TRUE" *) logic rxd_s1_q;

  typedef enum logic [2:0] {S_IDLE, S_START, S_DATA, S_STOP, S_WAITHI} state_e;
  state_e         state_q;
  logic [CW-1:0]  cnt_q;
  logic [2:0]     bitn_q;
  logic [7:0]     sh_q;

  always_ff @(posedge clk) begin
    rxd_cdc_s0_q <= rxd;
    rxd_s1_q     <= rxd_cdc_s0_q;
  end

  always_ff @(posedge clk) begin
    m_valid   <= 1'b0;
    frame_err <= 1'b0;
    if (rst) begin
      state_q <= S_IDLE;
      cnt_q   <= '0;
      bitn_q  <= '0;
    end else begin
      case (state_q)
        S_IDLE: begin
          if (!rxd_s1_q) begin
            cnt_q   <= CW'(HALF - 1);
            state_q <= S_START;
          end
        end
        S_START: begin
          if (cnt_q == '0) begin
            if (!rxd_s1_q) begin          // real start bit
              cnt_q   <= CW'(DIV - 1);
              bitn_q  <= 3'd0;
              state_q <= S_DATA;
            end else begin
              state_q <= S_IDLE;          // glitch
            end
          end else begin
            cnt_q <= cnt_q - 1'b1;
          end
        end
        S_DATA: begin
          if (cnt_q == '0) begin
            sh_q  <= {rxd_s1_q, sh_q[7:1]};
            cnt_q <= CW'(DIV - 1);
            if (bitn_q == 3'd7) state_q <= S_STOP;
            bitn_q <= bitn_q + 1'b1;
          end else begin
            cnt_q <= cnt_q - 1'b1;
          end
        end
        S_STOP: begin
          if (cnt_q == '0) begin
            if (rxd_s1_q) begin
              m_data  <= sh_q;
              m_valid <= 1'b1;
              state_q <= S_IDLE;
            end else begin
              frame_err <= 1'b1;
              state_q   <= S_WAITHI;
            end
          end else begin
            cnt_q <= cnt_q - 1'b1;
          end
        end
        S_WAITHI: begin
          if (rxd_s1_q) state_q <= S_IDLE;
        end
        default: state_q <= S_IDLE;
      endcase
    end
  end
endmodule
