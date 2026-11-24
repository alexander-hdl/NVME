// =============================================================================
// xadc_mon — the single XADC of the XC7A200T, read out as raw 12-bit codes.
//
// Purpose : continuous-sequence XADC (on-chip temperature, VCCINT, VCCAUX,
//           VCCBRAM; averaging 16), read over DRP after each End-Of-Sequence.
//           MIG's own XADC is disabled; xadc.temp is the code for MIG's
//           device_temp_i (reaches the ui domain through X14, cdc_bus).
// Domain  : link_clk (200 MHz) = DCLK; ADCCLK = DCLK/8 = 25 MHz.
// Ports   : clk, rst (sync, active 1)                                  // link
//           xadc : xadc_t raw codes (temp/vccint/vccaux/vccbram[11:0], valid)
//                  valid = sticky, set after the first complete refresh
//           upd  : 1-clk pulse each time all four codes were refreshed
//           All outputs are registers.
// Format  : code = DRP result [15:4] (upper 12 bit; 16-bit result left-aligned).
//           T[C] = code*503.975/4096 - 273.15 ; U[V] = code*3/4096
//           (same as tools/recorder.py xadc_temp_c / xadc_volt).
// Spec    : RECORDER_SPEC.md 4.4 WP7, 10 (WP7), 9.1 tb_xadc_mon.
// =============================================================================
module xadc_mon
  import rec_pkg::*;
#(
  parameter SIM_MONITOR_FILE = "design.txt"   // analog stimulus (xsim only)
) (
  input  logic  clk,        // link
  input  logic  rst,        // link, sync
  output xadc_t xadc,       // link
  output logic  upd         // link, 1-clk pulse
);

  // DRP status registers (UG480): 00 temp, 01 VCCINT, 02 VCCAUX, 06 VCCBRAM
  localparam logic [6:0] A_TEMP = 7'h00, A_VINT = 7'h01, A_VAUX = 7'h02, A_VBRAM = 7'h06;

  logic        drdy, eos, den;
  logic [15:0] dout;
  logic [6:0]  daddr;

  // INIT_40: averaging 16 (bits 13:12 = 01).  INIT_41: SEQ = 0010 (continuous
  // sequence), calibration enabled (7:4), alarms disabled (11:8, 3:0).
  // INIT_42: CD = 8 -> ADCCLK = DCLK/8.  INIT_48: temp, VCCINT, VCCAUX, VCCBRAM
  // + calibration.  INIT_4A: averaging enabled for the same four (no calib.).
  XADC #(
    .INIT_40(16'h1000), .INIT_41(16'h2FFF), .INIT_42(16'h0800),
    .INIT_48(16'h4701), .INIT_49(16'h0000),
    .INIT_4A(16'h4700), .INIT_4B(16'h0000),
    .INIT_4C(16'h0000), .INIT_4D(16'h0000),
    .SIM_DEVICE("7SERIES"), .SIM_MONITOR_FILE(SIM_MONITOR_FILE)
  ) u_xadc (
    .DCLK(clk), .RESET(rst),
    .DADDR(daddr), .DEN(den), .DWE(1'b0), .DI(16'h0000), .DO(dout), .DRDY(drdy),
    .CONVST(1'b0), .CONVSTCLK(1'b0),
    .VP(1'b0), .VN(1'b0), .VAUXP(16'h0000), .VAUXN(16'h0000),
    .BUSY(), .EOC(), .EOS(eos), .JTAGBUSY(), .JTAGLOCKED(), .JTAGMODIFIED(),
    .OT(), .ALM(), .CHANNEL(), .MUXADDR()
  );

  typedef enum logic [1:0] {S_WAIT, S_REQ, S_RD, S_DONE} st_e;
  st_e         st_q;
  logic [1:0]  idx_q;
  logic [11:0] tmp_q [0:3];

  function automatic logic [6:0] addr_of(input logic [1:0] i);
    case (i)
      2'd0:    return A_TEMP;
      2'd1:    return A_VINT;
      2'd2:    return A_VAUX;
      default: return A_VBRAM;
    endcase
  endfunction

  always_ff @(posedge clk) begin
    den <= 1'b0;
    upd <= 1'b0;
    if (rst) begin
      st_q <= S_WAIT; idx_q <= '0; xadc <= '0; daddr <= '0;
    end else begin
      case (st_q)
        S_WAIT: begin                       // wait for End Of Sequence
          idx_q <= '0;
          if (eos) st_q <= S_REQ;
        end
        S_REQ: begin                        // DRP read request (1-clk DEN)
          daddr <= addr_of(idx_q);
          den   <= 1'b1;
          st_q  <= S_RD;
        end
        S_RD: if (drdy) begin
          tmp_q[idx_q] <= dout[15:4];
          idx_q        <= idx_q + 2'd1;
          st_q         <= (idx_q == 2'd3) ? S_DONE : S_REQ;
        end
        S_DONE: begin                       // publish all four codes together
          xadc.temp    <= tmp_q[0];
          xadc.vccint  <= tmp_q[1];
          xadc.vccaux  <= tmp_q[2];
          xadc.vccbram <= tmp_q[3];
          xadc.valid   <= 1'b1;
          upd          <= 1'b1;
          st_q         <= S_WAIT;
        end
        default: st_q <= S_WAIT;
      endcase
    end
  end

endmodule
