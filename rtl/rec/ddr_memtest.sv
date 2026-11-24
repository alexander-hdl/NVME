// =============================================================================
// ddr_memtest.sv - DDR3 memory test over the MIG native app interface.
//
// Domain : ui_clk (single clock domain, `rst` synchronous active 1).
// Spec   : STAND/RECORDER_SPEC.md 4.4 (WP4 DDR), 5.3, 9.1.  Instantiated inside
//          ddr_fifo, which multiplexes the MIG app port between the FIFO engine
//          and this module (select = `busy`).
//
// Test   : two passes over the first `mib` MiB of the DDR, every pass is
//          "write everything, then read everything back and compare":
//            pass 0: data = address as data (lane i of the 256-bit word = {3'b0, addr[28:3], i});
//            pass 1: data = eight independent 32-bit Galois LFSRs (one step per word).
//          One 256-bit word = one BL8 command; address step 8 per command.
//          err_cnt counts mismatching 256-bit words (both passes together);
//          first_err_addr is the app_addr of the first mismatching word.
//
// Ports  : start      - 1-cycle pulse, ignored while busy or when !init_calib_complete;
//                       mib is sampled at the start pulse (0 is treated as 1, max 1024).
//          abort      - 1-cycle pulse: stop issuing commands, wait for the read responses still in
//                       flight (dropped), then busy=0; done stays 0, err_cnt/first_err_addr are kept.
//          busy       - level, from the start pulse to done.
//          done       - sticky, set at the end of the test, cleared by the next start.
//          err_cnt, first_err_addr - valid when done (cleared by start).
//          app_*      - MIG native interface (outputs combinationally gated by app_rdy /
//                       app_wdf_rdy: a command and its data are issued only in a cycle in
//                       which both are accepted, so the "hold until ready" rule holds trivially).
// =============================================================================
module ddr_memtest (
  input  logic          clk,
  input  logic          rst,
  input  logic          start,                    // ui: 1-cycle pulse
  input  logic          abort,                    // ui: 1-cycle pulse, stops the test (see header)
  input  logic [10:0]   mib,                      // ui: size of the tested area, MiB
  input  logic          init_calib_complete,      // ui
  output logic          busy,                     // ui
  output logic          done,                     // ui: sticky
  output logic [31:0]   err_cnt,                  // ui
  output logic [28:0]   first_err_addr,           // ui
  // MIG native interface (ui)
  output logic [28:0]   app_addr,
  output logic [2:0]    app_cmd,
  output logic          app_en,
  input  logic          app_rdy,
  output logic [255:0]  app_wdf_data,
  output logic          app_wdf_wren,
  output logic          app_wdf_end,
  input  logic          app_wdf_rdy,
  input  logic [255:0]  app_rd_data,
  input  logic          app_rd_data_valid
);

  typedef enum logic [2:0] {S_IDLE, S_WR, S_RD, S_DONE, S_ABORT} state_e;

  localparam logic [31:0] LFSR_POLY = 32'h8020_0003;      // x^32+x^22+x^2+x+1 (Galois, shift right)
  localparam logic [31:0] LFSR_SEED = 32'h1BAD_5EED;      // lane i starts at SEED + i * LANE_STEP
  localparam logic [31:0] LANE_STEP = 32'h9E37_79B9;

  state_e        st_q;
  logic          pass_q;              // 0: address-as-data, 1: LFSR
  logic [25:0]   ncmd_q;              // commands per phase = mib * 32768
  logic [25:0]   wleft_q;             // commands still to issue (write or read phase)
  logic [28:0]   waddr_q;             // address of the next command
  logic [25:0]   rleft_q;             // responses still expected (read phase)
  logic [28:0]   raddr_q;             // address of the next expected response
  logic [31:0]   wlf_q [8];           // LFSR lanes, write/issue side
  logic [31:0]   rlf_q [8];           // LFSR lanes, response side
  logic          have_err_q;
  logic [7:0]    infl_q;              // read commands accepted, response not yet received

  function automatic logic [31:0] lfsr_step(input logic [31:0] s);
    return (s >> 1) ^ (s[0] ? LFSR_POLY : 32'h0);
  endfunction

  function automatic logic [255:0] addr_word(input logic [28:0] a);
    logic [255:0] w;
    for (int i = 0; i < 8; i++) w[32*i +: 32] = {3'b000, a[28:3], 3'(i)};
    return w;
  endfunction

  logic [255:0] wdata, rexp;
  always_comb begin
    for (int i = 0; i < 8; i++) begin
      wdata[32*i +: 32] = wlf_q[i];
      rexp [32*i +: 32] = rlf_q[i];
    end
    if (!pass_q) begin
      wdata = addr_word(waddr_q);
      rexp  = addr_word(raddr_q);
    end
  end

  logic wr_ph, rd_ph, acc_w, acc_r;
  assign wr_ph = (st_q == S_WR);
  assign rd_ph = (st_q == S_RD);
  assign acc_w = wr_ph && !rst && !abort && (wleft_q != 26'd0) && app_rdy && app_wdf_rdy;
  assign acc_r = rd_ph && !rst && !abort && (wleft_q != 26'd0) && app_rdy;

  assign app_en       = acc_w | acc_r;
  assign app_cmd      = acc_r ? 3'b001 : 3'b000;
  assign app_addr     = waddr_q;
  assign app_wdf_data = wdata;
  assign app_wdf_wren = acc_w;
  assign app_wdf_end  = acc_w;

  logic        fin_q;
  assign busy = ((st_q != S_IDLE) && (st_q != S_DONE)) || fin_q;

  logic resp, mism;
  assign resp = rd_ph && !abort && app_rd_data_valid;
  logic any_resp;
  assign any_resp = (rd_ph || st_q == S_ABORT) && app_rd_data_valid;
  // timing: app_rd_data is compared per 32-bit lane into registers, the error counter works one clock later
  logic [7:0]  mismw_q;
  logic        resp_q;
  logic [28:0] eaddr_q;
  always_ff @(posedge clk) begin
    for (int i = 0; i < 8; i++) mismw_q[i] <= (app_rd_data[32*i +: 32] != rexp[32*i +: 32]);
    eaddr_q <= raddr_q;
    resp_q  <= resp && !rst;
  end
  assign mism = |mismw_q;               // valid with resp_q

  always_ff @(posedge clk) begin
    if (rst) begin
      st_q           <= S_IDLE;
      pass_q         <= 1'b0;
      done           <= 1'b0;
      fin_q          <= 1'b0;
      err_cnt        <= 32'd0;
      first_err_addr <= 29'd0;
      have_err_q     <= 1'b0;
      ncmd_q         <= 26'd0;
      wleft_q        <= 26'd0;
      rleft_q        <= 26'd0;
      waddr_q        <= 29'd0;
      raddr_q        <= 29'd0;
      infl_q         <= 8'd0;
    end else begin
      infl_q <= infl_q + 8'(acc_r) - 8'(any_resp);
      if (fin_q) begin fin_q <= 1'b0; done <= 1'b1; end
      if (abort && (st_q == S_WR || st_q == S_RD)) st_q <= S_ABORT;
      else case (st_q)
        S_ABORT: if (infl_q == 8'd0) st_q <= S_IDLE;

        S_IDLE, S_DONE: begin
          if (start && init_calib_complete && !fin_q) begin
            st_q           <= S_WR;
            pass_q         <= 1'b0;
            done           <= 1'b0;
            err_cnt        <= 32'd0;
            first_err_addr <= 29'd0;
            have_err_q     <= 1'b0;
            ncmd_q         <= {((mib == 11'd0) ? 11'd1 : mib), 15'd0};
            wleft_q        <= {((mib == 11'd0) ? 11'd1 : mib), 15'd0};
            rleft_q        <= {((mib == 11'd0) ? 11'd1 : mib), 15'd0};
            waddr_q        <= 29'd0;
            raddr_q        <= 29'd0;
            for (int i = 0; i < 8; i++) begin
              wlf_q[i] <= LFSR_SEED + LANE_STEP * 32'(i);
              rlf_q[i] <= LFSR_SEED + LANE_STEP * 32'(i);
            end
          end
        end

        S_WR: begin
          if (acc_w) begin
            waddr_q <= waddr_q + 29'd8;
            wleft_q <= wleft_q - 26'd1;
            for (int i = 0; i < 8; i++) wlf_q[i] <= lfsr_step(wlf_q[i]);
            if (wleft_q == 26'd1) begin              // phase done: rewind for the read-back
              st_q    <= S_RD;
              waddr_q <= 29'd0;
              wleft_q <= ncmd_q;
              rleft_q <= ncmd_q;
              raddr_q <= 29'd0;
              for (int i = 0; i < 8; i++) begin
                wlf_q[i] <= LFSR_SEED + LANE_STEP * 32'(i);
                rlf_q[i] <= LFSR_SEED + LANE_STEP * 32'(i);
              end
            end
          end
        end

        S_RD: begin
          if (acc_r) begin
            waddr_q <= waddr_q + 29'd8;
            wleft_q <= wleft_q - 26'd1;
          end
          if (resp) begin
            raddr_q <= raddr_q + 29'd8;
            rleft_q <= rleft_q - 26'd1;
            for (int i = 0; i < 8; i++) rlf_q[i] <= lfsr_step(rlf_q[i]);
            if (rleft_q == 26'd1) begin
              if (!pass_q) begin                      // next: LFSR pass, write phase
                st_q    <= S_WR;
                pass_q  <= 1'b1;
                waddr_q <= 29'd0;
                wleft_q <= ncmd_q;
                for (int i = 0; i < 8; i++) wlf_q[i] <= LFSR_SEED + LANE_STEP * 32'(i);
              end else begin
                st_q  <= S_DONE;
                fin_q <= 1'b1;                        // done follows once the last comparison is counted
              end
            end
          end
        end

        default: st_q <= S_IDLE;
      endcase
      if (resp_q && mism) begin
        err_cnt <= err_cnt + 32'd1;
        if (!have_err_q) begin
          have_err_q     <= 1'b1;
          first_err_addr <= eaddr_q;
        end
      end
    end
  end

endmodule
