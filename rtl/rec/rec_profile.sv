// =============================================================================
// rec_profile.sv - rate profile engine: up to 64 segments {step, dur}, repeat, live rate (WP3).
//
// Domain  : ui_clk (100 MHz).
// Ports   : spec 4.4 "WP3 / rec_profile" letter for letter, plus one extra output
//           prof_end_s (sticky level of prof_end, held until clear / rst: ui_stat_t.prof_end_s).
//           rd_en/rd_addr/rd_data = table port B (tdp_ram_2clk, latency 2): entry =
//           {step[31:0], dur[39:0]}.
// Behaviour:
//   * const mode (prof_en=0): step = const_step, loaded at start and on every rate_upd.
//   * profile mode: segment k drives `step` = step_k for exactly dur_k clocks (dur 0 is treated
//     as 1).  The first clock with step_0 is the clock AFTER the one in which start is sampled.
//     Segment k+1 follows without a gap.  seg_idx/rep_cnt are those of the current segment
//     (rep_cnt is the 0-based pass number).  After the last segment of the last pass (rep
//     passes, rep=0 -> endless): end_stop=1 -> step=0, prof_active=0 and a 1-clock prof_end
//     (in the first clock after the last segment, together with step=0); end_stop=0 (HOLD) -> step_last is held, no pulse.
//   * nseg = 0 (or > 64): the engine does not start (step stays 0, prof_active 0).
//   * stop freezes the profile (prof_active=0, step unchanged).
//   * The table is read ahead into an 8-deep FIFO that is (re)filled from entry 0 after `clear`
//     and whenever nseg/rep change, so 1-clock segments run at full rate.  The table must be
//     static from clear until the end of the session (spec 4.3 X3).  If start comes before the
//     FIFO holds entry 0 the profile begins as soon as it arrives (a few clocks late).
// Spec    : STAND/RECORDER_SPEC.md 4.4 (WP3), 2.3 (PROF), 3.1.
// =============================================================================
module rec_profile (
  input  logic         clk,          // ui
  input  logic         rst,          // ui
  input  logic         clear,        // ui, ui_clear
  input  logic         start,        // ui, src_start
  input  logic         stop,         // ui
  input  logic         prof_en,      // ui, valid in the clock of start
  input  logic [6:0]   nseg,         // ui, 1..64
  input  logic [15:0]  rep,          // ui, 0 = endless
  input  logic         end_stop,     // ui, 0 HOLD, 1 STOP
  input  logic [31:0]  const_step,   // ui, X2a data
  input  logic         rate_upd,     // ui, X2a dst_upd
  output logic         rd_en,        // ui
  output logic [5:0]   rd_addr,      // ui
  input  logic [71:0]  rd_data,      // ui, 2 clocks after rd_en
  output logic [31:0]  step,         // ui
  output logic [5:0]   seg_idx,      // ui
  output logic [15:0]  rep_cnt,      // ui
  output logic         prof_active,  // ui
  output logic         prof_end,     // ui, 1-clock pulse
  output logic         prof_end_s    // ui, sticky until clear
);

  localparam int FD = 8;                 // read-ahead FIFO depth
  localparam int PW = 3;                 // log2(FD)

  logic       nseg_ok;
  logic [6:0] nseg_m1;
  assign nseg_ok = (nseg != 7'd0) && (nseg <= 7'd64);
  assign nseg_m1 = nseg - 7'd1;

  // ---------------------------------------------------------------- read-ahead (fetch) side
  logic [71:0]   fifo_q [FD];
  logic [PW-1:0] wp_q, rp_q;
  logic [PW:0]   cnt_q;
  logic          rd_en_q, v1_q, v2_q;
  logic [5:0]    f_idx_q;
  logic [15:0]   f_pass_q;
  logic          f_done_q;
  logic [6:0]    nseg_l_q;
  logic [15:0]   rep_l_q;

  logic          flush;
  logic          pop;
  logic [71:0]   head;
  logic          issue;
  logic [4:0]    resv;
  logic          f_last_seg;

  assign head       = fifo_q[rp_q];
  assign resv       = {1'b0, cnt_q} + {4'b0, rd_en_q} + {4'b0, v1_q} + {4'b0, v2_q};
  assign flush      = rst || clear || (nseg != nseg_l_q) || (rep != rep_l_q);
  assign issue      = !flush && !f_done_q && nseg_ok && (resv < 5'(FD));
  assign f_last_seg = ({1'b0, f_idx_q} == nseg_m1);
  assign rd_en      = rd_en_q;

  always_ff @(posedge clk) begin
    nseg_l_q <= nseg;
    rep_l_q  <= rep;
    if (issue) rd_addr <= f_idx_q;                  // data path, valid with rd_en
    if (flush) begin
      rd_en_q  <= 1'b0;
      v1_q     <= 1'b0;
      v2_q     <= 1'b0;
      f_idx_q  <= 6'd0;
      f_pass_q <= 16'd0;
      f_done_q <= 1'b0;
      wp_q     <= '0;
      rp_q     <= '0;
      cnt_q    <= '0;
    end else begin
      rd_en_q <= issue;
      v1_q    <= rd_en_q;
      v2_q    <= v1_q;
      if (issue) begin
        if (f_last_seg) begin
          f_idx_q  <= 6'd0;
          f_pass_q <= f_pass_q + 16'd1;
          if (rep != 16'd0 && (f_pass_q + 16'd1) == rep) f_done_q <= 1'b1;
        end else begin
          f_idx_q <= f_idx_q + 6'd1;
        end
      end
      if (v2_q) begin
        fifo_q[wp_q] <= rd_data;
        wp_q         <= wp_q + 1'b1;
      end
      if (pop) rp_q <= rp_q + 1'b1;
      cnt_q <= cnt_q + {{PW{1'b0}}, v2_q} - {{PW{1'b0}}, pop};
    end
  end

  // ---------------------------------------------------------------- consumer side
  logic [39:0]  rem_q;
  logic [31:0]  step_q;
  logic [5:0]   seg_q;
  logic [15:0]  rep_q;
  logic         act_q;        // profile running (or holding)
  logic         hold_q;       // HOLD after the last segment
  logic         prof_q;       // profile mode latched at start
  logic         pend_q;       // start seen, waiting for entry 0
  logic         end_q;        // ended with STOP

  logic         have;         // FIFO holds an entry
  logic         start_ok;
  logic         begin_now;
  logic         seg_end;
  logic         last_seg, last_pass;
  logic [39:0]  head_dur;

  assign have      = (cnt_q != '0);
  assign head_dur  = (head[39:0] == 40'd0) ? 40'd1 : head[39:0];
  assign start_ok  = start && !act_q && !end_q && prof_en && nseg_ok;
  assign begin_now = (start_ok || pend_q) && have;
  assign seg_end   = act_q && !hold_q && (rem_q == 40'd1);
  assign last_seg  = ({1'b0, seg_q} == nseg_m1);
  assign last_pass = (rep != 16'd0) && ((rep_q + 16'd1) == rep);
  assign pop       = begin_now || (seg_end && !(last_seg && last_pass) && have && !stop);

  assign step        = step_q;
  assign seg_idx     = seg_q;
  assign rep_cnt     = rep_q;
  assign prof_active = act_q;

  always_ff @(posedge clk) begin
    prof_end <= 1'b0;
    if (rst || clear) begin
      step_q     <= 32'd0;
      seg_q      <= 6'd0;
      rep_q      <= 16'd0;
      act_q      <= 1'b0;
      hold_q     <= 1'b0;
      prof_q     <= 1'b0;
      pend_q     <= 1'b0;
      end_q      <= 1'b0;
      rem_q      <= 40'd0;
      prof_end_s <= 1'b0;
    end else begin
      if (start && !act_q && !end_q) begin
        prof_q <= prof_en;
        if (!prof_en) step_q <= const_step;
      end
      // const mode: live rate change
      if (!prof_q && rate_upd) step_q <= const_step;

      // start of the profile
      if (start_ok && !have) pend_q <= 1'b1;
      if (begin_now) begin
        step_q <= head[71:40];
        rem_q  <= head_dur;
        seg_q  <= 6'd0;
        rep_q  <= 16'd0;
        act_q  <= 1'b1;
        pend_q <= 1'b0;
      end

      // running profile
      if (act_q) begin
        if (stop) begin
          act_q <= 1'b0;
        end else if (!hold_q) begin
          rem_q <= rem_q - 40'd1;
          if (seg_end) begin
            if (last_seg && last_pass) begin
              if (end_stop) begin
                step_q     <= 32'd0;
                act_q      <= 1'b0;
                end_q      <= 1'b1;
                prof_end   <= 1'b1;
                prof_end_s <= 1'b1;
              end else begin
                hold_q <= 1'b1;
              end
            end else if (have) begin
              step_q <= head[71:40];
              rem_q  <= head_dur;
              if (last_seg) begin
                seg_q <= 6'd0;
                rep_q <= rep_q + 16'd1;
              end else begin
                seg_q <= seg_q + 6'd1;
              end
            end else begin
              rem_q <= rem_q;      // FIFO underrun: stretch the segment (must not happen)
            end
          end
        end
      end
    end
  end

endmodule
