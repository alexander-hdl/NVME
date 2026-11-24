// =============================================================================
// rec_framer.sv - NCO block source: 4 KiB blocks with header, pattern payload, CRC-32 trailer,
//                 drop (RT) / wait (BP), TRUNC / PAD / LAST end of session (WP3).
//
// Domain  : ui_clk (100 MHz), one clock, AXIS128 master m_*.
// Ports   : spec 4.4 "WP3 / rec_framer" letter for letter, plus one extra output len_reached_s
//           (sticky level of len_reached, held until clear / rst: ui_stat_t.len_reached_s).
// Principle (see INTEGRATION_NOTES.md WP3 for the decisions):
//   * NCO: acc += step every clock (acc starts at 0xFFFFFFFF at start, so the first beat is due
//     in the first clock for any step >= 1); a carry = one 16-byte beat is "due".
//     256 carries = one block.  The beat-0 carry fixes the block header (seq, ts = clocks from
//     start to that carry, seg/rep, ddr_kib, lost, flags).
//   * Blocks are emitted as a burst when they are COMPLETE (256 carries consumed): the header
//     flags (TRUNC, valid, LAST, PF) are thereby always correct.  There is no data buffer: the
//     payload is a function of pos and the beat number.
//   * Admission at beat 0 of a block: needs space_ok and a free "waiting" slot.  RT: otherwise
//     the whole block is dropped (256 carries swallowed, seq++, lost++).  BP: the NCO waits.
//     Inside an accepted block m_tready=0 never loses a beat (the block waits in its descriptor).
//   * stop / pf / len_blocks reached (len_reached pulse): the NCO stops; the block in progress
//     is closed as TRUNC (valid = payload beats already due), or an empty TRUNC (valid 0) is
//     created if there is none; then PAD blocks up to a multiple of 32 blocks; the last one has
//     LAST.  After pf the TRUNC / PAD / LAST blocks (whose header is not yet sent) carry PF.
//     Every block, PAD and TRUNC included, increments seq and pos.
//   * active = NCO running (falls one clock after stop/pf/len); done = everything incl. PAD has been accepted by the sink.
//   * One session per clear: start is ignored until clear after done.
// Metrics : src_blocks = seq counter (incl. dropped, PAD, TRUNC); wr_blocks = blocks completely
//           accepted by m_tready; bp_max_beats / rt_stall_max = maximum backlog, in beats, of a
//           completed block that waits behind the block being sent (BP / RT mode; 0 in normal
//           operation).
// Spec    : STAND/RECORDER_SPEC.md 3.1, 3.2, 4.4 (WP3), M11, M13.
// =============================================================================
module rec_framer (
  input  logic         clk,            // ui
  input  logic         rst,            // ui
  input  logic         clear,          // ui, ui_clear
  input  logic         start,          // ui, src_start
  input  logic         stop,           // ui
  input  logic         pf,             // ui
  input  logic         bp_mode,        // ui, sampled at start
  input  logic         prof_en,        // ui, sampled at start (flag PROF)
  input  logic [31:0]  sid,            // ui, sampled at start
  input  logic [39:0]  len_blocks,     // ui, sampled at start, 0 = no limit
  input  logic [31:0]  step,           // ui, NCO step
  input  logic [5:0]   seg_idx,        // ui
  input  logic [15:0]  rep_cnt,        // ui
  input  logic         space_ok,       // ui
  input  logic [31:0]  ddr_fill_kib,   // ui
  output logic [127:0] m_tdata,        // ui
  output logic         m_tvalid,       // ui
  input  logic         m_tready,       // ui
  output logic [47:0]  src_blocks,     // ui
  output logic [47:0]  wr_blocks,      // ui
  output logic [31:0]  drop_blocks,    // ui
  output logic [15:0]  pad_blocks,     // ui
  output logic [15:0]  trunc_blocks,   // ui
  output logic [39:0]  bp_max_beats,   // ui
  output logic [15:0]  rt_stall_max,   // ui
  output logic         len_reached,    // ui, 1-clock pulse
  output logic         len_reached_s,  // ui, sticky until clear
  output logic         active,         // ui
  output logic         done            // ui
);
  import rec_pkg::*;

  // ---------------------------------------------------------------- state
  logic         run_q, fin_q, closed_q, done_q, pf_q;
  logic         bp_q, prof_q;
  logic [31:0]  sid_q;
  logic [39:0]  len_q;
  logic [31:0]  acc_q;
  logic [63:0]  ts_q;
  logic [47:0]  src_q, pos_q, wr_q;
  logic [31:0]  drop_q;
  logic [15:0]  pad_q, trunc_q;
  logic [4:0]   pad_left_q;
  logic         drop_pend_q;
  logic [39:0]  bpmax_q;
  logic [15:0]  rtmax_q;
  logic         len_p_q, len_s_q;

  // block being accumulated (G)
  logic         g_act_q, g_drop_q, g_len_q;
  logic [7:0]   g_beat_q;
  logic [47:0]  g_pos_q, g_seq_q;
  logic [63:0]  g_ts_q;
  logic [7:0]   g_flags_q;
  logic [5:0]   g_seg_q;
  logic [15:0]  g_rep_q;
  logic [31:0]  g_lost_q, g_kib_q;

  // completed block waiting (W) and block being sent (E)
  logic         w_valid_q;
  logic [47:0]  w_pos_q, w_seq_q;
  logic [63:0]  w_ts_q;
  logic [7:0]   w_flags_q;
  logic [5:0]   w_seg_q;
  logic [15:0]  w_rep_q, w_valid_n_q;
  logic [31:0]  w_lost_q, w_kib_q;

  logic         e_act_q;
  logic [7:0]   e_beat_q;
  logic [47:0]  e_pos_q, e_seq_q;
  logic [63:0]  e_ts_q;
  logic [7:0]   e_flags_q;
  logic [5:0]   e_seg_q;
  logic [15:0]  e_rep_q, e_valid_q;
  logic [31:0]  e_lost_q, e_kib_q, e_n7_q;

  logic [127:0] m_data_q;
  logic         m_valid_q, m_last_q;

  // ---------------------------------------------------------------- NCO / generator control
  logic        e_load;
  logic        allow, nco_en, carry, blk_start, accept, drop_ev, g_done_carry, is_len;
  logic [32:0] sum;
  logic [7:0]  new_flags;

  // W may be refilled in the cycle the emitter takes it (e_load): otherwise beat 0 of block N+1 at > 0.5 beat/clk
  // meets a still-full W and the whole block is dropped although DDR is empty
  assign allow     = space_ok && (!w_valid_q || e_load);
  assign nco_en    = run_q && !fin_q && !stop && !pf && !(bp_q && g_beat_q == 8'd0 && !allow);
  assign sum       = {1'b0, acc_q} + {1'b0, step};
  assign carry     = nco_en && sum[32];
  assign blk_start = carry && (g_beat_q == 8'd0);
  assign accept    = blk_start && allow;
  assign drop_ev   = blk_start && !allow;
  assign g_done_carry = carry && (g_beat_q == 8'hFF);
  assign is_len    = (len_q != 40'd0) && ({8'd0, len_q} == src_q + 48'd1);

  function automatic logic [7:0] mk_flags(input logic first, input logic prof, input logic bp,
                                          input logic dropp);
    logic [7:0] f;
    f = 8'h00;
    f[FLG_FIRST] = first;
    f[FLG_PROF]  = prof;
    f[FLG_BP]    = bp;
    f[FLG_DROP]  = dropp;
    return f;
  endfunction

  assign new_flags = mk_flags(pos_q == 48'd0, prof_q, bp_q, drop_pend_q);

  // closing / pad block construction
  logic        closing_ev, pad_ev;
  logic [47:0] pos_after;
  logic [4:0]  pad_new;
  logic [7:0]  close_flags, pad_flags;
  assign closing_ev = fin_q && !closed_q && !w_valid_q;
  assign pad_ev     = closed_q && (pad_left_q != 5'd0) && !w_valid_q;
  assign pos_after  = g_act_q ? pos_q : (pos_q + 48'd1);
  assign pad_new    = 5'(-pos_after[4:0]);
  always_comb begin
    close_flags = g_act_q ? g_flags_q : new_flags;
    close_flags[FLG_TRUNC] = 1'b1;
    if (pad_new == 5'd0) close_flags[FLG_LAST] = 1'b1;
    pad_flags = new_flags;
    pad_flags[FLG_PAD] = 1'b1;
    if (pad_left_q == 5'd1) pad_flags[FLG_LAST] = 1'b1;
  end

  // ---------------------------------------------------------------- emitter (E) combinational
  logic         adv, e_prod, e_lastbeat;
  logic [127:0] beat_c;
  logic [31:0]  n_c, crc_q;
  logic [7:0]   hdr_flags;
  logic [31:0]  n7_init;
  logic [31:0]  px;

  assign adv        = !m_valid_q || m_tready;
  assign e_prod     = e_act_q && adv;
  assign e_lastbeat = e_prod && (e_beat_q == 8'hFF);
  assign e_load     = w_valid_q && (!e_act_q || e_lastbeat);

  assign px      = {8'd0, w_pos_q[23:0]};
  assign n7_init = ((px << 3) - px) << 8;                 // 7 * (pos << 8) mod 2^32

  assign n_c       = {e_pos_q[23:0], e_beat_q};
  assign hdr_flags = e_flags_q | ((pf_q && (e_flags_q[FLG_TRUNC] || e_flags_q[FLG_PAD] ||
                                            e_flags_q[FLG_LAST])) ? (8'h01 << FLG_PF) : 8'h00);

  always_comb begin
    if (e_beat_q == 8'd0)
      beat_c = {16'd0, e_pos_q, sid_q, BLK_MAGIC};
    else if (e_beat_q == 8'd1)
      beat_c = {e_ts_q, 16'd0, e_seq_q};
    else if (e_beat_q == 8'd2)
      beat_c = {e_kib_q, e_lost_q, e_valid_q, e_rep_q, 10'd0, e_seg_q, hdr_flags, BLK_VER};
    else if (e_beat_q == 8'hFF)
      beat_c = {BLK_MAGIC_END, ~crc_q, e_pos_q[31:0], e_seq_q[31:0]};
    else
      beat_c = {~n_c, n_c ^ 32'hA5A5_A5A5, e_n7_q, n_c};
  end

  rec_crc128 u_crc (
    .clk  (clk),
    .first(e_beat_q == 8'd0),
    .en   (e_prod && (e_beat_q != 8'hFF)),
    .d    (beat_c),
    .crc_q(crc_q)
  );

  // ---------------------------------------------------------------- outputs
  assign m_tdata       = m_data_q;
  assign m_tvalid      = m_valid_q;
  assign src_blocks    = src_q;
  assign wr_blocks     = wr_q;
  assign drop_blocks   = drop_q;
  assign pad_blocks    = pad_q;
  assign trunc_blocks  = trunc_q;
  assign bp_max_beats  = bpmax_q;
  assign rt_stall_max  = rtmax_q;
  assign len_reached   = len_p_q;
  assign len_reached_s = len_s_q;
  assign active        = run_q && !fin_q && !done_q;   // falls the clock after stop/pf/len, not at done
  assign done          = done_q;

  logic [8:0] excess;
  assign excess = 9'd256 - {1'b0, e_beat_q};

  // ---------------------------------------------------------------- sequential
  always_ff @(posedge clk) begin
    len_p_q <= 1'b0;
    if (rst || clear) begin
      run_q <= 1'b0;  fin_q <= 1'b0;  closed_q <= 1'b0;  done_q <= 1'b0;  pf_q <= 1'b0;
      bp_q  <= 1'b0;  prof_q <= 1'b0;
      src_q <= '0;  pos_q <= '0;  wr_q <= '0;  drop_q <= '0;  pad_q <= '0;  trunc_q <= '0;
      pad_left_q <= '0;  drop_pend_q <= 1'b0;  bpmax_q <= '0;  rtmax_q <= '0;  len_s_q <= 1'b0;
      g_act_q <= 1'b0;  g_drop_q <= 1'b0;  g_len_q <= 1'b0;  g_beat_q <= '0;
      w_valid_q <= 1'b0;  e_act_q <= 1'b0;  e_beat_q <= '0;
      m_valid_q <= 1'b0;  m_last_q <= 1'b0;
      acc_q <= 32'hFFFF_FFFF;  ts_q <= '0;
    end else begin
      // ---- start
      if (start && !run_q && !done_q) begin
        run_q <= 1'b1;
        sid_q <= sid;
        len_q <= len_blocks;
        bp_q  <= bp_mode;
        prof_q <= prof_en;
        acc_q <= 32'hFFFF_FFFF;
        ts_q  <= '0;
      end else if (run_q) begin
        ts_q <= ts_q + 64'd1;
        if (nco_en) acc_q <= sum[31:0];
      end
      if (pf && run_q && !done_q) pf_q <= 1'b1;
      if (run_q && !fin_q && (stop || pf)) fin_q <= 1'b1;

      // ---- generator: new block / carries
      if (accept) begin
        g_act_q   <= 1'b1;
        g_drop_q  <= 1'b0;
        g_len_q   <= is_len;
        g_beat_q  <= 8'd1;
        g_pos_q   <= pos_q;
        g_seq_q   <= src_q;
        g_ts_q    <= ts_q;
        g_flags_q <= new_flags;
        g_seg_q   <= seg_idx;
        g_rep_q   <= rep_cnt;
        g_lost_q  <= drop_q;
        g_kib_q   <= ddr_fill_kib;
        src_q     <= src_q + 48'd1;
        pos_q     <= pos_q + 48'd1;
        drop_pend_q <= 1'b0;
      end else if (drop_ev) begin
        g_act_q   <= 1'b0;
        g_drop_q  <= 1'b1;
        g_len_q   <= is_len;
        g_beat_q  <= 8'd1;
        src_q     <= src_q + 48'd1;
        drop_q    <= drop_q + 32'd1;
        drop_pend_q <= 1'b1;
      end else if (carry) begin
        g_beat_q <= g_beat_q + 8'd1;
        if (g_done_carry) begin
          g_beat_q <= 8'd0;
          g_act_q  <= 1'b0;
          g_drop_q <= 1'b0;
          g_len_q  <= 1'b0;
          if (g_len_q) begin
            fin_q    <= 1'b1;
            len_p_q  <= 1'b1;
            len_s_q  <= 1'b1;
          end
        end
      end

      // ---- push into W
      if (e_load) w_valid_q <= 1'b0;
      if (g_done_carry && g_act_q) begin
        w_valid_q   <= 1'b1;
        w_pos_q     <= g_pos_q;   w_seq_q <= g_seq_q;   w_ts_q <= g_ts_q;
        w_flags_q   <= g_flags_q; w_seg_q <= g_seg_q;   w_rep_q <= g_rep_q;
        w_valid_n_q <= 16'd252;   w_lost_q <= g_lost_q; w_kib_q <= g_kib_q;
      end else if (closing_ev) begin
        w_valid_q   <= 1'b1;
        closed_q    <= 1'b1;
        trunc_q     <= trunc_q + 16'd1;
        pad_left_q  <= pad_new;
        w_flags_q   <= close_flags;
        drop_pend_q <= 1'b0;
        g_act_q     <= 1'b0;
        g_drop_q    <= 1'b0;
        g_beat_q    <= 8'd0;
        if (g_act_q) begin
          w_pos_q <= g_pos_q;  w_seq_q <= g_seq_q;  w_ts_q <= g_ts_q;
          w_seg_q <= g_seg_q;  w_rep_q <= g_rep_q;  w_lost_q <= g_lost_q;  w_kib_q <= g_kib_q;
          w_valid_n_q <= (g_beat_q > 8'd3) ? {8'd0, g_beat_q - 8'd3} : 16'd0;
        end else begin
          w_pos_q <= pos_q;  w_seq_q <= src_q;  w_ts_q <= ts_q;
          w_seg_q <= seg_idx; w_rep_q <= rep_cnt;  w_lost_q <= drop_q;  w_kib_q <= ddr_fill_kib;
          w_valid_n_q <= 16'd0;
          src_q <= src_q + 48'd1;
          pos_q <= pos_q + 48'd1;
        end
      end else if (pad_ev) begin
        w_valid_q   <= 1'b1;
        w_pos_q     <= pos_q;  w_seq_q <= src_q;  w_ts_q <= ts_q;
        w_flags_q   <= pad_flags;
        w_seg_q     <= seg_idx; w_rep_q <= rep_cnt;
        w_valid_n_q <= 16'd0;  w_lost_q <= drop_q;  w_kib_q <= ddr_fill_kib;
        src_q       <= src_q + 48'd1;
        pos_q       <= pos_q + 48'd1;
        pad_left_q  <= pad_left_q - 5'd1;
        pad_q       <= pad_q + 16'd1;
        drop_pend_q <= 1'b0;
      end

      // ---- emitter
      if (e_load) begin
        e_act_q   <= 1'b1;
        e_beat_q  <= 8'd0;
        e_pos_q   <= w_pos_q;   e_seq_q <= w_seq_q;   e_ts_q <= w_ts_q;
        e_flags_q <= w_flags_q; e_seg_q <= w_seg_q;   e_rep_q <= w_rep_q;
        e_valid_q <= w_valid_n_q; e_lost_q <= w_lost_q; e_kib_q <= w_kib_q;
        e_n7_q    <= n7_init;
      end else if (e_prod) begin
        e_beat_q <= e_beat_q + 8'd1;
        e_n7_q   <= e_n7_q + 32'd7;
        if (e_beat_q == 8'hFF) e_act_q <= 1'b0;
      end

      // ---- output register
      if (adv) begin
        m_valid_q <= e_act_q;
        if (e_act_q) begin
          m_data_q <= beat_c;
          m_last_q <= (e_beat_q == 8'hFF);
        end
      end
      if (m_valid_q && m_tready && m_last_q) wr_q <= wr_q + 48'd1;

      // ---- backlog metrics
      if (w_valid_q && e_act_q) begin
        if (bp_q) begin
          if ({31'd0, excess} > bpmax_q) bpmax_q <= {31'd0, excess};
        end else begin
          if ({7'd0, excess} > rtmax_q) rtmax_q <= {7'd0, excess};
        end
      end

      // ---- done
      if (closed_q && pad_left_q == 5'd0 && !w_valid_q && !e_act_q && !m_valid_q) done_q <= 1'b1;
    end
  end

endmodule
