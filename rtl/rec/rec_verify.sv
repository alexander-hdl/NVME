// =============================================================================
// rec_verify.sv — verification of the recorded blocks read back from the SSD (WP6).
//
// Domain  : ui_clk only (clk/rst/clear).  Spec: STAND/RECORDER_SPEC.md 3.1, 3.3, 4.4 (WP6).
// Ports   : s_tdata = {lba32[192:161], off17[160:144], be16[143:128], data128[127:0]} from vrf_tap,
//           AXI-S sink, s_tready is constant 1 after reset (one beat per ui_clk = 1.6 GB/s at 100 MHz,
//           faster than the tap ever delivers, so the tap FIFO drains).  res/end_found/mvp_flags are
//           registers.  `clear` = 1-cycle pulse (ui_clear): flushes the pipeline, results and contexts.
//           mvp_flags (extra output, not in spec 4.4, may stay unconnected) = `flags` byte of the
//           max-valid-position block (bit 1 = LAST).
// Beat position: byte = ((lba32 - START_LBA[31:0]) << lbads) + off; pos = byte >> 12; j = byte[11:4].
// Pipeline (10 stages, 1 beat/clk): S1 input reg; S2 lba - START; S3 << lbads; S4 + off;
//           S5 pos/j/n; S6 expected beat; S7 per-byte compare; S8 pre-decode (header / trailer compares, CRC
//           contribution of the data, seq + lost, first mismatching DW: no context access);
//           S9 per-block context read-modify-write (the CRC update is linear: crc' = L(crc) ^ crc_of_data),
//           the result of the beat goes into registered event flags; S10 result counters from the event flags.
// Block context: N_CTX entries indexed by pos[CTX_SHIFT +: log2(N_CTX)] (CTX_SHIFT = log2 of the
//           blocks per slot, 5 = 128 KiB), so blocks of different slots whose beats interleave do
//           not share a context; a block is checked in order inside its context.
// Counting rules (all in 3.3 terms):
//   vb   = sum of byte-enabled bytes of every received beat
//   ve   = beats with at least one error (payload mismatch, header/trailer field mismatch, CRC,
//          out-of-order, misaligned, partial header/trailer beat)
//   ok   = blocks closed (beat 255) with no error;  bad = closed blocks with an error + blocks
//          abandoned (beat 0 of a new block while the previous one of the context never closed)
//   crc  = blocks whose CRC-32 differs (CRC checked only when beats 0..254 were full and in order)
//   pe   = payload beats with a byte mismatch under `be`
//   ooo  = beats out of order (j > 0 not the expected next beat, or misaligned address) + abandoned blocks
//   prt  = beats with be != FFFF (header/trailer partial beats make the block bad; a partial
//          payload beat only prevents the CRC check)
//   pad  = OK blocks with FLG_PAD
//   fbp  = min pos of a bad block (all ones = none); end_found = fbp exists
//   mvp/lsq/lts/lst = max pos among OK blocks, its seq, ts, lost
//   fep/feb/exp_dw/got_dw = first payload mismatch: beat position {pos,j}, DW index 0..3, expected/received DW
//   ovf  = tap FIFO overflow seen since clear (sticky): result invalid
// =============================================================================
module rec_verify
  import rec_pkg::*;
#(
  parameter logic [63:0] START_LBA = 64'd0,
  parameter int          N_CTX     = 8,       // power of 2
  parameter int          CTX_SHIFT = 5        // log2(SLOT_BLOCKS)
) (
  input  logic          clk,                  // ui
  input  logic          rst,                  // ui
  input  logic          clear,                // ui, 1-cycle pulse
  input  logic [31:0]   exp_sid,              // ui, quasi-static
  input  logic [3:0]    lbads,                // ui, quasi-static
  input  logic [192:0]  s_tdata,              // ui
  input  logic          s_tvalid,             // ui
  output logic          s_tready,             // ui
  input  logic          tap_ovf,              // ui
  output vrf_res_t      res,                  // ui
  output logic          end_found,            // ui
  output logic [7:0]    mvp_flags             // ui
);

  localparam int          CW      = $clog2(N_CTX);
  localparam logic [31:0] START32 = START_LBA[31:0];

  assign s_tready = ~rst;

  // ---------------------------------------------------------------- S1 .. S7
  logic         v1_q, v2_q, v3_q, v4_q, v5_q, v6_q, v7_q, v8_q;
  logic [31:0]  lba1_q, d2_q;
  logic [16:0]  off1_q, off2_q, off3_q;
  logic [43:0]  sh3_q, byte4_q;
  logic [15:0]  be1_q, be2_q, be3_q, be4_q, be5_q, be6_q, be7_q;
  logic [127:0] g1_q, g2_q, g3_q, g4_q, g5_q, g6_q, g7_q;
  logic [31:0]  n5_q, pos5_q, pos6_q, pos7_q;
  logic [7:0]   j5_q, j6_q, j7_q;
  logic         al5_q, al6_q, al7_q;
  logic [127:0] exp6_q, exp7_q;
  logic [15:0]  mism7_q;
  logic [4:0]   pc7_q;

  logic [31:0] n7x;
  assign n7x = {n5_q[28:0], 3'b000} - n5_q;

  always_ff @(posedge clk) begin
    if (rst || clear) begin
      v1_q <= 1'b0; v2_q <= 1'b0; v3_q <= 1'b0; v4_q <= 1'b0;
      v5_q <= 1'b0; v6_q <= 1'b0; v7_q <= 1'b0; v8_q <= 1'b0;
    end else begin
      v1_q <= s_tvalid;
      v2_q <= v1_q; v3_q <= v2_q; v4_q <= v3_q; v5_q <= v4_q; v6_q <= v5_q; v7_q <= v6_q; v8_q <= v7_q;
    end
    // S1
    lba1_q <= s_tdata[192:161];
    off1_q <= s_tdata[160:144];
    be1_q  <= s_tdata[143:128];
    g1_q   <= s_tdata[127:0];
    // S2
    d2_q   <= lba1_q - START32;
    off2_q <= off1_q;  be2_q <= be1_q;  g2_q <= g1_q;
    // S3 (lbads 9..12, anything else is treated as 9)
    case (lbads)
      4'd10:   sh3_q <= {12'b0, d2_q} << 10;
      4'd11:   sh3_q <= {12'b0, d2_q} << 11;
      4'd12:   sh3_q <= {12'b0, d2_q} << 12;
      default: sh3_q <= {12'b0, d2_q} << 9;
    endcase
    off3_q <= off2_q;  be3_q <= be2_q;  g3_q <= g2_q;
    // S4
    byte4_q <= sh3_q + 44'(off3_q);
    be4_q <= be3_q;  g4_q <= g3_q;
    // S5
    n5_q   <= byte4_q[35:4];
    pos5_q <= byte4_q[43:12];
    j5_q   <= byte4_q[11:4];
    al5_q  <= (byte4_q[3:0] == 4'd0);
    be5_q  <= be4_q;  g5_q <= g4_q;
    // S6
    exp6_q <= {~n5_q, n5_q ^ 32'hA5A5_A5A5, n7x, n5_q};
    pos6_q <= pos5_q;  j6_q <= j5_q;  al6_q <= al5_q;  be6_q <= be5_q;  g6_q <= g5_q;
    // S7
    for (int i = 0; i < 16; i++)
      mism7_q[i] <= be6_q[i] && (g6_q[8*i +: 8] != exp6_q[8*i +: 8]);
    pc7_q <= 5'(be6_q[0]) + 5'(be6_q[1]) + 5'(be6_q[2]) + 5'(be6_q[3]) + 5'(be6_q[4]) + 5'(be6_q[5])
           + 5'(be6_q[6]) + 5'(be6_q[7]) + 5'(be6_q[8]) + 5'(be6_q[9]) + 5'(be6_q[10]) + 5'(be6_q[11])
           + 5'(be6_q[12]) + 5'(be6_q[13]) + 5'(be6_q[14]) + 5'(be6_q[15]);
    exp7_q <= exp6_q;  pos7_q <= pos6_q;  j7_q <= j6_q;  al7_q <= al6_q;  be7_q <= be6_q;  g7_q <= g6_q;
  end

  // ---------------------------------------------------------------- S8: pre-decode (no context access)
  logic [31:0]  pos8_q;
  logic [7:0]   j8_q;
  logic         al8_q, full8_q, anym8_q, hdr0bad8_q, verbad8_q, endbad8_q;
  logic [4:0]   pc8_q;
  logic [63:0]  pl8_q;                 // pos + lost: seq check without a 64-bit subtract in the context stage
  logic [31:0]  cd8_q;                 // crc32_128(0, data)
  logic [127:0] g8_q;
  logic [1:0]   fdw8_q;
  logic [31:0]  expdw8_q, gotdw8_q;
  logic [1:0]   fdw7;
  logic         j0_8q, j1_8q, j2_8q, jmid_8q, j255_8q, jle254_8q;   // beat class (j = 0, 1, 2, 3..254, 255, <= 254)

  always_comb
    fdw7 = (|mism7_q[3:0]) ? 2'd0 : (|mism7_q[7:4]) ? 2'd1 : (|mism7_q[11:8]) ? 2'd2 : 2'd3;

  always_ff @(posedge clk) begin
    pos8_q     <= pos7_q;
    j8_q       <= j7_q;
    j0_8q      <= (j7_q == 8'd0);
    j1_8q      <= (j7_q == 8'd1);
    j2_8q      <= (j7_q == 8'd2);
    jmid_8q    <= (j7_q >= 8'd3) && (j7_q <= 8'd254);
    j255_8q    <= (j7_q == 8'd255);
    jle254_8q  <= (j7_q <= 8'd254);
    al8_q      <= al7_q;
    full8_q    <= (be7_q == 16'hFFFF);
    pc8_q      <= pc7_q;
    anym8_q    <= |mism7_q;
    hdr0bad8_q <= (g7_q[31:0] != BLK_MAGIC) || (g7_q[63:32] != exp_sid) || (g7_q[127:64] != {32'd0, pos7_q});
    verbad8_q  <= (g7_q[7:0] != BLK_VER) || (g7_q[63:48] > 16'd252);
    endbad8_q  <= (g7_q[127:96] != BLK_MAGIC_END) || (g7_q[63:32] != pos7_q);
    pl8_q      <= {32'd0, pos7_q} + {32'd0, g7_q[95:64]};
    cd8_q      <= crc32_128(32'd0, g7_q);
    g8_q       <= g7_q;
    fdw8_q     <= fdw7;
    expdw8_q   <= exp7_q[32*fdw7 +: 32];
    gotdw8_q   <= g7_q[32*fdw7 +: 32];
  end

  // ---------------------------------------------------------------- S9: contexts
  typedef struct packed {
    logic        open;       // a block of cpos is in progress
    logic [31:0] cpos;
    logic [7:0]  nj;         // next expected beat
    logic [31:0] crc;        // raw CRC register over beats 0..nj-1
    logic        crc_ok;     // beats 0..nj-1 were full and in order
    logic        err;        // an error was seen in this block
    logic        seq_v;      // beat 1 captured
    logic [63:0] seq;
    logic [63:0] ts;
    logic [31:0] lost;
    logic [7:0]  flags;
  } ctx_t;

  ctx_t ctx_q [N_CTX];
  logic fbp_set_q, mvp_set_q, fe_set_q;

  // event flags of the beat that was in S9 (registered, consumed by S10); all flags are 0 for an empty slot
  logic [4:0]   e_pc_q;
  logic         e_ooo_q, e_ve_q, e_bad_q, e_prt_q, e_pe_q, e_crcerr_q, e_cok_q, e_pad_q;
  logic [31:0]  e_fbpc_q, e_pos_q;
  logic [7:0]   e_j_q;
  logic [1:0]   e_fdw_q;
  logic [31:0]  e_exp_q, e_got_q;
  logic [63:0]  e_seq_q, e_ts_q;
  logic [31:0]  e_lost_q;
  logic [7:0]   e_flags_q;

  // working variables (blocking, local to the always_ff below)
  ctx_t          c0, c;
  logic [CW-1:0] ci;
  logic          full, inord, same_blk, beat_err, crc_err, err_fin, aband;
  logic          cpos_eq, nj_eq, crc_ne, seq_ne, lo_ne;
  logic          l_ooo, l_ve, l_bad, l_prt, l_pe, l_cok;
  logic [31:0]   l_fbpc;

  // The comparisons against the context (cpos, nj, crc, seq) are computed side by side from the unmodified context
  // c0; the checks of a beat that is not in order are irrelevant (such a beat is an error anyway), so the error
  // decision is a flat function of these compares and of the registered beat class flags (S8).
  always_ff @(posedge clk) begin
    e_pc_q <= 5'd0;
    e_ooo_q <= 1'b0; e_ve_q <= 1'b0; e_bad_q <= 1'b0; e_prt_q <= 1'b0; e_pe_q <= 1'b0;
    e_crcerr_q <= 1'b0; e_cok_q <= 1'b0; e_pad_q <= 1'b0;
    if (rst || clear) begin
      for (int k = 0; k < N_CTX; k++) ctx_q[k] <= '0;
    end else if (v8_q) begin
      ci       = pos8_q[CTX_SHIFT +: CW];
      c0       = ctx_q[ci];
      c        = c0;
      full     = full8_q;
      cpos_eq  = (c0.cpos == pos8_q);
      nj_eq    = (c0.nj == j8_q);
      crc_ne   = (g8_q[95:64] != ~c0.crc);
      seq_ne   = (c0.seq != pl8_q);                 // seq - lost != pos
      lo_ne    = (g8_q[31:0] != c0.seq[31:0]);
      same_blk = c0.open && cpos_eq;
      inord    = same_blk && nj_eq;
      aband    = j0_8q && c0.open;                  // previous block of this context never closed
      // ---- error decision of the beat
      if (j0_8q)         beat_err = !full || hdr0bad8_q;
      else if (!inord)   beat_err = 1'b1;
      else if (j1_8q)    beat_err = !full;
      else if (j2_8q)    beat_err = !full || (full && verbad8_q) || (full && c0.seq_v && seq_ne);
      else if (jmid_8q)  beat_err = anym8_q;
      else               beat_err = !full || endbad8_q || (c0.seq_v && lo_ne) || (c0.crc_ok && crc_ne);
      crc_err  = j255_8q && inord && full && c0.crc_ok && crc_ne;
      err_fin  = beat_err || (inord && !j0_8q && c0.err);
      // ---- context update
      if (j0_8q) begin
        c        = '0;
        c.open   = 1'b1;
        c.cpos   = pos8_q;
        c.nj     = 8'd1;
        c.crc    = CRC32_INIT;
        c.crc_ok = 1'b1;
      end else if (!inord) begin
        if (!same_blk) begin
          c = '0;  c.open = 1'b1;  c.cpos = pos8_q;
        end
        c.crc_ok = 1'b0;
        c.nj     = j8_q + 8'd1;
      end else begin
        c.nj = j8_q + 8'd1;
      end
      if (jle254_8q) begin
        // crc32_128 is linear: crc32_128(crc, d) = crc32_128(crc, 0) ^ crc32_128(0, d); the data term is cd8_q
        if (c.crc_ok && full) c.crc = crc32_128(c.crc, 128'd0) ^ cd8_q;
        else                  c.crc_ok = 1'b0;
      end
      if (j1_8q && full) begin
        c.seq = g8_q[63:0];  c.ts = g8_q[127:64];  c.seq_v = 1'b1;
      end
      if (j2_8q && full) begin
        c.flags = g8_q[15:8];
        c.lost  = g8_q[95:64];
      end
      c.err = err_fin;
      if (j255_8q) c.open = 1'b0;
      // ---- events of the beat (misaligned: nothing is checked, the context is written back unchanged)
      if (!al8_q) begin
        c = c0;
        l_ooo = 1'b1;  l_ve = 1'b1;  l_bad = 1'b0;  l_prt = 1'b0;  l_pe = 1'b0;  l_cok = 1'b0;
        crc_err = 1'b0;
        l_fbpc  = pos8_q;
      end else begin
        l_ooo  = aband || (!j0_8q && !inord);
        l_ve   = beat_err;
        l_bad  = aband || (j255_8q && err_fin);
        l_cok  = j255_8q && !err_fin;
        l_prt  = !full;
        l_pe   = jmid_8q && anym8_q;
        l_fbpc = aband ? c0.cpos : pos8_q;
      end
      e_pc_q <= pc8_q;
      e_ooo_q <= l_ooo; e_ve_q <= l_ve; e_bad_q <= l_bad; e_prt_q <= l_prt; e_pe_q <= l_pe;
      e_crcerr_q <= crc_err; e_cok_q <= l_cok; e_pad_q <= l_cok && c.flags[FLG_PAD];
      e_fbpc_q <= l_fbpc; e_pos_q <= pos8_q; e_j_q <= j8_q;
      e_fdw_q <= fdw8_q; e_exp_q <= expdw8_q; e_got_q <= gotdw8_q;
      e_seq_q <= c.seq; e_ts_q <= c.ts; e_lost_q <= c.lost; e_flags_q <= c.flags;
      ctx_q[ci] <= c;
    end
  end

  // ---------------------------------------------------------------- S10: result counters
  always_ff @(posedge clk) begin
    if (rst || clear) begin
      res        <= '0;
      res.fbp    <= '1;
      res.ovf    <= 1'b0;
      fbp_set_q  <= 1'b0;
      mvp_set_q  <= 1'b0;
      fe_set_q   <= 1'b0;
      end_found  <= 1'b0;
      mvp_flags  <= 8'd0;
    end else begin
      if (tap_ovf) res.ovf <= 1'b1;
      res.vb <= res.vb + 48'(e_pc_q);
      if (e_ooo_q)    res.ooo <= res.ooo + 32'd1;
      if (e_ve_q)     res.ve  <= res.ve + 32'd1;
      if (e_prt_q)    res.prt <= res.prt + 32'd1;
      if (e_crcerr_q) res.crc <= res.crc + 32'd1;
      if (e_bad_q) begin
        res.bad <= res.bad + 32'd1;
        if (!fbp_set_q || {8'd0, e_fbpc_q} < res.fbp) begin
          res.fbp <= {8'd0, e_fbpc_q};  fbp_set_q <= 1'b1;
        end
      end
      if (e_pe_q) begin
        res.pe <= res.pe + 32'd1;
        if (!fe_set_q) begin
          fe_set_q    <= 1'b1;
          res.fep     <= {e_pos_q, e_j_q};
          res.feb     <= {6'd0, e_fdw_q};
          res.exp_dw  <= e_exp_q;
          res.got_dw  <= e_got_q;
        end
      end
      if (e_cok_q) begin
        res.ok <= res.ok + 40'd1;
        if (e_pad_q) res.pad <= res.pad + 32'd1;
        if (!mvp_set_q || {8'd0, e_pos_q} > res.mvp) begin
          res.mvp <= {8'd0, e_pos_q};  mvp_set_q <= 1'b1;
          res.lsq <= e_seq_q;  res.lts <= e_ts_q;  res.lst <= e_lost_q;
          mvp_flags <= e_flags_q;
          res.mvp_flags <= e_flags_q;                 // WP8: also in the result struct (!RV lfl)
        end
      end
      end_found <= fbp_set_q;
    end
  end

endmodule
