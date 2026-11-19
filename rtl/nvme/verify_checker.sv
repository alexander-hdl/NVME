// =============================================================================
// verify_checker.sv - compares the data of NVMe Read commands with the test pattern
// while it streams in, without storing it.   ARCHITECTURE.md section 5.19
//
// The verify run reads back the LBA range that a recording wrote.  The SSD delivers the
// data as MWr TLPs into the DATA region (data-ring addresses); hostmem_writer turns them
// into 16-byte beats (wr_en_data, region-relative wr_beat_addr, wr_data, wr_be).  For every
// beat this module computes the expected pattern and compares it under the byte enables.
//
// Pattern (rtl/board/kc705_top.sv generator, sim/tb/tb_top.sv): 128-bit beat n (32-bit,
// wraps) = {~n, n ^ 32'hA5A5A5A5, n*7, n}, beat n at byte offset 16*n from START_LBA.
// A beat at DATA-relative address `addr` belongs to slot k = addr / SLOT_BYTES with byte
// offset off = addr % SLOT_BYTES inside it; the Read command of slot k started at LBA
// slot_lba[k] (nvme_write_engine.slot_lba_flat), so
//     n = (((slot_lba[k] - START_LBA) << lbads) + off) >> 4        (32 LSB only)
// Only the low 32 bits of slot_lba / START_LBA matter for n; the full 64-bit slot_lba is
// kept for first_err_lba.  The table is sampled in the FIRST pipeline stage (the engine
// changes slot_lba[k] only after the previous command of the slot has completed, i.e.
// after its data went through hostmem_writer).
//
// Pipeline (one beat per cycle, fully pipelined, no stalls; every stage is a register):
//   S1  slot select: slot = addr[OFFW +: SW]; L1 = slot_lba[slot]; off = addr[OFFW-1:0]
//   S2  d = L1[31:0] - START_LBA[31:0]        (32-bit subtract)      ; offl = off >> lbads
//   S3  sh = d << lbads                       (4:1 mux, lbads 9..12)
//   S4  S = sh + off (36 bit) -> n = S[35:4]  (one adder)            ; lo = L1[31:0] + offl
//   S5  n7 = (n << 3) - n (one subtractor)    ; exp = {~n, n ^ A5A5A5A5, n7, n}
//                                             ; err LBA = {L1[63:32] + carry, lo}
//   S6  per-byte compare: mism[i] = be[i] && (got[i] != exp[i])
//   S7  reduce: beat_err = |mism, first mismatching DW, popcount(be)
//   S8  select the DW, accumulate the statistics (registered outputs)
// Latency from wr_en to the statistics: 8 cycles.  Every logic level between two registers
// is at most one adder / one 8-bit compare / one small mux, so the module is meant to close
// timing at 125 MHz on Artix-7 (-2) with margin.
//
// Outputs (registered, cleared by rst / clear):
//   chk_bytes       number of byte-enabled bytes compared
//   err_beats       beats with at least one mismatching enabled byte
//   first_err_lba   LBA containing the first mismatching beat (valid when err_seen)
//   first_err_exp / first_err_got   expected / received DW of the first mismatching DW of
//                   that beat (DW with at least one mismatching enabled byte; compared whole)
//   err_seen        sticky: at least one mismatching beat
// `clear` (1-cycle pulse = run restart) also flushes the pipeline.  lbads is quasi-static.
// =============================================================================
module verify_checker #(
  parameter int unsigned N_SLOTS    = 8,
  parameter int unsigned SLOT_BYTES = 131072,        // power of 2
  parameter logic [63:0] START_LBA  = 64'd0
) (
  input  logic                    clk,
  input  logic                    rst,
  input  logic                    clear,            // 1-cycle pulse: new run

  input  logic [3:0]              lbads,            // LBA size = 1 << lbads (9..12)
  input  logic [N_SLOTS*64-1:0]   slot_lba_flat,    // slot k at [64k +: 64]

  input  logic                    wr_en,            // hostmem_writer wr_en_data
  input  logic [31:0]             wr_beat_addr,     // DATA-relative, 16 B aligned
  input  logic [127:0]            wr_data,
  input  logic [15:0]             wr_be,

  output logic [63:0]             chk_bytes,
  output logic [31:0]             err_beats,
  output logic [63:0]             first_err_lba,
  output logic [31:0]             first_err_exp,
  output logic [31:0]             first_err_got,
  output logic                    err_seen
);

  localparam int unsigned OFFW = $clog2(SLOT_BYTES);
  localparam int unsigned SW   = (N_SLOTS > 1) ? $clog2(N_SLOTS) : 1;
  localparam logic [31:0] START32 = START_LBA[31:0];

  if (SLOT_BYTES != (1 << OFFW)) begin : g_slot_check
    $error("verify_checker: SLOT_BYTES (%0d) must be a power of 2", SLOT_BYTES);
  end

  // ---------------------------------------------------------------------------
  // S1: slot select
  // ---------------------------------------------------------------------------
  logic [SW-1:0]  slot_i;
  logic [63:0]    lba_sel;
  assign slot_i = wr_beat_addr[OFFW +: SW];
  always_comb begin
    lba_sel = '0;
    for (int k = 0; k < N_SLOTS; k++)
      if (slot_i == SW'(k)) lba_sel = slot_lba_flat[64*k +: 64];
  end

  logic           v1, v2, v3, v4, v5, v6, v7;       // stage valid
  logic [63:0]    L1, L2, L3, L4;
  logic [OFFW-1:0] off1, off2, off3;
  logic [127:0]   g1, g2, g3, g4, g5, g6, g7;       // received data, delayed
  logic [15:0]    b1, b2, b3, b4, b5, b6;           // byte enables, delayed

  always_ff @(posedge clk) begin
    if (rst || clear) v1 <= 1'b0; else v1 <= wr_en;
    L1   <= lba_sel;
    off1 <= wr_beat_addr[OFFW-1:0];
    g1   <= wr_data;
    b1   <= wr_be;
  end

  // ---------------------------------------------------------------------------
  // S2: d = lba - START_LBA ; offl = off >> lbads
  // ---------------------------------------------------------------------------
  logic [31:0]     d2;
  logic [OFFW-1:0] offl2;
  always_ff @(posedge clk) begin
    if (rst || clear) v2 <= 1'b0; else v2 <= v1;
    d2    <= L1[31:0] - START32;
    offl2 <= off1 >> lbads;
    L2    <= L1;
    off2  <= off1;
    g2    <= g1;
    b2    <= b1;
  end

  // ---------------------------------------------------------------------------
  // S3: sh = d << lbads (lbads 9..12; anything else is treated as 9)
  // ---------------------------------------------------------------------------
  logic [35:0]     sh3;
  logic [OFFW-1:0] offl3;
  always_ff @(posedge clk) begin
    if (rst || clear) v3 <= 1'b0; else v3 <= v2;
    case (lbads)
      4'd10:   sh3 <= {4'b0, d2} << 10;
      4'd11:   sh3 <= {4'b0, d2} << 11;
      4'd12:   sh3 <= {4'b0, d2} << 12;
      default: sh3 <= {4'b0, d2} << 9;
    endcase
    offl3 <= offl2;
    L3    <= L2;
    off3  <= off2;
    g3    <= g2;
    b3    <= b2;
  end

  // ---------------------------------------------------------------------------
  // S4: n = (sh + off) >> 4 ; error-LBA low half
  // ---------------------------------------------------------------------------
  logic [35:0]     sum4;
  logic [31:0]     n4;
  logic [32:0]     lo4;
  assign sum4 = sh3 + 36'(off3);
  always_ff @(posedge clk) begin
    if (rst || clear) v4 <= 1'b0; else v4 <= v3;
    n4   <= sum4[35:4];
    lo4  <= {1'b0, L3[31:0]} + 33'(offl3);
    L4   <= L3;
    g4   <= g3;
    b4   <= b3;
  end

  // ---------------------------------------------------------------------------
  // S5: expected beat {~n, n ^ A5A5A5A5, n*7, n} ; error-LBA high half
  // ---------------------------------------------------------------------------
  logic [127:0]    exp5;
  logic [63:0]     elba5;
  logic [31:0]     n7_4;
  assign n7_4 = {n4[28:0], 3'b000} - n4;
  always_ff @(posedge clk) begin
    if (rst || clear) v5 <= 1'b0; else v5 <= v4;
    exp5  <= {~n4, n4 ^ 32'hA5A5_A5A5, n7_4, n4};
    elba5 <= {L4[63:32] + 32'(lo4[32]), lo4[31:0]};
    g5    <= g4;
    b5    <= b4;
  end

  // ---------------------------------------------------------------------------
  // S6: per-byte compare under the byte enables
  // ---------------------------------------------------------------------------
  logic [15:0]  mism6;
  logic [127:0] exp6;
  logic [63:0]  elba6;
  always_ff @(posedge clk) begin
    if (rst || clear) v6 <= 1'b0; else v6 <= v5;
    for (int i = 0; i < 16; i++)
      mism6[i] <= b5[i] && (g5[8*i +: 8] != exp5[8*i +: 8]);
    exp6  <= exp5;
    elba6 <= elba5;
    g6    <= g5;
    b6    <= b5;
  end

  // ---------------------------------------------------------------------------
  // S7: reduce
  // ---------------------------------------------------------------------------
  logic         err7;
  logic [1:0]   dw7;
  logic [4:0]   nb7;
  logic [127:0] exp7;
  logic [63:0]  elba7;
  logic [3:0]   dwerr6;
  always_comb
    for (int j = 0; j < 4; j++) dwerr6[j] = |mism6[4*j +: 4];

  always_ff @(posedge clk) begin
    if (rst || clear) v7 <= 1'b0; else v7 <= v6;
    err7  <= |mism6;
    dw7   <= dwerr6[0] ? 2'd0 : dwerr6[1] ? 2'd1 : dwerr6[2] ? 2'd2 : 2'd3;
    nb7   <= 5'(b6[0]) + 5'(b6[1]) + 5'(b6[2]) + 5'(b6[3]) + 5'(b6[4]) + 5'(b6[5]) + 5'(b6[6]) + 5'(b6[7])
           + 5'(b6[8]) + 5'(b6[9]) + 5'(b6[10]) + 5'(b6[11]) + 5'(b6[12]) + 5'(b6[13]) + 5'(b6[14]) + 5'(b6[15]);
    exp7  <= exp6;
    elba7 <= elba6;
    g7    <= g6;
  end

  // ---------------------------------------------------------------------------
  // S8: statistics
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst || clear) begin
      chk_bytes     <= 64'd0;
      err_beats     <= 32'd0;
      first_err_lba <= 64'd0;
      first_err_exp <= 32'd0;
      first_err_got <= 32'd0;
      err_seen      <= 1'b0;
    end else begin
      if (v7) begin
        chk_bytes <= chk_bytes + 64'(nb7);
        if (err7) begin
          if (err_beats != 32'hFFFF_FFFF) err_beats <= err_beats + 1'b1;
          if (!err_seen) begin
            err_seen      <= 1'b1;
            first_err_lba <= elba7;
            first_err_exp <= exp7[32*dw7 +: 32];
            first_err_got <= g7[32*dw7 +: 32];
          end
        end
      end
    end
  end

endmodule
