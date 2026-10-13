// =============================================================================
// data_ring.sv — BRAM ring that holds the user's data until the SSD has read it
//                (ARCHITECTURE.md §5.9)
//
//   data_clk domain                    clk (user_clk) domain
//   s_data --> [async FIFO 512] --> ring writer --> N_SLOTS x slot RAM
//                                        |                |
//                                   slot_full[k] <--      +--> read port (completer)
//                                   slot_free[k] --> (pulse from nvme_write_engine)
//
// * The user streams 128-bit beats at its own clock. A 512-deep async FIFO
//   crosses into clk.
// * The ring writer pops the FIFO and writes beats sequentially: beat 0..B-1
//   into slot 0, then slot 1, ... (B = SLOT_BYTES/16), wrapping after slot
//   N_SLOTS-1. When the last beat of slot k is written, slot_full[k] goes high:
//   the write engine may now build a command for that slot.
// * slot_full[k] stays high until a one-cycle pulse on slot_free[k] (given
//   after the SSD's CQE for that command). While the slot the writer wants to
//   fill is still full the FIFO is not popped, the FIFO fills up and the user
//   sees s_data_tready = 0 (back-pressure all the way to the source).
//   If a slot_free pulse and the "last beat" event of the same slot collide,
//   the set wins (cannot happen in a correct flow).
// * One sdp_ram per slot (depth SLOT_BYTES/16 beats): the slot index is the
//   address bits above the slot size, so the read mux is a plain N_SLOTS:1
//   mux steered by the (2-cycle delayed) slot index. SLOT_BYTES must be a power
//   of two (>= 4096) and N_SLOTS >= 2.
// * Input acceptance (no silent drops).  s_data_tready is LOW whenever the ring cannot
//   store the beat: while data_rst is active, while the user_clk-domain reset `rst`
//   is active (synchronised inside axis_fifo_async), and while `accept_enable`
//   (user_clk domain, driven by the core from the sequencer's `running`) is low.
//   `accept_enable` is 2-FF synchronised into data_clk; its gate acts on BOTH tvalid
//   and tready of the FIFO, so a handshake ("tvalid && tready") always means the beat
//   was stored.  A source that waits for tready therefore loses nothing before the
//   write engine is enabled, during a link retrain or after the stop request.
// * Read port: rd_addr is the region-relative byte address (DATA_BASE already
//   subtracted) of a 16-byte beat; rd_data is valid exactly 2 cycles after rd_en.
//
// Statistics (clk domain):
//   beats_in      beats written into the ring (= beats accepted and popped)
//   stall_cycles  clk cycles during which the user was held off (data_clk-side
//                 level "tvalid && !tready", 2-FF synchronised)
//   input_stalled that synchronised level
// =============================================================================
module data_ring
  import nvme_host_pkg::*;
#(
  parameter int unsigned N_SLOTS    = N_SLOTS_DEF,
  parameter int unsigned SLOT_BYTES = SLOT_BYTES_DEF,
  parameter int unsigned FIFO_DEPTH = 512
)(
  // user stream (data_clk domain)
  input  logic         data_clk,
  input  logic         data_rst,
  input  logic [127:0] s_data_tdata,
  input  logic         s_data_tvalid,
  output logic         s_data_tready,
  // user_clk domain
  input  logic         clk,
  input  logic         rst,
  input  logic         accept_enable = 1'b1,   // user_clk domain; 0 = s_data_tready held low
  output logic [N_SLOTS-1:0]       slot_full,
  input  logic [N_SLOTS-1:0]       slot_free,
  output logic [$clog2(N_SLOTS):0] slots_used,
  output logic         input_stalled,
  output logic [63:0]  beats_in,
  output logic [31:0]  stall_cycles,
  // completer read port
  input  logic         rd_en,
  input  logic [31:0]  rd_addr,
  output logic [127:0] rd_data
);

  localparam int unsigned BEATS_PER_SLOT = SLOT_BYTES / 16;
  localparam int unsigned BEAT_W         = $clog2(BEATS_PER_SLOT);
  localparam int unsigned SLOT_W         = $clog2(N_SLOTS);
  localparam logic [15:0] BE_ALL         = 16'hFFFF;

  // elaboration-time sanity check.  SLOT_BYTES <= 2 MiB: prp_synth serves ONE 4 KiB PRP list
  // page per slot (512 entries + PRP1 = 513 pages = 2 MiB + 4 KiB max); a chained PRP list is not supported.
  if ((SLOT_BYTES & (SLOT_BYTES - 1)) != 0 || SLOT_BYTES < 4096 || SLOT_BYTES > 2*1024*1024 || N_SLOTS < 2) begin : g_param_check
    $fatal(1, "data_ring: SLOT_BYTES must be a power of two in [4096, 2 MiB] and N_SLOTS >= 2");
  end

  // ---------------------------------------------------------------------------
  // Clock-domain crossing FIFO
  // ---------------------------------------------------------------------------
  logic [127:0] f_tdata;
  logic         f_tvalid, f_tready;

  // accept_enable (user_clk) -> data_clk, power-up value "not accepting"
  (* ASYNC_REG = "TRUE" *) logic acc_s1_q = 1'b0, acc_s2_q = 1'b0;
  always_ff @(posedge data_clk) begin
    acc_s1_q <= accept_enable;
    acc_s2_q <= acc_s1_q;
  end

  logic s_f_tready, s_f_tvalid;
  assign s_f_tvalid    = s_data_tvalid & acc_s2_q & ~data_rst;
  assign s_data_tready = s_f_tready    & acc_s2_q & ~data_rst;

  axis_fifo_async #(.W(128), .DEPTH(FIFO_DEPTH)) u_fifo (
    .s_clk(data_clk), .s_rst(data_rst),
    .s_tdata(s_data_tdata), .s_tvalid(s_f_tvalid), .s_tready(s_f_tready),
    .m_clk(clk), .m_rst(rst),
    .m_tdata(f_tdata), .m_tvalid(f_tvalid), .m_tready(f_tready));

  // ---------------------------------------------------------------------------
  // Ring writer
  // ---------------------------------------------------------------------------
  logic [SLOT_W-1:0] wr_slot;
  logic [BEAT_W-1:0] wr_beat;

  assign f_tready = !slot_full[wr_slot];
  wire   wr_fire  = f_tvalid && f_tready;
  wire   wr_end   = wr_fire && (wr_beat == BEAT_W'(BEATS_PER_SLOT - 1));

  always_ff @(posedge clk) begin
    if (rst) begin
      wr_slot   <= '0;
      wr_beat   <= '0;
      slot_full <= '0;
      beats_in  <= '0;
    end else begin
      // clear first, set afterwards (set wins on a collision)
      slot_full <= (slot_full & ~slot_free);
      if (wr_fire) begin
        beats_in <= beats_in + 64'd1;
        wr_beat  <= wr_beat + 1'b1;
      end
      if (wr_end) begin
        wr_beat <= '0;
        wr_slot <= (wr_slot == SLOT_W'(N_SLOTS - 1)) ? '0 : wr_slot + 1'b1;
        slot_full[wr_slot] <= 1'b1;
      end
    end
  end

  always_comb begin
    slots_used = '0;
    for (int k = 0; k < N_SLOTS; k++) slots_used = slots_used + (slot_full[k] ? 1'b1 : 1'b0);
  end

  // ---------------------------------------------------------------------------
  // Slot RAMs and read mux
  // ---------------------------------------------------------------------------
  logic [127:0] slot_rd [N_SLOTS];
  logic [SLOT_W-1:0] rd_slot, rd_slot_d1, rd_slot_d2;
  logic              rd_ok_d1, rd_ok_d2;

  assign rd_slot = rd_addr[BEAT_W+4 +: SLOT_W];

  // Read-address fan-out.  rd_addr / rd_en would otherwise drive EVERY slot RAM
  // (N_SLOTS * SLOT_BYTES/4096 RAMB36 = 256 for the default 8 x 128 KiB).  Each slot gets its
  // OWN register copy of the beat address and of its enable (u_ram sees only that copy, so
  // the fan-out is SLOT_BYTES/4096 = 32 RAMB36 per register; dont_touch stops Vivado from
  // merging the copies back into one).  The copy is the first of the two read stages, so
  // the read latency stays exactly 2 cycles: stage 1 = address copy register, stage 2 =
  // the RAM's read latch (sdp_ram with OUT_REG = 0, no output register); the slot mux is
  // steered by the 2-cycle delayed slot index below.
  always_ff @(posedge clk) begin
    rd_slot_d1 <= rd_slot;   rd_slot_d2 <= rd_slot_d1;
    rd_ok_d1   <= (rd_addr[31:BEAT_W+4] < 32'(N_SLOTS));
    rd_ok_d2   <= rd_ok_d1;
  end

  for (genvar k = 0; k < N_SLOTS; k++) begin : g_slot
    (* dont_touch = "true", max_fanout = 32 *) logic [BEAT_W-1:0] rd_addr_cp_q;
    (* dont_touch = "true", max_fanout = 32 *) logic              rd_en_cp_q;

    always_ff @(posedge clk) begin
      rd_addr_cp_q <= rd_addr[4 +: BEAT_W];
      rd_en_cp_q   <= rd_en && (rd_slot == SLOT_W'(k));
    end

    sdp_ram #(.DEPTH(BEATS_PER_SLOT), .W(128), .OUT_REG(0)) u_ram (
      .clk(clk),
      .wr_en(wr_fire && (wr_slot == SLOT_W'(k))), .wr_be(BE_ALL),
      .wr_addr(wr_beat), .wr_data(f_tdata),
      .rd_en(rd_en_cp_q), .rd_addr(rd_addr_cp_q),
      .rd_data(slot_rd[k]));
  end

  assign rd_data = rd_ok_d2 ? slot_rd[rd_slot_d2] : 128'd0;

  // ---------------------------------------------------------------------------
  // Statistics
  // ---------------------------------------------------------------------------
  logic stall_lvl_d;
  (* ASYNC_REG = "TRUE" *) logic stall_s1, stall_s2;      // 2-FF synchroniser into clk

  always_ff @(posedge data_clk) begin
    if (data_rst) stall_lvl_d <= 1'b0;
    else          stall_lvl_d <= s_data_tvalid && !s_data_tready;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      stall_s1 <= 1'b0; stall_s2 <= 1'b0; stall_cycles <= '0;
    end else begin
      stall_s1 <= stall_lvl_d; stall_s2 <= stall_s1;
      if (stall_s2) stall_cycles <= stall_cycles + 32'd1;
    end
  end
  assign input_stalled = stall_s2;

endmodule
