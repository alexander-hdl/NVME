// =============================================================================
// dw_shift.sv — double-word (32-bit) shifter for payload packet streams.
//
// Input : packets of 128-bit beats, tkeep is DW-granular (4'hF per present DW,
//         the DWs of a packet are contiguous from DW0 of the first beat; all
//         beats but the last are full).
// dir=0 (LEFT)  : drop the first `shift` DWs of the packet and compact the rest.
// dir=1 (RIGHT) : insert `shift` empty DWs (tkeep=0) in front of DW0.
// `shift`/`dir` are sampled with the first beat of every packet.
//
// Algorithm (a 2-beat window)
//   Let R be the right shift (R=0 also covers "left by 0": pass-through).
//   RIGHT : output beat m = { cur[3-R:0] , prev[3:4-R] }   -> produced the
//           moment input beat m is accepted (prev = input beat m-1; for m=0
//           prev contributes nothing).  If the last input beat has DWs left
//           over (cur[3:4-R] non-empty) they are "spilled" into one extra
//           output beat -> the FLUSH state.
//   LEFT by L (1..3) : output beat m = { cur[L-1:0] , prev[3:L] } where prev is
//           input beat m and cur is input beat m+1, so the first input beat only
//           PRIMES the window (no output).  At the end the remaining DWs of the
//           last beat (cur[3:L]) are flushed in an extra output beat, unless the
//           last beat held <= L DWs (then they were already merged).
//           A 1-beat left-shifted packet is emitted directly: cur >> L; if
//           nothing is left (n <= L) one beat with tkeep=0 and tlast=1 is
//           produced so the packet boundary is preserved.
//   `prev_*_q` is the small "pending register"; the output beat is registered
//   (out_*_q) so m_* are flop outputs.  Throughput: one input beat per cycle.
//   The only stall is the FLUSH cycle; it is hidden whenever the next packet is
//   a left-shift packet longer than one beat (its first beat only primes and
//   produces no output), so s_tready may be high during the flush cycle then.
//   s_tready therefore depends combinationally on s_shift/s_dir/s_tlast of the
//   beat being offered (legal for AXI-S: ready may depend on the offered beat).
// =============================================================================
module dw_shift (
  input  logic         clk,
  input  logic         rst,
  input  logic [127:0] s_tdata,
  input  logic [15:0]  s_tkeep,
  input  logic         s_tlast,
  input  logic         s_tvalid,
  output logic         s_tready,
  input  logic [1:0]   s_shift,
  input  logic         s_dir,
  output logic [127:0] m_tdata,
  output logic [15:0]  m_tkeep,
  output logic         m_tlast,
  output logic         m_tvalid,
  input  logic         m_tready
);

  localparam int DW_W    = 32;          // bits per DW
  localparam int BEAT_DW = 4;           // DWs per beat
  localparam logic DIR_LEFT  = 1'b0;
  localparam logic DIR_RIGHT = 1'b1;

  // ---- state ----------------------------------------------------------------
  logic         first_q;                // next accepted beat is the first of a packet
  logic [1:0]   shift_q;                // shift of the packet in progress
  logic         dir_q;
  logic [127:0] prev_data_q;            // pending register: previous input beat
  logic [3:0]   prev_keep_q;            //   its per-DW valid bits
  logic         flush_q;                // an extra (spill) output beat is pending

  logic         out_valid_q;
  logic [127:0] out_data_q;
  logic [3:0]   out_keep_q;
  logic         out_last_q;

  // ---- helpers ----------------------------------------------------------------
  logic         can_load;               // output register free or being drained
  logic [3:0]   cur_keep;
  logic [1:0]   sh;                     // effective shift for the offered beat
  logic         dir_eff;
  logic         is_right;
  logic         prime_new;              // offered beat is a priming first beat
  logic [2:0]   fl_sh;                  // flush shift (in DWs)
  logic         acc;                    // beat accepted this cycle

  assign can_load  = ~out_valid_q | m_tready;
  assign cur_keep  = {s_tkeep[12], s_tkeep[8], s_tkeep[4], s_tkeep[0]};
  assign sh        = first_q ? s_shift : shift_q;
  assign dir_eff   = first_q ? s_dir   : dir_q;
  assign is_right  = (dir_eff == DIR_RIGHT) || (sh == 2'd0);
  assign prime_new = (s_dir == DIR_LEFT) && (s_shift != 2'd0) && !s_tlast;
  assign s_tready  = can_load && (!flush_q || prime_new);
  assign acc       = s_tvalid && s_tready;

  // flush shift: RIGHT spills cur[3:4-R] ; LEFT spills cur[3:L]
  assign fl_sh = (dir_q == DIR_RIGHT) ? 3'(BEAT_DW) - 3'(shift_q) : 3'(shift_q);

  // ---- combinational datapath ------------------------------------------------
  logic         o_valid, o_last, set_flush, spill;
  logic [127:0] o_data;
  logic [3:0]   o_keep;

  always_comb begin
    o_valid   = 1'b0;
    o_data    = '0;
    o_keep    = '0;
    o_last    = 1'b0;
    set_flush = 1'b0;
    spill     = 1'b0;

    if (flush_q) begin
      if (can_load) begin
        o_valid = 1'b1;
        o_data  = prev_data_q >> (DW_W * fl_sh);
        o_keep  = prev_keep_q >> fl_sh;
        o_last  = 1'b1;
      end
    end else if (acc) begin
      if (is_right) begin
        o_valid = 1'b1;
        o_data  = s_tdata << (DW_W * sh);
        o_keep  = cur_keep << sh;
        if (!first_q) begin
          o_data |= prev_data_q >> (DW_W * (BEAT_DW - sh));
          o_keep |= prev_keep_q >> (BEAT_DW - sh);
        end
        spill     = ((cur_keep >> (BEAT_DW - sh)) != 4'h0);
        o_last    = s_tlast && !spill;
        set_flush = s_tlast && spill;
      end else if (first_q) begin
        // LEFT, first beat
        if (s_tlast) begin
          o_valid = 1'b1;
          o_data  = s_tdata >> (DW_W * sh);
          o_keep  = cur_keep >> sh;
          o_last  = 1'b1;
        end                                   // else: priming, no output
      end else begin
        // LEFT, window full: merge tail of prev with head of cur
        o_valid   = 1'b1;
        o_data    = (prev_data_q >> (DW_W * sh)) | (s_tdata << (DW_W * (BEAT_DW - sh)));
        o_keep    = (prev_keep_q >> sh) | (cur_keep << (BEAT_DW - sh));
        spill     = ((cur_keep >> sh) != 4'h0);
        o_last    = s_tlast && !spill;
        set_flush = s_tlast && spill;
      end
    end
  end

  // ---- sequential ---------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      first_q     <= 1'b1;
      flush_q     <= 1'b0;
      out_valid_q <= 1'b0;
    end else begin
      if (can_load) out_valid_q <= o_valid;
      if (flush_q && can_load) flush_q <= 1'b0;
      if (set_flush)           flush_q <= 1'b1;
      if (acc)                 first_q <= s_tlast;
    end
  end

  always_ff @(posedge clk) begin
    if (can_load && o_valid) begin
      out_data_q <= o_data;
      out_keep_q <= o_keep;
      out_last_q <= o_last;
    end
    if (acc) begin
      prev_data_q <= s_tdata;
      prev_keep_q <= cur_keep;
      if (first_q) begin
        shift_q <= s_shift;
        dir_q   <= s_dir;
      end
    end
  end

  assign m_tvalid = out_valid_q;
  assign m_tdata  = out_data_q;
  assign m_tlast  = out_last_q;
  assign m_tkeep  = {{4{out_keep_q[3]}}, {4{out_keep_q[2]}}, {4{out_keep_q[1]}}, {4{out_keep_q[0]}}};

endmodule
