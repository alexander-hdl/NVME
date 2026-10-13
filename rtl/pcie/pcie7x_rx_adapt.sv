// =============================================================================
// pcie7x_rx_adapt.sv — pcie_7x 128-bit RX AXI-Stream  ->  internal tlp_axis.
// (ARCHITECTURE.md §3 and §5.1, research/03 §2)
//
// What the core gives us (128-bit mode)
//   * tlast is not used; packet boundaries come from tuser:
//       is_sof  = tuser[14] (SOF present), tuser[13] (1: SOF at DW2, 0: at DW0)
//       is_eof  = tuser[21] (EOF present), tuser[20:19] = DW index of the last DW
//   * a TLP may start at DW0 or DW2; one TLP can end in the lower half of a beat
//     while the next one starts at DW2 ("straddle");
//   * payload DWs are big-endian inside a DW; a 3DW header makes the payload
//     start at an odd DW position.
//
// What we produce (internal format)
//   header beat {H3,H2,H1,H0} (H3=0 for 3DW) with tkeep=FFFF, then payload beats
//   starting at DW0 of beat 1, payload DWs byte-swapped (memory order), tkeep
//   DW-granular, tlast on the final beat; tuser[0]=err_fwd, tuser[1]=ecrc_err.
//
// Architecture (four parts)
//   0. CORE SLICE (axis_skid, 128+22 bits): m_axis_rx_tready is the REGISTERED ready of a
//      full-throughput register slice.  Without it tready was a ~19-level combinational
//      function of the splitter and of the downstream FIFOs' ready (and continued into the
//      IP's rx_null_gen), which is a likely timing failure after routing (review M1).  The
//      slice costs one cycle of latency and no bandwidth; the PG054 rule "tready may drop at
//      any time" is respected (the slice only ever drops it when its skid register is full).
//   1. SPLITTER (this file's always_comb): every core beat holds at most two
//      pieces: segment A = tail of the TLP already open, segment B = head of a
//      new TLP (SOF).  Each piece is handled in ONE cycle:
//        - header DWs are collected in h0_q/h1_q (SOF at DW2: H0,H1 now, H2/H3
//          in the next beat) and pushed as one 128-bit word into a small
//          HEADER FIFO;
//        - payload lanes are pushed as a "payload packet" into a dw_shift
//          instance.  The lane position of the first payload DW P (3DW@0 ->3,
//          3DW@2 ->1, 4DW@2 ->2, 4DW@0 -> next beat lane 0) becomes the LEFT
//          shift of dw_shift, which also repacks the following beats.
//          The number of payload DWs is taken from the Length field (0 = 1024),
//          not from tkeep/EOF: this also removes a TLP digest (TD=1) DW.
//        - a closed TLP leaves one entry (ecrc flag) in an EOF FIFO.
//   2. dw_shift   : the DW realigner (payload only).
//   3. SEQUENCER  : SEQ_HDR sends the next header from the FIFO; if the TLP has
//      payload it moves to SEQ_PAY and copies dw_shift's output up to tlast,
//      then returns to SEQ_HDR with no idle cycle.  The last beat of every TLP
//      carries ecrc_err from the EOF FIFO.
//   A core beat is accepted only when every resource it needs this cycle can
//   take its part, so a straddled beat is still consumed in one cycle; the
//   "extra" output beats appear downstream (header + payload are separate beats).
//   Throughput: one core beat per cycle for non-straddled traffic.
//
// rx_np_ok (review m5).  The adapter holds up to HDR_FIFO_DEPTH MRd headers in its header
// FIFO, behind which CQE MWr / requester CplD are blocked (head-of-line), and the completer
// queues up to NP_EXT_DEPTH more.  rx_np_ok is deasserted when the number of non-posted
// requests queued END TO END (MRd headers in this adapter's header FIFO + `np_ext_count`
// from the completer's request FIFO) reaches HDR_FIFO_DEPTH + NP_EXT_DEPTH - 2, so the core
// is told to let posted/completion TLPs bypass NP TLPs early enough.  `np_backpressure`
// (completer request FIFO almost full) still forces it low as well.  rx_np_ok is registered.
//
// Types.  The splitter works on 2/3-bit lane numbers and counts (no 32-bit `int`): only the
// payload remainder pl_rem_q is wide (11 bit), and it is compared with 3-bit values through
// its upper bits, so the comparators are a few LUTs instead of carry chains.
// =============================================================================
module pcie7x_rx_adapt #(
  parameter int HDR_FIFO_DEPTH = 8,
  parameter int EOF_FIFO_DEPTH = 16,
  parameter int NP_EXT_DEPTH   = 16        // request FIFO depth of the completer (np_ext_count range)
) (
  input  logic         clk,
  input  logic         rst,
  // core side (PG054 128-bit RX)
  input  logic [127:0] m_axis_rx_tdata,
  input  logic [15:0]  m_axis_rx_tkeep,      // unused: length comes from the TLP header
  input  logic         m_axis_rx_tlast,      // tied low by the core in 128-bit mode
  input  logic         m_axis_rx_tvalid,
  output logic         m_axis_rx_tready,
  input  logic [21:0]  m_axis_rx_tuser,
  output logic         rx_np_ok,
  input  logic         np_backpressure,    // completer request FIFO almost full
  input  logic [$clog2(NP_EXT_DEPTH):0] np_ext_count,   // completer request FIFO occupancy (MRd queued there)
  // internal tlp_axis (source)
  output logic [127:0] m_tlp_tdata,
  output logic [15:0]  m_tlp_tkeep,
  output logic         m_tlp_tlast,
  output logic [1:0]   m_tlp_tuser,
  output logic         m_tlp_tvalid,
  input  logic         m_tlp_tready
);

  import nvme_host_pkg::*;

  // ---- core tuser fields (PG054 Table 2-10) -----------------------------------
  localparam int TU_ECRC_ERR = 0;
  localparam int TU_ERR_FWD  = 1;
  localparam int TU_SOF      = 14;   // is_sof[4]: SOF present
  localparam int TU_SOF_DW2  = 13;   // is_sof[3]: SOF at DW2 (else DW0)
  localparam int TU_EOF      = 21;   // is_eof[4]: EOF present
  localparam int TU_EOF_DW_H = 20;   // is_eof[3:2]: EOF DW index
  localparam int TU_EOF_DW_L = 19;

  localparam int LANES      = 4;
  localparam int DW_W       = 32;
  localparam int MAX_PL_DW  = 1024;                    // Length 0 encodes 1024
  localparam int PLW        = $clog2(MAX_PL_DW) + 1;   // 11 bits

  // ---- core-side register slice (registered m_axis_rx_tready) ------------------------
  logic [127:0] c_tdata;
  logic [21:0]  c_tuser;
  logic         c_tvalid, c_tready;

  axis_skid #(.W(128), .UW(22)) u_core_skid (
    .clk(clk), .rst(rst),
    .s_tdata(m_axis_rx_tdata), .s_tkeep(16'h0000), .s_tlast(1'b0), .s_tuser(m_axis_rx_tuser),
    .s_tvalid(m_axis_rx_tvalid), .s_tready(m_axis_rx_tready),       // registered ready
    .m_tdata(c_tdata), .m_tkeep(), .m_tlast(), .m_tuser(c_tuser),
    .m_tvalid(c_tvalid), .m_tready(c_tready));

  // ---- beat decode ---------------------------------------------------------------
  logic        sofv, sof2, eofv;
  logic [1:0]  eofdw;
  logic        err_in, ecrc_in;
  logic [31:0] d0, d1, d2, d3;

  assign sofv    = c_tuser[TU_SOF];
  assign sof2    = c_tuser[TU_SOF_DW2];
  assign eofv    = c_tuser[TU_EOF];
  assign eofdw   = c_tuser[TU_EOF_DW_H:TU_EOF_DW_L];
  assign err_in  = c_tuser[TU_ERR_FWD];
  assign ecrc_in = c_tuser[TU_ECRC_ERR];
  assign {d3, d2, d1, d0} = c_tdata;

  // ---- splitter state -------------------------------------------------------------
  logic            pkt_open_q;      // a TLP has started and its EOF has not been seen
  logic            hdr_wait_q;      // H0/H1 captured, H2(/H3) arrive in the next beat
  logic [31:0]     h0_q, h1_q;
  logic            err_q;           // err_fwd of the open TLP
  logic [PLW-1:0]  pl_rem_q;        // payload DWs still to be forwarded

  logic            pkt_open_n, hdr_wait_n, err_n;
  logic [31:0]     h0_n, h1_n;
  logic [PLW-1:0]  pl_rem_n;

  // per-beat actions (before qualification by the handshake)
  logic            hdr_push_i, hdr_has_pl, hdr_err, eof_push_i, pl_push_i, pl_last;
  logic [127:0]    hdr_word;
  logic [3:0]      pl_keep;
  logic [1:0]      pl_shift;

  // ---- sinks of the splitter ------------------------------------------------------
  logic hf_s_tready, ef_s_tready, dw_s_tready;
  logic fire;                       // core beat consumed this cycle

  // thermometer mask: n lanes -> n low bits set (n = 0..4)
  function automatic logic [3:0] lane_mask(input logic [2:0] n);
    case (n)
      3'd0:    return 4'b0000;
      3'd1:    return 4'b0001;
      3'd2:    return 4'b0011;
      3'd3:    return 4'b0111;
      default: return 4'b1111;
    endcase
  endfunction

  always_comb begin : splitter
    logic [1:0]     sof_lane;                 // 0 or 2
    logic [2:0]     a_end, b_end, a_first, a_n, cnt, rem_lo;
    logic           rem_big, rem_le_an;       // pl_rem_q >= 8 / pl_rem_q <= a_n
    logic           eof_in_a, eof_in_b, b_is4, b_data;
    logic [31:0]    b_h0;
    logic [PLW-1:0] b_len;

    pkt_open_n = pkt_open_q;
    hdr_wait_n = hdr_wait_q;
    h0_n       = h0_q;
    h1_n       = h1_q;
    err_n      = err_q;
    pl_rem_n   = pl_rem_q;
    hdr_push_i = 1'b0;
    hdr_word   = '0;
    hdr_has_pl = 1'b0;
    hdr_err    = 1'b0;
    eof_push_i = 1'b0;
    pl_push_i  = 1'b0;
    pl_keep    = 4'h0;
    pl_last    = 1'b0;
    pl_shift   = 2'd0;

    sof_lane = sof2 ? 2'd2 : 2'd0;
    // The EOF belongs to the open TLP (segment A) if it comes before the SOF lane:
    //   no SOF in this beat, or SOF at DW2 and the EOF DW is 0 or 1 (SOF at DW0 -> nothing before it)
    eof_in_a = pkt_open_q && eofv && (!sofv || (sof2 && !eofdw[1]));
    eof_in_b = sofv && eofv && !eof_in_a;
    // lanes [a_first, a_end) of this beat belong to the open TLP, lanes [.., b_end) to the new one
    a_end    = eof_in_a ? {1'b0, eofdw} + 3'd1 : (sofv ? {1'b0, sof_lane} : 3'd4);
    b_end    = eof_in_b ? {1'b0, eofdw} + 3'd1 : 3'd4;
    a_first  = 3'd0;
    a_n      = 3'd0;
    cnt      = 3'd0;
    rem_big  = (pl_rem_q[PLW-1:3] != '0);
    rem_lo   = pl_rem_q[2:0];
    rem_le_an = 1'b0;
    b_h0     = sof2 ? d2 : d0;
    b_is4    = tlp_is_4dw(b_h0);
    b_data   = tlp_has_data(b_h0);
    b_len    = (b_h0[9:0] == 10'd0) ? PLW'(MAX_PL_DW) : PLW'(b_h0[9:0]);

    // ---------------- segment A: continuation of the open TLP -------------------
    if (pkt_open_q) begin
      if (hdr_wait_q) begin
        // H2 (and H3) are in the first lanes of this beat -> header complete
        a_first    = tlp_is_4dw(h0_q) ? 3'd2 : 3'd1;
        hdr_push_i = 1'b1;
        hdr_word   = {tlp_is_4dw(h0_q) ? d1 : 32'h0, d0, h1_q, h0_q};
        hdr_err    = err_q;
        hdr_wait_n = 1'b0;
      end
      a_n       = (a_end > a_first) ? (a_end - a_first) : 3'd0;
      rem_le_an = !rem_big && (rem_lo <= a_n);
      if (pl_rem_q != '0 && a_n != 3'd0) begin
        cnt       = rem_le_an ? rem_lo : a_n;           // min(a_n, pl_rem_q)
        pl_push_i = 1'b1;
        pl_keep   = lane_mask(a_first + cnt);           // header lanes are kept, dw_shift drops them
        pl_last   = rem_le_an || eof_in_a;              // cnt == pl_rem_q
        pl_shift  = a_first[1:0];
        pl_rem_n  = pl_rem_q - PLW'(cnt);
        hdr_has_pl = hdr_wait_q;
      end
      if (eof_in_a) begin
        pkt_open_n = 1'b0;
        eof_push_i = 1'b1;
      end
    end

    // ---------------- segment B: a new TLP starts in this beat ---------------------
    if (sofv) begin
      err_n = err_in;
      if (sof2) begin
        // H0,H1 in lanes 2,3; H2(/H3) follow in the next beat
        h0_n       = d2;
        h1_n       = d3;
        hdr_wait_n = 1'b1;
        pkt_open_n = 1'b1;
        pl_rem_n   = b_data ? b_len : '0;
      end else begin
        // whole header in this beat
        hdr_push_i = 1'b1;
        hdr_word   = {b_is4 ? d3 : 32'h0, d2, d1, d0};
        hdr_err    = err_in;
        hdr_has_pl = 1'b0;
        pkt_open_n = !eof_in_b;
        pl_rem_n   = '0;
        if (b_data) begin
          if (!b_is4) begin
            if (b_end == 3'd4) begin              // lane 3 carries payload DW0
              pl_push_i  = 1'b1;
              pl_keep    = 4'hF;
              pl_shift   = 2'd3;
              pl_last    = (b_len == PLW'(1)) || eof_in_b;
              pl_rem_n   = b_len - PLW'(1);
              hdr_has_pl = 1'b1;
            end
          end else if (!eof_in_b) begin           // payload starts in the next beat, lane 0
            hdr_has_pl = 1'b1;
            pl_rem_n   = b_len;
          end
        end
        if (eof_in_b) eof_push_i = 1'b1;
      end
    end
  end

  // ---- resource check, handshake --------------------------------------------------
  logic rdy_all;
  assign rdy_all = (!hdr_push_i || hf_s_tready) &&
                   (!eof_push_i || ef_s_tready) &&
                   (!pl_push_i  || dw_s_tready);
  assign c_tready = !c_tvalid || rdy_all;
  assign fire     = c_tvalid && rdy_all;

  always_ff @(posedge clk) begin
    if (rst) begin
      pkt_open_q <= 1'b0;
      hdr_wait_q <= 1'b0;
      pl_rem_q   <= '0;
    end else if (fire) begin
      pkt_open_q <= pkt_open_n;
      hdr_wait_q <= hdr_wait_n;
      pl_rem_q   <= pl_rem_n;
    end
  end

  always_ff @(posedge clk) begin
    if (fire) begin
      h0_q  <= h0_n;
      h1_q  <= h1_n;
      err_q <= err_n;
    end
  end

  // ---- header FIFO ----------------------------------------------------------------
  logic [127:0] hf_tdata;
  logic         hf_tlast, hf_tvalid, hf_tready;
  logic [1:0]   hf_tuser;
  logic [15:0]  hf_tkeep;
  logic [$clog2(HDR_FIFO_DEPTH):0] hf_count;
  logic         hf_almost_full;

  axis_fifo_sync #(.W(128), .UW(2), .DEPTH(HDR_FIFO_DEPTH)) u_hdr_fifo (
    .clk(clk), .rst(rst),
    .s_tdata(hdr_word), .s_tkeep(16'hFFFF), .s_tlast(!hdr_has_pl), .s_tuser({1'b0, hdr_err}),
    .s_tvalid(fire && hdr_push_i), .s_tready(hf_s_tready),
    .m_tdata(hf_tdata), .m_tkeep(hf_tkeep), .m_tlast(hf_tlast), .m_tuser(hf_tuser),
    .m_tvalid(hf_tvalid), .m_tready(hf_tready),
    .count(hf_count), .almost_full(hf_almost_full));

  // ---- EOF FIFO (one entry per closed TLP: ecrc_err) --------------------------------
  logic [7:0]  ef_tdata;
  logic [0:0]  ef_tkeep, ef_tuser;
  logic        ef_tlast, ef_tvalid, ef_tready;
  logic [$clog2(EOF_FIFO_DEPTH):0] ef_count;
  logic        ef_almost_full;

  axis_fifo_sync #(.W(8), .UW(1), .DEPTH(EOF_FIFO_DEPTH)) u_eof_fifo (
    .clk(clk), .rst(rst),
    .s_tdata({7'b0, ecrc_in}), .s_tkeep(1'b1), .s_tlast(1'b1), .s_tuser(1'b0),
    .s_tvalid(fire && eof_push_i), .s_tready(ef_s_tready),
    .m_tdata(ef_tdata), .m_tkeep(ef_tkeep), .m_tlast(ef_tlast), .m_tuser(ef_tuser),
    .m_tvalid(ef_tvalid), .m_tready(ef_tready),
    .count(ef_count), .almost_full(ef_almost_full));

  // ---- payload realigner ---------------------------------------------------------------
  logic [127:0] pl_tdata_sw;
  logic [15:0]  pl_tkeep_ext;
  logic [127:0] dw_tdata;
  logic [15:0]  dw_tkeep;
  logic         dw_tlast, dw_tvalid, dw_tready;

  always_comb begin
    for (int i = 0; i < LANES; i++) begin
      pl_tdata_sw[DW_W*i +: DW_W] = bswap32(c_tdata[DW_W*i +: DW_W]);
      pl_tkeep_ext[4*i +: 4]      = {4{pl_keep[i]}};
    end
  end

  dw_shift u_pl_shift (
    .clk(clk), .rst(rst),
    .s_tdata(pl_tdata_sw), .s_tkeep(pl_tkeep_ext), .s_tlast(pl_last),
    .s_tvalid(fire && pl_push_i), .s_tready(dw_s_tready),
    .s_shift(pl_shift), .s_dir(1'b0),
    .m_tdata(dw_tdata), .m_tkeep(dw_tkeep), .m_tlast(dw_tlast),
    .m_tvalid(dw_tvalid), .m_tready(dw_tready));

  // ---- output sequencer ---------------------------------------------------------------
  typedef enum logic {SEQ_HDR = 1'b0, SEQ_PAY = 1'b1} seq_t;
  seq_t seq_q;
  logic pay_err_q;
  logic out_fire;

  always_comb begin : sequencer
    m_tlp_tdata  = hf_tdata;
    m_tlp_tkeep  = 16'hFFFF;
    m_tlp_tlast  = hf_tlast;
    m_tlp_tuser  = {hf_tlast & ef_tdata[0], hf_tuser[0]};
    m_tlp_tvalid = 1'b0;
    hf_tready    = 1'b0;
    dw_tready    = 1'b0;
    ef_tready    = 1'b0;
    out_fire     = 1'b0;
    case (seq_q)
      SEQ_HDR: begin
        m_tlp_tvalid = hf_tvalid && (!hf_tlast || ef_tvalid);
        out_fire     = m_tlp_tvalid && m_tlp_tready;
        hf_tready    = out_fire;
        ef_tready    = out_fire && hf_tlast;
      end
      default: begin // SEQ_PAY
        m_tlp_tdata  = dw_tdata;
        m_tlp_tkeep  = dw_tkeep;
        m_tlp_tlast  = dw_tlast;
        m_tlp_tuser  = {dw_tlast & ef_tdata[0], pay_err_q};
        m_tlp_tvalid = dw_tvalid && (!dw_tlast || ef_tvalid);
        out_fire     = m_tlp_tvalid && m_tlp_tready;
        dw_tready    = out_fire;
        ef_tready    = out_fire && dw_tlast;
      end
    endcase
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      seq_q <= SEQ_HDR;
    end else if (out_fire) begin
      if (seq_q == SEQ_HDR) begin
        if (!hf_tlast) seq_q <= SEQ_PAY;
      end else if (dw_tlast) begin
        seq_q <= SEQ_HDR;
      end
    end
  end

  always_ff @(posedge clk) begin
    if (out_fire && seq_q == SEQ_HDR) pay_err_q <= hf_tuser[0];
  end

  // ---- non-posted bookkeeping: MRd headers waiting in the header FIFO ------------------
  localparam int NP_LIMIT = HDR_FIFO_DEPTH + NP_EXT_DEPTH - 2;
  logic                         np_push, np_pop, np_ok_d;
  logic [$clog2(HDR_FIFO_DEPTH):0] np_cnt_q;
  logic [$clog2(HDR_FIFO_DEPTH+NP_EXT_DEPTH)+1:0] np_total;

  assign np_push = fire && hdr_push_i && tlp_is_mrd(hdr_word[31:0]);
  assign np_pop  = hf_tready && tlp_is_mrd(hf_tdata[31:0]);   // hf_tready = header beat leaves
  assign np_total = np_cnt_q + np_ext_count;
  assign np_ok_d  = !np_backpressure && (np_total < NP_LIMIT);

  always_ff @(posedge clk) begin
    if (rst) begin
      np_cnt_q <= '0;
      rx_np_ok <= 1'b1;
    end else begin
      np_cnt_q <= np_cnt_q + (np_push ? 1'b1 : 1'b0) - (np_pop ? 1'b1 : 1'b0);
      rx_np_ok <= np_ok_d;
    end
  end

  // unused inputs
  logic unused_ok;
  assign unused_ok = &{1'b0, m_axis_rx_tkeep, m_axis_rx_tlast, hf_count, hf_almost_full,
                       ef_count, ef_almost_full, ef_tkeep, ef_tlast, ef_tuser, ef_tdata[7:1],
                       hf_tkeep, hf_tuser[1]};

endmodule
