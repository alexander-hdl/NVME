// =============================================================================
// pcie7x_tx_adapt.sv — internal tlp_axis  ->  pcie_7x 128-bit TX AXI-Stream.
// (ARCHITECTURE.md §5.2, research/03 §2)
//
// Core TX rules: a TLP always starts at DW0 of a beat, the DWs are contiguous
// (a 3DW header is followed IMMEDIATELY by payload DW0 in lane 3), payload DWs
// are big-endian (byte-swapped relative to our memory-order format), tkeep on
// the last beat is 000F/00FF/0FFF/FFFF, tuser = 0 (no streaming mode).
//
// Datapath (three stages)
//   1. axis_skid  : registers the input, so the data-dependent ready below never
//                   reaches the upstream block combinationally.
//   2. CONVERTER  : 4DW headers pass unchanged (payload byte-swapped).
//        3DW header: the stream is shifted LEFT by one DW, i.e. output beat m
//        is  { in[m+1].DW0 , in[m].DW3..DW1 }  with in[0] = {H2,H1,H0} placed so
//        that  beat0 = {D0,H2,H1,H0}  and  beat m = {D(4m), D(4m-1..4m-3)}.
//        The first (header) beat only PRIMES the window; every following input
//        beat produces one output beat; if the last input beat holds >= 2 DWs
//        the remaining DWs are SPILLED into one extra output beat (flush).  The
//        flush cycle is overlapped with the header beat of the next packet when
//        that one is a 3DW packet with payload (which produces no output), so
//        the output keeps running at one beat per cycle across packets.
//        Header-only 3DW packets become one beat with tkeep=0FFF.
//   3. axis_fifo_sync + packet counter (STORE_AND_FORWARD=1): converted beats
//        are written into a FIFO and a packet is released to the core only
//        after its last beat is stored, so tvalid never drops inside a packet
//        (the core's streaming-mode rule), regardless of source gaps.  The
//        output side then streams at one beat per cycle while tready=1, with no
//        bubbles inside or between packets that are already complete.
//        STORE_AND_FORWARD=0 gives cut-through (lowest latency, but tvalid
//        follows source gaps).  FIFO_DEPTH must be >= the longest packet in beats
//        (MPS 512 B -> 33 beats), otherwise store-and-forward would deadlock.
// =============================================================================
module pcie7x_tx_adapt #(
  parameter bit STORE_AND_FORWARD = 1'b1,
  parameter int FIFO_DEPTH        = 64
) (
  input  logic         clk,
  input  logic         rst,
  // internal tlp_axis (sink)
  input  logic [127:0] s_tlp_tdata,
  input  logic [15:0]  s_tlp_tkeep,
  input  logic         s_tlp_tlast,
  input  logic [1:0]   s_tlp_tuser,       // unused on the TX side
  input  logic         s_tlp_tvalid,
  output logic         s_tlp_tready,
  // core side (PG054 128-bit TX)
  output logic [127:0] s_axis_tx_tdata,
  output logic [15:0]  s_axis_tx_tkeep,
  output logic         s_axis_tx_tlast,
  output logic         s_axis_tx_tvalid,
  input  logic         s_axis_tx_tready,
  output logic [3:0]   s_axis_tx_tuser,
  input  logic [5:0]   tx_buf_av
);

  import nvme_host_pkg::*;

  localparam int LANES = 4;
  localparam int DW_W  = 32;
  localparam int CNT_W = $clog2(FIFO_DEPTH) + 1;

  assign s_axis_tx_tuser = 4'b0000;

  // ---- stage 1: input register slice ------------------------------------------------
  logic [127:0] k_tdata;
  logic [15:0]  k_tkeep;
  logic         k_tlast, k_tvalid, conv_ready;
  logic [1:0]   k_tuser;

  axis_skid #(.W(128), .UW(2)) u_in_skid (
    .clk(clk), .rst(rst),
    .s_tdata(s_tlp_tdata), .s_tkeep(s_tlp_tkeep), .s_tlast(s_tlp_tlast), .s_tuser(s_tlp_tuser),
    .s_tvalid(s_tlp_tvalid), .s_tready(s_tlp_tready),
    .m_tdata(k_tdata), .m_tkeep(k_tkeep), .m_tlast(k_tlast), .m_tuser(k_tuser),
    .m_tvalid(k_tvalid), .m_tready(conv_ready));

  // ---- stage 2: converter -----------------------------------------------------------
  logic         first_q;            // next accepted beat is a header beat
  logic         mode3_q;            // packet in progress has a 3DW header
  logic         flush_q;            // spill beat pending
  logic [127:0] prev_data_q;        // window register (3DW mode)
  logic [3:0]   prev_keep_q;

  logic         fifo_rdy;           // converted-beat FIFO can take a beat
  logic [3:0]   cur_keep;
  logic [127:0] sw_data;            // payload with every DW byte-swapped
  logic         hdr_is3;            // offered beat is a 3DW header
  logic         prime_new;          // offered beat is a header that produces no output
  logic         acc;

  always_comb begin
    for (int i = 0; i < LANES; i++) sw_data[DW_W*i +: DW_W] = bswap32(k_tdata[DW_W*i +: DW_W]);
  end

  assign cur_keep   = {k_tkeep[12], k_tkeep[8], k_tkeep[4], k_tkeep[0]};
  assign hdr_is3    = !tlp_is_4dw(k_tdata[31:0]);
  assign prime_new  = hdr_is3 && !k_tlast;
  assign conv_ready = fifo_rdy && (!flush_q || prime_new);
  assign acc        = k_tvalid && conv_ready;

  logic         w_valid, w_last, set_flush;
  logic [127:0] w_data;
  logic [3:0]   w_keep;

  always_comb begin
    w_valid   = 1'b0;
    w_data    = '0;
    w_keep    = '0;
    w_last    = 1'b0;
    set_flush = 1'b0;
    if (flush_q) begin
      if (fifo_rdy) begin                   // spill: remaining DWs of the last beat
        w_valid = 1'b1;
        w_data  = {32'h0, prev_data_q[127:32]};
        w_keep  = {1'b0, prev_keep_q[3:1]};
        w_last  = 1'b1;
      end
    end else if (acc) begin
      if (first_q) begin
        if (hdr_is3) begin
          if (k_tlast) begin                // 3DW header only
            w_valid = 1'b1;
            w_data  = {32'h0, k_tdata[95:0]};
            w_keep  = 4'b0111;
            w_last  = 1'b1;
          end                               // else: prime, no output
        end else begin                      // 4DW header: pass unchanged
          w_valid = 1'b1;
          w_data  = k_tdata;
          w_keep  = 4'hF;
          w_last  = k_tlast;
        end
      end else if (!mode3_q) begin          // 4DW packet payload
        w_valid = 1'b1;
        w_data  = sw_data;
        w_keep  = cur_keep;
        w_last  = k_tlast;
      end else begin                        // 3DW packet payload: left by one DW
        w_valid   = 1'b1;
        w_data    = {sw_data[31:0], prev_data_q[127:32]};
        w_keep    = {cur_keep[0], prev_keep_q[3:1]};
        w_last    = k_tlast && !cur_keep[1];
        set_flush = k_tlast && cur_keep[1];
      end
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      first_q <= 1'b1;
      flush_q <= 1'b0;
    end else begin
      if (flush_q && fifo_rdy) flush_q <= 1'b0;
      if (set_flush)           flush_q <= 1'b1;
      if (acc)                 first_q <= k_tlast;
    end
  end

  always_ff @(posedge clk) begin
    if (acc) begin
      if (first_q) begin
        mode3_q     <= hdr_is3;
        prev_data_q <= {k_tdata[95:0], 32'h0};   // {H2,H1,H0} in lanes 3:1
        prev_keep_q <= 4'hF;
      end else begin
        prev_data_q <= sw_data;
        prev_keep_q <= cur_keep;
      end
    end
  end

  // ---- stage 3: store-and-forward FIFO -------------------------------------------------
  logic [127:0] f_tdata;
  logic [15:0]  f_tkeep;
  logic         f_tlast, f_tvalid, f_tready;
  logic [0:0]   f_tuser_unused;
  logic [$clog2(FIFO_DEPTH):0] f_count;
  logic         f_almost_full;
  logic [15:0]  w_keep_ext;

  always_comb begin
    for (int i = 0; i < LANES; i++) w_keep_ext[4*i +: 4] = {4{w_keep[i]}};
  end

  axis_fifo_sync #(.W(128), .UW(1), .DEPTH(FIFO_DEPTH)) u_out_fifo (
    .clk(clk), .rst(rst),
    .s_tdata(w_data), .s_tkeep(w_keep_ext), .s_tlast(w_last), .s_tuser(1'b0),
    .s_tvalid(w_valid), .s_tready(fifo_rdy),
    .m_tdata(f_tdata), .m_tkeep(f_tkeep), .m_tlast(f_tlast), .m_tuser(f_tuser_unused),
    .m_tvalid(f_tvalid), .m_tready(f_tready),
    .count(f_count), .almost_full(f_almost_full));

  logic [CNT_W-1:0] pkt_cnt_q;      // complete packets stored in the FIFO
  logic             pkt_avail;
  logic             pkt_in, pkt_out;

  assign pkt_avail = (STORE_AND_FORWARD != 1'b0) ? (pkt_cnt_q != '0) : 1'b1;
  assign pkt_in    = w_valid && fifo_rdy && w_last;
  assign pkt_out   = s_axis_tx_tvalid && s_axis_tx_tready && f_tlast;

  always_ff @(posedge clk) begin
    if (rst) pkt_cnt_q <= '0;
    else     pkt_cnt_q <= pkt_cnt_q + CNT_W'(pkt_in) - CNT_W'(pkt_out);
  end

  assign s_axis_tx_tdata  = f_tdata;
  assign s_axis_tx_tkeep  = f_tkeep;
  assign s_axis_tx_tlast  = f_tlast;
  assign s_axis_tx_tvalid = f_tvalid && pkt_avail;
  assign f_tready         = s_axis_tx_tready && pkt_avail;

  logic unused_ok;
  assign unused_ok = &{1'b0, k_tuser, f_tuser_unused, f_count, f_almost_full, tx_buf_av};

endmodule
