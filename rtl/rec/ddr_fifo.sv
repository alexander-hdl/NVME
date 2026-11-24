// =============================================================================
// ddr_fifo.sv - AXIS128 FIFO whose storage is a window of the DDR3 behind the MIG
//               native app interface (256 bit, BL8, 4:1).
//
// Domain : ui_clk only (`clk`); `rst` synchronous active 1; `clear` is a 1-cycle
//          functional clear (not a reset).
// Spec   : STAND/RECORDER_SPEC.md 4.4 "WP4 DDR", 5.1, 5.4, M10.
//
//   s_axis(128) -> pack 2x128->256 -> staging RAM 256x512 (16 KiB)
//        -> [whole batches of 64 BL8 write commands = 2 KiB] -> DDR window
//        -> [whole batches of 64 BL8 read commands]  -> output RAM 256x512 (16 KiB)
//        -> unpack 256->2x128 -> m_axis(128)
//   Beat order: the first 128-bit beat is bits [127:0] of the 256-bit word.
//   Only complete batches are ever moved: a tail shorter than 2 KiB stays in the
//   staging RAM (all session volumes are multiples of 2 KiB, spec 4.4).
//
// DDR window: byte 0 .. size_mib MiB - 1, circular. app_addr is counted in 32-bit
//   words, BL8 -> +8 per command (1 command = 32 bytes). Window end = size_mib << 18.
//   size_mib is sampled only while the module is empty (`idle`=1) - change it only
//   in IDLE of the session; clear also resets the address pointers to 0.
//
// Arbitration (M10): at a batch boundary alternate write/read when both have a full
//   batch; otherwise serve the side that is ready. Write batch is ready when staging
//   holds >= 64 words and the window has a free batch; read batch when the window
//   holds >= 1 batch and the output RAM has room for 64 more words (room is reserved
//   when the read batch starts, so read data returning from the MIG can never overflow).
//
// MIG handshake: command and write data are issued in the SAME cycle, and only in a
//   cycle in which both app_rdy and app_wdf_rdy are 1 (app_en / app_wdf_wren are
//   combinationally gated by those inputs); therefore "hold until ready" cannot be
//   violated and a clear never leaves a command or write data dangling.  Read data of
//   the MIG has no back pressure; it is written straight into the output RAM.
//
// Counters (all cleared by `clear`; stream level, i.e. at s_/m_ ports):
//   wr_bytes   bytes accepted at s_axis,   rd_bytes   bytes delivered at m_axis,
//   fill_bytes = wr_bytes - rd_bytes (everything inside: staging + DDR + output RAM),
//   fill_max_bytes = maximum fill_bytes since clear.
// space_ok (registered): window has >= 8 KiB free (batch units) AND staging has
//   >= 8 KiB free AND calibrated AND not clearing. The framer decides per block (4 KiB)
//   at block start; a block admitted with space_ok=1 always fits (s_tready stays 1).
//
// clear: unconditional. Command issue stops at once, the module waits for all read
//   responses still in flight in the MIG (they are dropped), then zeroes pointers,
//   counters and the maximum. <= ~100 cycles (rd latency + <= 64 in-flight reads).
//   While clearing s_tready = 0, m_tvalid = 0 (also in the cycle of `clear`), `space_ok` = 0.
//   Beats accepted at s_axis in the cycle of `clear` are discarded.
//
// rst is the MIG's own reset only (ui_clk_sync_rst): a reset of the FIFO alone would lose the reads still in flight in the MIG
//   (rd_infl_q 0 -> -1).  A system reset (RESET key) must be brought in as `clear` (drain), see ax7a200b_rec_top.
//   Level `clear` is allowed (held while the key is down): it keeps the engine stopped and the counters zero.
// clear also aborts a running/pending memtest (ddr_memtest drains its own reads in flight, then mt_busy=0,
//   mt_done stays 0); the FIFO engine gets the MIG port back only after that.
// Memtest: mt_start (1-cycle pulse) waits (mt_busy=1) until the FIFO engine is between
//   batches with nothing in flight, then ddr_memtest owns the MIG port. It DESTROYS the DDR
//   contents; use it only when the FIFO is empty (idle=1). mt_done is sticky.
// =============================================================================
module ddr_fifo (
  input  logic          clk,                      // ui
  input  logic          rst,                      // ui
  input  logic          clear,                    // ui: pulse
  input  logic [10:0]   size_mib,                 // ui: 1..1024 (0 is treated as 1)
  // input stream
  input  logic [127:0]  s_tdata,                  // ui
  input  logic          s_tvalid,                 // ui
  output logic          s_tready,                 // ui
  // output stream
  output logic [127:0]  m_tdata,                  // ui
  output logic          m_tvalid,                 // ui
  input  logic          m_tready,                 // ui
  // status
  output logic          space_ok,                 // ui
  output logic [30:0]   fill_bytes,               // ui
  output logic [30:0]   fill_max_bytes,           // ui
  output logic          idle,                     // ui
  output logic [47:0]   wr_bytes,                 // ui
  output logic [47:0]   rd_bytes,                 // ui
  // memtest
  input  logic          mt_start,                 // ui: pulse
  input  logic [10:0]   mt_mib,                   // ui
  output logic          mt_busy,                  // ui
  output logic          mt_done,                  // ui: sticky
  output logic [31:0]   mt_err_cnt,               // ui
  output logic [28:0]   mt_first_err_addr,        // ui
  // MIG native interface (ui)
  output logic [28:0]   app_addr,
  output logic [2:0]    app_cmd,
  output logic          app_en,
  input  logic          app_rdy,
  output logic [255:0]  app_wdf_data,
  output logic [31:0]   app_wdf_mask,
  output logic          app_wdf_wren,
  output logic          app_wdf_end,
  input  logic          app_wdf_rdy,
  input  logic [255:0]  app_rd_data,
  input  logic          app_rd_data_valid,
  input  logic          app_rd_data_end,
  output logic          app_sr_req,
  output logic          app_ref_req,
  output logic          app_zq_req,
  input  logic          init_calib_complete
);

  localparam int          RAM_DEPTH  = 512;                 // 256-bit words per RAM
  localparam int          BATCH_CMDS = 64;                  // BL8 commands per batch (2 KiB)
  localparam int          SPACE_B    = 4;                   // 8 KiB in batches
  localparam int          SPACE_W    = 256;                 // 8 KiB in staging words (keep this many free)

  typedef enum logic [1:0] {ST_IDLE, ST_WR, ST_RD} eng_e;

  assign app_wdf_mask = 32'h0;
  assign app_sr_req   = 1'b0;
  assign app_ref_req  = 1'b0;
  assign app_zq_req   = 1'b0;

  // ---------------------------------------------------------------------------
  // state
  // ---------------------------------------------------------------------------
  eng_e         st_q;
  logic [6:0]   left_q;                 // commands left in the current batch
  logic         last_rd_q;              // last batch was a read
  logic         clr_busy_q;
  logic         mt_pend_q;

  logic [28:0]  wa_q, ra_q;             // next write / read command address
  logic [19:0]  ddr_b_q;                // complete batches in the DDR window
  logic [19:0]  win_b_q;                // window size in batches
  logic [28:0]  win_end_q;              // window size in words
  logic [9:0]   rd_infl_q;              // read commands accepted, response not yet received

  // input side
  logic [127:0] half_q;
  logic         hv_q;
  logic [8:0]   s_wp_q, s_rp_q;
  logic [9:0]   s_occ_q;                // words in staging RAM not yet fetched
  logic         wdv_q;                  // staging RAM output holds the next write word

  // output side
  logic [8:0]   o_wp_q, o_rp_q;
  logic [9:0]   o_occ_q;                // words in output RAM not yet fetched
  logic [9:0]   o_rsv_q;                // words reserved: in the RAM + still in flight
  logic         owv_q;                  // output RAM latch holds the word being unpacked
  logic         h_q;                    // second half of that word is next

  logic [27:0]  fill_q, fillmax_q;      // beats (16 B)
  logic [43:0]  wr_beats_q, rd_beats_q;
  logic         idle_q;

  logic [255:0] stg_rd_data, out_rd_data;

  // ---------------------------------------------------------------------------
  // combinational
  // ---------------------------------------------------------------------------
  logic mt_run, eng_en;
  logic         mt_busy_i, mt_done_i;
  logic acc_w, acc_r, left_last;
  logic s_acc, s_wr_ram, fetch_s;
  logic o_resp, o_wr, fetch_o, o_hs, o_consume;
  logic pick_ok, w_ok, r_ok, pick_wr, pick_rd;
  logic [9:0]  s_occ_nx, o_occ_nx, o_rsv_nx, rd_infl_nx;
  logic [10:0] s_avail;
  logic [28:0] wa_nx, ra_nx;
  logic [10:0] size_eff;
  logic        clr_fin;

  assign mt_run  = mt_busy_i;
  assign eng_en  = !rst && !clr_busy_q && !clear && !mt_run && init_calib_complete;   // !rst: no command in a reset cycle (acc_r would not be counted)

  assign acc_w     = (st_q == ST_WR) && eng_en && wdv_q && app_rdy && app_wdf_rdy;
  assign acc_r     = (st_q == ST_RD) && eng_en && app_rdy;
  assign left_last = (left_q == 7'd1);

  assign s_tready  = !clr_busy_q && (!hv_q || (s_occ_q < 10'(RAM_DEPTH)));
  assign s_acc     = s_tvalid && s_tready;
  assign s_wr_ram  = s_acc && hv_q;
  assign fetch_s   = (!wdv_q || acc_w) && (s_occ_q != 10'd0) && !clr_busy_q;
  assign s_occ_nx  = s_occ_q + 10'(s_wr_ram) - 10'(fetch_s);

  assign o_resp     = app_rd_data_valid && !mt_run;
  assign o_wr       = o_resp && !clr_busy_q;
  assign m_tvalid   = owv_q && !clr_busy_q && !clear;
  assign o_hs       = m_tvalid && m_tready;
  assign o_consume  = o_hs && h_q;
  assign fetch_o    = (!owv_q || o_consume) && (o_occ_q != 10'd0) && !clr_busy_q;
  assign o_occ_nx   = o_occ_q + 10'(o_wr) - 10'(fetch_o);
  assign m_tdata    = h_q ? out_rd_data[255:128] : out_rd_data[127:0];

  assign rd_infl_nx = rd_infl_q + 10'(acc_r) - 10'(o_resp);

  assign s_avail = {1'b0, s_occ_q} + 11'(wdv_q);
  assign pick_ok = (st_q == ST_IDLE) && eng_en && !mt_pend_q;
  assign w_ok    = (s_avail >= 11'(BATCH_CMDS)) && (ddr_b_q < win_b_q);
  assign r_ok    = (ddr_b_q != 20'd0) && (o_rsv_q <= 10'(RAM_DEPTH - BATCH_CMDS));
  assign pick_wr = pick_ok && w_ok && (!r_ok || last_rd_q);
  assign pick_rd = pick_ok && r_ok && (!w_ok || !last_rd_q);
  assign o_rsv_nx = o_rsv_q + (pick_rd ? 10'(BATCH_CMDS) : 10'd0) - 10'(fetch_o);

  assign wa_nx = wa_q + 29'd8;
  assign ra_nx = ra_q + 29'd8;
  assign size_eff = (size_mib == 11'd0) ? 11'd1 : size_mib;

  assign clr_fin = clr_busy_q && (rd_infl_nx == 10'd0);

  // ---------------------------------------------------------------------------
  // RAMs (1-cycle read, output holds until the next read)
  // ---------------------------------------------------------------------------
  sdp_ram #(.DEPTH(RAM_DEPTH), .W(256), .OUT_REG(1'b0)) u_sdp_ram_stg (
    .clk(clk), .wr_en(s_wr_ram), .wr_be({32{1'b1}}), .wr_addr(s_wp_q), .wr_data({s_tdata, half_q}),
    .rd_en(fetch_s), .rd_addr(s_rp_q), .rd_data(stg_rd_data)
  );

  sdp_ram #(.DEPTH(RAM_DEPTH), .W(256), .OUT_REG(1'b0)) u_sdp_ram_out (
    .clk(clk), .wr_en(o_wr), .wr_be({32{1'b1}}), .wr_addr(o_wp_q), .wr_data(app_rd_data),
    .rd_en(fetch_o), .rd_addr(o_rp_q), .rd_data(out_rd_data)
  );

  // ---------------------------------------------------------------------------
  // memtest and MIG port multiplexer
  // ---------------------------------------------------------------------------
  logic         mt_go;
  logic [28:0]  mt_app_addr;
  logic [2:0]   mt_app_cmd;
  logic         mt_app_en, mt_app_wren, mt_app_end;
  logic [255:0] mt_app_wdata;

  assign mt_go = mt_pend_q && (st_q == ST_IDLE) && (rd_infl_q == 10'd0) && !clr_busy_q && !clear;

  ddr_memtest u_ddr_memtest (
    .clk(clk), .rst(rst), .start(mt_go), .abort(clear), .mib(mt_mib), .init_calib_complete(init_calib_complete),
    .busy(mt_busy_i), .done(mt_done_i), .err_cnt(mt_err_cnt), .first_err_addr(mt_first_err_addr),
    .app_addr(mt_app_addr), .app_cmd(mt_app_cmd), .app_en(mt_app_en), .app_rdy(app_rdy),
    .app_wdf_data(mt_app_wdata), .app_wdf_wren(mt_app_wren), .app_wdf_end(mt_app_end),
    .app_wdf_rdy(app_wdf_rdy), .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid)
  );

  assign mt_busy     = mt_busy_i || mt_pend_q;
  assign mt_done     = mt_done_i && !mt_pend_q;

  assign app_addr     = mt_run ? mt_app_addr  : ((st_q == ST_RD) ? ra_q : wa_q);
  assign app_cmd      = mt_run ? mt_app_cmd   : (acc_r ? 3'b001 : 3'b000);
  assign app_en       = mt_run ? mt_app_en    : (acc_w | acc_r);
  assign app_wdf_data = mt_run ? mt_app_wdata : stg_rd_data;
  assign app_wdf_wren = mt_run ? mt_app_wren  : acc_w;
  assign app_wdf_end  = mt_run ? mt_app_end   : acc_w;

  // ---------------------------------------------------------------------------
  // output stream and status
  // ---------------------------------------------------------------------------
  logic [31:0] fill_b32, fillmax_b32;
  assign fill_b32       = {fill_q, 4'b0};
  assign fillmax_b32    = {fillmax_q, 4'b0};
  assign fill_bytes     = fill_b32[30:0];
  assign fill_max_bytes = fillmax_b32[30:0];
  assign wr_bytes       = {wr_beats_q, 4'b0};
  assign rd_bytes       = {rd_beats_q, 4'b0};
  assign idle           = idle_q;

  logic empty_c;
  assign empty_c = (s_occ_q == 10'd0) && !wdv_q && !hv_q && (ddr_b_q == 20'd0) && (o_rsv_q == 10'd0)
                   && !owv_q && (st_q == ST_IDLE) && (rd_infl_q == 10'd0) && !clr_busy_q
                   && !mt_pend_q && !mt_busy_i;

  logic [27:0] fill_nx;
  assign fill_nx = fill_q + 28'(s_acc) - 28'(o_hs);

  always_ff @(posedge clk) begin
    if (rst) begin
      st_q       <= ST_IDLE;
      left_q     <= 7'd0;
      last_rd_q  <= 1'b0;
      clr_busy_q <= 1'b0;
      mt_pend_q  <= 1'b0;
      wa_q       <= 29'd0;
      ra_q       <= 29'd0;
      ddr_b_q    <= 20'd0;
      win_b_q    <= 20'd512;
      win_end_q  <= 29'h0004_0000;
      rd_infl_q  <= 10'd0;
      hv_q       <= 1'b0;
      s_wp_q     <= 9'd0;
      s_rp_q     <= 9'd0;
      s_occ_q    <= 10'd0;
      wdv_q      <= 1'b0;
      o_wp_q     <= 9'd0;
      o_rp_q     <= 9'd0;
      o_occ_q    <= 10'd0;
      o_rsv_q    <= 10'd0;
      owv_q      <= 1'b0;
      h_q        <= 1'b0;
      fill_q     <= 28'd0;
      fillmax_q  <= 28'd0;
      wr_beats_q <= 44'd0;
      rd_beats_q <= 44'd0;
      idle_q     <= 1'b1;
      space_ok   <= 1'b0;
    end else begin
      idle_q <= empty_c;
      // window size is only sampled while empty
      if (idle_q) begin
        win_b_q   <= {size_eff, 9'd0};
        win_end_q <= {size_eff, 18'd0};
      end

      space_ok <= init_calib_complete && !clr_busy_q && !clear && !mt_run
                  && (({1'b0, ddr_b_q} + 21'(SPACE_B)) <= {1'b0, win_b_q})
                  && (({1'b0, s_occ_q} + 11'(wdv_q) + 11'(hv_q)) <= 11'(SPACE_W));

      // ---- memtest request ----
      if (clear)                           mt_pend_q <= 1'b0;
      else if (mt_start && init_calib_complete) mt_pend_q <= 1'b1;
      else if (mt_go)                      mt_pend_q <= 1'b0;

      // ---- input packer + staging ----
      if (s_acc) begin
        if (!hv_q) begin
          half_q <= s_tdata;
          hv_q   <= 1'b1;
        end else begin
          hv_q   <= 1'b0;
          s_wp_q <= s_wp_q + 9'd1;
        end
      end
      if (fetch_s) s_rp_q <= s_rp_q + 9'd1;
      s_occ_q <= s_occ_nx;
      if (fetch_s)    wdv_q <= 1'b1;
      else if (acc_w) wdv_q <= 1'b0;

      // ---- engine ----
      case (st_q)
        ST_IDLE: begin
          if (pick_wr) begin
            st_q   <= ST_WR;
            left_q <= 7'(BATCH_CMDS);
          end else if (pick_rd) begin
            st_q   <= ST_RD;
            left_q <= 7'(BATCH_CMDS);
            ddr_b_q <= ddr_b_q - 20'd1;
          end
        end
        ST_WR: begin
          if (acc_w) begin
            wa_q   <= (wa_nx >= win_end_q) ? 29'd0 : wa_nx;
            left_q <= left_q - 7'd1;
            if (left_last) begin
              st_q      <= ST_IDLE;
              ddr_b_q   <= ddr_b_q + 20'd1;
              last_rd_q <= 1'b0;
            end
          end
        end
        ST_RD: begin
          if (acc_r) begin
            ra_q   <= (ra_nx >= win_end_q) ? 29'd0 : ra_nx;
            left_q <= left_q - 7'd1;
            if (left_last) begin
              st_q      <= ST_IDLE;
              last_rd_q <= 1'b1;
            end
          end
        end
        default: st_q <= ST_IDLE;
      endcase
      rd_infl_q <= rd_infl_nx;

      // ---- output side ----
      if (o_wr) o_wp_q <= o_wp_q + 9'd1;
      if (fetch_o) o_rp_q <= o_rp_q + 9'd1;
      o_occ_q <= o_occ_nx;
      o_rsv_q <= o_rsv_nx;
      if (fetch_o) begin
        owv_q <= 1'b1;
        h_q   <= 1'b0;
      end else if (o_hs) begin
        if (h_q) begin owv_q <= 1'b0; h_q <= 1'b0; end
        else     h_q <= 1'b1;
      end

      // ---- counters ----
      fill_q <= fill_nx;
      if (fill_nx > fillmax_q) fillmax_q <= fill_nx;
      if (s_acc) wr_beats_q <= wr_beats_q + 44'd1;
      if (o_hs)  rd_beats_q <= rd_beats_q + 44'd1;

      // ---- clear ----
      if (clear && !clr_busy_q) clr_busy_q <= 1'b1;
      if (clr_fin) begin
        clr_busy_q <= 1'b0;
        st_q       <= ST_IDLE;
        left_q     <= 7'd0;
        last_rd_q  <= 1'b0;
        wa_q       <= 29'd0;
        ra_q       <= 29'd0;
        ddr_b_q    <= 20'd0;
        hv_q       <= 1'b0;
        s_wp_q     <= 9'd0;
        s_rp_q     <= 9'd0;
        s_occ_q    <= 10'd0;
        wdv_q      <= 1'b0;
        o_wp_q     <= 9'd0;
        o_rp_q     <= 9'd0;
        o_occ_q    <= 10'd0;
        o_rsv_q    <= 10'd0;
        owv_q      <= 1'b0;
        h_q        <= 1'b0;
        fill_q     <= 28'd0;
        fillmax_q  <= 28'd0;
        wr_beats_q <= 44'd0;
        rd_beats_q <= 44'd0;
      end
    end
  end

endmodule
