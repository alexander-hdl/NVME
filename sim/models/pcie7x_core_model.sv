// =============================================================================
// pcie7x_core_model.sv — behavioural replacement of the Xilinx pcie_7x v3.3 core
//                        (Root Port, Gen2 x4, 128-bit AXI-Stream @ 125 MHz)
//                        as seen from the user side (nvme_host_core).
//
// SIMULATION ONLY.  Reference: research/03_pcie_7x_root_port_brief.md §2-§6
// (PG054 v3.3 Tables 2-9/2-10, 3-4..3-9, 3-12, 3-25, Fig 3-61).
//
// What it does
// ------------
//   TX  (s_axis_tx, host -> SSD): collects AXI beats into TLPs exactly as the
//       7-series core interprets them (128-bit TX never straddles; 3DW header ->
//       payload DW0 sits in DW3 of beat 0; 4DW header -> payload starts in beat 1;
//       payload DWs are big-endian on the bus and are converted here to a byte
//       array in memory order), checks the protocol, and hands each TLP after
//       `tx_latency_cycles` to the SSD model (pcie_sim_pkg::mb_down).
//   RX  (m_axis_rx, SSD -> host): takes TLPs from the SSD model (mb_up) and
//       serialises them in the 128-bit RX format with RANDOMISED but legal
//       framing: SOF at DW0 or DW2, straddling (EOF of packet A at DW0/DW1 and SOF
//       of packet B at DW2 in the same beat), payload misalignment of 3DW headers,
//       is_sof / is_eof encodings in tuser, rx_err_fwd, junk in unused lanes.
//   cfg_mgmt: a register file for the Root Port's own Type-1 config space
//       (Command, bus numbers, PCIe capability at 0x60: DevCap/DevCtl/LinkCap/
//       LinkCtl/LinkSta).  rd_wr_done one cycle after an enable.
//   Root-Port dispositions (PG054 Table 3-12/3-25): if the RP Command.BME = 0,
//       upstream MRd from the SSD are answered by this model itself with Cpl-UR
//       (sent back to the SSD through the tx_cfg_req/tx_cfg_gnt handshake) and
//       upstream MWr are dropped ($warning).  Completions always pass.
//   Link: user_lnk_up after link_up_delay_cycles user_clk cycles from sys_rst_n
//       release; pl_ltssm_state walks Detect/Polling/Config to L0 (0x16);
//       pl_sel_lnk_rate goes to 1 (5 GT/s) gen2_delay_cycles after link-up
//       (with a short Recovery excursion); pl_sel_lnk_width = 2'b10 (x4).
//
// Knobs (module variables; set them hierarchically from the tb at time 0,
//        e.g.  `u_core.straddle_pct = 50;`)
// ------------------------------------------------------------------------
//   tx_stall_pct       0..100  probability (per cycle) that s_axis_tx_tready is low   [0]
//   tx_stall_midpkt    1 = random stalls may also happen in the middle of a packet;
//                      0 = only between packets                                         [1]
//   tx_latency_cycles  cycles from the last accepted TX beat to delivery to the SSD     [16]
//   sof_dw2_pct        0..100  probability that a packet that could start at DW0 is
//                      started at DW2 instead (no straddle, DW0/DW1 empty)              [0]
//   straddle_pct       0..100  probability that, when a packet ends at DW0/DW1 and
//                      another packet is pending, the next one starts at DW2 of the
//                      same beat (straddle)                                             [0]
//   rx_gap_pct         0..100  probability of an idle cycle before starting a packet    [0]
//   rx_midpkt_gap_pct  0..100  probability of tvalid=0 in the middle of a packet        [0]
//   rx_poison_pct      0..100  probability that an upstream MWr/CplD is poisoned
//                      (EP=1, tuser[1] rx_err_fwd=1) — error-path tests only            [0]
//   np_ok_lag          cycles of lag with which rx_np_ok is honoured (PG054: the user
//                      must deassert rx_np_ok >= 2 cycles before it is full)            [2]
//   np_req_mode        1 = rx_np_req credit mode instead of rx_np_ok                    [0]
//   junk_fill          1 = fill unused RX DW lanes with random data (catches DUTs that
//                      look at lanes outside SOF/EOF)                                   [1]
//   reset_until_link   1 = user_reset_out stays asserted until user_lnk_up             [1]
//   force_bme_off      1 = behave as if the RP Command.BME were 0 whatever the user wrote
//                      (negative test: the SSD's DMA must then fail with UR)            [0]
//   verbose            1 = print every TLP                                              [0]
//
// Checks ($error, counted in err_count)
// -------------------------------------
//   TX-1  tkeep on non-last beats != 16'hFFFF
//   TX-2  tkeep on the last beat not in {000F, 00FF, 0FFF, FFFF}
//   TX-3  tvalid deasserted in the middle of a packet (between SOF and tlast)
//   TX-4  AXI: tvalid dropped or tdata/tkeep/tlast changed while tready = 0
//   TX-5  number of DWs in the packet != header DWs + Length (incl. missing tlast)
//   TX-6  payload > Root Port DevCtl.MPS ("malformed", the TLP is dropped)
//   TX-7  MWr crossing a 4 KiB boundary; Cfg request with Length != 1
//   TX-8  tx_err_fwd / tx_src_dsc set, TD or EP bit set in the header
//   RX-1  upstream MWr/CplD payload > RP DevCtl.MPS ("malformed", dropped)
//   RX-2  upstream MWr crossing 4 KiB ("malformed", dropped)
//   RX-3  payload byte count inconsistent with Length in a TLP from the SSD model
//   CM-1  cfg_mgmt_wr_en and cfg_mgmt_rd_en asserted together
// Warnings: BME=0 dispositions (UR / drop), tx_cfg_gnt held low for > 1000 cycles.
// =============================================================================
`timescale 1ns/1ps

module pcie7x_core_model
  import pcie_sim_pkg::*;
#(
  parameter int TX_BUFFERS = 29         // tx_buf_av when idle ("high" performance level, MPS 256)
) (
  input  logic         sys_rst_n,
  // clock / reset / link
  output logic         user_clk_out,
  output logic         user_reset_out,
  output logic         user_lnk_up,
  // RX (core -> user)
  output logic [127:0] m_axis_rx_tdata,
  output logic [15:0]  m_axis_rx_tkeep,
  output logic         m_axis_rx_tlast,
  output logic         m_axis_rx_tvalid,
  input  logic         m_axis_rx_tready,
  output logic [21:0]  m_axis_rx_tuser,
  input  logic         rx_np_ok,
  input  logic         rx_np_req,
  // TX (user -> core)
  input  logic [127:0] s_axis_tx_tdata,
  input  logic [15:0]  s_axis_tx_tkeep,
  input  logic         s_axis_tx_tlast,
  input  logic         s_axis_tx_tvalid,
  output logic         s_axis_tx_tready,
  input  logic [3:0]   s_axis_tx_tuser,
  output logic [5:0]   tx_buf_av,
  output logic         tx_cfg_req,
  input  logic         tx_cfg_gnt,
  // configuration management port (Root Port's own config space)
  input  logic [31:0]  cfg_mgmt_di,
  input  logic [3:0]   cfg_mgmt_byte_en,
  input  logic [9:0]   cfg_mgmt_dwaddr,
  input  logic         cfg_mgmt_wr_en,
  input  logic         cfg_mgmt_rd_en,
  output logic [31:0]  cfg_mgmt_do,
  output logic         cfg_mgmt_rd_wr_done,
  input  logic         cfg_mgmt_wr_readonly,
  input  logic         cfg_mgmt_wr_rw1c_as_rw,
  // physical layer status
  output logic [5:0]   pl_ltssm_state,
  output logic         pl_sel_lnk_rate,
  output logic [1:0]   pl_sel_lnk_width,
  output logic         pl_link_gen2_cap,
  output logic         pl_link_partner_gen2_supported,
  // model-only control
  input  int           link_up_delay_cycles,
  input  int           gen2_delay_cycles
);

  // ---------------------------------------------------------------------------
  // Knobs (see header)
  // ---------------------------------------------------------------------------
  int tx_stall_pct      = 0;
  bit tx_stall_midpkt   = 1;
  int tx_latency_cycles = 16;
  int sof_dw2_pct       = 0;
  int straddle_pct      = 0;
  int rx_gap_pct        = 0;
  int rx_midpkt_gap_pct = 0;
  int rx_poison_pct     = 0;
  int np_ok_lag         = 2;
  bit np_req_mode       = 0;
  bit junk_fill         = 1;
  bit reset_until_link  = 1;
  bit force_bme_off     = 0;
  bit verbose           = 0;

  // ---------------------------------------------------------------------------
  // Statistics / observability for testbenches
  // ---------------------------------------------------------------------------
  int     err_count      = 0;
  longint cyc            = 0;      // user_clk cycle counter (since time 0)
  longint tx_beats_total = 0;      // TX beats accepted (tvalid && tready)
  longint tx_tlps        = 0;
  longint rx_beats_total = 0;
  longint rx_tlps        = 0;
  int     n_straddles    = 0;      // beats that carried an EOF and an SOF
  int     n_sof_dw2      = 0;
  int     n_bme_ur       = 0;      // upstream MRd answered with UR because BME = 0
  int     n_bme_drop     = 0;      // upstream MWr dropped because BME = 0
  logic [45:0] cfgmgmt_wr_log[$];  // one entry per cfg_mgmt write: {be[3:0], dwaddr[9:0], value_after_write[31:0]}

  function automatic void err(input string s);
    err_count++;
    $error("[pcie7x_core_model @%0t] %s", $time, s);
  endfunction

  // ---------------------------------------------------------------------------
  // Clock: 125 MHz user_clk_out
  // ---------------------------------------------------------------------------
  initial begin
    user_clk_out = 1'b0;
    forever #4 user_clk_out = ~user_clk_out;
  end
  wire clk = user_clk_out;
  always @(posedge clk) cyc <= cyc + 1;

  // ---------------------------------------------------------------------------
  // Root Port configuration space (cfg_mgmt).  Dword addresses.
  // ---------------------------------------------------------------------------
  localparam int DW_ID      = 10'h000;
  localparam int DW_CMD     = 10'h001;
  localparam int DW_CLASS   = 10'h002;
  localparam int DW_HDR     = 10'h003;
  localparam int DW_BUSNUM  = 10'h006;
  localparam int DW_CAPPTR  = 10'h00D;
  localparam int DW_PCIECAP = 10'h018;   // 0x60
  localparam int DW_DEVCAP  = 10'h019;   // 0x64
  localparam int DW_DEVCTL  = 10'h01A;   // 0x68
  localparam int DW_LINKCAP = 10'h01B;   // 0x6C
  localparam int DW_LINKCTL = 10'h01C;   // 0x70 (LinkCtl [15:0], LinkSta [31:16])

  logic [31:0] rp_cfg   [0:1023];
  logic [31:0] rp_wmask [0:1023];

  function automatic void rp_cfg_reset();
    for (int i = 0; i < 1024; i++) begin rp_cfg[i] = '0; rp_wmask[i] = 32'hFFFF_FFFF; end
    rp_cfg[DW_ID]      = 32'h7124_10EE;  rp_wmask[DW_ID]      = '0;   // Xilinx VID, a 7-series RP DID
    rp_cfg[DW_CMD]     = 32'h0010_0000;  rp_wmask[DW_CMD]     = 32'h0000_FFFF;  // Status: cap list
    rp_cfg[DW_CLASS]   = 32'h0604_0000;  rp_wmask[DW_CLASS]   = '0;   // PCI-PCI bridge
    rp_cfg[DW_HDR]     = 32'h0001_0000;  rp_wmask[DW_HDR]     = 32'h0000_FFFF;  // header type 1
    rp_cfg[DW_CAPPTR]  = 32'h0000_0060;  rp_wmask[DW_CAPPTR]  = '0;
    rp_cfg[DW_PCIECAP] = 32'h0042_0010;  rp_wmask[DW_PCIECAP] = '0;   // PCIe cap v2, Root Port
    rp_cfg[DW_DEVCAP]  = 32'h0000_8021;  rp_wmask[DW_DEVCAP]  = '0;   // MPSS=1 (256 B), ETFS=1
    rp_cfg[DW_DEVCTL]  = 32'h0000_2810;  rp_wmask[DW_DEVCTL]  = 32'h0000_FFFF;  // MRRS 512, NS, RO; MPS 128
    rp_cfg[DW_LINKCAP] = 32'h0000_0C42;  rp_wmask[DW_LINKCAP] = '0;   // 5 GT/s, x4
    rp_cfg[DW_LINKCTL] = 32'h1041_0000;  rp_wmask[DW_LINKCTL] = 32'h0000_FFFF;  // LinkSta: 2.5 GT/s x4 (until Gen2)
  endfunction

  initial rp_cfg_reset();

  // Convenience accessors for testbenches
  function automatic logic [15:0] rp_command(); return rp_cfg[DW_CMD][15:0];    endfunction
  function automatic logic [15:0] rp_devctl();  return rp_cfg[DW_DEVCTL][15:0]; endfunction
  function automatic bit          rp_bme();     return rp_cfg[DW_CMD][2] && !force_bme_off; endfunction
  function automatic int unsigned rp_mps_bytes(); return 128 << rp_cfg[DW_DEVCTL][7:5]; endfunction

  // cfg_mgmt access: one access per enable; rd_wr_done is a 1-cycle pulse one
  // cycle after the enable is sampled.  Enables seen in the done cycle are ignored
  // (the user deasserts them after seeing done).
  always @(posedge clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
      cfg_mgmt_rd_wr_done <= 1'b0;
      cfg_mgmt_do         <= '0;
      rp_cfg_reset();
    end else begin
      if (cfg_mgmt_rd_wr_done) begin
        cfg_mgmt_rd_wr_done <= 1'b0;
      end else if (cfg_mgmt_wr_en || cfg_mgmt_rd_en) begin
        if (cfg_mgmt_wr_en && cfg_mgmt_rd_en) err("CM-1: cfg_mgmt_wr_en and cfg_mgmt_rd_en asserted together");
        if (cfg_mgmt_wr_en) begin
          logic [31:0] m, v;
          m = cfg_mgmt_wr_readonly ? 32'hFFFF_FFFF : rp_wmask[cfg_mgmt_dwaddr];
          v = rp_cfg[cfg_mgmt_dwaddr];
          for (int b = 0; b < 4; b++)
            if (cfg_mgmt_byte_en[b])
              for (int i = 8*b; i < 8*b+8; i++) if (m[i]) v[i] = cfg_mgmt_di[i];
          rp_cfg[cfg_mgmt_dwaddr] = v;
          cfgmgmt_wr_log.push_back({cfg_mgmt_byte_en, cfg_mgmt_dwaddr, v});
          if (verbose) $display("[core_model] cfg_mgmt WR dw 0x%03h be %b data %08h -> %08h",
                                cfg_mgmt_dwaddr, cfg_mgmt_byte_en, cfg_mgmt_di, v);
        end
        cfg_mgmt_do         <= rp_cfg[cfg_mgmt_dwaddr];
        cfg_mgmt_rd_wr_done <= 1'b1;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Reset and link model
  // ---------------------------------------------------------------------------
  int  rst_cnt, lnk_cnt, g2_cnt, rec_cnt;
  initial begin
    user_reset_out = 1'b1; user_lnk_up = 1'b0; pl_ltssm_state = 6'h00; pl_sel_lnk_rate = 1'b0;
  end
  assign pl_sel_lnk_width               = 2'b10;   // x4
  assign pl_link_gen2_cap               = 1'b1;
  assign pl_link_partner_gen2_supported = user_lnk_up;

  always @(posedge clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
      user_reset_out  <= 1'b1;
      user_lnk_up     <= 1'b0;
      pl_ltssm_state  <= 6'h00;    // Detect.Quiet
      pl_sel_lnk_rate <= 1'b0;
      rst_cnt <= 0; lnk_cnt <= 0; g2_cnt <= 0; rec_cnt <= 0;
    end else begin
      if (rst_cnt < 16) rst_cnt <= rst_cnt + 1;
      if (!user_lnk_up) begin
        lnk_cnt <= lnk_cnt + 1;
        // a plausible LTSSM walk: Detect.Active -> Polling.Active -> Config -> L0
        if      (lnk_cnt < link_up_delay_cycles/4)   pl_ltssm_state <= 6'h01;
        else if (lnk_cnt < link_up_delay_cycles/2)   pl_ltssm_state <= 6'h02;
        else                                         pl_ltssm_state <= 6'h07;
        if (lnk_cnt + 1 >= link_up_delay_cycles) begin
          user_lnk_up    <= 1'b1;
          pl_ltssm_state <= 6'h16;   // L0
        end
      end else if (!pl_sel_lnk_rate) begin
        g2_cnt <= g2_cnt + 1;
        if (g2_cnt + 1 >= gen2_delay_cycles) begin
          // speed change through Recovery (user_lnk_up stays high)
          rec_cnt <= rec_cnt + 1;
          pl_ltssm_state <= 6'h0D;
          if (rec_cnt >= 8) begin
            pl_sel_lnk_rate <= 1'b1;
            pl_ltssm_state  <= 6'h16;
            rp_cfg[DW_LINKCTL][31:16] <= 16'h1042;   // LinkSta: 5 GT/s x4
          end
        end
      end
      if (rst_cnt >= 16 && (!reset_until_link || user_lnk_up)) user_reset_out <= 1'b0;
    end
  end

  // ===========================================================================
  // TX path: s_axis_tx -> TLP -> SSD model
  // ===========================================================================
  typedef struct { tlp_t t; longint due; } txd_t;
  txd_t        tx_dq[$];           // TLPs accepted, waiting for tx_latency_cycles (models core buffers)
  tlp_t        int_q[$];           // TLPs generated by the RP itself (Cpl-UR), waiting for tx_cfg_gnt
  logic [31:0] tx_dws[$];
  int unsigned tx_exp_dws;
  bit          tx_in_pkt;
  bit          tx_prev_stall;
  logic [127:0] tx_prev_data; logic [15:0] tx_prev_keep; logic tx_prev_last;
  int          int_grant;
  int          gnt_wait;

  initial begin
    s_axis_tx_tready = 1'b0; tx_cfg_req = 1'b0; tx_buf_av = 6'(TX_BUFFERS);
    tx_in_pkt = 0; tx_prev_stall = 0; int_grant = 0; gnt_wait = 0;
  end

  function automatic void tx_finish_packet();
    tlp_t        t;
    int unsigned hd, n;
    logic [31:0] h0;
    h0 = tx_dws[0];
    hd = hdr_dws(h0);
    t.hdr[3] = '0;
    for (int i = 0; i < hd; i++) t.hdr[i] = tx_dws[i];
    for (int i = hd; i < tx_dws.size(); i++) begin
      t.data.push_back(tx_dws[i][31:24]);   // big-endian payload DW: lowest address in [31:24]
      t.data.push_back(tx_dws[i][23:16]);
      t.data.push_back(tx_dws[i][15:8]);
      t.data.push_back(tx_dws[i][7:0]);
    end
    t.poison    = h0[14];
    t.acc_cycle = cyc;
    t.acc_beats = tx_beats_total;
    tx_tlps++;
    if (verbose) $display("[core_model] TX  %s", tlp_str(t));
    // ---- checks ----
    if (tx_dws.size() != tx_exp_dws)
      err($sformatf("TX-5: %s: packet has %0d DWs, header+Length says %0d", tlp_name(h0), tx_dws.size(), tx_exp_dws));
    if (h0[15]) err($sformatf("TX-8: %s with TD=1 (no ECRC expected)", tlp_name(h0)));
    if (h0[14]) err($sformatf("TX-8: %s with EP=1 (poisoned TLP transmitted)", tlp_name(h0)));
    if (is_cfg0(h0) && h0[9:0] != 10'd1) err("TX-7: Cfg request with Length != 1");
    if (is_mwr(h0)) begin
      logic [63:0] a; a = req_addr(t);
      if ((a & 64'hFFF) + t.data.size() > 4096) err($sformatf("TX-7: MWr @%h len %0d crosses 4 KiB", a, t.data.size()));
    end
    if (t.data.size() > rp_mps_bytes()) begin
      err($sformatf("TX-6: malformed: %s payload %0d B > RP DevCtl.MPS %0d B (dropped)",
                    tlp_name(h0), t.data.size(), rp_mps_bytes()));
      return;
    end
    begin
      txd_t d; d.t = t; d.due = cyc + tx_latency_cycles;
      tx_dq.push_back(d);
    end
  endfunction

  always @(posedge clk) begin
    if (user_reset_out) begin
      s_axis_tx_tready <= 1'b0;
      tx_in_pkt     = 0;
      tx_prev_stall = 0;
      tx_dws.delete();
    end else begin
      bit nxt;
      // ---- AXI stability while stalled (TX-4) ----
      if (tx_prev_stall) begin
        if (!s_axis_tx_tvalid)
          err("TX-4: s_axis_tx_tvalid dropped while tready was low");
        else if (s_axis_tx_tdata !== tx_prev_data || s_axis_tx_tkeep !== tx_prev_keep || s_axis_tx_tlast !== tx_prev_last)
          err("TX-4: s_axis_tx data/keep/last changed while tready was low");
      end
      // ---- tvalid gap inside a packet (TX-3) ----
      if (tx_in_pkt && !s_axis_tx_tvalid)
        err("TX-3: s_axis_tx_tvalid deasserted in the middle of a packet");
      // ---- accept a beat ----
      if (s_axis_tx_tvalid && s_axis_tx_tready) begin
        tx_beats_total++;
        if (!tx_in_pkt) begin
          tx_dws.delete();
          tx_exp_dws = hdr_dws(s_axis_tx_tdata[31:0]) + len_dw(s_axis_tx_tdata[31:0]);
          if (s_axis_tx_tuser[1]) err("TX-8: tx_err_fwd (tuser[1]) set");
          if (s_axis_tx_tuser[3]) err("TX-8: tx_src_dsc (tuser[3]) set");
        end
        if (!s_axis_tx_tlast && s_axis_tx_tkeep !== 16'hFFFF)
          err($sformatf("TX-1: tkeep=%h on a non-last beat (must be FFFF)", s_axis_tx_tkeep));
        if (s_axis_tx_tlast && !(s_axis_tx_tkeep inside {16'h000F, 16'h00FF, 16'h0FFF, 16'hFFFF}))
          err($sformatf("TX-2: tkeep=%h on the last beat", s_axis_tx_tkeep));
        for (int i = 0; i < 4; i++)
          if (s_axis_tx_tkeep[4*i +: 4] == 4'hF) tx_dws.push_back(s_axis_tx_tdata[32*i +: 32]);
        if (s_axis_tx_tlast) begin
          tx_finish_packet();
          tx_in_pkt = 0;
        end else begin
          tx_in_pkt = 1;
          if (tx_dws.size() >= tx_exp_dws)
            err($sformatf("TX-5: %s: all %0d DWs received but tlast not asserted",
                          tlp_name(tx_dws[0]), tx_exp_dws));
        end
      end
      tx_prev_stall = s_axis_tx_tvalid && !s_axis_tx_tready;
      tx_prev_data  = s_axis_tx_tdata; tx_prev_keep = s_axis_tx_tkeep; tx_prev_last = s_axis_tx_tlast;

      // ---- deliver accepted TLPs to the SSD after the latency ----
      while (tx_dq.size() != 0 && tx_dq[0].due <= cyc) begin
        txd_t d; d = tx_dq.pop_front();
        void'(mb_down.try_put(d.t));
      end

      // ---- next tready ----
      nxt = user_lnk_up;
      if (!tx_in_pkt && tx_dq.size() >= TX_BUFFERS) nxt = 0;              // no free buffer
      if ((tx_stall_midpkt || !tx_in_pkt) && ($urandom_range(99) < tx_stall_pct)) nxt = 0;
      // RP-internal TLP (Cpl-UR) insertion through tx_cfg_req / tx_cfg_gnt
      if (int_grant > 0) begin
        nxt = 0;
        int_grant--;
        if (int_grant == 0) begin
          txd_t d; d.t = int_q.pop_front(); d.due = cyc + tx_latency_cycles;
          tx_dq.push_back(d);
        end
      end else if (int_q.size() != 0) begin
        if (tx_cfg_gnt && !tx_in_pkt) begin
          int_grant = 2; nxt = 0; gnt_wait = 0;
        end else if (!tx_cfg_gnt) begin
          gnt_wait++;
          if (gnt_wait == 1000) $warning("[pcie7x_core_model] tx_cfg_gnt held low for 1000 cycles with tx_cfg_req=1");
        end
      end
      s_axis_tx_tready <= nxt;
      tx_cfg_req       <= (int_q.size() != 0);
      tx_buf_av        <= 6'((tx_dq.size() >= TX_BUFFERS) ? 0 : TX_BUFFERS - tx_dq.size());
    end
  end

  // ===========================================================================
  // RX path: SSD model TLPs -> m_axis_rx (128-bit, straddle, SOF DW0/DW2)
  // ===========================================================================
  tlp_t        rx_q[$];            // upstream TLPs waiting to be framed
  logic [31:0] rx_cur[$];          // remaining bus-order DWs of the packet in progress
  bit          rx_cur_active;
  bit          rx_cur_poison;
  logic [15:0] np_ok_hist;         // rx_np_ok history (bit 0 = this cycle)
  int          np_credits;

  initial begin
    m_axis_rx_tvalid = 1'b0; m_axis_rx_tdata = '0; m_axis_rx_tkeep = '0; m_axis_rx_tuser = '0;
    rx_cur_active = 0; np_ok_hist = '1; np_credits = 0;
  end
  assign m_axis_rx_tlast = 1'b0;   // tied low in 128-bit mode

  // Upstream intake: checks and Root Port dispositions
  function automatic void rx_intake(input tlp_t t);
    logic [31:0] h0;
    h0 = t.hdr[0];
    if (t.data.size() != 4 * len_dw(h0)) begin
      err($sformatf("RX-3: %s has %0d payload bytes but Length says %0d DW", tlp_name(h0), t.data.size(), len_dw(h0)));
      return;
    end
    if (t.data.size() > rp_mps_bytes()) begin
      err($sformatf("RX-1: malformed upstream %s: payload %0d B > RP DevCtl.MPS %0d B (dropped; is the RP MPS programmed?)",
                    tlp_name(h0), t.data.size(), rp_mps_bytes()));
      return;
    end
    if (is_mwr(h0)) begin
      logic [63:0] a; a = req_addr(t);
      if ((a & 64'hFFF) + t.data.size() > 4096) begin
        err($sformatf("RX-2: malformed upstream MWr @%h len %0d crosses 4 KiB (dropped)", a, t.data.size()));
        return;
      end
    end
    if ((is_mrd(h0) || is_mwr(h0)) && !rp_bme()) begin
      if (is_mrd(h0)) begin
        tlp_t        c;
        logic [127:0] hc;
        logic [63:0]  ra;
        ra = req_addr(t);
        n_bme_ur++;
        $warning("[pcie7x_core_model @%0t] Root Port Command.BME=0: upstream MRd @%h answered with Cpl-UR",
                 $time, req_addr(t));
        hc = nvme_host_pkg::hdr_cpl(1'b0, nvme_host_pkg::RP_ID, nvme_host_pkg::CPL_UR,
                                    12'(4 * len_dw({8'h40, h0[23:0]})), t.hdr[1][31:16], t.hdr[1][15:8],
                                    ra[6:0], 10'd0);
        c.hdr[0] = hc[31:0]; c.hdr[1] = hc[63:32]; c.hdr[2] = hc[95:64]; c.hdr[3] = '0;
        c.hdr[0][22:20] = h0[22:20]; c.hdr[0][13:12] = h0[13:12];   // TC/Attr copied
        c.poison = 0;
        int_q.push_back(c);
      end else begin
        n_bme_drop++;
        $warning("[pcie7x_core_model @%0t] Root Port Command.BME=0: upstream MWr @%h dropped", $time, req_addr(t));
      end
      return;
    end
    if (is_cfg0(h0)) begin
      $warning("[pcie7x_core_model] upstream CfgType0 request: RP answers UR internally (dropped by model)");
      return;
    end
    if (h0[28:27] == 2'b10) begin   // Msg / MsgD: not routed to AXI-S by default
      if (verbose) $display("[core_model] upstream Message dropped (not routed): %s", tlp_str(t));
      return;
    end
    if ((is_mwr(h0) || is_cpl(h0)) && t.data.size() != 0 && $urandom_range(99) < rx_poison_pct) begin
      t.poison = 1;
      t.hdr[0][14] = 1'b1;
      $display("[core_model @%0t] poison injected into upstream %s", $time, tlp_name(h0));
    end
    rx_q.push_back(t);
  endfunction

  // Pick the next upstream TLP honouring rx_np_ok (posted / completions may bypass blocked NP)
  function automatic bit rx_pick(output tlp_t t);
    bit np_allowed;
    np_allowed = np_req_mode ? (np_credits > 0) : np_ok_hist[np_ok_lag];
    for (int i = 0; i < rx_q.size(); i++) begin
      if (is_np(rx_q[i].hdr[0]) && !np_allowed) continue;
      t = rx_q[i];
      rx_q.delete(i);
      if (is_np(t.hdr[0]) && np_req_mode) np_credits--;
      return 1;
    end
    return 0;
  endfunction

  // Start a packet: load its bus-order DWs into rx_cur
  function automatic void rx_load(input tlp_t t);
    int unsigned hd;
    hd = hdr_dws(t.hdr[0]);
    rx_cur.delete();
    for (int i = 0; i < hd; i++) rx_cur.push_back(t.hdr[i]);
    for (int j = 0; j + 3 < t.data.size(); j += 4)
      rx_cur.push_back(bytes_to_bus_dw(t.data[j], t.data[j+1], t.data[j+2], t.data[j+3]));
    rx_cur_active = 1;
    rx_cur_poison = t.poison;
    rx_tlps++;
    if (verbose) $display("[core_model] RX  %s", tlp_str(t));
  endfunction

  // Build one 128-bit RX beat. Returns 0 if there is nothing to send this cycle.
  function automatic bit rx_build(output logic [127:0] d, output logic [15:0] k, output logic [21:0] u);
    int   slot;
    bit   any, sof, sof2, eof, pois;
    int   eof_dw, pos;
    tlp_t t;
    for (int i = 0; i < 4; i++) d[32*i +: 32] = junk_fill ? $urandom : 32'h0;
    k = '0; u = '0; any = 0; sof = 0; sof2 = 0; eof = 0; eof_dw = 0; pois = 0; slot = 0;
    // 1) continue the packet in progress from DW0
    if (rx_cur_active) begin
      if ($urandom_range(99) < rx_midpkt_gap_pct) return 0;
      for (int i = 0; i < 4 && rx_cur.size() != 0; i++) begin
        d[32*i +: 32] = rx_cur.pop_front();
        k[4*i +: 4]   = 4'hF;
        slot = i + 1;
      end
      any  = 1;
      pois = rx_cur_poison;
      if (rx_cur.size() == 0) begin
        eof = 1; eof_dw = slot - 1; rx_cur_active = 0;
      end
    end
    // 2) possibly start a new packet in the same beat (DW0, or DW2 = straddle / SOF@DW2)
    if (!rx_cur_active && slot <= 2) begin
      bit try_start;
      try_start = 0;
      if (slot == 0) begin
        if ($urandom_range(99) >= rx_gap_pct) begin
          try_start = 1;
          pos = ($urandom_range(99) < sof_dw2_pct) ? 2 : 0;
        end
      end else begin
        if ($urandom_range(99) < straddle_pct) begin try_start = 1; pos = 2; end
      end
      if (try_start && rx_pick(t)) begin
        rx_load(t);
        sof = 1; sof2 = (pos == 2); pois = t.poison;   // err_fwd applies to the starting packet
        if (slot != 0) n_straddles++;
        if (sof2) n_sof_dw2++;
        for (int i = pos; i < 4 && rx_cur.size() != 0; i++) begin
          d[32*i +: 32] = rx_cur.pop_front();
          k[4*i +: 4]   = 4'hF;
          if (rx_cur.size() == 0) begin
            eof = 1; eof_dw = i; rx_cur_active = 0;
          end
        end
        any = 1;
      end
    end
    if (!any) return 0;
    u[0]     = 1'b0;                                       // rx_ecrc_err
    u[1]     = pois;                                       // rx_err_fwd
    u[9:2]   = 8'h00;                                      // bar hit: none in RP mode
    u[14:10] = sof ? (sof2 ? 5'b11000 : 5'b10000) : 5'b00000;
    u[21:17] = eof ? {1'b1, 2'(eof_dw), 2'b11} : 5'b00000;
    return 1;
  endfunction

  always @(posedge clk) begin
    if (user_reset_out) begin
      m_axis_rx_tvalid <= 1'b0;
      np_ok_hist       = '1;
    end else begin
      tlp_t t;
      np_ok_hist = {np_ok_hist[14:0], rx_np_ok};
      if (rx_np_req) np_credits++;
      // Intake from the SSD model (unbounded: the real core has a large RX buffer)
      while (mb_up.try_get(t)) rx_intake(t);
      if (m_axis_rx_tvalid && m_axis_rx_tready) rx_beats_total++;
      if (!m_axis_rx_tvalid || m_axis_rx_tready) begin
        logic [127:0] d; logic [15:0] k; logic [21:0] u;
        if (user_lnk_up && rx_build(d, k, u)) begin
          m_axis_rx_tdata  <= d;
          m_axis_rx_tkeep  <= k;
          m_axis_rx_tuser  <= u;
          m_axis_rx_tvalid <= 1'b1;
        end else begin
          m_axis_rx_tvalid <= 1'b0;
          if (junk_fill) m_axis_rx_tdata <= {$urandom, $urandom, $urandom, $urandom};
        end
      end
    end
  end

  // Idle indicator for testbenches: nothing queued anywhere in the model
  function automatic bit idle();
    return (rx_q.size() == 0) && !rx_cur_active && (tx_dq.size() == 0) && (int_q.size() == 0) && !tx_in_pkt;
  endfunction

endmodule
