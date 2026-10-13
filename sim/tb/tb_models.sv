// =============================================================================
// tb_models.sv — self-test of the two simulation models, back to back, WITHOUT the DUT
//
//   [tiny behavioural host in this tb] <-AXI-S/cfg_mgmt-> pcie7x_core_model <-TLP-> nvme_ssd_model
//
// Purpose: prove that the models are correct before they are used against the DUT,
// so that model bugs are never confused with DUT bugs.
//
// The tiny host contains its own, independent implementation of
//   * the 128-bit RX de-framer (is_sof/is_eof decoding, straddle, SOF at DW2,
//     big-endian payload DWs) — it checks every framing rule the core model must obey;
//   * the 128-bit TX framer (3DW: {D0,H2,H1,H0}; tkeep rules);
//   * a host-memory completer (sparse byte map `hmem`) answering the SSD's MRds with
//     RANDOMLY split but legal CplDs (split at RCB=64 B or MPS boundaries, BC/LA
//     correct, completions of different tags interleaved);
//   * a requester for config / MMIO requests (CRS retry).
//
// Flow: reset/link -> RP cfg_mgmt (Command, bus numbers, DevCtl MPS=256/ETE) ->
// enumeration (VID/DID with CRS retries, class, BAR0 sizing + assignment, capability
// walk, DevCtl MPS/MRRS/ETE, Command 0x0406) -> NVMe init (CAP/VS checks, AQA (4-entry
// admin queues so they wrap), ASQ/ACQ, CC, RDY) -> Identify Controller + Namespace (data
// checked) -> Set Features -> Create CQ/SQ (4 entries: the I/O CQ wraps, phase flips) ->
// invalid-opcode admin command (expects SC=01h) -> I/O Writes: 8 KiB (PRP2 = page),
// 32 KiB (PRP list at a non-zero offset), 12 KiB with PRP1 offset and a CHAINED PRP list,
// then 12 x 4 KiB writes with up to 3 outstanding (CQE out-of-order allowed) ->
// lba_mem checked with check_range -> Flush -> Delete SQ, Delete CQ -> CC.SHN -> SHST.
// Randomisation (on by default): straddle / SOF@DW2 / RX gaps, TX tready stalls,
// m_axis_rx_tready stalls, rx_np_ok toggling, CQE split / coalesce, random MRd sizes,
// PRP-list chunking, concurrent commands.
//
// Plusargs
//   +calm         disable all randomised stalls/framing (straight-line run)
//   +verbose      models print every TLP
//   +neg=N / +negN NEGATIVE self-checks: corrupt something on purpose and PASS only if
//                 the models flag it with $error:
//                   1 = host completer returns a wrong Byte Count       (SSD check D-6)
//                   2 = host completer returns a wrong Lower Address    (SSD check D-7)
//                   3 = host completer splits at a non-RCB address      (SSD check D-8)
//                   4 = host completer returns a CplD > MPS             (core TX-6 malformed)
//                   5 = Root Port BME left 0 -> core answers SQE fetch with UR (SSD D-2)
//                   6 = SSD Command.BME left 0 -> SSD flags DMA without BME (SSD D-1)
//                   7 = host writes a TX packet with a bad tkeep        (core TX-1)
//                   8 = host rings a doorbell beyond the queue size     (SSD N-7)
// Ends with "TEST PASSED" or $fatal.
// =============================================================================
`timescale 1ns/1ps

module tb_models;
  import pcie_sim_pkg::*;

  // ---------------------------------------------------------------------------
  // Constants of the tiny host
  // ---------------------------------------------------------------------------
  localparam logic [15:0] SSD_BDF  = 16'h0100;
  localparam logic [31:0] BAR0     = 32'hC000_0000;
  localparam logic [31:0] ASQ      = 32'h1000_0000;
  localparam logic [31:0] ACQ      = 32'h1000_1000;
  localparam logic [31:0] IDENT    = 32'h1000_2000;
  localparam logic [31:0] IOSQ     = 32'h1000_3000;
  localparam logic [31:0] IOCQ     = 32'h1000_4000;
  localparam int          AQ_SIZE  = 4;      // admin queues: 4 entries (wrap quickly)
  localparam int          IOQ_SIZE = 4;      // I/O queues: 4 entries
  localparam int          HOST_MPS = 256;
  localparam int          RCB      = 64;

  // ---------------------------------------------------------------------------
  // DUT-side signals (here: driven by the tiny host)
  // ---------------------------------------------------------------------------
  logic         sys_rst_n;
  wire          clk, user_reset, user_lnk_up;
  wire  [127:0] rx_tdata;  wire [15:0] rx_tkeep; wire rx_tlast, rx_tvalid; logic rx_tready;
  wire  [21:0]  rx_tuser;
  logic         rx_np_ok, rx_np_req;
  logic [127:0] tx_tdata;  logic [15:0] tx_tkeep; logic tx_tlast, tx_tvalid; wire tx_tready;
  logic [3:0]   tx_tuser;
  wire  [5:0]   tx_buf_av; wire tx_cfg_req; logic tx_cfg_gnt;
  logic [31:0]  cm_di; logic [3:0] cm_be; logic [9:0] cm_addr; logic cm_wr, cm_rd;
  wire  [31:0]  cm_do; wire cm_done;
  wire  [5:0]   ltssm; wire lnk_rate; wire [1:0] lnk_width; wire g2cap, g2partner;

  pcie7x_core_model u_core (
    .sys_rst_n(sys_rst_n), .user_clk_out(clk), .user_reset_out(user_reset), .user_lnk_up(user_lnk_up),
    .m_axis_rx_tdata(rx_tdata), .m_axis_rx_tkeep(rx_tkeep), .m_axis_rx_tlast(rx_tlast),
    .m_axis_rx_tvalid(rx_tvalid), .m_axis_rx_tready(rx_tready), .m_axis_rx_tuser(rx_tuser),
    .rx_np_ok(rx_np_ok), .rx_np_req(rx_np_req),
    .s_axis_tx_tdata(tx_tdata), .s_axis_tx_tkeep(tx_tkeep), .s_axis_tx_tlast(tx_tlast),
    .s_axis_tx_tvalid(tx_tvalid), .s_axis_tx_tready(tx_tready), .s_axis_tx_tuser(tx_tuser),
    .tx_buf_av(tx_buf_av), .tx_cfg_req(tx_cfg_req), .tx_cfg_gnt(tx_cfg_gnt),
    .cfg_mgmt_di(cm_di), .cfg_mgmt_byte_en(cm_be), .cfg_mgmt_dwaddr(cm_addr),
    .cfg_mgmt_wr_en(cm_wr), .cfg_mgmt_rd_en(cm_rd), .cfg_mgmt_do(cm_do), .cfg_mgmt_rd_wr_done(cm_done),
    .cfg_mgmt_wr_readonly(1'b0), .cfg_mgmt_wr_rw1c_as_rw(1'b0),
    .pl_ltssm_state(ltssm), .pl_sel_lnk_rate(lnk_rate), .pl_sel_lnk_width(lnk_width),
    .pl_link_gen2_cap(g2cap), .pl_link_partner_gen2_supported(g2partner),
    .link_up_delay_cycles(300), .gen2_delay_cycles(200));

  nvme_ssd_model #(.NSZE(64'd1048576)) u_ssd (.clk(clk), .perst_n(sys_rst_n));

  int  tb_errs = 0;
  int  neg     = 0;
  bit  calm    = 0;

  function automatic void tb_err(input string s);
    tb_errs++;
    $error("[tb_models @%0t] %s", $time, s);
  endfunction

  // ---------------------------------------------------------------------------
  // Host memory (sparse)
  // ---------------------------------------------------------------------------
  byte unsigned hmem [longint unsigned];

  function automatic byte unsigned hrd8(input longint unsigned a);
    return hmem.exists(a) ? hmem[a] : 8'h00;
  endfunction
  function automatic logic [31:0] hrd32(input longint unsigned a);
    return {hrd8(a+3), hrd8(a+2), hrd8(a+1), hrd8(a)};
  endfunction
  function automatic void hwr32(input longint unsigned a, input logic [31:0] v);
    for (int i = 0; i < 4; i++) hmem[a+i] = v[8*i +: 8];
  endfunction
  function automatic void hwr64(input longint unsigned a, input logic [63:0] v);
    hwr32(a, v[31:0]); hwr32(a+4, v[63:32]);
  endfunction

  // ---------------------------------------------------------------------------
  // TX framer: txq of TLPs -> s_axis_tx beats
  // ---------------------------------------------------------------------------
  tlp_t         txq[$];
  logic [127:0] cur_d[$];
  logic [15:0]  cur_k[$];
  bit           cur_l[$];
  int           bad_tkeep_once = 0;

  function automatic void tx_load(input tlp_t t);
    logic [31:0] dws[$];
    int unsigned n, hd;
    hd = hdr_dws(t.hdr[0]);
    for (int i = 0; i < hd; i++) dws.push_back(t.hdr[i]);
    for (int j = 0; j < t.data.size(); j += 4) dws.push_back(bytes_to_bus_dw(t.data[j], t.data[j+1], t.data[j+2], t.data[j+3]));
    n = dws.size();
    for (int b = 0; b < (n + 3) / 4; b++) begin
      logic [127:0] d; logic [15:0] k;
      d = '0; k = '0;
      for (int i = 0; i < 4; i++) if (4*b + i < n) begin d[32*i +: 32] = dws[4*b + i]; k[4*i +: 4] = 4'hF; end
      if (bad_tkeep_once == 1 && b == 0 && n > 4) begin k = 16'h0FFF; bad_tkeep_once = 2; end   // neg=7
      cur_d.push_back(d); cur_k.push_back(k); cur_l.push_back(b == (n + 3) / 4 - 1);
    end
  endfunction

  always @(posedge clk) begin
    if (user_reset) begin
      tx_tvalid <= 1'b0; tx_tdata <= '0; tx_tkeep <= '0; tx_tlast <= 1'b0;
      cur_d.delete(); cur_k.delete(); cur_l.delete();
    end else begin
      if (tx_tvalid && tx_tready) begin void'(cur_d.pop_front()); void'(cur_k.pop_front()); void'(cur_l.pop_front()); end
      if (cur_d.size() == 0 && txq.size() != 0) tx_load(txq.pop_front());
      if (cur_d.size() != 0) begin
        tx_tvalid <= 1'b1; tx_tdata <= cur_d[0]; tx_tkeep <= cur_k[0]; tx_tlast <= cur_l[0];
      end else begin
        tx_tvalid <= 1'b0;
      end
    end
  end
  assign tx_tuser = 4'b0000;

  // ---------------------------------------------------------------------------
  // RX de-framer (independent implementation of PG054 128-bit RX rules)
  // ---------------------------------------------------------------------------
  logic [31:0] rp_dws[$];
  bit          rp_in = 0;
  bit          rp_poison;
  int          n_rx_tlps = 0, n_rx_straddle = 0, n_rx_sof2 = 0;

  task automatic rx_finish();
    tlp_t        t;
    int unsigned hd;
    hd = hdr_dws(rp_dws[0]);
    if (rp_dws.size() != hd + len_dw(rp_dws[0]))
      tb_err($sformatf("RX framing: %s has %0d DWs, expected %0d", tlp_name(rp_dws[0]), rp_dws.size(), hd + len_dw(rp_dws[0])));
    t.hdr[3] = '0;
    for (int i = 0; i < hd; i++) t.hdr[i] = rp_dws[i];
    for (int i = hd; i < rp_dws.size(); i++) begin
      t.data.push_back(rp_dws[i][31:24]); t.data.push_back(rp_dws[i][23:16]);
      t.data.push_back(rp_dws[i][15:8]);  t.data.push_back(rp_dws[i][7:0]);
    end
    t.poison = rp_poison;
    if (rp_poison != t.hdr[0][14]) tb_err("RX: rx_err_fwd does not match the EP bit");
    n_rx_tlps++;
    host_rx(t);
  endtask

  always @(posedge clk) begin
    if (rx_tvalid && rx_tready) begin
      bit sof, sof2, eof; int edw;
      sof  = rx_tuser[14]; sof2 = rx_tuser[13]; eof = rx_tuser[21]; edw = rx_tuser[20:19];
      if (sof  && (rx_tuser[12:10] != 3'b000 || rx_tuser[11])) tb_err($sformatf("RX: bad is_sof encoding %b", rx_tuser[14:10]));
      if (eof  && rx_tuser[18:17] != 2'b11) tb_err($sformatf("RX: bad is_eof encoding %b", rx_tuser[21:17]));
      if (!sof && rx_tuser[13:10] != 0) tb_err("RX: is_sof[3:0] set without is_sof[4]");
      if (rx_tlast) tb_err("RX: m_axis_rx_tlast must be tied low in 128-bit mode");
      if (rp_in) begin
        if (sof && !sof2) tb_err("RX: SOF at DW0 while a packet is in progress");
        if (sof && !eof)  tb_err("RX: SOF at DW2 while a packet is in progress and no EOF in this beat");
        if (sof && eof && edw > 1) tb_err("RX: straddle with EOF at DW2/DW3");
        for (int i = 0; i <= (eof ? edw : 3); i++) begin
          if (rx_tkeep[4*i +: 4] != 4'hF) tb_err("RX: tkeep low on a used DW");
          rp_dws.push_back(rx_tdata[32*i +: 32]);
        end
        if (eof) begin rx_finish(); rp_in = 0; end
        if (sof) begin
          n_rx_straddle++;
          rp_dws.delete(); rp_poison = rx_tuser[1];
          rp_dws.push_back(rx_tdata[95:64]); rp_dws.push_back(rx_tdata[127:96]);
          rp_in = 1;
        end
      end else begin
        int s;
        if (!sof) tb_err("RX: beat without SOF while no packet is in progress");
        else begin
          s = sof2 ? 2 : 0;
          if (sof2) n_rx_sof2++;
          rp_dws.delete(); rp_poison = rx_tuser[1];
          if (eof) begin
            if (edw < s + 2) tb_err("RX: packet shorter than 3 DW (SOF/EOF in one beat)");
            for (int i = s; i <= edw; i++) rp_dws.push_back(rx_tdata[32*i +: 32]);
            rx_finish();
          end else begin
            for (int i = s; i < 4; i++) rp_dws.push_back(rx_tdata[32*i +: 32]);
            rp_in = 1;
          end
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Host dispositions of received TLPs: completer (MRd), memory (MWr), requester (Cpl)
  // ---------------------------------------------------------------------------
  class resp_c; tlp_t q[$]; endclass
  resp_c active[$];
  tlp_t  hcpl [256];
  bit    hcpl_v [256];
  int    n_host_mrd = 0, n_host_mwr = 0, n_host_cpld = 0;
  int    neg_done = 0;

  // Choose the size of the next CplD of a request: legal ends are RCB-aligned addresses in
  // (cur, cur+MPS], or the end of the request if it is within cur+MPS.
  function automatic int unsigned pick_chunk(input longint unsigned cur, input int unsigned rem);
    longint unsigned lim, e;
    int unsigned     nopt;
    lim = cur + HOST_MPS;
    if (cur + rem <= lim && (calm || $urandom_range(1))) return rem;
    e = ((cur + rem < lim) ? cur + rem : lim) / RCB * RCB;        // highest RCB boundary
    if (e <= cur) return rem;
    nopt = (e - (cur / RCB * RCB)) / RCB;                          // RCB boundaries in (cur, e]
    if (!calm && nopt > 1) e = e - RCB * $urandom_range(0, nopt - 1);
    if (e <= cur) e = e + RCB;
    return e - cur;
  endfunction

  task automatic host_rx(input tlp_t t);
    logic [31:0] h0;
    h0 = t.hdr[0];
    if (is_mwr(h0)) begin
      longint unsigned a;
      a = req_addr(t);
      n_host_mwr++;
      foreach (t.data[i]) hmem[a + i] = t.data[i];
    end else if (is_mrd(h0)) begin
      resp_c           r;
      longint unsigned a, cur;
      int unsigned     len, rem, n;
      n_host_mrd++;
      r   = new();
      a   = req_addr(t);
      len = 4 * ((h0[9:0] == 0) ? 1024 : h0[9:0]);
      cur = a; rem = len;
      while (rem > 0) begin
        tlp_t         c;
        logic [127:0] hc;
        logic [11:0]  bc;
        logic [6:0]   la;
        n  = pick_chunk(cur, rem);
        bc = 12'(rem); la = cur[6:0];
        // ---- negative self-checks of the SSD model ----
        if (neg == 1 && neg_done == 0 && rem != n) begin bc = bc + 4; neg_done = 1; end
        if (neg == 2 && neg_done == 0 && rem != n) begin la = la + 4; neg_done = 1; end
        if (neg == 3 && neg_done == 0 && rem > 64 && (cur % RCB) == 0) begin n = 60; neg_done = 1; end
        if (neg == 4 && neg_done == 0 && rem > HOST_MPS) begin n = rem; neg_done = 1; end
        hc = nvme_host_pkg::hdr_cpl(1'b1, 16'h0000, nvme_host_pkg::CPL_SC, bc, t.hdr[1][31:16], t.hdr[1][15:8], la, 10'(n / 4));
        c.hdr[0] = hc[31:0]; c.hdr[1] = hc[63:32]; c.hdr[2] = hc[95:64]; c.hdr[3] = '0; c.poison = 0;
        for (int i = 0; i < n; i++) c.data.push_back(hrd8(cur + i));
        r.q.push_back(c);
        cur += n; rem -= n;
      end
      active.push_back(r);
    end else if (is_cpl(h0)) begin
      int tag;
      tag = t.hdr[2][15:8];
      n_host_cpld++;
      if (hcpl_v[tag]) tb_err($sformatf("host: duplicate completion for tag %0d", tag));
      hcpl[tag] = t; hcpl_v[tag] = 1;
    end else begin
      tb_err($sformatf("host: unexpected TLP %s", tlp_str(t)));
    end
  endtask

  // Completion scheduler: keeps txq short, interleaves completions of up to 3 requests
  always @(posedge clk) begin
    if (!user_reset && active.size() != 0 && txq.size() < 2) begin
      int i;
      i = calm ? 0 : $urandom_range((active.size() > 3 ? 3 : active.size()) - 1);
      txq.push_back(active[i].q.pop_front());
      if (active[i].q.size() == 0) active.delete(i);
    end
  end

  // Random back-pressure on the RX side and rx_np_ok toggling
  always @(posedge clk) begin
    rx_tready <= calm ? 1'b1 : ($urandom_range(99) < 80);
    rx_np_ok  <= calm ? 1'b1 : ($urandom_range(99) < 85);
  end
  assign rx_np_req  = 1'b0;
  assign tx_cfg_gnt = 1'b1;

  // ---------------------------------------------------------------------------
  // Host requester tasks
  // ---------------------------------------------------------------------------
  logic [7:0] htag = 0;
  int         crs_seen = 0;

  function automatic tlp_t mk_tlp(input logic [127:0] h, input byte unsigned d[$]);
    tlp_t t;
    t.hdr[0] = h[31:0]; t.hdr[1] = h[63:32]; t.hdr[2] = h[95:64]; t.hdr[3] = h[127:96];
    t.data.delete();
    foreach (d[i]) t.data.push_back(d[i]);   // element-wise: xsim 2022.2 loses `t.data = d`
    t.poison = 0;
    return t;
  endfunction

  task automatic host_np(input tlp_t t, output tlp_t c);
    int tag, w;
    tag = htag; htag++;
    t.hdr[1][15:8] = tag;
    hcpl_v[tag] = 0;
    txq.push_back(t);
    w = 0;
    while (!hcpl_v[tag]) begin
      @(posedge clk);
      if (++w > 200000) $fatal(1, "[tb_models] no completion for host request %s", tlp_str(t));
    end
    c = hcpl[tag];
    if (c.hdr[1][31:16] != ((tlp_name(t.hdr[0]) == "CfgRd0" || tlp_name(t.hdr[0]) == "CfgWr0") ? SSD_BDF : u_ssd.bdf))
      tb_err($sformatf("host: completer ID %04h unexpected", c.hdr[1][31:16]));
  endtask

  task automatic cfg_rd(input logic [11:0] r, output logic [31:0] v);
    tlp_t t, c;
    byte unsigned d[$];
    forever begin
      t = mk_tlp(nvme_host_pkg::hdr_cfg0(1'b0, SSD_BDF, 10'(r >> 2), 16'h0000, 8'h0, 4'hF), d);
      host_np(t, c);
      if (c.hdr[1][15:13] == nvme_host_pkg::CPL_CRS) begin crs_seen++; repeat (50) @(posedge clk); continue; end
      if (c.hdr[1][15:13] != nvme_host_pkg::CPL_SC || c.data.size() != 4) begin
        tb_err($sformatf("CfgRd 0x%03h failed: %s", r, tlp_str(c))); v = 'x; return;
      end
      v = le32(c.data, 0);
      return;
    end
  endtask

  task automatic cfg_wr(input logic [11:0] r, input logic [31:0] v, input logic [3:0] be);
    tlp_t t, c;
    byte unsigned d[$];
    d.delete(); d.push_back(v[7:0]); d.push_back(v[15:8]); d.push_back(v[23:16]); d.push_back(v[31:24]);
    forever begin
      t = mk_tlp(nvme_host_pkg::hdr_cfg0(1'b1, SSD_BDF, 10'(r >> 2), 16'h0000, 8'h0, be), d);
      host_np(t, c);
      if (c.hdr[1][15:13] == nvme_host_pkg::CPL_CRS) begin crs_seen++; repeat (50) @(posedge clk); continue; end
      if (c.hdr[1][15:13] != nvme_host_pkg::CPL_SC || c.hdr[0][30]) tb_err($sformatf("CfgWr 0x%03h failed: %s", r, tlp_str(c)));
      return;
    end
  endtask

  task automatic mmio_rd(input logic [15:0] off, output logic [31:0] v);
    tlp_t t, c;
    byte unsigned d[$];
    t = mk_tlp(nvme_host_pkg::hdr_mrd32(BAR0 + off, 10'd1, 16'h0000, 8'h0, 4'hF, 4'h0), d);
    host_np(t, c);
    if (c.hdr[1][15:13] != 0 || c.data.size() != 4 || c.hdr[1][11:0] != 4 || c.hdr[2][6:0] != off[6:0])
      tb_err($sformatf("MMIO read 0x%04h: bad completion %s", off, tlp_str(c)));
    v = le32(c.data, 0);
  endtask

  task automatic mmio_wr(input logic [15:0] off, input logic [31:0] v);
    byte unsigned d[$];
    d.delete(); d.push_back(v[7:0]); d.push_back(v[15:8]); d.push_back(v[23:16]); d.push_back(v[31:24]);
    txq.push_back(mk_tlp(nvme_host_pkg::hdr_mwr32(BAR0 + off, 10'd1, 16'h0000, 8'h0, 4'hF, 4'h0), d));
  endtask

  task automatic cm_access(input bit wr, input logic [9:0] a, input logic [31:0] v, input logic [3:0] be,
                           output logic [31:0] rv);
    @(posedge clk);
    cm_addr <= a; cm_di <= v; cm_be <= be; cm_wr <= wr; cm_rd <= !wr;
    do @(posedge clk); while (!cm_done);
    rv = cm_do;
    cm_wr <= 1'b0; cm_rd <= 1'b0;
  endtask

  // ---------------------------------------------------------------------------
  // NVMe queue helpers
  // ---------------------------------------------------------------------------
  int asq_tail = 0, acq_head = 0; bit acq_phase = 1;
  int iosq_tail = 0, iocq_head = 0; bit iocq_phase = 1;

  task automatic wait_cqe(input logic [31:0] base, input int head, input bit ph, output logic [31:0] cqe[4]);
    int w;
    logic [31:0] d3;
    w = 0;
    forever begin
      d3 = hrd32(base + 16*head + 12);
      if (d3[16] === ph) break;
      @(posedge clk);
      if (++w > 400000) $fatal(1, "[tb_models] timeout waiting for CQE at %h[%0d] phase %0d", base, head, ph);
    end
    for (int i = 0; i < 4; i++) cqe[i] = hrd32(base + 16*head + 4*i);
  endtask

  task automatic admin_cmd(input logic [31:0] s[16], output logic [31:0] cqe[4]);
    for (int i = 0; i < 16; i++) hwr32(ASQ + 64*asq_tail + 4*i, s[i]);
    asq_tail = (asq_tail + 1) % AQ_SIZE;
    mmio_wr(16'h1000, asq_tail);
    wait_cqe(ACQ, acq_head, acq_phase, cqe);
    if (cqe[3][15:0] != s[0][31:16]) tb_err($sformatf("admin CQE CID %0d, expected %0d", cqe[3][15:0], s[0][31:16]));
    if (cqe[2][15:0] != asq_tail)    tb_err($sformatf("admin CQE SQHD %0d, expected %0d", cqe[2][15:0], asq_tail));
    if (cqe[2][31:16] != 0)          tb_err("admin CQE SQID != 0");
    acq_head++;
    if (acq_head == AQ_SIZE) begin acq_head = 0; acq_phase = !acq_phase; end
    mmio_wr(16'h1004, acq_head);
  endtask

  function automatic void sqe_clear(output logic [31:0] s[16]);
    for (int i = 0; i < 16; i++) s[i] = 0;
  endfunction

  task automatic io_submit(input logic [31:0] s[16]);
    for (int i = 0; i < 16; i++) hwr32(IOSQ + 64*iosq_tail + 4*i, s[i]);
    iosq_tail = (iosq_tail + 1) % IOQ_SIZE;
  endtask

  task automatic io_reap(output logic [31:0] cqe[4]);
    wait_cqe(IOCQ, iocq_head, iocq_phase, cqe);
    if (cqe[2][31:16] != 1) tb_err("I/O CQE SQID != 1");
    iocq_head++;
    if (iocq_head == IOQ_SIZE) begin iocq_head = 0; iocq_phase = !iocq_phase; end
    mmio_wr(16'h100C, iocq_head);
  endtask

  // Build a Write SQE
  function automatic void mk_write(output logic [31:0] s[16], input logic [15:0] cid, input longint unsigned slba,
                                   input int nlb, input logic [63:0] prp1, input logic [63:0] prp2);
    sqe_clear(s);
    s[0] = {cid, 8'h00, 8'h01}; s[1] = 1;
    s[6] = prp1[31:0]; s[7] = prp1[63:32]; s[8] = prp2[31:0]; s[9] = prp2[63:32];
    s[10] = slba[31:0]; s[11] = slba[63:32]; s[12] = nlb - 1;
  endfunction

  // Fill `len` bytes of host memory at addr with a pattern and append them to ref
  function automatic void fill_data(input longint unsigned addr, input int unsigned len, input int seed,
                                    ref byte unsigned r[$]);
    for (int unsigned i = 0; i < len; i++) begin
      byte unsigned v;
      v = 8'((i * 7) ^ (i >> 8) ^ seed);
      hmem[addr + i] = v;
      r.push_back(v);
    end
  endfunction

  int total_check_errs = 0;
  task automatic check_lba(input longint unsigned slba, input byte unsigned r[$]);
    byte unsigned ra[];
    int e;
    ra = new[r.size()];
    foreach (r[i]) ra[i] = r[i];
    e = u_ssd.check_range(slba, r.size() / 512, ra);
    if (e != 0) tb_err($sformatf("lba_mem mismatch at SLBA %0d: %0d bytes", slba, e));
    total_check_errs += e;
  endtask

  // ---------------------------------------------------------------------------
  // Negative-test watcher: pass as soon as the models flag the injected error
  // ---------------------------------------------------------------------------
  initial begin
    if (!$value$plusargs("neg=%d", neg))
      // +negN form (xsim.bat on Windows mangles '=' in -testplusarg)
      neg = $test$plusargs("neg1") ? 1 : $test$plusargs("neg2") ? 2 : $test$plusargs("neg3") ? 3 :
            $test$plusargs("neg4") ? 4 : $test$plusargs("neg5") ? 5 : $test$plusargs("neg6") ? 6 :
            $test$plusargs("neg7") ? 7 : $test$plusargs("neg8") ? 8 : 0;
    if (neg != 0) begin
      int base;
      $display("[tb_models] NEGATIVE self-check %0d: the models MUST report an $error", neg);
      wait (sys_rst_n === 1'b1);
      fork
        begin
          wait (u_ssd.err_count + u_core.err_count > 0);
          repeat (10) @(posedge clk);
          $display("[tb_models] negative test %0d: models reported %0d error(s) as expected",
                   neg, u_ssd.err_count + u_core.err_count);
          $display("TEST PASSED");
          $finish;
        end
        begin
          #(3ms);
          $fatal(1, "[tb_models] negative test %0d: the models did NOT flag the injected error", neg);
        end
      join_none
    end
  end

  // ---------------------------------------------------------------------------
  // Main sequence
  // ---------------------------------------------------------------------------
  initial begin
    logic [31:0] v, s[16], cqe[4];
    byte unsigned refA[$], refB[$], refC[$];
    int          cap, outstanding, cids_seen[int];

    calm = $test$plusargs("calm");
    if ($test$plusargs("verbose")) begin u_core.verbose = 1; u_ssd.verbose = 1; end
    if (!calm) begin
      u_core.straddle_pct      = 60;
      u_core.sof_dw2_pct       = 30;
      u_core.rx_gap_pct        = 10;
      u_core.rx_midpkt_gap_pct = 5;
      u_core.tx_stall_pct      = 20;
      u_ssd.cqe_split_pct      = 40;
      u_ssd.cqe_coalesce_pct   = 50;
      u_ssd.concurrent_cmds    = 4;
      u_ssd.max_outstanding    = 16;
      u_ssd.mrd_gap_max        = 3;
    end
    if (neg == 7) bad_tkeep_once = 1;

    sys_rst_n = 1'b0;
    cm_wr = 0; cm_rd = 0; cm_di = 0; cm_be = 0; cm_addr = 0;
    #(500ns);
    sys_rst_n = 1'b1;
    wait (user_lnk_up === 1'b1);
    $display("[tb_models @%0t] link up, LTSSM %02h", $time, ltssm);
    wait (lnk_rate === 1'b1);
    if (ltssm != 6'h16 || lnk_width != 2'b10) tb_err($sformatf("link state ltssm=%02h width=%b", ltssm, lnk_width));
    $display("[tb_models @%0t] Gen2 x4 reached", $time);

    // ---- Root Port own config space via cfg_mgmt ----
    if (neg != 5) cm_access(1, 10'h001, 32'h0000_0006, 4'b0001, v);
    cm_access(1, 10'h006, 32'h0001_0100, 4'b0111, v);
    cm_access(0, 10'h01A, 0, 0, v);
    if (v[15:0] != 16'h2810) tb_err($sformatf("RP DevCtl reset value %04h", v[15:0]));
    v[7:5] = 3'd1; v[8] = 1'b1;
    cm_access(1, 10'h01A, v, 4'b0011, v);
    cm_access(0, 10'h01A, 0, 0, v);
    if (v[7:5] != 1 || !v[8]) tb_err($sformatf("RP DevCtl readback %04h", v[15:0]));
    if (u_core.rp_mps_bytes() != 256) tb_err("RP MPS not 256");

    // ---- Enumeration ----
    cfg_rd(12'h000, v);
    if (v != 32'hA808_144D) tb_err($sformatf("VID/DID %08h", v));
    if (crs_seen != 2) tb_err($sformatf("expected 2 CRS completions, saw %0d", crs_seen));
    cfg_rd(12'h008, v);
    if (v[31:8] != 24'h010802) tb_err($sformatf("class %06h", v[31:8]));
    cfg_wr(12'h010, 32'hFFFF_FFFF, 4'hF); cfg_rd(12'h010, v);
    if (v != 32'hFFFF_C004) tb_err($sformatf("BAR0 sizing read %08h", v));
    cfg_wr(12'h014, 32'hFFFF_FFFF, 4'hF); cfg_rd(12'h014, v);
    if (v != 32'hFFFF_FFFF) tb_err($sformatf("BAR1 sizing read %08h", v));
    cfg_wr(12'h010, BAR0, 4'hF); cfg_wr(12'h014, 0, 4'hF);
    cfg_rd(12'h010, v); if (v != (BAR0 | 4)) tb_err($sformatf("BAR0 readback %08h", v));
    if (u_ssd.bdf != SSD_BDF) tb_err($sformatf("SSD captured BDF %04h", u_ssd.bdf));
    cfg_rd(12'h034, v); cap = v[7:0];
    begin
      int caps[$];
      while (cap != 0) begin
        cfg_rd(12'(cap), v); caps.push_back(cap); caps.push_back(v[7:0]);
        if (v[7:0] == 8'h10) break;
        cap = v[15:8];
      end
      if (caps.size() != 6 || caps[0] != 8'h40 || caps[1] != 1 || caps[2] != 8'h50 || caps[3] != 5 || caps[4] != 8'h80 || caps[5] != 8'h10)
        tb_err($sformatf("capability walk %p", caps));
    end
    cfg_rd(12'(cap + 4), v);
    if (v[2:0] != 3'd2 || !v[5]) tb_err($sformatf("DevCap %08h", v));
    cfg_rd(12'(cap + 8), v);
    if (v[15:0] != 16'h2810) tb_err($sformatf("SSD DevCtl reset %04h", v[15:0]));
    v[7:5] = 3'd1; v[14:12] = 3'd2; v[8] = 1'b1;
    cfg_wr(12'(cap + 8), v, 4'b0011);
    cfg_rd(12'(cap + 8), v);
    if (u_ssd.mps_bytes() != 256 || u_ssd.mrrs_bytes() != 512 || !u_ssd.ete()) tb_err($sformatf("SSD DevCtl %04h", v[15:0]));
    cfg_rd(12'(cap + 12), v); if (v[3:0] != 2 || v[9:4] != 4) tb_err($sformatf("LinkCap %08h", v));
    cfg_rd(12'(cap + 16), v); if (v[19:16] != 2 || v[25:20] != 4) tb_err($sformatf("LinkSta %04h", v[31:16]));
    cfg_wr(12'h004, (neg == 6) ? 32'h0000_0402 : 32'h0000_0406, 4'b0011);
    $display("[tb_models @%0t] enumeration done", $time);

    // ---- NVMe registers ----
    mmio_rd(16'h0000, v); if (v != 32'h0101_03FF) tb_err($sformatf("CAP lo %08h", v));
    mmio_rd(16'h0004, v); if (v != 32'h0000_0020) tb_err($sformatf("CAP hi %08h", v));
    mmio_rd(16'h0008, v); if (v != 32'h0001_0400) tb_err($sformatf("VS %08h", v));
    mmio_rd(16'h0014, v); if (v != 0) tb_err($sformatf("CC %08h", v));
    mmio_rd(16'h001C, v); if (v != 0) tb_err($sformatf("CSTS %08h", v));
    mmio_wr(16'h0024, {4'h0, 12'(AQ_SIZE-1), 4'h0, 12'(AQ_SIZE-1)});
    mmio_wr(16'h0028, ASQ); mmio_wr(16'h002C, 0);
    mmio_wr(16'h0030, ACQ); mmio_wr(16'h0034, 0);
    mmio_wr(16'h0014, 32'h0046_0000);
    mmio_wr(16'h0014, 32'h0046_0001);
    do begin repeat (20) @(posedge clk); mmio_rd(16'h001C, v); end while (!v[0] && !v[1]);
    if (v[1]) $fatal(1, "[tb_models] CSTS.CFS=1");
    $display("[tb_models @%0t] controller ready", $time);
    if (neg == 8) mmio_wr(16'h1000, 32'd7);   // doorbell beyond the 4-entry admin queue

    // ---- Admin commands ----
    sqe_clear(s); s[0] = 32'h0000_0006; s[6] = IDENT; s[10] = 1;            // Identify Controller, CID 0
    admin_cmd(s, cqe);
    if (cqe[3][31:17] != 0) tb_err($sformatf("Identify ctrl status %h", cqe[3][31:17]));
    if (hrd8(IDENT+77) != 5 || hrd8(IDENT+512) != 8'h66 || hrd8(IDENT+513) != 8'h44 || hrd32(IDENT+516) != 1 ||
        hrd8(IDENT+525) != 1 || hrd8(IDENT) != 8'h4D || hrd8(IDENT+1) != 8'h14 || hrd32(IDENT+80) != 32'h0001_0400)
      tb_err("Identify Controller data wrong");
    sqe_clear(s); s[0] = 32'h0001_0006; s[1] = 1; s[6] = IDENT; s[10] = 0;  // Identify Namespace, CID 1
    admin_cmd(s, cqe);
    if (cqe[3][31:17] != 0) tb_err($sformatf("Identify ns status %h", cqe[3][31:17]));
    if (hrd32(IDENT) != 1048576 || hrd32(IDENT+4) != 0 || hrd8(IDENT+25) != 1 || hrd8(IDENT+26) != 0 ||
        hrd32(IDENT+128) != 32'h0009_0000 || hrd32(IDENT+132) != 32'h000C_0000)
      tb_err("Identify Namespace data wrong");
    sqe_clear(s); s[0] = 32'h0002_0009; s[10] = 7; s[11] = 0;               // Set Features NQ
    admin_cmd(s, cqe);
    if (cqe[3][31:17] != 0 || cqe[0] != 0) tb_err("Set Features failed");
    sqe_clear(s); s[0] = 32'h0003_0005; s[6] = IOCQ; s[10] = {16'(IOQ_SIZE-1), 16'd1}; s[11] = 1;   // Create CQ
    admin_cmd(s, cqe);
    if (cqe[3][31:17] != 0) tb_err($sformatf("Create CQ status %h", cqe[3][31:17]));
    sqe_clear(s); s[0] = 32'h0004_0001; s[6] = IOSQ; s[10] = {16'(IOQ_SIZE-1), 16'd1}; s[11] = 32'h0001_0001; // Create SQ
    admin_cmd(s, cqe);
    if (cqe[3][31:17] != 0) tb_err($sformatf("Create SQ status %h", cqe[3][31:17]));
    sqe_clear(s); s[0] = 32'h0005_007F;                                      // invalid opcode
    admin_cmd(s, cqe);
    if (cqe[3][24:17] != 8'h01 || cqe[3][27:25] != 0) tb_err($sformatf("invalid opcode status %h", cqe[3][31:17]));
    $display("[tb_models @%0t] admin init done (admin queue wrapped: head %0d phase %0d)", $time, acq_head, acq_phase);

    // ---- I/O: Write A, 8 KiB, PRP2 = second page ----
    fill_data(64'h4000_0000, 8192, 8'h11, refA);
    mk_write(s, 16'd10, 64'd16, 16, 64'h4000_0000, 64'h4000_1000);
    io_submit(s); mmio_wr(16'h1008, iosq_tail);
    io_reap(cqe);
    if (cqe[3][15:0] != 10 || cqe[3][31:17] != 0) tb_err($sformatf("Write A CQE %h", cqe[3]));
    check_lba(16, refA);
    $display("[tb_models @%0t] Write A (8 KiB, PRP1+PRP2) OK", $time);

    // ---- Write B (32 KiB, PRP list at offset 0x100) + Write C (12 KiB, PRP1 offset 0x200, chained list), one doorbell ----
    fill_data(64'h4001_0000, 32768, 8'h22, refB);
    for (int i = 0; i < 7; i++) hwr64(64'h2000_0100 + 8*i, 64'h4001_1000 + 64'h1000*i);
    mk_write(s, 16'd11, 64'd100, 64, 64'h4001_0000, 64'h2000_0100);
    io_submit(s);
    // C: 12 KiB starting at offset 0x200 -> 4 pages -> 3 list entries; list at 0x2000_0FF0 holds 2 slots:
    //    [0xFF0] = page 1, [0xFF8] = chain pointer -> 0x2000_2000 = { page 2, page 3 }
    fill_data(64'h4003_0200, 12288, 8'h33, refC);
    hwr64(64'h2000_0FF0, 64'h4003_1000);
    hwr64(64'h2000_0FF8, 64'h2000_2000);
    hwr64(64'h2000_2000, 64'h4003_2000);
    hwr64(64'h2000_2008, 64'h4003_3000);
    mk_write(s, 16'd12, 64'd1000, 24, 64'h4003_0200, 64'h2000_0FF0);
    io_submit(s);
    mmio_wr(16'h1008, iosq_tail);
    for (int k = 0; k < 2; k++) begin
      io_reap(cqe);
      if (cqe[3][31:17] != 0) tb_err($sformatf("Write B/C status %h", cqe[3][31:17]));
      cids_seen[cqe[3][15:0]] = 1;
    end
    if (!cids_seen.exists(11) || !cids_seen.exists(12)) tb_err("Write B/C CIDs missing");
    check_lba(100, refB);
    check_lba(1000, refC);
    $display("[tb_models @%0t] Write B (PRP list) and C (offset + chained list) OK", $time);

    // ---- 12 x 4 KiB writes, up to 3 outstanding (queue wraps several times) ----
    begin
      byte unsigned refs[12][$];
      int submitted, reaped;
      submitted = 0; reaped = 0; outstanding = 0;
      cids_seen.delete();
      while (reaped < 12) begin
        while (submitted < 12 && outstanding < IOQ_SIZE - 1) begin
          fill_data(64'h4100_0000 + 64'h1000 * submitted, 4096, submitted, refs[submitted]);
          mk_write(s, 16'(20 + submitted), 64'd4000 + 8 * submitted, 8, 64'h4100_0000 + 64'h1000 * submitted, 0);
          io_submit(s);
          submitted++; outstanding++;
          if ($urandom_range(1) || calm) mmio_wr(16'h1008, iosq_tail);   // sometimes ring once for two
        end
        mmio_wr(16'h1008, iosq_tail);
        io_reap(cqe);
        if (cqe[3][31:17] != 0) tb_err($sformatf("4K write status %h", cqe[3][31:17]));
        if (cids_seen.exists(cqe[3][15:0])) tb_err($sformatf("duplicate CID %0d", cqe[3][15:0]));
        cids_seen[cqe[3][15:0]] = 1;
        reaped++; outstanding--;
      end
      for (int i = 0; i < 12; i++) check_lba(4000 + 8*i, refs[i]);
    end
    $display("[tb_models @%0t] 12 x 4 KiB writes OK (I/O CQ wrapped, phase now %0d)", $time, iocq_phase);

    // ---- Flush ----
    sqe_clear(s); s[0] = {16'd99, 8'h00, 8'h00}; s[1] = 1;
    io_submit(s); mmio_wr(16'h1008, iosq_tail);
    io_reap(cqe);
    if (cqe[3][15:0] != 99 || cqe[3][31:17] != 0) tb_err("Flush CQE");

    // ---- Shutdown ----
    sqe_clear(s); s[0] = 32'h0006_0000; s[10] = 1; admin_cmd(s, cqe);    // Delete SQ 1
    if (cqe[3][31:17] != 0) tb_err("Delete SQ failed");
    sqe_clear(s); s[0] = 32'h0007_0004; s[10] = 1; admin_cmd(s, cqe);    // Delete CQ 1
    if (cqe[3][31:17] != 0) tb_err("Delete CQ failed");
    mmio_wr(16'h0014, 32'h0046_4001);
    do begin repeat (50) @(posedge clk); mmio_rd(16'h001C, v); end while (v[3:2] != 2'b10);
    $display("[tb_models @%0t] shutdown complete (CSTS=%08h)", $time, v);

    repeat (200) @(posedge clk);
    if (!u_core.idle() || !u_ssd.idle()) tb_err("models not idle at the end");
    $display("--------------------------------------------------------------------------");
    $display(" tb_models summary: host saw %0d TLPs on RX (%0d straddled beats, %0d SOF@DW2),",
             n_rx_tlps, n_rx_straddle, n_rx_sof2);
    $display("   core model: TX TLPs %0d, RX TLPs %0d, straddles %0d, sof@dw2 %0d",
             u_core.tx_tlps, u_core.rx_tlps, u_core.n_straddles, u_core.n_sof_dw2);
    $display("   SSD model: MRd %0d, CplD %0d, MWr %0d, writes %0d, flush %0d, CQEs %0d, data %0d B, %0d LBAs",
             u_ssd.n_mrd, u_ssd.n_cpld, u_ssd.n_mwr, u_ssd.n_io_writes, u_ssd.n_flush, u_ssd.n_cqe,
             u_ssd.data_bytes, u_ssd.lba_mem.num());
    $display("   errors: tb %0d, core model %0d, SSD model %0d, data mismatches %0d",
             tb_errs, u_core.err_count, u_ssd.err_count, total_check_errs);
    $display("--------------------------------------------------------------------------");
    if (neg != 0) $fatal(1, "[tb_models] negative test %0d finished without the expected model error", neg);
    if (!calm && (n_rx_straddle == 0 || n_rx_sof2 == 0)) tb_err("randomised framing was not exercised");
    if (tb_errs == 0 && u_core.err_count == 0 && u_ssd.err_count == 0 && total_check_errs == 0)
      $display("TEST PASSED");
    else
      $fatal(1, "TEST FAILED");
    $finish;
  end

  initial begin
    #(20ms);
    $fatal(1, "[tb_models] global timeout");
  end

endmodule
