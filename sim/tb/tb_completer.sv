// =============================================================================
// tb_completer — testbench for hostmem_completer (ARCHITECTURE.md §5.5)
//
// * Memory model: 4 read sources (ASQ/IOSQ/PRP/DATA) implemented as a pure hash
//   function of (sel, byte address); the read port has exactly 2 cycles of
//   latency and returns random garbage when rd_en was not asserted.
// * Stimulus: random MRd requests (1 DW .. 4096 B, DW aligned starts incl.
//   unaligned within a 16-byte beat, random partial First/Last BE, random
//   TC/Attr, 3DW and 4DW headers, unmapped / region-straddling addresses),
//   directed cases (PCIe worked example, zero-length read, every 1-DW BE),
//   MPS 128/256/512/1024, random m_cpl_tready.
// * Checker: reassembles every completion against the request it belongs to
//   (requests are served in order) and verifies Fmt/Type, Status, CplID,
//   ReqID/Tag/TC/Attr, Length, split boundaries (first CplD ends at the next MPS
//   boundary, others MPS-sized, last = tail), Byte Count chain, Lower Address,
//   increasing contiguous addresses, payload data vs. the memory model, tkeep /
//   tlast structure, UR packets for unmapped addresses, and AXI-S stability
//   under back-pressure. np_backpressure must be high whenever s_mrd_tready is low.
// * Measurements: request->first-beat latency, and output beats/cycle over a
//   burst of 64 x 512 B requests (MPS 256) excluding the first 20 cycles.
// =============================================================================
module tb_completer;
  import nvme_host_pkg::*;

  localparam int unsigned N_SLOTS    = 4;
  localparam int unsigned SLOT_BYTES = 16384;
  localparam int unsigned PRP_BYTES  = N_SLOTS * 4096;
  localparam int unsigned DATA_BYTES = N_SLOTS * SLOT_BYTES;
  localparam logic [15:0] COMP_ID    = 16'h5A00;

  logic clk = 1'b0;
  logic rst = 1'b1;
  always #4 clk = ~clk;

  // ---------------------------------------------------------------------------
  // DUT
  // ---------------------------------------------------------------------------
  logic [127:0] s_mrd_tdata;
  logic         s_mrd_tvalid, s_mrd_tready, np_backpressure;
  logic [127:0] m_cpl_tdata;
  logic [15:0]  m_cpl_tkeep;
  logic         m_cpl_tlast, m_cpl_tvalid, m_cpl_tready;
  logic [1:0]   m_cpl_tuser;
  logic [2:0]   mps_code;
  logic [31:0]  rd_addr;
  logic [1:0]   rd_sel;
  logic         rd_en;
  logic [127:0] rd_data;

  hostmem_completer #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES)) dut (
    .clk(clk), .rst(rst),
    .s_mrd_tdata(s_mrd_tdata), .s_mrd_tkeep(16'hFFFF), .s_mrd_tlast(1'b1), .s_mrd_tuser(2'b00),
    .s_mrd_tvalid(s_mrd_tvalid), .s_mrd_tready(s_mrd_tready),
    .np_backpressure(np_backpressure),
    .m_cpl_tdata(m_cpl_tdata), .m_cpl_tkeep(m_cpl_tkeep), .m_cpl_tlast(m_cpl_tlast),
    .m_cpl_tuser(m_cpl_tuser), .m_cpl_tvalid(m_cpl_tvalid), .m_cpl_tready(m_cpl_tready),
    .mps_code(mps_code), .completer_id(COMP_ID),
    .rd_addr(rd_addr), .rd_sel(rd_sel), .rd_en(rd_en), .rd_data(rd_data));

  // ---------------------------------------------------------------------------
  // Memory model
  // ---------------------------------------------------------------------------
  function automatic logic [7:0] mem_byte(input logic [1:0] sel, input logic [31:0] rel);
    logic [31:0] h;
    h = (rel ^ {sel, 30'h1ABC_DEF}) * 32'h9E37_79B1;
    h = h ^ (h >> 15);
    h = h * 32'h85EB_CA6B;
    h = h ^ (h >> 13);
    return h[7:0] ^ h[23:16] ^ rel[7:0];
  endfunction

  function automatic logic [127:0] mem_beat(input logic [1:0] sel, input logic [31:0] rel);
    logic [127:0] r;
    for (int b = 0; b < 16; b++) r[8*b +: 8] = mem_byte(sel, rel + b);
    return r;
  endfunction

  function automatic int unsigned region_size(input logic [1:0] sel);
    case (sel)
      RSEL_ASQ:  return ASQ_BYTES;
      RSEL_IOSQ: return IOSQ_BYTES;
      RSEL_PRP:  return PRP_BYTES;
      default:   return DATA_BYTES;
    endcase
  endfunction

  logic [127:0] src_s1, src_s2;
  int           src_errors = 0;
  assign rd_data = src_s2;

  always @(posedge clk) begin
    if (rd_en === 1'b1) begin
      if (rd_addr[3:0] != 4'h0) begin
        src_errors++; $display("[%0t] ERR: rd_addr not 16 B aligned: %h", $time, rd_addr);
      end
      if (rd_addr >= region_size(rd_sel)) begin
        src_errors++; $display("[%0t] ERR: rd_addr %h outside region sel=%0d", $time, rd_addr, rd_sel);
      end
      src_s1 <= mem_beat(rd_sel, rd_addr);
    end else begin
      src_s1 <= {$urandom, $urandom, $urandom, $urandom};     // garbage when not enabled
    end
    src_s2 <= src_s1;
  end

  // ---------------------------------------------------------------------------
  // Request bookkeeping
  // ---------------------------------------------------------------------------
  typedef struct {
    logic [31:0] addr;
    bit          is4dw;
    bit          hi_nz;
    int          len_dw;
    logic [3:0]  fbe, lbe;
    logic [7:0]  tag;
    logic [15:0] reqid;
    logic [2:0]  tc;
    logic [1:0]  attr;
    bit          mapped;
    logic [1:0]  sel;
    logic [31:0] rel;
    int          gap;
  } req_t;

  req_t snd_q [$];      // waiting to be sent
  req_t exp_q [$];      // sent (or being sent), waiting to be completed

  int  errors = 0;
  int  n_reqs_sent = 0, n_reqs_done = 0, n_cplds = 0, n_ur = 0;
  longint unsigned n_bytes = 0;
  logic [7:0] tag_ctr = 8'h00;
  int  mps_bytes = 256;

  task automatic fail(input string msg);
    errors++;
    $display("[%0t] ERROR: %s", $time, msg);
    if (errors > 20) $fatal(1, "too many errors");
  endtask

  // --- address decode of the testbench (independent of the DUT) ---
  function automatic bit find_region(input logic [31:0] addr, input int nbytes,
                                     output logic [1:0] sel, output logic [31:0] rel);
    longint unsigned a, e;
    a = {32'b0, addr};
    e = a + nbytes;
    sel = 0; rel = 0;
    if (a >= 64'(ASQ_BASE)  && e <= 64'(ASQ_BASE)  + ASQ_BYTES)  begin sel = RSEL_ASQ;  rel = 32'(a - ASQ_BASE);  return 1; end
    if (a >= 64'(IOSQ_BASE) && e <= 64'(IOSQ_BASE) + IOSQ_BYTES) begin sel = RSEL_IOSQ; rel = 32'(a - IOSQ_BASE); return 1; end
    if (a >= 64'(PRP_BASE)  && e <= 64'(PRP_BASE)  + PRP_BYTES)  begin sel = RSEL_PRP;  rel = 32'(a - PRP_BASE);  return 1; end
    if (a >= 64'(DATA_BASE) && e <= 64'(DATA_BASE) + DATA_BYTES) begin sel = RSEL_DATA; rel = 32'(a - DATA_BASE); return 1; end
    return 0;
  endfunction

  function automatic int lo_bit(input logic [3:0] x);      // lowest set bit (0 if none)
    for (int i = 0; i < 4; i++) if (x[i]) return i;
    return 0;
  endfunction
  function automatic int hi_bit(input logic [3:0] x);      // highest set bit (0 if none)
    for (int i = 3; i >= 0; i--) if (x[i]) return i;
    return 0;
  endfunction

  // Byte count of the whole request according to PCIe Base Table 2-27
  function automatic int bc_total_ref(input req_t r);
    if (r.len_dw == 1) begin
      if (r.fbe == 4'h0) return 1;
      return hi_bit(r.fbe) - lo_bit(r.fbe) + 1;
    end
    return r.len_dw * 4 - lo_bit(r.fbe) - (3 - hi_bit(r.lbe));
  endfunction

  function automatic req_t make_req(input logic [31:0] addr, input int len_dw,
                                    input logic [3:0] fbe, input logic [3:0] lbe,
                                    input bit is4dw = 0, input bit hi_nz = 0);
    req_t r;
    r.addr = addr; r.len_dw = len_dw; r.fbe = fbe; r.lbe = lbe;
    r.is4dw = is4dw; r.hi_nz = hi_nz;
    r.tag = tag_ctr; tag_ctr = tag_ctr + 8'd1;
    r.reqid = $urandom;
    r.tc = $urandom_range(0, 7);
    r.attr = $urandom_range(0, 3);
    r.mapped = !hi_nz && find_region(addr, len_dw * 4, r.sel, r.rel);
    r.gap = 0;
    return r;
  endfunction

  function automatic logic [127:0] req_hdr(input req_t r);
    logic [127:0] h;
    h = hdr_mrd32(r.addr, 10'(r.len_dw), r.reqid, r.tag, r.fbe, r.lbe);
    h[22:20] = r.tc;
    h[13:12] = r.attr;
    if (r.is4dw) begin
      h[31:24]   = FT_MRD64;
      h[95:64]   = r.hi_nz ? 32'h0000_0001 : 32'h0;
      h[127:96]  = r.addr;
    end
    return h;
  endfunction

  task automatic enqueue(input req_t r);
    snd_q.push_back(r);
    exp_q.push_back(r);
    n_reqs_sent++;
  endtask

  // ---------------------------------------------------------------------------
  // Request driver (clocked)
  // ---------------------------------------------------------------------------
  int gap_cnt = 0;
  bit accepted;
  longint unsigned cycle = 0;
  longint unsigned last_accept_cycle = 0;
  bit              accept_seen = 0;

  always @(posedge clk) begin
    cycle <= cycle + 1;
    if (rst) begin
      s_mrd_tvalid <= 1'b0;
      s_mrd_tdata  <= '0;
      gap_cnt      <= 0;
    end else begin
      accepted = s_mrd_tvalid && s_mrd_tready;
      if (accepted) begin
        s_mrd_tvalid <= 1'b0;
        last_accept_cycle <= cycle;
        accept_seen <= 1'b1;
      end
      if ((!s_mrd_tvalid || accepted) && snd_q.size() != 0) begin
        if (gap_cnt > 0) gap_cnt <= gap_cnt - 1;
        else begin
          req_t r;
          r = snd_q.pop_front();
          s_mrd_tdata  <= req_hdr(r);
          s_mrd_tvalid <= 1'b1;
          gap_cnt      <= r.gap;
        end
      end
    end
  end

  // np_backpressure must be asserted whenever the request FIFO cannot take a request
  int np_bp_seen = 0;
  always @(posedge clk) if (!rst) begin
    if (!s_mrd_tready && !np_backpressure) fail("s_mrd_tready low while np_backpressure low");
    if (np_backpressure) np_bp_seen++;
  end

  // ---------------------------------------------------------------------------
  // m_cpl_tready generator
  // ---------------------------------------------------------------------------
  int tready_mode = 0;      // 0: always 1, 1: random 50 %, 2: long stalls, 3: always 0
  int stall_left  = 0;
  always @(negedge clk) begin
    case (tready_mode)
      0: m_cpl_tready = 1'b1;
      1: m_cpl_tready = ($urandom_range(0, 1) == 1);
      2: begin
        if (stall_left > 0) begin stall_left--; m_cpl_tready = 1'b0; end
        else begin
          m_cpl_tready = 1'b1;
          if ($urandom_range(0, 15) == 0) stall_left = $urandom_range(1, 40);
        end
      end
      default: m_cpl_tready = 1'b0;
    endcase
  end

  // ---------------------------------------------------------------------------
  // Output monitor + checker
  // ---------------------------------------------------------------------------
  // stability under back-pressure
  logic         prev_pending = 0;
  logic [127:0] prev_data;
  logic [15:0]  prev_keep;
  logic         prev_last;

  // packet assembly
  bit           in_pkt = 0;
  logic [127:0] pkt_hdr;
  logic [31:0]  pay_q [$];
  logic [31:0]  empty_q [$];

  // progress of the request currently being completed
  bit           cur_active = 0;
  req_t         cur;
  int           cur_next_addr_off;        // bytes from cur.addr
  int           cur_bc_rem;
  bit           cur_first;

  // throughput bookkeeping
  longint unsigned beat_count = 0;
  int gap_cycles = 0, tp_gaps = 0;
  longint unsigned tp_beats = 0, tp_t0 = 0, tp_t1 = 0;
  bit              tp_started = 0;
  bit              tp_enable = 0;
  longint unsigned first_valid_cycle = 0;
  bit              first_valid_seen = 0;

  task automatic check_packet(input logic [127:0] h, ref logic [31:0] pay[$]);
    logic [31:0] h0, h1, h2;
    int len_f, start, req_end, exp_end, mps, nbytes, bc_f, bc_e, la_e, nb;
    h0 = h[31:0]; h1 = h[63:32]; h2 = h[95:64];

    if (!cur_active) begin
      if (exp_q.size() == 0) begin fail("completion with no outstanding request"); return; end
      cur = exp_q.pop_front();
      cur_active = 1; cur_first = 1; cur_next_addr_off = 0; cur_bc_rem = bc_total_ref(cur);
    end
    mps = mps_bytes;

    // common fields
    if (h1[31:16] !== COMP_ID)       fail($sformatf("CplID %h", h1[31:16]));
    if (h2[31:16] !== cur.reqid)     fail($sformatf("ReqID %h exp %h", h2[31:16], cur.reqid));
    if (h2[15:8]  !== cur.tag)       fail($sformatf("Tag %h exp %h", h2[15:8], cur.tag));
    if (h0[22:20] !== cur.tc)        fail($sformatf("TC %0d exp %0d", h0[22:20], cur.tc));
    if (h0[13:12] !== cur.attr)      fail($sformatf("Attr %0d exp %0d", h0[13:12], cur.attr));
    if (h1[12] !== 1'b0)             fail("BCM set");
    if (h0[30:29] !== ((cur.mapped) ? 2'b10 : 2'b00)) fail($sformatf("Fmt %b", h0[31:29]));

    if (!cur.mapped) begin
      // ---------------- UR completion ----------------
      bc_e = bc_total_ref(cur);
      la_e = {cur.addr[6:2], 2'(lo_bit(cur.fbe))};
      if (h0[31:24] !== FT_CPL)        fail($sformatf("UR: Fmt/Type %h", h0[31:24]));
      if (h1[15:13] !== CPL_UR)        fail($sformatf("UR: status %0d", h1[15:13]));
      if (h0[9:0] !== 10'd0)           fail($sformatf("UR: Length %0d", h0[9:0]));
      if (pay.size() != 0)             fail("UR with payload");
      if (h1[11:0] !== 12'(bc_e))      fail($sformatf("UR: BC %0d exp %0d", h1[11:0], bc_e));
      if (h2[6:0] !== 7'(la_e))        fail($sformatf("UR: LA %h exp %h", h2[6:0], la_e));
      n_ur++;
      cur_active = 0; n_reqs_done++;
      return;
    end

    // ---------------- CplD ----------------
    len_f = (h0[9:0] == 10'd0) ? 1024 : int'(h0[9:0]);
    start   = cur_next_addr_off;
    req_end = cur.len_dw * 4;
    nbytes  = len_f * 4;
    if (h0[31:24] !== FT_CPLD)       fail($sformatf("CplD: Fmt/Type %h", h0[31:24]));
    if (h1[15:13] !== CPL_SC)        fail($sformatf("CplD: status %0d", h1[15:13]));
    if (pay.size() != len_f)         fail($sformatf("CplD: payload DWs %0d != Length %0d", pay.size(), len_f));
    if (nbytes > mps)                fail($sformatf("CplD: %0d B exceeds MPS %0d", nbytes, mps));
    // split boundary: ends at the next MPS boundary of the byte address or at the request end
    begin
      int abs_start;
      abs_start = (cur.addr & 32'hFFF) + start;           // MPS <= 4 KiB: only the low 12 bits matter
      exp_end = ((abs_start / mps) + 1) * mps - (cur.addr & 32'hFFF);
      if (exp_end > req_end) exp_end = req_end;
      if (start + nbytes != exp_end)
        fail($sformatf("split: CplD [%0d,+%0d) but expected end %0d (req addr %h len %0d MPS %0d)",
                       start, nbytes, exp_end, cur.addr, cur.len_dw*4, mps));
    end
    // byte count
    bc_f = (h1[11:0] == 12'd0) ? 4096 : int'(h1[11:0]);
    if (bc_f != cur_bc_rem)
      fail($sformatf("BC %0d exp %0d (req addr %h len %0d fbe %b lbe %b, CplD off %0d)",
                     bc_f, cur_bc_rem, cur.addr, cur.len_dw*4, cur.fbe, cur.lbe, start));
    // lower address
    la_e = cur_first ? {cur.addr[6:2], 2'(lo_bit(cur.fbe))} : int'((cur.addr + start) & 32'h7F);
    if (h2[6:0] !== 7'(la_e))
      fail($sformatf("LA %h exp %h (req addr %h off %0d)", h2[6:0], la_e, cur.addr, start));
    // payload data
    for (int i = 0; i < pay.size(); i++) begin
      for (int b = 0; b < 4; b++) begin
        logic [7:0] e;
        e = mem_byte(cur.sel, cur.rel + start + 4*i + b);
        if (pay[i][8*b +: 8] !== e) begin
          fail($sformatf("data mismatch at req off %0d (+%0d.%0d): got %h exp %h", start + 4*i + b, i, b, pay[i][8*b +: 8], e));
          i = pay.size(); break;
        end
      end
    end
    // advance
    nb = nbytes - (cur_first ? lo_bit(cur.fbe) : 0);
    cur_bc_rem -= nb;
    cur_next_addr_off += nbytes;
    cur_first = 0;
    n_cplds++;
    n_bytes += nbytes;
    if (cur_next_addr_off >= req_end) begin
      cur_active = 0; n_reqs_done++;
    end
  endtask

  always @(posedge clk) begin
    if (rst) begin
      in_pkt <= 0; prev_pending <= 0; beat_count <= 0;
    end else begin
      // X check
      if (m_cpl_tvalid === 1'bx) fail("m_cpl_tvalid is X");
      // stability
      if (prev_pending) begin
        if (!m_cpl_tvalid) fail("m_cpl_tvalid dropped while !tready");
        else if (m_cpl_tdata !== prev_data || m_cpl_tkeep !== prev_keep || m_cpl_tlast !== prev_last)
          fail("m_cpl data changed while !tready");
      end
      // count idle cycles inside a packet (header seen, tlast not yet): the TX path would stall on those
      if (in_pkt && !m_cpl_tvalid) begin
        gap_cycles++;
        if (tp_enable) tp_gaps++;
      end
      prev_pending <= m_cpl_tvalid && !m_cpl_tready;
      prev_data <= m_cpl_tdata; prev_keep <= m_cpl_tkeep; prev_last <= m_cpl_tlast;

      if (m_cpl_tvalid && !first_valid_seen) begin
        first_valid_seen <= 1; first_valid_cycle <= cycle;
      end
      // throughput window
      if (tp_enable && m_cpl_tvalid && m_cpl_tready) begin
        if (!tp_started) begin tp_started <= 1; tp_t0 <= cycle; tp_beats <= 0; end
        else if (cycle >= tp_t0 + 20) tp_beats <= tp_beats + 1;
        tp_t1 <= cycle;
      end

      if (m_cpl_tvalid && m_cpl_tready) begin
        beat_count <= beat_count + 1;
        if (!in_pkt) begin
          if (m_cpl_tkeep !== 16'hFFFF) fail($sformatf("header beat tkeep %h", m_cpl_tkeep));
          if (m_cpl_tlast) begin
            empty_q.delete();
            check_packet(m_cpl_tdata, empty_q);
          end else begin
            in_pkt <= 1; pkt_hdr <= m_cpl_tdata; pay_q.delete();
          end
        end else begin
          // payload beat: tkeep must be a DW prefix; full unless last
          int nd;
          nd = 0;
          for (int j = 0; j < 4; j++) if (m_cpl_tkeep[4*j +: 4] == 4'hF) nd = j + 1;
          if (m_cpl_tkeep !== ((nd == 4) ? 16'hFFFF : (16'(1) << (4*nd)) - 16'd1))
            fail($sformatf("payload tkeep %h not a DW prefix", m_cpl_tkeep));
          if (!m_cpl_tlast && nd != 4) fail("short payload beat before tlast");
          for (int j = 0; j < nd; j++) pay_q.push_back(m_cpl_tdata[32*j +: 32]);
          if (m_cpl_tlast) begin
            in_pkt <= 0;
            check_packet(pkt_hdr, pay_q);
          end
        end
      end
    end
  end

  // global watchdog
  initial begin
    #200ms;
    $fatal(1, "TIMEOUT");
  end

  // ---------------------------------------------------------------------------
  // Stimulus helpers
  // ---------------------------------------------------------------------------
  task automatic wait_idle(input int max_cycles = 200000);
    int n;
    n = 0;
    // wait for everything to be sent and completed
    while ((snd_q.size() != 0 || s_mrd_tvalid || exp_q.size() != 0 || cur_active || in_pkt) && n < max_cycles) begin
      @(posedge clk); n++;
    end
    if (n >= max_cycles) fail("wait_idle timeout (completions missing?)");
    repeat (10) @(posedge clk);
    if (m_cpl_tvalid) fail("m_cpl_tvalid after all requests completed");
  endtask

  task automatic set_mps(input int code);
    wait_idle();
    mps_code = 3'(code);
    mps_bytes = 128 << code;
    repeat (2) @(posedge clk);
  endtask

  function automatic req_t random_req();
    int         kind, region, ldist, max_dw, len;
    logic [31:0] base, addr;
    int unsigned size, off;
    logic [3:0] fbe, lbe;
    bit         is4, hinz;
    kind = $urandom_range(0, 99);
    is4 = ($urandom_range(0, 9) == 0);
    hinz = 0;
    if (kind < 8) begin
      // unmapped / straddling addresses
      case ($urandom_range(0, 9))
        0: addr = ACQ_BASE + 4 * $urandom_range(0, 100);
        1: addr = IDENT_BASE + 4 * $urandom_range(0, 100);
        2: addr = IOCQ_BASE;
        3: addr = BAR0_ADDR + 4 * $urandom_range(0, 100);
        4: addr = 32'h0000_0100;
        5: addr = ASQ_BASE + ASQ_BYTES - 8;                 // straddles the end of the ASQ
        6: addr = DATA_BASE + DATA_BYTES - 4;               // straddles the end of the ring
        7: addr = PRP_BASE + PRP_BYTES;                     // first byte past the PRP window
        8: addr = DATA_BASE - 4;
        default: begin addr = DATA_BASE + 4 * $urandom_range(0, 100); hinz = 1; is4 = 1; end
      endcase
    end else begin
      region = $urandom_range(0, 3);
      case (region)
        0: begin base = ASQ_BASE;  size = ASQ_BYTES;  end
        1: begin base = IOSQ_BASE; size = IOSQ_BYTES; end
        2: begin base = PRP_BASE;  size = PRP_BYTES;  end
        default: begin base = DATA_BASE; size = DATA_BYTES; end
      endcase
      off = 4 * $urandom_range(0, size / 4 - 1);
      case ($urandom_range(0, 3))
        0: off = off & ~32'hF;
        1: off = off & ~32'h3F;
        default: ;
      endcase
      addr = base + off;
    end
    ldist = $urandom_range(0, 99);
    if      (ldist < 22) len = 1;
    else if (ldist < 45) len = $urandom_range(2, 4);
    else if (ldist < 70) len = $urandom_range(5, 64);
    else if (ldist < 90) len = $urandom_range(65, 256);
    else                 len = $urandom_range(257, 1024);
    max_dw = (4096 - (addr & 32'hFFF)) / 4;                 // a request never crosses 4 KiB
    if (len > max_dw) len = max_dw;
    if (kind >= 8) begin                                    // mapped: stay inside the region
      int rem;
      rem = (base + size - addr) / 4;
      if (len > rem) len = rem;
    end
    if (len == 1) begin
      fbe = ($urandom_range(0, 7) == 0) ? 4'h0 : 4'($urandom_range(1, 15));
      if ($urandom_range(0, 2) == 0) fbe = 4'hF;
      lbe = 4'h0;
    end else begin
      fbe = ($urandom_range(0, 9) < 7) ? 4'hF : 4'($urandom_range(1, 15));
      lbe = ($urandom_range(0, 9) < 7) ? 4'hF : 4'($urandom_range(1, 15));
    end
    return make_req(addr, len, fbe, lbe, is4, hinz);
  endfunction

  // ---------------------------------------------------------------------------
  // Test sequence
  // ---------------------------------------------------------------------------
  real tp_rate;
  int  lat_cycles;

  task automatic run_throughput(input string name, input int byte_off, input int mps_c, input int nreq, input int nbytes);
    req_t r;
    set_mps(mps_c);
    tready_mode = 0;
    tp_enable = 0; tp_started = 0; tp_beats = 0; tp_gaps = 0;
    repeat (3) @(posedge clk);
    tp_enable = 1;
    for (int n = 0; n < nreq; n++) begin
      logic [31:0] a;
      if (byte_off != 0)
        a = DATA_BASE + ((n / 7) * 4096) % DATA_BYTES + byte_off + (n % 7) * 512;
      else
        a = DATA_BASE + (n * nbytes) % DATA_BYTES;
      r = make_req(a, nbytes / 4, 4'hF, 4'hF);
      enqueue(r);
    end
    wait_idle();
    tp_enable = 0;
    tp_rate = real'(tp_beats) / real'(tp_t1 - tp_t0 - 20 + 1);
    $display("THROUGHPUT %-34s: %0d beats in %0d cycles (after first 20) = %0.4f beats/cycle, intra-packet gaps=%0d",
             name, tp_beats, tp_t1 - tp_t0 - 20 + 1, tp_rate, tp_gaps);
  endtask

  initial begin
    req_t r;
    mps_code = 3'd1; mps_bytes = 256;
    m_cpl_tready = 1'b1;
    repeat (5) @(posedge clk);
    rst <= 1'b0;
    repeat (5) @(posedge clk);

    // ---- latency: single request into an idle completer ----
    begin
      r = make_req(DATA_BASE + 32'h100, 64, 4'hF, 4'hF);     // 256 B
      first_valid_seen = 0;
      enqueue(r);
      wait_idle();
      lat_cycles = int'(first_valid_cycle - last_accept_cycle);
      $display("LATENCY: request header accepted at cycle %0d, first CplD beat valid at cycle %0d -> %0d cycles",
               last_accept_cycle, first_valid_cycle, lat_cycles);
      if (lat_cycles > 12) fail($sformatf("latency %0d > 12", lat_cycles));
    end

    // ---- directed: PCIe worked example (512 B at 0x...040, MPS 256) ----
    enqueue(make_req(DATA_BASE + 32'h040, 128, 4'hF, 4'hF));
    // aligned and unaligned 512 B, MPS 256
    enqueue(make_req(DATA_BASE + 32'h200, 128, 4'hF, 4'hF));
    enqueue(make_req(DATA_BASE + 32'h1F4, 128 - 1 - 0, 4'hF, 4'hF));   // 508 B at 0x1F4: ends at 0x3F0
    // every first-BE value for a 1-DW read, every alignment within the beat
    for (int a = 0; a < 4; a++)
      for (int be = 0; be < 16; be++)
        enqueue(make_req(IOSQ_BASE + 32'h10 + 4*a, 1, 4'(be), 4'h0));
    // zero-length read
    enqueue(make_req(ASQ_BASE + 32'h20, 1, 4'h0, 4'h0));
    // 2-DW reads with partial BEs
    for (int be = 1; be < 16; be += 2) enqueue(make_req(ASQ_BASE + 32'h38, 2, 4'(be), 4'(16 - be)));
    // 4096 B reads
    enqueue(make_req(PRP_BASE, 1024, 4'hF, 4'hF));
    enqueue(make_req(PRP_BASE + 32'h1000 + 32'h10, 1020, 4'hF, 4'hF));
    enqueue(make_req(IOSQ_BASE, 1024, 4'hF, 4'hF));
    // reads at the very end of each region and just beyond
    enqueue(make_req(ASQ_BASE + ASQ_BYTES - 64, 16, 4'hF, 4'hF));
    enqueue(make_req(DATA_BASE + DATA_BYTES - 64, 16, 4'hF, 4'hF));
    enqueue(make_req(DATA_BASE + DATA_BYTES - 4, 1, 4'hF, 4'h0));
    enqueue(make_req(DATA_BASE + DATA_BYTES, 1, 4'hF, 4'h0));            // UR
    enqueue(make_req(ACQ_BASE, 4, 4'hF, 4'hF));                          // UR (not readable)
    enqueue(make_req(BAR0_ADDR, 1, 4'h3, 4'h0));                         // UR
    wait_idle();
    $display("directed done: %0d requests, %0d CplD, %0d UR, errors=%0d", n_reqs_done, n_cplds, n_ur, errors);

    // ---- random phases at each MPS ----
    for (int mpsc = 0; mpsc <= 3; mpsc++) begin
      set_mps(mpsc);
      for (int mode = 0; mode < 3; mode++) begin
        tready_mode = mode;
        for (int n = 0; n < 400; n++) begin
          r = random_req();
          r.gap = (mode == 0) ? 0 : $urandom_range(0, 3) == 0 ? $urandom_range(1, 20) : 0;
          enqueue(r);
        end
        wait_idle();
      end
      $display("random phase MPS=%0d done: %0d requests, %0d CplD, %0d UR, errors=%0d",
               mps_bytes, n_reqs_done, n_cplds, n_ur, errors);
    end

    // ---- np_backpressure: stall the output and fill the request FIFO ----
    set_mps(1);
    tready_mode = 3;
    np_bp_seen = 0;
    for (int n = 0; n < 60; n++) enqueue(make_req(DATA_BASE + 32'h1000 * (n % 4) + 32'h40 * (n / 4), 16, 4'hF, 4'hF));
    repeat (200) @(posedge clk);
    if (np_bp_seen == 0) fail("np_backpressure never asserted while output stalled");
    $display("np_backpressure asserted for %0d cycles while the output was stalled; snd_q left=%0d", np_bp_seen, snd_q.size());
    tready_mode = 0;
    wait_idle();

    // ---- throughput ----
    run_throughput("64 x 512 B aligned,  MPS 256", 0, 1, 64, 512);
    if (tp_rate < 0.95) fail($sformatf("throughput %0.4f < 0.95", tp_rate));
    run_throughput("64 x 512 B @0x..040, MPS 256", 32'h40, 1, 64, 512);
    if (tp_rate < 0.95) fail($sformatf("unaligned throughput %0.4f < 0.95", tp_rate));
    run_throughput("64 x 512 B @0x..044, MPS 256", 32'h44, 1, 64, 512);
    run_throughput("64 x 512 B @0x..048, MPS 256", 32'h48, 1, 64, 512);
    run_throughput("64 x 512 B @0x..04C, MPS 256", 32'h4C, 1, 64, 512);
    run_throughput("64 x 512 B aligned,  MPS 128", 0, 0, 64, 512);
    run_throughput("64 x 512 B aligned,  MPS 512", 0, 2, 64, 512);
    run_throughput("256 x 64 B aligned,  MPS 256", 0, 1, 256, 64);
    run_throughput("256 x 64 B @0x..044, MPS 256", 32'h44, 1, 256, 64);
    run_throughput("128 x 4096 B,        MPS 256", 0, 1, 128, 4096);

    wait_idle();
    $display("intra-packet idle cycles over the whole test (incl. random back-pressure phases and cold starts): %0d", gap_cycles);
    $display("SUMMARY: %0d requests, %0d CplD, %0d UR, %0d bytes, src_errors=%0d, errors=%0d",
             n_reqs_done, n_cplds, n_ur, n_bytes, src_errors, errors);
    if (errors != 0 || src_errors != 0 || n_reqs_done != n_reqs_sent) $fatal(1, "TEST FAILED");
    $display("TEST PASSED");
    $finish;
  end
endmodule
