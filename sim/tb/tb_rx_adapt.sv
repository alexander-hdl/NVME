// =============================================================================
// tb_rx_adapt.sv — self-checking test of rtl/pcie/pcie7x_rx_adapt.sv
//
// A bus-functional "core" packs random TLPs into pcie_7x 128-bit RX beats:
//   * MRd32 / MRd64 (header only), MWr32 / MWr64 (1..64 DW, plus 1024 DW with
//     Length=0), CplD / Cpl (3DW), Msg / MsgD (4DW), optional TLP digest (TD=1)
//   * SOF at DW0 or DW2, 3DW or 4DW headers, payload DWs big-endian
//   * straddling: a TLP ending at DW0/DW1 followed by a TLP starting at DW2 in
//     the same beat; SOF at DW2 without straddle; garbage in unused lanes and in
//     unused tuser sub-fields; random tkeep garbage (must be ignored)
//   * random tvalid gaps and random sink tready (phase 1), then a PERF phase
//     (no straddling, back-to-back, tvalid=1, tready=1)
// The checker rebuilds every TLP from the adapter output and compares the header
// DWs (H3=0 for 3DW) and the memory-order payload with the generator's record,
// plus tkeep (DW-granular), tlast and tuser (err_fwd on every beat, ecrc_err on
// the last beat only).
// PERF check: the output must stay saturated (cycles <= output beats + fill).
// =============================================================================
`timescale 1ns/1ps

module tb_rx_adapt;
  import nvme_host_pkg::*;

  localparam int N_DIRECT = 60;
  localparam int N_RAND   = 3500;
  localparam int N_PERF   = 400;
  localparam int N_TLP    = N_DIRECT + N_RAND + N_PERF;
  localparam int PERF_T0  = N_DIRECT + N_RAND;

  logic clk = 1'b0;
  logic rst = 1'b1;
  always #4 clk = ~clk;

  // DUT
  logic [127:0] m_axis_rx_tdata;  logic [15:0] m_axis_rx_tkeep;  logic m_axis_rx_tlast;
  logic         m_axis_rx_tvalid, m_axis_rx_tready;  logic [21:0] m_axis_rx_tuser;
  logic         rx_np_ok, np_backpressure;
  logic [127:0] m_tlp_tdata;  logic [15:0] m_tlp_tkeep;  logic m_tlp_tlast;  logic [1:0] m_tlp_tuser;
  logic         m_tlp_tvalid, m_tlp_tready;

  pcie7x_rx_adapt dut (.*);

  initial np_backpressure = 1'b0;

  // ---------------------------------------------------------------- TLP records
  class tlp_c;
    bit          is4, has_data, td, err, ecrc, force0;
    int          n;
    logic [31:0] h [4];
    logic [31:0] w [];
    logic [31:0] digest;
    int          kind;
  endclass

  tlp_c tlps [N_TLP];

  typedef struct {
    logic [127:0] d;
    bit sof, sof2, eof;
    bit [1:0] eofdw;
    bit err, ecrc;
  } beat_t;

  beat_t beats [$];
  int    perf_beat0;

  function automatic tlp_c make_tlp(input int kind, input int n_in, input bit td, input bit force0);
    tlp_c t;
    logic [7:0] ft;
    int n;
    t = new();
    n = n_in;
    t.kind = kind; t.td = td; t.force0 = force0;
    case (kind)
      0: ft = FT_MRD32;
      1: ft = FT_MWR32;
      2: ft = FT_CPLD;
      3: ft = FT_CPL;
      4: ft = FT_MRD64;
      5: ft = FT_MWR64;
      6: ft = 8'h30 + 8'($urandom_range(0, 7));      // Msg (4DW, no data)
      default: ft = 8'h70 + 8'($urandom_range(0, 7)); // MsgD (4DW + data)
    endcase
    t.has_data = ft[6];
    t.is4      = ft[5];
    t.n        = t.has_data ? n : 0;
    for (int k = 0; k < 4; k++) t.h[k] = $urandom;
    t.h[0] = {ft, 1'b0, 3'($urandom_range(0, 7)), 4'b0000, td, 1'($urandom_range(0, 1)),
              2'($urandom_range(0, 3)), 2'b00,
              t.has_data ? ((n == 1024) ? 10'd0 : 10'(n)) : 10'($urandom_range(0, 1023))};
    t.h[1] = $urandom;
    t.h[2] = $urandom;
    t.h[3] = t.is4 ? $urandom : 32'h0;
    t.w    = new[t.n > 0 ? t.n : 1];
    foreach (t.w[k]) t.w[k] = $urandom;
    t.digest = $urandom;
    t.err    = ($urandom_range(0, 9) == 0);
    t.ecrc   = ($urandom_range(0, 9) == 0);
    return t;
  endfunction

  // ---------------------------------------------------------------- generator
  logic [127:0] cd;
  int           lane;
  bit           cb_sof, cb_sof2, cb_eof, cb_err, cb_ecrc;
  int           cb_eofdw;
  int           n_straddle = 0, n_sof2 = 0;

  task automatic new_beat(input bit cont_err);
    cd       = {$urandom, $urandom, $urandom, $urandom};
    lane     = 0;
    cb_sof   = 0; cb_sof2 = 0; cb_eof = 0; cb_eofdw = 0;
    cb_err   = cont_err; cb_ecrc = ($urandom_range(0, 1) == 1);
  endtask

  task automatic push_beat(input bit next_cont_err);
    beat_t b;
    b.d = cd; b.sof = cb_sof; b.sof2 = cb_sof2; b.eof = cb_eof; b.eofdw = 2'(cb_eofdw);
    b.err = cb_err; b.ecrc = cb_ecrc;
    if (cb_sof && cb_sof2) n_sof2++;
    beats.push_back(b);
    new_beat(next_cont_err);
  endtask

  task automatic place_tlp(input tlp_c t);
    logic [31:0] dws [$];
    int start, hl;
    dws.delete();
    hl = t.is4 ? 4 : 3;
    for (int k = 0; k < hl; k++) dws.push_back(t.h[k]);
    for (int k = 0; k < t.n; k++) dws.push_back(bswap32(t.w[k]));
    if (t.td) dws.push_back(t.digest);

    if (lane == 0) begin
      start = (!t.force0 && $urandom_range(0, 99) < 25) ? 2 : 0;
    end else if (lane <= 2 && !t.force0 && $urandom_range(0, 99) < 65) begin
      start = 2;                                   // straddle: previous TLP ended at DW0/DW1
      n_straddle++;
    end else begin
      push_beat(1'b0);
      start = (!t.force0 && $urandom_range(0, 99) < 25) ? 2 : 0;
    end
    lane     = start;
    cb_sof   = 1;
    cb_sof2  = (start == 2);
    cb_err   = t.err;
    for (int i = 0; i < dws.size(); i++) begin
      cd[32*lane +: 32] = dws[i];
      lane++;
      if (i == dws.size() - 1) begin
        cb_eof = 1; cb_eofdw = lane - 1; cb_ecrc = t.ecrc;
      end
      if (lane == 4) push_beat(t.err);
    end
    // a beat that is left open (EOF before DW3) can be completed by a straddling TLP
  endtask

  initial begin
    tlp_c t;
    int idx, n, kind;
    beats.delete();
    new_beat(0);
    idx = 0;
    // directed: 1024-DW payloads (Length field 0), small sweeps
    tlps[idx++] = make_tlp(1, 1024, 0, 0);
    tlps[idx++] = make_tlp(5, 1024, 1, 0);
    tlps[idx++] = make_tlp(2, 1024, 0, 0);
    tlps[idx++] = make_tlp(1, 1024, 0, 0);
    for (int k = 0; idx < N_DIRECT; k++) begin
      case (k % 3)
        0: kind = 1;
        1: kind = 5;
        default: kind = 2;
      endcase
      tlps[idx++] = make_tlp(kind, (k % 12) + 1, (k % 5 == 4), 0);
      if (idx < N_DIRECT) tlps[idx++] = make_tlp((k % 2) ? 4 : 0, 0, 0, 0);   // header-only MRd32 / MRd64
    end
    for (int i = N_DIRECT; i < PERF_T0; i++) begin
      kind = $urandom_range(0, 7);
      case ($urandom_range(0, 9))
        0, 1, 2: n = $urandom_range(1, 4);
        3, 4:    n = 4 * $urandom_range(1, 16);
        default: n = $urandom_range(1, 64);
      endcase
      tlps[i] = make_tlp(kind, (kind == 7) ? $urandom_range(1, 8) : n, ($urandom_range(0, 9) == 0), 0);
    end
    // perf phase: no straddling, SOF@0, payload multiples of 4 DW
    for (int i = PERF_T0; i < N_TLP; i++) begin
      case ($urandom_range(0, 5))
        0: tlps[i] = make_tlp(2, 4 * $urandom_range(1, 16), 0, 1);
        1: tlps[i] = make_tlp(2, 64, 0, 1);
        2: tlps[i] = make_tlp(1, 64, 0, 1);
        3: tlps[i] = make_tlp(5, 4 * $urandom_range(1, 16), 0, 1);
        4: tlps[i] = make_tlp(0, 0, 0, 1);
        default: tlps[i] = make_tlp(4, 0, 0, 1);
      endcase
    end
    for (int i = 0; i < N_TLP; i++) begin
      if (i == PERF_T0) begin
        if (lane != 0 || cb_sof || cb_eof) push_beat(0);
        perf_beat0 = beats.size();
      end
      place_tlp(tlps[i]);
    end
    if (lane != 0 || cb_sof || cb_eof) push_beat(0);
  end

  // ---------------------------------------------------------------- driver
  int  in_beat = 0, out_t = 0, out_b = 0, errors = 0, cyc = 0;
  int  mode = 0;                          // 0 random, 1 perf
  bit  perf_go = 0, perf_done = 0;
  int  perf_c0 = 0, perf_c1 = 0, perf_out_beats = 0, perf_in_beats = 0;
  int  out_beats_total = 0;

  task automatic fail(input string msg);
    errors++;
    $display("[%0t] ERROR: %s (out tlp %0d beat %0d)", $time, msg, out_t, out_b);
    if (errors > 10) $fatal(1, "too many errors");
  endtask

  task automatic drive_idle();
    m_axis_rx_tvalid <= 1'b0;
    m_axis_rx_tdata  <= {$urandom, $urandom, $urandom, $urandom};
    m_axis_rx_tuser  <= $urandom;
    m_axis_rx_tkeep  <= $urandom;
    m_axis_rx_tlast  <= 1'b0;
  endtask

  task automatic drive_beat(input int bi);
    beat_t b;
    logic [21:0] u;
    b = beats[bi];
    u = $urandom;
    u[0]  = b.ecrc;
    u[1]  = b.err;
    u[14] = b.sof;
    if (b.sof) begin u[13] = b.sof2; u[12:10] = 3'b000; end
    u[16:15] = 2'b00;
    u[21] = b.eof;
    if (b.eof) begin u[20:19] = b.eofdw; u[18:17] = 2'b11; end
    m_axis_rx_tvalid <= 1'b1;
    m_axis_rx_tdata  <= b.d;
    m_axis_rx_tuser  <= u;
    m_axis_rx_tkeep  <= $urandom;           // must be ignored
    m_axis_rx_tlast  <= 1'b0;
  endtask

  initial begin
    m_axis_rx_tvalid = 0; m_axis_rx_tdata = '0; m_axis_rx_tuser = '0; m_axis_rx_tkeep = '0; m_axis_rx_tlast = 0;
    m_tlp_tready = 0;
    repeat (6) @(posedge clk);
    rst <= 1'b0;
  end

  always @(posedge clk) begin
    cyc++;
    if (!rst) begin
      if (m_axis_rx_tvalid && m_axis_rx_tready) begin
        in_beat++;
        if (perf_go && in_beat == perf_beat0 + 1) perf_c0 = cyc;   // first perf beat accepted
      end
      if (!m_axis_rx_tvalid || m_axis_rx_tready) begin
        if (in_beat >= beats.size()) drive_idle();
        else if (in_beat == perf_beat0 && !perf_go && out_t != PERF_T0) drive_idle();   // drain before perf
        else begin
          if (in_beat == perf_beat0 && !perf_go) perf_go = 1;
          mode = perf_go ? 1 : 0;
          if (mode == 0 && $urandom_range(0, 99) < 30) drive_idle();
          else drive_beat(in_beat);
        end
      end
    end
  end

  // sink
  always @(posedge clk) begin
    if (rst) m_tlp_tready <= 1'b0;
    else if (mode == 1) m_tlp_tready <= 1'b1;
    else                m_tlp_tready <= ($urandom_range(0, 99) < 70);
  end

  // ---------------------------------------------------------------- checker
  bit           prev_hold;
  logic [127:0] prev_d;  logic [15:0] prev_k;  logic prev_l;  logic [1:0] prev_u;

  function automatic int pl_beats(input tlp_c t);
    return (t.n + 3) / 4;
  endfunction

  always @(posedge clk) begin
    if (rst) prev_hold <= 1'b0;
    else begin
      if (prev_hold) begin
        if (!m_tlp_tvalid) fail("m_tlp_tvalid dropped while stalled");
        else if (m_tlp_tdata !== prev_d || m_tlp_tkeep !== prev_k || m_tlp_tlast !== prev_l || m_tlp_tuser !== prev_u)
          fail("m_tlp_* changed while stalled");
      end
      prev_hold <= m_tlp_tvalid && !m_tlp_tready;
      prev_d <= m_tlp_tdata; prev_k <= m_tlp_tkeep; prev_l <= m_tlp_tlast; prev_u <= m_tlp_tuser;

      if (m_tlp_tvalid && m_tlp_tready) begin
        if (out_t >= N_TLP) fail("unexpected output beat");
        else begin
          automatic tlp_c t = tlps[out_t];
          automatic int npb = pl_beats(t);
          automatic bit last_exp = (out_b == npb);
          if (m_tlp_tuser[0] !== t.err) fail($sformatf("tuser[0] (err_fwd) got %b exp %b", m_tlp_tuser[0], t.err));
          if (m_tlp_tuser[1] !== (last_exp ? t.ecrc : 1'b0))
            fail($sformatf("tuser[1] (ecrc) got %b exp %b", m_tlp_tuser[1], last_exp ? t.ecrc : 1'b0));
          if (m_tlp_tlast !== last_exp) fail($sformatf("tlast got %b exp %b", m_tlp_tlast, last_exp));
          if (out_b == 0) begin
            if (m_tlp_tkeep !== 16'hFFFF) fail($sformatf("header tkeep %h", m_tlp_tkeep));
            for (int k = 0; k < 4; k++)
              if (m_tlp_tdata[32*k +: 32] !== t.h[k])
                fail($sformatf("header DW%0d got %h exp %h (kind %0d n %0d)", k, m_tlp_tdata[32*k +: 32], t.h[k], t.kind, t.n));
          end else begin
            for (int j = 0; j < 4; j++) begin
              automatic int p = 4 * (out_b - 1) + j;
              automatic bit ev = (p < t.n);
              if (m_tlp_tkeep[4*j +: 4] !== {4{ev}}) fail($sformatf("tkeep lane %0d got %h exp %0d", j, m_tlp_tkeep[4*j +: 4], ev));
              else if (ev && m_tlp_tdata[32*j +: 32] !== t.w[p])
                fail($sformatf("payload DW%0d got %h exp %h (kind %0d n %0d)", p, m_tlp_tdata[32*j +: 32], t.w[p], t.kind, t.n));
            end
          end
          if (last_exp) begin
            out_t++; out_b = 0;
            if (out_t == N_TLP && perf_go) begin perf_done = 1; perf_c1 = cyc; end
          end else out_b++;
          if (out_t >= PERF_T0 && perf_go) perf_out_beats++;
        end
      end
    end
  end

  // ---------------------------------------------------------------- finish
  initial begin
    int exp_perf_out;
    wait (out_t == N_TLP);
    repeat (30) @(posedge clk);
    if (m_tlp_tvalid) fail("extra output beats at the end");
    exp_perf_out = 0;
    for (int i = PERF_T0; i < N_TLP; i++) exp_perf_out += 1 + pl_beats(tlps[i]);
    perf_in_beats = beats.size() - perf_beat0;
    $display("beats: %0d core beats, %0d straddles, %0d SOF@DW2 beats", beats.size(), n_straddle, n_sof2);
    $display("PERF: %0d core beats in, %0d output beats, %0d cycles (first accepted beat -> last output beat)",
             perf_in_beats, exp_perf_out, perf_c1 - perf_c0 + 1);
    if (perf_c1 - perf_c0 + 1 > exp_perf_out + 8)
      fail($sformatf("PERF: output not saturated: %0d cycles for %0d output beats", perf_c1 - perf_c0 + 1, exp_perf_out));
    if (errors == 0) $display("TEST PASSED");
    else $fatal(1, "TEST FAILED: %0d errors", errors);
    $finish;
  end

  initial begin
    #200_000_000;
    $fatal(1, "TIMEOUT: in_beat=%0d/%0d out_t=%0d/%0d", in_beat, beats.size(), out_t, N_TLP);
  end

endmodule
