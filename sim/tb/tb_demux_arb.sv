// =============================================================================
// tb_demux_arb.sv — self-checking tests of tlp_rx_demux and tlp_tx_arb
//
// DEMUX : random TLPs (MRd32/64, MWr32/64, Cpl, CplD, Msg, MsgD, CfgRd/Wr type 0/1,
//         unknown types; random poison on MWr and others) with random gaps and
//         random per-output tready.  Expected: Cpl/CplD -> m_cpl, MRd -> m_mrd,
//         clean MWr -> m_mwr, everything else and poisoned MWr dropped; every
//         output stream equals the expected beat sequence (data, tkeep, tlast,
//         tuser); drop_cnt_inc pulses == number of dropped TLPs.  A final PERF
//         phase (valid=1, all tready=1) checks that s_tlp_tready is always high.
// ARB   : two sources of random multi-beat packets (with gaps inside packets).
//         Checks: packets are never interleaved, per-source order and content are
//         preserved, nothing is lost.  In the saturation phase (both sources always
//         pending, sink always ready) it checks fairness (the winner alternates, no
//         source wins twice in a row while the other is pending) and zero bubbles
//         (the output tvalid stays high for the whole phase).
// =============================================================================
`timescale 1ns/1ps

module tb_demux_arb;

  logic clk = 1'b0;
  logic rst = 1'b1;
  always #4 clk = ~clk;

  int errors = 0;
  int cyc = 0;
  always @(posedge clk) cyc++;

  task automatic fail(input string msg);
    errors++;
    $display("[%0t] ERROR: %s", $time, msg);
    if (errors > 10) $fatal(1, "too many errors");
  endtask

  typedef struct {
    logic [127:0] d;
    logic [15:0]  k;
    bit           l;
    logic [1:0]   u;
  } beat_t;

  // ==========================================================================
  // DEMUX
  // ==========================================================================
  localparam int ND_RAND = 3000;
  localparam int ND_PERF = 300;
  localparam int ND_TOT  = ND_RAND + ND_PERF;

  logic [127:0] dm_tdata;  logic [15:0] dm_tkeep;  logic dm_tlast, dm_tvalid, dm_tready;  logic [1:0] dm_tuser;
  logic [127:0] c_tdata, r_tdata, w_tdata;
  logic [15:0]  c_tkeep, r_tkeep, w_tkeep;
  logic         c_tlast, r_tlast, w_tlast, c_tvalid, r_tvalid, w_tvalid, c_tready, r_tready, w_tready;
  logic [1:0]   c_tuser, r_tuser, w_tuser;
  logic         drop_cnt_inc;

  tlp_rx_demux u_demux (
    .clk(clk), .rst(rst),
    .s_tlp_tdata(dm_tdata), .s_tlp_tkeep(dm_tkeep), .s_tlp_tlast(dm_tlast), .s_tlp_tuser(dm_tuser),
    .s_tlp_tvalid(dm_tvalid), .s_tlp_tready(dm_tready),
    .m_cpl_tdata(c_tdata), .m_cpl_tkeep(c_tkeep), .m_cpl_tlast(c_tlast), .m_cpl_tuser(c_tuser),
    .m_cpl_tvalid(c_tvalid), .m_cpl_tready(c_tready),
    .m_mrd_tdata(r_tdata), .m_mrd_tkeep(r_tkeep), .m_mrd_tlast(r_tlast), .m_mrd_tuser(r_tuser),
    .m_mrd_tvalid(r_tvalid), .m_mrd_tready(r_tready),
    .m_mwr_tdata(w_tdata), .m_mwr_tkeep(w_tkeep), .m_mwr_tlast(w_tlast), .m_mwr_tuser(w_tuser),
    .m_mwr_tvalid(w_tvalid), .m_mwr_tready(w_tready),
    .drop_cnt_inc(drop_cnt_inc));

  beat_t src_beats [$];             // all input beats
  beat_t exp_q [3][$];              // expected beats per output (0 cpl, 1 mrd, 2 mwr)
  int    exp_drops = 0;
  int    exp_cnt [3];
  int    perf_first_beat;
  int    a_total_beats;
  int    got_drops = 0;
  int    n_route [4];

  function automatic int route_of(input logic [7:0] ft, input bit poison);
    case (ft)
      8'h0A, 8'h4A:       return 0;
      8'h00, 8'h20:       return 1;
      8'h40, 8'h60:       return poison ? 3 : 2;
      default:            return 3;
    endcase
  endfunction

  initial begin
    logic [7:0] fts [$];
    fts = '{8'h00, 8'h20, 8'h40, 8'h60, 8'h0A, 8'h4A, 8'h30, 8'h34, 8'h70, 8'h74, 8'h05, 8'h45, 8'h04, 8'h44,
            8'h01, 8'h1B, 8'h7F, 8'h4B, 8'h3F, 8'h02, 8'h5A};
    for (int r = 0; r < 4; r++) n_route[r] = 0;
    for (int i = 0; i < ND_TOT; i++) begin
      logic [7:0] ft;
      int nb, rt;
      bit poison;
      if (i < ND_RAND) begin
        ft = ($urandom_range(0, 99) < 55) ? fts[$urandom_range(0, 5)] : fts[$urandom_range(0, fts.size() - 1)];
        poison = ($urandom_range(0, 4) == 0);
      end else begin
        case ($urandom_range(0, 3))
          0: ft = 8'h4A;  1: ft = 8'h00;  2: ft = 8'h40;  default: ft = 8'h60;
        endcase
        poison = 0;
      end
      nb = ft[6] ? $urandom_range(1, 5) : 1;
      if (i == ND_RAND) perf_first_beat = src_beats.size();
      rt = route_of(ft, poison);
      n_route[rt]++;
      if (rt == 3) exp_drops++;
      for (int b = 0; b < nb; b++) begin
        beat_t bt;
        bt.d = {$urandom, $urandom, $urandom, $urandom};
        if (b == 0) bt.d[31:24] = ft;
        bt.k = (b == nb - 1) ? 16'($urandom) : 16'hFFFF;
        bt.l = (b == nb - 1);
        bt.u = {$urandom_range(0, 1) == 1, poison};
        src_beats.push_back(bt);
        if (rt < 3) exp_q[rt].push_back(bt);
      end
    end
    for (int o = 0; o < 3; o++) exp_cnt[o] = exp_q[o].size();
  end

  int dm_in = 0, dm_mode = 0, dm_cnt[3];
  bit dm_perf = 0;
  int dm_perf_stall = 0;

  initial begin
    dm_tvalid = 0; dm_tdata = '0; dm_tkeep = '0; dm_tlast = 0; dm_tuser = 0;
    dm_cnt[0] = 0; dm_cnt[1] = 0; dm_cnt[2] = 0;
  end

  always @(posedge clk) begin
    if (!rst) begin
      if (dm_tvalid && dm_tready) dm_in++;
      if (dm_perf && dm_tvalid && !dm_tready) dm_perf_stall++;
      if (!dm_tvalid || dm_tready) begin
        if (dm_in >= src_beats.size()) dm_tvalid <= 1'b0;
        else if (dm_in == perf_first_beat && !dm_perf) begin
          // start of the perf phase: the demux has no buffering, so everything before is already consumed
          dm_perf = 1;
          dm_tvalid <= 1'b0;
        end else begin
          if (!dm_perf && $urandom_range(0, 99) < 30) begin
            dm_tvalid <= 1'b0;
            dm_tdata  <= {$urandom, $urandom, $urandom, $urandom};
            dm_tuser  <= $urandom;
          end else begin
            dm_tvalid <= 1'b1;
            dm_tdata  <= src_beats[dm_in].d;
            dm_tkeep  <= src_beats[dm_in].k;
            dm_tlast  <= src_beats[dm_in].l;
            dm_tuser  <= src_beats[dm_in].u;
          end
        end
      end
    end
  end

  always @(posedge clk) begin
    if (rst) begin
      c_tready <= 0; r_tready <= 0; w_tready <= 0;
    end else if (dm_perf) begin
      c_tready <= 1; r_tready <= 1; w_tready <= 1;
    end else begin
      c_tready <= ($urandom_range(0, 99) < 60);
      r_tready <= ($urandom_range(0, 99) < 60);
      w_tready <= ($urandom_range(0, 99) < 60);
    end
  end

  // output checkers
  task automatic check_out(input int o, input logic [127:0] d, input logic [15:0] k, input logic l, input logic [1:0] u);
    if (exp_q[o].size() == 0) fail($sformatf("demux out %0d: unexpected beat", o));
    else begin
      beat_t e;
      e = exp_q[o].pop_front();
      if (d !== e.d || k !== e.k || l !== e.l || u !== e.u)
        fail($sformatf("demux out %0d: beat %0d mismatch got d=%h k=%h l=%b u=%b exp d=%h k=%h l=%b u=%b",
                       o, dm_cnt[o], d, k, l, u, e.d, e.k, e.l, e.u));
      dm_cnt[o]++;
    end
  endtask

  always @(posedge clk) if (!rst) begin
    if (c_tvalid && c_tready) check_out(0, c_tdata, c_tkeep, c_tlast, c_tuser);
    if (r_tvalid && r_tready) check_out(1, r_tdata, r_tkeep, r_tlast, r_tuser);
    if (w_tvalid && w_tready) check_out(2, w_tdata, w_tkeep, w_tlast, w_tuser);
    if (drop_cnt_inc) got_drops++;
    // exclusivity
    if ((c_tvalid + r_tvalid + w_tvalid) > 1) fail("demux: more than one output valid");
  end

  // output-side stall stability: a stalled valid output holds its data
  logic [127:0] c_prev_d, r_prev_d, w_prev_d;
  bit c_hold, r_hold, w_hold;
  always @(posedge clk) if (!rst) begin
    if (c_hold && (!c_tvalid || c_tdata !== c_prev_d)) fail("demux m_cpl not stable while stalled");
    if (r_hold && (!r_tvalid || r_tdata !== r_prev_d)) fail("demux m_mrd not stable while stalled");
    if (w_hold && (!w_tvalid || w_tdata !== w_prev_d)) fail("demux m_mwr not stable while stalled");
    c_hold <= c_tvalid && !c_tready; r_hold <= r_tvalid && !r_tready; w_hold <= w_tvalid && !w_tready;
    c_prev_d <= c_tdata; r_prev_d <= r_tdata; w_prev_d <= w_tdata;
  end

  // ==========================================================================
  // ARBITER
  // ==========================================================================
  localparam int NA_RAND = 1200;          // packets per source, random phase
  localparam int NA_SAT  = 400;           // packets per source, saturation phase
  localparam int NA_TOT  = NA_RAND + NA_SAT;

  logic [127:0] ac_tdata, ar_tdata, ao_tdata;
  logic [15:0]  ac_tkeep, ar_tkeep, ao_tkeep;
  logic         ac_tlast, ar_tlast, ao_tlast, ac_tvalid, ar_tvalid, ao_tvalid, ac_tready, ar_tready, ao_tready;
  logic [1:0]   ac_tuser, ar_tuser, ao_tuser;

  tlp_tx_arb u_arb (
    .clk(clk), .rst(rst),
    .s_cpl_tdata(ac_tdata), .s_cpl_tkeep(ac_tkeep), .s_cpl_tlast(ac_tlast), .s_cpl_tuser(ac_tuser),
    .s_cpl_tvalid(ac_tvalid), .s_cpl_tready(ac_tready),
    .s_req_tdata(ar_tdata), .s_req_tkeep(ar_tkeep), .s_req_tlast(ar_tlast), .s_req_tuser(ar_tuser),
    .s_req_tvalid(ar_tvalid), .s_req_tready(ar_tready),
    .m_tlp_tdata(ao_tdata), .m_tlp_tkeep(ao_tkeep), .m_tlp_tlast(ao_tlast), .m_tlp_tuser(ao_tuser),
    .m_tlp_tvalid(ao_tvalid), .m_tlp_tready(ao_tready));

  // per-source packet streams. Beat data = {src[7:0], pkt[23:0], beat[7:0], random...}
  beat_t a_beats [2][$];
  int    a_pkt_first [2][NA_TOT+1];      // beat index where each packet starts

  initial begin
    for (int s = 0; s < 2; s++) begin
      for (int p = 0; p < NA_TOT; p++) begin
        int nb;
        nb = (p >= NA_RAND) ? $urandom_range(1, 4) : $urandom_range(1, 6);
        a_pkt_first[s][p] = a_beats[s].size();
        for (int b = 0; b < nb; b++) begin
          beat_t bt;
          bt.d = {8'(s), 24'(p), 8'(b), 88'($urandom)};
          bt.d[63:0] = {$urandom, $urandom};
          bt.k = 16'hFFFF;
          bt.l = (b == nb - 1);
          bt.u = 2'(s);
          a_beats[s].push_back(bt);
        end
      end
      a_pkt_first[s][NA_TOT] = a_beats[s].size();
    end
    a_total_beats = a_beats[0].size() + a_beats[1].size();
  end

  int a_in [2];
  bit a_sat = 0;                   // saturation phase
  int a_in_pkt [2];

  initial begin
    a_in[0] = 0; a_in[1] = 0;
    ac_tvalid = 0; ar_tvalid = 0; ac_tdata = '0; ar_tdata = '0; ac_tkeep = '0; ar_tkeep = '0;
    ac_tlast = 0; ar_tlast = 0; ac_tuser = 0; ar_tuser = 0;
    ao_tready = 0;
  end

  // source drivers (index 0 = completer, 1 = requester)
  always @(posedge clk) begin
    if (!rst) begin
      // --- completer
      if (ac_tvalid && ac_tready) a_in[0]++;
      if (!ac_tvalid || ac_tready) begin
        if (a_in[0] >= a_beats[0].size()) ac_tvalid <= 1'b0;
        else if (a_in[0] == a_pkt_first[0][NA_RAND] && !a_sat) ac_tvalid <= 1'b0;   // wait for sat start
        else if (!a_sat && $urandom_range(0, 99) < 35) ac_tvalid <= 1'b0;
        else begin
          ac_tvalid <= 1'b1;
          ac_tdata <= a_beats[0][a_in[0]].d; ac_tkeep <= a_beats[0][a_in[0]].k;
          ac_tlast <= a_beats[0][a_in[0]].l; ac_tuser <= a_beats[0][a_in[0]].u;
        end
      end
      // --- requester
      if (ar_tvalid && ar_tready) a_in[1]++;
      if (!ar_tvalid || ar_tready) begin
        if (a_in[1] >= a_beats[1].size()) ar_tvalid <= 1'b0;
        else if (a_in[1] == a_pkt_first[1][NA_RAND] && !a_sat) ar_tvalid <= 1'b0;
        else if (!a_sat && $urandom_range(0, 99) < 50) ar_tvalid <= 1'b0;
        else begin
          ar_tvalid <= 1'b1;
          ar_tdata <= a_beats[1][a_in[1]].d; ar_tkeep <= a_beats[1][a_in[1]].k;
          ar_tlast <= a_beats[1][a_in[1]].l; ar_tuser <= a_beats[1][a_in[1]].u;
        end
      end
    end
  end

  // saturation phase starts when both sources finished the random phase and the arbiter drained
  int a_out_beats = 0, a_sat_beats = 0;
  int a_exp_pkt [2];
  int a_exp_beat [2];
  int a_cur_src = -1;                       // source of the packet in progress on the output
  int a_last_src = -1;                      // source of the last completed packet (saturation phase)
  int sat_start_cyc = 0, sat_last_cyc = 0, sat_idle = 0, sat_run = 0;
  bit sat_both_pending;

  initial begin
    a_exp_pkt[0] = 0; a_exp_pkt[1] = 0; a_exp_beat[0] = 0; a_exp_beat[1] = 0;
    wait (!rst);
    // wait until both sources are parked at the start of the saturation phase and the arbiter is empty
    wait (a_in[0] == a_pkt_first[0][NA_RAND] && a_in[1] == a_pkt_first[1][NA_RAND] && !ao_tvalid);
    repeat (10) @(posedge clk);
    a_sat = 1;
  end

  always @(posedge clk) begin
    if (rst) ao_tready <= 1'b0;
    else if (a_sat) ao_tready <= 1'b1;
    else            ao_tready <= ($urandom_range(0, 99) < 65);
  end

  // output checker
  logic [127:0] ao_prev_d; bit ao_hold;
  int sat_pkts_seen [2];
  initial begin sat_pkts_seen[0] = 0; sat_pkts_seen[1] = 0; end

  always @(posedge clk) begin
    if (!rst) begin
      if (ao_hold && (!ao_tvalid || ao_tdata !== ao_prev_d)) fail("arb output not stable while stalled");
      ao_hold <= ao_tvalid && !ao_tready;
      ao_prev_d <= ao_tdata;

      if (ao_tvalid && ao_tready) begin
        automatic int s;
        automatic beat_t e;
        a_out_beats++;
        if (a_cur_src < 0) begin
          s = int'(ao_tdata[127:120]);                 // source id from the data
          if (s > 1) begin fail("arb: bad source id"); s = 0; end
          a_cur_src = s;
          // fairness in the saturation phase: sources must alternate while both have packets left
          if (a_sat) begin
            if (a_last_src == s && a_exp_pkt[1 - s] < NA_TOT && a_exp_pkt[s] < NA_TOT)
              fail($sformatf("arb fairness: source %0d won twice in a row (pkts %0d/%0d)", s, a_exp_pkt[0], a_exp_pkt[1]));
          end
        end
        s = a_cur_src;
        e = a_beats[s][a_pkt_first[s][a_exp_pkt[s]] + a_exp_beat[s]];
        if (ao_tdata !== e.d || ao_tkeep !== e.k || ao_tlast !== e.l || ao_tuser !== e.u)
          fail($sformatf("arb: src %0d pkt %0d beat %0d mismatch (got %h exp %h)", s, a_exp_pkt[s], a_exp_beat[s], ao_tdata, e.d));
        if (ao_tlast) begin
          a_exp_pkt[s]++; a_exp_beat[s] = 0; a_last_src = s; a_cur_src = -1;
        end else a_exp_beat[s]++;
        if (a_sat) begin
          a_sat_beats++;
          sat_last_cyc = cyc;
          if (sat_start_cyc == 0) sat_start_cyc = cyc;
        end
      end else if (a_sat && sat_start_cyc != 0 && !ao_tvalid && a_out_beats < a_total_beats) begin
        sat_idle++;
      end
    end
  end

  // ==========================================================================
  // finish
  // ==========================================================================
  initial begin
    wait (!rst);
    wait (a_out_beats == a_total_beats);
    wait (dm_cnt[0] == exp_cnt[0] && dm_cnt[1] == exp_cnt[1] && dm_cnt[2] == exp_cnt[2]);
    repeat (30) @(posedge clk);
    if (exp_q[0].size() != 0 || exp_q[1].size() != 0 || exp_q[2].size() != 0) fail("demux: expected beats missing");
    if (got_drops != exp_drops) fail($sformatf("demux: drop_cnt_inc pulses %0d, expected %0d", got_drops, exp_drops));
    if (dm_perf_stall != 0) fail($sformatf("demux PERF: s_tlp_tready low %0d times with all sinks ready", dm_perf_stall));
    if (sat_idle != 0) fail($sformatf("arb saturation: %0d idle output cycles", sat_idle));
    if (ao_tvalid) fail("arb: extra output at end");
    $display("DEMUX: routes cpl=%0d mrd=%0d mwr=%0d dropped=%0d TLPs (%0d drop pulses)",
             n_route[0], n_route[1], n_route[2], n_route[3], got_drops);
    $display("ARB: %0d beats total, saturation phase %0d beats in %0d cycles, %0d idle",
             a_out_beats, a_sat_beats, sat_last_cyc - sat_start_cyc + 1, sat_idle);
    if (errors == 0) $display("TEST PASSED");
    else $fatal(1, "TEST FAILED: %0d errors", errors);
    $finish;
  end

  initial begin
    repeat (6) @(posedge clk);
    rst <= 1'b0;
  end

  initial begin
    #100_000_000;
    $fatal(1, "TIMEOUT: dm_in=%0d/%0d a_in=%0d,%0d a_out=%0d", dm_in, src_beats.size(), a_in[0], a_in[1], a_out_beats);
  end

endmodule
