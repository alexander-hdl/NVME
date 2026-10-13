// =============================================================================
// tb_tx_adapt.sv — self-checking test of rtl/pcie/pcie7x_tx_adapt.sv
//
// Source: random internal-format TLPs (3DW/4DW header, with/without payload of
//         1..64 DW; the unused H3 of 3DW TLPs is randomised and must be dropped).
// Checks on the core-format output:
//   * DW stream = {H0,H1,H2,[H3],bswap32(payload)} packed 4 DW/beat, contiguous
//   * tkeep = FFFF on all but the last beat, 000F/00FF/0FFF/FFFF on the last
//   * tlast placement, tuser == 0
//   * tvalid never drops inside a packet and a stalled beat stays stable
//     (checked in every phase, with random source gaps and random tready)
//   * PERF phases (source always valid, tready=1): the output is never idle
//     while a complete packet has been waiting for more than LAT cycles, i.e.
//     no bubbles inside or between packets; a stream of CplD-like packets
//     (3DW header, 64 DW payload) must run with zero idle cycles.
// =============================================================================
`timescale 1ns/1ps

module tb_tx_adapt;
  import nvme_host_pkg::*;

  localparam int N_RAND   = 3000;
  localparam int N_PERF   = 1500;
  localparam int N_CPLD   = 200;
  localparam int N_PKTS   = N_RAND + N_PERF + N_CPLD;
  localparam int PERF_START  = N_RAND;
  localparam int CPLD_START  = N_RAND + N_PERF;
  localparam int MAXP     = 72;
  localparam int LAT      = 4;       // allowed latency from last input beat to availability

  logic clk = 1'b0;
  logic rst = 1'b1;
  always #4 clk = ~clk;

  logic [127:0] s_tlp_tdata;  logic [15:0] s_tlp_tkeep;  logic s_tlp_tlast, s_tlp_tvalid, s_tlp_tready;
  logic [1:0]   s_tlp_tuser;
  logic [127:0] s_axis_tx_tdata;  logic [15:0] s_axis_tx_tkeep;  logic s_axis_tx_tlast, s_axis_tx_tvalid;
  logic         s_axis_tx_tready;  logic [3:0] s_axis_tx_tuser;
  logic [5:0]   tx_buf_av;

  pcie7x_tx_adapt dut (.*);

  assign tx_buf_av = 6'd29;

  // ---------------------------------------------------------------- packets
  typedef struct {
    bit          hdr4;
    int          n;                 // payload DWs (0 = none)
    logic [31:0] h [4];
    logic [31:0] w [MAXP];          // payload in memory order
  } pkt_t;

  pkt_t pkts [N_PKTS];

  function automatic int in_beats(input pkt_t p);
    return 1 + (p.n + 3) / 4;
  endfunction
  function automatic int total_dw(input pkt_t p);
    return (p.hdr4 ? 4 : 3) + p.n;
  endfunction
  function automatic int out_beats(input pkt_t p);
    return (total_dw(p) + 3) / 4;
  endfunction
  function automatic logic [31:0] exp_dw(input pkt_t p, input int t);
    int hl;
    hl = p.hdr4 ? 4 : 3;
    if (t < hl) return p.h[t];
    return bswap32(p.w[t - hl]);
  endfunction

  initial begin
    for (int i = 0; i < N_PKTS; i++) begin
      bit h4, dat;
      int n;
      if (i >= CPLD_START) begin
        h4 = 0; dat = 1; n = 64;                     // CplD of 256 B
      end else begin
        h4  = ($urandom_range(0, 3) == 0);
        dat = ($urandom_range(0, 3) != 0);
        n   = dat ? (($urandom_range(0, 9) == 0) ? $urandom_range(1, 4) : $urandom_range(1, 64)) : 0;
      end
      pkts[i].hdr4 = h4;
      pkts[i].n    = n;
      for (int k = 0; k < 4; k++) pkts[i].h[k] = $urandom;
      pkts[i].h[0][31]    = 1'b0;
      pkts[i].h[0][30]    = dat;
      pkts[i].h[0][29]    = h4;
      pkts[i].h[0][9:0]   = (n == 1024) ? 10'd0 : 10'(n);
      if (!h4 && ($urandom_range(0, 1) == 0)) pkts[i].h[3] = 32'h0;   // H3 = 0 as per the contract (half the time)
      for (int k = 0; k < MAXP; k++) pkts[i].w[k] = $urandom;
    end
  end

  // ---------------------------------------------------------------- modes
  int  mode = 0;               // 0 random/random, 1 perf (valid=1, ready=1)
  int  in_pkt = 0, in_beat = 0, out_pkt = 0, out_beat = 0, errors = 0, cyc = 0;
  int  accept_cycle [N_PKTS];
  int  bubbles = 0, perf_cycles = 0, perf_idle = 0;
  bit  perf_started = 0;
  int  cpld_idle = 0, cpld_cycles = 0;
  bit  cpld_started = 0, cpld_done = 0;
  int  perf_in_stall = 0;

  task automatic fail(input string msg);
    errors++;
    $display("[%0t] ERROR: %s (out_pkt %0d beat %0d)", $time, msg, out_pkt, out_beat);
    if (errors > 10) $fatal(1, "too many errors");
  endtask

  initial begin
    s_tlp_tvalid = 0; s_tlp_tdata = '0; s_tlp_tkeep = '0; s_tlp_tlast = 0; s_tlp_tuser = 0;
    s_axis_tx_tready = 0;
    repeat (6) @(posedge clk);
    rst <= 1'b0;
  end

  task automatic drive_beat(input int pi, input int bi);
    int nd;
    logic [127:0] dat;
    logic [15:0]  kp;
    dat = {$urandom, $urandom, $urandom, $urandom};
    kp  = '0;
    if (bi == 0) begin
      dat = {pkts[pi].h[3], pkts[pi].h[2], pkts[pi].h[1], pkts[pi].h[0]};
      kp  = 16'hFFFF;
    end else begin
      nd = ((bi == in_beats(pkts[pi]) - 1) ? (pkts[pi].n - 4 * (bi - 1)) : 4);
      for (int j = 0; j < 4; j++) begin
        if (j < nd) begin
          dat[32*j +: 32] = pkts[pi].w[4*(bi-1) + j];
          kp[4*j +: 4]    = 4'hF;
        end else begin
          dat[32*j +: 32] = $urandom;
        end
      end
    end
    s_tlp_tdata  <= dat;
    s_tlp_tkeep  <= kp;
    s_tlp_tlast  <= (bi == in_beats(pkts[pi]) - 1);
    s_tlp_tuser  <= $urandom_range(0, 3);       // must be ignored
    s_tlp_tvalid <= 1'b1;
  endtask

  // source
  always @(posedge clk) begin
    cyc++;
    if (!rst) begin
      if (s_tlp_tvalid && s_tlp_tready) begin
        if (s_tlp_tlast) begin accept_cycle[in_pkt] = cyc; in_pkt++; in_beat = 0; end
        else in_beat++;
      end
      if (!s_tlp_tvalid || s_tlp_tready) begin
        if (in_pkt >= N_PKTS) s_tlp_tvalid <= 1'b0;
        else if ((in_pkt == PERF_START || in_pkt == CPLD_START) && in_beat == 0 && out_pkt != in_pkt) begin
          s_tlp_tvalid <= 1'b0;                  // drain between phases
        end else begin
          mode = (in_pkt >= PERF_START) ? 1 : 0;
          if (mode == 0 && $urandom_range(0, 99) < 25) s_tlp_tvalid <= 1'b0;
          else drive_beat(in_pkt, in_beat);
        end
      end
    end
  end

  // sink
  always @(posedge clk) begin
    if (rst) s_axis_tx_tready <= 1'b0;
    else if (mode == 1) s_axis_tx_tready <= 1'b1;
    else                s_axis_tx_tready <= ($urandom_range(0, 99) < 55);
  end

  // ---------------------------------------------------------------- monitor
  bit           prev_hold;
  bit           in_packet_out;       // inside a packet on the output (after its first accepted beat)
  logic [127:0] prev_d;  logic [15:0] prev_k;  logic prev_l;

  always @(posedge clk) begin
    if (rst) begin
      prev_hold <= 1'b0; in_packet_out = 0;
    end else begin
      if (s_axis_tx_tuser !== 4'b0000) fail("tuser != 0");
      // tvalid must not drop once asserted until accepted, nor inside a packet
      if (prev_hold && !s_axis_tx_tvalid) fail("tvalid dropped while stalled");
      if (prev_hold && s_axis_tx_tvalid && (s_axis_tx_tdata !== prev_d || s_axis_tx_tkeep !== prev_k || s_axis_tx_tlast !== prev_l))
        fail("tdata/tkeep/tlast changed while stalled");
      if (in_packet_out && !s_axis_tx_tvalid) fail("tvalid dropped inside a packet");
      prev_hold <= s_axis_tx_tvalid && !s_axis_tx_tready;
      prev_d <= s_axis_tx_tdata; prev_k <= s_axis_tx_tkeep; prev_l <= s_axis_tx_tlast;

      if (s_axis_tx_tvalid && s_axis_tx_tready) begin
        automatic int nb = (out_pkt < N_PKTS) ? out_beats(pkts[out_pkt]) : 0;
        automatic int tot = (out_pkt < N_PKTS) ? total_dw(pkts[out_pkt]) : 0;
        if (out_pkt >= N_PKTS) fail("unexpected output beat");
        else begin
          for (int j = 0; j < 4; j++) begin
            automatic int t = 4 * out_beat + j;
            automatic bit ev = (t < tot);
            if (s_axis_tx_tkeep[4*j +: 4] !== {4{ev}})
              fail($sformatf("tkeep lane %0d got %h exp %0d (tot %0d)", j, s_axis_tx_tkeep[4*j +: 4], ev, tot));
            else if (ev && s_axis_tx_tdata[32*j +: 32] !== exp_dw(pkts[out_pkt], t))
              fail($sformatf("data lane %0d got %h exp %h", j, s_axis_tx_tdata[32*j +: 32], exp_dw(pkts[out_pkt], t)));
          end
          if (s_axis_tx_tlast) begin
            if (out_beat != nb - 1) fail($sformatf("tlast at beat %0d expected %0d", out_beat, nb - 1));
            out_pkt++; out_beat = 0; in_packet_out = 0;
          end else begin
            if (out_beat >= nb - 1) fail("missing tlast");
            out_beat++; in_packet_out = 1;
          end
        end
      end

      // bubble accounting in the PERF phases
      if (mode == 1 && out_pkt < N_PKTS && out_pkt >= PERF_START) begin
        if (!s_axis_tx_tvalid && out_pkt < in_pkt && accept_cycle[out_pkt] + LAT <= cyc) report_bubble();
        if (!perf_started) perf_started = 1;
        perf_cycles++;
        if (!s_axis_tx_tvalid) perf_idle++;
        if (s_tlp_tvalid && !s_tlp_tready) perf_in_stall++;
        // steady state of the CplD stream: after the first CplD packet (pipeline fill)
        if (out_pkt > CPLD_START) begin
          cpld_cycles++;
          if (!s_axis_tx_tvalid) cpld_idle++;
        end
      end
    end
  end

  task automatic report_bubble();
    bubbles++;
    fail($sformatf("BUBBLE: output idle but packet %0d complete since cycle %0d (now %0d)", out_pkt, accept_cycle[out_pkt], cyc));
  endtask

  // ---------------------------------------------------------------- finish
  initial begin
    wait (out_pkt == N_PKTS);
    repeat (30) @(posedge clk);
    if (s_axis_tx_tvalid) fail("extra output beats at the end");
    $display("PERF window: %0d cycles, %0d idle output cycles, %0d input stall cycles, %0d bubbles",
             perf_cycles, perf_idle, perf_in_stall, bubbles);
    $display("CplD stream (3DW hdr + 64 DW payload): %0d cycles, %0d idle", cpld_cycles, cpld_idle);
    // the CplD-like stream must run at one beat per cycle
    if (cpld_idle != 0) fail($sformatf("CplD stream not bubble-free: %0d idle cycles", cpld_idle));
    if (errors == 0) $display("TEST PASSED");
    else $fatal(1, "TEST FAILED: %0d errors", errors);
    $finish;
  end

  initial begin
    #100_000_000;
    $fatal(1, "TIMEOUT: in_pkt=%0d out_pkt=%0d", in_pkt, out_pkt);
  end

endmodule
