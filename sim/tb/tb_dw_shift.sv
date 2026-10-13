// =============================================================================
// tb_dw_shift.sv — self-checking test of rtl/common/dw_shift.sv
//
//  * 320 directed packets: every length 1..40 DW x {left 0..3, right 0..3}
//  * then random packets (length 1..40, random dir/shift) with 4 handshake modes
//  * then a PERF window with tvalid=1 / tready=1 where the exact number of
//    input cycles is predicted (1 cycle per beat + 1 per un-hidden flush)
//  * reference model: the output is compared lane by lane against the DW
//    sequence computed from the shift definition (tkeep=0 for inserted DWs)
//  * shift/dir pins are randomised on non-first beats to prove they are only
//    sampled with the first beat
// =============================================================================
`timescale 1ns/1ps

module tb_dw_shift;

  localparam int N_DIRECTED = 320;
  localparam int N_RANDOM   = 4000;
  localparam int N_PERF     = 1500;
  localparam int N_PKTS     = N_DIRECTED + N_RANDOM + N_PERF;
  localparam int PERF_START = N_DIRECTED + N_RANDOM;
  localparam int MAXDW      = 48;

  logic clk = 1'b0;
  logic rst = 1'b1;
  always #5 clk = ~clk;

  // DUT ports
  logic [127:0] s_tdata;  logic [15:0] s_tkeep;  logic s_tlast, s_tvalid, s_tready;
  logic [1:0]   s_shift;  logic s_dir;
  logic [127:0] m_tdata;  logic [15:0] m_tkeep;  logic m_tlast, m_tvalid, m_tready;

  dw_shift dut (.*);

  // ---------------------------------------------------------------- packets
  typedef struct {
    int          n;
    bit          dir;       // 0 left, 1 right
    int          sh;
    logic [31:0] d [MAXDW];
  } pkt_t;

  pkt_t pkts [N_PKTS];

  function automatic int in_beats(input pkt_t p);
    return (p.n + 3) / 4;
  endfunction

  function automatic int out_beats(input pkt_t p);
    int tot;
    if (p.dir == 1'b1 || p.sh == 0) begin
      tot = p.n + p.sh;
      return (tot + 3) / 4;
    end else begin
      tot = p.n - p.sh;
      return (tot <= 0) ? 1 : (tot + 3) / 4;
    end
  endfunction

  // expected lane content for output global DW position g
  function automatic bit exp_valid(input pkt_t p, input int g);
    if (p.dir == 1'b1 || p.sh == 0) return (g >= p.sh) && (g < p.sh + p.n);
    else                            return (g < p.n - p.sh);
  endfunction

  function automatic logic [31:0] exp_dw(input pkt_t p, input int g);
    if (p.dir == 1'b1 || p.sh == 0) return p.d[g - p.sh];
    else                            return p.d[g + p.sh];
  endfunction

  // does the packet end with a spill beat?
  function automatic bit has_spill(input pkt_t p);
    int k, nlast;
    k     = in_beats(p);
    nlast = p.n - 4 * (k - 1);
    if (p.dir == 1'b1 || p.sh == 0) return (p.sh != 0) && (nlast > 4 - p.sh);
    else                            return (k >= 2) && (nlast > p.sh);
  endfunction

  // does a packet's first beat only prime (no output)?
  function automatic bit primes(input pkt_t p);
    return (p.dir == 1'b0) && (p.sh != 0) && (in_beats(p) >= 2);
  endfunction

  initial begin
    for (int i = 0; i < N_PKTS; i++) begin
      if (i < N_DIRECTED) begin
        pkts[i].n = (i / 8) + 1;
        case (i % 8)
          0: begin pkts[i].dir = 0; pkts[i].sh = 0; end
          1: begin pkts[i].dir = 0; pkts[i].sh = 1; end
          2: begin pkts[i].dir = 0; pkts[i].sh = 2; end
          3: begin pkts[i].dir = 0; pkts[i].sh = 3; end
          4: begin pkts[i].dir = 1; pkts[i].sh = 0; end
          5: begin pkts[i].dir = 1; pkts[i].sh = 1; end
          6: begin pkts[i].dir = 1; pkts[i].sh = 2; end
          default: begin pkts[i].dir = 1; pkts[i].sh = 3; end
        endcase
      end else begin
        pkts[i].n   = $urandom_range(1, 40);
        pkts[i].dir = $urandom_range(0, 1);
        pkts[i].sh  = $urandom_range(0, 3);
      end
      for (int k = 0; k < MAXDW; k++) pkts[i].d[k] = $urandom;
    end
  end

  // ---------------------------------------------------------------- modes
  // 0: random/random  1: tvalid=1,random ready  2: random valid, ready=1  3: PERF (1/1)
  int mode = 0;

  function automatic int mode_of(input int idx);
    if (idx >= PERF_START)                    return 3;
    if (idx < N_DIRECTED)                     return 0;
    case (((idx - N_DIRECTED) / 500) % 3)
      0: return 0;
      1: return 1;
      default: return 2;
    endcase
  endfunction

  // ---------------------------------------------------------------- driver
  int in_pkt  = 0;
  int in_beat = 0;
  int out_pkt = 0;
  int out_beat = 0;
  int errors = 0;
  bit perf_started = 0, perf_done = 0;
  int perf_cycles = 0, perf_expected = 0;

  task automatic fail(input string msg);
    errors++;
    $display("[%0t] ERROR: %s (pkt %0d beat %0d)", $time, msg, out_pkt, out_beat);
    if (errors > 10) $fatal(1, "too many errors");
  endtask

  initial begin
    s_tvalid = 0; s_tdata = '0; s_tkeep = '0; s_tlast = 0; s_shift = 0; s_dir = 0;
    repeat (5) @(posedge clk);
    rst <= 1'b0;
  end

  function automatic bit pass_valid(input int m);
    case (m)
      0: return ($urandom_range(0, 99) < 65);
      1: return 1'b1;
      2: return ($urandom_range(0, 99) < 65);
      default: return 1'b1;
    endcase
  endfunction

  task automatic drive_beat(input int pi, input int bi);
    int k, nlast, nd;
    logic [127:0] dat;
    logic [15:0]  kp;
    k     = in_beats(pkts[pi]);
    nlast = pkts[pi].n - 4 * (k - 1);
    nd    = (bi == k - 1) ? nlast : 4;
    dat   = '0; kp = '0;
    for (int j = 0; j < 4; j++) begin
      if (j < nd) begin
        dat[32*j +: 32] = pkts[pi].d[4*bi + j];
        kp[4*j +: 4]    = 4'hF;
      end else begin
        dat[32*j +: 32] = $urandom;             // garbage in unused lanes
      end
    end
    s_tdata  <= dat;
    s_tkeep  <= kp;
    s_tlast  <= (bi == k - 1);
    s_tvalid <= 1'b1;
    if (bi == 0) begin
      s_shift <= pkts[pi].sh[1:0];
      s_dir   <= pkts[pi].dir;
    end else begin
      s_shift <= $urandom_range(0, 3);
      s_dir   <= $urandom_range(0, 1);
    end
  endtask

  always @(posedge clk) begin
    if (!rst) begin
      if (s_tvalid && s_tready) begin
        if (perf_started && !perf_done) perf_cycles++;
        if (s_tlast) begin in_pkt++; in_beat = 0; end
        else in_beat++;
      end else if (perf_started && !perf_done && s_tvalid) begin
        perf_cycles++;          // stall cycle inside perf window
      end
      if (!s_tvalid || s_tready) begin
        if (in_pkt >= N_PKTS) begin
          s_tvalid <= 1'b0;
        end else if (in_pkt == PERF_START && in_beat == 0 && !perf_started && out_pkt != PERF_START) begin
          // before the perf window: let everything drain first
          s_tvalid <= 1'b0;
          s_shift  <= $urandom_range(0, 3);
          s_dir    <= $urandom_range(0, 1);
        end else begin
          if (in_pkt == PERF_START && in_beat == 0 && !perf_started) begin
            perf_started = 1;
            mode = 3;
          end
          if (!perf_started) mode = mode_of(in_pkt);
          if (pass_valid(mode)) drive_beat(in_pkt, in_beat);
          else begin
            s_tvalid <= 1'b0;
            s_shift  <= $urandom_range(0, 3);
            s_dir    <= $urandom_range(0, 1);
            s_tdata  <= {$urandom, $urandom, $urandom, $urandom};
          end
        end
      end
      // end of perf window: the last input beat of the last packet was accepted
      if (perf_started && !perf_done && in_pkt == N_PKTS) begin
        perf_done = 1;
      end
    end
  end

  // ---------------------------------------------------------------- sink
  always @(posedge clk) begin
    if (rst) m_tready <= 1'b0;
    else case (mode)
      0: m_tready <= ($urandom_range(0, 99) < 60);
      1: m_tready <= ($urandom_range(0, 99) < 60);
      2: m_tready <= 1'b1;
      default: m_tready <= 1'b1;
    endcase
  end

  // ---------------------------------------------------------------- monitor
  logic         prev_hold;
  logic [127:0] prev_d;
  logic [15:0]  prev_k;
  logic         prev_l;

  always @(posedge clk) begin
    if (rst) begin
      prev_hold <= 1'b0;
    end else begin
      // stability: a stalled valid beat must persist unchanged
      if (prev_hold) begin
        if (!m_tvalid) fail("m_tvalid dropped while stalled");
        else if (m_tdata !== prev_d || m_tkeep !== prev_k || m_tlast !== prev_l) fail("m_* changed while stalled");
      end
      prev_hold <= m_tvalid && !m_tready;
      prev_d <= m_tdata; prev_k <= m_tkeep; prev_l <= m_tlast;

      if (m_tvalid && m_tready) begin
        automatic int nb = (out_pkt < N_PKTS) ? out_beats(pkts[out_pkt]) : 0;
        if (out_pkt >= N_PKTS) fail("unexpected output beat");
        else begin
          for (int j = 0; j < 4; j++) begin
            automatic int g = 4 * out_beat + j;
            automatic bit ev = exp_valid(pkts[out_pkt], g);
            if (m_tkeep[4*j +: 4] !== {4{ev}}) fail($sformatf("tkeep lane %0d: got %h exp %0d", j, m_tkeep[4*j +: 4], ev));
            else if (ev && m_tdata[32*j +: 32] !== exp_dw(pkts[out_pkt], g))
              fail($sformatf("data lane %0d: got %h exp %h", j, m_tdata[32*j +: 32], exp_dw(pkts[out_pkt], g)));
          end
          if (m_tlast) begin
            if (out_beat != nb - 1) fail($sformatf("tlast at beat %0d, expected %0d", out_beat, nb - 1));
            out_pkt++; out_beat = 0;
          end else begin
            if (out_beat >= nb - 1) fail("missing tlast");
            out_beat++;
          end
        end
      end
    end
  end

  // ---------------------------------------------------------------- finish
  initial begin
    // predicted input cycles of the perf window
    perf_expected = 0;
    wait (perf_started);
    wait (perf_done);
    // compute expected here (after packets exist)
    for (int i = PERF_START; i < N_PKTS; i++) begin
      perf_expected += in_beats(pkts[i]);
      if (i < N_PKTS - 1 && has_spill(pkts[i]) && !primes(pkts[i+1])) perf_expected++;
    end
    if (perf_cycles != perf_expected)
      fail($sformatf("PERF: input took %0d cycles, expected %0d", perf_cycles, perf_expected));
    else
      $display("PERF: %0d packets, %0d input cycles (= predicted, 1 beat/cycle + un-hidden flushes)", N_PERF, perf_cycles);
    wait (out_pkt == N_PKTS);
    repeat (20) @(posedge clk);
    if (m_tvalid) fail("extra output beats at the end");
    if (errors == 0) $display("TEST PASSED");
    else $fatal(1, "TEST FAILED: %0d errors", errors);
    $finish;
  end

  initial begin
    #50_000_000;
    $fatal(1, "TIMEOUT: in_pkt=%0d out_pkt=%0d", in_pkt, out_pkt);
  end

endmodule
