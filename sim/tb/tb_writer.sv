// =============================================================================
// tb_writer — testbench for hostmem_writer + hostmem_rams (+ prp_synth mux)
//
// Part 1  MWr traffic into Identify / ACQ / IOCQ:
//         random DW alignment, Length 1..64 DW (plus a 1024-DW zero fill),
//         partial First/Last BE, CQE-shaped writes (1..3 CQEs per MWr and split
//         8+8 byte CQE writes), 4DW headers, poisoned / unmapped / straddling
//         TLPs (must be dropped and counted), random tvalid gaps.
//         Checks (a) every beat on the shared write port against a byte-level
//         reference (wr_be, wr_data, wr_beat_addr, wr_en_*), (b) drop counter,
//         (c) RAM contents read back through acq/iocq/ident read ports
//         (2-cycle latency).
// Part 2  CQ clear FSM: fill ACQ/IOCQ, clear, read back zeros, Identify intact,
//         clear_done protocol, normal writes afterwards.
// Part 3  Completer read mux: ASQ/IOSQ writes through their ports and reads via
//         cpl_rd_* (sel ASQ/IOSQ/PRP/DATA, random addresses, 2-cycle latency,
//         cpl_rd_en_data only for RSEL_DATA).
// =============================================================================
module tb_writer;
  import nvme_host_pkg::*;

  localparam int unsigned N_SLOTS    = 4;
  localparam int unsigned SLOT_BYTES = 16384;

  logic clk = 1'b0;
  logic rst = 1'b1;
  always #4 clk = ~clk;

  // ---------------- DUT: writer ----------------
  logic [127:0] s_mwr_tdata;
  logic [15:0]  s_mwr_tkeep;
  logic         s_mwr_tlast, s_mwr_tvalid, s_mwr_tready;
  logic [1:0]   s_mwr_tuser;
  logic         wr_en_ident, wr_en_acq, wr_en_iocq, drop_cnt_inc;
  logic [31:0]  wr_beat_addr;
  logic [127:0] wr_data;
  logic [15:0]  wr_be;

  hostmem_writer dut_w (
    .clk(clk), .rst(rst),
    .s_mwr_tdata(s_mwr_tdata), .s_mwr_tkeep(s_mwr_tkeep), .s_mwr_tlast(s_mwr_tlast),
    .s_mwr_tuser(s_mwr_tuser), .s_mwr_tvalid(s_mwr_tvalid), .s_mwr_tready(s_mwr_tready),
    .wr_en_ident(wr_en_ident), .wr_en_acq(wr_en_acq), .wr_en_iocq(wr_en_iocq),
    .wr_beat_addr(wr_beat_addr), .wr_data(wr_data), .wr_be(wr_be), .drop_cnt_inc(drop_cnt_inc));

  // ---------------- DUT: rams ----------------
  logic         asq_wr_en = 0, iosq_wr_en = 0, clear_cqs = 0, clear_done;
  logic [6:0]   asq_wr_addr = 0;
  logic [7:0]   iosq_wr_addr = 0;
  logic [127:0] asq_wr_data = 0, iosq_wr_data = 0;
  logic         cpl_rd_en = 0, cpl_rd_en_data;
  logic [1:0]   cpl_rd_sel = 0;
  logic [31:0]  cpl_rd_addr = 0;
  logic [127:0] cpl_rd_data, data_rd_data;
  logic         acq_rd_en = 0, iocq_rd_en = 0, ident_rd_en = 0;
  logic [4:0]   acq_rd_addr = 0;
  logic [5:0]   iocq_rd_addr = 0;
  logic [7:0]   ident_rd_addr = 0;
  logic [127:0] acq_rd_data, iocq_rd_data, ident_rd_data;

  hostmem_rams #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES)) dut_r (
    .clk(clk), .rst(rst),
    .asq_wr_en(asq_wr_en), .asq_wr_addr(asq_wr_addr), .asq_wr_data(asq_wr_data),
    .iosq_wr_en(iosq_wr_en), .iosq_wr_addr(iosq_wr_addr), .iosq_wr_data(iosq_wr_data),
    .hw_wr_en_ident(wr_en_ident), .hw_wr_en_acq(wr_en_acq), .hw_wr_en_iocq(wr_en_iocq),
    .hw_wr_beat_addr(wr_beat_addr), .hw_wr_data(wr_data), .hw_wr_be(wr_be),
    .clear_cqs(clear_cqs), .clear_done(clear_done),
    .cpl_rd_en(cpl_rd_en), .cpl_rd_sel(cpl_rd_sel), .cpl_rd_addr(cpl_rd_addr),
    .cpl_rd_data(cpl_rd_data), .cpl_rd_en_data(cpl_rd_en_data), .data_rd_data(data_rd_data),
    .acq_rd_en(acq_rd_en), .acq_rd_addr(acq_rd_addr), .acq_rd_data(acq_rd_data),
    .iocq_rd_en(iocq_rd_en), .iocq_rd_addr(iocq_rd_addr), .iocq_rd_data(iocq_rd_data),
    .ident_rd_en(ident_rd_en), .ident_rd_addr(ident_rd_addr), .ident_rd_data(ident_rd_data));

  // data ring stand-in: pattern of the address, 2-cycle latency
  function automatic logic [127:0] data_pat(input logic [31:0] a);
    return {~a, a ^ 32'hC0DE_0000, a * 32'd3, 32'hD474_0000 | a[15:0]};
  endfunction
  logic [127:0] dr1, dr2;
  always @(posedge clk) begin
    dr1 <= cpl_rd_en_data ? data_pat(cpl_rd_addr) : {4{32'hBAD0_BAD0}};
    dr2 <= dr1;
  end
  assign data_rd_data = dr2;

  // ---------------- error handling ----------------
  int errors = 0;
  task automatic fail(input string msg);
    errors++;
    $display("[%0t] ERROR: %s", $time, msg);
    if (errors > 20) $fatal(1, "too many errors");
  endtask

  // ---------------- reference memories (byte level) ----------------
  logic [7:0] ref_mem  [3][4096];      // [0]=ident [1]=acq [2]=iocq
  logic [7:0] port_mem [3][4096];
  int         region_size [3] = '{4096, 512, 1024};
  int         exp_drops = 0, got_drops = 0;
  int         n_pkts = 0, n_written_pkts = 0, n_beats = 0;

  function automatic logic [31:0] region_base(input int r);
    case (r) 0: return IDENT_BASE; 1: return ACQ_BASE; default: return IOCQ_BASE; endcase
  endfunction

  // ---------------- write-port monitor ----------------
  always @(posedge clk) if (!rst) begin
    if (drop_cnt_inc) got_drops++;
    if ((wr_en_ident + wr_en_acq + wr_en_iocq) > 1) fail("more than one wr_en");
    if (wr_en_ident || wr_en_acq || wr_en_iocq) begin
      int r;
      r = wr_en_ident ? 0 : wr_en_acq ? 1 : 2;
      n_beats++;
      if (wr_beat_addr[3:0] != 4'h0) fail($sformatf("wr_beat_addr %h not 16 B aligned", wr_beat_addr));
      if (wr_beat_addr + 16 > region_size[r]) fail($sformatf("wr_beat_addr %h beyond region %0d", wr_beat_addr, r));
      else
        for (int b = 0; b < 16; b++)
          if (wr_be[b]) port_mem[r][wr_beat_addr + b] = wr_data[8*b +: 8];
    end
  end

  // ---------------- MWr driver ----------------
  int gap_prob = 0;      // % chance of an idle cycle before each beat

  task automatic push_beat(input logic [127:0] d, input logic [15:0] k, input logic last, input logic [1:0] u);
    while (gap_prob != 0 && $urandom_range(0, 99) < gap_prob) begin
      s_mwr_tvalid <= 1'b0;
      @(posedge clk);
    end
    s_mwr_tdata <= d; s_mwr_tkeep <= k; s_mwr_tlast <= last; s_mwr_tuser <= u; s_mwr_tvalid <= 1'b1;
    do @(posedge clk); while (!s_mwr_tready);
    s_mwr_tvalid <= 1'b0;
  endtask

  // send one MWr and update the reference model
  //   region: 0 ident, 1 acq, 2 iocq, 3 = raw address given in `raw_addr`
  task automatic send_mwr(input int region, input int rel, input int len,
                          input logic [3:0] fbe, input logic [3:0] lbe,
                          input bit poison = 0, input bit is4dw = 0, input bit hi_nz = 0,
                          input logic [31:0] raw_addr = 0, input bit zero_data = 0);
    logic [31:0] addr;
    logic [127:0] hdr;
    logic [31:0] dw [];
    bit expect_write;
    int total_beats;
    if (region == 3) addr = raw_addr; else addr = region_base(region) + rel;
    hdr = hdr_mwr32(addr, 10'(len), 16'h0100, 8'(n_pkts), fbe, lbe);
    if (is4dw) begin
      hdr[31:24] = FT_MWR64;
      hdr[95:64] = hi_nz ? 32'h0000_0003 : 32'h0;
      hdr[127:96] = addr;
    end
    dw = new[len];
    for (int i = 0; i < len; i++) dw[i] = zero_data ? 32'h0 : $urandom;
    // does it land in a region? (independent decode)
    expect_write = !poison && !hi_nz;
    if (region == 3) begin
      expect_write = 0;
      for (int r = 0; r < 3; r++)
        if (!poison && !hi_nz && addr >= region_base(r) && (addr + len*4) <= region_base(r) + region_size[r]) begin
          expect_write = 1; region = r; rel = addr - region_base(r);
        end
    end
    if (expect_write) begin
      for (int i = 0; i < len; i++) begin
        logic [3:0] be;
        be = (i == 0) ? fbe : (i == len - 1) ? lbe : 4'hF;
        if (len == 1) be = fbe;
        for (int b = 0; b < 4; b++)
          if (be[b]) ref_mem[region][rel + 4*i + b] = dw[i][8*b +: 8];
      end
      n_written_pkts++;
    end else exp_drops++;
    n_pkts++;
    // beats
    push_beat(hdr, 16'hFFFF, 1'b0, {1'b0, poison});
    total_beats = (len + 3) / 4;
    for (int bt = 0; bt < total_beats; bt++) begin
      logic [127:0] d;
      logic [15:0]  k;
      d = '0; k = '0;
      for (int j = 0; j < 4; j++)
        if (4*bt + j < len) begin d[32*j +: 32] = dw[4*bt + j]; k[4*j +: 4] = 4'hF; end
      push_beat(d, k, (bt == total_beats - 1), {1'b0, poison});
    end
  endtask

  function automatic logic [3:0] rnd_fbe(input int len);
    if (len == 1) return ($urandom_range(0, 9) == 0) ? 4'h0 : ($urandom_range(0, 2) == 0) ? 4'hF : 4'($urandom_range(1, 15));
    return ($urandom_range(0, 9) < 6) ? 4'hF : 4'($urandom_range(1, 15));
  endfunction
  function automatic logic [3:0] rnd_lbe(input int len);
    if (len == 1) return 4'h0;
    return ($urandom_range(0, 9) < 6) ? 4'hF : 4'($urandom_range(1, 15));
  endfunction

  task automatic send_random_mwr();
    int region, len, maxdw, rel, kind;
    bit poison, is4, hinz;
    kind = $urandom_range(0, 99);
    region = $urandom_range(0, 2);
    maxdw = region_size[region] / 4;
    poison = 0; is4 = 0; hinz = 0;
    case (1)
      kind < 40: len = $urandom_range(1, 4);
      kind < 70: len = $urandom_range(5, 16);
      default:   len = $urandom_range(17, 64);
    endcase
    if (len > maxdw) len = maxdw;
    rel = 4 * $urandom_range(0, maxdw - len);
    if (kind >= 90 && kind < 93) poison = 1;
    if (kind >= 93 && kind < 96) is4 = 1;
    if (kind == 96) begin is4 = 1; hinz = 1; end
    if (kind >= 97) begin
      // unmapped / straddling
      case ($urandom_range(0, 5))
        0: send_mwr(3, 0, len, rnd_fbe(len), rnd_lbe(len), 0, 0, 0, 32'hFEE0_0000);                      // MSI
        1: send_mwr(3, 0, len, rnd_fbe(len), rnd_lbe(len), 0, 0, 0, ASQ_BASE + 4 * $urandom_range(0, 8));
        2: send_mwr(3, 0, 4, 4'hF, 4'hF, 0, 0, 0, IDENT_BASE + 4096 - 8);                               // straddles end
        3: send_mwr(3, 0, 8, 4'hF, 4'hF, 0, 0, 0, ACQ_BASE + ACQ_BYTES - 16);
        4: send_mwr(3, 0, len, rnd_fbe(len), rnd_lbe(len), 0, 0, 0, DATA_BASE);
        default: send_mwr(3, 0, 2, 4'hF, 4'hF, 0, 0, 0, IOCQ_BASE + IOCQ_BYTES - 4);
      endcase
      return;
    end
    send_mwr(region, rel, len, rnd_fbe(len), rnd_lbe(len), poison, is4, hinz);
  endtask

  // ---------------- read-back helpers ----------------
  task automatic readback_cmp(input string name);
    logic [127:0] got [3][256];
    // issue reads on all three ports, one beat per cycle
    for (int n = 0; n < 260; n++) begin
      @(posedge clk);
      ident_rd_en <= (n < 256); ident_rd_addr <= 8'(n);
      acq_rd_en   <= (n < 32);  acq_rd_addr   <= 5'(n);
      iocq_rd_en  <= (n < 64);  iocq_rd_addr  <= 6'(n);
      // capture data of the read issued two cycles earlier
      if (n >= 3) begin
        got[0][n-3] = ident_rd_data;
        if (n-3 < 32) got[1][n-3] = acq_rd_data;
        if (n-3 < 64) got[2][n-3] = iocq_rd_data;
      end
    end
    @(posedge clk); ident_rd_en <= 0; acq_rd_en <= 0; iocq_rd_en <= 0;
    // note: data captured at the edge following the 2-cycle latency
    for (int r = 0; r < 3; r++) begin
      int nb;
      nb = region_size[r] / 16;
      for (int bt = 0; bt < nb; bt++)
        for (int b = 0; b < 16; b++)
          if (got[r][bt][8*b +: 8] !== ref_mem[r][16*bt + b]) begin
            fail($sformatf("%s: RAM %0d byte %0d: got %h exp %h", name, r, 16*bt + b, got[r][bt][8*b +: 8], ref_mem[r][16*bt + b]));
            bt = nb; break;
          end
    end
  endtask

  task automatic compare_port_ref(input string name);
    for (int r = 0; r < 3; r++)
      for (int a = 0; a < region_size[r]; a++)
        if (port_mem[r][a] !== ref_mem[r][a]) begin
          fail($sformatf("%s: write-port memory %0d byte %0d: got %h exp %h", name, r, a, port_mem[r][a], ref_mem[r][a]));
          break;
        end
  endtask

  // ---------------- Part 3 helpers: completer read mux ----------------
  logic [127:0] asq_ref [128];
  logic [127:0] iosq_ref [256];
  function automatic logic [127:0] prp_ref(input logic [31:0] a);
    longint unsigned k, i, pages;
    logic [127:0] v;
    k = a[31:12]; i = a[11:3];
    pages = SLOT_BYTES / 4096;
    v = '0;
    for (int l = 0; l < 2; l++)
      if (k < N_SLOTS && (i + l + 1) < pages)
        v[64*l +: 64] = 64'(DATA_BASE) + k * SLOT_BYTES + (i + l + 1) * 4096;
    return v;
  endfunction

  // ---------------- main ----------------
  initial begin
    for (int r = 0; r < 3; r++)
      for (int a = 0; a < 4096; a++) begin ref_mem[r][a] = 8'h00; port_mem[r][a] = 8'h00; end
    s_mwr_tvalid = 0; s_mwr_tdata = 0; s_mwr_tkeep = 0; s_mwr_tlast = 0; s_mwr_tuser = 0;
    repeat (4) @(posedge clk);
    rst <= 1'b0;
    repeat (4) @(posedge clk);

    // ===== Part 1 =====
    // power-up clear of the CQ RAMs (their contents are unknown until cleared)
    @(posedge clk); clear_cqs <= 1'b1;
    @(posedge clk); clear_cqs <= 1'b0;
    repeat (80) @(posedge clk);
    if (clear_done !== 1'b1) fail("power-up clear did not finish");
    // identify buffer zero fill (Length = 1024 -> encoded 0), then random traffic
    send_mwr(0, 0, 1024, 4'hF, 4'hF, 0, 0, 0, 0, 1);
    // CQE shaped writes
    for (int n = 0; n < 40; n++) begin
      int reg_, ncqe, rel;
      reg_ = 1 + $urandom_range(0, 1);
      ncqe = $urandom_range(1, 3);
      rel  = 16 * $urandom_range(0, region_size[reg_] / 16 - ncqe);
      send_mwr(reg_, rel, 4 * ncqe, 4'hF, 4'hF);
    end
    // split 8+8 CQE writes (phase bit written last)
    for (int n = 0; n < 40; n++) begin
      int reg_, rel;
      reg_ = 1 + $urandom_range(0, 1);
      rel  = 16 * $urandom_range(0, region_size[reg_] / 16 - 1);
      send_mwr(reg_, rel,     2, 4'hF, 4'hF);
      send_mwr(reg_, rel + 8, 2, 4'hF, 4'hF);
    end
    gap_prob = 0;
    for (int n = 0; n < 600; n++) send_random_mwr();       // back-to-back
    gap_prob = 40;
    for (int n = 0; n < 600; n++) send_random_mwr();       // with idle cycles inside packets
    gap_prob = 0;
    repeat (20) @(posedge clk);
    if (got_drops != exp_drops) fail($sformatf("drop count %0d expected %0d", got_drops, exp_drops));
    compare_port_ref("part1");
    readback_cmp("part1");
    $display("Part 1: %0d MWr (%0d written, %0d dropped), %0d memory beats, errors=%0d",
             n_pkts, n_written_pkts, exp_drops, n_beats, errors);

    // ===== Part 2: clear FSM =====
    begin
      int t;
      // dirty the CQs first
      for (int n = 0; n < 32; n++) send_mwr(1, 16 * (n % 32), 4, 4'hF, 4'hF);
      for (int n = 0; n < 64; n++) send_mwr(2, 16 * (n % 64), 4, 4'hF, 4'hF);
      // expected state after clear: CQs zero, Identify untouched
      @(posedge clk); clear_cqs <= 1'b1;
      @(posedge clk); clear_cqs <= 1'b0;
      @(posedge clk);
      if (clear_done !== 1'b0) fail("clear_done not low after clear_cqs");
      t = 0;
      while (clear_done !== 1'b1 && t < 200) begin @(posedge clk); t++; end
      if (t >= 200) fail("clear_done never asserted");
      $display("clear_done after %0d cycles", t + 2);
      for (int r = 1; r < 3; r++)
        for (int a = 0; a < region_size[r]; a++) begin ref_mem[r][a] = 8'h00; port_mem[r][a] = 8'h00; end
      readback_cmp("after clear");
      // normal writes still work
      for (int n = 0; n < 50; n++) send_random_mwr();
      repeat (20) @(posedge clk);
      compare_port_ref("part2");
      readback_cmp("part2");
      $display("Part 2 done, errors=%0d", errors);
    end

    // ===== Part 3: completer read mux =====
    for (int n = 0; n < 128; n++) begin
      asq_ref[n] = {$urandom, $urandom, $urandom, $urandom};
      @(posedge clk); asq_wr_en <= 1; asq_wr_addr <= 7'(n); asq_wr_data <= asq_ref[n];
    end
    for (int n = 0; n < 256; n++) begin
      iosq_ref[n] = {$urandom, $urandom, $urandom, $urandom};
      @(posedge clk); iosq_wr_en <= 1; iosq_wr_addr <= 8'(n); iosq_wr_data <= iosq_ref[n];
    end
    @(posedge clk); asq_wr_en <= 0; iosq_wr_en <= 0;
    begin
      logic [127:0] exp_q [$];
      logic         v_q [$];
      logic         en_q [$];
      automatic int checked = 0;
      // pipelined random reads, 40 % idle cycles
      for (int n = 0; n < 6000; n++) begin
        logic [1:0] sel;
        logic [31:0] a;
        bit en;
        en = ($urandom_range(0, 9) < 6);
        sel = $urandom_range(0, 3);
        case (sel)
          RSEL_ASQ:  a = 16 * $urandom_range(0, 127);
          RSEL_IOSQ: a = 16 * $urandom_range(0, 255);
          RSEL_PRP:  a = 16 * ($urandom_range(0, 2 * N_SLOTS * 256 - 1));
          default:   a = 16 * $urandom_range(0, 65535);
        endcase
        if (sel == RSEL_PRP && $urandom_range(0, 3) == 0) a = 16 * $urandom_range(0, 255) + 32'h1000 * $urandom_range(0, 5);
        // drive on this edge
        @(posedge clk);
        // check data of the read issued three edges ago (rd_en set after edge n, sampled at n+1, data after n+2)
        if (v_q.size() >= 3) begin
          logic [127:0] e; logic ev;
          e = exp_q[0]; ev = v_q[0];
          if (ev) begin
            checked++;
            if (cpl_rd_data !== e) fail($sformatf("cpl_rd_data got %h exp %h", cpl_rd_data, e));
          end
          exp_q.pop_front(); v_q.pop_front();
        end
        cpl_rd_en <= en; cpl_rd_sel <= sel; cpl_rd_addr <= a;
        // expected value for the read being issued now
        case (sel)
          RSEL_ASQ:  exp_q.push_back(asq_ref[a[10:4]]);
          RSEL_IOSQ: exp_q.push_back(iosq_ref[a[11:4]]);
          RSEL_PRP:  exp_q.push_back(prp_ref(a));
          default:   exp_q.push_back(data_pat(a));
        endcase
        v_q.push_back(en);
      end
      @(posedge clk); cpl_rd_en <= 0;
      $display("Part 3: %0d completer-mux reads checked, errors=%0d", checked, errors);
    end
    // cpl_rd_en_data only for RSEL_DATA
    @(posedge clk); cpl_rd_en <= 1; cpl_rd_sel <= RSEL_ASQ;
    @(posedge clk); #1 if (cpl_rd_en_data !== 1'b0) fail("cpl_rd_en_data for ASQ read");
    cpl_rd_sel <= RSEL_DATA;
    @(posedge clk); #1 if (cpl_rd_en_data !== 1'b1) fail("cpl_rd_en_data missing for DATA read");
    cpl_rd_en <= 0;
    @(posedge clk); #1 if (cpl_rd_en_data !== 1'b0) fail("cpl_rd_en_data without cpl_rd_en");

    repeat (5) @(posedge clk);
    $display("SUMMARY: errors=%0d", errors);
    if (errors != 0) $fatal(1, "TEST FAILED");
    $display("TEST PASSED");
    $finish;
  end

  initial begin
    #50ms;
    $fatal(1, "TIMEOUT");
  end
endmodule
