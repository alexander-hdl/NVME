// =============================================================================
// tb_verify_checker.sv - self-checking testbench for rtl/nvme/verify_checker.sv
//
// Three harness instances (vc_harness) run in parallel with different geometry:
//   V0  4 slots x 4 KiB,   LBA 512 B,  START_LBA 0
//   V1  8 slots x 16 KiB,  LBA 4 KiB,  START_LBA 0x1_0000_0123 (upper bits must not matter)
//   V2  1 slot  x 4 KiB,   LBA 2 KiB,  START_LBA 0xFFFF_FFFF_FFFF_FF00 (the LBA difference wraps)
// Every harness drives random beats (random slot / 16 B offset, random slot_lba table with
// 64-bit values, random byte enables - disabled bytes carry garbage - random idle cycles)
// carrying the pattern {~n, n ^ A5A5A5A5, n*7, n} of the reference model, injects single-byte
// errors into a few percent of the beats, and compares chk_bytes / err_beats / first_err_*
// / err_seen with a scoreboard.  Phases: (1) mixed traffic with errors, (2) `clear` returns every
// output to zero (also when beats are still in the pipeline), (3) clean traffic: no error,
// err_seen = 0, (4) error in the very first beat after a clear, (5) the pattern n wraps around
// 2^32 inside a slot, (6) back-to-back beats without gaps, one error in the middle, beats with
// all byte enables low.
// =============================================================================
`timescale 1ns/1ps

module vc_harness #(
  parameter int N_SLOTS    = 4,
  parameter int SLOT_BYTES = 4096,
  parameter int LBADS      = 9,
  parameter logic [63:0] START_LBA = 64'h0,
  parameter int SEED_OFF   = 0
) (
  input  logic clk,
  output logic done,
  output int   errors
);
  logic        rst = 1'b1;
  logic        clear = 1'b0;
  logic [3:0]  lbads = 4'(LBADS);
  logic [N_SLOTS*64-1:0] slot_lba_flat = '0;
  logic        wr_en = 1'b0;
  logic [31:0] wr_beat_addr = '0;
  logic [127:0] wr_data = '0;
  logic [15:0] wr_be = '0;
  logic [63:0] chk_bytes, first_err_lba;
  logic [31:0] err_beats, first_err_exp, first_err_got;
  logic        err_seen;

  verify_checker #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES), .START_LBA(START_LBA)) dut (
    .clk(clk), .rst(rst), .clear(clear), .lbads(lbads), .slot_lba_flat(slot_lba_flat),
    .wr_en(wr_en), .wr_beat_addr(wr_beat_addr), .wr_data(wr_data), .wr_be(wr_be),
    .chk_bytes(chk_bytes), .err_beats(err_beats), .first_err_lba(first_err_lba),
    .first_err_exp(first_err_exp), .first_err_got(first_err_got), .err_seen(err_seen));

  initial errors = 0;
  task automatic fail_msg(input string m);
    errors++;
    $display("[%0t] V(lbads %0d, %0d slots) ERROR: %s", $time, LBADS, N_SLOTS, m);
  endtask
  task automatic check(input bit c, input string m);
    if (!c) fail_msg(m);
  endtask

  // ---- reference model ----
  logic [63:0] lba_tab [N_SLOTS];

  function automatic logic [31:0] beat_index(input int k, input int off);
    logic [63:0] b;
    b = ((lba_tab[k] - START_LBA) << LBADS) + 64'(off);
    return b[35:4];
  endfunction

  function automatic logic [127:0] pattern(input logic [31:0] n);
    return {~n, n ^ 32'hA5A5A5A5, n * 32'd7, n};
  endfunction

  longint      sc_bytes;
  int          sc_err_beats;
  bit          sc_seen;
  logic [63:0] sc_lba;
  logic [31:0] sc_exp, sc_got;

  task automatic sc_clear();
    sc_bytes = 0; sc_err_beats = 0; sc_seen = 0; sc_lba = 0; sc_exp = 0; sc_got = 0;
  endtask

  // random slot table; near_wrap: n of slot k starts below 2^32 so that it wraps inside the slot
  task automatic randomize_table(input bit near_wrap = 0);
    longint unsigned lbas, unit;
    lbas = SLOT_BYTES >> LBADS;
    unit = 1 << (LBADS - 4);                  // beats per LBA
    for (int k = 0; k < N_SLOTS; k++) begin
      lba_tab[k] = START_LBA + 64'($urandom_range(0, 1 << 20)) * 64'(lbas);
      if (k % 3 == 2) lba_tab[k][63:40] = $urandom;                // exotic upper bits (n only needs the low ones)
      if (near_wrap)
        lba_tab[k] = START_LBA + (64'h1_0000_0000 / unit) - 64'(lbas / 2) - 64'(k);
      slot_lba_flat[64*k +: 64] = lba_tab[k];
    end
  endtask

  // Drive one beat and update the scoreboard.
  task automatic drive_beat(input int k, input int boff, input bit inject, input bit full_be, input bit gap_after);
    logic [127:0] exp_d, got_d;
    logic [15:0]  be;
    int           off;
    bit           bad_dw [4];
    off   = boff * 16;
    exp_d = pattern(beat_index(k, off));
    got_d = exp_d;
    be    = full_be ? 16'hFFFF : 16'($urandom);
    for (int i = 0; i < 16; i++) if (!be[i]) got_d[8*i +: 8] = $urandom;      // disabled bytes carry garbage
    if (inject) begin
      int bi;
      bi = $urandom_range(0, 15);
      got_d[8*bi +: 8] = got_d[8*bi +: 8] ^ 8'(1 << $urandom_range(0, 7));
    end
    for (int j = 0; j < 4; j++) begin
      bad_dw[j] = 0;
      for (int b = 0; b < 4; b++)
        if (be[4*j + b] && got_d[32*j + 8*b +: 8] != exp_d[32*j + 8*b +: 8]) bad_dw[j] = 1;
    end
    for (int i = 0; i < 16; i++) if (be[i]) sc_bytes++;
    if (bad_dw[0] || bad_dw[1] || bad_dw[2] || bad_dw[3]) begin
      sc_err_beats++;
      if (!sc_seen) begin
        int jf;
        jf = bad_dw[0] ? 0 : bad_dw[1] ? 1 : bad_dw[2] ? 2 : 3;
        sc_seen = 1;
        sc_lba  = lba_tab[k] + 64'(off >> LBADS);
        sc_exp  = exp_d[32*jf +: 32];
        sc_got  = got_d[32*jf +: 32];
      end
    end
    wr_en <= 1'b1; wr_beat_addr <= 32'(k * SLOT_BYTES + off); wr_data <= got_d; wr_be <= be;
    @(posedge clk);
    wr_en <= 1'b0; wr_data <= {4{32'hDEAD_BEEF}}; wr_be <= 16'hFFFF;         // junk while idle (must be ignored)
    if (gap_after) repeat ($urandom_range(1, 4)) @(posedge clk);
  endtask

  task automatic traffic(input int n, input int err_pct, input bit gaps, input bit allow_partial = 1);
    for (int i = 0; i < n; i++)
      drive_beat($urandom_range(0, N_SLOTS - 1), $urandom_range(0, SLOT_BYTES / 16 - 1),
                 ($urandom_range(0, 99) < err_pct), !allow_partial || ($urandom_range(0, 99) < 60),
                 gaps && ($urandom_range(0, 2) == 0));
  endtask

  task automatic settle_and_compare(input string ph);
    repeat (14) @(posedge clk);
    check(chk_bytes == 64'(sc_bytes), $sformatf("%s: chk_bytes %0d expected %0d", ph, chk_bytes, sc_bytes));
    check(err_beats == 32'(sc_err_beats), $sformatf("%s: err_beats %0d expected %0d", ph, err_beats, sc_err_beats));
    check(err_seen == sc_seen, $sformatf("%s: err_seen %0b expected %0b", ph, err_seen, sc_seen));
    check(first_err_lba == sc_lba, $sformatf("%s: first_err_lba %h expected %h", ph, first_err_lba, sc_lba));
    check(first_err_exp == sc_exp, $sformatf("%s: first_err_exp %h expected %h", ph, first_err_exp, sc_exp));
    check(first_err_got == sc_got, $sformatf("%s: first_err_got %h expected %h", ph, first_err_got, sc_got));
  endtask

  task automatic do_clear();
    clear <= 1'b1;
    @(posedge clk);
    clear <= 1'b0;
    sc_clear();
  endtask

  initial begin
    done = 1'b0;
    sc_clear();
    repeat (6) @(posedge clk);
    rst <= 1'b0;
    repeat (3) @(posedge clk);
    settle_and_compare("reset");

    // ---- phase 1: mixed traffic with errors and gaps ----
    randomize_table();
    @(posedge clk);
    traffic(4000, 3, 1);
    settle_and_compare("phase 1");
    check(sc_err_beats > 20 && sc_seen, "phase 1: scoreboard saw too few errors (test too weak)");
    $display("V(lbads %0d, %0d slots) phase 1: %0d bytes, %0d bad beats, first bad LBA %0d, exp %h got %h",
             LBADS, N_SLOTS, sc_bytes, sc_err_beats, sc_lba, sc_exp, sc_got);

    // ---- phase 2: clear, also with beats still in the pipeline ----
    do_clear();
    repeat (14) @(posedge clk);
    check(chk_bytes == 0 && err_beats == 0 && !err_seen && first_err_lba == 0 && first_err_exp == 0 && first_err_got == 0,
          "phase 2: outputs not zero after clear");
    randomize_table();
    @(posedge clk);
    traffic(6, 100, 0);                       // 6 bad beats ...
    do_clear();                               // ... flushed by clear while still in the pipeline
    repeat (14) @(posedge clk);
    settle_and_compare("phase 2b (clear flushes the pipeline)");
    check(chk_bytes == 0 && err_beats == 0 && !err_seen, "phase 2b: pipeline content survived clear");

    // ---- phase 3: clean traffic ----
    randomize_table();
    @(posedge clk);
    traffic(3000, 0, 0);
    settle_and_compare("phase 3");
    check(!err_seen && err_beats == 0 && chk_bytes != 0, "phase 3: clean traffic flagged / not counted");

    // ---- phase 4: error in the very first beat after a clear ----
    do_clear();
    drive_beat(N_SLOTS - 1, SLOT_BYTES / 16 - 1, 1, 1, 0);       // last beat of the last slot, enabled byte flipped
    traffic(50, 0, 0, 0);
    settle_and_compare("phase 4");
    check(err_beats == 1 && err_seen, "phase 4: exactly one bad beat expected");

    // ---- phase 5: n wraps around 2^32 inside a slot ----
    do_clear();
    randomize_table(1);
    @(posedge clk);
    traffic(1500, 2, 1);
    settle_and_compare("phase 5 (wrap)");

    // ---- phase 6: sustained back-to-back beats, one error in the middle, byte enable 0 beats ----
    do_clear();
    randomize_table();
    @(posedge clk);
    for (int i = 0; i < 2000; i++)
      drive_beat(i % N_SLOTS, (i / N_SLOTS) % (SLOT_BYTES / 16), (i == 1000), 1, 0);
    settle_and_compare("phase 6");
    check(err_beats == 1, "phase 6: exactly one bad beat expected");
    for (int i = 0; i < 20; i++) begin
      wr_en <= 1'b1; wr_beat_addr <= 32'(16 * i); wr_data <= {4{32'h1234_5678}}; wr_be <= 16'h0000;
      @(posedge clk);
    end
    wr_en <= 1'b0;
    settle_and_compare("phase 6b (be=0)");
    done = 1'b1;
  end
endmodule

module tb_verify_checker;
  logic clk = 1'b0;
  always #4 clk = ~clk;

  logic [2:0] done;
  int e0, e1, e2;

  vc_harness #(.N_SLOTS(4), .SLOT_BYTES(4096),  .LBADS(9),  .START_LBA(64'h0), .SEED_OFF(0))
    v0 (.clk(clk), .done(done[0]), .errors(e0));
  vc_harness #(.N_SLOTS(8), .SLOT_BYTES(16384), .LBADS(12), .START_LBA(64'h0000_0001_0000_0123), .SEED_OFF(1))
    v1 (.clk(clk), .done(done[1]), .errors(e1));
  vc_harness #(.N_SLOTS(1), .SLOT_BYTES(4096),  .LBADS(11), .START_LBA(64'hFFFF_FFFF_FFFF_FF00), .SEED_OFF(2))
    v2 (.clk(clk), .done(done[2]), .errors(e2));

  initial begin
    wait (&done);
    repeat (10) @(posedge clk);
    if (e0 + e1 + e2 == 0) $display("TEST PASSED");
    else $fatal(1, "TEST FAILED: %0d errors", e0 + e1 + e2);
    $finish;
  end
  initial begin
    #50ms;
    $fatal(1, "TEST FAILED: global watchdog");
  end
endmodule
