// =============================================================================
// tb_rec_verify.sv — WP6: vrf_tap (user_clk 125 MHz -> async FIFO -> ui_clk 100.3 MHz) + rec_verify.
// Reference blocks: sim/vectors/vrf_blocks.hex (tools/gen_vrf_blocks.py, built with tools/rec_decode.py
// build_block; NOT from WP3).  idx 0..63 clean chain (sid C0DE0001, lost=3 from 10, TRUNC 58, PAD 59-63,
// LAST 63), 64 magic, 65 ver, 66 crc, 67 payload bit, 68 seq gap, 69 duplicate of 62, 70 sid, 71 magic_end,
// 72 pos_lo, 73 seq_lo, 74 valid>252.
// Scenarios: clean stream at line rate (bursts, uneven clock ratio, two ui clock periods), lbads 9..12,
// interleaved slots, every corruption type, PAD/TRUNC/LAST, last valid block after truncation, hole
// (mvp > fbp), missing LAST, out-of-order, partial beats, byte-enable masking, FIFO overflow + recovery,
// negative control of the checker itself.  Mutation control is done from outside (see the WP6 report).
// =============================================================================
`timescale 1ns/1ps

module tb_rec_verify;
  import rec_pkg::*;

  localparam int N_SLOTS    = 4;
  localparam int SLOT_BYTES = 131072;
  localparam logic [63:0] START_LBA = 64'h0000_0000_FFFF_FF00;   // low 32 bit wrap inside the test
  localparam logic [31:0] SID = 32'hC0DE0001;
  localparam int NBLK = 75;
  localparam int BLK_GAP = 80;

  // ---------------------------------------------------------------- clocks / resets
  logic uclk = 0, vclk = 0, urst = 1, vrst = 1;
  real  vper = 9.97;
  always #4.0 uclk = ~uclk;                       // 125 MHz
  always begin #(vper/2.0) vclk = ~vclk; end      // ~100.3 MHz (changed in one test)

  // ---------------------------------------------------------------- DUT
  logic                  wr_en = 0, enable = 0;
  logic [31:0]           wr_beat_addr = 0;
  logic [127:0]          wr_data = 0;
  logic [15:0]           wr_be = 0;
  logic [N_SLOTS*64-1:0] slot_lba_flat = '0;
  logic [192:0]          m_tdata;
  logic                  m_tvalid, m_tready, tap_ovf;
  logic                  clear = 0;
  logic [3:0]            lbads = 4'd12;
  logic [31:0]           exp_sid = SID;
  vrf_res_t              res;
  logic                  end_found;
  logic [7:0]            mvp_flags;

  vrf_tap #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES)) u_vrf_tap (
    .uclk(uclk), .urst(urst), .wr_en(wr_en), .wr_beat_addr(wr_beat_addr), .wr_data(wr_data),
    .wr_be(wr_be), .slot_lba_flat(slot_lba_flat), .enable(enable),
    .vclk(vclk), .vrst(vrst), .m_tdata(m_tdata), .m_tvalid(m_tvalid), .m_tready(m_tready), .ovf(tap_ovf));

  rec_verify #(.START_LBA(START_LBA)) u_rec_verify (
    .clk(vclk), .rst(vrst), .clear(clear), .exp_sid(exp_sid), .lbads(lbads),
    .s_tdata(m_tdata), .s_tvalid(m_tvalid), .s_tready(m_tready), .tap_ovf(tap_ovf),
    .res(res), .end_found(end_found), .mvp_flags(mvp_flags));

  // ---------------------------------------------------------------- reference blocks
  logic [127:0] blk [0:NBLK*256-1];
  logic [127:0] tmp [0:32*256-1];

  // ---------------------------------------------------------------- checking helpers
  int errors = 0;
  int neg_det = 0;
  bit neg_mode = 0;

  task automatic fail(input string msg);
    if (neg_mode) neg_det++;
    else begin errors++; $display("ERROR: %s", msg); end
  endtask

  task automatic ck(input string nm, input string what, input longint got, input longint exp);
    if (exp >= 0 && got !== exp) fail($sformatf("%s: %s got %0d exp %0d", nm, what, got, exp));
  endtask

  // -1 = do not care
  task automatic chk(input string nm, input longint ok, input longint bad, input longint crc,
                     input longint pe, input longint ooo, input longint prt, input longint ve);
    ck(nm, "ok",  longint'(res.ok),  ok);
    ck(nm, "bad", longint'(res.bad), bad);
    ck(nm, "crc", longint'(res.crc), crc);
    ck(nm, "pe",  longint'(res.pe),  pe);
    ck(nm, "ooo", longint'(res.ooo), ooo);
    ck(nm, "prt", longint'(res.prt), prt);
    ck(nm, "ve",  longint'(res.ve),  ve);
  endtask

  // ---------------------------------------------------------------- stimulus helpers
  int  ov_j = -1;                   // beat whose data/byte enables are overridden
  logic [127:0] ov_xor = '0;
  logic [15:0]  ov_be  = 16'hFFFF;

  task automatic set_slot(input int ap);
    int slot;
    longint unsigned base, lba;
    slot = (ap >> 5) % N_SLOTS;
    base = longint'(ap & ~31);
    lba  = longint'(START_LBA[31:0]) + ((base * 4096) >> lbads);
    slot_lba_flat[slot*64 +: 64] <= {32'hA5A50000 + 32'(slot), lba[31:0]};
  endtask

  // one beat; idx < 0 = all-zero data
  task automatic beat(input int idx, input int ap, input int j);
    logic [127:0] d;
    logic [15:0]  be;
    int slot;
    slot = (ap >> 5) % N_SLOTS;
    d  = (idx < 0) ? 128'd0 : blk[idx*256 + j];
    be = 16'hFFFF;
    if (j == ov_j) begin d = d ^ ov_xor; be = ov_be; end
    @(posedge uclk);
    wr_en <= 1'b1;
    wr_beat_addr <= 32'(slot * SLOT_BYTES + (ap % 32) * 4096 + j * 16);
    wr_data <= d;
    wr_be   <= be;
  endtask

  task automatic idle(input int n);
    repeat (n) begin @(posedge uclk); wr_en <= 1'b0; end
  endtask

  task automatic send(input int idx, input int ap, input int j0, input int j1);
    set_slot(ap);
    for (int j = j0; j <= j1; j++) beat(idx, ap, j);
  endtask

  task automatic send_blk(input int idx, input int ap);
    send(idx, ap, 0, 255);
    idle(BLK_GAP);                  // average rate <= 0.8 beat/user_clk (< 100.3/125 of the consumer)
  endtask

  task automatic send_two(input int ia, input int apa, input int ib, input int apb);
    set_slot(apa); set_slot(apb);
    for (int j = 0; j < 256; j++) begin beat(ia, apa, j); beat(ib, apb, j); end
    idle(2*BLK_GAP);
  endtask

  task automatic drain();
    idle(8);
    repeat (1500) @(posedge vclk);
  endtask

  task automatic do_clear();
    @(posedge vclk); clear <= 1'b1;
    @(posedge vclk); clear <= 1'b0;
    repeat (4) @(posedge vclk);
  endtask

  task automatic begin_test(input int lb);
    lbads = 4'(lb);
    ov_j = -1;
    do_clear();
  endtask

  // ---------------------------------------------------------------- stream monitors
  bit run_en = 0;
  int consec = 0, consec_max = 0;
  always @(posedge vclk) if (run_en) begin
    if (!m_tready) begin errors++; $display("ERROR: rec_verify s_tready dropped"); end
    if (m_tvalid) begin consec++; if (consec > consec_max) consec_max = consec; end
    else consec = 0;
  end

  // ---------------------------------------------------------------- the test
  initial begin
    longint t0, t1;
    $readmemh("../../vectors/vrf_blocks.hex", blk);
    if (blk[255] === 128'bx || blk[75*256-1] === 128'bx) begin
      $display("ERROR: vrf_blocks.hex not loaded"); $fatal(1, "no vectors");
    end
    repeat (20) @(posedge uclk);
    vrst = 0;
    repeat (20) @(posedge uclk);
    urst = 0;
    repeat (30) @(posedge uclk);
    enable = 1;
    run_en = 1;
    repeat (10) @(posedge uclk);

    // ---- 1. clean chain at line rate: 256-beat bursts, 50 % duty, lbads 9, uneven clock ratio
    begin_test(9);
    for (int p = 0; p < 64; p++) begin send_blk(p, p); idle(256); end
    drain();
    chk("clean64", 64, 0, 0, 0, 0, 0, 0);
    ck("clean64", "vb", longint'(res.vb), 64*4096);
    ck("clean64", "pad", longint'(res.pad), 5);
    ck("clean64", "mvp", longint'(res.mvp), 63);
    ck("clean64", "lsq", longint'(res.lsq), 66);
    ck("clean64", "lts", longint'(res.lts), 63*1000+7);
    ck("clean64", "lst", longint'(res.lst), 3);
    ck("clean64", "fbp_none", res.fbp === 40'hFF_FFFF_FFFF, 1);
    ck("clean64", "end_found", longint'(end_found), 0);
    ck("clean64", "ovf", longint'(res.ovf), 0);
    ck("clean64", "tap_ovf", longint'(tap_ovf), 0);
    ck("clean64", "LAST flag", longint'(mvp_flags[1]), 1);
    ck("clean64", "PAD flag", longint'(mvp_flags[2]), 1);

    // ---- 1b. same stream, other ui/user clock ratio (ui faster), continuous 2-block bursts
    vper = 7.11;
    begin_test(10);
    for (int p = 0; p < 16; p += 2) begin send_blk(p, p); send_blk(p+1, p+1); idle(600); end
    drain();
    chk("clean16_fast_ui", 16, 0, 0, 0, 0, 0, 0);
    ck("clean16_fast_ui", "ovf", longint'(res.ovf), 0);
    vper = 9.97;

    // ---- 2. lbads sweep (address -> pos mapping)
    for (int lb = 9; lb <= 12; lb++) begin
      begin_test(lb);
      for (int p = 28; p < 36; p++) begin send_blk(p, p); idle(8); end   // crosses the slot boundary at 32
      drain();
      chk($sformatf("lbads%0d", lb), 8, 0, 0, 0, 0, 0, 0);
      ck("lbads", "mvp", longint'(res.mvp), 35);
    end

    // ---- 3. two slots interleaved beat by beat
    begin_test(12);
    send_two(3, 3, 35, 35);
    send_two(4, 4, 36, 36);
    drain();
    chk("interleave", 4, 0, 0, 0, 0, 0, 0);

    // ---- 4. PAD / TRUNC / LAST
    begin_test(12);
    for (int p = 58; p < 64; p++) send_blk(p, p);
    drain();
    chk("pad_trunc", 6, 0, 0, 0, 0, 0, 0);
    ck("pad_trunc", "pad", longint'(res.pad), 5);
    ck("pad_trunc", "mvp", longint'(res.mvp), 63);
    ck("pad_trunc", "LAST", longint'(mvp_flags[1]), 1);

    // ---- 5. corruptions (one block each)
    //            name    idx ok bad crc pe ooo prt ve
    begin_test(12); send_blk(64, 64); drain(); chk("magic",       0, 1, 0, 0, 0, 0, 1);
    ck("magic", "fbp", longint'(res.fbp), 64); ck("magic", "end_found", longint'(end_found), 1);
    begin_test(12); send_blk(65, 65); drain(); chk("ver",         0, 1, 0, 0, 0, 0, 1);
    begin_test(12); send_blk(66, 66); drain(); chk("crc",         0, 1, 1, 0, 0, 0, 1);
    begin_test(12); send_blk(67, 67); drain(); chk("payload",     0, 1, 1, 1, 0, 0, 2);
    ck("payload", "fep", longint'(res.fep), 67*256 + 100);
    ck("payload", "feb", longint'(res.feb), 2);
    ck("payload", "exp_dw", longint'(res.exp_dw), longint'((67*256 + 100) ^ 32'hA5A5A5A5));
    ck("payload", "got_dw", longint'(res.got_dw), longint'(((67*256 + 100) ^ 32'hA5A5A5A5) ^ 32'h2000));
    begin_test(12); send_blk(68, 68); drain(); chk("seq_gap",     0, 1, 0, 0, 0, 0, 1);
    begin_test(12); send_blk(69, 69); drain(); chk("duplicate",   0, 1, 0, 252, 0, 0, 255);
    begin_test(12); send_blk(70, 70); drain(); chk("wrong_sid",   0, 1, 0, 0, 0, 0, 1);
    begin_test(12); send_blk(71, 71); drain(); chk("magic_end",   0, 1, 0, 0, 0, 0, 1);
    begin_test(12); send_blk(72, 72); drain(); chk("pos_lo",      0, 1, 0, 0, 0, 0, 1);
    begin_test(12); send_blk(73, 73); drain(); chk("seq_lo",      0, 1, 0, 0, 0, 0, 1);
    begin_test(12); send_blk(74, 74); drain(); chk("valid_gt252", 0, 1, 0, 0, 0, 0, 1);
    // good block 20 for a wrong expected SID
    begin_test(12); exp_sid = 32'h11111111; send_blk(20, 20); drain(); chk("exp_sid", 0, 1, 0, 0, 0, 0, 1);
    exp_sid = SID;
    // a block that is correct but stored at the wrong address (shifted by one block): pos mismatch
    begin_test(12); send_blk(21, 22); drain(); chk("wrong_pos", 0, 1, 0, 252, 0, 0, -1);

    // ---- 6. partial beats and byte enables
    begin_test(12); ov_j = 50; ov_be = 16'h7FFF;
    send_blk(30, 30); drain(); chk("prt_payload", 1, 0, 0, 0, 0, 1, 0);
    ck("prt_payload", "vb", longint'(res.vb), 4095);
    begin_test(12); ov_j = 1; ov_be = 16'h7FFF;
    send_blk(30, 30); drain(); chk("prt_hdr", 0, 1, 0, 0, 0, 1, 1);
    begin_test(12); ov_j = 60; ov_be = 16'hFFFE; ov_xor = 128'd1;      // corrupted byte masked out
    send_blk(31, 31); drain(); chk("masked_err", 1, 0, 0, 0, 0, 1, 0);
    begin_test(12); ov_j = 60; ov_be = 16'hFFFF; ov_xor = 128'd1;      // same corruption, enabled
    send_blk(31, 31); drain(); chk("enabled_err", 0, 1, 1, 1, 0, 0, 2);
    ck("enabled_err", "feb", longint'(res.feb), 0);
    ck("enabled_err", "fep", longint'(res.fep), 31*256 + 60);
    ov_j = -1; ov_xor = '0; ov_be = 16'hFFFF;

    // ---- 7. out of order: beats 100..149 missing
    begin_test(12);
    send(20, 20, 0, 99); send(20, 20, 150, 255); drain();
    chk("ooo", 0, 1, 0, 0, 1, 0, 1);

    // ---- 8. truncated stream: valid 0..9, block 10 cut after beat 99 -> last valid block = 9
    begin_test(12);
    for (int p = 0; p < 10; p++) send_blk(p, p);
    send(10, 10, 0, 99);
    drain();
    chk("trunc_only", 10, 0, 0, 0, 0, 0, 0);
    ck("trunc_only", "mvp", longint'(res.mvp), 9);
    ck("trunc_only", "end_found", longint'(end_found), 0);
    // ... followed by stale (zero) blocks as READ END would see them
    send(-1, 11, 0, 255); idle(BLK_GAP); send(-1, 12, 0, 255); idle(BLK_GAP); drain();
    chk("trunc_stale", 10, 3, 2, -1, 1, 0, -1);   // zero blocks also fail the CRC
    ck("trunc_stale", "mvp", longint'(res.mvp), 9);
    ck("trunc_stale", "fbp", longint'(res.fbp), 10);
    ck("trunc_stale", "end_found", longint'(end_found), 1);
    ck("trunc_stale", "mvp<fbp", longint'(res.mvp < res.fbp), 1);

    // ---- 8b. hole: bad block in the middle, valid blocks after it -> mvp > fbp
    begin_test(12);
    for (int p = 0; p < 5; p++) send_blk(p, p);
    send(-1, 5, 0, 255); idle(BLK_GAP);
    for (int p = 6; p < 9; p++) send_blk(p, p);
    drain();
    chk("hole", 8, 1, 1, -1, 0, 0, -1);
    ck("hole", "fbp", longint'(res.fbp), 5);
    ck("hole", "mvp", longint'(res.mvp), 8);
    ck("hole", "mvp>fbp", longint'(res.mvp > res.fbp), 1);

    // ---- 8c. missing LAST: chain ends without the LAST flag, blocks are all valid
    begin_test(12);
    for (int p = 40; p < 58; p++) send_blk(p, p);
    drain();
    chk("no_last", 18, 0, 0, 0, 0, 0, 0);
    ck("no_last", "mvp", longint'(res.mvp), 57);
    ck("no_last", "LAST", longint'(mvp_flags[1]), 0);

    // ---- 9. FIFO overflow: continuous 125 MHz stream > 100 MHz consumer, then recovery
    begin_test(12);
    consec_max = 0;
    t0 = $time;
    for (int p = 0; p < 24; p++) begin set_slot(p); for (int j = 0; j < 256; j++) beat(p, p, j); end   // no gaps
    t1 = $time;
    idle(4);
    drain();
    ck("overflow", "res.ovf", longint'(res.ovf), 1);
    ck("overflow", "tap_ovf", longint'(tap_ovf), 1);
    // read speed: consumer never stalls while the FIFO has data (1 beat per ui_clk)
    $display("read speed: longest run of back-to-back beats = %0d ui_clk (1 beat/clk, %0.2f GB/s at %0.1f MHz)",
             consec_max, 16.0 * 1000.0 / vper / 1000.0, 1000.0 / vper);
    if (consec_max < 400) fail($sformatf("overflow: expected long continuous drain, got %0d", consec_max));
    enable = 0; repeat (20) @(posedge vclk);
    do_clear();
    ck("ovf_clear", "res.ovf", longint'(res.ovf), 0);
    ck("ovf_clear", "tap_ovf", longint'(tap_ovf), 0);
    enable = 1; repeat (10) @(posedge uclk);
    send_blk(1, 1); send_blk(2, 2); drain();
    chk("after_ovf", 2, 0, 0, 0, 0, 0, 0);
    ck("after_ovf", "res.ovf", longint'(res.ovf), 0);

    // ---- 9b. WP3 framer output vectors (blocks_stop / blocks_pf_drop / blocks_len), sid 692D3A00
    exp_sid = 32'h692D3A00;
    for (int f = 0; f < 3; f++) begin
      if (f == 0) $readmemh("../../vectors/blocks_stop.hex", tmp);
      if (f == 1) $readmemh("../../vectors/blocks_pf_drop.hex", tmp);
      if (f == 2) $readmemh("../../vectors/blocks_len.hex", tmp);
      for (int i = 0; i < 32*256; i++) blk[i] = tmp[i];      // reuse blocks 0..31 of the reference array
      for (int lb = 9; lb <= 12; lb += 3) begin
        begin_test(lb);
        for (int p = 0; p < 32; p++) send_blk(p, p);
        drain();
        chk($sformatf("wp3_%0d_lb%0d", f, lb), 32, 0, 0, 0, 0, 0, 0);
        ck("wp3", "vb", longint'(res.vb), 32*4096);
        ck("wp3", "pad", longint'(res.pad), (f == 0) ? 25 : (f == 1) ? 24 : 23);
        ck("wp3", "mvp", longint'(res.mvp), 31);
        ck("wp3", "lsq", longint'(res.lsq), (f == 1) ? 41 : 31);
        ck("wp3", "lts", longint'(res.lts), (f == 0) ? 9472 : (f == 1) ? 14892 : 9984);
        ck("wp3", "lst", longint'(res.lst), (f == 1) ? 10 : 0);
        ck("wp3", "fbp_none", res.fbp === 40'hFF_FFFF_FFFF, 1);
        ck("wp3", "end_found", longint'(end_found), 0);
        ck("wp3", "ovf", longint'(res.ovf), 0);
        ck("wp3", "flags", longint'(mvp_flags), (f == 1) ? 8'h16 : 8'h46);
      end
    end
    exp_sid = SID;

    // ---- 10. negative control of the checker: a wrong expectation must be detected
    neg_mode = 1;
    chk("neg_control", 99, 0, 0, 0, 0, 0, 0);
    neg_mode = 0;
    if (neg_det != 1) begin errors++; $display("ERROR: negative control not detected (%0d)", neg_det); end

    if (errors == 0) $display("TEST PASSED");
    else $fatal(1, "tb_rec_verify: %0d error(s)", errors);
    $finish;
  end

  initial begin
    #40_000_000;
    $display("ERROR: timeout");
    $fatal(1, "timeout");
  end

endmodule
