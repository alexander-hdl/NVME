// =============================================================================
// tb_data_ring — testbench for data_ring (ARCHITECTURE.md §5.9)
//
// Small ring (4 slots x 16 KiB = 1024 beats per slot) and two UNRELATED clocks
// (data_clk 7 ns, user clk 8 ns).
//
// Phase 1  no slot is ever freed: the 4 slots fill (slot_full = 1111), the async
//          FIFO fills, the source is stalled (s_data_tready low, input_stalled,
//          stall_cycles counting), no beat is lost or written into a full slot.
//          Read-back of every beat (continuous and random-gap reads, 2-cycle
//          latency) equals the streamed pattern.
// Phase 2  slot_free protocol: freeing slot 1 first does not let the writer
//          pass slot 0 (still full); freeing slot 0 lets it refill slots 0 and 1
//          with the next 2048 beats, slot_full rises again.
// Phase 3  steady-state consumer: for many laps wait for slot_full[k], read the
//          whole slot back and compare with the stream, free it after a random
//          delay; source valid probability < 1 (so FIFO empties too).
// Finish   every accepted beat is accounted for (beats_in) and no data is lost.
// =============================================================================
module tb_data_ring;
  import nvme_host_pkg::*;

  localparam int unsigned N_SLOTS    = 4;
  localparam int unsigned SLOT_BYTES = 16384;
  localparam int unsigned BEATS      = SLOT_BYTES / 16;         // 1024
  localparam int unsigned FIFO_DEPTH = 512;

  logic data_clk = 1'b0, clk = 1'b0;
  always #3.5 data_clk = ~data_clk;
  always #4   clk      = ~clk;

  logic data_rst = 1'b1, rst = 1'b1;

  logic [127:0] s_data_tdata;
  logic         s_data_tvalid = 1'b0, s_data_tready;
  logic [N_SLOTS-1:0] slot_full, slot_free = '0;
  logic [$clog2(N_SLOTS):0] slots_used;
  logic         input_stalled;
  logic [63:0]  beats_in;
  logic [31:0]  stall_cycles;
  logic         rd_en = 1'b0;
  logic [31:0]  rd_addr = 32'd0;
  logic [127:0] rd_data;

  data_ring #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES), .FIFO_DEPTH(FIFO_DEPTH)) dut (
    .data_clk(data_clk), .data_rst(data_rst),
    .s_data_tdata(s_data_tdata), .s_data_tvalid(s_data_tvalid), .s_data_tready(s_data_tready),
    .clk(clk), .rst(rst),
    .slot_full(slot_full), .slot_free(slot_free), .slots_used(slots_used),
    .input_stalled(input_stalled), .beats_in(beats_in), .stall_cycles(stall_cycles),
    .rd_en(rd_en), .rd_addr(rd_addr), .rd_data(rd_data));

  int errors = 0;
  task automatic fail(input string msg);
    errors++;
    $display("[%0t] ERROR: %s", $time, msg);
    if (errors > 20) $fatal(1, "too many errors");
  endtask

  // ---------------- source ----------------
  function automatic logic [127:0] pat(input longint unsigned n);
    logic [31:0] a;
    a = n[31:0];
    return {a ^ 32'hA5A5_0000, ~a, a * 32'd7 + 32'd1, 32'hDEAD_0000 | a[15:0]};
  endfunction

  longint unsigned prod_idx = 0;       // index of the beat currently offered
  longint unsigned accepted = 0;       // beats accepted by the DUT
  int              valid_prob = 100;   // % chance to present a beat in a free slot
  bit              prod_run = 0;

  assign s_data_tdata = pat(prod_idx);

  always @(posedge data_clk) begin
    if (data_rst) begin
      s_data_tvalid <= 1'b0; prod_idx <= 0; accepted <= 0;
    end else begin
      if (s_data_tvalid && s_data_tready) begin
        prod_idx <= prod_idx + 1;
        accepted <= accepted + 1;
        s_data_tvalid <= prod_run && ($urandom_range(0, 99) < valid_prob);
      end else if (!s_data_tvalid) begin
        s_data_tvalid <= prod_run && ($urandom_range(0, 99) < valid_prob);
      end
    end
  end

  // ---------------- read helper ----------------
  // beat `beat` of slot `slot` in lap `lap` carries stream index (lap*N_SLOTS+slot)*BEATS+beat
  function automatic longint unsigned stream_idx(input int slot, input int beat, input longint unsigned lap);
    return (lap * N_SLOTS + slot) * BEATS + beat;
  endfunction

  longint unsigned reads_checked = 0;

  task automatic read_range(input int slot, input int first, input int count,
                            input longint unsigned lap, input int idle_pct = 0);
    logic [127:0] e_d [4];
    bit           e_v [4];
    int           issued, n, drain, chk, idx;
    bit           do_rd;
    issued = 0; n = 0; drain = 0;
    for (int i = 0; i < 4; i++) e_v[i] = 0;
    while (issued < count || drain < 3) begin
      @(posedge clk);
      // rd_en set after edge n-3, sampled at n-2, data after n-1: checked at edge n
      chk = (n + 1) % 4;
      idx = n % 4;
      if (e_v[chk]) begin
        reads_checked++;
        if (rd_data !== e_d[chk])
          fail($sformatf("slot %0d lap %0d: rd_data %h exp %h", slot, lap, rd_data, e_d[chk]));
      end
      do_rd = (issued < count) && ($urandom_range(0, 99) >= idle_pct);
      rd_en   <= do_rd;
      rd_addr <= slot * SLOT_BYTES + (first + issued) * 16;
      e_v[idx] = do_rd;
      e_d[idx] = pat(stream_idx(slot, first + issued, lap));
      if (do_rd) issued++;
      else if (issued >= count) drain++;
      n++;
    end
    @(posedge clk); rd_en <= 1'b0;
  endtask

  task automatic free_slot(input int slot);
    @(posedge clk); slot_free[slot] <= 1'b1;
    @(posedge clk); slot_free[slot] <= 1'b0;
  endtask

  longint unsigned lap_of [N_SLOTS];
  longint unsigned acc_snap, bi_snap;
  int unsigned     sc_snap;

  initial begin
    for (int k = 0; k < N_SLOTS; k++) lap_of[k] = 0;
    repeat (5) @(posedge clk);
    rst <= 1'b0; data_rst <= 1'b0;
    repeat (5) @(posedge clk);

    // ================= Phase 1: fill, no frees =================
    prod_run = 1; valid_prob = 100;
    // start the source
    begin
      int t;
      t = 0;
      while (slot_full !== 4'b1111 && t < 20000) begin @(posedge clk); t++; end
      if (t >= 20000) fail("slots never all full");
      repeat (2000) @(posedge clk);                 // let the async FIFO fill up and the source stall
      if (slot_full !== 4'b1111) fail($sformatf("slot_full %b", slot_full));
      if (slots_used !== 3'd4)   fail($sformatf("slots_used %0d", slots_used));
      if (input_stalled !== 1'b1) fail("input_stalled not asserted with a stalled source");
      if (s_data_tready !== 1'b0) fail("s_data_tready still high with full ring + full FIFO");
      if (accepted < 4096 + FIFO_DEPTH || accepted > 4096 + FIFO_DEPTH + 4)
        fail($sformatf("accepted %0d beats before stalling (expected 4096 + FIFO depth %0d)", accepted, FIFO_DEPTH));
      if (beats_in !== 64'd4096) fail($sformatf("beats_in %0d expected 4096", beats_in));
      $display("Phase 1: ring full after %0d accepted beats, beats_in=%0d, stall_cycles=%0d", accepted, beats_in, stall_cycles);
      acc_snap = accepted; bi_snap = beats_in; sc_snap = stall_cycles;
      repeat (500) @(posedge clk);
      if (accepted != acc_snap || beats_in != bi_snap) fail("data moved although every slot is full");
      if (stall_cycles <= sc_snap) fail("stall_cycles not counting while stalled");
      // read back everything: continuous, then with random idle cycles
      for (int k = 0; k < N_SLOTS; k++) read_range(k, 0, BEATS, 0);
      for (int k = 0; k < N_SLOTS; k++) read_range(k, $urandom_range(0, 500), 400, 0, 50);
      $display("Phase 1: read-back of 4 slots OK so far (errors=%0d, reads=%0d)", errors, reads_checked);
    end

    // ================= Phase 2: free protocol =================
    free_slot(1);
    repeat (5) @(posedge clk);
    if (slot_full !== 4'b1101) fail($sformatf("after free(1): slot_full %b", slot_full));
    if (slots_used !== 3'd3)   fail($sformatf("after free(1): slots_used %0d", slots_used));
    acc_snap = accepted; bi_snap = beats_in;
    repeat (1000) @(posedge clk);
    if (accepted != acc_snap || beats_in != bi_snap) fail("writer passed slot 0 although it is still full");
    lap_of[1] = 1;
    free_slot(0);
    lap_of[0] = 1;
    begin
      int t;
      t = 0;
      while ((slot_full[0] !== 1'b1 || slot_full[1] !== 1'b1) && t < 20000) begin @(posedge clk); t++; end
      if (t >= 20000) fail("slots 0/1 not refilled after free");
      repeat (1500) @(posedge clk);
      if (slot_full !== 4'b1111) fail($sformatf("after refill: slot_full %b", slot_full));
      if (beats_in !== 64'd6144) fail($sformatf("beats_in %0d expected 6144", beats_in));
      read_range(0, 0, BEATS, 1);
      read_range(1, 0, BEATS, 1);
      $display("Phase 2: refill OK (errors=%0d, reads=%0d)", errors, reads_checked);
    end

    // ================= Phase 3: steady-state consumer =================
    valid_prob = 60;
    for (int it = 0; it < 28; it++) begin
      int k, t;
      k = (2 + it) % N_SLOTS;
      t = 0;
      while (slot_full[k] !== 1'b1 && t < 40000) begin @(posedge clk); t++; end
      if (t >= 40000) begin fail($sformatf("slot %0d never full (lap %0d)", k, lap_of[k])); break; end
      read_range(k, 0, BEATS, lap_of[k], (it % 3 == 0) ? 30 : 0);
      repeat ($urandom_range(0, 300)) @(posedge clk);
      free_slot(k);
      lap_of[k]++;
    end
    $display("Phase 3: consumer loop done (errors=%0d, reads=%0d, accepted=%0d, beats_in=%0d, stall_cycles=%0d)",
             errors, reads_checked, accepted, beats_in, stall_cycles);

    // ================= drain =================
    prod_run = 0;
    repeat (50) @(posedge data_clk);
    for (int k = 0; k < N_SLOTS; k++) begin
      if (slot_full[k]) begin free_slot(k); lap_of[k]++; end
    end
    repeat (6000) @(posedge clk);
    for (int k = 0; k < N_SLOTS; k++) if (slot_full[k]) free_slot(k);
    repeat (6000) @(posedge clk);
    if (beats_in !== accepted) fail($sformatf("beats_in %0d != accepted %0d", beats_in, accepted));
    $display("SUMMARY: accepted=%0d beats_in=%0d reads_checked=%0d stall_cycles=%0d errors=%0d",
             accepted, beats_in, reads_checked, stall_cycles, errors);
    if (errors != 0) $fatal(1, "TEST FAILED");
    $display("TEST PASSED");
    $finish;
  end

  initial begin
    #100ms;
    $fatal(1, "TIMEOUT");
  end
endmodule
