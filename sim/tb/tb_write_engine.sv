// =============================================================================
// tb_write_engine.sv - self-checking testbench for rtl/nvme/nvme_write_engine.sv
//
// A reusable harness (we_harness) contains a DUT, a behavioural data ring
// (slot_full / slot_free), behavioural iosq_ram / iocq_ram (2-cycle read),
// the requester command-port model (port 1: MWr doorbells only) and an SSD
// model that, on every SQ tail doorbell, fetches and checks the SQEs and then
// posts CQEs out of order with random latencies (CQE written as a 16-byte
// beat, lower half first, phase/CID half last).
//
// Four harness instances run in parallel:
//   H0 (SCEN 0) 4 slots x 128 KiB, 600 commands (CQ/SQ wrap > 9 times), random out-of-order
//               completion, injected error CQEs, enable gating, flush while busy and at
//               the end, stats (bytes/cmds/errors/last_status/max latency).
//   H1 (SCEN 1) 70 slots x 4 KiB (PRP2 unused), long completion latency so the SQ fills:
//               at most IO_QSIZE-1 commands outstanding, sq_full_stall counted.
//   H3 (SCEN 3) 1 slot x 4 KiB: slot release race (slot_full still high after the CQE).
//   H2 (SCEN 2) 4 slots x 8 KiB (PRP2 = 2nd data page), 4 KiB LBAs, 64-bit SLBA, nsze
//               allows only 3 commands: LBA range guard.
//   H4 (SCEN 4) I/O timeout (M4): the SSD model never completes the 6th Write.  After IO_TMO_MS
//               "ms" (tick_ms = every 100 clk) io_timeout is set sticky, no further command is
//               submitted, the slot is NOT freed, a Flush request is answered at once with
//               flush_done + flush_skipped and no Flush SQE; `restart` then returns the engine
//               to its initial state (flags cleared, next_lba reloaded, queue pointers 0).
// First-error reporting (M6): every harness compares io_error / first_error_lba with the first
// failed Write seen by the SSD model (H0 injects errors).
// =============================================================================
`timescale 1ns/1ps

module we_harness
  import nvme_host_pkg::*;
#(
  parameter int SCEN       = 0,
  parameter int N_SLOTS    = 4,
  parameter int SLOT_BYTES = 131072,
  parameter int LBADS      = 9,
  parameter int N_CMDS     = 100,
  parameter int LAT_MIN    = 20,
  parameter int LAT_MAX    = 150,
  parameter int ERR_EVERY  = 0,                          // every n-th command completes with an error (0 = never)
  parameter logic [63:0] START_LBA = 64'h0,
  parameter logic [63:0] NSZE      = 64'h0000_0000_7470_0000,
  parameter int IO_TMO_MS  = 0,                          // engine I/O timeout in tick_ms units (0 = off)
  parameter int DROP_NTH   = -1                          // the SSD model never completes data command #n
) (
  input  logic clk,
  output logic done,
  output int   errors
);
  localparam int LBA_PER_SLOT = SLOT_BYTES >> LBADS;
  localparam int SW           = (N_SLOTS > 1) ? $clog2(N_SLOTS) : 1;

  logic rst = 1'b1;

  // ---- DUT connections ----
  logic        enable = 1'b0;
  logic [63:0] start_lba = START_LBA;
  logic [3:0]  lbads_s = 4'(LBADS);
  logic [63:0] nsze_s = NSZE;
  logic [3:0]  dstrd_s = 4'd0;
  logic [N_SLOTS-1:0] slot_full = '0;
  logic [N_SLOTS-1:0] slot_free;
  logic         iosq_wr_en;
  logic [7:0]   iosq_wr_addr;
  logic [127:0] iosq_wr_data;
  logic         iocq_rd_en;
  logic [5:0]   iocq_rd_addr;
  logic [127:0] iocq_rd_data;
  logic         req_valid, req_ready;
  logic [1:0]   req_op;
  logic [31:0]  req_addr, req_wdata;
  logic [3:0]   req_be;
  logic         rsp_valid = 1'b0;
  logic [2:0]   rsp_status = 3'd0;
  logic         flush_req = 1'b0, flush_done;
  logic [63:0]  next_lba, bytes_done, cmds_done;
  logic [31:0]  cmd_errors, max_latency_cycles, sq_full_stall_cycles;
  logic [15:0]  last_status;
  logic         sq_full_stall, slots_idle, lba_range_err, doorbell_err;
  logic         io_timeout, flush_skipped, io_error, restart = 1'b0;
  logic [63:0]  first_error_lba;
  logic         tick_ms = 1'b0;
  int           tick_div = 0;
  always_ff @(posedge clk) begin                      // 1 "ms" = 100 clk (only when the timeout is used)
    tick_div <= (tick_div == 99) ? 0 : tick_div + 1;
    tick_ms  <= (IO_TMO_MS != 0) && (tick_div == 99);
  end
  logic [6:0]   sq_tail, cq_head;

  nvme_write_engine #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES), .IO_TIMEOUT_MS(IO_TMO_MS)) dut (
    .clk(clk), .rst(rst), .tick_ms(tick_ms), .restart(restart),
    .enable(enable), .start_lba(start_lba), .lbads(lbads_s), .nsze(nsze_s), .dstrd(dstrd_s),
    .slot_full(slot_full), .slot_free(slot_free),
    .iosq_wr_en(iosq_wr_en), .iosq_wr_addr(iosq_wr_addr), .iosq_wr_data(iosq_wr_data),
    .iocq_rd_en(iocq_rd_en), .iocq_rd_addr(iocq_rd_addr), .iocq_rd_data(iocq_rd_data),
    .req_valid(req_valid), .req_ready(req_ready), .req_op(req_op), .req_addr(req_addr),
    .req_wdata(req_wdata), .req_be(req_be), .rsp_valid(rsp_valid), .rsp_status(rsp_status),
    .flush_req(flush_req), .flush_done(flush_done),
    .next_lba(next_lba), .bytes_done(bytes_done), .cmds_done(cmds_done), .cmd_errors(cmd_errors),
    .last_status(last_status), .max_latency_cycles(max_latency_cycles),
    .sq_full_stall(sq_full_stall), .sq_full_stall_cycles(sq_full_stall_cycles),
    .sq_tail(sq_tail), .cq_head(cq_head), .slots_idle(slots_idle),
    .lba_range_err(lba_range_err), .doorbell_err(doorbell_err),
    .io_timeout(io_timeout), .flush_skipped(flush_skipped), .io_error(io_error), .first_error_lba(first_error_lba));

  // ---- error bookkeeping ----
  initial errors = 0;
  task automatic fail_msg(input string m);
    errors++;
    $display("[%0t] H%0d ERROR: %s", $time, SCEN, m);
  endtask
  task automatic check(input bit c, input string m);
    if (!c) fail_msg(m);
  endtask

  // ---- RAMs ----
  logic [127:0] iosq_mem [0:255];
  logic [127:0] iocq_mem [0:63];
  logic [127:0] iocq_q1;
  initial begin
    for (int i = 0; i < 256; i++) iosq_mem[i] = '0;
    for (int i = 0; i < 64;  i++) iocq_mem[i] = '0;
  end
  always @(posedge clk) if (iosq_wr_en) iosq_mem[iosq_wr_addr] <= iosq_wr_data;
  always_ff @(posedge clk) begin
    iocq_q1      <= iocq_mem[iocq_rd_addr];
    iocq_rd_data <= iocq_q1;
  end

  // ---- cycle counter ----
  int cyc = 0;
  always_ff @(posedge clk) cyc <= cyc + 1;

  // ---- data ring model: slots get filled in order; slot_free clears slot_full 2 cycles later ----
  int          fills_done = 0;
  int          fill_ptr = 0;
  int          fill_wait = 0;
  int          fill_limit;
  logic [N_SLOTS-1:0] free_d1 = '0, free_d2 = '0;
  int          free_pulses = 0;
  initial fill_limit = (SCEN == 2) ? 6 : N_CMDS;

  always_ff @(posedge clk) begin
    free_d1 <= slot_free;
    free_d2 <= free_d1;
    for (int k = 0; k < N_SLOTS; k++) begin
      if (free_d2[k]) slot_full[k] <= 1'b0;
      if (slot_free[k]) free_pulses <= free_pulses + 1;
    end
    if (!rst && fills_done < fill_limit) begin
      if (fill_wait > 0) fill_wait <= fill_wait - 1;
      else if (!slot_full[fill_ptr] && !free_d1[fill_ptr] && !free_d2[fill_ptr] && !slot_free[fill_ptr]) begin
        slot_full[fill_ptr] <= 1'b1;
        fills_done <= fills_done + 1;
        fill_ptr   <= (fill_ptr + 1) % N_SLOTS;
        fill_wait  <= (SCEN == 1) ? 0 : ($urandom % 6);
      end
    end
  end

  // ---- command-port model + SSD model ----
  logic        gate = 1'b1;
  logic        cp_busy = 1'b0;
  int          cp_delay = 0;
  assign req_ready = gate && !cp_busy;

  // SSD side state (blocking variables, used only in the model block)
  int  sq_head_m = 0;                // next SQE the SSD will fetch
  int  cq_tail_m = 0;                // next CQ slot the SSD will write
  logic cq_ph_m  = 1'b1;
  int  cq_consumed_m = 0;            // head value of the last CQ doorbell
  int  cq_posted_total = 0;
  int  n_sqe = 0;                    // data SQEs fetched
  int  n_flush_sqe = 0;
  int  n_ok = 0, n_err = 0;
  logic [14:0] last_err_status = 15'h0;
  int  last_sq_db = 0;
  int  sq_db_count = 0, cq_db_count = 0;
  int  max_inflight = 0;
  int  tb_max_lat_lo = 0;
  int  db_cycle [0:N_SLOTS-1];

  // in-flight table
  int  ifl_n = 0;
  int  ifl_slot [0:N_SLOTS+1];
  logic [63:0] ifl_lba [0:N_SLOTS+1];
  logic [63:0] first_err_slba_m = 64'h0;
  bit          first_err_seen_m = 1'b0;
  int  ifl_due  [0:N_SLOTS+1];
  bit  ifl_flush[0:N_SLOTS+1];
  bit  slot_inflight [0:N_SLOTS-1];
  int  flush_idx_active = 0;

  // CQE writer
  int  cqw_state = 0;
  int  cqw_wait = 0;
  int  cqw_slot_idx;
  logic [127:0] cqw_beat;
  int  cqw_ifl_pick;

  initial for (int i = 0; i < N_SLOTS; i++) begin slot_inflight[i] = 0; db_cycle[i] = 0; end

  // helper: expected data SQE
  function automatic logic [511:0] exp_data_sqe(input int j);
    logic [511:0] s;
    int k;
    logic [63:0] lba;
    logic [31:0] data_addr;
    k = j % N_SLOTS;
    lba = START_LBA + 64'(j) * 64'(LBA_PER_SLOT);
    data_addr = DATA_BASE + 32'(k) * 32'(SLOT_BYTES);
    s = '0;
    s[31:0]     = {16'(k), 8'h00, 8'h01};
    s[63:32]    = 32'h1;
    s[32*6+:32] = data_addr;
    if (SLOT_BYTES > 8192)       s[32*8+:32] = PRP_BASE + 32'(k) * 32'h1000;
    else if (SLOT_BYTES == 8192) s[32*8+:32] = data_addr + 32'h1000;
    s[32*10+:32] = lba[31:0];
    s[32*11+:32] = lba[63:32];
    s[32*12+:32] = 32'(LBA_PER_SLOT - 1);
    s[32*13+:32] = 32'h40;
    return s;
  endfunction

  logic        db_is_cq;
  int          db_qid;
  always @(posedge clk) begin : ssd_model
    int  off, q, v, nfetch, pick, cand, kk, rr, ll;
    logic [14:0] st;
    logic [15:0] cid_v;
    logic [511:0] got;
    logic [511:0] expd;
    bit  have;

    rsp_valid <= 1'b0;
    gate      <= ($urandom % 3) != 0;

    if (rst) begin
      cp_busy <= 1'b0;
    end else begin
      // ----------------------------------------------------------------
      // command port: accept one MWr, answer a few cycles later
      // ----------------------------------------------------------------
      if (cp_busy) begin
        if (cp_delay > 0) cp_delay = cp_delay - 1;
        else begin
          rsp_valid <= 1'b1;
          rsp_status <= 3'd0;
          cp_busy <= 1'b0;
        end
      end else if (req_valid && req_ready) begin
        cp_busy  <= 1'b1;
        cp_delay = 1 + ($urandom % 3);
        check(req_op == 2'd3, "command port op must be MWR32");
        check(req_be == 4'hF, "command port BE must be F");
        check(req_wdata[31:16] == 16'h0, "doorbell upper bits must be 0");
        off = req_addr - BAR0_ADDR;
        check(req_addr[31:16] == BAR0_ADDR[31:16], "doorbell outside BAR0");
        q = (off - 'h1000) / 4;
        check(((off - 'h1000) % 4) == 0 && off >= 'h1000, "bad doorbell offset");
        check(q / 2 == 1, "doorbell for a queue other than QID 1");
        if (q % 2 == 0) begin
          // ----------------- SQ1 tail doorbell: fetch + check new SQEs -----------------
          v = req_wdata[15:0];
          sq_db_count++;
          check(v < IO_QSIZE, "SQ tail doorbell out of range");
          check(v != sq_head_m, "SQ doorbell without new entries");
          nfetch = (v - sq_head_m + IO_QSIZE) % IO_QSIZE;
          for (int e = 0; e < nfetch; e++) begin
            for (int b = 0; b < 4; b++) got[128*b +: 128] = iosq_mem[sq_head_m*4 + b];
            if (got[7:0] == 8'h00) begin
              // ---- Flush ----
              check(got[31:16] == 16'h00FF, "flush CID");
              check(got[63:32] == 32'h1, "flush NSID");
              check(got[511:64] == 448'h0 && got[31:8] == 24'h00FF00, "flush SQE other fields");
              check(ifl_n == 0, "flush submitted while data commands outstanding");
              check(flush_req, "flush submitted without flush_req");
              check(n_flush_sqe == 0 || !flush_idx_active, "second flush while one is active");
              ifl_slot[ifl_n] = 0; ifl_due[ifl_n] = cyc + LAT_MIN + ($urandom % (LAT_MAX - LAT_MIN + 1));
              ifl_flush[ifl_n] = 1; ifl_n++;
              n_flush_sqe++; flush_idx_active = 1;
            end else begin
              expd = exp_data_sqe(n_sqe);
              check(got == expd, $sformatf("SQE %0d mismatch\n got=%h\n exp=%h", n_sqe, got, expd));
              kk = got[31:16];
              if (kk < N_SLOTS) begin
                check(!slot_inflight[kk], $sformatf("slot %0d resubmitted before its CQE", kk));
                slot_inflight[kk] = 1;
                db_cycle[kk] = cyc;
                check(slot_full[kk], $sformatf("slot %0d submitted while not full", kk));
                if (DROP_NTH >= 0 && n_sqe == DROP_NTH) begin
                  // the SSD "loses" this command: no CQE, ever
                end else begin
                  ifl_slot[ifl_n] = kk;
                  ifl_lba[ifl_n]  = {got[32*11 +: 32], got[32*10 +: 32]};
                  ifl_due[ifl_n]  = cyc + LAT_MIN + ($urandom % (LAT_MAX - LAT_MIN + 1));
                  ifl_flush[ifl_n] = 0; ifl_n++;
                end
              end
              n_sqe++;
              if (ifl_n > max_inflight) max_inflight = ifl_n;
              check(ifl_n <= IO_QSIZE - 1, "more than IO_QSIZE-1 commands outstanding");
            end
            sq_head_m = (sq_head_m + 1) % IO_QSIZE;
          end
        end else begin
          // ----------------- CQ1 head doorbell -----------------
          v = req_wdata[15:0];
          cq_db_count++;
          check(v < IO_QSIZE, "CQ head doorbell out of range");
          check(v != cq_consumed_m, "CQ doorbell without progress");
          check(((v - cq_consumed_m + IO_QSIZE) % IO_QSIZE) <= ((cq_tail_m - cq_consumed_m + IO_QSIZE) % IO_QSIZE),
                $sformatf("CQ head doorbell %0d beyond posted entries (tail %0d)", v, cq_tail_m));
          cq_consumed_m = v;
        end
      end

      // ----------------------------------------------------------------
      // CQE writer: pick a due command (random among due ones), 2-step write
      // ----------------------------------------------------------------
      case (cqw_state)
        0: begin
          cand = 0; pick = -1;
          for (int i = 0; i < ifl_n; i++) if (ifl_due[i] <= cyc) cand++;
          if (cand > 0 && ((cq_tail_m + 1) % IO_QSIZE) != cq_consumed_m) begin
            rr = $urandom % cand;
            for (int i = 0; i < ifl_n; i++) if (ifl_due[i] <= cyc) begin
              if (rr == 0 && pick < 0) pick = i;
              rr--;
            end
            cqw_ifl_pick = pick;
            // build the beat: DW0 = result, DW1 = 0, DW2 = {SQID=1, SQHD}, DW3 = {status, P, CID}
            st = 15'h0;
            cid_v = ifl_flush[pick] ? 16'h00FF : 16'(ifl_slot[pick]);
            // error injection: every ERR_EVERY-th completed data command
            if (!ifl_flush[pick] && ERR_EVERY != 0 && ((n_ok + n_err) % ERR_EVERY) == ERR_EVERY - 1) st = 15'h0081;
            cqw_beat = {st, cq_ph_m, cid_v, 16'h0001, 16'(sq_head_m), 32'h0, 32'h0};
            iocq_mem[cq_tail_m][63:0] <= 64'h0;               // lower half (DW0, DW1) first
            cqw_wait  = 2;
            cqw_state = 1;
          end
        end
        1: begin
          if (cqw_wait > 0) cqw_wait = cqw_wait - 1;
          else begin
            iocq_mem[cq_tail_m][127:64] <= cqw_beat[127:64];  // DW2 + DW3 (P) last
            cqw_state = 2;
          end
        end
        2: begin
          // bookkeeping once the entry is visible
          if (ifl_flush[cqw_ifl_pick]) begin
            flush_idx_active = 0;
          end else begin
            kk = ifl_slot[cqw_ifl_pick];
            ll = cyc - db_cycle[kk];
            slot_inflight[kk] = 0;
            if (ll > tb_max_lat_lo) tb_max_lat_lo = ll;
            if (cqw_beat[127:113] != 15'h0) begin
              n_err++; last_err_status = cqw_beat[127:113];
              if (!first_err_seen_m) begin first_err_seen_m = 1'b1; first_err_slba_m = ifl_lba[cqw_ifl_pick]; end
            end
            else n_ok++;
          end
          for (int i = cqw_ifl_pick; i < ifl_n - 1; i++) begin
            ifl_slot[i] = ifl_slot[i+1]; ifl_due[i] = ifl_due[i+1]; ifl_flush[i] = ifl_flush[i+1];
            ifl_lba[i] = ifl_lba[i+1];
          end
          ifl_n--;
          if (cq_tail_m == IO_QSIZE - 1) begin cq_tail_m = 0; cq_ph_m = ~cq_ph_m; end
          else cq_tail_m = cq_tail_m + 1;
          cq_posted_total++;
          cqw_state = 0;
        end
        default: cqw_state = 0;
      endcase
    end
  end

  // ---- invariant monitors ----
  always_ff @(posedge clk) begin
    if (!rst) begin
      for (int k = 0; k < N_SLOTS; k++)
        if (slot_free[k] && slot_inflight[k]) fail_msg("slot_free pulse before the CQE was posted");
      check(!doorbell_err, "doorbell_err raised");
    end
  end

  // ---- scenario control ----
  task automatic wait_cycles(input int n);
    repeat (n) @(posedge clk);
  endtask

  initial begin
    int total_expected;
    int t;
    int snap;
    done = 1'b0;
    rst  = 1'b1;
    wait_cycles(8);
    rst  = 1'b0;
    // data is available but enable is low: nothing may happen
    wait_cycles(80);
    check(sq_db_count == 0 && !iosq_wr_en, "activity before enable");
    check(next_lba == START_LBA, "next_lba not loaded from start_lba");
    enable = 1'b1;

    case (SCEN)
      0: begin
        // part 1: run 40 commands, then a flush while busy
        t = 0; while (n_ok + n_err < 40 && t < 400000) begin @(posedge clk); t++; end
        check(t < 400000, "H0: watchdog (first 40 commands)");
        flush_req = 1'b1;
        t = 0; while (!flush_done && t < 100000) begin @(posedge clk); t++; end
        check(flush_done, "H0: flush_done never came (mid-run flush)");
        check(n_flush_sqe == 1, "H0: expected exactly one flush SQE");
        wait_cycles(30);
        flush_req = 1'b0;
        wait_cycles(5);
        check(!flush_done, "H0: flush_done must drop with flush_req");
        // part 2: enable gating - pause, nothing new may be submitted
        enable = 1'b0;
        wait_cycles(300);
        snap = n_sqe;
        wait_cycles(300);
        check(n_sqe == snap, "H0: SQE fetched while enable=0");
        enable = 1'b1;
        // part 3: drain everything
        t = 0; while (n_ok + n_err < N_CMDS && t < 3000000) begin @(posedge clk); t++; end
        check(t < 3000000, "H0: watchdog (all commands)");
      end
      1: begin
        t = 0; while (n_ok + n_err < N_CMDS && t < 3000000) begin @(posedge clk); t++; end
        check(t < 3000000, "H1: watchdog");
        check(max_inflight == IO_QSIZE - 1, $sformatf("H1: max outstanding %0d, expected %0d", max_inflight, IO_QSIZE - 1));
        check(sq_full_stall_cycles > 0, "H1: sq_full_stall_cycles stayed 0");
      end
      2: begin
        wait_cycles(4000);
        check(n_sqe == 3, $sformatf("H2: %0d commands submitted, expected 3", n_sqe));
        check(lba_range_err, "H2: lba_range_err not set");
      end
      3: begin   // single slot: exercises the slot_free / slot_full release race
        t = 0; while (n_ok + n_err < N_CMDS && t < 3000000) begin @(posedge clk); t++; end
        check(t < 3000000, "H3: watchdog");
      end
      4: begin   // I/O timeout, skipped Flush, restart
        t = 0; while (!io_timeout && t < 40000) begin @(posedge clk); t++; end
        check(io_timeout, "H4: io_timeout never set");
        wait_cycles(200);
        check(n_sqe == DROP_NTH + N_SLOTS, $sformatf("H4: %0d commands submitted, expected %0d (blocked behind the lost one)", n_sqe, DROP_NTH + N_SLOTS));
        check(n_ok == DROP_NTH + N_SLOTS - 1, $sformatf("H4: %0d completions", n_ok));
        snap = n_sqe;
        wait_cycles(600);
        check(n_sqe == snap, "H4: a command was submitted after io_timeout");
        check(!slots_idle, "H4: slots_idle although a command is outstanding");
        check(!flush_skipped && !flush_done, "H4: flush flags before the request");
        flush_req = 1'b1;
        t = 0; while (!flush_done && t < 100) begin @(posedge clk); t++; end
        check(flush_done, "H4: flush_done not given at once with a hung command");
        check(flush_skipped, "H4: flush_skipped not set");
        wait_cycles(50);
        check(n_flush_sqe == 0, "H4: a Flush SQE was submitted behind a hung command");
        flush_req = 1'b0;
        wait_cycles(5);
        check(!flush_done, "H4: flush_done must drop with flush_req");
        check(flush_skipped && io_timeout, "H4: sticky flags");
        check(!io_error, "H4: io_error without a failed Write");
        // restart: back to the initial state (next_slot keeps following the ring)
        enable = 1'b0;
        wait_cycles(5);
        restart = 1'b1; wait_cycles(1); restart = 1'b0;
        wait_cycles(4);
        check(!io_timeout && !flush_skipped && !io_error && !lba_range_err, "H4: flags not cleared by restart");
        check(slots_idle, "H4: outstanding not cleared by restart");
        check(next_lba == START_LBA, "H4: next_lba not reloaded by restart");
        check(sq_tail == 7'd0 && cq_head == 7'd0, "H4: queue pointers not reset by restart");
        check(bytes_done == 64'd0 && cmds_done == 64'd0, "H4: statistics not cleared by restart");
        $display("H4: io_timeout after %0d ms, Flush skipped, restart OK (sqe=%0d ok=%0d)", IO_TMO_MS, n_sqe, n_ok);
        done = 1'b1;
        wait (1'b0);
      end
      default: ;
    endcase

    // let the last CQEs / doorbells settle, then final flush
    wait_cycles(400);
    flush_req = 1'b1;
    t = 0; while (!flush_done && t < 100000) begin @(posedge clk); t++; end
    check(flush_done, "final flush_done never came");
    wait_cycles(200);
    flush_req = 1'b0;
    wait_cycles(400);

    // ---- final checks ----
    total_expected = (SCEN == 2) ? 3 : N_CMDS;
    check(n_sqe == total_expected, $sformatf("commands fetched %0d expected %0d", n_sqe, total_expected));
    check(n_ok + n_err == total_expected, $sformatf("completions %0d expected %0d", n_ok + n_err, total_expected));
    check(cmds_done == 64'(n_ok), $sformatf("cmds_done %0d expected %0d", cmds_done, n_ok));
    check(cmd_errors == 32'(n_err), $sformatf("cmd_errors %0d expected %0d", cmd_errors, n_err));
    check(bytes_done == 64'(n_ok) * 64'(SLOT_BYTES), $sformatf("bytes_done %0d", bytes_done));
    if (n_err > 0) check(last_status == {1'b0, last_err_status}, $sformatf("last_status 0x%0h", last_status));
    else           check(last_status == 16'h0, "last_status must stay 0");
    check(next_lba == START_LBA + 64'(total_expected) * 64'(LBA_PER_SLOT), "next_lba");
    check(free_pulses == total_expected, $sformatf("slot_free pulses %0d expected %0d", free_pulses, total_expected));
    check(max_latency_cycles >= 32'(tb_max_lat_lo - 2), $sformatf("max_latency %0d < tb lower bound %0d", max_latency_cycles, tb_max_lat_lo));
    check(max_latency_cycles <= 32'(tb_max_lat_lo + 40), $sformatf("max_latency %0d far above tb bound %0d", max_latency_cycles, tb_max_lat_lo));
    check(cq_consumed_m == cq_tail_m, "CQ head doorbell did not catch up with the posted CQEs");
    check(cq_head == 7'(cq_tail_m) && sq_tail == 7'(sq_head_m), "head/tail debug outputs");
    check(slots_idle, "slots_idle must be 1 at the end");
    check(io_error == (n_err > 0), $sformatf("io_error %0b but %0d failed Writes", io_error, n_err));
    if (n_err > 0) check(first_error_lba == first_err_slba_m,
                         $sformatf("first_error_lba %h expected %h", first_error_lba, first_err_slba_m));
    check(!io_timeout && !flush_skipped, "unexpected io_timeout / flush_skipped");
    check(SCEN == 2 || !lba_range_err, "unexpected lba_range_err");
    if (SCEN == 0) check(n_flush_sqe == 2, $sformatf("H0: flush SQEs %0d expected 2", n_flush_sqe));
    else           check(n_flush_sqe == 1, $sformatf("flush SQEs %0d expected 1", n_flush_sqe));
    if (SCEN == 0) check(n_err > 0, "H0: no error CQE injected");
    $display("H%0d: commands=%0d ok=%0d err=%0d sq_db=%0d cq_db=%0d (%0.2f CQEs per CQ doorbell) max_lat=%0d (tb>=%0d) stall_cycles=%0d max_outstanding=%0d",
             SCEN, n_sqe, n_ok, n_err, sq_db_count, cq_db_count, (cq_db_count != 0) ? (real'(cq_posted_total) / real'(cq_db_count)) : 0.0,
             max_latency_cycles, tb_max_lat_lo, sq_full_stall_cycles, max_inflight);
    done = 1'b1;
  end
endmodule

module tb_write_engine;
  logic clk = 1'b0;
  always #4 clk = ~clk;

  logic [4:0] done;
  int e0, e1, e2, e3, e4;

  we_harness #(.SCEN(0), .N_SLOTS(4),  .SLOT_BYTES(131072), .LBADS(9),  .N_CMDS(600),
               .LAT_MIN(20),  .LAT_MAX(150), .ERR_EVERY(17)) h0 (.clk(clk), .done(done[0]), .errors(e0));
  we_harness #(.SCEN(1), .N_SLOTS(70), .SLOT_BYTES(4096),   .LBADS(9),  .N_CMDS(300),
               .LAT_MIN(400), .LAT_MAX(900), .ERR_EVERY(0)) h1 (.clk(clk), .done(done[1]), .errors(e1));
  we_harness #(.SCEN(2), .N_SLOTS(4),  .SLOT_BYTES(8192),   .LBADS(12), .N_CMDS(3),
               .LAT_MIN(10),  .LAT_MAX(40), .ERR_EVERY(0),
               .START_LBA(64'h0000_0001_FFFF_FFFE), .NSZE(64'h0000_0002_0000_0005))
               h2 (.clk(clk), .done(done[2]), .errors(e2));

  we_harness #(.SCEN(3), .N_SLOTS(1),  .SLOT_BYTES(4096),   .LBADS(9),  .N_CMDS(80),
               .LAT_MIN(5),  .LAT_MAX(30), .ERR_EVERY(0)) h3 (.clk(clk), .done(done[3]), .errors(e3));

  we_harness #(.SCEN(4), .N_SLOTS(4),  .SLOT_BYTES(4096),   .LBADS(9),  .N_CMDS(12),
               .LAT_MIN(10),  .LAT_MAX(30), .ERR_EVERY(0), .IO_TMO_MS(3), .DROP_NTH(5))
               h4 (.clk(clk), .done(done[4]), .errors(e4));

  initial begin
    wait (&done);
    repeat (10) @(posedge clk);
    if (e0 + e1 + e2 + e3 + e4 == 0) $display("TEST PASSED");
    else $fatal(1, "TEST FAILED: %0d errors", e0 + e1 + e2 + e3 + e4);
    $finish;
  end
  initial begin
    #100ms;
    $fatal(1, "TEST FAILED: global watchdog");
  end
endmodule
