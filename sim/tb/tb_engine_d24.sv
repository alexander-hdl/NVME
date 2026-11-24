// =============================================================================
// tb_engine_d24 - audit D24: READ commands must not move the write-ring slot pointer of nvme_write_engine.
//   Phase A: read_mode, max_cmds = 3 (3 mod N_SLOTS != 0), the data ring does not move (no slot_free).
//   Phase B: restart, write mode; the ring offers 8 slots (0,1,2,3,0,...).  The Writes must use slots 0,1,2,3,0,... in
//            order (SQE CID = slot), each one only when its slot is full.  The old engine started with slot 3 (the
//            Reads advanced next_slot) -> wrong CID order / hang.
// Minimal SSD stub: fetches SQEs on the SQ1 doorbell and completes them in order with status 0.
// =============================================================================
`timescale 1ns/1ps

module tb_engine_d24;
  import nvme_host_pkg::*;
  localparam int N_SLOTS = 4, SLOT_BYTES = 16384, LBADS = 9, N_RD = 3, N_WR = 8;

  logic clk = 1'b0;
  always #4 clk = ~clk;
  logic rst = 1'b1;
  int errors = 0;
  task automatic check(input bit c, input string m);
    if (!c) begin errors++; $display("[%0t] ERROR: %s", $time, m); end
  endtask

  logic enable = 1'b0, read_mode = 1'b0, restart = 1'b0;
  logic [31:0] max_cmds = 32'd0;
  logic [N_SLOTS-1:0] slot_full = '0, slot_free;
  logic [N_SLOTS*64-1:0] slot_lba_flat;
  logic cmd_limit_done;
  logic iosq_wr_en; logic [7:0] iosq_wr_addr; logic [127:0] iosq_wr_data;
  logic iocq_rd_en; logic [5:0] iocq_rd_addr; logic [127:0] iocq_rd_data;
  logic req_valid, req_ready; logic [1:0] req_op; logic [31:0] req_addr, req_wdata; logic [3:0] req_be;
  logic rsp_valid = 1'b0; logic [2:0] rsp_status = 3'd0;
  logic flush_req = 1'b0, flush_done;
  logic [63:0] next_lba, bytes_done, cmds_done, first_error_lba;
  logic [31:0] cmd_errors, max_latency_cycles, sq_full_stall_cycles;
  logic [15:0] last_status;
  logic sq_full_stall, slots_idle, lba_range_err, doorbell_err, io_timeout, flush_skipped, io_error;
  logic [6:0] sq_tail, cq_head;

  nvme_write_engine #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES), .IO_TIMEOUT_MS(0)) dut (
    .clk(clk), .rst(rst), .tick_ms(1'b0), .restart(restart),
    .enable(enable), .read_mode(read_mode), .max_cmds(max_cmds), .start_lba(64'h1000), .lbads(4'(LBADS)),
    .nsze(64'h0000_0000_7470_0000), .dstrd(4'd0),
    .slot_full(slot_full), .slot_free(slot_free), .slot_lba_flat(slot_lba_flat), .cmd_limit_done(cmd_limit_done),
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

  // ---- queue RAMs ----
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

  // ---- data ring model: fill_en lets slots fill in order; slot_free clears slot_full ----
  bit fill_en = 1'b0;
  int fill_ptr = 0, fills = 0, fill_limit = 0;
  logic [N_SLOTS-1:0] free_d1 = '0, free_d2 = '0;
  always_ff @(posedge clk) begin
    free_d1 <= slot_free; free_d2 <= free_d1;
    for (int k = 0; k < N_SLOTS; k++) if (free_d2[k]) slot_full[k] <= 1'b0;
    if (!rst && fill_en && fills < fill_limit && !slot_full[fill_ptr] && !free_d1[fill_ptr] && !free_d2[fill_ptr] && !slot_free[fill_ptr]) begin
      slot_full[fill_ptr] <= 1'b1; fills <= fills + 1; fill_ptr <= (fill_ptr + 1) % N_SLOTS;
    end
  end

  // ---- SSD stub ----
  logic pend_rsp = 1'b0;
  assign req_ready = !pend_rsp;
  int sq_head = 0, cq_tail = 0;
  logic cq_ph = 1'b1;
  int n_rd_sqe = 0, n_wr_sqe = 0;
  int cmd_q [$];                       // CIDs fetched, not yet completed
  always @(posedge clk) begin : ssd
    int off, q, v, nf, cid;
    logic [511:0] got;
    rsp_valid <= 1'b0;
    pend_rsp  <= 1'b0;
    if (restart) begin                 // the core re-creates the queues at init_start: SSD side restarts too
      sq_head = 0; cq_tail = 0; cq_ph = 1'b1; cmd_q.delete();
      for (int i = 0; i < 64; i++) iocq_mem[i] <= '0;
    end else if (!rst) begin
      if (req_valid && req_ready) begin
        pend_rsp <= 1'b1;
        off = req_addr - BAR0_ADDR; q = (off - 'h1000) / 4; v = req_wdata[15:0];
        if (q % 2 == 0) begin
          nf = (v - sq_head + IO_QSIZE) % IO_QSIZE;
          for (int e = 0; e < nf; e++) begin
            for (int b = 0; b < 4; b++) got[128*b +: 128] = iosq_mem[sq_head*4 + b];
            cid = got[31:16];
            if (read_mode) begin
              check(got[7:0] == 8'h02, "phase A: data SQE is not a Read");
              check(cid == n_rd_sqe % N_SLOTS, $sformatf("phase A: Read #%0d has CID %0d", n_rd_sqe, cid));
              n_rd_sqe++;
            end else begin
              check(got[7:0] == 8'h01, "phase B: data SQE is not a Write");
              check(cid == n_wr_sqe % N_SLOTS, $sformatf("phase B: Write #%0d uses ring slot %0d, expected %0d (D24)", n_wr_sqe, cid, n_wr_sqe % N_SLOTS));
              check(slot_full[cid % N_SLOTS], $sformatf("phase B: Write #%0d submitted while its slot %0d is empty", n_wr_sqe, cid));
              n_wr_sqe++;
            end
            cmd_q.push_back(cid);
            sq_head = (sq_head + 1) % IO_QSIZE;
          end
        end
      end
      if (pend_rsp) rsp_valid <= 1'b1;
      if (cmd_q.size() > 0 && ($urandom % 8 == 0)) begin
        cid = cmd_q.pop_front();
        iocq_mem[cq_tail] <= {15'h0, cq_ph, 16'(cid), 16'h0001, 16'(sq_head), 32'h0, 32'h0};
        if (cq_tail == IO_QSIZE - 1) begin cq_tail = 0; cq_ph = ~cq_ph; end else cq_tail = cq_tail + 1;
      end
    end
  end

  task automatic wait_cycles(input int n); repeat (n) @(posedge clk); endtask

  initial begin
    int t;
    wait_cycles(8); rst = 1'b0; wait_cycles(20);
    // ---- phase A: 3 Reads, ring idle ----
    read_mode = 1'b1; max_cmds = 32'(N_RD); enable = 1'b1;
    t = 0; while (!cmd_limit_done && t < 20000) begin @(posedge clk); t++; end
    check(cmd_limit_done, "phase A: cmd_limit_done never came");
    wait_cycles(200);
    check(n_rd_sqe == N_RD && slots_idle, "phase A: Reads not finished");
    // ---- phase B: new write session on the same hardware (restart, as the core does at init_start) ----
    enable = 1'b0; wait_cycles(5);
    read_mode = 1'b0; max_cmds = 32'd0;
    restart = 1'b1; wait_cycles(1); restart = 1'b0; wait_cycles(5);
    fill_limit = N_WR; fill_en = 1'b1;
    enable = 1'b1;
    t = 0; while (n_wr_sqe < N_WR && t < 30000) begin @(posedge clk); t++; end
    check(n_wr_sqe == N_WR, $sformatf("phase B: only %0d of %0d Writes submitted (hang)", n_wr_sqe, N_WR));
    wait_cycles(1000);
    check(cmds_done == 64'(N_WR) && cmd_errors == 0, $sformatf("phase B: cmds_done %0d errors %0d", cmds_done, cmd_errors));
    check(slots_idle && !lba_range_err && !doorbell_err, "phase B: end state");
    if (errors == 0) $display("TEST PASSED");
    else $fatal(1, "TEST FAILED: %0d errors", errors);
    $finish;
  end
  initial begin #20ms; $fatal(1, "TEST FAILED: watchdog"); end
endmodule
