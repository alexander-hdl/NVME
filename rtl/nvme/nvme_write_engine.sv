// =============================================================================
// nvme_write_engine.sv - turns full data-ring slots into NVMe Write commands,
// reaps the completions and releases the slots.   ARCHITECTURE.md section 5.16
//
// Overview
// --------
//   data_ring --slot_full[k]--> [submit FSM] --SQE--> iosq_ram --doorbell--> SSD
//   data_ring <--slot_free[k]-- [reap logic] <--CQE-- iocq_ram <--MWr-------- SSD
//
// Slot k is the command with CID = k.  One command writes SLOT_BYTES bytes of
// ring slot k to LBAs  next_lba ... next_lba + (SLOT_BYTES>>lbads) - 1.
//
// Submit FSM  (sub_state)
//   SUB_IDLE   pick the next ring slot (round robin) when
//                enable && !flush_req && slot_full[k] && !busy[k] && SQ not full
//              (busy[k] gates a slot until its CQE arrived; wait_clear[k] gates it
//               until data_ring has dropped slot_full[k] after our slot_free pulse)
//              or the Flush command when flush_req and no data command is busy.
//              The LBA range guard (next_lba + NLB+1 <= nsze) sets lba_range_err
//              and stops submitting.
//   SUB_WR     write the 4 beats (16 bytes each) of the SQE into iosq_ram.
//   SUB_COMMIT tail++, busy[k]=1, next_lba += LBAs per slot, schedule the SQ doorbell.
//
// SQE (Write): DW0 {CID=k, 0, OPC=1}, DW1 NSID=1, DW6 PRP1 = DATA_BASE + k*SLOT_BYTES,
//   DW8 PRP2 = PRP_BASE + k*4096 (PRP list, synthesised by the completer; for
//   SLOT_BYTES<=4096 PRP2=0, for 8192 PRP2 = second data page),
//   DW10/11 SLBA, DW12 NLB = (SLOT_BYTES>>lbads)-1, DW13 = 0x40 (sequential hint).
// SQE (Flush): DW0 {CID=0xFF, 0, OPC=0}, DW1 NSID=1.
//
// Doorbell FSM (db_state)
//   Only requester command port 1 (MWr) is used.  Two requests are kept as
//   "dirty" flags:  SQ tail doorbell (set when an SQE is committed) and CQ head
//   doorbell (set when a CQE is reaped).  A doorbell always carries the *latest*
//   tail / head, so several submits or reaps that happen before the MWr goes
//   out are coalesced.  The CQ doorbell is only sent when the reaper is quiet
//   (no CQE consumed in the current cycle), so a burst of CQEs costs one MWr.
//   SQ and CQ doorbells alternate when both are pending.
//   MWr SQ1TDBL / CQ1HDBL addresses come from doorbell_addr(dstrd, 1, is_cq).
//
// Reap logic
//   iocq_ram[head] is read EVERY cycle (iocq_rd_en = 1); data returns two
//   cycles later.  A result is a new CQE if DW3[16] == phase.  When a CQE is
//   consumed, the address of the read issued in the same cycle is already the
//   next head, but the read issued one cycle earlier (still the old head)
//   returns in the following cycle: that result is discarded (skip_r).
//   Hits are only evaluated while commands are outstanding.  Per CQE:
//     CID<N_SLOTS : slot_free[CID] pulse, busy[CID]=0, bytes/cmds/latency stats
//     CID==0xFF   : Flush done
//     status!=0   : cmd_errors++, last_status = {0, DW3[31:17]}
//   head++ (wrap -> phase flips), CQ doorbell scheduled.
//
// Stats: bytes_done / cmds_done count successful Writes only; cmd_errors counts
//   failed or unexpected completions; max_latency_cycles = max over commands of
//   (cycle of CQE reap) - (cycle the SQ doorbell MWr was accepted by the requester);
//   sq_full_stall is high (and counted in sq_full_stall_cycles) while a slot is
//   ready but the SQ has no room.
//
// Start/stop: next_lba is loaded from start_lba while enable has never been
// asserted.  The engine supports one run per reset.
// =============================================================================
module nvme_write_engine
  import nvme_host_pkg::*;
#(
  parameter int unsigned N_SLOTS    = N_SLOTS_DEF,
  parameter int unsigned SLOT_BYTES = SLOT_BYTES_DEF
) (
  input  logic        clk,
  input  logic        rst,
  input  logic        tick_ms,            // unused

  input  logic        enable,
  input  logic [63:0] start_lba,
  input  logic [3:0]  lbads,
  input  logic [63:0] nsze,
  input  logic [3:0]  dstrd,

  input  logic [N_SLOTS-1:0] slot_full,
  output logic [N_SLOTS-1:0] slot_free,   // 1-cycle pulse per released slot

  // iosq_ram write port (entry*4 + beat), iocq_ram read port (2-cycle latency)
  output logic         iosq_wr_en,
  output logic [7:0]   iosq_wr_addr,
  output logic [127:0] iosq_wr_data,
  output logic         iocq_rd_en,
  output logic [5:0]   iocq_rd_addr,
  input  logic [127:0] iocq_rd_data,

  // requester command port 1 (MWr only)
  output logic        req_valid,
  input  logic        req_ready,
  output logic [1:0]  req_op,
  output logic [31:0] req_addr,
  output logic [31:0] req_wdata,
  output logic [3:0]  req_be,
  input  logic        rsp_valid,
  input  logic [2:0]  rsp_status,

  input  logic        flush_req,
  output logic        flush_done,         // level, stays 1 until flush_req drops

  output logic [63:0] next_lba,
  output logic [63:0] bytes_done,
  output logic [63:0] cmds_done,
  output logic [31:0] cmd_errors,
  output logic [15:0] last_status,
  output logic [31:0] max_latency_cycles,
  output logic        sq_full_stall,
  output logic [31:0] sq_full_stall_cycles,
  output logic [6:0]  sq_tail,
  output logic [6:0]  cq_head,
  output logic        slots_idle,         // no command outstanding, not submitting
  output logic        lba_range_err,      // sticky: next write would exceed nsze
  output logic        doorbell_err        // sticky: a doorbell MWr was not completed with SC
);

  // ---------------------------------------------------------------------------
  // Constants
  // ---------------------------------------------------------------------------
  localparam int unsigned QW       = $clog2(IO_QSIZE);                 // queue index width (6)
  localparam int unsigned SLOT_W   = (N_SLOTS > 1) ? $clog2(N_SLOTS) : 1;
  localparam logic [15:0] FLUSH_CID = 16'h00FF;
  localparam logic [1:0]  OP_MWR32  = 2'd3;
  localparam logic [31:0] DSM_SEQ_HINT = 32'h0000_0040;               // CDW13: sequential request
  localparam logic [31:0] NSID_1       = 32'h0000_0001;
  localparam int unsigned SLOT_PAGES   = SLOT_BYTES / 4096;
  localparam int unsigned MAX_OUTSTANDING = IO_QSIZE - 1;             // SQ holds at most size-1 entries

  typedef enum logic [1:0] { SUB_IDLE, SUB_WR, SUB_COMMIT } sub_state_t;
  typedef enum logic [1:0] { DB_IDLE, DB_REQ, DB_RSP } db_state_t;
  sub_state_t sub_state;
  db_state_t  db_state;

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------
  logic [QW-1:0]       sq_tail_r, cq_head_r;
  logic                cq_phase;
  logic [N_SLOTS-1:0]  busy;
  logic [N_SLOTS-1:0]  wait_clear;
  logic [SLOT_W-1:0]   next_slot;
  logic [7:0]          outstanding;
  logic                started;

  // current submission
  logic                cur_flush;
  logic [SLOT_W-1:0]   cur_slot;
  logic [63:0]         cur_lba;
  logic [1:0]          cur_beat;

  // flush
  logic                flush_sent, flush_complete;

  // reaper
  logic                rd_v1, rd_v2, skip_r;

  // doorbells
  logic                sq_db_dirty, cq_db_dirty;
  logic                db_last_sq;
  logic                db_is_sq;
  logic [31:0]         db_addr_r, db_data_r;
  logic [N_SLOTS-1:0]  ts_pending, ts_snap;

  // latency
  logic [31:0]         cyc;
  logic [31:0]         t_start [N_SLOTS];

  // ---------------------------------------------------------------------------
  // Derived values
  // ---------------------------------------------------------------------------
  logic [63:0] lba_per_slot;
  assign lba_per_slot = 64'(SLOT_BYTES) >> lbads;

  logic [SLOT_W-1:0] last_slot_idx;
  assign last_slot_idx = SLOT_W'(N_SLOTS - 1);

  // submit decision
  logic want_data, lba_overflow, sq_blocked, want_flush;
  assign sq_blocked   = (outstanding >= 8'(MAX_OUTSTANDING));
  assign lba_overflow = ((65'(next_lba) + 65'(lba_per_slot)) > 65'(nsze)) || (lba_per_slot == 64'd0)
                        || (lba_per_slot > 64'd65536);
  assign want_data    = enable && !flush_req && !lba_range_err && slot_full[next_slot]
                        && !busy[next_slot] && !wait_clear[next_slot];
  assign want_flush   = flush_req && !flush_sent && !flush_complete && (busy == '0)
                        && (outstanding == 8'd0);

  // reaper
  logic        reap_active, reap_valid, reap_hit;
  logic [QW-1:0] cq_head_next;
  logic [15:0] cqe_cid;
  logic [14:0] cqe_status;
  logic        cqe_is_flush, cqe_is_slot, cqe_expected;
  logic [SLOT_W-1:0] cqe_slot;

  assign reap_active = (outstanding != 8'd0);
  assign reap_valid  = rd_v2 && !skip_r;
  assign reap_hit    = reap_active && reap_valid && (iocq_rd_data[112] == cq_phase);
  assign cq_head_next = (cq_head_r == QW'(IO_QSIZE-1)) ? '0 : cq_head_r + 1'b1;
  assign cqe_cid     = iocq_rd_data[111:96];
  assign cqe_status  = iocq_rd_data[127:113];
  assign cqe_is_flush = (cqe_cid == FLUSH_CID);
  assign cqe_is_slot  = (cqe_cid < 16'(N_SLOTS));
  assign cqe_slot     = cqe_cid[SLOT_W-1:0];
  assign cqe_expected = cqe_is_flush ? (flush_sent && !flush_complete)
                                     : (cqe_is_slot && busy[cqe_slot]);

  assign iocq_rd_en   = 1'b1;
  assign iocq_rd_addr = 6'(reap_hit ? cq_head_next : cq_head_r);

  // ---------------------------------------------------------------------------
  // SQE construction
  // ---------------------------------------------------------------------------
  logic [31:0] slot_data_addr, slot_prp_list_addr, prp2_value;
  assign slot_data_addr     = DATA_BASE + 32'(cur_slot) * 32'(SLOT_BYTES);
  assign slot_prp_list_addr = PRP_BASE  + 32'(cur_slot) * 32'h1000;
  assign prp2_value = (SLOT_PAGES <= 1) ? 32'h0 :
                      (SLOT_PAGES == 2) ? (slot_data_addr + 32'h1000) : slot_prp_list_addr;

  logic [15:0] nlb_m1;
  assign nlb_m1 = 16'(lba_per_slot - 64'd1);

  always_comb begin
    iosq_wr_en   = (sub_state == SUB_WR);
    iosq_wr_addr = 8'({sq_tail_r, cur_beat});
    iosq_wr_data = 128'h0;
    case (cur_beat)
      2'd0: iosq_wr_data = {32'h0, 32'h0, NSID_1,
                            cur_flush ? {FLUSH_CID, 8'h00, NOPC_FLUSH}
                                      : {16'(cur_slot), 8'h00, NOPC_WRITE}};
      2'd1: iosq_wr_data = cur_flush ? 128'h0 : {32'h0, slot_data_addr, 64'h0};                  // DW7 DW6 DW5 DW4
      2'd2: iosq_wr_data = cur_flush ? 128'h0 : {cur_lba[63:32], cur_lba[31:0], 32'h0, prp2_value}; // DW11 DW10 DW9 DW8
      default: iosq_wr_data = cur_flush ? 128'h0 : {32'h0, 32'h0, DSM_SEQ_HINT, {16'h0, nlb_m1}};   // DW15 DW14 DW13 DW12
    endcase
  end

  // ---------------------------------------------------------------------------
  // Doorbell requester port
  // ---------------------------------------------------------------------------
  assign req_valid = (db_state == DB_REQ);
  assign req_op    = OP_MWR32;
  assign req_addr  = db_addr_r;
  assign req_wdata = db_data_r;
  assign req_be    = 4'hF;

  // The CQ doorbell waits until the reaper is quiet (no CQE consumed now, none
  // just consumed) so that a burst of CQEs is acknowledged by one MWr.
  // When both doorbells are pending they alternate.
  logic cq_quiet, pick_sq, pick_cq;
  assign cq_quiet = !reap_hit && !skip_r;
  assign pick_sq  = sq_db_dirty && !(cq_db_dirty && cq_quiet && db_last_sq);
  assign pick_cq  = !pick_sq && cq_db_dirty && cq_quiet;

  // ---------------------------------------------------------------------------
  // Outputs / status
  // ---------------------------------------------------------------------------
  assign sq_tail     = 7'(sq_tail_r);
  assign cq_head     = 7'(cq_head_r);
  assign flush_done  = flush_complete;
  assign slots_idle  = (outstanding == 8'd0) && (sub_state == SUB_IDLE);
  assign sq_full_stall = want_data && !lba_overflow && sq_blocked && (sub_state == SUB_IDLE);

  // ---------------------------------------------------------------------------
  // Main sequential logic
  // ---------------------------------------------------------------------------
  logic        do_commit;
  logic [31:0] lat;
  assign do_commit = (sub_state == SUB_COMMIT);
  assign lat = cyc - t_start[cqe_slot];

  always_ff @(posedge clk) begin
    if (rst) begin
      sub_state          <= SUB_IDLE;
      db_state           <= DB_IDLE;
      sq_tail_r          <= '0;
      cq_head_r          <= '0;
      cq_phase           <= 1'b1;
      busy               <= '0;
      wait_clear         <= '0;
      next_slot          <= '0;
      outstanding        <= 8'd0;
      started            <= 1'b0;
      cur_flush          <= 1'b0;
      cur_slot           <= '0;
      cur_lba            <= 64'd0;
      cur_beat           <= 2'd0;
      flush_sent         <= 1'b0;
      flush_complete     <= 1'b0;
      rd_v1              <= 1'b0;
      rd_v2              <= 1'b0;
      skip_r             <= 1'b0;
      sq_db_dirty        <= 1'b0;
      cq_db_dirty        <= 1'b0;
      db_last_sq         <= 1'b0;
      db_is_sq           <= 1'b0;
      db_addr_r          <= 32'h0;
      db_data_r          <= 32'h0;
      ts_pending         <= '0;
      ts_snap            <= '0;
      cyc                <= 32'd0;
      slot_free          <= '0;
      next_lba           <= 64'd0;
      bytes_done         <= 64'd0;
      cmds_done          <= 64'd0;
      cmd_errors         <= 32'd0;
      last_status        <= 16'd0;
      max_latency_cycles <= 32'd0;
      sq_full_stall_cycles <= 32'd0;
      lba_range_err      <= 1'b0;
      doorbell_err       <= 1'b0;
      for (int i = 0; i < N_SLOTS; i++) t_start[i] <= 32'd0;
    end else begin
      cyc       <= cyc + 1'b1;
      slot_free <= '0;
      rd_v1     <= 1'b1;
      rd_v2     <= rd_v1;
      skip_r    <= reap_hit;                 // the read issued before this consume is stale

      if (!started) next_lba <= start_lba;
      if (enable)   started  <= 1'b1;
      if (sq_full_stall && sq_full_stall_cycles != 32'hFFFF_FFFF) sq_full_stall_cycles <= sq_full_stall_cycles + 1'b1;

      if (!flush_req) begin
        flush_sent     <= 1'b0;
        flush_complete <= 1'b0;
      end

      // ------------------------------------------------------------------
      // Submit FSM
      // ------------------------------------------------------------------
      case (sub_state)
        SUB_IDLE: begin
          if (want_data && lba_overflow) begin
            lba_range_err <= 1'b1;              // stop: this write would run past the namespace
          end else if (want_data && !sq_blocked) begin
            cur_flush <= 1'b0;
            cur_slot  <= next_slot;
            cur_lba   <= next_lba;
            cur_beat  <= 2'd0;
            sub_state <= SUB_WR;
          end else if (want_flush) begin
            cur_flush  <= 1'b1;
            cur_slot   <= '0;
            cur_lba    <= 64'd0;
            cur_beat   <= 2'd0;
            flush_sent <= 1'b1;
            sub_state  <= SUB_WR;
          end
        end
        SUB_WR: begin                           // iosq_wr_en high: one beat per cycle
          cur_beat <= cur_beat + 1'b1;
          if (cur_beat == 2'd3) sub_state <= SUB_COMMIT;
        end
        SUB_COMMIT: begin
          sq_tail_r   <= (sq_tail_r == QW'(IO_QSIZE-1)) ? '0 : sq_tail_r + 1'b1;
          if (!cur_flush) begin
            busy[cur_slot]       <= 1'b1;
            next_lba             <= next_lba + lba_per_slot;
            next_slot            <= (next_slot == last_slot_idx) ? '0 : next_slot + 1'b1;
          end
          sub_state <= SUB_IDLE;
        end
        default: sub_state <= SUB_IDLE;
      endcase

      // ------------------------------------------------------------------
      // Reap: one CQE per hit
      // ------------------------------------------------------------------
      if (reap_hit) begin
        cq_head_r   <= cq_head_next;
        if (cq_head_r == QW'(IO_QSIZE-1)) cq_phase <= ~cq_phase;
        if (cqe_status != 15'd0) begin
          last_status <= {1'b0, cqe_status};
        end
        if (!cqe_expected) begin
          cmd_errors <= cmd_errors + 1'b1;               // CQE for a command we did not submit
        end else begin
          if (cqe_status != 15'd0) cmd_errors <= cmd_errors + 1'b1;
          if (cqe_is_flush) begin
            flush_complete <= 1'b1;
          end else begin
            busy[cqe_slot]       <= 1'b0;
            wait_clear[cqe_slot] <= 1'b1;
            slot_free[cqe_slot]  <= 1'b1;
            if (cqe_status == 15'd0) begin
              bytes_done <= bytes_done + 64'(SLOT_BYTES);
              cmds_done  <= cmds_done + 64'd1;
            end
            if (lat > max_latency_cycles) max_latency_cycles <= lat;
          end
        end
      end
      // outstanding counts commands between SUB_COMMIT and their CQE
      case ({do_commit, reap_hit && cqe_expected})
        2'b10:   outstanding <= outstanding + 8'd1;
        2'b01:   outstanding <= outstanding - 8'd1;
        default: ;
      endcase
      // ring slot becomes available again once data_ring has dropped slot_full
      for (int i = 0; i < N_SLOTS; i++)
        if (wait_clear[i] && !slot_full[i] && !(reap_hit && cqe_expected && cqe_is_slot && cqe_slot == SLOT_W'(i)))
          wait_clear[i] <= 1'b0;

      // ------------------------------------------------------------------
      // Doorbell FSM
      // ------------------------------------------------------------------
      case (db_state)
        DB_IDLE: begin
          if (pick_sq) begin
            // SQ doorbell: latest tail
            db_is_sq    <= 1'b1;
            db_last_sq  <= 1'b1;
            db_addr_r   <= doorbell_addr(dstrd, 1, 1'b0);
            db_data_r   <= 32'(sq_tail_r);
            ts_snap     <= ts_pending;
            ts_pending  <= '0;
            sq_db_dirty <= 1'b0;
            db_state    <= DB_REQ;
          end else if (pick_cq) begin
            // CQ doorbell: latest head (reaper is quiet, so head is stable)
            db_is_sq    <= 1'b0;
            db_last_sq  <= 1'b0;
            db_addr_r   <= doorbell_addr(dstrd, 1, 1'b1);
            db_data_r   <= 32'(cq_head_r);
            cq_db_dirty <= 1'b0;
            db_state    <= DB_REQ;
          end
        end
        DB_REQ: begin
          if (req_ready) begin
            if (db_is_sq)
              for (int i = 0; i < N_SLOTS; i++)
                if (ts_snap[i]) t_start[i] <= cyc;
            db_state <= rsp_valid ? DB_IDLE : DB_RSP;
            if (rsp_valid && rsp_status != 3'd0) doorbell_err <= 1'b1;
          end
        end
        DB_RSP: begin
          if (rsp_valid) begin
            if (rsp_status != 3'd0) doorbell_err <= 1'b1;
            db_state <= DB_IDLE;
          end
        end
        default: db_state <= DB_IDLE;
      endcase

      // Setting the "dirty" flags comes last so that a flag raised in the same
      // cycle in which the doorbell FSM consumed the previous request survives.
      if (do_commit)  sq_db_dirty <= 1'b1;
      if (reap_hit)   cq_db_dirty <= 1'b1;
      if (do_commit && !cur_flush) ts_pending[cur_slot] <= 1'b1;
    end
  end

endmodule
