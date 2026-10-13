// =============================================================================
// nvme_admin.sv - NVMe controller bring-up (admin queue) and orderly shutdown.
// ARCHITECTURE.md section 5.15; register values from research/02 sections 1, 2, 9, 10, 12.
//
// What this module is
// -------------------
// A small FSM that plays the role of the "NVMe driver" initialisation code.  It
//   * reads / writes the SSD's BAR0 registers through requester command port 0
//     (req_* / rsp_* signature of rp_requester, ARCHITECTURE.md 5.10/5.11),
//   * places 64-byte submission queue entries (SQEs) in asq_ram (16 B per beat,
//     4 beats per SQE) and rings the admin SQ tail doorbell,
//   * polls acq_ram (written by the SSD through hostmem_writer) for the CQE,
//   * reads the Identify data back out of ident_ram.
//
// All NVMe values are little-endian DWs.  A DW i of an SQE lives at bits
// [32*(i%4) +: 32] of RAM beat (i/4).  A CQE occupies one 16-byte beat:
// DW0 in [31:0] ... DW3 in [127:96].
//
// FSM overview (single "state" register, three groups of states)
// --------------------------------------------------------------
//  1. MMIO helper states       S_MM_REQ / S_MM_RSP
//       Every BAR0 access sets mm_* registers (task mm_rd / mm_wr) and jumps
//       to S_MM_REQ.  S_MM_REQ drives req_valid until req_ready, S_MM_RSP
//       waits for rsp_valid, checks rsp_status (!= SC -> fail) and continues
//       in the state stored in mm_next.
//  2. Controller enable        S_CAP_LO_DONE ... S_REGSEQ_NEXT, S_CSTS_*
//       CAP lo/hi -> CC -> (EN? write 0, poll RDY==0) -> clear CQs ->
//       AQA, ASQ lo/hi, ACQ lo/hi, CC_CONFIG, CC_ENABLE -> poll RDY==1.
//       CSTS polling: one read, then wait for the next tick_ms (1 ms), repeat.
//       Timeout = (CAP.TO+1)*500 ms (capped by RDY_TIMEOUT_MS_MAX).
//  3. admin_cmd "sub-FSM"      S_CMD_START, S_AC_WR, S_AC_*, S_EX_*, S_CMD_FINISH
//       build SQE -> 4 beat writes to asq_ram -> SQ0TDBL doorbell ->
//       poll acq_ram[head] until DW3[16]==phase -> check status ->
//       head++ (phase flips on wrap) -> CQ0HDBL doorbell -> (Identify only)
//       extract fields from ident_ram.
//     The init command list and the shutdown command list both run through it.
//
// Init command order (CID = running counter, starts at 0 on start_init):
//   Identify Controller, Identify Namespace, Set Features(FID 7),
//   Create IO CQ 1, Create IO SQ 1                       -> init_done
// Shutdown (start_shutdown): Delete IO SQ 1, Delete IO CQ 1, CC=CC_SHUTDOWN,
//   poll CSTS.SHST==10b every ms (10 s timeout -> still shutdown_done, with
//   shutdown_timeout=1).
//
// Level outputs: init_done / init_fail / shutdown_done stay set until the next
// start_init / start_shutdown.  start_* are sampled in S_IDLE only (pulse is fine).
//
// RAM timing contract: a read issued with *_rd_en in cycle t returns data
// that is valid during cycle t+2.
// =============================================================================
module nvme_admin
  import nvme_host_pkg::*;
#(
  parameter int unsigned RDY_TIMEOUT_MS_MAX  = 128_000,  // upper cap of (CAP.TO+1)*500 ms (TO max = 255)
  parameter int unsigned ADMIN_TIMEOUT_MS    = 1_000,    // wait for one admin CQE
  parameter int unsigned SHUTDOWN_TIMEOUT_MS = 10_000    // wait for CSTS.SHST == 10b
) (
  input  logic        clk,
  input  logic        rst,
  input  logic        tick_ms,           // 1 pulse per ms (per us with SIM_FAST)

  // ---- control / status ----
  input  logic        start_init,
  output logic        init_done,
  output logic        init_fail,
  output logic [7:0]  fail_step,         // see F_* codes below
  output logic [15:0] cqe_status_last,   // {1'b0, DW3[31:17]} of the most recent CQE
  input  logic        start_shutdown,
  output logic        shutdown_done,
  output logic        shutdown_timeout,  // shutdown_done reached without SHST==10b (or a bus error)

  // ---- results of Identify / CAP ----
  output logic [3:0]  lbads,
  output logic [63:0] nsze,
  output logic [7:0]  mdts,
  output logic [3:0]  dstrd,
  output logic        vwc,

  // ---- CQ clear handshake (hostmem_rams) ----
  output logic        clear_cqs,         // 1-cycle pulse
  input  logic        clear_done,        // goes high when the zero fill has finished

  output logic [7:0]  dbg_asq_tail,
  output logic [7:0]  dbg_acq_head,

  // ---- requester command port 0 ----
  output logic        req_valid,
  input  logic        req_ready,
  output logic [1:0]  req_op,            // 2=MRD32, 3=MWR32
  output logic [31:0] req_addr,
  output logic [31:0] req_wdata,
  output logic [3:0]  req_be,
  input  logic        rsp_valid,
  input  logic [2:0]  rsp_status,
  input  logic [31:0] rsp_rdata,

  // ---- asq_ram write port ----
  output logic         asq_wr_en,
  output logic [6:0]   asq_wr_addr,      // entry*4 + beat
  output logic [127:0] asq_wr_data,

  // ---- acq_ram read port (2-cycle latency) ----
  output logic         acq_rd_en,
  output logic [4:0]   acq_rd_addr,
  input  logic [127:0] acq_rd_data,

  // ---- ident_ram read port (2-cycle latency) ----
  output logic         ident_rd_en,
  output logic [7:0]   ident_rd_addr,    // 16-byte beat index
  input  logic [127:0] ident_rd_data
);

  // ---------------------------------------------------------------------------
  // Constants
  // ---------------------------------------------------------------------------
  localparam logic [1:0] OP_MRD32 = 2'd2;
  localparam logic [1:0] OP_MWR32 = 2'd3;
  localparam logic [2:0] RSP_SC   = 3'd0;

  localparam int unsigned ASQ_W = $clog2(ADMIN_QSIZE);   // entry index width (5)

  // fail_step codes
  localparam logic [7:0] F_NONE       = 8'h00;
  localparam logic [7:0] F_CAP_LO     = 8'h01;  // BAR0 access error: CAP low
  localparam logic [7:0] F_CAP_HI     = 8'h02;  // BAR0 access error: CAP high
  localparam logic [7:0] F_CAP_UNSUP  = 8'h03;  // CAP.MPSMIN != 0 or CAP.CSS[37]==0
  localparam logic [7:0] F_CAP_MQES   = 8'h04;  // CAP.MQES too small for IO_QSIZE
  localparam logic [7:0] F_CC_RD      = 8'h05;  // BAR0 access error: CC read
  localparam logic [7:0] F_CC_DISABLE = 8'h06;  // BAR0 access error: CC <= 0
  localparam logic [7:0] F_CSTS_RD    = 8'h07;  // BAR0 access error: CSTS read
  localparam logic [7:0] F_RDY0_TO    = 8'h08;  // CSTS.RDY did not fall
  localparam logic [7:0] F_REG_WR     = 8'h09;  // BAR0 access error: AQA/ASQ/ACQ/CC write
  localparam logic [7:0] F_RDY1_TO    = 8'h0A;  // CSTS.RDY did not rise
  localparam logic [7:0] F_CFS        = 8'h0B;  // CSTS.CFS (fatal) while waiting for RDY
  localparam logic [7:0] F_DOORBELL   = 8'h0C;  // BAR0 access error: doorbell write
  localparam logic [7:0] F_SHN_WR     = 8'h0D;  // BAR0 access error: CC <= SHUTDOWN
  localparam logic [7:0] F_IDENT_QES  = 8'h10;  // SQES/CQES of the controller are not 0x66/0x44
  localparam logic [7:0] F_NS_MS      = 8'h11;  // namespace format has metadata (MS != 0)
  localparam logic [7:0] F_NS_LBADS   = 8'h12;  // LBADS out of the 512 B .. 32 KiB range
  localparam logic [7:0] F_CQE_CID    = 8'h13;  // CQE.CID does not match the submitted command
  localparam logic [7:0] F_CMD_STATUS = 8'h20;  // | CID[4:0]: CQE status != 0
  localparam logic [7:0] F_CMD_TMO    = 8'h40;  // | CID[4:0]: no CQE within ADMIN_TIMEOUT_MS

  // Identify data: beat indices / byte offsets inside the 4 KiB buffer
  localparam logic [7:0] IDC_BEAT_MDTS = 8'd4;    // bytes 64..79   (MDTS = byte 77)
  localparam int unsigned IDC_OFS_MDTS = 13;
  localparam logic [7:0] IDC_BEAT_QES  = 8'd32;   // bytes 512..527 (SQES 512, CQES 513, VWC 525)
  localparam int unsigned IDC_OFS_VWC  = 13;
  localparam logic [7:0] IDN_BEAT_NSZE = 8'd0;    // bytes 0..15    (NSZE = bytes 0..7)
  localparam logic [7:0] IDN_BEAT_FLBAS= 8'd1;    // bytes 16..31   (FLBAS = byte 26)
  localparam int unsigned IDN_OFS_FLBAS= 10;
  localparam logic [7:0] IDN_BEAT_LBAF = 8'd8;    // bytes 128..    (LBAF[n] = 128 + 4n)
  localparam logic [7:0] SQES_EXPECT   = 8'h66;
  localparam logic [7:0] CQES_EXPECT   = 8'h44;

  // Admin command kinds
  typedef enum logic [2:0] {
    K_IDENT_CTRL = 3'd0,
    K_IDENT_NS   = 3'd1,
    K_SET_FEAT   = 3'd2,
    K_CREATE_CQ  = 3'd3,
    K_CREATE_SQ  = 3'd4,
    K_DELETE_SQ  = 3'd5,
    K_DELETE_CQ  = 3'd6
  } cmd_kind_t;

  localparam int unsigned INIT_CMDS = 5;      // K_IDENT_CTRL .. K_CREATE_SQ
  localparam int unsigned SHN_CMDS  = 2;      // K_DELETE_SQ, K_DELETE_CQ

  typedef enum logic [1:0] { POLL_RDY0 = 2'd0, POLL_RDY1 = 2'd1, POLL_SHST = 2'd2 } poll_t;

  // States
  typedef enum logic [5:0] {
    S_IDLE,
    S_MM_REQ, S_MM_RSP,
    S_CAP_LO_DONE, S_CAP_HI_DONE, S_CC_RD_DONE, S_CC_DIS_DONE,
    S_CSTS_RD, S_CSTS_CHK, S_POLL_WAIT,
    S_CLEAR_PULSE, S_CLEAR_GAP, S_CLEAR_WAIT,
    S_REGSEQ, S_REGSEQ_NEXT,
    S_CMD_START, S_AC_WR, S_AC_SQDB_DONE,
    S_AC_POLL_RD, S_AC_POLL_W1, S_AC_POLL_CHK, S_AC_CQDB_DONE,
    S_EX_RD, S_EX_W1, S_EX_CHK,
    S_CMD_FINISH, S_SHN_WR_DONE
  } state_t;

  state_t state;

  // ---------------------------------------------------------------------------
  // Registers
  // ---------------------------------------------------------------------------
  // MMIO transaction registers
  logic [1:0]  mm_op;
  logic [31:0] mm_addr, mm_wdata;
  logic [7:0]  mm_step;       // fail code if the access fails
  state_t      mm_next;       // continuation state
  logic [31:0] mm_rdata;

  // CAP-derived
  logic [15:0] cap_mqes;
  logic [31:0] rdy_limit_ms;

  // polling
  poll_t        poll_mode;
  logic [31:0]  poll_limit;
  logic [31:0]  ms_cnt;       // free-running ms counter (restarted by the FSM)

  // CQ clear handshake
  logic [1:0]  gap_cnt;

  // register write sequence
  logic [2:0]  reg_idx;
  localparam logic [2:0] REGSEQ_LAST = 3'd6;

  // admin queue state
  logic [ASQ_W-1:0] asq_tail;
  logic [ASQ_W-1:0] acq_head;
  logic             acq_phase;
  logic [1:0]       wr_beat;
  logic [15:0]      cid;
  cmd_kind_t        cmd_kind;
  logic [2:0]       cmd_seq;
  logic             in_shutdown;

  // Identify extraction
  logic [1:0]  ex_step;
  logic [7:0]  ex_addr;
  logic [3:0]  ex_fmt;

  // ---------------------------------------------------------------------------
  // Combinational helpers
  // ---------------------------------------------------------------------------
  // The 16 DWs of the SQE for the current command.  DW i at [32*i +: 32].
  function automatic logic [511:0] build_sqe(input cmd_kind_t kind, input logic [15:0] cid_in);
    logic [511:0] s;
    logic [7:0]   opc;
    logic [31:0]  nsid, prp1, cdw10, cdw11;
    opc = 8'h00; nsid = 32'h0; prp1 = 32'h0; cdw10 = 32'h0; cdw11 = 32'h0;
    case (kind)
      K_IDENT_CTRL: begin opc = AOPC_IDENTIFY;     prp1 = IDENT_BASE; cdw10 = 32'h1;           end // CNS=1
      K_IDENT_NS:   begin opc = AOPC_IDENTIFY;     prp1 = IDENT_BASE; cdw10 = 32'h0; nsid = 32'h1; end // CNS=0
      K_SET_FEAT:   begin opc = AOPC_SET_FEATURES; cdw10 = 32'h7;     cdw11 = 32'h0;           end // FID 7, 1 queue each
      K_CREATE_CQ:  begin opc = AOPC_CREATE_CQ;    prp1 = IOCQ_BASE;
                          cdw10 = {16'(IO_QSIZE-1), 16'd1}; cdw11 = 32'h1;                     end // QID1, IEN=0, PC=1
      K_CREATE_SQ:  begin opc = AOPC_CREATE_SQ;    prp1 = IOSQ_BASE;
                          cdw10 = {16'(IO_QSIZE-1), 16'd1}; cdw11 = {16'd1, 16'd1};            end // CQID=1, PC=1
      K_DELETE_SQ:  begin opc = AOPC_DELETE_SQ;    cdw10 = 32'h1;                              end
      K_DELETE_CQ:  begin opc = AOPC_DELETE_CQ;    cdw10 = 32'h1;                              end
      default: ;
    endcase
    s = '0;
    s[32*0  +: 32] = {cid_in, 8'h00, opc};   // CDW0: CID, PSDT/FUSE = 0, OPC
    s[32*1  +: 32] = nsid;                   // NSID
    s[32*6  +: 32] = prp1;                   // PRP1 low   (PRP1 high = DW7 = 0, PRP2 = DW8/9 = 0)
    s[32*10 +: 32] = cdw10;
    s[32*11 +: 32] = cdw11;
    return s;
  endfunction

  logic [511:0] sqe_v;
  assign sqe_v = build_sqe(cmd_kind, cid);

  // sequence tables
  function automatic cmd_kind_t init_cmd(input logic [2:0] idx);
    case (idx)
      3'd0:    return K_IDENT_CTRL;
      3'd1:    return K_IDENT_NS;
      3'd2:    return K_SET_FEAT;
      3'd3:    return K_CREATE_CQ;
      default: return K_CREATE_SQ;
    endcase
  endfunction
  function automatic cmd_kind_t shn_cmd(input logic [2:0] idx);
    return (idx == 3'd0) ? K_DELETE_SQ : K_DELETE_CQ;   // delete the SQ before its CQ
  endfunction

  // AQA / ASQ / ACQ / CC register write list
  function automatic logic [15:0] regseq_off(input logic [2:0] idx);
    case (idx)
      3'd0:    return NVME_AQA;
      3'd1:    return NVME_ASQ_LO;
      3'd2:    return NVME_ASQ_HI;
      3'd3:    return NVME_ACQ_LO;
      3'd4:    return NVME_ACQ_HI;
      3'd5:    return NVME_CC;
      default: return NVME_CC;
    endcase
  endfunction
  function automatic logic [31:0] regseq_val(input logic [2:0] idx);
    case (idx)
      3'd0:    return AQA_VALUE;
      3'd1:    return ASQ_BASE;
      3'd2:    return 32'h0;
      3'd3:    return ACQ_BASE;
      3'd4:    return 32'h0;
      3'd5:    return CC_CONFIG;     // EN=0 with the queue entry sizes / MPS / CSS programmed
      default: return CC_ENABLE;     // EN=1
    endcase
  endfunction

  logic [ASQ_W-1:0] tail_next, head_next;
  assign tail_next = (asq_tail == ASQ_W'(ADMIN_QSIZE-1)) ? '0 : asq_tail + 1'b1;
  assign head_next = (acq_head == ASQ_W'(ADMIN_QSIZE-1)) ? '0 : acq_head + 1'b1;

  // ---------------------------------------------------------------------------
  // Output decoding (purely from state / registers)
  // ---------------------------------------------------------------------------
  assign req_valid   = (state == S_MM_REQ);
  assign req_op      = mm_op;
  assign req_addr    = mm_addr;
  assign req_wdata   = mm_wdata;
  assign req_be      = 4'hF;
  assign clear_cqs   = (state == S_CLEAR_PULSE);

  assign asq_wr_en   = (state == S_AC_WR);
  assign asq_wr_addr = {asq_tail, wr_beat};
  assign asq_wr_data = sqe_v[128*wr_beat +: 128];

  assign acq_rd_en   = (state == S_AC_POLL_RD);
  assign acq_rd_addr = 5'(acq_head);
  assign ident_rd_en = (state == S_EX_RD);
  assign ident_rd_addr = ex_addr;

  assign dbg_asq_tail = 8'(asq_tail);
  assign dbg_acq_head = 8'(acq_head);

  // ---------------------------------------------------------------------------
  // Tasks (called only from the always_ff below)
  // ---------------------------------------------------------------------------
  task automatic mm_rd(input logic [15:0] off, input logic [7:0] step, input state_t nxt);
    mm_op    <= OP_MRD32;
    mm_addr  <= BAR0_ADDR + 32'(off);
    mm_wdata <= 32'h0;
    mm_step  <= step;
    mm_next  <= nxt;
    state    <= S_MM_REQ;
  endtask

  task automatic mm_wr(input logic [31:0] addr, input logic [31:0] data, input logic [7:0] step, input state_t nxt);
    mm_op    <= OP_MWR32;
    mm_addr  <= addr;
    mm_wdata <= data;
    mm_step  <= step;
    mm_next  <= nxt;
    state    <= S_MM_REQ;
  endtask

  // A failure aborts init (init_fail + fail_step).  During shutdown a failure only
  // ends the sequence early, flagged by shutdown_timeout.
  task automatic do_fail(input logic [7:0] code);
    if (in_shutdown) begin
      shutdown_done    <= 1'b1;
      shutdown_timeout <= 1'b1;
    end else begin
      init_fail <= 1'b1;
      fail_step <= code;
    end
    state <= S_IDLE;
  endtask

  // ---------------------------------------------------------------------------
  // Main FSM
  // ---------------------------------------------------------------------------
  logic [14:0] cqe_status;
  logic        cqe_hit;
  assign cqe_hit    = (acq_rd_data[112] == acq_phase);       // DW3[16] = P
  assign cqe_status = acq_rd_data[127:113];                   // DW3[31:17]

  logic [31:0] dw_sel;     // selected LBAF dword
  assign dw_sel = ident_rd_data[32*ex_fmt[1:0] +: 32];

  always_ff @(posedge clk) begin
    if (rst) begin
      state            <= S_IDLE;
      init_done        <= 1'b0;
      init_fail        <= 1'b0;
      fail_step        <= F_NONE;
      cqe_status_last  <= 16'h0;
      shutdown_done    <= 1'b0;
      shutdown_timeout <= 1'b0;
      lbads            <= 4'd0;
      nsze             <= 64'd0;
      mdts             <= 8'd0;
      dstrd            <= 4'd0;
      vwc              <= 1'b0;
      mm_op            <= OP_MRD32;
      mm_addr          <= 32'h0;
      mm_wdata         <= 32'h0;
      mm_step          <= F_NONE;
      mm_next          <= S_IDLE;
      mm_rdata         <= 32'h0;
      cap_mqes         <= 16'd0;
      rdy_limit_ms     <= 32'd1;
      poll_mode        <= POLL_RDY0;
      poll_limit       <= 32'd1;
      ms_cnt           <= 32'd0;
      gap_cnt          <= 2'd0;
      reg_idx          <= 3'd0;
      asq_tail         <= '0;
      acq_head         <= '0;
      acq_phase        <= 1'b1;
      wr_beat          <= 2'd0;
      cid              <= 16'd0;
      cmd_kind         <= K_IDENT_CTRL;
      cmd_seq          <= 3'd0;
      in_shutdown      <= 1'b0;
      ex_step          <= 2'd0;
      ex_addr          <= 8'd0;
      ex_fmt           <= 4'd0;
    end else begin
      // free-running millisecond counter; states restart it with ms_cnt <= 0
      if (tick_ms) ms_cnt <= ms_cnt + 1'b1;

      case (state)
        // ---------------------------------------------------------------
        S_IDLE: begin
          if (start_init) begin
            init_done        <= 1'b0;
            init_fail        <= 1'b0;
            fail_step        <= F_NONE;
            shutdown_done    <= 1'b0;
            shutdown_timeout <= 1'b0;
            in_shutdown      <= 1'b0;
            asq_tail         <= '0;
            acq_head         <= '0;
            acq_phase        <= 1'b1;
            cid              <= 16'd0;
            cmd_seq          <= 3'd0;
            mm_rd(NVME_CAP_LO, F_CAP_LO, S_CAP_LO_DONE);
          end else if (start_shutdown) begin
            shutdown_done    <= 1'b0;
            shutdown_timeout <= 1'b0;
            in_shutdown      <= 1'b1;
            cmd_seq          <= 3'd0;
            if (init_done) begin
              state <= S_CMD_START;
            end else begin               // never initialised: nothing to shut down
              shutdown_done    <= 1'b1;
              shutdown_timeout <= 1'b1;
            end
          end
        end

        // ---------------------------------------------------------------
        // Generic BAR0 access: drive the command port, wait for the response.
        S_MM_REQ: begin
          if (req_ready) begin
            if (rsp_valid) begin                       // response in the same cycle (fast requester)
              if (rsp_status != RSP_SC) begin
                do_fail(mm_step);
              end else begin
                mm_rdata <= rsp_rdata;
                state    <= mm_next;
              end
            end else begin
              state <= S_MM_RSP;
            end
          end
        end
        S_MM_RSP: begin
          if (rsp_valid) begin
            if (rsp_status != RSP_SC) begin
              do_fail(mm_step);
            end else begin
              mm_rdata <= rsp_rdata;
              state    <= mm_next;
            end
          end
        end

        // ---------------------------------------------------------------
        // Step 1: CAP, CC
        S_CAP_LO_DONE: begin     // CAP[31:0]: MQES [15:0], TO [31:24]
          cap_mqes <= mm_rdata[15:0];
          // (TO+1)*500 ms, capped
          rdy_limit_ms <= ((32'(mm_rdata[31:24]) + 32'd1) * 32'd500 > RDY_TIMEOUT_MS_MAX)
                          ? 32'(RDY_TIMEOUT_MS_MAX) : (32'(mm_rdata[31:24]) + 32'd1) * 32'd500;
          mm_rd(NVME_CAP_HI, F_CAP_HI, S_CAP_HI_DONE);
        end
        S_CAP_HI_DONE: begin     // CAP[63:32]: DSTRD [3:0], CSS [12:5] (bit 37 = [5]), MPSMIN [19:16]
          dstrd <= mm_rdata[3:0];
          if (mm_rdata[19:16] != 4'd0 || !mm_rdata[5]) begin
            do_fail(F_CAP_UNSUP);
          end else if (cap_mqes < 16'(IO_QSIZE-1)) begin
            do_fail(F_CAP_MQES);
          end else begin
            mm_rd(NVME_CC, F_CC_RD, S_CC_RD_DONE);
          end
        end
        S_CC_RD_DONE: begin
          if (mm_rdata[0]) begin                        // controller still enabled -> disable it first
            mm_wr(BAR0_ADDR + 32'(NVME_CC), 32'h0, F_CC_DISABLE, S_CC_DIS_DONE);
          end else begin
            state <= S_CLEAR_PULSE;
          end
        end
        S_CC_DIS_DONE: begin
          poll_mode  <= POLL_RDY0;
          poll_limit <= rdy_limit_ms;
          ms_cnt     <= 32'd0;
          state      <= S_CSTS_RD;
        end

        // ---------------------------------------------------------------
        // CSTS polling (RDY==0, RDY==1, SHST==10b)
        S_CSTS_RD: mm_rd(NVME_CSTS, F_CSTS_RD, S_CSTS_CHK);
        S_CSTS_CHK: begin
          case (poll_mode)
            POLL_RDY0: begin
              if (!mm_rdata[0]) state <= S_CLEAR_PULSE;
              else              state <= S_POLL_WAIT;
            end
            POLL_RDY1: begin
              if (mm_rdata[1])      do_fail(F_CFS);
              else if (mm_rdata[0]) begin
                cmd_seq <= 3'd0;
                state   <= S_CMD_START;
              end else              state <= S_POLL_WAIT;
            end
            default: begin   // POLL_SHST
              if (mm_rdata[3:2] == 2'b10) begin
                shutdown_done <= 1'b1;
                state         <= S_IDLE;
              end else begin
                state <= S_POLL_WAIT;
              end
            end
          endcase
        end
        S_POLL_WAIT: begin       // next read at the next 1 ms tick; give up after poll_limit ms
          if (ms_cnt >= poll_limit) begin
            case (poll_mode)
              POLL_RDY0: do_fail(F_RDY0_TO);
              POLL_RDY1: do_fail(F_RDY1_TO);
              default: begin
                shutdown_done    <= 1'b1;
                shutdown_timeout <= 1'b1;
                state            <= S_IDLE;
              end
            endcase
          end else if (tick_ms) begin
            state <= S_CSTS_RD;
          end
        end

        // ---------------------------------------------------------------
        // Step 3: zero the CQs (phase tags must be 0 before CC.EN=1)
        S_CLEAR_PULSE: begin
          gap_cnt <= 2'd2;
          state   <= S_CLEAR_GAP;
        end
        S_CLEAR_GAP: begin       // lets a level-type clear_done fall before it is sampled
          gap_cnt <= gap_cnt - 1'b1;
          if (gap_cnt == 2'd1) state <= S_CLEAR_WAIT;
        end
        S_CLEAR_WAIT: begin
          if (clear_done) begin
            reg_idx <= 3'd0;
            state   <= S_REGSEQ;
          end
        end

        // ---------------------------------------------------------------
        // Step 4/5: AQA, ASQ lo/hi, ACQ lo/hi, CC_CONFIG, CC_ENABLE
        S_REGSEQ: mm_wr(BAR0_ADDR + 32'(regseq_off(reg_idx)), regseq_val(reg_idx), F_REG_WR, S_REGSEQ_NEXT);
        S_REGSEQ_NEXT: begin
          if (reg_idx == REGSEQ_LAST) begin
            poll_mode  <= POLL_RDY1;
            poll_limit <= rdy_limit_ms;
            ms_cnt     <= 32'd0;
            state      <= S_CSTS_RD;
          end else begin
            reg_idx <= reg_idx + 1'b1;
            state   <= S_REGSEQ;
          end
        end

        // ---------------------------------------------------------------
        // admin_cmd sub-FSM
        S_CMD_START: begin
          cmd_kind <= in_shutdown ? shn_cmd(cmd_seq) : init_cmd(cmd_seq);
          wr_beat  <= 2'd0;
          state    <= S_AC_WR;
        end
        S_AC_WR: begin           // asq_wr_en is high in this state; one beat per cycle
          wr_beat <= wr_beat + 1'b1;
          if (wr_beat == 2'd3) begin
            asq_tail <= tail_next;
            // SQ0TDBL = new tail (index of the next free slot)
            mm_wr(doorbell_addr(dstrd, 0, 1'b0), 32'(tail_next), F_DOORBELL, S_AC_SQDB_DONE);
          end
        end
        S_AC_SQDB_DONE: begin
          ms_cnt <= 32'd0;
          state  <= S_AC_POLL_RD;
        end
        S_AC_POLL_RD: state <= S_AC_POLL_W1;          // acq_rd_en high this cycle
        S_AC_POLL_W1: state <= S_AC_POLL_CHK;         // RAM latency
        S_AC_POLL_CHK: begin                          // acq_rd_data valid now
          if (cqe_hit) begin
            cqe_status_last <= {1'b0, cqe_status};
            acq_head        <= head_next;
            if (acq_head == ASQ_W'(ADMIN_QSIZE-1)) acq_phase <= ~acq_phase;
            if (acq_rd_data[111:96] != cid && !in_shutdown) begin
              do_fail(F_CQE_CID);
            end else if (cqe_status != 15'd0 && !in_shutdown) begin
              do_fail(F_CMD_STATUS | {3'b000, cid[4:0]});
            end else begin
              // CQ0HDBL = new head
              mm_wr(doorbell_addr(dstrd, 0, 1'b1), 32'(head_next), F_DOORBELL, S_AC_CQDB_DONE);
            end
          end else if (ms_cnt >= ADMIN_TIMEOUT_MS) begin
            if (in_shutdown) state <= S_CMD_FINISH;   // keep shutting down regardless
            else             do_fail(F_CMD_TMO | {3'b000, cid[4:0]});
          end else begin
            state <= S_AC_POLL_RD;
          end
        end
        S_AC_CQDB_DONE: begin
          if (!in_shutdown && (cmd_kind == K_IDENT_CTRL || cmd_kind == K_IDENT_NS)) begin
            ex_step <= 2'd0;
            ex_addr <= (cmd_kind == K_IDENT_CTRL) ? IDC_BEAT_MDTS : IDN_BEAT_NSZE;
            state   <= S_EX_RD;
          end else begin
            state <= S_CMD_FINISH;
          end
        end

        // Identify field extraction: read a beat, wait 2 cycles, pick fields.
        S_EX_RD: state <= S_EX_W1;                     // ident_rd_en high this cycle
        S_EX_W1: state <= S_EX_CHK;
        S_EX_CHK: begin
          state <= S_EX_RD;                            // default: another beat to read
          if (cmd_kind == K_IDENT_CTRL) begin
            case (ex_step)
              2'd0: begin
                mdts    <= ident_rd_data[8*IDC_OFS_MDTS +: 8];
                ex_addr <= IDC_BEAT_QES;
                ex_step <= 2'd1;
              end
              default: begin
                vwc <= ident_rd_data[8*IDC_OFS_VWC];
                if (ident_rd_data[7:0] != SQES_EXPECT || ident_rd_data[15:8] != CQES_EXPECT) do_fail(F_IDENT_QES);
                else state <= S_CMD_FINISH;
              end
            endcase
          end else begin
            case (ex_step)
              2'd0: begin                              // NSZE = bytes 0..7
                nsze    <= ident_rd_data[63:0];
                ex_addr <= IDN_BEAT_FLBAS;
                ex_step <= 2'd1;
              end
              2'd1: begin                              // FLBAS (byte 26) -> LBAF[n] address
                ex_fmt  <= ident_rd_data[8*IDN_OFS_FLBAS +: 4];
                ex_addr <= IDN_BEAT_LBAF + 8'(ident_rd_data[8*IDN_OFS_FLBAS+2 +: 2]);
                ex_step <= 2'd2;
              end
              default: begin                           // LBAF[n]: MS [15:0], LBADS [23:16]
                lbads <= dw_sel[19:16];
                if (dw_sel[15:0] != 16'd0) do_fail(F_NS_MS);
                else if (dw_sel[23:20] != 4'd0 || dw_sel[19:16] < 4'd9) do_fail(F_NS_LBADS);
                else state <= S_CMD_FINISH;
              end
            endcase
          end
        end

        S_CMD_FINISH: begin
          cid     <= cid + 1'b1;              // CIDs keep incrementing
          cmd_seq <= cmd_seq + 1'b1;
          if (!in_shutdown) begin
            if (cmd_seq == 3'(INIT_CMDS-1)) begin
              init_done <= 1'b1;
              state     <= S_IDLE;
            end else begin
              state <= S_CMD_START;
            end
          end else begin
            if (cmd_seq == 3'(SHN_CMDS-1)) begin
              mm_wr(BAR0_ADDR + 32'(NVME_CC), CC_SHUTDOWN, F_SHN_WR, S_SHN_WR_DONE);
            end else begin
              state <= S_CMD_START;
            end
          end
        end

        S_SHN_WR_DONE: begin
          poll_mode  <= POLL_SHST;
          poll_limit <= 32'(SHUTDOWN_TIMEOUT_MS);
          ms_cnt     <= 32'd0;
          state      <= S_CSTS_RD;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
