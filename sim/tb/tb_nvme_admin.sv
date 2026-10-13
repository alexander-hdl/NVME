// =============================================================================
// tb_nvme_admin.sv - self-checking testbench for rtl/nvme/nvme_admin.sv
//
// The testbench contains a behavioural NVMe controller seen through the
// requester command port (req_* / rsp_*) plus behavioural asq_ram / acq_ram /
// ident_ram with the 2-cycle read latency contract.  The model
//   * implements CAP/CC/CSTS/AQA/ASQ/ACQ and the doorbells,
//   * checks every register write value and order (write log is compared),
//   * fetches each SQE from asq_ram when the SQ doorbell is rung and checks
//     OPC/CID/NSID/PRP1/CDW10/CDW11 (all other DWs must be zero),
//   * fills ident_ram with realistic (and deliberately noisy) Identify data,
//   * posts the CQE into acq_ram as two 8-byte halves with the phase bit last,
//   * checks doorbell values (SQ tail = last+1, CQ head = number of CQEs).
// Scenarios: see the "tests" section at the end (T1..T14).
//   T11 MDTS smaller than SLOT_BYTES -> F_MDTS (0x14); T12 MDTS = 0 / larger / SQES 0x76 / CQES 0x54
//   are accepted; T13 SQES required size 7 or CQES maximum 3 are rejected (nibble-wise check);
//   T14 EN=0 but RDY=1 at power-up: CC.EN must not be set before CSTS.RDY has fallen.
//
// REAL_RAMS=0 (default, test tb_nvme_admin): behavioural RAMs.
// REAL_RAMS=1 (test tb_admin_rams, wrapper in tb_admin_rams.sv): the real hostmem_rams
// (sdp_ram, CQ clear FSM) is used instead; the model then writes CQEs / Identify
// data through the hostmem_writer-style write port (hw_wr_*).
// =============================================================================
`timescale 1ns/1ps
module tb_nvme_admin #(parameter bit REAL_RAMS = 1'b0);
  import nvme_host_pkg::*;

  // --------------------------------------------------------------------------
  // Clock / reset / ms tick
  // --------------------------------------------------------------------------
  logic clk = 1'b0;
  logic rst = 1'b1;
  always #4 clk = ~clk;

  localparam int TICK_PERIOD = 40;       // clocks per "ms"
  int tick_cnt = 0;
  logic tick_ms = 1'b0;
  always_ff @(posedge clk) begin
    if (tick_cnt == TICK_PERIOD-1) begin tick_cnt <= 0; tick_ms <= 1'b1; end
    else                           begin tick_cnt <= tick_cnt + 1; tick_ms <= 1'b0; end
  end

  // --------------------------------------------------------------------------
  // DUT
  // --------------------------------------------------------------------------
  logic        start_init, init_done, init_fail;
  logic [7:0]  fail_step;
  logic [15:0] cqe_status_last;
  logic        start_shutdown, shutdown_done, shutdown_timeout;
  logic [3:0]  lbads, dstrd;
  logic [63:0] nsze;
  logic [7:0]  mdts;
  logic        vwc;
  logic        clear_cqs, clear_done;
  logic        clear_done_m, clear_done_r;      // model / hostmem_rams versions
  assign clear_done = REAL_RAMS ? clear_done_r : clear_done_m;
  logic [7:0]  dbg_asq_tail, dbg_acq_head;
  logic        req_valid, req_ready;
  logic [1:0]  req_op;
  logic [31:0] req_addr, req_wdata;
  logic [3:0]  req_be;
  logic        rsp_valid;
  logic [2:0]  rsp_status;
  logic [31:0] rsp_rdata;
  logic         asq_wr_en;
  logic [6:0]   asq_wr_addr;
  logic [127:0] asq_wr_data;
  logic         acq_rd_en;
  logic [4:0]   acq_rd_addr;
  logic [127:0] acq_rd_data;
  logic         ident_rd_en;
  logic [7:0]   ident_rd_addr;
  logic [127:0] ident_rd_data;

  nvme_admin #(.RDY_TIMEOUT_MS_MAX(40), .ADMIN_TIMEOUT_MS(30), .SHUTDOWN_TIMEOUT_MS(20)) dut (
    .clk(clk), .rst(rst), .tick_ms(tick_ms),
    .start_init(start_init), .init_done(init_done), .init_fail(init_fail), .fail_step(fail_step),
    .cqe_status_last(cqe_status_last),
    .start_shutdown(start_shutdown), .shutdown_done(shutdown_done), .shutdown_timeout(shutdown_timeout),
    .lbads(lbads), .nsze(nsze), .mdts(mdts), .dstrd(dstrd), .vwc(vwc),
    .clear_cqs(clear_cqs), .clear_done(clear_done),
    .dbg_asq_tail(dbg_asq_tail), .dbg_acq_head(dbg_acq_head),
    .req_valid(req_valid), .req_ready(req_ready), .req_op(req_op), .req_addr(req_addr),
    .req_wdata(req_wdata), .req_be(req_be), .rsp_valid(rsp_valid), .rsp_status(rsp_status),
    .rsp_rdata(rsp_rdata),
    .asq_wr_en(asq_wr_en), .asq_wr_addr(asq_wr_addr), .asq_wr_data(asq_wr_data),
    .acq_rd_en(acq_rd_en), .acq_rd_addr(acq_rd_addr), .acq_rd_data(acq_rd_data),
    .ident_rd_en(ident_rd_en), .ident_rd_addr(ident_rd_addr), .ident_rd_data(ident_rd_data));

  // --------------------------------------------------------------------------
  // Error bookkeeping
  // --------------------------------------------------------------------------
  int errors = 0;
  task automatic fail_msg(input string m);
    errors++;
    $display("[%0t] ERROR: %s", $time, m);
  endtask
  task automatic check(input bit cond, input string m);
    if (!cond) fail_msg(m);
  endtask

  // --------------------------------------------------------------------------
  // Model configuration knobs (set by the tests before reset)
  // --------------------------------------------------------------------------
  int          cfg_dstrd     = 0;
  bit          cfg_css       = 1;     // CAP.CSS[37]
  bit          cfg_init_en   = 0;     // CC.EN at power-up
  bit          cfg_rdy_never = 0;     // RDY never rises after EN=1
  bit          cfg_cfs       = 0;     // CSTS.CFS set when EN is written 1
  int          cfg_err_cmd   = -1;    // command index that completes with an error status
  int          cfg_drop_cmd  = -1;    // command index whose CQE is never posted
  bit          cfg_bad_qes   = 0;     // SQES=0x55
  bit          cfg_shn_never = 0;     // SHST never reaches 10b
  int          cfg_flbas     = 0;     // FLBAS (format index)
  int          cfg_ms        = 0;     // metadata size of the selected LBAF
  int          cfg_lbads     = 9;     // LBADS of the selected LBAF
  int          cfg_mdts      = 5;     // MDTS of Identify Controller (5 -> 128 KiB = the default SLOT_BYTES)
  int          cfg_sqes      = 8'h66; // SQES byte (when cfg_bad_qes = 0)
  int          cfg_cqes      = 8'h44; // CQES byte
  bit          cfg_rdy_stale = 0;     // power-up with CC.EN = 0 but CSTS.RDY = 1 (reset still running)

  localparam logic [63:0] NSZE_VALUE = 64'h0000_0000_7470_0000;

  // --------------------------------------------------------------------------
  // RAMs (2-cycle read latency).  asq_mem is always a testbench shadow copy of
  // the DUT's write port (the model reads the SQEs from it).
  // --------------------------------------------------------------------------
  logic [127:0] asq_mem   [0:127];
  logic [127:0] acq_mem   [0:31];
  logic [127:0] ident_mem [0:255];

  always_ff @(posedge clk) if (asq_wr_en) asq_mem[asq_wr_addr] <= asq_wr_data;

  // write port used in REAL_RAMS mode (same shape as hostmem_writer's)
  logic         hw_en_ident = 1'b0, hw_en_acq = 1'b0;
  logic [31:0]  hw_addr = 32'h0;
  logic [127:0] hw_data = '0;
  logic [15:0]  hw_be = 16'h0;

  generate
    if (!REAL_RAMS) begin : g_beh_ram
      logic [127:0] acq_q1;
      logic         acq_v1;
      always_ff @(posedge clk) begin
        acq_q1 <= acq_mem[acq_rd_addr];
        acq_v1 <= acq_rd_en;
        acq_rd_data <= acq_v1 ? acq_q1 : {128{1'bx}};      // X unless a read was issued 2 cycles ago
      end
      logic [127:0] ident_q1;
      logic         ident_v1;
      always_ff @(posedge clk) begin
        ident_q1 <= ident_mem[ident_rd_addr];
        ident_v1 <= ident_rd_en;
        ident_rd_data <= ident_v1 ? ident_q1 : {128{1'bx}};
      end
    end else begin : g_real_ram
      logic [127:0] iocq_rd_data_unused;
      hostmem_rams u_rams (
        .clk(clk), .rst(rst),
        .asq_wr_en(asq_wr_en), .asq_wr_addr(asq_wr_addr), .asq_wr_data(asq_wr_data),
        .iosq_wr_en(1'b0), .iosq_wr_addr(8'h0), .iosq_wr_data(128'h0),
        .hw_wr_en_ident(hw_en_ident), .hw_wr_en_acq(hw_en_acq), .hw_wr_en_iocq(1'b0),
        .hw_wr_beat_addr(hw_addr), .hw_wr_data(hw_data), .hw_wr_be(hw_be),
        .clear_cqs(clear_cqs), .clear_done(clear_done_r),
        .cpl_rd_en(1'b0), .cpl_rd_sel(2'b00), .cpl_rd_addr(32'h0), .cpl_rd_data(), .cpl_rd_en_data(),
        .data_rd_data(128'h0),
        .acq_rd_en(acq_rd_en), .acq_rd_addr(acq_rd_addr), .acq_rd_data(acq_rd_data),
        .iocq_rd_en(1'b0), .iocq_rd_addr(6'h0), .iocq_rd_data(iocq_rd_data_unused),
        .ident_rd_en(ident_rd_en), .ident_rd_addr(ident_rd_addr), .ident_rd_data(ident_rd_data));
    end
  endgenerate

  // --------------------------------------------------------------------------
  // Controller model
  // --------------------------------------------------------------------------
  typedef enum logic [1:0] { M_IDLE, M_WAIT, M_RSP } mstate_t;
  mstate_t mstate;
  int      mdelay;
  logic    ready_gate = 1'b1;         // pseudo-random req_ready gating
  logic [1:0]  m_op;
  logic [31:0] m_addr, m_wdata;

  assign req_ready = (mstate == M_IDLE) && ready_gate;

  // registers
  logic [31:0] cc_r, aqa_r, asq_lo_r, asq_hi_r, acq_lo_r, acq_hi_r;
  logic        rdy_r, cfs_r;
  logic [1:0]  shst_r;
  int          rdy_ticks, shn_ticks;

  // queues / commands
  int          sq_head_m, sq_tail_m, cq_tail_m, cmd_idx, cqe_posted;
  logic        cq_phase_m;
  bit          clear_seen, clear_busy;
  wire         clr_busy_eff = REAL_RAMS ? (clear_seen && !clear_done_r) : clear_busy;
  int          ident_burst = -1;                // REAL_RAMS: next Identify beat to write
  int          clear_cnt;

  // CQE posting
  logic        cqe_pend;
  int          cqe_delay;
  logic [15:0] cqe_cid;
  logic [14:0] cqe_stat;
  logic [31:0] cqe_dw0;
  logic [1:0]  cqe_phase_st;          // 0 idle, 1 lower half written -> waiting, 2 done
  int          cqe_wait;
  int          cqe_slot;
  logic        cqe_ph;

  // write log
  logic [15:0] wlog_off  [$];
  logic [31:0] wlog_data [$];

  // ----- expected SQE content -----
  function automatic logic [511:0] expected_sqe(input int idx);
    logic [511:0] s;
    s = '0;
    case (idx)
      0: begin s[31:0] = {16'd0, 8'h00, 8'h06}; s[191+32:192] = IDENT_BASE; s[32*10+:32] = 32'h1; end
      1: begin s[31:0] = {16'd1, 8'h00, 8'h06}; s[63:32] = 32'h1; s[32*6+:32] = IDENT_BASE; s[32*10+:32] = 32'h0; end
      2: begin s[31:0] = {16'd2, 8'h00, 8'h09}; s[32*10+:32] = 32'h7; end
      3: begin s[31:0] = {16'd3, 8'h00, 8'h05}; s[32*6+:32] = 32'h1000_4000; s[32*10+:32] = 32'h003F_0001; s[32*11+:32] = 32'h1; end
      4: begin s[31:0] = {16'd4, 8'h00, 8'h01}; s[32*6+:32] = 32'h1000_3000; s[32*10+:32] = 32'h003F_0001; s[32*11+:32] = 32'h0001_0001; end
      5: begin s[31:0] = {16'd5, 8'h00, 8'h00}; s[32*10+:32] = 32'h1; end
      6: begin s[31:0] = {16'd6, 8'h00, 8'h04}; s[32*10+:32] = 32'h1; end
      default: ;
    endcase
    return s;
  endfunction

  // ----- Identify data ----------------------------------------------------
  task automatic ident_set_byte(input int b, input logic [7:0] v);
    ident_mem[b/16][8*(b%16) +: 8] = v;
  endtask
  task automatic ident_noise();
    for (int i = 0; i < 256; i++)
      for (int j = 0; j < 16; j++)
        ident_mem[i][8*j +: 8] = 8'((i*16 + j) * 7 + 3);
  endtask
  task automatic ident_fill_ctrl();
    ident_noise();
    ident_set_byte(77, 8'(cfg_mdts));              // MDTS
    ident_set_byte(512, cfg_bad_qes ? 8'h55 : 8'(cfg_sqes));   // SQES
    ident_set_byte(513, 8'(cfg_cqes));             // CQES
    ident_set_byte(525, 8'h01);                    // VWC
    ident_set_byte(78, 8'h00);                     // neighbours of the fields
    ident_set_byte(76, 8'h00);
    ident_set_byte(524, 8'h00);
    ident_set_byte(526, 8'h00);
  endtask
  task automatic ident_fill_ns();
    logic [31:0] dw;
    ident_noise();
    for (int i = 0; i < 8; i++) ident_set_byte(i, NSZE_VALUE[8*i +: 8]);
    ident_set_byte(25, 8'd3);                      // NLBAF
    ident_set_byte(26, 8'(cfg_flbas));             // FLBAS
    // LBAF table: distractors have metadata and a different LBADS, the selected one is as configured
    for (int n = 0; n < 16; n++) begin
      dw = {8'h00, 8'd12 - 8'(n % 3), 16'd8};      // MS=8: invalid unless selected
      if (n == (cfg_flbas & 15)) dw = {8'h00, 8'(cfg_lbads), 16'(cfg_ms)};
      for (int i = 0; i < 4; i++) ident_set_byte(128 + 4*n + i, dw[8*i +: 8]);
    end
  endtask

  // ----- doorbell decode -----------------------------------------------------
  function automatic int db_stride();
    return 4 << cfg_dstrd;
  endfunction

  // ----- main model process ---------------------------------------------------
  initial begin
    for (int i = 0; i < 32; i++)  acq_mem[i]   = {128{1'b1}};   // garbage with P=1 (must be cleared by the DUT)
    for (int i = 0; i < 128; i++) asq_mem[i]   = '0;
    ident_noise();
  end

  always_ff @(posedge clk) begin
    // pseudo random ready gating (never stuck for long)
    ready_gate <= ($urandom % 4) != 0;
  end

  // CAP value helpers: TO=1 (500 ms), MQES=1023; high word built below
  function automatic logic [31:0] cap_lo();
    return {8'd1, 8'd0, 16'd1023};
  endfunction

  // Build the CAP high word explicitly (bit 5 = CSS bit 37; [3:0] = DSTRD)
  logic [31:0] cap_hi_v;
  always_comb begin
    cap_hi_v = 32'h0;
    cap_hi_v[3:0] = 4'(cfg_dstrd);
    cap_hi_v[5]   = cfg_css;
  end

  always_ff @(posedge clk) begin : model
    logic [31:0] off;
    int          q, k;
    logic [511:0] got, exp;
    logic [31:0] rd;

    rsp_valid <= 1'b0;
    rsp_status <= 3'd0;
    rsp_rdata <= 32'h0;
    hw_en_acq <= 1'b0;
    hw_en_ident <= 1'b0;
    // REAL_RAMS: stream the Identify buffer into ident_ram, one beat per cycle
    if (ident_burst >= 0) begin
      hw_en_ident <= 1'b1; hw_addr <= 32'(ident_burst * 16); hw_data <= ident_mem[ident_burst]; hw_be <= 16'hFFFF;
      ident_burst = (ident_burst == 255) ? -1 : ident_burst + 1;
    end

    if (rst) begin
      mstate      <= M_IDLE;
      cc_r        <= cfg_init_en ? 32'h0046_0001 : 32'h0;
      rdy_r       <= cfg_init_en | cfg_rdy_stale;
      cfs_r       <= 1'b0;
      shst_r      <= 2'b00;
      rdy_ticks   <= 0;
      shn_ticks   <= 0;
      aqa_r       <= 0; asq_lo_r <= 0; asq_hi_r <= 0; acq_lo_r <= 0; acq_hi_r <= 0;
      sq_head_m   <= 0; sq_tail_m <= 0; cq_tail_m <= 0; cmd_idx <= 0; cqe_posted <= 0;
      cq_phase_m  <= 1'b1;
      clear_seen  <= 1'b0; clear_busy <= 1'b0; clear_cnt <= 0;
      clear_done_m <= 1'b1;              // level, high while idle
      cqe_pend    <= 1'b0; cqe_phase_st <= 2'd0;
      wlog_off.delete(); wlog_data.delete();
    end else begin
      // ------------------------------------------------------------ RDY/SHST
      if (tick_ms) begin
        if (cc_r[0] != rdy_r && !(cfg_rdy_never && cc_r[0])) begin
          rdy_ticks <= rdy_ticks + 1;
          if (rdy_ticks == 2) begin rdy_r <= cc_r[0]; rdy_ticks <= 0; end
        end else rdy_ticks <= 0;
        if (cc_r[15:14] == 2'b01 && !cfg_shn_never && shst_r != 2'b10) begin
          shn_ticks <= shn_ticks + 1;
          if (shn_ticks == 1) shst_r <= 2'b01;
          if (shn_ticks == 3) shst_r <= 2'b10;
        end
      end

      // ------------------------------------------------------------ CQ clear
      if (clear_cqs) begin
        clear_seen <= 1'b1;
        clear_busy <= 1'b1;
        clear_done_m <= 1'b0;
        clear_cnt  <= 12;
      end else if (clear_busy) begin
        clear_cnt <= clear_cnt - 1;
        if (clear_cnt == 1) begin
          for (int i = 0; i < 32; i++) acq_mem[i] <= '0;
          clear_busy <= 1'b0;
          clear_done_m <= 1'b1;
        end
      end

      // ------------------------------------------------------------ MMIO port
      case (mstate)
        M_IDLE: begin
          if (req_valid && req_ready) begin
            check(req_be == 4'hF, "req_be must be 4'hF");
            check(req_op == 2'd2 || req_op == 2'd3, "req_op must be MRD32/MWR32");
            check(req_addr[1:0] == 2'b00, "req_addr must be DW aligned");
            m_op <= req_op; m_addr <= req_addr; m_wdata <= req_wdata;
            mdelay <= 1 + ($urandom % 4);
            mstate <= M_WAIT;
          end
        end
        M_WAIT: begin
          if (mdelay > 1) mdelay <= mdelay - 1;
          else begin
            off = m_addr - BAR0_ADDR;
            check(m_addr[31:16] == BAR0_ADDR[31:16], "BAR0 access outside BAR0");
            rd = 32'h0;
            if (m_op == 2'd2) begin
              // ---------------- register read ----------------
              case (off[15:0])
                NVME_CAP_LO: rd = cap_lo();
                NVME_CAP_HI: rd = cap_hi_v;
                NVME_VS:     rd = 32'h0001_0400;
                NVME_CC:     rd = cc_r;
                NVME_CSTS:   rd = {26'h0, 2'b00, shst_r, cfs_r, rdy_r};
                default:     fail_msg($sformatf("read of unexpected BAR0 offset 0x%0h", off));
              endcase
              rsp_rdata <= rd;
            end else begin
              // ---------------- register / doorbell write ----------------
              wlog_off.push_back(off[15:0]);
              wlog_data.push_back(m_wdata);
              case (off[15:0])
                NVME_CC: begin
                  // all queue registers must have been programmed before EN goes 1
                  if (m_wdata[0] && !cc_r[0]) begin
                    check(aqa_r == AQA_VALUE, "AQA not programmed before CC.EN");
                    check(asq_lo_r == ASQ_BASE && acq_lo_r == ACQ_BASE, "ASQ/ACQ not programmed before CC.EN");
                    check(clear_seen && !clr_busy_eff, "CQs not cleared before CC.EN");
                    check(m_wdata == CC_ENABLE, "CC enable value");
                    check(!rdy_r, "CC.EN set to 1 while CSTS.RDY is still 1");
                    if (cfg_cfs) cfs_r <= 1'b1;
                  end
                  if (m_wdata[0] == 1'b0 && cc_r[0]) begin end
                  cc_r <= m_wdata;
                end
                NVME_AQA:    begin check(!cc_r[0], "AQA written while EN=1"); aqa_r <= m_wdata; end
                NVME_ASQ_LO: begin check(!cc_r[0], "ASQ written while EN=1"); asq_lo_r <= m_wdata; check(clear_seen && !clr_busy_eff, "queue regs before CQ clear done"); end
                NVME_ASQ_HI: begin check(asq_lo_r == ASQ_BASE, "ASQ_HI must follow ASQ_LO"); asq_hi_r <= m_wdata; end
                NVME_ACQ_LO: begin check(!cc_r[0], "ACQ written while EN=1"); acq_lo_r <= m_wdata; end
                NVME_ACQ_HI: begin check(acq_lo_r == ACQ_BASE, "ACQ_HI must follow ACQ_LO"); acq_hi_r <= m_wdata; end
                default: begin
                  if (off[15:0] >= NVME_DBL_BASE) begin
                    q = (off[15:0] - NVME_DBL_BASE);
                    check((q % db_stride()) == 0, "doorbell offset not stride aligned");
                    q = q / db_stride();
                    check((q / 2) == 0, "doorbell for non-admin queue");
                    check(m_wdata[31:16] == 16'h0, "doorbell upper bits must be 0");
                    if ((q % 2) == 0) begin
                      // ------------- SQ0 tail doorbell -------------
                      check(m_wdata[15:0] == 16'((sq_tail_m + 1) % 32), "SQ0TDBL value");
                      sq_tail_m <= (sq_tail_m + 1) % 32;
                      // fetch + check the SQE
                      for (int b = 0; b < 4; b++) got[128*b +: 128] = asq_mem[sq_head_m*4 + b];
                      exp = expected_sqe(cmd_idx);
                      check(got == exp, $sformatf("SQE %0d content mismatch\n got=%h\n exp=%h", cmd_idx, got, exp));
                      check(cc_r[0] && rdy_r, "doorbell rung before controller ready");
                      check(got[31:16] == 16'(cmd_idx), "CID != command index");
                      sq_head_m <= (sq_head_m + 1) % 32;
                      // Identify data lands in ident_ram before the CQE
                      if (cmd_idx == 0) ident_fill_ctrl();
                      if (cmd_idx == 1) ident_fill_ns();
                      if (REAL_RAMS && cmd_idx < 2) ident_burst = 0;
                      cqe_cid   <= got[31:16];
                      cqe_stat  <= (cmd_idx == cfg_err_cmd) ? 15'h0002 : 15'h0;
                      cqe_dw0   <= 32'h0;
                      cqe_pend  <= (cmd_idx != cfg_drop_cmd);
                      cqe_delay <= (REAL_RAMS ? 300 : 6) + ($urandom % 10);   // REAL_RAMS: let the Identify burst finish first
                      cmd_idx   <= cmd_idx + 1;
                    end else begin
                      // ------------- CQ0 head doorbell -------------
                      check(m_wdata[15:0] == 16'(cq_tail_m), $sformatf("CQ0HDBL value %0d expected %0d", m_wdata[15:0], cq_tail_m));
                    end
                  end else begin
                    fail_msg($sformatf("write to unexpected BAR0 offset 0x%0h", off));
                  end
                end
              endcase
            end
            mstate    <= M_RSP;
            rsp_valid <= 1'b1;
          end
        end
        M_RSP: begin
          mstate <= M_IDLE;
        end
        default: mstate <= M_IDLE;
      endcase

      // ------------------------------------------------------------ CQE posting
      // lower 8 bytes (DW0, DW1) first, then DW2 and DW3 (with the phase bit) - P is written last
      case (cqe_phase_st)
        2'd0: if (cqe_pend) begin
          if (cqe_delay > 0) cqe_delay <= cqe_delay - 1;
          else begin
            if (!REAL_RAMS) acq_mem[cq_tail_m][63:0] <= {32'h0, cqe_dw0};
            else begin hw_en_acq <= 1'b1; hw_addr <= 32'(cq_tail_m * 16); hw_data <= {64'h0, 32'h0, cqe_dw0}; hw_be <= 16'h00FF; end
            cqe_slot     <= cq_tail_m;
            cqe_ph       <= cq_phase_m;
            cqe_wait     <= 3;
            cqe_phase_st <= 2'd1;
          end
        end
        2'd1: begin
          if (cqe_wait > 0) cqe_wait <= cqe_wait - 1;
          else begin
            if (!REAL_RAMS) acq_mem[cqe_slot][127:64] <= {cqe_stat, cqe_ph, cqe_cid, 16'h0, 16'(sq_head_m)};
            else begin hw_en_acq <= 1'b1; hw_addr <= 32'(cqe_slot * 16); hw_data <= {cqe_stat, cqe_ph, cqe_cid, 16'h0, 16'(sq_head_m), 64'h0}; hw_be <= 16'hFF00; end
            // DW2 = {SQID=0, SQHD}; DW3 = {status, P, CID}
            cqe_phase_st <= 2'd2;
          end
        end
        2'd2: begin
          cqe_pend     <= 1'b0;
          cqe_phase_st <= 2'd0;
          cqe_posted   <= cqe_posted + 1;
          if (cq_tail_m == 31) begin cq_tail_m <= 0; cq_phase_m <= ~cq_phase_m; end
          else                  cq_tail_m <= cq_tail_m + 1;
        end
        default: cqe_phase_st <= 2'd0;
      endcase
    end
  end

  // --------------------------------------------------------------------------
  // Helpers used by the tests
  // --------------------------------------------------------------------------
  task automatic do_reset();
    start_init     = 1'b0;
    start_shutdown = 1'b0;
    rst = 1'b1;
    repeat (6) @(posedge clk);
    rst = 1'b0;
    repeat (3) @(posedge clk);
  endtask

  task automatic pulse_start_init();
    @(posedge clk); start_init <= 1'b1;
    @(posedge clk); start_init <= 1'b0;
  endtask
  task automatic pulse_start_shutdown();
    @(posedge clk); start_shutdown <= 1'b1;
    @(posedge clk); start_shutdown <= 1'b0;
  endtask

  task automatic wait_init(input int max_clk);
    int n = 0;
    while (!init_done && !init_fail && n < max_clk) begin @(posedge clk); n++; end
    check(n < max_clk, "watchdog: init neither done nor failed");
  endtask
  task automatic wait_shutdown(input int max_clk);
    int n = 0;
    while (!shutdown_done && n < max_clk) begin @(posedge clk); n++; end
    check(n < max_clk, "watchdog: shutdown_done never came");
  endtask

  // expected write log for a successful init
  int lidx;
  int base;
  task automatic exp_w(input logic [15:0] off, input logic [31:0] d);
    if (lidx >= wlog_off.size()) fail_msg($sformatf("write log too short at %0d", lidx));
    else begin
      check(wlog_off[lidx]  == off, $sformatf("log[%0d] offset 0x%0h expected 0x%0h", lidx, wlog_off[lidx], off));
      check(wlog_data[lidx] == d,   $sformatf("log[%0d] data 0x%0h expected 0x%0h", lidx, wlog_data[lidx], d));
    end
    lidx++;
  endtask
  task automatic check_init_log(input bit initial_en);
    int sqdb = 16'h1000;
    int cqdb = 16'h1000 + (4 << cfg_dstrd);
    lidx = 0;
    if (initial_en) exp_w(NVME_CC, 32'h0);
    exp_w(NVME_AQA,    AQA_VALUE);
    exp_w(NVME_ASQ_LO, ASQ_BASE);
    exp_w(NVME_ASQ_HI, 32'h0);
    exp_w(NVME_ACQ_LO, ACQ_BASE);
    exp_w(NVME_ACQ_HI, 32'h0);
    exp_w(NVME_CC,     CC_CONFIG);
    exp_w(NVME_CC,     CC_ENABLE);
    for (int i = 0; i < 5; i++) begin
      exp_w(16'(sqdb), 32'(i + 1));
      exp_w(16'(cqdb), 32'(i + 1));
    end
    check(wlog_off.size() == lidx, $sformatf("write log has %0d entries, expected %0d", wlog_off.size(), lidx));
  endtask

  task automatic check_shutdown_log(input int base);
    int sqdb = 16'h1000;
    int cqdb = 16'h1000 + (4 << cfg_dstrd);
    check(wlog_off.size() == base + 5, $sformatf("shutdown log size %0d expected %0d", wlog_off.size(), base + 5));
    if (wlog_off.size() >= base + 5) begin
      check(wlog_off[base+0] == 16'(sqdb) && wlog_data[base+0] == 32'd6, "shutdown: SQ db (delete SQ)");
      check(wlog_off[base+1] == 16'(cqdb) && wlog_data[base+1] == 32'd6, "shutdown: CQ db (delete SQ)");
      check(wlog_off[base+2] == 16'(sqdb) && wlog_data[base+2] == 32'd7, "shutdown: SQ db (delete CQ)");
      check(wlog_off[base+3] == 16'(cqdb) && wlog_data[base+3] == 32'd7, "shutdown: CQ db (delete CQ)");
      check(wlog_off[base+4] == NVME_CC   && wlog_data[base+4] == CC_SHUTDOWN, "shutdown: CC write");
    end
  endtask

  task automatic reset_knobs();
    cfg_dstrd = 0; cfg_css = 1; cfg_init_en = 0; cfg_rdy_never = 0; cfg_cfs = 0;
    cfg_err_cmd = -1; cfg_drop_cmd = -1; cfg_bad_qes = 0; cfg_shn_never = 0;
    cfg_flbas = 0; cfg_ms = 0; cfg_lbads = 9;
    cfg_mdts = 5; cfg_sqes = 8'h66; cfg_cqes = 8'h44; cfg_rdy_stale = 0;
  endtask

  task automatic expect_fail(input string name, input logic [7:0] code);
    check(init_fail, {name, ": init_fail not set"});
    check(!init_done, {name, ": init_done set"});
    check(fail_step == code, $sformatf("%s: fail_step 0x%0h expected 0x%0h", name, fail_step, code));
    $display("  %s -> init_fail, fail_step=0x%0h, cqe_status_last=0x%0h (as expected)", name, fail_step, cqe_status_last);
  endtask

  // --------------------------------------------------------------------------
  // Tests
  // --------------------------------------------------------------------------
  initial begin
    start_init = 0; start_shutdown = 0;
    reset_knobs();

    // ---- T1: controller found enabled (EN=1), DSTRD=0, FLBAS=0, full init + shutdown ----
    $display("T1: full init (EN initially 1, DSTRD 0) and shutdown");
    cfg_init_en = 1;
    do_reset();
    pulse_start_init();
    wait_init(200000);
    check(init_done && !init_fail, "T1: init_done");
    check(lbads == 4'd9, "T1: lbads");
    check(nsze == NSZE_VALUE, "T1: nsze");
    check(mdts == 8'd5, "T1: mdts");
    check(dstrd == 4'd0, "T1: dstrd");
    check(vwc == 1'b1, "T1: vwc");
    check(cqe_status_last == 16'h0, "T1: cqe_status_last");
    check(cmd_idx == 5, "T1: five admin commands");
    check_init_log(1'b1);
    begin
      base = wlog_off.size();
      pulse_start_shutdown();
      wait_shutdown(400000);
      check(shutdown_done && !shutdown_timeout, "T1: shutdown ok");
      check(cmd_idx == 7, "T1: two delete commands");
      check(shst_r == 2'b10, "T1: SHST");
      check_shutdown_log(base);
    end

    // ---- T2: EN initially 0, DSTRD=2 (doorbell stride 16 B), FLBAS=3 with 4 KiB LBAs ----
    $display("T2: DSTRD=2, FLBAS=3 (LBADS=12)");
    reset_knobs(); cfg_dstrd = 2; cfg_flbas = 3; cfg_lbads = 12;
    do_reset();
    pulse_start_init();
    wait_init(200000);
    check(init_done && !init_fail, "T2: init_done");
    check(lbads == 4'd12, "T2: lbads");
    check(dstrd == 4'd2, "T2: dstrd");
    check_init_log(1'b0);
    begin
      base = wlog_off.size();
      pulse_start_shutdown();
      wait_shutdown(400000);
      check(shutdown_done && !shutdown_timeout, "T2: shutdown ok");
      check_shutdown_log(base);
    end

    // ---- T3: Set Features completes with an error status ----
    $display("T3: CQE error status on command 2");
    reset_knobs(); cfg_err_cmd = 2;
    do_reset();
    pulse_start_init();
    wait_init(200000);
    expect_fail("T3", 8'h22);
    check(cqe_status_last == 16'h0002, "T3: cqe_status_last");

    // ---- T4: CAP.CSS bit 37 clear ----
    $display("T4: CSS not supported");
    reset_knobs(); cfg_css = 0;
    do_reset();
    pulse_start_init();
    wait_init(200000);
    expect_fail("T4", 8'h03);

    // ---- T5: RDY never rises ----
    $display("T5: CSTS.RDY timeout");
    reset_knobs(); cfg_rdy_never = 1;
    do_reset();
    pulse_start_init();
    wait_init(2000000);
    expect_fail("T5", 8'h0A);

    // ---- T6: CFS set ----
    $display("T6: CSTS.CFS");
    reset_knobs(); cfg_cfs = 1; cfg_rdy_never = 1;
    do_reset();
    pulse_start_init();
    wait_init(2000000);
    expect_fail("T6", 8'h0B);

    // ---- T7: SQES wrong ----
    $display("T7: bad SQES");
    reset_knobs(); cfg_bad_qes = 1;
    do_reset();
    pulse_start_init();
    wait_init(200000);
    expect_fail("T7", 8'h10);

    // ---- T8: shutdown timeout ----
    $display("T8: SHST never reaches 10b");
    reset_knobs(); cfg_shn_never = 1;
    do_reset();
    pulse_start_init();
    wait_init(200000);
    check(init_done, "T8: init_done");
    pulse_start_shutdown();
    wait_shutdown(2000000);
    check(shutdown_done && shutdown_timeout, "T8: shutdown_timeout flag");
    $display("  T8 -> shutdown_done with shutdown_timeout=%0b (as expected)", shutdown_timeout);

    // ---- T9: namespace format with metadata ----
    $display("T9: MS != 0");
    reset_knobs(); cfg_ms = 8;
    do_reset();
    pulse_start_init();
    wait_init(200000);
    expect_fail("T9", 8'h11);

    // ---- T10: CQE of command 1 never arrives ----
    $display("T10: admin command timeout");
    reset_knobs(); cfg_drop_cmd = 1;
    do_reset();
    pulse_start_init();
    wait_init(2000000);
    expect_fail("T10", 8'h41);

    // ---- T11: MDTS too small for SLOT_BYTES (128 KiB): MDTS=4 -> 64 KiB ----
    $display("T11: MDTS smaller than SLOT_BYTES");
    reset_knobs(); cfg_mdts = 4;
    do_reset();
    pulse_start_init();
    wait_init(200000);
    expect_fail("T11", 8'h14);
    check(mdts == 8'd4, "T11: mdts output");

    // ---- T12: MDTS 0 (no limit), MDTS 6, SQES 0x76 / CQES 0x54 (wider maximum) are fine ----
    $display("T12: MDTS 0 / 6, SQES 0x76, CQES 0x54 accepted");
    reset_knobs(); cfg_mdts = 0; cfg_sqes = 8'h76; cfg_cqes = 8'h54;
    do_reset();
    pulse_start_init();
    wait_init(200000);
    check(init_done && !init_fail, "T12a: init_done (MDTS 0, SQES 0x76, CQES 0x54)");
    reset_knobs(); cfg_mdts = 6;
    do_reset();
    pulse_start_init();
    wait_init(200000);
    check(init_done && !init_fail, "T12b: init_done (MDTS 6)");

    // ---- T13: SQES required size 7 / CQES maximum 3 are rejected ----
    $display("T13: SQES/CQES nibble check");
    reset_knobs(); cfg_sqes = 8'h67;
    do_reset();
    pulse_start_init();
    wait_init(200000);
    expect_fail("T13a (SQES 0x67)", 8'h10);
    reset_knobs(); cfg_cqes = 8'h33;
    do_reset();
    pulse_start_init();
    wait_init(200000);
    expect_fail("T13b (CQES 0x33)", 8'h10);
    reset_knobs(); cfg_sqes = 8'h56;              // maximum 5 < 6
    do_reset();
    pulse_start_init();
    wait_init(200000);
    expect_fail("T13c (SQES 0x56)", 8'h10);

    // ---- T14: EN = 0 but RDY = 1 at power-up ----
    $display("T14: wait for CSTS.RDY==0 even though CC.EN is already 0");
    reset_knobs(); cfg_rdy_stale = 1;
    do_reset();
    pulse_start_init();
    wait_init(200000);
    check(init_done && !init_fail, "T14: init_done");
    check_init_log(1'b0);

    // ---- T15: init_done / init_fail drop in the cycle start_init is presented ----
    $display("T15: done/fail flags drop with the start edge");
    reset_knobs(); cfg_mdts = 4;
    do_reset();
    pulse_start_init();
    wait_init(200000);
    check(init_fail, "T15: precondition init_fail");
    begin
      bit saw_fail_in_start_cycle;
      saw_fail_in_start_cycle = 1'b0;
      @(posedge clk);
      start_init <= 1'b1;
      @(negedge clk);                       // start_init is visible in this cycle, the FSM has not taken it yet
      saw_fail_in_start_cycle = init_fail || init_done;
      @(posedge clk);
      start_init <= 1'b0;
      check(!saw_fail_in_start_cycle, "T15: init_fail/init_done still high in the cycle of start_init");
    end
    repeat (5) @(posedge clk);

    // ---- done ----
    repeat (20) @(posedge clk);
    if (errors == 0) $display("TEST PASSED");
    else $fatal(1, "TEST FAILED: %0d errors", errors);
    $finish;
  end

  // global watchdog
  initial begin
    #200ms;
    $fatal(1, "TEST FAILED: global watchdog");
  end
endmodule
