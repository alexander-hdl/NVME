// =============================================================================
// nvme_ssd_model.sv — behavioural NVMe SSD (PCIe Endpoint) behind pcie7x_core_model
//
// SIMULATION ONLY.  Talks whole TLPs with the core model through
// pcie_sim_pkg::mb_down (host -> SSD) and pcie_sim_pkg::mb_up (SSD -> host).
// Reference: research/02_nvme_minimal_host_brief.md (NVMe 1.4 behaviour),
//            research/03_pcie_7x_root_port_brief.md §3-§5 (TLP / completion rules).
//
// Features
// --------
//  PCIe config space (Type 0):  VID/DID 0x144D/0xA808, class 0x010802, Command
//    (MSE/BME/INTxDis writable), 64-bit non-prefetchable BAR0 of 16 KiB at 0x10/0x14
//    (all-ones sizing reads back 0xFFFFC004 / 0xFFFFFFFF), capability list
//    0x34 -> 0x40 PM -> 0x50 MSI -> 0x80 PCIe (DevCap MPSS=2 (512 B), ETFS=1;
//    DevCtl reset 0x2810; LinkCap/LinkCtl/LinkSta Gen2 x4).
//    The first `crs_count` config requests after PERST# release get CRS.
//    The bus/device number is captured on the first (non-CRS) CfgWr0 and used as
//    Requester ID / Completer ID afterwards (0x0000 before that).
//  BAR0 NVMe registers: CAP (MQES=1023, CQR=1, TO=1, DSTRD=0, CSS bit 37,
//    MPSMIN=MPSMAX=0), VS=0x00010400, INTMS/INTMC, CC, CSTS (RDY follows EN after
//    rdy_delay_cycles; CFS on bad configuration; SHST 01 -> 10 after
//    shn_delay_cycles), AQA, ASQ, ACQ (64-bit, two 32-bit halves), doorbells at
//    0x1000 + 4*(2*qid + is_cq).  BAR access only when Command.MSE=1.
//  Admin queue engine (SQ0/CQ0): Identify (CNS 1 controller, CNS 0 namespace),
//    Set Features (FID 7 Number of Queues -> DW0 = 0, FID 6 VWC), Create I/O CQ /
//    SQ (QID 1 only, PC=1 required), Delete I/O SQ / CQ, everything else ->
//    Invalid Opcode.
//  I/O engine (QID 1): fetches up to `sq_fetch_max` SQEs per MRd, runs up to
//    `concurrent_cmds` commands concurrently (their data fetches interleave),
//    Write (opc 01h) with PRP1/PRP2/PRP list (incl. list chaining), Flush (00h).
//    Data is fetched with MRds of random size in [mrd_size_min, MRRS] (DW aligned,
//    never crossing 4 KiB) with up to `max_outstanding` tags in flight; the data
//    lands in `lba_mem` (associative array of 512-byte blocks indexed by LBA).
//  CQE posting with phase tag (P=1 on the first pass, flips on wrap), SQHD, CQ-full
//    back-pressure (waits for the head doorbell), optional split (two 8-byte MWr,
//    P in the second) and coalescing (two CQEs in one 32-byte MWr).
//
// Knobs (module variables; set hierarchically from the tb at time 0)
// -----------------------------------------------------------------
//   crs_count            config requests answered with CRS after PERST# release     [2]
//   rdy_delay_cycles     CC.EN change -> CSTS.RDY change                             [200]
//   shn_delay_cycles     CC.SHN write -> CSTS.SHST = 10b                             [500]
//   mmio_lat_cycles      latency of config / BAR completions                         [8]
//   fetch_delay_cycles   doorbell seen -> SQE fetch MRd issued                       [20]
//   cmd_latency_min/max  random processing latency per command (cycles)              [20/200]
//   concurrent_cmds      I/O commands processed concurrently                         [4]
//   sq_fetch_max         max SQEs fetched by one MRd                                 [4]
//   max_outstanding      max outstanding MRd tags (all purposes)                     [16]
//   mrd_size_min         minimum random data MRd size in bytes (max = MRRS)          [64]
//   mrd_gap_max          random 0..N idle cycles between data MRds of one command    [0]
//   prp_chunk_pct        % of PRP-list fetches done as several random 8..64 B MRds   [50]
//                        (the rest: one MRd per <= MRRS piece)
//   cqe_split_pct        % of single CQEs written as two 8-byte MWr (P last)         [0]
//   cqe_coalesce_pct     % chance to write two ready CQEs in one 32-byte MWr         [0]
//   inject_status_at_cmd 0-based index of the I/O Write that gets an error status    [-1 = off]
//   inject_status_value  status written for it: {DNR,M,CRD,SCT,SC}  [DNR=1, SCT=2, SC=80h write fault]
//                        (the data is still stored in lba_mem so data checks still pass)
//   rcb_bytes            Read Completion Boundary of the Root Port                   [64]
//   exp_cpl_id           expected Completer ID in CplDs from the Root Port           [16'h0000]
//   cpl_timeout_cycles   MRd completion timeout                                      [100000]
//   allow_overwrite      0 = writing an LBA twice is an $error                       [0]
//   verbose              1 = print every TLP / command                               [0]
//
// Checks ($error, counted in err_count)
// -------------------------------------
//  Config:  C-1 Length!=1 / LastBE!=0 / TC,Attr!=0;  C-2 Type-0 request to dev/fn != 0
//  BAR:     B-1 BAR access with MSE=0 or outside BAR0;  B-2 MMIO read > 2 DW
//           B-3 doorbell write with BE != F
//  NVMe:    N-1 AQA/ASQ/ACQ written while CC.EN=1;  N-2 CC.MPS/CSS/AMS changed while EN=1
//           N-3 IOSQES/IOCQES changed while I/O queues exist;  N-4 EN 0->1 while RDY=1
//           N-5 bad configuration at EN (AQA sizes 0, ASQ/ACQ not 4 KiB aligned, MPS!=0,
//               CSS!=0) -> CSTS.CFS=1;  N-6 doorbell while not ready / for a queue that
//               does not exist;  N-7 doorbell value >= queue size;  N-8 SQ tail doorbell
//               that overfills the queue or moves backwards;  N-9 CQ head doorbell
//               beyond the posted entries;  N-10 CC.SHN = 11b
//  DMA:     D-1 any upstream request while Command.BME=0 ("host triggered DMA
//               without BME");  D-2 completion with status != SC (e.g. UR because
//               the Root Port BME is 0);  D-3 Cpl without data for an MRd
//           D-4 unexpected tag / ReqID not echoed / CplID != exp_cpl_id / TC,Attr not
//               echoed / BCM set;  D-5 CplD Length > MPS;  D-6 Byte Count != bytes
//               remaining;  D-7 Lower Address wrong (also catches out-of-order
//               completions for one tag);  D-8 split at an address that is not
//               RCB aligned;  D-9 more data than requested / zero-length CplD
//           D-10 poisoned CplD;  D-11 completion timeout
//  Data:    W-1 LBA written twice (unless allow_overwrite)
// Command errors (bad NSID, LBA range, PRP offsets, queue ids, ...) are reported to the
// host with the proper NVMe status code and also printed as $warning.
//
// Scoreboard API for testbenches
// ------------------------------
//   lba_mem[lba]                      512-byte block written by Write commands
//   check_range(start_lba, nblocks, ref_mem)  compares nblocks blocks with ref_mem
//                                     (byte array, ref_mem[0] = first byte of start_lba);
//                                     returns the number of mismatching bytes
//   ev_log[$] / ev_find(...)          ordered log of config / MMIO / admin / I/O events
//   n_io_writes, n_flush, n_cqe, data_cpl_first/last_*    statistics
// =============================================================================
`timescale 1ns/1ps

module nvme_ssd_model
  import pcie_sim_pkg::*;
#(
  parameter longint unsigned NSZE    = 64'd16777216,   // namespace size in 512-B LBAs (8 GiB)
  parameter logic [31:0]     VID_DID = 32'hA808_144D   // {DID, VID}
) (
  input logic clk,       // use the core model's user_clk_out (all delays are in these cycles)
  input logic perst_n    // PCIe PERST# (active low); the model starts when it is released
);

  // ---------------------------------------------------------------------------
  // Knobs
  // ---------------------------------------------------------------------------
  int          crs_count            = 2;
  int          rdy_delay_cycles     = 200;
  int          shn_delay_cycles     = 500;
  int          mmio_lat_cycles      = 8;
  int          fetch_delay_cycles   = 20;
  int          cmd_latency_min      = 20;
  int          cmd_latency_max      = 200;
  int          concurrent_cmds      = 4;
  int          sq_fetch_max         = 4;
  int          max_outstanding      = 16;
  int          mrd_size_min         = 64;
  int          mrd_gap_max          = 0;
  int          prp_chunk_pct        = 50;
  int          cqe_split_pct        = 0;
  int          cqe_coalesce_pct     = 0;
  int          inject_status_at_cmd = -1;
  logic [14:0] inject_status_value  = {1'b1, 1'b0, 2'b00, 3'd2, 8'h80};
  int          rcb_bytes            = 64;
  logic [15:0] exp_cpl_id           = 16'h0000;
  int          cpl_timeout_cycles   = 100000;
  bit          allow_overwrite      = 0;
  bit          verbose              = 0;

  // Identify data constants
  localparam byte unsigned MDTS  = 8'd5;
  localparam int unsigned  RTD3E = 32'd1_000_000;   // us
  localparam int unsigned  MQES  = 1023;

  // ---------------------------------------------------------------------------
  // Statistics / scoreboard
  // ---------------------------------------------------------------------------
  int     err_count    = 0;
  longint cyc          = 0;
  int     n_io_writes  = 0;
  int     n_flush      = 0;
  int     n_cqe        = 0;
  int     n_mrd        = 0;
  int     n_mwr        = 0;
  int     n_cpld       = 0;
  int     n_ur_cpl     = 0;
  longint data_cpl_first_cycle = -1, data_cpl_first_beats = 0;
  longint data_cpl_last_cycle  = -1, data_cpl_last_beats  = 0;
  longint data_bytes   = 0;

  typedef byte unsigned blk_t [512];
  blk_t   lba_mem [longint unsigned];

  ev_t    ev_log[$];

  always @(posedge clk) cyc <= cyc + 1;

  function automatic void err(input string s);
    err_count++;
    $error("[nvme_ssd_model @%0t] %s", $time, s);
  endfunction

  function automatic void log_ev(input ev_kind_e k, input logic [63:0] a, input logic [63:0] b);
    ev_t e;
    e.kind = k; e.cycle = cyc; e.a = a; e.b = b;
    ev_log.push_back(e);
  endfunction

  // Find the first event of kind k at index >= from whose (a & amask) == (va & amask). -1 if none.
  function automatic int ev_find(input ev_kind_e k, input logic [63:0] va, input logic [63:0] amask, input int from = 0);
    for (int i = from; i < ev_log.size(); i++)
      if (ev_log[i].kind == k && ((ev_log[i].a & amask) == (va & amask))) return i;
    return -1;
  endfunction

  // Scoreboard: compare nblocks LBAs starting at start_lba with ref_mem
  function automatic int check_range(input longint unsigned start_lba, input int unsigned nblocks,
                                     ref byte unsigned ref_mem[]);
    int errs, shown;
    errs = 0; shown = 0;
    if (ref_mem.size() < nblocks * 512) begin
      $error("[nvme_ssd_model] check_range: ref_mem has %0d bytes, need %0d", ref_mem.size(), nblocks * 512);
      return nblocks * 512;
    end
    for (int unsigned b = 0; b < nblocks; b++) begin
      longint unsigned lba;
      lba = start_lba + b;
      if (!lba_mem.exists(lba)) begin
        errs += 512;
        if (shown++ < 10) $display("[nvme_ssd_model] check_range: LBA %0d was never written", lba);
        continue;
      end
      for (int i = 0; i < 512; i++)
        if (lba_mem[lba][i] !== ref_mem[b*512 + i]) begin
          errs++;
          if (shown++ < 10)
            $display("[nvme_ssd_model] check_range: LBA %0d byte %0d = %02h, expected %02h",
                     lba, i, lba_mem[lba][i], ref_mem[b*512 + i]);
        end
    end
    return errs;
  endfunction

  // ---------------------------------------------------------------------------
  // PCIe configuration space
  // ---------------------------------------------------------------------------
  logic [31:0] cfg    [0:1023];
  logic [31:0] cfg_wm [0:1023];
  int          crs_left;
  bit          bdf_captured;
  logic [15:0] bdf;                 // our Requester/Completer ID

  localparam int CFG_DEVCTL_DW = 32'h88 >> 2;

  function automatic void cfg_init();
    for (int i = 0; i < 1024; i++) begin cfg[i] = '0; cfg_wm[i] = '0; end
    cfg[0]  = VID_DID;
    cfg[1]  = 32'h0010_0000;  cfg_wm[1]  = 32'h0000_0546;   // Status.CapList; Command MSE/BME/PERR/SERR/INTxDis
    cfg[2]  = 32'h0108_0200;                                // class 01/08/02, rev 0
    cfg[3]  = 32'h0000_0000;  cfg_wm[3]  = 32'h0000_00FF;   // header type 0
    cfg[4]  = 32'h0000_0004;  cfg_wm[4]  = 32'hFFFF_C000;   // BAR0: 64-bit, non-prefetchable, 16 KiB
    cfg[5]  = 32'h0000_0000;  cfg_wm[5]  = 32'hFFFF_FFFF;   // BAR1: upper 32 bits of BAR0
    cfg[11] = 32'hA801_144D;                                // subsystem IDs
    cfg[13] = 32'h0000_0040;                                // capabilities pointer
    cfg[15] = 32'h0000_0100;  cfg_wm[15] = 32'h0000_00FF;   // INTA
    cfg[16] = 32'h0003_5001;                                // 0x40 PM cap, next 0x50
    cfg[17] = 32'h0000_0008;  cfg_wm[17] = 32'h0000_0003;   // PMCSR (No_Soft_Reset)
    cfg[20] = 32'h0080_8005;  cfg_wm[20] = 32'h0071_0000;   // 0x50 MSI cap (64-bit), next 0x80
    cfg_wm[21] = 32'hFFFF_FFFC; cfg_wm[22] = 32'hFFFF_FFFF; cfg_wm[23] = 32'h0000_FFFF;
    cfg[32] = 32'h0002_0010;                                // 0x80 PCIe cap v2, endpoint, next 0
    cfg[33] = 32'h0000_8022;                                // 0x84 DevCap: MPSS=2 (512 B), ETFS=1, RBER
    cfg[34] = 32'h0000_2810;  cfg_wm[34] = 32'h0000_7FFF;   // 0x88 DevCtl (MRRS 512, NS, RO, MPS 128)
    cfg[35] = 32'h0000_0C42;                                // 0x8C LinkCap: 5 GT/s, x4, ASPM L0s/L1
    cfg[36] = 32'h1042_0000;  cfg_wm[36] = 32'h0000_03CB;   // 0x90 LinkCtl / LinkSta (5 GT/s x4, SCC)
    cfg[41] = 32'h0000_001F;                                // 0xA4 DevCap2
    cfg_wm[42] = 32'h0000_041F;                             // 0xA8 DevCtl2
    cfg[44] = 32'h0000_0002;  cfg_wm[44] = 32'h0000_007F;   // 0xB0 LinkCtl2 (target 5 GT/s)
  endfunction

  function automatic bit mse();          return cfg[1][1]; endfunction
  function automatic bit bme();          return cfg[1][2]; endfunction
  function automatic int unsigned mps_bytes();  return 128 << cfg[CFG_DEVCTL_DW][7:5];   endfunction
  function automatic int unsigned mrrs_bytes(); return 128 << cfg[CFG_DEVCTL_DW][14:12]; endfunction
  function automatic bit ete();          return cfg[CFG_DEVCTL_DW][8]; endfunction
  function automatic logic [63:0] bar0_base(); return {cfg[5], cfg[4] & 32'hFFFF_C000}; endfunction

  // ---------------------------------------------------------------------------
  // NVMe controller registers
  // ---------------------------------------------------------------------------
  localparam logic [63:0] CAP      = {32'h0000_0020, 32'h0101_0000 | MQES};
  localparam logic [31:0] VS       = 32'h0001_0400;
  localparam logic [31:0] CC_WMASK = 32'h00FF_FFF1;

  logic [31:0] cc, aqa, intms;
  logic [63:0] asq, acq;
  bit          csts_rdy, csts_cfs;
  logic [1:0]  csts_shst;
  int          en_cnt, en_evt, shn_cnt;
  bit          nq_set;               // Set Features Number of Queues done
  int          active_cmds;          // I/O commands in flight
  int          io_write_idx;         // running index of I/O Write commands

  nvme_sq sqs[2];
  nvme_cq cqs[2];

  function automatic void ctrl_reset();
    cc = 0; aqa = 0; intms = 0; asq = 0; acq = 0;
    csts_rdy = 0; csts_cfs = 0; csts_shst = 0; en_cnt = 0; en_evt = 0; shn_cnt = 0; nq_set = 0;
    for (int i = 0; i < 2; i++) begin sqs[i] = new(); cqs[i] = new(); end
  endfunction

  function automatic logic [31:0] csts();
    return {28'h0, csts_shst, csts_cfs, csts_rdy};
  endfunction

  function automatic logic [14:0] mk_status(input int sct, input int sc, input bit dnr = 1'b1);
    return {dnr, 1'b0, 2'b00, 3'(sct), 8'(sc)};
  endfunction

  function automatic logic [31:0] merge_be(input logic [31:0] old, input logic [31:0] v, input logic [3:0] be);
    logic [31:0] r;
    r = old;
    for (int b = 0; b < 4; b++) if (be[b]) r[8*b +: 8] = v[8*b +: 8];
    return r;
  endfunction

  // ===========================================================================
  // TLP helpers
  // ===========================================================================
  function automatic tlp_t mk_cpl(input bit with_data, input logic [15:0] cplid, input logic [2:0] st,
                                  input logic [11:0] bc, input logic [31:0] rq_h0, input logic [31:0] rq_h1,
                                  input logic [6:0] la, input byte unsigned d[$]);
    tlp_t         c;
    logic [127:0] hc;
    hc = nvme_host_pkg::hdr_cpl(with_data, cplid, st, bc, rq_h1[31:16], rq_h1[15:8], la, 10'(d.size() / 4));
    c.hdr[0] = hc[31:0]; c.hdr[1] = hc[63:32]; c.hdr[2] = hc[95:64]; c.hdr[3] = '0;
    c.hdr[0][22:20] = rq_h0[22:20];   // TC
    c.hdr[0][13:12] = rq_h0[13:12];   // Attr
    c.data.delete();
    foreach (d[i]) c.data.push_back(d[i]);   // NOTE: element-wise on purpose: xsim 2022.2 loses
                                             // `c.data = d` (queue argument -> struct member)
    c.poison = 0;
    return c;
  endfunction

  // Delayed upstream send (completions of config / MMIO requests).  Implemented with a
  // time-ordered queue serviced by delay_thread (keeps the order of the responses).
  typedef struct { tlp_t t; longint due; } dly_t;
  dly_t dly_q[$];

  function automatic void send_later(input tlp_t t, input int lat);
    dly_t d;
    d.t = t; d.due = cyc + lat;
    dly_q.push_back(d);
  endfunction

  task automatic delay_thread();
    forever begin
      @(posedge clk);
      while (dly_q.size() != 0 && dly_q[0].due <= cyc) begin
        dly_t d;
        d = dly_q.pop_front();
        mb_up.put(d.t);
      end
    end
  endtask

  // Byte Count of a memory read request per PCIe Table 2-27 (FBE/LBE)
  function automatic logic [11:0] calc_bc(input int unsigned n, input logic [3:0] fbe, input logic [3:0] lbe);
    int first, last;
    if (n == 1) return nvme_host_pkg::bc_from_fbe(fbe);
    first = fbe[0] ? 0 : fbe[1] ? 1 : fbe[2] ? 2 : 3;
    last  = lbe[3] ? 3 : lbe[2] ? 2 : lbe[1] ? 1 : 0;
    return 12'(n * 4 - first - (3 - last));
  endfunction

  // ===========================================================================
  // Configuration requests
  // ===========================================================================
  task automatic handle_cfg(input tlp_t t);
    logic [31:0] h0, h1, h2, v;
    logic [9:0]  dw;
    logic [15:0] tgt;
    logic [2:0]  st;
    bit          wr;
    byte unsigned d[$];
    tlp_t        c;
    h0 = t.hdr[0]; h1 = t.hdr[1]; h2 = t.hdr[2];
    wr  = h0[30];
    dw  = h2[11:2];
    tgt = h2[31:16];
    st  = nvme_host_pkg::CPL_SC;
    if (h0[9:0] != 10'd1 || h1[7:4] != 4'b0 || h0[22:20] != 3'b0 || h0[13:12] != 2'b0)
      err($sformatf("C-1: malformed config request H0=%08h H1=%08h", h0, h1));
    if (tgt[7:0] != 8'h00) begin
      err($sformatf("C-2: Type-0 config request to dev %0d fn %0d (only 0/0 exists) -> UR", tgt[7:3], tgt[2:0]));
      st = nvme_host_pkg::CPL_UR;
    end else if (crs_left > 0) begin
      crs_left--;
      st = nvme_host_pkg::CPL_CRS;
      log_ev(EV_CFG_CRS, {dw, 2'b00}, 0);
      if (verbose) $display("[ssd_model] Cfg%s 0x%03h -> CRS", wr ? "Wr" : "Rd", {dw, 2'b00});
    end else if (wr) begin
      v = le32(t.data, 0);
      for (int b = 0; b < 4; b++)
        if (h1[b]) for (int i = 8*b; i < 8*b + 8; i++) if (cfg_wm[dw][i]) cfg[dw][i] = v[i];
      if (!bdf_captured) begin
        bdf_captured = 1;
        bdf = {tgt[15:3], 3'b000};
        $display("[nvme_ssd_model @%0t] captured bus/device number: BDF = %04h", $time, bdf);
      end
      log_ev(EV_CFGWR, {dw, 2'b00}, {28'h0, h1[3:0], cfg[dw]});
      if (verbose) $display("[ssd_model] CfgWr 0x%03h be %b data %08h -> %08h", {dw, 2'b00}, h1[3:0], v, cfg[dw]);
    end else begin
      v = cfg[dw];
      d.delete(); d.push_back(v[7:0]); d.push_back(v[15:8]); d.push_back(v[23:16]); d.push_back(v[31:24]);
      log_ev(EV_CFGRD, {dw, 2'b00}, v);
      if (verbose) $display("[ssd_model] CfgRd 0x%03h -> %08h", {dw, 2'b00}, v);
    end
    // Config completions: Completer ID = the addressed BDF, Byte Count = 4, Lower Address = 0
    if (!wr && st == nvme_host_pkg::CPL_SC) c = mk_cpl(1, tgt, st, 12'd4, h0, h1, 7'd0, d);
    else begin d.delete(); c = mk_cpl(0, tgt, st, 12'd4, h0, h1, 7'd0, d); end
    send_later(c, mmio_lat_cycles);
  endtask

  // ===========================================================================
  // BAR0 (NVMe registers, doorbells)
  // ===========================================================================
  function automatic logic [31:0] reg_read(input int unsigned off);
    case (off)
      32'h00: return CAP[31:0];
      32'h04: return CAP[63:32];
      32'h08: return VS;
      32'h0C, 32'h10: return intms;
      32'h14: return cc;
      32'h1C: return csts();
      32'h24: return aqa;
      32'h28: return asq[31:0];
      32'h2C: return asq[63:32];
      32'h30: return acq[31:0];
      32'h34: return acq[63:32];
      default: return 32'h0;
    endcase
  endfunction

  function automatic void cc_write(input logic [31:0] nv);
    logic [31:0] old;
    old = cc;
    nv  = nv & CC_WMASK;
    if (old[0] && nv[0] && (old[13:4] != nv[13:4]))
      err($sformatf("N-2: CC.MPS/CSS/AMS changed while EN=1 (%08h -> %08h)", old, nv));
    if ((sqs[1].exists || cqs[1].exists) && (old[23:16] != nv[23:16]))
      err("N-3: CC.IOSQES/IOCQES changed while I/O queues exist");
    if (nv[15:14] == 2'b11) err("N-10: CC.SHN = 11b (reserved)");
    cc = nv;
    if (!old[0] && nv[0]) begin
      if (csts_rdy) err("N-4: CC.EN set 0->1 while CSTS.RDY=1");
      en_evt = 1; en_cnt = rdy_delay_cycles;
    end
    if (old[0] && !nv[0]) begin
      en_evt = 2; en_cnt = rdy_delay_cycles;
    end
    if (nv[15:14] != 2'b00 && old[15:14] == 2'b00) begin
      if (active_cmds != 0) $warning("[nvme_ssd_model] CC.SHN written with %0d I/O commands in flight", active_cmds);
      csts_shst = 2'b01;
      shn_cnt   = shn_delay_cycles;
      log_ev(EV_SHST, 1, 0);
    end
  endfunction

  function automatic void doorbell(input int unsigned off, input logic [31:0] v);
    int unsigned idx, qid, val, sz, adv, avail, newcnt, oldcnt;
    bit          is_cq;
    idx   = (off - 32'h1000) >> 2;      // DSTRD = 0
    qid   = idx >> 1;
    is_cq = idx[0];
    val   = v[15:0];
    if (v[31:16] != 0) $warning("[nvme_ssd_model] doorbell write with bits 31:16 set (%08h)", v);
    if (!csts_rdy) begin err($sformatf("N-6: doorbell %s%0d written while CSTS.RDY=0", is_cq ? "CQ" : "SQ", qid)); return; end
    if (qid > 1) begin err($sformatf("N-6: doorbell for queue %0d which does not exist", qid)); return; end
    if (is_cq) begin
      nvme_cq q; q = cqs[qid];
      if (!q.exists) begin err($sformatf("N-6: CQ%0d head doorbell but CQ%0d does not exist", qid, qid)); return; end
      sz = q.size;
      if (val >= sz) begin err($sformatf("N-7: CQ%0d head doorbell %0d >= size %0d", qid, val, sz)); return; end
      adv   = (val + sz - q.head) % sz;
      avail = (q.tail + sz - q.head) % sz;
      if (adv > avail) begin
        err($sformatf("N-9: CQ%0d head doorbell %0d moves past the posted entries (head %0d, tail %0d)", qid, val, q.head, q.tail));
        return;
      end
      q.head = val;
    end else begin
      nvme_sq q; q = sqs[qid];
      if (!q.exists) begin err($sformatf("N-6: SQ%0d tail doorbell but SQ%0d does not exist", qid, qid)); return; end
      sz = q.size;
      if (val >= sz) begin err($sformatf("N-7: SQ%0d tail doorbell %0d >= size %0d", qid, val, sz)); return; end
      newcnt = (val + sz - q.head) % sz;
      oldcnt = (q.tail + sz - q.head) % sz;
      if (newcnt < oldcnt) begin
        err($sformatf("N-8: SQ%0d tail doorbell %0d overfills the queue / moves backwards (head %0d, old tail %0d)",
                      qid, val, q.head, q.tail));
        return;
      end
      q.tail = val;
    end
  endfunction

  function automatic void reg_write(input int unsigned off, input logic [31:0] v, input logic [3:0] be);
    logic [31:0] m;
    m = merge_be(reg_read(off), v, be);
    log_ev(EV_MMIO_WR, off, m);
    if (verbose) $display("[ssd_model] MMIO WR 0x%04h = %08h (be %b)", off, m, be);
    if (off >= 32'h1000) begin
      if (be != 4'hF) err($sformatf("B-3: doorbell 0x%04h written with BE %b", off, be));
      doorbell(off, m);
      return;
    end
    case (off)
      32'h0C: intms = intms | m;
      32'h10: intms = intms & ~m;
      32'h14: cc_write(m);
      32'h20: $warning("[nvme_ssd_model] NSSR written (%08h) - ignored", m);
      32'h24, 32'h28, 32'h2C, 32'h30, 32'h34: begin
        if (cc[0]) err($sformatf("N-1: admin queue register 0x%02h written while CC.EN=1", off));
        case (off)
          32'h24: aqa = m & 32'h0FFF_0FFF;
          32'h28: asq[31:0]  = m;
          32'h2C: asq[63:32] = m;
          32'h30: acq[31:0]  = m;
          32'h34: acq[63:32] = m;
        endcase
      end
      default: $warning("[nvme_ssd_model] write to read-only/reserved BAR0 offset 0x%04h ignored", off);
    endcase
  endfunction

  task automatic handle_mem(input tlp_t t);
    logic [31:0] h0, h1;
    logic [63:0] addr, base;
    int unsigned n, off;
    bit          rd, hit;
    byte unsigned d[$];
    tlp_t        c;
    h0 = t.hdr[0]; h1 = t.hdr[1];
    rd   = is_mrd(h0);
    addr = req_addr(t);
    n    = (h0[9:0] == 0) ? 1024 : h0[9:0];
    base = bar0_base();
    hit  = (base != 0) && (addr >= base) && (addr < base + 64'd16384);
    if (!hit || !mse()) begin
      err($sformatf("B-1: %s @%h: %s", tlp_name(h0), addr, !mse() ? "Command.MSE=0" : "outside BAR0"));
      if (rd) begin
        c = mk_cpl(0, bdf, nvme_host_pkg::CPL_UR, 12'(n * 4), h0, h1, addr[6:0], d);
        send_later(c, mmio_lat_cycles);
      end
      return;
    end
    off = addr - base;
    if (rd) begin
      if (n > 2) err($sformatf("B-2: MMIO read of %0d DW at 0x%04h", n, off));
      for (int i = 0; i < n; i++) begin
        logic [31:0] v;
        v = reg_read(off + 4*i);
        d.push_back(v[7:0]); d.push_back(v[15:8]); d.push_back(v[23:16]); d.push_back(v[31:24]);
        log_ev(EV_MMIO_RD, off + 4*i, v);
        if (verbose) $display("[ssd_model] MMIO RD 0x%04h -> %08h", off + 4*i, v);
      end
      c = mk_cpl(1, bdf, nvme_host_pkg::CPL_SC, calc_bc(n, h1[3:0], h1[7:4]), h0, h1,
                 nvme_host_pkg::lower_addr_first(addr[31:0], h1[3:0]), d);
      send_later(c, mmio_lat_cycles);
    end else begin
      for (int i = 0; i < n; i++) begin
        logic [3:0] be;
        be = (n == 1) ? h1[3:0] : (i == 0) ? h1[3:0] : (i == n - 1) ? h1[7:4] : 4'hF;
        reg_write(off + 4*i, le32(t.data, 4*i), be);
      end
    end
  endtask

  // ===========================================================================
  // Upstream DMA: MRd with tag management, MWr
  // ===========================================================================
  typedef struct {
    bit          busy;
    logic [63:0] addr;
    int unsigned len;
    int unsigned rcvd;
    dma_ctx      ctx;
    int unsigned off;
    longint      t_issue;
    logic [15:0] reqid;
    bit          is_data;
    bit          to_reported;
  } rd_ent_t;

  rd_ent_t rd_tab [256];
  int      outstanding = 0;

  // Issue one MRd (blocks until a tag is available). len: multiple of 4, <= MRRS, no 4 KiB crossing.
  task automatic issue_mrd(input logic [63:0] addr, input int unsigned len, input dma_ctx ctx,
                           input int unsigned off, input bit is_data);
    int   tag, tspace, start;
    tlp_t t;
    if (len == 0 || len % 4 != 0 || addr[1:0] != 0 || (addr & 64'hFFF) + len > 4096 || len > 4096)
      err($sformatf("MODEL BUG: illegal MRd request @%h len %0d", addr, len));
    tag = -1;
    forever begin
      tspace = ete() ? 256 : 32;
      if (outstanding < max_outstanding) begin
        start = $urandom_range(tspace - 1);
        for (int i = 0; i < tspace; i++)
          if (!rd_tab[(start + i) % tspace].busy) begin tag = (start + i) % tspace; break; end
        if (tag >= 0) break;
      end
      @(posedge clk);
    end
    if (!bme()) err($sformatf("D-1: SSD must issue MRd @%h but its Command.BME=0 (host triggered DMA without BME)", addr));
    rd_tab[tag].busy        = 1;
    rd_tab[tag].addr        = addr;
    rd_tab[tag].len         = len;
    rd_tab[tag].rcvd        = 0;
    rd_tab[tag].ctx         = ctx;
    rd_tab[tag].off         = off;
    rd_tab[tag].t_issue     = cyc;
    rd_tab[tag].reqid       = bdf;
    rd_tab[tag].is_data     = is_data;
    rd_tab[tag].to_reported = 0;
    outstanding++;
    ctx.pending++;
    n_mrd++;
    t.poison = 0;
    t.hdr[3] = '0;
    if (addr[63:32] != 0) begin
      t.hdr[0] = {8'h20, 14'h0, 10'(len / 4)};
      t.hdr[2] = addr[63:32];
      t.hdr[3] = {addr[31:2], 2'b00};
    end else begin
      t.hdr[0] = {8'h00, 14'h0, 10'(len / 4)};
      t.hdr[2] = {addr[31:2], 2'b00};
    end
    t.hdr[1] = {bdf, 8'(tag), (len == 4) ? 4'h0 : 4'hF, 4'hF};
    if (verbose) $display("[ssd_model] MRd @%h len %0d tag %0d", addr, len, tag);
    mb_up.put(t);
  endtask

  // Read [addr, addr+len) as one MRd per piece (pieces <= MRRS and inside one 4 KiB page)
  task automatic issue_read_pieces(input logic [63:0] addr, input int unsigned len, input dma_ctx ctx,
                                   input int unsigned off, input bit random_small);
    int unsigned pos, sz, room;
    pos = 0;
    while (pos < len) begin
      room = 4096 - ((addr + pos) & 64'hFFF);
      if (random_small) sz = 8 * $urandom_range(1, 8);      // 8..64 B, QW multiples
      else              sz = mrrs_bytes();
      if (sz > len - pos) sz = len - pos;
      if (sz > room)      sz = room;
      issue_mrd(addr + pos, sz, ctx, off + pos, 0);
      pos += sz;
    end
  endtask

  task automatic wait_ctx(input dma_ctx ctx);
    while (ctx.pending != 0) @(posedge clk);
  endtask

  // Posted write, split at MPS and 4 KiB boundaries
  task automatic dma_write(input logic [63:0] addr, input byte unsigned d[$]);
    int unsigned pos, chunk, room;
    if (!bme()) err($sformatf("D-1: SSD must issue MWr @%h but its Command.BME=0 (host triggered DMA without BME)", addr));
    pos = 0;
    while (pos < d.size()) begin
      tlp_t        t;
      logic [63:0] a;
      a     = addr + pos;
      room  = 4096 - (a & 64'hFFF);
      chunk = d.size() - pos;
      if (chunk > mps_bytes()) chunk = mps_bytes();
      if (chunk > room)        chunk = room;
      t.poison = 0;
      t.hdr[3] = '0;
      if (a[63:32] != 0) begin
        t.hdr[0] = {8'h60, 14'h0, 10'(chunk / 4)};
        t.hdr[2] = a[63:32];
        t.hdr[3] = {a[31:2], 2'b00};
      end else begin
        t.hdr[0] = {8'h40, 14'h0, 10'(chunk / 4)};
        t.hdr[2] = {a[31:2], 2'b00};
      end
      t.hdr[1] = {bdf, 8'h00, (chunk == 4) ? 4'h0 : 4'hF, 4'hF};
      for (int i = 0; i < chunk; i++) t.data.push_back(d[pos + i]);
      n_mwr++;
      if (verbose) $display("[ssd_model] MWr @%h len %0d", a, chunk);
      mb_up.put(t);
      pos += chunk;
    end
  endtask

  // Completion handling with all requester-side checks
  function automatic void finish_tag(input int tag);
    rd_tab[tag].busy = 0;
    rd_tab[tag].ctx.pending--;
    outstanding--;
  endfunction

  function automatic void handle_cpl(input tlp_t t);
    logic [31:0] h0, h1, h2;
    int          tag;
    int unsigned nbytes, remaining, bc;
    logic [63:0] cur;
    h0 = t.hdr[0]; h1 = t.hdr[1]; h2 = t.hdr[2];
    tag = h2[15:8];
    if (!rd_tab[tag].busy) begin
      err($sformatf("D-4: unexpected completion (no outstanding MRd with tag %0d): %s", tag, tlp_str(t)));
      return;
    end
    if (h1[31:16] !== exp_cpl_id)
      err($sformatf("D-4: tag %0d: Completer ID %04h, expected %04h", tag, h1[31:16], exp_cpl_id));
    if (h2[31:16] !== rd_tab[tag].reqid)
      err($sformatf("D-4: tag %0d: Requester ID %04h not echoed (sent %04h)", tag, h2[31:16], rd_tab[tag].reqid));
    if (h0[22:20] != 0 || h0[13:12] != 0)
      err($sformatf("D-4: tag %0d: TC/Attr not echoed (H0=%08h)", tag, h0));
    if (h1[12]) err($sformatf("D-4: tag %0d: BCM set", tag));
    if (h1[15:13] != nvme_host_pkg::CPL_SC) begin
      n_ur_cpl++;
      err($sformatf("D-2: completion status %0d (%s) for MRd @%h len %0d tag %0d%s", h1[15:13],
                    (h1[15:13] == nvme_host_pkg::CPL_UR) ? "UR" : (h1[15:13] == nvme_host_pkg::CPL_CA) ? "CA" : "?",
                    rd_tab[tag].addr, rd_tab[tag].len, tag,
                    (h1[15:13] == nvme_host_pkg::CPL_UR) ? " - Root Port BME=0 or address not decoded by the host?" : ""));
      rd_tab[tag].ctx.errors++;
      finish_tag(tag);
      return;
    end
    if (!h0[30]) begin
      err($sformatf("D-3: Cpl without data (status SC) for MRd tag %0d", tag));
      rd_tab[tag].ctx.errors++;
      finish_tag(tag);
      return;
    end
    if (t.poison || h0[14]) begin
      err($sformatf("D-10: poisoned CplD for tag %0d", tag));
      rd_tab[tag].ctx.errors++;
    end
    n_cpld++;
    nbytes    = t.data.size();
    remaining = rd_tab[tag].len - rd_tab[tag].rcvd;
    bc        = (h1[11:0] == 0) ? 4096 : h1[11:0];
    cur       = rd_tab[tag].addr + rd_tab[tag].rcvd;
    if (nbytes > mps_bytes())
      err($sformatf("D-5: tag %0d: CplD payload %0d B > MPS %0d B", tag, nbytes, mps_bytes()));
    if (bc != remaining)
      err($sformatf("D-6: tag %0d (MRd @%h len %0d): Byte Count %0d, expected %0d remaining bytes",
                    tag, rd_tab[tag].addr, rd_tab[tag].len, bc, remaining));
    if (h2[6:0] != cur[6:0])
      err($sformatf("D-7: tag %0d (MRd @%h len %0d): Lower Address %02h, expected %02h (next byte @%h)",
                    tag, rd_tab[tag].addr, rd_tab[tag].len, h2[6:0], cur[6:0], cur));
    if (nbytes == 0 || nbytes > remaining) begin
      err($sformatf("D-9: tag %0d: CplD with %0d B, %0d B remaining", tag, nbytes, remaining));
      if (nbytes > remaining) nbytes = remaining;
    end
    if (nbytes < remaining && ((cur + nbytes) % rcb_bytes) != 0)
      err($sformatf("D-8: tag %0d: completion split at %h which is not %0d-B (RCB) aligned", tag, cur + nbytes, rcb_bytes));
    for (int i = 0; i < nbytes; i++) rd_tab[tag].ctx.mem[rd_tab[tag].off + rd_tab[tag].rcvd + i] = t.data[i];
    rd_tab[tag].rcvd += nbytes;
    if (rd_tab[tag].is_data) begin
      data_bytes += nbytes;
      if (data_cpl_first_cycle < 0) begin data_cpl_first_cycle = t.acc_cycle; data_cpl_first_beats = t.acc_beats; end
      data_cpl_last_cycle = t.acc_cycle; data_cpl_last_beats = t.acc_beats;
    end
    if (rd_tab[tag].rcvd >= rd_tab[tag].len) finish_tag(tag);
  endfunction

  // Completion timeout watchdog
  task automatic watchdog();
    forever begin
      repeat (256) @(posedge clk);
      for (int i = 0; i < 256; i++)
        if (rd_tab[i].busy && !rd_tab[i].to_reported && (cyc - rd_tab[i].t_issue) > cpl_timeout_cycles) begin
          rd_tab[i].to_reported = 1;
          err($sformatf("D-11: completion timeout: MRd @%h len %0d tag %0d (received %0d B)",
                        rd_tab[i].addr, rd_tab[i].len, i, rd_tab[i].rcvd));
        end
    end
  endtask

  // ===========================================================================
  // Downstream TLP dispatcher
  // ===========================================================================
  task automatic dispatcher();
    tlp_t t;
    forever begin
      mb_down.get(t);
      if (verbose) $display("[ssd_model @%0t] rx %s", $time, tlp_str(t));
      if (is_cfg0(t.hdr[0]))                         handle_cfg(t);
      else if (is_mrd(t.hdr[0]) || is_mwr(t.hdr[0])) handle_mem(t);
      else if (is_cpl(t.hdr[0]))                     handle_cpl(t);
      else if (t.hdr[0][28:27] == 2'b10)             $display("[nvme_ssd_model] message received: %s (ignored)", tlp_str(t));
      else if (t.hdr[0][28:24] == 5'b00101) begin
        tlp_t        c;
        byte unsigned d[$];
        $warning("[nvme_ssd_model] Type-1 config request received -> UR");
        c = mk_cpl(0, bdf, nvme_host_pkg::CPL_UR, 12'd4, t.hdr[0], t.hdr[1], 7'd0, d);
        send_later(c, mmio_lat_cycles);
      end
      else err($sformatf("unexpected TLP from the host: %s", tlp_str(t)));
    end
  endtask

  // ===========================================================================
  // Controller state machine (EN / RDY / SHN)
  // ===========================================================================
  task automatic ctrl_thread();
    forever begin
      @(posedge clk);
      if (en_cnt > 0) begin
        en_cnt--;
        if (en_cnt == 0) begin
          if (en_evt == 1) begin
            bit bad; bad = 0;
            if (aqa[11:0] == 0 || aqa[27:16] == 0) begin err($sformatf("N-5: AQA=%08h has a zero (=1 entry) size at CC.EN", aqa)); bad = 1; end
            if (asq[11:0] != 0 || acq[11:0] != 0)   begin err($sformatf("N-5: ASQ=%h / ACQ=%h not 4 KiB aligned", asq, acq)); bad = 1; end
            if (cc[10:7] != 0) begin err($sformatf("N-5: CC.MPS=%0d but CAP.MPSMAX=0", cc[10:7])); bad = 1; end
            if (cc[6:4]  != 0) begin err($sformatf("N-5: CC.CSS=%0d unsupported", cc[6:4])); bad = 1; end
            if (bad) csts_cfs = 1;
            else begin
              sqs[0] = new(); cqs[0] = new();
              sqs[0].exists = 1; sqs[0].base = {asq[63:12], 12'h0}; sqs[0].size = aqa[11:0] + 1;  sqs[0].cqid = 0;
              cqs[0].exists = 1; cqs[0].base = {acq[63:12], 12'h0}; cqs[0].size = aqa[27:16] + 1; cqs[0].phase = 1;
              csts_rdy = 1;
              log_ev(EV_RDY, 1, 0);
              $display("[nvme_ssd_model @%0t] CSTS.RDY=1 (ASQ %h x%0d, ACQ %h x%0d)", $time,
                       sqs[0].base, sqs[0].size, cqs[0].base, cqs[0].size);
            end
          end else if (en_evt == 2) begin
            csts_rdy = 0; csts_shst = 2'b00; nq_set = 0;
            for (int i = 0; i < 2; i++) begin sqs[i] = new(); cqs[i] = new(); end
            log_ev(EV_RDY, 0, 0);
            $display("[nvme_ssd_model @%0t] CSTS.RDY=0 (controller reset)", $time);
          end
          en_evt = 0;
        end
      end
      if (shn_cnt > 0) begin
        shn_cnt--;
        if (shn_cnt == 0) begin
          csts_shst = 2'b10;
          log_ev(EV_SHST, 2, 0);
          $display("[nvme_ssd_model @%0t] CSTS.SHST=10b (shutdown complete)", $time);
        end
      end
    end
  endtask

  // ===========================================================================
  // CQE posting
  // ===========================================================================
  function automatic void post_cqe(input int qid, input logic [15:0] sqid, input logic [15:0] cid,
                                   input logic [14:0] status, input logic [31:0] dw0);
    cqe_t c;
    c.dw0 = dw0; c.sqid = sqid; c.cid = cid; c.status = status;
    if (!cqs[qid].exists) begin
      err($sformatf("MODEL: CQE for CQ%0d which does not exist (cid %0d) dropped", qid, cid));
      return;
    end
    cqs[qid].pend.push_back(c);
  endfunction

  function automatic void cqe_bytes(input cqe_t c, input bit phase, input logic [15:0] sqhd, ref byte unsigned q[$]);
    logic [31:0] dw[4];
    dw[0] = c.dw0; dw[1] = 32'h0; dw[2] = {c.sqid, sqhd}; dw[3] = {c.status, phase, c.cid};
    for (int i = 0; i < 4; i++) begin
      q.push_back(dw[i][7:0]); q.push_back(dw[i][15:8]); q.push_back(dw[i][23:16]); q.push_back(dw[i][31:24]);
    end
  endfunction

  task automatic cq_poster(input int qid);
    int full_cycles;
    full_cycles = 0;
    forever begin
      nvme_cq      q;
      int unsigned used, free, n, t0;
      byte unsigned b[$];
      logic [63:0] a;
      @(posedge clk);
      q = cqs[qid];
      if (!q.exists || q.pend.size() == 0) continue;
      used = (q.tail + q.size - q.head) % q.size;
      free = q.size - 1 - used;
      if (free == 0) begin
        full_cycles++;
        if (full_cycles == 100000) $warning("[nvme_ssd_model] CQ%0d full for 100000 cycles (host not consuming?)", qid);
        continue;
      end
      full_cycles = 0;
      n = 1;
      if (q.pend.size() >= 2 && free >= 2 && q.tail + 1 < q.size && $urandom_range(99) < cqe_coalesce_pct) n = 2;
      t0 = q.tail;
      for (int k = 0; k < n; k++) begin
        cqe_t        c;
        logic [15:0] sqhd;
        c    = q.pend.pop_front();
        sqhd = (c.sqid < 2 && sqs[c.sqid].exists) ? 16'(sqs[c.sqid].head) : 16'h0;
        cqe_bytes(c, q.phase, sqhd, b);
        log_ev(EV_CQE, qid, {32'h0, c.status, q.phase, c.cid});
        n_cqe++;
        if (verbose) $display("[ssd_model] CQE CQ%0d[%0d] cid %0d status %h P=%0d", qid, q.tail, c.cid, c.status, q.phase);
        q.tail++;
        if (q.tail == q.size) begin q.tail = 0; q.phase = ~q.phase; end
      end
      a = q.base + 16 * t0;
      if (n == 1 && $urandom_range(99) < cqe_split_pct) begin
        byte unsigned lo[$], hi[$];
        for (int i = 0; i < 8; i++) begin lo.push_back(b[i]); hi.push_back(b[8 + i]); end
        dma_write(a, lo);
        repeat ($urandom_range(0, 3)) @(posedge clk);
        dma_write(a + 8, hi);                 // DW2/DW3 incl. the phase tag last
      end else begin
        dma_write(a, b);
      end
    end
  endtask

  // ===========================================================================
  // Identify data
  // ===========================================================================
  function automatic void put_le(ref byte unsigned b[$], input int off, input logic [63:0] v, input int n);
    for (int i = 0; i < n; i++) b[off + i] = v[8*i +: 8];
  endfunction

  function automatic void put_str(ref byte unsigned b[$], input int off, input string s, input int n);
    for (int i = 0; i < n; i++) b[off + i] = (i < s.len()) ? s[i] : 8'h20;
  endfunction

  function automatic void build_id_ctrl(ref byte unsigned b[$]);
    b.delete();
    repeat (4096) b.push_back(8'h00);
    put_le(b, 0, VID_DID[15:0], 2);                 // VID
    put_le(b, 2, VID_DID[15:0], 2);                 // SSVID
    put_str(b, 4,  "SIMSSD0000000001", 20);         // SN
    put_str(b, 24, "NVMe behavioural SSD model", 40); // MN
    put_str(b, 64, "SIM1.0", 8);                    // FR
    b[72] = 8'd2;                                   // RAB
    b[77] = MDTS;                                   // MDTS = 5 -> 128 KiB
    put_le(b, 78, 1, 2);                            // CNTLID
    put_le(b, 80, 32'h0001_0400, 4);                // VER
    put_le(b, 88, RTD3E, 4);                        // RTD3E (us)
    b[258] = 8'd3;  b[259] = 8'd3;                  // ACL, AERL
    b[260] = 8'h02;                                 // FRMW
    put_le(b, 266, 343, 2);                         // WCTEMP (K)
    put_le(b, 268, 353, 2);                         // CCTEMP (K)
    b[512] = 8'h66;                                 // SQES
    b[513] = 8'h44;                                 // CQES
    put_le(b, 516, 1, 4);                           // NN
    b[525] = 8'h01;                                 // VWC present
    put_str(b, 768, "nqn.2026-10.local.sim:nvme-ssd-model", 256);  // SUBNQN
    put_le(b, 2048, 2500, 2);                       // PSD0 MP (0.01 W units)
  endfunction

  function automatic void build_id_ns(ref byte unsigned b[$]);
    b.delete();
    repeat (4096) b.push_back(8'h00);
    put_le(b, 0,  NSZE, 8);                         // NSZE
    put_le(b, 8,  NSZE, 8);                         // NCAP
    put_le(b, 16, NSZE, 8);                         // NUSE
    b[25] = 8'd1;                                   // NLBAF (0's based: 2 formats)
    b[26] = 8'd0;                                   // FLBAS: format 0
    put_le(b, 128, 32'h0009_0000, 4);               // LBAF0: LBADS=9 (512 B), MS=0
    put_le(b, 132, 32'h000C_0000, 4);               // LBAF1: LBADS=12 (4 KiB), MS=0
  endfunction

  // Write a 4 KiB buffer to PRP1 / PRP2 (no PRP list; Identify)
  task automatic write_prp_4k(input logic [63:0] prp1, input logic [63:0] prp2, input byte unsigned d[$]);
    int unsigned  first;
    byte unsigned p[$];
    first = 4096 - (prp1 & 64'hFFF);
    if (first >= 4096) dma_write(prp1, d);
    else begin
      for (int i = 0; i < first; i++) p.push_back(d[i]);
      dma_write(prp1, p);
      p.delete();
      for (int i = first; i < 4096; i++) p.push_back(d[i]);
      dma_write(prp2, p);
    end
  endtask

  // ===========================================================================
  // Admin queue engine
  // ===========================================================================
  task automatic process_admin(input logic [31:0] s[16]);
    logic [7:0]   opc;
    logic [15:0]  cid, qid;
    logic [31:0]  nsid, cdw10, cdw11, dw0;
    logic [63:0]  prp1, prp2;
    logic [14:0]  st;
    byte unsigned b[$];
    opc = s[0][7:0]; cid = s[0][31:16]; nsid = s[1];
    prp1 = {s[7], s[6]}; prp2 = {s[9], s[8]};
    cdw10 = s[10]; cdw11 = s[11];
    st = 0; dw0 = 0;
    repeat ($urandom_range(cmd_latency_min, cmd_latency_max)) @(posedge clk);
    log_ev(EV_ADMIN, opc, {cdw10, 16'h0, cid});
    if (verbose) $display("[ssd_model] admin opc %02h cid %0d cdw10 %08h cdw11 %08h", opc, cid, cdw10, cdw11);
    if (s[0][15:14] != 0 || s[0][9:8] != 0) st = mk_status(0, 8'h02);     // PSDT / FUSE not supported
    else case (opc)
      8'h06: begin  // Identify
        case (cdw10[7:0])
          8'h01: begin build_id_ctrl(b); write_prp_4k(prp1, prp2, b); end
          8'h00: begin
            if (nsid != 1) st = mk_status(0, 8'h0B);
            else begin build_id_ns(b); write_prp_4k(prp1, prp2, b); end
          end
          default: st = mk_status(0, 8'h02);
        endcase
      end
      8'h09: begin  // Set Features
        case (cdw10[7:0])
          8'h07: begin nq_set = 1; dw0 = 32'h0000_0000; end      // NCQA=0, NSQA=0 (one each)
          8'h06: dw0 = 0;                                       // volatile write cache
          default: st = mk_status(0, 8'h02);
        endcase
      end
      8'h05: begin  // Create I/O CQ
        qid = cdw10[15:0];
        if (!nq_set)                                          st = mk_status(0, 8'h0C);
        else if (qid != 1 || cqs[1].exists)                   st = mk_status(1, 8'h01);
        else if (cdw10[31:16] < 1 || cdw10[31:16] > MQES)     st = mk_status(1, 8'h02);
        else if (cc[23:20] != 4)                              st = mk_status(1, 8'h02);
        else if (!cdw11[0])                                   st = mk_status(0, 8'h02);   // CAP.CQR=1 needs PC=1
        else if (prp1[11:0] != 0)                             st = mk_status(0, 8'h13);
        else begin
          cqs[1] = new();
          cqs[1].exists = 1; cqs[1].base = prp1; cqs[1].size = cdw10[31:16] + 1; cqs[1].phase = 1;
          if (cdw11[1]) $warning("[nvme_ssd_model] Create CQ with IEN=1 (interrupts are not modelled)");
          $display("[nvme_ssd_model @%0t] I/O CQ1 created: base %h, %0d entries", $time, prp1, cqs[1].size);
        end
      end
      8'h01: begin  // Create I/O SQ
        qid = cdw10[15:0];
        if (!nq_set)                                          st = mk_status(0, 8'h0C);
        else if (qid != 1 || sqs[1].exists)                   st = mk_status(1, 8'h01);
        else if (cdw11[31:16] != 1 || !cqs[1].exists)         st = mk_status(1, 8'h00);
        else if (cdw10[31:16] < 1 || cdw10[31:16] > MQES)     st = mk_status(1, 8'h02);
        else if (cc[19:16] != 6)                              st = mk_status(1, 8'h02);
        else if (!cdw11[0])                                   st = mk_status(0, 8'h02);
        else if (prp1[11:0] != 0)                             st = mk_status(0, 8'h13);
        else begin
          sqs[1] = new();
          sqs[1].exists = 1; sqs[1].base = prp1; sqs[1].size = cdw10[31:16] + 1; sqs[1].cqid = 1;
          $display("[nvme_ssd_model @%0t] I/O SQ1 created: base %h, %0d entries", $time, prp1, sqs[1].size);
        end
      end
      8'h00: begin  // Delete I/O SQ
        qid = cdw10[15:0];
        if (qid != 1 || !sqs[1].exists) st = mk_status(1, 8'h01);
        else begin
          if (sqs[1].head != sqs[1].tail)
            $warning("[nvme_ssd_model] Delete SQ1 with %0d unfetched commands (aborted)",
                     (sqs[1].tail + sqs[1].size - sqs[1].head) % sqs[1].size);
          if (active_cmds != 0) $warning("[nvme_ssd_model] Delete SQ1 while %0d commands are in flight (waiting)", active_cmds);
          while (active_cmds != 0) @(posedge clk);
          sqs[1].exists = 0;
        end
      end
      8'h04: begin  // Delete I/O CQ
        qid = cdw10[15:0];
        if (qid != 1 || !cqs[1].exists) st = mk_status(1, 8'h01);
        else if (sqs[1].exists)         st = mk_status(1, 8'h0C);   // Invalid Queue Deletion
        else begin
          while (cqs[1].pend.size() != 0) @(posedge clk);
          cqs[1].exists = 0;
        end
      end
      default: st = mk_status(0, 8'h01);   // Invalid Command Opcode
    endcase
    if (st != 0) $warning("[nvme_ssd_model] admin opc %02h cid %0d completed with status %04h (SCT %0d SC %02h)",
                          opc, cid, st, st[10:8], st[7:0]);
    post_cqe(0, 0, cid, st, dw0);
  endtask

  task automatic admin_engine();
    forever begin
      logic [31:0] s[16];
      dma_ctx      ctx;
      @(posedge clk);
      if (!csts_rdy || !sqs[0].exists || csts_shst != 0) continue;
      if (sqs[0].head == sqs[0].tail) continue;
      repeat (fetch_delay_cycles) @(posedge clk);
      ctx = new(64);
      issue_mrd(sqs[0].base + 64 * sqs[0].head, 64, ctx, 0, 0);
      wait_ctx(ctx);
      for (int i = 0; i < 16; i++) s[i] = {ctx.mem[4*i+3], ctx.mem[4*i+2], ctx.mem[4*i+1], ctx.mem[4*i]};
      sqs[0].head = (sqs[0].head + 1) % sqs[0].size;
      if (ctx.errors != 0) begin
        err("admin SQE fetch failed - command dropped");
        continue;
      end
      process_admin(s);
    end
  endtask

  // ===========================================================================
  // I/O engine
  // ===========================================================================
  // Fetch a PRP list of `entries` entries starting at prp2 (follows chain pointers)
  task automatic fetch_prp_list(input logic [63:0] prp2, input int unsigned entries,
                                output logic [63:0] list[$], output bit bad);
    logic [63:0] cur;
    int unsigned rem, slots, n;
    bad = 0;
    list.delete();
    cur = prp2;
    rem = entries;
    while (rem > 0) begin
      dma_ctx ctx;
      bit     use_small;
      slots = (4096 - (cur & 64'hFFF)) / 8;
      n     = (rem <= slots) ? rem : slots;
      ctx   = new(8 * n);
      use_small = ($urandom_range(99) < prp_chunk_pct);
      issue_read_pieces(cur, 8 * n, ctx, 0, use_small);
      wait_ctx(ctx);
      if (ctx.errors != 0) begin bad = 1; return; end
      for (int j = 0; j < n; j++) begin
        logic [63:0] e;
        for (int k = 0; k < 8; k++) e[8*k +: 8] = ctx.mem[8*j + k];
        if (rem <= slots || j < n - 1) list.push_back(e);
        else begin
          if (e[11:0] != 0) begin bad = 1; return; end   // chain pointer must be page aligned
          cur = e;
        end
      end
      if (rem <= slots) rem = 0;
      else rem -= (n - 1);
    end
  endtask

  task automatic do_write(input logic [31:0] s[16], output logic [14:0] st);
    longint unsigned slba;
    int unsigned     nlb, nbytes, npages, pos, seg, sz, myidx;
    logic [63:0]     prp1, prp2, a;
    logic [63:0]     pages[$];
    logic [63:0]     list[$];
    bit              bad;
    dma_ctx          ctx;
    st     = 0;
    slba   = {s[11], s[10]};
    nlb    = s[12][15:0] + 1;
    nbytes = nlb * 512;
    prp1   = {s[7], s[6]};
    prp2   = {s[9], s[8]};
    myidx  = io_write_idx++;
    if (s[1] != 1)                          begin st = mk_status(0, 8'h0B); return; end   // Invalid Namespace
    if (slba + nlb > NSZE)                  begin st = mk_status(0, 8'h80); return; end   // LBA Out of Range
    if (nbytes > (4096 << MDTS))            begin st = mk_status(0, 8'h02); return; end   // > MDTS
    if (prp1[1:0] != 0)                     begin st = mk_status(0, 8'h13); return; end   // PRP Offset Invalid
    npages = ((prp1 & 64'hFFF) + nbytes + 4095) / 4096;
    pages.push_back(prp1);
    if (npages == 2) begin
      if (prp2[11:0] != 0) begin st = mk_status(0, 8'h13); return; end
      pages.push_back(prp2);
    end else if (npages > 2) begin
      if (prp2[2:0] != 0) begin st = mk_status(0, 8'h13); return; end
      fetch_prp_list(prp2, npages - 1, list, bad);
      if (bad) begin st = mk_status(0, 8'h13); return; end
      foreach (list[i]) begin
        if (list[i][11:0] != 0) begin st = mk_status(0, 8'h13); return; end
        pages.push_back(list[i]);
      end
    end
    // ---- data fetch ----
    ctx = new(nbytes);
    pos = 0;
    foreach (pages[i]) begin
      a   = pages[i];
      seg = 4096 - (a & 64'hFFF);
      if (seg > nbytes - pos) seg = nbytes - pos;
      while (seg > 0) begin
        sz = 4 * $urandom_range(mrd_size_min / 4, mrrs_bytes() / 4);
        if (sz > seg) sz = seg;
        issue_mrd(a, sz, ctx, pos, 1);
        a += sz; pos += sz; seg -= sz;
        if (mrd_gap_max > 0) repeat ($urandom_range(0, mrd_gap_max)) @(posedge clk);
      end
    end
    wait_ctx(ctx);
    if (ctx.errors != 0) begin st = mk_status(0, 8'h04); return; end   // Data Transfer Error
    // ---- store ----
    for (int unsigned b = 0; b < nlb; b++) begin
      blk_t blk;
      if (lba_mem.exists(slba + b) && !allow_overwrite)
        err($sformatf("W-1: LBA %0d written twice", slba + b));
      for (int i = 0; i < 512; i++) blk[i] = ctx.mem[512*b + i];
      lba_mem[slba + b] = blk;
    end
    n_io_writes++;
    if (myidx == inject_status_at_cmd) begin
      st = inject_status_value;
      $display("[nvme_ssd_model @%0t] injecting status %04h into I/O Write #%0d (cid %0d)", $time, st, myidx, s[0][31:16]);
    end
  endtask

  task automatic run_io_cmd(input logic [31:0] s[16]);
    logic [7:0]  opc;
    logic [15:0] cid;
    logic [14:0] st;
    opc = s[0][7:0]; cid = s[0][31:16];
    st  = 0;
    repeat ($urandom_range(cmd_latency_min, cmd_latency_max)) @(posedge clk);
    if (s[0][15:14] != 0 || s[0][9:8] != 0) st = mk_status(0, 8'h02);
    else case (opc)
      8'h01: begin
        do_write(s, st);
        log_ev(EV_IO, opc, {s[10], s[12][15:0], cid});
      end
      8'h00: begin
        if (s[1] != 1 && s[1] != 32'hFFFF_FFFF) st = mk_status(0, 8'h0B);
        else if (s[1] == 32'hFFFF_FFFF)         st = mk_status(0, 8'h0B);   // VWC bits 2:1 = 00: broadcast not supported
        n_flush++;
        log_ev(EV_IO, opc, {32'h0, 16'h0, cid});
        $display("[nvme_ssd_model @%0t] Flush (cid %0d)", $time, cid);
      end
      default: st = mk_status(0, 8'h01);
    endcase
    if (verbose) $display("[ssd_model] I/O opc %02h cid %0d done, status %04h", opc, cid, st);
    if (st != 0 && st != inject_status_value)
      $warning("[nvme_ssd_model] I/O opc %02h cid %0d completed with status %04h (SCT %0d SC %02h)",
               opc, cid, st, st[10:8], st[7:0]);
    post_cqe(sqs[1].cqid, 1, cid, st, 0);
    active_cmds--;
  endtask

  task automatic io_engine();
    forever begin
      int unsigned avail, contig, n, h;
      dma_ctx      ctx;
      @(posedge clk);
      if (!csts_rdy || !sqs[1].exists || csts_shst != 0) continue;
      if (sqs[1].head == sqs[1].tail) continue;
      if (active_cmds >= concurrent_cmds) continue;
      repeat (fetch_delay_cycles) @(posedge clk);
      if (!sqs[1].exists) continue;
      avail  = (sqs[1].tail + sqs[1].size - sqs[1].head) % sqs[1].size;
      contig = sqs[1].size - sqs[1].head;
      n = avail;
      if (n > contig)                        n = contig;
      if (n > sq_fetch_max)                  n = sq_fetch_max;
      if (n > concurrent_cmds - active_cmds) n = concurrent_cmds - active_cmds;
      if (n > mrrs_bytes() / 64)             n = mrrs_bytes() / 64;
      if (n < 1) n = 1;
      h   = sqs[1].head;
      ctx = new(64 * n);
      issue_mrd(sqs[1].base + 64 * h, 64 * n, ctx, 0, 0);
      wait_ctx(ctx);
      if (ctx.errors != 0) begin err("I/O SQE fetch failed - commands dropped"); end
      sqs[1].head = (h + n) % sqs[1].size;
      if (ctx.errors != 0) continue;
      for (int k = 0; k < n; k++) begin
        automatic logic [31:0] s[16];
        for (int i = 0; i < 16; i++)
          s[i] = {ctx.mem[64*k + 4*i + 3], ctx.mem[64*k + 4*i + 2], ctx.mem[64*k + 4*i + 1], ctx.mem[64*k + 4*i]};
        active_cmds++;
        fork
          run_io_cmd(s);
        join_none
      end
    end
  endtask

  // ===========================================================================
  // Start-up
  // ===========================================================================
  bit started = 0;

  initial begin
    cfg_init();
    ctrl_reset();
    for (int i = 0; i < 256; i++) rd_tab[i].busy = 0;
    bdf_captured = 0; bdf = 16'h0000; active_cmds = 0; io_write_idx = 0;
    crs_left = crs_count;
    wait (perst_n === 1'b1);
    crs_left = crs_count;       // knob may have been changed by the tb at time 0
    started  = 1;
    fork
      dispatcher();
      delay_thread();
      ctrl_thread();
      admin_engine();
      io_engine();
      cq_poster(0);
      cq_poster(1);
      watchdog();
    join_none
  end

  always @(negedge perst_n)
    if (started) err("PERST# re-asserted after start-up: not supported by the model");

  // Idle: no command in flight, nothing to post, no outstanding read
  function automatic bit idle();
    return (active_cmds == 0) && (outstanding == 0) && (cqs[0].pend.size() == 0) && (cqs[1].pend.size() == 0)
           && (!sqs[0].exists || sqs[0].head == sqs[0].tail) && (!sqs[1].exists || sqs[1].head == sqs[1].tail);
  endfunction

endmodule
