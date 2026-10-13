// =============================================================================
// pcie_sim_pkg.sv — shared simulation-only types for the behavioural models
//
//   pcie7x_core_model  <== TLP mailboxes ==>  nvme_ssd_model
//
// The two models exchange whole TLPs (not AXI beats) through two package-level
// mailboxes.  Because the mailboxes live in the package there can be exactly ONE
// core-model / SSD-model pair per simulation (sufficient for every testbench of
// this project).
//
//   mb_down : core model -> SSD  (TLPs the host transmitted on s_axis_tx, plus
//                                 completions the Root Port generates itself,
//                                 e.g. Cpl-UR when the RP Command.BME = 0)
//   mb_up   : SSD -> core model  (TLPs the SSD transmits towards the host; the
//                                 core model serialises them onto m_axis_rx)
//
// TLP representation (tlp_t):
//   hdr[0..3]  header DWs in PCIe-spec bit layout (exactly what appears in
//              tdata[31:0], [63:32], ... of the 7-series AXI-Stream);
//              hdr[3] is 0 for 3DW headers.
//   data[$]    payload as a byte queue in MEMORY ORDER: data[0] is the byte at
//              the lowest address (the byte enabled by First BE bit 0).  On the
//              7-series AXI-Stream the payload DWs are big-endian, i.e. data[0]
//              travels in bits [31:24] of the first payload DW — the conversion is
//              done ONLY inside pcie7x_core_model (and the tb's tiny host).
//   poison     EP / rx_err_fwd indication.
//   acc_cycle, acc_beats : metadata stamped by the core model on TX TLPs (user_clk
//              cycle at which the last beat was accepted, cumulative number of TX
//              beats accepted up to then) — used for the throughput measurement.
// =============================================================================
package pcie_sim_pkg;

  typedef struct {
    logic [31:0]  hdr [4];
    byte unsigned data[$];
    bit           poison;
    longint       acc_cycle;
    longint       acc_beats;
  } tlp_t;

  mailbox #(tlp_t) mb_down = new();
  mailbox #(tlp_t) mb_up   = new();

  // --------------------------------------------------------------------------
  // Small helpers (header field layout = PCIe spec, see nvme_host_pkg)
  // --------------------------------------------------------------------------
  function automatic int unsigned hdr_dws(input logic [31:0] h0);
    return h0[29] ? 4 : 3;
  endfunction

  function automatic bit has_data(input logic [31:0] h0);
    return h0[30];
  endfunction

  // Payload DW count encoded in Length (0 = 1024); 0 for header-only TLPs.
  function automatic int unsigned len_dw(input logic [31:0] h0);
    if (!h0[30]) return 0;
    return (h0[9:0] == 10'd0) ? 1024 : int'(h0[9:0]);
  endfunction

  // Non-posted (MRd / IORd / IOWr / CfgRd / CfgWr): Type in {00000(MRd only, no data), 00010, 0010x}
  function automatic bit is_np(input logic [31:0] h0);
    logic [4:0] ty; ty = h0[28:24];
    if (ty == 5'b00000) return !h0[30];              // MRd (MWr is posted)
    if (ty == 5'b00001) return !h0[30];              // MRdLk
    if (ty == 5'b00010) return 1'b1;                 // IORd/IOWr
    if (ty[4:1] == 4'b0010) return 1'b1;             // Cfg0/Cfg1
    return 1'b0;
  endfunction

  function automatic bit is_mrd (input logic [31:0] h0); return (h0[28:24] == 5'b00000) && !h0[30]; endfunction
  function automatic bit is_mwr (input logic [31:0] h0); return (h0[28:24] == 5'b00000) &&  h0[30]; endfunction
  function automatic bit is_cpl (input logic [31:0] h0); return (h0[28:24] == 5'b01010); endfunction
  function automatic bit is_cfg0(input logic [31:0] h0); return (h0[28:24] == 5'b00100); endfunction

  // Address of a memory request (3DW: H2; 4DW: {H2,H3})
  function automatic logic [63:0] req_addr(input tlp_t t);
    if (t.hdr[0][29]) return {t.hdr[2], t.hdr[3][31:2], 2'b00};
    return {32'h0, t.hdr[2][31:2], 2'b00};
  endfunction

  // Bus-order DW (big-endian within the DW) <-> memory-order bytes
  function automatic logic [31:0] bytes_to_bus_dw(input byte unsigned b0, b1, b2, b3);
    return {b0, b1, b2, b3};
  endfunction

  // Little-endian value <-> bytes (memory order) helpers
  function automatic logic [31:0] le32(input byte unsigned d[$], input int unsigned off);
    return {d[off+3], d[off+2], d[off+1], d[off]};
  endfunction

  function automatic string tlp_name(input logic [31:0] h0);
    case (h0[31:24])
      8'h00: return "MRd32";   8'h20: return "MRd64";
      8'h40: return "MWr32";   8'h60: return "MWr64";
      8'h04: return "CfgRd0";  8'h44: return "CfgWr0";
      8'h05: return "CfgRd1";  8'h45: return "CfgWr1";
      8'h0A: return "Cpl";     8'h4A: return "CplD";
      default: return $sformatf("FmtType_%02h", h0[31:24]);
    endcase
  endfunction

  function automatic string tlp_str(input tlp_t t);
    return $sformatf("%s H=%08h_%08h_%08h_%08h len=%0dB%s", tlp_name(t.hdr[0]),
                     t.hdr[0], t.hdr[1], t.hdr[2], t.hdr[3], t.data.size(), t.poison ? " POISONED" : "");
  endfunction

  // --------------------------------------------------------------------------
  // SSD model internals (declared here because xsim handles package classes best)
  // --------------------------------------------------------------------------
  // DMA read context: one per SQE fetch / PRP-list fetch / data fetch.  Every MRd
  // issued for it increments `pending`; the completion handler copies the data to
  // mem[off..] and decrements `pending`.
  class dma_ctx;
    byte unsigned mem[];
    int           pending;
    int           errors;
    function new(int unsigned n);
      mem = new[n];
      foreach (mem[i]) mem[i] = 8'h00;
      pending = 0;
      errors  = 0;
    endfunction
  endclass

  // One completion queue entry waiting to be posted
  typedef struct {
    logic [31:0] dw0;
    logic [15:0] sqid;
    logic [15:0] cid;
    logic [14:0] status;    // CQE DW3[31:17] = {DNR, M, CRD[1:0], SCT[2:0], SC[7:0]}
  } cqe_t;

  class nvme_sq;
    bit          exists;
    logic [63:0] base;
    int unsigned size;      // entries (1's based)
    int unsigned tail;      // last value written to the tail doorbell
    int unsigned head;      // controller fetch pointer (reported as SQHD)
    int unsigned cqid;
    function new(); exists = 0; base = 0; size = 0; tail = 0; head = 0; cqid = 0; endfunction
  endclass

  class nvme_cq;
    bit          exists;
    logic [63:0] base;
    int unsigned size;
    int unsigned tail;      // next entry the controller writes
    int unsigned head;      // last value written to the head doorbell
    bit          phase;     // phase tag the controller writes (1 on the first pass)
    cqe_t        pend[$];   // CQEs waiting to be posted
    function new(); exists = 0; base = 0; size = 0; tail = 0; head = 0; phase = 1; endfunction
  endclass

  // --------------------------------------------------------------------------
  // SSD model event log (inspected by tb_top for sequence checks)
  // --------------------------------------------------------------------------
  typedef enum int {
    EV_CFGRD,      // a = cfg byte offset, b = value returned
    EV_CFGWR,      // a = cfg byte offset, b = {be[3:0], value_after_merge[31:0]}
    EV_CFG_CRS,    // a = cfg byte offset (request answered with CRS)
    EV_MMIO_RD,    // a = BAR0 offset, b = value returned
    EV_MMIO_WR,    // a = BAR0 offset, b = value (merged 32-bit)
    EV_ADMIN,      // a = opcode, b = {cdw10, cid}  (logged when the command is executed)
    EV_IO,         // a = opcode, b = {slba[31:0]... } see nvme_ssd_model
    EV_CQE,        // a = qid,   b = {status[14:0], cid}
    EV_RDY,        // a = new CSTS.RDY
    EV_SHST        // a = new CSTS.SHST
  } ev_kind_e;

  typedef struct {
    ev_kind_e    kind;
    longint      cycle;
    logic [63:0] a;
    logic [63:0] b;
  } ev_t;

endpackage
