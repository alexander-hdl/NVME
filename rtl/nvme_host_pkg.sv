// =============================================================================
// nvme_host_pkg.sv — shared constants, address map and TLP header helpers.
// Design contract: see ARCHITECTURE.md §2, §3, §6.
// =============================================================================
package nvme_host_pkg;

  // ---------------------------------------------------------------------------
  // Build-time parameters (defaults; overridable via top-level parameters)
  // ---------------------------------------------------------------------------
  localparam int unsigned N_SLOTS_DEF     = 8;
  localparam int unsigned SLOT_BYTES_DEF  = 131072;      // 128 KiB per command
  localparam int unsigned ADMIN_QSIZE     = 32;
  localparam int unsigned IO_QSIZE        = 64;
  localparam logic [2:0]  MPS_CAP_CODE    = 3'd1;        // 256 B (must equal pcie_7x GUI MPS)
  localparam logic [2:0]  MRRS_CODE       = 3'd2;        // 512 B
  localparam bit          EXT_TAG_ALLOW   = 1'b1;
  localparam logic [15:0] RP_ID           = 16'h0000;    // Root Port requester/completer ID (bus0 dev0 fn0)
  localparam logic [15:0] SSD_BDF         = 16'h0100;    // bus1 dev0 fn0

  // ---------------------------------------------------------------------------
  // Host memory map as seen by the SSD (32-bit addresses) — ARCHITECTURE.md §2
  // ---------------------------------------------------------------------------
  localparam logic [31:0] ASQ_BASE   = 32'h1000_0000;   // 32 x 64 B
  localparam logic [31:0] ACQ_BASE   = 32'h1000_1000;   // 32 x 16 B
  localparam logic [31:0] IDENT_BASE = 32'h1000_2000;   // 4 KiB
  localparam logic [31:0] IOSQ_BASE  = 32'h1000_3000;   // 64 x 64 B
  localparam logic [31:0] IOCQ_BASE  = 32'h1000_4000;   // 64 x 16 B
  localparam logic [31:0] PRP_BASE   = 32'h2000_0000;   // + k*0x1000
  localparam logic [31:0] DATA_BASE  = 32'h4000_0000;   // + k*SLOT_BYTES
  localparam logic [31:0] BAR0_ADDR  = 32'hC000_0000;

  localparam logic [31:0] ASQ_BYTES   = ADMIN_QSIZE * 64;
  localparam logic [31:0] ACQ_BYTES   = ADMIN_QSIZE * 16;
  localparam logic [31:0] IDENT_BYTES = 32'd4096;
  localparam logic [31:0] IOSQ_BYTES  = IO_QSIZE * 64;
  localparam logic [31:0] IOCQ_BYTES  = IO_QSIZE * 16;

  // Region select codes for the completer read mux
  localparam logic [1:0] RSEL_ASQ  = 2'd0;
  localparam logic [1:0] RSEL_IOSQ = 2'd1;
  localparam logic [1:0] RSEL_PRP  = 2'd2;
  localparam logic [1:0] RSEL_DATA = 2'd3;

  // ---------------------------------------------------------------------------
  // NVMe controller registers (BAR0 offsets) and constants
  // ---------------------------------------------------------------------------
  localparam logic [15:0] NVME_CAP_LO = 16'h0000, NVME_CAP_HI = 16'h0004, NVME_VS = 16'h0008;
  localparam logic [15:0] NVME_CC = 16'h0014, NVME_CSTS = 16'h001C, NVME_AQA = 16'h0024;
  localparam logic [15:0] NVME_ASQ_LO = 16'h0028, NVME_ASQ_HI = 16'h002C;
  localparam logic [15:0] NVME_ACQ_LO = 16'h0030, NVME_ACQ_HI = 16'h0034;
  localparam logic [15:0] NVME_DBL_BASE = 16'h1000;

  localparam logic [31:0] CC_CONFIG   = 32'h0046_0000;  // IOCQES=4, IOSQES=6, MPS=0, CSS=0, AMS=0, EN=0
  localparam logic [31:0] CC_ENABLE   = 32'h0046_0001;
  localparam logic [31:0] CC_SHUTDOWN = 32'h0046_4001;  // SHN=01b (normal)
  localparam logic [31:0] AQA_VALUE   = {4'd0, 12'(ADMIN_QSIZE-1), 4'd0, 12'(ADMIN_QSIZE-1)};

  // Doorbell byte offset: 0x1000 + (2*qid + is_cq) * (4 << dstrd)
  function automatic logic [31:0] doorbell_addr(input logic [3:0] dstrd, input int unsigned qid, input bit is_cq);
    logic [31:0] stride;
    stride = 32'd4 << dstrd;
    return BAR0_ADDR + 32'(NVME_DBL_BASE) + (32'(2*qid) + 32'(is_cq)) * stride;
  endfunction

  // Admin opcodes
  localparam logic [7:0] AOPC_DELETE_SQ = 8'h00, AOPC_CREATE_SQ = 8'h01, AOPC_DELETE_CQ = 8'h04,
                         AOPC_CREATE_CQ = 8'h05, AOPC_IDENTIFY = 8'h06, AOPC_SET_FEATURES = 8'h09;
  // NVM command set opcodes
  localparam logic [7:0] NOPC_FLUSH = 8'h00, NOPC_WRITE = 8'h01, NOPC_READ = 8'h02;

  // ---------------------------------------------------------------------------
  // PCIe config space offsets
  // ---------------------------------------------------------------------------
  localparam logic [7:0] CFG_VID_DID = 8'h00, CFG_COMMAND = 8'h04, CFG_CLASS = 8'h08,
                         CFG_BAR0 = 8'h10, CFG_BAR1 = 8'h14, CFG_CAP_PTR = 8'h34;
  localparam logic [7:0] PCIE_CAP_ID = 8'h10;
  localparam logic [31:0] CMD_MSE_BME_INTXDIS = 32'h0000_0406;

  // Root Port own config space (cfg_mgmt dword addresses) — PG054, capability base 0x60
  localparam logic [9:0] RP_DW_COMMAND  = 10'h001;
  localparam logic [9:0] RP_DW_BUSNUM   = 10'h006;
  localparam logic [9:0] RP_DW_DEVCTL   = 10'h01A;   // 0x68
  localparam logic [9:0] RP_DW_LINKCTL  = 10'h01C;   // 0x70

  // ---------------------------------------------------------------------------
  // TLP header helpers. Header DWs use the PCIe spec bit layout.
  // ---------------------------------------------------------------------------
  localparam logic [7:0] FT_MRD32 = 8'h00, FT_MRD64 = 8'h20, FT_MWR32 = 8'h40, FT_MWR64 = 8'h60,
                         FT_CFGRD0 = 8'h04, FT_CFGWR0 = 8'h44, FT_CPL = 8'h0A, FT_CPLD = 8'h4A;

  localparam logic [2:0] CPL_SC = 3'b000, CPL_UR = 3'b001, CPL_CRS = 3'b010, CPL_CA = 3'b100;

  function automatic logic [7:0]  tlp_fmt_type(input logic [31:0] h0); return h0[31:24]; endfunction
  function automatic logic [9:0]  tlp_len_dw (input logic [31:0] h0); return h0[9:0];   endfunction
  function automatic bit tlp_is_4dw (input logic [31:0] h0); return h0[29]; endfunction
  function automatic bit tlp_has_data(input logic [31:0] h0); return h0[30]; endfunction
  function automatic bit tlp_is_mrd (input logic [31:0] h0); return (h0[31:24] == FT_MRD32) || (h0[31:24] == FT_MRD64); endfunction
  function automatic bit tlp_is_mwr (input logic [31:0] h0); return (h0[31:24] == FT_MWR32) || (h0[31:24] == FT_MWR64); endfunction
  function automatic bit tlp_is_cpl (input logic [31:0] h0); return (h0[31:24] == FT_CPL)  || (h0[31:24] == FT_CPLD);  endfunction
  function automatic bit tlp_is_cpld(input logic [31:0] h0); return (h0[31:24] == FT_CPLD); endfunction

  // Payload length in DWs (0 encodes 1024). Header-only TLPs return 0.
  function automatic int unsigned tlp_payload_dw(input logic [31:0] h0);
    if (!h0[30]) return 0;
    return (h0[9:0] == 10'd0) ? 1024 : int'(h0[9:0]);
  endfunction

  // --- request header fields (DW1) ---
  function automatic logic [15:0] req_reqid(input logic [31:0] h1); return h1[31:16]; endfunction
  function automatic logic [7:0]  req_tag  (input logic [31:0] h1); return h1[15:8];  endfunction
  function automatic logic [3:0]  req_lbe  (input logic [31:0] h1); return h1[7:4];   endfunction
  function automatic logic [3:0]  req_fbe  (input logic [31:0] h1); return h1[3:0];   endfunction
  // --- completion header fields ---
  function automatic logic [15:0] cpl_cplid (input logic [31:0] h1); return h1[31:16]; endfunction
  function automatic logic [2:0]  cpl_status(input logic [31:0] h1); return h1[15:13]; endfunction
  function automatic logic [11:0] cpl_bc    (input logic [31:0] h1); return h1[11:0];  endfunction
  function automatic logic [15:0] cpl_reqid (input logic [31:0] h2); return h2[31:16]; endfunction
  function automatic logic [7:0]  cpl_tag   (input logic [31:0] h2); return h2[15:8];  endfunction
  function automatic logic [6:0]  cpl_la    (input logic [31:0] h2); return h2[6:0];   endfunction

  // --- header builders (return {H3,H2,H1,H0} packed as a 128-bit beat, H0 in [31:0]) ---
  function automatic logic [127:0] hdr_mrd32(input logic [31:0] addr, input logic [9:0] len_dw,
                                             input logic [15:0] reqid, input logic [7:0] tag,
                                             input logic [3:0] fbe, input logic [3:0] lbe);
    logic [31:0] h0, h1, h2;
    h0 = {FT_MRD32, 1'b0, 3'b000, 4'b0000, 1'b0, 1'b0, 2'b00, 2'b00, len_dw};
    h1 = {reqid, tag, lbe, fbe};
    h2 = {addr[31:2], 2'b00};
    return {32'h0, h2, h1, h0};
  endfunction

  function automatic logic [127:0] hdr_mwr32(input logic [31:0] addr, input logic [9:0] len_dw,
                                             input logic [15:0] reqid, input logic [7:0] tag,
                                             input logic [3:0] fbe, input logic [3:0] lbe);
    logic [31:0] h0, h1, h2;
    h0 = {FT_MWR32, 1'b0, 3'b000, 4'b0000, 1'b0, 1'b0, 2'b00, 2'b00, len_dw};
    h1 = {reqid, tag, lbe, fbe};
    h2 = {addr[31:2], 2'b00};
    return {32'h0, h2, h1, h0};
  endfunction

  // Config Type 0: bdf = {bus[7:0], dev[4:0], fn[2:0]}; reg_dw = 10-bit dword register number {extreg[3:0], reg[5:0]}
  function automatic logic [127:0] hdr_cfg0(input bit is_write, input logic [15:0] bdf, input logic [9:0] reg_dw,
                                            input logic [15:0] reqid, input logic [7:0] tag, input logic [3:0] fbe);
    logic [31:0] h0, h1, h2;
    h0 = {(is_write ? FT_CFGWR0 : FT_CFGRD0), 1'b0, 3'b000, 4'b0000, 1'b0, 1'b0, 2'b00, 2'b00, 10'd1};
    h1 = {reqid, tag, 4'b0000, fbe};
    h2 = {bdf, 4'b0000, reg_dw, 2'b00};
    return {32'h0, h2, h1, h0};
  endfunction

  function automatic logic [127:0] hdr_cpl(input bit with_data, input logic [15:0] cplid, input logic [2:0] status,
                                           input logic [11:0] bc, input logic [15:0] reqid, input logic [7:0] tag,
                                           input logic [6:0] lower_addr, input logic [9:0] len_dw);
    logic [31:0] h0, h1, h2;
    h0 = {(with_data ? FT_CPLD : FT_CPL), 1'b0, 3'b000, 4'b0000, 1'b0, 1'b0, 2'b00, 2'b00, len_dw};
    h1 = {cplid, status, 1'b0, bc};
    h2 = {reqid, tag, 1'b0, lower_addr};
    return {32'h0, h2, h1, h0};
  endfunction

  function automatic logic [31:0] bswap32(input logic [31:0] x);
    return {x[7:0], x[15:8], x[23:16], x[31:24]};
  endfunction

  // Byte count for a 1-DW read per PCIe Base Table 2-27 (first BE only, last BE = 0)
  function automatic logic [11:0] bc_from_fbe(input logic [3:0] fbe);
    // Byte count = distance from first enabled byte to last enabled byte, inclusive.
    casez (fbe)
      4'b1??1: return 12'd4;          // 1001,1011,1101,1111
      4'b01?1: return 12'd3;          // 0101,0111
      4'b1?10: return 12'd3;          // 1010,1110
      4'b0011, 4'b0110, 4'b1100: return 12'd2;
      default: return 12'd1;          // single byte or 0000 (zero-length read -> 1)
    endcase
  endfunction

  // Lower address of the first enabled byte for a request starting at DW-aligned addr with first BE fbe
  function automatic logic [6:0] lower_addr_first(input logic [31:0] addr, input logic [3:0] fbe);
    logic [1:0] off;
    off = fbe[0] ? 2'd0 : fbe[1] ? 2'd1 : fbe[2] ? 2'd2 : fbe[3] ? 2'd3 : 2'd0;
    return {addr[6:2], off};
  endfunction

endpackage
