// =============================================================================
// hostmem_completer.sv — answers the SSD's Memory Read requests (ARCHITECTURE.md §5.5)
//
// Input : header-only MRd packets (tlp_axis, one beat each).
// Output: CplD packets (header beat + payload beats) or header-only Cpl(UR).
//
// DATA PATH (one request flows left to right; every stage is elastic):
//
//  s_mrd -> [req FIFO 16] -> W (working request) -> D (descriptor reg)
//                                 |  one CplD per      |  issues one rd_en
//                                 |  clock cycle       |  per 16-byte beat
//                                 v                    v
//                         split / BC / LA math    header -> [hdr FIFO]   rd_en/rd_addr/rd_sel
//                                                                         | (memory, 2 cycles)
//        m_cpl <- [out FIFO 16] <- merger <- dw_shift(left) <- [ret FIFO 16] <- rd_data
//                                   (header beat, then payload beats of that CplD)
//
// 1) W holds the request being split. Each cycle it produces ONE completion
//    descriptor (the next CplD) into D, so a request with n CplDs costs n cycles
//    and consecutive requests cost zero extra cycles (W reloads from the request
//    FIFO in the same cycle its last descriptor is produced).
// 2) D is both the descriptor register and the read issuer. It emits one
//    rd_en per 16-byte beat (address rounded down to 16 B), and when its last
//    beat is issued the next descriptor is loaded in that same cycle: the read
//    stream never has a bubble between CplDs or requests ("prefetch").
// 3) Read data returns exactly 2 cycles after rd_en and is parked in a return
//    FIFO. Back-pressure is exact: a credit counter allows at most RET_DEPTH
//    beats between "rd_en issued" and "popped from the return FIFO", so the
//    return FIFO can never overflow whatever the downstream does.
// 4) dw_shift (left, shift = start_addr[3:2]) drops the DWs in front of the
//    first requested DW so the payload starts at DW0 of beat 1. The merger
//    emits the header beat of a CplD only when its first payload beat is
//    already available (no gap between header and payload inside a packet),
//    then the payload beats until dw_shift's tlast.
//
// SPLITTING / BYTE COUNT / LOWER ADDRESS (PCIe Base 2.0 §2.3.1.1, Table 2-27)
//   mps      = 128 << mps_code bytes.
//   CplD #1  covers [start, next MPS-aligned boundary) or up to the request end.
//   CplD #n  covers one MPS-aligned block (or the tail).
//   Length   = number of DWs in this CplD (1024 is encoded as 0).
//   BC       = bytes still to be delivered INCLUDING this CplD:
//                 total = len*4 - lead - trail          (len > 1)
//                 total = bc_from_fbe(FBE)              (len == 1, FBE==0 -> 1)
//              where lead  = disabled low bytes of the first DW (from FBE) and
//                    trail = disabled high bytes of the last DW (from LBE).
//              BC of the following CplDs = previous BC - (bytes of the previous
//              CplD, minus `lead` for the very first one).  4096 encodes as 0.
//   LA       = first CplD: address of the first enabled byte
//                          (lower_addr_first(addr, FBE));
//              later CplDs: start_addr[6:0] (always DW aligned, hence 0 or a
//                          multiple of 4).
//   Worked example: MRd 512 B at 0x...040, MPS 256:
//        #1: 48 DW (192 B) BC=512 LA=0x40  | #2: 64 DW BC=320 LA=0x00
//        #3: 16 DW (64 B)  BC=64  LA=0x00
//
// ADDRESS DECODE (ARCHITECTURE.md §2). A request is served only if its first
// AND last byte are inside one readable region (ASQ, IOSQ, PRP window, data
// ring); otherwise the answer is a header-only Cpl with Status=UR, BC = the
// requested byte count, LA = lower_addr_first, Length = 0.
// 4DW requests use the PCIe layout (H2 = addr[63:32], H3 = addr[31:2]); a
// non-zero upper half is outside the 32-bit map and gets UR.
//
// Requests are served strictly in arrival order, so completions of one tag
// are in increasing address order and tags never need reordering.
// Latency (header accepted -> first CplD beat): 8 cycles with the reference
// primitives (see tb_completer).
// =============================================================================
module hostmem_completer
  import nvme_host_pkg::*;
#(
  parameter int unsigned N_SLOTS    = N_SLOTS_DEF,
  parameter int unsigned SLOT_BYTES = SLOT_BYTES_DEF
)(
  input  logic         clk,
  input  logic         rst,
  // MRd requests (tlp_axis, header only)
  input  logic [127:0] s_mrd_tdata,
  input  logic [15:0]  s_mrd_tkeep,
  input  logic         s_mrd_tlast,
  input  logic [1:0]   s_mrd_tuser,
  input  logic         s_mrd_tvalid,
  output logic         s_mrd_tready,
  output logic         np_backpressure,      // request FIFO has < 2 free entries
  // completions (tlp_axis)
  output logic [127:0] m_cpl_tdata,
  output logic [15:0]  m_cpl_tkeep,
  output logic         m_cpl_tlast,
  output logic [1:0]   m_cpl_tuser,
  output logic         m_cpl_tvalid,
  input  logic         m_cpl_tready,
  // configuration
  input  logic [2:0]   mps_code,             // 0=128 B, 1=256 B, 2=512 B ...
  input  logic [15:0]  completer_id,
  // memory read port (data valid exactly 2 cycles after rd_en)
  output logic [31:0]  rd_addr,              // region-relative byte address of a 16 B beat
  output logic [1:0]   rd_sel,               // RSEL_*
  output logic         rd_en,
  input  logic [127:0] rd_data
);

  // ---------------------------------------------------------------------------
  // Constants
  // ---------------------------------------------------------------------------
  localparam int unsigned REQ_DEPTH = 16;
  localparam int unsigned HDR_DEPTH = 8;
  localparam int unsigned RET_DEPTH = 16;
  localparam int unsigned OUT_DEPTH = 16;
  localparam int unsigned RET_CW    = $clog2(RET_DEPTH) + 1;

  localparam logic [31:0] PRP_BYTES  = 32'(N_SLOTS) * 32'd4096;
  localparam logic [31:0] DATA_BYTES = 32'(N_SLOTS) * 32'(SLOT_BYTES);

  localparam logic [2:0]  MPS_CODE_MAX = 3'd5;       // 4096 B

  // ---------------------------------------------------------------------------
  // Helper functions
  // ---------------------------------------------------------------------------
  // True if [addr, addr+nbytes) lies inside [base, base+size).
  function automatic logic fits(input logic [31:0] addr, input logic [31:0] base,
                                input logic [31:0] size, input logic [12:0] nbytes);
    logic [32:0] off;
    logic [33:0] fin;
    off = {1'b0, addr} - {1'b0, base};          // bit 32 set <=> addr < base
    fin = {1'b0, off} + {21'b0, nbytes};
    return (!off[32]) && (fin <= {2'b00, size});
  endfunction

  typedef struct packed {
    logic        hit;
    logic [1:0]  sel;
    logic [31:0] rel;       // region-relative byte address
  } region_t;

  function automatic region_t decode_region(input logic [31:0] addr, input logic [12:0] nbytes);
    region_t r;
    r = '0;
    if (fits(addr, ASQ_BASE, ASQ_BYTES, nbytes)) begin
      r.hit = 1'b1; r.sel = RSEL_ASQ;  r.rel = addr - ASQ_BASE;
    end else if (fits(addr, IOSQ_BASE, IOSQ_BYTES, nbytes)) begin
      r.hit = 1'b1; r.sel = RSEL_IOSQ; r.rel = addr - IOSQ_BASE;
    end else if (fits(addr, PRP_BASE, PRP_BYTES, nbytes)) begin
      r.hit = 1'b1; r.sel = RSEL_PRP;  r.rel = addr - PRP_BASE;
    end else if (fits(addr, DATA_BASE, DATA_BYTES, nbytes)) begin
      r.hit = 1'b1; r.sel = RSEL_DATA; r.rel = addr - DATA_BASE;
    end
    return r;
  endfunction

  // Disabled bytes at the low end of the first DW (from First BE).
  function automatic logic [1:0] lead_bytes(input logic [3:0] fbe);
    casez (fbe)
      4'b???1: return 2'd0;
      4'b??10: return 2'd1;
      4'b?100: return 2'd2;
      4'b1000: return 2'd3;
      default: return 2'd0;
    endcase
  endfunction

  // Disabled bytes at the high end of the last DW (from Last BE).
  function automatic logic [1:0] trail_bytes(input logic [3:0] lbe);
    casez (lbe)
      4'b1???: return 2'd0;
      4'b01??: return 2'd1;
      4'b001?: return 2'd2;
      4'b0001: return 2'd3;
      default: return 2'd0;
    endcase
  endfunction

  // 96-bit completion header {H2,H1,H0} with TC/Attr copied from the request.
  function automatic logic [95:0] cpl_hdr96(input logic with_data, input logic [15:0] cplid,
                                            input logic [2:0] status, input logic [11:0] bc,
                                            input logic [15:0] reqid, input logic [7:0] tag,
                                            input logic [6:0] la, input logic [9:0] len,
                                            input logic [2:0] tc, input logic [1:0] attr);
    logic [127:0] h;
    h = hdr_cpl(with_data, cplid, status, bc, reqid, tag, la, len);
    h[22:20] = tc;
    h[13:12] = attr;
    return h[95:0];
  endfunction

  // tkeep of the last input beat given the number of valid DWs-1 in it.
  function automatic logic [15:0] keep_last(input logic [1:0] dws_m1);
    case (dws_m1)
      2'd0:    return 16'h000F;
      2'd1:    return 16'h00FF;
      2'd2:    return 16'h0FFF;
      default: return 16'hFFFF;
    endcase
  endfunction

  // ---------------------------------------------------------------------------
  // 1. Request FIFO
  // ---------------------------------------------------------------------------
  logic [127:0] reqf_tdata;
  logic         reqf_valid, reqf_pop, reqf_afull;

  axis_fifo_sync #(.W(128), .UW(2), .DEPTH(REQ_DEPTH)) u_reqf (
    .clk(clk), .rst(rst),
    .s_tdata(s_mrd_tdata), .s_tkeep(s_mrd_tkeep), .s_tlast(s_mrd_tlast), .s_tuser(s_mrd_tuser),
    .s_tvalid(s_mrd_tvalid), .s_tready(s_mrd_tready),
    .m_tdata(reqf_tdata), .m_tkeep(), .m_tlast(), .m_tuser(),
    .m_tvalid(reqf_valid), .m_tready(reqf_pop),
    .count(), .almost_full(reqf_afull));

  assign np_backpressure = reqf_afull;     // free <= 2 (superset of "free < 2")

  // ---------------------------------------------------------------------------
  // 2. Request decode (combinational on the FIFO head) -> working request W
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic        ur;         // answer with Cpl(UR)
    logic [1:0]  sel;
    logic [31:0] rel;        // region-relative byte address of the next CplD start (DW aligned)
    logic [10:0] dw_left;    // DWs still to deliver (1..1024)
    logic [12:0] bc_rem;     // BC of the next CplD (bytes still owed incl. that CplD, <= 4096)
    logic        first;      // next CplD is the first of the request
    logic [1:0]  lead;       // disabled bytes at the start of the first DW
    logic [6:0]  la_first;   // Lower Address of the first CplD
    logic [2:0]  tc;
    logic [1:0]  attr;
    logic [15:0] reqid;
    logic [7:0]  tag;
  } wreq_t;

  wreq_t w, w_load;
  logic  w_valid;

  logic [31:0] rq_h0, rq_h1, rq_h2, rq_h3, rq_addr;
  logic        rq_is4dw, rq_hi_bad;
  logic [10:0] rq_len;
  logic [12:0] rq_nbytes;
  logic [3:0]  rq_fbe, rq_lbe;
  logic [12:0] rq_bc;
  region_t     rq_reg;

  always_comb begin
    {rq_h3, rq_h2, rq_h1, rq_h0} = reqf_tdata;
    rq_is4dw  = rq_h0[29];
    rq_addr   = rq_is4dw ? rq_h3 : rq_h2;
    rq_hi_bad = rq_is4dw && (rq_h2 != 32'd0);
    rq_len    = (rq_h0[9:0] == 10'd0) ? 11'd1024 : {1'b0, rq_h0[9:0]};
    rq_nbytes = {rq_len, 2'b00};
    rq_fbe    = rq_h1[3:0];
    rq_lbe    = rq_h1[7:4];
    rq_reg    = decode_region(rq_addr, rq_nbytes);
    if (rq_len == 11'd1)
      rq_bc = {1'b0, bc_from_fbe(rq_fbe)};
    else
      rq_bc = rq_nbytes - {11'd0, lead_bytes(rq_fbe)} - {11'd0, trail_bytes(rq_lbe)};

    w_load.ur       = rq_hi_bad || !rq_reg.hit;
    w_load.sel      = rq_reg.sel;
    w_load.rel      = rq_reg.rel;
    w_load.dw_left  = rq_len;
    w_load.bc_rem   = rq_bc;
    w_load.first    = 1'b1;
    w_load.lead     = lead_bytes(rq_fbe);
    w_load.la_first = lower_addr_first(rq_addr, rq_fbe);
    w_load.tc       = rq_h0[22:20];
    w_load.attr     = rq_h0[13:12];
    w_load.reqid    = rq_h1[31:16];
    w_load.tag      = rq_h1[15:8];
  end

  // ---------------------------------------------------------------------------
  // 3. Next-CplD computation (combinational on W)
  // ---------------------------------------------------------------------------
  logic [10:0] mps_dw, off_dw, avail_dw, cpl_dw;
  logic        a_last, a_done;
  logic [12:0] a_bytes, a_consumed;
  logic [6:0]  a_la;
  logic [11:0] a_total_in;       // DWs fed to dw_shift: shift + cpl_dw
  logic [8:0]  a_nbeats;
  logic [1:0]  a_last_code;
  logic [95:0] a_hdr;

  always_comb begin
    mps_dw     = 11'd32 << ((mps_code > MPS_CODE_MAX) ? MPS_CODE_MAX : mps_code);
    off_dw     = {1'b0, w.rel[11:2]} & (mps_dw - 11'd1);       // DW offset inside the MPS block
    avail_dw   = mps_dw - off_dw;                              // DWs to the next MPS boundary
    cpl_dw     = (w.dw_left < avail_dw) ? w.dw_left : avail_dw;
    a_last     = (cpl_dw == w.dw_left);
    a_done     = w.ur || a_last;
    a_bytes    = {cpl_dw, 2'b00};
    a_consumed = a_bytes - (w.first ? {11'd0, w.lead} : 13'd0);
    a_la       = w.first ? w.la_first : {w.rel[6:2], 2'b00};
    a_total_in = 12'(w.rel[3:2]) + 12'(cpl_dw);
    a_nbeats   = 9'((a_total_in + 12'd3) >> 2);
    a_last_code = 2'(a_total_in - 12'd1);
    if (w.ur)
      a_hdr = cpl_hdr96(1'b0, completer_id, CPL_UR, w.bc_rem[11:0], w.reqid, w.tag,
                        w.la_first, 10'd0, w.tc, w.attr);
    else
      a_hdr = cpl_hdr96(1'b1, completer_id, CPL_SC, w.bc_rem[11:0], w.reqid, w.tag,
                        a_la, cpl_dw[9:0], w.tc, w.attr);
  end

  // ---------------------------------------------------------------------------
  // 4. Descriptor register D = read issuer
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic        ur;
    logic [95:0] hdr;
    logic [1:0]  sel;
    logic [31:0] rd_addr;      // next beat to read (16 B aligned, region relative)
    logic [1:0]  shift;        // DWs to drop in front (start_addr[3:2])
    logic [8:0]  beats_left;
    logic [1:0]  last_code;    // valid DWs-1 in the last input beat
  } desc_t;

  desc_t d;
  logic  d_valid, d_hdr_sent;
  logic  issue, ur_fire, d_consume, emit;
  logic  hdrf_ready;
  logic [RET_CW-1:0] outstanding;
  logic  ret_pop;

  wire credit_ok  = (outstanding < RET_CW'(RET_DEPTH));
  wire beat_last  = (d.beats_left == 9'd1);
  wire [15:0] beat_keep = beat_last ? keep_last(d.last_code) : 16'hFFFF;

  assign issue     = d_valid && !d.ur && credit_ok && (d_hdr_sent || hdrf_ready);
  assign ur_fire   = d_valid &&  d.ur && hdrf_ready;
  assign d_consume = (issue && beat_last) || ur_fire;
  assign emit      = w_valid && (!d_valid || d_consume);
  assign reqf_pop  = reqf_valid && (!w_valid || (emit && a_done));

  assign rd_en   = issue;
  assign rd_addr = d.rd_addr;
  assign rd_sel  = d.sel;

  always_ff @(posedge clk) begin
    if (rst) begin
      w_valid     <= 1'b0;
      d_valid     <= 1'b0;
      d_hdr_sent  <= 1'b0;
    end else begin
      // --- working request ---
      if (emit) begin
        w.rel     <= w.rel + {19'd0, a_bytes};
        w.dw_left <= w.dw_left - cpl_dw;
        w.bc_rem  <= w.bc_rem - a_consumed;
        w.first   <= 1'b0;
        if (a_done) w_valid <= 1'b0;
      end
      if (reqf_pop) begin
        w       <= w_load;
        w_valid <= 1'b1;
      end
      // --- descriptor / issuer ---
      if (issue) begin
        d.rd_addr    <= d.rd_addr + 32'd16;
        d.beats_left <= d.beats_left - 9'd1;
        d_hdr_sent   <= 1'b1;
      end
      if (d_consume) d_valid <= 1'b0;
      if (emit) begin
        d.ur         <= w.ur;
        d.hdr        <= a_hdr;
        d.sel        <= w.sel;
        d.rd_addr    <= {w.rel[31:4], 4'b0000};
        d.shift      <= w.rel[3:2];
        d.beats_left <= a_nbeats;
        d.last_code  <= a_last_code;
        d_valid      <= 1'b1;
        d_hdr_sent   <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // 5. Header FIFO (one entry per CplD, pushed when its first read is issued)
  // ---------------------------------------------------------------------------
  logic         hdrf_push;
  logic [127:0] hdrf_m_tdata;
  logic         hdrf_m_tlast, hdrf_m_tvalid, hdrf_m_tready;

  assign hdrf_push = (issue && !d_hdr_sent) || ur_fire;

  axis_fifo_sync #(.W(128), .UW(2), .DEPTH(HDR_DEPTH)) u_hdrf (
    .clk(clk), .rst(rst),
    .s_tdata({32'd0, d.hdr}), .s_tkeep(16'hFFFF), .s_tlast(d.ur), .s_tuser(2'b00),
    .s_tvalid(hdrf_push), .s_tready(hdrf_ready),
    .m_tdata(hdrf_m_tdata), .m_tkeep(), .m_tlast(hdrf_m_tlast), .m_tuser(),
    .m_tvalid(hdrf_m_tvalid), .m_tready(hdrf_m_tready),
    .count(), .almost_full());

  // ---------------------------------------------------------------------------
  // 6. Read return path: 2-cycle tag pipeline aligned with rd_data -> return FIFO
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic        v;
    logic [15:0] keep;
    logic        last;
    logic [1:0]  shift;
  } rtag_t;

  rtag_t p1, p2;
  always_ff @(posedge clk) begin
    if (rst) begin
      p1 <= '0; p2 <= '0;
    end else begin
      p1 <= '{v: issue, keep: beat_keep, last: beat_last, shift: d.shift};
      p2 <= p1;
    end
  end

  logic [127:0] ret_tdata;
  logic [15:0]  ret_tkeep;
  logic         ret_tlast, ret_tvalid;
  logic [1:0]   ret_tuser;
  logic         dws_s_tready;

  axis_fifo_sync #(.W(128), .UW(2), .DEPTH(RET_DEPTH)) u_ret (
    .clk(clk), .rst(rst),
    .s_tdata(rd_data), .s_tkeep(p2.keep), .s_tlast(p2.last), .s_tuser(p2.shift),
    .s_tvalid(p2.v), .s_tready(),
    .m_tdata(ret_tdata), .m_tkeep(ret_tkeep), .m_tlast(ret_tlast), .m_tuser(ret_tuser),
    .m_tvalid(ret_tvalid), .m_tready(dws_s_tready),
    .count(), .almost_full());

  assign ret_pop = ret_tvalid && dws_s_tready;

  always_ff @(posedge clk) begin
    if (rst) outstanding <= '0;
    else     outstanding <= outstanding + RET_CW'(issue) - RET_CW'(ret_pop);
  end

  // ---------------------------------------------------------------------------
  // 7. DW alignment (drop `shift` leading DWs of every CplD payload)
  // ---------------------------------------------------------------------------
  logic [127:0] dws_tdata;
  logic [15:0]  dws_tkeep;
  logic         dws_tlast, dws_tvalid, dws_tready;

  dw_shift u_dws (
    .clk(clk), .rst(rst),
    .s_tdata(ret_tdata), .s_tkeep(ret_tkeep), .s_tlast(ret_tlast),
    .s_tvalid(ret_tvalid), .s_tready(dws_s_tready),
    .s_shift(ret_tuser), .s_dir(1'b0),
    .m_tdata(dws_tdata), .m_tkeep(dws_tkeep), .m_tlast(dws_tlast),
    .m_tvalid(dws_tvalid), .m_tready(dws_tready));

  // ---------------------------------------------------------------------------
  // 8. Merger: header beat, then payload beats, into the output FIFO
  // ---------------------------------------------------------------------------
  typedef enum logic {M_HDR = 1'b0, M_PAY = 1'b1} mstate_t;
  mstate_t mstate;

  logic         outf_s_tready, outf_s_tvalid, outf_s_tlast;
  logic [127:0] outf_s_tdata;
  logic [15:0]  outf_s_tkeep;
  logic         hdr_go;

  always_comb begin
    hdr_go        = hdrf_m_tvalid && (hdrf_m_tlast || dws_tvalid) && outf_s_tready;
    outf_s_tvalid = 1'b0;
    outf_s_tdata  = hdrf_m_tdata;
    outf_s_tkeep  = 16'hFFFF;
    outf_s_tlast  = hdrf_m_tlast;
    hdrf_m_tready = 1'b0;
    dws_tready    = 1'b0;
    case (mstate)
      M_HDR: begin
        outf_s_tvalid = hdr_go;
        hdrf_m_tready = hdr_go;
      end
      M_PAY: begin
        outf_s_tvalid = dws_tvalid;
        outf_s_tdata  = dws_tdata;
        outf_s_tkeep  = dws_tkeep;
        outf_s_tlast  = dws_tlast;
        dws_tready    = outf_s_tready;
      end
    endcase
  end

  always_ff @(posedge clk) begin
    if (rst) mstate <= M_HDR;
    else begin
      case (mstate)
        M_HDR: if (hdr_go && !hdrf_m_tlast) mstate <= M_PAY;
        M_PAY: if (dws_tvalid && outf_s_tready && dws_tlast) mstate <= M_HDR;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // 9. Output FIFO
  // ---------------------------------------------------------------------------
  axis_fifo_sync #(.W(128), .UW(2), .DEPTH(OUT_DEPTH)) u_outf (
    .clk(clk), .rst(rst),
    .s_tdata(outf_s_tdata), .s_tkeep(outf_s_tkeep), .s_tlast(outf_s_tlast), .s_tuser(2'b00),
    .s_tvalid(outf_s_tvalid), .s_tready(outf_s_tready),
    .m_tdata(m_cpl_tdata), .m_tkeep(m_cpl_tkeep), .m_tlast(m_cpl_tlast), .m_tuser(m_cpl_tuser),
    .m_tvalid(m_cpl_tvalid), .m_tready(m_cpl_tready),
    .count(), .almost_full());

endmodule
