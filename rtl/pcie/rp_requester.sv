// =============================================================================
// rp_requester.sv - Root Port transaction issuer (ARCHITECTURE.md 5.10)
// -----------------------------------------------------------------------------
// Turns a simple command port into TLPs on the internal "tlp_axis" stream
// (ARCHITECTURE.md 3) and matches the returning completions.
//
//   command port   : req_valid/req_ready, req_op, req_addr, req_wdata, req_be
//   response port  : rsp_valid (one pulse per command), rsp_status, rsp_rdata
//   TLP out        : m_req_*  (to tlp_tx_arb -> TX adapter)
//   completions in : s_cpl_*  (from tlp_rx_demux)
//
// Operations (req_op):
//   0 CFGRD0 : Type-0 config read.   req_addr = H2 layout of the config request:
//              {bus[7:0], dev[4:0], fn[2:0], 4'b0, extreg[3:0], reg[5:0], 2'b0}
//   1 CFGWR0 : Type-0 config write, one payload DW (req_wdata), req_be = first BE
//   2 MRD32  : memory read, 1 DW, req_addr = byte address, req_be = first BE
//   3 MWR32  : memory write, 1 DW (posted); rsp_valid right after the packet left
//
// Rules implemented
//   * one NON-POSTED request outstanding at a time (CfgRd, CfgWr, MRd);
//   * the 8-bit tag increments once per non-posted request;
//   * a completion matches when it is a Cpl/CplD, ReqID == requester_id and
//     Tag == tag of the request in flight;
//   * anything else arriving on s_cpl (wrong tag, late after a timeout, ...)
//     is consumed and counted in unexpected_cpl_cnt (saturating);
//   * s_cpl_tready is tied to 1: completions are never back-pressured;
//   * the response is issued when the LAST beat of the matching packet has been
//     consumed, so a new command can never overlap the tail of an old packet.
//
// rsp_status: 0 SC, 1 UR, 2 CRS, 3 CA, 4 TIMEOUT, 5 UNEXPECTED
//   UNEXPECTED = completion with the right tag but a wrong shape (SC read
//   without exactly one data DW, SC write carrying data, reserved status code,
//   poisoned/ECRC-error flag on tuser).
//
// The payload DW is little-endian ("memory order") on this interface; the TX
// adapter byte-swaps it for the 7-series core, the RX adapter swaps it back.
// =============================================================================
module rp_requester
  import nvme_host_pkg::*;
(
  input  logic         clk,
  input  logic         rst,

  // ---- command port ----
  input  logic         req_valid,
  output logic         req_ready,
  input  logic [1:0]   req_op,
  input  logic [31:0]  req_addr,
  input  logic [31:0]  req_wdata,
  input  logic [3:0]   req_be,
  output logic         rsp_valid,
  output logic [2:0]   rsp_status,
  output logic [31:0]  rsp_rdata,
  input  logic [31:0]  timeout_cycles,

  // ---- TLP stream to the TX arbiter ----
  output logic [127:0] m_req_tdata,
  output logic [15:0]  m_req_tkeep,
  output logic         m_req_tlast,
  output logic         m_req_tvalid,
  output logic [1:0]   m_req_tuser,
  input  logic         m_req_tready,

  // ---- completions from the RX demux ----
  input  logic [127:0] s_cpl_tdata,
  input  logic [15:0]  s_cpl_tkeep,
  input  logic         s_cpl_tlast,
  input  logic         s_cpl_tvalid,
  input  logic [1:0]   s_cpl_tuser,
  output logic         s_cpl_tready,

  input  logic [15:0]  requester_id,
  output logic [15:0]  unexpected_cpl_cnt
);

  // ---------------------------------------------------------------------------
  // Command / status encodings (shared with req_mux and pcie_enum)
  // ---------------------------------------------------------------------------
  localparam logic [1:0] OP_CFGRD0 = 2'd0;
  localparam logic [1:0] OP_CFGWR0 = 2'd1;
  localparam logic [1:0] OP_MRD32  = 2'd2;
  localparam logic [1:0] OP_MWR32  = 2'd3;

  localparam logic [2:0] RSP_SC         = 3'd0;
  localparam logic [2:0] RSP_UR         = 3'd1;
  localparam logic [2:0] RSP_CRS        = 3'd2;
  localparam logic [2:0] RSP_CA         = 3'd3;
  localparam logic [2:0] RSP_TIMEOUT    = 3'd4;
  localparam logic [2:0] RSP_UNEXPECTED = 3'd5;

  localparam logic [9:0]  ONE_DW       = 10'd1;
  localparam logic [15:0] KEEP_ALL     = 16'hFFFF;   // header beat
  localparam logic [15:0] KEEP_ONE_DW  = 16'h000F;   // payload beat with a single DW

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    S_IDLE,       // waiting for a command
    S_HDR,        // driving the header beat
    S_DATA,       // driving the payload beat (CfgWr / MWr)
    S_WAIT_CPL,   // non-posted request sent, waiting for the completion
    S_RSP         // one-cycle response pulse
  } state_t;

  state_t        state;
  logic [1:0]    op_q;             // latched operation
  logic [127:0]  hdr_q;            // header beat (H3..H0)
  logic [31:0]   wdata_q;          // payload DW
  logic [7:0]    tag_q;            // tag for the NEXT non-posted request
  logic [7:0]    exp_tag_q;        // tag of the request in flight
  logic [31:0]   to_cnt;           // timeout down-counter
  logic [2:0]    rsp_status_q;
  logic [31:0]   rsp_rdata_q;

  wire op_is_write = op_q[0];                  // CFGWR0 / MWR32
  wire op_is_posted = (op_q == OP_MWR32);

  // Completion packet tracking (persists across beats of one packet)
  logic          cpl_mid_q;        // inside a completion packet (beat >= 1 next)
  logic          cpl_match_q;      // this packet matched the request in flight
  logic          pkt_pl_q;         // first payload beat already captured
  logic          pkt_bad_q;        // wrong shape / poisoned
  logic [2:0]    pkt_status_q;     // mapped status of the packet header
  logic [31:0]   pkt_rdata_q;

  // ---------------------------------------------------------------------------
  // Completion decoding (combinational view of the beat on s_cpl)
  // ---------------------------------------------------------------------------
  assign s_cpl_tready = 1'b1;

  function automatic logic [2:0] map_status(input logic [2:0] cs);
    case (cs)
      CPL_SC:  return RSP_SC;
      CPL_UR:  return RSP_UR;
      CPL_CRS: return RSP_CRS;
      CPL_CA:  return RSP_CA;
      default: return RSP_UNEXPECTED;    // reserved encodings
    endcase
  endfunction

  logic [31:0] c_h0, c_h1, c_h2;
  assign c_h0 = s_cpl_tdata[31:0];
  assign c_h1 = s_cpl_tdata[63:32];
  assign c_h2 = s_cpl_tdata[95:64];

  logic        c_beat0;            // current s_cpl beat is the header beat
  logic        c_hdr_match;        // header beat matches the request in flight
  logic        c_match;            // this beat belongs to the matching packet
  logic        c_expects_data;     // request is a read -> CplD with 1 DW expected
  logic [2:0]  c_status;           // packet status after this beat
  logic        c_bad;              // packet shape error after this beat
  logic        c_pl;               // payload captured after this beat
  logic [31:0] c_rdata;            // data after this beat
  logic        c_finish;           // matching packet complete -> respond

  always_comb begin
    c_beat0        = !cpl_mid_q;
    c_expects_data = !op_is_write;
    c_hdr_match    = s_cpl_tvalid && c_beat0 && (state == S_WAIT_CPL) &&
                     tlp_is_cpl(c_h0) &&
                     (cpl_reqid(c_h2) == requester_id) &&
                     (cpl_tag(c_h2)   == exp_tag_q);
    c_match        = c_beat0 ? c_hdr_match : cpl_match_q;

    if (c_beat0) begin
      c_status = map_status(cpl_status(c_h1));
      c_pl     = 1'b0;
      c_rdata  = 32'h0;
      c_bad    = 1'b0;
      if (cpl_status(c_h1) == CPL_SC) begin
        // successful completion: reads need a CplD with Length 1, writes a bare Cpl
        if (c_expects_data) c_bad = !tlp_has_data(c_h0) || (tlp_len_dw(c_h0) != ONE_DW);
        else                c_bad = tlp_has_data(c_h0);
      end
    end else begin
      c_status = pkt_status_q;
      c_bad    = pkt_bad_q;
      c_pl     = pkt_pl_q;
      c_rdata  = pkt_rdata_q;
      if (!pkt_pl_q) begin                       // first payload beat: DW0 is the data
        c_pl    = 1'b1;
        c_rdata = s_cpl_tdata[31:0];
      end
    end
    if (s_cpl_tuser != 2'b00) c_bad = 1'b1;      // err_fwd / ecrc_err

    c_finish = s_cpl_tvalid && s_cpl_tlast && c_match;
  end

  // A matching SC read whose packet ended before any payload beat is malformed.
  logic c_final_bad;
  assign c_final_bad = c_bad ||
                       ((c_status == RSP_SC) && c_expects_data && !c_pl);

  // ---------------------------------------------------------------------------
  // Main sequential logic
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      state              <= S_IDLE;
      op_q               <= OP_CFGRD0;
      hdr_q              <= '0;
      wdata_q            <= '0;
      tag_q              <= '0;
      exp_tag_q          <= '0;
      to_cnt             <= '0;
      rsp_status_q       <= RSP_SC;
      rsp_rdata_q        <= '0;
      cpl_mid_q          <= 1'b0;
      cpl_match_q        <= 1'b0;
      pkt_pl_q           <= 1'b0;
      pkt_bad_q          <= 1'b0;
      pkt_status_q       <= RSP_SC;
      pkt_rdata_q        <= '0;
      unexpected_cpl_cnt <= '0;
    end else begin
      // ---------------- completion packet bookkeeping ----------------
      if (s_cpl_tvalid) begin
        cpl_mid_q <= !s_cpl_tlast;
        if (s_cpl_tlast)  cpl_match_q <= 1'b0;          // packet over
        else if (c_beat0) cpl_match_q <= c_hdr_match;   // packet continues
        if (c_beat0 && !c_hdr_match && (unexpected_cpl_cnt != 16'hFFFF))
          unexpected_cpl_cnt <= unexpected_cpl_cnt + 16'd1;
        pkt_status_q <= c_status;
        pkt_bad_q    <= c_bad;
        pkt_pl_q     <= c_pl;
        pkt_rdata_q  <= c_rdata;
      end

      // ---------------- command FSM ----------------
      case (state)
        S_IDLE: begin
          if (req_valid) begin
            op_q    <= req_op;
            wdata_q <= req_wdata;
            case (req_op)
              OP_CFGRD0: hdr_q <= hdr_cfg0(1'b0, req_addr[31:16], req_addr[11:2],
                                           requester_id, tag_q, req_be);
              OP_CFGWR0: hdr_q <= hdr_cfg0(1'b1, req_addr[31:16], req_addr[11:2],
                                           requester_id, tag_q, req_be);
              OP_MRD32:  hdr_q <= hdr_mrd32(req_addr, ONE_DW, requester_id, tag_q, req_be, 4'h0);
              default:   hdr_q <= hdr_mwr32(req_addr, ONE_DW, requester_id, tag_q, req_be, 4'h0);
            endcase
            if (req_op != OP_MWR32) begin        // non-posted: this tag is now in use
              exp_tag_q <= tag_q;
              tag_q     <= tag_q + 8'd1;
            end
            state <= S_HDR;
          end
        end

        S_HDR: begin
          if (m_req_tready) begin
            if (op_is_write)       state <= S_DATA;
            else begin
              to_cnt <= timeout_cycles;
              state  <= S_WAIT_CPL;
            end
          end
        end

        S_DATA: begin
          if (m_req_tready) begin
            if (op_is_posted) begin               // MWr: done once the packet is sent
              rsp_status_q <= RSP_SC;
              rsp_rdata_q  <= 32'h0;
              state        <= S_RSP;
            end else begin                        // CfgWr: wait for the Cpl
              to_cnt <= timeout_cycles;
              state  <= S_WAIT_CPL;
            end
          end
        end

        S_WAIT_CPL: begin
          if (c_finish) begin
            rsp_status_q <= c_final_bad ? RSP_UNEXPECTED : c_status;
            rsp_rdata_q  <= (!c_final_bad && c_status == RSP_SC && c_expects_data) ? c_rdata : 32'h0;
            state        <= S_RSP;
          end else if (!c_match && !cpl_match_q) begin
            // no matching packet in progress: run the timeout
            if (to_cnt == 32'd0) begin
              rsp_status_q <= RSP_TIMEOUT;
              rsp_rdata_q  <= 32'h0;
              state        <= S_RSP;
            end else begin
              to_cnt <= to_cnt - 32'd1;
            end
          end
        end

        S_RSP: state <= S_IDLE;

        default: state <= S_IDLE;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Outputs
  // ---------------------------------------------------------------------------
  assign req_ready  = (state == S_IDLE);
  assign rsp_valid  = (state == S_RSP);
  assign rsp_status = rsp_status_q;
  assign rsp_rdata  = rsp_rdata_q;

  assign m_req_tvalid = (state == S_HDR) || (state == S_DATA);
  assign m_req_tdata  = (state == S_DATA) ? {96'h0, wdata_q} : hdr_q;
  assign m_req_tkeep  = (state == S_DATA) ? KEEP_ONE_DW : KEEP_ALL;
  assign m_req_tlast  = (state == S_DATA) || ((state == S_HDR) && !op_is_write);
  assign m_req_tuser  = 2'b00;

endmodule
