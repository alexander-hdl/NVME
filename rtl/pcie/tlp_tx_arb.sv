// =============================================================================
// tlp_tx_arb.sv — two-input packet arbiter for the TX TLP stream (ARCHITECTURE §5.4).
//
// Inputs : s_cpl_* (completer, normally has priority), s_req_* (requester).
// Output : one tlp_axis, packets are never interleaved.
//
// Policy (evaluated only at a packet boundary):
//   * only one input has a packet pending -> it wins;
//   * both pending -> the completer wins, UNLESS the completer also won the
//     previous packet, in which case the requester goes once (so doorbells /
//     config requests cannot be starved by a continuous CplD stream).
// Once a packet is granted the grant is locked until its tlast beat has been
// transferred (the source may insert gaps mid-packet).
//
// Zero-bubble: the selection is combinational on the inputs' tvalid, so the
// first beat of the next packet follows the last beat of the current one with
// no idle cycle.  An output axis_skid (OUT_SKID=1) registers the output and
// keeps the sources' tready independent of the downstream tready path while
// still sustaining one beat per cycle.
// =============================================================================
module tlp_tx_arb #(
  parameter bit OUT_SKID = 1'b1
) (
  input  logic         clk,
  input  logic         rst,
  // completer input
  input  logic [127:0] s_cpl_tdata,
  input  logic [15:0]  s_cpl_tkeep,
  input  logic         s_cpl_tlast,
  input  logic [1:0]   s_cpl_tuser,
  input  logic         s_cpl_tvalid,
  output logic         s_cpl_tready,
  // requester input
  input  logic [127:0] s_req_tdata,
  input  logic [15:0]  s_req_tkeep,
  input  logic         s_req_tlast,
  input  logic [1:0]   s_req_tuser,
  input  logic         s_req_tvalid,
  output logic         s_req_tready,
  // output tlp_axis
  output logic [127:0] m_tlp_tdata,
  output logic [15:0]  m_tlp_tkeep,
  output logic         m_tlp_tlast,
  output logic [1:0]   m_tlp_tuser,
  output logic         m_tlp_tvalid,
  input  logic         m_tlp_tready
);

  logic locked_q;        // a packet is in progress
  logic lock_req_q;      // ... and it belongs to the requester
  logic last_cpl_q;      // the previous packet was a completer packet

  logic pick_req, sel_req;
  logic a_valid, a_ready, a_last;
  logic [127:0] a_tdata;
  logic [15:0]  a_tkeep;
  logic [1:0]   a_tuser;

  // choice at a packet boundary
  assign pick_req = (s_cpl_tvalid && s_req_tvalid) ? last_cpl_q : s_req_tvalid;
  assign sel_req  = locked_q ? lock_req_q : pick_req;

  assign a_valid = sel_req ? s_req_tvalid : s_cpl_tvalid;
  assign a_tdata = sel_req ? s_req_tdata  : s_cpl_tdata;
  assign a_tkeep = sel_req ? s_req_tkeep  : s_cpl_tkeep;
  assign a_last  = sel_req ? s_req_tlast  : s_cpl_tlast;
  assign a_tuser = sel_req ? s_req_tuser  : s_cpl_tuser;

  assign s_cpl_tready = !sel_req && a_ready;
  assign s_req_tready =  sel_req && a_ready;

  always_ff @(posedge clk) begin
    if (rst) begin
      locked_q   <= 1'b0;
      lock_req_q <= 1'b0;
      last_cpl_q <= 1'b0;
    end else if (a_valid && a_ready) begin
      if (a_last) begin
        locked_q   <= 1'b0;
        last_cpl_q <= !sel_req;
      end else begin
        locked_q   <= 1'b1;
        lock_req_q <= sel_req;
      end
    end
  end

  generate
    if (OUT_SKID) begin : g_skid
      axis_skid #(.W(128), .UW(2)) u_out_skid (
        .clk(clk), .rst(rst),
        .s_tdata(a_tdata), .s_tkeep(a_tkeep), .s_tlast(a_last), .s_tuser(a_tuser),
        .s_tvalid(a_valid), .s_tready(a_ready),
        .m_tdata(m_tlp_tdata), .m_tkeep(m_tlp_tkeep), .m_tlast(m_tlp_tlast), .m_tuser(m_tlp_tuser),
        .m_tvalid(m_tlp_tvalid), .m_tready(m_tlp_tready));
    end else begin : g_direct
      assign m_tlp_tdata  = a_tdata;
      assign m_tlp_tkeep  = a_tkeep;
      assign m_tlp_tlast  = a_last;
      assign m_tlp_tuser  = a_tuser;
      assign m_tlp_tvalid = a_valid;
      assign a_ready      = m_tlp_tready;
    end
  endgenerate

endmodule
