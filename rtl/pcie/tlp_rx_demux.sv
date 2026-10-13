// =============================================================================
// tlp_rx_demux.sv — route internal-format RX TLPs by Fmt/Type (ARCHITECTURE §5.3).
//
//   Cpl / CplD        -> m_cpl_*
//   MRd32 / MRd64     -> m_mrd_*
//   MWr32 / MWr64     -> m_mwr_*   (poisoned MWr, tuser[0]=1, is dropped)
//   anything else (Msg, Cfg type 1, unknown)           -> dropped
//
// The decision is taken on beat 0 (header beat, H0 in tdata[31:0]) and latched
// in route_q for the remaining beats of the packet.  Routing is purely
// combinational (no extra latency, no bubble): tvalid of the selected output
// follows s_tlp_tvalid, s_tlp_tready follows the selected output's tready; a
// dropped packet is consumed at full rate.  drop_cnt_inc is a registered one-clock
// pulse per dropped TLP.
// =============================================================================
module tlp_rx_demux (
  input  logic         clk,
  input  logic         rst,
  // input tlp_axis
  input  logic [127:0] s_tlp_tdata,
  input  logic [15:0]  s_tlp_tkeep,
  input  logic         s_tlp_tlast,
  input  logic [1:0]   s_tlp_tuser,
  input  logic         s_tlp_tvalid,
  output logic         s_tlp_tready,
  // completions
  output logic [127:0] m_cpl_tdata,
  output logic [15:0]  m_cpl_tkeep,
  output logic         m_cpl_tlast,
  output logic [1:0]   m_cpl_tuser,
  output logic         m_cpl_tvalid,
  input  logic         m_cpl_tready,
  // memory reads
  output logic [127:0] m_mrd_tdata,
  output logic [15:0]  m_mrd_tkeep,
  output logic         m_mrd_tlast,
  output logic [1:0]   m_mrd_tuser,
  output logic         m_mrd_tvalid,
  input  logic         m_mrd_tready,
  // memory writes
  output logic [127:0] m_mwr_tdata,
  output logic [15:0]  m_mwr_tkeep,
  output logic         m_mwr_tlast,
  output logic [1:0]   m_mwr_tuser,
  output logic         m_mwr_tvalid,
  input  logic         m_mwr_tready,
  // statistics
  output logic         drop_cnt_inc
);

  import nvme_host_pkg::*;

  typedef enum logic [1:0] {RT_CPL = 2'd0, RT_MRD = 2'd1, RT_MWR = 2'd2, RT_DROP = 2'd3} route_t;

  logic   first_q;                 // next beat is the header beat of a packet
  route_t route_q;                 // route of the packet in progress
  route_t route;                   // route of the beat being offered
  route_t first_route;

  localparam int TU_ERR_FWD = 0;

  always_comb begin
    logic [31:0] h0;
    h0 = s_tlp_tdata[31:0];
    if      (tlp_is_cpl(h0)) first_route = RT_CPL;
    else if (tlp_is_mrd(h0)) first_route = RT_MRD;
    else if (tlp_is_mwr(h0)) first_route = s_tlp_tuser[TU_ERR_FWD] ? RT_DROP : RT_MWR;
    else                     first_route = RT_DROP;
  end

  assign route = first_q ? first_route : route_q;

  // payload pass-through (same data on all three outputs, only tvalid differs)
  assign m_cpl_tdata = s_tlp_tdata;  assign m_cpl_tkeep = s_tlp_tkeep;
  assign m_cpl_tlast = s_tlp_tlast;  assign m_cpl_tuser = s_tlp_tuser;
  assign m_mrd_tdata = s_tlp_tdata;  assign m_mrd_tkeep = s_tlp_tkeep;
  assign m_mrd_tlast = s_tlp_tlast;  assign m_mrd_tuser = s_tlp_tuser;
  assign m_mwr_tdata = s_tlp_tdata;  assign m_mwr_tkeep = s_tlp_tkeep;
  assign m_mwr_tlast = s_tlp_tlast;  assign m_mwr_tuser = s_tlp_tuser;

  assign m_cpl_tvalid = s_tlp_tvalid && (route == RT_CPL);
  assign m_mrd_tvalid = s_tlp_tvalid && (route == RT_MRD);
  assign m_mwr_tvalid = s_tlp_tvalid && (route == RT_MWR);

  always_comb begin
    case (route)
      RT_CPL:  s_tlp_tready = m_cpl_tready;
      RT_MRD:  s_tlp_tready = m_mrd_tready;
      RT_MWR:  s_tlp_tready = m_mwr_tready;
      default: s_tlp_tready = 1'b1;               // drop: swallow the packet
    endcase
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      first_q      <= 1'b1;
      drop_cnt_inc <= 1'b0;
    end else begin
      drop_cnt_inc <= 1'b0;
      if (s_tlp_tvalid && s_tlp_tready) begin
        if (first_q) begin
          route_q <= route;
          if (route == RT_DROP) drop_cnt_inc <= 1'b1;
        end
        first_q <= s_tlp_tlast;
      end
    end
  end

endmodule
