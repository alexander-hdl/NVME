// =============================================================================
// vrf_tap.sv — read-data tap of the core: user_clk register + async FIFO to ui_clk (WP6).
//
// Domains : user_clk (uclk/urst: tap inputs, register, FIFO write), ui_clk (vclk/vrst: FIFO read,
//           m_*, ovf).  Spec: STAND/RECORDER_SPEC.md 3.3, 4.3 (X15), 4.4 (WP6), 8.2.
// Ports   : wr_en/wr_beat_addr/wr_data/wr_be/slot_lba_flat/enable = core tap ports (user).
//           m_tdata = {lba32[192:161], off17[160:144], be16[143:128], data128[127:0]},
//           AXI-S, FWFT (valid does not depend on ready) // ui.
//           ovf = sticky (ui): a beat could not be stored (FIFO full).
// user side: ONE register stage (slot select of slot_lba as in stage S1 of verify_checker; the
//           table entry changes only after the previous command of the slot has finished) and the
//           FIFO write.  Nothing else in user_clk.
// Overflow: sticky flag in user_clk (set when the FIFO was full for a registered beat, held
//           while enable=1, cleared when enable=0 i.e. when no READ is running) -> cdc_sync
//           (sticky level, single bit) -> ovf in ui_clk.  The beat is lost (FIFO full), the
//           verification result is invalid (rec_verify copies the flag into res.ovf).
// =============================================================================
module vrf_tap #(
  parameter int N_SLOTS    = 4,
  parameter int SLOT_BYTES = 131072          // power of 2, <= 128 KiB (off17)
) (
  input  logic                   uclk,                // user
  input  logic                   urst,                // user
  input  logic                   wr_en,               // user
  input  logic [31:0]            wr_beat_addr,        // user, DATA-relative, 16 B aligned
  input  logic [127:0]           wr_data,             // user
  input  logic [15:0]            wr_be,               // user
  input  logic [N_SLOTS*64-1:0]  slot_lba_flat,       // user
  input  logic                   enable,              // user, verify_mode (quasi-static)

  input  logic                   vclk,                // ui
  input  logic                   vrst,                // ui
  output logic [192:0]           m_tdata,             // ui
  output logic                   m_tvalid,            // ui
  input  logic                   m_tready,            // ui
  output logic                   ovf                  // ui, sticky
);

  localparam int OFFW = $clog2(SLOT_BYTES);
  localparam int SW   = (N_SLOTS > 1) ? $clog2(N_SLOTS) : 1;
  if (OFFW > 17) begin : g_chk
    $error("vrf_tap: SLOT_BYTES (%0d) exceeds 128 KiB (off17)", SLOT_BYTES);
  end

  // ------------------------------------------------------------ user side
  logic [SW-1:0] slot_i;
  logic [31:0]   lba_sel;
  assign slot_i = wr_beat_addr[OFFW +: SW];
  always_comb begin
    lba_sel = '0;
    for (int k = 0; k < N_SLOTS; k++)
      if (slot_i == SW'(k)) lba_sel = slot_lba_flat[64*k +: 32];
  end

  logic         v_q;
  logic [192:0] d_q;
  logic         ovf_u_q;
  logic         s_tready;

  always_ff @(posedge uclk) begin
    if (urst) v_q <= 1'b0;
    else      v_q <= wr_en & enable;
    d_q <= {lba_sel, 17'(wr_beat_addr[OFFW-1:0]), wr_be, wr_data};
  end

  always_ff @(posedge uclk) begin
    if (urst || !enable)            ovf_u_q <= 1'b0;
    else if (v_q && !s_tready)      ovf_u_q <= 1'b1;
  end

  axis_fifo_async #(.W(193), .DEPTH(512)) u_axis_fifo_async_tap (
    .s_clk(uclk), .s_rst(urst), .s_tdata(d_q), .s_tvalid(v_q), .s_tready(s_tready),
    .m_clk(vclk), .m_rst(vrst), .m_tdata(m_tdata), .m_tvalid(m_tvalid), .m_tready(m_tready)
  );

  // sticky single-bit level user -> ui
  cdc_sync #(.W(1), .STAGES(2)) u_cdc_sync_ovf (.clk(vclk), .d(ovf_u_q), .q(ovf));

endmodule
