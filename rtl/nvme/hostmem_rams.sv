// =============================================================================
// hostmem_rams.sv — the five small "host memory" BRAMs (ARCHITECTURE.md §5.7)
//
//   RAM       beats  written by                          read by
//   asq_ram    128   nvme_admin (SQE writes)             completer (RSEL_ASQ)
//   iosq_ram   256   nvme_write_engine (SQE writes)      completer (RSEL_IOSQ)
//   acq_ram     32   hostmem_writer, CQ clear FSM        nvme_admin  (acq_rd_*)
//   iocq_ram    64   hostmem_writer, CQ clear FSM        nvme_write_engine (iocq_rd_*)
//   ident_ram  256   hostmem_writer                      nvme_admin  (ident_rd_*)
//
// All RAMs are 128 bit wide (one 16-byte beat per word) with byte-enable write
// and a 2-cycle read latency (sdp_ram contract).
//
// The completer's single read port (cpl_rd_*) is multiplexed here:
//   RSEL_ASQ / RSEL_IOSQ -> the two SQ RAMs,
//   RSEL_PRP             -> prp_synth (synthesised PRP list entries),
//   RSEL_DATA            -> NOT served here: cpl_rd_en_data is exposed for the
//                           data_ring (the integrator wires cpl_rd_addr to the
//                           data_ring read port as well) and data_rd_data comes
//                           back in. The select is delayed by 2 cycles to steer
//                           the read-data mux, so all four sources look alike.
//
// CQ clear: a rising edge on `clear_cqs` starts a 64-cycle sequence that writes
// zeros (all byte enables) into every acq_ram and iocq_ram word. clear_done is
// low from the cycle after the request until the sequence has finished and
// stays high until the next request (it is low after reset, until the first
// clear completes). Clear has priority over the writer port: MWr beats arriving
// while the clear runs are lost (nothing may be in flight before CC.EN).
// =============================================================================
module hostmem_rams
  import nvme_host_pkg::*;
#(
  parameter int unsigned N_SLOTS    = N_SLOTS_DEF,
  parameter int unsigned SLOT_BYTES = SLOT_BYTES_DEF
)(
  input  logic         clk,
  input  logic         rst,
  // ASQ write port (nvme_admin)
  input  logic         asq_wr_en,
  input  logic [6:0]   asq_wr_addr,
  input  logic [127:0] asq_wr_data,
  // IOSQ write port (nvme_write_engine)
  input  logic         iosq_wr_en,
  input  logic [7:0]   iosq_wr_addr,
  input  logic [127:0] iosq_wr_data,
  // writer port (hostmem_writer)
  input  logic         hw_wr_en_ident,
  input  logic         hw_wr_en_acq,
  input  logic         hw_wr_en_iocq,
  input  logic [31:0]  hw_wr_beat_addr,
  input  logic [127:0] hw_wr_data,
  input  logic [15:0]  hw_wr_be,
  // CQ clear
  input  logic         clear_cqs,
  output logic         clear_done,
  // completer read port
  input  logic         cpl_rd_en,
  input  logic [1:0]   cpl_rd_sel,
  input  logic [31:0]  cpl_rd_addr,
  output logic [127:0] cpl_rd_data,
  output logic         cpl_rd_en_data,   // read request for the data_ring
  input  logic [127:0] data_rd_data,     // data_ring read data (2-cycle latency)
  // ACQ / IOCQ / Identify read ports (nvme_admin, nvme_write_engine)
  input  logic         acq_rd_en,
  input  logic [4:0]   acq_rd_addr,
  output logic [127:0] acq_rd_data,
  input  logic         iocq_rd_en,
  input  logic [5:0]   iocq_rd_addr,
  output logic [127:0] iocq_rd_data,
  input  logic         ident_rd_en,
  input  logic [7:0]   ident_rd_addr,
  output logic [127:0] ident_rd_data
);

  localparam logic [15:0] BE_ALL = 16'hFFFF;

  // ---------------------------------------------------------------------------
  // CQ clear FSM
  // ---------------------------------------------------------------------------
  localparam int unsigned CLR_LAST = 63;                 // iocq has 64 beats (acq 32)

  logic        clr_busy, clear_q;
  logic [5:0]  clr_idx;

  wire clr_start = clear_cqs && !clear_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      clr_busy   <= 1'b0;
      clr_idx    <= '0;
      clear_done <= 1'b0;
      clear_q    <= 1'b0;
    end else begin
      clear_q <= clear_cqs;
      if (clr_start) begin
        clr_busy   <= 1'b1;
        clr_idx    <= '0;
        clear_done <= 1'b0;
      end else if (clr_busy) begin
        if (clr_idx == 6'(CLR_LAST)) begin
          clr_busy   <= 1'b0;
          clear_done <= 1'b1;
        end
        clr_idx <= clr_idx + 6'd1;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Write-port muxes for the CQ RAMs (clear FSM has priority)
  // ---------------------------------------------------------------------------
  logic         acq_wr_en,  iocq_wr_en, ident_wr_en;
  logic [4:0]   acq_wr_addr;
  logic [5:0]   iocq_wr_addr;
  logic [127:0] cq_wr_data;
  logic [15:0]  cq_wr_be;

  always_comb begin
    if (clr_busy) begin
      acq_wr_en    = (clr_idx < 6'd32);
      iocq_wr_en   = 1'b1;
      acq_wr_addr  = clr_idx[4:0];
      iocq_wr_addr = clr_idx;
      cq_wr_data   = '0;
      cq_wr_be     = BE_ALL;
    end else begin
      acq_wr_en    = hw_wr_en_acq;
      iocq_wr_en   = hw_wr_en_iocq;
      acq_wr_addr  = hw_wr_beat_addr[8:4];
      iocq_wr_addr = hw_wr_beat_addr[9:4];
      cq_wr_data   = hw_wr_data;
      cq_wr_be     = hw_wr_be;
    end
    ident_wr_en = hw_wr_en_ident && !clr_busy;
  end

  // ---------------------------------------------------------------------------
  // PRP synthesiser and completer read steering
  // ---------------------------------------------------------------------------
  logic [127:0] asq_rd_data, iosq_rd_data, prp_rd_data;

  wire rd_asq  = cpl_rd_en && (cpl_rd_sel == RSEL_ASQ);
  wire rd_iosq = cpl_rd_en && (cpl_rd_sel == RSEL_IOSQ);
  wire rd_prp  = cpl_rd_en && (cpl_rd_sel == RSEL_PRP);
  assign cpl_rd_en_data = cpl_rd_en && (cpl_rd_sel == RSEL_DATA);

  logic [1:0] sel_d1, sel_d2;
  always_ff @(posedge clk) begin
    sel_d1 <= cpl_rd_sel;
    sel_d2 <= sel_d1;
  end

  always_comb begin
    case (sel_d2)
      RSEL_ASQ:  cpl_rd_data = asq_rd_data;
      RSEL_IOSQ: cpl_rd_data = iosq_rd_data;
      RSEL_PRP:  cpl_rd_data = prp_rd_data;
      default:   cpl_rd_data = data_rd_data;
    endcase
  end

  prp_synth #(.N_SLOTS(N_SLOTS), .SLOT_BYTES(SLOT_BYTES)) u_prp (
    .clk(clk), .rd_en(rd_prp), .rd_addr(cpl_rd_addr), .rd_data(prp_rd_data));

  // ---------------------------------------------------------------------------
  // The five RAMs
  // ---------------------------------------------------------------------------
  sdp_ram #(.DEPTH(ASQ_BYTES/16), .W(128)) u_asq (
    .clk(clk),
    .wr_en(asq_wr_en), .wr_be(BE_ALL), .wr_addr(asq_wr_addr), .wr_data(asq_wr_data),
    .rd_en(rd_asq), .rd_addr(cpl_rd_addr[10:4]), .rd_data(asq_rd_data));

  sdp_ram #(.DEPTH(IOSQ_BYTES/16), .W(128)) u_iosq (
    .clk(clk),
    .wr_en(iosq_wr_en), .wr_be(BE_ALL), .wr_addr(iosq_wr_addr), .wr_data(iosq_wr_data),
    .rd_en(rd_iosq), .rd_addr(cpl_rd_addr[11:4]), .rd_data(iosq_rd_data));

  sdp_ram #(.DEPTH(ACQ_BYTES/16), .W(128)) u_acq (
    .clk(clk),
    .wr_en(acq_wr_en), .wr_be(cq_wr_be), .wr_addr(acq_wr_addr), .wr_data(cq_wr_data),
    .rd_en(acq_rd_en), .rd_addr(acq_rd_addr), .rd_data(acq_rd_data));

  sdp_ram #(.DEPTH(IOCQ_BYTES/16), .W(128)) u_iocq (
    .clk(clk),
    .wr_en(iocq_wr_en), .wr_be(cq_wr_be), .wr_addr(iocq_wr_addr), .wr_data(cq_wr_data),
    .rd_en(iocq_rd_en), .rd_addr(iocq_rd_addr), .rd_data(iocq_rd_data));

  sdp_ram #(.DEPTH(IDENT_BYTES/16), .W(128)) u_ident (
    .clk(clk),
    .wr_en(ident_wr_en), .wr_be(hw_wr_be), .wr_addr(hw_wr_beat_addr[11:4]), .wr_data(hw_wr_data),
    .rd_en(ident_rd_en), .rd_addr(ident_rd_addr), .rd_data(ident_rd_data));

endmodule
