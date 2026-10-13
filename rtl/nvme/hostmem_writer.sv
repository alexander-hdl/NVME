// =============================================================================
// hostmem_writer.sv — absorbs the SSD's Memory Write TLPs (ARCHITECTURE.md §5.6)
//
// The SSD writes completion queue entries (ACQ / IOCQ) and the Identify data
// into host memory. This module turns one MWr TLP (header beat + payload
// beats, internal tlp_axis format) into a sequence of 16-byte-aligned memory
// beats with per-byte write enables.
//
// ALGORITHM
//   header beat : decode  H0 (fmt/type, Length), H1 (FBE/LBE), H2 (address).
//                 4DW headers use the PCIe layout (H2 = addr[63:32], H3 = addr[31:2]);
//                 a non-zero upper half is outside the 32-bit map -> drop.
//                 Region decode (§2): Identify, ACQ, IOCQ — the WHOLE write must fit
//                 in one region. Anything else, a poisoned TLP (tuser[0]) or a TLP that
//                 is not a memory write is consumed and counted on drop_cnt_inc.
//   payload     : the DWs are in memory order starting at the DW address `addr`.
//                 dw_shift (right, shift = addr[3:2]) inserts `shift` empty DWs in
//                 front of DW0, so every output beat is exactly one 16-byte aligned
//                 memory beat; the first beat has tkeep=0 in the DWs before `addr`
//                 and the last beat has tkeep=0 in the DWs after the final one.
//   byte enable : wr_be[4*j+b] = (DW j present) AND
//                    first payload DW  -> First BE[b]
//                    last  payload DW  -> Last  BE[b]   (only if Length > 1)
//                    any other DW      -> 1
//                 (for Length == 1 only First BE applies, PCIe Base §2.2.5).
//   address     : wr_beat_addr = region-relative byte address of the beat,
//                 incremented by 16 per output beat.
//
// One MWr is processed at a time (header accepted only after the previous
// packet has left dw_shift): CQE traffic is a few packets per command, so the
// few idle cycles per packet are irrelevant. Outputs are registered.
// =============================================================================
module hostmem_writer
  import nvme_host_pkg::*;
(
  input  logic         clk,
  input  logic         rst,
  // MWr packets (tlp_axis)
  input  logic [127:0] s_mwr_tdata,
  input  logic [15:0]  s_mwr_tkeep,
  input  logic         s_mwr_tlast,
  input  logic [1:0]   s_mwr_tuser,
  input  logic         s_mwr_tvalid,
  output logic         s_mwr_tready,
  // memory write port, shared by the three target RAMs
  output logic         wr_en_ident,
  output logic         wr_en_acq,
  output logic         wr_en_iocq,
  output logic [31:0]  wr_beat_addr,     // region-relative byte address of the 16 B beat
  output logic [127:0] wr_data,
  output logic [15:0]  wr_be,
  output logic         drop_cnt_inc      // pulse: TLP consumed without being written
);

  // ---------------------------------------------------------------------------
  // Region decode
  // ---------------------------------------------------------------------------
  localparam logic [1:0] WSEL_IDENT = 2'd0, WSEL_ACQ = 2'd1, WSEL_IOCQ = 2'd2;

  function automatic logic fits(input logic [31:0] addr, input logic [31:0] base,
                                input logic [31:0] size, input logic [12:0] nbytes);
    logic [32:0] off;
    logic [33:0] fin;
    off = {1'b0, addr} - {1'b0, base};
    fin = {1'b0, off} + {21'b0, nbytes};
    return (!off[32]) && (fin <= {2'b00, size});
  endfunction

  // ---------------------------------------------------------------------------
  // Header decode (combinational on the incoming beat)
  // ---------------------------------------------------------------------------
  logic [31:0] h0, h1, h2, h3, hd_addr;
  logic        hd_is4dw, hd_hi_bad, hd_poison, hd_hit;
  logic [10:0] hd_len;
  logic [12:0] hd_nbytes;
  logic [1:0]  hd_sel;
  logic [31:0] hd_rel;
  logic        hd_ok;

  always_comb begin
    {h3, h2, h1, h0} = s_mwr_tdata;
    hd_is4dw  = h0[29];
    hd_addr   = hd_is4dw ? h3 : h2;
    hd_hi_bad = hd_is4dw && (h2 != 32'd0);
    hd_poison = s_mwr_tuser[0];
    hd_len    = (h0[9:0] == 10'd0) ? 11'd1024 : {1'b0, h0[9:0]};
    hd_nbytes = {hd_len, 2'b00};
    hd_hit    = 1'b1;
    hd_sel    = WSEL_IDENT;
    hd_rel    = hd_addr - IDENT_BASE;
    if (fits(hd_addr, IDENT_BASE, IDENT_BYTES, hd_nbytes)) begin
      hd_sel = WSEL_IDENT; hd_rel = hd_addr - IDENT_BASE;
    end else if (fits(hd_addr, ACQ_BASE, ACQ_BYTES, hd_nbytes)) begin
      hd_sel = WSEL_ACQ;   hd_rel = hd_addr - ACQ_BASE;
    end else if (fits(hd_addr, IOCQ_BASE, IOCQ_BYTES, hd_nbytes)) begin
      hd_sel = WSEL_IOCQ;  hd_rel = hd_addr - IOCQ_BASE;
    end else begin
      hd_hit = 1'b0;
    end
    // header-only "MWr" (tlast on the header beat) has no payload -> reject too
    hd_ok = tlp_is_mwr(h0) && !hd_poison && !hd_hi_bad && hd_hit && !s_mwr_tlast;
  end

  // ---------------------------------------------------------------------------
  // Per-packet state
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {ST_HDR, ST_PAY, ST_DROP, ST_DRAIN} state_t;
  state_t state;

  logic [1:0]  p_sel;
  logic [1:0]  p_shift;       // addr[3:2]
  logic [3:0]  p_fbe, p_lbe;
  logic        p_len1;        // Length == 1
  logic [31:0] o_addr;        // region-relative address of the next output beat
  logic        o_first;       // next output beat is the first of the packet
  logic        out_done_seen; // dw_shift delivered its last beat

  // ---------------------------------------------------------------------------
  // dw_shift (right)
  // ---------------------------------------------------------------------------
  logic         dws_s_tready;
  logic [127:0] dws_tdata;
  logic [15:0]  dws_tkeep;
  logic         dws_tlast, dws_tvalid;
  wire          dws_in_valid = (state == ST_PAY) && s_mwr_tvalid;

  dw_shift u_dws (
    .clk(clk), .rst(rst),
    .s_tdata(s_mwr_tdata), .s_tkeep(s_mwr_tkeep), .s_tlast(s_mwr_tlast),
    .s_tvalid(dws_in_valid), .s_tready(dws_s_tready),
    .s_shift(p_shift), .s_dir(1'b1),
    .m_tdata(dws_tdata), .m_tkeep(dws_tkeep), .m_tlast(dws_tlast),
    .m_tvalid(dws_tvalid), .m_tready(1'b1));

  always_comb begin
    case (state)
      ST_HDR:  s_mwr_tready = 1'b1;
      ST_PAY:  s_mwr_tready = dws_s_tready;
      ST_DROP: s_mwr_tready = 1'b1;
      default: s_mwr_tready = 1'b0;
    endcase
  end

  wire in_fire  = s_mwr_tvalid && s_mwr_tready;
  wire out_fire = dws_tvalid;                       // dw_shift m_tready is tied high
  wire out_last = dws_tvalid && dws_tlast;

  // ---------------------------------------------------------------------------
  // Byte enables for the current output beat
  // ---------------------------------------------------------------------------
  logic [15:0] be_comb;
  logic [1:0]  last_j;

  always_comb begin
    last_j = dws_tkeep[12] ? 2'd3 : dws_tkeep[8] ? 2'd2 : dws_tkeep[4] ? 2'd1 : 2'd0;
    for (int j = 0; j < 4; j++) begin
      logic [3:0] be_dw;
      be_dw = 4'hF;
      if (o_first && (j == int'(p_shift)))        be_dw = p_fbe;
      if (dws_tlast && !p_len1 && (j == int'(last_j))) be_dw = p_lbe;
      be_comb[4*j +: 4] = dws_tkeep[4*j] ? be_dw : 4'h0;
    end
  end

  // ---------------------------------------------------------------------------
  // FSM + registered write port
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      state         <= ST_HDR;
      o_first       <= 1'b1;
      out_done_seen <= 1'b0;
      wr_en_ident   <= 1'b0;
      wr_en_acq     <= 1'b0;
      wr_en_iocq    <= 1'b0;
      drop_cnt_inc  <= 1'b0;
    end else begin
      wr_en_ident  <= 1'b0;
      wr_en_acq    <= 1'b0;
      wr_en_iocq   <= 1'b0;
      drop_cnt_inc <= 1'b0;

      // ---- output side: one 16-byte beat per dw_shift output beat ----
      if (out_fire) begin
        wr_data      <= dws_tdata;
        wr_be        <= be_comb;
        wr_beat_addr <= o_addr;
        wr_en_ident  <= (p_sel == WSEL_IDENT);
        wr_en_acq    <= (p_sel == WSEL_ACQ);
        wr_en_iocq   <= (p_sel == WSEL_IOCQ);
        o_addr       <= o_addr + 32'd16;
        o_first      <= 1'b0;
        if (dws_tlast) out_done_seen <= 1'b1;
      end

      // ---- input side ----
      case (state)
        ST_HDR: begin
          if (in_fire) begin
            if (hd_ok) begin
              p_sel         <= hd_sel;
              p_shift       <= hd_rel[3:2];
              p_fbe         <= h1[3:0];
              p_lbe         <= h1[7:4];
              p_len1        <= (hd_len == 11'd1);
              o_addr        <= {hd_rel[31:4], 4'b0000};
              o_first       <= 1'b1;
              out_done_seen <= 1'b0;
              state         <= ST_PAY;
            end else begin
              drop_cnt_inc <= 1'b1;
              if (!s_mwr_tlast) state <= ST_DROP;
            end
          end
        end
        ST_PAY: begin
          if (in_fire && s_mwr_tlast) state <= ST_DRAIN;
        end
        ST_DROP: begin
          if (in_fire && s_mwr_tlast) state <= ST_HDR;
        end
        ST_DRAIN: begin
          if (out_done_seen || out_last) state <= ST_HDR;
        end
      endcase
    end
  end

endmodule
