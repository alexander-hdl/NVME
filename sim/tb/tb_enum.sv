// =============================================================================
// tb_enum.sv - testbench for rp_requester + req_mux + pcie_enum
// -----------------------------------------------------------------------------
//   pcie_enum --p0--> req_mux --> rp_requester --m_req(tlp_axis)--> SSD model
//   doorbell driver --p1-->                    <--s_cpl(tlp_axis)--
//
// SSD model (behavioural, on the internal tlp_axis format of ARCHITECTURE.md 3):
//   * config space: VID/DID 0x144D/0xA808 (dword 0xA808_144D), class 01_08_02,
//     64-bit 16 KiB BAR0 (sizing reads 0xFFFF_C004 / 0xFFFF_FFFF),
//     cap list 0x34 -> 0x40 (PM) -> 0x50 (MSI) -> 0x80 (PCIe), DevCap 0x84
//     (MPSS=2, ETFS=1), DevCtl 0x88 = 0x2810;
//   * CRS for the first N CfgRd of VID, UR / silence / loops on demand;
//   * MRd32 to BAR0 -> CplD with a pattern, MWr32 logged;
//   * random completion latency, random m_req back-pressure, optional
//     unexpected completions (wrong tag / wrong ReqID) in front of each answer.
//
// Tests: 5 positive runs (N = 0, 5, random, doorbell traffic + back-pressure +
// unexpected completions in parallel) and negative runs (UR on VID, VID all
// ones, UR on BAR write, no completion = TIMEOUT at step 1 and step 7 with a
// late completion afterwards, CRS never ending, capability loop, no PCIe
// capability).
// =============================================================================
`timescale 1ns/1ps
module tb_enum;
  import nvme_host_pkg::*;

  localparam int unsigned CRS_MAX   = 8;       // DUT: CRS_RETRY_MAX
  localparam int unsigned CRS_DELAY = 6;       // DUT: CRS_RETRY_DELAY_CYCLES
  localparam int unsigned TMO_CYC   = 300;     // requester timeout
  localparam logic [31:0] INIT_DEVCTL = 32'h0010_2810;
  localparam logic [15:0] REQ_ID    = 16'h0000;

  logic clk = 1'b0;
  logic rst = 1'b1;
  always #4 clk = ~clk;

  int errors = 0;
  task automatic err(input string msg);
    errors++;
    $error("%s", msg);
  endtask

  // ---------------------------------------------------------------------------
  // DUT wiring
  // ---------------------------------------------------------------------------
  // p0: pcie_enum
  logic        e_req_valid, e_req_ready, e_rsp_valid;
  logic [1:0]  e_req_op;
  logic [31:0] e_req_addr, e_req_wdata, e_rsp_rdata;
  logic [3:0]  e_req_be;
  logic [2:0]  e_rsp_status;
  // p1: doorbell driver (this testbench)
  logic        p1_req_valid = 1'b0, p1_req_ready, p1_rsp_valid;
  logic [1:0]  p1_req_op = 2'd0;
  logic [31:0] p1_req_addr = 32'h0, p1_req_wdata = 32'h0, p1_rsp_rdata;
  logic [3:0]  p1_req_be = 4'hF;
  logic [2:0]  p1_rsp_status;
  // mux -> requester
  logic        r_req_valid, r_req_ready, r_rsp_valid;
  logic [1:0]  r_req_op;
  logic [31:0] r_req_addr, r_req_wdata, r_rsp_rdata;
  logic [3:0]  r_req_be;
  logic [2:0]  r_rsp_status;
  // tlp_axis requester -> model
  logic [127:0] m_req_tdata;
  logic [15:0]  m_req_tkeep;
  logic         m_req_tlast, m_req_tvalid;
  logic [1:0]   m_req_tuser;
  logic         m_req_tready;
  // tlp_axis model -> requester
  logic [127:0] s_cpl_tdata  = '0;
  logic [15:0]  s_cpl_tkeep  = '0;
  logic         s_cpl_tlast  = 1'b0;
  logic         s_cpl_tvalid = 1'b0;
  logic [1:0]   s_cpl_tuser  = 2'b00;
  logic         s_cpl_tready;

  logic [15:0]  unexpected_cpl_cnt;
  logic [31:0]  timeout_cycles = TMO_CYC;

  // pcie_enum
  logic         enum_start = 1'b0;
  logic         enum_done, enum_fail;
  logic [7:0]   enum_fail_step;
  logic [2:0]   mps_code;
  logic         ext_tag_en;
  logic [31:0]  vid_did, bar0_size_mask, devcap;
  logic         class_ok;
  logic [7:0]   pcie_cap_off;

  pcie_enum #(
    .CRS_RETRY_MAX(CRS_MAX), .CRS_RETRY_DELAY_CYCLES(CRS_DELAY)
  ) u_enum (
    .clk(clk), .rst(rst),
    .start(enum_start), .done(enum_done), .fail(enum_fail), .fail_step(enum_fail_step),
    .mps_code(mps_code), .ext_tag_en(ext_tag_en), .vid_did(vid_did),
    .bar0_size_mask(bar0_size_mask), .class_ok(class_ok), .pcie_cap_off(pcie_cap_off),
    .devcap(devcap),
    .req_valid(e_req_valid), .req_ready(e_req_ready), .req_op(e_req_op),
    .req_addr(e_req_addr), .req_wdata(e_req_wdata), .req_be(e_req_be),
    .rsp_valid(e_rsp_valid), .rsp_status(e_rsp_status), .rsp_rdata(e_rsp_rdata)
  );

  req_mux u_mux (
    .clk(clk), .rst(rst),
    .p0_req_valid(e_req_valid), .p0_req_ready(e_req_ready), .p0_req_op(e_req_op),
    .p0_req_addr(e_req_addr), .p0_req_wdata(e_req_wdata), .p0_req_be(e_req_be),
    .p0_rsp_valid(e_rsp_valid), .p0_rsp_status(e_rsp_status), .p0_rsp_rdata(e_rsp_rdata),
    .p1_req_valid(p1_req_valid), .p1_req_ready(p1_req_ready), .p1_req_op(p1_req_op),
    .p1_req_addr(p1_req_addr), .p1_req_wdata(p1_req_wdata), .p1_req_be(p1_req_be),
    .p1_rsp_valid(p1_rsp_valid), .p1_rsp_status(p1_rsp_status), .p1_rsp_rdata(p1_rsp_rdata),
    .r_req_valid(r_req_valid), .r_req_ready(r_req_ready), .r_req_op(r_req_op),
    .r_req_addr(r_req_addr), .r_req_wdata(r_req_wdata), .r_req_be(r_req_be),
    .r_rsp_valid(r_rsp_valid), .r_rsp_status(r_rsp_status), .r_rsp_rdata(r_rsp_rdata)
  );

  rp_requester u_requester (
    .clk(clk), .rst(rst),
    .req_valid(r_req_valid), .req_ready(r_req_ready), .req_op(r_req_op),
    .req_addr(r_req_addr), .req_wdata(r_req_wdata), .req_be(r_req_be),
    .rsp_valid(r_rsp_valid), .rsp_status(r_rsp_status), .rsp_rdata(r_rsp_rdata),
    .timeout_cycles(timeout_cycles),
    .m_req_tdata(m_req_tdata), .m_req_tkeep(m_req_tkeep), .m_req_tlast(m_req_tlast),
    .m_req_tvalid(m_req_tvalid), .m_req_tuser(m_req_tuser), .m_req_tready(m_req_tready),
    .s_cpl_tdata(s_cpl_tdata), .s_cpl_tkeep(s_cpl_tkeep), .s_cpl_tlast(s_cpl_tlast),
    .s_cpl_tvalid(s_cpl_tvalid), .s_cpl_tuser(s_cpl_tuser), .s_cpl_tready(s_cpl_tready),
    .requester_id(REQ_ID), .unexpected_cpl_cnt(unexpected_cpl_cnt)
  );

  // ---------------------------------------------------------------------------
  // SSD model: knobs (set by the test before releasing reset)
  // ---------------------------------------------------------------------------
  int  k_crs_n        = 0;       // CRS answers to CfgRd of VID
  int  k_ur_at        = -1;      // UR for the config request with this index
  int  k_silent_at    = -1;      // no completion for the config request with this index
  bit  k_vid_ffff     = 1'b0;    // VID reads as 0xFFFF_FFFF
  bit  k_cap_loop     = 1'b0;    // capability list loops 0x40 <-> 0x50
  bit  k_no_pcie_cap  = 1'b0;    // list ends without the PCIe capability
  bit  k_inj_unexp    = 1'b0;    // unexpected completions before every answer
  bit  k_bp           = 1'b0;    // random back-pressure on m_req

  // model state
  logic [31:0] cfg [0:63];
  int          crs_left;
  int          cfg_count;        // config requests seen
  logic [7:0]  exp_tag;          // next expected non-posted tag
  int          n_inj;            // unexpected completions injected
  logic [15:0] silent_reqid;
  logic [7:0]  silent_tag;

  // logs
  bit          cl_wr   [$];
  logic [9:0]  cl_reg  [$];
  logic [31:0] cl_data [$];
  logic [3:0]  cl_be   [$];
  logic [31:0] ml_addr [$];      // MWr log
  logic [31:0] ml_data [$];
  logic [3:0]  ml_be   [$];
  int          n_mrd;

  function automatic logic [31:0] pattern(input logic [31:0] addr);
    return 32'hC0DE_0000 | {16'h0, addr[15:0]};
  endfunction

  task automatic init_cfg();
    for (int i = 0; i < 64; i++) cfg[i] = 32'h0;
    cfg[8'h00 >> 2] = k_vid_ffff ? 32'hFFFF_FFFF : 32'hA808_144D;   // {DID, VID}
    cfg[8'h08 >> 2] = 32'h0108_0200;                               // class 01_08_02, rev 0
    cfg[8'h10 >> 2] = 32'h0000_0004;                               // BAR0: 64-bit, non-prefetch
    cfg[8'h34 >> 2] = 32'h0000_0040;
    cfg[8'h40 >> 2] = 32'h0003_5001;                               // PM, next 0x50
    cfg[8'h50 >> 2] = k_cap_loop    ? 32'h0080_5005 :              // MSI, next = itself
                      k_no_pcie_cap ? 32'h0080_0005 :              // MSI, end of list
                                      32'h0080_8005;               // MSI, next 0x80
    cfg[8'h80 >> 2] = 32'h0002_0010;                               // PCIe cap, last
    cfg[8'h84 >> 2] = 32'h0000_8022;                               // DevCap: MPSS=2, ETFS=1
    cfg[8'h88 >> 2] = INIT_DEVCTL;                                 // DevCtl 0x2810
  endtask

  task automatic cfg_write(input logic [9:0] dw, input logic [31:0] data, input logic [3:0] be);
    logic [31:0] tmp;
    if (dw >= 64) return;
    tmp = cfg[dw];
    for (int b = 0; b < 4; b++) if (be[b]) tmp[8*b +: 8] = data[8*b +: 8];
    case (dw)
      10'(8'h10 >> 2): cfg[dw] = (tmp & 32'hFFFF_C000) | 32'h4;     // 16 KiB, 64-bit flags
      10'(8'h14 >> 2): cfg[dw] = tmp;
      10'(8'h04 >> 2): cfg[dw] = tmp;
      10'(8'h88 >> 2): cfg[dw] = {cfg[dw][31:16], tmp[15:0]};       // DevStatus read-only here
      default: ;                                                    // read-only / ignored
    endcase
  endtask

  // ---------------------------------------------------------------------------
  // SSD model: completion queue + driver
  // ---------------------------------------------------------------------------
  logic [127:0] q_b0  [$];
  bit           q_pl  [$];
  logic [31:0]  q_dat [$];
  int           q_dly [$];

  task automatic push_cpl(input logic [127:0] b0, input bit has_pl, input logic [31:0] data, input int dly);
    q_b0.push_back(b0);
    q_pl.push_back(has_pl);
    q_dat.push_back(data);
    q_dly.push_back(dly);
  endtask

  // answer a request; injects unexpected completions first when enabled
  task automatic respond(input bit with_data, input logic [2:0] st, input logic [31:0] data,
                         input logic [15:0] reqid, input logic [7:0] tag, input logic [6:0] la);
    int dly;
    dly = $urandom_range(1, 6);
    if (k_inj_unexp) begin
      push_cpl(hdr_cpl(1'b1, SSD_BDF, CPL_SC, 12'd4, reqid, tag + 8'd1, 7'h0, 10'd1), 1'b1, 32'hBAD0_0001, dly);
      push_cpl(hdr_cpl(1'b1, SSD_BDF, CPL_SC, 12'd4, reqid ^ 16'h0100, tag, 7'h0, 10'd1), 1'b1, 32'hBAD0_0002, 0);
      push_cpl(hdr_cpl(1'b0, SSD_BDF, CPL_UR, 12'd4, reqid, tag + 8'd7, 7'h0, 10'd0), 1'b0, 32'h0, 0);
      n_inj += 3;
      dly = 0;
    end
    if (with_data)
      push_cpl(hdr_cpl(1'b1, SSD_BDF, st, 12'd4, reqid, tag, la, 10'd1), 1'b1, data, dly);
    else
      push_cpl(hdr_cpl(1'b0, SSD_BDF, st, 12'd4, reqid, tag, la, 10'd0), 1'b0, 32'h0, dly);
  endtask

  // driver of s_cpl
  bit   drv_beat1;       // next beat to send is the payload beat
  logic [31:0] drv_pl;
  bit   dly_loaded;
  int   dly_cnt;

  always @(posedge clk) begin
    if (rst) begin
      s_cpl_tvalid <= 1'b0;
      s_cpl_tlast  <= 1'b0;
      s_cpl_tkeep  <= '0;
      s_cpl_tdata  <= '0;
      drv_beat1    <= 1'b0;
      dly_loaded   <= 1'b0;
      dly_cnt      <= 0;
      q_b0.delete(); q_pl.delete(); q_dat.delete(); q_dly.delete();
    end else begin
      if (s_cpl_tvalid && s_cpl_tready) begin
        if (drv_beat1) begin
          s_cpl_tdata <= {96'h0, drv_pl};
          s_cpl_tkeep <= 16'h000F;
          s_cpl_tlast <= 1'b1;
          drv_beat1   <= 1'b0;
        end else begin
          s_cpl_tvalid <= 1'b0;
          s_cpl_tlast  <= 1'b0;
        end
      end else if (!s_cpl_tvalid) begin
        if (q_b0.size() > 0) begin
          if (!dly_loaded) begin
            dly_cnt    <= q_dly[0];
            dly_loaded <= 1'b1;
          end else if (dly_cnt == 0) begin
            s_cpl_tdata  <= q_b0.pop_front();
            s_cpl_tkeep  <= 16'hFFFF;
            s_cpl_tvalid <= 1'b1;
            drv_beat1    <= q_pl[0];
            s_cpl_tlast  <= !q_pl[0];
            drv_pl       <= q_dat[0];
            void'(q_pl.pop_front());
            void'(q_dat.pop_front());
            void'(q_dly.pop_front());
            dly_loaded   <= 1'b0;
          end else begin
            dly_cnt <= dly_cnt - 1;
          end
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // SSD model: request receiver
  // ---------------------------------------------------------------------------
  logic [31:0] rq_h0, rq_h1, rq_h2;
  bit          rq_in_pkt;
  bit          prev_stall;
  logic [127:0] prev_tdata;
  logic [15:0]  prev_tkeep;
  bit           prev_tlast;

  // random back-pressure
  logic rdy_q = 1'b1;
  always @(posedge clk) rdy_q <= k_bp ? ($urandom_range(0, 2) != 0) : 1'b1;
  assign m_req_tready = rdy_q;

  task automatic process_request(input logic [31:0] h0, input logic [31:0] h1, input logic [31:0] h2,
                                 input bit has_pl, input logic [31:0] pl);
    logic [7:0]  ft;
    logic [15:0] bdf;
    logic [9:0]  dw;
    logic [3:0]  fbe;
    bit          is_wr;
    logic [2:0]  st;
    int          idx;
    ft  = tlp_fmt_type(h0);
    fbe = req_fbe(h1);

    if (ft == FT_CFGRD0 || ft == FT_CFGWR0) begin
      is_wr = (ft == FT_CFGWR0);
      bdf   = h2[31:16];
      dw    = h2[11:2];
      // format checks
      if (h0[23:0] !== 24'h00_0001) err($sformatf("config TLP H0 low bits wrong: 0x%08h", h0));
      if (req_lbe(h1) !== 4'h0) err("config TLP last BE must be 0");
      if (req_reqid(h1) !== REQ_ID) err("config TLP requester ID wrong");
      if (req_tag(h1) !== exp_tag) err($sformatf("config TLP tag %0d, expected %0d", req_tag(h1), exp_tag));
      if (h2[15:12] !== 4'h0 || h2[1:0] !== 2'b00) err("config TLP H2 reserved bits not zero");
      if (is_wr !== has_pl) err("config write/read payload presence mismatch");
      exp_tag = exp_tag + 8'd1;
      idx = cfg_count;
      cfg_count++;
      cl_wr.push_back(is_wr);
      cl_reg.push_back(dw);
      cl_data.push_back(pl);
      cl_be.push_back(fbe);

      if (idx == k_silent_at) begin
        silent_reqid = req_reqid(h1);
        silent_tag   = req_tag(h1);
        return;                                       // no completion at all
      end
      if (bdf != SSD_BDF || idx == k_ur_at) begin
        respond(1'b0, CPL_UR, 32'h0, req_reqid(h1), req_tag(h1), 7'h0);
      end else if (!is_wr && dw == 10'd0 && crs_left > 0) begin
        crs_left--;
        respond(1'b0, CPL_CRS, 32'h0, req_reqid(h1), req_tag(h1), 7'h0);
      end else if (is_wr) begin
        cfg_write(dw, pl, fbe);
        respond(1'b0, CPL_SC, 32'h0, req_reqid(h1), req_tag(h1), 7'h0);
      end else begin
        respond(1'b1, CPL_SC, (dw < 64) ? cfg[dw] : 32'h0, req_reqid(h1), req_tag(h1), 7'h0);
      end

    end else if (ft == FT_MRD32) begin
      if (req_tag(h1) !== exp_tag) err($sformatf("MRd tag %0d, expected %0d", req_tag(h1), exp_tag));
      exp_tag = exp_tag + 8'd1;
      if (tlp_len_dw(h0) !== 10'd1) err("MRd length must be 1 DW");
      if (h2[31:30] != 2'b11 || h2[1:0] != 2'b00) err("MRd address outside BAR0 / unaligned");
      n_mrd++;
      respond(1'b1, CPL_SC, pattern(h2), req_reqid(h1), req_tag(h1), h2[6:0]);

    end else if (ft == FT_MWR32) begin
      if (tlp_len_dw(h0) !== 10'd1 || !has_pl) err("MWr must carry exactly 1 DW");
      ml_addr.push_back(h2);
      ml_data.push_back(pl);
      ml_be.push_back(fbe);

    end else begin
      err($sformatf("unexpected TLP type 0x%02h from requester", ft));
    end
  endtask

  always @(posedge clk) begin
    if (rst) begin
      rq_in_pkt  <= 1'b0;
      prev_stall <= 1'b0;
    end else begin
      // stability while stalled
      if (prev_stall) begin
        if (!m_req_tvalid) err("m_req_tvalid dropped while tready=0");
        else if (m_req_tdata !== prev_tdata || m_req_tkeep !== prev_tkeep || m_req_tlast !== prev_tlast)
          err("m_req data changed while stalled");
      end
      prev_stall <= m_req_tvalid && !m_req_tready;
      prev_tdata <= m_req_tdata;
      prev_tkeep <= m_req_tkeep;
      prev_tlast <= m_req_tlast;

      if (s_cpl_tready !== 1'b1) err("s_cpl_tready must stay high");
      if (m_req_tuser !== 2'b00 && m_req_tvalid) err("m_req_tuser must be 0");

      if (m_req_tvalid && m_req_tready) begin
        if (!rq_in_pkt) begin
          if (m_req_tkeep !== 16'hFFFF) err("header beat tkeep must be FFFF");
          if (m_req_tdata[127:96] !== 32'h0) err("H3 must be zero");
          rq_h0 <= m_req_tdata[31:0];
          rq_h1 <= m_req_tdata[63:32];
          rq_h2 <= m_req_tdata[95:64];
          if (m_req_tlast) begin
            if (tlp_has_data(m_req_tdata[31:0])) err("TLP with data format but no payload beat");
            process_request(m_req_tdata[31:0], m_req_tdata[63:32], m_req_tdata[95:64], 1'b0, 32'h0);
          end else begin
            if (!tlp_has_data(m_req_tdata[31:0])) err("payload beat for a header-only TLP");
            rq_in_pkt <= 1'b1;
          end
        end else begin
          if (m_req_tkeep !== 16'h000F) err("payload beat tkeep must be 000F");
          if (m_req_tlast !== 1'b1) err("payload beat must be the last one");
          if (m_req_tdata[127:32] !== 96'h0) err("payload beat upper DWs must be zero");
          process_request(rq_h0, rq_h1, rq_h2, 1'b1, m_req_tdata[31:0]);
          rq_in_pkt <= 1'b0;
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Doorbell traffic on port 1 (MWr to BAR0 doorbells, MRd of a BAR0 register)
  // ---------------------------------------------------------------------------
  bit  p1_en = 1'b0;
  bit  p1_busy = 1'b0;
  int  p1_n_wr, p1_n_rd, p1_errors;
  logic [31:0] p1_wr_addr [$];
  logic [31:0] p1_wr_data [$];

  task automatic p1_cmd(input logic [1:0] op, input logic [31:0] addr, input logic [31:0] wd,
                        output logic [2:0] st, output logic [31:0] rd);
    @(posedge clk);
    p1_req_valid <= 1'b1;
    p1_req_op    <= op;
    p1_req_addr  <= addr;
    p1_req_wdata <= wd;
    p1_req_be    <= 4'hF;
    forever begin
      @(posedge clk);
      if (p1_req_ready) break;
    end
    p1_req_valid <= 1'b0;
    forever begin
      @(posedge clk);
      if (p1_rsp_valid) break;
    end
    st = p1_rsp_status;
    rd = p1_rsp_rdata;
  endtask

  initial begin
    logic [2:0]  st;
    logic [31:0] rd, a;
    forever begin
      wait (p1_en);
      p1_busy = 1'b1;
      repeat ($urandom_range(0, 3)) @(posedge clk);
      if ($urandom_range(0, 1) == 0) begin
        a = BAR0_ADDR + 32'h1000 + 4 * p1_n_wr;
        p1_cmd(2'd3, a, 32'hD00D_0000 + p1_n_wr, st, rd);
        p1_wr_addr.push_back(a);
        p1_wr_data.push_back(32'hD00D_0000 + p1_n_wr);
        p1_n_wr++;
        if (st !== 3'd0) begin p1_errors++; err("p1 MWr status not SC"); end
      end else begin
        a = BAR0_ADDR + 32'h0000_0020 + 4 * p1_n_rd;
        p1_cmd(2'd2, a, 32'h0, st, rd);
        p1_n_rd++;
        if (st !== 3'd0) begin p1_errors++; err("p1 MRd status not SC"); end
        if (rd !== pattern(a)) begin p1_errors++; err($sformatf("p1 MRd data 0x%08h, expected 0x%08h", rd, pattern(a))); end
      end
      p1_busy = 1'b0;
      @(posedge clk);
    end
  end

  // ---------------------------------------------------------------------------
  // Mux fairness monitor: while p1 waits, at most ONE p0 command may be accepted
  // ---------------------------------------------------------------------------
  int p0_while_p1;
  always @(posedge clk) begin
    if (rst) p0_while_p1 <= 0;
    else begin
      if (e_req_valid && e_req_ready && p1_req_valid) begin
        p0_while_p1 <= p0_while_p1 + 1;
        if (p0_while_p1 >= 1) err("req_mux: p1 starved by more than one p0 command");
      end
      if (p1_req_valid && p1_req_ready) p0_while_p1 <= 0;
    end
  end

  // ---------------------------------------------------------------------------
  // Test infrastructure
  // ---------------------------------------------------------------------------
  // expected config sequence
  bit          ex_wr   [$];
  logic [9:0]  ex_reg  [$];
  logic [31:0] ex_data [$];
  logic [3:0]  ex_be   [$];

  task automatic ex_add(input bit w, input logic [9:0] dw, input logic [31:0] d, input logic [3:0] be);
    ex_wr.push_back(w); ex_reg.push_back(dw); ex_data.push_back(d); ex_be.push_back(be);
  endtask

  function automatic logic [15:0] exp_devctl_value();
    logic [2:0] mps;
    mps = (3'd2 < MPS_CAP_CODE) ? 3'd2 : MPS_CAP_CODE;       // min(MPSS=2, cap)
    return (INIT_DEVCTL[15:0] & ~16'h71E0) | (16'(mps) << 5) | (16'(MRRS_CODE) << 12) |
           (16'(EXT_TAG_ALLOW & 1'b1) << 8);
  endfunction

  task automatic build_expected(input int n_crs);
    ex_wr.delete(); ex_reg.delete(); ex_data.delete(); ex_be.delete();
    for (int i = 0; i <= n_crs; i++) ex_add(0, 10'h000, 0, 4'hF);          // VID (CRS retries + final)
    ex_add(0, 10'h002, 0, 4'hF);                                           // class
    ex_add(1, 10'h004, 32'hFFFF_FFFF, 4'hF);  ex_add(0, 10'h004, 0, 4'hF); // BAR0 sizing
    ex_add(1, 10'h005, 32'hFFFF_FFFF, 4'hF);  ex_add(0, 10'h005, 0, 4'hF); // BAR1 sizing
    ex_add(1, 10'h004, BAR0_ADDR, 4'hF);      ex_add(1, 10'h005, 32'h0, 4'hF);
    ex_add(0, 10'h00D, 0, 4'hF);                                           // 0x34
    ex_add(0, 10'h010, 0, 4'hF);                                           // 0x40
    ex_add(0, 10'h014, 0, 4'hF);                                           // 0x50
    ex_add(0, 10'h020, 0, 4'hF);                                           // 0x80
    ex_add(0, 10'h021, 0, 4'hF);                                           // 0x84 DevCap
    ex_add(0, 10'h022, 0, 4'hF);                                           // 0x88 DevCtl
    ex_add(1, 10'h022, {16'h0, exp_devctl_value()}, 4'b0011);
    ex_add(1, 10'h001, 32'h0000_0406, 4'b0011);                            // Command
  endtask

  function automatic logic [31:0] be_mask(input logic [3:0] be);
    return {{8{be[3]}}, {8{be[2]}}, {8{be[1]}}, {8{be[0]}}};
  endfunction

  task automatic check_sequence(input string name);
    int n;
    n = cl_wr.size();
    if (n != ex_wr.size())
      err($sformatf("%s: %0d config TLPs seen, %0d expected", name, n, ex_wr.size()));
    else begin
      for (int i = 0; i < n; i++) begin
        bit w; logic [9:0] r; logic [31:0] d; logic [3:0] b;
        bit xw; logic [9:0] xr; logic [31:0] xd; logic [3:0] xb;
        w = cl_wr[i]; r = cl_reg[i]; d = cl_data[i]; b = cl_be[i];
        xw = ex_wr[i]; xr = ex_reg[i]; xd = ex_data[i]; xb = ex_be[i];
        if (w !== xw || r !== xr || b !== xb)
          err($sformatf("%s: TLP #%0d is %s dw 0x%03h be %b, expected %s dw 0x%03h be %b", name, i,
                        w ? "WR" : "RD", r, b, xw ? "WR" : "RD", xr, xb));
        else if (w && ((d & be_mask(b)) !== (xd & be_mask(b))))
          err($sformatf("%s: TLP #%0d write data 0x%08h, expected 0x%08h (be %b)", name, i, d, xd, b));
      end
    end
  endtask

  // reset DUT + model with the knobs already set
  task automatic do_reset();
    p1_en = 1'b0;
    while (p1_busy) @(posedge clk);
    rst <= 1'b1;
    repeat (4) @(posedge clk);
    cl_wr.delete(); cl_reg.delete(); cl_data.delete(); cl_be.delete();
    ml_addr.delete(); ml_data.delete(); ml_be.delete();
    p1_wr_addr.delete(); p1_wr_data.delete();
    n_inj = 0; n_mrd = 0; cfg_count = 0; exp_tag = 8'h00; crs_left = k_crs_n;
    p1_n_wr = 0; p1_n_rd = 0; p1_errors = 0;
    init_cfg();
    rst <= 1'b0;
    repeat (3) @(posedge clk);
  endtask

  task automatic start_enum();
    @(posedge clk); enum_start <= 1'b1;
    @(posedge clk); enum_start <= 1'b0;
  endtask

  task automatic wait_finish(input string name);
    int cyc;
    cyc = 0;
    while (!enum_done && !enum_fail) begin
      @(posedge clk);
      cyc++;
      if (cyc > 100000) begin err({name, ": enum never finished"}); return; end
    end
    p1_en = 1'b0;
    while (p1_busy) @(posedge clk);
    repeat (10) @(posedge clk);
  endtask

  task automatic reset_knobs();
    k_crs_n = 0; k_ur_at = -1; k_silent_at = -1; k_vid_ffff = 1'b0; k_cap_loop = 1'b0;
    k_no_pcie_cap = 1'b0; k_inj_unexp = 1'b0; k_bp = 1'b0;
  endtask

  // positive test
  task automatic run_positive(input string name, input int n_crs, input bit stress);
    reset_knobs();
    k_crs_n = n_crs;
    k_inj_unexp = stress;
    k_bp = stress;
    do_reset();
    build_expected(n_crs);
    start_enum();
    p1_en = stress;
    wait_finish(name);

    check_sequence(name);
    if (enum_done !== 1'b1 || enum_fail !== 1'b0) err({name, ": expected done=1 fail=0"});
    if (mps_code !== 3'd1)            err($sformatf("%s: mps_code %0d, expected 1", name, mps_code));
    if (ext_tag_en !== 1'b1)          err({name, ": ext_tag_en expected 1"});
    if (vid_did !== 32'hA808_144D)    err($sformatf("%s: vid_did 0x%08h", name, vid_did));
    if (bar0_size_mask !== 32'hFFFF_C004) err($sformatf("%s: bar0_size_mask 0x%08h", name, bar0_size_mask));
    if (class_ok !== 1'b1)            err({name, ": class_ok expected 1"});
    if (pcie_cap_off !== 8'h80)       err($sformatf("%s: pcie_cap_off 0x%02h", name, pcie_cap_off));
    if (devcap !== 32'h0000_8022)     err($sformatf("%s: devcap 0x%08h", name, devcap));
    // model state after the run
    if (cfg[8'h10 >> 2] !== 32'hC000_0004) err($sformatf("%s: model BAR0 0x%08h, expected C0000004", name, cfg[8'h10 >> 2]));
    if (cfg[8'h14 >> 2] !== 32'h0)         err($sformatf("%s: model BAR1 0x%08h, expected 0", name, cfg[8'h14 >> 2]));
    if (cfg[8'h04 >> 2][15:0] !== 16'h0406) err($sformatf("%s: model Command 0x%04h, expected 0406", name, cfg[8'h04 >> 2][15:0]));
    if (cfg[8'h88 >> 2][15:0] !== exp_devctl_value())
      err($sformatf("%s: model DevCtl 0x%04h, expected 0x%04h", name, cfg[8'h88 >> 2][15:0], exp_devctl_value()));
    if (cfg[8'h88 >> 2][31:16] !== INIT_DEVCTL[31:16]) err({name, ": DevStatus must not be written"});
    // unexpected completions all counted, none mis-matched
    if (unexpected_cpl_cnt !== 16'(n_inj))
      err($sformatf("%s: unexpected_cpl_cnt %0d, expected %0d", name, unexpected_cpl_cnt, n_inj));
    // p1 traffic
    if (stress) begin
      if (p1_n_wr + p1_n_rd < 4) err($sformatf("%s: p1 made only %0d accesses during enumeration", name, p1_n_wr + p1_n_rd));
      if (ml_addr.size() != p1_n_wr) err($sformatf("%s: %0d MWr seen, %0d sent", name, ml_addr.size(), p1_n_wr));
      else for (int i = 0; i < p1_n_wr; i++) begin
        logic [31:0] ea, ed, ga, gd;
        ea = p1_wr_addr[i]; ed = p1_wr_data[i]; ga = ml_addr[i]; gd = ml_data[i];
        if (ga !== ea || gd !== ed) err($sformatf("%s: MWr #%0d addr/data 0x%08h/0x%08h, expected 0x%08h/0x%08h", name, i, ga, gd, ea, ed));
        if (ml_be[i] !== 4'hF) err({name, ": MWr BE must be F"});
      end
      if (n_mrd != p1_n_rd) err($sformatf("%s: %0d MRd seen, %0d sent", name, n_mrd, p1_n_rd));
      if (p1_errors != 0) err({name, ": p1 response errors"});
    end
    $display("[%0t] %-28s PASS-CHECKED: %0d cfg TLPs, p1 wr/rd %0d/%0d, unexpected cpl %0d, mps=%0d ete=%0b",
             $time, name, cl_wr.size(), p1_n_wr, p1_n_rd, unexpected_cpl_cnt, mps_code, ext_tag_en);
  endtask

  // negative test
  task automatic run_negative(input string name, input int exp_step, input int exp_tlps);
    do_reset();
    start_enum();
    wait_finish(name);
    if (enum_fail !== 1'b1 || enum_done !== 1'b0)
      err($sformatf("%s: expected fail=1 done=0 (got fail=%0b done=%0b)", name, enum_fail, enum_done));
    if (enum_fail_step !== 8'(exp_step))
      err($sformatf("%s: fail_step %0d, expected %0d", name, enum_fail_step, exp_step));
    if (exp_tlps >= 0 && cl_wr.size() != exp_tlps)
      err($sformatf("%s: %0d config TLPs sent, expected %0d", name, cl_wr.size(), exp_tlps));
    $display("[%0t] %-28s fail=%0b step=%0d (cfg TLPs sent: %0d)", $time, name, enum_fail, enum_fail_step, cl_wr.size());
  endtask

  // ---------------------------------------------------------------------------
  // Main
  // ---------------------------------------------------------------------------
  initial begin
    repeat (4) @(posedge clk);
    rst <= 1'b0;

    // ---------------- positive ----------------
    run_positive("pos: N=0 CRS",           0, 1'b0);
    run_positive("pos: N=5 CRS",           5, 1'b0);
    run_positive("pos: random N, stress",  $urandom_range(0, 5), 1'b1);
    run_positive("pos: N=3 stress",        3, 1'b1);
    run_positive("pos: random N",          $urandom_range(0, 5), 1'b0);

    // ---------------- negative ----------------
    reset_knobs(); k_ur_at = 0;
    run_negative("neg: UR on VID",          1, 1);

    reset_knobs(); k_vid_ffff = 1'b1;
    run_negative("neg: VID all ones",       1, 1);

    reset_knobs(); k_ur_at = 2;                         // third request = first BAR0 sizing write
    run_negative("neg: UR on BAR write",    3, 3);

    reset_knobs(); k_silent_at = 0;
    run_negative("neg: timeout on VID",     1, 1);
    begin
      logic [15:0] cnt0;
      cnt0 = unexpected_cpl_cnt;
      // late completion for the request that timed out: must be consumed and counted
      push_cpl(hdr_cpl(1'b1, SSD_BDF, CPL_SC, 12'd4, silent_reqid, silent_tag, 7'h0, 10'd1), 1'b1, 32'h1A7E_0001, 0);
      repeat (20) @(posedge clk);
      if (unexpected_cpl_cnt !== cnt0 + 16'd1)
        err($sformatf("late completion: unexpected_cpl_cnt %0d, expected %0d", unexpected_cpl_cnt, cnt0 + 1));
    end

    reset_knobs(); k_silent_at = 14;                    // DevCtl write (N=0)
    run_negative("neg: timeout on DevCtl WR", 7, 15);

    reset_knobs(); k_crs_n = 100;
    run_negative("neg: CRS never ends",     1, CRS_MAX + 1);

    reset_knobs(); k_cap_loop = 1'b1;
    run_negative("neg: capability loop",    5, 9 + 48);

    reset_knobs(); k_no_pcie_cap = 1'b1;
    run_negative("neg: no PCIe capability", 5, 9 + 2);

    reset_knobs();
    if (errors != 0) $fatal(1, "TEST FAILED: %0d error(s)", errors);
    $display("TEST PASSED");
    $finish;
  end

  initial begin
    #20_000_000;
    $fatal(1, "TEST FAILED: global watchdog");
  end
endmodule
