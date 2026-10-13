// =============================================================================
// pcie_enum.sv - enumerate the SSD behind the Root Port (ARCHITECTURE.md 5.14)
// -----------------------------------------------------------------------------
// A small sequencer that talks to the SSD's PCI config space (bus 1, dev 0,
// fn 0 = SSD_BDF) through one command port of rp_requester / req_mux.
//
//   step 1  CfgRd 0x00           vid_did (must not be 0xFFFF)
//   step 2  CfgRd 0x08           class_ok = (class code == 01_08_02, NVMe); warning only
//   step 3  BAR sizing           CfgWr 0x10 = FFFF_FFFF, CfgRd 0x10 -> bar0_size_mask,
//                                CfgWr 0x14 = FFFF_FFFF, CfgRd 0x14
//   step 4  BAR assignment       CfgWr 0x10 = BAR0_ADDR, CfgWr 0x14 = 0
//   step 5  capability walk      CfgRd 0x34 -> pointer; follow the list until
//                                the PCIe capability (ID 0x10); max 48 hops
//   step 6  CfgRd cap+0x04       DevCap: MPSS[2:0], ETFS[5]
//                                mps_code = min(MPSS, MPS_CAP_CODE)
//                                ext_tag_en = ETFS & EXT_TAG_ALLOW
//   step 7  CfgRd cap+0x08       DevCtl: [7:5]=mps_code, [14:12]=MRRS_CODE,
//                                [8]=ext_tag_en, then CfgWr cap+0x08 (BE 0011)
//   step 8  CfgWr 0x04 = 0x0406  Command: MSE + BME + INTx disable (BE 0011)
//
// Error handling for every request:
//   * status CRS  -> wait CRS_RETRY_DELAY_CYCLES and re-issue the SAME request,
//                    at most CRS_RETRY_MAX retries per request, then fail;
//   * status UR / CA / TIMEOUT / UNEXPECTED -> fail immediately;
//   * fail_step = step number (1..8) of the failing request. Also reported
//     as a failure: VID 0xFFFF (step 1), invalid / looping capability list
//     (step 5), and exhausting CRS retries.
//
// start is edge-sensitive. done / fail are levels that stay set until the next
// start; they drop in the very cycle in which the start edge is presented
// (done = done_r & ~start_evt), so a sequencer that samples them in the cycle of its own
// start pulse never sees the flags of the previous run. The command port follows the rp_requester signature.
// =============================================================================
module pcie_enum
  import nvme_host_pkg::SSD_BDF, nvme_host_pkg::CFG_VID_DID, nvme_host_pkg::CFG_COMMAND,
         nvme_host_pkg::CFG_CLASS, nvme_host_pkg::CFG_BAR0, nvme_host_pkg::CFG_BAR1,
         nvme_host_pkg::CFG_CAP_PTR, nvme_host_pkg::PCIE_CAP_ID,
         nvme_host_pkg::CMD_MSE_BME_INTXDIS;
#(
  parameter int unsigned CRS_RETRY_MAX          = 1000,
  parameter int unsigned CRS_RETRY_DELAY_CYCLES = 125_000,   // 1 ms @ 125 MHz
  parameter logic [2:0]  MPS_CAP_CODE           = nvme_host_pkg::MPS_CAP_CODE,
  parameter logic [2:0]  MRRS_CODE              = nvme_host_pkg::MRRS_CODE,
  parameter bit          EXT_TAG_ALLOW         = nvme_host_pkg::EXT_TAG_ALLOW,
  parameter logic [31:0] BAR0_ADDR              = nvme_host_pkg::BAR0_ADDR
) (
  input  logic        clk,
  input  logic        rst,

  input  logic        start,
  output logic        done,             // level; cleared in the SAME cycle as the start edge (comb. mask)
  output logic        fail,
  output logic [7:0]  fail_step,

  // results
  output logic [2:0]  mps_code,
  output logic        ext_tag_en,
  output logic [31:0] vid_did,
  output logic [31:0] bar0_size_mask,
  output logic        class_ok,
  output logic [7:0]  pcie_cap_off,
  output logic [31:0] devcap,

  // command port (to req_mux p0 / rp_requester)
  output logic        req_valid,
  input  logic        req_ready,
  output logic [1:0]  req_op,
  output logic [31:0] req_addr,
  output logic [31:0] req_wdata,
  output logic [3:0]  req_be,
  input  logic        rsp_valid,
  input  logic [2:0]  rsp_status,
  input  logic [31:0] rsp_rdata
);

  // ---------------------------------------------------------------------------
  // Encodings and constants
  // ---------------------------------------------------------------------------
  localparam logic [1:0] OP_CFGRD0 = 2'd0;
  localparam logic [1:0] OP_CFGWR0 = 2'd1;

  localparam logic [2:0] RSP_SC  = 3'd0;
  localparam logic [2:0] RSP_CRS = 3'd2;

  localparam logic [3:0] BE_ALL = 4'b1111;
  localparam logic [3:0] BE_LO2 = 4'b0011;

  localparam logic [31:0] ALL_ONES        = 32'hFFFF_FFFF;
  localparam logic [15:0] VID_INVALID     = 16'hFFFF;
  localparam logic [23:0] CLASS_NVME      = 24'h01_08_02;
  localparam logic [7:0]  CAP_OFF_MIN     = 8'h40;       // capabilities live above the header
  localparam logic [7:0]  CAP_OFF_MAX     = 8'hF0;       // leave room for cap+8
  localparam logic [7:0]  CAP_PTR_ALIGN   = 8'hFC;       // pointers are DW aligned
  localparam logic [7:0]  OFF_DEVCAP      = 8'h04;       // PCIe cap: Device Capabilities
  localparam logic [7:0]  OFF_DEVCTL      = 8'h08;       // PCIe cap: Device Control
  localparam int unsigned CAP_WALK_MAX    = 48;

  // DevCap fields
  localparam int unsigned DEVCAP_ETFS_BIT = 5;
  // DevCtl fields
  localparam logic [15:0] DEVCTL_MASK     = 16'h71E0;    // MRRS[14:12], ETE[8], MPS[7:5]
  localparam int unsigned DEVCTL_MPS_LSB  = 5;
  localparam int unsigned DEVCTL_ETE_BIT  = 8;
  localparam int unsigned DEVCTL_MRRS_LSB = 12;

  // ---------------------------------------------------------------------------
  // One micro-step = one config request
  // ---------------------------------------------------------------------------
  typedef enum logic [4:0] {
    M_IDLE,
    M_VID,                      // step 1
    M_CLASS,                    // step 2
    M_B0_SIZE_W, M_B0_SIZE_R,   // step 3
    M_B1_SIZE_W, M_B1_SIZE_R,
    M_B0_SET,    M_B1_SET,      // step 4
    M_CAP_PTR,   M_CAP_WALK,    // step 5
    M_DEVCAP,                   // step 6
    M_DEVCTL_R,  M_DEVCTL_W,    // step 7
    M_CMD_W,                    // step 8
    M_DONE,
    M_FAIL
  } micro_t;

  // phase inside a micro-step
  typedef enum logic [1:0] {
    P_ISSUE,       // req_valid until req_ready
    P_WAIT,        // wait for the response
    P_CRS_WAIT,    // CRS: delay before retrying
    P_EVAL         // response data available in rdata_q
  } phase_t;

  micro_t      m_q;
  phase_t      ph_q;
  logic        start_q;
  logic [31:0] rdata_q;
  logic [7:0]  ptr_q;            // capability pointer being read
  logic [7:0]  walk_cnt;
  logic [31:0] wdata_q;          // prepared DevCtl write data
  logic [31:0] crs_retry_cnt;
  logic [31:0] crs_delay_cnt;

  // ---- step number of a micro-step ----
  function automatic logic [7:0] step_of(input micro_t m);
    case (m)
      M_VID:                          return 8'd1;
      M_CLASS:                        return 8'd2;
      M_B0_SIZE_W, M_B0_SIZE_R,
      M_B1_SIZE_W, M_B1_SIZE_R:       return 8'd3;
      M_B0_SET, M_B1_SET:             return 8'd4;
      M_CAP_PTR, M_CAP_WALK:          return 8'd5;
      M_DEVCAP:                       return 8'd6;
      M_DEVCTL_R, M_DEVCTL_W:         return 8'd7;
      M_CMD_W:                        return 8'd8;
      default:                        return 8'd0;
    endcase
  endfunction

  // ---- request decode (comb): op / byte offset / data / BE of the micro-step ----
  logic [7:0] off;
  always_comb begin
    req_op    = OP_CFGRD0;
    off       = 8'h00;
    req_wdata = 32'h0;
    req_be    = BE_ALL;
    case (m_q)
      M_VID:       off = CFG_VID_DID;
      M_CLASS:     off = CFG_CLASS;
      M_B0_SIZE_W: begin req_op = OP_CFGWR0; off = CFG_BAR0; req_wdata = ALL_ONES; end
      M_B0_SIZE_R: off = CFG_BAR0;
      M_B1_SIZE_W: begin req_op = OP_CFGWR0; off = CFG_BAR1; req_wdata = ALL_ONES; end
      M_B1_SIZE_R: off = CFG_BAR1;
      M_B0_SET:    begin req_op = OP_CFGWR0; off = CFG_BAR0; req_wdata = BAR0_ADDR; end
      M_B1_SET:    begin req_op = OP_CFGWR0; off = CFG_BAR1; req_wdata = 32'h0; end
      M_CAP_PTR:   off = CFG_CAP_PTR;
      M_CAP_WALK:  off = ptr_q;
      M_DEVCAP:    off = pcie_cap_off + OFF_DEVCAP;
      M_DEVCTL_R:  off = pcie_cap_off + OFF_DEVCTL;
      M_DEVCTL_W:  begin req_op = OP_CFGWR0; off = pcie_cap_off + OFF_DEVCTL;
                         req_wdata = wdata_q; req_be = BE_LO2; end
      M_CMD_W:     begin req_op = OP_CFGWR0; off = CFG_COMMAND;
                         req_wdata = CMD_MSE_BME_INTXDIS; req_be = BE_LO2; end
      default: ;
    endcase
    // H2 layout: {bus,dev,fn} = SSD_BDF, 4'b0, extreg = 0, reg = off[7:2], 2'b0
    req_addr = {SSD_BDF, 4'b0000, 4'b0000, off[7:2], 2'b00};
  end

  assign req_valid = (ph_q == P_ISSUE) && (m_q != M_IDLE) && (m_q != M_DONE) && (m_q != M_FAIL);

  // ---- DevCtl value to write (comb from the just-read dword) ----
  logic [15:0] devctl_new;
  always_comb begin
    devctl_new = (rdata_q[15:0] & ~DEVCTL_MASK)
               | (16'(mps_code)     << DEVCTL_MPS_LSB)
               | (16'(MRRS_CODE)  << DEVCTL_MRRS_LSB)
               | (16'(ext_tag_en)   << DEVCTL_ETE_BIT);
  end

  // ---- helper: go to the next micro-step ----
  micro_t m_next;
  always_comb begin
    case (m_q)
      M_VID:       m_next = M_CLASS;
      M_CLASS:     m_next = M_B0_SIZE_W;
      M_B0_SIZE_W: m_next = M_B0_SIZE_R;
      M_B0_SIZE_R: m_next = M_B1_SIZE_W;
      M_B1_SIZE_W: m_next = M_B1_SIZE_R;
      M_B1_SIZE_R: m_next = M_B0_SET;
      M_B0_SET:    m_next = M_B1_SET;
      M_B1_SET:    m_next = M_CAP_PTR;
      M_CAP_PTR:   m_next = M_CAP_WALK;
      M_CAP_WALK:  m_next = M_DEVCAP;
      M_DEVCAP:    m_next = M_DEVCTL_R;
      M_DEVCTL_R:  m_next = M_DEVCTL_W;
      M_DEVCTL_W:  m_next = M_CMD_W;
      M_CMD_W:     m_next = M_DONE;
      default:     m_next = M_FAIL;
    endcase
  end

  // capability pointer validity
  function automatic bit cap_ptr_ok(input logic [7:0] p);
    return (p >= CAP_OFF_MIN) && (p <= CAP_OFF_MAX);
  endfunction

  // ---------------------------------------------------------------------------
  // Sequencer
  // ---------------------------------------------------------------------------
  // done / fail registers and their start-edge mask
  logic done_r, fail_r;
  wire  start_evt = start && !start_q && (m_q == M_IDLE || m_q == M_DONE || m_q == M_FAIL);
  assign done = done_r & ~start_evt;
  assign fail = fail_r & ~start_evt;

  always_ff @(posedge clk) begin
    if (rst) begin
      m_q            <= M_IDLE;
      ph_q           <= P_ISSUE;
      start_q        <= 1'b0;
      done_r           <= 1'b0;
      fail_r           <= 1'b0;
      fail_step      <= 8'h00;
      mps_code       <= 3'd0;
      ext_tag_en     <= 1'b0;
      vid_did        <= 32'h0;
      bar0_size_mask <= 32'h0;
      class_ok       <= 1'b0;
      pcie_cap_off   <= 8'h00;
      devcap         <= 32'h0;
      rdata_q        <= 32'h0;
      ptr_q          <= 8'h00;
      walk_cnt       <= 8'h00;
      wdata_q        <= 32'h0;
      crs_retry_cnt  <= '0;
      crs_delay_cnt  <= '0;
    end else begin
      start_q <= start;

      case (m_q)
        // ------------------------------------------------------------------
        M_IDLE, M_DONE, M_FAIL: begin
          if (start && !start_q) begin
            done_r          <= 1'b0;
            fail_r          <= 1'b0;
            fail_step     <= 8'h00;
            class_ok      <= 1'b0;
            crs_retry_cnt <= '0;
            m_q           <= M_VID;
            ph_q          <= P_ISSUE;
          end
        end

        // ------------------------------------------------------------------
        default: begin
          case (ph_q)
            P_ISSUE: if (req_ready) ph_q <= P_WAIT;

            P_WAIT: if (rsp_valid) begin
              if (rsp_status == RSP_SC) begin
                rdata_q <= rsp_rdata;
                ph_q    <= P_EVAL;
              end else if (rsp_status == RSP_CRS && crs_retry_cnt < CRS_RETRY_MAX) begin
                crs_retry_cnt <= crs_retry_cnt + 32'd1;
                crs_delay_cnt <= CRS_RETRY_DELAY_CYCLES;
                ph_q          <= P_CRS_WAIT;
              end else begin                       // UR / CA / TIMEOUT / UNEXPECTED / CRS exhausted
                fail_r      <= 1'b1;
                fail_step <= step_of(m_q);
                m_q       <= M_FAIL;
              end
            end

            P_CRS_WAIT: begin
              if (crs_delay_cnt == 32'd0) ph_q <= P_ISSUE;     // retry the same request
              else                        crs_delay_cnt <= crs_delay_cnt - 32'd1;
            end

            P_EVAL: begin
              // default: advance to the next micro-step
              m_q           <= m_next;
              ph_q          <= P_ISSUE;
              crs_retry_cnt <= '0;

              case (m_q)
                M_VID: begin
                  vid_did <= rdata_q;
                  if (rdata_q[15:0] == VID_INVALID) begin
                    fail_r <= 1'b1; fail_step <= step_of(m_q); m_q <= M_FAIL;
                  end
                end

                M_CLASS: class_ok <= (rdata_q[31:8] == CLASS_NVME);

                M_B0_SIZE_R: bar0_size_mask <= rdata_q;

                M_CAP_PTR: begin
                  ptr_q    <= rdata_q[7:0] & CAP_PTR_ALIGN;
                  walk_cnt <= 8'h00;
                  if (!cap_ptr_ok(rdata_q[7:0] & CAP_PTR_ALIGN)) begin
                    fail_r <= 1'b1; fail_step <= step_of(m_q); m_q <= M_FAIL;
                  end
                end

                M_CAP_WALK: begin
                  if (rdata_q[7:0] == PCIE_CAP_ID) begin
                    pcie_cap_off <= ptr_q;                     // found: go to step 6
                  end else begin
                    // not it: follow the next pointer (stay in M_CAP_WALK)
                    ptr_q    <= rdata_q[15:8] & CAP_PTR_ALIGN;
                    walk_cnt <= walk_cnt + 8'd1;
                    m_q      <= M_CAP_WALK;
                    if (!cap_ptr_ok(rdata_q[15:8] & CAP_PTR_ALIGN) ||
                        (int'(walk_cnt) + 1 >= CAP_WALK_MAX)) begin
                      fail_r <= 1'b1; fail_step <= step_of(m_q); m_q <= M_FAIL;
                    end
                  end
                end

                M_DEVCAP: begin
                  devcap     <= rdata_q;
                  mps_code   <= (rdata_q[2:0] < MPS_CAP_CODE) ? rdata_q[2:0] : MPS_CAP_CODE;
                  ext_tag_en <= rdata_q[DEVCAP_ETFS_BIT] & EXT_TAG_ALLOW;
                end

                M_DEVCTL_R: wdata_q <= {16'h0000, devctl_new};

                M_CMD_W: done_r <= 1'b1;

                default: ;
              endcase
            end

            default: ph_q <= P_ISSUE;
          endcase
        end
      endcase
    end
  end

endmodule
