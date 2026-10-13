// =============================================================================
// rp_cfg_init.sv - program the Root Port's OWN config space (ARCHITECTURE.md 5.13)
// -----------------------------------------------------------------------------
// The pcie_7x Root Port must be configured through its management port
// ("cfg_mgmt", PG054 Fig 3-61/3-62). Handshake: put address / data / byte
// enable and wr_en (or rd_en) on the port and HOLD them until
// cfg_mgmt_rd_wr_done pulses; then drop the enable for at least one cycle.
//
// Sequence (dword addresses from nvme_host_pkg):
//   1. WR RP_DW_COMMAND = 0x0000_0006  BE 0001   Command.MSE + BME (BME is a must:
//                                                 without it upstream MRd get UR)
//   2. WR RP_DW_BUSNUM  = 0x0001_0100  BE 0111   primary 0, secondary 1, subordinate 1
//   3. RD RP_DW_DEVCTL                           read Device Control
//   4. WR RP_DW_DEVCTL  = modified     BE 0011   [7:5]=mps_code, [8]=ext_tag_en,
//                                                 other bits (incl. MRRS) preserved
//
// start is edge-sensitive (a rising edge begins a run); done goes high when the
// sequence has finished and stays high until the next run starts.  It drops in the very
// cycle of the start edge (done = done_r & ~start_evt), so a sequencer never sees the
// done flag of the previous run in the cycle in which it issues start.
// mps_code / ext_tag_en are sampled when the modified DevCtl value is built.
// =============================================================================
module rp_cfg_init
  import nvme_host_pkg::*;
(
  input  logic        clk,
  input  logic        rst,

  input  logic        start,
  output logic        done,       // level; cleared in the SAME cycle as the start edge (comb. mask)
  input  logic [2:0]  mps_code,
  input  logic        ext_tag_en,

  // pcie_7x v3.3 cfg_mgmt port
  output logic [31:0] cfg_mgmt_di,
  output logic [3:0]  cfg_mgmt_byte_en,
  output logic [9:0]  cfg_mgmt_dwaddr,
  output logic        cfg_mgmt_wr_en,
  output logic        cfg_mgmt_rd_en,
  input  logic [31:0] cfg_mgmt_do,
  input  logic        cfg_mgmt_rd_wr_done,
  output logic        cfg_mgmt_wr_readonly,
  output logic        cfg_mgmt_wr_rw1c_as_rw
);

  localparam logic [31:0] CMD_MSE_BME    = 32'h0000_0006;
  localparam logic [3:0]  BE_CMD         = 4'b0001;
  localparam logic [31:0] BUSNUM_VALUE   = 32'h0001_0100;  // sub=1, sec=1, pri=0
  localparam logic [3:0]  BE_BUSNUM      = 4'b0111;
  localparam logic [3:0]  BE_DEVCTL      = 4'b0011;
  localparam int unsigned DEVCTL_MPS_LSB = 5;               // [7:5]
  localparam int unsigned DEVCTL_ETE_BIT = 8;               // Extended Tag Enable
  localparam logic [15:0] DEVCTL_MASK    = 16'h01E0;        // bits we modify: [8],[7:5]

  typedef enum logic [3:0] {
    S_IDLE,
    S_WR_CMD,   S_GAP_CMD,
    S_WR_BUS,   S_GAP_BUS,
    S_RD_DEV,   S_GAP_RD,
    S_WR_DEV,   S_GAP_DEV,
    S_DONE
  } state_t;

  state_t state;
  logic        start_q;
  logic [31:0] devctl_rd_q;

  assign cfg_mgmt_wr_readonly   = 1'b0;
  assign cfg_mgmt_wr_rw1c_as_rw = 1'b0;

  // ---- modified DevCtl value (low 16 bits written, BE 0011) ----
  logic [15:0] devctl_new;
  always_comb begin
    devctl_new = (devctl_rd_q[15:0] & ~DEVCTL_MASK)
               | (16'(mps_code) << DEVCTL_MPS_LSB)
               | (16'(ext_tag_en) << DEVCTL_ETE_BIT);
  end

  // ---- port drive: all comb from the state register, held until done ----
  always_comb begin
    cfg_mgmt_di      = 32'h0;
    cfg_mgmt_byte_en = 4'b0000;
    cfg_mgmt_dwaddr  = 10'h000;
    cfg_mgmt_wr_en   = 1'b0;
    cfg_mgmt_rd_en   = 1'b0;
    case (state)
      S_WR_CMD: begin
        cfg_mgmt_dwaddr = RP_DW_COMMAND;
        cfg_mgmt_di     = CMD_MSE_BME;
        cfg_mgmt_byte_en = BE_CMD;
        cfg_mgmt_wr_en  = 1'b1;
      end
      S_WR_BUS: begin
        cfg_mgmt_dwaddr = RP_DW_BUSNUM;
        cfg_mgmt_di     = BUSNUM_VALUE;
        cfg_mgmt_byte_en = BE_BUSNUM;
        cfg_mgmt_wr_en  = 1'b1;
      end
      S_RD_DEV: begin
        cfg_mgmt_dwaddr = RP_DW_DEVCTL;
        cfg_mgmt_byte_en = 4'b1111;
        cfg_mgmt_rd_en  = 1'b1;
      end
      S_WR_DEV: begin
        cfg_mgmt_dwaddr = RP_DW_DEVCTL;
        cfg_mgmt_di     = {devctl_rd_q[31:16], devctl_new};
        cfg_mgmt_byte_en = BE_DEVCTL;
        cfg_mgmt_wr_en  = 1'b1;
      end
      default: ;
    endcase
  end

  logic done_r;
  wire  start_evt = start && !start_q && (state == S_IDLE || state == S_DONE);
  assign done = done_r & ~start_evt;

  always_ff @(posedge clk) begin
    if (rst) begin
      state       <= S_IDLE;
      start_q     <= 1'b0;
      done_r        <= 1'b0;
      devctl_rd_q <= '0;
    end else begin
      start_q <= start;
      case (state)
        S_IDLE, S_DONE: begin
          if (start && !start_q) begin
            done_r  <= 1'b0;
            state <= S_WR_CMD;
          end
        end

        S_WR_CMD:  if (cfg_mgmt_rd_wr_done) state <= S_GAP_CMD;
        S_GAP_CMD:                          state <= S_WR_BUS;     // one idle cycle

        S_WR_BUS:  if (cfg_mgmt_rd_wr_done) state <= S_GAP_BUS;
        S_GAP_BUS:                          state <= S_RD_DEV;

        S_RD_DEV: if (cfg_mgmt_rd_wr_done) begin
          devctl_rd_q <= cfg_mgmt_do;
          state       <= S_GAP_RD;
        end
        S_GAP_RD:                           state <= S_WR_DEV;

        S_WR_DEV:  if (cfg_mgmt_rd_wr_done) state <= S_GAP_DEV;
        S_GAP_DEV: begin
          done_r  <= 1'b1;
          state <= S_DONE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
