// =============================================================================
// host_sequencer.sv - top-level control FSM of the NVMe host.  ARCHITECTURE.md 5.17
//
//   IDLE -> LINK -> ENUM -> RPCFG -> ADMIN_INIT -> RUN -> DRAIN -> FLUSH -> SHUTDOWN -> DONE
//             \_______________ any failure _______________/                  ERROR
//
//  IDLE        waits for `start` (level).
//  LINK        waits for link_ready (PCIe link up + 100 ms settle, from link_bringup).
//  ENUM        pulses enum_start; pcie_enum walks the SSD config space.  done / fail.
//  RPCFG       pulses rpcfg_start; rp_cfg_init programs the Root Port (BME, bus numbers, MPS).
//  ADMIN_INIT  pulses init_start; nvme_admin enables the controller, runs Identify etc.
//  RUN         we_enable=1: the write engine turns full ring slots into Write commands.
//              Leaves on `stop`.
//  DRAIN       we_enable stays 1 until every ring slot has been written and completed
//              (all_slots_free held for DRAIN_STABLE cycles).  The user must have stopped the
//              data stream before `stop`; a partially filled slot is never written.
//  FLUSH       flush_req=1 until flush_done (NVMe Flush so that cached data is persistent).
//  SHUTDOWN    pulses shutdown_start; nvme_admin deletes the I/O queues, sets CC.SHN and
//              waits for SHST=10b.  shutdown_done ends it.
//  DONE        `done`=1.  Back to IDLE once `start` is released.
//  ERROR       sticky until `start` is released; error_code = failing stage (E_*), and
//              error_detail = the fail_step reported by the failing module.
//
// Start pulses (enum_start, rpcfg_start, init_start, shutdown_start) are one-cycle pulses
// generated on entering the corresponding state.  we_enable / flush_req are levels.
// If `stop` is already set when the bring-up reaches RPCFG, the sequencer skips the NVMe
// init and goes straight to DONE.  The link is supervised in every active state:
// link_ready dropping raises E_LINK_LOST.
// =============================================================================
module host_sequencer #(
  parameter int unsigned DRAIN_STABLE = 16        // cycles all_slots_free must hold before FLUSH
) (
  input  logic       clk,
  input  logic       rst,

  input  logic       start,
  input  logic       stop,

  input  logic       link_ready,
  input  logic       enum_done,
  input  logic       enum_fail,
  input  logic [7:0] enum_fail_step,
  input  logic       rpcfg_done,
  input  logic       init_done,
  input  logic       init_fail,
  input  logic [7:0] init_fail_step,
  input  logic       flush_done,
  input  logic       shutdown_done,
  input  logic       all_slots_free,

  output logic       enum_start,
  output logic       rpcfg_start,
  output logic       init_start,
  output logic       we_enable,
  output logic       flush_req,
  output logic       shutdown_start,

  output logic [3:0] state,
  output logic [7:0] error_code,
  output logic [7:0] error_detail,
  output logic       running,
  output logic       done
);

  // State encoding (also exported on `state`)
  localparam logic [3:0] ST_IDLE       = 4'd0;
  localparam logic [3:0] ST_LINK       = 4'd1;
  localparam logic [3:0] ST_ENUM       = 4'd2;
  localparam logic [3:0] ST_RPCFG      = 4'd3;
  localparam logic [3:0] ST_ADMIN_INIT = 4'd4;
  localparam logic [3:0] ST_RUN        = 4'd5;
  localparam logic [3:0] ST_DRAIN      = 4'd6;
  localparam logic [3:0] ST_FLUSH      = 4'd7;
  localparam logic [3:0] ST_SHUTDOWN   = 4'd8;
  localparam logic [3:0] ST_DONE       = 4'd9;
  localparam logic [3:0] ST_ERROR      = 4'd15;

  // error_code values
  localparam logic [7:0] E_NONE      = 8'h00;
  localparam logic [7:0] E_LINK_LOST = 8'h01;   // link_ready dropped after link-up
  localparam logic [7:0] E_ENUM      = 8'h02;   // pcie_enum failed (detail = its fail_step)
  localparam logic [7:0] E_ADMIN     = 8'h03;   // nvme_admin init failed (detail = its fail_step)

  logic [3:0]  cur, prev;     // current state and the state of the previous cycle
  logic [15:0] stable_cnt;    // DRAIN: consecutive cycles with all_slots_free

  assign state = cur;

  // One-cycle start pulses on state entry
  wire entered = (cur != prev);
  assign enum_start     = entered && (cur == ST_ENUM);
  assign rpcfg_start    = entered && (cur == ST_RPCFG);
  assign init_start     = entered && (cur == ST_ADMIN_INIT);
  assign shutdown_start = entered && (cur == ST_SHUTDOWN);

  // Levels
  assign we_enable = (cur == ST_RUN) || (cur == ST_DRAIN);
  assign flush_req = (cur == ST_FLUSH);
  assign running   = (cur == ST_RUN);
  assign done      = (cur == ST_DONE);

  // The link must stay up in every active state
  wire link_needed = (cur != ST_IDLE) && (cur != ST_LINK) && (cur != ST_DONE) && (cur != ST_ERROR);

  always_ff @(posedge clk) begin
    if (rst) begin
      cur          <= ST_IDLE;
      prev         <= ST_IDLE;
      error_code   <= E_NONE;
      error_detail <= 8'h00;
      stable_cnt   <= 16'd0;
    end else begin
      prev <= cur;

      if (link_needed && !link_ready) begin
        cur          <= ST_ERROR;
        error_code   <= E_LINK_LOST;
        error_detail <= {4'h0, cur};          // state in which the link was lost
      end else begin
        case (cur)
          ST_IDLE: begin                       // the last error stays visible until the next start
            if (start) begin
              error_code   <= E_NONE;
              error_detail <= 8'h00;
              cur          <= ST_LINK;
            end
          end

          ST_LINK: if (link_ready) cur <= ST_ENUM;

          ST_ENUM: begin
            if (enum_fail) begin
              cur          <= ST_ERROR;
              error_code   <= E_ENUM;
              error_detail <= enum_fail_step;
            end else if (enum_done) begin
              cur <= ST_RPCFG;
            end
          end

          ST_RPCFG: if (rpcfg_done) cur <= stop ? ST_DONE : ST_ADMIN_INIT;

          ST_ADMIN_INIT: begin
            if (init_fail) begin
              cur          <= ST_ERROR;
              error_code   <= E_ADMIN;
              error_detail <= init_fail_step;
            end else if (init_done) begin
              cur <= ST_RUN;
            end
          end

          ST_RUN: begin
            stable_cnt <= 16'd0;
            if (stop) cur <= ST_DRAIN;
          end

          ST_DRAIN: begin
            if (all_slots_free) begin
              if (stable_cnt >= 16'(DRAIN_STABLE)) cur <= ST_FLUSH;
              else                                 stable_cnt <= stable_cnt + 1'b1;
            end else begin
              stable_cnt <= 16'd0;
            end
          end

          ST_FLUSH:    if (flush_done)    cur <= ST_SHUTDOWN;

          ST_SHUTDOWN: if (shutdown_done) cur <= ST_DONE;

          ST_DONE:  if (!start) cur <= ST_IDLE;
          ST_ERROR: if (!start) cur <= ST_IDLE;

          default: cur <= ST_ERROR;
        endcase
      end
    end
  end

endmodule
