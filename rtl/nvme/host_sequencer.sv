// =============================================================================
// host_sequencer.sv - top-level control FSM of the NVMe host.  ARCHITECTURE.md 5.17
//
//   IDLE -> LINK -> ENUM -> RPCFG -> ADMIN_INIT -> RUN -> DRAIN -> FLUSH -> SHUTDOWN -> DONE
//             \_______________ any failure _______________/                  ERROR
//
//  IDLE        waits for `start` (level).  Refuses to leave while `link_lost` is set (see
//              "Link loss" below) and reports E_LINK_LOST instead.
//  LINK        waits for link_ready (PCIe link up + 100 ms settle, from link_bringup).
//              `link_fail` (link-up timeout, retries exhausted) -> ERROR / E_LINK_TIMEOUT.
//  ENUM        pulses enum_start; pcie_enum walks the SSD config space.  done / fail.
//  RPCFG       pulses rpcfg_start; rp_cfg_init programs the Root Port (BME, bus numbers, MPS).
//  ADMIN_INIT  pulses init_start; nvme_admin enables the controller, runs Identify etc.
//  RUN         we_enable=1: the write engine turns full ring slots into Write commands.
//              Leaves on `stop` (-> DRAIN) or on io_timeout / lba_range_err (-> FLUSH, error).
//  DRAIN       we_enable stays 1 until every ring slot has been written and completed
//              (all_slots_free held for DRAIN_STABLE cycles).  The user must have stopped the
//              data stream before `stop`; a partially filled slot is never written.
//              DRAIN always terminates: io_timeout (a command never completed) or
//              lba_range_err (namespace full: the remaining slots are never submitted) end
//              it at once with E_IO_TIMEOUT / E_LBA_RANGE and the shutdown still follows.
//  FLUSH       flush_req=1 until flush_done (NVMe Flush so that cached data is persistent).
//              The write engine skips the Flush (flush_done + flush_skipped) when commands
//              are still busy after an io_timeout.
//  SHUTDOWN    pulses shutdown_start; nvme_admin deletes the I/O queues, sets CC.SHN and
//              waits for SHST=10b.  shutdown_done ends it.  It never waits for ring slots.
//  DONE        `done`=1 = the SSD was shut down in an orderly way (safe to power off).  If
//              error_code != 0 the run itself did not complete normally (E_IO_TIMEOUT /
//              E_LBA_RANGE recorded in RUN/DRAIN).  Back to IDLE once `start` is released.
//  ERROR       sticky until `start` is released; error_code = failing stage (E_*), and
//              error_detail = the fail_step reported by the failing module.  No shutdown
//              is attempted from ERROR (the SSD may still be enabled).
//
// A second run without a reset is possible: DONE/ERROR -> (start low) IDLE -> (start high) LINK ...
// The completion levels of the previous run may still be visible when a stage is entered, so
// they are IGNORED in the cycle of the start pulse (M2); the submodules clear done/fail in
// that very cycle as well.  (nvme_host_core also restarts the write engine on init_start.)
//
// Link loss (M3).  link_lost is a STICKY flag generated in link_bringup (link_clk domain,
// cleared only by the board reset) and synchronised into user_clk by nvme_host_core.  A link
// drop in an active state sends the sequencer to ERROR (E_LINK_LOST, error_detail = state of
// the drop).  The pcie_7x IP then resets the whole host core (user_reset), so the sequencer is
// back in IDLE a moment later - but IDLE keeps reporting E_LINK_LOST and does not start while
// link_lost is set, even when `start` is still high.  A recording is therefore never silently
// restarted from START_LBA over data that is already on the SSD; only a board reset clears the
// condition.
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
  input  logic       link_lost,          // sticky (survives user_reset), synchronised into user_clk
  input  logic       link_fail,          // sticky link-up timeout from link_bringup
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
  input  logic       io_timeout,         // write engine: a command / the Flush did not complete in time
  input  logic       lba_range_err,      // write engine: namespace end reached, submission stopped

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
  localparam logic [7:0] E_NONE         = 8'h00;
  localparam logic [7:0] E_LINK_LOST    = 8'h01;   // link dropped after link-up (sticky until board reset)
  localparam logic [7:0] E_ENUM         = 8'h02;   // pcie_enum failed (detail = its fail_step)
  localparam logic [7:0] E_ADMIN        = 8'h03;   // nvme_admin init failed (detail = its fail_step)
  localparam logic [7:0] E_IO_TIMEOUT   = 8'h04;   // an I/O command / the Flush never completed (detail = state)
  localparam logic [7:0] E_LBA_RANGE    = 8'h05;   // namespace too small for the stream (detail = state)
  localparam logic [7:0] E_LINK_TIMEOUT = 8'h06;   // link never came up (link_bringup retries exhausted)

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
  // link_lost aborts every state that is not idle / terminal
  wire lost_abort  = link_lost && (cur != ST_IDLE) && (cur != ST_DONE) && (cur != ST_ERROR);
  // completion levels of a stage are valid only after the cycle of its start pulse
  wire fresh       = !entered;

  always_ff @(posedge clk) begin
    if (rst) begin
      cur          <= ST_IDLE;
      prev         <= ST_IDLE;
      error_code   <= E_NONE;
      error_detail <= 8'h00;
      stable_cnt   <= 16'd0;
    end else begin
      prev <= cur;

      if (lost_abort || (link_needed && !link_ready)) begin
        cur          <= ST_ERROR;
        error_code   <= E_LINK_LOST;
        error_detail <= {4'h0, cur};          // state in which the link was lost
      end else begin
        case (cur)
          ST_IDLE: begin                       // the last error stays visible until the next start
            if (link_lost) begin               // sticky: survives user_reset, never auto-restart
              error_code <= E_LINK_LOST;
            end else if (start) begin
              error_code   <= E_NONE;
              error_detail <= 8'h00;
              cur          <= ST_LINK;
            end
          end

          ST_LINK: begin
            if (link_fail) begin
              cur          <= ST_ERROR;
              error_code   <= E_LINK_TIMEOUT;
              error_detail <= 8'h00;
            end else if (link_ready) begin
              cur <= ST_ENUM;
            end
          end

          ST_ENUM: if (fresh) begin
            if (enum_fail) begin
              cur          <= ST_ERROR;
              error_code   <= E_ENUM;
              error_detail <= enum_fail_step;
            end else if (enum_done) begin
              cur <= ST_RPCFG;
            end
          end

          ST_RPCFG: if (fresh && rpcfg_done) cur <= stop ? ST_DONE : ST_ADMIN_INIT;

          ST_ADMIN_INIT: if (fresh) begin
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
            if (io_timeout) begin                 // go to the shutdown path at once
              cur          <= ST_FLUSH;
              error_code   <= E_IO_TIMEOUT;
              error_detail <= {4'h0, cur};
            end else if (lba_range_err) begin
              cur          <= ST_FLUSH;
              error_code   <= E_LBA_RANGE;
              error_detail <= {4'h0, cur};
            end else if (stop) begin
              cur <= ST_DRAIN;
            end
          end

          ST_DRAIN: begin
            if (io_timeout) begin                 // DRAIN must terminate; Flush/shutdown still follow
              cur          <= ST_FLUSH;
              error_code   <= E_IO_TIMEOUT;
              error_detail <= {4'h0, cur};
            end else if (lba_range_err) begin
              cur          <= ST_FLUSH;
              error_code   <= E_LBA_RANGE;
              error_detail <= {4'h0, cur};
            end else if (all_slots_free) begin
              if (stable_cnt >= 16'(DRAIN_STABLE)) cur <= ST_FLUSH;
              else                                 stable_cnt <= stable_cnt + 1'b1;
            end else begin
              stable_cnt <= 16'd0;
            end
          end

          ST_FLUSH:    if (fresh && flush_done)    cur <= ST_SHUTDOWN;

          ST_SHUTDOWN: if (fresh && shutdown_done) cur <= ST_DONE;

          ST_DONE:  if (!start) cur <= ST_IDLE;
          ST_ERROR: if (!start) cur <= ST_IDLE;

          default: cur <= ST_ERROR;
        endcase
      end
    end
  end

endmodule
