// =============================================================================
// req_mux.sv - two command ports -> one rp_requester (ARCHITECTURE.md 5.11)
// -----------------------------------------------------------------------------
//   port 0 : admin FSM (config + BAR0 register accesses)   - preferred
//   port 1 : write-engine doorbells (MWr)                   - never starved
//
// Both ports use the rp_requester command-port signature. The mux is a
// "one command in flight" arbiter:
//
//   1. while idle, pick a port: p0 wins, EXCEPT when both ports have a command
//      pending and the previous command was served from p0 -> then p1 goes.
//      So p1 waits at most for one p0 command.
//   2. the chosen command is passed combinationally to the requester;
//   3. the owner is remembered and the response (rsp_valid) is routed back to
//      it only; status/rdata are broadcast, qualified by pN_rsp_valid.
//   4. the next arbitration happens after the response of the current command.
//
// The requester side uses the prefix r_ ("to requester") to avoid confusion
// with the m_req_* TLP stream of rp_requester.
// =============================================================================
module req_mux (
  input  logic        clk,
  input  logic        rst,

  // ---- port 0 (admin) ----
  input  logic        p0_req_valid,
  output logic        p0_req_ready,
  input  logic [1:0]  p0_req_op,
  input  logic [31:0] p0_req_addr,
  input  logic [31:0] p0_req_wdata,
  input  logic [3:0]  p0_req_be,
  output logic        p0_rsp_valid,
  output logic [2:0]  p0_rsp_status,
  output logic [31:0] p0_rsp_rdata,

  // ---- port 1 (doorbells) ----
  input  logic        p1_req_valid,
  output logic        p1_req_ready,
  input  logic [1:0]  p1_req_op,
  input  logic [31:0] p1_req_addr,
  input  logic [31:0] p1_req_wdata,
  input  logic [3:0]  p1_req_be,
  output logic        p1_rsp_valid,
  output logic [2:0]  p1_rsp_status,
  output logic [31:0] p1_rsp_rdata,

  // ---- to rp_requester ----
  output logic        r_req_valid,
  input  logic        r_req_ready,
  output logic [1:0]  r_req_op,
  output logic [31:0] r_req_addr,
  output logic [31:0] r_req_wdata,
  output logic [3:0]  r_req_be,
  input  logic        r_rsp_valid,
  input  logic [2:0]  r_rsp_status,
  input  logic [31:0] r_rsp_rdata
);

  logic busy_q;        // a command has been handed to the requester, response pending
  logic owner_q;       // 0 = p0, 1 = p1 (valid while busy_q)
  logic last_p0_q;     // the most recently accepted command came from p0

  // ---- arbitration (only meaningful while idle) ----
  logic pick_p1;
  assign pick_p1 = p1_req_valid && (!p0_req_valid || last_p0_q);

  assign r_req_valid = !busy_q && (pick_p1 ? p1_req_valid : p0_req_valid);
  assign r_req_op    = pick_p1 ? p1_req_op    : p0_req_op;
  assign r_req_addr  = pick_p1 ? p1_req_addr  : p0_req_addr;
  assign r_req_wdata = pick_p1 ? p1_req_wdata : p0_req_wdata;
  assign r_req_be    = pick_p1 ? p1_req_be    : p0_req_be;

  assign p0_req_ready = !busy_q && r_req_ready && !pick_p1;
  assign p1_req_ready = !busy_q && r_req_ready &&  pick_p1;

  // ---- ownership tracking ----
  always_ff @(posedge clk) begin
    if (rst) begin
      busy_q    <= 1'b0;
      owner_q   <= 1'b0;
      last_p0_q <= 1'b0;
    end else begin
      if (r_req_valid && r_req_ready) begin       // command accepted
        busy_q    <= 1'b1;
        owner_q   <= pick_p1;
        last_p0_q <= !pick_p1;
      end else if (busy_q && r_rsp_valid) begin   // response delivered
        busy_q <= 1'b0;
      end
    end
  end

  // ---- response routing ----
  assign p0_rsp_valid  = r_rsp_valid && busy_q && !owner_q;
  assign p1_rsp_valid  = r_rsp_valid && busy_q &&  owner_q;
  assign p0_rsp_status = r_rsp_status;
  assign p1_rsp_status = r_rsp_status;
  assign p0_rsp_rdata  = r_rsp_rdata;
  assign p1_rsp_rdata  = r_rsp_rdata;

endmodule
