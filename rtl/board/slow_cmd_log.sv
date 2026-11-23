// =============================================================================
// slow_cmd_log - debug log of slow NVMe data commands (board top, user_clk domain)
//
// Every completed data command (write engine cpl_evt / cpl_lat) whose latency is >= thr cycles
// (thr != 0) is counted in `count`.  The first DEPTH of them are stored in a BRAM:
//   entry = {t_cpl[39:0], idx[31:0], lat[31:0]}  (104 bits)
//     t_cpl : run timer at the completion (cycles of user_clk since RUN, run_cyc of the board top)
//     idx   : commands completed before this one (stat_cmds_done, low 32 bits) -> position in the stream
//     lat   : command latency in cycles (submit .. CQE reap)
// `rd_addr` selects the entry presented on `rd_data` (registered, 1-cycle read): quasi-static, set
// from JTAG between reads.  rst (the board / VIO reset) clears the count and starts a new log.
// =============================================================================
module slow_cmd_log #(
  parameter int unsigned DEPTH = 1024
) (
  input  logic                     clk,
  input  logic                     rst,
  input  logic                     evt,
  input  logic [31:0]              lat,
  input  logic [39:0]              t_now,
  input  logic [31:0]              idx,
  input  logic [31:0]              thr,
  input  logic [$clog2(DEPTH)-1:0] rd_addr,
  output logic [103:0]             rd_data,
  output logic [31:0]              count
);
  localparam int unsigned AW = $clog2(DEPTH);

  (* ram_style = "block" *) logic [103:0] mem [DEPTH];

  logic hit;
  assign hit = evt && (thr != 32'd0) && (lat >= thr);

  always_ff @(posedge clk) begin
    if (rst) begin
      count <= 32'd0;
    end else if (hit) begin
      if (count != 32'hFFFF_FFFF) count <= count + 32'd1;
    end
  end

  always_ff @(posedge clk) begin
    if (!rst && hit && count < DEPTH) mem[count[AW-1:0]] <= {t_now, idx, lat};
    rd_data <= mem[rd_addr];
  end
endmodule
