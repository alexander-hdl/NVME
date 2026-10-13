// =============================================================================
// sdp_ram.sv — simple dual-port RAM, byte-enable write, 2-cycle registered read.
//
//   * Port A (write): wr_en + per-byte wr_be, one clock.
//   * Port B (read) : rd_en/rd_addr are registered into the memory output
//                     (stage 1 = RAM output latch), then into a second register
//                     (stage 2 = BRAM output register).  rd_data therefore is
//                     valid 2 cycles after rd_en and HOLDS its value until the
//                     next read (both registers are enabled only by reads).
//   * No reset on the array -> Vivado infers block RAM (byte-write enables).
//   * Read-during-write at the same address returns the OLD data (read-first).
//   * The initial block zero-fills the array: gives BRAM INIT=0 in hardware and
//     avoids X propagation in simulation.
// =============================================================================
module sdp_ram #(
  parameter int DEPTH = 256,
  parameter int W     = 128
) (
  input  logic                       clk,
  input  logic                       wr_en,
  input  logic [W/8-1:0]             wr_be,
  input  logic [$clog2(DEPTH)-1:0]   wr_addr,
  input  logic [W-1:0]               wr_data,
  input  logic                       rd_en,
  input  logic [$clog2(DEPTH)-1:0]   rd_addr,
  output logic [W-1:0]               rd_data
);

  (* ram_style = "block" *) logic [W-1:0] mem [0:DEPTH-1];

  logic [W-1:0] rd_stage1_q;     // RAM output latch
  logic [W-1:0] rd_stage2_q;     // output register
  logic         rd_v1_q;         // stage-1 data is a real read

  // Plain 'always' (not always_ff): the initial block below also writes the
  // registers, which SystemVerilog forbids for always_ff variables.
  initial begin
    for (int i = 0; i < DEPTH; i++) mem[i] = '0;
    rd_stage1_q = '0;
    rd_stage2_q = '0;
    rd_v1_q     = 1'b0;
  end

  always @(posedge clk) begin
    if (wr_en) begin
      for (int b = 0; b < W/8; b++)
        if (wr_be[b]) mem[wr_addr][8*b +: 8] <= wr_data[8*b +: 8];
    end
    if (rd_en) rd_stage1_q <= mem[rd_addr];
    rd_v1_q <= rd_en;
    if (rd_v1_q) rd_stage2_q <= rd_stage1_q;
  end

  assign rd_data = rd_stage2_q;

endmodule
