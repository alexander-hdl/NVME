// =============================================================================
// tdp_ram_2clk.sv — true dual-port RAM with independent clocks (WP0).
//
// Domains : A = a_clk (read/write), B = b_clk (read only).
// Ports   : A: a_clk, a_we, a_addr, a_wdata[W], a_rdata[W] ;
//           B: b_clk, b_en, b_addr, b_rdata[W].
// Latency : 2 cycles on both read ports (RAM output latch + output register), like sdp_ram.
//           A reads every cycle (read-first: a write returns the OLD data at the same
//           address).  B: b_rdata is loaded 2 cycles after b_en and keeps its value until
//           the next read (stage 1 only moves on b_en).
// Hazard  : reading B at the address being written by A in the same time window gives an
//           undefined (old or new) value; users keep the table static while it is read
//           (spec 4.3 X3: A writes only in IDLE).
// Init    : the array is zero-filled by an initial block (maps to BRAM INIT=0, no X in
//           simulation); no reset on the array.
// Spec    : STAND/RECORDER_SPEC.md 4.4 (WP0), X3.
// =============================================================================
module tdp_ram_2clk #(
  parameter int W     = 72,
  parameter int DEPTH = 64
) (
  input  logic                       a_clk,
  input  logic                       a_we,
  input  logic [$clog2(DEPTH)-1:0]   a_addr,
  input  logic [W-1:0]               a_wdata,
  output logic [W-1:0]               a_rdata,
  input  logic                       b_clk,
  input  logic                       b_en,
  input  logic [$clog2(DEPTH)-1:0]   b_addr,
  output logic [W-1:0]               b_rdata
);

  (* ram_style = "block" *) logic [W-1:0] mem [0:DEPTH-1];

  logic [W-1:0] a_s1_q, a_s2_q;
  logic [W-1:0] b_s1_q, b_s2_q;

  // Plain 'always' (not always_ff): initial block also writes the variables.
  initial begin
    for (int i = 0; i < DEPTH; i++) mem[i] = '0;
    a_s1_q = '0; a_s2_q = '0; b_s1_q = '0; b_s2_q = '0;
  end

  always @(posedge a_clk) begin
    if (a_we) mem[a_addr] <= a_wdata;
    a_s1_q <= mem[a_addr];
    a_s2_q <= a_s1_q;
  end

  always @(posedge b_clk) begin
    if (b_en) b_s1_q <= mem[b_addr];
    b_s2_q <= b_s1_q;
  end

  assign a_rdata = a_s2_q;
  assign b_rdata = b_s2_q;

endmodule
