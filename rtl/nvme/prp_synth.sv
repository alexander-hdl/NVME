// =============================================================================
// prp_synth.sv — PRP list "memory" computed by arithmetic (ARCHITECTURE.md §5.8)
//
// Every I/O command writes SLOT_BYTES from a data-ring slot k. The SSD needs
// the physical addresses of all 4 KiB pages: PRP1 = slot base (in the SQE),
// PRP2 = address of a PRP list in the PRP window, whose entries are
//     entry(j) = DATA_BASE + k*SLOT_BYTES + (j+1)*4096         (page j+1 of the slot)
// -- a pure function of the address being read, so no RAM is needed.
//
// Read address (region-relative to PRP_BASE, 16-byte aligned):
//     k = rd_addr[31:12]   slot (list page k belongs to slot k)
//     i = rd_addr[11:3]    entry index of the LOWER qword of the beat
// The 128-bit beat is {entry(i+1), entry(i)}, each a 64-bit little-endian value
// whose upper DW is 0. Entries past the last page of the slot (j+1 >= pages per
// slot) and slots >= N_SLOTS return 0 (a correct SSD never fetches them).
//
// Two pipeline stages, so rd_data is valid exactly 2 cycles after rd_en like
// every other memory source:
//   stage 1: slot base = DATA_BASE + k*SLOT_BYTES, entry indices
//   stage 2: add (j+1)*4096, range check, build the beat
// =============================================================================
module prp_synth
  import nvme_host_pkg::*;
#(
  parameter int unsigned N_SLOTS    = N_SLOTS_DEF,
  parameter int unsigned SLOT_BYTES = SLOT_BYTES_DEF
)(
  input  logic         clk,
  input  logic         rd_en,
  input  logic [31:0]  rd_addr,
  output logic [127:0] rd_data
);

  localparam int unsigned PAGES = SLOT_BYTES / 4096;     // pages per slot (PRP entries incl. PRP1)

  // ---- stage 1 ----
  logic        s1_valid, s1_slot_ok;
  logic [31:0] s1_base;
  logic [10:0] s1_page0, s1_page1;      // (i+1) and (i+2): page number inside the slot

  always_ff @(posedge clk) begin
    s1_valid <= rd_en;
    if (rd_en) begin
      s1_slot_ok <= (rd_addr[31:12] < 20'(N_SLOTS));
      s1_base    <= DATA_BASE + 32'(rd_addr[31:12]) * 32'(SLOT_BYTES);
      s1_page0   <= 11'(rd_addr[11:3]) + 11'd1;
      s1_page1   <= 11'(rd_addr[11:3]) + 11'd2;
    end
  end

  // ---- stage 2 ----
  function automatic logic [63:0] entry(input logic ok, input logic [31:0] base, input logic [10:0] page);
    logic [31:0] a;
    a = base + {9'd0, page, 12'd0};
    if (ok && (int'(page) < PAGES)) return {32'd0, a};
    return 64'd0;
  endfunction

  always_ff @(posedge clk) begin
    if (s1_valid)
      rd_data <= {entry(s1_slot_ok, s1_base, s1_page1), entry(s1_slot_ok, s1_base, s1_page0)};
  end

endmodule
