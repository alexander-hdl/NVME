// =============================================================================
// tb_prp_synth — compares prp_synth against the formula of ARCHITECTURE.md §2/§5.8
//   entry(j) = DATA_BASE + k*SLOT_BYTES + (j+1)*4096, zero if j >= SLOT_BYTES/4096-1 or k >= N_SLOTS
//   rd_data = {entry(i+1), entry(i)}, valid exactly 2 cycles after rd_en.
// Two DUTs: the simulation size (4 x 16 KiB) and the default size (8 x 128 KiB).
// =============================================================================
module tb_prp_synth;
  import nvme_host_pkg::*;

  logic clk = 0;
  always #4 clk = ~clk;

  logic        rd_en = 0;
  logic [31:0] rd_addr = 0;
  logic [127:0] rd_data_s, rd_data_d;

  prp_synth #(.N_SLOTS(4), .SLOT_BYTES(16384))   dut_s (.clk(clk), .rd_en(rd_en), .rd_addr(rd_addr), .rd_data(rd_data_s));
  prp_synth #(.N_SLOTS(8), .SLOT_BYTES(131072))  dut_d (.clk(clk), .rd_en(rd_en), .rd_addr(rd_addr), .rd_data(rd_data_d));

  // reference model
  function automatic logic [63:0] ref_entry(input int unsigned nslots, input int unsigned slot_bytes,
                                            input logic [31:0] addr, input int unsigned lane);
    longint unsigned k, j, pages, v;
    k = addr[31:12];
    j = addr[11:3] + lane;
    pages = slot_bytes / 4096;
    if (k >= nslots || (j + 1) >= pages) return 64'd0;
    v = 64'(DATA_BASE) + k * slot_bytes + (j + 1) * 4096;
    return v;
  endfunction

  function automatic logic [127:0] ref_beat(input int unsigned nslots, input int unsigned slot_bytes,
                                            input logic [31:0] addr);
    return {ref_entry(nslots, slot_bytes, addr, 1), ref_entry(nslots, slot_bytes, addr, 0)};
  endfunction

  // 2-cycle expected-value pipeline
  logic [127:0] exp_s1, exp_d1, exp_s2, exp_d2;
  logic         v1, v2;
  int errors = 0, checks = 0;

  always @(posedge clk) begin
    v1 <= rd_en;       v2 <= v1;
    exp_s1 <= ref_beat(4, 16384, rd_addr);   exp_s2 <= exp_s1;
    exp_d1 <= ref_beat(8, 131072, rd_addr);  exp_d2 <= exp_d1;
    if (v2) begin
      checks++;
      if (rd_data_s !== exp_s2) begin
        errors++; $display("ERR small addr? exp=%h got=%h", exp_s2, rd_data_s);
      end
      if (rd_data_d !== exp_d2) begin
        errors++; $display("ERR default exp=%h got=%h", exp_d2, rd_data_d);
      end
    end
  end

  initial begin
    repeat (3) @(posedge clk);
    // directed: first/last list pages, slot 0, last slot, out of range slot
    foreach_dir: begin
      logic [31:0] da [12];
      da[0] = 32'h0000_0000;  da[1] = 32'h0000_0010; da[2] = 32'h0000_0FF0;
      da[3] = 32'h0000_1000;  da[4] = 32'h0000_3000; da[5] = 32'h0000_4000;
      da[6] = 32'h0000_7FF0;  da[7] = 32'h0000_8000; da[8] = 32'h0010_0000;
      da[9] = 32'hFFFF_FFF0;  da[10]= 32'h0000_0060; da[11]= 32'h0000_2FE0;
      for (int n = 0; n < 12; n++) begin
        @(negedge clk); rd_en = 1; rd_addr = da[n];
      end
      @(negedge clk); rd_en = 0;
    end
    // random, with gaps
    for (int n = 0; n < 20000; n++) begin
      @(negedge clk);
      rd_en = ($urandom_range(0, 3) != 0);
      case ($urandom_range(0, 2))
        0: rd_addr = ($urandom_range(0, 9) << 12) | ($urandom_range(0, 255) << 4);
        1: rd_addr = ($urandom_range(0, 9) << 12) | ($urandom_range(0, 255) << 4) | ($urandom_range(0, 1) << 11);
        default: rd_addr = $urandom & 32'hFFFF_FFF0;
      endcase
    end
    @(negedge clk); rd_en = 0;
    repeat (5) @(posedge clk);
    // spot value: slot 2, entry 0 of the default size = DATA_BASE + 2*128K + 4096
    @(negedge clk); rd_addr = 32'h0000_2000; rd_en = 1;
    @(negedge clk); rd_en = 0;
    @(negedge clk);
    if (rd_data_d[63:0] !== {32'd0, DATA_BASE + 32'd2*32'd131072 + 32'd4096}) begin
      errors++; $display("ERR spot entry(0) slot 2: %h", rd_data_d[63:0]);
    end
    if (rd_data_d[127:64] !== {32'd0, DATA_BASE + 32'd2*32'd131072 + 32'd8192}) begin
      errors++; $display("ERR spot entry(1) slot 2: %h", rd_data_d[127:64]);
    end
    $display("tb_prp_synth: %0d beats checked, %0d errors", checks, errors);
    if (errors != 0) $fatal(1, "TEST FAILED");
    $display("TEST PASSED");
    $finish;
  end
endmodule
