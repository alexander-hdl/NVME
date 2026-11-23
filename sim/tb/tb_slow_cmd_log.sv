// tb_slow_cmd_log - unit test of rtl/board/slow_cmd_log.sv
`timescale 1ns/1ps
module tb_slow_cmd_log;
  logic clk = 0; always #4 clk = ~clk;
  logic rst = 1, evt = 0;
  logic [31:0] lat = 0, idx = 0, thr = 0;
  logic [39:0] t_now = 0;
  logic [2:0] rd_addr = 0;
  logic [103:0] rd_data;
  logic [31:0] count;
  slow_cmd_log #(.DEPTH(8)) dut (.*);

  always @(posedge clk) t_now <= t_now + 1;
  task automatic cpl(input [31:0] l);
    @(negedge clk); evt = 1; lat = l; idx = idx + 1; @(negedge clk); evt = 0;
  endtask

  int exp_n = 0;
  logic [31:0] exp_lat [8];
  logic [31:0] exp_idx [8];
  initial begin
    repeat (3) @(negedge clk); rst = 0;
    cpl(1000);                                        // thr = 0: logging off
    if (count != 0) $fatal(1, "count %0d with thr=0", count);
    thr = 500;
    for (int i = 0; i < 30; i++) begin
      automatic logic [31:0] l = (i % 3 == 0) ? 500 + i : 100 + i;   // every third is slow (>= thr)
      cpl(l);
      if (l >= thr) begin
        if (exp_n < 8) begin exp_lat[exp_n] = l; exp_idx[exp_n] = idx; end
        exp_n++;
      end
    end
    @(negedge clk);
    if (count != exp_n) $fatal(1, "count %0d, expected %0d", count, exp_n);
    for (int a = 0; a < 8 && a < exp_n; a++) begin
      rd_addr = a; @(negedge clk); @(negedge clk);
      if (rd_data[31:0] != exp_lat[a] || rd_data[63:32] != exp_idx[a])
        $fatal(1, "entry %0d: lat %0d idx %0d, expected %0d %0d", a, rd_data[31:0], rd_data[63:32], exp_lat[a], exp_idx[a]);
      if (a > 0 && rd_data[103:64] == 0) $fatal(1, "entry %0d: no timestamp", a);
    end
    // full log: further hits only counted, entry 0 unchanged
    cpl(9999); @(negedge clk);
    if (count != exp_n + 1) $fatal(1, "count after full %0d", count);
    rd_addr = 0; @(negedge clk); @(negedge clk);
    if (rd_data[31:0] != exp_lat[0]) $fatal(1, "entry 0 overwritten");
    rst = 1; @(negedge clk); rst = 0; @(negedge clk);
    if (count != 0) $fatal(1, "reset did not clear count");
    $display("TEST PASSED");
    $finish;
  end
endmodule
