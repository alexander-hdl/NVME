// =============================================================================
// tb_ddr_memtest.sv - ddr_memtest alone on mig_app_model (spec 9.1 tb_ddr_memtest).
//   pass: 1 MiB and 2 MiB with random MIG ready / refresh -> 0 errors, whole area written and read
//   fail: model flips a bit on a read of word address A -> err_cnt = 2 (one per pass), first = A exactly;
//   start is ignored while busy; a new start clears done and the counters.
// =============================================================================
`timescale 1ns/1ps
module tb_ddr_memtest;
  logic clk, rst, init_calib_complete, start, abort = 0, busy, done;
  logic [10:0] mib;
  logic [31:0] err_cnt; logic [28:0] first_err_addr;
  logic [28:0] app_addr; logic [2:0] app_cmd; logic app_en, app_rdy;
  logic [255:0] app_wdf_data; logic app_wdf_wren, app_wdf_end, app_wdf_rdy;
  logic [255:0] app_rd_data; logic app_rd_data_valid, app_rd_data_end;
  logic app_sr_active, app_ref_ack, app_zq_ack;

  mig_app_model #(.CALIB_DELAY(50), .RDY_DROP_PCT(25)) u_mig (
    .ui_clk(clk), .ui_clk_sync_rst(rst), .init_calib_complete(init_calib_complete), .device_temp_i(12'd0),
    .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en), .app_rdy(app_rdy),
    .app_wdf_data(app_wdf_data), .app_wdf_mask(32'h0), .app_wdf_wren(app_wdf_wren),
    .app_wdf_end(app_wdf_end), .app_wdf_rdy(app_wdf_rdy),
    .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid), .app_rd_data_end(app_rd_data_end),
    .app_sr_req(1'b0), .app_ref_req(1'b0), .app_zq_req(1'b0),
    .app_sr_active(app_sr_active), .app_ref_ack(app_ref_ack), .app_zq_ack(app_zq_ack));

  ddr_memtest dut (.clk(clk), .rst(rst), .start(start), .abort(abort), .mib(mib), .init_calib_complete(init_calib_complete),
    .busy(busy), .done(done), .err_cnt(err_cnt), .first_err_addr(first_err_addr),
    .app_addr(app_addr), .app_cmd(app_cmd), .app_en(app_en), .app_rdy(app_rdy),
    .app_wdf_data(app_wdf_data), .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end),
    .app_wdf_rdy(app_wdf_rdy), .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid));

  longint unsigned cyc = 0;
  always @(posedge clk) cyc <= cyc + 1;

  task automatic chk(input bit c, input string msg);
    if (!c) $fatal(1, "CHECK FAILED @cyc=%0d: %s", cyc, msg);
  endtask

  task automatic run_test(input int m);
    int t = 0;
    mib = 11'(m);
    @(posedge clk); start <= 1; @(posedge clk); start <= 0;
    @(posedge clk);
    chk(busy == 1 && done == 0, "busy after start");
    @(posedge clk); start <= 1; @(posedge clk); start <= 0;        // ignored while busy
    while (!done) begin @(posedge clk); t++; if (t > 3_000_000) $fatal(1, "timeout"); end
    chk(busy == 0, "busy low when done");
  endtask

  initial begin
    start = 0; mib = 1;
    wait (init_calib_complete);
    repeat (5) @(posedge clk);
    // pass, 1 MiB
    run_test(1);
    chk(err_cnt == 0, "1 MiB: no errors");
    chk(u_mig.wr_cmds == 2 * 32768 && u_mig.rd_cmds == 2 * 32768, "1 MiB: 2 passes x 32768 commands written and read");
    chk(u_mig.unwritten_reads == 0, "reads only of written words");
    chk(u_mig.max_addr == 29'(8 * 32767), "highest address of 1 MiB");
    // pass, 2 MiB
    run_test(2);
    chk(err_cnt == 0, "2 MiB: no errors");
    chk(u_mig.max_addr == 29'(8 * 65535), "highest address of 2 MiB");
    // fault in pass 0 and 1 (model corrupts every read of that word)
    u_mig.inject_err_addr = 8 * 777; u_mig.injected = 0;
    run_test(1);
    chk(err_cnt == 2, "fault: once per pass");
    chk(first_err_addr == 29'(8 * 777), "fault: exact first error address");
    chk(u_mig.injected == 2, "fault: model injected twice");
    // a later, second fault word is also counted; first stays
    u_mig.inject_err_addr = 8 * 32000;
    run_test(1);
    chk(err_cnt == 2 && first_err_addr == 29'(8 * 32000), "second run: counters restart");
    u_mig.inject_err_addr = -1;
    run_test(1);
    chk(err_cnt == 0, "clean again after the fault is removed");
    // abort in the write phase and in the read phase: busy drops after the reads in flight are drained,
    // done stays 0, then a complete test passes again
    for (int k = 0; k < 2; k++) begin
      int t = 0;
      longint unsigned rd0;
      rd0 = u_mig.rd_cmds;
      @(posedge clk); start <= 1; @(posedge clk); start <= 0;
      if (k == 0) repeat (300) @(posedge clk);
      else begin while (u_mig.rd_cmds - rd0 < 50) @(posedge clk); end
      chk(busy == 1, "abort: running");
      @(posedge clk); abort <= 1; @(posedge clk); abort <= 0;
      while (busy) begin @(posedge clk); t++; if (t > 200) $fatal(1, "abort: busy stuck"); end
      repeat (60) @(posedge clk);
      chk(done == 0 && u_mig.rq.size() == 0, "abort: done stays 0, no read left");
    end
    run_test(1);
    chk(err_cnt == 0 && u_mig.unwritten_reads == 0, "after abort: clean pass");
    chk(u_mig.proto_errors == 0, "MIG protocol");
    $display("TEST PASSED");
    $finish;
  end
  initial begin #(300ms); $fatal(1, "watchdog"); end
endmodule
