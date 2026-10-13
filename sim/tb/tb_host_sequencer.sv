// =============================================================================
// tb_host_sequencer.sv - self-checking testbench for rtl/nvme/host_sequencer.sv
// Scenarios: full flow with stop / drain / flush / shutdown, enum failure, admin-init
// failure, link loss while running, stop before the NVMe init, restart after DONE.
// =============================================================================
`timescale 1ns/1ps
module tb_host_sequencer;
  logic clk = 1'b0, rst = 1'b1;
  always #4 clk = ~clk;

  logic start = 0, stop = 0, link_ready = 0;
  logic enum_done = 0, enum_fail = 0, rpcfg_done = 0, init_done = 0, init_fail = 0;
  logic flush_done = 0, shutdown_done = 0, all_slots_free = 0;
  logic [7:0] enum_fail_step = 8'h00, init_fail_step = 8'h00;
  logic enum_start, rpcfg_start, init_start, we_enable, flush_req, shutdown_start;
  logic [3:0] state;
  logic [7:0] error_code, error_detail;
  logic running, done;

  host_sequencer #(.DRAIN_STABLE(8)) dut (.*);

  localparam logic [3:0] ST_IDLE = 0, ST_LINK = 1, ST_ENUM = 2, ST_RPCFG = 3, ST_ADMIN_INIT = 4,
                         ST_RUN = 5, ST_DRAIN = 6, ST_FLUSH = 7, ST_SHUTDOWN = 8, ST_DONE = 9, ST_ERROR = 15;

  int errors = 0;
  int n_enum_start, n_rpcfg_start, n_init_start, n_shutdown_start;
  task automatic check(input bit c, input string m);
    if (!c) begin errors++; $display("[%0t] ERROR: %s", $time, m); end
  endtask

  // count start pulses and verify they are single-cycle
  always_ff @(posedge clk) begin
    if (rst) begin n_enum_start <= 0; n_rpcfg_start <= 0; n_init_start <= 0; n_shutdown_start <= 0; end
    else begin
      if (enum_start)     n_enum_start     <= n_enum_start + 1;
      if (rpcfg_start)    n_rpcfg_start    <= n_rpcfg_start + 1;
      if (init_start)     n_init_start     <= n_init_start + 1;
      if (shutdown_start) n_shutdown_start <= n_shutdown_start + 1;
    end
  end

  task automatic wait_state(input logic [3:0] s, input int max_clk);
    int n = 0;
    while (state !== s && n < max_clk) begin @(posedge clk); n++; end
    check(state === s, $sformatf("timeout waiting for state %0d (state=%0d)", s, state));
  endtask
  task automatic cycles(input int n); repeat (n) @(posedge clk); endtask

  task automatic do_reset();
    rst = 1; start = 0; stop = 0; link_ready = 0;
    enum_done = 0; enum_fail = 0; rpcfg_done = 0; init_done = 0; init_fail = 0;
    flush_done = 0; shutdown_done = 0; all_slots_free = 0;
    cycles(4); rst = 0; cycles(2);
  endtask

  task automatic bring_up_to_run();
    start = 1;
    cycles(5);
    check(state == ST_LINK, "waits in LINK until link_ready");
    link_ready = 1;
    wait_state(ST_ENUM, 10);
    cycles(3); enum_done = 1; cycles(1); enum_done = 0;
    wait_state(ST_RPCFG, 10);
    cycles(3); rpcfg_done = 1; cycles(1); rpcfg_done = 0;
    wait_state(ST_ADMIN_INIT, 10);
    cycles(3); init_done = 1;
    wait_state(ST_RUN, 10);
    init_done = 0;
  endtask

  initial begin
    // ---------------- S1: complete run ----------------
    $display("S1: full flow");
    do_reset();
    bring_up_to_run();
    check(n_enum_start == 1 && n_rpcfg_start == 1 && n_init_start == 1, "each start pulse exactly once");
    check(we_enable && running && !flush_req && !done, "RUN outputs");
    cycles(20);
    stop = 1;
    wait_state(ST_DRAIN, 5);
    check(we_enable && !running, "DRAIN keeps we_enable");
    cycles(20);
    check(state == ST_DRAIN, "DRAIN waits for all_slots_free");
    all_slots_free = 1; cycles(3); all_slots_free = 0; cycles(3);       // glitch shorter than DRAIN_STABLE
    check(state == ST_DRAIN, "DRAIN must debounce all_slots_free");
    all_slots_free = 1;
    wait_state(ST_FLUSH, 30);
    check(flush_req && !we_enable, "FLUSH: flush_req=1, we_enable=0");
    cycles(10);
    check(state == ST_FLUSH, "FLUSH waits for flush_done");
    flush_done = 1;
    wait_state(ST_SHUTDOWN, 5);
    cycles(2);
    check(n_shutdown_start == 1, "shutdown_start pulse");
    cycles(10);
    check(state == ST_SHUTDOWN, "SHUTDOWN waits for shutdown_done");
    shutdown_done = 1;
    wait_state(ST_DONE, 5);
    check(done && !running && !we_enable && !flush_req, "DONE outputs");
    check(error_code == 8'h00, "no error");
    cycles(10);
    check(state == ST_DONE, "DONE is sticky while start=1");
    start = 0;
    wait_state(ST_IDLE, 5);

    // ---------------- S2: enumeration failure ----------------
    $display("S2: enum failure");
    do_reset();
    start = 1; link_ready = 1;
    wait_state(ST_ENUM, 20);
    enum_fail_step = 8'h07; cycles(2); enum_fail = 1;
    wait_state(ST_ERROR, 5);
    check(error_code == 8'h02 && error_detail == 8'h07, $sformatf("enum error code/detail %h/%h", error_code, error_detail));
    check(!we_enable && !running && !done, "ERROR outputs");
    cycles(10);
    check(state == ST_ERROR, "ERROR sticky");
    start = 0; enum_fail = 0;
    wait_state(ST_IDLE, 5);
    check(error_code == 8'h02, "error_code is kept until the next start");   // cleared in IDLE when start rises
    cycles(2);

    // ---------------- S3: admin init failure ----------------
    $display("S3: admin init failure");
    do_reset();
    start = 1; link_ready = 1;
    wait_state(ST_ENUM, 20);
    enum_done = 1; cycles(1); enum_done = 0;
    wait_state(ST_RPCFG, 5);
    rpcfg_done = 1; cycles(1); rpcfg_done = 0;
    wait_state(ST_ADMIN_INIT, 5);
    init_fail_step = 8'h22; cycles(2); init_fail = 1;
    wait_state(ST_ERROR, 5);
    check(error_code == 8'h03 && error_detail == 8'h22, $sformatf("init error code/detail %h/%h", error_code, error_detail));

    // ---------------- S4: link lost while running ----------------
    $display("S4: link loss in RUN");
    do_reset();
    bring_up_to_run();
    cycles(10);
    link_ready = 0;
    wait_state(ST_ERROR, 5);
    check(error_code == 8'h01 && error_detail == 8'h05, $sformatf("link-loss code/detail %h/%h", error_code, error_detail));
    check(!we_enable, "we_enable low in ERROR");

    // ---------------- S5: stop before NVMe init ----------------
    $display("S5: stop requested during bring-up");
    do_reset();
    start = 1; stop = 1; link_ready = 1;
    wait_state(ST_ENUM, 20);
    enum_done = 1; cycles(1); enum_done = 0;
    wait_state(ST_RPCFG, 5);
    rpcfg_done = 1; cycles(1); rpcfg_done = 0;
    wait_state(ST_DONE, 5);
    check(n_init_start == 0, "no NVMe init when stop was requested");

    // ---------------- S6: restart after DONE ----------------
    $display("S6: restart");
    start = 0; stop = 0;
    wait_state(ST_IDLE, 5);
    cycles(2);
    start = 1;
    wait_state(ST_LINK, 5);
    wait_state(ST_ENUM, 10);
    cycles(2);
    check(n_enum_start == 2, "second run starts enumeration again");

    cycles(10);
    if (errors == 0) $display("TEST PASSED");
    else $fatal(1, "TEST FAILED: %0d errors", errors);
    $finish;
  end

  initial begin
    #5ms;
    $fatal(1, "TEST FAILED: watchdog");
  end
endmodule
