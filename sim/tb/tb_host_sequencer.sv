// =============================================================================
// tb_host_sequencer.sv - self-checking testbench for rtl/nvme/host_sequencer.sv
// Scenarios: full flow with stop / drain / flush / shutdown, enum failure, admin-init
// failure, link loss while running, stop before the NVMe init, restart after DONE.
//   S7  second run without reset with module mocks that keep their done/fail levels of the
//       previous run until the cycle AFTER the start pulse (the worst case the sequencer must
//       survive): every stage must really wait for its own completion (dwell >= mock latency);
//       a retry after init_fail must not fall straight back into ERROR.
//   S8  sticky link_lost (survives the user reset): ERROR/E_LINK_LOST, then after a reset the
//       sequencer stays in IDLE (start still high) reporting E_LINK_LOST until link_lost clears.
//   S9  link_fail (link-up timeout) in LINK -> E_LINK_TIMEOUT.
//   S10 DRAIN terminates on io_timeout / lba_range_err (also from RUN) with the right error code
//       and still runs FLUSH -> SHUTDOWN -> DONE.
// =============================================================================
`timescale 1ns/1ps
module tb_host_sequencer;
  logic clk = 1'b0, rst = 1'b1;
  always #4 clk = ~clk;

  logic start = 0, stop = 0, link_ready = 0;
  logic enum_done = 0, enum_fail = 0, rpcfg_done = 0, init_done = 0, init_fail = 0;
  logic flush_done = 0, shutdown_done = 0, all_slots_free = 0;
  logic link_lost = 0, link_fail = 0, io_timeout = 0, lba_range_err = 0;
  logic [7:0] enum_fail_step = 8'h00, init_fail_step = 8'h00;
  logic enum_start, rpcfg_start, init_start, we_enable, flush_req, shutdown_start;
  logic [3:0] state;
  logic [7:0] error_code, error_detail;
  logic running, done;
  bit   mock_en = 0;

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
    link_lost = 0; link_fail = 0; io_timeout = 0; lba_range_err = 0; mock_en = 0;
    cycles(4); rst = 0; cycles(2);
  endtask

  // ---- module mocks (S7): completion levels persist until the cycle AFTER the start pulse ----
  int mock_lat = 6;
  bit mock_init_fail_next = 0;
  int cd_enum = 0, cd_rpcfg = 0, cd_init = 0, cd_flush = 0, cd_shn = 0;
  always @(posedge clk) if (mock_en) begin
    if (enum_start)  begin enum_done <= 0; enum_fail <= 0; cd_enum <= mock_lat; end
    else if (cd_enum != 0) begin cd_enum <= cd_enum - 1; if (cd_enum == 1) enum_done <= 1; end
    if (rpcfg_start) begin rpcfg_done <= 0; cd_rpcfg <= mock_lat; end
    else if (cd_rpcfg != 0) begin cd_rpcfg <= cd_rpcfg - 1; if (cd_rpcfg == 1) rpcfg_done <= 1; end
    if (init_start)  begin init_done <= 0; init_fail <= 0; cd_init <= mock_lat; end
    else if (cd_init != 0) begin
      cd_init <= cd_init - 1;
      if (cd_init == 1) begin
        if (mock_init_fail_next) begin init_fail <= 1; init_fail_step <= 8'h31; mock_init_fail_next <= 0; end
        else init_done <= 1;
      end
    end
    if (shutdown_start) begin shutdown_done <= 0; cd_shn <= mock_lat; end
    else if (cd_shn != 0) begin cd_shn <= cd_shn - 1; if (cd_shn == 1) shutdown_done <= 1; end
    // flush: level follows flush_req (like the write engine)
    if (!flush_req) begin flush_done <= 0; cd_flush <= mock_lat; end
    else if (cd_flush != 0) begin cd_flush <= cd_flush - 1; if (cd_flush == 1) flush_done <= 1; end
  end

  // dwell time (cycles) of every state, recorded when the state is left
  int dwell [16];
  int dwell_cnt = 0;
  logic [3:0] st_d = 0;
  always @(posedge clk) begin
    if (state === st_d) dwell_cnt <= dwell_cnt + 1;
    else begin dwell[st_d] = dwell_cnt + 1; dwell_cnt <= 0; end
    st_d <= state;
  end

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
    cycles(2); enum_done = 1; cycles(1); enum_done = 0;
    wait_state(ST_RPCFG, 5);
    cycles(2); rpcfg_done = 1; cycles(1); rpcfg_done = 0;
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
    cycles(2); enum_done = 1; cycles(1); enum_done = 0;
    wait_state(ST_RPCFG, 5);
    cycles(2); rpcfg_done = 1; cycles(1); rpcfg_done = 0;
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

    // ---------------- S7: second run without reset (stale completion levels) ----------------
    $display("S7: second run without reset");
    do_reset();
    mock_en = 1; mock_lat = 6;
    start = 1; link_ready = 1;
    wait_state(ST_RUN, 200); cycles(2);
    check(dwell[ST_ENUM] >= mock_lat && dwell[ST_RPCFG] >= mock_lat && dwell[ST_ADMIN_INIT] >= mock_lat,
          $sformatf("run 1: stages did not wait for their modules (dwell enum %0d rpcfg %0d init %0d)",
                    dwell[ST_ENUM], dwell[ST_RPCFG], dwell[ST_ADMIN_INIT]));
    cycles(10); stop = 1; wait_state(ST_DRAIN, 5);
    all_slots_free = 1;
    wait_state(ST_DONE, 200);
    check(error_code == 8'h00, "run 1 error code");
    check(enum_done && rpcfg_done && init_done && shutdown_done, "mock: stale levels of run 1 are still high");
    start = 0; stop = 0; all_slots_free = 0;
    wait_state(ST_IDLE, 5);
    cycles(3);
    begin
      int e0, r0, i0, s0;
      e0 = n_enum_start; r0 = n_rpcfg_start; i0 = n_init_start; s0 = n_shutdown_start;
      start = 1;                                      // run 2 - NO reset, all levels stale
      wait_state(ST_ENUM, 20);
      wait_state(ST_RPCFG, 60); cycles(2);
      check(dwell[ST_ENUM] >= mock_lat, $sformatf("run 2: ENUM left after %0d cycles (stale enum_done), mock latency %0d", dwell[ST_ENUM], mock_lat));
      wait_state(ST_ADMIN_INIT, 60); cycles(2);
      check(dwell[ST_RPCFG] >= mock_lat, $sformatf("run 2: RPCFG left after %0d cycles (stale rpcfg_done)", dwell[ST_RPCFG]));
      wait_state(ST_RUN, 60); cycles(2);
      check(dwell[ST_ADMIN_INIT] >= mock_lat, $sformatf("run 2: ADMIN_INIT left after %0d cycles (stale init_done)", dwell[ST_ADMIN_INIT]));
      cycles(10); stop = 1; wait_state(ST_DRAIN, 5);
      all_slots_free = 1;
      wait_state(ST_SHUTDOWN, 200);
      wait_state(ST_DONE, 200); cycles(2);
      check(dwell[ST_SHUTDOWN] >= mock_lat, $sformatf("run 2: SHUTDOWN left after %0d cycles (stale shutdown_done)", dwell[ST_SHUTDOWN]));
      check(n_enum_start == e0 + 1 && n_rpcfg_start == r0 + 1 && n_init_start == i0 + 1 && n_shutdown_start == s0 + 1,
            "run 2: each start pulse exactly once");
      check(error_code == 8'h00, "run 2 error code");
    end
    // retry after an init failure without reset
    start = 0; stop = 0; all_slots_free = 0;
    wait_state(ST_IDLE, 5); cycles(2);
    mock_init_fail_next = 1;
    start = 1;
    wait_state(ST_ERROR, 100);
    check(error_code == 8'h03 && error_detail == 8'h31, $sformatf("run 3: admin failure code/detail %h/%h", error_code, error_detail));
    check(init_fail, "mock: init_fail level is high (stale for the retry)");
    start = 0; wait_state(ST_IDLE, 5); cycles(2);
    start = 1;                                        // retry: init_fail of run 3 is stale at ADMIN_INIT entry
    wait_state(ST_RUN, 200); cycles(2);
    check(dwell[ST_ADMIN_INIT] >= mock_lat, $sformatf("retry: ADMIN_INIT left after %0d cycles", dwell[ST_ADMIN_INIT]));
    check(error_code == 8'h00, "retry: error code cleared");
    start = 0; mock_en = 0;

    // ---------------- S8: sticky link_lost survives the reset ----------------
    $display("S8: sticky link_lost");
    do_reset();
    bring_up_to_run();
    cycles(5);
    link_ready = 0; link_lost = 1;                    // link_bringup: drop after READY
    wait_state(ST_ERROR, 5);
    check(error_code == 8'h01 && error_detail == 8'h05, $sformatf("S8: code/detail %h/%h", error_code, error_detail));
    // the IP now asserts user_reset: the sequencer is wiped, start is still high, link_lost stays
    rst = 1; cycles(4); rst = 0; cycles(2);
    begin
      int e0;
      e0 = n_enum_start;
      link_ready = 1;                                 // link retrained
      cycles(60);
      check(state == ST_IDLE, $sformatf("S8: sequencer left IDLE while link_lost is set (state %0d)", state));
      check(error_code == 8'h01, $sformatf("S8: E_LINK_LOST not reported after reset (%h)", error_code));
      check(n_enum_start == e0 && !we_enable, "S8: restarted despite link_lost");
      link_lost = 0;                                  // board reset (modelled)
      cycles(3);
      check(state == ST_LINK || state == ST_ENUM, "S8: does not start after link_lost cleared");
    end
    // cold: start refused while link_lost; link_lost in ENUM aborts
    do_reset();
    link_lost = 1; start = 1; link_ready = 1;
    cycles(20);
    check(state == ST_IDLE && error_code == 8'h01, "S8: start refused while link_lost (cold)");
    link_lost = 0;
    wait_state(ST_ENUM, 20);
    link_lost = 1;
    wait_state(ST_ERROR, 5);
    check(error_code == 8'h01 && error_detail == 8'h02, $sformatf("S8: link_lost in ENUM: %h/%h", error_code, error_detail));
    link_lost = 0; start = 0;

    // ---------------- S9: link-up timeout ----------------
    $display("S9: link_fail");
    do_reset();
    start = 1; link_ready = 0;
    wait_state(ST_LINK, 10);
    cycles(5); check(state == ST_LINK, "S9: waits in LINK");
    link_fail = 1;
    wait_state(ST_ERROR, 5);
    check(error_code == 8'h06, $sformatf("S9: E_LINK_TIMEOUT expected, got %h", error_code));
    link_fail = 0; start = 0;

    // ---------------- S10: DRAIN terminates ----------------
    $display("S10: io_timeout / lba_range_err");
    for (int k = 0; k < 4; k++) begin
      // k=0 io_timeout in DRAIN, k=1 lba_range_err in DRAIN, k=2 io_timeout in RUN, k=3 lba_range_err in RUN
      do_reset();
      bring_up_to_run();
      cycles(10);
      if (k >= 2) begin
        if (k == 2) io_timeout = 1; else lba_range_err = 1;
        wait_state(ST_FLUSH, 5);
      end else begin
        stop = 1; wait_state(ST_DRAIN, 5);
        cycles(30);
        check(state == ST_DRAIN, "S10: DRAIN waits while slots are busy");
        if (k == 0) io_timeout = 1; else lba_range_err = 1;
        wait_state(ST_FLUSH, 5);
      end
      check(error_code == ((k % 2 == 0) ? 8'h04 : 8'h05), $sformatf("S10[%0d]: error code %h", k, error_code));
      check(flush_req && !we_enable, $sformatf("S10[%0d]: FLUSH outputs", k));
      cycles(5); flush_done = 1;
      wait_state(ST_SHUTDOWN, 5);
      cycles(5); shutdown_done = 1;
      wait_state(ST_DONE, 5);
      check(done && error_code == ((k % 2 == 0) ? 8'h04 : 8'h05), $sformatf("S10[%0d]: DONE with the error code kept", k));
    end

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
