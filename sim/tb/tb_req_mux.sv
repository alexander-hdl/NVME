// =============================================================================
// tb_req_mux.sv - unit test of req_mux with a stub requester
// Stub requester: accepts one command when idle, answers 1..4 cycles later with
//   rsp_status = {1'b0, op}, rsp_rdata = req_addr ^ 32'hA5A5_A5A5.
// Checks
//   * both ports permanently busy -> grants strictly alternate (p1 waits for at
//     most one p0 command, p0 never starves);
//   * single-port operation, random mixed traffic;
//   * every response is routed to the owner only, with the owner's data.
// =============================================================================
`timescale 1ns/1ps
module tb_req_mux;

  logic clk = 1'b0;
  logic rst = 1'b1;
  always #4 clk = ~clk;

  int errors = 0;
  task automatic err(input string msg);
    errors++;
    $error("%s", msg);
  endtask

  // port 0 / port 1 command signals (driven here)
  logic        p_req_valid [2];
  logic        p_req_ready [2];
  logic [1:0]  p_req_op    [2];
  logic [31:0] p_req_addr  [2];
  logic [31:0] p_req_wdata [2];
  logic [3:0]  p_req_be    [2];
  logic        p_rsp_valid [2];
  logic [2:0]  p_rsp_status[2];
  logic [31:0] p_rsp_rdata [2];

  // requester side
  logic        r_req_valid, r_req_ready, r_rsp_valid;
  logic [1:0]  r_req_op;
  logic [31:0] r_req_addr, r_req_wdata, r_rsp_rdata;
  logic [3:0]  r_req_be;
  logic [2:0]  r_rsp_status;

  req_mux dut (
    .clk(clk), .rst(rst),
    .p0_req_valid(p_req_valid[0]), .p0_req_ready(p_req_ready[0]), .p0_req_op(p_req_op[0]),
    .p0_req_addr(p_req_addr[0]), .p0_req_wdata(p_req_wdata[0]), .p0_req_be(p_req_be[0]),
    .p0_rsp_valid(p_rsp_valid[0]), .p0_rsp_status(p_rsp_status[0]), .p0_rsp_rdata(p_rsp_rdata[0]),
    .p1_req_valid(p_req_valid[1]), .p1_req_ready(p_req_ready[1]), .p1_req_op(p_req_op[1]),
    .p1_req_addr(p_req_addr[1]), .p1_req_wdata(p_req_wdata[1]), .p1_req_be(p_req_be[1]),
    .p1_rsp_valid(p_rsp_valid[1]), .p1_rsp_status(p_rsp_status[1]), .p1_rsp_rdata(p_rsp_rdata[1]),
    .r_req_valid(r_req_valid), .r_req_ready(r_req_ready), .r_req_op(r_req_op),
    .r_req_addr(r_req_addr), .r_req_wdata(r_req_wdata), .r_req_be(r_req_be),
    .r_rsp_valid(r_rsp_valid), .r_rsp_status(r_rsp_status), .r_rsp_rdata(r_rsp_rdata)
  );

  // ---- stub requester ----
  logic        st_busy;
  int          st_cnt;
  logic [1:0]  st_op;
  logic [31:0] st_addr;
  assign r_req_ready  = !st_busy && !r_rsp_valid;
  assign r_rsp_status = {1'b0, st_op};
  assign r_rsp_rdata  = st_addr ^ 32'hA5A5_A5A5;

  always @(posedge clk) begin
    if (rst) begin
      st_busy     <= 1'b0;
      r_rsp_valid <= 1'b0;
      st_cnt      <= 0;
    end else begin
      r_rsp_valid <= 1'b0;
      if (r_req_valid && r_req_ready) begin
        st_busy <= 1'b1;
        st_op   <= r_req_op;
        st_addr <= r_req_addr;
        st_cnt  <= $urandom_range(0, 3);
      end else if (st_busy) begin
        if (st_cnt == 0) begin
          st_busy     <= 1'b0;
          r_rsp_valid <= 1'b1;
        end else st_cnt <= st_cnt - 1;
      end
    end
  end

  // ---- port drivers ----
  int  rate [2];                  // percent chance to present a command when idle
  int  n_acc [2];
  int  n_rsp [2];
  int          exp_port [$];      // commands accepted by the mux, waiting for their response
  logic [31:0] exp_addr [$];
  logic [1:0]  exp_op   [$];
  int  order [$];                 // port of each accepted command
  logic [31:0] seq [2];
  int  p0_run_while_p1;           // p0 accepts while p1 valid and waiting

  always @(posedge clk) begin
    int ep; logic [31:0] ea; logic [1:0] eo;
    if (rst) begin
      for (int i = 0; i < 2; i++) begin
        p_req_valid[i] <= 1'b0;
        n_acc[i] = 0; n_rsp[i] = 0;
        seq[i] = 32'h1000_0000 * (i + 1);
      end
      p0_run_while_p1 = 0;
    end else begin
      for (int i = 0; i < 2; i++) begin
        // handshake
        if (p_req_valid[i] && p_req_ready[i]) begin
          p_req_valid[i] <= 1'b0;
          exp_port.push_back(i);
          exp_addr.push_back(p_req_addr[i]);
          exp_op.push_back(p_req_op[i]);
          n_acc[i]++;
          order.push_back(i);
          if (i == 0 && p_req_valid[1]) begin
            p0_run_while_p1++;
            if (p0_run_while_p1 > 1) err("p1 waited for more than one p0 command");
          end
          if (i == 1) p0_run_while_p1 = 0;
        end
        // response
        if (p_rsp_valid[i]) begin
          if (exp_port.size() == 0) err($sformatf("port %0d: response without command", i));
          else begin
            ep = exp_port.pop_front();
            ea = exp_addr.pop_front();
            eo = exp_op.pop_front();
            if (ep != i) err($sformatf("response for port %0d delivered to port %0d", ep, i));
            if (p_rsp_rdata[i] !== (ea ^ 32'hA5A5_A5A5)) err($sformatf("port %0d: wrong rdata", i));
            if (p_rsp_status[i] !== {1'b0, eo}) err($sformatf("port %0d: wrong status", i));
            n_rsp[i]++;
          end
        end
        // new command whenever the port is not already presenting one (so a command can be
        // pending while the previous one of the same port is still in flight)
        if (!p_req_valid[i] && ($urandom_range(0, 99) < rate[i])) begin
          p_req_valid[i] <= 1'b1;
          p_req_op[i]    <= $urandom_range(0, 3);
          p_req_addr[i]  <= seq[i];
          p_req_wdata[i] <= 32'hDA7A_0000 + seq[i][15:0];
          p_req_be[i]    <= 4'hF;
          seq[i] = seq[i] + 4;
        end
      end
      if (p_rsp_valid[0] && p_rsp_valid[1]) err("both response valids asserted");
    end
  end

  // ---- phases ----
  task automatic phase(input string name, input int r0, input int r1, input int cycles);
    int a0, a1, n0, n1;
    // let outstanding commands drain
    rate[0] = 0; rate[1] = 0;
    repeat (20) @(posedge clk);
    order.delete();
    a0 = n_acc[0]; a1 = n_acc[1];
    rate[0] = r0; rate[1] = r1;
    repeat (cycles) @(posedge clk);
    rate[0] = 0; rate[1] = 0;
    repeat (20) @(posedge clk);
    n0 = n_acc[0] - a0; n1 = n_acc[1] - a1;
    $display("[%0t] %-24s p0 %0d cmds, p1 %0d cmds", $time, name, n0, n1);
    if (n_acc[0] != n_rsp[0] || n_acc[1] != n_rsp[1]) err({name, ": command/response count mismatch"});
    if (r0 > 0 && n0 == 0) err({name, ": p0 starved"});
    if (r1 > 0 && n1 == 0) err({name, ": p1 starved"});
    if (r0 == 0 && n0 != 0) err({name, ": p0 served although idle"});
    if (r1 == 0 && n1 != 0) err({name, ": p1 served although idle"});
  endtask

  initial begin
    rate[0] = 0; rate[1] = 0;
    repeat (4) @(posedge clk);
    rst <= 1'b0;
    repeat (4) @(posedge clk);

    // both permanently busy: strict alternation
    phase("both saturated", 100, 100, 600);
    begin
      int bad;
      bad = 0;
      for (int k = 1; k < order.size(); k++) if (order[k] == order[k-1]) bad++;
      if (bad != 0) err($sformatf("saturated: %0d non-alternating grants", bad));
      if (order.size() < 50) err("saturated: too few commands");
    end
    phase("only p0",        100, 0,   300);
    phase("only p1",        0,   100, 300);
    phase("random mix",     30,  30,  3000);
    phase("p0 heavy, p1 rare", 100, 10, 3000);

    if (errors != 0) $fatal(1, "TEST FAILED: %0d error(s)", errors);
    $display("TEST PASSED");
    $finish;
  end

  initial begin
    #5_000_000;
    $fatal(1, "TEST FAILED: global watchdog");
  end
endmodule
