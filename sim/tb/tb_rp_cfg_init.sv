// =============================================================================
// tb_rp_cfg_init.sv - testbench for rp_cfg_init
// A behavioural cfg_mgmt register file (random latency, one-cycle rd_wr_done,
// garbage on cfg_mgmt_do outside the done cycle) checks:
//   * the three writes + one read, their order, byte enables and data;
//   * enables are held (address/data/BE stable) until rd_wr_done;
//   * at least one idle cycle between operations;
//   * the read-modify-write value of Device Control, upper bits and the
//     bits we do not own (MRRS) preserved;
//   * start is edge sensitive (a level held high does not re-run).
// =============================================================================
`timescale 1ns/1ps
module tb_rp_cfg_init;
  import nvme_host_pkg::*;

  logic clk = 1'b0;
  logic rst = 1'b1;
  always #4 clk = ~clk;

  logic        start, done;
  logic [2:0]  mps_code;
  logic        ext_tag_en;
  logic [31:0] cfg_mgmt_di, cfg_mgmt_do;
  logic [3:0]  cfg_mgmt_byte_en;
  logic [9:0]  cfg_mgmt_dwaddr;
  logic        cfg_mgmt_wr_en, cfg_mgmt_rd_en, cfg_mgmt_rd_wr_done;
  logic        cfg_mgmt_wr_readonly, cfg_mgmt_wr_rw1c_as_rw;

  rp_cfg_init dut (.*);

  // ---------------------------------------------------------------------------
  // cfg_mgmt register model
  // ---------------------------------------------------------------------------
  typedef struct {
    bit          wr;
    logic [9:0]  addr;
    logic [31:0] data;
    logic [3:0]  be;
  } op_t;

  op_t         oplog[$];
  logic [31:0] regs [0:1023];

  typedef enum logic [1:0] {M_IDLE, M_WAIT, M_PULSE} mstate_t;
  mstate_t     mstate;
  int          lat;
  bit          lat_wr;
  logic [9:0]  l_addr;
  logic [31:0] l_di;
  logic [3:0]  l_be;
  bit          after_pulse;
  int          errors = 0;

  task automatic err(input string msg);
    errors++;
    $error("%s", msg);
  endtask

  always @(posedge clk) begin
    if (rst) begin
      mstate              <= M_IDLE;
      cfg_mgmt_rd_wr_done <= 1'b0;
      cfg_mgmt_do         <= 32'hDEAD_BEEF;
      after_pulse         <= 1'b0;
    end else begin
      cfg_mgmt_rd_wr_done <= 1'b0;
      cfg_mgmt_do         <= $urandom;                    // garbage unless done
      case (mstate)
        M_IDLE: begin
          if (after_pulse && (cfg_mgmt_wr_en || cfg_mgmt_rd_en))
            err("no idle cycle between two cfg_mgmt operations");
          after_pulse <= 1'b0;
          if (cfg_mgmt_wr_en || cfg_mgmt_rd_en) begin
            if (cfg_mgmt_wr_en && cfg_mgmt_rd_en) err("wr_en and rd_en both asserted");
            lat_wr  <= cfg_mgmt_wr_en;
            l_addr  <= cfg_mgmt_dwaddr;
            l_di    <= cfg_mgmt_di;
            l_be    <= cfg_mgmt_byte_en;
            lat     <= $urandom_range(0, 4);
            mstate  <= M_WAIT;
          end
        end
        M_WAIT: begin
          // enables and payload must be held until done
          if (lat_wr ? !cfg_mgmt_wr_en : !cfg_mgmt_rd_en) err("enable dropped before rd_wr_done");
          if (cfg_mgmt_dwaddr !== l_addr) err("dwaddr changed while waiting");
          if (lat_wr && (cfg_mgmt_di !== l_di)) err("di changed while waiting");
          if (cfg_mgmt_byte_en !== l_be) err("byte_en changed while waiting");
          if (lat == 0) begin
            oplog.push_back('{wr: lat_wr, addr: l_addr, data: l_di, be: l_be});
            if (lat_wr) begin
              for (int b = 0; b < 4; b++)
                if (l_be[b]) regs[l_addr][8*b +: 8] <= l_di[8*b +: 8];
            end else begin
              cfg_mgmt_do <= regs[l_addr];
            end
            cfg_mgmt_rd_wr_done <= 1'b1;
            mstate              <= M_PULSE;
          end else begin
            lat <= lat - 1;
          end
        end
        M_PULSE: begin
          after_pulse <= 1'b1;
          mstate      <= M_IDLE;
        end
        default: mstate <= M_IDLE;
      endcase
    end
  end

  // static outputs
  always @(posedge clk) begin
    if (!rst && (cfg_mgmt_wr_readonly !== 1'b0 || cfg_mgmt_wr_rw1c_as_rw !== 1'b0))
      err("wr_readonly / wr_rw1c_as_rw must be tied low");
  end

  // ---------------------------------------------------------------------------
  // Test sequence
  // ---------------------------------------------------------------------------
  function automatic logic [15:0] exp_devctl(input logic [15:0] cur, input logic [2:0] m, input logic e);
    return (cur & ~16'h01E0) | (16'(m) << 5) | (16'(e) << 8);
  endfunction

  task automatic run_once(input logic [2:0] m, input logic e, input bit hold_start);
    logic [15:0] exp_lo;
    logic [15:0] cur_lo;
    int          n0, cycles, nops;
    op_t         o0, o1, o2, o3;
    n0     = oplog.size();
    cur_lo = regs[RP_DW_DEVCTL][15:0];
    exp_lo = exp_devctl(cur_lo, m, e);

    mps_code   = m;
    ext_tag_en = e;
    @(posedge clk); start <= 1'b1;
    if (!hold_start) begin
      @(posedge clk); start <= 1'b0;
    end
    repeat (3) @(posedge clk);
    if (done !== 1'b0) err("done must drop after start");
    cycles = 0;
    while (!done) begin
      @(posedge clk);
      cycles++;
      if (cycles > 200) $fatal(1, "timeout waiting for done");
    end
    repeat (10) @(posedge clk);                 // a re-trigger would show up here
    start <= 1'b0;
    @(posedge clk);

    nops = oplog.size() - n0;
    if (nops != 4) begin
      err($sformatf("expected 4 operations, got %0d", nops));
    end else begin
      o0 = oplog[n0];
      o1 = oplog[n0 + 1];
      o2 = oplog[n0 + 2];
      o3 = oplog[n0 + 3];
      // 1: Command
      if (o0.wr !== 1 || o0.addr !== RP_DW_COMMAND || o0.data !== 32'h0000_0006 || o0.be !== 4'b0001)
        err("op1: Command write wrong");
      // 2: Bus numbers
      if (o1.wr !== 1 || o1.addr !== RP_DW_BUSNUM || o1.data !== 32'h0001_0100 || o1.be !== 4'b0111)
        err("op2: bus number write wrong");
      // 3: DevCtl read
      if (o2.wr !== 0 || o2.addr !== RP_DW_DEVCTL) err("op3: DevCtl read wrong");
      // 4: DevCtl write
      if (o3.wr !== 1 || o3.addr !== RP_DW_DEVCTL || o3.be !== 4'b0011)
        err("op4: DevCtl write header wrong");
      if (o3.data[15:0] !== exp_lo)
        err($sformatf("op4: DevCtl data 0x%04h, expected 0x%04h", o3.data[15:0], exp_lo));
    end
    if (regs[RP_DW_COMMAND][7:0] !== 8'h06) err("model Command register wrong");
    if (regs[RP_DW_BUSNUM][23:0] !== 24'h01_0100) err("model bus number register wrong");
    if (regs[RP_DW_DEVCTL][15:0] !== exp_lo) err("model DevCtl low half wrong");
    if (regs[RP_DW_DEVCTL][31:16] !== 16'h0010) err("DevCtl upper half (DevStatus) must be preserved");
    $display("[%0t] run mps=%0d ete=%0b: DevCtl 0x%04h -> 0x%04h", $time, m, e, cur_lo, regs[RP_DW_DEVCTL][15:0]);
  endtask

  initial begin
    for (int i = 0; i < 1024; i++) regs[i] = 32'h0;
    regs[RP_DW_DEVCTL] = 32'h0010_2810;
    start      = 1'b0;
    mps_code   = 3'd0;
    ext_tag_en = 1'b0;
    repeat (5) @(posedge clk);
    rst <= 1'b0;
    repeat (3) @(posedge clk);

    if (done !== 1'b0) err("done must be low before the first run");
    run_once(3'd1, 1'b1, 1'b0);                 // pulse start: MPS 256 B, ETE on  -> 0x2930
    if (regs[RP_DW_DEVCTL][15:0] !== 16'h2930) err("run 1: expected DevCtl 0x2930");
    if (done !== 1'b1) err("done must stay high after the run");
    run_once(3'd2, 1'b0, 1'b1);                 // level start held: MPS 512 B, ETE off -> 0x2850
    if (regs[RP_DW_DEVCTL][15:0] !== 16'h2850) err("run 2: expected DevCtl 0x2840");
    if (oplog.size() != 8) err("start held high must not retrigger");

    if (errors != 0) $fatal(1, "TEST FAILED: %0d error(s)", errors);
    $display("TEST PASSED");
    $finish;
  end

  // global watchdog
  initial begin
    #2_000_000;
    $fatal(1, "TEST FAILED: global watchdog");
  end
endmodule
