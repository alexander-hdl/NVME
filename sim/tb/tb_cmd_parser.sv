// =============================================================================
// tb_cmd_parser.sv — uart_rx + rec_cmd_parser (WP1), driven with real 921600-baud bit timing.
// Covers: every command of spec 2.3 (all argument forms), 1..16 hex digits, lower case,
// blanks/tabs, CR / LF / CRLF / LFCR and blank lines, hex-letter words (PROF ADD vs READ ADD),
// every error code the parser can raise (01, 02, 03, 0A, 0C, 0D), a 64-char line (valid) and
// 65 chars (0A), back-to-back lines, a slow consumer (cmd_ready low) and the single ERR 0C.
// Negative controls: a deliberately wrong expectation must be caught by the scoreboard;
// lines with errors must give exactly the expected code (checked for all of them).
// =============================================================================
`timescale 1ns/1ps

module tb_cmd_parser;
  import rec_pkg::*;

  logic clk = 1'b0;
  logic rst = 1'b1;
  always #2.5 clk = ~clk;

  localparam int DIV = 217;

  logic rxd = 1'b1;
  logic [7:0] rx_data;
  logic rx_valid, rx_ferr;
  logic cmd_valid, cmd_ready = 1'b1;
  logic [5:0]  cmd_op;
  logic [3:0]  cmd_sub;
  logic [1:0]  cmd_nargs;
  logic [63:0] cmd_arg0, cmd_arg1, cmd_arg2;
  logic [7:0]  cmd_err;

  uart_rx #(.CLK_HZ(200_000_000), .BAUD(921_600)) u_rx (
    .clk(clk), .rst(rst), .rxd(rxd), .m_data(rx_data), .m_valid(rx_valid), .frame_err(rx_ferr));
  rec_cmd_parser u_dut (
    .clk(clk), .rst(rst), .rx_data(rx_data), .rx_valid(rx_valid), .rx_ferr(rx_ferr),
    .cmd_valid(cmd_valid), .cmd_ready(cmd_ready), .cmd_op(cmd_op), .cmd_sub(cmd_sub),
    .cmd_nargs(cmd_nargs), .cmd_arg0(cmd_arg0), .cmd_arg1(cmd_arg1), .cmd_arg2(cmd_arg2),
    .cmd_err(cmd_err));

  // ------------------------------------------------------------ scoreboard
  typedef struct {
    logic [5:0]  op;
    logic [3:0]  sub;
    logic [1:0]  na;
    logic [63:0] a0, a1, a2;
    logic [7:0]  err;
  } rec_t;
  rec_t got_q [$];
  rec_t exp_q [$];
  int   errors = 0;
  int   neg_detected = 0;
  bit   neg_mode = 1'b0;
  int   ncmd_total = 0;

  // payload must be stable while cmd_valid && !cmd_ready (protocol check)
  rec_t held;
  logic held_v = 1'b0;
  always @(posedge clk) begin
    if (rst) held_v <= 1'b0;
    else begin
      if (cmd_valid && !cmd_ready) begin
        if (held_v && ({cmd_op, cmd_sub, cmd_nargs, cmd_arg0, cmd_arg1, cmd_arg2, cmd_err} !==
                       {held.op, held.sub, held.na, held.a0, held.a1, held.a2, held.err})) begin
          errors++; $display("ERROR: payload changed while cmd_valid && !cmd_ready");
        end
        held <= '{cmd_op, cmd_sub, cmd_nargs, cmd_arg0, cmd_arg1, cmd_arg2, cmd_err};
        held_v <= 1'b1;
      end else held_v <= 1'b0;
      if (cmd_valid && cmd_ready) begin
        got_q.push_back('{cmd_op, cmd_sub, cmd_nargs, cmd_arg0, cmd_arg1, cmd_arg2, cmd_err});
        ncmd_total++;
      end
    end
  end

  task automatic wait_clks(input int n);
    repeat (n) @(posedge clk);
    #1;
  endtask

  task automatic send_byte(input logic [7:0] b, input logic stop_bit = 1'b1);
    rxd = 1'b0;                    wait_clks(DIV);
    for (int i = 0; i < 8; i++) begin rxd = b[i]; wait_clks(DIV); end
    rxd = stop_bit;                wait_clks(DIV);
    rxd = 1'b1;
  endtask

  // eol: 0 LF, 1 CR, 2 CRLF, 3 LFCR, 4 none
  task automatic send_text(input string s, input int eol);
    for (int i = 0; i < s.len(); i++) send_byte(s.getc(i));
    case (eol)
      0: send_byte(8'h0A);
      1: send_byte(8'h0D);
      2: begin send_byte(8'h0D); send_byte(8'h0A); end
      3: begin send_byte(8'h0A); send_byte(8'h0D); end
      default: ;
    endcase
  endtask

  function automatic rec_t mk(input int op, input int sub, input int na,
                              input logic [63:0] a0, input logic [63:0] a1, input logic [63:0] a2,
                              input int err);
    rec_t r;
    r.op = 6'(op); r.sub = 4'(sub); r.na = 2'(na); r.a0 = a0; r.a1 = a1; r.a2 = a2; r.err = 8'(err);
    return r;
  endfunction

  task automatic expect_ok(input int op, input int sub, input int na,
                           input logic [63:0] a0 = 0, input logic [63:0] a1 = 0, input logic [63:0] a2 = 0);
    exp_q.push_back(mk(op, sub, na, a0, a1, a2, 0));
  endtask
  task automatic expect_err(input int code);
    exp_q.push_back(mk(OP_PERR, 0, 0, 0, 0, 0, code));
  endtask

  // compare the queues, report differences; neg_mode: count them as "detected" instead
  task automatic check_queues(input string tag);
    int n;
    bit bad;
    bad = 1'b0;
    if (got_q.size() != exp_q.size()) begin
      bad = 1'b1;
      if (!neg_mode) $display("ERROR [%s]: %0d commands received, %0d expected", tag, got_q.size(), exp_q.size());
    end
    n = (got_q.size() < exp_q.size()) ? got_q.size() : exp_q.size();
    for (int i = 0; i < n; i++) begin
      if (got_q[i] !== exp_q[i]) begin
        bad = 1'b1;
        if (!neg_mode)
          $display("ERROR [%s] #%0d: got op=%0d sub=%0d na=%0d a0=%h a1=%h a2=%h err=%02h  exp op=%0d sub=%0d na=%0d a0=%h a1=%h a2=%h err=%02h",
                   tag, i, got_q[i].op, got_q[i].sub, got_q[i].na, got_q[i].a0, got_q[i].a1, got_q[i].a2, got_q[i].err,
                   exp_q[i].op, exp_q[i].sub, exp_q[i].na, exp_q[i].a0, exp_q[i].a1, exp_q[i].a2, exp_q[i].err);
      end
    end
    if (bad) begin if (neg_mode) neg_detected++; else errors++; end
    got_q.delete(); exp_q.delete();
  endtask

  // one line, result checked at once
  task automatic line_ok(input string s, input int eol, input int op, input int sub, input int na,
                         input logic [63:0] a0 = 0, input logic [63:0] a1 = 0, input logic [63:0] a2 = 0);
    send_text(s, eol);
    wait_clks(60);
    expect_ok(op, sub, na, a0, a1, a2);
    check_queues(s);
  endtask
  task automatic line_err(input string s, input int eol, input int code);
    send_text(s, eol);
    wait_clks(60);
    expect_err(code);
    check_queues(s);
  endtask

  function automatic logic [3:0] hexv(input byte c);
    return (c <= "9") ? 4'(c - "0") : 4'(c - "A" + 10);
  endfunction

  function automatic string padto(input string s, input int n);
    string r;
    r = s;
    while (r.len() < n) r = $sformatf("%s%c", r, 8'h20);
    return r;
  endfunction

  function automatic string rep(input string ch, input int n);
    string r;
    r = "";
    for (int i = 0; i < n; i++) r = $sformatf("%s%s", r, ch);
    return r;
  endfunction

  initial begin
    wait_clks(10);
    rst = 1'b0;
    wait_clks(20);

    // ---------------------------------------------------------------- every command (LF)
    line_ok("PING",              0, OP_PING,   0, 0);
    line_ok("STAT?",             0, OP_STAT,   0, 0);
    line_ok("CFG?",              0, OP_CFGQ,   0, 0);
    line_ok("RES?",              0, OP_RESQ,   0, 0);
    line_ok("TEL 03E8",          0, OP_TEL,    0, 1, 64'h03E8);
    line_ok("TEL 0",             0, OP_TEL,    0, 1, 64'h0);
    line_ok("TEL 000A",          0, OP_TEL,    0, 1, 64'h0A);
    line_ok("SMART 0064",        0, OP_SMART,  0, 1, 64'h64);
    line_ok("SMART?",            0, OP_SMARTQ, 0, 0);
    line_ok("MODE RT",           0, OP_MODE,   0, 0);
    line_ok("MODE BP",           0, OP_MODE,   1, 0);
    line_ok("RATE 2CCCCCCC",     0, OP_RATE,   0, 1, 64'h2CCCCCCC);
    line_ok("LEN 00000008",      0, OP_LEN,    0, 1, 64'h8);
    line_ok("SID 692D3A00",      0, OP_SID,    0, 1, 64'h692D3A00);
    line_ok("DDRSZ 0020",        0, OP_DDRSZ,  0, 1, 64'h20);
    line_ok("PROF CLR",          0, OP_PROF_CLR, 0, 0);
    line_ok("PROF ADD 2CCCCCCC 00B2D05E00", 0, OP_PROF_ADD, 0, 2, 64'h2CCCCCCC, 64'hB2D05E00);
    line_ok("PROF ADD 0 0002FAF080",        0, OP_PROF_ADD, 0, 2, 64'h0, 64'h2FAF080);
    line_ok("PROF SET 01 2CCCCCCC 0000000100", 0, OP_PROF_SET, 0, 3, 64'h1, 64'h2CCCCCCC, 64'h100);
    line_ok("PROF REP 0004",     0, OP_PROF_REP, 0, 1, 64'h4);
    line_ok("PROF END HOLD",     0, OP_PROF_END, 0, 0);
    line_ok("PROF END STOP",     0, OP_PROF_END, 1, 0);
    line_ok("PROF?",             0, OP_PROFQ,  0, 0);
    line_ok("REC",               0, OP_REC,    0, 0);
    line_ok("REC PROF",          0, OP_REC_PROF, 0, 0);
    line_ok("STOP",              0, OP_STOP,   0, 0);
    line_ok("PFAIL",             0, OP_PFAIL,  0, 0);
    line_ok("READ LAST",         0, OP_READ_LAST, 0, 0);
    line_ok("READ 00000020",     0, OP_READ_N, 0, 1, 64'h20);
    line_ok("READ END",          0, OP_READ_END, 0, 0);
    line_ok("READ SID 692D3A00", 0, OP_READ_SID, 0, 1, 64'h692D3A00);
    line_ok("FORMAT YES",        0, OP_FORMAT, 0, 0);
    line_ok("MON",               0, OP_MON,    0, 0);
    line_ok("MEMTEST",           0, OP_MEMTEST, 0, 0);
    line_ok("MEMTEST 0020",      0, OP_MEMTEST, 0, 1, 64'h20);
    line_ok("DTEST",             0, OP_DTEST,  0, 0);
    line_ok("DDR?",              0, OP_DDRQ,   0, 0);
    line_ok("ABORT",             0, OP_ABORT,  0, 0);

    // ---------------------------------------------------------------- hex widths 1..16, case
    begin
      string digs;
      logic [63:0] v;
      digs = "FEDCBA9876543210";
      v = 64'd0;
      for (int n = 1; n <= 16; n++) begin
        v = 64'h0;
        for (int i = 0; i < n; i++) v = {v[59:0], hexv(digs.getc(16 - n + i))};
        line_ok({"SID ", digs.substr(16 - n, 15)}, 0, OP_SID, 0, 1, v);
      end
    end
    line_ok("rate 2cccccCC",               1, OP_RATE, 0, 1, 64'h2CCCCCCC);
    line_ok("Prof Add abcdef01 0000000ff", 0, OP_PROF_ADD, 0, 2, 64'hABCDEF01, 64'hFF);
    line_ok("mode bp",                     0, OP_MODE, 1, 0);
    line_ok("read last",                   0, OP_READ_LAST, 0, 0);
    line_ok("Format yes",                  0, OP_FORMAT, 0, 0);
    line_ok("SID 0000000000000000",        0, OP_SID, 0, 1, 64'h0);

    // ---------------------------------------------------------------- hex-letter words
    line_ok("PROF ADD ADD ADD",  0, OP_PROF_ADD, 0, 2, 64'hADD, 64'hADD);   // word, then two numbers
    line_ok("PROF SET ADD BEEF dead", 0, OP_PROF_SET, 0, 3, 64'hADD, 64'hBEEF, 64'hDEAD);
    line_ok("READ ADD",          0, OP_READ_N, 0, 1, 64'hADD);               // a number here
    line_ok("TEL ADD",           0, OP_TEL, 0, 1, 64'hADD);
    line_ok("READ BEEF",         0, OP_READ_N, 0, 1, 64'hBEEF);
    line_ok("READ SID ADD",      0, OP_READ_SID, 0, 1, 64'hADD);
    line_err("PROF BEEF 1 2",    0, ERR_SYNTAX);                              // BEEF is no sub-word

    // ---------------------------------------------------------------- blanks and tabs
    line_ok("   PING",           0, OP_PING, 0, 0);
    line_ok("PING   ",           0, OP_PING, 0, 0);
    line_ok("\t PING \t",        0, OP_PING, 0, 0);
    line_ok(" PROF \t ADD  1   2  ", 0, OP_PROF_ADD, 0, 2, 64'h1, 64'h2);

    // ---------------------------------------------------------------- line terminators
    line_ok("PING", 1, OP_PING, 0, 0);          // CR
    line_ok("PING", 2, OP_PING, 0, 0);          // CRLF: one line, the LF is a blank line
    line_ok("PING", 3, OP_PING, 0, 0);          // LFCR
    line_ok("TEL 0064", 2, OP_TEL, 0, 1, 64'h64);
    // blank lines in any form produce nothing
    send_text("", 0); send_text("", 1); send_text("", 2); send_text("   ", 0); send_text("\t", 2);
    wait_clks(60);
    check_queues("blank lines");
    // a line without EOL produces nothing until the EOL arrives
    send_text("STOP", 4);
    wait_clks(60);
    check_queues("no EOL yet");
    send_text("", 0);
    wait_clks(60);
    expect_ok(OP_STOP, 0, 0);
    check_queues("EOL later");

    // ---------------------------------------------------------------- syntax errors (01)
    line_err("PING 1",                 0, ERR_SYNTAX);
    line_err("PING PING",              0, ERR_SYNTAX);
    line_err("STAT? 0",                0, ERR_SYNTAX);
    line_err("TEL",                    0, ERR_SYNTAX);
    line_err("TEL ZZ",                 0, ERR_SYNTAX);
    line_err("TEL 0x10",               0, ERR_SYNTAX);
    line_err("TEL -1",                 0, ERR_SYNTAX);
    line_err("TEL 1 2",                0, ERR_SYNTAX);
    line_err("TEL 1,2",                0, ERR_SYNTAX);
    line_err("RATE STOP",              0, ERR_SYNTAX);
    line_err("MODE",                   0, ERR_SYNTAX);
    line_err("MODE XX",                0, ERR_SYNTAX);
    line_err("MODE RT BP",             0, ERR_SYNTAX);
    line_err("MODE 1",                 0, ERR_SYNTAX);
    line_err("PROF",                   0, ERR_SYNTAX);
    line_err("PROF FOO",               0, ERR_SYNTAX);
    line_err("PROF CLR 1",             0, ERR_SYNTAX);
    line_err("PROF ADD 1",             0, ERR_SYNTAX);
    line_err("PROF ADD 1 2 3",         0, ERR_SYNTAX);
    line_err("PROF SET 1 2",           0, ERR_SYNTAX);
    line_err("PROF SET 1 2 3 4",       0, ERR_SYNTAX);
    line_err("PROF REP",               0, ERR_SYNTAX);
    line_err("PROF END",               0, ERR_SYNTAX);
    line_err("PROF END ZERO",          0, ERR_SYNTAX);
    line_err("PROF END HOLD HOLD",     0, ERR_SYNTAX);
    line_err("REC FOO",                0, ERR_SYNTAX);
    line_err("REC PROF PROF",          0, ERR_SYNTAX);
    line_err("REC 1",                  0, ERR_SYNTAX);
    line_err("STOP NOW",               0, ERR_SYNTAX);
    line_err("READ",                   0, ERR_SYNTAX);
    line_err("READ SID",               0, ERR_SYNTAX);
    line_err("READ LAST 1",            0, ERR_SYNTAX);
    line_err("READ END END",           0, ERR_SYNTAX);
    line_err("READ ZZZ",               0, ERR_SYNTAX);
    line_err("READ 1 2",               0, ERR_SYNTAX);
    line_err("FORMAT",                 0, ERR_SYNTAX);
    line_err("FORMAT NO",              0, ERR_SYNTAX);
    line_err("FORMAT YES YES",         0, ERR_SYNTAX);
    line_err("MEMTEST 1 2",            0, ERR_SYNTAX);
    line_err("MEMTEST ZZ",             0, ERR_SYNTAX);
    line_err("PI$NG",                  0, ERR_SYNTAX);        // bad character in the keyword
    line_err("$T v=01",                0, ERR_SYNTAX);
    line_err("PING =",                 0, ERR_SYNTAX);
    line_err("PING 1 2 3 4 5",         0, ERR_SYNTAX);
    // ---------------------------------------------------------------- unknown keyword (02)
    line_err("FOO",                    0, ERR_UNKNOWN);
    line_err("FOO 1 2",                0, ERR_UNKNOWN);      // first error wins
    line_err("PIN",                    0, ERR_UNKNOWN);
    line_err("PINGG",                  0, ERR_UNKNOWN);
    line_err("123",                    0, ERR_UNKNOWN);
    line_err("ADD",                    0, ERR_UNKNOWN);      // sub-word is no keyword
    line_err("CLR",                    0, ERR_UNKNOWN);
    line_err("LONGKEYWORDX",           0, ERR_UNKNOWN);      // > 8 letters
    line_err("?",                      0, ERR_UNKNOWN);
    line_err("PING?",                  0, ERR_UNKNOWN);
    line_err("STAT",                   0, ERR_UNKNOWN);      // without '?'
    // ---------------------------------------------------------------- out of range (03): > 16 digits
    line_err("TEL 12345678901234567",  0, ERR_RANGE);
    line_err("SID 00000000000000001",  0, ERR_RANGE);
    line_err("READ FFFFFFFFFFFFFFFFF", 0, ERR_RANGE);
    line_err("PROF ADD 1 11111111111111111", 0, ERR_RANGE);
    line_ok ("SID FFFFFFFFFFFFFFFF",   0, OP_SID, 0, 1, 64'hFFFFFFFFFFFFFFFF);  // 16 digits are fine
    // ---------------------------------------------------------------- line length (0A)
    begin
      string l64, l65, l100;
      l64  = padto("PROF SET 01 2CCCCCCC 0000000100", 64);
      if (l64.len() != 64) begin errors++; $display("tb: l64 length %0d", l64.len()); end
      line_ok(l64, 0, OP_PROF_SET, 0, 3, 64'h1, 64'h2CCCCCCC, 64'h100);   // exactly 64 characters
      l65 = padto("PROF SET 01 2CCCCCCC 0000000100", 65);
      line_err(l65, 0, ERR_LONG);                                          // 65 characters
      l100 = padto("PING", 100);
      line_err(l100, 0, ERR_LONG);
      line_err({padto("", 70), "PING"}, 0, ERR_LONG);
      line_err({"FOO ", rep("X", 70)}, 0, ERR_LONG);                        // 0A wins over 02
      line_err({"PING ", rep("1", 70)}, 2, ERR_LONG);                       // CRLF: one error only
      line_ok("PING", 0, OP_PING, 0, 0);                                    // parser recovered
    end
    // ---------------------------------------------------------------- stop-bit error in a line (0D)
    send_byte("P"); send_byte("I"); send_byte("X", 1'b0); wait_clks(DIV); send_byte("N"); send_byte("G"); send_byte(8'h0A);
    wait_clks(60);
    expect_err(ERR_UART);
    check_queues("ferr in line");
    send_byte(8'h5A, 1'b0); wait_clks(DIV); send_byte(8'h0A);              // only a bad byte
    wait_clks(60);
    expect_err(ERR_UART);
    check_queues("ferr alone");
    send_byte("T"); send_byte("E"); send_byte("L"); send_byte(" "); send_byte("1"); send_byte(8'h0A, 1'b0);
    wait_clks(60);                                                          // EOL itself corrupted: no line yet
    check_queues("ferr EOL");
    send_byte(8'h0A);                                                       // the line ends now: 0D
    wait_clks(60);
    expect_err(ERR_UART);
    check_queues("ferr EOL 2");
    line_ok("PING", 0, OP_PING, 0, 0);                                      // recovered

    // ---------------------------------------------------------------- back-to-back lines
    begin
      string seq;
      seq = "PING\nSTAT?\nTEL 0064\nPROF ADD 1 2\n\nPROF END STOP\015\nREAD LAST\015FOO\nSID 1\nREC PROF\nSTOP\nABORT\n";
      for (int i = 0; i < seq.len(); i++) send_byte(seq.getc(i));
      wait_clks(60);
      expect_ok(OP_PING, 0, 0);
      expect_ok(OP_STAT, 0, 0);
      expect_ok(OP_TEL, 0, 1, 64'h64);
      expect_ok(OP_PROF_ADD, 0, 2, 64'h1, 64'h2);
      expect_ok(OP_PROF_END, 1, 0);
      expect_ok(OP_READ_LAST, 0, 0);
      expect_err(ERR_UNKNOWN);
      expect_ok(OP_SID, 0, 1, 64'h1);
      expect_ok(OP_REC_PROF, 0, 0);
      expect_ok(OP_STOP, 0, 0);
      expect_ok(OP_ABORT, 0, 0);
      check_queues("back to back");
    end

    // ---------------------------------------------------------------- previous command not taken (0C)
    cmd_ready = 1'b0;
    send_text("PING", 0);
    wait_clks(60);
    check(cmd_valid === 1'b1 && cmd_op == OP_PING, "PING waits for cmd_ready");
    send_text("STAT?", 0);               // dropped: ERR 0C later
    send_text("FOO", 0);                 // dropped too: still a single 0C
    wait_clks(60);
    check(cmd_valid === 1'b1 && cmd_op == OP_PING, "PING still held");
    cmd_ready = 1'b1;
    wait_clks(100);
    expect_ok(OP_PING, 0, 0);
    expect_err(ERR_BUSY);
    check_queues("ERR 0C");
    // a slow consumer that releases before the next line ends: no drop
    cmd_ready = 1'b0;
    send_text("PING", 0);
    wait_clks(400);
    cmd_ready = 1'b1;
    wait_clks(5);
    send_text("STOP", 0);
    wait_clks(60);
    expect_ok(OP_PING, 0, 0);
    expect_ok(OP_STOP, 0, 0);
    check_queues("slow consumer");
    // the line that ends while the consumer is not ready is dropped, the following one works
    cmd_ready = 1'b0;
    send_text("MON", 0);
    send_text("DTEST", 0);
    cmd_ready = 1'b1;
    wait_clks(100);
    send_text("DDR?", 0);
    wait_clks(60);
    expect_ok(OP_MON, 0, 0);
    expect_err(ERR_BUSY);
    expect_ok(OP_DDRQ, 0, 0);
    check_queues("drop then recover");

    // ---------------------------------------------------------------- reset in the middle of a line
    send_text("PROF ADD 1", 4);
    rst = 1'b1; wait_clks(3); rst = 1'b0; wait_clks(5);
    send_text("PING", 0);
    wait_clks(60);
    expect_ok(OP_PING, 0, 0);
    check_queues("reset in a line");

    // ---------------------------------------------------------------- negative control of the scoreboard
    neg_mode = 1'b1;
    send_text("PING", 0); wait_clks(60);
    expect_ok(OP_STAT, 0, 0);                                    // deliberately wrong op
    check_queues("neg op");
    send_text("TEL 0064", 0); wait_clks(60);
    expect_ok(OP_TEL, 0, 1, 64'h65);                             // wrong argument
    check_queues("neg arg");
    send_text("FOO", 0); wait_clks(60);
    expect_err(ERR_SYNTAX);                                      // wrong error code
    check_queues("neg err");
    send_text("PING", 0); wait_clks(60);
    check_queues("neg missing");                                 // expected nothing, got one
    neg_mode = 1'b0;
    check(neg_detected == 4, $sformatf("negative control: %0d of 4 detected", neg_detected));

    if (errors != 0) $fatal(1, "tb_cmd_parser: %0d errors", errors);
    $display("tb_cmd_parser: %0d commands handled", ncmd_total);
    $display("TEST PASSED");
    $finish;
  end

  task automatic check(input bit cond, input string msg);
    if (!cond) begin errors++; $display("ERROR: %s", msg); end
  endtask

  initial begin
    #600ms;
    $fatal(1, "tb_cmd_parser: timeout");
  end
endmodule
