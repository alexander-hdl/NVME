// =============================================================================
// rec_uart_bfm.sv - UART bus-functional model of the PC side for the recorder system tests (WP8).
// 921600 8N1 with real bit timing (BIT_NS).  Drives the DUT's uart_rx pin (txd) and decodes the DUT's uart_tx pin (rxd)
// into text lines (LF terminated, CR ignored).  Tests use the tasks below hierarchically.
//   send_line(s) / send_raw(b) / send_bad_stop(b)   PC -> device
//   cmd(s, reply, tmo_us)    sends s, returns the first OK/ERR line that arrives afterwards (events/$T stay in lines[])
//   wait_line(prefix, from, tmo_us, idx)             first line with that prefix at index >= from
//   fld(line, key)           hex value of " key=" in a line, -1 if absent
// All received lines are kept in lines[]; dump() writes them to a file (fed to tools/recorder.py parse_line by the
// regression script / by hand).
// =============================================================================
`timescale 1ns/1ps
module rec_uart_bfm #(
  parameter real BIT_NS = 1085.069             // 1 / 921600
) (
  output logic txd,                            // -> DUT uart_rx
  input  logic rxd                             // <- DUT uart_tx
);
  string lines [$];
  int    framing_errs = 0;
  longint rx_bytes = 0;
  string cur = "";

  initial txd = 1'b1;

  // ---- receiver ----
  logic [7:0] rb;
  initial begin
    forever begin
      @(negedge rxd);
      #(BIT_NS * 1.5);
      for (int i = 0; i < 8; i++) begin
        rb[i] = rxd;
        #(BIT_NS);
      end
      if (rxd !== 1'b1) framing_errs++;
      rx_bytes++;
      if (rb == 8'h0A) begin
        if (cur.len() > 0) begin
          lines.push_back(cur);
          if (cur.len() < 3 || cur.substr(0, 2) != "$T ") $display("[%0t] UART< %s", $time, cur);
        end
        cur = "";
      end else if (rb != 8'h0D) begin
        cur = {cur, string'(rb)};
      end
    end
  end

  // ---- transmitter ----
  task automatic send_raw(input logic [7:0] b);
    txd = 1'b0; #(BIT_NS);
    for (int i = 0; i < 8; i++) begin txd = b[i]; #(BIT_NS); end
    txd = 1'b1; #(BIT_NS);
  endtask

  // byte with the stop bit forced to 0 (framing error), then the line returns high for 2 bit times
  task automatic send_bad_stop(input logic [7:0] b);
    txd = 1'b0; #(BIT_NS);
    for (int i = 0; i < 8; i++) begin txd = b[i]; #(BIT_NS); end
    txd = 1'b0; #(BIT_NS);
    txd = 1'b1; #(2 * BIT_NS);
  endtask

  task automatic send_line(input string s);
    $display("[%0t] UART> %s", $time, s);
    for (int i = 0; i < s.len(); i++) send_raw(s.getc(i));
    send_raw(8'h0A);
  endtask

  // ---- line access ----
  function automatic int find_line(input string pfx, input int from);
    for (int i = from; i < lines.size(); i++)
      if (lines[i].len() >= pfx.len() && lines[i].substr(0, pfx.len() - 1) == pfx) return i;
    return -1;
  endfunction

  task automatic wait_line(input string pfx, input int from, input int tmo_us, output int idx);
    int t;
    idx = find_line(pfx, from);
    t = 0;
    while (idx < 0 && t < tmo_us) begin
      #(1000.0);
      t++;
      idx = find_line(pfx, from);
    end
  endtask

  // first OK / ERR line after the command was sent
  task automatic cmd(input string c, output string reply, input int tmo_us = 20000);
    int mark, t, i;
    mark = lines.size();
    send_line(c);
    reply = "";
    t = 0;
    while (reply == "" && t < tmo_us) begin
      for (i = mark; i < lines.size(); i++)
        if ((lines[i].len() >= 2 && lines[i].substr(0, 1) == "OK") || (lines[i].len() >= 3 && lines[i].substr(0, 2) == "ERR")) begin
          reply = lines[i];
          break;
        end
      if (reply == "") begin #(1000.0); t++; end
    end
  endtask

  // hex value of " key=" (key preceded by a space); -1 if absent / not hex
  function automatic longint fld(input string line, input string key);
    string k;
    int p, n;
    longint v;
    k = {" ", key, "="};
    n = line.len();
    for (p = 0; p + k.len() <= n; p++) begin
      if (line.substr(p, p + k.len() - 1) == k) begin
        v = 0;
        for (int j = p + k.len(); j < n; j++) begin
          byte c;
          c = line.getc(j);
          if (c >= "0" && c <= "9")      v = (v << 4) | (c - "0");
          else if (c >= "A" && c <= "F") v = (v << 4) | (c - "A" + 10);
          else if (c >= "a" && c <= "f") v = (v << 4) | (c - "a" + 10);
          else break;
        end
        return v;
      end
    end
    return -1;
  endfunction

  function automatic bit has(input string line, input string key);
    return (fld(line, key) >= 0);
  endfunction

  task automatic dump(input string fn);
    int fd;
    fd = $fopen(fn, "w");
    foreach (lines[i]) $fdisplay(fd, "%s", lines[i]);
    $fclose(fd);
  endtask
endmodule
