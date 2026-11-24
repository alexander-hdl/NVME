// =============================================================================
// tb_rec_pkg.sv — rec_pkg: crc32_128 against zlib vectors, smart_t layout (spec 6.3),
// struct widths.  Negative control: a deliberately wrong expected CRC must be flagged
// by the checker (the check task is exercised with a bad value and must report an error).
// =============================================================================
`timescale 1ns/1ps

module tb_rec_pkg;
  import rec_pkg::*;

  int errors = 0;
  int detected_bad = 0;

  function automatic logic [31:0] zlib_crc_of(input logic [127:0] d0, input logic [127:0] d1, input int nbeats);
    logic [31:0] c;
    c = CRC32_INIT;
    c = crc32_128(c, d0);
    if (nbeats > 1) c = crc32_128(c, d1);
    return ~c;
  endfunction

  task automatic check_crc(input logic [127:0] d0, input logic [127:0] d1, input int nb,
                           input logic [31:0] exp_crc, input bit expect_fail);
    logic [31:0] got;
    got = zlib_crc_of(d0, d1, nb);
    if (got !== exp_crc) begin
      if (expect_fail) detected_bad++;
      else begin errors++; $display("ERROR: crc got %h exp %h", got, exp_crc); end
    end else if (expect_fail) begin
      errors++; $display("ERROR: wrong expected CRC was not detected");
    end
  endtask

  logic [127:0] a, b;
  smart_t sm;

  initial begin
    // bytes 0..15 and 16..31 (little endian: byte k = bits [8k+7:8k])
    for (int k = 0; k < 16; k++) begin a[8*k +: 8] = k[7:0]; b[8*k +: 8] = 8'(16 + k); end
    // vectors from Python zlib.crc32
    check_crc(a, b, 1, 32'hcecee288, 1'b0);
    check_crc(a, b, 2, 32'h91267e8a, 1'b0);
    a = {"F","E","D","C","B","A","9","8","7","6","5","4","3","2","1","0"};
    check_crc(a, b, 1, 32'h983c37b5, 1'b0);
    // negative control: wrong expectation must be detected
    check_crc(a, b, 1, 32'h983c37b4, 1'b1);
    if (detected_bad != 1) begin errors++; $display("ERROR: negative control not exercised"); end

    // widths
    if ($bits(smart_t) != 384) begin errors++; $display("ERROR: smart_t width %0d", $bits(smart_t)); end
    if ($bits(xadc_t)  != 49)  begin errors++; $display("ERROR: xadc_t width %0d", $bits(xadc_t)); end
    $display("widths: rec_cfg_t=%0d ui_stat_t=%0d core_stat_t=%0d vrf_res_t=%0d rec_snap_t=%0d",
             $bits(rec_cfg_t), $bits(ui_stat_t), $bits(core_stat_t), $bits(vrf_res_t), $bits(rec_snap_t));

    // smart_t layout per spec 6.3: [7:0] cw, [23:8] tk, [31:24] sp, [39:32] used, [111:48] dur,
    // [175:112] duw, [207:176] pc, [239:208] poh, [271:240] uns, [303:272] me, [335:304] wt,
    // [367:336] ct, [383:368] t1
    sm = '0;
    sm.cw = 8'hA1; sm.tk = 16'hB2C3; sm.sp = 8'hD4; sm.sp_used = 8'hE5;
    sm.dur = 64'h0102030405060708; sm.duw = 64'h1112131415161718;
    sm.pc = 32'h21222324; sm.poh = 32'h31323334; sm.uns = 32'h41424344; sm.me = 32'h51525354;
    sm.wt = 32'h61626364; sm.ct = 32'h71727374; sm.t1 = 16'h8182;
    if (sm[7:0]     !== 8'hA1)               begin errors++; $display("ERROR: smart cw"); end
    if (sm[23:8]    !== 16'hB2C3)            begin errors++; $display("ERROR: smart tk"); end
    if (sm[31:24]   !== 8'hD4)               begin errors++; $display("ERROR: smart sp"); end
    if (sm[39:32]   !== 8'hE5)               begin errors++; $display("ERROR: smart used"); end
    if (sm[47:40]   !== 8'h00)               begin errors++; $display("ERROR: smart rsvd"); end
    if (sm[111:48]  !== 64'h0102030405060708) begin errors++; $display("ERROR: smart dur"); end
    if (sm[175:112] !== 64'h1112131415161718) begin errors++; $display("ERROR: smart duw"); end
    if (sm[207:176] !== 32'h21222324)        begin errors++; $display("ERROR: smart pc"); end
    if (sm[239:208] !== 32'h31323334)        begin errors++; $display("ERROR: smart poh"); end
    if (sm[271:240] !== 32'h41424344)        begin errors++; $display("ERROR: smart uns"); end
    if (sm[303:272] !== 32'h51525354)        begin errors++; $display("ERROR: smart me"); end
    if (sm[335:304] !== 32'h61626364)        begin errors++; $display("ERROR: smart wt"); end
    if (sm[367:336] !== 32'h71727374)        begin errors++; $display("ERROR: smart ct"); end
    if (sm[383:368] !== 16'h8182)            begin errors++; $display("ERROR: smart t1"); end

    if (errors != 0) $fatal(1, "tb_rec_pkg: %0d errors", errors);
    $display("TEST PASSED");
    $finish;
  end
endmodule
