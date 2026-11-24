// =============================================================================
// rec_cmd_parser.sv — character-serial command parser of the recorder (WP1).
//
// Domain  : link_clk (200 MHz).  Spec: STAND/RECORDER_SPEC.md 2.1-2.4, 4.4 (WP1).
// Input   : rx_data/rx_valid (1-clock pulse per byte, from uart_rx), rx_ferr (1-clock pulse,
//           stop-bit error).  Bytes must be at least 8 clocks apart (a UART gives >= 2000).
// Output  : one command per line through cmd_valid/cmd_ready (cmd_valid is held, with all
//           payload, until cmd_ready; payload is registered).
//             cmd_op     rec_op_e.  OP_PERR = the line is wrong, cmd_err holds the ERR code.
//             cmd_sub    MODE: 0 RT, 1 BP;  PROF END: 0 HOLD, 1 STOP;  otherwise 0.
//             cmd_nargs  number of numeric arguments delivered in cmd_arg0..2 (0..3).
//             cmd_arg0.. numeric arguments in order of appearance, zero extended (<= 16 digits).
//                        PROF ADD: step, dur;  PROF SET: i, step, dur;
//                        READ SID / READ <n> / TEL / SMART / RATE / LEN / SID / DDRSZ /
//                        PROF REP / MEMTEST: arg0.
//             cmd_err    ERR code (spec 2.4) when cmd_op = OP_PERR, else 0.
// Line    : WS* keyword (WS+ arg){0,3} WS* EOL;  WS = space or tab;  EOL = CR, LF or CRLF
//           (or LFCR); empty / blank lines are ignored (so CRLF is one line).  Case-insensitive.
//           A line is at most 64 characters (excluding EOL).
// Errors  : 0D stop-bit error in the line (wins over everything else), 0A line > 64 characters,
//           02 unknown keyword, 01 syntax (bad character, wrong/missing/extra argument, unknown
//           sub-word), 03 hex argument with more than 16 digits, 0C a line ended while the
//           previous command had not yet been taken (cmd_valid && !cmd_ready): the new line is
//           dropped and a single OP_PERR/0C is issued after the previous command is taken.
//           The first error of a line wins (except 0D, 0A).  Only the lexical form is checked
//           here; value ranges and the state check (ERR 03/04/05...) are done by rec_ctrl.
// Words   : a token made only of hex letters (ADD) is a word where a word is expected
//           (PROF ADD) and a number elsewhere (READ ADD = 0xADD slots).
//           The word table has 34 entries (24 keywords + 10 sub-words), matched in parallel.
// =============================================================================
module rec_cmd_parser
  import rec_pkg::*;
(
  input  logic        clk,
  input  logic        rst,
  input  logic [7:0]  rx_data,     // link
  input  logic        rx_valid,    // link
  input  logic        rx_ferr,     // link
  output logic        cmd_valid,   // link
  input  logic        cmd_ready,   // link
  output logic [5:0]  cmd_op,      // link, rec_op_e
  output logic [3:0]  cmd_sub,     // link
  output logic [1:0]  cmd_nargs,   // link
  output logic [63:0] cmd_arg0,    // link
  output logic [63:0] cmd_arg1,    // link
  output logic [63:0] cmd_arg2,    // link
  output logic [7:0]  cmd_err      // link
);
  // ---------------------------------------------------------------- word table
  localparam logic [5:0]
    W_NONE = 6'd0,  W_PING = 6'd1,  W_STATQ = 6'd2,  W_CFGQ = 6'd3,   W_RESQ = 6'd4,
    W_TEL = 6'd5,   W_SMART = 6'd6, W_SMARTQ = 6'd7, W_MODE = 6'd8,   W_RATE = 6'd9,
    W_LEN = 6'd10,  W_SID = 6'd11,  W_DDRSZ = 6'd12, W_PROF = 6'd13,  W_PROFQ = 6'd14,
    W_REC = 6'd15,  W_STOP = 6'd16, W_PFAIL = 6'd17, W_READ = 6'd18,  W_FORMAT = 6'd19,
    W_MON = 6'd20,  W_MEMTEST = 6'd21, W_DTEST = 6'd22, W_DDRQ = 6'd23, W_ABORT = 6'd24,
    // sub-words (not keywords)
    W_CLR = 6'd25,  W_ADD = 6'd26,  W_REP = 6'd27,   W_SET = 6'd28,   W_END = 6'd29,
    W_HOLD = 6'd30, W_LAST = 6'd31, W_YES = 6'd32,   W_RT = 6'd33,    W_BP = 6'd34;

  // what the next token must be
  typedef enum logic [2:0] {E_NONE, E_HEX, E_MODE, E_PROF, E_END, E_RECW, E_READ, E_YES} exp_e;
  typedef enum logic [1:0] {PH_IDLE, PH_DEC, PH_APPLY, PH_FIN} ph_e;

  function automatic logic [5:0] word_of(input logic [63:0] b, input logic [4:0] len,
                                         input logic valid);
    logic [5:0] w;
    w = W_NONE;
    if (valid && len <= 5'd8) begin
      case (b)
        "PING":    w = W_PING;    "STAT?":  w = W_STATQ;   "CFG?":    w = W_CFGQ;
        "RES?":    w = W_RESQ;    "TEL":    w = W_TEL;     "SMART":   w = W_SMART;
        "SMART?":  w = W_SMARTQ;  "MODE":   w = W_MODE;    "RATE":    w = W_RATE;
        "LEN":     w = W_LEN;     "SID":    w = W_SID;     "DDRSZ":   w = W_DDRSZ;
        "PROF":    w = W_PROF;    "PROF?":  w = W_PROFQ;   "REC":     w = W_REC;
        "STOP":    w = W_STOP;    "PFAIL":  w = W_PFAIL;   "READ":    w = W_READ;
        "FORMAT":  w = W_FORMAT;  "MON":    w = W_MON;     "MEMTEST": w = W_MEMTEST;
        "DTEST":   w = W_DTEST;   "DDR?":   w = W_DDRQ;    "ABORT":   w = W_ABORT;
        "CLR":     w = W_CLR;     "ADD":    w = W_ADD;     "REP":     w = W_REP;
        "SET":     w = W_SET;     "END":    w = W_END;     "HOLD":    w = W_HOLD;
        "LAST":    w = W_LAST;    "YES":    w = W_YES;     "RT":      w = W_RT;
        "BP":      w = W_BP;
        default:   w = W_NONE;
      endcase
    end
    return w;
  endfunction

  function automatic logic [7:0] up(input logic [7:0] c);
    return (c >= 8'h61 && c <= 8'h7A) ? (c - 8'h20) : c;
  endfunction
  function automatic logic is_hex(input logic [7:0] c);   // c already upper case
    return (c >= 8'h30 && c <= 8'h39) || (c >= 8'h41 && c <= 8'h46);
  endfunction
  function automatic logic [3:0] hexval(input logic [7:0] c);
    return (c >= 8'h41) ? 4'(c - 8'h37) : 4'(c - 8'h30);
  endfunction
  function automatic logic is_tokch(input logic [7:0] c); // [A-Z0-9?]
    return (c >= 8'h30 && c <= 8'h39) || (c >= 8'h41 && c <= 8'h5A) || c == 8'h3F;
  endfunction

  // ---------------------------------------------------------------- state
  ph_e         ph_q;
  // token
  logic        in_tok_q, tok_pend_q, thex_q, tvalid_q;
  logic [4:0]  tlen_q;
  logic [63:0] tbuf_q, tval_q;
  logic [5:0]  wid_q;
  // line
  logic [6:0]  nch_q;
  logic        long_q, ferr_q, eol_q, have_kw_q, eol_ok_q, ovr_pend_q;
  logic [7:0]  err_q;
  exp_e        exp_q;
  logic [1:0]  hexl_q;
  logic [5:0]  op_q;
  logic [3:0]  sub_q;
  logic [1:0]  na_q;
  logic [63:0] a0_q, a1_q, a2_q;

  logic [7:0]  c_w;
  assign c_w = up(rx_data);

  always_ff @(posedge clk) begin
    if (cmd_valid && cmd_ready) cmd_valid <= 1'b0;   // may be overridden by a load below

    if (rst) begin
      cmd_valid  <= 1'b0;
      cmd_op     <= 6'(OP_NOP);
      cmd_sub    <= '0;
      cmd_nargs  <= '0;
      cmd_arg0   <= '0;
      cmd_arg1   <= '0;
      cmd_arg2   <= '0;
      cmd_err    <= '0;
      ph_q       <= PH_IDLE;
      in_tok_q   <= 1'b0;
      tok_pend_q <= 1'b0;
      nch_q      <= '0;
      long_q     <= 1'b0;
      ferr_q     <= 1'b0;
      eol_q      <= 1'b0;
      have_kw_q  <= 1'b0;
      eol_ok_q   <= 1'b0;
      ovr_pend_q <= 1'b0;
      err_q      <= '0;
      exp_q      <= E_NONE;
      hexl_q     <= '0;
      op_q       <= 6'(OP_NOP);
      sub_q      <= '0;
      na_q       <= '0;
      a0_q       <= '0;
      a1_q       <= '0;
      a2_q       <= '0;
    end else begin
      case (ph_q)
        // ------------------------------------------------ wait for characters
        PH_IDLE: begin
          if (rx_valid) begin
            if (c_w == 8'h0D || c_w == 8'h0A) begin
              eol_q      <= 1'b1;
              tok_pend_q <= in_tok_q;
              in_tok_q   <= 1'b0;
              ph_q       <= PH_DEC;
            end else begin
              if (nch_q != 7'd127) nch_q <= nch_q + 1'b1;
              if (nch_q >= 7'd64) begin
                long_q <= 1'b1;                         // 65th and later characters
              end else if (c_w == 8'h20 || c_w == 8'h09) begin
                if (in_tok_q) begin
                  in_tok_q   <= 1'b0;
                  tok_pend_q <= 1'b1;
                  ph_q       <= PH_DEC;
                end
              end else if (!in_tok_q) begin
                in_tok_q <= 1'b1;
                tlen_q   <= 5'd1;
                tbuf_q   <= {56'd0, c_w};
                tval_q   <= {60'd0, hexval(c_w)};
                thex_q   <= is_hex(c_w);
                tvalid_q <= is_tokch(c_w);
              end else begin
                if (tlen_q != 5'd31) tlen_q <= tlen_q + 1'b1;
                tbuf_q   <= {tbuf_q[55:0], c_w};
                tval_q   <= {tval_q[59:0], hexval(c_w)};
                thex_q   <= thex_q & is_hex(c_w);
                tvalid_q <= tvalid_q & is_tokch(c_w);
              end
            end
          end else if (ovr_pend_q && (!cmd_valid || cmd_ready)) begin
            cmd_valid  <= 1'b1;                          // ERR 0C for a line that was dropped
            cmd_op     <= 6'(OP_PERR);
            cmd_sub    <= '0;
            cmd_nargs  <= '0;
            cmd_arg0   <= '0;
            cmd_arg1   <= '0;
            cmd_arg2   <= '0;
            cmd_err    <= 8'(ERR_BUSY);
            ovr_pend_q <= 1'b0;
          end
        end

        // ------------------------------------------------ classify the finished token
        PH_DEC: begin
          wid_q <= word_of(tbuf_q, tlen_q, tvalid_q);
          ph_q  <= PH_APPLY;
        end

        // ------------------------------------------------ apply the grammar
        PH_APPLY: begin
          tok_pend_q <= 1'b0;
          ph_q       <= eol_q ? PH_FIN : PH_IDLE;
          if (tok_pend_q && err_q == 8'd0 && !long_q) begin
            if (!have_kw_q) begin
              // ---- keyword
              have_kw_q <= 1'b1;
              exp_q     <= E_NONE;
              eol_ok_q  <= 1'b1;
              hexl_q    <= '0;
              if (wid_q == W_NONE || wid_q > W_ABORT) begin
                err_q <= tvalid_q ? 8'(ERR_UNKNOWN) : 8'(ERR_SYNTAX);
              end else begin
                case (wid_q)
                  W_PING:    op_q <= 6'(OP_PING);
                  W_STATQ:   op_q <= 6'(OP_STAT);
                  W_CFGQ:    op_q <= 6'(OP_CFGQ);
                  W_RESQ:    op_q <= 6'(OP_RESQ);
                  W_SMARTQ:  op_q <= 6'(OP_SMARTQ);
                  W_PROFQ:   op_q <= 6'(OP_PROFQ);
                  W_STOP:    op_q <= 6'(OP_STOP);
                  W_PFAIL:   op_q <= 6'(OP_PFAIL);
                  W_MON:     op_q <= 6'(OP_MON);
                  W_DTEST:   op_q <= 6'(OP_DTEST);
                  W_DDRQ:    op_q <= 6'(OP_DDRQ);
                  W_ABORT:   op_q <= 6'(OP_ABORT);
                  W_TEL:     begin op_q <= 6'(OP_TEL);   exp_q <= E_HEX; hexl_q <= 2'd1; eol_ok_q <= 1'b0; end
                  W_SMART:   begin op_q <= 6'(OP_SMART); exp_q <= E_HEX; hexl_q <= 2'd1; eol_ok_q <= 1'b0; end
                  W_RATE:    begin op_q <= 6'(OP_RATE);  exp_q <= E_HEX; hexl_q <= 2'd1; eol_ok_q <= 1'b0; end
                  W_LEN:     begin op_q <= 6'(OP_LEN);   exp_q <= E_HEX; hexl_q <= 2'd1; eol_ok_q <= 1'b0; end
                  W_SID:     begin op_q <= 6'(OP_SID);   exp_q <= E_HEX; hexl_q <= 2'd1; eol_ok_q <= 1'b0; end
                  W_DDRSZ:   begin op_q <= 6'(OP_DDRSZ); exp_q <= E_HEX; hexl_q <= 2'd1; eol_ok_q <= 1'b0; end
                  W_MEMTEST: begin op_q <= 6'(OP_MEMTEST); exp_q <= E_HEX; hexl_q <= 2'd1; end   // optional
                  W_MODE:    begin op_q <= 6'(OP_MODE);  exp_q <= E_MODE; eol_ok_q <= 1'b0; end
                  W_PROF:    begin op_q <= 6'(OP_NOP);   exp_q <= E_PROF; eol_ok_q <= 1'b0; end
                  W_REC:     begin op_q <= 6'(OP_REC);   exp_q <= E_RECW; end                   // optional PROF
                  W_READ:    begin op_q <= 6'(OP_NOP);   exp_q <= E_READ; eol_ok_q <= 1'b0; end
                  W_FORMAT:  begin op_q <= 6'(OP_FORMAT); exp_q <= E_YES; eol_ok_q <= 1'b0; end
                  default:   err_q <= 8'(ERR_UNKNOWN);
                endcase
              end
            end else begin
              // ---- argument
              case (exp_q)
                E_HEX: begin
                  if (!thex_q)              err_q <= 8'(ERR_SYNTAX);
                  else if (tlen_q > 5'd16)  err_q <= 8'(ERR_RANGE);
                  else begin
                    case (na_q)
                      2'd0:    a0_q <= tval_q;
                      2'd1:    a1_q <= tval_q;
                      default: a2_q <= tval_q;
                    endcase
                    na_q   <= na_q + 1'b1;
                    hexl_q <= hexl_q - 1'b1;
                    if (hexl_q == 2'd1) begin exp_q <= E_NONE; eol_ok_q <= 1'b1; end
                  end
                end
                E_MODE: begin
                  if (wid_q == W_RT || wid_q == W_BP) begin
                    sub_q <= (wid_q == W_BP) ? 4'd1 : 4'd0;
                    exp_q <= E_NONE; eol_ok_q <= 1'b1;
                  end else err_q <= 8'(ERR_SYNTAX);
                end
                E_PROF: begin
                  exp_q <= E_NONE;
                  case (wid_q)
                    W_CLR:   begin op_q <= 6'(OP_PROF_CLR); eol_ok_q <= 1'b1; end
                    W_ADD:   begin op_q <= 6'(OP_PROF_ADD); exp_q <= E_HEX; hexl_q <= 2'd2; end
                    W_SET:   begin op_q <= 6'(OP_PROF_SET); exp_q <= E_HEX; hexl_q <= 2'd3; end
                    W_REP:   begin op_q <= 6'(OP_PROF_REP); exp_q <= E_HEX; hexl_q <= 2'd1; end
                    W_END:   begin op_q <= 6'(OP_PROF_END); exp_q <= E_END; end
                    default: err_q <= 8'(ERR_SYNTAX);
                  endcase
                end
                E_END: begin
                  if (wid_q == W_HOLD || wid_q == W_STOP) begin
                    sub_q <= (wid_q == W_STOP) ? 4'd1 : 4'd0;
                    exp_q <= E_NONE; eol_ok_q <= 1'b1;
                  end else err_q <= 8'(ERR_SYNTAX);
                end
                E_RECW: begin
                  if (wid_q == W_PROF) begin
                    op_q <= 6'(OP_REC_PROF); exp_q <= E_NONE; eol_ok_q <= 1'b1;
                  end else err_q <= 8'(ERR_SYNTAX);
                end
                E_YES: begin
                  if (wid_q == W_YES) begin exp_q <= E_NONE; eol_ok_q <= 1'b1; end
                  else err_q <= 8'(ERR_SYNTAX);
                end
                E_READ: begin
                  exp_q <= E_NONE;
                  if (wid_q == W_LAST)      begin op_q <= 6'(OP_READ_LAST); eol_ok_q <= 1'b1; end
                  else if (wid_q == W_END)  begin op_q <= 6'(OP_READ_END);  eol_ok_q <= 1'b1; end
                  else if (wid_q == W_SID)  begin op_q <= 6'(OP_READ_SID);  exp_q <= E_HEX; hexl_q <= 2'd1; end
                  else if (thex_q) begin
                    if (tlen_q > 5'd16) err_q <= 8'(ERR_RANGE);
                    else begin
                      op_q <= 6'(OP_READ_N); a0_q <= tval_q; na_q <= 2'd1; eol_ok_q <= 1'b1;
                    end
                  end else err_q <= 8'(ERR_SYNTAX);
                end
                default: err_q <= 8'(ERR_SYNTAX);       // E_NONE: extra argument
              endcase
            end
          end
        end

        // ------------------------------------------------ end of line: emit and reset
        PH_FIN: begin
          ph_q       <= PH_IDLE;
          nch_q      <= '0;
          long_q     <= 1'b0;
          ferr_q     <= 1'b0;
          eol_q      <= 1'b0;
          have_kw_q  <= 1'b0;
          err_q      <= '0;
          exp_q      <= E_NONE;
          hexl_q     <= '0;
          op_q       <= 6'(OP_NOP);
          sub_q      <= '0;
          na_q       <= '0;
          a0_q       <= '0;
          a1_q       <= '0;
          a2_q       <= '0;
          if (ferr_q || long_q || have_kw_q || err_q != 8'd0) begin
            if (ovr_pend_q || (cmd_valid && !cmd_ready)) begin
              ovr_pend_q <= 1'b1;                       // previous command not yet taken
            end else begin
              cmd_valid <= 1'b1;
              if (ferr_q || long_q || err_q != 8'd0 || !eol_ok_q) begin
                cmd_op    <= 6'(OP_PERR);
                cmd_sub   <= '0;
                cmd_nargs <= '0;
                cmd_arg0  <= '0;
                cmd_arg1  <= '0;
                cmd_arg2  <= '0;
                cmd_err   <= ferr_q ? 8'(ERR_UART) : long_q ? 8'(ERR_LONG)
                           : (err_q != 8'd0) ? err_q : 8'(ERR_SYNTAX);
              end else begin
                cmd_op    <= op_q;
                cmd_sub   <= sub_q;
                cmd_nargs <= na_q;
                cmd_arg0  <= a0_q;
                cmd_arg1  <= a1_q;
                cmd_arg2  <= a2_q;
                cmd_err   <= '0;
              end
            end
          end
        end
        default: ph_q <= PH_IDLE;
      endcase

      // a byte that arrives while the line logic is busy is lost: flag the line
      if (rx_valid && ph_q != PH_IDLE && err_q == 8'd0) err_q <= 8'(ERR_SYNTAX);
      if (rx_ferr) ferr_q <= 1'b1;
    end
  end
endmodule
